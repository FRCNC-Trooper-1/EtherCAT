// Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.
//
// CiA 402 (IEC 61800-7-201) drive state machine.
//
// Pure logic: statusword in, controlword out. No I/O, no allocation, no
// exceptions, no dependency on the fieldbus. That makes it exhaustively
// testable without hardware, which matters because getting this wrong on a
// real machine means a servo slamming to a stale setpoint.
//
// Reference: docs/04-drive-cia402.md

#pragma once

#include <cstdint>

namespace frcnc::drive {

/// Drive states, decoded from the statusword (0x6041).
enum class State : std::uint8_t {
    NotReadyToSwitchOn,
    SwitchOnDisabled,
    ReadyToSwitchOn,
    SwitchedOn,           ///< Power stage live, but NO torque.
    OperationEnabled,     ///< Servo loop closed, following commands.
    QuickStopActive,
    FaultReactionActive,
    Fault,
    Unknown,              ///< Statusword matched no known pattern.
};

/// Modes of operation (0x6060 / 0x6061).
enum class Mode : std::int8_t {
    NoMode = 0,
    ProfilePosition = 1,
    Velocity = 2,
    ProfileVelocity = 3,
    ProfileTorque = 4,
    Homing = 6,
    InterpolatedPosition = 7,
    CyclicSyncPosition = 8,   ///< CSP — what a CNC uses.
    CyclicSyncVelocity = 9,
    CyclicSyncTorque = 10,
};

/// Controlword (0x6040) command values.
namespace cw {
inline constexpr std::uint16_t Shutdown = 0x0006;
inline constexpr std::uint16_t SwitchOn = 0x0007;
inline constexpr std::uint16_t EnableOperation = 0x000F;
inline constexpr std::uint16_t DisableVoltage = 0x0000;
inline constexpr std::uint16_t QuickStop = 0x0002;
inline constexpr std::uint16_t DisableOperation = 0x0007;  ///< same encoding as SwitchOn
inline constexpr std::uint16_t FaultReset = 0x0080;        ///< RISING EDGE on bit 7
inline constexpr std::uint16_t HaltBit = 0x0100;
}  // namespace cw

/// Decode the statusword into a state.
///
/// Test order matters and is not obvious. `0x4F` tests bits 0,1,2,3,6 — bit 5
/// is don't-care. `0x6F` additionally requires bit 5 set. Because
/// `0x0F & 0x4F == 0x0F`, FaultReactionActive must be tested BEFORE Fault or it
/// is unreachable. Bit 4 (voltage enabled) is never part of any state test.
[[nodiscard]] State decode_state(std::uint16_t statusword) noexcept;

[[nodiscard]] const char* to_string(State s) noexcept;
[[nodiscard]] const char* to_string(Mode m) noexcept;

// --- Statusword bit accessors -----------------------------------------------
// Bits 12 and 13 are mode-specific; the CSP and Homing meanings differ.

[[nodiscard]] bool sw_fault(std::uint16_t sw) noexcept;             ///< bit 3
[[nodiscard]] bool sw_voltage_enabled(std::uint16_t sw) noexcept;   ///< bit 4
[[nodiscard]] bool sw_warning(std::uint16_t sw) noexcept;           ///< bit 7
[[nodiscard]] bool sw_remote(std::uint16_t sw) noexcept;            ///< bit 9
[[nodiscard]] bool sw_target_reached(std::uint16_t sw) noexcept;    ///< bit 10
[[nodiscard]] bool sw_internal_limit(std::uint16_t sw) noexcept;    ///< bit 11

[[nodiscard]] bool sw_csp_follows_command(std::uint16_t sw) noexcept;  ///< bit 12, CSP
[[nodiscard]] bool sw_csp_following_error(std::uint16_t sw) noexcept;  ///< bit 13, CSP
[[nodiscard]] bool sw_homing_attained(std::uint16_t sw) noexcept;      ///< bit 12, HM
[[nodiscard]] bool sw_homing_error(std::uint16_t sw) noexcept;         ///< bit 13, HM

/// What the application wants the drive to be doing.
enum class Request : std::uint8_t {
    Disable,    ///< Walk down to SwitchOnDisabled. The safe default.
    Ready,      ///< Hold at SwitchedOn — powered, no torque.
    Enable,     ///< Drive to OperationEnabled.
    QuickStop,  ///< Controlled stop via the drive's quick-stop ramp.
};

/// Drives one axis through the CiA 402 state machine.
///
/// Call update() exactly once per cycle with the statusword just received, and
/// write the returned controlword into the outgoing PDO. The object is a plain
/// value type — no allocation, no virtuals, safe in the cyclic path.
class StateMachine {
public:
    struct Step {
        /// Controlword to write into the RxPDO this cycle.
        std::uint16_t controlword;

        /// When true, the caller MUST set target position (0x607A) equal to
        /// actual position (0x6064) this cycle.
        ///
        /// This is the single most safety-relevant output of this class. While
        /// the drive is not in OperationEnabled, the target must track actual
        /// continuously — so that at the instant torque is applied there is no
        /// position step. A stale target at enable makes the drive slam to it
        /// at maximum acceleration. See docs/04-drive-cia402.md §7.
        bool hold_target_at_actual;
    };

    /// Set the desired posture. Safe to call every cycle.
    void request(Request r) noexcept { request_ = r; }

    [[nodiscard]] Request request() const noexcept { return request_; }

    /// Advance one cycle. Returns what to write this cycle.
    [[nodiscard]] Step update(std::uint16_t statusword) noexcept;

    [[nodiscard]] State state() const noexcept { return state_; }

    /// True only in OperationEnabled — i.e. motion commands will be followed.
    [[nodiscard]] bool enabled() const noexcept { return state_ == State::OperationEnabled; }

    /// True in Fault or FaultReactionActive.
    [[nodiscard]] bool faulted() const noexcept;

    /// Number of observed state changes. Useful for diagnostics; a drive that
    /// keeps transitioning is a drive that keeps dropping out.
    [[nodiscard]] std::uint32_t transitions() const noexcept { return transitions_; }

    /// Cycles spent in the current state.
    [[nodiscard]] std::uint32_t dwell_cycles() const noexcept { return dwell_; }

    void reset() noexcept;

private:
    Request request_ = Request::Disable;
    State state_ = State::Unknown;
    std::uint16_t last_controlword_ = 0;
    std::uint32_t transitions_ = 0;
    std::uint32_t dwell_ = 0;
};

}  // namespace frcnc::drive
