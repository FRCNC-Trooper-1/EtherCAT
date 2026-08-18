// Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.
//
// Per-axis control: user units in, drive counts out.
//
// Sits between the motion planner, which thinks in millimetres, and the PDO
// image, which holds encoder counts. It owns the CiA 402 state machine for one
// axis, converts units, enforces limits, and decides when the axis has faulted.
//
// Pure logic — no fieldbus, no allocation, no exceptions — so every safety rule
// in it is testable without a drive on the bench. That matters more here than
// anywhere else in the codebase: these are the rules that decide whether a
// servo slams to a stale setpoint.

#pragma once

#include "frcnc/drive/cia402.hpp"

#include <cstdint>

namespace frcnc::app {

/// Why an axis stopped being usable.
enum class FaultReason : std::uint8_t {
    None,
    DriveFault,       ///< statusword bit 3
    FollowingError,   ///< measured error beyond the configured limit
    SoftLimitLow,
    SoftLimitHigh,
    NotOperational,   ///< PDO data not valid this cycle
    ModeMismatch,     ///< drive is not in the mode we asked for
    EnableTimeout,    ///< did not reach Operation Enabled in the allowed time
};

[[nodiscard]] const char* to_string(FaultReason r) noexcept;

struct AxisConfig {
    /// Encoder counts per user unit (per millimetre).
    ///
    /// Do the conversion here rather than in the drive: vendors differ on
    /// whether their scaling objects actually apply to 0x607A and 0x6064, and a
    /// factor applied in two places is a factor applied twice. See
    /// docs/05-motion-architecture.md §5 and docs/06-vendor-notes.md.
    double counts_per_unit = 1.0;

    /// Reverse the direction of travel relative to the drive's own sense.
    bool invert = false;

    /// Travel limits in user units. Equal values disable the check.
    double soft_limit_min = 0.0;
    double soft_limit_max = 0.0;

    /// Following error limit in user units. 0 disables the check.
    ///
    /// This is the master-side supervisor. The drive has its own via 0x6065,
    /// but the master must react too: when one axis reports following error,
    /// EVERY axis has to stop, and only the master can coordinate that.
    double following_error_limit = 0.0;

    /// Mode to command. CSP for coordinated motion.
    drive::Mode mode = drive::Mode::CyclicSyncPosition;

    /// 0x6072 max torque, per mille of rated torque, written every cycle
    /// whenever the drive maps it.
    ///
    /// Defaults to 100% of rated rather than to zero or to the drive's own
    /// power-on value, and the reason is a failure this project hit on the
    /// bench: a richer PDO mapping (Yaskawa 0x1600) carries 0x6072, the
    /// process image starts zeroed, and an unwritten entry is therefore a
    /// commanded torque limit of ZERO. The axis enables, sets "internal limit
    /// active", and refuses to move.
    ///
    /// 100% is the conservative default in the other direction too: Yaskawa
    /// drives power up allowing several times rated torque, so writing this
    /// LOWERS the limit. Raise it deliberately, per machine.
    ///
    /// There is no "leave the object alone" setting, because once 0x6072 is in
    /// the RxPDO there is no such thing: the process image is what the drive
    /// reads, every cycle, and not writing it means writing zero. CyclicTask
    /// refuses to start on a mapped 0x6072 with this left at 0.
    std::uint16_t max_torque_per_mille = 1000;
};

/// What the cyclic task read out of this axis's TxPDO.
struct AxisInputs {
    std::uint16_t statusword = 0;
    std::int32_t position_counts = 0;
    std::int32_t following_error_counts = 0;
    std::int8_t mode_display = 0;

    /// False when process data was not valid this cycle — a working counter
    /// mismatch, or the slave having left OPERATIONAL.
    bool pdo_valid = false;
};

/// What the cyclic task must write into this axis's RxPDO.
struct AxisOutputs {
    std::uint16_t controlword = 0;
    std::int32_t target_counts = 0;
    std::int32_t velocity_offset = 0;  ///< 0x60B1, counts/s
    std::int16_t torque_offset = 0;    ///< 0x60B2, per mille of rated torque
    std::uint16_t max_torque = 0;      ///< 0x6072, per mille; 0 means do not write
    std::int8_t mode = 0;
};

/// The motion command for this cycle, in user units.
struct AxisCommand {
    double position = 0.0;
    double velocity = 0.0;
    double acceleration = 0.0;

    /// False when the planner produced nothing this cycle. The axis holds
    /// position rather than following a stale or extrapolated setpoint.
    bool valid = false;
};

class AxisController {
public:
    void configure(const AxisConfig& cfg) noexcept;

    /// Ask the axis to enable, disable, hold, or quick-stop.
    void request(drive::Request r) noexcept;
    [[nodiscard]] drive::Request request() const noexcept { return sm_.request(); }

    /// Advance one cycle.
    [[nodiscard]] AxisOutputs update(const AxisInputs& in, const AxisCommand& cmd) noexcept;

    // --- observation ---

    [[nodiscard]] double position() const noexcept { return position_; }
    [[nodiscard]] double commanded_position() const noexcept { return commanded_; }
    [[nodiscard]] double following_error() const noexcept { return following_error_; }

    [[nodiscard]] drive::State state() const noexcept { return sm_.state(); }
    [[nodiscard]] bool enabled() const noexcept { return sm_.enabled(); }

    [[nodiscard]] bool faulted() const noexcept { return fault_ != FaultReason::None; }
    [[nodiscard]] FaultReason fault() const noexcept { return fault_; }

    /// Clear a master-detected fault. Drive-reported faults clear only when the
    /// drive itself leaves the Fault state, so this cannot mask one.
    void clear_fault() noexcept;

    [[nodiscard]] std::uint64_t cycles() const noexcept { return cycles_; }

    void reset() noexcept;

    // --- unit conversion, exposed for configuration and test ---

    [[nodiscard]] std::int32_t to_counts(double units) const noexcept;
    [[nodiscard]] double to_units(std::int32_t counts) const noexcept;

private:
    AxisConfig cfg_{};
    drive::StateMachine sm_{};

    double position_ = 0.0;
    double commanded_ = 0.0;
    double following_error_ = 0.0;

    FaultReason fault_ = FaultReason::None;
    std::uint64_t cycles_ = 0;
};

}  // namespace frcnc::app
