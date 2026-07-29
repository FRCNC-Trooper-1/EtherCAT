// Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.
//
// Multi-axis coordination: the machine's behaviour, independent of the fieldbus.
//
// Owns every axis, consumes setpoints, publishes status, and decides what
// happens when something goes wrong. The fieldbus plumbing lives in CyclicTask;
// everything here is pure logic so the safety behaviour is testable without a
// drive on the bench.
//
// The rule that shapes the design:
//
//   WHEN ONE AXIS FAULTS, EVERY AXIS STOPS TOGETHER.
//
// Stopping only the faulted axis leaves the others interpolating against a
// stationary one. On a 3-axis machine that drags the tool off the path at full
// feed — it gouges the part, and usually breaks the tool.

#pragma once

#include "frcnc/app/axis_controller.hpp"
#include "frcnc/ipc/messages.hpp"

#include <cstdint>

namespace frcnc::app {

inline constexpr int kMaxAxes = ipc::kMaxAxes;

enum class MachineState : std::uint8_t {
    Idle,        ///< axes disabled, nothing commanded
    Enabling,    ///< walking axes up to Operation Enabled
    Ready,       ///< enabled, holding position, no motion queued
    Running,     ///< consuming setpoints
    Stopping,    ///< coordinated decelerating stop in progress
    Faulted,     ///< stopped and disabled after a fault
    Disabling,   ///< walking axes down on request
};

[[nodiscard]] const char* to_string(MachineState s) noexcept;

/// Why the machine stopped. Records the FIRST cause, so a cascade of secondary
/// faults cannot obscure what actually happened.
struct FaultRecord {
    bool active = false;
    int axis = -1;                            ///< -1 for machine-level causes
    FaultReason reason = FaultReason::None;
    bool bus_fault = false;                   ///< working counter or not operational
    std::uint64_t cycle = 0;
};

struct MachineConfig {
    int axis_count = 3;
    AxisConfig axis[kMaxAxes]{};

    /// Deceleration used for a coordinated stop, in user units/s^2.
    ///
    /// Should be at or above the planner's acceleration limit: a stop that
    /// cannot decelerate as hard as the motion accelerated will overshoot.
    double stop_deceleration = 5000.0;

    /// Cycle period in seconds. Needed to integrate the stop ramp.
    double cycle_time = 0.001;

    /// Consecutive bad working counters tolerated before declaring a bus fault.
    ///
    /// A single dropped frame is recoverable; a run of them is a cable, a slave,
    /// or a timing problem. Zero means fault on the first.
    std::uint32_t wkc_error_tolerance = 2;

    /// Cycles allowed for every axis to reach Operation Enabled. 0 disables the
    /// check. An axis that never enables would otherwise sit in Enabling for
    /// ever, with the operator staring at a machine that is not refusing and not
    /// working either.
    std::uint32_t enable_timeout_cycles = 2000;
};

/// What the fieldbus produced this cycle.
///
/// `axis[i].pdo_valid` here means only "this slave is in OPERATIONAL and its
/// data is in the frame". Whether a working-counter miss makes the frame
/// unusable is a MACHINE-level decision — one dropped frame is survivable, a
/// run of them is not — so it is taken once here against
/// MachineConfig::wkc_error_tolerance and applied to every axis, rather than
/// each axis deciding for itself.
struct BusInputs {
    bool operational = false;
    bool wkc_ok = false;
    std::int64_t dc_error_ns = 0;
    bool dc_locked = false;
    int working_counter = 0;
    int expected_wkc = 0;
    std::uint64_t wkc_errors = 0;

    AxisInputs axis[kMaxAxes]{};
};

/// What must be written back into the PDOs.
struct MachineOutputs {
    AxisOutputs axis[kMaxAxes]{};
};

class MachineController {
public:
    void configure(const MachineConfig& cfg) noexcept;

    /// Attach the channels shared with the planner. Any may be null.
    void set_channels(ipc::SetpointQueue* setpoints, ipc::CommandQueue* commands,
                      ipc::StatusChannel* status) noexcept;

    /// Advance one cycle. Real-time path: no allocation, no logging, no locks.
    [[nodiscard]] MachineOutputs update(const BusInputs& in) noexcept;

    // --- requests, usable without the command queue ---

    void request_enable() noexcept;
    void request_disable() noexcept;

    /// Begin a coordinated decelerating stop. Axes stay enabled and holding.
    void request_stop() noexcept;

    /// Clear master-detected faults on every axis and reset the record. Does not
    /// clear a drive-reported fault; only the drive leaving Fault does that.
    void clear_faults() noexcept;

    // --- observation ---

    [[nodiscard]] MachineState state() const noexcept { return state_; }
    [[nodiscard]] const FaultRecord& fault() const noexcept { return fault_; }
    [[nodiscard]] bool enabled() const noexcept;
    [[nodiscard]] bool moving() const noexcept {
        return state_ == MachineState::Running || state_ == MachineState::Stopping;
    }
    [[nodiscard]] bool feed_held() const noexcept { return feed_held_; }

    /// Loop timing measured by the caller, forwarded into published status. The
    /// controller cannot measure its own jitter — it only sees cycle contents.
    void set_timing(std::int64_t jitter_ns, std::int64_t max_jitter_ns) noexcept {
        jitter_ns_ = jitter_ns;
        max_jitter_ns_ = max_jitter_ns;
    }

    [[nodiscard]] const AxisController& axis(int i) const noexcept;
    [[nodiscard]] int axis_count() const noexcept { return cfg_.axis_count; }

    [[nodiscard]] std::uint64_t cycles() const noexcept { return cycles_; }
    [[nodiscard]] std::uint64_t starve_events() const noexcept { return starve_events_; }

    /// Speed along the path, from the last consumed setpoint.
    [[nodiscard]] double path_velocity() const noexcept { return path_velocity_; }

    void reset() noexcept;

private:
    void raise_fault(int axis_index, FaultReason reason, bool bus) noexcept;
    void begin_stop() noexcept;
    void apply_stop_ramp(AxisCommand* cmd) noexcept;
    void flush_setpoints() noexcept;
    void drain_commands() noexcept;
    void enter(MachineState s) noexcept;
    void request_all(drive::Request r) noexcept;
    void seed_hold_from_actual() noexcept;
    void publish_status(const BusInputs& in, bool starved) noexcept;

    /// True while any axis is still in Operation Enabled — i.e. a quick stop is
    /// still running and must not yet be converted into a shutdown.
    [[nodiscard]] bool any_enabled() const noexcept;

    /// True when every axis has reached a de-energised CiA 402 state, or the
    /// bus is down and nothing we write is reaching a drive anyway.
    [[nodiscard]] bool all_down(const BusInputs& in) const noexcept;

    MachineConfig cfg_{};
    AxisController axis_[kMaxAxes]{};

    ipc::SetpointQueue* setpoints_ = nullptr;
    ipc::CommandQueue* commands_ = nullptr;
    ipc::StatusChannel* status_ = nullptr;

    MachineState state_ = MachineState::Idle;
    FaultRecord fault_{};

    // Last commanded motion, held so a stop can decelerate from it rather than
    // freezing the target — freezing is an infinite-deceleration stop.
    double last_position_[kMaxAxes] = {};
    double last_velocity_[kMaxAxes] = {};
    bool have_last_ = false;

    /// Previous cycle's measured position, for differentiating actual velocity.
    double prev_position_[kMaxAxes] = {};

    /// Scales the retained velocity during a stop: 1 at the start, 0 when done.
    /// Every axis is scaled by the SAME factor, which is what makes the stop
    /// coordinated — the tool decelerates along its last direction of travel
    /// instead of each axis independently freezing.
    double stop_scale_ = 0.0;
    double stop_rate_ = 0.0;   ///< decrement of stop_scale_ per cycle
    double stop_speed_ = 0.0;  ///< path speed when the stop began
    double path_velocity_ = 0.0;

    std::int64_t jitter_ns_ = 0;
    std::int64_t max_jitter_ns_ = 0;

    std::uint32_t consecutive_wkc_errors_ = 0;
    std::uint32_t state_cycles_ = 0;
    std::uint64_t cycles_ = 0;
    std::uint64_t starve_events_ = 0;
    std::uint64_t setpoint_sequence_ = 0;
    std::uint32_t block_id_ = 0;
    bool feed_held_ = false;
};

}  // namespace frcnc::app
