// Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.

#include "frcnc/app/machine_controller.hpp"

#include <cmath>

namespace frcnc::app {

namespace {

constexpr double kEps = 1e-12;

/// Commands drained per cycle. Bounded so a flooded command queue cannot
/// stretch a cycle: the queue is 16 deep and an operator cannot press stop
/// eight times in one millisecond, so this only ever bounds a misbehaving
/// producer.
constexpr int kMaxCommandsPerCycle = 8;

}  // namespace

const char* to_string(MachineState s) noexcept {
    switch (s) {
        case MachineState::Idle:      return "Idle";
        case MachineState::Enabling:  return "Enabling";
        case MachineState::Ready:     return "Ready";
        case MachineState::Running:   return "Running";
        case MachineState::Stopping:  return "Stopping";
        case MachineState::Faulted:   return "Faulted";
        case MachineState::Disabling: return "Disabling";
    }
    return "?";
}

// --- setup ------------------------------------------------------------------

void MachineController::configure(const MachineConfig& cfg) noexcept {
    cfg_ = cfg;

    if (cfg_.axis_count < 0) {
        cfg_.axis_count = 0;
    } else if (cfg_.axis_count > kMaxAxes) {
        cfg_.axis_count = kMaxAxes;
    }
    if (!(cfg_.cycle_time > kEps)) {
        cfg_.cycle_time = 0.001;
    }
    if (!(cfg_.stop_deceleration > kEps)) {
        cfg_.stop_deceleration = 1.0;
    }

    for (int i = 0; i < cfg_.axis_count; i++) {
        axis_[i].configure(cfg_.axis[i]);
    }
    reset();
}

void MachineController::set_channels(ipc::SetpointQueue* setpoints,
                                     ipc::CommandQueue* commands,
                                     ipc::StatusChannel* status) noexcept {
    setpoints_ = setpoints;
    commands_ = commands;
    status_ = status;
}

void MachineController::reset() noexcept {
    for (int i = 0; i < cfg_.axis_count; i++) {
        axis_[i].reset();
    }
    state_ = MachineState::Idle;
    fault_ = FaultRecord{};

    for (int i = 0; i < kMaxAxes; i++) {
        last_position_[i] = 0.0;
        last_velocity_[i] = 0.0;
        prev_position_[i] = 0.0;
    }
    have_last_ = false;

    stop_scale_ = 0.0;
    stop_rate_ = 0.0;
    stop_speed_ = 0.0;
    path_velocity_ = 0.0;

    consecutive_wkc_errors_ = 0;
    state_cycles_ = 0;
    cycles_ = 0;
    starve_events_ = 0;
    setpoint_sequence_ = 0;
    block_id_ = 0;
    feed_held_ = false;
}

const AxisController& MachineController::axis(int i) const noexcept {
    if (i < 0) {
        i = 0;
    } else if (i >= kMaxAxes) {
        i = kMaxAxes - 1;
    }
    return axis_[i];
}

bool MachineController::enabled() const noexcept {
    if (cfg_.axis_count <= 0) {
        return false;
    }
    for (int i = 0; i < cfg_.axis_count; i++) {
        if (!axis_[i].enabled()) {
            return false;
        }
    }
    return true;
}

bool MachineController::any_enabled() const noexcept {
    for (int i = 0; i < cfg_.axis_count; i++) {
        if (axis_[i].enabled()) {
            return true;
        }
    }
    return false;
}

bool MachineController::all_down(const BusInputs& in) const noexcept {
    if (!in.operational) {
        return true;
    }
    for (int i = 0; i < cfg_.axis_count; i++) {
        const drive::State s = axis_[i].state();
        if (s != drive::State::SwitchOnDisabled && s != drive::State::NotReadyToSwitchOn) {
            return false;
        }
    }
    return true;
}

// --- internal helpers -------------------------------------------------------

void MachineController::enter(MachineState s) noexcept {
    if (state_ != s) {
        state_ = s;
        state_cycles_ = 0;
    }
}

void MachineController::request_all(drive::Request r) noexcept {
    for (int i = 0; i < cfg_.axis_count; i++) {
        axis_[i].request(r);
    }
}

void MachineController::seed_hold_from_actual() noexcept {
    // The hold target has to start from where the machine actually IS. Coming
    // out of Enabling with a stale (or zero) hold position would command a jump
    // the instant the axes start following.
    for (int i = 0; i < cfg_.axis_count; i++) {
        last_position_[i] = axis_[i].position();
        last_velocity_[i] = 0.0;
    }
    have_last_ = true;
    path_velocity_ = 0.0;
}

void MachineController::raise_fault(int axis_index, FaultReason reason, bool bus) noexcept {
    if (!fault_.active) {
        // Record the FIRST cause only. A fault cascades — a following error
        // trips a bus error trips a not-operational — and the last one to
        // arrive is never the one that explains anything.
        fault_.active = true;
        fault_.axis = axis_index;
        fault_.reason = reason;
        fault_.bus_fault = bus;
        fault_.cycle = cycles_;
    }

    // EVERY axis stops, not just the offender. Quick stop rather than the
    // master-side ramp used by request_stop(): a coordinated ramp needs every
    // axis to still be following commands, and a fault means at least one is
    // not. Quick stop is executed by the drive itself on its 0x605A ramp, so it
    // still works for the axis that stopped listening to us.
    request_all(drive::Request::QuickStop);
    flush_setpoints();

    stop_scale_ = 0.0;
    path_velocity_ = 0.0;
    for (int i = 0; i < cfg_.axis_count; i++) {
        last_velocity_[i] = 0.0;
    }

    enter(MachineState::Faulted);
}

void MachineController::flush_setpoints() noexcept {
    if (setpoints_ == nullptr) {
        return;
    }
    // Bounded by the ring depth, so this is a fixed handful of nanoseconds and
    // safe in the cyclic path.
    ipc::AxisSetpoint discard;
    for (std::size_t n = 0; n < ipc::kSetpointQueueDepth && setpoints_->pop(discard); n++) {
    }
}

void MachineController::begin_stop() noexcept {
    // Every queued setpoint describes the path we are about to leave. Keeping
    // them would mean the machine resumes the old move from wherever the stop
    // ended — a jump of up to the whole stopping distance, at full feed.
    //
    // The planner re-plans from the position reported in status. It must see
    // motion_active go false before it pushes again; that handshake is the
    // contract, not something this side can enforce.
    flush_setpoints();

    double v2 = 0.0;
    for (int i = 0; i < cfg_.axis_count; i++) {
        v2 += last_velocity_[i] * last_velocity_[i];
    }
    stop_speed_ = std::sqrt(v2);

    if (!have_last_ || stop_speed_ < kEps) {
        // Already stopped. Holding the last commanded position IS the stop.
        stop_scale_ = 0.0;
        stop_rate_ = 0.0;
        path_velocity_ = 0.0;
        for (int i = 0; i < cfg_.axis_count; i++) {
            last_velocity_[i] = 0.0;
        }
        enter(MachineState::Ready);
        return;
    }

    // Deceleration is applied to the PATH, and each axis is scaled by the same
    // factor, so the direction of travel is preserved all the way to zero.
    //
    //   ramp time  = speed / decel
    //   ramp cycles = ramp time / dt
    //   per-cycle decrement of the scale = 1 / ramp cycles
    stop_scale_ = 1.0;
    stop_rate_ = cfg_.stop_deceleration * cfg_.cycle_time / stop_speed_;
    if (stop_rate_ > 1.0) {
        stop_rate_ = 1.0;  // stops within a single cycle
    }
    path_velocity_ = stop_speed_;
    enter(MachineState::Stopping);
}

void MachineController::apply_stop_ramp(AxisCommand* cmd) noexcept {
    // Anything the planner still had queued describes a path we are no longer
    // on. Drop it every cycle of the ramp, so a straggler pushed while we were
    // decelerating cannot restart the move from a stale position.
    flush_setpoints();

    double next = stop_scale_ - stop_rate_;
    // Relative epsilon, not an exact compare: ten subtractions of 0.1 do not
    // land on zero in binary floating point, and a ramp that never quite
    // finishes leaves the machine in Stopping for ever.
    if (next < stop_rate_ * 1e-6) {
        next = 0.0;
    }

    // Trapezoidal integration of a linear ramp is exact, so the distance
    // travelled during the stop is exactly speed^2 / (2 * decel) — the same
    // number the planner uses when it decides how far ahead to look.
    const double avg = 0.5 * (stop_scale_ + next);

    for (int i = 0; i < cfg_.axis_count; i++) {
        last_position_[i] += last_velocity_[i] * avg * cfg_.cycle_time;
        cmd[i].position = last_position_[i];
        cmd[i].velocity = last_velocity_[i] * next;
        cmd[i].acceleration = 0.0;
        cmd[i].valid = true;
    }

    stop_scale_ = next;
    path_velocity_ = stop_speed_ * next;

    if (stop_scale_ <= 0.0) {
        for (int i = 0; i < cfg_.axis_count; i++) {
            last_velocity_[i] = 0.0;
        }
        path_velocity_ = 0.0;
        enter(MachineState::Ready);
    }
}

void MachineController::drain_commands() noexcept {
    if (commands_ == nullptr) {
        return;
    }

    ipc::Command c;
    for (int n = 0; n < kMaxCommandsPerCycle && commands_->pop(c); n++) {
        switch (c.kind) {
            case ipc::CommandKind::None:
                break;

            case ipc::CommandKind::Enable:
                request_enable();
                break;

            case ipc::CommandKind::Disable:
                request_disable();
                break;

            case ipc::CommandKind::Stop:
                request_stop();
                break;

            case ipc::CommandKind::EmergencyStop:
                // Not a fault: the operator asked for it, so nothing is latched
                // and nothing needs clearing. Quick stop first, then down.
                request_all(drive::Request::QuickStop);
                flush_setpoints();
                stop_scale_ = 0.0;
                path_velocity_ = 0.0;
                for (int i = 0; i < cfg_.axis_count; i++) {
                    last_velocity_[i] = 0.0;
                }
                enter(MachineState::Disabling);
                break;

            case ipc::CommandKind::ClearFault:
                clear_faults();
                break;

            case ipc::CommandKind::HoldFeed:
                // Feed hold stops the machine on its path and latches. Resuming
                // is the PLANNER's job: the queued setpoints are stale the
                // moment we decelerate away from them, so it must re-plan from
                // the reported position rather than replaying them.
                feed_held_ = true;
                request_stop();
                break;

            case ipc::CommandKind::ResumeFeed:
                feed_held_ = false;
                break;
        }
    }
}

// --- requests ---------------------------------------------------------------

void MachineController::request_enable() noexcept {
    if (state_ == MachineState::Faulted || fault_.active) {
        return;  // clear the fault first; enabling into one is how tools break
    }
    if (state_ == MachineState::Running || state_ == MachineState::Stopping) {
        return;
    }
    request_all(drive::Request::Enable);
    enter(MachineState::Enabling);
}

void MachineController::request_disable() noexcept {
    request_all(drive::Request::Disable);
    flush_setpoints();
    stop_scale_ = 0.0;
    path_velocity_ = 0.0;
    for (int i = 0; i < cfg_.axis_count; i++) {
        last_velocity_[i] = 0.0;
    }
    enter(MachineState::Disabling);
}

void MachineController::request_stop() noexcept {
    if (state_ == MachineState::Running || state_ == MachineState::Stopping) {
        begin_stop();
    }
}

void MachineController::clear_faults() noexcept {
    for (int i = 0; i < cfg_.axis_count; i++) {
        axis_[i].clear_fault();
        // Ready, not Disable: in CiA 402 Fault, the state machine only emits
        // the fault-reset edge when something other than Disable is asked for.
        // Ready leaves the axis at Switched On — powered, no torque — which is
        // the right posture to re-enable from.
        axis_[i].request(drive::Request::Ready);
    }
    fault_ = FaultRecord{};
    consecutive_wkc_errors_ = 0;
    enter(MachineState::Idle);
}

// --- the cycle --------------------------------------------------------------

MachineOutputs MachineController::update(const BusInputs& in) noexcept {
    cycles_++;
    if (state_cycles_ != UINT32_MAX) {
        state_cycles_++;
    }

    drain_commands();

    // --- fieldbus supervision ----------------------------------------------

    if (in.wkc_ok) {
        consecutive_wkc_errors_ = 0;
    } else if (consecutive_wkc_errors_ != UINT32_MAX) {
        consecutive_wkc_errors_++;
    }

    const bool bus_ok =
        in.operational && consecutive_wkc_errors_ <= cfg_.wkc_error_tolerance;

    // Idle tolerates a down bus — that is what the machine looks like before
    // the master has brought the slaves to OPERATIONAL. Anywhere else it is a
    // fault, including while disabling: a stop we cannot transmit has not
    // happened.
    if (!bus_ok && state_ != MachineState::Idle && state_ != MachineState::Faulted) {
        raise_fault(-1, FaultReason::NotOperational, true);
    }

    // --- decide this cycle's commands --------------------------------------

    AxisCommand cmd[kMaxAxes]{};
    bool starved = false;

    switch (state_) {
        case MachineState::Idle:
        case MachineState::Enabling:
            // Not following anything yet. Leaving every command invalid makes
            // AxisController hold target at actual, which is exactly what has
            // to happen right up to the instant torque is applied.
            if (state_ == MachineState::Enabling) {
                if (enabled()) {
                    seed_hold_from_actual();
                    enter(MachineState::Ready);
                } else if (cfg_.enable_timeout_cycles != 0 &&
                           state_cycles_ > cfg_.enable_timeout_cycles) {
                    int culprit = -1;
                    for (int i = 0; i < cfg_.axis_count; i++) {
                        if (!axis_[i].enabled()) {
                            culprit = i;
                            break;
                        }
                    }
                    raise_fault(culprit, FaultReason::EnableTimeout, false);
                }
            }
            break;

        case MachineState::Ready:
            // Hold the last commanded position, not the measured one: holding
            // at actual would let the axis creep by one following error every
            // time the machine pauses.
            if (have_last_) {
                for (int i = 0; i < cfg_.axis_count; i++) {
                    cmd[i].position = last_position_[i];
                    cmd[i].valid = true;
                }
            }
            path_velocity_ = 0.0;

            // Motion starts when the planner has something queued. There is no
            // separate "go" — pushing setpoints IS the go.
            if (!feed_held_ && setpoints_ != nullptr && !setpoints_->empty()) {
                enter(MachineState::Running);
            } else {
                break;
            }
            [[fallthrough]];

        case MachineState::Running: {
            ipc::AxisSetpoint sp;
            if (setpoints_ != nullptr && setpoints_->pop(sp)) {
                setpoint_sequence_ = sp.sequence;
                block_id_ = sp.block_id;

                double v2 = 0.0;
                for (int i = 0; i < cfg_.axis_count; i++) {
                    last_position_[i] = sp.position[i];
                    last_velocity_[i] = sp.velocity[i];
                    v2 += sp.velocity[i] * sp.velocity[i];

                    cmd[i].position = sp.position[i];
                    cmd[i].velocity = sp.velocity[i];
                    cmd[i].acceleration = sp.acceleration[i];
                    cmd[i].valid = true;
                }
                have_last_ = true;
                path_velocity_ = std::sqrt(v2);

                if (sp.end_of_path) {
                    // The planner brought the path to rest itself; no ramp
                    // needed, just stop consuming and hold here.
                    for (int i = 0; i < cfg_.axis_count; i++) {
                        last_velocity_[i] = 0.0;
                    }
                    path_velocity_ = 0.0;
                    enter(MachineState::Ready);
                }
            } else {
                // Starvation. The planner is supposed to keep a 64-deep queue
                // full; if it could not, the honest response is to stop on the
                // path rather than guess where the tool should go next.
                starved = true;
                if (starve_events_ != UINT64_MAX) {
                    starve_events_++;
                }
                begin_stop();
                if (state_ == MachineState::Stopping) {
                    apply_stop_ramp(cmd);
                } else if (have_last_) {
                    for (int i = 0; i < cfg_.axis_count; i++) {
                        cmd[i].position = last_position_[i];
                        cmd[i].valid = true;
                    }
                }
            }
            break;
        }

        case MachineState::Stopping:
            apply_stop_ramp(cmd);
            break;

        case MachineState::Faulted:
        case MachineState::Disabling:
            // Commands stay invalid: the axes hold target at actual while the
            // drives execute their own stop. Once none of them is enabled the
            // quick stop is over, so convert it into a shutdown — leaving a
            // faulted axis sitting powered is not a resting state.
            if (!any_enabled()) {
                request_all(drive::Request::Disable);
            }
            if (state_ == MachineState::Disabling && all_down(in)) {
                enter(MachineState::Idle);
            }
            path_velocity_ = 0.0;
            break;
    }

    // --- drive the axes -----------------------------------------------------

    MachineOutputs out{};
    for (int i = 0; i < cfg_.axis_count; i++) {
        AxisInputs ai = in.axis[i];
        // One authority on whether process data is usable. An axis must not
        // decide for itself that the frame was good when the working counter
        // says the frame was not.
        ai.pdo_valid = ai.pdo_valid && bus_ok;

        out.axis[i] = axis_[i].update(ai, cmd[i]);
    }

    // --- axis faults, after the axes have looked at this cycle's data -------

    if (state_ != MachineState::Faulted && state_ != MachineState::Idle) {
        for (int i = 0; i < cfg_.axis_count; i++) {
            if (axis_[i].faulted()) {
                raise_fault(i, axis_[i].fault(), false);
                break;
            }
        }
    }

    publish_status(in, starved);
    return out;
}

// --- status -----------------------------------------------------------------

void MachineController::publish_status(const BusInputs& in, bool starved) noexcept {
    if (status_ == nullptr) {
        // Still needed for velocity differentiation next cycle.
        for (int i = 0; i < cfg_.axis_count; i++) {
            prev_position_[i] = axis_[i].position();
        }
        return;
    }

    ipc::MachineStatus s{};

    const double inv_dt = 1.0 / cfg_.cycle_time;
    for (int i = 0; i < cfg_.axis_count; i++) {
        const AxisController& a = axis_[i];
        const double p = a.position();

        s.axis[i].position_actual = p;
        s.axis[i].position_command = a.commanded_position();
        s.axis[i].following_error = a.following_error();
        s.axis[i].velocity_actual = (p - prev_position_[i]) * inv_dt;
        s.axis[i].statusword = in.axis[i].statusword;
        s.axis[i].mode_display = in.axis[i].mode_display;
        s.axis[i].enabled = a.enabled();
        s.axis[i].faulted = a.faulted();
        s.axis[i].warning = drive::sw_warning(in.axis[i].statusword);
        s.axis[i].limit_active = drive::sw_internal_limit(in.axis[i].statusword);

        prev_position_[i] = p;
    }

    s.cycle = cycles_;
    s.setpoint_sequence = setpoint_sequence_;
    s.block_id = block_id_;

    s.working_counter = in.working_counter;
    s.expected_wkc = in.expected_wkc;
    s.bus_operational = in.operational;
    s.wkc_ok = in.wkc_ok;
    s.wkc_errors = in.wkc_errors;
    s.consecutive_wkc_errors = consecutive_wkc_errors_;

    s.dc_error_ns = in.dc_error_ns;
    s.dc_locked = in.dc_locked;
    s.cycle_jitter_ns = jitter_ns_;
    s.max_cycle_jitter_ns = max_jitter_ns_;

    s.path_velocity = path_velocity_;
    s.motion_active = moving();
    s.feed_held = feed_held_;
    s.starved = starved;
    s.starve_events = starve_events_;
    s.fault_active = fault_.active;

    status_->store(s);
}

}  // namespace frcnc::app
