// Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.

#include "frcnc/app/axis_controller.hpp"

#include <cmath>

namespace frcnc::app {

namespace {

constexpr double kEps = 1e-12;

constexpr std::int32_t kInt32Max = 2147483647;
constexpr std::int32_t kInt32Min = -2147483647 - 1;

/// Saturate rather than wrap. A double that exceeds INT32 range is a
/// configuration or planner error, and wrapping would command the axis to the
/// far end of its travel at full speed.
std::int32_t saturate_i32(double v) noexcept {
    if (!(v == v)) {  // NaN
        return 0;
    }
    if (v > static_cast<double>(kInt32Max)) {
        return kInt32Max;
    }
    if (v < static_cast<double>(kInt32Min)) {
        return kInt32Min;
    }
    return static_cast<std::int32_t>(v);
}

std::int16_t saturate_i16(double v) noexcept {
    if (!(v == v)) {
        return 0;
    }
    if (v > 32767.0) {
        return 32767;
    }
    if (v < -32768.0) {
        return -32768;
    }
    return static_cast<std::int16_t>(v);
}

}  // namespace

const char* to_string(FaultReason r) noexcept {
    switch (r) {
        case FaultReason::None:           return "None";
        case FaultReason::DriveFault:     return "DriveFault";
        case FaultReason::FollowingError: return "FollowingError";
        case FaultReason::SoftLimitLow:   return "SoftLimitLow";
        case FaultReason::SoftLimitHigh:  return "SoftLimitHigh";
        case FaultReason::NotOperational: return "NotOperational";
        case FaultReason::ModeMismatch:   return "ModeMismatch";
        case FaultReason::EnableTimeout:  return "EnableTimeout";
    }
    return "?";
}

void AxisController::configure(const AxisConfig& cfg) noexcept {
    cfg_ = cfg;
    if (std::fabs(cfg_.counts_per_unit) < kEps) {
        cfg_.counts_per_unit = 1.0;  // never divide by zero downstream
    }
    reset();
}

void AxisController::reset() noexcept {
    sm_.reset();
    position_ = 0.0;
    commanded_ = 0.0;
    following_error_ = 0.0;
    fault_ = FaultReason::None;
    cycles_ = 0;
}

void AxisController::request(drive::Request r) noexcept {
    sm_.request(r);
}

void AxisController::clear_fault() noexcept {
    // Only master-detected faults are cleared here. A drive-reported fault
    // stays until the drive itself leaves the Fault state, so this cannot be
    // used to paper over one.
    if (fault_ != FaultReason::DriveFault) {
        fault_ = FaultReason::None;
    }
}

std::int32_t AxisController::to_counts(double units) const noexcept {
    const double sign = cfg_.invert ? -1.0 : 1.0;
    return saturate_i32(units * cfg_.counts_per_unit * sign);
}

double AxisController::to_units(std::int32_t counts) const noexcept {
    const double sign = cfg_.invert ? -1.0 : 1.0;
    return (static_cast<double>(counts) / cfg_.counts_per_unit) * sign;
}

AxisOutputs AxisController::update(const AxisInputs& in, const AxisCommand& cmd) noexcept {
    cycles_++;

    AxisOutputs out{};
    out.mode = static_cast<std::int8_t>(cfg_.mode);

    // --- read feedback ------------------------------------------------------

    if (in.pdo_valid) {
        position_ = to_units(in.position_counts);
        following_error_ = to_units(in.following_error_counts);
    }

    // --- fault detection ----------------------------------------------------
    //
    // Order matters: the most fundamental condition wins, so a bus dropout is
    // not reported as a following error caused by the dropout.

    // DriveFault and NotOperational mirror a CONDITION, so they lift when the
    // condition does. FollowingError and the soft limits record an EVENT and
    // latch until clear_fault(): the axis has already moved somewhere it should
    // not have, and jogging back inside the envelope does not undo that.
    //
    // Mirroring is not a weaker guarantee. MachineController latches its own
    // FaultRecord, so a bus dropout still needs acknowledging by an operator —
    // it just does not leave every axis permanently unusable after the frames
    // come back, which would make a cold start impossible to recover from.
    if (!in.pdo_valid) {
        fault_ = FaultReason::NotOperational;
    } else if (drive::sw_fault(in.statusword)) {
        fault_ = FaultReason::DriveFault;
    } else if (fault_ == FaultReason::DriveFault || fault_ == FaultReason::NotOperational) {
        fault_ = FaultReason::None;
    }

    if (fault_ == FaultReason::None && in.pdo_valid) {
        if (cfg_.following_error_limit > kEps &&
            std::fabs(following_error_) > cfg_.following_error_limit) {
            fault_ = FaultReason::FollowingError;
        }
    }

    // Soft limits are checked against MEASURED position, not the command. A
    // command inside the envelope does not prove the axis is.
    if (fault_ == FaultReason::None && in.pdo_valid &&
        cfg_.soft_limit_max > cfg_.soft_limit_min + kEps) {
        if (position_ < cfg_.soft_limit_min) {
            fault_ = FaultReason::SoftLimitLow;
        } else if (position_ > cfg_.soft_limit_max) {
            fault_ = FaultReason::SoftLimitHigh;
        }
    }

    // A master-detected fault must not leave the axis energised and following.
    //
    // WHICH stop depends on whether the axis can still execute one. While the
    // drive is in Operation Enabled it can run its own quick-stop ramp (0x605A),
    // which is a controlled deceleration. Once it is not — or once the PDOs
    // stopped being valid, so nothing we write is being read — the only honest
    // request is to walk the axis down. The condition is re-evaluated every
    // cycle, so a quick stop turns into a disable as soon as it completes.
    if (fault_ != FaultReason::None && fault_ != FaultReason::DriveFault) {
        const bool can_ramp = in.pdo_valid && fault_ != FaultReason::NotOperational &&
                              drive::decode_state(in.statusword) == drive::State::OperationEnabled;
        sm_.request(can_ramp ? drive::Request::QuickStop : drive::Request::Disable);
    }

    // --- drive state machine ------------------------------------------------

    const drive::StateMachine::Step step = sm_.update(in.statusword);
    out.controlword = step.controlword;

    // --- setpoint -----------------------------------------------------------
    //
    // THE safety rule. While the servo loop is not closed, the target must
    // track the measured position exactly, so that the instant torque is
    // applied there is no position step. A stale target at enable makes the
    // drive slam to it at maximum acceleration.
    //
    // The same applies when the planner starves: hold where we are rather than
    // extrapolating, because an extrapolated setpoint is a guess about a
    // machine that is cutting metal.
    if (step.hold_target_at_actual || !cmd.valid) {
        commanded_ = position_;
        out.target_counts = in.position_counts;
        out.velocity_offset = 0;
        out.torque_offset = 0;
        return out;
    }

    // Clamp the command into the soft-limit envelope. The planner enforces this
    // too; doing it here as well means a planner bug cannot drive into a stop.
    double target = cmd.position;
    if (cfg_.soft_limit_max > cfg_.soft_limit_min + kEps) {
        if (target < cfg_.soft_limit_min) {
            target = cfg_.soft_limit_min;
        } else if (target > cfg_.soft_limit_max) {
            target = cfg_.soft_limit_max;
        }
    }

    commanded_ = target;
    out.target_counts = to_counts(target);

    // Feedforward. Velocity offset is in the drive's velocity unit, which for a
    // CSP drive is counts/s; torque offset is left to the caller's scaling and
    // passed through as supplied.
    const double sign = cfg_.invert ? -1.0 : 1.0;
    out.velocity_offset = saturate_i32(cmd.velocity * cfg_.counts_per_unit * sign);
    out.torque_offset = saturate_i16(cmd.acceleration * sign);

    return out;
}

}  // namespace frcnc::app
