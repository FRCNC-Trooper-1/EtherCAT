// Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.

#include "frcnc/drive/cia402.hpp"

namespace frcnc::drive {

namespace {

constexpr std::uint16_t kMask4F = 0x004F;  // bits 0,1,2,3,6   (bit 5 don't care)
constexpr std::uint16_t kMask6F = 0x006F;  // bits 0,1,2,3,5,6 (bit 5 must be 1)

constexpr bool bit(std::uint16_t sw, unsigned n) noexcept {
    return (sw & static_cast<std::uint16_t>(1u << n)) != 0;
}

}  // namespace

State decode_state(std::uint16_t sw) noexcept {
    // ORDER IS LOAD-BEARING.
    //
    // FaultReactionActive (0x4F/0x0F) must precede Fault (0x4F/0x08): the
    // pattern 0x0F satisfies both masks, so testing Fault first makes
    // FaultReactionActive unreachable and the drive appears to sit in Fault
    // while it is still decelerating.
    if ((sw & kMask4F) == 0x0F) {
        return State::FaultReactionActive;
    }
    if ((sw & kMask4F) == 0x08) {
        return State::Fault;
    }

    // QuickStopActive (0x6F/0x07) must precede OperationEnabled-style tests
    // only in the sense that both require bit 5 discrimination; listed here
    // for clarity. 0x27 vs 0x07 differ solely in bit 5 (quick stop, inverted).
    if ((sw & kMask6F) == 0x27) {
        return State::OperationEnabled;
    }
    if ((sw & kMask6F) == 0x23) {
        return State::SwitchedOn;
    }
    if ((sw & kMask6F) == 0x21) {
        return State::ReadyToSwitchOn;
    }
    if ((sw & kMask6F) == 0x07) {
        return State::QuickStopActive;
    }

    // NotReadyToSwitchOn and SwitchOnDisabled are distinguished only by bit 6.
    if ((sw & kMask4F) == 0x40) {
        return State::SwitchOnDisabled;
    }
    if ((sw & kMask4F) == 0x00) {
        return State::NotReadyToSwitchOn;
    }

    return State::Unknown;
}

const char* to_string(State s) noexcept {
    switch (s) {
        case State::NotReadyToSwitchOn:  return "NotReadyToSwitchOn";
        case State::SwitchOnDisabled:    return "SwitchOnDisabled";
        case State::ReadyToSwitchOn:     return "ReadyToSwitchOn";
        case State::SwitchedOn:          return "SwitchedOn";
        case State::OperationEnabled:    return "OperationEnabled";
        case State::QuickStopActive:     return "QuickStopActive";
        case State::FaultReactionActive: return "FaultReactionActive";
        case State::Fault:               return "Fault";
        case State::Unknown:             return "Unknown";
    }
    return "Unknown";
}

const char* to_string(Mode m) noexcept {
    switch (m) {
        case Mode::NoMode:               return "NoMode";
        case Mode::ProfilePosition:      return "ProfilePosition";
        case Mode::Velocity:             return "Velocity";
        case Mode::ProfileVelocity:      return "ProfileVelocity";
        case Mode::ProfileTorque:        return "ProfileTorque";
        case Mode::Homing:               return "Homing";
        case Mode::InterpolatedPosition: return "InterpolatedPosition";
        case Mode::CyclicSyncPosition:   return "CyclicSyncPosition";
        case Mode::CyclicSyncVelocity:   return "CyclicSyncVelocity";
        case Mode::CyclicSyncTorque:     return "CyclicSyncTorque";
    }
    return "Unknown";
}

bool sw_fault(std::uint16_t sw) noexcept { return bit(sw, 3); }

bool sw_voltage_enabled(std::uint16_t sw) noexcept { return bit(sw, 4); }

bool sw_warning(std::uint16_t sw) noexcept { return bit(sw, 7); }

bool sw_remote(std::uint16_t sw) noexcept { return bit(sw, 9); }

bool sw_target_reached(std::uint16_t sw) noexcept { return bit(sw, 10); }

bool sw_internal_limit(std::uint16_t sw) noexcept { return bit(sw, 11); }

bool sw_csp_follows_command(std::uint16_t sw) noexcept { return bit(sw, 12); }

bool sw_csp_following_error(std::uint16_t sw) noexcept { return bit(sw, 13); }

bool sw_homing_attained(std::uint16_t sw) noexcept { return bit(sw, 12); }

bool sw_homing_error(std::uint16_t sw) noexcept { return bit(sw, 13); }

bool StateMachine::faulted() const noexcept {
    return state_ == State::Fault || state_ == State::FaultReactionActive;
}

void StateMachine::reset() noexcept {
    request_ = Request::Disable;
    state_ = State::Unknown;
    last_controlword_ = 0;
    transitions_ = 0;
    dwell_ = 0;
}

StateMachine::Step StateMachine::update(std::uint16_t statusword) noexcept {
    const State observed = decode_state(statusword);

    if (observed != state_) {
        state_ = observed;
        transitions_++;
        dwell_ = 0;
    } else if (dwell_ != UINT32_MAX) {
        dwell_++;
    }

    std::uint16_t out = cw::DisableVoltage;

    switch (state_) {
        case State::Fault:
            // Fault reset is EDGE triggered on bit 7. Holding 0x0080 does not
            // retry -- bit 7 must go low between attempts, so alternate.
            if (request_ == Request::Disable) {
                out = cw::DisableVoltage;
            } else if ((last_controlword_ & cw::FaultReset) != 0) {
                out = cw::Shutdown;  // drop bit 7, arming the next rising edge
            } else {
                out = cw::FaultReset;
            }
            break;

        case State::FaultReactionActive:
            // The drive is executing its own fault ramp. Do not interfere;
            // it moves to Fault on its own.
            out = cw::DisableVoltage;
            break;

        case State::SwitchOnDisabled:
            out = (request_ == Request::Disable) ? cw::DisableVoltage : cw::Shutdown;
            break;

        case State::ReadyToSwitchOn:
            out = (request_ == Request::Disable) ? cw::DisableVoltage : cw::SwitchOn;
            break;

        case State::SwitchedOn:
            if (request_ == Request::Enable) {
                out = cw::EnableOperation;
            } else if (request_ == Request::Disable) {
                out = cw::Shutdown;
            } else {
                out = cw::SwitchOn;  // hold here for Ready / QuickStop
            }
            break;

        case State::OperationEnabled:
            switch (request_) {
                case Request::Enable:    out = cw::EnableOperation;  break;
                case Request::QuickStop: out = cw::QuickStop;        break;
                case Request::Ready:     out = cw::DisableOperation; break;
                case Request::Disable:   out = cw::Shutdown;         break;
            }
            break;

        case State::QuickStopActive:
            // Recovery to OperationEnabled is only permitted when the drive's
            // quick stop option code (0x605A) is 5-8; otherwise it drops to
            // SwitchOnDisabled on its own and we pick up from there.
            out = (request_ == Request::Enable) ? cw::EnableOperation : cw::DisableVoltage;
            break;

        case State::NotReadyToSwitchOn:
        case State::Unknown:
            out = cw::DisableVoltage;
            break;
    }

    last_controlword_ = out;

    // Target must track actual whenever the servo loop is not closed, so that
    // enabling never produces a position step.
    return Step{out, state_ != State::OperationEnabled};
}

}  // namespace frcnc::drive
