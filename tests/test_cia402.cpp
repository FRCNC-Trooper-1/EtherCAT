// Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.
//
// Exhaustive tests for the CiA 402 state machine. No hardware, no network.
//
// The decode table is tested against every documented mask/value pair AND
// against the whole 16-bit statusword space, because the masks overlap and an
// ordering mistake is silent.

#include "frcnc/drive/cia402.hpp"

#include <cstdio>
#include <cstdint>

namespace {

int g_failures = 0;
int g_checks = 0;

void check(bool cond, const char* expr, const char* file, int line) {
    g_checks++;
    if (!cond) {
        g_failures++;
        std::printf("  FAIL %s:%d  %s\n", file, line, expr);
    }
}

#define CHECK(expr) check((expr), #expr, __FILE__, __LINE__)

using namespace frcnc::drive;

// --- decode -----------------------------------------------------------------

void test_decode_canonical_patterns() {
    CHECK(decode_state(0x0000) == State::NotReadyToSwitchOn);
    CHECK(decode_state(0x0040) == State::SwitchOnDisabled);
    CHECK(decode_state(0x0021) == State::ReadyToSwitchOn);
    CHECK(decode_state(0x0023) == State::SwitchedOn);
    CHECK(decode_state(0x0027) == State::OperationEnabled);
    CHECK(decode_state(0x0007) == State::QuickStopActive);
    CHECK(decode_state(0x000F) == State::FaultReactionActive);
    CHECK(decode_state(0x0008) == State::Fault);
}

void test_decode_ignores_dont_care_bits() {
    // Bit 4 (voltage enabled) is never part of a state test.
    CHECK(decode_state(0x0027 | 0x0010) == State::OperationEnabled);
    CHECK(decode_state(0x0008 | 0x0010) == State::Fault);

    // Bits 7-15 are never part of a state test: warning, remote, target
    // reached, internal limit, mode-specific, manufacturer-specific.
    CHECK(decode_state(0x0027 | 0xFF80) == State::OperationEnabled);
    CHECK(decode_state(0x0040 | 0xFF80) == State::SwitchOnDisabled);
    CHECK(decode_state(0x0008 | 0xFF80) == State::Fault);

    // Bit 5 IS don't-care for the 0x4F states.
    CHECK(decode_state(0x0008 | 0x0020) == State::Fault);
    CHECK(decode_state(0x000F | 0x0020) == State::FaultReactionActive);
    CHECK(decode_state(0x0040 | 0x0020) == State::SwitchOnDisabled);
}

void test_decode_bit5_discriminates_quickstop() {
    // OperationEnabled and QuickStopActive differ ONLY in bit 5, which is
    // inverted logic: 1 = quick stop not active.
    CHECK(decode_state(0x0027) == State::OperationEnabled);   // bit5 set
    CHECK(decode_state(0x0007) == State::QuickStopActive);    // bit5 clear
    CHECK(decode_state(0x0023) == State::SwitchedOn);         // bit5 set
    CHECK(decode_state(0x0021) == State::ReadyToSwitchOn);    // bit5 set
}

void test_decode_ordering_trap() {
    // THE trap: 0x0F & 0x4F == 0x0F, and 0x08 & 0x4F == 0x08. If Fault were
    // tested with a looser comparison first, FaultReactionActive would be
    // unreachable. Verify both are distinguishable in both bit-5 positions.
    CHECK(decode_state(0x000F) == State::FaultReactionActive);
    CHECK(decode_state(0x002F) == State::FaultReactionActive);
    CHECK(decode_state(0x0008) == State::Fault);
    CHECK(decode_state(0x0028) == State::Fault);

    // And that a fault-reaction pattern is never reported as plain Fault.
    for (std::uint32_t high = 0; high <= 0xFF; high++) {
        const auto sw = static_cast<std::uint16_t>((high << 8) | 0x0F);
        CHECK(decode_state(sw) == State::FaultReactionActive);
    }
}

void test_decode_exhaustive_consistency() {
    // Sweep the entire 16-bit space. Every statusword must decode to exactly
    // the state implied by the documented masks, and decoding must be a pure
    // function of bits 0-3, 5, 6.
    int mismatches = 0;
    for (std::uint32_t v = 0; v <= 0xFFFF; v++) {
        const auto sw = static_cast<std::uint16_t>(v);
        const State s = decode_state(sw);

        // Decoding must depend only on bits 0,1,2,3,5,6.
        const auto reduced = static_cast<std::uint16_t>(sw & 0x006F);
        if (decode_state(reduced) != s) {
            mismatches++;
        }

        // Every fault pattern must be reported as a fault state.
        if ((sw & 0x004F) == 0x08 && s != State::Fault) {
            mismatches++;
        }
        if ((sw & 0x004F) == 0x0F && s != State::FaultReactionActive) {
            mismatches++;
        }
    }
    CHECK(mismatches == 0);
}

void test_decode_unknown_is_reachable() {
    // 0x0001 sets bit 0 only: matches no documented pattern.
    CHECK(decode_state(0x0001) == State::Unknown);
    CHECK(decode_state(0x0002) == State::Unknown);
}

// --- statusword bits --------------------------------------------------------

void test_statusword_bits() {
    CHECK(sw_fault(0x0008));
    CHECK(!sw_fault(0x0027));
    CHECK(sw_voltage_enabled(0x0010));
    CHECK(sw_warning(0x0080));
    CHECK(sw_remote(0x0200));
    CHECK(sw_target_reached(0x0400));
    CHECK(sw_internal_limit(0x0800));
    CHECK(sw_csp_follows_command(0x1000));
    CHECK(sw_csp_following_error(0x2000));
    // Bits 12/13 are mode-specific and alias between CSP and Homing.
    CHECK(sw_homing_attained(0x1000));
    CHECK(sw_homing_error(0x2000));
}

// --- state machine ----------------------------------------------------------

/// Minimal drive model: applies the controlword and returns the new statusword.
/// Enough to exercise transitions without hardware.
std::uint16_t simulate(std::uint16_t state_sw, std::uint16_t controlword) {
    const State s = decode_state(state_sw);
    switch (s) {
        case State::SwitchOnDisabled:
            if ((controlword & 0x0087) == 0x0006) return 0x0021;  // -> ReadyToSwitchOn
            return 0x0040;
        case State::ReadyToSwitchOn:
            if ((controlword & 0x008F) == 0x0007) return 0x0023;  // -> SwitchedOn
            if ((controlword & 0x0082) == 0x0000) return 0x0040;  // -> SwitchOnDisabled
            return 0x0021;
        case State::SwitchedOn:
            if ((controlword & 0x008F) == 0x000F) return 0x0027;  // -> OperationEnabled
            if ((controlword & 0x0087) == 0x0006) return 0x0021;  // -> ReadyToSwitchOn
            return 0x0023;
        case State::OperationEnabled:
            if ((controlword & 0x008F) == 0x0007) return 0x0023;  // -> SwitchedOn
            if ((controlword & 0x0087) == 0x0006) return 0x0021;  // -> ReadyToSwitchOn
            if ((controlword & 0x0086) == 0x0002) return 0x0007;  // -> QuickStopActive
            return 0x0027;
        case State::Fault:
            // Rising edge on bit 7 only.
            return 0x0008;
        default:
            return state_sw;
    }
}

void test_enable_sequence_reaches_operation_enabled() {
    StateMachine sm;
    sm.request(Request::Enable);

    std::uint16_t sw = 0x0040;  // SwitchOnDisabled
    bool reached = false;

    for (int i = 0; i < 20; i++) {
        const auto step = sm.update(sw);
        if (sm.state() == State::OperationEnabled) {
            reached = true;
            break;
        }
        sw = simulate(sw, step.controlword);
    }

    CHECK(reached);
    CHECK(sm.enabled());
}

void test_enable_sequence_order() {
    // The controlword sequence must be Shutdown -> SwitchOn -> EnableOperation.
    StateMachine sm;
    sm.request(Request::Enable);

    std::uint16_t sw = 0x0040;

    auto s1 = sm.update(sw);
    CHECK(s1.controlword == cw::Shutdown);
    sw = simulate(sw, s1.controlword);

    auto s2 = sm.update(sw);
    CHECK(s2.controlword == cw::SwitchOn);
    sw = simulate(sw, s2.controlword);

    auto s3 = sm.update(sw);
    CHECK(s3.controlword == cw::EnableOperation);
    sw = simulate(sw, s3.controlword);

    auto s4 = sm.update(sw);
    CHECK(sm.state() == State::OperationEnabled);
    CHECK(s4.controlword == cw::EnableOperation);
}

void test_hold_target_until_enabled() {
    // THE safety property: the caller must be told to hold target == actual
    // every cycle until the drive is actually in OperationEnabled. A stale
    // target at enable makes the drive slam to it.
    StateMachine sm;
    sm.request(Request::Enable);

    std::uint16_t sw = 0x0040;
    for (int i = 0; i < 10; i++) {
        const auto step = sm.update(sw);
        if (sm.state() == State::OperationEnabled) {
            CHECK(!step.hold_target_at_actual);
        } else {
            CHECK(step.hold_target_at_actual);
        }
        sw = simulate(sw, step.controlword);
    }
}

void test_fault_reset_is_edge_triggered() {
    // Holding 0x0080 does not repeatedly reset. Bit 7 must toggle low and back.
    StateMachine sm;
    sm.request(Request::Enable);

    const std::uint16_t fault_sw = 0x0008;

    const auto a = sm.update(fault_sw);
    CHECK(sm.state() == State::Fault);
    CHECK(a.controlword == cw::FaultReset);       // bit 7 rises

    const auto b = sm.update(fault_sw);
    CHECK((b.controlword & cw::FaultReset) == 0); // bit 7 must fall

    const auto c = sm.update(fault_sw);
    CHECK(c.controlword == cw::FaultReset);       // and rise again
}

void test_fault_reaction_is_not_interfered_with() {
    StateMachine sm;
    sm.request(Request::Enable);

    const auto step = sm.update(0x000F);
    CHECK(sm.state() == State::FaultReactionActive);
    CHECK(sm.faulted());
    // Must not attempt a reset while the drive is still ramping down.
    CHECK((step.controlword & cw::FaultReset) == 0);
}

void test_disable_request_never_enables() {
    StateMachine sm;
    sm.request(Request::Disable);

    std::uint16_t sw = 0x0040;
    for (int i = 0; i < 20; i++) {
        const auto step = sm.update(sw);
        CHECK(step.controlword != cw::EnableOperation);
        sw = simulate(sw, step.controlword);
        CHECK(sm.state() != State::OperationEnabled);
    }
}

void test_quick_stop_from_enabled() {
    StateMachine sm;
    sm.request(Request::Enable);

    std::uint16_t sw = 0x0040;
    for (int i = 0; i < 10 && sm.state() != State::OperationEnabled; i++) {
        sw = simulate(sw, sm.update(sw).controlword);
    }
    (void)sm.update(sw);
    CHECK(sm.state() == State::OperationEnabled);

    sm.request(Request::QuickStop);
    const auto step = sm.update(sw);
    CHECK(step.controlword == cw::QuickStop);
}

void test_transition_and_dwell_counters() {
    StateMachine sm;
    (void)sm.update(0x0040);
    CHECK(sm.transitions() == 1);
    CHECK(sm.dwell_cycles() == 0);

    (void)sm.update(0x0040);
    (void)sm.update(0x0040);
    CHECK(sm.transitions() == 1);
    CHECK(sm.dwell_cycles() == 2);

    (void)sm.update(0x0021);
    CHECK(sm.transitions() == 2);
    CHECK(sm.dwell_cycles() == 0);
}

void test_reset() {
    StateMachine sm;
    sm.request(Request::Enable);
    (void)sm.update(0x0027);
    CHECK(sm.enabled());

    sm.reset();
    CHECK(sm.state() == State::Unknown);
    CHECK(sm.request() == Request::Disable);
    CHECK(sm.transitions() == 0);
}

void test_mode_values() {
    // CSP must be 8. Older DS402 v2.0 docs wrongly call 8-127 reserved.
    CHECK(static_cast<int>(Mode::CyclicSyncPosition) == 8);
    CHECK(static_cast<int>(Mode::CyclicSyncVelocity) == 9);
    CHECK(static_cast<int>(Mode::CyclicSyncTorque) == 10);
    CHECK(static_cast<int>(Mode::Homing) == 6);
    CHECK(static_cast<int>(Mode::ProfilePosition) == 1);
}

}  // namespace

int main() {
    std::printf("test_cia402\n");

    test_decode_canonical_patterns();
    test_decode_ignores_dont_care_bits();
    test_decode_bit5_discriminates_quickstop();
    test_decode_ordering_trap();
    test_decode_exhaustive_consistency();
    test_decode_unknown_is_reachable();
    test_statusword_bits();
    test_enable_sequence_reaches_operation_enabled();
    test_enable_sequence_order();
    test_hold_target_until_enabled();
    test_fault_reset_is_edge_triggered();
    test_fault_reaction_is_not_interfered_with();
    test_disable_request_never_enables();
    test_quick_stop_from_enabled();
    test_transition_and_dwell_counters();
    test_reset();
    test_mode_values();

    std::printf("  %d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
