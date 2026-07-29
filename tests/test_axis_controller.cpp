// Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.
//
// Tests for per-axis control.
//
// These are the safety rules of the machine expressed as code, so they get
// tested as rules rather than as functions. The one that matters most is that
// the commanded position tracks the measured position whenever the servo loop
// is open — a stale target at the moment torque is applied makes the drive slam
// to it at maximum acceleration.

#include "frcnc/app/axis_controller.hpp"

#include <cmath>
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

void check_near(double a, double b, double tol, const char* expr, const char* file, int line) {
    g_checks++;
    if (!(std::fabs(a - b) <= tol)) {
        g_failures++;
        std::printf("  FAIL %s:%d  %s   (%.9g vs %.9g)\n", file, line, expr, a, b);
    }
}

#define CHECK(expr) check((expr), #expr, __FILE__, __LINE__)
#define CHECK_NEAR(a, b, tol) check_near((a), (b), (tol), #a " ~= " #b, __FILE__, __LINE__)

using namespace frcnc::app;
using frcnc::drive::Request;
using frcnc::drive::State;

/// 10,000 counts per mm — a 10 mm ballscrew with a 100,000 count/rev encoder.
AxisConfig basic_config() {
    AxisConfig c;
    c.counts_per_unit = 10000.0;
    c.soft_limit_min = -10.0;
    c.soft_limit_max = 500.0;
    c.following_error_limit = 0.5;
    return c;
}

// Statusword patterns, from docs/04-drive-cia402.md.
constexpr std::uint16_t kSwSwitchOnDisabled = 0x0040;
constexpr std::uint16_t kSwReadyToSwitchOn = 0x0021;
constexpr std::uint16_t kSwSwitchedOn = 0x0023;
constexpr std::uint16_t kSwOperationEnabled = 0x0027;
constexpr std::uint16_t kSwFault = 0x0008;

AxisInputs inputs(std::uint16_t sw, std::int32_t pos, std::int32_t ferr = 0) {
    AxisInputs in;
    in.statusword = sw;
    in.position_counts = pos;
    in.following_error_counts = ferr;
    in.pdo_valid = true;
    return in;
}

AxisCommand command(double pos, double vel = 0.0, double acc = 0.0) {
    AxisCommand c;
    c.position = pos;
    c.velocity = vel;
    c.acceleration = acc;
    c.valid = true;
    return c;
}

/// Drive the axis to Operation Enabled, holding at a fixed position.
void bring_up(AxisController& ax, std::int32_t at_counts) {
    ax.request(Request::Enable);
    const std::uint16_t seq[] = {kSwSwitchOnDisabled, kSwReadyToSwitchOn, kSwSwitchedOn,
                                 kSwOperationEnabled};
    for (std::uint16_t sw : seq) {
        AxisCommand idle;
        (void)ax.update(inputs(sw, at_counts), idle);
    }
}

// --- unit conversion --------------------------------------------------------

void test_unit_conversion_roundtrip() {
    AxisController ax;
    ax.configure(basic_config());

    CHECK(ax.to_counts(1.0) == 10000);
    CHECK(ax.to_counts(-2.5) == -25000);
    CHECK_NEAR(ax.to_units(10000), 1.0, 1e-12);
    CHECK_NEAR(ax.to_units(-25000), -2.5, 1e-12);
}

void test_inverted_axis() {
    AxisConfig c = basic_config();
    c.invert = true;
    AxisController ax;
    ax.configure(c);

    CHECK(ax.to_counts(1.0) == -10000);
    CHECK_NEAR(ax.to_units(-10000), 1.0, 1e-12);
}

void test_conversion_saturates_rather_than_wrapping() {
    // A double beyond INT32 range is a bug upstream. Wrapping it would command
    // the axis to the far end of travel at full speed.
    AxisController ax;
    ax.configure(basic_config());
    CHECK(ax.to_counts(1e9) == 2147483647);
    CHECK(ax.to_counts(-1e9) == -2147483647 - 1);
    CHECK(ax.to_counts(std::nan("")) == 0);
}

void test_zero_scale_is_rejected() {
    // Dividing by counts_per_unit later would produce infinities.
    AxisConfig c = basic_config();
    c.counts_per_unit = 0.0;
    AxisController ax;
    ax.configure(c);
    CHECK(ax.to_counts(1.0) == 1);  // fell back to 1.0
}

// --- the slam-prevention rule -----------------------------------------------

void test_target_tracks_actual_while_not_enabled() {
    // THE rule. Whatever the planner says, while the servo loop is open the
    // target must equal the measured position.
    AxisController ax;
    ax.configure(basic_config());
    ax.request(Request::Enable);

    const std::int32_t at = 123456;

    // A wildly different commanded position must be ignored entirely.
    const AxisCommand far = command(400.0);

    const std::uint16_t seq[] = {kSwSwitchOnDisabled, kSwReadyToSwitchOn, kSwSwitchedOn};
    for (std::uint16_t sw : seq) {
        const AxisOutputs out = ax.update(inputs(sw, at), far);
        CHECK(out.target_counts == at);
        CHECK(out.velocity_offset == 0);
        CHECK(out.torque_offset == 0);
        CHECK(!ax.enabled());
    }
}

void test_no_position_step_at_the_moment_of_enable() {
    // Walk to Operation Enabled while the axis sits at a non-zero position,
    // then check the first enabled cycle does not jump.
    AxisController ax;
    ax.configure(basic_config());
    ax.request(Request::Enable);

    const std::int32_t at = 250000;  // 25 mm
    bring_up(ax, at);
    CHECK(ax.enabled());

    // First enabled cycle with a command AT the current position.
    const AxisOutputs out = ax.update(inputs(kSwOperationEnabled, at), command(25.0));
    CHECK(out.target_counts == at);
    CHECK_NEAR(ax.commanded_position(), 25.0, 1e-9);
}

void test_starved_planner_holds_position() {
    // No setpoint this cycle: hold, do not extrapolate. An extrapolated
    // setpoint is a guess about a machine that is cutting metal.
    AxisController ax;
    ax.configure(basic_config());
    bring_up(ax, 100000);
    CHECK(ax.enabled());

    AxisCommand none;  // valid == false
    const AxisOutputs out = ax.update(inputs(kSwOperationEnabled, 100000), none);
    CHECK(out.target_counts == 100000);
    CHECK(out.velocity_offset == 0);
}

// --- normal operation -------------------------------------------------------

void test_follows_command_when_enabled() {
    AxisController ax;
    ax.configure(basic_config());
    bring_up(ax, 0);
    CHECK(ax.enabled());

    const AxisOutputs out = ax.update(inputs(kSwOperationEnabled, 0), command(10.0, 100.0, 5.0));
    CHECK(out.target_counts == 100000);
    CHECK(out.velocity_offset == 1000000);  // 100 mm/s * 10000 counts/mm
    CHECK(out.controlword == frcnc::drive::cw::EnableOperation);
}

void test_feedforward_sign_follows_inversion() {
    AxisConfig c = basic_config();
    c.invert = true;
    AxisController ax;
    ax.configure(c);
    bring_up(ax, 0);

    const AxisOutputs out = ax.update(inputs(kSwOperationEnabled, 0), command(10.0, 100.0));
    CHECK(out.target_counts == -100000);
    CHECK(out.velocity_offset == -1000000);
}

void test_position_is_reported_in_user_units() {
    AxisController ax;
    ax.configure(basic_config());
    AxisCommand idle;
    (void)ax.update(inputs(kSwSwitchOnDisabled, 375000), idle);
    CHECK_NEAR(ax.position(), 37.5, 1e-9);
}

// --- faults -----------------------------------------------------------------

void test_drive_fault_is_detected() {
    AxisController ax;
    ax.configure(basic_config());
    bring_up(ax, 0);

    AxisCommand idle;
    (void)ax.update(inputs(kSwFault, 0), idle);
    CHECK(ax.faulted());
    CHECK(ax.fault() == FaultReason::DriveFault);
    CHECK(!ax.enabled());
}

void test_drive_fault_cannot_be_cleared_by_the_master() {
    // clear_fault() must not mask a drive-reported fault; only the drive
    // leaving its Fault state may lift it.
    AxisController ax;
    ax.configure(basic_config());
    AxisCommand idle;
    (void)ax.update(inputs(kSwFault, 0), idle);
    CHECK(ax.fault() == FaultReason::DriveFault);

    ax.clear_fault();
    CHECK(ax.fault() == FaultReason::DriveFault);

    // Once the drive is out of Fault, the latch lifts.
    (void)ax.update(inputs(kSwSwitchOnDisabled, 0), idle);
    CHECK(!ax.faulted());
}

void test_following_error_faults_the_axis() {
    AxisController ax;
    ax.configure(basic_config());  // limit 0.5 mm
    bring_up(ax, 0);

    // 0.4 mm is within limit.
    (void)ax.update(inputs(kSwOperationEnabled, 0, 4000), command(0.0));
    CHECK(!ax.faulted());

    // 0.6 mm is not.
    (void)ax.update(inputs(kSwOperationEnabled, 0, 6000), command(0.0));
    CHECK(ax.faulted());
    CHECK(ax.fault() == FaultReason::FollowingError);
}

void test_following_error_check_can_be_disabled() {
    AxisConfig c = basic_config();
    c.following_error_limit = 0.0;
    AxisController ax;
    ax.configure(c);
    bring_up(ax, 0);

    (void)ax.update(inputs(kSwOperationEnabled, 0, 999999), command(0.0));
    CHECK(!ax.faulted());
}

void test_soft_limits_fault_on_measured_position() {
    // Checked against measurement, not the command: a command inside the
    // envelope does not prove the axis is inside it.
    AxisController ax;
    ax.configure(basic_config());  // -10 .. 500 mm
    bring_up(ax, 0);

    (void)ax.update(inputs(kSwOperationEnabled, 5010000), command(0.0));  // 501 mm
    CHECK(ax.faulted());
    CHECK(ax.fault() == FaultReason::SoftLimitHigh);

    AxisController ax2;
    ax2.configure(basic_config());
    bring_up(ax2, 0);
    (void)ax2.update(inputs(kSwOperationEnabled, -110000), command(0.0));  // -11 mm
    CHECK(ax2.fault() == FaultReason::SoftLimitLow);
}

void test_command_is_clamped_into_the_envelope() {
    // Belt and braces against a planner bug: even a valid command outside the
    // envelope must not be passed through to the drive.
    AxisController ax;
    ax.configure(basic_config());
    bring_up(ax, 0);

    const AxisOutputs out = ax.update(inputs(kSwOperationEnabled, 0), command(9999.0));
    CHECK(out.target_counts == ax.to_counts(500.0));
}

void test_invalid_pdo_faults_and_disables() {
    AxisController ax;
    ax.configure(basic_config());
    bring_up(ax, 1000);

    AxisInputs bad = inputs(kSwOperationEnabled, 1000);
    bad.pdo_valid = false;

    (void)ax.update(bad, command(5.0));
    CHECK(ax.faulted());
    CHECK(ax.fault() == FaultReason::NotOperational);
    CHECK(ax.request() == Request::Disable);
}

void test_bus_dropout_reported_as_dropout_not_following_error() {
    // Fault ordering: the most fundamental condition wins, so a dropout is not
    // misreported as the following error it causes.
    AxisController ax;
    ax.configure(basic_config());
    bring_up(ax, 0);

    AxisInputs bad = inputs(kSwOperationEnabled, 0, 999999);
    bad.pdo_valid = false;

    (void)ax.update(bad, command(0.0));
    CHECK(ax.fault() == FaultReason::NotOperational);
}

void test_master_fault_requests_a_stop() {
    // A master-detected fault must not leave the axis energised and following.
    // While the drive is still in Operation Enabled it can run its own
    // quick-stop ramp, so that is what we ask for -- a shutdown here would
    // coast or brake per 0x605B instead of decelerating under control.
    AxisController ax;
    ax.configure(basic_config());
    bring_up(ax, 0);
    CHECK(ax.request() == Request::Enable);

    (void)ax.update(inputs(kSwOperationEnabled, 0, 6000), command(0.0));
    CHECK(ax.fault() == FaultReason::FollowingError);
    CHECK(ax.request() == Request::QuickStop);

    // Once the drive is no longer enabled it cannot ramp, so the request must
    // fall back to walking the axis down rather than leaving it powered.
    (void)ax.update(inputs(kSwSwitchOnDisabled, 0, 6000), command(0.0));
    CHECK(ax.request() == Request::Disable);
}

void test_master_fault_can_be_cleared_and_axis_re_enabled() {
    AxisController ax;
    ax.configure(basic_config());
    bring_up(ax, 0);

    (void)ax.update(inputs(kSwOperationEnabled, 0, 6000), command(0.0));
    CHECK(ax.faulted());

    ax.clear_fault();
    CHECK(!ax.faulted());

    bring_up(ax, 0);
    CHECK(ax.enabled());
}

// --- lifecycle --------------------------------------------------------------

void test_disable_request_never_enables() {
    AxisController ax;
    ax.configure(basic_config());
    ax.request(Request::Disable);

    AxisCommand idle;
    for (int i = 0; i < 20; i++) {
        const AxisOutputs out = ax.update(inputs(kSwSwitchOnDisabled, 0), idle);
        CHECK(out.controlword != frcnc::drive::cw::EnableOperation);
        CHECK(!ax.enabled());
    }
}

void test_reset_clears_everything() {
    AxisController ax;
    ax.configure(basic_config());
    bring_up(ax, 500000);
    (void)ax.update(inputs(kSwFault, 500000), command(0.0));
    CHECK(ax.faulted());

    ax.reset();
    CHECK(!ax.faulted());
    CHECK(ax.state() == State::Unknown);
    CHECK(ax.cycles() == 0);
    CHECK_NEAR(ax.position(), 0.0, 1e-12);
}

void test_mode_is_commanded() {
    AxisController ax;
    ax.configure(basic_config());
    AxisCommand idle;
    const AxisOutputs out = ax.update(inputs(kSwSwitchOnDisabled, 0), idle);
    CHECK(out.mode == 8);  // CSP
}

}  // namespace

int main() {
    std::printf("test_axis_controller\n");

    test_unit_conversion_roundtrip();
    test_inverted_axis();
    test_conversion_saturates_rather_than_wrapping();
    test_zero_scale_is_rejected();

    test_target_tracks_actual_while_not_enabled();
    test_no_position_step_at_the_moment_of_enable();
    test_starved_planner_holds_position();

    test_follows_command_when_enabled();
    test_feedforward_sign_follows_inversion();
    test_position_is_reported_in_user_units();

    test_drive_fault_is_detected();
    test_drive_fault_cannot_be_cleared_by_the_master();
    test_following_error_faults_the_axis();
    test_following_error_check_can_be_disabled();
    test_soft_limits_fault_on_measured_position();
    test_command_is_clamped_into_the_envelope();
    test_invalid_pdo_faults_and_disables();
    test_bus_dropout_reported_as_dropout_not_following_error();
    test_master_fault_requests_a_stop();
    test_master_fault_can_be_cleared_and_axis_re_enabled();

    test_disable_request_never_enables();
    test_reset_clears_everything();
    test_mode_is_commanded();

    std::printf("  %d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
