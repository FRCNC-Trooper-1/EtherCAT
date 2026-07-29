// Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.
//
// Tests for multi-axis coordination.
//
// The rule under test, above all others:
//
//   WHEN ONE AXIS FAULTS, EVERY AXIS STOPS TOGETHER.
//
// and its companion, that a stop is a deceleration along the path rather than
// three axes independently freezing. Both are things you cannot safely discover
// on a machine with a tool in the spindle, so they are pinned down here against
// a simulated drive instead.

#include "frcnc/app/machine_controller.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>

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
namespace cw = frcnc::drive::cw;
using frcnc::drive::State;

constexpr std::uint16_t kSwSwitchOnDisabled = 0x0040;
constexpr std::uint16_t kSwReadyToSwitchOn = 0x0021;
constexpr std::uint16_t kSwSwitchedOn = 0x0023;
constexpr std::uint16_t kSwOperationEnabled = 0x0027;
constexpr std::uint16_t kSwQuickStopActive = 0x0007;
constexpr std::uint16_t kSwFault = 0x0008;

constexpr double kCountsPerMm = 10000.0;
constexpr double kDt = 0.001;
constexpr double kStopDecel = 5000.0;  // mm/s^2

// --- a drive on the other end of the wire -----------------------------------

/// Minimal CiA 402 slave: follows the controlword through the state machine and
/// tracks the commanded position exactly. A perfect servo is the right model
/// here — the point is to test the master's coordination, not a servo loop.
struct DriveSim {
    std::uint16_t sw = kSwSwitchOnDisabled;
    std::int32_t pos = 0;
    std::int32_t ferr = 0;

    /// Report a drive-side fault from now on.
    bool report_fault = false;
    /// Ignore the controlword entirely — a drive that will not come up.
    bool stuck = false;

    void apply(const AxisOutputs& o) noexcept {
        if (report_fault) {
            sw = kSwFault;
            return;
        }
        if (stuck) {
            return;
        }

        switch (frcnc::drive::decode_state(sw)) {
            case State::SwitchOnDisabled:
                if (o.controlword == cw::Shutdown) {
                    sw = kSwReadyToSwitchOn;
                }
                break;

            case State::ReadyToSwitchOn:
                if (o.controlword == cw::SwitchOn) {
                    sw = kSwSwitchedOn;
                } else if (o.controlword == cw::DisableVoltage) {
                    sw = kSwSwitchOnDisabled;
                }
                break;

            case State::SwitchedOn:
                if (o.controlword == cw::EnableOperation) {
                    sw = kSwOperationEnabled;
                } else if (o.controlword == cw::Shutdown) {
                    sw = kSwReadyToSwitchOn;
                } else if (o.controlword == cw::DisableVoltage) {
                    sw = kSwSwitchOnDisabled;
                }
                break;

            case State::OperationEnabled:
                if (o.controlword == cw::QuickStop) {
                    sw = kSwQuickStopActive;
                } else if (o.controlword == cw::SwitchOn) {
                    sw = kSwSwitchedOn;
                } else if (o.controlword == cw::Shutdown) {
                    sw = kSwReadyToSwitchOn;
                } else if (o.controlword == cw::DisableVoltage) {
                    sw = kSwSwitchOnDisabled;
                }
                break;

            case State::QuickStopActive:
                // Quick stop option code 0x605A default: decelerate, then drop
                // to Switch On Disabled on its own.
                sw = (o.controlword == cw::EnableOperation) ? kSwOperationEnabled
                                                            : kSwSwitchOnDisabled;
                break;

            case State::Fault:
                if ((o.controlword & cw::FaultReset) != 0) {
                    sw = kSwSwitchOnDisabled;
                }
                break;

            default:
                break;
        }

        pos = o.target_counts;
    }
};

/// Master plus three simulated drives, stepped one cycle at a time.
struct Machine {
    MachineController mc;
    frcnc::ipc::SetpointQueue setpoints;
    frcnc::ipc::CommandQueue commands;
    frcnc::ipc::StatusChannel status;
    DriveSim drive[3];
    MachineOutputs last{};

    void init(double decel = kStopDecel, std::uint32_t enable_timeout = 2000) {
        MachineConfig cfg;
        cfg.axis_count = 3;
        cfg.cycle_time = kDt;
        cfg.stop_deceleration = decel;
        cfg.enable_timeout_cycles = enable_timeout;
        for (int i = 0; i < 3; i++) {
            cfg.axis[i].counts_per_unit = kCountsPerMm;
            cfg.axis[i].soft_limit_min = -100.0;
            cfg.axis[i].soft_limit_max = 100.0;
            cfg.axis[i].following_error_limit = 0.5;
        }
        mc.configure(cfg);
        mc.set_channels(&setpoints, &commands, &status);
    }

    BusInputs sample(bool operational, bool wkc_ok) const {
        BusInputs in;
        in.operational = operational;
        in.wkc_ok = wkc_ok;
        in.working_counter = wkc_ok ? 9 : 7;
        in.expected_wkc = 9;
        in.dc_locked = true;
        for (int i = 0; i < 3; i++) {
            in.axis[i].statusword = drive[i].sw;
            in.axis[i].position_counts = drive[i].pos;
            in.axis[i].following_error_counts = drive[i].ferr;
            in.axis[i].mode_display = 8;
            // Slave-level only: the working-counter tolerance is the master's
            // decision, applied centrally. See BusInputs.
            in.axis[i].pdo_valid = operational;
        }
        return in;
    }

    /// One EtherCAT cycle: read the drives, run the master, write the drives.
    const MachineOutputs& step(bool operational = true, bool wkc_ok = true) {
        last = mc.update(sample(operational, wkc_ok));
        if (operational) {
            for (int i = 0; i < 3; i++) {
                drive[i].apply(last.axis[i]);
            }
        }
        return last;
    }

    void run(int cycles, bool operational = true, bool wkc_ok = true) {
        for (int i = 0; i < cycles; i++) {
            (void)step(operational, wkc_ok);
        }
    }

    /// Walk to Ready. Six cycles is enough for the four-state climb plus the
    /// cycle in which every axis is seen enabled.
    void bring_up() {
        mc.request_enable();
        run(8);
    }

    double position(int i) const { return static_cast<double>(drive[i].pos) / kCountsPerMm; }
};

frcnc::ipc::AxisSetpoint make_setpoint(double px, double py, double pz, double vx, double vy,
                                       double vz, std::uint64_t seq) {
    frcnc::ipc::AxisSetpoint sp;
    sp.position[0] = px;
    sp.position[1] = py;
    sp.position[2] = pz;
    sp.velocity[0] = vx;
    sp.velocity[1] = vy;
    sp.velocity[2] = vz;
    sp.sequence = seq;
    sp.block_id = 1;
    return sp;
}

// --- bring-up ---------------------------------------------------------------

void test_starts_idle() {
    Machine m;
    m.init();
    CHECK(m.mc.state() == MachineState::Idle);
    CHECK(!m.mc.enabled());
    CHECK(!m.mc.fault().active);
}

void test_enable_walks_every_axis_up() {
    Machine m;
    m.init();
    m.mc.request_enable();
    CHECK(m.mc.state() == MachineState::Enabling);

    m.run(8);
    CHECK(m.mc.state() == MachineState::Ready);
    CHECK(m.mc.enabled());
    for (int i = 0; i < 3; i++) {
        CHECK(m.drive[i].sw == kSwOperationEnabled);
    }
}

void test_target_tracks_actual_until_enabled() {
    // The rule that keeps a drive from slamming to a stale setpoint: while the
    // servo loop is open, the commanded position must equal the measured one.
    Machine m;
    m.init();
    for (int i = 0; i < 3; i++) {
        m.drive[i].pos = 12345 * (i + 1);
    }

    m.mc.request_enable();
    for (int c = 0; c < 8; c++) {
        const MachineOutputs& o = m.step();
        if (!m.mc.enabled()) {
            for (int i = 0; i < 3; i++) {
                CHECK(o.axis[i].target_counts == m.drive[i].pos);
            }
        }
    }
    CHECK(m.mc.enabled());
    for (int i = 0; i < 3; i++) {
        CHECK(m.drive[i].pos == 12345 * (i + 1));
    }
}

void test_ready_holds_the_commanded_position() {
    Machine m;
    m.init();
    for (int i = 0; i < 3; i++) {
        m.drive[i].pos = 50000;  // 5 mm
    }
    m.bring_up();
    CHECK(m.mc.state() == MachineState::Ready);

    m.run(50);
    for (int i = 0; i < 3; i++) {
        CHECK(m.drive[i].pos == 50000);
    }
    CHECK_NEAR(m.mc.path_velocity(), 0.0, 1e-12);
}

void test_enable_times_out_on_a_drive_that_never_comes_up() {
    Machine m;
    m.init(kStopDecel, 20);
    m.drive[1].stuck = true;

    m.mc.request_enable();
    m.run(30);

    CHECK(m.mc.state() == MachineState::Faulted);
    CHECK(m.mc.fault().active);
    CHECK(m.mc.fault().axis == 1);
    CHECK(m.mc.fault().reason == FaultReason::EnableTimeout);
}

// --- motion -----------------------------------------------------------------

void test_queued_setpoints_start_motion() {
    Machine m;
    m.init();
    m.bring_up();

    for (int k = 1; k <= 5; k++) {
        CHECK(m.setpoints.push(make_setpoint(0.1 * k, 0.2 * k, 0.0, 100.0, 200.0, 0.0,
                                             static_cast<std::uint64_t>(k))));
    }

    (void)m.step();
    CHECK(m.mc.state() == MachineState::Running);
    CHECK(m.mc.moving());
    CHECK_NEAR(m.position(0), 0.1, 1e-9);
    CHECK_NEAR(m.position(1), 0.2, 1e-9);
    CHECK_NEAR(m.mc.path_velocity(), std::sqrt(100.0 * 100.0 + 200.0 * 200.0), 1e-9);

    m.run(4);
    CHECK_NEAR(m.position(0), 0.5, 1e-9);
    CHECK_NEAR(m.position(1), 1.0, 1e-9);
}

void test_velocity_feedforward_reaches_the_pdo() {
    Machine m;
    m.init();
    m.bring_up();
    CHECK(m.setpoints.push(make_setpoint(0.1, 0.0, 0.0, 100.0, 0.0, 0.0, 1)));

    const MachineOutputs& o = m.step();
    // 100 mm/s * 10000 counts/mm = 1,000,000 counts/s into 0x60B1.
    CHECK(o.axis[0].velocity_offset == 1000000);
    CHECK(o.axis[1].velocity_offset == 0);
}

void test_end_of_path_returns_to_ready_and_holds() {
    Machine m;
    m.init();
    m.bring_up();

    frcnc::ipc::AxisSetpoint sp = make_setpoint(1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1);
    sp.end_of_path = true;
    CHECK(m.setpoints.push(sp));

    (void)m.step();
    CHECK(m.mc.state() == MachineState::Ready);
    CHECK_NEAR(m.position(0), 1.0, 1e-9);

    m.run(20);
    CHECK_NEAR(m.position(0), 1.0, 1e-9);
    CHECK_NEAR(m.position(1), 2.0, 1e-9);
}

// --- the coordinated stop ---------------------------------------------------

void test_starvation_stops_on_the_path() {
    // A planner that cannot keep the queue full is a planner that has lost the
    // path. Extrapolating is a guess about a machine that is cutting metal, so
    // the machine decelerates instead.
    Machine m;
    m.init();
    m.bring_up();

    CHECK(m.setpoints.push(make_setpoint(0.03, 0.04, 0.0, 30.0, 40.0, 0.0, 1)));
    (void)m.step();
    CHECK(m.mc.state() == MachineState::Running);

    (void)m.step();  // queue empty
    CHECK(m.mc.state() == MachineState::Stopping);
    CHECK(m.mc.starve_events() == 1);
}

void test_coordinated_stop_keeps_the_tool_on_its_last_direction() {
    // THE test. Every axis is scaled by the same factor, so the ratio of the
    // distances travelled during the stop equals the ratio of the velocities.
    // Freezing each axis independently — or stopping only one — drags the tool
    // off the path at full feed.
    Machine m;
    m.init();
    m.bring_up();

    CHECK(m.setpoints.push(make_setpoint(0.03, 0.04, 0.0, 30.0, 40.0, 0.0, 1)));
    (void)m.step();

    const double x0 = m.position(0);
    const double y0 = m.position(1);

    // speed 50 mm/s, decel 5000 mm/s^2 -> 10 ms, 10 cycles.
    for (int c = 0; c < 10; c++) {
        (void)m.step();
        CHECK(m.mc.state() == MachineState::Stopping || m.mc.state() == MachineState::Ready);
    }
    CHECK(m.mc.state() == MachineState::Ready);

    const double dx = m.position(0) - x0;
    const double dy = m.position(1) - y0;

    // Path distance is exactly v^2 / (2a) = 50^2 / 10000 = 0.25 mm.
    CHECK_NEAR(std::sqrt(dx * dx + dy * dy), 0.25, 1e-6);
    // Direction preserved: the 3-4-5 ratio survives the whole ramp.
    CHECK_NEAR(dx, 0.15, 1e-6);
    CHECK_NEAR(dy, 0.20, 1e-6);
    CHECK_NEAR(dy / dx, 40.0 / 30.0, 1e-6);
    CHECK_NEAR(m.mc.path_velocity(), 0.0, 1e-12);

    // And it stays put afterwards, holding the commanded position rather than
    // creeping by a following error every time it pauses.
    const double xr = m.position(0);
    m.run(20);
    CHECK_NEAR(m.position(0), xr, 1e-12);
}

void test_stop_ramp_is_monotonic() {
    Machine m;
    m.init();
    m.bring_up();
    CHECK(m.setpoints.push(make_setpoint(0.03, 0.04, 0.0, 30.0, 40.0, 0.0, 1)));
    (void)m.step();

    double previous = m.mc.path_velocity();
    double last_dx = 1e9;
    double x = m.position(0);
    for (int c = 0; c < 10; c++) {
        (void)m.step();
        const double v = m.mc.path_velocity();
        CHECK(v <= previous + 1e-12);
        previous = v;

        const double dx = m.position(0) - x;
        CHECK(dx >= -1e-12);          // never reverses
        CHECK(dx <= last_dx + 1e-12);  // and never speeds up
        last_dx = dx;
        x = m.position(0);
    }
    CHECK_NEAR(previous, 0.0, 1e-12);
}

void test_operator_stop_decelerates_rather_than_freezing() {
    Machine m;
    m.init();
    m.bring_up();
    for (int k = 1; k <= 40; k++) {
        CHECK(m.setpoints.push(make_setpoint(0.05 * k, 0.0, 0.0, 50.0, 0.0, 0.0,
                                             static_cast<std::uint64_t>(k))));
    }
    m.run(3);
    CHECK(m.mc.state() == MachineState::Running);

    const double x0 = m.position(0);
    m.mc.request_stop();
    m.run(20);

    CHECK(m.mc.state() == MachineState::Ready);
    CHECK(m.mc.starve_events() == 0);  // stopped on request, not by starving
    // 50^2 / (2 * 5000) = 0.25 mm.
    CHECK_NEAR(m.position(0) - x0, 0.25, 1e-6);
}

void test_stop_from_rest_is_a_no_op() {
    Machine m;
    m.init();
    m.bring_up();
    const double x0 = m.position(0);

    m.mc.request_stop();
    m.run(5);
    CHECK(m.mc.state() == MachineState::Ready);
    CHECK_NEAR(m.position(0) - x0, 0.0, 1e-12);
}

// --- faults -----------------------------------------------------------------

void test_one_axis_following_error_stops_every_axis() {
    // THE rule. A following error on Y must not leave X and Z interpolating
    // against a stationary axis.
    Machine m;
    m.init();
    m.bring_up();
    for (int k = 1; k <= 40; k++) {
        CHECK(m.setpoints.push(make_setpoint(0.05 * k, 0.05 * k, 0.0, 50.0, 50.0, 0.0,
                                             static_cast<std::uint64_t>(k))));
    }
    m.run(3);
    CHECK(m.mc.state() == MachineState::Running);

    m.drive[1].ferr = 8000;  // 0.8 mm, past the 0.5 mm limit
    (void)m.step();

    CHECK(m.mc.state() == MachineState::Faulted);
    CHECK(m.mc.fault().active);
    CHECK(m.mc.fault().axis == 1);
    CHECK(m.mc.fault().reason == FaultReason::FollowingError);
    CHECK(!m.mc.fault().bus_fault);

    // The offending axis quick-stops on the cycle the error is seen; the others
    // one cycle later, because the fault is detected from data that arrived in
    // the same cycle whose frame is already being composed. One cycle is 1 ms
    // here and 250 us at the design rate, and during it the healthy axes are
    // still following an on-path setpoint.
    CHECK(m.last.axis[1].controlword == cw::QuickStop);

    const MachineOutputs& o = m.step();
    CHECK(o.axis[0].controlword == cw::QuickStop);
    CHECK(o.axis[2].controlword == cw::QuickStop);
    // Axis 1 is already past it: its quick stop began a cycle earlier, so the
    // drive is in Quick Stop Active and must not be commanded back into
    // Operation Enabled.
    CHECK(m.mc.axis(1).state() == State::QuickStopActive);
    CHECK(o.axis[1].controlword != cw::EnableOperation);

    // And they all end up de-energised, not just the one that faulted.
    m.run(10);
    for (int i = 0; i < 3; i++) {
        CHECK(!m.mc.axis(i).enabled());
    }
    CHECK(!m.mc.enabled());
}

void test_drive_reported_fault_stops_every_axis() {
    Machine m;
    m.init();
    m.bring_up();
    CHECK(m.setpoints.push(make_setpoint(0.05, 0.0, 0.0, 50.0, 0.0, 0.0, 1)));
    m.run(1);

    m.drive[2].report_fault = true;
    m.run(2);

    CHECK(m.mc.state() == MachineState::Faulted);
    CHECK(m.mc.fault().axis == 2);
    CHECK(m.mc.fault().reason == FaultReason::DriveFault);
    CHECK(!m.mc.enabled());
}

void test_fault_record_keeps_the_first_cause() {
    // Faults cascade: a following error trips a bus error trips a
    // not-operational. The last one to arrive never explains anything.
    Machine m;
    m.init();
    m.bring_up();
    m.run(1);

    m.drive[0].ferr = 9000;
    (void)m.step();
    CHECK(m.mc.fault().reason == FaultReason::FollowingError);
    CHECK(m.mc.fault().axis == 0);
    const std::uint64_t at = m.mc.fault().cycle;

    // Now drop the bus on top of it.
    m.run(10, /*operational=*/false, /*wkc_ok=*/false);
    CHECK(m.mc.fault().reason == FaultReason::FollowingError);
    CHECK(m.mc.fault().axis == 0);
    CHECK(m.mc.fault().cycle == at);
    CHECK(!m.mc.fault().bus_fault);
}

void test_working_counter_run_faults_after_the_tolerance() {
    Machine m;
    m.init();
    m.bring_up();

    // Tolerance is 2, so two consecutive bad counters are survivable.
    m.run(2, true, /*wkc_ok=*/false);
    CHECK(m.mc.state() != MachineState::Faulted);

    (void)m.step(true, false);
    CHECK(m.mc.state() == MachineState::Faulted);
    CHECK(m.mc.fault().bus_fault);
    CHECK(m.mc.fault().axis == -1);
    CHECK(m.mc.fault().reason == FaultReason::NotOperational);
}

void test_isolated_working_counter_errors_are_survivable() {
    Machine m;
    m.init();
    m.bring_up();

    for (int k = 0; k < 20; k++) {
        (void)m.step(true, /*wkc_ok=*/false);
        m.run(5);  // recovers
    }
    CHECK(m.mc.state() == MachineState::Ready);
    CHECK(!m.mc.fault().active);
}

void test_bus_leaving_operational_faults() {
    Machine m;
    m.init();
    m.bring_up();

    (void)m.step(/*operational=*/false);
    CHECK(m.mc.state() == MachineState::Faulted);
    CHECK(m.mc.fault().bus_fault);
}

void test_idle_tolerates_a_bus_that_is_not_up_yet() {
    // Before the master has driven the slaves to OPERATIONAL this is simply
    // what a machine looks like. Faulting here would mean every cold start
    // begins with a fault to acknowledge.
    Machine m;
    m.init();
    m.run(50, /*operational=*/false, /*wkc_ok=*/false);
    CHECK(m.mc.state() == MachineState::Idle);
    CHECK(!m.mc.fault().active);

    // And the machine still enables normally once the bus does come up.
    m.bring_up();
    CHECK(m.mc.state() == MachineState::Ready);
    CHECK(m.mc.enabled());
}

void test_enable_is_refused_while_faulted() {
    Machine m;
    m.init();
    m.bring_up();
    m.drive[0].ferr = 9000;
    m.run(3);
    CHECK(m.mc.state() == MachineState::Faulted);

    m.mc.request_enable();
    m.run(10);
    CHECK(m.mc.state() == MachineState::Faulted);
    CHECK(!m.mc.enabled());
}

void test_clear_fault_then_re_enable() {
    Machine m;
    m.init();
    m.bring_up();
    m.drive[0].ferr = 9000;
    m.run(5);
    CHECK(m.mc.state() == MachineState::Faulted);

    m.drive[0].ferr = 0;
    m.mc.clear_faults();
    CHECK(m.mc.state() == MachineState::Idle);
    CHECK(!m.mc.fault().active);

    m.run(5);
    m.bring_up();
    CHECK(m.mc.state() == MachineState::Ready);
    CHECK(m.mc.enabled());
}

void test_drive_fault_is_reset_and_the_machine_recovers() {
    Machine m;
    m.init();
    m.bring_up();
    m.drive[1].report_fault = true;
    m.run(3);
    CHECK(m.mc.fault().reason == FaultReason::DriveFault);

    m.drive[1].report_fault = false;
    m.mc.clear_faults();
    m.run(10);  // fault-reset edge, then back to Switch On Disabled
    CHECK(!m.mc.axis(1).faulted());

    m.bring_up();
    CHECK(m.mc.state() == MachineState::Ready);
    CHECK(m.mc.enabled());
}

// --- operator commands ------------------------------------------------------

void test_commands_arrive_through_the_queue() {
    Machine m;
    m.init();

    frcnc::ipc::Command c;
    c.kind = frcnc::ipc::CommandKind::Enable;
    CHECK(m.commands.push(c));

    m.run(8);
    CHECK(m.mc.state() == MachineState::Ready);
    CHECK(m.mc.enabled());

    c.kind = frcnc::ipc::CommandKind::Disable;
    CHECK(m.commands.push(c));
    m.run(10);
    CHECK(m.mc.state() == MachineState::Idle);
    CHECK(!m.mc.enabled());
}

void test_emergency_stop_quick_stops_then_disables() {
    Machine m;
    m.init();
    m.bring_up();
    for (int k = 1; k <= 20; k++) {
        CHECK(m.setpoints.push(make_setpoint(0.05 * k, 0.0, 0.0, 50.0, 0.0, 0.0,
                                             static_cast<std::uint64_t>(k))));
    }
    m.run(3);

    frcnc::ipc::Command c;
    c.kind = frcnc::ipc::CommandKind::EmergencyStop;
    CHECK(m.commands.push(c));

    const MachineOutputs& o = m.step();
    CHECK(m.mc.state() == MachineState::Disabling);
    for (int i = 0; i < 3; i++) {
        CHECK(o.axis[i].controlword == cw::QuickStop);
    }

    m.run(10);
    CHECK(m.mc.state() == MachineState::Idle);
    CHECK(!m.mc.enabled());
    // An operator stop is not a fault: nothing to acknowledge afterwards.
    CHECK(!m.mc.fault().active);
}

void test_feed_hold_stops_and_latches_until_resumed() {
    Machine m;
    m.init();
    m.bring_up();
    for (int k = 1; k <= 40; k++) {
        CHECK(m.setpoints.push(make_setpoint(0.05 * k, 0.0, 0.0, 50.0, 0.0, 0.0,
                                             static_cast<std::uint64_t>(k))));
    }
    m.run(3);

    frcnc::ipc::Command c;
    c.kind = frcnc::ipc::CommandKind::HoldFeed;
    CHECK(m.commands.push(c));
    m.run(20);

    CHECK(m.mc.state() == MachineState::Ready);
    CHECK(m.mc.feed_held());
    const double held_at = m.position(0);

    // The queued setpoints described the path we just decelerated away from.
    // Replaying them would jump the axis forward by the stopping distance, so
    // they are gone and the planner must re-plan from the reported position.
    CHECK(m.setpoints.empty());

    m.run(20);
    CHECK(m.mc.state() == MachineState::Ready);
    CHECK_NEAR(m.position(0), held_at, 1e-12);

    // Resuming alone does not restart motion: nothing is queued.
    c.kind = frcnc::ipc::CommandKind::ResumeFeed;
    CHECK(m.commands.push(c));
    m.run(3);
    CHECK(!m.mc.feed_held());
    CHECK(m.mc.state() == MachineState::Ready);
    CHECK_NEAR(m.position(0), held_at, 1e-12);

    // A re-planned path, starting from where the machine actually stopped, does.
    CHECK(m.setpoints.push(make_setpoint(held_at + 0.001, 0.0, 0.0, 1.0, 0.0, 0.0, 100)));
    (void)m.step();
    CHECK(m.mc.state() == MachineState::Running);
    CHECK_NEAR(m.position(0), held_at + 0.001, 1e-9);
}

void test_feed_hold_blocks_motion_that_has_not_started() {
    Machine m;
    m.init();
    m.bring_up();

    frcnc::ipc::Command c;
    c.kind = frcnc::ipc::CommandKind::HoldFeed;
    CHECK(m.commands.push(c));
    m.run(2);
    CHECK(m.mc.feed_held());

    CHECK(m.setpoints.push(make_setpoint(1.0, 0.0, 0.0, 50.0, 0.0, 0.0, 1)));
    m.run(5);
    CHECK(m.mc.state() == MachineState::Ready);
    CHECK_NEAR(m.position(0), 0.0, 1e-12);
}

// --- status -----------------------------------------------------------------

void test_status_is_published_every_cycle() {
    Machine m;
    m.init();
    m.bring_up();
    m.mc.set_timing(4200, 21500);
    CHECK(m.setpoints.push(make_setpoint(0.05, 0.0, 0.0, 50.0, 0.0, 0.0, 7)));
    CHECK(m.setpoints.push(make_setpoint(0.10, 0.0, 0.0, 50.0, 0.0, 0.0, 8)));
    // Two cycles: status reports the position the drive FED BACK, so the move
    // commanded in cycle one shows up in cycle two.
    m.run(2);

    frcnc::ipc::MachineStatus s;
    CHECK(m.status.load(s));
    CHECK(s.cycle == m.mc.cycles());
    CHECK(s.setpoint_sequence == 8);
    CHECK(s.bus_operational);
    CHECK(s.wkc_ok);
    CHECK(s.motion_active);
    CHECK(!s.fault_active);
    CHECK(!s.starved);
    CHECK(s.cycle_jitter_ns == 4200);
    CHECK(s.max_cycle_jitter_ns == 21500);
    CHECK_NEAR(s.path_velocity, 50.0, 1e-9);
    CHECK_NEAR(s.axis[0].position_actual, 0.05, 1e-9);
    CHECK(s.axis[0].enabled);

    // Actual velocity is differentiated from measured position: 0.05 mm in
    // 1 ms is 50 mm/s.
    CHECK_NEAR(s.axis[0].velocity_actual, 50.0, 1e-6);
}

void test_status_reports_the_fault() {
    Machine m;
    m.init();
    m.bring_up();
    m.drive[0].ferr = 9000;
    m.run(2);

    frcnc::ipc::MachineStatus s;
    CHECK(m.status.load(s));
    CHECK(s.fault_active);
    CHECK(s.axis[0].faulted);
    CHECK(!s.motion_active);
}

// --- housekeeping -----------------------------------------------------------

void test_reset_returns_to_a_cold_machine() {
    Machine m;
    m.init();
    m.bring_up();
    m.drive[0].ferr = 9000;
    m.run(3);
    CHECK(m.mc.fault().active);

    m.mc.reset();
    CHECK(m.mc.state() == MachineState::Idle);
    CHECK(!m.mc.fault().active);
    CHECK(m.mc.cycles() == 0);
    CHECK(m.mc.starve_events() == 0);
    CHECK(!m.mc.enabled());
}

void test_state_names_are_all_present() {
    const MachineState all[] = {MachineState::Idle,     MachineState::Enabling,
                                MachineState::Ready,    MachineState::Running,
                                MachineState::Stopping, MachineState::Faulted,
                                MachineState::Disabling};
    for (MachineState s : all) {
        CHECK(to_string(s)[0] != '?');
    }
    CHECK(to_string(FaultReason::EnableTimeout)[0] != '?');
}

}  // namespace

int main() {
    std::printf("machine_controller\n");

    test_starts_idle();
    test_enable_walks_every_axis_up();
    test_target_tracks_actual_until_enabled();
    test_ready_holds_the_commanded_position();
    test_enable_times_out_on_a_drive_that_never_comes_up();

    test_queued_setpoints_start_motion();
    test_velocity_feedforward_reaches_the_pdo();
    test_end_of_path_returns_to_ready_and_holds();

    test_starvation_stops_on_the_path();
    test_coordinated_stop_keeps_the_tool_on_its_last_direction();
    test_stop_ramp_is_monotonic();
    test_operator_stop_decelerates_rather_than_freezing();
    test_stop_from_rest_is_a_no_op();

    test_one_axis_following_error_stops_every_axis();
    test_drive_reported_fault_stops_every_axis();
    test_fault_record_keeps_the_first_cause();
    test_working_counter_run_faults_after_the_tolerance();
    test_isolated_working_counter_errors_are_survivable();
    test_bus_leaving_operational_faults();
    test_idle_tolerates_a_bus_that_is_not_up_yet();
    test_enable_is_refused_while_faulted();
    test_clear_fault_then_re_enable();
    test_drive_fault_is_reset_and_the_machine_recovers();

    test_commands_arrive_through_the_queue();
    test_emergency_stop_quick_stops_then_disables();
    test_feed_hold_stops_and_latches_until_resumed();
    test_feed_hold_blocks_motion_that_has_not_started();

    test_status_is_published_every_cycle();
    test_status_reports_the_fault();

    test_reset_returns_to_a_cold_machine();
    test_state_names_are_all_present();

    std::printf("  %d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
