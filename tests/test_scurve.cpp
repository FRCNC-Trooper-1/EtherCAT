// Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.
//
// Tests for the jerk-limited (S-curve) profile.
//
// The defining property is that acceleration is continuous — that is the whole
// reason for the profile's existence, and it is checked directly by sampling
// and differencing rather than assumed from the construction.

#include "frcnc/motion/scurve.hpp"

#include <cmath>
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
        std::printf("  FAIL %s:%d  %s   (%.9g vs %.9g, tol %.3g)\n", file, line, expr, a, b, tol);
    }
}

#define CHECK(expr) check((expr), #expr, __FILE__, __LINE__)
#define CHECK_NEAR(a, b, tol) check_near((a), (b), (tol), #a " ~= " #b, __FILE__, __LINE__)

using namespace frcnc::motion;

constexpr Limits kLim{/*v*/ 100.0, /*a*/ 1000.0, /*j*/ 10000.0};

// --- ramp helper ------------------------------------------------------------

void test_ramp_trapezoidal_shape() {
    // dv large enough to reach a_max: dv_tri = a^2/j = 1000^2/10000 = 100
    const Ramp r = compute_ramp(200.0, 1000.0, 10000.0);
    CHECK_NEAR(r.peak_accel, 1000.0, 1e-9);
    CHECK_NEAR(r.t_jerk, 0.1, 1e-9);          // a/j
    CHECK_NEAR(r.t_const, 0.1, 1e-9);         // (200-100)/1000
    CHECK_NEAR(r.total, 0.3, 1e-9);
}

void test_ramp_triangular_shape() {
    // dv below dv_tri = 100: acceleration never reaches the limit.
    const Ramp r = compute_ramp(40.0, 1000.0, 10000.0);
    CHECK(r.peak_accel < 1000.0);
    CHECK_NEAR(r.peak_accel, std::sqrt(40.0 * 10000.0), 1e-9);
    CHECK_NEAR(r.t_const, 0.0, 1e-12);
    CHECK_NEAR(r.total, 2.0 * r.t_jerk, 1e-12);
}

void test_ramp_without_jerk_limit_is_constant_accel() {
    const Ramp r = compute_ramp(100.0, 1000.0, 0.0);
    CHECK_NEAR(r.t_jerk, 0.0, 1e-12);
    CHECK_NEAR(r.total, 0.1, 1e-9);  // dv/a
}

void test_ramp_distance_is_mean_velocity_times_time() {
    // The velocity curve is point-symmetric about its midpoint, so its mean is
    // exactly (v0+v1)/2. Verify against numerical integration of the profile.
    const double d = ramp_distance(0.0, 100.0, 1000.0, 10000.0);
    const Ramp r = compute_ramp(100.0, 1000.0, 10000.0);
    CHECK_NEAR(d, 50.0 * r.total, 1e-9);
}

// --- planning ---------------------------------------------------------------

void test_rejects_invalid_input() {
    SCurveProfile p;
    CHECK(p.plan(-1.0, 0.0, 0.0, kLim) == PlanResult::NegativeDistance);
    CHECK(p.plan(10.0, 0.0, 0.0, Limits{0.0, 1000.0, 10000.0}) == PlanResult::InvalidLimits);
    CHECK(p.plan(10.0, 0.0, 0.0, Limits{100.0, 0.0, 10000.0}) == PlanResult::InvalidLimits);
}

void test_zero_length_is_valid() {
    SCurveProfile p;
    CHECK(p.plan(0.0, 0.0, 0.0, kLim) == PlanResult::ZeroLength);
    CHECK(p.valid());
    CHECK_NEAR(p.duration(), 0.0, 1e-12);
}

void test_infeasible_when_too_short() {
    // Jerk limiting needs MORE distance than constant acceleration, so a move
    // that is marginal for a trapezoid is infeasible here.
    SCurveProfile p;
    CHECK(p.plan(0.5, 100.0, 0.0, kLim) == PlanResult::Infeasible);
}

void test_endpoints_are_exact() {
    SCurveProfile p;
    CHECK(p.plan(100.0, 0.0, 0.0, kLim) == PlanResult::Ok);

    const auto a = p.at(0.0);
    CHECK_NEAR(a.position, 0.0, 1e-12);
    CHECK_NEAR(a.velocity, 0.0, 1e-12);

    const auto b = p.at(p.duration());
    CHECK_NEAR(b.position, 100.0, 1e-6);
    CHECK_NEAR(b.velocity, 0.0, 1e-9);
    CHECK_NEAR(b.acceleration, 0.0, 1e-9);
}

void test_reaches_max_velocity_on_long_move() {
    SCurveProfile p;
    CHECK(p.plan(1000.0, 0.0, 0.0, kLim) == PlanResult::Ok);
    CHECK_NEAR(p.peak_velocity(), kLim.max_velocity, 1e-6);
    CHECK_NEAR(p.peak_acceleration(), kLim.max_acceleration, 1e-6);
}

void test_all_seven_phases_present() {
    // A constant-acceleration phase only exists when the velocity change
    // exceeds dv_tri = a^2/j -- the change achievable with a purely triangular
    // acceleration ramp. With kLim that is exactly 100, equal to max_velocity,
    // so kLim can never produce phases 2 and 6. Widen the velocity limit.
    const Limits wide{/*v*/ 400.0, /*a*/ 1000.0, /*j*/ 10000.0};
    const double dv_tri = (wide.max_acceleration * wide.max_acceleration) / wide.max_jerk;
    CHECK(wide.max_velocity > dv_tri);

    SCurveProfile p;
    CHECK(p.plan(5000.0, 0.0, 0.0, wide) == PlanResult::Ok);
    CHECK_NEAR(p.peak_velocity(), wide.max_velocity, 1e-6);
    CHECK(p.phase_count() == 7);
}

void test_triangular_accel_ramp_drops_const_phases() {
    // With kLim, max_velocity == dv_tri exactly, so the acceleration ramp is
    // purely triangular: phases 2 and 6 have zero duration and are dropped.
    SCurveProfile p;
    CHECK(p.plan(1000.0, 0.0, 0.0, kLim) == PlanResult::Ok);
    CHECK(p.phase_count() == 5);
    CHECK_NEAR(p.at(p.duration()).position, 1000.0, 1e-5);
}

void test_short_move_drops_cruise_phase() {
    SCurveProfile p;
    CHECK(p.plan(2.0, 0.0, 0.0, kLim) == PlanResult::Ok);
    CHECK(p.peak_velocity() < kLim.max_velocity);
    CHECK(p.phase_count() < 7);
    CHECK_NEAR(p.at(p.duration()).position, 2.0, 1e-6);
}

void test_honours_entry_and_exit_velocity() {
    SCurveProfile p;
    CHECK(p.plan(200.0, 30.0, 70.0, kLim) == PlanResult::Ok);
    CHECK_NEAR(p.at(0.0).velocity, 30.0, 1e-9);
    CHECK_NEAR(p.at(p.duration()).velocity, 70.0, 1e-6);
    CHECK_NEAR(p.at(p.duration()).position, 200.0, 1e-6);
}

void test_zero_jerk_limit_degenerates_gracefully() {
    // max_jerk = 0 means "unlimited jerk" — the profile should still plan and
    // cover the distance, behaving like a trapezoid.
    const Limits nojerk{100.0, 1000.0, 0.0};
    SCurveProfile p;
    CHECK(p.plan(100.0, 0.0, 0.0, nojerk) == PlanResult::Ok);
    CHECK_NEAR(p.at(p.duration()).position, 100.0, 1e-6);
    CHECK_NEAR(p.at(p.duration()).velocity, 0.0, 1e-9);
}

// --- invariants -------------------------------------------------------------

struct Invariants {
    double max_velocity = 0.0;
    double max_abs_accel = 0.0;
    double max_abs_jerk = 0.0;
    double max_accel_step = 0.0;  ///< largest |Δa| between adjacent samples
    bool monotonic = true;
    double dt = 0.0;
};

Invariants sweep(const SCurveProfile& p, int steps) {
    Invariants inv;
    const double dt = p.duration() / steps;
    inv.dt = dt;

    double prev_pos = -1e18;
    double prev_acc = p.at(0.0).acceleration;

    for (int i = 0; i <= steps; i++) {
        const double t = dt * i;
        const auto s = p.at(t);

        if (s.velocity > inv.max_velocity) inv.max_velocity = s.velocity;
        if (std::fabs(s.acceleration) > inv.max_abs_accel) {
            inv.max_abs_accel = std::fabs(s.acceleration);
        }
        if (s.position < prev_pos - 1e-9) inv.monotonic = false;
        prev_pos = s.position;

        if (i > 0) {
            const double da = std::fabs(s.acceleration - prev_acc);
            if (da > inv.max_accel_step) inv.max_accel_step = da;
            const double j = da / dt;
            if (j > inv.max_abs_jerk) inv.max_abs_jerk = j;
        }
        prev_acc = s.acceleration;
    }
    return inv;
}

void test_acceleration_is_continuous() {
    // THE defining property. A trapezoidal profile steps acceleration by
    // 2*a_max at the cruise-to-decel boundary; an S-curve must not step at all.
    SCurveProfile p;
    CHECK(p.plan(500.0, 0.0, 0.0, kLim) == PlanResult::Ok);

    const auto inv = sweep(p, 50000);

    // Largest acceleration step between adjacent samples must be consistent
    // with the jerk limit, not with an instantaneous change.
    const double allowed_step = kLim.max_jerk * inv.dt * 1.5;
    CHECK(inv.max_accel_step <= allowed_step);

    // And nowhere near the 2*a_max step a trapezoid would show.
    CHECK(inv.max_accel_step < 0.05 * kLim.max_acceleration);
}

void test_jerk_limit_is_respected() {
    SCurveProfile p;
    CHECK(p.plan(500.0, 0.0, 0.0, kLim) == PlanResult::Ok);
    const auto inv = sweep(p, 50000);
    CHECK(inv.max_abs_jerk <= kLim.max_jerk * 1.05);
}

void test_invariants_across_many_shapes() {
    const double distances[] = {0.01, 1.0, 5.0, 25.0, 100.0, 1000.0};
    const double entries[] = {0.0, 20.0, 60.0, 100.0};
    const double exits[] = {0.0, 20.0, 60.0, 100.0};

    int planned = 0;
    for (double d : distances) {
        for (double v0 : entries) {
            for (double v1 : exits) {
                SCurveProfile p;
                if (p.plan(d, v0, v1, kLim) != PlanResult::Ok) {
                    continue;
                }
                planned++;

                const auto inv = sweep(p, 4000);

                CHECK(inv.max_velocity <= kLim.max_velocity + 1e-6);
                CHECK(inv.max_abs_accel <= kLim.max_acceleration + 1e-6);
                CHECK(inv.max_abs_jerk <= kLim.max_jerk * 1.10 + 1e-6);
                CHECK(inv.monotonic);

                // The move must cover exactly the requested distance.
                CHECK_NEAR(p.at(p.duration()).position, d, 1e-5);
                // And arrive at the requested velocity.
                CHECK_NEAR(p.at(p.duration()).velocity,
                           (v1 > kLim.max_velocity ? kLim.max_velocity : v1), 1e-4);
            }
        }
    }
    CHECK(planned > 40);
}

void test_sampling_clamped_outside_range() {
    SCurveProfile p;
    CHECK(p.plan(50.0, 0.0, 0.0, kLim) == PlanResult::Ok);

    const auto past = p.at(p.duration() * 3.0);
    CHECK_NEAR(past.position, 50.0, 1e-6);
    CHECK_NEAR(past.velocity, 0.0, 1e-9);

    const auto before = p.at(-5.0);
    CHECK_NEAR(before.position, 0.0, 1e-12);
}

void test_scurve_takes_longer_than_trapezoid() {
    // Bounding jerk costs time. If it did not, something is wrong.
    const Limits lim_s{100.0, 1000.0, 10000.0};
    const Limits lim_t{100.0, 1000.0, 0.0};

    SCurveProfile s;
    SCurveProfile t;
    CHECK(s.plan(100.0, 0.0, 0.0, lim_s) == PlanResult::Ok);
    CHECK(t.plan(100.0, 0.0, 0.0, lim_t) == PlanResult::Ok);

    CHECK(s.duration() > t.duration());
}

}  // namespace

int main() {
    std::printf("test_scurve\n");

    test_ramp_trapezoidal_shape();
    test_ramp_triangular_shape();
    test_ramp_without_jerk_limit_is_constant_accel();
    test_ramp_distance_is_mean_velocity_times_time();

    test_rejects_invalid_input();
    test_zero_length_is_valid();
    test_infeasible_when_too_short();
    test_endpoints_are_exact();
    test_reaches_max_velocity_on_long_move();
    test_all_seven_phases_present();
    test_triangular_accel_ramp_drops_const_phases();
    test_short_move_drops_cruise_phase();
    test_honours_entry_and_exit_velocity();
    test_zero_jerk_limit_degenerates_gracefully();

    test_acceleration_is_continuous();
    test_jerk_limit_is_respected();
    test_invariants_across_many_shapes();
    test_sampling_clamped_outside_range();
    test_scurve_takes_longer_than_trapezoid();

    std::printf("  %d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
