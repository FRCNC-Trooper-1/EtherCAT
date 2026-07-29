// Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.
//
// Tests for the trapezoidal motion profile.
//
// Beyond the obvious cases, these check the physical invariants that matter on
// a real machine: velocity never exceeds the limit, acceleration never exceeds
// the limit, position is monotonic, and numerically integrating the sampled
// velocity reproduces the sampled position. A profile that violates any of
// those will fault a drive or cut a wrong part.

#include "frcnc/motion/profile.hpp"

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

constexpr Limits kLim{/*v*/ 100.0, /*a*/ 1000.0, /*j*/ 0.0};

// --- basic planning ---------------------------------------------------------

void test_rejects_invalid_input() {
    TrapezoidalProfile p;
    CHECK(p.plan(-1.0, 0.0, 0.0, kLim) == PlanResult::NegativeDistance);
    CHECK(p.plan(10.0, 0.0, 0.0, Limits{0.0, 1000.0, 0.0}) == PlanResult::InvalidLimits);
    CHECK(p.plan(10.0, 0.0, 0.0, Limits{100.0, 0.0, 0.0}) == PlanResult::InvalidLimits);
}

void test_zero_length_is_valid_not_an_error() {
    // Look-ahead legitimately produces zero-length segments at repeated points.
    TrapezoidalProfile p;
    CHECK(p.plan(0.0, 0.0, 0.0, kLim) == PlanResult::ZeroLength);
    CHECK(p.valid());
    CHECK_NEAR(p.duration(), 0.0, 1e-12);

    const auto s = p.at(0.0);
    CHECK_NEAR(s.velocity, 0.0, 1e-12);
}

void test_infeasible_when_distance_too_short() {
    // Decelerating from 100 to 0 at 1000 needs 5 units. Ask for it in 1.
    TrapezoidalProfile p;
    CHECK(p.plan(1.0, 100.0, 0.0, kLim) == PlanResult::Infeasible);
}

// --- trapezoid --------------------------------------------------------------

void test_full_trapezoid_reaches_max_velocity() {
    TrapezoidalProfile p;
    CHECK(p.plan(100.0, 0.0, 0.0, kLim) == PlanResult::Ok);
    CHECK_NEAR(p.peak_velocity(), 100.0, 1e-9);
    CHECK(p.cruise_time() > 0.0);

    // accel: 0->100 at 1000 = 0.1 s, covering 5 units. Same to decelerate.
    CHECK_NEAR(p.accel_time(), 0.1, 1e-9);
    CHECK_NEAR(p.decel_time(), 0.1, 1e-9);
    // cruise: remaining 90 units at 100 = 0.9 s
    CHECK_NEAR(p.cruise_time(), 0.9, 1e-9);
    CHECK_NEAR(p.duration(), 1.1, 1e-9);
}

void test_endpoints_are_exact() {
    TrapezoidalProfile p;
    CHECK(p.plan(100.0, 0.0, 0.0, kLim) == PlanResult::Ok);

    const auto start = p.at(0.0);
    CHECK_NEAR(start.position, 0.0, 1e-12);
    CHECK_NEAR(start.velocity, 0.0, 1e-12);

    const auto end = p.at(p.duration());
    CHECK_NEAR(end.position, 100.0, 1e-9);
    CHECK_NEAR(end.velocity, 0.0, 1e-9);
}

void test_triangular_when_distance_too_short_to_cruise() {
    // 1 unit at a=1000: peak = sqrt(2*1000*0.5) = ~31.6, well under v_max.
    TrapezoidalProfile p;
    CHECK(p.plan(1.0, 0.0, 0.0, kLim) == PlanResult::Ok);
    CHECK(p.peak_velocity() < kLim.max_velocity);
    CHECK_NEAR(p.cruise_time(), 0.0, 1e-9);
    CHECK_NEAR(p.peak_velocity(), std::sqrt(1000.0 * 1.0), 1e-9);

    const auto end = p.at(p.duration());
    CHECK_NEAR(end.position, 1.0, 1e-9);
}

// --- entry / exit velocities (what look-ahead produces) ---------------------

void test_honours_entry_and_exit_velocity() {
    TrapezoidalProfile p;
    CHECK(p.plan(100.0, 40.0, 60.0, kLim) == PlanResult::Ok);

    const auto start = p.at(0.0);
    CHECK_NEAR(start.velocity, 40.0, 1e-9);

    const auto end = p.at(p.duration());
    CHECK_NEAR(end.velocity, 60.0, 1e-9);
    CHECK_NEAR(end.position, 100.0, 1e-9);
}

void test_pure_acceleration_segment() {
    // Entry 0, exit at max, distance exactly the accel distance: no decel phase.
    const double d = (100.0 * 100.0) / (2.0 * 1000.0);  // 5 units
    TrapezoidalProfile p;
    CHECK(p.plan(d, 0.0, 100.0, kLim) == PlanResult::Ok);
    CHECK_NEAR(p.decel_time(), 0.0, 1e-9);
    CHECK_NEAR(p.at(p.duration()).velocity, 100.0, 1e-6);
}

void test_entry_velocity_is_clamped_to_limit() {
    TrapezoidalProfile p;
    CHECK(p.plan(100.0, 500.0, 0.0, kLim) == PlanResult::Ok);
    CHECK(p.at(0.0).velocity <= kLim.max_velocity + 1e-9);
}

// --- physical invariants ----------------------------------------------------

struct Invariants {
    double max_velocity = 0.0;
    double max_abs_accel = 0.0;
    bool monotonic = true;
    double integrated_error = 0.0;
    double dt = 0.0;
};

/// Numerically integrating sampled velocity should reproduce sampled position.
///
/// The trapezoid rule is EXACT while acceleration is constant, so error only
/// accrues in the one interval that straddles each phase boundary, bounded by
/// O(a*dt^2). Measured ratio is ~0.24; the tolerance below allows 4x that.
/// A fixed tolerance is wrong here — the bound grows with move duration.
double integration_tolerance(double dt, double max_accel) {
    return 1.0 * max_accel * dt * dt + 1e-12;
}

Invariants sweep(const TrapezoidalProfile& p, int steps = 20000) {
    Invariants inv;
    const double dt = p.duration() / steps;
    inv.dt = dt;

    double prev_pos = -1e18;
    double integrated = 0.0;
    double prev_vel = p.at(0.0).velocity;

    for (int i = 0; i <= steps; i++) {
        const double t = dt * i;
        const auto s = p.at(t);

        if (s.velocity > inv.max_velocity) {
            inv.max_velocity = s.velocity;
        }
        if (std::fabs(s.acceleration) > inv.max_abs_accel) {
            inv.max_abs_accel = std::fabs(s.acceleration);
        }
        if (s.position < prev_pos - 1e-9) {
            inv.monotonic = false;
        }
        prev_pos = s.position;

        if (i > 0) {
            // Trapezoidal integration of velocity should reproduce position.
            integrated += 0.5 * (prev_vel + s.velocity) * dt;
            const double err = std::fabs(integrated - s.position);
            if (err > inv.integrated_error) {
                inv.integrated_error = err;
            }
        }
        prev_vel = s.velocity;
    }
    return inv;
}

void test_invariants_across_many_shapes() {
    const double distances[] = {0.001, 0.1, 1.0, 5.0, 10.0, 100.0, 1000.0};
    const double entries[] = {0.0, 10.0, 50.0, 100.0};
    const double exits[] = {0.0, 10.0, 50.0, 100.0};

    int planned = 0;
    for (double d : distances) {
        for (double v0 : entries) {
            for (double v1 : exits) {
                TrapezoidalProfile p;
                const auto r = p.plan(d, v0, v1, kLim);
                if (r != PlanResult::Ok) {
                    continue;  // Infeasible combinations are legitimately rejected.
                }
                planned++;

                const auto inv = sweep(p, 4000);

                // Never exceed the velocity limit.
                CHECK(inv.max_velocity <= kLim.max_velocity + 1e-6);
                // Never exceed the acceleration limit.
                CHECK(inv.max_abs_accel <= kLim.max_acceleration + 1e-6);
                // Position must not go backwards on a forward move.
                CHECK(inv.monotonic);
                // Integrated velocity must track position, within the bound
                // implied by the discretisation.
                CHECK(inv.integrated_error <
                      integration_tolerance(inv.dt, kLim.max_acceleration));
                // The move must actually cover the distance.
                CHECK_NEAR(p.at(p.duration()).position, d, 1e-6);
            }
        }
    }
    CHECK(planned > 50);  // ensure the sweep actually exercised something
}

void test_sampling_is_clamped_outside_range() {
    TrapezoidalProfile p;
    CHECK(p.plan(10.0, 0.0, 0.0, kLim) == PlanResult::Ok);

    // A cyclic task that overruns by a cycle must get the endpoint, not a
    // position beyond it.
    const auto past = p.at(p.duration() * 2.0);
    CHECK_NEAR(past.position, 10.0, 1e-9);
    CHECK_NEAR(past.velocity, 0.0, 1e-9);

    const auto before = p.at(-1.0);
    CHECK_NEAR(before.position, 0.0, 1e-12);
}

// --- look-ahead helpers -----------------------------------------------------

void test_min_distance_for_velocity_change() {
    // 0 -> 100 at a=1000 needs v^2/(2a) = 5
    CHECK_NEAR(min_distance_for_velocity_change(0.0, 100.0, 1000.0), 5.0, 1e-9);
    // Symmetric: decelerating needs the same distance.
    CHECK_NEAR(min_distance_for_velocity_change(100.0, 0.0, 1000.0), 5.0, 1e-9);
    // No change needs no distance.
    CHECK_NEAR(min_distance_for_velocity_change(50.0, 50.0, 1000.0), 0.0, 1e-12);
}

void test_max_reachable_velocity() {
    // From rest over 5 units at a=1000: sqrt(2*1000*5) = 100
    CHECK_NEAR(max_reachable_velocity(0.0, 5.0, 1000.0), 100.0, 1e-9);
    CHECK_NEAR(max_reachable_velocity(0.0, 0.0, 1000.0), 0.0, 1e-12);
}

void test_helpers_agree_with_profile() {
    // A profile planned over exactly the minimum distance must be feasible,
    // and one over slightly less must not be.
    const double d_min = min_distance_for_velocity_change(100.0, 0.0, kLim.max_acceleration);

    TrapezoidalProfile p;
    CHECK(p.plan(d_min, 100.0, 0.0, kLim) == PlanResult::Ok);
    CHECK(p.plan(d_min * 0.5, 100.0, 0.0, kLim) == PlanResult::Infeasible);
}

}  // namespace

int main() {
    std::printf("test_profile\n");

    test_rejects_invalid_input();
    test_zero_length_is_valid_not_an_error();
    test_infeasible_when_distance_too_short();
    test_full_trapezoid_reaches_max_velocity();
    test_endpoints_are_exact();
    test_triangular_when_distance_too_short_to_cruise();
    test_honours_entry_and_exit_velocity();
    test_pure_acceleration_segment();
    test_entry_velocity_is_clamped_to_limit();
    test_invariants_across_many_shapes();
    test_sampling_is_clamped_outside_range();
    test_min_distance_for_velocity_change();
    test_max_reachable_velocity();
    test_helpers_agree_with_profile();

    std::printf("  %d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
