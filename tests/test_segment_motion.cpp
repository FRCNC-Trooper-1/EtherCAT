// Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.
//
// Tests for per-axis command generation.
//
// The critical property is that the reported velocity and acceleration are the
// true derivatives of the reported position. If they are not, feedforward makes
// tracking WORSE than sending position alone — it would be pushing the axis in
// the wrong direction at exactly the moments that matter.
//
// The centripetal term gets particular attention: on an arc at constant speed
// the tangential acceleration is zero but the axis acceleration is not, and a
// naive implementation that only feeds forward tangential acceleration is
// silently wrong on every curve.

#include "frcnc/motion/segment_motion.hpp"

#include <cmath>
#include <cstdio>
#include <initializer_list>

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

constexpr double kPi = 3.14159265358979323846;

AxisLimits machine_limits() {
    AxisLimits l;
    for (int i = 0; i < 3; i++) {
        l[i] = Limits{/*v*/ 500.0, /*a*/ 5000.0, /*j*/ 50000.0};
    }
    return l;
}

PathConstraints constraints_at(double cycle_time) {
    PathConstraints c;
    c.chord_tolerance = 0.001;  // 1 micron
    c.cycle_time = cycle_time;
    c.max_centripetal_accel = 5000.0;
    return c;
}

// --- chord error ------------------------------------------------------------

void test_chord_error_formula() {
    // 167 mm/s, r = 10 mm, dt = 1 ms:  (0.167)^2 / 80 = 0.000349 mm
    const double e = chord_error(167.0, 10.0, 0.001);
    CHECK_NEAR(e, (0.167 * 0.167) / 80.0, 1e-9);

    // Quartering the cycle time reduces chord error 16x.
    const double e4 = chord_error(167.0, 10.0, 0.00025);
    CHECK_NEAR(e / e4, 16.0, 1e-6);
}

void test_chord_velocity_bound_is_inverse_of_error() {
    const double r = 5.0;
    const double tol = 0.001;
    const double dt = 0.00025;

    const double v = max_velocity_for_chord(r, tol, dt);
    // Running at exactly that speed must produce exactly the tolerance.
    CHECK_NEAR(chord_error(v, r, dt), tol, 1e-9);
}

void test_chord_velocity_bound_scales_inversely_with_cycle_time() {
    // The chord BOUND scales as 1/dt. Note this is the bound in isolation --
    // whether it translates into more feed depends on whether chord error is
    // the binding constraint, which on a real machine it usually is not.
    // See test_centripetal_binds_before_chord_on_a_real_machine().
    const double v_1ms = max_velocity_for_chord(1.0, 0.001, 0.001);
    const double v_250us = max_velocity_for_chord(1.0, 0.001, 0.00025);
    CHECK_NEAR(v_250us / v_1ms, 4.0, 1e-9);
}

// --- linear motion ----------------------------------------------------------

void test_linear_command_endpoints() {
    PathSegment seg;
    CHECK(make_linear(seg, Vec3{0, 0, 0}, Vec3{100, 0, 0}) == PathResult::Ok);

    SegmentMotion m;
    CHECK(m.plan(seg, 0.0, 0.0, machine_limits(), constraints_at(0.00025)) == PlanResult::Ok);

    const auto a = m.at(0.0);
    CHECK_NEAR(a.position.x, 0.0, 1e-9);
    CHECK_NEAR(a.velocity.x, 0.0, 1e-9);

    const auto b = m.at(m.duration());
    CHECK_NEAR(b.position.x, 100.0, 1e-6);
    CHECK_NEAR(b.velocity.x, 0.0, 1e-6);
    CHECK(b.finished);
}

void test_linear_velocity_is_tangential_only() {
    // A pure X move must produce zero Y and Z velocity at every sample.
    PathSegment seg;
    CHECK(make_linear(seg, Vec3{0, 0, 0}, Vec3{50, 0, 0}) == PathResult::Ok);

    SegmentMotion m;
    CHECK(m.plan(seg, 0.0, 0.0, machine_limits(), constraints_at(0.00025)) == PlanResult::Ok);

    for (int i = 0; i <= 200; i++) {
        const auto c = m.at(m.duration() * i / 200.0);
        CHECK_NEAR(c.velocity.y, 0.0, 1e-12);
        CHECK_NEAR(c.velocity.z, 0.0, 1e-12);
        CHECK_NEAR(c.acceleration.y, 0.0, 1e-12);
    }
}

void test_diagonal_axis_split() {
    // 3-4-5 in XY: velocity must split 0.6 / 0.8 of path speed.
    PathSegment seg;
    CHECK(make_linear(seg, Vec3{0, 0, 0}, Vec3{30, 40, 0}) == PathResult::Ok);

    SegmentMotion m;
    CHECK(m.plan(seg, 0.0, 0.0, machine_limits(), constraints_at(0.00025)) == PlanResult::Ok);

    const auto c = m.at(m.duration() * 0.5);
    CHECK_NEAR(c.velocity.x, 0.6 * c.path_velocity, 1e-9);
    CHECK_NEAR(c.velocity.y, 0.8 * c.path_velocity, 1e-9);
}

// --- derivative consistency, the property feedforward depends on ------------

struct DerivCheck {
    double worst_velocity_error = 0.0;
    double worst_accel_error = 0.0;
    double h = 0.0;
};

/// Upper bound on path jerk for these tests.
///
/// Path jerk is min_i(axis_jerk_i / |tangent_i|) over the moving axes, so it is
/// never below the axis limit and at most axis_jerk * sqrt(3) for a 3D diagonal.
constexpr double kPathJerkBound = 50000.0 * 1.7320508 + 1.0;

/// Central differencing of a piecewise-polynomial profile is exact within a
/// phase but not across a phase boundary, where jerk steps. The error there is
/// O(jerk * h^2) for a first derivative and O(jerk * h) for a second. Fixed
/// tolerances would be meaningless — they must scale with the discretisation.
double velocity_tolerance(double h) {
    return 8.0 * kPathJerkBound * h * h + 1e-9;
}

double accel_tolerance(double h) {
    return 2.0 * kPathJerkBound * h + 1e-6;
}

/// Central-difference the sampled position and velocity and compare against the
/// reported velocity and acceleration.
DerivCheck verify_derivatives(const SegmentMotion& m, int steps) {
    DerivCheck out;
    const double T = m.duration();
    const double h = T / (steps * 4);  // small offset for differencing
    out.h = h;

    for (int i = 1; i < steps; i++) {
        const double t = T * i / steps;
        if (t - h <= 0.0 || t + h >= T) {
            continue;
        }

        const auto lo = m.at(t - h);
        const auto mid = m.at(t);
        const auto hi = m.at(t + h);

        for (int ax = 0; ax < 3; ax++) {
            const double dv = (hi.position[ax] - lo.position[ax]) / (2.0 * h);
            const double err_v = std::fabs(dv - mid.velocity[ax]);
            if (err_v > out.worst_velocity_error) {
                out.worst_velocity_error = err_v;
            }

            const double da = (hi.velocity[ax] - lo.velocity[ax]) / (2.0 * h);
            const double err_a = std::fabs(da - mid.acceleration[ax]);
            if (err_a > out.worst_accel_error) {
                out.worst_accel_error = err_a;
            }
        }
    }
    return out;
}

void test_linear_derivatives_are_consistent() {
    PathSegment seg;
    CHECK(make_linear(seg, Vec3{0, 0, 0}, Vec3{100, 50, 25}) == PathResult::Ok);

    SegmentMotion m;
    CHECK(m.plan(seg, 0.0, 0.0, machine_limits(), constraints_at(0.00025)) == PlanResult::Ok);

    const auto d = verify_derivatives(m, 400);
    CHECK(d.worst_velocity_error < velocity_tolerance(d.h));
    CHECK(d.worst_accel_error < accel_tolerance(d.h));
}

void test_arc_derivatives_are_consistent() {
    // THE test. If the centripetal term were missing, the reported acceleration
    // would not be the derivative of the reported velocity and this fails.
    PathSegment seg;
    CHECK(make_arc(seg, Vec3{20, 0, 0}, Vec3{-20, 0, 0}, Vec3{0, 0, 0}, Plane::XY, false) ==
          PathResult::Ok);

    SegmentMotion m;
    CHECK(m.plan(seg, 0.0, 0.0, machine_limits(), constraints_at(0.00025)) == PlanResult::Ok);

    const auto d = verify_derivatives(m, 800);
    CHECK(d.worst_velocity_error < velocity_tolerance(d.h));
    CHECK(d.worst_accel_error < accel_tolerance(d.h));
}

void test_helical_derivatives_are_consistent() {
    PathSegment seg;
    CHECK(make_arc(seg, Vec3{15, 0, 0}, Vec3{0, 15, 10}, Vec3{0, 0, 0}, Plane::XY, false) ==
          PathResult::Ok);

    SegmentMotion m;
    CHECK(m.plan(seg, 0.0, 0.0, machine_limits(), constraints_at(0.00025)) == PlanResult::Ok);

    const auto d = verify_derivatives(m, 800);
    CHECK(d.worst_velocity_error < velocity_tolerance(d.h));
    CHECK(d.worst_accel_error < accel_tolerance(d.h));
}

// --- centripetal acceleration ----------------------------------------------

void test_centripetal_term_is_present_on_arcs() {
    // At constant speed on an arc, tangential acceleration is ~0 but axis
    // acceleration must be v^2/r directed at the centre.
    PathSegment seg;
    CHECK(make_arc(seg, Vec3{10, 0, 0}, Vec3{-10, 0, 0}, Vec3{0, 0, 0}, Plane::XY, false) ==
          PathResult::Ok);

    SegmentMotion m;
    CHECK(m.plan(seg, 0.0, 0.0, machine_limits(), constraints_at(0.00025)) == PlanResult::Ok);

    // Mid-arc, where the profile is cruising.
    const auto c = m.at(m.duration() * 0.5);

    const double a_mag = norm(c.acceleration);
    const double expected_centripetal = (c.path_velocity * c.path_velocity) / 10.0;

    CHECK(a_mag > 0.0);
    // Tangential component is near zero at cruise, so the magnitude should be
    // dominated by the centripetal term.
    CHECK_NEAR(a_mag, expected_centripetal, expected_centripetal * 0.05 + 1e-6);

    // And it must point at the centre: the acceleration dotted with the outward
    // radial direction should be negative.
    const Vec3 radial{c.position.x, c.position.y, 0.0};
    CHECK(dot(radial, c.acceleration) < 0.0);
}

void test_zero_curvature_on_line_means_no_centripetal_term() {
    PathSegment seg;
    CHECK(make_linear(seg, Vec3{0, 0, 0}, Vec3{100, 0, 0}) == PathResult::Ok);
    const Vec3 d2 = seg.second_derivative_at(50.0);
    CHECK_NEAR(norm(d2), 0.0, 1e-15);
}

void test_arc_curvature_magnitude_is_one_over_radius() {
    for (double r : {1.0, 5.0, 25.0, 100.0}) {
        PathSegment seg;
        CHECK(make_arc(seg, Vec3{r, 0, 0}, Vec3{-r, 0, 0}, Vec3{0, 0, 0}, Plane::XY, false) ==
              PathResult::Ok);
        const Vec3 d2 = seg.second_derivative_at(seg.length() * 0.5);
        CHECK_NEAR(norm(d2), 1.0 / r, 1e-9);
    }
}

// --- feed limiting ----------------------------------------------------------

void test_requested_feed_can_only_lower_the_limit() {
    PathSegment seg;
    CHECK(make_linear(seg, Vec3{0, 0, 0}, Vec3{500, 0, 0}) == PathResult::Ok);

    SegmentMotion fast;
    CHECK(fast.plan(seg, 0.0, 0.0, machine_limits(), constraints_at(0.00025), 0.0) ==
          PlanResult::Ok);

    SegmentMotion slow;
    CHECK(slow.plan(seg, 0.0, 0.0, machine_limits(), constraints_at(0.00025), 50.0) ==
          PlanResult::Ok);

    CHECK_NEAR(slow.feed_limit(), 50.0, 1e-9);
    CHECK(slow.duration() > fast.duration());

    // A programmed feed above the machine limit must not raise it.
    SegmentMotion over;
    CHECK(over.plan(seg, 0.0, 0.0, machine_limits(), constraints_at(0.00025), 99999.0) ==
          PlanResult::Ok);
    CHECK(over.feed_limit() <= 500.0 + 1e-9);
}

void test_tight_arc_feed_is_limited_and_respects_chord_tolerance() {
    // A 0.5 mm radius arc at 250 us with a 1 micron tolerance.
    PathSegment seg;
    CHECK(make_arc(seg, Vec3{0.5, 0, 0}, Vec3{-0.5, 0, 0}, Vec3{0, 0, 0}, Plane::XY, false) ==
          PathResult::Ok);

    const auto c = constraints_at(0.00025);
    SegmentMotion m;
    CHECK(m.plan(seg, 0.0, 0.0, machine_limits(), c) == PlanResult::Ok);

    // Whatever speed was chosen, the resulting chord error must be within
    // tolerance at every sample.
    for (int i = 0; i <= 200; i++) {
        const auto s = m.at(m.duration() * i / 200.0);
        const double e = chord_error(s.path_velocity, seg.radius, c.cycle_time);
        CHECK(e <= c.chord_tolerance * 1.001 + 1e-12);
    }
}

void test_centripetal_binds_before_chord_on_a_real_machine() {
    // Counter-intuitive but important: with realistic acceleration and a
    // micron-scale tolerance, CENTRIPETAL acceleration is the binding
    // constraint on arcs at every radius -- not chord error.
    //
    // Chord binds only when dt > sqrt(8*tol/a). With tol = 1 um and
    // a = 5000 mm/s^2 that is 1.265 ms, so at 1 ms or below it never binds.
    const double a = 5000.0;
    const double tol = 0.001;
    const double crossover = std::sqrt(8.0 * tol / a);
    CHECK(crossover > 0.001);   // above a 1 ms cycle

    for (double r : {0.5, 1.0, 2.0, 10.0, 50.0}) {
        PathSegment seg;
        CHECK(make_arc(seg, Vec3{r, 0, 0}, Vec3{-r, 0, 0}, Vec3{0, 0, 0}, Plane::XY, false) ==
              PathResult::Ok);

        SegmentMotion at_1ms;
        SegmentMotion at_250us;
        CHECK(at_1ms.plan(seg, 0.0, 0.0, machine_limits(), constraints_at(0.001)) ==
              PlanResult::Ok);
        CHECK(at_250us.plan(seg, 0.0, 0.0, machine_limits(), constraints_at(0.00025)) ==
              PlanResult::Ok);

        // Same feed at both cycle times, because acceleration is the wall.
        CHECK_NEAR(at_250us.feed_limit(), at_1ms.feed_limit(), 1e-6);
        CHECK_NEAR(at_1ms.feed_limit(), std::sqrt(a * r), 1e-6);
    }
}

void test_shorter_cycle_buys_accuracy_not_feed() {
    // What the faster cycle actually delivers. Running at the centripetal
    // limit v = sqrt(a*r), the chord error is
    //     (v*dt)^2 / (8r) = a*dt^2 / 8
    // -- independent of radius, and quadratic in cycle time.
    const double a = 5000.0;

    for (double r : {0.5, 2.0, 50.0}) {
        const double v = std::sqrt(a * r);
        const double e_1ms = chord_error(v, r, 0.001);
        const double e_250us = chord_error(v, r, 0.00025);

        // Independent of radius.
        CHECK_NEAR(e_1ms, a * 0.001 * 0.001 / 8.0, 1e-12);
        // Quartering the cycle reduces error 16x.
        CHECK_NEAR(e_1ms / e_250us, 16.0, 1e-9);
    }
}

void test_shorter_cycle_raises_the_usable_acceleration_ceiling() {
    // The indirect route to more feed. Holding chord error at tolerance,
    //     a_max = 8 * tol / dt^2
    // so a shorter cycle lifts the acceleration a machine may usefully employ,
    // and higher acceleration is what actually raises arc feed.
    const double tol = 0.001;
    const double a_at_1ms = 8.0 * tol / (0.001 * 0.001);
    const double a_at_250us = 8.0 * tol / (0.00025 * 0.00025);

    CHECK_NEAR(a_at_1ms, 8000.0, 1e-6);
    CHECK_NEAR(a_at_250us, 128000.0, 1e-6);
    CHECK_NEAR(a_at_250us / a_at_1ms, 16.0, 1e-9);

    // At 1 ms, chord error caps usable acceleration at a value a good servo
    // machine can exceed. At 250 us the cap is far beyond any real machine,
    // so chord error stops being a design constraint at all.
    CHECK(a_at_1ms < 20000.0);
    CHECK(a_at_250us > 100000.0);
}

void test_feed_does_increase_when_chord_actually_binds() {
    // Past the crossover -- long cycle times -- chord error is the binding
    // constraint and halving the cycle does double the feed.
    PathSegment seg;
    CHECK(make_arc(seg, Vec3{2, 0, 0}, Vec3{-2, 0, 0}, Vec3{0, 0, 0}, Plane::XY, false) ==
          PathResult::Ok);

    SegmentMotion at_4ms;
    SegmentMotion at_2ms;
    CHECK(at_4ms.plan(seg, 0.0, 0.0, machine_limits(), constraints_at(0.004)) == PlanResult::Ok);
    CHECK(at_2ms.plan(seg, 0.0, 0.0, machine_limits(), constraints_at(0.002)) == PlanResult::Ok);

    CHECK(at_2ms.feed_limit() > at_4ms.feed_limit());
    CHECK_NEAR(at_2ms.feed_limit() / at_4ms.feed_limit(), 2.0, 1e-6);
}

void test_zero_length_segment_is_handled() {
    PathSegment seg;
    CHECK(make_linear(seg, Vec3{5, 5, 5}, Vec3{5, 5, 5}) == PathResult::ZeroLength);

    SegmentMotion m;
    CHECK(m.plan(seg, 0.0, 0.0, machine_limits(), constraints_at(0.00025)) ==
          PlanResult::ZeroLength);

    const auto c = m.at(0.0);
    CHECK_NEAR(c.position.x, 5.0, 1e-12);
    CHECK(c.finished);
}

}  // namespace

int main() {
    std::printf("test_segment_motion\n");

    test_chord_error_formula();
    test_chord_velocity_bound_is_inverse_of_error();
    test_chord_velocity_bound_scales_inversely_with_cycle_time();

    test_linear_command_endpoints();
    test_linear_velocity_is_tangential_only();
    test_diagonal_axis_split();

    test_linear_derivatives_are_consistent();
    test_arc_derivatives_are_consistent();
    test_helical_derivatives_are_consistent();

    test_centripetal_term_is_present_on_arcs();
    test_zero_curvature_on_line_means_no_centripetal_term();
    test_arc_curvature_magnitude_is_one_over_radius();

    test_requested_feed_can_only_lower_the_limit();
    test_tight_arc_feed_is_limited_and_respects_chord_tolerance();
    test_centripetal_binds_before_chord_on_a_real_machine();
    test_shorter_cycle_buys_accuracy_not_feed();
    test_shorter_cycle_raises_the_usable_acceleration_ceiling();
    test_feed_does_increase_when_chord_actually_binds();
    test_zero_length_segment_is_handled();

    std::printf("  %d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
