// Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.
//
// Tests for look-ahead velocity planning.
//
// The purpose of look-ahead is to avoid stopping at every block boundary, so
// the headline test is exactly that: a program of many short collinear moves
// must reach commanded feed and finish in roughly the time a single long move
// would take. If it does not, look-ahead is not working regardless of what the
// unit tests say.
//
// The safety-critical property is the opposite one: velocity must be
// CONTINUOUS across every junction. If segment i exits at a speed segment i+1
// cannot be entered with, the machine is commanded a velocity step it cannot
// physically make, and the drives fault.

#include "frcnc/motion/lookahead.hpp"

#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <memory>

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

AxisLimits machine_limits() {
    AxisLimits l;
    for (int i = 0; i < 3; i++) {
        l[i] = Limits{/*v*/ 200.0, /*a*/ 3000.0, /*j*/ 30000.0};
    }
    return l;
}

PathConstraints constraints() {
    PathConstraints c;
    c.chord_tolerance = 0.001;
    c.cycle_time = 0.00025;
    c.max_centripetal_accel = 3000.0;
    return c;
}

JunctionPolicy policy(double deviation = 0.01) {
    JunctionPolicy p;
    p.deviation = deviation;
    return p;
}

// LookAhead is large; keep it off the stack.
std::unique_ptr<LookAhead> make_planner(double deviation = 0.01) {
    auto la = std::make_unique<LookAhead>();
    la->configure(machine_limits(), constraints(), policy(deviation));
    return la;
}

PathSegment line(const Vec3& a, const Vec3& b) {
    PathSegment s;
    (void)make_linear(s, a, b);
    return s;
}

/// A planner with merging disabled, for tests where segment identity matters.
std::unique_ptr<LookAhead> make_planner_no_merge() {
    auto la = std::make_unique<LookAhead>();
    JunctionPolicy p = policy();
    p.merge_collinear = false;
    la->configure(machine_limits(), constraints(), p);
    return la;
}

/// Zig-zag chain: consecutive segments are never collinear, so nothing merges.
PathSegment zigzag(int i, double len) {
    const double x = i * len;
    const double y = (i % 2 == 0) ? 0.0 : len * 0.05;
    const double y2 = (i % 2 == 0) ? len * 0.05 : 0.0;
    return line(Vec3{x, y, 0}, Vec3{x + len, y2, 0});
}

// --- reachable velocity with jerk -------------------------------------------

void test_jerk_limited_reachable_is_below_constant_accel() {
    const double v0 = 0.0;
    const double d = 10.0;
    const double a = 3000.0;

    const double no_jerk = max_reachable_velocity_jerk(v0, d, a, 0.0);
    const double with_jerk = max_reachable_velocity_jerk(v0, d, a, 30000.0);

    CHECK_NEAR(no_jerk, std::sqrt(2.0 * a * d), 1e-6);
    // Bounding jerk costs distance, so less velocity is reachable.
    CHECK(with_jerk < no_jerk);
    CHECK(with_jerk > 0.0);
}

void test_reachable_velocity_is_consistent_with_ramp_distance() {
    // The velocity returned must be exactly what fits in the distance.
    const double v0 = 20.0;
    const double d = 5.0;
    const double a = 3000.0;
    const double j = 30000.0;

    const double v = max_reachable_velocity_jerk(v0, d, a, j);
    CHECK(ramp_distance(v0, v, a, j) <= d + 1e-6);
}

// --- junction velocity ------------------------------------------------------

void test_collinear_junction_is_unrestricted() {
    const PathSegment a = line(Vec3{0, 0, 0}, Vec3{10, 0, 0});
    const PathSegment b = line(Vec3{10, 0, 0}, Vec3{20, 0, 0});
    const double v = junction_velocity(a, b, 3000.0, policy());
    CHECK(v > 1e6);  // effectively unlimited
}

void test_reversal_junction_forces_a_stop() {
    const PathSegment a = line(Vec3{0, 0, 0}, Vec3{10, 0, 0});
    const PathSegment b = line(Vec3{10, 0, 0}, Vec3{0, 0, 0});
    CHECK_NEAR(junction_velocity(a, b, 3000.0, policy()), 0.0, 1e-9);
}

void test_right_angle_junction_is_limited() {
    const PathSegment a = line(Vec3{0, 0, 0}, Vec3{10, 0, 0});
    const PathSegment b = line(Vec3{10, 0, 0}, Vec3{10, 10, 0});

    const double v = junction_velocity(a, b, 3000.0, policy(0.01));
    CHECK(v > 0.0);
    CHECK(v < 200.0);  // well below the axis limit

    // sin(theta/2) = cos(45 deg) = 0.7071; r = dev * h / (1 - h)
    const double h = std::sqrt(0.5);
    const double r = 0.01 * h / (1.0 - h);
    CHECK_NEAR(v, std::sqrt(3000.0 * r), 1e-6);
}

void test_sharper_corner_means_lower_junction_speed() {
    const PathSegment a = line(Vec3{0, 0, 0}, Vec3{10, 0, 0});

    double prev = 1e300;
    // Increasingly sharp turns: 30, 60, 90, 135 degrees away from straight.
    for (double deg : {30.0, 60.0, 90.0, 135.0}) {
        const double rad = deg * 3.14159265358979323846 / 180.0;
        const PathSegment b =
            line(Vec3{10, 0, 0}, Vec3{10 + 10 * std::cos(rad), 10 * std::sin(rad), 0});
        const double v = junction_velocity(a, b, 3000.0, policy());
        CHECK(v < prev);
        prev = v;
    }
}

void test_larger_deviation_allows_faster_corners() {
    const PathSegment a = line(Vec3{0, 0, 0}, Vec3{10, 0, 0});
    const PathSegment b = line(Vec3{10, 0, 0}, Vec3{10, 10, 0});

    double prev = 0.0;
    for (double dev : {0.001, 0.01, 0.1, 1.0}) {
        const double v = junction_velocity(a, b, 3000.0, policy(dev));
        CHECK(v > prev);  // looser tolerance, faster corner
        prev = v;
    }
}

void test_axis_velocity_jump_cap_is_applied() {
    const PathSegment a = line(Vec3{0, 0, 0}, Vec3{10, 0, 0});
    const PathSegment b = line(Vec3{10, 0, 0}, Vec3{10, 10, 0});

    JunctionPolicy p;
    p.deviation = 10.0;                // effectively disabled
    p.max_axis_velocity_jump = 5.0;    // the binding constraint

    // Tangents are (1,0,0) then (0,1,0): worst per-axis change is 1.0,
    // so the bound is 5.0 / 1.0.
    CHECK_NEAR(junction_velocity(a, b, 3000.0, p), 5.0, 1e-9);
}

// --- queue behaviour --------------------------------------------------------

void test_push_rejects_degenerate_and_respects_capacity() {
    auto la = make_planner();
    CHECK(la->empty());

    CHECK(!la->push(line(Vec3{1, 1, 1}, Vec3{1, 1, 1})));  // zero length
    CHECK(la->empty());

    CHECK(la->push(line(Vec3{0, 0, 0}, Vec3{1, 0, 0})));
    CHECK(la->size() == 1);

    la->clear();
    CHECK(la->empty());

    for (int i = 0; i < LookAhead::kCapacity; i++) {
        CHECK(la->push(zigzag(i, 1.0)));
    }
    CHECK(la->full());
    CHECK(!la->push(line(Vec3{999, 0, 0}, Vec3{999, 1000, 0})));
}

void test_velocity_is_continuous_across_junctions() {
    // The safety property: segment i's exit must equal segment i+1's entry.
    auto la = make_planner();
    for (int i = 0; i < 20; i++) {
        CHECK(la->push(zigzag(i, 5.0)));
    }
    CHECK(la->plan(0.0, 0.0));
    CHECK(la->size() == 20);  // zig-zag must not merge

    for (int i = 0; i + 1 < la->size(); i++) {
        CHECK_NEAR(la->exit_velocity(i), la->entry_velocity(i + 1), 1e-9);
    }

    // Starts and ends at rest.
    CHECK_NEAR(la->entry_velocity(0), 0.0, 1e-9);
    CHECK_NEAR(la->exit_velocity(la->size() - 1), 0.0, 1e-9);
}

void test_sampled_velocity_has_no_step_at_boundaries() {
    auto la = make_planner();
    for (int i = 0; i < 12; i++) {
        const double x = i * 4.0;
        CHECK(la->push(line(Vec3{x, 0, 0}, Vec3{x + 4.0, 0, 0})));
    }
    CHECK(la->plan(0.0, 0.0));

    const double T = la->total_duration();
    const int steps = 20000;
    double worst_step = 0.0;
    double prev = la->at(0.0).path_velocity;

    for (int i = 1; i <= steps; i++) {
        const double v = la->at(T * i / steps).path_velocity;
        const double d = std::fabs(v - prev);
        if (d > worst_step) {
            worst_step = d;
        }
        prev = v;
    }

    // With acceleration bounded at 3000 and dt = T/steps, the largest legitimate
    // change per sample is a*dt with margin.
    const double dt = T / steps;
    CHECK(worst_step < 3000.0 * dt * 4.0 + 1e-6);
}

// --- the headline behaviour -------------------------------------------------

void test_many_short_moves_reach_commanded_feed() {
    // THE point of look-ahead. 200 collinear 1 mm moves should behave almost
    // exactly like one 200 mm move.
    auto with_la = make_planner();
    for (int i = 0; i < 200; i++) {
        const double x = static_cast<double>(i);
        CHECK(with_la->push(line(Vec3{x, 0, 0}, Vec3{x + 1.0, 0, 0})));
    }
    CHECK(with_la->plan(0.0, 0.0));

    // The middle of the program must be at full commanded feed.
    double peak = 0.0;
    for (int i = 0; i < with_la->size(); i++) {
        if (with_la->motion(i).peak_velocity() > peak) {
            peak = with_la->motion(i).peak_velocity();
        }
    }
    CHECK_NEAR(peak, 200.0, 1e-3);  // the axis velocity limit

    // And the total time should be close to a single equivalent long move.
    auto single = make_planner();
    CHECK(single->push(line(Vec3{0, 0, 0}, Vec3{200, 0, 0})));
    CHECK(single->plan(0.0, 0.0));

    CHECK_NEAR(with_la->total_duration(), single->total_duration(), 1e-3);
    CHECK_NEAR(with_la->total_length(), single->total_length(), 1e-9);
}

void test_without_lookahead_the_same_program_is_far_slower() {
    // Planning each segment in isolation (entry = exit = 0) is what a naive
    // controller does. Quantify the penalty so the value is explicit.
    const int n = 200;

    auto with_la = make_planner();
    for (int i = 0; i < n; i++) {
        const double x = static_cast<double>(i);
        CHECK(with_la->push(line(Vec3{x, 0, 0}, Vec3{x + 1.0, 0, 0})));
    }
    CHECK(with_la->plan(0.0, 0.0));

    double naive_time = 0.0;
    for (int i = 0; i < n; i++) {
        SegmentMotion m;
        const PathSegment s = line(Vec3{0, 0, 0}, Vec3{1, 0, 0});
        CHECK(m.plan(s, 0.0, 0.0, machine_limits(), constraints()) == PlanResult::Ok);
        naive_time += m.duration();
    }

    CHECK(naive_time > with_la->total_duration() * 3.0);
}

void test_deceleration_propagates_backwards_through_segments() {
    // A stop at the end must slow down several preceding segments, not just the
    // last one. With a = 3000 and v = 200, stopping needs v^2/(2a) = 6.7 mm,
    // which spans more than one 2 mm segment.
    auto la = make_planner_no_merge();
    for (int i = 0; i < 40; i++) {
        const double x = i * 2.0;
        CHECK(la->push(line(Vec3{x, 0, 0}, Vec3{x + 2.0, 0, 0})));
    }
    CHECK(la->plan(0.0, 0.0));
    CHECK(la->size() == 40);

    const int last = la->size() - 1;
    CHECK_NEAR(la->exit_velocity(last), 0.0, 1e-9);

    // Entry velocities must be decreasing over the final few segments.
    CHECK(la->entry_velocity(last) < la->entry_velocity(last - 1));
    CHECK(la->entry_velocity(last - 1) < la->entry_velocity(last - 2));
    CHECK(la->entry_velocity(last - 2) < la->entry_velocity(last - 3));
}

void test_sharp_corner_slows_the_approach() {
    // Straight run, then a right angle. The segments before the corner must be
    // planned to arrive at the junction velocity, not at full feed.
    auto la = make_planner_no_merge();
    for (int i = 0; i < 10; i++) {
        const double x = i * 10.0;
        CHECK(la->push(line(Vec3{x, 0, 0}, Vec3{x + 10.0, 0, 0})));
    }
    CHECK(la->push(line(Vec3{100, 0, 0}, Vec3{100, 50, 0})));
    CHECK(la->plan(0.0, 0.0));

    const int corner = 9;  // last segment before the turn
    const double vj = junction_velocity(la->segment(corner), la->segment(corner + 1),
                                        la->segment(corner).max_path_acceleration(machine_limits()),
                                        policy());

    CHECK_NEAR(la->exit_velocity(corner), vj, 1e-6);
    // And it is genuinely a slowdown from mid-program speed.
    CHECK(la->exit_velocity(corner) < la->entry_velocity(5));
}

void test_arcs_in_the_queue_plan_successfully() {
    auto la = make_planner();

    CHECK(la->push(line(Vec3{0, 0, 0}, Vec3{50, 0, 0})));

    PathSegment arc;
    CHECK(make_arc(arc, Vec3{50, 0, 0}, Vec3{60, 10, 0}, Vec3{50, 10, 0}, Plane::XY, false) ==
          PathResult::Ok);
    CHECK(la->push(arc));

    CHECK(la->push(line(Vec3{60, 10, 0}, Vec3{60, 60, 0})));

    CHECK(la->plan(0.0, 0.0));

    for (int i = 0; i + 1 < la->size(); i++) {
        CHECK_NEAR(la->exit_velocity(i), la->entry_velocity(i + 1), 1e-9);
    }
    CHECK(la->total_duration() > 0.0);
}

void test_sampling_walks_segment_boundaries() {
    auto la = make_planner();
    for (int i = 0; i < 5; i++) {
        const double x = i * 10.0;
        CHECK(la->push(line(Vec3{x, 0, 0}, Vec3{x + 10.0, 0, 0})));
    }
    CHECK(la->plan(0.0, 0.0));

    CHECK_NEAR(la->at(0.0).position.x, 0.0, 1e-9);

    const auto end = la->at(la->total_duration());
    CHECK_NEAR(end.position.x, 50.0, 1e-4);
    CHECK(end.finished);

    // Position must advance monotonically across the whole queue.
    double prev = -1.0;
    for (int i = 0; i <= 2000; i++) {
        const double x = la->at(la->total_duration() * i / 2000.0).position.x;
        CHECK(x >= prev - 1e-9);
        prev = x;
    }
}

void test_start_and_terminal_velocities_are_honoured() {
    auto la = make_planner();
    for (int i = 0; i < 10; i++) {
        const double x = i * 20.0;
        CHECK(la->push(line(Vec3{x, 0, 0}, Vec3{x + 20.0, 0, 0})));
    }
    CHECK(la->plan(50.0, 30.0));

    CHECK_NEAR(la->entry_velocity(0), 50.0, 1e-6);
    CHECK_NEAR(la->exit_velocity(la->size() - 1), 30.0, 1e-6);
}

void test_collinear_segments_are_merged() {
    auto la = make_planner();
    for (int i = 0; i < 200; i++) {
        const double x = static_cast<double>(i);
        CHECK(la->push(line(Vec3{x, 0, 0}, Vec3{x + 1.0, 0, 0})));
    }
    CHECK(la->size() == 1);
    CHECK(la->merged_count() == 199);
    CHECK_NEAR(la->total_length(), 200.0, 1e-9);
}

void test_non_collinear_segments_are_not_merged() {
    // A CAM-generated curve is a chain of nearly-collinear chords. Merging
    // those would flatten the curve, so the threshold must be strict.
    auto la = make_planner();
    for (int i = 0; i < 50; i++) {
        CHECK(la->push(zigzag(i, 1.0)));
    }
    CHECK(la->size() == 50);
    CHECK(la->merged_count() == 0);
}

void test_merging_is_what_makes_short_moves_fast() {
    // Quantify the effect the merge exists for.
    auto merged = make_planner();
    auto unmerged = make_planner_no_merge();

    for (int i = 0; i < 200; i++) {
        const double x = static_cast<double>(i);
        const PathSegment s = line(Vec3{x, 0, 0}, Vec3{x + 1.0, 0, 0});
        CHECK(merged->push(s));
        CHECK(unmerged->push(s));
    }
    CHECK(merged->plan(0.0, 0.0));
    CHECK(unmerged->plan(0.0, 0.0));

    CHECK(merged->size() == 1);
    CHECK(unmerged->size() == 200);

    // Same distance, but merging lets the profile sustain acceleration.
    CHECK_NEAR(merged->total_length(), unmerged->total_length(), 1e-9);
    CHECK(merged->total_duration() < unmerged->total_duration());
}

void test_segment_boundaries_cost_acceleration_under_jerk_limiting() {
    // Documents the constraint that motivates merging: an S-curve must ramp
    // acceleration 0 -> A -> 0 within a segment, so the velocity gained over a
    // fixed distance falls sharply as the entry velocity rises.
    const double a = 3000.0;
    const double j = 30000.0;

    const double gain_from_rest = max_reachable_velocity_jerk(0.0, 1.0, a, j) - 0.0;
    const double gain_at_speed = max_reachable_velocity_jerk(180.0, 1.0, a, j) - 180.0;

    CHECK(gain_from_rest > 25.0);
    CHECK(gain_at_speed < 1.0);

    // One long segment reaches far more than the chain of short ones.
    const double one_long = max_reachable_velocity_jerk(0.0, 200.0, a, j);
    CHECK(one_long > 900.0);
}

void test_empty_queue_is_safe() {
    auto la = make_planner();
    CHECK(la->plan(0.0, 0.0));
    CHECK_NEAR(la->total_duration(), 0.0, 1e-12);
    CHECK_NEAR(la->total_length(), 0.0, 1e-12);
    const auto c = la->at(1.0);
    CHECK_NEAR(c.path_velocity, 0.0, 1e-12);
}

}  // namespace

int main() {
    std::printf("test_lookahead\n");

    test_jerk_limited_reachable_is_below_constant_accel();
    test_reachable_velocity_is_consistent_with_ramp_distance();

    test_collinear_junction_is_unrestricted();
    test_reversal_junction_forces_a_stop();
    test_right_angle_junction_is_limited();
    test_sharper_corner_means_lower_junction_speed();
    test_larger_deviation_allows_faster_corners();
    test_axis_velocity_jump_cap_is_applied();

    test_push_rejects_degenerate_and_respects_capacity();
    test_velocity_is_continuous_across_junctions();
    test_sampled_velocity_has_no_step_at_boundaries();

    test_many_short_moves_reach_commanded_feed();
    test_without_lookahead_the_same_program_is_far_slower();
    test_deceleration_propagates_backwards_through_segments();
    test_sharp_corner_slows_the_approach();
    test_arcs_in_the_queue_plan_successfully();
    test_sampling_walks_segment_boundaries();
    test_start_and_terminal_velocities_are_honoured();
    test_collinear_segments_are_merged();
    test_non_collinear_segments_are_not_merged();
    test_merging_is_what_makes_short_moves_fast();
    test_segment_boundaries_cost_acceleration_under_jerk_limiting();
    test_empty_queue_is_safe();

    std::printf("  %d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
