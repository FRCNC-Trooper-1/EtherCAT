// Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.
//
// Tests for path geometry: linear and circular segments.
//
// The checks that matter for a machine tool are geometric accuracy (points
// actually lie on the commanded circle), correct arc direction (G2 vs G3 is a
// scrapped part if reversed), and feedrate limiting that respects axis limits,
// centripetal acceleration, and chord error.

#include "frcnc/motion/path.hpp"

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

AxisLimits uniform_limits(double v, double a, double j) {
    AxisLimits l;
    for (int i = 0; i < 3; i++) {
        l[i] = Limits{v, a, j};
    }
    return l;
}

// --- vectors ----------------------------------------------------------------

void test_vec3_basics() {
    const Vec3 a{1.0, 2.0, 3.0};
    const Vec3 b{4.0, 5.0, 6.0};
    CHECK_NEAR(dot(a, b), 32.0, 1e-12);
    CHECK_NEAR(norm(Vec3{3.0, 4.0, 0.0}), 5.0, 1e-12);
    CHECK_NEAR(distance(Vec3{0, 0, 0}, Vec3{0, 0, 2}), 2.0, 1e-12);

    const Vec3 s = a + b;
    CHECK_NEAR(s.x, 5.0, 1e-12);
    CHECK_NEAR(s.z, 9.0, 1e-12);

    // Index accessor must agree with the named members.
    CHECK_NEAR(a[0], a.x, 1e-12);
    CHECK_NEAR(a[1], a.y, 1e-12);
    CHECK_NEAR(a[2], a.z, 1e-12);
}

// --- linear -----------------------------------------------------------------

void test_linear_length_and_points() {
    PathSegment seg;
    CHECK(make_linear(seg, Vec3{0, 0, 0}, Vec3{3, 4, 0}) == PathResult::Ok);
    CHECK_NEAR(seg.length(), 5.0, 1e-12);

    const Vec3 mid = seg.point_at(2.5);
    CHECK_NEAR(mid.x, 1.5, 1e-12);
    CHECK_NEAR(mid.y, 2.0, 1e-12);

    const Vec3 end = seg.point_at(seg.length());
    CHECK_NEAR(end.x, 3.0, 1e-12);
    CHECK_NEAR(end.y, 4.0, 1e-12);
}

void test_linear_3d_and_tangent() {
    PathSegment seg;
    CHECK(make_linear(seg, Vec3{0, 0, 0}, Vec3{1, 2, 2}) == PathResult::Ok);
    CHECK_NEAR(seg.length(), 3.0, 1e-12);

    const Vec3 t = seg.tangent_at(0.0);
    CHECK_NEAR(norm(t), 1.0, 1e-12);
    CHECK_NEAR(t.x, 1.0 / 3.0, 1e-12);
    CHECK_NEAR(t.z, 2.0 / 3.0, 1e-12);
}

void test_linear_zero_length() {
    PathSegment seg;
    CHECK(make_linear(seg, Vec3{1, 1, 1}, Vec3{1, 1, 1}) == PathResult::ZeroLength);
    CHECK_NEAR(seg.length(), 0.0, 1e-12);
    // Must not produce NaN.
    const Vec3 p = seg.point_at(0.5);
    CHECK_NEAR(p.x, 1.0, 1e-12);
}

void test_linear_sampling_is_clamped() {
    PathSegment seg;
    CHECK(make_linear(seg, Vec3{0, 0, 0}, Vec3{10, 0, 0}) == PathResult::Ok);
    CHECK_NEAR(seg.point_at(-5.0).x, 0.0, 1e-12);
    CHECK_NEAR(seg.point_at(999.0).x, 10.0, 1e-12);
}

// --- arcs -------------------------------------------------------------------

void test_arc_quarter_circle_ccw() {
    // Centre at origin, radius 10, from (10,0) to (0,10), counter-clockwise.
    PathSegment seg;
    CHECK(make_arc(seg, Vec3{10, 0, 0}, Vec3{0, 10, 0}, Vec3{0, 0, 0}, Plane::XY, false) ==
          PathResult::Ok);

    CHECK_NEAR(seg.radius, 10.0, 1e-12);
    CHECK_NEAR(seg.sweep, kPi / 2.0, 1e-12);
    CHECK_NEAR(seg.length(), 10.0 * kPi / 2.0, 1e-12);

    const Vec3 mid = seg.point_at(seg.length() * 0.5);
    const double r = std::sqrt(mid.x * mid.x + mid.y * mid.y);
    CHECK_NEAR(r, 10.0, 1e-9);
    // 45 degrees
    CHECK_NEAR(mid.x, 10.0 * std::cos(kPi / 4.0), 1e-9);
    CHECK_NEAR(mid.y, 10.0 * std::sin(kPi / 4.0), 1e-9);
}

void test_arc_direction_matters() {
    // Same endpoints, opposite directions: CCW sweeps 90 degrees, CW sweeps 270.
    PathSegment ccw;
    PathSegment cw;
    CHECK(make_arc(ccw, Vec3{10, 0, 0}, Vec3{0, 10, 0}, Vec3{0, 0, 0}, Plane::XY, false) ==
          PathResult::Ok);
    CHECK(make_arc(cw, Vec3{10, 0, 0}, Vec3{0, 10, 0}, Vec3{0, 0, 0}, Plane::XY, true) ==
          PathResult::Ok);

    CHECK(ccw.sweep > 0.0);
    CHECK(cw.sweep < 0.0);
    CHECK_NEAR(std::fabs(cw.sweep), 3.0 * kPi / 2.0, 1e-12);
    CHECK(cw.length() > ccw.length());

    // The clockwise arc must pass through the far side (negative y at some point).
    bool saw_negative_y = false;
    for (int i = 0; i <= 100; i++) {
        const Vec3 p = cw.point_at(cw.length() * i / 100.0);
        if (p.y < -1.0) {
            saw_negative_y = true;
        }
    }
    CHECK(saw_negative_y);
}

void test_arc_every_point_lies_on_the_circle() {
    // The core geometric guarantee. If this drifts, circles come out as spirals.
    PathSegment seg;
    CHECK(make_arc(seg, Vec3{25, 0, 0}, Vec3{-25, 0, 0}, Vec3{0, 0, 0}, Plane::XY, false) ==
          PathResult::Ok);

    double worst = 0.0;
    for (int i = 0; i <= 1000; i++) {
        const Vec3 p = seg.point_at(seg.length() * i / 1000.0);
        const double r = std::sqrt(p.x * p.x + p.y * p.y);
        const double err = std::fabs(r - 25.0);
        if (err > worst) {
            worst = err;
        }
    }
    CHECK(worst < 1e-9);
}

void test_arc_full_circle_when_endpoints_coincide() {
    PathSegment seg;
    CHECK(make_arc(seg, Vec3{5, 0, 0}, Vec3{5, 0, 0}, Vec3{0, 0, 0}, Plane::XY, false) ==
          PathResult::Ok);
    CHECK_NEAR(std::fabs(seg.sweep), 2.0 * kPi, 1e-12);
    CHECK_NEAR(seg.length(), 2.0 * kPi * 5.0, 1e-9);

    // Ends where it started.
    const Vec3 end = seg.point_at(seg.length());
    CHECK_NEAR(end.x, 5.0, 1e-9);
    CHECK_NEAR(end.y, 0.0, 1e-9);
}

void test_arc_radius_mismatch_is_rejected() {
    // Endpoints not equidistant from the centre — a real G-code error.
    PathSegment seg;
    CHECK(make_arc(seg, Vec3{10, 0, 0}, Vec3{0, 20, 0}, Vec3{0, 0, 0}, Plane::XY, false) ==
          PathResult::RadiusMismatch);
}

void test_arc_invalid_radius_is_rejected() {
    PathSegment seg;
    CHECK(make_arc(seg, Vec3{0, 0, 0}, Vec3{0, 0, 0}, Vec3{0, 0, 0}, Plane::XY, false) ==
          PathResult::InvalidRadius);
}

void test_arc_planes() {
    int u = 0;
    int v = 0;
    int w = 0;

    plane_axes(Plane::XY, u, v, w);
    CHECK(u == 0 && v == 1 && w == 2);
    plane_axes(Plane::ZX, u, v, w);
    CHECK(u == 2 && v == 0 && w == 1);
    plane_axes(Plane::YZ, u, v, w);
    CHECK(u == 1 && v == 2 && w == 0);

    // An arc in ZX must not move Y.
    PathSegment seg;
    CHECK(make_arc(seg, Vec3{10, 7, 0}, Vec3{0, 7, 10}, Vec3{0, 7, 0}, Plane::ZX, false) ==
          PathResult::Ok);
    for (int i = 0; i <= 50; i++) {
        const Vec3 p = seg.point_at(seg.length() * i / 50.0);
        CHECK_NEAR(p.y, 7.0, 1e-9);
    }
}

void test_helical_arc() {
    // Quarter circle in XY while rising 5 in Z.
    PathSegment seg;
    CHECK(make_arc(seg, Vec3{10, 0, 0}, Vec3{0, 10, 5}, Vec3{0, 0, 0}, Plane::XY, false) ==
          PathResult::Ok);

    const double arc_in_plane = 10.0 * kPi / 2.0;
    CHECK_NEAR(seg.length(), std::sqrt(arc_in_plane * arc_in_plane + 25.0), 1e-9);
    CHECK_NEAR(seg.helix_rise, 5.0, 1e-12);

    CHECK_NEAR(seg.point_at(0.0).z, 0.0, 1e-12);
    CHECK_NEAR(seg.point_at(seg.length()).z, 5.0, 1e-9);
    CHECK_NEAR(seg.point_at(seg.length() * 0.5).z, 2.5, 1e-9);
}

void test_arc_tangent_is_perpendicular_to_radius() {
    PathSegment seg;
    CHECK(make_arc(seg, Vec3{10, 0, 0}, Vec3{-10, 0, 0}, Vec3{0, 0, 0}, Plane::XY, false) ==
          PathResult::Ok);

    for (int i = 1; i < 20; i++) {
        const double s = seg.length() * i / 20.0;
        const Vec3 p = seg.point_at(s);
        const Vec3 t = seg.tangent_at(s);
        // Radius vector from centre to point; tangent must be perpendicular.
        const Vec3 radial{p.x, p.y, 0.0};
        CHECK_NEAR(dot(radial, t), 0.0, 1e-6);
        CHECK_NEAR(norm(t), 1.0, 1e-9);
    }
}

void test_arc_length_matches_numerical_integration() {
    PathSegment seg;
    CHECK(make_arc(seg, Vec3{15, 0, 0}, Vec3{0, 15, 3}, Vec3{0, 0, 0}, Plane::XY, false) ==
          PathResult::Ok);

    // Walking the sampled points must reproduce the analytic length.
    double walked = 0.0;
    Vec3 prev = seg.point_at(0.0);
    const int n = 20000;
    for (int i = 1; i <= n; i++) {
        const Vec3 p = seg.point_at(seg.length() * i / n);
        walked += distance(prev, p);
        prev = p;
    }
    CHECK_NEAR(walked, seg.length(), 1e-4);
}

// --- feedrate limiting ------------------------------------------------------

void test_linear_feed_limited_by_dominant_axis() {
    // Pure X move: bound is exactly the X velocity limit.
    AxisLimits lim = uniform_limits(100.0, 1000.0, 0.0);
    PathConstraints c{};

    PathSegment seg;
    CHECK(make_linear(seg, Vec3{0, 0, 0}, Vec3{10, 0, 0}) == PathResult::Ok);
    CHECK_NEAR(seg.max_path_velocity(lim, c), 100.0, 1e-9);

    // 45 degrees in XY: each axis sees v/sqrt(2), so path speed can be
    // sqrt(2)*100 before either axis saturates.
    PathSegment diag;
    CHECK(make_linear(diag, Vec3{0, 0, 0}, Vec3{10, 10, 0}) == PathResult::Ok);
    CHECK_NEAR(diag.max_path_velocity(lim, c), 100.0 * std::sqrt(2.0), 1e-6);
}

void test_linear_feed_limited_by_slowest_axis() {
    // Z is slow; a move that is mostly Z must be limited by Z.
    AxisLimits lim = uniform_limits(100.0, 1000.0, 0.0);
    lim[2].max_velocity = 10.0;
    PathConstraints c{};

    PathSegment seg;
    CHECK(make_linear(seg, Vec3{0, 0, 0}, Vec3{0, 0, 10}) == PathResult::Ok);
    CHECK_NEAR(seg.max_path_velocity(lim, c), 10.0, 1e-9);
}

void test_arc_feed_limited_by_centripetal_acceleration() {
    // v <= sqrt(a*r). With a=1000 and r=1, that is ~31.6 — far below the
    // 100 mm/s axis limit. Ignoring this is why machines fault on tight arcs.
    AxisLimits lim = uniform_limits(100.0, 1000.0, 0.0);
    PathConstraints c{};
    c.chord_tolerance = 1.0;  // large, so chord error does not bind
    c.cycle_time = 0.001;
    c.max_centripetal_accel = 1000.0;

    PathSegment seg;
    CHECK(make_arc(seg, Vec3{1, 0, 0}, Vec3{-1, 0, 0}, Vec3{0, 0, 0}, Plane::XY, false) ==
          PathResult::Ok);

    const double v = seg.max_path_velocity(lim, c);
    CHECK_NEAR(v, std::sqrt(1000.0 * 1.0), 1e-6);
    CHECK(v < 100.0);
}

void test_arc_feed_limited_by_chord_tolerance() {
    // v <= sqrt(8*r*tol)/dt. With r=100, tol=0.001, dt=0.001:
    //   sqrt(8*100*0.001)/0.001 = sqrt(0.8)/0.001 ~= 894
    // Set a huge centripetal cap so only the chord bound is active.
    AxisLimits lim = uniform_limits(1e6, 1e9, 0.0);
    PathConstraints c{};
    c.chord_tolerance = 0.001;
    c.cycle_time = 0.001;
    c.max_centripetal_accel = 1e9;

    PathSegment seg;
    CHECK(make_arc(seg, Vec3{100, 0, 0}, Vec3{-100, 0, 0}, Vec3{0, 0, 0}, Plane::XY, false) ==
          PathResult::Ok);

    const double expected = std::sqrt(8.0 * 100.0 * 0.001) / 0.001;
    CHECK_NEAR(seg.max_path_velocity(lim, c), expected, 1e-3);
}

void test_smaller_radius_forces_lower_feed() {
    AxisLimits lim = uniform_limits(1000.0, 1000.0, 0.0);
    PathConstraints c{};
    c.chord_tolerance = 0.01;
    c.cycle_time = 0.001;
    c.max_centripetal_accel = 1000.0;

    double prev = 1e300;
    for (double r : {100.0, 50.0, 10.0, 1.0, 0.5}) {
        PathSegment seg;
        CHECK(make_arc(seg, Vec3{r, 0, 0}, Vec3{-r, 0, 0}, Vec3{0, 0, 0}, Plane::XY, false) ==
              PathResult::Ok);
        const double v = seg.max_path_velocity(lim, c);
        CHECK(v < prev);  // tighter radius must never allow more speed
        prev = v;
    }
}

void test_path_acceleration_and_jerk_limits() {
    AxisLimits lim = uniform_limits(100.0, 1000.0, 10000.0);

    PathSegment seg;
    CHECK(make_linear(seg, Vec3{0, 0, 0}, Vec3{10, 0, 0}) == PathResult::Ok);
    CHECK_NEAR(seg.max_path_acceleration(lim), 1000.0, 1e-9);
    CHECK_NEAR(seg.max_path_jerk(lim), 10000.0, 1e-9);

    // An axis with no jerk limit makes the whole path unlimited.
    AxisLimits mixed = lim;
    mixed[0].max_jerk = 0.0;
    CHECK_NEAR(seg.max_path_jerk(mixed), 0.0, 1e-12);
}

// --- junctions --------------------------------------------------------------

void test_junction_cosine() {
    PathSegment a;
    PathSegment b;

    // Collinear: cosine 1.
    CHECK(make_linear(a, Vec3{0, 0, 0}, Vec3{10, 0, 0}) == PathResult::Ok);
    CHECK(make_linear(b, Vec3{10, 0, 0}, Vec3{20, 0, 0}) == PathResult::Ok);
    CHECK_NEAR(junction_cosine(a, b), 1.0, 1e-12);

    // Right angle: cosine 0.
    CHECK(make_linear(b, Vec3{10, 0, 0}, Vec3{10, 10, 0}) == PathResult::Ok);
    CHECK_NEAR(junction_cosine(a, b), 0.0, 1e-12);

    // Full reversal: cosine -1.
    CHECK(make_linear(b, Vec3{10, 0, 0}, Vec3{0, 0, 0}) == PathResult::Ok);
    CHECK_NEAR(junction_cosine(a, b), -1.0, 1e-12);
}

void test_junction_line_to_arc_is_smooth_when_tangential() {
    // A line along +X ending at (0,-10), meeting a circle centred at origin at
    // its lowest point, travelling counter-clockwise. Tangents should align.
    PathSegment line;
    CHECK(make_linear(line, Vec3{-20, -10, 0}, Vec3{0, -10, 0}) == PathResult::Ok);

    PathSegment arc;
    CHECK(make_arc(arc, Vec3{0, -10, 0}, Vec3{10, 0, 0}, Vec3{0, 0, 0}, Plane::XY, false) ==
          PathResult::Ok);

    CHECK_NEAR(junction_cosine(line, arc), 1.0, 1e-6);
}

}  // namespace

int main() {
    std::printf("test_path\n");

    test_vec3_basics();

    test_linear_length_and_points();
    test_linear_3d_and_tangent();
    test_linear_zero_length();
    test_linear_sampling_is_clamped();

    test_arc_quarter_circle_ccw();
    test_arc_direction_matters();
    test_arc_every_point_lies_on_the_circle();
    test_arc_full_circle_when_endpoints_coincide();
    test_arc_radius_mismatch_is_rejected();
    test_arc_invalid_radius_is_rejected();
    test_arc_planes();
    test_helical_arc();
    test_arc_tangent_is_perpendicular_to_radius();
    test_arc_length_matches_numerical_integration();

    test_linear_feed_limited_by_dominant_axis();
    test_linear_feed_limited_by_slowest_axis();
    test_arc_feed_limited_by_centripetal_acceleration();
    test_arc_feed_limited_by_chord_tolerance();
    test_smaller_radius_forces_lower_feed();
    test_path_acceleration_and_jerk_limits();

    test_junction_cosine();
    test_junction_line_to_arc_is_smooth_when_tangential();

    std::printf("  %d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
