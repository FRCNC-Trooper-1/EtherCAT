// Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.

#include "frcnc/motion/path.hpp"

#include <cmath>

namespace frcnc::motion {

namespace {

constexpr double kEps = 1e-9;
constexpr double kTwoPi = 6.283185307179586476925286766559;

constexpr double clamp(double v, double lo, double hi) noexcept {
    return v < lo ? lo : (v > hi ? hi : v);
}

}  // namespace

Vec3 operator+(const Vec3& a, const Vec3& b) noexcept {
    return Vec3{a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 operator-(const Vec3& a, const Vec3& b) noexcept {
    return Vec3{a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3 operator*(const Vec3& a, double s) noexcept {
    return Vec3{a.x * s, a.y * s, a.z * s};
}

double dot(const Vec3& a, const Vec3& b) noexcept {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

double norm(const Vec3& a) noexcept {
    return std::sqrt(dot(a, a));
}

double distance(const Vec3& a, const Vec3& b) noexcept {
    return norm(a - b);
}

void plane_axes(Plane p, int& u, int& v, int& w) noexcept {
    switch (p) {
        case Plane::XY: u = 0; v = 1; w = 2; break;  // G17
        case Plane::ZX: u = 2; v = 0; w = 1; break;  // G18
        case Plane::YZ: u = 1; v = 2; w = 0; break;  // G19
    }
}

PathResult make_linear(PathSegment& seg, const Vec3& from, const Vec3& to) noexcept {
    seg = PathSegment{};
    seg.type = SegmentType::Linear;
    seg.start = from;
    seg.end = to;
    seg.length_ = distance(from, to);
    return (seg.length_ <= kEps) ? PathResult::ZeroLength : PathResult::Ok;
}

PathResult make_arc(PathSegment& seg, const Vec3& from, const Vec3& to, const Vec3& centre,
                    Plane plane, bool clockwise, int full_turns) noexcept {
    seg = PathSegment{};
    seg.type = SegmentType::Arc;
    seg.start = from;
    seg.end = to;
    seg.centre = centre;
    seg.plane = plane;
    seg.clockwise = clockwise;

    int u = 0;
    int v = 1;
    int w = 2;
    plane_axes(plane, u, v, w);

    const double du1 = from[u] - centre[u];
    const double dv1 = from[v] - centre[v];
    const double du2 = to[u] - centre[u];
    const double dv2 = to[v] - centre[v];

    const double r1 = std::sqrt(du1 * du1 + dv1 * dv1);
    const double r2 = std::sqrt(du2 * du2 + dv2 * dv2);

    if (r1 <= kEps) {
        return PathResult::InvalidRadius;
    }

    // Both endpoints must lie on the same circle. G-code interpreters normally
    // allow a small tolerance here because I/J/K come from finite-precision
    // text; a mismatch beyond it is a programming error worth reporting.
    const double radius_tol = 1e-4 * (r1 > 1.0 ? r1 : 1.0);
    if (std::fabs(r1 - r2) > radius_tol) {
        return PathResult::RadiusMismatch;
    }

    seg.radius = r1;
    seg.theta_start = std::atan2(dv1, du1);

    const double theta_end = std::atan2(dv2, du2);
    double sweep = theta_end - seg.theta_start;

    // Normalise into the requested direction. Note the sign convention: in the
    // plane's (u, v) frame, increasing theta is counter-clockwise, so a
    // clockwise arc has negative sweep.
    if (clockwise) {
        while (sweep >= 0.0) {
            sweep -= kTwoPi;
        }
        while (sweep < -kTwoPi) {
            sweep += kTwoPi;
        }
    } else {
        while (sweep <= 0.0) {
            sweep += kTwoPi;
        }
        while (sweep > kTwoPi) {
            sweep -= kTwoPi;
        }
    }

    // Coincident endpoints mean a full circle, not a zero-length arc.
    const double endpoint_gap = std::sqrt((du2 - du1) * (du2 - du1) + (dv2 - dv1) * (dv2 - dv1));
    if (endpoint_gap <= kEps) {
        sweep = clockwise ? -kTwoPi : kTwoPi;
    }

    if (full_turns > 0) {
        sweep += (clockwise ? -1.0 : 1.0) * kTwoPi * static_cast<double>(full_turns);
    }

    seg.sweep = sweep;
    seg.helix_rise = to[w] - from[w];

    const double arc_in_plane = std::fabs(sweep) * seg.radius;
    seg.length_ = std::sqrt(arc_in_plane * arc_in_plane + seg.helix_rise * seg.helix_rise);

    return (seg.length_ <= kEps) ? PathResult::ZeroLength : PathResult::Ok;
}

Vec3 PathSegment::point_at(double s) const noexcept {
    if (length_ <= kEps) {
        return start;
    }
    const double f = clamp(s / length_, 0.0, 1.0);

    if (type == SegmentType::Linear) {
        return start + (end - start) * f;
    }

    int u = 0;
    int v = 1;
    int w = 2;
    plane_axes(plane, u, v, w);

    const double theta = theta_start + sweep * f;

    Vec3 p{};
    p[u] = centre[u] + radius * std::cos(theta);
    p[v] = centre[v] + radius * std::sin(theta);
    p[w] = start[w] + helix_rise * f;
    return p;
}

Vec3 PathSegment::tangent_at(double s) const noexcept {
    if (length_ <= kEps) {
        return Vec3{};
    }

    if (type == SegmentType::Linear) {
        const Vec3 d = end - start;
        const double n = norm(d);
        return (n > kEps) ? d * (1.0 / n) : Vec3{};
    }

    int u = 0;
    int v = 1;
    int w = 2;
    plane_axes(plane, u, v, w);

    const double f = clamp(s / length_, 0.0, 1.0);
    const double theta = theta_start + sweep * f;

    // d(point)/ds. In-plane speed is |sweep|*r / length per unit s; the sign of
    // sweep sets the direction of travel around the circle.
    const double dtheta_ds = sweep / length_;

    Vec3 t{};
    t[u] = -radius * std::sin(theta) * dtheta_ds;
    t[v] = radius * std::cos(theta) * dtheta_ds;
    t[w] = helix_rise / length_;

    const double n = norm(t);
    return (n > kEps) ? t * (1.0 / n) : Vec3{};
}

double PathSegment::max_path_velocity(const AxisLimits& limits,
                                      const PathConstraints& constraints) const noexcept {
    if (length_ <= kEps) {
        return 0.0;
    }

    double v_limit = 1e300;

    if (type == SegmentType::Linear) {
        // Axis i moves at v_path * |tangent_i|. Invert for the bound.
        const Vec3 t = tangent_at(0.0);
        for (int i = 0; i < 3; i++) {
            const double component = std::fabs(t[i]);
            if (component <= kEps) {
                continue;  // axis does not move; imposes no limit
            }
            if (limits[i].max_velocity <= kEps) {
                continue;
            }
            const double bound = limits[i].max_velocity / component;
            if (bound < v_limit) {
                v_limit = bound;
            }
        }
        return v_limit;
    }

    // --- arc ---

    int u = 0;
    int v = 1;
    int w = 2;
    plane_axes(plane, u, v, w);

    // The tangent sweeps as the arc progresses, so bound by the worst direction
    // the arc actually reaches. Sampling is conservative and cheap; a closed
    // form would need to consider where cos/sin peak within the swept range.
    constexpr int kSamples = 32;
    for (int k = 0; k <= kSamples; k++) {
        const double s = length_ * (static_cast<double>(k) / kSamples);
        const Vec3 t = tangent_at(s);
        for (int i = 0; i < 3; i++) {
            const double component = std::fabs(t[i]);
            if (component <= kEps || limits[i].max_velocity <= kEps) {
                continue;
            }
            const double bound = limits[i].max_velocity / component;
            if (bound < v_limit) {
                v_limit = bound;
            }
        }
    }

    if (radius > kEps) {
        // Centripetal acceleration: a = v^2 / r. On a small radius this binds
        // long before any axis velocity limit does, and ignoring it is why
        // machines fault or lose accuracy on tight arcs.
        double a_cap = constraints.max_centripetal_accel;
        if (a_cap <= kEps) {
            a_cap = max_path_acceleration(limits);
        }
        if (a_cap > kEps) {
            const double bound = std::sqrt(a_cap * radius);
            if (bound < v_limit) {
                v_limit = bound;
            }
        }

        // Chord error: successive setpoints are a chord across the true arc.
        // Sagitta for a step of arc length L is approximately L^2 / (8r), so
        //     tol >= (v*dt)^2 / (8r)  =>  v <= sqrt(8*r*tol)/dt
        if (constraints.chord_tolerance > kEps && constraints.cycle_time > kEps) {
            const double bound =
                std::sqrt(8.0 * radius * constraints.chord_tolerance) / constraints.cycle_time;
            if (bound < v_limit) {
                v_limit = bound;
            }
        }
    }

    return v_limit;
}

double PathSegment::max_path_acceleration(const AxisLimits& limits) const noexcept {
    if (length_ <= kEps) {
        return 0.0;
    }

    double a_limit = 1e300;
    bool any = false;

    // Use the worst tangent direction, as for velocity.
    const int samples = (type == SegmentType::Linear) ? 1 : 32;
    for (int k = 0; k < samples; k++) {
        const double s =
            (samples == 1) ? 0.0 : length_ * (static_cast<double>(k) / (samples - 1));
        const Vec3 t = tangent_at(s);
        for (int i = 0; i < 3; i++) {
            const double component = std::fabs(t[i]);
            if (component <= kEps || limits[i].max_acceleration <= kEps) {
                continue;
            }
            const double bound = limits[i].max_acceleration / component;
            if (bound < a_limit) {
                a_limit = bound;
            }
            any = true;
        }
    }
    return any ? a_limit : 0.0;
}

double PathSegment::max_path_jerk(const AxisLimits& limits) const noexcept {
    if (length_ <= kEps) {
        return 0.0;
    }

    double j_limit = 1e300;
    bool any = false;

    const int samples = (type == SegmentType::Linear) ? 1 : 32;
    for (int k = 0; k < samples; k++) {
        const double s =
            (samples == 1) ? 0.0 : length_ * (static_cast<double>(k) / (samples - 1));
        const Vec3 t = tangent_at(s);
        for (int i = 0; i < 3; i++) {
            const double component = std::fabs(t[i]);
            if (component <= kEps) {
                continue;
            }
            if (limits[i].max_jerk <= kEps) {
                return 0.0;  // any moving axis unlimited => path unlimited
            }
            const double bound = limits[i].max_jerk / component;
            if (bound < j_limit) {
                j_limit = bound;
            }
            any = true;
        }
    }
    return any ? j_limit : 0.0;
}

double junction_cosine(const PathSegment& a, const PathSegment& b) noexcept {
    const Vec3 ta = a.tangent_at(a.length());
    const Vec3 tb = b.tangent_at(0.0);
    const double na = norm(ta);
    const double nb = norm(tb);
    if (na <= kEps || nb <= kEps) {
        return 1.0;  // degenerate segment: treat as collinear
    }
    return clamp(dot(ta, tb) / (na * nb), -1.0, 1.0);
}

}  // namespace frcnc::motion
