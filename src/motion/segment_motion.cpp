// Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.

#include "frcnc/motion/segment_motion.hpp"

#include <cmath>

namespace frcnc::motion {

namespace {
constexpr double kEps = 1e-9;
}

double chord_error(double velocity, double radius, double cycle_time) noexcept {
    if (radius <= kEps) {
        return 0.0;
    }
    const double step = velocity * cycle_time;
    return (step * step) / (8.0 * radius);
}

double max_velocity_for_chord(double radius, double tolerance, double cycle_time) noexcept {
    if (radius <= kEps || tolerance <= kEps || cycle_time <= kEps) {
        return 1e300;
    }
    return std::sqrt(8.0 * radius * tolerance) / cycle_time;
}

void SegmentMotion::reset() noexcept {
    *this = SegmentMotion{};
}

PlanResult SegmentMotion::plan(const PathSegment& seg, double entry_vel, double exit_vel,
                               const AxisLimits& axes, const PathConstraints& constraints,
                               double requested_feed) noexcept {
    reset();
    segment_ = seg;

    if (seg.length() <= kEps) {
        valid_ = true;
        return PlanResult::ZeroLength;
    }

    // Geometry decides the ceiling: per-axis velocity through the tangent,
    // centripetal acceleration, and chord error.
    double v_limit = seg.max_path_velocity(axes, constraints);

    // The programmed feed can only lower it, never raise it.
    if (requested_feed > kEps && requested_feed < v_limit) {
        v_limit = requested_feed;
    }
    feed_limit_ = v_limit;

    const double a_limit = seg.max_path_acceleration(axes);
    const double j_limit = seg.max_path_jerk(axes);

    if (v_limit <= kEps || a_limit <= kEps) {
        return PlanResult::InvalidLimits;
    }

    // Entry and exit velocities come from look-ahead and may exceed what this
    // segment's geometry allows — a tight arc after a long straight, say.
    entry_vel_ = (entry_vel > v_limit) ? v_limit : entry_vel;
    exit_vel_ = (exit_vel > v_limit) ? v_limit : exit_vel;
    if (entry_vel_ < 0.0) {
        entry_vel_ = 0.0;
    }
    if (exit_vel_ < 0.0) {
        exit_vel_ = 0.0;
    }

    const Limits lim{v_limit, a_limit, j_limit};
    const PlanResult r = profile_.plan(seg.length(), entry_vel_, exit_vel_, lim);
    valid_ = (r == PlanResult::Ok || r == PlanResult::ZeroLength);
    return r;
}

AxisCommand SegmentMotion::at(double t) const noexcept {
    AxisCommand cmd{};

    if (!valid_) {
        return cmd;
    }

    if (segment_.length() <= kEps) {
        cmd.position = segment_.start;
        cmd.finished = true;
        return cmd;
    }

    const Sample s = profile_.at(t);

    cmd.path_position = s.position;
    cmd.path_velocity = s.velocity;
    cmd.path_acceleration = s.acceleration;
    cmd.finished = (t >= profile_.duration());

    cmd.position = segment_.point_at(s.position);

    const Vec3 tangent = segment_.tangent_at(s.position);

    // Velocity is purely tangential: dp/dt = (dp/ds)(ds/dt).
    cmd.velocity = tangent * s.velocity;

    // Acceleration has two parts:
    //     a = (d2p/ds2) * v^2   +   tangent * a_tangential
    //          centripetal          along-path
    //
    // The first term is what curves the path. Dropping it makes torque
    // feedforward wrong on every arc, which is precisely where feedforward
    // earns its keep.
    const Vec3 curvature = segment_.second_derivative_at(s.position);
    cmd.acceleration = curvature * (s.velocity * s.velocity) + tangent * s.acceleration;

    return cmd;
}

}  // namespace frcnc::motion
