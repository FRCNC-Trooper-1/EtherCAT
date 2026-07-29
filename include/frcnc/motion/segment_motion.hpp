// Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.
//
// Motion along one path segment: geometry + profile -> per-axis commands.
//
// This is where the pieces meet. A PathSegment supplies geometry, an
// SCurveProfile supplies timing along it, and this class produces what the
// cyclic task actually writes into the drives every cycle:
//
//   position     -> 0x607A Target Position
//   velocity     -> 0x60B1 Velocity Offset   (velocity feedforward)
//   acceleration -> 0x60B2 Torque Offset     (via inertia, torque feedforward)
//
// Why feedforward matters more than it sounds:
//
// In CSP the master sends position only. A classical position loop must
// generate error in order to produce velocity — following error is roughly
// v / Kv. At 167 mm/s with a typical Kv of 30 1/s that is about 5.5 mm of lag.
// On a straight line every axis lags equally and the part is unaffected. On a
// corner or an arc the lags differ per axis, and the result is pure path
// distortion — the usual cause of out-of-round circles.
//
// Feeding the commanded velocity forward removes most of that lag. The master
// knows the exact intended velocity, which beats the drive differentiating a
// quantised position command.
//
// Reference: docs/05-motion-architecture.md §4

#pragma once

#include "frcnc/motion/path.hpp"
#include "frcnc/motion/profile.hpp"
#include "frcnc/motion/scurve.hpp"

namespace frcnc::motion {

/// What the cyclic task writes to the drives for one sample.
struct AxisCommand {
    Vec3 position{};      ///< machine coordinates
    Vec3 velocity{};      ///< units/s — velocity feedforward
    Vec3 acceleration{};  ///< units/s^2 — torque feedforward input

    double path_position = 0.0;      ///< arc length travelled
    double path_velocity = 0.0;      ///< speed along the path
    double path_acceleration = 0.0;  ///< tangential acceleration

    bool finished = false;  ///< true once t >= duration()
};

/// Plans and samples motion along a single path segment.
///
/// Planning is non-real-time; sampling is cheap enough for the cyclic path
/// (a handful of trig calls, no allocation, no branching on data size).
class SegmentMotion {
public:
    /// Plan motion along `seg`.
    ///
    /// Feedrate is bounded by the segment's own limits — per-axis velocity,
    /// centripetal acceleration and chord error — so the caller does not need
    /// to pre-clamp. `requested_feed` is the programmed F value; pass 0 to run
    /// as fast as the constraints permit.
    PlanResult plan(const PathSegment& seg, double entry_vel, double exit_vel,
                    const AxisLimits& axes, const PathConstraints& constraints,
                    double requested_feed = 0.0) noexcept;

    [[nodiscard]] bool valid() const noexcept { return valid_; }
    [[nodiscard]] double duration() const noexcept { return profile_.duration(); }
    [[nodiscard]] double length() const noexcept { return segment_.length(); }

    /// Path speed actually reached, after all constraints.
    [[nodiscard]] double peak_velocity() const noexcept { return profile_.peak_velocity(); }

    /// The feed ceiling this segment's geometry imposed, before profiling.
    [[nodiscard]] double feed_limit() const noexcept { return feed_limit_; }

    [[nodiscard]] double entry_velocity() const noexcept { return entry_vel_; }
    [[nodiscard]] double exit_velocity() const noexcept { return exit_vel_; }

    /// Sample at time t from the start of the segment. Clamped to [0, duration].
    [[nodiscard]] AxisCommand at(double t) const noexcept;

    [[nodiscard]] const PathSegment& segment() const noexcept { return segment_; }

    void reset() noexcept;

private:
    bool valid_ = false;
    PathSegment segment_{};
    SCurveProfile profile_{};
    double feed_limit_ = 0.0;
    double entry_vel_ = 0.0;
    double exit_vel_ = 0.0;
};

/// Chord error for a given speed, radius and cycle time.
///
/// Successive setpoints are a chord across the true arc; the sagitta is
/// approximately L^2 / (8r) for a step of arc length L = v*dt. This is the
/// relationship that ties path accuracy directly to the servo update rate.
[[nodiscard]] double chord_error(double velocity, double radius, double cycle_time) noexcept;

/// Highest speed that keeps chord error within tolerance.
[[nodiscard]] double max_velocity_for_chord(double radius, double tolerance,
                                            double cycle_time) noexcept;

}  // namespace frcnc::motion
