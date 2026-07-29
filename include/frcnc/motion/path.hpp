// Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.
//
// Path geometry: linear and circular segments in machine coordinates.
//
// A segment maps an arc-length parameter s in [0, length] to a Cartesian point.
// Motion along it is planned by running a scalar profile over s, which is what
// makes all axes start and finish together — the defining property of
// coordinated motion.
//
// PathSegment is deliberately a plain tagged struct rather than a class
// hierarchy: it must be trivially copyable to cross the lock-free ring between
// the planner and the cyclic task without allocation or indirection.
// See docs/05-motion-architecture.md §3.

#pragma once

#include "frcnc/motion/profile.hpp"

#include <cstdint>

namespace frcnc::motion {

/// A point or vector in machine coordinates. Units are whatever the machine
/// configuration uses; millimetres throughout this project.
struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    [[nodiscard]] double operator[](int i) const noexcept {
        return (i == 0) ? x : (i == 1) ? y : z;
    }
    double& operator[](int i) noexcept { return (i == 0) ? x : (i == 1) ? y : z; }
};

[[nodiscard]] Vec3 operator+(const Vec3& a, const Vec3& b) noexcept;
[[nodiscard]] Vec3 operator-(const Vec3& a, const Vec3& b) noexcept;
[[nodiscard]] Vec3 operator*(const Vec3& a, double s) noexcept;
[[nodiscard]] double dot(const Vec3& a, const Vec3& b) noexcept;
[[nodiscard]] double norm(const Vec3& a) noexcept;
[[nodiscard]] double distance(const Vec3& a, const Vec3& b) noexcept;

/// Arc plane selection. Matches G17/G18/G19.
enum class Plane : std::uint8_t {
    XY = 0,  ///< G17 — arc in X/Y, normal Z
    ZX = 1,  ///< G18 — arc in Z/X, normal Y
    YZ = 2,  ///< G19 — arc in Y/Z, normal X
};

/// Per-axis kinematic limits.
struct AxisLimits {
    Limits axis[3]{};

    [[nodiscard]] const Limits& operator[](int i) const noexcept { return axis[i]; }
    Limits& operator[](int i) noexcept { return axis[i]; }
};

/// Constraints used when limiting feedrate on curved paths.
struct PathConstraints {
    /// Maximum tolerated deviation between the true arc and the chord actually
    /// traversed between two cyclic setpoints. Bounds feedrate on tight radii.
    double chord_tolerance = 0.01;  ///< mm

    /// Control cycle time. Chord error grows with it.
    double cycle_time = 0.001;  ///< s

    /// Cap on centripetal acceleration, v^2/r. Usually the tightest axis
    /// acceleration limit.
    double max_centripetal_accel = 0.0;  ///< mm/s^2; 0 disables the check
};

enum class SegmentType : std::uint8_t {
    Linear,
    Arc,
};

/// Result of constructing a segment.
enum class PathResult : std::uint8_t {
    Ok,
    ZeroLength,      ///< degenerate but valid; planner may skip it
    RadiusMismatch,  ///< start and end are not equidistant from the centre
    InvalidRadius,   ///< centre coincides with the start point
};

/// A geometric path segment. Trivially copyable by design.
struct PathSegment {
    SegmentType type = SegmentType::Linear;

    Vec3 start{};
    Vec3 end{};

    // --- arc-only fields ---
    Vec3 centre{};
    Plane plane = Plane::XY;
    bool clockwise = false;
    double radius = 0.0;
    double theta_start = 0.0;  ///< radians, in-plane
    double sweep = 0.0;        ///< radians, signed; negative for clockwise
    double helix_rise = 0.0;   ///< displacement along the plane normal

    double length_ = 0.0;

    [[nodiscard]] double length() const noexcept { return length_; }

    /// Point at arc-length s along the segment. s is clamped to [0, length].
    [[nodiscard]] Vec3 point_at(double s) const noexcept;

    /// Unit tangent at arc-length s. Zero vector for a degenerate segment.
    [[nodiscard]] Vec3 tangent_at(double s) const noexcept;

    /// Second derivative of position with respect to arc length, d²p/ds².
    ///
    /// This is the curvature vector: zero on a line, and on an arc it points at
    /// the centre with magnitude 1/r. It is what supplies the centripetal term
    /// when converting path acceleration into per-axis acceleration:
    ///
    ///     a_axis = (d²p/ds²) * v_path²  +  tangent * a_path
    ///
    /// Omitting it makes torque feedforward wrong on every arc — exactly where
    /// the feedforward matters most.
    [[nodiscard]] Vec3 second_derivative_at(double s) const noexcept;

    /// Highest path speed that keeps every axis inside its own velocity limit,
    /// and — for arcs — inside the centripetal and chord-error constraints.
    ///
    /// For a line the tangent is constant, so the bound is exact. For an arc the
    /// tangent sweeps, so the axis bound is taken over the worst direction the
    /// arc actually reaches, which is conservative but never optimistic.
    [[nodiscard]] double max_path_velocity(const AxisLimits& limits,
                                           const PathConstraints& constraints) const noexcept;

    /// Tightest acceleration limit across the axes this segment actually moves.
    [[nodiscard]] double max_path_acceleration(const AxisLimits& limits) const noexcept;

    /// Tightest jerk limit across the axes this segment actually moves.
    /// Returns 0 (unlimited) if any moving axis has no jerk limit set.
    [[nodiscard]] double max_path_jerk(const AxisLimits& limits) const noexcept;
};

/// Build a straight segment.
[[nodiscard]] PathResult make_linear(PathSegment& seg, const Vec3& from, const Vec3& to) noexcept;

/// Build a circular or helical segment, centre format (G2/G3 with I/J/K).
///
/// @param clockwise  as seen looking down the plane normal, matching G2
/// @param full_turns extra whole revolutions beyond the shorter sweep; 0 for a
///                   normal arc. When `from` and `to` coincide the arc is a full
///                   circle regardless.
[[nodiscard]] PathResult make_arc(PathSegment& seg, const Vec3& from, const Vec3& to,
                                  const Vec3& centre, Plane plane, bool clockwise,
                                  int full_turns = 0) noexcept;

/// Axis index pair for the plane, plus the normal axis.
/// XY -> (0,1) normal 2;  ZX -> (2,0) normal 1;  YZ -> (1,2) normal 0.
void plane_axes(Plane p, int& u, int& v, int& w) noexcept;

/// Cosine of the angle between two segments at their junction, in [-1, 1].
/// 1 means collinear, -1 a full reversal. Used by look-ahead for corner speed.
[[nodiscard]] double junction_cosine(const PathSegment& a, const PathSegment& b) noexcept;

}  // namespace frcnc::motion
