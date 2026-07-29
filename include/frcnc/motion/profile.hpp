// Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.
//
// Scalar motion profile generation.
//
// A profile plans motion along a single scalar distance — either one axis, or
// the arc-length parameter of a multi-axis path. Coordinated moves are built by
// planning one profile along the path and mapping it onto axes, which is what
// keeps all axes starting and finishing together.
//
// Pure math: no allocation, no I/O, no dependency on the fieldbus. Planning
// happens in the non-RT domain; sampling is cheap enough for the cyclic path.
//
// Reference: docs/05-motion-architecture.md §4

#pragma once

#include <cstdint>

namespace frcnc::motion {

/// Kinematic limits for a move.
struct Limits {
    double max_velocity = 0.0;      ///< units/s, must be > 0
    double max_acceleration = 0.0;  ///< units/s^2, must be > 0
    double max_jerk = 0.0;          ///< units/s^3; 0 means unlimited (trapezoidal)
};

/// State at a point in time.
struct Sample {
    double position = 0.0;      ///< units, from the start of the move
    double velocity = 0.0;      ///< units/s
    double acceleration = 0.0;  ///< units/s^2
};

/// Why planning failed.
enum class PlanResult : std::uint8_t {
    Ok,
    ZeroLength,        ///< distance is zero; profile is a no-op but valid
    InvalidLimits,     ///< non-positive velocity or acceleration limit
    NegativeDistance,  ///< distance < 0; callers should pass magnitude and sign separately
    Infeasible,        ///< cannot reach the requested end velocity within the distance
};

/// Trapezoidal (constant-acceleration) velocity profile.
///
/// Three phases: accelerate from the entry velocity to a peak, cruise, then
/// decelerate to the exit velocity. When the distance is too short to reach
/// max_velocity the cruise phase vanishes and the profile becomes triangular —
/// the peak is solved for rather than clamped.
///
/// Entry and exit velocities are non-zero in normal use: that is what look-ahead
/// produces, and what lets a machine carry speed through a corner instead of
/// stopping at every block boundary.
class TrapezoidalProfile {
public:
    /// Plan a move. Safe to call repeatedly; each call fully replaces the plan.
    ///
    /// @param distance   total distance, must be >= 0
    /// @param entry_vel  velocity at t = 0, clamped to [0, max_velocity]
    /// @param exit_vel   target velocity at the end, clamped to [0, max_velocity]
    PlanResult plan(double distance, double entry_vel, double exit_vel,
                    const Limits& limits) noexcept;

    /// True when the last plan() produced a usable profile.
    [[nodiscard]] bool valid() const noexcept { return valid_; }

    /// Total move duration in seconds. Zero for a zero-length move.
    [[nodiscard]] double duration() const noexcept { return t_total_; }

    /// Total distance covered.
    [[nodiscard]] double distance() const noexcept { return distance_; }

    /// Peak velocity actually reached — below max_velocity on a short move.
    [[nodiscard]] double peak_velocity() const noexcept { return v_peak_; }

    /// Sample the profile. Times outside [0, duration()] are clamped, so a
    /// caller that overruns by a cycle gets the endpoint rather than nonsense.
    [[nodiscard]] Sample at(double t) const noexcept;

    /// Phase boundary times, for diagnostics and tests.
    [[nodiscard]] double accel_time() const noexcept { return t_accel_; }
    [[nodiscard]] double cruise_time() const noexcept { return t_cruise_; }
    [[nodiscard]] double decel_time() const noexcept { return t_decel_; }

    void reset() noexcept;

private:
    bool valid_ = false;

    double distance_ = 0.0;
    double v_entry_ = 0.0;
    double v_exit_ = 0.0;
    double v_peak_ = 0.0;
    double accel_ = 0.0;

    double t_accel_ = 0.0;
    double t_cruise_ = 0.0;
    double t_decel_ = 0.0;
    double t_total_ = 0.0;

    double d_accel_ = 0.0;
    double d_cruise_ = 0.0;
};

/// Shortest distance in which velocity can change from `from` to `to` under a
/// constant acceleration limit. Look-ahead uses this to decide whether a
/// junction velocity is reachable before committing to it.
[[nodiscard]] double min_distance_for_velocity_change(double from, double to,
                                                      double accel) noexcept;

/// Highest velocity reachable at the end of `distance`, starting from
/// `entry_vel`, under a constant acceleration limit.
[[nodiscard]] double max_reachable_velocity(double entry_vel, double distance,
                                            double accel) noexcept;

}  // namespace frcnc::motion
