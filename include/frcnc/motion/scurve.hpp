// Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.
//
// Jerk-limited (S-curve) motion profile.
//
// A trapezoidal profile steps acceleration instantaneously at every phase
// boundary. That step excites every resonance in the machine structure: audible
// noise, visible chatter marks, and mechanical wear. Bounding the rate of change
// of acceleration — the jerk — removes the step.
//
// This is not a luxury feature on an industrial machine. Surface finish and
// mechanical life both depend on it.
//
// Seven phases, by the acceleration they apply:
//
//   phase │ 1      2      3      4      5      6      7
//   ──────┼──────────────────────────────────────────────
//   jerk  │ +j     0      -j     0      -j     0      +j
//   accel │ 0→+A   +A     +A→0   0      0→-A   -A     -A→0
//   vel   │ rising rising rising peak   fall   fall   fall
//
//   1 jerk up      a: 0 to +a_peak
//   2 const accel  a: +a_peak
//   3 jerk down    a: +a_peak to 0     (velocity reaches its peak)
//   4 cruise       a: 0
//   5 jerk down    a: 0 to -a_peak
//   6 const decel  a: -a_peak
//   7 jerk up      a: -a_peak to 0     (velocity reaches the exit value)
//
// Phases 2 and 6 vanish when the velocity change is small enough that the
// acceleration limit is never reached; phase 4 vanishes when the distance is
// too short to cruise.
//
// Reference: docs/05-motion-architecture.md §4.2

#pragma once

#include "frcnc/motion/profile.hpp"

#include <cstdint>

namespace frcnc::motion {

/// Jerk-limited profile along a scalar distance.
///
/// Shares Limits, Sample and PlanResult with TrapezoidalProfile, and exposes the
/// same interface, so the two are interchangeable at a call site. Setting
/// Limits::max_jerk to 0 makes this behave as a trapezoidal profile.
class SCurveProfile {
public:
    static constexpr int kMaxPhases = 7;

    /// Plan a move.
    ///
    /// @param distance   total distance, must be >= 0
    /// @param entry_vel  velocity at t = 0, clamped to [0, max_velocity]
    /// @param exit_vel   target velocity at the end, clamped to [0, max_velocity]
    PlanResult plan(double distance, double entry_vel, double exit_vel,
                    const Limits& limits) noexcept;

    [[nodiscard]] bool valid() const noexcept { return valid_; }
    [[nodiscard]] double duration() const noexcept { return t_total_; }
    [[nodiscard]] double distance() const noexcept { return distance_; }
    [[nodiscard]] double peak_velocity() const noexcept { return v_peak_; }

    /// Highest |acceleration| actually reached. Below max_acceleration when the
    /// velocity change is too small to ramp all the way up.
    [[nodiscard]] double peak_acceleration() const noexcept { return a_peak_; }

    /// Sample the profile. Times outside [0, duration()] are clamped.
    [[nodiscard]] Sample at(double t) const noexcept;

    /// Number of active phases (those with non-zero duration).
    [[nodiscard]] int phase_count() const noexcept { return phase_count_; }

    void reset() noexcept;

private:
    /// One constant-jerk segment. Integrating within a phase is exact:
    ///   a(t) = a0 + j*t
    ///   v(t) = v0 + a0*t + j*t^2/2
    ///   p(t) = p0 + v0*t + a0*t^2/2 + j*t^3/6
    struct Phase {
        double duration = 0.0;
        double jerk = 0.0;
        double t0 = 0.0;  ///< absolute start time
        double p0 = 0.0;  ///< state at phase start
        double v0 = 0.0;
        double a0 = 0.0;
    };

    bool valid_ = false;

    double distance_ = 0.0;
    double v_entry_ = 0.0;
    double v_exit_ = 0.0;
    double v_peak_ = 0.0;
    double a_peak_ = 0.0;
    double t_total_ = 0.0;

    Phase phases_[kMaxPhases]{};
    int phase_count_ = 0;
};

/// A jerk-limited ramp between two velocities.
///
/// Exposed because look-ahead needs to ask "how far does this velocity change
/// take?" without building a whole profile.
struct Ramp {
    double t_jerk = 0.0;    ///< duration of EACH of the two jerk phases
    double t_const = 0.0;   ///< duration of the constant-acceleration phase
    double total = 0.0;     ///< 2*t_jerk + t_const
    double peak_accel = 0.0;
};

/// Compute the ramp for a velocity change of magnitude |dv|.
///
/// With max_jerk <= 0 this degenerates to a constant-acceleration ramp.
[[nodiscard]] Ramp compute_ramp(double dv, double max_accel, double max_jerk) noexcept;

/// Distance covered by a jerk-limited ramp between two velocities.
///
/// Exact: the velocity curve of a symmetric jerk-limited ramp is point-symmetric
/// about its midpoint, so its mean value is exactly (v_from + v_to)/2.
[[nodiscard]] double ramp_distance(double v_from, double v_to, double max_accel,
                                   double max_jerk) noexcept;

}  // namespace frcnc::motion
