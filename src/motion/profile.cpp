// Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.

#include "frcnc/motion/profile.hpp"

#include <cmath>

namespace frcnc::motion {

namespace {

/// Distances and velocities below this are treated as zero. Chosen well below
/// any meaningful machine resolution (1 nm / 1 nm/s) but far above double
/// rounding noise for the magnitudes involved.
constexpr double kEps = 1e-9;

constexpr double clamp(double v, double lo, double hi) noexcept {
    return v < lo ? lo : (v > hi ? hi : v);
}

}  // namespace

double min_distance_for_velocity_change(double from, double to, double accel) noexcept {
    if (accel <= kEps) {
        return 0.0;
    }
    return std::fabs(to * to - from * from) / (2.0 * accel);
}

double max_reachable_velocity(double entry_vel, double distance, double accel) noexcept {
    if (distance <= 0.0 || accel <= 0.0) {
        return entry_vel;
    }
    return std::sqrt(entry_vel * entry_vel + 2.0 * accel * distance);
}

void TrapezoidalProfile::reset() noexcept {
    *this = TrapezoidalProfile{};
}

PlanResult TrapezoidalProfile::plan(double distance, double entry_vel, double exit_vel,
                                    const Limits& limits) noexcept {
    reset();

    if (distance < 0.0) {
        return PlanResult::NegativeDistance;
    }
    if (limits.max_velocity <= kEps || limits.max_acceleration <= kEps) {
        return PlanResult::InvalidLimits;
    }

    // A zero-length move is legitimate — look-ahead produces them at repeated
    // points — and must not be an error. It is simply a no-op of zero duration.
    if (distance <= kEps) {
        valid_ = true;
        return PlanResult::ZeroLength;
    }

    const double a = limits.max_acceleration;
    const double v_max = limits.max_velocity;

    const double v0 = clamp(entry_vel, 0.0, v_max);
    const double v1 = clamp(exit_vel, 0.0, v_max);

    // Can the requested velocity change even happen in this distance? If not,
    // the caller's look-ahead has produced an inconsistent plan; report it
    // rather than silently producing a profile that violates the accel limit.
    if (min_distance_for_velocity_change(v0, v1, a) > distance + kEps) {
        return PlanResult::Infeasible;
    }

    // Peak velocity for a triangular profile, i.e. the velocity at which the
    // accelerate and decelerate distances exactly fill the move:
    //     (v^2 - v0^2)/(2a) + (v^2 - v1^2)/(2a) = d
    //  => v = sqrt((v0^2 + v1^2)/2 + a*d)
    const double v_tri = std::sqrt((v0 * v0 + v1 * v1) * 0.5 + a * distance);

    // If that exceeds the velocity limit there is room to cruise.
    const double v_peak = (v_tri > v_max) ? v_max : v_tri;

    double d_acc = (v_peak * v_peak - v0 * v0) / (2.0 * a);
    double d_dec = (v_peak * v_peak - v1 * v1) / (2.0 * a);

    // Guard against tiny negatives from rounding when v_peak == v0 or v1.
    if (d_acc < 0.0) {
        d_acc = 0.0;
    }
    if (d_dec < 0.0) {
        d_dec = 0.0;
    }

    double d_cruise = distance - d_acc - d_dec;
    if (d_cruise < 0.0) {
        d_cruise = 0.0;
    }

    const double t_acc = (v_peak - v0) / a;
    const double t_dec = (v_peak - v1) / a;
    const double t_cru = (v_peak > kEps) ? (d_cruise / v_peak) : 0.0;

    distance_ = distance;
    v_entry_ = v0;
    v_exit_ = v1;
    v_peak_ = v_peak;
    accel_ = a;

    t_accel_ = (t_acc > 0.0) ? t_acc : 0.0;
    t_decel_ = (t_dec > 0.0) ? t_dec : 0.0;
    t_cruise_ = (t_cru > 0.0) ? t_cru : 0.0;
    t_total_ = t_accel_ + t_cruise_ + t_decel_;

    d_accel_ = d_acc;
    d_cruise_ = d_cruise;

    valid_ = true;
    return PlanResult::Ok;
}

Sample TrapezoidalProfile::at(double t) const noexcept {
    Sample s{};

    if (!valid_ || t_total_ <= kEps) {
        // Zero-length or unplanned: hold at the start.
        s.position = (t_total_ <= kEps) ? distance_ : 0.0;
        s.velocity = 0.0;
        s.acceleration = 0.0;
        return s;
    }

    // Clamp rather than extrapolate. A cyclic task that overruns the end of a
    // segment by one cycle should get the endpoint, not a position past it.
    if (t <= 0.0) {
        s.position = 0.0;
        s.velocity = v_entry_;
        s.acceleration = (t_accel_ > 0.0) ? accel_ : 0.0;
        return s;
    }
    if (t >= t_total_) {
        s.position = distance_;
        s.velocity = v_exit_;
        s.acceleration = 0.0;
        return s;
    }

    if (t < t_accel_) {
        s.acceleration = accel_;
        s.velocity = v_entry_ + accel_ * t;
        s.position = v_entry_ * t + 0.5 * accel_ * t * t;
        return s;
    }

    const double t_after_accel = t - t_accel_;
    if (t_after_accel < t_cruise_) {
        s.acceleration = 0.0;
        s.velocity = v_peak_;
        s.position = d_accel_ + v_peak_ * t_after_accel;
        return s;
    }

    const double td = t_after_accel - t_cruise_;
    s.acceleration = -accel_;
    s.velocity = v_peak_ - accel_ * td;
    s.position = d_accel_ + d_cruise_ + v_peak_ * td - 0.5 * accel_ * td * td;

    // Rounding can nudge these outside their physical range near the endpoint.
    if (s.velocity < 0.0) {
        s.velocity = 0.0;
    }
    if (s.position > distance_) {
        s.position = distance_;
    }
    return s;
}

}  // namespace frcnc::motion
