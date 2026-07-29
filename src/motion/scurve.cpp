// Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.

#include "frcnc/motion/scurve.hpp"

#include <cmath>

namespace frcnc::motion {

namespace {

constexpr double kEps = 1e-9;

constexpr double clamp(double v, double lo, double hi) noexcept {
    return v < lo ? lo : (v > hi ? hi : v);
}

}  // namespace

Ramp compute_ramp(double dv, double max_accel, double max_jerk) noexcept {
    Ramp r{};

    const double d = std::fabs(dv);
    if (d <= kEps || max_accel <= kEps) {
        return r;
    }

    // No jerk limit: a single constant-acceleration segment.
    if (max_jerk <= kEps) {
        r.t_const = d / max_accel;
        r.total = r.t_const;
        r.peak_accel = max_accel;
        return r;
    }

    // Velocity change achievable with a purely triangular acceleration profile,
    // i.e. jerking straight up to max_accel and straight back down:
    //   dv_tri = 2 * (a^2 / 2j) = a^2 / j
    const double dv_tri = (max_accel * max_accel) / max_jerk;

    if (d >= dv_tri) {
        // Acceleration limit is reached; a constant-acceleration phase exists.
        r.t_jerk = max_accel / max_jerk;
        r.t_const = (d - dv_tri) / max_accel;
        r.peak_accel = max_accel;
    } else {
        // Triangular: acceleration peaks below the limit.
        r.peak_accel = std::sqrt(d * max_jerk);
        r.t_jerk = r.peak_accel / max_jerk;
        r.t_const = 0.0;
    }

    r.total = 2.0 * r.t_jerk + r.t_const;
    return r;
}

double ramp_distance(double v_from, double v_to, double max_accel, double max_jerk) noexcept {
    const Ramp r = compute_ramp(v_to - v_from, max_accel, max_jerk);
    // Mean velocity over a symmetric ramp is exactly the midpoint.
    return 0.5 * (v_from + v_to) * r.total;
}

void SCurveProfile::reset() noexcept {
    *this = SCurveProfile{};
}

PlanResult SCurveProfile::plan(double distance, double entry_vel, double exit_vel,
                               const Limits& limits) noexcept {
    reset();

    if (distance < 0.0) {
        return PlanResult::NegativeDistance;
    }
    if (limits.max_velocity <= kEps || limits.max_acceleration <= kEps) {
        return PlanResult::InvalidLimits;
    }
    if (distance <= kEps) {
        valid_ = true;
        return PlanResult::ZeroLength;
    }

    const double v_max = limits.max_velocity;
    const double a_max = limits.max_acceleration;
    const double j_max = limits.max_jerk;

    const double v0 = clamp(entry_vel, 0.0, v_max);
    const double v1 = clamp(exit_vel, 0.0, v_max);

    // Distance required for a given peak velocity, accelerating v0 -> v then
    // decelerating v -> v1. Monotonically increasing in v, which is what makes
    // the bisection below valid.
    const auto required_distance = [&](double v) noexcept {
        return ramp_distance(v0, v, a_max, j_max) + ramp_distance(v, v1, a_max, j_max);
    };

    // The cheapest possible peak is max(v0, v1): any lower and one of the ramps
    // would have to reverse direction, which costs more distance, not less.
    const double v_lo = (v0 > v1) ? v0 : v1;

    if (required_distance(v_lo) > distance + kEps) {
        // Cannot even get from v0 to v1 in this distance under these limits.
        return PlanResult::Infeasible;
    }

    double v_peak = v_max;
    double d_cruise = 0.0;

    const double d_at_max = required_distance(v_max);
    if (d_at_max <= distance) {
        // Room to cruise at the velocity limit.
        d_cruise = distance - d_at_max;
    } else {
        // No cruise phase: solve for the peak that exactly fills the distance.
        //
        // required_distance() is continuous and monotonically increasing in v,
        // so bisection converges reliably. There is no closed form once the
        // ramps switch between triangular and trapezoidal shapes, and planning
        // runs off the real-time path, so this is the right trade.
        double lo = v_lo;
        double hi = v_max;
        for (int i = 0; i < 100; i++) {
            const double mid = 0.5 * (lo + hi);
            if (required_distance(mid) > distance) {
                hi = mid;
            } else {
                lo = mid;
            }
        }
        v_peak = lo;
        d_cruise = distance - required_distance(v_peak);
        if (d_cruise < 0.0) {
            d_cruise = 0.0;
        }
    }

    const Ramp up = compute_ramp(v_peak - v0, a_max, j_max);
    const Ramp down = compute_ramp(v_peak - v1, a_max, j_max);

    const double t_cruise = (v_peak > kEps) ? (d_cruise / v_peak) : 0.0;

    // Build the phase list. Zero-duration phases are dropped so that sampling
    // never has to special-case them.
    const double jerk_up = (j_max > kEps) ? j_max : 0.0;

    struct Spec {
        double duration;
        double jerk;
    };
    const Spec specs[kMaxPhases] = {
        {up.t_jerk, +jerk_up},    // 1 jerk up
        {up.t_const, 0.0},        // 2 const accel
        {up.t_jerk, -jerk_up},    // 3 jerk down
        {t_cruise, 0.0},          // 4 cruise
        {down.t_jerk, -jerk_up},  // 5 jerk down
        {down.t_const, 0.0},      // 6 const decel
        {down.t_jerk, +jerk_up},  // 7 jerk up
    };

    // With no jerk limit the ramps collapse to a single constant-acceleration
    // phase each, expressed through phases 2 and 6.
    double a_now = 0.0;
    if (j_max <= kEps) {
        // Phase 2 accelerates, phase 6 decelerates.
        // Signs are applied below via the phase-entry acceleration.
    }

    double t = 0.0;
    double p = 0.0;
    double v = v0;
    int n = 0;

    for (int i = 0; i < kMaxPhases; i++) {
        double dur = specs[i].duration;
        if (dur <= kEps) {
            continue;
        }
        double jerk = specs[i].jerk;

        // Constant-acceleration phases carry the acceleration reached by the
        // preceding jerk phase. Without a jerk limit there is no preceding jerk
        // phase, so set it explicitly.
        if (i == 1) {  // const accel
            a_now = (j_max > kEps) ? a_now : +up.peak_accel;
        } else if (i == 3) {  // cruise
            a_now = 0.0;
        } else if (i == 5) {  // const decel
            a_now = (j_max > kEps) ? a_now : -down.peak_accel;
        }

        phases_[n].duration = dur;
        phases_[n].jerk = jerk;
        phases_[n].t0 = t;
        phases_[n].p0 = p;
        phases_[n].v0 = v;
        phases_[n].a0 = a_now;

        // Advance exact closed-form state to the end of this phase.
        const double dt = dur;
        p += v * dt + 0.5 * a_now * dt * dt + (jerk * dt * dt * dt) / 6.0;
        v += a_now * dt + 0.5 * jerk * dt * dt;
        a_now += jerk * dt;

        t += dt;
        n++;
    }

    distance_ = distance;
    v_entry_ = v0;
    v_exit_ = v1;
    v_peak_ = v_peak;
    a_peak_ = (up.peak_accel > down.peak_accel) ? up.peak_accel : down.peak_accel;
    t_total_ = t;
    phase_count_ = n;
    valid_ = true;

    return PlanResult::Ok;
}

Sample SCurveProfile::at(double t) const noexcept {
    Sample s{};

    if (!valid_ || phase_count_ == 0 || t_total_ <= kEps) {
        s.position = distance_;  // zero-length move: already at the end
        return s;
    }

    if (t <= 0.0) {
        s.position = 0.0;
        s.velocity = v_entry_;
        s.acceleration = phases_[0].a0;
        return s;
    }
    if (t >= t_total_) {
        s.position = distance_;
        s.velocity = v_exit_;
        s.acceleration = 0.0;
        return s;
    }

    // Linear scan: at most 7 phases, so this is cheaper than a binary search
    // and has no branch-misprediction surprises in the cyclic path.
    const Phase* ph = &phases_[phase_count_ - 1];
    for (int i = 0; i < phase_count_; i++) {
        if (t < phases_[i].t0 + phases_[i].duration) {
            ph = &phases_[i];
            break;
        }
    }

    const double dt = t - ph->t0;
    s.acceleration = ph->a0 + ph->jerk * dt;
    s.velocity = ph->v0 + ph->a0 * dt + 0.5 * ph->jerk * dt * dt;
    s.position = ph->p0 + ph->v0 * dt + 0.5 * ph->a0 * dt * dt +
                 (ph->jerk * dt * dt * dt) / 6.0;

    if (s.velocity < 0.0) {
        s.velocity = 0.0;
    }
    if (s.position > distance_) {
        s.position = distance_;
    }
    return s;
}

}  // namespace frcnc::motion
