// Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.

#include "frcnc/motion/lookahead.hpp"

#include "frcnc/motion/scurve.hpp"

#include <cmath>

namespace frcnc::motion {

namespace {

constexpr double kEps = 1e-9;

constexpr double min2(double a, double b) noexcept {
    return a < b ? a : b;
}

}  // namespace

double max_reachable_velocity_jerk(double v_start, double distance, double max_accel,
                                   double max_jerk) noexcept {
    if (distance <= kEps || max_accel <= kEps) {
        return v_start;
    }

    // Without a jerk limit the closed form is exact.
    const double v_const_accel = std::sqrt(v_start * v_start + 2.0 * max_accel * distance);
    if (max_jerk <= kEps) {
        return v_const_accel;
    }

    // With jerk bounded, the same distance yields less velocity. ramp_distance()
    // is monotonically increasing in the target velocity, so bisect between the
    // start velocity and the constant-acceleration overestimate.
    double lo = v_start;
    double hi = v_const_accel;
    for (int i = 0; i < 60; i++) {
        const double mid = 0.5 * (lo + hi);
        if (ramp_distance(v_start, mid, max_accel, max_jerk) > distance) {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    return lo;
}

double junction_velocity(const PathSegment& a, const PathSegment& b, double max_accel,
                         const JunctionPolicy& policy) noexcept {
    if (max_accel <= kEps) {
        return 0.0;
    }

    const double cos_j = junction_cosine(a, b);

    // sin(theta/2) where theta is the interior angle of the corner.
    // With phi the angle between the direction vectors, theta = pi - phi, so
    //   sin(theta/2) = cos(phi/2) = sqrt((1 + cos phi) / 2)
    double half = 0.5 * (1.0 + cos_j);
    if (half < 0.0) {
        half = 0.0;
    }
    half = std::sqrt(half);

    // Collinear: nothing to slow down for.
    if (half >= 1.0 - 1e-12) {
        return 1e300;
    }
    // Full reversal: must stop.
    if (half <= kEps) {
        return 0.0;
    }

    double v = 1e300;

    if (policy.deviation > kEps) {
        // Radius of the arc that is tangent to both segments and deviates from
        // the vertex by `deviation`.
        const double r = policy.deviation * half / (1.0 - half);
        v = std::sqrt(max_accel * r);
    }

    // Independently bound the instantaneous per-axis velocity step. A corner
    // that reverses an axis subjects it to a shock the path tolerance says
    // nothing about.
    if (policy.max_axis_velocity_jump > kEps) {
        const Vec3 ta = a.tangent_at(a.length());
        const Vec3 tb = b.tangent_at(0.0);

        double worst = 0.0;
        for (int i = 0; i < 3; i++) {
            const double d = std::fabs(tb[i] - ta[i]);
            if (d > worst) {
                worst = d;
            }
        }
        if (worst > kEps) {
            const double bound = policy.max_axis_velocity_jump / worst;
            v = min2(v, bound);
        }
    }

    return v;
}

void LookAhead::configure(const AxisLimits& axes, const PathConstraints& constraints,
                          const JunctionPolicy& policy) noexcept {
    axes_ = axes;
    constraints_ = constraints;
    policy_ = policy;
}

void LookAhead::clear() noexcept {
    count_ = 0;
    merged_ = 0;
    planned_ = false;
}

bool LookAhead::push(const PathSegment& seg, double requested_feed) noexcept {
    if (seg.length() <= kEps) {
        return false;  // degenerate; nothing to plan
    }

    // Absorb a collinear continuation into the previous segment.
    //
    // A jerk-limited profile ramps acceleration 0 -> A -> 0 within each
    // segment, so every boundary discards the ramp. A chain of short collinear
    // moves therefore never sustains full acceleration, and the machine crawls
    // through exactly the programs look-ahead exists to speed up. Merging first
    // removes the boundaries that carry no geometric information.
    if (count_ > 0 && policy_.merge_collinear && seg.type == SegmentType::Linear &&
        segment_[count_ - 1].type == SegmentType::Linear &&
        std::fabs(feed_[count_ - 1] - requested_feed) <= kEps &&
        distance(segment_[count_ - 1].end, seg.start) <= 1e-9 &&
        junction_cosine(segment_[count_ - 1], seg) >= policy_.merge_cosine) {
        PathSegment merged{};
        if (make_linear(merged, segment_[count_ - 1].start, seg.end) == PathResult::Ok) {
            segment_[count_ - 1] = merged;

            double v = merged.max_path_velocity(axes_, constraints_);
            if (requested_feed > kEps && requested_feed < v) {
                v = requested_feed;
            }
            vmax_[count_ - 1] = v;

            merged_++;
            planned_ = false;
            return true;
        }
    }

    if (count_ >= kCapacity) {
        return false;
    }

    segment_[count_] = seg;
    feed_[count_] = requested_feed;

    // The ceiling this segment's own geometry imposes.
    double v = seg.max_path_velocity(axes_, constraints_);
    if (requested_feed > kEps && requested_feed < v) {
        v = requested_feed;
    }
    vmax_[count_] = v;

    entry_[count_] = 0.0;
    exit_[count_] = 0.0;

    count_++;
    planned_ = false;
    return true;
}

bool LookAhead::plan(double start_velocity, double terminal_velocity) noexcept {
    if (count_ == 0) {
        planned_ = true;
        return true;
    }

    const int n = count_;

    // --- junction velocities -------------------------------------------------
    // exit_[i] starts as the corner limit between segment i and i+1; the last
    // segment exits at the caller's terminal velocity.
    for (int i = 0; i < n - 1; i++) {
        const double a_lim = segment_[i].max_path_acceleration(axes_);
        double vj = junction_velocity(segment_[i], segment_[i + 1], a_lim, policy_);
        vj = min2(vj, vmax_[i]);
        vj = min2(vj, vmax_[i + 1]);
        exit_[i] = vj;
    }
    exit_[n - 1] = min2(terminal_velocity, vmax_[n - 1]);

    // --- backward pass -------------------------------------------------------
    // Highest velocity each segment may be entered with while still able to
    // decelerate to its required exit. Walking backwards propagates a stop at
    // the end of the program into however many segments it takes to slow down.
    for (int i = n - 1; i >= 0; i--) {
        const double a_lim = segment_[i].max_path_acceleration(axes_);
        const double j_lim = segment_[i].max_path_jerk(axes_);

        const double reachable =
            max_reachable_velocity_jerk(exit_[i], segment_[i].length(), a_lim, j_lim);

        entry_[i] = min2(vmax_[i], reachable);

        if (i > 0) {
            // Segment i-1 cannot exit faster than segment i can be entered.
            exit_[i - 1] = min2(exit_[i - 1], entry_[i]);
        }
    }

    // --- forward pass --------------------------------------------------------
    // Clamp entry velocities to what is actually reachable from where the
    // previous segment left off, then propagate forward.
    double v_in = min2(start_velocity, vmax_[0]);

    bool ok = true;
    for (int i = 0; i < n; i++) {
        const double a_lim = segment_[i].max_path_acceleration(axes_);
        const double j_lim = segment_[i].max_path_jerk(axes_);

        entry_[i] = min2(entry_[i], v_in);

        const double reachable =
            max_reachable_velocity_jerk(entry_[i], segment_[i].length(), a_lim, j_lim);
        exit_[i] = min2(exit_[i], reachable);

        const PlanResult r = motion_[i].plan(segment_[i], entry_[i], exit_[i], axes_,
                                             constraints_, feed_[i]);
        if (r != PlanResult::Ok && r != PlanResult::ZeroLength) {
            ok = false;
        }

        // Whatever the segment actually ends at becomes the next entry.
        v_in = exit_[i];
    }

    planned_ = true;
    return ok;
}

double LookAhead::total_duration() const noexcept {
    double t = 0.0;
    for (int i = 0; i < count_; i++) {
        t += motion_[i].duration();
    }
    return t;
}

double LookAhead::total_length() const noexcept {
    double d = 0.0;
    for (int i = 0; i < count_; i++) {
        d += segment_[i].length();
    }
    return d;
}

AxisCommand LookAhead::at(double t) const noexcept {
    AxisCommand cmd{};
    if (count_ == 0) {
        return cmd;
    }

    if (t <= 0.0) {
        return motion_[0].at(0.0);
    }

    double acc = 0.0;
    for (int i = 0; i < count_; i++) {
        const double d = motion_[i].duration();
        if (t < acc + d) {
            cmd = motion_[i].at(t - acc);
            cmd.finished = false;
            return cmd;
        }
        acc += d;
    }

    // Past the end of the queue.
    cmd = motion_[count_ - 1].at(motion_[count_ - 1].duration());
    cmd.finished = true;
    return cmd;
}

}  // namespace frcnc::motion
