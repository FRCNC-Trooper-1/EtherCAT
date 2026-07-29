// Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.
//
// Look-ahead: velocity planning across a queue of path segments.
//
// Without it, every segment must start and end at zero velocity, because the
// planner cannot know whether the next block continues in the same direction.
// On a program of thousands of short moves — which is what any CAM system
// emits for a curved surface — the machine never reaches commanded feed and
// the cut takes many times longer than it should.
//
// Two passes over the queue:
//
//   backward   from the end, compute the highest velocity each segment may be
//              ENTERED with while still being able to decelerate to its
//              required exit velocity.
//
//   forward    from the start, clamp those entry velocities to what is actually
//              REACHABLE given where the previous segment left off.
//
// Between the passes sits the junction velocity: how fast the tool may carry
// through the corner between two segments. That is bounded by how much the
// path is allowed to deviate from the exact corner.
//
// Reference: docs/05-motion-architecture.md §4.5, §4.6

#pragma once

#include "frcnc/motion/path.hpp"
#include "frcnc/motion/segment_motion.hpp"

#include <cstdint>

namespace frcnc::motion {

/// Corner-rounding policy.
struct JunctionPolicy {
    /// How far the actual path may deviate from the programmed corner, in
    /// machine units. Larger values allow faster cornering and rounder corners.
    /// This is the user-facing path tolerance — the G64 P-value equivalent.
    double deviation = 0.01;

    /// Optional cap on the instantaneous per-axis velocity change at a
    /// junction. A corner reverses some axes, and the step in that axis's
    /// velocity is a shock to the mechanics regardless of what the path
    /// tolerance permits. 0 disables the check.
    double max_axis_velocity_jump = 0.0;

    /// Combine consecutive collinear linear segments into one before planning.
    ///
    /// This is not an optimisation — it is required for correctness of the
    /// result. A jerk-limited profile must ramp acceleration 0 -> A -> 0 WITHIN
    /// each segment, so a chain of short segments discards the ramp at every
    /// boundary and never sustains full acceleration. Measured on a 1 mm
    /// segment at a = 3000, j = 30000: reachable velocity gain falls from
    /// 31 mm/s at rest to 0.23 mm/s at 180 mm/s. Merging 200 such segments
    /// into one 200 mm move lifts the reachable velocity from ~131 to ~956.
    bool merge_collinear = true;

    /// Cosine threshold for treating two segments as collinear. Deliberately
    /// strict: a CAM-generated curve is a chain of nearly-collinear chords, and
    /// merging those would flatten the curve.
    double merge_cosine = 1.0 - 1e-9;
};

/// Highest velocity reachable over `distance` under jerk-limited acceleration.
///
/// The constant-acceleration form sqrt(v0^2 + 2ad) OVERESTIMATES when jerk is
/// bounded, because ramping acceleration up and down costs distance. Look-ahead
/// must use the same model as the profile generator, or it will promise
/// velocities the profile then rejects as infeasible at runtime.
[[nodiscard]] double max_reachable_velocity_jerk(double v_start, double distance,
                                                 double max_accel, double max_jerk) noexcept;

/// Velocity permitted through the junction between two segments.
///
/// The corner is modelled as an arc tangent to both segments, deviating from
/// the vertex by `policy.deviation`. Collinear segments carry full speed;
/// a reversal forces a stop.
[[nodiscard]] double junction_velocity(const PathSegment& a, const PathSegment& b,
                                       double max_accel, const JunctionPolicy& policy) noexcept;

/// A bounded queue of segments with velocity planning across them.
///
/// Fixed capacity, no allocation. Planning runs in the non-real-time domain;
/// the cyclic task consumes the planned SegmentMotion objects.
///
/// Large — roughly 150 kB at default capacity. Allocate statically or on the
/// heap, never on the stack.
class LookAhead {
public:
    static constexpr int kCapacity = 256;

    void configure(const AxisLimits& axes, const PathConstraints& constraints,
                   const JunctionPolicy& policy) noexcept;

    void clear() noexcept;

    /// Append a segment. Returns false if the queue is full or the segment is
    /// degenerate. `requested_feed` is the programmed F value; 0 means "as fast
    /// as the constraints allow".
    bool push(const PathSegment& seg, double requested_feed = 0.0) noexcept;

    [[nodiscard]] int size() const noexcept { return count_; }
    [[nodiscard]] bool empty() const noexcept { return count_ == 0; }
    [[nodiscard]] bool full() const noexcept { return count_ >= kCapacity; }

    /// Run both passes and plan every segment.
    ///
    /// @param start_velocity     speed entering the first segment
    /// @param terminal_velocity  speed leaving the last; 0 to come to rest
    /// @return false if any segment failed to plan
    bool plan(double start_velocity = 0.0, double terminal_velocity = 0.0) noexcept;

    [[nodiscard]] const SegmentMotion& motion(int i) const noexcept { return motion_[i]; }
    [[nodiscard]] const PathSegment& segment(int i) const noexcept { return segment_[i]; }

    /// Planned entry/exit velocity of segment i, after both passes.
    [[nodiscard]] double entry_velocity(int i) const noexcept { return entry_[i]; }
    [[nodiscard]] double exit_velocity(int i) const noexcept { return exit_[i]; }

    /// Total time to traverse the whole queue.
    [[nodiscard]] double total_duration() const noexcept;

    /// Total path length queued.
    [[nodiscard]] double total_length() const noexcept;

    /// How many pushed segments were absorbed into a predecessor by merging.
    [[nodiscard]] int merged_count() const noexcept { return merged_; }

    /// Sample the queue at absolute time t, walking segment boundaries.
    [[nodiscard]] AxisCommand at(double t) const noexcept;

private:
    AxisLimits axes_{};
    PathConstraints constraints_{};
    JunctionPolicy policy_{};

    PathSegment segment_[kCapacity]{};
    SegmentMotion motion_[kCapacity]{};
    double feed_[kCapacity]{};
    double entry_[kCapacity]{};
    double exit_[kCapacity]{};
    double vmax_[kCapacity]{};

    int count_ = 0;
    int merged_ = 0;
    bool planned_ = false;
};

}  // namespace frcnc::motion
