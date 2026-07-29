// Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.
//
// What crosses the planner/cyclic-task boundary.
//
// Everything here is trivially copyable by requirement, not by accident: these
// types are handed between threads by raw copy, with no construction and no
// pointer chasing. A std::string or a heap-owning container in any of them
// would put an allocation in the real-time path.

#pragma once

#include "frcnc/ipc/spsc_ring.hpp"

#include <cstdint>

namespace frcnc::ipc {

inline constexpr int kMaxAxes = 3;

/// Setpoints for one cycle, produced by the planner, consumed by the cyclic
/// task and written into the drives' PDOs.
struct AxisSetpoint {
    double position[kMaxAxes] = {};      ///< machine units -> 0x607A
    double velocity[kMaxAxes] = {};      ///< units/s       -> 0x60B1
    double acceleration[kMaxAxes] = {};  ///< units/s^2     -> 0x60B2

    /// Monotonic sequence number, for detecting a gap the planner failed to fill.
    std::uint64_t sequence = 0;

    /// Program block this sample belongs to, for status reporting.
    std::uint32_t block_id = 0;

    /// Last sample of the queued path. The cyclic task holds position after it.
    bool end_of_path = false;
};

/// What the operator asked for. Planner to cyclic task, out of band from
/// setpoints so a stop is not stuck behind a full setpoint queue.
enum class CommandKind : std::uint8_t {
    None,
    Enable,           ///< drive axes to CiA 402 Operation Enabled
    Disable,          ///< walk axes down to a safe state
    Stop,             ///< controlled stop on the programmed path
    EmergencyStop,    ///< fastest safe stop, then disable
    ClearFault,       ///< fault reset on every axis
    HoldFeed,
    ResumeFeed,
};

struct Command {
    CommandKind kind = CommandKind::None;
    std::uint64_t sequence = 0;
    double parameter = 0.0;  ///< feed override fraction, etc.
};

/// Per-axis condition, reported every cycle.
struct AxisStatus {
    double position_actual = 0.0;
    double position_command = 0.0;
    double following_error = 0.0;
    double velocity_actual = 0.0;

    std::uint16_t statusword = 0;
    std::int8_t mode_display = 0;

    bool enabled = false;
    bool faulted = false;
    bool warning = false;
    bool limit_active = false;
};

/// Machine condition, written by the cyclic task once per cycle and read by the
/// planner and HMI. Latest-value semantics: nobody wants a backlog of stale
/// positions, they want the current one.
struct MachineStatus {
    AxisStatus axis[kMaxAxes] = {};

    std::uint64_t cycle = 0;
    std::uint64_t setpoint_sequence = 0;
    std::uint32_t block_id = 0;

    // --- fieldbus health ---
    int working_counter = 0;
    int expected_wkc = 0;
    bool bus_operational = false;
    bool wkc_ok = false;
    std::uint64_t wkc_errors = 0;
    std::uint32_t consecutive_wkc_errors = 0;

    // --- timing ---
    std::int64_t dc_error_ns = 0;
    std::int64_t cycle_jitter_ns = 0;
    std::int64_t max_cycle_jitter_ns = 0;
    bool dc_locked = false;

    // --- motion ---
    double path_velocity = 0.0;
    bool motion_active = false;
    bool feed_held = false;

    /// Set when the cyclic task had no setpoint to consume. A starved cycle is
    /// a planner that could not keep up, and on a machine it means the tool
    /// stops mid-cut with the spindle still turning.
    bool starved = false;
    std::uint64_t starve_events = 0;

    /// True when any axis faulted, the bus dropped, or the loop overran. The
    /// cyclic task stops all axes together on this; a single-axis stop leaves
    /// the others interpolating against a stopped one.
    bool fault_active = false;
};

// --- concrete channel types -------------------------------------------------

/// Setpoint queue depth.
///
/// This is a latency/robustness trade, not a throughput one. Deeper survives a
/// longer planner stall without starving; shallower means an operator's stop
/// takes effect sooner, since already-queued setpoints are consumed first.
/// 64 cycles is 64 ms at 1 ms, or 16 ms at 250 us.
inline constexpr std::size_t kSetpointQueueDepth = 64;

inline constexpr std::size_t kCommandQueueDepth = 16;

using SetpointQueue = SpscRing<AxisSetpoint, kSetpointQueueDepth>;
using CommandQueue = SpscRing<Command, kCommandQueueDepth>;
using StatusChannel = LatestValue<MachineStatus>;

}  // namespace frcnc::ipc
