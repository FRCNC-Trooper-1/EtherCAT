// Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.
//
// Real-time thread configuration and cycle timing.
//
// This header has no dependencies beyond libc and pthreads — deliberately. It is
// the lowest layer of the controller and the one whose correctness everything
// else rests on, so it stays auditable in isolation. See docs/05-motion-architecture.md §9.

#pragma once

#include <cstddef>
#include <cstdint>
#include <ctime>
#include <string>

namespace frcnc::rt {

/// Configuration for a real-time thread.
struct RtConfig {
    /// CPU to pin to. Must be an isolated core (see docs/02 §4).
    int cpu = 2;

    /// SCHED_FIFO priority. 80-90 is the usual band for a cyclic fieldbus task:
    /// above the NIC softirq threads, below nothing that matters.
    int priority = 85;

    /// Stack bytes to pre-fault before going real-time.
    std::size_t stack_prefault_bytes = 512 * 1024;

    /// Heap bytes to pre-fault before going real-time.
    std::size_t heap_prefault_bytes = 8 * 1024 * 1024;

    /// Lock all memory. Leave enabled; a page fault in the cyclic loop is a
    /// missed cycle. Configurable only so tests can run unprivileged.
    bool lock_memory = true;
};

/// Prepare the calling thread for real-time execution.
///
/// Locks memory, pre-faults stack and heap, pins to the configured CPU, and
/// switches to SCHED_FIFO. Call this once, from the thread that will run the
/// cyclic loop, and **after** all allocation is complete.
///
/// Returns an empty string on success, or a human-readable description of what
/// failed. Failure is not always fatal — running without RT privileges is useful
/// for development — so the caller decides how to react.
std::string configure_current_thread(const RtConfig& cfg);

/// Statistics for a running cyclic loop.
///
/// Jitter is measured as (actual wake time - intended deadline). It is always
/// >= 0 in practice: the kernel does not wake a thread early.
///
/// **Read max_jitter_ns, not mean_jitter_ns.** A loop that is on time 99.99% of
/// the time and 400 us late once an hour will fault a servo drive once an hour.
/// The tail is what matters; the average tells you nothing. (docs/02 §1)
struct CycleStats {
    std::uint64_t cycles = 0;

    std::int64_t min_jitter_ns = INT64_MAX;
    std::int64_t max_jitter_ns = 0;
    std::int64_t mean_jitter_ns = 0;

    /// Cycles whose jitter exceeded the configured overrun threshold.
    std::uint64_t overruns = 0;

    /// Longest observed execution time of the loop body, i.e. the time between
    /// returning from wait_next() and calling it again. If this approaches the
    /// cycle period, the loop is doing too much work.
    std::int64_t max_exec_ns = 0;

    void reset();

    /// Human-readable one-line summary.
    std::string to_string() const;
};

/// Absolute-deadline cycle timer.
///
/// Sleeps to an absolute next-wake time via clock_nanosleep(TIMER_ABSTIME).
/// Never use a relative sleep for a control loop: it accumulates the execution
/// time of the loop body as drift, every single cycle.
///
/// Typical use:
///
///     CycleTimer timer(1'000'000);   // 1 ms
///     timer.start();
///     for (;;) {
///         timer.wait_next();
///         // ... exchange process data, run control ...
///         timer.mark_work_done();
///     }
class CycleTimer {
public:
    /// @param period_ns   cycle period in nanoseconds
    /// @param overrun_threshold_ns  jitter above this counts as an overrun
    explicit CycleTimer(std::int64_t period_ns, std::int64_t overrun_threshold_ns = 100'000);

    /// Establish the first deadline. Call immediately before entering the loop.
    void start();

    /// Sleep until the next deadline.
    /// @return jitter in nanoseconds (actual wake - intended deadline).
    std::int64_t wait_next();

    /// Record that the loop body has finished, for execution-time statistics.
    /// Optional — omit it and max_exec_ns simply stays zero.
    void mark_work_done();

    /// Apply a phase correction to the next deadline.
    ///
    /// This is where the Distributed Clocks drift controller feeds in: it nudges
    /// the master cycle into phase with the slaves' Sync0 pulse. See
    /// docs/03-ethercat-bringup.md §6.
    void adjust(std::int64_t offset_ns);

    const CycleStats& stats() const { return stats_; }

    void reset_stats() { stats_.reset(); }

    std::int64_t period_ns() const { return period_ns_; }

private:
    std::int64_t period_ns_;
    std::int64_t overrun_threshold_ns_;
    std::int64_t offset_ns_ = 0;

    struct timespec deadline_ {};

    CycleStats stats_;
    std::int64_t jitter_sum_ns_ = 0;
};

/// Add nanoseconds to a timespec, normalizing the result.
void timespec_add_ns(struct timespec& ts, std::int64_t ns);

/// Difference b - a in nanoseconds.
std::int64_t timespec_diff_ns(const struct timespec& a, const struct timespec& b);

}  // namespace frcnc::rt
