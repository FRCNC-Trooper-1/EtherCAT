// Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.

#include "frcnc/rt/cyclic_task.hpp"

#include <malloc.h>
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <vector>

namespace frcnc::rt {

namespace {

constexpr std::int64_t kNsPerSec = 1'000'000'000;

/// Touch every page of a stack-allocated region so it is resident before the
/// thread goes real-time. Marked noinline and volatile so the compiler cannot
/// optimize the whole thing away — the side effect *is* the point.
[[gnu::noinline]] void prefault_stack(std::size_t bytes) {
    if (bytes == 0) {
        return;
    }
    // Deliberately a VLA-ish alloca-style buffer on the stack.
    std::vector<char> dummy;  // not used; real work is below
    (void)dummy;

    // Touch pages by walking down the stack in page-sized steps.
    constexpr std::size_t kPage = 4096;
    volatile char probe[kPage];
    std::memset(const_cast<char*>(probe), 0, kPage);

    if (bytes > kPage) {
        prefault_stack(bytes - kPage);
    }
}

std::string errno_str(const char* what) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s: %s", what, std::strerror(errno));
    return std::string(buf);
}

}  // namespace

void timespec_add_ns(struct timespec& ts, std::int64_t ns) {
    ts.tv_nsec += ns;
    while (ts.tv_nsec >= kNsPerSec) {
        ts.tv_nsec -= kNsPerSec;
        ts.tv_sec += 1;
    }
    while (ts.tv_nsec < 0) {
        ts.tv_nsec += kNsPerSec;
        ts.tv_sec -= 1;
    }
}

std::int64_t timespec_diff_ns(const struct timespec& a, const struct timespec& b) {
    return (static_cast<std::int64_t>(b.tv_sec) - static_cast<std::int64_t>(a.tv_sec)) * kNsPerSec +
           (static_cast<std::int64_t>(b.tv_nsec) - static_cast<std::int64_t>(a.tv_nsec));
}

std::string configure_current_thread(const RtConfig& cfg) {
    // 1. Lock memory. A major page fault in the cyclic loop costs milliseconds.
    if (cfg.lock_memory) {
        if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
            return errno_str("mlockall failed (need CAP_IPC_LOCK or a memlock limit)");
        }
    }

    // 2. Stop the allocator from returning memory to the OS, so that a later
    //    free/alloc cycle cannot trigger brk()/mmap() from the RT thread.
    mallopt(M_TRIM_THRESHOLD, -1);
    mallopt(M_MMAP_MAX, 0);

    // 3. Pre-fault the heap: allocate, touch, release. Because of the mallopt
    //    calls above the pages stay mapped to this process.
    if (cfg.heap_prefault_bytes > 0) {
        void* p = std::malloc(cfg.heap_prefault_bytes);
        if (p != nullptr) {
            std::memset(p, 0, cfg.heap_prefault_bytes);
            std::free(p);
        }
    }

    // 4. Pre-fault the stack.
    prefault_stack(cfg.stack_prefault_bytes);

    // 5. Pin to the isolated core.
    if (cfg.cpu < 0) {
        return "invalid CPU index";
    }
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(static_cast<unsigned>(cfg.cpu), &set);
    if (pthread_setaffinity_np(pthread_self(), sizeof(set), &set) != 0) {
        return errno_str("pthread_setaffinity_np failed");
    }

    // 6. SCHED_FIFO. Do this last: once we are real-time, a mistake above would
    //    be much harder to recover from.
    struct sched_param param {};
    param.sched_priority = cfg.priority;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
        return errno_str("pthread_setschedparam(SCHED_FIFO) failed (need CAP_SYS_NICE)");
    }

    return {};
}

void CycleStats::reset() {
    *this = CycleStats{};
}

std::string CycleStats::to_string() const {
    char buf[256];
    std::snprintf(buf, sizeof(buf),
                  "cycles=%llu jitter min/mean/max = %lld/%lld/%lld ns  "
                  "max_exec=%lld ns  overruns=%llu",
                  static_cast<unsigned long long>(cycles),
                  static_cast<long long>(min_jitter_ns == INT64_MAX ? 0 : min_jitter_ns),
                  static_cast<long long>(mean_jitter_ns), static_cast<long long>(max_jitter_ns),
                  static_cast<long long>(max_exec_ns),
                  static_cast<unsigned long long>(overruns));
    return std::string(buf);
}

CycleTimer::CycleTimer(std::int64_t period_ns, std::int64_t overrun_threshold_ns)
    : period_ns_(period_ns), overrun_threshold_ns_(overrun_threshold_ns) {}

void CycleTimer::start() {
    clock_gettime(CLOCK_MONOTONIC, &deadline_);
    stats_.reset();
    jitter_sum_ns_ = 0;
}

std::int64_t CycleTimer::wait_next() {
    // Advance to the next absolute deadline. Any DC phase correction applies here.
    timespec_add_ns(deadline_, period_ns_ + offset_ns_);
    offset_ns_ = 0;

    // Absolute sleep. TIMER_ABSTIME is what makes this drift-free: the deadline
    // is independent of how long the loop body took.
    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline_, nullptr) == EINTR) {
        // Interrupted by a signal — resume waiting for the same deadline.
    }

    struct timespec now {};
    clock_gettime(CLOCK_MONOTONIC, &now);

    const std::int64_t jitter = timespec_diff_ns(deadline_, now);

    stats_.cycles++;
    if (jitter < stats_.min_jitter_ns) {
        stats_.min_jitter_ns = jitter;
    }
    if (jitter > stats_.max_jitter_ns) {
        stats_.max_jitter_ns = jitter;
    }
    if (jitter > overrun_threshold_ns_) {
        stats_.overruns++;
    }
    jitter_sum_ns_ += jitter;
    stats_.mean_jitter_ns = jitter_sum_ns_ / static_cast<std::int64_t>(stats_.cycles);

    return jitter;
}

void CycleTimer::mark_work_done() {
    struct timespec now {};
    clock_gettime(CLOCK_MONOTONIC, &now);

    // Time from the intended deadline to now — i.e. jitter plus loop body time.
    const std::int64_t elapsed = timespec_diff_ns(deadline_, now);
    if (elapsed > stats_.max_exec_ns) {
        stats_.max_exec_ns = elapsed;
    }
}

void CycleTimer::adjust(std::int64_t offset_ns) {
    offset_ns_ = offset_ns;
}

}  // namespace frcnc::rt
