// Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.
//
// Tests for the timespec arithmetic and cycle timer. These are pure functions of
// their inputs and run anywhere — no hardware, no privileges, no network.

#include "frcnc/rt/cyclic_task.hpp"

#include <cstdio>
#include <cstdlib>

namespace {

int g_failures = 0;
int g_checks = 0;

void check(bool cond, const char* expr, const char* file, int line) {
    g_checks++;
    if (!cond) {
        g_failures++;
        std::printf("  FAIL %s:%d  %s\n", file, line, expr);
    }
}

#define CHECK(expr) check((expr), #expr, __FILE__, __LINE__)

void test_timespec_add_basic() {
    struct timespec ts {};
    ts.tv_sec = 10;
    ts.tv_nsec = 0;

    frcnc::rt::timespec_add_ns(ts, 500'000'000);
    CHECK(ts.tv_sec == 10);
    CHECK(ts.tv_nsec == 500'000'000);
}

void test_timespec_add_carries() {
    struct timespec ts {};
    ts.tv_sec = 10;
    ts.tv_nsec = 800'000'000;

    // 800ms + 400ms should carry into the next second.
    frcnc::rt::timespec_add_ns(ts, 400'000'000);
    CHECK(ts.tv_sec == 11);
    CHECK(ts.tv_nsec == 200'000'000);
}

void test_timespec_add_multi_second() {
    struct timespec ts {};
    ts.tv_sec = 0;
    ts.tv_nsec = 0;

    // A single add larger than one second must normalize fully, not leave
    // tv_nsec >= 1e9. This is the case a naive `if` instead of `while` gets wrong.
    frcnc::rt::timespec_add_ns(ts, 3'500'000'000);
    CHECK(ts.tv_sec == 3);
    CHECK(ts.tv_nsec == 500'000'000);
}

void test_timespec_add_negative() {
    struct timespec ts {};
    ts.tv_sec = 5;
    ts.tv_nsec = 100'000'000;

    // A negative DC phase correction must borrow correctly.
    frcnc::rt::timespec_add_ns(ts, -200'000'000);
    CHECK(ts.tv_sec == 4);
    CHECK(ts.tv_nsec == 900'000'000);
}

void test_timespec_diff() {
    struct timespec a {};
    struct timespec b {};

    a.tv_sec = 100;
    a.tv_nsec = 0;
    b.tv_sec = 100;
    b.tv_nsec = 250'000;
    CHECK(frcnc::rt::timespec_diff_ns(a, b) == 250'000);

    // Across a second boundary.
    a.tv_sec = 100;
    a.tv_nsec = 900'000'000;
    b.tv_sec = 101;
    b.tv_nsec = 100'000'000;
    CHECK(frcnc::rt::timespec_diff_ns(a, b) == 200'000'000);

    // Negative difference (b before a).
    a.tv_sec = 101;
    a.tv_nsec = 0;
    b.tv_sec = 100;
    b.tv_nsec = 0;
    CHECK(frcnc::rt::timespec_diff_ns(a, b) == -1'000'000'000);
}

void test_stats_reset() {
    frcnc::rt::CycleStats s;
    s.cycles = 42;
    s.max_jitter_ns = 99;
    s.overruns = 7;

    s.reset();
    CHECK(s.cycles == 0);
    CHECK(s.max_jitter_ns == 0);
    CHECK(s.overruns == 0);
}

void test_cycle_timer_runs() {
    // 1 ms cycle, generous overrun threshold — this runs unprivileged in CI on a
    // non-RT kernel, so we assert behaviour, not latency.
    frcnc::rt::CycleTimer timer(1'000'000, 100'000'000);
    timer.start();

    for (int i = 0; i < 20; i++) {
        timer.wait_next();
        timer.mark_work_done();
    }

    const auto& st = timer.stats();
    CHECK(st.cycles == 20);
    // The kernel never wakes a thread early, so jitter should not be negative.
    CHECK(st.min_jitter_ns >= 0);
    CHECK(st.max_jitter_ns >= st.min_jitter_ns);
    CHECK(st.mean_jitter_ns >= 0);
}

void test_cycle_timer_period_is_honoured() {
    constexpr std::int64_t kPeriodNs = 2'000'000;  // 2 ms
    constexpr int kCycles = 25;

    struct timespec begin {};
    clock_gettime(CLOCK_MONOTONIC, &begin);

    frcnc::rt::CycleTimer timer(kPeriodNs, 100'000'000);
    timer.start();
    for (int i = 0; i < kCycles; i++) {
        timer.wait_next();
    }

    struct timespec end {};
    clock_gettime(CLOCK_MONOTONIC, &end);

    const std::int64_t elapsed = frcnc::rt::timespec_diff_ns(begin, end);
    const std::int64_t expected = kPeriodNs * kCycles;

    // Must take at least the nominal time — a timer that returns early is broken.
    CHECK(elapsed >= expected);
    // And should not take wildly longer, even on a loaded CI box.
    CHECK(elapsed < expected * 5);
}

}  // namespace

int main() {
    std::printf("test_cyclic_task\n");

    test_timespec_add_basic();
    test_timespec_add_carries();
    test_timespec_add_multi_second();
    test_timespec_add_negative();
    test_timespec_diff();
    test_stats_reset();
    test_cycle_timer_runs();
    test_cycle_timer_period_is_honoured();

    std::printf("  %d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
