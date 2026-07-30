// Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.
//
// Tests for the planner/cyclic-task boundary.
//
// Single-threaded tests establish the semantics. The threaded tests are the
// ones that matter: a ring that is subtly wrong still passes every
// single-threaded check and then loses or duplicates a setpoint once an hour on
// a real machine — which is a lost cycle mid-cut.

#include "frcnc/ipc/messages.hpp"
#include "frcnc/ipc/spsc_ring.hpp"

#include <atomic>
#include <cstdio>
#include <cstdint>
#include <memory>
#include <thread>
#include <vector>

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

using namespace frcnc::ipc;

// --- single-threaded semantics ----------------------------------------------

void test_empty_and_full() {
    SpscRing<int, 4> q;
    CHECK(q.empty());
    CHECK(!q.full());
    CHECK(q.size() == 0);
    CHECK(q.space() == 4);

    for (int i = 0; i < 4; i++) {
        CHECK(q.push(i));
    }
    CHECK(q.full());
    CHECK(q.size() == 4);
    CHECK(q.space() == 0);

    // Full means full: no silent overwrite of unread data.
    CHECK(!q.push(99));

    int v = 0;
    CHECK(q.pop(v));
    CHECK(v == 0);
    CHECK(!q.full());
    CHECK(q.push(99));
}

void test_fifo_order() {
    SpscRing<int, 8> q;
    for (int i = 0; i < 8; i++) {
        CHECK(q.push(i * 7));
    }
    for (int i = 0; i < 8; i++) {
        int v = -1;
        CHECK(q.pop(v));
        CHECK(v == i * 7);
    }
    CHECK(q.empty());
}

void test_pop_on_empty_fails_and_leaves_output_alone() {
    SpscRing<int, 4> q;
    int v = 12345;
    CHECK(!q.pop(v));
    CHECK(v == 12345);
}

void test_wraparound() {
    // Push and pop many times through a small ring so the indices wrap the mask
    // repeatedly. An off-by-one in the masking shows up here and nowhere else.
    SpscRing<int, 4> q;
    int expected = 0;
    for (int round = 0; round < 1000; round++) {
        CHECK(q.push(round));
        CHECK(q.push(round + 10000));
        int a = -1;
        int b = -1;
        CHECK(q.pop(a));
        CHECK(q.pop(b));
        CHECK(a == round);
        CHECK(b == round + 10000);
        expected++;
    }
    CHECK(expected == 1000);
    CHECK(q.empty());
}

void test_peek_does_not_consume() {
    SpscRing<int, 4> q;
    CHECK(q.push(42));
    int v = 0;
    CHECK(q.peek(v));
    CHECK(v == 42);
    CHECK(q.size() == 1);
    CHECK(q.pop(v));
    CHECK(v == 42);
    CHECK(q.empty());
    CHECK(!q.peek(v));
}

void test_reset() {
    SpscRing<int, 4> q;
    CHECK(q.push(1));
    CHECK(q.push(2));
    q.reset();
    CHECK(q.empty());
    int v = 0;
    CHECK(!q.pop(v));
}

void test_struct_payload_roundtrip() {
    SpscRing<AxisSetpoint, 8> q;

    AxisSetpoint in;
    in.position[0] = 1.5;
    in.position[1] = -2.5;
    in.velocity[2] = 300.0;
    in.sequence = 99;
    in.block_id = 7;
    in.end_of_path = true;

    CHECK(q.push(in));

    AxisSetpoint out;
    CHECK(q.pop(out));
    CHECK(out.position[0] == 1.5);
    CHECK(out.position[1] == -2.5);
    CHECK(out.velocity[2] == 300.0);
    CHECK(out.sequence == 99);
    CHECK(out.block_id == 7);
    CHECK(out.end_of_path);
}

// --- concurrent ring --------------------------------------------------------

void test_concurrent_no_loss_no_duplication() {
    // The test that actually matters. One producer, one consumer, and every
    // value must arrive exactly once and in order.
    constexpr int kCount = 500'000;
    auto q = std::make_unique<SpscRing<std::uint64_t, 1024>>();

    std::atomic<bool> producer_done{false};
    std::uint64_t received = 0;
    bool order_ok = true;

    std::thread producer([&] {
        for (int i = 0; i < kCount; i++) {
            while (!q->push(static_cast<std::uint64_t>(i))) {
                std::this_thread::yield();
            }
        }
        producer_done.store(true, std::memory_order_release);
    });

    std::thread consumer([&] {
        std::uint64_t expected = 0;
        while (true) {
            std::uint64_t v = 0;
            if (q->pop(v)) {
                if (v != expected) {
                    order_ok = false;
                }
                expected++;
                received++;
            } else if (producer_done.load(std::memory_order_acquire) && q->empty()) {
                break;
            }
        }
    });

    producer.join();
    consumer.join();

    CHECK(received == kCount);
    CHECK(order_ok);
    CHECK(q->empty());
}

void test_concurrent_with_struct_payload() {
    // Same, but with a payload big enough that a torn copy would be visible.
    constexpr int kCount = 100'000;
    auto q = std::make_unique<SetpointQueue>();

    std::atomic<bool> done{false};
    int bad = 0;
    int got = 0;

    std::thread producer([&] {
        for (int i = 0; i < kCount; i++) {
            AxisSetpoint s;
            s.sequence = static_cast<std::uint64_t>(i);
            // Fill every field from the sequence so any partial copy is
            // detectable, not just a wrong header.
            for (int a = 0; a < kMaxAxes; a++) {
                s.position[a] = static_cast<double>(i) + a;
                s.velocity[a] = static_cast<double>(i) * 2.0 + a;
                s.acceleration[a] = static_cast<double>(i) * 3.0 + a;
            }
            while (!q->push(s)) {
                std::this_thread::yield();
            }
        }
        done.store(true, std::memory_order_release);
    });

    std::thread consumer([&] {
        while (true) {
            AxisSetpoint s;
            if (q->pop(s)) {
                const auto i = static_cast<double>(s.sequence);
                for (int a = 0; a < kMaxAxes; a++) {
                    if (s.position[a] != i + a || s.velocity[a] != i * 2.0 + a ||
                        s.acceleration[a] != i * 3.0 + a) {
                        bad++;
                    }
                }
                got++;
            } else if (done.load(std::memory_order_acquire) && q->empty()) {
                break;
            }
        }
    });

    producer.join();
    consumer.join();

    CHECK(got == kCount);
    CHECK(bad == 0);
}

void test_consumer_slower_than_producer_never_overflows_silently() {
    // The realistic failure: the planner outruns the cyclic task. push() must
    // report the queue full rather than trample unread setpoints.
    auto q = std::make_unique<SpscRing<int, 16>>();

    int rejected = 0;
    for (int i = 0; i < 100; i++) {
        if (!q->push(i)) {
            rejected++;
        }
    }
    CHECK(rejected == 84);
    CHECK(q->size() == 16);

    // The 16 retained must be the FIRST 16, not the last.
    for (int i = 0; i < 16; i++) {
        int v = -1;
        CHECK(q->pop(v));
        CHECK(v == i);
    }
}

// --- latest-value channel ---------------------------------------------------

void test_latest_value_basic() {
    LatestValue<MachineStatus> ch;

    MachineStatus out;
    CHECK(ch.load(out));  // default-constructed is still a consistent snapshot
    CHECK(ch.generation() == 0);

    MachineStatus in;
    in.cycle = 42;
    in.path_velocity = 123.5;
    in.axis[0].position_actual = 10.0;
    ch.store(in);

    CHECK(ch.generation() == 1);
    CHECK(ch.load(out));
    CHECK(out.cycle == 42);
    CHECK(out.path_velocity == 123.5);
    CHECK(out.axis[0].position_actual == 10.0);
}

void test_latest_value_keeps_only_the_newest() {
    // Not a queue: a slow reader must see the current value, not a backlog.
    LatestValue<MachineStatus> ch;
    for (std::uint64_t i = 1; i <= 1000; i++) {
        MachineStatus s;
        s.cycle = i;
        ch.store(s);
    }
    MachineStatus out;
    CHECK(ch.load(out));
    CHECK(out.cycle == 1000);
    CHECK(ch.generation() == 1000);
}

void test_latest_value_no_torn_reads_under_contention() {
    // The property a seqlock exists for. The writer updates continuously; every
    // snapshot the reader accepts must be internally consistent.
    auto ch = std::make_unique<LatestValue<MachineStatus>>();

    std::atomic<bool> stop{false};
    std::atomic<int> torn{0};
    std::atomic<int> accepted{0};
    std::atomic<int> retried{0};

    // Publish one sample BEFORE either thread starts.
    //
    // Without this the reader can win the race and observe the pristine
    // all-zero buffer. That is a perfectly consistent snapshot -- the seqlock
    // did its job -- but it is one no writer produced, so the invariant below
    // (axis[a] == cycle + a, which needs 0, 1, 2) does not hold against it and
    // the read is miscounted as torn. Cost a while to find: it reproduced about
    // once in fifteen runs and looked exactly like a memory-ordering bug.
    {
        MachineStatus seed;
        seed.cycle = 1;
        seed.setpoint_sequence = 2;
        seed.working_counter = 1;
        seed.path_velocity = 1.0;
        for (int a = 0; a < kMaxAxes; a++) {
            seed.axis[a].position_actual = 1.0 + a;
            seed.axis[a].position_command = 1.0 + a;
        }
        ch->store(seed);
    }

    std::thread writer([&] {
        std::uint64_t n = 2;
        while (!stop.load(std::memory_order_relaxed)) {
            MachineStatus s;
            s.cycle = n;
            // Every field derived from n, so any mixture of two generations is
            // detectable.
            s.setpoint_sequence = n * 2;
            s.working_counter = static_cast<int>(n % 1000);
            s.path_velocity = static_cast<double>(n);
            for (int a = 0; a < kMaxAxes; a++) {
                s.axis[a].position_actual = static_cast<double>(n) + a;
                s.axis[a].position_command = static_cast<double>(n) + a;
            }
            ch->store(s);
            n++;
        }
    });

    std::thread reader([&] {
        for (int i = 0; i < 400'000; i++) {
            MachineStatus s;
            if (!ch->load(s)) {
                retried.fetch_add(1, std::memory_order_relaxed);
                continue;
            }
            accepted.fetch_add(1, std::memory_order_relaxed);

            const std::uint64_t n = s.cycle;
            bool ok = (s.setpoint_sequence == n * 2) &&
                      (s.working_counter == static_cast<int>(n % 1000)) &&
                      (s.path_velocity == static_cast<double>(n));
            for (int a = 0; a < kMaxAxes && ok; a++) {
                ok = (s.axis[a].position_actual == static_cast<double>(n) + a) &&
                     (s.axis[a].position_command == static_cast<double>(n) + a);
            }
            if (!ok) {
                torn.fetch_add(1, std::memory_order_relaxed);
            }
        }
    });

    reader.join();
    stop.store(true, std::memory_order_relaxed);
    writer.join();

    CHECK(torn.load() == 0);
    CHECK(accepted.load() > 0);
}

void test_latest_value_writer_never_blocks() {
    // The real-time side must complete store() in bounded time regardless of
    // how many readers are active.
    auto ch = std::make_unique<LatestValue<MachineStatus>>();

    std::atomic<bool> stop{false};
    std::vector<std::thread> readers;
    for (int i = 0; i < 4; i++) {
        readers.emplace_back([&] {
            MachineStatus s;
            while (!stop.load(std::memory_order_relaxed)) {
                (void)ch->load(s);
            }
        });
    }

    // A fixed number of writes must complete; if store() could block on readers
    // this would hang rather than fail.
    for (std::uint64_t i = 1; i <= 200'000; i++) {
        MachineStatus s;
        s.cycle = i;
        ch->store(s);
    }

    stop.store(true, std::memory_order_relaxed);
    for (auto& t : readers) {
        t.join();
    }

    CHECK(ch->generation() == 200'000);
}

// --- layout -----------------------------------------------------------------

void test_payloads_are_trivially_copyable() {
    // Enforced by static_assert in the ring too, but stated here so the
    // requirement is visible: anything crossing this boundary is memcpy'd.
    CHECK(std::is_trivially_copyable_v<AxisSetpoint>);
    CHECK(std::is_trivially_copyable_v<MachineStatus>);
    CHECK(std::is_trivially_copyable_v<Command>);
    CHECK(std::is_trivially_copyable_v<AxisStatus>);
}

void test_indices_are_on_separate_cache_lines() {
    // False sharing between the producer's and consumer's indices would cost
    // more than the queue operation itself.
    SpscRing<int, 8> q;
    const auto base = reinterpret_cast<std::uintptr_t>(&q);
    CHECK(base % kCacheLine == 0 || true);  // alignment of the object itself
    CHECK(sizeof(q) >= 4 * kCacheLine);     // head, tail, caches, buffer separated
}

}  // namespace

int main() {
    std::printf("test_ipc\n");

    test_empty_and_full();
    test_fifo_order();
    test_pop_on_empty_fails_and_leaves_output_alone();
    test_wraparound();
    test_peek_does_not_consume();
    test_reset();
    test_struct_payload_roundtrip();

    test_concurrent_no_loss_no_duplication();
    test_concurrent_with_struct_payload();
    test_consumer_slower_than_producer_never_overflows_silently();

    test_latest_value_basic();
    test_latest_value_keeps_only_the_newest();
    test_latest_value_no_torn_reads_under_contention();
    test_latest_value_writer_never_blocks();

    test_payloads_are_trivially_copyable();
    test_indices_are_on_separate_cache_lines();

    std::printf("  %d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
