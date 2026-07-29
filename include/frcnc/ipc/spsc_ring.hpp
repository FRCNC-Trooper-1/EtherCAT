// Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.
//
// Lock-free single-producer / single-consumer ring buffer.
//
// This is the boundary between the planner and the cyclic task. The cyclic task
// must never block, never allocate, and never wait on the planner — a mutex here
// would put a 1 ms deadline at the mercy of whatever the planner is doing, and
// an uncontended mutex can still page-fault.
//
// With exactly one producer and one consumer, no compare-and-swap is needed:
// each side owns one index outright and only reads the other's. Both push() and
// pop() are wait-free and take a handful of nanoseconds.
//
// See docs/05-motion-architecture.md §3.

#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <new>
#include <type_traits>

namespace frcnc::ipc {

/// Cache line size. Head and tail live on separate lines so the producer
/// writing its index does not invalidate the consumer's cache line — false
/// sharing here costs more than the queue operation itself.
inline constexpr std::size_t kCacheLine = 64;

/// Wait-free SPSC ring buffer.
///
/// @tparam T         payload; must be trivially copyable, since it crosses a
///                   thread boundary by raw copy with no construction
/// @tparam Capacity  slot count; must be a power of two so index wrapping is a
///                   mask rather than a modulo
template <typename T, std::size_t Capacity>
class SpscRing {
    static_assert(std::is_trivially_copyable_v<T>,
                  "SPSC payload must be trivially copyable: it is handed across a thread "
                  "boundary without construction or destruction");
    static_assert(Capacity >= 2, "capacity must be at least 2");
    static_assert((Capacity & (Capacity - 1)) == 0, "capacity must be a power of two");

public:
    using value_type = T;

    static constexpr std::size_t capacity() noexcept { return Capacity; }

    /// Producer side. Returns false if the queue is full.
    [[nodiscard]] bool push(const T& value) noexcept {
        // We own tail_, so a relaxed load is enough.
        const std::size_t tail = tail_.load(std::memory_order_relaxed);
        const std::size_t next = tail + 1;

        // Use the cached head first; only re-read the consumer's atomic when the
        // cache says we are full. On a queue that is not near-full this avoids
        // touching the consumer's cache line at all.
        if (next - cached_head_ > Capacity) {
            cached_head_ = head_.load(std::memory_order_acquire);
            if (next - cached_head_ > Capacity) {
                return false;
            }
        }

        buffer_[tail & kMask] = value;

        // Release: the payload write above must be visible before the consumer
        // can observe the new tail.
        tail_.store(next, std::memory_order_release);
        return true;
    }

    /// Consumer side. Returns false if the queue is empty.
    [[nodiscard]] bool pop(T& out) noexcept {
        const std::size_t head = head_.load(std::memory_order_relaxed);

        if (head == cached_tail_) {
            cached_tail_ = tail_.load(std::memory_order_acquire);
            if (head == cached_tail_) {
                return false;
            }
        }

        out = buffer_[head & kMask];
        head_.store(head + 1, std::memory_order_release);
        return true;
    }

    /// Consumer side: look at the front without removing it.
    [[nodiscard]] bool peek(T& out) const noexcept {
        const std::size_t head = head_.load(std::memory_order_relaxed);
        const std::size_t tail = tail_.load(std::memory_order_acquire);
        if (head == tail) {
            return false;
        }
        out = buffer_[head & kMask];
        return true;
    }

    /// Approximate occupancy. Exact when called from the consumer.
    [[nodiscard]] std::size_t size() const noexcept {
        const std::size_t tail = tail_.load(std::memory_order_acquire);
        const std::size_t head = head_.load(std::memory_order_acquire);
        return tail - head;  // unsigned wraparound is well defined and correct
    }

    [[nodiscard]] bool empty() const noexcept { return size() == 0; }
    [[nodiscard]] bool full() const noexcept { return size() >= Capacity; }

    /// Free slots. Producer-side estimate.
    [[nodiscard]] std::size_t space() const noexcept { return Capacity - size(); }

    /// Drop everything. NOT safe while either side is running — for teardown
    /// and test setup only.
    void reset() noexcept {
        head_.store(0, std::memory_order_relaxed);
        tail_.store(0, std::memory_order_relaxed);
        cached_head_ = 0;
        cached_tail_ = 0;
    }

private:
    static constexpr std::size_t kMask = Capacity - 1;

    // The payload comes FIRST, ahead of the indices, purely to keep GCC's
    // -Wstringop-overflow analysis honest: with an atomic as the first member it
    // decides the whole object is eight bytes long and warns about every slot
    // write past the first. Every index is masked to [0, Capacity), so the
    // warning is spurious — but silencing it by ordering is better than a
    // pragma that would also hide a real overflow here later.
    //
    // Each index still gets its own cache line, which is the part that matters:
    // the producer writing tail_ must not invalidate the line the consumer is
    // reading head_ from.
    alignas(kCacheLine) T buffer_[Capacity];

    // Consumer's index, and the producer's cached copy of it.
    alignas(kCacheLine) std::atomic<std::size_t> head_{0};
    alignas(kCacheLine) std::size_t cached_tail_{0};

    // Producer's index, and the consumer's cached copy of it.
    alignas(kCacheLine) std::atomic<std::size_t> tail_{0};
    alignas(kCacheLine) std::size_t cached_head_{0};
};

/// Single-writer / multi-reader latest-value slot, via a sequence lock.
///
/// Status flowing back from the cyclic task is not a queue — nobody wants a
/// backlog of stale positions, they want the current one. A ring would either
/// fill up or force the writer to drop, and both are wrong here.
///
/// The writer never blocks and never retries, which is what the real-time side
/// requires. Readers retry if they catch a write in progress.
///
/// NOTE: the payload copy races with the writer by construction; the sequence
/// check is what makes a torn read detectable rather than impossible. This is
/// the standard seqlock trade and is why the payload must be trivially
/// copyable and read through memcpy rather than field by field.
template <typename T>
class LatestValue {
    static_assert(std::is_trivially_copyable_v<T>,
                  "seqlock payload must be trivially copyable");

public:
    /// Writer side. Wait-free.
    void store(const T& value) noexcept {
        const std::uint64_t seq = seq_.load(std::memory_order_relaxed);

        // Odd sequence marks a write in progress.
        seq_.store(seq + 1, std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_release);

        std::memcpy(&storage_, &value, sizeof(T));

        std::atomic_thread_fence(std::memory_order_release);
        seq_.store(seq + 2, std::memory_order_relaxed);
    }

    /// Reader side. Retries while a write is in progress.
    ///
    /// @param max_attempts bound so a reader cannot spin forever against a
    ///                     writer that is updating every cycle.
    /// @return false if no consistent snapshot was obtained.
    [[nodiscard]] bool load(T& out, int max_attempts = 16) const noexcept {
        for (int attempt = 0; attempt < max_attempts; attempt++) {
            const std::uint64_t before = seq_.load(std::memory_order_relaxed);
            if ((before & 1u) != 0) {
                continue;  // write in progress
            }

            std::atomic_thread_fence(std::memory_order_acquire);
            std::memcpy(&out, &storage_, sizeof(T));
            std::atomic_thread_fence(std::memory_order_acquire);

            if (seq_.load(std::memory_order_relaxed) == before) {
                return true;  // no write overlapped the copy
            }
        }
        return false;
    }

    /// Writes completed so far.
    [[nodiscard]] std::uint64_t generation() const noexcept {
        return seq_.load(std::memory_order_relaxed) / 2;
    }

private:
    alignas(kCacheLine) std::atomic<std::uint64_t> seq_{0};
    alignas(kCacheLine) T storage_{};
};

}  // namespace frcnc::ipc
