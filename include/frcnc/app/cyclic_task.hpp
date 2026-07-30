// Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.
//
// The real-time thread. Everything below this file exists to be called from it.
//
// One thread, one loop, one job: wake on an absolute deadline, exchange process
// data, run the machine, write the result back. Nothing in the loop body
// allocates, blocks, logs, or takes a lock — see docs/05-motion-architecture.md §2.
//
// Requires SOEM. Only built against -DFRCNC_WITH_SOEM=ON.
//
// The split with MachineController is deliberate. Everything that decides what
// the machine DOES lives there, in pure logic that is tested against a simulated
// drive. What lives here is the part that cannot be tested without a NIC and a
// servo: thread setup, the bring-up sequence, and moving bytes between the
// process image and typed structures. Keeping the two apart is what makes the
// coordinated stop testable at all.
//
// Bring-up order matters and is not obvious:
//
//   open -> configure -> START CYCLING -> wait for DC lock -> OPERATIONAL
//
// Process data must already be flowing before OPERATIONAL is requested, and DC
// must be locked before that. Doing it in the other order gives you drives that
// refuse OP, or accept it and drop straight back out on a sync error.

#pragma once

#include "frcnc/app/machine_controller.hpp"
#include "frcnc/fieldbus/bus.hpp"
#include "frcnc/rt/cyclic_task.hpp"

#include <atomic>
#include <cstdint>
#include <thread>

namespace frcnc::app {

enum class TaskState : std::uint8_t {
    Stopped,
    Starting,     ///< bus is up, cycling in SAFE-OP, waiting for DC to lock
    Running,      ///< OPERATIONAL, machine controller in charge
    ShuttingDown, ///< walking the axes down before dropping the bus
    Failed,       ///< see error()
};

[[nodiscard]] const char* to_string(TaskState s) noexcept;

struct CyclicTaskConfig {
    fieldbus::BusConfig bus{};
    rt::RtConfig rt{};
    MachineConfig machine{};

    /// EtherCAT slave position for each axis, 1-indexed as the bus addresses
    /// them. Order is the machine's axis order, which is not necessarily the
    /// order the drives are cabled in.
    int axis_slave[kMaxAxes] = {1, 2, 3};

    /// Jitter above this counts as an overrun in the cycle statistics.
    std::int64_t overrun_threshold_ns = 100'000;

    /// Cycles allowed for the DC drift controller to lock before giving up.
    /// 5000 is five seconds at 1 ms, and twenty at 250 us.
    std::uint32_t dc_lock_timeout_cycles = 5000;

    /// Refuse to start if a drive does not map 0x60B1.
    ///
    /// Without velocity feedforward, CSP following error is proportional to
    /// feed — roughly one cycle of travel. At 250 us and 10 m/min that is
    /// 42 um of lag on every axis, and on a circle it becomes radius error.
    /// Default off so a bench drive with a minimal PDO set still runs.
    bool require_velocity_feedforward = false;

    /// Cycles to keep running after a stop is requested, so the axes can be
    /// walked down to a de-energised state before the bus drops.
    std::uint32_t shutdown_cycles = 500;
};

/// Owns the real-time thread, the bus, and the machine.
///
/// LARGE — SOEM's context embeds every slave's storage plus the I/O map. Put it
/// on the heap or in static storage, never on a stack.
///
/// All cross-thread traffic goes through the lock-free channels: setpoints and
/// commands in, status out. There is deliberately no way to reach into the
/// MachineController and mutate it from another thread.
class CyclicTask {
public:
    CyclicTask() = default;
    ~CyclicTask();

    CyclicTask(const CyclicTask&) = delete;
    CyclicTask& operator=(const CyclicTask&) = delete;

    /// Open the NIC, enumerate, map, and spawn the real-time thread.
    ///
    /// Not real-time, and not callable from the real-time thread. Returns once
    /// the bus is configured and the thread is running; the thread itself then
    /// works through DC lock and OPERATIONAL, which state() reports.
    [[nodiscard]] fieldbus::BusResult start(const CyclicTaskConfig& cfg);

    /// Ask the axes down, then drop the bus and join the thread. Idempotent.
    void stop();

    [[nodiscard]] TaskState state() const noexcept {
        return state_.load(std::memory_order_acquire);
    }

    /// Description of why the task failed, or an empty string. Stable after the
    /// thread has exited.
    [[nodiscard]] const char* error() const noexcept { return error_; }

    /// Why real-time scheduling could not be established, or an empty string if
    /// it was. Kept apart from error() deliberately: a loop that silently ran
    /// without SCHED_FIFO explains every timing symptom downstream, and losing
    /// that to a later, more visible failure is how people chase the wrong bug.
    [[nodiscard]] const char* rt_status() const noexcept { return rt_status_; }
    [[nodiscard]] bool is_realtime() const noexcept { return rt_status_[0] == '\0'; }

    // --- channels shared with the planner ---

    [[nodiscard]] ipc::SetpointQueue& setpoints() noexcept { return setpoints_; }
    [[nodiscard]] ipc::CommandQueue& commands() noexcept { return commands_; }
    [[nodiscard]] ipc::StatusChannel& status() noexcept { return status_; }

    /// Latest cycle statistics. Read max_jitter_ns, not the mean — a loop that
    /// is 400 us late once an hour faults a drive once an hour.
    [[nodiscard]] bool cycle_stats(rt::CycleStats& out) const noexcept {
        return stats_.load(out);
    }

    /// Bus enumeration results, for reporting after start() succeeds. Safe to
    /// read while the thread runs: the slave table is fixed at configure time.
    [[nodiscard]] const fieldbus::Bus& bus() const noexcept { return bus_; }

private:
    void run() noexcept;
    void bind_axes() noexcept;
    void read_inputs(BusInputs& in, const fieldbus::ExchangeStatus& st) const noexcept;
    void write_outputs(const MachineOutputs& out) noexcept;
    void fail(const char* message) noexcept;

    /// Where one axis lives in the process image, resolved once at start.
    struct AxisBinding {
        std::uint8_t* outputs = nullptr;
        const std::uint8_t* inputs = nullptr;
        fieldbus::AxisPdoMap pdo{};
        bool bound = false;
    };

    CyclicTaskConfig cfg_{};
    fieldbus::Bus bus_{};
    MachineController machine_{};
    AxisBinding axis_[kMaxAxes]{};

    ipc::SetpointQueue setpoints_{};
    ipc::CommandQueue commands_{};
    ipc::StatusChannel status_{};
    ipc::LatestValue<rt::CycleStats> stats_{};

    std::thread thread_{};
    std::atomic<TaskState> state_{TaskState::Stopped};
    std::atomic<bool> stop_requested_{false};

    char error_[192] = {};
    char rt_status_[192] = {};
};

}  // namespace frcnc::app
