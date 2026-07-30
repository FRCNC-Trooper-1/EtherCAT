// Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.

#include "frcnc/app/cyclic_task.hpp"

#include <cstdio>
#include <cstring>

namespace frcnc::app {

namespace fb = fieldbus;

const char* to_string(TaskState s) noexcept {
    switch (s) {
        case TaskState::Stopped:      return "Stopped";
        case TaskState::Starting:     return "Starting";
        case TaskState::Running:      return "Running";
        case TaskState::ShuttingDown: return "ShuttingDown";
        case TaskState::Failed:       return "Failed";
    }
    return "?";
}

CyclicTask::~CyclicTask() {
    stop();
}

void CyclicTask::fail(const char* message) noexcept {
    // Fixed buffer, no allocation: this can be called from the real-time thread.
    std::snprintf(error_, sizeof(error_), "%s", message);
    state_.store(TaskState::Failed, std::memory_order_release);
}

// --- start-up (not real-time) ------------------------------------------------

fb::BusResult CyclicTask::start(const CyclicTaskConfig& cfg) {
    if (state_.load(std::memory_order_acquire) != TaskState::Stopped) {
        return fb::BusResult::AlreadyOpen;
    }

    cfg_ = cfg;
    error_[0] = '\0';
    stop_requested_.store(false, std::memory_order_relaxed);

    // The machine's cycle time is the bus cycle time. Two sources of truth here
    // would put the stop ramp and the interpolation period out of step, which
    // shows up as a stop that overshoots by a factor nobody can explain.
    cfg_.machine.cycle_time = static_cast<double>(cfg_.bus.cycle_ns) * 1e-9;

    machine_.configure(cfg_.machine);
    machine_.set_channels(&setpoints_, &commands_, &status_);

    fb::BusResult r = bus_.open(cfg_.bus);
    if (r != fb::BusResult::Ok) {
        fail(fb::to_string(r));
        return r;
    }

    r = bus_.configure();
    if (r != fb::BusResult::Ok) {
        fail(fb::to_string(r));
        bus_.close();
        return r;
    }

    bind_axes();
    for (int i = 0; i < cfg_.machine.axis_count; i++) {
        if (!axis_[i].bound) {
            char msg[128];
            std::snprintf(msg, sizeof(msg),
                          "axis %d (slave %d): PDO map is not usable for CSP", i,
                          cfg_.axis_slave[i]);
            fail(msg);
            bus_.close();
            return fb::BusResult::MappingFailed;
        }
        if (cfg_.require_velocity_feedforward && !axis_[i].pdo.has_velocity_feedforward()) {
            char msg[128];
            std::snprintf(msg, sizeof(msg),
                          "axis %d (slave %d): 0x60B1 not mapped, velocity feedforward "
                          "required", i, cfg_.axis_slave[i]);
            fail(msg);
            bus_.close();
            return fb::BusResult::MappingFailed;
        }
    }

    state_.store(TaskState::Starting, std::memory_order_release);

    // Everything that allocates has now happened. The thread pre-faults and
    // locks memory as its first act, so nothing after this point touches the
    // allocator.
    thread_ = std::thread([this] { run(); });
    return fb::BusResult::Ok;
}

void CyclicTask::stop() {
    if (!thread_.joinable()) {
        if (state_.load(std::memory_order_acquire) != TaskState::Failed) {
            state_.store(TaskState::Stopped, std::memory_order_release);
        }
        bus_.close();
        return;
    }

    stop_requested_.store(true, std::memory_order_release);
    thread_.join();
    bus_.close();

    if (state_.load(std::memory_order_acquire) != TaskState::Failed) {
        state_.store(TaskState::Stopped, std::memory_order_release);
    }
}

void CyclicTask::bind_axes() noexcept {
    for (int i = 0; i < cfg_.machine.axis_count && i < kMaxAxes; i++) {
        axis_[i] = AxisBinding{};

        const int n = cfg_.axis_slave[i];
        if (n < 1 || n > bus_.slave_count()) {
            continue;
        }

        const fb::SlaveInfo& s = bus_.slave(n);
        if (!s.pdo_discovered || !s.pdo.usable_for_csp()) {
            continue;
        }

        axis_[i].outputs = s.outputs;
        axis_[i].inputs = s.inputs;
        axis_[i].pdo = s.pdo;
        axis_[i].bound = (s.outputs != nullptr && s.inputs != nullptr);
    }
}

// --- process image <-> typed structures --------------------------------------

void CyclicTask::read_inputs(BusInputs& in, const fb::ExchangeStatus& st) const noexcept {
    in.operational = (bus_.state() == fb::BusState::Operational);
    in.wkc_ok = st.wkc_ok;
    in.working_counter = st.working_counter;
    in.expected_wkc = st.expected_wkc;
    in.wkc_errors = st.wkc_errors;
    in.dc_error_ns = bus_.dc().error_ns();
    in.dc_locked = bus_.dc().locked();

    for (int i = 0; i < cfg_.machine.axis_count; i++) {
        const AxisBinding& b = axis_[i];
        AxisInputs& a = in.axis[i];

        if (!b.bound) {
            a = AxisInputs{};
            continue;
        }

        a.statusword = fb::read_u16(b.inputs, b.pdo.statusword);
        a.position_counts = fb::read_i32(b.inputs, b.pdo.position_actual);
        a.mode_display = b.pdo.modes_display.present()
                             ? fb::read_i8(b.inputs, b.pdo.modes_display)
                             : static_cast<std::int8_t>(0);

        // 0x60F4 is optional. When the drive does not map it, the master-side
        // following-error supervisor has nothing to work with, so leave it at
        // zero rather than synthesising a number: AxisController's limit check
        // then does nothing, and the drive's own 0x6065 remains the guard.
        a.following_error_counts = b.pdo.following_error.present()
                                       ? fb::read_i32(b.inputs, b.pdo.following_error)
                                       : 0;

        // Slave-level validity only. Whether a working-counter miss makes the
        // frame unusable is MachineController's call, against its tolerance.
        a.pdo_valid = in.operational;
    }
}

void CyclicTask::write_outputs(const MachineOutputs& out) noexcept {
    for (int i = 0; i < cfg_.machine.axis_count; i++) {
        AxisBinding& b = axis_[i];
        if (!b.bound) {
            continue;
        }
        const AxisOutputs& o = out.axis[i];

        fb::write_u16(b.outputs, b.pdo.controlword, o.controlword);
        fb::write_i32(b.outputs, b.pdo.target_position, o.target_counts);

        if (b.pdo.modes_of_operation.present()) {
            fb::write_i8(b.outputs, b.pdo.modes_of_operation, o.mode);
        }
        if (b.pdo.velocity_offset.present()) {
            fb::write_i32(b.outputs, b.pdo.velocity_offset, o.velocity_offset);
        }
        if (b.pdo.torque_offset.present()) {
            fb::write_i16(b.outputs, b.pdo.torque_offset, o.torque_offset);
        }
    }
}

// --- the real-time loop ------------------------------------------------------

void CyclicTask::run() noexcept {
    // Memory locking, pre-faulting, CPU pinning, SCHED_FIFO. A failure here is
    // not fatal — an unprivileged run is useful on a development box — but it
    // is recorded, because a machine that quietly runs without real-time
    // scheduling will pass every bench test and fault under load.
    const std::string rt_error = rt::configure_current_thread(cfg_.rt);
    if (!rt_error.empty()) {
        std::snprintf(error_, sizeof(error_), "not real-time: %s", rt_error.c_str());
    }

    rt::CycleTimer timer(cfg_.bus.cycle_ns, cfg_.overrun_threshold_ns);
    timer.start();

    std::uint32_t settle_cycles = 0;
    std::uint32_t shutdown_left = 0;
    bool operational = false;
    bool shutting_down = false;

    for (;;) {
        const std::int64_t jitter_ns = timer.wait_next();

        const fb::ExchangeStatus st = bus_.exchange();

        // Feed the DC drift correction into the next deadline. This is what
        // pulls the master's cycle into phase with the slaves' Sync0 pulse;
        // without it the two walk apart at the difference of their crystals.
        if (st.dc_correction != 0) {
            timer.adjust(st.dc_correction);
        }

        // --- bring-up ------------------------------------------------------
        //
        // Process data is already flowing by the time we get here, which is the
        // precondition for OPERATIONAL that most bring-ups miss.
        if (!operational && !shutting_down) {
            const bool dc_ready = !bus_.dc_available() || bus_.dc().locked();
            if (dc_ready) {
                const fb::BusResult r = bus_.go_operational();
                if (r != fb::BusResult::Ok) {
                    fail(fb::to_string(r));
                    break;
                }
                operational = true;
                state_.store(TaskState::Running, std::memory_order_release);
            } else if (cfg_.dc_lock_timeout_cycles != 0 &&
                       ++settle_cycles > cfg_.dc_lock_timeout_cycles) {
                fail("distributed clocks did not lock");
                break;
            }
        }

        // --- shutdown ------------------------------------------------------
        //
        // Requested from another thread, but acted on HERE: dropping the bus
        // from outside the loop would strand the axes energised, following a
        // setpoint that stopped being updated.
        if (!shutting_down && stop_requested_.load(std::memory_order_acquire)) {
            shutting_down = true;
            shutdown_left = cfg_.shutdown_cycles;
            state_.store(TaskState::ShuttingDown, std::memory_order_release);
            machine_.request_disable();
        }

        // --- the machine ---------------------------------------------------

        BusInputs in;
        read_inputs(in, st);

        machine_.set_timing(jitter_ns, timer.stats().max_jitter_ns);
        const MachineOutputs out = machine_.update(in);
        write_outputs(out);

        stats_.store(timer.stats());
        timer.mark_work_done();

        if (shutting_down) {
            // Leave once every axis is down, or once the grace period expires.
            // Waiting for ever on an axis that will not walk down would hang
            // the application on exit.
            if (!machine_.enabled() && machine_.state() == MachineState::Idle) {
                break;
            }
            if (shutdown_left == 0) {
                break;
            }
            shutdown_left--;
        }
    }

    // Best effort: leave the bus somewhere harmless. If the loop broke on a
    // failure this may not succeed, and that is fine — close() follows.
    (void)bus_.go_safe_operational();
}

}  // namespace frcnc::app
