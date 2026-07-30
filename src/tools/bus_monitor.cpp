// Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.
//
// bus_monitor — cyclic communication test. Moves nothing.
//
// bus_scan stops at SAFE-OP: it tells you what is on the bus, not whether the
// bus stays healthy once frames are flowing. This runs the real cyclic task at
// the real cycle time and watches what happens over minutes or hours:
//
//   - does DC lock, and how far does it drift once locked
//   - does the working counter stay at the expected value
//   - do the slaves stay in OPERATIONAL
//   - what does loop jitter look like WITH fieldbus traffic, which is the only
//     jitter number that means anything (rt_probe measures an empty loop)
//
// IT NEVER SENDS ENABLE. The machine controller sits in Idle for the whole run:
// every axis holds Request::Disable, target position tracks actual, and no
// torque is ever commanded. That makes it safe to run against drives with no
// motors attached, against a machine with the axes on hard stops, or overnight.
//
//   sudo ./build/bus_monitor ethX --axes 3 --cycle 250 --duration 60
//   sudo ./build/bus_monitor ethX --axes 3 --cycle 250 --duration 86400
//
// The second form is the 24-hour soak that docs/hardware-qualification calls
// for. Run it before a machine ships.
//
// WITH NO MOTORS CONNECTED the drives will sit in Fault, and that is the
// correct result -- a servo drive with no encoder cannot come up. Everything
// this tool measures is independent of that. Use bus_scan to read the alarm
// code if you want to confirm why.
//
// Reference: docs/03-ethercat-bringup.md, docs/02-pc-realtime-setup.md §7

#include "frcnc/app/cyclic_task.hpp"

#include <chrono>
#include <cinttypes>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <thread>

using namespace frcnc;

namespace {

volatile std::sig_atomic_t g_interrupted = 0;

void on_signal(int) {
    g_interrupted = 1;
}

struct Options {
    const char* interface = nullptr;
    int axes = 1;
    int slave[app::kMaxAxes] = {1, 2, 3};
    std::int64_t cycle_us = 1000;
    int cpu = 2;
    long duration_s = 60;   ///< 0 runs until interrupted
    long interval_ms = 1000;
    long dc_tolerance_ns = 0;   ///< 0 keeps the DcSync default
    long dc_lock_cycles = 0;    ///< 0 keeps the DcSync default
    long rx_timeout_us = 0;     ///< 0 derives from the cycle
    long dc_timeout_cycles = 0; ///< 0 keeps the CyclicTask default
    bool block_lrw = false;
    bool no_dc = false;
};

void usage() {
    std::printf(
        "usage: bus_monitor <interface> [options]\n"
        "\n"
        "  --axes N        axes to bring up (default 1)\n"
        "  --slave a,b,c   EtherCAT slave position per axis (default 1,2,3)\n"
        "  --cycle N       cycle time in microseconds (default 1000)\n"
        "  --cpu N         isolated CPU to pin the RT thread to (default 2)\n"
        "  --duration N    seconds to run, 0 for until Ctrl-C (default 60)\n"
        "  --interval N    milliseconds between report lines (default 1000)\n"
        "  --dc-tol N      DC phase error counted as in-lock, ns (default 1000)\n"
        "  --dc-lock N     consecutive in-tolerance cycles to declare lock (100)\n"
        "  --rx-timeout N  frame receive timeout, us (default: cycle/4)\n"
        "  --dc-timeout N  cycles to wait for DC lock before failing (5000)\n"
        "  --block-lrw     force LRD/LWR instead of LRW (Yaskawa Sigma-7)\n"
        "  --no-dc         run without distributed clocks (diagnostics only)\n"
        "\n"
        "Never enables an axis. Safe with no motors connected.\n");
}

bool parse_slaves(const char* s, int* slave) {
    int n = 0;
    const char* p = s;
    while (*p != '\0' && n < app::kMaxAxes) {
        char* end = nullptr;
        const long v = std::strtol(p, &end, 10);
        if (end == p || v < 1) {
            return false;
        }
        slave[n++] = static_cast<int>(v);
        p = end;
        if (*p == ',') {
            p++;
        } else if (*p != '\0') {
            return false;
        }
    }
    return n > 0;
}

bool parse_args(int argc, char** argv, Options& o) {
    if (argc < 2) {
        return false;
    }
    o.interface = argv[1];

    for (int i = 2; i < argc; i++) {
        const char* a = argv[i];

        if (std::strcmp(a, "--block-lrw") == 0) {
            o.block_lrw = true;
            continue;
        }
        if (std::strcmp(a, "--no-dc") == 0) {
            o.no_dc = true;
            continue;
        }
        if (i + 1 >= argc) {
            std::printf("missing value for %s\n", a);
            return false;
        }

        const char* v = argv[++i];
        if (std::strcmp(a, "--slave") == 0) {
            if (!parse_slaves(v, o.slave)) {
                return false;
            }
            continue;
        }

        char* end = nullptr;
        const long n = std::strtol(v, &end, 10);
        if (end == v || *end != '\0') {
            std::printf("bad value for %s: %s\n", a, v);
            return false;
        }

        if (std::strcmp(a, "--axes") == 0) {
            o.axes = static_cast<int>(n);
        } else if (std::strcmp(a, "--cycle") == 0) {
            o.cycle_us = n;
        } else if (std::strcmp(a, "--cpu") == 0) {
            o.cpu = static_cast<int>(n);
        } else if (std::strcmp(a, "--duration") == 0) {
            o.duration_s = n;
        } else if (std::strcmp(a, "--interval") == 0) {
            o.interval_ms = n;
        } else if (std::strcmp(a, "--dc-tol") == 0) {
            o.dc_tolerance_ns = n;
        } else if (std::strcmp(a, "--dc-lock") == 0) {
            o.dc_lock_cycles = n;
        } else if (std::strcmp(a, "--rx-timeout") == 0) {
            o.rx_timeout_us = n;
        } else if (std::strcmp(a, "--dc-timeout") == 0) {
            o.dc_timeout_cycles = n;
        } else {
            std::printf("unknown option %s\n", a);
            return false;
        }
    }

    if (o.axes < 1 || o.axes > app::kMaxAxes) {
        std::printf("--axes must be 1..%d\n", app::kMaxAxes);
        return false;
    }
    if (o.cycle_us < 50) {
        std::printf("--cycle below 50 us is not credible on any PC\n");
        return false;
    }
    if (o.interval_ms < 50) {
        o.interval_ms = 50;
    }
    return true;
}

void print_axis_line(const ipc::MachineStatus& s, int i, int slave) {
    const drive::State st = drive::decode_state(s.axis[i].statusword);
    std::printf("    axis %d  slave %-2d  sw 0x%04X  %-20s mode %-3d %s%s\n", i, slave,
                s.axis[i].statusword, drive::to_string(st), s.axis[i].mode_display,
                s.axis[i].faulted ? "FAULT " : "",
                s.axis[i].warning ? "WARNING" : "");
}

}  // namespace

int main(int argc, char** argv) {
    Options o;
    if (!parse_args(argc, argv, o)) {
        usage();
        return 2;
    }

    // Every abnormal exit must still walk the bus down. Without this, Ctrl-C
    // leaves the slaves in OPERATIONAL with no master talking to them.
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    app::CyclicTaskConfig cfg;
    std::snprintf(cfg.bus.interface, sizeof(cfg.bus.interface), "%s", o.interface);
    cfg.bus.cycle_ns = o.cycle_us * 1000;
    cfg.bus.sync0_shift_ns = static_cast<std::int32_t>(cfg.bus.cycle_ns / 4);
    cfg.bus.use_dc = !o.no_dc;
    cfg.bus.force_block_lrw = o.block_lrw;
    if (o.rx_timeout_us > 0) {
        cfg.bus.rx_timeout_us = static_cast<int>(o.rx_timeout_us);
    }
    if (o.dc_tolerance_ns > 0) {
        cfg.bus.dc.lock_tolerance_ns = o.dc_tolerance_ns;
    }
    if (o.dc_lock_cycles > 0) {
        cfg.bus.dc.lock_cycles = static_cast<std::uint32_t>(o.dc_lock_cycles);
    }
    if (o.dc_timeout_cycles > 0) {
        cfg.dc_lock_timeout_cycles = static_cast<std::uint32_t>(o.dc_timeout_cycles);
    }

    cfg.rt.cpu = o.cpu;
    cfg.machine.axis_count = o.axes;
    for (int i = 0; i < o.axes; i++) {
        cfg.axis_slave[i] = o.slave[i];
        // Scaling and limits are irrelevant here: nothing is ever commanded.
        // Leave the following-error check off so a drive parked against a stop
        // does not look like a fault of ours.
        cfg.machine.axis[i].counts_per_unit = 1.0;
        cfg.machine.axis[i].following_error_limit = 0.0;
    }

    auto task = std::make_unique<app::CyclicTask>();

    std::printf("opening %s, %d axes, %" PRId64 " us cycle, DC %s\n", o.interface, o.axes,
                o.cycle_us, o.no_dc ? "off" : "on");

    const fieldbus::BusResult r = task->start(cfg);
    if (r != fieldbus::BusResult::Ok) {
        std::printf("start failed: %s\n  %s\n", fieldbus::to_string(r), task->error());
        if (r == fieldbus::BusResult::InterfaceFailed) {
            char adapters[2048];
            if (fieldbus::list_interfaces(adapters, sizeof(adapters)) > 0) {
                std::printf("\nInterfaces this machine offers:\n%s", adapters);
            }
        }
        return 1;
    }

    std::printf("%d slaves found, rx timeout %d us.\n", task->bus().slave_count(),
                task->bus().rx_timeout_us());

    // Zero the slaves' own error counters so what we read at the end describes
    // THIS run, not everything since the drives were last powered up.
    {
        auto& mutable_bus = const_cast<fieldbus::Bus&>(task->bus());
        for (int i = 1; i <= mutable_bus.slave_count(); i++) {
            (void)mutable_bus.clear_port_errors(i);
        }
    }
    std::printf("Waiting for DC lock and OPERATIONAL...\n\n");

    // --- run ----------------------------------------------------------------

    const auto started = std::chrono::steady_clock::now();
    ipc::MachineStatus s{};
    bool reached_op = false;
    bool lost_op = false;
    std::uint64_t last_cycle = 0;
    std::uint64_t stalled_reports = 0;

    for (;;) {
        std::this_thread::sleep_for(std::chrono::milliseconds(o.interval_ms));

        const double elapsed =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();

        if (g_interrupted != 0) {
            std::printf("\ninterrupted.\n");
            break;
        }
        if (task->state() == app::TaskState::Failed) {
            std::printf("\ntask failed: %s\n", task->error());
            break;
        }
        if (o.duration_s > 0 && elapsed >= static_cast<double>(o.duration_s)) {
            break;
        }
        if (!task->status().load(s)) {
            continue;  // writer was mid-update; try again next interval
        }

        if (s.bus_operational) {
            if (!reached_op) {
                std::printf("OPERATIONAL after %.1f s.\n\n", elapsed);
                reached_op = true;
            }
        } else if (reached_op) {
            lost_op = true;
        }

        // A cycle counter that stops moving means the RT thread is wedged --
        // worth catching explicitly, because every other number simply freezes
        // and the run looks healthy.
        if (s.cycle == last_cycle) {
            stalled_reports++;
        }
        last_cycle = s.cycle;

        std::printf("t=%7.1fs  cycles %-12" PRIu64 " wkc %d/%d  errs %-6" PRIu64
                    " dc %+8" PRId64 " ns %-7s jitter %6.1f us (max %6.1f)\n",
                    elapsed, s.cycle, s.working_counter, s.expected_wkc, s.wkc_errors,
                    s.dc_error_ns, s.dc_locked ? "LOCKED" : "hunting",
                    static_cast<double>(s.cycle_jitter_ns) / 1000.0,
                    static_cast<double>(s.max_cycle_jitter_ns) / 1000.0);

        for (int i = 0; i < o.axes; i++) {
            print_axis_line(s, i, o.slave[i]);
        }
    }

    // --- summary ------------------------------------------------------------

    const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    (void)task->status().load(s);

    const std::int64_t dc_peak = task->bus().dc().peak_error_ns();
    rt::CycleStats stats;
    const bool have_stats = task->cycle_stats(stats);

    std::printf("\n--- summary ---\n");
    std::printf("  duration          %.1f s\n", elapsed);
    std::printf("  cycles            %" PRIu64 "\n", s.cycle);
    std::printf("  working counter   %d of %d expected\n", s.working_counter, s.expected_wkc);
    std::printf("  wkc errors        %" PRIu64 "\n", s.wkc_errors);
    // The number that separates a lost frame from a late one.
    std::printf("  max exchange      %.1f us\n",
                static_cast<double>(task->bus().max_exchange_ns()) / 1000.0);
    std::printf("  max exchange BAD  %.1f us   (rx timeout %d us)\n",
                static_cast<double>(task->bus().max_failed_exchange_ns()) / 1000.0,
                task->bus().rx_timeout_us());
    std::printf("  worst bad wkc     %d of %d\n", task->bus().min_failed_wkc(),
                s.expected_wkc);
    std::printf("  reached OP        %s\n", reached_op ? "yes" : "NO");
    std::printf("  stayed in OP      %s\n", lost_op ? "NO -- dropped out" : "yes");
    if (!o.no_dc) {
        std::printf("  DC locked         %s\n", s.dc_locked ? "yes" : "NO");
        std::printf("  DC peak error     %+" PRId64 " ns\n", dc_peak);
        std::printf("  DC best lock run  %u cycles in tolerance\n",
                    task->bus().dc().peak_lock_run());
    }
    // Reported separately and always: a loop that quietly ran without
    // SCHED_FIFO explains every timing number above it.
    std::printf("  real-time         %s\n",
                task->is_realtime() ? "yes" : task->rt_status());
    if (have_stats) {
        std::printf("  jitter max        %.1f us\n",
                    static_cast<double>(stats.max_jitter_ns) / 1000.0);
        std::printf("  jitter mean       %.1f us\n",
                    static_cast<double>(stats.mean_jitter_ns) / 1000.0);
        std::printf("  overruns          %" PRIu64 "\n", stats.overruns);
        std::printf("  max loop exec     %.1f us  (budget %" PRId64 " us)\n",
                    static_cast<double>(stats.max_exec_ns) / 1000.0, o.cycle_us);
    }

    std::printf("\n");
    for (int i = 0; i < o.axes; i++) {
        print_axis_line(s, i, o.slave[i]);
    }

    // --- what the SLAVES saw ------------------------------------------------
    //
    // The master only knows a frame did not come back. The slaves know whether
    // they saw it and found it corrupt. That is the difference between a bad
    // cable and a bad NIC, and nothing else in this tool can tell them apart.
    std::printf("\n--- slave-side error counters (this run) ---\n");
    bool all_clean = true;
    bool read_any = false;
    {
        auto& mutable_bus = const_cast<fieldbus::Bus&>(task->bus());
        for (int i = 1; i <= mutable_bus.slave_count(); i++) {
            fieldbus::PortErrors pe;
            if (!mutable_bus.read_port_errors(i, pe)) {
                std::printf("  slave %d: could not read error registers\n", i);
                continue;
            }
            read_any = true;
            if (pe.clean()) {
                std::printf("  slave %d: clean\n", i);
                continue;
            }
            all_clean = false;
            std::printf("  slave %d: invalid=%u/%u/%u/%u  rxerr=%u/%u/%u/%u  "
                        "fwd=%u/%u/%u/%u  lostlink=%u/%u/%u/%u  pu=%u pdi=%u\n",
                        i, pe.invalid_frame[0], pe.invalid_frame[1], pe.invalid_frame[2],
                        pe.invalid_frame[3], pe.rx_error[0], pe.rx_error[1], pe.rx_error[2],
                        pe.rx_error[3], pe.forwarded_rx_error[0], pe.forwarded_rx_error[1],
                        pe.forwarded_rx_error[2], pe.forwarded_rx_error[3], pe.lost_link[0],
                        pe.lost_link[1], pe.lost_link[2], pe.lost_link[3],
                        pe.processing_unit_error, pe.pdi_error);
        }
    }
    if (read_any && s.wkc_errors > 0) {
        if (all_clean) {
            std::printf(
                "\n  Every slave saw a clean wire, yet the master lost %" PRIu64 " frames.\n"
                "  No frame was CORRUPTED on the segment -- which, with a worst bad\n"
                "  working counter of %d, means the frame never made a round trip at\n"
                "  all rather than dying partway along it. The host either failed to\n"
                "  put it on the wire or failed to deliver it back up. Either way it\n"
                "  is the NIC or its driver, not cabling: fit an Intel i210/i211\n"
                "  before tuning anything else.\n",
                s.wkc_errors, task->bus().min_failed_wkc());
        } else {
            std::printf(
                "\n  A slave counted errors on the wire. The port with a non-zero\n"
                "  count is the one RECEIVING the bad frames, so the fault is in the\n"
                "  segment feeding it -- cable, connector, or noise. Port 0 is the\n"
                "  IN port; a count there on slave 1 means the run from the NIC.\n");
        }
    }

    // --- verdict ------------------------------------------------------------

    bool pass = reached_op && !lost_op && s.wkc_errors == 0 && stalled_reports == 0;
    if (!o.no_dc && !s.dc_locked) {
        pass = false;
    }

    std::printf("\n  %s\n", pass ? "COMMUNICATION OK" : "PROBLEMS FOUND");

    if (s.wkc_errors > 0) {
        const double bad_us = static_cast<double>(task->bus().max_failed_exchange_ns()) / 1000.0;
        const double timeout_us = static_cast<double>(task->bus().rx_timeout_us());
        if (bad_us >= timeout_us * 0.9) {
            std::printf(
                "  Frames are being LOST, not delayed: the worst bad cycle sat at\n"
                "  the receive timeout, meaning nothing came back at all. That is\n"
                "  cabling, a port, or the NIC dropping frames.\n");
        } else {
            std::printf(
                "  Frames are coming back LATE, not lost: the worst bad cycle\n"
                "  finished well inside the receive timeout. Raising --rx-timeout\n"
                "  may clear it, but late frames on an idle bus mean the NIC or\n"
                "  its interrupt path is the limit.\n");
        }
        std::printf("  See docs/03-ethercat-bringup.md.\n");
    }
    if (!o.no_dc && !s.dc_locked) {
        std::printf(
            "  DC never locked. Every drive will trip a sync error the moment\n"
            "  you enable it. Do not proceed to motion until this is clean.\n");
    }
    if (stalled_reports > 0) {
        std::printf("  The cycle counter stopped advancing %" PRIu64
                    " times -- the RT thread stalled.\n",
                    stalled_reports);
    }
    if (have_stats && stats.overruns > 0) {
        std::printf("  %" PRIu64
                    " cycles overran their deadline. This cycle time is not usable\n"
                    "  on this machine as configured.\n",
                    stats.overruns);
    }

    // Drives in Fault are expected with no motor attached, so this is a note,
    // not a failure of the communication test.
    for (int i = 0; i < o.axes; i++) {
        if (s.axis[i].faulted) {
            std::printf(
                "  Axis %d is in Fault. With no motor connected that is normal --\n"
                "  the drive cannot find an encoder. Run bus_scan to read the\n"
                "  alarm code and confirm.\n",
                i);
            break;
        }
    }

    std::printf("\nstopping...\n");
    task->stop();
    std::printf("done.\n");

    return pass ? 0 : 1;
}
