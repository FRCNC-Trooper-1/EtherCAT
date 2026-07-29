// Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.
//
// axis_jog — first motion.
//
// Brings the bus up, enables the axes, and moves ONE of them a measured
// distance under a jerk-limited profile, then reports what the machine did.
// This is the tool that turns a bus that enumerates into a machine that moves,
// and it is the first time a mistake in scaling, direction or PDO offsets shows
// up as something physical.
//
//   sudo ./build/axis_jog enp3s0 --counts-per-mm 10000 --distance 1 --feed 60
//
// RUN IT WITH THE MOTOR OFF THE MACHINE FIRST. A wrong counts-per-mm is a
// factor-of-anything error in how far the axis goes, and the first place it
// shows up is at the end of the travel.
//
// Defaults are deliberately timid: 1 mm at 60 mm/min. Argue upwards only once
// the axis has moved the distance you asked for, in the direction you expected.
//
// Reference: docs/00-roadmap.md phase 5, docs/04-drive-cia402.md

#include "frcnc/app/cyclic_task.hpp"
#include "frcnc/motion/scurve.hpp"

#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <thread>

using namespace frcnc;

namespace {

struct Options {
    const char* interface = nullptr;
    int axes = 1;
    int axis = 0;
    int slave[app::kMaxAxes] = {1, 2, 3};
    double counts_per_mm = 10000.0;
    double distance_mm = 1.0;
    double feed_mm_min = 60.0;
    double accel = 200.0;      ///< mm/s^2
    double jerk = 2000.0;      ///< mm/s^3
    double travel_mm = 50.0;   ///< +/- soft limit around the start
    double ferr_limit = 1.0;   ///< mm
    std::int64_t cycle_us = 1000;
    int cpu = 2;
    bool block_lrw = false;
    bool no_dc = false;
    bool assume_yes = false;
};

void usage() {
    std::printf(
        "usage: axis_jog <interface> [options]\n"
        "\n"
        "  --axes N            axes to bring up (default 1)\n"
        "  --axis N            axis to move, 0-based (default 0)\n"
        "  --slave a,b,c       EtherCAT slave position per axis (default 1,2,3)\n"
        "  --counts-per-mm F   encoder counts per millimetre (default 10000)\n"
        "  --distance F        distance to move in mm, may be negative (default 1)\n"
        "  --feed F            feed in mm/min (default 60)\n"
        "  --accel F           acceleration in mm/s^2 (default 200)\n"
        "  --jerk F            jerk in mm/s^3, 0 for trapezoidal (default 2000)\n"
        "  --travel F          soft limit either side of the start, mm (default 50)\n"
        "  --ferr F            following error limit in mm (default 1)\n"
        "  --cycle N           cycle time in microseconds (default 1000)\n"
        "  --cpu N             isolated CPU to pin the RT thread to (default 2)\n"
        "  --block-lrw         force LRD/LWR instead of LRW (Yaskawa Sigma-7)\n"
        "  --no-dc             run without distributed clocks (diagnostics only)\n"
        "  --yes               skip the confirmation prompt\n");
}

bool parse_double(const char* s, double& out) {
    char* end = nullptr;
    const double v = std::strtod(s, &end);
    if (end == s || *end != '\0') {
        return false;
    }
    out = v;
    return true;
}

bool parse_int(const char* s, long& out) {
    char* end = nullptr;
    const long v = std::strtol(s, &end, 10);
    if (end == s || *end != '\0') {
        return false;
    }
    out = v;
    return true;
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
        const bool has_value = (i + 1 < argc);

        if (std::strcmp(a, "--block-lrw") == 0) {
            o.block_lrw = true;
        } else if (std::strcmp(a, "--no-dc") == 0) {
            o.no_dc = true;
        } else if (std::strcmp(a, "--yes") == 0) {
            o.assume_yes = true;
        } else if (!has_value) {
            std::printf("missing value for %s\n", a);
            return false;
        } else if (std::strcmp(a, "--counts-per-mm") == 0) {
            if (!parse_double(argv[++i], o.counts_per_mm) || o.counts_per_mm == 0.0) {
                return false;
            }
        } else if (std::strcmp(a, "--distance") == 0) {
            if (!parse_double(argv[++i], o.distance_mm)) {
                return false;
            }
        } else if (std::strcmp(a, "--feed") == 0) {
            if (!parse_double(argv[++i], o.feed_mm_min) || o.feed_mm_min <= 0.0) {
                return false;
            }
        } else if (std::strcmp(a, "--accel") == 0) {
            if (!parse_double(argv[++i], o.accel) || o.accel <= 0.0) {
                return false;
            }
        } else if (std::strcmp(a, "--jerk") == 0) {
            if (!parse_double(argv[++i], o.jerk) || o.jerk < 0.0) {
                return false;
            }
        } else if (std::strcmp(a, "--travel") == 0) {
            if (!parse_double(argv[++i], o.travel_mm) || o.travel_mm <= 0.0) {
                return false;
            }
        } else if (std::strcmp(a, "--ferr") == 0) {
            if (!parse_double(argv[++i], o.ferr_limit) || o.ferr_limit < 0.0) {
                return false;
            }
        } else if (std::strcmp(a, "--slave") == 0) {
            if (!parse_slaves(argv[++i], o.slave)) {
                return false;
            }
        } else {
            long v = 0;
            if (!parse_int(argv[i + 1], v)) {
                return false;
            }
            i++;
            if (std::strcmp(a, "--axes") == 0) {
                o.axes = static_cast<int>(v);
            } else if (std::strcmp(a, "--axis") == 0) {
                o.axis = static_cast<int>(v);
            } else if (std::strcmp(a, "--cycle") == 0) {
                o.cycle_us = v;
            } else if (std::strcmp(a, "--cpu") == 0) {
                o.cpu = static_cast<int>(v);
            } else {
                std::printf("unknown option %s\n", a);
                return false;
            }
        }
    }

    if (o.axes < 1 || o.axes > app::kMaxAxes) {
        std::printf("--axes must be 1..%d\n", app::kMaxAxes);
        return false;
    }
    if (o.axis < 0 || o.axis >= o.axes) {
        std::printf("--axis must be 0..%d\n", o.axes - 1);
        return false;
    }
    if (o.cycle_us < 50) {
        std::printf("--cycle below 50 us is not credible on any PC\n");
        return false;
    }
    return true;
}

bool confirm(const Options& o) {
    std::printf(
        "\nAbout to move axis %d (slave %d) by %+.4f mm at %.1f mm/min.\n"
        "Scaling is %.1f counts/mm -- if that is wrong, so is the distance.\n"
        "Type 'yes' to proceed: ",
        o.axis, o.slave[o.axis], o.distance_mm, o.feed_mm_min, o.counts_per_mm);
    std::fflush(stdout);

    char line[16] = {};
    if (std::fgets(line, sizeof(line), stdin) == nullptr) {
        return false;
    }
    return std::strncmp(line, "yes", 3) == 0;
}

/// Wait for a predicate on the published status, with a timeout.
template <typename Pred>
bool wait_for(app::CyclicTask& task, Pred p, int timeout_ms, ipc::MachineStatus& out) {
    for (int elapsed = 0; elapsed < timeout_ms; elapsed += 10) {
        if (task.state() == app::TaskState::Failed) {
            return false;
        }
        if (task.status().load(out) && p(out)) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
}

void print_status(const ipc::MachineStatus& s, int axes) {
    std::printf("  cycle %-10" PRIu64 " wkc %d/%d  dc %+" PRId64 " ns%s  jitter max %" PRId64
                " ns\n",
                s.cycle, s.working_counter, s.expected_wkc, s.dc_error_ns,
                s.dc_locked ? " (locked)" : "", s.max_cycle_jitter_ns);
    for (int i = 0; i < axes; i++) {
        std::printf("  axis %d  pos %10.5f  cmd %10.5f  ferr %+9.6f  sw 0x%04X %s%s\n", i,
                    s.axis[i].position_actual, s.axis[i].position_command,
                    s.axis[i].following_error, s.axis[i].statusword,
                    s.axis[i].enabled ? "ENABLED" : "-", s.axis[i].faulted ? " FAULT" : "");
    }
}

}  // namespace

int main(int argc, char** argv) {
    Options o;
    if (!parse_args(argc, argv, o)) {
        usage();
        return 2;
    }

    const double dt = static_cast<double>(o.cycle_us) * 1e-6;
    const double feed = o.feed_mm_min / 60.0;

    // --- configuration ------------------------------------------------------

    app::CyclicTaskConfig cfg;
    std::snprintf(cfg.bus.interface, sizeof(cfg.bus.interface), "%s", o.interface);
    cfg.bus.cycle_ns = o.cycle_us * 1000;
    cfg.bus.sync0_shift_ns = static_cast<std::int32_t>(cfg.bus.cycle_ns / 4);
    cfg.bus.use_dc = !o.no_dc;
    cfg.bus.force_block_lrw = o.block_lrw;

    cfg.rt.cpu = o.cpu;
    cfg.machine.axis_count = o.axes;
    // Stop at least as hard as we accelerate, or a stop overshoots the move.
    cfg.machine.stop_deceleration = o.accel;

    for (int i = 0; i < o.axes; i++) {
        cfg.axis_slave[i] = o.slave[i];
        cfg.machine.axis[i].counts_per_unit = o.counts_per_mm;
        cfg.machine.axis[i].following_error_limit = o.ferr_limit;
        cfg.machine.axis[i].mode = drive::Mode::CyclicSyncPosition;
        // Soft limits are applied around the start position once it is known;
        // until then leave them disabled rather than guessing an envelope.
        cfg.machine.axis[i].soft_limit_min = 0.0;
        cfg.machine.axis[i].soft_limit_max = 0.0;
    }

    if (!o.assume_yes && !confirm(o)) {
        std::printf("aborted\n");
        return 1;
    }

    // Large: SOEM's context embeds every slave's storage. Heap, not stack.
    auto task = std::make_unique<app::CyclicTask>();

    std::printf("\nopening %s at %" PRId64 " us%s...\n", o.interface, o.cycle_us,
                o.no_dc ? " (no DC)" : "");

    const fieldbus::BusResult r = task->start(cfg);
    if (r != fieldbus::BusResult::Ok) {
        std::printf("start failed: %s\n  %s\n", fieldbus::to_string(r), task->error());
        return 1;
    }

    std::printf("%d slaves, waiting for OPERATIONAL...\n", task->bus().slave_count());

    ipc::MachineStatus s{};
    if (!wait_for(*task, [](const ipc::MachineStatus& x) { return x.bus_operational; }, 15000,
                  s)) {
        std::printf("did not reach OPERATIONAL: %s\n", task->error());
        task->stop();
        return 1;
    }
    std::printf("OPERATIONAL.\n");
    print_status(s, o.axes);

    // --- enable -------------------------------------------------------------

    ipc::Command c;
    c.kind = ipc::CommandKind::Enable;
    if (!task->commands().push(c)) {
        std::printf("command queue full\n");
        task->stop();
        return 1;
    }

    std::printf("\nenabling...\n");
    if (!wait_for(
            *task,
            [&](const ipc::MachineStatus& x) {
                for (int i = 0; i < o.axes; i++) {
                    if (!x.axis[i].enabled) {
                        return false;
                    }
                }
                return true;
            },
            5000, s)) {
        std::printf("axes did not enable%s\n", s.fault_active ? " (fault active)" : "");
        print_status(s, o.axes);
        task->stop();
        return 1;
    }
    std::printf("enabled.\n");
    print_status(s, o.axes);

    // --- plan ---------------------------------------------------------------
    //
    // Start from the COMMANDED position, not the measured one. They differ by
    // the following error, and starting from the measured position would put a
    // step of exactly that size into the first setpoint.

    double start[app::kMaxAxes] = {};
    for (int i = 0; i < o.axes; i++) {
        start[i] = s.axis[i].position_command;
    }

    const double direction = (o.distance_mm < 0.0) ? -1.0 : 1.0;
    const double magnitude = (o.distance_mm < 0.0) ? -o.distance_mm : o.distance_mm;

    motion::Limits limits;
    limits.max_velocity = feed;
    limits.max_acceleration = o.accel;
    limits.max_jerk = o.jerk;

    motion::SCurveProfile profile;
    const motion::PlanResult pr = profile.plan(magnitude, 0.0, 0.0, limits);
    if (pr != motion::PlanResult::Ok && pr != motion::PlanResult::ZeroLength) {
        std::printf("could not plan the move (result %d)\n", static_cast<int>(pr));
        task->stop();
        return 1;
    }

    std::printf("\nmoving %+.4f mm in %.3f s (peak %.2f mm/s, %d phases)\n", o.distance_mm,
                profile.duration(), profile.peak_velocity(), profile.phase_count());

    // --- feed the queue -----------------------------------------------------
    //
    // This is the non-real-time producer. It only has to stay ahead of the
    // cyclic task; if it falls behind, the machine decelerates to a stop on the
    // path rather than guessing, and starve_events counts it.

    const double duration = profile.duration();
    double t = 0.0;
    std::uint64_t sequence = 0;
    bool queued_all = false;

    while (!queued_all) {
        if (task->state() == app::TaskState::Failed) {
            std::printf("task failed: %s\n", task->error());
            break;
        }
        if (task->status().load(s) && s.fault_active) {
            break;
        }

        // Keep the queue near full, but leave headroom so push() never spins.
        while (task->setpoints().space() > 4 && !queued_all) {
            t += dt;
            if (t >= duration) {
                t = duration;
                queued_all = true;
            }
            const motion::Sample sample = profile.at(t);

            ipc::AxisSetpoint sp;
            for (int i = 0; i < o.axes; i++) {
                sp.position[i] = start[i];
            }
            sp.position[o.axis] = start[o.axis] + direction * sample.position;
            sp.velocity[o.axis] = direction * sample.velocity;
            sp.acceleration[o.axis] = direction * sample.acceleration;
            sp.sequence = ++sequence;
            sp.block_id = 1;
            sp.end_of_path = queued_all;

            if (!task->setpoints().push(sp)) {
                queued_all = false;
                t -= dt;
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    // --- wait for the motion to finish -------------------------------------

    (void)wait_for(
        *task,
        [&](const ipc::MachineStatus& x) {
            return x.fault_active || (!x.motion_active && x.setpoint_sequence >= sequence);
        },
        static_cast<int>(duration * 1000.0) + 5000, s);

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    (void)task->status().load(s);

    std::printf("\n--- result ---\n");
    print_status(s, o.axes);

    const double moved = s.axis[o.axis].position_actual - start[o.axis];
    std::printf("\n  commanded %+.5f mm\n", o.distance_mm);
    std::printf("  measured  %+.5f mm   (error %+.5f mm)\n", moved, moved - o.distance_mm);
    std::printf("  setpoints %" PRIu64 " of %" PRIu64 ", starved %" PRIu64 " times\n",
                s.setpoint_sequence, sequence, s.starve_events);

    rt::CycleStats stats;
    if (task->cycle_stats(stats)) {
        std::printf("  %s\n", stats.to_string().c_str());
    }

    if (s.fault_active) {
        std::printf("\n  FAULT ACTIVE -- the machine stopped itself.\n");
    }
    if (s.starve_events > 0) {
        std::printf(
            "\n  The planner could not keep the queue full, so the axis stopped on\n"
            "  the path. That is the designed response, but it means this box\n"
            "  cannot feed setpoints at %" PRId64 " us. Raise --cycle or isolate a core.\n",
            o.cycle_us);
    }

    std::printf("\nstopping...\n");
    task->stop();
    std::printf("done.\n");

    return s.fault_active ? 1 : 0;
}
