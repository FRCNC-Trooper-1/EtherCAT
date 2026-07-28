// Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.
//
// rt_probe — validate the real-time setup using this project's own primitives.
//
// cyclictest measures the kernel. rt_probe measures *our cyclic loop*: the same
// CycleTimer, the same memory locking, the same scheduling class the controller
// will use. If rt_probe is clean, the timing foundation is sound.
//
// Needs no EtherCAT hardware and no SOEM. Run it on the control PC as soon as
// the kernel is tuned, before any bus work starts.
//
//   sudo ./build/rt_probe --cycle 1000 --cpu 2 --duration 3600

#include "frcnc/rt/cyclic_task.hpp"

#include <atomic>
#include <cinttypes>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace {

std::atomic<bool> g_stop{false};

void handle_signal(int) {
    g_stop.store(true, std::memory_order_relaxed);
}

struct Options {
    std::int64_t cycle_us = 1000;   // 1 ms
    int cpu = 2;
    int priority = 85;
    std::int64_t duration_s = 60;
    std::int64_t overrun_us = 100;
    bool no_rt = false;             // skip privileged setup, for unprivileged smoke tests
};

void usage(const char* argv0) {
    std::printf(
        "rt_probe — validate real-time cycle timing\n"
        "\n"
        "usage: %s [options]\n"
        "\n"
        "  --cycle <us>      cycle period in microseconds   (default 1000)\n"
        "  --cpu <n>         CPU to pin to, must be isolated (default 2)\n"
        "  --priority <n>    SCHED_FIFO priority             (default 85)\n"
        "  --duration <s>    run time in seconds, 0 = forever (default 60)\n"
        "  --overrun <us>    jitter above this counts as an overrun (default 100)\n"
        "  --no-rt           skip mlockall/affinity/SCHED_FIFO (unprivileged test)\n"
        "  --help\n"
        "\n"
        "Run for at least 12 hours under load before signing off Phase 1.\n"
        "See docs/02-pc-realtime-setup.md §7 for acceptance criteria.\n",
        argv0);
}

bool parse_args(int argc, char** argv, Options& opt) {
    for (int i = 1; i < argc; i++) {
        const std::string a = argv[i];
        auto next = [&](std::int64_t& out) {
            if (i + 1 >= argc) {
                return false;
            }
            out = std::strtoll(argv[++i], nullptr, 10);
            return true;
        };

        if (a == "--help" || a == "-h") {
            usage(argv[0]);
            std::exit(0);
        } else if (a == "--no-rt") {
            opt.no_rt = true;
        } else if (a == "--cycle") {
            if (!next(opt.cycle_us)) return false;
        } else if (a == "--duration") {
            if (!next(opt.duration_s)) return false;
        } else if (a == "--overrun") {
            if (!next(opt.overrun_us)) return false;
        } else if (a == "--cpu") {
            std::int64_t v = 0;
            if (!next(v)) return false;
            opt.cpu = static_cast<int>(v);
        } else if (a == "--priority") {
            std::int64_t v = 0;
            if (!next(v)) return false;
            opt.priority = static_cast<int>(v);
        } else {
            std::fprintf(stderr, "unknown option: %s\n", a.c_str());
            return false;
        }
    }
    return true;
}

/// Log-ish histogram buckets, in microseconds.
constexpr std::int64_t kBuckets[] = {1, 2, 5, 10, 20, 50, 100, 200, 500, 1000};
constexpr std::size_t kNumBuckets = sizeof(kBuckets) / sizeof(kBuckets[0]);

void print_histogram(const std::vector<std::uint64_t>& hist, std::uint64_t total) {
    std::printf("\n  jitter distribution\n");
    std::printf("  %-14s %12s  %7s\n", "bucket", "count", "share");
    std::printf("  ---------------------------------------------\n");

    std::int64_t lo = 0;
    for (std::size_t i = 0; i < kNumBuckets; i++) {
        char label[32];
        std::snprintf(label, sizeof(label), "%" PRId64 "-%" PRId64 " us", lo, kBuckets[i]);
        const double pct = total ? (100.0 * static_cast<double>(hist[i]) / static_cast<double>(total)) : 0.0;
        std::printf("  %-14s %12llu  %6.2f%%\n", label,
                    static_cast<unsigned long long>(hist[i]), pct);
        lo = kBuckets[i];
    }
    char label[32];
    std::snprintf(label, sizeof(label), ">%" PRId64 " us", kBuckets[kNumBuckets - 1]);
    const double pct = total ? (100.0 * static_cast<double>(hist[kNumBuckets]) / static_cast<double>(total)) : 0.0;
    std::printf("  %-14s %12llu  %6.2f%%\n", label,
                static_cast<unsigned long long>(hist[kNumBuckets]), pct);
}

}  // namespace

int main(int argc, char** argv) {
    Options opt;
    if (!parse_args(argc, argv, opt)) {
        usage(argv[0]);
        return 2;
    }

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    const std::int64_t cycle_ns = opt.cycle_us * 1000;
    const std::int64_t overrun_ns = opt.overrun_us * 1000;

    std::printf("rt_probe — cycle %" PRId64 " us, CPU %d, priority %d, ",
                opt.cycle_us, opt.cpu, opt.priority);
    if (opt.duration_s > 0) {
        std::printf("duration %" PRId64 " s\n", opt.duration_s);
    } else {
        std::printf("duration unlimited\n");
    }

    // Allocate everything before going real-time.
    std::vector<std::uint64_t> hist(kNumBuckets + 1, 0);

    if (!opt.no_rt) {
        frcnc::rt::RtConfig cfg;
        cfg.cpu = opt.cpu;
        cfg.priority = opt.priority;

        const std::string err = frcnc::rt::configure_current_thread(cfg);
        if (!err.empty()) {
            std::fprintf(stderr,
                         "\nWARNING: real-time setup failed: %s\n"
                         "Results below are NOT representative. Run as root, or use --no-rt.\n\n",
                         err.c_str());
        } else {
            std::printf("real-time thread configured\n");
        }
    } else {
        std::printf("running WITHOUT real-time configuration (--no-rt)\n");
    }

    const std::uint64_t target_cycles =
        opt.duration_s > 0
            ? static_cast<std::uint64_t>((opt.duration_s * 1'000'000) / opt.cycle_us)
            : 0;

    frcnc::rt::CycleTimer timer(cycle_ns, overrun_ns);
    timer.start();

    std::printf("running... (Ctrl-C to stop early)\n");

    std::uint64_t next_report = 0;
    const std::uint64_t report_interval = static_cast<std::uint64_t>(10'000'000 / opt.cycle_us);

    while (!g_stop.load(std::memory_order_relaxed)) {
        const std::int64_t jitter = timer.wait_next();

        // --- this is where the EtherCAT exchange and control step would go ---

        timer.mark_work_done();

        // Bucket the jitter. Branchless enough; no allocation, no syscall.
        const std::int64_t jitter_us = jitter / 1000;
        std::size_t b = kNumBuckets;
        for (std::size_t i = 0; i < kNumBuckets; i++) {
            if (jitter_us <= kBuckets[i]) {
                b = i;
                break;
            }
        }
        hist[b]++;

        const auto& st = timer.stats();

        if (report_interval > 0 && st.cycles >= next_report) {
            next_report = st.cycles + report_interval;
            std::printf("\r  %s   ", st.to_string().c_str());
            std::fflush(stdout);
        }

        if (target_cycles > 0 && st.cycles >= target_cycles) {
            break;
        }
    }

    const auto& st = timer.stats();

    std::printf("\n\n=== Results ===\n");
    std::printf("  cycles        : %llu\n", static_cast<unsigned long long>(st.cycles));
    std::printf("  jitter min    : %" PRId64 " ns\n",
                st.min_jitter_ns == INT64_MAX ? 0 : st.min_jitter_ns);
    std::printf("  jitter mean   : %" PRId64 " ns\n", st.mean_jitter_ns);
    std::printf("  jitter max    : %" PRId64 " ns   <-- this is the number that matters\n",
                st.max_jitter_ns);
    std::printf("  overruns      : %llu  (jitter > %" PRId64 " us)\n",
                static_cast<unsigned long long>(st.overruns), opt.overrun_us);

    print_histogram(hist, st.cycles);

    // Verdict against the docs/02 §7 budget for this cycle time.
    const std::int64_t budget_us = opt.cycle_us >= 1000  ? 100
                                   : opt.cycle_us >= 500 ? 50
                                                         : 25;
    const std::int64_t max_us = st.max_jitter_ns / 1000;

    std::printf("\n=== Verdict ===\n");
    std::printf("  budget for a %" PRId64 " us cycle: max jitter < %" PRId64 " us\n",
                opt.cycle_us, budget_us);

    if (opt.no_rt) {
        std::printf("  INCONCLUSIVE — ran without real-time configuration.\n");
        return 0;
    }
    if (max_us < budget_us) {
        std::printf("  PASS (%" PRId64 " us)\n", max_us);
        std::printf("\n  Now run for 12 hours under load before signing off Phase 1:\n");
        std::printf("    sudo stress-ng --cpu 2 --io 2 --vm 2 --vm-bytes 1G --taskset 0,1 &\n");
        std::printf("    sudo ./build/rt_probe --cycle %" PRId64 " --cpu %d --duration 43200\n",
                    opt.cycle_us, opt.cpu);
        return 0;
    }

    std::printf("  FAIL (%" PRId64 " us)\n", max_us);
    std::printf("\n  Work through, in order:\n");
    std::printf("    1. hwlatdetect --duration=30m --threshold=10us   (SMI / BIOS)\n");
    std::printf("    2. BIOS: C-states, SpeedStep, Turbo, hyper-threading, USB legacy\n");
    std::printf("    3. ./scripts/check-realtime.sh <iface>\n");
    std::printf("  See docs/02-pc-realtime-setup.md\n");
    return 1;
}
