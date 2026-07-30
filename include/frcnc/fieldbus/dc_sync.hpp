// Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.
//
// Distributed Clocks drift compensation.
//
// The master's clock and the reference slave's clock run at slightly different
// rates. Left alone they diverge, the master's cycle drifts out of phase with
// the Sync0 pulse, and eventually frames arrive on the wrong side of it — at
// which point drives see a stale or missing setpoint and fault.
//
// The fix is a PI controller that nudges the master's sleep interval so its
// cycle stays locked to the slaves' shared clock. This is the same structure
// SOEM's own sample uses, with the gains exposed and the state observable.
//
// Pure integer arithmetic, no SOEM dependency, so the loop is testable without
// hardware.
//
// See docs/03-ethercat-bringup.md §6.

#pragma once

#include <cstdint>

namespace frcnc::fieldbus {

struct DcSyncConfig {
    /// Master cycle period. Must equal the Sync0 period and 0x60C2.
    std::int64_t cycle_ns = 1'000'000;

    /// Where in the cycle the master aims to sit relative to the Sync0 edge.
    ///
    /// Frames must have reached every slave before Sync0 fires, but the next
    /// frame must not have been sent yet. Too small a shift and the last slave
    /// on the segment reads the previous cycle's data — the classic "one axis
    /// lags exactly one cycle" fault. Typically 20-50% of the cycle.
    std::int64_t shift_ns = 0;

    /// Proportional gain, applied to the phase error.
    double p_gain = 0.10;

    /// Integral gain. Removes the constant offset from clock rate mismatch.
    double i_gain = 0.02;

    /// Bound on the correction applied to any single cycle, so a transient
    /// cannot produce a large sleep adjustment.
    std::int64_t max_correction_ns = 100'000;

    /// Phase error considered "in lock". 0 derives it from the cycle.
    ///
    /// This CANNOT sensibly be a constant across cycle times. Clock rate error
    /// between master and reference slave accumulates in TIME, so four times the
    /// cycle means four times the phase drift between corrections and roughly
    /// four times the residual ripple. Measured on a real segment: a few hundred
    /// nanoseconds of error at 1 ms became 1-4 us at 4 ms, with the frame loss
    /// twelve times LOWER -- so a fixed 1 us tolerance that is nearly achievable
    /// at 1 ms is simply unreachable at 4 ms, and the failure looks like a bus
    /// problem rather than a badly posed threshold.
    ///
    /// Derived default is cycle/1000 with a 1 us floor: 1 us at 250 us and 1 ms,
    /// 4 us at 4 ms. Set explicitly to override.
    std::int64_t lock_tolerance_ns = 0;

    /// Consecutive in-tolerance cycles required before locked() reports true.
    std::uint32_t lock_cycles = 100;
};

/// PI controller locking the master cycle to the DC reference clock.
class DcSync {
public:
    void configure(const DcSyncConfig& cfg) noexcept;

    /// Feed the DC system time captured this cycle.
    ///
    /// @return nanoseconds to add to the next sleep interval. Positive length-
    ///         ens the cycle, negative shortens it.
    [[nodiscard]] std::int64_t update(std::int64_t dc_time_ns) noexcept;

    /// Phase error from the last update. Zero means perfectly in phase.
    [[nodiscard]] std::int64_t error_ns() const noexcept { return error_ns_; }

    /// Largest absolute phase error observed since reset.
    [[nodiscard]] std::int64_t peak_error_ns() const noexcept { return peak_error_ns_; }

    /// Correction applied on the last update.
    [[nodiscard]] std::int64_t correction_ns() const noexcept { return correction_ns_; }

    [[nodiscard]] std::uint64_t cycles() const noexcept { return cycles_; }

    /// True once the phase error has stayed within the configured tolerance
    /// for the configured number of consecutive updates.
    ///
    /// Do not enter OPERATIONAL before this is true: a drive that receives its
    /// first setpoints while the master is still hunting will fault on sync
    /// error, and the usual workaround for that — disabling the sync error
    /// counter — hides a real problem rather than fixing it.
    [[nodiscard]] bool locked() const noexcept { return in_tolerance_run_ >= cfg_.lock_cycles; }

    /// Consecutive in-tolerance cycles so far.
    [[nodiscard]] std::uint32_t lock_run() const noexcept { return in_tolerance_run_; }

    /// Longest run of in-tolerance cycles ever achieved. When lock never
    /// happens this says whether the controller was close or nowhere near --
    /// a peak of 90 against a requirement of 100 is a different problem from a
    /// peak of 3.
    [[nodiscard]] std::uint32_t peak_lock_run() const noexcept { return peak_lock_run_; }

    void reset() noexcept;

private:
    DcSyncConfig cfg_{};

    double integral_ = 0.0;
    std::int64_t error_ns_ = 0;
    std::int64_t peak_error_ns_ = 0;
    std::int64_t correction_ns_ = 0;
    std::uint64_t cycles_ = 0;
    std::uint32_t in_tolerance_run_ = 0;
    std::uint32_t peak_lock_run_ = 0;
};

/// Phase error of `dc_time` relative to the cycle, folded into
/// (-cycle/2, +cycle/2].
///
/// Folding matters: without it a phase that has slipped just past the cycle
/// boundary reads as nearly a whole cycle of error, and the controller drives
/// the wrong way around the circle to correct it.
[[nodiscard]] std::int64_t dc_phase_error(std::int64_t dc_time_ns, std::int64_t cycle_ns,
                                          std::int64_t shift_ns) noexcept;

}  // namespace frcnc::fieldbus
