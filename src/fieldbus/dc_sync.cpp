// Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.

#include "frcnc/fieldbus/dc_sync.hpp"

namespace frcnc::fieldbus {

namespace {

constexpr std::int64_t clamp_i64(std::int64_t v, std::int64_t lo, std::int64_t hi) noexcept {
    return v < lo ? lo : (v > hi ? hi : v);
}

constexpr std::int64_t abs_i64(std::int64_t v) noexcept {
    return v < 0 ? -v : v;
}

}  // namespace

std::int64_t dc_phase_error(std::int64_t dc_time_ns, std::int64_t cycle_ns,
                            std::int64_t shift_ns) noexcept {
    if (cycle_ns <= 0) {
        return 0;
    }

    std::int64_t delta = (dc_time_ns - shift_ns) % cycle_ns;

    // C's % keeps the sign of the dividend, so a negative dc_time or shift can
    // produce a negative remainder. Normalise into [0, cycle).
    if (delta < 0) {
        delta += cycle_ns;
    }

    // Fold into (-cycle/2, +cycle/2]. A phase that has slipped just past the
    // boundary is a small error in the other direction, not a nearly-whole-cycle
    // error in this one; without the fold the controller corrects the long way
    // round and never settles.
    if (delta > cycle_ns / 2) {
        delta -= cycle_ns;
    }

    // The correction must oppose the drift.
    return -delta;
}

void DcSync::configure(const DcSyncConfig& cfg) noexcept {
    cfg_ = cfg;

    // Derive the lock tolerance from the cycle unless the caller pinned it.
    // A constant threshold is wrong here: the drift the controller has to
    // reject between corrections is proportional to the cycle, so the residual
    // ripple is too. See the note on DcSyncConfig::lock_tolerance_ns.
    if (cfg_.lock_tolerance_ns <= 0) {
        cfg_.lock_tolerance_ns = cfg_.cycle_ns / 1000;
        if (cfg_.lock_tolerance_ns < 1'000) {
            cfg_.lock_tolerance_ns = 1'000;
        }
    }

    reset();
}

void DcSync::reset() noexcept {
    integral_ = 0.0;
    error_ns_ = 0;
    peak_error_ns_ = 0;
    correction_ns_ = 0;
    cycles_ = 0;
    in_tolerance_run_ = 0;
    peak_lock_run_ = 0;
}

std::int64_t DcSync::update(std::int64_t dc_time_ns) noexcept {
    if (cfg_.cycle_ns <= 0) {
        return 0;
    }

    error_ns_ = dc_phase_error(dc_time_ns, cfg_.cycle_ns, cfg_.shift_ns);

    const std::int64_t magnitude = abs_i64(error_ns_);
    if (magnitude > peak_error_ns_) {
        peak_error_ns_ = magnitude;
    }

    integral_ += static_cast<double>(error_ns_);

    // Bound the integral so a long excursion — a cable fault, a slave dropping
    // out — cannot wind up a correction that then overshoots wildly on recovery.
    const double integral_limit =
        static_cast<double>(cfg_.max_correction_ns) / (cfg_.i_gain > 0.0 ? cfg_.i_gain : 1.0);
    if (integral_ > integral_limit) {
        integral_ = integral_limit;
    } else if (integral_ < -integral_limit) {
        integral_ = -integral_limit;
    }

    const double raw =
        static_cast<double>(error_ns_) * cfg_.p_gain + integral_ * cfg_.i_gain;

    correction_ns_ = clamp_i64(static_cast<std::int64_t>(raw), -cfg_.max_correction_ns,
                               cfg_.max_correction_ns);

    // Lock tracking belongs here, not in the query: locked() must be a pure
    // observation, and the run length has to advance once per cycle regardless
    // of whether anyone asks.
    if (magnitude <= cfg_.lock_tolerance_ns) {
        if (in_tolerance_run_ < 0xFFFFFFFFu) {
            in_tolerance_run_++;
        }
        if (in_tolerance_run_ > peak_lock_run_) {
            peak_lock_run_ = in_tolerance_run_;
        }
    } else {
        in_tolerance_run_ = 0;
    }

    cycles_++;
    return correction_ns_;
}

}  // namespace frcnc::fieldbus
