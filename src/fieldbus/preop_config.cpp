// Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.

#include "frcnc/fieldbus/preop_config.hpp"

namespace frcnc::fieldbus {

namespace {

/// 10^n for n in 0..9. A table rather than a loop so this stays constant-time
/// and obviously correct.
constexpr std::int64_t kPow10[10] = {1,        10,        100,        1'000,       10'000,
                                     100'000,  1'000'000, 10'000'000, 100'000'000, 1'000'000'000};

}  // namespace

std::int64_t InterpolationPeriod::to_ns() const noexcept {
    if (!valid) {
        return 0;
    }
    // The exponent is a power of ten of SECONDS, so nanoseconds add nine.
    const int n = exponent + 9;
    if (n < 0 || n > 9) {
        return 0;
    }
    return static_cast<std::int64_t>(units) * kPow10[n];
}

InterpolationPeriod encode_interpolation_period(std::int64_t cycle_ns) noexcept {
    InterpolationPeriod p;
    if (cycle_ns <= 0) {
        return p;
    }

    // Whole milliseconds take the conventional (n, -3) form. Every drive on the
    // bench reports 4 ms that way, and matching what the device already says
    // keeps a readback comparison meaningful to whoever reads the log.
    if (cycle_ns % 1'000'000 == 0 && cycle_ns / 1'000'000 <= 255) {
        p.units = static_cast<std::uint8_t>(cycle_ns / 1'000'000);
        p.exponent = -3;
        p.valid = true;
        return p;
    }

    // Otherwise start at microseconds and coarsen only as far as the byte
    // forces. 250 us stays (250, -6); 500 us cannot, and becomes (50, -5).
    if (cycle_ns % 1'000 != 0) {
        return p;  // sub-microsecond: not representable, and nobody wants one
    }

    std::int64_t units = cycle_ns / 1'000;
    int exponent = -6;
    while (units > 255) {
        if (units % 10 != 0 || exponent >= -1) {
            return p;  // needs more significant digits than a byte holds
        }
        units /= 10;
        exponent++;
    }

    p.units = static_cast<std::uint8_t>(units);
    p.exponent = static_cast<std::int8_t>(exponent);
    p.valid = true;
    return p;
}

bool PreOpConfig::add(const SdoWrite& w) noexcept {
    if (write_count >= kMaxWrites || !w.valid() || w.slave < 0) {
        return false;
    }
    write[write_count++] = w;
    return true;
}

}  // namespace frcnc::fieldbus
