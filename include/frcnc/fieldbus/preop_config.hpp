// Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.
//
// Configuration written to a slave during the PRE-OP -> SAFE-OP transition.
//
// Two things a CNC master must settle before the process image is mapped:
//
//   1. WHICH PDOs are assigned to the sync managers (0x1C12 / 0x1C13). The
//      drive powers up with a vendor default, and on Yaskawa Sigma-X that
//      default (0x1601 / 0x1A01) carries only controlword + target position and
//      statusword + position actual. The richer predefined mappings 0x1600 /
//      0x1A00 add modes of operation, modes display, following error and torque
//      actual — all of which the supervisor here wants.
//
//   2. The interpolation time period (0x60C2). CSP is a *timed* interface: the
//      drive interpolates between setpoints over exactly this period, so a
//      value that disagrees with the master's cycle makes every commanded move
//      come out at the wrong speed, silently and by exactly the ratio of the
//      two. The bench drives report 125 us out of the box.
//
// Both MUST happen in PRE-OP. Sync manager assignment changes the process data
// length, and the length is fixed once the ESC's sync managers are programmed
// for SAFE-OP; 0x60C2 is only writable while the drive is not cycling.
//
// Pure data and arithmetic — no SOEM dependency, so the 0x60C2 encoding is
// testable without hardware. Applied by fieldbus::Bus.

#pragma once

#include <cstdint>

namespace frcnc::fieldbus {

/// One SDO write applied during the PRE-OP -> SAFE-OP transition.
struct SdoWrite {
    /// 1-based slave, or 0 for "every CoE slave on the bus".
    int slave = 0;
    std::uint16_t index = 0;
    std::uint8_t subindex = 0;
    std::uint8_t bytes = 0;  ///< 1, 2 or 4
    std::uint32_t value = 0;

    [[nodiscard]] bool valid() const noexcept {
        return bytes == 1 || bytes == 2 || bytes == 4;
    }

    /// Does this write apply to @p n?
    [[nodiscard]] bool applies_to(int n) const noexcept { return slave == 0 || slave == n; }
};

/// 0x60C2 encoded as CiA 402 wants it: a byte count and a decimal exponent.
///
/// 0x60C2:01 (interpolation time units) is UNSIGNED8, so the count cannot
/// exceed 255 — which is why the exponent exists and why a 500 us cycle cannot
/// be expressed in microseconds.
struct InterpolationPeriod {
    std::uint8_t units = 0;
    std::int8_t exponent = 0;
    bool valid = false;

    /// Back to nanoseconds, for verifying a readback.
    [[nodiscard]] std::int64_t to_ns() const noexcept;

    [[nodiscard]] bool operator==(const InterpolationPeriod& o) const noexcept {
        return valid == o.valid && units == o.units && exponent == o.exponent;
    }
};

/// Encode a cycle period for 0x60C2.
///
/// A whole number of milliseconds gets the (n, -3) form and everything else the
/// finest form that fits in a byte, starting at microseconds and coarsening one
/// decade at a time. That is what drives themselves report: 4 ms reads back as
/// (4, -3) and 125 us as (125, -6). Returns an invalid result for a period that
/// cannot be represented exactly — sub-microsecond, or needing more than three
/// significant digits.
[[nodiscard]] InterpolationPeriod encode_interpolation_period(std::int64_t cycle_ns) noexcept;

/// What the master writes to each slave before the process image is mapped.
struct PreOpConfig {
    static constexpr int kMaxWrites = 32;

    /// Mapping object to assign to the output sync manager (0x1C12), e.g.
    /// 0x1600. Zero leaves the drive's own assignment alone.
    std::uint16_t rx_pdo_assign = 0;

    /// Mapping object to assign to the input sync manager (0x1C13), e.g.
    /// 0x1A00. Zero leaves the drive's own assignment alone.
    std::uint16_t tx_pdo_assign = 0;

    /// Write 0x60C2 to match the bus cycle, and verify it took.
    ///
    /// Off by default: it is a real change to how the drive interprets every
    /// setpoint, and a master that quietly rewrites drive parameters is not one
    /// anybody should trust. Turn it on deliberately.
    bool set_interpolation_period = false;

    SdoWrite write[kMaxWrites]{};
    int write_count = 0;

    /// @return false when the table is full or the write is malformed.
    bool add(const SdoWrite& w) noexcept;

    /// Nothing to apply, so the hook need not be registered at all.
    [[nodiscard]] bool empty() const noexcept {
        return rx_pdo_assign == 0 && tx_pdo_assign == 0 && !set_interpolation_period &&
               write_count == 0;
    }
};

}  // namespace frcnc::fieldbus
