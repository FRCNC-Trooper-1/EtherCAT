// Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.
//
// EtherCAT bus lifecycle and cyclic exchange, over SOEM.
//
// Requires SOEM. Only build this against -DFRCNC_WITH_SOEM=ON.
//
// The bring-up order here is not arbitrary — each step depends on the previous
// one, and one of them is the step most people get wrong:
//
//   open()             ecx_init: open the raw socket
//   configure()        enumerate, set blockLRW, discover PDO offsets, map,
//                      configure DC, wait for SAFE-OP
//   start_cycling()    run the cyclic exchange and let DC settle
//   go_operational()   only once DC is locked
//
// Process data must ALREADY be flowing before OPERATIONAL is requested. A drive
// will refuse to enter OP, or drop straight back out of it, if the exchange has
// not started. See docs/03-ethercat-bringup.md §4.

#pragma once

#include "frcnc/fieldbus/dc_sync.hpp"
#include "frcnc/fieldbus/pdo_map.hpp"

#include "soem/soem.h"

#include <cstdint>

namespace frcnc::fieldbus {

enum class BusState : std::uint8_t {
    Closed,
    Opened,
    PreOp,
    SafeOp,
    Operational,
    Fault,
};

enum class BusResult : std::uint8_t {
    Ok,
    AlreadyOpen,
    NotOpen,
    NotConfigured,
    InterfaceFailed,   ///< could not open the NIC; usually not root
    NoSlaves,          ///< nothing responded
    TooManySlaves,
    MappingFailed,
    NoDistributedClocks,  ///< DC requested but no slave supports it
    SafeOpFailed,
    OperationalFailed,
    DcNotLocked,       ///< go_operational() called before DC settled
};

[[nodiscard]] const char* to_string(BusResult r) noexcept;
[[nodiscard]] const char* to_string(BusState s) noexcept;

/// List the network interfaces SOEM can bind to, newline-separated.
///
/// Exists because "InterfaceFailed" on its own sends people hunting for a
/// permissions or driver problem when the real answer is almost always that the
/// interface is named something else on this machine. Not real-time; call it
/// only from a diagnostic path.
///
/// @return characters written, excluding the terminator.
std::size_t list_interfaces(char* buffer, std::size_t capacity) noexcept;

struct BusConfig {
    /// NIC name, e.g. "enp3s0". Must be a dedicated port with no IP stack.
    char interface[32] = {};

    /// Cycle period. Must equal Sync0 and each drive's 0x60C2.
    std::int64_t cycle_ns = 1'000'000;

    /// Sync0 shift. Frames must reach every slave before Sync0 fires, but the
    /// next frame must not have been sent. Typically 20-50% of the cycle.
    std::int32_t sync0_shift_ns = 250'000;

    bool use_dc = true;

    /// Force LRD/LWR instead of a combined LRW datagram.
    ///
    /// Yaskawa Sigma-7 does not support LRW; Sigma-X does. SOEM sets this from
    /// the slave's SII when the vendor programs the flag, but not every vendor
    /// does. See docs/06-vendor-notes.md.
    bool force_block_lrw = false;

    /// Timeout for each EtherCAT state transition, microseconds.
    int state_timeout_us = 200'000;

    /// Mailbox transfers drained per cycle. Required in SOEM v2 for SDO access
    /// while the bus is in OP.
    int mailbox_per_cycle = 4;

    DcSyncConfig dc{};
};

struct SlaveInfo {
    char name[64] = {};
    std::uint32_t vendor_id = 0;
    std::uint32_t product_code = 0;
    std::uint32_t revision = 0;

    std::uint16_t mbx_proto = 0;
    bool has_coe = false;
    bool has_soe = false;
    bool has_dc = false;
    bool block_lrw = false;

    std::uint32_t out_bytes = 0;
    std::uint32_t in_bytes = 0;

    std::uint8_t* outputs = nullptr;
    const std::uint8_t* inputs = nullptr;

    /// Discovered from the device, not assumed. Only meaningful for CoE slaves.
    AxisPdoMap pdo{};
    bool pdo_discovered = false;
};

/// Result of one cyclic exchange.
struct ExchangeStatus {
    int working_counter = 0;
    int expected_wkc = 0;
    bool wkc_ok = false;

    std::int64_t dc_time = 0;
    std::int64_t dc_correction = 0;  ///< add to the next sleep interval

    std::uint64_t cycles = 0;
    std::uint64_t wkc_errors = 0;
    std::uint32_t consecutive_wkc_errors = 0;

    /// True when everything is nominal this cycle. A false here is a fault
    /// condition: on a CNC it means a coordinated stop of every axis.
    [[nodiscard]] bool healthy() const noexcept { return wkc_ok; }
};

/// Owns the bus. Large (SOEM's context embeds all slave storage) — allocate
/// statically or on the heap, never on the stack.
class Bus {
public:
    static constexpr int kMaxSlaves = 64;
    static constexpr int kIoMapBytes = 8192;

    Bus() = default;
    Bus(const Bus&) = delete;
    Bus& operator=(const Bus&) = delete;

    /// Open the NIC. Needs root for the raw socket.
    [[nodiscard]] BusResult open(const BusConfig& cfg) noexcept;

    /// Enumerate, discover PDO offsets, map process data, configure DC, and
    /// reach SAFE-OP.
    [[nodiscard]] BusResult configure() noexcept;

    /// Run one cyclic exchange. Real-time path: no allocation, no logging.
    ///
    /// Call this from the cyclic task from SAFE-OP onwards — including while
    /// waiting for DC to lock, and before requesting OPERATIONAL.
    [[nodiscard]] ExchangeStatus exchange() noexcept;

    /// Request OPERATIONAL. Requires the exchange to be running and, when DC is
    /// enabled, the drift controller to be locked.
    [[nodiscard]] BusResult go_operational() noexcept;

    /// Request SAFE-OP, stopping motion but leaving the bus up.
    [[nodiscard]] BusResult go_safe_operational() noexcept;

    void close() noexcept;

    [[nodiscard]] BusState state() const noexcept { return state_; }
    [[nodiscard]] int slave_count() const noexcept { return slave_count_; }

    /// Slaves are 1-indexed, matching EtherCAT addressing. Index 0 is the
    /// aggregate pseudo-slave and is not exposed here.
    [[nodiscard]] const SlaveInfo& slave(int i) const noexcept;

    [[nodiscard]] const DcSync& dc() const noexcept { return dc_; }
    [[nodiscard]] bool dc_available() const noexcept { return dc_available_; }

    [[nodiscard]] int expected_wkc() const noexcept { return expected_wkc_; }
    [[nodiscard]] int iomap_size() const noexcept { return iomap_size_; }

    /// Re-read every slave's actual EtherCAT state. Not real-time: this issues
    /// datagrams outside the cyclic exchange.
    [[nodiscard]] int read_lowest_state() noexcept;

    /// Read an object over SDO.
    ///
    /// NOT real-time — this is mailbox traffic, and the response time is at the
    /// mercy of the slave. Use it during bring-up, or from the non-RT side while
    /// the cyclic task drains the mailbox queue. Never from the cyclic path.
    ///
    /// @param size in: capacity of @p data; out: bytes actually read.
    /// @return false if the slave refused or did not answer.
    [[nodiscard]] bool read_sdo(int slave, std::uint16_t index, std::uint8_t subindex,
                                void* data, int& size) noexcept;

    /// Convenience wrappers for the widths CiA 402 diagnostics use.
    [[nodiscard]] bool read_sdo_u8(int slave, std::uint16_t index, std::uint8_t subindex,
                                   std::uint8_t& value) noexcept;
    [[nodiscard]] bool read_sdo_u16(int slave, std::uint16_t index, std::uint8_t subindex,
                                    std::uint16_t& value) noexcept;
    [[nodiscard]] bool read_sdo_u32(int slave, std::uint16_t index, std::uint8_t subindex,
                                    std::uint32_t& value) noexcept;
    [[nodiscard]] bool read_sdo_i8(int slave, std::uint16_t index, std::uint8_t subindex,
                                   std::int8_t& value) noexcept;

    /// Drain SOEM's error list into a caller buffer. Not real-time.
    /// @return number of characters written, excluding the terminator.
    std::size_t drain_errors(char* buffer, std::size_t capacity) noexcept;

private:
    BusResult discover_pdo_map(int slave) noexcept;

    BusConfig cfg_{};
    BusState state_ = BusState::Closed;

    ecx_contextt ctx_{};
    std::uint8_t iomap_[kIoMapBytes] = {};
    int iomap_size_ = 0;

    SlaveInfo slaves_[kMaxSlaves + 1]{};  // 1-indexed
    SlaveInfo null_slave_{};
    int slave_count_ = 0;

    DcSync dc_{};
    bool dc_available_ = false;

    int expected_wkc_ = 0;
    std::uint64_t cycles_ = 0;
    std::uint64_t wkc_errors_ = 0;
    std::uint32_t consecutive_wkc_errors_ = 0;
};

}  // namespace frcnc::fieldbus
