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
#include "frcnc/fieldbus/preop_config.hpp"

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
    PreOpConfigFailed, ///< a PRE-OP SDO write was refused; see preop_error()
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

    /// How long the cyclic exchange waits for the frame to come back, in
    /// microseconds. 0 derives it from the cycle time.
    ///
    /// This MUST be well under the cycle period. SOEM's own default,
    /// EC_TIMEOUTRET, is 2000 us — twice a 1 ms cycle and eight times a 250 us
    /// one. Using it means a single lost frame stalls the loop past its
    /// deadline and into the following cycles, so one dropped frame becomes a
    /// burst of overruns, the DC phase is kicked every time, and the drift
    /// controller can never accumulate a run of good cycles to lock on.
    ///
    /// A frame that has not returned in a fraction of the cycle is lost.
    /// Waiting longer does not recover it; it only damages the next cycle.
    /// Round trip on a small segment is tens of microseconds, so a quarter of
    /// the cycle is generous.
    ///
    /// NOTE: SOEM waits in ppoll with a 50 us step, so the effective resolution
    /// is 50 us regardless of what is asked for. At a 250 us cycle the derived
    /// 62 us timeout therefore costs up to ~100 us of waiting, and the whole
    /// exchange (send + wait + mailbox) can exceed the nominal figure. Measured:
    /// a 62 us timeout produced a 115 us worst exchange. Budget accordingly.
    int rx_timeout_us = 0;

    /// Mailbox transfers drained per cycle, and — as the same decision — whether
    /// CoE slaves are registered for cyclic mailbox handling at all.
    ///
    /// In SOEM v2 a slave marked cyclic has its mailbox served by
    /// ecx_mbxhandler, which only runs inside exchange(). That is what makes SDO
    /// access work from a non-real-time thread while the bus is in OP. It also
    /// means a caller that never cycles must leave this at 0: otherwise every
    /// SDO blocks waiting for a handler that is never called.
    ///
    /// Cycling (CyclicTask): leave it at 4. Scanning only (bus_scan): set 0.
    int mailbox_per_cycle = 4;

    /// SDO writes applied to every slave in PRE-OP, before the process image is
    /// mapped. See preop_config.hpp for why that moment and no other.
    PreOpConfig preop{};

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

    /// Wall time spent inside send + receive this cycle.
    ///
    /// This is what separates a LOST frame from a LATE one. A frame that never
    /// comes back costs the full receive timeout; a frame that is merely slow
    /// costs whatever it actually took. Same working-counter error either way,
    /// completely different fault.
    std::int64_t exchange_ns = 0;

    /// True when everything is nominal this cycle. A false here is a fault
    /// condition: on a CNC it means a coordinated stop of every axis.
    [[nodiscard]] bool healthy() const noexcept { return wkc_ok; }
};

/// A slave's own count of what went wrong on the wire, from the ESC error
/// registers at 0x0300..0x0313 (ETG.1000.4).
///
/// This is the diagnostic that localises frame loss. The master only knows a
/// frame did not come back; the slaves know whether they SAW it and found it
/// corrupt. If a slave reports RX errors or invalid frames, the physical layer
/// on the segment feeding that port is at fault -- cable, connector, or noise.
/// If every slave reports zero and the master is still losing frames, nothing
/// on the wire went wrong and the NIC or its driver dropped them on receive.
///
/// Counters saturate at 255 and are cleared by writing to them.
struct PortErrors {
    std::uint8_t invalid_frame[4] = {};       ///< 0x0300 + 2n, low byte
    std::uint8_t rx_error[4] = {};            ///< 0x0300 + 2n, high byte
    std::uint8_t forwarded_rx_error[4] = {};  ///< 0x0308 + n
    std::uint8_t lost_link[4] = {};           ///< 0x0310 + n
    std::uint8_t processing_unit_error = 0;   ///< 0x030C
    std::uint8_t pdi_error = 0;               ///< 0x030D

    /// True when this slave saw nothing wrong at all.
    [[nodiscard]] bool clean() const noexcept;

    /// Total across every port, for a one-line verdict.
    [[nodiscard]] unsigned total() const noexcept;

    /// Did this device FIND a fault itself, rather than pass one along?
    ///
    /// This is the distinction that matters, and conflating it with the
    /// forwarded counters points the finger at the wrong component. A port only
    /// increments invalid_frame or rx_error for damage it found on the wire
    /// arriving at THAT port -- so a non-zero count here indicts the segment
    /// feeding it.
    ///
    /// processing_unit_error counts too, and originally did not, which made the
    /// tooling report "no port detected damage" on a run where a slave had
    /// counted 18. It is a different LAYER, not a different verdict: the ports
    /// check the physical layer, the processing unit checks the frame it was
    /// handed. Use detected_physical_error() when the wire specifically is the
    /// question.
    [[nodiscard]] bool detected_error() const noexcept;

    /// Did any PORT detect damage on the wire arriving at it?
    ///
    /// Narrower than detected_error(): this is the one that indicts cabling,
    /// connectors or noise on a specific segment.
    [[nodiscard]] bool detected_physical_error() const noexcept;

    /// Did this slave only ever pass on damage someone else had already marked?
    ///
    /// forwarded_rx_error counts frames that arrived ALREADY flagged. On its
    /// own, with every detection counter clear, it says the corruption happened
    /// upstream of this device -- and if the first slave's IN port is also
    /// clean, upstream of every slave, which leaves the master.
    [[nodiscard]] bool forwarded_only() const noexcept;
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

    /// Request OPERATIONAL and WAIT for it. Blocks for up to state_timeout_us.
    ///
    /// NOT for the cyclic loop — a 200 ms blocking state check inside a 1 ms
    /// loop is a 200-cycle overrun. Use request_operational() + poll_operational()
    /// there. Kept for non-real-time callers and tests.
    [[nodiscard]] BusResult go_operational() noexcept;

    /// Ask for OPERATIONAL without waiting. Safe in the cyclic path: it writes
    /// the AL control register and returns.
    ///
    /// Requires the exchange to already be running and, when DC is enabled, the
    /// drift controller to be locked.
    [[nodiscard]] BusResult request_operational() noexcept;

    /// Has OPERATIONAL been reached? Issues datagrams, so call it every few
    /// cycles rather than every cycle. Promotes state() on success.
    [[nodiscard]] bool poll_operational() noexcept;

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

    /// True when at least one slave's mailbox is served cyclically, i.e. SDO
    /// access from another thread works while the bus is in OP.
    [[nodiscard]] bool cyclic_mailbox() const noexcept { return cyclic_mailbox_; }

    /// Resolved receive timeout actually in use, microseconds.
    [[nodiscard]] int rx_timeout_us() const noexcept { return rx_timeout_us_; }

    /// Longest exchange seen, and the longest seen on a cycle whose working
    /// counter was wrong. If the failing figure sits at the receive timeout the
    /// frames are being lost; if it sits well below, they are arriving late and
    /// the timeout is what rejected them.
    [[nodiscard]] std::int64_t max_exchange_ns() const noexcept { return max_exchange_ns_; }
    [[nodiscard]] std::int64_t max_failed_exchange_ns() const noexcept {
        return max_failed_exchange_ns_;
    }

    /// Lowest working counter seen on a cycle that failed, or -2 if none has.
    ///
    /// Distinguishes a frame that never came back from one that was cut short
    /// partway along the segment:
    ///
    ///   -1  EC_NOFRAME — nothing returned at all within the receive timeout
    ///    0  a frame returned but no slave had processed it
    ///  1..n a frame returned having been processed by only some slaves, and
    ///       the number says how far along the segment it got
    [[nodiscard]] int min_failed_wkc() const noexcept { return min_failed_wkc_; }

    /// Re-read every slave's actual EtherCAT state. Not real-time: this issues
    /// datagrams outside the cyclic exchange.
    [[nodiscard]] int read_lowest_state() noexcept;

    /// Read a slave's ESC error counters. NOT real-time — an extra datagram.
    [[nodiscard]] bool read_port_errors(int slave, PortErrors& out) noexcept;

    /// Zero a slave's ESC error counters, so a later read measures one run
    /// rather than everything since the drive was powered on.
    bool clear_port_errors(int slave) noexcept;

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

    /// Write an object over SDO. Same real-time caveat as read_sdo().
    ///
    /// Most drive parameters are only writable in PRE-OP, and anything that
    /// changes the process data length is only writable there. Prefer
    /// BusConfig::preop, which applies writes at the one moment in the bring-up
    /// where both are true; this is the escape hatch for everything else.
    [[nodiscard]] bool write_sdo(int slave, std::uint16_t index, std::uint8_t subindex,
                                 const void* data, int size) noexcept;

    [[nodiscard]] bool write_sdo_u8(int slave, std::uint16_t index, std::uint8_t subindex,
                                    std::uint8_t value) noexcept;
    [[nodiscard]] bool write_sdo_u16(int slave, std::uint16_t index, std::uint8_t subindex,
                                     std::uint16_t value) noexcept;
    [[nodiscard]] bool write_sdo_u32(int slave, std::uint16_t index, std::uint8_t subindex,
                                     std::uint32_t value) noexcept;
    [[nodiscard]] bool write_sdo_i8(int slave, std::uint16_t index, std::uint8_t subindex,
                                    std::int8_t value) noexcept;

    /// Why the PRE-OP configuration failed, or an empty string if it did not.
    ///
    /// Carried separately from the BusResult because "a write was refused" is
    /// useless on its own: which slave, which object, and whether it was
    /// refused outright or silently ignored are the whole diagnosis.
    [[nodiscard]] const char* preop_error() const noexcept { return preop_error_; }

    /// Drain SOEM's error list into a caller buffer. Not real-time.
    /// @return number of characters written, excluding the terminator.
    std::size_t drain_errors(char* buffer, std::size_t capacity) noexcept;

private:
    BusResult discover_pdo_map(int slave) noexcept;

    /// SOEM's PRE-OP -> SAFE-OP slave configuration hook.
    ///
    /// This is the only correct place for these writes, and the reason is
    /// ordering inside SOEM rather than preference: ecx_map_coe_soe waits for
    /// PRE-OP (so the mailbox is live — direct reads immediately after
    /// ecx_config_init are not), calls this hook, and only THEN reads the PDO
    /// mapping to compute the process data length. A sync manager reassignment
    /// made here is therefore reflected in the image SOEM builds; the same
    /// write a few lines later in configure() would not be.
    ///
    /// Recovers the Bus from ctx->userdata, which SOEM never touches.
    static int preop_hook(ecx_contextt* ctx, std::uint16_t slave) noexcept;

    int apply_preop(int slave) noexcept;
    bool assign_pdo(int slave, std::uint16_t assign_index, std::uint16_t mapping_index) noexcept;
    bool apply_interpolation_period(int slave) noexcept;
    void fail_preop(int slave, const char* what, std::uint16_t index,
                    std::uint8_t subindex) noexcept;

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
    bool cyclic_mailbox_ = false;

    char preop_error_[160] = {};
    bool preop_failed_ = false;

    int expected_wkc_ = 0;
    int rx_timeout_us_ = 250;
    std::int64_t max_exchange_ns_ = 0;
    std::int64_t max_failed_exchange_ns_ = 0;
    int min_failed_wkc_ = -2;
    std::uint64_t cycles_ = 0;
    std::uint64_t wkc_errors_ = 0;
    std::uint32_t consecutive_wkc_errors_ = 0;
};

}  // namespace frcnc::fieldbus
