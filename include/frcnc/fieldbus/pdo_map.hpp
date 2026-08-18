// Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.
//
// Locating CiA 402 objects inside a slave's process image.
//
// Default PDO mapping contents are vendor-specific and change between firmware
// revisions. Hardcoding byte offsets is the most common cause of a bus that
// enumerates perfectly and then commands garbage: the numbers look plausible,
// the working counter is correct, and the axis moves to the wrong place.
//
// So offsets are DISCOVERED. The master reads 0x1600../0x1A00.. back from the
// device over SDO, and this file turns that entry list into byte offsets.
//
// Pure logic — no SOEM dependency, so the offset arithmetic is testable without
// hardware. See docs/03-ethercat-bringup.md §9.

#pragma once

#include <cstddef>
#include <cstdint>

namespace frcnc::fieldbus {

/// One object located within a slave's process image.
struct PdoEntry {
    std::uint16_t index = 0;
    std::uint8_t subindex = 0;
    std::uint8_t bit_length = 0;
    std::uint32_t byte_offset = 0;  ///< from the start of this slave's area
    bool valid = false;

    [[nodiscard]] bool present() const noexcept { return valid; }
};

/// The CiA 402 objects a CNC axis needs, located in the process image.
///
/// Optional entries stay invalid when the drive does not map them, and the
/// caller must check rather than assume. A drive that does not map 0x60F4 is
/// usable; one that does not map 0x6041 is not.
struct AxisPdoMap {
    // RxPDO — master to drive
    PdoEntry controlword;         ///< 0x6040, required
    PdoEntry target_position;     ///< 0x607A, required for CSP
    PdoEntry modes_of_operation;  ///< 0x6060
    PdoEntry velocity_offset;     ///< 0x60B1, velocity feedforward
    PdoEntry torque_offset;       ///< 0x60B2, torque feedforward
    PdoEntry touch_probe_function;///< 0x60B8

    /// 0x6072, max torque, per mille of rated.
    ///
    /// The one RxPDO entry that is DANGEROUS TO LEAVE ALONE. A richer mapping
    /// such as Yaskawa's 0x1600 includes it, and everything in the process
    /// image starts at zero — so a master that maps it and does not write it
    /// commands a torque limit of zero on every cycle. The axis then enables,
    /// reports "internal limit active", and does not move.
    ///
    /// Its neighbours in the same mapping do not have this problem: 0x60FF
    /// target velocity and 0x6071 target torque are ignored by the drive while
    /// it is in CSP, so leaving them at zero costs nothing.
    PdoEntry max_torque;

    // TxPDO — drive to master
    PdoEntry statusword;          ///< 0x6041, required
    PdoEntry position_actual;     ///< 0x6064, required for CSP
    PdoEntry modes_display;       ///< 0x6061
    PdoEntry following_error;     ///< 0x60F4
    PdoEntry velocity_actual;     ///< 0x606C
    PdoEntry torque_actual;       ///< 0x6077
    PdoEntry digital_inputs;      ///< 0x60FD
    PdoEntry touch_probe_status;  ///< 0x60B9
    PdoEntry touch_probe_pos1;    ///< 0x60BA

    std::uint32_t rx_bytes = 0;  ///< total mapped output size
    std::uint32_t tx_bytes = 0;  ///< total mapped input size

    /// Everything CSP needs is mapped in both directions.
    [[nodiscard]] bool usable_for_csp() const noexcept;

    /// Feedforward is available. Not required, but strongly wanted — see
    /// docs/05-motion-architecture.md.
    [[nodiscard]] bool has_velocity_feedforward() const noexcept {
        return velocity_offset.valid;
    }
    [[nodiscard]] bool has_torque_feedforward() const noexcept { return torque_offset.valid; }
};

/// Direction of a PDO mapping.
enum class PdoDirection : std::uint8_t {
    Rx,  ///< master to drive, 0x1600..0x1603
    Tx,  ///< drive to master, 0x1A00..0x1A03
};

/// Decode one 32-bit PDO mapping entry.
///
/// Layout: bits 31..16 object index, 15..8 subindex, 7..0 bit length.
/// An index of 0 denotes padding, which still consumes space.
void decode_mapping_entry(std::uint32_t raw, std::uint16_t& index, std::uint8_t& subindex,
                          std::uint8_t& bit_length) noexcept;

/// Builds an AxisPdoMap from mapping entries read back off the device.
///
/// Feed entries in the order the drive reports them. Offsets accumulate
/// bit-sequentially, so padding entries shift everything after them — which is
/// exactly why guessing offsets goes wrong.
class PdoMapBuilder {
public:
    /// Start a direction. Resets the running bit offset.
    void begin(PdoDirection dir) noexcept;

    /// Add the next mapping entry, encoded as it appears in 0x1600../0x1A00...
    /// Returns false if called before begin().
    bool add_raw(std::uint32_t raw) noexcept;

    /// Add an already-decoded entry.
    bool add(std::uint16_t index, std::uint8_t subindex, std::uint8_t bit_length) noexcept;

    /// Finish the current direction, recording its total size.
    void end() noexcept;

    [[nodiscard]] const AxisPdoMap& result() const noexcept { return map_; }

    /// Bits consumed so far in the current direction. Non-byte-aligned totals
    /// indicate a mapping with sub-byte entries, which CiA 402 does not use but
    /// simple I/O slaves do.
    [[nodiscard]] std::uint32_t current_bits() const noexcept { return bit_offset_; }

    void reset() noexcept;

private:
    AxisPdoMap map_{};
    PdoDirection dir_ = PdoDirection::Rx;
    bool in_direction_ = false;
    std::uint32_t bit_offset_ = 0;

    /// Route a decoded entry to the matching AxisPdoMap field.
    void assign(std::uint16_t index, std::uint8_t subindex, std::uint8_t bit_length,
                std::uint32_t byte_offset) noexcept;
};

// --- typed process-data access ---------------------------------------------
//
// Little-endian, unaligned-safe. EtherCAT process data is little-endian on the
// wire regardless of host, and entries are not guaranteed to land on natural
// alignment once padding is involved.

void write_u16(std::uint8_t* base, const PdoEntry& e, std::uint16_t value) noexcept;
void write_i32(std::uint8_t* base, const PdoEntry& e, std::int32_t value) noexcept;
void write_i16(std::uint8_t* base, const PdoEntry& e, std::int16_t value) noexcept;
void write_i8(std::uint8_t* base, const PdoEntry& e, std::int8_t value) noexcept;

[[nodiscard]] std::uint16_t read_u16(const std::uint8_t* base, const PdoEntry& e) noexcept;
[[nodiscard]] std::int32_t read_i32(const std::uint8_t* base, const PdoEntry& e) noexcept;
[[nodiscard]] std::int16_t read_i16(const std::uint8_t* base, const PdoEntry& e) noexcept;
[[nodiscard]] std::int8_t read_i8(const std::uint8_t* base, const PdoEntry& e) noexcept;
[[nodiscard]] std::uint32_t read_u32(const std::uint8_t* base, const PdoEntry& e) noexcept;

}  // namespace frcnc::fieldbus
