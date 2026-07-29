// Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.

#include "frcnc/fieldbus/pdo_map.hpp"

namespace frcnc::fieldbus {

namespace {

constexpr std::uint16_t kControlword = 0x6040;
constexpr std::uint16_t kStatusword = 0x6041;
constexpr std::uint16_t kModesOfOperation = 0x6060;
constexpr std::uint16_t kModesDisplay = 0x6061;
constexpr std::uint16_t kPositionActual = 0x6064;
constexpr std::uint16_t kVelocityActual = 0x606C;
constexpr std::uint16_t kTorqueActual = 0x6077;
constexpr std::uint16_t kTargetPosition = 0x607A;
constexpr std::uint16_t kVelocityOffset = 0x60B1;
constexpr std::uint16_t kTorqueOffset = 0x60B2;
constexpr std::uint16_t kTouchProbeFunction = 0x60B8;
constexpr std::uint16_t kTouchProbeStatus = 0x60B9;
constexpr std::uint16_t kTouchProbePos1 = 0x60BA;
constexpr std::uint16_t kFollowingError = 0x60F4;
constexpr std::uint16_t kDigitalInputs = 0x60FD;

void fill(PdoEntry& e, std::uint16_t index, std::uint8_t sub, std::uint8_t bits,
          std::uint32_t offset) noexcept {
    e.index = index;
    e.subindex = sub;
    e.bit_length = bits;
    e.byte_offset = offset;
    e.valid = true;
}

}  // namespace

bool AxisPdoMap::usable_for_csp() const noexcept {
    return controlword.valid && target_position.valid && statusword.valid &&
           position_actual.valid;
}

void decode_mapping_entry(std::uint32_t raw, std::uint16_t& index, std::uint8_t& subindex,
                          std::uint8_t& bit_length) noexcept {
    index = static_cast<std::uint16_t>((raw >> 16) & 0xFFFFu);
    subindex = static_cast<std::uint8_t>((raw >> 8) & 0xFFu);
    bit_length = static_cast<std::uint8_t>(raw & 0xFFu);
}

void PdoMapBuilder::reset() noexcept {
    map_ = AxisPdoMap{};
    dir_ = PdoDirection::Rx;
    in_direction_ = false;
    bit_offset_ = 0;
}

void PdoMapBuilder::begin(PdoDirection dir) noexcept {
    dir_ = dir;
    in_direction_ = true;
    bit_offset_ = 0;
}

void PdoMapBuilder::end() noexcept {
    if (!in_direction_) {
        return;
    }
    // Round up: a slave whose mapping is not byte-aligned still occupies whole
    // bytes in the process image.
    const std::uint32_t bytes = (bit_offset_ + 7u) / 8u;
    if (dir_ == PdoDirection::Rx) {
        map_.rx_bytes = bytes;
    } else {
        map_.tx_bytes = bytes;
    }
    in_direction_ = false;
}

bool PdoMapBuilder::add_raw(std::uint32_t raw) noexcept {
    std::uint16_t index = 0;
    std::uint8_t sub = 0;
    std::uint8_t bits = 0;
    decode_mapping_entry(raw, index, sub, bits);
    return add(index, sub, bits);
}

bool PdoMapBuilder::add(std::uint16_t index, std::uint8_t subindex,
                        std::uint8_t bit_length) noexcept {
    if (!in_direction_) {
        return false;
    }

    // Index 0 is padding. It occupies space but names nothing, so it must still
    // advance the offset — this is what shifts every following entry.
    if (index != 0) {
        assign(index, subindex, bit_length, bit_offset_ / 8u);
    }

    bit_offset_ += bit_length;
    return true;
}

void PdoMapBuilder::assign(std::uint16_t index, std::uint8_t subindex, std::uint8_t bit_length,
                           std::uint32_t byte_offset) noexcept {
    if (dir_ == PdoDirection::Rx) {
        switch (index) {
            case kControlword:        fill(map_.controlword, index, subindex, bit_length, byte_offset); break;
            case kTargetPosition:     fill(map_.target_position, index, subindex, bit_length, byte_offset); break;
            case kModesOfOperation:   fill(map_.modes_of_operation, index, subindex, bit_length, byte_offset); break;
            case kVelocityOffset:     fill(map_.velocity_offset, index, subindex, bit_length, byte_offset); break;
            case kTorqueOffset:       fill(map_.torque_offset, index, subindex, bit_length, byte_offset); break;
            case kTouchProbeFunction: fill(map_.touch_probe_function, index, subindex, bit_length, byte_offset); break;
            default: break;
        }
        return;
    }

    switch (index) {
        case kStatusword:       fill(map_.statusword, index, subindex, bit_length, byte_offset); break;
        case kPositionActual:   fill(map_.position_actual, index, subindex, bit_length, byte_offset); break;
        case kModesDisplay:     fill(map_.modes_display, index, subindex, bit_length, byte_offset); break;
        case kFollowingError:   fill(map_.following_error, index, subindex, bit_length, byte_offset); break;
        case kVelocityActual:   fill(map_.velocity_actual, index, subindex, bit_length, byte_offset); break;
        case kTorqueActual:     fill(map_.torque_actual, index, subindex, bit_length, byte_offset); break;
        case kDigitalInputs:    fill(map_.digital_inputs, index, subindex, bit_length, byte_offset); break;
        case kTouchProbeStatus: fill(map_.touch_probe_status, index, subindex, bit_length, byte_offset); break;
        case kTouchProbePos1:   fill(map_.touch_probe_pos1, index, subindex, bit_length, byte_offset); break;
        default: break;
    }
}

// --- typed access -----------------------------------------------------------
//
// Byte-at-a-time rather than a reinterpret_cast: process data is little-endian
// on the wire regardless of host, and padding entries mean an object can land
// on any byte boundary. Writing through a misaligned pointer is undefined
// behaviour and, on some targets, a fault.

void write_u16(std::uint8_t* base, const PdoEntry& e, std::uint16_t value) noexcept {
    if (!e.valid || base == nullptr) {
        return;
    }
    std::uint8_t* p = base + e.byte_offset;
    p[0] = static_cast<std::uint8_t>(value & 0xFFu);
    p[1] = static_cast<std::uint8_t>((value >> 8) & 0xFFu);
}

void write_i16(std::uint8_t* base, const PdoEntry& e, std::int16_t value) noexcept {
    write_u16(base, e, static_cast<std::uint16_t>(value));
}

void write_i32(std::uint8_t* base, const PdoEntry& e, std::int32_t value) noexcept {
    if (!e.valid || base == nullptr) {
        return;
    }
    const auto u = static_cast<std::uint32_t>(value);
    std::uint8_t* p = base + e.byte_offset;
    p[0] = static_cast<std::uint8_t>(u & 0xFFu);
    p[1] = static_cast<std::uint8_t>((u >> 8) & 0xFFu);
    p[2] = static_cast<std::uint8_t>((u >> 16) & 0xFFu);
    p[3] = static_cast<std::uint8_t>((u >> 24) & 0xFFu);
}

void write_i8(std::uint8_t* base, const PdoEntry& e, std::int8_t value) noexcept {
    if (!e.valid || base == nullptr) {
        return;
    }
    base[e.byte_offset] = static_cast<std::uint8_t>(value);
}

std::uint16_t read_u16(const std::uint8_t* base, const PdoEntry& e) noexcept {
    if (!e.valid || base == nullptr) {
        return 0;
    }
    const std::uint8_t* p = base + e.byte_offset;
    return static_cast<std::uint16_t>(static_cast<std::uint16_t>(p[0]) |
                                      (static_cast<std::uint16_t>(p[1]) << 8));
}

std::uint32_t read_u32(const std::uint8_t* base, const PdoEntry& e) noexcept {
    if (!e.valid || base == nullptr) {
        return 0;
    }
    const std::uint8_t* p = base + e.byte_offset;
    return static_cast<std::uint32_t>(p[0]) | (static_cast<std::uint32_t>(p[1]) << 8) |
           (static_cast<std::uint32_t>(p[2]) << 16) | (static_cast<std::uint32_t>(p[3]) << 24);
}

std::int32_t read_i32(const std::uint8_t* base, const PdoEntry& e) noexcept {
    return static_cast<std::int32_t>(read_u32(base, e));
}

std::int16_t read_i16(const std::uint8_t* base, const PdoEntry& e) noexcept {
    return static_cast<std::int16_t>(read_u16(base, e));
}

std::int8_t read_i8(const std::uint8_t* base, const PdoEntry& e) noexcept {
    if (!e.valid || base == nullptr) {
        return 0;
    }
    return static_cast<std::int8_t>(base[e.byte_offset]);
}

}  // namespace frcnc::fieldbus
