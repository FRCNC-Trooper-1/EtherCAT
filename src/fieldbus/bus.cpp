// Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.

#include "frcnc/fieldbus/bus.hpp"

#include <cstdio>
#include <cstring>

namespace frcnc::fieldbus {

namespace {

/// Sync manager PDO assignment objects.
constexpr std::uint16_t kRxPdoAssign = 0x1C12;
constexpr std::uint16_t kTxPdoAssign = 0x1C13;

void copy_name(char* dst, std::size_t cap, const char* src) noexcept {
    if (cap == 0) {
        return;
    }
    std::size_t i = 0;
    for (; src != nullptr && src[i] != '\0' && i + 1 < cap; i++) {
        dst[i] = src[i];
    }
    dst[i] = '\0';
}

}  // namespace

const char* to_string(BusResult r) noexcept {
    switch (r) {
        case BusResult::Ok:                  return "Ok";
        case BusResult::AlreadyOpen:         return "AlreadyOpen";
        case BusResult::NotOpen:             return "NotOpen";
        case BusResult::NotConfigured:       return "NotConfigured";
        case BusResult::InterfaceFailed:     return "InterfaceFailed";
        case BusResult::NoSlaves:            return "NoSlaves";
        case BusResult::TooManySlaves:       return "TooManySlaves";
        case BusResult::MappingFailed:       return "MappingFailed";
        case BusResult::NoDistributedClocks: return "NoDistributedClocks";
        case BusResult::SafeOpFailed:        return "SafeOpFailed";
        case BusResult::OperationalFailed:   return "OperationalFailed";
        case BusResult::DcNotLocked:         return "DcNotLocked";
    }
    return "?";
}

const char* to_string(BusState s) noexcept {
    switch (s) {
        case BusState::Closed:      return "Closed";
        case BusState::Opened:      return "Opened";
        case BusState::PreOp:       return "PreOp";
        case BusState::SafeOp:      return "SafeOp";
        case BusState::Operational: return "Operational";
        case BusState::Fault:       return "Fault";
    }
    return "?";
}

BusResult Bus::open(const BusConfig& cfg) noexcept {
    if (state_ != BusState::Closed) {
        return BusResult::AlreadyOpen;
    }

    cfg_ = cfg;

    // ecx_init does NOT zero the context; static storage is zero-initialised,
    // but this object may be heap-allocated.
    std::memset(&ctx_, 0, sizeof(ctx_));

    if (ecx_init(&ctx_, cfg_.interface) <= 0) {
        return BusResult::InterfaceFailed;
    }

    DcSyncConfig dccfg = cfg_.dc;
    dccfg.cycle_ns = cfg_.cycle_ns;
    dccfg.shift_ns = cfg_.sync0_shift_ns;
    dc_.configure(dccfg);

    cycles_ = 0;
    wkc_errors_ = 0;
    consecutive_wkc_errors_ = 0;

    state_ = BusState::Opened;
    return BusResult::Ok;
}

BusResult Bus::discover_pdo_map(int slave) noexcept {
    SlaveInfo& info = slaves_[slave];
    if (!info.has_coe) {
        return BusResult::Ok;  // nothing to discover
    }

    PdoMapBuilder builder;

    struct Direction {
        std::uint16_t assign_index;
        PdoDirection dir;
    };
    const Direction directions[2] = {{kRxPdoAssign, PdoDirection::Rx},
                                     {kTxPdoAssign, PdoDirection::Tx}};

    for (const Direction& d : directions) {
        builder.begin(d.dir);

        // How many PDOs are assigned to this sync manager?
        std::uint8_t pdo_count = 0;
        int size = sizeof(pdo_count);
        if (ecx_SDOread(&ctx_, static_cast<std::uint16_t>(slave), d.assign_index, 0x00, FALSE,
                        &size, &pdo_count, EC_TIMEOUTRXM) <= 0) {
            builder.end();
            continue;  // drive does not expose the assignment; not fatal
        }

        for (std::uint8_t p = 1; p <= pdo_count; p++) {
            std::uint16_t mapping_index = 0;
            size = sizeof(mapping_index);
            if (ecx_SDOread(&ctx_, static_cast<std::uint16_t>(slave), d.assign_index, p, FALSE,
                            &size, &mapping_index, EC_TIMEOUTRXM) <= 0) {
                continue;
            }
            if (mapping_index == 0) {
                continue;
            }

            // Entries within this mapping object.
            std::uint8_t entries = 0;
            size = sizeof(entries);
            if (ecx_SDOread(&ctx_, static_cast<std::uint16_t>(slave), mapping_index, 0x00, FALSE,
                            &size, &entries, EC_TIMEOUTRXM) <= 0) {
                continue;
            }

            for (std::uint8_t e = 1; e <= entries; e++) {
                std::uint32_t raw = 0;
                size = sizeof(raw);
                if (ecx_SDOread(&ctx_, static_cast<std::uint16_t>(slave), mapping_index, e, FALSE,
                                &size, &raw, EC_TIMEOUTRXM) <= 0) {
                    continue;
                }
                (void)builder.add_raw(raw);
            }
        }

        builder.end();
    }

    info.pdo = builder.result();
    info.pdo_discovered = true;
    return BusResult::Ok;
}

BusResult Bus::configure() noexcept {
    if (state_ != BusState::Opened) {
        return BusResult::NotOpen;
    }

    const int found = ecx_config_init(&ctx_);
    if (found <= 0) {
        return BusResult::NoSlaves;
    }
    if (found > kMaxSlaves) {
        return BusResult::TooManySlaves;
    }
    slave_count_ = found;
    state_ = BusState::PreOp;

    // Force LRD/LWR before mapping, if the vendor did not program the SII flag.
    // This must happen before ecx_config_map_group, which promotes the per-slave
    // flag to the group.
    if (cfg_.force_block_lrw) {
        for (int i = 1; i <= slave_count_; i++) {
            ctx_.slavelist[i].blockLRW = 1;
            ctx_.slavelist[0].blockLRW++;
        }
    }

    // Register CoE slaves for cyclic mailbox handling. Without this, SDO access
    // from a non-cyclic thread while in OP will time out — a genuine v2 change.
    for (int i = 1; i <= slave_count_; i++) {
        if (ctx_.slavelist[i].CoEdetails > 0) {
            (void)ecx_slavembxcyclic(&ctx_, static_cast<std::uint16_t>(i));
        }
    }

    // Read the real PDO layout off each device while still in PRE-OP, where
    // mailbox traffic is unencumbered.
    for (int i = 1; i <= slave_count_; i++) {
        slaves_[i].has_coe = (ctx_.slavelist[i].mbx_proto & ECT_MBXPROT_COE) != 0;
        slaves_[i].has_soe = (ctx_.slavelist[i].mbx_proto & ECT_MBXPROT_SOE) != 0;
        (void)discover_pdo_map(i);
    }

    iomap_size_ = ecx_config_map_group(&ctx_, iomap_, 0);
    if (iomap_size_ <= 0 || iomap_size_ > kIoMapBytes) {
        state_ = BusState::Fault;
        return BusResult::MappingFailed;
    }

    if (cfg_.use_dc) {
        dc_available_ = ecx_configdc(&ctx_) != 0;
        if (!dc_available_) {
            state_ = BusState::Fault;
            return BusResult::NoDistributedClocks;
        }
        for (int i = 1; i <= slave_count_; i++) {
            if (ctx_.slavelist[i].hasdc) {
                ecx_dcsync0(&ctx_, static_cast<std::uint16_t>(i), TRUE,
                            static_cast<std::uint32_t>(cfg_.cycle_ns), cfg_.sync0_shift_ns);
            }
        }
    }

    // Snapshot per-slave detail now that mapping has assigned process data.
    for (int i = 1; i <= slave_count_; i++) {
        SlaveInfo& info = slaves_[i];
        const ec_slavet& s = ctx_.slavelist[i];

        copy_name(info.name, sizeof(info.name), s.name);
        info.vendor_id = s.eep_man;
        info.product_code = s.eep_id;
        info.revision = s.eep_rev;
        info.mbx_proto = s.mbx_proto;
        info.has_dc = s.hasdc != 0;
        info.block_lrw = s.blockLRW != 0;
        info.out_bytes = s.Obytes;
        info.in_bytes = s.Ibytes;
        info.outputs = s.outputs;
        info.inputs = s.inputs;
    }

    expected_wkc_ = (ctx_.grouplist[0].outputsWKC * 2) + ctx_.grouplist[0].inputsWKC;

    (void)ecx_statecheck(&ctx_, 0, EC_STATE_SAFE_OP, cfg_.state_timeout_us);
    if ((ecx_readstate(&ctx_) & 0x0F) < EC_STATE_SAFE_OP) {
        state_ = BusState::Fault;
        return BusResult::SafeOpFailed;
    }

    state_ = BusState::SafeOp;
    return BusResult::Ok;
}

ExchangeStatus Bus::exchange() noexcept {
    ExchangeStatus st;

    if (state_ != BusState::SafeOp && state_ != BusState::Operational) {
        return st;
    }

    (void)ecx_send_processdata(&ctx_);
    const int wkc = ecx_receive_processdata(&ctx_, EC_TIMEOUTRET);

    // Drain queued mailbox traffic. Required in v2 so SDO access stays possible
    // while the bus is in OP.
    if (cfg_.mailbox_per_cycle > 0) {
        (void)ecx_mbxhandler(&ctx_, 0, cfg_.mailbox_per_cycle);
    }

    cycles_++;

    st.working_counter = wkc;
    st.expected_wkc = expected_wkc_;
    st.wkc_ok = (wkc >= expected_wkc_);

    if (!st.wkc_ok) {
        wkc_errors_++;
        if (consecutive_wkc_errors_ < 0xFFFFFFFFu) {
            consecutive_wkc_errors_++;
        }
    } else {
        consecutive_wkc_errors_ = 0;
    }

    if (dc_available_ && wkc > 0) {
        st.dc_time = ctx_.DCtime;
        st.dc_correction = dc_.update(ctx_.DCtime);
    }

    st.cycles = cycles_;
    st.wkc_errors = wkc_errors_;
    st.consecutive_wkc_errors = consecutive_wkc_errors_;
    return st;
}

BusResult Bus::go_operational() noexcept {
    if (state_ != BusState::SafeOp) {
        return BusResult::NotConfigured;
    }

    // The exchange must already be running: a drive refuses OP, or drops
    // straight back out, if process data is not flowing.
    if (cycles_ == 0) {
        return BusResult::OperationalFailed;
    }

    // And DC must be locked. Requesting OP while the master is still hunting
    // makes drives fault on sync error; the usual workaround for that is to
    // disable the sync error counter, which hides the real problem.
    if (dc_available_ && !dc_.locked()) {
        return BusResult::DcNotLocked;
    }

    ctx_.slavelist[0].state = EC_STATE_OPERATIONAL;
    (void)ecx_writestate(&ctx_, 0);
    (void)ecx_statecheck(&ctx_, 0, EC_STATE_OPERATIONAL, cfg_.state_timeout_us);

    if ((ecx_readstate(&ctx_) & 0x0F) != EC_STATE_OPERATIONAL) {
        return BusResult::OperationalFailed;
    }

    state_ = BusState::Operational;
    return BusResult::Ok;
}

BusResult Bus::go_safe_operational() noexcept {
    if (state_ != BusState::Operational && state_ != BusState::SafeOp) {
        return BusResult::NotConfigured;
    }

    ctx_.slavelist[0].state = EC_STATE_SAFE_OP;
    (void)ecx_writestate(&ctx_, 0);
    (void)ecx_statecheck(&ctx_, 0, EC_STATE_SAFE_OP, cfg_.state_timeout_us);

    state_ = BusState::SafeOp;
    return BusResult::Ok;
}

int Bus::read_lowest_state() noexcept {
    if (state_ == BusState::Closed) {
        return 0;
    }
    return ecx_readstate(&ctx_);
}

bool Bus::read_sdo(int slave, std::uint16_t index, std::uint8_t subindex, void* data,
                   int& size) noexcept {
    if (state_ == BusState::Closed || slave < 1 || slave > slave_count_ || data == nullptr) {
        return false;
    }
    return ecx_SDOread(&ctx_, static_cast<std::uint16_t>(slave), index, subindex, FALSE, &size,
                       data, EC_TIMEOUTRXM) > 0;
}

bool Bus::read_sdo_u8(int slave, std::uint16_t index, std::uint8_t subindex,
                      std::uint8_t& value) noexcept {
    int size = static_cast<int>(sizeof(value));
    return read_sdo(slave, index, subindex, &value, size) && size == sizeof(value);
}

bool Bus::read_sdo_u16(int slave, std::uint16_t index, std::uint8_t subindex,
                       std::uint16_t& value) noexcept {
    int size = static_cast<int>(sizeof(value));
    return read_sdo(slave, index, subindex, &value, size) && size == sizeof(value);
}

bool Bus::read_sdo_u32(int slave, std::uint16_t index, std::uint8_t subindex,
                       std::uint32_t& value) noexcept {
    int size = static_cast<int>(sizeof(value));
    return read_sdo(slave, index, subindex, &value, size) && size == sizeof(value);
}

bool Bus::read_sdo_i8(int slave, std::uint16_t index, std::uint8_t subindex,
                      std::int8_t& value) noexcept {
    int size = static_cast<int>(sizeof(value));
    return read_sdo(slave, index, subindex, &value, size) && size == sizeof(value);
}

void Bus::close() noexcept {
    if (state_ == BusState::Closed) {
        return;
    }

    // Walk the bus down rather than yanking the socket: leaving drives in OP
    // with the master gone is how an axis ends up free-running.
    ctx_.slavelist[0].state = EC_STATE_INIT;
    (void)ecx_writestate(&ctx_, 0);
    (void)ecx_statecheck(&ctx_, 0, EC_STATE_INIT, cfg_.state_timeout_us);

    ecx_close(&ctx_);

    state_ = BusState::Closed;
    slave_count_ = 0;
    dc_available_ = false;
    expected_wkc_ = 0;
    iomap_size_ = 0;
}

const SlaveInfo& Bus::slave(int i) const noexcept {
    if (i < 1 || i > slave_count_) {
        return null_slave_;
    }
    return slaves_[i];
}

std::size_t Bus::drain_errors(char* buffer, std::size_t capacity) noexcept {
    if (buffer == nullptr || capacity == 0) {
        return 0;
    }
    buffer[0] = '\0';

    std::size_t used = 0;
    while (ecx_iserror(&ctx_) && used + 1 < capacity) {
        const char* msg = ecx_elist2string(&ctx_);
        if (msg == nullptr) {
            break;
        }
        const std::size_t len = std::strlen(msg);
        const std::size_t room = capacity - used - 1;
        const std::size_t take = (len < room) ? len : room;
        std::memcpy(buffer + used, msg, take);
        used += take;
        buffer[used] = '\0';
        if (take < len) {
            break;
        }
    }
    return used;
}

}  // namespace frcnc::fieldbus
