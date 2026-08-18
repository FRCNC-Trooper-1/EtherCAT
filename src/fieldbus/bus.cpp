// Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.

#include "frcnc/fieldbus/bus.hpp"

#include <cstdio>
#include <ctime>
#include <cstring>

namespace frcnc::fieldbus {

namespace {

/// Sync manager PDO assignment objects.
constexpr std::uint16_t kRxPdoAssign = 0x1C12;
constexpr std::uint16_t kTxPdoAssign = 0x1C13;

/// Interpolation time period: :01 units, :02 decimal exponent.
constexpr std::uint16_t kInterpolationTimePeriod = 0x60C2;

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
        case BusResult::PreOpConfigFailed:   return "PreOpConfigFailed";
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

std::size_t list_interfaces(char* buffer, std::size_t capacity) noexcept {
    if (buffer == nullptr || capacity == 0) {
        return 0;
    }
    buffer[0] = '\0';

    ec_adaptert* head = ec_find_adapters();
    std::size_t written = 0;

    for (ec_adaptert* a = head; a != nullptr; a = a->next) {
        const int n = std::snprintf(buffer + written, capacity - written, "  %-16s %s\n",
                                    a->name, a->desc);
        if (n <= 0 || static_cast<std::size_t>(n) >= capacity - written) {
            break;  // truncate rather than overrun
        }
        written += static_cast<std::size_t>(n);
    }

    if (head != nullptr) {
        ec_free_adapters(head);
    }
    return written;
}

BusResult Bus::open(const BusConfig& cfg) noexcept {
    if (state_ != BusState::Closed) {
        return BusResult::AlreadyOpen;
    }

    cfg_ = cfg;

    // Resolve the receive timeout once. A quarter of the cycle, floored at
    // 50 us so a very short cycle does not time out before the wire can
    // physically answer, and capped so it can never exceed the cycle itself.
    rx_timeout_us_ = cfg_.rx_timeout_us;
    if (rx_timeout_us_ <= 0) {
        rx_timeout_us_ = static_cast<int>(cfg_.cycle_ns / 4000);
        if (rx_timeout_us_ < 50) {
            rx_timeout_us_ = 50;
        }
    }
    const int cycle_us = static_cast<int>(cfg_.cycle_ns / 1000);
    if (cycle_us > 0 && rx_timeout_us_ >= cycle_us) {
        rx_timeout_us_ = cycle_us - 1;
    }

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
    preop_failed_ = false;
    preop_error_[0] = '\0';

    state_ = BusState::Opened;
    return BusResult::Ok;
}

// --- PRE-OP configuration ----------------------------------------------------

void Bus::fail_preop(int slave, const char* what, std::uint16_t index,
                     std::uint8_t subindex) noexcept {
    // Keep the FIRST failure. Once the assignment is half-written the follow-on
    // errors are consequences, and the consequence is never the diagnosis.
    if (preop_failed_) {
        return;
    }
    std::snprintf(preop_error_, sizeof(preop_error_), "slave %d: %s (0x%04X:%02X)", slave, what,
                  static_cast<unsigned>(index), static_cast<unsigned>(subindex));
    preop_failed_ = true;
}

bool Bus::assign_pdo(int slave, std::uint16_t assign_index,
                     std::uint16_t mapping_index) noexcept {
    // Zero the count first. The sub-entries of 0x1C12/0x1C13 are read-only
    // while the assignment is active, so writing :01 straight away is refused
    // by a conforming drive and — worse — silently ignored by some others.
    if (!write_sdo_u8(slave, assign_index, 0x00, 0)) {
        fail_preop(slave, "cannot clear PDO assignment", assign_index, 0x00);
        return false;
    }
    if (!write_sdo_u16(slave, assign_index, 0x01, mapping_index)) {
        fail_preop(slave, "PDO assignment refused", assign_index, 0x01);
        return false;
    }
    if (!write_sdo_u8(slave, assign_index, 0x00, 1)) {
        fail_preop(slave, "cannot re-enable PDO assignment", assign_index, 0x00);
        return false;
    }

    // Read it back. A drive that accepts the write and keeps its own mapping
    // produces a bus that comes up perfectly and moves the axis using offsets
    // taken from a mapping it is not using — the exact failure this whole
    // discover-don't-assume design exists to prevent.
    std::uint16_t actual = 0;
    if (!read_sdo_u16(slave, assign_index, 0x01, actual) || actual != mapping_index) {
        fail_preop(slave, "PDO assignment did not take", assign_index, 0x01);
        return false;
    }
    return true;
}

bool Bus::apply_interpolation_period(int slave) noexcept {
    const InterpolationPeriod want = encode_interpolation_period(cfg_.cycle_ns);
    if (!want.valid) {
        fail_preop(slave, "cycle time not representable in 0x60C2",
                   kInterpolationTimePeriod, 0x00);
        return false;
    }

    // Exponent before units. Either order passes through an intermediate the
    // drive may not like; going coarse-then-fine passes through a period that
    // is too LONG, and a drive is far likelier to reject one that is too short.
    if (!write_sdo_i8(slave, kInterpolationTimePeriod, 0x02, want.exponent)) {
        fail_preop(slave, "interpolation time index refused", kInterpolationTimePeriod, 0x02);
        return false;
    }
    if (!write_sdo_u8(slave, kInterpolationTimePeriod, 0x01, want.units)) {
        fail_preop(slave, "interpolation time units refused", kInterpolationTimePeriod, 0x01);
        return false;
    }

    // Verified, not assumed. A drive that clamps this to its own supported
    // period rather than refusing the write would otherwise interpolate every
    // setpoint over the wrong interval, and the only symptom is that all axes
    // run at a constant wrong fraction of commanded speed.
    InterpolationPeriod got;
    if (!read_sdo_u8(slave, kInterpolationTimePeriod, 0x01, got.units) ||
        !read_sdo_i8(slave, kInterpolationTimePeriod, 0x02, got.exponent)) {
        fail_preop(slave, "cannot read back 0x60C2", kInterpolationTimePeriod, 0x00);
        return false;
    }
    got.valid = true;
    if (got.to_ns() != cfg_.cycle_ns) {
        fail_preop(slave, "0x60C2 does not match the cycle", kInterpolationTimePeriod, 0x00);
        return false;
    }
    return true;
}

int Bus::apply_preop(int slave) noexcept {
    if (preop_failed_ || slave < 1 || slave > kMaxSlaves) {
        return 0;
    }
    // has_coe is filled in by configure() before the mapping runs, so it is
    // valid here. Non-CoE slaves have no object dictionary to write to.
    if (!slaves_[slave].has_coe) {
        return 1;
    }

    if (cfg_.preop.rx_pdo_assign != 0 &&
        !assign_pdo(slave, kRxPdoAssign, cfg_.preop.rx_pdo_assign)) {
        return 0;
    }
    if (cfg_.preop.tx_pdo_assign != 0 &&
        !assign_pdo(slave, kTxPdoAssign, cfg_.preop.tx_pdo_assign)) {
        return 0;
    }
    if (cfg_.preop.set_interpolation_period && !apply_interpolation_period(slave)) {
        return 0;
    }

    for (int i = 0; i < cfg_.preop.write_count; i++) {
        const SdoWrite& w = cfg_.preop.write[i];
        if (!w.applies_to(slave)) {
            continue;
        }
        // Pack little-endian explicitly rather than aliasing the host integer.
        // CoE is little-endian on the wire whatever the host is, and the whole
        // point of this table is that the caller wrote a plain number.
        std::uint8_t bytes[4] = {};
        for (int b = 0; b < w.bytes && b < 4; b++) {
            bytes[b] = static_cast<std::uint8_t>((w.value >> (8 * b)) & 0xFFu);
        }
        if (!write_sdo(slave, w.index, w.subindex, bytes, w.bytes)) {
            fail_preop(slave, "SDO write refused", w.index, w.subindex);
            return 0;
        }
    }

    return 1;
}

int Bus::preop_hook(ecx_contextt* ctx, std::uint16_t slave) noexcept {
    if (ctx == nullptr || ctx->userdata == nullptr) {
        return 0;
    }
    return static_cast<Bus*>(ctx->userdata)->apply_preop(static_cast<int>(slave));
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

    for (int i = 1; i <= slave_count_; i++) {
        slaves_[i].has_coe = (ctx_.slavelist[i].mbx_proto & ECT_MBXPROT_COE) != 0;
        slaves_[i].has_soe = (ctx_.slavelist[i].mbx_proto & ECT_MBXPROT_SOE) != 0;
    }

    // Register the PRE-OP configuration hook. ecx_config_map_group calls it per
    // slave, after PRE-OP is confirmed and before it reads the PDO mapping —
    // which is the only window where a sync manager reassignment both works and
    // is seen by the mapping. See Bus::preop_hook.
    if (!cfg_.preop.empty()) {
        ctx_.userdata = this;
        for (int i = 1; i <= slave_count_; i++) {
            ctx_.slavelist[i].PO2SOconfig = &Bus::preop_hook;
        }
    }

    iomap_size_ = ecx_config_map_group(&ctx_, iomap_, 0);

    // Check this BEFORE the mapping size. SOEM ignores the hook's return value
    // and carries on, so a refused write shows up as a plausible-looking image
    // built from the mapping we failed to replace — reporting "MappingFailed",
    // or nothing at all, would send the reader hunting in the wrong place.
    if (preop_failed_) {
        state_ = BusState::Fault;
        return BusResult::PreOpConfigFailed;
    }

    if (iomap_size_ <= 0 || iomap_size_ > kIoMapBytes) {
        state_ = BusState::Fault;
        return BusResult::MappingFailed;
    }

    // Discover the PDO layout HERE, after mapping, not before.
    //
    // The obvious place is right after ecx_config_init, in PRE-OP, and it does
    // not work: ecx_config_init REQUESTS PRE-OP but does not wait for it --
    // SOEM's own statecheck for PRE-OP lives in ecx_map_coe_soe, which does not
    // run until ecx_config_map_group. Reading earlier means reading the mailbox
    // of a slave that may still be in INIT, where the mailbox is not active.
    //
    // Waiting for PRE-OP explicitly was not enough on Yaskawa Sigma-X hardware;
    // reading after the mapping is what actually answers. Measured, not
    // reasoned: the identical reads failed before this point and succeeded
    // after it. Only the byte offsets WITHIN each slave's area are computed
    // here, and those do not depend on where the mapping put that area, so
    // nothing is lost by doing it late.
    //
    // Rewriting a mapping is a different matter and does require PRE-OP.
    for (int i = 1; i <= slave_count_; i++) {
        (void)discover_pdo_map(i);
    }

    // Register CoE slaves for cyclic mailbox handling — after the mapping, and
    // after discovery. ecx_slavembxcyclic bails out unless slavelist[i].mbxstatus
    // is set, and that pointer is assigned inside ecx_config_map_group, so any
    // earlier it returns 0 and does nothing at all.
    //
    // After discovery because once a slave is cyclic, ecx_mbxsend queues the
    // request and waits for ecx_mbxhandler to service it -- and nothing is
    // pumping that queue until the caller starts cycling. Every SDO issued
    // between here and the first exchange() would block until timeout.
    if (cfg_.mailbox_per_cycle > 0) {
        for (int i = 1; i <= slave_count_; i++) {
            if (slaves_[i].has_coe) {
                cyclic_mailbox_ |= ecx_slavembxcyclic(&ctx_, static_cast<std::uint16_t>(i)) > 0;
            }
        }
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

    struct timespec t0 {};
    struct timespec t1 {};
    (void)clock_gettime(CLOCK_MONOTONIC, &t0);

    (void)ecx_send_processdata(&ctx_);
    const int wkc = ecx_receive_processdata(&ctx_, rx_timeout_us_);

    (void)clock_gettime(CLOCK_MONOTONIC, &t1);
    st.exchange_ns = ((t1.tv_sec - t0.tv_sec) * 1'000'000'000LL) + (t1.tv_nsec - t0.tv_nsec);
    if (st.exchange_ns > max_exchange_ns_) {
        max_exchange_ns_ = st.exchange_ns;
    }

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
        if (st.exchange_ns > max_failed_exchange_ns_) {
            max_failed_exchange_ns_ = st.exchange_ns;
        }
        if (min_failed_wkc_ < -1 || wkc < min_failed_wkc_) {
            min_failed_wkc_ = wkc;
        }
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

BusResult Bus::request_operational() noexcept {
    if (state_ != BusState::SafeOp) {
        return BusResult::NotConfigured;
    }
    if (cycles_ == 0) {
        return BusResult::OperationalFailed;
    }
    if (dc_available_ && !dc_.locked()) {
        return BusResult::DcNotLocked;
    }

    // Write the request and return. The slaves take their own time to accept
    // it; waiting here would block the caller's cycle for state_timeout_us,
    // which at 200 ms against a 1 ms cycle is a two-hundred-cycle overrun.
    ctx_.slavelist[0].state = EC_STATE_OPERATIONAL;
    (void)ecx_writestate(&ctx_, 0);
    return BusResult::Ok;
}

bool Bus::poll_operational() noexcept {
    if (state_ == BusState::Operational) {
        return true;
    }
    if (state_ != BusState::SafeOp) {
        return false;
    }
    if ((ecx_readstate(&ctx_) & 0x0F) != EC_STATE_OPERATIONAL) {
        return false;
    }
    state_ = BusState::Operational;
    return true;
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

bool PortErrors::clean() const noexcept {
    return total() == 0;
}

bool PortErrors::detected_physical_error() const noexcept {
    for (int i = 0; i < 4; i++) {
        if (invalid_frame[i] != 0 || rx_error[i] != 0) {
            return true;
        }
    }
    return false;
}

bool PortErrors::detected_error() const noexcept {
    // The processing unit belongs here. Leaving it out let a run where a slave
    // counted 18 processing-unit errors be reported as "no port detected
    // damage", which is true of the ports and false of the device.
    return detected_physical_error() || processing_unit_error != 0;
}

bool PortErrors::forwarded_only() const noexcept {
    return !detected_error() && total() > 0;
}

unsigned PortErrors::total() const noexcept {
    unsigned n = processing_unit_error + pdi_error;
    for (int i = 0; i < 4; i++) {
        n += invalid_frame[i];
        n += rx_error[i];
        n += forwarded_rx_error[i];
        n += lost_link[i];
    }
    return n;
}

bool Bus::read_port_errors(int slave, PortErrors& out) noexcept {
    if (state_ == BusState::Closed || slave < 1 || slave > slave_count_) {
        return false;
    }

    // 0x0300..0x0313 in one datagram: per-port RX error counters, forwarded RX
    // error counters, the processing-unit and PDI counters, then per-port lost
    // link counters. Layout is ETG.1000.4 and is the same on every ESC.
    std::uint8_t buf[0x14] = {};
    const std::uint16_t adr = ctx_.slavelist[slave].configadr;
    if (ecx_FPRD(&ctx_.port, adr, 0x0300, sizeof(buf), buf, EC_TIMEOUTRET) <= 0) {
        return false;
    }

    out = PortErrors{};
    for (int p = 0; p < 4; p++) {
        out.invalid_frame[p] = buf[(p * 2) + 0];
        out.rx_error[p] = buf[(p * 2) + 1];
        out.forwarded_rx_error[p] = buf[0x08 + p];
        out.lost_link[p] = buf[0x10 + p];
    }
    out.processing_unit_error = buf[0x0C];
    out.pdi_error = buf[0x0D];
    return true;
}

bool Bus::clear_port_errors(int slave) noexcept {
    if (state_ == BusState::Closed || slave < 1 || slave > slave_count_) {
        return false;
    }
    std::uint8_t zero[0x14] = {};
    const std::uint16_t adr = ctx_.slavelist[slave].configadr;
    return ecx_FPWR(&ctx_.port, adr, 0x0300, sizeof(zero), zero, EC_TIMEOUTRET) > 0;
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

bool Bus::write_sdo(int slave, std::uint16_t index, std::uint8_t subindex, const void* data,
                    int size) noexcept {
    if (state_ == BusState::Closed || slave < 1 || slave > slave_count_ || data == nullptr ||
        size <= 0) {
        return false;
    }
    // ecx_SDOwrite does not modify the buffer, but takes it non-const.
    return ecx_SDOwrite(&ctx_, static_cast<std::uint16_t>(slave), index, subindex, FALSE, size,
                        const_cast<void*>(data), EC_TIMEOUTRXM) > 0;
}

bool Bus::write_sdo_u8(int slave, std::uint16_t index, std::uint8_t subindex,
                       std::uint8_t value) noexcept {
    return write_sdo(slave, index, subindex, &value, static_cast<int>(sizeof(value)));
}

bool Bus::write_sdo_u16(int slave, std::uint16_t index, std::uint8_t subindex,
                        std::uint16_t value) noexcept {
    return write_sdo(slave, index, subindex, &value, static_cast<int>(sizeof(value)));
}

bool Bus::write_sdo_u32(int slave, std::uint16_t index, std::uint8_t subindex,
                        std::uint32_t value) noexcept {
    return write_sdo(slave, index, subindex, &value, static_cast<int>(sizeof(value)));
}

bool Bus::write_sdo_i8(int slave, std::uint16_t index, std::uint8_t subindex,
                       std::int8_t value) noexcept {
    return write_sdo(slave, index, subindex, &value, static_cast<int>(sizeof(value)));
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
