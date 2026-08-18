// Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.
//
// Tests for PDO offset discovery and DC drift compensation.
//
// Both are the parts of the fieldbus layer that can be wrong silently. A PDO
// offset that is off by two bytes still produces a valid working counter and a
// moving axis — just to the wrong position. A DC controller that corrects the
// wrong way still runs, and the bus drops out minutes later under load.

#include "frcnc/fieldbus/dc_sync.hpp"
#include "frcnc/fieldbus/pdo_map.hpp"
#include "frcnc/fieldbus/preop_config.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <initializer_list>

namespace {

int g_failures = 0;
int g_checks = 0;

void check(bool cond, const char* expr, const char* file, int line) {
    g_checks++;
    if (!cond) {
        g_failures++;
        std::printf("  FAIL %s:%d  %s\n", file, line, expr);
    }
}

#define CHECK(expr) check((expr), #expr, __FILE__, __LINE__)

using namespace frcnc::fieldbus;

// --- mapping entry decode ---------------------------------------------------

void test_decode_mapping_entry() {
    std::uint16_t idx = 0;
    std::uint8_t sub = 0;
    std::uint8_t bits = 0;

    decode_mapping_entry(0x60400010u, idx, sub, bits);
    CHECK(idx == 0x6040 && sub == 0x00 && bits == 16);

    decode_mapping_entry(0x607A0020u, idx, sub, bits);
    CHECK(idx == 0x607A && sub == 0x00 && bits == 32);

    decode_mapping_entry(0x60610008u, idx, sub, bits);
    CHECK(idx == 0x6061 && sub == 0x00 && bits == 8);

    // Padding: index 0, still occupies bits.
    decode_mapping_entry(0x00000008u, idx, sub, bits);
    CHECK(idx == 0 && bits == 8);

    // Non-zero subindex.
    decode_mapping_entry(0x60C20120u, idx, sub, bits);
    CHECK(idx == 0x60C2 && sub == 0x01 && bits == 32);
}

// --- offset accumulation ----------------------------------------------------

void test_minimal_csp_mapping() {
    // Yaskawa 0x1601 / 0x1A01: the minimal CSP pair.
    PdoMapBuilder b;

    b.begin(PdoDirection::Rx);
    CHECK(b.add_raw(0x60400010u));  // controlword, 2 bytes at 0
    CHECK(b.add_raw(0x607A0020u));  // target position, 4 bytes at 2
    b.end();

    b.begin(PdoDirection::Tx);
    CHECK(b.add_raw(0x60410010u));  // statusword, 2 bytes at 0
    CHECK(b.add_raw(0x60640020u));  // position actual, 4 bytes at 2
    b.end();

    const AxisPdoMap& m = b.result();
    CHECK(m.controlword.byte_offset == 0);
    CHECK(m.target_position.byte_offset == 2);
    CHECK(m.statusword.byte_offset == 0);
    CHECK(m.position_actual.byte_offset == 2);
    CHECK(m.rx_bytes == 6);
    CHECK(m.tx_bytes == 6);
    CHECK(m.usable_for_csp());
}

void test_full_yaskawa_mapping() {
    // Yaskawa 0x1600 / 0x1A00 as recorded in docs/06-vendor-notes.md.
    PdoMapBuilder b;

    b.begin(PdoDirection::Rx);
    CHECK(b.add_raw(0x60400010u));  // controlword       @0,  2
    CHECK(b.add_raw(0x607A0020u));  // target position   @2,  4
    CHECK(b.add_raw(0x60FF0020u));  // target velocity   @6,  4  (not tracked)
    CHECK(b.add_raw(0x60710010u));  // target torque     @10, 2  (not tracked)
    CHECK(b.add_raw(0x60720010u));  // max torque        @12, 2  (not tracked)
    CHECK(b.add_raw(0x60600008u));  // modes of op       @14, 1
    CHECK(b.add_raw(0x60B80010u));  // touch probe fn    @15, 2
    b.end();

    const AxisPdoMap& m = b.result();
    CHECK(m.controlword.byte_offset == 0);
    CHECK(m.target_position.byte_offset == 2);
    CHECK(m.modes_of_operation.byte_offset == 14);
    CHECK(m.touch_probe_function.byte_offset == 15);
    CHECK(m.rx_bytes == 17);
}

void test_padding_shifts_following_entries() {
    // The failure mode hardcoded offsets produce. Padding names nothing but
    // consumes space, so everything after it moves.
    PdoMapBuilder b;

    b.begin(PdoDirection::Rx);
    CHECK(b.add_raw(0x60400010u));  // controlword @0
    CHECK(b.add_raw(0x00000010u));  // 16 bits of padding
    CHECK(b.add_raw(0x607A0020u));  // target position @4, NOT @2
    b.end();

    const AxisPdoMap& m = b.result();
    CHECK(m.controlword.byte_offset == 0);
    CHECK(m.target_position.byte_offset == 4);
    CHECK(m.rx_bytes == 8);
}

void test_feedforward_entries_are_located() {
    PdoMapBuilder b;
    b.begin(PdoDirection::Rx);
    CHECK(b.add_raw(0x60400010u));  // @0
    CHECK(b.add_raw(0x607A0020u));  // @2
    CHECK(b.add_raw(0x60B10020u));  // velocity offset @6
    CHECK(b.add_raw(0x60B20010u));  // torque offset   @10
    b.end();

    const AxisPdoMap& m = b.result();
    CHECK(m.has_velocity_feedforward());
    CHECK(m.has_torque_feedforward());
    CHECK(m.velocity_offset.byte_offset == 6);
    CHECK(m.torque_offset.byte_offset == 10);
}

void test_missing_required_object_is_detected() {
    // A drive that does not map the statusword cannot be used for CSP, and the
    // master must refuse rather than read zeros and think it is fault-free.
    PdoMapBuilder b;
    b.begin(PdoDirection::Rx);
    CHECK(b.add_raw(0x60400010u));
    CHECK(b.add_raw(0x607A0020u));
    b.end();
    b.begin(PdoDirection::Tx);
    CHECK(b.add_raw(0x60640020u));  // position only, no statusword
    b.end();

    CHECK(!b.result().usable_for_csp());
    CHECK(!b.result().statusword.valid);
}

void test_unmapped_optional_entries_stay_invalid() {
    PdoMapBuilder b;
    b.begin(PdoDirection::Rx);
    CHECK(b.add_raw(0x60400010u));
    CHECK(b.add_raw(0x607A0020u));
    b.end();
    b.begin(PdoDirection::Tx);
    CHECK(b.add_raw(0x60410010u));
    CHECK(b.add_raw(0x60640020u));
    b.end();

    const AxisPdoMap& m = b.result();
    CHECK(!m.following_error.valid);
    CHECK(!m.digital_inputs.valid);
    CHECK(m.usable_for_csp());  // still fine; those are optional
}

void test_add_without_begin_is_rejected() {
    PdoMapBuilder b;
    CHECK(!b.add_raw(0x60400010u));
}

void test_sub_byte_mapping_rounds_up() {
    // Simple digital I/O slaves map single bits.
    PdoMapBuilder b;
    b.begin(PdoDirection::Rx);
    for (int i = 0; i < 4; i++) {
        CHECK(b.add(0x7000, static_cast<std::uint8_t>(i + 1), 1));
    }
    b.end();
    CHECK(b.result().rx_bytes == 1);
}

// --- typed access -----------------------------------------------------------

void test_roundtrip_little_endian() {
    std::uint8_t buf[32];
    std::memset(buf, 0xAA, sizeof(buf));

    PdoEntry cw{0x6040, 0, 16, 0, true};
    PdoEntry tp{0x607A, 0, 32, 2, true};
    PdoEntry mo{0x6060, 0, 8, 6, true};
    PdoEntry to{0x60B2, 0, 16, 7, true};

    write_u16(buf, cw, 0x000F);
    write_i32(buf, tp, -123456);
    write_i8(buf, mo, 8);
    write_i16(buf, to, -300);

    CHECK(read_u16(buf, cw) == 0x000F);
    CHECK(read_i32(buf, tp) == -123456);
    CHECK(read_i8(buf, mo) == 8);
    CHECK(read_i16(buf, to) == -300);

    // Little-endian on the wire, regardless of host byte order.
    CHECK(buf[0] == 0x0F && buf[1] == 0x00);
}

void test_unaligned_access_is_safe() {
    // Padding puts a 32-bit object on an odd byte. Reading it through a cast
    // would be undefined behaviour; byte-wise access must work.
    std::uint8_t buf[32];
    std::memset(buf, 0, sizeof(buf));

    PdoEntry odd{0x607A, 0, 32, 3, true};
    write_i32(buf, odd, 0x12345678);
    CHECK(read_i32(buf, odd) == 0x12345678);
    CHECK(buf[3] == 0x78 && buf[6] == 0x12);
}

void test_invalid_entry_access_is_a_no_op() {
    std::uint8_t buf[8];
    std::memset(buf, 0x5A, sizeof(buf));

    PdoEntry absent{};  // valid == false
    write_i32(buf, absent, 0x11223344);

    // Nothing written, and reads return zero rather than garbage.
    CHECK(buf[0] == 0x5A);
    CHECK(read_i32(buf, absent) == 0);
    CHECK(read_u16(buf, absent) == 0);
}

void test_null_base_is_a_no_op() {
    PdoEntry e{0x6040, 0, 16, 0, true};
    write_u16(nullptr, e, 1);  // must not crash
    CHECK(read_u16(nullptr, e) == 0);
}

// --- DC phase error ---------------------------------------------------------

void test_phase_error_sign_and_fold() {
    const std::int64_t cycle = 1'000'000;  // 1 ms

    // Exactly in phase.
    CHECK(dc_phase_error(0, cycle, 0) == 0);
    CHECK(dc_phase_error(cycle * 5, cycle, 0) == 0);

    // Slightly late: correction must be negative to pull back.
    CHECK(dc_phase_error(1000, cycle, 0) == -1000);

    // Just before the boundary: this is a SMALL positive error, not a
    // nearly-whole-cycle negative one. Without folding the controller would
    // correct the long way round and never settle.
    CHECK(dc_phase_error(cycle - 1000, cycle, 0) == 1000);

    // Shift moves the target point.
    CHECK(dc_phase_error(250'000, cycle, 250'000) == 0);
}

void test_phase_error_handles_negative_times() {
    const std::int64_t cycle = 1'000'000;
    // C's % keeps the dividend's sign; the implementation must normalise.
    const std::int64_t e = dc_phase_error(-1000, cycle, 0);
    CHECK(e == 1000);
}

void test_phase_error_zero_cycle_is_safe() {
    CHECK(dc_phase_error(12345, 0, 0) == 0);
}

// --- DC controller ----------------------------------------------------------

DcSyncConfig sync_config(std::int64_t cycle = 250'000) {
    DcSyncConfig c;
    c.cycle_ns = cycle;
    c.shift_ns = cycle / 4;
    c.p_gain = 0.10;
    c.i_gain = 0.02;
    c.max_correction_ns = cycle / 10;
    c.lock_tolerance_ns = 500;
    c.lock_cycles = 50;
    return c;
}

/// Simulate the two clocks.
///
/// The master wakes every (cycle + correction) of ITS OWN time. The reference
/// clock runs at a slightly different rate, so the DC time observed at each
/// wake-up is the master's elapsed time scaled by (1 + ppm), plus whatever
/// constant offset the two clocks started with.
struct ClockSim {
    std::int64_t cycle = 250'000;
    double ppm = 0.0;                ///< reference clock fast by this fraction
    std::int64_t initial_offset = 0;
    std::int64_t master_time = 0;

    /// Advance one cycle and return the DC time seen at the new wake-up.
    std::int64_t step(std::int64_t correction) {
        master_time += cycle + correction;
        return static_cast<std::int64_t>(static_cast<double>(master_time) * (1.0 + ppm)) +
               initial_offset;
    }
};

void test_controller_converges_from_an_initial_offset() {
    DcSync dc;
    dc.configure(sync_config());

    ClockSim sim;
    sim.cycle = 250'000;
    sim.initial_offset = 40'000;  // start well out of phase

    for (int i = 0; i < 2000; i++) {
        (void)dc.update(sim.step(dc.correction_ns()));
    }

    CHECK(std::abs(dc.error_ns()) < 500);
    CHECK(dc.cycles() == 2000);
    CHECK(dc.locked());
}

void test_controller_tracks_a_constant_rate_mismatch() {
    // The case the integral term exists for: the two clocks run at different
    // rates, so a purely proportional controller would settle with a permanent
    // offset.
    DcSync dc;
    dc.configure(sync_config());

    ClockSim sim;
    sim.cycle = 250'000;
    sim.ppm = 100e-6;  // reference clock 100 ppm fast

    for (int i = 0; i < 20000; i++) {
        (void)dc.update(sim.step(dc.correction_ns()));
    }

    // Each cycle the reference gains ppm * cycle nanoseconds, so the master
    // must SHORTEN its cycle by the same amount to hold phase. The steady-state
    // correction is therefore the negative of the drift -- getting this sign
    // backwards produces a controller that runs away instead of locking.
    const std::int64_t drift_per_cycle =
        static_cast<std::int64_t>(sim.ppm * static_cast<double>(sim.cycle));
    CHECK(drift_per_cycle == 25);
    CHECK(std::abs(dc.correction_ns() + drift_per_cycle) < 15);
    CHECK(std::abs(dc.error_ns()) < 500);
}

void test_correction_is_bounded() {
    DcSync dc;
    DcSyncConfig cfg = sync_config();
    cfg.max_correction_ns = 5000;
    dc.configure(cfg);

    // A wildly out-of-phase reading must not produce an unbounded sleep change.
    for (int i = 0; i < 100; i++) {
        const std::int64_t c = dc.update(123'456);
        CHECK(std::abs(c) <= 5000);
    }
}

void test_lock_requires_a_sustained_run() {
    DcSync dc;
    DcSyncConfig cfg = sync_config();
    cfg.lock_tolerance_ns = 500;
    cfg.lock_cycles = 10;
    dc.configure(cfg);

    // In phase (shift is cycle/4, so feeding exactly that gives zero error).
    for (int i = 0; i < 9; i++) {
        (void)dc.update(cfg.shift_ns);
        CHECK(!dc.locked());
    }
    (void)dc.update(cfg.shift_ns);
    CHECK(dc.locked());

    // A single excursion breaks the run immediately.
    (void)dc.update(cfg.shift_ns + 50'000);
    CHECK(!dc.locked());
    CHECK(dc.lock_run() == 0);
}

void test_locked_is_a_pure_query() {
    DcSync dc;
    DcSyncConfig cfg = sync_config();
    cfg.lock_cycles = 5;
    dc.configure(cfg);

    for (int i = 0; i < 5; i++) {
        (void)dc.update(cfg.shift_ns);
    }
    // Repeated queries must not advance or reset the run.
    const std::uint32_t run = dc.lock_run();
    for (int i = 0; i < 100; i++) {
        CHECK(dc.locked());
    }
    CHECK(dc.lock_run() == run);
}

void test_peak_error_is_retained() {
    DcSync dc;
    dc.configure(sync_config());

    (void)dc.update(sync_config().shift_ns + 30'000);
    const std::int64_t peak = dc.peak_error_ns();
    CHECK(peak >= 29'000);

    for (int i = 0; i < 100; i++) {
        (void)dc.update(sync_config().shift_ns);
    }
    // The peak survives even though the current error is now ~0.
    CHECK(dc.peak_error_ns() == peak);

    dc.reset();
    CHECK(dc.peak_error_ns() == 0);
    CHECK(dc.cycles() == 0);
}

void test_zero_cycle_config_is_safe() {
    DcSync dc;
    DcSyncConfig cfg;
    cfg.cycle_ns = 0;
    dc.configure(cfg);
    CHECK(dc.update(12345) == 0);
}

}  // namespace

void test_dc_lock_tolerance_scales_with_the_cycle() {
    // A constant tolerance is wrong across cycle times: the drift to be
    // rejected between corrections is proportional to the cycle, so the
    // residual ripple is too. Measured on hardware -- a few hundred ns of error
    // at 1 ms became 1-4 us at 4 ms with TWELVE TIMES less frame loss, so the
    // fixed 1 us threshold made a healthier bus look like a failing one.
    struct Case {
        std::int64_t cycle_ns;
        std::int64_t expect_tolerance_ns;
    };
    const Case cases[] = {
        {250'000, 1'000},    // floor applies
        {1'000'000, 1'000},  // floor applies
        {4'000'000, 4'000},  // scales
        {8'000'000, 8'000},
    };

    for (const Case& c : cases) {
        DcSyncConfig cfg;
        cfg.cycle_ns = c.cycle_ns;
        cfg.lock_tolerance_ns = 0;  // derive
        cfg.lock_cycles = 1;

        DcSync dc;
        dc.configure(cfg);

        // Just inside tolerance locks; just outside does not.
        const std::int64_t inside = c.expect_tolerance_ns - 1;
        (void)dc.update(cfg.shift_ns + inside);
        CHECK(dc.locked());

        DcSync dc2;
        dc2.configure(cfg);
        const std::int64_t outside = c.expect_tolerance_ns + 1;
        (void)dc2.update(cfg.shift_ns + outside);
        CHECK(!dc2.locked());
    }
}

void test_dc_explicit_lock_tolerance_is_honoured() {
    DcSyncConfig cfg;
    cfg.cycle_ns = 4'000'000;
    cfg.lock_tolerance_ns = 500;  // pinned, must not be widened to 4000
    cfg.lock_cycles = 1;

    DcSync dc;
    dc.configure(cfg);
    (void)dc.update(cfg.shift_ns + 600);
    CHECK(!dc.locked());
}

// --- 0x60C2 interpolation time period ---------------------------------------
//
// The only value in the PRE-OP configuration that is computed rather than
// copied, and getting it wrong is invisible: the bus comes up, the axis moves,
// and every commanded feed is out by exactly the ratio of the two periods.

void test_interpolation_period_round_trips() {
    // Whatever the encoding chooses, it must mean the period it was given.
    for (std::int64_t ns : {125'000LL, 250'000LL, 500'000LL, 1'000'000LL, 2'000'000LL,
                            4'000'000LL, 1'500'000LL, 10'000'000LL}) {
        const InterpolationPeriod p = encode_interpolation_period(ns);
        CHECK(p.valid);
        CHECK(p.to_ns() == ns);
    }
}

void test_interpolation_period_uses_the_forms_drives_report() {
    // Whole milliseconds in the (n, -3) form, sub-millisecond in microseconds.
    // Matching what the device itself reports is what makes a readback
    // comparison legible in a log.
    const InterpolationPeriod ms4 = encode_interpolation_period(4'000'000);
    CHECK(ms4.units == 4);
    CHECK(ms4.exponent == -3);

    const InterpolationPeriod us125 = encode_interpolation_period(125'000);
    CHECK(us125.units == 125);
    CHECK(us125.exponent == -6);

    const InterpolationPeriod us250 = encode_interpolation_period(250'000);
    CHECK(us250.units == 250);
    CHECK(us250.exponent == -6);
}

void test_interpolation_period_coarsens_when_the_byte_forces_it() {
    // 500 us cannot be (500, -6): 0x60C2:01 is UNSIGNED8. It must coarsen a
    // decade rather than truncate, overflow, or claim success.
    const InterpolationPeriod p = encode_interpolation_period(500'000);
    CHECK(p.valid);
    CHECK(p.units == 50);
    CHECK(p.exponent == -5);
    CHECK(p.to_ns() == 500'000);
}

void test_interpolation_period_rejects_what_it_cannot_represent() {
    // Sub-microsecond, and a period needing more significant digits than a
    // byte holds. Refusing is the only safe answer -- a rounded value would
    // make every feed wrong by a fraction nobody would think to look for.
    CHECK(!encode_interpolation_period(1'500).valid);   // 1.5 us
    CHECK(!encode_interpolation_period(0).valid);
    CHECK(!encode_interpolation_period(-1'000'000).valid);
    CHECK(!encode_interpolation_period(1'234'000).valid);  // 1234 us
    CHECK(encode_interpolation_period(-1'000'000).to_ns() == 0);
}

void test_preop_config_starts_empty_and_records_writes() {
    PreOpConfig cfg;
    CHECK(cfg.empty());

    cfg.rx_pdo_assign = 0x1600;
    CHECK(!cfg.empty());

    PreOpConfig other;
    CHECK(other.add(SdoWrite{2, 0x6098, 0x00, 1, 35}));
    CHECK(other.write_count == 1);
    CHECK(!other.empty());

    // Malformed writes are refused rather than stored: a zero-width SDO write
    // is not a smaller write, it is a bug.
    CHECK(!other.add(SdoWrite{1, 0x6040, 0x00, 3, 0}));
    CHECK(!other.add(SdoWrite{1, 0x6040, 0x00, 0, 0}));
    CHECK(other.write_count == 1);
}

void test_preop_write_targets_one_slave_or_all() {
    const SdoWrite everywhere{0, 0x60C2, 0x01, 1, 4};
    CHECK(everywhere.applies_to(1));
    CHECK(everywhere.applies_to(7));

    const SdoWrite just_two{2, 0x60C2, 0x01, 1, 4};
    CHECK(!just_two.applies_to(1));
    CHECK(just_two.applies_to(2));
}

void test_preop_write_table_is_bounded() {
    PreOpConfig cfg;
    for (int i = 0; i < PreOpConfig::kMaxWrites; i++) {
        CHECK(cfg.add(SdoWrite{0, 0x2000, static_cast<std::uint8_t>(i), 2, 1}));
    }
    CHECK(!cfg.add(SdoWrite{0, 0x2000, 0x00, 2, 1}));
    CHECK(cfg.write_count == PreOpConfig::kMaxWrites);
}

int main() {
    test_dc_lock_tolerance_scales_with_the_cycle();
    test_dc_explicit_lock_tolerance_is_honoured();
    std::printf("test_fieldbus\n");

    test_decode_mapping_entry();
    test_minimal_csp_mapping();
    test_full_yaskawa_mapping();
    test_padding_shifts_following_entries();
    test_feedforward_entries_are_located();
    test_missing_required_object_is_detected();
    test_unmapped_optional_entries_stay_invalid();
    test_add_without_begin_is_rejected();
    test_sub_byte_mapping_rounds_up();

    test_roundtrip_little_endian();
    test_unaligned_access_is_safe();
    test_invalid_entry_access_is_a_no_op();
    test_null_base_is_a_no_op();

    test_phase_error_sign_and_fold();
    test_phase_error_handles_negative_times();
    test_phase_error_zero_cycle_is_safe();

    test_controller_converges_from_an_initial_offset();
    test_controller_tracks_a_constant_rate_mismatch();
    test_correction_is_bounded();
    test_lock_requires_a_sustained_run();
    test_locked_is_a_pure_query();
    test_peak_error_is_retained();
    test_zero_cycle_config_is_safe();

    test_interpolation_period_round_trips();
    test_interpolation_period_uses_the_forms_drives_report();
    test_interpolation_period_coarsens_when_the_byte_forces_it();
    test_interpolation_period_rejects_what_it_cannot_represent();
    test_preop_config_starts_empty_and_records_writes();
    test_preop_write_targets_one_slave_or_all();
    test_preop_write_table_is_bounded();

    std::printf("  %d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
