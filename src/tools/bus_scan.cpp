// Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.
//
// bus_scan — enumerate an EtherCAT segment and report what each device
// actually advertises, rather than what its datasheet claims.
//
// This is the first thing to run against new hardware. It answers, from the
// device itself:
//   - Does the NIC bind and can we send raw frames? (works with ZERO slaves)
//   - Vendor ID / product code / revision, for slave identity checks
//   - CoE or SoE? Settles the profile question in seconds
//   - Does the slave's SII request blockLRW? (Yaskawa Sigma-7 does)
//   - The REAL PDO byte offsets, read back over SDO rather than assumed
//
// Running it with no slaves connected is still useful: it proves the socket,
// permissions and driver path work before any drive is on the bench.
//
//   sudo ./build/bus_scan <interface>
//   sudo ./build/bus_scan <interface> --cycle 250 --block-lrw
//
// Reference: docs/03-ethercat-bringup.md, docs/06-vendor-notes.md

#include "frcnc/drive/cia402.hpp"
#include "frcnc/fieldbus/bus.hpp"

#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>

using namespace frcnc::fieldbus;

namespace {

void print_entry(const char* label, const PdoEntry& e) {
    if (!e.valid) {
        std::printf("      %-22s  --\n", label);
        return;
    }
    std::printf("      %-22s  0x%04X:%02X  %2u bits  @ byte %u\n", label, e.index, e.subindex,
                e.bit_length, e.byte_offset);
}

void print_pdo_map(const AxisPdoMap& m) {
    std::printf("    PDO map (read back from the device):\n");
    std::printf("      RxPDO %u bytes, TxPDO %u bytes\n", m.rx_bytes, m.tx_bytes);

    std::printf("    --- master to drive ---\n");
    print_entry("Controlword", m.controlword);
    print_entry("Target position", m.target_position);
    print_entry("Modes of operation", m.modes_of_operation);
    print_entry("Velocity offset", m.velocity_offset);
    print_entry("Torque offset", m.torque_offset);
    print_entry("Touch probe function", m.touch_probe_function);

    std::printf("    --- drive to master ---\n");
    print_entry("Statusword", m.statusword);
    print_entry("Position actual", m.position_actual);
    print_entry("Modes display", m.modes_display);
    print_entry("Following error", m.following_error);
    print_entry("Velocity actual", m.velocity_actual);
    print_entry("Torque actual", m.torque_actual);
    print_entry("Digital inputs", m.digital_inputs);

    std::printf("    CSP usable          : %s\n", m.usable_for_csp() ? "YES" : "NO");
    std::printf("    Velocity feedforward: %s\n", m.has_velocity_feedforward() ? "yes" : "no");
    std::printf("    Torque feedforward  : %s\n", m.has_torque_feedforward() ? "yes" : "no");

    if (!m.usable_for_csp()) {
        std::printf("    ** Missing a required CSP object. Check the PDO assignment.\n");
    }
    if (!m.has_velocity_feedforward()) {
        std::printf("    ** No 0x60B1. Without velocity feedforward the servo loop must\n");
        std::printf("       generate following error to produce velocity, which distorts\n");
        std::printf("       the path on every corner. See docs/05-motion-architecture.md.\n");
    }
}

/// Read the CiA 402 diagnostic objects over SDO.
///
/// This is what answers "the drive is sitting in Fault -- why?". The statusword
/// only says that it faulted; 0x603F says what the drive thinks went wrong, and
/// 0x1001 says which class of problem it is. With no motor connected, expect an
/// encoder or motor-detection alarm here -- that is correct, not a bus problem.
void print_drive_diagnostics(Bus& bus, int slave) {
    std::printf("    --- CiA 402 diagnostics (SDO) ---\n");

    std::uint32_t modes = 0;
    if (bus.read_sdo_u32(slave, 0x6502, 0x00, modes)) {
        // 0x6502 bit assignments, IEC 61800-7-201. Bit 7 is the one a CNC needs.
        std::printf("      Supported modes (0x6502) : 0x%08" PRIX32 "  %s%s%s%s\n", modes,
                    (modes & (1u << 7)) ? "CSP " : "",
                    (modes & (1u << 8)) ? "CSV " : "",
                    (modes & (1u << 6)) ? "IP " : "",
                    (modes & (1u << 5)) ? "HM" : "");
        if ((modes & (1u << 7)) == 0) {
            std::printf("      ** No CSP. This drive cannot do coordinated CNC motion.\n");
        }
    } else {
        std::printf("      Supported modes (0x6502) : not readable\n");
    }

    std::int8_t mode_display = 0;
    if (bus.read_sdo_i8(slave, 0x6061, 0x00, mode_display)) {
        std::printf("      Mode display    (0x6061) : %d\n", mode_display);
    }

    std::uint16_t statusword = 0;
    if (bus.read_sdo_u16(slave, 0x6041, 0x00, statusword)) {
        std::printf("      Statusword      (0x6041) : 0x%04X  %s\n", statusword,
                    frcnc::drive::to_string(frcnc::drive::decode_state(statusword)));
    }

    std::uint8_t error_register = 0;
    if (bus.read_sdo_u8(slave, 0x1001, 0x00, error_register)) {
        std::printf("      Error register  (0x1001) : 0x%02X\n", error_register);
    }

    std::uint16_t error_code = 0;
    if (bus.read_sdo_u16(slave, 0x603F, 0x00, error_code)) {
        std::printf("      Error code      (0x603F) : 0x%04X%s\n", error_code,
                    error_code == 0 ? "  (no fault)" : "");
        if (error_code != 0) {
            std::printf("      ** Look this code up in the drive's manual. With no motor\n");
            std::printf("         connected an encoder alarm here is expected.\n");
        }
    }

    // 0x60C2 must equal the master's cycle time exactly. A drive interpolating
    // over a different period than the master produces velocity ripple that
    // looks like a mechanical problem and is not.
    std::uint8_t ip_units = 0;
    std::int8_t ip_index = 0;
    if (bus.read_sdo_u8(slave, 0x60C2, 0x01, ip_units) &&
        bus.read_sdo_i8(slave, 0x60C2, 0x02, ip_index)) {
        // value = units * 10^index seconds
        double period = static_cast<double>(ip_units);
        for (int k = 0; k < -ip_index; k++) {
            period /= 10.0;
        }
        for (int k = 0; k < ip_index; k++) {
            period *= 10.0;
        }
        std::printf("      Interp. period  (0x60C2) : %u x 10^%d s = %.1f us\n", ip_units,
                    ip_index, period * 1e6);
    }
}

void usage(const char* argv0) {
    std::printf(
        "bus_scan — enumerate an EtherCAT segment\n"
        "\n"
        "usage: %s <interface> [options]\n"
        "\n"
        "  --cycle <us>    cycle time for DC configuration (default 1000)\n"
        "  --shift <us>    Sync0 shift (default: 25%% of cycle)\n"
        "  --no-dc         skip Distributed Clocks configuration\n"
        "  --block-lrw     force LRD/LWR instead of LRW (Yaskawa Sigma-7)\n"
        "\n"
        "Must run as root (raw socket). Works with zero slaves connected —\n"
        "that still validates the NIC, socket and driver path.\n",
        argv0);
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        usage(argv[0]);
        return 2;
    }

    BusConfig cfg;
    std::snprintf(cfg.interface, sizeof(cfg.interface), "%s", argv[1]);
    cfg.cycle_ns = 1'000'000;
    cfg.use_dc = true;
    cfg.force_block_lrw = false;
    bool shift_given = false;

    for (int i = 2; i < argc; i++) {
        if (std::strcmp(argv[i], "--cycle") == 0 && i + 1 < argc) {
            cfg.cycle_ns = std::strtoll(argv[++i], nullptr, 10) * 1000;
        } else if (std::strcmp(argv[i], "--shift") == 0 && i + 1 < argc) {
            cfg.sync0_shift_ns = static_cast<std::int32_t>(std::strtol(argv[++i], nullptr, 10) * 1000);
            shift_given = true;
        } else if (std::strcmp(argv[i], "--no-dc") == 0) {
            cfg.use_dc = false;
        } else if (std::strcmp(argv[i], "--block-lrw") == 0) {
            cfg.force_block_lrw = true;
        } else {
            usage(argv[0]);
            return 2;
        }
    }
    if (!shift_given) {
        cfg.sync0_shift_ns = static_cast<std::int32_t>(cfg.cycle_ns / 4);
    }

    std::printf("=== EtherCAT bus scan: %s ===\n", cfg.interface);
    std::printf("cycle %" PRId64 " us, Sync0 shift %d us, DC %s, blockLRW %s\n\n",
                cfg.cycle_ns / 1000, cfg.sync0_shift_ns / 1000, cfg.use_dc ? "on" : "off",
                cfg.force_block_lrw ? "forced" : "auto");
    std::fflush(stdout);

    // Bus embeds SOEM's context and the IOmap; far too large for the stack.
    auto bus = std::make_unique<Bus>();

    BusResult r = bus->open(cfg);
    if (r != BusResult::Ok) {
        std::fprintf(stderr, "FAILED to open '%s': %s\n", cfg.interface, to_string(r));

        // Name it, don't make them guess. This failure looks like a permissions
        // or driver problem and almost never is -- the interface is just called
        // something else on this machine.
        char adapters[2048];
        if (list_interfaces(adapters, sizeof(adapters)) > 0) {
            std::fprintf(stderr, "\nInterfaces this machine offers:\n%s", adapters);
        }
        std::fprintf(stderr,
                     "\n  - the name must match one above exactly\n"
                     "  - run as root (raw AF_PACKET socket)\n"
                     "  - the interface must be UP:  sudo ip link set %s up\n"
                     "  - carrier is not required to open, but is to see slaves\n",
                     cfg.interface);
        return 1;
    }
    std::printf("NIC opened OK — socket, permissions and driver path all work.\n\n");

    r = bus->configure();
    if (r == BusResult::NoSlaves) {
        std::printf("No slaves found.\n\n");
        std::printf("  Expected with nothing connected, and this run still proved the\n");
        std::printf("  software path. With hardware attached, check:\n");
        std::printf("    - slave powered\n");
        std::printf("    - cable in the slave's IN port (EtherCAT is directional)\n");
        std::printf("    - link light on the NIC\n");
        bus->close();
        return 0;
    }
    if (r != BusResult::Ok) {
        std::fprintf(stderr, "configure() failed: %s\n", to_string(r));
        if (r == BusResult::NoDistributedClocks) {
            std::fprintf(stderr, "  No slave reported DC capability. Retry with --no-dc,\n");
            std::fprintf(stderr, "  but note coordinated multi-axis motion requires DC.\n");
        }
        char errbuf[1024];
        if (bus->drain_errors(errbuf, sizeof(errbuf)) > 0) {
            std::fprintf(stderr, "  EtherCAT errors: %s\n", errbuf);
        }
        bus->close();
        return 1;
    }

    std::printf("Slaves        : %d\n", bus->slave_count());
    std::printf("IOmap         : %d bytes\n", bus->iomap_size());
    std::printf("Expected WKC  : %d\n", bus->expected_wkc());
    std::printf("DC available  : %s\n", bus->dc_available() ? "yes" : "no");
    std::printf("State         : %s\n\n", to_string(bus->state()));

    for (int i = 1; i <= bus->slave_count(); i++) {
        const SlaveInfo& s = bus->slave(i);

        std::printf("--- Slave %d: %s\n", i, s.name);
        std::printf("    Vendor ID    : 0x%08" PRIX32 "\n", s.vendor_id);
        std::printf("    Product code : 0x%08" PRIX32 "\n", s.product_code);
        std::printf("    Revision     : 0x%08" PRIX32 "\n", s.revision);
        std::printf("    Mailbox      : 0x%04X%s%s\n", s.mbx_proto, s.has_coe ? " CoE" : "",
                    s.has_soe ? " SoE" : "");

        if (s.has_coe) {
            std::printf("    Profile      : CoE — CiA 402 applies\n");
        } else if (s.has_soe) {
            std::printf("    Profile      : SoE — CiA 402 does NOT apply\n");
        }

        std::printf("    Process data : %u bytes out, %u bytes in\n", s.out_bytes, s.in_bytes);
        std::printf("    hasdc        : %s\n", s.has_dc ? "yes" : "no");
        std::printf("    blockLRW     : %u%s\n", s.block_lrw ? 1u : 0u,
                    s.block_lrw ? "  (device requests LRD/LWR)" : "");

        if (s.pdo_discovered && s.has_coe) {
            print_pdo_map(s.pdo);
        }
        if (s.has_coe) {
            print_drive_diagnostics(*bus, i);
        }
        std::printf("\n");
    }

    char errbuf[1024];
    if (bus->drain_errors(errbuf, sizeof(errbuf)) > 0) {
        std::printf("=== EtherCAT errors ===\n%s\n", errbuf);
    }

    // Walks the bus down to INIT rather than dropping the socket, so nothing is
    // left energised with no master.
    bus->close();
    std::printf("Done.\n");
    return 0;
}
