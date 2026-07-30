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
#include <initializer_list>
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

/// What object is this, in CNC terms?
const char* object_role(std::uint16_t index) {
    switch (index) {
        case 0x6040: return "controlword          REQUIRED";
        case 0x607A: return "target position      REQUIRED for CSP";
        case 0x6060: return "modes of operation   sets CSP over PDO";
        case 0x60B1: return "velocity offset      VELOCITY FEEDFORWARD";
        case 0x60B2: return "torque offset        torque feedforward";
        case 0x60B8: return "touch probe function";
        case 0x6041: return "statusword           REQUIRED";
        case 0x6064: return "position actual      REQUIRED for CSP";
        case 0x6061: return "modes display        confirms CSP took";
        case 0x60F4: return "following error      master-side supervision";
        case 0x606C: return "velocity actual";
        case 0x6077: return "torque actual";
        case 0x60FD: return "digital inputs       limits, home switch";
        case 0x603F: return "error code";
        case 0x0000: return "PADDING";
        default:     return "";
    }
}

/// List every PDO mapping object the device will talk about, not just the one
/// currently assigned.
///
/// The assigned mapping is often the minimal one -- controlword and target
/// position out, statusword and position actual in -- which is enough to move
/// an axis and not enough to do it well. Velocity feedforward needs 0x60B1
/// mapped, and nothing can add it at runtime: either another predefined mapping
/// already contains it, or the mapping has to be rewritten in PRE-OP.
///
/// This says which. Print it before deciding how to configure the drive.
void print_pdo_catalog(Bus& bus, int slave) {
    std::printf("    --- PDO mapping objects this device offers ---\n");

    struct Group {
        const char* label;
        std::uint16_t first;
        std::uint16_t last;
    };
    // 0x1600-0x17FF is the RxPDO range and 0x1A00-0x1BFF the TxPDO range.
    // Scanning the first few of each finds the vendor's predefined sets without
    // issuing hundreds of SDO reads.
    const Group groups[2] = {{"master to drive", 0x1600, 0x1607},
                             {"drive to master", 0x1A00, 0x1A07}};

    for (const Group& g : groups) {
        std::printf("      %s:\n", g.label);
        bool found_any = false;

        for (std::uint16_t m = g.first; m <= g.last; m++) {
            std::uint8_t entries = 0;
            if (!bus.read_sdo_u8(slave, m, 0x00, entries)) {
                continue;  // not implemented; silence is the useful answer here
            }
            found_any = true;
            std::printf("        0x%04X  %u entries\n", m, entries);

            std::uint32_t bits = 0;
            for (std::uint8_t e = 1; e <= entries; e++) {
                std::uint32_t raw = 0;
                if (!bus.read_sdo_u32(slave, m, e, raw)) {
                    std::printf("          [%u] READ FAILED\n", e);
                    continue;
                }
                std::uint16_t idx = 0;
                std::uint8_t sub = 0;
                std::uint8_t len = 0;
                decode_mapping_entry(raw, idx, sub, len);
                bits += len;
                std::printf("          0x%04X:%02X %3u bits  %s\n", idx, sub, len,
                            object_role(idx));
            }
            if (entries > 0) {
                std::printf("          = %u bytes\n", bits / 8);
            }
        }

        if (!found_any) {
            std::printf("        none answered — fixed mapping, use the ESI XML\n");
        }
    }
}

/// Dump the PDO assignment objects raw, one SDO read at a time.
///
/// Runs when discovery came back empty. Discovery walks
/// 0x1C12/0x1C13 -> 0x16xx/0x1Axx and gives up quietly on the first read that
/// does not answer, which is useless when you need to know WHICH read failed.
/// This says so, and prints what each object actually contains.
///
/// A device with a FIXED mapping is a legitimate outcome here: the sync manager
/// assignment is then not writable and may not even be readable, while the
/// process data is still laid out correctly. In that case read the byte offsets
/// off the ESI XML and configure them by hand.
void probe_pdo_assignment(Bus& bus, int slave) {
    std::printf("    --- raw PDO assignment probe ---\n");

    struct Assign {
        const char* label;
        std::uint16_t index;
    };
    const Assign assigns[2] = {{"RxPDO assign (SM2)", 0x1C12},
                               {"TxPDO assign (SM3)", 0x1C13}};

    bool any = false;

    for (const Assign& a : assigns) {
        std::uint8_t count = 0;
        if (!bus.read_sdo_u8(slave, a.index, 0x00, count)) {
            std::printf("      0x%04X:00  READ FAILED   (%s)\n", a.index, a.label);
            continue;
        }
        any = true;
        std::printf("      0x%04X:00  = %u        (%s)\n", a.index, count, a.label);

        for (std::uint8_t p = 1; p <= count; p++) {
            std::uint16_t mapping = 0;
            if (!bus.read_sdo_u16(slave, a.index, p, mapping)) {
                std::printf("      0x%04X:%02X  READ FAILED\n", a.index, p);
                continue;
            }
            std::printf("      0x%04X:%02X  = 0x%04X\n", a.index, p, mapping);
            if (mapping == 0) {
                continue;
            }

            std::uint8_t entries = 0;
            if (!bus.read_sdo_u8(slave, mapping, 0x00, entries)) {
                std::printf("        0x%04X:00  READ FAILED\n", mapping);
                continue;
            }
            std::printf("        0x%04X:00  = %u entries\n", mapping, entries);

            for (std::uint8_t e = 1; e <= entries; e++) {
                std::uint32_t raw = 0;
                if (!bus.read_sdo_u32(slave, mapping, e, raw)) {
                    std::printf("        0x%04X:%02X  READ FAILED\n", mapping, e);
                    continue;
                }
                std::uint16_t idx = 0;
                std::uint8_t sub = 0;
                std::uint8_t bits = 0;
                decode_mapping_entry(raw, idx, sub, bits);
                std::printf("        0x%04X:%02X  = 0x%08" PRIX32 "  -> 0x%04X:%02X, %u bits\n",
                            mapping, e, raw, idx, sub, bits);
            }
        }
    }

    // Some devices refuse the assignment objects but still answer the mapping
    // objects directly. Worth asking before concluding the mapping is fixed.
    if (!any) {
        std::printf("      Assignment objects did not answer. Trying the mapping\n"
                    "      objects directly:\n");
        for (std::uint16_t m : {0x1600, 0x1601, 0x1A00, 0x1A01}) {
            std::uint8_t entries = 0;
            if (bus.read_sdo_u8(slave, m, 0x00, entries)) {
                std::printf("        0x%04X:00  = %u entries\n", m, entries);
            } else {
                std::printf("        0x%04X:00  READ FAILED\n", m);
            }
        }
        std::printf("      If none answer, this device has a FIXED mapping. Take the\n"
                    "      byte offsets from its ESI XML and configure them by hand.\n");
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

    // This tool never runs a cyclic exchange, so nothing would ever service a
    // cyclic mailbox queue. Leaving it on would make every SDO read here block
    // until timeout -- including the diagnostics this tool exists to print.
    cfg.mailbox_per_cycle = 0;
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
            // An empty map alongside non-zero process data means discovery
            // failed, not that the device maps nothing. Find out which.
            if (s.pdo.rx_bytes == 0 && s.pdo.tx_bytes == 0 &&
                (s.out_bytes > 0 || s.in_bytes > 0)) {
                std::printf(
                    "    ** Discovery found no mapping, but the device has %u/%u bytes of\n"
                    "       process data. The mapping is there; reading it failed.\n",
                    s.out_bytes, s.in_bytes);
                probe_pdo_assignment(*bus, i);
            }
            print_pdo_catalog(*bus, i);
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
