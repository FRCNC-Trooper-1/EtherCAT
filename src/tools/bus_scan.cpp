// Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.
//
// bus_scan — enumerate an EtherCAT segment and report what each slave actually
// advertises, rather than what its datasheet claims.
//
// This is the first thing to run against new hardware. It answers, from the
// device itself:
//   - Does the NIC bind and can we send raw frames? (works with ZERO slaves)
//   - Vendor ID / product code / revision, for slave identity checks
//   - CoE or SoE? Settles the profile question in five seconds
//   - Does the slave's SII request blockLRW? (Yaskawa Sigma-7 needs it)
//   - Process data sizes, and the real PDO mapping read back over SDO
//
// Running it with no slaves connected is still useful: it proves the socket,
// permissions, and driver path work before any drive is on the bench.
//
//   sudo ./build/bus_scan <interface>
//   sudo ./build/bus_scan enp1s0 --pdo
//
// Reference: docs/03-ethercat-bringup.md, docs/06-vendor-notes.md

#include "soem/soem.h"

#include <cinttypes>
#include <cstdio>
#include <cstring>
#include <string>

namespace {

// The context is large (EC_MAXSLAVE records + EEPROM cache + mailbox pool).
// Static storage: never a stack local, and zero-initialised as ecx_init()
// does not memset it.
ecx_contextt g_ctx;
uint8_t g_iomap[4096];

const char* state_name(uint16_t s) {
    switch (s & 0x0F) {
        case EC_STATE_INIT:        return "INIT";
        case EC_STATE_PRE_OP:      return "PRE-OP";
        case EC_STATE_BOOT:        return "BOOT";
        case EC_STATE_SAFE_OP:     return "SAFE-OP";
        case EC_STATE_OPERATIONAL: return "OP";
        default:                   return "?";
    }
}

void print_mailbox_protocols(uint16_t proto) {
    if (proto == 0) {
        std::printf("none");
        return;
    }
    bool first = true;
    auto emit = [&](const char* s) {
        std::printf("%s%s", first ? "" : "+", s);
        first = false;
    };
    if (proto & ECT_MBXPROT_AOE) emit("AoE");
    if (proto & ECT_MBXPROT_EOE) emit("EoE");
    if (proto & ECT_MBXPROT_COE) emit("CoE");
    if (proto & ECT_MBXPROT_FOE) emit("FoE");
    if (proto & ECT_MBXPROT_SOE) emit("SoE");
    if (proto & ECT_MBXPROT_VOE) emit("VoE");
}

/// Read back a PDO mapping object and print its entries.
///
/// Default mapping contents are vendor-specific. Never hardcode offsets --
/// read what the device reports. See docs/03 §9.
void dump_pdo_mapping(uint16_t slave, uint16_t index) {
    uint8_t count = 0;
    int size = sizeof(count);
    if (ecx_SDOread(&g_ctx, slave, index, 0x00, FALSE, &size, &count, EC_TIMEOUTRXM) <= 0) {
        return;
    }
    if (count == 0) {
        return;
    }

    std::printf("      0x%04X: %u entr%s\n", index, count, count == 1 ? "y" : "ies");
    for (uint8_t i = 1; i <= count; i++) {
        uint32_t entry = 0;
        size = sizeof(entry);
        if (ecx_SDOread(&g_ctx, slave, index, i, FALSE, &size, &entry, EC_TIMEOUTRXM) <= 0) {
            continue;
        }
        const uint16_t obj = static_cast<uint16_t>((entry >> 16) & 0xFFFF);
        const uint8_t sub = static_cast<uint8_t>((entry >> 8) & 0xFF);
        const uint8_t bits = static_cast<uint8_t>(entry & 0xFF);
        if (obj == 0) {
            std::printf("        %2u. padding, %u bits\n", i, bits);
        } else {
            std::printf("        %2u. 0x%04X:%02X  %2u bits\n", i, obj, sub, bits);
        }
    }
}

void usage(const char* argv0) {
    std::printf(
        "bus_scan — enumerate an EtherCAT segment\n"
        "\n"
        "usage: %s <interface> [--pdo]\n"
        "\n"
        "  --pdo    also read back PDO mappings over SDO (needs PRE-OP)\n"
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
    const std::string ifname = argv[1];
    bool want_pdo = false;
    for (int i = 2; i < argc; i++) {
        if (std::strcmp(argv[i], "--pdo") == 0) {
            want_pdo = true;
        } else {
            usage(argv[0]);
            return 2;
        }
    }

    std::printf("=== EtherCAT bus scan: %s ===\n\n", ifname.c_str());
    std::fflush(stdout);  // keep ordering sane when stderr is redirected separately

    // 1. Open the NIC. Nonzero means success.
    if (ecx_init(&g_ctx, ifname.c_str()) <= 0) {
        std::fprintf(stderr,
                     "FAILED to open '%s'.\n"
                     "  - run as root (raw AF_PACKET socket)\n"
                     "  - check the interface name: ip -br link\n"
                     "  - the interface must be UP\n",
                     ifname.c_str());
        return 1;
    }
    std::printf("NIC opened OK — socket, permissions and driver path all work.\n\n");

    // 2. Enumerate.
    const int found = ecx_config_init(&g_ctx);
    if (found <= 0) {
        std::printf("No slaves found.\n\n");
        std::printf("  That is expected with nothing connected, and this run still\n");
        std::printf("  proved the software path. With hardware attached, check:\n");
        std::printf("    - slave powered\n");
        std::printf("    - cable in the slave's IN port (EtherCAT is directional)\n");
        std::printf("    - link light on the NIC\n");
        ecx_close(&g_ctx);
        return 0;
    }

    std::printf("Found %d slave%s\n", found, found == 1 ? "" : "s");
    std::printf("Lowest state: %s\n\n", state_name(ecx_readstate(&g_ctx)));

    const int iomap_size = ecx_config_map_group(&g_ctx, g_iomap, 0);
    const boolean has_dc = ecx_configdc(&g_ctx);

    std::printf("IOmap size    : %d bytes\n", iomap_size);
    std::printf("DC capable    : %s\n", has_dc ? "yes" : "no");
    std::printf("Group blockLRW: %u  (non-zero => LRD/LWR instead of LRW)\n\n",
                g_ctx.grouplist[0].blockLRW);

    for (int i = 1; i <= g_ctx.slavecount; i++) {
        const ec_slavet& s = g_ctx.slavelist[i];

        std::printf("--- Slave %d: %s\n", i, s.name);
        std::printf("    Vendor ID    : 0x%08" PRIX32 "\n", s.eep_man);
        std::printf("    Product code : 0x%08" PRIX32 "\n", s.eep_id);
        std::printf("    Revision     : 0x%08" PRIX32 "\n", s.eep_rev);
        std::printf("    State        : %s\n", state_name(s.state));

        std::printf("    Mailbox      : ");
        print_mailbox_protocols(s.mbx_proto);
        std::printf("   (CoEdetails=0x%02X SoEdetails=0x%02X)\n", s.CoEdetails, s.SoEdetails);

        // The profile question, settled by the device itself.
        if (s.mbx_proto & ECT_MBXPROT_COE) {
            std::printf("    Profile      : CoE — CiA 402 applies\n");
        } else if (s.mbx_proto & ECT_MBXPROT_SOE) {
            std::printf("    Profile      : SoE — CiA 402 does NOT apply\n");
        }

        std::printf("    Process data : %u bits out (%u bytes), %u bits in (%u bytes)\n",
                    s.Obits, s.Obytes, s.Ibits, s.Ibytes);
        if (s.Obits > 0 && s.Obits < 8) {
            std::printf("                   NOTE: Obits < 8 so Obytes is 0 — use Ostartbit\n");
        }

        std::printf("    IOmap offset : out %u, in %u\n", s.Ooffset, s.Ioffset);
        std::printf("    hasdc        : %s\n", s.hasdc ? "yes" : "no");

        // Does the device itself ask for LRW to be avoided? Yaskawa Sigma-7
        // does not support LRW; Sigma-X does. See docs/06-vendor-notes.md.
        std::printf("    blockLRW     : %u%s\n", s.blockLRW,
                    s.blockLRW ? "  (device requests LRD/LWR)" : "");

        if (want_pdo && (s.mbx_proto & ECT_MBXPROT_COE)) {
            std::printf("    PDO mapping (read back over SDO):\n");
            for (uint16_t idx = 0x1600; idx <= 0x1603; idx++) {
                dump_pdo_mapping(static_cast<uint16_t>(i), idx);
            }
            for (uint16_t idx = 0x1A00; idx <= 0x1A03; idx++) {
                dump_pdo_mapping(static_cast<uint16_t>(i), idx);
            }
        }

        std::printf("\n");
    }

    if (g_ctx.ecaterror) {
        std::printf("=== EtherCAT errors ===\n");
        while (ecx_iserror(&g_ctx)) {
            std::printf("  %s", ecx_elist2string(&g_ctx));
        }
        std::printf("\n");
    }

    ecx_close(&g_ctx);
    std::printf("Done.\n");
    return 0;
}
