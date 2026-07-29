# Industrial EtherCAT CNC Controller

A real-time EtherCAT master and motion controller for industrial 3-axis (and
higher) CNC machines, running on Linux with `PREEMPT_RT`.

**Status:** Phase 0 — foundations. Documentation and real-time validation tooling
are in place; fieldbus and motion layers are not yet implemented.

---

## What this is

A commercial machine controller built on:

- **Linux + `PREEMPT_RT`** — mainline since kernel 6.12, no out-of-tree patches
- **EtherCAT** over a dedicated NIC, userspace raw sockets
- **CiA 402 CSP mode** with Distributed Clocks for coordinated multi-axis motion
- **BSD-3-Clause** for our own code

Target: 1 ms cycle initially, 500 µs and below once the timing baseline is proven.

---

## Read this first

⚠️ **The EtherCAT stack licensing is not settled, and it constrains the product.**

SOEM — the obvious open-source EtherCAT master — is **not** permissively licensed,
despite what much of the internet says:

- **SOEM v2** is **GPLv3 or commercial**. No linking exception.
- **SOEM v1.4.0** is GPLv2 **with** a linking exception that does permit
  proprietary linking.
- **Both** require an **EtherCAT Master License from Beckhoff**, which applies no
  matter which stack you use — including one you write yourself.

**This does not block development.** GPL obligations trigger on *distribution*,
not on use. Build, test, and iterate on current SOEM v2 freely; the decision must
be made before the first machine ships.

Full analysis, with license text quoted verbatim and options ranked:
**[`docs/08-licensing.md`](docs/08-licensing.md)**.

---

## Documentation

Read in order. Each builds on the previous.

| Doc | What it covers |
|---|---|
| [`00-roadmap.md`](docs/00-roadmap.md) | **Start here.** Phase-by-phase plan with exit criteria |
| [`01-hardware-selection.md`](docs/01-hardware-selection.md) | NIC, control PC, drives, topology, safety hardware |
| [`02-pc-realtime-setup.md`](docs/02-pc-realtime-setup.md) | RT kernel, BIOS, isolation, tuning, latency validation |
| [`03-ethercat-bringup.md`](docs/03-ethercat-bringup.md) | SOEM v2 API, bus enumeration, DC sync, diagnostics |
| [`04-drive-cia402.md`](docs/04-drive-cia402.md) | Drive state machine, CSP mode, PDO mapping, homing |
| [`05-motion-architecture.md`](docs/05-motion-architecture.md) | RT/non-RT separation, trajectory generation, fault handling |
| [`07-safety-and-compliance.md`](docs/07-safety-and-compliance.md) | Safety architecture, NFPA 79, UL 508A, CE, cybersecurity |
| [`08-licensing.md`](docs/08-licensing.md) | Licensing analysis and the decision that must be made |

---

## Quick start

On the **real control PC** — not a VM, not a laptop:

```bash
# 1. Install tooling
sudo ./scripts/install-deps.sh

# 2. Audit what still needs doing (read-only, changes nothing)
./scripts/check-realtime.sh <ethercat-interface>

# 3. Apply runtime tuning (does not cover BIOS or kernel cmdline)
sudo ./scripts/setup-realtime.sh <ethercat-interface> 2

# 4. Build (add -DFRCNC_WITH_SOEM=ON for the fieldbus tools)
git submodule update --init --recursive
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo -DFRCNC_WITH_SOEM=ON
cmake --build build
ctest --test-dir build --output-on-failure

# 5. Validate the timing foundation
sudo ./build/rt_probe --cycle 1000 --cpu 2 --duration 60
sudo ./scripts/hwlat.sh 600 10

# 6. Scan the EtherCAT segment (works with zero slaves connected)
sudo ./build/bus_scan <ethercat-interface>
sudo ./build/bus_scan <ethercat-interface> --pdo
```

`rt_probe` runs the same `CycleTimer`, memory locking, and scheduling the real
controller will use. If it passes, the timing foundation is sound. Run it for
12 hours under load before signing off Phase 1.

---

## Repository layout

```
docs/           the plan and the reference material
scripts/        RT setup, audit, dependency install
include/frcnc/  public headers
src/
  rt/           RT thread setup, cycle timing        (no dependencies)
  tools/        rt_probe and other diagnostics
  fieldbus/     SOEM wrapper, DC sync                (not yet implemented)
  drive/        CiA 402 state machine                (not yet implemented)
  motion/       trajectory, interpolation, look-ahead (not yet implemented)
  ipc/          lock-free rings to the non-RT side   (not yet implemented)
tests/          unit tests, no external dependencies
extern/soem/    SOEM submodule                       (not yet added)
```

---

## Build options

| Option | Default | Meaning |
|---|---|---|
| `FRCNC_BUILD_TESTS` | `ON` | Build and register unit tests |
| `FRCNC_WITH_SOEM` | `OFF` | Build components requiring SOEM (needs the submodule) |

SOEM v2 requires CMake ≥ 3.28, which is the project floor.

---

## Immediate next steps

Two are paperwork and block nothing; two are engineering and block everything.

1. **Join the EtherCAT Technology Group** and start the Beckhoff Master License
   process. Free, and required to ship.
2. **Request licensing quotes** — `sales@rt-labs.com` for SOEM, acontis for
   EC-Master. Turns an open question into a line item.
3. **Qualify a control PC.** Run `hwlatdetect` for 30 minutes and `rt_probe` for
   12 hours. If the board fails, replace the board — no software fixes an SMI.
4. **Buy one drive and one motor.** Verify CSP, Distributed Clocks, and whether
   `0x60C2` is writable *before* committing to a BOM. That single object can pin
   your entire machine's cycle time.

---

## Contributing

- Every new dependency needs a license check in the same pull request that adds
  it. See [`docs/08-licensing.md §8`](docs/08-licensing.md).
- The cyclic path has hard rules — no allocation, no locks, no exceptions. See
  [`docs/05-motion-architecture.md §2`](docs/05-motion-architecture.md).
- Format with `clang-format` before committing.

---

## License

BSD 3-Clause for this repository's own code. See [`LICENSE`](LICENSE).

Third-party components are **not** all permissively licensed — see
[`docs/08-licensing.md`](docs/08-licensing.md) before distributing anything.
