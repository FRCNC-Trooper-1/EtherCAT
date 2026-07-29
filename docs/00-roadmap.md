# 00 — Project Roadmap

**An industrial 3-axis CNC controller: EtherCAT master on real-time Linux.**

This is the master plan. Every phase has a **goal**, a set of **steps**, and an
**exit criterion** — a specific, measurable thing that must be true before moving
on. Do not skip exit criteria. Every one of them exists because skipping it is a
known way to lose months.

---

## The shape of the project

```
Phase 0  Foundations          paperwork, licensing, hardware decisions
Phase 1  Control PC           real-time kernel, tuning, latency proof
Phase 2  Bus bring-up         SOEM builds, slaves enumerate, bus reaches OP
Phase 3  One axis moves       CiA 402 CSP, a motor turns under command
Phase 4  Three axes sync      Distributed Clocks, coordinated motion
Phase 5  Interpolation        linear + circular, look-ahead, jerk limiting
Phase 6  G-code               parser, queue, program execution
Phase 7  Productization       HMI, config, safety integration, install
Phase 8  Qualification        endurance, accuracy, compliance, first article
```

Realistic timeline for a small team: **12–18 months to a first customer machine.**
Phases 1–4 are the technically risky part and should take about a third of that.
Phases 5–6 are substantial but well-understood engineering. Phase 7–8 always take
longer than anyone plans.

---

## Phase 0 — Foundations

**Goal:** remove the decisions that would force rework later.

Nothing here requires hardware, and most of it runs in parallel with Phase 1.

### Steps

1. **Join the EtherCAT Technology Group** ([ethercat.org](https://www.ethercat.org)).
   Membership is free. Begin the **EtherCAT Master License** process with
   Beckhoff — required to sell any product containing an EtherCAT master,
   regardless of which stack you use.
2. **Request licensing quotes.** Email `sales@rt-labs.com` for a SOEM commercial
   license and acontis for EC-Master. You are not committing; you are turning an
   open question into a number. See [`08-licensing.md`](08-licensing.md).
3. **Decide the machine specification.** Travels, rapid feed, cutting feed,
   spindle, accuracy target, tool changer. Every later decision depends on these.
4. **Start the risk assessment** (ISO 12100 / ANSI B11.0). It constrains the
   electrical design, so it must precede it. See
   [`07-safety-and-compliance.md`](07-safety-and-compliance.md).
5. **Buy one drive, one motor, one candidate control PC.** Not three. One.

### Exit criteria

- [ ] ETG membership active; Master License process started
- [ ] Licensing quotes received and compared
- [ ] Machine specification written down and agreed
- [ ] Risk assessment started, required Performance Levels identified
- [ ] Evaluation hardware on the bench

---

## Phase 1 — Control PC

**Goal:** a machine that wakes a thread on time, every time, for weeks.

Full detail in [`02-pc-realtime-setup.md`](02-pc-realtime-setup.md). This is the
foundation; every later phase inherits its timing.

### Steps

1. Install Ubuntu 24.04 LTS (or your chosen base).
2. Build and install a mainline kernel with `CONFIG_PREEMPT_RT=y`. **PREEMPT_RT
   has been merged into mainline since Linux 6.12** — you no longer need
   out-of-tree patches or a vendor RT subscription.
3. Configure BIOS: disable HT, C-states, SpeedStep, Turbo, ASPM, USB legacy.
4. Set kernel command line: `isolcpus`, `nohz_full`, `rcu_nocbs`, `irqaffinity`.
5. Disable `irqbalance`; pin NIC IRQs deliberately.
6. Tune the NIC: coalescing off, offloads off, small rings, no flow control,
   forced 100 Mbit full duplex.
7. **Measure.** `hwlatdetect` for 30 minutes. `cyclictest` for 12 hours under load.

### Exit criteria

- [ ] `cat /sys/kernel/realtime` prints `1`
- [ ] `hwlatdetect --duration=30m` reports **zero** samples over threshold
- [ ] `cyclictest` 12 h under load: **max** latency under budget (< 100 µs for a
      1 ms cycle)
- [ ] Results recorded in `docs/hardware-qualification/` for this board model

> If the board fails `hwlatdetect`, **replace the board**. This is a hardware
> problem and no amount of software tuning will fix it. Better to learn this in
> week two than after committing to a BOM.

---

## Phase 2 — Bus bring-up

**Goal:** the EtherCAT bus enumerates and reaches `OPERATIONAL`.

Full detail in [`03-ethercat-bringup.md`](03-ethercat-bringup.md).

### Steps

1. Add SOEM as a git submodule under `extern/soem`. Build it via CMake.
2. Run the `slaveinfo` sample against your dedicated NIC. Confirm the drive is
   discovered, and record its vendor ID, product code, and revision.
3. **Read the drive's ESI (EtherCAT Slave Information) XML.** Learn its actual
   default PDO mapping — do not assume it.
4. Walk the state machine: `INIT` → `PRE-OP` → `SAFE-OP` → `OP`.
5. Verify the working counter matches the expected value every cycle.
6. Run a cyclic exchange at 1 ms for 24 hours with the drive disabled. Zero
   working counter errors.

### Exit criteria

- [ ] All slaves enumerate with correct identity
- [ ] Bus reaches `OPERATIONAL` reliably from a cold start
- [ ] 24 h cyclic run at 1 ms with **zero** working counter errors
- [ ] `ecx_configdc()` reports DC-capable slaves

---

## Phase 3 — One axis moves

**Goal:** a motor turns, under CSP, commanded by your software.

Full detail in [`04-drive-cia402.md`](04-drive-cia402.md).

This is the first phase where something can hurt someone. **Uncouple the motor
from the machine.** Bolt it to the bench, no load, nothing attached to the shaft.

### Steps

1. Implement the **CiA 402 state machine**: decode Statusword `0x6041`, drive
   Controlword `0x6040` through `Shutdown (0x0006)` → `Switch On (0x0007)` →
   `Enable Operation (0x000F)`.
2. Configure PDO mapping via SDO in `PRE-OP` — clear, populate, restore counts,
   re-assign `0x1C12`/`0x1C13`.
3. Set `0x6060 = 8` (CSP) and confirm `0x6061` echoes `8`.
4. Set `0x60C2` (interpolation time period) to match your cycle exactly.
5. Set following error window `0x6065` and software limits `0x607D`.
6. Command `0x607A` (target position) with a slow sine or a jog ramp. **Always
   command a position derived from the current actual position on enable** — a
   step change in `0x607A` at enable will slam the motor.
7. Verify `0x6064` (actual position) tracks the command.

The tool for this is `axis_jog`. It brings the bus up, enables the axes, and
moves one of them a measured distance under a jerk-limited profile, then reports
what actually happened:

```
sudo ./build/axis_jog enp3s0 --counts-per-mm 10000 --distance 1 --feed 60
```

It defaults to 1 mm at 60 mm/min and asks for confirmation before moving. Do the
first run with the motor **off the machine** — a wrong `--counts-per-mm` is a
factor-of-anything error in how far the axis travels, and the first place that
shows up is at the end of the travel.

Read three numbers from its output:

- **measured vs commanded distance** — disagreement means scaling is wrong, and
  the ratio tells you by how much.
- **starve events** — non-zero means this PC could not feed setpoints at the
  requested cycle time. The axis stopped on the path, which is the designed
  response, but the cycle time is not usable.
- **max jitter** — should match what `rt_probe` measured. If it is worse, the
  bus traffic is costing more than the loop budget allows.

### Exit criteria

- [ ] Drive reaches `Operation Enabled` reliably and returns to a safe state on exit
- [ ] Motor follows a commanded profile smoothly at 1 ms
- [ ] Following error stays within budget across the speed range
- [ ] Faults are detected, reported, and recoverable via `Fault Reset (0x0080)`
- [ ] Killing the control process leaves the drive **disabled**, never free-running

---

## Phase 4 — Three axes synchronized

**Goal:** three axes apply their setpoints at the same instant.

This is where Distributed Clocks stop being optional.

### Why DC is mandatory

In CSP the master pushes a new target position every cycle and the drive
interpolates between setpoints. Without DC, each drive applies its setpoint
whenever its frame happens to be processed. That inter-axis skew becomes a
**position error proportional to feedrate**:

> At 10 m/min (167 mm/s), **100 µs of skew = 16.7 µm of path error.**

It varies with feed, so it appears as corner rounding and out-of-round arcs — the
symptoms people spend months misattributing to mechanics. Many drives also simply
refuse to leave `SAFE-OP` without Sync0.

### The three periods must be equal

```
master cycle time  ==  Sync0 period  ==  0x60C2 interpolation time period
```

For a 1 ms cycle: master = 1 000 000 ns, Sync0 = 1 000 000 ns,
`0x60C2:01 = 1`, `0x60C2:02 = -3`.

> **Check `0x60C2` writability on every drive model before choosing your cycle
> time.** On some drives it is read-only and fixed (2 ms is a common value). If
> any drive in the chain has a fixed period, **the whole machine is pinned to it.**
> This has forced more than one retrofit to redesign its real-time loop.

### Steps

1. Call `ecx_configdc()` after mapping; confirm slaves report DC capability.
2. Call `ecx_dcsync0()` per drive with your cycle time and an appropriate shift
   (roughly 20–50% of cycle, past worst-case propagation to the last slave).
3. Implement the **DC drift PI controller** — adjust your sleep so the master
   cycle tracks the reference clock. SOEM's `ec_sample` sample is the reference.
4. Verify all three axes latch setpoints simultaneously.
5. Command a coordinated 3-axis linear move and measure straightness.

### Exit criteria

- [ ] All axes DC-synchronized; drift PI controller stable over 24 h
- [ ] Sync0 jitter measured and within budget
- [ ] Coordinated linear move produces a straight line at all feedrates
- [ ] A fault on any axis stops **all** axes together, on path

---

## Phase 5 — Interpolation

**Goal:** real toolpaths — lines, arcs, blended corners, jerk-limited.

Full detail in [`05-motion-architecture.md`](05-motion-architecture.md).

### Steps

1. **Kinematics**: user units (mm) ↔ drive units (counts), per axis. Decide
   deliberately whether scaling lives in the drive (`0x6091`/`0x6092`/`0x608F`) or
   in the master. **For mixed-vendor machines, do it in the master** — vendors
   differ on whether those objects even apply to `0x607A`/`0x6064`.
2. **Trapezoidal velocity profile** first — get it correct before making it smooth.
3. **S-curve (jerk-limited) profile** — bounded jerk protects the mechanics and
   dramatically improves surface finish.
4. **Linear interpolation** across 3 axes.
5. **Circular interpolation** in each plane (G17/G18/G19).
6. **Look-ahead**: a segment queue deep enough (typically 100–1000 segments) to
   plan cornering velocity without stopping at every block boundary.
7. **Corner blending**: constrain junction velocity by allowable deviation.

### Exit criteria

- [ ] Straightness, circularity, and squareness measured against spec
- [ ] Ballbar or equivalent circular test passes
- [ ] No velocity discontinuity at segment boundaries
- [ ] Look-ahead keeps the pipeline full at maximum feed without starving

---

## Phase 6 — G-code

**Goal:** run a real part program.

### Steps

1. G-code parser: motion (G0/G1/G2/G3), plane and units, feed and speed, tool and
   work offsets (G54–G59), canned cycles as needed.
2. Program queue feeding the look-ahead planner.
3. Modal state, program flow, subroutines.
4. Jog, MDI, single-block, feed hold, feed override, and — carefully — resume.
5. Tool length and cutter compensation.
6. Homing sequence (drive-internal HM mode is strongly preferred over master-side;
   the drive sees the index pulse at encoder resolution, not bus-cycle resolution).

### Exit criteria

- [ ] A representative part program runs start to finish
- [ ] Feed hold and resume are safe and repeatable mid-program
- [ ] Coordinate systems and offsets behave correctly
- [ ] Cutting a test part produces dimensionally correct results

---

## Phase 7 — Productization

**Goal:** something a customer can own, and a technician can service.

### Steps

1. **HMI** — operator interface, separate process, never in the RT path.
2. **Machine configuration** — declarative files for axes, drives, I/O, limits.
   No recompiling to change a machine parameter.
3. **Safety integration** — certified safety relay, drive STO, guard interlocks.
   Wire the control channel to *observe* safety state; never to *enforce* it.
4. **Diagnostics** — following error trends, drive temperatures, bus error
   counters, cycle-time histograms. A technician needs these to service a machine.
5. **Update mechanism** — signed, atomic, with rollback.
6. **Machine image** — pinned kernel, pinned software, reproducible build. **Do not
   let customer machines take distro kernel updates**; an unattended upgrade
   silently reverts every real-time guarantee.
7. **Manuals** — operator and maintenance, with residual-risk warnings.

### Exit criteria

- [ ] Machine configurable without recompiling
- [ ] Safety circuit built, tested, validated against required PL
- [ ] Update tested including rollback from a failed update
- [ ] Reproducible build from a clean checkout
- [ ] Documentation complete

---

## Phase 8 — Qualification

**Goal:** evidence that it works, and permission to sell it.

### Steps

1. **Endurance**: 30 days continuous, zero bus errors, zero missed cycles.
2. **Accuracy**: laser interferometer positioning, ballbar circularity, ISO 230-2
   repeatability.
3. **Environmental**: full temperature range, power interruption, EMC — a VFD
   spindle starting is a real test of your cable routing.
4. **Safety validation**: every safety function exercised and documented.
5. **Compliance**: NFPA 79, UL 508A panel listing or NRTL field evaluation, risk
   assessment finalized. CE technical file if exporting to the EU.
6. **First article**: build one on the production process, with production
   documentation, and record exactly what shipped.

### Exit criteria

- [ ] 30-day endurance passed
- [ ] Accuracy meets published specification
- [ ] Safety validation documented
- [ ] Regulatory compliance complete
- [ ] Traceability: serial number → software, kernel, firmware, configuration

---

## Cross-cutting practices

Adopt from day one. Retrofitting any of these is painful.

| Practice | Why |
|---|---|
| **Everything in git** — code, configs, schematics, risk assessment, test results | When a customer reports a fault on an 18-month-old machine, you must reconstruct exactly what shipped |
| **Measure before optimizing** | Every timing claim needs a number behind it |
| **No allocation in the RT path** | One `malloc` in the cyclic loop is one missed cycle |
| **Fail safe, always** | Every fault path ends with drives disabled |
| **License check on every dependency** | One convenience library can compromise the whole product's licensing |
| **Hardware qualification records per board model** | `hwlatdetect` and `cyclictest` results, per model, versioned |

---

## The critical path

If you only track four things:

1. **Does the control PC meet latency?** (Phase 1) — if not, nothing else matters.
2. **Does the drive do CSP + DC at your cycle time?** (Phases 2–4) — verify with
   one drive before buying three.
3. **Is `0x60C2` writable on your drives?** (Phase 4) — it can pin your entire
   machine's cycle time.
4. **Is the EtherCAT stack licensing resolved?** (Phase 0/8) — does not block
   development, absolutely blocks shipping.

---

## Where to start today

```bash
# 1. Read these, in order:
docs/08-licensing.md            # the constraint that shapes everything
docs/01-hardware-selection.md   # what to buy
docs/02-pc-realtime-setup.md    # how to tune it

# 2. On your real-time machine:
sudo ./scripts/install-deps.sh
sudo ./scripts/check-realtime.sh    # tells you what still needs doing

# 3. Then Phase 1, and do not move past its exit criteria.
```
