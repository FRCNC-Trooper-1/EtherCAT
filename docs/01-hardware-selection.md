# 01 — Hardware Selection

> **Goal of this stage:** a bill of materials you can buy repeatedly, qualify
> once, and ship for years.
>
> **Exit criterion:** one qualified controller board model, one qualified NIC,
> one qualified drive family, all validated against
> [`02-pc-realtime-setup.md §7`](02-pc-realtime-setup.md#7-acceptance-criteria).

For a one-off machine you buy what is on the shelf. For a **product**, every
component choice is a ten-year commitment: spares, revision changes, firmware
regressions, lead times, and end-of-life notices. Choose accordingly, and write
down *why* you chose each part — your future self will need the reasoning when a
part goes EOL.

---

## 1. Network interface — the single most important choice

SOEM sends raw Ethernet frames through an `AF_PACKET` socket. There is no
EtherCAT-specific driver involved, which means **the NIC and its Linux driver
are directly in your jitter path.**

### Recommended

| Controller | Driver | Notes |
|---|---|---|
| **Intel i210 / i211** | `igb` | The default choice. Cheap, ubiquitous, excellent Linux driver, predictable latency. i210 has more offload options; i211 is the cost-reduced version and is entirely sufficient. |
| **Intel i350** | `igb` | Multi-port version of the same silicon. Use when you want EtherCAT and LAN on one card from a known-good family. |
| **Intel I219-LM / I225 / I226** | `e1000e` / `igc` | Common on modern industrial boards. Workable, but I225/I226 had well-documented early-stepping errata — verify the stepping and qualify carefully. |

### Avoid

- **Realtek RTL8111/8168** (`r8169`) — present on most consumer boards. It works,
  but latency is measurably worse and the driver has historically had
  power-management quirks that produce sporadic multi-hundred-microsecond stalls.
  Fine for a bench prototype, not for a product.
- **USB Ethernet adapters** — unusable. USB adds unbounded latency by design.
- **Any NIC behind a PCIe switch with aggressive ASPM** — verify ASPM is off.

### Rules

1. **Dedicate the NIC.** The EtherCAT port carries no IP traffic, has no address,
   and is unmanaged by NetworkManager. One port for the fieldbus, a separate
   port for the plant network and remote support.
2. **Prefer a discrete PCIe card over an onboard controller** where you can — it
   decouples your NIC qualification from motherboard revisions.
3. **Qualify the exact part.** "Intel gigabit" is not a specification. Record
   the PCI vendor:device ID (`lspci -nn`) in your BOM, because vendors silently
   change controllers between board revisions.

```bash
lspci -nn | grep -i ethernet          # record the [8086:1533]-style ID
ethtool -i enp3s0                     # record driver and firmware version
```

---

## 2. Control PC

### Requirements

| Aspect | Requirement | Why |
|---|---|---|
| CPU | 4+ physical cores, x86-64 | Two cores isolated for control, two for OS/HMI |
| Hyper-Threading | Must be **disableable** in BIOS | See [`02 §3`](02-pc-realtime-setup.md#3-biosuefi--do-this-before-touching-software) |
| C-states / SpeedStep | Must be **disableable** in BIOS | Non-negotiable for RT |
| RAM | 8 GB minimum, ECC preferred | ECC surfaces memory faults before they corrupt a toolpath |
| Storage | Industrial SATA/NVMe SSD with power-loss protection | Consumer SSDs corrupt on the abrupt power cuts that machine tools experience routinely |
| Form factor | Industrial mini-ITX, DIN-rail box PC, or embedded | Fanless preferred — shop air is full of coolant mist and aluminium chips |
| Temperature | 0–50 °C operating, verified | Panel interiors run hot |
| Power | 24 VDC input | Matches the machine's control voltage; avoids an extra AC supply in the panel |

### CPU clock speed vs. core count

Favour **single-thread performance** over core count. The cyclic task is one
thread on one core, and its deadline is what constrains cycle time. A 4-core
part at 3.5 GHz beats a 16-core part at 2.0 GHz for this workload.

### What to avoid

- Consumer motherboards with locked-down BIOS (no C-state control)
- Anything where the vendor will not commit to a multi-year supply
- Boards you cannot get an SMI-clean `hwlatdetect` result from — **test before
  committing to the BOM**, not after

> **Qualify at least two board models.** Single-sourcing your controller is how a
> product line stops shipping when one vendor has a supply problem.

---

## 3. Servo drives

For a 3-axis machine you need drives that speak **EtherCAT CoE with the CiA 402
profile in Cyclic Synchronous Position (CSP) mode** and support **Distributed
Clocks**. All three are required. A drive that does CSP but not DC cannot be
coordinated with other axes for contouring.

### Non-negotiable checklist

Before buying any drive, confirm from the manufacturer's manual and its ESI file:

- [ ] CiA 402 profile, **CSP mode (0x6060 = 8)** supported
- [ ] **Distributed Clocks** with DC Sync0 supported — not just "EtherCAT support"
- [ ] Minimum supported cycle time meets your target (want ≤ 500 µs headroom)
- [ ] **STO (Safe Torque Off)** inputs, rated to at least SIL 2 / PLd
- [ ] ESI (EtherCAT Slave Information) XML file publicly available
- [ ] Following-error window (0x6065) and software position limits (0x607D) supported
- [ ] Documented PDO mapping, ideally configurable
- [ ] Encoder feedback resolution adequate for your mechanics
- [ ] Multi-year availability commitment

### Candidate families

| Family | Notes |
|---|---|
| **Delta ASDA-A3 / A2-E** | Strong price/performance, widely available in North America, good CoE documentation, common in this class of machine |
| **Leadshine EL7-EC / EL8-EC** | Low cost, well-supported in the open-source CNC community, adequate for lighter machines |
| **Omron 1S** | Excellent documentation, strong safety integration, higher cost |
| **Beckhoff AX5000 / AX8000** | Reference-quality EtherCAT implementation (Beckhoff invented the protocol), premium cost |
| **Yaskawa Sigma-7** | Excellent hardware; note it commonly uses the **SoE** profile rather than CoE — verify which your part number speaks before designing around CiA 402 |

> **Buy one drive and one motor before designing anything.** Bring it up on the
> bench, read its ESI, exercise its state machine, and measure its behaviour at
> your target cycle time. Vendor datasheets routinely overstate EtherCAT
> conformance. This single purchase de-risks the largest unknown in the project.

---

## 4. Bus topology and I/O

EtherCAT is a **logical ring built on a physical line**. Frames pass through each
slave, get processed on the fly, and return from the last device.

```
  Control PC
      │
      │  dedicated NIC, 100BASE-TX
      ▼
  ┌────────┐   ┌────────┐   ┌────────┐   ┌────────┐
  │ X axis │──▶│ Y axis │──▶│ Z axis │──▶│  I/O   │
  │ drive  │   │ drive  │   │ drive  │   │ module │
  └────────┘   └────────┘   └────────┘   └────────┘
```

### Rules

- **Order matters for diagnostics, not for function.** Slaves are addressed by
  physical position, so a device inserted mid-bus renumbers everything after it.
  Fix your topology early and use it consistently across every machine you build —
  a technician's mental model should transfer between units.
- **Put I/O last.** Drives are latency-sensitive; digital I/O is not.
- **Use industrial cable.** Shielded Cat5e, EtherCAT-rated, with proper strain
  relief. Not office patch cable.
- **Keep segments short.** 100 m is the spec limit; in a machine you should be
  well under 10 m.
- **Cable routing:** never parallel to motor power leads. Cross at 90°. Servo
  cables radiate enough to corrupt frames.

### I/O modules

Standard EtherCAT digital/analog I/O handles limits, home switches, probe input,
spindle enable, coolant, and so on. Beckhoff EL-series is the reference; Omron,
Delta, and Weintek offer cheaper equivalents.

**Latency-critical inputs are the exception.** A touch probe or a spindle
sync/index pulse needs microsecond capture, not cycle-rate polling. Use a drive's
**touch-probe / latch input** (CiA 402 objects 0x60B8–0x60BB) for these, which
timestamps in hardware, rather than a general-purpose I/O module read once per
cycle.

---

## 5. Safety hardware — read this before you buy anything

**Machine safety must not depend on Linux, on your application, or on EtherCAT.**

A 3-axis machine with servo drives can kill someone. Linux with PREEMPT_RT is a
real-time operating system; it is not a **safety-rated** one. Your BSD-licensed
control application is not safety-rated either, and no amount of careful coding
makes it so.

Safety functions must be implemented in **certified hardware** on an independent
channel:

| Function | Implementation |
|---|---|
| Emergency stop | Certified safety relay or safety controller (Pilz PNOZ, Sick Flexi Soft, Banner, Omron G9SP) — **hardwired**, not software |
| Safe Torque Off | Drive STO inputs, driven directly by the safety relay |
| Guard/door interlock | Safety-rated interlock switch into the safety relay |
| Hard limit switches | Wired to the safety circuit **and** read by the control for diagnostics |
| Contactor drop-out | Safety relay drops main power contactor |

The control PC may **observe** safety state (to display it and to stop motion
gracefully), but it must never be the thing that **enforces** it.

Target **ISO 13849-1 Category 3, Performance Level d (PLd)** for a machine tool
of this class. If you are also evaluating **FSoE (Safety over EtherCAT)**, note
that it requires certified safety slaves and a certified safety controller — it
does not let your application become the safety system.

Details and the US-specific regulatory picture are in
[`07-safety-and-compliance.md`](07-safety-and-compliance.md). **Read it before
finalizing the electrical design**, not after.

---

## 6. Reference BOM template

Fill this in and keep it under version control. Every field matters when a part
goes EOL three years from now.

| Item | Part number | PCI/Vendor ID | Qty | Qualified on | Notes |
|---|---|---|---|---|---|
| Control PC | | | 1 | | `hwlatdetect` clean, cyclictest max ___ µs |
| EtherCAT NIC | | `[8086:____]` | 1 | | driver ___ , firmware ___ |
| LAN NIC | | | 1 | | plant network / remote support |
| Servo drive X | | | 1 | | CSP + DC verified, min cycle ___ µs |
| Servo drive Y | | | 1 | | |
| Servo drive Z | | | 1 | | |
| Servo motors | | | 3 | | encoder ___ counts/rev |
| EtherCAT I/O | | | 1 | | ___ DI / ___ DO / ___ AI |
| Safety relay | | | 1 | | ISO 13849-1 Cat 3 PLd |
| Cabling | | | | | shielded Cat5e, EtherCAT rated |

---

## 7. Before you spend real money

The cheapest possible de-risking sequence, in order:

1. Buy **one drive + one motor** and **one candidate control PC**.
2. Run `hwlatdetect` and 12 h `cyclictest` on the PC. If it fails, return it and
   try another board. **Do this first** — it is a two-day test that can invalidate
   your entire hardware direction.
3. Bring the drive up on the bench with SOEM's `slaveinfo` sample. Confirm it
   reaches `OPERATIONAL`, confirm DC works, confirm CSP.
4. Spin the motor under CSP at your target cycle time for 24 hours. Zero working
   counter errors.
5. **Only then** buy the remaining axes and commit to the BOM.

Steps 1–4 cost a few thousand dollars and a couple of weeks. Skipping them
routinely costs projects a year.

---

**Next:** [`02-pc-realtime-setup.md`](02-pc-realtime-setup.md) — tuning the
control PC.
