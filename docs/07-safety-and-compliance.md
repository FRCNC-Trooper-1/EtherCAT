# 07 — Safety and Regulatory Compliance

> **Read this before finalizing the electrical design.** Safety architecture
> constrains your wiring, your panel layout, and your drive selection. Retrofitting
> it after the prototype works is expensive and usually means rebuilding the panel.

This document covers what applies to a commercial industrial CNC machine sold in
the United States, with notes on export to the EU.

> **This is engineering guidance, not legal advice.** Before you ship a machine to
> a paying customer, engage (a) a qualified machine-safety engineer for the risk
> assessment and (b) your product liability insurer. Both will ask for the
> documentation described here.

---

## 1. The governing principle

**Safety must not depend on Linux, on your application, or on EtherCAT.**

Linux with `PREEMPT_RT` is a real-time operating system. It is not a
**safety-rated** one. There is no certification path that makes a general-purpose
kernel plus a BSD-licensed C++ application into a safety system, and no amount of
careful coding substitutes for one.

The consequence for your architecture:

```
        ┌──────────────────────────────────────────────┐
        │  CONTROL CHANNEL (this repository)           │
        │  Linux + PREEMPT_RT + your application       │
        │  Plans motion, commands drives, shows state  │
        │  NOT SAFETY RATED                            │
        └──────────────────────────────────────────────┘
                            │
                     observes only
                            │
        ┌───────────────────▼──────────────────────────┐
        │  SAFETY CHANNEL (certified hardware)         │
        │  Safety relay / safety controller            │
        │  E-stop, guard interlock, hard limits        │
        │  Drives drive-STO and the main contactor     │
        │  ISO 13849-1 Cat 3 / PLd                     │
        └──────────────────────────────────────────────┘
```

The control channel may **read** safety state to display it and to bring motion
down gracefully. It must never be the thing that **enforces** a stop.

If your answer to "what stops the machine if the software hangs?" is anything
that involves the software, the design is wrong.

---

## 2. Required safety functions

For a 3-axis machining centre, at minimum:

| Function | Implementation | Standard |
|---|---|---|
| **Emergency stop** | Hardwired E-stop buttons → certified safety relay → drive STO + main contactor | ISO 13850, NFPA 79 §10.7 |
| **Safe Torque Off** | Dual-channel STO inputs on every servo drive, driven by the safety relay | IEC 61800-5-2 |
| **Guard interlock** | Safety-rated interlock switch on enclosure doors → safety relay | ISO 14119 |
| **Hard overtravel limits** | Limit switches wired into the safety circuit **and** read by the control for diagnostics | NFPA 79 |
| **Power-loss behaviour** | Drives brake and hold; vertical axis brake engages on power loss | |
| **Restart interlock** | Machine must not restart on power restoration without deliberate operator action | ISO 13849 |

### Vertical axis brakes

A Z axis with a heavy head will drop when torque is removed. The mechanical
brake must be **fail-safe** — spring-applied, electrically released — so that
loss of power engages it. This is a wiring and drive-selection requirement, not
something the software can compensate for.

---

## 3. Risk assessment comes first

Before design, not after. **ISO 12100** is the process standard, and **ANSI
B11.0** is its US counterpart.

The sequence:

1. **Identify hazards** — crushing, shearing, entanglement, ejection of workpiece
   or tool, electrical, coolant mist, noise, stored energy.
2. **Estimate risk** for each — severity × frequency of exposure × possibility of
   avoidance.
3. **Reduce risk**, in this fixed order of preference:
   - Inherently safe design (eliminate the hazard)
   - Guarding and protective devices (enclose it)
   - Information for use (warn about it)
4. **Determine the required Performance Level (PLr)** for each safety function.
5. **Verify** the implemented Performance Level meets or exceeds PLr.
6. **Document all of it.** This file is the core of your technical file and the
   first thing an expert witness will ask for.

For a machine tool of this class, safety functions typically land at
**ISO 13849-1 Category 3, Performance Level d (PLd)**: dual-channel, monitored,
single-fault-tolerant.

Keep the risk assessment in this repository under `docs/safety/` and version it
alongside the code. It is a living document — it changes every time the machine
changes.

---

## 4. US regulatory requirements

### 4.1 Standards that apply

| Standard | Scope | Status |
|---|---|---|
| **NFPA 79** | Electrical Standard for Industrial Machinery | The central one for US machine builders. Governs wiring, disconnects, grounding, colour codes, E-stop circuits, documentation. |
| **UL 508A** | Industrial Control Panels | Panel construction. Building panels in a **UL 508A listed shop** with the appropriate label dramatically simplifies acceptance. |
| **ANSI B11.0** | Safety of Machinery — general | Risk assessment framework |
| **ANSI B11.8** | Drilling, milling, boring machines | Machine-type-specific requirements |
| **ANSI B11.19** | Risk reduction measures | Guarding and protective device performance |
| **ISO 13849-1** | Safety-related parts of control systems | How PL is calculated and verified |
| **OSHA 29 CFR 1910.212** | General machine guarding | Federal law; applies to your customer's workplace, and your machine must let them comply |
| **OSHA 29 CFR 1910.147** | Lockout/Tagout | Machine must be lockable in an energy-isolated state |

### 4.2 There is no "CE mark" for the US

The US has no single pre-market approval for industrial machinery. What you
actually need:

- **NFPA 79 compliance** — self-declared, but demonstrable
- **A UL 508A listed control panel**, or an **NRTL field evaluation** (UL, Intertek/ETL,
  TÜV) performed at the customer site. Field evaluations run a few thousand dollars
  per machine — building to 508A in a listed shop is far cheaper at volume.
- **Documented risk assessment** per ANSI B11.0
- **Operator and maintenance manuals** with residual-risk warnings
- **Product liability insurance** — your insurer will want the risk assessment

Many local Authorities Having Jurisdiction (electrical inspectors) will require an
NRTL mark on the control panel before allowing the machine to be energized at the
customer's facility. Plan for this; discovering it during installation is a bad day.

### 4.3 Exporting to the EU

Two regulations matter, both with deadlines that fall inside a realistic
development timeline for this project:

- **Machinery Regulation (EU) 2023/1230** replaces the Machinery Directive
  2006/42/EC and applies from **20 January 2027**. CE marking, technical file,
  Declaration of Conformity. Machine tools with safety functions may require
  third-party involvement depending on the final classification — check Annex I.
- **Cyber Resilience Act (EU) 2024/2847** — applies to products with digital
  elements. Vulnerability reporting obligations begin **September 2026**, full
  obligations **December 2027**. A networked CNC controller is squarely in scope.
  See §6.

If EU sale is plausible, design for both now. Retrofitting CE conformity is far
more expensive than building to it.

---

## 5. Software's actual safety role

Your control software is **not** a safety system, but it still has real
responsibilities. These belong in the cyclic task and are implemented in
`src/rt/` and `src/drive/`:

| Responsibility | Implementation |
|---|---|
| **Watchdog** | If the cyclic loop misses its deadline by more than N cycles, command a controlled stop and drop the enable signal |
| **Following-error monitoring** | CiA 402 object 0x6065; the drive faults on excess error, and the control must react to that fault by stopping every other axis |
| **Software position limits** | 0x607D in the drive, plus planner-side limits — these prevent *normal* overtravel; hard limits handle the abnormal case |
| **Working counter validation** | Every cycle. A WKC mismatch means a slave dropped out — treat as a fault and stop |
| **Slave state monitoring** | Any slave leaving `OPERATIONAL` is a fault condition |
| **Coordinated stop** | When one axis faults, all axes must stop together on their programmed path — an uncoordinated stop can break a tool or crash the machine |
| **Safe state on exit** | Any abnormal termination — crash, signal, exception — must leave drives disabled, not free-running |

**Fail-safe defaults:** every one of these must fail toward "stopped." A watchdog
that fails open is not a watchdog.

---

## 6. Cybersecurity and the software supply chain

A modern industrial controller is a networked computer, and regulators now treat
it as one.

### Software Bill of Materials

Generate and ship an **SBOM** (SPDX or CycloneDX) for every release. It is
already required for US federal procurement, is central to the EU CRA, and is
increasingly requested by large industrial customers.

CMake can emit one; wire it into CI so it is never out of date with the binary.

### Practical baseline

- No default passwords. Ever.
- The EtherCAT NIC has **no IP stack** — it is physically incapable of routing
  (this is already required for latency reasons; it is also a security property).
- Separate the plant-network interface, firewall it, and default to closed.
- Signed software updates with rollback.
- Documented process for receiving and acting on vulnerability reports —
  a `SECURITY.md` and a monitored contact address. The CRA makes this a legal
  obligation for EU sales.
- Log security-relevant events with enough retention to investigate an incident.

---

## 7. Open-source license compliance

You are shipping a **BSD-3-Clause** product that links third-party code. That is
straightforward, but it is not zero-obligation.

### SOEM — ⚠️ not permissively licensed

**SOEM v2 is dual-licensed GPLv3 or commercial. It does not permit proprietary
commercial distribution without buying a license.** SOEM v1.4.0 was GPLv2 *with a
linking exception* that does permit it. Both versions additionally require an
**EtherCAT Master License from Beckhoff**, which applies regardless of which stack
you use.

This is analysed in full — with the license text quoted verbatim and the options
ranked — in [`08-licensing.md`](08-licensing.md). **Read it before shipping.**

The short version: GPL obligations trigger on *distribution*, not development, so
this does not block building the machine. It must be resolved before unit #1
ships.

### What to do

1. Maintain `THIRD_PARTY_LICENSES.md`, generated from the actual dependency tree,
   and ship it with every machine (in the manual, in the UI's About box, or as a
   file on disk).
2. **Never** link GPL code into your application binary. Specifically:
   - LinuxCNC — GPLv2
   - IgH EtherCAT master kernel module — GPLv2
   - Anything under `libethercat`'s GPL components
3. The **Linux kernel is GPLv2, and that is fine.** The kernel's userspace API is
   explicitly carved out by the syscall exception. A normal userspace application
   calling `socket()`, `clock_nanosleep()`, and `sched_setscheduler()` is not a
   derivative work of the kernel. This is why the SOEM-plus-raw-sockets approach
   keeps your code proprietary while the IgH kernel module approach would not.
4. Audit dependencies before adding them. A single GPL library pulled in for
   convenience can compromise the licensing position of the whole product. Add a
   CI check that fails the build on a copyleft license appearing in the
   dependency tree.

See [`08-licensing.md`](08-licensing.md) for the full analysis.

---

## 8. Documentation you must produce and keep

Under version control, in this repository, updated with the code:

- [ ] Risk assessment (ISO 12100 / ANSI B11.0)
- [ ] Safety function specification, with PLr and achieved PL for each
- [ ] Electrical schematics, NFPA 79 compliant
- [ ] Panel layout and UL 508A documentation
- [ ] Validation and test records — including `cyclictest` / `hwlatdetect` results
      per hardware model
- [ ] Operator manual, with residual-risk warnings
- [ ] Maintenance manual, including LOTO procedures
- [ ] SBOM and third-party license notices
- [ ] Declaration of Conformity (if CE marking)
- [ ] Change log tying software versions to machine serial numbers

That last item matters more than it looks. When a customer reports a problem on a
machine built eighteen months ago, you must be able to reconstruct **exactly**
what software, kernel, drive firmware, and configuration that specific serial
number shipped with. Build this traceability in from the first machine — it is
nearly impossible to reconstruct later.

---

## 9. Sequencing

Do these in this order. Later items depend on earlier ones.

| When | Action |
|---|---|
| **Before electrical design** | Risk assessment; determine required PL for each safety function |
| **Before buying drives** | Confirm STO ratings meet the required PL |
| **Before the first prototype** | Safety circuit designed and reviewed |
| **Before powering a machine with an operator near it** | Safety circuit built, tested, and validated |
| **Before the first customer machine** | NRTL panel listing or field evaluation; liability insurance; manuals |
| **Before EU sale** | CE technical file; Machinery Regulation and CRA conformity |

---

**Next:** [`08-licensing.md`](08-licensing.md) — the licensing analysis in full.
