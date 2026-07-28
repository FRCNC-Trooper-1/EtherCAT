# 04 — CiA 402 Drive Control (CSP Mode)

> **Goal of this stage:** a motor turns smoothly under Cyclic Synchronous Position
> control, commanded by your software, and returns to a safe state on every exit path.
>
> ⚠️ **This is the first phase where the machine can injure someone.**
> **Uncouple the motor from the machine.** Bolt it to the bench, no load, nothing
> on the shaft. Keep an E-stop within reach and use it before you need it.

---

## 1. Sources and confidence

The tables below come from vendor documentation (Synapticon, Kollmorgen, Beckhoff
Infosys, Leadshine, KEB, STÖBER), the `canopen` Python library's `p402.py`, and
the CiA 402 / IEC 61800-7-201 profile.

**ETG.6010** (*Implementation Directive for CiA402 Drive Profile*) is the
authoritative document and is available to ETG members. Obtain it and verify the
state-machine and statusword tables against it before shipping. Values marked
**[VENDOR]** below are known to differ between manufacturers and **must** be read
from your specific drive's manual and ESI file.

---

## 2. State machine

### Controlword `0x6040` (UINT16)

Only bits **0, 1, 2, 3, and 7** affect the state machine.

| Bit | Name | Notes |
|---|---|---|
| 0 | Switch On | |
| 1 | Enable Voltage | |
| 2 | Quick Stop | **Inverted** — 0 means quick stop active |
| 3 | Enable Operation | |
| 4 | Op-mode specific | PP: new set-point; HM: homing start. **Unused in CSP** |
| 5 | Op-mode specific | PP: change set immediately. Unused in CSP |
| 6 | Op-mode specific | PP: absolute/relative. Unused in CSP |
| 7 | Fault Reset | **Rising edge triggered** |
| 8 | Halt | |
| 9–15 | Op-mode specific / reserved / manufacturer | |

### Commands

| Command | Value | Mask |
|---|---|---|
| Shutdown | `0x0006` | `0x0087` |
| Switch On | `0x0007` | `0x008F` |
| Switch On + Enable Operation | `0x000F` | `0x008F` |
| Disable Voltage | `0x0000` | `0x0082` |
| Quick Stop | `0x0002` | `0x0086` |
| Disable Operation | `0x0007` | `0x008F` |
| Enable Operation | `0x000F` | `0x008F` |
| Fault Reset | `0x0080` | rising edge, bit 7 |

### Statusword `0x6041` (UINT16) — state decoding

| State | Mask | Value |
|---|---|---|
| Not Ready to Switch On | `0x4F` | `0x00` |
| Switch On Disabled | `0x4F` | `0x40` |
| Ready to Switch On | `0x6F` | `0x21` |
| Switched On | `0x6F` | `0x23` |
| **Operation Enabled** | `0x6F` | `0x27` |
| Quick Stop Active | `0x6F` | `0x07` |
| Fault Reaction Active | `0x4F` | `0x0F` |
| Fault | `0x4F` | `0x08` |

**The masking detail people get wrong:**

- `0x4F` = `0100 1111` tests bits 0,1,2,3,6. **Bit 5 is don't-care.**
- `0x6F` = `0110 1111` tests bits 0,1,2,3,5,6. **Bit 5 must be 1.**
- **Bit 4 (voltage enabled) is never part of any state test.**
- Bits 7–15 are always don't-care for state decoding.

**Test order matters in an if/else chain.** Check `Fault Reaction Active`
(`0x4F`/`0x0F`) *before* `Fault` (`0x4F`/`0x08`), because `0x0F & 0x4F == 0x0F`
would otherwise never be reached. Check faults before everything else.

Other statusword bits:

| Bit | Meaning |
|---|---|
| 4 | Voltage enabled |
| 5 | Quick stop (**inverted** — 0 = active) |
| 7 | Warning |
| 9 | Remote |
| 10 | Target reached (PP/PV/HM; reserved in CSP) |
| 11 | Internal limit active |
| 12 | **CSP: drive follows command value** |
| 13 | **CSP: following error** |

> Bits 12 and 13 in CSP are **[VENDOR]**-variable — some implementations leave
> bits 9–15 unused. Verify on your drives. If bit 12 is implemented, gate your
> "motion allowed" logic on it.

### Transitions

| # | From → To | Trigger |
|---|---|---|
| 0 | (power on) → Not Ready to Switch On | automatic |
| 1 | Not Ready → Switch On Disabled | automatic, self-test done |
| 2 | Switch On Disabled → Ready to Switch On | Shutdown `0x0006` |
| 3 | Ready to Switch On → Switched On | Switch On `0x0007` |
| 4 | Switched On → Operation Enabled | Enable Operation `0x000F` |
| 5 | Operation Enabled → Switched On | Disable Operation `0x0007` |
| 6 | Switched On → Ready to Switch On | Shutdown `0x0006` |
| 7 | Ready to Switch On → Switch On Disabled | Disable Voltage `0x0000` / Quick Stop `0x0002` |
| 8 | Operation Enabled → Ready to Switch On | Shutdown `0x0006` |
| 9 | Operation Enabled → Switch On Disabled | Disable Voltage `0x0000` |
| 10 | Switched On → Switch On Disabled | Disable Voltage `0x0000` / Quick Stop `0x0002` |
| 11 | Operation Enabled → Quick Stop Active | Quick Stop `0x0002` |
| 12 | Quick Stop Active → Switch On Disabled | automatic when ramp completes (`0x605A`) |
| 13 | **any state** → Fault Reaction Active | internal fault |
| 14 | Fault Reaction Active → Fault | automatic |
| 15 | Fault → Switch On Disabled | Fault Reset `0x0080`, **rising edge** |
| 16 | Quick Stop Active → Operation Enabled | Enable Operation `0x000F`, only if `0x605A` ∈ {5,6,7,8} |

Two implementation consequences:

1. **Transition 13 fires from any state.** Evaluate the fault check *before* the
   normal state dispatch, every cycle.
2. **Transition 15 needs a real 0→1 edge on bit 7.** Holding `0x0080` will not
   repeatedly reset. Drop bit 7 low (typically back to `0x0006`) between attempts.

Option codes: `0x605A` quick stop, `0x605B` shutdown, `0x605C` disable operation,
`0x605D` halt, `0x605E` fault reaction.

---

## 3. Modes of operation

- **`0x6060`** Modes of Operation — INT8, RW
- **`0x6061`** Modes of Operation Display — INT8, RO

| Value | Mode |
|---|---|
| 1 | Profile Position (PP) |
| 3 | Profile Velocity (PV) |
| 4 | Profile Torque (TQ) |
| 6 | **Homing (HM)** |
| 7 | Interpolated Position (IP) — legacy |
| 8 | **Cyclic Synchronous Position (CSP)** ← what a CNC uses |
| 9 | Cyclic Synchronous Velocity (CSV) |
| 10 | Cyclic Synchronous Torque (CST) |

**Always wait for `0x6061` to echo the value written to `0x6060`** before acting
on the new mode. Mode changes are not instantaneous.

> Some older DS402 v2.0 documentation claims values 8–127 are "reserved." That is
> obsolete. CSP/CSV/CST were added in CiA 402-2 and are part of IEC 61800-7-201.

---

## 4. CSP process data

### RxPDO (master → drive)

| Object | Sub | Name | Type | Bytes | Required |
|---|---|---|---|---|---|
| `0x6040` | 00 | Controlword | UINT16 | 2 | **Yes** |
| `0x607A` | 00 | Target Position | INT32 | 4 | **Yes** |
| `0x6060` | 00 | Modes of Operation | INT8 | 1 | Recommended |
| `0x60B1` | 00 | Velocity Offset (feedforward) | INT32 | 4 | Optional |
| `0x60B2` | 00 | Torque Offset (feedforward) | INT16 | 2 | Optional |
| `0x60B8` | 00 | Touch Probe Function | UINT16 | 2 | Optional |

### TxPDO (drive → master)

| Object | Sub | Name | Type | Bytes | Required |
|---|---|---|---|---|---|
| `0x6041` | 00 | Statusword | UINT16 | 2 | **Yes** |
| `0x6064` | 00 | Position Actual Value | INT32 | 4 | **Yes** |
| `0x6061` | 00 | Modes of Operation Display | INT8 | 1 | Strongly recommended |
| `0x60F4` | 00 | Following Error Actual | INT32 | 4 | Recommended |
| `0x606C` | 00 | Velocity Actual | INT32 | 4 | Optional |
| `0x6077` | 00 | Torque Actual | INT16 | 2 | Optional |
| `0x60FD` | 00 | Digital Inputs | UINT32 | 4 | **Recommended for CNC** |
| `0x60B9` | 00 | Touch Probe Status | UINT16 | 2 | Optional |
| `0x60BA` | 00 | Touch Probe Pos1 Positive | INT32 | 4 | Optional |

Minimum CSP image is 6 bytes out / 6 bytes in per axis. A practical CNC axis is
8–10 out / 12–20 in.

**For a CNC specifically, map these:**

- **`0x60FD`** — limit and home switch state must arrive in the *same cycle* as
  position, not via a separate SDO round-trip.
- **`0x60F4`** — supervise following error in the master, not only in the drive.
- **`0x6061`** — makes CSP ↔ HM mode handshakes cyclic instead of SDO round-trips.

---

## 5. PDO configuration

### Mapping entry encoding

Each sub-index of `0x1600`/`0x1A00` is a UINT32:

```
bits 31..16 = object index
bits 15..8  = sub-index
bits  7..0  = bit length
```

| Entry | Meaning |
|---|---|
| `0x60400010` | Controlword, sub 0, 16 bits |
| `0x607A0020` | Target Position, sub 0, 32 bits |
| `0x60410010` | Statusword, sub 0, 16 bits |
| `0x60640020` | Position Actual, sub 0, 32 bits |
| `0x60610008` | Modes of Op Display, sub 0, 8 bits |
| `0x00000008` | 8-bit padding (byte alignment) |

### Configuration sequence

All via SDO in **PRE-OP**, before the PRE-OP → SAFE-OP transition, because
SyncManager sizes are validated on that transition. In SOEM this belongs in the
`PO2SOconfig` hook.

1. Zero the assignment: `0x1C12:00 = 0`, `0x1C13:00 = 0`
2. Zero the mapping count: `0x1600:00 = 0`, `0x1A00:00 = 0`
   *(most drives reject entry writes while the count is non-zero)*
3. Write entries: `0x1600:01 = 0x60400010`, `0x1600:02 = 0x607A0020`, …
   and `0x1A00:01 = 0x60410010`, `0x1A00:02 = 0x60640020`, …
4. Restore counts: `0x1600:00 = n`, `0x1A00:00 = m`
5. Re-assign: `0x1C12:01 = 0x1600`, `0x1C12:00 = 1`;
   `0x1C13:01 = 0x1A00`, `0x1C13:00 = 1`
6. Write CSP config: `0x6060 = 8`, `0x60C2`, scaling, limits
7. Configure DC / Sync0
8. PRE-OP → SAFE-OP → OP

**[VENDOR] caveats:**

- Some drives **require SDO Complete Access** on `0x1C12`/`0x1C13`. Pass `CA = TRUE`.
- Many drives have **fixed, read-only PDO mappings** and only allow *selecting*
  among predefined `0x1600`/`0x1601`/`0x1602` via `0x1C12`. Check the ESI for
  `Fixed="1"` attributes.
- **Default mapping content is vendor-specific.** Read it back rather than
  assuming — see [`03 §9`](03-ethercat-bringup.md#9-read-the-drives-actual-pdo-mapping).

---

## 6. Configuration objects

| Object | Sub | Name | Purpose |
|---|---|---|---|
| `0x6091` | 01/02 | Gear Ratio | motor revs ÷ shaft revs |
| `0x6092` | 01/02 | Feed Constant | linear travel per shaft rev (ballscrew pitch) |
| `0x608F` | 01/02 | Position Encoder Resolution | encoder increments per motor rev |
| `0x607C` | 00 | Home Offset | machine zero vs. home position |
| `0x6098` | 00 | Homing Method | |
| `0x6099` | 01/02 | Homing Speeds | fast switch search / slow index seek |
| `0x609A` | 00 | Homing Acceleration | |
| `0x60C2` | 01/02 | **Interpolation Time Period** | period = sub1 × 10^sub2 seconds |
| `0x6065` | 00 | Following Error Window | half-width; `0xFFFFFFFF` disables monitoring |
| `0x6066` | 00 | Following Error Timeout | time outside window before bit 13 sets |
| `0x607D` | 01/02 | Software Position Limit | min / max, clamps position demand |
| `0x607E` | 00 | Polarity | direction inversion |

### `0x60C2` must match your cycle

For 1 ms: `0x60C2:01 = 1`, `0x60C2:02 = -3`.

| Mismatch | Symptom |
|---|---|
| `0x60C2` > actual cycle | Drive runs at a **fraction of commanded velocity**, continuous lag, eventual following-error fault |
| `0x60C2` < actual cycle | Drive **dwells** at each setpoint — velocity ripple, audible cogging, rough finish |
| Master cycle ≠ Sync0 | Stale or missing setpoints; sync manager watchdog / "synchronization overflow" counters climb |

> ⚠️ **`0x60C2` is read-only on some drives**, fixed at values like 2 ms. If any
> drive in the chain has a fixed period, **the entire machine's cycle time is
> pinned to it.** Check this on every drive model *before* designing your real-time
> loop. This has forced complete redesigns.

### Scaling: do it in the master

The unit chain is:

```
encoder increments → (÷ 0x608F) motor revs → (÷ 0x6091) shaft revs → (× 0x6092) mm
```

If two axes resolve to different counts-per-mm and the master assumes a common
unit, **every interpolated move produces a path error that grows with distance** —
circles become ellipses, 45° cuts drift.

**[VENDOR]:** vendors differ on whether these objects are actually applied to
`0x607A`/`0x6064` at all. For mixed-vendor machines, the safe approach is to set
gear 1:1 and feed constant 1:1, and do all unit conversion in the master where you
control the arithmetic exactly. **Verify empirically** — command a known move and
measure the result.

### Following error is your safety net

In CSP the *master* is the trajectory generator, so a following error means the
drive could not keep up — a crash, a stall, or lost position.

> **When any axis reports a following error, fault every axis.** Otherwise the
> remaining axes keep interpolating against a stopped axis and gouge the part.
> Do not rely on the drive's own reaction alone.

Similarly, `0x607D` clamps position *demand* — some drives silently limit rather
than fault, which on a coordinated move again distorts the path. Enforce soft
limits in the master too, and treat drive-side clamping as a backstop.

---

## 7. Enable sequence

Order matters, and one step in particular prevents a violent surprise.

```
1. Bus in OP, cyclic exchange running, DC locked
2. Read Statusword. If Fault  -> Fault Reset (rising edge on bit 7), then 0x0006
3. Set 0x6060 = 8 (CSP). Wait for 0x6061 == 8
4. ***Set 0x607A = current 0x6064***      <-- CRITICAL
5. Controlword 0x0006 (Shutdown)          -> Ready to Switch On
6. Controlword 0x0007 (Switch On)         -> Switched On
7. Controlword 0x000F (Enable Operation)  -> Operation Enabled
8. Keep writing 0x607A every cycle, starting from the current position
```

> **Step 4 is not optional.** If `0x607A` still holds a stale value when you
> enable, the drive will slam to that position at maximum acceleration the instant
> torque is applied. This breaks machines and injures people. **Seed the target
> position from the actual position on every enable, without exception.**

### Disable sequence

```
1. Ramp commanded position to a stop (do not just stop writing)
2. Controlword 0x0007 (Disable Operation)
3. Controlword 0x0000 (Disable Voltage)
```

**Every abnormal exit path — crash, signal, exception, watchdog — must leave the
drives disabled.** Install signal handlers and a `std::terminate` handler that
drive the disable sequence. A control process that dies leaving drives enabled is
a machine that keeps moving with nobody in charge.

---

## 8. Homing

### Methods (`0x6098`, INT8)

| Value | Method |
|---|---|
| 0 | No homing |
| 1 | Negative limit switch + index pulse |
| 2 | Positive limit switch + index pulse |
| 3, 4 | Positive home switch + index pulse |
| 5, 6 | Negative home switch + index pulse |
| 7–14 | Home switch + limit switch + index (various approaches) |
| 17–30 | Same as 1–14 but **without** index pulse — switch edge only |
| 33, 34 | Index pulse only (negative / positive) |
| 35 | Current position = home *(older editions, e.g. EPOS2)* |
| 37 | Current position = home *(CiA 402-2, e.g. EPOS4)* |
| −1…−4 | Manufacturer specific (typically hard-stop / torque homing) |

> **The 35 vs 37 split is a real edition difference, not a typo.** Using the wrong
> one gets an SDO abort or, worse, an unexpected homing move. Check your drive.

### Do homing in the drive, not the master

Switch `0x6060 = 6`, wait for `0x6061 == 6`, set controlword bit 4, poll
statusword bit 12 (attained) / bit 13 (error), then return to `0x6060 = 8`.

The drive sees the index pulse at **encoder timing resolution**. Master-side
homing can only resolve the switch edge to one bus cycle — at approach speed
that is a real repeatability loss.

**Absolute encoders remove the problem entirely.** Multi-turn absolute encoders
with battery backup retain position across power loss; homing completes
immediately with no motion. For a commercial CNC this is the preferred design.

**Master-side homing** is justified only when you need a cross-axis constraint the
drives cannot express — gantry squaring on a dual-driven axis, for instance. Then
you stay in CSP, jog via `0x607A`, and watch `0x60FD` — which is exactly why
`0x60FD` belongs in the TxPDO.

---

## 9. Exit criteria

- [ ] Drive reaches `Operation Enabled` reliably from cold start and from fault
- [ ] Fault reset works, including the rising-edge requirement
- [ ] Motor follows a commanded profile smoothly at target cycle time
- [ ] Following error within budget across the full speed range
- [ ] `0x607A` seeded from `0x6064` on every enable — **verified by test**
- [ ] Every abnormal exit path leaves the drive disabled — **verified by killing
      the process at speed**
- [ ] Soft limits enforced in the master and in the drive
- [ ] Homing repeatable to specification

---

## 10. Verify before shipping

Items that are vendor-specific or that could not be confirmed against ETG.6010:

1. Full transition table and statusword bit layout — verify against **ETG.6010**
2. Default PDO mapping — **read from the drive**, never hardcode
3. `0x60C2` writability — check every drive model
4. Whether `0x6091`/`0x6092`/`0x608F` actually scale `0x607A`/`0x6064` — verify
   empirically with a measured move
5. Complete Access requirement on `0x1C12`/`0x1C13`
6. Homing method 35 vs 37
7. Statusword bits 12/13 semantics in CSP

---

**Next:** [`05-motion-architecture.md`](05-motion-architecture.md) — trajectory
generation and the real-time software architecture.
