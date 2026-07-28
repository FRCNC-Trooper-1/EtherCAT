# 06 — Vendor-Specific Drive Notes

Per-manufacturer findings that don't belong in the generic CiA 402 reference.
Add to this as each drive family is qualified — a quirk discovered on the bench
and written down here is a week nobody else loses.

Every entry states its confidence and source. **Anything marked UNVERIFIED must
be confirmed against the drive's manual or the drive itself before it is
designed around.**

---

## Yaskawa — Sigma-7 (SGD7S) and Sigma-X (SGDXS)

### Part number decode

Example: `SGDXS-7R6AA0AY3600A`

| Field | Value | Meaning | Confidence |
|---|---|---|---|
| `SGDXS` | Σ-X series | Single-axis SERVOPACK (XW = 2 axis, XT = 3 axis) | High |
| `7R6` | 7.6 A | ≈ 1.0 kW at 200 V | High |
| `A` | 200 V class | 200–240 VAC | High |
| `A0` | **EtherCAT (CoE)** | The network-interface field | High |
| `A` | Design revision | | Medium |
| `Y3600A` | **Custom / BTO order code** | **Meaning unverified** | — |

The `A0` field is the one that matters. It is confirmed by Yaskawa's own ESI
filenames (`Yaskawa_SGD7S-xxxxA0x.xml`, `Yaskawa_SGDXS-xxxxA0x.xml`), where the
wildcards are capacity and revision while `A0` stays literal. The analog/pulse
variants use `00A` in the same position.

### Profile: CoE, not SoE ✅

**Yaskawa EtherCAT servo drives use CoE with the CiA 402 profile.** Every
Yaskawa servo EtherCAT manual across Σ-V, Σ-7 and Σ-X is titled *"EtherCAT (CoE)
Communications Reference"*, and the ingested Sigma-7 ESI describes a standard
CiA 402 object set.

This resolves the concern raised in [`01-hardware-selection.md`](01-hardware-selection.md#3-servo-drives).
Everything in [`04-drive-cia402.md`](04-drive-cia402.md) applies.

> Do not confuse **SoE** (Servo drive profile over EtherCAT) with **FSoE**
> (Fail Safe over EtherCAT). Yaskawa does offer FSoE — that is a safety
> protocol, not a drive profile, and it does not change the CoE picture.

**Settle it on the bench in five minutes.** After `ecx_config_init()`, inspect
what the slave advertises in its SII rather than trusting any document:

```c
printf("slave %d mbx_proto=0x%04x CoEdetails=0x%02x SoEdetails=0x%02x\n",
       i, ctx.slavelist[i].mbx_proto,
       ctx.slavelist[i].CoEdetails, ctx.slavelist[i].SoEdetails);
```

This is authoritative for the actual hardware in front of you, including a
custom-spec unit.

### ⚠️ LRW is not supported — CONFIRMED in the manual

**Source: SGD7S EtherCAT (CoE) Communications Reference, verbatim:**

> APRD, FPRD, BRD, LRD, APWR, FPWR, BWR, LWR, ARMW, and FRMW
> **(APRW, FPRW, BRW, and LRW commands are not supported.)**

and, in §12.2:

> The SERVOPACK does not support EtherCAT Read/Write commands
> (APRW, FPRW, BRW, and **LRW**).

This is now manufacturer-confirmed, not community folklore. **SOEM sends LRW by
default** — see
[`03-ethercat-bringup.md`](03-ethercat-bringup.md#lrw-vs-lrdlwr--some-drives-cannot-accept-a-combined-readwrite)
for the `blockLRW` fix. **Mandatory for these drives.**

### Sync Manager and FMMU layout (fixed)

| Sync Manager | Assignment | Size | Start address |
|---|---|---|---|
| SM0 | Receive mailbox | 128 bytes (fixed) | `0x1000` |
| SM1 | Transmit mailbox | 128 bytes (fixed) | `0x1080` |
| SM2 | Receive PDOs (RxPDO) | 0–256 bytes | `0x1100` |
| SM3 | Transmit PDOs (TxPDO) | 0–256 bytes | `0x1400` |

FMMU0 → RxPDO area, FMMU1 → TxPDO area, FMMU2 → mailbox status.

### ⚠️ SDO writes to PDO-mapped objects are silently ignored

From the trial-operation procedure, repeated several times:

> Manipulate the objects that were mapped to PDOs.
> **Values will not be written if you manipulate SDOs.**

Once an object is mapped into a PDO, writing it over SDO does nothing — and
does not error. Configure via SDO in `PRE-OP`, then drive everything mapped
through process data. A "why is my controlword being ignored" session usually
ends here.

### ⚠️ Sigma-X dropping to PRE-OP after servo-on

Reported on the LinuxCNC forum: Sigma-X reaches `OPERATIONAL`, then falls back
to `PRE-OP`/`PRE-OP+E` once the servo-on controlword is sent. The circulating
workaround is to set the **Sync Error Counter Limit** (`0x1C32:11` / `0x1C33:11`)
to `0`, disabling the check.

**That workaround masks a real synchronisation problem rather than fixing it.**
The drive is telling you its sync error counter is climbing, which means the
master's timing is not good enough. Treat it as a jitter/DC problem first:
verify Sync0 shift, verify the DC drift controller has settled, verify
`0x60C2` matches the actual cycle. Only disable the counter if you have proven
the timing is sound and still need to move on.

Related: alarm **A.12 / "EtherCAT Output Synchronization Error"** is the same
class of problem.

### Default PDO mappings — Sigma-7 (product code `0x02200301`)

From the ingested Sigma-7 ESI. Identical across revisions r7.04 → r8.19.
**Sigma-X very likely mirrors this but is UNVERIFIED — read your own ESI.**

| RxPDO | Contents |
|---|---|
| `0x1600` | `6040` Controlword, `607A` Target position, `60FF` Target velocity, `6071` Target torque, `6072` Max torque, `6060` Modes of operation, `60B8` Touch probe function |
| **`0x1601`** | `6040`, `607A` — **minimal CSP set** |
| `0x1602` | `6040`, `60FF` |
| `0x1603` | `6040`, `6071` |

| TxPDO | Contents |
|---|---|
| `0x1A00` | `6041` Statusword, `6064` Position actual, `6077` Torque actual, `60F4` Following error, `6061` Modes display, `60B9` Touch probe status, `60BA` Touch probe 1 |
| **`0x1A01`** | `6041`, `6064` — **minimal CSP set** |
| `0x1A02` | `6041`, `6064` |
| `0x1A03` | `6041`, `6064`, `6077` |

Working IgH CSP configurations select `0x1601` / `0x1A01` on SM2 (out) / SM3 (in).

For a CNC, prefer `0x1A00` over `0x1A01` — it already carries `60F4` (following
error) and `6061` (modes display), both of which you want cyclically rather
than over SDO. See [`04 §4`](04-drive-cia402.md#4-csp-process-data).

### ✅ Cycle time and `0x60C2` — both resolved from the manual

**Supported DC cycles (manual, verbatim):**

> Free-Run Mode and DC Mode (Can be switched.)
> **Applicable DC cycles: 125 μs to 4 ms in 125-μs increments**

So 125 µs, 250 µs, 375 µs, 500 µs … 4 ms. **250 µs is supported. So is 125 µs.**

**`0x60C2` is READ-WRITE** — from the object dictionary table:

| Index | Sub | Name | Access | PDO map | Type |
|---|---|---|---|---|---|
| `60C2h` | 1 | Interpolation time period value | **RW** | No | USINT |
| `60C2h` | 2 | Interpolation time index | **RW** | No | SINT |

**The drive does not pin your cycle time.** This was the single biggest open
risk on the BOM and it is closed. You set the machine's cycle; the drive follows.

### Distributed Clocks

Mode is selected in the ESC Sync Control registers (`0x980`/`0x981`):

- **Free-Run** — `0x980 = 0x0000`, local cycle independent of the master
- **DC Mode** — `0x980 = 0x0300`, synchronized to Sync0

`0x0300` matches the AssignActivate value used by working IgH configurations,
which also use a **150 µs Sync0 shift**. The drive can serve as the DC reference
clock. Use DC for coordinated CSP; free-run exists but is not appropriate for
contouring.

### PDO mapping is writable only in PRE-OP

Manual, verbatim:

> The PDO mapping objects (indexes 1600h to 1603h and 1A00h to 1A03h) and the
> Sync Manager PDO assignment objects (index 1C12h and 1C13h) **can be written
> only in Pre-Operational state.**

This confirms the configuration sequence in
[`04 §5`](04-drive-cia402.md#5-pdo-configuration) — do it all in the
`PO2SOconfig` hook.

### Supported CiA 402 modes

Homing, Profile Position, Interpolated Position, Profile Velocity, Profile
Torque, **Cyclic Synchronous Position**, Cyclic Synchronous Velocity, Cyclic
Synchronous Torque, Touch Probe, Torque Limit.

### Modes of operation — do not trust the default

One documented unit shipped defaulting to **mode 9 (CSV)**, not 8 (CSP).
**Always write `0x6060` explicitly and wait for `0x6061` to echo it.**

### ⚠️ Scaling uses manufacturer objects, NOT the CiA 402 standard ones

This is the finding most likely to produce wrong parts.

[`04 §6`](04-drive-cia402.md#6-configuration-objects) describes the standard
scaling chain — `0x6091` gear ratio, `0x6092` feed constant, `0x608F` encoder
resolution. **Yaskawa does not use those.** It uses manufacturer-specific
objects:

| Index | Purpose |
|---|---|
| `2701h` | **Position user unit** — sub 1 Numerator, sub 2 Denominator |
| `2702h` | Velocity user unit |
| `2703h` | Acceleration user unit |
| `2704h` | Torque user unit |
| `2705h` | Encoder selection |

```
2701h:01  Numerator    UDINT  RW  1 to 1,073,741,823  (default 1)
2701h:02  Denominator  UDINT  RW  1 to 1,073,741,823  (default 1)
```

**Constraint:** `1/4096 < Numerator/Denominator < 65536`. Outside that range the
drive raises alarm **A.A20 (Parameter Setting Error)**.

### ⚠️ A 24-bit encoder does not give you 24 bits

From §5.14.1, verbatim:

> For a Rotary Servomotor with an encoder resolution of 24 bits (16,777,216),
> Pn20E (Electronic Gear Ratio (Numerator)) is automatically set to 16 and
> Pn210 (Electronic Gear Ratio (Denominator)) is automatically set to 1.
> **Therefore, the encoder resolution will be equivalent to 20 bits (1,048,576).**

The drive silently applies a 16:1 electronic gear, so a 24-bit encoder presents
**1,048,576 counts/rev**, not 16,777,216.

Get this wrong in your counts-per-mm and **every axis is off by a factor of 16**.
Verify empirically on the bench: command a known move, measure the actual travel,
and confirm the arithmetic before trusting any datasheet number. This is exactly
why [`05 §5`](05-motion-architecture.md#5-kinematics-and-units) recommends doing
unit conversion in the master rather than in the drive.

### Encoder — batteryless absolute ✅

Sigma-X motors (SGMXJ / SGMXA / SGMXG) use a **26-bit batteryless absolute**
encoder (67,108,864 counts/rev), retaining multi-turn position without a battery.

**Practical effect: no homing sequence required.** Absolute machine position is
available at power-up. For a commercial CNC this is a significant advantage —
no home switches to wire, no homing move on every power cycle, no repeatability
loss from switch-edge resolution.

> Encoder type is encoded in the **motor** part number, not the drive's.
> Incremental variants exist. Verify the motor P/N you are actually quoting.

### Safety

Built-in **STO** is standard. Reported as **SIL2 / PLd / Cat 3** with
single-channel I/O, and **SIL3 / PLe / Cat 3** with dual-channel via the
Advanced Safety Module options. Declared against EN ISO 13849-1:2015,
EN 62061, EN 61800-5-2.

This meets the **Cat 3 PLd** target in
[`07-safety-and-compliance.md`](07-safety-and-compliance.md) without an option
module — but the PL rating of the *built-in* STO specifically is **UNVERIFIED**.
Obtain Yaskawa's safety declaration; you need the PFHd and MTTFd figures for
your own ISO 13849 calculation regardless.

### ESI files

Publicly downloadable, no login:

```
https://www.yaskawa.com/downloads/search-index/details?showType=details&docnum=Yaskawa_Sigma-X_CoE_ESI_Files
```

Filename: `Yaskawa_SGDXS-xxxxA0x.xml`

**This one file answers several open questions in under an hour.** Read:

- `<Dc>` / `<OpMode>` → `<CycleTimeSync0>` and supported AssignActivate values —
  settles minimum cycle time
- `<TxPdo>` / `<RxPdo>` `Fixed=` and `Sm=` attributes — settles whether mappings
  are remappable
- grep for `60C2` — settles interpolation-time-period access

> SOEM does **not** consume ESI files — it reads SII and CoE from the device
> itself. This is an advantage here: the ESI-revision-mismatch failures that
> break TwinCAT-style masters on Yaskawa firmware do not affect SOEM. Read the
> ESI as *documentation*, not as a runtime dependency.

### Identifiers

| | |
|---|---|
| Vendor ID | `0x00000539` |
| Sigma-7 single-axis 200 V EtherCAT product code | `0x02200301` |
| Sigma-X product code | **UNVERIFIED** |

### Identity (confirmed, object `1018h`)

| Sub | Field | Value |
|---|---|---|
| 1 | Vendor ID | `0x00000539` |
| 2 | Product code (SGD7S) | `0x02200301` |
| 3 | Revision | bits 31–16 major, 15–0 minor |
| 4 | Serial number | always `0x00000000` — not used |

### Open questions

Resolved from the SGD7S manual: cycle time, `0x60C2` access, LRW, PDO mapping
state restriction, SM/FMMU layout, identity, scaling objects.

Still open:

1. **What does `Y3600A` change?** Custom/BTO codes can alter the object
   dictionary, default PDO assignment, parameter write access, or firmware.
   Get the BTO datasheet from Yaskawa quoting the full model string. **If no one
   will produce that document, treat the part as unqualified.**
2. **Does Sigma-X match Sigma-7?** Everything confirmed above is from the
   **SGD7S** manual. Sigma-X is a different generation — for SGDXS, get
   `SIEP C710812 02` and re-verify at minimum the DC cycle range, `0x60C2`
   access, and whether LRW is still unsupported.
3. **Do `0x1C12`/`0x1C13` require SDO Complete Access?** Not mentioned in the
   manual. Try CA first, fall back to per-subindex.
4. **Sigma-X product code** — unknown.

> **Note on `10F1h` (Sync error setting).** Present in the object dictionary.
> This is the knob behind the Sigma-X PRE-OP drop-out workaround. Treat a
> climbing sync error counter as a timing problem to fix, not a check to disable.

### Sigma-7 vs Sigma-X for a first build

| | Sigma-7 (SGD7S) | Sigma-X (SGDXS) |
|---|---|---|
| US availability | Multiple distributors, active secondary market | Limited; ~12 week lead observed |
| Published pricing | Yes | Not found |
| Open-source reference code | Working IgH examples exist | None found |
| Community experience | Substantial (LinuxCNC, IgH) | Minimal |
| Product code / ESI ingested | Yes | No |
| Bandwidth | 3.1 kHz | 3.5 kHz |
| Safety options | Good | Newer, more extensive |

For a **first commercial build on a master you are writing yourself**, Sigma-7
carries materially less integration risk: a supply chain, published prices, a
known product code, and other people who have already hit and solved the failure
modes. Sigma-X buys modestly higher bandwidth and newer safety options.

Worth pricing `SGD7S-7R6AA0A` alongside whatever the Sigma-X quote turns out to be.

---

## Template for new entries

```markdown
## <Vendor> — <Family>

### Part number decode
### Profile (CoE / SoE)
### Cycle time and 0x60C2
### Default PDO mappings
### Distributed Clocks
### Encoder
### Safety
### Quirks and workarounds
### Identifiers (vendor ID, product code)
### Open questions
```

**Record `slaveinfo` output for every drive model** into
`docs/hardware-qualification/`. It is the ground truth you check against when a
drive that worked last month stops working.
