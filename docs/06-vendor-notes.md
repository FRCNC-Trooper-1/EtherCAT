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

### ⚠️ LRW — Sigma-7 does NOT support it, Sigma-X DOES

This differs between generations. Both statements are quoted from the
manufacturer's own EtherCAT specification tables.

**Sigma-7 (SGD7S)** — SIEP S800001 xx:

> APRD, FPRD, BRD, LRD, APWR, FPWR, BWR, LWR, ARMW, and FRMW
> **(APRW, FPRW, BRW, and LRW commands are not supported.)**

and in §12.2:

> The SERVOPACK does not support EtherCAT Read/Write commands
> (APRW, FPRW, BRW, and **LRW**).

**Sigma-X (SGDXS)** — SIEP C710812 02:

> APRD, APWR, **APRW**, FPRD, FPWR, **FPRW**, BRD, BWR, **BRW**,
> LRD, LWR, **LRW**, ARMW, FRMW

No exclusion clause. **Sigma-X fixed the limitation.**

| | LRW | `blockLRW` needed? |
|---|---|---|
| Sigma-7 (SGD7S) | ✗ Not supported | **Yes — mandatory** |
| Sigma-X (SGDXS) | ✓ Supported | No |

See
[`03-ethercat-bringup.md`](03-ethercat-bringup.md#lrw-vs-lrdlwr--some-drives-cannot-accept-a-combined-readwrite)
for the fix.

> **Mixing generations on one bus:** `blockLRW` is per-slave, but
> `ecx_config_map_group()` promotes it to the group
> (`grouplist[group].blockLRW`). One Sigma-7 anywhere on the segment drops the
> **entire group** to LRD/LWR. Functionally fine, marginally less efficient —
> but know that it is happening rather than discovering it in a frame capture.

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

### ✅ Sigma-X default assignment — MEASURED

Read off two `SGDXS-xxxxA0xY3503A` drives with `bus_scan`, not taken from an ESI:

| Object | Value | Contents (6 bytes each) |
|---|---|---|
| `0x1C12:01` | **`0x1601`** | `6040` Controlword, `607A` Target position |
| `0x1C13:01` | **`0x1A01`** | `6041` Statusword, `6064` Position actual |

So the Sigma-X ships selecting the **minimal CSP set**, exactly as the Sigma-7
IgH configurations do. That is enough to move an axis and not enough to
supervise one: no `6060`, no `6061`, no `60F4`, no `6077`.

`0x1600` / `0x1A00` are richer and are what this controller assigns:

```bash
sudo ./build/bus_monitor ethX --axes 2 --cycle 4000 --rx-pdo 0x1600 --tx-pdo 0x1A00
```

**Neither `0x1600` nor `0x1A00` carries `0x60B1` (velocity offset).** Velocity
feedforward therefore needs a *custom* mapping written into a spare object, not
just a reassignment.

### ✅ `0x60B1` exists — feedforward is reachable, by composing a mapping

`bus_scan`'s object-dictionary probe, both drives:

| Object | | |
|---|---|---|
| `0x60B1` | velocity offset | **present** |
| `0x60B2` | torque offset | **present** |
| `0x606C` | velocity actual | **present** |

Every object the controller wants is in the dictionary. What is missing is a
*predefined mapping* that carries them — a different problem with a different
fix.

And the fix is available, because the spare mapping objects are **present and
empty**:

```
0x1602  2 entries    0x0000:00  0 bits  PADDING   (x2)   = 0 bytes
0x1603  2 entries    ... empty
0x1A02  2 entries    ... empty
0x1A03  3 entries    ... empty
```

`0x1604`/`0x1A04` and above return `06020000 The object does not exist`, so the
device offers exactly four mapping objects per direction: two populated by the
vendor, two free.

> The reported sub-entry count of 2 (3 on `0x1A03`) is the *current* count of an
> empty object, **not a ceiling** — confirmed by writing: `0x1602` accepted 5
> entries and `0x1A02` accepted 6.

**Use `--custom-map`.** It composes into `0x1602`/`0x1A02` by default, never over
a vendor mapping:

| | RxPDO `0x1602` | TxPDO `0x1A02` |
|---|---|---|
| | `6040` controlword | `6041` statusword |
| | `607A` target position | `6064` position actual |
| | **`60B1` velocity offset** | `606C` velocity actual |
| | `60B2` torque offset | `60F4` following error |
| | `6060` modes of operation | `6077` torque actual |
| | | `6061` modes display |
| Size | 13 bytes | 17 bytes |

`0x6072` max torque is deliberately **excluded**. Composing our own mapping
means it is simply not in the image, so the limit stays in the drive's own
parameters instead of becoming an obligation on the master every cycle — the
opposite trade from `0x1600`, and the better one.

**✅ CONFIRMED ON HARDWARE.** Both drives accepted the composed mapping and read
back exactly what was written:

```
composing 0x1602 (5 entries, 13 bytes) and 0x1A02 (6 entries, 17 bytes)
slave 1   13 out / 17 in bytes  CSP usable
  + 0x6060 mode  - 0x6072 max-trq  + 0x60B1 vel-ff  + 0x60B2 trq-ff
  + 0x6061 mode-disp  + 0x60F4 foll-err  + 0x606C vel-act  + 0x6077 trq-act
```

Every object the controller wants, and only those. Reached OPERATIONAL in 4.0 s
— faster than either predefined mapping — and held it for the run.

**This is the mapping to use.** `--custom-map` also switches
`require_velocity_feedforward` on, so a run that somehow loses `0x60B1` fails at
start-up rather than quietly running without the feedforward the mapping exists
to provide.

#### Result of the reassignment — MEASURED

| | `0x1601`/`0x1A01` (default) | `0x1600`/`0x1A00` (assigned) |
|---|---|---|
| Process data | 6 out / 6 in | **18 out / 20 in** |
| `0x6060` modes of operation | — | **✅** |
| `0x6061` modes display | — | **✅** |
| `0x60F4` following error | — | **✅** |
| `0x6077` torque actual | — | **✅** |
| `0x6072` max torque | — | **✅ — see the warning below** |
| `0x60B1` velocity offset | — | — |
| `0x606C` velocity actual | — | — |

Sigma-X `0x1600`/`0x1A00` therefore mirror the Sigma-7 ESI exactly, including
the trailing pad byte (17 mapped bytes reported as 18, 19 as 20).

With `0x6060` mapped, `0x6061` came back as **8** — the first direct
confirmation that the drives accept CSP from this master rather than an
inference from the fact that nothing complained.

### ⚠️ `0x1600` carries `0x6072` — a mapped max torque you MUST write

The single hazard in taking the richer mapping, and it is not obvious, because
it is a *new obligation* rather than a missing feature.

`0x1600` includes **`0x6072` Max torque**. The process image starts zeroed, so a
master that assigns this mapping and does not write that entry commands a torque
limit of **zero**, on every cycle. The axis enables, reports `internal limit
active` (statusword bit 11), and does not move — a fault that looks like a drive
problem and is not one.

This is a property of the process image, not an observation: an RxPDO entry the
master does not write holds whatever the image holds, and the image starts at
zero. It needs no bench evidence and has none.

> **A retracted claim, kept because the reasoning is the trap.** The statusword
> on the bench read `0x0E08` — Fault + Remote + Target reached + **Internal
> limit active** — and bit 11 was first read as confirmation of the zero torque
> limit. It is not. The comparison run with `0x1601`/`0x1A01`, where `0x6072` is
> not mapped at all and nothing can be commanding zero, reports the same
> `0x0E08`; and bit 11 is already set in PRE-OP, before any PDO write takes
> effect. It belongs to the A.C90 encoder alarm. A symptom that fits a
> hypothesis is not evidence for it until the case without the cause has been
> checked.

`AxisConfig::max_torque_per_mille` handles it, defaulting to **1000** (100% of
rated). That is conservative in both directions — it prevents the zero, and it
*lowers* the limit relative to the several-times-rated the drive powers up with.
`CyclicTask` refuses to start if `0x6072` is mapped and the value is 0, because
once the object is in the RxPDO there is no such thing as leaving it alone.

Its neighbours are harmless: `0x60FF` target velocity and `0x6071` target torque
are ignored by the drive in CSP, so zero costs nothing.

### ✅ Cycle time and `0x60C2` — resolved for both generations

**Supported DC cycles:**

| Generation | Manual, verbatim |
|---|---|
| Sigma-7 | Applicable DC cycles: **125 μs to 4 ms in 125-μs increments** |
| Sigma-X | Applicable DC cycles: **62.5 μs to 4 ms in 62.5-μs increments** |

Both support free-run and DC mode, switchable. **250 µs is comfortably within
range on either.** Sigma-X resolves twice as finely.

**`0x60C2` is READ-WRITE on both.** Sigma-X object dictionary:

| Index | Sub | Name | Type | Access | Range | Default |
|---|---|---|---|---|---|---|
| `60C2h` | 0 | Number of entries | USINT | RO | — | 2 |
| `60C2h` | 1 | Interpolation time period value | USINT | **RW** | 1 to 250 | **125** |
| `60C2h` | 2 | Interpolation time index | SINT | **RW** | −6 to −3 | **−6** |

Period = `value × 10^index` seconds. The default `125 × 10⁻⁶` is **125 µs**.

| Target cycle | value | index |
|---|---|---|
| 125 µs | 125 | −6 |
| 250 µs | 250 | −6 |
| 500 µs | 50 | −5 |
| 1 ms | 100 | −5 |
| 4 ms | 4 | −3 |

**The drive does not pin your cycle time.** This was the single biggest open
risk on the BOM and it is closed on both generations. You set the machine's
cycle; the drive follows.

> Note the default is **125 µs**, not 1 ms. If you bring a drive up at a 1 ms
> master cycle without writing `0x60C2`, the drive expects a new setpoint every
> 125 µs and will interpolate against a period eight times shorter than reality.
> **Always write `0x60C2` explicitly.**

Confirmed on the bench: both drives read back `0x60C2 = 125 µs` out of the box.
`--set-interp` writes it to match the master cycle and verifies the readback —
see [`03 §10`](03-ethercat-bringup.md#10-writing-configuration-in-pre-op).

One encoding difference from the table above, and it is not a discrepancy:
`encode_interpolation_period()` emits 1 ms as **(1, −3)** rather than (100, −5).
Both are inside the documented ranges (`:01` is 1..250, `:02` is −6..−3) and
both are exactly 1 ms. Whole milliseconds take the `−3` form because that is
what the drive itself reports for 4 ms, which keeps a readback comparison
legible in a log. Note the units range is **1 to 250**, narrower than the
UNSIGNED8 type — a cycle needing 251..255 units is refused by the drive, and the
readback check reports it rather than letting it pass.

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

### Sigma-7 vs Sigma-X — confirmed differences

Both verified from their own manuals (SIEP S800001 xx and SIEP C710812 02).

| | Sigma-7 (SGD7S) | Sigma-X (SGDXS) |
|---|---|---|
| **LRW** | ✗ Not supported | ✓ **Supported** |
| **DC cycle range** | 125 µs – 4 ms, 125 µs steps | **62.5 µs** – 4 ms, 62.5 µs steps |
| `0x60C2` | RW | RW, default 125 µs |
| Scaling objects | `2701h`–`2704h` | `2701h`–`2704h` (unchanged) |
| SubDevice Information IF | 256 bytes | **4 KB** |
| CiA 402 modes | HM, PP, IP, PV, PT, CSP, CSV, CST | Identical |
| Terminology | Master / Slave | **MainDevice / SubDevice** |
| Extras | — | Σ-LINK II, FSoE Advanced Safety Module |

Scaling is unchanged between generations — Sigma-X still uses the
manufacturer-specific `2701h` position user unit, **not** the CiA 402
`0x6091`/`0x6092`/`0x608F`.

> Sigma-X §5.18 adds *"Σ-V/Σ-7 Compatible Function and Settings"*, including
> **Encoder Resolution Compatibility Selection**. That strongly implies Sigma-X's
> native encoder resolution differs from Sigma-7's, with a compatibility mode for
> retrofits. **Verify counts-per-rev empirically on the bench** rather than
> carrying a Sigma-7 number across.

### SDO Complete Access

Not stated as required, but the abort-code table confirms it is implemented and
that objects may refuse it:

| Abort code | Meaning |
|---|---|
| `0x06010004` | The object cannot be accessed through complete access |
| `0x06010003` | The entry was not written because the subindex was other than 0 |

The second is the classic **"zero subindex 0 before writing entries"** rule from
[`04 §5`](04-drive-cia402.md#5-pdo-configuration). Try CA first, fall back to
per-subindex on `0x06010004`.

### Open questions

1. **What does `Y3600A` change?** Still unknown, and still the largest risk.
   Custom/BTO codes can alter the object dictionary, default PDO assignment,
   parameter write access, or firmware. Get the BTO datasheet from Yaskawa
   quoting the full model string. **If no one will produce that document, treat
   the part as unqualified.**
2. ~~**Sigma-X product code** (`1018h:02`)~~ — **resolved by `bus_scan`:**
   vendor `0x00000539`, product code `0x02200901`, revision `0x01055030`, on
   both `SGDXS-xxxxA0xY3503A` units.
3. **Sigma-X native encoder resolution** — see the compatibility-mode note above.
4. ~~**Will the drive accept a composed mapping?**~~ — **yes, confirmed.**
   `0x1602` took 5 entries / 13 bytes and `0x1A02` took 6 / 17, read back
   verbatim, with `0x60B1` in the image. Velocity feedforward is available on
   these drives.

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
