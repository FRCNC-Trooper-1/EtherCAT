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

### ⚠️ LRW is not supported — this one will cost you days

Yaskawa Sigma drives **do not accept a combined LRW logical read/write**.
Working IgH configurations create two separate domains, with the source comment
*"yaskawa drive requires separated domain"*, and LinuxCNC users report the same.

SOEM sends LRW by default. See
[`03-ethercat-bringup.md`](03-ethercat-bringup.md#lrw-vs-lrdlwr--some-drives-cannot-accept-a-combined-readwrite)
for the `blockLRW` fix. **Plan for this from day one.**

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

### Distributed Clocks — Sigma-7 reference values

From a working IgH configuration:

```c
AssignActivate = 0x0300      /* Sync0 enabled, Sync1 unused */
Sync0 shift    = 150000 ns   /* 150 us */
```

The drive can serve as the DC reference clock. Free-run (SM-synchronous)
operation also works, but use DC for coordinated CSP.

### Modes of operation — do not trust the default

One documented unit shipped defaulting to **mode 9 (CSV)**, not 8 (CSP).
**Always write `0x6060` explicitly and wait for `0x6061` to echo it.**

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

### Open questions — resolve before committing to a BOM

1. **What does `Y3600A` change?** Custom/BTO codes can alter the object
   dictionary, default PDO assignment, parameter write access, or firmware.
   Get the BTO datasheet from Yaskawa quoting the full model string. **If no one
   will produce that document, treat the part as unqualified.**
2. **Minimum EtherCAT cycle time for SGDXS.** No figure obtained. If the design
   depends on 250 µs, this is a blocker.
3. **Is `0x60C2` writable?** No information found. A fixed value pins the entire
   machine's cycle time.
4. **Sigma-X PDO mappings and product code** — all mapping data above is
   Sigma-**7**.
5. **Do `0x1C12`/`0x1C13` require SDO Complete Access?** Try CA first, fall back.

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
