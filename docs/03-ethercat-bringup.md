# 03 — EtherCAT Bus Bring-Up with SOEM

> **Goal of this stage:** every slave enumerates with correct identity, the bus
> reaches `OPERATIONAL`, and a cyclic exchange runs for 24 hours with zero
> working-counter errors.
>
> **Prerequisite:** Phase 1 complete — [`02-pc-realtime-setup.md`](02-pc-realtime-setup.md)
> exit criteria met. Do not debug EtherCAT timing on an untuned machine.

> ⚠️ **Licensing:** SOEM v2 is GPLv3-or-commercial, not permissive. Development is
> not distribution, so this does not block you now. See
> [`08-licensing.md`](08-licensing.md) before shipping.

---

## 1. API baseline

Everything in this document was verified against SOEM `master` at commit
`2f73eaa8` (`project(SOEM VERSION 2.0.0)`). **The v2 API is not the v1 API** —
most tutorials, forum posts, and AI-generated examples online target v1 and will
not compile.

The differences that will bite you first:

| v1 (everywhere online) | v2 (what you actually have) |
|---|---|
| `ec_init(ifname)` | `ecx_init(&ctx, ifname)` |
| `ec_config_init(FALSE)` | `ecx_config_init(&ctx)` — no `usetable` argument |
| `ec_config_map(&IOmap)` | `ecx_config_map_group(&ctx, IOmap, 0)` |
| `ec_slave[i]`, `ec_slavecount` | `ctx.slavelist[i]`, `ctx.slavecount` |
| `ec_DCtime` | `ctx.DCtime` |
| `#include "ethercat.h"` | `#include "soem/soem.h"` |

There is **no `EC_VER1` compatibility layer**. There is also **no
`ecx_config_map()`** and **no public overlap-map function** — overlap is selected
by setting `ctx.overlappedMode = TRUE` before calling `ecx_config_map_group()`.

---

## 2. Adding SOEM to the build

```bash
git submodule add https://github.com/OpenEtherCATsociety/SOEM.git extern/soem
git submodule update --init --recursive
```

Pin the submodule to a specific commit and record it. "Latest master" is not a
dependency specification for a product.

SOEM exports a plain target named `soem` — **no namespace**, so it is
`soem`, not `SOEM::soem`:

```cmake
add_subdirectory(extern/soem)
target_link_libraries(your_app PRIVATE soem)
```

Its `target_include_directories` are `PUBLIC`, and `cmake/Linux.cmake` links
`pthread` and `rt` transitively, so you get all of that for free.

Two gotchas:

- SOEM v2 requires **CMake ≥ 3.28**. Ubuntu 24.04 ships exactly 3.28 — fine, but
  tight. Check before assuming.
- **SOEM v2 cannot be compiled outside CMake without extra work.**
  `include/soem/ec_options.h` does not exist in the repo; it is generated from
  `ec_options.h.in` via `configure_file()`. All tuning constants (`EC_MAXSLAVE`,
  `EC_TIMEOUTRXM`, …) are CMake cache variables now, not editable `#define`s.

---

## 3. The context object

In v2, `ecx_contextt` is a **large, fully-inlined struct** — all slave records,
the EEPROM cache, and a 32-entry mailbox pool live inside it.

```c
#include "soem/soem.h"

static ecx_contextt ctx;          /* static or heap. NEVER a stack local. */
static uint8_t      IOmap[4096];
```

Two rules:

1. **Never stack-allocate it.** `sizeof(ecx_contextt)` is large (200 slave records
   plus a mailbox pool of 32 × 1487 bytes). Static or heap only.
2. **Zero it before `ecx_init()`.** `ecx_init()` does *not* memset the context —
   it only calls `ecx_initmbxpool()` and `ecx_setupnic()`. Static storage is
   zero-initialized by definition; heap allocation is not.

Public fields you will use:

```c
ec_slavet slavelist[EC_MAXSLAVE];   /* by value in v2, was a pointer in v1 */
int       slavecount;               /* by value, was int*  */
ec_groupt grouplist[EC_MAXGROUP];
boolean   ecaterror;
int64     DCtime;                   /* ns, updated every receive */
```

Optional fields to set **before** use: `manualstatechange`, `userdata`,
`overlappedMode`, `packedMode`, `ENI`, and the `PO2SOconfig` per-slave hook.

---

## 4. Bring-up sequence

The order matters. Each step depends on the previous one.

```c
/* 1. Open the NIC. Nonzero return means success. */
if (!ecx_init(&ctx, "enp3s0")) { /* fail */ }

/* 2. Enumerate. Returns the number of slaves found. */
int nslaves = ecx_config_init(&ctx);
if (nslaves <= 0) { /* no slaves — check cabling, link, power */ }

/* 3. Per-slave configuration hooks run during PRE-OP -> SAFE-OP.
      Register them BEFORE mapping. This is where CiA 402 PDO setup goes. */
for (int i = 1; i <= ctx.slavecount; i++) {
    ctx.slavelist[i].PO2SOconfig = drive_po2so_config;
}

/* 4. Map process data. Returns IOmap size. Group 0 means all groups. */
int iomap_size = ecx_config_map_group(&ctx, IOmap, 0);

/* 5. Configure Distributed Clocks. Returns TRUE if DC-capable slaves found. */
boolean has_dc = ecx_configdc(&ctx);

/* 6. Per-drive Sync0. cycle_ns and shift_ns in nanoseconds. */
for (int i = 1; i <= ctx.slavecount; i++) {
    ecx_dcsync0(&ctx, i, TRUE, cycle_ns, shift_ns);
}

/* 7. Wait for SAFE-OP. */
ecx_statecheck(&ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

/* 8. Compute expected working counter. */
int expected_wkc = (ctx.grouplist[0].outputsWKC * 2) + ctx.grouplist[0].inputsWKC;

/* 9. Start the cyclic exchange BEFORE requesting OP.
      Drives will not enter OP without valid process data arriving. */
ecx_send_processdata(&ctx);
ecx_receive_processdata(&ctx, EC_TIMEOUTRET);

/* 10. Request OPERATIONAL. */
ctx.slavelist[0].state = EC_STATE_OPERATIONAL;
ecx_writestate(&ctx, 0);
ecx_statecheck(&ctx, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE * 4);
```

> **Step 9 is the one people miss.** A drive will refuse to enter `OP`, or drop
> straight back to `SAFE-OP`, if process data is not already flowing. Start the
> cyclic loop first, then request `OP`.

### Slave numbering

Slaves are **1-indexed**. Index `0` is the aggregate/broadcast pseudo-slave —
`ctx.slavelist[0].state` addresses all slaves at once. `ecx_statecheck(&ctx, 0, …)`
returns the **bitwise OR** of all slave states, which means `EC_STATE_BOOT` can
alias `INIT|PRE_OP`. Check individual slaves when diagnosing.

---

## 5. The cyclic loop

```c
struct timespec ts;
osal_get_monotonic_time(&ts);

for (;;) {
    add_timespec(&ts, cycle_ns + toff);          /* toff from the DC PI controller */
    osal_monotonic_sleep(&ts);

    ecx_send_processdata(&ctx);
    int wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);

    if (wkc < expected_wkc) {
        /* A slave dropped out. This is a fault — stop motion. */
    }

    /* Drain queued mailbox traffic. Required in v2 for SDO access during OP. */
    ecx_mbxhandler(&ctx, 0, 4);

    if (ctx.slavelist[0].hasdc && wkc > 0) {
        dc_sync(ctx.DCtime, cycle_ns, &toff);    /* PI controller, see §6 */
    }

    /* Application: read inputs, run control, write outputs */
}
```

### Absolute, not relative, sleep

Always sleep to an **absolute** next-wake time
(`clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, …)`, which is what
`osal_monotonic_sleep()` wraps). A relative sleep accumulates the execution time
of your loop body as drift, every single cycle.

### Working counter

`expected_wkc = (outputsWKC * 2) + inputsWKC`. Outputs count twice because a
write-read datagram increments the counter twice. **Check it every cycle.** A
mismatch means a slave stopped responding — on a CNC that is a fault condition
requiring a coordinated stop of every axis.

### LRW vs LRD/LWR — some drives cannot accept a combined read/write

By default SOEM sends a single **LRW** datagram covering the combined
output+input logical address range. This is efficient and works for most
slaves. **Some drives — Yaskawa Sigma-7 and Sigma-X among them — do not support
it**, and the symptom is a bus that enumerates perfectly and then produces
garbage or a wrong working counter once process data starts flowing.

SOEM handles this per slave via `blockLRW`. When set, it emits separate **LRD**
and **LWR** datagrams instead (and emulates the LRW double-increment on LWR, so
your working counter arithmetic is unchanged).

`ecx_config_map_group()` sets the flag automatically from the slave's SII
General section (`src/ec_config.c:371`), so a correctly-programmed EEPROM needs
no intervention. Not every vendor sets that bit. To force it:

```c
/* After ecx_config_init(), BEFORE ecx_config_map_group(). */
for (int i = 1; i <= ctx.slavecount; i++) {
    ctx.slavelist[i].blockLRW = 1;
    ctx.slavelist[0].blockLRW++;
}
```

`ecx_send_processdata_group()` checks `grouplist[group].blockLRW`
(`src/ec_main.c:2466`) and switches datagram type accordingly.

> Choosing the normal map over the overlap map does **not** avoid LRW — the
> combined datagram is used either way. `blockLRW` is the only control.
>
> Check whether your drive advertises the SII flag on the bench before assuming
> either behaviour, and log which datagram type ended up in use.

### Mailbox handling — new in v2

If a slave's `mbxhandlerstate == ECT_MBXH_CYCLIC`, `ecx_mbxsend()` **queues** the
transfer rather than sending it, and that queue is only drained by
`ecx_mbxhandler()`. Register slaves with `ecx_slavembxcyclic(&ctx, i)` during
setup, then call `ecx_mbxhandler(&ctx, 0, 4)` every cycle.

**Without this, any SDO read from a non-cyclic thread while the bus is in `OP`
will time out.** This is a genuine v2 architecture change and it is what makes
reading drive diagnostics safe while motion is running.

---

## 6. Distributed Clocks

### The API

```c
boolean ecx_configdc(ecx_contextt *ctx);
void    ecx_dcsync0(ecx_contextt *ctx, uint16 slave, boolean act,
                    uint32 CyclTime, int32 CyclShift);
```

`ecx_configdc()` latches port times on all slaves, measures propagation delay,
and writes each slave's system-time offset. Note it subtracts `946684800`
seconds — **EtherCAT's epoch is 2000-01-01, not 1970-01-01.**

`ecx_dcsync0()` sets the Sync0 pulse. First trigger lands at:

```
t = ((t1 + SyncDelay) / CyclTime) * CyclTime + CyclTime + CyclShift
```

> **No sample in the SOEM repository calls `ecx_dcsync0()`.** You are writing this
> sequence yourself. The register writes are unchanged from v1, so v1-era ordering
> guidance still applies.

### Reading DC time

`ctx.DCtime` (int64, nanoseconds) is filled automatically:
`ecx_send_processdata_group()` appends an `FRMW` datagram on `ECT_REG_DCSYSTIME`
against the group's reference slave, and `ecx_receive_processdata_group()` stores
the result. So the order is **send → receive → read `ctx.DCtime`**.

### The drift PI controller

Your master's clock and the reference slave's clock run at slightly different
rates. Without correction they diverge and Sync0 alignment degrades. Adapted from
SOEM's `samples/ec_sample/ec_sample.c`:

```c
void dc_sync(int64_t reftime, int64_t cycletime, int64_t *offsettime)
{
    static int64_t integral = 0;
    int64_t delta = (reftime - SYNC_OFFSET_NS) % cycletime;

    if (delta > (cycletime / 2)) {
        delta -= cycletime;          /* choose the shorter direction */
    }

    int64_t timeerror = -delta;
    integral += timeerror;
    *offsettime = (int64_t)((timeerror * P_GAIN) + (integral * I_GAIN));
}
```

The returned `offsettime` is added to your next sleep target, nudging the master
cycle into phase with Sync0. It should settle within a few seconds and then stay
within a few microseconds indefinitely. **Log it** — a drifting or oscillating
`toff` is an early warning of a timing problem.

### Sync0 shift

Set the shift so frames have reached every slave before Sync0 fires, but the next
frame has not yet been sent. Roughly **20–50% of cycle time** past worst-case
propagation delay to the last slave.

> Too small a shift produces the classic bug: **one axis lags exactly one cycle**
> behind the others, because the last slave on the segment reads the previous
> cycle's data. It looks like a mechanical problem and it is not.

---

## 7. SDO access

```c
int ecx_SDOread(ecx_contextt *ctx, uint16 slave, uint16 index, uint8 subindex,
                boolean CA, int *psize, void *p, int timeout);
int ecx_SDOwrite(ecx_contextt *ctx, uint16 slave, uint16 index, uint8 subindex,
                 boolean CA, int psize, const void *p, int timeout);
```

- `CA` selects Complete Access. Some drives **require** it for `0x1C12`/`0x1C13`.
- For reads, `psize` is in/out: pass buffer size, receive actual size.
- Use `EC_TIMEOUTRXM` as the default timeout.
- SDO is for configuration in `PRE-OP` and diagnostics in `OP` — **never** for
  cyclic data.

```c
uint32_t value = 0;
int size = sizeof(value);
ecx_SDOread(&ctx, 1, 0x1018, 0x02, FALSE, &size, &value, EC_TIMEOUTRXM);
```

---

## 8. Process data access

```c
uint16 Obits;  uint32 Obytes;  uint8 *outputs;  uint32 Ooffset;  uint8 Ostartbit;
uint16 Ibits;  uint32 Ibytes;  uint8 *inputs;   uint32 Ioffset;  uint8 Istartbit;
```

`Ooffset`/`Ioffset` are new in v2 and give the slave's offset within the IOmap.

```c
uint8_t *out = ctx.slavelist[axis].outputs;
uint8_t *in  = ctx.slavelist[axis].inputs;
```

> **If `Obits < 8` then `Obytes == 0`.** For sub-byte slaves (simple digital I/O)
> you must use `Obits`/`Ostartbit`, not `Obytes`. Silently reading zero bytes from
> a bit-mapped slave is a common bring-up bug.

**Never hardcode PDO offsets.** Read the actual mapping at startup (§9) and build
your offset table from what the drive reports. Vendors change default mappings
between firmware revisions.

---

## 9. Read the drive's actual PDO mapping

The default contents of `0x1600`/`0x1A00` are **vendor-specific**. Do not assume
`0x6040` + `0x607A`. Either parse the ESI XML, or read the mapping back over SDO
at startup:

```c
/* Number of mapped entries */
uint8_t n = 0; int size = sizeof(n);
ecx_SDOread(&ctx, slave, 0x1600, 0x00, FALSE, &size, &n, EC_TIMEOUTRXM);

/* Each entry: bits 31..16 = index, 15..8 = subindex, 7..0 = bit length */
for (uint8_t i = 1; i <= n; i++) {
    uint32_t entry = 0; size = sizeof(entry);
    ecx_SDOread(&ctx, slave, 0x1600, i, FALSE, &size, &entry, EC_TIMEOUTRXM);
    uint16_t obj_index = (entry >> 16) & 0xFFFF;
    uint8_t  obj_sub   = (entry >> 8)  & 0xFF;
    uint8_t  bit_len   =  entry        & 0xFF;
    /* build the offset table */
}
```

This is the single most common source of silently misaligned process images in
multi-vendor setups. Build the table at runtime, log it, and verify it against
what you expect.

---

## 10. Diagnostics

### `slaveinfo`

SOEM's own sample is the first tool to reach for:

```bash
./extern/soem/build/samples/slaveinfo/slaveinfo enp3s0 -sdo -map
```

It prints slave identity, state, PDO mapping, and the SDO object dictionary.
**Save its output for each drive model** into `docs/hardware-qualification/` — it
is the ground truth you will check against when something stops working.

### Common failures

| Symptom | Likely cause |
|---|---|
| No slaves found | Wrong interface, no link, slave unpowered, cable in the wrong port (EtherCAT is directional — IN vs OUT) |
| Enumerates but won't leave `PRE-OP` | PDO mapping rejected — check SDO abort codes; try Complete Access |
| Reaches `SAFE-OP`, never `OP` | Process data not flowing before the `OP` request (§4 step 9), or Sync0 missing |
| Drops `OP` → `SAFE-OP` after seconds | Sync manager watchdog — cycle time inconsistent, or `0x60C2` mismatch |
| WKC lower than expected | A slave stopped responding — check cabling and power |
| One axis lags exactly one cycle | Sync0 shift too small |
| SDO times out while in `OP` | `ecx_mbxhandler()` not being called (§5) |

### Error queue

```c
if (ctx.ecaterror) {
    while (ecx_iserror(&ctx)) {
        printf("%s\n", ecx_elist2string(&ctx));
    }
}
```

Drain it every cycle and log it. EtherCAT errors are how the bus tells you a
problem is developing before it becomes a fault.

---

## 11. Exit criteria

- [ ] All slaves enumerate with correct vendor ID, product code, revision
- [ ] Bus reaches `OPERATIONAL` reliably from cold start, 20 consecutive times
- [ ] 24 h cyclic run at target cycle: **zero** WKC errors, zero state drops
- [ ] DC drift controller stable; `toff` bounded and logged
- [ ] PDO offset table built at runtime from the drive's reported mapping
- [ ] `slaveinfo` output archived per drive model

---

**Next:** [`04-drive-cia402.md`](04-drive-cia402.md) — making a motor turn.
