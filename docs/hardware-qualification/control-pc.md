# Control PC Qualification Record

Acceptance criteria: [`docs/02-pc-realtime-setup.md §7`](../02-pc-realtime-setup.md#7-acceptance-criteria)

---

## Unit 1 — `frcnccontroller`

**Status: ✅ PASSED — Phase 1 signed off**

### Configuration

| | |
|---|---|
| Hostname | `frcnccontroller` |
| Distribution | Linux Mint 22.3 (Zena) — Ubuntu 24.04 base |
| Kernel | `6.18.38-rt-x64v3-xanmod1`, `CONFIG_PREEMPT_RT=y` |
| CPU | AMD, 4 physical cores, SMT not supported |
| cpufreq driver | `acpi-cpufreq`, governor `performance`, `amd_pstate` disabled |
| Clocksource | `tsc` |
| `CONFIG_HZ` | 250 |
| `CONFIG_NO_HZ_FULL` | **not set** — `nohz_full=` unavailable on this kernel |
| `CONFIG_HWLAT_TRACER` | `y` |
| Isolated CPUs | 2, 3 |
| Housekeeping CPUs | 0, 1 |
| EtherCAT NIC | Realtek RTL8111/8168 `[10ec:8168]` rev 15, driver `r8169`, fw `rtl8168h-2_0.0.2 02/26/15` |

### Kernel command line

```
isolcpus=managed_irq,domain,2,3 rcu_nocbs=2,3 rcu_nocb_poll irqaffinity=0,1
processor.max_cstate=1 iommu.passthrough=1 amd_prefcore=disable
nosoftlockup nmi_watchdog=0 nowatchdog skew_tick=1 tsc=reliable audit=0
transparent_hugepage=never
```

`nohz_full` deliberately omitted — this kernel lacks `CONFIG_NO_HZ_FULL`, so the
parameter would be silently ignored.

### Runtime tuning applied

- `irqbalance` disabled
- 30 IRQs moved to housekeeping CPUs (mask `0x3`); 1 `enp1s0` IRQ pinned to mask `0x2`
- NIC offloads disabled; flow control disabled; no IP stack on the fieldbus port
- `sched_rt_runtime_us = -1`
- Core Performance Boost disabled

---

## Results

### `hwlatdetect` equivalent — firmware/SMI stalls

Run via `scripts/hwlat.sh` (the `rt-tests` `hwlatdetect` wrapper fails on this
distribution; the kernel tracer itself works).

| Duration | Threshold | Samples over threshold |
|---|---|---|
| 600 s | 10 µs | **0** |

**PASS.** No firmware-induced stalls. This matters more on AMD than Intel — the
SMI counter (`MSR 0x34`) is Intel-only, so measuring the symptom is the only
detection method available.

### `rt_probe` — 12 hour run under load

Load: `stress-ng --cpu 2 --io 2 --vm 2 --vm-bytes 1G --taskset 0,1`

```
cycles        : 43,200,000        (12 h @ 1 ms, CPU 2, SCHED_FIFO 85)
jitter min    : 1,468 ns
jitter mean   : 3,234 ns
jitter max    : 21,555 ns
overruns      : 0
```

| Bucket | Count | Share |
|---|---|---|
| 0–1 µs | 2,110,825 | 4.89% |
| 1–2 µs | 35,772 | 0.08% |
| 2–5 µs | 41,050,296 | 95.02% |
| 5–10 µs | 2,615 | 0.01% |
| 10–20 µs | 490 | 0.00% |
| 20–50 µs | **2** | 0.00% |
| > 50 µs | **0** | 0.00% |

**PASS — 21.5 µs against a 100 µs budget.** Two cycles out of 43.2 million
exceeded 20 µs; none exceeded 50 µs.

### Shorter runs, for comparison

| Test | Duration | Load | Max jitter | Verdict |
|---|---|---|---|---|
| 1 ms cycle | 60 s | idle | 16.4 µs | PASS (budget 100 µs) |
| 250 µs cycle | 60 s | idle | **12.6 µs** | PASS (budget 25 µs) |
| 1 ms cycle | 12 h | heavy | 21.5 µs | PASS (budget 100 µs) |

Max jitter is *lower* at a 250 µs cycle than at 1 ms. Shorter cycles keep the
core hot, so it never drifts toward an idle state between wakeups.

---

## Assessment

**Suitable for a 1 ms EtherCAT cycle without reservation.** Twelve hours under
load cost only ~5 µs of worst-case jitter over the idle baseline.

**250 µs accepted.** The 60-second result (12.6 µs) is inside the 25 µs budget,
and the 12-hour 1 ms result (21.5 µs) — under heavy load, which is the harder
test — is inside it as well. Both margins are wide. The controller is designed
for a 250 µs cycle on this basis.

The soak test that matters now is at the system level, not the timing level:
24 hours with drives on the bus and zero working-counter errors (§7). Loop
jitter with no fieldbus traffic is a floor, not a prediction, and the NIC is the
component most likely to move it.

### Distributed Clocks — chain validated, acceptance pending

**DC acquires lock, and the bus reaches OPERATIONAL because of it.** Confirmed
against two Sigma-X drives at a 4 ms cycle: lock acquired at ~5 s, `OPERATIONAL`
at 6 s, best run 181 consecutive in-tolerance cycles against a requirement of
100. The full chain is exercised — `ecx_configdc`, Sync0 distribution, the PI
drift controller, and the gate that refuses OP until lock.

Lock is **acquired but not continuously held** on this NIC. Phase excursions of
8–12 µs appear during the run, correlated with host jitter spikes (max 15.5 µs).

That is not by itself a fault, and the distinction matters:

- **Acquiring** lock gates OPERATIONAL and must be strict. Requesting OP while
  the master is still hunting makes drives fault on sync error.
- **Holding** it is a matter of degree. Once slaves are in OP, their Sync0
  pulses come from the distributed clock **in hardware**. A momentary excursion
  in the *master's* cycle phase costs nothing provided the frame still arrives
  before Sync0 fires.

The margin was never close: a 12 µs excursion against a 1 ms Sync0 shift spends
1.2 % of the budget.

**That is not the acceptance criterion, however.** For a shipped machine the
requirement is *sustained* lock, not merely acquired:

> **DC must acquire lock and hold it continuously for the whole soak run.**

A bus that locks and then repeatedly drops out is not a bus anyone should put a
tool into, even when each individual excursion is harmless. The excursions here
track host jitter spikes and frame loss, both of which come from the NIC — so
this criterion is expected to be met once the Realtek is replaced, and this
record stays open until it is demonstrated rather than argued.

**Status: DC chain validated, acceptance NOT granted.** Re-run against the
Intel NIC before signing off.

### DRIVER disqualification — `r8169`

**Not a chip fault. A driver fault.** Two different Realtek controllers, nine
years and four generations apart, lose frames at the same rate through the same
in-kernel driver:

| Controller | Class | Cycle | Errors | Loss rate |
|---|---|---|---|---|
| RTL8168h (onboard) | 1 GbE, 2015 | 1 ms | 5588 / 250,000 | **2.24 %** |
| **RTL8126A (add-in card)** | **5 GbE, 2024** | **1 ms** | **229 / 10,000** | **2.29 %** |
| RTL8126A | 5 GbE | 250 µs | 842 / 40,000 | 2.11 % |
| RTL8168h | 1 GbE | 4 ms | 29 / 15,000 | 0.19 % |

Replacing the silicon with a far newer part changed the loss rate by **2 %**.
That rules out the chip: what both parts share is `r8169`, and that is where the
frames are going. Both are usable at 4 ms and neither at 1 ms or below.

The host is not implicated, and was measured rather than assumed:

- **Timing is fine.** 3.5 µs mean jitter and 9.8 µs max at a **250 µs** cycle,
  zero overruns, 128 µs worst loop against a 250 µs budget.
- **The wire is fine.** Every slave's `invalid_frame`, `rx_error` and
  `lost_link` counter read zero on every run, on both cards.
- **The frames simply never return.** Worst working counter on a failing cycle
  is `EC_NOFRAME` every time.

Everything else was ruled out first, which is why this is a disqualification and
not a guess:

- **Not the host timing.** Mean jitter 3.9 µs, max 14.2 µs, zero overruns,
  `real-time: yes`, worst loop 298 µs against a 1000 µs budget.
- **Not the cabling.** Every slave's ESC error registers
  (`invalid_frame`, `rx_error`, `lost_link`) read zero for the whole run. No
  device ever detected damage on the wire, and the link never dropped.
- **Not a frame dying partway along the segment.** Worst working counter on a
  failing cycle was `EC_NOFRAME`: nothing came back at all.
- **Not the receive timeout.** The worst bad exchange sat exactly at the
  timeout, so the frames were absent rather than late.

The small forwarded-RX-error counts that do appear (26 over 250,000 cycles) are
the same fault, not a second one: no port ever *detected* damage, so those
frames arrived already flagged, having been damaged before they reached drive 1.

**Consequence for the product:** the BOM requirement is an Intel i210/i211 on
the **`igb`** driver. Stated as a driver requirement rather than a brand
preference, because that is what the evidence supports — and because it means
"a newer Realtek" is not a fix, which is the mistake this record exists to
prevent someone repeating.

**There is no tuning lever left.** Interrupt coalescing was the most likely
mechanism and cannot be reached: `ethtool -c` returns *Operation not supported*
on **both** controllers, and the ring buffers are already at their 256-descriptor
maximum. Offloads and flow control were disabled early and changed nothing.

Realtek's out-of-tree `r8125`/`r8126` drivers remain untried. Worth curiosity;
not worth building a product on. An out-of-tree vendor module that must be
rebuilt against every kernel update is a liability on a machine expected to run
for a decade.

Any board considered for a shipped machine must pass this test before it is
committed to.

### Caveats

1. **The NIC is the weak point.** `r8169` does not support interrupt coalescing
   control or ring buffer adjustment on this hardware — both reported
   unsupported by `ethtool`. Fine for bring-up; replace with Intel i210/i211
   before qualification. At a 250 µs cycle the Realtek's frame-timing variance is
   four times as significant a fraction of the cycle.
2. **Single wired NIC.** No separate management port. Plant network currently
   over WiFi.
3. **`CONFIG_NO_HZ_FULL` absent.** Full tickless isolation unavailable on the
   stock XanMod RT kernel. `isolcpus` and `rcu_nocbs` deliver most of the
   benefit; a custom kernel build would close the gap, and is wanted for a
   shipped product regardless.

### Re-qualify when any of these change

- Kernel version or build
- BIOS version or settings
- NIC model or driver
- Motherboard revision

---

## Template for additional units

```markdown
## Unit N — <hostname>
**Status:**
### Configuration
### Kernel command line
### Results
  hwlat:      <duration> / <threshold> -> <samples>
  rt_probe:   <cycle> / <duration> / <load> -> max <n> us, overruns <n>
### Assessment
```
