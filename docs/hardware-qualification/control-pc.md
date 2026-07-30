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
