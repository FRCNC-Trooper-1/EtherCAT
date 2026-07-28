# 02 — Control PC Real-Time Setup

> **Goal of this stage:** a machine that can wake a thread every cycle, on time,
> every time, for weeks — with a worst-case wake-up latency small enough that a
> servo loop never misses a frame.
>
> **Exit criterion:** 12 hours of `cyclictest` under load with a maximum latency
> below your budget (see [§7](#7-acceptance-criteria)), and `hwlatdetect` showing
> zero hardware-induced stalls.

Nothing in this document is optional for a commercial machine. A CNC that drops
an EtherCAT frame under load produces a following error mid-cut, and on a real
machine that means a scrapped part, a broken tool, or worse.

---

## 1. The one number that matters

Everything below exists to bound **worst-case latency**, not average latency.

Average latency is irrelevant. A control loop that is on time 99.99% of the time
and 400 µs late once an hour will fault a servo drive once an hour. The entire
discipline of real-time tuning is about the tail of the distribution.

Your budget:

| Cycle time | Max acceptable jitter | Typical use |
|---|---|---|
| 1 ms | < 100 µs | Entry-level, forgiving drives, good starting point |
| 500 µs | < 50 µs | Solid industrial multi-axis |
| 250 µs | < 25 µs | High-performance contouring |

**Start at 1 ms.** Get the machine cutting. Reduce cycle time later, once you
have a latency baseline you trust and drives you know are healthy. Chasing
250 µs on day one is how projects die.

---

## 2. Kernel: PREEMPT_RT is now mainline

This is the single most important recent change in this space, and most guides
online are out of date.

**As of Linux 6.12 (released November 2024), the PREEMPT_RT patch set is merged
into the mainline kernel.** You no longer need to hunt for an out-of-tree patch
matching your exact kernel version, and you no longer need a vendor's RT kernel
subscription. You build a stock kernel with `CONFIG_PREEMPT_RT=y`.

For a **commercial product** this matters twice over:

- It removes the maintenance treadmill of rebasing RT patches.
- It sidesteps the licensing question around Ubuntu's "Real-time Ubuntu"
  (`linux-image-realtime`), which is delivered through Ubuntu Pro and is **free
  only for personal use** — commercial deployment on customer machines requires
  a paid Ubuntu Pro subscription per machine. Building your own mainline RT
  kernel avoids that recurring per-unit cost entirely.

> The kernel is GPLv2 either way. That does not affect your BSD-licensed
> userspace application — see [`08-licensing.md`](08-licensing.md) for why the
> kernel/userspace boundary keeps your code proprietary.

### 2.1 Verify what you are running

```bash
# Authoritative — the config embedded in the running kernel image.
zcat /proc/config.gz | grep CONFIG_PREEMPT_RT
# expect: CONFIG_PREEMPT_RT=y

# Fallback if /proc/config.gz is unavailable (needs CONFIG_IKCONFIG_PROC=y):
grep CONFIG_PREEMPT_RT "/boot/config-$(uname -r)"

# Secondary signal — generated at build time from CONFIG_PREEMPT_RT:
uname -v | grep PREEMPT_RT
```

`PREEMPT_DYNAMIC` is **not** `PREEMPT_RT`. A stock Ubuntu kernel reports
`PREEMPT_DYNAMIC` and will not meet the latency budget above.

> **Do not test `/sys/kernel/realtime`.** Widely repeated advice says to check
> that file for `1`. It was never in mainline — it came from the out-of-tree RT
> patchset and disappeared once RT kernels became mainline-based in 6.12. On any
> modern RT kernel it is simply absent, and treating that as failure gives a
> false negative. Use the kernel config.

### 2.2 Building a mainline RT kernel (Ubuntu 24.04)

```bash
sudo apt install -y build-essential libncurses-dev bison flex libssl-dev \
                    libelf-dev bc dwarves rsync zstd fakeroot

# Use a 6.12+ longterm kernel. 6.12.x is the current LTS with RT merged.
KVER=6.12.30
cd /usr/src
sudo wget https://cdn.kernel.org/pub/linux/kernel/v6.x/linux-${KVER}.tar.xz
sudo tar xf linux-${KVER}.tar.xz
cd linux-${KVER}

# Start from the running config
sudo cp /boot/config-$(uname -r) .config
sudo make olddefconfig
```

Then set the RT options. Either run `sudo make menuconfig` and select
**General setup → Preemption Model → Fully Preemptible Kernel (Real-Time)**, or
set them directly:

```bash
sudo ./scripts/config --enable  PREEMPT_RT
sudo ./scripts/config --enable  HIGH_RES_TIMERS
sudo ./scripts/config --enable  NO_HZ_FULL
sudo ./scripts/config --enable  RCU_NOCB_CPU
sudo ./scripts/config --enable  CPU_ISOLATION

# Latency killers — these must be OFF in a production RT kernel.
sudo ./scripts/config --disable CPU_FREQ
sudo ./scripts/config --disable CPU_IDLE
sudo ./scripts/config --disable DEBUG_PREEMPT
sudo ./scripts/config --disable LATENCYTOP
sudo ./scripts/config --disable FTRACE          # keep ON while tuning, OFF in production
sudo ./scripts/config --disable TRANSPARENT_HUGEPAGE_ALWAYS

# Ubuntu's config ships signing keys that break out-of-tree builds
sudo ./scripts/config --disable SYSTEM_TRUSTED_KEYS
sudo ./scripts/config --disable SYSTEM_REVOCATION_KEYS

sudo make olddefconfig
sudo make -j"$(nproc)" bindeb-pkg
sudo dpkg -i ../linux-image-*.deb ../linux-headers-*.deb
```

Reboot, then confirm `cat /sys/kernel/realtime` prints `1`.

> **Production note:** pin this kernel version, build it once in CI, and ship the
> resulting `.deb` as part of your machine image. Do **not** let customer machines
> take kernel updates from the distro — an unattended kernel upgrade silently
> reverts every guarantee in this document. See
> [`docs/07-safety-and-compliance.md`](07-safety-and-compliance.md).

---

## 3. BIOS/UEFI — do this before touching software

The BIOS causes latency spikes that no amount of kernel tuning can fix, because
System Management Interrupts (SMIs) preempt the kernel itself. The CPU is stolen
by firmware and Linux never even knows it happened.

Set the following. Exact names vary by vendor; the intent is what matters.

| Setting | Value | Why |
|---|---|---|
| Hyper-Threading / SMT | **Disabled** | Sibling threads contend for execution units; adds unbounded jitter |
| Intel SpeedStep / EIST | **Disabled** | Frequency transitions stall the pipeline |
| Turbo Boost | **Disabled** | Clock changes cause timing variance; a predictable 2.1 GHz beats a variable 3.5 GHz |
| C-States (all, incl. package) | **Disabled** | Exiting a deep C-state costs tens of µs |
| P-States / OS control of power | **Disabled** / "Max Performance" | Same reason |
| ASPM (PCIe power mgmt) | **Disabled** | Link wake-up latency |
| USB Legacy Support / USB emulation | **Disabled** | Classic SMI source — a plugged-in USB device can generate SMIs forever |
| Fan control mode | Fixed / max | Firmware thermal polling generates SMIs |
| Memory power management | Disabled | |
| Virtualization (VT-d/VT-x) | Disabled unless required | Reduces trap overhead |
| Watchdog timers | Disabled | Can trigger SMIs |

Then **verify** with `hwlatdetect` (§6.2). Do not trust the BIOS screen — trust
the measurement. Boards with hidden SMI sources are common, and some are simply
unfit for real-time work. Discovering that during hardware qualification is
cheap; discovering it after you have shipped 40 machines is not.

---

## 4. Kernel command line

Reserve CPUs so the OS never schedules anything on them, then run your control
thread there.

On this project's 4-core reference target, the split is:

- **CPU 0, 1** — housekeeping: OS, HMI, planner, logging, interrupts
- **CPU 2** — isolated: the EtherCAT cyclic task
- **CPU 3** — isolated: reserved (second cyclic task, or headroom)

Edit `/etc/default/grub`:

```bash
GRUB_CMDLINE_LINUX_DEFAULT="isolcpus=managed_irq,domain,2,3 nohz_full=2,3 rcu_nocbs=2,3 rcu_nocb_poll irqaffinity=0,1 intel_pstate=disable processor.max_cstate=1 intel_idle.max_cstate=0 idle=poll nosoftlockup nmi_watchdog=0 nowatchdog skew_tick=1 tsc=reliable mce=off audit=0 transparent_hugepage=never"
```

```bash
sudo update-grub && sudo reboot
```

What each flag buys you:

| Flag | Effect |
|---|---|
| `isolcpus=managed_irq,domain,2,3` | Removes CPUs 2–3 from the scheduler's load balancing and steers managed IRQs away |
| `nohz_full=2,3` | Stops the 1 kHz scheduler tick on those cores — removes a periodic ~5 µs interruption |
| `rcu_nocbs=2,3` + `rcu_nocb_poll` | Offloads RCU callback processing off the isolated cores |
| `irqaffinity=0,1` | Default IRQ handling stays on housekeeping cores |
| `intel_pstate=disable` | Frequency stays fixed |
| `processor.max_cstate=1`, `intel_idle.max_cstate=0` | Blocks deep sleep states (belt-and-braces with the BIOS) |
| `idle=poll` | Idle cores spin instead of sleeping — **costs significant power and heat**, so validate thermals; drop it if C-states are genuinely disabled |
| `nosoftlockup`, `nmi_watchdog=0`, `nowatchdog` | Removes watchdog timer interrupts |
| `skew_tick=1` | Staggers remaining timer ticks across cores to avoid lock convoys |
| `tsc=reliable` | Avoids clocksource re-validation stalls |
| `transparent_hugepage=never` | THP compaction causes multi-millisecond stalls — a notorious RT killer |

> `mce=off` disables machine-check reporting. It removes an SMI source but also
> hides genuine hardware faults. Consider `mce=ignore_ce` instead for production
> machines where you want ECC errors surfaced. Make this call deliberately.

### 4.1 Check the kernel config first — some parameters are silently ignored

**`nohz_full=` only works if the kernel was built with `CONFIG_NO_HZ_FULL=y`.**
If it was not, the parameter appears on the command line, `/proc/cmdline` shows
it, and it **does nothing at all**. There is no warning.

This is not a corner case. Several popular prebuilt RT kernels — including
**XanMod's `-rt` builds** — ship `CONFIG_NO_HZ_IDLE=y` instead, so tickless
isolation is unavailable no matter what you put on the command line.

Check before you tune:

```bash
# /proc/config.gz is authoritative — it is embedded in the running kernel.
zcat /proc/config.gz | grep -E 'CONFIG_NO_HZ|CONFIG_PREEMPT_RT|CONFIG_HZ=|CONFIG_CPU_ISOLATION|CONFIG_RCU_NOCB_CPU|CONFIG_HWLAT_TRACER'
```

| Symbol | Needed for |
|---|---|
| `CONFIG_PREEMPT_RT=y` | Real-time preemption — **mandatory** |
| `CONFIG_NO_HZ_FULL=y` | `nohz_full=` — without it the parameter is ignored |
| `CONFIG_CPU_ISOLATION=y` | `isolcpus=` |
| `CONFIG_RCU_NOCB_CPU=y` | `rcu_nocbs=` |
| `CONFIG_HWLAT_TRACER=y` | `hwlatdetect` |
| `CONFIG_IKCONFIG_PROC=y` | `/proc/config.gz` itself |

If `CONFIG_NO_HZ_FULL` is missing, you have two choices: accept it (`isolcpus`
and `rcu_nocbs` still work and deliver most of the benefit), or build your own
kernel. For a shipped product you will want your own pinned kernel build anyway —
see §2.2.

> **Do not use `/sys/kernel/realtime` to detect an RT kernel.** That file was
> never part of mainline; it existed only in the out-of-tree patchset and
> disappeared from RT kernels once they became mainline-based in 6.12. Its
> absence proves nothing. There is currently no sysfs interface in mainline that
> reports the preemption model — use `/proc/config.gz`, or `PREEMPT_RT` in
> `uname -v` (which is generated at build time from `CONFIG_PREEMPT_RT`).

### 4.2 AMD processors

The command line in §4 is Intel-specific in two places. On AMD,
`intel_pstate=disable` and `intel_idle.max_cstate=0` are **silently
meaningless** — neither driver ever loads. There is no `amd_idle` driver at all;
AMD uses `acpi_idle`.

Replace those two flags with:

```bash
# AMD equivalent of the frequency/idle portion
amd_pstate=passive amd_prefcore=disable processor.max_cstate=1 iommu.passthrough=1
```

| Flag | Effect |
|---|---|
| `amd_pstate=passive` | The governor requests the performance level instead of the hardware choosing autonomously. The common default is `active` (EPP), which is the **least** deterministic mode. `amd_pstate=disable` falls back to `acpi-cpufreq`, also fine. |
| `amd_prefcore=disable` | Stops firmware biasing work toward "preferred" cores — irrelevant once threads are pinned, but removes a variable |
| `processor.max_cstate=1` | Drives `acpi_idle`, and **works on both vendors**. Use `1`, not `0` — the kernel clamps `0` to `1` anyway |
| `iommu.passthrough=1` | Bypasses DMA translation. Lazy IOTLB invalidation (the common default) batches flushes, and those flushes are latency spikes on the NIC's DMA path |

Also worth setting at runtime, since boost causes clock transitions:

```bash
echo 0 | sudo tee /sys/devices/system/cpu/cpufreq/boost      # flatter clock beats higher peak
cat /sys/devices/system/cpu/amd_pstate/status                # expect: passive
```

> ⚠️ **You cannot count SMIs on AMD.** The SMI counter (`MSR 0x34`,
> `perf stat -e msr/smi/`, turbostat's `SMI` column) is **Intel-only**. AMD
> platforms absolutely do take SMIs — BMC, thermal, and fTPM are common sources,
> and Ryzen fTPM in particular has a documented reputation for periodic stalls —
> you simply cannot enumerate them. **Measure the symptom instead** with
> `hwlatdetect`, `osnoise`, and `timerlat`. This makes `hwlatdetect` more
> important on AMD, not less.

### 4.3 Verify after reboot

```bash
cat /proc/cmdline
cat /sys/devices/system/cpu/isolated     # expect: 2-3
cat /sys/devices/system/cpu/nohz_full    # expect: 2-3, or empty if CONFIG_NO_HZ_FULL is off

# The TSC must stay the clocksource. A demotion to hpet/acpi_pm makes every
# clock read far more expensive and RT latency collapses.
cat /sys/devices/system/clocksource/clocksource0/current_clocksource   # expect: tsc
```

---

## 5. Runtime tuning

These do not survive a reboot on their own — `scripts/setup-realtime.sh` in this
repo applies them, and the systemd unit in [§5.5](#55-make-it-persistent) makes
it permanent.

### 5.1 CPU governor

```bash
# Only applies if CPU_FREQ was left enabled; harmless otherwise.
for g in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
    echo performance | sudo tee "$g" >/dev/null
done
```

### 5.2 Stop irqbalance and pin interrupts

`irqbalance` will happily migrate an interrupt onto your isolated core at the
worst possible moment.

```bash
sudo systemctl disable --now irqbalance
```

Move every interrupt to the housekeeping cores, then pin the EtherCAT NIC's
interrupt deliberately:

```bash
# Push all IRQs to CPUs 0-1 (mask 0x3). Some are unmovable; errors are expected.
for irq in /proc/irq/[0-9]*; do
    echo 3 | sudo tee "$irq/smp_affinity" >/dev/null 2>&1 || true
done

# Then pin the EtherCAT NIC IRQ to CPU 1 (mask 0x2), adjacent to but not on
# the isolated control core.
ETH=enp3s0     # <-- your dedicated EtherCAT interface
for irq in $(grep -E "${ETH}" /proc/interrupts | awk -F: '{print $1}' | tr -d ' '); do
    echo 2 | sudo tee "/proc/irq/${irq}/smp_affinity" >/dev/null
done
```

> **Design note.** SOEM on Linux uses a raw `AF_PACKET` socket, so frame TX and
> RX happen in the context of *your* thread plus the NIC's softirq — not in a
> kernel EtherCAT driver. Keeping the NIC IRQ on a housekeeping core adjacent to
> the control core (shared L3, not shared L2) is the configuration that measures
> best on most Intel parts. It is worth testing both ways on your final hardware
> and keeping the numbers.

### 5.3 NIC tuning

Every offload and coalescing feature exists to raise throughput by adding
latency. For EtherCAT you want the opposite trade in every case.

```bash
ETH=enp3s0

# Disable interrupt coalescing — we want the frame the instant it lands.
sudo ethtool -C "$ETH" rx-usecs 0 tx-usecs 0 rx-frames 1 tx-frames 1 adaptive-rx off adaptive-tx off

# Disable offloads: they reorder, batch, and delay frames.
sudo ethtool -K "$ETH" gro off gso off tso off lro off rx-vlan-offload off tx-vlan-offload off

# Small ring buffers — deep queues only add latency for a 1-frame-per-cycle protocol.
sudo ethtool -G "$ETH" rx 128 tx 128

# No flow control. A pause frame would stall the cyclic exchange.
sudo ethtool -A "$ETH" rx off tx off autoneg off

# Fixed 100 Mbit full duplex. EtherCAT slaves are 100BASE-TX; forcing the link
# removes autonegotiation renegotiation events.
sudo ethtool -s "$ETH" speed 100 duplex full autoneg off
```

Also keep the EtherCAT interface out of the OS network stack entirely — no IP
address, no DHCP, no NetworkManager, no IPv6 router solicitations:

```bash
sudo tee /etc/NetworkManager/conf.d/99-ethercat.conf >/dev/null <<EOF
[keyfile]
unmanaged-devices=interface-name:${ETH}
EOF
sudo systemctl reload NetworkManager
sudo ip addr flush dev "$ETH"
sudo ip link set "$ETH" up
```

### 5.4 Real-time scheduling limits

The kernel throttles `SCHED_FIFO` tasks to 95% of each period by default, as a
safety net against a runaway RT thread locking the machine.

```bash
# Allow RT tasks 100% of the CPU. Only safe because the control task is
# isolated on its own core and provably bounded.
echo -1 | sudo tee /proc/sys/kernel/sched_rt_runtime_us

# Let non-root run RT threads and lock memory (for development).
sudo tee /etc/security/limits.d/99-realtime.conf >/dev/null <<'EOF'
@realtime   -   rtprio      99
@realtime   -   memlock     unlimited
@realtime   -   nice        -20
EOF
sudo groupadd -f realtime && sudo usermod -aG realtime "$USER"
```

> Setting `sched_rt_runtime_us` to `-1` means a bug in your cyclic loop can hard-hang
> the machine. That is an acceptable trade on an isolated core in production, but
> keep the default while you are still developing the loop.

### 5.5 Make it persistent

```bash
sudo cp scripts/setup-realtime.sh /usr/local/sbin/
sudo tee /etc/systemd/system/ethercat-tuning.service >/dev/null <<'EOF'
[Unit]
Description=EtherCAT real-time system tuning
After=network-pre.target
Wants=network-pre.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/local/sbin/setup-realtime.sh

[Install]
WantedBy=multi-user.target
EOF
sudo systemctl enable ethercat-tuning.service
```

---

## 6. Measure — do not assume

Tuning you have not measured is superstition. Two tools, both mandatory.

```bash
sudo apt install -y rt-tests stress-ng
```

### 6.1 `cyclictest` — scheduler wake-up latency

```bash
# Quick smoke test (5 minutes), on the isolated core:
sudo cyclictest -m -S -p 90 -i 1000 -d 0 -h 400 -D 5m -a 2 -t 1

# The test that actually counts: 12 hours, under heavy load on the
# housekeeping cores. An idle machine tells you nothing.
sudo stress-ng --cpu 2 --io 2 --vm 2 --vm-bytes 1G --taskset 0,1 &
sudo cyclictest -m -S -p 90 -i 1000 -d 0 -h 400 -D 12h -a 2 -t 1 \
     --histfile=latency/cyclictest-12h.txt
```

Flags: `-m` locks memory, `-p 90` sets RT priority, `-i 1000` sets a 1000 µs
interval matching your target cycle, `-a 2` pins to CPU 2, `-h 400` builds a
histogram, `-D` sets duration.

**Read the `Max` column. Ignore `Avg`.**

### 6.2 `hwlatdetect` — firmware/SMI stalls

This is the test that catches a bad motherboard. It disables interrupts and
watches for time disappearing — time that can only have been stolen by SMM
firmware.

```bash
sudo hwlatdetect --duration=30m --threshold=10us
```

**Any non-zero sample count is a hardware problem, not a software one.** Go back
to §3, or reject the board. No kernel tuning can recover time stolen by SMIs.
Qualify every motherboard model you intend to ship with this test before you
commit to it in the BOM.

---

## 7. Acceptance criteria

Record these in `docs/hardware-qualification/` for each board model you ship.

| Test | 1 ms cycle | 500 µs cycle | 250 µs cycle |
|---|---|---|---|
| `cyclictest` max, 12 h under load | < 100 µs | < 50 µs | < 25 µs |
| `hwlatdetect` samples, 30 min | 0 | 0 | 0 |
| EtherCAT working-counter errors, 24 h | 0 | 0 | 0 |
| Lost/late frames, 24 h | 0 | 0 | 0 |

If `cyclictest` max is above budget, work through, in order: BIOS SMI sources
(§3), `hwlatdetect` results, IRQ placement (§5.2), THP and other memory stalls,
then NIC configuration (§5.3).

---

## 8. Application-side requirements

The tuning above is wasted if the control process does not hold up its end.
These are implemented in `src/rt/` and covered in
[`05-motion-architecture.md`](05-motion-architecture.md):

- **`mlockall(MCL_CURRENT | MCL_FUTURE)`** — a page fault in the cyclic loop is
  a missed cycle.
- **Pre-fault the stack and heap** at startup; touch every page you will ever use.
- **`SCHED_FIFO`, priority 80–90** — above the NIC softirq threads, below nothing
  that matters.
- **Pin to the isolated core** with `pthread_setaffinity_np`.
- **Zero allocation in the cyclic path.** No `malloc`, no `new`, no `std::string`,
  no `std::vector` growth, no exceptions, no logging that touches a filesystem.
- **`clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ...)`** against an absolute
  next-wake time — never a relative sleep, which accumulates drift.
- **No unbounded locks** between the RT thread and anything else. Communication
  with the planner goes through a lock-free single-producer/single-consumer ring.

---

## 9. What this looks like when it is working

```
$ cat /sys/kernel/realtime
1
$ cat /sys/devices/system/cpu/isolated
2-3
$ sudo hwlatdetect --duration=30m --threshold=10us
...
Samples exceeding threshold: 0
$ sudo cyclictest -m -S -p 90 -i 1000 -a 2 -t 1 -D 12h
T: 0 (  4021) P:90 I:1000 C:43200000 Min:  2 Act:  4 Avg:  4 Max:  18
```

`Max: 18` µs against a 1000 µs cycle is a healthy industrial machine. That is the
baseline to record, and to re-verify on every hardware or kernel change.

---

**Next:** [`03-ethercat-bringup.md`](03-ethercat-bringup.md) — building SOEM and
getting the bus to `OPERATIONAL`.
