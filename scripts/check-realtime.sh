#!/usr/bin/env bash
# check-realtime.sh — Audit a machine's real-time readiness for EtherCAT.
#
# Read-only. Makes no changes. Run this FIRST, on the real control PC, to see
# what still needs doing before Phase 1 can be signed off.
#
#   ./scripts/check-realtime.sh [ethercat-interface]
#
# Copyright (c) 2026 Front Range CNC. BSD 3-Clause.

set -uo pipefail

ETH="${1:-}"

PASS=0
WARN=0
FAIL=0

c_red()   { printf '\033[31m%s\033[0m' "$1"; }
c_grn()   { printf '\033[32m%s\033[0m' "$1"; }
c_yel()   { printf '\033[33m%s\033[0m' "$1"; }
c_bold()  { printf '\033[1m%s\033[0m'  "$1"; }

ok()   { printf '  [%s] %s\n' "$(c_grn PASS)" "$1"; PASS=$((PASS+1)); }
warn() { printf '  [%s] %s\n' "$(c_yel WARN)" "$1"; WARN=$((WARN+1)); }
bad()  { printf '  [%s] %s\n' "$(c_red FAIL)" "$1"; FAIL=$((FAIL+1)); }
info() { printf '         %s\n' "$1"; }

section() { printf '\n%s\n' "$(c_bold "$1")"; }

printf '%s\n' "$(c_bold '=== EtherCAT Real-Time Readiness Check ===')"
printf 'host: %s   kernel: %s   date: %s\n' "$(hostname)" "$(uname -r)" "$(date -Is)"

# ---------------------------------------------------------------- kernel ----
section "1. Kernel"

# Detect PREEMPT_RT using several signals, most authoritative first.
#
# NOTE: /sys/kernel/realtime came from the OUT-OF-TREE RT patchset. Kernels
# built from mainline PREEMPT_RT (merged in 6.12) generally do NOT create it,
# so its absence proves nothing on a modern kernel. The kernel config is the
# only definitive source.
RT_VERDICT="unknown"

KCONFIG=""
if [[ -r "/boot/config-$(uname -r)" ]]; then
    KCONFIG="/boot/config-$(uname -r)"
fi

if [[ -n "$KCONFIG" ]]; then
    if grep -q '^CONFIG_PREEMPT_RT=y' "$KCONFIG"; then
        ok "CONFIG_PREEMPT_RT=y in ${KCONFIG}"
        RT_VERDICT="rt"
    else
        bad "CONFIG_PREEMPT_RT is not set in ${KCONFIG}"
        RT_VERDICT="not-rt"
    fi
elif [[ -r /proc/config.gz ]]; then
    if zcat /proc/config.gz 2>/dev/null | grep -q '^CONFIG_PREEMPT_RT=y'; then
        ok "CONFIG_PREEMPT_RT=y in /proc/config.gz"
        RT_VERDICT="rt"
    else
        bad "CONFIG_PREEMPT_RT is not set in /proc/config.gz"
        RT_VERDICT="not-rt"
    fi
else
    info "Kernel config not readable — falling back to the version string"
fi

# Version string: generated at build time from the preemption model, so a
# reliable secondary signal when the config is unavailable.
case "$(uname -v)" in
    *PREEMPT_RT*)
        ok "Version string reports PREEMPT_RT"
        [[ "$RT_VERDICT" == "unknown" ]] && RT_VERDICT="rt"
        ;;
    *PREEMPT_DYNAMIC*)
        bad "Version string reports PREEMPT_DYNAMIC — this is NOT PREEMPT_RT"
        [[ "$RT_VERDICT" == "unknown" ]] && RT_VERDICT="not-rt"
        ;;
    *PREEMPT*)
        warn "Version string reports PREEMPT but not PREEMPT_RT"
        [[ "$RT_VERDICT" == "unknown" ]] && RT_VERDICT="not-rt"
        ;;
    *)
        warn "No preemption model in the version string"
        ;;
esac

# Legacy indicator. Informational only — never a failure on its own.
if [[ -e /sys/kernel/realtime ]]; then
    info "/sys/kernel/realtime = $(cat /sys/kernel/realtime 2>/dev/null) (legacy out-of-tree indicator)"
else
    info "/sys/kernel/realtime absent — expected on mainline-RT kernels, not a fault"
fi

if [[ "$RT_VERDICT" == "not-rt" ]]; then
    info "PREEMPT_RT is mainline since Linux 6.12. See docs/02-pc-realtime-setup.md §2."
fi

# CONFIG_HZ affects the tick rate on non-isolated cores. With hrtimers driving
# the cyclic task and nohz_full on the isolated cores it is not critical, but
# a higher value slightly improves housekeeping-core responsiveness.
if [[ -n "$KCONFIG" ]]; then
    HZ=$(awk -F= '/^CONFIG_HZ=/{print $2}' "$KCONFIG" 2>/dev/null)
    if [[ -n "$HZ" ]]; then
        if (( HZ >= 1000 )); then
            ok "CONFIG_HZ=${HZ}"
        else
            info "CONFIG_HZ=${HZ} (1000 is the common RT choice; minor with nohz_full + hrtimers)"
        fi
    fi
fi

KMAJ=$(uname -r | cut -d. -f1)
KMIN=$(uname -r | cut -d. -f2)
if (( KMAJ > 6 || (KMAJ == 6 && KMIN >= 12) )); then
    ok "Kernel ${KMAJ}.${KMIN} supports mainline PREEMPT_RT (>= 6.12)"
else
    warn "Kernel ${KMAJ}.${KMIN} predates mainline PREEMPT_RT (6.12)"
fi

# ------------------------------------------------------------ virtualized ----
section "2. Platform"

VIRT="none"
if command -v systemd-detect-virt >/dev/null 2>&1; then
    VIRT=$(systemd-detect-virt 2>/dev/null || echo none)
fi
if [[ "$VIRT" != "none" ]]; then
    bad "Running under virtualization: ${VIRT}"
    info "A VM cannot meet EtherCAT latency requirements. Use bare metal."
else
    ok "Running on bare metal"
fi

# ------------------------------------------------------------------- cpu ----
section "3. CPU isolation"

CMDLINE=$(cat /proc/cmdline)

ISOL=$(cat /sys/devices/system/cpu/isolated 2>/dev/null || echo "")
if [[ -n "$ISOL" ]]; then
    ok "Isolated CPUs: ${ISOL}"
else
    bad "No isolated CPUs — add isolcpus= to the kernel command line"
fi

NOHZ=$(cat /sys/devices/system/cpu/nohz_full 2>/dev/null || echo "")
if [[ -n "$NOHZ" && "$NOHZ" != "(null)" ]]; then
    ok "nohz_full CPUs: ${NOHZ}"
else
    warn "nohz_full not set — the 1 kHz scheduler tick still interrupts control cores"
fi

for opt in rcu_nocbs irqaffinity; do
    if grep -qw -- "${opt}=[^ ]*" <<<"$CMDLINE" || grep -q -- "${opt}=" <<<"$CMDLINE"; then
        ok "${opt} present on kernel command line"
    else
        warn "${opt} not set on kernel command line"
    fi
done

if grep -q "nosoftlockup" <<<"$CMDLINE"; then ok "nosoftlockup set"; else warn "nosoftlockup not set"; fi
if grep -q "nowatchdog"   <<<"$CMDLINE"; then ok "nowatchdog set";   else warn "nowatchdog not set"; fi

NCPU=$(nproc)
if (( NCPU >= 4 )); then
    ok "${NCPU} CPUs available"
else
    warn "${NCPU} CPUs — 4 or more recommended (2 housekeeping + 2 isolated)"
fi

# ------------------------------------------------------------- frequency ----
section "4. Power and frequency"

GOV_FILE=/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
if [[ -r "$GOV_FILE" ]]; then
    GOV=$(cat "$GOV_FILE")
    if [[ "$GOV" == "performance" ]]; then
        ok "CPU governor: performance"
    else
        bad "CPU governor: ${GOV} — set to 'performance'"
    fi
else
    ok "No cpufreq control exposed (CPU_FREQ disabled, or fixed by firmware)"
fi

if grep -q "intel_pstate=disable" <<<"$CMDLINE"; then
    ok "intel_pstate disabled"
else
    warn "intel_pstate not explicitly disabled"
fi

if grep -qE "processor.max_cstate=[01]" <<<"$CMDLINE"; then
    ok "Deep C-states blocked via kernel command line"
else
    warn "processor.max_cstate not restricted — verify C-states are off in BIOS"
fi

# SMT / hyper-threading
SMT_FILE=/sys/devices/system/cpu/smt/control
if [[ -r "$SMT_FILE" ]]; then
    SMT=$(cat "$SMT_FILE")
    case "$SMT" in
        off|notsupported|notimplemented) ok "SMT/hyper-threading: ${SMT}" ;;
        *) bad "SMT/hyper-threading is ${SMT} — disable it (BIOS preferred)" ;;
    esac
else
    THREADS_PER_CORE=$(lscpu 2>/dev/null | awk -F: '/Thread\(s\) per core/{gsub(/ /,"",$2); print $2}')
    if [[ "$THREADS_PER_CORE" == "1" ]]; then
        ok "1 thread per core — SMT effectively off"
    else
        warn "SMT state could not be confirmed"
    fi
fi

# ------------------------------------------------------------------ mem ----
section "5. Memory"

THP_FILE=/sys/kernel/mm/transparent_hugepage/enabled
if [[ -r "$THP_FILE" ]]; then
    if grep -q "\[never\]" "$THP_FILE"; then
        ok "Transparent hugepages: never"
    else
        bad "Transparent hugepages enabled — THP compaction causes millisecond stalls"
        info "Add transparent_hugepage=never to the kernel command line"
    fi
else
    ok "Transparent hugepages not configurable"
fi

# ------------------------------------------------------------------ irq ----
section "6. Interrupts"

if systemctl is-active --quiet irqbalance 2>/dev/null; then
    bad "irqbalance is running — it will migrate IRQs onto isolated cores"
    info "sudo systemctl disable --now irqbalance"
else
    ok "irqbalance not running"
fi

# ------------------------------------------------------------ scheduling ----
section "7. Real-time scheduling"

RT_RUNTIME=$(cat /proc/sys/kernel/sched_rt_runtime_us 2>/dev/null || echo "?")
if [[ "$RT_RUNTIME" == "-1" ]]; then
    ok "sched_rt_runtime_us = -1 (RT throttling disabled)"
else
    warn "sched_rt_runtime_us = ${RT_RUNTIME} — RT tasks throttled to 95%"
    info "Fine during development; set to -1 for production (see docs/02 §5.4)"
fi

RTPRIO=$(ulimit -Hr 2>/dev/null || echo 0)
if [[ "$RTPRIO" == "unlimited" ]] || (( RTPRIO >= 90 )) 2>/dev/null; then
    ok "RT priority limit: ${RTPRIO}"
else
    warn "RT priority hard limit is ${RTPRIO} — see /etc/security/limits.d/"
fi

MEMLOCK=$(ulimit -Hl 2>/dev/null || echo 0)
if [[ "$MEMLOCK" == "unlimited" ]]; then
    ok "memlock limit: unlimited"
else
    warn "memlock limit is ${MEMLOCK} KB — mlockall() may fail"
fi

# ------------------------------------------------------------------ nic ----
section "8. EtherCAT network interface"

if [[ -z "$ETH" ]]; then
    warn "No interface given — pass one, e.g. ./scripts/check-realtime.sh enp3s0"
    info "Candidates:"
    for d in /sys/class/net/*; do
        n=$(basename "$d")
        [[ "$n" == "lo" ]] && continue
        drv=$(basename "$(readlink -f "$d/device/driver" 2>/dev/null)" 2>/dev/null || echo "?")
        info "  ${n} (driver: ${drv})"
    done
else
    if [[ ! -e "/sys/class/net/${ETH}" ]]; then
        bad "Interface ${ETH} does not exist"
    else
        DRV=$(basename "$(readlink -f "/sys/class/net/${ETH}/device/driver" 2>/dev/null)" 2>/dev/null || echo "?")
        case "$DRV" in
            igb|igc|e1000e)
                ok "Interface ${ETH} driver: ${DRV} (Intel — recommended)"
                ;;
            r8169)
                warn "Interface ${ETH} driver: r8169 (Realtek RTL8111/8168 family)"
                info "Usable for development and bring-up, NOT recommended for a shipped product."
                info "The r8169 driver has higher and less predictable latency than Intel igb,"
                info "and a history of power-management quirks causing sporadic stalls."
                info "Plan a swap to an Intel i210/i211 before qualification. See docs/01 §1."
                ;;
            virtio_net|veth|tun)
                bad "Interface ${ETH} driver: ${DRV} (virtual — cannot carry EtherCAT)"
                ;;
            *)
                warn "Interface ${ETH} driver: ${DRV} (unrecognised — qualify carefully)"
                ;;
        esac

        # A product needs a dedicated fieldbus port AND a separate management
        # port. Count physical wired interfaces to catch the single-NIC case.
        WIRED=0
        for d in /sys/class/net/*; do
            n=$(basename "$d")
            [[ "$n" == "lo" ]] && continue
            [[ -e "$d/wireless" || "$n" == wl* ]] && continue
            wdrv=$(basename "$(readlink -f "$d/device/driver" 2>/dev/null)" 2>/dev/null || echo "")
            case "$wdrv" in
                virtio_net|veth|tun|"") continue ;;
            esac
            WIRED=$((WIRED + 1))
        done

        if (( WIRED >= 2 )); then
            ok "${WIRED} wired interfaces — fieldbus and management can be separated"
        else
            warn "Only ${WIRED} wired interface — no separate management port"
            info "EtherCAT requires a dedicated NIC with no IP stack. With one wired port,"
            info "the plant network must run over WiFi. Acceptable on a bench; add a second"
            info "wired NIC (M.2 or PCIe Intel i210/i211) for a shipped machine."
        fi

        # No IP address should be configured on the fieldbus port
        if ip -4 addr show dev "$ETH" 2>/dev/null | grep -q "inet "; then
            bad "Interface ${ETH} has an IPv4 address — the fieldbus port must have no IP stack"
        else
            ok "Interface ${ETH} has no IPv4 address"
        fi

        if command -v ethtool >/dev/null 2>&1; then
            CO=$(ethtool -c "$ETH" 2>/dev/null || true)
            RXU=$(awk -F: '/^rx-usecs:/{gsub(/ /,"",$2); print $2}' <<<"$CO")
            if [[ "$RXU" == "0" ]]; then
                ok "Interrupt coalescing disabled (rx-usecs 0)"
            elif [[ -n "$RXU" ]]; then
                warn "rx-usecs = ${RXU} — set to 0 to remove coalescing latency"
            fi

            OFF=$(ethtool -k "$ETH" 2>/dev/null || true)
            for feat in generic-receive-offload generic-segmentation-offload tcp-segmentation-offload; do
                if grep -q "^${feat}: on" <<<"$OFF"; then
                    warn "Offload ${feat} is on — disable it"
                fi
            done
        else
            warn "ethtool not installed — cannot audit NIC settings"
        fi
    fi
fi

# ----------------------------------------------------------------- tools ----
section "9. Tooling"

for t in cyclictest hwlatdetect stress-ng cmake gcc git; do
    if command -v "$t" >/dev/null 2>&1; then
        ok "${t} installed"
    else
        warn "${t} not installed — run scripts/install-deps.sh"
    fi
done

# --------------------------------------------------------------- summary ----
printf '\n%s\n' "$(c_bold '=== Summary ===')"
printf '  %s: %d    %s: %d    %s: %d\n\n' \
    "$(c_grn PASS)" "$PASS" "$(c_yel WARN)" "$WARN" "$(c_red FAIL)" "$FAIL"

if (( FAIL > 0 )); then
    printf '%s\n' "$(c_red 'NOT READY') — resolve every FAIL before Phase 1 sign-off."
    printf 'See docs/02-pc-realtime-setup.md\n'
    exit 1
elif (( WARN > 0 )); then
    printf '%s\n' "$(c_yel 'USABLE, NOT OPTIMAL') — review each WARN."
    printf 'Then measure: sudo hwlatdetect --duration=30m --threshold=10us\n'
    printf '              sudo cyclictest -m -S -p 90 -i 1000 -a 2 -t 1 -D 12h\n'
    exit 0
else
    printf '%s\n' "$(c_grn 'CONFIGURATION OK') — now prove it by measuring:"
    printf '  sudo hwlatdetect --duration=30m --threshold=10us   (expect 0 samples)\n'
    printf '  sudo cyclictest -m -S -p 90 -i 1000 -a 2 -t 1 -D 12h  (expect max < 100us)\n'
    exit 0
fi
