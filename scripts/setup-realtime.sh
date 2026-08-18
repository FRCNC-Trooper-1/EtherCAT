#!/usr/bin/env bash
# setup-realtime.sh — Apply runtime real-time tuning.
#
# Applies the settings from docs/02-pc-realtime-setup.md §5 that do not survive
# a reboot. Install as a systemd oneshot unit to make them permanent (docs/02 §5.5).
#
# Does NOT change: BIOS settings, the kernel command line, or the kernel itself.
# Those are covered in docs/02 §3 and §4 and must be done separately.
#
#   sudo ./scripts/setup-realtime.sh <ethercat-interface> [isolated-cpu]
#
# Copyright (c) 2026 Front Range CNC. BSD 3-Clause.

set -euo pipefail

ETH="${1:-}"
RT_CPU="${2:-2}"

if [[ $EUID -ne 0 ]]; then
    echo "This script must run as root: sudo $0 <interface> [isolated-cpu]" >&2
    exit 1
fi

if [[ -z "$ETH" ]]; then
    echo "Usage: sudo $0 <ethercat-interface> [isolated-cpu]" >&2
    echo "Example: sudo $0 <nic> 2" >&2
    exit 1
fi

if [[ ! -e "/sys/class/net/${ETH}" ]]; then
    echo "Interface ${ETH} does not exist." >&2
    exit 1
fi

# Housekeeping CPUs are everything except the isolated ones. On the reference
# 4-core target that is CPUs 0-1, i.e. affinity mask 0x3.
HOUSEKEEPING_MASK="${HOUSEKEEPING_MASK:-3}"
# NIC IRQs land on CPU 1 — adjacent to the control core, sharing L3 but not L2.
NIC_IRQ_MASK="${NIC_IRQ_MASK:-2}"

log() { printf '  %s\n' "$1"; }

echo "=== Real-time tuning: interface=${ETH} isolated-cpu=${RT_CPU} ==="

# ------------------------------------------------------------- governor ----
echo "[1/6] CPU frequency governor"
if compgen -G "/sys/devices/system/cpu/cpu*/cpufreq/scaling_governor" >/dev/null; then
    for g in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
        echo performance > "$g" 2>/dev/null || true
    done
    log "set to performance"
else
    log "no cpufreq control exposed — skipping"
fi

# ----------------------------------------------------------- irqbalance ----
echo "[2/6] irqbalance"
if systemctl is-active --quiet irqbalance 2>/dev/null; then
    systemctl disable --now irqbalance
    log "disabled"
else
    log "already inactive"
fi

# ------------------------------------------------------------------ irq ----
echo "[3/6] IRQ affinity"
MOVED=0
for irq in /proc/irq/[0-9]*; do
    [[ -w "${irq}/smp_affinity" ]] || continue
    if echo "$HOUSEKEEPING_MASK" > "${irq}/smp_affinity" 2>/dev/null; then
        MOVED=$((MOVED+1))
    fi
done
log "moved ${MOVED} IRQs to housekeeping CPUs (mask 0x${HOUSEKEEPING_MASK})"

PINNED=0
while read -r irqnum; do
    [[ -n "$irqnum" ]] || continue
    if echo "$NIC_IRQ_MASK" > "/proc/irq/${irqnum}/smp_affinity" 2>/dev/null; then
        PINNED=$((PINNED+1))
    fi
done < <(grep -E "[[:space:]]${ETH}(-|\$|[[:space:]])" /proc/interrupts 2>/dev/null \
         | awk -F: '{gsub(/ /,"",$1); print $1}')
log "pinned ${PINNED} ${ETH} IRQs to mask 0x${NIC_IRQ_MASK}"

# ------------------------------------------------------------------ nic ----
echo "[4/6] NIC tuning (${ETH})"
if ! command -v ethtool >/dev/null 2>&1; then
    log "ethtool not installed — skipping (run scripts/install-deps.sh)"
else
    # Each of these is unsupported on some hardware; failures are non-fatal.
    ethtool -C "$ETH" rx-usecs 0 tx-usecs 0 adaptive-rx off adaptive-tx off 2>/dev/null \
        && log "coalescing disabled" || log "coalescing: not supported"

    ethtool -K "$ETH" gro off gso off tso off lro off 2>/dev/null \
        && log "offloads disabled" || log "offloads: partially unsupported"

    ethtool -G "$ETH" rx 128 tx 128 2>/dev/null \
        && log "ring buffers set to 128" || log "ring buffers: not adjustable"

    ethtool -A "$ETH" rx off tx off autoneg off 2>/dev/null \
        && log "flow control disabled" || log "flow control: not supported"
fi

# Fieldbus port carries no IP traffic.
ip addr flush dev "$ETH" 2>/dev/null || true
ip link set "$ETH" up
log "IP addresses flushed, link up"

# ----------------------------------------------------------- scheduling ----
echo "[5/6] Real-time scheduling"
if [[ "${DISABLE_RT_THROTTLING:-1}" == "1" ]]; then
    echo -1 > /proc/sys/kernel/sched_rt_runtime_us
    log "sched_rt_runtime_us = -1 (throttling disabled)"
else
    log "RT throttling left at default (DISABLE_RT_THROTTLING=0)"
fi

# ------------------------------------------------------------------ thp ----
echo "[6/6] Transparent hugepages"
if [[ -w /sys/kernel/mm/transparent_hugepage/enabled ]]; then
    echo never > /sys/kernel/mm/transparent_hugepage/enabled
    echo never > /sys/kernel/mm/transparent_hugepage/defrag 2>/dev/null || true
    log "disabled"
else
    log "not configurable at runtime — set transparent_hugepage=never on the kernel command line"
fi

echo
echo "=== Runtime tuning applied ==="
echo
echo "This does NOT cover BIOS settings or the kernel command line."
echo "Verify the whole picture with:"
echo "  ./scripts/check-realtime.sh ${ETH}"
