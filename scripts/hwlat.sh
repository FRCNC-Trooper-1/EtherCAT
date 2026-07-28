#!/usr/bin/env bash
# hwlat.sh — Detect hardware/firmware-induced latency (SMIs) via the kernel's
# hwlat tracer, without the rt-tests hwlatdetect Python wrapper.
#
# hwlatdetect is a thin wrapper around this tracer and fails on some
# distributions even when the tracer itself is present and working. This script
# drives the tracer directly, so it works wherever the kernel supports it.
#
# The tracer disables interrupts for a sampling window and watches for time to
# disappear. Time that vanishes with interrupts off can only have been taken by
# something below the kernel — System Management Interrupts from firmware.
#
# ANY non-zero result is a hardware/BIOS problem. No amount of kernel tuning
# recovers time stolen by SMM firmware.
#
# On AMD this is especially important: the SMI counter (MSR 0x34) is Intel-only,
# so measuring the symptom is the ONLY way to detect firmware stalls.
#
#   sudo ./scripts/hwlat.sh [duration_seconds] [threshold_us]
#
# Copyright (c) 2026 Front Range CNC. BSD 3-Clause.

set -uo pipefail

DURATION="${1:-600}"      # seconds
THRESHOLD="${2:-10}"      # microseconds
WIDTH="${WIDTH:-500000}"  # sampling width, microseconds
WINDOW="${WINDOW:-1000000}"  # sampling window, microseconds

if [[ $EUID -ne 0 ]]; then
    echo "Must run as root: sudo $0 [duration_seconds] [threshold_us]" >&2
    exit 1
fi

# --------------------------------------------------------------- tracefs ----

TRACEFS=""
for d in /sys/kernel/tracing /sys/kernel/debug/tracing; do
    if [[ -w "$d/current_tracer" ]]; then
        TRACEFS="$d"
        break
    fi
done

if [[ -z "$TRACEFS" ]]; then
    echo "ERROR: tracefs not found or not writable." >&2
    echo "  Try: mount -t tracefs nodev /sys/kernel/tracing" >&2
    exit 1
fi

if ! grep -qw hwlat "$TRACEFS/available_tracers" 2>/dev/null; then
    echo "ERROR: 'hwlat' tracer not available." >&2
    echo "  available: $(cat "$TRACEFS/available_tracers" 2>/dev/null)" >&2
    echo "  The kernel needs CONFIG_HWLAT_TRACER=y." >&2
    exit 1
fi

# ------------------------------------------------------------ save state ----

PREV_TRACER=$(cat "$TRACEFS/current_tracer" 2>/dev/null || echo nop)
PREV_THRESH=$(cat "$TRACEFS/tracing_thresh" 2>/dev/null || echo 0)

restore() {
    echo 0 > "$TRACEFS/tracing_on"        2>/dev/null || true
    echo "$PREV_TRACER" > "$TRACEFS/current_tracer" 2>/dev/null || true
    echo "$PREV_THRESH" > "$TRACEFS/tracing_thresh" 2>/dev/null || true
}
trap restore EXIT INT TERM

# ---------------------------------------------------------------- config ----

echo "=== Hardware latency detection ==="
echo "  tracefs   : ${TRACEFS}"
echo "  duration  : ${DURATION} s"
echo "  threshold : ${THRESHOLD} us"
echo "  width     : ${WIDTH} us  (sampling period with interrupts disabled)"
echo "  window    : ${WINDOW} us (total period per sample)"
echo

echo 0    > "$TRACEFS/tracing_on"
echo nop  > "$TRACEFS/current_tracer"
echo      > "$TRACEFS/trace"                      # clear the ring buffer

echo hwlat > "$TRACEFS/current_tracer"
echo "$THRESHOLD" > "$TRACEFS/tracing_thresh"

# width/window live under hwlat_detector/ on most kernels.
if [[ -w "$TRACEFS/hwlat_detector/width" ]]; then
    echo "$WIDTH"  > "$TRACEFS/hwlat_detector/width"
    echo "$WINDOW" > "$TRACEFS/hwlat_detector/window"
    # round-robin moves the sampling thread across CPUs so firmware stalls are
    # caught wherever they occur, not just on one core.
    if [[ -w "$TRACEFS/hwlat_detector/mode" ]]; then
        echo "round-robin" > "$TRACEFS/hwlat_detector/mode" 2>/dev/null || true
        echo "  mode      : $(cat "$TRACEFS/hwlat_detector/mode")"
    fi
else
    echo "  (hwlat_detector/ tunables not present — using kernel defaults)"
fi

echo 1 > "$TRACEFS/tracing_on"

echo "Sampling for ${DURATION}s... (Ctrl-C to stop early)"

REMAIN="$DURATION"
while (( REMAIN > 0 )); do
    STEP=$(( REMAIN > 10 ? 10 : REMAIN ))
    sleep "$STEP"
    REMAIN=$(( REMAIN - STEP ))
    HITS=$(grep -c '^ *#' -v "$TRACEFS/trace" 2>/dev/null || echo 0)
    printf '\r  elapsed %5ds / %ds   samples over threshold: %s   ' \
        "$(( DURATION - REMAIN ))" "$DURATION" "${HITS:-0}"
done

echo 0 > "$TRACEFS/tracing_on"
echo
echo

# --------------------------------------------------------------- results ----

TRACE_OUT=$(cat "$TRACEFS/trace" 2>/dev/null || echo "")
# Non-comment, non-blank lines are detections.
COUNT=$(grep -v '^ *#' <<<"$TRACE_OUT" | grep -c '[^[:space:]]' || true)
COUNT=${COUNT:-0}

echo "=== Results ==="
echo "  samples exceeding ${THRESHOLD} us: ${COUNT}"

if (( COUNT > 0 )); then
    echo
    echo "  detections (first 20):"
    grep -v '^ *#' <<<"$TRACE_OUT" | grep '[^[:space:]]' | head -20 | sed 's/^/    /'

    # Lines carry an "inner/outer" pair in microseconds; surface the largest.
    MAXLAT=$(grep -o 'inner/outer([^)]*)' <<<"$TRACE_OUT" 2>/dev/null \
             | tr -dc '0-9\n/' | tr '/' '\n' | sort -n | tail -1 || echo "")
    [[ -n "$MAXLAT" ]] && echo "  worst observed: ${MAXLAT} us"

    echo
    echo "=== Verdict ==="
    echo "  FAIL — firmware is stealing CPU time from the kernel."
    echo
    echo "  This is a HARDWARE/BIOS problem. Kernel tuning cannot fix it."
    echo "  Work through, in the BIOS:"
    echo "    - Disable USB legacy support / USB emulation  (classic SMI source)"
    echo "    - Set fan control to a fixed profile          (thermal polling)"
    echo "    - Disable hardware monitoring / health polling"
    echo "    - Disable TPM / fTPM if unused                (known Ryzen stalls)"
    echo "    - Disable ASPM and PCIe power management"
    echo "    - Update the BIOS"
    echo
    echo "  If it cannot be eliminated, this board is unfit for hard real-time."
    echo "  See docs/02-pc-realtime-setup.md §3."
    exit 1
fi

echo
echo "=== Verdict ==="
echo "  PASS — no firmware-induced stalls above ${THRESHOLD} us."
echo
echo "  Run again for longer before signing off Phase 1; some SMI sources fire"
echo "  only every few minutes:"
echo "    sudo $0 1800 10"
exit 0
