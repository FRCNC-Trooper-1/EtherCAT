#!/usr/bin/env bash
# install-deps.sh — Install build and real-time tooling on Ubuntu/Debian.
#
#   sudo ./scripts/install-deps.sh
#
# Copyright (c) 2026 Front Range CNC. BSD 3-Clause.

set -euo pipefail

if [[ $EUID -ne 0 ]]; then
    echo "This script must run as root: sudo $0" >&2
    exit 1
fi

echo "=== Installing build toolchain and real-time tools ==="

apt-get update

apt-get install -y \
    build-essential \
    cmake \
    ninja-build \
    pkg-config \
    git \
    clang-format \
    clang-tidy \
    `# real-time measurement — mandatory, see docs/02 §6` \
    rt-tests \
    stress-ng \
    `# network diagnostics` \
    ethtool \
    tcpdump \
    iproute2 \
    `# kernel build prerequisites, for building a PREEMPT_RT kernel` \
    libncurses-dev \
    bison \
    flex \
    libssl-dev \
    libelf-dev \
    bc \
    dwarves \
    rsync \
    zstd \
    fakeroot \
    `# useful for hardware identification` \
    pciutils \
    usbutils

echo
echo "=== Installed ==="
for t in cmake ninja gcc g++ git cyclictest hwlatdetect stress-ng ethtool; do
    printf '  %-14s ' "$t"
    if command -v "$t" >/dev/null 2>&1; then
        echo "$(command -v "$t")"
    else
        echo "MISSING"
    fi
done

echo
echo "Next:"
echo "  ./scripts/check-realtime.sh <ethercat-interface>"
echo
echo "If the kernel is not PREEMPT_RT, see docs/02-pc-realtime-setup.md §2"
echo "(PREEMPT_RT has been mainline since Linux 6.12 — no out-of-tree patch needed)."
