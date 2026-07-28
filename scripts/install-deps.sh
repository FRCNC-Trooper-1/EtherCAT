#!/usr/bin/env bash
# install-deps.sh — Install build and real-time tooling.
#
# Detects the distribution and uses the appropriate package manager. Supports
# Debian/Ubuntu, Fedora/RHEL/Rocky/Alma, Arch, and openSUSE.
#
#   sudo ./scripts/install-deps.sh
#
# Copyright (c) 2026 Front Range CNC. BSD 3-Clause.

set -euo pipefail

if [[ $EUID -ne 0 ]]; then
    echo "This script must run as root: sudo $0" >&2
    exit 1
fi

# ------------------------------------------------------------ detect os ----

DISTRO_ID=""
DISTRO_LIKE=""
if [[ -r /etc/os-release ]]; then
    # shellcheck disable=SC1091
    . /etc/os-release
    DISTRO_ID="${ID:-}"
    DISTRO_LIKE="${ID_LIKE:-}"
fi

pick_family() {
    case "$DISTRO_ID" in
        debian|ubuntu|linuxmint|pop|raspbian) echo debian; return ;;
        fedora|rhel|centos|rocky|almalinux)   echo rhel;   return ;;
        arch|manjaro|endeavouros)             echo arch;   return ;;
        opensuse*|sles)                       echo suse;   return ;;
    esac
    # Fall back to ID_LIKE for derivatives we do not know by name.
    case "$DISTRO_LIKE" in
        *debian*) echo debian; return ;;
        *rhel*|*fedora*) echo rhel; return ;;
        *arch*)   echo arch; return ;;
        *suse*)   echo suse; return ;;
    esac
    echo unknown
}

FAMILY="$(pick_family)"

echo "=== Detected: ${PRETTY_NAME:-unknown} (family: ${FAMILY}) ==="
echo

if [[ "$FAMILY" == "unknown" ]]; then
    cat >&2 <<'EOF'
Unrecognised distribution. Install these manually:

  Build:       gcc g++ cmake ninja pkg-config git clang-format
  Real-time:   rt-tests (provides cyclictest, hwlatdetect), stress-ng
  Network:     ethtool tcpdump iproute2
  Kernel build: ncurses bison flex openssl-dev elfutils-dev bc pahole rsync zstd
  Hardware:    pciutils usbutils

rt-tests is the important one — cyclictest and hwlatdetect are how Phase 1
is validated. See docs/02-pc-realtime-setup.md §6.
EOF
    exit 1
fi

# ------------------------------------------------------------- install ----

case "$FAMILY" in
debian)
    apt-get update
    apt-get install -y \
        build-essential cmake ninja-build pkg-config git \
        clang-format clang-tidy \
        rt-tests stress-ng \
        ethtool tcpdump iproute2 \
        libncurses-dev bison flex libssl-dev libelf-dev bc dwarves \
        rsync zstd fakeroot \
        pciutils usbutils
    ;;

rhel)
    if command -v dnf >/dev/null 2>&1; then PM=dnf; else PM=yum; fi
    $PM install -y \
        gcc gcc-c++ make cmake ninja-build pkgconf-pkg-config git \
        clang-tools-extra \
        rt-tests stress-ng \
        ethtool tcpdump iproute \
        ncurses-devel bison flex openssl-devel elfutils-libelf-devel bc \
        rsync zstd \
        pciutils usbutils
    ;;

arch)
    pacman -Sy --noconfirm \
        base-devel cmake ninja pkgconf git \
        clang \
        rt-tests stress-ng \
        ethtool tcpdump iproute2 \
        ncurses bison flex openssl bc \
        rsync zstd \
        pciutils usbutils
    ;;

suse)
    zypper --non-interactive install -y \
        gcc gcc-c++ make cmake ninja pkg-config git \
        clang-tools \
        rt-tests stress-ng \
        ethtool tcpdump iproute2 \
        ncurses-devel bison flex libopenssl-devel libelf-devel bc \
        rsync zstd \
        pciutils usbutils
    ;;
esac

# -------------------------------------------------------------- verify ----

echo
echo "=== Verification ==="
MISSING=0
for t in cmake ninja gcc g++ git cyclictest hwlatdetect stress-ng ethtool; do
    printf '  %-14s ' "$t"
    if command -v "$t" >/dev/null 2>&1; then
        command -v "$t"
    else
        echo "MISSING"
        MISSING=$((MISSING + 1))
    fi
done

if (( MISSING > 0 )); then
    echo
    echo "WARNING: ${MISSING} tool(s) missing."
    echo "cyclictest and hwlatdetect come from 'rt-tests' and are REQUIRED to"
    echo "validate Phase 1. On some distros rt-tests lives in a separate repo."
fi

# --------------------------------------------------------------- notes ----

echo
if [[ -e /sys/kernel/realtime ]] && [[ "$(cat /sys/kernel/realtime 2>/dev/null)" == "1" ]]; then
    echo "PREEMPT_RT kernel detected."
else
    echo "NOTE: this kernel is NOT PREEMPT_RT."
    echo "      PREEMPT_RT is mainline since Linux 6.12 — build a stock kernel"
    echo "      with CONFIG_PREEMPT_RT=y. See docs/02-pc-realtime-setup.md §2."
fi

if [[ "$DISTRO_ID" == "ubuntu" ]]; then
    cat <<'EOF'

COMMERCIAL LICENSING NOTE (Ubuntu only)
  Ubuntu's "Real-time Ubuntu" kernel (linux-image-realtime) ships via Ubuntu
  Pro, which is free for PERSONAL use only. Deploying it on machines you SELL
  requires a paid Ubuntu Pro subscription per machine.

  Building your own mainline kernel with CONFIG_PREEMPT_RT=y avoids that
  recurring per-unit cost entirely. See docs/02-pc-realtime-setup.md §2.
EOF
fi

echo
echo "Next:  ./scripts/check-realtime.sh <ethercat-interface>"
