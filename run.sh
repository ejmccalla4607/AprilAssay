#!/bin/bash
# Boost the camera IRQ kernel thread to SCHED_FIFO 49 before running the pipeline.
# Detects Pi 4 (unicam) or Pi 5 (rp1-cfe) automatically.
# Usage: sudo ./run.sh [vision args...]

if [[ $EUID -ne 0 ]]; then
    echo "Must run as root (sudo ./run.sh ...)" >&2
    exit 1
fi

# PipeWire/WirePlumber grab camera devices and hold them open, causing
# VIDIOC_S_FMT to fail with EBUSY.  Stop both the system service and the
# user session service (PipeWire typically runs as the latter on Pi OS).
REAL_USER=$(logname 2>/dev/null || who | awk 'NR==1{print $1}')
for svc in pipewire wireplumber pipewire-pulse; do
    systemctl stop "$svc" 2>/dev/null
    [[ -n "$REAL_USER" ]] && sudo -u "$REAL_USER" systemctl --user stop "$svc" 2>/dev/null
done
pkill -x pipewire wireplumber 2>/dev/null; sleep 0.5

# Pi 4 unicam creates a named threaded-IRQ kernel thread; boost it so the
# CSI-2 frame-end wakeup is immediate (priority 49 > capture thread FIFO 10).
# Pi 5 rp1-cfe may also create a named IRQ thread once streaming; try both.
IRQ_TID=""
IRQ_NAME=""
for needle in "unicam" "rp1-cfe"; do
    IRQ_TID=$(grep -rl "$needle" /proc/*/comm 2>/dev/null | grep -oP '(?<=/proc/)\d+' | head -1)
    if [[ -n "$IRQ_TID" ]]; then
        IRQ_NAME="$needle"
        break
    fi
done

if [[ -n "$IRQ_TID" ]]; then
    chrt -f -p 49 "$IRQ_TID"
    echo "[run.sh] $IRQ_NAME IRQ thread (tid=$IRQ_TID) boosted to SCHED_FIFO 49"
else
    echo "[run.sh] warning: camera IRQ thread not found — skipping IRQ boost"
fi

exec "$(dirname "$0")/build/vision" "$@"
