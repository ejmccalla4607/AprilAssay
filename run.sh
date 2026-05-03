#!/bin/bash
# Boost the camera IRQ kernel thread to SCHED_FIFO 49 before running the pipeline.
# Detects Pi 4 (unicam) or Pi 5 (rp1-cfe) automatically.
# Usage: sudo ./run.sh [vision args...]

if [[ $EUID -ne 0 ]]; then
    echo "Must run as root (sudo ./run.sh ...)" >&2
    exit 1
fi

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
