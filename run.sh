#!/bin/bash
# Boost the unicam IRQ thread to SCHED_FIFO 49 before running the pipeline.
# This needs to happen each boot, so run via this script instead of ./build/vision directly.
# Usage: sudo ./run.sh [vision args...]

if [[ $EUID -ne 0 ]]; then
    echo "Must run as root (sudo ./run.sh ...)" >&2
    exit 1
fi

# Boost unicam IRQ kernel thread — fires at CSI-2 frame-end, unblocks capture loop.
# Priority 49 puts it above our capture thread (SCHED_FIFO 10) so the wakeup
# is immediate rather than waiting for a scheduler tick.
IRQ_TID=$(grep -r "unicam" /proc/*/comm 2>/dev/null | grep -oP '(?<=/proc/)\d+' | head -1)
if [[ -n "$IRQ_TID" ]]; then
    chrt -f -p 49 "$IRQ_TID"
    echo "[run.sh] unicam IRQ thread (tid=$IRQ_TID) boosted to SCHED_FIFO 49"
else
    echo "[run.sh] warning: unicam IRQ thread not found — skipping IRQ boost"
fi

exec "$(dirname "$0")/build/vision" "$@"
