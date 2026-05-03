#!/bin/bash
# Characterization readiness sweep — TC1 gain×exposure grid (40 points).
# Usage: sudo ./sweep.sh [camera] [seconds_per_point]
#   sudo ./sweep.sh imx296 5
set -euo pipefail

[[ $EUID -ne 0 ]] && { echo "Must run as root: sudo ./sweep.sh" >&2; exit 1; }

CAMERA=${1:-imx296}
DURATION=${2:-5}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG_BASE="$SCRIPT_DIR/logs/sweep_${TIMESTAMP}"
mkdir -p "$SCRIPT_DIR/logs"

GAINS_DB=(0 6 12 18 24)
EXPOSURES_US=(25 50 100 200 400 800 1600 3200)
TOTAL=$(( ${#GAINS_DB[@]} * ${#EXPOSURES_US[@]} ))

echo "=== Sweep: $TOTAL points × ${DURATION}s each ==="
echo "Camera: $CAMERA | Log: ${LOG_BASE}_telemetry.csv"
echo ""

POINT=0
for GAIN_DB in "${GAINS_DB[@]}"; do
    GAIN_LIN=$(python3 -c "print(f'{10**($GAIN_DB/20):.4f}')")
    for EXP_US in "${EXPOSURES_US[@]}"; do
        POINT=$(( POINT + 1 ))
        EXP_MS=$(python3 -c "print(f'{$EXP_US/1000:.3f}')")
        printf "[%2d/%d] gain=%2d dB (%-7s x)  exposure=%4d us ... " \
               "$POINT" "$TOTAL" "$GAIN_DB" "$GAIN_LIN" "$EXP_US"

        # timeout sends SIGTERM after DURATION seconds; run.sh execs vision, so
        # vision is the child receiving SIGTERM.  The existing signal handler sets
        # running=false and the process exits cleanly.  Allow up to 4 s for drain.
        timeout $(( DURATION + 4 )) "$SCRIPT_DIR/run.sh" \
            --camera        "$CAMERA"   \
            --exposure      "$EXP_MS"   \
            --gain          "$GAIN_LIN" \
            --fps           60          \
            --nthreads      2           \
            --quad-decimate 1.0         \
            --log           "$LOG_BASE" \
        || true   # timeout exits 124; vision exits 0 — both are expected

        echo "done"
    done
done

echo ""
echo "=== Sweep complete ==="
echo "Telemetry: ${LOG_BASE}_telemetry.csv"
echo "Summarize: python3 $SCRIPT_DIR/scripts/sweep_summary.py ${LOG_BASE}_telemetry.csv"
