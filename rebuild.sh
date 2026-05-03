#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

PI_FLAGS=""
if grep -q "Raspberry Pi 5" /proc/cpuinfo 2>/dev/null || grep -q "bcm2712" /proc/cpuinfo 2>/dev/null; then
    PI_FLAGS="-DRPI5=ON"
    echo "Detected Pi 5 — building with RPI5=ON"
fi

rm -rf "$SCRIPT_DIR/build"
cmake $PI_FLAGS -S "$SCRIPT_DIR" -B "$SCRIPT_DIR/build"
cmake --build "$SCRIPT_DIR/build" -j$(nproc)

echo ""
echo "=== Build complete ==="
echo "Executable is at $SCRIPT_DIR/build/vision"