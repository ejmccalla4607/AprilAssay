#!/bin/bash
set -e

PROJECT_DIR=~/projects/tbd

rm -rf "$PROJECT_DIR/build"
cmake -S "$PROJECT_DIR" -B "$PROJECT_DIR/build"
cmake --build "$PROJECT_DIR/build" -j$(nproc)

echo ""
echo "=== Build complete ==="
echo "Executable is at $PROJECT_DIR/build/vision"