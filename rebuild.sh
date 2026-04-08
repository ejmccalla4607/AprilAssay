#!/bin/bash
set -e

PROJECT_DIR=~/projects/tbd

# Pass DEBUG=1 to enable terminal logging: ./rebuild.sh DEBUG=1
DEBUG_FLAG=""
if [[ "$1" == "DEBUG=1" ]]; then
    DEBUG_FLAG="-DDEBUG_LOGGING=1"
    echo "=== Debug build (logging to stdout) ==="
else
    echo "=== Release build (logging to file) ==="
fi

rm -rf "$PROJECT_DIR/build"
cmake -S "$PROJECT_DIR" -B "$PROJECT_DIR/build" $DEBUG_FLAG
cmake --build "$PROJECT_DIR/build" -j$(nproc)

echo "=== Build complete ==="
echo "Executable is at $PROJECT_DIR/build/vision"