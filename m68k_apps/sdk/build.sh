#!/bin/bash
#
# M68K SDK - Build Script
#
# Usage:
#   ./build.sh              Build the SDK library
#   ./build.sh myapp.c      Build an application with the SDK
#   ./build.sh clean        Clean build artifacts
#
# Set CROSS environment variable to override toolchain prefix:
#   CROSS=m68k-elf- ./build.sh myapp.c
#

set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

if [ "$1" = "clean" ]; then
    echo "Cleaning build artifacts..."
    make -f Makefile clean
    exit 0
fi

# Build the SDK library first
echo "=== Building M68K SDK ==="
make -f Makefile all

# If an application source file is provided, build it
if [ -n "$1" ]; then
    echo ""
    echo "=== Building Application: $1 ==="
    make -f Makefile APP="$1" app
    echo ""
    APP_NAME=$(basename "$1" .c)
    echo "Done! Binary: build/${APP_NAME}.bin"
    echo "Load with:  load /sdcard/${APP_NAME}.bin"
    echo "Run with:   loadrun /sdcard/${APP_NAME}.bin"
fi
