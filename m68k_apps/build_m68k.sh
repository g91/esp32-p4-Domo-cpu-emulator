#!/bin/bash
# Build script for M68K test applications
# Requires m68k-elf-gcc toolchain

echo "================================================"
echo "Building M68K Test Applications"
echo "================================================"
echo ""

# Check if m68k-elf-gcc is available
if ! command -v m68k-elf-gcc &> /dev/null; then
    echo "ERROR: m68k-elf-gcc not found in PATH!"
    echo ""
    echo "Please install m68k-elf-gcc toolchain."
    echo "See M68K_COMPILER_SETUP.md for instructions."
    echo ""
    exit 1
fi

echo "M68K Toolchain found:"
m68k-elf-gcc --version | grep gcc
echo ""

# Create output directory
mkdir -p build

# Build Test 1: Console Output
echo "[1/3] Building test1_console.c..."
m68k-elf-gcc -c -O2 -m68000 -nostdlib test1_console.c -o build/test1_console.o || exit 1
m68k-elf-ld -T m68k.ld build/test1_console.o -o build/test1_console.elf || exit 1
m68k-elf-objcopy -O binary build/test1_console.elf build/test1_console.bin || exit 1
echo "      Output: build/test1_console.bin"
ls -lh build/test1_console.bin | awk '{print "      Size:", $5}'
echo ""

# Build Test 2: Register and Memory Test
echo "[2/3] Building test2_registers.c..."
m68k-elf-gcc -c -O2 -m68000 -nostdlib test2_registers.c -o build/test2_registers.o || exit 1
m68k-elf-ld -T m68k.ld build/test2_registers.o -o build/test2_registers.elf || exit 1
m68k-elf-objcopy -O binary build/test2_registers.elf build/test2_registers.bin || exit 1
echo "      Output: build/test2_registers.bin"
ls -lh build/test2_registers.bin | awk '{print "      Size:", $5}'
echo ""

# Build Test 3: Assembly Test (if VASM is available)
if command -v vasmm68k_mot &> /dev/null; then
    echo "[3/3] Building test3_asm.s with VASM..."
    vasmm68k_mot -Fbin -o build/test3_asm.bin test3_asm.s || exit 1
    echo "      Output: build/test3_asm.bin"
    ls -lh build/test3_asm.bin | awk '{print "      Size:", $5}'
    echo ""
else
    echo "[3/3] Skipping test3_asm.s (VASM not found)"
    echo "      Install VASM to build assembly programs"
    echo ""
fi

echo "================================================"
echo "Build completed successfully!"
echo "================================================"
echo ""
echo "Binary files are in the 'build' directory:"
ls -1 build/*.bin
echo ""
echo "To test on ESP32-P4:"
echo "  1. Copy .bin files to SD card"
echo "  2. In ESP32-P4 console: loadsd /sdcard/test1_console.bin"
echo "  3. Run: reset"
echo "  4. Run: run 10000"
echo ""
