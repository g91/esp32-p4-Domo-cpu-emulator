#!/bin/bash
# Build M68K-OS Operating System for WSL/Linux
# Requires: sudo apt install gcc-m68k-linux-gnu g++-m68k-linux-gnu

echo "================================================"
echo "Building M68K-OS Operating System"
echo "================================================"
echo

# Try different M68K compiler prefixes
M68K_PREFIX=""

if command -v m68k-linux-gnu-g++ &> /dev/null; then
    M68K_PREFIX="m68k-linux-gnu-"
elif command -v m68k-elf-g++ &> /dev/null; then
    M68K_PREFIX="m68k-elf-"
elif command -v m68k-unknown-elf-g++ &> /dev/null; then
    M68K_PREFIX="m68k-unknown-elf-"
else
    echo "ERROR: No M68K cross-compiler found!"
    echo
    echo "Install the toolchain with:"
    echo "  sudo apt update"
    echo "  sudo apt install gcc-m68k-linux-gnu g++-m68k-linux-gnu"
    echo
    exit 1
fi

echo "Found M68K toolchain: ${M68K_PREFIX}g++"
${M68K_PREFIX}g++ --version | head -1
echo

# Create output directory
mkdir -p build

# Compile M68K-OS
echo "[1/3] Compiling m68k_os.cpp..."
${M68K_PREFIX}g++ -c -O2 -m68000 -nostdlib -fno-exceptions -fno-rtti -ffreestanding m68k_os.cpp -o build/m68k_os.o
if [ $? -ne 0 ]; then
    echo "ERROR: Compilation failed!"
    exit 1
fi

echo "[2/3] Linking..."
${M68K_PREFIX}ld -T m68k.ld build/m68k_os.o -o build/m68k_os.elf
if [ $? -ne 0 ]; then
    echo "ERROR: Linking failed!"
    exit 1
fi

echo "[3/3] Creating binary..."
${M68K_PREFIX}objcopy -O binary build/m68k_os.elf build/OS.bin
if [ $? -ne 0 ]; then
    echo "ERROR: Binary creation failed!"
    exit 1
fi

echo
echo "================================================"
echo "Build Successful!"
echo "================================================"
echo
echo "Output file: build/OS.bin"
echo "File size: $(stat -c%s build/OS.bin) bytes"
echo
echo "To install the OS:"
echo "  1. Copy build/OS.bin to the root of your SD card"
echo "  2. The bootloader will auto-load it on boot"
echo
echo "Or load manually in the BIOS:"
echo "  loadsd OS.bin"
echo "  reset"
echo

# Show symbol table
echo "Symbol table:"
${M68K_PREFIX}nm build/m68k_os.elf | grep -E "_start|os_main|__bss"
echo
