# M68K Test Applications

This directory contains test applications for the ESP32-P4 M68K emulator.

## Test Programs

### test1_console.c - Console I/O Test
**Purpose:** Tests console output functionality
**Features:**
- Prints welcome message
- Counts from 0 to 9
- Demonstrates basic C programming on M68K

**Expected Output:**
```
=== M68K Test Application 1 ===
Console I/O Test
This program is running on the M68K emulator!

Counting test:
Count: 0
Count: 1
...
Count: 9

Test completed successfully!
=================================
```

### test2_registers.c - Register and Memory Test
**Purpose:** Tests CPU registers and memory operations
**Features:**
- Data register arithmetic (D0, D1, D2)
- Memory read/write verification
- Hex value printing
- Loop counter test

**Expected Output:**
```
=== M68K Test Application 2 ===
Register and Memory Test

Testing data registers:
D0 = 0x12345678
D1 = 0xAABBCCDD
D2 = D0 + D1 = 0xBCF02355

Testing memory operations:
Wrote 0xDEADBEEF to address 0x00100000
Read back: 0xDEADBEEF
Memory test PASSED!

Loop counter test (0-9):
0 1 2 3 4 5 6 7 8 9 

All tests completed!
============================
```

### test3_asm.s - Pure Assembly Test
**Purpose:** Tests 68000 assembly instructions
**Features:**
- MOVEQ, ADD, CMP, branch instructions
- Address register operations
- String output subroutines
- Memory read/write in assembly

**Expected Output:**
```
=== M68K Assembly Test ===
Pure 68000 Assembly Program

Test 1 - MOVEQ: 0x2A
Test 2 - ADD (10+20): 0x1E
Test 3 - Loop (0-9): 0 1 2 3 4 5 6 7 8 9 
Test 4 - Memory: PASS

All tests completed!
========================
```

## Building

### Prerequisites

**Option 1: m68k-elf-gcc (Recommended)**
- Download from: https://github.com/vintagepc/gcc-m68k-build/releases
- Install to `C:\m68k-elf-gcc\` (Windows) or `/opt/m68k-elf/` (Linux)
- Add `bin` directory to PATH

**Option 2: VASM/VLINK (Assembly only)**
- Download from: http://www.ibaug.de/vasm/
- Required only for test3_asm.s

### Build Commands

**Windows:**
```cmd
cd m68k_apps
build_m68k.cmd
```

**Linux/Mac:**
```bash
cd m68k_apps
chmod +x build_m68k.sh
./build_m68k.sh
```

**Manual Build:**
```bash
# Test 1
m68k-elf-gcc -c -O2 -m68000 -nostdlib test1_console.c -o test1_console.o
m68k-elf-ld -T m68k.ld test1_console.o -o test1_console.elf
m68k-elf-objcopy -O binary test1_console.elf test1_console.bin

# Test 2
m68k-elf-gcc -c -O2 -m68000 -nostdlib test2_registers.c -o test2_registers.o
m68k-elf-ld -T m68k.ld test2_registers.o -o test2_registers.elf
m68k-elf-objcopy -O binary test2_registers.elf test2_registers.bin

# Test 3 (requires VASM)
vasmm68k_mot -Fbin -o test3_asm.bin test3_asm.s
```

## Running on ESP32-P4

### Method 1: Via SD Card

1. **Copy binaries to SD card:**
   - Insert SD card in computer
   - Copy `*.bin` files from `build/` directory to SD card root
   - Safely eject SD card
   - Insert into ESP32-P4

2. **Load and run:**
   ```
   m68k> sd_mount
   m68k> loadsd /sdcard/test1_console.bin
   m68k> reset
   m68k> run 10000
   m68k> state
   ```

### Method 2: Via FTP

1. **Connect to WiFi:**
   ```
   m68k> wifi connect YourSSID YourPassword
   ```

2. **Upload via FTP:**
   - Connect to `ftp://[ESP32_IP]:21`
   - Username: any
   - Password: any
   - Upload `*.bin` files

3. **Load and run:**
   ```
   m68k> loadsd /sdcard/test1_console.bin
   m68k> reset
   m68k> run 10000
   ```

## Debugging

**View CPU state:**
```
m68k> state
```
Shows all registers, flags, instruction count, and cycle count.

**Single-step execution:**
```
m68k> step
```
Executes one instruction and shows result.

**Dump memory:**
```
m68k> dump 0x100 64
```
Shows memory contents in hex format.

**Run specific number of instructions:**
```
m68k> run 1000
```

## Memory Map

M68K programs have access to:

| Address Range | Description |
|---------------|-------------|
| `0x00000000` | Reset vectors (SSP, PC) |
| `0x00000010` | Program code start |
| `0x00100000` | Safe area for variables/stack |
| `0x00F02000` | Console output (write byte to print) |
| `0x00F00000` | Network device registers |
| `0x00F01000` | Filesystem device |
| `0x00F03000` | DMA controller |

## Troubleshooting

**Program doesn't run:**
- Verify binary is loaded: `dump 0 32`
- Check reset vectors are correct (should show stack pointer and PC)
- Ensure you ran `reset` command after loading

**Console output not showing:**
- Check that console address is `0x00F02000`
- Verify LCD is initialized if using display
- Try UART-only output first

**Build errors:**
- Ensure m68k-elf-gcc is in PATH
- Check linker script (m68k.ld) is present
- Use `-nostdlib` flag (no standard library)

**Binary too large:**
- Maximum program size: ~15MB
- Optimize with `-O2` or `-Os` flag
- Remove unused functions/data

## Creating Your Own Programs

Use the test programs as templates. Key points:

1. **Entry point must be `_start`**
2. **No standard library** (use `-nostdlib`)
3. **Console I/O:** Write bytes to `0x00F02000`
4. **Memory:** Use addresses `0x00100000` and above for data
5. **Reset vectors:** Linker script handles automatically

**Example minimal program:**
```c
#define CONSOLE ((volatile char *)0x00F02000)

void _start(void) {
    *CONSOLE = 'H';
    *CONSOLE = 'i';
    *CONSOLE = '!';
    *CONSOLE = '\n';
    while(1);  // Loop forever
}
```

## See Also

- [M68K_COMPILER_SETUP.md](../M68K_COMPILER_SETUP.md) - Detailed toolchain setup
- [M68K_EMULATOR_README.md](../M68K_EMULATOR_README.md) - Emulator documentation
- [BUS_CONTROLLER_GUIDE.md](../BUS_CONTROLLER_GUIDE.md) - Memory-mapped I/O reference
