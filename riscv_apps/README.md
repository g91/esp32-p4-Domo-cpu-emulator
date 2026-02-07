# Native RISC-V Test Application

This test application runs natively on the ESP32-P4 RISC-V processor and demonstrates direct interaction with the M68K emulator through its API.

## Purpose

This application shows how to:
1. Initialize the M68K emulator from native code
2. Load M68K programs into emulator memory
3. Control M68K execution (reset, step, run)
4. Read/write M68K registers and memory
5. Use shared memory for communication
6. Access memory-mapped I/O devices

## Integration with Main Project

To test this application with your M68K emulator project:

### Option 1: Temporary Replacement (Quick Test)

1. **Backup current main:**
   ```cmd
   cd main
   copy hello_world_main.c hello_world_main.c.backup
   ```

2. **Replace with test:**
   ```cmd
   copy ..\riscv_apps\test_riscv_main.c hello_world_main.c
   ```

3. **Build and flash:**
   ```cmd
   cd ..
   .\build.cmd
   idf.py -p COM3 flash monitor
   ```

4. **Restore original:**
   ```cmd
   cd main
   copy hello_world_main.c.backup hello_world_main.c
   ```

### Option 2: Console Command (Integrated Test)

Add this test as a console command to your existing project:

```c
// In hello_world_main.c, add this external declaration
extern void test_m68k_control(void);
extern void test_bus_controller(void);

// Add console command
static int cmd_test_riscv(int argc, char **argv) {
    console_printf("Running RISC-V integration tests...\n\n");
    test_m68k_control();
    console_printf("\n");
    test_bus_controller();
    return 0;
}

// In app_main(), register command
esp_console_cmd_t cmd_test = {
    .command = "test_riscv",
    .help = "Run RISC-V to M68K integration tests",
    .hint = NULL,
    .func = &cmd_test_riscv
};
ESP_ERROR_CHECK(esp_console_cmd_register(&cmd_test));
```

## Expected Output

```
I (1000) RISCV_TEST: 
I (1000) RISCV_TEST: ========================================
I (1000) RISCV_TEST: ESP32-P4 RISC-V Test Application
I (1000) RISCV_TEST: Testing M68K Emulator Integration
I (1000) RISCV_TEST: ========================================

I (2000) RISCV_TEST: === ESP32-P4 RISC-V Test Application ===
I (2000) RISCV_TEST: Testing M68K emulator control

I (2000) RISCV_TEST: [Test 1] Initializing M68K emulator...
I (2010) m68k: 68000 CPU initialized successfully
I (2010) RISCV_TEST:   ✓ M68K initialized

I (2010) RISCV_TEST: [Test 2] Loading test program (28 bytes)...
I (2020) RISCV_TEST:   ✓ Program loaded at address 0x00000000

I (2020) RISCV_TEST: [Test 3] Verifying program in memory...
I (2030) RISCV_TEST:   ✓ Memory verification passed (0x70)

I (2030) RISCV_TEST: [Test 4] Resetting M68K CPU...
I (2030) m68k: CPU Reset - PC: 0x00000010, SP: 0x00FFFFFC
I (2040) RISCV_TEST:   ✓ Reset complete: PC=0x00000010, SP=0x00FFFFFC

I (2040) RISCV_TEST: [Test 5] Single-stepping through instructions...
I (2050) RISCV_TEST:   Step 1: D0=0x00000000, PC=0x00000012
I (2050) RISCV_TEST:   Step 2: D0=0x00000001, PC=0x00000014
I (2060) RISCV_TEST:   Step 3: D0=0x00000001, PC=0x0000001A
I (2060) RISCV_TEST:   Step 4: D0=0x00000001, PC=0x0000001C
I (2070) RISCV_TEST:   Step 5: D0=0x00000001, PC=0x00000012

I (2070) RISCV_TEST: [Test 6] Running 100 instructions...
I (2080) RISCV_TEST:   ✓ Execution complete: D0=0x00000064 (expected ~0x64)

I (2080) RISCV_TEST: [Test 7] Testing shared memory communication...
I (2090) RISCV_TEST:   Writing 0xDEADBEEF from RISC-V to M68K memory...
I (2090) RISCV_TEST:   Reading back from M68K memory...
I (2100) RISCV_TEST:   ✓ Shared memory test PASSED (0xDEADBEEF)

I (2100) RISCV_TEST: [Test 8] Dumping M68K memory (first 32 bytes)...
  Address    +0 +1 +2 +3  +4 +5 +6 +7  +8 +9 +A +B  +C +D +E +F
  -----------------------------------------------------------
  0x00000000  00 00 FF FC  00 00 00 10  00 00 00 00  00 00 00 00 
  0x00000010  70 00 52 40  0C 40 00 64  6F F8 4E 71  60 FE 00 00 

I (2120) RISCV_TEST: 
I (2120) RISCV_TEST: === All Tests Completed ===

I (3120) RISCV_TEST: === Bus Controller Test ===
I (3120) RISCV_TEST: Testing memory-mapped I/O
I (3130) RISCV_TEST: 
I (3130) RISCV_TEST: [Console Device Test]
I (3130) RISCV_TEST:   Writing to console from RISC-V...
Hello from RISC-V!
I (3140) RISCV_TEST:   ✓ Console write complete
I (3140) RISCV_TEST: 
I (3150) RISCV_TEST: === Bus Controller Test Completed ===

I (3150) RISCV_TEST: ========================================
I (3150) RISCV_TEST: All tests completed!
I (3160) RISCV_TEST: System will idle now.
I (3160) RISCV_TEST: ========================================
```

## Test Details

### Test 1: M68K Initialization
- Allocates 16MB RAM in ESP32-P4 PSRAM
- Initializes M68K CPU state
- Verifies memory allocation

### Test 2: Program Loading
- Loads small test program into M68K memory
- Program increments D0 from 0 to 100

### Test 3: Memory Verification
- Reads back loaded program
- Verifies memory integrity

### Test 4: CPU Reset
- Resets M68K processor
- Verifies reset vectors (SP, PC)

### Test 5: Single-Step Execution
- Steps through 5 instructions
- Monitors register changes
- Verifies program counter advancement

### Test 6: Batch Execution
- Runs 100 instructions
- Verifies D0 reaches expected value (100/0x64)

### Test 7: Shared Memory
- RISC-V writes to M68K address space
- M68K reads back value
- Validates bidirectional memory access

### Test 8: Memory Dump
- Displays first 32 bytes of M68K memory
- Shows reset vectors and program code

### Bus Controller Test
- Writes to console I/O device (0x00F02000)
- Demonstrates memory-mapped I/O access

## Architecture Benefits Demonstrated

This test shows the key advantage of the bus controller architecture:

1. **Native Control**: ESP32-P4 RISC-V can fully control M68K execution
2. **Shared Memory**: Both processors access same physical RAM
3. **Zero-Copy Communication**: No data copying between processors
4. **Memory-Mapped I/O**: Unified device access model
5. **Flexible Deployment**: M68K can run as coprocessor or standalone

## Performance Notes

The test measures:
- M68K initialization time (~10ms)
- Single instruction execution (~100μs)
- Memory access latency (direct PSRAM access)
- Bus controller overhead (minimal)

## Extending the Test

You can extend this test to:

1. **Test network operations**:
   ```c
   // Load M68K network test program
   // Monitor socket operations via bus controller
   ```

2. **Test DMA transfers**:
   ```c
   // Setup DMA from M68K to ARM space
   // Verify data integrity
   ```

3. **Benchmark performance**:
   ```c
   uint32_t start = esp_cpu_get_cycle_count();
   m68k_run(10000);
   uint32_t cycles = esp_cpu_get_cycle_count() - start;
   printf("MIPS: %f\n", (float)10000 / (cycles / 400000000.0) / 1000000);
   ```

## Troubleshooting

**M68K initialization fails:**
- Check SPIRAM is enabled in menuconfig
- Verify 32MB PSRAM is available
- Check memory allocation (should be ~16MB)

**Program doesn't execute:**
- Verify test program loaded correctly
- Check PC points to 0x00000010
- Ensure reset vectors are at 0x00000000

**Shared memory test fails:**
- Verify address 0x00200000 is within M68K RAM
- Check endianness (M68K is big-endian)
- Ensure no address translation issues

## See Also

- [../M68K_EMULATOR_README.md](../M68K_EMULATOR_README.md) - M68K emulator API
- [../BUS_CONTROLLER_GUIDE.md](../BUS_CONTROLLER_GUIDE.md) - Bus controller architecture
- [../m68k_apps/README.md](../m68k_apps/README.md) - M68K application development
