/* Native ESP32-P4 RISC-V Test Application
 * Demonstrates interaction with M68K emulator via bus controller
 * 
 * This application runs natively on the ESP32-P4 and can:
 * - Load programs into M68K memory
 * - Start/stop M68K emulator
 * - Read M68K registers and memory
 * - Communicate with M68K via shared memory
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "m68k_emulator.h"
#include "bus_controller.h"

static const char* TAG = "RISCV_TEST";

// Test program: Simple M68K loop that increments D0
static const uint8_t test_program[] = {
    // Reset vectors at 0x00000000
    0x00, 0x00, 0xFF, 0xFC,  // Initial SSP = 0x00FFFFFC (top of RAM)
    0x00, 0x00, 0x00, 0x10,  // Initial PC = 0x00000010 (start of program)
    
    0x00, 0x00, 0x00, 0x00,  // Padding
    0x00, 0x00, 0x00, 0x00,
    
    // Program starts at 0x00000010
    0x70, 0x00,              // MOVEQ #0, D0          ; D0 = 0
    0x52, 0x40,              // ADDQ.W #1, D0         ; D0++
    0x0C, 0x40, 0x00, 0x64,  // CMPI.W #100, D0       ; Compare D0 with 100
    0x6F, 0xF8,              // BLE -8                ; Branch if <= (loop)
    0x4E, 0x71,              // NOP                   ; No operation
    0x60, 0xFE,              // BRA -2                ; Loop forever
};

// Shared memory test: ESP32-P4 writes, M68K reads
#define SHARED_MEM_ADDR 0x00200000  // Safe location in M68K RAM

void test_m68k_control(void) {
    esp_err_t ret;
    
    ESP_LOGI(TAG, "=== ESP32-P4 RISC-V Test Application ===");
    ESP_LOGI(TAG, "Testing M68K emulator control");
    ESP_LOGI(TAG, "");
    
    // Test 1: Initialize M68K
    ESP_LOGI(TAG, "[Test 1] Initializing M68K emulator...");
    ret = m68k_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize M68K: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "  ✓ M68K initialized");
    
    // Test 2: Load test program
    ESP_LOGI(TAG, "[Test 2] Loading test program (%d bytes)...", sizeof(test_program));
    m68k_load_program(test_program, sizeof(test_program), 0);
    ESP_LOGI(TAG, "  ✓ Program loaded at address 0x00000000");
    
    // Test 3: Verify program in memory
    ESP_LOGI(TAG, "[Test 3] Verifying program in memory...");
    uint8_t verify_byte = m68k_read_memory_8(0x10);
    if (verify_byte == 0x70) {  // First instruction: MOVEQ #0, D0
        ESP_LOGI(TAG, "  ✓ Memory verification passed (0x%02X)", verify_byte);
    } else {
        ESP_LOGE(TAG, "  ✗ Memory verification failed (expected 0x70, got 0x%02X)", verify_byte);
    }
    
    // Test 4: Reset and check registers
    ESP_LOGI(TAG, "[Test 4] Resetting M68K CPU...");
    m68k_reset();
    uint32_t pc = m68k_get_reg(M68K_REG_PC);
    uint32_t sp = m68k_get_reg(M68K_REG_SP);
    ESP_LOGI(TAG, "  ✓ Reset complete: PC=0x%08lX, SP=0x%08lX", pc, sp);
    
    // Test 5: Single-step execution
    ESP_LOGI(TAG, "[Test 5] Single-stepping through instructions...");
    for (int i = 0; i < 5; i++) {
        m68k_step();
        uint32_t d0 = m68k_get_reg(M68K_REG_D0);
        uint32_t new_pc = m68k_get_reg(M68K_REG_PC);
        ESP_LOGI(TAG, "  Step %d: D0=0x%08lX, PC=0x%08lX", i+1, d0, new_pc);
    }
    
    // Test 6: Run for N instructions
    ESP_LOGI(TAG, "[Test 6] Running 100 instructions...");
    m68k_run(100);
    uint32_t final_d0 = m68k_get_reg(M68K_REG_D0);
    ESP_LOGI(TAG, "  ✓ Execution complete: D0=0x%08lX (expected ~0x64)", final_d0);
    
    // Test 7: Shared memory communication
    ESP_LOGI(TAG, "[Test 7] Testing shared memory communication...");
    ESP_LOGI(TAG, "  Writing 0xDEADBEEF from RISC-V to M68K memory...");
    m68k_write_memory_32(SHARED_MEM_ADDR, 0xDEADBEEF);
    
    ESP_LOGI(TAG, "  Reading back from M68K memory...");
    uint32_t read_val = m68k_read_memory_32(SHARED_MEM_ADDR);
    if (read_val == 0xDEADBEEF) {
        ESP_LOGI(TAG, "  ✓ Shared memory test PASSED (0x%08lX)", read_val);
    } else {
        ESP_LOGE(TAG, "  ✗ Shared memory test FAILED (expected 0xDEADBEEF, got 0x%08lX)", read_val);
    }
    
    // Test 8: Memory dump
    ESP_LOGI(TAG, "[Test 8] Dumping M68K memory (first 32 bytes)...");
    printf("  Address    +0 +1 +2 +3  +4 +5 +6 +7  +8 +9 +A +B  +C +D +E +F\n");
    printf("  -----------------------------------------------------------\n");
    for (int i = 0; i < 32; i += 16) {
        printf("  0x%08X ", i);
        for (int j = 0; j < 16; j++) {
            if (j % 4 == 0) printf(" ");
            uint8_t byte = m68k_read_memory_8(i + j);
            printf("%02X ", byte);
        }
        printf("\n");
    }
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== All Tests Completed ===");
}

void test_bus_controller(void) {
    ESP_LOGI(TAG, "=== Bus Controller Test ===");
    ESP_LOGI(TAG, "Testing memory-mapped I/O");
    ESP_LOGI(TAG, "");
    
    // Test console I/O device
    ESP_LOGI(TAG, "[Console Device Test]");
    ESP_LOGI(TAG, "  Writing to console from RISC-V...");
    const char *msg = "Hello from RISC-V!\r\n";
    for (int i = 0; msg[i]; i++) {
        m68k_write_memory_8(0x00F02000, msg[i]);
    }
    ESP_LOGI(TAG, "  ✓ Console write complete");
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== Bus Controller Test Completed ===");
}

void app_main(void) {
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "ESP32-P4 RISC-V Test Application");
    ESP_LOGI(TAG, "Testing M68K Emulator Integration");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "");
    
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Run M68K control tests
    test_m68k_control();
    
    ESP_LOGI(TAG, "");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Run bus controller tests
    test_bus_controller();
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "All tests completed!");
    ESP_LOGI(TAG, "System will idle now.");
    ESP_LOGI(TAG, "========================================");
    
    // Cleanup
    m68k_destroy();
    
    // Idle forever
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
