/*
 * Motorola 68000 CPU Emulator Header
 */

#ifndef M68K_EMULATOR_H
#define M68K_EMULATOR_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// Initialize the 68000 CPU
esp_err_t m68k_init(void);

// Reset the CPU to initial state
void m68k_reset(void);

// Load a program into memory
void m68k_load_program(const uint8_t *data, uint32_t size, uint32_t addr);

// Run the CPU for a specified number of instructions
void m68k_run(uint32_t instructions);

// Execute a single instruction
void m68k_step(void);

// Stop CPU execution
void m68k_stop(void);

// Check if CPU is halted or stopped
bool m68k_is_halted(void);

// Get CPU state as string
void m68k_get_state(char *buffer, size_t buf_size);

// Dump memory contents
void m68k_dump_memory(uint32_t addr, uint32_t length);

// Show crash context for post-mortem debugging
void m68k_show_crash_context(void);

// Clean up and free resources
void m68k_destroy(void);

// Memory access functions (for bus controller DMA)
uint8_t m68k_read_memory_8(uint32_t address);
uint16_t m68k_read_memory_16(uint32_t address);
uint32_t m68k_read_memory_32(uint32_t address);
void m68k_write_memory_8(uint32_t address, uint8_t value);
void m68k_write_memory_16(uint32_t address, uint16_t value);
void m68k_write_memory_32(uint32_t address, uint32_t value);

// Register access functions
typedef enum {
    M68K_REG_D0, M68K_REG_D1, M68K_REG_D2, M68K_REG_D3,
    M68K_REG_D4, M68K_REG_D5, M68K_REG_D6, M68K_REG_D7,
    M68K_REG_A0, M68K_REG_A1, M68K_REG_A2, M68K_REG_A3,
    M68K_REG_A4, M68K_REG_A5, M68K_REG_A6, M68K_REG_A7,
    M68K_REG_PC, M68K_REG_SR, M68K_REG_SP
} m68k_register_t;

uint32_t m68k_get_reg(m68k_register_t reg);
void m68k_set_reg(m68k_register_t reg, uint32_t value);

// Exception and crash recovery
typedef enum {
    M68K_EXCEPTION_NONE = 0,
    M68K_EXCEPTION_BUS_ERROR,
    M68K_EXCEPTION_ADDRESS_ERROR,
    M68K_EXCEPTION_ILLEGAL_INSTRUCTION,
    M68K_EXCEPTION_DIV_BY_ZERO,
    M68K_EXCEPTION_CHK,
    M68K_EXCEPTION_TRAPV,
    M68K_EXCEPTION_PRIVILEGE,
    M68K_EXCEPTION_WATCHDOG,
    M68K_EXCEPTION_TIMEOUT
} m68k_exception_t;

// Enable/disable exception recovery mode
void m68k_set_exception_recovery(bool enable);

// Recover from crash by resetting CPU to safe state
bool m68k_recover_from_crash(void);

// Set execution timeout (0 = unlimited)
void m68k_set_timeout(uint32_t milliseconds);

// I/O hooks for peripheral emulation (Atari ST, etc.)
// read_hook: returns value, sets *handled=true if it handled the access
// write_hook: writes value, sets *handled=true if it handled the access
typedef uint32_t (*m68k_io_read_hook_t)(uint32_t addr, int size, bool *handled);
typedef void (*m68k_io_write_hook_t)(uint32_t addr, uint32_t value, int size, bool *handled);
void m68k_set_io_hooks(m68k_io_read_hook_t read_hook, m68k_io_write_hook_t write_hook);

// External interrupt (IRQ) support
// Set the interrupt priority level (0-7, 0=none, 7=NMI)
void m68k_set_irq(uint8_t level);

// IRQ acknowledge callback - called when CPU acknowledges interrupt at given level
// Should return the vector number (0-255), or 0 for autovector
// If no callback set, all interrupts are autovectored
typedef uint8_t (*m68k_irq_ack_hook_t)(uint8_t level);
void m68k_set_irq_ack_hook(m68k_irq_ack_hook_t hook);

// Memory garbage collection
void m68k_gc_mark_region(uint32_t start, uint32_t end);
void m68k_gc_collect(void);
void m68k_gc_stats(uint32_t *used, uint32_t *free, uint32_t *regions);

// Get CPU cycle counter (for accurate timing by emulated hardware)
uint64_t m68k_get_cycles(void);

// RESET instruction callback - called when M68K executes RESET opcode
// This resets external hardware only (CIAs, custom chips), NOT the CPU
typedef void (*m68k_reset_hook_t)(void);
void m68k_set_reset_hook(m68k_reset_hook_t hook);

// Watchdog control
void m68k_watchdog_reset(void);
void m68k_watchdog_enable(bool enable);

#endif // M68K_EMULATOR_H
