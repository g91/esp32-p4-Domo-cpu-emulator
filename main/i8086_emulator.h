/*
 * Intel 8086 PC Emulator for ESP32-P4
 * Based on FabGL i8086 implementation
 * Includes BIOS, PIC, PIT, RTC support
 */

#ifndef I8086_EMULATOR_H
#define I8086_EMULATOR_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Memory configuration
#define I86_RAM_SIZE        (640 * 1024)     // 640KB conventional memory
#define I86_VIDEO_SIZE      (128 * 1024)     // 128KB video memory
#define I86_BIOS_SIZE       (64 * 1024)      // 64KB BIOS ROM

// CPU state
typedef enum {
    I86_STATE_STOPPED,
    I86_STATE_RUNNING,
    I86_STATE_HALTED,
    I86_STATE_ERROR
} i86_state_t;

// Emulator configuration
typedef struct {
    uint32_t ram_size;          // RAM size in bytes (default 640KB)
    uint32_t cpu_freq_mhz;      // CPU frequency in MHz (default 8MHz)
    bool enable_fpu;            // Enable x87 FPU emulation
    bool enable_sound;          // Enable PC speaker emulation
    const char *boot_disk;      // Boot disk image path
} i86_config_t;

// CPU registers structure for state inspection
typedef struct {
    uint16_t ax, bx, cx, dx;
    uint16_t si, di, bp, sp;
    uint16_t cs, ds, es, ss;
    uint16_t ip;
    uint16_t flags;
} i86_regs_t;

// Statistics
typedef struct {
    uint64_t total_cycles;
    uint64_t instructions_executed;
    uint32_t interrupts_processed;
    uint32_t disk_reads;
    uint32_t disk_writes;
} i86_stats_t;

/**
 * Initialize the i8086 emulator
 * @param config Configuration structure (NULL for defaults)
 * @return ESP_OK on success
 */
esp_err_t i86_init(i86_config_t *config);

/**
 * Reset the CPU to power-on state
 * Starts at CS:IP = 0xFFFF:0x0000 (BIOS entry)
 */
void i86_reset(void);

/**
 * Execute N instructions
 * @param count Number of instructions to execute (0 = run continuously)
 * @return Number of instructions actually executed
 */
uint32_t i86_run(uint32_t count);

/**
 * Execute single instruction (step mode)
 * @return ESP_OK if instruction executed successfully
 */
esp_err_t i86_step(void);

/**
 * Stop execution
 */
void i86_stop(void);

/**
 * Get current CPU state
 * @return Current state
 */
i86_state_t i86_get_state(void);

/**
 * Get CPU registers
 * @param regs Pointer to register structure to fill
 */
void i86_get_registers(i86_regs_t *regs);

/**
 * Get emulator statistics
 * @param stats Pointer to statistics structure to fill
 */
void i86_get_stats(i86_stats_t *stats);

/**
 * Print CPU state to console
 * @param buffer Buffer to write state string (min 1024 bytes)
 * @param size Buffer size
 */
void i86_get_state_string(char *buffer, size_t size);

/**
 * Dump memory region
 * @param addr Start address
 * @param length Number of bytes to dump
 */
void i86_dump_memory(uint32_t addr, uint32_t length);

/**
 * Load binary into memory
 * @param data Binary data
 * @param size Data size
 * @param addr Load address (segment:offset)
 * @return ESP_OK on success
 */
esp_err_t i86_load_binary(const uint8_t *data, uint32_t size, uint32_t addr);

/**
 * Trigger interrupt
 * @param num Interrupt number (0-255)
 * @return true if interrupt was accepted
 */
bool i86_interrupt(uint8_t num);

/**
 * Check if emulator is initialized
 * @return true if initialized
 */
bool i86_is_initialized(void);

/**
 * Get direct memory pointer (use with caution)
 * @return Pointer to RAM base
 */
uint8_t *i86_get_memory(void);

/**
 * Get video memory pointer
 * @return Pointer to video RAM (for display integration)
 */
uint8_t *i86_get_video_memory(void);

/**
 * Send a key press to the emulator
 * @param scancode PC XT scancode
 * @param ascii ASCII value of the key
 */
void i86_send_key(uint8_t scancode, uint8_t ascii);

/**
 * Mount a disk image file
 * @param drive Drive number (0=A:, 1=B:, 2=C:, 3=D:)
 * @param path Path to disk image file
 * @return ESP_OK on success
 */
esp_err_t i86_mount_disk(int drive, const char *path);

/**
 * Cleanup and free resources
 */
void i86_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // I8086_EMULATOR_H
