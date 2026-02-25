/**
 * @file i8086_emulator.h
 * @brief Intel 8086 PC Emulator for ESP32-P4
 *
 * Complete IBM PC/XT emulation with:
 * - Full 8086/80186 CPU (from FabGL, ported to C)
 * - BIOS with INT 10h/11h/12h/13h/14h/15h/16h/19h/1Ah handlers
 * - DOS INT 21h services (file I/O via SD card)
 * - PIC 8259, PIT 8253, i8042 keyboard, MC146818 RTC
 * - CGA text mode video → LCD console output
 * - PC speaker emulation
 * - Disk images (floppy + hard disk) via disk_image module
 *
 * Boots FreeDOS, MS-DOS, or any DOS from disk image files.
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
    uint32_t cpu_freq_mhz;     // CPU frequency in MHz (default 8MHz)
    bool enable_fpu;            // Enable x87 FPU emulation
    bool enable_sound;          // Enable PC speaker emulation
    const char *boot_disk;      // Boot disk image path (auto-mount as A: or C:)
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
    uint32_t dos_calls;
    uint32_t files_opened;
} i86_stats_t;

/**
 * Initialize the i8086 emulator
 * @param config Configuration structure (NULL for defaults)
 * @return ESP_OK on success
 */
esp_err_t i86_init(i86_config_t *config);

/**
 * Deinitialize and free resources
 */
void i86_deinit(void);

/**
 * Reset the CPU to power-on state
 */
void i86_reset(void);

/**
 * Execute N instructions with full hardware emulation
 * Includes PIT ticking, PIC interrupt delivery, video refresh
 * @param count Number of instructions to execute (0 = run until stopped)
 * @return Number of instructions actually executed
 */
uint32_t i86_run(uint32_t count);

/**
 * Boot and run continuously (blocking call for FreeRTOS task)
 * Initializes, resets, boots from disk, runs until stopped.
 */
void i86_boot_and_run(void);

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
 */
i86_state_t i86_get_state(void);

/**
 * Get CPU registers
 */
void i86_get_registers(i86_regs_t *regs);

/**
 * Get emulator statistics
 */
void i86_get_stats(i86_stats_t *stats);

/**
 * Print CPU state to console
 */
void i86_get_state_string(char *buffer, size_t size);

/**
 * Dump memory region
 */
void i86_dump_memory(uint32_t addr, uint32_t length);

/**
 * Load binary into memory
 */
esp_err_t i86_load_binary(const uint8_t *data, uint32_t size, uint32_t addr);

/**
 * Trigger interrupt
 */
bool i86_interrupt(uint8_t num);

/**
 * Send a key press to the emulator (scancode + ascii for BIOS buffer)
 */
void i86_send_key(uint8_t scancode, uint8_t ascii);

/**
 * Send a raw XT scancode (make or break)
 */
void i86_send_scancode(uint8_t scancode);

/**
 * Mount a disk image file
 * @param drive Drive number (0=A:, 1=B:, 2=C:, 3=D:)
 * @param path Path to disk image file
 * @return ESP_OK on success
 */
esp_err_t i86_mount_disk(int drive, const char *path);

// Utility
bool i86_is_initialized(void);
uint8_t *i86_get_memory(void);
uint8_t *i86_get_video_memory(void);

/**
 * Force video refresh to LCD
 */
void i86_refresh_display(void);

#ifdef __cplusplus
}
#endif

#endif // I8086_EMULATOR_H
