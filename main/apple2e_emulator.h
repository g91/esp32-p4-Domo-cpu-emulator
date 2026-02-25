/*
 * Apple IIe Emulator Integration Layer
 * 6502-based Apple IIe with 64KB RAM, text/lores/hires graphics,
 * keyboard, speaker, and Disk II controller emulation.
 */

#ifndef APPLE2E_EMULATOR_H
#define APPLE2E_EMULATOR_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Apple IIe memory map
#define APPLE2_MEM_SIZE       0x10000   // 64KB main RAM
#define APPLE2_AUX_MEM_SIZE   0x10000   // 64KB auxiliary RAM (enhanced IIe)
#define APPLE2_TEXT_PAGE1      0x0400    // Text/LoRes page 1: $0400-$07FF
#define APPLE2_TEXT_PAGE2      0x0800    // Text/LoRes page 2: $0800-$0BFF
#define APPLE2_HIRES_PAGE1     0x2000    // HiRes page 1: $2000-$3FFF
#define APPLE2_HIRES_PAGE2     0x4000    // HiRes page 2: $4000-$5FFF
#define APPLE2_IO_BASE         0xC000    // I/O space: $C000-$C0FF
#define APPLE2_SLOT_ROM        0xC100    // Slot ROM: $C100-$C7FF
#define APPLE2_SLOT_EXP_ROM    0xC800    // Slot expansion ROM: $C800-$CFFF
#define APPLE2_ROM_BASE        0xD000    // ROM: $D000-$FFFF (12KB)

// Soft switches (I/O addresses)
#define APPLE2_KBD             0xC000    // Keyboard data + strobe
#define APPLE2_KBDSTRB         0xC010    // Keyboard strobe clear
#define APPLE2_SPKR            0xC030    // Speaker toggle
#define APPLE2_TXTCLR          0xC050    // Graphics mode
#define APPLE2_TXTSET          0xC051    // Text mode
#define APPLE2_MIXCLR          0xC052    // Full screen
#define APPLE2_MIXSET          0xC053    // Mixed text+graphics
#define APPLE2_LOWSCR          0xC054    // Display page 1
#define APPLE2_HISCR           0xC055    // Display page 2
#define APPLE2_LORES           0xC056    // LoRes graphics
#define APPLE2_HIRES           0xC057    // HiRes graphics
#define APPLE2_AN0OFF          0xC058    // Annunciator 0 off
#define APPLE2_AN0ON           0xC059    // Annunciator 0 on
#define APPLE2_AN1OFF          0xC05A    // Annunciator 1 off
#define APPLE2_AN1ON           0xC05B    // Annunciator 1 on
#define APPLE2_AN2OFF          0xC05C    // Annunciator 2 off
#define APPLE2_AN2ON           0xC05D    // Annunciator 2 on
#define APPLE2_AN3OFF          0xC05E    // Annunciator 3 off / double-hires
#define APPLE2_AN3ON           0xC05F    // Annunciator 3 on / single-hires

// IIe enhanced soft switches
#define APPLE2_80STOREOFF      0xC000    // 80STORE off (write)
#define APPLE2_80STOREON       0xC001    // 80STORE on (write)
#define APPLE2_RAMRDOFF        0xC002    // Read main memory
#define APPLE2_RAMRDON         0xC003    // Read aux memory
#define APPLE2_RAMWRTOFF       0xC004    // Write main memory
#define APPLE2_RAMWRTON        0xC005    // Write aux memory
#define APPLE2_ALTZPOFF        0xC008    // Main zero-page/stack
#define APPLE2_ALTZPON         0xC009    // Aux zero-page/stack
#define APPLE2_80COLOFF        0xC00C    // 40-column mode
#define APPLE2_80COLON         0xC00D    // 80-column mode

/**
 * Initialize the Apple IIe emulator
 */
esp_err_t apple2e_init(void);

/**
 * Load ROM files from SD card
 * Looks for: /sdcard/apple2e/apple2e.rom (or apple2e_enhanced.rom)
 * Optional:  /sdcard/apple2e/disk2.rom (Disk II controller ROM)
 */
esp_err_t apple2e_load_roms(void);

/**
 * Reset CPU (read vectors from $FFFC)
 */
void apple2e_reset(void);

/**
 * Execute N instructions, returns CPU cycles consumed
 */
uint64_t apple2e_run(uint32_t instructions);

/**
 * Execute single instruction
 */
void apple2e_step(void);

/**
 * Stop execution
 */
void apple2e_stop(void);

/**
 * Check if CPU is halted
 */
bool apple2e_is_halted(void);

/**
 * Check if emulator is initialized
 */
bool apple2e_is_initialized(void);

/**
 * Get CPU state as string
 */
void apple2e_get_state(char *buffer, size_t size);

/**
 * Dump memory contents
 */
void apple2e_dump_memory(uint32_t addr, uint32_t length);

/**
 * Load binary at address
 */
void apple2e_load_program(const uint8_t *data, uint32_t size, uint32_t addr);

/**
 * Clean up resources
 */
void apple2e_destroy(void);

/**
 * Memory access for DMA
 */
uint8_t apple2e_read_memory(uint16_t addr);
void    apple2e_write_memory(uint16_t addr, uint8_t val);

/**
 * Render display to GPU framebuffer
 */
void apple2e_render_screen(void);

/**
 * Poll keyboard from UART/PS2/USB
 */
void apple2e_poll_keyboard(void);

/**
 * Check if exit requested
 */
bool apple2e_exit_requested(void);

/**
 * Mount a disk image (.dsk, .do, .po, .nib) into a drive (0=drive1, 1=drive2)
 */
esp_err_t apple2e_mount_disk(int drive, const char *path);

#ifdef __cplusplus
}
#endif

#endif // APPLE2E_EMULATOR_H
