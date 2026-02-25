/*
 * MOS 6502 Emulator Integration Layer
 * Connects the 6502 CPU core to the ESP32-P4 bus controller
 */

#ifndef MOS6502_EMULATOR_H
#define MOS6502_EMULATOR_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Memory map configuration
#define MOS6502_MEM_SIZE     0x10000   // 64KB total address space
#define MOS6502_RAM_END      0xBFFF    // RAM: $0000-$BFFF (48KB)
#define MOS6502_VIDEO_BASE   0xC000    // Video card window: $C000-$CFFF
#define MOS6502_VIDEO_END    0xCFFF
#define MOS6502_IO_BASE      0xD000    // Bus I/O devices: $D000-$DFFF
#define MOS6502_IO_END       0xDFFF
#define MOS6502_ROM_BASE     0xE000    // ROM/program: $E000-$FFFF

// Machine modes
typedef enum {
    MOS6502_MODE_BARE,       // Bare 6502 with bus I/O
    MOS6502_MODE_C64,        // Commodore 64 emulation
    MOS6502_MODE_VIC20,      // VIC-20 emulation
    MOS6502_MODE_C128        // Commodore 128 emulation
} mos6502_machine_mode_t;

// C64 memory map constants
#define C64_BASIC_ROM_ADDR   0xA000    // BASIC ROM: $A000-$BFFF (8KB)
#define C64_BASIC_ROM_SIZE   0x2000
#define C64_KERNAL_ROM_ADDR  0xE000    // KERNAL ROM: $E000-$FFFF (8KB)
#define C64_KERNAL_ROM_SIZE  0x2000
#define C64_CHARGEN_ROM_ADDR 0xD000    // Character ROM: $D000-$DFFF (4KB)
#define C64_CHARGEN_ROM_SIZE 0x1000
#define C64_IO_BASE          0xD000    // I/O area: $D000-$DFFF
#define C64_VIC_BASE         0xD000    // VIC-II registers: $D000-$D3FF
#define C64_SID_BASE         0xD400    // SID registers: $D400-$D7FF
#define C64_COLOR_RAM        0xD800    // Color RAM: $D800-$DBFF
#define C64_CIA1_BASE        0xDC00    // CIA1: $DC00-$DCFF
#define C64_CIA2_BASE        0xDD00    // CIA2: $DD00-$DDFF

// VIC-20 memory map constants
#define VIC20_BASIC_ROM_ADDR  0xC000    // BASIC ROM: $C000-$DFFF (8KB)
#define VIC20_BASIC_ROM_SIZE  0x2000
#define VIC20_KERNAL_ROM_ADDR 0xE000    // KERNAL ROM: $E000-$FFFF (8KB)
#define VIC20_KERNAL_ROM_SIZE 0x2000
#define VIC20_CHARGEN_ROM_ADDR 0x8000   // Character ROM: $8000-$8FFF (4KB)
#define VIC20_CHARGEN_ROM_SIZE 0x1000
#define VIC20_VIA1_BASE       0x9110    // VIA1: $9110-$911F
#define VIC20_VIA2_BASE       0x9120    // VIA2: $9120-$912F
#define VIC20_VIC_BASE        0x9000    // VIC chip: $9000-$900F

// C128 memory map constants
#define C128_RAM_BANK_SIZE   0x10000   // 64KB per bank (2 banks = 128KB)
#define C128_BASIC_ROM_ADDR  0x4000    // BASIC 7.0 ROM: $4000-$BFFF (32KB)
#define C128_BASIC_ROM_SIZE  0x8000
#define C128_KERNAL_ROM_ADDR 0xE000    // KERNAL ROM: $E000-$FFFF (8KB)
#define C128_KERNAL_ROM_SIZE 0x2000
#define C128_CHARGEN_ROM_SIZE 0x2000   // Character ROM: 8KB (2 charsets)
#define C128_MMU_BASE        0xD500    // MMU registers: $D500-$D50B
#define C128_VDC_BASE        0xD600    // VDC 80-col chip: $D600-$D601

/**
 * Initialize the 6502 emulator (allocate memory, set up bus)
 */
esp_err_t mos6502_emu_init(void);

/**
 * Initialize in a specific machine mode (C64, VIC-20, or bare)
 */
esp_err_t mos6502_emu_init_mode(mos6502_machine_mode_t mode);

/**
 * Load ROM files for current machine mode from SD card
 * For C64: looks for basic.rom, kernal.rom, chargen.rom in /sdcard/c64/
 * For VIC-20: looks for basic.rom, kernal.rom, chargen.rom in /sdcard/vic20/
 */
esp_err_t mos6502_emu_load_roms(void);

/**
 * Get current machine mode
 */
mos6502_machine_mode_t mos6502_emu_get_mode(void);

/**
 * Reset the CPU (read vectors from $FFFC)
 */
void mos6502_emu_reset(void);

/**
 * Execute N instructions.
 * Returns actual CPU cycles consumed.
 */
uint64_t mos6502_emu_run(uint32_t instructions);

/**
 * Execute single instruction
 */
void mos6502_emu_step(void);

/**
 * Stop execution
 */
void mos6502_emu_stop(void);

/**
 * Check if CPU is halted
 */
bool mos6502_emu_is_halted(void);

/**
 * Check if emulator is initialized
 */
bool mos6502_emu_is_initialized(void);

/**
 * Get CPU state as human-readable string
 */
void mos6502_emu_get_state(char *buffer, size_t size);

/**
 * Dump memory contents (hex dump)
 */
void mos6502_emu_dump_memory(uint32_t addr, uint32_t length);

/**
 * Load binary data into memory at specified address
 */
void mos6502_emu_load_program(const uint8_t *data, uint32_t size, uint32_t addr);

/**
 * Clean up and free resources
 */
void mos6502_emu_destroy(void);

/**
 * Memory access for bus controller DMA
 */
uint8_t mos6502_emu_read_memory(uint16_t addr);
void    mos6502_emu_write_memory(uint16_t addr, uint8_t val);

/**
 * Trigger interrupts
 */
void mos6502_emu_trigger_irq(void);
void mos6502_emu_trigger_nmi(void);

/**
 * C64 hardware tick - advance raster, CIA timers, fire IRQs.
 * Call after executing a batch of instructions, passing the number of
 * CPU cycles that elapsed.
 */
void mos6502_emu_tick(uint32_t cpu_cycles);

/**
 * Render C64 screen to console/LCD (diff-based - only outputs changed lines)
 */
void mos6502_emu_render_screen(void);

/**
 * Poll keyboard input from UART and inject into C64 keyboard buffer
 */
void mos6502_emu_poll_keyboard(void);

/**
 * Check if user requested exit (Ctrl+C) from C64 emulator
 */
bool mos6502_emu_exit_requested(void);

#ifdef __cplusplus
}
#endif

#endif // MOS6502_EMULATOR_H
