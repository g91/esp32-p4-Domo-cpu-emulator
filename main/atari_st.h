/*
 * Atari ST Emulator Header
 * 
 * Emulates Atari ST hardware on ESP32-P4 using the existing M68K CPU core.
 * Supports TOS ROM loading, video shifter, MFP 68901, ACIA keyboard,
 * YM2149 sound/floppy control, and DMA/FDC for floppy disk access.
 *
 * Hardware emulated:
 *   - Motorola 68000 CPU @ 8 MHz (via m68k_emulator.c)
 *   - Video Shifter (320x200x16, 640x200x4, 640x400x2)
 *   - MFP 68901 (timers, interrupts, serial I/O)
 *   - ACIA 6850 x2 (keyboard IKBD + MIDI)
 *   - YM2149 PSG (sound + floppy/printer port control)
 *   - DMA controller + WD1772 FDC (floppy disk)
 *   - Glue chip (bus arbitration, interrupts)
 *
 * Memory Map:
 *   $000000-$0FFFFF : RAM (up to 1MB, expandable)
 *   $FA0000-$FBFFFF : Cartridge ROM (128KB, optional)
 *   $FC0000-$FEFFFF : TOS ROM (192KB for TOS 1.x)
 *   $FF8000-$FF8FFF : I/O hardware registers
 *   $FFFA00-$FFFA3F : MFP 68901
 *   $FFFC00-$FFFC07 : ACIA (keyboard + MIDI)
 */

#ifndef ATARI_ST_H
#define ATARI_ST_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// ============================================================================
// Memory Map Constants
// ============================================================================

#define ATARI_RAM_SIZE          (1024 * 1024)    // 1MB RAM (ST standard)
#define ATARI_RAM_BASE          0x000000
#define ATARI_RAM_END           (ATARI_RAM_BASE + ATARI_RAM_SIZE)

#define ATARI_CART_BASE         0xFA0000         // Cartridge ROM
#define ATARI_CART_SIZE         (128 * 1024)

#define ATARI_TOS_BASE          0xFC0000         // TOS ROM (TOS 1.x)
#define ATARI_TOS_SIZE          (192 * 1024)     // 192KB for TOS 1.00-1.04
#define ATARI_TOS_END           (ATARI_TOS_BASE + ATARI_TOS_SIZE)

// Alternate TOS locations for TOS 2.x (STE)
#define ATARI_TOS2_BASE         0xE00000
#define ATARI_TOS2_SIZE         (256 * 1024)

// ============================================================================
// I/O Register Addresses
// ============================================================================

// Memory Configuration
#define ATARI_IO_BASE           0xFF8000
#define ATARI_MEM_CONF          0xFF8001         // Memory configuration

// Video Shifter
#define ATARI_VIDEO_BASE        0xFF8200
#define ATARI_VID_ADDR_HI       0xFF8201         // Video base address high byte
#define ATARI_VID_ADDR_MID      0xFF8203         // Video base address mid byte
#define ATARI_VID_ADDR_LO       0xFF820D         // Video address counter low (STE)
#define ATARI_VID_SYNC          0xFF820A         // Sync mode (50/60Hz)
#define ATARI_VID_RES           0xFF8260         // Video resolution
#define ATARI_VID_PALETTE       0xFF8240         // 16 color palette ($FF8240-$FF825F)

// DMA / FDC (Floppy Disk Controller)
#define ATARI_DMA_BASE          0xFF8600
#define ATARI_DMA_DATA          0xFF8604         // DMA data (FDC access)
#define ATARI_DMA_MODE          0xFF8606         // DMA mode/status
#define ATARI_DMA_ADDR_HI       0xFF8609         // DMA address high
#define ATARI_DMA_ADDR_MID      0xFF860B         // DMA address mid
#define ATARI_DMA_ADDR_LO       0xFF860D         // DMA address low

// YM2149 PSG (Sound + I/O)
#define ATARI_YM_BASE           0xFF8800
#define ATARI_YM_REG_SELECT     0xFF8800         // Register select / read
#define ATARI_YM_REG_WRITE      0xFF8802         // Register write

// MFP 68901 (Multi-Function Peripheral)
#define ATARI_MFP_BASE          0xFFFA00
#define ATARI_MFP_GPIP          0xFFFA01         // General purpose I/O
#define ATARI_MFP_AER           0xFFFA03         // Active edge register
#define ATARI_MFP_DDR           0xFFFA05         // Data direction register
#define ATARI_MFP_IERA          0xFFFA07         // Interrupt enable A
#define ATARI_MFP_IERB          0xFFFA09         // Interrupt enable B
#define ATARI_MFP_IPRA          0xFFFA0B         // Interrupt pending A
#define ATARI_MFP_IPRB          0xFFFA0D         // Interrupt pending B
#define ATARI_MFP_ISRA          0xFFFA0F         // Interrupt in-service A
#define ATARI_MFP_ISRB          0xFFFA11         // Interrupt in-service B
#define ATARI_MFP_IMRA          0xFFFA13         // Interrupt mask A
#define ATARI_MFP_IMRB          0xFFFA15         // Interrupt mask B
#define ATARI_MFP_VR            0xFFFA17         // Vector register
#define ATARI_MFP_TACR          0xFFFA19         // Timer A control
#define ATARI_MFP_TBCR          0xFFFA1B         // Timer B control
#define ATARI_MFP_TCDCR         0xFFFA1D         // Timer C/D control
#define ATARI_MFP_TADR          0xFFFA1F         // Timer A data
#define ATARI_MFP_TBDR          0xFFFA21         // Timer B data
#define ATARI_MFP_TCDR          0xFFFA23         // Timer C data
#define ATARI_MFP_TDDR          0xFFFA25         // Timer D data
#define ATARI_MFP_SCR           0xFFFA27         // Sync character
#define ATARI_MFP_UCR           0xFFFA29         // USART control
#define ATARI_MFP_RSR           0xFFFA2B         // Receiver status
#define ATARI_MFP_TSR           0xFFFA2D         // Transmitter status
#define ATARI_MFP_UDR           0xFFFA2F         // USART data

// ACIA 6850 (Keyboard + MIDI)
#define ATARI_ACIA_BASE         0xFFFC00
#define ATARI_KBD_CTRL          0xFFFC00         // Keyboard ACIA control
#define ATARI_KBD_DATA          0xFFFC02         // Keyboard ACIA data
#define ATARI_MIDI_CTRL         0xFFFC04         // MIDI ACIA control
#define ATARI_MIDI_DATA         0xFFFC06         // MIDI ACIA data

// ============================================================================
// Video Constants
// ============================================================================

typedef enum {
    ATARI_RES_LOW  = 0,   // 320x200, 16 colors (ST low)
    ATARI_RES_MED  = 1,   // 640x200, 4 colors  (ST medium)
    ATARI_RES_HIGH = 2,   // 640x400, 2 colors  (ST high/mono)
} atari_resolution_t;

#define ATARI_SCREEN_LOW_W      320
#define ATARI_SCREEN_LOW_H      200
#define ATARI_SCREEN_MED_W      640
#define ATARI_SCREEN_MED_H      200
#define ATARI_SCREEN_HI_W       640
#define ATARI_SCREEN_HI_H       400

// Video memory size: 32KB for screen
#define ATARI_VRAM_SIZE         (32 * 1024)

// ============================================================================
// Interrupt Vectors
// ============================================================================

#define ATARI_VEC_HBL           0x68     // HBL auto-vector (level 2)
#define ATARI_VEC_VBL           0x70     // VBL auto-vector (level 4)
#define ATARI_MFP_VEC_BASE      0x100    // MFP interrupt vectors (level 6)

// ============================================================================
// FDC (WD1772) Constants
// ============================================================================

#define FDC_CMD_RESTORE         0x00
#define FDC_CMD_SEEK            0x10
#define FDC_CMD_STEP            0x20
#define FDC_CMD_STEP_IN         0x40
#define FDC_CMD_STEP_OUT        0x60
#define FDC_CMD_READ_SECTOR     0x80
#define FDC_CMD_WRITE_SECTOR    0xA0
#define FDC_CMD_READ_ADDRESS    0xC0
#define FDC_CMD_FORCE_INT       0xD0
#define FDC_CMD_READ_TRACK      0xE0
#define FDC_CMD_WRITE_TRACK     0xF0

#define FDC_STATUS_BUSY         0x01
#define FDC_STATUS_DRQ          0x02
#define FDC_STATUS_LOST_DATA    0x04
#define FDC_STATUS_CRC_ERROR    0x08
#define FDC_STATUS_RNF          0x10
#define FDC_STATUS_MOTOR_ON     0x80

// ============================================================================
// Configuration Structure
// ============================================================================

typedef struct {
    uint32_t ram_size;           // RAM size (default 1MB)
    const char *tos_path;        // Path to TOS ROM file on SD card
    const char *floppy_a_path;   // Path to floppy A: disk image (optional)
    const char *floppy_b_path;   // Path to floppy B: disk image (optional)
    const char *cart_path;       // Path to cartridge ROM (optional)
    bool enable_lcd_output;      // Enable LCD frame buffer output
} atari_st_config_t;

// ============================================================================
// Public API
// ============================================================================

// Initialize the Atari ST emulator (loads TOS, sets up hardware)
esp_err_t atari_st_init(const atari_st_config_t *config);

// Reset the Atari ST (cold reset)
void atari_st_reset(void);

// Run the emulator (call from FreeRTOS task - runs until stopped)
void atari_st_run(void);

// Stop the emulator
void atari_st_stop(void);

// Check if emulator is running
bool atari_st_is_running(void);

// Get emulator status string
void atari_st_get_state(char *buf, size_t buf_size);

// Insert/eject floppy disk
esp_err_t atari_st_insert_floppy(int drive, const char *path);
void atari_st_eject_floppy(int drive);

// Send keyboard scancode (Atari ST IKBD protocol)
void atari_st_key_press(uint8_t scancode);
void atari_st_key_release(uint8_t scancode);

// Render current screen to LCD
void atari_st_render_screen(void);

// Cleanup
void atari_st_destroy(void);

#endif // ATARI_ST_H
