/**
 * @file amiga.h
 * @brief Commodore Amiga Hardware Emulation for ESP32-P4
 *
 * Emulates the essential hardware of a Commodore Amiga 500:
 *   - Motorola 68000 CPU @ 7.09 MHz (PAL) via m68k_emulator.c
 *   - Agnus: DMA controller, Copper coprocessor, Blitter
 *   - Denise: Bitplane video (up to 6 planes, OCS)
 *   - Paula: Interrupt controller, Disk controller, Audio (stub)
 *   - CIA 8520 x2: Timers, keyboard, disk control, TOD
 *   - Floppy disk: ADF image support (880KB)
 *   - Hard disk: HDF image via emulated IDE controller
 *
 * Memory Map (68000 24-bit address bus):
 *   $000000-$07FFFF : Chip RAM (512KB default, up to 2MB)
 *   $BFD000-$BFDFFF : CIA-B (even addresses)
 *   $BFE001-$BFEFFF : CIA-A (odd addresses)
 *   $C00000-$D7FFFF : Slow (Ranger) RAM (optional)
 *   $DFF000-$DFF1FF : Custom chip registers
 *   $E80000-$E8FFFF : Autoconfig (Zorro II)
 *   $F80000-$FFFFFF : Kickstart ROM (256KB or 512KB)
 */

#ifndef AMIGA_H
#define AMIGA_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Memory Map Constants
// ============================================================================

#define AMIGA_CHIP_RAM_DEFAULT  (512 * 1024)     // 512KB default Chip RAM
#define AMIGA_CHIP_RAM_MAX      (2 * 1024 * 1024) // 2MB max Chip RAM
#define AMIGA_SLOW_RAM_BASE     0xC00000
#define AMIGA_SLOW_RAM_SIZE     (512 * 1024)     // 512KB Slow RAM

#define AMIGA_CIA_A_BASE        0xBFE001         // CIA-A (odd addresses, $100 spacing)
#define AMIGA_CIA_B_BASE        0xBFD000         // CIA-B (even addresses, $100 spacing)
#define AMIGA_CUSTOM_BASE       0xDFF000         // Custom chip registers
#define AMIGA_CUSTOM_SIZE       0x200            // 512 bytes of registers

#define AMIGA_AUTOCONFIG_BASE   0xE80000         // Autoconfig space
#define AMIGA_AUTOCONFIG_SIZE   0x10000          // 64KB

#define AMIGA_KICK_256K_BASE    0xFC0000         // KS 1.x ROM (256KB)
#define AMIGA_KICK_512K_BASE    0xF80000         // KS 2.0+ ROM (512KB)
#define AMIGA_KICK_256K_SIZE    (256 * 1024)
#define AMIGA_KICK_512K_SIZE    (512 * 1024)

// Extended ROM (KS 2.0+ extended, stub)
#define AMIGA_EXT_ROM_BASE      0xE00000
#define AMIGA_EXT_ROM_SIZE      (512 * 1024)

// ============================================================================
// Custom Chip Register Offsets (relative to $DFF000)
// ============================================================================

// Read-only registers
#define CUSTOM_DMACONR  0x002   // DMA control read
#define CUSTOM_VPOSR    0x004   // Beam position V (high + LOF)
#define CUSTOM_VHPOSR   0x006   // Beam position V/H
#define CUSTOM_DSKDATR  0x008   // Disk DMA data read (unused)
#define CUSTOM_JOY0DAT  0x00A   // Joystick 0 data
#define CUSTOM_JOY1DAT  0x00C   // Joystick 1 data
#define CUSTOM_CLXDAT   0x00E   // Collision data
#define CUSTOM_ADKCONR  0x010   // Audio/disk control read
#define CUSTOM_POT0DAT  0x012   // Pot counter 0
#define CUSTOM_POT1DAT  0x014   // Pot counter 1
#define CUSTOM_POTGOR   0x016   // Pot pin data read
#define CUSTOM_SERDATR  0x018   // Serial port data read
#define CUSTOM_DSKBYTR  0x01A   // Disk byte and status
#define CUSTOM_INTENAR  0x01C   // Interrupt enable read
#define CUSTOM_INTREQR  0x01E   // Interrupt request read
#define CUSTOM_DENISEID 0x07C   // Denise ID (ECS)

// Write-only registers
#define CUSTOM_DSKPTH   0x020   // Disk DMA pointer high
#define CUSTOM_DSKPTL   0x022   // Disk DMA pointer low
#define CUSTOM_DSKLEN   0x024   // Disk DMA length
#define CUSTOM_DSKDAT   0x026   // Disk DMA data write
#define CUSTOM_DSKSYNC  0x07E   // Disk sync pattern
#define CUSTOM_COPCON   0x02E   // Copper control

// Blitter registers
#define CUSTOM_BLTCON0  0x040   // Blitter control 0
#define CUSTOM_BLTCON1  0x042   // Blitter control 1
#define CUSTOM_BLTAFWM  0x044   // Blitter first word mask A
#define CUSTOM_BLTALWM  0x046   // Blitter last word mask A
#define CUSTOM_BLTCPTH  0x048   // Blitter source C pointer high
#define CUSTOM_BLTCPTL  0x04A
#define CUSTOM_BLTBPTH  0x04C   // Blitter source B pointer high
#define CUSTOM_BLTBPTL  0x04E
#define CUSTOM_BLTAPTH  0x050   // Blitter source A pointer high
#define CUSTOM_BLTAPTL  0x052
#define CUSTOM_BLTDPTH  0x054   // Blitter dest D pointer high
#define CUSTOM_BLTDPTL  0x056
#define CUSTOM_BLTSIZE  0x058   // Blitter size (triggers blit)
#define CUSTOM_BLTCMOD  0x060   // Blitter source C modulo
#define CUSTOM_BLTBMOD  0x062   // Blitter source B modulo
#define CUSTOM_BLTAMOD  0x064   // Blitter source A modulo
#define CUSTOM_BLTDMOD  0x066   // Blitter dest D modulo
#define CUSTOM_BLTCDAT  0x070   // Blitter source C data
#define CUSTOM_BLTBDAT  0x072   // Blitter source B data
#define CUSTOM_BLTADAT  0x074   // Blitter source A data

// Copper registers
#define CUSTOM_COP1LCH  0x080   // Copper list 1 location high
#define CUSTOM_COP1LCL  0x082
#define CUSTOM_COP2LCH  0x084   // Copper list 2 location high
#define CUSTOM_COP2LCL  0x086
#define CUSTOM_COPJMP1  0x088   // Copper jump to list 1
#define CUSTOM_COPJMP2  0x08A   // Copper jump to list 2

// Display window
#define CUSTOM_DIWSTRT  0x08E   // Display window start
#define CUSTOM_DIWSTOP  0x090   // Display window stop
#define CUSTOM_DDFSTRT  0x092   // Data fetch start
#define CUSTOM_DDFSTOP  0x094   // Data fetch stop

// DMA and interrupt control (accent write)
#define CUSTOM_DMACON   0x096   // DMA control write
#define CUSTOM_CLXCON   0x098   // Collision control
#define CUSTOM_INTENA   0x09A   // Interrupt enable write
#define CUSTOM_INTREQ   0x09C   // Interrupt request write
#define CUSTOM_ADKCON   0x09E   // Audio/disk control write

// Bitplane pointers
#define CUSTOM_BPL1PTH  0x0E0
#define CUSTOM_BPL1PTL  0x0E2
#define CUSTOM_BPL2PTH  0x0E4
#define CUSTOM_BPL2PTL  0x0E6
#define CUSTOM_BPL3PTH  0x0E8
#define CUSTOM_BPL3PTL  0x0EA
#define CUSTOM_BPL4PTH  0x0EC
#define CUSTOM_BPL4PTL  0x0EE
#define CUSTOM_BPL5PTH  0x0F0
#define CUSTOM_BPL5PTL  0x0F2
#define CUSTOM_BPL6PTH  0x0F4
#define CUSTOM_BPL6PTL  0x0F6

// Bitplane control
#define CUSTOM_BPLCON0  0x100   // Bitplane control 0
#define CUSTOM_BPLCON1  0x102   // Bitplane control 1 (scroll)
#define CUSTOM_BPLCON2  0x104   // Bitplane control 2 (priority)
#define CUSTOM_BPL1MOD  0x108   // Odd bitplane modulo
#define CUSTOM_BPL2MOD  0x10A   // Even bitplane modulo
// Bitplane data registers
#define CUSTOM_BPL1DAT  0x110
#define CUSTOM_BPL2DAT  0x112
#define CUSTOM_BPL3DAT  0x114
#define CUSTOM_BPL4DAT  0x116
#define CUSTOM_BPL5DAT  0x118
#define CUSTOM_BPL6DAT  0x11A

// Sprite registers (8 sprites)
#define CUSTOM_SPR0PTH  0x120
#define CUSTOM_SPR0PTL  0x122
// ... SPR1-7 at +8 each

// Color palette
#define CUSTOM_COLOR00  0x180   // 32 color registers ($180-$1BE)

// ============================================================================
// Interrupt Bit Definitions (INTENA/INTREQ)
// ============================================================================

#define AMIGA_INTF_SETCLR   0x8000  // Bit 15: set/clear control
#define AMIGA_INTF_INTEN    0x4000  // Bit 14: master interrupt enable
#define AMIGA_INTF_EXTER    0x2000  // Bit 13: CIA-B (level 6)
#define AMIGA_INTF_DSKSYN   0x1000  // Bit 12: disk sync (level 5)
#define AMIGA_INTF_RBF      0x0800  // Bit 11: serial receive (level 5)
#define AMIGA_INTF_AUD3     0x0400  // Bit 10: audio 3 (level 4)
#define AMIGA_INTF_AUD2     0x0200  // Bit 9: audio 2 (level 4)
#define AMIGA_INTF_AUD1     0x0100  // Bit 8: audio 1 (level 4)
#define AMIGA_INTF_AUD0     0x0080  // Bit 7: audio 0 (level 4)
#define AMIGA_INTF_BLIT     0x0040  // Bit 6: blitter done (level 3)
#define AMIGA_INTF_VERTB    0x0020  // Bit 5: vertical blank (level 3)
#define AMIGA_INTF_COPER    0x0010  // Bit 4: copper (level 3)
#define AMIGA_INTF_PORTS    0x0008  // Bit 3: CIA-A/ports (level 2)
#define AMIGA_INTF_SOFT     0x0004  // Bit 2: software (level 1)
#define AMIGA_INTF_DSKBLK   0x0002  // Bit 1: disk block done (level 1)
#define AMIGA_INTF_TBE      0x0001  // Bit 0: serial transmit (level 1)

// ============================================================================
// DMA Control Bit Definitions (DMACON)
// ============================================================================

#define AMIGA_DMAF_SETCLR   0x8000  // Bit 15: set/clear
#define AMIGA_DMAF_BBUSY    0x4000  // Bit 14: blitter busy (read-only)
#define AMIGA_DMAF_BZERO    0x2000  // Bit 13: blitter zero (read-only)
#define AMIGA_DMAF_BLTPRI   0x0400  // Bit 10: blitter priority
#define AMIGA_DMAF_DMAEN    0x0200  // Bit 9: master DMA enable
#define AMIGA_DMAF_BPLEN    0x0100  // Bit 8: bitplane DMA
#define AMIGA_DMAF_COPEN    0x0080  // Bit 7: copper DMA
#define AMIGA_DMAF_BLTEN    0x0040  // Bit 6: blitter DMA
#define AMIGA_DMAF_SPREN    0x0020  // Bit 5: sprite DMA
#define AMIGA_DMAF_DSKEN    0x0010  // Bit 4: disk DMA
#define AMIGA_DMAF_AUD3EN   0x0008  // Bit 3: audio 3 DMA
#define AMIGA_DMAF_AUD2EN   0x0004  // Bit 2: audio 2 DMA
#define AMIGA_DMAF_AUD1EN   0x0002  // Bit 1: audio 1 DMA
#define AMIGA_DMAF_AUD0EN   0x0001  // Bit 0: audio 0 DMA

// ============================================================================
// CIA Register Offsets (relative to CIA base, spaced $100 apart)
// ============================================================================

#define CIA_PRA     0x000
#define CIA_PRB     0x100
#define CIA_DDRA    0x200
#define CIA_DDRB    0x300
#define CIA_TALO    0x400
#define CIA_TAHI    0x500
#define CIA_TBLO    0x600
#define CIA_TBHI    0x700
#define CIA_TODLO   0x800
#define CIA_TODMID  0x900
#define CIA_TODHI   0xA00
#define CIA_SDR     0xC00
#define CIA_ICR     0xD00
#define CIA_CRA     0xE00
#define CIA_CRB     0xF00

// CIA ICR bits
#define CIA_ICR_TA      0x01    // Timer A underflow
#define CIA_ICR_TB      0x02    // Timer B underflow
#define CIA_ICR_ALARM   0x04    // TOD alarm
#define CIA_ICR_SP      0x08    // Serial port (keyboard for CIA-A)
#define CIA_ICR_FLG     0x10    // FLAG pin (/INDEX for CIA-B)
#define CIA_ICR_SETCLR  0x80    // Set/clear control

// CIA-A PRA bits
#define CIAA_PRA_OVL    0x01    // Overlay (ROM at $000000 when set)
#define CIAA_PRA_LED    0x02    // Power LED (active low)
#define CIAA_PRA_CHNG   0x04    // Disk change
#define CIAA_PRA_WPROT  0x08    // Write protect
#define CIAA_PRA_TK0    0x10    // Track 0
#define CIAA_PRA_RDY    0x20    // Disk ready
#define CIAA_PRA_FIR1   0x40    // Gameport 1 fire
#define CIAA_PRA_FIR0   0x80    // Gameport 0 fire

// CIA-B PRA bits (accent active low except DIR)
#define CIAB_PRA_STEP   0x01    // /STEP pulse
#define CIAB_PRA_DIR    0x02    // Direction (0=inward, 1=outward)
#define CIAB_PRA_SIDE   0x04    // /SIDE select (0=upper, 1=lower)
#define CIAB_PRA_SEL0   0x08    // /SEL0 drive 0
#define CIAB_PRA_SEL1   0x10    // /SEL1 drive 1
#define CIAB_PRA_SEL2   0x20    // /SEL2 drive 2
#define CIAB_PRA_SEL3   0x40    // /SEL3 drive 3
#define CIAB_PRA_MTR    0x80    // /MTR motor

// ============================================================================
// Floppy Disk Constants
// ============================================================================

#define AMIGA_ADF_SIZE          901120  // 880KB standard ADF
#define AMIGA_ADF_TRACKS        80
#define AMIGA_ADF_SIDES         2
#define AMIGA_ADF_SECTORS       11
#define AMIGA_ADF_SECTOR_SIZE   512
#define AMIGA_ADF_TRACK_SIZE    (AMIGA_ADF_SECTORS * AMIGA_ADF_SECTOR_SIZE) // 5632 bytes
#define AMIGA_MAX_DRIVES        4

// ============================================================================
// Display Constants
// ============================================================================

#define AMIGA_LORES_W       320
#define AMIGA_LORES_H       200     // NTSC visible
#define AMIGA_PAL_H         256     // PAL visible
#define AMIGA_HIRES_W       640

// PAL timing
#define AMIGA_PAL_LINES     312     // Total lines per frame (PAL)
#define AMIGA_NTSC_LINES    262     // Total lines per frame (NTSC)
#define AMIGA_PAL_HCLOCKS   227     // Color clocks per line
#define AMIGA_CPU_FREQ      7093790 // 7.09 MHz (PAL)

// ============================================================================
// Configuration Structure
// ============================================================================

typedef struct {
    uint32_t    chip_ram_size;      // Chip RAM size (default 512KB)
    bool        slow_ram_enabled;   // Enable 512KB slow RAM at $C00000
    const char *kick_path;          // Path to Kickstart ROM file
    const char *floppy_path[AMIGA_MAX_DRIVES]; // Floppy ADF images (DF0:-DF3:)
    const char *hdf_path[2];        // Hard disk HDF images (DH0:-DH1:)
    bool        enable_lcd_output;  // Enable LCD rendering via GPU
    bool        pal_mode;           // true=PAL (50Hz), false=NTSC (60Hz)
} amiga_config_t;

// ============================================================================
// Public API
// ============================================================================

/** Initialize the Amiga emulator (loads Kickstart, sets up hardware) */
esp_err_t amiga_init(const amiga_config_t *config);

/** Cold reset the Amiga */
void amiga_reset(void);

/** Run the emulator (call from FreeRTOS task - runs until stopped) */
void amiga_run(void);

/** Stop the emulator */
void amiga_stop(void);

/** Check if emulator is running */
bool amiga_is_running(void);

/** Get emulator status string */
void amiga_get_state(char *buf, size_t buf_size);

/** Insert/eject floppy disk (drive 0-3) */
esp_err_t amiga_insert_floppy(int drive, const char *path);
void amiga_eject_floppy(int drive);

/** Mount/unmount hard disk image (unit 0-1) */
esp_err_t amiga_mount_hdf(int unit, const char *path);
void amiga_unmount_hdf(int unit);


/** Cleanup and free all resources */
void amiga_destroy(void);

#ifdef __cplusplus
}
#endif

#endif // AMIGA_H
