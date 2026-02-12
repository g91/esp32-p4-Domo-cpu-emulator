/**
 * @file amiga.c
 * @brief Commodore Amiga Hardware Emulation Implementation
 *
 * Complete Amiga 500 hardware emulation:
 *   - CIA 8520 x2 (timers, keyboard, disk control)
 *   - Paula (interrupt controller, disk DMA)
 *   - Agnus (DMA control, copper, blitter)
 *   - Denise (bitplane video, color palette)
 *   - Floppy disk (ADF image support)
 *   - IDE hard disk (HDF image support via Autoconfig)
 *   - Keyboard (PS/2/USB → Amiga scancode via CIA-A SDR)
 *   - Video rendering (bitplane → RGB565 via GPU pipeline)
 */

#include "amiga.h"
#include "m68k_emulator.h"
#include "video_card.h"
#include "ps2_keyboard.h"
#include "usb_keyboard.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "amiga";

// External console_printf from main
extern int console_printf(const char *fmt, ...);

// ============================================================================
// CIA 8520 State
// ============================================================================

typedef struct {
    uint8_t  pra;           // Port A data register
    uint8_t  prb;           // Port B data register
    uint8_t  ddra;          // Direction A (1=output)
    uint8_t  ddrb;          // Direction B
    uint16_t timer_a;       // Timer A counter
    uint16_t timer_a_latch; // Timer A reload value
    uint16_t timer_b;       // Timer B counter
    uint16_t timer_b_latch; // Timer B reload value
    uint32_t tod;           // Time-of-day counter (24-bit)
    uint32_t tod_alarm;     // TOD alarm
    uint32_t tod_latch;     // TOD latched value
    bool     tod_latched;   // TOD is latched (freeze on read hi)
    bool     tod_stopped;   // TOD write in progress
    uint8_t  sdr;           // Serial data register
    uint8_t  icr_data;      // ICR pending flags (what happened)
    uint8_t  icr_mask;      // ICR mask (what's enabled)
    uint8_t  cra;           // Control register A
    uint8_t  crb;           // Control register B
    bool     ta_running;    // Timer A is running
    bool     tb_running;    // Timer B is running
    // Keyboard shift register state (CIA-A only)
    bool     sdr_pending;   // SDR has data to deliver
    int      sdr_bit_count; // Bits shifted
} cia8520_t;

// ============================================================================
// Floppy Drive State
// ============================================================================

typedef struct {
    FILE    *image;             // ADF file handle (NULL = no disk)
    char     path[256];         // Image file path
    uint32_t size;              // Image file size
    int      track;             // Current head position (0-79)
    bool     motor_on;          // Motor is running
    bool     write_protected;   // Disk is write-protected
    bool     disk_changed;      // Disk was changed since last check
    bool     inserted;          // Disk is inserted
} floppy_drive_t;

// ============================================================================
// Blitter State
// ============================================================================

typedef struct {
    uint16_t bltcon0;
    uint16_t bltcon1;
    uint16_t bltafwm;       // First word mask A
    uint16_t bltalwm;       // Last word mask A
    uint32_t bltcpt;        // Source C pointer
    uint32_t bltbpt;        // Source B pointer
    uint32_t bltapt;        // Source A pointer
    uint32_t bltdpt;        // Destination D pointer
    int16_t  bltcmod;       // Source C modulo
    int16_t  bltbmod;       // Source B modulo
    int16_t  bltamod;       // Source A modulo
    int16_t  bltdmod;       // Destination D modulo
    uint16_t bltcdat;       // Source C data
    uint16_t bltbdat;       // Source B data
    uint16_t bltadat;       // Source A data
    bool     busy;          // Blitter is busy
    bool     zero;          // Last blit result was zero
} blitter_t;

// ============================================================================
// Copper State
// ============================================================================

typedef struct {
    uint32_t cop1lc;        // Copper list 1 location
    uint32_t cop2lc;        // Copper list 2 location
    uint32_t pc;            // Current copper PC
    bool     enabled;       // Copper DMA enabled
    uint8_t  copcon;        // Copper control (danger bit)
} copper_t;

// ============================================================================
// IDE Hard Disk State
// ============================================================================

typedef struct {
    FILE    *image;
    char     path[256];
    uint32_t size;              // Image size in bytes
    uint32_t sectors;           // Total sectors (size / 512)
    bool     mounted;

    // IDE registers
    uint8_t  error;             // Error register
    uint8_t  sector_count;      // Sector count
    uint8_t  sector_num;        // Sector number (LBA low)
    uint8_t  cylinder_lo;       // Cylinder low (LBA mid)
    uint8_t  cylinder_hi;       // Cylinder high (LBA high)
    uint8_t  drive_head;        // Drive/head
    uint8_t  status;            // Status register
    uint8_t  command;           // Last command

    // Data transfer
    uint8_t  data_buf[512];     // Sector buffer
    int      data_pos;          // Current position in buffer
    int      data_len;          // Bytes to transfer
    bool     data_ready;        // Data buffer ready for read
} ide_drive_t;

// ============================================================================
// Autoconfig State
// ============================================================================

typedef struct {
    bool     configured;        // Board has been configured
    uint32_t base_addr;         // Assigned base address
    int      config_step;       // Current config register being read
} autoconfig_t;

// ============================================================================
// Complete Amiga State
// ============================================================================

typedef struct {
    bool initialized;
    bool running;
    bool stopped;

    // Configuration
    amiga_config_t config;

    // Kickstart ROM
    uint8_t *kick_rom;          // ROM data buffer
    uint32_t kick_size;         // ROM size (256KB or 512KB)
    uint32_t kick_base;         // ROM mapping address ($F80000 or $FC0000)
    bool     ovl;               // Overlay: ROM visible at $000000

    // CIA chips
    cia8520_t cia_a;            // CIA-A ($BFE001) - keyboard, joystick, OVL
    cia8520_t cia_b;            // CIA-B ($BFD000) - disk control, serial

    // Custom chip registers
    uint16_t intena;            // Interrupt enable
    uint16_t intreq;            // Interrupt request
    uint16_t dmacon;            // DMA control
    uint16_t adkcon;            // Audio/disk control

    // Beam position
    uint16_t vpos;              // Vertical beam position (0-311 PAL)
    uint16_t hpos;              // Horizontal beam position (0-226)
    bool     lof;               // Long frame (interlace)

    // Display
    uint16_t bplcon0;           // Bitplane control 0
    uint16_t bplcon1;           // Bitplane control 1 (scroll)
    uint16_t bplcon2;           // Bitplane control 2 (priority)
    int16_t  bpl1mod;           // Odd bitplane modulo
    int16_t  bpl2mod;           // Even bitplane modulo
    uint32_t bplpt[6];          // Bitplane pointers
    uint16_t diwstrt;           // Display window start
    uint16_t diwstop;           // Display window stop
    uint16_t ddfstrt;           // Data fetch start
    uint16_t ddfstop;           // Data fetch stop
    uint16_t color[32];         // Color palette (12-bit 0x0RGB)
    uint16_t color_rgb565[32];  // Pre-converted to RGB565

    // Sprite pointers (8 sprites)
    uint32_t sprpt[8];

    // Disk DMA
    uint32_t dskpt;            // Disk DMA pointer
    uint16_t dsklen;           // Disk DMA length
    uint16_t dsksync;          // Disk sync word
    uint16_t prev_dsklen;      // Previous DSKLEN write (double-write detection)

    // Copper
    copper_t copper;

    // Blitter
    blitter_t blitter;

    // Floppy drives
    floppy_drive_t floppy[AMIGA_MAX_DRIVES];
    int      current_drive;     // Currently selected drive (-1 = none)
    int      disk_side;         // Current side (0 or 1)
    uint8_t  prev_ciab_pra;     // Previous CIA-B PRA for edge detection

    // IDE hard disk
    ide_drive_t ide[2];         // Two IDE drives
    autoconfig_t autoconfig;    // Autoconfig state

    // Keyboard
    uint8_t  kbd_buffer[32];    // Keyboard scancode buffer
    int      kbd_head;
    int      kbd_tail;
    int      kbd_count;
    bool     kbd_handshake;     // Host has acknowledged last key

    // Timing
    uint64_t cycles;            // Total CPU cycles
    uint32_t vbl_cycles;        // CPU cycles per frame
    uint32_t line_cycles;       // CPU cycles per line
    uint32_t cycles_this_line;  // Cycle counter within current line
    uint32_t frame_count;       // Total frames

    // IRQ
    uint8_t  active_irq_level;  // Current IRQ level asserted to M68K
    bool     prev_cia_a_int;    // Previous CIA-A INT pin state (true = asserted)
    bool     prev_cia_b_int;    // Previous CIA-B INT pin state (true = asserted)

    // Debug / diagnostic
    uint32_t hw_reset_count;    // Number of RESET instructions executed
    bool     first_vbl_logged;  // One-time VBL interrupt log flag
} amiga_state_t;

static amiga_state_t *ami = NULL;

// ============================================================================
// Forward Declarations
// ============================================================================

static uint32_t amiga_io_read_hook(uint32_t addr, int size, bool *handled);
static void amiga_io_write_hook(uint32_t addr, uint32_t value, int size, bool *handled);
static uint8_t amiga_irq_ack(uint8_t level);
static void update_irq_level(void);
static void cia_tick(cia8520_t *cia, int cycles, bool is_cia_b);
static void copper_execute_line(void);
static void blitter_execute(uint16_t bltsize);
static void disk_dma_transfer(void);
static void render_bitplanes(void);
static void kbd_push_scancode(uint8_t code);
static void kbd_deliver(void);
static void amiga_hw_reset(void);

// ============================================================================
// Amiga to RGB565 Color Conversion
// ============================================================================

// Convert Amiga 12-bit color (0x0RGB, 4 bits per channel) to RGB565
static inline uint16_t amiga_color_to_rgb565(uint16_t amiga_color) {
    uint8_t r4 = (amiga_color >> 8) & 0x0F;
    uint8_t g4 = (amiga_color >> 4) & 0x0F;
    uint8_t b4 = amiga_color & 0x0F;
    // Expand 4-bit to 5/6/5 bit
    uint16_t r5 = (r4 << 1) | (r4 >> 3);
    uint16_t g6 = (g4 << 2) | (g4 >> 2);
    uint16_t b5 = (b4 << 1) | (b4 >> 3);
    return (r5 << 11) | (g6 << 5) | b5;
}

// ============================================================================
// PS/2 to Amiga Scancode Mapping
// ============================================================================

// Map ASCII/PS2 codes to Amiga raw key codes
// Amiga raw key codes are different from PC scancodes
static const uint8_t ps2_to_amiga_scancode[128] = {
    [0x00] = 0xFF, // No key
    [0x1B] = 0x45, // ESC
    [0x08] = 0x41, // Backspace
    [0x09] = 0x42, // Tab
    [0x0D] = 0x44, // Return
    [0x20] = 0x40, // Space
    // Number row
    ['1']  = 0x01, ['2']  = 0x02, ['3']  = 0x03, ['4']  = 0x04,
    ['5']  = 0x05, ['6']  = 0x06, ['7']  = 0x07, ['8']  = 0x08,
    ['9']  = 0x09, ['0']  = 0x0A,
    // Letters (lowercase)
    ['a']  = 0x20, ['b']  = 0x35, ['c']  = 0x33, ['d']  = 0x22,
    ['e']  = 0x12, ['f']  = 0x23, ['g']  = 0x24, ['h']  = 0x25,
    ['i']  = 0x17, ['j']  = 0x26, ['k']  = 0x27, ['l']  = 0x28,
    ['m']  = 0x37, ['n']  = 0x36, ['o']  = 0x18, ['p']  = 0x19,
    ['q']  = 0x10, ['r']  = 0x13, ['s']  = 0x21, ['t']  = 0x14,
    ['u']  = 0x16, ['v']  = 0x34, ['w']  = 0x11, ['x']  = 0x32,
    ['y']  = 0x15, ['z']  = 0x31,
    // Letters (uppercase → same scancode, shift handled separately)
    ['A']  = 0x20, ['B']  = 0x35, ['C']  = 0x33, ['D']  = 0x22,
    ['E']  = 0x12, ['F']  = 0x23, ['G']  = 0x24, ['H']  = 0x25,
    ['I']  = 0x17, ['J']  = 0x26, ['K']  = 0x27, ['L']  = 0x28,
    ['M']  = 0x37, ['N']  = 0x36, ['O']  = 0x18, ['P']  = 0x19,
    ['Q']  = 0x10, ['R']  = 0x13, ['S']  = 0x21, ['T']  = 0x14,
    ['U']  = 0x16, ['V']  = 0x34, ['W']  = 0x11, ['X']  = 0x32,
    ['Y']  = 0x15, ['Z']  = 0x31,
    // Punctuation
    ['-']  = 0x0B, ['=']  = 0x0C, ['[']  = 0x1A, [']']  = 0x1B,
    ['\\'] = 0x0D, [';']  = 0x29, ['\''] = 0x2A, ['`']  = 0x00,
    [',']  = 0x38, ['.']  = 0x39, ['/']  = 0x3A,
    // Function keys (F1-F10 mapped to 0x50-0x59)
};

// ============================================================================
// CIA 8520 Implementation
// ============================================================================

static void cia_init(cia8520_t *cia) {
    memset(cia, 0, sizeof(cia8520_t));
    cia->timer_a_latch = 0xFFFF;
    cia->timer_b_latch = 0xFFFF;
    cia->timer_a = 0xFFFF;
    cia->timer_b = 0xFFFF;
}

static uint8_t cia_read(cia8520_t *cia, uint16_t reg, bool is_cia_b) {
    switch (reg) {
        case CIA_PRA: {
            uint8_t val = cia->pra;
            if (!is_cia_b) {
                // CIA-A PRA: output bits from PRA register, input bits from hardware
                val &= cia->ddra;   // Keep only output bits (OVL, LED)
                uint8_t input = 0xFF; // All inputs HIGH by default (active low signals)
                // Bits 7-6: Gameport fire buttons (active low, no button = high)
                // Already 0xFF so FIR0, FIR1 are high (not pressed)

                // Bits 5-2: Floppy drive status (active low)
                // These come from the currently selected drive
                if (ami->current_drive >= 0 && ami->current_drive < AMIGA_MAX_DRIVES) {
                    floppy_drive_t *drv = &ami->floppy[ami->current_drive];
                    if (drv->inserted && drv->disk_changed) {
                        input &= ~CIAA_PRA_CHNG;   // bit 2: /CHNG low = disk changed
                    }
                    if (drv->inserted && drv->write_protected) {
                        input &= ~CIAA_PRA_WPROT;  // bit 3: /WPROT low = write protected
                    }
                    if (drv->track == 0) {
                        input &= ~CIAA_PRA_TK0;    // bit 4: /TK0 low = at track 0
                    }
                    if (drv->inserted && drv->motor_on) {
                        input &= ~CIAA_PRA_RDY;    // bit 5: /RDY low = drive ready
                    }
                }
                // No drive selected or no disk: all status bits stay HIGH (inactive)
                val |= (input & ~cia->ddra);
            }
            return val;
        }
        case CIA_PRB: {
            uint8_t val = cia->prb;
            if (is_cia_b) {
                // CIA-B PRB: parallel port data register
                // Input bits read from hardware (accent accent accent accent default to 0xFF = inactive)
                val = (cia->prb & cia->ddrb) | (0xFF & ~cia->ddrb);
            }
            return val;
        }
        case CIA_DDRA:  return cia->ddra;
        case CIA_DDRB:  return cia->ddrb;
        case CIA_TALO:  return cia->timer_a & 0xFF;
        case CIA_TAHI:  return (cia->timer_a >> 8) & 0xFF;
        case CIA_TBLO:  return cia->timer_b & 0xFF;
        case CIA_TBHI:  return (cia->timer_b >> 8) & 0xFF;
        case CIA_TODLO: {
            uint32_t val = cia->tod_latched ? cia->tod_latch : cia->tod;
            cia->tod_latched = false; // Unlatch on low read
            return val & 0xFF;
        }
        case CIA_TODMID: return (cia->tod >> 8) & 0xFF;
        case CIA_TODHI: {
            // Reading high byte latches the TOD
            cia->tod_latch = cia->tod;
            cia->tod_latched = true;
            return (cia->tod >> 16) & 0xFF;
        }
        case CIA_SDR:   return cia->sdr;
        case CIA_ICR: {
            // Read ICR: returns pending flags and clears them
            uint8_t val = cia->icr_data;
            // Set bit 7 if any enabled interrupt is pending
            if (val & cia->icr_mask) {
                val |= 0x80;
            }
            cia->icr_data = 0; // Clear on read
            // Re-evaluate interrupts
            update_irq_level();
            return val;
        }
        case CIA_CRA:   return cia->cra;
        case CIA_CRB:   return cia->crb;
        default:        return 0xFF;
    }
}

static void cia_write(cia8520_t *cia, uint16_t reg, uint8_t value, bool is_cia_b) {
    switch (reg) {
        case CIA_PRA:
            cia->pra = value;
            if (!is_cia_b) {
                // CIA-A PRA bit 0 = OVL
                // Only update OVL when bit 0 is configured as output (DDRA bit0=1).
                // When input (DDRA=0), the physical pin has a pullup (HIGH), but
                // we skip modeling that transient to avoid corrupting chip RAM
                // reads in our non-cycle-accurate emulator.
                if (cia->ddra & CIAA_PRA_OVL) {
                    bool new_ovl = (value & CIAA_PRA_OVL) != 0;
                    if (ami->ovl && !new_ovl) {
                        ESP_LOGI(TAG, "OVL cleared - Chip RAM now at $000000");
                    }
                    ami->ovl = new_ovl;
                }
            }
            if (is_cia_b) {
                // CIA-B PRA: disk control - detect step pulses
                uint8_t old = ami->prev_ciab_pra;
                uint8_t new_val = value;

                // Determine selected drive (active low)
                ami->current_drive = -1;
                if (!(new_val & CIAB_PRA_SEL0)) ami->current_drive = 0;
                else if (!(new_val & CIAB_PRA_SEL1)) ami->current_drive = 1;
                else if (!(new_val & CIAB_PRA_SEL2)) ami->current_drive = 2;
                else if (!(new_val & CIAB_PRA_SEL3)) ami->current_drive = 3;

                // Motor control
                if (ami->current_drive >= 0 && ami->current_drive < AMIGA_MAX_DRIVES) {
                    ami->floppy[ami->current_drive].motor_on = !(new_val & CIAB_PRA_MTR);
                }

                // Side select (active low: 0=upper/side 1, 1=lower/side 0)
                ami->disk_side = (new_val & CIAB_PRA_SIDE) ? 0 : 1;

                // Step detection: /STEP goes low→high
                if ((old & CIAB_PRA_STEP) == 0 && (new_val & CIAB_PRA_STEP) != 0) {
                    if (ami->current_drive >= 0 && ami->current_drive < AMIGA_MAX_DRIVES) {
                        floppy_drive_t *drv = &ami->floppy[ami->current_drive];
                        if (new_val & CIAB_PRA_DIR) {
                            // Step outward (toward track 0)
                            if (drv->track > 0) drv->track--;
                        } else {
                            // Step inward (toward track 79)
                            if (drv->track < AMIGA_ADF_TRACKS - 1) drv->track++;
                        }
                    }
                }

                ami->prev_ciab_pra = new_val;
            }
            break;

        case CIA_PRB:
            cia->prb = value;
            break;

        case CIA_DDRA:
            cia->ddra = value;
            if (!is_cia_b) {
                // Re-evaluate OVL when DDRA changes direction of bit 0
                // OVL pin = output: (PRA & DDRA), input: external pullup (HIGH)
                bool ovl_output = (cia->pra & value & CIAA_PRA_OVL) != 0;
                bool ovl_input = ((~value) & CIAA_PRA_OVL) != 0; // pullup HIGH when input
                bool new_ovl = ovl_output || ovl_input;
                if (ami->ovl && !new_ovl) {
                    ESP_LOGI(TAG, "OVL cleared (DDRA change) - Chip RAM now at $000000");
                }
                ami->ovl = new_ovl;
            }
            break;

        case CIA_DDRB:
            cia->ddrb = value;
            break;

        case CIA_TALO:
            cia->timer_a_latch = (cia->timer_a_latch & 0xFF00) | value;
            break;

        case CIA_TAHI:
            cia->timer_a_latch = (cia->timer_a_latch & 0x00FF) | (value << 8);
            // If timer not running, load latch into counter
            if (!cia->ta_running) {
                cia->timer_a = cia->timer_a_latch;
            }
            break;

        case CIA_TBLO:
            cia->timer_b_latch = (cia->timer_b_latch & 0xFF00) | value;
            break;

        case CIA_TBHI:
            cia->timer_b_latch = (cia->timer_b_latch & 0x00FF) | (value << 8);
            if (!cia->tb_running) {
                cia->timer_b = cia->timer_b_latch;
            }
            break;

        case CIA_TODLO:
            if (cia->crb & 0x80) {
                // Set alarm
                cia->tod_alarm = (cia->tod_alarm & 0xFFFF00) | value;
            } else {
                cia->tod = (cia->tod & 0xFFFF00) | value;
                cia->tod_stopped = false; // Start TOD
            }
            break;

        case CIA_TODMID:
            if (cia->crb & 0x80) {
                cia->tod_alarm = (cia->tod_alarm & 0xFF00FF) | (value << 8);
            } else {
                cia->tod = (cia->tod & 0xFF00FF) | (value << 8);
            }
            break;

        case CIA_TODHI:
            if (cia->crb & 0x80) {
                cia->tod_alarm = (cia->tod_alarm & 0x00FFFF) | (value << 16);
            } else {
                cia->tod = (cia->tod & 0x00FFFF) | (value << 16);
                cia->tod_stopped = true; // Stop TOD until low written
            }
            break;

        case CIA_SDR:
            cia->sdr = value;
            if (!is_cia_b) {
                // CIA-A SDR write = keyboard handshake acknowledge
                ami->kbd_handshake = true;
            }
            break;

        case CIA_ICR:
            if (value & CIA_ICR_SETCLR) {
                // Set bits
                cia->icr_mask |= (value & 0x1F);
            } else {
                // Clear bits
                cia->icr_mask &= ~(value & 0x1F);
            }
            update_irq_level();
            break;

        case CIA_CRA:
            cia->cra = value;
            cia->ta_running = (value & 0x01) != 0;
            if (value & 0x10) {
                // Force load
                cia->timer_a = cia->timer_a_latch;
            }
            break;

        case CIA_CRB:
            cia->crb = value;
            cia->tb_running = (value & 0x01) != 0;
            if (value & 0x10) {
                cia->timer_b = cia->timer_b_latch;
            }
            break;
    }
}

// Tick CIA timers. CIA timers run at ~709379 Hz (CPU clock / 10 for E clock,
// but timers actually count at CPU clock for mode 0).
// For simplicity, we tick timers at a scaled rate.
static void cia_tick(cia8520_t *cia, int cycles, bool is_cia_b) {
    // Timer A
    if (cia->ta_running) {
        // Mode: count CPU clocks (simplified - real CIA counts E clocks or CNT)
        int mode = (cia->cra >> 5) & 0x01; // 0 = system clock, 1 = CNT pin
        if (mode == 0) {
            // Scale: CIA runs at E clock (CPU/10 = ~709 KHz)
            // We simplify: decrement by cycles/10 (rounded)
            int ticks = cycles;  // We'll use direct CPU cycles for responsiveness
            while (ticks > 0 && cia->ta_running) {
                if (cia->timer_a <= (uint16_t)ticks) {
                    ticks -= cia->timer_a + 1;
                    // Timer A underflow
                    cia->icr_data |= CIA_ICR_TA;
                    if (cia->cra & 0x08) {
                        // One-shot mode: stop timer
                        cia->ta_running = false;
                        cia->cra &= ~0x01;
                    }
                    cia->timer_a = cia->timer_a_latch; // Reload
                    update_irq_level();
                } else {
                    cia->timer_a -= ticks;
                    ticks = 0;
                }
            }
        }
    }

    // Timer B
    if (cia->tb_running) {
        int mode = (cia->crb >> 5) & 0x03;
        if (mode == 0) {
            int ticks = cycles;
            while (ticks > 0 && cia->tb_running) {
                if (cia->timer_b <= (uint16_t)ticks) {
                    ticks -= cia->timer_b + 1;
                    cia->icr_data |= CIA_ICR_TB;
                    if (cia->crb & 0x08) {
                        cia->tb_running = false;
                        cia->crb &= ~0x01;
                    }
                    cia->timer_b = cia->timer_b_latch;
                    update_irq_level();
                } else {
                    cia->timer_b -= ticks;
                    ticks = 0;
                }
            }
        }
    }
}

// ============================================================================
// Paula Interrupt Controller
// ============================================================================

// Map INTREQ bits to M68K interrupt levels
static uint8_t paula_irq_level(void) {
    if (!ami) return 0;

    // Master enable must be set
    if (!(ami->intena & AMIGA_INTF_INTEN)) return 0;

    uint16_t active = ami->intreq & ami->intena & 0x3FFF;
    if (!active) return 0;

    // Level 6: EXTER (CIA-B)
    if (active & AMIGA_INTF_EXTER) return 6;
    // Level 5: DSKSYN, RBF
    if (active & (AMIGA_INTF_DSKSYN | AMIGA_INTF_RBF)) return 5;
    // Level 4: AUD0-AUD3
    if (active & (AMIGA_INTF_AUD0 | AMIGA_INTF_AUD1 | AMIGA_INTF_AUD2 | AMIGA_INTF_AUD3)) return 4;
    // Level 3: BLIT, VERTB, COPER
    if (active & (AMIGA_INTF_BLIT | AMIGA_INTF_VERTB | AMIGA_INTF_COPER)) return 3;
    // Level 2: PORTS (CIA-A)
    if (active & AMIGA_INTF_PORTS) return 2;
    // Level 1: TBE, DSKBLK, SOFT
    if (active & (AMIGA_INTF_TBE | AMIGA_INTF_DSKBLK | AMIGA_INTF_SOFT)) return 1;

    return 0;
}

static void update_irq_level(void) {
    if (!ami) return;

    // Check CIA interrupts → set Paula INTREQ bits on EDGE (new interrupt only)
    // In real hardware, CIA /INT pin goes low when (icr_data & icr_mask) != 0.
    // Paula latches INTREQ bit on the falling edge of CIA /INT.
    // Reading CIA ICR clears icr_data, releasing /INT (goes high).
    // We only set INTREQ when CIA INT transitions from not-asserted to asserted.

    // CIA-A → PORTS (level 2)
    bool cia_a_int = (ami->cia_a.icr_data & ami->cia_a.icr_mask) != 0;
    if (cia_a_int && !ami->prev_cia_a_int) {
        ami->intreq |= AMIGA_INTF_PORTS;  // Edge: newly asserted
    }
    ami->prev_cia_a_int = cia_a_int;

    // CIA-B → EXTER (level 6)
    bool cia_b_int = (ami->cia_b.icr_data & ami->cia_b.icr_mask) != 0;
    if (cia_b_int && !ami->prev_cia_b_int) {
        ami->intreq |= AMIGA_INTF_EXTER;  // Edge: newly asserted
    }
    ami->prev_cia_b_int = cia_b_int;

    uint8_t level = paula_irq_level();
    ami->active_irq_level = level;
    m68k_set_irq(level);
}

// IRQ acknowledge callback from M68K CPU
static uint8_t amiga_irq_ack(uint8_t level) {
    if (!ami) return 0;
    // Amiga uses autovector interrupts (vectors 25-30 for levels 1-6)
    // Return 0 to indicate autovector
    return 0;
}

// Write to INTENA or INTREQ with set/clear logic
static void paula_write_int_reg(uint16_t *reg, uint16_t value) {
    if (value & AMIGA_INTF_SETCLR) {
        *reg |= (value & 0x7FFF);  // Set bits
    } else {
        *reg &= ~(value & 0x7FFF); // Clear bits
    }
    update_irq_level();
}

// ============================================================================
// Custom Chip Register Read
// ============================================================================

static uint16_t custom_read(uint16_t reg) {
    reg &= 0x1FE; // Word-aligned, 9-bit offset

    switch (reg) {
        case CUSTOM_DMACONR:
            return ami->dmacon | (ami->blitter.busy ? AMIGA_DMAF_BBUSY : 0)
                               | (ami->blitter.zero ? AMIGA_DMAF_BZERO : 0);

        case CUSTOM_VPOSR:
            // Bit 15 = LOF (long frame)
            // Bits 14-8 = Agnus ID: $00=old DIP (A1000), $10=Fat Agnus 8370 (A500 OCS)
            //   $20=ECS 8372A, $30=AGA. We emulate A500 OCS Fat Agnus.
            // Bit 0 = V8 (high bit of vertical beam position)
            return (ami->lof ? 0x8000 : 0) | 0x1000 | ((ami->vpos >> 8) & 0x01);

        case CUSTOM_VHPOSR:
            return (uint16_t)((ami->vpos & 0xFF) << 8) | (ami->hpos & 0xFF);

        case CUSTOM_JOY0DAT:
            return 0;  // Joystick 0 (stub)
        case CUSTOM_JOY1DAT:
            return 0;  // Joystick 1 (stub)

        case CUSTOM_CLXDAT:
            return 0;  // No collisions

        case CUSTOM_ADKCONR:
            return ami->adkcon;

        case CUSTOM_POT0DAT:
        case CUSTOM_POT1DAT:
            return 0;

        case CUSTOM_POTGOR:
            return 0xFF00; // All buttons up

        case CUSTOM_SERDATR:
            // Serial data - return TBE (transmit buffer empty)
            return 0x3000; // TBE + TSRE set, no data

        case CUSTOM_DSKBYTR:
            return 0x8000; // DSKBYT (byte ready) but no data

        case CUSTOM_INTENAR:
            return ami->intena;

        case CUSTOM_INTREQR:
            return ami->intreq;

        case CUSTOM_DENISEID:
            return 0xFFFF; // OCS Denise (not ECS)

        default:
            return 0;
    }
}

// ============================================================================
// Custom Chip Register Write
// ============================================================================

static void custom_write(uint16_t reg, uint16_t value) {
    reg &= 0x1FE;

    switch (reg) {
        // --- DMA Control ---
        case CUSTOM_DMACON:
            if (value & AMIGA_DMAF_SETCLR) {
                ami->dmacon |= (value & 0x7FFF);
            } else {
                ami->dmacon &= ~(value & 0x7FFF);
            }
            ami->copper.enabled = (ami->dmacon & (AMIGA_DMAF_DMAEN | AMIGA_DMAF_COPEN))
                                == (AMIGA_DMAF_DMAEN | AMIGA_DMAF_COPEN);
            break;

        // --- Interrupt Control ---
        case CUSTOM_INTENA:
            paula_write_int_reg(&ami->intena, value);
            break;
        case CUSTOM_INTREQ:
            paula_write_int_reg(&ami->intreq, value);
            break;

        // --- Audio/Disk Control ---
        case CUSTOM_ADKCON:
            if (value & 0x8000) {
                ami->adkcon |= (value & 0x7FFF);
            } else {
                ami->adkcon &= ~(value & 0x7FFF);
            }
            break;

        // --- Disk DMA ---
        case CUSTOM_DSKPTH:
            ami->dskpt = (ami->dskpt & 0x0000FFFF) | ((uint32_t)value << 16);
            break;
        case CUSTOM_DSKPTL:
            ami->dskpt = (ami->dskpt & 0xFFFF0000) | value;
            break;
        case CUSTOM_DSKLEN: {
            // Double-write starts DMA: both writes must have bit 15 set
            if ((value & 0x8000) && (ami->prev_dsklen & 0x8000)) {
                ami->dsklen = value;
                disk_dma_transfer();
            }
            ami->prev_dsklen = value;
            break;
        }
        case CUSTOM_DSKDAT:
            break; // Disk write data (not needed for reads)
        case CUSTOM_DSKSYNC:
            ami->dsksync = value;
            break;

        // --- Copper ---
        case CUSTOM_COPCON:
            ami->copper.copcon = value & 0xFF;
            break;
        case CUSTOM_COP1LCH:
            ami->copper.cop1lc = (ami->copper.cop1lc & 0x0000FFFF) | ((uint32_t)value << 16);
            break;
        case CUSTOM_COP1LCL:
            ami->copper.cop1lc = (ami->copper.cop1lc & 0xFFFF0000) | value;
            break;
        case CUSTOM_COP2LCH:
            ami->copper.cop2lc = (ami->copper.cop2lc & 0x0000FFFF) | ((uint32_t)value << 16);
            break;
        case CUSTOM_COP2LCL:
            ami->copper.cop2lc = (ami->copper.cop2lc & 0xFFFF0000) | value;
            break;
        case CUSTOM_COPJMP1:
            ami->copper.pc = ami->copper.cop1lc;
            break;
        case CUSTOM_COPJMP2:
            ami->copper.pc = ami->copper.cop2lc;
            break;

        // --- Blitter ---
        case CUSTOM_BLTCON0:
            ami->blitter.bltcon0 = value;
            break;
        case CUSTOM_BLTCON1:
            ami->blitter.bltcon1 = value;
            break;
        case CUSTOM_BLTAFWM:
            ami->blitter.bltafwm = value;
            break;
        case CUSTOM_BLTALWM:
            ami->blitter.bltalwm = value;
            break;
        case CUSTOM_BLTCPTH:
            ami->blitter.bltcpt = (ami->blitter.bltcpt & 0x0000FFFF) | ((uint32_t)value << 16);
            break;
        case CUSTOM_BLTCPTL:
            ami->blitter.bltcpt = (ami->blitter.bltcpt & 0xFFFF0000) | value;
            break;
        case CUSTOM_BLTBPTH:
            ami->blitter.bltbpt = (ami->blitter.bltbpt & 0x0000FFFF) | ((uint32_t)value << 16);
            break;
        case CUSTOM_BLTBPTL:
            ami->blitter.bltbpt = (ami->blitter.bltbpt & 0xFFFF0000) | value;
            break;
        case CUSTOM_BLTAPTH:
            ami->blitter.bltapt = (ami->blitter.bltapt & 0x0000FFFF) | ((uint32_t)value << 16);
            break;
        case CUSTOM_BLTAPTL:
            ami->blitter.bltapt = (ami->blitter.bltapt & 0xFFFF0000) | value;
            break;
        case CUSTOM_BLTDPTH:
            ami->blitter.bltdpt = (ami->blitter.bltdpt & 0x0000FFFF) | ((uint32_t)value << 16);
            break;
        case CUSTOM_BLTDPTL:
            ami->blitter.bltdpt = (ami->blitter.bltdpt & 0xFFFF0000) | value;
            break;
        case CUSTOM_BLTSIZE:
            // Writing BLTSIZE triggers the blit
            ami->blitter.busy = true;
            blitter_execute(value);
            break;
        case CUSTOM_BLTCMOD:
            ami->blitter.bltcmod = (int16_t)value;
            break;
        case CUSTOM_BLTBMOD:
            ami->blitter.bltbmod = (int16_t)value;
            break;
        case CUSTOM_BLTAMOD:
            ami->blitter.bltamod = (int16_t)value;
            break;
        case CUSTOM_BLTDMOD:
            ami->blitter.bltdmod = (int16_t)value;
            break;
        case CUSTOM_BLTCDAT:
            ami->blitter.bltcdat = value;
            break;
        case CUSTOM_BLTBDAT:
            ami->blitter.bltbdat = value;
            break;
        case CUSTOM_BLTADAT:
            ami->blitter.bltadat = value;
            break;

        // --- Display ---
        case CUSTOM_DIWSTRT:
            ami->diwstrt = value;
            break;
        case CUSTOM_DIWSTOP:
            ami->diwstop = value;
            break;
        case CUSTOM_DDFSTRT:
            ami->ddfstrt = value;
            break;
        case CUSTOM_DDFSTOP:
            ami->ddfstop = value;
            break;

        case CUSTOM_BPLCON0:
            ami->bplcon0 = value;
            break;
        case CUSTOM_BPLCON1:
            ami->bplcon1 = value;
            break;
        case CUSTOM_BPLCON2:
            ami->bplcon2 = value;
            break;
        case CUSTOM_BPL1MOD:
            ami->bpl1mod = (int16_t)value;
            break;
        case CUSTOM_BPL2MOD:
            ami->bpl2mod = (int16_t)value;
            break;

        // --- Bitplane pointers ---
        case CUSTOM_BPL1PTH: ami->bplpt[0] = (ami->bplpt[0] & 0x0000FFFF) | ((uint32_t)value << 16); break;
        case CUSTOM_BPL1PTL: ami->bplpt[0] = (ami->bplpt[0] & 0xFFFF0000) | value; break;
        case CUSTOM_BPL2PTH: ami->bplpt[1] = (ami->bplpt[1] & 0x0000FFFF) | ((uint32_t)value << 16); break;
        case CUSTOM_BPL2PTL: ami->bplpt[1] = (ami->bplpt[1] & 0xFFFF0000) | value; break;
        case CUSTOM_BPL3PTH: ami->bplpt[2] = (ami->bplpt[2] & 0x0000FFFF) | ((uint32_t)value << 16); break;
        case CUSTOM_BPL3PTL: ami->bplpt[2] = (ami->bplpt[2] & 0xFFFF0000) | value; break;
        case CUSTOM_BPL4PTH: ami->bplpt[3] = (ami->bplpt[3] & 0x0000FFFF) | ((uint32_t)value << 16); break;
        case CUSTOM_BPL4PTL: ami->bplpt[3] = (ami->bplpt[3] & 0xFFFF0000) | value; break;
        case CUSTOM_BPL5PTH: ami->bplpt[4] = (ami->bplpt[4] & 0x0000FFFF) | ((uint32_t)value << 16); break;
        case CUSTOM_BPL5PTL: ami->bplpt[4] = (ami->bplpt[4] & 0xFFFF0000) | value; break;
        case CUSTOM_BPL6PTH: ami->bplpt[5] = (ami->bplpt[5] & 0x0000FFFF) | ((uint32_t)value << 16); break;
        case CUSTOM_BPL6PTL: ami->bplpt[5] = (ami->bplpt[5] & 0xFFFF0000) | value; break;

        // --- Sprite pointers ---
        case CUSTOM_SPR0PTH: case CUSTOM_SPR0PTH+4: case CUSTOM_SPR0PTH+8: case CUSTOM_SPR0PTH+12:
        case CUSTOM_SPR0PTH+16: case CUSTOM_SPR0PTH+20: case CUSTOM_SPR0PTH+24: case CUSTOM_SPR0PTH+28: {
            int idx = (reg - CUSTOM_SPR0PTH) / 4;
            if (idx < 8) ami->sprpt[idx] = (ami->sprpt[idx] & 0x0000FFFF) | ((uint32_t)value << 16);
            break;
        }
        case CUSTOM_SPR0PTL: case CUSTOM_SPR0PTL+4: case CUSTOM_SPR0PTL+8: case CUSTOM_SPR0PTL+12:
        case CUSTOM_SPR0PTL+16: case CUSTOM_SPR0PTL+20: case CUSTOM_SPR0PTL+24: case CUSTOM_SPR0PTL+28: {
            int idx = (reg - CUSTOM_SPR0PTL) / 4;
            if (idx < 8) ami->sprpt[idx] = (ami->sprpt[idx] & 0xFFFF0000) | value;
            break;
        }

        default:
            // Color palette: $180-$1BE (32 entries, 2 bytes each)
            if (reg >= CUSTOM_COLOR00 && reg < CUSTOM_COLOR00 + 64) {
                int idx = (reg - CUSTOM_COLOR00) / 2;
                if (idx < 32) {
                    ami->color[idx] = value & 0x0FFF;
                    ami->color_rgb565[idx] = amiga_color_to_rgb565(value & 0x0FFF);
                }
            }
            break;
    }
}

// ============================================================================
// Blitter Implementation
// ============================================================================

// Read word from M68K memory (chip RAM only for blitter)
static inline uint16_t chip_read_word(uint32_t addr) {
    addr &= 0x1FFFFE; // Chip RAM mask (2MB max), word aligned
    if (addr + 1 < ami->config.chip_ram_size) {
        // Read from M68K memory using public API would be ideal,
        // but we need direct access for speed. Use m68k_get_reg workaround.
        // Actually, just read directly via memory read functions.
        // We'll use the M68K memory read functions through the hook system
        bool handled = false;
        uint32_t val = amiga_io_read_hook(addr, 2, &handled);
        if (handled) return (uint16_t)val;
        // Direct memory access for chip RAM
        return (uint16_t)((m68k_read_memory_8(addr) << 8) | m68k_read_memory_8(addr + 1));
    }
    return 0;
}

static inline void chip_write_word(uint32_t addr, uint16_t value) {
    addr &= 0x1FFFFE;
    if (addr + 1 < ami->config.chip_ram_size) {
        m68k_write_memory_8(addr, (value >> 8) & 0xFF);
        m68k_write_memory_8(addr + 1, value & 0xFF);
    }
}

// Blitter implementation
static void blitter_execute(uint16_t bltsize) {
    int height = (bltsize >> 6) & 0x3FF;  // bits 15-6
    int width  = bltsize & 0x3F;          // bits 5-0 (in words)
    if (height == 0) height = 1024;
    if (width == 0) width = 64;

    uint16_t con0 = ami->blitter.bltcon0;
    uint16_t con1 = ami->blitter.bltcon1;
    uint8_t minterm = con0 & 0xFF;
    bool use_a = (con0 & 0x0800) != 0;
    bool use_b = (con0 & 0x0400) != 0;
    bool use_c = (con0 & 0x0200) != 0;
    bool use_d = (con0 & 0x0100) != 0;
    int ash = (con0 >> 12) & 0x0F;  // Shift A
    int bsh = (con1 >> 12) & 0x0F;  // Shift B
    bool desc = (con1 & 0x02) != 0; // Descending mode
    bool fill = (con1 & 0x08) != 0; // Fill mode

    uint32_t apt = ami->blitter.bltapt;
    uint32_t bpt = ami->blitter.bltbpt;
    uint32_t cpt = ami->blitter.bltcpt;
    uint32_t dpt = ami->blitter.bltdpt;

    int16_t amod = ami->blitter.bltamod;
    int16_t bmod = ami->blitter.bltbmod;
    int16_t cmod = ami->blitter.bltcmod;
    int16_t dmod = ami->blitter.bltdmod;

    int step = desc ? -2 : 2;
    ami->blitter.zero = true;

    uint16_t prev_a = 0, prev_b = 0;

    for (int y = 0; y < height; y++) {
        bool fill_carry = false;
        uint16_t first_mask = ami->blitter.bltafwm;
        uint16_t last_mask  = ami->blitter.bltalwm;

        for (int x = 0; x < width; x++) {
            // Read sources
            uint16_t adat = use_a ? chip_read_word(apt & 0x1FFFFE) : ami->blitter.bltadat;
            uint16_t bdat = use_b ? chip_read_word(bpt & 0x1FFFFE) : ami->blitter.bltbdat;
            uint16_t cdat = use_c ? chip_read_word(cpt & 0x1FFFFE) : ami->blitter.bltcdat;

            // Apply masks to A
            uint16_t amask = 0xFFFF;
            if (x == 0) amask &= first_mask;
            if (x == width - 1) amask &= last_mask;
            adat &= amask;

            // Shift A (barrel shift using previous word)
            if (ash) {
                uint32_t combined = ((uint32_t)prev_a << 16) | adat;
                adat = (combined >> ash) & 0xFFFF;
            }
            prev_a = use_a ? chip_read_word(apt & 0x1FFFFE) : ami->blitter.bltadat;
            prev_a &= amask;

            // Shift B
            if (bsh) {
                uint32_t combined = ((uint32_t)prev_b << 16) | bdat;
                bdat = (combined >> bsh) & 0xFFFF;
            }
            prev_b = bdat;

            // Apply minterm
            uint16_t result = 0;
            for (int bit = 0; bit < 16; bit++) {
                int a = (adat >> bit) & 1;
                int b = (bdat >> bit) & 1;
                int c = (cdat >> bit) & 1;
                int index = (a << 2) | (b << 1) | c;
                if (minterm & (1 << index)) {
                    result |= (1 << bit);
                }
            }

            // Fill mode
            if (fill) {
                uint16_t filled = 0;
                // Inclusive fill (IFE) or Exclusive fill (EFE)
                bool exclusive = (con1 & 0x10) != 0;
                for (int bit = 0; bit < 16; bit++) {
                    if (result & (1 << bit)) {
                        if (exclusive) {
                            fill_carry = !fill_carry;
                        } else {
                            fill_carry = !fill_carry;
                        }
                    }
                    if (fill_carry) {
                        filled |= (1 << bit);
                    }
                }
                result = filled;
            }

            if (result != 0) ami->blitter.zero = false;

            // Write destination
            if (use_d) {
                chip_write_word(dpt & 0x1FFFFE, result);
            }

            // Advance pointers
            if (use_a) apt += step;
            if (use_b) bpt += step;
            if (use_c) cpt += step;
            if (use_d) dpt += step;
        }

        // Apply modulos at end of line
        if (use_a) apt += amod;
        if (use_b) bpt += bmod;
        if (use_c) cpt += cmod;
        if (use_d) dpt += dmod;
    }

    // Update pointers in state
    ami->blitter.bltapt = apt;
    ami->blitter.bltbpt = bpt;
    ami->blitter.bltcpt = cpt;
    ami->blitter.bltdpt = dpt;

    ami->blitter.busy = false;

    // Fire BLIT interrupt
    ami->intreq |= AMIGA_INTF_BLIT;
    update_irq_level();
}

// ============================================================================
// Copper Implementation
// ============================================================================

// Per-line copper execution: runs copper instructions until a WAIT blocks
// or the end of the list is reached. Called once per scanline.
static void copper_execute_line(void) {
    if (!ami->copper.enabled) return;
    if (!(ami->dmacon & (AMIGA_DMAF_DMAEN | AMIGA_DMAF_COPEN))) return;

    uint32_t pc = ami->copper.pc;
    if (pc == 0) return;  // No copper list loaded

    int max_instructions = 100; // Safety limit per line

    while (max_instructions-- > 0) {
        // Read instruction (2 words)
        uint16_t ir1 = chip_read_word(pc & 0x1FFFFE);
        uint16_t ir2 = chip_read_word((pc + 2) & 0x1FFFFE);
        pc += 4;

        if (!(ir1 & 0x0001)) {
            // MOVE instruction: ir1[15:1] = register offset, ir2 = data
            uint16_t reg = ir1 & 0x01FE;
            uint16_t min_reg = (ami->copper.copcon & 0x02) ? 0x040 : 0x080;
            if (reg >= min_reg) {
                custom_write(reg, ir2);
            }
        } else {
            // WAIT or SKIP instruction
            bool is_skip = (ir2 & 0x0001) != 0;
            uint8_t vp = (ir1 >> 8) & 0xFF;
            uint8_t hp = ir1 & 0xFE;
            uint8_t vm = (ir2 >> 8) & 0x7F;
            uint8_t hm = ir2 & 0xFE;

            // End of copper list marker
            if (ir1 == 0xFFFF && ir2 == 0xFFFE) {
                ami->copper.enabled = false;
                break;
            }

            if (!is_skip) {
                // WAIT: check if beam has reached the wait position
                uint8_t cur_vp = ami->vpos & 0xFF;
                uint8_t cur_hp = ami->hpos & 0xFE;
                bool reached = ((cur_vp & vm) > (vp & vm)) ||
                              (((cur_vp & vm) == (vp & vm)) && ((cur_hp & hm) >= (hp & hm)));
                if (!reached) {
                    pc -= 4; // Undo advance, retry on next line
                    break;
                }
                // Beam has passed: continue executing
            } else {
                // SKIP: skip next instruction if beam past position
                uint8_t cur_vp = ami->vpos & 0xFF;
                uint8_t cur_hp = ami->hpos & 0xFE;
                bool past = ((cur_vp & vm) > (vp & vm)) ||
                           (((cur_vp & vm) == (vp & vm)) && ((cur_hp & hm) >= (hp & hm)));
                if (past) {
                    pc += 4;
                }
            }
        }
    }

    ami->copper.pc = pc;
}

// ============================================================================
// Floppy Disk DMA Transfer
// ============================================================================

static void disk_dma_transfer(void) {
    if (ami->current_drive < 0 || ami->current_drive >= AMIGA_MAX_DRIVES) return;

    floppy_drive_t *drv = &ami->floppy[ami->current_drive];
    if (!drv->inserted || !drv->image) {
        ESP_LOGW(TAG, "Disk DMA: no disk in drive %d", ami->current_drive);
        return;
    }

    uint16_t len = ami->dsklen & 0x3FFF;  // Word count
    bool write = (ami->dsklen & 0x4000) != 0;
    uint32_t dma_addr = ami->dskpt & 0x1FFFFE;

    if (write) {
        ESP_LOGW(TAG, "Disk write not implemented");
        ami->intreq |= AMIGA_INTF_DSKBLK;
        update_irq_level();
        return;
    }

    // Calculate file offset: track, side
    int track = drv->track;
    int side = ami->disk_side;

    // ADF layout: track 0 side 0, track 0 side 1, track 1 side 0, ...
    uint32_t track_offset = (uint32_t)(track * 2 + side) * AMIGA_ADF_TRACK_SIZE;

    // Read sync word + data
    // The DMA engine in the real Amiga reads raw MFM data and finds the sync word.
    // For ADF (decoded data), we skip the sync/header and just copy sector data.
    // We read up to 'len' words of data from the track.

    uint32_t bytes_to_read = (uint32_t)len * 2;
    if (bytes_to_read > AMIGA_ADF_TRACK_SIZE) {
        bytes_to_read = AMIGA_ADF_TRACK_SIZE;
    }

    // Seek to track position in ADF
    if (track_offset + bytes_to_read > drv->size) {
        ESP_LOGW(TAG, "Disk read past end of ADF (track=%d side=%d)", track, side);
        ami->intreq |= AMIGA_INTF_DSKBLK;
        update_irq_level();
        return;
    }

    fseek(drv->image, track_offset, SEEK_SET);

    // Read data into chip RAM via DMA
    uint8_t buf[512];
    uint32_t remaining = bytes_to_read;
    uint32_t dest = dma_addr;

    while (remaining > 0) {
        uint32_t chunk = remaining > sizeof(buf) ? sizeof(buf) : remaining;
        size_t got = fread(buf, 1, chunk, drv->image);
        if (got == 0) break;

        for (uint32_t i = 0; i < got; i++) {
            m68k_write_memory_8(dest + i, buf[i]);
        }
        dest += got;
        remaining -= got;
    }

    ESP_LOGI(TAG, "Disk DMA: track=%d side=%d, %lu bytes → $%06lX",
             track, side, (unsigned long)bytes_to_read, (unsigned long)dma_addr);

    // Fire DSKBLK interrupt
    ami->intreq |= AMIGA_INTF_DSKBLK;
    update_irq_level();

    // Also fire DSKSYNC if sync word was requested
    if (ami->adkcon & 0x0400) { // WORDSYNC enabled
        ami->intreq |= AMIGA_INTF_DSKSYN;
        update_irq_level();
    }
}

// ============================================================================
// IDE Hard Disk
// ============================================================================

#define IDE_STATUS_BSY   0x80
#define IDE_STATUS_DRDY  0x40
#define IDE_STATUS_DRQ   0x08
#define IDE_STATUS_ERR   0x01

#define IDE_CMD_IDENTIFY    0xEC
#define IDE_CMD_READ_SEC    0x20
#define IDE_CMD_WRITE_SEC   0x30
#define IDE_CMD_SET_FEAT    0xEF

static void ide_execute_command(ide_drive_t *drv) {
    switch (drv->command) {
        case IDE_CMD_IDENTIFY: {
            // Build identify data
            memset(drv->data_buf, 0, 512);
            uint16_t *id = (uint16_t *)drv->data_buf;
            id[0] = 0x0040;  // Fixed disk
            id[1] = drv->sectors / (16 * 63);  // Cylinders
            id[3] = 16;       // Heads
            id[6] = 63;       // Sectors per track
            id[49] = 0x0200;  // LBA supported
            id[60] = drv->sectors & 0xFFFF;        // Total sectors low
            id[61] = (drv->sectors >> 16) & 0xFFFF; // Total sectors high

            // Serial number (20 chars)
            const char *serial = "ESP32AMIGAHD001     ";
            for (int i = 0; i < 10; i++) {
                id[10 + i] = (serial[i*2] << 8) | serial[i*2 + 1];
            }
            // Model (40 chars)
            const char *model = "ESP32-P4 Amiga IDE Hard Disk            ";
            for (int i = 0; i < 20; i++) {
                id[27 + i] = (model[i*2] << 8) | model[i*2 + 1];
            }

            drv->data_pos = 0;
            drv->data_len = 512;
            drv->data_ready = true;
            drv->status = IDE_STATUS_DRDY | IDE_STATUS_DRQ;
            break;
        }

        case IDE_CMD_READ_SEC: {
            if (!drv->image) {
                drv->status = IDE_STATUS_DRDY | IDE_STATUS_ERR;
                drv->error = 0x04; // Abort
                break;
            }
            // Calculate LBA
            uint32_t lba;
            if (drv->drive_head & 0x40) {
                // LBA mode
                lba = ((drv->drive_head & 0x0F) << 24) |
                      (drv->cylinder_hi << 16) |
                      (drv->cylinder_lo << 8) |
                      drv->sector_num;
            } else {
                // CHS mode (simplified)
                lba = drv->sector_num; // Very simplified
            }

            if (lba >= drv->sectors) {
                drv->status = IDE_STATUS_DRDY | IDE_STATUS_ERR;
                drv->error = 0x10; // ID not found
                break;
            }

            fseek(drv->image, (long)lba * 512, SEEK_SET);
            size_t got = fread(drv->data_buf, 1, 512, drv->image);
            if (got < 512) memset(drv->data_buf + got, 0, 512 - got);

            drv->data_pos = 0;
            drv->data_len = 512;
            drv->data_ready = true;
            drv->status = IDE_STATUS_DRDY | IDE_STATUS_DRQ;
            break;
        }

        case IDE_CMD_WRITE_SEC: {
            // Set up for data receive
            drv->data_pos = 0;
            drv->data_len = 512;
            drv->data_ready = false; // Waiting for data
            drv->status = IDE_STATUS_DRDY | IDE_STATUS_DRQ;
            break;
        }

        case IDE_CMD_SET_FEAT:
            drv->status = IDE_STATUS_DRDY;
            break;

        default:
            ESP_LOGW(TAG, "IDE: unknown command 0x%02X", drv->command);
            drv->status = IDE_STATUS_DRDY | IDE_STATUS_ERR;
            drv->error = 0x04; // Abort
            break;
    }
}

static void ide_write_data(ide_drive_t *drv, uint16_t value) {
    if (drv->data_pos < 512) {
        drv->data_buf[drv->data_pos++] = (value >> 8) & 0xFF;
        drv->data_buf[drv->data_pos++] = value & 0xFF;
    }
    if (drv->data_pos >= 512 && drv->command == IDE_CMD_WRITE_SEC) {
        // Write sector to disk
        if (drv->image) {
            uint32_t lba;
            if (drv->drive_head & 0x40) {
                lba = ((drv->drive_head & 0x0F) << 24) |
                      (drv->cylinder_hi << 16) |
                      (drv->cylinder_lo << 8) |
                      drv->sector_num;
            } else {
                lba = drv->sector_num;
            }
            if (lba < drv->sectors) {
                fseek(drv->image, (long)lba * 512, SEEK_SET);
                fwrite(drv->data_buf, 1, 512, drv->image);
                fflush(drv->image);
            }
        }
        drv->status = IDE_STATUS_DRDY;
        drv->data_ready = false;

        // Handle multi-sector write
        if (drv->sector_count > 1) {
            drv->sector_count--;
            drv->sector_num++;
            drv->data_pos = 0;
            drv->status = IDE_STATUS_DRDY | IDE_STATUS_DRQ;
        }
    }
}

static uint16_t ide_read_data(ide_drive_t *drv) {
    if (!drv->data_ready || drv->data_pos >= drv->data_len) return 0;
    uint16_t val = (drv->data_buf[drv->data_pos] << 8) | drv->data_buf[drv->data_pos + 1];
    drv->data_pos += 2;
    if (drv->data_pos >= drv->data_len) {
        drv->data_ready = false;
        drv->status = IDE_STATUS_DRDY;

        // Multi-sector read: load next sector
        if (drv->command == IDE_CMD_READ_SEC && drv->sector_count > 1) {
            drv->sector_count--;
            drv->sector_num++;
            ide_execute_command(drv);
        }
    }
    return val;
}

// ============================================================================
// Autoconfig Implementation
// ============================================================================

// Autoconfig board descriptor for our IDE controller
// This makes it appear as a Zorro-II board to Kickstart
static uint8_t autoconfig_read(uint32_t offset) {
    if (ami->autoconfig.configured) return 0xFF;

    // Nibble-based reads (offset in bytes, data in high nibble)
    switch (offset & 0xFF) {
        case 0x00: return 0xC0;  // Type: board, no shutup, Zorro II
        case 0x02: return 0x10;  // Product: 1 (in high nibble)
        case 0x04: return 0x00;  // Flags
        case 0x06: return 0x00;
        case 0x08: return 0x07;  // Manufacturer high (0x07FF = test)
        case 0x0A: return 0xF0;
        case 0x0C: return 0xF0;
        case 0x0E: return 0xF0;
        case 0x10: return 0x00;  // Serial
        case 0x12: return 0x00;
        case 0x14: return 0x00;
        case 0x16: return 0x00;
        case 0x18: return 0x00;
        case 0x1A: return 0x00;
        case 0x1C: return 0x00;
        case 0x1E: return 0x00;
        default:   return 0xFF;
    }
}

static void autoconfig_write(uint32_t offset, uint8_t value) {
    offset &= 0xFF;
    if (offset == 0x48) {
        // Base address register - board is being configured
        ami->autoconfig.base_addr = ((uint32_t)value) << 16;
        ami->autoconfig.configured = true;
        ESP_LOGI(TAG, "Autoconfig: IDE board configured at $%06lX",
                 (unsigned long)ami->autoconfig.base_addr);
    } else if (offset == 0x4C) {
        // Shutup - board is being told to go away
        ami->autoconfig.configured = true;
        ami->autoconfig.base_addr = 0;
        ESP_LOGI(TAG, "Autoconfig: board shut up");
    }
}

// ============================================================================
// Keyboard
// ============================================================================

static void kbd_push_scancode(uint8_t code) {
    if (ami->kbd_count >= (int)(sizeof(ami->kbd_buffer) / sizeof(ami->kbd_buffer[0]))) return;
    ami->kbd_buffer[ami->kbd_tail] = code;
    ami->kbd_tail = (ami->kbd_tail + 1) % (int)(sizeof(ami->kbd_buffer) / sizeof(ami->kbd_buffer[0]));
    ami->kbd_count++;
}

// Deliver next key from buffer to CIA-A SDR
static void kbd_deliver(void) {
    if (ami->kbd_count == 0) return;
    if (!ami->kbd_handshake) return; // Wait for host to acknowledge

    uint8_t code = ami->kbd_buffer[ami->kbd_head];
    ami->kbd_head = (ami->kbd_head + 1) % (int)(sizeof(ami->kbd_buffer) / sizeof(ami->kbd_buffer[0]));
    ami->kbd_count--;

    // Amiga keyboard sends keycode in SDR:
    // The code is bitwise inverted and rotated left by 1
    // key_code = ~(raw_code << 1 | raw_code >> 7)
    // Actually: SDR = ~((code << 1) | (code >> 7))
    ami->cia_a.sdr = ~((code << 1) | (code >> 7));
    ami->cia_a.icr_data |= CIA_ICR_SP; // Serial port interrupt
    ami->kbd_handshake = false;

    update_irq_level();
}

// ============================================================================
// Video Rendering
// ============================================================================

static void render_bitplanes(void) {
    if (!video_card_is_initialized()) return;

    uint16_t *fb = video_card_get_back_buffer();
    if (!fb) return;

    uint16_t fb_w = video_card_get_width();   // 480
    uint16_t fb_h = video_card_get_height();  // 320

    // Determine number of active bitplanes
    int num_planes = (ami->bplcon0 >> 12) & 0x07;
    if (num_planes > 6) num_planes = 6;
    bool hires = (ami->bplcon0 & 0x8000) != 0;

    int disp_h = ami->config.pal_mode ? 256 : 200;

    // Background color
    uint16_t bg_color = ami->color_rgb565[0];

    // Center the Amiga display on the LCD
    int x_off, y_off;

    if (hires) {
        // 640→480: can't fit, crop or scale
        x_off = 0;
    } else {
        // 320→480: 1.5x horizontal
        x_off = (fb_w - 320) / 2; // Center with borders: 80px each side
    }
    y_off = (fb_h - disp_h) / 2;
    if (y_off < 0) y_off = 0;

    // Fill borders with background color
    // Top border
    for (int y = 0; y < y_off; y++) {
        uint16_t *row = &fb[y * fb_w];
        for (int x = 0; x < fb_w; x++) row[x] = bg_color;
    }
    // Bottom border
    for (int y = y_off + disp_h; y < fb_h; y++) {
        uint16_t *row = &fb[y * fb_w];
        for (int x = 0; x < fb_w; x++) row[x] = bg_color;
    }

    if (num_planes == 0) {
        // No bitplanes - fill entire display area with color 0
        for (int y = y_off; y < y_off + disp_h && y < fb_h; y++) {
            uint16_t *row = &fb[y * fb_w];
            for (int x = 0; x < fb_w; x++) row[x] = bg_color;
        }
        return;
    }

    // Render bitplane data line by line
    // Bitplane pointers were set by copper or CPU
    uint32_t bpl_addr[6];
    for (int p = 0; p < num_planes; p++) {
        bpl_addr[p] = ami->bplpt[p];
    }

    int words_per_line = hires ? 40 : 20; // 640/16=40, 320/16=20

    for (int line = 0; line < disp_h; line++) {
        int fb_y = y_off + line;
        if (fb_y >= fb_h) break;
        uint16_t *fb_row = &fb[fb_y * fb_w];

        if (!hires) {
            // Low-res: left border
            for (int x = 0; x < x_off; x++) fb_row[x] = bg_color;
            // Right border
            for (int x = x_off + 320; x < fb_w; x++) fb_row[x] = bg_color;
        }

        // Decode bitplanes for this line
        for (int word = 0; word < words_per_line; word++) {
            // Read 16-bit word from each bitplane
            uint16_t plane_data[6] = {0};
            for (int p = 0; p < num_planes; p++) {
                uint32_t addr = bpl_addr[p] + word * 2;
                addr &= 0x1FFFFE;
                if (addr + 1 < ami->config.chip_ram_size) {
                    plane_data[p] = (m68k_read_memory_8(addr) << 8) | m68k_read_memory_8(addr + 1);
                }
            }

            // Extract 16 pixels from interleaved bitplanes
            for (int bit = 15; bit >= 0; bit--) {
                uint8_t color_idx = 0;
                for (int p = 0; p < num_planes; p++) {
                    if (plane_data[p] & (1 << bit)) {
                        color_idx |= (1 << p);
                    }
                }

                int px;
                if (hires) {
                    px = word * 16 + (15 - bit);
                    // Scale 640→480: skip every 4th pixel
                    int scaled_x = (px * 3) / 4;
                    if (scaled_x < fb_w) {
                        fb_row[scaled_x] = ami->color_rgb565[color_idx & 0x1F];
                    }
                } else {
                    px = x_off + word * 16 + (15 - bit);
                    if (px < fb_w) {
                        fb_row[px] = ami->color_rgb565[color_idx & 0x1F];
                    }
                }
            }
        }

        // Advance bitplane pointers
        for (int p = 0; p < num_planes; p++) {
            bpl_addr[p] += words_per_line * 2;
            // Apply modulos (odd planes use bpl1mod, even planes use bpl2mod)
            if (p & 1) {
                bpl_addr[p] += ami->bpl2mod;
            } else {
                bpl_addr[p] += ami->bpl1mod;
            }
        }
    }
}

// ============================================================================
// I/O Hook - Read
// ============================================================================

static uint32_t amiga_io_read_hook(uint32_t addr, int size, bool *handled) {
    if (!ami) return 0;

    addr &= 0x00FFFFFF; // 24-bit address bus

    // --- OVL: ROM overlay at $000000 when set ---
    if (ami->ovl && addr < ami->kick_size) {
        *handled = true;
        if (size == 1) {
            return ami->kick_rom[addr];
        } else if (size == 2) {
            return (ami->kick_rom[addr] << 8) | ami->kick_rom[addr + 1];
        } else {
            return (ami->kick_rom[addr] << 24) | (ami->kick_rom[addr + 1] << 16) |
                   (ami->kick_rom[addr + 2] << 8) | ami->kick_rom[addr + 3];
        }
    }

    // --- CIA-A ($BFE001, odd addresses, $100 spacing) ---
    if (addr >= 0xBFE001 && addr <= 0xBFEF01 && (addr & 1)) {
        *handled = true;
        uint16_t reg = (addr - 0xBFE001) & 0x0F00;
        return cia_read(&ami->cia_a, reg, false);
    }

    // --- CIA-B ($BFD000, even addresses, $100 spacing) ---
    if (addr >= 0xBFD000 && addr <= 0xBFDF00 && !(addr & 1)) {
        *handled = true;
        uint16_t reg = (addr - 0xBFD000) & 0x0F00;
        return cia_read(&ami->cia_b, reg, true);
    }

    // --- Remaining CIA address space ($BFD000-$BFFFFF) catch-all ---
    // Handles unmatched CIA addresses (odd bytes in CIA-B, even in CIA-A, $BFF000+)
    if (addr >= 0xBFD000 && addr < 0xC00000) {
        *handled = true;
        return (size == 1) ? 0xFF : (size == 2) ? 0xFFFF : 0xFFFFFFFF;
    }

    // --- Custom chip registers ($DFF000-$DFF1FF) ---
    if (addr >= AMIGA_CUSTOM_BASE && addr < AMIGA_CUSTOM_BASE + AMIGA_CUSTOM_SIZE) {
        *handled = true;
        uint16_t reg = addr - AMIGA_CUSTOM_BASE;
        uint16_t val = custom_read(reg & 0x1FE);
        if (size == 1) {
            return (addr & 1) ? (val & 0xFF) : (val >> 8);
        }
        return val;
    }

    // --- Custom register mirrors ($DFF200-$DFFFFF) ---
    if (addr >= AMIGA_CUSTOM_BASE + AMIGA_CUSTOM_SIZE && addr < 0xE00000) {
        *handled = true;
        return 0;
    }

    // --- Autoconfig space ($E80000-$E8FFFF) ---
    if (addr >= AMIGA_AUTOCONFIG_BASE && addr < AMIGA_AUTOCONFIG_BASE + AMIGA_AUTOCONFIG_SIZE) {
        *handled = true;
        // Only respond if we have a hard disk to configure
        if (ami->ide[0].mounted && !ami->autoconfig.configured) {
            return autoconfig_read(addr - AMIGA_AUTOCONFIG_BASE);
        }
        return 0xFF;
    }

    // --- IDE registers (at autoconfig-assigned address) ---
    if (ami->autoconfig.configured && ami->autoconfig.base_addr != 0) {
        uint32_t ide_base = ami->autoconfig.base_addr;
        if (addr >= ide_base && addr < ide_base + 0x10000) {
            *handled = true;
            uint32_t off = addr - ide_base;
            ide_drive_t *drv = &ami->ide[0]; // Primary drive

            switch (off & 0x1F) {
                case 0x00: return ide_read_data(drv);
                case 0x02: return drv->error;
                case 0x04: return drv->sector_count;
                case 0x06: return drv->sector_num;
                case 0x08: return drv->cylinder_lo;
                case 0x0A: return drv->cylinder_hi;
                case 0x0C: return drv->drive_head;
                case 0x0E: return drv->status;
                case 0x1C: return drv->status; // Alternate status
                default:   return 0xFF;
            }
        }
    }

    // --- Kickstart ROM ($F80000-$FFFFFF or $FC0000-$FFFFFF) ---
    if (addr >= ami->kick_base && addr < ami->kick_base + ami->kick_size) {
        *handled = true;
        uint32_t off = addr - ami->kick_base;
        if (size == 1) {
            return ami->kick_rom[off];
        } else if (size == 2) {
            return (ami->kick_rom[off] << 8) | ami->kick_rom[off + 1];
        } else {
            return (ami->kick_rom[off] << 24) | (ami->kick_rom[off + 1] << 16) |
                   (ami->kick_rom[off + 2] << 8) | ami->kick_rom[off + 3];
        }
    }

    // Mirror 256KB ROM at $F80000-$FBFFFF for KS 1.x
    if (ami->kick_size == AMIGA_KICK_256K_SIZE && addr >= AMIGA_KICK_512K_BASE && addr < AMIGA_KICK_256K_BASE) {
        *handled = true;
        uint32_t off = addr - AMIGA_KICK_512K_BASE; // Offset into $F80000 range
        if (size == 1) {
            return ami->kick_rom[off];
        } else if (size == 2) {
            return (ami->kick_rom[off] << 8) | ami->kick_rom[off + 1];
        } else {
            return (ami->kick_rom[off] << 24) | (ami->kick_rom[off + 1] << 16) |
                   (ami->kick_rom[off + 2] << 8) | ami->kick_rom[off + 3];
        }
    }

    // --- Extended ROM / expansion area ---
    if ((addr >= 0xE00000 && addr < 0xE80000) ||
        (addr >= 0xE90000 && addr < 0xF00000) ||
        (addr >= 0xF00000 && addr < ami->kick_base)) {
        *handled = true;
        return (size == 1) ? 0xFF : (size == 2) ? 0xFFFF : 0xFFFFFFFF;
    }

    // --- Dead zones (above chip RAM, below CIA range) ---
    if (addr >= ami->config.chip_ram_size && addr < 0xA00000) {
        // Check for slow RAM
        if (ami->config.slow_ram_enabled &&
            addr >= AMIGA_SLOW_RAM_BASE &&
            addr < AMIGA_SLOW_RAM_BASE + AMIGA_SLOW_RAM_SIZE) {
            // Slow RAM - let it fall through to M68K memory (if allocated)
            return 0;
        }
        // Dead zone
        *handled = true;
        return (size == 1) ? 0xFF : (size == 2) ? 0xFFFF : 0xFFFFFFFF;
    }

    // --- Reserved areas above $A00000 ---
    if (addr >= 0xA00000 && addr < 0xBFD000) {
        *handled = true;
        return (size == 1) ? 0xFF : (size == 2) ? 0xFFFF : 0xFFFFFFFF;
    }

    // Between CIAs and custom regs
    if (addr >= 0xC00000 && addr < 0xDFF000) {
        // Slow RAM check
        if (ami->config.slow_ram_enabled &&
            addr >= AMIGA_SLOW_RAM_BASE &&
            addr < AMIGA_SLOW_RAM_BASE + AMIGA_SLOW_RAM_SIZE) {
            return 0; // Let through
        }
        *handled = true;
        return (size == 1) ? 0xFF : (size == 2) ? 0xFFFF : 0xFFFFFFFF;
    }

    // --- Final catch-all: any address above chip RAM returns open bus ---
    // This prevents non-chip-RAM addresses from reading cpu->memory[] (16MB backing RAM)
    if (addr >= ami->config.chip_ram_size) {
        // Allow slow RAM through if enabled
        if (ami->config.slow_ram_enabled &&
            addr >= AMIGA_SLOW_RAM_BASE &&
            addr < AMIGA_SLOW_RAM_BASE + AMIGA_SLOW_RAM_SIZE) {
            return 0; // Not handled - let M68K core use direct memory for slow RAM
        }
        *handled = true;
        return (size == 1) ? 0xFF : (size == 2) ? 0xFFFF : 0xFFFFFFFF;
    }

    return 0; // Not handled - chip RAM, let M68K core use direct memory
}

// ============================================================================
// I/O Hook - Write
// ============================================================================

static void amiga_io_write_hook(uint32_t addr, uint32_t value, int size, bool *handled) {
    if (!ami) return;

    addr &= 0x00FFFFFF;

    // --- OVL area: writes go to Chip RAM even when OVL=1 ---
    // (Don't intercept writes to $000000+ — let them go to CPU memory)

    // --- CIA-A ($BFE001, odd addresses) ---
    if (addr >= 0xBFE001 && addr <= 0xBFEF01 && (addr & 1)) {
        *handled = true;
        uint16_t reg = (addr - 0xBFE001) & 0x0F00;
        cia_write(&ami->cia_a, reg, (uint8_t)value, false);
        return;
    }

    // --- CIA-B ($BFD000, even addresses) ---
    if (addr >= 0xBFD000 && addr <= 0xBFDF00 && !(addr & 1)) {
        *handled = true;
        uint16_t reg = (addr - 0xBFD000) & 0x0F00;
        cia_write(&ami->cia_b, reg, (uint8_t)value, true);
        return;
    }

    // --- Remaining CIA address space ($BFD000-$BFFFFF) catch-all ---
    if (addr >= 0xBFD000 && addr < 0xC00000) {
        *handled = true;
        return;
    }

    // --- Custom chip registers ($DFF000-$DFF1FF) ---
    if (addr >= AMIGA_CUSTOM_BASE && addr < AMIGA_CUSTOM_BASE + AMIGA_CUSTOM_SIZE) {
        *handled = true;
        uint16_t reg = addr - AMIGA_CUSTOM_BASE;
        if (size == 2) {
            custom_write(reg & 0x1FE, (uint16_t)value);
        } else if (size == 1) {
            // Byte write to custom regs: rare but handle it
            // Most custom regs are word-only; byte writes are unusual
        } else if (size == 4) {
            // Long write = two word writes
            custom_write(reg & 0x1FE, (uint16_t)(value >> 16));
            custom_write((reg + 2) & 0x1FE, (uint16_t)(value & 0xFFFF));
        }
        return;
    }

    // --- Custom register mirrors ($DFF200-$DFFFFF) ---
    if (addr >= AMIGA_CUSTOM_BASE + AMIGA_CUSTOM_SIZE && addr < 0xE00000) {
        *handled = true;
        return;
    }

    // --- Autoconfig ($E80000-$E8FFFF) ---
    if (addr >= AMIGA_AUTOCONFIG_BASE && addr < AMIGA_AUTOCONFIG_BASE + AMIGA_AUTOCONFIG_SIZE) {
        *handled = true;
        if (ami->ide[0].mounted && !ami->autoconfig.configured) {
            autoconfig_write(addr - AMIGA_AUTOCONFIG_BASE, (uint8_t)value);
        }
        return;
    }

    // --- IDE registers ---
    if (ami->autoconfig.configured && ami->autoconfig.base_addr != 0) {
        uint32_t ide_base = ami->autoconfig.base_addr;
        if (addr >= ide_base && addr < ide_base + 0x10000) {
            *handled = true;
            uint32_t off = addr - ide_base;
            ide_drive_t *drv = &ami->ide[0];

            switch (off & 0x1F) {
                case 0x00: ide_write_data(drv, (uint16_t)value); break;
                case 0x02: /* features */ break;
                case 0x04: drv->sector_count = value & 0xFF; break;
                case 0x06: drv->sector_num = value & 0xFF; break;
                case 0x08: drv->cylinder_lo = value & 0xFF; break;
                case 0x0A: drv->cylinder_hi = value & 0xFF; break;
                case 0x0C: drv->drive_head = value & 0xFF; break;
                case 0x0E:
                    drv->command = value & 0xFF;
                    ide_execute_command(drv);
                    break;
            }
            return;
        }
    }

    // --- Kickstart ROM (read-only, absorb writes) ---
    if (addr >= ami->kick_base && addr < ami->kick_base + ami->kick_size) {
        *handled = true;
        return;
    }
    if (ami->kick_size == AMIGA_KICK_256K_SIZE && addr >= AMIGA_KICK_512K_BASE && addr < AMIGA_KICK_256K_BASE) {
        *handled = true;
        return;
    }

    // --- Extended ROM / expansion areas ---
    if ((addr >= 0xE00000 && addr < 0xE80000) ||
        (addr >= 0xE90000 && addr < 0xF00000) ||
        (addr >= 0xF00000 && addr < ami->kick_base)) {
        *handled = true;
        return;
    }

    // --- Dead zones ---
    if (addr >= ami->config.chip_ram_size && addr < 0xA00000) {
        if (ami->config.slow_ram_enabled &&
            addr >= AMIGA_SLOW_RAM_BASE &&
            addr < AMIGA_SLOW_RAM_BASE + AMIGA_SLOW_RAM_SIZE) {
            return; // Let through to memory
        }
        *handled = true;
        return;
    }

    if (addr >= 0xA00000 && addr < 0xBFD000) {
        *handled = true;
        return;
    }

    if (addr >= 0xC00000 && addr < 0xDFF000) {
        if (ami->config.slow_ram_enabled &&
            addr >= AMIGA_SLOW_RAM_BASE &&
            addr < AMIGA_SLOW_RAM_BASE + AMIGA_SLOW_RAM_SIZE) {
            return;
        }
        *handled = true;
        return;
    }

    // --- Final catch-all: absorb writes above chip RAM ---
    if (addr >= ami->config.chip_ram_size) {
        if (ami->config.slow_ram_enabled &&
            addr >= AMIGA_SLOW_RAM_BASE &&
            addr < AMIGA_SLOW_RAM_BASE + AMIGA_SLOW_RAM_SIZE) {
            return; // Let through for slow RAM
        }
        *handled = true;
        return;
    }
}

// ============================================================================
// Hardware RESET callback (M68K RESET instruction)
// Resets external peripherals only - CIAs, custom chips, DMA, interrupts
// Does NOT affect CPU state (that's the whole point of the RESET instruction)
// ============================================================================

static void amiga_hw_reset(void) {
    if (!ami) return;

    ami->hw_reset_count++;
    ESP_LOGI(TAG, "Hardware reset #%lu (RESET instruction) - clearing CIAs, custom chips",
             (unsigned long)ami->hw_reset_count);

    // After the initial double-RESET, any further RESETs indicate ColdReboot
    if (ami->hw_reset_count > 2) {
        uint32_t sp = m68k_get_reg(M68K_REG_SP);
        uint32_t pc = m68k_get_reg(M68K_REG_PC);
        ESP_LOGW(TAG, "ColdReboot detected (RESET #%lu): PC=$%06lX SP=$%06lX SR=$%04X",
                 (unsigned long)ami->hw_reset_count,
                 (unsigned long)pc, (unsigned long)sp,
                 (uint16_t)m68k_get_reg(M68K_REG_SR));
        ESP_LOGW(TAG, "  D0=$%08lX D1=$%08lX A0=$%08lX A1=$%08lX",
                 (unsigned long)m68k_get_reg(M68K_REG_D0),
                 (unsigned long)m68k_get_reg(M68K_REG_D1),
                 (unsigned long)m68k_get_reg(M68K_REG_A0),
                 (unsigned long)m68k_get_reg(M68K_REG_A1));
        ESP_LOGW(TAG, "  INTENA=$%04X INTREQ=$%04X DMACON=$%04X",
                 ami->intena, ami->intreq, ami->dmacon);
        // Stack dump for call trace
        for (int i = 0; i < 8; i++) {
            uint32_t addr = (sp + i * 4) & 0xFFFFFF;
            if (addr + 3 < ami->config.chip_ram_size) {
                uint32_t val = (m68k_read_memory_8(addr) << 24) |
                               (m68k_read_memory_8(addr + 1) << 16) |
                               (m68k_read_memory_8(addr + 2) << 8) |
                                m68k_read_memory_8(addr + 3);
                ESP_LOGW(TAG, "  [SP+%02d] $%06lX: $%08lX", i * 4, (unsigned long)addr, (unsigned long)val);
            }
        }
    }

    // Preserve OVL state across reset.  On real hardware, RESET zeros CIAs
    // which briefly makes OVL HIGH (pullup), but Kickstart immediately
    // reconfigures CIA-A to clear it.  In our non-cycle-accurate emulator
    // we skip that transient to avoid corrupting chip RAM reads.
    bool saved_ovl = ami->ovl;

    // Reset CIAs: zeros all registers, timers reload to 0xFFFF
    cia_init(&ami->cia_a);
    cia_init(&ami->cia_b);

    ami->prev_ciab_pra = 0x00; // CIA-B PRA is now 0 after reset

    // Restore OVL to pre-reset state (Kickstart will set it explicitly)
    ami->ovl = saved_ovl;

    // Clear all interrupts and DMA
    ami->intena = 0;
    ami->intreq = 0;
    ami->dmacon = 0;
    ami->adkcon = 0;

    // Reset CIA edge detection
    ami->prev_cia_a_int = false;
    ami->prev_cia_b_int = false;

    // Reset copper
    ami->copper.pc = 0;
    ami->copper.enabled = false;

    // Reset blitter
    ami->blitter.busy = false;

    // Clear pending disk DMA
    ami->prev_dsklen = 0;

    // Update IRQ (should be 0 now)
    m68k_set_irq(0);
    ami->active_irq_level = 0;
}

// ============================================================================
// Initialization
// ============================================================================

esp_err_t amiga_init(const amiga_config_t *config) {
    if (ami) {
        ESP_LOGW(TAG, "Amiga already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ami = heap_caps_calloc(1, sizeof(amiga_state_t), MALLOC_CAP_DEFAULT);
    if (!ami) {
        ESP_LOGE(TAG, "Failed to allocate Amiga state");
        return ESP_ERR_NO_MEM;
    }

    // Copy config
    ami->config = *config;
    if (ami->config.chip_ram_size == 0) {
        ami->config.chip_ram_size = AMIGA_CHIP_RAM_DEFAULT;
    }
    if (ami->config.chip_ram_size > AMIGA_CHIP_RAM_MAX) {
        ami->config.chip_ram_size = AMIGA_CHIP_RAM_MAX;
    }

    // --- Load Kickstart ROM ---
    if (!config->kick_path) {
        ESP_LOGE(TAG, "No Kickstart ROM path specified");
        free(ami);
        ami = NULL;
        return ESP_ERR_INVALID_ARG;
    }

    FILE *f = fopen(config->kick_path, "rb");
    if (!f) {
        ESP_LOGE(TAG, "Cannot open Kickstart ROM: %s", config->kick_path);
        free(ami);
        ami = NULL;
        return ESP_ERR_NOT_FOUND;
    }

    fseek(f, 0, SEEK_END);
    long rom_size = ftell(f);
    fseek(f, 0, SEEK_SET);

    // Validate ROM size (256KB or 512KB)
    if (rom_size != AMIGA_KICK_256K_SIZE && rom_size != AMIGA_KICK_512K_SIZE) {
        ESP_LOGE(TAG, "Invalid Kickstart ROM size: %ld (expected 256KB or 512KB)", rom_size);
        fclose(f);
        free(ami);
        ami = NULL;
        return ESP_ERR_INVALID_SIZE;
    }

    ami->kick_rom = heap_caps_malloc(rom_size, MALLOC_CAP_SPIRAM);
    if (!ami->kick_rom) {
        ami->kick_rom = heap_caps_malloc(rom_size, MALLOC_CAP_DEFAULT);
    }
    if (!ami->kick_rom) {
        ESP_LOGE(TAG, "Failed to allocate %ld bytes for Kickstart ROM", rom_size);
        fclose(f);
        free(ami);
        ami = NULL;
        return ESP_ERR_NO_MEM;
    }

    size_t got = fread(ami->kick_rom, 1, rom_size, f);
    fclose(f);

    if (got != (size_t)rom_size) {
        ESP_LOGE(TAG, "Failed to read Kickstart ROM (got %d, expected %ld)", (int)got, rom_size);
        free(ami->kick_rom);
        free(ami);
        ami = NULL;
        return ESP_FAIL;
    }

    ami->kick_size = (uint32_t)rom_size;
    ami->kick_base = (rom_size == AMIGA_KICK_512K_SIZE) ? AMIGA_KICK_512K_BASE : AMIGA_KICK_256K_BASE;

    ESP_LOGI(TAG, "Kickstart ROM loaded: %ld bytes (%s) at $%06lX",
             rom_size,
             rom_size == AMIGA_KICK_256K_SIZE ? "KS 1.x 256KB" : "KS 2.0+ 512KB",
             (unsigned long)ami->kick_base);

    // --- Initialize M68K CPU ---
    m68k_init();

    // Disable exception recovery (Kickstart has its own exception handlers)
    m68k_set_exception_recovery(false);
    m68k_watchdog_enable(false);

    // --- Initialize CIAs ---
    cia_init(&ami->cia_a);
    cia_init(&ami->cia_b);

    // CIA-A: OVL bit starts high (ROM overlay active at reset)
    ami->cia_a.pra = CIAA_PRA_OVL | CIAA_PRA_LED;
    ami->cia_a.ddra = 0x03; // OVL + LED are outputs
    ami->ovl = true;

    // CIA-B: all drive select lines high (no drive selected)
    ami->cia_b.pra = 0xFF;
    ami->cia_b.ddra = 0xFF; // All outputs
    ami->cia_b.ddrb = 0x00; // All inputs (drive status)
    ami->prev_ciab_pra = 0xFF;

    // --- Initialize custom chips ---
    ami->intena = 0;
    ami->intreq = 0;
    ami->dmacon = 0;
    ami->adkcon = 0;
    ami->bplcon0 = 0;
    ami->diwstrt = 0x2C81; // Default display window
    ami->diwstop = 0xF4C1;
    ami->ddfstrt = 0x0038;
    ami->ddfstop = 0x00D0;

    // Default palette (Kickstart colors)
    ami->color[0] = 0x0AAA;  // Gray background
    ami->color[1] = 0x0000;  // Black
    ami->color[2] = 0x0FFF;  // White
    ami->color[3] = 0x068B;  // Blue
    for (int i = 0; i < 32; i++) {
        ami->color_rgb565[i] = amiga_color_to_rgb565(ami->color[i]);
    }

    // Copper
    ami->copper.cop1lc = 0;
    ami->copper.cop2lc = 0;
    ami->copper.pc = 0;
    ami->copper.enabled = false;

    // Blitter
    memset(&ami->blitter, 0, sizeof(blitter_t));
    ami->blitter.bltafwm = 0xFFFF;
    ami->blitter.bltalwm = 0xFFFF;

    // Beam position
    ami->vpos = 0;
    ami->hpos = 0;
    ami->lof = true;

    // Timing (PAL)
    int total_lines = ami->config.pal_mode ? AMIGA_PAL_LINES : AMIGA_NTSC_LINES;
    ami->line_cycles = AMIGA_CPU_FREQ / (total_lines * 50); // ~227 CPU cycles per line at PAL
    ami->vbl_cycles = ami->line_cycles * total_lines;

    // Keyboard
    ami->kbd_head = 0;
    ami->kbd_tail = 0;
    ami->kbd_count = 0;
    ami->kbd_handshake = true; // Ready for first key

    // Floppy drives
    for (int i = 0; i < AMIGA_MAX_DRIVES; i++) {
        memset(&ami->floppy[i], 0, sizeof(floppy_drive_t));
    }
    ami->current_drive = -1;
    ami->disk_side = 0;
    ami->prev_dsklen = 0;

    // IDE drives
    for (int i = 0; i < 2; i++) {
        memset(&ami->ide[i], 0, sizeof(ide_drive_t));
        ami->ide[i].status = IDE_STATUS_DRDY;
    }
    memset(&ami->autoconfig, 0, sizeof(autoconfig_t));

    // --- Copy ROM data to M68K memory at ROM base address ---
    for (uint32_t i = 0; i < ami->kick_size; i++) {
        m68k_write_memory_8(ami->kick_base + i, ami->kick_rom[i]);
    }

    // For 256KB ROMs, also mirror at $F80000
    if (ami->kick_size == AMIGA_KICK_256K_SIZE) {
        for (uint32_t i = 0; i < ami->kick_size; i++) {
            m68k_write_memory_8(AMIGA_KICK_512K_BASE + i, ami->kick_rom[i]);
        }
    }

    // --- Copy ROM exception vector table to chip RAM ($000000-$3FF) ---
    // This ensures all 256 vectors have valid ROM defaults before KS installs its own.
    // Writes go to cpu->memory[] even with OVL active (write hook doesn't intercept $000000+).
    for (int i = 0; i < 1024 && i < (int)ami->kick_size; i++) {
        m68k_write_memory_8(i, ami->kick_rom[i]);
    }

    // --- Register I/O hooks ---
    m68k_set_io_hooks(amiga_io_read_hook, amiga_io_write_hook);
    m68k_set_irq_ack_hook(amiga_irq_ack);
    m68k_set_reset_hook(amiga_hw_reset);

    // --- Mount floppy disks if specified ---
    for (int i = 0; i < AMIGA_MAX_DRIVES; i++) {
        if (config->floppy_path[i]) {
            amiga_insert_floppy(i, config->floppy_path[i]);
        }
    }

    // --- Mount hard disks if specified ---
    for (int i = 0; i < 2; i++) {
        if (config->hdf_path[i]) {
            amiga_mount_hdf(i, config->hdf_path[i]);
        }
    }

    ami->initialized = true;
    ESP_LOGI(TAG, "Amiga emulator initialized: %luKB Chip RAM, %s",
             (unsigned long)(ami->config.chip_ram_size / 1024),
             ami->config.pal_mode ? "PAL" : "NTSC");

    return ESP_OK;
}

// ============================================================================
// Reset
// ============================================================================

void amiga_reset(void) {
    if (!ami) return;

    // Re-write reset vectors from ROM
    for (int i = 0; i < 8; i++) {
        m68k_write_memory_8(i, ami->kick_rom[i]);
    }

    // Reset CPU
    m68k_reset();

    // Reset OVL
    ami->cia_a.pra |= CIAA_PRA_OVL;
    ami->ovl = true;

    // Reset CIAs
    cia_init(&ami->cia_a);
    cia_init(&ami->cia_b);
    ami->cia_a.pra = CIAA_PRA_OVL | CIAA_PRA_LED;
    ami->cia_a.ddra = 0x03;
    ami->cia_b.pra = 0xFF;
    ami->cia_b.ddra = 0xFF;
    ami->cia_b.ddrb = 0x00;
    ami->prev_ciab_pra = 0xFF;

    // Reset custom chips
    ami->intena = 0;
    ami->intreq = 0;
    ami->dmacon = 0;
    ami->adkcon = 0;
    ami->bplcon0 = 0;
    ami->prev_dsklen = 0;
    memset(&ami->blitter, 0, sizeof(blitter_t));
    ami->blitter.bltafwm = 0xFFFF;
    ami->blitter.bltalwm = 0xFFFF;
    ami->copper.pc = 0;
    ami->copper.enabled = false;

    // Reset beam
    ami->vpos = 0;
    ami->hpos = 0;
    ami->lof = true;
    ami->cycles = 0;
    ami->frame_count = 0;

    // Reset CIA edge detection state
    ami->prev_cia_a_int = false;
    ami->prev_cia_b_int = false;

    // Reset keyboard
    ami->kbd_head = 0;
    ami->kbd_tail = 0;
    ami->kbd_count = 0;
    ami->kbd_handshake = true;

    // Reset floppy state
    for (int i = 0; i < AMIGA_MAX_DRIVES; i++) {
        ami->floppy[i].track = 0;
        ami->floppy[i].motor_on = false;
        ami->floppy[i].disk_changed = ami->floppy[i].inserted;
    }
    ami->current_drive = -1;

    ami->stopped = false;

    ESP_LOGI(TAG, "Amiga reset complete. SSP=$%08lX PC=$%08lX OVL=%d",
             (unsigned long)m68k_get_reg(M68K_REG_SP),
             (unsigned long)m68k_get_reg(M68K_REG_PC),
             ami->ovl);
}

// ============================================================================
// Main Emulation Loop
// ============================================================================

void amiga_run(void) {
    if (!ami || !ami->initialized) return;

    ami->running = true;
    ami->stopped = false;

    amiga_reset();

    ESP_LOGI(TAG, "Amiga emulation starting...");

    uint32_t cycles_this_line = 0;
    uint32_t render_timer = 0;
    int64_t last_time = esp_timer_get_time();

    while (ami->running && !ami->stopped) {
        // Execute a batch of M68K instructions
        uint64_t before_cycles = m68k_get_cycles();
        bool cpu_was_stopped = false;

        for (int i = 0; i < 100 && ami->running; i++) {
            if (m68k_is_halted()) {
                // CPU is STOPped - waiting for interrupt.
                // Advance time by one line so beam/CIA/VBL progress.
                // CIA ticks and beam advancement happen below.
                cpu_was_stopped = true;
                break;
            }

            m68k_step();
        }

        // Get actual cycles executed by CPU
        uint64_t after_cycles = m68k_get_cycles();
        uint32_t batch_cycles;

        if (cpu_was_stopped) {
            // CPU didn't execute anything, but time must still pass
            // Advance by one scanline worth of cycles
            batch_cycles = ami->line_cycles;
            vTaskDelay(pdMS_TO_TICKS(1));
        } else {
            batch_cycles = (uint32_t)(after_cycles - before_cycles);
            if (batch_cycles == 0) batch_cycles = 4; // Minimum progress
        }

        ami->cycles += batch_cycles;
        cycles_this_line += batch_cycles;

        // Tick CIA timers (CIAs run at E clock = CPU/10)
        int cia_ticks = batch_cycles / 10;
        if (cia_ticks < 1) cia_ticks = 1;
        cia_tick(&ami->cia_a, cia_ticks, false);
        cia_tick(&ami->cia_b, cia_ticks, true);

        // Deliver keyboard data
        kbd_deliver();

        // Advance beam position (one line at a time)
        while (cycles_this_line >= ami->line_cycles) {
            cycles_this_line -= ami->line_cycles;
            ami->hpos = 0;
            ami->vpos++;

            // CIA-B TOD is clocked by HSYNC (once per scanline)
            if (!ami->cia_b.tod_stopped) {
                ami->cia_b.tod = (ami->cia_b.tod + 1) & 0xFFFFFF;
                if (ami->cia_b.tod == ami->cia_b.tod_alarm) {
                    ami->cia_b.icr_data |= CIA_ICR_ALARM;
                    update_irq_level();
                }
            }

            int total_lines = ami->config.pal_mode ? AMIGA_PAL_LINES : AMIGA_NTSC_LINES;

            // Execute copper for this line (before VBL check)
            if (ami->vpos < total_lines) {
                copper_execute_line();
            }

            if (ami->vpos >= total_lines) {
                ami->vpos = 0;
                ami->frame_count++;
                ami->lof = !ami->lof;

                // Generate VBL interrupt
                ami->intreq |= AMIGA_INTF_VERTB;
                update_irq_level();

                // CIA-A TOD is clocked by VBLANK (50Hz PAL / 60Hz NTSC)
                if (!ami->cia_a.tod_stopped) {
                    ami->cia_a.tod = (ami->cia_a.tod + 1) & 0xFFFFFF;
                    if (ami->cia_a.tod == ami->cia_a.tod_alarm) {
                        ami->cia_a.icr_data |= CIA_ICR_ALARM;
                        update_irq_level();
                    }
                }

                // One-time log: first VBL with interrupts enabled
                if (!ami->first_vbl_logged &&
                    (ami->intena & AMIGA_INTF_INTEN) &&
                    (ami->intena & AMIGA_INTF_VERTB)) {
                    ESP_LOGI(TAG, "First VBL interrupt delivered (frame %lu, INTENA=$%04X DMACON=$%04X)",
                             (unsigned long)ami->frame_count, ami->intena, ami->dmacon);
                    ami->first_vbl_logged = true;
                }

                // Restart copper at COP1LC
                if (ami->dmacon & AMIGA_DMAF_COPEN) {
                    ami->copper.pc = ami->copper.cop1lc;
                    ami->copper.enabled = true;
                }

                // Execute copper for line 0
                copper_execute_line();

                // Render screen at ~20 fps (every 2-3 VBLs)
                render_timer++;
                if (render_timer >= 2) {
                    render_timer = 0;
                    render_bitplanes();
                    video_card_present();
                }

                // Poll keyboard input
                if (ps2_keyboard_is_initialized()) {
                    while (ps2_keyboard_available()) {
                        uint8_t key = ps2_keyboard_read();
                        if (key < 128 && ps2_to_amiga_scancode[key] != 0xFF && ps2_to_amiga_scancode[key] != 0) {
                            kbd_push_scancode(ps2_to_amiga_scancode[key]);      // Key press
                            kbd_push_scancode(ps2_to_amiga_scancode[key] | 0x80); // Key release
                        }
                    }
                }

                if (usb_keyboard_is_initialized()) {
                    while (usb_keyboard_available()) {
                        uint8_t key = usb_keyboard_read();
                        if (key < 128 && ps2_to_amiga_scancode[key] != 0xFF && ps2_to_amiga_scancode[key] != 0) {
                            kbd_push_scancode(ps2_to_amiga_scancode[key]);
                            kbd_push_scancode(ps2_to_amiga_scancode[key] | 0x80);
                        }
                    }
                }

                // Check UART for Ctrl+C
                uint8_t uart_buf[4];
                int uart_len = uart_read_bytes(0, uart_buf, sizeof(uart_buf), 0);
                for (int u = 0; u < uart_len; u++) {
                    if (uart_buf[u] == 0x03) { // Ctrl+C
                        ESP_LOGI(TAG, "Ctrl+C received, stopping Amiga");
                        ami->running = false;
                        break;
                    }
                }

                // Yield to other tasks
                int64_t now = esp_timer_get_time();
                if (now - last_time < 18000) { // Less than 18ms per frame
                    vTaskDelay(pdMS_TO_TICKS(1));
                }
                last_time = now;
            }
        }

        // Update hpos within line (approximate)
        ami->hpos = (cycles_this_line * AMIGA_PAL_HCLOCKS) / ami->line_cycles;
        if (ami->hpos > AMIGA_PAL_HCLOCKS - 1) ami->hpos = AMIGA_PAL_HCLOCKS - 1;
    }

    ESP_LOGI(TAG, "Amiga emulation stopped after %lu frames, %llu cycles",
             (unsigned long)ami->frame_count, (unsigned long long)ami->cycles);
}

// ============================================================================
// Stop / Status
// ============================================================================

void amiga_stop(void) {
    if (ami) {
        ami->running = false;
    }
}

bool amiga_is_running(void) {
    return ami && ami->running && !ami->stopped;
}

void amiga_get_state(char *buf, size_t buf_size) {
    if (!ami) {
        snprintf(buf, buf_size, "Amiga: not initialized\n");
        return;
    }

    snprintf(buf, buf_size,
        "Amiga State:\n"
        "  PC=$%08lX SP=$%08lX SR=$%04X\n"
        "  Cycles=%llu Frames=%lu\n"
        "  OVL=%d Beam V=%d H=%d\n"
        "  INTENA=$%04X INTREQ=$%04X DMACON=$%04X\n"
        "  BPLCON0=$%04X (%d planes, %s)\n"
        "  CIA-A: PRA=$%02X ICR=$%02X/%02X TA=%04X TB=%04X\n"
        "  CIA-B: PRA=$%02X ICR=$%02X/%02X TA=%04X TB=%04X\n"
        "  Copper: PC=$%06lX %s\n"
        "  Drive: %d Track: %d Side: %d Motor: %s\n"
        "  DF0: %s\n"
        "  HD0: %s\n",
        (unsigned long)m68k_get_reg(M68K_REG_PC),
        (unsigned long)m68k_get_reg(M68K_REG_SP),
        (uint16_t)m68k_get_reg(M68K_REG_SR),
        (unsigned long long)ami->cycles,
        (unsigned long)ami->frame_count,
        ami->ovl, ami->vpos, ami->hpos,
        ami->intena, ami->intreq, ami->dmacon,
        ami->bplcon0,
        (ami->bplcon0 >> 12) & 7,
        (ami->bplcon0 & 0x8000) ? "hires" : "lores",
        ami->cia_a.pra, ami->cia_a.icr_data, ami->cia_a.icr_mask,
        ami->cia_a.timer_a, ami->cia_a.timer_b,
        ami->cia_b.pra, ami->cia_b.icr_data, ami->cia_b.icr_mask,
        ami->cia_b.timer_a, ami->cia_b.timer_b,
        (unsigned long)ami->copper.pc,
        ami->copper.enabled ? "enabled" : "disabled",
        ami->current_drive,
        ami->current_drive >= 0 ? ami->floppy[ami->current_drive].track : 0,
        ami->disk_side,
        (ami->current_drive >= 0 && ami->floppy[ami->current_drive].motor_on) ? "ON" : "OFF",
        ami->floppy[0].inserted ? ami->floppy[0].path : "(empty)",
        ami->ide[0].mounted ? ami->ide[0].path : "(none)"
    );
}

// ============================================================================
// Floppy Disk Management
// ============================================================================

esp_err_t amiga_insert_floppy(int drive, const char *path) {
    if (!ami || drive < 0 || drive >= AMIGA_MAX_DRIVES) return ESP_ERR_INVALID_ARG;

    // Eject current disk if any
    amiga_eject_floppy(drive);

    FILE *f = fopen(path, "rb");
    if (!f) {
        ESP_LOGE(TAG, "Cannot open floppy image: %s", path);
        return ESP_ERR_NOT_FOUND;
    }

    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (size != AMIGA_ADF_SIZE) {
        ESP_LOGW(TAG, "Non-standard ADF size: %ld (expected %d)", size, AMIGA_ADF_SIZE);
    }

    ami->floppy[drive].image = f;
    ami->floppy[drive].size = (uint32_t)size;
    ami->floppy[drive].inserted = true;
    ami->floppy[drive].disk_changed = true;
    ami->floppy[drive].write_protected = true; // Default read-only
    ami->floppy[drive].track = 0;
    ami->floppy[drive].motor_on = false;
    strncpy(ami->floppy[drive].path, path, sizeof(ami->floppy[drive].path) - 1);

    ESP_LOGI(TAG, "DF%d: inserted %s (%ld bytes)", drive, path, size);
    return ESP_OK;
}

void amiga_eject_floppy(int drive) {
    if (!ami || drive < 0 || drive >= AMIGA_MAX_DRIVES) return;

    if (ami->floppy[drive].image) {
        fclose(ami->floppy[drive].image);
        ami->floppy[drive].image = NULL;
    }
    ami->floppy[drive].inserted = false;
    ami->floppy[drive].disk_changed = true;
    ami->floppy[drive].path[0] = '\0';

    ESP_LOGI(TAG, "DF%d: ejected", drive);
}

// ============================================================================
// Hard Disk Management
// ============================================================================

esp_err_t amiga_mount_hdf(int unit, const char *path) {
    if (!ami || unit < 0 || unit > 1) return ESP_ERR_INVALID_ARG;

    amiga_unmount_hdf(unit);

    FILE *f = fopen(path, "r+b"); // Read-write
    if (!f) {
        f = fopen(path, "rb"); // Fall back to read-only
    }
    if (!f) {
        ESP_LOGE(TAG, "Cannot open HDF image: %s", path);
        return ESP_ERR_NOT_FOUND;
    }

    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);

    ami->ide[unit].image = f;
    ami->ide[unit].size = (uint32_t)size;
    ami->ide[unit].sectors = (uint32_t)(size / 512);
    ami->ide[unit].mounted = true;
    ami->ide[unit].status = IDE_STATUS_DRDY;
    strncpy(ami->ide[unit].path, path, sizeof(ami->ide[unit].path) - 1);

    ESP_LOGI(TAG, "DH%d: mounted %s (%ld bytes, %lu sectors)",
             unit, path, size, (unsigned long)ami->ide[unit].sectors);
    return ESP_OK;
}

void amiga_unmount_hdf(int unit) {
    if (!ami || unit < 0 || unit > 1) return;

    if (ami->ide[unit].image) {
        fclose(ami->ide[unit].image);
        ami->ide[unit].image = NULL;
    }
    ami->ide[unit].mounted = false;
    ami->ide[unit].path[0] = '\0';

    ESP_LOGI(TAG, "DH%d: unmounted", unit);
}

// ============================================================================
// Cleanup
// ============================================================================

void amiga_destroy(void) {
    if (!ami) return;

    ami->running = false;

    // Close floppy images
    for (int i = 0; i < AMIGA_MAX_DRIVES; i++) {
        if (ami->floppy[i].image) {
            fclose(ami->floppy[i].image);
        }
    }

    // Close HDF images
    for (int i = 0; i < 2; i++) {
        if (ami->ide[i].image) {
            fclose(ami->ide[i].image);
        }
    }

    // Free ROM
    if (ami->kick_rom) {
        free(ami->kick_rom);
    }

    // Clear hooks
    m68k_set_io_hooks(NULL, NULL);
    m68k_set_irq_ack_hook(NULL);
    m68k_set_reset_hook(NULL);

    free(ami);
    ami = NULL;

    ESP_LOGI(TAG, "Amiga emulator destroyed");
}
