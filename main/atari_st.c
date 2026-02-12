/*
 * Atari ST Emulator Implementation
 * 
 * Uses the existing M68K CPU core (m68k_emulator.c) with Atari ST hardware
 * overlaid via I/O intercepts. The M68K emulator's read/write functions are
 * hooked to route hardware register accesses to this module.
 *
 * Architecture:
 *   - M68K CPU core handles all instruction execution
 *   - This module intercepts memory reads/writes to I/O region ($FF8000+)
 *   - TOS ROM is loaded into M68K address space at $FC0000
 *   - Video shifter converts Atari bitplane format to RGB565 for LCD
 *   - MFP 68901 provides timer interrupts (Timer C = 200Hz system tick)
 *   - ACIA handles keyboard input via IKBD protocol
 *   - FDC/DMA handles floppy disk access (.ST image files)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "atari_st.h"
#include "m68k_emulator.h"
#include "lcd_console.h"
#include "ps2_keyboard.h"
#include "usb_keyboard.h"
#include "usb_mouse.h"
#include "driver/uart.h"

static const char *TAG = "AtariST";

// ============================================================================
// Atari ST Hardware State
// ============================================================================

// MFP 68901 Timer structure
typedef struct {
    uint8_t control;     // Timer control register
    uint8_t data;        // Timer data register
    uint8_t counter;     // Current counter value
    uint8_t prescale;    // Prescaler counter
} mfp_timer_t;

// MFP 68901 state
typedef struct {
    uint8_t gpip;        // General purpose I/O port
    uint8_t aer;         // Active edge register
    uint8_t ddr;         // Data direction register
    uint8_t iera;        // Interrupt enable A
    uint8_t ierb;        // Interrupt enable B
    uint8_t ipra;        // Interrupt pending A
    uint8_t iprb;        // Interrupt pending B
    uint8_t isra;        // Interrupt in-service A
    uint8_t isrb;        // Interrupt in-service B
    uint8_t imra;        // Interrupt mask A
    uint8_t imrb;        // Interrupt mask B
    uint8_t vr;          // Vector register
    mfp_timer_t timer_a;
    mfp_timer_t timer_b;
    mfp_timer_t timer_c;
    mfp_timer_t timer_d;
    uint8_t scr;         // Sync character
    uint8_t ucr;         // USART control
    uint8_t rsr;         // Receiver status
    uint8_t tsr;         // Transmitter status
    uint8_t udr;         // USART data
} mfp_state_t;

// ACIA 6850 state
typedef struct {
    uint8_t control;
    uint8_t status;
    uint8_t rx_data;
    uint8_t tx_data;
    bool rx_full;
    bool tx_empty;
} acia_state_t;

// YM2149 PSG state
typedef struct {
    uint8_t selected_reg;
    uint8_t registers[16];
    // Register 14 = Port A (active-low): floppy side, drive select, RTS, DTR
    // Register 15 = Port B (active-low): parallel port data
} ym2149_state_t;

// Video Shifter state
typedef struct {
    uint32_t video_base;     // Video RAM base address
    uint32_t video_counter;  // Current video address counter
    uint8_t  resolution;     // 0=low, 1=med, 2=high
    uint8_t  sync_mode;      // 0=60Hz, 2=50Hz
    uint16_t palette[16];    // ST palette (0x0RGB, 3 bits per channel)
    uint16_t palette_rgb565[16]; // Pre-converted RGB565 palette for LCD
} video_state_t;

// FDC WD1772 state
typedef struct {
    uint8_t  command;
    uint8_t  status;
    uint8_t  track;
    uint8_t  sector;
    uint8_t  data;
    uint8_t  step_dir;       // 0=out (toward 0), 1=in
    bool     motor_on;
    // DMA
    uint32_t dma_addr;
    uint16_t dma_mode;
    uint16_t dma_sector_count;
    // Floppy images
    FILE    *floppy[2];
    char     floppy_path[2][256];
    uint32_t floppy_size[2];
    int      current_drive;  // 0 or 1
    int      current_side;   // 0 or 1
    // FDC busy state machine
    uint8_t  pending_cmd;    // Command waiting to execute (0 = idle)
    uint32_t busy_cycles;    // CPU cycles remaining until command completes
    bool     status_type_i;  // True if last command was Type I (affects status register bits)
} fdc_state_t;

// Keyboard IKBD state
typedef struct {
    uint8_t buffer[64];
    int     head;
    int     tail;
    int     count;
    // IKBD command state machine
    uint8_t cmd;            // Current command being received (0 = idle)
    int     cmd_bytes_left; // Remaining parameter bytes for current command
    // Reset response delay (real IKBD takes ~63ms for self-test)
    bool    reset_pending;    // True when reset response is being delayed
    uint32_t reset_delay;     // CPU cycles remaining before sending response
} ikbd_state_t;

// Complete Atari ST state
typedef struct {
    bool initialized;
    bool running;
    bool stopped;
    
    // Hardware
    mfp_state_t  mfp;
    acia_state_t kbd_acia;
    acia_state_t midi_acia;
    ym2149_state_t ym;
    video_state_t video;
    fdc_state_t  fdc;
    ikbd_state_t ikbd;
    
    // TOS ROM (loaded from file)
    uint8_t *tos_rom;
    uint32_t tos_size;
    uint32_t tos_base;     // Where TOS is mapped ($FC0000 or $E00000)
    
    // Frame buffer for LCD output
    uint16_t *fb_slice;    // RGB565 DMA slice buffer
    
    // Timing
    uint64_t cycles;
    uint32_t vbl_cycles;   // Cycles per VBL (160256 for 50Hz @ 8MHz)
    uint32_t hbl_cycles;   // Cycles per HBL (512)
    uint32_t mfp_cycles;   // Cycles since last MFP tick
    int      scanline;     // Current scanline
    
    // Interrupt state
    uint8_t  pending_mfp_vector; // MFP vector to return on IRQ ACK (0=none)
    uint8_t  active_irq;         // Currently asserted IRQ level
    bool     vbl_pending;        // VBL interrupt pending (level 4)
    bool     hbl_pending;        // HBL interrupt pending (level 2)
    
    // Config
    atari_st_config_t config;
    
    // I/O diagnostic trace
    uint32_t last_io_read_addr;
    uint32_t io_read_count;
} atari_st_state_t;

static atari_st_state_t *st = NULL;

// MFP timer prescaler values (indexed by control register bits 0-2)
static const int mfp_prescale[] = { 0, 4, 10, 16, 50, 64, 100, 200 };

// ============================================================================
// Forward Declarations
// ============================================================================

static uint8_t atari_io_read_byte(uint32_t addr);
static uint16_t atari_io_read_word(uint32_t addr);
static void atari_io_write_byte(uint32_t addr, uint8_t value);
static void atari_io_write_word(uint32_t addr, uint16_t value);
static void mfp_tick(int cpu_cycles);
static void fdc_execute_command(uint8_t cmd);
static void fdc_complete_command(uint8_t cmd);
static void fdc_tick(int cpu_cycles);
static void ikbd_push(uint8_t byte);
static uint8_t ikbd_pop(void);
static void ikbd_flush(void);
static void ikbd_tick(int cpu_cycles);
static void ikbd_process_byte(uint8_t value);
static void atari_palette_to_rgb565(int index);
static void render_low_res(void);
static void update_irq_level(void);

// ============================================================================
// IRQ Acknowledge Hook
// ============================================================================

// Called by M68K CPU when it acknowledges an interrupt
// Returns the vector number for vectored interrupts, 0 for autovector
static uint8_t atari_irq_ack(uint8_t level) {
    if (!st) return 0;

    switch (level) {
        case 6: { // MFP interrupt - vectored
            uint8_t vec = st->pending_mfp_vector;
            st->pending_mfp_vector = 0;

            // NOW clear the MFP pending bit (not before CPU acknowledges)
            uint8_t vec_base = st->mfp.vr & 0xF0;
            // MC68901 datasheet: S bit (bit 3): S=0 → auto-EOI, S=1 → software EOI
            bool auto_eoi = (st->mfp.vr & 0x08) == 0;
            if (vec >= vec_base + 8) {
                // Register A interrupt (vectors vec_base+8 to vec_base+15)
                int bit = vec - (vec_base + 8);
                st->mfp.ipra &= ~(1 << bit);
                if (!auto_eoi) {
                    // Software EOI: set in-service bit (handler clears it later)
                    st->mfp.isra |= (1 << bit);
                }
                // Auto-EOI: don't set ISR bit at all
            } else if (vec >= vec_base) {
                // Register B interrupt (vectors vec_base+0 to vec_base+7)
                int bit = vec - vec_base;
                st->mfp.iprb &= ~(1 << bit);
                if (!auto_eoi) {
                    st->mfp.isrb |= (1 << bit);
                }
            }

            // Re-evaluate if more interrupts are pending
            update_irq_level();
            return vec; // Return MFP vector number (non-zero = vectored)
        }
        case 4: // VBL - autovectored
            st->vbl_pending = false;
            update_irq_level();
            return 0; // Autovector
        case 2: // HBL - autovectored
            st->hbl_pending = false;
            update_irq_level();
            return 0; // Autovector
        default:
            update_irq_level();
            return 0;
    }
}

// Update the IRQ level asserted to the M68K CPU based on pending interrupts
// Priority: MFP (6) > VBL (4) > HBL (2)
// IMPORTANT: This function does NOT clear MFP pending bits - that happens
// only when the CPU actually acknowledges the interrupt in atari_irq_ack().
static void update_irq_level(void) {
    if (!st) return;

    uint8_t level = 0;

    // Check MFP pending interrupts (level 6) - highest priority
    uint8_t pend_a = st->mfp.ipra & st->mfp.iera & st->mfp.imra & ~st->mfp.isra;
    uint8_t pend_b = st->mfp.iprb & st->mfp.ierb & st->mfp.imrb & ~st->mfp.isrb;

    if (pend_a || pend_b) {
        uint8_t vec_base = st->mfp.vr & 0xF0;

        // Find highest priority pending MFP interrupt (don't clear yet)
        if (pend_a) {
            for (int i = 7; i >= 0; i--) {
                if (pend_a & (1 << i)) {
                    st->pending_mfp_vector = vec_base + 8 + i;
                    level = 6;
                    break;
                }
            }
        } else {
            for (int i = 7; i >= 0; i--) {
                if (pend_b & (1 << i)) {
                    st->pending_mfp_vector = vec_base + i;
                    level = 6;
                    break;
                }
            }
        }
    }

    // Check VBL (level 4) - only if no higher priority pending
    if (level < 4 && st->vbl_pending) {
        level = 4;
    }

    // Check HBL (level 2) - only if no higher priority pending
    if (level < 2 && st->hbl_pending) {
        level = 2;
    }

    st->active_irq = level;
    m68k_set_irq(level);
}

// ============================================================================
// Atari ST I/O Read Handlers  
// ============================================================================

static uint8_t atari_io_read_byte(uint32_t addr) {
    
    // MFP 68901 ($FFFA00-$FFFA3F) - odd addresses only
    if (addr >= 0xFFFA00 && addr <= 0xFFFA3F) {
        switch (addr) {
            case ATARI_MFP_GPIP:  return st->mfp.gpip;
            case ATARI_MFP_AER:   return st->mfp.aer;
            case ATARI_MFP_DDR:   return st->mfp.ddr;
            case ATARI_MFP_IERA:  return st->mfp.iera;
            case ATARI_MFP_IERB:  return st->mfp.ierb;
            case ATARI_MFP_IPRA:  return st->mfp.ipra;
            case ATARI_MFP_IPRB:  return st->mfp.iprb;
            case ATARI_MFP_ISRA:  return st->mfp.isra;
            case ATARI_MFP_ISRB:  return st->mfp.isrb;
            case ATARI_MFP_IMRA:  return st->mfp.imra;
            case ATARI_MFP_IMRB:  return st->mfp.imrb;
            case ATARI_MFP_VR:    return st->mfp.vr;
            case ATARI_MFP_TACR:  return st->mfp.timer_a.control;
            case ATARI_MFP_TBCR:  return st->mfp.timer_b.control;
            case ATARI_MFP_TCDCR: return (st->mfp.timer_c.control << 4) | st->mfp.timer_d.control;
            case ATARI_MFP_TADR:  return st->mfp.timer_a.counter;
            case ATARI_MFP_TBDR:  return st->mfp.timer_b.counter;
            case ATARI_MFP_TCDR:  return st->mfp.timer_c.counter;
            case ATARI_MFP_TDDR:  return st->mfp.timer_d.counter;
            case ATARI_MFP_SCR:   return st->mfp.scr;
            case ATARI_MFP_UCR:   return st->mfp.ucr;
            case ATARI_MFP_RSR:   return st->mfp.rsr;
            case ATARI_MFP_TSR:   return st->mfp.tsr | 0x80; // Transmitter always ready
            case ATARI_MFP_UDR:   return st->mfp.udr;
            default:              return 0;
        }
    }
    
    // Keyboard ACIA ($FFFC00-$FFFC03)
    if (addr >= 0xFFFC00 && addr <= 0xFFFC03) {
        if (addr == ATARI_KBD_CTRL || addr == (ATARI_KBD_CTRL + 1)) {
            // Status register (reference: acia.v serial_status)
            //   bit 0: RDRF (RX data register full)
            //   bit 1: TDRE (TX data register empty)
            //   bit 7: IRQ (composite of RX/TX interrupt conditions)
            uint8_t status = 0x02; // TX empty
            if (st->ikbd.count > 0) {
                status |= 0x01;  // RX full (RDRF)
                // bit 7: IRQ active if RX interrupt enabled (control reg bit 7)
                if (st->kbd_acia.control & 0x80) {
                    status |= 0x80;  // IRQ flag
                }
            }
            return status;
        }
        if (addr == ATARI_KBD_DATA || addr == (ATARI_KBD_DATA + 1)) {
            // Data register - pop from IKBD buffer
            if (st->ikbd.count > 0) {
                uint8_t byte = ikbd_pop();
                if (st->ikbd.count > 0) {
                    // More data in buffer - keep ACIA IRQ asserted
                    // GPIP4 stays LOW, re-trigger MFP pending bit
                    st->mfp.iprb |= 0x40;
                    update_irq_level();
                } else {
                    // Buffer now empty - de-assert ACIA IRQ
                    st->mfp.gpip |= 0x10;  // GPIP4 HIGH = no ACIA data
                }
                return byte;
            }
            // No data available
            st->mfp.gpip |= 0x10;  // GPIP4 HIGH = no ACIA data
            return 0;
        }
    }
    
    // MIDI ACIA ($FFFC04-$FFFC07)
    if (addr >= 0xFFFC04 && addr <= 0xFFFC07) {
        if (addr == ATARI_MIDI_CTRL || addr == (ATARI_MIDI_CTRL + 1)) {
            return 0x02; // TX empty, no RX
        }
        return 0;
    }
    
    // Video registers ($FF8200-$FF8260)
    if (addr >= 0xFF8200 && addr <= 0xFF8260) {
        switch (addr) {
            case ATARI_VID_ADDR_HI:  return (st->video.video_base >> 16) & 0xFF;
            case ATARI_VID_ADDR_MID: return (st->video.video_base >> 8) & 0xFF;
            case ATARI_VID_SYNC:     return st->video.sync_mode;
            case ATARI_VID_RES:      return st->video.resolution;
            default:
                // Palette registers ($FF8240-$FF825F)
                if (addr >= 0xFF8240 && addr < 0xFF8260) {
                    int idx = (addr - 0xFF8240) / 2;
                    if (addr & 1) return st->video.palette[idx] & 0xFF;
                    return (st->video.palette[idx] >> 8) & 0xFF;
                }
                return 0;
        }
    }
    
    // YM2149 ($FF8800)
    if (addr >= 0xFF8800 && addr <= 0xFF8803) {
        if (addr == ATARI_YM_REG_SELECT || addr == (ATARI_YM_REG_SELECT + 1)) {
            if (st->ym.selected_reg < 16) {
                return st->ym.registers[st->ym.selected_reg];
            }
        }
        return 0;
    }
    
    // DMA/FDC ($FF8604-$FF860D)
    if (addr >= 0xFF8604 && addr <= 0xFF860D) {
        switch (addr) {
            case 0xFF8604: case 0xFF8605:
                // FDC/HDC register access (depends on DMA mode register)
                // Reference (MiSTeryNano dma.v):
                //   bit4=1 → sector count register
                //   bit4=0, bit3=0 → FDC register (selected by bits 2:1)
                //   bit4=0, bit3=1 → HDC/ACSI register
                if (st->fdc.dma_mode & 0x10) {
                    // Sector count register
                    return st->fdc.dma_sector_count;
                } else if ((st->fdc.dma_mode & 0x08) == 0) {
                    // FDC register selected by dma_mode bits 2:1
                    switch ((st->fdc.dma_mode >> 1) & 3) {
                        case 0: {
                            // Status register read
                            uint8_t status = st->fdc.status;
                            // Type I status: update dynamic bits on read (Hatari reference)
                            if (st->fdc.status_type_i && !(status & FDC_STATUS_BUSY)) {
                                if (st->fdc.track == 0) status |= 0x04; else status &= ~0x04;
                                // WPRT: set if no disk inserted
                                int drv = st->fdc.current_drive;
                                if (drv < 0 || drv > 1 || !st->fdc.floppy[drv]) status |= 0x40;
                                else status &= ~0x40;
                            }
                            // Reading FDC status clears IRQ (WD1772 behavior)
                            st->mfp.gpip |= 0x20;  // GPIP5 HIGH = FDC IRQ deasserted
                            return status;
                        }
                        case 1: return st->fdc.track;
                        case 2: return st->fdc.sector;
                        case 3: return st->fdc.data;
                    }
                }
                // HDC/ACSI read - no device present, return $FF
                return 0xFF;
            case 0xFF8606: case 0xFF8607:
                // DMA status register (read):
                //   bit 0: 1=DMA OK, 0=DMA error (reference: always 1)
                //   bit 1: 1=sector count non-zero
                //   bit 2: FDC DRQ state
                return (st->fdc.dma_sector_count != 0) ? 0x03 : 0x01;
            default:
                return 0;
        }
    }
    
    // Memory configuration ($FF8001)
    if (addr == 0xFF8001) {
        // Return memory config based on RAM size (0x0A = 1MB)
        if (st->config.ram_size >= 4 * 1024 * 1024) return 0x0E;
        if (st->config.ram_size >= 2 * 1024 * 1024) return 0x0C;
        if (st->config.ram_size >= 1 * 1024 * 1024) return 0x0A;
        return 0x08; // 512KB
    }
    
    return 0;
}

static uint16_t atari_io_read_word(uint32_t addr) {
    // I/O diagnostic trace (word reads)
    if (st) {
        st->last_io_read_addr = addr;
        st->io_read_count++;
    }
    // Palette registers are 16-bit
    if (addr >= 0xFF8240 && addr < 0xFF8260) {
        int idx = (addr - 0xFF8240) / 2;
        return st->video.palette[idx];
    }
    // DMA status (word read)
    if (addr == 0xFF8606) {
        return (st->fdc.dma_sector_count != 0) ? 0x03 : 0x01;
    }
    // DMA/FDC data register (word read at $FF8604)
    if (addr == 0xFF8604) {
        // Use same logic as byte read: check dma_mode bits
        if (st->fdc.dma_mode & 0x10) {
            return st->fdc.dma_sector_count;
        } else if ((st->fdc.dma_mode & 0x08) == 0) {
            // FDC register read
            switch ((st->fdc.dma_mode >> 1) & 3) {
                case 0: {
                    uint8_t status = st->fdc.status;
                    if (st->fdc.status_type_i && !(status & FDC_STATUS_BUSY)) {
                        if (st->fdc.track == 0) status |= 0x04; else status &= ~0x04;
                        int drv = st->fdc.current_drive;
                        if (drv < 0 || drv > 1 || !st->fdc.floppy[drv]) status |= 0x40;
                        else status &= ~0x40;
                    }
                    st->mfp.gpip |= 0x20;  // Reading status clears FDC IRQ
                    return status;
                }
                case 1: return st->fdc.track;
                case 2: return st->fdc.sector;
                case 3: return st->fdc.data;
            }
        } else {
            // HDC/ACSI register read - no device, return $FF
            return 0xFF;
        }
        return 0;
    }
    // Everything else - combine two byte reads
    return (atari_io_read_byte(addr) << 8) | atari_io_read_byte(addr + 1);
}

// ============================================================================
// Atari ST I/O Write Handlers
// ============================================================================

static void atari_io_write_byte(uint32_t addr, uint8_t value) {
    
    // MFP 68901 ($FFFA00-$FFFA3F)
    if (addr >= 0xFFFA00 && addr <= 0xFFFA3F) {
        switch (addr) {
            case ATARI_MFP_GPIP:
                // Only modify bits configured as outputs (DDR=1)
                // Input bits (DDR=0) retain hardware-driven state
                st->mfp.gpip = (st->mfp.gpip & ~st->mfp.ddr) | (value & st->mfp.ddr);
                break;
            case ATARI_MFP_AER:   st->mfp.aer = value; break;
            case ATARI_MFP_DDR:   st->mfp.ddr = value; break;
            case ATARI_MFP_IERA:  st->mfp.iera = value; update_irq_level(); break;
            case ATARI_MFP_IERB:  st->mfp.ierb = value; update_irq_level(); break;
            case ATARI_MFP_IPRA:  st->mfp.ipra &= value; update_irq_level(); break;  // Writing 0 clears bits
            case ATARI_MFP_IPRB:  st->mfp.iprb &= value; update_irq_level(); break;
            case ATARI_MFP_ISRA:  st->mfp.isra &= value; update_irq_level(); break;
            case ATARI_MFP_ISRB:  st->mfp.isrb &= value; update_irq_level(); break;
            case ATARI_MFP_IMRA:  st->mfp.imra = value; update_irq_level(); break;
            case ATARI_MFP_IMRB:  st->mfp.imrb = value; update_irq_level(); break;
            case ATARI_MFP_VR:
                st->mfp.vr = value;
                // Reference (mfp.v): when S bit (bit 3) is 0 (auto-EOI mode),
                // all ISR bits are cleared immediately
                if (!(value & 0x08)) {
                    st->mfp.isra = 0;
                    st->mfp.isrb = 0;
                }
                update_irq_level();
                break;
            case ATARI_MFP_TACR:
                st->mfp.timer_a.control = value & 0x0F;
                if (value == 0) st->mfp.timer_a.counter = st->mfp.timer_a.data;
                break;
            case ATARI_MFP_TBCR:
                st->mfp.timer_b.control = value & 0x0F;
                if (value == 0) st->mfp.timer_b.counter = st->mfp.timer_b.data;
                break;
            case ATARI_MFP_TCDCR:
                st->mfp.timer_c.control = (value >> 4) & 0x07;
                st->mfp.timer_d.control = value & 0x07;
                break;
            case ATARI_MFP_TADR:
                st->mfp.timer_a.data = value;
                if (st->mfp.timer_a.control == 0) st->mfp.timer_a.counter = value;
                break;
            case ATARI_MFP_TBDR:
                st->mfp.timer_b.data = value;
                if (st->mfp.timer_b.control == 0) st->mfp.timer_b.counter = value;
                break;
            case ATARI_MFP_TCDR:
                st->mfp.timer_c.data = value;
                if (st->mfp.timer_c.control == 0) st->mfp.timer_c.counter = value;
                break;
            case ATARI_MFP_TDDR:
                st->mfp.timer_d.data = value;
                if (st->mfp.timer_d.control == 0) st->mfp.timer_d.counter = value;
                break;
            case ATARI_MFP_SCR:   st->mfp.scr = value; break;
            case ATARI_MFP_UCR:   st->mfp.ucr = value; break;
            case ATARI_MFP_RSR:   st->mfp.rsr = value; break;
            case ATARI_MFP_TSR:   st->mfp.tsr = value; break;
            case ATARI_MFP_UDR:   st->mfp.udr = value; break;
        }
        return;
    }
    
    // Keyboard ACIA ($FFFC00-$FFFC03)
    if (addr >= 0xFFFC00 && addr <= 0xFFFC03) {
        if (addr == ATARI_KBD_CTRL || addr == (ATARI_KBD_CTRL + 1)) {
            st->kbd_acia.control = value;
            // Reference (acia.v): master reset when CR[1:0] = 11b
            if ((value & 0x03) == 0x03) {
                // Master reset - flush receive buffer and reset state
                st->kbd_acia.status = 0x02;
                st->kbd_acia.rx_full = false;
                st->kbd_acia.tx_empty = true;
                ikbd_flush();
                st->ikbd.cmd = 0;
                st->ikbd.cmd_bytes_left = 0;
                st->mfp.gpip |= 0x10;  // GPIP4 HIGH = no ACIA data
            }
        }
        if (addr == ATARI_KBD_DATA || addr == (ATARI_KBD_DATA + 1)) {
            st->kbd_acia.tx_data = value;
            // Process IKBD command byte
            ikbd_process_byte(value);
        }
        return;
    }
    
    // MIDI ACIA ($FFFC04-$FFFC07)
    if (addr >= 0xFFFC04 && addr <= 0xFFFC07) {
        if (addr == ATARI_MIDI_CTRL || addr == (ATARI_MIDI_CTRL + 1)) {
            st->midi_acia.control = value;
        }
        return;
    }
    
    // Video registers ($FF8200-$FF8260)
    if (addr >= 0xFF8200 && addr <= 0xFF8260) {
        switch (addr) {
            case ATARI_VID_ADDR_HI:
                st->video.video_base = (st->video.video_base & 0x00FFFF) | ((uint32_t)(value & 0x3F) << 16);
                break;
            case ATARI_VID_ADDR_MID:
                st->video.video_base = (st->video.video_base & 0xFF00FF) | ((uint32_t)value << 8);
                break;
            case ATARI_VID_SYNC:
                st->video.sync_mode = value & 0x02;
                break;
            case ATARI_VID_RES:
                st->video.resolution = value & 0x03;
                ESP_LOGI(TAG, "Video resolution set to %d (%s)", value & 3,
                         (value & 3) == 0 ? "320x200x16" :
                         (value & 3) == 1 ? "640x200x4" : "640x400x2");
                break;
            default:
                // Palette registers ($FF8240-$FF825F) - byte writes
                if (addr >= 0xFF8240 && addr < 0xFF8260) {
                    int idx = (addr - 0xFF8240) / 2;
                    if (addr & 1) {
                        st->video.palette[idx] = (st->video.palette[idx] & 0xFF00) | value;
                    } else {
                        st->video.palette[idx] = (st->video.palette[idx] & 0x00FF) | (value << 8);
                    }
                    atari_palette_to_rgb565(idx);
                }
                break;
        }
        return;
    }
    
    // YM2149 ($FF8800-$FF8803)
    if (addr >= 0xFF8800 && addr <= 0xFF8803) {
        if (addr == ATARI_YM_REG_SELECT || addr == (ATARI_YM_REG_SELECT + 1)) {
            st->ym.selected_reg = value & 0x0F;
        }
        if (addr == ATARI_YM_REG_WRITE || addr == (ATARI_YM_REG_WRITE + 1)) {
            if (st->ym.selected_reg < 16) {
                st->ym.registers[st->ym.selected_reg] = value;
                
                // Register 14 (Port A) controls floppy
                if (st->ym.selected_reg == 14) {
                    st->fdc.current_side = (value & 0x01) ? 0 : 1;
                    // Drive select: bit 1 = drive A, bit 2 = drive B (active low)
                    if (!(value & 0x02)) st->fdc.current_drive = 0;
                    if (!(value & 0x04)) st->fdc.current_drive = 1;
                }
            }
        }
        return;
    }
    
    // DMA/FDC ($FF8604-$FF860D)
    if (addr >= 0xFF8604 && addr <= 0xFF860D) {
        switch (addr) {
            case 0xFF8604: case 0xFF8605:
                // Reference (MiSTeryNano dma.v):
                //   bit4=1 → sector count register write
                //   bit4=0, bit3=0 → FDC register write (selected by bits 2:1)
                //   bit4=0, bit3=1 → HDC/ACSI register write
                if (st->fdc.dma_mode & 0x10) {
                    // Sector count register
                    st->fdc.dma_sector_count = value;
                } else if ((st->fdc.dma_mode & 0x08) == 0) {
                    // FDC register write
                    switch ((st->fdc.dma_mode >> 1) & 3) {
                        case 0:
                            st->fdc.command = value;
                            fdc_execute_command(value);
                            break;
                        case 1: st->fdc.track = value; break;
                        case 2: st->fdc.sector = value; break;
                        case 3: st->fdc.data = value; break;
                    }
                }
                // HDC/ACSI write - not implemented
                break;
            case 0xFF8606: case 0xFF8607: {
                // Byte write to DMA mode register
                // Even byte ($FF8606) = high byte, Odd byte ($FF8607) = low byte
                uint16_t new_mode;
                if (addr == 0xFF8606) {
                    new_mode = (st->fdc.dma_mode & 0x00FF) | ((uint16_t)value << 8);
                } else {
                    new_mode = (st->fdc.dma_mode & 0xFF00) | value;
                }
                // Toggling bit 8 (R/W direction) resets DMA (Hatari reference)
                if ((st->fdc.dma_mode ^ new_mode) & 0x100) {
                    st->fdc.dma_sector_count = 0;
                }
                st->fdc.dma_mode = new_mode;
                break;
            }
            case 0xFF8609:
                st->fdc.dma_addr = (st->fdc.dma_addr & 0x00FFFF) | ((uint32_t)(value & 0x3F) << 16);
                break;
            case 0xFF860B:
                st->fdc.dma_addr = (st->fdc.dma_addr & 0xFF00FF) | ((uint32_t)value << 8);
                break;
            case 0xFF860D:
                st->fdc.dma_addr = (st->fdc.dma_addr & 0xFFFF00) | value;
                break;
        }
        return;
    }
    
    // Memory configuration
    if (addr == 0xFF8001) {
        // Read-only on real hardware, ignore
        return;
    }
}

static void atari_io_write_word(uint32_t addr, uint16_t value) {
    // Palette registers are naturally 16-bit
    if (addr >= 0xFF8240 && addr < 0xFF8260) {
        int idx = (addr - 0xFF8240) / 2;
        st->video.palette[idx] = value;
        atari_palette_to_rgb565(idx);
        return;
    }
    // DMA mode register
    if (addr == 0xFF8606) {
        // Toggling bit 8 (R/W direction) resets DMA state (Hatari reference)
        if ((st->fdc.dma_mode ^ value) & 0x100) {
            st->fdc.dma_sector_count = 0;
        }
        st->fdc.dma_mode = value;
        return;
    }
    // DMA data / FDC access (word write at $FF8604)
    // Reference (MiSTeryNano dma.v):
    //   bit4=1 → sector count register
    //   bit4=0, bit3=0 → FDC register (selected by bits 2:1)
    //   bit4=0, bit3=1 → HDC/ACSI register
    if (addr == 0xFF8604) {
        if (st->fdc.dma_mode & 0x10) {
            // Sector count register
            st->fdc.dma_sector_count = value;
        } else if ((st->fdc.dma_mode & 0x08) == 0) {
            // FDC register write
            switch ((st->fdc.dma_mode >> 1) & 3) {
                case 0: st->fdc.command = value & 0xFF; fdc_execute_command(value & 0xFF); break;
                case 1: st->fdc.track = value & 0xFF; break;
                case 2: st->fdc.sector = value & 0xFF; break;
                case 3: st->fdc.data = value & 0xFF; break;
            }
        }
        // HDC/ACSI word write - not implemented
        return;
    }
    // Split into byte writes for other registers
    atari_io_write_byte(addr, (value >> 8) & 0xFF);
    atari_io_write_byte(addr + 1, value & 0xFF);
}

// ============================================================================
// MFP 68901 Timer Emulation
// ============================================================================

static void mfp_timer_tick(mfp_timer_t *timer, uint8_t int_bit, bool is_a_reg) {
    if (timer->control == 0 || timer->control > 7) return;
    
    int prescale = mfp_prescale[timer->control];
    timer->prescale++;
    
    if (timer->prescale >= prescale) {
        timer->prescale = 0;
        timer->counter--;
        
        if (timer->counter == 0) {
            timer->counter = timer->data;
            
            // Set interrupt pending
            if (is_a_reg) {
                st->mfp.ipra |= int_bit;
            } else {
                st->mfp.iprb |= int_bit;
            }
        }
    }
}

static void mfp_tick(int cpu_cycles) {
    // MFP runs at 2.4576 MHz on Atari ST
    // CPU runs at 8 MHz, so ~3.26 CPU cycles per MFP cycle
    // We approximate: tick MFP every 4 CPU cycles
    
    st->mfp_cycles += cpu_cycles;
    
    while (st->mfp_cycles >= 4) {
        st->mfp_cycles -= 4;
        
        // Tick all timers
        mfp_timer_tick(&st->mfp.timer_a, 0x20, true);   // Timer A = IPRA bit 5
        mfp_timer_tick(&st->mfp.timer_b, 0x01, true);   // Timer B = IPRA bit 0
        mfp_timer_tick(&st->mfp.timer_c, 0x20, false);  // Timer C = IPRB bit 5
        mfp_timer_tick(&st->mfp.timer_d, 0x10, false);  // Timer D = IPRB bit 4
    }
    
    // Update IRQ level based on pending MFP interrupts
    update_irq_level();
}

// ============================================================================
// FDC WD1772 Emulation
// ============================================================================

// Assert FDC completion interrupt via MFP GPIP bit 5
// On real hardware, FDC pulls IRQ line low → MFP GPIP5 edge → IPRB bit 7
static void fdc_assert_interrupt(void) {
    st->mfp.gpip &= ~0x20;     // GPIP bit 5 low (FDC IRQ active)
    st->mfp.iprb |= 0x80;      // IPRB bit 7 = GPIP5 interrupt pending
    update_irq_level();          // Re-evaluate so CPU sees it immediately
}

// Start FDC command: sets BUSY status, defers actual execution to fdc_tick/fdc_complete_command
static void fdc_execute_command(uint8_t cmd) {
    uint8_t cmd_type = cmd & 0xF0;
    
    ESP_LOGI(TAG, "FDC command: 0x%02X type=%s track=%d sector=%d side=%d drive=%d",
             cmd,
             cmd_type <= 0x70 ? "I" : (cmd_type <= 0xB0 ? "II" : "III"),
             st->fdc.track, st->fdc.sector, st->fdc.current_side, st->fdc.current_drive);
    
    // WD1772: Writing to the command register clears the IRQ output
    // This is CRITICAL - without this, GPIP5 stays LOW from previous command
    // and TOS ACSI probe sees the stale IRQ as "device responded"
    st->mfp.gpip |= 0x20;  // GPIP5 HIGH = FDC IRQ cleared
    
    // Handle Force Interrupt first (Type IV - doesn't set BUSY)
    if (cmd_type == 0xD0) {
        // Cancel any pending/running command
        st->fdc.busy_cycles = 0;
        st->fdc.pending_cmd = 0;
        // If FDC was idle (not busy), force status format to Type I (Hatari reference)
        if (!(st->fdc.status & FDC_STATUS_BUSY)) {
            st->fdc.status_type_i = true;
        }
        // Construct Type I status: clear BUSY, keep motor, set TR00 if at track 0
        st->fdc.status = st->fdc.motor_on ? FDC_STATUS_MOTOR_ON : 0x00;
        if (st->fdc.track == 0) st->fdc.status |= 0x04;
        // $D8 (I3 bit set = immediate interrupt): assert IRQ
        // $D0 (no conditions): just clear IRQ (already done above)
        if (cmd & 0x08) {
            fdc_assert_interrupt();
        }
        return;
    }
    
    // For all other commands: set BUSY and motor immediately
    st->fdc.status = FDC_STATUS_BUSY | FDC_STATUS_MOTOR_ON;
    st->fdc.motor_on = true;
    st->fdc.pending_cmd = cmd;
    
    // Set delay and status_type_i based on command type
    if (cmd_type <= 0x70) {
        // Type I (RESTORE, SEEK, STEP): ~1ms = 8000 CPU cycles
        st->fdc.status_type_i = true;
        st->fdc.busy_cycles = 8000;
    } else {
        // Type II/III (READ/WRITE): ~5ms = 40000 CPU cycles
        st->fdc.status_type_i = false;
        st->fdc.busy_cycles = 40000;
    }
}

// Complete FDC command after busy delay - the actual command execution
static void fdc_complete_command(uint8_t cmd) {
    uint8_t cmd_type = cmd & 0xF0;
    
    switch (cmd_type) {
        case FDC_CMD_RESTORE:
            // Seek to track 0
            st->fdc.track = 0;
            st->fdc.status = 0x80 | 0x04; // Motor On + Track 0
            fdc_assert_interrupt();
            break;
            
        case FDC_CMD_SEEK:
            st->fdc.track = st->fdc.data;
            st->fdc.status = 0x80; // Motor On
            if (st->fdc.track == 0) st->fdc.status |= 0x04;
            fdc_assert_interrupt();
            break;
            
        case FDC_CMD_STEP:
        case FDC_CMD_STEP + 0x10: // Step-in
        case FDC_CMD_STEP_IN:
        case FDC_CMD_STEP_IN + 0x10:
            if (cmd_type >= FDC_CMD_STEP_IN) st->fdc.step_dir = 1;
            st->fdc.track += st->fdc.step_dir ? 1 : -1;
            if (st->fdc.track > 80) st->fdc.track = 80;
            st->fdc.status = 0x80 | ((st->fdc.track == 0) ? 0x04 : 0x00);
            fdc_assert_interrupt();
            break;
            
        case FDC_CMD_STEP_OUT:
        case FDC_CMD_STEP_OUT + 0x10:
            st->fdc.step_dir = 0;
            if (st->fdc.track > 0) st->fdc.track--;
            st->fdc.status = 0x80 | ((st->fdc.track == 0) ? 0x04 : 0x00);
            fdc_assert_interrupt();
            break;
            
        case FDC_CMD_READ_SECTOR: {
            // Read one sector (512 bytes) from floppy image
            int drive = st->fdc.current_drive;
            if (drive < 0 || drive > 1 || st->fdc.floppy[drive] == NULL) {
                ESP_LOGD(TAG, "FDC READ_SECTOR: no disk in drive %d", drive);
                st->fdc.status = 0x80 | FDC_STATUS_RNF; // Motor On + RNF
                fdc_assert_interrupt();
                break;
            }
            
            // Calculate sector offset in .ST image (sequential sector layout)
            // .ST format: track * sectors_per_track * sides + side * sectors_per_track + (sector - 1)
            // Standard: 80 tracks, 9 sectors/track, 2 sides
            int spt = 9; // sectors per track
            uint32_t offset = ((uint32_t)st->fdc.track * 2 + st->fdc.current_side) * spt + (st->fdc.sector - 1);
            offset *= 512;
            
            if (offset + 512 > st->fdc.floppy_size[drive]) {
                ESP_LOGW(TAG, "FDC read beyond image: track=%d sector=%d offset=%lu",
                         st->fdc.track, st->fdc.sector, (unsigned long)offset);
                st->fdc.status = 0x80 | FDC_STATUS_RNF; // Motor On + RNF
                fdc_assert_interrupt();
                break;
            }
            
            // Read data via DMA
            fseek(st->fdc.floppy[drive], offset, SEEK_SET);
            uint8_t sector_buf[512];
            size_t read = fread(sector_buf, 1, 512, st->fdc.floppy[drive]);
            
            if (read == 512) {
                // Write to M68K memory at DMA address
                for (int i = 0; i < 512; i++) {
                    m68k_write_memory_8(st->fdc.dma_addr + i, sector_buf[i]);
                }
                st->fdc.dma_addr += 512;
                st->fdc.dma_sector_count--;
                st->fdc.status = 0x80; // Motor On, no error
                
                // Auto-increment sector
                st->fdc.sector++;
                if (st->fdc.sector > spt) {
                    st->fdc.sector = 1;
                }
            } else {
                st->fdc.status = 0x80 | FDC_STATUS_RNF;
            }
            
            fdc_assert_interrupt();
            break;
        }
        
        case FDC_CMD_WRITE_SECTOR: {
            int drive = st->fdc.current_drive;
            if (drive < 0 || drive > 1 || st->fdc.floppy[drive] == NULL) {
                st->fdc.status = 0x80 | FDC_STATUS_RNF;
                fdc_assert_interrupt();
                break;
            }
            
            int spt = 9;
            uint32_t offset = ((uint32_t)st->fdc.track * 2 + st->fdc.current_side) * spt + (st->fdc.sector - 1);
            offset *= 512;
            
            // Read from M68K memory and write to disk image
            uint8_t sector_buf[512];
            for (int i = 0; i < 512; i++) {
                sector_buf[i] = m68k_read_memory_8(st->fdc.dma_addr + i);
            }
            
            fseek(st->fdc.floppy[drive], offset, SEEK_SET);
            fwrite(sector_buf, 1, 512, st->fdc.floppy[drive]);
            fflush(st->fdc.floppy[drive]);
            
            st->fdc.dma_addr += 512;
            st->fdc.dma_sector_count--;
            st->fdc.status = 0x80; // Motor On
            st->fdc.sector++;
            if (st->fdc.sector > spt) st->fdc.sector = 1;
            
            fdc_assert_interrupt();
            break;
        }
        
        case FDC_CMD_READ_ADDRESS:
            // Return current track/side/sector info
            st->fdc.status = 0x80; // Motor On
            fdc_assert_interrupt();
            break;
            
        case FDC_CMD_FORCE_INT:
            // Force Interrupt should never reach fdc_complete_command
            // (handled directly in fdc_execute_command), but handle as safety net
            st->fdc.status = st->fdc.motor_on ? FDC_STATUS_MOTOR_ON : 0x00;
            if (st->fdc.track == 0) st->fdc.status |= 0x04;
            break;
            
        default:
            ESP_LOGW(TAG, "Unhandled FDC command: 0x%02X", cmd);
            st->fdc.status = 0x80; // Motor On
            fdc_assert_interrupt();
            break;
    }
}

// FDC tick - count down busy delay, complete command when done
static void fdc_tick(int cpu_cycles) {
    if (st->fdc.busy_cycles == 0) return;
    
    if ((uint32_t)cpu_cycles >= st->fdc.busy_cycles) {
        st->fdc.busy_cycles = 0;
        fdc_complete_command(st->fdc.pending_cmd);
        st->fdc.pending_cmd = 0;
    } else {
        st->fdc.busy_cycles -= cpu_cycles;
    }
}

// ============================================================================
// IKBD (Keyboard) Buffer
// ============================================================================

static void ikbd_push(uint8_t byte) {
    if (st->ikbd.count >= 64) return; // Buffer full
    st->ikbd.buffer[st->ikbd.tail] = byte;
    st->ikbd.tail = (st->ikbd.tail + 1) & 63;
    st->ikbd.count++;
    
    // Assert ACIA IRQ on MFP GPIP4 (active low) - TOS polls this pin!
    // On real hardware: ACIA IRQ output → MFP GPIP4 pin → edge detect → IPRB bit 6
    st->mfp.gpip &= ~0x10;     // GPIP bit 4 LOW = ACIA has data
    st->mfp.iprb |= 0x40;      // IPRB bit 6 = ACIA interrupt pending
    update_irq_level();          // Re-evaluate so CPU sees it immediately
}

static uint8_t ikbd_pop(void) {
    if (st->ikbd.count == 0) return 0;
    uint8_t byte = st->ikbd.buffer[st->ikbd.head];
    st->ikbd.head = (st->ikbd.head + 1) & 63;
    st->ikbd.count--;
    
    // Update GPIP4 based on remaining data
    if (st->ikbd.count == 0) {
        // No more data → ACIA IRQ deasserted → GPIP4 goes HIGH
        st->mfp.gpip |= 0x10;
    }
    // If data still remains, GPIP4 stays LOW (set in ikbd_push)
    
    return byte;
}

static void ikbd_flush(void) {
    st->ikbd.head = 0;
    st->ikbd.tail = 0;
    st->ikbd.count = 0;
}

// IKBD tick - handle delayed responses (e.g., reset self-test takes ~63ms)
// Without this delay, the ACIA interrupt fires before TOS installs its MFP
// interrupt handlers, which can leave ISR bits stuck and block Timer C.
static void ikbd_tick(int cpu_cycles) {
    if (!st->ikbd.reset_pending) return;
    
    if ((uint32_t)cpu_cycles >= st->ikbd.reset_delay) {
        st->ikbd.reset_delay = 0;
        st->ikbd.reset_pending = false;
        ESP_LOGI(TAG, "IKBD: Sending delayed reset response $F0 $F1");
        ikbd_push(0xF0);  // Self-test OK
        ikbd_push(0xF1);  // Reset acknowledge
    } else {
        st->ikbd.reset_delay -= cpu_cycles;
    }
}

// Process a byte sent TO the IKBD (commands from TOS)
static void ikbd_process_byte(uint8_t value) {
    // If we're in the middle of a multi-byte command, consume parameter byte
    if (st->ikbd.cmd_bytes_left > 0) {
        st->ikbd.cmd_bytes_left--;
        if (st->ikbd.cmd == 0x80 && st->ikbd.cmd_bytes_left == 0) {
            // IKBD reset command complete ($80 $01)
            // Real IKBD takes ~63ms for self-test before sending response.
            // Delay is CRITICAL: if we respond immediately, the ACIA interrupt
            // fires before TOS has finished setting up MFP interrupt handlers,
            // which can leave ISR bits stuck and block Timer C (200Hz system tick).
            ESP_LOGI(TAG, "IKBD: Reset received, delaying response ~63ms");
            st->ikbd.reset_pending = true;
            st->ikbd.reset_delay = 500000;  // ~63ms at 8MHz
        }
        if (st->ikbd.cmd_bytes_left == 0) {
            st->ikbd.cmd = 0;  // Command complete
        }
        return;
    }

    // New command byte
    st->ikbd.cmd = value;

    // Determine how many parameter bytes this command expects
    switch (value) {
        case 0x80:  // Reset (1 param: $01)
            st->ikbd.cmd_bytes_left = 1;
            break;
        case 0x07:  // Set mouse button action (1 param)
            st->ikbd.cmd_bytes_left = 1;
            break;
        case 0x08:  // Set relative mouse position reporting
        case 0x09:  // Set absolute mouse positioning (4 params: Xmax_hi, Xmax_lo, Ymax_hi, Ymax_lo)
            st->ikbd.cmd_bytes_left = (value == 0x09) ? 4 : 0;
            break;
        case 0x0A:  // Set mouse keycode mode (2 params: deltaX, deltaY)
            st->ikbd.cmd_bytes_left = 2;
            break;
        case 0x0B:  // Set mouse threshold (2 params: X, Y)
            st->ikbd.cmd_bytes_left = 2;
            break;
        case 0x0C:  // Set mouse scale (2 params: X, Y)
            st->ikbd.cmd_bytes_left = 2;
            break;
        case 0x0D:  // Interrogate mouse position
            // Send mouse position report: $F7 + buttons + 0 + 0 + 0 + 0
            ikbd_push(0xF7);
            ikbd_push(0x00); // buttons
            ikbd_push(0x00); // X high
            ikbd_push(0x00); // X low
            ikbd_push(0x00); // Y high
            ikbd_push(0x00); // Y low
            st->ikbd.cmd = 0;
            break;
        case 0x0E:  // Load mouse position (5 params)
            st->ikbd.cmd_bytes_left = 5;
            break;
        case 0x0F:  // Set Y=0 at bottom
        case 0x10:  // Set Y=0 at top
        case 0x12:  // Disable mouse
        case 0x1A:  // Disable joystick
            st->ikbd.cmd = 0;  // Single byte, no response
            break;
        case 0x11:  // Set joystick event reporting
        case 0x13:  // Pause output
        case 0x14:  // Set joystick keycode mode (6 params)
            st->ikbd.cmd_bytes_left = (value == 0x14) ? 6 : 0;
            if (value != 0x14) st->ikbd.cmd = 0;
            break;
        case 0x15:  // Time-of-day clock set (6 params)
            st->ikbd.cmd_bytes_left = 6;
            break;
        case 0x16:  // Interrogate time-of-day
            // Send time: $FC + YY MM DD hh mm ss
            ikbd_push(0xFC);
            ikbd_push(26);  // 2026
            ikbd_push(1);   // January
            ikbd_push(1);   // 1st
            ikbd_push(0);   // 00:00:00
            ikbd_push(0);
            ikbd_push(0);
            st->ikbd.cmd = 0;
            break;
        case 0x17:  // Set memory (write IKBD RAM) - variable length, but rarely used
        case 0x18:  // Load memory
        case 0x19:  // Read memory
            st->ikbd.cmd_bytes_left = 0;
            st->ikbd.cmd = 0;
            break;
        case 0x1B:  // Time-of-day clock set (6 params)
            st->ikbd.cmd_bytes_left = 6;
            break;
        default:
            // Unknown/single-byte command
            ESP_LOGD(TAG, "IKBD: Unknown command 0x%02X", value);
            st->ikbd.cmd = 0;
            break;
    }
}

// PS/2 scancode to Atari ST scancode translation table
static const uint8_t ps2_to_atari_scancode[128] = {
    [0x1C] = 0x1E, // A
    [0x32] = 0x30, // B
    [0x21] = 0x2E, // C
    [0x23] = 0x20, // D
    [0x24] = 0x12, // E
    [0x2B] = 0x21, // F
    [0x34] = 0x22, // G
    [0x33] = 0x23, // H
    [0x43] = 0x17, // I
    [0x3B] = 0x24, // J
    [0x42] = 0x25, // K
    [0x4B] = 0x26, // L
    [0x3A] = 0x32, // M
    [0x31] = 0x31, // N
    [0x44] = 0x18, // O
    [0x4D] = 0x19, // P
    [0x15] = 0x10, // Q
    [0x2D] = 0x13, // R
    [0x1B] = 0x1F, // S
    [0x2C] = 0x14, // T
    [0x3C] = 0x16, // U
    [0x2A] = 0x2F, // V
    [0x1D] = 0x11, // W
    [0x22] = 0x2D, // X
    [0x35] = 0x15, // Y
    [0x1A] = 0x2C, // Z
    [0x45] = 0x0B, // 0
    [0x16] = 0x02, // 1
    [0x1E] = 0x03, // 2
    [0x26] = 0x04, // 3
    [0x25] = 0x05, // 4
    [0x2E] = 0x06, // 5
    [0x36] = 0x07, // 6
    [0x3D] = 0x08, // 7
    [0x3E] = 0x09, // 8
    [0x46] = 0x0A, // 9
    [0x29] = 0x39, // Space
    [0x5A] = 0x1C, // Enter
    [0x76] = 0x01, // Escape
    [0x66] = 0x0E, // Backspace
    [0x0D] = 0x0F, // Tab
};

// ============================================================================
// Video Rendering (Atari ST bitplane -> RGB565)
// ============================================================================

static void atari_palette_to_rgb565(int index) {
    // Atari ST palette: 0x0RGB (3 bits per channel, stored in bits 0-2)
    uint16_t st_color = st->video.palette[index] & 0x0FFF;
    
    // Extract 3-bit RGB components (ST uses bits 8-10=R, 4-6=G, 0-2=B)
    uint8_t r3 = (st_color >> 8) & 0x07;
    uint8_t g3 = (st_color >> 4) & 0x07;
    uint8_t b3 = (st_color >> 0) & 0x07;
    
    // Expand 3-bit to 5/6/5-bit for RGB565
    uint8_t r5 = (r3 << 2) | (r3 >> 1);
    uint8_t g6 = (g3 << 3) | g3;
    uint8_t b5 = (b3 << 2) | (b3 >> 1);
    
    st->video.palette_rgb565[index] = (r5 << 11) | (g6 << 5) | b5;
}

// Render low-res screen (320x200, 16 colors, 4 bitplanes)
static void render_low_res(void) {
    if (!st->fb_slice) return;
    if (!lcd_console_is_initialized()) return;
    
    uint32_t vram_base = st->video.video_base;
    
    // LCD is 480x320, ST low-res is 320x200
    // Scale: render at 1:1 with offset, or scale 1.5x
    // For simplicity, render 1:1 centered on LCD
    int x_offset = (480 - 320) / 2;  // 80 pixels offset
    int y_offset = (320 - 200) / 2;  // 60 pixels offset
    
    // Process in horizontal slices of 8 scanlines for DMA efficiency
    for (int slice = 0; slice < 25; slice++) {
        int y_start = slice * 8;
        int slice_pixels = 320 * 8;  // 320 pixels wide, 8 lines per slice
        
        // Decode bitplanes for this slice
        for (int line = 0; line < 8; line++) {
            int y = y_start + line;
            if (y >= 200) break;
            
            // Each ST scanline: 80 words (160 bytes) for low-res
            // Bitplane layout: word0_plane0, word0_plane1, word0_plane2, word0_plane3, 
            //                  word1_plane0, word1_plane1, ...
            uint32_t line_addr = vram_base + y * 160;
            
            for (int col = 0; col < 20; col++) {
                // Read 4 bitplane words for this 16-pixel group
                uint32_t addr = line_addr + col * 8;
                uint16_t plane0 = m68k_read_memory_16(addr + 0);
                uint16_t plane1 = m68k_read_memory_16(addr + 2);
                uint16_t plane2 = m68k_read_memory_16(addr + 4);
                uint16_t plane3 = m68k_read_memory_16(addr + 6);
                
                // Decode 16 pixels from bitplanes
                for (int px = 15; px >= 0; px--) {
                    int color_idx = ((plane0 >> px) & 1) |
                                    (((plane1 >> px) & 1) << 1) |
                                    (((plane2 >> px) & 1) << 2) |
                                    (((plane3 >> px) & 1) << 3);
                    
                    int x = col * 16 + (15 - px);
                    int buf_idx = line * 320 + x;
                    if (buf_idx < slice_pixels) {
                        st->fb_slice[buf_idx] = st->video.palette_rgb565[color_idx];
                    }
                }
            }
        }
        
        // DMA this slice to LCD (centered)
        int lcd_y_start = y_offset + y_start;
        int lcd_y_end = lcd_y_start + 8;
        if (lcd_y_end > 320) lcd_y_end = 320;
        
        lcd_console_draw_raw(x_offset, lcd_y_start, x_offset + 320, lcd_y_end,
                            st->fb_slice);
    }
}

// ============================================================================
// Memory Hook Functions (intercept M68K read/write for Atari hardware)
// ============================================================================

// These are called by the M68K CPU emulator for every memory access.
// We check if the address falls in the I/O or ROM range and redirect.

uint8_t atari_st_read_byte(uint32_t addr) {
    addr &= 0x00FFFFFF; // 24-bit address bus
    
    // TOS ROM
    if (addr >= st->tos_base && addr < st->tos_base + st->tos_size) {
        return st->tos_rom[addr - st->tos_base];
    }
    
    // I/O space
    if (addr >= 0xFF8000) {
        return atari_io_read_byte(addr);
    }
    
    // Cartridge ROM
    if (addr >= ATARI_CART_BASE && addr < ATARI_CART_BASE + ATARI_CART_SIZE) {
        return 0xFF; // No cartridge
    }
    
    return 0; // Let M68K handle RAM access normally
}

uint16_t atari_st_read_word(uint32_t addr) {
    addr &= 0x00FFFFFF;
    
    // TOS ROM
    if (addr >= st->tos_base && addr < st->tos_base + st->tos_size - 1) {
        uint32_t off = addr - st->tos_base;
        return (st->tos_rom[off] << 8) | st->tos_rom[off + 1];
    }
    
    // I/O space
    if (addr >= 0xFF8000) {
        return atari_io_read_word(addr);
    }
    
    return 0;
}

void atari_st_write_byte(uint32_t addr, uint8_t value) {
    addr &= 0x00FFFFFF;
    
    // I/O space
    if (addr >= 0xFF8000) {
        atari_io_write_byte(addr, value);
        return;
    }
    
    // ROM areas - ignore writes
    if (addr >= st->tos_base && addr < st->tos_base + st->tos_size) return;
    if (addr >= ATARI_CART_BASE && addr < ATARI_CART_BASE + ATARI_CART_SIZE) return;
}

void atari_st_write_word(uint32_t addr, uint16_t value) {
    addr &= 0x00FFFFFF;
    
    // I/O space
    if (addr >= 0xFF8000) {
        atari_io_write_word(addr, value);
        return;
    }
    
    // ROM areas - ignore
    if (addr >= st->tos_base && addr < st->tos_base + st->tos_size) return;
}

// ============================================================================
// M68K I/O Hook Functions (called by m68k_emulator.c memory access functions)
// ============================================================================

static uint32_t atari_st_io_read_hook(uint32_t addr, int size, bool *handled) {
    if (!st || !st->initialized) return 0;
    
    // Atari ST I/O registers: $FF8000-$FFFFFF
    if (addr >= 0xFF8000) {
        *handled = true;
        
        // MFP registers are byte-wide at ODD addresses only
        // Handle word/long accesses to MFP by splitting into bytes
        if (addr >= 0xFFFA00 && addr <= 0xFFFA3F) {
            if (size == 1) return atari_io_read_byte(addr);
            if (size == 2) {
                // Word read from odd address - read two consecutive bytes
                uint8_t b0 = atari_io_read_byte(addr);
                uint8_t b1 = atari_io_read_byte(addr + 1);
                return (b0 << 8) | b1;
            }
            // Long read - read 4 consecutive bytes
            uint32_t val = 0;
            for (int i = 0; i < 4; i++) {
                val = (val << 8) | atari_io_read_byte(addr + i);
            }
            return val;
        }
        
        // Other I/O regions - normal access
        if (size == 1) return atari_io_read_byte(addr);
        if (size == 2) return atari_io_read_word(addr);
        return (uint32_t)(atari_io_read_word(addr) << 16) | atari_io_read_word(addr + 2);
    }
    
    // TOS ROM read - let CPU read from its own memory (already copied there)
    // but intercept if address is in I/O-adjacent ROM space that might be
    // routed to bus controller instead
    if (addr >= st->tos_base && addr < st->tos_base + st->tos_size) {
        // TOS is in M68K memory, but these addresses overlap BUS_IO range
        // so we must handle them here to prevent bus_controller from eating them
        *handled = true;
        uint32_t off = addr - st->tos_base;
        if (size == 1) {
            return st->tos_rom[off];
        } else if (size == 2) {
            if (off + 1 < st->tos_size)
                return (st->tos_rom[off] << 8) | st->tos_rom[off + 1];
            return st->tos_rom[off] << 8;
        } else {
            uint32_t val = 0;
            for (int i = 0; i < 4 && (off + i) < st->tos_size; i++)
                val = (val << 8) | st->tos_rom[off + i];
            return val;
        }
    }
    
    // Cartridge ROM space
    if (addr >= ATARI_CART_BASE && addr < ATARI_CART_BASE + ATARI_CART_SIZE) {
        *handled = true;
        return 0xFF; // No cartridge inserted
    }

    // Dead zone: addresses above ST RAM but below cartridge/ROM/IO
    // On real hardware this generates a bus error; we return $FF to simulate no RAM
    if (addr >= st->config.ram_size && addr < ATARI_CART_BASE) {
        *handled = true;
        return 0xFFFFFFFF; // No hardware at this address
    }

    // Not handled - let M68K core handle normally (RAM, etc.)
    return 0;
}

static void atari_st_io_write_hook(uint32_t addr, uint32_t value, int size, bool *handled) {
    if (!st || !st->initialized) return;

    // Atari ST I/O registers: $FF8000-$FFFFFF
    if (addr >= 0xFF8000) {
        *handled = true;
        
        // MFP registers are byte-wide at ODD addresses only
        // Handle word/long accesses to MFP by splitting into bytes
        if (addr >= 0xFFFA00 && addr <= 0xFFFA3F) {
            if (size == 1) { atari_io_write_byte(addr, value & 0xFF); return; }
            if (size == 2) {
                // Word write to odd address - write two consecutive bytes
                atari_io_write_byte(addr, (value >> 8) & 0xFF);
                atari_io_write_byte(addr + 1, value & 0xFF);
                return;
            }
            // Long write - write 4 consecutive bytes
            atari_io_write_byte(addr,     (value >> 24) & 0xFF);
            atari_io_write_byte(addr + 1, (value >> 16) & 0xFF);
            atari_io_write_byte(addr + 2, (value >> 8)  & 0xFF);
            atari_io_write_byte(addr + 3, value & 0xFF);
            return;
        }
        
        // Other I/O regions - normal access
        if (size == 1) { atari_io_write_byte(addr, value & 0xFF); return; }
        if (size == 2) { atari_io_write_word(addr, value & 0xFFFF); return; }
        atari_io_write_word(addr, (value >> 16) & 0xFFFF);
        atari_io_write_word(addr + 2, value & 0xFFFF);
        return;
    }
    
    // TOS ROM write protection - silently discard
    if (addr >= st->tos_base && addr < st->tos_base + st->tos_size) {
        *handled = true;
        return;
    }
    
    // Cartridge ROM write protection
    if (addr >= ATARI_CART_BASE && addr < ATARI_CART_BASE + ATARI_CART_SIZE) {
        *handled = true;
        return;
    }

    // Dead zone: above ST RAM, below cartridge/ROM/IO - discard writes
    if (addr >= st->config.ram_size && addr < ATARI_CART_BASE) {
        *handled = true;
        return;
    }

    // Not handled - let M68K core handle (RAM writes, etc.)
}

// ============================================================================
// Initialization
// ============================================================================

esp_err_t atari_st_init(const atari_st_config_t *config) {
    if (st != NULL) {
        ESP_LOGW(TAG, "Atari ST already initialized, destroying first");
        atari_st_destroy();
    }
    
    ESP_LOGI(TAG, "Initializing Atari ST emulator...");
    
    // Allocate state
    st = heap_caps_calloc(1, sizeof(atari_st_state_t), MALLOC_CAP_DEFAULT);
    if (!st) {
        ESP_LOGE(TAG, "Failed to allocate Atari ST state");
        return ESP_ERR_NO_MEM;
    }
    
    memcpy(&st->config, config, sizeof(atari_st_config_t));
    if (st->config.ram_size == 0) st->config.ram_size = ATARI_RAM_SIZE;
    
    // Initialize M68K CPU core
    esp_err_t ret = m68k_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init M68K CPU: %s", esp_err_to_name(ret));
        free(st);
        st = NULL;
        return ret;
    }
    
    // Load TOS ROM
    if (!config->tos_path) {
        ESP_LOGE(TAG, "No TOS ROM path specified");
        m68k_destroy();
        free(st);
        st = NULL;
        return ESP_ERR_INVALID_ARG;
    }
    
    FILE *tos_file = fopen(config->tos_path, "rb");
    if (!tos_file) {
        ESP_LOGE(TAG, "Cannot open TOS ROM: %s", config->tos_path);
        m68k_destroy();
        free(st);
        st = NULL;
        return ESP_ERR_NOT_FOUND;
    }
    
    fseek(tos_file, 0, SEEK_END);
    st->tos_size = ftell(tos_file);
    fseek(tos_file, 0, SEEK_SET);
    
    ESP_LOGI(TAG, "TOS ROM: %s (%lu bytes)", config->tos_path, (unsigned long)st->tos_size);
    
    // Determine TOS base address from size
    if (st->tos_size <= 192 * 1024) {
        st->tos_base = ATARI_TOS_BASE;  // $FC0000 for TOS 1.x
    } else {
        st->tos_base = ATARI_TOS2_BASE; // $E00000 for TOS 2.x+
    }
    
    // Allocate and load TOS ROM into local buffer
    st->tos_rom = heap_caps_malloc(st->tos_size, MALLOC_CAP_SPIRAM);
    if (!st->tos_rom) {
        ESP_LOGE(TAG, "Failed to allocate TOS ROM buffer (%lu bytes)", (unsigned long)st->tos_size);
        fclose(tos_file);
        m68k_destroy();
        free(st);
        st = NULL;
        return ESP_ERR_NO_MEM;
    }
    
    size_t read = fread(st->tos_rom, 1, st->tos_size, tos_file);
    fclose(tos_file);
    
    if (read != st->tos_size) {
        ESP_LOGE(TAG, "TOS ROM read error: got %d of %lu bytes", (int)read, (unsigned long)st->tos_size);
        heap_caps_free(st->tos_rom);
        m68k_destroy();
        free(st);
        st = NULL;
        return ESP_ERR_INVALID_SIZE;
    }
    
    ESP_LOGI(TAG, "TOS ROM loaded at $%06lX-%06lX", 
             (unsigned long)st->tos_base, 
             (unsigned long)(st->tos_base + st->tos_size - 1));
    
    // Copy TOS ROM into M68K memory space so CPU can read it directly.
    // The initial SSP comes from the GLUE chip (top of RAM), NOT from the ROM.
    // The ROM's first 4 bytes may contain code (e.g. BRA.S), not a valid SSP.
    uint32_t initial_pc  = (st->tos_rom[4] << 24) | (st->tos_rom[5] << 16) |
                           (st->tos_rom[6] << 8) | st->tos_rom[7];
    uint32_t rom_ssp_raw = (st->tos_rom[0] << 24) | (st->tos_rom[1] << 16) |
                           (st->tos_rom[2] << 8) | st->tos_rom[3];

    ESP_LOGI(TAG, "TOS entry: PC=$%08lX (ROM SSP=$%08lX, using GLUE SSP=$%08lX)",
             (unsigned long)initial_pc, (unsigned long)rom_ssp_raw,
             (unsigned long)st->config.ram_size);

    // Write proper reset vectors to $000000:
    //   SSP = top of RAM (provided by GLUE chip on real hardware)
    //   PC  = from TOS ROM offset 4-7
    uint32_t glue_ssp = st->config.ram_size;
    m68k_write_memory_8(0, (glue_ssp >> 24) & 0xFF);
    m68k_write_memory_8(1, (glue_ssp >> 16) & 0xFF);
    m68k_write_memory_8(2, (glue_ssp >> 8)  & 0xFF);
    m68k_write_memory_8(3,  glue_ssp        & 0xFF);
    for (int i = 4; i < 8; i++) {
        m68k_write_memory_8(i, st->tos_rom[i]);
    }
    
    // Copy the entire TOS ROM into M68K address space
    // This lets the CPU core read it directly without hooks
    for (uint32_t i = 0; i < st->tos_size; i++) {
        m68k_write_memory_8(st->tos_base + i, st->tos_rom[i]);
    }
    
    // Allocate LCD frame buffer slice (320 * 8 pixels * 2 bytes = 5120 bytes)
    if (config->enable_lcd_output) {
        st->fb_slice = heap_caps_malloc(320 * 8 * sizeof(uint16_t), MALLOC_CAP_DMA);
        if (!st->fb_slice) {
            ESP_LOGW(TAG, "Failed to allocate LCD frame buffer, LCD output disabled");
        }
    }
    
    // Initialize hardware state
    // MFP: Timer C default = 192 at prescale 64 → 200 Hz system tick
    st->mfp.vr = 0x40;  // Vector base $40
    st->mfp.timer_c.data = 192;
    st->mfp.timer_c.counter = 192;
    st->mfp.timer_c.control = 5;  // Prescale /64
    st->mfp.iera = 0x00;
    st->mfp.ierb = 0x20;  // Timer C enabled
    st->mfp.imra = 0x00;
    st->mfp.imrb = 0x20;
    st->mfp.gpip = 0xFF;  // Color monitor: bit 7=1 (mono_detect HIGH=color)
    // MiSTeryNano reference: GPIP = {mono_detect, 1, !fdc_irq, !acia_irq, ...}
    // mono_detect: 0=monochrome, 1=color. All other signals inactive (HIGH).
    st->mfp.tsr = 0x80;   // Transmitter buffer empty
    
    // ACIA default state
    st->kbd_acia.status = 0x02;  // TX empty
    st->midi_acia.status = 0x02;
    
    // YM2149 default: all drives deselected, side 0
    st->ym.registers[14] = 0xFF;
    
    // Video: default to low-res, 60Hz, video base at $78000
    st->video.resolution = ATARI_RES_LOW;
    st->video.sync_mode = 0;  // 60Hz
    st->video.video_base = 0x78000;
    
    // Default ST palette (TOS will set this, but provide fallback)
    st->video.palette[0]  = 0x0FFF; // White (background)
    st->video.palette[1]  = 0x0F00; // Red
    st->video.palette[2]  = 0x00F0; // Green
    st->video.palette[3]  = 0x0000; // Black (text)
    st->video.palette[15] = 0x0000; // Black
    for (int i = 0; i < 16; i++) {
        atari_palette_to_rgb565(i);
    }
    
    // Timing: 8 MHz CPU, 50Hz VBL (PAL) or 60Hz (NTSC)
    st->vbl_cycles = 160256;  // ~50 Hz
    st->hbl_cycles = 512;
    
    // FDC: no disk inserted by default
    st->fdc.status = 0x00;
    st->fdc.current_drive = 0;
    st->fdc.current_side = 0;
    
    // Mount floppy A if specified
    if (config->floppy_a_path) {
        atari_st_insert_floppy(0, config->floppy_a_path);
    }
    if (config->floppy_b_path) {
        atari_st_insert_floppy(1, config->floppy_b_path);
    }
    
    st->initialized = true;
    
    // Register M68K I/O hooks so CPU memory accesses route to our handlers
    m68k_set_io_hooks(atari_st_io_read_hook, atari_st_io_write_hook);
    m68k_set_irq_ack_hook(atari_irq_ack);
    
    // Disable watchdog and exception recovery - TOS initialization does long
    // sequences of CPU-only work that can falsely trigger the watchdog, and
    // exception recovery interferes with TOS's own exception handling
    m68k_set_exception_recovery(false);
    m68k_watchdog_enable(false);
    
    ESP_LOGI(TAG, "Atari ST emulator initialized");
    ESP_LOGI(TAG, "  RAM:  %lu KB", (unsigned long)(st->config.ram_size / 1024));
    ESP_LOGI(TAG, "  TOS:  $%06lX (%lu KB)", (unsigned long)st->tos_base, (unsigned long)(st->tos_size / 1024));
    ESP_LOGI(TAG, "  Video: %s", st->video.resolution == 0 ? "320x200x16" : "640x200x4");
    ESP_LOGI(TAG, "  LCD:  %s", st->fb_slice ? "enabled" : "disabled");
    
    return ESP_OK;
}

// ============================================================================
// Reset
// ============================================================================

void atari_st_reset(void) {
    if (!st || !st->initialized) return;
    
    ESP_LOGI(TAG, "Atari ST cold reset");

    // On real Atari ST hardware, the GLUE chip provides the initial SSP during
    // reset - it does NOT come from the ROM. The ROM's first 4 bytes may contain
    // executable code (e.g. BRA.S) rather than a valid SSP.
    // We write proper reset vectors to $000000:
    //   $000000-$000003: SSP = top of configured RAM (from GLUE chip)
    //   $000004-$000007: PC = from TOS ROM offset 4-7
    uint32_t initial_ssp = st->config.ram_size;  // e.g. $100000 for 1MB
    m68k_write_memory_8(0, (initial_ssp >> 24) & 0xFF);
    m68k_write_memory_8(1, (initial_ssp >> 16) & 0xFF);
    m68k_write_memory_8(2, (initial_ssp >> 8)  & 0xFF);
    m68k_write_memory_8(3,  initial_ssp        & 0xFF);
    // Copy PC from TOS ROM (offset 4-7)
    for (int i = 4; i < 8; i++) {
        m68k_write_memory_8(i, st->tos_rom[i]);
    }

    // Reset M68K CPU - will read SSP from $000000 and PC from $000004
    m68k_reset();
    
    // Reset hardware
    st->mfp.ipra = 0;
    st->mfp.iprb = 0;
    st->mfp.isra = 0;
    st->mfp.isrb = 0;
    st->mfp_cycles = 0;
    st->cycles = 0;
    st->scanline = 0;
    st->vbl_pending = false;
    st->hbl_pending = false;
    st->pending_mfp_vector = 0;
    st->active_irq = 0;
    
    // IKBD: clear buffer and command state; TOS will send $80 $01 reset
    // command itself, and ikbd_process_byte() will queue the $F0 $F1 response
    ikbd_flush();
    st->ikbd.cmd = 0;
    st->ikbd.cmd_bytes_left = 0;

    st->stopped = false;
}

// ============================================================================
// Main Emulation Loop
// ============================================================================

void atari_st_run(void) {
    if (!st || !st->initialized) return;
    
    st->running = true;
    st->stopped = false;
    
    atari_st_reset();
    
    ESP_LOGI(TAG, "Atari ST emulation starting...");
    
    uint32_t vbl_counter = 0;
    uint32_t cycles_this_vbl = 0;
    uint32_t render_timer = 0;
    int64_t last_time = esp_timer_get_time();
    
    while (st->running && !st->stopped) {
        // Execute a batch of M68K instructions
        // ~100 instructions per batch gives good responsiveness
        int batch_cycles = 0;
        bool cpu_idle = false;
        
        for (int i = 0; i < 100 && st->running; i++) {
            if (m68k_is_halted()) {
                // CPU is stopped (STOP instruction) or halted.
                // CRITICAL: We must still advance time so MFP timers tick
                // and can generate interrupts to wake the CPU from STOP.
                // On real hardware, MFP runs on its own clock even when CPU is stopped.
                cpu_idle = true;
                batch_cycles += 4;
                continue;
            }
            
            m68k_step();
            batch_cycles += 4; // Average 4 cycles per instruction (rough approximation)
        }
        
        // If CPU was idle for the whole batch, yield to avoid busy-looping
        if (cpu_idle && m68k_is_halted()) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        
        st->cycles += batch_cycles;
        cycles_this_vbl += batch_cycles;
        
        // Tick MFP timers
        mfp_tick(batch_cycles);
        
        // Tick FDC busy countdown
        fdc_tick(batch_cycles);
        
        // Tick IKBD delayed responses (e.g., reset self-test)
        ikbd_tick(batch_cycles);
        
        // Check for VBL (vertical blank)
        if (cycles_this_vbl >= st->vbl_cycles) {
            cycles_this_vbl -= st->vbl_cycles;
            vbl_counter++;
            
            // Generate VBL interrupt (auto-vector level 4)
            st->vbl_pending = true;
            update_irq_level();
            
            // Diagnostic: log MFP/ACIA state periodically (every ~1 second = 50 VBLs)
            if (vbl_counter <= 5 || (vbl_counter % 50) == 0) {
                uint32_t pc = m68k_get_reg(M68K_REG_PC);
                uint32_t sr = m68k_get_reg(M68K_REG_SR);
                ESP_LOGI(TAG, "VBL#%lu PC=$%06lX SR=$%04lX irq=%d | MFP: IERA=%02X IERB=%02X IPRA=%02X IPRB=%02X IMRA=%02X IMRB=%02X ISRA=%02X ISRB=%02X VR=%02X | TimC: ctrl=%d data=%d cnt=%d | IKBD: %d bytes",
                    (unsigned long)vbl_counter,
                    (unsigned long)pc, (unsigned long)sr, st->active_irq,
                    st->mfp.iera, st->mfp.ierb, st->mfp.ipra, st->mfp.iprb,
                    st->mfp.imra, st->mfp.imrb, st->mfp.isra, st->mfp.isrb, st->mfp.vr,
                    st->mfp.timer_c.control, st->mfp.timer_c.data, st->mfp.timer_c.counter,
                    st->ikbd.count);
                ESP_LOGI(TAG, "  FDC: cmd=%02X status=%02X track=%d sector=%d motor=%d busy=%lu | DMA: mode=%04X addr=%06lX cnt=%d | Video: base=$%06lX res=%d | GPIP=%02X AER=%02X DDR=%02X",
                    st->fdc.command, st->fdc.status, st->fdc.track, st->fdc.sector,
                    st->fdc.motor_on, (unsigned long)st->fdc.busy_cycles,
                    st->fdc.dma_mode, (unsigned long)st->fdc.dma_addr, st->fdc.dma_sector_count,
                    (unsigned long)st->video.video_base, st->video.resolution,
                    st->mfp.gpip, st->mfp.aer, st->mfp.ddr);
                // Display diagnostics: palette and video RAM sample
                uint32_t vb = st->video.video_base;
                ESP_LOGI(TAG, "  Palette[0]=$%03X [1]=$%03X [3]=$%03X [15]=$%03X | VRAM@$%06lX: %04X %04X %04X %04X",
                    st->video.palette[0] & 0xFFF, st->video.palette[1] & 0xFFF,
                    st->video.palette[3] & 0xFFF, st->video.palette[15] & 0xFFF,
                    (unsigned long)vb,
                    m68k_read_memory_16(vb), m68k_read_memory_16(vb + 2),
                    m68k_read_memory_16(vb + 4), m68k_read_memory_16(vb + 6));
                st->io_read_count = 0;  // Reset per diagnostic interval
            }
            
            // Render screen at ~20 fps (every 2-3 VBLs for 50Hz)
            render_timer++;
            if (render_timer >= 3 && st->fb_slice) {
                render_timer = 0;
                render_low_res();
            }
            
            // Check PS/2 keyboard input
            if (ps2_keyboard_is_initialized()) {
                while (ps2_keyboard_available()) {
                    uint8_t key = ps2_keyboard_read();
                    // Simple ASCII to Atari scancode conversion
                    if (key < 128 && ps2_to_atari_scancode[key]) {
                        ikbd_push(ps2_to_atari_scancode[key]);      // Key press
                        ikbd_push(ps2_to_atari_scancode[key] | 0x80); // Key release
                    }
                }
            }
            
            // Check USB keyboard input
            if (usb_keyboard_is_initialized()) {
                while (usb_keyboard_available()) {
                    uint8_t key = usb_keyboard_read();
                    if (key < 128 && ps2_to_atari_scancode[key]) {
                        ikbd_push(ps2_to_atari_scancode[key]);      // Key press
                        ikbd_push(ps2_to_atari_scancode[key] | 0x80); // Key release
                    }
                }
            }
            
            // Check USB mouse input - send IKBD relative mouse packets
            if (usb_mouse_is_connected()) {
                usb_mouse_report_t mouse_report;
                while (usb_mouse_read(&mouse_report)) {
                    // IKBD relative mouse report format:
                    // Byte 0: 0xF8 | (button_state & 0x03)
                    //   bit0 = right button, bit1 = left button (Atari swaps from USB)
                    // Byte 1: X displacement (signed byte)
                    // Byte 2: Y displacement (signed byte)
                    uint8_t buttons = 0;
                    if (mouse_report.buttons & 0x01) buttons |= 0x02; // USB left -> Atari left (bit1)
                    if (mouse_report.buttons & 0x02) buttons |= 0x01; // USB right -> Atari right (bit0)
                    
                    ikbd_push(0xF8 | buttons);
                    ikbd_push((uint8_t)mouse_report.dx);
                    ikbd_push((uint8_t)mouse_report.dy);
                }
            }
            
            // Check UART input for keyboard
            uint8_t uart_data;
            while (uart_read_bytes(0, &uart_data, 1, 0) > 0) {
                if (uart_data == 0x03) { // Ctrl+C
                    ESP_LOGI(TAG, "Ctrl+C detected, stopping Atari ST");
                    st->running = false;
                    break;
                }
                // Map common ASCII to Atari scancodes
                uint8_t sc = 0;
                if (uart_data >= 'a' && uart_data <= 'z') {
                    // Simple letter mapping
                    static const uint8_t letter_sc[] = {
                        0x1E,0x30,0x2E,0x20,0x12,0x21,0x22,0x23,0x17,0x24,
                        0x25,0x26,0x32,0x31,0x18,0x19,0x10,0x13,0x1F,0x14,
                        0x16,0x2F,0x11,0x2D,0x15,0x2C
                    };
                    sc = letter_sc[uart_data - 'a'];
                } else if (uart_data >= '0' && uart_data <= '9') {
                    static const uint8_t digit_sc[] = {
                        0x0B,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A
                    };
                    sc = digit_sc[uart_data - '0'];
                } else if (uart_data == '\r' || uart_data == '\n') {
                    sc = 0x1C; // Return
                } else if (uart_data == ' ') {
                    sc = 0x39; // Space
                } else if (uart_data == 0x1B) {
                    sc = 0x01; // Escape
                } else if (uart_data == 0x08 || uart_data == 0x7F) {
                    sc = 0x0E; // Backspace
                }
                
                if (sc) {
                    ikbd_push(sc);           // Key press
                    ikbd_push(sc | 0x80);    // Key release
                }
            }
            
            // Yield to other FreeRTOS tasks occasionally  
            int64_t now = esp_timer_get_time();
            if (now - last_time < 20000) { // Less than 20ms per VBL frame
                vTaskDelay(pdMS_TO_TICKS(1));
            }
            last_time = now;
        }
    }
    
    st->running = false;
    ESP_LOGI(TAG, "Atari ST emulation stopped after %llu cycles, %lu VBLs",
             st->cycles, (unsigned long)vbl_counter);
}

void atari_st_stop(void) {
    if (st) {
        st->running = false;
        st->stopped = true;
    }
}

bool atari_st_is_running(void) {
    return st && st->running;
}

// ============================================================================
// Floppy Disk Management
// ============================================================================

esp_err_t atari_st_insert_floppy(int drive, const char *path) {
    if (!st || drive < 0 || drive > 1) return ESP_ERR_INVALID_ARG;
    
    // Eject current disk
    atari_st_eject_floppy(drive);
    
    FILE *f = fopen(path, "r+b"); // Read-write
    if (!f) {
        f = fopen(path, "rb"); // Read-only fallback
    }
    if (!f) {
        ESP_LOGE(TAG, "Cannot open floppy image: %s", path);
        return ESP_ERR_NOT_FOUND;
    }
    
    fseek(f, 0, SEEK_END);
    st->fdc.floppy_size[drive] = ftell(f);
    fseek(f, 0, SEEK_SET);
    
    st->fdc.floppy[drive] = f;
    strncpy(st->fdc.floppy_path[drive], path, sizeof(st->fdc.floppy_path[drive]) - 1);
    
    ESP_LOGI(TAG, "Floppy %c: inserted: %s (%lu bytes)",
             'A' + drive, path, (unsigned long)st->fdc.floppy_size[drive]);
    
    return ESP_OK;
}

void atari_st_eject_floppy(int drive) {
    if (!st || drive < 0 || drive > 1) return;
    
    if (st->fdc.floppy[drive]) {
        fclose(st->fdc.floppy[drive]);
        st->fdc.floppy[drive] = NULL;
        st->fdc.floppy_size[drive] = 0;
        st->fdc.floppy_path[drive][0] = '\0';
        ESP_LOGI(TAG, "Floppy %c: ejected", 'A' + drive);
    }
}

// ============================================================================
// Keyboard Input
// ============================================================================

void atari_st_key_press(uint8_t scancode) {
    if (st) ikbd_push(scancode);
}

void atari_st_key_release(uint8_t scancode) {
    if (st) ikbd_push(scancode | 0x80);
}

// ============================================================================
// Screen Rendering (public)
// ============================================================================

void atari_st_render_screen(void) {
    if (!st || !st->initialized) return;
    
    switch (st->video.resolution) {
        case ATARI_RES_LOW:
            render_low_res();
            break;
        case ATARI_RES_MED:
        case ATARI_RES_HIGH:
            // TODO: implement medium and high-res rendering
            render_low_res(); // Fallback
            break;
    }
}

// ============================================================================
// State Query
// ============================================================================

void atari_st_get_state(char *buf, size_t buf_size) {
    if (!st || !st->initialized) {
        snprintf(buf, buf_size, "Atari ST: not initialized\n");
        return;
    }
    
    char state_buf[512];
    m68k_get_state(state_buf, sizeof(state_buf));
    
    snprintf(buf, buf_size,
        "=== Atari ST ===\n"
        "Cycles: %llu\n"
        "TOS: $%06lX (%lu KB)\n"
        "Video: %s base=$%06lX\n"
        "Floppy A: %s\n"
        "Floppy B: %s\n"
        "MFP: IERA=%02X IERB=%02X IPRA=%02X IPRB=%02X\n"
        "YM Reg14(port A)=%02X\n"
        "IKBD buffer: %d bytes\n"
        "\n%s",
        st->cycles,
        (unsigned long)st->tos_base, (unsigned long)(st->tos_size / 1024),
        st->video.resolution == 0 ? "320x200x16" :
        st->video.resolution == 1 ? "640x200x4" : "640x400x2",
        (unsigned long)st->video.video_base,
        st->fdc.floppy[0] ? st->fdc.floppy_path[0] : "(empty)",
        st->fdc.floppy[1] ? st->fdc.floppy_path[1] : "(empty)",
        st->mfp.iera, st->mfp.ierb, st->mfp.ipra, st->mfp.iprb,
        st->ym.registers[14],
        st->ikbd.count,
        state_buf
    );
}

// ============================================================================
// Cleanup
// ============================================================================

void atari_st_destroy(void) {
    if (!st) return;
    
    ESP_LOGI(TAG, "Destroying Atari ST emulator");
    
    // Clear M68K I/O hooks and IRQ first
    m68k_set_io_hooks(NULL, NULL);
    m68k_set_irq_ack_hook(NULL);
    m68k_set_irq(0);
    
    st->running = false;
    
    // Close floppy images
    atari_st_eject_floppy(0);
    atari_st_eject_floppy(1);
    
    // Free TOS ROM
    if (st->tos_rom) {
        heap_caps_free(st->tos_rom);
    }
    
    // Free frame buffer
    if (st->fb_slice) {
        heap_caps_free(st->fb_slice);
    }
    
    // Destroy M68K CPU
    m68k_destroy();
    
    free(st);
    st = NULL;
    
    ESP_LOGI(TAG, "Atari ST emulator destroyed");
}
