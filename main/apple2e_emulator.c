/*
 * Apple IIe Emulator Integration Layer
 * 6502-based Apple IIe with 64KB main + 64KB aux RAM,
 * text/lores/hires/double-hires graphics, keyboard, speaker,
 * and Disk II controller emulation.
 *
 * Memory Map:
 *   $0000-$00FF  Zero page
 *   $0100-$01FF  Stack
 *   $0200-$03FF  System use
 *   $0400-$07FF  Text/LoRes page 1 (1KB)
 *   $0800-$0BFF  Text/LoRes page 2 (1KB)
 *   $0C00-$1FFF  Free RAM
 *   $2000-$3FFF  HiRes page 1 (8KB)
 *   $4000-$5FFF  HiRes page 2 (8KB)
 *   $6000-$BFFF  Free RAM
 *   $C000-$C0FF  I/O (soft switches)
 *   $C100-$C7FF  Slot ROM space
 *   $C800-$CFFF  Slot expansion ROM
 *   $D000-$FFFF  ROM (12KB: Applesoft BASIC + Monitor)
 *
 * Display: 40x24 text, 40x48 LoRes, 280x192 HiRes, 560x192 DHR (IIe)
 */

#include "apple2e_emulator.h"
#include "mos6502_cpu.h"
#include "video_card.h"
#include "ps2_keyboard.h"
#include "usb_keyboard.h"
#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "driver/uart.h"

static const char *TAG = "Apple2e";

extern int console_printf(const char *fmt, ...);

// ============================================================================
// Apple II color palette (RGB565) - standard 16 colors
// ============================================================================
static const uint16_t apple2_colors_rgb565[16] = {
    0x0000,  //  0 - Black
    0xD00A,  //  1 - Magenta/Deep Red
    0x0015,  //  2 - Dark Blue
    0xD81F,  //  3 - Purple/Violet
    0x0340,  //  4 - Dark Green
    0x528A,  //  5 - Grey 1 (Dark)
    0x055F,  //  6 - Medium Blue
    0xA5DF,  //  7 - Light Blue
    0x5200,  //  8 - Brown
    0xFBC0,  //  9 - Orange
    0xAD55,  // 10 - Grey 2 (Light)
    0xFCD3,  // 11 - Pink
    0x0700,  // 12 - Green (LoRes)
    0xFFE0,  // 13 - Yellow
    0x57EA,  // 14 - Aqua/Cyan
    0xFFFF   // 15 - White
};

// Apple II character ROM - built-in 7x8 font (ASCII subset)
// Standard Apple II character set: inverse, flash, normal
// We embed a minimal 96-char normal set based on ASCII
#include "apple2e_font.inc"

// ============================================================================
// Disk II controller state
// ============================================================================
typedef struct {
    FILE    *image_file;        // Disk image file handle
    uint8_t *track_data;        // Current track data buffer (raw nibbles or sectors)
    int      current_track;     // 0-34 (half-tracks: 0-69)
    int      current_phase;     // Stepper motor phase
    int      head_position;     // Byte position within track
    int      track_size;        // Bytes in current track
    bool     motor_on;          // Drive motor
    bool     write_mode;        // Read/write mode
    bool     mounted;           // Disk inserted
    uint32_t image_size;        // Total image file size
    uint8_t  data_latch;        // Data register
} disk_drive_t;

typedef struct {
    disk_drive_t drive[2];      // Two drives per controller
    int  active_drive;          // 0 or 1
    uint8_t rom[256];           // Disk II controller ROM ($C600-$C6FF)
    bool rom_loaded;
} disk_controller_t;

// ============================================================================
// Emulator state
// ============================================================================
static struct {
    mos6502_t cpu;
    uint8_t  *main_ram;         // 64KB main memory
    uint8_t  *aux_ram;          // 64KB auxiliary memory (IIe)
    uint8_t  *rom;              // 12KB ROM ($D000-$FFFF) - loaded from file
    bool      initialized;

    // Soft switch states
    bool      text_mode;        // TEXT vs graphics
    bool      mixed_mode;       // Mixed text+graphics (4 lines of text at bottom)
    bool      page2;            // Display page 2 active
    bool      hires;            // HiRes vs LoRes
    bool      col80;            // 80-column mode (IIe)
    bool      store80;          // 80STORE soft switch
    bool      ramrd;            // Read from aux memory
    bool      ramwrt;           // Write to aux memory
    bool      altzp;            // Aux zero page/stack
    bool      intcxrom;         // Internal slot ROM
    bool      slotc3rom;        // Slot 3 ROM
    bool      dhires;           // Double hi-res mode

    // Keyboard
    uint8_t   key_data;         // Last key pressed (bit 7 = strobe)
    bool      key_strobe;       // Key available

    // Speaker
    bool      speaker_state;    // Speaker output level

    // Disk II
    disk_controller_t disk;

    // Screen rendering
    uint8_t   prev_screen[960]; // 40x24 text tracking for diff
    uint32_t  frame_count;

    // Language card (bank-switched RAM at $D000-$FFFF)
    bool      lc_ram_read;      // Read from LC RAM (vs ROM)
    bool      lc_ram_write;     // Write to LC RAM
    bool      lc_bank2;         // Bank 2 ($D000-$DFFF)
    bool      lc_prewrite;      // Pre-write state

    // Exit flag
    bool      exit_requested;
} s_a2;

// ============================================================================
// HiRes line address lookup table
// Apple II HiRes memory layout is interleaved (not sequential)
// ============================================================================
static uint16_t hires_line_addr[192];

static void build_hires_table(void) {
    for (int y = 0; y < 192; y++) {
        int group = y / 64;          // 0-2 (thirds)
        int row_in_group = y % 8;    // 0-7 (character row)
        int line_in_row = (y / 8) % 8; // 0-7 (scan line)
        hires_line_addr[y] = (group * 0x28) + (row_in_group * 0x80) + (line_in_row * 0x400);
    }
}

// ============================================================================
// Text line address lookup table (same interleaving as HiRes, but 40 cols)
// ============================================================================
static uint16_t text_line_addr[24];

static void build_text_table(void) {
    for (int row = 0; row < 24; row++) {
        int group = row / 8;
        int line_in_group = row % 8;
        text_line_addr[row] = (group * 0x28) + (line_in_group * 0x80);
    }
}

// ============================================================================
// Disk II emulation
// ============================================================================

// Standard DOS 3.3 sector interleave
__attribute__((unused))
static const int dos33_interleave[16] = {
    0, 7, 14, 6, 13, 5, 12, 4, 11, 3, 10, 2, 9, 1, 8, 15
};

static void disk_load_track(disk_drive_t *drv) {
    if (!drv->mounted || !drv->image_file) return;

    int track = drv->current_track / 2; // Convert half-track to track
    if (track > 34) track = 34;

    // For .dsk format: 16 sectors * 256 bytes = 4096 bytes per track
    int track_offset = track * 16 * 256;
    if (!drv->track_data) {
        drv->track_data = malloc(16 * 256);
        if (!drv->track_data) return;
    }

    fseek(drv->image_file, track_offset, SEEK_SET);
    size_t read = fread(drv->track_data, 1, 16 * 256, drv->image_file);
    drv->track_size = (int)read;
    drv->head_position = 0;
}

static uint8_t disk_read_byte(disk_drive_t *drv) {
    if (!drv->mounted || !drv->track_data || drv->track_size == 0) {
        return 0x00;
    }
    uint8_t val = drv->track_data[drv->head_position % drv->track_size];
    drv->head_position = (drv->head_position + 1) % drv->track_size;
    return val;
}

// Disk II I/O at $C0E0-$C0EF (slot 6)
static uint8_t disk_io_read(uint8_t reg) {
    disk_controller_t *dc = &s_a2.disk;
    disk_drive_t *drv = &dc->drive[dc->active_drive];

    switch (reg & 0x0F) {
    case 0x0: case 0x1:  // Phase 0
    case 0x2: case 0x3:  // Phase 1
    case 0x4: case 0x5:  // Phase 2
    case 0x6: case 0x7:  // Phase 3
    {
        // Stepper motor phase control
        int phase = (reg >> 1) & 0x03;
        bool on = (reg & 0x01) != 0;
        if (on) {
            int delta = phase - drv->current_phase;
            if (delta == 1 || delta == -3) {
                if (drv->current_track < 69) drv->current_track++;
            } else if (delta == -1 || delta == 3) {
                if (drv->current_track > 0) drv->current_track--;
            }
            drv->current_phase = phase;
            disk_load_track(drv);
        }
        return 0;
    }
    case 0x8:  // Motor off
        drv->motor_on = false;
        return 0;
    case 0x9:  // Motor on
        drv->motor_on = true;
        return 0;
    case 0xA:  // Select drive 1
        dc->active_drive = 0;
        return 0;
    case 0xB:  // Select drive 2
        dc->active_drive = 1;
        return 0;
    case 0xC:  // Q6L - Read
        if (!drv->write_mode) {
            return disk_read_byte(drv);
        }
        return drv->data_latch;
    case 0xD:  // Q6H - Write protect sense
        return 0x00; // Not write-protected
    case 0xE:  // Q7L - Read mode
        drv->write_mode = false;
        return drv->data_latch;
    case 0xF:  // Q7H - Write mode
        drv->write_mode = true;
        return 0;
    }
    return 0;
}

static void disk_io_write(uint8_t reg, uint8_t val) {
    disk_controller_t *dc = &s_a2.disk;
    disk_drive_t *drv = &dc->drive[dc->active_drive];

    // Most disk accesses are reads, writes to data latch
    if ((reg & 0x0F) == 0x0D) {
        drv->data_latch = val;
    } else {
        disk_io_read(reg); // Most operations are same for read/write
    }
}

// ============================================================================
// Language Card (bank-switched RAM at $D000-$FFFF)
// ============================================================================
static uint8_t lc_io_read(uint16_t addr) {
    int sw = (addr - 0xC080) & 0x0F;
    bool write_flag = (sw & 0x01) != 0;
    bool bank2 = (sw & 0x08) != 0;
    bool read_ram = ((sw & 0x03) == 0x03) || ((sw & 0x03) == 0x00);

    if (write_flag && s_a2.lc_prewrite) {
        s_a2.lc_ram_write = true;
    } else if (write_flag) {
        s_a2.lc_prewrite = true;
    } else {
        s_a2.lc_prewrite = false;
        s_a2.lc_ram_write = false;
    }

    s_a2.lc_ram_read = read_ram;
    s_a2.lc_bank2 = bank2;

    return 0; // Floating bus
}

// ============================================================================
// Memory read/write with soft switch handling
// ============================================================================

static uint8_t mem_read(void *ctx, uint16_t addr) {
    (void)ctx;

    // I/O space ($C000-$CFFF)
    if (addr >= 0xC000 && addr <= 0xCFFF) {
        // Soft switches $C000-$C07F (reads)
        if (addr <= 0xC07F) {
            switch (addr) {
            case 0xC000:  // KBD - keyboard data
                return s_a2.key_data;
            case 0xC010:  // KBDSTRB - clear keyboard strobe
                s_a2.key_data &= 0x7F;
                return 0;
            case 0xC011:  // LCRAM2 - read LC bank 2
                return s_a2.lc_bank2 ? 0x80 : 0x00;
            case 0xC012:  // LCRAM - read LC RAM
                return s_a2.lc_ram_read ? 0x80 : 0x00;
            case 0xC013:  // RAMRD
                return s_a2.ramrd ? 0x80 : 0x00;
            case 0xC014:  // RAMWRT
                return s_a2.ramwrt ? 0x80 : 0x00;
            case 0xC015:  // INTCXROM
                return s_a2.intcxrom ? 0x80 : 0x00;
            case 0xC016:  // ALTZP
                return s_a2.altzp ? 0x80 : 0x00;
            case 0xC017:  // SLOTC3ROM
                return s_a2.slotc3rom ? 0x80 : 0x00;
            case 0xC018:  // 80STORE
                return s_a2.store80 ? 0x80 : 0x00;
            case 0xC019:  // VBLANK
                return (s_a2.frame_count & 1) ? 0x80 : 0x00; // Toggle approx
            case 0xC01A:  // TEXT
                return s_a2.text_mode ? 0x80 : 0x00;
            case 0xC01B:  // MIXED
                return s_a2.mixed_mode ? 0x80 : 0x00;
            case 0xC01C:  // PAGE2
                return s_a2.page2 ? 0x80 : 0x00;
            case 0xC01D:  // HIRES
                return s_a2.hires ? 0x80 : 0x00;
            case 0xC01E:  // ALTCHARSET
                return 0x00; // Normal charset
            case 0xC01F:  // 80COL
                return s_a2.col80 ? 0x80 : 0x00;
            case 0xC030:  // SPKR
                s_a2.speaker_state = !s_a2.speaker_state;
                return 0;
            case 0xC050:  // TXTCLR (graphics mode)
                s_a2.text_mode = false;
                return 0;
            case 0xC051:  // TXTSET (text mode)
                s_a2.text_mode = true;
                return 0;
            case 0xC052:  // MIXCLR
                s_a2.mixed_mode = false;
                return 0;
            case 0xC053:  // MIXSET
                s_a2.mixed_mode = true;
                return 0;
            case 0xC054:  // LOWSCR (page 1)
                s_a2.page2 = false;
                return 0;
            case 0xC055:  // HISCR (page 2)
                s_a2.page2 = true;
                return 0;
            case 0xC056:  // LORES
                s_a2.hires = false;
                return 0;
            case 0xC057:  // HIRES
                s_a2.hires = true;
                return 0;
            case 0xC05E:  // DHIRES on
                s_a2.dhires = true;
                return 0;
            case 0xC05F:  // DHIRES off
                s_a2.dhires = false;
                return 0;
            default:
                break;
            }
            return 0;
        }

        // Language card switches ($C080-$C08F)
        if (addr >= 0xC080 && addr <= 0xC08F) {
            return lc_io_read(addr);
        }

        // Disk II controller slot 6 ($C0E0-$C0EF)
        if (addr >= 0xC0E0 && addr <= 0xC0EF) {
            return disk_io_read((uint8_t)(addr & 0x0F));
        }

        // Slot 6 ROM ($C600-$C6FF) - Disk II boot ROM
        if (addr >= 0xC600 && addr <= 0xC6FF) {
            if (s_a2.disk.rom_loaded) {
                return s_a2.disk.rom[addr - 0xC600];
            }
            return 0;
        }

        // Other slot ROM / expansion ROM
        return 0;
    }

    // ROM area ($D000-$FFFF)
    if (addr >= 0xD000) {
        if (s_a2.lc_ram_read) {
            // Read from language card RAM
            uint8_t *ram = s_a2.altzp ? s_a2.aux_ram : s_a2.main_ram;
            return ram[addr];
        }
        // Read from ROM
        if (s_a2.rom) {
            return s_a2.rom[addr - 0xD000];
        }
        return 0xFF;
    }

    // Zero page / stack with ALTZP
    if (addr < 0x0200 && s_a2.altzp && s_a2.aux_ram) {
        return s_a2.aux_ram[addr];
    }

    // 80STORE: text/hires pages go to aux when page2 selected
    if (s_a2.store80 && s_a2.aux_ram) {
        if (s_a2.page2) {
            if (addr >= 0x0400 && addr < 0x0800) return s_a2.aux_ram[addr];
            if (s_a2.hires && addr >= 0x2000 && addr < 0x4000) return s_a2.aux_ram[addr];
        }
    }

    // RAMRD: read aux memory
    if (s_a2.ramrd && s_a2.aux_ram && addr < 0xC000) {
        return s_a2.aux_ram[addr];
    }

    return s_a2.main_ram[addr];
}

static void mem_write(void *ctx, uint16_t addr, uint8_t val) {
    (void)ctx;

    // I/O space
    if (addr >= 0xC000 && addr <= 0xCFFF) {
        // Soft switch writes ($C000-$C00F) - IIe enhanced
        if (addr <= 0xC00F) {
            switch (addr) {
            case 0xC000: s_a2.store80 = false; break;
            case 0xC001: s_a2.store80 = true; break;
            case 0xC002: s_a2.ramrd = false; break;
            case 0xC003: s_a2.ramrd = true; break;
            case 0xC004: s_a2.ramwrt = false; break;
            case 0xC005: s_a2.ramwrt = true; break;
            case 0xC006: s_a2.intcxrom = false; break;
            case 0xC007: s_a2.intcxrom = true; break;
            case 0xC008: s_a2.altzp = false; break;
            case 0xC009: s_a2.altzp = true; break;
            case 0xC00A: s_a2.slotc3rom = false; break;
            case 0xC00B: s_a2.slotc3rom = true; break;
            case 0xC00C: s_a2.col80 = false; break;
            case 0xC00D: s_a2.col80 = true; break;
            }
            return;
        }

        // Writing to read-switches also triggers them
        if (addr == 0xC010) { s_a2.key_data &= 0x7F; return; }
        if (addr == 0xC030) { s_a2.speaker_state = !s_a2.speaker_state; return; }

        // Graphics soft switches (also writable)
        if (addr >= 0xC050 && addr <= 0xC05F) {
            mem_read(ctx, addr); // Read-effect triggers the switch
            return;
        }

        // Language card
        if (addr >= 0xC080 && addr <= 0xC08F) {
            lc_io_read(addr);
            return;
        }

        // Disk II
        if (addr >= 0xC0E0 && addr <= 0xC0EF) {
            disk_io_write((uint8_t)(addr & 0x0F), val);
            return;
        }

        return; // Other I/O - ignore writes
    }

    // ROM area - write to language card RAM if enabled
    if (addr >= 0xD000) {
        if (s_a2.lc_ram_write) {
            uint8_t *ram = s_a2.altzp ? s_a2.aux_ram : s_a2.main_ram;
            ram[addr] = val;
        }
        return; // Don't write to ROM
    }

    // Zero page / stack with ALTZP
    if (addr < 0x0200 && s_a2.altzp && s_a2.aux_ram) {
        s_a2.aux_ram[addr] = val;
        return;
    }

    // 80STORE: text/hires pages go to aux when page2 selected
    if (s_a2.store80 && s_a2.aux_ram) {
        if (s_a2.page2) {
            if (addr >= 0x0400 && addr < 0x0800) { s_a2.aux_ram[addr] = val; return; }
            if (s_a2.hires && addr >= 0x2000 && addr < 0x4000) { s_a2.aux_ram[addr] = val; return; }
        }
    }

    // RAMWRT: write to aux memory
    if (s_a2.ramwrt && s_a2.aux_ram && addr < 0xC000) {
        s_a2.aux_ram[addr] = val;
        return;
    }

    s_a2.main_ram[addr] = val;
}

// ============================================================================
// Screen rendering to GPU framebuffer
// ============================================================================

// Convert Apple II screen code to displayable character
static char apple2_screen_to_ascii(uint8_t ch) {
    // Normal text (most common): screen codes $A0-$FF map to ASCII $20-$7F
    if (ch >= 0xA0) return (char)(ch & 0x7F);
    // Inverse: $00-$3F
    if (ch < 0x40) return (char)(ch + 0x40);
    // Flash: $40-$7F
    if (ch < 0x80) return (char)(ch);
    // $80-$9F: inverse uppercase
    return (char)(ch & 0x7F);
}

static void render_text_mode(uint16_t *fb, int fb_w, int fb_h) {
    uint16_t text_base = s_a2.page2 ? 0x0800 : 0x0400;
    uint16_t fg = 0x07E0; // Green (Apple II green phosphor)
    uint16_t bg = 0x0000; // Black

    // 40x24 text, centered on 480x320 LCD
    // Each character: 7 pixels wide, 8 pixels tall → 280x192 in original
    // Scale up: 480/280 ≈ 1.7, but let's use integer scale
    // Simple approach: 12x13 pixel per char = 480x312 (close to 320)
    int char_w = 12;
    int char_h = 13;
    int x_off = 0;
    int y_off = (fb_h - 24 * char_h) / 2;
    if (y_off < 0) {
        char_h = fb_h / 24;
        y_off = 0;
    }

    // Fill with black
    memset(fb, 0, fb_w * fb_h * sizeof(uint16_t));

    for (int row = 0; row < 24; row++) {
        uint16_t line_offset = text_line_addr[row];
        for (int col = 0; col < 40; col++) {
            uint8_t sc = s_a2.main_ram[text_base + line_offset + col];
            bool inverse = (sc < 0x40) || (sc >= 0x40 && sc < 0x80);
            uint8_t ascii_ch = apple2_screen_to_ascii(sc);
            if (ascii_ch < 0x20 || ascii_ch > 0x7E) ascii_ch = ' ';
            uint8_t glyph_idx = ascii_ch - 0x20;

            uint16_t fg_c = inverse ? bg : fg;
            uint16_t bg_c = inverse ? fg : bg;

            // Render character scaled
            int px = x_off + col * char_w;
            int py = y_off + row * char_h;

            for (int gy = 0; gy < 8 && (py + gy * char_h / 8) < fb_h; gy++) {
                uint8_t glyph_row = 0;
                #ifdef APPLE2E_FONT_DATA
                if (glyph_idx < 96) glyph_row = apple2e_font[glyph_idx * 8 + gy];
                #endif

                for (int gx = 0; gx < 7; gx++) {
                    uint16_t color = (glyph_row & (0x40 >> gx)) ? fg_c : bg_c;
                    // Scale pixel to char_w x char_h
                    int sx = px + gx * char_w / 7;
                    int ex = px + (gx + 1) * char_w / 7;
                    int sy = py + gy * char_h / 8;
                    int ey = py + (gy + 1) * char_h / 8;
                    for (int y = sy; y < ey && y < fb_h; y++) {
                        for (int x = sx; x < ex && x < fb_w; x++) {
                            fb[y * fb_w + x] = color;
                        }
                    }
                }
            }
        }
    }
}

static void render_lores_mode(uint16_t *fb, int fb_w, int fb_h) {
    uint16_t text_base = s_a2.page2 ? 0x0800 : 0x0400;

    // LoRes: 40x48, each byte has 2 vertical nibbles (top nibble = upper, lower nibble = lower)
    // Actually: lower nibble = top block, upper nibble = bottom block
    int block_w = fb_w / 40;    // 12 pixels wide
    int block_h = fb_h / 48;    // ~6.67 pixels tall
    if (block_h < 1) block_h = 1;
    int y_off = (fb_h - 48 * block_h) / 2;

    memset(fb, 0, fb_w * fb_h * sizeof(uint16_t));

    int text_rows = s_a2.mixed_mode ? 20 : 24;

    for (int row = 0; row < text_rows; row++) {
        uint16_t line_offset = text_line_addr[row];
        for (int col = 0; col < 40; col++) {
            uint8_t val = s_a2.main_ram[text_base + line_offset + col];
            uint8_t lo_nib = val & 0x0F;        // Top block
            uint8_t hi_nib = (val >> 4) & 0x0F;  // Bottom block

            uint16_t color_top = apple2_colors_rgb565[lo_nib];
            uint16_t color_bot = apple2_colors_rgb565[hi_nib];

            int px = col * block_w;
            int py_top = y_off + row * 2 * block_h;
            int py_bot = y_off + (row * 2 + 1) * block_h;

            for (int y = py_top; y < py_top + block_h && y < fb_h; y++)
                for (int x = px; x < px + block_w && x < fb_w; x++)
                    fb[y * fb_w + x] = color_top;

            for (int y = py_bot; y < py_bot + block_h && y < fb_h; y++)
                for (int x = px; x < px + block_w && x < fb_w; x++)
                    fb[y * fb_w + x] = color_bot;
        }
    }

    // If mixed mode, render text in bottom 4 rows
    if (s_a2.mixed_mode) {
        uint16_t fg = 0x07E0;
        uint16_t bg_color = 0x0000;
        int char_w = block_w;
        int char_h = block_h * 2;
        for (int row = 20; row < 24; row++) {
            uint16_t line_offset = text_line_addr[row];
            for (int col = 0; col < 40; col++) {
                uint8_t sc = s_a2.main_ram[text_base + line_offset + col];
                uint8_t ascii_ch = apple2_screen_to_ascii(sc);
                if (ascii_ch < 0x20 || ascii_ch > 0x7E) ascii_ch = ' ';

                int px = col * char_w;
                int py = y_off + row * 2 * block_h;

                // Simple: fill with foreground or background based on character
                uint16_t color = (ascii_ch != ' ') ? fg : bg_color;
                for (int y = py; y < py + char_h && y < fb_h; y++)
                    for (int x = px; x < px + char_w && x < fb_w; x++)
                        fb[y * fb_w + x] = color;
            }
        }
    }
}

static void render_hires_mode(uint16_t *fb, int fb_w, int fb_h) {
    uint16_t hires_base = s_a2.page2 ? 0x4000 : 0x2000;

    // HiRes: 280x192, each byte has 7 pixels (bit 7 = color shift)
    // Scale: 480/280 ≈ 1.71 → use nearest neighbor
    int x_off = (fb_w - 280 * fb_w / 280) / 2; // 0 for width-fit
    int y_off = (fb_h - 192 * fb_h / 192) / 2; // 0 for height-fit
    (void)x_off; (void)y_off;

    float sx = (float)fb_w / 280.0f;
    float sy = (float)fb_h / 192.0f;
    if (sy > sx) sy = sx; // Keep aspect ratio
    int scaled_w = (int)(280 * sx);
    int scaled_h = (int)(192 * sy);
    int ox = (fb_w - scaled_w) / 2;
    int oy = (fb_h - scaled_h) / 2;

    memset(fb, 0, fb_w * fb_h * sizeof(uint16_t));

    int hires_rows = s_a2.mixed_mode ? 160 : 192;

    for (int y = 0; y < hires_rows; y++) {
        uint16_t addr = hires_base + hires_line_addr[y];
        for (int byte_col = 0; byte_col < 40; byte_col++) {
            uint8_t val = s_a2.main_ram[addr + byte_col];
            bool palette = (val & 0x80) != 0; // Color palette bit
            for (int bit = 0; bit < 7; bit++) {
                int px = byte_col * 7 + bit;
                bool on = (val & (1 << bit)) != 0;

                uint16_t color;
                if (!on) {
                    color = 0x0000; // Black
                } else if (palette) {
                    color = (px & 1) ? 0x055F : 0xFBC0; // Blue / Orange
                } else {
                    color = (px & 1) ? 0x07E0 : 0xD00A; // Green / Violet
                }

                // Scale to framebuffer
                int dx = ox + (int)(px * sx);
                int dy = oy + (int)(y * sy);
                int dx2 = ox + (int)((px + 1) * sx);
                int dy2 = oy + (int)((y + 1) * sy);
                if (dx2 <= dx) dx2 = dx + 1;
                if (dy2 <= dy) dy2 = dy + 1;

                for (int fy = dy; fy < dy2 && fy < fb_h; fy++)
                    for (int fx = dx; fx < dx2 && fx < fb_w; fx++)
                        fb[fy * fb_w + fx] = color;
            }
        }
    }
}

void apple2e_render_screen(void) {
    if (!s_a2.initialized) return;
    if (!video_card_is_initialized()) return;

    uint16_t *fb = video_card_get_back_buffer();
    if (!fb) return;

    int fb_w = video_card_get_width();
    int fb_h = video_card_get_height();

    if (s_a2.text_mode) {
        render_text_mode(fb, fb_w, fb_h);
    } else if (s_a2.hires) {
        render_hires_mode(fb, fb_w, fb_h);
    } else {
        render_lores_mode(fb, fb_w, fb_h);
    }

    video_card_present();

    // UART serial text output (diff-based, only text mode)
    if (s_a2.text_mode) {
        uint16_t text_base = s_a2.page2 ? 0x0800 : 0x0400;
        bool any_change = false;
        for (int i = 0; i < 960; i++) {
            uint8_t c = s_a2.main_ram[text_base + (i / 40 < 24 ? text_line_addr[i / 40] + (i % 40) : 0)];
            if (c != s_a2.prev_screen[i]) { any_change = true; break; }
        }
        if (any_change || s_a2.frame_count == 0) {
            if (s_a2.frame_count == 0) printf("\033[2J\033[H");
            for (int row = 0; row < 24; row++) {
                uint16_t line_offset = text_line_addr[row];
                char line_buf[42];
                for (int col = 0; col < 40; col++) {
                    uint8_t sc = s_a2.main_ram[text_base + line_offset + col];
                    line_buf[col] = apple2_screen_to_ascii(sc);
                    if (line_buf[col] < 0x20 || (unsigned char)line_buf[col] > 0x7E) line_buf[col] = ' ';
                }
                line_buf[40] = '\0';
                printf("\033[%d;1H%-40s", 1 + row, line_buf);
            }
            fflush(stdout);
            // Update tracking buffer
            for (int i = 0; i < 960; i++) {
                int row = i / 40;
                int col = i % 40;
                s_a2.prev_screen[i] = s_a2.main_ram[text_base + text_line_addr[row] + col];
            }
        }
    }
    s_a2.frame_count++;
}

// ============================================================================
// Keyboard polling
// ============================================================================

void apple2e_poll_keyboard(void) {
    if (!s_a2.initialized) return;
    if (s_a2.key_data & 0x80) return; // Previous key not consumed yet

    uint8_t ch = 0;

    // PS/2 keyboard
    if (ch == 0 && ps2_keyboard_is_initialized() && ps2_keyboard_available()) {
        ch = ps2_keyboard_read();
    }

    // USB keyboard
    if (ch == 0 && usb_keyboard_is_initialized() && usb_keyboard_available()) {
        ch = usb_keyboard_read();
    }

    // UART
    if (ch == 0) {
        size_t avail = 0;
        uart_get_buffered_data_len(CONFIG_ESP_CONSOLE_UART_NUM, &avail);
        if (avail > 0) {
            uint8_t buf;
            if (uart_read_bytes(CONFIG_ESP_CONSOLE_UART_NUM, &buf, 1, 0) == 1) {
                ch = buf;
            }
        }
    }

    if (ch == 0) return;

    // Ctrl+C = exit
    if (ch == 0x03) {
        s_a2.exit_requested = true;
        return;
    }

    // Arrow keys → Apple II equivalents
    switch (ch) {
    case PS2_KEY_LEFT:  ch = 0x08; break; // Backspace/Left
    case PS2_KEY_RIGHT: ch = 0x15; break; // Right (Ctrl+U)
    case PS2_KEY_UP:    ch = 0x0B; break; // Up (Ctrl+K)
    case PS2_KEY_DOWN:  ch = 0x0A; break; // Down (Ctrl+J)
    case PS2_KEY_ESCAPE: ch = 0x1B; break;
    case PS2_KEY_DELETE: ch = 0x7F; break;
    default:
        // Convert lowercase to uppercase (Apple II default)
        if (ch >= 'a' && ch <= 'z') ch -= 0x20;
        break;
    }

    // Set key with strobe
    s_a2.key_data = ch | 0x80;
}

// ============================================================================
// Public API
// ============================================================================

esp_err_t apple2e_init(void) {
    if (s_a2.initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    memset(&s_a2, 0, sizeof(s_a2));

    // Allocate main RAM (64KB)
    s_a2.main_ram = heap_caps_calloc(1, APPLE2_MEM_SIZE, MALLOC_CAP_SPIRAM);
    if (!s_a2.main_ram) {
        s_a2.main_ram = heap_caps_calloc(1, APPLE2_MEM_SIZE, MALLOC_CAP_DEFAULT);
    }
    if (!s_a2.main_ram) {
        ESP_LOGE(TAG, "Failed to allocate 64KB main RAM");
        return ESP_ERR_NO_MEM;
    }

    // Allocate auxiliary RAM (64KB) for enhanced IIe
    s_a2.aux_ram = heap_caps_calloc(1, APPLE2_AUX_MEM_SIZE, MALLOC_CAP_SPIRAM);
    if (!s_a2.aux_ram) {
        ESP_LOGW(TAG, "Aux RAM allocation failed - 40-column mode only");
    }

    // ROM buffer (12KB $D000-$FFFF)
    s_a2.rom = heap_caps_calloc(1, 0x3000, MALLOC_CAP_SPIRAM);
    if (!s_a2.rom) {
        s_a2.rom = malloc(0x3000);
    }

    // Initialize CPU
    mos6502_init(&s_a2.cpu);
    mos6502_set_callbacks(&s_a2.cpu, mem_read, mem_write, NULL);

    // Default soft switch states (power-on)
    s_a2.text_mode = true;
    s_a2.mixed_mode = false;
    s_a2.page2 = false;
    s_a2.hires = false;
    s_a2.col80 = false;
    s_a2.lc_ram_read = false;
    s_a2.lc_ram_write = false;

    // Build lookup tables
    build_hires_table();
    build_text_table();

    s_a2.initialized = true;
    ESP_LOGI(TAG, "Apple IIe emulator initialized (64KB main + %s aux)",
             s_a2.aux_ram ? "64KB" : "no");

    return ESP_OK;
}

esp_err_t apple2e_load_roms(void) {
    if (!s_a2.initialized) return ESP_ERR_INVALID_STATE;

    // Try loading Apple IIe ROM (12KB or 16KB)
    const char *rom_paths[] = {
        "/sdcard/apple2e/apple2e_enhanced.rom",
        "/sdcard/apple2e/apple2e.rom",
        "/sdcard/apple2e/APPLE2E.ROM",
        "/sdcard/apple2e/Apple2e.rom",
        NULL
    };

    bool rom_loaded = false;
    for (int i = 0; rom_paths[i] && !rom_loaded; i++) {
        FILE *f = fopen(rom_paths[i], "rb");
        if (f) {
            fseek(f, 0, SEEK_END);
            long fsize = ftell(f);
            fseek(f, 0, SEEK_SET);

            if (fsize >= 0x3000) {
                // If ROM is larger than 12KB, skip to last 12KB
                if (fsize > 0x3000) {
                    fseek(f, fsize - 0x3000, SEEK_SET);
                }
                size_t read = fread(s_a2.rom, 1, 0x3000, f);
                ESP_LOGI(TAG, "Loaded Apple IIe ROM: %s (%u bytes)", rom_paths[i], (unsigned)read);
                rom_loaded = true;

                // Also copy ROM into main RAM at $D000-$FFFF (for reset vectors)
                memcpy(&s_a2.main_ram[0xD000], s_a2.rom, 0x3000);
            }
            fclose(f);
        }
    }

    if (!rom_loaded) {
        ESP_LOGW(TAG, "Apple IIe ROM not found - place ROM in /sdcard/apple2e/");
        console_printf("Warning: Apple IIe ROM not found at /sdcard/apple2e/apple2e.rom\n");
        console_printf("  The Apple IIe requires a ROM image to boot.\n");

        // Install a minimal monitor that just shows a prompt
        // (RESET vector points to a simple loop)
        if (s_a2.rom) {
            // Put a simple JMP loop at the reset vector location
            s_a2.rom[0x2FFD - 0x0000] = 0x00; // Reset vector low = $FF00
            s_a2.rom[0x2FFE - 0x0000] = 0xFF; // Reset vector high
            // At $FF00: infinite loop (JMP $FF00)
            s_a2.rom[0x2F00] = 0x4C; // JMP
            s_a2.rom[0x2F01] = 0x00; // $FF00
            s_a2.rom[0x2F02] = 0xFF;

            memcpy(&s_a2.main_ram[0xD000], s_a2.rom, 0x3000);

            // Put "]" prompt text on screen
            s_a2.main_ram[0x0400 + text_line_addr[0]] = 0xDD; // "]" in Apple II screen code
        }
    }

    // Load Disk II controller ROM (256 bytes)
    const char *disk_rom_paths[] = {
        "/sdcard/apple2e/disk2.rom",
        "/sdcard/apple2e/DISK2.ROM",
        "/sdcard/apple2e/controller.rom",
        NULL
    };

    for (int i = 0; disk_rom_paths[i]; i++) {
        FILE *f = fopen(disk_rom_paths[i], "rb");
        if (f) {
            size_t read = fread(s_a2.disk.rom, 1, 256, f);
            fclose(f);
            if (read == 256) {
                s_a2.disk.rom_loaded = true;
                ESP_LOGI(TAG, "Loaded Disk II ROM: %s", disk_rom_paths[i]);
                break;
            }
        }
    }

    return ESP_OK;
}

void apple2e_reset(void) {
    if (!s_a2.initialized) return;

    // Copy ROM into memory so reset vectors are accessible
    if (s_a2.rom) {
        memcpy(&s_a2.main_ram[0xD000], s_a2.rom, 0x3000);
    }

    // Reset soft switches to default
    s_a2.text_mode = true;
    s_a2.mixed_mode = false;
    s_a2.page2 = false;
    s_a2.hires = false;
    s_a2.col80 = false;
    s_a2.store80 = false;
    s_a2.ramrd = false;
    s_a2.ramwrt = false;
    s_a2.altzp = false;
    s_a2.lc_ram_read = false;
    s_a2.lc_ram_write = false;
    s_a2.lc_bank2 = false;
    s_a2.lc_prewrite = false;
    s_a2.exit_requested = false;
    s_a2.frame_count = 0;
    memset(s_a2.prev_screen, 0, sizeof(s_a2.prev_screen));

    // Fill text screen with spaces (Apple II screen code for space = $A0)
    for (int row = 0; row < 24; row++) {
        for (int col = 0; col < 40; col++) {
            s_a2.main_ram[0x0400 + text_line_addr[row] + col] = 0xA0;
        }
    }

    mos6502_reset(&s_a2.cpu);
    ESP_LOGI(TAG, "Apple IIe reset: PC=$%04X", s_a2.cpu.pc);
}

uint64_t apple2e_run(uint32_t instructions) {
    if (!s_a2.initialized || s_a2.cpu.halted) return 0;
    uint64_t before = s_a2.cpu.cycles;
    mos6502_run(&s_a2.cpu, instructions);
    return s_a2.cpu.cycles - before;
}

void apple2e_step(void) {
    if (!s_a2.initialized) return;
    mos6502_step(&s_a2.cpu);
}

void apple2e_stop(void) {
    if (!s_a2.initialized) return;
    s_a2.cpu.halted = true;
    s_a2.exit_requested = true;
}

bool apple2e_is_halted(void) {
    return !s_a2.initialized || s_a2.cpu.halted;
}

bool apple2e_is_initialized(void) {
    return s_a2.initialized;
}

bool apple2e_exit_requested(void) {
    return s_a2.exit_requested;
}

void apple2e_get_state(char *buffer, size_t size) {
    if (!s_a2.initialized) {
        snprintf(buffer, size, "Apple IIe not initialized\n");
        return;
    }
    mos6502_t *c = &s_a2.cpu;
    snprintf(buffer, size,
             "Apple IIe 6502 State:\n"
             "  PC=$%04X A=$%02X X=$%02X Y=$%02X SP=$%02X\n"
             "  Flags: %c%c-%c%c%c%c%c\n"
             "  Cycles: %llu  Instructions: %llu\n"
             "  Mode: %s%s%s Page: %d\n"
             "  80col: %s  RAMRD: %s  RAMWRT: %s  ALTZP: %s\n",
             c->pc, c->a, c->x, c->y, c->sp,
             (c->status & MOS6502_FLAG_N) ? 'N' : 'n',
             (c->status & MOS6502_FLAG_V) ? 'V' : 'v',
             (c->status & MOS6502_FLAG_D) ? 'D' : 'd',
             (c->status & MOS6502_FLAG_I) ? 'I' : 'i',
             (c->status & MOS6502_FLAG_Z) ? 'Z' : 'z',
             (c->status & MOS6502_FLAG_C) ? 'C' : 'c',
             (c->status & MOS6502_FLAG_B) ? 'B' : 'b',
             (unsigned long long)c->cycles,
             (unsigned long long)c->instructions,
             s_a2.text_mode ? "TEXT" : (s_a2.hires ? "HIRES" : "LORES"),
             s_a2.mixed_mode ? "+MIXED" : "",
             s_a2.dhires ? "+DHIRES" : "",
             s_a2.page2 ? 2 : 1,
             s_a2.col80 ? "ON" : "OFF",
             s_a2.ramrd ? "AUX" : "MAIN",
             s_a2.ramwrt ? "AUX" : "MAIN",
             s_a2.altzp ? "AUX" : "MAIN");
}

void apple2e_dump_memory(uint32_t addr, uint32_t length) {
    if (!s_a2.initialized) return;
    for (uint32_t i = 0; i < length; i += 16) {
        console_printf("%04X: ", (unsigned)(addr + i));
        for (uint32_t j = 0; j < 16 && (i + j) < length; j++) {
            console_printf("%02X ", s_a2.main_ram[(addr + i + j) & 0xFFFF]);
        }
        console_printf("\n");
    }
}

void apple2e_load_program(const uint8_t *data, uint32_t size, uint32_t addr) {
    if (!s_a2.initialized || !data) return;
    for (uint32_t i = 0; i < size && (addr + i) < APPLE2_MEM_SIZE; i++) {
        s_a2.main_ram[addr + i] = data[i];
    }
}

uint8_t apple2e_read_memory(uint16_t addr) {
    if (!s_a2.initialized) return 0;
    return mem_read(NULL, addr);
}

void apple2e_write_memory(uint16_t addr, uint8_t val) {
    if (!s_a2.initialized) return;
    mem_write(NULL, addr, val);
}

void apple2e_destroy(void) {
    if (!s_a2.initialized) return;

    for (int d = 0; d < 2; d++) {
        if (s_a2.disk.drive[d].image_file) {
            fclose(s_a2.disk.drive[d].image_file);
            s_a2.disk.drive[d].image_file = NULL;
        }
        free(s_a2.disk.drive[d].track_data);
        s_a2.disk.drive[d].track_data = NULL;
    }

    heap_caps_free(s_a2.main_ram);
    heap_caps_free(s_a2.aux_ram);
    heap_caps_free(s_a2.rom);
    s_a2.main_ram = NULL;
    s_a2.aux_ram = NULL;
    s_a2.rom = NULL;
    s_a2.initialized = false;
    ESP_LOGI(TAG, "Apple IIe emulator destroyed");
}

esp_err_t apple2e_mount_disk(int drive, const char *path) {
    if (!s_a2.initialized) return ESP_ERR_INVALID_STATE;
    if (drive < 0 || drive > 1) return ESP_ERR_INVALID_ARG;

    disk_drive_t *drv = &s_a2.disk.drive[drive];

    // Close existing
    if (drv->image_file) {
        fclose(drv->image_file);
        drv->image_file = NULL;
        drv->mounted = false;
    }

    drv->image_file = fopen(path, "rb");
    if (!drv->image_file) {
        ESP_LOGE(TAG, "Failed to open disk image: %s", path);
        return ESP_ERR_NOT_FOUND;
    }

    fseek(drv->image_file, 0, SEEK_END);
    drv->image_size = ftell(drv->image_file);
    fseek(drv->image_file, 0, SEEK_SET);

    drv->mounted = true;
    drv->current_track = 0;
    drv->current_phase = 0;
    drv->head_position = 0;
    drv->motor_on = false;
    drv->write_mode = false;

    // Load first track
    disk_load_track(drv);

    ESP_LOGI(TAG, "Mounted disk %d: %s (%u bytes)", drive + 1, path, drv->image_size);
    return ESP_OK;
}
