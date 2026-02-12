/*
 * Video Card / GPU Device Implementation - Complete Rewrite
 *
 * Architecture:
 *   TEXT MODE:  Character buffer (80x25) rendered to framebuffer by refresh task.
 *              Like DOS video RAM at 0xB8000 - writes are instant, no flicker.
 *   GFX MODE:  Double-buffered 480x320 RGB565 with 2D acceleration.
 *              User draws on back_buf, FLIP swaps to front for display.
 *   3D MODE:   Extension of GFX mode with vertex/index buffer pipeline.
 *
 * All CPU emulators (M68K, i8086, 6502, etc.) access this device through
 * BUS_DEV_VIDEO registers at offset 0x08000 from BUS_IO_BASE.
 *
 * Console text output (from bus controller) is routed here when initialized,
 * replacing lcd_console to eliminate SPI bus conflicts.
 */

#include "video_card.h"
#include "bus_controller.h"
#include "lcd_console.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "VIDEO";

// External functions
extern int console_printf(const char *fmt, ...);

// Memory read callback for accessing emulator RAM (set by active emulator)
static video_emu_read_fn s_emu_read = NULL;

// Text mode constants (6x12 font gives 80x25 text on 480x320 LCD)
#define VID_TEXT_FONT_W   6
#define VID_TEXT_FONT_H   12
#define VID_TEXT_COLS      80    // 480 / 6 = 80
#define VID_TEXT_ROWS      25    // 300 / 12 = 25 (with 10px top + 10px bottom border)
#define VID_TEXT_BUF_SIZE  (VID_TEXT_COLS * VID_TEXT_ROWS)         // 2000
#define VID_TEXT_BORDER_Y  10   // Vertical border to center 300px in 320px

// DMA strip height for LCD transfer
#define DMA_STRIP_HEIGHT  8
#define DMA_BUF_SIZE      (VIDEO_MAX_WIDTH * DMA_STRIP_HEIGHT * 2)

// Default VGA attribute: light gray on black
#define VID_TEXT_DEFAULT_ATTR  0x07

// ============================================================================
// Video card state
// ============================================================================

static struct {
    bool initialized;
    bool enabled;

    // Display mode
    bool text_mode;          // true = text mode (char buffer), false = graphics mode

    // Framebuffers (SPIRAM, 480x320 x 2 bytes = 307,200 bytes each)
    uint16_t *front_buf;     // Buffer being displayed (sent to LCD by refresh task)
    uint16_t *back_buf;      // Buffer being drawn to (by 2D/3D commands)
    uint16_t *dma_buf;       // DMA bounce buffer (internal RAM, for SPI transfer)
    uint16_t  width;
    uint16_t  height;
    uint8_t   bpp;
    uint32_t  pitch;
    uint8_t   mode;

    // ---- Text mode state (like DOS video RAM at 0xB8000) ----
    uint8_t  text_chars[VID_TEXT_BUF_SIZE];  // Character codes (ASCII)
    uint8_t  text_attrs[VID_TEXT_BUF_SIZE];  // Attributes: hi nibble=bg, lo nibble=fg
    int      text_col, text_row;              // Cursor position
    uint8_t  text_cur_attr;                   // Current attribute for new characters

    // ---- Graphics mode state ----
    uint16_t  fg_color;
    uint16_t  bg_color;
    uint16_t  draw_color;

    // Text cursor (register values, for VID_REG_CURSOR_X/Y)
    uint16_t  cursor_x;
    uint16_t  cursor_y;

    // Drawing params (set via registers before command)
    int16_t   draw_x, draw_y;
    int16_t   draw_w, draw_h;
    int16_t   src_x, src_y;
    uint32_t  fb_addr;
    uint32_t  font_addr;
    int16_t   scroll_y;

    // Clipping rectangle
    int16_t   clip_x1, clip_y1;
    int16_t   clip_x2, clip_y2;

    // Palette (256 entries, RGB565)
    uint16_t  palette[256];

    // Sprites
    video_sprite_t sprites[VIDEO_MAX_SPRITES];

    // Tilemaps (2 layers)
    video_tilemap_t tilemap[2];

    // 3D engine control
    bool      d3d_enabled;
    bool      d3d_zbuffer;
    bool      d3d_backface_cull;
    uint8_t   d3d_shade_mode;
    uint32_t  d3d_vbuf_addr;
    uint32_t  d3d_vbuf_count;
    uint32_t  d3d_ibuf_addr;
    uint32_t  d3d_ibuf_count;
    int32_t   d3d_viewport_x, d3d_viewport_y;
    int32_t   d3d_viewport_w, d3d_viewport_h;
    fixed16_t d3d_light_x, d3d_light_y, d3d_light_z;
    uint16_t  d3d_light_color;
    uint16_t  d3d_ambient;

    // 3D texture state
    uint32_t  d3d_tex_addr;      // Texture data address in emulator RAM
    uint16_t  d3d_tex_width;     // Texture width in pixels
    uint16_t  d3d_tex_height;    // Texture height in pixels
    bool      d3d_textured;      // Texture mapping enabled

    // Display refresh
    TaskHandle_t refresh_task;
    uint32_t  frame_count;
    uint8_t   refresh_rate;
    bool      double_buffer;
    bool      vblank;

    // Command parameters (PARAM0-PARAM5)
    uint32_t  params[6];

    SemaphoreHandle_t mutex;
} s_vid;

// Forward declarations
static void exec_3d_command(uint32_t cmd);
static void render_text_to_framebuffer(void);

// ============================================================================
// 6x12 bitmap font (ASCII 32-126, 95 glyphs)
// Each glyph: 12 bytes (one per row), top 6 bits used (bit7=col0 .. bit2=col5)
// 5-pixel character body with 1 pixel right spacing for inter-character gap
// ============================================================================

static const uint8_t vid_font_6x12[][12] = {
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // 32 ' '
    {0x00,0x00,0x20,0x20,0x20,0x20,0x20,0x00,0x20,0x00,0x00,0x00}, // 33 '!'
    {0x00,0x00,0x50,0x50,0x50,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // 34 '"'
    {0x00,0x00,0x50,0xF8,0x50,0x50,0xF8,0x50,0x00,0x00,0x00,0x00}, // 35 '#'
    {0x00,0x20,0x70,0xA8,0xA0,0x70,0x28,0xA8,0x70,0x20,0x00,0x00}, // 36 '$'
    {0x00,0x00,0xC8,0xC8,0x10,0x20,0x40,0x98,0x98,0x00,0x00,0x00}, // 37 '%'
    {0x00,0x00,0x40,0xA0,0xA0,0x40,0xA8,0x90,0x68,0x00,0x00,0x00}, // 38 '&'
    {0x00,0x00,0x20,0x20,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // 39 '''
    {0x00,0x00,0x10,0x20,0x40,0x40,0x40,0x20,0x10,0x00,0x00,0x00}, // 40 '('
    {0x00,0x00,0x40,0x20,0x10,0x10,0x10,0x20,0x40,0x00,0x00,0x00}, // 41 ')'
    {0x00,0x00,0x20,0xA8,0x70,0xA8,0x20,0x00,0x00,0x00,0x00,0x00}, // 42 '*'
    {0x00,0x00,0x00,0x20,0x20,0xF8,0x20,0x20,0x00,0x00,0x00,0x00}, // 43 '+'
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x20,0x20,0x40,0x00,0x00}, // 44 ','
    {0x00,0x00,0x00,0x00,0x00,0xF8,0x00,0x00,0x00,0x00,0x00,0x00}, // 45 '-'
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x20,0x00,0x00,0x00}, // 46 '.'
    {0x00,0x00,0x08,0x08,0x10,0x20,0x40,0x80,0x80,0x00,0x00,0x00}, // 47 '/'
    {0x00,0x00,0x70,0x88,0x98,0xA8,0xC8,0x88,0x70,0x00,0x00,0x00}, // 48 '0'
    {0x00,0x00,0x20,0x60,0x20,0x20,0x20,0x20,0x70,0x00,0x00,0x00}, // 49 '1'
    {0x00,0x00,0x70,0x88,0x08,0x10,0x20,0x40,0xF8,0x00,0x00,0x00}, // 50 '2'
    {0x00,0x00,0x70,0x88,0x08,0x30,0x08,0x88,0x70,0x00,0x00,0x00}, // 51 '3'
    {0x00,0x00,0x10,0x30,0x50,0x90,0xF8,0x10,0x10,0x00,0x00,0x00}, // 52 '4'
    {0x00,0x00,0xF8,0x80,0xF0,0x08,0x08,0x88,0x70,0x00,0x00,0x00}, // 53 '5'
    {0x00,0x00,0x30,0x40,0x80,0xF0,0x88,0x88,0x70,0x00,0x00,0x00}, // 54 '6'
    {0x00,0x00,0xF8,0x08,0x10,0x20,0x20,0x20,0x20,0x00,0x00,0x00}, // 55 '7'
    {0x00,0x00,0x70,0x88,0x88,0x70,0x88,0x88,0x70,0x00,0x00,0x00}, // 56 '8'
    {0x00,0x00,0x70,0x88,0x88,0x78,0x08,0x10,0x60,0x00,0x00,0x00}, // 57 '9'
    {0x00,0x00,0x00,0x00,0x20,0x00,0x00,0x20,0x00,0x00,0x00,0x00}, // 58 ':'
    {0x00,0x00,0x00,0x00,0x20,0x00,0x00,0x20,0x20,0x40,0x00,0x00}, // 59 ';'
    {0x00,0x00,0x08,0x10,0x20,0x40,0x20,0x10,0x08,0x00,0x00,0x00}, // 60 '<'
    {0x00,0x00,0x00,0x00,0xF8,0x00,0xF8,0x00,0x00,0x00,0x00,0x00}, // 61 '='
    {0x00,0x00,0x80,0x40,0x20,0x10,0x20,0x40,0x80,0x00,0x00,0x00}, // 62 '>'
    {0x00,0x00,0x70,0x88,0x08,0x10,0x20,0x00,0x20,0x00,0x00,0x00}, // 63 '?'
    {0x00,0x00,0x70,0x88,0xB8,0xA8,0xB8,0x80,0x70,0x00,0x00,0x00}, // 64 '@'
    {0x00,0x00,0x70,0x88,0x88,0xF8,0x88,0x88,0x88,0x00,0x00,0x00}, // 65 'A'
    {0x00,0x00,0xF0,0x88,0x88,0xF0,0x88,0x88,0xF0,0x00,0x00,0x00}, // 66 'B'
    {0x00,0x00,0x70,0x88,0x80,0x80,0x80,0x88,0x70,0x00,0x00,0x00}, // 67 'C'
    {0x00,0x00,0xF0,0x88,0x88,0x88,0x88,0x88,0xF0,0x00,0x00,0x00}, // 68 'D'
    {0x00,0x00,0xF8,0x80,0x80,0xF0,0x80,0x80,0xF8,0x00,0x00,0x00}, // 69 'E'
    {0x00,0x00,0xF8,0x80,0x80,0xF0,0x80,0x80,0x80,0x00,0x00,0x00}, // 70 'F'
    {0x00,0x00,0x70,0x88,0x80,0x80,0x98,0x88,0x70,0x00,0x00,0x00}, // 71 'G'
    {0x00,0x00,0x88,0x88,0x88,0xF8,0x88,0x88,0x88,0x00,0x00,0x00}, // 72 'H'
    {0x00,0x00,0x70,0x20,0x20,0x20,0x20,0x20,0x70,0x00,0x00,0x00}, // 73 'I'
    {0x00,0x00,0x38,0x10,0x10,0x10,0x10,0x90,0x60,0x00,0x00,0x00}, // 74 'J'
    {0x00,0x00,0x88,0x90,0xA0,0xC0,0xA0,0x90,0x88,0x00,0x00,0x00}, // 75 'K'
    {0x00,0x00,0x80,0x80,0x80,0x80,0x80,0x80,0xF8,0x00,0x00,0x00}, // 76 'L'
    {0x00,0x00,0x88,0xD8,0xA8,0xA8,0x88,0x88,0x88,0x00,0x00,0x00}, // 77 'M'
    {0x00,0x00,0x88,0xC8,0xA8,0x98,0x88,0x88,0x88,0x00,0x00,0x00}, // 78 'N'
    {0x00,0x00,0x70,0x88,0x88,0x88,0x88,0x88,0x70,0x00,0x00,0x00}, // 79 'O'
    {0x00,0x00,0xF0,0x88,0x88,0xF0,0x80,0x80,0x80,0x00,0x00,0x00}, // 80 'P'
    {0x00,0x00,0x70,0x88,0x88,0x88,0xA8,0x90,0x68,0x00,0x00,0x00}, // 81 'Q'
    {0x00,0x00,0xF0,0x88,0x88,0xF0,0xA0,0x90,0x88,0x00,0x00,0x00}, // 82 'R'
    {0x00,0x00,0x70,0x88,0x80,0x70,0x08,0x88,0x70,0x00,0x00,0x00}, // 83 'S'
    {0x00,0x00,0xF8,0x20,0x20,0x20,0x20,0x20,0x20,0x00,0x00,0x00}, // 84 'T'
    {0x00,0x00,0x88,0x88,0x88,0x88,0x88,0x88,0x70,0x00,0x00,0x00}, // 85 'U'
    {0x00,0x00,0x88,0x88,0x88,0x88,0x50,0x50,0x20,0x00,0x00,0x00}, // 86 'V'
    {0x00,0x00,0x88,0x88,0x88,0xA8,0xA8,0xD8,0x88,0x00,0x00,0x00}, // 87 'W'
    {0x00,0x00,0x88,0x88,0x50,0x20,0x50,0x88,0x88,0x00,0x00,0x00}, // 88 'X'
    {0x00,0x00,0x88,0x88,0x50,0x20,0x20,0x20,0x20,0x00,0x00,0x00}, // 89 'Y'
    {0x00,0x00,0xF8,0x08,0x10,0x20,0x40,0x80,0xF8,0x00,0x00,0x00}, // 90 'Z'
    {0x00,0x00,0x70,0x40,0x40,0x40,0x40,0x40,0x70,0x00,0x00,0x00}, // 91 '['
    {0x00,0x00,0x80,0x80,0x40,0x20,0x10,0x08,0x08,0x00,0x00,0x00}, // 92 '\'
    {0x00,0x00,0x70,0x10,0x10,0x10,0x10,0x10,0x70,0x00,0x00,0x00}, // 93 ']'
    {0x00,0x00,0x20,0x50,0x88,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // 94 '^'
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF8,0x00,0x00,0x00}, // 95 '_'
    {0x00,0x00,0x40,0x20,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // 96 '`'
    {0x00,0x00,0x00,0x00,0x70,0x08,0x78,0x88,0x78,0x00,0x00,0x00}, // 97 'a'
    {0x00,0x00,0x80,0x80,0xF0,0x88,0x88,0x88,0xF0,0x00,0x00,0x00}, // 98 'b'
    {0x00,0x00,0x00,0x00,0x70,0x88,0x80,0x88,0x70,0x00,0x00,0x00}, // 99 'c'
    {0x00,0x00,0x08,0x08,0x78,0x88,0x88,0x88,0x78,0x00,0x00,0x00}, //100 'd'
    {0x00,0x00,0x00,0x00,0x70,0x88,0xF8,0x80,0x70,0x00,0x00,0x00}, //101 'e'
    {0x00,0x00,0x30,0x48,0x40,0xF0,0x40,0x40,0x40,0x00,0x00,0x00}, //102 'f'
    {0x00,0x00,0x00,0x00,0x78,0x88,0x88,0x78,0x08,0x70,0x00,0x00}, //103 'g'
    {0x00,0x00,0x80,0x80,0xF0,0x88,0x88,0x88,0x88,0x00,0x00,0x00}, //104 'h'
    {0x00,0x00,0x20,0x00,0x60,0x20,0x20,0x20,0x70,0x00,0x00,0x00}, //105 'i'
    {0x00,0x00,0x10,0x00,0x30,0x10,0x10,0x10,0x90,0x60,0x00,0x00}, //106 'j'
    {0x00,0x00,0x80,0x80,0x90,0xA0,0xC0,0xA0,0x90,0x00,0x00,0x00}, //107 'k'
    {0x00,0x00,0x60,0x20,0x20,0x20,0x20,0x20,0x70,0x00,0x00,0x00}, //108 'l'
    {0x00,0x00,0x00,0x00,0xD0,0xA8,0xA8,0xA8,0x88,0x00,0x00,0x00}, //109 'm'
    {0x00,0x00,0x00,0x00,0xF0,0x88,0x88,0x88,0x88,0x00,0x00,0x00}, //110 'n'
    {0x00,0x00,0x00,0x00,0x70,0x88,0x88,0x88,0x70,0x00,0x00,0x00}, //111 'o'
    {0x00,0x00,0x00,0x00,0xF0,0x88,0x88,0xF0,0x80,0x80,0x00,0x00}, //112 'p'
    {0x00,0x00,0x00,0x00,0x78,0x88,0x88,0x78,0x08,0x08,0x00,0x00}, //113 'q'
    {0x00,0x00,0x00,0x00,0xB0,0xC8,0x80,0x80,0x80,0x00,0x00,0x00}, //114 'r'
    {0x00,0x00,0x00,0x00,0x78,0x80,0x70,0x08,0xF0,0x00,0x00,0x00}, //115 's'
    {0x00,0x00,0x40,0x40,0xF0,0x40,0x40,0x48,0x30,0x00,0x00,0x00}, //116 't'
    {0x00,0x00,0x00,0x00,0x88,0x88,0x88,0x88,0x78,0x00,0x00,0x00}, //117 'u'
    {0x00,0x00,0x00,0x00,0x88,0x88,0x50,0x50,0x20,0x00,0x00,0x00}, //118 'v'
    {0x00,0x00,0x00,0x00,0x88,0xA8,0xA8,0xA8,0x50,0x00,0x00,0x00}, //119 'w'
    {0x00,0x00,0x00,0x00,0x88,0x50,0x20,0x50,0x88,0x00,0x00,0x00}, //120 'x'
    {0x00,0x00,0x00,0x00,0x88,0x88,0x88,0x78,0x08,0x70,0x00,0x00}, //121 'y'
    {0x00,0x00,0x00,0x00,0xF8,0x10,0x20,0x40,0xF8,0x00,0x00,0x00}, //122 'z'
    {0x00,0x00,0x18,0x20,0x20,0xC0,0x20,0x20,0x18,0x00,0x00,0x00}, //123 '{'
    {0x00,0x00,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x00,0x00,0x00}, //124 '|'
    {0x00,0x00,0xC0,0x20,0x20,0x18,0x20,0x20,0xC0,0x00,0x00,0x00}, //125 '}'
    {0x00,0x00,0x48,0xA8,0x90,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, //126 '~'
};

// VGA 16-color palette (RGB565) for text mode attributes
static const uint16_t vga_16_palette[16] = {
    0x0000,  // 0: Black
    0x0015,  // 1: Blue
    0x02E0,  // 2: Green
    0x02F5,  // 3: Cyan
    0xA800,  // 4: Red
    0xA815,  // 5: Magenta
    0xA560,  // 6: Brown
    0xAD55,  // 7: Light Gray
    0x52AA,  // 8: Dark Gray
    0x52BF,  // 9: Light Blue
    0x57EA,  // 10: Light Green
    0x57FF,  // 11: Light Cyan
    0xFAAA,  // 12: Light Red
    0xFABF,  // 13: Light Magenta
    0xFFE0,  // 14: Yellow
    0xFFFF,  // 15: White
};

// ============================================================================
// Internal 2D drawing primitives
// ============================================================================

static inline bool in_clip(int x, int y) {
    return x >= s_vid.clip_x1 && x < s_vid.clip_x2 &&
           y >= s_vid.clip_y1 && y < s_vid.clip_y2;
}

static inline void put_pixel(int x, int y, uint16_t color) {
    if (x >= 0 && x < s_vid.width && y >= 0 && y < s_vid.height && in_clip(x, y)) {
        s_vid.back_buf[y * s_vid.width + x] = color;
    }
}

static inline uint16_t get_pixel(int x, int y) {
    if (x >= 0 && x < s_vid.width && y >= 0 && y < s_vid.height) {
        return s_vid.back_buf[y * s_vid.width + x];
    }
    return 0;
}

static void draw_hline(int x, int y, int w, uint16_t color) {
    if (y < s_vid.clip_y1 || y >= s_vid.clip_y2) return;
    int x1 = (x < s_vid.clip_x1) ? s_vid.clip_x1 : x;
    int x2 = (x + w > s_vid.clip_x2) ? s_vid.clip_x2 : x + w;
    if (x1 >= x2) return;
    uint16_t *row = &s_vid.back_buf[y * s_vid.width];
    for (int i = x1; i < x2; i++) {
        row[i] = color;
    }
}

static void draw_vline(int x, int y, int h, uint16_t color) {
    if (x < s_vid.clip_x1 || x >= s_vid.clip_x2) return;
    int y1 = (y < s_vid.clip_y1) ? s_vid.clip_y1 : y;
    int y2 = (y + h > s_vid.clip_y2) ? s_vid.clip_y2 : y + h;
    for (int j = y1; j < y2; j++) {
        s_vid.back_buf[j * s_vid.width + x] = color;
    }
}

static void draw_line(int x0, int y0, int x1, int y1, uint16_t color) {
    int dx = abs(x1 - x0);
    int dy = -abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;

    while (true) {
        put_pixel(x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

static void fill_rect(int x, int y, int w, int h, uint16_t color) {
    for (int j = y; j < y + h; j++) {
        draw_hline(x, j, w, color);
    }
}

__attribute__((unused))
static void draw_rect_outline(int x, int y, int w, int h, uint16_t color) {
    draw_hline(x, y, w, color);
    draw_hline(x, y + h - 1, w, color);
    draw_vline(x, y, h, color);
    draw_vline(x + w - 1, y, h, color);
}

static void draw_circle(int cx, int cy, int r, uint16_t color, bool filled) {
    int x = 0, y = r;
    int d = 1 - r;

    while (x <= y) {
        if (filled) {
            draw_hline(cx - x, cy + y, 2 * x + 1, color);
            draw_hline(cx - x, cy - y, 2 * x + 1, color);
            draw_hline(cx - y, cy + x, 2 * y + 1, color);
            draw_hline(cx - y, cy - x, 2 * y + 1, color);
        } else {
            put_pixel(cx + x, cy + y, color);
            put_pixel(cx - x, cy + y, color);
            put_pixel(cx + x, cy - y, color);
            put_pixel(cx - x, cy - y, color);
            put_pixel(cx + y, cy + x, color);
            put_pixel(cx - y, cy + x, color);
            put_pixel(cx + y, cy - x, color);
            put_pixel(cx - y, cy - x, color);
        }
        x++;
        if (d < 0) {
            d += 2 * x + 1;
        } else {
            y--;
            d += 2 * (x - y) + 1;
        }
    }
}

// ============================================================================
// Sprite rendering
// ============================================================================

__attribute__((unused))
static void render_sprites(void) {
    for (int pri = 255; pri >= 0; pri--) {
        for (int i = 0; i < VIDEO_MAX_SPRITES; i++) {
            video_sprite_t *sp = &s_vid.sprites[i];
            if (!(sp->flags & SPRITE_FLAG_VISIBLE)) continue;
            if (sp->priority != pri) continue;
            if (!s_emu_read) continue;

            for (int sy = 0; sy < sp->h; sy++) {
                for (int sx = 0; sx < sp->w; sx++) {
                    int src_x = (sp->flags & SPRITE_FLAG_FLIP_H) ? (sp->w - 1 - sx) : sx;
                    int src_y = (sp->flags & SPRITE_FLAG_FLIP_V) ? (sp->h - 1 - sy) : sy;

                    uint16_t color;
                    if (sp->flags & SPRITE_FLAG_PALETTE) {
                        uint8_t idx = s_emu_read(sp->data_addr + src_y * sp->w + src_x);
                        color = s_vid.palette[idx];
                    } else {
                        uint32_t offset = (src_y * sp->w + src_x) * 2;
                        uint8_t lo = s_emu_read(sp->data_addr + offset);
                        uint8_t hi = s_emu_read(sp->data_addr + offset + 1);
                        color = (uint16_t)(lo | (hi << 8));
                    }

                    if ((sp->flags & SPRITE_FLAG_TRANS_KEY) && color == sp->trans_color) {
                        continue;
                    }

                    put_pixel(sp->x + sx, sp->y + sy, color);
                }
            }
        }
    }
}

// ============================================================================
// Tilemap rendering
// ============================================================================

__attribute__((unused))
static void render_tilemap(int layer) {
    if (layer < 0 || layer > 1) return;
    video_tilemap_t *tm = &s_vid.tilemap[layer];
    if (!tm->enabled || !s_emu_read || tm->tile_size == 0) return;

    int ts = tm->tile_size;
    int start_tx = tm->scroll_x / ts;
    int start_ty = tm->scroll_y / ts;
    int offset_x = tm->scroll_x % ts;
    int offset_y = tm->scroll_y % ts;

    int tiles_x = (s_vid.width / ts) + 2;
    int tiles_y = (s_vid.height / ts) + 2;

    for (int ty = 0; ty < tiles_y; ty++) {
        for (int tx = 0; tx < tiles_x; tx++) {
            int map_x = (start_tx + tx) % tm->map_w;
            int map_y = (start_ty + ty) % tm->map_h;
            if (map_x < 0) map_x += tm->map_w;
            if (map_y < 0) map_y += tm->map_h;

            uint32_t map_offset = (map_y * tm->map_w + map_x) * 2;
            uint8_t lo = s_emu_read(tm->map_addr + map_offset);
            uint8_t hi = s_emu_read(tm->map_addr + map_offset + 1);
            uint16_t tile_idx = (uint16_t)(lo | (hi << 8));

            uint32_t tile_data = tm->set_addr + tile_idx * ts * ts * 2;

            int scr_x = tx * ts - offset_x;
            int scr_y = ty * ts - offset_y;

            for (int py = 0; py < ts; py++) {
                for (int px = 0; px < ts; px++) {
                    uint32_t pix_offset = (py * ts + px) * 2;
                    uint8_t plo = s_emu_read(tile_data + pix_offset);
                    uint8_t phi = s_emu_read(tile_data + pix_offset + 1);
                    uint16_t color = (uint16_t)(plo | (phi << 8));
                    if (color != 0) {
                        put_pixel(scr_x + px, scr_y + py, color);
                    }
                }
            }
        }
    }
}

// ============================================================================
// Text mode: render character buffer to pixel framebuffer
// Called by refresh task every frame when in text mode.
// Renders text_chars[] + text_attrs[] to front_buf using 6x12 font.
// 80x25 chars = 480x300 pixels, centered in 480x320 with 10px top/bottom border.
// ============================================================================

static void render_text_to_framebuffer(void) {
    if (!s_vid.front_buf) return;

    uint16_t *fb = s_vid.front_buf;
    int fb_w = s_vid.width;

    // Clear top and bottom border to black
    memset(fb, 0, fb_w * VID_TEXT_BORDER_Y * sizeof(uint16_t));
    int bot_start = VID_TEXT_BORDER_Y + VID_TEXT_ROWS * VID_TEXT_FONT_H;
    if (bot_start < s_vid.height) {
        memset(&fb[bot_start * fb_w], 0, fb_w * (s_vid.height - bot_start) * sizeof(uint16_t));
    }

    for (int row = 0; row < VID_TEXT_ROWS; row++) {
        for (int col = 0; col < VID_TEXT_COLS; col++) {
            int idx = row * VID_TEXT_COLS + col;
            uint8_t ch = s_vid.text_chars[idx];
            uint8_t attr = s_vid.text_attrs[idx];

            uint16_t fg = vga_16_palette[attr & 0x0F];
            uint16_t bg = vga_16_palette[(attr >> 4) & 0x0F];

            int ci = (int)ch - 32;
            if (ci < 0 || ci > 94) ci = 0;
            const uint8_t *glyph = vid_font_6x12[ci];

            int px = col * VID_TEXT_FONT_W;
            int py = VID_TEXT_BORDER_Y + row * VID_TEXT_FONT_H;

            for (int gy = 0; gy < VID_TEXT_FONT_H; gy++) {
                int y = py + gy;
                if (y >= s_vid.height) break;
                uint8_t bits = glyph[gy];
                uint16_t *row_ptr = &fb[y * fb_w + px];
                // Unrolled 6-pixel write
                row_ptr[0] = (bits & 0x80) ? fg : bg;
                row_ptr[1] = (bits & 0x40) ? fg : bg;
                row_ptr[2] = (bits & 0x20) ? fg : bg;
                row_ptr[3] = (bits & 0x10) ? fg : bg;
                row_ptr[4] = (bits & 0x08) ? fg : bg;
                row_ptr[5] = (bits & 0x04) ? fg : bg;
            }
        }
    }
}

// ============================================================================
// Display refresh task
// Runs at configured refresh rate, sends framebuffer to LCD via SPI.
// In text mode: renders char buffer to front_buf first, then sends to LCD.
// In gfx mode: sends front_buf directly to LCD (already has pixel data from FLIP).
// ============================================================================

static void video_refresh_task(void *arg) {
    while (s_vid.initialized) {
        if (s_vid.enabled && s_vid.front_buf && s_vid.dma_buf) {
            if (s_vid.mutex && xSemaphoreTake(s_vid.mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                s_vid.vblank = false;

                // In text mode, render the character buffer to pixels
                if (s_vid.text_mode) {
                    render_text_to_framebuffer();
                }

                // Transfer front_buf to LCD in DMA strips
                for (int y = 0; y < s_vid.height; y += DMA_STRIP_HEIGHT) {
                    int strip_h = DMA_STRIP_HEIGHT;
                    if (y + strip_h > s_vid.height) strip_h = s_vid.height - y;
                    size_t strip_pixels = s_vid.width * strip_h;

                    memcpy(s_vid.dma_buf, &s_vid.front_buf[y * s_vid.width],
                           strip_pixels * sizeof(uint16_t));

                    lcd_console_draw_raw(0, y, s_vid.width, y + strip_h, s_vid.dma_buf);
                }

                s_vid.vblank = true;
                s_vid.frame_count++;
                xSemaphoreGive(s_vid.mutex);
            }
        }

        uint32_t delay = (s_vid.refresh_rate > 0) ? (1000 / s_vid.refresh_rate) : 33;
        vTaskDelay(pdMS_TO_TICKS(delay));
    }
    vTaskDelete(NULL);
}

// ============================================================================
// Color resolution: palette index → RGB565 in 8bpp mode
// ============================================================================

static inline uint16_t resolve_color(uint16_t color) {
    if (s_vid.bpp == 8) {
        // In 8bpp palette mode, color is a palette index (0-255)
        return s_vid.palette[color & 0xFF];
    }
    // In 16bpp mode, color is already RGB565
    return color;
}

// ============================================================================
// Command execution (2D GPU commands)
// ============================================================================

static void exec_command(uint32_t cmd) {
    switch (cmd) {
    case VID_CMD_CLEAR:
        if (s_vid.text_mode) {
            // Clear text buffer
            memset(s_vid.text_chars, ' ', VID_TEXT_BUF_SIZE);
            memset(s_vid.text_attrs, s_vid.text_cur_attr, VID_TEXT_BUF_SIZE);
            s_vid.text_col = 0;
            s_vid.text_row = 0;
        } else if (s_vid.back_buf) {
            uint16_t clr = resolve_color(s_vid.bg_color);
            for (int i = 0; i < s_vid.width * s_vid.height; i++) {
                s_vid.back_buf[i] = clr;
            }
        }
        break;

    case VID_CMD_FLIP:
        if (s_vid.text_mode) {
            // In text mode, FLIP is a no-op - refresh task continuously renders
            break;
        }
        if (s_vid.double_buffer && s_vid.mutex) {
            xSemaphoreTake(s_vid.mutex, portMAX_DELAY);
            uint16_t *tmp = s_vid.front_buf;
            s_vid.front_buf = s_vid.back_buf;
            s_vid.back_buf = tmp;
            // Update 3D pipeline's framebuffer pointer after swap
            extern void video_3d_set_framebuffer(uint16_t *fb);
            video_3d_set_framebuffer(s_vid.back_buf);
            xSemaphoreGive(s_vid.mutex);
        } else if (!s_vid.double_buffer && s_vid.back_buf) {
            memcpy(s_vid.front_buf, s_vid.back_buf,
                   s_vid.width * s_vid.height * sizeof(uint16_t));
        }
        break;

    case VID_CMD_DRAW_PIXEL:
        if (!s_vid.text_mode && s_vid.back_buf)
            put_pixel(s_vid.draw_x, s_vid.draw_y, resolve_color(s_vid.draw_color));
        break;

    case VID_CMD_DRAW_LINE:
        if (!s_vid.text_mode && s_vid.back_buf)
            draw_line(s_vid.draw_x, s_vid.draw_y, s_vid.draw_w, s_vid.draw_h, resolve_color(s_vid.draw_color));
        break;

    case VID_CMD_FILL_RECT:
        if (!s_vid.text_mode && s_vid.back_buf)
            fill_rect(s_vid.draw_x, s_vid.draw_y, s_vid.draw_w, s_vid.draw_h, resolve_color(s_vid.draw_color));
        break;

    case VID_CMD_HLINE:
        if (!s_vid.text_mode && s_vid.back_buf)
            draw_hline(s_vid.draw_x, s_vid.draw_y, s_vid.draw_w, resolve_color(s_vid.draw_color));
        break;

    case VID_CMD_VLINE:
        if (!s_vid.text_mode && s_vid.back_buf)
            draw_vline(s_vid.draw_x, s_vid.draw_y, s_vid.draw_h, resolve_color(s_vid.draw_color));
        break;

    case VID_CMD_CIRCLE:
        if (!s_vid.text_mode && s_vid.back_buf)
            draw_circle(s_vid.draw_x, s_vid.draw_y, s_vid.draw_w, resolve_color(s_vid.draw_color), false);
        break;

    case VID_CMD_FILL_CIRCLE:
        if (!s_vid.text_mode && s_vid.back_buf)
            draw_circle(s_vid.draw_x, s_vid.draw_y, s_vid.draw_w, resolve_color(s_vid.draw_color), true);
        break;

    case VID_CMD_BLIT:
        if (!s_vid.text_mode && s_vid.back_buf && s_emu_read) {
            uint32_t src_addr = s_vid.fb_addr;
            int dx = s_vid.draw_x, dy = s_vid.draw_y;
            int w = s_vid.draw_w, h = s_vid.draw_h;
            for (int j = 0; j < h; j++) {
                for (int i = 0; i < w; i++) {
                    uint32_t off = ((s_vid.src_y + j) * w + (s_vid.src_x + i)) * 2;
                    uint8_t lo = s_emu_read(src_addr + off);
                    uint8_t hi = s_emu_read(src_addr + off + 1);
                    uint16_t color = (uint16_t)(lo | (hi << 8));
                    put_pixel(dx + i, dy + j, color);
                }
            }
        }
        break;

    case VID_CMD_SET_PALETTE: {
        uint16_t idx = s_vid.draw_x & 0xFF;
        s_vid.palette[idx] = s_vid.draw_color;
        break;
    }

    case VID_CMD_SCROLL:
        if (s_vid.text_mode) {
            // Scroll text buffer up by scroll_y text rows
            int rows = s_vid.scroll_y;
            if (rows > 0 && rows < VID_TEXT_ROWS) {
                int move = (VID_TEXT_ROWS - rows) * VID_TEXT_COLS;
                memmove(s_vid.text_chars, s_vid.text_chars + rows * VID_TEXT_COLS, move);
                memmove(s_vid.text_attrs, s_vid.text_attrs + rows * VID_TEXT_COLS, move);
                // Clear newly exposed rows
                memset(s_vid.text_chars + move, ' ', rows * VID_TEXT_COLS);
                memset(s_vid.text_attrs + move, s_vid.text_cur_attr, rows * VID_TEXT_COLS);
            }
        } else if (s_vid.back_buf && s_vid.scroll_y > 0 && s_vid.scroll_y < s_vid.height) {
            int rows_to_copy = s_vid.height - s_vid.scroll_y;
            memmove(s_vid.back_buf,
                    &s_vid.back_buf[s_vid.scroll_y * s_vid.width],
                    rows_to_copy * s_vid.width * sizeof(uint16_t));
            uint16_t clr = resolve_color(s_vid.bg_color);
            for (int i = rows_to_copy * s_vid.width; i < s_vid.width * s_vid.height; i++) {
                s_vid.back_buf[i] = clr;
            }
        }
        break;

    case VID_CMD_INIT: {
        // Switch video mode
        uint8_t mode = s_vid.mode;
        uint16_t w = s_vid.width, h = s_vid.height;

        if (mode == VID_MODE_TEXT_80x25) {
            // Text mode (using our 60x20 grid on 480x320)
            s_vid.text_mode = true;
            s_vid.bpp = 0;
            memset(s_vid.text_chars, ' ', VID_TEXT_BUF_SIZE);
            memset(s_vid.text_attrs, VID_TEXT_DEFAULT_ATTR, VID_TEXT_BUF_SIZE);
            s_vid.text_col = 0;
            s_vid.text_row = 0;
            ESP_LOGI(TAG, "Switched to TEXT mode (%dx%d chars)", VID_TEXT_COLS, VID_TEXT_ROWS);
            break;
        }

        // Graphics mode
        s_vid.text_mode = false;

        switch (mode) {
        case VID_MODE_320x200x8:
            w = 320; h = 200; s_vid.bpp = 8; break;
        case VID_MODE_320x240x16:
            w = 320; h = 240; s_vid.bpp = 16; break;
        case VID_MODE_480x320x16:
            w = 480; h = 320; s_vid.bpp = 16; break;
        default:
            break;
        }

        if (w > VIDEO_MAX_WIDTH) w = VIDEO_MAX_WIDTH;
        if (h > VIDEO_MAX_HEIGHT) h = VIDEO_MAX_HEIGHT;
        s_vid.width = w;
        s_vid.height = h;
        s_vid.pitch = w * 2;

        // Reset clipping
        s_vid.clip_x1 = 0;
        s_vid.clip_y1 = 0;
        s_vid.clip_x2 = w;
        s_vid.clip_y2 = h;

        // Clear both buffers
        if (s_vid.front_buf) {
            memset(s_vid.front_buf, 0, VIDEO_MAX_WIDTH * VIDEO_MAX_HEIGHT * sizeof(uint16_t));
        }
        if (s_vid.back_buf) {
            memset(s_vid.back_buf, 0, VIDEO_MAX_WIDTH * VIDEO_MAX_HEIGHT * sizeof(uint16_t));
        }

        s_vid.enabled = true;
        ESP_LOGI(TAG, "Switched to GFX mode: %dx%d, %d bpp", w, h, s_vid.bpp);
        break;
    }

    default:
        ESP_LOGD(TAG, "Unknown video command: 0x%02X", (unsigned)cmd);
        break;
    }
}

// ============================================================================
// Register read/write interface
// ============================================================================

uint32_t video_card_read(uint32_t reg, uint8_t size) {
    (void)size;

    // Palette registers (0x100-0x2FF)
    if (reg >= VID_PALETTE_OFFSET && reg < VID_PALETTE_OFFSET + VID_PALETTE_SIZE * 4) {
        uint32_t idx = (reg - VID_PALETTE_OFFSET) / 4;
        if (idx < 256) return s_vid.palette[idx];
        return 0;
    }

    switch (reg) {
    case VID_REG_STATUS: {
        uint32_t status = VID_STATUS_READY;
        if (s_vid.vblank) status |= VID_STATUS_VBLANK;
        if (s_vid.text_mode) status |= VID_STATUS_TEXT_MODE;
        else status |= VID_STATUS_GFX_MODE;
        return status;
    }
    case VID_REG_MODE:       return s_vid.mode;
    case VID_REG_WIDTH:      return s_vid.width;
    case VID_REG_HEIGHT:     return s_vid.height;
    case VID_REG_BPP:        return s_vid.bpp;
    case VID_REG_PITCH:      return s_vid.pitch;
    case VID_REG_FB_ADDR:    return s_vid.fb_addr;
    case VID_REG_FB_SIZE:    return s_vid.width * s_vid.height * 2;
    case VID_REG_CURSOR_X:   return s_vid.text_mode ? s_vid.text_col : s_vid.cursor_x;
    case VID_REG_CURSOR_Y:   return s_vid.text_mode ? s_vid.text_row : s_vid.cursor_y;
    case VID_REG_FG_COLOR:   return s_vid.fg_color;
    case VID_REG_BG_COLOR:   return s_vid.bg_color;
    case VID_REG_DRAW_X:     return (uint32_t)(int32_t)s_vid.draw_x;
    case VID_REG_DRAW_Y:     return (uint32_t)(int32_t)s_vid.draw_y;
    case VID_REG_DRAW_W:     return (uint32_t)(int32_t)s_vid.draw_w;
    case VID_REG_DRAW_H:     return (uint32_t)(int32_t)s_vid.draw_h;
    case VID_REG_DRAW_COLOR: return s_vid.draw_color;
    case VID_REG_VBLANK:     return s_vid.frame_count;
    case VID_REG_SCROLL_Y:   return (uint32_t)(int32_t)s_vid.scroll_y;
    default: return 0;
    }
}

void video_card_write(uint32_t reg, uint32_t data, uint8_t size) {
    (void)size;

    // Palette registers (0x100-0x2FF)
    if (reg >= VID_PALETTE_OFFSET && reg < VID_PALETTE_OFFSET + VID_PALETTE_SIZE * 4) {
        uint32_t idx = (reg - VID_PALETTE_OFFSET) / 4;
        if (idx < 256) s_vid.palette[idx] = (uint16_t)data;
        return;
    }

    // Sprite registers (0x300-0x6FF)
    if (reg >= 0x300 && reg < 0x300 + VIDEO_MAX_SPRITES * VIDEO_SPRITE_REGS) {
        uint32_t sprite_offset = reg - 0x300;
        uint32_t sprite_idx = sprite_offset / VIDEO_SPRITE_REGS;
        uint32_t field = sprite_offset % VIDEO_SPRITE_REGS;
        video_sprite_t *sp = &s_vid.sprites[sprite_idx];

        switch (field) {
        case 0x00: sp->x = (int16_t)data; break;
        case 0x02: sp->y = (int16_t)data; break;
        case 0x04: sp->w = (uint16_t)data; break;
        case 0x06: sp->h = (uint16_t)data; break;
        case 0x08: sp->data_addr = data; break;
        case 0x0C: sp->flags = (uint8_t)data; break;
        case 0x0D: sp->priority = (uint8_t)data; break;
        case 0x0E: sp->trans_color = (uint16_t)data; break;
        }
        return;
    }

    // Tilemap registers (0x700-0x7FF)
    if (reg >= 0x700 && reg < 0x800) {
        uint32_t tr = reg - 0x700;
        switch (tr) {
        case 0x00:
            s_vid.tilemap[0].tile_size = (data & 0x03) == 0 ? 8 : (data & 0x03) == 1 ? 16 : 32;
            s_vid.tilemap[0].enabled = (data & 0x01) != 0;
            s_vid.tilemap[1].enabled = (data & 0x02) != 0;
            break;
        case 0x04: s_vid.tilemap[0].map_addr = data; break;
        case 0x08: s_vid.tilemap[0].map_w = (uint16_t)data; break;
        case 0x0A: s_vid.tilemap[0].map_h = (uint16_t)data; break;
        case 0x0C: s_vid.tilemap[0].set_addr = data; break;
        case 0x10: s_vid.tilemap[0].scroll_x = (uint16_t)data; break;
        case 0x12: s_vid.tilemap[0].scroll_y = (uint16_t)data; break;
        case 0x14: s_vid.tilemap[1].map_addr = data; break;
        case 0x18: s_vid.tilemap[1].scroll_x = (uint16_t)data; break;
        case 0x1A: s_vid.tilemap[1].scroll_y = (uint16_t)data; break;
        }
        return;
    }

    // 3D engine registers (0x800-0x8FF)
    if (reg >= 0x800 && reg < 0x900) {
        uint32_t tr = reg - 0x800;
        switch (tr) {
        case 0x00:
            s_vid.d3d_enabled = (data & 0x01) != 0;
            s_vid.d3d_zbuffer = (data & 0x02) != 0;
            s_vid.d3d_backface_cull = (data & 0x04) != 0;
            s_vid.d3d_textured = (data & 0x08) != 0;
            s_vid.d3d_shade_mode = (data >> 4) & 0x03;
            break;
        case 0x04:
            exec_3d_command(data);
            break;
        case 0x08: s_vid.d3d_vbuf_addr = data; break;
        case 0x0C: s_vid.d3d_vbuf_count = data; break;
        case 0x10: s_vid.d3d_ibuf_addr = data; break;
        case 0x14: s_vid.d3d_ibuf_count = data; break;
        case 0x58: s_vid.d3d_viewport_x = (int32_t)data; break;
        case 0x5C: s_vid.d3d_viewport_y = (int32_t)data; break;
        case 0x60: s_vid.d3d_viewport_w = (int32_t)data; break;
        case 0x64: s_vid.d3d_viewport_h = (int32_t)data; break;
        case 0x68: s_vid.d3d_light_x = (fixed16_t)data; break;
        case 0x6C: s_vid.d3d_light_y = (fixed16_t)data; break;
        case 0x70: s_vid.d3d_light_z = (fixed16_t)data; break;
        case 0x74: s_vid.d3d_light_color = (uint16_t)data; break;
        case 0x78: s_vid.d3d_ambient = (uint16_t)data; break;
        case 0x80: s_vid.params[0] = data; break;
        case 0x84: s_vid.params[1] = data; break;
        case 0x88: s_vid.params[2] = data; break;
        case 0x8C: s_vid.params[3] = data; break;
        case 0x90: s_vid.params[4] = data; break;
        case 0x94: s_vid.params[5] = data; break;
        case 0x98: s_vid.d3d_tex_addr = data; break;
        case 0x9C: s_vid.d3d_tex_width = (uint16_t)data; break;
        case 0xA0: s_vid.d3d_tex_height = (uint16_t)data; break;
        default:
            if (tr >= 0x18 && tr < 0x58) {
                int idx = (tr - 0x18) / 4;
                if (idx < 16) {
                    video_matrix_t *mat = video_3d_get_matrix();
                    if (mat) mat->m[idx / 4][idx % 4] = (fixed16_t)data;
                }
            }
            break;
        }
        return;
    }

    // Standard control registers
    switch (reg) {
    case VID_REG_COMMAND:
        exec_command(data);
        break;
    case VID_REG_MODE:
        s_vid.mode = (uint8_t)data;
        break;
    case VID_REG_WIDTH:
        s_vid.width = (uint16_t)data;
        if (s_vid.width > VIDEO_MAX_WIDTH) s_vid.width = VIDEO_MAX_WIDTH;
        break;
    case VID_REG_HEIGHT:
        s_vid.height = (uint16_t)data;
        if (s_vid.height > VIDEO_MAX_HEIGHT) s_vid.height = VIDEO_MAX_HEIGHT;
        break;
    case VID_REG_BPP:
        s_vid.bpp = (uint8_t)data;
        break;
    case VID_REG_FB_ADDR:
        s_vid.fb_addr = data;
        break;
    case VID_REG_CURSOR_X:
        s_vid.cursor_x = (uint16_t)data;
        if (s_vid.text_mode) s_vid.text_col = (int)data;
        break;
    case VID_REG_CURSOR_Y:
        s_vid.cursor_y = (uint16_t)data;
        if (s_vid.text_mode) s_vid.text_row = (int)data;
        break;
    case VID_REG_FG_COLOR:
        s_vid.fg_color = (uint16_t)data;
        break;
    case VID_REG_BG_COLOR:
        s_vid.bg_color = (uint16_t)data;
        break;
    case VID_REG_DRAW_X:
        s_vid.draw_x = (int16_t)data;
        break;
    case VID_REG_DRAW_Y:
        s_vid.draw_y = (int16_t)data;
        break;
    case VID_REG_DRAW_W:
        s_vid.draw_w = (int16_t)data;
        break;
    case VID_REG_DRAW_H:
        s_vid.draw_h = (int16_t)data;
        break;
    case VID_REG_DRAW_COLOR:
        s_vid.draw_color = (uint16_t)data;
        break;
    case VID_REG_SRC_X:
        s_vid.src_x = (int16_t)data;
        break;
    case VID_REG_SRC_Y:
        s_vid.src_y = (int16_t)data;
        break;
    case VID_REG_FONT_ADDR:
        s_vid.font_addr = data;
        break;
    case VID_REG_SCROLL_Y:
        s_vid.scroll_y = (int16_t)data;
        break;
    }
}

// ============================================================================
// 3D command dispatch
// ============================================================================

static void exec_3d_command(uint32_t cmd) {
    // 3D commands only work in graphics mode
    if (s_vid.text_mode) {
        // Auto-switch to graphics mode for 3D
        s_vid.text_mode = false;
        s_vid.mode = VID_MODE_480x320x16;
        s_vid.bpp = 16;
        ESP_LOGI(TAG, "Auto-switched to GFX mode for 3D command");
    }

    switch (cmd) {
    case 0x01: // Clear Z-buffer
        video_3d_clear_zbuffer();
        break;
    case 0x02: // Draw triangles from vertex/index buffers
        if (s_emu_read && s_vid.d3d_vbuf_count > 0 && s_vid.d3d_ibuf_count > 0) {
            uint32_t vcount = s_vid.d3d_vbuf_count;
            uint32_t icount = s_vid.d3d_ibuf_count;
            if (vcount > VIDEO_MAX_VERTICES) vcount = VIDEO_MAX_VERTICES;
            if (icount > VIDEO_MAX_INDICES) icount = VIDEO_MAX_INDICES;

            video_vertex_t *verts = malloc(vcount * sizeof(video_vertex_t));
            uint16_t *indices = malloc(icount * sizeof(uint16_t));
            if (!verts || !indices) {
                free(verts);
                free(indices);
                break;
            }

            // Read vertices from emulator RAM (M68K = big-endian)
            for (uint32_t i = 0; i < vcount; i++) {
                uint32_t addr = s_vid.d3d_vbuf_addr + i * sizeof(video_vertex_t);
                int32_t *fields = (int32_t *)&verts[i];
                // Read 6 fixed-point fields: x,y,z, nx,ny,nz
                for (int f = 0; f < 6; f++) {
                    uint32_t a = addr + f * 4;
                    fields[f] = (int32_t)((s_emu_read(a) << 24) |
                                          (s_emu_read(a+1) << 16) |
                                          (s_emu_read(a+2) << 8) |
                                           s_emu_read(a+3));
                }
                // Read color (offset 24) and pad (offset 26)
                uint32_t ca = addr + 24;
                verts[i].color = (uint16_t)((s_emu_read(ca) << 8) | s_emu_read(ca+1));
                verts[i]._pad = 0;
                // Read UV coordinates (offset 28, 32)
                uint32_t ua = addr + 28;
                verts[i].u = (int32_t)((s_emu_read(ua) << 24) | (s_emu_read(ua+1) << 16) |
                                       (s_emu_read(ua+2) << 8) | s_emu_read(ua+3));
                uint32_t va = addr + 32;
                verts[i].v = (int32_t)((s_emu_read(va) << 24) | (s_emu_read(va+1) << 16) |
                                       (s_emu_read(va+2) << 8) | s_emu_read(va+3));
            }

            // Read indices from emulator RAM (big-endian)
            for (uint32_t i = 0; i < icount; i++) {
                uint32_t addr = s_vid.d3d_ibuf_addr + i * 2;
                indices[i] = (uint16_t)((s_emu_read(addr) << 8) | s_emu_read(addr + 1));
            }

            // Set texture and backface cull state before drawing
            video_3d_set_backface_cull(s_vid.d3d_backface_cull);
            if (s_vid.d3d_textured && s_emu_read) {
                video_3d_set_texture(s_vid.d3d_tex_addr, s_vid.d3d_tex_width,
                                     s_vid.d3d_tex_height, s_emu_read);
            } else {
                video_3d_set_texture(0, 0, 0, NULL);
            }

            video_3d_draw_triangles(
                verts, vcount, indices, icount,
                s_vid.d3d_shade_mode, s_vid.d3d_zbuffer,
                s_vid.d3d_light_x, s_vid.d3d_light_y, s_vid.d3d_light_z,
                s_vid.d3d_light_color, s_vid.d3d_ambient);

            free(verts);
            free(indices);
        }
        break;
    case 0x03: video_3d_identity(); break;
    case 0x04: video_3d_translate(s_vid.params[0], s_vid.params[1], s_vid.params[2]); break;
    case 0x05: video_3d_rotate_x(s_vid.params[0]); break;
    case 0x06: video_3d_rotate_y(s_vid.params[0]); break;
    case 0x07: video_3d_rotate_z(s_vid.params[0]); break;
    case 0x08: video_3d_scale(s_vid.params[0], s_vid.params[1], s_vid.params[2]); break;
    case 0x09: video_3d_perspective(s_vid.params[0], s_vid.params[1],
                                    s_vid.params[2], s_vid.params[3]); break;
    }
}

// ============================================================================
// Public API - Initialization
// ============================================================================

esp_err_t video_card_init(void) {
    if (s_vid.initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    memset(&s_vid, 0, sizeof(s_vid));

    // Allocate framebuffers in SPIRAM
    size_t fb_size = VIDEO_MAX_WIDTH * VIDEO_MAX_HEIGHT * sizeof(uint16_t);

    s_vid.front_buf = heap_caps_calloc(1, fb_size, MALLOC_CAP_SPIRAM);
    s_vid.back_buf = heap_caps_calloc(1, fb_size, MALLOC_CAP_SPIRAM);
    if (!s_vid.front_buf || !s_vid.back_buf) {
        ESP_LOGE(TAG, "Failed to allocate framebuffers (%u bytes each)", (unsigned)fb_size);
        heap_caps_free(s_vid.front_buf);
        heap_caps_free(s_vid.back_buf);
        s_vid.front_buf = NULL;
        s_vid.back_buf = NULL;
        return ESP_ERR_NO_MEM;
    }

    // DMA bounce buffer in internal RAM (for SPI LCD transfer)
    // MUST be in internal RAM for SPI DMA - SPIRAM will NOT work
    s_vid.dma_buf = heap_caps_malloc(DMA_BUF_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!s_vid.dma_buf) {
        ESP_LOGW(TAG, "DMA|INTERNAL alloc failed, trying INTERNAL only");
        s_vid.dma_buf = heap_caps_malloc(DMA_BUF_SIZE, MALLOC_CAP_INTERNAL);
    }
    if (!s_vid.dma_buf) {
        ESP_LOGE(TAG, "Failed to allocate DMA buffer");
        heap_caps_free(s_vid.front_buf);
        heap_caps_free(s_vid.back_buf);
        s_vid.front_buf = NULL;
        s_vid.back_buf = NULL;
        return ESP_ERR_NO_MEM;
    }

    // Default settings
    s_vid.width = VIDEO_MAX_WIDTH;
    s_vid.height = VIDEO_MAX_HEIGHT;
    s_vid.bpp = 16;
    s_vid.pitch = s_vid.width * 2;
    s_vid.mode = VID_MODE_480x320x16;
    s_vid.clip_x1 = 0;
    s_vid.clip_y1 = 0;
    s_vid.clip_x2 = s_vid.width;
    s_vid.clip_y2 = s_vid.height;
    s_vid.fg_color = 0xFFFF;
    s_vid.bg_color = 0x0000;
    s_vid.refresh_rate = 30;
    s_vid.double_buffer = true;

    // Initialize palette with VGA 16-color + grayscale gradient
    memcpy(s_vid.palette, vga_16_palette, sizeof(vga_16_palette));
    for (int i = 16; i < 256; i++) {
        uint8_t gray = (i - 16) * 255 / 239;
        uint16_t r5 = (gray >> 3) & 0x1F;
        uint16_t g6 = (gray >> 2) & 0x3F;
        uint16_t b5 = (gray >> 3) & 0x1F;
        s_vid.palette[i] = (r5 << 11) | (g6 << 5) | b5;
    }

    // Initialize 3D engine (uses back_buf for drawing)
    video_3d_init(s_vid.back_buf, s_vid.width, s_vid.height);

    // Create mutex for framebuffer access synchronization
    s_vid.mutex = xSemaphoreCreateMutex();

    // ---- Start in TEXT MODE with welcome screen ----
    s_vid.text_mode = true;
    s_vid.text_cur_attr = VID_TEXT_DEFAULT_ATTR;  // Light gray on black
    memset(s_vid.text_chars, ' ', VID_TEXT_BUF_SIZE);
    memset(s_vid.text_attrs, VID_TEXT_DEFAULT_ATTR, VID_TEXT_BUF_SIZE);
    s_vid.text_col = 0;
    s_vid.text_row = 0;

    s_vid.initialized = true;
    s_vid.enabled = true;

    // Start refresh task (sends framebuffer to LCD at configured rate)
    // Stack needs 6KB+ for SPI transaction overhead in esp_lcd_panel_draw_bitmap
    xTaskCreate(video_refresh_task, "vid_refresh", 6144, NULL, 5, &s_vid.refresh_task);

    ESP_LOGI(TAG, "Video card initialized: %dx%d, TEXT mode (%dx%d chars), %dHz refresh",
             s_vid.width, s_vid.height, VID_TEXT_COLS, VID_TEXT_ROWS, s_vid.refresh_rate);

    // Show welcome message on screen
    video_card_print("ESP32-P4 GPU Video Card v3.0\n");
    video_card_print("480x320 RGB565 | 3D Accelerator\n");
    video_card_print("Text: 80x25 | GFX: 480x320x16bpp\n");
    video_card_print("================================================================================\n");
    video_card_print("Ready.\n\n");

    return ESP_OK;
}

void video_card_deinit(void) {
    if (!s_vid.initialized) return;

    s_vid.initialized = false;
    s_vid.enabled = false;

    // Wait for refresh task to exit
    vTaskDelay(pdMS_TO_TICKS(100));

    video_3d_deinit();

    if (s_vid.mutex) {
        vSemaphoreDelete(s_vid.mutex);
        s_vid.mutex = NULL;
    }

    heap_caps_free(s_vid.front_buf);
    heap_caps_free(s_vid.back_buf);
    heap_caps_free(s_vid.dma_buf);
    s_vid.front_buf = NULL;
    s_vid.back_buf = NULL;
    s_vid.dma_buf = NULL;

    ESP_LOGI(TAG, "Video card deinitialized");
}

bool video_card_is_initialized(void) {
    return s_vid.initialized;
}

uint32_t video_card_get_frame_count(void) {
    return s_vid.frame_count;
}

void video_card_set_emu_read(video_emu_read_fn fn) {
    s_emu_read = fn;
}

// ============================================================================
// Public API - Text console (character buffer based, zero-flicker)
//
// In TEXT mode: writes to char buffer, refresh task renders to pixels at 30fps.
// In GFX mode: renders glyphs directly to front_buf (for overlay/debug text).
// ============================================================================

// Helper: render a single glyph directly to front_buf (used in GFX mode only)
static void gfx_draw_glyph(int px, int py, char ch, uint16_t fg, uint16_t bg) {
    if (!s_vid.front_buf) return;
    int ci = (int)ch - 32;
    if (ci < 0 || ci > 94) ci = 0;
    const uint8_t *glyph = vid_font_6x12[ci];
    uint16_t *fb = s_vid.front_buf;
    int w = s_vid.width;
    int h = s_vid.height;

    for (int gy = 0; gy < VID_TEXT_FONT_H; gy++) {
        int y = py + gy;
        if (y < 0 || y >= h) continue;
        uint8_t bits = glyph[gy];
        uint16_t *row_ptr = &fb[y * w + px];
        if (px >= 0 && px + 5 < w) {
            row_ptr[0] = (bits & 0x80) ? fg : bg;
            row_ptr[1] = (bits & 0x40) ? fg : bg;
            row_ptr[2] = (bits & 0x20) ? fg : bg;
            row_ptr[3] = (bits & 0x10) ? fg : bg;
            row_ptr[4] = (bits & 0x08) ? fg : bg;
            row_ptr[5] = (bits & 0x04) ? fg : bg;
        }
    }
}

void video_card_putchar(char ch) {
    if (!s_vid.initialized) return;

    // Auto-restore text mode when console output arrives while in GFX mode.
    // This handles the case where a GPU test/demo finishes and the console
    // resumes printing - we switch back to text mode so the user can see it.
    if (!s_vid.text_mode) {
        ESP_LOGI(TAG, "Auto-switching to TEXT mode for console output");
        s_vid.text_mode = true;
        s_vid.bpp = 0;
        s_vid.mode = VID_MODE_TEXT_80x25;
        s_vid.width = VIDEO_MAX_WIDTH;
        s_vid.height = VIDEO_MAX_HEIGHT;
        s_vid.clip_x1 = 0;
        s_vid.clip_y1 = 0;
        s_vid.clip_x2 = VIDEO_MAX_WIDTH;
        s_vid.clip_y2 = VIDEO_MAX_HEIGHT;
        memset(s_vid.text_chars, ' ', VID_TEXT_BUF_SIZE);
        memset(s_vid.text_attrs, VID_TEXT_DEFAULT_ATTR, VID_TEXT_BUF_SIZE);
        s_vid.text_col = 0;
        s_vid.text_row = 0;
        s_vid.text_cur_attr = VID_TEXT_DEFAULT_ATTR;
        // Clear front buffer to remove leftover GFX content
        if (s_vid.front_buf) {
            memset(s_vid.front_buf, 0, VIDEO_MAX_WIDTH * VIDEO_MAX_HEIGHT * sizeof(uint16_t));
        }
    }

    if (s_vid.text_mode) {
        // ---- TEXT MODE: update character buffer ----
        if (ch == '\r') {
            s_vid.text_col = 0;
            return;
        }
        if (ch == '\n') {
            s_vid.text_col = 0;
            s_vid.text_row++;
            if (s_vid.text_row >= VID_TEXT_ROWS) {
                s_vid.text_row = VID_TEXT_ROWS - 1;
                // Scroll text buffer up by one row
                memmove(s_vid.text_chars, s_vid.text_chars + VID_TEXT_COLS,
                        (VID_TEXT_ROWS - 1) * VID_TEXT_COLS);
                memmove(s_vid.text_attrs, s_vid.text_attrs + VID_TEXT_COLS,
                        (VID_TEXT_ROWS - 1) * VID_TEXT_COLS);
                // Clear last row
                int last = (VID_TEXT_ROWS - 1) * VID_TEXT_COLS;
                memset(&s_vid.text_chars[last], ' ', VID_TEXT_COLS);
                memset(&s_vid.text_attrs[last], s_vid.text_cur_attr, VID_TEXT_COLS);
            }
            return;
        }
        if (ch == '\b') {
            if (s_vid.text_col > 0) {
                s_vid.text_col--;
                int idx = s_vid.text_row * VID_TEXT_COLS + s_vid.text_col;
                s_vid.text_chars[idx] = ' ';
                s_vid.text_attrs[idx] = s_vid.text_cur_attr;
            }
            return;
        }
        if (ch == '\t') {
            int next = (s_vid.text_col + 8) & ~7;
            while (s_vid.text_col < next && s_vid.text_col < VID_TEXT_COLS) {
                int idx = s_vid.text_row * VID_TEXT_COLS + s_vid.text_col;
                s_vid.text_chars[idx] = ' ';
                s_vid.text_attrs[idx] = s_vid.text_cur_attr;
                s_vid.text_col++;
            }
            if (s_vid.text_col >= VID_TEXT_COLS) {
                s_vid.text_col = 0;
                s_vid.text_row++;
                if (s_vid.text_row >= VID_TEXT_ROWS) {
                    s_vid.text_row = VID_TEXT_ROWS - 1;
                    memmove(s_vid.text_chars, s_vid.text_chars + VID_TEXT_COLS,
                            (VID_TEXT_ROWS - 1) * VID_TEXT_COLS);
                    memmove(s_vid.text_attrs, s_vid.text_attrs + VID_TEXT_COLS,
                            (VID_TEXT_ROWS - 1) * VID_TEXT_COLS);
                    int last = (VID_TEXT_ROWS - 1) * VID_TEXT_COLS;
                    memset(&s_vid.text_chars[last], ' ', VID_TEXT_COLS);
                    memset(&s_vid.text_attrs[last], s_vid.text_cur_attr, VID_TEXT_COLS);
                }
            }
            return;
        }
        // Printable character
        if (ch >= 32 && ch <= 126) {
            int idx = s_vid.text_row * VID_TEXT_COLS + s_vid.text_col;
            if (idx >= 0 && idx < VID_TEXT_BUF_SIZE) {
                s_vid.text_chars[idx] = (uint8_t)ch;
                s_vid.text_attrs[idx] = s_vid.text_cur_attr;
            }
            s_vid.text_col++;
            if (s_vid.text_col >= VID_TEXT_COLS) {
                s_vid.text_col = 0;
                s_vid.text_row++;
                if (s_vid.text_row >= VID_TEXT_ROWS) {
                    s_vid.text_row = VID_TEXT_ROWS - 1;
                    memmove(s_vid.text_chars, s_vid.text_chars + VID_TEXT_COLS,
                            (VID_TEXT_ROWS - 1) * VID_TEXT_COLS);
                    memmove(s_vid.text_attrs, s_vid.text_attrs + VID_TEXT_COLS,
                            (VID_TEXT_ROWS - 1) * VID_TEXT_COLS);
                    int last = (VID_TEXT_ROWS - 1) * VID_TEXT_COLS;
                    memset(&s_vid.text_chars[last], ' ', VID_TEXT_COLS);
                    memset(&s_vid.text_attrs[last], s_vid.text_cur_attr, VID_TEXT_COLS);
                }
            }
        }
    } else {
        // ---- GFX MODE: render glyph directly to front_buf for debug overlay ----
        uint16_t fg = s_vid.fg_color;
        uint16_t bg = s_vid.bg_color;

        if (ch == '\r') { s_vid.text_col = 0; return; }
        if (ch == '\n') {
            s_vid.text_col = 0;
            s_vid.text_row++;
            if (s_vid.text_row >= VID_TEXT_ROWS) {
                s_vid.text_row = VID_TEXT_ROWS - 1;
                // Scroll front_buf pixels up
                int row_pixels = VID_TEXT_FONT_H;
                int move_lines = s_vid.height - row_pixels;
                if (move_lines > 0) {
                    memmove(s_vid.front_buf, s_vid.front_buf + s_vid.width * row_pixels,
                            move_lines * s_vid.width * sizeof(uint16_t));
                }
                uint16_t *last_row = s_vid.front_buf + move_lines * s_vid.width;
                int fill = row_pixels * s_vid.width;
                for (int i = 0; i < fill; i++) last_row[i] = bg;
            }
            return;
        }
        if (ch == '\b') {
            if (s_vid.text_col > 0) {
                s_vid.text_col--;
                gfx_draw_glyph(s_vid.text_col * VID_TEXT_FONT_W,
                               s_vid.text_row * VID_TEXT_FONT_H, ' ', fg, bg);
            }
            return;
        }
        if (ch >= 32 && ch <= 126) {
            gfx_draw_glyph(s_vid.text_col * VID_TEXT_FONT_W,
                           s_vid.text_row * VID_TEXT_FONT_H, ch, fg, bg);
            s_vid.text_col++;
            if (s_vid.text_col >= VID_TEXT_COLS) {
                s_vid.text_col = 0;
                s_vid.text_row++;
                if (s_vid.text_row >= VID_TEXT_ROWS) {
                    s_vid.text_row = VID_TEXT_ROWS - 1;
                    int row_pixels = VID_TEXT_FONT_H;
                    int move_lines = s_vid.height - row_pixels;
                    if (move_lines > 0) {
                        memmove(s_vid.front_buf, s_vid.front_buf + s_vid.width * row_pixels,
                                move_lines * s_vid.width * sizeof(uint16_t));
                    }
                    uint16_t *last_row = s_vid.front_buf + move_lines * s_vid.width;
                    int fill = row_pixels * s_vid.width;
                    for (int i = 0; i < fill; i++) last_row[i] = bg;
                }
            }
        }
    }
}

void video_card_print(const char *str) {
    if (!str) return;
    while (*str) {
        video_card_putchar(*str++);
    }
}

void video_card_text_clear(void) {
    if (!s_vid.initialized) return;

    if (s_vid.text_mode) {
        memset(s_vid.text_chars, ' ', VID_TEXT_BUF_SIZE);
        memset(s_vid.text_attrs, s_vid.text_cur_attr, VID_TEXT_BUF_SIZE);
    } else if (s_vid.front_buf) {
        int total = s_vid.width * s_vid.height;
        for (int i = 0; i < total; i++) {
            s_vid.front_buf[i] = s_vid.bg_color;
        }
    }
    s_vid.text_col = 0;
    s_vid.text_row = 0;
}

void video_card_text_set_colors(uint16_t fg, uint16_t bg) {
    // Set colors for GFX mode overlay
    s_vid.fg_color = fg;
    s_vid.bg_color = bg;

    // Also set text mode attribute from closest VGA palette match
    // For simplicity: fg=15 (white), bg=0 (black) by default
    // Users can set exact attribute via text_cur_attr directly
    int fg_idx = 7, bg_idx = 0;  // Default: light gray on black
    if (fg == 0xFFFF) fg_idx = 15;      // White
    else if (fg == 0x0000) fg_idx = 0;  // Black
    else if (fg == 0x07E0) fg_idx = 10; // Green
    else if (fg == 0xFFE0) fg_idx = 14; // Yellow
    else if (fg == 0xF800) fg_idx = 12; // Red
    else if (fg == 0x001F) fg_idx = 9;  // Blue

    s_vid.text_cur_attr = (uint8_t)((bg_idx << 4) | fg_idx);
}

// Set text attribute directly (VGA style: hi=bg, lo=fg)
void video_card_text_set_attr(uint8_t attr) {
    s_vid.text_cur_attr = attr;
}

// ============================================================================
// Public API - Direct framebuffer access (for 3D apps and emulator renderers)
// ============================================================================

uint16_t *video_card_get_back_buffer(void) {
    if (!s_vid.initialized) return NULL;
    return s_vid.back_buf;
}

void video_card_present(void) {
    if (!s_vid.initialized) return;

    // In text mode, present is a no-op - refresh task continuously renders
    if (s_vid.text_mode) return;

    if (s_vid.double_buffer && s_vid.mutex) {
        xSemaphoreTake(s_vid.mutex, portMAX_DELAY);
        uint16_t *tmp = s_vid.front_buf;
        s_vid.front_buf = s_vid.back_buf;
        s_vid.back_buf = tmp;
        // Update 3D pipeline framebuffer pointer to new back buffer
        extern void video_3d_set_framebuffer(uint16_t *fb);
        video_3d_set_framebuffer(s_vid.back_buf);
        xSemaphoreGive(s_vid.mutex);
    }
}

uint16_t video_card_get_width(void) {
    return s_vid.initialized ? s_vid.width : 0;
}

uint16_t video_card_get_height(void) {
    return s_vid.initialized ? s_vid.height : 0;
}

// Check if currently in text mode
bool video_card_is_text_mode(void) {
    return s_vid.initialized && s_vid.text_mode;
}

// Switch from text mode to GFX mode for direct framebuffer rendering
void video_card_enter_gfx_mode(void) {
    if (!s_vid.initialized) return;
    if (!s_vid.text_mode) return;  // Already in GFX mode

    s_vid.text_mode = false;
    s_vid.bpp = 16;
    s_vid.width = VIDEO_MAX_WIDTH;
    s_vid.height = VIDEO_MAX_HEIGHT;
    s_vid.pitch = s_vid.width * 2;
    s_vid.mode = VID_MODE_480x320x16;

    // Reset clipping to full screen
    s_vid.clip_x1 = 0;
    s_vid.clip_y1 = 0;
    s_vid.clip_x2 = s_vid.width;
    s_vid.clip_y2 = s_vid.height;

    // Clear both framebuffers to black
    if (s_vid.front_buf)
        memset(s_vid.front_buf, 0, VIDEO_MAX_WIDTH * VIDEO_MAX_HEIGHT * sizeof(uint16_t));
    if (s_vid.back_buf)
        memset(s_vid.back_buf, 0, VIDEO_MAX_WIDTH * VIDEO_MAX_HEIGHT * sizeof(uint16_t));

    ESP_LOGI(TAG, "Switched to GFX mode: %dx%d (for emulator rendering)", s_vid.width, s_vid.height);
}
