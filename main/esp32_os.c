/*
 * ESP32 Native OS - Windows 3.1-style Desktop Environment
 * Runs natively on ESP32-P4 Xtensa LX7 @ 400MHz
 * 480x320 RGB565 display via video_card.c double-buffered GPU
 * USB HID mouse + keyboard, PS/2 keyboard fallback
 *
 * Layout (480x320):
 *   Desktop (teal)  ............ y=0  to y=297
 *   Taskbar strip   ............ y=298 to y=319 (22px, draggable)
 *
 * Windows 3.1 visual style:
 *   Navy title bar (active), gray (inactive)
 *   3D beveled borders: white highlight top/left, gray shadow bottom/right
 *   Silver (#C0C0C0) window backgrounds
 *   Teal (#008080) desktop
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <dirent.h>
#include <sys/stat.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "video_card.h"
#include "usb_keyboard.h"
#include "usb_mouse.h"
#include "ps2_keyboard.h"

static const char *TAG = "esp32os";

// ============================================================================
// CONSTANTS
// ============================================================================

#define OS_W  480
#define OS_H  320

// RGB565 helper: R[4:0] G[5:0] B[4:0]
#define RGB565(r,g,b) ((uint16_t)((((r)&0xF8u)<<8)|(((g)&0xFCu)<<3)|((b)>>3)))

// Windows 3.1 palette
#define C_DESKTOP     RGB565(  0,128,128)
#define C_WIN_BG      RGB565(192,192,192)
#define C_TITLE_ACT   RGB565(  0,  0,128)
#define C_TITLE_INACT RGB565(128,128,128)
#define C_TITLE_TXT   RGB565(255,255,255)
#define C_HILIGHT     RGB565(255,255,255)
#define C_SHADOW      RGB565(128,128,128)
#define C_DKSHADOW    RGB565( 64, 64, 64)
#define C_BLACK       RGB565(  0,  0,  0)
#define C_WHITE       RGB565(255,255,255)
#define C_TASKBAR     RGB565(192,192,192)
#define C_SELECTED    RGB565(  0,  0,128)
#define C_SEL_TXT     RGB565(255,255,255)
#define C_TEXT        RGB565(  0,  0,  0)
#define C_FOLDER      RGB565(255,220,  0)
#define C_DOCWHITE    RGB565(255,255,255)
#define C_APPBLUE     RGB565(  0,128,255)
#define C_SCROLLBG    RGB565(168,168,168)

// Layout
#define TITLE_H   18
#define BORDER_W   2
#define TBAR_H    22
#define MBAR_H    18
#define FONT_W     8
#define FONT_H     8
#define ICON_W    40
#define ICON_H    40
#define ICON_LH   10

// Limits
#define MAX_WINS   8
#define MAX_ICONS 16
#define MAX_FILES 96

// Win hit zones
#define HIT_NONE      0
#define HIT_TITLEBAR  1
#define HIT_SYSMENU   2
#define HIT_CLOSE_BTN 3
#define HIT_MIN_BTN   4
#define HIT_CONTENT   5
#define HIT_TASKBAR   6
#define HIT_ICON      7
#define HIT_MENUBAR   8

// Window IDs
#define WID_PROGMAN 0
#define WID_FILEMGR 1

// ============================================================================
// DATA TYPES
// ============================================================================

typedef struct {
    int16_t x, y, w, h;
    char    title[48];
    uint8_t vis;          // 1=visible, 2=minimized
    int8_t  zorder;
    bool    dragging;
    int16_t drag_ox, drag_oy;
    void (*on_draw)(int wid);
    void (*on_click)(int wid, int lx, int ly, int btn);
    void (*on_key)(int wid, uint8_t k);
} win_t;

typedef struct {
    int16_t x, y;
    char    label[24];
    int     itype;        // 0=folder 1=doc 2=app 3=computer
    bool    selected;
    bool    dragging;
    int16_t drag_ox, drag_oy;
    void  (*on_open)(void);
} icon_t;

typedef struct {
    int16_t  x, y, w, h;
    int      edge;        // 0=bottom 1=top 2=left 3=right
    bool     dragging;
    int16_t  drag_ox, drag_oy;
} taskbar_t;

typedef struct {
    char path[256];
    char names[MAX_FILES][64];
    bool isdir[MAX_FILES];
    int  count;
    int  scroll;
    int  sel;
    char status[80];
    int  content_y;       // y offset of content area inside window
    int  item_h;          // height of each item row
} filebrowser_t;

typedef struct {
    win_t        wins[MAX_WINS];
    icon_t       icons[MAX_ICONS];
    int          nicn;
    taskbar_t    tbar;
    int          focus;       // focused window index, -1=none
    int16_t      mx, my;
    uint8_t      mbtns, mbtns_prev;
    uint32_t     last_lclick_tick;
    int16_t      last_lclick_x, last_lclick_y;
    bool         dragging_win;
    int          drag_win;
    bool         dragging_icon;
    int          drag_icon;
    bool         dragging_tbar;
    bool         running;
    filebrowser_t fb;
    uint32_t     frame;
    int          menu_open;   // -1=none, else window index
} os_t;

static os_t G;

// ============================================================================
// 8x8 BITMAP FONT  (ASCII 0x20-0x7F, index = char - 0x20)
// Bit 7 = leftmost pixel, 8 rows top-to-bottom
// ============================================================================

static const uint8_t FONT[96][8] = {
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // 20 space
    {0x18,0x3C,0x3C,0x18,0x18,0x00,0x18,0x00}, // 21 !
    {0x6C,0x6C,0x48,0x00,0x00,0x00,0x00,0x00}, // 22 "
    {0x6C,0x6C,0xFE,0x6C,0xFE,0x6C,0x6C,0x00}, // 23 #
    {0x10,0x7C,0x16,0x3C,0x68,0x7C,0x10,0x00}, // 24 $
    {0x00,0x66,0x34,0x18,0x2C,0x66,0x00,0x00}, // 25 %
    {0x38,0x6C,0x38,0x76,0xDC,0xCC,0x76,0x00}, // 26 &
    {0x18,0x18,0x0C,0x00,0x00,0x00,0x00,0x00}, // 27 '
    {0x18,0x0C,0x06,0x06,0x06,0x0C,0x18,0x00}, // 28 (
    {0x06,0x0C,0x18,0x18,0x18,0x0C,0x06,0x00}, // 29 )
    {0x00,0x66,0x3C,0xFF,0x3C,0x66,0x00,0x00}, // 2A *
    {0x00,0x18,0x18,0x7E,0x18,0x18,0x00,0x00}, // 2B +
    {0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x0C}, // 2C ,
    {0x00,0x00,0x00,0x7E,0x00,0x00,0x00,0x00}, // 2D -
    {0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00}, // 2E .
    {0x40,0x20,0x10,0x08,0x04,0x02,0x01,0x00}, // 2F /
    {0x3C,0x66,0x76,0x7E,0x6E,0x66,0x3C,0x00}, // 30 0
    {0x18,0x1C,0x18,0x18,0x18,0x18,0x7E,0x00}, // 31 1
    {0x3C,0x66,0x60,0x30,0x0C,0x06,0x7E,0x00}, // 32 2
    {0x3C,0x66,0x60,0x38,0x60,0x66,0x3C,0x00}, // 33 3
    {0x30,0x38,0x3C,0x36,0x7E,0x30,0x78,0x00}, // 34 4
    {0x7E,0x06,0x3E,0x60,0x60,0x66,0x3C,0x00}, // 35 5
    {0x38,0x0C,0x06,0x3E,0x66,0x66,0x3C,0x00}, // 36 6
    {0x7E,0x66,0x30,0x18,0x0C,0x0C,0x0C,0x00}, // 37 7
    {0x3C,0x66,0x66,0x3C,0x66,0x66,0x3C,0x00}, // 38 8
    {0x3C,0x66,0x66,0x7C,0x60,0x30,0x1C,0x00}, // 39 9
    {0x00,0x18,0x18,0x00,0x18,0x18,0x00,0x00}, // 3A :
    {0x00,0x18,0x18,0x00,0x18,0x18,0x0C,0x00}, // 3B ;
    {0x30,0x18,0x0C,0x06,0x0C,0x18,0x30,0x00}, // 3C <
    {0x00,0x00,0x7E,0x00,0x7E,0x00,0x00,0x00}, // 3D =
    {0x06,0x0C,0x18,0x30,0x18,0x0C,0x06,0x00}, // 3E >
    {0x3C,0x66,0x60,0x30,0x18,0x00,0x18,0x00}, // 3F ?
    {0x3C,0x66,0x76,0x76,0x76,0x06,0x3C,0x00}, // 40 @
    {0x18,0x3C,0x66,0x66,0x7E,0x66,0x66,0x00}, // 41 A
    {0x3E,0x66,0x66,0x3E,0x66,0x66,0x3E,0x00}, // 42 B
    {0x3C,0x66,0x06,0x06,0x06,0x66,0x3C,0x00}, // 43 C
    {0x1E,0x36,0x66,0x66,0x66,0x36,0x1E,0x00}, // 44 D
    {0x7E,0x06,0x06,0x3E,0x06,0x06,0x7E,0x00}, // 45 E
    {0x7E,0x06,0x06,0x3E,0x06,0x06,0x06,0x00}, // 46 F
    {0x3C,0x66,0x06,0x76,0x66,0x66,0x3C,0x00}, // 47 G
    {0x66,0x66,0x66,0x7E,0x66,0x66,0x66,0x00}, // 48 H
    {0x3C,0x18,0x18,0x18,0x18,0x18,0x3C,0x00}, // 49 I
    {0x78,0x30,0x30,0x30,0x36,0x36,0x1C,0x00}, // 4A J
    {0x66,0x36,0x1E,0x0E,0x1E,0x36,0x66,0x00}, // 4B K
    {0x06,0x06,0x06,0x06,0x06,0x06,0x7E,0x00}, // 4C L
    {0xC6,0xEE,0xFE,0xD6,0xC6,0xC6,0xC6,0x00}, // 4D M
    {0xC6,0xCE,0xDE,0xF6,0xE6,0xC6,0xC6,0x00}, // 4E N
    {0x3C,0x66,0x66,0x66,0x66,0x66,0x3C,0x00}, // 4F O
    {0x3E,0x66,0x66,0x3E,0x06,0x06,0x06,0x00}, // 50 P
    {0x3C,0x66,0x66,0x66,0x76,0x3C,0x60,0x00}, // 51 Q
    {0x3E,0x66,0x66,0x3E,0x1E,0x36,0x66,0x00}, // 52 R
    {0x3C,0x66,0x06,0x3C,0x60,0x66,0x3C,0x00}, // 53 S
    {0x7E,0x18,0x18,0x18,0x18,0x18,0x18,0x00}, // 54 T
    {0x66,0x66,0x66,0x66,0x66,0x66,0x3C,0x00}, // 55 U
    {0x66,0x66,0x66,0x66,0x66,0x3C,0x18,0x00}, // 56 V
    {0xC6,0xC6,0xC6,0xD6,0xFE,0xEE,0xC6,0x00}, // 57 W
    {0x66,0x66,0x3C,0x18,0x3C,0x66,0x66,0x00}, // 58 X
    {0x66,0x66,0x3C,0x18,0x18,0x18,0x18,0x00}, // 59 Y
    {0x7E,0x60,0x30,0x18,0x0C,0x06,0x7E,0x00}, // 5A Z
    {0x1E,0x06,0x06,0x06,0x06,0x06,0x1E,0x00}, // 5B [
    {0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x00}, // 5C backslash
    {0x1E,0x18,0x18,0x18,0x18,0x18,0x1E,0x00}, // 5D ]
    {0x18,0x3C,0x66,0x00,0x00,0x00,0x00,0x00}, // 5E ^
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF}, // 5F _
    {0x18,0x18,0x30,0x00,0x00,0x00,0x00,0x00}, // 60 `
    {0x00,0x00,0x3C,0x60,0x7C,0x66,0x7C,0x00}, // 61 a
    {0x06,0x06,0x3E,0x66,0x66,0x66,0x3E,0x00}, // 62 b
    {0x00,0x00,0x3C,0x06,0x06,0x06,0x3C,0x00}, // 63 c
    {0x60,0x60,0x7C,0x66,0x66,0x66,0x7C,0x00}, // 64 d
    {0x00,0x00,0x3C,0x66,0x7E,0x06,0x3C,0x00}, // 65 e
    {0x38,0x0C,0x0C,0x3E,0x0C,0x0C,0x0C,0x00}, // 66 f
    {0x00,0x00,0x7C,0x66,0x66,0x7C,0x60,0x3C}, // 67 g
    {0x06,0x06,0x3E,0x66,0x66,0x66,0x66,0x00}, // 68 h
    {0x18,0x00,0x1E,0x18,0x18,0x18,0x3C,0x00}, // 69 i
    {0x30,0x00,0x3C,0x30,0x30,0x30,0x36,0x1C}, // 6A j
    {0x06,0x06,0x66,0x36,0x1E,0x36,0x66,0x00}, // 6B k
    {0x1E,0x18,0x18,0x18,0x18,0x18,0x3C,0x00}, // 6C l
    {0x00,0x00,0x6C,0xFE,0xD6,0xC6,0xC6,0x00}, // 6D m
    {0x00,0x00,0x3E,0x66,0x66,0x66,0x66,0x00}, // 6E n
    {0x00,0x00,0x3C,0x66,0x66,0x66,0x3C,0x00}, // 6F o
    {0x00,0x00,0x3E,0x66,0x66,0x3E,0x06,0x06}, // 70 p
    {0x00,0x00,0x7C,0x66,0x66,0x7C,0x60,0x60}, // 71 q
    {0x00,0x00,0x3E,0x66,0x06,0x06,0x06,0x00}, // 72 r
    {0x00,0x00,0x3C,0x06,0x3C,0x60,0x3E,0x00}, // 73 s
    {0x0C,0x0C,0x3E,0x0C,0x0C,0x0C,0x38,0x00}, // 74 t
    {0x00,0x00,0x66,0x66,0x66,0x66,0x7C,0x00}, // 75 u
    {0x00,0x00,0x66,0x66,0x66,0x3C,0x18,0x00}, // 76 v
    {0x00,0x00,0xC6,0xC6,0xD6,0xFE,0x6C,0x00}, // 77 w
    {0x00,0x00,0x66,0x3C,0x18,0x3C,0x66,0x00}, // 78 x
    {0x00,0x00,0x66,0x66,0x66,0x7C,0x60,0x3C}, // 79 y
    {0x00,0x00,0x7E,0x30,0x18,0x0C,0x7E,0x00}, // 7A z
    {0x38,0x0C,0x0C,0x07,0x0C,0x0C,0x38,0x00}, // 7B {
    {0x18,0x18,0x18,0x00,0x18,0x18,0x18,0x00}, // 7C |
    {0x07,0x0C,0x0C,0x38,0x0C,0x0C,0x07,0x00}, // 7D }
    {0x6E,0x3B,0x00,0x00,0x00,0x00,0x00,0x00}, // 7E ~
    {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}, // 7F block
};

// Arrow cursor: 0=transparent 1=white 2=black
static const uint8_t CURSOR[16][12] = {
    {2,0,0,0,0,0,0,0,0,0,0,0},
    {2,2,0,0,0,0,0,0,0,0,0,0},
    {2,1,2,0,0,0,0,0,0,0,0,0},
    {2,1,1,2,0,0,0,0,0,0,0,0},
    {2,1,1,1,2,0,0,0,0,0,0,0},
    {2,1,1,1,1,2,0,0,0,0,0,0},
    {2,1,1,1,1,1,2,0,0,0,0,0},
    {2,1,1,1,1,1,1,2,0,0,0,0},
    {2,1,1,1,1,1,1,1,2,0,0,0},
    {2,1,1,1,2,2,2,2,0,0,0,0},
    {2,1,1,2,2,0,0,0,0,0,0,0},
    {2,1,2,0,2,1,2,0,0,0,0,0},
    {2,2,0,0,0,2,1,2,0,0,0,0},
    {0,0,0,0,0,0,2,2,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0},
};

// ============================================================================
// DRAWING PRIMITIVES
// ============================================================================

static inline void px(uint16_t *fb, int x, int y, uint16_t c) {
    if ((unsigned)x < OS_W && (unsigned)y < OS_H)
        fb[y * OS_W + x] = c;
}

static void fill_rect(uint16_t *fb, int x, int y, int w, int h, uint16_t c) {
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > OS_W) w = OS_W - x;
    if (y + h > OS_H) h = OS_H - y;
    if (w <= 0 || h <= 0) return;
    for (int ry = y; ry < y + h; ry++) {
        uint16_t *p = fb + ry * OS_W + x;
        for (int i = 0; i < w; i++) *p++ = c;
    }
}

static void hline(uint16_t *fb, int x, int y, int w, uint16_t c) {
    fill_rect(fb, x, y, w, 1, c);
}
static void vline(uint16_t *fb, int x, int y, int h, uint16_t c) {
    fill_rect(fb, x, y, 1, h, c);
}

static void draw_rect(uint16_t *fb, int x, int y, int w, int h, uint16_t c) {
    hline(fb, x, y, w, c);
    hline(fb, x, y+h-1, w, c);
    vline(fb, x, y, h, c);
    vline(fb, x+w-1, y, h, c);
}

static void draw_char(uint16_t *fb, int x, int y, char ch, uint16_t fg, uint16_t bg) {
    int idx = (unsigned char)ch;
    if (idx < 0x20 || idx > 0x7F) idx = 0x20;
    const uint8_t *g = FONT[idx - 0x20];
    for (int row = 0; row < FONT_H; row++) {
        int py = y + row;
        if (py < 0 || py >= OS_H) continue;
        uint8_t bits = g[row];
        for (int col = 0; col < FONT_W; col++) {
            int ppx = x + col;
            if (ppx < 0 || ppx >= OS_W) continue;
            fb[py * OS_W + ppx] = (bits & (0x80u >> col)) ? fg : bg;
        }
    }
}

static void draw_str(uint16_t *fb, int x, int y, const char *s,
                     uint16_t fg, uint16_t bg) {
    while (*s) {
        draw_char(fb, x, y, *s++, fg, bg);
        x += FONT_W;
    }
}

// Draw string, only opaque foreground pixels (transparent background)
static void draw_str_t(uint16_t *fb, int x, int y, const char *s, uint16_t fg) {
    while (*s) {
        int idx = (unsigned char)*s;
        if (idx >= 0x20 && idx <= 0x7F) {
            const uint8_t *g = FONT[idx - 0x20];
            for (int row = 0; row < FONT_H; row++) {
                uint8_t bits = g[row];
                if (!bits) continue;
                int py = y + row;
                if ((unsigned)py >= OS_H) continue;
                for (int col = 0; col < FONT_W; col++) {
                    if (bits & (0x80u >> col)) {
                        int ppx = x + col;
                        if ((unsigned)ppx < OS_W)
                            fb[py * OS_W + ppx] = fg;
                    }
                }
            }
        }
        x += FONT_W;
        s++;
    }
}

static int str_w(const char *s) { return (int)strlen(s) * FONT_W; }

// ============================================================================
// WIDGET DRAWING
// ============================================================================

// Raised 3D border (button face): outer black, inner hilite/shadow
static void raised_border(uint16_t *fb, int x, int y, int w, int h) {
    // outermost: dark
    hline(fb, x, y, w, C_DKSHADOW);
    vline(fb, x, y, h, C_DKSHADOW);
    hline(fb, x, y+h-1, w, C_DKSHADOW);
    vline(fb, x+w-1, y, h, C_DKSHADOW);
    // inner top/left: highlight
    hline(fb, x+1, y+1, w-2, C_HILIGHT);
    vline(fb, x+1, y+1, h-2, C_HILIGHT);
    // inner bottom/right: shadow
    hline(fb, x+1, y+h-2, w-2, C_SHADOW);
    vline(fb, x+w-2, y+1, h-2, C_SHADOW);
}

// Sunken 3D border (inset panel)
static void sunken_border(uint16_t *fb, int x, int y, int w, int h) {
    hline(fb, x, y, w, C_SHADOW);
    vline(fb, x, y, h, C_SHADOW);
    hline(fb, x, y+h-1, w, C_HILIGHT);
    vline(fb, x+w-1, y, h, C_HILIGHT);
    hline(fb, x+1, y+1, w-2, C_DKSHADOW);
    vline(fb, x+1, y+1, h-2, C_DKSHADOW);
    hline(fb, x+1, y+h-2, w-2, C_HILIGHT);
    vline(fb, x+w-2, y+1, h-2, C_HILIGHT);
}

// Draw a raised button with centered text label
static void draw_button(uint16_t *fb, int x, int y, int w, int h,
                        const char *label, bool pressed) {
    fill_rect(fb, x, y, w, h, C_WIN_BG);
    if (pressed) {
        sunken_border(fb, x, y, w, h);
    } else {
        raised_border(fb, x, y, w, h);
    }
    int tx = x + (w - str_w(label)) / 2 + (pressed ? 1 : 0);
    int ty = y + (h - FONT_H) / 2 + (pressed ? 1 : 0);
    draw_str(fb, tx, ty, label, C_TEXT, C_WIN_BG);
}

// Scrollbar track + thumb
static void draw_vscrollbar(uint16_t *fb, int x, int y, int h,
                             int total, int visible, int offset) {
    int w = 12;
    // Track
    fill_rect(fb, x, y, w, h, C_SCROLLBG);
    // Up arrow button
    draw_button(fb, x, y, w, 12, "\x1e", false);  // up triangle glyph
    // Down arrow button
    draw_button(fb, x, y+h-12, w, 12, "\x1f", false);
    // Thumb
    if (total > visible) {
        int track_h = h - 24;
        int thumb_h = (visible * track_h) / total;
        if (thumb_h < 10) thumb_h = 10;
        int thumb_y = y + 12 + (offset * (track_h - thumb_h)) / (total - visible);
        fill_rect(fb, x, thumb_y, w, thumb_h, C_WIN_BG);
        raised_border(fb, x, thumb_y, w, thumb_h);
    }
}

// Draw a folder icon
static void draw_icon_folder(uint16_t *fb, int x, int y) {
    // Tab on folder
    fill_rect(fb, x+2, y+8,  12, 4, C_FOLDER);
    hline    (fb, x+2, y+8,  12, C_DKSHADOW);
    hline    (fb, x+2, y+11, 12, C_DKSHADOW);
    vline    (fb, x+2, y+8,   4, C_DKSHADOW);
    vline    (fb, x+14,y+8,   4, C_DKSHADOW);
    // Body
    fill_rect(fb, x+1, y+11, 26, 16, C_FOLDER);
    hline    (fb, x+1, y+11, 26, C_DKSHADOW);
    hline    (fb, x+1, y+26, 26, C_DKSHADOW);
    vline    (fb, x+1, y+11, 16, C_DKSHADOW);
    vline    (fb, x+27,y+11, 16, C_DKSHADOW);
    // Highlight on body
    hline    (fb, x+2, y+12, 24, C_HILIGHT);
    vline    (fb, x+2, y+12, 14, C_HILIGHT);
}

// Draw a document icon
static void draw_icon_doc(uint16_t *fb, int x, int y) {
    fill_rect(fb, x+5, y+3, 18, 24, C_DOCWHITE);
    // Outline
    hline    (fb, x+5, y+3, 14, C_DKSHADOW);
    vline    (fb, x+5, y+3, 24, C_DKSHADOW);
    hline    (fb, x+5, y+26,18, C_DKSHADOW);
    vline    (fb, x+23,y+3,  18+2, C_DKSHADOW);
    vline    (fb, x+19,y+3,   5, C_DKSHADOW);
    // Folded corner
    hline    (fb, x+19,y+7,   5, C_DKSHADOW);
    fill_rect(fb, x+19,y+3, 4, 4, C_WIN_BG);
    // Lines on page
    hline    (fb, x+8, y+11,12, C_SHADOW);
    hline    (fb, x+8, y+14,12, C_SHADOW);
    hline    (fb, x+8, y+17,10, C_SHADOW);
    hline    (fb, x+8, y+20, 8, C_SHADOW);
}

// Draw an application icon (blue grid)
static void draw_icon_app(uint16_t *fb, int x, int y) {
    fill_rect(fb, x+3, y+3, 24, 24, C_APPBLUE);
    draw_rect(fb, x+3, y+3, 24, 24, C_DKSHADOW);
    hline    (fb, x+3, y+14, 24, C_HILIGHT);
    vline    (fb, x+15,y+3,  24, C_HILIGHT);
    // 4 mini windows
    fill_rect(fb, x+5,  y+5,  8, 7, C_WHITE);
    fill_rect(fb, x+17, y+5,  8, 7, C_WHITE);
    fill_rect(fb, x+5,  y+17, 8, 7, C_WHITE);
    fill_rect(fb, x+17, y+17, 8, 7, C_WHITE);
}

// Draw a computer/monitor icon
static void draw_icon_computer(uint16_t *fb, int x, int y) {
    // Monitor body
    fill_rect(fb, x+3, y+2, 24, 18, C_WIN_BG);
    draw_rect(fb, x+3, y+2, 24, 18, C_DKSHADOW);
    // Screen
    fill_rect(fb, x+6, y+5, 18, 12, C_APPBLUE);
    // Neck
    fill_rect(fb, x+13,y+20, 4, 4, C_WIN_BG);
    draw_rect(fb, x+13,y+20, 4, 4, C_DKSHADOW);
    // Base
    fill_rect(fb, x+8, y+24,14, 3, C_WIN_BG);
    draw_rect(fb, x+8, y+24,14, 3, C_DKSHADOW);
}

// ============================================================================
// DESKTOP ICON RENDERING
// ============================================================================

static void draw_desktop_icon(uint16_t *fb, int idx) {
    icon_t *ic = &G.icons[idx];
    int x = ic->x, y = ic->y;

    // Draw icon graphic (28x28, centered in 40x40 cell, top portion)
    int gx = x + (ICON_W - 28) / 2;
    int gy = y;

    if (ic->selected) {
        fill_rect(fb, gx-1, gy-1, 30, 30, C_SELECTED);
    }

    switch (ic->itype) {
        case 0: draw_icon_folder  (fb, gx, gy); break;
        case 1: draw_icon_doc     (fb, gx, gy); break;
        case 2: draw_icon_app     (fb, gx, gy); break;
        case 3: draw_icon_computer(fb, gx, gy); break;
    }

    // Label below icon
    int lw = str_w(ic->label);
    int lx = x + (ICON_W - lw) / 2;
    int ly = y + 30;
    uint16_t lbg = ic->selected ? C_SELECTED : C_DESKTOP;
    uint16_t lfg = ic->selected ? C_SEL_TXT  : C_WHITE;
    fill_rect(fb, lx-1, ly, lw+2, FONT_H, lbg);
    draw_str(fb, lx, ly, ic->label, lfg, lbg);
}

// ============================================================================
// WINDOW FRAME
// ============================================================================

// Get the inner content rect for a window (below title+menubar)
static void win_content_rect(int wi, int *cx, int *cy, int *cw, int *ch) {
    win_t *w = &G.wins[wi];
    *cx = w->x + BORDER_W + 1;
    *cy = w->y + BORDER_W + TITLE_H + 2;
    *cw = w->w - (BORDER_W + 1) * 2;
    *ch = w->h - BORDER_W - TITLE_H - 2 - BORDER_W - 1;
}

static void draw_window(uint16_t *fb, int wi) {
    win_t *w = &G.wins[wi];
    if (!w->vis) return;

    int x = w->x, y = w->y, ww = w->w, wh = w->h;
    bool active = (wi == G.focus);

    // --- Window background ---
    fill_rect(fb, x, y, ww, wh, C_WIN_BG);

    // --- Outer 3D border ---
    // outermost dark line
    draw_rect(fb, x, y, ww, wh, C_DKSHADOW);
    // inner highlight (top, left)
    hline(fb, x+1, y+1, ww-2, C_HILIGHT);
    vline(fb, x+1, y+1, wh-2, C_HILIGHT);
    // inner shadow (bottom, right)
    hline(fb, x+1, y+wh-2, ww-2, C_SHADOW);
    vline(fb, x+ww-2, y+1, wh-2, C_SHADOW);

    // --- Title bar ---
    int ty = y + BORDER_W;
    int th = TITLE_H;
    uint16_t tcol = active ? C_TITLE_ACT : C_TITLE_INACT;
    fill_rect(fb, x + BORDER_W, ty, ww - BORDER_W*2, th, tcol);

    // Title text (leave room for sys-menu and close button)
    int tx_start = x + BORDER_W + 20;   // after sys-menu button
    int tx_end   = x + ww - BORDER_W - 34; // before min+close buttons
    int avail_chars = (tx_end - tx_start) / FONT_W;
    char title_buf[48];
    strncpy(title_buf, w->title, sizeof(title_buf)-1);
    title_buf[sizeof(title_buf)-1] = 0;
    if ((int)strlen(title_buf) > avail_chars && avail_chars > 3) {
        title_buf[avail_chars-3] = '.';
        title_buf[avail_chars-2] = '.';
        title_buf[avail_chars-1] = '.';
        title_buf[avail_chars]   = 0;
    }
    int tty = ty + (th - FONT_H) / 2;
    draw_str(fb, tx_start, tty, title_buf, C_TITLE_TXT, tcol);

    // --- System menu button (left of title) ---
    int sbx = x + BORDER_W + 2;
    int sby = ty + (th - 13) / 2;
    fill_rect(fb, sbx, sby, 14, 13, C_WIN_BG);
    raised_border(fb, sbx, sby, 14, 13);
    hline(fb, sbx+3, sby+6, 8, C_DKSHADOW); // dash

    // --- Close button [X] (rightmost) ---
    int cbx = x + ww - BORDER_W - 2 - 14;
    int cby = ty + (th - 14) / 2;
    fill_rect(fb, cbx, cby, 14, 14, C_WIN_BG);
    raised_border(fb, cbx, cby, 14, 14);
    for (int d = 2; d <= 10; d++) {
        px(fb, cbx + d, cby + d, C_DKSHADOW);
        px(fb, cbx + 11 - d + 2, cby + d, C_DKSHADOW);
    }

    // --- Minimize button (left of close) ---
    int mbx = cbx - 16;
    int mby = cby;
    fill_rect(fb, mbx, mby, 14, 14, C_WIN_BG);
    raised_border(fb, mbx, mby, 14, 14);
    hline(fb, mbx+3, mby+9, 8, C_DKSHADOW);

    // --- Content area sunken border ---
    int cx, cy, cw, ch;
    win_content_rect(wi, &cx, &cy, &cw, &ch);
    // sunken outline
    hline(fb, cx-1, cy-1, cw+2, C_SHADOW);
    vline(fb, cx-1, cy-1, ch+2, C_SHADOW);
    hline(fb, cx-1, cy+ch, cw+2, C_HILIGHT);
    vline(fb, cx+cw,cy-1, ch+2, C_HILIGHT);
    fill_rect(fb, cx, cy, cw, ch, C_WIN_BG);

    // --- Call window content renderer ---
    if (w->on_draw) w->on_draw(wi);
}

// ============================================================================
// PROGRAM MANAGER WINDOW CONTENT
// ============================================================================

static void pm_draw_icon(uint16_t *fb, int gx, int gy, int itype,
                         const char *lbl, bool sel) {
    uint16_t lbg = sel ? C_SELECTED : C_WIN_BG;
    uint16_t lfg = sel ? C_SEL_TXT  : C_TEXT;
    if (sel) fill_rect(fb, gx-1, gy-1, 28+2, 28+FONT_H+3, C_SELECTED);
    switch (itype) {
        case 0: draw_icon_folder  (fb, gx, gy); break;
        case 1: draw_icon_doc     (fb, gx, gy); break;
        case 2: draw_icon_app     (fb, gx, gy); break;
        case 3: draw_icon_computer(fb, gx, gy); break;
    }
    int lw = str_w(lbl);
    int lx = gx + (28 - lw) / 2;
    fill_rect(fb, lx-1, gy+29, lw+2, FONT_H, lbg);
    draw_str(fb, lx, gy+29, lbl, lfg, lbg);
}

static int pm_selected = 0; // which icon is selected in prog manager

static void progman_draw(int wi) {
    int cx, cy, cw, ch;
    win_content_rect(wi, &cx, &cy, &cw, &ch);
    uint16_t *fb = video_card_get_back_buffer();

    // Title bar inside window (sub-title for group)
    fill_rect(fb, cx, cy, cw, 14, C_TITLE_INACT);
    draw_str(fb, cx+4, cy+3, "Main Group", C_WHITE, C_TITLE_INACT);

    // Icons in 3-column grid
    struct { int t; const char *n; } items[] = {
        {0, "File Mgr"},
        {3, "My Computer"},
        {2, "Programs"},
        {1, "Notepad"},
        {2, "Settings"},
        {1, "Readme"},
    };
    int ncols = 3;
    int cell_w = (cw - 10) / ncols;
    int cell_h = 50;
    int start_y = cy + 18;
    int n = (int)(sizeof(items)/sizeof(items[0]));
    for (int i = 0; i < n; i++) {
        int col = i % ncols;
        int row = i / ncols;
        int gx = cx + 5 + col * cell_w + (cell_w - 28) / 2;
        int gy = start_y + row * cell_h + 2;
        if (gy + 40 > cy + ch) break;
        pm_draw_icon(fb, gx, gy, items[i].t, items[i].n, pm_selected == i);
    }
}

static void progman_click(int wi, int lx, int ly, int btn) {
    int cx, cy, cw, ch;
    win_content_rect(wi, &cx, &cy, &cw, &ch);

    struct { int t; const char *n; } items[] = {
        {0, "File Mgr"},
        {3, "My Computer"},
        {2, "Programs"},
        {1, "Notepad"},
        {2, "Settings"},
        {1, "Readme"},
    };
    int ncols = 3;
    int cell_w = (cw - 10) / ncols;
    int cell_h = 50;
    int start_y = cy + 18;
    int n = (int)(sizeof(items)/sizeof(items[0]));

    for (int i = 0; i < n; i++) {
        int col = i % ncols;
        int row = i / ncols;
        int gx = cx + 5 + col * cell_w;
        int gy = start_y + row * cell_h;
        if (lx >= gx && lx < gx + cell_w &&
            ly >= gy && ly < gy + cell_h) {
            if (pm_selected == i && btn == 1) {
                // Double-click opens
                if (i == 0) {
                    // Open File Manager window
                    G.wins[WID_FILEMGR].vis = 1;
                    G.focus = WID_FILEMGR;
                }
            } else {
                pm_selected = i;
            }
            return;
        }
    }
}

// ============================================================================
// FILE BROWSER WINDOW CONTENT
// ============================================================================

static void fb_load_dir(const char *path) {
    filebrowser_t *f = &G.fb;
    strncpy(f->path, path, sizeof(f->path)-1);
    f->path[sizeof(f->path)-1] = 0;
    f->count = 0;
    f->scroll = 0;
    f->sel = -1;

    // Add ".." parent entry unless at sdcard root
    bool at_root = (strcmp(path, "/sdcard") == 0 ||
                    strcmp(path, "/sdcard/") == 0);
    if (!at_root) {
        strcpy(f->names[0], "..");
        f->isdir[0] = true;
        f->count = 1;
    }

    DIR *dir = opendir(path);
    if (!dir) {
        snprintf(f->status, sizeof(f->status), "Cannot open: %s", path);
        return;
    }

    struct dirent *ent;
    while ((ent = readdir(dir)) != NULL && f->count < MAX_FILES) {
        if (ent->d_name[0] == '.') continue;
        strncpy(f->names[f->count], ent->d_name, 63);
        f->names[f->count][63] = 0;
        f->isdir[f->count] = (ent->d_type == DT_DIR);
        f->count++;
    }
    closedir(dir);

    // Sort: directories first, then files
    for (int i = 0; i < f->count - 1; i++) {
        for (int j = i+1; j < f->count; j++) {
            bool swap = false;
            if (f->isdir[j] && !f->isdir[i]) swap = true;
            else if (f->isdir[i] == f->isdir[j] &&
                     strcasecmp(f->names[i], f->names[j]) > 0) swap = true;
            if (swap) {
                char tmp[64]; memcpy(tmp, f->names[i], 64);
                memcpy(f->names[i], f->names[j], 64);
                memcpy(f->names[j], tmp, 64);
                bool bd = f->isdir[i]; f->isdir[i] = f->isdir[j]; f->isdir[j] = bd;
            }
        }
    }

    snprintf(f->status, sizeof(f->status), "%d item(s) in %s", f->count, path);
}

static void filebrowser_draw(int wi) {
    int cx, cy, cw, ch;
    win_content_rect(wi, &cx, &cy, &cw, &ch);
    uint16_t *fb = video_card_get_back_buffer();
    filebrowser_t *f = &G.fb;

    // Address bar at top
    int addr_h = 14;
    fill_rect(fb, cx, cy, cw, addr_h, C_WIN_BG);
    sunken_border(fb, cx, cy, cw - 14, addr_h);
    // Path text clipped to bar
    char path_disp[64];
    int max_chars = (cw - 14 - 4) / FONT_W;
    if (max_chars < 1) max_chars = 1;
    strncpy(path_disp, f->path, sizeof(path_disp)-1);
    if ((int)strlen(path_disp) > max_chars) {
        // Show last max_chars chars
        const char *start = f->path + strlen(f->path) - max_chars;
        path_disp[0] = '.'; path_disp[1] = '.';
        strncpy(path_disp+2, start, max_chars-2);
        path_disp[max_chars] = 0;
    }
    draw_str(fb, cx+3, cy+3, path_disp, C_TEXT, C_WIN_BG);

    // Up-directory button
    draw_button(fb, cx + cw - 12, cy, 12, addr_h, "^", false);

    // File list area
    int list_y = cy + addr_h + 2;
    int list_h = ch - addr_h - 2 - 12; // reserve bottom status bar
    int item_h = FONT_H + 3;
    int visible_items = list_h / item_h;
    int sb_w = 12;
    int list_w = cw - sb_w;

    // List background
    fill_rect(fb, cx, list_y, list_w, list_h, C_WHITE);
    sunken_border(fb, cx, list_y, list_w, list_h);

    // Draw items
    for (int i = 0; i < visible_items; i++) {
        int idx = i + f->scroll;
        if (idx >= f->count) break;
        int iy = list_y + 1 + i * item_h;
        bool sel = (idx == f->sel);
        uint16_t ibg = sel ? C_SELECTED : C_WHITE;
        uint16_t ifg = sel ? C_SEL_TXT  : C_TEXT;
        fill_rect(fb, cx+1, iy, list_w-2, item_h-1, ibg);
        // Icon: small dir or file indicator
        uint16_t icol = f->isdir[idx] ? C_FOLDER : C_SHADOW;
        fill_rect(fb, cx+3, iy+2, 8, 8, icol);
        draw_rect(fb, cx+3, iy+2, 8, 8, C_DKSHADOW);
        if (f->isdir[idx]) {
            // folder tab hint
            hline(fb, cx+3, iy+1, 4, icol);
        }
        // Name
        char nm[48];
        strncpy(nm, f->names[idx], sizeof(nm)-1);
        if (f->isdir[idx] && strcmp(nm, "..") != 0) {
            // append trailing slash
            int nl = strlen(nm);
            if (nl < 47) { nm[nl] = '/'; nm[nl+1] = 0; }
        }
        draw_str(fb, cx+14, iy+2, nm, ifg, ibg);
    }

    // Scrollbar
    draw_vscrollbar(fb, cx + list_w, list_y, list_h,
                    f->count, visible_items, f->scroll);

    // Status bar
    int sb_y = cy + ch - 12;
    fill_rect(fb, cx, sb_y, cw, 12, C_WIN_BG);
    sunken_border(fb, cx, sb_y, cw, 12);
    draw_str(fb, cx+3, sb_y+2, f->status, C_TEXT, C_WIN_BG);

    // Store for click handling
    f->content_y = list_y;
    f->item_h    = item_h;
}

static void filebrowser_navigate(const char *name) {
    filebrowser_t *f = &G.fb;
    if (strcmp(name, "..") == 0) {
        // Go up
        char parent[256];
        strncpy(parent, f->path, sizeof(parent)-1);
        char *last = strrchr(parent, '/');
        if (last && last != parent) {
            *last = 0;
        } else {
            strcpy(parent, "/sdcard");
        }
        fb_load_dir(parent);
    } else {
        char newpath[256];
        snprintf(newpath, sizeof(newpath), "%s/%s", f->path, name);
        fb_load_dir(newpath);
    }
}

static void filebrowser_click(int wi, int lx, int ly, int btn) {
    int cx, cy, cw, ch;
    win_content_rect(wi, &cx, &cy, &cw, &ch);
    filebrowser_t *f = &G.fb;
    int addr_h = 14;

    // Up button click
    if (lx >= cx + cw - 12 && lx < cx + cw &&
        ly >= cy && ly < cy + addr_h) {
        filebrowser_navigate("..");
        return;
    }

    // List area click
    int list_y = f->content_y;
    int item_h = f->item_h;
    if (item_h < 1) item_h = FONT_H + 3;
    int list_w = cw - 12;

    if (ly >= list_y && lx >= cx && lx < cx + list_w) {
        int rel_y = ly - list_y - 1;
        int idx = (rel_y / item_h) + f->scroll;
        if (idx >= 0 && idx < f->count) {
            uint32_t now = xTaskGetTickCount();
            bool dbl = (f->sel == idx) && btn == 1 &&
                       ((now - G.last_lclick_tick) < pdMS_TO_TICKS(500));
            if (dbl && f->isdir[idx]) {
                filebrowser_navigate(f->names[idx]);
            } else {
                f->sel = idx;
            }
        }
    }
}

static void filebrowser_key(int wi, uint8_t k) {
    filebrowser_t *f = &G.fb;
    // Arrow keys: PS/2 / USB return special codes
    if (k == 0x1B) { // ESC -> go up
        filebrowser_navigate("..");
    } else if (k == '\r' || k == '\n') {
        if (f->sel >= 0 && f->sel < f->count && f->isdir[f->sel]) {
            filebrowser_navigate(f->names[f->sel]);
        }
    } else if (k == 0x80) { // UP arrow (common mapping)
        if (f->sel > 0) f->sel--;
        if (f->sel < f->scroll) f->scroll = f->sel;
    } else if (k == 0x81) { // DOWN arrow
        if (f->sel < f->count - 1) f->sel++;
    }
}

// ============================================================================
// TASKBAR
// ============================================================================

static void draw_taskbar(uint16_t *fb) {
    taskbar_t *tb = &G.tbar;
    int x = tb->x, y = tb->y, w = tb->w, h = tb->h;

    fill_rect(fb, x, y, w, h, C_TASKBAR);
    hline(fb, x, y,   w, C_HILIGHT);
    hline(fb, x, y+h-1, w, C_DKSHADOW);

    // MENU/Start button
    draw_button(fb, x+2, y+2, 48, h-4, "Menu", false);

    // Task buttons for visible windows
    int bx = x + 54;
    int bw = 82;
    int bh = h - 4;
    for (int i = 0; i < MAX_WINS; i++) {
        win_t *wn = &G.wins[i];
        if (!wn->vis) continue;
        if (bx + bw > x + w - 64) break;
        bool act = (i == G.focus);
        draw_button(fb, bx, y+2, bw, bh, wn->title, act);
        bx += bw + 2;
    }

    // Clock
    time_t now = time(NULL);
    struct tm *t = localtime(&now);
    char clk[10];
    snprintf(clk, sizeof(clk), "%02d:%02d", t->tm_hour, t->tm_min);
    int cw_px = str_w(clk) + 6;
    int clk_x = x + w - cw_px - 2;
    sunken_border(fb, clk_x, y+2, cw_px, h-4);
    draw_str(fb, clk_x+3, y+2+(h-4-FONT_H)/2, clk, C_TEXT, C_WIN_BG);
}

// ============================================================================
// WINDOW MANAGER
// ============================================================================

static void wm_bring_to_front(int wi) {
    G.focus = wi;
    // Re-sort z-orders: bump this window to top
    int8_t max_z = 0;
    for (int i = 0; i < MAX_WINS; i++) {
        if (G.wins[i].vis && G.wins[i].zorder > max_z)
            max_z = G.wins[i].zorder;
    }
    G.wins[wi].zorder = max_z + 1;
}

// Find which window is topmost at (x, y)
// Returns window index or -1
static int wm_window_at(int x, int y) {
    int best = -1;
    int8_t best_z = -1;
    for (int i = 0; i < MAX_WINS; i++) {
        win_t *w = &G.wins[i];
        if (!w->vis) continue;
        if (x >= w->x && x < w->x + w->w &&
            y >= w->y && y < w->y + w->h) {
            if (w->zorder > best_z) {
                best_z = w->zorder;
                best = i;
            }
        }
    }
    return best;
}

// Hit test within a window: returns HIT_* zone
static int wm_hit_zone(int wi, int x, int y,
                        int *rel_x, int *rel_y) {
    win_t *w = &G.wins[wi];
    int lx = x - w->x;
    int ly = y - w->y;
    if (rel_x) *rel_x = lx;
    if (rel_y) *rel_y = ly;

    // Title bar area
    if (ly >= BORDER_W && ly < BORDER_W + TITLE_H) {
        // Close button
        int cbx = w->w - BORDER_W - 2 - 14;
        int cby = BORDER_W + (TITLE_H - 14) / 2;
        if (lx >= cbx && lx < cbx+14 && ly >= cby && ly < cby+14)
            return HIT_CLOSE_BTN;
        // Minimize button
        int mbx = cbx - 16;
        if (lx >= mbx && lx < mbx+14 && ly >= cby && ly < cby+14)
            return HIT_MIN_BTN;
        // System menu
        int sbx = BORDER_W + 2;
        int sby = BORDER_W + (TITLE_H - 13) / 2;
        if (lx >= sbx && lx < sbx+14 && ly >= sby && ly < sby+13)
            return HIT_SYSMENU;
        return HIT_TITLEBAR;
    }
    return HIT_CONTENT;
}

// Close a window
static void wm_close(int wi) {
    G.wins[wi].vis = 0;
    if (G.focus == wi) G.focus = -1;
    // Focus next visible window
    for (int i = 0; i < MAX_WINS; i++) {
        if (G.wins[i].vis) { G.focus = i; break; }
    }
}

// ============================================================================
// INPUT HANDLING
// ============================================================================

static uint8_t read_key(void) {
    uint8_t k = 0;
    if (ps2_keyboard_is_initialized() && ps2_keyboard_available())
        k = ps2_keyboard_read();
    if (!k && usb_keyboard_is_initialized() && usb_keyboard_available())
        k = usb_keyboard_read();
    return k;
}

static void handle_keyboard(uint8_t k) {
    if (!k) return;
    // ESC exits the OS
    if (k == 0x1B) { G.running = false; return; }
    // Dispatch to focused window
    if (G.focus >= 0 && G.focus < MAX_WINS && G.wins[G.focus].on_key)
        G.wins[G.focus].on_key(G.focus, k);
}

static void handle_mouse(void) {
    usb_mouse_report_t rep;
    if (!usb_mouse_available()) return;
    if (!usb_mouse_read(&rep)) return;

    G.mbtns_prev = G.mbtns;
    G.mbtns = rep.buttons;

    // Update cursor
    int nx = (int)G.mx + rep.dx;
    int ny = (int)G.my + rep.dy;
    if (nx < 0) nx = 0;
    if (ny < 0) ny = 0;
    if (nx >= OS_W) nx = OS_W - 1;
    if (ny >= OS_H) ny = OS_H - 1;
    G.mx = (int16_t)nx;
    G.my = (int16_t)ny;

    bool lb_down = (G.mbtns & 1) && !(G.mbtns_prev & 1);  // left button just pressed
    bool lb_up   = !(G.mbtns & 1) && (G.mbtns_prev & 1);  // just released
    bool rb_down = (G.mbtns & 2) && !(G.mbtns_prev & 2);

    // --- Dragging windows ---
    if (G.dragging_win && G.mbtns & 1) {
        win_t *w = &G.wins[G.drag_win];
        w->x = (int16_t)(G.mx - w->drag_ox);
        w->y = (int16_t)(G.my - w->drag_oy);
        // Clamp so title bar is always accessible
        if (w->y < 0) w->y = 0;
        if (w->y + TITLE_H > G.tbar.y) w->y = (int16_t)(G.tbar.y - TITLE_H);
        return;
    }
    if (lb_up && G.dragging_win) {
        G.dragging_win = false;
        return;
    }

    // --- Dragging icons ---
    if (G.dragging_icon && G.mbtns & 1) {
        icon_t *ic = &G.icons[G.drag_icon];
        ic->x = (int16_t)(G.mx - ic->drag_ox);
        ic->y = (int16_t)(G.my - ic->drag_oy);
        // Clamp to desktop area
        if (ic->x < 0) ic->x = 0;
        if (ic->y < 0) ic->y = 0;
        if (ic->x + ICON_W > OS_W) ic->x = (int16_t)(OS_W - ICON_W);
        if (ic->y + ICON_H + ICON_LH > G.tbar.y)
            ic->y = (int16_t)(G.tbar.y - ICON_H - ICON_LH);
        return;
    }
    if (lb_up && G.dragging_icon) {
        G.dragging_icon = false;
        return;
    }

    // --- Dragging taskbar ---
    if (G.dragging_tbar && G.mbtns & 1) {
        // Snap to nearest edge
        int half_w = OS_W / 2, half_h = OS_H / 2;
        int dist_b = OS_H - G.my;
        int dist_t = G.my;
        int dist_l = G.mx;
        int dist_r = OS_W - G.mx;
        int mn = dist_b;
        int edge = 0;
        if (dist_t < mn) { mn = dist_t; edge = 1; }
        if (dist_l < mn) { mn = dist_l; edge = 2; }
        if (dist_r < mn) { mn = dist_r; edge = 3; }
        G.tbar.edge = edge;
        if (edge == 0) { G.tbar.x=0; G.tbar.y=OS_H-TBAR_H; G.tbar.w=OS_W; G.tbar.h=TBAR_H; }
        else if (edge==1) { G.tbar.x=0; G.tbar.y=0; G.tbar.w=OS_W; G.tbar.h=TBAR_H; }
        else if (edge==2) { G.tbar.x=0; G.tbar.y=0; G.tbar.w=TBAR_H; G.tbar.h=OS_H; }
        else              { G.tbar.x=OS_W-TBAR_H; G.tbar.y=0; G.tbar.w=TBAR_H; G.tbar.h=OS_H; }
        return;
    }
    if (lb_up && G.dragging_tbar) {
        G.dragging_tbar = false;
        return;
    }

    if (!lb_down && !rb_down) return;

    // --- Left button press hit testing ---
    if (lb_down) {
        uint32_t now = xTaskGetTickCount();

        // Check taskbar first
        taskbar_t *tb = &G.tbar;
        if (G.mx >= tb->x && G.mx < tb->x + tb->w &&
            G.my >= tb->y && G.my < tb->y + tb->h) {
            // Check task buttons
            int bx = tb->x + 54;
            int bw = 82;
            bool hit_task = false;
            for (int i = 0; i < MAX_WINS; i++) {
                if (!G.wins[i].vis) continue;
                if (G.mx >= bx && G.mx < bx + bw) {
                    // Toggle focus
                    if (G.focus == i) {
                        G.wins[i].vis = 0; // minimize
                    } else {
                        G.wins[i].vis = 1;
                        wm_bring_to_front(i);
                    }
                    hit_task = true;
                    break;
                }
                bx += bw + 2;
                if (bx + bw > tb->x + tb->w - 64) break;
            }
            if (!hit_task) {
                // Drag taskbar
                G.dragging_tbar = true;
            }
            return;
        }

        // Check windows (topmost first)
        int wi = wm_window_at(G.mx, G.my);
        if (wi >= 0) {
            wm_bring_to_front(wi);
            int rlx, rly;
            int zone = wm_hit_zone(wi, G.mx, G.my, &rlx, &rly);
            switch (zone) {
            case HIT_CLOSE_BTN:
                wm_close(wi);
                break;
            case HIT_MIN_BTN:
                G.wins[wi].vis = 0; // minimize
                if (G.focus == wi) G.focus = -1;
                break;
            case HIT_TITLEBAR:
                G.dragging_win = true;
                G.drag_win = wi;
                G.wins[wi].drag_ox = (int16_t)(G.mx - G.wins[wi].x);
                G.wins[wi].drag_oy = (int16_t)(G.my - G.wins[wi].y);
                break;
            case HIT_CONTENT: {
                // Check double-click for window content
                if (G.wins[wi].on_click) {
                    bool dbl = (G.focus == wi) &&
                               ((now - G.last_lclick_tick) < pdMS_TO_TICKS(500)) &&
                               abs(G.mx - G.last_lclick_x) < 6 &&
                               abs(G.my - G.last_lclick_y) < 6;
                    (void)dbl;
                    G.wins[wi].on_click(wi, G.mx, G.my, dbl ? 1 : 0);
                }
                break;
            }
            default: break;
            }
            G.last_lclick_tick = now;
            G.last_lclick_x = G.mx;
            G.last_lclick_y = G.my;
            return;
        }

        // Check desktop icons
        for (int i = 0; i < G.nicn; i++) {
            icon_t *ic = &G.icons[i];
            if (G.mx >= ic->x && G.mx < ic->x + ICON_W &&
                G.my >= ic->y && G.my < ic->y + ICON_H + ICON_LH) {
                bool dbl = (ic->selected) &&
                           ((now - G.last_lclick_tick) < pdMS_TO_TICKS(500)) &&
                           abs(G.mx - G.last_lclick_x) < 10 &&
                           abs(G.my - G.last_lclick_y) < 10;
                // Deselect all others
                for (int j = 0; j < G.nicn; j++) G.icons[j].selected = false;
                ic->selected = true;
                if (dbl && ic->on_open) {
                    ic->on_open();
                } else {
                    // Start drag
                    G.dragging_icon = true;
                    G.drag_icon = i;
                    ic->drag_ox = (int16_t)(G.mx - ic->x);
                    ic->drag_oy = (int16_t)(G.my - ic->y);
                }
                G.last_lclick_tick = now;
                G.last_lclick_x = G.mx;
                G.last_lclick_y = G.my;
                return;
            }
        }

        // Click on desktop: deselect all icons
        for (int i = 0; i < G.nicn; i++) G.icons[i].selected = false;
        G.last_lclick_tick = now;
        G.last_lclick_x = G.mx;
        G.last_lclick_y = G.my;
    }
}

// ============================================================================
// RENDERING
// ============================================================================

static void draw_cursor(uint16_t *fb) {
    int cx = G.mx, cy = G.my;
    for (int row = 0; row < 16; row++) {
        for (int col = 0; col < 12; col++) {
            uint8_t v = CURSOR[row][col];
            if (!v) continue;
            int px_ = cx + col, py_ = cy + row;
            if ((unsigned)px_ < OS_W && (unsigned)py_ < OS_H) {
                fb[py_ * OS_W + px_] = (v == 1) ? C_WHITE : C_BLACK;
            }
        }
    }
}

// Render a frame
static void render_frame(void) {
    uint16_t *fb = video_card_get_back_buffer();

    // 1) Desktop background
    fill_rect(fb, 0, 0, OS_W, OS_H, C_DESKTOP);

    // 2) Desktop icons (under windows)
    for (int i = 0; i < G.nicn; i++)
        draw_desktop_icon(fb, i);

    // 3) Windows back-to-front (by z-order)
    for (int8_t z = 0; z <= 10; z++) {
        for (int i = 0; i < MAX_WINS; i++) {
            if (G.wins[i].vis && G.wins[i].zorder == z)
                draw_window(fb, i);
        }
    }

    // 4) Taskbar (always on top of windows)
    draw_taskbar(fb);

    // 5) Mouse cursor (topmost)
    draw_cursor(fb);

    video_card_present();
}

// ============================================================================
// ICON OPEN CALLBACKS
// ============================================================================

static void open_filemgr(void) {
    win_t *wfm = &G.wins[WID_FILEMGR];
    wfm->vis = 1;
    wm_bring_to_front(WID_FILEMGR);
    fb_load_dir("/sdcard");
}

static void open_progman(void) {
    G.wins[WID_PROGMAN].vis = 1;
    wm_bring_to_front(WID_PROGMAN);
}

// ============================================================================
// INITIALIZATION
// ============================================================================

static void os_init(void) {
    memset(&G, 0, sizeof(G));
    G.focus  = -1;
    G.mx     = OS_W / 2;
    G.my     = OS_H / 2;
    G.running = true;

    // --- Taskbar at bottom ---
    G.tbar.x = 0;
    G.tbar.y = OS_H - TBAR_H;
    G.tbar.w = OS_W;
    G.tbar.h = TBAR_H;
    G.tbar.edge = 0;

    // --- Desktop icons ---
    // File Manager
    G.icons[0].x = 10;
    G.icons[0].y = 10;
    G.icons[0].itype = 0;
    strcpy(G.icons[0].label, "File Mgr");
    G.icons[0].on_open = open_filemgr;
    // Program Manager
    G.icons[1].x = 10;
    G.icons[1].y = 64;
    G.icons[1].itype = 2;
    strcpy(G.icons[1].label, "Programs");
    G.icons[1].on_open = open_progman;
    // My Computer
    G.icons[2].x = 10;
    G.icons[2].y = 118;
    G.icons[2].itype = 3;
    strcpy(G.icons[2].label, "Computer");
    G.icons[2].on_open = NULL;
    G.nicn = 3;

    // --- Windows ---
    // Program Manager
    win_t *pm = &G.wins[WID_PROGMAN];
    pm->x = 50;
    pm->y = 20;
    pm->w = 320;
    pm->h = 220;
    strcpy(pm->title, "Program Manager");
    pm->vis    = 1;
    pm->zorder = 1;
    pm->on_draw  = progman_draw;
    pm->on_click = progman_click;

    // File Manager (hidden until opened)
    win_t *fm = &G.wins[WID_FILEMGR];
    fm->x = 60;
    fm->y = 30;
    fm->w = 380;
    fm->h = 260;
    strcpy(fm->title, "File Manager - /sdcard");
    fm->vis    = 0;
    fm->zorder = 2;
    fm->on_draw  = filebrowser_draw;
    fm->on_click = filebrowser_click;
    fm->on_key   = filebrowser_key;

    G.focus = WID_PROGMAN;

    // Init file browser path
    strcpy(G.fb.path, "/sdcard");
    G.fb.item_h = FONT_H + 3;
    fb_load_dir("/sdcard");

    ESP_LOGI(TAG, "ESP32 Native OS initialized. 480x320 desktop ready.");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void esp32_os_run(void *arg) {
    (void)arg;

    // Switch display to graphics mode
    video_card_enter_gfx_mode();

    os_init();

    // Draw a "booting..." splash for 1 second
    {
        uint16_t *fb = video_card_get_back_buffer();
        fill_rect(fb, 0, 0, OS_W, OS_H, C_TITLE_ACT);
        const char *msg  = "ESP32 Native OS";
        const char *msg2 = "Windows 3.1 Desktop Environment";
        const char *msg3 = "Starting...";
        draw_str(fb, (OS_W - str_w(msg))  / 2, 120, msg,  C_WHITE, C_TITLE_ACT);
        draw_str(fb, (OS_W - str_w(msg2)) / 2, 135, msg2, C_WHITE, C_TITLE_ACT);
        draw_str(fb, (OS_W - str_w(msg3)) / 2, 155, msg3, C_SHADOW, C_TITLE_ACT);
        video_card_present();
        vTaskDelay(pdMS_TO_TICKS(1200));
    }

    ESP_LOGI(TAG, "Entering desktop main loop. ESC to exit.");

    while (G.running) {
        handle_mouse();

        uint8_t k = read_key();
        handle_keyboard(k);

        render_frame();

        // ~30 fps
        vTaskDelay(pdMS_TO_TICKS(33));
        G.frame++;
    }

    ESP_LOGI(TAG, "ESP32 OS exiting.");
    // Switch back to text mode is handled by caller (video_card_enter_text_mode
    // not available, so we just leave it in gfx and let the next boot handle it)
}
