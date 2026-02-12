/**
 * @file i8086_emulator.c
 * @brief Intel 8086 PC Emulator - Complete Integration Layer
 *
 * Full IBM PC/XT emulation with:
 * - BIOS interrupt services (INT 08h-1Ah)
 * - DOS INT 21h with file I/O via ESP-IDF VFS (SD card)
 * - PC hardware: PIC 8259, PIT 8253, keyboard, RTC, CGA
 * - Video text mode output to LCD console
 * - Keyboard input from UART and PS/2
 * - Proper run loop with hardware tick processing
 *
 * Part of the ESP32-P4 Multi-CPU Emulator Project
 */

#include "i8086_emulator.h"
#include "i8086_cpu.h"
#include "i8086_pc.h"
#include "disk_image.h"
#include "video_card.h"
#include "ps2_keyboard.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <sys/stat.h>
#include <dirent.h>
#include <ctype.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"

static const char *TAG = "i8086";

// External console_printf from main
extern int console_printf(const char *fmt, ...);

// ============================================================================
// DOS File Handle Management
// ============================================================================

#define DOS_MAX_FILES       20
#define DOS_FIRST_HANDLE    5       // 0-4 are reserved (stdin/out/err/aux/prn)
#define DOS_PATH_MAX        256

typedef struct {
    FILE   *fp;
    char    path[DOS_PATH_MAX];
    bool    open;
    bool    read_only;
    uint8_t access_mode;    // 0=read, 1=write, 2=read/write
} dos_file_t;

typedef struct {
    // File handles
    dos_file_t files[DOS_MAX_FILES];

    // Current directory (maps to SD card path)
    char current_dir[DOS_PATH_MAX];     // DOS-style: C:/
    char current_path[DOS_PATH_MAX];    // Real path: /sdcard/

    // DOS error state
    uint16_t last_error;
    uint8_t  verify_flag;

    // DTA (Disk Transfer Area) for Find First/Next
    uint32_t dta_segment;
    uint16_t dta_offset;

    // Find state for FindFirst/FindNext
    DIR     *find_dir;
    char     find_pattern[DOS_PATH_MAX];
    char     find_dir_path[DOS_PATH_MAX];
    uint8_t  find_attrib;

    // PSP segment
    uint16_t psp_segment;

    // Program return code
    uint8_t  return_code;

    // Ctrl+C check flag
    bool     break_check;
} dos_state_t;

// ============================================================================
// Emulator State
// ============================================================================

static struct {
    bool initialized;
    i86_state_t state;
    i86_config_t config;
    i86_stats_t stats;
    TaskHandle_t task_handle;
    SemaphoreHandle_t mutex;

    // PC Hardware
    pc_hardware_t hw;

    // BIOS keyboard buffer (INT 16h)
    // Stored as (scancode << 8) | ascii, circular buffer at BDA 0x41E
    uint16_t bios_kbd_buffer[16];
    int bios_kbd_head;
    int bios_kbd_tail;

    // Video mode
    uint8_t video_mode;
    uint8_t cursor_x, cursor_y;
    uint8_t text_attr;
    uint8_t active_page;

    // Timer tick count (for INT 1Ah)
    uint32_t timer_ticks;
    bool     timer_midnight;

    // DOS state
    dos_state_t dos;

    // Video refresh tracking
    int64_t last_refresh_us;
    bool    video_dirty;

    // Run control
    volatile bool stop_requested;
} s_i86;

// ============================================================================
// BIOS Data Area (BDA) Addresses
// ============================================================================
#define BDA_BASE        0x0400
#define BDA_COM1        0x0400  // COM1 base address
#define BDA_COM2        0x0402
#define BDA_LPT1        0x0408
#define BDA_EQUIPMENT   0x0410  // Equipment word
#define BDA_MEMORY_SIZE 0x0413  // Memory size in KB
#define BDA_KBD_FLAGS1  0x0417  // Keyboard shift flags 1
#define BDA_KBD_FLAGS2  0x0418  // Keyboard shift flags 2
#define BDA_KBD_HEAD    0x041A  // Keyboard buffer head pointer
#define BDA_KBD_TAIL    0x041C  // Keyboard buffer tail pointer
#define BDA_KBD_BUFFER  0x041E  // 16-word keyboard buffer (0x41E-0x43D)
#define BDA_VIDEO_MODE  0x0449  // Current video mode
#define BDA_VIDEO_COLS  0x044A  // Number of columns
#define BDA_VIDEO_PAGE_SIZE 0x044C  // Video page size in bytes
#define BDA_VIDEO_PAGE_OFF  0x044E  // Offset of current video page
#define BDA_CURSOR_POS  0x0450  // Cursor positions for 8 pages (16 bytes)
#define BDA_CURSOR_TYPE 0x0460  // Cursor type (start/end scan lines)
#define BDA_ACTIVE_PAGE 0x0462  // Active display page
#define BDA_CRTC_PORT   0x0463  // CRTC base port
#define BDA_CRT_MODE    0x0465  // Current CRT mode register value
#define BDA_CRT_PALETTE 0x0466  // Current CRT palette
#define BDA_TIMER_COUNT 0x046C  // Timer tick count (4 bytes)
#define BDA_TIMER_OFLOW 0x0470  // Timer overflow (midnight)
#define BDA_KBD_FLAGS3  0x0496  // Enhanced keyboard flags
#define BDA_DISK_STATUS 0x0474  // Last disk operation status
#define BDA_DISK_COUNT  0x0475  // Number of hard disks
#define BDA_FLOPPY_STATUS 0x0441 // Diskette status
#define BDA_MOTOR_STATUS  0x043F // Diskette motor status
#define BDA_PRINT_TIMEOUT 0x0478 // Printer timeout values

// ============================================================================
// ASCII to XT Scancode mapping (for UART input)
// ============================================================================

// Map ASCII chars to XT scancodes
static const uint8_t ascii_to_scancode[128] = {
    0,0x1E,0x30,0x2E,0x20,0x12,0x21,0x22, //  0-7: NUL,^A-^G
    0x0E,0x0F,0x1C,0x25,0x26,0x1C,0x31,0x18, //  8-15: BS,TAB,LF,^K,^L,CR,^N,^O
    0x19,0x10,0x13,0x1F,0x14,0x16,0x2F,0x11, // 16-23: ^P-^W
    0x2D,0x15,0x2C,0x01,0x2B,0x1B,0x07,0x0C, // 24-31: ^X-^Z,ESC,^\,^],^^,^_
    0x39,0x02,0x28,0x04,0x05,0x06,0x08,0x28, // 32-39: SPC,!,"",#,$,%,&,'
    0x0A,0x0B,0x09,0x0D,0x33,0x0C,0x34,0x35, // 40-47: (,),*,+,,,-./ 
    0x0B,0x02,0x03,0x04,0x05,0x06,0x07,0x08, // 48-55: 0-7
    0x09,0x0A,0x27,0x27,0x33,0x0D,0x34,0x35, // 56-63: 8,9,:,;,<,=,>,?
    0x03,0x1E,0x30,0x2E,0x20,0x12,0x21,0x22, // 64-71: @,A-G
    0x23,0x17,0x24,0x25,0x26,0x32,0x31,0x18, // 72-79: H-O
    0x19,0x10,0x13,0x1F,0x14,0x16,0x2F,0x11, // 80-87: P-W
    0x2D,0x15,0x2C,0x1A,0x2B,0x1B,0x07,0x0C, // 88-95: X-Z,[,\,],^,_
    0x29,0x1E,0x30,0x2E,0x20,0x12,0x21,0x22, // 96-103: `,a-g
    0x23,0x17,0x24,0x25,0x26,0x32,0x31,0x18, // 104-111: h-o
    0x19,0x10,0x13,0x1F,0x14,0x16,0x2F,0x11, // 112-119: p-w
    0x2D,0x15,0x2C,0x1A,0x2B,0x1B,0x29,0x0E, // 120-127: x-z,{,|,},~,DEL
};

// ============================================================================
// Port I/O Handlers (dispatch to PC hardware)
// ============================================================================

static uint8_t port_read_handler(void *ctx, uint16_t port) {
    return pc_hw_read_port(&s_i86.hw, port);
}

static void port_write_handler(void *ctx, uint16_t port, uint8_t value) {
    pc_hw_write_port(&s_i86.hw, port, value);
}

// ============================================================================
// Video Memory Handlers
// ============================================================================

static void video_write8(void *ctx, uint32_t addr, uint8_t value) {
    uint8_t *mem = i86_cpu_get_memory();
    if (addr < I8086_TOTAL_MEMORY) {
        mem[addr] = value;
        // Mark video dirty if writing to text mode area
        if (addr >= 0xB8000 && addr < 0xB9000) {
            s_i86.video_dirty = true;
            s_i86.hw.cga.needs_refresh = true;
        }
    }
}

static void video_write16(void *ctx, uint32_t addr, uint16_t value) {
    video_write8(ctx, addr, value & 0xFF);
    video_write8(ctx, addr + 1, value >> 8);
}

static uint8_t video_read8(void *ctx, uint32_t addr) {
    uint8_t *mem = i86_cpu_get_memory();
    if (addr < I8086_TOTAL_MEMORY) return mem[addr];
    return 0;
}

static uint16_t video_read16(void *ctx, uint32_t addr) {
    return video_read8(ctx, addr) | (video_read8(ctx, addr + 1) << 8);
}

// ============================================================================
// Helper: Write BDA fields
// ============================================================================

static inline void bda_write8(uint16_t addr, uint8_t val) {
    i86_cpu_write_byte(addr, val);
}

static inline void bda_write16(uint16_t addr, uint16_t val) {
    i86_cpu_write_word(addr, val);
}

static inline uint8_t bda_read8(uint16_t addr) {
    return i86_cpu_read_byte(addr);
}

static inline uint16_t bda_read16(uint16_t addr) {
    return i86_cpu_read_word(addr);
}

// ============================================================================
// Helper: Console output (goes to both UART + LCD)
// ============================================================================

static void pc_putchar(char c) {
    // Write to CGA video RAM
    uint8_t *video = i86_cpu_get_video();
    if (!video) return;

    if (c == '\r') {
        s_i86.cursor_x = 0;
    } else if (c == '\n') {
        s_i86.cursor_y++;
    } else if (c == '\b') {
        if (s_i86.cursor_x > 0) {
            s_i86.cursor_x--;
            int offset = (s_i86.cursor_y * 80 + s_i86.cursor_x) * 2;
            video[0x18000 + offset] = 0x20;
        }
    } else if (c == '\t') {
        // Tab to next 8-column boundary
        s_i86.cursor_x = (s_i86.cursor_x + 8) & ~7;
        if (s_i86.cursor_x >= 80) {
            s_i86.cursor_x = 0;
            s_i86.cursor_y++;
        }
    } else if (c == 0x07) {
        // BEL - beep (ignore)
    } else if ((uint8_t)c >= 0x20) {
        int offset = (s_i86.cursor_y * 80 + s_i86.cursor_x) * 2;
        video[0x18000 + offset] = (uint8_t)c;
        video[0x18000 + offset + 1] = s_i86.text_attr;
        s_i86.cursor_x++;
        if (s_i86.cursor_x >= 80) {
            s_i86.cursor_x = 0;
            s_i86.cursor_y++;
        }
    }

    // Handle scroll
    if (s_i86.cursor_y >= 25) {
        // Scroll video RAM up one line
        memmove(video + 0x18000, video + 0x18000 + 160, 160 * 24);
        // Clear last line
        for (int i = 0; i < 80; i++) {
            video[0x18000 + 160 * 24 + i * 2] = 0x20;
            video[0x18000 + 160 * 24 + i * 2 + 1] = s_i86.text_attr;
        }
        s_i86.cursor_y = 24;
    }

    s_i86.video_dirty = true;

    // Also output to UART for debugging
    putchar(c);
}

static void __attribute__((unused)) pc_print(const char *str) {
    while (*str) {
        pc_putchar(*str++);
    }
}

// ============================================================================
// DOS Path Helpers
// ============================================================================

// Convert DOS path (C:\DIR\FILE.TXT) to VFS path (/sdcard/dir/file.txt)
static void dos_to_vfs_path(const char *dos_path, char *vfs_path, size_t size) {
    // Start with base SD card path
    if (dos_path[0] && dos_path[1] == ':') {
        // Has drive letter, skip it
        dos_path += 2;
    }

    if (dos_path[0] == '\\' || dos_path[0] == '/') {
        snprintf(vfs_path, size, "/sdcard%s", dos_path);
    } else {
        // Relative to current directory
        snprintf(vfs_path, size, "%s/%s", s_i86.dos.current_path, dos_path);
    }

    // Convert backslashes to forward slashes
    for (char *p = vfs_path; *p; p++) {
        if (*p == '\\') *p = '/';
    }

    // Convert to lowercase (DOS is case-insensitive)
    // But don't change the /sdcard prefix
    for (char *p = vfs_path + 7; *p; p++) {
        *p = tolower((unsigned char)*p);
    }
}

// Read a '$'-terminated or NUL-terminated string from emulator memory
static void read_dos_string(uint16_t seg, uint16_t off, char *buf, size_t maxlen, char term) {
    uint8_t *mem = i86_cpu_get_memory();
    uint32_t addr = (uint32_t)seg * 16 + off;
    size_t i = 0;
    while (i < maxlen - 1) {
        uint8_t c = mem[addr + i];
        if (c == (uint8_t)term || c == 0) break;
        buf[i++] = c;
    }
    buf[i] = '\0';
}

// Write a string to emulator memory
static void write_dos_string(uint16_t seg, uint16_t off, const char *str) {
    uint8_t *mem = i86_cpu_get_memory();
    uint32_t addr = (uint32_t)seg * 16 + off;
    while (*str) {
        mem[addr++] = *str++;
    }
    mem[addr] = '\0';
}

// Set carry flag (error indicator for DOS calls)
static void set_carry(bool set) {
    i86_cpu_set_flag(FLAG_IDX_CF, set ? 1 : 0);
}

// Forward declaration for poll_input (used by INT 16h wait loop)
static void poll_input(void);

// ============================================================================
// BIOS Interrupt Handlers
// ============================================================================

static bool handle_int08h(void) {
    // INT 08h - Timer tick (IRQ 0)
    s_i86.timer_ticks++;

    // Update BDA timer count
    uint8_t *mem = i86_cpu_get_memory();
    mem[BDA_TIMER_COUNT] = s_i86.timer_ticks & 0xFF;
    mem[BDA_TIMER_COUNT + 1] = (s_i86.timer_ticks >> 8) & 0xFF;
    mem[BDA_TIMER_COUNT + 2] = (s_i86.timer_ticks >> 16) & 0xFF;
    mem[BDA_TIMER_COUNT + 3] = (s_i86.timer_ticks >> 24) & 0xFF;

    // Check for midnight (1573040 ticks ≈ 24 hours)
    if (s_i86.timer_ticks >= 1573040) {
        s_i86.timer_ticks = 0;
        s_i86.timer_midnight = true;
        mem[BDA_TIMER_OFLOW] = 1;
    }

    // Chain to user timer hook (INT 1Ch)
    // The real BIOS does this, but we'll skip for simplicity
    // as most DOS programs set their own INT 1Ch handler

    // Send EOI to PIC
    pc_hw_write_port(&s_i86.hw, 0x20, 0x20);
    return true;
}

static bool handle_int09h(void) {
    // INT 09h - Keyboard interrupt (IRQ 1)
    // Read scancode from keyboard controller
    uint8_t scancode = pc_hw_read_port(&s_i86.hw, 0x60);

    // Process scancode into BIOS keyboard buffer
    bool is_break = (scancode & 0x80) != 0;
    uint8_t make_code = scancode & 0x7F;

    // Update shift flags
    switch (make_code) {
        case 0x2A: // Left Shift
            if (is_break)
                s_i86.hw.keyboard.shift_flags &= ~0x02;
            else
                s_i86.hw.keyboard.shift_flags |= 0x02;
            break;
        case 0x36: // Right Shift
            if (is_break)
                s_i86.hw.keyboard.shift_flags &= ~0x01;
            else
                s_i86.hw.keyboard.shift_flags |= 0x01;
            break;
        case 0x1D: // Ctrl
            if (is_break)
                s_i86.hw.keyboard.shift_flags &= ~0x04;
            else
                s_i86.hw.keyboard.shift_flags |= 0x04;
            break;
        case 0x38: // Alt
            if (is_break)
                s_i86.hw.keyboard.shift_flags &= ~0x08;
            else
                s_i86.hw.keyboard.shift_flags |= 0x08;
            break;
        case 0x3A: // Caps Lock
            if (!is_break)
                s_i86.hw.keyboard.shift_flags ^= 0x40;
            break;
        case 0x45: // Num Lock
            if (!is_break)
                s_i86.hw.keyboard.shift_flags ^= 0x20;
            break;
        case 0x46: // Scroll Lock
            if (!is_break)
                s_i86.hw.keyboard.shift_flags ^= 0x10;
            break;
    }

    // Update BDA shift flags
    bda_write8(BDA_KBD_FLAGS1, s_i86.hw.keyboard.shift_flags);

    // On key press (not release), add to BIOS keyboard buffer
    if (!is_break) {
        // Simple scancode → ASCII lookup (XT scancode set 1)
        static const char scancode_to_ascii_lower[] =
            "\0\x1B" "1234567890-=\b\t"  // 00-0F
            "qwertyuiop[]\r\0"           // 10-1D (1D=Ctrl)
            "asdfghjkl;'`\0\\"           // 1E-2B
            "zxcvbnm,./\0*\0 ";          // 2C-39

        static const char scancode_to_ascii_upper[] =
            "\0\x1B" "!@#$%^&*()_+\b\t"  // 00-0F
            "QWERTYUIOP{}\r\0"           // 10-1D
            "ASDFGHJKL:\"~\0|"           // 1E-2B
            "ZXCVBNM<>?\0*\0 ";          // 2C-39

        uint8_t ascii = 0;
        bool shift = (s_i86.hw.keyboard.shift_flags & 0x03) != 0;
        bool caps = (s_i86.hw.keyboard.shift_flags & 0x40) != 0;

        if (make_code < 0x3A) {
            ascii = shift ? scancode_to_ascii_upper[make_code]
                          : scancode_to_ascii_lower[make_code];

            // CapsLock affects letters only
            if (caps && ascii >= 'a' && ascii <= 'z') ascii -= 32;
            else if (caps && ascii >= 'A' && ascii <= 'Z') ascii += 32;
        }

        // Handle special keys
        switch (make_code) {
            case 0x01: ascii = 0x1B; break; // ESC
            case 0x0E: ascii = 0x08; break; // Backspace
            case 0x0F: ascii = 0x09; break; // Tab
            case 0x1C: ascii = 0x0D; break; // Enter
            case 0x39: ascii = 0x20; break; // Space
        }

        // Ctrl+key combinations
        if (s_i86.hw.keyboard.shift_flags & 0x04) {
            if (ascii >= 'a' && ascii <= 'z') ascii = ascii - 'a' + 1;
            else if (ascii >= 'A' && ascii <= 'Z') ascii = ascii - 'A' + 1;
        }

        // Add to BIOS keyboard buffer
        int next = (s_i86.bios_kbd_head + 1) % 16;
        if (next != s_i86.bios_kbd_tail) {
            s_i86.bios_kbd_buffer[s_i86.bios_kbd_head] = (make_code << 8) | ascii;
            s_i86.bios_kbd_head = next;
        }
    }

    // Send EOI to PIC
    pc_hw_write_port(&s_i86.hw, 0x20, 0x20);
    return true;
}

static bool handle_int10h(void) {
    // INT 10h - Video BIOS services
    uint8_t ah = (i86_cpu_get_ax() >> 8) & 0xFF;
    uint8_t al = i86_cpu_get_ax() & 0xFF;
    uint8_t bh = (i86_cpu_get_bx() >> 8) & 0xFF;
    uint8_t bl = i86_cpu_get_bx() & 0xFF;
    uint8_t dh = (i86_cpu_get_dx() >> 8) & 0xFF;
    uint8_t dl = i86_cpu_get_dx() & 0xFF;
    uint16_t cx = i86_cpu_get_cx();
    uint8_t ch_reg = (cx >> 8) & 0xFF;
    uint8_t cl_reg = cx & 0xFF;

    (void)bh; (void)ch_reg; (void)cl_reg;

    uint8_t *video = i86_cpu_get_video();
    if (!video) return true;

    switch (ah) {
        case 0x00: // Set video mode
            s_i86.video_mode = al & 0x7F;  // Bit 7 = don't clear
            if (!(al & 0x80)) {
                // Clear screen
                for (int i = 0; i < 4000; i += 2) {
                    video[0x18000 + i] = 0x20;
                    video[0x18000 + i + 1] = 0x07;
                }
                s_i86.cursor_x = 0;
                s_i86.cursor_y = 0;
            }
            s_i86.text_attr = 0x07;
            bda_write8(BDA_VIDEO_MODE, s_i86.video_mode);
            s_i86.video_dirty = true;
            ESP_LOGD(TAG, "INT 10h: Set video mode %02Xh", al);
            return true;

        case 0x01: // Set cursor shape
            bda_write16(BDA_CURSOR_TYPE, cx);
            return true;

        case 0x02: // Set cursor position
            s_i86.cursor_x = dl;
            s_i86.cursor_y = dh;
            // Update BDA cursor position for page bh
            bda_write16(BDA_CURSOR_POS + (bh * 2), (dh << 8) | dl);
            return true;

        case 0x03: // Get cursor position
            i86_cpu_set_dx((s_i86.cursor_y << 8) | s_i86.cursor_x);
            i86_cpu_set_cx(bda_read16(BDA_CURSOR_TYPE));
            return true;

        case 0x05: // Select active page
            s_i86.active_page = al;
            bda_write8(BDA_ACTIVE_PAGE, al);
            return true;

        case 0x06: // Scroll up
        {
            uint8_t lines = al;
            uint8_t attr = bl;  // BL not BH for scroll attribute
            // Actually BH = fill attribute for scroll
            attr = bh;
            uint8_t top_row = (cx >> 8) & 0xFF;
            uint8_t left_col = cx & 0xFF;
            uint8_t bot_row = (i86_cpu_get_dx() >> 8) & 0xFF;
            uint8_t right_col = i86_cpu_get_dx() & 0xFF;

            if (lines == 0) {
                // Clear window
                for (int r = top_row; r <= bot_row && r < 25; r++) {
                    for (int c = left_col; c <= right_col && c < 80; c++) {
                        int off = (r * 80 + c) * 2;
                        video[0x18000 + off] = 0x20;
                        video[0x18000 + off + 1] = attr;
                    }
                }
            } else {
                // Scroll up by 'lines' lines
                for (int r = top_row; r <= bot_row - lines && r < 25; r++) {
                    for (int c = left_col; c <= right_col && c < 80; c++) {
                        int dst = (r * 80 + c) * 2;
                        int src = ((r + lines) * 80 + c) * 2;
                        video[0x18000 + dst] = video[0x18000 + src];
                        video[0x18000 + dst + 1] = video[0x18000 + src + 1];
                    }
                }
                // Clear bottom lines
                for (int r = bot_row - lines + 1; r <= bot_row && r < 25; r++) {
                    for (int c = left_col; c <= right_col && c < 80; c++) {
                        int off = (r * 80 + c) * 2;
                        video[0x18000 + off] = 0x20;
                        video[0x18000 + off + 1] = attr;
                    }
                }
            }
            s_i86.video_dirty = true;
            return true;
        }

        case 0x07: // Scroll down
        {
            uint8_t lines = al;
            uint8_t attr = bh;
            uint8_t top_row = (cx >> 8) & 0xFF;
            uint8_t left_col = cx & 0xFF;
            uint8_t bot_row = (i86_cpu_get_dx() >> 8) & 0xFF;
            uint8_t right_col = i86_cpu_get_dx() & 0xFF;

            if (lines == 0) {
                for (int r = top_row; r <= bot_row && r < 25; r++) {
                    for (int c = left_col; c <= right_col && c < 80; c++) {
                        int off = (r * 80 + c) * 2;
                        video[0x18000 + off] = 0x20;
                        video[0x18000 + off + 1] = attr;
                    }
                }
            } else {
                for (int r = bot_row; r >= top_row + lines; r--) {
                    for (int c = left_col; c <= right_col && c < 80; c++) {
                        int dst = (r * 80 + c) * 2;
                        int src = ((r - lines) * 80 + c) * 2;
                        video[0x18000 + dst] = video[0x18000 + src];
                        video[0x18000 + dst + 1] = video[0x18000 + src + 1];
                    }
                }
                for (int r = top_row; r < top_row + lines && r <= bot_row; r++) {
                    for (int c = left_col; c <= right_col && c < 80; c++) {
                        int off = (r * 80 + c) * 2;
                        video[0x18000 + off] = 0x20;
                        video[0x18000 + off + 1] = attr;
                    }
                }
            }
            s_i86.video_dirty = true;
            return true;
        }

        case 0x08: // Read char/attr at cursor
        {
            int offset = (s_i86.cursor_y * 80 + s_i86.cursor_x) * 2;
            uint8_t ch = video[0x18000 + offset];
            uint8_t at = video[0x18000 + offset + 1];
            i86_cpu_set_ax((at << 8) | ch);
            return true;
        }

        case 0x09: // Write char/attr at cursor (CX times)
        case 0x0A: // Write char at cursor (CX times)
        {
            int offset = (s_i86.cursor_y * 80 + s_i86.cursor_x) * 2;
            for (uint16_t i = 0; i < cx; i++) {
                int pos = offset + i * 2;
                if (pos >= 4000) break;
                video[0x18000 + pos] = al;
                if (ah == 0x09) {
                    video[0x18000 + pos + 1] = bl;
                }
            }
            s_i86.video_dirty = true;
            return true;
        }

        case 0x0E: // TTY output
            pc_putchar(al);
            return true;

        case 0x0F: // Get video mode
            i86_cpu_set_ax((80 << 8) | s_i86.video_mode);
            i86_cpu_set_bx((s_i86.active_page << 8) | (i86_cpu_get_bx() & 0xFF));
            return true;

        case 0x10: // Set palette registers (VGA)
            // Stub - return success
            return true;

        case 0x11: // Character generator
            // Stub - return font info
            if (al == 0x30) {
                // Get font information
                i86_cpu_set_cx(16);   // Bytes per character
                i86_cpu_set_dx(24);   // Rows - 1
                i86_cpu_set_es(0xF000);
                i86_cpu_set_bp(0xFA6E); // Font table pointer
            }
            return true;

        case 0x12: // Alternate function select
            if ((i86_cpu_get_bx() & 0xFF) == 0x10) {
                // Get EGA info
                i86_cpu_set_bx(0x0003); // 256K, color
                i86_cpu_set_cx(0x0009); // Feature bits, switch settings
            }
            return true;

        case 0x13: // Write string
        {
            uint8_t *mem = i86_cpu_get_memory();
            uint16_t es = i86_cpu_get_es();
            uint16_t bp_val = i86_cpu_get_bp();
            uint32_t str_addr = (uint32_t)es * 16 + bp_val;

            // Save cursor
            uint8_t save_x = s_i86.cursor_x;
            uint8_t save_y = s_i86.cursor_y;

            // Position cursor
            s_i86.cursor_x = dl;
            s_i86.cursor_y = dh;

            for (uint16_t i = 0; i < cx; i++) {
                if (al & 0x02) {
                    // Char + attr pairs
                    pc_putchar(mem[str_addr + i * 2]);
                    s_i86.text_attr = mem[str_addr + i * 2 + 1];
                } else {
                    // Chars only, use BL as attribute
                    s_i86.text_attr = bl;
                    pc_putchar(mem[str_addr + i]);
                }
            }

            // Restore cursor if mode says so
            if (!(al & 0x01)) {
                s_i86.cursor_x = save_x;
                s_i86.cursor_y = save_y;
            }
            return true;
        }

        case 0x1A: // Display combination code (VGA)
            i86_cpu_set_ax(0x001A); // Function supported
            i86_cpu_set_bx(0x0008); // VGA color
            return true;

        default:
            ESP_LOGD(TAG, "INT 10h AH=%02Xh (unhandled)", ah);
            return true;
    }
}

static bool handle_int11h(void) {
    // INT 11h - Equipment list
    // Bits: 0=floppy, 1=x87, 2-3=RAM banks, 4-5=video mode, 6-7=floppies-1
    uint16_t equip = 0x0021;  // 1 floppy, 80x25 CGA
    if (disk_is_mounted(0)) equip |= 0x0001;
    i86_cpu_set_ax(equip);
    return true;
}

static bool handle_int12h(void) {
    // INT 12h - Memory size
    i86_cpu_set_ax(640);  // 640KB
    return true;
}

static bool handle_int13h(void) {
    // INT 13h - Disk BIOS services
    uint8_t ah = (i86_cpu_get_ax() >> 8) & 0xFF;
    uint8_t al = i86_cpu_get_ax() & 0xFF;
    uint8_t ch = (i86_cpu_get_cx() >> 8) & 0xFF;
    uint8_t cl = i86_cpu_get_cx() & 0xFF;
    uint8_t dh = (i86_cpu_get_dx() >> 8) & 0xFF;
    uint8_t dl = i86_cpu_get_dx() & 0xFF;
    uint16_t es = i86_cpu_get_es();
    uint16_t bx = i86_cpu_get_bx();

    // Map drive number: 00h-01h=floppy (A:,B:), 80h-81h=HD (C:,D:)
    uint8_t drive_idx = (dl < 0x80) ? dl : (dl - 0x80 + 2);
    if (drive_idx >= MAX_DISK_DRIVES) drive_idx = 0;

    switch (ah) {
        case 0x00: // Reset disk system
            set_carry(false);
            i86_cpu_set_ax(0x0000);
            return true;

        case 0x01: // Get disk status
            i86_cpu_set_ax(bda_read8(BDA_DISK_STATUS) << 8);
            set_carry(false);
            return true;

        case 0x02: // Read sectors
            if (!disk_is_mounted(drive_idx)) {
                i86_cpu_set_ax(0x8000);  // Drive not ready
                set_carry(true);
                return true;
            }
            {
                int cylinder = ch | ((cl & 0xC0) << 2);
                int sector = cl & 0x3F;
                int head = dh;
                int count = al;

                int32_t lba = disk_chs_to_lba(drive_idx, cylinder, head, sector);
                if (lba < 0) {
                    i86_cpu_set_ax(0x0400);  // Sector not found
                    set_carry(true);
                    return true;
                }
                uint8_t *buffer = i86_cpu_get_memory() + 16 * es + bx;

                int sectors_read = disk_read_sectors(drive_idx, (uint32_t)lba, count, buffer);
                if (sectors_read > 0) {
                    i86_cpu_set_ax(sectors_read & 0xFF);
                    s_i86.stats.disk_reads++;
                    set_carry(false);
                } else {
                    i86_cpu_set_ax(0x0400);
                    set_carry(true);
                }
            }
            return true;

        case 0x03: // Write sectors
            if (!disk_is_mounted(drive_idx)) {
                i86_cpu_set_ax(0x8000);
                set_carry(true);
                return true;
            }
            {
                int cylinder = ch | ((cl & 0xC0) << 2);
                int sector = cl & 0x3F;
                int head = dh;
                int count = al;

                int32_t lba = disk_chs_to_lba(drive_idx, cylinder, head, sector);
                if (lba < 0) {
                    i86_cpu_set_ax(0x0400);
                    set_carry(true);
                    return true;
                }
                uint8_t *buffer = i86_cpu_get_memory() + 16 * es + bx;

                int sectors_written = disk_write_sectors(drive_idx, (uint32_t)lba, count, buffer);
                if (sectors_written > 0) {
                    i86_cpu_set_ax(sectors_written & 0xFF);
                    s_i86.stats.disk_writes++;
                    set_carry(false);
                } else {
                    i86_cpu_set_ax(0x0400);
                    set_carry(true);
                }
            }
            return true;

        case 0x04: // Verify sectors
            i86_cpu_set_ax(al);
            set_carry(false);
            return true;

        case 0x05: // Format track
            set_carry(false);
            i86_cpu_set_ax(0x0000);
            return true;

        case 0x08: // Get drive parameters
        {
            disk_info_t info;
            if (disk_get_info(drive_idx, &info) != ESP_OK || !info.mounted) {
                i86_cpu_set_ax(0x0700);
                set_carry(true);
                return true;
            }

            uint16_t cyls = info.geometry.cylinders > 0 ? info.geometry.cylinders - 1 : 0;
            uint8_t spt = info.geometry.sectors_per_track;
            uint8_t heads = info.geometry.heads > 0 ? info.geometry.heads - 1 : 0;

            i86_cpu_set_ax(0x0000);
            i86_cpu_set_bx(dl < 0x80 ? 0x0004 : 0x0000); // Drive type (floppy: 1.44M)
            i86_cpu_set_cx(((cyls & 0xFF) << 8) | ((cyls >> 2) & 0xC0) | (spt & 0x3F));
            i86_cpu_set_dx((heads << 8) | 0x01);  // DH=max head, DL=number of drives

            // Count mounted drives
            int drive_count = 0;
            for (int d = 0; d < MAX_DISK_DRIVES; d++) {
                if (disk_is_mounted(d)) drive_count++;
            }
            if (dl < 0x80) {
                i86_cpu_set_dx((heads << 8) | (drive_count < 2 ? 1 : 2));
            } else {
                int hd_count = 0;
                if (disk_is_mounted(2)) hd_count++;
                if (disk_is_mounted(3)) hd_count++;
                i86_cpu_set_dx((heads << 8) | (hd_count > 0 ? hd_count : 1));
                bda_write8(BDA_DISK_COUNT, hd_count);
            }
            set_carry(false);
            return true;
        }

        case 0x15: // Get disk type
        {
            disk_info_t info;
            if (disk_get_info(drive_idx, &info) == ESP_OK && info.mounted) {
                if (dl < 0x80) {
                    i86_cpu_set_ax(0x0200);  // Floppy with change-line
                } else {
                    i86_cpu_set_ax(0x0300);  // Hard disk
                }
                uint32_t sectors = info.geometry.total_sectors;
                i86_cpu_set_cx(sectors >> 16);
                i86_cpu_set_dx(sectors & 0xFFFF);
                set_carry(false);
            } else {
                i86_cpu_set_ax(0x0000);  // No drive
                set_carry(true);
            }
            return true;
        }

        case 0x16: // Change of disk status (floppy)
            i86_cpu_set_ax(0x0000);
            set_carry(false);
            return true;

        case 0x17: // Set disk type for format
            set_carry(false);
            return true;

        case 0x18: // Set media type for format
            set_carry(false);
            return true;

        default:
            ESP_LOGD(TAG, "INT 13h AH=%02Xh (unhandled)", ah);
            i86_cpu_set_ax(0x0100);
            set_carry(true);
            return true;
    }
}

static bool handle_int14h(void) {
    // INT 14h - Serial port services (stub)
    uint8_t ah = (i86_cpu_get_ax() >> 8) & 0xFF;

    switch (ah) {
        case 0x00: // Initialize serial port
            i86_cpu_set_ax(0x0000);
            return true;
        case 0x03: // Status
            i86_cpu_set_ax(0x6000);  // Transmitter empty, ready
            return true;
        default:
            return true;
    }
}

static bool handle_int15h(void) {
    // INT 15h - System services
    uint8_t ah = (i86_cpu_get_ax() >> 8) & 0xFF;

    switch (ah) {
        case 0x24: // A20 gate (stub)
            set_carry(false);
            return true;

        case 0x41: // Wait for external event
            set_carry(true);  // Function not supported
            return true;

        case 0x4F: // Keyboard intercept
            set_carry(true);  // Not handled, let default processing occur
            return true;

        case 0x86: // Wait (microseconds in CX:DX)
        {
            uint32_t us = ((uint32_t)i86_cpu_get_cx() << 16) | i86_cpu_get_dx();
            if (us > 1000000) us = 1000000;  // Cap at 1 second
            vTaskDelay(pdMS_TO_TICKS(us / 1000));
            set_carry(false);
            return true;
        }

        case 0x87: // Move block (extended memory)
            set_carry(true);  // Not supported on XT
            i86_cpu_set_ax(0x8600);
            return true;

        case 0x88: // Get extended memory size
            i86_cpu_set_ax(0);  // No extended memory on XT
            set_carry(false);
            return true;

        case 0xC0: // Get system configuration
            set_carry(true);  // Not supported
            return true;

        case 0xC1: // Get EBDA segment
            i86_cpu_set_es(0x9FC0);
            set_carry(false);
            return true;

        default:
            set_carry(true);
            return true;
    }
}

static bool handle_int16h(void) {
    // INT 16h - Keyboard BIOS services
    uint8_t ah = (i86_cpu_get_ax() >> 8) & 0xFF;

    switch (ah) {
        case 0x00: // Wait for keypress
        case 0x10: // Enhanced wait for keypress
        {
            // Poll for key - yield to FreeRTOS while waiting
            int timeout = 0;
            while (s_i86.bios_kbd_head == s_i86.bios_kbd_tail) {
                vTaskDelay(pdMS_TO_TICKS(10));
                poll_input();  // Actually check for new keys while waiting
                timeout++;
                if (s_i86.state != I86_STATE_RUNNING || s_i86.stop_requested) {
                    i86_cpu_set_ax(0);
                    return true;
                }
                // Poll UART and PS/2 for new keys
                // This is done by the input polling in the main run loop
                if (timeout > 3000) {  // 30 second timeout
                    i86_cpu_set_ax(0);
                    return true;
                }
            }
            uint16_t key = s_i86.bios_kbd_buffer[s_i86.bios_kbd_tail];
            s_i86.bios_kbd_tail = (s_i86.bios_kbd_tail + 1) % 16;
            i86_cpu_set_ax(key);
            return true;
        }

        case 0x01: // Check for keypress
        case 0x11: // Enhanced check
            if (s_i86.bios_kbd_head != s_i86.bios_kbd_tail) {
                i86_cpu_set_ax(s_i86.bios_kbd_buffer[s_i86.bios_kbd_tail]);
                i86_cpu_set_flag(FLAG_IDX_ZF, 0);  // Key available
            } else {
                i86_cpu_set_ax(0x0000);
                i86_cpu_set_flag(FLAG_IDX_ZF, 1);  // No key
            }
            return true;

        case 0x02: // Get shift flags
            i86_cpu_set_ax(s_i86.hw.keyboard.shift_flags);
            return true;

        case 0x03: // Set typematic rate
            return true;

        case 0x05: // Store keypress in buffer
        {
            int next = (s_i86.bios_kbd_head + 1) % 16;
            if (next != s_i86.bios_kbd_tail) {
                s_i86.bios_kbd_buffer[s_i86.bios_kbd_head] = i86_cpu_get_cx();
                s_i86.bios_kbd_head = next;
                i86_cpu_set_ax(0x0000);  // Success
            } else {
                i86_cpu_set_ax(0x0100);  // Buffer full
            }
            return true;
        }

        case 0x12: // Extended shift flags
            i86_cpu_set_ax(s_i86.hw.keyboard.shift_flags);
            return true;

        default:
            ESP_LOGD(TAG, "INT 16h AH=%02Xh (unhandled)", ah);
            return true;
    }
}

static bool handle_int17h(void) {
    // INT 17h - Printer services (stub)
    i86_cpu_set_ax(0xD000);  // Not busy, selected, no error
    return true;
}

static bool handle_int19h(void) {
    // INT 19h - Bootstrap loader
    ESP_LOGI(TAG, "INT 19h: Bootstrap loader");

    // Try boot from first mounted disk
    for (int i = 0; i < MAX_DISK_DRIVES; i++) {
        if (disk_is_mounted(i)) {
            uint8_t *buffer = i86_cpu_get_memory() + 0x7C00;
            int sectors = disk_read_sectors(i, 0, 1, buffer);
            if (sectors == 1) {
                if (buffer[510] == 0x55 && buffer[511] == 0xAA) {
                    ESP_LOGI(TAG, "Booting from drive %d (0x%02X)",
                             i, i < 2 ? i : 0x80 + (i - 2));

                    console_printf("Booting from %s...\n",
                                   i < 2 ? "floppy" : "hard disk");

                    // INT 19h does NOT return via IRET.
                    // The CPU core calls our callback BEFORE pushing anything
                    // to the stack. If we return true, it simply returns to
                    // the exec loop. So we just set CS:IP directly.
                    
                    // Directly set CS:IP to boot sector
                    i86_cpu_set_cs(0x0000);
                    i86_cpu_set_ip(0x7C00);

                    // Set boot drive in DL
                    i86_cpu_set_dx(i < 2 ? i : 0x80 + (i - 2));

                    // Clear other registers for boot (standard BIOS convention)
                    i86_cpu_set_ax(0);
                    i86_cpu_set_bx(0);
                    i86_cpu_set_cx(0);
                    i86_cpu_set_si(0);
                    i86_cpu_set_di(0);
                    i86_cpu_set_bp(0);
                    i86_cpu_set_ds(0);
                    i86_cpu_set_es(0);
                    i86_cpu_set_ss(0x0000);
                    i86_cpu_set_sp(0x7C00);  // Stack below boot sector

                    // Verify first bytes of boot sector
                    ESP_LOGI(TAG, "Boot sector at 7C00: %02X %02X %02X %02X %02X %02X %02X %02X",
                             buffer[0], buffer[1], buffer[2], buffer[3],
                             buffer[4], buffer[5], buffer[6], buffer[7]);
                    ESP_LOGI(TAG, "CS:IP=0000:7C00 SP=7C00 DL=0x%02X, executing boot sector",
                             i < 2 ? i : 0x80 + (i - 2));

                    return true;  // Handled - CPU continues at 0000:7C00
                }
            }
        }
    }

    ESP_LOGW(TAG, "No bootable disk found");
    console_printf("No bootable disk found.\n");
    console_printf("Use 'mount A <image.img>' to mount a boot disk.\n");
    s_i86.state = I86_STATE_HALTED;
    return true;
}

static bool handle_int1ah(void) {
    // INT 1Ah - Real-time clock services
    uint8_t ah = (i86_cpu_get_ax() >> 8) & 0xFF;

    switch (ah) {
        case 0x00: // Get tick count
            i86_cpu_set_cx((s_i86.timer_ticks >> 16) & 0xFFFF);
            i86_cpu_set_dx(s_i86.timer_ticks & 0xFFFF);
            i86_cpu_set_ax(s_i86.timer_midnight ? 1 : 0);
            s_i86.timer_midnight = false;
            return true;

        case 0x01: // Set tick count
            s_i86.timer_ticks = ((uint32_t)i86_cpu_get_cx() << 16) | i86_cpu_get_dx();
            return true;

        case 0x02: // Get RTC time
        {
            rtc_update_time(&s_i86.hw.rtc);
            i86_cpu_set_cx((s_i86.hw.rtc.cmos[0x04] << 8) | s_i86.hw.rtc.cmos[0x02]);
            i86_cpu_set_dx((s_i86.hw.rtc.cmos[0x00] << 8) | 0x00);
            set_carry(false);
            return true;
        }

        case 0x04: // Get RTC date
        {
            rtc_update_time(&s_i86.hw.rtc);
            i86_cpu_set_cx((s_i86.hw.rtc.cmos[0x32] << 8) | s_i86.hw.rtc.cmos[0x09]);
            i86_cpu_set_dx((s_i86.hw.rtc.cmos[0x08] << 8) | s_i86.hw.rtc.cmos[0x07]);
            set_carry(false);
            return true;
        }

        default:
            set_carry(true);
            return true;
    }
}

// ============================================================================
// DOS INT 21h Handler
// ============================================================================

static void dos_init_state(void) {
    memset(&s_i86.dos, 0, sizeof(dos_state_t));
    strcpy(s_i86.dos.current_dir, "C:\\");
    strcpy(s_i86.dos.current_path, "/sdcard");
    s_i86.dos.dta_segment = 0x0080;  // Default DTA in PSP
    s_i86.dos.dta_offset = 0x0080;
    s_i86.dos.psp_segment = 0x0060;
}

static int dos_find_free_handle(void) {
    for (int i = DOS_FIRST_HANDLE; i < DOS_MAX_FILES; i++) {
        if (!s_i86.dos.files[i].open) return i;
    }
    return -1;
}

static bool handle_int21h(void) {
    uint8_t ah = (i86_cpu_get_ax() >> 8) & 0xFF;
    uint8_t al = i86_cpu_get_ax() & 0xFF;

    s_i86.stats.dos_calls++;

    switch (ah) {
        case 0x00: // Terminate program
            s_i86.state = I86_STATE_HALTED;
            set_carry(false);
            return true;

        case 0x01: // Read character with echo
        {
            // Wait for key from BIOS buffer
            while (s_i86.bios_kbd_head == s_i86.bios_kbd_tail) {
                vTaskDelay(pdMS_TO_TICKS(10));
                if (s_i86.state != I86_STATE_RUNNING || s_i86.stop_requested) {
                    i86_cpu_set_ax(0);
                    return true;
                }
            }
            uint16_t key = s_i86.bios_kbd_buffer[s_i86.bios_kbd_tail];
            s_i86.bios_kbd_tail = (s_i86.bios_kbd_tail + 1) % 16;
            uint8_t ch = key & 0xFF;
            pc_putchar(ch);  // Echo
            i86_cpu_set_ax((ah << 8) | ch);
            return true;
        }

        case 0x02: // Write character
            pc_putchar(i86_cpu_get_dx() & 0xFF);
            return true;

        case 0x06: // Direct console I/O
            if ((i86_cpu_get_dx() & 0xFF) == 0xFF) {
                // Input
                if (s_i86.bios_kbd_head != s_i86.bios_kbd_tail) {
                    uint16_t key = s_i86.bios_kbd_buffer[s_i86.bios_kbd_tail];
                    s_i86.bios_kbd_tail = (s_i86.bios_kbd_tail + 1) % 16;
                    i86_cpu_set_ax((ah << 8) | (key & 0xFF));
                    i86_cpu_set_flag(FLAG_IDX_ZF, 0);  // Key available
                } else {
                    i86_cpu_set_ax(ah << 8);
                    i86_cpu_set_flag(FLAG_IDX_ZF, 1);  // No key
                }
            } else {
                // Output
                pc_putchar(i86_cpu_get_dx() & 0xFF);
            }
            return true;

        case 0x07: // Direct char input without echo
        case 0x08: // Char input without echo
        {
            while (s_i86.bios_kbd_head == s_i86.bios_kbd_tail) {
                vTaskDelay(pdMS_TO_TICKS(10));
                if (s_i86.state != I86_STATE_RUNNING || s_i86.stop_requested) return true;
            }
            uint16_t key = s_i86.bios_kbd_buffer[s_i86.bios_kbd_tail];
            s_i86.bios_kbd_tail = (s_i86.bios_kbd_tail + 1) % 16;
            i86_cpu_set_ax((ah << 8) | (key & 0xFF));
            return true;
        }

        case 0x09: // Write string (terminated by '$')
        {
            uint8_t *mem = i86_cpu_get_memory();
            uint32_t addr = 16 * i86_cpu_get_ds() + i86_cpu_get_dx();
            while (mem[addr] != '$' && addr < I8086_TOTAL_MEMORY) {
                pc_putchar(mem[addr++]);
            }
            return true;
        }

        case 0x0A: // Buffered input
        {
            uint8_t *mem = i86_cpu_get_memory();
            uint32_t buf_addr = 16 * i86_cpu_get_ds() + i86_cpu_get_dx();
            uint8_t max_len = mem[buf_addr];
            uint8_t pos = 0;

            while (pos < max_len - 1) {
                while (s_i86.bios_kbd_head == s_i86.bios_kbd_tail) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                    if (s_i86.state != I86_STATE_RUNNING || s_i86.stop_requested) {
                        mem[buf_addr + 1] = pos;
                        mem[buf_addr + 2 + pos] = '\r';
                        return true;
                    }
                }
                uint16_t key = s_i86.bios_kbd_buffer[s_i86.bios_kbd_tail];
                s_i86.bios_kbd_tail = (s_i86.bios_kbd_tail + 1) % 16;
                uint8_t ch = key & 0xFF;

                if (ch == '\r' || ch == '\n') {
                    pc_putchar('\r');
                    pc_putchar('\n');
                    break;
                } else if (ch == '\b') {
                    if (pos > 0) {
                        pos--;
                        pc_putchar('\b');
                        pc_putchar(' ');
                        pc_putchar('\b');
                    }
                } else if (ch >= 0x20) {
                    mem[buf_addr + 2 + pos] = ch;
                    pos++;
                    pc_putchar(ch);
                }
            }
            mem[buf_addr + 1] = pos;
            mem[buf_addr + 2 + pos] = '\r';
            return true;
        }

        case 0x0B: // Check standard input status
            if (s_i86.bios_kbd_head != s_i86.bios_kbd_tail) {
                i86_cpu_set_ax(0x0BFF);
            } else {
                i86_cpu_set_ax(0x0B00);
            }
            return true;

        case 0x0C: // Clear keyboard buffer and invoke input function
            s_i86.bios_kbd_head = 0;
            s_i86.bios_kbd_tail = 0;
            // Fall through to input function specified in AL
            if (al == 0x01 || al == 0x06 || al == 0x07 || al == 0x08 || al == 0x0A) {
                i86_cpu_set_ax((al << 8) | (i86_cpu_get_ax() & 0xFF));
                return handle_int21h();  // Recurse with new AH
            }
            return true;

        case 0x0D: // Disk reset
            disk_sync_all();
            return true;

        case 0x0E: // Select disk
        {
            uint8_t drive = i86_cpu_get_dx() & 0xFF;
            (void)drive;
            i86_cpu_set_ax((ah << 8) | MAX_DISK_DRIVES);
            return true;
        }

        case 0x19: // Get current disk
            i86_cpu_set_ax((ah << 8) | 0x02);  // C: drive
            return true;

        case 0x1A: // Set DTA address
            s_i86.dos.dta_segment = i86_cpu_get_ds();
            s_i86.dos.dta_offset = i86_cpu_get_dx();
            return true;

        case 0x25: // Set interrupt vector
        {
            uint8_t vec = al;
            uint8_t *mem = i86_cpu_get_memory();
            uint16_t new_seg = i86_cpu_get_ds();
            uint16_t new_off = i86_cpu_get_dx();
            mem[vec * 4 + 0] = new_off & 0xFF;
            mem[vec * 4 + 1] = (new_off >> 8) & 0xFF;
            mem[vec * 4 + 2] = new_seg & 0xFF;
            mem[vec * 4 + 3] = (new_seg >> 8) & 0xFF;
            return true;
        }

        case 0x2A: // Get date
        {
            rtc_update_time(&s_i86.hw.rtc);
            time_t now;
            time(&now);
            struct tm *t = localtime(&now);
            if (t) {
                i86_cpu_set_cx(t->tm_year + 1900);
                i86_cpu_set_dx(((t->tm_mon + 1) << 8) | t->tm_mday);
                i86_cpu_set_ax((ah << 8) | t->tm_wday);
            }
            return true;
        }

        case 0x2C: // Get time
        {
            time_t now;
            time(&now);
            struct tm *t = localtime(&now);
            if (t) {
                i86_cpu_set_cx((t->tm_hour << 8) | t->tm_min);
                i86_cpu_set_dx((t->tm_sec << 8) | 0x00);
            }
            return true;
        }

        case 0x30: // Get DOS version
            i86_cpu_set_ax(0x0005);  // DOS 5.0
            i86_cpu_set_bx(0x0000);
            i86_cpu_set_cx(0x0000);
            return true;

        case 0x33: // Get/set Ctrl+C check flag
            if (al == 0x00) {
                i86_cpu_set_dx(s_i86.dos.break_check ? 1 : 0);
            } else if (al == 0x01) {
                s_i86.dos.break_check = (i86_cpu_get_dx() & 1) != 0;
            } else if (al == 0x06) {
                // Get true DOS version
                i86_cpu_set_bx(0x0500);  // 5.00
                i86_cpu_set_dx(0x0000);
            }
            return true;

        case 0x35: // Get interrupt vector
        {
            uint8_t vec = al;
            uint8_t *mem = i86_cpu_get_memory();
            uint16_t seg = mem[vec * 4 + 2] | (mem[vec * 4 + 3] << 8);
            uint16_t off = mem[vec * 4 + 0] | (mem[vec * 4 + 1] << 8);
            i86_cpu_set_es(seg);
            i86_cpu_set_bx(off);
            return true;
        }

        case 0x36: // Get disk free space
        {
            // Report generous free space
            i86_cpu_set_ax(64);     // Sectors per cluster
            i86_cpu_set_bx(10000);  // Available clusters
            i86_cpu_set_cx(512);    // Bytes per sector
            i86_cpu_set_dx(20000);  // Total clusters
            return true;
        }

        case 0x3B: // Change directory
        {
            char dos_path[DOS_PATH_MAX];
            read_dos_string(i86_cpu_get_ds(), i86_cpu_get_dx(), dos_path, sizeof(dos_path), '\0');
            char vfs_path[DOS_PATH_MAX];
            dos_to_vfs_path(dos_path, vfs_path, sizeof(vfs_path));

            struct stat st;
            if (stat(vfs_path, &st) == 0 && S_ISDIR(st.st_mode)) {
                strncpy(s_i86.dos.current_path, vfs_path, DOS_PATH_MAX - 1);
                set_carry(false);
            } else {
                i86_cpu_set_ax(0x03);  // Path not found
                set_carry(true);
            }
            return true;
        }

        case 0x3C: // Create file
        {
            char dos_path[DOS_PATH_MAX];
            read_dos_string(i86_cpu_get_ds(), i86_cpu_get_dx(), dos_path, sizeof(dos_path), '\0');
            char vfs_path[DOS_PATH_MAX];
            dos_to_vfs_path(dos_path, vfs_path, sizeof(vfs_path));

            int handle = dos_find_free_handle();
            if (handle < 0) {
                i86_cpu_set_ax(0x04);  // Too many open files
                set_carry(true);
                return true;
            }

            FILE *fp = fopen(vfs_path, "wb");
            if (!fp) {
                i86_cpu_set_ax(0x03);  // Path not found
                set_carry(true);
                return true;
            }

            s_i86.dos.files[handle].fp = fp;
            s_i86.dos.files[handle].open = true;
            s_i86.dos.files[handle].access_mode = 1;
            strncpy(s_i86.dos.files[handle].path, vfs_path, DOS_PATH_MAX - 1);
            s_i86.stats.files_opened++;

            i86_cpu_set_ax(handle);
            set_carry(false);
            return true;
        }

        case 0x3D: // Open file
        {
            char dos_path[DOS_PATH_MAX];
            read_dos_string(i86_cpu_get_ds(), i86_cpu_get_dx(), dos_path, sizeof(dos_path), '\0');
            char vfs_path[DOS_PATH_MAX];
            dos_to_vfs_path(dos_path, vfs_path, sizeof(vfs_path));

            int handle = dos_find_free_handle();
            if (handle < 0) {
                i86_cpu_set_ax(0x04);
                set_carry(true);
                return true;
            }

            const char *mode = "rb";
            uint8_t access = al & 0x07;
            if (access == 1) mode = "r+b";       // Write only
            else if (access == 2) mode = "r+b";  // Read/Write

            FILE *fp = fopen(vfs_path, mode);
            if (!fp && access > 0) {
                fp = fopen(vfs_path, "w+b");  // Create if write mode
            }
            if (!fp) {
                i86_cpu_set_ax(0x02);  // File not found
                set_carry(true);
                return true;
            }

            s_i86.dos.files[handle].fp = fp;
            s_i86.dos.files[handle].open = true;
            s_i86.dos.files[handle].access_mode = access;
            strncpy(s_i86.dos.files[handle].path, vfs_path, DOS_PATH_MAX - 1);
            s_i86.stats.files_opened++;

            i86_cpu_set_ax(handle);
            set_carry(false);
            return true;
        }

        case 0x3E: // Close file
        {
            uint16_t handle = i86_cpu_get_bx();
            if (handle < DOS_FIRST_HANDLE) {
                // Standard handles - ignore close
                set_carry(false);
                return true;
            }
            if (handle >= DOS_MAX_FILES || !s_i86.dos.files[handle].open) {
                i86_cpu_set_ax(0x06);  // Invalid handle
                set_carry(true);
                return true;
            }
            fclose(s_i86.dos.files[handle].fp);
            s_i86.dos.files[handle].open = false;
            s_i86.dos.files[handle].fp = NULL;
            set_carry(false);
            return true;
        }

        case 0x3F: // Read file
        {
            uint16_t handle = i86_cpu_get_bx();
            uint16_t count = i86_cpu_get_cx();
            uint8_t *mem = i86_cpu_get_memory();
            uint32_t buf_addr = 16 * i86_cpu_get_ds() + i86_cpu_get_dx();

            if (handle == 0x0000) {
                // Read from STDIN (keyboard)
                uint16_t read_count = 0;
                while (read_count < count) {
                    while (s_i86.bios_kbd_head == s_i86.bios_kbd_tail) {
                        vTaskDelay(pdMS_TO_TICKS(10));
                        if (s_i86.state != I86_STATE_RUNNING || s_i86.stop_requested) {
                            i86_cpu_set_ax(read_count);
                            set_carry(false);
                            return true;
                        }
                    }
                    uint16_t key = s_i86.bios_kbd_buffer[s_i86.bios_kbd_tail];
                    s_i86.bios_kbd_tail = (s_i86.bios_kbd_tail + 1) % 16;
                    uint8_t ch = key & 0xFF;
                    mem[buf_addr + read_count] = ch;
                    read_count++;
                    pc_putchar(ch);
                    if (ch == '\r') {
                        mem[buf_addr + read_count] = '\n';
                        read_count++;
                        pc_putchar('\n');
                        break;
                    }
                }
                i86_cpu_set_ax(read_count);
                set_carry(false);
                return true;
            }

            if (handle >= DOS_MAX_FILES || !s_i86.dos.files[handle].open) {
                i86_cpu_set_ax(0x06);
                set_carry(true);
                return true;
            }

            size_t bytes = fread(mem + buf_addr, 1, count, s_i86.dos.files[handle].fp);
            i86_cpu_set_ax(bytes);
            set_carry(false);
            return true;
        }

        case 0x40: // Write file
        {
            uint16_t handle = i86_cpu_get_bx();
            uint16_t count = i86_cpu_get_cx();
            uint8_t *mem = i86_cpu_get_memory();
            uint32_t buf_addr = 16 * i86_cpu_get_ds() + i86_cpu_get_dx();

            if (handle <= 0x0002) {
                // STDOUT/STDERR - output to console
                for (uint16_t i = 0; i < count; i++) {
                    pc_putchar(mem[buf_addr + i]);
                }
                i86_cpu_set_ax(count);
                set_carry(false);
                return true;
            }

            if (handle >= DOS_MAX_FILES || !s_i86.dos.files[handle].open) {
                i86_cpu_set_ax(0x06);
                set_carry(true);
                return true;
            }

            size_t bytes = fwrite(mem + buf_addr, 1, count, s_i86.dos.files[handle].fp);
            i86_cpu_set_ax(bytes);
            set_carry(false);
            return true;
        }

        case 0x41: // Delete file
        {
            char dos_path[DOS_PATH_MAX];
            read_dos_string(i86_cpu_get_ds(), i86_cpu_get_dx(), dos_path, sizeof(dos_path), '\0');
            char vfs_path[DOS_PATH_MAX];
            dos_to_vfs_path(dos_path, vfs_path, sizeof(vfs_path));

            if (remove(vfs_path) == 0) {
                set_carry(false);
            } else {
                i86_cpu_set_ax(0x02);
                set_carry(true);
            }
            return true;
        }

        case 0x42: // Move file pointer (lseek)
        {
            uint16_t handle = i86_cpu_get_bx();
            if (handle >= DOS_MAX_FILES || !s_i86.dos.files[handle].open) {
                i86_cpu_set_ax(0x06);
                set_carry(true);
                return true;
            }

            int32_t offset = (int32_t)(((uint32_t)i86_cpu_get_cx() << 16) | i86_cpu_get_dx());
            int whence = SEEK_SET;
            if (al == 1) whence = SEEK_CUR;
            else if (al == 2) whence = SEEK_END;

            if (fseek(s_i86.dos.files[handle].fp, offset, whence) == 0) {
                long pos = ftell(s_i86.dos.files[handle].fp);
                i86_cpu_set_dx((pos >> 16) & 0xFFFF);
                i86_cpu_set_ax(pos & 0xFFFF);
                set_carry(false);
            } else {
                i86_cpu_set_ax(0x19);  // Seek error
                set_carry(true);
            }
            return true;
        }

        case 0x43: // Get/set file attributes
        {
            char dos_path[DOS_PATH_MAX];
            read_dos_string(i86_cpu_get_ds(), i86_cpu_get_dx(), dos_path, sizeof(dos_path), '\0');
            char vfs_path[DOS_PATH_MAX];
            dos_to_vfs_path(dos_path, vfs_path, sizeof(vfs_path));

            struct stat st;
            if (stat(vfs_path, &st) == 0) {
                if (al == 0x00) {
                    // Get attributes
                    uint16_t attr = 0x20;  // Archive
                    if (S_ISDIR(st.st_mode)) attr = 0x10;
                    i86_cpu_set_cx(attr);
                }
                set_carry(false);
            } else {
                i86_cpu_set_ax(0x02);
                set_carry(true);
            }
            return true;
        }

        case 0x44: // IOCTL
        {
            if (al == 0x00) {
                // Get device information
                uint16_t handle = i86_cpu_get_bx();
                if (handle <= 0x0002) {
                    i86_cpu_set_dx(0x80D3);  // Character device, STDIN/STDOUT
                } else {
                    i86_cpu_set_dx(0x0000);  // File
                }
                set_carry(false);
            } else {
                set_carry(true);
                i86_cpu_set_ax(0x01);
            }
            return true;
        }

        case 0x47: // Get current directory
        {
            // Write directory path to DS:SI
            char *dir = s_i86.dos.current_dir;
            // Skip drive letter and backslash
            if (dir[0] && dir[1] == ':' && dir[2] == '\\') dir += 3;
            write_dos_string(i86_cpu_get_ds(), i86_cpu_get_si(), dir);
            set_carry(false);
            return true;
        }

        case 0x48: // Allocate memory
        {
            (void)i86_cpu_get_bx(); // paragraphs requested
            // Simple memory allocation from top of conventional memory
            // For now, report failure (DOS manages its own memory via MCBs)
            i86_cpu_set_ax(0x08);  // Insufficient memory
            i86_cpu_set_bx(0x0000);  // Largest available
            set_carry(true);
            return true;
        }

        case 0x49: // Free memory
            set_carry(false);
            return true;

        case 0x4A: // Resize memory block
            set_carry(false);
            return true;

        case 0x4C: // Terminate with return code
            s_i86.dos.return_code = al;
            ESP_LOGI(TAG, "INT 21h AH=4Ch: Program terminated (code %d)", al);
            s_i86.state = I86_STATE_HALTED;
            return true;

        case 0x4D: // Get return code
            i86_cpu_set_ax(s_i86.dos.return_code);
            return true;

        case 0x4E: // Find first matching file
        {
            char dos_path[DOS_PATH_MAX];
            read_dos_string(i86_cpu_get_ds(), i86_cpu_get_dx(), dos_path, sizeof(dos_path), '\0');
            char vfs_path[DOS_PATH_MAX];
            dos_to_vfs_path(dos_path, vfs_path, sizeof(vfs_path));

            // Extract directory and pattern
            char *slash = strrchr(vfs_path, '/');
            char dir_path[DOS_PATH_MAX];
            char pattern[64] = "*";

            if (slash) {
                strncpy(dir_path, vfs_path, slash - vfs_path);
                dir_path[slash - vfs_path] = '\0';
                strncpy(pattern, slash + 1, sizeof(pattern) - 1);
            } else {
                strcpy(dir_path, s_i86.dos.current_path);
                strncpy(pattern, vfs_path, sizeof(pattern) - 1);
            }

            if (s_i86.dos.find_dir) {
                closedir(s_i86.dos.find_dir);
            }
            s_i86.dos.find_dir = opendir(dir_path);
            if (!s_i86.dos.find_dir) {
                i86_cpu_set_ax(0x12);  // No more files
                set_carry(true);
                return true;
            }

            strncpy(s_i86.dos.find_pattern, pattern, sizeof(s_i86.dos.find_pattern) - 1);
            strncpy(s_i86.dos.find_dir_path, dir_path, sizeof(s_i86.dos.find_dir_path) - 1);
            s_i86.dos.find_attrib = i86_cpu_get_cx() & 0xFF;

            // Fall through to find next
        }
        /* fall through */

        case 0x4F: // Find next matching file
        {
            if (!s_i86.dos.find_dir) {
                i86_cpu_set_ax(0x12);
                set_carry(true);
                return true;
            }

            struct dirent *entry;
            while ((entry = readdir(s_i86.dos.find_dir)) != NULL) {
                // Simple wildcard match (*.* matches all)
                bool match = false;
                if (strcmp(s_i86.dos.find_pattern, "*.*") == 0 ||
                    strcmp(s_i86.dos.find_pattern, "*") == 0) {
                    match = true;
                } else {
                    // Basic extension match
                    char *pat_dot = strrchr(s_i86.dos.find_pattern, '.');
                    char *ent_dot = strrchr(entry->d_name, '.');
                    if (s_i86.dos.find_pattern[0] == '*' && pat_dot) {
                        // *.ext pattern
                        if (ent_dot && strcasecmp(pat_dot, ent_dot) == 0) match = true;
                    } else {
                        if (strcasecmp(entry->d_name, s_i86.dos.find_pattern) == 0) match = true;
                    }
                }

                if (match) {
                    // Fill DTA with file info
                    uint8_t *mem = i86_cpu_get_memory();
                    uint32_t dta = (uint32_t)s_i86.dos.dta_segment * 16 + s_i86.dos.dta_offset;

                    // DTA format:
                    // +00h: 21 bytes reserved (search state)
                    // +15h: file attributes
                    // +16h: file time
                    // +18h: file date
                    // +1Ah: file size (4 bytes)
                    // +1Eh: filename (13 bytes, null-terminated)

                    memset(mem + dta, 0, 43);

                    // Get file info
                    char fullpath[DOS_PATH_MAX + 256];
                    snprintf(fullpath, sizeof(fullpath), "%s/%s", s_i86.dos.find_dir_path, entry->d_name);
                    struct stat st;
                    if (stat(fullpath, &st) == 0) {
                        uint8_t attr = 0x20;  // Archive
                        if (S_ISDIR(st.st_mode)) attr = 0x10;  // Directory
                        mem[dta + 0x15] = attr;

                        // File size
                        uint32_t fsize = (uint32_t)st.st_size;
                        mem[dta + 0x1A] = fsize & 0xFF;
                        mem[dta + 0x1B] = (fsize >> 8) & 0xFF;
                        mem[dta + 0x1C] = (fsize >> 16) & 0xFF;
                        mem[dta + 0x1D] = (fsize >> 24) & 0xFF;
                    }

                    // Filename (uppercase, 8.3 format)
                    size_t name_len = strlen(entry->d_name);
                    if (name_len > 12) name_len = 12;
                    for (size_t i = 0; i < name_len; i++) {
                        mem[dta + 0x1E + i] = toupper((unsigned char)entry->d_name[i]);
                    }
                    mem[dta + 0x1E + name_len] = '\0';

                    set_carry(false);
                    return true;
                }
            }

            // No more files
            closedir(s_i86.dos.find_dir);
            s_i86.dos.find_dir = NULL;
            i86_cpu_set_ax(0x12);
            set_carry(true);
            return true;
        }

        case 0x50: // Set PSP segment
            s_i86.dos.psp_segment = i86_cpu_get_bx();
            return true;

        case 0x51: // Get PSP segment
        case 0x62: // Get PSP segment
            i86_cpu_set_bx(s_i86.dos.psp_segment);
            return true;

        case 0x56: // Rename file
        {
            char old_dos[DOS_PATH_MAX], new_dos[DOS_PATH_MAX];
            char old_vfs[DOS_PATH_MAX], new_vfs[DOS_PATH_MAX];
            read_dos_string(i86_cpu_get_ds(), i86_cpu_get_dx(), old_dos, sizeof(old_dos), '\0');
            read_dos_string(i86_cpu_get_es(), i86_cpu_get_di(), new_dos, sizeof(new_dos), '\0');
            dos_to_vfs_path(old_dos, old_vfs, sizeof(old_vfs));
            dos_to_vfs_path(new_dos, new_vfs, sizeof(new_vfs));

            if (rename(old_vfs, new_vfs) == 0) {
                set_carry(false);
            } else {
                i86_cpu_set_ax(0x05);
                set_carry(true);
            }
            return true;
        }

        case 0x57: // Get/set file date/time
            if (al == 0x00) {
                i86_cpu_set_cx(0x0000);  // Time = 0
                i86_cpu_set_dx(0x0021);  // Date = 1980-01-01
            }
            set_carry(false);
            return true;

        case 0x58: // Get/set memory allocation strategy
            if (al == 0x00) {
                i86_cpu_set_ax(0x0000);  // First fit
            }
            set_carry(false);
            return true;

        case 0x59: // Get extended error information
            i86_cpu_set_ax(s_i86.dos.last_error);
            i86_cpu_set_bx(0x0001);  // Error class: out of resource
            set_carry(false);
            return true;

        default:
            ESP_LOGD(TAG, "INT 21h AH=%02Xh (unhandled)", ah);
            set_carry(true);
            return true;
    }
}

// ============================================================================
// Main Interrupt Dispatcher
// ============================================================================

static bool i86_interrupt_handler(void *ctx, uint8_t num) {
    s_i86.stats.interrupts_processed++;

    // Debug: Log important interrupts from boot sector
    if (num == 0x10 || num == 0x13 || num == 0x16) {
        uint8_t ah = (i86_cpu_get_ax() >> 8) & 0xFF;
        ESP_LOGI(TAG, "INT %02Xh AH=%02Xh", num, ah);
    }

    switch (num) {
        case 0x08: return handle_int08h();
        case 0x09: return handle_int09h();
        case 0x10: return handle_int10h();
        case 0x11: return handle_int11h();
        case 0x12: return handle_int12h();
        case 0x13: return handle_int13h();
        case 0x14: return handle_int14h();
        case 0x15: return handle_int15h();
        case 0x16: return handle_int16h();
        case 0x17: return handle_int17h();
        case 0x19: return handle_int19h();
        case 0x1A: return handle_int1ah();
        case 0x21: return handle_int21h();
        case 0x28: return true;  // DOS idle interrupt
        case 0x2F: return true;  // Multiplex interrupt (stub)
        default:
            if (num < 0x08 || num > 0x0F) {
                ESP_LOGD(TAG, "INT %02Xh (unhandled)", num);
            }
            return false;  // Let CPU handle via IVT
    }
}

// ============================================================================
// BIOS / IVT Setup
// ============================================================================

static void setup_bios(void) {
    uint8_t *mem = i86_cpu_get_memory();

    // Clear conventional memory
    memset(mem, 0, 0x500);  // Clear IVT + BDA

    // Set up IVT - all vectors point to IRET at F000:FF00
    for (int i = 0; i < 256; i++) {
        mem[i * 4 + 0] = 0x00;  // Offset low
        mem[i * 4 + 1] = 0xFF;  // Offset high
        mem[i * 4 + 2] = 0x00;  // Segment low
        mem[i * 4 + 3] = 0xF0;  // Segment high
    }

    // IRET instruction at F000:FF00
    mem[0xFFF00] = 0xCF;  // IRET

    // --- BIOS Data Area (BDA) setup ---

    // Equipment word: 1 floppy drive, 80x25 CGA video
    bda_write16(BDA_EQUIPMENT, 0x0021);

    // Memory size
    bda_write16(BDA_MEMORY_SIZE, 640);

    // Keyboard buffer: empty, head = tail = 0x41E
    bda_write16(BDA_KBD_HEAD, 0x001E);
    bda_write16(BDA_KBD_TAIL, 0x001E);

    // Video mode 03h (80x25 color text)
    bda_write8(BDA_VIDEO_MODE, 0x03);
    bda_write16(BDA_VIDEO_COLS, 80);
    bda_write16(BDA_VIDEO_PAGE_SIZE, 4096);
    bda_write16(BDA_VIDEO_PAGE_OFF, 0);
    bda_write16(BDA_CURSOR_TYPE, 0x0607);  // Cursor: start 6, end 7
    bda_write8(BDA_ACTIVE_PAGE, 0);
    bda_write16(BDA_CRTC_PORT, 0x03D4);

    // Timer ticks (start from 0)
    bda_write16(BDA_TIMER_COUNT, 0);
    bda_write16(BDA_TIMER_COUNT + 2, 0);
    bda_write8(BDA_TIMER_OFLOW, 0);

    // Hard disk count
    int hd_count = 0;
    if (disk_is_mounted(2)) hd_count++;
    if (disk_is_mounted(3)) hd_count++;
    bda_write8(BDA_DISK_COUNT, hd_count);

    // BIOS ROM area (F0000-FFFFF)
    // Entry point at FFFF:0000 → JMP F000:E05B
    mem[0xFFFF0] = 0xEA;  // JMP FAR
    mem[0xFFFF1] = 0x5B;  // Offset low
    mem[0xFFFF2] = 0xE0;  // Offset high
    mem[0xFFFF3] = 0x00;  // Segment low
    mem[0xFFFF4] = 0xF0;  // Segment high

    // Boot code at F000:E05B - invoke INT 19h then halt
    mem[0xFE05B] = 0xCD;  // INT 19h
    mem[0xFE05C] = 0x19;
    mem[0xFE05D] = 0xF4;  // HLT
    mem[0xFE05E] = 0xEB;  // JMP $-2
    mem[0xFE05F] = 0xFC;

    // BIOS date string at FFFF:0005
    const char *bios_date = "02/07/26";
    memcpy(mem + 0xFFFF5, bios_date, 8);

    // Machine ID at FFFF:000E
    mem[0xFFFFE] = 0xFC;  // AT class machine

    // Set up hard disk parameter tables
    // INT 41h vector → HD0 parameters at F000:E401
    if (disk_is_mounted(2)) {
        disk_info_t info;
        if (disk_get_info(2, &info) == ESP_OK) {
            uint32_t table_addr = 0xFE401;
            // Hard disk parameter table format
            mem[table_addr + 0] = info.geometry.cylinders & 0xFF;
            mem[table_addr + 1] = (info.geometry.cylinders >> 8) & 0xFF;
            mem[table_addr + 2] = info.geometry.heads;
            mem[table_addr + 5] = 0xFF;  // Write precompensation (none)
            mem[table_addr + 6] = 0xFF;
            mem[table_addr + 8] = 0x00;  // Control byte
            mem[table_addr + 14] = info.geometry.sectors_per_track;

            // Point INT 41h to this table
            mem[0x41 * 4 + 0] = 0x01;  // Offset = E401
            mem[0x41 * 4 + 1] = 0xE4;
            mem[0x41 * 4 + 2] = 0x00;  // Segment = F000
            mem[0x41 * 4 + 3] = 0xF0;
        }
    }

    // Clear video RAM (B8000-B8FFF = 80x25 text mode)
    uint8_t *video = mem + 0xB8000;
    for (int i = 0; i < 4000; i += 2) {
        video[i] = 0x20;      // Space
        video[i + 1] = 0x07;  // Light gray on black
    }

    ESP_LOGI(TAG, "BIOS initialized: IVT, BDA, BIOS ROM, video RAM");
}

// ============================================================================
// Input Polling (UART + PS/2)
// ============================================================================

static void poll_input(void) {
    // Check PS/2 keyboard
    if (ps2_keyboard_available()) {
        uint8_t ascii = ps2_keyboard_read();
        uint8_t scancode = (ascii < 128) ? ascii_to_scancode[ascii] : 0;

        // Push scancode to keyboard controller (triggers IRQ 1)
        kbd_push_scancode(&s_i86.hw.keyboard, scancode);

        // Also add directly to BIOS keyboard buffer
        int next = (s_i86.bios_kbd_head + 1) % 16;
        if (next != s_i86.bios_kbd_tail) {
            s_i86.bios_kbd_buffer[s_i86.bios_kbd_head] = (scancode << 8) | ascii;
            s_i86.bios_kbd_head = next;
        }
    }

    // Check UART (truly non-blocking via UART driver)
    // Note: fgetc(stdin) BLOCKS and cannot be used here since the
    // emulator runs in a separate task from the console REPL.
    {
        size_t buffered = 0;
        uart_get_buffered_data_len(CONFIG_ESP_CONSOLE_UART_NUM, &buffered);
        if (buffered > 0) {
            uint8_t byte;
            if (uart_read_bytes(CONFIG_ESP_CONSOLE_UART_NUM, &byte, 1, 0) > 0) {
                uint8_t ascii = byte;
                uint8_t scancode = (ascii < 128) ? ascii_to_scancode[ascii] : 0;

                // Push to keyboard controller
                kbd_push_scancode(&s_i86.hw.keyboard, scancode);

                // Add to BIOS keyboard buffer
                int next = (s_i86.bios_kbd_head + 1) % 16;
                if (next != s_i86.bios_kbd_tail) {
                    s_i86.bios_kbd_buffer[s_i86.bios_kbd_head] = (scancode << 8) | ascii;
                    s_i86.bios_kbd_head = next;
                }
            }
        }
    }
}

// ============================================================================
// Video Refresh to LCD
// ============================================================================

static void refresh_lcd(void) {
    if (!video_card_is_initialized()) return;
    if (!s_i86.video_dirty) return;

    int64_t now = esp_timer_get_time();
    if (now - s_i86.last_refresh_us < 33333) return;  // 30 FPS max
    s_i86.last_refresh_us = now;
    s_i86.video_dirty = false;

    // Render CGA text mode to GPU back buffer via pc_hw_render_text()
    // video memory starts at offset 0x18000 from video base (B8000h - A0000h)
    uint8_t *video = i86_cpu_get_video();
    if (!video) return;

    s_i86.hw.cga.needs_refresh = true;
    pc_hw_render_text(&s_i86.hw, video + 0x18000);
}

// ============================================================================
// Public API Implementation
// ============================================================================

esp_err_t i86_init(i86_config_t *config) {
    if (s_i86.initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing i8086 PC emulator...");

    memset(&s_i86, 0, sizeof(s_i86));

    if (config) {
        s_i86.config = *config;
    } else {
        s_i86.config.ram_size = I86_RAM_SIZE;
        s_i86.config.cpu_freq_mhz = 8;
        s_i86.config.enable_fpu = false;
        s_i86.config.enable_sound = false;
        s_i86.config.boot_disk = NULL;
    }

    // Initialize CPU core (allocates 1MB from PSRAM)
    if (i86_cpu_init() != 0) {
        ESP_LOGE(TAG, "Failed to initialize CPU core");
        return ESP_ERR_NO_MEM;
    }

    // Set CPU callbacks
    i86_cpu_set_callbacks(
        NULL,
        port_read_handler,
        port_write_handler,
        video_read8,
        video_write8,
        video_read16,
        video_write16,
        i86_interrupt_handler
    );

    // Initialize PC hardware (PIC, PIT, keyboard, RTC, CGA)
    pc_hw_init(&s_i86.hw, s_i86.config.cpu_freq_mhz);

    // Initialize disk system
    disk_init();

    // Initialize DOS state
    dos_init_state();

    // Initialize keyboard buffer
    s_i86.bios_kbd_head = 0;
    s_i86.bios_kbd_tail = 0;

    // Video defaults
    s_i86.video_mode = 0x03;
    s_i86.cursor_x = 0;
    s_i86.cursor_y = 0;
    s_i86.text_attr = 0x07;
    s_i86.active_page = 0;

    // Timer
    s_i86.timer_ticks = 0;
    s_i86.timer_midnight = false;

    s_i86.mutex = xSemaphoreCreateMutex();
    s_i86.state = I86_STATE_STOPPED;
    s_i86.initialized = true;
    s_i86.stop_requested = false;

    // Set up BIOS
    setup_bios();

    // Auto-mount boot disk if specified
    if (s_i86.config.boot_disk) {
        // Detect if floppy or HD by extension/size
        disk_type_t type = disk_detect_type(s_i86.config.boot_disk);
        int drive = (type == DISK_TYPE_HARD_DISK) ? 2 : 0;
        esp_err_t ret = disk_mount(drive, s_i86.config.boot_disk, false);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Auto-mounted boot disk: %s as drive %c:",
                     s_i86.config.boot_disk, drive < 2 ? 'A' + drive : 'C' + (drive - 2));
        }
    }

    ESP_LOGI(TAG, "i8086 PC emulator initialized");
    ESP_LOGI(TAG, "  RAM: 640 KB");
    ESP_LOGI(TAG, "  Video: 128 KB (CGA text mode)");
    ESP_LOGI(TAG, "  CPU Speed: %lu MHz", (unsigned long)s_i86.config.cpu_freq_mhz);
    ESP_LOGI(TAG, "  Hardware: PIC, PIT, KBD, RTC, CGA");
    ESP_LOGI(TAG, "  DOS: INT 21h with file I/O");

    return ESP_OK;
}

void i86_deinit(void) {
    if (!s_i86.initialized) return;

    ESP_LOGI(TAG, "Shutting down i8086 emulator...");

    s_i86.state = I86_STATE_STOPPED;
    s_i86.stop_requested = true;

    if (s_i86.task_handle) {
        vTaskDelete(s_i86.task_handle);
        s_i86.task_handle = NULL;
    }

    // Close any open DOS files
    for (int i = 0; i < DOS_MAX_FILES; i++) {
        if (s_i86.dos.files[i].open && s_i86.dos.files[i].fp) {
            fclose(s_i86.dos.files[i].fp);
        }
    }
    if (s_i86.dos.find_dir) {
        closedir(s_i86.dos.find_dir);
    }

    disk_deinit();
    i86_cpu_shutdown();

    if (s_i86.mutex) {
        vSemaphoreDelete(s_i86.mutex);
        s_i86.mutex = NULL;
    }

    s_i86.initialized = false;
    ESP_LOGI(TAG, "i8086 emulator shut down");
}

void i86_reset(void) {
    if (!s_i86.initialized) return;

    ESP_LOGI(TAG, "Resetting i8086 CPU...");

    i86_cpu_reset();
    setup_bios();

    s_i86.bios_kbd_head = 0;
    s_i86.bios_kbd_tail = 0;
    s_i86.video_mode = 0x03;
    s_i86.cursor_x = 0;
    s_i86.cursor_y = 0;
    s_i86.text_attr = 0x07;
    s_i86.timer_ticks = 0;
    s_i86.timer_midnight = false;
    s_i86.stop_requested = false;

    memset(&s_i86.stats, 0, sizeof(s_i86.stats));
    s_i86.state = I86_STATE_HALTED;
}

uint32_t i86_run(uint32_t count) {
    if (!s_i86.initialized) return 0;

    s_i86.state = I86_STATE_RUNNING;
    s_i86.stop_requested = false;

    uint32_t total_executed = 0;
    uint32_t batch = 128;  // Execute instructions in batches for efficiency
    bool continuous = (count == 0);

    while (continuous || total_executed < count) {
        if (s_i86.stop_requested) break;
        if (s_i86.state != I86_STATE_RUNNING) break;

        // Diagnostic: trace CS:IP for first instructions and periodically
        uint32_t effective_batch = batch;
        if (total_executed < 50) {
            effective_batch = 1;  // Single-step for detailed trace
            uint16_t cs = i86_cpu_get_cs();
            uint16_t ip = i86_cpu_get_ip();
            uint8_t *mem = i86_cpu_get_memory();
            uint32_t phys = 16 * cs + ip;
            ESP_LOGI(TAG, "TRACE [%lu] %04X:%04X phys=%05X op=%02X %02X %02X %02X halted=%d",
                     (unsigned long)total_executed, cs, ip, phys,
                     mem[phys], mem[phys+1], mem[phys+2], mem[phys+3],
                     i86_cpu_is_halted());
        } else if (total_executed % 50000 == 0) {
            uint16_t cs = i86_cpu_get_cs();
            uint16_t ip = i86_cpu_get_ip();
            ESP_LOGI(TAG, "TRACE [%lu] %04X:%04X halted=%d",
                     (unsigned long)total_executed, cs, ip, i86_cpu_is_halted());
        }

        // Execute a batch of instructions
        uint32_t to_run = effective_batch;
        if (!continuous && (count - total_executed) < effective_batch) {
            to_run = count - total_executed;
        }

        uint32_t executed = i86_cpu_run(to_run);
        total_executed += executed;
        s_i86.stats.instructions_executed += executed;

        // After the MOVSW (instruction ~11), verify the relocation copy
        if (total_executed == 11 || total_executed == 12) {
            uint8_t *mem = i86_cpu_get_memory();
            ESP_LOGI(TAG, "VERIFY[%lu] src@7C00=%02X%02X%02X%02X src@7C22=%02X%02X%02X%02X",
                     (unsigned long)total_executed,
                     mem[0x7C00], mem[0x7C01], mem[0x7C02], mem[0x7C03],
                     mem[0x7C22], mem[0x7C23], mem[0x7C24], mem[0x7C25]);
            ESP_LOGI(TAG, "VERIFY[%lu] dst@27A00=%02X%02X%02X%02X dst@27A22=%02X%02X%02X%02X",
                     (unsigned long)total_executed,
                     mem[0x27A00], mem[0x27A01], mem[0x27A02], mem[0x27A03],
                     mem[0x27A22], mem[0x27A23], mem[0x27A24], mem[0x27A25]);
            ESP_LOGI(TAG, "VERIFY[%lu] ES=%04X DI=%04X SI=%04X CX=%04X DS=%04X",
                     (unsigned long)total_executed,
                     i86_cpu_get_es(), i86_cpu_get_di(), i86_cpu_get_si(),
                     i86_cpu_get_cx(), i86_cpu_get_ds());
        }

        // Tick PC hardware (PIT, PIC, keyboard)
        int irq_vector = pc_hw_tick(&s_i86.hw, executed);
        if (irq_vector >= 0) {
            i86_cpu_irq(irq_vector);
        }

        // Poll input every batch
        poll_input();

        // Refresh LCD periodically
        refresh_lcd();

        // Yield to FreeRTOS every few batches
        if (total_executed % 1024 == 0) {
            vTaskDelay(1);  // Minimal yield
        }

        if (i86_cpu_is_halted()) {
            // CPU is in HLT state - wait for interrupt
            vTaskDelay(pdMS_TO_TICKS(1));

            // Tick hardware to potentially generate an interrupt
            irq_vector = pc_hw_tick(&s_i86.hw, 100);
            if (irq_vector >= 0) {
                i86_cpu_irq(irq_vector);
                // CPU should wake from HLT on interrupt
            }

            poll_input();
        }
    }

    if (i86_cpu_is_halted() && s_i86.state == I86_STATE_RUNNING) {
        s_i86.state = I86_STATE_HALTED;
    }

    return total_executed;
}

void i86_boot_and_run(void) {
    if (!s_i86.initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return;
    }

    // Reset and boot
    i86_reset();

    console_printf("=== IBM PC/XT Emulator ===\n");
    console_printf("CPU: Intel 8086 @ %lu MHz\n", (unsigned long)s_i86.config.cpu_freq_mhz);
    console_printf("RAM: 640 KB\n");
    console_printf("Video: CGA 80x25 text mode\n\n");

    // List mounted disks
    char disk_buf[512];
    disk_list(disk_buf, sizeof(disk_buf));
    console_printf("%s\n", disk_buf);

    // Start running - this will invoke INT 19h via BIOS reset vector
    s_i86.state = I86_STATE_RUNNING;
    i86_run(0);  // Run until stopped

    console_printf("\ni8086 emulator stopped.\n");
}

esp_err_t i86_step(void) {
    if (!s_i86.initialized) return ESP_ERR_INVALID_STATE;

    s_i86.state = I86_STATE_RUNNING;
    i86_cpu_step();
    s_i86.stats.instructions_executed++;

    // Tick hardware
    int irq_vector = pc_hw_tick(&s_i86.hw, 1);
    if (irq_vector >= 0) {
        i86_cpu_irq(irq_vector);
    }

    return ESP_OK;
}

void i86_stop(void) {
    s_i86.stop_requested = true;
    s_i86.state = I86_STATE_STOPPED;
}

i86_state_t i86_get_state(void) {
    return s_i86.state;
}

void i86_get_registers(i86_regs_t *regs) {
    if (!regs) return;
    regs->ax = i86_cpu_get_ax();
    regs->bx = i86_cpu_get_bx();
    regs->cx = i86_cpu_get_cx();
    regs->dx = i86_cpu_get_dx();
    regs->si = i86_cpu_get_si();
    regs->di = i86_cpu_get_di();
    regs->bp = i86_cpu_get_bp();
    regs->sp = i86_cpu_get_sp();
    regs->cs = i86_cpu_get_cs();
    regs->ds = i86_cpu_get_ds();
    regs->es = i86_cpu_get_es();
    regs->ss = i86_cpu_get_ss();
    regs->ip = i86_cpu_get_ip();
    regs->flags = i86_cpu_get_flags();
}

void i86_get_stats(i86_stats_t *stats) {
    if (stats) {
        memcpy(stats, &s_i86.stats, sizeof(i86_stats_t));
    }
}

void i86_get_state_string(char *buffer, size_t size) {
    if (!buffer || size == 0) return;

    i86_regs_t regs;
    i86_get_registers(&regs);

    snprintf(buffer, size,
             "=== i8086 PC Emulator State ===\n"
             "AX=%04X  BX=%04X  CX=%04X  DX=%04X\n"
             "SI=%04X  DI=%04X  BP=%04X  SP=%04X\n"
             "CS=%04X  DS=%04X  ES=%04X  SS=%04X\n"
             "IP=%04X  FLAGS=%04X [%c%c%c%c%c%c%c%c%c]\n"
             "CS:IP=%04X:%04X  Physical=%05"PRIX32"\n"
             "\nState: %s\n"
             "Instructions: %llu\n"
             "Interrupts: %"PRIu32"  DOS calls: %"PRIu32"\n"
             "Disk: %"PRIu32"R/%"PRIu32"W  Files: %"PRIu32"\n"
             "Timer ticks: %"PRIu32"\n",
             regs.ax, regs.bx, regs.cx, regs.dx,
             regs.si, regs.di, regs.bp, regs.sp,
             regs.cs, regs.ds, regs.es, regs.ss,
             regs.ip, regs.flags,
             (regs.flags & 0x800) ? 'O' : '-',
             (regs.flags & 0x400) ? 'D' : '-',
             (regs.flags & 0x200) ? 'I' : '-',
             (regs.flags & 0x100) ? 'T' : '-',
             (regs.flags & 0x080) ? 'S' : '-',
             (regs.flags & 0x040) ? 'Z' : '-',
             (regs.flags & 0x010) ? 'A' : '-',
             (regs.flags & 0x004) ? 'P' : '-',
             (regs.flags & 0x001) ? 'C' : '-',
             regs.cs, regs.ip, (uint32_t)((uint32_t)regs.cs * 16 + regs.ip),
             s_i86.state == I86_STATE_RUNNING ? "Running" :
             s_i86.state == I86_STATE_HALTED ? "Halted" :
             s_i86.state == I86_STATE_STOPPED ? "Stopped" : "Error",
             s_i86.stats.instructions_executed,
             s_i86.stats.interrupts_processed,
             s_i86.stats.dos_calls,
             s_i86.stats.disk_reads,
             s_i86.stats.disk_writes,
             s_i86.stats.files_opened,
             s_i86.timer_ticks);
}

void i86_dump_memory(uint32_t addr, uint32_t length) {
    if (!s_i86.initialized) return;

    uint8_t *mem = i86_cpu_get_memory();
    if (!mem) return;

    if (addr + length > I8086_TOTAL_MEMORY) {
        length = I8086_TOTAL_MEMORY - addr;
    }

    console_printf("Memory dump at 0x%05"PRIX32" (%"PRIu32" bytes):\n", addr, length);

    for (uint32_t i = 0; i < length; i += 16) {
        console_printf("%05"PRIX32": ", addr + i);
        for (uint32_t j = 0; j < 16 && (i + j) < length; j++) {
            console_printf("%02X ", mem[addr + i + j]);
        }
        console_printf(" |");
        for (uint32_t j = 0; j < 16 && (i + j) < length; j++) {
            uint8_t c = mem[addr + i + j];
            console_printf("%c", (c >= 32 && c < 127) ? c : '.');
        }
        console_printf("|\n");
    }
}

esp_err_t i86_load_binary(const uint8_t *data, uint32_t size, uint32_t addr) {
    if (!s_i86.initialized || !data) return ESP_ERR_INVALID_ARG;
    if (addr + size > I8086_TOTAL_MEMORY) return ESP_ERR_INVALID_SIZE;

    uint8_t *mem = i86_cpu_get_memory();
    memcpy(mem + addr, data, size);
    ESP_LOGI(TAG, "Loaded %"PRIu32" bytes at 0x%05"PRIX32, size, addr);
    return ESP_OK;
}

bool i86_interrupt(uint8_t num) {
    if (!s_i86.initialized) return false;
    return i86_cpu_irq(num);
}

void i86_send_key(uint8_t scancode, uint8_t ascii) {
    if (!s_i86.initialized) return;

    // Push to keyboard controller hardware
    kbd_push_scancode(&s_i86.hw.keyboard, scancode);

    // Also add to BIOS keyboard buffer directly
    int next = (s_i86.bios_kbd_head + 1) % 16;
    if (next != s_i86.bios_kbd_tail) {
        s_i86.bios_kbd_buffer[s_i86.bios_kbd_head] = (scancode << 8) | ascii;
        s_i86.bios_kbd_head = next;
    }
}

void i86_send_scancode(uint8_t scancode) {
    if (!s_i86.initialized) return;
    kbd_push_scancode(&s_i86.hw.keyboard, scancode);
}

bool i86_is_initialized(void) {
    return s_i86.initialized;
}

uint8_t *i86_get_memory(void) {
    return i86_cpu_get_memory();
}

uint8_t *i86_get_video_memory(void) {
    return i86_cpu_get_video();
}

esp_err_t i86_mount_disk(int drive, const char *path) {
    if (!s_i86.initialized || drive < 0 || drive >= MAX_DISK_DRIVES) {
        return ESP_ERR_INVALID_ARG;
    }

    if (disk_is_mounted(drive)) {
        disk_unmount(drive);
    }

    esp_err_t err = disk_mount(drive, path, false);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount disk %d from %s", drive, path);
        return err;
    }

    // Update BDA hard disk count
    int hd_count = 0;
    if (disk_is_mounted(2)) hd_count++;
    if (disk_is_mounted(3)) hd_count++;
    if (s_i86.initialized) {
        bda_write8(BDA_DISK_COUNT, hd_count);
    }

    // Update CMOS drive type
    if (drive == 0 || drive == 1) {
        uint8_t floppy_type = 0x40;  // 1.44M default
        disk_info_t info;
        if (disk_get_info(drive, &info) == ESP_OK) {
            switch (info.type) {
                case DISK_TYPE_FLOPPY_360K: floppy_type = 0x10; break;
                case DISK_TYPE_FLOPPY_720K: floppy_type = 0x30; break;
                case DISK_TYPE_FLOPPY_1M2:  floppy_type = 0x20; break;
                case DISK_TYPE_FLOPPY_1M44: floppy_type = 0x40; break;
                case DISK_TYPE_FLOPPY_2M88: floppy_type = 0x50; break;
                default: floppy_type = 0x40; break;
            }
        }
        if (drive == 0) {
            s_i86.hw.rtc.cmos[0x10] = (s_i86.hw.rtc.cmos[0x10] & 0x0F) | (floppy_type & 0xF0);
        } else {
            s_i86.hw.rtc.cmos[0x10] = (s_i86.hw.rtc.cmos[0x10] & 0xF0) | ((floppy_type >> 4) & 0x0F);
        }
    }

    ESP_LOGI(TAG, "Mounted %s as drive %c:", path, drive < 2 ? 'A' + drive : 'C' + (drive - 2));
    return ESP_OK;
}

void i86_refresh_display(void) {
    s_i86.video_dirty = true;
    s_i86.last_refresh_us = 0;  // Force immediate refresh
    refresh_lcd();
}
