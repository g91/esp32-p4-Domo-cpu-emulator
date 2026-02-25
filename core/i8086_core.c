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
#include "usb_keyboard.h"
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

    // Video mode
    uint8_t video_mode;
    uint8_t cursor_x, cursor_y;
    uint8_t text_attr;
    uint8_t active_page;

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
        // Mark video dirty if writing to CGA video memory area
        // Text mode uses B8000-B8FFF, graphics modes use B8000-BBFFF (16KB)
        if (addr >= 0xB8000 && addr < 0xBC000) {
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
// BDA Keyboard Buffer Helpers
// Buffer at 0040:001E-003D (16 words), pointers are offsets from seg 0040h
// ============================================================================

static bool bda_kbd_push(uint16_t key) {
    uint16_t tail = bda_read16(BDA_KBD_TAIL);
    uint16_t next_tail = tail + 2;
    if (next_tail >= 0x003E) next_tail = 0x001E;
    if (next_tail == bda_read16(BDA_KBD_HEAD)) return false; // Full
    bda_write16(0x0400 + tail, key);
    bda_write16(BDA_KBD_TAIL, next_tail);
    return true;
}

static bool bda_kbd_pop(uint16_t *key) {
    uint16_t head = bda_read16(BDA_KBD_HEAD);
    if (head == bda_read16(BDA_KBD_TAIL)) return false; // Empty
    *key = bda_read16(0x0400 + head);
    head += 2;
    if (head >= 0x003E) head = 0x001E;
    bda_write16(BDA_KBD_HEAD, head);
    return true;
}

static bool bda_kbd_peek(uint16_t *key) {
    uint16_t head = bda_read16(BDA_KBD_HEAD);
    if (head == bda_read16(BDA_KBD_TAIL)) return false;
    *key = bda_read16(0x0400 + head);
    return true;
}

static bool bda_kbd_empty(void) {
    return bda_read16(BDA_KBD_HEAD) == bda_read16(BDA_KBD_TAIL);
}

static void bda_kbd_flush(void) {
    bda_write16(BDA_KBD_HEAD, 0x001E);
    bda_write16(BDA_KBD_TAIL, 0x001E);
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

    // Sync CGA hardware cursor
    s_i86.hw.cga.cursor_pos = s_i86.cursor_y * 80 + s_i86.cursor_x;
    s_i86.hw.cga.cursor_visible = true;
    bda_write16(BDA_CURSOR_POS, (s_i86.cursor_y << 8) | s_i86.cursor_x);
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
    // When the IVT still points to our BIOS, handle via callback for speed.
    // When FreeDOS has hooked INT 08h, the callback returns false and the
    // CPU dispatches through IVT. FreeDOS chains back to F000:E008 (real
    // x86 code) which does BDA update + INT 1Ch + EOI + IRET.

    // Update BDA timer count directly in memory
    uint8_t *mem = i86_cpu_get_memory();
    uint32_t ticks = mem[BDA_TIMER_COUNT] |
                     ((uint32_t)mem[BDA_TIMER_COUNT + 1] << 8) |
                     ((uint32_t)mem[BDA_TIMER_COUNT + 2] << 16) |
                     ((uint32_t)mem[BDA_TIMER_COUNT + 3] << 24);
    ticks++;

    // Check for midnight (1573040 ticks ~= 24 hours at 18.2 Hz)
    if (ticks >= 1573040) {
        ticks = 0;
        mem[BDA_TIMER_OFLOW] = 1;
    }

    mem[BDA_TIMER_COUNT]     = ticks & 0xFF;
    mem[BDA_TIMER_COUNT + 1] = (ticks >> 8) & 0xFF;
    mem[BDA_TIMER_COUNT + 2] = (ticks >> 16) & 0xFF;
    mem[BDA_TIMER_COUNT + 3] = (ticks >> 24) & 0xFF;

    // Send EOI to PIC
    pc_hw_write_port(&s_i86.hw, 0x20, 0x20);

    // Note: We don't chain INT 1Ch from the callback because we can't
    // invoke a software interrupt from here. The x86 code at F000:E008
    // handles INT 1Ch chaining when FreeDOS chains back to us.
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

        // Add to BDA keyboard buffer (the authoritative keyboard buffer)
        // BDA buffer is at 0040:001E to 0040:003D (16 words)
        // Head/tail are offsets relative to segment 0040h
        uint16_t tail = bda_read16(BDA_KBD_TAIL);
        uint16_t next_tail = tail + 2;
        if (next_tail >= 0x003E) next_tail = 0x001E;
        uint16_t head = bda_read16(BDA_KBD_HEAD);
        if (next_tail != head) {  // Buffer not full
            // Write key word (scancode:ascii) to BDA buffer
            bda_write16(0x0400 + tail, (make_code << 8) | ascii);
            bda_write16(BDA_KBD_TAIL, next_tail);
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
        {
            uint8_t mode = al & 0x7F;  // Bit 7 = don't clear
            bool no_clear = (al & 0x80) != 0;
            s_i86.video_mode = mode;

            // Set CGA mode register based on BIOS mode number
            switch (mode) {
                case 0x00: // 40x25 text, B/W
                case 0x01: // 40x25 text, color
                    s_i86.hw.cga.mode_reg = 0x2C; // text, 40col, enable, blink
                    bda_write16(BDA_VIDEO_COLS, 40);
                    bda_write16(BDA_VIDEO_PAGE_SIZE, 2048);
                    if (!no_clear) {
                        memset(video + 0x18000, 0, 2048);
                        for (int i = 0; i < 2000; i += 2) {
                            video[0x18000 + i] = 0x20;
                            video[0x18000 + i + 1] = 0x07;
                        }
                    }
                    break;
                case 0x02: // 80x25 text, B/W
                case 0x03: // 80x25 text, color
                    s_i86.hw.cga.mode_reg = 0x29; // text, 80col, enable
                    bda_write16(BDA_VIDEO_COLS, 80);
                    bda_write16(BDA_VIDEO_PAGE_SIZE, 4096);
                    if (!no_clear) {
                        for (int i = 0; i < 4000; i += 2) {
                            video[0x18000 + i] = 0x20;
                            video[0x18000 + i + 1] = 0x07;
                        }
                    }
                    break;
                case 0x04: // 320x200 graphics, 4 colors
                case 0x05: // 320x200 graphics, 4 colors (alt palette)
                    s_i86.hw.cga.mode_reg = 0x0A; // graphics, enable
                    bda_write16(BDA_VIDEO_COLS, 40);
                    bda_write16(BDA_VIDEO_PAGE_SIZE, 16384);
                    if (!no_clear) {
                        memset(video + 0x18000, 0, 16384);
                    }
                    break;
                case 0x06: // 640x200 graphics, 2 colors
                    s_i86.hw.cga.mode_reg = 0x1A; // graphics, 640, enable
                    bda_write16(BDA_VIDEO_COLS, 80);
                    bda_write16(BDA_VIDEO_PAGE_SIZE, 16384);
                    if (!no_clear) {
                        memset(video + 0x18000, 0, 16384);
                    }
                    break;
                default:
                    // Unknown mode - treat as 80x25 text
                    s_i86.hw.cga.mode_reg = 0x29;
                    bda_write16(BDA_VIDEO_COLS, 80);
                    bda_write16(BDA_VIDEO_PAGE_SIZE, 4096);
                    break;
            }

            s_i86.cursor_x = 0;
            s_i86.cursor_y = 0;
            s_i86.text_attr = 0x07;
            s_i86.active_page = 0;
            s_i86.hw.cga.mem_offset = 0;
            bda_write8(BDA_VIDEO_MODE, mode);
            bda_write16(BDA_VIDEO_PAGE_OFF, 0);
            bda_write8(BDA_ACTIVE_PAGE, 0);
            bda_write16(BDA_CURSOR_POS, 0);
            bda_write8(BDA_CRT_MODE, s_i86.hw.cga.mode_reg);
            bda_write8(BDA_CRT_PALETTE, s_i86.hw.cga.color_reg);
            s_i86.video_dirty = true;
            s_i86.hw.cga.needs_refresh = true;
            ESP_LOGD(TAG, "INT 10h: Set video mode %02Xh (CGA reg=%02Xh)", mode, s_i86.hw.cga.mode_reg);
            return true;
        }

        case 0x01: // Set cursor shape
            bda_write16(BDA_CURSOR_TYPE, cx);
            return true;

        case 0x02: // Set cursor position
            s_i86.cursor_x = dl;
            s_i86.cursor_y = dh;
            // Update BDA cursor position for page bh
            bda_write16(BDA_CURSOR_POS + (bh * 2), (dh << 8) | dl);
            // Sync CGA hardware cursor so renderer draws it
            s_i86.hw.cga.cursor_pos = dh * 80 + dl;
            s_i86.hw.cga.cursor_visible = true;
            s_i86.video_dirty = true;
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

        case 0x0B: // Set color palette
        {
            if (bh == 0x00) {
                // BL = background/border color (bits 0-3)
                s_i86.hw.cga.color_reg = (s_i86.hw.cga.color_reg & 0xF0) | (bl & 0x0F);
            } else if (bh == 0x01) {
                // BL = palette id (0 or 1)
                if (bl & 1) {
                    s_i86.hw.cga.color_reg |= 0x20;  // Palette 1 (cyan/magenta/white)
                } else {
                    s_i86.hw.cga.color_reg &= ~0x20;  // Palette 0 (green/red/brown)
                }
            }
            bda_write8(BDA_CRT_PALETTE, s_i86.hw.cga.color_reg);
            s_i86.video_dirty = true;
            s_i86.hw.cga.needs_refresh = true;
            return true;
        }

        case 0x0C: // Write pixel (graphics modes)
        {
            uint16_t px = i86_cpu_get_cx();  // CX = column
            uint16_t py = i86_cpu_get_dx();  // DX = row
            uint8_t color = al;

            if (s_i86.video_mode == 0x04 || s_i86.video_mode == 0x05) {
                // 320x200, 4 colors: 2 bits per pixel
                if (px < 320 && py < 200) {
                    uint32_t offset = ((py & 1) ? 0x2000 : 0x0000) + (py >> 1) * 80 + (px >> 2);
                    int shift = 6 - (px & 3) * 2;
                    uint8_t mask = ~(0x03 << shift);
                    uint8_t *p = &video[0x18000 + offset];
                    if (color & 0x80) {
                        // XOR mode
                        *p ^= (color & 0x03) << shift;
                    } else {
                        *p = (*p & mask) | ((color & 0x03) << shift);
                    }
                    s_i86.video_dirty = true;
                    s_i86.hw.cga.needs_refresh = true;
                }
            } else if (s_i86.video_mode == 0x06) {
                // 640x200, 2 colors: 1 bit per pixel
                if (px < 640 && py < 200) {
                    uint32_t offset = ((py & 1) ? 0x2000 : 0x0000) + (py >> 1) * 80 + (px >> 3);
                    int bit = 7 - (px & 7);
                    uint8_t *p = &video[0x18000 + offset];
                    if (color & 0x80) {
                        *p ^= (1 << bit);
                    } else if (color & 1) {
                        *p |= (1 << bit);
                    } else {
                        *p &= ~(1 << bit);
                    }
                    s_i86.video_dirty = true;
                    s_i86.hw.cga.needs_refresh = true;
                }
            }
            return true;
        }

        case 0x0D: // Read pixel (graphics modes)
        {
            uint16_t px = i86_cpu_get_cx();  // CX = column
            uint16_t py = i86_cpu_get_dx();  // DX = row
            uint8_t color = 0;

            if (s_i86.video_mode == 0x04 || s_i86.video_mode == 0x05) {
                if (px < 320 && py < 200) {
                    uint32_t offset = ((py & 1) ? 0x2000 : 0x0000) + (py >> 1) * 80 + (px >> 2);
                    int shift = 6 - (px & 3) * 2;
                    color = (video[0x18000 + offset] >> shift) & 0x03;
                }
            } else if (s_i86.video_mode == 0x06) {
                if (px < 640 && py < 200) {
                    uint32_t offset = ((py & 1) ? 0x2000 : 0x0000) + (py >> 1) * 80 + (px >> 3);
                    int bit = 7 - (px & 7);
                    color = (video[0x18000 + offset] >> bit) & 0x01;
                }
            }
            i86_cpu_set_ax((i86_cpu_get_ax() & 0xFF00) | color);
            return true;
        }

        case 0x0E: // TTY output
            pc_putchar(al);
            return true;

        case 0x0F: // Get video mode
            i86_cpu_set_ax((80 << 8) | s_i86.video_mode);
            i86_cpu_set_bx((s_i86.active_page << 8) | (i86_cpu_get_bx() & 0xFF));
            return true;

        case 0x10: // VGA palette functions
        {
            switch (al) {
                case 0x00: // Set individual palette register
                case 0x01: // Set overscan (border) color
                case 0x02: // Set all palette registers + overscan
                case 0x03: // Toggle blink/intensity bit
                case 0x07: // Read individual palette register
                case 0x08: // Read overscan register
                case 0x09: // Read all palette registers
                    break;  // Accept silently
                case 0x10: // Set individual DAC register
                case 0x12: // Set block of DAC registers
                    break;  // Accept silently
                case 0x15: // Read individual DAC register
                    i86_cpu_set_dx(0x0000);  // Green=0
                    i86_cpu_set_cx(0x0000);  // Red=0, Blue=0
                    break;
                case 0x17: // Read block of DAC registers
                    break;
                case 0x1A: // Get/set DAC color page state
                    if (bl == 0x00) {
                        i86_cpu_set_bx(0x0000);  // Paging mode 0, page 0
                    }
                    break;
                default:
                    break;
            }
            return true;
        }

        case 0x11: // Character generator
            if (al == 0x30) {
                // Get font information
                uint8_t info_sel = (i86_cpu_get_bx() >> 8) & 0xFF;
                (void)info_sel;
                i86_cpu_set_cx(8);    // Bytes per character (CGA 8x8)
                i86_cpu_set_dx(24);   // Rows - 1
                i86_cpu_set_es(0xF000);
                i86_cpu_set_bp(0xFA6E); // Font table pointer in BIOS
            }
            // Other sub-functions (load font, etc.) - accept silently
            return true;

        case 0x12: // Alternate function select
        {
            uint8_t bl12 = i86_cpu_get_bx() & 0xFF;
            if (bl12 == 0x10) {
                // Get EGA info
                i86_cpu_set_bx(0x0003); // 256K, color, EGA active
                i86_cpu_set_cx(0x0009); // Feature bits, switch settings
            } else if (bl12 == 0x30) {
                // Select vertical resolution (for text modes)
                // AL=0: 200 lines, AL=1: 350, AL=2: 400
                // Accept silently (we always use our fixed resolution)
            } else if (bl12 == 0x34) {
                // Cursor emulation
                i86_cpu_set_ax(0x1200);  // Function supported
            } else if (bl12 == 0x36) {
                // Video refresh control
                // AL=0: enable, AL=1: disable
            }
            return true;
        }

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
            if (al == 0x00) {
                // Get display combination
                i86_cpu_set_ax(0x001A); // Function supported
                i86_cpu_set_bx(0x0008); // BL=active display (VGA color), BH=alternate
            } else {
                // Set display combination - accept silently
                i86_cpu_set_ax(0x001A);
            }
            return true;

        case 0x1B: // Functionality/state information
        {
            // Fill buffer at ES:DI with video state info
            uint8_t *mem1b = i86_cpu_get_memory();
            uint32_t buf1b = ((uint32_t)i86_cpu_get_es() * 16 + i86_cpu_get_di()) & 0xFFFFF;
            // Minimal state info - 64 bytes
            memset(mem1b + buf1b, 0, 64);
            // Static functionality table pointer (just point to zeroes)
            mem1b[buf1b + 0] = 0x00;  // Offset low
            mem1b[buf1b + 1] = 0x00;  // Offset high
            mem1b[buf1b + 2] = 0x00;  // Segment low
            mem1b[buf1b + 3] = 0x00;  // Segment high
            // Current video mode
            mem1b[buf1b + 4] = s_i86.video_mode;
            // Number of columns
            mem1b[buf1b + 5] = 80; mem1b[buf1b + 6] = 0;
            // Active display page
            mem1b[buf1b + 34] = s_i86.active_page;
            // Cursor positions
            mem1b[buf1b + 35] = s_i86.cursor_x;
            mem1b[buf1b + 36] = s_i86.cursor_y;
            // Number of rows
            mem1b[buf1b + 42] = 25;
            i86_cpu_set_ax(0x001B);  // Function supported
            return true;
        }

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
                    ESP_LOGW(TAG, "INT13 Read: CHS=%d/%d/%d FAILED (bad CHS) drv=0x%02X",
                             cylinder, head, sector, dl);
                    i86_cpu_set_ax(0x0400);  // Sector not found
                    set_carry(true);
                    return true;
                }
                uint32_t phys_addr = ((uint32_t)es * 16 + bx) & 0xFFFFF;
                uint8_t *buffer = i86_cpu_get_memory() + phys_addr;

                ESP_LOGI(TAG, "INT13 Read: CHS=%d/%d/%d LBA=%d cnt=%d -> %05X drv=0x%02X",
                         cylinder, head, sector, (int)lba, count, phys_addr, dl);

                int sectors_read = disk_read_sectors(drive_idx, (uint32_t)lba, count, buffer);
                if (sectors_read > 0) {
                    i86_cpu_set_ax(sectors_read & 0xFF);
                    s_i86.stats.disk_reads++;
                    set_carry(false);
                } else {
                    ESP_LOGW(TAG, "INT13 Read FAILED: LBA=%d cnt=%d", (int)lba, count);
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
                uint32_t phys_addr_w = ((uint32_t)es * 16 + bx) & 0xFFFFF;
                uint8_t *buffer = i86_cpu_get_memory() + phys_addr_w;

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

        case 0x41: // INT 13h Extensions - Check extensions present
        {
            if (dl >= 0x80 && i86_cpu_get_bx() == 0x55AA) {
                // Extensions supported: fixed disk access (subset 1)
                i86_cpu_set_ax(0x3000);  // Version 3.0
                i86_cpu_set_bx(0xAA55);  // Signature
                i86_cpu_set_cx(0x0001);  // Subset 1: fixed disk access
                set_carry(false);
            } else {
                set_carry(true);
            }
            return true;
        }

        case 0x42: // INT 13h Extensions - Extended Read
        {
            if (dl < 0x80) {
                set_carry(true);
                i86_cpu_set_ax(0x0100);
                return true;
            }
            // DAP (Disk Address Packet) at DS:SI
            uint8_t *mem2 = i86_cpu_get_memory();
            uint32_t dap_addr = ((uint32_t)i86_cpu_get_ds() * 16 + i86_cpu_get_si()) & 0xFFFFF;
            // uint8_t dap_size = mem2[dap_addr];
            uint16_t dap_count = mem2[dap_addr + 2] | ((uint16_t)mem2[dap_addr + 3] << 8);
            uint16_t dap_buf_off = mem2[dap_addr + 4] | ((uint16_t)mem2[dap_addr + 5] << 8);
            uint16_t dap_buf_seg = mem2[dap_addr + 6] | ((uint16_t)mem2[dap_addr + 7] << 8);
            uint32_t dap_lba = mem2[dap_addr + 8] | ((uint32_t)mem2[dap_addr + 9] << 8) |
                               ((uint32_t)mem2[dap_addr + 10] << 16) | ((uint32_t)mem2[dap_addr + 11] << 24);

            uint32_t buf_phys = ((uint32_t)dap_buf_seg * 16 + dap_buf_off) & 0xFFFFF;
            uint8_t *buf = mem2 + buf_phys;

            ESP_LOGI(TAG, "INT13 ExtRead: LBA=%u cnt=%u -> %04X:%04X (%05X) drv=0x%02X",
                     dap_lba, dap_count, dap_buf_seg, dap_buf_off, buf_phys, dl);

            int sectors_read2 = disk_read_sectors(drive_idx, dap_lba, dap_count, buf);
            if (sectors_read2 > 0) {
                i86_cpu_set_ax(0x0000);
                s_i86.stats.disk_reads++;
                set_carry(false);
            } else {
                ESP_LOGW(TAG, "INT13 ExtRead FAILED: LBA=%u cnt=%u", dap_lba, dap_count);
                i86_cpu_set_ax(0x0400);
                set_carry(true);
            }
            return true;
        }

        case 0x43: // INT 13h Extensions - Extended Write
        {
            if (dl < 0x80) {
                set_carry(true);
                i86_cpu_set_ax(0x0100);
                return true;
            }
            uint8_t *mem2 = i86_cpu_get_memory();
            uint32_t dap_addr = ((uint32_t)i86_cpu_get_ds() * 16 + i86_cpu_get_si()) & 0xFFFFF;
            uint16_t dap_count = mem2[dap_addr + 2] | ((uint16_t)mem2[dap_addr + 3] << 8);
            uint16_t dap_buf_off = mem2[dap_addr + 4] | ((uint16_t)mem2[dap_addr + 5] << 8);
            uint16_t dap_buf_seg = mem2[dap_addr + 6] | ((uint16_t)mem2[dap_addr + 7] << 8);
            uint32_t dap_lba = mem2[dap_addr + 8] | ((uint32_t)mem2[dap_addr + 9] << 8) |
                               ((uint32_t)mem2[dap_addr + 10] << 16) | ((uint32_t)mem2[dap_addr + 11] << 24);

            uint32_t buf_phys = ((uint32_t)dap_buf_seg * 16 + dap_buf_off) & 0xFFFFF;
            uint8_t *buf = mem2 + buf_phys;

            int sectors_written2 = disk_write_sectors(drive_idx, dap_lba, dap_count, buf);
            if (sectors_written2 > 0) {
                i86_cpu_set_ax(0x0000);
                s_i86.stats.disk_writes++;
                set_carry(false);
            } else {
                i86_cpu_set_ax(0x0400);
                set_carry(true);
            }
            return true;
        }

        case 0x48: // INT 13h Extensions - Get drive parameters
        {
            if (dl < 0x80) {
                set_carry(true);
                return true;
            }
            disk_info_t info2;
            if (disk_get_info(drive_idx, &info2) == ESP_OK && info2.mounted) {
                uint8_t *mem2 = i86_cpu_get_memory();
                uint32_t buf_addr = ((uint32_t)i86_cpu_get_ds() * 16 + i86_cpu_get_si()) & 0xFFFFF;
                // Result buffer size
                mem2[buf_addr + 0] = 0x1A; mem2[buf_addr + 1] = 0x00; // Size = 26
                // Information flags
                mem2[buf_addr + 2] = 0x02; mem2[buf_addr + 3] = 0x00; // DMA boundary errors handled
                // Cylinders
                uint32_t cyls = info2.geometry.cylinders;
                mem2[buf_addr + 4] = cyls & 0xFF;
                mem2[buf_addr + 5] = (cyls >> 8) & 0xFF;
                mem2[buf_addr + 6] = (cyls >> 16) & 0xFF;
                mem2[buf_addr + 7] = (cyls >> 24) & 0xFF;
                // Heads
                uint32_t heads = info2.geometry.heads;
                mem2[buf_addr + 8] = heads & 0xFF;
                mem2[buf_addr + 9] = (heads >> 8) & 0xFF;
                mem2[buf_addr + 10] = (heads >> 16) & 0xFF;
                mem2[buf_addr + 11] = (heads >> 24) & 0xFF;
                // Sectors per track
                uint32_t spt = info2.geometry.sectors_per_track;
                mem2[buf_addr + 12] = spt & 0xFF;
                mem2[buf_addr + 13] = (spt >> 8) & 0xFF;
                mem2[buf_addr + 14] = (spt >> 16) & 0xFF;
                mem2[buf_addr + 15] = (spt >> 24) & 0xFF;
                // Total sectors (64-bit)
                uint32_t total = info2.geometry.total_sectors;
                mem2[buf_addr + 16] = total & 0xFF;
                mem2[buf_addr + 17] = (total >> 8) & 0xFF;
                mem2[buf_addr + 18] = (total >> 16) & 0xFF;
                mem2[buf_addr + 19] = (total >> 24) & 0xFF;
                mem2[buf_addr + 20] = 0; mem2[buf_addr + 21] = 0;
                mem2[buf_addr + 22] = 0; mem2[buf_addr + 23] = 0;
                // Bytes per sector
                mem2[buf_addr + 24] = 0x00; mem2[buf_addr + 25] = 0x02; // 512
                i86_cpu_set_ax(0x0000);
                set_carry(false);
            } else {
                set_carry(true);
            }
            return true;
        }

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
        case 0x24: // A20 gate control
        {
            uint8_t al_val = i86_cpu_get_ax() & 0xFF;
            switch (al_val) {
                case 0x00: // Disable A20
                case 0x01: // Enable A20
                    // We always keep A20 enabled (no gate in our emulation)
                    set_carry(false);
                    i86_cpu_set_ax(i86_cpu_get_ax() & 0xFF00); // AH=0 success
                    break;
                case 0x02: // Get A20 status
                    set_carry(false);
                    i86_cpu_set_ax((i86_cpu_get_ax() & 0xFF00) | 0x01); // AL=1 (enabled)
                    break;
                case 0x03: // Query A20 support
                    set_carry(false);
                    i86_cpu_set_ax(i86_cpu_get_ax() & 0xFF00);
                    i86_cpu_set_bx(0x0003); // Keyboard controller + port 92
                    break;
                default:
                    set_carry(true);
                    break;
            }
            return true;
        }

        case 0x41: // Wait for external event
            set_carry(true);  // Function not supported
            return true;

        case 0x4F: // Keyboard intercept
            set_carry(true);  // Not handled, let default processing occur
            return true;

        case 0x53: // APM (Advanced Power Management) - FreeDOS checks this
        {
            uint8_t al_val = i86_cpu_get_ax() & 0xFF;
            if (al_val == 0x00) {
                // APM installation check
                i86_cpu_set_ax(0x0102); // APM version 1.2
                i86_cpu_set_bx(0x504D); // "PM" signature
                i86_cpu_set_cx(0x0003); // Flags: 16-bit + 32-bit
                set_carry(false);
            } else {
                set_carry(true); // Not supported
            }
            return true;
        }

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
            // Poll for key using BDA keyboard buffer - no timeout.
            while (bda_read16(BDA_KBD_HEAD) == bda_read16(BDA_KBD_TAIL)) {
                vTaskDelay(pdMS_TO_TICKS(10));
                poll_input();

                // Tick hardware so timer IRQ continues firing
                int irq_vector = pc_hw_tick(&s_i86.hw, 100);
                if (irq_vector >= 0) {
                    i86_cpu_irq(irq_vector);
                }

                if (s_i86.state != I86_STATE_RUNNING || s_i86.stop_requested) {
                    i86_cpu_set_ax(0);
                    return true;
                }
            }
            // Read key from BDA buffer
            uint16_t head = bda_read16(BDA_KBD_HEAD);
            uint16_t key = bda_read16(0x0400 + head);
            head += 2;
            if (head >= 0x003E) head = 0x001E;
            bda_write16(BDA_KBD_HEAD, head);
            i86_cpu_set_ax(key);
            return true;
        }

        case 0x01: // Check for keypress (non-blocking)
        case 0x11: // Enhanced check
        {
            uint16_t head = bda_read16(BDA_KBD_HEAD);
            uint16_t tail = bda_read16(BDA_KBD_TAIL);
            if (head != tail) {
                uint16_t key = bda_read16(0x0400 + head);
                i86_cpu_set_ax(key);
                i86_cpu_set_flag(FLAG_IDX_ZF, 0);  // Key available
            } else {
                i86_cpu_set_ax(0x0000);
                i86_cpu_set_flag(FLAG_IDX_ZF, 1);  // No key
            }
            return true;
        }

        case 0x02: // Get shift flags
            i86_cpu_set_ax(s_i86.hw.keyboard.shift_flags);
            return true;

        case 0x03: // Set typematic rate
            return true;

        case 0x05: // Store keypress in buffer
        {
            if (bda_kbd_push(i86_cpu_get_cx())) {
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
        case 0x00: // Get tick count - read from BDA (authoritative source)
        {
            uint8_t *mem = i86_cpu_get_memory();
            uint32_t ticks = mem[BDA_TIMER_COUNT] |
                             ((uint32_t)mem[BDA_TIMER_COUNT + 1] << 8) |
                             ((uint32_t)mem[BDA_TIMER_COUNT + 2] << 16) |
                             ((uint32_t)mem[BDA_TIMER_COUNT + 3] << 24);
            i86_cpu_set_cx((ticks >> 16) & 0xFFFF);
            i86_cpu_set_dx(ticks & 0xFFFF);
            uint8_t midnight = mem[BDA_TIMER_OFLOW];
            i86_cpu_set_ax(midnight ? 1 : 0);
            mem[BDA_TIMER_OFLOW] = 0;  // Clear midnight flag after read
            return true;
        }

        case 0x01: // Set tick count - write to BDA
        {
            uint8_t *mem = i86_cpu_get_memory();
            uint32_t ticks = ((uint32_t)i86_cpu_get_cx() << 16) | i86_cpu_get_dx();
            mem[BDA_TIMER_COUNT]     = ticks & 0xFF;
            mem[BDA_TIMER_COUNT + 1] = (ticks >> 8) & 0xFF;
            mem[BDA_TIMER_COUNT + 2] = (ticks >> 16) & 0xFF;
            mem[BDA_TIMER_COUNT + 3] = (ticks >> 24) & 0xFF;
            return true;
        }

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
            uint16_t key;
            while (!bda_kbd_pop(&key)) {
                vTaskDelay(pdMS_TO_TICKS(10));
                poll_input();
                if (s_i86.state != I86_STATE_RUNNING || s_i86.stop_requested) {
                    i86_cpu_set_ax(0);
                    return true;
                }
            }
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
                uint16_t key;
                if (bda_kbd_pop(&key)) {
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
            uint16_t key;
            while (!bda_kbd_pop(&key)) {
                vTaskDelay(pdMS_TO_TICKS(10));
                poll_input();
                if (s_i86.state != I86_STATE_RUNNING || s_i86.stop_requested) return true;
            }
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
            uint32_t buf_addr = ((uint32_t)i86_cpu_get_ds() * 16 + i86_cpu_get_dx()) & 0xFFFFF;
            uint8_t max_len = mem[buf_addr];
            uint8_t pos = 0;

            while (pos < max_len - 1) {
                uint16_t key;
                while (!bda_kbd_pop(&key)) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                    poll_input();
                    if (s_i86.state != I86_STATE_RUNNING || s_i86.stop_requested) {
                        mem[buf_addr + 1] = pos;
                        mem[buf_addr + 2 + pos] = '\r';
                        return true;
                    }
                }
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
            if (!bda_kbd_empty()) {
                i86_cpu_set_ax(0x0BFF);
            } else {
                i86_cpu_set_ax(0x0B00);
            }
            return true;

        case 0x0C: // Clear keyboard buffer and invoke input function
            bda_kbd_flush();
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

        case 0x1B: // Get allocation info for default drive
        case 0x1C: // Get allocation info for specific drive
        {
            // Return fake drive info: 64 sectors/cluster, 512 bytes/sector
            i86_cpu_set_ax(64);     // Sectors per cluster
            i86_cpu_set_cx(512);    // Bytes per sector
            i86_cpu_set_dx(20000);  // Total clusters
            // DS:BX should point to media ID byte - use a byte in BDA area
            uint8_t *mem1b = i86_cpu_get_memory();
            mem1b[0x0504] = 0xF8;  // Hard disk media ID
            i86_cpu_set_ds(0x0050);
            i86_cpu_set_bx(0x0004);
            return true;
        }

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

        case 0x26: // Create new PSP
        {
            // Copy current PSP (256 bytes) to segment in DX
            uint8_t *mem26 = i86_cpu_get_memory();
            uint16_t new_psp_seg = i86_cpu_get_dx();
            uint32_t src_addr = (uint32_t)s_i86.dos.psp_segment * 16;
            uint32_t dst_addr = (uint32_t)new_psp_seg * 16;
            if (src_addr + 256 <= I8086_TOTAL_MEMORY && dst_addr + 256 <= I8086_TOTAL_MEMORY) {
                memcpy(mem26 + dst_addr, mem26 + src_addr, 256);
                // Update INT 22h (terminate addr) in new PSP
                mem26[dst_addr + 0x0A] = i86_cpu_get_ip() & 0xFF;
                mem26[dst_addr + 0x0B] = (i86_cpu_get_ip() >> 8) & 0xFF;
                mem26[dst_addr + 0x0C] = i86_cpu_get_cs() & 0xFF;
                mem26[dst_addr + 0x0D] = (i86_cpu_get_cs() >> 8) & 0xFF;
            }
            return true;
        }

        case 0x29: // Parse filename into FCB
        {
            uint8_t *mem29 = i86_cpu_get_memory();
            uint32_t src29 = ((uint32_t)i86_cpu_get_ds() * 16 + i86_cpu_get_si()) & 0xFFFFF;
            uint32_t dst29 = ((uint32_t)i86_cpu_get_es() * 16 + i86_cpu_get_di()) & 0xFFFFF;

            // Skip leading separators if AL bit 0 set
            int pos = 0;
            if (al & 0x01) {
                while (mem29[src29 + pos] == ' ' || mem29[src29 + pos] == '\t') pos++;
            }

            // Initialize FCB with blanks
            mem29[dst29 + 0] = 0;  // Drive number (0=default)
            memset(mem29 + dst29 + 1, ' ', 11);  // Filename + extension

            // Check for drive letter
            if (mem29[src29 + pos + 1] == ':') {
                uint8_t drv = toupper(mem29[src29 + pos]) - 'A' + 1;
                mem29[dst29 + 0] = drv;
                pos += 2;
            }

            // Parse filename (up to 8 chars)
            int fn_pos = 0;
            while (fn_pos < 8 && mem29[src29 + pos] != '.' &&
                   mem29[src29 + pos] != ' ' && mem29[src29 + pos] != '\0' &&
                   mem29[src29 + pos] != '\r' && mem29[src29 + pos] != '/' &&
                   mem29[src29 + pos] != '\\') {
                if (mem29[src29 + pos] == '*') {
                    while (fn_pos < 8) mem29[dst29 + 1 + fn_pos++] = '?';
                } else {
                    mem29[dst29 + 1 + fn_pos++] = toupper(mem29[src29 + pos]);
                }
                pos++;
            }

            // Parse extension
            if (mem29[src29 + pos] == '.') {
                pos++;
                int ext_pos = 0;
                while (ext_pos < 3 && mem29[src29 + pos] != ' ' &&
                       mem29[src29 + pos] != '\0' && mem29[src29 + pos] != '\r') {
                    if (mem29[src29 + pos] == '*') {
                        while (ext_pos < 3) mem29[dst29 + 9 + ext_pos++] = '?';
                    } else {
                        mem29[dst29 + 9 + ext_pos++] = toupper(mem29[src29 + pos]);
                    }
                    pos++;
                }
            }

            // Update SI to point past parsed name
            i86_cpu_set_si((i86_cpu_get_si() + pos) & 0xFFFF);

            // AL return: 0=no wildcards, 1=wildcards found
            bool has_wild = false;
            for (int w = 1; w <= 11; w++) {
                if (mem29[dst29 + w] == '?') { has_wild = true; break; }
            }
            i86_cpu_set_ax((ah << 8) | (has_wild ? 0x01 : 0x00));
            return true;
        }

        case 0x2F: // Get DTA address
            i86_cpu_set_es(s_i86.dos.dta_segment);
            i86_cpu_set_bx(s_i86.dos.dta_offset);
            return true;

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

        case 0x34: // Get address of InDOS flag
        {
            // Place InDOS flag at a fixed location in BIOS area (0x0050:0x0010 = phys 0x510)
            uint8_t *mem34 = i86_cpu_get_memory();
            mem34[0x510] = 0x00;  // Not in DOS
            i86_cpu_set_es(0x0050);
            i86_cpu_set_bx(0x0010);
            return true;
        }

        case 0x38: // Get/set country info
        {
            if (al == 0x00 || al == 0x01) {
                // Get country info - fill buffer at DS:DX
                uint8_t *mem38 = i86_cpu_get_memory();
                uint32_t buf38 = ((uint32_t)i86_cpu_get_ds() * 16 + i86_cpu_get_dx()) & 0xFFFFF;
                memset(mem38 + buf38, 0, 34);
                // Date format: 0=USA (m/d/y)
                mem38[buf38 + 0] = 0x00; mem38[buf38 + 1] = 0x00;
                // Currency symbol "$\0\0\0\0"
                mem38[buf38 + 2] = '$'; mem38[buf38 + 3] = 0;
                // Thousands separator ","
                mem38[buf38 + 7] = ','; mem38[buf38 + 8] = 0;
                // Decimal separator "."
                mem38[buf38 + 9] = '.'; mem38[buf38 + 10] = 0;
                // Date separator "/"
                mem38[buf38 + 11] = '/'; mem38[buf38 + 12] = 0;
                // Time separator ":"
                mem38[buf38 + 13] = ':'; mem38[buf38 + 14] = 0;
                // Currency format: 0 = $1.23
                mem38[buf38 + 15] = 0x00;
                // Digits after decimal in currency
                mem38[buf38 + 16] = 0x02;
                // Time format: 0 = 12-hour
                mem38[buf38 + 17] = 0x00;
                i86_cpu_set_bx(0x0001);  // Country code = 1 (US)
                set_carry(false);
            } else {
                set_carry(true);
            }
            return true;
        }

        case 0x39: // Create directory (MKDIR)
        {
            char dos_path39[DOS_PATH_MAX];
            read_dos_string(i86_cpu_get_ds(), i86_cpu_get_dx(), dos_path39, sizeof(dos_path39), '\0');
            char vfs_path39[DOS_PATH_MAX];
            dos_to_vfs_path(dos_path39, vfs_path39, sizeof(vfs_path39));

            if (mkdir(vfs_path39, 0755) == 0) {
                set_carry(false);
            } else {
                i86_cpu_set_ax(0x03);  // Path not found
                set_carry(true);
            }
            return true;
        }

        case 0x3A: // Remove directory (RMDIR)
        {
            char dos_path3a[DOS_PATH_MAX];
            read_dos_string(i86_cpu_get_ds(), i86_cpu_get_dx(), dos_path3a, sizeof(dos_path3a), '\0');
            char vfs_path3a[DOS_PATH_MAX];
            dos_to_vfs_path(dos_path3a, vfs_path3a, sizeof(vfs_path3a));

            if (rmdir(vfs_path3a) == 0) {
                set_carry(false);
            } else {
                i86_cpu_set_ax(0x03);
                set_carry(true);
            }
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
                    uint16_t key;
                    while (!bda_kbd_pop(&key)) {
                        vTaskDelay(pdMS_TO_TICKS(10));
                        poll_input();
                        if (s_i86.state != I86_STATE_RUNNING || s_i86.stop_requested) {
                            i86_cpu_set_ax(read_count);
                            set_carry(false);
                            return true;
                        }
                    }
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

        case 0x45: // Duplicate file handle
        {
            uint16_t src_handle = i86_cpu_get_bx();
            if (src_handle < DOS_FIRST_HANDLE) {
                // Standard handles - return next available handle pointing to same device
                int new_handle = dos_find_free_handle();
                if (new_handle < 0) {
                    i86_cpu_set_ax(0x04);
                    set_carry(true);
                } else {
                    s_i86.dos.files[new_handle].open = true;
                    s_i86.dos.files[new_handle].fp = NULL;  // Device handle
                    s_i86.dos.files[new_handle].access_mode = 2;
                    i86_cpu_set_ax(new_handle);
                    set_carry(false);
                }
            } else if (src_handle < DOS_MAX_FILES && s_i86.dos.files[src_handle].open) {
                int new_handle = dos_find_free_handle();
                if (new_handle < 0) {
                    i86_cpu_set_ax(0x04);
                    set_carry(true);
                } else {
                    // Duplicate: share the same FILE* (not a true dup, but sufficient)
                    s_i86.dos.files[new_handle] = s_i86.dos.files[src_handle];
                    i86_cpu_set_ax(new_handle);
                    set_carry(false);
                }
            } else {
                i86_cpu_set_ax(0x06);
                set_carry(true);
            }
            return true;
        }

        case 0x46: // Force duplicate file handle
        {
            uint16_t src46 = i86_cpu_get_bx();
            uint16_t dst46 = i86_cpu_get_cx();
            if (dst46 < DOS_MAX_FILES) {
                // Close destination if open
                if (s_i86.dos.files[dst46].open && s_i86.dos.files[dst46].fp) {
                    fclose(s_i86.dos.files[dst46].fp);
                }
                if (src46 < DOS_FIRST_HANDLE) {
                    s_i86.dos.files[dst46].open = true;
                    s_i86.dos.files[dst46].fp = NULL;
                    s_i86.dos.files[dst46].access_mode = 2;
                } else if (src46 < DOS_MAX_FILES && s_i86.dos.files[src46].open) {
                    s_i86.dos.files[dst46] = s_i86.dos.files[src46];
                }
                set_carry(false);
            } else {
                i86_cpu_set_ax(0x06);
                set_carry(true);
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

        case 0x52: // Get List of Lists (SYSVARS)
        {
            // Return pointer to minimal LoL structure at 0x0060:0x0000 (phys 0x600)
            // Set up in setup_bios()
            i86_cpu_set_es(0x0060);
            i86_cpu_set_bx(0x0000);
            return true;
        }

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

        case 0x5B: // Create new file (fail if exists)
        {
            char dos_path5b[DOS_PATH_MAX];
            read_dos_string(i86_cpu_get_ds(), i86_cpu_get_dx(), dos_path5b, sizeof(dos_path5b), '\0');
            char vfs_path5b[DOS_PATH_MAX];
            dos_to_vfs_path(dos_path5b, vfs_path5b, sizeof(vfs_path5b));

            // Check if file already exists
            struct stat st5b;
            if (stat(vfs_path5b, &st5b) == 0) {
                i86_cpu_set_ax(0x50);  // File exists
                set_carry(true);
                return true;
            }

            int handle5b = dos_find_free_handle();
            if (handle5b < 0) {
                i86_cpu_set_ax(0x04);
                set_carry(true);
                return true;
            }

            FILE *fp5b = fopen(vfs_path5b, "w+b");
            if (!fp5b) {
                i86_cpu_set_ax(0x03);
                set_carry(true);
                return true;
            }

            s_i86.dos.files[handle5b].fp = fp5b;
            s_i86.dos.files[handle5b].open = true;
            s_i86.dos.files[handle5b].access_mode = 2;
            strncpy(s_i86.dos.files[handle5b].path, vfs_path5b, DOS_PATH_MAX - 1);
            s_i86.stats.files_opened++;
            i86_cpu_set_ax(handle5b);
            set_carry(false);
            return true;
        }

        case 0x60: // Get canonical filename (truename)
        {
            char dos_path60[DOS_PATH_MAX];
            read_dos_string(i86_cpu_get_ds(), i86_cpu_get_si(), dos_path60, sizeof(dos_path60), '\0');

            // Build canonical path: prepend drive letter if missing
            char canon[DOS_PATH_MAX];
            if (dos_path60[1] == ':') {
                // Already has drive letter
                snprintf(canon, sizeof(canon), "%s", dos_path60);
            } else if (dos_path60[0] == '\\') {
                snprintf(canon, sizeof(canon), "C:%s", dos_path60);
            } else {
                snprintf(canon, sizeof(canon), "C:\\%s", dos_path60);
            }
            // Uppercase
            for (int i = 0; canon[i]; i++) {
                canon[i] = toupper((unsigned char)canon[i]);
            }
            write_dos_string(i86_cpu_get_es(), i86_cpu_get_di(), canon);
            set_carry(false);
            return true;
        }

        case 0x65: // Get extended country information
        {
            if (al == 0x01) {
                // Get general internationalization info
                uint8_t *mem65 = i86_cpu_get_memory();
                uint32_t buf65 = ((uint32_t)i86_cpu_get_es() * 16 + i86_cpu_get_di()) & 0xFFFFF;
                uint16_t size65 = i86_cpu_get_cx();
                if (size65 > 41) size65 = 41;
                memset(mem65 + buf65, 0, size65);
                mem65[buf65 + 0] = 0x01;  // Info ID
                if (size65 >= 3) { mem65[buf65 + 1] = 38; mem65[buf65 + 2] = 0; } // Size
                if (size65 >= 5) { mem65[buf65 + 3] = 0x01; mem65[buf65 + 4] = 0x00; } // Country=1 (US)
                if (size65 >= 7) { mem65[buf65 + 5] = 0xB5; mem65[buf65 + 6] = 0x01; } // Codepage=437
                if (size65 >= 9) { mem65[buf65 + 7] = 0x00; mem65[buf65 + 8] = 0x00; } // Date format=MDY
                if (size65 >= 14) { mem65[buf65 + 9] = '$'; } // Currency symbol
                if (size65 >= 16) { mem65[buf65 + 14] = ','; } // Thousands separator
                if (size65 >= 18) { mem65[buf65 + 16] = '.'; } // Decimal separator
                i86_cpu_set_cx(size65);
                set_carry(false);
            } else {
                set_carry(true);
            }
            return true;
        }

        case 0x66: // Get/set global code page
            if (al == 0x01) {
                // Get code page
                i86_cpu_set_bx(437);  // Active code page (US)
                i86_cpu_set_dx(437);  // System code page
            }
            set_carry(false);
            return true;

        case 0x67: // Set handle count
            // Accept and ignore - we always support DOS_MAX_FILES
            set_carry(false);
            return true;

        case 0x68: // Commit file (flush)
        {
            uint16_t handle68 = i86_cpu_get_bx();
            if (handle68 < DOS_MAX_FILES && s_i86.dos.files[handle68].open &&
                s_i86.dos.files[handle68].fp) {
                fflush(s_i86.dos.files[handle68].fp);
            }
            set_carry(false);
            return true;
        }

        default:
            ESP_LOGD(TAG, "INT 21h AH=%02Xh (unhandled)", ah);
            set_carry(true);
            return true;
    }
}

// ============================================================================
// Main Interrupt Dispatcher
// ============================================================================

// BIOS IVT signature: each handled interrupt gets a unique BIOS ROM address.
// Format: IVT entry points to F000:E0xx where xx = interrupt number.
// At that ROM address we place an IRET. The callback checks if the IVT
// still points to our BIOS address; if the guest OS has overridden it,
// we return false so the CPU dispatches through the IVT instead.
#define BIOS_IVT_SEG  0xF000
#define BIOS_IVT_BASE 0xE000  // F000:E0xx → physical 0xFE0xx

static bool ivt_is_bios_default(uint8_t num) {
    uint8_t *mem = i86_cpu_get_memory();
    uint16_t vec_off = mem[num * 4 + 0] | (mem[num * 4 + 1] << 8);
    uint16_t vec_seg = mem[num * 4 + 2] | (mem[num * 4 + 3] << 8);
    // Check if vector still points to our BIOS ROM area (F000:xxxx)
    return (vec_seg == BIOS_IVT_SEG);
}

static bool i86_interrupt_handler(void *ctx, uint8_t num) {
    s_i86.stats.interrupts_processed++;

    // For BIOS-level interrupts: only handle via callback if the IVT
    // still points to our BIOS ROM. Once the guest OS (FreeDOS) installs
    // its own handler, we defer to the IVT dispatch so the OS handler runs.
    switch (num) {
        // Hardware IRQ handlers - handle only if IVT still points to BIOS.
        // Once the guest OS hooks these, let CPU dispatch through IVT.
        // The guest OS chains back to F000:E0xx (real x86 code) for
        // the actual hardware handling (BDA update, EOI, etc).
        case 0x08: if (ivt_is_bios_default(num)) return handle_int08h(); return false;
        case 0x09: if (ivt_is_bios_default(num)) return handle_int09h(); return false;

        // BIOS service interrupts - handle only if IVT is default
        case 0x10: if (ivt_is_bios_default(num)) return handle_int10h(); return false;
        case 0x11: if (ivt_is_bios_default(num)) return handle_int11h(); return false;
        case 0x12: if (ivt_is_bios_default(num)) return handle_int12h(); return false;
        case 0x13: if (ivt_is_bios_default(num)) return handle_int13h(); return false;
        case 0x14: if (ivt_is_bios_default(num)) return handle_int14h(); return false;
        case 0x15: if (ivt_is_bios_default(num)) return handle_int15h(); return false;
        case 0x16: if (ivt_is_bios_default(num)) return handle_int16h(); return false;
        case 0x17: if (ivt_is_bios_default(num)) return handle_int17h(); return false;
        case 0x19: return handle_int19h();  // Bootstrap - always handle
        case 0x1A: if (ivt_is_bios_default(num)) return handle_int1ah(); return false;
        case 0x1C: return true;  // User timer tick - IRET (placeholder for guest hooks)

        // DOS service interrupts - handle only if IVT is default (before DOS loads)
        case 0x20: // DOS terminate
            if (ivt_is_bios_default(num)) { s_i86.state = I86_STATE_HALTED; return true; }
            return false;
        case 0x21:
            if (ivt_is_bios_default(num)) return handle_int21h();
            return false;
        case 0x28: // DOS idle
            if (ivt_is_bios_default(num)) return true;
            return false;
        case 0x2F: // Multiplex
            if (ivt_is_bios_default(num)) return true;
            return false;

        default:
            return false;  // Let CPU handle via IVT
    }
}

// ============================================================================
// BIOS / IVT Setup
// ============================================================================

static void setup_bios(void) {
    uint8_t *mem = i86_cpu_get_memory();

    // Clear IVT + BDA area
    memset(mem, 0, 0x500);

    // Clear BIOS ROM area (F0000-FFFFF)
    memset(mem + 0xF0000, 0, 0x10000);

    // =====================================================================
    // Set up IVT with distinct per-interrupt BIOS ROM addresses
    // Each vector points to F000:E0xx where xx = interrupt number
    // This allows ivt_is_bios_default() to work and provides distinct stubs
    // =====================================================================
    for (int i = 0; i < 256; i++) {
        uint16_t offset = BIOS_IVT_BASE + i;  // F000:E0xx
        mem[i * 4 + 0] = offset & 0xFF;
        mem[i * 4 + 1] = (offset >> 8) & 0xFF;
        mem[i * 4 + 2] = 0x00;  // Segment low (F000)
        mem[i * 4 + 3] = 0xF0;  // Segment high
        // Place IRET at each stub address (physical F0000 + E0xx)
        mem[0xF0000 + offset] = 0xCF;  // IRET
    }

    // Also place IRET at the old catch-all address F000:FF00
    mem[0xFFF00] = 0xCF;

    // =====================================================================
    // INT 08h - Timer tick handler (real x86 code, chainable)
    // Must be real code because FreeDOS hooks INT 08h and chains back.
    // When chaining via CALL FAR, the callback won't fire.
    // F000:E008 (physical 0xFE008), 25 bytes
    // =====================================================================
    {
        uint32_t a = 0xF0000 + BIOS_IVT_BASE + 0x08;
        int p = 0;
        // PUSH DS
        mem[a + p++] = 0x1E;
        // PUSH AX
        mem[a + p++] = 0x50;
        // MOV AX, 0040h
        mem[a + p++] = 0xB8; mem[a + p++] = 0x40; mem[a + p++] = 0x00;
        // MOV DS, AX
        mem[a + p++] = 0x8E; mem[a + p++] = 0xD8;
        // INC WORD PTR [006Ch] (BDA timer low word)
        mem[a + p++] = 0xFF; mem[a + p++] = 0x06;
        mem[a + p++] = 0x6C; mem[a + p++] = 0x00;
        // JNZ +4 (skip high word increment)
        mem[a + p++] = 0x75; mem[a + p++] = 0x04;
        // INC WORD PTR [006Eh] (BDA timer high word)
        mem[a + p++] = 0xFF; mem[a + p++] = 0x06;
        mem[a + p++] = 0x6E; mem[a + p++] = 0x00;
        // INT 1Ch (user timer tick hook)
        mem[a + p++] = 0xCD; mem[a + p++] = 0x1C;
        // MOV AL, 20h (EOI command)
        mem[a + p++] = 0xB0; mem[a + p++] = 0x20;
        // OUT 20h, AL (send EOI to PIC)
        mem[a + p++] = 0xE6; mem[a + p++] = 0x20;
        // POP AX
        mem[a + p++] = 0x58;
        // POP DS
        mem[a + p++] = 0x1F;
        // IRET
        mem[a + p++] = 0xCF;
        ESP_LOGD(TAG, "INT 08h handler at %05Xh, %d bytes", a, p);
    }

    // =====================================================================
    // INT 1Eh - Diskette Parameter Table (data, not code)
    // Place at F000:EFC0 (physical 0xFEFC0), 11 bytes
    // =====================================================================
    {
        uint32_t dpt_addr = 0xFEFC0;
        mem[dpt_addr + 0] = 0xDF;  // SRT=D, HUT=F (step rate, head unload)
        mem[dpt_addr + 1] = 0x02;  // HLT=01, DMA=yes
        mem[dpt_addr + 2] = 0x25;  // Motor off delay (ticks)
        mem[dpt_addr + 3] = 0x02;  // Bytes per sector: 512 (N=2)
        mem[dpt_addr + 4] = 18;    // Sectors per track (1.44M)
        mem[dpt_addr + 5] = 0x1B;  // Gap length
        mem[dpt_addr + 6] = 0xFF;  // Data length
        mem[dpt_addr + 7] = 0x54;  // Format gap length
        mem[dpt_addr + 8] = 0xF6;  // Format fill byte
        mem[dpt_addr + 9] = 0x0F;  // Head settle time (ms)
        mem[dpt_addr + 10] = 0x08; // Motor start time (1/8 sec units)
        // Point INT 1Eh to this table
        mem[0x1E * 4 + 0] = 0xC0;  // Offset = EFC0
        mem[0x1E * 4 + 1] = 0xEF;
        mem[0x1E * 4 + 2] = 0x00;  // Segment = F000
        mem[0x1E * 4 + 3] = 0xF0;
    }

    // =====================================================================
    // BIOS Data Area (BDA) at 0x0400-0x04FF
    // =====================================================================

    // COM port base addresses
    bda_write16(BDA_COM1, 0x03F8);
    bda_write16(BDA_COM2, 0x02F8);

    // LPT port base addresses
    bda_write16(BDA_LPT1, 0x0378);

    // Equipment word: 1 floppy, 80x25 CGA color, no FPU
    // Bits: 0=has floppy, 1=FPU, 4-5=video (10=80x25 color), 6-7=floppies-1
    bda_write16(BDA_EQUIPMENT, 0x0021);

    // Memory size in KB
    bda_write16(BDA_MEMORY_SIZE, 640);

    // Keyboard buffer: head and tail are ABSOLUTE offsets from seg 0040
    // Buffer is at 0040:001E to 0040:003D (0x41E to 0x43D)
    bda_write16(BDA_KBD_HEAD, 0x001E);
    bda_write16(BDA_KBD_TAIL, 0x001E);

    // Keyboard flags
    bda_write8(BDA_KBD_FLAGS1, 0x00);
    bda_write8(BDA_KBD_FLAGS2, 0x00);

    // Video: mode 03h (80x25 color text)
    bda_write8(BDA_VIDEO_MODE, 0x03);
    bda_write16(BDA_VIDEO_COLS, 80);
    bda_write16(BDA_VIDEO_PAGE_SIZE, 4096);
    bda_write16(BDA_VIDEO_PAGE_OFF, 0);
    bda_write16(BDA_CURSOR_TYPE, 0x0607);
    bda_write8(BDA_ACTIVE_PAGE, 0);
    bda_write16(BDA_CRTC_PORT, 0x03D4);

    // Number of video rows - 1 (at 0x0484, used by DOS)
    mem[0x0484] = 24;

    // Timer ticks (start from 0)
    bda_write16(BDA_TIMER_COUNT, 0);
    bda_write16(BDA_TIMER_COUNT + 2, 0);
    bda_write8(BDA_TIMER_OFLOW, 0);

    // Hard disk count
    int hd_count = 0;
    if (disk_is_mounted(2)) hd_count++;
    if (disk_is_mounted(3)) hd_count++;
    bda_write8(BDA_DISK_COUNT, hd_count);

    // Print timeout values
    bda_write8(BDA_PRINT_TIMEOUT, 20);
    bda_write8(BDA_PRINT_TIMEOUT + 1, 20);

    // =====================================================================
    // BIOS ROM Entry Points
    // =====================================================================

    // Power-on entry: FFFF:0000 → JMP FAR F000:E05B
    mem[0xFFFF0] = 0xEA;
    mem[0xFFFF1] = 0x5B;
    mem[0xFFFF2] = 0xE0;
    mem[0xFFFF3] = 0x00;
    mem[0xFFFF4] = 0xF0;

    // Boot code at F000:E05B: INT 19h then halt
    mem[0xFE05B] = 0xCD;  // INT 19h
    mem[0xFE05C] = 0x19;
    mem[0xFE05D] = 0xF4;  // HLT
    mem[0xFE05E] = 0xEB;  // JMP $-2
    mem[0xFE05F] = 0xFC;

    // BIOS date string at FFFF:0005
    const char *bios_date = "02/13/26";
    memcpy(mem + 0xFFFF5, bios_date, 8);

    // Machine ID at FFFF:000E
    mem[0xFFFFE] = 0xFC;  // AT class

    // =====================================================================
    // Hard Disk Parameter Tables
    // =====================================================================
    if (disk_is_mounted(2)) {
        disk_info_t info;
        if (disk_get_info(2, &info) == ESP_OK) {
            uint32_t table_addr = 0xFE401;
            mem[table_addr + 0] = info.geometry.cylinders & 0xFF;
            mem[table_addr + 1] = (info.geometry.cylinders >> 8) & 0xFF;
            mem[table_addr + 2] = info.geometry.heads;
            mem[table_addr + 5] = 0xFF;
            mem[table_addr + 6] = 0xFF;
            mem[table_addr + 8] = 0x00;
            mem[table_addr + 14] = info.geometry.sectors_per_track;

            // INT 41h → HD0 parameter table
            mem[0x41 * 4 + 0] = 0x01;
            mem[0x41 * 4 + 1] = 0xE4;
            mem[0x41 * 4 + 2] = 0x00;
            mem[0x41 * 4 + 3] = 0xF0;
        }
    }

    // =====================================================================
    // Program PIT channel 0 for ~18.2 Hz (reload = 0 = 65536)
    // This mimics what the BIOS does during POST
    // =====================================================================
    pc_hw_write_port(&s_i86.hw, 0x43, 0x36);  // Ch0, LSB/MSB, mode 3 (square wave)
    pc_hw_write_port(&s_i86.hw, 0x40, 0x00);  // Reload low byte (0 = 65536)
    pc_hw_write_port(&s_i86.hw, 0x40, 0x00);  // Reload high byte

    // Unmask timer and keyboard IRQs on PIC
    pc_hw_write_port(&s_i86.hw, 0x21, 0xFC);  // Mask = 1111_1100 → unmask IRQ0+IRQ1

    // =====================================================================
    // Clear video RAM (B8000-B8FFF = 80x25 text mode)
    // =====================================================================
    uint8_t *video = mem + 0xB8000;
    for (int i = 0; i < 4000; i += 2) {
        video[i] = 0x20;
        video[i + 1] = 0x07;
    }

    // =====================================================================
    // DOS List of Lists (LoL) structure at 0060:0000 (physical 0x600)
    // INT 21h/52h returns ES:BX pointing here. Minimal structure to
    // prevent crashes when FreeDOS or programs inspect it.
    // =====================================================================
    {
        uint32_t lol = 0x600;
        memset(mem + lol - 2, 0, 0x30);
        // Offset -2: First MCB segment (0x0080 = just above PSP area)
        mem[lol - 2] = 0x80; mem[lol - 1] = 0x00;
        // Offset +0: First DPB pointer (FFFF:FFFF = none)
        mem[lol + 0] = 0xFF; mem[lol + 1] = 0xFF;
        mem[lol + 2] = 0xFF; mem[lol + 3] = 0xFF;
        // Offset +4: First SFT pointer (FFFF:FFFF = none)
        mem[lol + 4] = 0xFF; mem[lol + 5] = 0xFF;
        mem[lol + 6] = 0xFF; mem[lol + 7] = 0xFF;
        // Offset +8: CLOCK$ device (FFFF:FFFF = none)
        mem[lol + 8] = 0xFF; mem[lol + 9] = 0xFF;
        mem[lol + 10] = 0xFF; mem[lol + 11] = 0xFF;
        // Offset +12: CON device (FFFF:FFFF = none)
        mem[lol + 12] = 0xFF; mem[lol + 13] = 0xFF;
        mem[lol + 14] = 0xFF; mem[lol + 15] = 0xFF;
        // Offset +16: Max bytes per sector
        mem[lol + 16] = 0x00; mem[lol + 17] = 0x02;  // 512
        // Offset +18: First disk buffer pointer (FFFF:FFFF)
        mem[lol + 18] = 0xFF; mem[lol + 19] = 0xFF;
        mem[lol + 20] = 0xFF; mem[lol + 21] = 0xFF;
        // Offset +22h (34): NUL device header at the LoL + 0x22
        // NUL device: next device = FFFF:FFFF, attributes = 0x8004 (char device, NUL)
        uint32_t nul = lol + 0x22;
        mem[nul + 0] = 0xFF; mem[nul + 1] = 0xFF;  // Next: FFFF:FFFF
        mem[nul + 2] = 0xFF; mem[nul + 3] = 0xFF;
        mem[nul + 4] = 0x04; mem[nul + 5] = 0x80;  // Attr: char device, NUL
        mem[nul + 6] = 0x00; mem[nul + 7] = 0x00;  // Strategy entry (stub)
        mem[nul + 8] = 0x00; mem[nul + 9] = 0x00;  // Interrupt entry (stub)
        // Device name "NUL     "
        memcpy(mem + nul + 10, "NUL     ", 8);
    }

    ESP_LOGI(TAG, "BIOS initialized: IVT (distinct entries), BDA, BIOS ROM, PIT, video RAM, LoL");
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

        // Also add directly to BDA keyboard buffer
        bda_kbd_push((scancode << 8) | ascii);
    }

    // Check USB keyboard
    if (usb_keyboard_is_initialized()) {
        while (usb_keyboard_available()) {
            uint8_t ascii = usb_keyboard_read();
            uint8_t scancode = (ascii < 128) ? ascii_to_scancode[ascii] : 0;
            if (scancode) {
                kbd_push_scancode(&s_i86.hw.keyboard, scancode);
                bda_kbd_push((scancode << 8) | ascii);
            }
        }
    }

    // Check UART (truly non-blocking via UART driver)
    {
        size_t buffered = 0;
        uart_get_buffered_data_len(CONFIG_ESP_CONSOLE_UART_NUM, &buffered);
        if (buffered > 0) {
            uint8_t byte;
            if (uart_read_bytes(CONFIG_ESP_CONSOLE_UART_NUM, &byte, 1, 0) > 0) {
                uint8_t ascii = byte;
                uint8_t scancode = (ascii < 128) ? ascii_to_scancode[ascii] : 0;

                kbd_push_scancode(&s_i86.hw.keyboard, scancode);
                bda_kbd_push((scancode << 8) | ascii);
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

    // video memory starts at offset 0x18000 from video base (B8000h - A0000h)
    uint8_t *video = i86_cpu_get_video();
    if (!video) return;

    // Sync CGA cursor from BIOS state (for software that uses INT 10h)
    // Programs that write CRTC ports directly already update hw.cga.cursor_pos
    uint16_t bda_cpos = bda_read16(BDA_CURSOR_POS);
    uint8_t bda_cy = (bda_cpos >> 8) & 0xFF;
    uint8_t bda_cx = bda_cpos & 0xFF;
    uint16_t hw_cpos = bda_cy * 80 + bda_cx;
    // Use BDA position if it differs from hardware (BIOS wrote it)
    if (hw_cpos != s_i86.hw.cga.cursor_pos && bda_cy < 25 && bda_cx < 80) {
        s_i86.hw.cga.cursor_pos = hw_cpos;
    }

    s_i86.hw.cga.needs_refresh = true;
    // Use mode-aware renderer: auto-detects text vs graphics from CGA mode register
    pc_hw_render_cga(&s_i86.hw, video + 0x18000);
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

    // Video defaults
    s_i86.video_mode = 0x03;
    s_i86.cursor_x = 0;
    s_i86.cursor_y = 0;
    s_i86.text_attr = 0x07;
    s_i86.active_page = 0;

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

    s_i86.video_mode = 0x03;
    s_i86.cursor_x = 0;
    s_i86.cursor_y = 0;
    s_i86.text_attr = 0x07;
    s_i86.stop_requested = false;

    memset(&s_i86.stats, 0, sizeof(s_i86.stats));
    s_i86.state = I86_STATE_HALTED;
}

uint32_t i86_run(uint32_t count) {
    if (!s_i86.initialized) return 0;

    s_i86.state = I86_STATE_RUNNING;
    s_i86.stop_requested = false;

    // Ensure video card is in GFX mode for CGA rendering
    if (video_card_is_initialized() && count == 0) {
        video_card_enter_gfx_mode();
    }

    uint32_t total_executed = 0;
    uint32_t batch = 128;  // Execute instructions in batches for efficiency
    bool continuous = (count == 0);

    while (continuous || total_executed < count) {
        if (s_i86.stop_requested) break;
        if (s_i86.state != I86_STATE_RUNNING) break;

        // Periodic trace (every 500K instructions)
        if (total_executed > 0 && total_executed % 500000 == 0) {
            uint16_t cs = i86_cpu_get_cs();
            uint16_t ip = i86_cpu_get_ip();
            ESP_LOGI(TAG, "[%luK] %04X:%04X",
                     (unsigned long)(total_executed / 1000), cs, ip);
        }

        // Execute a batch of instructions
        uint32_t to_run = batch;
        if (!continuous && (count - total_executed) < batch) {
            to_run = count - total_executed;
        }

        uint32_t executed = i86_cpu_run(to_run);
        total_executed += executed;
        s_i86.stats.instructions_executed += executed;

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

    // Switch video card to GFX mode for direct CGA rendering to LCD
    if (video_card_is_initialized()) {
        video_card_enter_gfx_mode();
        video_card_present();  // Show initial black frame
    }

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
             (uint32_t)(bda_read16(BDA_TIMER_COUNT) | ((uint32_t)bda_read16(BDA_TIMER_COUNT + 2) << 16)));
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

    // Also add to BDA keyboard buffer directly
    bda_kbd_push((scancode << 8) | ascii);
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
