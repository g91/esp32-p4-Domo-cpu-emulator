/*
 * z80_emulator.c - CP/M 2.2 machine emulator on Z80 CPU
 *
 * Implements:
 *  - 64KB Z80 address space (PSRAM-backed)
 *  - CP/M 2.2 BDOS calls via trap at 0x0005
 *  - BIOS warm-boot via trap at 0x0000
 *  - Virtual disk images on SD card: /sdcard/cpm/[drive].img
 *  - Console I/O to UART (and LCD if initialized)
 *  - Keyboard input from PS/2, USB keyboard, or UART
 *  - FreeRTOS task with configurable CPU frequency throttle
 */

#include "z80_emulator.h"
#include "z80_cpu.h"
#include "lcd_console.h"
#include "ps2_keyboard.h"
#include "usb_keyboard.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <ctype.h>
#include <sys/stat.h>
#include <dirent.h>

static const char *TAG = "CPM";

/* =====================================================================
 * Internal state
 * ===================================================================== */

/* FCB - File Control Block (CP/M 2.2 structure) */
typedef struct {
    uint8_t  drive;        /* 0 = current, 1..16 = A..P */
    char     name[8];      /* filename, space padded */
    char     ext[3];       /* extension, space padded */
    uint8_t  ex;           /* extent number (low) */
    uint8_t  s1;           /* reserved */
    uint8_t  s2;           /* extent number (high) */
    uint8_t  rc;           /* record count in current extent */
    uint8_t  dm[16];       /* disk map (allocation blocks) */
    uint8_t  cr;           /* current record */
    uint8_t  r0, r1, r2;  /* random record number */
} __attribute__((packed)) fcb_t;

/* CP/M file handle (we emulate a simple flat-file system) */
typedef struct {
    bool   in_use;
    char   host_path[256]; /* actual file path on SD card */
    FILE  *fp;
    bool   read_only;
} cpm_file_t;

#define CPM_MAX_FILES   16
#define CPM_MAX_OPEN    8  /* simultaneously open FCBs */

typedef struct {
    /* Z80 CPU */
    z80_t       cpu;

    /* 64KB RAM */
    uint8_t    *ram;           /* heap_caps_malloc(65536, SPIRAM) */

    /* Disk images */
    FILE       *disk[CPM_MAX_DRIVES];
    char        disk_path[CPM_MAX_DRIVES][256];
    bool        disk_ro[CPM_MAX_DRIVES];
    uint8_t     current_drive; /* 0=A, 1=B, ... */

    /* DMA address */
    uint16_t    dma_addr;      /* default 0x0080 */

    /* I/O byte */
    uint8_t     iobyte;

    /* User number (0-15) */
    uint8_t     user_num;

    /* Console output buffer */
    char        con_buf[256];
    int         con_buf_len;

    /* Input queue (from UART / PS2 / USB) */
    char        kbd_queue[256];
    int         kbd_head, kbd_tail;

    /* Config */
    z80_emu_config_t cfg;

    /* State */
    volatile bool running;
    volatile bool stop_requested;

    /* Host directory listing for search first/next */
    DIR        *search_dir;
    char        search_dir_path[256];
    char        search_pattern[16];  /* up to 8.3 */
    uint8_t     search_drive;

    /* Simple file handle table (indexed by FCB address in Z80 RAM) */
    struct {
        bool    valid;
        char    path[256];
        FILE   *fp;
        bool    read_only;
        uint16_t fcb_addr; /* Z80 address of associated FCB */
    } file_table[CPM_MAX_OPEN];

} cpm_state_t;

static cpm_state_t s_cpm;

/* =====================================================================
 * Console helpers
 * ===================================================================== */

/* Output a single character to UART and optionally LCD */
static void cpm_putchar(char ch) {
    /* Convert CR+LF for LCD */
    putchar(ch);
    fflush(stdout);
    if (s_cpm.cfg.enable_lcd && lcd_console_is_initialized()) {
        char buf[2] = { ch, 0 };
        lcd_console_print(buf);
    }
}

static void cpm_print(const char *s) {
    while (*s) cpm_putchar(*s++);
}

static void cpm_printf(const char *fmt, ...) {
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    cpm_print(buf);
}

/* Read a character from keyboard queue */
static bool cpm_kbd_avail(void) {
    return s_cpm.kbd_head != s_cpm.kbd_tail;
}

static uint8_t cpm_kbd_read(void) {
    if (!cpm_kbd_avail()) return 0;
    uint8_t ch = (uint8_t)s_cpm.kbd_queue[s_cpm.kbd_head];
    s_cpm.kbd_head = (s_cpm.kbd_head + 1) & 0xFF;
    return ch;
}

static void cpm_kbd_push(uint8_t ch) {
    int next = (s_cpm.kbd_tail + 1) & 0xFF;
    if (next != s_cpm.kbd_head) {
        s_cpm.kbd_queue[s_cpm.kbd_tail] = (char)ch;
        s_cpm.kbd_tail = next;
    }
}

/* Poll keyboard sources */
static void cpm_poll_kbd(void) {
    /* PS/2 keyboard */
    if (ps2_keyboard_is_initialized()) {
        while (ps2_keyboard_available()) {
            uint8_t k = ps2_keyboard_read();
            /* Convert PS/2 special keys to ASCII */
            if (k >= 0x20 && k < 0x7F) cpm_kbd_push(k);
            else if (k == 0x0D || k == 0x0A) cpm_kbd_push('\r');
            else if (k == 0x08 || k == 0x7F) cpm_kbd_push('\b');
            else if (k == 0x1B) cpm_kbd_push(0x1B);
            else if (k >= 0x01 && k < 0x20) cpm_kbd_push(k);
        }
    }
    /* USB keyboard */
    if (usb_keyboard_is_initialized()) {
        while (usb_keyboard_available()) {
            uint8_t k = usb_keyboard_read();
            if (k) cpm_kbd_push(k);
        }
    }
    /* UART */
    uint8_t uart_ch;
    int len = uart_read_bytes(UART_NUM_0, &uart_ch, 1, 0);
    if (len > 0) {
        if (uart_ch == 0x03) { /* Ctrl+C: stop */
            s_cpm.stop_requested = true;
        } else {
            cpm_kbd_push(uart_ch);
        }
    }
}

/* =====================================================================
 * Memory callbacks
 * ===================================================================== */
static uint8_t z80_mem_read(void *user, uint16_t addr) {
    (void)user;
    return s_cpm.ram[addr];
}

static void z80_mem_write(void *user, uint16_t addr, uint8_t val) {
    (void)user;
    s_cpm.ram[addr] = val;
}

/* No I/O ports used in CP/M (all goes through BDOS) */
static uint8_t z80_io_read(void *user, uint16_t port) {
    (void)user; (void)port;
    return 0xFF;
}
static void z80_io_write(void *user, uint16_t port, uint8_t val) {
    (void)user; (void)port; (void)val;
}

/* =====================================================================
 * FCB helpers
 * ===================================================================== */

/* Read FCB from Z80 memory at given address */
static void read_fcb(uint16_t addr, fcb_t *fcb) {
    for (int i = 0; i < (int)sizeof(fcb_t); i++) {
        ((uint8_t*)fcb)[i] = s_cpm.ram[(uint16_t)(addr + i)];
    }
}

/* Write FCB back to Z80 memory */
static void write_fcb(uint16_t addr, const fcb_t *fcb) {
    for (int i = 0; i < (int)sizeof(fcb_t); i++) {
        s_cpm.ram[(uint16_t)(addr + i)] = ((const uint8_t*)fcb)[i];
    }
}

/* Convert FCB 8.3 name to "FILENAME.EXT" string (null terminated) */
static void fcb_to_name(const fcb_t *fcb, char *out, int out_sz) {
    char name[9], ext[4];
    memcpy(name, fcb->name, 8); name[8] = 0;
    memcpy(ext,  fcb->ext,  3); ext[3]  = 0;

    /* Strip attribute bits (high bit) and trailing spaces */
    for (int i = 0; i < 8; i++) name[i] &= 0x7F;
    for (int i = 0; i < 3; i++) ext[i]  &= 0x7F;

    /* rtrim */
    for (int i = 7; i >= 0 && name[i] == ' '; i--) name[i] = 0;
    for (int i = 2; i >= 0 && ext[i]  == ' '; i--) ext[i]  = 0;

    if (ext[0])
        snprintf(out, out_sz, "%s.%s", name, ext);
    else
        snprintf(out, out_sz, "%s", name);
}

/* Build host path for a file on the given drive */
static void build_host_path(uint8_t drive, const char *filename, char *out, int out_sz) {
    const char *dir = s_cpm.cfg.disk_dir ? s_cpm.cfg.disk_dir : "/sdcard/cpm";
    char drive_letter = 'a' + drive;
    /* Subdirectory per drive: /sdcard/cpm/a/, /sdcard/cpm/b/, etc. */
    snprintf(out, out_sz, "%s/%c/%s", dir, drive_letter, filename);
}

/* Resolve FCB to host path */
static void fcb_host_path(const fcb_t *fcb, char *out, int out_sz) {
    char name[16];
    fcb_to_name(fcb, name, sizeof(name));
    uint8_t drive = (fcb->drive == 0) ? s_cpm.current_drive : (uint8_t)(fcb->drive - 1);
    build_host_path(drive, name, out, out_sz);
}

/* Find or allocate a file table slot for given host path */
static int file_table_find(const char *path) {
    for (int i = 0; i < CPM_MAX_OPEN; i++) {
        if (s_cpm.file_table[i].valid &&
            strcasecmp(s_cpm.file_table[i].path, path) == 0) {
            return i;
        }
    }
    return -1;
}

static int file_table_alloc(void) {
    for (int i = 0; i < CPM_MAX_OPEN; i++) {
        if (!s_cpm.file_table[i].valid) return i;
    }
    return -1;
}

/* Get current record number from FCB (sequential) */
static uint32_t fcb_get_seq_record(const fcb_t *fcb) {
    return (uint32_t)(((fcb->s2 & 0x3F) << 10) | ((fcb->ex & 0x1F) << 5) | (fcb->cr & 0x1F));
}

/* Get random record number from FCB */
static uint32_t fcb_get_rnd_record(const fcb_t *fcb) {
    return (uint32_t)(fcb->r0 | ((uint32_t)fcb->r1 << 8) | ((uint32_t)fcb->r2 << 16));
}

/* Set random record number in FCB */
static void fcb_set_rnd_record(fcb_t *fcb, uint32_t r) {
    fcb->r0 = (uint8_t)(r);
    fcb->r1 = (uint8_t)(r >> 8);
    fcb->r2 = (uint8_t)(r >> 16);
}

/* =====================================================================
 * CP/M BDOS handler
 * Called when Z80 executes CALL 0x0005 (trap callback)
 * ===================================================================== */
static bool cpm_bdos_call(void *user, z80_t *cpu) {
    (void)user;
    uint8_t  func = cpu->C;           /* BDOS function in C register */
    uint16_t de   = z80_get_de(cpu);  /* DE: parameter */
    uint8_t  e    = cpu->E;           /* E:  single-byte parameter */

    /* Return value in A (HL for 16-bit returns) */
    uint8_t  ret8  = 0;
    uint16_t ret16 = 0;

    switch (func) {

    /* ---- 0: System Reset (warm boot) ---- */
    case BDOS_SYSRESET:
        ESP_LOGI(TAG, "CP/M warm boot");
        s_cpm.stop_requested = true;
        break;

    /* ---- 1: Console Input ---- */
    case BDOS_CONIN:
        cpm_poll_kbd();
        while (!cpm_kbd_avail()) {
            vTaskDelay(pdMS_TO_TICKS(10));
            cpm_poll_kbd();
            if (s_cpm.stop_requested) return true;
        }
        ret8 = cpm_kbd_read();
        cpm_putchar((char)ret8); /* echo */
        break;

    /* ---- 2: Console Output ---- */
    case BDOS_CONOUT:
        cpm_putchar((char)e);
        break;

    /* ---- 3: Reader Input (same as console) ---- */
    case BDOS_READER:
        cpm_poll_kbd();
        while (!cpm_kbd_avail()) {
            vTaskDelay(pdMS_TO_TICKS(10));
            cpm_poll_kbd();
        }
        ret8 = cpm_kbd_read();
        break;

    /* ---- 4: Punch Output (same as console) ---- */
    case BDOS_PUNCH:
        cpm_putchar((char)e);
        break;

    /* ---- 5: List Output ---- */
    case BDOS_LIST:
        cpm_putchar((char)e);
        break;

    /* ---- 6: Direct Console I/O ---- */
    case BDOS_DIRCONIO:
        if (e == 0xFF) {
            /* Input: return char or 0 if none */
            cpm_poll_kbd();
            ret8 = cpm_kbd_avail() ? cpm_kbd_read() : 0;
        } else if (e == 0xFE) {
            /* Console status */
            cpm_poll_kbd();
            ret8 = cpm_kbd_avail() ? 0xFF : 0x00;
        } else {
            cpm_putchar((char)e);
        }
        break;

    /* ---- 7: Get I/O Byte ---- */
    case BDOS_GETIOBYTE:
        ret8 = s_cpm.iobyte;
        break;

    /* ---- 8: Set I/O Byte ---- */
    case BDOS_SETIOBYTE:
        s_cpm.iobyte = e;
        break;

    /* ---- 9: Print String (DE = address, $ terminated) ---- */
    case BDOS_PRINTSTR: {
        uint16_t addr = de;
        for (int i = 0; i < 4096; i++) {
            char ch = (char)s_cpm.ram[addr++];
            if (ch == '$') break;
            cpm_putchar(ch);
        }
        break;
    }

    /* ---- 10: Read Console Buffer ---- */
    case BDOS_READBUF: {
        uint8_t maxlen = s_cpm.ram[de];
        uint8_t count  = 0;
        char    line[256];

        /* Simple line editor: echo input, support backspace */
        while (count < maxlen) {
            cpm_poll_kbd();
            if (!cpm_kbd_avail()) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }
            if (s_cpm.stop_requested) break;
            char ch = (char)cpm_kbd_read();
            if (ch == '\r' || ch == '\n') {
                cpm_putchar('\r'); cpm_putchar('\n');
                break;
            } else if ((ch == '\b' || ch == 0x7F) && count > 0) {
                count--;
                cpm_putchar('\b'); cpm_putchar(' '); cpm_putchar('\b');
            } else if (ch >= 0x20 && ch < 0x7F) {
                line[count++] = ch;
                cpm_putchar(ch);
            }
        }
        line[count] = 0;
        s_cpm.ram[(uint16_t)(de + 1)] = count;
        for (uint8_t i = 0; i < count; i++) {
            s_cpm.ram[(uint16_t)(de + 2 + i)] = (uint8_t)line[i];
        }
        break;
    }

    /* ---- 11: Get Console Status ---- */
    case BDOS_CONSTAT:
        cpm_poll_kbd();
        ret8 = cpm_kbd_avail() ? 0xFF : 0x00;
        break;

    /* ---- 12: Return Version Number ---- */
    case BDOS_GETVER:
        ret16 = 0x0022;  /* CP/M 2.2 */
        ret8  = 0x22;
        break;

    /* ---- 13: Reset Disk ---- */
    case BDOS_RESETDSK:
        s_cpm.current_drive = 0;
        s_cpm.dma_addr = CPM_ADDR_DMA;
        break;

    /* ---- 14: Select Disk ---- */
    case BDOS_SELDSK:
        if (e < CPM_MAX_DRIVES) {
            s_cpm.current_drive = e;
            ret8 = 0x00; /* success */
        } else {
            ret8 = 0xFF; /* error */
        }
        break;

    /* ---- 15: Open File ---- */
    case BDOS_OPEN: {
        fcb_t fcb;
        read_fcb(de, &fcb);
        char path[256];
        fcb_host_path(&fcb, path, sizeof(path));

        int slot = file_table_find(path);
        if (slot < 0) {
            slot = file_table_alloc();
            if (slot < 0) { ret8 = 0xFF; break; }
            /* Try read-write, then read-only */
            FILE *fp = fopen(path, "r+b");
            bool ro = false;
            if (!fp) { fp = fopen(path, "rb"); ro = true; }
            if (!fp) { ret8 = 0xFF; break; }
            s_cpm.file_table[slot].valid    = true;
            s_cpm.file_table[slot].fp       = fp;
            s_cpm.file_table[slot].read_only = ro;
            strncpy(s_cpm.file_table[slot].path, path, sizeof(s_cpm.file_table[slot].path)-1);
            s_cpm.file_table[slot].fcb_addr = de;
        }

        /* Compute file size in records */
        struct stat st;
        uint32_t total_recs = 0;
        if (stat(path, &st) == 0) {
            total_recs = (uint32_t)((st.st_size + CPM_SECTOR_SZ - 1) / CPM_SECTOR_SZ);
        }

        fcb.cr   = 0;
        fcb.ex   = 0;
        fcb.s1   = 0;
        fcb.s2   = 0;
        fcb.rc   = (uint8_t)(total_recs > 127 ? 127 : total_recs);
        write_fcb(de, &fcb);
        ret8 = 0x00; /* success (directory index 0) */
        break;
    }

    /* ---- 16: Close File ---- */
    case BDOS_CLOSE: {
        fcb_t fcb;
        read_fcb(de, &fcb);
        char path[256];
        fcb_host_path(&fcb, path, sizeof(path));
        int slot = file_table_find(path);
        if (slot >= 0) {
            fclose(s_cpm.file_table[slot].fp);
            s_cpm.file_table[slot].valid = false;
        }
        ret8 = 0x00;
        break;
    }

    /* ---- 17: Search First ---- */
    case BDOS_SFIRST: {
        fcb_t fcb;
        read_fcb(de, &fcb);

        /* Close any previous search */
        if (s_cpm.search_dir) { closedir(s_cpm.search_dir); s_cpm.search_dir = NULL; }

        uint8_t drive = (fcb.drive == 0) ? s_cpm.current_drive : (uint8_t)(fcb.drive - 1);
        const char *dir = s_cpm.cfg.disk_dir ? s_cpm.cfg.disk_dir : "/sdcard/cpm";
        snprintf(s_cpm.search_dir_path, sizeof(s_cpm.search_dir_path), "%s/%c", dir, 'a' + drive);
        s_cpm.search_drive = drive;

        /* Build search pattern (with wildcards) */
        fcb_to_name(&fcb, s_cpm.search_pattern, sizeof(s_cpm.search_pattern));

        s_cpm.search_dir = opendir(s_cpm.search_dir_path);
        if (!s_cpm.search_dir) { ret8 = 0xFF; break; }
        /* Fall through to search-next logic below */
        /* (intentional: let SNEXT handle the first result) */
    }
    /* FALLTHROUGH */

    /* ---- 18: Search Next ---- */
    case BDOS_SNEXT: {
        if (!s_cpm.search_dir) { ret8 = 0xFF; break; }
        struct dirent *de_entry;
        ret8 = 0xFF; /* not found yet */
        while ((de_entry = readdir(s_cpm.search_dir)) != NULL) {
            if (de_entry->d_name[0] == '.') continue;
            /* Match against pattern (simple wildcard: '?' = any char, '*' in ext = match all) */
            /* For now accept any file (CP/M programs usually issue *.* ) */
            /* Build directory entry in DMA buffer */
            const char *fname = de_entry->d_name;
            /* Extract name/ext */
            char nm[9] = "        ";
            char ex[4] = "   ";
            const char *dot = strchr(fname, '.');
            int nm_len = dot ? (int)(dot - fname) : (int)strlen(fname);
            if (nm_len > 8) nm_len = 8;
            for (int i = 0; i < nm_len; i++) nm[i] = toupper((unsigned char)fname[i]);
            if (dot) {
                int ex_len = (int)strlen(dot + 1);
                if (ex_len > 3) ex_len = 3;
                for (int i = 0; i < ex_len; i++) ex[i] = toupper((unsigned char)dot[1+i]);
            }
            /* Fill DMA buffer with directory entry (32 bytes) */
            uint8_t *dma = &s_cpm.ram[s_cpm.dma_addr];
            memset(dma, 0, 32);
            dma[0] = s_cpm.search_drive; /* user/drive */
            for (int i = 0; i < 8; i++) dma[1+i] = (uint8_t)nm[i];
            for (int i = 0; i < 3; i++) dma[9+i] = (uint8_t)ex[i];
            /* Get size */
            char full_path[512];
            snprintf(full_path, sizeof(full_path), "%s/%s", s_cpm.search_dir_path, fname);
            struct stat st;
            if (stat(full_path, &st) == 0) {
                uint32_t recs = (uint32_t)((st.st_size + 127) / 128);
                dma[15] = (uint8_t)(recs & 0x7F); /* RC: record count in extent */
            }
            ret8 = 0x00; /* found at position 0 in DMA */
            break;
        }
        if (ret8 == 0xFF && s_cpm.search_dir) {
            closedir(s_cpm.search_dir);
            s_cpm.search_dir = NULL;
        }
        break;
    }

    /* ---- 19: Erase File ---- */
    case BDOS_ERASE: {
        fcb_t fcb;
        read_fcb(de, &fcb);
        char path[256];
        fcb_host_path(&fcb, path, sizeof(path));
        /* Close if open */
        int slot = file_table_find(path);
        if (slot >= 0) {
            fclose(s_cpm.file_table[slot].fp);
            s_cpm.file_table[slot].valid = false;
        }
        ret8 = (remove(path) == 0) ? 0x00 : 0xFF;
        break;
    }

    /* ---- 20: Read Sequential ---- */
    case BDOS_READSEQ: {
        fcb_t fcb;
        read_fcb(de, &fcb);
        char path[256];
        fcb_host_path(&fcb, path, sizeof(path));
        int slot = file_table_find(path);
        if (slot < 0) { ret8 = 0x10; break; } /* file not open */

        uint32_t rec = fcb_get_seq_record(&fcb);
        long offset  = (long)rec * CPM_SECTOR_SZ;
        FILE *fp = s_cpm.file_table[slot].fp;
        if (fseek(fp, offset, SEEK_SET) != 0) { ret8 = 0x10; break; }

        uint8_t buf[CPM_SECTOR_SZ];
        memset(buf, 0x1A, CPM_SECTOR_SZ); /* CP/M EOF = 0x1A */
        size_t n = fread(buf, 1, CPM_SECTOR_SZ, fp);

        if (n == 0) { ret8 = 0x01; break; } /* EOF */

        /* Copy to DMA */
        memcpy(&s_cpm.ram[s_cpm.dma_addr], buf, CPM_SECTOR_SZ);

        /* Advance sequential position */
        fcb.cr++;
        if (fcb.cr >= 128) {
            fcb.cr = 0;
            fcb.ex = (fcb.ex + 1) & 0x1F;
            if (fcb.ex == 0) fcb.s2 = (fcb.s2 + 1) & 0x3F;
        }
        write_fcb(de, &fcb);
        ret8 = 0x00;
        break;
    }

    /* ---- 21: Write Sequential ---- */
    case BDOS_WRITESEQ: {
        fcb_t fcb;
        read_fcb(de, &fcb);
        char path[256];
        fcb_host_path(&fcb, path, sizeof(path));
        int slot = file_table_find(path);
        if (slot < 0 || s_cpm.file_table[slot].read_only) { ret8 = 0xFF; break; }

        uint32_t rec = fcb_get_seq_record(&fcb);
        long offset  = (long)rec * CPM_SECTOR_SZ;
        FILE *fp = s_cpm.file_table[slot].fp;
        if (fseek(fp, offset, SEEK_SET) != 0) { ret8 = 0xFF; break; }
        if (fwrite(&s_cpm.ram[s_cpm.dma_addr], 1, CPM_SECTOR_SZ, fp) != CPM_SECTOR_SZ) { ret8 = 0xFF; break; }
        fflush(fp);

        /* Advance sequential position */
        fcb.cr++;
        if (fcb.cr >= 128) {
            fcb.cr = 0;
            fcb.ex = (fcb.ex + 1) & 0x1F;
            if (fcb.ex == 0) fcb.s2 = (fcb.s2 + 1) & 0x3F;
        }
        write_fcb(de, &fcb);
        ret8 = 0x00;
        break;
    }

    /* ---- 22: Make File ---- */
    case BDOS_MAKE: {
        fcb_t fcb;
        read_fcb(de, &fcb);
        char path[256];
        fcb_host_path(&fcb, path, sizeof(path));

        /* Make parent directory if needed */
        char dir_path[256];
        strncpy(dir_path, path, sizeof(dir_path)-1);
        char *last_slash = strrchr(dir_path, '/');
        if (last_slash) {
            *last_slash = 0;
            mkdir(dir_path, 0777);
        }

        int slot = file_table_alloc();
        if (slot < 0) { ret8 = 0xFF; break; }
        FILE *fp = fopen(path, "w+b");
        if (!fp) { ret8 = 0xFF; break; }
        s_cpm.file_table[slot].valid    = true;
        s_cpm.file_table[slot].fp       = fp;
        s_cpm.file_table[slot].read_only = false;
        strncpy(s_cpm.file_table[slot].path, path, sizeof(s_cpm.file_table[slot].path)-1);
        s_cpm.file_table[slot].fcb_addr = de;
        fcb.cr = fcb.ex = fcb.s1 = fcb.s2 = fcb.rc = 0;
        write_fcb(de, &fcb);
        ret8 = 0x00;
        break;
    }

    /* ---- 23: Rename File ---- */
    case BDOS_RENAME: {
        fcb_t fcb_src;
        read_fcb(de, &fcb_src);
        /* Second FCB is 16 bytes after the first */
        fcb_t fcb_dst;
        read_fcb((uint16_t)(de + 16), &fcb_dst);

        char src_path[256], dst_path[256];
        fcb_host_path(&fcb_src, src_path, sizeof(src_path));
        fcb_host_path(&fcb_dst, dst_path, sizeof(dst_path));

        ret8 = (rename(src_path, dst_path) == 0) ? 0x00 : 0xFF;
        break;
    }

    /* ---- 24: Return Login Vector ---- */
    case BDOS_LOGINVEC:
        ret16 = 0x0001; /* only drive A logged in by default */
        ret8  = 0x01;
        break;

    /* ---- 25: Return Current Disk ---- */
    case BDOS_CURDISK:
        ret8 = s_cpm.current_drive;
        break;

    /* ---- 26: Set DMA Address ---- */
    case BDOS_SETDMA:
        s_cpm.dma_addr = de;
        break;

    /* ---- 27: Get Allocation Vector (stub) ---- */
    case BDOS_ALLOCVEC:
        ret16 = 0x0000;
        break;

    /* ---- 28: Write Protect Disk ---- */
    case BDOS_WPROTDSK:
        /* Mark current disk as read-only */
        if (s_cpm.current_drive < CPM_MAX_DRIVES) {
            s_cpm.disk_ro[s_cpm.current_drive] = true;
        }
        break;

    /* ---- 29: Get Read-Only Vector ---- */
    case BDOS_ROTVEC: {
        uint16_t v = 0;
        for (int i = 0; i < CPM_MAX_DRIVES; i++) {
            if (s_cpm.disk_ro[i]) v |= (1 << i);
        }
        ret16 = v;
        ret8  = (uint8_t)v;
        break;
    }

    /* ---- 30: Set File Attributes (stub) ---- */
    case BDOS_SETATTR:
        ret8 = 0x00;
        break;

    /* ---- 31: Get Disk Parameters (stub - returns a minimal DPH) ---- */
    case BDOS_DISKPARMS:
        ret16 = 0x0000; /* NULL pointer - minimal stub */
        break;

    /* ---- 32: Set/Get User Code ---- */
    case BDOS_USERCODE:
        if (e == 0xFF) {
            ret8 = s_cpm.user_num;
        } else {
            s_cpm.user_num = e & 0x0F;
            ret8 = s_cpm.user_num;
        }
        break;

    /* ---- 33: Read Random ---- */
    case BDOS_READRND: {
        fcb_t fcb;
        read_fcb(de, &fcb);
        char path[256];
        fcb_host_path(&fcb, path, sizeof(path));
        int slot = file_table_find(path);
        if (slot < 0) { ret8 = 0x10; break; }

        uint32_t rec    = fcb_get_rnd_record(&fcb);
        long     offset = (long)rec * CPM_SECTOR_SZ;
        FILE    *fp     = s_cpm.file_table[slot].fp;
        if (fseek(fp, offset, SEEK_SET) != 0) { ret8 = 0x06; break; } /* past EOF */

        uint8_t buf[CPM_SECTOR_SZ];
        memset(buf, 0x1A, CPM_SECTOR_SZ);
        size_t n = fread(buf, 1, CPM_SECTOR_SZ, fp);
        if (n == 0) { ret8 = 0x01; break; } /* EOF */
        memcpy(&s_cpm.ram[s_cpm.dma_addr], buf, CPM_SECTOR_SZ);

        /* Update sequential position to match */
        fcb.cr   = (uint8_t)(rec & 0x7F);
        fcb.ex   = (uint8_t)((rec >> 7) & 0x1F);
        fcb.s2   = (uint8_t)((rec >> 12) & 0x3F);
        write_fcb(de, &fcb);
        ret8 = 0x00;
        break;
    }

    /* ---- 34: Write Random ---- */
    case BDOS_WRITERND: {
        fcb_t fcb;
        read_fcb(de, &fcb);
        char path[256];
        fcb_host_path(&fcb, path, sizeof(path));
        int slot = file_table_find(path);
        if (slot < 0 || s_cpm.file_table[slot].read_only) { ret8 = 0xFF; break; }

        uint32_t rec    = fcb_get_rnd_record(&fcb);
        long     offset = (long)rec * CPM_SECTOR_SZ;
        FILE    *fp     = s_cpm.file_table[slot].fp;
        if (fseek(fp, offset, SEEK_SET) != 0) { ret8 = 0xFF; break; }
        if (fwrite(&s_cpm.ram[s_cpm.dma_addr], 1, CPM_SECTOR_SZ, fp) != CPM_SECTOR_SZ) { ret8 = 0xFF; break; }
        fflush(fp);
        ret8 = 0x00;
        break;
    }

    /* ---- 35: Compute File Size ---- */
    case BDOS_FILESIZE: {
        fcb_t fcb;
        read_fcb(de, &fcb);
        char path[256];
        fcb_host_path(&fcb, path, sizeof(path));
        struct stat st;
        if (stat(path, &st) == 0) {
            uint32_t recs = (uint32_t)((st.st_size + CPM_SECTOR_SZ - 1) / CPM_SECTOR_SZ);
            fcb_set_rnd_record(&fcb, recs);
            write_fcb(de, &fcb);
            ret8 = 0x00;
        } else {
            ret8 = 0xFF;
        }
        break;
    }

    /* ---- 36: Set Random Record ---- */
    case BDOS_SETRND: {
        fcb_t fcb;
        read_fcb(de, &fcb);
        uint32_t rec = fcb_get_seq_record(&fcb);
        fcb_set_rnd_record(&fcb, rec);
        write_fcb(de, &fcb);
        break;
    }

    /* ---- 37: Reset Drive ---- */
    case BDOS_RESETDRV:
        break;

    /* ---- 40: Write Random with Zero Fill ---- */
    case BDOS_WRITEZERO: {
        /* Extend file with zeros to the random record position then write */
        fcb_t fcb;
        read_fcb(de, &fcb);
        char path[256];
        fcb_host_path(&fcb, path, sizeof(path));
        int slot = file_table_find(path);
        if (slot < 0 || s_cpm.file_table[slot].read_only) { ret8 = 0xFF; break; }
        uint32_t rec    = fcb_get_rnd_record(&fcb);
        long     offset = (long)rec * CPM_SECTOR_SZ;
        FILE    *fp     = s_cpm.file_table[slot].fp;
        /* Get current size */
        fseek(fp, 0, SEEK_END);
        long cur_size = ftell(fp);
        /* Fill gap with zeros */
        if (offset > cur_size) {
            uint8_t zeros[CPM_SECTOR_SZ];
            memset(zeros, 0, CPM_SECTOR_SZ);
            while (cur_size < offset) {
                fwrite(zeros, 1, CPM_SECTOR_SZ, fp);
                cur_size += CPM_SECTOR_SZ;
            }
        }
        if (fseek(fp, offset, SEEK_SET) != 0) { ret8 = 0xFF; break; }
        if (fwrite(&s_cpm.ram[s_cpm.dma_addr], 1, CPM_SECTOR_SZ, fp) != CPM_SECTOR_SZ) { ret8 = 0xFF; break; }
        fflush(fp);
        ret8 = 0x00;
        break;
    }

    default:
        ESP_LOGW(TAG, "Unknown BDOS function %d", func);
        ret8 = 0xFF;
        break;
    }

    /* Set return values in CPU registers */
    cpu->A = ret8;
    cpu->L = ret8;
    cpu->H = (uint8_t)(ret16 >> 8);
    if (ret16 > 0xFF) {
        cpu->L = (uint8_t)ret16;
    }

    return true; /* trap handled — caller will simulate RET */
}

/* =====================================================================
 * Install CP/M zero-page stubs
 * ===================================================================== */
static void install_cpm_stubs(void) {
    uint8_t *ram = s_cpm.ram;

    /* 0x0000: Warm-boot vector (HALT — our trap stops on HALT) */
    ram[0x0000] = 0x76; /* HALT */

    /* 0x0001: IOBYTE */
    ram[0x0001] = 0x00;

    /* 0x0002: default drive/user (0 = drive A, user 0) */
    ram[0x0002] = 0x00;

    /* 0x0003-0x0004: pad (part of warm-boot jump in real CP/M) */
    ram[0x0003] = 0x00;
    ram[0x0004] = 0x00;

    /* 0x0005: BDOS entry — our trap handles this automatically.
     * Place a HALT here so stray code doesn't hang; the trap fires before HALT executes. */
    ram[0x0005] = 0x76; /* HALT */
    ram[0x0006] = 0x00;
    ram[0x0007] = 0x00;

    /* 0x0080: Default DMA buffer (128 bytes) — leave as zeros */
}

/* =====================================================================
 * Create drive directory if needed
 * ===================================================================== */
static void ensure_drive_dirs(void) {
    const char *dir = s_cpm.cfg.disk_dir ? s_cpm.cfg.disk_dir : "/sdcard/cpm";
    /* Create base directory */
    mkdir(dir, 0777);
    /* Create per-drive subdirectories */
    for (int i = 0; i < CPM_MAX_DRIVES; i++) {
        char path[256];
        snprintf(path, sizeof(path), "%s/%c", dir, 'a' + i);
        mkdir(path, 0777);
    }
}

/* =====================================================================
 * Public API
 * ===================================================================== */

esp_err_t z80_emu_init(const z80_emu_config_t *config) {
    memset(&s_cpm, 0, sizeof(s_cpm));

    if (config) {
        s_cpm.cfg = *config;
    } else {
        s_cpm.cfg.disk_dir    = "/sdcard/cpm";
        s_cpm.cfg.enable_lcd  = true;
        s_cpm.cfg.cpu_freq_hz = 4000000; /* 4 MHz default */
    }

    /* Allocate 64KB RAM from PSRAM */
    s_cpm.ram = heap_caps_calloc(1, CPM_RAM_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_cpm.ram) {
        /* Fallback: internal RAM */
        s_cpm.ram = calloc(1, CPM_RAM_SIZE);
        if (!s_cpm.ram) {
            ESP_LOGE(TAG, "Failed to allocate Z80 RAM");
            return ESP_ERR_NO_MEM;
        }
    }

    /* Ensure drive directories exist */
    ensure_drive_dirs();

    /* Init CPU */
    z80_init(&s_cpm.cpu);
    s_cpm.cpu.mem_read    = z80_mem_read;
    s_cpm.cpu.mem_write   = z80_mem_write;
    s_cpm.cpu.io_read     = z80_io_read;
    s_cpm.cpu.io_write    = z80_io_write;
    s_cpm.cpu.trap_cb     = cpm_bdos_call;
    s_cpm.cpu.trap_addr   = CPM_ADDR_BDOS;
    s_cpm.cpu.user_data   = &s_cpm;

    /* Install CP/M zero-page stubs */
    install_cpm_stubs();

    /* Set initial state */
    s_cpm.dma_addr      = CPM_ADDR_DMA;
    s_cpm.current_drive = 0;
    s_cpm.iobyte        = 0x00;

    /* Reset CPU to TPA start */
    z80_reset(&s_cpm.cpu);
    s_cpm.cpu.PC = CPM_ADDR_TPA;  /* Programs start at 0x0100 */
    s_cpm.cpu.SP = 0xFF00;        /* Stack near top */

    ESP_LOGI(TAG, "CP/M 2.2 emulator initialized (64KB RAM at %p)", s_cpm.ram);
    return ESP_OK;
}

esp_err_t z80_emu_load_com(const char *path) {
    if (!s_cpm.ram) return ESP_ERR_INVALID_STATE;

    FILE *f = fopen(path, "rb");
    if (!f) {
        ESP_LOGE(TAG, "Cannot open .COM file: %s", path);
        return ESP_ERR_NOT_FOUND;
    }

    /* Get size */
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (sz <= 0 || sz > (CPM_RAM_SIZE - CPM_ADDR_TPA)) {
        fclose(f);
        ESP_LOGE(TAG, ".COM file too large: %ld bytes", sz);
        return ESP_ERR_INVALID_SIZE;
    }

    size_t read = fread(&s_cpm.ram[CPM_ADDR_TPA], 1, (size_t)sz, f);
    fclose(f);

    if ((long)read != sz) {
        ESP_LOGE(TAG, "Failed to read .COM file");
        return ESP_FAIL;
    }

    /* Set PC to TPA start */
    s_cpm.cpu.PC = CPM_ADDR_TPA;
    s_cpm.cpu.SP = 0xFF00;

    /* Set up default FCB at 0x005C from filename */
    const char *basename = strrchr(path, '/');
    basename = basename ? basename + 1 : path;
    memset(&s_cpm.ram[CPM_ADDR_FCB], 0, 36);
    const char *dot = strchr(basename, '.');
    int nlen = dot ? (int)(dot - basename) : (int)strlen(basename);
    if (nlen > 8) nlen = 8;
    s_cpm.ram[CPM_ADDR_FCB] = 0; /* default drive */
    memset(&s_cpm.ram[CPM_ADDR_FCB + 1], ' ', 11);
    for (int i = 0; i < nlen; i++) {
        s_cpm.ram[CPM_ADDR_FCB + 1 + i] = toupper((unsigned char)basename[i]);
    }
    if (dot) {
        const char *ext = dot + 1;
        int elen = (int)strlen(ext);
        if (elen > 3) elen = 3;
        for (int i = 0; i < elen; i++) {
            s_cpm.ram[CPM_ADDR_FCB + 9 + i] = toupper((unsigned char)ext[i]);
        }
    }

    ESP_LOGI(TAG, "Loaded %s (%ld bytes) to TPA (0x%04X)", basename, sz, CPM_ADDR_TPA);
    return ESP_OK;
}

void z80_emu_run(void) {
    if (!s_cpm.ram) {
        ESP_LOGE(TAG, "Not initialized");
        return;
    }

    s_cpm.running       = true;
    s_cpm.stop_requested = false;

    cpm_printf("\r\nCP/M 2.2 for ESP32-P4 (Z80 @ %lu MHz)\r\n",
               (unsigned long)(s_cpm.cfg.cpu_freq_hz / 1000000));
    cpm_printf("Drive A: %s/a/\r\n",
               s_cpm.cfg.disk_dir ? s_cpm.cfg.disk_dir : "/sdcard/cpm");
    cpm_printf("TPA: 0x%04X  Stack: 0x%04X  PC: 0x%04X\r\n",
               CPM_ADDR_TPA, s_cpm.cpu.SP, s_cpm.cpu.PC);
    cpm_printf("Press Ctrl+C to exit\r\n\r\n");

    /* Timing throttle: how many cycles per yield */
    const int CYCLES_PER_SLICE = 10000;
    const int DELAY_MS         = 1; /* 10000 cycles @ 4MHz ≈ 2.5ms; yield 1ms */

    uint64_t last_yield = 0;

    while (!s_cpm.stop_requested) {
        /* Check for HALT at warm-boot vector */
        if (s_cpm.cpu.halted) {
            if (s_cpm.cpu.PC == 0x0001) {
                /* Halted at warm-boot vector */
                cpm_print("\r\n[CP/M: program returned to warm boot]\r\n");
                break;
            }
            /* Any other HALT: check for keyboard interrupt */
            cpm_poll_kbd();
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        /* Execute a slice of instructions */
        int slice_cycles = 0;
        while (slice_cycles < CYCLES_PER_SLICE && !s_cpm.stop_requested) {
            /* Check HALT-at-warm-boot */
            if (s_cpm.cpu.halted) break;

            /* Also check PC==0x0000 (warm boot JP) */
            if (s_cpm.cpu.PC == 0x0000) {
                cpm_print("\r\n[CP/M: warm boot]\r\n");
                s_cpm.stop_requested = true;
                break;
            }

            slice_cycles += z80_step(&s_cpm.cpu);
        }

        /* Poll keyboard and yield periodically */
        cpm_poll_kbd();
        uint64_t now = s_cpm.cpu.total_cycles;
        if (now - last_yield > (uint64_t)CYCLES_PER_SLICE * 4) {
            vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
            last_yield = now;
        }
    }

    s_cpm.running = false;
    cpm_print("\r\n[CP/M emulator stopped]\r\n\r\n");
    ESP_LOGI(TAG, "Stopped. Total cycles: %llu", s_cpm.cpu.total_cycles);
}

void z80_emu_stop(void) {
    s_cpm.stop_requested = true;
}

void z80_emu_deinit(void) {
    s_cpm.stop_requested = true;

    /* Close all open files */
    for (int i = 0; i < CPM_MAX_OPEN; i++) {
        if (s_cpm.file_table[i].valid && s_cpm.file_table[i].fp) {
            fclose(s_cpm.file_table[i].fp);
        }
    }

    /* Close search dir */
    if (s_cpm.search_dir) {
        closedir(s_cpm.search_dir);
        s_cpm.search_dir = NULL;
    }

    /* Free RAM */
    if (s_cpm.ram) {
        free(s_cpm.ram);
        s_cpm.ram = NULL;
    }

    memset(&s_cpm, 0, sizeof(s_cpm));
    ESP_LOGI(TAG, "CP/M emulator deinitialized");
}

bool z80_emu_is_running(void) {
    return s_cpm.running;
}

void z80_emu_dump_state(void) {
    z80_dump_state(&s_cpm.cpu);
    printf("CP/M: drive=%c  DMA=0x%04X  user=%d\n",
           'A' + s_cpm.current_drive, s_cpm.dma_addr, s_cpm.user_num);
}
