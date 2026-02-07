/**
 * @file i8086_emulator.c
 * @brief Intel 8086 PC Emulator - Integration Layer
 * 
 * This file integrates the i8086 CPU core with:
 * - BIOS interrupt services (INT 10h, 13h, 16h, 21h)
 * - Disk image management
 * - Video text mode output
 * - Keyboard input
 * 
 * Part of the ESP32-P4 Multi-CPU Emulator Project
 */

#include "i8086_emulator.h"
#include "i8086_cpu.h"
#include "disk_image.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "i8086";

// External console_printf from main
extern int console_printf(const char *fmt, ...);

// Emulator state
static struct {
    bool initialized;
    i86_state_t state;
    i86_config_t config;
    i86_stats_t stats;
    TaskHandle_t task_handle;
    SemaphoreHandle_t mutex;
    
    // Keyboard buffer (circular)
    uint16_t kbd_buffer[16];
    int kbd_head;
    int kbd_tail;
    
    // Video mode
    uint8_t video_mode;
    uint8_t cursor_x, cursor_y;
    uint8_t text_attr;
    
    // Disk drive mapping - no separate pointers, use disk_image.h API
} s_i86;

// ============================================================================
// BIOS Data Area (BDA) Addresses
// ============================================================================
#define BDA_BASE        0x0400
#define BDA_COM1        0x0400
#define BDA_LPT1        0x0408
#define BDA_EQUIPMENT   0x0410
#define BDA_MEMORY_SIZE 0x0413
#define BDA_KBD_HEAD    0x041A
#define BDA_KBD_TAIL    0x041C
#define BDA_KBD_BUFFER  0x041E
#define BDA_VIDEO_MODE  0x0449
#define BDA_VIDEO_COLS  0x044A
#define BDA_CURSOR_POS  0x0450
#define BDA_CURSOR_TYPE 0x0460
#define BDA_TIMER_COUNT 0x046C
#define BDA_TIMER_OFLOW 0x0470
#define BDA_DISK_STATUS 0x0474

// ============================================================================
// Port I/O Handlers
// ============================================================================

static uint8_t i86_read_port(void *ctx, uint16_t port) {
    switch (port) {
        case 0x20:  // PIC1 command
            return 0x00;
        case 0x21:  // PIC1 data (IRQ mask)
            return 0xFF;  // All IRQs masked by default
        case 0x40:  // PIT channel 0 counter
        case 0x41:  // PIT channel 1 counter
        case 0x42:  // PIT channel 2 counter
            return 0x00;
        case 0x60:  // Keyboard data
            if (s_i86.kbd_head != s_i86.kbd_tail) {
                return s_i86.kbd_buffer[s_i86.kbd_tail] & 0xFF;
            }
            return 0x00;
        case 0x61:  // System control port B
            return 0x00;
        case 0x64:  // Keyboard status
            return (s_i86.kbd_head != s_i86.kbd_tail) ? 0x01 : 0x00;
        case 0x3DA: // CGA status register
            // Toggle VSync bit to prevent timeout
            {
                static uint8_t cga_status = 0;
                cga_status ^= 0x08;  // Toggle vertical retrace
                return cga_status;
            }
        default:
            ESP_LOGD(TAG, "IN port %04X (unhandled)", port);
            return 0xFF;
    }
}

static void i86_write_port(void *ctx, uint16_t port, uint8_t value) {
    switch (port) {
        case 0x20:  // PIC1 command
        case 0x21:  // PIC1 data
        case 0xA0:  // PIC2 command
        case 0xA1:  // PIC2 data
            break;  // Ignore PIC writes
        case 0x40:  // PIT channel 0
        case 0x41:  // PIT channel 1
        case 0x42:  // PIT channel 2
        case 0x43:  // PIT control
            break;  // Ignore timer writes
        case 0x60:  // Keyboard data
        case 0x64:  // Keyboard command
            break;  // Ignore keyboard controller writes
        case 0x3D4: // CRT controller address
        case 0x3D5: // CRT controller data
            break;  // Ignore CRT controller
        default:
            ESP_LOGD(TAG, "OUT port %04X = %02X (unhandled)", port, value);
            break;
    }
}

// ============================================================================
// Video Memory Handlers
// ============================================================================

static void i86_write_video8(void *ctx, uint32_t addr, uint8_t value) {
    uint8_t *mem = i86_cpu_get_memory();
    mem[addr] = value;
    
    // Text mode output (B8000-B8FFF)
    if (addr >= 0xB8000 && addr < 0xB9000 && s_i86.video_mode <= 3) {
        uint32_t offset = addr - 0xB8000;
        if ((offset & 1) == 0) {
            // Character byte - could output to console here
        }
    }
}

static void i86_write_video16(void *ctx, uint32_t addr, uint16_t value) {
    i86_write_video8(ctx, addr, value & 0xFF);
    i86_write_video8(ctx, addr + 1, value >> 8);
}

static uint8_t i86_read_video8(void *ctx, uint32_t addr) {
    uint8_t *mem = i86_cpu_get_memory();
    return mem[addr];
}

static uint16_t i86_read_video16(void *ctx, uint32_t addr) {
    return i86_read_video8(ctx, addr) | (i86_read_video8(ctx, addr + 1) << 8);
}

// ============================================================================
// BIOS Interrupt Handlers
// ============================================================================

static bool handle_int10h(void) {
    // INT 10h - Video BIOS services
    uint8_t ah = (i86_cpu_get_ax() >> 8) & 0xFF;
    uint8_t al = i86_cpu_get_ax() & 0xFF;
    uint8_t bh = (i86_cpu_get_bx() >> 8) & 0xFF;
    uint8_t bl = i86_cpu_get_bx() & 0xFF;
    uint8_t dh = (i86_cpu_get_dx() >> 8) & 0xFF;
    uint8_t dl = i86_cpu_get_dx() & 0xFF;
    uint16_t cx = i86_cpu_get_cx();
    
    (void)bh; // Suppress unused warning
    
    switch (ah) {
        case 0x00: // Set video mode
            s_i86.video_mode = al;
            s_i86.cursor_x = 0;
            s_i86.cursor_y = 0;
            s_i86.text_attr = 0x07;
            // Clear screen
            if (s_i86.video_mode <= 3) {
                uint8_t *video = i86_cpu_get_video();
                memset(video + 0x18000, 0, 4000);  // B8000-B8FFF
            }
            ESP_LOGI(TAG, "INT 10h: Set video mode %02Xh", al);
            return true;
            
        case 0x01: // Set cursor shape
            return true;
            
        case 0x02: // Set cursor position
            s_i86.cursor_x = dl;
            s_i86.cursor_y = dh;
            return true;
            
        case 0x03: // Get cursor position
            i86_cpu_set_dx((s_i86.cursor_y << 8) | s_i86.cursor_x);
            i86_cpu_set_cx(0x0607);
            return true;
            
        case 0x05: // Select active page
            return true;
            
        case 0x06: // Scroll up
        case 0x07: // Scroll down
            if (al == 0) {
                uint8_t *video = i86_cpu_get_video();
                for (int i = 0; i < 4000; i += 2) {
                    video[0x18000 + i] = 0x20;
                    video[0x18000 + i + 1] = bl;
                }
            }
            return true;
            
        case 0x08: // Read char/attr at cursor
            {
                uint8_t *video = i86_cpu_get_video();
                int offset = (s_i86.cursor_y * 80 + s_i86.cursor_x) * 2;
                i86_cpu_set_ax(video[0x18000 + offset] | (video[0x18000 + offset + 1] << 8));
            }
            return true;
            
        case 0x09: // Write char/attr at cursor (CX times)
        case 0x0A: // Write char at cursor (CX times)
            {
                uint8_t *video = i86_cpu_get_video();
                int offset = (s_i86.cursor_y * 80 + s_i86.cursor_x) * 2;
                for (uint16_t i = 0; i < cx && offset < 4000; i++, offset += 2) {
                    video[0x18000 + offset] = al;
                    if (ah == 0x09) {
                        video[0x18000 + offset + 1] = bl;
                    }
                }
            }
            return true;
            
        case 0x0E: // Teletype output
            {
                if (al == '\r') {
                    s_i86.cursor_x = 0;
                } else if (al == '\n') {
                    s_i86.cursor_y++;
                    if (s_i86.cursor_y >= 25) {
                        s_i86.cursor_y = 24;
                    }
                } else if (al == '\b') {
                    if (s_i86.cursor_x > 0) s_i86.cursor_x--;
                } else if (al >= 0x20) {
                    uint8_t *video = i86_cpu_get_video();
                    int offset = (s_i86.cursor_y * 80 + s_i86.cursor_x) * 2;
                    video[0x18000 + offset] = al;
                    video[0x18000 + offset + 1] = s_i86.text_attr;
                    s_i86.cursor_x++;
                    if (s_i86.cursor_x >= 80) {
                        s_i86.cursor_x = 0;
                        s_i86.cursor_y++;
                    }
                    putchar(al);
                }
            }
            return true;
            
        case 0x0F: // Get video mode
            i86_cpu_set_ax((80 << 8) | s_i86.video_mode);
            i86_cpu_set_bx(0);
            return true;
            
        default:
            ESP_LOGD(TAG, "INT 10h AH=%02Xh (unhandled)", ah);
            return true;
    }
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
    
    // Map drive number to our drive array
    // 00h=A:, 01h=B: (floppies), 80h=C:, 81h=D: (hard disks)
    uint8_t drive_idx = (dl < 0x80) ? dl : (dl - 0x80 + 2);
    if (drive_idx >= MAX_DISK_DRIVES) drive_idx = 0;
    
    switch (ah) {
        case 0x00: // Reset disk system
            i86_cpu_set_ax(0x0000);
            return true;
            
        case 0x01: // Get disk status
            i86_cpu_set_ax(0x0000);
            return true;
            
        case 0x02: // Read sectors
            if (!disk_is_mounted(drive_idx)) {
                ESP_LOGW(TAG, "INT 13h: No disk in drive %02Xh", dl);
                i86_cpu_set_ax(0x0100);
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
                    return true;
                }
                uint8_t *buffer = i86_cpu_get_memory() + 16 * es + bx;
                
                ESP_LOGD(TAG, "INT 13h: Read %d sectors from C%d/H%d/S%d (LBA %d) to %04X:%04X",
                         count, cylinder, head, sector, lba, es, bx);
                
                int sectors_read = disk_read_sectors(drive_idx, (uint32_t)lba, count, buffer);
                if (sectors_read > 0) {
                    i86_cpu_set_ax(sectors_read & 0xFF);
                    s_i86.stats.disk_reads++;
                } else {
                    i86_cpu_set_ax(0x0400);  // Sector not found
                }
            }
            return true;
            
        case 0x03: // Write sectors
            if (!disk_is_mounted(drive_idx)) {
                i86_cpu_set_ax(0x0100);
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
                    return true;
                }
                uint8_t *buffer = i86_cpu_get_memory() + 16 * es + bx;
                
                int sectors_written = disk_write_sectors(drive_idx, (uint32_t)lba, count, buffer);
                if (sectors_written > 0) {
                    i86_cpu_set_ax(sectors_written & 0xFF);
                    s_i86.stats.disk_writes++;
                } else {
                    i86_cpu_set_ax(0x0400);  // Write fault
                }
            }
            return true;
            
        case 0x04: // Verify sectors
            i86_cpu_set_ax(al);
            return true;
            
        case 0x08: // Get drive parameters
            {
                disk_info_t info;
                if (disk_get_info(drive_idx, &info) != ESP_OK || !info.mounted) {
                    i86_cpu_set_ax(0x0700);
                    return true;
                }
                i86_cpu_set_ax(0x0000);
                i86_cpu_set_bx(dl < 0x80 ? 0x0004 : 0x0000);
                i86_cpu_set_cx(((info.geometry.cylinders - 1) & 0xFF) | 
                               (((info.geometry.cylinders - 1) >> 2) & 0xC0) |
                               ((info.geometry.sectors_per_track & 0x3F) << 8));
                i86_cpu_set_dx(((info.geometry.heads - 1) << 8) | 1);
            }
            return true;
            
        case 0x15: // Get disk type
            {
                disk_info_t info;
                if (disk_get_info(drive_idx, &info) == ESP_OK && info.mounted) {
                    i86_cpu_set_ax(dl < 0x80 ? 0x0200 : 0x0300);
                    uint32_t sectors = info.geometry.total_sectors;
                    i86_cpu_set_cx(sectors >> 16);
                    i86_cpu_set_dx(sectors & 0xFFFF);
                } else {
                    i86_cpu_set_ax(0x0000);
                }
            }
            return true;
            
        default:
            ESP_LOGD(TAG, "INT 13h AH=%02Xh (unhandled)", ah);
            i86_cpu_set_ax(0x0100);
            return true;
    }
}

static bool handle_int16h(void) {
    // INT 16h - Keyboard BIOS services
    uint8_t ah = (i86_cpu_get_ax() >> 8) & 0xFF;
    
    switch (ah) {
        case 0x00: // Wait for keypress
        case 0x10: // Wait for keypress (enhanced)
            while (s_i86.kbd_head == s_i86.kbd_tail) {
                vTaskDelay(pdMS_TO_TICKS(10));
                if (s_i86.state != I86_STATE_RUNNING) {
                    return true;
                }
            }
            {
                uint16_t key = s_i86.kbd_buffer[s_i86.kbd_tail];
                s_i86.kbd_tail = (s_i86.kbd_tail + 1) % 16;
                i86_cpu_set_ax(key);
            }
            return true;
            
        case 0x01: // Check for keypress
        case 0x11: // Check for keypress (enhanced)
            if (s_i86.kbd_head != s_i86.kbd_tail) {
                i86_cpu_set_ax(s_i86.kbd_buffer[s_i86.kbd_tail]);
            } else {
                i86_cpu_set_ax(0x0000);
            }
            return true;
            
        case 0x02: // Get shift flags
        case 0x12: // Get extended shift flags
            i86_cpu_set_ax(0x0000);
            return true;
            
        default:
            ESP_LOGD(TAG, "INT 16h AH=%02Xh (unhandled)", ah);
            return true;
    }
}

static bool handle_int19h(void) {
    // INT 19h - Bootstrap loader
    ESP_LOGI(TAG, "INT 19h: Bootstrap loader");
    
    for (int i = 0; i < MAX_DISK_DRIVES; i++) {
        if (disk_is_mounted(i)) {
            uint8_t *buffer = i86_cpu_get_memory() + 0x7C00;
            int sectors = disk_read_sectors(i, 0, 1, buffer);
            if (sectors == 1) {
                if (buffer[510] == 0x55 && buffer[511] == 0xAA) {
                    ESP_LOGI(TAG, "Booting from drive %d", i);
                    i86_cpu_set_cs(0x0000);
                    i86_cpu_set_ip(0x7C00);
                    i86_cpu_set_dx(i < 2 ? i : 0x80 + (i - 2));
                    return true;
                }
            }
        }
    }
    
    ESP_LOGW(TAG, "No bootable disk found");
    return true;
}

static bool handle_int21h(void) {
    // INT 21h - DOS services (minimal implementation)
    uint8_t ah = (i86_cpu_get_ax() >> 8) & 0xFF;
    
    switch (ah) {
        case 0x02: // Character output
            {
                char c = i86_cpu_get_dx() & 0xFF;
                putchar(c);
            }
            return true;
            
        case 0x09: // String output
            {
                uint8_t *mem = i86_cpu_get_memory();
                uint32_t addr = 16 * i86_cpu_get_ds() + i86_cpu_get_dx();
                while (mem[addr] != '$') {
                    putchar(mem[addr++]);
                }
            }
            return true;
            
        case 0x25: // Set interrupt vector
        case 0x35: // Get interrupt vector
            return true;
            
        case 0x30: // Get DOS version
            i86_cpu_set_ax(0x0500);
            i86_cpu_set_bx(0x0000);
            i86_cpu_set_cx(0x0000);
            return true;
            
        case 0x4C: // Terminate program
            ESP_LOGI(TAG, "INT 21h AH=4Ch: Program terminated with code %02Xh",
                     i86_cpu_get_ax() & 0xFF);
            s_i86.state = I86_STATE_HALTED;
            return true;
            
        default:
            ESP_LOGD(TAG, "INT 21h AH=%02Xh (unhandled)", ah);
            return true;
    }
}

// Main interrupt dispatcher
static bool i86_interrupt_handler(void *ctx, uint8_t num) {
    s_i86.stats.interrupts_processed++;
    
    switch (num) {
        case 0x10: return handle_int10h();
        case 0x13: return handle_int13h();
        case 0x16: return handle_int16h();
        case 0x19: return handle_int19h();
        case 0x21: return handle_int21h();
        default:
            ESP_LOGD(TAG, "INT %02Xh (unhandled)", num);
            return false;
    }
}

// ============================================================================
// Minimal BIOS Initialization
// ============================================================================

static void setup_ivt_and_bios(void) {
    uint8_t *mem = i86_cpu_get_memory();
    
    // Set up IVT
    for (int i = 0; i < 256; i++) {
        mem[i * 4 + 0] = 0x00;
        mem[i * 4 + 1] = 0xFF;
        mem[i * 4 + 2] = 0x00;
        mem[i * 4 + 3] = 0xF0;
    }
    
    // IRET at F000:FF00
    mem[0xFFF00] = 0xCF;
    
    // BDA setup
    mem[BDA_EQUIPMENT] = 0x21;
    mem[BDA_EQUIPMENT + 1] = 0x00;
    mem[BDA_MEMORY_SIZE] = 640 & 0xFF;
    mem[BDA_MEMORY_SIZE + 1] = 640 >> 8;
    mem[BDA_VIDEO_MODE] = 0x03;
    mem[BDA_VIDEO_COLS] = 80;
    
    // BIOS entry at FFFF:0000
    mem[0xFFFF0] = 0xEA;
    mem[0xFFFF1] = 0x5B;
    mem[0xFFFF2] = 0xE0;
    mem[0xFFFF3] = 0x00;
    mem[0xFFFF4] = 0xF0;
    
    // Boot code at F000:E05B
    mem[0xFE05B] = 0xCD;
    mem[0xFE05C] = 0x19;
    mem[0xFE05D] = 0xF4;
    mem[0xFE05E] = 0xEB;
    mem[0xFE05F] = 0xFC;
    
    ESP_LOGI(TAG, "IVT and minimal BIOS initialized");
}

// ============================================================================
// Public API Implementation
// ============================================================================

esp_err_t i86_init(i86_config_t *config) {
    if (s_i86.initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing i8086 emulator...");

    if (config) {
        s_i86.config = *config;
    } else {
        s_i86.config.ram_size = I86_RAM_SIZE;
        s_i86.config.cpu_freq_mhz = 8;
        s_i86.config.enable_fpu = false;
        s_i86.config.enable_sound = false;
        s_i86.config.boot_disk = NULL;
    }

    if (i86_cpu_init() != 0) {
        ESP_LOGE(TAG, "Failed to initialize CPU core");
        return ESP_ERR_NO_MEM;
    }
    
    i86_cpu_set_callbacks(
        NULL,
        i86_read_port,
        i86_write_port,
        i86_read_video8,
        i86_write_video8,
        i86_read_video16,
        i86_write_video16,
        i86_interrupt_handler
    );

    disk_init();

    s_i86.kbd_head = 0;
    s_i86.kbd_tail = 0;
    
    s_i86.video_mode = 0x03;
    s_i86.cursor_x = 0;
    s_i86.cursor_y = 0;
    s_i86.text_attr = 0x07;

    s_i86.mutex = xSemaphoreCreateMutex();
    
    memset(&s_i86.stats, 0, sizeof(s_i86.stats));
    s_i86.state = I86_STATE_STOPPED;
    s_i86.initialized = true;

    setup_ivt_and_bios();

    ESP_LOGI(TAG, "i8086 emulator initialized");
    ESP_LOGI(TAG, "  RAM: 640 KB");
    ESP_LOGI(TAG, "  Video: 128 KB");
    ESP_LOGI(TAG, "  CPU Speed: %u MHz", s_i86.config.cpu_freq_mhz);

    return ESP_OK;
}

void i86_reset(void) {
    if (!s_i86.initialized) return;

    ESP_LOGI(TAG, "Resetting i8086 CPU...");
    
    i86_cpu_reset();
    setup_ivt_and_bios();
    
    s_i86.kbd_head = 0;
    s_i86.kbd_tail = 0;
    s_i86.video_mode = 0x03;
    s_i86.cursor_x = 0;
    s_i86.cursor_y = 0;
    
    memset(&s_i86.stats, 0, sizeof(s_i86.stats));
    s_i86.state = I86_STATE_HALTED;
}

uint32_t i86_run(uint32_t count) {
    if (!s_i86.initialized) return 0;
    
    s_i86.state = I86_STATE_RUNNING;
    uint32_t executed = i86_cpu_run(count);
    s_i86.stats.instructions_executed += executed;
    
    if (i86_cpu_is_halted()) {
        s_i86.state = I86_STATE_HALTED;
    }
    
    return executed;
}

esp_err_t i86_step(void) {
    if (!s_i86.initialized) return ESP_ERR_INVALID_STATE;
    
    i86_cpu_step();
    s_i86.stats.instructions_executed++;
    return ESP_OK;
}

void i86_stop(void) {
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
             "=== i8086 CPU State ===\n"
             "AX=%04X  BX=%04X  CX=%04X  DX=%04X\n"
             "SI=%04X  DI=%04X  BP=%04X  SP=%04X\n"
             "CS=%04X  DS=%04X  ES=%04X  SS=%04X\n"
             "IP=%04X  FLAGS=%04X [%c%c%c%c%c%c%c%c%c]\n"
             "\nState: %s\n"
             "Instructions: %llu\n"
             "Interrupts: %"PRIu32"  Disk: %"PRIu32"R/%"PRIu32"W\n",
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
             s_i86.state == I86_STATE_RUNNING ? "Running" :
             s_i86.state == I86_STATE_HALTED ? "Halted" :
             s_i86.state == I86_STATE_STOPPED ? "Stopped" : "Error",
             s_i86.stats.instructions_executed,
             s_i86.stats.interrupts_processed,
             s_i86.stats.disk_reads,
             s_i86.stats.disk_writes);
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
    if (!s_i86.initialized || !data) {
        return ESP_ERR_INVALID_ARG;
    }

    if (addr + size > I8086_TOTAL_MEMORY) {
        ESP_LOGE(TAG, "Binary too large or address out of range");
        return ESP_ERR_INVALID_SIZE;
    }

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
    
    int next = (s_i86.kbd_head + 1) % 16;
    if (next != s_i86.kbd_tail) {
        s_i86.kbd_buffer[s_i86.kbd_head] = (scancode << 8) | ascii;
        s_i86.kbd_head = next;
    }
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
    
    // Unmount if already mounted
    if (disk_is_mounted(drive)) {
        disk_unmount(drive);
    }
    
    esp_err_t err = disk_mount(drive, path, false);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount disk %d from %s", drive, path);
        return err;
    }
    
    ESP_LOGI(TAG, "Mounted %s as drive %c:", path, drive < 2 ? 'A' + drive : 'C' + (drive - 2));
    return ESP_OK;
}

void i86_deinit(void) {
    if (!s_i86.initialized) return;

    ESP_LOGI(TAG, "Shutting down i8086 emulator...");

    s_i86.state = I86_STATE_STOPPED;

    if (s_i86.task_handle) {
        vTaskDelete(s_i86.task_handle);
        s_i86.task_handle = NULL;
    }

    // disk_deinit unmounts all drives
    disk_deinit();
    i86_cpu_shutdown();
    
    if (s_i86.mutex) {
        vSemaphoreDelete(s_i86.mutex);
        s_i86.mutex = NULL;
    }

    s_i86.initialized = false;
    ESP_LOGI(TAG, "i8086 emulator shut down");
}
