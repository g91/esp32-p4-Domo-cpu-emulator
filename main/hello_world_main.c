/*
 * Motorola 68000 Emulator Application
 * ESP32-P4 with 16MB RAM @ 60MHz simulation
 * With LCD Console and PS/2 Keyboard support
 * With i8086 PC Emulator support
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <sys/stat.h>
#include <dirent.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_fat.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "driver/sdmmc_host.h"
#include "sdmmc_cmd.h"
#if SOC_SDMMC_IO_POWER_EXTERNAL
#include "sd_pwr_ctrl_by_on_chip_ldo.h"
#endif
#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"
#include "nvs_flash.h"
#include "m68k_emulator.h"
#include "i8086_emulator.h"
#include "mos6502_emulator.h"
#include "video_card.h"
#include "disk_image.h"
#include "lcd_console.h"
#include "ps2_keyboard.h"
#include "usb_keyboard.h"
// Bluetooth support via ESP32-C6 SDIO using NimBLE + ESP-Hosted
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
// WiFi support via ESP32-C6 SDIO
#include "esp_wifi.h"
#include "esp_event.h" 
#include "esp_netif.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "driver/usb_serial_jtag.h"
#include "bus_controller.h"
#include "atari_st.h"
#include "amiga.h"
#include "wifi_ftp_server.h"
#include "ssh_debug_server.h"
#include <ctype.h>

static const char* TAG = "m68k_app";

// WiFi state for ESP32-C6 SDIO communication
static bool wifi_initialized = false;
static bool wifi_connected = false;
static char connected_ssid[32] = {0};
static esp_netif_ip_info_t ip_info = {0};

// Bluetooth state for ESP32-C6 SDIO communication
static bool bt_initialized = false;
static bool bt_scanning = false;

// i386 PC emulator (uses tiny386 engine via pc_emulator module)
static TaskHandle_t s_i86_task_handle = NULL;
static void i86_run_task(void *arg);

// Atari ST emulator (uses M68K core + Atari ST hardware)
static TaskHandle_t s_atari_task_handle = NULL;
static void atari_st_run_task(void *arg);

// Amiga emulator (uses M68K core + Amiga custom chips)
static TaskHandle_t s_amiga_task_handle = NULL;
static void amiga_run_task(void *arg);

#define MAX_BLE_DEVICES 20
typedef struct {
    uint8_t address[6];
    char name[64];
    int8_t rssi;
    bool valid;
} ble_device_t;
static ble_device_t ble_devices[MAX_BLE_DEVICES];
static int ble_device_count = 0;

// WiFi event handler
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WiFi station started");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_connected = false;
        memset(connected_ssid, 0, sizeof(connected_ssid));
        ftp_server_stop();
        ESP_LOGI(TAG, "WiFi disconnected, FTP server stopped");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ip_info = event->ip_info;
        wifi_connected = true;
        ESP_LOGI(TAG, "WiFi connected, IP: " IPSTR, IP2STR(&event->ip_info.ip));
        
        // Start FTP server automatically when WiFi connects
        esp_err_t ret = ftp_server_init();
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "FTP server started at ftp://" IPSTR ":21 (root: /sdcard)", IP2STR(&event->ip_info.ip));
        } else {
            ESP_LOGE(TAG, "Failed to start FTP server: %s", esp_err_to_name(ret));
        }
        
        // Start SSH/Telnet debug server
        ret = ssh_debug_server_init();
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Telnet debug console started at: telnet " IPSTR " 23", IP2STR(&event->ip_info.ip));
            ESP_LOGI(TAG, "Use 'crash' and 'dump' commands for M68K debugging");
        } else {
            ESP_LOGE(TAG, "Failed to start telnet server: %s", esp_err_to_name(ret));
        }
    }
}

// NimBLE GAP event handler for device discovery
static int gap_event_handler(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_DISC:
            // Device discovered during scan
            if (ble_device_count < MAX_BLE_DEVICES) {
                bool already_found = false;
                for (int i = 0; i < ble_device_count; i++) {
                    if (memcmp(ble_devices[i].address, event->disc.addr.val, 6) == 0) {
                        already_found = true;
                        ble_devices[i].rssi = event->disc.rssi;  // Update RSSI
                        break;
                    }
                }
                
                if (!already_found) {
                    memcpy(ble_devices[ble_device_count].address, event->disc.addr.val, 6);
                    ble_devices[ble_device_count].rssi = event->disc.rssi;
                    
                    // Extract device name from advertising data
                    struct ble_hs_adv_fields fields;
                    bool name_found = false;
                    
                    if (event->disc.length_data > 0) {
                        int rc = ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);
                        if (rc == 0) {
                            // Try complete local name first
                            if (fields.name != NULL && fields.name_len > 0 && fields.name_len < 64) {
                                memcpy(ble_devices[ble_device_count].name, fields.name, fields.name_len);
                                ble_devices[ble_device_count].name[fields.name_len] = '\0';
                                name_found = true;
                            }
                        }
                    }
                    
                    if (!name_found) {
                        snprintf(ble_devices[ble_device_count].name, sizeof(ble_devices[0].name), "Unknown");
                    }
                    
                    ble_devices[ble_device_count].valid = true;
                    ble_device_count++;
                }
            }
            return 0;
            
        case BLE_GAP_EVENT_DISC_COMPLETE:
            ESP_LOGI(TAG, "BLE scan complete");
            bt_scanning = false;
            return 0;
            
        default:
            return 0;
    }
}

// NimBLE host task
static void nimble_host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// Flag to track if LCD console is active
static bool lcd_console_active = false;

// Active CPU selection
typedef enum {
    ACTIVE_CPU_M68K,
    ACTIVE_CPU_I8086,
    ACTIVE_CPU_6502
} active_cpu_t;

static active_cpu_t active_cpu = ACTIVE_CPU_M68K;

// Command history file
#define HISTORY_FILE "/sdcard/.bash_history"
#define MAX_HISTORY_LINES 500

// Local command history for arrow key navigation
static char *cmd_history[MAX_HISTORY_LINES];
static int cmd_history_count = 0;

// Current working directory (for ls, cd, etc.)
static char current_working_dir[256] = "/sdcard";

/*
 * Resolve a path relative to the current working directory
 * Returns a malloc'd string that must be freed by the caller
 */
static char* resolve_path(const char *path) {
    char *result = malloc(512);
    if (!result) return NULL;
    
    if (path == NULL || strlen(path) == 0 || strcmp(path, ".") == 0) {
        // Current directory
        strcpy(result, current_working_dir);
    } else if (path[0] == '/') {
        // Absolute path
        strcpy(result, path);
    } else if (strcmp(path, "..") == 0) {
        // Parent directory
        strcpy(result, current_working_dir);
        char *last_slash = strrchr(result, '/');
        if (last_slash && last_slash != result) {
            *last_slash = '\0';
        } else {
            strcpy(result, "/sdcard");
        }
    } else if (strncmp(path, "../", 3) == 0) {
        // Parent + more path
        strcpy(result, current_working_dir);
        char *last_slash = strrchr(result, '/');
        if (last_slash && last_slash != result) {
            *last_slash = '\0';
        }
        strcat(result, "/");
        strcat(result, path + 3);
    } else {
        // Relative path
        snprintf(result, 512, "%s/%s", current_working_dir, path);
    }
    
    // Ensure we don't go above /sdcard
    if (strncmp(result, "/sdcard", 7) != 0) {
        strcpy(result, "/sdcard");
    }
    
    return result;
}

/*
 * Dual-output printf - sends to both UART and LCD console
 * Non-static: also used by i8086_emulator.c via console_printf()
 */
void dual_printf(const char *format, ...) {
    char buffer[512];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    // Always print to UART
    printf("%s", buffer);
    
    // Route to video card framebuffer if active (avoids LCD conflict)
    if (video_card_is_initialized()) {
        video_card_print(buffer);
    } else if (lcd_console_active && lcd_console_is_initialized()) {
        lcd_console_print(buffer);
    }
}

// Real exported function for other modules (i8086_emulator.c, etc.)
int console_printf(const char *fmt, ...) {
    char buffer[512];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    printf("%s", buffer);
    if (video_card_is_initialized()) {
        video_card_print(buffer);
    } else if (lcd_console_active && lcd_console_is_initialized()) {
        lcd_console_print(buffer);
    }
    return len;
}

// Boot splash screen with animation
static void show_boot_splash(void) {
    if (!lcd_console_is_initialized()) return;
    
    // Single clear and draw - no flash effects to avoid SPI queue overflow
    lcd_console_clear();
    vTaskDelay(pdMS_TO_TICKS(50));  // Let SPI queue drain
    
    // Draw simple ASCII boot logo (avoid Unicode issues)
    const char *logo[] = {
        "================================",
        "   M68K FIRMWARE BIOS v1.0",
        "      ESP32-P4 @ 400MHz",
        "================================",
        "",
        "  Motorola 68000 @ 60MHz",
        "  Memory: 16MB PSRAM",
        "  Bus Controller: Active",
        "",
        "  System Ready",
    };
    
    for (int i = 0; i < sizeof(logo)/sizeof(logo[0]); i++) {
        lcd_console_print(logo[i]);
        lcd_console_print("\n");
    }
    
    vTaskDelay(pdMS_TO_TICKS(1000));
}

// SD Card variables
static sdmmc_card_t *sd_card = NULL;
static bool sd_mounted = false;

// Load command history from SD card
static void load_history(void) {
    if (!sd_mounted) return;
    
    FILE *f = fopen(HISTORY_FILE, "r");
    if (f == NULL) {
        ESP_LOGI(TAG, "No history file found");
        return;
    }
    
    char line[256];
    int count = 0;
    while (fgets(line, sizeof(line), f) != NULL && count < MAX_HISTORY_LINES) {
        // Remove newline
        line[strcspn(line, "\r\n")] = 0;
        if (strlen(line) > 0) {
            linenoiseHistoryAdd(line);
            // Also add to our local history
            cmd_history[cmd_history_count] = strdup(line);
            if (cmd_history[cmd_history_count]) {
                cmd_history_count++;
            }
            count++;
        }
    }
    fclose(f);
    ESP_LOGI(TAG, "Loaded %d history entries", count);
}

// Save command history to SD card
__attribute__((unused)) static void save_history(void) {
    if (!sd_mounted) return;
    
    FILE *f = fopen(HISTORY_FILE, "w");
    if (f == NULL) {
        ESP_LOGW(TAG, "Failed to save history");
        return;
    }
    
    // Get history from linenoise and save to file
    // Note: linenoise doesn't expose history directly, so we'll save on each add
    fclose(f);
}

// Append command to history file
static void append_to_history_file(const char *line) {
    if (!sd_mounted || line == NULL || strlen(line) == 0) return;
    
    FILE *f = fopen(HISTORY_FILE, "a");
    if (f != NULL) {
        fprintf(f, "%s\n", line);
        fclose(f);
    }
}

// SD Card pin definitions (SDMMC Slot 0 - ESP32-P4-NANO)
// Note: NANO doesn't have WiFi companion chip, so all SDMMC pins available
// Using default ESP32-P4 SDMMC pins
#define SD_CLK_PIN  43  // GPIO43 (SDMMC CLK)
#define SD_CMD_PIN  44  // GPIO44 (SDMMC CMD)
#define SD_D0_PIN   39  // GPIO39 (SDMMC D0)
#define SD_D1_PIN   40  // GPIO40 (SDMMC D1)
#define SD_D2_PIN   41  // GPIO41 (SDMMC D2)
#define SD_D3_PIN   42  // GPIO42 (SDMMC D3)

// I2S Audio pins (if ES8311 Codec available on NANO)
// NANO may not have audio codec - these are placeholder pins
#define I2S_MCLK_PIN    33  // GPIO33
#define I2S_SCLK_PIN    32  // GPIO32
#define I2S_DOUT_PIN    25  // GPIO25 (USBIP1_P0)
#define I2S_LRCK_PIN    26  // GPIO26 (USBIP1_N1)
#define I2S_DIN_PIN     36  // GPIO36
#define PA_CTRL_PIN     53  // GPIO53 (ADC2_CHANNEL6)

// I2C pins (shared bus for peripherals)
#define I2C_SCL_PIN     8   // GPIO8 (TOUCH_CHANNEL6) - SCL on pinout
#define I2C_SDA_PIN     7   // GPIO7 (TOUCH_CHANNEL5) - SDA on pinout
#define ES8311_I2C_ADDR 0x18

// Example 68000 program: Simple loop that increments D0
static const uint8_t example_program[] = {
    // Reset vectors at 0x00000000
    0x00, 0x00, 0xFF, 0xFC,  // Initial SSP = 0x00FFFFFC (top of RAM)
    0x00, 0x00, 0x00, 0x10,  // Initial PC = 0x00000010 (start of program)
    
    0x00, 0x00, 0x00, 0x00,  // Padding
    0x00, 0x00, 0x00, 0x00,
    
    // Program starts at 0x00000010
    0x70, 0x00,              // MOVEQ #0, D0          ; D0 = 0
    0x52, 0x40,              // ADDQ.W #1, D0         ; D0++
    0x0C, 0x40, 0x00, 0x0A,  // CMPI.W #10, D0        ; Compare D0 with 10
    0x6F, 0xF8,              // BLE -8                ; Branch if <= (loop)
    0x4E, 0x71,              // NOP                   ; No operation
    0x60, 0xFE,              // BRA -2                ; Loop forever
};

/* ====== SD Card Functions ====== */

static esp_err_t mount_sd_card(void) {
    if (sd_mounted) {
        ESP_LOGW(TAG, "SD card already mounted");
        return ESP_OK;
    }

    esp_err_t ret;

    // Note: ESP32-P4-NANO uses SDMMC Slot 0 for SD card (4-bit mode)
    // Unlike the WIFI6 board, NANO doesn't have WiFi companion chip on SDIO
    // So SDMMC Slot 0 is the only slot, used for SD card
    
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 20,  // Increased for FTP file operations
        .allocation_unit_size = 16 * 1024
    };

    // Use SDMMC Slot 0 (4-wire mode)
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.slot = SDMMC_HOST_SLOT_0;  // NANO only has Slot 0 (no WiFi chip)
    // Note: max_freq_khz defaults to SDMMC_FREQ_DEFAULT (20MHz) which is safer for init

    // ESP32-P4 requires LDO power control for SD card pins
#if SOC_SDMMC_IO_POWER_EXTERNAL
    sd_pwr_ctrl_ldo_config_t ldo_config = {
        .ldo_chan_id = 4,  // LDO channel 4 for ESP32-P4
    };
    sd_pwr_ctrl_handle_t pwr_ctrl_handle = NULL;

    ret = sd_pwr_ctrl_new_on_chip_ldo(&ldo_config, &pwr_ctrl_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create LDO power control: %s", esp_err_to_name(ret));
        return ret;
    }
    host.pwr_ctrl_handle = pwr_ctrl_handle;
    ESP_LOGI(TAG, "LDO power control initialized (channel 4)");
#endif

    // SDMMC Slot 0 configuration (4-bit mode)
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 4;  // 4-bit mode
    slot_config.clk = SD_CLK_PIN;
    slot_config.cmd = SD_CMD_PIN;
    slot_config.d0 = SD_D0_PIN;
    slot_config.d1 = SD_D1_PIN;
    slot_config.d2 = SD_D2_PIN;
    slot_config.d3 = SD_D3_PIN;
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    ESP_LOGI(TAG, "Mounting SD card via SDMMC Slot 0 (4-bit, CLK=%d, CMD=%d, D0=%d, D1=%d, D2=%d, D3=%d)...",
             SD_CLK_PIN, SD_CMD_PIN, SD_D0_PIN, SD_D1_PIN, SD_D2_PIN, SD_D3_PIN);
    ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &sd_card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. Format card as FAT32.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SD card: %s", esp_err_to_name(ret));
        }
        return ret;
    }

    sd_mounted = true;
    ESP_LOGI(TAG, "SD card mounted successfully via SDMMC Slot 0");

    // Print card info
    sdmmc_card_print_info(stdout, sd_card);
    printf("SD Card: %.2f GB\n", ((uint64_t)sd_card->csd.capacity) * sd_card->csd.sector_size / (1024.0 * 1024.0 * 1024.0));

    return ESP_OK;
}

static void unmount_sd_card(void) {
    if (!sd_mounted) {
        return;
    }

    esp_vfs_fat_sdcard_unmount("/sdcard", sd_card);
    sd_mounted = false;
    
    // Note: LDO power control handle cleanup would go here if we stored it
    // For now, it's local to mount_sd_card() so no cleanup needed
    
    sd_card = NULL;
    ESP_LOGI(TAG, "SD card unmounted");
}

/* ====== Auto-Boot Feature ====== */

// M68K OS execution task
static void m68k_os_task(void *arg) {
    printf("M68K-OS task started\n");
    
    // Video card is already initialized and showing welcome text.
    // Do NOT call lcd_console_clear() - that fights with the video card refresh task.
    // If we need a clean screen, use the video card's text clear instead.
    if (video_card_is_initialized()) {
        video_card_text_clear();
        video_card_print("M68K-OS Booting...\n\n");
    }
    
    // Run M68K continuously - it will handle console I/O through bus controller
    while (true) {
        // Execute instructions in batches
        m68k_run(1000);
        
        // Check if halted
        if (m68k_is_halted()) {
            printf("M68K-OS halted\n");
            break;
        }
        
        // Small yield to allow other tasks
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    printf("M68K-OS task ended\n");
    vTaskDelete(NULL);
}

// Forward declarations for boot menu
static TaskHandle_t s_6502_task_handle = NULL;
static void mos6502_run_task(void *arg);

// Try to auto-boot OS.bin from SD card root
// Returns true if OS was loaded and started, false otherwise
static bool try_autoboot(void) {
    if (!sd_mounted) {
        return false;
    }
    
    const char *os_path = "/sdcard/OS.bin";
    
    // Check if OS.bin exists
    struct stat st;
    bool os_exists = (stat(os_path, &st) == 0);
    
    // Check if any i8086 boot disk exists
    bool i86_disk_found = false;
    const char *i86_disk_paths[] = {
        "/sdcard/freedos.img",
        "/sdcard/boot.img",
        "/sdcard/dos.img",
        "/sdcard/floppy.img",
        "/sdcard/hdd.img",
        NULL
    };
    const char *i86_boot_disk = NULL;
    for (int i = 0; i86_disk_paths[i] != NULL; i++) {
        struct stat st2;
        if (stat(i86_disk_paths[i], &st2) == 0) {
            i86_disk_found = true;
            i86_boot_disk = i86_disk_paths[i];
            break;
        }
    }
    
    // Check for 6502 program
    bool mos6502_prog_found = false;
    struct stat st6502;
    if (stat("/sdcard/6502.bin", &st6502) == 0) {
        mos6502_prog_found = true;
    }

    // Check for C64 ROMs (need at least kernal.rom)
    bool c64_roms_found = false;
    struct stat st_c64;
    if (stat("/sdcard/c64/kernal.rom", &st_c64) == 0) {
        c64_roms_found = true;
    }

    // Check for VIC-20 ROMs (need at least kernal.rom)
    bool vic20_roms_found = false;
    struct stat st_vic20;
    if (stat("/sdcard/vic20/kernal.rom", &st_vic20) == 0) {
        vic20_roms_found = true;
    }

    // Check for Atari ST TOS ROM
    bool atari_tos_found = false;
    const char *atari_tos_path = NULL;
    const char *atari_tos_paths[] = {
        "/sdcard/atarist/tos.img",
        "/sdcard/atarist/tos.rom",
        "/sdcard/atarist/tos104.img",
        "/sdcard/tos.img",
        "/sdcard/tos.rom",
        NULL
    };
    for (int i = 0; atari_tos_paths[i] != NULL; i++) {
        struct stat st_tos;
        if (stat(atari_tos_paths[i], &st_tos) == 0) {
            atari_tos_found = true;
            atari_tos_path = atari_tos_paths[i];
            break;
        }
    }
    // Check for Atari ST floppy disk
    const char *atari_floppy_path = NULL;
    const char *atari_floppy_paths[] = {
        "/sdcard/atarist/disk.st",
        "/sdcard/atarist/floppy.st",
        "/sdcard/disk.st",
        NULL
    };
    for (int i = 0; atari_floppy_paths[i] != NULL; i++) {
        struct stat st_fl;
        if (stat(atari_floppy_paths[i], &st_fl) == 0) {
            atari_floppy_path = atari_floppy_paths[i];
            break;
        }
    }

    // Check for Amiga Kickstart ROM
    bool amiga_kick_found = false;
    const char *amiga_kick_path = NULL;
    const char *amiga_kick_paths[] = {
        "/sdcard/amiga/kick.rom",
        "/sdcard/amiga/kick13.rom",
        "/sdcard/amiga/kick20.rom",
        "/sdcard/amiga/kick31.rom",
        "/sdcard/kick.rom",
        NULL
    };
    for (int i = 0; amiga_kick_paths[i] != NULL; i++) {
        struct stat st_ak;
        if (stat(amiga_kick_paths[i], &st_ak) == 0) {
            amiga_kick_found = true;
            amiga_kick_path = amiga_kick_paths[i];
            break;
        }
    }
    // Check for Amiga floppy disk (ADF)
    const char *amiga_floppy_path = NULL;
    const char *amiga_floppy_paths[] = {
        "/sdcard/amiga/df0.adf",
        "/sdcard/amiga/workbench.adf",
        "/sdcard/amiga/boot.adf",
        NULL
    };
    for (int i = 0; amiga_floppy_paths[i] != NULL; i++) {
        struct stat st_af;
        if (stat(amiga_floppy_paths[i], &st_af) == 0) {
            amiga_floppy_path = amiga_floppy_paths[i];
            break;
        }
    }
    // Check for Amiga hard disk (HDF)
    const char *amiga_hdf_path = NULL;
    const char *amiga_hdf_paths[] = {
        "/sdcard/amiga/hd0.hdf",
        "/sdcard/amiga/hardfile.hdf",
        NULL
    };
    for (int i = 0; amiga_hdf_paths[i] != NULL; i++) {
        struct stat st_ah;
        if (stat(amiga_hdf_paths[i], &st_ah) == 0) {
            amiga_hdf_path = amiga_hdf_paths[i];
            break;
        }
    }

    // If nothing bootable exists, skip boot menu
    if (!os_exists && !i86_disk_found && !mos6502_prog_found && !c64_roms_found && !vic20_roms_found && !atari_tos_found && !amiga_kick_found) {
        return false;
    }
    
    // Show boot menu
    printf("\n");
    printf("╔══════════════════════════════════════════════════╗\n");
    printf("║              SYSTEM BOOT MENU                    ║\n");
    printf("╚══════════════════════════════════════════════════╝\n");
    printf("\n");
    if (os_exists) {
        printf("  [1] Boot M68K-OS (Motorola 68000, from OS.bin)\n");
    } else {
        printf("  [1] (M68K-OS not found)\n");
    }
    printf("  [2] Enter BIOS Shell\n");
    if (i86_disk_found) {
        printf("  [3] Boot IBM PC (Intel 8086, from %s)\n", i86_boot_disk);
    } else {
        printf("  [3] Boot IBM PC (mount disk first)\n");
    }
    if (mos6502_prog_found) {
        printf("  [4] Boot MOS 6502 (from 6502.bin)\n");
    } else {
        printf("  [4] Boot MOS 6502 (bare, no program)\n");
    }
    if (c64_roms_found) {
        printf("  [5] Boot Commodore 64 (C64 emulation)\n");
    } else {
        printf("  [5] Boot C64 (ROMs needed in /sdcard/c64/)\n");
    }
    if (vic20_roms_found) {
        printf("  [6] Boot VIC-20 (VIC-20 emulation)\n");
    } else {
        printf("  [6] Boot VIC-20 (ROMs needed in /sdcard/vic20/)\n");
    }
    if (atari_tos_found) {
        printf("  [7] Boot Atari ST (TOS from %s)\n", atari_tos_path);
    } else {
        printf("  [7] Boot Atari ST (TOS needed in /sdcard/atarist/)\n");
    }
    if (amiga_kick_found) {
        printf("  [8] Boot Amiga (Kickstart from %s)\n", amiga_kick_path);
    } else {
        printf("  [8] Boot Amiga (ROM needed in /sdcard/amiga/)\n");
    }
    printf("\n");
    printf("Select option (1-8): ");
    
    // Show boot menu on display - use video card if active, otherwise lcd_console
    if (video_card_is_initialized()) {
        video_card_text_clear();
        video_card_print("SYSTEM BOOT MENU\n");
        video_card_print("================\n\n");
        if (os_exists) video_card_print("[1] Boot M68K-OS\n");
        video_card_print("[2] Enter BIOS\n");
        if (i86_disk_found) video_card_print("[3] Boot IBM PC\n");
        video_card_print("[4] MOS 6502\n");
        if (c64_roms_found) video_card_print("[5] Commodore 64\n");
        if (vic20_roms_found) video_card_print("[6] VIC-20\n");
        if (atari_tos_found) video_card_print("[7] Atari ST\n");
        if (amiga_kick_found) video_card_print("[8] Amiga\n");
        video_card_print("\nSelect 1-8...\n");
    } else if (lcd_console_is_initialized()) {
        lcd_console_clear();
        lcd_console_print("SYSTEM BOOT MENU\n");
        lcd_console_print("================\n\n");
        if (os_exists) {
            lcd_console_print("[1] Boot M68K-OS\n");
        }
        lcd_console_print("[2] Enter BIOS\n");
        if (i86_disk_found) {
            lcd_console_print("[3] Boot IBM PC\n");
        }
        lcd_console_print("[4] MOS 6502\n");
        if (c64_roms_found) {
            lcd_console_print("[5] Commodore 64\n");
        }
        if (vic20_roms_found) {
            lcd_console_print("[6] VIC-20\n");
        }
        if (atari_tos_found) {
            lcd_console_print("[7] Atari ST\n");
        }
        if (amiga_kick_found) {
            lcd_console_print("[8] Amiga\n");
        }
        lcd_console_print("\nSelect 1-8...\n");
    }
    
    // Wait for user selection with 5-second timeout (default to BIOS if no input)
    int selection = 0;
    int timeout_count = 0;
    while (selection == 0) {
        // Check PS/2 keyboard input first (if initialized)
        if (ps2_keyboard_is_initialized()) {
            if (ps2_keyboard_available()) {
                uint8_t key = ps2_keyboard_read();
                if (key == '1' && os_exists) {
                    selection = 1;
                    printf("1\n");
                } else if (key == '2') {
                    selection = 2;
                    printf("2\n");
                } else if (key == '3') {
                    selection = 3;
                    printf("3\n");
                } else if (key == '4') {
                    selection = 4;
                    printf("4\n");
                } else if (key == '5') {
                    selection = 5;
                    printf("5\n");
                } else if (key == '6') {
                    selection = 6;
                    printf("6\n");
                } else if (key == '7') {
                    selection = 7;
                    printf("7\n");
                } else if (key == '8') {
                    selection = 8;
                    printf("8\n");
                }
            }
        }

        // Check USB keyboard input (if initialized)
        if (selection == 0 && usb_keyboard_is_initialized() && usb_keyboard_available()) {
            uint8_t key = usb_keyboard_read();
            if (key == '1' && os_exists) { selection = 1; printf("1\n"); }
            else if (key == '2') { selection = 2; printf("2\n"); }
            else if (key == '3') { selection = 3; printf("3\n"); }
            else if (key == '4') { selection = 4; printf("4\n"); }
            else if (key == '5') { selection = 5; printf("5\n"); }
            else if (key == '6') { selection = 6; printf("6\n"); }
            else if (key == '7') { selection = 7; printf("7\n"); }
            else if (key == '8') { selection = 8; printf("8\n"); }
        }

        // Check UART/USB Serial input (non-blocking)
        uint8_t uart_data;
        int len = uart_read_bytes(UART_NUM_0, &uart_data, 1, pdMS_TO_TICKS(10));
        if (len > 0) {
            if (uart_data == '1' && os_exists) {
                selection = 1;
                printf("1\n");
            } else if (uart_data == '2') {
                selection = 2;
                printf("2\n");
            } else if (uart_data == '3') {
                selection = 3;
                printf("3\n");
            } else if (uart_data == '4') {
                selection = 4;
                printf("4\n");
            } else if (uart_data == '5') {
                selection = 5;
                printf("5\n");
            } else if (uart_data == '6') {
                selection = 6;
                printf("6\n");
            } else if (uart_data == '7') {
                selection = 7;
                printf("7\n");
            } else if (uart_data == '8') {
                selection = 8;
                printf("8\n");
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));
        timeout_count++;
    }
    
    if (selection == 2) {
        // User chose BIOS
        printf("\nEntering BIOS...\n\n");
        if (video_card_is_initialized()) {
            video_card_text_clear();
            video_card_print("Entering BIOS...\n");
        } else if (lcd_console_is_initialized()) {
            lcd_console_clear();
            lcd_console_print("Entering BIOS...\n");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
        return false;
    }
    
    if (selection == 3) {
        // User chose i8086 PC mode
        printf("\nBooting IBM PC (i8086)...\n\n");
        if (video_card_is_initialized()) {
            video_card_text_clear();
            video_card_print("Booting IBM PC...\n\n");
        } else if (lcd_console_is_initialized()) {
            lcd_console_print("Booting IBM PC...\n\n");
        }

        // Initialize video card GPU for display output
        if (!video_card_is_initialized()) {
            video_card_init();
        }

        // Initialize i8086 emulator
        i86_config_t i86_config = {
            .ram_size = I86_RAM_SIZE,
            .cpu_freq_mhz = 8,
            .enable_fpu = false,
            .enable_sound = false,
            .boot_disk = i86_boot_disk,
        };
        esp_err_t i86_ret = i86_init(&i86_config);
        if (i86_ret != ESP_OK) {
            printf("Failed to init i8086: %s\n", esp_err_to_name(i86_ret));
            return false;
        }
        
        // If a boot disk was found, mount it
        if (i86_boot_disk) {
            disk_type_t type = disk_detect_type(i86_boot_disk);
            int drive = (type == DISK_TYPE_HARD_DISK) ? 2 : 0;
            i86_mount_disk(drive, i86_boot_disk);
            printf("Mounted %s as drive %c:\n", i86_boot_disk,
                   drive < 2 ? 'A' + drive : 'C' + (drive - 2));
        }
        
        // Boot in a FreeRTOS task
        BaseType_t task_ret = xTaskCreate(i86_run_task, "i8086_emu", 16384, NULL, 5, &s_i86_task_handle);
        if (task_ret == pdPASS) {
            printf("i8086 PC emulator running. UART available for monitoring.\n");
            printf("Use 'stop86' to halt, or reset device.\n\n");
            while (s_i86_task_handle != NULL) {
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
            return false;  // Return to BIOS shell when emulator exits
        } else {
            printf("Failed to start i8086 task\n");
            return false;
        }
    }
    
    // ---- MOS 6502 / C64 / VIC-20 boot handlers ----
    if (selection >= 4 && selection <= 6) {
        mos6502_machine_mode_t mode;
        const char *mode_name;

        switch (selection) {
        case 4:
            mode = MOS6502_MODE_BARE;
            mode_name = "MOS 6502 (bare)";
            break;
        case 5:
            mode = MOS6502_MODE_C64;
            mode_name = "Commodore 64";
            break;
        case 6:
        default:
            mode = MOS6502_MODE_VIC20;
            mode_name = "VIC-20";
            break;
        }

        printf("\nBooting %s...\n\n", mode_name);
        {
            char lcd_msg[64];
            snprintf(lcd_msg, sizeof(lcd_msg), "Booting %s...\n\n", mode_name);
            if (video_card_is_initialized()) {
                video_card_text_clear();
                video_card_print(lcd_msg);
            } else if (lcd_console_is_initialized()) {
                lcd_console_clear();
                lcd_console_print(lcd_msg);
            }
        }

        // Initialize video card GPU (handles all display output)
        if (!video_card_is_initialized()) {
            esp_err_t vid_ret = video_card_init();
            if (vid_ret != ESP_OK) {
                printf("Warning: Video card init failed: %s\n", esp_err_to_name(vid_ret));
            }
        }

        // Initialize emulator in the selected mode
        esp_err_t ret = mos6502_emu_init_mode(mode);
        if (ret != ESP_OK) {
            printf("Failed to init 6502: %s\n", esp_err_to_name(ret));
            return false;
        }

        // Load ROMs for C64/VIC-20 modes
        if (mode != MOS6502_MODE_BARE) {
            ret = mos6502_emu_load_roms();
            if (ret != ESP_OK) {
                printf("Warning: ROM loading had issues\n");
            }
        }

        // For bare mode, try to load 6502.bin
        if (mode == MOS6502_MODE_BARE && mos6502_prog_found) {
            FILE *f = fopen("/sdcard/6502.bin", "rb");
            if (f) {
                fseek(f, 0, SEEK_END);
                long fsize = ftell(f);
                fseek(f, 0, SEEK_SET);
                uint8_t *buf = malloc(fsize);
                if (buf) {
                    fread(buf, 1, fsize, f);
                    mos6502_emu_load_program(buf, fsize, 0xE000);
                    free(buf);
                    printf("Loaded 6502.bin (%ld bytes at $E000)\n", fsize);
                }
                fclose(f);
            }
        }

        // Reset CPU (reads vectors from $FFFC/$FFFD)
        mos6502_emu_reset();

        // Set active CPU
        active_cpu = ACTIVE_CPU_6502;

        // Start running in a FreeRTOS task
        if (s_6502_task_handle == NULL) {
            BaseType_t task_ret = xTaskCreate(mos6502_run_task, "6502_run", 8192, NULL, 5, &s_6502_task_handle);
            if (task_ret == pdPASS) {
                printf("%s running. Use 'stop6502' to halt.\n\n", mode_name);
                while (s_6502_task_handle != NULL) {
                    vTaskDelay(pdMS_TO_TICKS(1000));
                }
                return false;  // Return to BIOS shell
            } else {
                printf("Failed to start 6502 task\n");
            }
        }
        return false;
    }

    // ---- Atari ST boot handler ----
    if (selection == 7) {
        printf("\nBooting Atari ST...\n\n");
        if (video_card_is_initialized()) {
            video_card_text_clear();
            video_card_print("Booting Atari ST...\n\n");
        } else if (lcd_console_is_initialized()) {
            lcd_console_clear();
            lcd_console_print("Booting Atari ST...\n\n");
        }

        atari_st_config_t atari_config = {
            .ram_size = ATARI_RAM_SIZE,
            .tos_path = atari_tos_path,
            .floppy_a_path = atari_floppy_path,
            .floppy_b_path = NULL,
            .cart_path = NULL,
            .enable_lcd_output = true,
        };

        esp_err_t atari_ret = atari_st_init(&atari_config);
        if (atari_ret != ESP_OK) {
            printf("Failed to init Atari ST: %s\n", esp_err_to_name(atari_ret));
            return false;
        }

        // Run in a FreeRTOS task
        BaseType_t task_ret = xTaskCreate(atari_st_run_task, "atari_st", 16384, NULL, 5, &s_atari_task_handle);
        if (task_ret == pdPASS) {
            printf("Atari ST emulator running. Press Ctrl+C in UART to stop.\n");
            if (atari_floppy_path) {
                printf("Floppy A: %s\n", atari_floppy_path);
            }
            printf("\n");
            while (s_atari_task_handle != NULL) {
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
            return false;  // Return to BIOS shell
        } else {
            printf("Failed to start Atari ST task\n");
            atari_st_destroy();
            return false;
        }
    }

    // ---- Amiga boot handler ----
    if (selection == 8) {
        if (!amiga_kick_found) {
            printf("\nNo Kickstart ROM found. Place kick.rom in /sdcard/amiga/\n");
            return false;
        }
        printf("\nBooting Amiga...\n\n");
        if (video_card_is_initialized()) {
            video_card_text_clear();
            video_card_print("Booting Amiga...\n\n");
        } else if (lcd_console_is_initialized()) {
            lcd_console_clear();
            lcd_console_print("Booting Amiga...\n\n");
        }

        // Initialize video card GPU for display output
        if (!video_card_is_initialized()) {
            esp_err_t vid_ret = video_card_init();
            if (vid_ret != ESP_OK) {
                printf("Warning: Video card init failed: %s\n", esp_err_to_name(vid_ret));
            }
        }

        amiga_config_t amiga_config = {
            .chip_ram_size = AMIGA_CHIP_RAM_DEFAULT,
            .slow_ram_enabled = false,
            .kick_path = amiga_kick_path,
            .floppy_path = {NULL, NULL, NULL, NULL},
            .hdf_path = {NULL, NULL},
            .pal_mode = true,
        };
        if (amiga_floppy_path) {
            amiga_config.floppy_path[0] = amiga_floppy_path;
        }
        if (amiga_hdf_path) {
            amiga_config.hdf_path[0] = amiga_hdf_path;
        }

        esp_err_t amiga_ret = amiga_init(&amiga_config);
        if (amiga_ret != ESP_OK) {
            printf("Failed to init Amiga: %s\n", esp_err_to_name(amiga_ret));
            return false;
        }

        // Run in a FreeRTOS task
        BaseType_t task_ret = xTaskCreate(amiga_run_task, "amiga", 16384, NULL, 5, &s_amiga_task_handle);
        if (task_ret == pdPASS) {
            printf("Amiga emulator running. Press Ctrl+C in UART to stop.\n");
            if (amiga_floppy_path) {
                printf("Floppy DF0: %s\n", amiga_floppy_path);
            }
            if (amiga_hdf_path) {
                printf("Hard disk: %s\n", amiga_hdf_path);
            }
            printf("\n");
            while (s_amiga_task_handle != NULL) {
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
            return false;  // Return to BIOS shell
        } else {
            printf("Failed to start Amiga task\n");
            amiga_destroy();
            return false;
        }
    }

    if (selection != 1 || !os_exists) {
        return false;
    }

    // User chose to boot M68K OS (selection == 1)
    printf("\nBooting M68K-OS...\n\n");
    if (video_card_is_initialized()) {
        video_card_text_clear();
        video_card_print("Booting M68K-OS...\n\n");
    } else if (lcd_console_is_initialized()) {
        lcd_console_clear();
        lcd_console_print("Booting M68K-OS...\n\n");
    }

    // Initialize GPU video card for M68K-OS
    if (!video_card_is_initialized()) {
        esp_err_t vid_ret = video_card_init();
        if (vid_ret == ESP_OK) {
            printf("Video card GPU initialized for M68K-OS\n");
        } else {
            printf("Warning: Video card init failed: %s\n", esp_err_to_name(vid_ret));
        }
    }
    // Set emulator memory read callback so GPU can read vertex/index data from M68K RAM
    video_card_set_emu_read(m68k_read_memory_8);
    
    long file_size = st.st_size;
    if (file_size <= 0 || file_size > 16 * 1024 * 1024) {
        printf("Invalid OS.bin size: %ld bytes\n", file_size);
        return false;
    }
    
    FILE *file = fopen(os_path, "rb");
    if (file == NULL) {
        printf("Failed to open OS.bin\n");
        return false;
    }
    
    // Allocate buffer and read file
    uint8_t *buffer = heap_caps_malloc(file_size, MALLOC_CAP_SPIRAM);
    if (buffer == NULL) {
        printf("Failed to allocate %ld bytes for OS\n", file_size);
        fclose(file);
        return false;
    }
    
    size_t bytes_read = fread(buffer, 1, file_size, file);
    fclose(file);
    
    if (bytes_read != file_size) {
        printf("Failed to read OS.bin: got %d, expected %ld\n", (int)bytes_read, file_size);
        free(buffer);
        return false;
    }
    
    printf("Loading M68K-OS (%ld bytes)...\n", file_size);
    
    // Load at address 0x0000 (contains vectors + code)
    m68k_load_program(buffer, file_size, 0x0000);
    free(buffer);
    
    printf("OS loaded at 0x00000000\n");
    printf("Resetting CPU to start execution...\n");
    
    // Reset CPU (loads PC and SP from vectors)
    m68k_reset();
    
    printf("Starting M68K-OS in background task...\n");
    printf("OS has full control of LCD and keyboard\n");
    printf("(UART still available for debugging)\n");
    printf("\n");
    
    // Create M68K OS execution task
    BaseType_t ret = xTaskCreate(
        m68k_os_task,
        "m68k_os",
        8192,           // 8KB stack
        NULL,
        5,              // Normal priority
        NULL
    );
    
    if (ret != pdPASS) {
        printf("Failed to create M68K OS task\n");
        return false;
    }
    
    // OS is running in background, don't show BIOS prompt
    return true;
}

/* ====== Console Commands ====== */

static int cmd_m68k_init(int argc, char **argv) {
    esp_err_t ret = m68k_init();
    if (ret == ESP_OK) {
        printf("68000 CPU initialized successfully\n");
        printf("  CPU: Motorola 68000 @ 60 MHz\n");
        printf("  RAM: 16 MB\n");
    } else {
        printf("Failed to initialize CPU: %s\n", esp_err_to_name(ret));
    }
    return 0;
}

static int cmd_m68k_reset(int argc, char **argv) {
    m68k_reset();
    printf("CPU reset\n");
    return 0;
}

static int cmd_m68k_load_example(int argc, char **argv) {
    m68k_load_program(example_program, sizeof(example_program), 0x0000);
    printf("Loaded example program (%d bytes)\n", sizeof(example_program));
    printf("Program description:\n");
    printf("  - Initializes D0 to 0\n");
    printf("  - Increments D0 in a loop\n");
    printf("  - Stops when D0 reaches 10\n");
    printf("\nUse 'reset' to set PC and SP from vectors\n");
    printf("Use 'run <N>' to execute N instructions\n");
    return 0;
}

static int cmd_m68k_run(int argc, char **argv) {
    if (argc < 2) {
        printf("Usage: run <instructions>\n");
        return 1;
    }
    
    uint32_t count = atoi(argv[1]);
    if (count == 0) count = 1;
    
    printf("Running %lu instructions...\n", (unsigned long)count);
    m68k_run(count);
    printf("Done\n");
    
    return 0;
}

static int cmd_m68k_step(int argc, char **argv) {
    printf("Executing 1 instruction...\n");
    m68k_step();
    return 0;
}

static int cmd_m68k_state(int argc, char **argv) {
    char buffer[1024];
    m68k_get_state(buffer, sizeof(buffer));
    printf("%s\n", buffer);
    return 0;
}

static int cmd_m68k_dump(int argc, char **argv) {
    uint32_t addr = 0;
    uint32_t length = 256;  // Default 256 bytes
    
    if (argc >= 2) {
        addr = strtol(argv[1], NULL, 16);
    }
    if (argc >= 3) {
        length = strtol(argv[2], NULL, 0);  // Accepts both decimal and hex (0x prefix)
    }
    
    if (argc == 1) {
        console_printf("Usage: dump [address_hex] [length]\n");
        console_printf("  address_hex: Starting address in hex (default: 0)\n");
        console_printf("  length: Number of bytes to dump (default: 256)\n");
        console_printf("Examples:\n");
        console_printf("  dump              - Dump 256 bytes from 0x00000000\n");
        console_printf("  dump 1000         - Dump 256 bytes from 0x00001000\n");
        console_printf("  dump 1000 64      - Dump 64 bytes from 0x00001000\n");
        console_printf("  dump 00EFFE90 128 - Dump 128 bytes from stack pointer area\n");
        return 0;
    }
    
    m68k_dump_memory(addr, length);
    return 0;
}

static int cmd_m68k_crash(int argc, char **argv) {
    m68k_show_crash_context();
    return 0;
}

static int cmd_m68k_poke(int argc, char **argv) {
    if (argc < 3) {
        printf("Usage: poke <addr_hex> <value_hex>\n");
        printf("Example: poke 1000 AA\n");
        return 1;
    }
    
    // This would require exposing write functions from emulator
    printf("Poke not yet implemented\n");
    return 0;
}

static int cmd_sd_mount(int argc, char **argv) {
    esp_err_t ret = mount_sd_card();
    if (ret == ESP_OK) {
        printf("SD card mounted at /sdcard\n");
    } else {
        printf("Failed to mount SD card: %s\n", esp_err_to_name(ret));
    }
    return 0;
}

static int cmd_sd_unmount(int argc, char **argv) {
    unmount_sd_card();
    printf("SD card unmounted\n");
    return 0;
}

/* ====== Linux-style File System Commands ====== */

// ls - List directory contents
static int cmd_ls(int argc, char **argv) {
    if (!sd_mounted) {
        console_printf("SD card not mounted. Use 'sdmount' first.\n");
        return 1;
    }

    char *path = NULL;
    bool long_format = false;
    bool show_all = false;
    
    // Parse arguments
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-l") == 0) {
            long_format = true;
        } else if (strcmp(argv[i], "-a") == 0) {
            show_all = true;
        } else if (strcmp(argv[i], "-la") == 0 || strcmp(argv[i], "-al") == 0) {
            long_format = true;
            show_all = true;
        } else if (argv[i][0] != '-') {
            path = resolve_path(argv[i]);
        }
    }
    
    if (path == NULL) {
        path = resolve_path(NULL);  // Use current directory
    }

    DIR *dir = opendir(path);
    if (dir == NULL) {
        console_printf("ls: cannot access '%s': No such directory\n", path);
        free(path);
        return 1;
    }

    struct dirent *entry;
    struct stat st;
    char filepath[512];
    int count = 0;

    if (long_format) {
        console_printf("total in %s:\n", path);
    }

    while ((entry = readdir(dir)) != NULL) {
        // Skip hidden files unless -a specified
        if (!show_all && entry->d_name[0] == '.') {
            continue;
        }
        
        snprintf(filepath, sizeof(filepath), "%s/%s", path, entry->d_name);
        
        if (long_format) {
            if (stat(filepath, &st) == 0) {
                char type = S_ISDIR(st.st_mode) ? 'd' : '-';
                console_printf("%crw-r--r-- %8ld  %s\n", type, st.st_size, entry->d_name);
            } else {
                console_printf("?????????? %8s  %s\n", "?", entry->d_name);
            }
        } else {
            if (stat(filepath, &st) == 0 && S_ISDIR(st.st_mode)) {
                console_printf("%s/  ", entry->d_name);
            } else {
                console_printf("%s  ", entry->d_name);
            }
            count++;
            if (count % 4 == 0) console_printf("\n");
        }
    }
    
    if (!long_format && count % 4 != 0) {
        console_printf("\n");
    }

    closedir(dir);
    free(path);
    return 0;
}

// cd - Change directory
static int cmd_cd(int argc, char **argv) {
    if (!sd_mounted) {
        console_printf("SD card not mounted. Use 'sdmount' first.\n");
        return 1;
    }

    const char *target = (argc >= 2) ? argv[1] : "/sdcard";
    char *new_path = resolve_path(target);
    
    if (new_path == NULL) {
        console_printf("cd: memory allocation failed\n");
        return 1;
    }
    
    // Verify the directory exists
    struct stat st;
    if (stat(new_path, &st) != 0 || !S_ISDIR(st.st_mode)) {
        console_printf("cd: %s: No such directory\n", target);
        free(new_path);
        return 1;
    }
    
    // Update current working directory
    strncpy(current_working_dir, new_path, sizeof(current_working_dir) - 1);
    current_working_dir[sizeof(current_working_dir) - 1] = '\0';
    
    free(new_path);
    return 0;
}

// pwd - Print working directory
static int cmd_pwd(int argc, char **argv) {
    (void)argc; (void)argv;
    console_printf("%s\n", current_working_dir);
    return 0;
}

// cat - Display file contents
static int cmd_cat(int argc, char **argv) {
    if (!sd_mounted) {
        console_printf("SD card not mounted. Use 'sdmount' first.\n");
        return 1;
    }

    if (argc < 2) {
        console_printf("Usage: cat <filename>\n");
        return 1;
    }

    char *filepath = resolve_path(argv[1]);
    if (filepath == NULL) {
        console_printf("cat: memory allocation failed\n");
        return 1;
    }

    FILE *f = fopen(filepath, "r");
    if (f == NULL) {
        console_printf("cat: %s: No such file\n", argv[1]);
        free(filepath);
        return 1;
    }

    char line[256];
    while (fgets(line, sizeof(line), f) != NULL) {
        console_printf("%s", line);
    }

    fclose(f);
    free(filepath);
    return 0;
}

// mkdir - Create directory
static int cmd_mkdir(int argc, char **argv) {
    if (!sd_mounted) {
        console_printf("SD card not mounted. Use 'sdmount' first.\n");
        return 1;
    }

    if (argc < 2) {
        console_printf("Usage: mkdir <dirname>\n");
        return 1;
    }

    char *dirpath = resolve_path(argv[1]);
    if (dirpath == NULL) {
        console_printf("mkdir: memory allocation failed\n");
        return 1;
    }

    if (mkdir(dirpath, 0755) != 0) {
        console_printf("mkdir: cannot create directory '%s'\n", argv[1]);
        free(dirpath);
        return 1;
    }

    free(dirpath);
    return 0;
}

// rm - Remove file
static int cmd_rm(int argc, char **argv) {
    if (!sd_mounted) {
        console_printf("SD card not mounted. Use 'sdmount' first.\n");
        return 1;
    }

    if (argc < 2) {
        console_printf("Usage: rm <filename>\n");
        return 1;
    }

    char *filepath = resolve_path(argv[1]);
    if (filepath == NULL) {
        console_printf("rm: memory allocation failed\n");
        return 1;
    }

    if (unlink(filepath) != 0) {
        console_printf("rm: cannot remove '%s'\n", argv[1]);
        free(filepath);
        return 1;
    }

    free(filepath);
    return 0;
}

// rmdir - Remove directory
static int cmd_rmdir(int argc, char **argv) {
    if (!sd_mounted) {
        console_printf("SD card not mounted. Use 'sdmount' first.\n");
        return 1;
    }

    if (argc < 2) {
        console_printf("Usage: rmdir <dirname>\n");
        return 1;
    }

    char *dirpath = resolve_path(argv[1]);
    if (dirpath == NULL) {
        console_printf("rmdir: memory allocation failed\n");
        return 1;
    }

    if (rmdir(dirpath) != 0) {
        console_printf("rmdir: failed to remove '%s'\n", argv[1]);
        free(dirpath);
        return 1;
    }

    free(dirpath);
    return 0;
}

// cp - Copy file
static int cmd_cp(int argc, char **argv) {
    if (!sd_mounted) {
        console_printf("SD card not mounted. Use 'sdmount' first.\n");
        return 1;
    }

    if (argc < 3) {
        console_printf("Usage: cp <source> <dest>\n");
        return 1;
    }

    char *src = resolve_path(argv[1]);
    char *dst = resolve_path(argv[2]);
    
    if (src == NULL || dst == NULL) {
        console_printf("cp: memory allocation failed\n");
        free(src);
        free(dst);
        return 1;
    }

    FILE *fsrc = fopen(src, "rb");
    if (fsrc == NULL) {
        console_printf("cp: cannot open '%s'\n", argv[1]);
        free(src);
        free(dst);
        return 1;
    }

    FILE *fdst = fopen(dst, "wb");
    if (fdst == NULL) {
        console_printf("cp: cannot create '%s'\n", argv[2]);
        fclose(fsrc);
        free(src);
        free(dst);
        return 1;
    }

    char buffer[512];
    size_t bytes;
    while ((bytes = fread(buffer, 1, sizeof(buffer), fsrc)) > 0) {
        fwrite(buffer, 1, bytes, fdst);
    }

    fclose(fsrc);
    fclose(fdst);
    free(src);
    free(dst);
    
    console_printf("File copied.\n");
    return 0;
}

// mv - Move/rename file
static int cmd_mv(int argc, char **argv) {
    if (!sd_mounted) {
        console_printf("SD card not mounted. Use 'sdmount' first.\n");
        return 1;
    }

    if (argc < 3) {
        console_printf("Usage: mv <source> <dest>\n");
        return 1;
    }

    char *src = resolve_path(argv[1]);
    char *dst = resolve_path(argv[2]);
    
    if (src == NULL || dst == NULL) {
        console_printf("mv: memory allocation failed\n");
        free(src);
        free(dst);
        return 1;
    }

    if (rename(src, dst) != 0) {
        console_printf("mv: cannot move '%s' to '%s'\n", argv[1], argv[2]);
        free(src);
        free(dst);
        return 1;
    }

    free(src);
    free(dst);
    return 0;
}

// touch - Create empty file or update timestamp
static int cmd_touch(int argc, char **argv) {
    if (!sd_mounted) {
        console_printf("SD card not mounted. Use 'sdmount' first.\n");
        return 1;
    }

    if (argc < 2) {
        console_printf("Usage: touch <filename>\n");
        return 1;
    }

    char *filepath = resolve_path(argv[1]);
    if (filepath == NULL) {
        console_printf("touch: memory allocation failed\n");
        return 1;
    }

    FILE *f = fopen(filepath, "a");
    if (f == NULL) {
        console_printf("touch: cannot touch '%s'\n", argv[1]);
        free(filepath);
        return 1;
    }

    fclose(f);
    free(filepath);
    return 0;
}

// echo - Print text (with > redirect support)
static int cmd_echo(int argc, char **argv) {
    if (argc < 2) {
        console_printf("\n");
        return 0;
    }

    // Check for redirection
    int redirect_idx = -1;
    bool append = false;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], ">") == 0) {
            redirect_idx = i;
            break;
        } else if (strcmp(argv[i], ">>") == 0) {
            redirect_idx = i;
            append = true;
            break;
        }
    }

    if (redirect_idx > 0 && redirect_idx < argc - 1) {
        // Redirect to file
        char *filepath = resolve_path(argv[redirect_idx + 1]);
        if (filepath == NULL) {
            console_printf("echo: memory allocation failed\n");
            return 1;
        }

        FILE *f = fopen(filepath, append ? "a" : "w");
        if (f == NULL) {
            console_printf("echo: cannot open '%s'\n", argv[redirect_idx + 1]);
            free(filepath);
            return 1;
        }

        for (int i = 1; i < redirect_idx; i++) {
            fprintf(f, "%s", argv[i]);
            if (i < redirect_idx - 1) fprintf(f, " ");
        }
        fprintf(f, "\n");
        fclose(f);
        free(filepath);
    } else {
        // Print to console
        for (int i = 1; i < argc; i++) {
            console_printf("%s", argv[i]);
            if (i < argc - 1) console_printf(" ");
        }
        console_printf("\n");
    }

    return 0;
}

// clear - Clear screen
static int cmd_clear(int argc, char **argv) {
    (void)argc; (void)argv;
    
    // Clear UART terminal (ANSI escape codes)
    printf("\033[2J\033[H");
    
    // Clear LCD if active
    if (lcd_console_active && lcd_console_is_initialized()) {
        lcd_console_clear();
    }
    
    return 0;
}

// history - Show command history
static int cmd_show_history(int argc, char **argv) {
    (void)argc; (void)argv;
    
    for (int i = 0; i < cmd_history_count; i++) {
        if (cmd_history[i]) {
            console_printf("%4d  %s\n", i + 1, cmd_history[i]);
        }
    }
    return 0;
}

// df - Show disk free space
static int cmd_df(int argc, char **argv) {
    (void)argc; (void)argv;
    
    if (!sd_mounted || !sd_card) {
        console_printf("SD card not mounted.\n");
        return 1;
    }
    
    uint64_t total = (uint64_t)sd_card->csd.capacity * sd_card->csd.sector_size;
    
    console_printf("Filesystem      Size      Used      Avail  Use%%  Mounted on\n");
    console_printf("/dev/mmcblk0    %4lluMB    ---       ---    ---   /sdcard\n", total / (1024*1024));
    
    return 0;
}

// head - Show first lines of a file
static int cmd_head(int argc, char **argv) {
    if (!sd_mounted) {
        console_printf("SD card not mounted.\n");
        return 1;
    }
    
    if (argc < 2) {
        console_printf("Usage: head [-n lines] <filename>\n");
        return 1;
    }
    
    int lines = 10;
    const char *filename = argv[1];
    
    if (argc >= 4 && strcmp(argv[1], "-n") == 0) {
        lines = atoi(argv[2]);
        filename = argv[3];
    }
    
    char *filepath = resolve_path(filename);
    if (filepath == NULL) return 1;
    
    FILE *f = fopen(filepath, "r");
    if (f == NULL) {
        console_printf("head: cannot open '%s'\n", filename);
        free(filepath);
        return 1;
    }
    
    char line[256];
    int count = 0;
    while (fgets(line, sizeof(line), f) != NULL && count < lines) {
        console_printf("%s", line);
        count++;
    }
    
    fclose(f);
    free(filepath);
    return 0;
}

// tail - Show last lines of a file
static int cmd_tail(int argc, char **argv) {
    if (!sd_mounted) {
        console_printf("SD card not mounted.\n");
        return 1;
    }
    
    if (argc < 2) {
        console_printf("Usage: tail [-n lines] <filename>\n");
        return 1;
    }
    
    int lines = 10;
    const char *filename = argv[1];
    
    if (argc >= 4 && strcmp(argv[1], "-n") == 0) {
        lines = atoi(argv[2]);
        filename = argv[3];
    }
    
    char *filepath = resolve_path(filename);
    if (filepath == NULL) return 1;
    
    // Read all lines into circular buffer
    FILE *f = fopen(filepath, "r");
    if (f == NULL) {
        console_printf("tail: cannot open '%s'\n", filename);
        free(filepath);
        return 1;
    }
    
    char **line_buf = malloc(lines * sizeof(char*));
    if (line_buf == NULL) {
        fclose(f);
        free(filepath);
        return 1;
    }
    
    for (int i = 0; i < lines; i++) line_buf[i] = NULL;
    
    char line[256];
    int idx = 0;
    int total = 0;
    while (fgets(line, sizeof(line), f) != NULL) {
        free(line_buf[idx]);
        line_buf[idx] = strdup(line);
        idx = (idx + 1) % lines;
        total++;
    }
    fclose(f);
    
    // Print last N lines
    int start = (total < lines) ? 0 : idx;
    int count = (total < lines) ? total : lines;
    for (int i = 0; i < count; i++) {
        int j = (start + i) % lines;
        if (line_buf[j]) {
            console_printf("%s", line_buf[j]);
            free(line_buf[j]);
        }
    }
    
    free(line_buf);
    free(filepath);
    return 0;
}

static int cmd_help(int argc, char **argv) {
    console_printf("\n=== ESP32-P4 Dual CPU Emulator - Available Commands ===\n\n");
    
    console_printf("📁 File System Commands:\n");
    console_printf("  ls [dir]           - List directory contents\n");
    console_printf("  cd <dir>           - Change directory\n"); 
    console_printf("  pwd                - Show current directory\n");
    console_printf("  cat <file>         - Display file contents\n");
    console_printf("  mkdir <dir>        - Create directory\n");
    console_printf("  rm <file>          - Remove file\n");
    console_printf("  rmdir <dir>        - Remove directory\n");
    console_printf("  cp <src> <dst>     - Copy file\n");
    console_printf("  mv <src> <dst>     - Move/rename file\n");
    console_printf("  touch <file>       - Create empty file\n");
    console_printf("  echo <text>        - Print text\n");
    console_printf("  head [-n N] <file> - Show first N lines\n");
    console_printf("  tail [-n N] <file> - Show last N lines\n");
    console_printf("  df                 - Show disk space\n\n");
    
    console_printf("💾 Storage Commands:\n");
    console_printf("  sdmount            - Mount SD card\n");
    console_printf("  sdunmount          - Unmount SD card\n");
    console_printf("  loadsd <file>      - Load binary from SD card\n\n");
    
    console_printf("🖥️  M68K Emulator Commands:\n");
    console_printf("  init               - Initialize M68K CPU\n");
    console_printf("  reset / resetm68k  - Reset M68K CPU\n");
    console_printf("  load <file>        - Load M68K binary\n");
    console_printf("  run [cycles]       - Run M68K emulation\n");
    console_printf("  runm68k [cycles]   - Run M68K emulation\n");
    console_printf("  step / stepm68k    - Single step M68K\n");
    console_printf("  state / statem68k  - Show M68K CPU state\n");
    console_printf("  dump <addr> <len>  - Dump M68K memory\n");
    console_printf("  switch m68k        - Set M68K as active CPU\n\n");
    
    console_printf("🖥️  i8086 Emulator Commands:\n");
    console_printf("  init86             - Initialize i8086 CPU\n");
    console_printf("  reset86            - Reset i8086 CPU\n");
    console_printf("  run86 [cycles]     - Run i8086 emulation\n");
    console_printf("  step86             - Single step i8086\n");
    console_printf("  state86            - Show i8086 CPU state\n");
    console_printf("  switch 86          - Set i8086 as active CPU\n\n");
    
    console_printf("💽 Disk Image Commands (i8086):\n");
    console_printf("  mkdisk <file> <size> - Create disk image\n");
    console_printf("  mount <drive> <file> - Mount disk to drive (A:, B:, C:, D:)\n");
    console_printf("  unmount <drive>    - Unmount drive\n");
    console_printf("  disks              - Show mounted disks\n\n");
    
    console_printf("📺 Display Commands:\n");
    console_printf("  lcd_init           - Initialize LCD display\n");
    console_printf("  lcd_test           - Test LCD display\n");
    console_printf("  lcdpins            - Show LCD GPIO pin configuration\n");
    console_printf("  clear              - Clear screen\n\n");
    
    console_printf("⌨️  Input Device Commands:\n");
    console_printf("  ps2test            - Test PS/2 keyboard (30 seconds)\n\n");
    
    console_printf("🔵 Bluetooth Commands:\n");
    console_printf("  btinit             - Initialize Bluetooth controller\n");
    console_printf("  btscan [sec]       - Scan for Bluetooth devices\n");
    console_printf("  btconnect <MAC>    - Connect to Bluetooth device\n");
    console_printf("  btdisconnect <MAC> - Disconnect Bluetooth device\n");
    console_printf("  btlist             - List Bluetooth devices\n\n");
    
    console_printf("📶 WiFi Commands:\n");
    console_printf("  scan               - Scan for WiFi networks\n");
    console_printf("  connect <ssid> <pw> - Connect to WiFi network\n");
    console_printf("  status             - Show WiFi status\n");
    console_printf("  disconnect         - Disconnect from WiFi\n\n");
    
    console_printf("🔧 System Commands:\n");
    console_printf("  history            - Show command history\n");
    console_printf("  appinfo            - Show application info\n");
    console_printf("  appstop            - Stop current application\n");
    console_printf("  help               - Show this help message\n\n");
    
    console_printf("💡 Quick Start:\n");
    console_printf("1. Mount SD card: sdmount\n");
    console_printf("2. Initialize display: lcd_init\n");
    console_printf("3. For M68K: init → loadsd program.bin → run\n");
    console_printf("4. For i8086: init86 → mkdisk boot.img 1.44M → mount A boot.img → run86\n");
    console_printf("5. For Bluetooth: btinit → btscan → btconnect <MAC>\n");
    console_printf("6. For WiFi: scan → connect <ssid> <password> → status\n\n");
    
    console_printf("📖 For detailed guides, see:\n");
    console_printf("   QUICK_START.md, I8086_QUICK_START.md, BLUETOOTH_HID_GUIDE.md\n\n");
    
    return 0;
}

// Register all Linux-style filesystem commands
static void register_linux_commands(void) {
    const esp_console_cmd_t ls_cmd = {
        .command = "ls",
        .help = "List directory contents",
        .hint = "[-l] [-a] [path]",
        .func = &cmd_ls,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&ls_cmd));
    
    const esp_console_cmd_t cd_cmd = {
        .command = "cd",
        .help = "Change directory",
        .hint = "[path]",
        .func = &cmd_cd,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cd_cmd));
    
    const esp_console_cmd_t pwd_cmd = {
        .command = "pwd",
        .help = "Print working directory",
        .hint = NULL,
        .func = &cmd_pwd,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&pwd_cmd));
    
    const esp_console_cmd_t cat_cmd = {
        .command = "cat",
        .help = "Display file contents",
        .hint = "<filename>",
        .func = &cmd_cat,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cat_cmd));
    
    const esp_console_cmd_t mkdir_cmd = {
        .command = "mkdir",
        .help = "Create directory",
        .hint = "<dirname>",
        .func = &cmd_mkdir,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&mkdir_cmd));
    
    const esp_console_cmd_t rm_cmd = {
        .command = "rm",
        .help = "Remove file",
        .hint = "<filename>",
        .func = &cmd_rm,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&rm_cmd));
    
    const esp_console_cmd_t rmdir_cmd = {
        .command = "rmdir",
        .help = "Remove directory",
        .hint = "<dirname>",
        .func = &cmd_rmdir,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&rmdir_cmd));
    
    const esp_console_cmd_t cp_cmd = {
        .command = "cp",
        .help = "Copy file",
        .hint = "<source> <dest>",
        .func = &cmd_cp,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cp_cmd));
    
    const esp_console_cmd_t mv_cmd = {
        .command = "mv",
        .help = "Move/rename file",
        .hint = "<source> <dest>",
        .func = &cmd_mv,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&mv_cmd));
    
    const esp_console_cmd_t touch_cmd = {
        .command = "touch",
        .help = "Create empty file",
        .hint = "<filename>",
        .func = &cmd_touch,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&touch_cmd));
    
    const esp_console_cmd_t echo_cmd = {
        .command = "echo",
        .help = "Print text or write to file",
        .hint = "<text> [> file]",
        .func = &cmd_echo,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&echo_cmd));
    
    const esp_console_cmd_t clear_cmd = {
        .command = "clear",
        .help = "Clear screen",
        .hint = NULL,
        .func = &cmd_clear,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&clear_cmd));
    
    const esp_console_cmd_t history_cmd = {
        .command = "history",
        .help = "Show command history",
        .hint = NULL,
        .func = &cmd_show_history,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&history_cmd));
    
    const esp_console_cmd_t df_cmd = {
        .command = "df",
        .help = "Show disk space",
        .hint = NULL,
        .func = &cmd_df,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&df_cmd));
    
    const esp_console_cmd_t head_cmd = {
        .command = "head",
        .help = "Show first lines of file",
        .hint = "[-n lines] <file>",
        .func = &cmd_head,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&head_cmd));
    
    const esp_console_cmd_t tail_cmd = {
        .command = "tail",
        .help = "Show last lines of file",
        .hint = "[-n lines] <file>",
        .func = &cmd_tail,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&tail_cmd));
    
    const esp_console_cmd_t help_cmd = {
        .command = "help",
        .help = "Show all available commands",
        .hint = NULL,
        .func = &cmd_help,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&help_cmd));
}

static int cmd_load_from_sd(int argc, char **argv) {
    if (!sd_mounted) {
        printf("SD card not mounted. Use 'sdmount' first.\n");
        return 1;
    }
    if (argc < 2) {
        printf("Usage: loadsd <filename> [address]\n");
        printf("Example: loadsd program.bin\n");
        printf("Example: loadsd program.bin 1000\n");
        return 1;
    }

    char filepath[300];
    if (argv[1][0] == '/') {
        snprintf(filepath, sizeof(filepath), "%s", argv[1]);
    } else {
        snprintf(filepath, sizeof(filepath), "/sdcard/%s", argv[1]);
    }

    uint32_t load_addr = 0x0000;
    if (argc >= 3) {
        load_addr = strtol(argv[2], NULL, 16);
    }

    FILE *file = fopen(filepath, "rb");
    if (file == NULL) {
        printf("Failed to open file: %s\n", filepath);
        return 1;
    }

    // Get file size
    fseek(file, 0, SEEK_END);
    long file_size = ftell(file);
    fseek(file, 0, SEEK_SET);

    if (file_size <= 0) {
        printf("Invalid file size: %ld\n", file_size);
        fclose(file);
        return 1;
    }

    if (file_size > 16 * 1024 * 1024) {
        printf("File too large: %ld bytes (max 16MB)\n", file_size);
        fclose(file);
        return 1;
    }

    // Allocate buffer and read file
    uint8_t *buffer = malloc(file_size);
    if (buffer == NULL) {
        printf("Failed to allocate %ld bytes\n", file_size);
        fclose(file);
        return 1;
    }

    size_t bytes_read = fread(buffer, 1, file_size, file);
    fclose(file);

    if (bytes_read != file_size) {
        printf("Failed to read file: got %d bytes, expected %ld\n", bytes_read, file_size);
        free(buffer);
        return 1;
    }

    // Load into emulator
    printf("Loading %s...\n", argv[1]);
    printf("  File size: %ld bytes\n", file_size);
    printf("  Load address: 0x%08lX\n", (unsigned long)load_addr);

    m68k_load_program(buffer, file_size, load_addr);
    free(buffer);

    printf("Program loaded successfully!\n");
    printf("Use 'reset' to start execution\n");

    return 0;
}

// Background task for loadrun - runs M68K continuously until halted
static void loadrun_task(void *arg) {
    printf("loadrun: M68K execution started\n");
    while (true) {
        m68k_run(1000);
        if (m68k_is_halted()) {
            printf("loadrun: M68K halted\n");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    printf("loadrun: task ended\n");
    vTaskDelete(NULL);
}

// Helper: detect M68K vector table in first 8 bytes (big-endian SP at 0, PC at 4)
static bool detect_vector_table(const uint8_t *data, uint32_t size, uint32_t *out_sp, uint32_t *out_pc) {
    if (size < 8) return false;

    // Read big-endian SP (bytes 0-3) and PC (bytes 4-7)
    uint32_t sp = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
                  ((uint32_t)data[2] << 8)  | data[3];
    uint32_t pc = ((uint32_t)data[4] << 24) | ((uint32_t)data[5] << 16) |
                  ((uint32_t)data[6] << 8)  | data[7];

    // Heuristic: SP should be even, non-zero, within 16MB RAM
    // PC should be even, non-zero, within 16MB, and >= 8 (past vectors)
    if (sp == 0 || pc == 0) return false;
    if (sp & 1) return false;       // SP must be even
    if (pc & 1) return false;       // PC must be even
    if (sp > 0x01000000) return false; // SP within 16MB
    if (pc > 0x01000000) return false; // PC within 16MB
    if (pc < 8) return false;         // PC past vector table itself

    *out_sp = sp;
    *out_pc = pc;
    return true;
}

static int cmd_loadrun(int argc, char **argv) {
    if (!sd_mounted) {
        printf("SD card not mounted. Use 'sdmount' first.\n");
        return 1;
    }
    if (argc < 2) {
        printf("Usage: loadrun <filename> [addr]\n");
        printf("  Loads a M68K binary from SD card and immediately runs it.\n");
        printf("  Auto-detects vector table (SP+PC in first 8 bytes).\n");
        printf("  If no vector table: loads at [addr] (default 0x1000) and runs.\n");
        printf("Examples:\n");
        printf("  loadrun 3d_demo.bin       - vector table program\n");
        printf("  loadrun program.bin 1000  - raw code at 0x1000\n");
        return 1;
    }

    char filepath[300];
    if (argv[1][0] == '/') {
        snprintf(filepath, sizeof(filepath), "%s", argv[1]);
    } else {
        snprintf(filepath, sizeof(filepath), "/sdcard/%s", argv[1]);
    }

    // Default load address for non-vector-table programs
    uint32_t raw_load_addr = 0x1000;
    if (argc >= 3) {
        raw_load_addr = strtol(argv[2], NULL, 16);
    }

    FILE *file = fopen(filepath, "rb");
    if (file == NULL) {
        printf("Failed to open file: %s\n", filepath);
        return 1;
    }

    fseek(file, 0, SEEK_END);
    long file_size = ftell(file);
    fseek(file, 0, SEEK_SET);

    if (file_size <= 0 || file_size > 16 * 1024 * 1024) {
        printf("Invalid file size: %ld bytes\n", file_size);
        fclose(file);
        return 1;
    }

    uint8_t *buffer = malloc(file_size);
    if (buffer == NULL) {
        printf("Failed to allocate %ld bytes\n", file_size);
        fclose(file);
        return 1;
    }

    size_t bytes_read = fread(buffer, 1, file_size, file);
    fclose(file);

    if ((long)bytes_read != file_size) {
        printf("Failed to read file: got %d bytes, expected %ld\n", (int)bytes_read, file_size);
        free(buffer);
        return 1;
    }

    printf("Loaded %s (%ld bytes)\n", argv[1], file_size);

    // Detect vector table
    uint32_t init_sp = 0, init_pc = 0;
    bool has_vectors = detect_vector_table(buffer, (uint32_t)file_size, &init_sp, &init_pc);

    if (has_vectors) {
        printf("  Vector table detected: SP=0x%08lX PC=0x%08lX\n",
               (unsigned long)init_sp, (unsigned long)init_pc);
        // Load at address 0 (vectors + code together)
        m68k_load_program(buffer, file_size, 0x0000);
        free(buffer);

        // Reset CPU first (clears all state), then override SP/PC
        m68k_reset();
        // m68k_reset already reads vectors from address 0, which now contain our program's vectors.
        // But let's be explicit to ensure correctness:
        m68k_set_reg(M68K_REG_A7, init_sp);
        m68k_set_reg(M68K_REG_SP, init_sp);
        m68k_set_reg(M68K_REG_PC, init_pc);
        printf("  CPU reset: SP=0x%08lX PC=0x%08lX\n",
               (unsigned long)init_sp, (unsigned long)init_pc);
    } else {
        printf("  No vector table - loading at 0x%08lX\n", (unsigned long)raw_load_addr);
        m68k_load_program(buffer, file_size, raw_load_addr);
        free(buffer);

        // Don't do full reset (would read address 0 as SP/PC which may be garbage).
        // Just set PC to load address and SP to a reasonable default.
        m68k_set_reg(M68K_REG_A7, 0x00C00000);
        m68k_set_reg(M68K_REG_SP, 0x00C00000);
        m68k_set_reg(M68K_REG_PC, raw_load_addr);
        printf("  CPU: SP=0x00C00000 PC=0x%08lX\n", (unsigned long)raw_load_addr);
    }

    // Start execution in background task
    printf("Starting execution...\n");
    BaseType_t ret = xTaskCreate(
        loadrun_task,
        "loadrun",
        8192,
        NULL,
        5,
        NULL
    );

    if (ret != pdPASS) {
        printf("Failed to create execution task\n");
        return 1;
    }

    return 0;
}

static void register_m68k_commands(void) {
    const esp_console_cmd_t init_cmd = {
        .command = "init",
        .help = "Initialize the 68000 CPU",
        .hint = NULL,
        .func = &cmd_m68k_init,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&init_cmd));
    
    const esp_console_cmd_t reset_cmd = {
        .command = "reset",
        .help = "Reset the CPU and load vectors",
        .hint = NULL,
        .func = &cmd_m68k_reset,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&reset_cmd));
    
    const esp_console_cmd_t load_cmd = {
        .command = "load",
        .help = "Load example program into memory",
        .hint = NULL,
        .func = &cmd_m68k_load_example,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&load_cmd));
    
    const esp_console_cmd_t run_cmd = {
        .command = "run",
        .help = "Run N instructions",
        .hint = "<count>",
        .func = &cmd_m68k_run,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&run_cmd));
    
    const esp_console_cmd_t step_cmd = {
        .command = "step",
        .help = "Execute single instruction",
        .hint = NULL,
        .func = &cmd_m68k_step,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&step_cmd));
    
    const esp_console_cmd_t state_cmd = {
        .command = "state",
        .help = "Display CPU state (registers, flags)",
        .hint = NULL,
        .func = &cmd_m68k_state,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&state_cmd));
    
    const esp_console_cmd_t dump_cmd = {
        .command = "dump",
        .help = "Dump memory contents",
        .hint = "[addr] [length]",
        .func = &cmd_m68k_dump,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&dump_cmd));
    
    const esp_console_cmd_t crash_cmd = {
        .command = "crash",
        .help = "Show crash context and suggest memory dumps",
        .hint = NULL,
        .func = &cmd_m68k_crash,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&crash_cmd));
    
    const esp_console_cmd_t poke_cmd = {
        .command = "poke",
        .help = "Write byte to memory",
        .hint = "<addr> <value>",
        .func = &cmd_m68k_poke,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&poke_cmd));
    
    const esp_console_cmd_t sdmount_cmd = {
        .command = "sdmount",
        .help = "Mount SD card",
        .hint = NULL,
        .func = &cmd_sd_mount,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&sdmount_cmd));
    
    const esp_console_cmd_t sdunmount_cmd = {
        .command = "sdunmount",
        .help = "Unmount SD card",
        .hint = NULL,
        .func = &cmd_sd_unmount,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&sdunmount_cmd));
    
    // Note: 'ls' command is registered in register_linux_commands()
    
    const esp_console_cmd_t loadsd_cmd = {
        .command = "loadsd",
        .help = "Load 68000 program from SD card",
        .hint = "<filename> [addr]",
        .func = &cmd_load_from_sd,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&loadsd_cmd));

    const esp_console_cmd_t loadrun_cmd = {
        .command = "loadrun",
        .help = "Load & run M68K program (auto-detects vector table)",
        .hint = "<filename> [addr]",
        .func = &cmd_loadrun,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&loadrun_cmd));
}

static void initialize_console(void) {
    fflush(stdout);
    fsync(fileno(stdout));
    setvbuf(stdin, NULL, _IONBF, 0);

    uart_vfs_dev_port_set_rx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CR);
    uart_vfs_dev_port_set_tx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CRLF);

    const uart_config_t uart_config = {
        .baud_rate = CONFIG_ESP_CONSOLE_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, 256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(CONFIG_ESP_CONSOLE_UART_NUM, &uart_config));
    uart_vfs_dev_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);

    esp_console_config_t console_config = {
        .max_cmdline_length = 256,
        .max_cmdline_args = 8,
#if CONFIG_LOG_COLORS
        .hint_color = atoi(LOG_COLOR_CYAN)
#endif
    };
    ESP_ERROR_CHECK(esp_console_init(&console_config));

    linenoiseSetMultiLine(1);
    linenoiseSetCompletionCallback(&esp_console_get_completion);
    linenoiseSetHintsCallback((linenoiseHintsCallback*) &esp_console_get_hint);
    linenoiseHistorySetMaxLen(MAX_HISTORY_LINES);
    linenoiseSetMaxLineLen(console_config.max_cmdline_length);
    linenoiseAllowEmpty(false);
}

/* ====== WiFi Commands ====== */
// WiFi enabled for ESP32-P4-NANO with ESP32-C6 coprocessor via SDIO

static int cmd_wifi_scan(int argc, char **argv) {
    if (!wifi_initialized) {
        console_printf("✗ WiFi not initialized. Run 'help' for setup info.\n");
        return 1;
    }
    
    console_printf("Scanning for WiFi networks...\n");
    
    // Start WiFi scan
    esp_err_t ret = esp_wifi_scan_start(NULL, true);  // blocking scan
    if (ret != ESP_OK) {
        console_printf("✗ WiFi scan failed: %s\n", esp_err_to_name(ret));
        return 1;
    }
    
    // Get scan results
    uint16_t ap_count = 0;
    esp_wifi_scan_get_ap_num(&ap_count);
    
    if (ap_count == 0) {
        console_printf("No networks found.\n");
        return 0;
    }
    
    wifi_ap_record_t *ap_records = malloc(sizeof(wifi_ap_record_t) * ap_count);
    if (!ap_records) {
        console_printf("✗ Memory allocation failed\n");
        return 1;
    }
    
    ret = esp_wifi_scan_get_ap_records(&ap_count, ap_records);
    if (ret != ESP_OK) {
        console_printf("✗ Failed to get scan results: %s\n", esp_err_to_name(ret));
        free(ap_records);
        return 1;
    }
    
    console_printf("\nFound %d networks:\n", ap_count);
    console_printf("%-32s %-6s %-8s %-7s\n", "SSID", "RSSI", "Auth", "Channel");
    console_printf("%-32s %-6s %-8s %-7s\n", "--------------------------------", "------", "--------", "-------");
    
    for (int i = 0; i < ap_count; i++) {
        const char *auth_str = "Unknown";
        switch (ap_records[i].authmode) {
            case WIFI_AUTH_OPEN: auth_str = "Open"; break;
            case WIFI_AUTH_WEP: auth_str = "WEP"; break;
            case WIFI_AUTH_WPA_PSK: auth_str = "WPA"; break;
            case WIFI_AUTH_WPA2_PSK: auth_str = "WPA2"; break;
            case WIFI_AUTH_WPA3_PSK: auth_str = "WPA3"; break;
            default: auth_str = "Mixed"; break;
        }
        console_printf("%-32s %-6d %-8s %-7d\n", 
                     (char*)ap_records[i].ssid, ap_records[i].rssi, auth_str, ap_records[i].primary);
    }
    
    if (ap_count > 0) {
        console_printf("\nUse 'connect <ssid> <password>' to connect\n");
    } else {
        console_printf("✗ No networks found\n");
    }
    
    free(ap_records);
    return 0;
}

static int cmd_wifi_connect(int argc, char **argv) {
    if (argc < 3) {
        console_printf("Usage: connect <ssid> <password>\n");
        return 1;
    }
    
    const char *ssid = argv[1];
    const char *password = argv[2];
    
    console_printf("Connecting to '%s'...\n", ssid);
    
    // Configure WiFi station
    wifi_config_t wifi_config = {0};
    strncpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char*)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);
    
    // Set WiFi mode to station
    esp_err_t ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        console_printf("✗ Failed to set WiFi mode: %s\n", esp_err_to_name(ret));
        return 1;
    }
    
    ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (ret != ESP_OK) {
        console_printf("✗ Failed to set WiFi config: %s\n", esp_err_to_name(ret));
        return 1;
    }
    
    ret = esp_wifi_connect();
    if (ret != ESP_OK) {
        console_printf("✗ Connection request failed: %s\n", esp_err_to_name(ret));
        return 1;
    }
    
    console_printf("✓ Connection initiated - check status with 'wifistatus'\n");
    return 0;
}

static int cmd_wifi_status(int argc, char **argv) {
    wifi_ap_record_t ap_info;
    esp_err_t ret = esp_wifi_sta_get_ap_info(&ap_info);
    
    if (ret == ESP_OK) {
        console_printf("WiFi Status: ✓ Connected\n");
        console_printf("  SSID: %s\n", ap_info.ssid);
        console_printf("  Signal strength: %d dBm\n", ap_info.rssi);
        console_printf("  Channel: %d\n", ap_info.primary);
        
        // Get IP address
        esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (netif) {
            esp_netif_ip_info_t ip_info;
            if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
                console_printf("  IP address: " IPSTR "\n", IP2STR(&ip_info.ip));
                console_printf("  Netmask: " IPSTR "\n", IP2STR(&ip_info.netmask));
                console_printf("  Gateway: " IPSTR "\n", IP2STR(&ip_info.gw));
            }
        }
    } else if (ret == ESP_ERR_WIFI_NOT_CONNECT) {
        console_printf("WiFi Status: Disconnected\n");
    } else {
        console_printf("WiFi Status: Unknown (error: %s)\n", esp_err_to_name(ret));
    }
    
    return 0;
}


static int cmd_wifi_disconnect(int argc, char **argv) {
    console_printf("Disconnecting from WiFi...\n");
    esp_err_t ret = esp_wifi_disconnect();
    if (ret == ESP_OK) {
        console_printf("✓ Disconnected\n");
    } else {
        console_printf("✗ Disconnect failed: %s\n", esp_err_to_name(ret));
    }
    
    return 0;
}

static int cmd_ipconfig(int argc, char **argv) {
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!netif) {
        console_printf("Network interface not found\n");
        return 1;
    }
    
    // Get MAC address
    uint8_t mac[6];
    esp_err_t ret = esp_netif_get_mac(netif, mac);
    if (ret == ESP_OK) {
        console_printf("MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n", 
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
    
    // Get IP info
    esp_netif_ip_info_t ip_info;
    ret = esp_netif_get_ip_info(netif, &ip_info);
    if (ret == ESP_OK) {
        console_printf("IP Address: " IPSTR "\n", IP2STR(&ip_info.ip));
        console_printf("Netmask:    " IPSTR "\n", IP2STR(&ip_info.netmask));
        console_printf("Gateway:    " IPSTR "\n", IP2STR(&ip_info.gw));
    } else {
        console_printf("No IP address assigned (not connected)\n");
    }
    
    // Get WiFi connection status
    wifi_ap_record_t ap_info;
    ret = esp_wifi_sta_get_ap_info(&ap_info);
    if (ret == ESP_OK) {
        console_printf("Connected to: %s (RSSI: %d dBm, Ch: %d)\n", 
            ap_info.ssid, ap_info.rssi, ap_info.primary);
    } else {
        console_printf("WiFi: Not connected\n");
    }
    
    // FTP server status
    if (ftp_server_is_running()) {
        console_printf("FTP Server: Running at ftp://" IPSTR ":21\n", IP2STR(&ip_info.ip));
    } else {
        console_printf("FTP Server: Not running\n");
    }
    
    return 0;
}

static int cmd_ftp_start(int argc, char **argv) {
    if (!wifi_is_connected()) {
        console_printf("✗ WiFi not connected. Connect to WiFi first.\n");
        return 1;
    }
    
    if (ftp_server_is_running()) {
        console_printf("✓ FTP server is already running\n");
        return 0;
    }
    
    esp_err_t ret = ftp_server_init();
    if (ret == ESP_OK) {
        char ip_str[16];
        wifi_get_ip(ip_str, sizeof(ip_str));
        console_printf("✓ FTP server started\n");
        console_printf("  Address: ftp://%s:21\n", ip_str);
        console_printf("  Root: /sdcard\n");
        console_printf("  Username: anonymous (any)\n");
        console_printf("  Password: (none)\n");
    } else {
        console_printf("✗ Failed to start FTP server: %s\n", esp_err_to_name(ret));
    }
    
    return 0;
}

static int cmd_ftp_stop(int argc, char **argv) {
    if (!ftp_server_is_running()) {
        console_printf("FTP server is not running\n");
        return 0;
    }
    
    ftp_server_stop();
    console_printf("✓ FTP server stopped\n");
    return 0;
}

static int cmd_ftp_status(int argc, char **argv) {
    if (ftp_server_is_running()) {
        char ip_str[16];
        if (wifi_get_ip(ip_str, sizeof(ip_str)) == ESP_OK) {
            console_printf("✓ FTP server running\n");
            console_printf("  Address: ftp://%s:21\n", ip_str);
            console_printf("  Root: /sdcard\n");
        } else {
            console_printf("✓ FTP server running (IP unavailable)\n");
        }
    } else {
        console_printf("FTP server is not running\n");
        console_printf("Use 'ftpstart' to start the server\n");
    }
    return 0;
}

// WiFi command registration enabled
static void register_wifi_ftp_commands(void) {
    const esp_console_cmd_t wifi_scan_cmd = {
        .command = "scan",
        .help = "Scan for WiFi networks",
        .hint = NULL,
        .func = &cmd_wifi_scan,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&wifi_scan_cmd));
    
    const esp_console_cmd_t wifi_connect_cmd = {
        .command = "connect",
        .help = "Connect to WiFi network",
        .hint = "<ssid> <password>",
        .func = &cmd_wifi_connect,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&wifi_connect_cmd));
    
    const esp_console_cmd_t wifi_status_cmd = {
        .command = "status",
        .help = "Show WiFi status",
        .hint = NULL,
        .func = &cmd_wifi_status,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&wifi_status_cmd));
    
    const esp_console_cmd_t wifi_disconnect_cmd = {
        .command = "disconnect",
        .help = "Disconnect from WiFi",
        .hint = NULL,
        .func = &cmd_wifi_disconnect,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&wifi_disconnect_cmd));
    
    const esp_console_cmd_t ipconfig_cmd = {
        .command = "ipconfig",
        .help = "Show network configuration",
        .hint = NULL,
        .func = &cmd_ipconfig,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&ipconfig_cmd));
    
    // FTP commands
    const esp_console_cmd_t ftp_start_cmd = {
        .command = "ftpstart",
        .help = "Start FTP server",
        .hint = NULL,
        .func = &cmd_ftp_start,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&ftp_start_cmd));
    
    const esp_console_cmd_t ftp_stop_cmd = {
        .command = "ftpstop",
        .help = "Stop FTP server",
        .hint = NULL,
        .func = &cmd_ftp_stop,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&ftp_stop_cmd));
    
    const esp_console_cmd_t ftp_status_cmd = {
        .command = "ftpstatus",
        .help = "Show FTP server status",
        .hint = NULL,
        .func = &cmd_ftp_status,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&ftp_status_cmd));
}
// WiFi/FTP commands enabled

/* ====== Application Loader Commands ====== */

static int cmd_run_m68k(int argc, char **argv) {
    if (argc < 2) {
        console_printf("Usage: runm68k <filename>\n");
        console_printf("Load and run M68K binary from SD card\n");
        return 1;
    }
    
    const char *filename = argv[1];
    console_printf("Loading M68K app: %s\n", filename);
    
    esp_err_t ret = app_run_m68k(filename);
    if (ret == ESP_OK) {
        console_printf("✓ M68K application loaded\n");
        console_printf("  Network access: Available via 0x%08X\n", BUS_IO_BASE);
        console_printf("  Use 'step' or 'run' to execute\n");
    } else {
        console_printf("✗ Failed to load M68K app: %s\n", esp_err_to_name(ret));
        return 1;
    }
    
    return 0;
}

static int cmd_appinfo(int argc, char **argv) {
    application_t *app = app_get_current();
    if (!app || !app->running) {
        console_printf("No application running\n");
        return 0;
    }
    
    console_printf("Current Application:\n");
    console_printf("  Name: %s\n", app->name);
    console_printf("  Type: %s\n", app->type == APP_TYPE_M68K ? "M68K" : "ARM Native");
    console_printf("  Entry: 0x%08lX\n", (unsigned long)app->entry_point);
    console_printf("  Stack: %lu KB\n", (unsigned long)(app->stack_size / 1024));
    console_printf("  Status: %s\n", app->running ? "Running" : "Stopped");
    
    if (app->type == APP_TYPE_M68K) {
        console_printf("\nNetwork device at: 0x%08X\n", BUS_IO_BASE);
        console_printf("  Commands: socket, bind, connect, send, recv\n");
        console_printf("  DMA at: 0x%08X\n", BUS_IO_BASE + BUS_DEV_DMA);
    }
    
    return 0;
}

/* ====== i8086 PC Emulator Commands ====== */

static int cmd_init86(int argc, char **argv) {
    if (i86_is_initialized()) {
        console_printf("i8086 already initialized\n");
        return 0;
    }
    
    esp_err_t ret = i86_init(NULL);
    if (ret == ESP_OK) {
        console_printf("✓ i8086 PC emulator initialized\n");
        console_printf("  640KB RAM, 128KB video memory\n");
        console_printf("  Use 'mount' to attach disk images\n");
        console_printf("  Use 'run86' to boot from disk\n");
    } else {
        console_printf("✗ Failed to initialize: %s\n", esp_err_to_name(ret));
    }
    return 0;
}

static int cmd_reset86(int argc, char **argv) {
    if (!i86_is_initialized()) {
        console_printf("i8086 not initialized. Use 'init86' first.\n");
        return 1;
    }
    
    i86_reset();
    console_printf("i8086 CPU reset\n");
    return 0;
}

static int cmd_state86(int argc, char **argv) {
    if (!i86_is_initialized()) {
        console_printf("i8086 not initialized\n");
        return 1;
    }
    
    char buffer[1024];
    i86_get_state_string(buffer, sizeof(buffer));
    console_printf("%s", buffer);
    return 0;
}

static int cmd_mount(int argc, char **argv) {
    if (argc < 3) {
        console_printf("Usage: mount <drive> <image>\n");
        console_printf("  drive: A, B, C, or D\n");
        console_printf("  image: path to .img file on SD card\n");
        console_printf("Example: mount A freedos.img\n");
        return 1;
    }
    
    char drive_letter = toupper(argv[1][0]);
    if (drive_letter < 'A' || drive_letter > 'D') {
        console_printf("Invalid drive: %c (must be A-D)\n", drive_letter);
        return 1;
    }
    
    uint8_t drive = drive_letter - 'A';
    const char *filename = argv[2];
    
    // Prepend /sdcard if not absolute path
    char fullpath[300];
    if (filename[0] != '/') {
        snprintf(fullpath, sizeof(fullpath), "/sdcard/%s", filename);
        filename = fullpath;
    }
    
    esp_err_t ret = disk_mount(drive, filename, false);
    if (ret == ESP_OK) {
        disk_info_t info;
        disk_get_info(drive, &info);
        console_printf("✓ Mounted %c: %s\n", drive_letter, info.filename);
        console_printf("  Type: %s\n", disk_type_name(info.type));
        console_printf("  Size: %llu KB\n", info.size_bytes / 1024);
    } else {
        console_printf("✗ Mount failed: %s\n", esp_err_to_name(ret));
    }
    
    return 0;
}

static int cmd_unmount(int argc, char **argv) {
    if (argc < 2) {
        console_printf("Usage: unmount <drive>\n");
        return 1;
    }
    
    char drive_letter = toupper(argv[1][0]);
    if (drive_letter < 'A' || drive_letter > 'D') {
        console_printf("Invalid drive\n");
        return 1;
    }
    
    uint8_t drive = drive_letter - 'A';
    disk_unmount(drive);
    console_printf("Drive %c: unmounted\n", drive_letter);
    return 0;
}

static int cmd_disks(int argc, char **argv) {
    char buffer[1024];
    disk_list(buffer, sizeof(buffer));
    console_printf("%s", buffer);
    return 0;
}

static int cmd_mkdisk(int argc, char **argv) {
    if (argc < 3) {
        console_printf("Usage: mkdisk <filename> <type>\n");
        console_printf("Types:\n");
        console_printf("  360  - 360KB floppy\n");
        console_printf("  720  - 720KB floppy\n");
        console_printf("  1.2  - 1.2MB floppy\n");
        console_printf("  1.44 - 1.44MB floppy (default)\n");
        console_printf("  2.88 - 2.88MB floppy\n");
        console_printf("  hd:N - Hard disk, N MB (e.g., hd:10)\n");
        console_printf("Example: mkdisk boot.img 1.44\n");
        return 1;
    }
    
    const char *filename = argv[1];
    const char *type_str = argv[2];
    
    // Prepend /sdcard if not absolute
    char fullpath[300];
    if (filename[0] != '/') {
        snprintf(fullpath, sizeof(fullpath), "/sdcard/%s", filename);
        filename = fullpath;
    }
    
    esp_err_t ret = ESP_FAIL;
    
    if (strncmp(type_str, "hd:", 3) == 0) {
        // Hard disk
        uint32_t size_mb = atoi(type_str + 3);
        if (size_mb == 0 || size_mb > 2048) {
            console_printf("Invalid HD size (1-2048 MB)\n");
            return 1;
        }
        ret = disk_create_hd_image(filename, size_mb);
    } else {
        // Floppy
        disk_type_t type = DISK_TYPE_FLOPPY_1M44; // default
        
        if (strcmp(type_str, "360") == 0) type = DISK_TYPE_FLOPPY_360K;
        else if (strcmp(type_str, "720") == 0) type = DISK_TYPE_FLOPPY_720K;
        else if (strcmp(type_str, "1.2") == 0) type = DISK_TYPE_FLOPPY_1M2;
        else if (strcmp(type_str, "1.44") == 0) type = DISK_TYPE_FLOPPY_1M44;
        else if (strcmp(type_str, "2.88") == 0) type = DISK_TYPE_FLOPPY_2M88;
        else {
            console_printf("Unknown type: %s\n", type_str);
            return 1;
        }
        
        ret = disk_create_image(filename, type);
    }
    
    if (ret == ESP_OK) {
        console_printf("✓ Created %s\n", filename);
        console_printf("Use 'mount A %s' to attach it\n", filename);
    } else {
        console_printf("✗ Failed to create image\n");
    }
    
    return 0;
}

static int cmd_switch(int argc, char **argv) {
    if (argc < 2) {
        const char *name = "M68K";
        if (active_cpu == ACTIVE_CPU_I8086) name = "i8086";
        else if (active_cpu == ACTIVE_CPU_6502) name = "6502";
        console_printf("Current CPU: %s\n", name);
        console_printf("Usage: switch <m68k|i8086|6502>\n");
        return 0;
    }

    if (strcasecmp(argv[1], "m68k") == 0) {
        active_cpu = ACTIVE_CPU_M68K;
        console_printf("Switched to M68K emulator\n");
    } else if (strcasecmp(argv[1], "i8086") == 0 || strcasecmp(argv[1], "86") == 0) {
        active_cpu = ACTIVE_CPU_I8086;
        console_printf("Switched to i8086 emulator\n");
    } else if (strcasecmp(argv[1], "6502") == 0) {
        active_cpu = ACTIVE_CPU_6502;
        console_printf("Switched to MOS 6502 emulator\n");
    } else {
        console_printf("Unknown CPU: %s\n", argv[1]);
        return 1;
    }
    
    return 0;
}

static int cmd_run86(int argc, char **argv) {
    if (!i86_is_initialized()) {
        console_printf("i8086 not initialized. Use 'init86' first.\n");
        return 1;
    }
    
    // Parse optional cycle count
    uint32_t count = 0;  // 0 = run continuously
    if (argc >= 2) {
        count = strtoul(argv[1], NULL, 0);
    }
    
    if (count > 0) {
        // Run a specific number of instructions
        console_printf("Running %lu i8086 instructions...\n", (unsigned long)count);
        uint32_t executed = i86_run(count);
        console_printf("Executed %lu instructions.\n", (unsigned long)executed);
    } else {
        // Boot and run continuously
        console_printf("Booting i8086 PC emulator...\n");
        console_printf("Press Ctrl+C to stop.\n\n");
        i86_boot_and_run();
    }
    
    return 0;
}

static void i86_run_task(void *arg) {
    i86_boot_and_run();
    s_i86_task_handle = NULL;
    vTaskDelete(NULL);
}

// ---- Atari ST emulator task and commands ----

static void atari_st_run_task(void *arg) {
    atari_st_run();  // Runs until stopped (Ctrl+C or atari_st_stop())
    atari_st_destroy();
    printf("Atari ST emulator exited.\n");
    s_atari_task_handle = NULL;
    vTaskDelete(NULL);
}

static int cmd_stop_atari(int argc, char **argv) {
    if (!atari_st_is_running()) {
        console_printf("Atari ST is not running.\n");
        return 1;
    }
    atari_st_stop();
    console_printf("Atari ST emulator stopping...\n");
    return 0;
}

static int cmd_state_atari(int argc, char **argv) {
    char buf[512];
    atari_st_get_state(buf, sizeof(buf));
    console_printf("%s", buf);
    return 0;
}

// ---- Amiga emulator task and commands ----
static void amiga_run_task(void *arg) {
    amiga_run();  // Runs until stopped (Ctrl+C or amiga_stop())
    amiga_destroy();
    printf("Amiga emulator exited.\n");
    s_amiga_task_handle = NULL;
    vTaskDelete(NULL);
}

static int cmd_stop_amiga(int argc, char **argv) {
    if (!amiga_is_running()) {
        console_printf("Amiga is not running.\n");
        return 1;
    }
    amiga_stop();
    console_printf("Amiga emulator stopping...\n");
    return 0;
}

static int cmd_state_amiga(int argc, char **argv) {
    char buf[512];
    amiga_get_state(buf, sizeof(buf));
    console_printf("%s", buf);
    return 0;
}

static int cmd_amiga_floppy(int argc, char **argv) {
    if (argc < 2) {
        console_printf("Usage: amigafloppy <path> [drive]\n");
        console_printf("  drive: 0-3 (default 0)\n");
        return 1;
    }
    int drive = 0;
    if (argc >= 3) {
        drive = atoi(argv[2]);
        if (drive < 0 || drive > 3) {
            console_printf("Invalid drive number (0-3)\n");
            return 1;
        }
    }
    if (strcmp(argv[1], "eject") == 0) {
        amiga_eject_floppy(drive);
        console_printf("Ejected floppy from DF%d\n", drive);
        return 0;
    }
    esp_err_t ret = amiga_insert_floppy(drive, argv[1]);
    if (ret == ESP_OK) {
        console_printf("Inserted %s in DF%d\n", argv[1], drive);
    } else {
        console_printf("Failed to insert floppy: %s\n", esp_err_to_name(ret));
    }
    return (ret == ESP_OK) ? 0 : 1;
}

static int cmd_amiga_hdf(int argc, char **argv) {
    if (argc < 2) {
        console_printf("Usage: amigahdf <path> [unit]\n");
        console_printf("  unit: 0-1 (default 0)\n");
        return 1;
    }
    int unit = 0;
    if (argc >= 3) {
        unit = atoi(argv[2]);
        if (unit < 0 || unit > 1) {
            console_printf("Invalid unit number (0-1)\n");
            return 1;
        }
    }
    if (strcmp(argv[1], "eject") == 0) {
        amiga_unmount_hdf(unit);
        console_printf("Unmounted HDF from unit %d\n", unit);
        return 0;
    }
    esp_err_t ret = amiga_mount_hdf(unit, argv[1]);
    if (ret == ESP_OK) {
        console_printf("Mounted %s as HD%d\n", argv[1], unit);
    } else {
        console_printf("Failed to mount HDF: %s\n", esp_err_to_name(ret));
    }
    return (ret == ESP_OK) ? 0 : 1;
}

static int cmd_boot86(int argc, char **argv) {
    if (!i86_is_initialized()) {
        // Auto-initialize
        i86_config_t config = {
            .ram_size = I86_RAM_SIZE,
            .cpu_freq_mhz = 8,
            .enable_fpu = false,
            .enable_sound = false,
            .boot_disk = NULL,
        };
        if (i86_init(&config) != ESP_OK) {
            console_printf("Failed to initialize i8086.\n");
            return 1;
        }
    }
    
    // Check if a disk image argument is provided
    if (argc >= 2) {
        const char *disk_path = argv[1];
        // Auto-mount as drive A: or C: depending on type
        disk_type_t type = disk_detect_type(disk_path);
        int drive = (type == DISK_TYPE_HARD_DISK) ? 2 : 0;
        
        if (i86_mount_disk(drive, disk_path) != ESP_OK) {
            console_printf("Failed to mount disk: %s\n", disk_path);
            return 1;
        }
        console_printf("Mounted %s as drive %c:\n", disk_path,
                       drive < 2 ? 'A' + drive : 'C' + (drive - 2));
    }
    
    // Check at least one disk is mounted
    bool any_disk = false;
    for (int i = 0; i < MAX_DISK_DRIVES; i++) {
        if (disk_is_mounted(i)) {
            any_disk = true;
            break;
        }
    }
    if (!any_disk) {
        console_printf("No disk mounted. Usage: boot86 <disk_image.img>\n");
        console_printf("Or mount a disk first: mount A /sdcard/freedos.img\n");
        return 1;
    }
    
    if (s_i86_task_handle != NULL) {
        console_printf("i8086 emulator is already running.\n");
        return 1;
    }
    
    // Run in a FreeRTOS task
    console_printf("Starting i8086 PC emulator...\n");
    BaseType_t ret = xTaskCreate(i86_run_task, "i8086_emu", 16384, NULL, 5, &s_i86_task_handle);
    if (ret != pdPASS) {
        console_printf("Failed to create emulator task.\n");
        return 1;
    }
    
    return 0;
}

static int cmd_stop86(int argc, char **argv) {
    if (!i86_is_initialized()) {
        console_printf("i8086 not initialized.\n");
        return 1;
    }
    i86_stop();
    console_printf("i8086 emulator stopped.\n");
    return 0;
}

static void register_i8086_commands(void) {
    const esp_console_cmd_t init86_cmd = {
        .command = "init86",
        .help = "Initialize i8086 PC emulator",
        .hint = NULL,
        .func = &cmd_init86,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&init86_cmd));
    
    const esp_console_cmd_t reset86_cmd = {
        .command = "reset86",
        .help = "Reset i8086 CPU",
        .hint = NULL,
        .func = &cmd_reset86,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&reset86_cmd));
    
    const esp_console_cmd_t state86_cmd = {
        .command = "state86",
        .help = "Show i8086 CPU state",
        .hint = NULL,
        .func = &cmd_state86,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&state86_cmd));
    
    const esp_console_cmd_t mount_cmd = {
        .command = "mount",
        .help = "Mount disk image",
        .hint = "<drive> <image>",
        .func = &cmd_mount,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&mount_cmd));
    
    const esp_console_cmd_t unmount_cmd = {
        .command = "unmount",
        .help = "Unmount disk",
        .hint = "<drive>",
        .func = &cmd_unmount,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&unmount_cmd));
    
    const esp_console_cmd_t disks_cmd = {
        .command = "disks",
        .help = "List mounted disks",
        .hint = NULL,
        .func = &cmd_disks,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&disks_cmd));
    
    const esp_console_cmd_t mkdisk_cmd = {
        .command = "mkdisk",
        .help = "Create blank disk image",
        .hint = "<filename> <type>",
        .func = &cmd_mkdisk,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&mkdisk_cmd));
    
    const esp_console_cmd_t switch_cmd = {
        .command = "switch",
        .help = "Switch active CPU",
        .hint = "<m68k|i8086>",
        .func = &cmd_switch,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&switch_cmd));
    
    const esp_console_cmd_t run86_cmd = {
        .command = "run86",
        .help = "Run i8086 emulator [count]",
        .hint = "[instructions]",
        .func = &cmd_run86,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&run86_cmd));

    const esp_console_cmd_t boot86_cmd = {
        .command = "boot86",
        .help = "Boot i8086 PC from disk image",
        .hint = "[image_path]",
        .func = &cmd_boot86,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&boot86_cmd));

    const esp_console_cmd_t stop86_cmd = {
        .command = "stop86",
        .help = "Stop i8086 emulator",
        .hint = NULL,
        .func = &cmd_stop86,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&stop86_cmd));
}

// ============================================================================
// MOS 6502 Emulator Commands
// ============================================================================

static int cmd_init6502(int argc, char **argv) {
    esp_err_t err = mos6502_emu_init();
    if (err == ESP_OK) {
        console_printf("MOS 6502 emulator initialized (64KB RAM)\n");
    } else {
        console_printf("Failed to initialize 6502: %s\n", esp_err_to_name(err));
    }
    return 0;
}

static int cmd_reset6502(int argc, char **argv) {
    if (!mos6502_emu_is_initialized()) {
        console_printf("6502 not initialized. Run 'init6502' first.\n");
        return 1;
    }
    mos6502_emu_reset();
    char buf[256];
    mos6502_emu_get_state(buf, sizeof(buf));
    console_printf("%s", buf);
    return 0;
}

static int cmd_state6502(int argc, char **argv) {
    char buf[256];
    mos6502_emu_get_state(buf, sizeof(buf));
    console_printf("%s", buf);
    return 0;
}

static int cmd_step6502(int argc, char **argv) {
    if (!mos6502_emu_is_initialized()) {
        console_printf("6502 not initialized.\n");
        return 1;
    }
    mos6502_emu_step();
    char buf[256];
    mos6502_emu_get_state(buf, sizeof(buf));
    console_printf("%s", buf);
    return 0;
}

static int cmd_dump6502(int argc, char **argv) {
    uint32_t addr = 0;
    uint32_t len = 128;
    if (argc > 1) addr = strtoul(argv[1], NULL, 16);
    if (argc > 2) len = strtoul(argv[2], NULL, 0);
    mos6502_emu_dump_memory(addr, len);
    return 0;
}

static int cmd_load6502(int argc, char **argv) {
    if (argc < 2) {
        console_printf("Usage: load6502 <file> [address]\n");
        console_printf("  Loads binary file from SD card into 6502 memory\n");
        console_printf("  Default address: $E000 (ROM area)\n");
        return 1;
    }

    if (!mos6502_emu_is_initialized()) {
        console_printf("6502 not initialized. Running init6502...\n");
        mos6502_emu_init();
    }

    uint32_t load_addr = 0xE000;
    if (argc > 2) load_addr = strtoul(argv[2], NULL, 16);

    // Build path
    char path[128];
    if (argv[1][0] == '/') {
        snprintf(path, sizeof(path), "%s", argv[1]);
    } else {
        snprintf(path, sizeof(path), "/sdcard/%s", argv[1]);
    }

    FILE *f = fopen(path, "rb");
    if (!f) {
        console_printf("Cannot open file: %s\n", path);
        return 1;
    }

    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0, SEEK_SET);

    uint8_t *buf = malloc(fsize);
    if (!buf) {
        console_printf("Out of memory for %ld bytes\n", fsize);
        fclose(f);
        return 1;
    }

    fread(buf, 1, fsize, f);
    fclose(f);

    mos6502_emu_load_program(buf, fsize, load_addr);
    free(buf);

    console_printf("Loaded %ld bytes from %s at $%04X\n", fsize, path, (uint16_t)load_addr);
    return 0;
}

static void mos6502_run_task(void *arg) {
    // C64 runs at ~1MHz. KERNAL boot takes ~1-2 seconds.
    // Suppress rendering during boot so the BASIC screen appears instantly.
    const uint32_t BATCH_INSTRUCTIONS = 500;
    const uint32_t BOOT_CYCLES = 2000000;  // ~2 seconds at 1MHz
    uint32_t total_cycles = 0;
    uint32_t cycles_since_render = 0;
    uint32_t poll_counter = 0;
    bool boot_complete = false;

    // Switch video card from TEXT mode to GFX mode so the C64 renderer
    // can use back-buffer drawing + present() for double-buffered output.
    // Without this, present() is a no-op and the LCD keeps showing the
    // text-mode welcome screen instead of the C64 display.
    if (video_card_is_initialized()) {
        video_card_enter_gfx_mode();
        video_card_present();  // Show initial black frame
    }

    // Phase 1: KERNAL boot - run CPU fast without rendering
    // This lets the BASIC startup message appear all at once
    while (!boot_complete) {
        uint64_t cycles = mos6502_emu_run(BATCH_INSTRUCTIONS);
        if (mos6502_emu_is_halted()) {
            printf("6502 halted during boot\n");
            goto task_exit;
        }
        mos6502_emu_tick((uint32_t)cycles);
        total_cycles += (uint32_t)cycles;

        // Yield every ~50k cycles so watchdog doesn't trigger
        cycles_since_render += (uint32_t)cycles;
        if (cycles_since_render >= 50000) {
            cycles_since_render = 0;
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        if (total_cycles >= BOOT_CYCLES) {
            boot_complete = true;
        }
    }

    // Phase 2: Boot complete - render first frame instantly on both displays
    printf("\033[2J\033[H");  // Clear serial terminal
    fflush(stdout);
    mos6502_emu_render_screen();  // Full screen draw with BASIC prompt
    cycles_since_render = 0;

    // Phase 3: Normal operation - render at 60Hz, poll keyboard, handle input
    while (true) {
        uint64_t cycles = mos6502_emu_run(BATCH_INSTRUCTIONS);

        if (mos6502_emu_is_halted()) {
            printf("\033[27;1H6502 halted\n");
            break;
        }

        mos6502_emu_tick((uint32_t)cycles);
        cycles_since_render += (uint32_t)cycles;

        // Poll keyboard every ~4 batches
        if (++poll_counter >= 4) {
            poll_counter = 0;
            mos6502_emu_poll_keyboard();
            if (mos6502_emu_exit_requested()) {
                printf("\033[27;1H\nExiting C64 emulator...\n");
                fflush(stdout);
                break;
            }
        }

        // ~60Hz screen refresh
        if (cycles_since_render >= 16667) {
            cycles_since_render = 0;
            mos6502_emu_render_screen();
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

task_exit:
    // Restore terminal
    printf("\033[2J\033[H");  // Clear screen
    printf("Returned to BIOS.\n");
    fflush(stdout);
    s_6502_task_handle = NULL;
    vTaskDelete(NULL);
}

static int cmd_run6502(int argc, char **argv) {
    if (!mos6502_emu_is_initialized()) {
        console_printf("6502 not initialized.\n");
        return 1;
    }

    if (argc > 1) {
        uint32_t count = strtoul(argv[1], NULL, 0);
        mos6502_emu_run(count);
        char buf[256];
        mos6502_emu_get_state(buf, sizeof(buf));
        console_printf("%s", buf);
    } else {
        if (s_6502_task_handle) {
            console_printf("6502 already running\n");
            return 0;
        }
        mos6502_emu_reset();
        xTaskCreate(mos6502_run_task, "6502_run", 8192, NULL, 5, &s_6502_task_handle);
        console_printf("6502 running in background\n");
    }
    return 0;
}

static int cmd_stop6502(int argc, char **argv) {
    if (s_6502_task_handle) {
        mos6502_emu_stop();
        console_printf("6502 stopped\n");
    } else {
        console_printf("6502 not running\n");
    }
    return 0;
}

static int cmd_poke6502(int argc, char **argv) {
    if (argc < 3) {
        console_printf("Usage: poke6502 <addr> <byte>\n");
        return 1;
    }
    uint16_t addr = (uint16_t)strtoul(argv[1], NULL, 16);
    uint8_t val = (uint8_t)strtoul(argv[2], NULL, 16);
    mos6502_emu_write_memory(addr, val);
    console_printf("$%04X = $%02X\n", addr, val);
    return 0;
}

// ============================================================================
// Video Card Commands
// ============================================================================

static int cmd_vinit(int argc, char **argv) {
    esp_err_t err = video_card_init();
    if (err == ESP_OK) {
        console_printf("Video card initialized (480x320, RGB565, double-buffered)\n");
    } else {
        console_printf("Failed to initialize video card: %s\n", esp_err_to_name(err));
    }
    return 0;
}

static int cmd_vtest(int argc, char **argv) {
    if (!video_card_is_initialized()) {
        console_printf("Video card not initialized. Run 'vinit' first.\n");
        return 1;
    }

    // Set mode and enable
    video_card_write(VID_REG_MODE, VID_MODE_480x320x16, 4);
    video_card_write(VID_REG_COMMAND, VID_CMD_INIT, 4);

    // Clear to dark blue
    video_card_write(VID_REG_BG_COLOR, 0x0011, 4);
    video_card_write(VID_REG_COMMAND, VID_CMD_CLEAR, 4);

    // Draw some shapes
    // Red filled rectangle
    video_card_write(VID_REG_DRAW_X, 20, 4);
    video_card_write(VID_REG_DRAW_Y, 20, 4);
    video_card_write(VID_REG_DRAW_W, 100, 4);
    video_card_write(VID_REG_DRAW_H, 80, 4);
    video_card_write(VID_REG_DRAW_COLOR, 0xF800, 4);
    video_card_write(VID_REG_COMMAND, VID_CMD_FILL_RECT, 4);

    // Green filled rectangle
    video_card_write(VID_REG_DRAW_X, 150, 4);
    video_card_write(VID_REG_DRAW_Y, 50, 4);
    video_card_write(VID_REG_DRAW_W, 80, 4);
    video_card_write(VID_REG_DRAW_H, 60, 4);
    video_card_write(VID_REG_DRAW_COLOR, 0x07E0, 4);
    video_card_write(VID_REG_COMMAND, VID_CMD_FILL_RECT, 4);

    // Blue circle
    video_card_write(VID_REG_DRAW_X, 350, 4);
    video_card_write(VID_REG_DRAW_Y, 160, 4);
    video_card_write(VID_REG_DRAW_W, 60, 4);
    video_card_write(VID_REG_DRAW_COLOR, 0x001F, 4);
    video_card_write(VID_REG_COMMAND, VID_CMD_FILL_CIRCLE, 4);

    // White diagonal lines
    for (int i = 0; i < 8; i++) {
        video_card_write(VID_REG_DRAW_X, 0, 4);
        video_card_write(VID_REG_DRAW_Y, i * 40, 4);
        video_card_write(VID_REG_DRAW_W, 479, 4);
        video_card_write(VID_REG_DRAW_H, 319 - i * 40, 4);
        video_card_write(VID_REG_DRAW_COLOR, 0xFFFF, 4);
        video_card_write(VID_REG_COMMAND, VID_CMD_DRAW_LINE, 4);
    }

    // Yellow horizontal line
    video_card_write(VID_REG_DRAW_X, 0, 4);
    video_card_write(VID_REG_DRAW_Y, 160, 4);
    video_card_write(VID_REG_DRAW_W, 480, 4);
    video_card_write(VID_REG_DRAW_COLOR, 0xFFE0, 4);
    video_card_write(VID_REG_COMMAND, VID_CMD_HLINE, 4);

    // Flip to display
    video_card_write(VID_REG_COMMAND, VID_CMD_FLIP, 4);

    console_printf("Video test pattern drawn\n");
    return 0;
}

static int cmd_vstatus(int argc, char **argv) {
    if (!video_card_is_initialized()) {
        console_printf("Video card: not initialized\n");
        return 0;
    }
    uint32_t status = video_card_read(VID_REG_STATUS, 4);
    uint32_t width = video_card_read(VID_REG_WIDTH, 4);
    uint32_t height = video_card_read(VID_REG_HEIGHT, 4);
    uint32_t frames = video_card_get_frame_count();

    console_printf("Video Card Status:\n");
    console_printf("  Resolution: %dx%d\n", width, height);
    console_printf("  Status: 0x%02X [%s%s%s]\n", status,
                   (status & VID_STATUS_READY) ? "READY " : "",
                   (status & VID_STATUS_VBLANK) ? "VBLANK " : "",
                   (status & VID_STATUS_GFX_MODE) ? "GFX" : "TEXT");
    console_printf("  Frames: %u\n", frames);
    return 0;
}

static int cmd_voff(int argc, char **argv) {
    if (video_card_is_initialized()) {
        video_card_deinit();
        console_printf("Video card disabled\n");
    } else {
        console_printf("Video card not active\n");
    }
    return 0;
}

// ============================================================================
// 6502 + Video Command Registration
// ============================================================================

static void register_6502_commands(void) {
    const esp_console_cmd_t cmds[] = {
        { .command = "init6502", .help = "Initialize MOS 6502 CPU emulator",
          .func = &cmd_init6502 },
        { .command = "reset6502", .help = "Reset 6502 CPU",
          .func = &cmd_reset6502 },
        { .command = "state6502", .help = "Show 6502 CPU state",
          .func = &cmd_state6502 },
        { .command = "step6502", .help = "Single-step 6502",
          .func = &cmd_step6502 },
        { .command = "run6502", .help = "Run 6502 [count] or continuously",
          .hint = "[count]", .func = &cmd_run6502 },
        { .command = "stop6502", .help = "Stop 6502 execution",
          .func = &cmd_stop6502 },
        { .command = "load6502", .help = "Load binary into 6502 memory",
          .hint = "<file> [addr]", .func = &cmd_load6502 },
        { .command = "dump6502", .help = "Dump 6502 memory",
          .hint = "[addr] [len]", .func = &cmd_dump6502 },
        { .command = "poke6502", .help = "Write byte to 6502 memory",
          .hint = "<addr> <byte>", .func = &cmd_poke6502 },
    };
    for (int i = 0; i < sizeof(cmds)/sizeof(cmds[0]); i++) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmds[i]));
    }
}

static void register_video_commands(void) {
    const esp_console_cmd_t cmds[] = {
        { .command = "vinit", .help = "Initialize video card (480x320 GPU)",
          .func = &cmd_vinit },
        { .command = "vtest", .help = "Draw video test pattern",
          .func = &cmd_vtest },
        { .command = "vstatus", .help = "Show video card status",
          .func = &cmd_vstatus },
        { .command = "voff", .help = "Disable video card",
          .func = &cmd_voff },
    };
    for (int i = 0; i < sizeof(cmds)/sizeof(cmds[0]); i++) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmds[i]));
    }
}

static int cmd_appstop(int argc, char **argv) {
    application_t *app = app_get_current();
    if (!app || !app->running) {
        console_printf("No application running\n");
        return 0;
    }
    
    console_printf("Stopping application: %s\n", app->name);
    app_stop();
    console_printf("✓ Application stopped\n");
    
    return 0;
}

// ===== Bluetooth Commands =====
// ESP32-P4 uses ESP32-C6 coprocessor for Bluetooth via SDIO
// Using NimBLE stack with ESP-Hosted framework

static int cmd_btinit(int argc, char **argv) {
    if (bt_initialized) {
        console_printf("Bluetooth already initialized\n");
        return 0;
    }
    
    console_printf("Initializing Bluetooth (ESP32-C6 via SDIO)...\n");
    
    // Initialize NimBLE controller
    esp_err_t ret = nimble_port_init();
    if (ret != ESP_OK) {
        console_printf("✗ NimBLE port init failed: %s\n", esp_err_to_name(ret));
        return 1;
    }
    
    // Initialize NimBLE host stack
    ble_hs_cfg.sync_cb = NULL;  // No sync callback needed for scanning
    ble_hs_cfg.reset_cb = NULL; // No reset callback
    
    // Start NimBLE host task
    nimble_port_freertos_init(nimble_host_task);
    
    bt_initialized = true;
    console_printf("✓ Bluetooth initialized and ready\n");
    console_printf("  Mode: BLE (Bluetooth Low Energy) via NimBLE\n");
    console_printf("  Transport: ESP-Hosted over SDIO\n");
    console_printf("  Use 'btscan' to discover devices\n");
    
    return 0;
}

static int cmd_btscan(int argc, char **argv) {
    if (!bt_initialized) {
        console_printf("Bluetooth not initialized. Run 'btinit' first.\n");
        return 1;
    }
    
    if (bt_scanning) {
        console_printf("Scan already in progress...\n");
        return 1;
    }
    
    uint32_t duration_ms = 10000; // Default 10 seconds
    if (argc >= 2) {
        int duration = atoi(argv[1]);
        if (duration <= 0 || duration > 60) {
            console_printf("Invalid duration. Use 1-60 seconds.\n");
            return 1;
        }
        duration_ms = duration * 1000;
    }
    
    // Clear previous scan results
    memset(ble_devices, 0, sizeof(ble_devices));
    ble_device_count = 0;
    
    console_printf("Starting BLE scan for %lu seconds...\n", (unsigned long)(duration_ms/1000));
    
    // Configure scan parameters
    struct ble_gap_disc_params disc_params = {
        .itvl = 0x10,  // Scan interval: 10ms
        .window = 0x10, // Scan window: 10ms
        .filter_policy = 0,  // Accept all
        .limited = 0,  // Unlimited scan
        .passive = 0,  // Active scan
        .filter_duplicates = 0
    };
    
    // Start BLE discovery
    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, duration_ms, &disc_params, gap_event_handler, NULL);
    if (rc != 0) {
        console_printf("✗ Failed to start BLE scan: %d\n", rc);
        return 1;
    }
    
    bt_scanning = true;
    
    // Wait for scan to complete
    console_printf("Scanning");
    for (uint32_t i = 0; i < duration_ms/1000; i++) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        console_printf(".");
        fflush(stdout);
    }
    console_printf("\n");
    
    // Wait a bit more for completion event
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Display results
    console_printf("\n=== Discovered BLE Devices (%d) ===\n", ble_device_count);
    if (ble_device_count == 0) {
        console_printf("No devices found.\n");
        console_printf("Tips:\n");
        console_printf("  - Make sure device is powered on and advertising\n");
        console_printf("  - Try increasing scan duration: btscan 20\n");
    } else {
        for (int i = 0; i < ble_device_count; i++) {
            console_printf("%d. %s\n", i + 1, ble_devices[i].name);
            console_printf("   MAC: %02X:%02X:%02X:%02X:%02X:%02X  RSSI: %d dBm\n",
                         ble_devices[i].address[0], ble_devices[i].address[1],
                         ble_devices[i].address[2], ble_devices[i].address[3],
                         ble_devices[i].address[4], ble_devices[i].address[5],
                         ble_devices[i].rssi);
        }
    }
    
    return 0;
}

static int cmd_btconnect(int argc, char **argv) {
    console_printf("Bluetooth connection not yet available.\n");
    console_printf("Run 'btinit' for details on Bluetooth support status.\n");
    return 0;
}

static int cmd_btdisconnect(int argc, char **argv) {
    console_printf("Bluetooth disconnection not yet available.\n");
    console_printf("Run 'btinit' for details on Bluetooth support status.\n");
    return 0;
}

static int cmd_btlist(int argc, char **argv) {
    if (!bt_initialized) {
        console_printf("Bluetooth not initialized. Run 'btinit' first.\n");
        return 1;
    }
    
    console_printf("\n=== Bluetooth Status ===\n");
    console_printf("Mode: BLE (NimBLE) via ESP-Hosted SDIO\n");
    console_printf("Status: %s\n\n", bt_scanning ? "Scanning..." : "Idle");
    
    if (ble_device_count == 0) {
        console_printf("No devices discovered. Run 'btscan' to find devices.\n");
    } else {
        console_printf("Discovered Devices: %d\n", ble_device_count);
        for (int i = 0; i < ble_device_count; i++) {
            console_printf("  %d. %s\n", i + 1, ble_devices[i].name);
            console_printf("     MAC: %02X:%02X:%02X:%02X:%02X:%02X  RSSI: %d dBm\n",
                         ble_devices[i].address[0], ble_devices[i].address[1],
                         ble_devices[i].address[2], ble_devices[i].address[3],
                         ble_devices[i].address[4], ble_devices[i].address[5],
                         ble_devices[i].rssi);
        }
    }
    
    return 0;
}

/*
 * i386 PC Emulator Commands (REMOVED)
 */
static int cmd_init386(int argc, char **argv) {
    console_printf("i386 PC emulator has been removed from this build\n");
    console_printf("The tiny386 integration was too large and unstable\n");
    return ESP_OK;
}

static void register_i386_commands(void) {
    const esp_console_cmd_t init386_cmd = {
        .command = "init386",
        .help = "i386 PC emulator (removed)",
        .hint = NULL,
        .func = &cmd_init386,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&init386_cmd));
}

static void register_bluetooth_commands(void) {
    const esp_console_cmd_t btinit_cmd = {
        .command = "btinit",
        .help = "Initialize Bluetooth controller",
        .hint = NULL,
        .func = &cmd_btinit,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&btinit_cmd));
    
    const esp_console_cmd_t btscan_cmd = {
        .command = "btscan",
        .help = "Scan for Bluetooth devices",
        .hint = "[duration_sec]",
        .func = &cmd_btscan,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&btscan_cmd));
    
    const esp_console_cmd_t btconnect_cmd = {
        .command = "btconnect",
        .help = "Connect to Bluetooth device",
        .hint = "<MAC address>",
        .func = &cmd_btconnect,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&btconnect_cmd));
    
    const esp_console_cmd_t btdisconnect_cmd = {
        .command = "btdisconnect",
        .help = "Disconnect Bluetooth device",
        .hint = "<MAC address>",
        .func = &cmd_btdisconnect,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&btdisconnect_cmd));
    
    const esp_console_cmd_t btlist_cmd = {
        .command = "btlist",
        .help = "List Bluetooth devices and status",
        .hint = NULL,
        .func = &cmd_btlist,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&btlist_cmd));
}

static void register_app_commands(void) {
    const esp_console_cmd_t runm68k_cmd = {
        .command = "runm68k",
        .help = "Load and run M68K binary from SD card",
        .hint = "<filename>",
        .func = &cmd_run_m68k,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&runm68k_cmd));
    
    const esp_console_cmd_t appinfo_cmd = {
        .command = "appinfo",
        .help = "Show current application info",
        .hint = NULL,
        .func = &cmd_appinfo,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&appinfo_cmd));
    
    const esp_console_cmd_t appstop_cmd = {
        .command = "appstop",
        .help = "Stop current application",
        .hint = NULL,
        .func = &cmd_appstop,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&appstop_cmd));
}

/*
 * Custom line input function that reads from both UART and PS/2 keyboard
 * Returns a malloc'd string that must be freed by the caller
 * Supports command history navigation with arrow keys
 */
static char* read_line_dual_input(const char *prompt) {
    static char line_buffer[256];
    int pos = 0;
    int history_index = cmd_history_count;  // Start at end (newest command)
    bool in_escape_seq = false;
    uint8_t escape_buf[3] = {0};
    int escape_pos = 0;
    
    // Print prompt to both outputs
    console_printf("%s", prompt);
    
    while (pos < sizeof(line_buffer) - 1) {
        uint8_t c = 0;
        
        // Check PS/2 keyboard input first (if initialized)
        if (ps2_keyboard_is_initialized() && ps2_keyboard_available()) {
            c = ps2_keyboard_read();
        }
        
        // Check USB keyboard input
        if (c == 0 && usb_keyboard_is_initialized() && usb_keyboard_available()) {
            c = usb_keyboard_read();
        }
        
        // If no PS/2 keyboard input, check UART/USB Serial
        if (c == 0) {
            int uart_char = -1;
            size_t len = 0;
            uart_get_buffered_data_len(CONFIG_ESP_CONSOLE_UART_NUM, &len);
            if (len > 0) {
                uint8_t buf;
                if (uart_read_bytes(CONFIG_ESP_CONSOLE_UART_NUM, &buf, 1, 0) == 1) {
                    uart_char = buf;
                }
            }
            if (uart_char >= 0) {
                c = (uint8_t)uart_char;
            }
        }
        
        // No input from any source
        if (c == 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        
        // Handle escape sequences for arrow keys
        if (in_escape_seq) {
            escape_buf[escape_pos++] = c;
            
            // Check if we have a complete arrow key sequence
            if (escape_pos >= 2) {
                if (escape_buf[0] == '[') {
                    // Arrow up: ESC[A - go back in history (older command)
                    if (escape_buf[1] == 'A' && cmd_history_count > 0 && history_index > 0) {
                        history_index--;
                        if (history_index < cmd_history_count && cmd_history[history_index]) {
                            // Clear current line
                            while (pos > 0) {
                                console_printf("\b \b");
                                pos--;
                            }
                            // Copy history entry to buffer
                            strncpy(line_buffer, cmd_history[history_index], sizeof(line_buffer) - 1);
                            line_buffer[sizeof(line_buffer) - 1] = '\0';
                            pos = strlen(line_buffer);
                            // Display it
                            console_printf("%s", line_buffer);
                        }
                    }
                    // Arrow down: ESC[B - go forward in history (newer command)
                    else if (escape_buf[1] == 'B' && cmd_history_count > 0) {
                        if (history_index < cmd_history_count) {
                            history_index++;
                            if (history_index >= cmd_history_count) {
                                // At newest entry, clear line
                                while (pos > 0) {
                                    console_printf("\b \b");
                                    pos--;
                                }
                                line_buffer[0] = '\0';
                            } else if (cmd_history[history_index]) {
                                // Clear current line
                                while (pos > 0) {
                                    console_printf("\b \b");
                                    pos--;
                                }
                                // Copy history entry to buffer
                                strncpy(line_buffer, cmd_history[history_index], sizeof(line_buffer) - 1);
                                line_buffer[sizeof(line_buffer) - 1] = '\0';
                                pos = strlen(line_buffer);
                                // Display it
                                console_printf("%s", line_buffer);
                            }
                        }
                    }
                    // Arrow right (ESC[C) and left (ESC[D) - ignore for now
                }
                in_escape_seq = false;
                escape_pos = 0;
            }
            continue;
        }
        
        // Handle special keys
        if (c == '\r' || c == '\n') {
            // Enter key - end of line
            line_buffer[pos] = '\0';
            console_printf("\n");
            
            // Return a copy of the line
            char *result = malloc(pos + 1);
            if (result) {
                strcpy(result, line_buffer);
            }
            return result;
        }
        else if (c == '\b' || c == 0x7F) {
            // Backspace
            if (pos > 0) {
                pos--;
                console_printf("\b \b");  // Erase character on screen
            }
        }
        else if (c == 0x03) {
            // Ctrl+C - cancel line
            console_printf("^C\n");
            return NULL;
        }
        else if (c == 0x1B) {
            // Start of escape sequence (arrow keys send ESC[A, ESC[B, etc.)
            in_escape_seq = true;
            escape_pos = 0;
            escape_buf[0] = 0;
            escape_buf[1] = 0;
            escape_buf[2] = 0;
        }
        else if (c >= 0x20 && c < 0x7F) {
            // Printable character
            line_buffer[pos++] = c;
            // Echo to both outputs
            char echo[2] = {c, '\0'};
            console_printf("%s", echo);
        }
        // Ignore other special keys (function keys, etc.)
    }
    
    // Buffer full
    line_buffer[pos] = '\0';
    char *result = malloc(pos + 1);
    if (result) {
        strcpy(result, line_buffer);
    }
    return result;
}

/*
 * PS/2 Keyboard diagnostic command
 */
static int cmd_ps2test(int argc, char **argv) {
    console_printf("=== PS/2 Keyboard Test ===\n");
    
    if (!ps2_keyboard_is_initialized()) {
        console_printf("ERROR: PS/2 keyboard not initialized!\n");
        return 1;
    }
    
    console_printf("PS/2 keyboard is initialized\n");
    console_printf("CLK Pin: GPIO5\n");
    console_printf("DATA Pin: GPIO23\n");
    console_printf("\n");
    console_printf("Press keys on PS/2 keyboard (Ctrl+C to exit)...\n");
    console_printf("Note: Also check UART serial output for debug messages\n");
    
    // Test for 30 seconds or until Ctrl+C
    int count = 0;
    for (int i = 0; i < 300; i++) {  // 30 seconds
        if (ps2_keyboard_available()) {
            uint8_t key = ps2_keyboard_read();
            if (key == 3) {  // Ctrl+C
                console_printf("\n");
                break;
            }
            console_printf("Key received: 0x%02X", key);
            if (key >= 32 && key < 127) {
                console_printf(" '%c'", key);
            }
            console_printf("\n");
            count++;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    console_printf("\nTest complete. Received %d keys.\n", count);
    
    if (count == 0) {
        console_printf("\nTroubleshooting:\n");
        console_printf("1. Check wiring: CLK=GPIO5, DATA=GPIO23, GND, VCC (5V)\n");
        console_printf("2. Verify PS/2 connector orientation\n");
        console_printf("3. Try external 4.7k pull-up resistors on CLK and DATA\n");
        console_printf("4. Check serial monitor for ISR/GPIO error messages\n");
    }
    
    return 0;
}

static int cmd_lcdpins(int argc, char **argv) {
    int rst_pin, bl_pin, mosi_pin, clk_pin;
    
    lcd_console_get_pins(&rst_pin, &bl_pin, &mosi_pin, &clk_pin);
    
    console_printf("\n=== LCD GPIO Pin Configuration ===\n");
    console_printf("RST (Reset):      GPIO%d\n", rst_pin);
    console_printf("BL (Backlight):   GPIO%d\n", bl_pin);
    console_printf("MOSI (Data):      GPIO%d\n", mosi_pin);
    console_printf("CLK (Clock):      GPIO%d\n", clk_pin);
    console_printf("\nThese are the pin numbers compiled into the firmware.\n");
    console_printf("If the display works on a different pin, there may be\n");
    console_printf("a hardware issue or ESP-IDF driver override.\n");
    
    return 0;
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    initialize_console();
    register_linux_commands();  // Register ls, cd, pwd, cat, etc. first
    register_m68k_commands();
    register_i8086_commands();
    register_6502_commands();
    register_video_commands();
    register_i386_commands();       // Register i386 emulator commands
    register_bluetooth_commands();  // Register Bluetooth commands
    register_wifi_ftp_commands();   // Register WiFi and FTP commands
    register_app_commands();
    
    // Register Atari ST commands
    const esp_console_cmd_t stop_atari_cmd = {
        .command = "stopatari",
        .help = "Stop Atari ST emulator",
        .hint = NULL,
        .func = &cmd_stop_atari,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&stop_atari_cmd));
    
    const esp_console_cmd_t state_atari_cmd = {
        .command = "stateatari",
        .help = "Show Atari ST emulator state",
        .hint = NULL,
        .func = &cmd_state_atari,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&state_atari_cmd));

    // Register Amiga commands
    const esp_console_cmd_t stop_amiga_cmd = {
        .command = "stopamiga",
        .help = "Stop Amiga emulator",
        .hint = NULL,
        .func = &cmd_stop_amiga,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&stop_amiga_cmd));

    const esp_console_cmd_t state_amiga_cmd = {
        .command = "stateamiga",
        .help = "Show Amiga emulator state",
        .hint = NULL,
        .func = &cmd_state_amiga,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&state_amiga_cmd));

    const esp_console_cmd_t amiga_floppy_cmd = {
        .command = "amigafloppy",
        .help = "Insert/eject Amiga floppy: amigafloppy <path|eject> [drive]",
        .hint = NULL,
        .func = &cmd_amiga_floppy,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&amiga_floppy_cmd));

    const esp_console_cmd_t amiga_hdf_cmd = {
        .command = "amigahdf",
        .help = "Mount/unmount Amiga HDF: amigahdf <path|eject> [unit]",
        .hint = NULL,
        .func = &cmd_amiga_hdf,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&amiga_hdf_cmd));

    // Register PS/2 test command
    const esp_console_cmd_t ps2test_cmd = {
        .command = "ps2test",
        .help = "Test PS/2 keyboard functionality",
        .hint = NULL,
        .func = &cmd_ps2test,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&ps2test_cmd));
    
    // Register LCD pins diagnostic command
    const esp_console_cmd_t lcdpins_cmd = {
        .command = "lcdpins",
        .help = "Show LCD GPIO pin configuration",
        .hint = NULL,
        .func = &cmd_lcdpins,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&lcdpins_cmd));
    
    // Initialize bus controller (shared networking, DMA, etc.)
    printf("Initializing bus controller...\n");
    ret = bus_controller_init();
    if (ret == ESP_OK) {
        printf("✓ Bus controller initialized\n");
        printf("  Shared TCP/UDP stack ready\n");
        printf("  DMA controller ready\n");
        printf("  Application loader ready\n");
    } else {
        printf("✗ Bus controller init failed: %s\n", esp_err_to_name(ret));
    }
    
    // Initialize LCD console
    printf("Initializing LCD console...\n");
    ret = lcd_console_init();
    if (ret == ESP_OK) {
        lcd_console_active = true;
        printf("✓ LCD console initialized (%dx%d chars)\n", CONSOLE_COLS, CONSOLE_ROWS);
    } else {
        printf("✗ LCD console initialization failed: %s\n", esp_err_to_name(ret));
        printf("  Continuing with UART console only\n");
    }
    
    // Initialize PS/2 keyboard (CLK=GPIO5, DATA=GPIO23)
    printf("Initializing PS/2 keyboard...\n");
    ret = ps2_keyboard_init();
    if (ret == ESP_OK) {
        printf("✓ PS/2 keyboard driver initialized\n");
    } else {
        printf("✗ PS/2 keyboard initialization failed: %s\n", esp_err_to_name(ret));
        printf("  Fallback to UART input\n");
    }
    
    // Initialize USB HID keyboard (USB Type-A host port)
    printf("Initializing USB keyboard...\n");
    ret = usb_keyboard_init();
    if (ret == ESP_OK) {
        printf("✓ USB keyboard driver initialized (plug in USB keyboard)\n");
    } else {
        printf("✗ USB keyboard initialization failed: %s\n", esp_err_to_name(ret));
        printf("  Continuing without USB keyboard support\n");
    }
    
    // Initialize WiFi with ESP32-C6 coprocessor (SDIO)
    printf("Initializing WiFi (ESP32-C6 via SDIO)...\n");
    
    // Initialize network interface
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    
    // Register WiFi event handler
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));
    
    // Initialize WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret == ESP_OK) {
        ret = esp_wifi_set_mode(WIFI_MODE_STA);
        if (ret == ESP_OK) {
            ret = esp_wifi_start();  // Start WiFi
            if (ret == ESP_OK) {
                wifi_initialized = true;
                printf("✓ WiFi initialized and started via SDIO\n");
                
                // Auto-connect to default WiFi network
                printf("Connecting to WiFi (doc007)...\n");
                wifi_config_t wifi_config = {0};
                strcpy((char *)wifi_config.sta.ssid, "doc007");
                strcpy((char *)wifi_config.sta.password, "16221699");
                wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
                wifi_config.sta.pmf_cfg.capable = true;
                wifi_config.sta.pmf_cfg.required = false;
                
                ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
                esp_err_t conn_ret = esp_wifi_connect();
                if (conn_ret == ESP_OK) {
                    printf("✓ WiFi connection initiated\n");
                    printf("  FTP server will auto-start when connected\n");
                } else {
                    printf("✗ WiFi connect failed: %s\n", esp_err_to_name(conn_ret));
                    printf("  Use 'connect <ssid> <password>' to retry\n");
                }
            } else {
                printf("✗ WiFi start failed: %s\n", esp_err_to_name(ret));
            }
        } else {
            printf("✗ WiFi mode set failed: %s\n", esp_err_to_name(ret));
        }
    } else {
        printf("✗ WiFi init failed: %s\n", esp_err_to_name(ret));
        printf("  WiFi commands will not be available\n");
    }
    
    // Initialize Bluetooth with ESP32-C6 coprocessor (SDIO)
    printf("Initializing Bluetooth (ESP32-C6 via SDIO)...\n");
    ret = nimble_port_init();
    if (ret == ESP_OK) {
        // Configure NimBLE host
        ble_hs_cfg.sync_cb = NULL;
        ble_hs_cfg.reset_cb = NULL;
        
        // Start NimBLE host task
        nimble_port_freertos_init(nimble_host_task);
        
        bt_initialized = true;
        printf("✓ Bluetooth initialized (BLE via NimBLE)\n");
        printf("  Use 'btscan' to discover devices, 'btlist' to show results\n");
    } else {
        printf("✗ Bluetooth init failed: %s\n", esp_err_to_name(ret));
        printf("  Bluetooth commands will not be available\n");
    }
    
    // Show boot splash on LCD (do this BEFORE other initializations)
    show_boot_splash();
    
    // Silent boot - only show critical errors
    ret = m68k_init();
    if (ret != ESP_OK) {
        printf("CPU init failed: %s\n", esp_err_to_name(ret));
    }
    
    // Quick SD card mount (single attempt, no verbose output)
    vTaskDelay(pdMS_TO_TICKS(200));
    ret = mount_sd_card();
    
    // Load command history from SD card
    if (ret == ESP_OK) {
        load_history();
        
        // Try to auto-boot OS.bin from SD card
        // If OS boots successfully, it takes over and we skip the BIOS shell
        bool os_booted = try_autoboot();
        if (os_booted) {
            // OS is running in background task, keep alive but don't show BIOS shell
            printf("M68K-OS is running. UART available for monitoring.\n");
            printf("Reset to return to BIOS.\n\n");
            while (true) {
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
    }
    
    // Clear splash and show ready prompt on LCD (only if OS didn't boot)
    // Guard: if video card is running, use it instead of lcd_console to avoid SPI contention
    if (video_card_is_initialized()) {
        video_card_text_clear();
        video_card_print("M68K BIOS READY\n");
        video_card_print("================\n\n");
        char buf[64];
        snprintf(buf, sizeof(buf), "CPU: 68000 @ 60MHz\n");
        video_card_print(buf);
        snprintf(buf, sizeof(buf), "RAM: %lu KB\n", 
                 (unsigned long)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));
        video_card_print(buf);
        if (sd_mounted) {
            video_card_print("SD:  Mounted\n");
        }
        video_card_print("\nType 'help' for commands\n\n");
    } else if (lcd_console_is_initialized()) {
        lcd_console_clear();
        lcd_console_print("M68K BIOS READY\n");
        lcd_console_print("================\n\n");
        
        // Show minimal system info on LCD
        char buf[64];
        snprintf(buf, sizeof(buf), "CPU: 68000 @ 60MHz\n");
        lcd_console_print(buf);
        snprintf(buf, sizeof(buf), "RAM: %lu KB\n", 
                 (unsigned long)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));
        lcd_console_print(buf);
        if (sd_mounted) {
            lcd_console_print("SD:  Mounted\n");
        }
        lcd_console_print("\nType 'help' for commands\n\n");
    }
    
    // UART shows minimal boot info
    printf("\n");
    printf("M68K BIOS v1.0\n");
    printf("Ready. Type 'help' for commands.\n");
    printf("\n");
    
    // Dynamic prompt showing current directory
    char prompt[300];
    
    // Use custom input that reads from both UART and PS/2
    while (true) {
        // Build prompt with current directory (bash-style)
        const char *dir = current_working_dir;
        // Show shortened path if in /sdcard
        if (strncmp(dir, "/sdcard", 7) == 0) {
            if (strlen(dir) == 7) {
                strncpy(prompt, "m68k:~$ ", sizeof(prompt) - 1);
            } else {
                snprintf(prompt, sizeof(prompt), "m68k:~%s$ ", dir + 7);
            }
        } else {
            snprintf(prompt, sizeof(prompt), "m68k:%s$ ", dir);
        }
        prompt[sizeof(prompt) - 1] = '\0';
        
        char* line = read_line_dual_input(prompt);
        if (line == NULL) {
            continue;
        }
        
        if (strlen(line) > 0) {
            linenoiseHistoryAdd(line);
            append_to_history_file(line);
            
            // Add to our local history for arrow key navigation
            if (cmd_history_count < MAX_HISTORY_LINES) {
                cmd_history[cmd_history_count] = strdup(line);
                if (cmd_history[cmd_history_count]) {
                    cmd_history_count++;
                }
            } else {
                // History full, shift and replace oldest
                free(cmd_history[0]);
                for (int i = 0; i < MAX_HISTORY_LINES - 1; i++) {
                    cmd_history[i] = cmd_history[i + 1];
                }
                cmd_history[MAX_HISTORY_LINES - 1] = strdup(line);
            }
        }
        
        int cmd_ret;
        esp_err_t err = esp_console_run(line, &cmd_ret);
        if (err == ESP_ERR_NOT_FOUND) {
            console_printf("Unknown command\n");
        } else if (err == ESP_ERR_INVALID_ARG) {
            // Command was empty
        } else if (err == ESP_OK && cmd_ret != ESP_OK) {
            console_printf("Error: 0x%x (%s)\n", cmd_ret, esp_err_to_name(cmd_ret));
        } else if (err != ESP_OK) {
            console_printf("Internal error: %s\n", esp_err_to_name(err));
        }
        
        free(line);
    }
}
