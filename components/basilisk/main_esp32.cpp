/*
 *  main_esp32.cpp - Main program entry point for ESP-IDF
 *  BasiliskII ESP32-P4 Port
 *
 *  Ported from Arduino/M5Stack to ESP-IDF native APIs.
 *  Runs CPU emulation on a dedicated FreeRTOS task.
 */

#include "sysdeps.h"

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "cpu_emulation.h"
#include "sys.h"
#include "rom_patches.h"
#include "xpram.h"
#include "timer.h"
#include "video.h"
#include "prefs.h"
#include "prefs_items.h"
#include "main.h"
#include "macos_util.h"
#include "user_strings.h"
#include "input.h"

#define DEBUG 1
#include "debug.h"

static const char *TAG = "B2_MAIN";

// ROM file size limits
static const uint32 ROM_MIN_SIZE = 64 * 1024;     // 64KB minimum
static const uint32 ROM_MAX_SIZE = 1024 * 1024;   // 1MB maximum

// Memory pointers (declared in basilisk_glue.cpp)
extern uint32 RAMBaseMac;
extern uint8 *RAMBaseHost;
extern uint32 RAMSize;
extern uint32 ROMBaseMac;
extern uint8 *ROMBaseHost;
extern uint32 ROMSize;

// Frame buffer pointers (defined in video_esp32.cpp)
extern uint8 *MacFrameBaseHost;
extern uint32 MacFrameSize;
extern int MacFrameLayout;

// CPU and FPU type
int CPUType = 4;            // 68040 (Quadra 900)
bool CPUIs68060 = false;
int FPUType = 0;            // No FPU
bool TwentyFourBitAddressing = false;

// Interrupt flags
uint32 InterruptFlags = 0;

// vde switch (unused, but declared in main.h)
char *vde_sock = NULL;

// Forward declarations
void basilisk_loop(void);
static bool InitEmulator(void);
static void RunEmulator(void);

// CPU tick counter for timing (used by newcpu.cpp)
int32 emulated_ticks = 80000;
static int32 emulated_ticks_quantum = 80000;

// ============================================================================
// IPS (Instructions Per Second) Monitoring
// ============================================================================
static volatile uint64_t ips_total_instructions = 0;
static volatile uint64_t ips_last_instructions = 0;
static volatile uint32_t ips_last_report_time = 0;
static volatile uint32_t ips_current = 0;
#define IPS_REPORT_INTERVAL_MS 5000

/*
 *  CPU tick check - called periodically during emulation
 */
void cpu_do_check_ticks(void)
{
    ips_total_instructions += emulated_ticks_quantum;
    basilisk_loop();
    emulated_ticks = emulated_ticks_quantum;
}

/*
 *  Report IPS statistics
 */
static void reportIPSStats(uint32 current_time)
{
    if (current_time - ips_last_report_time >= IPS_REPORT_INTERVAL_MS) {
        uint64_t instructions_delta = ips_total_instructions - ips_last_instructions;
        uint32_t time_delta_ms = current_time - ips_last_report_time;

        if (time_delta_ms > 0) {
            ips_current = (uint32_t)((instructions_delta * 1000ULL) / time_delta_ms);
            float mips = ips_current / 1000000.0f;
            ESP_LOGI(TAG, "IPS: %u (%.2f MIPS), total: %llu",
                     ips_current, mips, ips_total_instructions);
        }

        ips_last_instructions = ips_total_instructions;
        ips_last_report_time = current_time;
    }
}

/*
 *  Get current IPS measurement
 */
uint32_t getEmulatorIPS(void)   { return ips_current; }
uint64_t getEmulatorTotalInstructions(void) { return ips_total_instructions; }

// ============================================================================
// Global emulator state
// ============================================================================
static bool emulator_running = false;
static uint32 last_60hz_time = 0;
static uint32 last_second_time = 0;
static uint32 last_video_signal = 0;
static uint32 last_disk_flush_time = 0;

#define VIDEO_SIGNAL_INTERVAL 42   // ~24 FPS
#define DISK_FLUSH_INTERVAL   2000 // 2 seconds

// FreeRTOS task handle for emulator
static TaskHandle_t emu_task_handle = NULL;

// ============================================================================
// Interrupt flag helpers (atomic)
// ============================================================================

void SetInterruptFlag(uint32 flag)
{
    __atomic_or_fetch(&InterruptFlags, flag, __ATOMIC_SEQ_CST);
}

void ClearInterruptFlag(uint32 flag)
{
    __atomic_and_fetch(&InterruptFlags, ~flag, __ATOMIC_SEQ_CST);
}

// ============================================================================
// 60Hz / 1Hz tick
// ============================================================================

static void handle_60hz_tick(void)
{
    SetInterruptFlag(INTFLAG_60HZ);
    SetInterruptFlag(INTFLAG_ADB);
    TriggerInterrupt();
}

static void handle_1hz_tick(void)
{
    SetInterruptFlag(INTFLAG_1HZ);
    TriggerInterrupt();
}

// ============================================================================
// Mutex functions (FreeRTOS)
// ============================================================================

B2_mutex *B2_create_mutex(void)
{
    B2_mutex *m = new B2_mutex;
    if (m) {
        m->sem = xSemaphoreCreateMutex();
        if (!m->sem) {
            ESP_LOGE(TAG, "Failed to create mutex");
            delete m;
            return NULL;
        }
    }
    return m;
}

void B2_lock_mutex(B2_mutex *mutex)
{
    if (mutex && mutex->sem)
        xSemaphoreTake(mutex->sem, portMAX_DELAY);
}

void B2_unlock_mutex(B2_mutex *mutex)
{
    if (mutex && mutex->sem)
        xSemaphoreGive(mutex->sem);
}

void B2_delete_mutex(B2_mutex *mutex)
{
    if (mutex) {
        if (mutex->sem) vSemaphoreDelete(mutex->sem);
        delete mutex;
    }
}

// ============================================================================
// Code cache flush (no-op for interpreter)
// ============================================================================

void FlushCodeCache(void *start, uint32 size)
{
    UNUSED(start);
    UNUSED(size);
}

// ============================================================================
// Alert functions
// ============================================================================

void ErrorAlert(const char *text)
{
    ESP_LOGE(TAG, "ERROR: %s", text);
}

void ErrorAlert(int string_id)
{
    ErrorAlert(GetString(string_id));
}

void WarningAlert(const char *text)
{
    ESP_LOGW(TAG, "WARNING: %s", text);
}

void WarningAlert(int string_id)
{
    WarningAlert(GetString(string_id));
}

bool ChoiceAlert(const char *text, const char *pos, const char *neg)
{
    ESP_LOGI(TAG, "CHOICE: %s (%s/%s)", text, pos, neg);
    return true;  // Always choose positive on embedded
}

// ============================================================================
// Quit emulator
// ============================================================================

void QuitEmulator(void)
{
    ESP_LOGI(TAG, "QuitEmulator called");
    emulator_running = false;
}

// ============================================================================
// ROM loading (POSIX FILE I/O)
// ============================================================================

static bool LoadROM(const char *rom_path)
{
    ESP_LOGI(TAG, "Loading ROM from: %s", rom_path);

    FILE *f = fopen(rom_path, "rb");
    if (!f) {
        ESP_LOGE(TAG, "Cannot open ROM file: %s", rom_path);
        return false;
    }

    // Get file size
    fseek(f, 0, SEEK_END);
    size_t rom_size = ftell(f);
    fseek(f, 0, SEEK_SET);

    ESP_LOGI(TAG, "ROM file size: %d bytes", (int)rom_size);

    if (rom_size < ROM_MIN_SIZE || rom_size > ROM_MAX_SIZE) {
        ESP_LOGE(TAG, "Invalid ROM size (expected %d-%d bytes)", ROM_MIN_SIZE, ROM_MAX_SIZE);
        fclose(f);
        return false;
    }

    // Round up to nearest 64KB
    ROMSize = (rom_size + 0xFFFF) & ~0xFFFF;

    // Allocate ROM buffer in PSRAM
    ROMBaseHost = (uint8 *)heap_caps_malloc(ROMSize, MALLOC_CAP_SPIRAM);
    if (!ROMBaseHost) {
        ESP_LOGE(TAG, "Cannot allocate ROM buffer (%d bytes)", ROMSize);
        fclose(f);
        return false;
    }

    memset(ROMBaseHost, 0, ROMSize);

    // Read ROM
    size_t bytes_read = fread(ROMBaseHost, 1, rom_size, f);
    fclose(f);

    if (bytes_read != rom_size) {
        ESP_LOGE(TAG, "ROM read failed (got %d, expected %d)", (int)bytes_read, (int)rom_size);
        free(ROMBaseHost);
        ROMBaseHost = NULL;
        return false;
    }

    ESP_LOGI(TAG, "ROM loaded at %p (%d bytes)", ROMBaseHost, ROMSize);

    // Print first 16 bytes
    char hex[64];
    char *p = hex;
    for (int i = 0; i < 16 && i < (int)rom_size; i++) {
        p += sprintf(p, "%02X ", ROMBaseHost[i]);
    }
    ESP_LOGI(TAG, "ROM header: %s", hex);

    return true;
}

// ============================================================================
// RAM allocation
// ============================================================================

static bool AllocateRAM(void)
{
    RAMSize = PrefsFindInt32("ramsize");
    if (RAMSize < 1024 * 1024) {
        RAMSize = 8 * 1024 * 1024;  // Default 8MB
    }

    ESP_LOGI(TAG, "Allocating %d bytes for Mac RAM...", RAMSize);

    RAMBaseHost = (uint8 *)heap_caps_malloc(RAMSize, MALLOC_CAP_SPIRAM);
    if (!RAMBaseHost) {
        ESP_LOGE(TAG, "Cannot allocate Mac RAM in PSRAM!");
        return false;
    }

    memset(RAMBaseHost, 0, RAMSize);
    ESP_LOGI(TAG, "Mac RAM at %p (%d bytes)", RAMBaseHost, RAMSize);
    return true;
}

// ============================================================================
// Initialize emulator
// ============================================================================

static bool InitEmulator(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  BasiliskII ESP32-P4 - Mac Emulator");
    ESP_LOGI(TAG, "========================================");

    // Memory info
    ESP_LOGI(TAG, "Free heap: %d bytes", (int)esp_get_free_heap_size());
    ESP_LOGI(TAG, "Free PSRAM: %d bytes",
             (int)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    size_t free_internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    size_t largest_internal = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    ESP_LOGI(TAG, "Internal SRAM: %d free, largest block: %d",
             (int)free_internal, (int)largest_internal);

    // Reserve opcode dispatch table early
    extern bool ReserveCpuFuncTable(void);
    ReserveCpuFuncTable();

    // Initialize preferences
    int dummy_argc = 0;
    char *dummy_argv_data[] = { NULL };
    char **dummy_argv = dummy_argv_data;
    PrefsInit(NULL, dummy_argc, dummy_argv);

    // Initialize system I/O
    SysInit();

    // Allocate Mac RAM
    if (!AllocateRAM()) {
        ErrorAlert("Failed to allocate Mac RAM");
        return false;
    }

    // Load ROM file
    const char *rom_path = PrefsFindString("rom");
    if (!rom_path) {
        rom_path = "/sdcard/mac/Q650.ROM";
    }

    if (!LoadROM(rom_path)) {
        ErrorAlert("Failed to load ROM file");
        return false;
    }

    // Initialize all emulator subsystems
    ESP_LOGI(TAG, "Calling InitAll()...");
    if (!InitAll(NULL)) {
        ErrorAlert("InitAll() failed");
        return false;
    }

    // Initialize input
    if (!InputInit()) {
        ESP_LOGW(TAG, "Input init failed (non-fatal)");
    }

    ESP_LOGI(TAG, "Emulator initialized successfully!");
    ESP_LOGI(TAG, "Tick quantum: %d instructions", emulated_ticks_quantum);

    // Memory status after init
    ESP_LOGI(TAG, "Free heap after init: %d bytes", (int)esp_get_free_heap_size());
    ESP_LOGI(TAG, "Free PSRAM after init: %d bytes",
             (int)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    return true;
}

// ============================================================================
// Run emulator (blocking - runs 68k CPU loop)
// ============================================================================

static void RunEmulator(void)
{
    ESP_LOGI(TAG, "Starting 68k CPU emulation...");

    emulator_running = true;
    uint32 now = millis();
    last_60hz_time = now;
    last_second_time = now;
    last_video_signal = now;
    last_disk_flush_time = now;
    ips_last_report_time = now;

    // This calls m68k_execute() which loops until QuitEmulator()
    Start680x0();

    ESP_LOGI(TAG, "68k CPU emulation ended");
}

// ============================================================================
// Emulator FreeRTOS task
// ============================================================================

static void basilisk_task(void *param)
{
    (void)param;

    if (!InitEmulator()) {
        ESP_LOGE(TAG, "Emulator initialization failed!");
        emulator_running = false;
        emu_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    RunEmulator();

    // Cleanup
    InputExit();
    ExitAll();
    SysExit();
    PrefsExit();

    ESP_LOGI(TAG, "BasiliskII shutdown complete");
    emulator_running = false;
    emu_task_handle = NULL;
    vTaskDelete(NULL);
}

// ============================================================================
// Main loop - called from CPU emulator periodically
// ============================================================================

void basilisk_loop(void)
{
    uint32 current_time = millis();

    // Flush dirty tiles for video
    VideoFlushDirtyTiles();

    // 60Hz tick (~16ms)
    while (current_time - last_60hz_time >= 16) {
        last_60hz_time += 16;
        handle_60hz_tick();
    }

    // 1Hz tick
    while (current_time - last_second_time >= 1000) {
        last_second_time += 1000;
        handle_1hz_tick();
    }

    // Signal video refresh
    if (current_time - last_video_signal >= VIDEO_SIGNAL_INTERVAL) {
        last_video_signal = current_time;
        VideoRefresh();
    }

    // Periodic disk flush
    if (current_time - last_disk_flush_time >= DISK_FLUSH_INTERVAL) {
        last_disk_flush_time = current_time;
        Sys_periodic_flush();
    }

    // Report IPS
    reportIPSStats(current_time);

    // Yield to allow other FreeRTOS tasks
    taskYIELD();
}

// ============================================================================
// Public C API (called from console commands in hello_world_main.c)
// ============================================================================

extern "C" {

bool basilisk_is_running(void)
{
    return emulator_running;
}

int basilisk_init_and_run(void)
{
    if (emulator_running || emu_task_handle != NULL) {
        ESP_LOGW(TAG, "Emulator already running");
        return -1;
    }

    // Create emulator task with large stack (CPU emulation is stack-heavy)
    BaseType_t ret = xTaskCreatePinnedToCore(
        basilisk_task,
        "basilisk",
        32768,              // 32KB stack
        NULL,
        5,                  // Normal priority
        &emu_task_handle,
        1                   // Pin to core 1
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create emulator task");
        return -1;
    }

    ESP_LOGI(TAG, "Emulator task started on core 1");
    return 0;
}

void basilisk_stop(void)
{
    if (emulator_running) {
        QuitEmulator();
        // Wait for task to finish
        int timeout = 50;  // 5 seconds
        while (emu_task_handle != NULL && timeout-- > 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        if (emu_task_handle != NULL) {
            ESP_LOGW(TAG, "Force-deleting emulator task");
            vTaskDelete(emu_task_handle);
            emu_task_handle = NULL;
        }
    }
}

uint32_t basilisk_get_ips(void)
{
    return ips_current;
}

} // extern "C"
