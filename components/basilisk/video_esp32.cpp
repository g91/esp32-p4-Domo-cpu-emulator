/*
 *  video_esp32.cpp - Video output for ESP-IDF using video_card API
 *  BasiliskII ESP32-P4 Port
 *
 *  Phase 1: Stub implementation - video output will be added in Phase 4
 */

#include "sysdeps.h"
#include "prefs.h"
#include "video.h"
#include "video_defs.h"

#define DEBUG 0
#include "debug.h"

static const char *TAG = "B2_VIDEO";

// Frame buffer in PSRAM
uint8 *MacFrameBaseHost = NULL;
uint32 MacFrameSize = 0;
int MacFrameLayout = FLAYOUT_DIRECT;

// Video mode info
static int display_width = 480;
static int display_height = 320;
static int display_depth = 8;  // bits per pixel

// Dirty tile tracking (placeholder)
static volatile uint32_t dirty_bitmap[4] = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF};

/*
 *  Video initialization
 */
bool VideoInit(bool classic)
{
    UNUSED(classic);
    
    // Parse screen preference
    const char *screen_pref = PrefsFindString("screen");
    if (screen_pref) {
        int w = 0, h = 0, d = 0;
        if (sscanf(screen_pref, "win/%d/%d/%d", &w, &h, &d) >= 2) {
            if (w > 0) display_width = w;
            if (h > 0) display_height = h;
            if (d > 0) display_depth = d;
        }
    }
    
    // Calculate frame buffer size
    int bytes_per_pixel = (display_depth + 7) / 8;
    if (display_depth == 1) bytes_per_pixel = 1;  // 1 bit = 1 byte for simplicity
    
    int row_bytes = display_width * bytes_per_pixel;
    if (display_depth == 1) {
        row_bytes = (display_width + 7) / 8;
    }
    
    MacFrameSize = row_bytes * display_height;
    
    // Allocate frame buffer in PSRAM
    MacFrameBaseHost = (uint8 *)heap_caps_malloc(MacFrameSize, MALLOC_CAP_SPIRAM);
    if (!MacFrameBaseHost) {
        ESP_LOGE(TAG, "Failed to allocate frame buffer (%d bytes)", MacFrameSize);
        return false;
    }
    
    memset(MacFrameBaseHost, 0, MacFrameSize);
    
    ESP_LOGI(TAG, "Video initialized: %dx%d @%d-bit, fb=%d bytes at %p",
             display_width, display_height, display_depth, MacFrameSize, MacFrameBaseHost);
    
    // Create video mode descriptor
    video_mode mode;
    mode.x = display_width;
    mode.y = display_height;
    mode.resolution_id = 0x80;
    
    switch (display_depth) {
        case 1:  mode.depth = VDEPTH_1BIT; break;
        case 2:  mode.depth = VDEPTH_2BIT; break;
        case 4:  mode.depth = VDEPTH_4BIT; break;
        case 8:  mode.depth = VDEPTH_8BIT; break;
        case 16: mode.depth = VDEPTH_16BIT; break;
        case 32: mode.depth = VDEPTH_32BIT; break;
        default: mode.depth = VDEPTH_8BIT; break;
    }
    mode.bytes_per_row = row_bytes;
    
    VideoModes.push_back(mode);
    
    // Set the video base address in Mac memory space
    // This is where the ROM/OS expects the framebuffer
    extern uint32 MacFrameBaseMac;
    (void)MacFrameBaseMac;  // Referenced via cpu_emulation.h
    
    return true;
}

/*
 *  Video deinitialization
 */
void VideoExit(void)
{
    if (MacFrameBaseHost) {
        free(MacFrameBaseHost);
        MacFrameBaseHost = NULL;
    }
    ESP_LOGI(TAG, "Video shutdown");
}

/*
 *  Set palette (stub for now)
 */
void video_set_palette(uint8 *pal, int num)
{
    UNUSED(pal);
    UNUSED(num);
}

/*
 *  Video refresh signal (called from main loop)
 *  Phase 4 will implement actual LCD rendering here
 */
void VideoRefresh(void)
{
    // TODO Phase 4: Render MacFrameBaseHost to video_card back buffer
}

/*
 *  Flush dirty tiles from CPU-side bitmap to shared bitmap
 */
void VideoFlushDirtyTiles(void)
{
    // TODO Phase 4: Tile-based dirty tracking
}

/*
 *  Video interrupt handler (called at 60Hz)
 */
void VideoInterrupt(void)
{
    // Signal that a new frame may be ready
}

/*
 *  Request video mode switch
 */
void video_switch_to_mode(const video_mode &mode)
{
    ESP_LOGI(TAG, "Mode switch request: %dx%d depth=%d", mode.x, mode.y, mode.depth);
}
