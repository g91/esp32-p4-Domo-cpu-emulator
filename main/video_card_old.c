/*
 * Video Card / GPU Device Implementation
 * 2D engine with framebuffer, sprites, tilemaps
 * Register interface for bus controller
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
typedef uint8_t (*emu_read_fn)(uint32_t addr);
static emu_read_fn s_emu_read = NULL;

// ============================================================================
// Video card state
// ============================================================================

static struct {
    bool initialized;
    bool enabled;

    // Framebuffer
    uint16_t *front_buf;     // Display buffer (SPIRAM)
    uint16_t *back_buf;      // Drawing buffer (SPIRAM)
    uint16_t *dma_buf;       // DMA bounce buffer (internal RAM)
    uint16_t  width;
    uint16_t  height;
    uint8_t   bpp;           // Bits per pixel (8 or 16)
    uint32_t  pitch;         // Bytes per scanline
    uint8_t   mode;          // Current video mode

    // Colors
    uint16_t  fg_color;
    uint16_t  bg_color;
    uint16_t  draw_color;

    // Cursor (text mode)
    uint16_t  cursor_x;
    uint16_t  cursor_y;

    // Drawing params (set via registers before command)
    int16_t   draw_x, draw_y;
    int16_t   draw_w, draw_h;
    int16_t   src_x, src_y;
    uint32_t  fb_addr;       // Framebuffer base in emulator RAM
    uint32_t  font_addr;     // Font data in emulator RAM
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
    uint8_t   d3d_shade_mode;     // 0=flat, 1=Gouraud
    uint32_t  d3d_vbuf_addr;
    uint32_t  d3d_vbuf_count;
    uint32_t  d3d_ibuf_addr;
    uint32_t  d3d_ibuf_count;
    int32_t   d3d_viewport_x, d3d_viewport_y;
    int32_t   d3d_viewport_w, d3d_viewport_h;
    fixed16_t d3d_light_x, d3d_light_y, d3d_light_z;
    uint16_t  d3d_light_color;
    uint16_t  d3d_ambient;

    // Display refresh
    TaskHandle_t refresh_task;
    uint32_t  frame_count;
    uint8_t   refresh_rate;  // Hz
    bool      double_buffer;
    bool      vblank;

    // Command parameters (PARAM0-PARAM5)
    uint32_t  params[6];

    SemaphoreHandle_t mutex;
} s_vid;

// DMA strip height for LCD transfer
#define DMA_STRIP_HEIGHT  8
#define DMA_BUF_SIZE      (VIDEO_MAX_WIDTH * DMA_STRIP_HEIGHT * 2)

// Forward declarations
static void exec_3d_command(uint32_t cmd);

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
    // Bresenham's line algorithm
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

static void draw_rect_outline(int x, int y, int w, int h, uint16_t color) {
    draw_hline(x, y, w, color);
    draw_hline(x, y + h - 1, w, color);
    draw_vline(x, y, h, color);
    draw_vline(x + w - 1, y, h, color);
}

static void draw_circle(int cx, int cy, int r, uint16_t color, bool filled) {
    // Midpoint circle algorithm
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

static void render_sprites(void) {
    // Render sprites back-to-front (highest priority number first)
    for (int pri = 255; pri >= 0; pri--) {
        for (int i = 0; i < VIDEO_MAX_SPRITES; i++) {
            video_sprite_t *sp = &s_vid.sprites[i];
            if (!(sp->flags & SPRITE_FLAG_VISIBLE)) continue;
            if (sp->priority != pri) continue;
            if (!s_emu_read) continue;

            for (int sy = 0; sy < sp->h; sy++) {
                for (int sx = 0; sx < sp->w; sx++) {
                    // Calculate source coordinates with flip
                    int src_x = (sp->flags & SPRITE_FLAG_FLIP_H) ? (sp->w - 1 - sx) : sx;
                    int src_y = (sp->flags & SPRITE_FLAG_FLIP_V) ? (sp->h - 1 - sy) : sy;

                    uint16_t color;
                    if (sp->flags & SPRITE_FLAG_PALETTE) {
                        // 8-bit indexed mode
                        uint8_t idx = s_emu_read(sp->data_addr + src_y * sp->w + src_x);
                        color = s_vid.palette[idx];
                    } else {
                        // RGB565 direct color
                        uint32_t offset = (src_y * sp->w + src_x) * 2;
                        uint8_t lo = s_emu_read(sp->data_addr + offset);
                        uint8_t hi = s_emu_read(sp->data_addr + offset + 1);
                        color = (uint16_t)(lo | (hi << 8));
                    }

                    // Transparency key check
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

            // Read tile index from tilemap (16-bit per entry)
            uint32_t map_offset = (map_y * tm->map_w + map_x) * 2;
            uint8_t lo = s_emu_read(tm->map_addr + map_offset);
            uint8_t hi = s_emu_read(tm->map_addr + map_offset + 1);
            uint16_t tile_idx = (uint16_t)(lo | (hi << 8));

            // Calculate tile pixel data address
            uint32_t tile_data = tm->set_addr + tile_idx * ts * ts * 2;

            // Draw tile
            int scr_x = tx * ts - offset_x;
            int scr_y = ty * ts - offset_y;

            for (int py = 0; py < ts; py++) {
                for (int px = 0; px < ts; px++) {
                    uint32_t pix_offset = (py * ts + px) * 2;
                    uint8_t plo = s_emu_read(tile_data + pix_offset);
                    uint8_t phi = s_emu_read(tile_data + pix_offset + 1);
                    uint16_t color = (uint16_t)(plo | (phi << 8));
                    if (color != 0) {  // Color 0 = transparent for tiles
                        put_pixel(scr_x + px, scr_y + py, color);
                    }
                }
            }
        }
    }
}

// ============================================================================
// Display refresh task
// ============================================================================

static void video_refresh_task(void *arg) {
    while (s_vid.initialized) {
        if (s_vid.enabled && s_vid.front_buf && s_vid.dma_buf) {
            // Hold mutex for entire LCD transfer - prevents FLIP from swapping
            // buffers mid-transfer (which would cause tearing).
            // FLIP will block here, giving natural vsync behavior.
            if (s_vid.mutex && xSemaphoreTake(s_vid.mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                s_vid.vblank = false;

                // Copy front buffer to LCD in strips via DMA bounce buffer
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
// Command execution
// ============================================================================

static void exec_command(uint32_t cmd) {
    if (!s_vid.back_buf) return;

    switch (cmd) {
    case VID_CMD_CLEAR:
        for (int i = 0; i < s_vid.width * s_vid.height; i++) {
            s_vid.back_buf[i] = s_vid.bg_color;
        }
        break;

    case VID_CMD_FLIP:
        if (s_vid.double_buffer && s_vid.mutex) {
            // Take mutex - blocks until LCD refresh finishes reading front_buf.
            // This prevents tearing by ensuring we never swap mid-transfer.
            xSemaphoreTake(s_vid.mutex, portMAX_DELAY);
            uint16_t *tmp = s_vid.front_buf;
            s_vid.front_buf = s_vid.back_buf;
            s_vid.back_buf = tmp;
            xSemaphoreGive(s_vid.mutex);
        } else if (!s_vid.double_buffer) {
            // Single buffer: copy back to front
            memcpy(s_vid.front_buf, s_vid.back_buf,
                   s_vid.width * s_vid.height * sizeof(uint16_t));
        }
        break;

    case VID_CMD_DRAW_PIXEL:
        put_pixel(s_vid.draw_x, s_vid.draw_y, s_vid.draw_color);
        break;

    case VID_CMD_DRAW_LINE:
        draw_line(s_vid.draw_x, s_vid.draw_y, s_vid.draw_w, s_vid.draw_h, s_vid.draw_color);
        break;

    case VID_CMD_FILL_RECT:
        fill_rect(s_vid.draw_x, s_vid.draw_y, s_vid.draw_w, s_vid.draw_h, s_vid.draw_color);
        break;

    case VID_CMD_HLINE:
        draw_hline(s_vid.draw_x, s_vid.draw_y, s_vid.draw_w, s_vid.draw_color);
        break;

    case VID_CMD_VLINE:
        draw_vline(s_vid.draw_x, s_vid.draw_y, s_vid.draw_h, s_vid.draw_color);
        break;

    case VID_CMD_CIRCLE:
        draw_circle(s_vid.draw_x, s_vid.draw_y, s_vid.draw_w, s_vid.draw_color, false);
        break;

    case VID_CMD_FILL_CIRCLE:
        draw_circle(s_vid.draw_x, s_vid.draw_y, s_vid.draw_w, s_vid.draw_color, true);
        break;

    case VID_CMD_BLIT:
        // Blit from emulator RAM to framebuffer
        if (s_emu_read) {
            uint32_t src_addr = s_vid.fb_addr; // Source address in emulator RAM
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
        // Index in draw_x, RGB565 color in draw_color
        uint16_t idx = s_vid.draw_x & 0xFF;
        s_vid.palette[idx] = s_vid.draw_color;
        break;
    }

    case VID_CMD_SCROLL:
        // Scroll framebuffer up by scroll_y pixels
        if (s_vid.scroll_y > 0 && s_vid.scroll_y < s_vid.height) {
            int rows_to_copy = s_vid.height - s_vid.scroll_y;
            memmove(s_vid.back_buf,
                    &s_vid.back_buf[s_vid.scroll_y * s_vid.width],
                    rows_to_copy * s_vid.width * sizeof(uint16_t));
            // Clear the newly exposed area
            for (int i = rows_to_copy * s_vid.width; i < s_vid.width * s_vid.height; i++) {
                s_vid.back_buf[i] = s_vid.bg_color;
            }
        }
        break;

    case VID_CMD_INIT: {
        // Initialize video mode based on VID_REG_MODE
        uint8_t mode = s_vid.mode;
        uint16_t w = s_vid.width, h = s_vid.height;

        switch (mode) {
        case VID_MODE_320x200x8:
            w = 320; h = 200; s_vid.bpp = 8;
            break;
        case VID_MODE_320x240x16:
            w = 320; h = 240; s_vid.bpp = 16;
            break;
        case VID_MODE_480x320x16:
            w = 480; h = 320; s_vid.bpp = 16;
            break;
        default:
            // Use current width/height
            break;
        }

        // Clamp to max
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
        ESP_LOGI(TAG, "Video mode set: %dx%d, %d bpp", w, h, s_vid.bpp);
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
        if (s_vid.mode >= VID_MODE_320x200x8) status |= VID_STATUS_GFX_MODE;
        return status;
    }
    case VID_REG_MODE:       return s_vid.mode;
    case VID_REG_WIDTH:      return s_vid.width;
    case VID_REG_HEIGHT:     return s_vid.height;
    case VID_REG_BPP:        return s_vid.bpp;
    case VID_REG_PITCH:      return s_vid.pitch;
    case VID_REG_FB_ADDR:    return s_vid.fb_addr;
    case VID_REG_FB_SIZE:    return s_vid.width * s_vid.height * 2;
    case VID_REG_CURSOR_X:   return s_vid.cursor_x;
    case VID_REG_CURSOR_Y:   return s_vid.cursor_y;
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

    // Sprite registers (0x300-0x6FF): 64 sprites x 16 bytes each
    // Map registers at offsets 0x300 + sprite_index * 16 + field_offset
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
        case 0x00: // TILE_CTRL
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
        case 0x00: // D3D_CTRL
            s_vid.d3d_enabled = (data & 0x01) != 0;
            s_vid.d3d_zbuffer = (data & 0x02) != 0;
            s_vid.d3d_backface_cull = (data & 0x04) != 0;
            s_vid.d3d_shade_mode = (data >> 4) & 0x03;
            break;
        case 0x04: // D3D_COMMAND
            exec_3d_command(data);
            break;
        case 0x08: s_vid.d3d_vbuf_addr = data; break;
        case 0x0C: s_vid.d3d_vbuf_count = data; break;
        case 0x10: s_vid.d3d_ibuf_addr = data; break;
        case 0x14: s_vid.d3d_ibuf_count = data; break;
        // Matrix registers at 0x18-0x57 handled separately
        case 0x58: s_vid.d3d_viewport_x = (int32_t)data; break;
        case 0x5C: s_vid.d3d_viewport_y = (int32_t)data; break;
        case 0x60: s_vid.d3d_viewport_w = (int32_t)data; break;
        case 0x64: s_vid.d3d_viewport_h = (int32_t)data; break;
        case 0x68: s_vid.d3d_light_x = (fixed16_t)data; break;
        case 0x6C: s_vid.d3d_light_y = (fixed16_t)data; break;
        case 0x70: s_vid.d3d_light_z = (fixed16_t)data; break;
        case 0x74: s_vid.d3d_light_color = (uint16_t)data; break;
        case 0x78: s_vid.d3d_ambient = (uint16_t)data; break;
        // Command parameter registers (for translate, rotate, scale, perspective)
        case 0x80: s_vid.params[0] = data; break;
        case 0x84: s_vid.params[1] = data; break;
        case 0x88: s_vid.params[2] = data; break;
        case 0x8C: s_vid.params[3] = data; break;
        case 0x90: s_vid.params[4] = data; break;
        case 0x94: s_vid.params[5] = data; break;
        default:
            // Matrix elements at 0x18 + (row*4+col)*4
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
        break;
    case VID_REG_CURSOR_Y:
        s_vid.cursor_y = (uint16_t)data;
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
    switch (cmd) {
    case 0x01: // Clear Z-buffer
        video_3d_clear_zbuffer();
        break;
    case 0x02: // Draw triangles from vertex/index buffers
        if (s_emu_read && s_vid.d3d_vbuf_count > 0 && s_vid.d3d_ibuf_count > 0) {
            // Read vertex and index data from emulator RAM
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
            // Each vertex: 6x fixed16_t (32-bit) + 1x uint16_t color + 1x uint16_t pad
            for (uint32_t i = 0; i < vcount; i++) {
                uint32_t addr = s_vid.d3d_vbuf_addr + i * sizeof(video_vertex_t);
                // Read 6 x 32-bit fixed-point fields (big-endian)
                int32_t *fields = (int32_t *)&verts[i];
                for (int f = 0; f < 6; f++) {
                    uint32_t a = addr + f * 4;
                    fields[f] = (int32_t)((s_emu_read(a) << 24) |
                                          (s_emu_read(a+1) << 16) |
                                          (s_emu_read(a+2) << 8) |
                                           s_emu_read(a+3));
                }
                // Read color (16-bit big-endian) at offset 24
                uint32_t ca = addr + 24;
                verts[i].color = (uint16_t)((s_emu_read(ca) << 8) | s_emu_read(ca+1));
                verts[i]._pad = 0;
            }

            // Read indices from emulator RAM (M68K = big-endian)
            for (uint32_t i = 0; i < icount; i++) {
                uint32_t addr = s_vid.d3d_ibuf_addr + i * 2;
                indices[i] = (uint16_t)((s_emu_read(addr) << 8) | s_emu_read(addr + 1));
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
    case 0x03: // Load identity matrix
        video_3d_identity();
        break;
    case 0x04: // Translate
        video_3d_translate(s_vid.params[0], s_vid.params[1], s_vid.params[2]);
        break;
    case 0x05: // Rotate X
        video_3d_rotate_x(s_vid.params[0]);
        break;
    case 0x06: // Rotate Y
        video_3d_rotate_y(s_vid.params[0]);
        break;
    case 0x07: // Rotate Z
        video_3d_rotate_z(s_vid.params[0]);
        break;
    case 0x08: // Scale
        video_3d_scale(s_vid.params[0], s_vid.params[1], s_vid.params[2]);
        break;
    case 0x09: // Perspective projection
        video_3d_perspective(s_vid.params[0], s_vid.params[1],
                            s_vid.params[2], s_vid.params[3]);
        break;
    }
}

// ============================================================================
// Public API
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

    // DMA bounce buffer in internal RAM
    s_vid.dma_buf = heap_caps_malloc(DMA_BUF_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!s_vid.dma_buf) {
        ESP_LOGW(TAG, "DMA buffer alloc failed, trying default");
        s_vid.dma_buf = heap_caps_malloc(DMA_BUF_SIZE, MALLOC_CAP_DEFAULT);
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
    s_vid.fg_color = 0xFFFF;  // White
    s_vid.bg_color = 0x0000;  // Black
    s_vid.refresh_rate = 30;
    s_vid.double_buffer = true;

    // Default palette (VGA-like)
    static const uint16_t default_pal[16] = {
        0x0000, 0x0015, 0x02A0, 0x02B5,  // Black, Blue, Green, Cyan
        0xA800, 0xA815, 0xAAA0, 0xAD55,  // Red, Magenta, Brown, LightGray
        0x52AA, 0x52BF, 0x57EA, 0x57FF,  // DarkGray, LightBlue, LightGreen, LightCyan
        0xFAAA, 0xFABF, 0xFFEA, 0xFFFF   // LightRed, LightMagenta, Yellow, White
    };
    memcpy(s_vid.palette, default_pal, sizeof(default_pal));
    // Fill rest with grayscale gradient
    for (int i = 16; i < 256; i++) {
        uint8_t gray = (i - 16) * 255 / 239;
        uint16_t r5 = (gray >> 3) & 0x1F;
        uint16_t g6 = (gray >> 2) & 0x3F;
        uint16_t b5 = (gray >> 3) & 0x1F;
        s_vid.palette[i] = (r5 << 11) | (g6 << 5) | b5;
    }

    // Initialize 3D engine
    video_3d_init(s_vid.back_buf, s_vid.width, s_vid.height);

    // Create mutex
    s_vid.mutex = xSemaphoreCreateMutex();

    s_vid.initialized = true;
    s_vid.enabled = true;  // Ready to display - refresh task will send front_buf to LCD

    // Start refresh task
    xTaskCreate(video_refresh_task, "vid_refresh", 4096, NULL, 5, &s_vid.refresh_task);

    ESP_LOGI(TAG, "Video card initialized: %dx%d, double-buffered, %dHz refresh",
             s_vid.width, s_vid.height, s_vid.refresh_rate);
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

// Set emulator memory read callback (called when active emulator changes)
void video_card_set_emu_read(emu_read_fn fn) {
    s_emu_read = fn;
}

// ============================================================================
// Direct framebuffer access for emulator renderers
// ============================================================================

uint16_t *video_card_get_back_buffer(void) {
    if (!s_vid.initialized) return NULL;
    return s_vid.back_buf;
}

void video_card_present(void) {
    if (!s_vid.initialized || !s_vid.double_buffer) return;
    if (s_vid.mutex) {
        // Block until LCD refresh finishes reading front_buf, then swap.
        // This gives natural vsync behavior - no tearing.
        xSemaphoreTake(s_vid.mutex, portMAX_DELAY);
        uint16_t *tmp = s_vid.front_buf;
        s_vid.front_buf = s_vid.back_buf;
        s_vid.back_buf = tmp;
        xSemaphoreGive(s_vid.mutex);
    }
}

uint16_t video_card_get_width(void) {
    return s_vid.initialized ? s_vid.width : 0;
}

uint16_t video_card_get_height(void) {
    return s_vid.initialized ? s_vid.height : 0;
}

// ============================================================================
// Text console overlay - renders 8x16 font glyphs directly onto GPU framebuffer
// ============================================================================

// 8x16 bitmap font (ASCII 32-126) - same as lcd_console
static const uint8_t vid_font_8x16[][16] = {
    // Space (32)
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    // ! (33)
    {0x00,0x00,0x18,0x3C,0x3C,0x3C,0x18,0x18,0x18,0x00,0x18,0x18,0x00,0x00,0x00,0x00},
    // " (34)
    {0x00,0x63,0x63,0x63,0x22,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    // # (35)
    {0x00,0x00,0x00,0x36,0x36,0x7F,0x36,0x36,0x36,0x7F,0x36,0x36,0x00,0x00,0x00,0x00},
    // $ (36)
    {0x0C,0x0C,0x3E,0x63,0x61,0x60,0x3E,0x03,0x03,0x43,0x63,0x3E,0x0C,0x0C,0x00,0x00},
    // % (37)
    {0x00,0x00,0x00,0x00,0x00,0x61,0x63,0x06,0x0C,0x18,0x33,0x63,0x00,0x00,0x00,0x00},
    // & (38)
    {0x00,0x00,0x00,0x1C,0x36,0x36,0x1C,0x3B,0x6E,0x66,0x66,0x3B,0x00,0x00,0x00,0x00},
    // ' (39)
    {0x00,0x30,0x30,0x30,0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    // ( (40)
    {0x00,0x00,0x0C,0x18,0x18,0x30,0x30,0x30,0x30,0x18,0x18,0x0C,0x00,0x00,0x00,0x00},
    // ) (41)
    {0x00,0x00,0x18,0x0C,0x0C,0x06,0x06,0x06,0x06,0x0C,0x0C,0x18,0x00,0x00,0x00,0x00},
    // * (42)
    {0x00,0x00,0x00,0x00,0x42,0x66,0x3C,0xFF,0x3C,0x66,0x42,0x00,0x00,0x00,0x00,0x00},
    // + (43)
    {0x00,0x00,0x00,0x00,0x18,0x18,0x18,0x7E,0x18,0x18,0x18,0x00,0x00,0x00,0x00,0x00},
    // , (44)
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x18,0x30,0x00,0x00},
    // - (45)
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFE,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    // . (46)
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00,0x00},
    // / (47)
    {0x00,0x00,0x00,0x00,0x02,0x06,0x0C,0x18,0x30,0x60,0xC0,0x80,0x00,0x00,0x00,0x00},
    // 0 (48)
    {0x00,0x00,0x3C,0x66,0xC3,0xC3,0xDB,0xDB,0xC3,0xC3,0x66,0x3C,0x00,0x00,0x00,0x00},
    // 1 (49)
    {0x00,0x00,0x18,0x38,0x78,0x18,0x18,0x18,0x18,0x18,0x18,0x7E,0x00,0x00,0x00,0x00},
    // 2 (50)
    {0x00,0x00,0x3C,0x66,0x06,0x06,0x0C,0x18,0x30,0x60,0x66,0x7E,0x00,0x00,0x00,0x00},
    // 3 (51)
    {0x00,0x00,0x3C,0x66,0x06,0x06,0x1C,0x06,0x06,0x06,0x66,0x3C,0x00,0x00,0x00,0x00},
    // 4 (52)
    {0x00,0x00,0x0C,0x1C,0x3C,0x6C,0xCC,0xFE,0x0C,0x0C,0x0C,0x1E,0x00,0x00,0x00,0x00},
    // 5 (53)
    {0x00,0x00,0x7E,0x60,0x60,0x60,0x7C,0x06,0x06,0x06,0x66,0x3C,0x00,0x00,0x00,0x00},
    // 6 (54)
    {0x00,0x00,0x1C,0x30,0x60,0x60,0x7C,0x66,0x66,0x66,0x66,0x3C,0x00,0x00,0x00,0x00},
    // 7 (55)
    {0x00,0x00,0x7E,0x66,0x06,0x0C,0x0C,0x18,0x18,0x18,0x18,0x18,0x00,0x00,0x00,0x00},
    // 8 (56)
    {0x00,0x00,0x3C,0x66,0x66,0x66,0x3C,0x66,0x66,0x66,0x66,0x3C,0x00,0x00,0x00,0x00},
    // 9 (57)
    {0x00,0x00,0x3C,0x66,0x66,0x66,0x3E,0x06,0x06,0x06,0x0C,0x38,0x00,0x00,0x00,0x00},
    // : (58)
    {0x00,0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00,0x00,0x00},
    // ; (59)
    {0x00,0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00,0x18,0x18,0x30,0x00,0x00,0x00,0x00},
    // < (60)
    {0x00,0x00,0x00,0x06,0x0C,0x18,0x30,0x60,0x30,0x18,0x0C,0x06,0x00,0x00,0x00,0x00},
    // = (61)
    {0x00,0x00,0x00,0x00,0x00,0x7E,0x00,0x00,0x7E,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    // > (62)
    {0x00,0x00,0x00,0x60,0x30,0x18,0x0C,0x06,0x0C,0x18,0x30,0x60,0x00,0x00,0x00,0x00},
    // ? (63)
    {0x00,0x00,0x3C,0x66,0x66,0x06,0x0C,0x18,0x18,0x00,0x18,0x18,0x00,0x00,0x00,0x00},
    // @ (64)
    {0x00,0x00,0x00,0x3C,0x42,0x5A,0x5A,0x5A,0x5C,0x40,0x40,0x3C,0x00,0x00,0x00,0x00},
    // A-Z (65-90)
    {0x00,0x00,0x18,0x3C,0x66,0x66,0x66,0x7E,0x66,0x66,0x66,0x66,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x7C,0x66,0x66,0x66,0x7C,0x66,0x66,0x66,0x66,0x7C,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x3C,0x66,0x66,0x60,0x60,0x60,0x60,0x66,0x66,0x3C,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x78,0x6C,0x66,0x66,0x66,0x66,0x66,0x66,0x6C,0x78,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x7E,0x60,0x60,0x60,0x78,0x60,0x60,0x60,0x60,0x7E,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x7E,0x60,0x60,0x60,0x78,0x60,0x60,0x60,0x60,0x60,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x3C,0x66,0x66,0x60,0x60,0x6E,0x66,0x66,0x66,0x3E,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x66,0x66,0x66,0x66,0x7E,0x66,0x66,0x66,0x66,0x66,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x3C,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x3C,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x1E,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x6C,0x6C,0x38,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x66,0x66,0x6C,0x6C,0x78,0x78,0x6C,0x6C,0x66,0x66,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x60,0x60,0x60,0x60,0x60,0x60,0x60,0x60,0x60,0x7E,0x00,0x00,0x00,0x00},
    {0x00,0x00,0xC6,0xEE,0xFE,0xFE,0xD6,0xC6,0xC6,0xC6,0xC6,0xC6,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x66,0x66,0x76,0x76,0x7E,0x6E,0x6E,0x66,0x66,0x66,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x3C,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x3C,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x7C,0x66,0x66,0x66,0x66,0x7C,0x60,0x60,0x60,0x60,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x3C,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x6E,0x3C,0x06,0x00,0x00,0x00},
    {0x00,0x00,0x7C,0x66,0x66,0x66,0x7C,0x78,0x6C,0x6C,0x66,0x66,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x3C,0x66,0x60,0x60,0x3C,0x06,0x06,0x06,0x66,0x3C,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x7E,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x3C,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x3C,0x3C,0x18,0x00,0x00,0x00,0x00},
    {0x00,0x00,0xC6,0xC6,0xC6,0xC6,0xC6,0xD6,0xFE,0xEE,0xC6,0xC6,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x66,0x66,0x66,0x3C,0x18,0x3C,0x66,0x66,0x66,0x66,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x66,0x66,0x66,0x66,0x3C,0x18,0x18,0x18,0x18,0x18,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x7E,0x06,0x06,0x0C,0x18,0x30,0x60,0x60,0x60,0x7E,0x00,0x00,0x00,0x00},
    // [ \ ] ^ _ ` (91-96)
    {0x00,0x00,0x3C,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x3C,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x80,0xC0,0x60,0x30,0x18,0x0C,0x06,0x02,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x3C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x3C,0x00,0x00,0x00,0x00},
    {0x18,0x3C,0x66,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00},
    {0x30,0x30,0x18,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    // a-z (97-122)
    {0x00,0x00,0x00,0x00,0x00,0x3C,0x06,0x3E,0x66,0x66,0x66,0x3E,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x60,0x60,0x60,0x7C,0x66,0x66,0x66,0x66,0x66,0x7C,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x3C,0x66,0x60,0x60,0x60,0x66,0x3C,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x06,0x06,0x06,0x3E,0x66,0x66,0x66,0x66,0x66,0x3E,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x3C,0x66,0x66,0x7E,0x60,0x60,0x3C,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x1C,0x36,0x30,0x30,0x7C,0x30,0x30,0x30,0x30,0x30,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x3E,0x66,0x66,0x66,0x66,0x3E,0x06,0x06,0x3C,0x00,0x00},
    {0x00,0x00,0x60,0x60,0x60,0x7C,0x66,0x66,0x66,0x66,0x66,0x66,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x18,0x18,0x00,0x38,0x18,0x18,0x18,0x18,0x18,0x3C,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x0C,0x0C,0x00,0x1C,0x0C,0x0C,0x0C,0x0C,0x0C,0x6C,0x6C,0x38,0x00,0x00},
    {0x00,0x00,0x60,0x60,0x60,0x66,0x6C,0x78,0x78,0x6C,0x66,0x66,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x38,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x3C,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x6C,0xFE,0xD6,0xD6,0xD6,0xC6,0xC6,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x7C,0x66,0x66,0x66,0x66,0x66,0x66,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x3C,0x66,0x66,0x66,0x66,0x66,0x3C,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x7C,0x66,0x66,0x66,0x66,0x7C,0x60,0x60,0x60,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x3E,0x66,0x66,0x66,0x66,0x3E,0x06,0x06,0x06,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x7C,0x66,0x66,0x60,0x60,0x60,0x60,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x3E,0x60,0x60,0x3C,0x06,0x06,0x7C,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x30,0x30,0x7C,0x30,0x30,0x30,0x30,0x36,0x1C,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x66,0x66,0x66,0x66,0x66,0x66,0x3E,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x66,0x66,0x66,0x66,0x3C,0x3C,0x18,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0xC6,0xC6,0xD6,0xD6,0xFE,0x6C,0x6C,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x66,0x66,0x3C,0x3C,0x66,0x66,0x66,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x66,0x66,0x66,0x66,0x66,0x3E,0x06,0x0C,0x78,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x7E,0x06,0x0C,0x18,0x30,0x60,0x7E,0x00,0x00,0x00,0x00},
    // { | } ~ (123-126)
    {0x00,0x00,0x0E,0x18,0x18,0x18,0x70,0x18,0x18,0x18,0x18,0x0E,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x18,0x18,0x18,0x18,0x18,0x00,0x18,0x18,0x18,0x18,0x18,0x00,0x00,0x00},
    {0x00,0x00,0x70,0x18,0x18,0x18,0x0E,0x18,0x18,0x18,0x18,0x70,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x3B,0x6E,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
};

// Text console state
#define VID_TEXT_FONT_W   8
#define VID_TEXT_FONT_H   16
#define VID_TEXT_COLS      (480 / VID_TEXT_FONT_W)    // 60
#define VID_TEXT_ROWS      (320 / VID_TEXT_FONT_H)    // 20

static struct {
    int col, row;           // Cursor position
    uint16_t fg, bg;        // Foreground/background colors (RGB565)
} s_text = {
    .col = 0, .row = 0,
    .fg = 0xFFFF,   // White
    .bg = 0x0000,   // Black
};

// Render a single glyph directly onto the FRONT buffer (like DOS video RAM)
// The 30fps refresh task continuously pushes front_buf to LCD - no swap needed
static void vid_draw_glyph(int px, int py, char ch, uint16_t fg, uint16_t bg) {
    if (!s_vid.initialized || !s_vid.front_buf) return;
    
    int ci = (int)ch - 32;
    if (ci < 0 || ci > 94) ci = 0;  // Default to space

    const uint8_t *glyph = vid_font_8x16[ci];
    uint16_t *fb = s_vid.front_buf;
    int w = s_vid.width;
    int h = s_vid.height;

    for (int row = 0; row < VID_TEXT_FONT_H; row++) {
        int y = py + row;
        if (y < 0 || y >= h) continue;
        uint8_t bits = glyph[row];
        for (int col = 0; col < VID_TEXT_FONT_W; col++) {
            int x = px + col;
            if (x < 0 || x >= w) continue;
            fb[y * w + x] = (bits & (0x80 >> col)) ? fg : bg;
        }
    }
}

// Scroll the front buffer up by one text row (16 pixels)
static void vid_text_scroll_up(void) {
    if (!s_vid.initialized || !s_vid.front_buf) return;

    uint16_t *fb = s_vid.front_buf;
    int w = s_vid.width;
    int h = s_vid.height;
    int row_pixels = VID_TEXT_FONT_H;
    
    // Move all rows up by 16 pixels
    int move_lines = h - row_pixels;
    if (move_lines > 0) {
        memmove(fb, fb + w * row_pixels, move_lines * w * sizeof(uint16_t));
    }
    
    // Clear the last row with background color
    uint16_t *last_row = fb + move_lines * w;
    int fill_pixels = row_pixels * w;
    for (int i = 0; i < fill_pixels; i++) {
        last_row[i] = s_text.bg;
    }
}

void video_card_putchar(char ch) {
    if (!s_vid.initialized || !s_vid.front_buf) return;

    if (ch == '\r') {
        s_text.col = 0;
        return;
    }
    
    if (ch == '\n') {
        s_text.col = 0;
        s_text.row++;
        if (s_text.row >= VID_TEXT_ROWS) {
            s_text.row = VID_TEXT_ROWS - 1;
            vid_text_scroll_up();
        }
        // No present needed - refresh task continuously pushes front_buf at 30fps
        return;
    }
    
    if (ch == '\b') {
        if (s_text.col > 0) {
            s_text.col--;
            // Erase character at cursor
            vid_draw_glyph(s_text.col * VID_TEXT_FONT_W, 
                          s_text.row * VID_TEXT_FONT_H, ' ', s_text.fg, s_text.bg);
        }
        return;
    }

    if (ch == '\t') {
        // Tab: advance to next 8-column boundary
        int next = (s_text.col + 8) & ~7;
        while (s_text.col < next && s_text.col < VID_TEXT_COLS) {
            vid_draw_glyph(s_text.col * VID_TEXT_FONT_W,
                          s_text.row * VID_TEXT_FONT_H, ' ', s_text.fg, s_text.bg);
            s_text.col++;
        }
        if (s_text.col >= VID_TEXT_COLS) {
            s_text.col = 0;
            s_text.row++;
            if (s_text.row >= VID_TEXT_ROWS) {
                s_text.row = VID_TEXT_ROWS - 1;
                vid_text_scroll_up();
            }
        }
        return;
    }

    // Printable character
    if (ch >= 32 && ch <= 126) {
        vid_draw_glyph(s_text.col * VID_TEXT_FONT_W,
                      s_text.row * VID_TEXT_FONT_H, ch, s_text.fg, s_text.bg);
        s_text.col++;
        if (s_text.col >= VID_TEXT_COLS) {
            s_text.col = 0;
            s_text.row++;
            if (s_text.row >= VID_TEXT_ROWS) {
                s_text.row = VID_TEXT_ROWS - 1;
                vid_text_scroll_up();
            }
        }
    }
}

void video_card_print(const char *str) {
    if (!str) return;
    while (*str) {
        video_card_putchar(*str++);
    }
    // No present needed - front_buf is live, refresh task pushes at 30fps
}

void video_card_text_clear(void) {
    if (!s_vid.initialized || !s_vid.front_buf) return;
    
    // Fill front buffer directly with background color
    int total = s_vid.width * s_vid.height;
    for (int i = 0; i < total; i++) {
        s_vid.front_buf[i] = s_text.bg;
    }
    s_text.col = 0;
    s_text.row = 0;
    // Refresh task will show cleared screen on next frame
}

void video_card_text_set_colors(uint16_t fg, uint16_t bg) {
    s_text.fg = fg;
    s_text.bg = bg;
}
