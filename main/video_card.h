/*
 * Video Card / GPU Device
 * Framebuffer-based video with 2D acceleration and 3D pipeline
 * Shared bus device accessible from all CPU emulators
 *
 * Register definitions are in bus_controller.h (VID_REG_*, VID_CMD_*)
 */

#ifndef VIDEO_CARD_H
#define VIDEO_CARD_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Maximum supported resolution (matches LCD)
#define VIDEO_MAX_WIDTH     480
#define VIDEO_MAX_HEIGHT    320

// Sprite engine
#define VIDEO_MAX_SPRITES   64
#define VIDEO_SPRITE_REGS   16  // Bytes per sprite in register space

// Tilemap engine
#define VIDEO_MAX_TILE_SIZE 32

// 3D engine
#define VIDEO_MAX_VERTICES  256
#define VIDEO_MAX_INDICES   768  // 256 triangles

// Fixed-point 16.16 math for 3D
typedef int32_t  fixed16_t;
#define FP_SHIFT    16
#define FP_ONE      (1 << FP_SHIFT)
#define FP_HALF     (1 << (FP_SHIFT - 1))
#define INT_TO_FP(x)  ((fixed16_t)(x) << FP_SHIFT)
#define FP_TO_INT(x)  ((x) >> FP_SHIFT)
#define FP_MUL(a, b)  ((fixed16_t)(((int64_t)(a) * (b)) >> FP_SHIFT))
#define FP_DIV(a, b)  ((fixed16_t)(((int64_t)(a) << FP_SHIFT) / (b)))

// Sprite structure (maps to register space at offset 0x300)
typedef struct {
    int16_t  x, y;          // Position
    uint16_t w, h;           // Size in pixels
    uint32_t data_addr;      // Pixel data address in emulator RAM
    uint8_t  flags;          // Visible, flip H/V, palette mode, transparency
    uint8_t  priority;       // Layer priority (0=front)
    uint16_t trans_color;    // Transparency key color (RGB565)
} video_sprite_t;

// Sprite flags
#define SPRITE_FLAG_VISIBLE     0x01
#define SPRITE_FLAG_FLIP_H      0x02
#define SPRITE_FLAG_FLIP_V      0x04
#define SPRITE_FLAG_PALETTE     0x08
#define SPRITE_FLAG_TRANS_KEY   0x10

// 3D vertex (in emulator RAM) - 36 bytes
typedef struct {
    fixed16_t x, y, z;      // Position
    fixed16_t nx, ny, nz;   // Normal
    uint16_t  color;         // Vertex color (RGB565)
    uint16_t  _pad;
    fixed16_t u, v;          // Texture coordinates (16.16 fixed-point, 0.0-1.0)
} video_vertex_t;

// 4x4 matrix (fixed-point)
typedef struct {
    fixed16_t m[4][4];
} video_matrix_t;

// Tilemap configuration
typedef struct {
    uint32_t map_addr;       // Tilemap data in emulator RAM
    uint32_t set_addr;       // Tileset pixel data in emulator RAM
    uint16_t map_w, map_h;   // Map dimensions in tiles
    uint16_t scroll_x, scroll_y;
    uint8_t  tile_size;      // 8, 16, or 32
    bool     enabled;
} video_tilemap_t;

/**
 * Initialize the video card device
 */
esp_err_t video_card_init(void);

/**
 * Deinitialize and free resources
 */
void video_card_deinit(void);

/**
 * Handle bus read from video card registers
 * @param reg Register offset within BUS_DEV_VIDEO
 * @param size Access size (1, 2, or 4 bytes)
 */
uint32_t video_card_read(uint32_t reg, uint8_t size);

/**
 * Handle bus write to video card registers
 * @param reg Register offset within BUS_DEV_VIDEO
 * @param data Data to write
 * @param size Access size (1, 2, or 4 bytes)
 */
void video_card_write(uint32_t reg, uint32_t data, uint8_t size);

/**
 * Check if video card is initialized
 */
bool video_card_is_initialized(void);

/**
 * Get current frame count
 */
uint32_t video_card_get_frame_count(void);

/**
 * Set emulator memory read callback (for sprite/tilemap/3D data access)
 */
typedef uint8_t (*video_emu_read_fn)(uint32_t addr);
void video_card_set_emu_read(video_emu_read_fn fn);

// ============================================================================
// 3D Pipeline (implemented in video_card_3d.c)
// ============================================================================

/**
 * Initialize 3D pipeline resources
 */
esp_err_t video_3d_init(uint16_t *framebuffer, uint16_t fb_width, uint16_t fb_height);

/**
 * Clean up 3D resources
 */
void video_3d_deinit(void);

/**
 * Clear the Z-buffer
 */
void video_3d_clear_zbuffer(void);

/**
 * Set the model-view-projection matrix
 */
void video_3d_set_matrix(const video_matrix_t *mat);

/**
 * Get pointer to current matrix for modification
 */
video_matrix_t *video_3d_get_matrix(void);

/**
 * Load identity matrix
 */
void video_3d_identity(void);

/**
 * Apply translation to current matrix
 */
void video_3d_translate(fixed16_t tx, fixed16_t ty, fixed16_t tz);

/**
 * Apply rotation around X/Y/Z axis (angle in fixed-point degrees)
 */
void video_3d_rotate_x(fixed16_t angle);
void video_3d_rotate_y(fixed16_t angle);
void video_3d_rotate_z(fixed16_t angle);

/**
 * Apply scaling to current matrix
 */
void video_3d_scale(fixed16_t sx, fixed16_t sy, fixed16_t sz);

/**
 * Set perspective projection
 */
void video_3d_perspective(fixed16_t fov, fixed16_t aspect, fixed16_t near, fixed16_t far);

/**
 * Transform and rasterize triangles
 * @param vertices Vertex array
 * @param num_verts Number of vertices
 * @param indices Index array (groups of 3 for triangles)
 * @param num_indices Number of indices
 * @param shade_mode 0=flat, 1=Gouraud
 * @param zbuffer_enable Enable depth testing
 * @param light_dir Directional light direction (normalized, fixed-point)
 * @param light_color Light color (RGB565)
 * @param ambient Ambient light color (RGB565)
 */
void video_3d_draw_triangles(
    const video_vertex_t *vertices, uint32_t num_verts,
    const uint16_t *indices, uint32_t num_indices,
    int shade_mode, bool zbuffer_enable,
    fixed16_t light_x, fixed16_t light_y, fixed16_t light_z,
    uint16_t light_color, uint16_t ambient);

/**
 * Set viewport dimensions
 */
void video_3d_set_viewport(int x, int y, int w, int h);

/**
 * Update the 3D pipeline's framebuffer pointer (called after buffer swap).
 */
void video_3d_set_framebuffer(uint16_t *fb);

/**
 * Enable/disable backface culling
 */
void video_3d_set_backface_cull(bool enabled);

/**
 * Set texture for subsequent draw calls.
 * Pass addr=0 and read_fn=NULL to disable texturing.
 * Texture format: RGB565 pixels in emulator RAM.
 */
void video_3d_set_texture(uint32_t addr, uint16_t width, uint16_t height,
                          video_emu_read_fn read_fn);

// ============================================================================
// Direct framebuffer access (for emulator renderers)
// All emulator video output should go through these functions so the GPU
// handles double-buffering and vsync'd LCD output with no tearing.
// ============================================================================

/**
 * Get pointer to back buffer for direct pixel rendering.
 * Buffer is WIDTH * HEIGHT uint16_t pixels in RGB565 format.
 * Safe to write without locking (only the emulator writes to back buffer).
 */
uint16_t *video_card_get_back_buffer(void);

/**
 * Present: swap front/back buffers, synchronized with LCD refresh.
 * In TEXT mode this is a no-op (refresh task continuously renders char buffer).
 * In GFX mode this performs a vsync'd page flip - prevents tearing.
 */
void video_card_present(void);

/**
 * Get current framebuffer dimensions.
 */
uint16_t video_card_get_width(void);
uint16_t video_card_get_height(void);

/**
 * Check if video card is currently in text mode.
 */
bool video_card_is_text_mode(void);

/**
 * Switch from TEXT mode to GFX mode (480x320x16).
 * Clears both framebuffers, resets clipping. Call before using
 * video_card_get_back_buffer() / video_card_present() for direct rendering.
 * No-op if already in GFX mode.
 */
void video_card_enter_gfx_mode(void);

// ============================================================================
// Text console
// In TEXT mode: writes to character buffer (rendered by refresh task at 30fps).
// In GFX mode: renders font glyphs directly onto front buffer for overlay text.
// ============================================================================

/**
 * Write a single character at the current text cursor position.
 * Handles \r, \n, \b, \t, scrolling. In text mode, zero-flicker.
 */
void video_card_putchar(char ch);

/**
 * Write a null-terminated string via video_card_putchar.
 */
void video_card_print(const char *str);

/**
 * Clear the text console (resets char buffer in text mode, or front buffer in gfx mode).
 */
void video_card_text_clear(void);

/**
 * Set text console foreground/background colors (RGB565 for gfx, mapped to VGA attr for text).
 */
void video_card_text_set_colors(uint16_t fg, uint16_t bg);

/**
 * Set text attribute directly (VGA style: hi nibble = bg color, lo nibble = fg color).
 * Uses VGA 16-color palette indices.
 */
void video_card_text_set_attr(uint8_t attr);

#ifdef __cplusplus
}
#endif

#endif // VIDEO_CARD_H
