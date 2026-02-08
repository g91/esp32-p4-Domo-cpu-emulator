/*
 * M68K SDK - Video/GPU API
 * ========================
 * Framebuffer-based video device with 2D drawing acceleration.
 * Supports text mode, 8-bit palette, and 16-bit RGB565 graphics.
 */

#ifndef M68K_VIDEO_H
#define M68K_VIDEO_H

#include "m68k_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* RGB565 color construction macro */
#define RGB565(r, g, b) \
    ((uint16_t)(((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | (((b) & 0xF8) >> 3))

/* Common RGB565 colors */
#define COLOR_BLACK      0x0000
#define COLOR_WHITE      0xFFFF
#define COLOR_RED        0xF800
#define COLOR_GREEN      0x07E0
#define COLOR_BLUE       0x001F
#define COLOR_YELLOW     0xFFE0
#define COLOR_CYAN       0x07FF
#define COLOR_MAGENTA    0xF81F
#define COLOR_ORANGE     0xFD20
#define COLOR_GRAY       0x8410

/* ---- Mode Setup ---- */

/* Initialize a video mode. fb_addr = M68K RAM address for framebuffer.
 * Use VID_MODE_* constants from m68k_io.h.
 * The framebuffer must be large enough for the mode (width × height × bpp/8).
 * Returns fb_size on success. */
uint32_t video_init(uint32_t mode, uint32_t fb_addr);

/* Get current screen width in pixels */
uint32_t video_width(void);

/* Get current screen height in pixels */
uint32_t video_height(void);

/* Get bits per pixel */
uint32_t video_bpp(void);

/* Get bytes per scanline */
uint32_t video_pitch(void);

/* Get framebuffer size in bytes */
uint32_t video_fb_size(void);

/* Get video status flags */
uint32_t video_status(void);

/* Get the vblank counter (increments each frame flip) */
uint32_t video_vblank(void);

/* ---- Drawing Commands ---- */

/* Clear the entire framebuffer to black */
void video_clear(void);

/* Flip/refresh the display (signal host to update LCD from framebuffer) */
void video_flip(void);

/* Draw a single pixel */
void video_pixel(uint32_t x, uint32_t y, uint32_t color);

/* Fill a rectangle */
void video_fill_rect(uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint32_t color);

/* Draw a horizontal line (fast) */
void video_hline(uint32_t x, uint32_t y, uint32_t w, uint32_t color);

/* Draw a vertical line (fast) */
void video_vline(uint32_t x, uint32_t y, uint32_t h, uint32_t color);

/* Draw a line between two points */
void video_line(uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2, uint32_t color);

/* Draw a circle outline */
void video_circle(uint32_t cx, uint32_t cy, uint32_t radius, uint32_t color);

/* Draw a filled circle */
void video_fill_circle(uint32_t cx, uint32_t cy, uint32_t radius, uint32_t color);

/* Draw a rectangle outline */
void video_rect(uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint32_t color);

/* Scroll the framebuffer vertically by n pixels */
void video_scroll(uint32_t pixels);

/* ---- Palette (8-bit modes only) ---- */

/* Set a palette entry (index 0-255, RGB value) */
void video_set_palette(uint32_t index, uint32_t rgb);

/* ---- Text Cursor ---- */

/* Set the text cursor position */
void video_set_cursor(uint32_t x, uint32_t y);

/* Set foreground and background text colors */
void video_set_colors(uint32_t fg, uint32_t bg);

/* Draw a character at the current cursor position */
void video_draw_char(char c);

/* ---- Blit ---- */

/* Blit a rectangular region from source to destination coordinates */
void video_blit(uint32_t src_x, uint32_t src_y,
                uint32_t dst_x, uint32_t dst_y,
                uint32_t w, uint32_t h);

/* Wait for vertical blank (useful for smooth animation) */
void video_wait_vblank(void);

#ifdef __cplusplus
}
#endif

#endif /* M68K_VIDEO_H */
