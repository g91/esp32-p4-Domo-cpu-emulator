/*
 * M68K SDK - Video/GPU Implementation
 */

#include "m68k_video.h"
#include "m68k_io.h"

uint32_t video_init(uint32_t mode, uint32_t fb_addr) {
    IO_WRITE32(VID_FB_ADDR, fb_addr);
    IO_WRITE32(VID_MODE, mode);
    IO_WRITE32(VID_COMMAND, VID_CMD_INIT);
    return IO_READ32(VID_FB_SIZE);
}

uint32_t video_width(void)   { return IO_READ32(VID_WIDTH); }
uint32_t video_height(void)  { return IO_READ32(VID_HEIGHT); }
uint32_t video_bpp(void)     { return IO_READ32(VID_BPP); }
uint32_t video_pitch(void)   { return IO_READ32(VID_PITCH); }
uint32_t video_fb_size(void) { return IO_READ32(VID_FB_SIZE); }
uint32_t video_status(void)  { return IO_READ32(VID_STATUS); }
uint32_t video_vblank(void)  { return IO_READ32(VID_VBLANK); }

void video_clear(void) {
    IO_WRITE32(VID_COMMAND, VID_CMD_CLEAR);
}

void video_flip(void) {
    IO_WRITE32(VID_COMMAND, VID_CMD_FLIP);
}

void video_pixel(uint32_t x, uint32_t y, uint32_t color) {
    IO_WRITE32(VID_DRAW_X, x);
    IO_WRITE32(VID_DRAW_Y, y);
    IO_WRITE32(VID_DRAW_COLOR, color);
    IO_WRITE32(VID_COMMAND, VID_CMD_DRAW_PIXEL);
}

void video_fill_rect(uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint32_t color) {
    IO_WRITE32(VID_DRAW_X, x);
    IO_WRITE32(VID_DRAW_Y, y);
    IO_WRITE32(VID_DRAW_W, w);
    IO_WRITE32(VID_DRAW_H, h);
    IO_WRITE32(VID_DRAW_COLOR, color);
    IO_WRITE32(VID_COMMAND, VID_CMD_FILL_RECT);
}

void video_hline(uint32_t x, uint32_t y, uint32_t w, uint32_t color) {
    IO_WRITE32(VID_DRAW_X, x);
    IO_WRITE32(VID_DRAW_Y, y);
    IO_WRITE32(VID_DRAW_W, w);
    IO_WRITE32(VID_DRAW_COLOR, color);
    IO_WRITE32(VID_COMMAND, VID_CMD_HLINE);
}

void video_vline(uint32_t x, uint32_t y, uint32_t h, uint32_t color) {
    IO_WRITE32(VID_DRAW_X, x);
    IO_WRITE32(VID_DRAW_Y, y);
    IO_WRITE32(VID_DRAW_H, h);
    IO_WRITE32(VID_DRAW_COLOR, color);
    IO_WRITE32(VID_COMMAND, VID_CMD_VLINE);
}

void video_line(uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2, uint32_t color) {
    IO_WRITE32(VID_DRAW_X, x1);
    IO_WRITE32(VID_DRAW_Y, y1);
    IO_WRITE32(VID_DRAW_W, x2);
    IO_WRITE32(VID_DRAW_H, y2);
    IO_WRITE32(VID_DRAW_COLOR, color);
    IO_WRITE32(VID_COMMAND, VID_CMD_DRAW_LINE);
}

void video_circle(uint32_t cx, uint32_t cy, uint32_t radius, uint32_t color) {
    IO_WRITE32(VID_DRAW_X, cx);
    IO_WRITE32(VID_DRAW_Y, cy);
    IO_WRITE32(VID_DRAW_W, radius);
    IO_WRITE32(VID_DRAW_COLOR, color);
    IO_WRITE32(VID_COMMAND, VID_CMD_CIRCLE);
}

void video_fill_circle(uint32_t cx, uint32_t cy, uint32_t radius, uint32_t color) {
    IO_WRITE32(VID_DRAW_X, cx);
    IO_WRITE32(VID_DRAW_Y, cy);
    IO_WRITE32(VID_DRAW_W, radius);
    IO_WRITE32(VID_DRAW_COLOR, color);
    IO_WRITE32(VID_COMMAND, VID_CMD_FILL_CIRCLE);
}

void video_rect(uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint32_t color) {
    /* Draw 4 edges */
    video_hline(x, y, w, color);
    video_hline(x, y + h - 1, w, color);
    video_vline(x, y, h, color);
    video_vline(x + w - 1, y, h, color);
}

void video_scroll(uint32_t pixels) {
    IO_WRITE32(VID_SCROLL_Y, pixels);
    IO_WRITE32(VID_COMMAND, VID_CMD_SCROLL);
}

void video_set_palette(uint32_t index, uint32_t rgb) {
    IO_WRITE32(VID_DRAW_X, index);
    IO_WRITE32(VID_DRAW_COLOR, rgb);
    IO_WRITE32(VID_COMMAND, VID_CMD_SET_PALETTE);
}

void video_set_cursor(uint32_t x, uint32_t y) {
    IO_WRITE32(VID_CURSOR_X, x);
    IO_WRITE32(VID_CURSOR_Y, y);
}

void video_set_colors(uint32_t fg, uint32_t bg) {
    IO_WRITE32(VID_FG_COLOR, fg);
    IO_WRITE32(VID_BG_COLOR, bg);
}

void video_draw_char(char c) {
    IO_WRITE32(VID_DRAW_COLOR, (uint32_t)c);
    IO_WRITE32(VID_COMMAND, VID_CMD_DRAW_CHAR);
}

void video_blit(uint32_t src_x, uint32_t src_y,
                uint32_t dst_x, uint32_t dst_y,
                uint32_t w, uint32_t h) {
    IO_WRITE32(VID_SRC_X, src_x);
    IO_WRITE32(VID_SRC_Y, src_y);
    IO_WRITE32(VID_DRAW_X, dst_x);
    IO_WRITE32(VID_DRAW_Y, dst_y);
    IO_WRITE32(VID_DRAW_W, w);
    IO_WRITE32(VID_DRAW_H, h);
    IO_WRITE32(VID_COMMAND, VID_CMD_BLIT);
}

void video_wait_vblank(void) {
    uint32_t start = IO_READ32(VID_VBLANK);
    while (IO_READ32(VID_VBLANK) == start) { /* spin */ }
}
