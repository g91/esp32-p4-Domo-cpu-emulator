/*
 * M68K SDK - Example: Graphics Demo
 *
 * Draws colorful shapes using the video framebuffer API.
 *
 * Build:  ./build.sh examples/gfx_demo.c
 * Run:    loadrun /sdcard/gfx_demo.bin
 */

#include "m68k_sdk.h"

int main(void) {
    con_println("=== M68K Graphics Demo ===");

    /* Initialize video in 16-bit color mode */
    con_print("Initializing video... ");
    int ret = video_init(VIDEO_MODE_16BPP);
    if (ret != M68K_OK) {
        con_println("FAILED");
        return 1;
    }
    int w = video_width();
    int h = video_height();
    con_printf("OK (%dx%d)\n", w, h);

    /* Clear to dark blue */
    video_clear(RGB565(0, 0, 64));

    /* Draw a red border */
    video_rect(0, 0, w - 1, h - 1, COLOR_RED);
    video_rect(1, 1, w - 2, h - 2, COLOR_RED);

    /* Draw filled rectangles in corners */
    video_fill_rect(10, 10, 80, 60, COLOR_RED);
    video_fill_rect(w - 90, 10, w - 10, 60, COLOR_GREEN);
    video_fill_rect(10, h - 70, 80, h - 10, COLOR_BLUE);
    video_fill_rect(w - 90, h - 70, w - 10, h - 10, COLOR_YELLOW);

    /* Draw circles in the center */
    int cx = w / 2;
    int cy = h / 2;
    video_fill_circle(cx, cy, 60, COLOR_WHITE);
    video_fill_circle(cx, cy, 50, RGB565(0, 0, 64));
    video_fill_circle(cx, cy, 40, COLOR_CYAN);
    video_fill_circle(cx, cy, 30, RGB565(0, 0, 64));
    video_fill_circle(cx, cy, 20, COLOR_MAGENTA);

    /* Draw diagonal lines */
    video_line(0, 0, cx - 70, cy - 70, COLOR_WHITE);
    video_line(w - 1, 0, cx + 70, cy - 70, COLOR_WHITE);
    video_line(0, h - 1, cx - 70, cy + 70, COLOR_WHITE);
    video_line(w - 1, h - 1, cx + 70, cy + 70, COLOR_WHITE);

    /* Draw some text */
    video_set_colors(COLOR_WHITE, RGB565(0, 0, 64));
    video_set_cursor(cx - 40, 10);
    const char *title = "GFX DEMO";
    for (int i = 0; title[i]; i++) {
        video_draw_char(title[i]);
    }

    /* Flip to display */
    video_flip();

    con_println("Graphics drawn. Press any key to exit.");
    con_getchar();

    return 0;
}
