/*
 * M68K SDK - Example Application: Hello SDK
 *
 * Demonstrates console output, FPU math, filesystem listing,
 * audio beep, video drawing, timer, and heap allocation.
 *
 * Build:
 *   ./build.sh hello_sdk.c        (Linux/macOS)
 *   build.cmd hello_sdk.c         (Windows)
 *
 * Run on ESP32-P4:
 *   loadrun /sdcard/hello_sdk.bin
 */

#include "m68k_sdk.h"

/* Heap region (defined in linker script) */
extern uint32_t _heap_start;
extern uint32_t _heap_end;

int main(void) {
    /* ---- Console Demo ---- */
    con_clear();
    con_set_color(CON_COLOR_CYAN, CON_COLOR_BLACK);
    con_println("========================================");
    con_println("  M68K SDK v" M68K_SDK_VERSION_STRING " - Hello World!");
    con_println("========================================");
    con_reset_color();
    con_println("");

    /* ---- FPU Demo ---- */
    con_println("[FPU] Computing math...");

    /* Calculate sqrt(2) */
    fpu_double_t sqrt2 = fpu_sqrt(fpu_from_int(2));
    con_print("  sqrt(2) = ");
    fpu_print("", sqrt2);

    /* Calculate pi * 2 */
    fpu_double_t pi = fpu_pi();
    fpu_double_t two_pi = fpu_mul(pi, fpu_from_int(2));
    con_print("  2*pi    = ");
    fpu_print("", two_pi);

    /* Calculate sin(pi/4) */
    fpu_double_t quarter_pi = fpu_div(pi, fpu_from_int(4));
    fpu_double_t sin_val = fpu_sin(quarter_pi);
    con_print("  sin(pi/4) = ");
    fpu_print("", sin_val);

    con_println("");

    /* ---- Heap Demo ---- */
    con_println("[Heap] Initializing allocator...");
    heap_init((uint32_t)&_heap_start, (uint32_t)&_heap_end);
    con_printf("  Total: %u bytes\n", heap_total_space());
    con_printf("  Free:  %u bytes\n", heap_free_space());

    char *buf = (char *)heap_alloc(256);
    if (buf) {
        m68k_strcpy(buf, "Hello from heap-allocated memory!");
        con_printf("  Alloc: %s\n", buf);
        con_printf("  Free after alloc: %u bytes\n", heap_free_space());
    }
    con_println("");

    /* ---- Filesystem Demo ---- */
    con_println("[FS] Listing /sdcard/ ...");
    char listing[512];
    int ret = fs_listdir("/sdcard", listing, sizeof(listing));
    if (ret == M68K_OK) {
        /* Print each entry (newline-separated) */
        char *p = listing;
        int count = 0;
        while (*p && count < 10) {
            char *nl = m68k_strchr(p, '\n');
            if (nl) *nl = '\0';
            con_printf("  %s\n", p);
            if (nl) p = nl + 1;
            else break;
            count++;
        }
        if (*p) con_println("  ...(more files)");
    } else {
        con_println("  (no SD card or error)");
    }
    con_println("");

    /* ---- Timer Demo ---- */
    con_println("[Timer] Reading system ticks...");
    uint32_t t1 = timer_ticks();
    delay_ms(100);
    uint32_t t2 = timer_ticks();
    con_printf("  Tick before: %u\n", t1);
    con_printf("  Tick after:  %u\n", t2);
    con_printf("  Elapsed:     %u ticks\n", t2 - t1);
    con_println("");

    /* ---- Audio Demo ---- */
    con_println("[Audio] Playing startup beep...");
    audio_init();
    audio_set_volume(200);
    audio_beep(880, 150);   /* A5 for 150ms */
    delay_ms(200);
    audio_beep(1047, 150);  /* C6 for 150ms */
    delay_ms(200);
    audio_beep(1319, 300);  /* E6 for 300ms */
    delay_ms(400);
    con_println("  Done.");
    con_println("");

    /* ---- Done ---- */
    con_set_color(CON_COLOR_GREEN, CON_COLOR_BLACK);
    con_println("All SDK demos completed successfully!");
    con_reset_color();

    return 0;
}
