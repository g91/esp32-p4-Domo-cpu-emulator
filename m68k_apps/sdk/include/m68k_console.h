/*
 * M68K SDK - Console I/O
 * ======================
 * Text input/output via the bus controller console device.
 * Output goes to both UART serial and the LCD display.
 */

#ifndef M68K_CONSOLE_H
#define M68K_CONSOLE_H

#include "m68k_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Output a single character */
void con_putchar(char c);

/* Print a null-terminated string */
void con_print(const char *s);

/* Print a string followed by newline */
void con_println(const char *s);

/* Print an unsigned integer in decimal */
void con_print_dec(int32_t val);

/* Print an unsigned integer in hex with the given number of digits */
void con_print_hex(uint32_t val, int digits);

/* Simple printf supporting: %s, %d, %u, %x, %c, %% */
void con_printf(const char *fmt, ...);

/* Check if a character is available for reading */
bool con_key_available(void);

/* Read a single character (blocks until available) */
char con_getchar(void);

/* Read a line of text with basic editing (backspace, Ctrl+C)
 * Returns number of characters read. buf is null-terminated. */
int con_getline(char *buf, int maxlen);

/* Send ANSI escape: clear screen */
void con_clear(void);

/* Send ANSI escape: move cursor to row, col (1-based) */
void con_gotoxy(int col, int row);

/* Send ANSI escape: set text color (0-7 standard, add 8 for bright) */
void con_set_color(int fg, int bg);

/* Reset text attributes to defaults */
void con_reset_color(void);

/* ANSI color constants */
#define CON_BLACK    0
#define CON_RED      1
#define CON_GREEN    2
#define CON_YELLOW   3
#define CON_BLUE     4
#define CON_MAGENTA  5
#define CON_CYAN     6
#define CON_WHITE    7

#ifdef __cplusplus
}
#endif

#endif /* M68K_CONSOLE_H */
