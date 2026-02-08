/*
 * M68K SDK - Console I/O Implementation
 */

#include "m68k_console.h"
#include "m68k_io.h"
#include "m68k_string.h"

/* ---- Output ---- */

void con_putchar(char c) {
    IO_WRITE8(CONSOLE_OUT, c);
}

void con_print(const char *s) {
    while (*s) {
        if (*s == '\n') con_putchar('\r');
        con_putchar(*s++);
    }
}

void con_println(const char *s) {
    con_print(s);
    con_putchar('\r');
    con_putchar('\n');
}

void con_print_dec(int32_t val) {
    if (val < 0) {
        con_putchar('-');
        val = -val;
    }
    if (val == 0) {
        con_putchar('0');
        return;
    }
    char buf[12];
    int i = 0;
    while (val > 0) {
        buf[i++] = '0' + (val % 10);
        val /= 10;
    }
    while (i > 0)
        con_putchar(buf[--i]);
}

void con_print_hex(uint32_t val, int digits) {
    static const char hex[] = "0123456789ABCDEF";
    for (int i = digits - 1; i >= 0; i--)
        con_putchar(hex[(val >> (i * 4)) & 0xF]);
}

void con_printf(const char *fmt, ...) {
    __builtin_va_list args;
    __builtin_va_start(args, fmt);

    while (*fmt) {
        if (*fmt == '%') {
            fmt++;
            /* Optional width for hex */
            int width = 0;
            while (*fmt >= '0' && *fmt <= '9') {
                width = width * 10 + (*fmt - '0');
                fmt++;
            }
            switch (*fmt) {
                case 's': {
                    const char *s = __builtin_va_arg(args, const char *);
                    con_print(s ? s : "(null)");
                    break;
                }
                case 'd': {
                    int32_t d = __builtin_va_arg(args, int32_t);
                    con_print_dec(d);
                    break;
                }
                case 'u': {
                    uint32_t u = __builtin_va_arg(args, uint32_t);
                    if (u == 0) { con_putchar('0'); break; }
                    char buf[12];
                    int i = 0;
                    while (u > 0) { buf[i++] = '0' + (u % 10); u /= 10; }
                    while (i > 0) con_putchar(buf[--i]);
                    break;
                }
                case 'x': {
                    uint32_t x = __builtin_va_arg(args, uint32_t);
                    con_print_hex(x, width ? width : 8);
                    break;
                }
                case 'c': {
                    int c = __builtin_va_arg(args, int);
                    con_putchar((char)c);
                    break;
                }
                case '%':
                    con_putchar('%');
                    break;
                default:
                    con_putchar('%');
                    con_putchar(*fmt);
                    break;
            }
        } else {
            if (*fmt == '\n') con_putchar('\r');
            con_putchar(*fmt);
        }
        fmt++;
    }
    __builtin_va_end(args);
}

/* ---- Input ---- */

bool con_key_available(void) {
    return (IO_READ8(CONSOLE_STATUS) & CONSOLE_RX_READY) != 0;
}

char con_getchar(void) {
    while (!con_key_available()) { /* spin */ }
    return (char)IO_READ8(CONSOLE_IN);
}

int con_getline(char *buf, int maxlen) {
    int pos = 0;
    while (pos < maxlen - 1) {
        char c = con_getchar();
        if (c == '\r' || c == '\n') {
            con_putchar('\r');
            con_putchar('\n');
            break;
        } else if (c == '\b' || c == 0x7F) {
            if (pos > 0) {
                pos--;
                con_putchar('\b');
                con_putchar(' ');
                con_putchar('\b');
            }
        } else if (c == 0x03) { /* Ctrl+C */
            con_print("^C\r\n");
            pos = 0;
            break;
        } else if (c >= 0x20 && c < 0x7F) {
            buf[pos++] = c;
            con_putchar(c);
        }
    }
    buf[pos] = '\0';
    return pos;
}

/* ---- ANSI escape sequences ---- */

void con_clear(void) {
    con_print("\033[2J\033[H");
}

void con_gotoxy(int col, int row) {
    con_print("\033[");
    con_print_dec(row);
    con_putchar(';');
    con_print_dec(col);
    con_putchar('H');
}

void con_set_color(int fg, int bg) {
    con_print("\033[");
    if (fg >= 8) {
        con_print_dec(90 + (fg - 8));
    } else {
        con_print_dec(30 + fg);
    }
    con_putchar(';');
    if (bg >= 8) {
        con_print_dec(100 + (bg - 8));
    } else {
        con_print_dec(40 + bg);
    }
    con_putchar('m');
}

void con_reset_color(void) {
    con_print("\033[0m");
}
