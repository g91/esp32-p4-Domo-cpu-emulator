/*
 * M68K SDK - String/Memory Utility Functions & Software Math
 *
 * Pure C implementations for freestanding M68K programs.
 * Includes software multiply/divide required for M68000.
 */

#include "m68k_string.h"

/* ================================================================
 * Software math routines for M68000 (no 32-bit MUL/DIV instructions)
 * These symbols are emitted by GCC for 32-bit multiply and divide.
 * ================================================================ */

int __mulsi3(int a, int b) {
    int result = 0;
    int neg = 0;
    if (a < 0) { a = -a; neg ^= 1; }
    if (b < 0) { b = -b; neg ^= 1; }
    while (b) {
        if (b & 1) result += a;
        a <<= 1;
        b >>= 1;
    }
    return neg ? -result : result;
}

unsigned int __udivsi3(unsigned int num, unsigned int den) {
    if (den == 0) return 0;
    unsigned int quot = 0;
    unsigned int bit = 1;
    /* Align divisor to the highest bit */
    while (den < num && !(den & 0x80000000)) {
        den <<= 1;
        bit <<= 1;
    }
    while (bit) {
        if (num >= den) {
            num -= den;
            quot |= bit;
        }
        den >>= 1;
        bit >>= 1;
    }
    return quot;
}

int __divsi3(int num, int den) {
    int neg = 0;
    if (num < 0) { num = -num; neg ^= 1; }
    if (den < 0) { den = -den; neg ^= 1; }
    int q = (int)__udivsi3((unsigned int)num, (unsigned int)den);
    return neg ? -q : q;
}

unsigned int __umodsi3(unsigned int num, unsigned int den) {
    return num - __udivsi3(num, den) * den;
}

int __modsi3(int num, int den) {
    int neg = 0;
    if (num < 0) { num = -num; neg = 1; }
    if (den < 0) { den = -den; }
    int r = (int)__umodsi3((unsigned int)num, (unsigned int)den);
    return neg ? -r : r;
}

/* ================================================================
 * Memory functions
 * ================================================================ */

void *m68k_memset(void *dst, int c, size_t n) {
    unsigned char *d = (unsigned char *)dst;
    while (n--) *d++ = (unsigned char)c;
    return dst;
}

void *m68k_memcpy(void *dst, const void *src, size_t n) {
    unsigned char *d = (unsigned char *)dst;
    const unsigned char *s = (const unsigned char *)src;
    while (n--) *d++ = *s++;
    return dst;
}

void *m68k_memmove(void *dst, const void *src, size_t n) {
    unsigned char *d = (unsigned char *)dst;
    const unsigned char *s = (const unsigned char *)src;
    if (d < s) {
        while (n--) *d++ = *s++;
    } else {
        d += n; s += n;
        while (n--) *--d = *--s;
    }
    return dst;
}

int m68k_memcmp(const void *s1, const void *s2, size_t n) {
    const unsigned char *a = (const unsigned char *)s1;
    const unsigned char *b = (const unsigned char *)s2;
    while (n--) {
        if (*a != *b) return *a - *b;
        a++; b++;
    }
    return 0;
}

/* ================================================================
 * String functions
 * ================================================================ */

size_t m68k_strlen(const char *s) {
    const char *p = s;
    while (*p) p++;
    return (size_t)(p - s);
}

char *m68k_strcpy(char *dst, const char *src) {
    char *d = dst;
    while ((*d++ = *src++)) {}
    return dst;
}

char *m68k_strncpy(char *dst, const char *src, size_t n) {
    char *d = dst;
    while (n && (*d++ = *src++)) n--;
    while (n--) *d++ = '\0';
    return dst;
}

int m68k_strcmp(const char *s1, const char *s2) {
    while (*s1 && *s1 == *s2) { s1++; s2++; }
    return (unsigned char)*s1 - (unsigned char)*s2;
}

int m68k_strncmp(const char *s1, const char *s2, size_t n) {
    while (n && *s1 && *s1 == *s2) { s1++; s2++; n--; }
    if (n == 0) return 0;
    return (unsigned char)*s1 - (unsigned char)*s2;
}

char *m68k_strcat(char *dst, const char *src) {
    char *d = dst;
    while (*d) d++;
    while ((*d++ = *src++)) {}
    return dst;
}

char *m68k_strncat(char *dst, const char *src, size_t n) {
    char *d = dst;
    while (*d) d++;
    while (n-- && (*d = *src++)) d++;
    *d = '\0';
    return dst;
}

char *m68k_strchr(const char *s, int c) {
    while (*s) {
        if (*s == (char)c) return (char *)s;
        s++;
    }
    return (c == '\0') ? (char *)s : NULL;
}

char *m68k_strrchr(const char *s, int c) {
    const char *last = NULL;
    while (*s) {
        if (*s == (char)c) last = s;
        s++;
    }
    if (c == '\0') return (char *)s;
    return (char *)last;
}

char *m68k_strstr(const char *haystack, const char *needle) {
    if (!*needle) return (char *)haystack;
    size_t nlen = m68k_strlen(needle);
    while (*haystack) {
        if (m68k_strncmp(haystack, needle, nlen) == 0)
            return (char *)haystack;
        haystack++;
    }
    return NULL;
}

/* ================================================================
 * Conversion functions
 * ================================================================ */

int m68k_atoi(const char *s) {
    int result = 0;
    int neg = 0;
    while (m68k_isspace(*s)) s++;
    if (*s == '-') { neg = 1; s++; }
    else if (*s == '+') { s++; }
    while (m68k_isdigit(*s)) {
        result = result * 10 + (*s - '0');
        s++;
    }
    return neg ? -result : result;
}

uint32_t m68k_strtoul(const char *s, char **endptr, int base) {
    uint32_t result = 0;
    while (m68k_isspace(*s)) s++;

    /* Auto-detect base */
    if (base == 0) {
        if (*s == '0') {
            s++;
            if (*s == 'x' || *s == 'X') { base = 16; s++; }
            else { base = 8; }
        } else {
            base = 10;
        }
    } else if (base == 16 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
        s += 2;
    }

    while (*s) {
        int digit;
        if (m68k_isdigit(*s))
            digit = *s - '0';
        else if (*s >= 'a' && *s <= 'f')
            digit = *s - 'a' + 10;
        else if (*s >= 'A' && *s <= 'F')
            digit = *s - 'A' + 10;
        else
            break;
        if (digit >= base) break;
        result = result * (uint32_t)base + (uint32_t)digit;
        s++;
    }
    if (endptr) *endptr = (char *)s;
    return result;
}

/* ================================================================
 * m68k_snprintf - Minimal formatted output to buffer
 * Supports: %s, %c, %d, %u, %x, %X, %p, %%, width, zero-pad, '-'
 * ================================================================ */

/* Variadic support - use GCC built-in */
typedef __builtin_va_list va_list;
#define va_start(v, l) __builtin_va_start(v, l)
#define va_end(v)      __builtin_va_end(v)
#define va_arg(v, t)   __builtin_va_arg(v, t)

static int snp_putc(char *buf, size_t size, int pos, char c) {
    if ((size_t)pos < size - 1)
        buf[pos] = c;
    return pos + 1;
}

static int snp_puts(char *buf, size_t size, int pos, const char *s, int width, int left) {
    int len = 0;
    const char *p = s;
    while (*p++) len++;
    int pad = (width > len) ? width - len : 0;
    if (!left) while (pad--) pos = snp_putc(buf, size, pos, ' ');
    p = s;
    while (*p) pos = snp_putc(buf, size, pos, *p++);
    if (left) while (pad--) pos = snp_putc(buf, size, pos, ' ');
    return pos;
}

static int snp_num(char *buf, size_t size, int pos, uint32_t val, int base,
                   int is_signed, int width, int zero_pad, int left, int upper) {
    char tmp[12];
    int neg = 0;
    int idx = 0;
    const char *digits = upper ? "0123456789ABCDEF" : "0123456789abcdef";

    if (is_signed && (int32_t)val < 0) {
        neg = 1;
        val = (uint32_t)(-(int32_t)val);
    }
    if (val == 0) {
        tmp[idx++] = '0';
    } else {
        while (val) {
            tmp[idx++] = digits[val % (uint32_t)base];
            val = val / (uint32_t)base;
        }
    }
    if (neg) tmp[idx++] = '-';

    int pad = (width > idx) ? width - idx : 0;
    char pc = zero_pad ? '0' : ' ';

    if (!left) {
        if (zero_pad && neg) {
            pos = snp_putc(buf, size, pos, '-');
            idx--; /* already printed '-' */
            pad = (width > idx + 1) ? width - idx - 1 : 0;
        }
        while (pad--) pos = snp_putc(buf, size, pos, pc);
    }
    while (idx > 0) pos = snp_putc(buf, size, pos, tmp[--idx]);
    if (left) while (pad--) pos = snp_putc(buf, size, pos, ' ');
    return pos;
}

int m68k_snprintf(char *buf, size_t size, const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    int pos = 0;

    if (size == 0) { va_end(ap); return 0; }

    while (*fmt) {
        if (*fmt != '%') {
            pos = snp_putc(buf, size, pos, *fmt++);
            continue;
        }
        fmt++; /* skip '%' */

        /* Flags */
        int left = 0, zero_pad = 0;
        while (*fmt == '-' || *fmt == '0') {
            if (*fmt == '-') left = 1;
            if (*fmt == '0') zero_pad = 1;
            fmt++;
        }
        if (left) zero_pad = 0; /* left-justify overrides zero-pad */

        /* Width */
        int width = 0;
        while (m68k_isdigit(*fmt)) {
            width = width * 10 + (*fmt - '0');
            fmt++;
        }

        /* Conversion */
        switch (*fmt) {
            case 'd': case 'i': {
                int32_t v = va_arg(ap, int32_t);
                pos = snp_num(buf, size, pos, (uint32_t)v, 10, 1, width, zero_pad, left, 0);
                break;
            }
            case 'u': {
                uint32_t v = va_arg(ap, uint32_t);
                pos = snp_num(buf, size, pos, v, 10, 0, width, zero_pad, left, 0);
                break;
            }
            case 'x': {
                uint32_t v = va_arg(ap, uint32_t);
                pos = snp_num(buf, size, pos, v, 16, 0, width, zero_pad, left, 0);
                break;
            }
            case 'X': {
                uint32_t v = va_arg(ap, uint32_t);
                pos = snp_num(buf, size, pos, v, 16, 0, width, zero_pad, left, 1);
                break;
            }
            case 'p': {
                uint32_t v = va_arg(ap, uint32_t);
                pos = snp_puts(buf, size, pos, "0x", 0, 0);
                pos = snp_num(buf, size, pos, v, 16, 0, 8, 1, 0, 0);
                break;
            }
            case 'c': {
                char c = (char)va_arg(ap, int);
                pos = snp_putc(buf, size, pos, c);
                break;
            }
            case 's': {
                const char *s = va_arg(ap, const char *);
                if (!s) s = "(null)";
                pos = snp_puts(buf, size, pos, s, width, left);
                break;
            }
            case '%':
                pos = snp_putc(buf, size, pos, '%');
                break;
            case '\0':
                goto done;
            default:
                pos = snp_putc(buf, size, pos, '%');
                pos = snp_putc(buf, size, pos, *fmt);
                break;
        }
        fmt++;
    }
done:
    /* NUL terminate */
    if ((size_t)pos < size)
        buf[pos] = '\0';
    else
        buf[size - 1] = '\0';

    va_end(ap);
    return pos;
}
