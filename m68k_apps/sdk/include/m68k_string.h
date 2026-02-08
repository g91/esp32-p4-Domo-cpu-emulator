/*
 * M68K SDK - String/Memory Utility Functions & Software Math
 *
 * Provides basic string and memory operations for freestanding M68K programs.
 * Also provides __mulsi3/__divsi3/__modsi3 software math routines required
 * because the M68000 CPU lacks 32-bit multiply and divide instructions.
 */

#ifndef M68K_STRING_H
#define M68K_STRING_H

#include "m68k_types.h"

/* ---- Memory functions ---- */
void *m68k_memset(void *dst, int c, size_t n);
void *m68k_memcpy(void *dst, const void *src, size_t n);
void *m68k_memmove(void *dst, const void *src, size_t n);
int   m68k_memcmp(const void *s1, const void *s2, size_t n);

/* ---- String functions ---- */
size_t m68k_strlen(const char *s);
char  *m68k_strcpy(char *dst, const char *src);
char  *m68k_strncpy(char *dst, const char *src, size_t n);
int    m68k_strcmp(const char *s1, const char *s2);
int    m68k_strncmp(const char *s1, const char *s2, size_t n);
char  *m68k_strcat(char *dst, const char *src);
char  *m68k_strncat(char *dst, const char *src, size_t n);
char  *m68k_strchr(const char *s, int c);
char  *m68k_strrchr(const char *s, int c);
char  *m68k_strstr(const char *haystack, const char *needle);

/* ---- Character classification ---- */
static inline int m68k_isspace(int c)  { return c == ' ' || (c >= '\t' && c <= '\r'); }
static inline int m68k_isdigit(int c)  { return c >= '0' && c <= '9'; }
static inline int m68k_isalpha(int c)  { return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z'); }
static inline int m68k_isalnum(int c)  { return m68k_isalpha(c) || m68k_isdigit(c); }
static inline int m68k_isupper(int c)  { return c >= 'A' && c <= 'Z'; }
static inline int m68k_islower(int c)  { return c >= 'a' && c <= 'z'; }
static inline int m68k_toupper(int c)  { return m68k_islower(c) ? c - 32 : c; }
static inline int m68k_tolower(int c)  { return m68k_isupper(c) ? c + 32 : c; }
static inline int m68k_isprint(int c)  { return c >= 0x20 && c <= 0x7E; }
static inline int m68k_isxdigit(int c) {
    return m68k_isdigit(c) || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f');
}

/* ---- Conversion functions ---- */
int      m68k_atoi(const char *s);
uint32_t m68k_strtoul(const char *s, char **endptr, int base);

/* ---- Formatting ---- */
int m68k_snprintf(char *buf, size_t size, const char *fmt, ...);

#endif /* M68K_STRING_H */
