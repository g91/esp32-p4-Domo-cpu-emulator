/*
 * M68K SDK - Type Definitions
 * ===========================
 * Standard types for M68K freestanding programs on ESP32-P4.
 */

#ifndef M68K_TYPES_H
#define M68K_TYPES_H

/* Fixed-width integer types */
typedef signed char         int8_t;
typedef unsigned char       uint8_t;
typedef signed short        int16_t;
typedef unsigned short      uint16_t;
typedef signed long         int32_t;
typedef unsigned long       uint32_t;

/* Size type */
typedef unsigned long       size_t;
typedef signed long         ssize_t;
typedef signed long         ptrdiff_t;
typedef unsigned long       uintptr_t;

/* Boolean */
#ifndef __cplusplus
typedef int                 bool;
#define true  1
#define false 0
#endif

/* NULL */
#ifndef NULL
#ifdef __cplusplus
#define NULL 0
#else
#define NULL ((void *)0)
#endif
#endif

/* Min/Max helpers */
#define M68K_MIN(a, b) (((a) < (b)) ? (a) : (b))
#define M68K_MAX(a, b) (((a) > (b)) ? (a) : (b))
#define M68K_CLAMP(x, lo, hi) M68K_MIN(M68K_MAX((x), (lo)), (hi))

/* Return codes */
#define M68K_OK          0
#define M68K_ERROR      (-1)
#define M68K_NOTFOUND   (-2)
#define M68K_NOMEM      (-3)
#define M68K_TIMEOUT    (-4)
#define M68K_BUSY       (-5)

#endif /* M68K_TYPES_H */
