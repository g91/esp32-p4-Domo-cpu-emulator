/*
 *  sysdeps.h - System dependent definitions for ESP32-P4 (ESP-IDF port)
 *
 *  BasiliskII ESP32 Port - Adapted for ESP-IDF (no Arduino)
 *  Based on Basilisk II (C) 1997-2008 Christian Bauer
 */

#ifndef SYSDEPS_H
#define SYSDEPS_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

// ESP-IDF headers
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

// C++ STL headers needed by BasiliskII
#ifdef __cplusplus
#include <vector>
#include <map>
using std::vector;
// Don't pull std::map into global namespace to avoid conflicts
#endif

// Include ESP32 user strings
#include "user_strings_esp32.h"

/*
 * CPU and addressing mode configuration
 */
#define EMULATED_68K 1
#define REAL_ADDRESSING 0
#define DIRECT_ADDRESSING 0
#define ROM_IS_WRITE_PROTECTED 1
#define USE_PREFETCH_BUFFER 0
#define SUPPORTS_EXTFS 0
#define SUPPORTS_UDP_TUNNEL 0
#define USE_CPU_EMUL_SERVICES 1

/*
 * ESP32-P4 is little-endian RISC-V
 */
#undef WORDS_BIGENDIAN

/*
 * Data type sizes for ESP32-P4
 */
#define SIZEOF_SHORT 2
#define SIZEOF_INT 4
#define SIZEOF_LONG 4
#define SIZEOF_LONG_LONG 8
#define SIZEOF_VOID_P 4
#define SIZEOF_FLOAT 4
#define SIZEOF_DOUBLE 8

/*
 * Basic data types
 */
typedef uint8_t uint8;
typedef int8_t int8;
typedef uint16_t uint16;
typedef int16_t int16;
typedef uint32_t uint32;
typedef int32_t int32;
typedef uint64_t uint64;
typedef int64_t int64;
typedef uint32_t uintptr;
typedef int32_t intptr;

typedef int32_t loff_t;
typedef char* caddr_t;
typedef uint64_t tm_time_t;

/*
 * UAE CPU data types
 */
typedef int8 uae_s8;
typedef uint8 uae_u8;
typedef int16 uae_s16;
typedef uint16 uae_u16;
typedef int32 uae_s32;
typedef uint32 uae_u32;
typedef int64 uae_s64;
typedef uint64 uae_u64;
typedef uae_u32 uaecptr;

/*
 * ESP32-P4 RISC-V does NOT support unaligned memory access safely
 */
#undef CPU_CAN_ACCESS_UNALIGNED

/*
 * 64-bit value macros
 */
#define VAL64(a) (a ## LL)
#define UVAL64(a) (a ## ULL)

/*
 * Memory pointer type for Mac addresses
 */
#define memptr uint32

/*
 * Float format
 */
#define IEEE_FLOAT_FORMAT 1
#define HOST_FLOAT_FORMAT IEEE_FLOAT_FORMAT

/*
 * Inline hints
 */
#define __inline__ inline
#define ALWAYS_INLINE inline __attribute__((always_inline))

/*
 * Byte swapping functions for little-endian ESP32 accessing big-endian Mac data
 */
static ALWAYS_INLINE uae_u32 do_byteswap_32(uae_u32 v) {
    return __builtin_bswap32(v);
}

static ALWAYS_INLINE uae_u16 do_byteswap_16(uae_u16 v) {
    return __builtin_bswap16(v);
}

static ALWAYS_INLINE uae_u32 do_get_mem_long(uae_u32 *a) {
    return __builtin_bswap32(*a);
}

static ALWAYS_INLINE uae_u32 do_get_mem_word(uae_u16 *a) {
    return __builtin_bswap16(*a);
}

#define do_get_mem_byte(a) ((uae_u32)*((uae_u8 *)(a)))

static ALWAYS_INLINE void do_put_mem_long(uae_u32 *a, uae_u32 v) {
    *a = __builtin_bswap32(v);
}

static ALWAYS_INLINE void do_put_mem_word(uae_u16 *a, uae_u32 v) {
    *a = __builtin_bswap16((uae_u16)v);
}

#define do_put_mem_byte(a, v) (*(uae_u8 *)(a) = (v))

/*
 * Memory bank access function call macros
 */
#define call_mem_get_func(func, addr) ((*func)(addr))
#define call_mem_put_func(func, addr, v) ((*func)(addr, v))

/*
 * CPU emulation size
 */
#define CPU_EMU_SIZE 0
#undef NO_INLINE_MEMORY_ACCESS

/*
 * Enum macros
 */
#define ENUMDECL typedef enum
#define ENUMNAME(name) name

/*
 * Logging - route to ESP-IDF logging
 */
#ifdef __cplusplus
extern "C" {
#endif
void basilisk_log(const char *fmt, ...);
#ifdef __cplusplus
}
#endif
#define write_log basilisk_log

/*
 * Register hints (not used on ESP32)
 */
#define REGPARAM
#define REGPARAM2

#ifndef UNUSED
#define UNUSED(x) ((void)(x))
#endif

/*
 * Branch prediction
 */
#ifndef likely
#define likely(x)   __builtin_expect(!!(x), 1)
#endif
#ifndef unlikely
#define unlikely(x) __builtin_expect(!!(x), 0)
#endif

/*
 * Spinlock (single-threaded CPU emulation, no-op)
 */
typedef volatile int b2_spinlock_t;
#define spinlock_t b2_spinlock_t
#define SPIN_LOCK_UNLOCKED 0

static inline void spin_lock(b2_spinlock_t *lock) { UNUSED(lock); }
static inline void spin_unlock(b2_spinlock_t *lock) { UNUSED(lock); }
static inline int spin_trylock(b2_spinlock_t *lock) { UNUSED(lock); return 1; }

/*
 * Mutex using FreeRTOS semaphores
 */
struct B2_mutex {
    SemaphoreHandle_t sem;
};

/*
 * Timing functions
 */
#ifdef __cplusplus
extern "C" {
#endif
extern uint64 GetTicks_usec(void);
extern void Delay_usec(uint64 usec);
#ifdef __cplusplus
}
#endif

/*
 * Arduino compatibility - millis() replacement
 */
static inline uint32_t millis(void) {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/*
 * PSRAM allocation helpers
 */
#define ps_malloc(size) heap_caps_malloc(size, MALLOC_CAP_SPIRAM)
#define ps_calloc(n, size) heap_caps_calloc(n, size, MALLOC_CAP_SPIRAM)
#define psram_malloc(size) heap_caps_malloc(size, MALLOC_CAP_SPIRAM)
#define psram_calloc(n, size) heap_caps_calloc(n, size, MALLOC_CAP_SPIRAM)

/*
 * strdup - may not be available in all toolchains
 */
#ifdef __cplusplus
extern "C" {
#endif
char *basilisk_strdup(const char *s);
#ifdef __cplusplus
}
#endif
// Override strdup if not defined
#ifndef strdup
#define strdup basilisk_strdup
#endif

/*
 * Disable features not needed on ESP32
 */
#undef ENABLE_MON
#undef USE_JIT
#undef ENABLE_GTK
#undef ENABLE_XF86_DGA
#undef USE_SDL
#undef USE_SDL_VIDEO
#undef USE_SDL_AUDIO

/*
 * FPU configuration
 */
#define FPU_IEEE 1
#define FPU_X86 0
#define FPU_UAE 0

#define ASM_SYM(a)

#ifndef O_RDONLY
#define O_RDONLY 0
#endif
#ifndef O_RDWR
#define O_RDWR 2
#endif

#ifndef DEBUG
#define DEBUG 0
#endif

#endif /* SYSDEPS_H */
