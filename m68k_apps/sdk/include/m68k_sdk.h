/*
 * M68K SDK - Master Include Header
 *
 * Include this single header to access all SDK functionality:
 *   #include "m68k_sdk.h"
 */

#ifndef M68K_SDK_H
#define M68K_SDK_H

/* Core types and hardware I/O */
#include "m68k_types.h"
#include "m68k_io.h"

/* String/memory utilities and software math */
#include "m68k_string.h"

/* Device APIs */
#include "m68k_console.h"    /* Console text I/O (UART + LCD) */
#include "m68k_fs.h"         /* SD card filesystem */
#include "m68k_net.h"        /* TCP/UDP networking (BSD sockets) */
#include "m68k_fpu.h"        /* Hardware FPU via bus controller */
#include "m68k_audio.h"      /* SB16-style audio */
#include "m68k_video.h"      /* Framebuffer video / GPU */
#include "m68k_system.h"     /* DMA, Timer, IRQ, Heap */

/* SDK version */
#define M68K_SDK_VERSION_MAJOR  1
#define M68K_SDK_VERSION_MINOR  0
#define M68K_SDK_VERSION_PATCH  0
#define M68K_SDK_VERSION_STRING "1.0.0"

#endif /* M68K_SDK_H */
