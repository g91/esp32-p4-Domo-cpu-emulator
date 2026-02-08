/*
 * M68K SDK - Bus I/O Definitions
 * ==============================
 * Memory-mapped I/O addresses and register definitions for all bus devices.
 * This is the hardware abstraction layer for the ESP32-P4 virtual bus controller.
 *
 * Memory Map:
 *   0x00000000 - 0x00EFFFFF : 15MB RAM
 *   0x00F00000 - 0x00FFFFFF : 1MB I/O space
 *
 * Device Map:
 *   0x00F00000 : Network (BSD sockets via lwIP)
 *   0x00F01000 : Filesystem (SD card via FATFS)
 *   0x00F02000 : Console (UART + LCD)
 *   0x00F03000 : DMA Controller
 *   0x00F04000 : Interrupt Controller
 *   0x00F05000 : Timer
 *   0x00F06000 : FPU (IEEE 754 coprocessor)
 *   0x00F07000 : Audio (Sound Blaster 16)
 *   0x00F08000 : Video/GPU (framebuffer + 2D accel)
 */

#ifndef M68K_IO_H
#define M68K_IO_H

#include "m68k_types.h"

/* ============================================================================
 * I/O Access Macros
 * ============================================================================ */

/* Read/write volatile memory-mapped registers */
#define IO_READ8(addr)        (*(volatile uint8_t  *)(addr))
#define IO_READ16(addr)       (*(volatile uint16_t *)(addr))
#define IO_READ32(addr)       (*(volatile uint32_t *)(addr))
#define IO_WRITE8(addr, val)  (*(volatile uint8_t  *)(addr) = (uint8_t)(val))
#define IO_WRITE16(addr, val) (*(volatile uint16_t *)(addr) = (uint16_t)(val))
#define IO_WRITE32(addr, val) (*(volatile uint32_t *)(addr) = (uint32_t)(val))

/* Bus base addresses */
#define BUS_IO_BASE          0x00F00000

/* ============================================================================
 * Console Device (0x00F02000)
 * ============================================================================ */
#define CONSOLE_BASE         0x00F02000
#define CONSOLE_OUT          (CONSOLE_BASE + 0x00)  /* Write: output character */
#define CONSOLE_IN           (CONSOLE_BASE + 0x04)  /* Read: input character */
#define CONSOLE_STATUS       (CONSOLE_BASE + 0x08)  /* Read: status register */

/* Console status bits */
#define CONSOLE_RX_READY     0x01  /* Character available to read */
#define CONSOLE_TX_READY     0x02  /* Ready to send (always set) */

/* ============================================================================
 * Filesystem Device (0x00F01000)
 * ============================================================================ */
#define FS_BASE              0x00F01000
#define FS_COMMAND           (FS_BASE + 0x000)  /* Write: command register */
#define FS_STATUS            (FS_BASE + 0x004)  /* Read: 0=busy, 1=ok, 2+=err */
#define FS_SIZE              (FS_BASE + 0x008)  /* R/W: data length */
#define FS_OFFSET            (FS_BASE + 0x00C)  /* R/W: file position */
#define FS_FILENAME          (FS_BASE + 0x100)  /* Write: path string (256b) */
#define FS_BUFFER            (FS_BASE + 0x200)  /* R/W: data buffer (512b) */

/* Filesystem commands */
#define FS_CMD_OPEN          0x01
#define FS_CMD_CLOSE         0x02
#define FS_CMD_READ          0x03
#define FS_CMD_WRITE         0x04
#define FS_CMD_SEEK          0x05
#define FS_CMD_STAT          0x06
#define FS_CMD_READDIR       0x07
#define FS_CMD_MKDIR         0x08
#define FS_CMD_REMOVE        0x09

/* Filesystem status values */
#define FS_STATUS_BUSY       0
#define FS_STATUS_OK         1
#define FS_STATUS_ERROR      2

/* Maximum buffer size for a single read/write */
#define FS_BUF_SIZE          512

/* ============================================================================
 * Network Device (0x00F00000)
 * ============================================================================ */
#define NET_BASE             0x00F00000
#define NET_COMMAND          (NET_BASE + 0x00)  /* Write: triggers command */
#define NET_STATUS           (NET_BASE + 0x04)  /* Read: status flags */
#define NET_SOCKET_ID        (NET_BASE + 0x08)  /* R/W: socket ID */
#define NET_ADDR_TYPE        (NET_BASE + 0x0C)  /* R/W: address family */
#define NET_ADDR_IP          (NET_BASE + 0x10)  /* R/W: IP address */
#define NET_ADDR_PORT        (NET_BASE + 0x14)  /* R/W: port number */
#define NET_DATA_LEN         (NET_BASE + 0x18)  /* R/W: data length */
#define NET_DATA_PTR         (NET_BASE + 0x1C)  /* R/W: M68K buffer address */
#define NET_FLAGS            (NET_BASE + 0x20)  /* R/W: flags */
#define NET_RESULT           (NET_BASE + 0x24)  /* Read: operation result */

/* Network commands */
#define NET_CMD_SOCKET       0x01
#define NET_CMD_BIND         0x02
#define NET_CMD_LISTEN       0x03
#define NET_CMD_ACCEPT       0x04
#define NET_CMD_CONNECT      0x05
#define NET_CMD_SEND         0x06
#define NET_CMD_RECV         0x07
#define NET_CMD_SENDTO       0x08
#define NET_CMD_RECVFROM     0x09
#define NET_CMD_CLOSE        0x0A
#define NET_CMD_GETSOCKOPT   0x0B
#define NET_CMD_SETSOCKOPT   0x0C
#define NET_CMD_GETINFO      0x0D
#define NET_CMD_PING         0x0E

/* Socket types */
#define NET_SOCK_STREAM      0x01  /* TCP */
#define NET_SOCK_DGRAM       0x02  /* UDP */
#define NET_SOCK_RAW         0x03  /* Raw */

/* Address families */
#define NET_AF_INET          0x02  /* IPv4 */
#define NET_AF_INET6         0x0A  /* IPv6 */

/* Network status bits */
#define NET_STATUS_READY     0x01
#define NET_STATUS_BUSY      0x02
#define NET_STATUS_ERROR     0x04
#define NET_STATUS_CONNECTED 0x08
#define NET_STATUS_LISTENING 0x10

/* ============================================================================
 * FPU Coprocessor (0x00F06000)
 * ============================================================================ */
#define FPU_BASE             0x00F06000
#define FPU_OP_A_HI          (FPU_BASE + 0x00)  /* Operand A high 32 bits */
#define FPU_OP_A_LO          (FPU_BASE + 0x04)  /* Operand A low 32 bits */
#define FPU_OP_B_HI          (FPU_BASE + 0x08)  /* Operand B high 32 bits */
#define FPU_OP_B_LO          (FPU_BASE + 0x0C)  /* Operand B low 32 bits */
#define FPU_COMMAND          (FPU_BASE + 0x10)  /* Write: triggers operation */
#define FPU_STATUS           (FPU_BASE + 0x14)  /* Read: status flags */
#define FPU_RESULT_HI        (FPU_BASE + 0x18)  /* Read: result high 32 bits */
#define FPU_RESULT_LO        (FPU_BASE + 0x1C)  /* Read: result low 32 bits */
#define FPU_INT_RESULT       (FPU_BASE + 0x20)  /* R/W: integer result */
#define FPU_CONTROL          (FPU_BASE + 0x24)  /* R/W: control register */

/* FPU operations */
#define FPU_CMD_ADD          0x01  /* result = A + B */
#define FPU_CMD_SUB          0x02  /* result = A - B */
#define FPU_CMD_MUL          0x03  /* result = A * B */
#define FPU_CMD_DIV          0x04  /* result = A / B */
#define FPU_CMD_SQRT         0x05  /* result = sqrt(A) */
#define FPU_CMD_ABS          0x06  /* result = |A| */
#define FPU_CMD_NEG          0x07  /* result = -A */
#define FPU_CMD_SIN          0x08  /* result = sin(A) */
#define FPU_CMD_COS          0x09  /* result = cos(A) */
#define FPU_CMD_TAN          0x0A  /* result = tan(A) */
#define FPU_CMD_ATAN         0x0B  /* result = atan(A) */
#define FPU_CMD_ATAN2        0x0C  /* result = atan2(A, B) */
#define FPU_CMD_LOG          0x0D  /* result = ln(A) */
#define FPU_CMD_LOG10        0x0E  /* result = log10(A) */
#define FPU_CMD_EXP          0x0F  /* result = exp(A) */
#define FPU_CMD_POW          0x10  /* result = A^B */
#define FPU_CMD_FMOD         0x11  /* result = fmod(A, B) */
#define FPU_CMD_FLOOR        0x12  /* result = floor(A) */
#define FPU_CMD_CEIL         0x13  /* result = ceil(A) */
#define FPU_CMD_ROUND        0x14  /* result = round(A) */
#define FPU_CMD_ITOF         0x15  /* result = (double)INT_RESULT */
#define FPU_CMD_FTOI         0x16  /* INT_RESULT = (int)A */
#define FPU_CMD_CMP          0x17  /* INT_RESULT = cmp(A,B) → -1,0,1 */
#define FPU_CMD_ASIN         0x18  /* result = asin(A) */
#define FPU_CMD_ACOS         0x19  /* result = acos(A) */
#define FPU_CMD_SINH         0x1A  /* result = sinh(A) */
#define FPU_CMD_COSH         0x1B  /* result = cosh(A) */
#define FPU_CMD_TANH         0x1C  /* result = tanh(A) */
#define FPU_CMD_PI           0x20  /* result = π */
#define FPU_CMD_E            0x21  /* result = e */

/* FPU status bits */
#define FPU_S_READY          0x01
#define FPU_S_ZERO           0x02
#define FPU_S_NEG            0x04
#define FPU_S_INF            0x08
#define FPU_S_NAN            0x10
#define FPU_S_OVERFLOW       0x20
#define FPU_S_UNDERFLOW      0x40
#define FPU_S_DIVZERO        0x80

/* ============================================================================
 * Audio Device (0x00F07000) - Sound Blaster 16 compatible
 * ============================================================================ */
#define AUD_BASE             0x00F07000
#define AUD_COMMAND          (AUD_BASE + 0x00)  /* Write: audio command */
#define AUD_STATUS           (AUD_BASE + 0x04)  /* Read: status */
#define AUD_FORMAT           (AUD_BASE + 0x08)  /* R/W: audio format */
#define AUD_SAMPLE_RATE      (AUD_BASE + 0x0C)  /* R/W: sample rate Hz */
#define AUD_VOLUME           (AUD_BASE + 0x10)  /* R/W: master volume 0-255 */
#define AUD_VOLUME_L         (AUD_BASE + 0x14)  /* R/W: left volume */
#define AUD_VOLUME_R         (AUD_BASE + 0x18)  /* R/W: right volume */
#define AUD_BUF_ADDR         (AUD_BASE + 0x1C)  /* R/W: DMA buffer address */
#define AUD_BUF_SIZE         (AUD_BASE + 0x20)  /* R/W: buffer size */
#define AUD_BUF_POS          (AUD_BASE + 0x24)  /* Read: playback position */
#define AUD_CHANNELS         (AUD_BASE + 0x28)  /* R/W: 1=mono, 2=stereo */
#define AUD_BITS             (AUD_BASE + 0x2C)  /* R/W: 8 or 16 */
#define AUD_IRQ_AT           (AUD_BASE + 0x30)  /* R/W: IRQ trigger position */

/* SB16 DSP registers (offset 0x100) */
#define AUD_SB_RESET         (AUD_BASE + 0x100) /* Write 1 to reset DSP */
#define AUD_SB_READ          (AUD_BASE + 0x104) /* Read: DSP data */
#define AUD_SB_WRITE         (AUD_BASE + 0x108) /* Write: DSP command/data */
#define AUD_SB_WSTATUS       (AUD_BASE + 0x10C) /* Read: write status */
#define AUD_SB_RSTATUS       (AUD_BASE + 0x110) /* Read: read status */
#define AUD_SB_MIXER_ADDR    (AUD_BASE + 0x114) /* Write: mixer reg address */
#define AUD_SB_MIXER_DATA    (AUD_BASE + 0x118) /* R/W: mixer reg data */

/* Audio DMA buffer (offset 0x200, 2KB) */
#define AUD_DMA_BUFFER       (AUD_BASE + 0x200)
#define AUD_DMA_BUFFER_SIZE  2048

/* Audio commands */
#define AUD_CMD_INIT         0x01
#define AUD_CMD_PLAY         0x02
#define AUD_CMD_STOP         0x03
#define AUD_CMD_PAUSE        0x04
#define AUD_CMD_RESUME       0x05
#define AUD_CMD_SET_FORMAT   0x06
#define AUD_CMD_BEEP         0x07  /* freq in BUF_SIZE, dur_ms in BUF_POS */
#define AUD_CMD_TONE         0x08

/* Audio format values */
#define AUD_FMT_U8_MONO     0x00
#define AUD_FMT_U8_STEREO   0x01
#define AUD_FMT_S16_MONO    0x10
#define AUD_FMT_S16_STEREO  0x11

/* Audio status bits */
#define AUD_S_READY          0x01
#define AUD_S_PLAYING        0x02
#define AUD_S_PAUSED         0x04
#define AUD_S_BUF_EMPTY      0x08
#define AUD_S_IRQ            0x10
#define AUD_S_ERROR          0x80

/* ============================================================================
 * Video/GPU Device (0x00F08000)
 * ============================================================================ */
#define VID_BASE             0x00F08000
#define VID_COMMAND          (VID_BASE + 0x00)  /* Write: GPU command */
#define VID_STATUS           (VID_BASE + 0x04)  /* Read: status */
#define VID_MODE             (VID_BASE + 0x08)  /* R/W: video mode */
#define VID_WIDTH            (VID_BASE + 0x0C)  /* Read: screen width px */
#define VID_HEIGHT           (VID_BASE + 0x10)  /* Read: screen height px */
#define VID_BPP              (VID_BASE + 0x14)  /* Read: bits per pixel */
#define VID_PITCH            (VID_BASE + 0x18)  /* Read: bytes per scanline */
#define VID_FB_ADDR          (VID_BASE + 0x1C)  /* R/W: framebuffer address */
#define VID_FB_SIZE          (VID_BASE + 0x20)  /* Read: framebuffer size */
#define VID_CURSOR_X         (VID_BASE + 0x24)  /* R/W: text cursor X */
#define VID_CURSOR_Y         (VID_BASE + 0x28)  /* R/W: text cursor Y */
#define VID_FG_COLOR         (VID_BASE + 0x2C)  /* R/W: foreground color */
#define VID_BG_COLOR         (VID_BASE + 0x30)  /* R/W: background color */
#define VID_DRAW_X           (VID_BASE + 0x34)  /* R/W: draw X coordinate */
#define VID_DRAW_Y           (VID_BASE + 0x38)  /* R/W: draw Y coordinate */
#define VID_DRAW_W           (VID_BASE + 0x3C)  /* R/W: draw width */
#define VID_DRAW_H           (VID_BASE + 0x40)  /* R/W: draw height */
#define VID_DRAW_COLOR       (VID_BASE + 0x44)  /* R/W: draw color */
#define VID_SRC_X            (VID_BASE + 0x48)  /* R/W: blit source X */
#define VID_SRC_Y            (VID_BASE + 0x4C)  /* R/W: blit source Y */
#define VID_FONT_ADDR        (VID_BASE + 0x50)  /* R/W: font data address */
#define VID_SCROLL_Y         (VID_BASE + 0x54)  /* R/W: vertical scroll */
#define VID_VBLANK           (VID_BASE + 0x58)  /* Read: vblank counter */

/* Video palette (offset 0x100, 256 entries × 4 bytes) */
#define VID_PALETTE          (VID_BASE + 0x100)
#define VID_PALETTE_ENTRIES  256

/* Video commands */
#define VID_CMD_INIT         0x01  /* Initialize video mode */
#define VID_CMD_CLEAR        0x02  /* Clear framebuffer */
#define VID_CMD_FLIP         0x03  /* Refresh display */
#define VID_CMD_FILL_RECT    0x04  /* Fill rectangle */
#define VID_CMD_BLIT         0x05  /* Blit rectangle */
#define VID_CMD_DRAW_LINE    0x06  /* Draw line */
#define VID_CMD_DRAW_PIXEL   0x07  /* Draw single pixel */
#define VID_CMD_DRAW_CHAR    0x08  /* Draw character at cursor */
#define VID_CMD_DRAW_TEXT    0x09  /* Draw text string */
#define VID_CMD_SET_PALETTE  0x0A  /* Set palette entry */
#define VID_CMD_SCROLL       0x0B  /* Scroll framebuffer */
#define VID_CMD_HLINE        0x0C  /* Fast horizontal line */
#define VID_CMD_VLINE        0x0D  /* Fast vertical line */
#define VID_CMD_CIRCLE       0x0E  /* Draw circle */
#define VID_CMD_FILL_CIRCLE  0x0F  /* Fill circle */

/* Video modes */
#define VID_MODE_TEXT_80x25  0x03  /* CGA text (80×25, 16 colors) */
#define VID_MODE_320x200     0x13  /* VGA mode 13h (256 colors) */
#define VID_MODE_320x240     0x50  /* 320×240 RGB565 */
#define VID_MODE_480x320     0x51  /* 480×320 RGB565 (SPI LCD) */
#define VID_MODE_640x480     0x52  /* 640×480 256 colors */
#define VID_MODE_800x480     0x53  /* 800×480 RGB565 (DSI LCD) */

/* Video status bits */
#define VID_S_READY          0x01
#define VID_S_VBLANK         0x02
#define VID_S_BUSY           0x04
#define VID_S_TEXT_MODE      0x08
#define VID_S_GFX_MODE       0x10

/* ============================================================================
 * DMA Controller (0x00F03000)
 * ============================================================================ */
#define DMA_BASE             0x00F03000
#define DMA_CONTROL          (DMA_BASE + 0x00)  /* Write: control/command */
#define DMA_SRC_ADDR         (DMA_BASE + 0x04)  /* R/W: source address */
#define DMA_DST_ADDR         (DMA_BASE + 0x08)  /* R/W: destination address */
#define DMA_LENGTH           (DMA_BASE + 0x0C)  /* R/W: transfer length */
#define DMA_STATUS           (DMA_BASE + 0x10)  /* Read: transfer status */

/* DMA commands */
#define DMA_CMD_START        0x01
#define DMA_CMD_ABORT        0x02
#define DMA_CMD_M68K_TO_ARM  0x10
#define DMA_CMD_ARM_TO_M68K  0x20

/* ============================================================================
 * Timer Device (0x00F05000)
 * ============================================================================ */
#define TIMER_BASE           0x00F05000
#define TIMER_COUNT          (TIMER_BASE + 0x00)  /* Read: tick counter */
#define TIMER_CONTROL        (TIMER_BASE + 0x04)  /* R/W: control register */

/* ============================================================================
 * Interrupt Controller (0x00F04000)
 * ============================================================================ */
#define IRQ_BASE             0x00F04000
#define IRQ_ENABLE           (IRQ_BASE + 0x00)   /* R/W: IRQ enable mask */
#define IRQ_PENDING          (IRQ_BASE + 0x04)   /* Read: pending IRQs */
#define IRQ_ACK              (IRQ_BASE + 0x08)   /* Write: acknowledge IRQ */

#endif /* M68K_IO_H */
