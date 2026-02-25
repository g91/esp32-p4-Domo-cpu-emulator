/*
 * z80_emulator.h - CP/M 2.2 machine emulator on Z80 CPU
 *
 * Provides a complete CP/M 2.2 environment:
 *  - Z80 CPU emulation via z80_cpu.c
 *  - 64KB address space with CP/M memory layout
 *  - BDOS function calls (0-40) intercepted via CALL 0x0005 trap
 *  - Virtual disk images stored as flat files on SD card in /sdcard/cpm/
 *  - Console I/O to LCD + UART
 *  - Keyboard input from PS/2, USB, or UART
 *
 * CP/M 2.2 Memory Map:
 *  0x0000 - 0x00FF  : Zero-page (warm-boot JP at 0x0000, BDOS JP at 0x0005)
 *  0x0100 - 0xCFFF  : TPA  - Transient Program Area (user programs load here)
 *  0xD000 - 0xEBFF  : Upper TPA / data area
 *  0xEC00 - 0xFAFF  : BDOS  (implemented as trap)
 *  0xFB00 - 0xFEFF  : BIOS  (implemented as trap)
 *  0xFF00 - 0xFFFF  : BIOS scratchpad
 *
 * Disk images:  /sdcard/cpm/a.img  (standard 8" floppy: 77*26*128 = 256256 bytes)
 *               /sdcard/cpm/b.img, c.img, d.img  (optional)
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =====================================================================
 * CP/M constants
 * ===================================================================== */

/* Default disk geometry (standard IBM 8" floppy compatible) */
#define CPM_TRACKS      77
#define CPM_SECTORS     26
#define CPM_SECTOR_SZ   128
#define CPM_DISK_SZ     (CPM_TRACKS * CPM_SECTORS * CPM_SECTOR_SZ)   /* 256,256 bytes */

/* Number of drives supported (A: through D:) */
#define CPM_MAX_DRIVES  4

/* CP/M address constants */
#define CPM_ADDR_ZERO   0x0000  /* Warm-boot / RST 0 */
#define CPM_ADDR_BDOS   0x0005  /* BDOS entry point (trap here) */
#define CPM_ADDR_TPA    0x0100  /* Transient Program Area start */
#define CPM_ADDR_FCB    0x005C  /* Default FCB in zero page */
#define CPM_ADDR_DMA    0x0080  /* Default DMA buffer */
#define CPM_RAM_SIZE    0x10000 /* Full 64KB address space */

/* CP/M BDOS function numbers */
#define BDOS_SYSRESET     0   /* System Reset */
#define BDOS_CONIN        1   /* Console Input */
#define BDOS_CONOUT       2   /* Console Output */
#define BDOS_READER       3   /* Reader Input */
#define BDOS_PUNCH        4   /* Punch Output */
#define BDOS_LIST         5   /* List Output */
#define BDOS_DIRCONIO     6   /* Direct Console I/O */
#define BDOS_GETIOBYTE    7   /* Get I/O Byte */
#define BDOS_SETIOBYTE    8   /* Set I/O Byte */
#define BDOS_PRINTSTR     9   /* Print String (DE = $ terminated) */
#define BDOS_READBUF      10  /* Read Console Buffer */
#define BDOS_CONSTAT      11  /* Get Console Status */
#define BDOS_GETVER       12  /* Return Version Number */
#define BDOS_RESETDSK     13  /* Reset Disk */
#define BDOS_SELDSK       14  /* Select Disk */
#define BDOS_OPEN         15  /* Open File */
#define BDOS_CLOSE        16  /* Close File */
#define BDOS_SFIRST       17  /* Search First */
#define BDOS_SNEXT        18  /* Search Next */
#define BDOS_ERASE        19  /* Erase File */
#define BDOS_READSEQ      20  /* Read Sequential */
#define BDOS_WRITESEQ     21  /* Write Sequential */
#define BDOS_MAKE         22  /* Make File */
#define BDOS_RENAME       23  /* Rename File */
#define BDOS_LOGINVEC     24  /* Return Login Vector */
#define BDOS_CURDISK      25  /* Return Current Disk */
#define BDOS_SETDMA       26  /* Set DMA Address */
#define BDOS_ALLOCVEC     27  /* Get Allocation Vector */
#define BDOS_WPROTDSK     28  /* Write Protect Disk */
#define BDOS_ROTVEC       29  /* Get Read-Only Vector */
#define BDOS_SETATTR      30  /* Set File Attributes */
#define BDOS_DISKPARMS    31  /* Get Disk Parameters */
#define BDOS_USERCODE     32  /* Set/Get User Code */
#define BDOS_READRND      33  /* Read Random */
#define BDOS_WRITERND     34  /* Write Random */
#define BDOS_FILESIZE     35  /* Compute File Size */
#define BDOS_SETRND       36  /* Set Random Record */
#define BDOS_RESETDRV     37  /* Reset Drive */
#define BDOS_WRITEZERO    40  /* Write Random With Zero Fill */

/* =====================================================================
 * Configuration
 * ===================================================================== */
typedef struct {
    const char  *disk_dir;          /* Directory for disk images, e.g. "/sdcard/cpm" */
    const char  *prog_path;         /* Optional: auto-load this .com file after boot */
    bool         enable_lcd;        /* Show output on LCD if available */
    uint32_t     cpu_freq_hz;       /* Target CPU frequency (0 = maximum) */
} z80_emu_config_t;

/* =====================================================================
 * API
 * ===================================================================== */

/** Initialize the Z80/CP/M emulator. Must be called before z80_emu_run(). */
esp_err_t z80_emu_init(const z80_emu_config_t *config);

/** Run CP/M until it exits or is stopped. Blocks the calling task. */
void z80_emu_run(void);

/** Signal the emulator to stop (from another task / ISR). */
void z80_emu_stop(void);

/** Deinitialize and free all resources. */
void z80_emu_deinit(void);

/** Returns true if the emulator is currently running. */
bool z80_emu_is_running(void);

/** Load a .COM file into the TPA (0x0100). Call before z80_emu_run(). */
esp_err_t z80_emu_load_com(const char *path);

/** Dump current CPU state to console. */
void z80_emu_dump_state(void);

#ifdef __cplusplus
}
#endif
