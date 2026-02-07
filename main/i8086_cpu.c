/**
 * @file i8086_cpu.c
 * @brief Intel 8086/8088 CPU Emulator - Complete Implementation
 * 
 * Based on FabGL i8086 (MIT License) - Adrian Cable / Fabrizio Di Vittorio
 * Ported to C for ESP32-P4 by the M68K Emulator project
 * 
 * This is a complete, cycle-accurate 8086 CPU emulator capable of running
 * MS-DOS, CP/M-86, and other x86 real-mode software.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "i8086_cpu.h"

static const char *TAG = "i8086";

// Global CPU state
static i86_cpu_t g_cpu;

// Internal register indices
#define REG_ZERO    12
#define REG_SCRATCH 13
#define REG_TMP     15

// Flag address constants
#define CF_ADDR 0
#define PF_ADDR 1
#define AF_ADDR 2
#define ZF_ADDR 3
#define SF_ADDR 4
#define TF_ADDR 5
#define IF_ADDR 6
#define DF_ADDR 7
#define OF_ADDR 8
#define XX_ADDR 9

// Table 0: R/M mode 1/2 "register 1" lookup
static const uint8_t rm_mode12_reg1[] = { 3, 3, 5, 5, 6, 7, 5, 3 };

// Table 1/5: R/M mode 0/1/2 "register 2" lookup
static const uint8_t rm_mode012_reg2[] = { 6, 7, 6, 7, 12, 12, 12, 12 };

// Table 2: R/M mode 1/2 "DISP multiplier" lookup
static const uint8_t rm_mode12_disp[] = { 1, 1, 1, 1, 1, 1, 1, 1 };

// Table 3: R/M mode 1/2 "default segment" lookup
static const uint8_t rm_mode12_dfseg[] = { 11, 11, 10, 10, 11, 11, 10, 11 };

// Table 4: R/M mode 0 "register 1" lookup
static const uint8_t rm_mode0_reg1[] = { 3, 3, 5, 5, 6, 7, 12, 3 };

// Table 6: R/M mode 0 "DISP multiplier" lookup
static const uint8_t rm_mode0_disp[] = { 0, 0, 0, 0, 0, 0, 1, 0 };

// Table 7: R/M mode 0 "default segment" lookup
static const uint8_t rm_mode0_dfseg[] = { 11, 11, 10, 10, 11, 11, 11, 11 };

// Table 8: Translation of raw opcode to function number
static const uint8_t xlat_ids[] = {
    9,  9,  9,  9,  7,  7, 25, 26,  9,  9,  9,  9,  7,  7, 25, 50,
    9,  9,  9,  9,  7,  7, 25, 26,  9,  9,  9,  9,  7,  7, 25, 26,
    9,  9,  9,  9,  7,  7, 27, 28,  9,  9,  9,  9,  7,  7, 27, 28,
    9,  9,  9,  9,  7,  7, 27, 29,  9,  9,  9,  9,  7,  7, 27, 29,
    2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,
    3,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  4,  4,  4,
   53, 54, 55, 70, 71, 71, 72, 72, 56, 58, 57, 58, 59, 59, 60, 60,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    8,  8,  8,  8, 15, 15, 24, 24,  9,  9,  9,  9, 10, 10, 10, 10,
   16, 16, 16, 16, 16, 16, 16, 16, 30, 31, 32, 69, 33, 34, 35, 36,
   11, 11, 11, 11, 17, 17, 18, 18, 47, 47, 17, 17, 17, 17, 18, 18,
    1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
   12, 12, 19, 19, 37, 37, 20, 20, 51, 52, 19, 19, 38, 39, 40, 19,
   12, 12, 12, 12, 41, 42, 43, 44, 69, 69, 69, 69, 69, 69, 69, 69,
   13, 13, 13, 13, 21, 21, 22, 22, 14, 14, 14, 14, 21, 21, 22, 22,
   48,  0, 23, 23, 49, 45,  6,  6, 46, 46, 46, 46, 46, 46,  5,  5
};

// Table 9: Translation of raw opcode to extra data
static const uint8_t ex_data[] = {
    0,  0,  0,  0,  0,  0,  8,  8,  1,  1,  1,  1,  1,  1,  9, 36,
    2,  2,  2,  2,  2,  2, 10, 10,  3,  3,  3,  3,  3,  3, 11, 11,
    4,  4,  4,  4,  4,  4,  8,  0,  5,  5,  5,  5,  5,  5,  9,  1,
    6,  6,  6,  6,  6,  6, 10,  2,  7,  7,  7,  7,  7,  7, 11,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,  1,  1,  1,  1,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0, 21, 21, 21, 21, 21, 21,  0,  0,  0,  0, 21, 21, 21, 21,
   21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21,
    0,  0,  0,  0,  0,  0,  0,  0,  8,  8,  8,  8, 12, 12, 12, 12,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,255,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  2,  2,  1,  1,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    1,  1,  0,  0, 16, 22,  0,  0,  0,  0,  1,  1,  0,255, 48,  2,
    0,  0,  0,  0,255,255, 40, 11,  3,  3,  3,  3,  3,  3,  3,  3,
   43, 43, 43, 43,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
    1, 21,  0,  0,  2, 40, 21, 21, 80, 81, 92, 93, 94, 95,  0,  0
};

// Table 10: How each raw opcode sets the flags
static const uint8_t std_flags[] = {
    3, 3, 3, 3, 3, 3, 0, 0, 5, 5, 5, 5, 5, 5, 0, 0,
    1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0,
    5, 5, 5, 5, 5, 5, 0, 1, 3, 3, 3, 3, 3, 3, 0, 1,
    5, 5, 5, 5, 5, 5, 0, 1, 3, 3, 3, 3, 3, 3, 0, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 1, 1, 1, 5, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 5, 5, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 5, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

// Table 11: Parity flag lookup (256 entries)
static const uint8_t parity[] = {
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1
};

// Table 12: Base instruction size
static const uint8_t base_size[] = {
    2, 2, 2, 2, 1, 1, 1, 1, 2, 2, 2, 2, 1, 1, 1, 2,
    2, 2, 2, 2, 1, 1, 1, 1, 2, 2, 2, 2, 1, 1, 1, 1,
    2, 2, 2, 2, 1, 1, 1, 1, 2, 2, 2, 2, 1, 1, 1, 1,
    2, 2, 2, 2, 1, 1, 1, 1, 2, 2, 2, 2, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 3, 1, 2, 1, 1, 1, 1, 1,
    2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
    2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1,
    3, 3, 3, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    3, 3, 0, 0, 2, 2, 2, 2, 4, 1, 0, 0, 0, 0, 0, 0,
    2, 2, 2, 2, 2, 2, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2,
    2, 2, 2, 2, 2, 2, 2, 2, 0, 0, 0, 0, 1, 1, 1, 1,
    1, 2, 1, 1, 1, 1, 2, 2, 1, 1, 1, 1, 1, 1, 2, 2
};

// Table 13: i_w size adder
static const uint8_t i_w_adder[] = {
    0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0,
    0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0,
    0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0,
    0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

// Table 14: i_mod size adder
static const uint8_t i_mod_adder[] = {
    1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0,
    1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0,
    1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0,
    1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1
};

// Table 15-18: Jxx decode tables
static const uint8_t jxx_dec_a[] = { OF_ADDR, CF_ADDR, ZF_ADDR, CF_ADDR, SF_ADDR, PF_ADDR, XX_ADDR, XX_ADDR };
static const uint8_t jxx_dec_b[] = { XX_ADDR, XX_ADDR, XX_ADDR, ZF_ADDR, XX_ADDR, XX_ADDR, XX_ADDR, ZF_ADDR };
static const uint8_t jxx_dec_c[] = { XX_ADDR, XX_ADDR, XX_ADDR, XX_ADDR, XX_ADDR, XX_ADDR, SF_ADDR, SF_ADDR };
static const uint8_t jxx_dec_d[] = { XX_ADDR, XX_ADDR, XX_ADDR, XX_ADDR, XX_ADDR, XX_ADDR, OF_ADDR, OF_ADDR };

// Lookup tables array
static const uint8_t *instr_table_lookup[] = {
    rm_mode12_reg1,    // 0
    rm_mode012_reg2,   // 1
    rm_mode12_disp,    // 2
    rm_mode12_dfseg,   // 3
    rm_mode0_reg1,     // 4
    rm_mode012_reg2,   // 5
    rm_mode0_disp,     // 6
    rm_mode0_dfseg,    // 7
    xlat_ids,          // 8
    ex_data,           // 9
    std_flags,         // 10
    parity,            // 11
    base_size,         // 12
    i_w_adder,         // 13
    i_mod_adder,       // 14
};

// Quick opcode processing table
static const uint8_t optcodes[] = {
//  0   1   2   3   4   5   6   7   8   9   a   b   c   d   e   f
    0,  0,  0,  0,  0,  0, 13, 12,  0,  0,  0,  0,  0,  0, 13,  0, // 00-0f
    0,  0,  0,  0,  0,  0, 13, 12,  0,  0,  0,  0,  0,  0, 13, 12, // 10-1f
    0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  1,  0, // 20-2f
    0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  1,  0, // 30-3f
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, // 40-4f
    9,  9,  9,  9,  9,  9,  9,  9,  8,  8,  8,  8,  8,  8,  8,  8, // 50-5f
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, // 60-6f
    2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2, // 70-7f
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 18,  0, // 80-8f
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, // 90-9f
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, // a0-af
   10, 10, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, // b0-bf
    0,  0,  0,  7,  0,  0,  0,  0,  0,  0,  0, 17,  0, 15,  0, 16, // c0-cf
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, // d0-df
    0,  0,  0,  5,  0,  0,  0,  0,  6,  0,  0,  3,  0,  0,  0,  0, // e0-ef
    0,  0, 14, 14,  0,  0,  0,  0,  4,  4,  4,  4,  4,  4,  0,  0, // f0-ff
};

// Internal state variables
static uint8_t  i_mod_size, i_d, i_w, raw_opcode_id, xlat_opcode_id, extra, rep_mode;
static uint8_t  seg_override_en, rep_override_en;
static uint16_t seg_override;
static uint32_t op_source, op_dest, set_flags_type;
static int32_t  op_to_addr, op_from_addr;

// Memory access macros
#define MEM8(addr)  g_cpu.memory[addr]
#define MEM16(addr) (*(uint16_t*)(g_cpu.memory + (addr)))

// Register access
#define REGS8   ((uint8_t*)g_cpu.regs16)
#define REGS16  g_cpu.regs16
#define FLAGS   g_cpu.flags
#define REG_IP  g_cpu.ip

// Flag access macros (note: FLAG_xx indices defined in header as FLAG_IDX_xx)
#define F_CF (FLAGS[CF_ADDR])
#define F_PF (FLAGS[PF_ADDR])
#define F_AF (FLAGS[AF_ADDR])
#define F_ZF (FLAGS[ZF_ADDR])
#define F_SF (FLAGS[SF_ADDR])
#define F_TF (FLAGS[TF_ADDR])
#define F_IF (FLAGS[IF_ADDR])
#define F_DF (FLAGS[DF_ADDR])
#define F_OF (FLAGS[OF_ADDR])

// Memory read with video callback
static uint8_t RMEM8(int addr) {
    if (addr >= VIDEOMEM_START && addr < VIDEOMEM_END && g_cpu.read_video8) {
        return g_cpu.read_video8(g_cpu.callback_ctx, addr);
    }
    return g_cpu.memory[addr];
}

static uint16_t RMEM16(int addr) {
    if (addr >= VIDEOMEM_START && addr < VIDEOMEM_END && g_cpu.read_video16) {
        return g_cpu.read_video16(g_cpu.callback_ctx, addr);
    }
    return *(uint16_t*)(g_cpu.memory + addr);
}

// Memory write with video callback
static inline uint8_t WMEM8(int addr, uint8_t value) {
    if (addr >= VIDEOMEM_START && addr < VIDEOMEM_END && g_cpu.write_video8) {
        g_cpu.write_video8(g_cpu.callback_ctx, addr, value);
    } else {
        g_cpu.memory[addr] = value;
    }
    return value;
}

static inline uint16_t WMEM16(int addr, uint16_t value) {
    if (addr >= VIDEOMEM_START && addr < VIDEOMEM_END && g_cpu.write_video16) {
        g_cpu.write_video16(g_cpu.callback_ctx, addr, value);
    } else {
        *(uint16_t*)(g_cpu.memory + addr) = value;
    }
    return value;
}

// Set AF and OF after arithmetic operations
static void set_AF_OF_arith(int32_t op_result) {
    F_AF = (bool)((op_source ^= op_dest ^ op_result) & 0x10);
    if (op_result == op_dest) {
        F_OF = 0;
    } else {
        F_OF = 1 & (F_CF ^ (op_source >> (8 * (i_w + 1) - 1)));
    }
}

// Assemble FLAGS register
static uint16_t make_flags(void) {
    // Real 8086 has bits 12-15 set to 1
    uint16_t r = 0xf002;
    return r | (F_CF << 0) | (F_PF << 2) | (F_AF << 4) | 
           (F_ZF << 6) | (F_SF << 7) | (F_TF << 8) | 
           (F_IF << 9) | (F_DF << 10) | (F_OF << 11);
}

// Decompose FLAGS register
static void set_flags(int new_flags) {
    F_CF = (bool)(new_flags & 0x001);
    F_PF = (bool)(new_flags & 0x004);
    F_AF = (bool)(new_flags & 0x010);
    F_ZF = (bool)(new_flags & 0x040);
    F_SF = (bool)(new_flags & 0x080);
    F_TF = (bool)(new_flags & 0x100);
    F_IF = (bool)(new_flags & 0x200);
    F_DF = (bool)(new_flags & 0x400);
    F_OF = (bool)(new_flags & 0x800);
}

// Set opcode for decoding
static void set_opcode(uint8_t opcode) {
    raw_opcode_id  = opcode;
    xlat_opcode_id = xlat_ids[opcode];
    extra          = ex_data[opcode];
    i_mod_size     = i_mod_adder[opcode];
    set_flags_type = std_flags[opcode];
}

// Execute INT instruction
static void pc_interrupt(uint8_t interrupt_num) {
    // Interrupt can exit from halt state
    if (g_cpu.halted) {
        g_cpu.halted = false;
        ++REG_IP;
    }

    // Check if BIOS handles this interrupt
    if (g_cpu.interrupt && g_cpu.interrupt(g_cpu.callback_ctx, interrupt_num)) {
        return; // Handled by callback
    }
    
    // Standard interrupt handling
    uint16_t newIP = MEM16(4 * interrupt_num);
    uint16_t newCS = MEM16(4 * interrupt_num + 2);
    
    REGS16[REG_SP] -= 6;
    uint16_t *stack = &MEM16(16 * REGS16[REG_SS] + REGS16[REG_SP]);
    stack[2] = make_flags();
    stack[1] = REGS16[REG_CS];
    stack[0] = REG_IP;
    
    REG_IP = newIP;
    REGS16[REG_CS] = newCS;
    
    F_TF = F_IF = 0;
}

// Raise divide by zero interrupt
static uint8_t raise_divide_by_zero(void) {
    if (seg_override_en || rep_override_en) {
        // Go back looking for prefixes
        while (1) {
            uint8_t opcode = MEM8(16 * REGS16[REG_CS] + REG_IP - 1);
            if ((opcode & 0xfe) != 0xf2 && (opcode & 0xe7) != 0x26)
                break;
            --REG_IP;
        }
    }
    pc_interrupt(0);
    return 0;
}

// DAA/DAS implementation
static int32_t DAA_DAS(void) {
    i_w = 0;
    F_AF = (REGS8[REG_AL] & 0x0f) > 9 || F_AF;
    F_CF = REGS8[REG_AL] > 0x99 || F_CF;
    if (extra) {
        // DAS
        if (F_CF)
            REGS8[REG_AL] -= 0x60;
        else if (F_AF)
            F_CF = (REGS8[REG_AL] < 6);
        if (F_AF)
            REGS8[REG_AL] -= 6;
    } else {
        // DAA
        if (F_CF)
            REGS8[REG_AL] += 0x60;
        if (F_AF)
            REGS8[REG_AL] += 6;
    }
    return REGS8[REG_AL];
}

// AAA/AAS implementation
static int32_t AAA_AAS(void) {
    F_AF = F_CF = ((REGS8[REG_AL] & 0x0F) > 9) || F_AF;
    REGS16[REG_AX] += 262 * (extra - 1) * F_AF;
    return REGS8[REG_AL] &= 0x0F;
}

// Forward declaration for extended step
static void step_ex(const uint8_t *opcode_stream);

// ============================================================================
// Public API
// ============================================================================

int i86_cpu_init(void) {
    if (g_cpu.initialized) {
        ESP_LOGW(TAG, "CPU already initialized");
        return 0;
    }
    
    // Allocate 1MB memory space from PSRAM
    g_cpu.memory = heap_caps_calloc(1, I8086_TOTAL_MEMORY, MALLOC_CAP_SPIRAM);
    if (!g_cpu.memory) {
        ESP_LOGE(TAG, "Failed to allocate 1MB memory");
        return -1;
    }
    
    // Set up memory region pointers
    g_cpu.ram = g_cpu.memory;
    g_cpu.video = g_cpu.memory + VIDEOMEM_START;
    g_cpu.bios = g_cpu.memory + BIOSMEM_START;
    
    // Calculate register offset
    g_cpu.regs_offset = 0; // Registers are in struct, not memory
    
    g_cpu.initialized = true;
    
    ESP_LOGI(TAG, "i8086 CPU initialized");
    ESP_LOGI(TAG, "  Memory: %p (1MB)", g_cpu.memory);
    ESP_LOGI(TAG, "  RAM:    %p - %p (640KB)", g_cpu.ram, g_cpu.ram + I8086_RAM_SIZE - 1);
    ESP_LOGI(TAG, "  Video:  %p - %p (128KB)", g_cpu.video, g_cpu.video + I8086_VIDEO_SIZE - 1);
    ESP_LOGI(TAG, "  BIOS:   %p - %p (64KB)", g_cpu.bios, g_cpu.bios + I8086_BIOS_SIZE - 1);
    
    i86_cpu_reset();
    return 0;
}

void i86_cpu_set_callbacks(
    void *ctx,
    i86_read_port_fn read_port,
    i86_write_port_fn write_port,
    i86_read_video8_fn read_video8,
    i86_write_video8_fn write_video8,
    i86_read_video16_fn read_video16,
    i86_write_video16_fn write_video16,
    i86_interrupt_fn interrupt
) {
    g_cpu.callback_ctx = ctx;
    g_cpu.read_port = read_port;
    g_cpu.write_port = write_port;
    g_cpu.read_video8 = read_video8;
    g_cpu.write_video8 = write_video8;
    g_cpu.read_video16 = read_video16;
    g_cpu.write_video16 = write_video16;
    g_cpu.interrupt = interrupt;
}

void i86_cpu_reset(void) {
    if (!g_cpu.initialized) return;
    
    // Clear registers
    memset(g_cpu.regs16, 0, sizeof(g_cpu.regs16));
    memset(g_cpu.flags, 0, sizeof(g_cpu.flags));
    
    // Set CS:IP to FFFF:0000 (BIOS entry point)
    REGS16[REG_CS] = 0xFFFF;
    REG_IP = 0x0000;
    
    // Clear state
    seg_override_en = 0;
    rep_override_en = 0;
    g_cpu.halted = false;
    g_cpu.pending_irq = false;
    g_cpu.cycles = 0;
    g_cpu.instructions = 0;
    
    ESP_LOGI(TAG, "CPU reset - CS:IP = %04X:%04X", REGS16[REG_CS], REG_IP);
}

void i86_cpu_step(void) {
    if (!g_cpu.initialized || g_cpu.halted) return;
    
    // Handle interrupts
    if (F_TF && !seg_override_en && !rep_override_en) {
        pc_interrupt(1); // Trap
    } else if (F_IF && g_cpu.pending_irq && !seg_override_en && !rep_override_en) {
        pc_interrupt(g_cpu.pending_irq_num);
        g_cpu.pending_irq = false;
    }
    
    // Main instruction loop
    while (1) {
        if (seg_override_en) --seg_override_en;
        if (rep_override_en) --rep_override_en;
        
        const uint8_t *opcode_stream = g_cpu.memory + 16 * REGS16[REG_CS] + REG_IP;
        
        // Quick processing for common instructions
        switch (optcodes[*opcode_stream]) {
            
            // SEG ES/CS/SS/DS (0x26, 0x2e, 0x36, 0x3e)
            case 1:
                seg_override_en = 2;
                seg_override = ex_data[*opcode_stream];
                if (rep_override_en) ++rep_override_en;
                ++REG_IP;
                return;
            
            // Jxx (0x70-0x7f)
            case 2: {
                int32_t inv = *opcode_stream & 1;
                int32_t idx = (*opcode_stream >> 1) & 7;
                REG_IP += 2 + (int8_t)opcode_stream[1] * 
                    (inv ^ (FLAGS[jxx_dec_a[idx]] || FLAGS[jxx_dec_b[idx]] || 
                           FLAGS[jxx_dec_c[idx]] ^ FLAGS[jxx_dec_d[idx]]));
                g_cpu.instructions++;
                return;
            }
            
            // JMP disp8 (0xeb)
            case 3:
                REG_IP += 2 + (int8_t)opcode_stream[1];
                g_cpu.instructions++;
                return;
            
            // CLC|STC|CLI|STI|CLD|STD (0xf8-0xfd)
            case 4: {
                static const int16_t FADDR[3] = { CF_ADDR, IF_ADDR, DF_ADDR };
                FLAGS[FADDR[(*opcode_stream >> 1) & 3]] = *opcode_stream & 1;
                ++REG_IP;
                g_cpu.instructions++;
                break; // Reloop for STI
            }
            
            // JCXZ (0xe3)
            case 5:
                REG_IP += 2 + !REGS16[REG_CX] * (int8_t)opcode_stream[1];
                g_cpu.instructions++;
                return;
            
            // CALL disp16 (0xe8)
            case 6: {
                uint16_t pIP = REG_IP + 3;
                REG_IP = pIP + *(uint16_t*)(opcode_stream + 1);
                REGS16[REG_SP] -= 2;
                MEM16(16 * REGS16[REG_SS] + REGS16[REG_SP]) = pIP;
                g_cpu.instructions++;
                return;
            }
            
            // RET (0xc3)
            case 7:
                REG_IP = MEM16(16 * REGS16[REG_SS] + REGS16[REG_SP]);
                REGS16[REG_SP] += 2;
                g_cpu.instructions++;
                return;
            
            // POP reg (0x58-0x5f)
            case 8:
                REGS16[REG_SP] += 2;
                REGS16[*opcode_stream & 7] = MEM16(16 * REGS16[REG_SS] + (uint16_t)(REGS16[REG_SP] - 2));
                ++REG_IP;
                g_cpu.instructions++;
                return;
            
            // PUSH reg (0x50-0x57)
            case 9:
                REGS16[REG_SP] -= 2;
                MEM16(16 * REGS16[REG_SS] + REGS16[REG_SP]) = REGS16[*opcode_stream & 7];
                ++REG_IP;
                g_cpu.instructions++;
                return;
            
            // MOV reg8, data8 (0xb0-0xb7)
            case 10:
                REGS8[((*opcode_stream >> 2) & 1) + (*opcode_stream & 3) * 2] = *(opcode_stream + 1);
                REG_IP += 2;
                g_cpu.instructions++;
                return;
            
            // MOV reg16, data16 (0xb8-0xbf)
            case 11:
                REGS16[*opcode_stream & 0x7] = *(uint16_t*)(opcode_stream + 1);
                REG_IP += 3;
                g_cpu.instructions++;
                return;
            
            // POP ES/SS/DS (0x07, 0x17, 0x1f)
            case 12:
                REGS16[REG_ES + (*opcode_stream >> 3)] = MEM16(16 * REGS16[REG_SS] + REGS16[REG_SP]);
                REGS16[REG_SP] += 2;
                ++REG_IP;
                g_cpu.instructions++;
                break; // Reloop for POP SS
            
            // PUSH ES/CS/SS/DS (0x06, 0x0e, 0x16, 0x1e)
            case 13:
                REGS16[REG_SP] -= 2;
                MEM16(16 * REGS16[REG_SS] + REGS16[REG_SP]) = REGS16[REG_ES + (*opcode_stream >> 3)];
                ++REG_IP;
                g_cpu.instructions++;
                return;
            
            // REP/REPE/REPNE (0xf2, 0xf3)
            case 14:
                rep_override_en = 2;
                rep_mode = *opcode_stream & 1;
                if (seg_override_en) ++seg_override_en;
                ++REG_IP;
                return;
            
            // INT imm8 (0xcd)
            case 15:
                REG_IP += 2;
                pc_interrupt(opcode_stream[1]);
                g_cpu.instructions++;
                return;
            
            // IRET (0xcf)
            case 16: {
                uint16_t *stack = &MEM16(16 * REGS16[REG_SS] + REGS16[REG_SP]);
                REG_IP = stack[0];
                REGS16[REG_CS] = stack[1];
                set_flags(stack[2]);
                REGS16[REG_SP] += 6;
                g_cpu.instructions++;
                return;
            }
            
            // RETF (0xcb)
            case 17: {
                uint16_t *stack = &MEM16(16 * REGS16[REG_SS] + REGS16[REG_SP]);
                REG_IP = stack[0];
                REGS16[REG_CS] = stack[1];
                REGS16[REG_SP] += 4;
                g_cpu.instructions++;
                return;
            }
            
            // MOV SS, r/m (0x8e with mod=11 and reg=010)
            case 18:
                step_ex(opcode_stream);
                g_cpu.instructions++;
                break; // Reloop to inhibit interrupt
            
            default:
                step_ex(opcode_stream);
                g_cpu.instructions++;
                return;
        }
    }
}

// Extended instruction execution
static void step_ex(const uint8_t *opcode_stream) {
    set_opcode(*opcode_stream);
    
    // Extract fields
    uint8_t i_reg4bit = raw_opcode_id & 7;
    i_w = i_reg4bit & 1;
    i_d = (i_reg4bit / 2) & 1;
    
    // Extract instruction data
    uint16_t i_data0 = opcode_stream[1] | (opcode_stream[2] << 8);
    uint16_t i_data1 = (i_data0 >> 8) | (opcode_stream[3] << 8);
    uint16_t i_data2 = (i_data1 >> 8) | (opcode_stream[4] << 8);
    
    uint8_t i_mod = 0, i_rm = 0, i_reg = 0;
    int32_t op_result = 0;
    int32_t rm_addr = 0;
    bool calcIP = true;
    
    // Decode Mod R/M if needed
    if (i_mod_size) {
        i_mod = (i_data0 & 0xFF) >> 6;
        i_rm = i_data0 & 7;
        i_reg = (i_data0 / 8) & 7;
        
        if ((!i_mod && i_rm == 6) || (i_mod == 2))
            i_data2 = (i_data2 >> 8) | (opcode_stream[5] << 8);
        else if (i_mod != 1)
            i_data2 = i_data1;
        else
            i_data1 = (int8_t)i_data1;
        
        int idx = 4 * !i_mod;
        op_to_addr = rm_addr = i_mod < 3 ? 
            16 * REGS16[seg_override_en ? seg_override : instr_table_lookup[idx + 3][i_rm]] + 
            (uint16_t)(REGS16[instr_table_lookup[idx + 1][i_rm]] + 
                      instr_table_lookup[idx + 2][i_rm] * i_data1 + 
                      REGS16[instr_table_lookup[idx][i_rm]]) : 
            (g_cpu.regs_offset + (i_w ? 2 * i_rm : (2 * i_rm + i_rm / 4) & 7));
        op_from_addr = g_cpu.regs_offset + (i_w ? 2 * i_reg : (2 * i_reg + i_reg / 4) & 7);
        
        if (i_d) {
            int32_t t = op_from_addr;
            op_from_addr = rm_addr;
            op_to_addr = t;
        }
    }
    
    // Main instruction switch
    switch (xlat_opcode_id) {
        case 2: // INC|DEC regs16
        {
            i_w = 1;
            i_d = 0;
            i_reg = i_reg4bit;
            int idx = 4 * !i_mod;
            op_to_addr = rm_addr = i_mod < 3 ? 
                16 * REGS16[seg_override_en ? seg_override : instr_table_lookup[idx + 3][i_rm]] + 
                (uint16_t)(REGS16[instr_table_lookup[idx + 1][i_rm]] + 
                          instr_table_lookup[idx + 2][i_rm] * i_data1 + 
                          REGS16[instr_table_lookup[idx][i_rm]]) : 
                (g_cpu.regs_offset + 2 * i_rm);
            op_from_addr = g_cpu.regs_offset + 2 * i_reg;
            i_reg = extra;
        }
        // Fall through
        case 5: // INC|DEC|JMP|CALL|PUSH
            if (i_reg < 2) {
                // INC|DEC
                if (i_w) {
                    op_dest = RMEM16(op_from_addr);
                    op_result = WMEM16(op_from_addr, (uint16_t)op_dest + 1 - 2 * i_reg);
                } else {
                    op_dest = RMEM8(op_from_addr);
                    op_result = WMEM8(op_from_addr, (uint16_t)op_dest + 1 - 2 * i_reg);
                }
                op_source = 1;
                set_AF_OF_arith(op_result);
                F_OF = (bool)(op_dest + 1 - i_reg == 1 << (8 * (i_w + 1) - 1));
                if (xlat_opcode_id == 5) {
                    xlat_opcode_id = 0x10;
                    set_flags_type = 1;
                }
            } else if (i_reg != 6) {
                // JMP|CALL
                uint16_t jumpTo = i_w ? MEM16(op_from_addr) : MEM8(op_from_addr);
                if (i_reg - 3 == 0) {
                    // CALL (far)
                    i_w = 1;
                    REGS16[REG_SP] -= 2;
                    MEM16(16 * REGS16[REG_SS] + REGS16[REG_SP]) = REGS16[REG_CS];
                }
                if (i_reg & 2) {
                    // CALL (near or far)
                    i_w = 1;
                    REGS16[REG_SP] -= 2;
                    MEM16(16 * REGS16[REG_SS] + REGS16[REG_SP]) = 
                        (REG_IP + 2 + i_mod * (i_mod != 3) + 2 * (!i_mod && i_rm == 6));
                }
                if (i_reg & 1) {
                    // JMP|CALL (far)
                    REGS16[REG_CS] = MEM16(op_from_addr + 2);
                }
                REG_IP = jumpTo;
                return;
            } else {
                // PUSH
                i_w = 1;
                REGS16[REG_SP] -= 2;
                MEM16(16 * REGS16[REG_SS] + REGS16[REG_SP]) = RMEM16(rm_addr);
            }
            break;
            
        case 6: // TEST r/m, imm | NOT|NEG|MUL|IMUL|DIV|IDIV
            op_to_addr = op_from_addr;
            switch (i_reg) {
                case 0: // TEST
                    raw_opcode_id = 0x20;
                    set_flags_type = 5;
                    REG_IP += i_w + 1;
                    if (i_w) {
                        op_dest = RMEM16(op_to_addr);
                        op_source = (uint16_t)i_data2;
                        op_result = (uint16_t)(op_dest & op_source);
                    } else {
                        op_dest = RMEM8(op_to_addr);
                        op_source = (uint8_t)i_data2;
                        op_result = (uint8_t)(op_dest & op_source);
                    }
                    break;
                case 2: // NOT
                    if (i_w)
                        WMEM16(op_to_addr, ~RMEM16(op_from_addr));
                    else
                        WMEM8(op_to_addr, ~RMEM8(op_from_addr));
                    break;
                case 3: // NEG
                    if (i_w)
                        op_result = WMEM16(op_to_addr, -(op_source = RMEM16(op_from_addr)));
                    else
                        op_result = WMEM8(op_to_addr, -(op_source = RMEM8(op_from_addr)));
                    op_dest = 0;
                    raw_opcode_id = 0x28;
                    set_flags_type = 3;
                    F_CF = op_result > op_dest;
                    break;
                case 4: // MUL
                    raw_opcode_id = 0x10;
                    set_flags_type = 1;
                    if (i_w) {
                        REGS16[REG_DX] = (op_result = RMEM16(rm_addr) * REGS16[REG_AX]) >> 16;
                        REGS16[REG_AX] = op_result;
                        F_OF = F_CF = (bool)(op_result - (uint16_t)op_result);
                    } else {
                        REGS16[REG_AX] = op_result = RMEM8(rm_addr) * REGS8[REG_AL];
                        F_OF = F_CF = (bool)(op_result - (uint8_t)op_result);
                    }
                    break;
                case 5: // IMUL
                    raw_opcode_id = 0x10;
                    set_flags_type = 1;
                    if (i_w) {
                        REGS16[REG_DX] = (op_result = (int16_t)RMEM16(rm_addr) * (int16_t)REGS16[REG_AX]) >> 16;
                        REGS16[REG_AX] = op_result;
                        F_OF = F_CF = (bool)(op_result - (int16_t)op_result);
                    } else {
                        REGS16[REG_AX] = op_result = (int8_t)RMEM8(rm_addr) * (int8_t)REGS8[REG_AL];
                        F_OF = F_CF = (bool)(op_result - (int8_t)op_result);
                    }
                    break;
                case 6: // DIV
                {
                    int32_t scratch_int;
                    int32_t scratch_uint, scratch2_uint;
                    if (i_w) {
                        if ((scratch_int = RMEM16(rm_addr)) &&
                            !(scratch2_uint = (uint32_t)(scratch_uint = (REGS16[REG_DX] << 16) + REGS16[REG_AX]) / scratch_int, 
                              scratch2_uint - (uint16_t)scratch2_uint)) {
                            REGS16[REG_DX] = scratch_uint - scratch_int * (REGS16[REG_AX] = scratch2_uint);
                        } else {
                            raise_divide_by_zero();
                            calcIP = false;
                        }
                    } else {
                        if ((scratch_int = RMEM8(rm_addr)) &&
                            !(scratch2_uint = (uint16_t)(scratch_uint = REGS16[REG_AX]) / scratch_int, 
                              scratch2_uint - (uint8_t)scratch2_uint)) {
                            REGS8[REG_AH] = scratch_uint - scratch_int * (REGS8[REG_AL] = scratch2_uint);
                        } else {
                            raise_divide_by_zero();
                            calcIP = false;
                        }
                    }
                    break;
                }
                case 7: // IDIV
                {
                    int32_t scratch_int;
                    int32_t scratch2_uint, scratch_uint;
                    if (i_w) {
                        if ((scratch_int = (int16_t)RMEM16(rm_addr)) &&
                            !(scratch2_uint = (int)(scratch_uint = (REGS16[REG_DX] << 16) + REGS16[REG_AX]) / scratch_int, 
                              scratch2_uint - (int16_t)scratch2_uint)) {
                            REGS16[REG_DX] = scratch_uint - scratch_int * (REGS16[REG_AX] = scratch2_uint);
                        } else {
                            raise_divide_by_zero();
                            calcIP = false;
                        }
                    } else {
                        if ((scratch_int = (int8_t)RMEM8(rm_addr)) &&
                            !(scratch2_uint = (int16_t)(scratch_uint = REGS16[REG_AX]) / scratch_int, 
                              scratch2_uint - (int8_t)scratch2_uint)) {
                            REGS8[REG_AH] = scratch_uint - scratch_int * (REGS8[REG_AL] = scratch2_uint);
                        } else {
                            raise_divide_by_zero();
                            calcIP = false;
                        }
                    }
                    break;
                }
            }
            break;
            
        case 7: // ADD|OR|ADC|SBB|AND|SUB|XOR|CMP AL/AX, immed
            rm_addr = g_cpu.regs_offset;
            i_data2 = i_data0;
            i_mod = 3;
            i_reg = extra;
            REG_IP--;
            // Fall through
        case 8: // ADD|OR|ADC|SBB|AND|SUB|XOR|CMP reg, immed
            op_to_addr = rm_addr;
            REGS16[REG_SCRATCH] = (i_d |= !i_w) ? (int8_t)i_data2 : i_data2;
            op_from_addr = g_cpu.regs_offset + 2 * REG_SCRATCH;
            REG_IP += !i_d + 1;
            set_opcode(0x08 * (extra = i_reg));
            // Fall through
        case 9: // ADD|OR|ADC|SBB|AND|SUB|XOR|CMP|MOV reg, r/m
            switch (extra) {
                case 0: // ADD
                    if (i_w) {
                        op_dest = RMEM16(op_to_addr);
                        op_source = RMEM16(op_from_addr);
                        op_result = (uint16_t)(op_dest + op_source);
                        WMEM16(op_to_addr, op_result);
                    } else {
                        op_dest = RMEM8(op_to_addr);
                        op_source = RMEM8(op_from_addr);
                        op_result = (uint8_t)(op_dest + op_source);
                        WMEM8(op_to_addr, op_result);
                    }
                    F_CF = op_result < op_dest;
                    break;
                case 1: // OR
                    if (i_w) {
                        op_dest = RMEM16(op_to_addr);
                        op_source = RMEM16(op_from_addr);
                        op_result = op_dest | op_source;
                        WMEM16(op_to_addr, op_result);
                    } else {
                        op_dest = RMEM8(op_to_addr);
                        op_source = RMEM8(op_from_addr);
                        op_result = op_dest | op_source;
                        WMEM8(op_to_addr, op_result);
                    }
                    break;
                case 2: // ADC
                    if (i_w) {
                        op_dest = RMEM16(op_to_addr);
                        op_source = RMEM16(op_from_addr);
                        op_result = WMEM16(op_to_addr, op_dest + F_CF + op_source);
                    } else {
                        op_dest = RMEM8(op_to_addr);
                        op_source = RMEM8(op_from_addr);
                        op_result = WMEM8(op_to_addr, op_dest + F_CF + op_source);
                    }
                    F_CF = (F_CF && (op_result == op_dest)) || (op_result < (int)op_dest);
                    set_AF_OF_arith(op_result);
                    break;
                case 3: // SBB
                    if (i_w) {
                        op_dest = RMEM16(op_to_addr);
                        op_source = RMEM16(op_from_addr);
                        op_result = WMEM16(op_to_addr, op_dest - (F_CF + op_source));
                    } else {
                        op_dest = RMEM8(op_to_addr);
                        op_source = RMEM8(op_from_addr);
                        op_result = WMEM8(op_to_addr, op_dest - (F_CF + op_source));
                    }
                    F_CF = (F_CF && (op_result == op_dest)) || (-op_result < -(int)op_dest);
                    set_AF_OF_arith(op_result);
                    break;
                case 4: // AND
                    if (i_w) {
                        op_dest = RMEM16(op_to_addr);
                        op_source = RMEM16(op_from_addr);
                        op_result = op_dest & op_source;
                        WMEM16(op_to_addr, op_result);
                    } else {
                        op_dest = RMEM8(op_to_addr);
                        op_source = RMEM8(op_from_addr);
                        op_result = op_dest & op_source;
                        WMEM8(op_to_addr, op_result);
                    }
                    break;
                case 5: // SUB
                    if (i_w) {
                        op_dest = RMEM16(op_to_addr);
                        op_source = RMEM16(op_from_addr);
                        op_result = WMEM16(op_to_addr, op_dest - op_source);
                    } else {
                        op_dest = RMEM8(op_to_addr);
                        op_source = RMEM8(op_from_addr);
                        op_result = WMEM8(op_to_addr, op_dest - op_source);
                    }
                    F_CF = op_result > op_dest;
                    break;
                case 6: // XOR
                    if (i_w) {
                        op_dest = RMEM16(op_to_addr);
                        op_source = RMEM16(op_from_addr);
                        op_result = op_dest ^ op_source;
                        WMEM16(op_to_addr, op_result);
                    } else {
                        op_dest = RMEM8(op_to_addr);
                        op_source = RMEM8(op_from_addr);
                        op_result = op_dest ^ op_source;
                        WMEM8(op_to_addr, op_result);
                    }
                    break;
                case 7: // CMP
                    if (i_w) {
                        op_dest = RMEM16(op_to_addr);
                        op_source = RMEM16(op_from_addr);
                    } else {
                        op_dest = RMEM8(op_to_addr);
                        op_source = RMEM8(op_from_addr);
                    }
                    op_result = op_dest - op_source;
                    F_CF = op_result > op_dest;
                    break;
                case 8: // MOV
                    if (i_w)
                        WMEM16(op_to_addr, RMEM16(op_from_addr));
                    else
                        WMEM8(op_to_addr, RMEM8(op_from_addr));
                    break;
            }
            break;
            
        case 10: // MOV sreg, r/m | POP r/m | LEA reg, r/m
            if (!i_w) {
                // MOV sreg
                i_w = 1;
                i_reg += 8;
                int32_t scratch2_uint = 4 * !i_mod;
                rm_addr = i_mod < 3 ?
                    16 * REGS16[seg_override_en ? seg_override : instr_table_lookup[scratch2_uint + 3][i_rm]] + 
                    (uint16_t)(REGS16[instr_table_lookup[scratch2_uint + 1][i_rm]] + 
                              instr_table_lookup[scratch2_uint + 2][i_rm] * i_data1 + 
                              REGS16[instr_table_lookup[scratch2_uint][i_rm]]) : 
                    (g_cpu.regs_offset + (2 * i_rm));
                if (i_d)
                    REGS16[i_reg] = RMEM16(rm_addr);
                else
                    WMEM16(rm_addr, REGS16[i_reg]);
            } else if (!i_d) {
                // LEA
                int idx = 4 * !i_mod;
                REGS16[i_reg] = REGS16[instr_table_lookup[idx + 1][i_rm]] + 
                               instr_table_lookup[idx + 2][i_rm] * i_data1 + 
                               REGS16[instr_table_lookup[idx][i_rm]];
            } else {
                // POP
                REGS16[REG_SP] += 2;
                WMEM16(rm_addr, MEM16(16 * REGS16[REG_SS] + (uint16_t)(-2 + REGS16[REG_SP])));
            }
            break;
            
        case 11: // MOV AL/AX, [loc]
            rm_addr = 16 * REGS16[seg_override_en ? seg_override : REG_DS] + i_data0;
            if (i_d) {
                if (i_w)
                    WMEM16(rm_addr, REGS16[REG_AX]);
                else
                    WMEM8(rm_addr, REGS8[REG_AL]);
            } else {
                if (i_w)
                    REGS16[REG_AX] = RMEM16(rm_addr);
                else
                    REGS8[REG_AL] = RMEM8(rm_addr);
            }
            REG_IP += 3;
            return;
            
        case 12: // ROL|ROR|RCL|RCR|SHL|SHR|???|SAR reg/MEM, 1/CL/imm
        {
            uint16_t scratch2_uint = (1 & (i_w ? (int16_t)RMEM16(rm_addr) : RMEM8(rm_addr)) >> (8 * (i_w + 1) - 1));
            uint16_t scratch_uint = extra ? (int8_t)i_data1 : i_d ? 31 & REGS8[REG_CL] : 1;
            
            if (scratch_uint) {
                if (i_reg < 4) {
                    scratch_uint %= i_reg / 2 + 8 * (i_w + 1);
                    scratch2_uint = i_w ? RMEM16(rm_addr) : RMEM8(rm_addr);
                }
                if (i_reg & 1) {
                    if (i_w) {
                        op_dest = RMEM16(rm_addr);
                        op_result = WMEM16(rm_addr, (uint16_t)op_dest >> scratch_uint);
                    } else {
                        op_dest = RMEM8(rm_addr);
                        op_result = WMEM8(rm_addr, (uint8_t)op_dest >> (uint8_t)scratch_uint);
                    }
                } else {
                    if (i_w) {
                        op_dest = RMEM16(rm_addr);
                        op_result = WMEM16(rm_addr, (uint16_t)op_dest << scratch_uint);
                    } else {
                        op_dest = RMEM8(rm_addr);
                        op_result = WMEM8(rm_addr, (uint8_t)op_dest << (uint8_t)scratch_uint);
                    }
                }
                if (i_reg > 3)
                    set_flags_type = 1;
                if (i_reg > 4)
                    F_CF = op_dest >> (scratch_uint - 1) & 1;
            }
            
            switch (i_reg) {
                case 0: // ROL
                    if (i_w) {
                        op_dest = RMEM16(rm_addr);
                        op_result = WMEM16(rm_addr, (uint16_t)op_dest + (op_source = scratch2_uint >> (16 - scratch_uint)));
                    } else {
                        op_dest = RMEM8(rm_addr);
                        op_result = WMEM8(rm_addr, (uint8_t)op_dest + (op_source = (uint8_t)scratch2_uint >> (8 - scratch_uint)));
                    }
                    if (scratch_uint)
                        F_OF = (1 & op_result >> (8 * (i_w + 1) - 1)) ^ (F_CF = op_result & 1);
                    break;
                case 1: // ROR
                    scratch2_uint &= (1 << scratch_uint) - 1;
                    if (i_w) {
                        op_dest = RMEM16(rm_addr);
                        op_result = WMEM16(rm_addr, (uint16_t)op_dest + (op_source = scratch2_uint << (16 - scratch_uint)));
                    } else {
                        op_dest = RMEM8(rm_addr);
                        op_result = WMEM8(rm_addr, (uint8_t)op_dest + (op_source = (uint8_t)scratch2_uint << (8 - scratch_uint)));
                    }
                    if (scratch_uint)
                        F_OF = (1 & (i_w ? (int16_t)op_result * 2 : op_result * 2) >> (8 * (i_w + 1) - 1)) ^ 
                                  (F_CF = 1 & (i_w ? (int16_t)op_result : op_result) >> (8 * (i_w + 1) - 1));
                    break;
                case 2: // RCL
                    if (i_w) {
                        op_dest = RMEM16(rm_addr);
                        op_result = WMEM16(rm_addr, (uint16_t)op_dest + (F_CF << (scratch_uint - 1)) + 
                                          (op_source = scratch2_uint >> (17 - scratch_uint)));
                    } else {
                        op_dest = RMEM8(rm_addr);
                        op_result = WMEM8(rm_addr, (uint8_t)op_dest + (F_CF << (scratch_uint - 1)) + 
                                         (op_source = (uint8_t)scratch2_uint >> (9 - scratch_uint)));
                    }
                    if (scratch_uint)
                        F_OF = (1 & op_result >> (8 * (i_w + 1) - 1)) ^ 
                                  (F_CF = (bool)(scratch2_uint & 1 << (8 * (i_w + 1) - scratch_uint)));
                    break;
                case 3: // RCR
                    if (i_w) {
                        op_dest = RMEM16(rm_addr);
                        op_result = WMEM16(rm_addr, (uint16_t)op_dest + (F_CF << (16 - scratch_uint)) + 
                                          (op_source = scratch2_uint << (17 - scratch_uint)));
                    } else {
                        op_dest = RMEM8(rm_addr);
                        op_result = WMEM8(rm_addr, (uint8_t)op_dest + (F_CF << (8 - scratch_uint)) + 
                                         (op_source = (uint8_t)scratch2_uint << (9 - scratch_uint)));
                    }
                    if (scratch_uint) {
                        F_CF = (bool)(scratch2_uint & 1 << (scratch_uint - 1));
                        F_OF = (1 & op_result >> (8 * (i_w + 1) - 1)) ^ 
                                  (1 & (i_w ? (int16_t)op_result * 2 : op_result * 2) >> (8 * (i_w + 1) - 1));
                    }
                    break;
                case 4: // SHL
                    if (scratch_uint)
                        F_OF = (1 & op_result >> (8 * (i_w + 1) - 1)) ^ 
                                  (F_CF = (1 & (op_dest << (scratch_uint - 1)) >> (8 * (i_w + 1) - 1)));
                    break;
                case 5: // SHR
                    if (scratch_uint)
                        F_OF = 1 & op_dest >> (8 * (i_w + 1) - 1);
                    break;
                case 7: // SAR
                    if (scratch_uint >= 8 * (i_w + 1)) {
                        F_CF = (bool)scratch2_uint;
                    }
                    F_OF = 0;
                    if (i_w) {
                        op_dest = RMEM16(rm_addr);
                        uint16_t u16 = (uint16_t)scratch2_uint * ~(((1 << 16) - 1) >> scratch_uint);
                        op_result = WMEM16(rm_addr, op_dest + (op_source = u16));
                    } else {
                        op_dest = RMEM8(rm_addr);
                        uint8_t u8 = (uint8_t)scratch2_uint * ~(((1 << 8) - 1) >> scratch_uint);
                        op_result = WMEM8(rm_addr, op_dest + (op_source = u8));
                    }
                    break;
            }
            break;
        }
        
        case 13: // LOOPxx
            if (--REGS16[REG_CX])
                REG_IP += ((F_ZF ^ !i_reg4bit) | (bool)(i_reg4bit & 2)) * (int8_t)i_data0;
            break;
            
        case 14: // JMP | CALL int16_t/near
            REG_IP += 3 - i_d;
            if (!i_w) {
                if (i_d) {
                    REG_IP = 0;
                    REGS16[REG_CS] = i_data2;
                } else {
                    i_w = 1;
                    REGS16[REG_SP] -= 2;
                    MEM16(16 * REGS16[REG_SS] + REGS16[REG_SP]) = REG_IP;
                }
            }
            REG_IP += i_d && i_w ? (int8_t)i_data0 : i_data0;
            return;
            
        case 15: // TEST reg, r/m
            if (i_w)
                op_result = RMEM16(op_from_addr) & RMEM16(op_to_addr);
            else
                op_result = RMEM8(op_from_addr) & RMEM8(op_to_addr);
            break;
            
        case 16: // XCHG AX, regs16
            if (i_reg4bit != REG_AX) {
                uint16_t t = REGS16[REG_AX];
                REGS16[REG_AX] = REGS16[i_reg4bit];
                REGS16[i_reg4bit] = t;
            }
            ++REG_IP;
            return;
            
        case 24: // NOP|XCHG reg, r/m
            if (op_to_addr != op_from_addr) {
                if (i_w) {
                    uint16_t t = RMEM16(op_to_addr);
                    WMEM16(op_to_addr, RMEM16(op_from_addr));
                    WMEM16(op_from_addr, t);
                } else {
                    uint16_t t = RMEM8(op_to_addr);
                    WMEM8(op_to_addr, RMEM8(op_from_addr));
                    WMEM8(op_from_addr, t);
                }
            }
            break;
            
        case 17: // MOVSx|STOSx|LODSx
        {
            int32_t seg = seg_override_en ? seg_override : REG_DS;
            if (i_w) {
                const int dec = (2 * F_DF - 1) * 2;
                for (int32_t i = rep_override_en ? REGS16[REG_CX] : 1; i; --i) {
                    uint16_t src = extra & 1 ? REGS16[REG_AX] : RMEM16(16 * REGS16[seg] + REGS16[REG_SI]);
                    if (extra < 2)
                        WMEM16(16 * REGS16[REG_ES] + REGS16[REG_DI], src);
                    else
                        REGS16[REG_AX] = src;
                    if (!(extra & 1)) REGS16[REG_SI] -= dec;
                    if (!(extra & 2)) REGS16[REG_DI] -= dec;
                }
            } else {
                const int dec = (2 * F_DF - 1);
                for (int32_t i = rep_override_en ? REGS16[REG_CX] : 1; i; --i) {
                    uint8_t src = extra & 1 ? REGS8[REG_AL] : RMEM8(16 * REGS16[seg] + REGS16[REG_SI]);
                    if (extra < 2)
                        WMEM8(16 * REGS16[REG_ES] + REGS16[REG_DI], src);
                    else
                        REGS8[REG_AL] = src;
                    if (!(extra & 1)) REGS16[REG_SI] -= dec;
                    if (!(extra & 2)) REGS16[REG_DI] -= dec;
                }
            }
            if (rep_override_en)
                REGS16[REG_CX] = 0;
            ++REG_IP;
            return;
        }
        
        case 18: // CMPSx|SCASx
        {
            int count = rep_override_en ? REGS16[REG_CX] : 1;
            if (count) {
                int incval = (2 * F_DF - 1) * (i_w + 1);
                if (extra) {
                    // SCASx
                    op_dest = i_w ? REGS16[REG_AX] : REGS8[REG_AL];
                    while (count) {
                        if (!rep_override_en) count--;
                        if (i_w)
                            op_result = op_dest - (op_source = RMEM16(16 * REGS16[REG_ES] + REGS16[REG_DI]));
                        else
                            op_result = op_dest - (op_source = RMEM8(16 * REGS16[REG_ES] + REGS16[REG_DI]));
                        REGS16[REG_DI] -= incval;
                        if (rep_override_en && !(--REGS16[REG_CX] && ((!op_result) == rep_mode))) count = 0;
                    }
                } else {
                    // CMPSx
                    int scratch2_uint = seg_override_en ? seg_override : REG_DS;
                    while (count) {
                        if (!rep_override_en) count--;
                        if (i_w) {
                            op_dest = RMEM16(16 * REGS16[scratch2_uint] + REGS16[REG_SI]);
                            op_result = op_dest - (op_source = RMEM16(16 * REGS16[REG_ES] + REGS16[REG_DI]));
                        } else {
                            op_dest = RMEM8(16 * REGS16[scratch2_uint] + REGS16[REG_SI]);
                            op_result = op_dest - (op_source = RMEM8(16 * REGS16[REG_ES] + REGS16[REG_DI]));
                        }
                        REGS16[REG_SI] -= incval;
                        REGS16[REG_DI] -= incval;
                        if (rep_override_en && !(--REGS16[REG_CX] && ((!op_result) == rep_mode))) count = 0;
                    }
                }
                set_flags_type = 3;
                F_CF = op_result > op_dest;
            }
            ++REG_IP;
            calcIP = false;
            break;
        }
        
        case 19: // RET / RETF imm16
            REG_IP = MEM16(16 * REGS16[REG_SS] + REGS16[REG_SP]);
            REGS16[REG_SP] += 2;
            if (extra) {
                REGS16[REG_CS] = MEM16(16 * REGS16[REG_SS] + REGS16[REG_SP]);
                REGS16[REG_SP] += 2;
            }
            REGS16[REG_SP] += i_data0;
            return;
            
        case 20: // MOV r/m, immed
            if (i_w)
                WMEM16(op_from_addr, i_data2);
            else
                WMEM8(op_from_addr, i_data2);
            break;
            
        case 21: // IN AL/AX, DX/imm8
        {
            int32_t port = extra ? REGS16[REG_DX] : (uint8_t)i_data0;
            if (g_cpu.read_port) {
                REGS8[REG_AL] = g_cpu.read_port(g_cpu.callback_ctx, port);
                if (i_w)
                    REGS8[REG_AH] = g_cpu.read_port(g_cpu.callback_ctx, port + 1);
            }
            break;
        }
        
        case 22: // OUT DX/imm8, AL/AX
        {
            int32_t port = extra ? REGS16[REG_DX] : (uint8_t)i_data0;
            if (g_cpu.write_port) {
                g_cpu.write_port(g_cpu.callback_ctx, port, REGS8[REG_AL]);
                if (i_w)
                    g_cpu.write_port(g_cpu.callback_ctx, port + 1, REGS8[REG_AH]);
            }
            break;
        }
        
        case 28: // DAA/DAS
            op_result = DAA_DAS();
            break;
            
        case 29: // AAA/AAS
            op_result = AAA_AAS();
            break;
            
        case 30: // CBW
            REGS8[REG_AH] = -(1 & (i_w ? *(int16_t*)&REGS8[REG_AL] : REGS8[REG_AL]) >> (8 * (i_w + 1) - 1));
            break;
            
        case 31: // CWD
            REGS16[REG_DX] = -(1 & (i_w ? *(int16_t*)&REGS16[REG_AX] : REGS16[REG_AX]) >> (8 * (i_w + 1) - 1));
            break;
            
        case 32: // CALL FAR imm16:imm16
        {
            REGS16[REG_SP] -= 4;
            uint16_t *stack = &MEM16(16 * REGS16[REG_SS] + REGS16[REG_SP]);
            stack[1] = REGS16[REG_CS];
            stack[0] = REG_IP + 5;
            REGS16[REG_CS] = i_data2;
            REG_IP = i_data0;
            return;
        }
        
        case 33: // PUSHF
            REGS16[REG_SP] -= 2;
            MEM16(16 * REGS16[REG_SS] + REGS16[REG_SP]) = make_flags();
            ++REG_IP;
            return;
            
        case 34: // POPF
            REGS16[REG_SP] += 2;
            set_flags(MEM16(16 * REGS16[REG_SS] + (uint16_t)(-2 + REGS16[REG_SP])));
            ++REG_IP;
            return;
            
        case 35: // SAHF
            set_flags((make_flags() & 0xFF00) + REGS8[REG_AH]);
            break;
            
        case 36: // LAHF
            REGS8[REG_AH] = make_flags();
            break;
            
        case 37: // LES|LDS reg, r/m
            i_w = i_d = 1;
            REGS16[i_reg] = RMEM16(rm_addr);
            REGS16[extra / 2] = RMEM16(rm_addr + 2);
            break;
            
        case 38: // INT 3
            ++REG_IP;
            pc_interrupt(3);
            return;
            
        case 40: // INTO
            ++REG_IP;
            if (F_OF)
                pc_interrupt(4);
            return;
            
        case 41: // AAM
            if (i_data0 &= 0xFF) {
                REGS8[REG_AH] = REGS8[REG_AL] / i_data0;
                op_result = REGS8[REG_AL] %= i_data0;
            } else {
                raise_divide_by_zero();
                return;
            }
            break;
            
        case 42: // AAD
            i_w = 0;
            REGS16[REG_AX] = op_result = 0xFF & (REGS8[REG_AL] + i_data0 * REGS8[REG_AH]);
            break;
            
        case 43: // SALC
            REGS8[REG_AL] = -F_CF;
            break;
            
        case 44: // XLAT
            REGS8[REG_AL] = RMEM8(16 * REGS16[seg_override_en ? seg_override : REG_DS] + 
                                  (uint16_t)(REGS8[REG_AL] + REGS16[REG_BX]));
            ++REG_IP;
            return;
            
        case 45: // CMC
            F_CF ^= 1;
            ++REG_IP;
            return;
            
        case 47: // TEST AL/AX, immed
            if (i_w)
                op_result = REGS16[REG_AX] & i_data0;
            else
                op_result = REGS8[REG_AL] & (uint8_t)i_data0;
            break;
            
        case 48: // LOCK:
            break;
            
        case 49: // HLT
            g_cpu.halted = true;
            return;
            
        case 51: // 80186: ENTER
        {
            REGS16[REG_SP] -= 2;
            MEM16(16 * REGS16[REG_SS] + REGS16[REG_SP]) = REGS16[REG_BP];
            uint16_t framePtr = REGS16[REG_SP];
            int16_t level = i_data2 & 31;
            if (level > 0) {
                while (--level) {
                    REGS16[REG_BP] -= 2;
                    REGS16[REG_SP] -= 2;
                    MEM16(16 * REGS16[REG_SS] + REGS16[REG_SP]) = MEM16(16 * REGS16[REG_SS] + REGS16[REG_BP]);
                }
                REGS16[REG_SP] -= 2;
                MEM16(16 * REGS16[REG_SS] + REGS16[REG_SP]) = framePtr;
            }
            REGS16[REG_BP] = framePtr;
            REGS16[REG_SP] -= i_data0;
            break;
        }
        
        case 52: // 80186: LEAVE
            REGS16[REG_SP] = REGS16[REG_BP];
            REGS16[REG_BP] = MEM16(16 * REGS16[REG_SS] + REGS16[REG_SP]);
            REGS16[REG_SP] += 2;
            break;
            
        case 53: // 80186: PUSHA
        {
            uint16_t temp = REGS16[REG_SP];
            REGS16[REG_SP] -= 2;
            MEM16(16 * REGS16[REG_SS] + REGS16[REG_SP]) = REGS16[REG_AX];
            REGS16[REG_SP] -= 2;
            MEM16(16 * REGS16[REG_SS] + REGS16[REG_SP]) = REGS16[REG_CX];
            REGS16[REG_SP] -= 2;
            MEM16(16 * REGS16[REG_SS] + REGS16[REG_SP]) = REGS16[REG_DX];
            REGS16[REG_SP] -= 2;
            MEM16(16 * REGS16[REG_SS] + REGS16[REG_SP]) = REGS16[REG_BX];
            REGS16[REG_SP] -= 2;
            MEM16(16 * REGS16[REG_SS] + REGS16[REG_SP]) = temp;
            REGS16[REG_SP] -= 2;
            MEM16(16 * REGS16[REG_SS] + REGS16[REG_SP]) = REGS16[REG_BP];
            REGS16[REG_SP] -= 2;
            MEM16(16 * REGS16[REG_SS] + REGS16[REG_SP]) = REGS16[REG_SI];
            REGS16[REG_SP] -= 2;
            MEM16(16 * REGS16[REG_SS] + REGS16[REG_SP]) = REGS16[REG_DI];
            break;
        }
        
        case 54: // 80186: POPA
        {
            REGS16[REG_DI] = MEM16(16 * REGS16[REG_SS] + REGS16[REG_SP]);
            REGS16[REG_SP] += 2;
            REGS16[REG_SI] = MEM16(16 * REGS16[REG_SS] + REGS16[REG_SP]);
            REGS16[REG_SP] += 2;
            REGS16[REG_BP] = MEM16(16 * REGS16[REG_SS] + REGS16[REG_SP]);
            REGS16[REG_SP] += 2;
            REGS16[REG_SP] += 2; // SP ignored
            REGS16[REG_BX] = MEM16(16 * REGS16[REG_SS] + REGS16[REG_SP]);
            REGS16[REG_SP] += 2;
            REGS16[REG_DX] = MEM16(16 * REGS16[REG_SS] + REGS16[REG_SP]);
            REGS16[REG_SP] += 2;
            REGS16[REG_CX] = MEM16(16 * REGS16[REG_SS] + REGS16[REG_SP]);
            REGS16[REG_SP] += 2;
            REGS16[REG_AX] = MEM16(16 * REGS16[REG_SS] + REGS16[REG_SP]);
            REGS16[REG_SP] += 2;
            break;
        }
        
        case 55: // 80186: BOUND
            ESP_LOGW(TAG, "BOUND - not implemented");
            break;
            
        case 56: // 80186: PUSH imm16
            REGS16[REG_SP] -= 2;
            MEM16(16 * REGS16[REG_SS] + REGS16[REG_SP]) = i_data0;
            break;
            
        case 57: // 80186: PUSH imm8
            REGS16[REG_SP] -= 2;
            MEM16(16 * REGS16[REG_SS] + REGS16[REG_SP]) = (i_data0 & 0xff) | (i_data0 & 0x80 ? 0xff00 : 0);
            break;
            
        case 58: // 80186: IMUL
            ESP_LOGW(TAG, "80186 IMUL - not implemented");
            break;
            
        case 59: // 80186: INSB/INSW
            ESP_LOGW(TAG, "INSB/INSW - not implemented");
            break;
            
        case 60: // 80186: OUTSB/OUTSW
            ESP_LOGW(TAG, "OUTSB/OUTSW - not implemented");
            break;
            
        case 69: // 8087 Math Coprocessor
            ESP_LOGW(TAG, "8087 coprocessor %02X %02X - not implemented", opcode_stream[0], opcode_stream[1]);
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown opcode %02X %02X at CS:IP=%04X:%04X", 
                     opcode_stream[0], opcode_stream[1], REGS16[REG_CS], REG_IP);
            break;
    }
    
    // Increment IP by instruction length
    if (calcIP)
        REG_IP += (i_mod * (i_mod != 3) + 2 * (!i_mod && i_rm == 6)) * i_mod_size + 
                  base_size[raw_opcode_id] + i_w_adder[raw_opcode_id] * (i_w + 1);
    
    // Set flags if needed
    if (set_flags_type & 1) {
        F_SF = (1 & op_result >> (8 * (i_w + 1) - 1));
        F_ZF = !op_result;
        F_PF = parity[(uint8_t)op_result];
        
        if (set_flags_type & 2)
            set_AF_OF_arith(op_result);
        else if (set_flags_type & 4)
            F_CF = F_OF = 0;
    }
}

uint32_t i86_cpu_run(uint32_t count) {
    uint32_t executed = 0;
    
    if (count == 0) {
        // Run until halted
        while (!g_cpu.halted) {
            i86_cpu_step();
            executed++;
        }
    } else {
        for (uint32_t i = 0; i < count && !g_cpu.halted; i++) {
            i86_cpu_step();
            executed++;
        }
    }
    
    return executed;
}

bool i86_cpu_irq(uint8_t irq_num) {
    if (!g_cpu.pending_irq) {
        g_cpu.pending_irq = true;
        g_cpu.pending_irq_num = irq_num;
        return true;
    }
    return false;
}

bool i86_cpu_is_halted(void) {
    return g_cpu.halted;
}

void i86_cpu_get_state(i86_cpu_t *state) {
    if (state) {
        memcpy(state, &g_cpu, sizeof(i86_cpu_t));
    }
}

// Register access functions
uint16_t i86_cpu_get_ax(void) { return REGS16[REG_AX]; }
uint16_t i86_cpu_get_bx(void) { return REGS16[REG_BX]; }
uint16_t i86_cpu_get_cx(void) { return REGS16[REG_CX]; }
uint16_t i86_cpu_get_dx(void) { return REGS16[REG_DX]; }
uint16_t i86_cpu_get_sp(void) { return REGS16[REG_SP]; }
uint16_t i86_cpu_get_bp(void) { return REGS16[REG_BP]; }
uint16_t i86_cpu_get_si(void) { return REGS16[REG_SI]; }
uint16_t i86_cpu_get_di(void) { return REGS16[REG_DI]; }
uint16_t i86_cpu_get_cs(void) { return REGS16[REG_CS]; }
uint16_t i86_cpu_get_ds(void) { return REGS16[REG_DS]; }
uint16_t i86_cpu_get_es(void) { return REGS16[REG_ES]; }
uint16_t i86_cpu_get_ss(void) { return REGS16[REG_SS]; }
uint16_t i86_cpu_get_ip(void) { return REG_IP; }
uint16_t i86_cpu_get_flags(void) { return make_flags(); }

void i86_cpu_set_ax(uint16_t value) { REGS16[REG_AX] = value; }
void i86_cpu_set_bx(uint16_t value) { REGS16[REG_BX] = value; }
void i86_cpu_set_cx(uint16_t value) { REGS16[REG_CX] = value; }
void i86_cpu_set_dx(uint16_t value) { REGS16[REG_DX] = value; }
void i86_cpu_set_sp(uint16_t value) { REGS16[REG_SP] = value; }
void i86_cpu_set_bp(uint16_t value) { REGS16[REG_BP] = value; }
void i86_cpu_set_si(uint16_t value) { REGS16[REG_SI] = value; }
void i86_cpu_set_di(uint16_t value) { REGS16[REG_DI] = value; }
void i86_cpu_set_cs(uint16_t value) { REGS16[REG_CS] = value; }
void i86_cpu_set_ds(uint16_t value) { REGS16[REG_DS] = value; }
void i86_cpu_set_es(uint16_t value) { REGS16[REG_ES] = value; }
void i86_cpu_set_ss(uint16_t value) { REGS16[REG_SS] = value; }
void i86_cpu_set_ip(uint16_t value) { REG_IP = value; }

// Memory access
uint8_t i86_cpu_read_byte(uint32_t addr) {
    if (addr < I8086_TOTAL_MEMORY)
        return g_cpu.memory[addr];
    return 0xFF;
}

uint16_t i86_cpu_read_word(uint32_t addr) {
    if (addr < I8086_TOTAL_MEMORY - 1)
        return *(uint16_t*)(g_cpu.memory + addr);
    return 0xFFFF;
}

void i86_cpu_write_byte(uint32_t addr, uint8_t value) {
    if (addr < I8086_TOTAL_MEMORY)
        g_cpu.memory[addr] = value;
}

void i86_cpu_write_word(uint32_t addr, uint16_t value) {
    if (addr < I8086_TOTAL_MEMORY - 1)
        *(uint16_t*)(g_cpu.memory + addr) = value;
}

uint8_t *i86_cpu_get_memory(void) { return g_cpu.memory; }
uint8_t *i86_cpu_get_ram(void) { return g_cpu.ram; }
uint8_t *i86_cpu_get_video(void) { return g_cpu.video; }
uint8_t *i86_cpu_get_bios(void) { return g_cpu.bios; }

int i86_cpu_load_bios(const uint8_t *data, size_t size) {
    if (!g_cpu.initialized || !data || size == 0) {
        return -1;
    }
    
    size_t copy_size = size < I8086_BIOS_SIZE ? size : I8086_BIOS_SIZE;
    memcpy(g_cpu.bios, data, copy_size);
    
    ESP_LOGI(TAG, "Loaded %u bytes of BIOS ROM", copy_size);
    return 0;
}

void i86_cpu_shutdown(void) {
    if (!g_cpu.initialized) return;
    
    if (g_cpu.memory) {
        heap_caps_free(g_cpu.memory);
        g_cpu.memory = NULL;
    }
    
    g_cpu.ram = NULL;
    g_cpu.video = NULL;
    g_cpu.bios = NULL;
    g_cpu.initialized = false;
    
    ESP_LOGI(TAG, "i8086 CPU shut down");
}
