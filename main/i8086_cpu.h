/**
 * @file i8086_cpu.h
 * @brief Intel 8086/8088 CPU Emulator - Complete Implementation
 * 
 * Based on FabGL i8086 (MIT License) - Adrian Cable / Fabrizio Di Vittorio
 * Ported to C for ESP32-P4 by the M68K Emulator project
 * 
 * COMPLETE INSTRUCTION SET:
 * - All 8086/8088 opcodes (256 main + Mod R/M variations)
 * - 80186/NEC V20 extensions: ENTER, LEAVE, PUSHA, POPA, PUSH imm
 * - Full flags handling (CF, PF, AF, ZF, SF, TF, IF, DF, OF)
 * - All addressing modes (register, immediate, memory, indexed)
 * - Segment override prefixes (ES:, CS:, SS:, DS:)
 * - REP/REPE/REPNE prefixes for string operations
 * - All ALU operations, shifts, rotates
 * - MUL, IMUL, DIV, IDIV (8/16-bit)
 * - String operations: MOVS, STOS, LODS, CMPS, SCAS
 * - I/O: IN, OUT (port and DX-addressed)
 * - Interrupts: INT, INTO, IRET, IRQ handling
 * - BCD: AAA, AAS, AAM, AAD, DAA, DAS
 * - HLT, NOP, LOCK prefix
 */

#ifndef I8086_CPU_H
#define I8086_CPU_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Configuration
#define I8086_RAM_SIZE      (640 * 1024)     // 640KB conventional memory
#define I8086_VIDEO_SIZE    (128 * 1024)     // 128KB video memory (A0000-BFFFF)
#define I8086_BIOS_SIZE     (64 * 1024)      // 64KB BIOS ROM (F0000-FFFFF)
#define I8086_TOTAL_MEMORY  (1024 * 1024)    // 1MB address space

#define VIDEOMEM_START      0xA0000
#define VIDEOMEM_END        0xC0000
#define BIOSMEM_START       0xF0000

// 16-bit register indices
#define REG_AX  0
#define REG_CX  1
#define REG_DX  2
#define REG_BX  3
#define REG_SP  4
#define REG_BP  5
#define REG_SI  6
#define REG_DI  7
#define REG_ES  8
#define REG_CS  9
#define REG_SS  10
#define REG_DS  11

// 8-bit register indices (into regs8 array)
#define REG_AL  0
#define REG_AH  1
#define REG_CL  2
#define REG_CH  3
#define REG_DL  4
#define REG_DH  5
#define REG_BL  6
#define REG_BH  7

// Flag array indices (for internal flags array)
#define FLAG_IDX_CF 0
#define FLAG_IDX_PF 1
#define FLAG_IDX_AF 2
#define FLAG_IDX_ZF 3
#define FLAG_IDX_SF 4
#define FLAG_IDX_TF 5
#define FLAG_IDX_IF 6
#define FLAG_IDX_DF 7
#define FLAG_IDX_OF 8

// Callback types
typedef void (*i86_write_port_fn)(void *ctx, uint16_t port, uint8_t value);
typedef uint8_t (*i86_read_port_fn)(void *ctx, uint16_t port);
typedef void (*i86_write_video8_fn)(void *ctx, uint32_t addr, uint8_t value);
typedef void (*i86_write_video16_fn)(void *ctx, uint32_t addr, uint16_t value);
typedef uint8_t (*i86_read_video8_fn)(void *ctx, uint32_t addr);
typedef uint16_t (*i86_read_video16_fn)(void *ctx, uint32_t addr);
typedef bool (*i86_interrupt_fn)(void *ctx, uint8_t num);

// CPU state
typedef struct {
    // General purpose registers (union for 8/16-bit access)
    union {
        uint16_t regs16[16];
        uint8_t  regs8[32];
    };
    
    // Instruction pointer
    uint16_t ip;
    
    // Flags (individual bytes for speed)
    uint8_t flags[10];
    
    // Memory pointers
    uint8_t *memory;      // Full 1MB address space
    uint8_t *ram;         // 640KB RAM (0x00000-0x9FFFF)
    uint8_t *video;       // 128KB video (0xA0000-0xBFFFF)
    uint8_t *bios;        // 64KB BIOS (0xF0000-0xFFFFF)
    
    // Callbacks
    void *callback_ctx;
    i86_read_port_fn read_port;
    i86_write_port_fn write_port;
    i86_read_video8_fn read_video8;
    i86_write_video8_fn write_video8;
    i86_read_video16_fn read_video16;
    i86_write_video16_fn write_video16;
    i86_interrupt_fn interrupt;
    
    // State
    bool halted;
    bool pending_irq;
    uint8_t pending_irq_num;
    
    // Statistics
    uint64_t cycles;
    uint64_t instructions;
    
    // Prefix state
    bool seg_override;
    uint8_t seg_override_reg;
    bool rep_override;
    uint8_t rep_mode;  // 0=REPNZ, 1=REPZ
    
    // Internal use
    int32_t regs_offset;
    
    bool initialized;
} i86_cpu_t;

/**
 * Initialize the i8086 CPU emulator
 * @return ESP_OK on success, error code on failure
 */
int i86_cpu_init(void);

/**
 * Set callback functions
 */
void i86_cpu_set_callbacks(
    void *ctx,
    i86_read_port_fn read_port,
    i86_write_port_fn write_port,
    i86_read_video8_fn read_video8,
    i86_write_video8_fn write_video8,
    i86_read_video16_fn read_video16,
    i86_write_video16_fn write_video16,
    i86_interrupt_fn interrupt
);

/**
 * Reset the CPU to power-on state
 */
void i86_cpu_reset(void);

/**
 * Execute a single instruction
 */
void i86_cpu_step(void);

/**
 * Execute multiple instructions
 * @param count Number of instructions to execute (0 = run until halted)
 * @return Number of instructions executed
 */
uint32_t i86_cpu_run(uint32_t count);

/**
 * Signal an interrupt request
 * @param irq_num Interrupt number (0-255)
 * @return true if IRQ was queued, false if already pending
 */
bool i86_cpu_irq(uint8_t irq_num);

/**
 * Check if CPU is halted
 */
bool i86_cpu_is_halted(void);

/**
 * Get CPU state (for debugging)
 */
void i86_cpu_get_state(i86_cpu_t *state);

/**
 * Get register values
 */
uint16_t i86_cpu_get_ax(void);
uint16_t i86_cpu_get_bx(void);
uint16_t i86_cpu_get_cx(void);
uint16_t i86_cpu_get_dx(void);
uint16_t i86_cpu_get_sp(void);
uint16_t i86_cpu_get_bp(void);
uint16_t i86_cpu_get_si(void);
uint16_t i86_cpu_get_di(void);
uint16_t i86_cpu_get_cs(void);
uint16_t i86_cpu_get_ds(void);
uint16_t i86_cpu_get_es(void);
uint16_t i86_cpu_get_ss(void);
uint16_t i86_cpu_get_ip(void);
uint16_t i86_cpu_get_flags(void);

/**
 * Set register values
 */
void i86_cpu_set_ax(uint16_t value);
void i86_cpu_set_bx(uint16_t value);
void i86_cpu_set_cx(uint16_t value);
void i86_cpu_set_dx(uint16_t value);
void i86_cpu_set_sp(uint16_t value);
void i86_cpu_set_bp(uint16_t value);
void i86_cpu_set_si(uint16_t value);
void i86_cpu_set_di(uint16_t value);
void i86_cpu_set_cs(uint16_t value);
void i86_cpu_set_ds(uint16_t value);
void i86_cpu_set_es(uint16_t value);
void i86_cpu_set_ss(uint16_t value);
void i86_cpu_set_ip(uint16_t value);

/**
 * Direct memory access
 */
uint8_t i86_cpu_read_byte(uint32_t addr);
uint16_t i86_cpu_read_word(uint32_t addr);
void i86_cpu_write_byte(uint32_t addr, uint8_t value);
void i86_cpu_write_word(uint32_t addr, uint16_t value);

/**
 * Set/get individual CPU flags
 * @param flag_idx Flag index (FLAG_IDX_CF, FLAG_IDX_ZF, etc.)
 * @param value Flag value (0 or 1)
 */
void i86_cpu_set_flag(int flag_idx, uint8_t value);
uint8_t i86_cpu_get_flag(int flag_idx);

/**
 * Get memory pointer for direct access
 */
uint8_t *i86_cpu_get_memory(void);
uint8_t *i86_cpu_get_ram(void);
uint8_t *i86_cpu_get_video(void);
uint8_t *i86_cpu_get_bios(void);

/**
 * Load BIOS ROM image
 */
int i86_cpu_load_bios(const uint8_t *data, size_t size);

/**
 * Shutdown and free resources
 */
void i86_cpu_shutdown(void);

#ifdef __cplusplus
}
#endif

#endif // I8086_CPU_H
