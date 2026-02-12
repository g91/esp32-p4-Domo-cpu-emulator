/*
 * MOS 6502 CPU Core
 * Pure CPU emulation - no platform dependencies
 */

#ifndef MOS6502_CPU_H
#define MOS6502_CPU_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Status register flags
#define MOS6502_FLAG_C  0x01    // Carry
#define MOS6502_FLAG_Z  0x02    // Zero
#define MOS6502_FLAG_I  0x04    // Interrupt disable
#define MOS6502_FLAG_D  0x08    // Decimal (BCD) mode
#define MOS6502_FLAG_B  0x10    // Break command
#define MOS6502_FLAG_U  0x20    // Unused (always 1)
#define MOS6502_FLAG_V  0x40    // Overflow
#define MOS6502_FLAG_N  0x80    // Negative

// Interrupt vectors
#define MOS6502_VEC_NMI    0xFFFA
#define MOS6502_VEC_RESET  0xFFFC
#define MOS6502_VEC_IRQ    0xFFFE

// Stack page
#define MOS6502_STACK_BASE 0x0100

// Memory access callbacks
typedef uint8_t (*mos6502_read_fn)(void *ctx, uint16_t addr);
typedef void    (*mos6502_write_fn)(void *ctx, uint16_t addr, uint8_t val);

// CPU state
typedef struct {
    uint8_t  a;             // Accumulator
    uint8_t  x;             // X index register
    uint8_t  y;             // Y index register
    uint8_t  sp;            // Stack pointer (offset into $0100-$01FF)
    uint16_t pc;            // Program counter
    uint8_t  status;        // Processor status (NV-BDIZC)

    bool     nmi_pending;   // NMI edge-triggered
    bool     irq_line;      // IRQ level (active low, true = asserted)
    bool     halted;        // CPU jammed/stopped

    uint64_t cycles;        // Total cycle count
    uint64_t instructions;  // Total instructions executed

    // Memory callbacks (set by integration layer)
    mos6502_read_fn  read;
    mos6502_write_fn write;
    void            *mem_ctx;

    bool initialized;
} mos6502_t;

/**
 * Initialize CPU state to power-on defaults
 */
void mos6502_init(mos6502_t *cpu);

/**
 * Reset CPU - reads reset vector from $FFFC/$FFFD
 */
void mos6502_reset(mos6502_t *cpu);

/**
 * Execute one instruction, returns cycle count for that instruction
 */
int mos6502_step(mos6502_t *cpu);

/**
 * Execute up to 'count' instructions, returns actual instructions executed
 */
uint32_t mos6502_run(mos6502_t *cpu, uint32_t count);

/**
 * Trigger a non-maskable interrupt (edge-triggered)
 */
void mos6502_nmi(mos6502_t *cpu);

/**
 * Set IRQ line state (level-sensitive, active when true)
 */
void mos6502_irq(mos6502_t *cpu, bool active);

/**
 * Set memory access callbacks
 */
void mos6502_set_callbacks(mos6502_t *cpu,
                           mos6502_read_fn read,
                           mos6502_write_fn write,
                           void *ctx);

#ifdef __cplusplus
}
#endif

#endif // MOS6502_CPU_H
