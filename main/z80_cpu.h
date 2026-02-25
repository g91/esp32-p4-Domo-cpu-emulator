/*
 * z80_cpu.h - Zilog Z80 CPU emulator interface
 *
 * Complete Z80 instruction set emulation including:
 *   - All base opcodes (0x00-0xFF)
 *   - CB prefix: bit/rotate/shift operations
 *   - DD prefix: IX-indexed operations
 *   - FD prefix: IY-indexed operations
 *   - ED prefix: extended instructions (LDIR, ADC HL, IN/OUT, etc.)
 *   - DD CB / FD CB: indexed bit operations
 *   - Interrupt modes IM0, IM1, IM2
 *   - NMI (non-maskable interrupt)
 *   - Undocumented instructions (IXH/IXL/IYH/IYL, SLL, etc.)
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =====================================================================
 * Z80 Flag Register Bits
 * ===================================================================== */
#define Z80_FLAG_C   0x01   /* Carry */
#define Z80_FLAG_N   0x02   /* Add/Subtract */
#define Z80_FLAG_PV  0x04   /* Parity/Overflow */
#define Z80_FLAG_X   0x08   /* Undocumented (bit 3) */
#define Z80_FLAG_H   0x10   /* Half Carry */
#define Z80_FLAG_Y   0x20   /* Undocumented (bit 5) */
#define Z80_FLAG_Z   0x40   /* Zero */
#define Z80_FLAG_S   0x80   /* Sign */

/* =====================================================================
 * Z80 CPU State
 * ===================================================================== */
typedef struct z80_s {
    /* Main register set */
    uint8_t  A, F;          /* Accumulator and Flags */
    uint8_t  B, C;          /* BC pair */
    uint8_t  D, E;          /* DE pair */
    uint8_t  H, L;          /* HL pair */

    /* Alternate (shadow) register set */
    uint8_t  A2, F2;        /* Shadow AF */
    uint8_t  B2, C2;        /* Shadow BC */
    uint8_t  D2, E2;        /* Shadow DE */
    uint8_t  H2, L2;        /* Shadow HL */

    /* Index registers */
    uint16_t IX, IY;

    /* Control registers */
    uint16_t SP;            /* Stack Pointer */
    uint16_t PC;            /* Program Counter */
    uint8_t  I;             /* Interrupt Vector */
    uint8_t  R;             /* Memory Refresh */

    /* Interrupt state */
    bool     IFF1;          /* Interrupt flip-flop 1 */
    bool     IFF2;          /* Interrupt flip-flop 2 */
    uint8_t  IM;            /* Interrupt mode: 0, 1, or 2 */
    bool     halted;        /* CPU is HALTed */
    bool     irq_pending;   /* IRQ line asserted */
    uint8_t  irq_vector;    /* Data bus value for IM0 */
    bool     nmi_pending;   /* NMI line asserted */
    bool     ei_pending;    /* EI executed - delay interrupt by one instruction */

    /* Timing */
    uint64_t total_cycles;  /* Total T-states executed */

    /* Memory access callbacks */
    uint8_t (*mem_read) (void *user, uint16_t addr);
    void    (*mem_write)(void *user, uint16_t addr, uint8_t val);

    /* I/O port callbacks (may be NULL) */
    uint8_t (*io_read) (void *user, uint16_t port);
    void    (*io_write)(void *user, uint16_t port, uint8_t val);

    /* Optional BDOS/BIOS trap callback (called when PC == trap_addr, before executing) */
    bool    (*trap_cb)(void *user, struct z80_s *cpu);
    uint16_t trap_addr;     /* Address to trigger trap (default 0x0005) */

    /* User data pointer */
    void *user_data;
} z80_t;

/* =====================================================================
 * API
 * ===================================================================== */

/** Initialise CPU state (call once) */
void z80_init(z80_t *cpu);

/** Reset CPU (registers set to power-on values, PC = 0) */
void z80_reset(z80_t *cpu);

/** Execute one instruction. Returns number of T-states (cycles) consumed. */
int z80_step(z80_t *cpu);

/** Assert or deassert the maskable interrupt (IRQ) line. */
void z80_set_irq(z80_t *cpu, bool level, uint8_t vector);

/** Trigger a Non-Maskable Interrupt. */
void z80_set_nmi(z80_t *cpu);

/* ---- Register pair accessors ---- */
static inline uint16_t z80_get_af(z80_t *c) { return (uint16_t)(c->A << 8) | c->F; }
static inline uint16_t z80_get_bc(z80_t *c) { return (uint16_t)(c->B << 8) | c->C; }
static inline uint16_t z80_get_de(z80_t *c) { return (uint16_t)(c->D << 8) | c->E; }
static inline uint16_t z80_get_hl(z80_t *c) { return (uint16_t)(c->H << 8) | c->L; }

static inline void z80_set_af(z80_t *c, uint16_t v) { c->A = v >> 8; c->F = v & 0xFF; }
static inline void z80_set_bc(z80_t *c, uint16_t v) { c->B = v >> 8; c->C = v & 0xFF; }
static inline void z80_set_de(z80_t *c, uint16_t v) { c->D = v >> 8; c->E = v & 0xFF; }
static inline void z80_set_hl(z80_t *c, uint16_t v) { c->H = v >> 8; c->L = v & 0xFF; }

/** Print register state to console (for debugging) */
void z80_dump_state(z80_t *cpu);

#ifdef __cplusplus
}
#endif
