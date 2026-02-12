/*
 * MOS 6502 CPU Core Implementation
 * Complete instruction set with all official opcodes and addressing modes
 * BCD arithmetic support, IRQ/NMI interrupt handling
 */

#include "mos6502_cpu.h"
#include <string.h>

// ============================================================================
// Internal helpers
// ============================================================================

static inline uint8_t cpu_read(mos6502_t *cpu, uint16_t addr) {
    return cpu->read(cpu->mem_ctx, addr);
}

static inline void cpu_write(mos6502_t *cpu, uint16_t addr, uint8_t val) {
    cpu->write(cpu->mem_ctx, addr, val);
}

static inline uint16_t cpu_read16(mos6502_t *cpu, uint16_t addr) {
    uint8_t lo = cpu_read(cpu, addr);
    uint8_t hi = cpu_read(cpu, addr + 1);
    return (uint16_t)(lo | (hi << 8));
}

// Read 16-bit with page-wrapping bug (for JMP indirect)
static inline uint16_t cpu_read16_wrap(mos6502_t *cpu, uint16_t addr) {
    uint8_t lo = cpu_read(cpu, addr);
    // High byte wraps within the same page
    uint16_t hi_addr = (addr & 0xFF00) | ((addr + 1) & 0x00FF);
    uint8_t hi = cpu_read(cpu, hi_addr);
    return (uint16_t)(lo | (hi << 8));
}

static inline void push8(mos6502_t *cpu, uint8_t val) {
    cpu_write(cpu, MOS6502_STACK_BASE + cpu->sp, val);
    cpu->sp--;
}

static inline uint8_t pull8(mos6502_t *cpu) {
    cpu->sp++;
    return cpu_read(cpu, MOS6502_STACK_BASE + cpu->sp);
}

static inline void push16(mos6502_t *cpu, uint16_t val) {
    push8(cpu, (uint8_t)(val >> 8));
    push8(cpu, (uint8_t)(val & 0xFF));
}

static inline uint16_t pull16(mos6502_t *cpu) {
    uint8_t lo = pull8(cpu);
    uint8_t hi = pull8(cpu);
    return (uint16_t)(lo | (hi << 8));
}

// Flag helpers
static inline void set_flag(mos6502_t *cpu, uint8_t flag, bool val) {
    if (val) cpu->status |= flag;
    else     cpu->status &= ~flag;
}

static inline bool get_flag(mos6502_t *cpu, uint8_t flag) {
    return (cpu->status & flag) != 0;
}

static inline void update_nz(mos6502_t *cpu, uint8_t val) {
    set_flag(cpu, MOS6502_FLAG_Z, val == 0);
    set_flag(cpu, MOS6502_FLAG_N, (val & 0x80) != 0);
}

// ============================================================================
// Addressing modes - return effective address
// ============================================================================

// Immediate: operand is next byte
static inline uint16_t addr_imm(mos6502_t *cpu) {
    return cpu->pc++;
}

// Zero Page: operand at zero page address
static inline uint16_t addr_zp(mos6502_t *cpu) {
    return cpu_read(cpu, cpu->pc++);
}

// Zero Page,X: zero page + X with wrapping
static inline uint16_t addr_zpx(mos6502_t *cpu) {
    return (cpu_read(cpu, cpu->pc++) + cpu->x) & 0xFF;
}

// Zero Page,Y: zero page + Y with wrapping
static inline uint16_t addr_zpy(mos6502_t *cpu) {
    return (cpu_read(cpu, cpu->pc++) + cpu->y) & 0xFF;
}

// Absolute: 16-bit address
static inline uint16_t addr_abs(mos6502_t *cpu) {
    uint16_t addr = cpu_read16(cpu, cpu->pc);
    cpu->pc += 2;
    return addr;
}

// Absolute,X: 16-bit + X, returns page_crossed via pointer
static inline uint16_t addr_abx(mos6502_t *cpu, bool *page_crossed) {
    uint16_t base = cpu_read16(cpu, cpu->pc);
    cpu->pc += 2;
    uint16_t addr = base + cpu->x;
    if (page_crossed) *page_crossed = ((base ^ addr) & 0xFF00) != 0;
    return addr;
}

// Absolute,Y: 16-bit + Y
static inline uint16_t addr_aby(mos6502_t *cpu, bool *page_crossed) {
    uint16_t base = cpu_read16(cpu, cpu->pc);
    cpu->pc += 2;
    uint16_t addr = base + cpu->y;
    if (page_crossed) *page_crossed = ((base ^ addr) & 0xFF00) != 0;
    return addr;
}

// Indexed Indirect (X): (zp + X) -> read 16-bit from zero page
static inline uint16_t addr_izx(mos6502_t *cpu) {
    uint8_t zp = (cpu_read(cpu, cpu->pc++) + cpu->x) & 0xFF;
    uint8_t lo = cpu_read(cpu, zp);
    uint8_t hi = cpu_read(cpu, (zp + 1) & 0xFF);
    return (uint16_t)(lo | (hi << 8));
}

// Indirect Indexed (Y): (zp) + Y -> read 16-bit from zero page, add Y
static inline uint16_t addr_izy(mos6502_t *cpu, bool *page_crossed) {
    uint8_t zp = cpu_read(cpu, cpu->pc++);
    uint8_t lo = cpu_read(cpu, zp);
    uint8_t hi = cpu_read(cpu, (zp + 1) & 0xFF);
    uint16_t base = (uint16_t)(lo | (hi << 8));
    uint16_t addr = base + cpu->y;
    if (page_crossed) *page_crossed = ((base ^ addr) & 0xFF00) != 0;
    return addr;
}

// Relative: signed 8-bit offset from PC (for branches)
static inline int8_t addr_rel(mos6502_t *cpu) {
    return (int8_t)cpu_read(cpu, cpu->pc++);
}

// ============================================================================
// ALU operations
// ============================================================================

static void op_adc(mos6502_t *cpu, uint8_t val) {
    if (get_flag(cpu, MOS6502_FLAG_D)) {
        // BCD mode
        uint16_t lo = (cpu->a & 0x0F) + (val & 0x0F) + (get_flag(cpu, MOS6502_FLAG_C) ? 1 : 0);
        uint16_t hi = (cpu->a & 0xF0) + (val & 0xF0);
        if (lo > 0x09) { lo += 0x06; hi += 0x10; }
        // Overflow is computed on binary result
        set_flag(cpu, MOS6502_FLAG_V,
            (~(cpu->a ^ val) & (cpu->a ^ (uint8_t)(hi + (lo & 0x0F))) & 0x80) != 0);
        if (hi > 0x90) hi += 0x60;
        set_flag(cpu, MOS6502_FLAG_C, hi > 0xFF);
        cpu->a = (uint8_t)((hi & 0xF0) | (lo & 0x0F));
        update_nz(cpu, cpu->a);
    } else {
        uint16_t sum = (uint16_t)cpu->a + val + (get_flag(cpu, MOS6502_FLAG_C) ? 1 : 0);
        set_flag(cpu, MOS6502_FLAG_C, sum > 0xFF);
        set_flag(cpu, MOS6502_FLAG_V,
            (~(cpu->a ^ val) & (cpu->a ^ (uint8_t)sum) & 0x80) != 0);
        cpu->a = (uint8_t)sum;
        update_nz(cpu, cpu->a);
    }
}

static void op_sbc(mos6502_t *cpu, uint8_t val) {
    if (get_flag(cpu, MOS6502_FLAG_D)) {
        // BCD mode
        int16_t lo = (cpu->a & 0x0F) - (val & 0x0F) - (get_flag(cpu, MOS6502_FLAG_C) ? 0 : 1);
        int16_t hi = (cpu->a & 0xF0) - (val & 0xF0);
        if (lo < 0) { lo -= 0x06; hi -= 0x10; }
        if (hi < 0) hi -= 0x60;
        // Carry/overflow computed on binary result
        uint16_t bin = (uint16_t)cpu->a - val - (get_flag(cpu, MOS6502_FLAG_C) ? 0 : 1);
        set_flag(cpu, MOS6502_FLAG_C, bin < 0x100);
        set_flag(cpu, MOS6502_FLAG_V,
            ((cpu->a ^ val) & (cpu->a ^ (uint8_t)bin) & 0x80) != 0);
        cpu->a = (uint8_t)((hi & 0xF0) | (lo & 0x0F));
        update_nz(cpu, cpu->a);
    } else {
        uint16_t diff = (uint16_t)cpu->a - val - (get_flag(cpu, MOS6502_FLAG_C) ? 0 : 1);
        set_flag(cpu, MOS6502_FLAG_C, diff < 0x100);
        set_flag(cpu, MOS6502_FLAG_V,
            ((cpu->a ^ val) & (cpu->a ^ (uint8_t)diff) & 0x80) != 0);
        cpu->a = (uint8_t)diff;
        update_nz(cpu, cpu->a);
    }
}

static inline void op_cmp(mos6502_t *cpu, uint8_t reg, uint8_t val) {
    uint16_t result = (uint16_t)reg - val;
    set_flag(cpu, MOS6502_FLAG_C, reg >= val);
    update_nz(cpu, (uint8_t)result);
}

static inline uint8_t op_asl(mos6502_t *cpu, uint8_t val) {
    set_flag(cpu, MOS6502_FLAG_C, (val & 0x80) != 0);
    val <<= 1;
    update_nz(cpu, val);
    return val;
}

static inline uint8_t op_lsr(mos6502_t *cpu, uint8_t val) {
    set_flag(cpu, MOS6502_FLAG_C, (val & 0x01) != 0);
    val >>= 1;
    update_nz(cpu, val);
    return val;
}

static inline uint8_t op_rol(mos6502_t *cpu, uint8_t val) {
    bool old_carry = get_flag(cpu, MOS6502_FLAG_C);
    set_flag(cpu, MOS6502_FLAG_C, (val & 0x80) != 0);
    val = (val << 1) | (old_carry ? 1 : 0);
    update_nz(cpu, val);
    return val;
}

static inline uint8_t op_ror(mos6502_t *cpu, uint8_t val) {
    bool old_carry = get_flag(cpu, MOS6502_FLAG_C);
    set_flag(cpu, MOS6502_FLAG_C, (val & 0x01) != 0);
    val = (val >> 1) | (old_carry ? 0x80 : 0);
    update_nz(cpu, val);
    return val;
}

// Branch helper: returns extra cycles (1 for taken, +1 for page cross)
static inline int do_branch(mos6502_t *cpu, bool condition) {
    int8_t offset = addr_rel(cpu);
    if (condition) {
        uint16_t old_pc = cpu->pc;
        cpu->pc = (uint16_t)(cpu->pc + offset);
        return ((old_pc ^ cpu->pc) & 0xFF00) ? 2 : 1;
    }
    return 0;
}

// ============================================================================
// Interrupt handling
// ============================================================================

static void do_irq(mos6502_t *cpu) {
    push16(cpu, cpu->pc);
    push8(cpu, (cpu->status | MOS6502_FLAG_U) & ~MOS6502_FLAG_B);
    set_flag(cpu, MOS6502_FLAG_I, true);
    cpu->pc = cpu_read16(cpu, MOS6502_VEC_IRQ);
    cpu->cycles += 7;
}

static void do_nmi(mos6502_t *cpu) {
    push16(cpu, cpu->pc);
    push8(cpu, (cpu->status | MOS6502_FLAG_U) & ~MOS6502_FLAG_B);
    set_flag(cpu, MOS6502_FLAG_I, true);
    cpu->pc = cpu_read16(cpu, MOS6502_VEC_NMI);
    cpu->nmi_pending = false;
    cpu->cycles += 7;
}

// ============================================================================
// Public API
// ============================================================================

void mos6502_init(mos6502_t *cpu) {
    memset(cpu, 0, sizeof(*cpu));
    cpu->status = MOS6502_FLAG_U | MOS6502_FLAG_I;
    cpu->sp = 0xFD;
    cpu->initialized = true;
}

void mos6502_reset(mos6502_t *cpu) {
    cpu->a = 0;
    cpu->x = 0;
    cpu->y = 0;
    cpu->sp = 0xFD;
    cpu->status = MOS6502_FLAG_U | MOS6502_FLAG_I;
    cpu->halted = false;
    cpu->nmi_pending = false;
    cpu->irq_line = false;
    cpu->pc = cpu_read16(cpu, MOS6502_VEC_RESET);
    cpu->cycles += 7;
}

void mos6502_nmi(mos6502_t *cpu) {
    cpu->nmi_pending = true;
}

void mos6502_irq(mos6502_t *cpu, bool active) {
    cpu->irq_line = active;
}

void mos6502_set_callbacks(mos6502_t *cpu,
                           mos6502_read_fn read,
                           mos6502_write_fn write,
                           void *ctx) {
    cpu->read = read;
    cpu->write = write;
    cpu->mem_ctx = ctx;
}

// ============================================================================
// Main instruction decoder & executor
// ============================================================================

int mos6502_step(mos6502_t *cpu) {
    if (cpu->halted) return 0;

    // Check interrupts
    if (cpu->nmi_pending) {
        do_nmi(cpu);
        return 7;
    }
    if (cpu->irq_line && !get_flag(cpu, MOS6502_FLAG_I)) {
        do_irq(cpu);
        return 7;
    }

    uint8_t opcode = cpu_read(cpu, cpu->pc++);
    int cycles = 0;
    bool page_crossed = false;
    uint16_t addr;
    uint8_t val;

    switch (opcode) {

    // ================================================================
    // LDA - Load Accumulator
    // ================================================================
    case 0xA9: // LDA #imm
        cpu->a = cpu_read(cpu, addr_imm(cpu));
        update_nz(cpu, cpu->a);
        cycles = 2;
        break;
    case 0xA5: // LDA zp
        cpu->a = cpu_read(cpu, addr_zp(cpu));
        update_nz(cpu, cpu->a);
        cycles = 3;
        break;
    case 0xB5: // LDA zp,X
        cpu->a = cpu_read(cpu, addr_zpx(cpu));
        update_nz(cpu, cpu->a);
        cycles = 4;
        break;
    case 0xAD: // LDA abs
        cpu->a = cpu_read(cpu, addr_abs(cpu));
        update_nz(cpu, cpu->a);
        cycles = 4;
        break;
    case 0xBD: // LDA abs,X
        addr = addr_abx(cpu, &page_crossed);
        cpu->a = cpu_read(cpu, addr);
        update_nz(cpu, cpu->a);
        cycles = 4 + page_crossed;
        break;
    case 0xB9: // LDA abs,Y
        addr = addr_aby(cpu, &page_crossed);
        cpu->a = cpu_read(cpu, addr);
        update_nz(cpu, cpu->a);
        cycles = 4 + page_crossed;
        break;
    case 0xA1: // LDA (zp,X)
        cpu->a = cpu_read(cpu, addr_izx(cpu));
        update_nz(cpu, cpu->a);
        cycles = 6;
        break;
    case 0xB1: // LDA (zp),Y
        addr = addr_izy(cpu, &page_crossed);
        cpu->a = cpu_read(cpu, addr);
        update_nz(cpu, cpu->a);
        cycles = 5 + page_crossed;
        break;

    // ================================================================
    // LDX - Load X Register
    // ================================================================
    case 0xA2: // LDX #imm
        cpu->x = cpu_read(cpu, addr_imm(cpu));
        update_nz(cpu, cpu->x);
        cycles = 2;
        break;
    case 0xA6: // LDX zp
        cpu->x = cpu_read(cpu, addr_zp(cpu));
        update_nz(cpu, cpu->x);
        cycles = 3;
        break;
    case 0xB6: // LDX zp,Y
        cpu->x = cpu_read(cpu, addr_zpy(cpu));
        update_nz(cpu, cpu->x);
        cycles = 4;
        break;
    case 0xAE: // LDX abs
        cpu->x = cpu_read(cpu, addr_abs(cpu));
        update_nz(cpu, cpu->x);
        cycles = 4;
        break;
    case 0xBE: // LDX abs,Y
        addr = addr_aby(cpu, &page_crossed);
        cpu->x = cpu_read(cpu, addr);
        update_nz(cpu, cpu->x);
        cycles = 4 + page_crossed;
        break;

    // ================================================================
    // LDY - Load Y Register
    // ================================================================
    case 0xA0: // LDY #imm
        cpu->y = cpu_read(cpu, addr_imm(cpu));
        update_nz(cpu, cpu->y);
        cycles = 2;
        break;
    case 0xA4: // LDY zp
        cpu->y = cpu_read(cpu, addr_zp(cpu));
        update_nz(cpu, cpu->y);
        cycles = 3;
        break;
    case 0xB4: // LDY zp,X
        cpu->y = cpu_read(cpu, addr_zpx(cpu));
        update_nz(cpu, cpu->y);
        cycles = 4;
        break;
    case 0xAC: // LDY abs
        cpu->y = cpu_read(cpu, addr_abs(cpu));
        update_nz(cpu, cpu->y);
        cycles = 4;
        break;
    case 0xBC: // LDY abs,X
        addr = addr_abx(cpu, &page_crossed);
        cpu->y = cpu_read(cpu, addr);
        update_nz(cpu, cpu->y);
        cycles = 4 + page_crossed;
        break;

    // ================================================================
    // STA - Store Accumulator
    // ================================================================
    case 0x85: // STA zp
        cpu_write(cpu, addr_zp(cpu), cpu->a);
        cycles = 3;
        break;
    case 0x95: // STA zp,X
        cpu_write(cpu, addr_zpx(cpu), cpu->a);
        cycles = 4;
        break;
    case 0x8D: // STA abs
        cpu_write(cpu, addr_abs(cpu), cpu->a);
        cycles = 4;
        break;
    case 0x9D: // STA abs,X
        cpu_write(cpu, addr_abx(cpu, NULL), cpu->a);
        cycles = 5;
        break;
    case 0x99: // STA abs,Y
        cpu_write(cpu, addr_aby(cpu, NULL), cpu->a);
        cycles = 5;
        break;
    case 0x81: // STA (zp,X)
        cpu_write(cpu, addr_izx(cpu), cpu->a);
        cycles = 6;
        break;
    case 0x91: // STA (zp),Y
        cpu_write(cpu, addr_izy(cpu, NULL), cpu->a);
        cycles = 6;
        break;

    // ================================================================
    // STX - Store X Register
    // ================================================================
    case 0x86: // STX zp
        cpu_write(cpu, addr_zp(cpu), cpu->x);
        cycles = 3;
        break;
    case 0x96: // STX zp,Y
        cpu_write(cpu, addr_zpy(cpu), cpu->x);
        cycles = 4;
        break;
    case 0x8E: // STX abs
        cpu_write(cpu, addr_abs(cpu), cpu->x);
        cycles = 4;
        break;

    // ================================================================
    // STY - Store Y Register
    // ================================================================
    case 0x84: // STY zp
        cpu_write(cpu, addr_zp(cpu), cpu->y);
        cycles = 3;
        break;
    case 0x94: // STY zp,X
        cpu_write(cpu, addr_zpx(cpu), cpu->y);
        cycles = 4;
        break;
    case 0x8C: // STY abs
        cpu_write(cpu, addr_abs(cpu), cpu->y);
        cycles = 4;
        break;

    // ================================================================
    // ADC - Add with Carry
    // ================================================================
    case 0x69: // ADC #imm
        op_adc(cpu, cpu_read(cpu, addr_imm(cpu)));
        cycles = 2;
        break;
    case 0x65: // ADC zp
        op_adc(cpu, cpu_read(cpu, addr_zp(cpu)));
        cycles = 3;
        break;
    case 0x75: // ADC zp,X
        op_adc(cpu, cpu_read(cpu, addr_zpx(cpu)));
        cycles = 4;
        break;
    case 0x6D: // ADC abs
        op_adc(cpu, cpu_read(cpu, addr_abs(cpu)));
        cycles = 4;
        break;
    case 0x7D: // ADC abs,X
        addr = addr_abx(cpu, &page_crossed);
        op_adc(cpu, cpu_read(cpu, addr));
        cycles = 4 + page_crossed;
        break;
    case 0x79: // ADC abs,Y
        addr = addr_aby(cpu, &page_crossed);
        op_adc(cpu, cpu_read(cpu, addr));
        cycles = 4 + page_crossed;
        break;
    case 0x61: // ADC (zp,X)
        op_adc(cpu, cpu_read(cpu, addr_izx(cpu)));
        cycles = 6;
        break;
    case 0x71: // ADC (zp),Y
        addr = addr_izy(cpu, &page_crossed);
        op_adc(cpu, cpu_read(cpu, addr));
        cycles = 5 + page_crossed;
        break;

    // ================================================================
    // SBC - Subtract with Carry
    // ================================================================
    case 0xE9: // SBC #imm
        op_sbc(cpu, cpu_read(cpu, addr_imm(cpu)));
        cycles = 2;
        break;
    case 0xE5: // SBC zp
        op_sbc(cpu, cpu_read(cpu, addr_zp(cpu)));
        cycles = 3;
        break;
    case 0xF5: // SBC zp,X
        op_sbc(cpu, cpu_read(cpu, addr_zpx(cpu)));
        cycles = 4;
        break;
    case 0xED: // SBC abs
        op_sbc(cpu, cpu_read(cpu, addr_abs(cpu)));
        cycles = 4;
        break;
    case 0xFD: // SBC abs,X
        addr = addr_abx(cpu, &page_crossed);
        op_sbc(cpu, cpu_read(cpu, addr));
        cycles = 4 + page_crossed;
        break;
    case 0xF9: // SBC abs,Y
        addr = addr_aby(cpu, &page_crossed);
        op_sbc(cpu, cpu_read(cpu, addr));
        cycles = 4 + page_crossed;
        break;
    case 0xE1: // SBC (zp,X)
        op_sbc(cpu, cpu_read(cpu, addr_izx(cpu)));
        cycles = 6;
        break;
    case 0xF1: // SBC (zp),Y
        addr = addr_izy(cpu, &page_crossed);
        op_sbc(cpu, cpu_read(cpu, addr));
        cycles = 5 + page_crossed;
        break;

    // ================================================================
    // AND - Logical AND
    // ================================================================
    case 0x29: // AND #imm
        cpu->a &= cpu_read(cpu, addr_imm(cpu));
        update_nz(cpu, cpu->a);
        cycles = 2;
        break;
    case 0x25: // AND zp
        cpu->a &= cpu_read(cpu, addr_zp(cpu));
        update_nz(cpu, cpu->a);
        cycles = 3;
        break;
    case 0x35: // AND zp,X
        cpu->a &= cpu_read(cpu, addr_zpx(cpu));
        update_nz(cpu, cpu->a);
        cycles = 4;
        break;
    case 0x2D: // AND abs
        cpu->a &= cpu_read(cpu, addr_abs(cpu));
        update_nz(cpu, cpu->a);
        cycles = 4;
        break;
    case 0x3D: // AND abs,X
        addr = addr_abx(cpu, &page_crossed);
        cpu->a &= cpu_read(cpu, addr);
        update_nz(cpu, cpu->a);
        cycles = 4 + page_crossed;
        break;
    case 0x39: // AND abs,Y
        addr = addr_aby(cpu, &page_crossed);
        cpu->a &= cpu_read(cpu, addr);
        update_nz(cpu, cpu->a);
        cycles = 4 + page_crossed;
        break;
    case 0x21: // AND (zp,X)
        cpu->a &= cpu_read(cpu, addr_izx(cpu));
        update_nz(cpu, cpu->a);
        cycles = 6;
        break;
    case 0x31: // AND (zp),Y
        addr = addr_izy(cpu, &page_crossed);
        cpu->a &= cpu_read(cpu, addr);
        update_nz(cpu, cpu->a);
        cycles = 5 + page_crossed;
        break;

    // ================================================================
    // ORA - Logical OR
    // ================================================================
    case 0x09: // ORA #imm
        cpu->a |= cpu_read(cpu, addr_imm(cpu));
        update_nz(cpu, cpu->a);
        cycles = 2;
        break;
    case 0x05: // ORA zp
        cpu->a |= cpu_read(cpu, addr_zp(cpu));
        update_nz(cpu, cpu->a);
        cycles = 3;
        break;
    case 0x15: // ORA zp,X
        cpu->a |= cpu_read(cpu, addr_zpx(cpu));
        update_nz(cpu, cpu->a);
        cycles = 4;
        break;
    case 0x0D: // ORA abs
        cpu->a |= cpu_read(cpu, addr_abs(cpu));
        update_nz(cpu, cpu->a);
        cycles = 4;
        break;
    case 0x1D: // ORA abs,X
        addr = addr_abx(cpu, &page_crossed);
        cpu->a |= cpu_read(cpu, addr);
        update_nz(cpu, cpu->a);
        cycles = 4 + page_crossed;
        break;
    case 0x19: // ORA abs,Y
        addr = addr_aby(cpu, &page_crossed);
        cpu->a |= cpu_read(cpu, addr);
        update_nz(cpu, cpu->a);
        cycles = 4 + page_crossed;
        break;
    case 0x01: // ORA (zp,X)
        cpu->a |= cpu_read(cpu, addr_izx(cpu));
        update_nz(cpu, cpu->a);
        cycles = 6;
        break;
    case 0x11: // ORA (zp),Y
        addr = addr_izy(cpu, &page_crossed);
        cpu->a |= cpu_read(cpu, addr);
        update_nz(cpu, cpu->a);
        cycles = 5 + page_crossed;
        break;

    // ================================================================
    // EOR - Exclusive OR
    // ================================================================
    case 0x49: // EOR #imm
        cpu->a ^= cpu_read(cpu, addr_imm(cpu));
        update_nz(cpu, cpu->a);
        cycles = 2;
        break;
    case 0x45: // EOR zp
        cpu->a ^= cpu_read(cpu, addr_zp(cpu));
        update_nz(cpu, cpu->a);
        cycles = 3;
        break;
    case 0x55: // EOR zp,X
        cpu->a ^= cpu_read(cpu, addr_zpx(cpu));
        update_nz(cpu, cpu->a);
        cycles = 4;
        break;
    case 0x4D: // EOR abs
        cpu->a ^= cpu_read(cpu, addr_abs(cpu));
        update_nz(cpu, cpu->a);
        cycles = 4;
        break;
    case 0x5D: // EOR abs,X
        addr = addr_abx(cpu, &page_crossed);
        cpu->a ^= cpu_read(cpu, addr);
        update_nz(cpu, cpu->a);
        cycles = 4 + page_crossed;
        break;
    case 0x59: // EOR abs,Y
        addr = addr_aby(cpu, &page_crossed);
        cpu->a ^= cpu_read(cpu, addr);
        update_nz(cpu, cpu->a);
        cycles = 4 + page_crossed;
        break;
    case 0x41: // EOR (zp,X)
        cpu->a ^= cpu_read(cpu, addr_izx(cpu));
        update_nz(cpu, cpu->a);
        cycles = 6;
        break;
    case 0x51: // EOR (zp),Y
        addr = addr_izy(cpu, &page_crossed);
        cpu->a ^= cpu_read(cpu, addr);
        update_nz(cpu, cpu->a);
        cycles = 5 + page_crossed;
        break;

    // ================================================================
    // CMP - Compare Accumulator
    // ================================================================
    case 0xC9: // CMP #imm
        op_cmp(cpu, cpu->a, cpu_read(cpu, addr_imm(cpu)));
        cycles = 2;
        break;
    case 0xC5: // CMP zp
        op_cmp(cpu, cpu->a, cpu_read(cpu, addr_zp(cpu)));
        cycles = 3;
        break;
    case 0xD5: // CMP zp,X
        op_cmp(cpu, cpu->a, cpu_read(cpu, addr_zpx(cpu)));
        cycles = 4;
        break;
    case 0xCD: // CMP abs
        op_cmp(cpu, cpu->a, cpu_read(cpu, addr_abs(cpu)));
        cycles = 4;
        break;
    case 0xDD: // CMP abs,X
        addr = addr_abx(cpu, &page_crossed);
        op_cmp(cpu, cpu->a, cpu_read(cpu, addr));
        cycles = 4 + page_crossed;
        break;
    case 0xD9: // CMP abs,Y
        addr = addr_aby(cpu, &page_crossed);
        op_cmp(cpu, cpu->a, cpu_read(cpu, addr));
        cycles = 4 + page_crossed;
        break;
    case 0xC1: // CMP (zp,X)
        op_cmp(cpu, cpu->a, cpu_read(cpu, addr_izx(cpu)));
        cycles = 6;
        break;
    case 0xD1: // CMP (zp),Y
        addr = addr_izy(cpu, &page_crossed);
        op_cmp(cpu, cpu->a, cpu_read(cpu, addr));
        cycles = 5 + page_crossed;
        break;

    // ================================================================
    // CPX - Compare X Register
    // ================================================================
    case 0xE0: // CPX #imm
        op_cmp(cpu, cpu->x, cpu_read(cpu, addr_imm(cpu)));
        cycles = 2;
        break;
    case 0xE4: // CPX zp
        op_cmp(cpu, cpu->x, cpu_read(cpu, addr_zp(cpu)));
        cycles = 3;
        break;
    case 0xEC: // CPX abs
        op_cmp(cpu, cpu->x, cpu_read(cpu, addr_abs(cpu)));
        cycles = 4;
        break;

    // ================================================================
    // CPY - Compare Y Register
    // ================================================================
    case 0xC0: // CPY #imm
        op_cmp(cpu, cpu->y, cpu_read(cpu, addr_imm(cpu)));
        cycles = 2;
        break;
    case 0xC4: // CPY zp
        op_cmp(cpu, cpu->y, cpu_read(cpu, addr_zp(cpu)));
        cycles = 3;
        break;
    case 0xCC: // CPY abs
        op_cmp(cpu, cpu->y, cpu_read(cpu, addr_abs(cpu)));
        cycles = 4;
        break;

    // ================================================================
    // BIT - Bit Test
    // ================================================================
    case 0x24: // BIT zp
        val = cpu_read(cpu, addr_zp(cpu));
        set_flag(cpu, MOS6502_FLAG_Z, (cpu->a & val) == 0);
        set_flag(cpu, MOS6502_FLAG_N, (val & 0x80) != 0);
        set_flag(cpu, MOS6502_FLAG_V, (val & 0x40) != 0);
        cycles = 3;
        break;
    case 0x2C: // BIT abs
        val = cpu_read(cpu, addr_abs(cpu));
        set_flag(cpu, MOS6502_FLAG_Z, (cpu->a & val) == 0);
        set_flag(cpu, MOS6502_FLAG_N, (val & 0x80) != 0);
        set_flag(cpu, MOS6502_FLAG_V, (val & 0x40) != 0);
        cycles = 4;
        break;

    // ================================================================
    // ASL - Arithmetic Shift Left
    // ================================================================
    case 0x0A: // ASL A
        cpu->a = op_asl(cpu, cpu->a);
        cycles = 2;
        break;
    case 0x06: // ASL zp
        addr = addr_zp(cpu);
        cpu_write(cpu, addr, op_asl(cpu, cpu_read(cpu, addr)));
        cycles = 5;
        break;
    case 0x16: // ASL zp,X
        addr = addr_zpx(cpu);
        cpu_write(cpu, addr, op_asl(cpu, cpu_read(cpu, addr)));
        cycles = 6;
        break;
    case 0x0E: // ASL abs
        addr = addr_abs(cpu);
        cpu_write(cpu, addr, op_asl(cpu, cpu_read(cpu, addr)));
        cycles = 6;
        break;
    case 0x1E: // ASL abs,X
        addr = addr_abx(cpu, NULL);
        cpu_write(cpu, addr, op_asl(cpu, cpu_read(cpu, addr)));
        cycles = 7;
        break;

    // ================================================================
    // LSR - Logical Shift Right
    // ================================================================
    case 0x4A: // LSR A
        cpu->a = op_lsr(cpu, cpu->a);
        cycles = 2;
        break;
    case 0x46: // LSR zp
        addr = addr_zp(cpu);
        cpu_write(cpu, addr, op_lsr(cpu, cpu_read(cpu, addr)));
        cycles = 5;
        break;
    case 0x56: // LSR zp,X
        addr = addr_zpx(cpu);
        cpu_write(cpu, addr, op_lsr(cpu, cpu_read(cpu, addr)));
        cycles = 6;
        break;
    case 0x4E: // LSR abs
        addr = addr_abs(cpu);
        cpu_write(cpu, addr, op_lsr(cpu, cpu_read(cpu, addr)));
        cycles = 6;
        break;
    case 0x5E: // LSR abs,X
        addr = addr_abx(cpu, NULL);
        cpu_write(cpu, addr, op_lsr(cpu, cpu_read(cpu, addr)));
        cycles = 7;
        break;

    // ================================================================
    // ROL - Rotate Left
    // ================================================================
    case 0x2A: // ROL A
        cpu->a = op_rol(cpu, cpu->a);
        cycles = 2;
        break;
    case 0x26: // ROL zp
        addr = addr_zp(cpu);
        cpu_write(cpu, addr, op_rol(cpu, cpu_read(cpu, addr)));
        cycles = 5;
        break;
    case 0x36: // ROL zp,X
        addr = addr_zpx(cpu);
        cpu_write(cpu, addr, op_rol(cpu, cpu_read(cpu, addr)));
        cycles = 6;
        break;
    case 0x2E: // ROL abs
        addr = addr_abs(cpu);
        cpu_write(cpu, addr, op_rol(cpu, cpu_read(cpu, addr)));
        cycles = 6;
        break;
    case 0x3E: // ROL abs,X
        addr = addr_abx(cpu, NULL);
        cpu_write(cpu, addr, op_rol(cpu, cpu_read(cpu, addr)));
        cycles = 7;
        break;

    // ================================================================
    // ROR - Rotate Right
    // ================================================================
    case 0x6A: // ROR A
        cpu->a = op_ror(cpu, cpu->a);
        cycles = 2;
        break;
    case 0x66: // ROR zp
        addr = addr_zp(cpu);
        cpu_write(cpu, addr, op_ror(cpu, cpu_read(cpu, addr)));
        cycles = 5;
        break;
    case 0x76: // ROR zp,X
        addr = addr_zpx(cpu);
        cpu_write(cpu, addr, op_ror(cpu, cpu_read(cpu, addr)));
        cycles = 6;
        break;
    case 0x6E: // ROR abs
        addr = addr_abs(cpu);
        cpu_write(cpu, addr, op_ror(cpu, cpu_read(cpu, addr)));
        cycles = 6;
        break;
    case 0x7E: // ROR abs,X
        addr = addr_abx(cpu, NULL);
        cpu_write(cpu, addr, op_ror(cpu, cpu_read(cpu, addr)));
        cycles = 7;
        break;

    // ================================================================
    // INC - Increment Memory
    // ================================================================
    case 0xE6: // INC zp
        addr = addr_zp(cpu);
        val = cpu_read(cpu, addr) + 1;
        cpu_write(cpu, addr, val);
        update_nz(cpu, val);
        cycles = 5;
        break;
    case 0xF6: // INC zp,X
        addr = addr_zpx(cpu);
        val = cpu_read(cpu, addr) + 1;
        cpu_write(cpu, addr, val);
        update_nz(cpu, val);
        cycles = 6;
        break;
    case 0xEE: // INC abs
        addr = addr_abs(cpu);
        val = cpu_read(cpu, addr) + 1;
        cpu_write(cpu, addr, val);
        update_nz(cpu, val);
        cycles = 6;
        break;
    case 0xFE: // INC abs,X
        addr = addr_abx(cpu, NULL);
        val = cpu_read(cpu, addr) + 1;
        cpu_write(cpu, addr, val);
        update_nz(cpu, val);
        cycles = 7;
        break;

    // ================================================================
    // DEC - Decrement Memory
    // ================================================================
    case 0xC6: // DEC zp
        addr = addr_zp(cpu);
        val = cpu_read(cpu, addr) - 1;
        cpu_write(cpu, addr, val);
        update_nz(cpu, val);
        cycles = 5;
        break;
    case 0xD6: // DEC zp,X
        addr = addr_zpx(cpu);
        val = cpu_read(cpu, addr) - 1;
        cpu_write(cpu, addr, val);
        update_nz(cpu, val);
        cycles = 6;
        break;
    case 0xCE: // DEC abs
        addr = addr_abs(cpu);
        val = cpu_read(cpu, addr) - 1;
        cpu_write(cpu, addr, val);
        update_nz(cpu, val);
        cycles = 6;
        break;
    case 0xDE: // DEC abs,X
        addr = addr_abx(cpu, NULL);
        val = cpu_read(cpu, addr) - 1;
        cpu_write(cpu, addr, val);
        update_nz(cpu, val);
        cycles = 7;
        break;

    // ================================================================
    // INX, INY, DEX, DEY - Register Increment/Decrement
    // ================================================================
    case 0xE8: // INX
        cpu->x++;
        update_nz(cpu, cpu->x);
        cycles = 2;
        break;
    case 0xC8: // INY
        cpu->y++;
        update_nz(cpu, cpu->y);
        cycles = 2;
        break;
    case 0xCA: // DEX
        cpu->x--;
        update_nz(cpu, cpu->x);
        cycles = 2;
        break;
    case 0x88: // DEY
        cpu->y--;
        update_nz(cpu, cpu->y);
        cycles = 2;
        break;

    // ================================================================
    // Transfer instructions
    // ================================================================
    case 0xAA: // TAX
        cpu->x = cpu->a;
        update_nz(cpu, cpu->x);
        cycles = 2;
        break;
    case 0xA8: // TAY
        cpu->y = cpu->a;
        update_nz(cpu, cpu->y);
        cycles = 2;
        break;
    case 0x8A: // TXA
        cpu->a = cpu->x;
        update_nz(cpu, cpu->a);
        cycles = 2;
        break;
    case 0x98: // TYA
        cpu->a = cpu->y;
        update_nz(cpu, cpu->a);
        cycles = 2;
        break;
    case 0xBA: // TSX
        cpu->x = cpu->sp;
        update_nz(cpu, cpu->x);
        cycles = 2;
        break;
    case 0x9A: // TXS
        cpu->sp = cpu->x;
        cycles = 2;
        break;

    // ================================================================
    // Stack operations
    // ================================================================
    case 0x48: // PHA
        push8(cpu, cpu->a);
        cycles = 3;
        break;
    case 0x08: // PHP
        push8(cpu, cpu->status | MOS6502_FLAG_B | MOS6502_FLAG_U);
        cycles = 3;
        break;
    case 0x68: // PLA
        cpu->a = pull8(cpu);
        update_nz(cpu, cpu->a);
        cycles = 4;
        break;
    case 0x28: // PLP
        cpu->status = (pull8(cpu) & ~MOS6502_FLAG_B) | MOS6502_FLAG_U;
        cycles = 4;
        break;

    // ================================================================
    // Branch instructions
    // ================================================================
    case 0x10: // BPL - Branch if Positive
        cycles = 2 + do_branch(cpu, !get_flag(cpu, MOS6502_FLAG_N));
        break;
    case 0x30: // BMI - Branch if Minus
        cycles = 2 + do_branch(cpu, get_flag(cpu, MOS6502_FLAG_N));
        break;
    case 0x50: // BVC - Branch if Overflow Clear
        cycles = 2 + do_branch(cpu, !get_flag(cpu, MOS6502_FLAG_V));
        break;
    case 0x70: // BVS - Branch if Overflow Set
        cycles = 2 + do_branch(cpu, get_flag(cpu, MOS6502_FLAG_V));
        break;
    case 0x90: // BCC - Branch if Carry Clear
        cycles = 2 + do_branch(cpu, !get_flag(cpu, MOS6502_FLAG_C));
        break;
    case 0xB0: // BCS - Branch if Carry Set
        cycles = 2 + do_branch(cpu, get_flag(cpu, MOS6502_FLAG_C));
        break;
    case 0xD0: // BNE - Branch if Not Equal (Z=0)
        cycles = 2 + do_branch(cpu, !get_flag(cpu, MOS6502_FLAG_Z));
        break;
    case 0xF0: // BEQ - Branch if Equal (Z=1)
        cycles = 2 + do_branch(cpu, get_flag(cpu, MOS6502_FLAG_Z));
        break;

    // ================================================================
    // Jump and Subroutine
    // ================================================================
    case 0x4C: // JMP abs
        cpu->pc = addr_abs(cpu);
        cycles = 3;
        break;
    case 0x6C: // JMP (ind) - with page-wrapping bug
        addr = addr_abs(cpu);
        cpu->pc = cpu_read16_wrap(cpu, addr);
        cycles = 5;
        break;
    case 0x20: // JSR abs
        addr = addr_abs(cpu);
        push16(cpu, cpu->pc - 1);  // Push return address - 1
        cpu->pc = addr;
        cycles = 6;
        break;
    case 0x60: // RTS
        cpu->pc = pull16(cpu) + 1;
        cycles = 6;
        break;
    case 0x40: // RTI
        cpu->status = (pull8(cpu) & ~MOS6502_FLAG_B) | MOS6502_FLAG_U;
        cpu->pc = pull16(cpu);
        cycles = 6;
        break;

    // ================================================================
    // BRK - Software Interrupt
    // ================================================================
    case 0x00: // BRK
        cpu->pc++;  // BRK has a padding byte
        push16(cpu, cpu->pc);
        push8(cpu, cpu->status | MOS6502_FLAG_B | MOS6502_FLAG_U);
        set_flag(cpu, MOS6502_FLAG_I, true);
        cpu->pc = cpu_read16(cpu, MOS6502_VEC_IRQ);
        cycles = 7;
        break;

    // ================================================================
    // Flag instructions
    // ================================================================
    case 0x18: // CLC
        set_flag(cpu, MOS6502_FLAG_C, false);
        cycles = 2;
        break;
    case 0x38: // SEC
        set_flag(cpu, MOS6502_FLAG_C, true);
        cycles = 2;
        break;
    case 0x58: // CLI
        set_flag(cpu, MOS6502_FLAG_I, false);
        cycles = 2;
        break;
    case 0x78: // SEI
        set_flag(cpu, MOS6502_FLAG_I, true);
        cycles = 2;
        break;
    case 0xD8: // CLD
        set_flag(cpu, MOS6502_FLAG_D, false);
        cycles = 2;
        break;
    case 0xF8: // SED
        set_flag(cpu, MOS6502_FLAG_D, true);
        cycles = 2;
        break;
    case 0xB8: // CLV
        set_flag(cpu, MOS6502_FLAG_V, false);
        cycles = 2;
        break;

    // ================================================================
    // NOP
    // ================================================================
    case 0xEA: // NOP
        cycles = 2;
        break;

    // ================================================================
    // Illegal/undocumented opcodes - treat as NOP or JAM
    // ================================================================
    // Multi-byte NOPs (skip operand bytes)
    case 0x80: case 0x82: case 0x89: case 0xC2: case 0xE2: // 2-byte NOPs
    case 0x04: case 0x44: case 0x64:                         // DOP zp
    case 0x14: case 0x34: case 0x54: case 0x74:             // DOP zp,X
    case 0xD4: case 0xF4:
        cpu->pc++;  // Skip 1 operand byte
        cycles = 2;
        break;

    case 0x0C:                                               // TOP abs
    case 0x1C: case 0x3C: case 0x5C: case 0x7C:             // TOP abs,X
    case 0xDC: case 0xFC:
        cpu->pc += 2;  // Skip 2 operand bytes
        cycles = 4;
        break;

    case 0x1A: case 0x3A: case 0x5A: case 0x7A:             // 1-byte NOPs
    case 0xDA: case 0xFA:
        cycles = 2;
        break;

    // Undocumented SBC mirror
    case 0xEB: // SBC #imm (undocumented mirror of $E9)
        op_sbc(cpu, cpu_read(cpu, addr_imm(cpu)));
        cycles = 2;
        break;

    // JAM/HLT opcodes - halt the CPU
    case 0x02: case 0x12: case 0x22: case 0x32:
    case 0x42: case 0x52: case 0x62: case 0x72:
    case 0x92: case 0xB2: case 0xD2: case 0xF2:
        cpu->halted = true;
        cycles = 2;
        break;

    // LAX - Load A and X (undocumented but commonly used)
    case 0xA7: // LAX zp
        cpu->a = cpu->x = cpu_read(cpu, addr_zp(cpu));
        update_nz(cpu, cpu->a);
        cycles = 3;
        break;
    case 0xB7: // LAX zp,Y
        cpu->a = cpu->x = cpu_read(cpu, addr_zpy(cpu));
        update_nz(cpu, cpu->a);
        cycles = 4;
        break;
    case 0xAF: // LAX abs
        cpu->a = cpu->x = cpu_read(cpu, addr_abs(cpu));
        update_nz(cpu, cpu->a);
        cycles = 4;
        break;
    case 0xBF: // LAX abs,Y
        addr = addr_aby(cpu, &page_crossed);
        cpu->a = cpu->x = cpu_read(cpu, addr);
        update_nz(cpu, cpu->a);
        cycles = 4 + page_crossed;
        break;
    case 0xA3: // LAX (zp,X)
        cpu->a = cpu->x = cpu_read(cpu, addr_izx(cpu));
        update_nz(cpu, cpu->a);
        cycles = 6;
        break;
    case 0xB3: // LAX (zp),Y
        addr = addr_izy(cpu, &page_crossed);
        cpu->a = cpu->x = cpu_read(cpu, addr);
        update_nz(cpu, cpu->a);
        cycles = 5 + page_crossed;
        break;

    // SAX - Store A AND X (undocumented)
    case 0x87: // SAX zp
        cpu_write(cpu, addr_zp(cpu), cpu->a & cpu->x);
        cycles = 3;
        break;
    case 0x97: // SAX zp,Y
        cpu_write(cpu, addr_zpy(cpu), cpu->a & cpu->x);
        cycles = 4;
        break;
    case 0x8F: // SAX abs
        cpu_write(cpu, addr_abs(cpu), cpu->a & cpu->x);
        cycles = 4;
        break;
    case 0x83: // SAX (zp,X)
        cpu_write(cpu, addr_izx(cpu), cpu->a & cpu->x);
        cycles = 6;
        break;

    // DCP - Decrement memory then Compare (undocumented)
    case 0xC7: // DCP zp
        addr = addr_zp(cpu);
        val = cpu_read(cpu, addr) - 1;
        cpu_write(cpu, addr, val);
        op_cmp(cpu, cpu->a, val);
        cycles = 5;
        break;
    case 0xD7: // DCP zp,X
        addr = addr_zpx(cpu);
        val = cpu_read(cpu, addr) - 1;
        cpu_write(cpu, addr, val);
        op_cmp(cpu, cpu->a, val);
        cycles = 6;
        break;
    case 0xCF: // DCP abs
        addr = addr_abs(cpu);
        val = cpu_read(cpu, addr) - 1;
        cpu_write(cpu, addr, val);
        op_cmp(cpu, cpu->a, val);
        cycles = 6;
        break;
    case 0xDF: // DCP abs,X
        addr = addr_abx(cpu, NULL);
        val = cpu_read(cpu, addr) - 1;
        cpu_write(cpu, addr, val);
        op_cmp(cpu, cpu->a, val);
        cycles = 7;
        break;
    case 0xDB: // DCP abs,Y
        addr = addr_aby(cpu, NULL);
        val = cpu_read(cpu, addr) - 1;
        cpu_write(cpu, addr, val);
        op_cmp(cpu, cpu->a, val);
        cycles = 7;
        break;
    case 0xC3: // DCP (zp,X)
        addr = addr_izx(cpu);
        val = cpu_read(cpu, addr) - 1;
        cpu_write(cpu, addr, val);
        op_cmp(cpu, cpu->a, val);
        cycles = 8;
        break;
    case 0xD3: // DCP (zp),Y
        addr = addr_izy(cpu, NULL);
        val = cpu_read(cpu, addr) - 1;
        cpu_write(cpu, addr, val);
        op_cmp(cpu, cpu->a, val);
        cycles = 8;
        break;

    // ISC/ISB - Increment memory then Subtract (undocumented)
    case 0xE7: // ISC zp
        addr = addr_zp(cpu);
        val = cpu_read(cpu, addr) + 1;
        cpu_write(cpu, addr, val);
        op_sbc(cpu, val);
        cycles = 5;
        break;
    case 0xF7: // ISC zp,X
        addr = addr_zpx(cpu);
        val = cpu_read(cpu, addr) + 1;
        cpu_write(cpu, addr, val);
        op_sbc(cpu, val);
        cycles = 6;
        break;
    case 0xEF: // ISC abs
        addr = addr_abs(cpu);
        val = cpu_read(cpu, addr) + 1;
        cpu_write(cpu, addr, val);
        op_sbc(cpu, val);
        cycles = 6;
        break;
    case 0xFF: // ISC abs,X
        addr = addr_abx(cpu, NULL);
        val = cpu_read(cpu, addr) + 1;
        cpu_write(cpu, addr, val);
        op_sbc(cpu, val);
        cycles = 7;
        break;
    case 0xFB: // ISC abs,Y
        addr = addr_aby(cpu, NULL);
        val = cpu_read(cpu, addr) + 1;
        cpu_write(cpu, addr, val);
        op_sbc(cpu, val);
        cycles = 7;
        break;
    case 0xE3: // ISC (zp,X)
        addr = addr_izx(cpu);
        val = cpu_read(cpu, addr) + 1;
        cpu_write(cpu, addr, val);
        op_sbc(cpu, val);
        cycles = 8;
        break;
    case 0xF3: // ISC (zp),Y
        addr = addr_izy(cpu, NULL);
        val = cpu_read(cpu, addr) + 1;
        cpu_write(cpu, addr, val);
        op_sbc(cpu, val);
        cycles = 8;
        break;

    // SLO - ASL then ORA (undocumented)
    case 0x07: // SLO zp
        addr = addr_zp(cpu);
        val = op_asl(cpu, cpu_read(cpu, addr));
        cpu_write(cpu, addr, val);
        cpu->a |= val;
        update_nz(cpu, cpu->a);
        cycles = 5;
        break;
    case 0x17: // SLO zp,X
        addr = addr_zpx(cpu);
        val = op_asl(cpu, cpu_read(cpu, addr));
        cpu_write(cpu, addr, val);
        cpu->a |= val;
        update_nz(cpu, cpu->a);
        cycles = 6;
        break;
    case 0x0F: // SLO abs
        addr = addr_abs(cpu);
        val = op_asl(cpu, cpu_read(cpu, addr));
        cpu_write(cpu, addr, val);
        cpu->a |= val;
        update_nz(cpu, cpu->a);
        cycles = 6;
        break;
    case 0x1F: // SLO abs,X
        addr = addr_abx(cpu, NULL);
        val = op_asl(cpu, cpu_read(cpu, addr));
        cpu_write(cpu, addr, val);
        cpu->a |= val;
        update_nz(cpu, cpu->a);
        cycles = 7;
        break;
    case 0x1B: // SLO abs,Y
        addr = addr_aby(cpu, NULL);
        val = op_asl(cpu, cpu_read(cpu, addr));
        cpu_write(cpu, addr, val);
        cpu->a |= val;
        update_nz(cpu, cpu->a);
        cycles = 7;
        break;
    case 0x03: // SLO (zp,X)
        addr = addr_izx(cpu);
        val = op_asl(cpu, cpu_read(cpu, addr));
        cpu_write(cpu, addr, val);
        cpu->a |= val;
        update_nz(cpu, cpu->a);
        cycles = 8;
        break;
    case 0x13: // SLO (zp),Y
        addr = addr_izy(cpu, NULL);
        val = op_asl(cpu, cpu_read(cpu, addr));
        cpu_write(cpu, addr, val);
        cpu->a |= val;
        update_nz(cpu, cpu->a);
        cycles = 8;
        break;

    // RLA - ROL then AND (undocumented)
    case 0x27: // RLA zp
        addr = addr_zp(cpu);
        val = op_rol(cpu, cpu_read(cpu, addr));
        cpu_write(cpu, addr, val);
        cpu->a &= val;
        update_nz(cpu, cpu->a);
        cycles = 5;
        break;
    case 0x37: // RLA zp,X
        addr = addr_zpx(cpu);
        val = op_rol(cpu, cpu_read(cpu, addr));
        cpu_write(cpu, addr, val);
        cpu->a &= val;
        update_nz(cpu, cpu->a);
        cycles = 6;
        break;
    case 0x2F: // RLA abs
        addr = addr_abs(cpu);
        val = op_rol(cpu, cpu_read(cpu, addr));
        cpu_write(cpu, addr, val);
        cpu->a &= val;
        update_nz(cpu, cpu->a);
        cycles = 6;
        break;
    case 0x3F: // RLA abs,X
        addr = addr_abx(cpu, NULL);
        val = op_rol(cpu, cpu_read(cpu, addr));
        cpu_write(cpu, addr, val);
        cpu->a &= val;
        update_nz(cpu, cpu->a);
        cycles = 7;
        break;
    case 0x3B: // RLA abs,Y
        addr = addr_aby(cpu, NULL);
        val = op_rol(cpu, cpu_read(cpu, addr));
        cpu_write(cpu, addr, val);
        cpu->a &= val;
        update_nz(cpu, cpu->a);
        cycles = 7;
        break;
    case 0x23: // RLA (zp,X)
        addr = addr_izx(cpu);
        val = op_rol(cpu, cpu_read(cpu, addr));
        cpu_write(cpu, addr, val);
        cpu->a &= val;
        update_nz(cpu, cpu->a);
        cycles = 8;
        break;
    case 0x33: // RLA (zp),Y
        addr = addr_izy(cpu, NULL);
        val = op_rol(cpu, cpu_read(cpu, addr));
        cpu_write(cpu, addr, val);
        cpu->a &= val;
        update_nz(cpu, cpu->a);
        cycles = 8;
        break;

    // SRE - LSR then EOR (undocumented)
    case 0x47: // SRE zp
        addr = addr_zp(cpu);
        val = op_lsr(cpu, cpu_read(cpu, addr));
        cpu_write(cpu, addr, val);
        cpu->a ^= val;
        update_nz(cpu, cpu->a);
        cycles = 5;
        break;
    case 0x57: // SRE zp,X
        addr = addr_zpx(cpu);
        val = op_lsr(cpu, cpu_read(cpu, addr));
        cpu_write(cpu, addr, val);
        cpu->a ^= val;
        update_nz(cpu, cpu->a);
        cycles = 6;
        break;
    case 0x4F: // SRE abs
        addr = addr_abs(cpu);
        val = op_lsr(cpu, cpu_read(cpu, addr));
        cpu_write(cpu, addr, val);
        cpu->a ^= val;
        update_nz(cpu, cpu->a);
        cycles = 6;
        break;
    case 0x5F: // SRE abs,X
        addr = addr_abx(cpu, NULL);
        val = op_lsr(cpu, cpu_read(cpu, addr));
        cpu_write(cpu, addr, val);
        cpu->a ^= val;
        update_nz(cpu, cpu->a);
        cycles = 7;
        break;
    case 0x5B: // SRE abs,Y
        addr = addr_aby(cpu, NULL);
        val = op_lsr(cpu, cpu_read(cpu, addr));
        cpu_write(cpu, addr, val);
        cpu->a ^= val;
        update_nz(cpu, cpu->a);
        cycles = 7;
        break;
    case 0x43: // SRE (zp,X)
        addr = addr_izx(cpu);
        val = op_lsr(cpu, cpu_read(cpu, addr));
        cpu_write(cpu, addr, val);
        cpu->a ^= val;
        update_nz(cpu, cpu->a);
        cycles = 8;
        break;
    case 0x53: // SRE (zp),Y
        addr = addr_izy(cpu, NULL);
        val = op_lsr(cpu, cpu_read(cpu, addr));
        cpu_write(cpu, addr, val);
        cpu->a ^= val;
        update_nz(cpu, cpu->a);
        cycles = 8;
        break;

    // RRA - ROR then ADC (undocumented)
    case 0x67: // RRA zp
        addr = addr_zp(cpu);
        val = op_ror(cpu, cpu_read(cpu, addr));
        cpu_write(cpu, addr, val);
        op_adc(cpu, val);
        cycles = 5;
        break;
    case 0x77: // RRA zp,X
        addr = addr_zpx(cpu);
        val = op_ror(cpu, cpu_read(cpu, addr));
        cpu_write(cpu, addr, val);
        op_adc(cpu, val);
        cycles = 6;
        break;
    case 0x6F: // RRA abs
        addr = addr_abs(cpu);
        val = op_ror(cpu, cpu_read(cpu, addr));
        cpu_write(cpu, addr, val);
        op_adc(cpu, val);
        cycles = 6;
        break;
    case 0x7F: // RRA abs,X
        addr = addr_abx(cpu, NULL);
        val = op_ror(cpu, cpu_read(cpu, addr));
        cpu_write(cpu, addr, val);
        op_adc(cpu, val);
        cycles = 7;
        break;
    case 0x7B: // RRA abs,Y
        addr = addr_aby(cpu, NULL);
        val = op_ror(cpu, cpu_read(cpu, addr));
        cpu_write(cpu, addr, val);
        op_adc(cpu, val);
        cycles = 7;
        break;
    case 0x63: // RRA (zp,X)
        addr = addr_izx(cpu);
        val = op_ror(cpu, cpu_read(cpu, addr));
        cpu_write(cpu, addr, val);
        op_adc(cpu, val);
        cycles = 8;
        break;
    case 0x73: // RRA (zp),Y
        addr = addr_izy(cpu, NULL);
        val = op_ror(cpu, cpu_read(cpu, addr));
        cpu_write(cpu, addr, val);
        op_adc(cpu, val);
        cycles = 8;
        break;

    // ANC - AND then copy N to C (undocumented)
    case 0x0B: case 0x2B:
        cpu->a &= cpu_read(cpu, addr_imm(cpu));
        update_nz(cpu, cpu->a);
        set_flag(cpu, MOS6502_FLAG_C, (cpu->a & 0x80) != 0);
        cycles = 2;
        break;

    // ALR - AND then LSR (undocumented)
    case 0x4B:
        cpu->a &= cpu_read(cpu, addr_imm(cpu));
        cpu->a = op_lsr(cpu, cpu->a);
        cycles = 2;
        break;

    // ARR - AND then ROR with special flag handling (undocumented)
    case 0x6B:
        cpu->a &= cpu_read(cpu, addr_imm(cpu));
        cpu->a = op_ror(cpu, cpu->a);
        // Special flag handling for ARR
        set_flag(cpu, MOS6502_FLAG_C, (cpu->a & 0x40) != 0);
        set_flag(cpu, MOS6502_FLAG_V, ((cpu->a & 0x40) ^ ((cpu->a & 0x20) << 1)) != 0);
        cycles = 2;
        break;

    // AXS/SBX - AND X with A, subtract immediate (undocumented)
    case 0xCB: {
        uint8_t imm = cpu_read(cpu, addr_imm(cpu));
        uint8_t ax = cpu->a & cpu->x;
        uint16_t result = (uint16_t)ax - imm;
        cpu->x = (uint8_t)result;
        set_flag(cpu, MOS6502_FLAG_C, ax >= imm);
        update_nz(cpu, cpu->x);
        cycles = 2;
        break;
    }

    // Remaining undocumented opcodes treated as 1-byte NOPs
    default:
        // Unknown opcode - treat as NOP
        cycles = 2;
        break;
    }

    cpu->cycles += cycles;
    cpu->instructions++;
    return cycles;
}

uint32_t mos6502_run(mos6502_t *cpu, uint32_t count) {
    uint32_t executed = 0;
    while (executed < count && !cpu->halted) {
        mos6502_step(cpu);
        executed++;
    }
    return executed;
}
