/*
 * z80_cpu.c - Zilog Z80 CPU emulator
 *
 * Complete implementation of the Z80 instruction set including:
 *   All 252 base opcodes, CB/DD/FD/ED/DDCB/FDCB prefix groups,
 *   interrupt modes IM0/1/2, NMI, and undocumented instructions.
 *
 * Cycle counts are approximate (based on official Z80 timing).
 */

#include "z80_cpu.h"
#include <stdio.h>
#include <string.h>
#include "esp_log.h"

static const char *TAG = "Z80";

/* =====================================================================
 * Internal register pair macros
 * ===================================================================== */
#define AF(c)    ((uint16_t)((c)->A << 8 | (c)->F))
#define BC(c)    ((uint16_t)((c)->B << 8 | (c)->C))
#define DE(c)    ((uint16_t)((c)->D << 8 | (c)->E))
#define HL(c)    ((uint16_t)((c)->H << 8 | (c)->L))

#define SET_BC(c,v) do { (c)->B=(uint8_t)((v)>>8); (c)->C=(uint8_t)(v); } while(0)
#define SET_DE(c,v) do { (c)->D=(uint8_t)((v)>>8); (c)->E=(uint8_t)(v); } while(0)
#define SET_HL(c,v) do { (c)->H=(uint8_t)((v)>>8); (c)->L=(uint8_t)(v); } while(0)

/* Flag bits */
#define FC   0x01
#define FN   0x02
#define FPV  0x04
#define FX   0x08
#define FH   0x10
#define FY   0x20
#define FZ   0x40
#define FS   0x80

/* =====================================================================
 * Parity lookup (1 = even parity)
 * ===================================================================== */
static const uint8_t s_parity[256] = {
    1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,
    0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
    0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
    1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,
    0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
    1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,
    1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,
    0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
};

/* =====================================================================
 * Memory / IO access helpers
 * ===================================================================== */
static inline uint8_t MR(z80_t *c, uint16_t a) {
    return c->mem_read(c->user_data, a);
}
static inline void MW(z80_t *c, uint16_t a, uint8_t v) {
    c->mem_write(c->user_data, a, v);
}
static inline uint8_t IOR(z80_t *c, uint16_t p) {
    return c->io_read ? c->io_read(c->user_data, p) : 0xFF;
}
static inline void IOW(z80_t *c, uint16_t p, uint8_t v) {
    if (c->io_write) c->io_write(c->user_data, p, v);
}

/* Fetch byte at PC and advance; R register incremented */
static inline uint8_t FETCH(z80_t *c) {
    uint8_t b = MR(c, c->PC++);
    c->R = (c->R & 0x80) | ((c->R + 1) & 0x7F);
    return b;
}
/* Fetch 16-bit little-endian immediate */
static inline uint16_t FETCH16(z80_t *c) {
    uint16_t lo = MR(c, c->PC++);
    uint16_t hi = MR(c, c->PC++);
    return (hi << 8) | lo;
}
/* Read 16-bit from address (little-endian) */
static inline uint16_t RM16(z80_t *c, uint16_t a) {
    return MR(c,a) | ((uint16_t)MR(c,(uint16_t)(a+1)) << 8);
}
/* Write 16-bit to address (little-endian) */
static inline void WM16(z80_t *c, uint16_t a, uint16_t v) {
    MW(c, a,   (uint8_t)(v));
    MW(c, (uint16_t)(a+1), (uint8_t)(v >> 8));
}
/* Stack push/pop */
static inline void PUSH(z80_t *c, uint16_t v) {
    c->SP -= 2;
    WM16(c, c->SP, v);
}
static inline uint16_t POP(z80_t *c) {
    uint16_t v = RM16(c, c->SP);
    c->SP += 2;
    return v;
}

/* =====================================================================
 * Flag helpers
 * ===================================================================== */

/* Flags after 8-bit ADD A + b + cy: returns new F, caller updates A */
static inline uint8_t fl_add8(uint8_t a, uint8_t b, uint8_t cy) {
    uint16_t r = (uint16_t)a + b + cy;
    uint8_t  rb = (uint8_t)r;
    uint8_t f = 0;
    if (!rb)              f |= FZ;
    if (rb & 0x80)        f |= FS;
    if (r > 0xFF)         f |= FC;
    if ((a ^ b ^ r) & 0x10) f |= FH;
    if (~(a ^ b) & (a ^ r) & 0x80) f |= FPV;
    f |= rb & (FX | FY);
    return f;
}

/* Flags after 8-bit SUB A - b - borrow */
static inline uint8_t fl_sub8(uint8_t a, uint8_t b, uint8_t borrow) {
    uint16_t r = (uint16_t)a - b - borrow;
    uint8_t  rb = (uint8_t)r;
    uint8_t f = FN;
    if (!rb)              f |= FZ;
    if (rb & 0x80)        f |= FS;
    if (r > 0xFF)         f |= FC;
    if ((a ^ b ^ r) & 0x10) f |= FH;
    if ((a ^ b) & (a ^ r) & 0x80) f |= FPV;
    f |= rb & (FX | FY);
    return f;
}

/* Flags after CP A - b (result not stored; undocumented bits from b) */
static inline uint8_t fl_cp8(uint8_t a, uint8_t b) {
    uint16_t r = (uint16_t)a - b;
    uint8_t  rb = (uint8_t)r;
    uint8_t f = FN;
    if (!rb)              f |= FZ;
    if (rb & 0x80)        f |= FS;
    if (r > 0xFF)         f |= FC;
    if ((a ^ b ^ r) & 0x10) f |= FH;
    if ((a ^ b) & (a ^ r) & 0x80) f |= FPV;
    f |= b & (FX | FY); /* undocumented: from b, not result */
    return f;
}

/* Flags after AND/OR/XOR: h_bit=1 for AND, 0 for OR/XOR */
static inline uint8_t fl_logic(uint8_t r, int h_bit) {
    uint8_t f = 0;
    if (!r)           f |= FZ;
    if (r & 0x80)     f |= FS;
    if (s_parity[r])  f |= FPV;
    if (h_bit)        f |= FH;
    f |= r & (FX | FY);
    return f;
}

/* INC / DEC 8-bit (preserves C flag) */
static inline uint8_t do_inc8(z80_t *c, uint8_t v) {
    uint8_t r = v + 1;
    uint8_t f = c->F & FC;
    if (!r)           f |= FZ;
    if (r & 0x80)     f |= FS;
    if ((v & 0x0F) == 0x0F) f |= FH;
    if (v == 0x7F)    f |= FPV;
    f |= r & (FX | FY);
    c->F = f;
    return r;
}
static inline uint8_t do_dec8(z80_t *c, uint8_t v) {
    uint8_t r = v - 1;
    uint8_t f = FN | (c->F & FC);
    if (!r)           f |= FZ;
    if (r & 0x80)     f |= FS;
    if (!(v & 0x0F))  f |= FH;
    if (v == 0x80)    f |= FPV;
    f |= r & (FX | FY);
    c->F = f;
    return r;
}

/* 16-bit ADD (affects only C, H, N; not S, Z, PV) */
static inline uint16_t do_add16(z80_t *c, uint16_t a, uint16_t b) {
    uint32_t r = (uint32_t)a + b;
    uint8_t f = c->F & (FS | FZ | FPV);
    if (r > 0xFFFF) f |= FC;
    if ((a ^ b ^ r) & 0x1000) f |= FH;
    f |= (uint8_t)((r >> 8) & (FX | FY));
    c->F = f;
    return (uint16_t)r;
}

/* 16-bit ADC HL */
static inline uint16_t do_adc16(z80_t *c, uint16_t a, uint16_t b) {
    uint32_t cy = (c->F & FC) ? 1 : 0;
    uint32_t r  = (uint32_t)a + b + cy;
    uint8_t  f  = 0;
    if (!(r & 0xFFFF))   f |= FZ;
    if (r & 0x8000)      f |= FS;
    if (r > 0xFFFF)      f |= FC;
    if ((a ^ b ^ r) & 0x1000) f |= FH;
    if (~(a ^ b) & (a ^ r) & 0x8000) f |= FPV;
    f |= (uint8_t)((r >> 8) & (FX | FY));
    c->F = f;
    return (uint16_t)r;
}

/* 16-bit SBC HL */
static inline uint16_t do_sbc16(z80_t *c, uint16_t a, uint16_t b) {
    uint32_t borrow = (c->F & FC) ? 1 : 0;
    uint32_t r = (uint32_t)a - b - borrow;
    uint8_t  f  = FN;
    if (!(r & 0xFFFF))   f |= FZ;
    if (r & 0x8000)      f |= FS;
    if (r > 0xFFFF)      f |= FC;
    if ((a ^ b ^ r) & 0x1000) f |= FH;
    if ((a ^ b) & (a ^ r) & 0x8000) f |= FPV;
    f |= (uint8_t)((r >> 8) & (FX | FY));
    c->F = f;
    return (uint16_t)r;
}

/* =====================================================================
 * Inline ALU helpers (update A and F)
 * ===================================================================== */
static inline void add_a(z80_t *c, uint8_t v) {
    c->F = fl_add8(c->A, v, 0); c->A += v;
}
static inline void adc_a(z80_t *c, uint8_t v) {
    uint8_t cy = c->F & FC ? 1 : 0;
    uint8_t nf = fl_add8(c->A, v, cy);
    c->A += v + cy;
    c->F = nf;
}
static inline void sub_a(z80_t *c, uint8_t v) {
    c->F = fl_sub8(c->A, v, 0); c->A -= v;
}
static inline void sbc_a(z80_t *c, uint8_t v) {
    uint8_t borrow = c->F & FC ? 1 : 0;
    uint8_t nf = fl_sub8(c->A, v, borrow);
    c->A -= v + borrow;
    c->F = nf;
}
static inline void and_a(z80_t *c, uint8_t v) { c->A &= v; c->F = fl_logic(c->A, 1); }
static inline void or_a (z80_t *c, uint8_t v) { c->A |= v; c->F = fl_logic(c->A, 0); }
static inline void xor_a(z80_t *c, uint8_t v) { c->A ^= v; c->F = fl_logic(c->A, 0); }
static inline void cp_a (z80_t *c, uint8_t v) { c->F = fl_cp8(c->A, v); }

/* =====================================================================
 * Get/Set 8-bit registers by index  (B=0 C=1 D=2 E=3 H=4 L=5 (HL)=6 A=7)
 * ===================================================================== */
static inline uint8_t GET_R(z80_t *c, int r) {
    switch (r) {
        case 0: return c->B;
        case 1: return c->C;
        case 2: return c->D;
        case 3: return c->E;
        case 4: return c->H;
        case 5: return c->L;
        case 6: return MR(c, HL(c));
        case 7: return c->A;
    }
    return 0;
}
static inline void SET_R(z80_t *c, int r, uint8_t v) {
    switch (r) {
        case 0: c->B = v; break;
        case 1: c->C = v; break;
        case 2: c->D = v; break;
        case 3: c->E = v; break;
        case 4: c->H = v; break;
        case 5: c->L = v; break;
        case 6: MW(c, HL(c), v); break;
        case 7: c->A = v; break;
    }
}

/* =====================================================================
 * Accumulator rotates (RLCA/RRCA/RLA/RRA) - simpler flags
 * ===================================================================== */
static inline void do_rlca(z80_t *c) {
    uint8_t cy = c->A >> 7;
    c->A = (c->A << 1) | cy;
    c->F = (c->F & (FS|FZ|FPV)) | (cy ? FC : 0) | (c->A & (FX|FY));
}
static inline void do_rrca(z80_t *c) {
    uint8_t cy = c->A & 1;
    c->A = (c->A >> 1) | (cy << 7);
    c->F = (c->F & (FS|FZ|FPV)) | (cy ? FC : 0) | (c->A & (FX|FY));
}
static inline void do_rla(z80_t *c) {
    uint8_t new_cy = c->A >> 7;
    c->A = (c->A << 1) | (c->F & FC ? 1 : 0);
    c->F = (c->F & (FS|FZ|FPV)) | (new_cy ? FC : 0) | (c->A & (FX|FY));
}
static inline void do_rra(z80_t *c) {
    uint8_t new_cy = c->A & 1;
    c->A = (c->A >> 1) | (c->F & FC ? 0x80 : 0);
    c->F = (c->F & (FS|FZ|FPV)) | (new_cy ? FC : 0) | (c->A & (FX|FY));
}

/* =====================================================================
 * DAA
 * ===================================================================== */
static inline void do_daa(z80_t *c) {
    uint8_t a = c->A;
    uint8_t f = c->F;
    uint8_t corr = 0;
    bool new_c = (f & FC) != 0;

    if (f & FN) {
        if (f & FH) corr |= 0x06;
        if (f & FC) corr |= 0x60;
        a -= corr;
    } else {
        if ((f & FH) || (a & 0x0F) > 9)  corr |= 0x06;
        if ((f & FC) || a > 0x99)         { corr |= 0x60; new_c = true; }
        a += corr;
    }

    c->F = (f & FN)
         | (a ? 0 : FZ)
         | (a & 0x80 ? FS : 0)
         | (new_c ? FC : 0)
         | (s_parity[a] ? FPV : 0)
         | ((c->A ^ a) & FH)
         | (a & (FX|FY));
    c->A = a;
}

/* =====================================================================
 * CPL, SCF, CCF
 * ===================================================================== */
static inline void do_cpl(z80_t *c) {
    c->A = ~c->A;
    c->F = (c->F & (FS|FZ|FPV|FC)) | FH | FN | (c->A & (FX|FY));
}
static inline void do_scf(z80_t *c) {
    c->F = (c->F & (FS|FZ|FPV)) | FC | (c->A & (FX|FY));
}
static inline void do_ccf(z80_t *c) {
    uint8_t old_c = c->F & FC;
    c->F = (c->F & (FS|FZ|FPV)) | (old_c ? FH : 0) | (old_c ? 0 : FC) | (c->A & (FX|FY));
}

/* =====================================================================
 * CB prefix: rotate/shift/bit operations
 * ===================================================================== */
static inline uint8_t cb_rlc(z80_t *c, uint8_t v) {
    uint8_t r = (v << 1) | (v >> 7);
    c->F = (r ? 0 : FZ) | (r & FS) | (s_parity[r] ? FPV : 0) | (v >> 7 ? FC : 0) | (r & (FX|FY));
    return r;
}
static inline uint8_t cb_rrc(z80_t *c, uint8_t v) {
    uint8_t r = (v >> 1) | (v << 7);
    c->F = (r ? 0 : FZ) | (r & FS) | (s_parity[r] ? FPV : 0) | (v & 1 ? FC : 0) | (r & (FX|FY));
    return r;
}
static inline uint8_t cb_rl(z80_t *c, uint8_t v) {
    uint8_t r = (v << 1) | (c->F & FC ? 1 : 0);
    c->F = (r ? 0 : FZ) | (r & FS) | (s_parity[r] ? FPV : 0) | (v >> 7 ? FC : 0) | (r & (FX|FY));
    return r;
}
static inline uint8_t cb_rr(z80_t *c, uint8_t v) {
    uint8_t r = (v >> 1) | (c->F & FC ? 0x80 : 0);
    c->F = (r ? 0 : FZ) | (r & FS) | (s_parity[r] ? FPV : 0) | (v & 1 ? FC : 0) | (r & (FX|FY));
    return r;
}
static inline uint8_t cb_sla(z80_t *c, uint8_t v) {
    uint8_t r = v << 1;
    c->F = (r ? 0 : FZ) | (r & FS) | (s_parity[r] ? FPV : 0) | (v >> 7 ? FC : 0) | (r & (FX|FY));
    return r;
}
static inline uint8_t cb_sra(z80_t *c, uint8_t v) {
    uint8_t r = (v >> 1) | (v & 0x80);
    c->F = (r ? 0 : FZ) | (r & FS) | (s_parity[r] ? FPV : 0) | (v & 1 ? FC : 0) | (r & (FX|FY));
    return r;
}
static inline uint8_t cb_sll(z80_t *c, uint8_t v) { /* undocumented SLL */
    uint8_t r = (v << 1) | 1;
    c->F = (r ? 0 : FZ) | (r & FS) | (s_parity[r] ? FPV : 0) | (v >> 7 ? FC : 0) | (r & (FX|FY));
    return r;
}
static inline uint8_t cb_srl(z80_t *c, uint8_t v) {
    uint8_t r = v >> 1;
    c->F = (r ? 0 : FZ) | (r & FS) | (s_parity[r] ? FPV : 0) | (v & 1 ? FC : 0) | (r & (FX|FY));
    return r;
}
static inline void cb_bit(z80_t *c, uint8_t bit, uint8_t v) {
    uint8_t b = v & (1 << bit);
    c->F = (c->F & FC) | FH | (b ? 0 : (FZ|FPV)) | (b & FS) | (v & (FX|FY));
}
static inline uint8_t cb_res(uint8_t bit, uint8_t v) { return v & ~(1 << bit); }
static inline uint8_t cb_set(uint8_t bit, uint8_t v) { return v |  (1 << bit); }

/* Execute CB-prefixed instruction */
static int exec_cb(z80_t *c) {
    uint8_t op  = FETCH(c);
    int     reg = op & 7;
    int     bit = (op >> 3) & 7;
    uint8_t v   = GET_R(c, reg);
    int cycles  = (reg == 6) ? 15 : 8;

    if (op < 0x40) {
        switch (op >> 3) {
            case 0: v = cb_rlc(c, v); break;
            case 1: v = cb_rrc(c, v); break;
            case 2: v = cb_rl (c, v); break;
            case 3: v = cb_rr (c, v); break;
            case 4: v = cb_sla(c, v); break;
            case 5: v = cb_sra(c, v); break;
            case 6: v = cb_sll(c, v); break;
            case 7: v = cb_srl(c, v); break;
        }
        SET_R(c, reg, v);
    } else if (op < 0x80) {
        cb_bit(c, bit, v);
        cycles = (reg == 6) ? 12 : 8;
    } else if (op < 0xC0) {
        SET_R(c, reg, cb_res(bit, v));
    } else {
        SET_R(c, reg, cb_set(bit, v));
    }
    return cycles;
}

/* =====================================================================
 * DDCB / FDCB prefix handler (indexed bit operations)
 * ===================================================================== */
static int exec_xycb(z80_t *c, uint16_t xy_base) {
    int8_t   d  = (int8_t)MR(c, c->PC++); /* displacement (no R increment) */
    uint8_t  op = MR(c, c->PC++);          /* opcode (no R increment) */
    uint16_t ea = (uint16_t)(xy_base + d);
    int      bit = (op >> 3) & 7;
    int      dest_reg = op & 7;             /* undocumented: also copy to register */
    uint8_t  v  = MR(c, ea);

    if (op < 0x40) {
        switch (op >> 3) {
            case 0: v = cb_rlc(c, v); break;
            case 1: v = cb_rrc(c, v); break;
            case 2: v = cb_rl (c, v); break;
            case 3: v = cb_rr (c, v); break;
            case 4: v = cb_sla(c, v); break;
            case 5: v = cb_sra(c, v); break;
            case 6: v = cb_sll(c, v); break;
            case 7: v = cb_srl(c, v); break;
        }
        MW(c, ea, v);
        if (dest_reg != 6) SET_R(c, dest_reg, v);
        return 23;
    } else if (op < 0x80) {
        uint8_t b = v & (1 << bit);
        c->F = (c->F & FC) | FH | (b ? 0 : (FZ|FPV)) | (b & FS) | (uint8_t)((ea >> 8) & (FX|FY));
        return 20;
    } else if (op < 0xC0) {
        v = cb_res(bit, v);
        MW(c, ea, v);
        if (dest_reg != 6) SET_R(c, dest_reg, v);
    } else {
        v = cb_set(bit, v);
        MW(c, ea, v);
        if (dest_reg != 6) SET_R(c, dest_reg, v);
    }
    return 23;
}

/* =====================================================================
 * DD / FD prefix handler (IX / IY instructions)
 * ===================================================================== */
static int exec_xy(z80_t *c, uint16_t *xy) {
    uint8_t op = FETCH(c);

    /* Nested prefix — restart with new register */
    if (op == 0xDD) { uint16_t *nxy = &c->IX; return exec_xy(c, nxy); }
    if (op == 0xFD) { uint16_t *nxy = &c->IY; return exec_xy(c, nxy); }
    if (op == 0xCB) return exec_xycb(c, *xy);

    uint8_t xyh = (uint8_t)(*xy >> 8);
    uint8_t xyl = (uint8_t)(*xy);
    int8_t  d;
    uint16_t ea;

    switch (op) {
        /* ADD xy, rr */
        case 0x09: *xy = do_add16(c, *xy, BC(c)); return 15;
        case 0x19: *xy = do_add16(c, *xy, DE(c)); return 15;
        case 0x29: *xy = do_add16(c, *xy, *xy);   return 15;
        case 0x39: *xy = do_add16(c, *xy, c->SP); return 15;

        /* LD xy, nn */
        case 0x21: *xy = FETCH16(c); return 14;

        /* LD (nn), xy */
        case 0x22: ea = FETCH16(c); WM16(c, ea, *xy); return 20;

        /* INC xy */
        case 0x23: (*xy)++; return 10;

        /* INC xyH (undocumented) */
        case 0x24: xyh = do_inc8(c, xyh); *xy = (uint16_t)(xyh << 8) | xyl; return 8;

        /* DEC xyH (undocumented) */
        case 0x25: xyh = do_dec8(c, xyh); *xy = (uint16_t)(xyh << 8) | xyl; return 8;

        /* LD xyH, n (undocumented) */
        case 0x26: xyh = FETCH(c); *xy = (uint16_t)(xyh << 8) | xyl; return 11;

        /* LD xy, (nn) */
        case 0x2A: ea = FETCH16(c); *xy = RM16(c, ea); return 20;

        /* DEC xy */
        case 0x2B: (*xy)--; return 10;

        /* INC xyL (undocumented) */
        case 0x2C: xyl = do_inc8(c, xyl); *xy = (uint16_t)(xyh << 8) | xyl; return 8;

        /* DEC xyL (undocumented) */
        case 0x2D: xyl = do_dec8(c, xyl); *xy = (uint16_t)(xyh << 8) | xyl; return 8;

        /* LD xyL, n (undocumented) */
        case 0x2E: xyl = FETCH(c); *xy = (uint16_t)(xyh << 8) | xyl; return 11;

        /* INC (xy+d) */
        case 0x34: d=(int8_t)FETCH(c); ea=(uint16_t)(*xy+d); MW(c,ea,do_inc8(c,MR(c,ea))); return 23;

        /* DEC (xy+d) */
        case 0x35: d=(int8_t)FETCH(c); ea=(uint16_t)(*xy+d); MW(c,ea,do_dec8(c,MR(c,ea))); return 23;

        /* LD (xy+d), n */
        case 0x36: d=(int8_t)FETCH(c); ea=(uint16_t)(*xy+d); MW(c,ea,FETCH(c)); return 19;

        /* LD B/C/D/E/A, xyH (undocumented) */
        case 0x44: c->B = xyh; return 8;
        case 0x4C: c->C = xyh; return 8;
        case 0x54: c->D = xyh; return 8;
        case 0x5C: c->E = xyh; return 8;
        case 0x7C: c->A = xyh; return 8;

        /* LD B/C/D/E/A, xyL (undocumented) */
        case 0x45: c->B = xyl; return 8;
        case 0x4D: c->C = xyl; return 8;
        case 0x55: c->D = xyl; return 8;
        case 0x5D: c->E = xyl; return 8;
        case 0x7D: c->A = xyl; return 8;

        /* LD xyH, r (undocumented) */
        case 0x60: xyh=c->B; *xy=(uint16_t)(xyh<<8)|xyl; return 8;
        case 0x61: xyh=c->C; *xy=(uint16_t)(xyh<<8)|xyl; return 8;
        case 0x62: xyh=c->D; *xy=(uint16_t)(xyh<<8)|xyl; return 8;
        case 0x63: xyh=c->E; *xy=(uint16_t)(xyh<<8)|xyl; return 8;
        case 0x64: return 8; /* LD xyH, xyH */
        case 0x65: xyh=xyl; *xy=(uint16_t)(xyh<<8)|xyl; return 8;
        case 0x67: xyh=c->A; *xy=(uint16_t)(xyh<<8)|xyl; return 8;

        /* LD xyL, r (undocumented) */
        case 0x68: xyl=c->B; *xy=(uint16_t)(xyh<<8)|xyl; return 8;
        case 0x69: xyl=c->C; *xy=(uint16_t)(xyh<<8)|xyl; return 8;
        case 0x6A: xyl=c->D; *xy=(uint16_t)(xyh<<8)|xyl; return 8;
        case 0x6B: xyl=c->E; *xy=(uint16_t)(xyh<<8)|xyl; return 8;
        case 0x6C: xyl=xyh; *xy=(uint16_t)(xyh<<8)|xyl; return 8;
        case 0x6D: return 8; /* LD xyL, xyL */
        case 0x6F: xyl=c->A; *xy=(uint16_t)(xyh<<8)|xyl; return 8;

        /* LD r, (xy+d) */
        case 0x46: d=(int8_t)FETCH(c); c->B=MR(c,(uint16_t)(*xy+d)); return 19;
        case 0x4E: d=(int8_t)FETCH(c); c->C=MR(c,(uint16_t)(*xy+d)); return 19;
        case 0x56: d=(int8_t)FETCH(c); c->D=MR(c,(uint16_t)(*xy+d)); return 19;
        case 0x5E: d=(int8_t)FETCH(c); c->E=MR(c,(uint16_t)(*xy+d)); return 19;
        case 0x66: d=(int8_t)FETCH(c); c->H=MR(c,(uint16_t)(*xy+d)); return 19;
        case 0x6E: d=(int8_t)FETCH(c); c->L=MR(c,(uint16_t)(*xy+d)); return 19;
        case 0x7E: d=(int8_t)FETCH(c); c->A=MR(c,(uint16_t)(*xy+d)); return 19;

        /* LD (xy+d), r */
        case 0x70: d=(int8_t)FETCH(c); MW(c,(uint16_t)(*xy+d),c->B); return 19;
        case 0x71: d=(int8_t)FETCH(c); MW(c,(uint16_t)(*xy+d),c->C); return 19;
        case 0x72: d=(int8_t)FETCH(c); MW(c,(uint16_t)(*xy+d),c->D); return 19;
        case 0x73: d=(int8_t)FETCH(c); MW(c,(uint16_t)(*xy+d),c->E); return 19;
        case 0x74: d=(int8_t)FETCH(c); MW(c,(uint16_t)(*xy+d),c->H); return 19;
        case 0x75: d=(int8_t)FETCH(c); MW(c,(uint16_t)(*xy+d),c->L); return 19;
        case 0x77: d=(int8_t)FETCH(c); MW(c,(uint16_t)(*xy+d),c->A); return 19;

        /* ALU A, (xy+d) */
        case 0x86: d=(int8_t)FETCH(c); add_a(c,MR(c,(uint16_t)(*xy+d))); return 19;
        case 0x8E: d=(int8_t)FETCH(c); adc_a(c,MR(c,(uint16_t)(*xy+d))); return 19;
        case 0x96: d=(int8_t)FETCH(c); sub_a(c,MR(c,(uint16_t)(*xy+d))); return 19;
        case 0x9E: d=(int8_t)FETCH(c); sbc_a(c,MR(c,(uint16_t)(*xy+d))); return 19;
        case 0xA6: d=(int8_t)FETCH(c); and_a(c,MR(c,(uint16_t)(*xy+d))); return 19;
        case 0xAE: d=(int8_t)FETCH(c); xor_a(c,MR(c,(uint16_t)(*xy+d))); return 19;
        case 0xB6: d=(int8_t)FETCH(c); or_a (c,MR(c,(uint16_t)(*xy+d))); return 19;
        case 0xBE: d=(int8_t)FETCH(c); cp_a (c,MR(c,(uint16_t)(*xy+d))); return 19;

        /* ALU A, xyH/xyL (undocumented) */
        case 0x84: add_a(c, xyh); return 8;
        case 0x85: add_a(c, xyl); return 8;
        case 0x8C: adc_a(c, xyh); return 8;
        case 0x8D: adc_a(c, xyl); return 8;
        case 0x94: sub_a(c, xyh); return 8;
        case 0x95: sub_a(c, xyl); return 8;
        case 0x9C: sbc_a(c, xyh); return 8;
        case 0x9D: sbc_a(c, xyl); return 8;
        case 0xA4: and_a(c, xyh); return 8;
        case 0xA5: and_a(c, xyl); return 8;
        case 0xAC: xor_a(c, xyh); return 8;
        case 0xAD: xor_a(c, xyl); return 8;
        case 0xB4: or_a (c, xyh); return 8;
        case 0xB5: or_a (c, xyl); return 8;
        case 0xBC: cp_a (c, xyh); return 8;
        case 0xBD: cp_a (c, xyl); return 8;

        /* POP / PUSH xy */
        case 0xE1: *xy = POP(c); return 14;
        case 0xE5: PUSH(c, *xy); return 15;

        /* EX (SP), xy */
        case 0xE3: {
            uint16_t tmp = RM16(c, c->SP);
            WM16(c, c->SP, *xy);
            *xy = tmp;
            return 23;
        }

        /* JP (xy) */
        case 0xE9: c->PC = *xy; return 8;

        /* LD SP, xy */
        case 0xF9: c->SP = *xy; return 10;

        default:
            /* Unknown DD/FD opcode - treat as NOP (next byte reprocessed by main) */
            c->PC--; /* un-consume */
            return 4;
    }
}

/* =====================================================================
 * ED prefix handler (extended instructions)
 * ===================================================================== */
static int exec_ed(z80_t *c) {
    uint8_t  op = FETCH(c);
    uint16_t ea;
    uint8_t  tmp;

    switch (op) {
        /* IN r, (C) */
        case 0x40: c->B = IOR(c, BC(c)); c->F=(c->F&FC)|(c->B?0:FZ)|(c->B&FS)|(s_parity[c->B]?FPV:0)|FH|(c->B&(FX|FY)); return 12;
        case 0x48: c->C = IOR(c, BC(c)); c->F=(c->F&FC)|(c->C?0:FZ)|(c->C&FS)|(s_parity[c->C]?FPV:0)|FH|(c->C&(FX|FY)); return 12;
        case 0x50: c->D = IOR(c, BC(c)); c->F=(c->F&FC)|(c->D?0:FZ)|(c->D&FS)|(s_parity[c->D]?FPV:0)|FH|(c->D&(FX|FY)); return 12;
        case 0x58: c->E = IOR(c, BC(c)); c->F=(c->F&FC)|(c->E?0:FZ)|(c->E&FS)|(s_parity[c->E]?FPV:0)|FH|(c->E&(FX|FY)); return 12;
        case 0x60: c->H = IOR(c, BC(c)); c->F=(c->F&FC)|(c->H?0:FZ)|(c->H&FS)|(s_parity[c->H]?FPV:0)|FH|(c->H&(FX|FY)); return 12;
        case 0x68: c->L = IOR(c, BC(c)); c->F=(c->F&FC)|(c->L?0:FZ)|(c->L&FS)|(s_parity[c->L]?FPV:0)|FH|(c->L&(FX|FY)); return 12;
        case 0x70: /* IN F,(C) - flags only */ tmp=IOR(c,BC(c)); c->F=(c->F&FC)|(tmp?0:FZ)|(tmp&FS)|(s_parity[tmp]?FPV:0)|FH|(tmp&(FX|FY)); return 12;
        case 0x78: c->A = IOR(c, BC(c)); c->F=(c->F&FC)|(c->A?0:FZ)|(c->A&FS)|(s_parity[c->A]?FPV:0)|FH|(c->A&(FX|FY)); return 12;

        /* OUT (C), r */
        case 0x41: IOW(c, BC(c), c->B); return 12;
        case 0x49: IOW(c, BC(c), c->C); return 12;
        case 0x51: IOW(c, BC(c), c->D); return 12;
        case 0x59: IOW(c, BC(c), c->E); return 12;
        case 0x61: IOW(c, BC(c), c->H); return 12;
        case 0x69: IOW(c, BC(c), c->L); return 12;
        case 0x71: IOW(c, BC(c), 0);    return 12;  /* undocumented: OUT (C),0 */
        case 0x79: IOW(c, BC(c), c->A); return 12;

        /* SBC HL, rr */
        case 0x42: SET_HL(c, do_sbc16(c, HL(c), BC(c))); return 15;
        case 0x52: SET_HL(c, do_sbc16(c, HL(c), DE(c))); return 15;
        case 0x62: SET_HL(c, do_sbc16(c, HL(c), HL(c))); return 15;
        case 0x72: SET_HL(c, do_sbc16(c, HL(c), c->SP)); return 15;

        /* ADC HL, rr */
        case 0x4A: SET_HL(c, do_adc16(c, HL(c), BC(c))); return 15;
        case 0x5A: SET_HL(c, do_adc16(c, HL(c), DE(c))); return 15;
        case 0x6A: SET_HL(c, do_adc16(c, HL(c), HL(c))); return 15;
        case 0x7A: SET_HL(c, do_adc16(c, HL(c), c->SP)); return 15;

        /* LD (nn), rr */
        case 0x43: ea=FETCH16(c); WM16(c,ea,BC(c)); return 20;
        case 0x53: ea=FETCH16(c); WM16(c,ea,DE(c)); return 20;
        case 0x63: ea=FETCH16(c); WM16(c,ea,HL(c)); return 20;
        case 0x73: ea=FETCH16(c); WM16(c,ea,c->SP); return 20;

        /* LD rr, (nn) */
        case 0x4B: ea=FETCH16(c); SET_BC(c,RM16(c,ea)); return 20;
        case 0x5B: ea=FETCH16(c); SET_DE(c,RM16(c,ea)); return 20;
        case 0x6B: ea=FETCH16(c); SET_HL(c,RM16(c,ea)); return 20;
        case 0x7B: ea=FETCH16(c); c->SP=RM16(c,ea);     return 20;

        /* NEG: A = 0 - A */
        case 0x44: case 0x4C: case 0x54: case 0x5C:
        case 0x64: case 0x6C: case 0x74: case 0x7C: {
            uint8_t old = c->A;
            c->A = 0;
            c->F = fl_sub8(0, old, 0);
            return 8;
        }

        /* RETN */
        case 0x45: case 0x55: case 0x65: case 0x75:
        case 0x4D: /* also RETI */
        case 0x5D: case 0x6D: case 0x7D:
            c->IFF1 = c->IFF2;
            c->PC   = POP(c);
            return 14;

        /* IM 0/1/2 */
        case 0x46: case 0x4E: case 0x66: case 0x6E: c->IM = 0; return 8;
        case 0x56: case 0x76:                        c->IM = 1; return 8;
        case 0x5E: case 0x7E:                        c->IM = 2; return 8;

        /* LD I,A / LD R,A */
        case 0x47: c->I = c->A; return 9;
        case 0x4F: c->R = c->A; return 9;

        /* LD A,I / LD A,R */
        case 0x57: {
            c->A = c->I;
            uint8_t f = (c->F & FC) | (c->A ? 0 : FZ) | (c->A & FS) | (c->IFF2 ? FPV : 0) | (c->A & (FX|FY));
            c->F = f;
            return 9;
        }
        case 0x5F: {
            c->A = c->R;
            uint8_t f = (c->F & FC) | (c->A ? 0 : FZ) | (c->A & FS) | (c->IFF2 ? FPV : 0) | (c->A & (FX|FY));
            c->F = f;
            return 9;
        }

        /* RRD */
        case 0x67: {
            uint16_t hl = HL(c);
            uint8_t mem = MR(c, hl);
            uint8_t new_mem = (c->A << 4) | (mem >> 4);
            c->A = (c->A & 0xF0) | (mem & 0x0F);
            MW(c, hl, new_mem);
            c->F = (c->F & FC) | (c->A ? 0 : FZ) | (c->A & FS) | (s_parity[c->A] ? FPV : 0) | (c->A & (FX|FY));
            return 18;
        }

        /* RLD */
        case 0x6F: {
            uint16_t hl = HL(c);
            uint8_t mem = MR(c, hl);
            uint8_t new_mem = (mem << 4) | (c->A & 0x0F);
            c->A = (c->A & 0xF0) | (mem >> 4);
            MW(c, hl, new_mem);
            c->F = (c->F & FC) | (c->A ? 0 : FZ) | (c->A & FS) | (s_parity[c->A] ? FPV : 0) | (c->A & (FX|FY));
            return 18;
        }

        /* LDI: (DE++) = (HL++); BC-- ; set/clear PV */
        case 0xA0: {
            uint8_t val = MR(c, HL(c));
            MW(c, DE(c), val);
            SET_HL(c, HL(c) + 1);
            SET_DE(c, DE(c) + 1);
            SET_BC(c, BC(c) - 1);
            uint8_t n = c->A + val;
            c->F = (c->F & (FS|FZ|FC)) | (BC(c) ? FPV : 0) | (n & FX) | ((n & 2) ? FY : 0);
            return 16;
        }

        /* CPI: compare (HL++) with A; BC-- */
        case 0xA1: {
            uint8_t val = MR(c, HL(c));
            SET_HL(c, HL(c) + 1);
            SET_BC(c, BC(c) - 1);
            uint8_t r = c->A - val;
            uint8_t half = ((c->A ^ val ^ r) & FH) ? FH : 0;
            uint8_t n = r - (half ? 1 : 0);
            c->F = (c->F & FC) | FN | half | (r ? 0 : FZ) | (r & FS) | (BC(c) ? FPV : 0) | (n & FX) | ((n & 2) ? FY : 0);
            return 16;
        }

        /* INI: (HL++) = IN(BC); B-- */
        case 0xA2: {
            uint8_t v = IOR(c, BC(c));
            MW(c, HL(c), v);
            SET_HL(c, HL(c) + 1);
            c->B = do_dec8(c, c->B);
            return 16;
        }

        /* OUTI: OUT(BC) = (HL++); B-- */
        case 0xA3: {
            uint8_t v = MR(c, HL(c));
            SET_HL(c, HL(c) + 1);
            c->B = do_dec8(c, c->B);
            IOW(c, BC(c), v);
            return 16;
        }

        /* LDD: (DE--) = (HL--); BC-- */
        case 0xA8: {
            uint8_t val = MR(c, HL(c));
            MW(c, DE(c), val);
            SET_HL(c, HL(c) - 1);
            SET_DE(c, DE(c) - 1);
            SET_BC(c, BC(c) - 1);
            uint8_t n = c->A + val;
            c->F = (c->F & (FS|FZ|FC)) | (BC(c) ? FPV : 0) | (n & FX) | ((n & 2) ? FY : 0);
            return 16;
        }

        /* CPD: compare (HL--) with A; BC-- */
        case 0xA9: {
            uint8_t val = MR(c, HL(c));
            SET_HL(c, HL(c) - 1);
            SET_BC(c, BC(c) - 1);
            uint8_t r = c->A - val;
            uint8_t half = ((c->A ^ val ^ r) & FH) ? FH : 0;
            uint8_t n = r - (half ? 1 : 0);
            c->F = (c->F & FC) | FN | half | (r ? 0 : FZ) | (r & FS) | (BC(c) ? FPV : 0) | (n & FX) | ((n & 2) ? FY : 0);
            return 16;
        }

        /* IND: (HL--) = IN(BC); B-- */
        case 0xAA: {
            uint8_t v = IOR(c, BC(c));
            MW(c, HL(c), v);
            SET_HL(c, HL(c) - 1);
            c->B = do_dec8(c, c->B);
            return 16;
        }

        /* OUTD: OUT(BC) = (HL--); B-- */
        case 0xAB: {
            uint8_t v = MR(c, HL(c));
            SET_HL(c, HL(c) - 1);
            c->B = do_dec8(c, c->B);
            IOW(c, BC(c), v);
            return 16;
        }

        /* LDIR: repeat LDI until BC == 0 */
        case 0xB0: {
            uint8_t val = MR(c, HL(c));
            MW(c, DE(c), val);
            SET_HL(c, HL(c) + 1);
            SET_DE(c, DE(c) + 1);
            SET_BC(c, BC(c) - 1);
            uint8_t n = c->A + val;
            if (BC(c) != 0) {
                c->PC -= 2;  /* repeat */
                c->F = (c->F & (FS|FZ|FC)) | FPV | (n & FX) | ((n & 2) ? FY : 0);
                return 21;
            }
            c->F = (c->F & (FS|FZ|FC)) | (n & FX) | ((n & 2) ? FY : 0);
            return 16;
        }

        /* CPIR: repeat CPI until BC==0 or match */
        case 0xB1: {
            uint8_t val = MR(c, HL(c));
            SET_HL(c, HL(c) + 1);
            SET_BC(c, BC(c) - 1);
            uint8_t r = c->A - val;
            uint8_t half = ((c->A ^ val ^ r) & FH) ? FH : 0;
            uint8_t n = r - (half ? 1 : 0);
            uint8_t zf = (r == 0) ? FZ : 0;
            c->F = (c->F & FC) | FN | half | zf | (r & FS) | (BC(c) ? FPV : 0) | (n & FX) | ((n & 2) ? FY : 0);
            if (BC(c) != 0 && !zf) {
                c->PC -= 2;
                return 21;
            }
            return 16;
        }

        /* INIR: repeat INI until B == 0 */
        case 0xB2: {
            uint8_t v = IOR(c, BC(c));
            MW(c, HL(c), v);
            SET_HL(c, HL(c) + 1);
            c->B = do_dec8(c, c->B);
            if (c->B != 0) { c->PC -= 2; return 21; }
            return 16;
        }

        /* OTIR: repeat OUTI until B == 0 */
        case 0xB3: {
            uint8_t v = MR(c, HL(c));
            SET_HL(c, HL(c) + 1);
            c->B = do_dec8(c, c->B);
            IOW(c, BC(c), v);
            if (c->B != 0) { c->PC -= 2; return 21; }
            return 16;
        }

        /* LDDR: repeat LDD until BC == 0 */
        case 0xB8: {
            uint8_t val = MR(c, HL(c));
            MW(c, DE(c), val);
            SET_HL(c, HL(c) - 1);
            SET_DE(c, DE(c) - 1);
            SET_BC(c, BC(c) - 1);
            uint8_t n = c->A + val;
            if (BC(c) != 0) {
                c->PC -= 2;
                c->F = (c->F & (FS|FZ|FC)) | FPV | (n & FX) | ((n & 2) ? FY : 0);
                return 21;
            }
            c->F = (c->F & (FS|FZ|FC)) | (n & FX) | ((n & 2) ? FY : 0);
            return 16;
        }

        /* CPDR: repeat CPD until BC==0 or match */
        case 0xB9: {
            uint8_t val = MR(c, HL(c));
            SET_HL(c, HL(c) - 1);
            SET_BC(c, BC(c) - 1);
            uint8_t r = c->A - val;
            uint8_t half = ((c->A ^ val ^ r) & FH) ? FH : 0;
            uint8_t n = r - (half ? 1 : 0);
            uint8_t zf = (r == 0) ? FZ : 0;
            c->F = (c->F & FC) | FN | half | zf | (r & FS) | (BC(c) ? FPV : 0) | (n & FX) | ((n & 2) ? FY : 0);
            if (BC(c) != 0 && !zf) {
                c->PC -= 2;
                return 21;
            }
            return 16;
        }

        /* INDR: repeat IND until B == 0 */
        case 0xBA: {
            uint8_t v = IOR(c, BC(c));
            MW(c, HL(c), v);
            SET_HL(c, HL(c) - 1);
            c->B = do_dec8(c, c->B);
            if (c->B != 0) { c->PC -= 2; return 21; }
            return 16;
        }

        /* OTDR: repeat OUTD until B == 0 */
        case 0xBB: {
            uint8_t v = MR(c, HL(c));
            SET_HL(c, HL(c) - 1);
            c->B = do_dec8(c, c->B);
            IOW(c, BC(c), v);
            if (c->B != 0) { c->PC -= 2; return 21; }
            return 16;
        }

        default:
            /* Ignore unknown ED opcodes (treated as NOP) */
            return 8;
    }
}

/* =====================================================================
 * Main instruction dispatcher
 * ===================================================================== */
int z80_step(z80_t *c) {
    /* --- Handle trap (BDOS / warm-boot) --- */
    if (c->trap_cb && (c->PC == c->trap_addr)) {
        if (c->trap_cb(c->user_data, c)) {
            /* Trap handled: simulate RET */
            c->PC = POP(c);
            c->total_cycles += 10;
            return 10;
        }
    }

    /* --- NMI --- */
    if (c->nmi_pending) {
        c->nmi_pending = false;
        c->IFF2 = c->IFF1;
        c->IFF1 = false;
        c->halted = false;
        PUSH(c, c->PC);
        c->PC = 0x0066;
        c->total_cycles += 11;
        return 11;
    }

    /* --- Maskable IRQ --- */
    if (c->irq_pending && c->IFF1 && !c->ei_pending) {
        c->IFF1 = c->IFF2 = false;
        c->halted = false;
        switch (c->IM) {
            case 0:
                /* IM0: execute instruction on data bus (usually RST n) */
                /* For simplicity, treat as RST (vector & 0x38) */
                PUSH(c, c->PC);
                c->PC = c->irq_vector & 0x38;
                c->total_cycles += 13;
                return 13;
            case 1:
                PUSH(c, c->PC);
                c->PC = 0x0038;
                c->total_cycles += 13;
                return 13;
            case 2: {
                uint16_t vec_addr = (uint16_t)((c->I << 8) | c->irq_vector);
                PUSH(c, c->PC);
                c->PC = RM16(c, vec_addr);
                c->total_cycles += 19;
                return 19;
            }
        }
    }

    /* Clear EI delay */
    if (c->ei_pending) c->ei_pending = false;

    /* --- HALT: spin in place --- */
    if (c->halted) {
        c->total_cycles += 4;
        return 4;
    }

    /* --- Fetch and decode --- */
    uint8_t op = FETCH(c);
    int cycles;
    uint16_t ea;
    uint8_t  tmp;

    switch (op) {

    /* ----------------------------------------------------------------
     * 0x00-0x3F: misc, loads, rotates
     * ---------------------------------------------------------------- */
    case 0x00: cycles = 4;  break; /* NOP */
    case 0x01: SET_BC(c, FETCH16(c)); cycles = 10; break; /* LD BC,nn */
    case 0x02: MW(c, BC(c), c->A);   cycles = 7;  break; /* LD (BC),A */
    case 0x03: SET_BC(c, BC(c) + 1); cycles = 6;  break; /* INC BC */
    case 0x04: c->B = do_inc8(c, c->B); cycles = 4; break; /* INC B */
    case 0x05: c->B = do_dec8(c, c->B); cycles = 4; break; /* DEC B */
    case 0x06: c->B = FETCH(c); cycles = 7; break; /* LD B,n */
    case 0x07: do_rlca(c); cycles = 4; break; /* RLCA */

    case 0x08: { /* EX AF,AF' */
        uint8_t ta=c->A, tf=c->F;
        c->A=c->A2; c->F=c->F2; c->A2=ta; c->F2=tf;
        cycles = 4; break;
    }
    case 0x09: SET_HL(c, do_add16(c, HL(c), BC(c))); cycles = 11; break; /* ADD HL,BC */
    case 0x0A: c->A = MR(c, BC(c)); cycles = 7; break; /* LD A,(BC) */
    case 0x0B: SET_BC(c, BC(c) - 1); cycles = 6; break; /* DEC BC */
    case 0x0C: c->C = do_inc8(c, c->C); cycles = 4; break; /* INC C */
    case 0x0D: c->C = do_dec8(c, c->C); cycles = 4; break; /* DEC C */
    case 0x0E: c->C = FETCH(c); cycles = 7; break; /* LD C,n */
    case 0x0F: do_rrca(c); cycles = 4; break; /* RRCA */

    case 0x10: { /* DJNZ e */
        int8_t e = (int8_t)FETCH(c);
        c->B--;
        if (c->B) { c->PC = (uint16_t)(c->PC + e); cycles = 13; }
        else       { cycles = 8; }
        break;
    }
    case 0x11: SET_DE(c, FETCH16(c)); cycles = 10; break; /* LD DE,nn */
    case 0x12: MW(c, DE(c), c->A); cycles = 7; break; /* LD (DE),A */
    case 0x13: SET_DE(c, DE(c) + 1); cycles = 6; break; /* INC DE */
    case 0x14: c->D = do_inc8(c, c->D); cycles = 4; break; /* INC D */
    case 0x15: c->D = do_dec8(c, c->D); cycles = 4; break; /* DEC D */
    case 0x16: c->D = FETCH(c); cycles = 7; break; /* LD D,n */
    case 0x17: do_rla(c); cycles = 4; break; /* RLA */

    case 0x18: { int8_t e=(int8_t)FETCH(c); c->PC=(uint16_t)(c->PC+e); cycles=12; break; } /* JR e */
    case 0x19: SET_HL(c, do_add16(c, HL(c), DE(c))); cycles = 11; break; /* ADD HL,DE */
    case 0x1A: c->A = MR(c, DE(c)); cycles = 7; break; /* LD A,(DE) */
    case 0x1B: SET_DE(c, DE(c) - 1); cycles = 6; break; /* DEC DE */
    case 0x1C: c->E = do_inc8(c, c->E); cycles = 4; break; /* INC E */
    case 0x1D: c->E = do_dec8(c, c->E); cycles = 4; break; /* DEC E */
    case 0x1E: c->E = FETCH(c); cycles = 7; break; /* LD E,n */
    case 0x1F: do_rra(c); cycles = 4; break; /* RRA */

    /* JR cc, e */
    case 0x20: { int8_t e=(int8_t)FETCH(c); if (!(c->F&FZ)) { c->PC=(uint16_t)(c->PC+e); cycles=12; } else cycles=7; break; } /* JR NZ */
    case 0x21: SET_HL(c, FETCH16(c)); cycles = 10; break; /* LD HL,nn */
    case 0x22: ea=FETCH16(c); WM16(c,ea,HL(c)); cycles=16; break; /* LD (nn),HL */
    case 0x23: SET_HL(c, HL(c) + 1); cycles = 6; break; /* INC HL */
    case 0x24: c->H = do_inc8(c, c->H); cycles = 4; break; /* INC H */
    case 0x25: c->H = do_dec8(c, c->H); cycles = 4; break; /* DEC H */
    case 0x26: c->H = FETCH(c); cycles = 7; break; /* LD H,n */
    case 0x27: do_daa(c); cycles = 4; break; /* DAA */

    case 0x28: { int8_t e=(int8_t)FETCH(c); if (c->F&FZ) { c->PC=(uint16_t)(c->PC+e); cycles=12; } else cycles=7; break; } /* JR Z */
    case 0x29: SET_HL(c, do_add16(c, HL(c), HL(c))); cycles = 11; break; /* ADD HL,HL */
    case 0x2A: ea=FETCH16(c); SET_HL(c,RM16(c,ea)); cycles=16; break; /* LD HL,(nn) */
    case 0x2B: SET_HL(c, HL(c) - 1); cycles = 6; break; /* DEC HL */
    case 0x2C: c->L = do_inc8(c, c->L); cycles = 4; break; /* INC L */
    case 0x2D: c->L = do_dec8(c, c->L); cycles = 4; break; /* DEC L */
    case 0x2E: c->L = FETCH(c); cycles = 7; break; /* LD L,n */
    case 0x2F: do_cpl(c); cycles = 4; break; /* CPL */

    case 0x30: { int8_t e=(int8_t)FETCH(c); if (!(c->F&FC)) { c->PC=(uint16_t)(c->PC+e); cycles=12; } else cycles=7; break; } /* JR NC */
    case 0x31: c->SP = FETCH16(c); cycles = 10; break; /* LD SP,nn */
    case 0x32: ea=FETCH16(c); MW(c,ea,c->A); cycles=13; break; /* LD (nn),A */
    case 0x33: c->SP++; cycles = 6; break; /* INC SP */
    case 0x34: { uint16_t hl=HL(c); MW(c,hl,do_inc8(c,MR(c,hl))); cycles=11; break; } /* INC (HL) */
    case 0x35: { uint16_t hl=HL(c); MW(c,hl,do_dec8(c,MR(c,hl))); cycles=11; break; } /* DEC (HL) */
    case 0x36: { uint16_t hl=HL(c); MW(c,hl,FETCH(c)); cycles=10; break; } /* LD (HL),n */
    case 0x37: do_scf(c); cycles = 4; break; /* SCF */

    case 0x38: { int8_t e=(int8_t)FETCH(c); if (c->F&FC) { c->PC=(uint16_t)(c->PC+e); cycles=12; } else cycles=7; break; } /* JR C */
    case 0x39: SET_HL(c, do_add16(c, HL(c), c->SP)); cycles = 11; break; /* ADD HL,SP */
    case 0x3A: ea=FETCH16(c); c->A=MR(c,ea); cycles=13; break; /* LD A,(nn) */
    case 0x3B: c->SP--; cycles = 6; break; /* DEC SP */
    case 0x3C: c->A = do_inc8(c, c->A); cycles = 4; break; /* INC A */
    case 0x3D: c->A = do_dec8(c, c->A); cycles = 4; break; /* DEC A */
    case 0x3E: c->A = FETCH(c); cycles = 7; break; /* LD A,n */
    case 0x3F: do_ccf(c); cycles = 4; break; /* CCF */

    /* ----------------------------------------------------------------
     * 0x40-0x7F: LD r, r'  (0x76 = HALT)
     * ---------------------------------------------------------------- */
    case 0x76: c->halted = true; cycles = 4; break; /* HALT */

    default:
        if (op >= 0x40 && op <= 0x7F) {
            /* LD r, r' */
            int dst = (op >> 3) & 7;
            int src = op & 7;
            uint8_t v = GET_R(c, src);
            SET_R(c, dst, v);
            cycles = (src == 6 || dst == 6) ? 7 : 4;
            break;
        }

    /* ----------------------------------------------------------------
     * 0x80-0xBF: ALU operations
     * ---------------------------------------------------------------- */
        if (op >= 0x80 && op <= 0xBF) {
            int  alu = (op >> 3) & 7;
            int  reg = op & 7;
            uint8_t v = GET_R(c, reg);
            cycles = (reg == 6) ? 7 : 4;
            switch (alu) {
                case 0: add_a(c, v); break; /* ADD A,r */
                case 1: adc_a(c, v); break; /* ADC A,r */
                case 2: sub_a(c, v); break; /* SUB r */
                case 3: sbc_a(c, v); break; /* SBC A,r */
                case 4: and_a(c, v); break; /* AND r */
                case 5: xor_a(c, v); break; /* XOR r */
                case 6: or_a (c, v); break; /* OR r */
                case 7: cp_a (c, v); break; /* CP r */
            }
            break;
        }

    /* ----------------------------------------------------------------
     * 0xC0-0xFF: control flow, stack, I/O, prefixes
     * ---------------------------------------------------------------- */

    /* RET cc */
    case 0xC0: if (!(c->F&FZ)) { c->PC=POP(c); cycles=11; } else cycles=5; break; /* RET NZ */
    case 0xC8: if (  c->F&FZ ) { c->PC=POP(c); cycles=11; } else cycles=5; break; /* RET Z  */
    case 0xD0: if (!(c->F&FC)) { c->PC=POP(c); cycles=11; } else cycles=5; break; /* RET NC */
    case 0xD8: if (  c->F&FC ) { c->PC=POP(c); cycles=11; } else cycles=5; break; /* RET C  */
    case 0xE0: if (!(c->F&FPV)){ c->PC=POP(c); cycles=11; } else cycles=5; break; /* RET PO */
    case 0xE8: if (  c->F&FPV ){ c->PC=POP(c); cycles=11; } else cycles=5; break; /* RET PE */
    case 0xF0: if (!(c->F&FS)) { c->PC=POP(c); cycles=11; } else cycles=5; break; /* RET P  */
    case 0xF8: if (  c->F&FS ) { c->PC=POP(c); cycles=11; } else cycles=5; break; /* RET M  */

    /* POP rr */
    case 0xC1: SET_BC(c, POP(c)); cycles = 10; break;
    case 0xD1: SET_DE(c, POP(c)); cycles = 10; break;
    case 0xE1: SET_HL(c, POP(c)); cycles = 10; break;
    case 0xF1: { uint16_t v=POP(c); c->A=v>>8; c->F=v&0xFF; cycles=10; break; } /* POP AF */

    /* PUSH rr */
    case 0xC5: PUSH(c, BC(c)); cycles = 11; break;
    case 0xD5: PUSH(c, DE(c)); cycles = 11; break;
    case 0xE5: PUSH(c, HL(c)); cycles = 11; break;
    case 0xF5: PUSH(c, AF(c)); cycles = 11; break;

    /* JP cc, nn */
    case 0xC2: ea=FETCH16(c); if (!(c->F&FZ)) c->PC=ea; cycles=10; break; /* JP NZ */
    case 0xCA: ea=FETCH16(c); if (  c->F&FZ ) c->PC=ea; cycles=10; break; /* JP Z  */
    case 0xD2: ea=FETCH16(c); if (!(c->F&FC)) c->PC=ea; cycles=10; break; /* JP NC */
    case 0xDA: ea=FETCH16(c); if (  c->F&FC ) c->PC=ea; cycles=10; break; /* JP C  */
    case 0xE2: ea=FETCH16(c); if (!(c->F&FPV))c->PC=ea; cycles=10; break; /* JP PO */
    case 0xEA: ea=FETCH16(c); if (  c->F&FPV) c->PC=ea; cycles=10; break; /* JP PE */
    case 0xF2: ea=FETCH16(c); if (!(c->F&FS)) c->PC=ea; cycles=10; break; /* JP P  */
    case 0xFA: ea=FETCH16(c); if (  c->F&FS ) c->PC=ea; cycles=10; break; /* JP M  */

    /* JP nn */
    case 0xC3: c->PC = FETCH16(c); cycles = 10; break;

    /* CALL cc, nn */
    case 0xC4: ea=FETCH16(c); if (!(c->F&FZ)) { PUSH(c,c->PC); c->PC=ea; cycles=17; } else cycles=10; break; /* CALL NZ */
    case 0xCC: ea=FETCH16(c); if (  c->F&FZ ) { PUSH(c,c->PC); c->PC=ea; cycles=17; } else cycles=10; break; /* CALL Z  */
    case 0xD4: ea=FETCH16(c); if (!(c->F&FC)) { PUSH(c,c->PC); c->PC=ea; cycles=17; } else cycles=10; break; /* CALL NC */
    case 0xDC: ea=FETCH16(c); if (  c->F&FC ) { PUSH(c,c->PC); c->PC=ea; cycles=17; } else cycles=10; break; /* CALL C  */
    case 0xE4: ea=FETCH16(c); if (!(c->F&FPV)){ PUSH(c,c->PC); c->PC=ea; cycles=17; } else cycles=10; break; /* CALL PO */
    case 0xEC: ea=FETCH16(c); if (  c->F&FPV) { PUSH(c,c->PC); c->PC=ea; cycles=17; } else cycles=10; break; /* CALL PE */
    case 0xF4: ea=FETCH16(c); if (!(c->F&FS)) { PUSH(c,c->PC); c->PC=ea; cycles=17; } else cycles=10; break; /* CALL P  */
    case 0xFC: ea=FETCH16(c); if (  c->F&FS ) { PUSH(c,c->PC); c->PC=ea; cycles=17; } else cycles=10; break; /* CALL M  */

    /* CALL nn */
    case 0xCD: ea=FETCH16(c); PUSH(c, c->PC); c->PC=ea; cycles=17; break;

    /* RET */
    case 0xC9: c->PC = POP(c); cycles = 10; break;

    /* ALU A, n */
    case 0xC6: add_a(c, FETCH(c)); cycles = 7; break; /* ADD A,n */
    case 0xCE: adc_a(c, FETCH(c)); cycles = 7; break; /* ADC A,n */
    case 0xD6: sub_a(c, FETCH(c)); cycles = 7; break; /* SUB n */
    case 0xDE: sbc_a(c, FETCH(c)); cycles = 7; break; /* SBC A,n */
    case 0xE6: and_a(c, FETCH(c)); cycles = 7; break; /* AND n */
    case 0xEE: xor_a(c, FETCH(c)); cycles = 7; break; /* XOR n */
    case 0xF6: or_a (c, FETCH(c)); cycles = 7; break; /* OR n */
    case 0xFE: cp_a (c, FETCH(c)); cycles = 7; break; /* CP n */

    /* RST n */
    case 0xC7: PUSH(c,c->PC); c->PC=0x00; cycles=11; break;
    case 0xCF: PUSH(c,c->PC); c->PC=0x08; cycles=11; break;
    case 0xD7: PUSH(c,c->PC); c->PC=0x10; cycles=11; break;
    case 0xDF: PUSH(c,c->PC); c->PC=0x18; cycles=11; break;
    case 0xE7: PUSH(c,c->PC); c->PC=0x20; cycles=11; break;
    case 0xEF: PUSH(c,c->PC); c->PC=0x28; cycles=11; break;
    case 0xF7: PUSH(c,c->PC); c->PC=0x30; cycles=11; break;
    case 0xFF: PUSH(c,c->PC); c->PC=0x38; cycles=11; break;

    /* OUT (n), A */
    case 0xD3: { uint8_t port=FETCH(c); IOW(c,(uint16_t)((c->A<<8)|port),c->A); cycles=11; break; }

    /* IN A, (n) */
    case 0xDB: { uint8_t port=FETCH(c); c->A=IOR(c,(uint16_t)((c->A<<8)|port)); cycles=11; break; }

    /* EXX */
    case 0xD9: {
        uint8_t tb=c->B,tc=c->C,td=c->D,te=c->E,th=c->H,tl=c->L;
        c->B=c->B2; c->C=c->C2; c->D=c->D2; c->E=c->E2; c->H=c->H2; c->L=c->L2;
        c->B2=tb; c->C2=tc; c->D2=td; c->E2=te; c->H2=th; c->L2=tl;
        cycles=4; break;
    }

    /* EX (SP), HL */
    case 0xE3: {
        uint16_t t = RM16(c, c->SP);
        WM16(c, c->SP, HL(c));
        SET_HL(c, t);
        cycles = 19; break;
    }

    /* JP (HL) */
    case 0xE9: c->PC = HL(c); cycles = 4; break;

    /* EX DE, HL */
    case 0xEB: {
        uint8_t td=c->D,te=c->E,th=c->H,tl=c->L;
        c->D=c->H; c->E=c->L; c->H=td; c->L=te;
        (void)th; (void)tl;
        cycles = 4; break;
    }

    /* DI / EI */
    case 0xF3: c->IFF1 = c->IFF2 = false; cycles = 4; break;
    case 0xFB: c->IFF1 = c->IFF2 = true; c->ei_pending = true; cycles = 4; break;

    /* LD SP, HL */
    case 0xF9: c->SP = HL(c); cycles = 6; break;

    /* Prefix bytes */
    case 0xCB: cycles = exec_cb(c); break;
    case 0xDD: cycles = exec_xy(c, &c->IX); break;
    case 0xED: cycles = exec_ed(c); break;
    case 0xFD: cycles = exec_xy(c, &c->IY); break;

    /* Unknown/illegal opcodes — treat as NOP */
        cycles = 4;
        break;
    } /* end switch(op) */

    c->total_cycles += (uint64_t)cycles;
    return cycles;
}

/* =====================================================================
 * Public API
 * ===================================================================== */

void z80_init(z80_t *cpu) {
    memset(cpu, 0, sizeof(z80_t));
    cpu->trap_addr = 0x0005; /* default CP/M BDOS trap */
}

void z80_reset(z80_t *cpu) {
    cpu->PC   = 0x0000;
    cpu->SP   = 0xFFFF;
    cpu->IFF1 = false;
    cpu->IFF2 = false;
    cpu->IM   = 0;
    cpu->I    = 0;
    cpu->R    = 0;
    cpu->halted      = false;
    cpu->irq_pending = false;
    cpu->nmi_pending = false;
    cpu->ei_pending  = false;
    cpu->A = cpu->F  = 0xFF;
    cpu->B = cpu->C  = 0xFF;
    cpu->D = cpu->E  = 0xFF;
    cpu->H = cpu->L  = 0xFF;
    cpu->IX = cpu->IY = 0xFFFF;
}

void z80_set_irq(z80_t *cpu, bool level, uint8_t vector) {
    cpu->irq_pending = level;
    cpu->irq_vector  = vector;
}

void z80_set_nmi(z80_t *cpu) {
    cpu->nmi_pending = true;
}

void z80_dump_state(z80_t *cpu) {
    printf("Z80 State:\n");
    printf("  AF=%04X  BC=%04X  DE=%04X  HL=%04X\n",
           AF(cpu), BC(cpu), DE(cpu), HL(cpu));
    printf("  AF'=%04X BC'=%04X DE'=%04X HL'=%04X\n",
           (uint16_t)((cpu->A2<<8)|cpu->F2),
           (uint16_t)((cpu->B2<<8)|cpu->C2),
           (uint16_t)((cpu->D2<<8)|cpu->E2),
           (uint16_t)((cpu->H2<<8)|cpu->L2));
    printf("  IX=%04X  IY=%04X  SP=%04X  PC=%04X\n",
           cpu->IX, cpu->IY, cpu->SP, cpu->PC);
    printf("  I=%02X  R=%02X  IM=%d  IFF1=%d IFF2=%d  HALT=%d\n",
           cpu->I, cpu->R, cpu->IM, cpu->IFF1, cpu->IFF2, cpu->halted);
    printf("  Flags: %c%c%c%c%c%c%c%c  Cycles=%llu\n",
           cpu->F & FS  ? 'S' : '-',
           cpu->F & FZ  ? 'Z' : '-',
           cpu->F & FY  ? 'Y' : '-',
           cpu->F & FH  ? 'H' : '-',
           cpu->F & FX  ? 'X' : '-',
           cpu->F & FPV ? 'V' : '-',
           cpu->F & FN  ? 'N' : '-',
           cpu->F & FC  ? 'C' : '-',
           (unsigned long long)cpu->total_cycles);
}
