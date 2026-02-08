/*
 * M68K SDK - FPU Implementation
 */

#include "m68k_fpu.h"
#include "m68k_io.h"
#include "m68k_console.h"

/* ---- Operand Setup ---- */

void fpu_set_a(uint32_t hi, uint32_t lo) {
    IO_WRITE32(FPU_OP_A_HI, hi);
    IO_WRITE32(FPU_OP_A_LO, lo);
}

void fpu_set_b(uint32_t hi, uint32_t lo) {
    IO_WRITE32(FPU_OP_B_HI, hi);
    IO_WRITE32(FPU_OP_B_LO, lo);
}

void fpu_set_a_d(fpu_double_t val) {
    fpu_set_a(val.hi, val.lo);
}

void fpu_set_b_d(fpu_double_t val) {
    fpu_set_b(val.hi, val.lo);
}

void fpu_set_int(int32_t val) {
    IO_WRITE32(FPU_INT_RESULT, (uint32_t)val);
}

void fpu_set_a_int(int32_t val) {
    fpu_set_int(val);
    fpu_exec(FPU_CMD_ITOF);
    fpu_double_t r = fpu_get_result();
    fpu_set_a(r.hi, r.lo);
}

void fpu_set_b_int(int32_t val) {
    fpu_set_int(val);
    fpu_exec(FPU_CMD_ITOF);
    fpu_double_t r = fpu_get_result();
    fpu_set_b(r.hi, r.lo);
}

/* ---- Execution ---- */

uint32_t fpu_exec(uint32_t cmd) {
    IO_WRITE32(FPU_COMMAND, cmd);
    return IO_READ32(FPU_STATUS);
}

/* ---- Result Reading ---- */

fpu_double_t fpu_get_result(void) {
    fpu_double_t r;
    r.hi = IO_READ32(FPU_RESULT_HI);
    r.lo = IO_READ32(FPU_RESULT_LO);
    return r;
}

uint32_t fpu_get_result_hi(void) {
    return IO_READ32(FPU_RESULT_HI);
}

uint32_t fpu_get_result_lo(void) {
    return IO_READ32(FPU_RESULT_LO);
}

int32_t fpu_get_int_result(void) {
    return (int32_t)IO_READ32(FPU_INT_RESULT);
}

uint32_t fpu_get_status(void) {
    return IO_READ32(FPU_STATUS);
}

/* ---- Convenience: single-operand ---- */

static fpu_double_t fpu_unary(fpu_double_t val, uint32_t cmd) {
    fpu_set_a_d(val);
    fpu_exec(cmd);
    return fpu_get_result();
}

fpu_double_t fpu_sqrt(fpu_double_t val) { return fpu_unary(val, FPU_CMD_SQRT); }
fpu_double_t fpu_abs(fpu_double_t val)  { return fpu_unary(val, FPU_CMD_ABS); }
fpu_double_t fpu_neg(fpu_double_t val)  { return fpu_unary(val, FPU_CMD_NEG); }
fpu_double_t fpu_sin(fpu_double_t val)  { return fpu_unary(val, FPU_CMD_SIN); }
fpu_double_t fpu_cos(fpu_double_t val)  { return fpu_unary(val, FPU_CMD_COS); }
fpu_double_t fpu_floor(fpu_double_t val) { return fpu_unary(val, FPU_CMD_FLOOR); }
fpu_double_t fpu_ceil(fpu_double_t val) { return fpu_unary(val, FPU_CMD_CEIL); }

/* ---- Convenience: two-operand ---- */

static fpu_double_t fpu_binary(fpu_double_t a, fpu_double_t b, uint32_t cmd) {
    fpu_set_a_d(a);
    fpu_set_b_d(b);
    fpu_exec(cmd);
    return fpu_get_result();
}

fpu_double_t fpu_add(fpu_double_t a, fpu_double_t b) { return fpu_binary(a, b, FPU_CMD_ADD); }
fpu_double_t fpu_sub(fpu_double_t a, fpu_double_t b) { return fpu_binary(a, b, FPU_CMD_SUB); }
fpu_double_t fpu_mul(fpu_double_t a, fpu_double_t b) { return fpu_binary(a, b, FPU_CMD_MUL); }
fpu_double_t fpu_div(fpu_double_t a, fpu_double_t b) { return fpu_binary(a, b, FPU_CMD_DIV); }
fpu_double_t fpu_pow(fpu_double_t base, fpu_double_t exp) { return fpu_binary(base, exp, FPU_CMD_POW); }

int32_t fpu_cmp(fpu_double_t a, fpu_double_t b) {
    fpu_set_a_d(a);
    fpu_set_b_d(b);
    fpu_exec(FPU_CMD_CMP);
    return fpu_get_int_result();
}

/* ---- Conversion ---- */

fpu_double_t fpu_from_int(int32_t val) {
    fpu_set_int(val);
    fpu_exec(FPU_CMD_ITOF);
    return fpu_get_result();
}

int32_t fpu_to_int(fpu_double_t val) {
    fpu_set_a_d(val);
    fpu_exec(FPU_CMD_FTOI);
    return fpu_get_int_result();
}

/* ---- Constants ---- */

fpu_double_t fpu_pi(void) {
    fpu_exec(FPU_CMD_PI);
    return fpu_get_result();
}

fpu_double_t fpu_e(void) {
    fpu_exec(FPU_CMD_E);
    return fpu_get_result();
}

/* ---- Printing ---- */

void fpu_print(const char *label) {
    uint32_t status = fpu_get_status();

    if (status & FPU_S_NAN) {
        con_printf("%s = NaN\n", label);
        return;
    }
    if (status & FPU_S_INF) {
        con_printf("%s = %sInfinity\n", label, (status & FPU_S_NEG) ? "-" : "");
        return;
    }

    /* Save the current result */
    fpu_double_t r = fpu_get_result();
    bool negative = (status & FPU_S_NEG) != 0;

    /* Get absolute value */
    if (negative) {
        fpu_set_a_d(r);
        fpu_exec(FPU_CMD_ABS);
        r = fpu_get_result();
    }

    /* Integer part via FTOI */
    fpu_set_a_d(r);
    fpu_exec(FPU_CMD_FTOI);
    int32_t int_part = fpu_get_int_result();

    /* Fractional part: frac = abs_result - floor(int_part) */
    fpu_set_int(int_part);
    fpu_exec(FPU_CMD_ITOF);
    fpu_double_t int_as_float = fpu_get_result();

    fpu_set_a_d(r);
    fpu_set_b_d(int_as_float);
    fpu_exec(FPU_CMD_SUB);

    /* Multiply by 1000000 for 6 decimal places */
    fpu_double_t frac = fpu_get_result();
    fpu_set_a_d(frac);
    fpu_set_b_int(1000000);
    fpu_exec(FPU_CMD_MUL);
    fpu_exec(FPU_CMD_FTOI);
    int32_t frac_part = fpu_get_int_result();
    if (frac_part < 0) frac_part = -frac_part;

    /* Print: label = [-]int.frac */
    con_printf("%s = %s%d.", label, negative ? "-" : "", int_part);

    /* Print fractional part with leading zeros (6 digits) */
    int32_t divisor = 100000;
    for (int i = 0; i < 6; i++) {
        con_putchar('0' + (char)(frac_part / divisor % 10));
        divisor /= 10;
    }
    con_putchar('\r');
    con_putchar('\n');
}
