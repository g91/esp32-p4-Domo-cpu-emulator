/*
 * M68K SDK - FPU (Floating Point Unit) API
 * =========================================
 * IEEE 754 double-precision math coprocessor via the bus controller.
 * The ESP32-P4 host performs all floating-point calculations instantly.
 *
 * Usage pattern:
 *   fpu_set_a_int(100);     // Set operand A = 100.0
 *   fpu_exec(FPU_CMD_SQRT); // Calculate sqrt(100)
 *   fpu_print("result");    // Prints "result = 10.000000"
 *
 * All double values are stored as two 32-bit halves (big-endian):
 *   HI = bits [63:32], LO = bits [31:0]
 */

#ifndef M68K_FPU_H
#define M68K_FPU_H

#include "m68k_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* A 64-bit double stored as two 32-bit halves */
typedef struct {
    uint32_t hi;
    uint32_t lo;
} fpu_double_t;

/* ---- Operand Setup ---- */

/* Set operand A from raw 64-bit halves */
void fpu_set_a(uint32_t hi, uint32_t lo);

/* Set operand B from raw 64-bit halves */
void fpu_set_b(uint32_t hi, uint32_t lo);

/* Set operand A from fpu_double_t */
void fpu_set_a_d(fpu_double_t val);

/* Set operand B from fpu_double_t */
void fpu_set_b_d(fpu_double_t val);

/* Set operand A from a 32-bit integer (converts to double via ITOF) */
void fpu_set_a_int(int32_t val);

/* Set operand B from a 32-bit integer (converts to double via ITOF) */
void fpu_set_b_int(int32_t val);

/* Set the integer register directly (used for ITOF input) */
void fpu_set_int(int32_t val);

/* ---- Execution ---- */

/* Execute an FPU command. Returns the status register.
 * See FPU_CMD_* constants in m68k_io.h */
uint32_t fpu_exec(uint32_t cmd);

/* ---- Result Reading ---- */

/* Get the result as raw 64-bit halves */
fpu_double_t fpu_get_result(void);

/* Get the result's high 32 bits */
uint32_t fpu_get_result_hi(void);

/* Get the result's low 32 bits */
uint32_t fpu_get_result_lo(void);

/* Get the integer result (from FTOI, CMP) */
int32_t fpu_get_int_result(void);

/* Get the FPU status register */
uint32_t fpu_get_status(void);

/* ---- Convenience: One-operand operations (uses A) ---- */

/* result = sqrt(val) */
fpu_double_t fpu_sqrt(fpu_double_t val);

/* result = |val| */
fpu_double_t fpu_abs(fpu_double_t val);

/* result = -val */
fpu_double_t fpu_neg(fpu_double_t val);

/* result = sin(val) */
fpu_double_t fpu_sin(fpu_double_t val);

/* result = cos(val) */
fpu_double_t fpu_cos(fpu_double_t val);

/* result = floor(val) */
fpu_double_t fpu_floor(fpu_double_t val);

/* result = ceil(val) */
fpu_double_t fpu_ceil(fpu_double_t val);

/* ---- Convenience: Two-operand operations (uses A, B) ---- */

/* result = a + b */
fpu_double_t fpu_add(fpu_double_t a, fpu_double_t b);

/* result = a - b */
fpu_double_t fpu_sub(fpu_double_t a, fpu_double_t b);

/* result = a * b */
fpu_double_t fpu_mul(fpu_double_t a, fpu_double_t b);

/* result = a / b */
fpu_double_t fpu_div(fpu_double_t a, fpu_double_t b);

/* result = a^b */
fpu_double_t fpu_pow(fpu_double_t base, fpu_double_t exp);

/* Compare a and b. Returns -1 (a<b), 0 (a==b), 1 (a>b) */
int32_t fpu_cmp(fpu_double_t a, fpu_double_t b);

/* ---- Conversion ---- */

/* Convert integer to fpu_double_t */
fpu_double_t fpu_from_int(int32_t val);

/* Convert fpu_double_t to integer (truncates) */
int32_t fpu_to_int(fpu_double_t val);

/* ---- Constants ---- */

/* Get pi (3.14159...) */
fpu_double_t fpu_pi(void);

/* Get e (2.71828...) */
fpu_double_t fpu_e(void);

/* ---- Printing ---- */

/* Print a floating-point result with label: "label = [-]int.frac\n"
 * Uses 6 decimal places. Must be called right after fpu_exec() or
 * a convenience function while result is still in registers. */
void fpu_print(const char *label);

#ifdef __cplusplus
}
#endif

#endif /* M68K_FPU_H */
