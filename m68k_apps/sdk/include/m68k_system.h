/*
 * M68K SDK - System API (DMA, Timer, Interrupts, Memory)
 * ======================================================
 * Low-level system services: DMA transfers, timer access,
 * interrupt management, and memory utilities.
 */

#ifndef M68K_SYSTEM_H
#define M68K_SYSTEM_H

#include "m68k_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- DMA Transfers ---- */

/* Copy data from M68K address to ARM/host address via DMA.
 * Useful for pushing data to host peripherals. */
int dma_m68k_to_arm(uint32_t src, uint32_t dst, uint32_t len);

/* Copy data from ARM/host address to M68K address via DMA. */
int dma_arm_to_m68k(uint32_t src, uint32_t dst, uint32_t len);

/* Check if DMA is busy (returns true if transfer in progress) */
bool dma_is_busy(void);

/* ---- Timer ---- */

/* Read the system tick counter (increments at some rate set by host) */
uint32_t timer_ticks(void);

/* Simple busy-wait delay using a decrement loop.
 * The actual time depends on CPU clock speed (~60MHz emulated).
 * Approximately: loops_per_ms ≈ 6000 at 60MHz. */
void delay_loops(uint32_t loops);

/* Approximate millisecond delay (calibrated for ~60MHz M68K) */
void delay_ms(uint32_t ms);

/* ---- Interrupts ---- */

/* Enable specific interrupt(s) by mask */
void irq_enable(uint32_t mask);

/* Disable specific interrupt(s) by mask */
void irq_disable(uint32_t mask);

/* Get pending interrupt mask */
uint32_t irq_pending(void);

/* Acknowledge (clear) an interrupt */
void irq_ack(uint32_t mask);

/* ---- CPU Control ---- */

/* Halt the CPU (infinite loop) */
void cpu_halt(void) __attribute__((noreturn));

/* Read a CPU data register (d0-d7) by index */
uint32_t cpu_read_dreg(int n);

/* Read a CPU address register (a0-a7) by index */
uint32_t cpu_read_areg(int n);

/* ---- Simple Memory Heap ---- */

/* Initialize the heap allocator.
 * heap_start: first usable address, heap_end: last+1 address */
void heap_init(uint32_t start, uint32_t end);

/* Allocate 'size' bytes (4-byte aligned). Returns NULL if out of memory. */
void *heap_alloc(uint32_t size);

/* Free memory (no-op in bump allocator, included for API completeness) */
void heap_free(void *ptr);

/* Get remaining free heap space */
uint32_t heap_free_space(void);

/* Get total heap size */
uint32_t heap_total_space(void);

#ifdef __cplusplus
}
#endif

#endif /* M68K_SYSTEM_H */
