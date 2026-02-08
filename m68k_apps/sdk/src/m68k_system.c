/*
 * M68K SDK - System Implementation
 */

#include "m68k_system.h"
#include "m68k_io.h"

/* ---- DMA ---- */

int dma_m68k_to_arm(uint32_t src, uint32_t dst, uint32_t len) {
    IO_WRITE32(DMA_SRC_ADDR, src);
    IO_WRITE32(DMA_DST_ADDR, dst);
    IO_WRITE32(DMA_LENGTH, len);
    IO_WRITE32(DMA_CONTROL, DMA_CMD_START | DMA_CMD_M68K_TO_ARM);
    /* Wait for completion */
    while (IO_READ32(DMA_STATUS) & 0x01) { /* spin */ }
    return 0;
}

int dma_arm_to_m68k(uint32_t src, uint32_t dst, uint32_t len) {
    IO_WRITE32(DMA_SRC_ADDR, src);
    IO_WRITE32(DMA_DST_ADDR, dst);
    IO_WRITE32(DMA_LENGTH, len);
    IO_WRITE32(DMA_CONTROL, DMA_CMD_START | DMA_CMD_ARM_TO_M68K);
    while (IO_READ32(DMA_STATUS) & 0x01) { /* spin */ }
    return 0;
}

bool dma_is_busy(void) {
    return (IO_READ32(DMA_STATUS) & 0x01) != 0;
}

/* ---- Timer ---- */

uint32_t timer_ticks(void) {
    return IO_READ32(TIMER_COUNT);
}

void delay_loops(uint32_t loops) {
    volatile uint32_t i = loops;
    while (i-- > 0) { /* spin */ }
}

void delay_ms(uint32_t ms) {
    /* At ~60MHz emulated, roughly 6000 loop iterations ≈ 1ms */
    delay_loops(ms * 6000);
}

/* ---- Interrupts ---- */

void irq_enable(uint32_t mask) {
    uint32_t current = IO_READ32(IRQ_ENABLE);
    IO_WRITE32(IRQ_ENABLE, current | mask);
}

void irq_disable(uint32_t mask) {
    uint32_t current = IO_READ32(IRQ_ENABLE);
    IO_WRITE32(IRQ_ENABLE, current & ~mask);
}

uint32_t irq_pending(void) {
    return IO_READ32(IRQ_PENDING);
}

void irq_ack(uint32_t mask) {
    IO_WRITE32(IRQ_ACK, mask);
}

/* ---- CPU Control ---- */

void cpu_halt(void) {
    while (1) { /* infinite loop */ }
    __builtin_unreachable();
}

/* ---- Simple Bump Allocator ---- */

static uint32_t heap_ptr   = 0;
static uint32_t heap_start = 0;
static uint32_t heap_end   = 0;

void heap_init(uint32_t start, uint32_t end) {
    heap_start = start;
    heap_end   = end;
    heap_ptr   = start;
}

void *heap_alloc(uint32_t size) {
    /* Align to 4 bytes */
    size = (size + 3) & ~3;
    if (heap_ptr + size > heap_end)
        return NULL;
    void *ptr = (void *)heap_ptr;
    heap_ptr += size;
    return ptr;
}

void heap_free(void *ptr) {
    (void)ptr; /* No-op in bump allocator */
}

uint32_t heap_free_space(void) {
    return heap_end - heap_ptr;
}

uint32_t heap_total_space(void) {
    return heap_end - heap_start;
}
