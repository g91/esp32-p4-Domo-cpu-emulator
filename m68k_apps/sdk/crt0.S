/*
 * M68K SDK - C Runtime Startup (crt0.S)
 *
 * Minimal startup for freestanding M68K programs loaded at 0x00100000.
 * Sets up the stack pointer, zeroes BSS, and calls main().
 * If main() returns, calls cpu_halt loop.
 */

    .section .text.startup, "ax"
    .globl _start
    .globl _exit

_start:
    /* Set stack pointer (grows down from 8MB mark) */
    lea     0x00800000, %sp

    /* Zero out .bss section */
    lea     __bss_start, %a0
    lea     __bss_end, %a1
    cmp.l   %a0, %a1
    beq     .bss_done
.bss_loop:
    clr.l   (%a0)+
    cmp.l   %a0, %a1
    bhi     .bss_loop
.bss_done:

    /* Call main(void) */
    jsr     main

    /* If main returns, halt */
_exit:
    /* Write return code to console */
    stop    #0x2700
    bra     _exit
