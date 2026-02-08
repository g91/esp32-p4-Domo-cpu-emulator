/*
 * M68K SDK - Linker Script for Application Programs
 *
 * Programs are loaded at 0x00100000 (1MB) by the M68K emulator.
 * Stack grows down from 0x00800000 (8MB).
 * I/O space: 0x00F00000 - 0x00FFFFFF (bus controller devices).
 */

OUTPUT_FORMAT("elf32-m68k")
OUTPUT_ARCH(m68k)
ENTRY(_start)

MEMORY
{
    RAM (rwx) : ORIGIN = 0x00100000, LENGTH = 7M    /* 1MB - 8MB */
    IO  (rw)  : ORIGIN = 0x00F00000, LENGTH = 1M    /* Bus controller I/O */
}

SECTIONS
{
    /* Code and read-only data */
    .text : {
        *(.text.startup)     /* crt0 entry point first */
        *(.text .text.*)
        *(.rodata .rodata.*)
        . = ALIGN(4);
    } > RAM

    /* Initialized data */
    .data : {
        *(.data .data.*)
        . = ALIGN(4);
    } > RAM

    /* Uninitialized data (zeroed by crt0) */
    .bss : {
        __bss_start = .;
        *(.bss .bss.*)
        *(COMMON)
        . = ALIGN(4);
        __bss_end = .;
    } > RAM

    /* Heap starts after BSS */
    _heap_start = __bss_end;
    _heap_end   = 0x00800000;    /* Up to stack base at 8MB */

    /* Discard unneeded sections */
    /DISCARD/ : {
        *(.comment)
        *(.note.*)
        *(.eh_frame*)
    }
}
