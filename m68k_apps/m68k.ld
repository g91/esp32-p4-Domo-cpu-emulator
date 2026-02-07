/* Linker script for M68K programs on ESP32-P4 emulator
 * Memory layout:
 * - Reset vectors at 0x00000000 (8 bytes)
 * - Program code starts at 0x00000010
 * - I/O space at 0x00F00000 - 0x00FFFFFF
 * - Maximum program size: ~14.5MB to leave room for stack
 */

OUTPUT_FORMAT("elf32-m68k")
OUTPUT_ARCH(m68k)
ENTRY(_start)

MEMORY
{
    /* Main RAM: 0x00000000 - 0x00BFFFFF (12MB usable, 4MB safety margin)
     * Limited to 12MB to ensure all operations stay well within 16MB physical limit
     * Stack grows down from 0x00C00000 (12MB mark) */
    RAM (rwx) : ORIGIN = 0x00000000, LENGTH = 12M
}

SECTIONS
{
    /* Reset vectors must be at address 0 */
    .vectors 0x00000000 : {
        LONG(0x00C00000);  /* Initial Stack Pointer at 12MB mark (safe within 16MB limit) */
        LONG(_start);       /* Initial Program Counter */
        . = 0x10;           /* Pad to offset 0x10 */
    } > RAM
    
    /* Code section starts at 0x00000010 */
    .text 0x00000010 : {
        *(.text)
        *(.text*)
        *(.rodata)
        *(.rodata*)
    } > RAM
    
    /* Data section */
    .data : {
        *(.data)
        *(.data*)
    } > RAM
    
    /* BSS section (uninitialized data) */
    .bss : {
        __bss_start = .;
        *(.bss)
        *(.bss*)
        *(COMMON)
        __bss_end = .;
    } > RAM
    
    _end = .;
    
    /* Discard debug and other sections */
    /DISCARD/ : {
        *(.comment)
        *(.note*)
        *(.eh_frame)
        *(.debug*)
    }
}
