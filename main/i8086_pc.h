/**
 * @file i8086_pc.h
 * @brief IBM PC/XT Hardware Emulation for ESP32-P4
 *
 * Emulates the essential hardware peripherals of an IBM PC/XT:
 * - PIC 8259A (Programmable Interrupt Controller) - master & slave
 * - PIT 8253 (Programmable Interval Timer) - 3 channels
 * - i8042 Keyboard Controller
 * - MC146818 Real-Time Clock + CMOS RAM
 * - CGA text mode video (80x25, 16 colors)
 * - PC Speaker
 * - DMA controller (stub)
 *
 * Based on the architecture of FabGL's PCEmulator (MIT portions)
 * Adapted for ESP32-P4 with LCD console output
 */

#ifndef I8086_PC_H
#define I8086_PC_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// PIC 8259A - Programmable Interrupt Controller
// ============================================================================

// IRQ assignments (IBM PC/XT standard)
#define IRQ_TIMER       0   // PIT Channel 0 → INT 08h
#define IRQ_KEYBOARD    1   // Keyboard → INT 09h
#define IRQ_CASCADE     2   // Cascade (slave PIC on AT, unused on XT)
#define IRQ_COM2        3   // COM2 → INT 0Bh
#define IRQ_COM1        4   // COM1 → INT 0Ch
#define IRQ_LPT2        5   // LPT2 → INT 0Dh
#define IRQ_FLOPPY      6   // Floppy → INT 0Eh
#define IRQ_LPT1        7   // LPT1/spurious → INT 0Fh
// Slave PIC (IRQ 8-15, AT only)
#define IRQ_RTC         8   // RTC → INT 70h
#define IRQ_REDIR       9   // Redirected IRQ2
#define IRQ_RESERVED1  10
#define IRQ_RESERVED2  11
#define IRQ_MOUSE      12   // PS/2 Mouse → INT 74h
#define IRQ_FPU        13   // FPU → INT 75h
#define IRQ_HDD        14   // Hard disk → INT 76h
#define IRQ_RESERVED3  15

typedef struct {
    uint8_t  IMR;           // Interrupt Mask Register
    uint8_t  IRR;           // Interrupt Request Register
    uint8_t  ISR;           // In-Service Register
    uint8_t  ICW1, ICW2, ICW3, ICW4;
    uint8_t  icw_step;      // ICW initialization step
    bool     init_mode;     // In initialization sequence
    uint8_t  vector_base;   // Base interrupt vector (ICW2)
    bool     read_isr;      // OCW3: read ISR instead of IRR
    uint8_t  priority;      // Lowest priority IRQ
    bool     auto_eoi;      // Automatic EOI mode
} pic8259_t;

void     pic_init(pic8259_t *pic);
void     pic_write(pic8259_t *pic, uint8_t port, uint8_t value);
uint8_t  pic_read(pic8259_t *pic, uint8_t port);
void     pic_signal_irq(pic8259_t *pic, int irq);
void     pic_clear_irq(pic8259_t *pic, int irq);
int      pic_get_pending_irq(pic8259_t *pic);
void     pic_acknowledge(pic8259_t *pic, int irq);

// ============================================================================
// PIT 8253 - Programmable Interval Timer
// ============================================================================

// PIT frequencies
#define PIT_CLOCK_FREQ      1193182     // 1.193182 MHz

// PIT modes
#define PIT_MODE_INTERRUPT      0
#define PIT_MODE_ONESHOT        1
#define PIT_MODE_RATEGEN        2
#define PIT_MODE_SQUAREWAVE     3
#define PIT_MODE_SWSTROBE       4
#define PIT_MODE_HWSTROBE       5

typedef struct {
    uint32_t reload;        // Reload value (0x10000 means 65536)
    uint32_t counter;       // Current counter
    uint8_t  mode;          // Operating mode (0-5)
    uint8_t  bcd;           // BCD mode
    uint8_t  rw_mode;       // Read/write mode
    uint16_t latch;         // Latched value (16-bit counter snapshot)
    bool     latched;       // Value is latched
    bool     gate;          // Gate input
    bool     output;        // Output state
    uint8_t  write_lsb;     // Write phase: 0=LSB first, 1=MSB
    uint8_t  read_lsb;      // Read phase: 0=LSB first, 1=MSB
    bool     loaded;        // Reload value has been loaded
} pit_channel_t;

typedef struct {
    pit_channel_t ch[3];    // 3 timer channels
    uint32_t tick_accum;    // Tick accumulator for fractional timing
} pit8253_t;

typedef void (*pit_output_fn)(int channel, bool state);

void     pit_init(pit8253_t *pit);
void     pit_write(pit8253_t *pit, uint8_t port, uint8_t value);
uint8_t  pit_read(pit8253_t *pit, uint8_t port);
void     pit_tick(pit8253_t *pit, int ticks);
bool     pit_get_output(pit8253_t *pit, int channel);

// ============================================================================
// Keyboard Controller (simplified i8042)
// ============================================================================

#define KBD_BUFFER_SIZE     32

typedef struct {
    uint8_t  buffer[KBD_BUFFER_SIZE];
    int      head, tail;
    uint8_t  status;        // Status register (port 0x64 read)
    uint8_t  command;       // Last command written
    bool     enabled;       // Keyboard enabled
    uint8_t  scancode_set;  // Active scancode set (1=XT)
    // Shift key state tracking
    uint8_t  shift_flags;   // Bit 0=RShift, 1=LShift, 2=Ctrl, 3=Alt,
                            // 4=ScrollLock, 5=NumLock, 6=CapsLock, 7=Insert
    uint8_t  kbd_leds;      // LED state
} kbd_ctrl_t;

void     kbd_init(kbd_ctrl_t *kbd);
void     kbd_write_data(kbd_ctrl_t *kbd, uint8_t value);
void     kbd_write_cmd(kbd_ctrl_t *kbd, uint8_t value);
uint8_t  kbd_read_data(kbd_ctrl_t *kbd);
uint8_t  kbd_read_status(kbd_ctrl_t *kbd);
bool     kbd_has_data(kbd_ctrl_t *kbd);
void     kbd_push_scancode(kbd_ctrl_t *kbd, uint8_t scancode);
void     kbd_push_key(kbd_ctrl_t *kbd, uint8_t scancode, uint8_t ascii);

// ============================================================================
// MC146818 Real-Time Clock + CMOS
// ============================================================================

#define CMOS_SIZE   128

typedef struct {
    uint8_t  cmos[CMOS_SIZE];   // CMOS RAM (includes RTC registers)
    uint8_t  reg_select;        // Currently selected register
    bool     nmi_enabled;       // NMI enable/disable
} rtc_t;

void     rtc_init(rtc_t *rtc);
void     rtc_write(rtc_t *rtc, uint8_t port, uint8_t value);
uint8_t  rtc_read(rtc_t *rtc, uint8_t port);
void     rtc_update_time(rtc_t *rtc);  // Update from system time

// ============================================================================
// CGA Video Controller (text mode focus)
// ============================================================================

#define CGA_TEXT_COLS   80
#define CGA_TEXT_ROWS   25
#define CGA_VRAM_BASE   0xB8000
#define CGA_VRAM_SIZE   16384   // 16KB

typedef struct {
    uint8_t  mode_reg;          // 0x3D8: Mode control register
    uint8_t  color_reg;         // 0x3D9: Color select register
    uint8_t  status;            // 0x3DA: Status register
    uint8_t  crtc_reg_select;   // 0x3D4: CRTC register index
    uint8_t  crtc[18];          // 0x3D5: CRTC registers
    uint16_t mem_offset;        // Display start offset
    uint16_t cursor_pos;        // Cursor position (offset)
    uint8_t  cursor_start;      // Cursor start scanline
    uint8_t  cursor_end;        // Cursor end scanline
    bool     cursor_visible;
    uint8_t  vsync_count;       // VSync toggle counter
    bool     needs_refresh;     // Screen needs updating
} cga_t;

void     cga_init(cga_t *cga);
void     cga_write(cga_t *cga, uint16_t port, uint8_t value);
uint8_t  cga_read(cga_t *cga, uint16_t port);
uint16_t cga_get_cursor_offset(cga_t *cga);

// ============================================================================
// PC System - Everything Together
// ============================================================================

typedef struct {
    pic8259_t   pic_master;
    pic8259_t   pic_slave;
    pit8253_t   pit;
    kbd_ctrl_t  keyboard;
    rtc_t       rtc;
    cga_t       cga;

    // Port 0x61 system control
    uint8_t     port_b;         // System control port B
    bool        speaker_enabled;
    uint32_t    speaker_freq;

    // DMA page registers (stub)
    uint8_t     dma_page[8];

    // Timing
    uint32_t    cpu_cycles;         // Cycle counter for PIT ticking
    uint32_t    cycles_per_pit_tick;// CPU cycles between PIT ticks
    uint32_t    pit_tick_accum;     // Accumulator for PIT ticks
    bool        prev_pit0_output;   // Previous PIT ch0 output for edge detection

    // Video refresh
    uint32_t    refresh_counter;    // Count instructions between LCD refreshes
    uint32_t    refresh_interval;   // Instructions between refreshes

    bool        initialized;
} pc_hardware_t;

/**
 * Initialize all PC hardware
 */
void pc_hw_init(pc_hardware_t *hw, uint32_t cpu_freq_mhz);

/**
 * Process a CPU port write
 */
void pc_hw_write_port(pc_hardware_t *hw, uint16_t port, uint8_t value);

/**
 * Process a CPU port read
 */
uint8_t pc_hw_read_port(pc_hardware_t *hw, uint16_t port);

/**
 * Tick hardware (call after each CPU step batch)
 * @param cpu_cycles Number of CPU cycles executed since last tick
 * @return IRQ number to inject, or -1 if none
 */
int pc_hw_tick(pc_hardware_t *hw, uint32_t cpu_cycles);

/**
 * Check if screen needs refresh and render to LCD
 * @param video_mem Pointer to video memory (0xB8000)
 */
void pc_hw_render_text(pc_hardware_t *hw, const uint8_t *video_mem);

/**
 * Render CGA 320x200x4 color graphics mode to LCD
 * @param video_mem Pointer to CGA video memory (0xB8000)
 */
void pc_hw_render_320x200(pc_hardware_t *hw, const uint8_t *video_mem);

/**
 * Render CGA 640x200x2 color graphics mode to LCD
 * @param video_mem Pointer to CGA video memory (0xB8000)
 */
void pc_hw_render_640x200(pc_hardware_t *hw, const uint8_t *video_mem);

/**
 * Auto-detect CGA mode and render appropriately (text or graphics)
 * @param video_mem Pointer to CGA video memory (0xB8000)
 */
void pc_hw_render_cga(pc_hardware_t *hw, const uint8_t *video_mem);

#ifdef __cplusplus
}
#endif

#endif // I8086_PC_H
