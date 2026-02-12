/*
 * MOS 6502 Emulator Integration Layer
 * Memory mapping, bus controller I/O, and ESP-IDF integration
 * With full C64 CIA/VIC-II emulation for BASIC boot
 */

#include "mos6502_emulator.h"
#include "mos6502_cpu.h"
#include "bus_controller.h"
#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "video_card.h"
#include "ps2_keyboard.h"
#include "usb_keyboard.h"

static const char *TAG = "6502";

// External console_printf from main
extern int console_printf(const char *fmt, ...);

// ============================================================================
// C64 CIA Timer emulation
// ============================================================================
typedef struct {
    uint16_t timer_a;       // Current timer A value
    uint16_t timer_a_latch; // Timer A latch (reload value)
    uint16_t timer_b;       // Current timer B value
    uint16_t timer_b_latch; // Timer B latch
    uint8_t  icr_data;      // Interrupt data register (what happened)
    uint8_t  icr_mask;      // Interrupt mask register (what's enabled)
    uint8_t  cra;           // Control register A
    uint8_t  crb;           // Control register B
    uint8_t  port_a;        // Port A data
    uint8_t  port_b;        // Port B data
    uint8_t  ddr_a;         // Port A direction
    uint8_t  ddr_b;         // Port B direction
    uint8_t  tod_10ths;     // TOD tenths of seconds
    uint8_t  tod_sec;       // TOD seconds
    uint8_t  tod_min;       // TOD minutes
    uint8_t  tod_hr;        // TOD hours
} cia_t;

// ============================================================================
// C64 Keyboard matrix - real 8x8 CIA1 scanning
// ============================================================================
typedef struct {
    uint8_t  matrix[8];     // 8 columns (PA0-7), bits=rows (PB0-7), active low
    int32_t  release_timer; // Cycles until auto-release all keys
} c64_keyboard_t;

// ============================================================================
// SID 6581 sound synthesis state
// ============================================================================
#define SID_CLOCK_PAL  985248   // SID clock frequency (PAL)
#define SID_SAMPLE_RATE 22050   // Audio output sample rate

typedef struct {
    uint32_t phase;         // 24-bit phase accumulator
    uint16_t freq;          // Frequency register (16-bit)
    uint16_t pulse_width;   // Pulse width (12-bit)
    uint8_t  control;       // Waveform control ($x4): gate, sync, ring, test, tri, saw, pulse, noise
    uint8_t  ad;            // Attack/Decay rates
    uint8_t  sr;            // Sustain level / Release rate
    uint8_t  env_state;     // 0=idle, 1=attack, 2=decay, 3=sustain, 4=release
    uint8_t  env_level;     // Current envelope level (0-255)
    uint16_t env_counter;   // Rate counter for envelope
    uint32_t noise_lfsr;    // 23-bit LFSR for noise waveform
} sid_voice_t;

typedef struct {
    sid_voice_t voice[3];
    uint8_t  volume;        // Master volume (0-15)
    uint32_t sample_accum;  // Cycle accumulator for sample generation
} sid_state_t;

// Emulator state
static struct {
    mos6502_t cpu;
    uint8_t  *memory;       // 64KB address space
    bool      initialized;
    mos6502_machine_mode_t mode;

    // C64/VIC-20 specific state
    uint8_t  *chargen_rom;   // Character generator ROM (4KB)
    uint8_t   c64_bank_cfg;  // C64 $0001 bank configuration register
    uint8_t   c64_port_dir;  // C64 $0000 data direction register

    // C64 VIC-II state
    uint8_t   vic_regs[64];
    uint16_t  raster_line;   // Current raster line (0-311 for PAL)
    uint8_t   vic_irq_status; // VIC IRQ status bits
    uint8_t   vic_irq_enable; // VIC IRQ enable bits

    // C64 SID
    uint8_t   sid_regs[32];
    sid_state_t sid;

    // C64 CIAs
    cia_t     cia1;
    cia_t     cia2;

    // Color RAM
    uint8_t   color_ram[1024];

    // Keyboard
    c64_keyboard_t keyboard;

    // Screen rendering state for UART serial diff
    uint8_t   prev_screen[1000]; // Previous screen contents for diff
    bool      screen_dirty;
    uint32_t  frame_count;

    // VIC-20 I/O state
    uint8_t   vic20_via1[16];
    uint8_t   vic20_via2[16];
    uint8_t   vic20_vic_regs[16];
} s_emu;

// ============================================================================
// C64 Color Palette (RGB565 format for LCD)
// ============================================================================
static const uint16_t c64_colors_rgb565[16] = {
    0x0000,  // 0  - Black
    0xFFFF,  // 1  - White
    0x8000,  // 2  - Red
    0xA7FC,  // 3  - Cyan/Turquoise
    0xC218,  // 4  - Purple
    0x064A,  // 5  - Green
    0x0014,  // 6  - Blue
    0xE74E,  // 7  - Yellow
    0xD42A,  // 8  - Orange
    0x6200,  // 9  - Brown
    0xFBAE,  // 10 - Light Red
    0x3186,  // 11 - Dark Grey (Grey 1)
    0x73AE,  // 12 - Grey (Grey 2)
    0xA7EC,  // 13 - Light Green
    0x043F,  // 14 - Light Blue
    0xB5D6   // 15 - Light Grey (Grey 3)
};

// ============================================================================
// Bus I/O helper (bare mode) - routes to bus controller devices
// ============================================================================

static uint8_t bus_io_read_mapped(uint16_t addr) {
    uint32_t io_offset = addr - MOS6502_IO_BASE;
    uint32_t page = (io_offset >> 8) & 0x0F;
    uint32_t reg = io_offset & 0xFF;
    uint32_t bus_addr;
    switch (page) {
    case 0x0: bus_addr = BUS_IO_BASE + BUS_DEV_NETWORK + reg; break;
    case 0x1: bus_addr = BUS_IO_BASE + BUS_DEV_FILESYSTEM + reg; break;
    case 0x2: bus_addr = BUS_IO_BASE + BUS_DEV_FILESYSTEM + 0x100 + reg; break;
    case 0x3: bus_addr = BUS_IO_BASE + BUS_DEV_FILESYSTEM + 0x200 + reg; break;
    case 0x4: bus_addr = BUS_IO_BASE + BUS_DEV_CONSOLE + reg; break;
    case 0x5: bus_addr = BUS_IO_BASE + BUS_DEV_DMA + reg; break;
    case 0x6: bus_addr = BUS_IO_BASE + BUS_DEV_IRQ + reg; break;
    case 0x7: bus_addr = BUS_IO_BASE + BUS_DEV_TIMER + reg; break;
    case 0x8: bus_addr = BUS_IO_BASE + BUS_DEV_FPU + reg; break;
    case 0x9: bus_addr = BUS_IO_BASE + BUS_DEV_AUDIO + reg; break;
    default:  return 0xFF;
    }
    return (uint8_t)bus_io_read(bus_addr, 1);
}

static void bus_io_write_mapped(uint16_t addr, uint8_t val) {
    uint32_t io_offset = addr - MOS6502_IO_BASE;
    uint32_t page = (io_offset >> 8) & 0x0F;
    uint32_t reg = io_offset & 0xFF;
    uint32_t bus_addr;
    switch (page) {
    case 0x0: bus_addr = BUS_IO_BASE + BUS_DEV_NETWORK + reg; break;
    case 0x1: bus_addr = BUS_IO_BASE + BUS_DEV_FILESYSTEM + reg; break;
    case 0x2: bus_addr = BUS_IO_BASE + BUS_DEV_FILESYSTEM + 0x100 + reg; break;
    case 0x3: bus_addr = BUS_IO_BASE + BUS_DEV_FILESYSTEM + 0x200 + reg; break;
    case 0x4: bus_addr = BUS_IO_BASE + BUS_DEV_CONSOLE + reg; break;
    case 0x5: bus_addr = BUS_IO_BASE + BUS_DEV_DMA + reg; break;
    case 0x6: bus_addr = BUS_IO_BASE + BUS_DEV_IRQ + reg; break;
    case 0x7: bus_addr = BUS_IO_BASE + BUS_DEV_TIMER + reg; break;
    case 0x8: bus_addr = BUS_IO_BASE + BUS_DEV_FPU + reg; break;
    case 0x9: bus_addr = BUS_IO_BASE + BUS_DEV_AUDIO + reg; break;
    default:  return;
    }
    bus_io_write(bus_addr, val, 1);
}

// ============================================================================
// C64 Keyboard Matrix - ASCII/PS2 to C64 matrix position mapping
// CIA1 Port A (output) selects column (active low)
// CIA1 Port B (input) reads row (active low)
// Matrix: 8 columns (PA0-PA7) x 8 rows (PB0-PB7)
//
//         PA0(C0)  PA1(C1)  PA2(C2)  PA3(C3)  PA4(C4)  PA5(C5)  PA6(C6)  PA7(C7)
// PB0(R0) DEL      RETURN   CRS-R    F7       F1       F3       F5       CRS-D
// PB1(R1) 3        W        A        4        Z        S        E        LSHIFT
// PB2(R2) 5        R        D        6        C        F        T        X
// PB3(R3) 7        Y        G        8        B        H        U        V
// PB4(R4) 9        I        J        0        M        K        O        N
// PB5(R5) +        P        L        -        .        :        @        ,
// PB6(R6) POUND    *        ;        HOME     RSHIFT   =        UP-ARR   /
// PB7(R7) 1        LEFT-ARR CTRL     2        SPACE    C=       Q        RUNSTOP
// ============================================================================

// Encoding: bits 5-3 = column, bits 2-0 = row, bit 6 = need LSHIFT, 0xFF = unmapped
#define K(c,r)   (((c)<<3)|(r))
#define KS(c,r)  (((c)<<3)|(r)|0x40)

// ASCII (0-127) to C64 matrix key encoding
__attribute__((unused))
static const uint8_t ascii_to_c64_key[128] = {
    // 0x00-0x0F
    0xFF,    0xFF,    0xFF,    0xFF,    0xFF,    0xFF,    0xFF,    0xFF,
    K(0,0),  0xFF,    0xFF,    0xFF,    0xFF,    K(1,0),  0xFF,    0xFF,
    // 0x10-0x1F
    0xFF,    0xFF,    0xFF,    0xFF,    0xFF,    0xFF,    0xFF,    0xFF,
    0xFF,    0xFF,    0xFF,    0xFF,    0xFF,    0xFF,    0xFF,    0xFF,
    // 0x20-0x2F: space ! " # $ % & ' ( ) * + , - . /
    K(4,7),  KS(0,7), KS(3,7), KS(0,1), KS(3,1), KS(0,2), KS(3,2), KS(0,3),
    KS(3,3), KS(0,4), K(1,6),  K(0,5),  K(7,5),  K(3,5),  K(4,5),  K(7,6),
    // 0x30-0x3F: 0 1 2 3 4 5 6 7 8 9 : ; < = > ?
    K(3,4),  K(0,7),  K(3,7),  K(0,1),  K(3,1),  K(0,2),  K(3,2),  K(0,3),
    K(3,3),  K(0,4),  K(5,5),  K(2,6),  KS(7,5), K(5,6),  KS(4,5), KS(7,6),
    // 0x40-0x4F: @ A B C D E F G H I J K L M N O
    K(6,5),  K(2,1),  K(4,3),  K(4,2),  K(2,2),  K(6,1),  K(5,2),  K(2,3),
    K(5,3),  K(1,4),  K(2,4),  K(5,4),  K(2,5),  K(4,4),  K(7,4),  K(6,4),
    // 0x50-0x5F: P Q R S T U V W X Y Z [ \ ] ^ _
    K(1,5),  K(6,7),  K(1,2),  K(5,1),  K(6,2),  K(6,3),  K(7,3),  K(1,1),
    K(7,2),  K(1,3),  K(4,1),  KS(5,5), 0xFF,    KS(2,6), K(6,6),  0xFF,
    // 0x60-0x6F: ` a b c d e f g h i j k l m n o
    0xFF,    K(2,1),  K(4,3),  K(4,2),  K(2,2),  K(6,1),  K(5,2),  K(2,3),
    K(5,3),  K(1,4),  K(2,4),  K(5,4),  K(2,5),  K(4,4),  K(7,4),  K(6,4),
    // 0x70-0x7F: p q r s t u v w x y z { | } ~ DEL
    K(1,5),  K(6,7),  K(1,2),  K(5,1),  K(6,2),  K(6,3),  K(7,3),  K(1,1),
    K(7,2),  K(1,3),  K(4,1),  0xFF,    0xFF,    0xFF,    0xFF,    K(0,0),
};

// Press a key in the matrix (set bits LOW = pressed)
__attribute__((unused))
static void c64_matrix_press(uint8_t encoded) {
    if (encoded == 0xFF) return;
    uint8_t col = (encoded >> 3) & 7;
    uint8_t row = encoded & 7;
    s_emu.keyboard.matrix[col] &= ~(1 << row);
    if (encoded & 0x40) {
        // Also press LSHIFT (col=7, row=1)
        s_emu.keyboard.matrix[7] &= ~(1 << 1);
    }
}

// Release all keys
static void c64_matrix_release_all(void) {
    memset(s_emu.keyboard.matrix, 0xFF, 8);
}

// ============================================================================
// C64 CIA emulation
// ============================================================================

static uint8_t cia_read(cia_t *cia, uint8_t reg, bool is_cia1) {
    switch (reg) {
    case 0x00: // Port A - output bits reflect written value, input bits = 0xFF
        return (cia->port_a & cia->ddr_a) | (~cia->ddr_a & 0xFF);
    case 0x01: // Port B
        if (is_cia1) {
            // CIA1 Port B: keyboard row input via matrix scanning
            // Software drives column select low on Port A, reads rows on Port B
            uint8_t col_select = cia->port_a | ~cia->ddr_a; // undriven bits = 1
            uint8_t result = 0xFF;
            for (int col = 0; col < 8; col++) {
                if (!(col_select & (1 << col))) {
                    // Column selected (driven low) - merge in row data
                    result &= s_emu.keyboard.matrix[col];
                }
            }
            return result;
        }
        return (cia->port_b & cia->ddr_b) | (~cia->ddr_b & 0xFF);
    case 0x02: return cia->ddr_a;
    case 0x03: return cia->ddr_b;
    case 0x04: return cia->timer_a & 0xFF;          // Timer A low
    case 0x05: return (cia->timer_a >> 8) & 0xFF;   // Timer A high
    case 0x06: return cia->timer_b & 0xFF;           // Timer B low
    case 0x07: return (cia->timer_b >> 8) & 0xFF;    // Timer B high
    case 0x08: return cia->tod_10ths;
    case 0x09: return cia->tod_sec;
    case 0x0A: return cia->tod_min;
    case 0x0B: return cia->tod_hr;
    case 0x0C: return 0; // Serial shift register
    case 0x0D: { // ICR - Interrupt Control Register (read clears)
        uint8_t val = cia->icr_data;
        cia->icr_data = 0;  // Reading clears
        return val;
    }
    case 0x0E: return cia->cra;
    case 0x0F: return cia->crb;
    }
    return 0;
}

static void cia_write(cia_t *cia, uint8_t reg, uint8_t val, bool is_cia1) {
    switch (reg) {
    case 0x00: cia->port_a = val; break;
    case 0x01: cia->port_b = val; break;
    case 0x02: cia->ddr_a = val; break;
    case 0x03: cia->ddr_b = val; break;
    case 0x04: cia->timer_a_latch = (cia->timer_a_latch & 0xFF00) | val; break;
    case 0x05:
        cia->timer_a_latch = (cia->timer_a_latch & 0x00FF) | ((uint16_t)val << 8);
        // If timer stopped, load latch into timer
        if (!(cia->cra & 0x01)) {
            cia->timer_a = cia->timer_a_latch;
        }
        break;
    case 0x06: cia->timer_b_latch = (cia->timer_b_latch & 0xFF00) | val; break;
    case 0x07:
        cia->timer_b_latch = (cia->timer_b_latch & 0x00FF) | ((uint16_t)val << 8);
        if (!(cia->crb & 0x01)) {
            cia->timer_b = cia->timer_b_latch;
        }
        break;
    case 0x08: cia->tod_10ths = val & 0x0F; break;
    case 0x09: cia->tod_sec = val & 0x7F; break;
    case 0x0A: cia->tod_min = val & 0x7F; break;
    case 0x0B: cia->tod_hr = val & 0x9F; break;
    case 0x0C: break; // Serial shift register
    case 0x0D: // ICR - Interrupt Control Register (write = mask)
        if (val & 0x80) {
            cia->icr_mask |= (val & 0x1F);  // SET bits
        } else {
            cia->icr_mask &= ~(val & 0x1F); // CLEAR bits
        }
        break;
    case 0x0E: // CRA
        if (val & 0x10) {
            // Force load: copy latch to timer
            cia->timer_a = cia->timer_a_latch;
            val &= ~0x10; // Clear force load bit
        }
        cia->cra = val;
        break;
    case 0x0F: // CRB
        if (val & 0x10) {
            cia->timer_b = cia->timer_b_latch;
            val &= ~0x10;
        }
        cia->crb = val;
        break;
    }
}

// Tick CIA timers by N cycles, return true if IRQ fired
static bool cia_tick(cia_t *cia, uint32_t cycles) {
    bool irq = false;
    bool timer_a_underflowed = false;

    // Timer A
    if (cia->cra & 0x01) { // Timer A running, counting system clocks
        uint32_t remaining = cycles;
        while (remaining > 0) {
            if (cia->timer_a <= remaining) {
                // Timer A underflow
                remaining -= cia->timer_a;
                cia->timer_a = cia->timer_a_latch; // Reload
                cia->icr_data |= 0x01; // Set TA underflow flag
                timer_a_underflowed = true;
                if (cia->icr_mask & 0x01) {
                    cia->icr_data |= 0x80;
                    irq = true;
                }
                if (cia->cra & 0x08) {
                    // One-shot: stop timer after underflow
                    cia->cra &= ~0x01;
                    break;
                }
                // Prevent infinite loop on latch=0
                if (cia->timer_a_latch == 0) break;
            } else {
                cia->timer_a -= remaining;
                remaining = 0;
            }
        }
    }

    // Timer B
    if (cia->crb & 0x01) { // Timer B running
        uint8_t tb_mode = (cia->crb >> 5) & 0x03;
        uint32_t tb_ticks;

        if (tb_mode == 0x02) {
            // Count Timer A underflows
            tb_ticks = timer_a_underflowed ? 1 : 0;
        } else {
            // Count system clocks (mode 0)
            tb_ticks = cycles;
        }

        if (tb_ticks > 0) {
            if (cia->timer_b <= tb_ticks) {
                cia->icr_data |= 0x02;
                cia->timer_b = cia->timer_b_latch;
                if (cia->icr_mask & 0x02) {
                    cia->icr_data |= 0x80;
                    irq = true;
                }
                if (cia->crb & 0x08) {
                    cia->crb &= ~0x01;
                }
            } else {
                cia->timer_b -= tb_ticks;
            }
        }
    }

    return irq;
}

// ============================================================================
// SID 6581 envelope rate lookup (cycles per step)
// ============================================================================
static const uint16_t sid_attack_rate[16] = {
    2, 8, 16, 24, 38, 56, 68, 80, 100, 250, 500, 800, 1000, 3000, 5000, 8000
};
static const uint16_t sid_decay_release_rate[16] = {
    6, 24, 48, 72, 114, 168, 204, 240, 300, 750, 1500, 2400, 3000, 9000, 15000, 24000
};

// Tick SID envelope for one voice
static void sid_tick_envelope(sid_voice_t *v) {
    uint16_t rate;

    switch (v->env_state) {
    case 1: // Attack
        rate = sid_attack_rate[v->ad >> 4];
        v->env_counter++;
        if (v->env_counter >= rate) {
            v->env_counter = 0;
            if (v->env_level < 255) {
                v->env_level++;
            }
            if (v->env_level >= 255) {
                v->env_level = 255;
                v->env_state = 2; // -> Decay
            }
        }
        break;
    case 2: // Decay
        rate = sid_decay_release_rate[v->ad & 0x0F];
        v->env_counter++;
        if (v->env_counter >= rate) {
            v->env_counter = 0;
            uint8_t sustain = (v->sr >> 4) * 17; // 0-15 -> 0-255
            if (v->env_level > sustain) {
                v->env_level--;
            }
            if (v->env_level <= sustain) {
                v->env_level = sustain;
                v->env_state = 3; // -> Sustain
            }
        }
        break;
    case 3: // Sustain - hold at sustain level
        v->env_level = (v->sr >> 4) * 17;
        break;
    case 4: // Release
        rate = sid_decay_release_rate[v->sr & 0x0F];
        v->env_counter++;
        if (v->env_counter >= rate) {
            v->env_counter = 0;
            if (v->env_level > 0) {
                v->env_level--;
            }
            if (v->env_level == 0) {
                v->env_state = 0; // -> Idle
            }
        }
        break;
    default: // Idle
        v->env_level = 0;
        break;
    }
}

// Generate one oscillator sample for a SID voice (returns signed 12-bit: -2048..+2047)
static int16_t sid_voice_output(sid_voice_t *v) {
    // Advance phase accumulator
    v->phase = (v->phase + v->freq) & 0x00FFFFFF;

    uint16_t waveform = 0;
    uint32_t msb12 = (v->phase >> 12) & 0xFFF; // Top 12 bits

    if (v->control & 0x80) {
        // Noise: use LFSR bits
        waveform = (uint16_t)(v->noise_lfsr & 0xFFF);
        // Clock LFSR when bit 19 transitions
        static uint32_t prev_bit19 = 0;
        uint32_t bit19 = v->phase & (1 << 19);
        if (bit19 && !prev_bit19) {
            // Galois LFSR: x^23 + x^18 + 1
            uint32_t feedback = ((v->noise_lfsr >> 22) ^ (v->noise_lfsr >> 17)) & 1;
            v->noise_lfsr = ((v->noise_lfsr << 1) | feedback) & 0x7FFFFF;
        }
        prev_bit19 = bit19;
    } else if (v->control & 0x40) {
        // Pulse wave
        uint16_t pw = v->pulse_width;
        waveform = (msb12 < pw) ? 0xFFF : 0x000;
    } else if (v->control & 0x20) {
        // Sawtooth
        waveform = msb12;
    } else if (v->control & 0x10) {
        // Triangle
        if (v->phase & (1 << 23)) {
            waveform = ~msb12 & 0xFFF;
        } else {
            waveform = msb12;
        }
    }

    // Apply envelope (waveform is 0-4095, envelope is 0-255)
    int32_t out = ((int32_t)waveform - 2048) * v->env_level / 256;
    return (int16_t)out;
}

// Update SID voice registers from reg writes
static void sid_update_voice(int voice_num) {
    sid_voice_t *v = &s_emu.sid.voice[voice_num];
    int base = voice_num * 7;
    v->freq = s_emu.sid_regs[base + 0] | ((uint16_t)s_emu.sid_regs[base + 1] << 8);
    v->pulse_width = (s_emu.sid_regs[base + 2] | ((uint16_t)(s_emu.sid_regs[base + 3] & 0x0F) << 8));

    uint8_t old_control = v->control;
    v->control = s_emu.sid_regs[base + 4];

    // Gate edge detection
    if ((v->control & 0x01) && !(old_control & 0x01)) {
        // Gate on: start attack
        v->env_state = 1;
        v->env_counter = 0;
    } else if (!(v->control & 0x01) && (old_control & 0x01)) {
        // Gate off: start release
        v->env_state = 4;
        v->env_counter = 0;
    }

    v->ad = s_emu.sid_regs[base + 5];
    v->sr = s_emu.sid_regs[base + 6];
}

// ============================================================================
// C64 I/O read/write - VIC-II, SID, CIA, Color RAM
// ============================================================================

static uint8_t c64_io_read(uint16_t addr) {
    if (addr >= C64_VIC_BASE && addr < C64_VIC_BASE + 0x400) {
        uint8_t reg = addr & 0x3F;
        switch (reg) {
        case 0x11: // Control register 1 + raster bit 8
            return (s_emu.vic_regs[0x11] & 0x7F) | ((s_emu.raster_line & 0x100) ? 0x80 : 0);
        case 0x12: // Raster counter (low 8 bits)
            return s_emu.raster_line & 0xFF;
        case 0x19: // IRQ status
            return s_emu.vic_irq_status | 0x70; // Unused bits read as 1
        case 0x1A: // IRQ enable
            return s_emu.vic_irq_enable | 0xF0;
        default:
            return s_emu.vic_regs[reg];
        }
    }
    if (addr >= C64_SID_BASE && addr < C64_SID_BASE + 0x400) {
        uint8_t reg = addr & 0x1F;
        if (reg == 0x1B) {
            // OSC3 output: return top 8 bits of voice 3 oscillator
            return (uint8_t)((s_emu.sid.voice[2].phase >> 16) & 0xFF);
        }
        if (reg == 0x1C) {
            // ENV3 output: return voice 3 envelope level
            return s_emu.sid.voice[2].env_level;
        }
        // Write-only registers read as 0
        if (reg < 0x19) return 0;
        return s_emu.sid_regs[reg];
    }
    if (addr >= C64_COLOR_RAM && addr < C64_COLOR_RAM + 0x400) {
        return s_emu.color_ram[addr - C64_COLOR_RAM] | 0xF0; // Upper nibble undefined
    }
    if (addr >= C64_CIA1_BASE && addr < C64_CIA1_BASE + 0x100) {
        return cia_read(&s_emu.cia1, addr & 0x0F, true);
    }
    if (addr >= C64_CIA2_BASE && addr < C64_CIA2_BASE + 0x100) {
        return cia_read(&s_emu.cia2, addr & 0x0F, false);
    }
    return 0xFF;
}

static void c64_io_write(uint16_t addr, uint8_t val) {
    if (addr >= C64_VIC_BASE && addr < C64_VIC_BASE + 0x400) {
        uint8_t reg = addr & 0x3F;
        s_emu.vic_regs[reg] = val;
        if (reg == 0x19) {
            // Writing 1 bits clears IRQ flags
            s_emu.vic_irq_status &= ~(val & 0x0F);
            if (!(s_emu.vic_irq_status & s_emu.vic_irq_enable & 0x0F)) {
                s_emu.vic_irq_status &= ~0x80; // Clear master IRQ bit
            }
        } else if (reg == 0x1A) {
            s_emu.vic_irq_enable = val & 0x0F;
        }
        return;
    }
    if (addr >= C64_SID_BASE && addr < C64_SID_BASE + 0x400) {
        uint8_t reg = addr & 0x1F;
        s_emu.sid_regs[reg] = val;
        // Update voice state when voice registers change
        if (reg < 7) sid_update_voice(0);
        else if (reg < 14) sid_update_voice(1);
        else if (reg < 21) sid_update_voice(2);
        else if (reg == 0x18) {
            s_emu.sid.volume = val & 0x0F;
        }
        return;
    }
    if (addr >= C64_COLOR_RAM && addr < C64_COLOR_RAM + 0x400) {
        s_emu.color_ram[addr - C64_COLOR_RAM] = val & 0x0F;
        return;
    }
    if (addr >= C64_CIA1_BASE && addr < C64_CIA1_BASE + 0x100) {
        cia_write(&s_emu.cia1, addr & 0x0F, val, true);
        return;
    }
    if (addr >= C64_CIA2_BASE && addr < C64_CIA2_BASE + 0x100) {
        cia_write(&s_emu.cia2, addr & 0x0F, val, false);
        return;
    }
}

// ============================================================================
// VIC-20 I/O read/write
// ============================================================================

static uint8_t vic20_io_read(uint16_t addr) {
    if (addr >= VIC20_VIC_BASE && addr < VIC20_VIC_BASE + 0x10) {
        return s_emu.vic20_vic_regs[addr & 0x0F];
    }
    if (addr >= VIC20_VIA1_BASE && addr < VIC20_VIA1_BASE + 0x10) {
        uint8_t reg = addr & 0x0F;
        if (reg == 0x01) return 0xFF; // VIA1 Port A: keyboard
        if (reg == 0x00) return 0xFF; // VIA1 Port B
        if (reg == 0x0D) return 0x00; // IFR: no interrupts
        return s_emu.vic20_via1[reg];
    }
    if (addr >= VIC20_VIA2_BASE && addr < VIC20_VIA2_BASE + 0x10) {
        uint8_t reg = addr & 0x0F;
        if (reg == 0x00) return 0xFF; // VIA2 Port B: keyboard
        if (reg == 0x0D) return 0x00; // IFR
        return s_emu.vic20_via2[reg];
    }
    return 0xFF;
}

static void vic20_io_write(uint16_t addr, uint8_t val) {
    if (addr >= VIC20_VIC_BASE && addr < VIC20_VIC_BASE + 0x10) {
        s_emu.vic20_vic_regs[addr & 0x0F] = val;
        return;
    }
    if (addr >= VIC20_VIA1_BASE && addr < VIC20_VIA1_BASE + 0x10) {
        s_emu.vic20_via1[addr & 0x0F] = val;
        return;
    }
    if (addr >= VIC20_VIA2_BASE && addr < VIC20_VIA2_BASE + 0x10) {
        s_emu.vic20_via2[addr & 0x0F] = val;
        return;
    }
}

// ============================================================================
// C64 Screen RAM rendering - reads $0400 and outputs to console/LCD
// Uses ANSI escape codes for stable in-place serial terminal output
// ============================================================================

// C64 screen code to ASCII conversion table
static char screen_code_to_ascii(uint8_t sc) {
    sc &= 0x7F; // Strip reverse bit
    if (sc == 0x00) return '@';
    if (sc >= 0x01 && sc <= 0x1A) return 'A' + (sc - 1);
    if (sc >= 0x1B && sc <= 0x1F) return "[\\]^_"[sc - 0x1B];
    if (sc == 0x20) return ' ';
    if (sc >= 0x21 && sc <= 0x3F) return '!' + (sc - 0x21);
    // Graphics characters → period as placeholder so screen geometry stays visible
    if (sc >= 0x40 && sc <= 0x5F) return '.';
    if (sc >= 0x60 && sc <= 0x7F) return '.';
    return '.';
}

// Build one screen row into buf (exactly 40 chars + NUL). Returns trimmed length.
static int c64_build_row(int row, char *buf) {
    uint16_t base = 0x0400 + row * 40;
    for (int col = 0; col < 40; col++) {
        buf[col] = screen_code_to_ascii(s_emu.memory[base + col]);
    }
    buf[40] = '\0';
    // Trim trailing spaces for cleaner serial output
    int len = 40;
    while (len > 0 && buf[len - 1] == ' ') len--;
    buf[len] = '\0';
    return len;
}

// ============================================================================
// GPU-accelerated C64 screen rendering
// Renders directly into the video card's back buffer as RGB565.
// The video card handles double-buffering and tear-free LCD output.
// ============================================================================

static void c64_render_to_gpu(void) {
    if (!s_emu.memory || !s_emu.chargen_rom) return;
    if (!video_card_is_initialized()) return;

    uint16_t *fb = video_card_get_back_buffer();
    if (!fb) return;

    uint16_t fb_w = video_card_get_width();
    uint16_t fb_h = video_card_get_height();

    uint8_t d011 = s_emu.vic_regs[0x11];
    uint8_t d016 = s_emu.vic_regs[0x16];
    uint8_t d018 = s_emu.vic_regs[0x18];
    bool bmm  = (d011 & 0x20) != 0; // Bitmap mode
    bool ecm  = (d011 & 0x40) != 0; // Extended color mode
    bool mcm  = (d016 & 0x10) != 0; // Multicolor mode
    bool den  = (d011 & 0x10) != 0; // Display enable

    uint16_t border_c = c64_colors_rgb565[s_emu.vic_regs[0x20] & 0x0F];
    uint16_t bg0 = c64_colors_rgb565[s_emu.vic_regs[0x21] & 0x0F];
    uint16_t bg1 = c64_colors_rgb565[s_emu.vic_regs[0x22] & 0x0F];
    uint16_t bg2 = c64_colors_rgb565[s_emu.vic_regs[0x23] & 0x0F];

    // CIA2 Port A bits 0-1 select VIC bank (inverted)
    uint16_t vic_bank = (~s_emu.cia2.port_a & 0x03) * 0x4000;
    uint16_t screen_addr = vic_bank + ((d018 >> 4) & 0x0F) * 0x0400;
    uint16_t char_addr = vic_bank + ((d018 >> 1) & 0x07) * 0x0800;
    uint16_t bitmap_addr = vic_bank + ((d018 >> 3) & 0x01) * 0x2000;

    // Scroll offsets
    int scroll_x = d016 & 0x07;
    int scroll_y = d011 & 0x07;
    bool col38 = !(d016 & 0x08); // 38-column mode
    bool row24 = !(d011 & 0x08); // 24-row mode
    (void)scroll_x; (void)scroll_y; // Used for fine scroll if needed
    (void)col38; (void)row24;

    // C64 screen (320x200) centered on LCD (480x320)
    int x_off = (fb_w - 320) / 2;   // 80
    int y_off = (fb_h - 200) / 2;   // 60

    // --- Fill entire framebuffer with border ---
    for (int y = 0; y < y_off; y++) {
        uint16_t *row = &fb[y * fb_w];
        for (int x = 0; x < fb_w; x++) row[x] = border_c;
    }
    for (int y = y_off + 200; y < fb_h; y++) {
        uint16_t *row = &fb[y * fb_w];
        for (int x = 0; x < fb_w; x++) row[x] = border_c;
    }

    if (!den) {
        // Display disabled - fill content area with border too
        for (int y = y_off; y < y_off + 200; y++) {
            uint16_t *row = &fb[y * fb_w];
            for (int x = 0; x < fb_w; x++) row[x] = border_c;
        }
        return;
    }

    // --- Render content area ---
    for (int crow = 0; crow < 25; crow++) {
        for (int cline = 0; cline < 8; cline++) {
            int y = y_off + crow * 8 + cline;
            uint16_t *line = &fb[y * fb_w];

            // Side borders
            for (int x = 0; x < x_off; x++) line[x] = border_c;
            for (int x = x_off + 320; x < fb_w; x++) line[x] = border_c;

            if (bmm && !mcm) {
                // --- Standard Bitmap Mode (320x200, 2 colors per 8x8 cell) ---
                for (int col = 0; col < 40; col++) {
                    uint16_t si = crow * 40 + col;
                    uint8_t bitmap_byte = s_emu.memory[(bitmap_addr + si * 8 + cline) & 0xFFFF];
                    uint8_t screen_byte = s_emu.memory[(screen_addr + si) & 0xFFFF];
                    uint16_t fg = c64_colors_rgb565[(screen_byte >> 4) & 0x0F];
                    uint16_t bg = c64_colors_rgb565[screen_byte & 0x0F];
                    int px = x_off + col * 8;
                    for (int bit = 0; bit < 8; bit++) {
                        line[px + bit] = (bitmap_byte & (0x80 >> bit)) ? fg : bg;
                    }
                }
            } else if (bmm && mcm) {
                // --- Multicolor Bitmap Mode (160x200 effective, 4 colors per 4x8 cell) ---
                for (int col = 0; col < 40; col++) {
                    uint16_t si = crow * 40 + col;
                    uint8_t bitmap_byte = s_emu.memory[(bitmap_addr + si * 8 + cline) & 0xFFFF];
                    uint8_t screen_byte = s_emu.memory[(screen_addr + si) & 0xFFFF];
                    uint8_t color_byte = s_emu.color_ram[si] & 0x0F;
                    uint16_t colors[4] = {
                        bg0,
                        c64_colors_rgb565[(screen_byte >> 4) & 0x0F],
                        c64_colors_rgb565[screen_byte & 0x0F],
                        c64_colors_rgb565[color_byte]
                    };
                    int px = x_off + col * 8;
                    for (int dp = 0; dp < 4; dp++) {
                        uint8_t idx = (bitmap_byte >> (6 - dp * 2)) & 0x03;
                        line[px + dp * 2 + 0] = colors[idx];
                        line[px + dp * 2 + 1] = colors[idx]; // Double-wide pixels
                    }
                }
            } else if (mcm && !bmm) {
                // --- Multicolor Character Mode (160x200 effective) ---
                for (int col = 0; col < 40; col++) {
                    uint16_t si = crow * 40 + col;
                    uint8_t sc = s_emu.memory[(screen_addr + si) & 0xFFFF];
                    uint8_t color_nib = s_emu.color_ram[si] & 0x0F;
                    int px = x_off + col * 8;

                    // Determine character data source
                    uint8_t glyph;
                    bool is_char_rom = (char_addr >= vic_bank + 0x1000 && char_addr < vic_bank + 0x2000);
                    if (is_char_rom && s_emu.chargen_rom) {
                        glyph = s_emu.chargen_rom[sc * 8 + cline];
                    } else {
                        glyph = s_emu.memory[(char_addr + sc * 8 + cline) & 0xFFFF];
                    }

                    if (color_nib & 0x08) {
                        // Multicolor: each 2-bit pair selects a color
                        uint16_t colors[4] = {
                            bg0, bg1, bg2,
                            c64_colors_rgb565[color_nib & 0x07]
                        };
                        for (int dp = 0; dp < 4; dp++) {
                            uint8_t idx = (glyph >> (6 - dp * 2)) & 0x03;
                            line[px + dp * 2 + 0] = colors[idx];
                            line[px + dp * 2 + 1] = colors[idx];
                        }
                    } else {
                        // Standard hires for this character
                        uint16_t fg = c64_colors_rgb565[color_nib];
                        for (int bit = 0; bit < 8; bit++) {
                            line[px + bit] = (glyph & (0x80 >> bit)) ? fg : bg0;
                        }
                    }
                }
            } else if (ecm && !bmm && !mcm) {
                // --- Extended Color Mode (bg color from top 2 bits of screen code) ---
                for (int col = 0; col < 40; col++) {
                    uint16_t si = crow * 40 + col;
                    uint8_t sc = s_emu.memory[(screen_addr + si) & 0xFFFF];
                    uint16_t fg = c64_colors_rgb565[s_emu.color_ram[si] & 0x0F];
                    uint8_t bg_sel = (sc >> 6) & 0x03;
                    uint16_t bg_ecm = c64_colors_rgb565[s_emu.vic_regs[0x21 + bg_sel] & 0x0F];
                    uint8_t glyph = s_emu.chargen_rom[(sc & 0x3F) * 8 + cline];
                    int px = x_off + col * 8;
                    for (int bit = 0; bit < 8; bit++) {
                        line[px + bit] = (glyph & (0x80 >> bit)) ? fg : bg_ecm;
                    }
                }
            } else {
                // --- Standard Character Mode (default) ---
                for (int col = 0; col < 40; col++) {
                    uint16_t si = crow * 40 + col;
                    uint8_t sc = s_emu.memory[(screen_addr + si) & 0xFFFF];
                    uint16_t fg = c64_colors_rgb565[s_emu.color_ram[si] & 0x0F];

                    // Determine character data source
                    uint8_t glyph;
                    bool is_char_rom = (char_addr >= vic_bank + 0x1000 && char_addr < vic_bank + 0x2000);
                    if (is_char_rom && s_emu.chargen_rom) {
                        glyph = s_emu.chargen_rom[sc * 8 + cline];
                    } else {
                        glyph = s_emu.memory[(char_addr + sc * 8 + cline) & 0xFFFF];
                    }

                    int px = x_off + col * 8;
                    line[px + 0] = (glyph & 0x80) ? fg : bg0;
                    line[px + 1] = (glyph & 0x40) ? fg : bg0;
                    line[px + 2] = (glyph & 0x20) ? fg : bg0;
                    line[px + 3] = (glyph & 0x10) ? fg : bg0;
                    line[px + 4] = (glyph & 0x08) ? fg : bg0;
                    line[px + 5] = (glyph & 0x04) ? fg : bg0;
                    line[px + 6] = (glyph & 0x02) ? fg : bg0;
                    line[px + 7] = (glyph & 0x01) ? fg : bg0;
                }
            }
        }
    }

    // --- Render sprites (after background, in priority order) ---
    uint8_t sprite_enable = s_emu.vic_regs[0x15];
    if (sprite_enable) {
        uint8_t sprite_mc = s_emu.vic_regs[0x1C];       // Multicolor flags
        uint8_t sprite_xe = s_emu.vic_regs[0x1D];       // X-expand
        uint8_t sprite_ye = s_emu.vic_regs[0x1E];       // Y-expand
        uint8_t sprite_pri = s_emu.vic_regs[0x1B];      // Priority (0=in front, 1=behind bg)
        uint8_t msb_x = s_emu.vic_regs[0x10];           // MSB of X coordinates
        uint16_t sprite_mc0 = c64_colors_rgb565[s_emu.vic_regs[0x25] & 0x0F]; // MC color 0
        uint16_t sprite_mc1 = c64_colors_rgb565[s_emu.vic_regs[0x26] & 0x0F]; // MC color 1
        __attribute__((unused)) uint8_t sprite_ptrs_offset = (d018 >> 4) * 0x400 + 0x3F8;

        // Render sprites back-to-front (sprite 7 first, sprite 0 last = highest priority)
        for (int spr = 7; spr >= 0; spr--) {
            if (!(sprite_enable & (1 << spr))) continue;

            int sx = s_emu.vic_regs[spr * 2] | ((msb_x & (1 << spr)) ? 256 : 0);
            int sy = s_emu.vic_regs[spr * 2 + 1];
            sx -= 24; // C64 sprite coordinate offset
            sy -= 50;

            uint8_t sprite_ptr = s_emu.memory[(screen_addr + 0x3F8 + spr) & 0xFFFF];
            uint16_t data_addr = vic_bank + sprite_ptr * 64;
            uint16_t sprite_color = c64_colors_rgb565[s_emu.vic_regs[0x27 + spr] & 0x0F];
            bool x_expand = (sprite_xe & (1 << spr)) != 0;
            bool y_expand = (sprite_ye & (1 << spr)) != 0;
            bool is_mc = (sprite_mc & (1 << spr)) != 0;
            bool behind_bg = (sprite_pri & (1 << spr)) != 0;

            int spr_h = y_expand ? 42 : 21;
            int spr_w = x_expand ? 48 : 24;

            for (int dy = 0; dy < spr_h; dy++) {
                int src_y = y_expand ? (dy / 2) : dy;
                int screen_y = sy + dy;
                if (screen_y < 0 || screen_y >= 200) continue;

                int fb_y = y_off + screen_y;
                uint16_t *line = &fb[fb_y * fb_w];

                for (int dx = 0; dx < spr_w; dx++) {
                    int src_x = x_expand ? (dx / 2) : dx;
                    int screen_x = sx + dx;
                    if (screen_x < 0 || screen_x >= 320) continue;
                    int fb_x = x_off + screen_x;

                    // Read sprite data (3 bytes per row, 21 rows)
                    int byte_idx = src_y * 3 + (src_x / 8);
                    int bit_idx = 7 - (src_x % 8);
                    uint8_t sprite_byte = s_emu.memory[(data_addr + byte_idx) & 0xFFFF];

                    if (is_mc) {
                        // Multicolor sprite: 2 bits per pixel (12 pixels wide)
                        (void)0; // pair_bit calculation below uses src_x directly
                        int pair_byte = src_y * 3 + (src_x / 8);
                        uint8_t b = s_emu.memory[(data_addr + pair_byte) & 0xFFFF];
                        int shift = 6 - ((src_x & 6));
                        uint8_t color_idx = (b >> shift) & 0x03;
                        if (color_idx == 0) continue; // Transparent
                        uint16_t pixel;
                        if (color_idx == 1) pixel = sprite_mc0;
                        else if (color_idx == 3) pixel = sprite_mc1;
                        else pixel = sprite_color;
                        if (!behind_bg || line[fb_x] == bg0) {
                            line[fb_x] = pixel;
                        }
                    } else {
                        // Standard sprite: 1 bit per pixel
                        if (sprite_byte & (1 << bit_idx)) {
                            if (!behind_bg || line[fb_x] == bg0) {
                                line[fb_x] = sprite_color;
                            }
                        }
                    }
                }
            }
        }
    }
}

// Render C64 screen: GPU framebuffer + UART serial terminal
// GPU path: renders to video card back buffer, then presents (vsync'd flip)
// UART path: diff-based ANSI escape code output for serial monitoring
static void c64_render_screen(void) {
    if (!s_emu.memory) return;

    uint16_t screen_base = 0x0400;

    // GPU path: render to video card back buffer and present
    c64_render_to_gpu();
    video_card_present();

    // Quick check: anything changed at all?
    bool any_change = false;
    for (int i = 0; i < 1000; i++) {
        if (s_emu.memory[screen_base + i] != s_emu.prev_screen[i]) {
            any_change = true;
            break;
        }
    }
    if (!any_change && s_emu.frame_count > 0) return;

    char line_buf[42];

    // First frame: clear screen and draw all 25 rows at ANSI rows 1-25
    if (s_emu.frame_count == 0) {
        // ANSI: clear screen + hide cursor during draw
        printf("\033[2J\033[H\033[?25l");
        for (int row = 0; row < 25; row++) {
            c64_build_row(row, line_buf);
            printf("\033[%d;1H%-40s", 1 + row, line_buf);
        }
        printf("\033[?25h");  // Show cursor again
        fflush(stdout);
        memcpy(s_emu.prev_screen, &s_emu.memory[screen_base], 1000);
        s_emu.frame_count++;
        return;
    }

    // Subsequent frames: only update changed rows
    for (int row = 0; row < 25; row++) {
        bool row_changed = false;
        for (int col = 0; col < 40; col++) {
            int idx = row * 40 + col;
            if (s_emu.memory[screen_base + idx] != s_emu.prev_screen[idx]) {
                row_changed = true;
                break;
            }
        }
        if (!row_changed) continue;

        // Build new row content
        for (int col = 0; col < 40; col++) {
            int idx = row * 40 + col;
            line_buf[col] = screen_code_to_ascii(s_emu.memory[screen_base + idx]);
        }
        line_buf[40] = '\0';

        // ANSI: move cursor to this row's position and overwrite
        printf("\033[%d;1H%-40s", 1 + row, line_buf);
    }

    // Update tracking buffer
    memcpy(s_emu.prev_screen, &s_emu.memory[screen_base], 1000);
    s_emu.frame_count++;

    // Position terminal cursor at C64 cursor location for typing feedback
    uint8_t c64_cursor_col = s_emu.memory[0x00D3];
    uint8_t c64_cursor_row = s_emu.memory[0x00D6];
    if (c64_cursor_col < 40 && c64_cursor_row < 25) {
        printf("\033[%d;%dH", 1 + c64_cursor_row, 1 + c64_cursor_col);
    }
    fflush(stdout);
}

// ============================================================================
// C64 hardware tick - advance raster, tick CIAs, generate IRQs
// Call this after executing a batch of CPU cycles
// ============================================================================

void mos6502_emu_tick(uint32_t cpu_cycles) {
    if (s_emu.mode != MOS6502_MODE_C64) return;

    // Advance raster line (PAL: 312 lines, ~63 cycles per line)
    static uint32_t raster_accum = 0;
    raster_accum += cpu_cycles;
    while (raster_accum >= 63) {
        raster_accum -= 63;
        s_emu.raster_line++;
        if (s_emu.raster_line >= 312) {
            s_emu.raster_line = 0;
            s_emu.screen_dirty = true;
        }

        // Check for raster IRQ
        uint16_t raster_compare = s_emu.vic_regs[0x12] | ((s_emu.vic_regs[0x11] & 0x80) << 1);
        if (s_emu.raster_line == raster_compare && (s_emu.vic_irq_enable & 0x01)) {
            s_emu.vic_irq_status |= 0x01; // Raster IRQ flag
            s_emu.vic_irq_status |= 0x80; // Master IRQ flag
            mos6502_irq(&s_emu.cpu, true);
        }
    }

    // Tick CIA1 (generates IRQ for keyboard scan, jiffy clock)
    if (cia_tick(&s_emu.cia1, cpu_cycles)) {
        mos6502_irq(&s_emu.cpu, true);
    }

    // Tick CIA2 (generates NMI)
    if (cia_tick(&s_emu.cia2, cpu_cycles)) {
        mos6502_nmi(&s_emu.cpu);
    }

    // Clear IRQ line if no active sources
    if (!(s_emu.vic_irq_status & 0x80) && !(s_emu.cia1.icr_data & 0x80)) {
        mos6502_irq(&s_emu.cpu, false);
    }

    // TOD clock update (~100ms intervals = ~98525 cycles at 985248 Hz)
    static uint32_t tod_accum = 0;
    tod_accum += cpu_cycles;
    if (tod_accum >= 98525) {
        tod_accum -= 98525;
        // Increment CIA1 TOD (BCD)
        s_emu.cia1.tod_10ths++;
        if (s_emu.cia1.tod_10ths >= 10) {
            s_emu.cia1.tod_10ths = 0;
            uint8_t sec = s_emu.cia1.tod_sec;
            sec = ((sec >> 4) * 10 + (sec & 0x0F)) + 1;
            if (sec >= 60) sec = 0;
            s_emu.cia1.tod_sec = ((sec / 10) << 4) | (sec % 10);
        }
        // CIA2 TOD same
        s_emu.cia2.tod_10ths++;
        if (s_emu.cia2.tod_10ths >= 10) {
            s_emu.cia2.tod_10ths = 0;
            uint8_t sec = s_emu.cia2.tod_sec;
            sec = ((sec >> 4) * 10 + (sec & 0x0F)) + 1;
            if (sec >= 60) sec = 0;
            s_emu.cia2.tod_sec = ((sec / 10) << 4) | (sec % 10);
        }
    }

    // Keyboard release timer
    if (s_emu.keyboard.release_timer > 0) {
        s_emu.keyboard.release_timer -= (int32_t)cpu_cycles;
        if (s_emu.keyboard.release_timer <= 0) {
            s_emu.keyboard.release_timer = 0;
            c64_matrix_release_all();
        }
    }

    // SID envelope tick (one tick per CPU cycle for each active voice)
    for (int i = 0; i < 3; i++) {
        if (s_emu.sid.voice[i].env_state > 0) {
            sid_tick_envelope(&s_emu.sid.voice[i]);
        }
    }

    // SID audio sample generation
    // Accumulate cycles and generate samples at SID_SAMPLE_RATE
    s_emu.sid.sample_accum += cpu_cycles;
    uint32_t cycles_per_sample = SID_CLOCK_PAL / SID_SAMPLE_RATE; // ~44
    while (s_emu.sid.sample_accum >= cycles_per_sample) {
        s_emu.sid.sample_accum -= cycles_per_sample;

        // Mix all 3 voices
        int32_t mixed = 0;
        for (int i = 0; i < 3; i++) {
            if (s_emu.sid.voice[i].env_state > 0 || s_emu.sid.voice[i].env_level > 0) {
                mixed += sid_voice_output(&s_emu.sid.voice[i]);
            }
        }

        // Apply master volume (0-15) and clamp
        mixed = (mixed * s_emu.sid.volume) / 15;
        if (mixed > 2047) mixed = 2047;
        if (mixed < -2048) mixed = -2048;

        // Convert to unsigned 8-bit for audio bus output
        uint8_t sample = (uint8_t)((mixed + 2048) >> 4); // 12-bit → 8-bit

        // Write sample to audio bus device buffer (if available)
        // The bus audio device at BUS_DEV_AUDIO has a ring buffer at offset 0x200
        static uint16_t audio_write_pos = 0;
        bus_io_write(BUS_IO_BASE + BUS_DEV_AUDIO + AUD_BUFFER_OFFSET + audio_write_pos, sample, 1);
        audio_write_pos = (audio_write_pos + 1) % AUD_BUFFER_SIZE;
    }
}

// ============================================================================
// Poll keyboard input from UART and inject into C64 KERNAL keyboard buffer
// Also handles Ctrl+C to stop emulator
// ============================================================================

// Flag set when user presses Ctrl+C to exit C64
static volatile bool s_exit_requested = false;

bool mos6502_emu_exit_requested(void) {
    return s_exit_requested;
}

// Convert ASCII character to PETSCII for C64 KERNAL keyboard buffer injection
static uint8_t ascii_to_petscii(uint8_t ch) {
    if (ch == '\r' || ch == '\n') return 0x0D;  // RETURN
    if (ch == 0x08 || ch == 0x7F) return 0x14;     // DEL (backspace)
    if (ch >= 'A' && ch <= 'Z') return ch;         // Uppercase → same in PETSCII
    if (ch >= 'a' && ch <= 'z') return ch - 0x20;  // Lowercase → uppercase PETSCII
    if (ch >= 0x20 && ch <= 0x3F) return ch;        // Space, digits, punctuation
    if (ch >= 0x40 && ch <= 0x5F) return ch;        // @, [, \, ], ^, _
    return ch;  // Pass through other characters
}

// Inject a PETSCII key into the C64 KERNAL keyboard buffer
// Buffer is at $0277-$0286 (10 bytes), count at $00C6
static void c64_inject_key(uint8_t petscii) {
    uint8_t count = s_emu.memory[0x00C6];
    if (count < 10) {  // Buffer max is 10 characters
        s_emu.memory[0x0277 + count] = petscii;
        s_emu.memory[0x00C6] = count + 1;
    }
}

static void c64_poll_keyboard(void) {
    // Process up to 4 characters per poll to avoid flooding the buffer
    int processed = 0;

    while (processed < 4) {
        uint8_t ch = 0;

        // 1. Check PS/2 keyboard first (physical keyboard)
        if (ch == 0 && ps2_keyboard_is_initialized() && ps2_keyboard_available()) {
            ch = ps2_keyboard_read();
        }

        // 2. Check UART serial console
        if (ch == 0) {
            size_t uart_avail = 0;
            uart_get_buffered_data_len(CONFIG_ESP_CONSOLE_UART_NUM, &uart_avail);
            if (uart_avail > 0) {
                uint8_t buf;
                if (uart_read_bytes(CONFIG_ESP_CONSOLE_UART_NUM, &buf, 1, 0) == 1) {
                    ch = buf;
                }
            }
        }

        // No input from any source
        if (ch == 0) break;

        processed++;

        // Ctrl+C (0x03) or ESC = exit C64 back to BIOS
        if (ch == 0x03 || ch == PS2_KEY_ESCAPE) {
            s_exit_requested = true;
            return;
        }

        // Map PS/2 special keys to C64 PETSCII
        uint8_t petscii = 0;
        switch (ch) {
        case PS2_KEY_UP:       petscii = 0x91; break;  // Cursor up
        case PS2_KEY_DOWN:     petscii = 0x11; break;  // Cursor down
        case PS2_KEY_LEFT:     petscii = 0x9D; break;  // Cursor left
        case PS2_KEY_RIGHT:    petscii = 0x1D; break;  // Cursor right
        case PS2_KEY_HOME:     petscii = 0x13; break;  // HOME
        case PS2_KEY_DELETE:   petscii = 0x14; break;  // INST/DEL
        case PS2_KEY_INSERT:   petscii = 0x94; break;  // INSERT
        case PS2_KEY_F1:       petscii = 0x85; break;  // F1
        case PS2_KEY_F3:       petscii = 0x86; break;  // F3
        case PS2_KEY_F5:       petscii = 0x87; break;  // F5
        case PS2_KEY_F7:       petscii = 0x88; break;  // F7
        default:
            // Regular ASCII - convert to PETSCII
            petscii = ascii_to_petscii(ch);
            break;
        }

        if (petscii != 0) {
            c64_inject_key(petscii);
        }
    }
}

// ============================================================================
// Memory access callbacks - mode-aware routing
// ============================================================================

static uint8_t mem_read(void *ctx, uint16_t addr) {
    (void)ctx;

    switch (s_emu.mode) {
    case MOS6502_MODE_BARE:
        // Video card window ($C000-$CFFF)
        if (addr >= MOS6502_VIDEO_BASE && addr <= MOS6502_VIDEO_END) {
            uint32_t offset = addr - MOS6502_VIDEO_BASE;
            return (uint8_t)bus_io_read(BUS_IO_BASE + BUS_DEV_VIDEO + offset, 1);
        }
        // Bus I/O ($D000-$DFFF)
        if (addr >= MOS6502_IO_BASE && addr <= MOS6502_IO_END) {
            return bus_io_read_mapped(addr);
        }
        break;

    case MOS6502_MODE_C64:
        // C64 processor port
        if (addr == 0x0000) return s_emu.c64_port_dir;
        if (addr == 0x0001) return s_emu.c64_bank_cfg;

        // C64 I/O area ($D000-$DFFF) - depends on bank config
        if (addr >= 0xD000 && addr <= 0xDFFF) {
            uint8_t cfg = s_emu.c64_bank_cfg & 0x07;
            if (cfg == 0x05 || cfg == 0x06 || cfg == 0x07) {
                // I/O visible
                return c64_io_read(addr);
            }
            if (cfg == 0x01 || cfg == 0x02 || cfg == 0x03) {
                // Character ROM visible
                if (s_emu.chargen_rom) {
                    return s_emu.chargen_rom[addr - 0xD000];
                }
            }
            // RAM underneath
            return s_emu.memory[addr];
        }

        // BASIC ROM ($A000-$BFFF) - visible if bit 0 and bit 1 set
        if (addr >= 0xA000 && addr <= 0xBFFF) {
            if ((s_emu.c64_bank_cfg & 0x03) == 0x03) {
                return s_emu.memory[addr]; // ROM is loaded into memory[]
            }
            return s_emu.memory[addr]; // RAM
        }

        // KERNAL ROM ($E000-$FFFF) - visible if bit 1 set
        if (addr >= 0xE000) {
            if (s_emu.c64_bank_cfg & 0x02) {
                return s_emu.memory[addr]; // ROM loaded into memory[]
            }
            return s_emu.memory[addr]; // RAM
        }
        break;

    case MOS6502_MODE_VIC20:
        // VIC-20 I/O area ($9000-$93FF)
        if (addr >= 0x9000 && addr <= 0x93FF) {
            return vic20_io_read(addr);
        }
        // Character ROM ($8000-$8FFF)
        if (addr >= 0x8000 && addr <= 0x8FFF && s_emu.chargen_rom) {
            return s_emu.chargen_rom[addr - 0x8000];
        }
        break;
    }

    // Default: regular RAM/ROM
    return s_emu.memory[addr];
}

static void mem_write(void *ctx, uint16_t addr, uint8_t val) {
    (void)ctx;

    switch (s_emu.mode) {
    case MOS6502_MODE_BARE:
        // Video card window
        if (addr >= MOS6502_VIDEO_BASE && addr <= MOS6502_VIDEO_END) {
            uint32_t offset = addr - MOS6502_VIDEO_BASE;
            bus_io_write(BUS_IO_BASE + BUS_DEV_VIDEO + offset, val, 1);
            return;
        }
        // Bus I/O
        if (addr >= MOS6502_IO_BASE && addr <= MOS6502_IO_END) {
            bus_io_write_mapped(addr, val);
            return;
        }
        break;

    case MOS6502_MODE_C64:
        // Processor port
        if (addr == 0x0000) { s_emu.c64_port_dir = val; return; }
        if (addr == 0x0001) { s_emu.c64_bank_cfg = val; return; }

        // I/O area
        if (addr >= 0xD000 && addr <= 0xDFFF) {
            uint8_t cfg = s_emu.c64_bank_cfg & 0x07;
            if (cfg == 0x05 || cfg == 0x06 || cfg == 0x07) {
                c64_io_write(addr, val);
                return;
            }
        }

        // Don't allow writes to ROM areas ($A000-$BFFF, $E000-$FFFF)
        // when ROMs are banked in - writes go to underlying RAM
        break;

    case MOS6502_MODE_VIC20:
        // I/O area
        if (addr >= 0x9000 && addr <= 0x93FF) {
            vic20_io_write(addr, val);
            return;
        }
        // Don't write to ROM areas
        if (addr >= 0xC000) {
            return;  // ROM is read-only
        }
        break;
    }

    // Default: write to RAM
    s_emu.memory[addr] = val;
}

// ============================================================================
// Public API
// ============================================================================

esp_err_t mos6502_emu_init(void) {
    if (s_emu.initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    // Allocate 64KB - try SPIRAM first, fall back to internal
    s_emu.memory = heap_caps_calloc(1, MOS6502_MEM_SIZE, MALLOC_CAP_SPIRAM);
    if (!s_emu.memory) {
        ESP_LOGW(TAG, "SPIRAM allocation failed, trying internal RAM");
        s_emu.memory = heap_caps_calloc(1, MOS6502_MEM_SIZE, MALLOC_CAP_DEFAULT);
    }
    if (!s_emu.memory) {
        ESP_LOGE(TAG, "Failed to allocate 64KB for 6502 memory");
        return ESP_ERR_NO_MEM;
    }

    // Initialize CPU core
    mos6502_init(&s_emu.cpu);
    mos6502_set_callbacks(&s_emu.cpu, mem_read, mem_write, NULL);

    s_emu.initialized = true;
    ESP_LOGI(TAG, "MOS 6502 emulator initialized (64KB RAM, video via GPU)");
    return ESP_OK;
}

void mos6502_emu_reset(void) {
    if (!s_emu.initialized) return;
    mos6502_reset(&s_emu.cpu);
    ESP_LOGI(TAG, "CPU reset, PC=$%04X", s_emu.cpu.pc);
}

uint64_t mos6502_emu_run(uint32_t instructions) {
    if (!s_emu.initialized) return 0;
    uint64_t before = s_emu.cpu.cycles;
    mos6502_run(&s_emu.cpu, instructions);
    return s_emu.cpu.cycles - before;
}

void mos6502_emu_step(void) {
    if (!s_emu.initialized) return;
    mos6502_step(&s_emu.cpu);
}

void mos6502_emu_stop(void) {
    if (!s_emu.initialized) return;
    s_emu.cpu.halted = true;
}

bool mos6502_emu_is_halted(void) {
    if (!s_emu.initialized) return true;
    return s_emu.cpu.halted;
}

bool mos6502_emu_is_initialized(void) {
    return s_emu.initialized;
}

void mos6502_emu_get_state(char *buffer, size_t size) {
    if (!s_emu.initialized) {
        snprintf(buffer, size, "6502 emulator not initialized\n");
        return;
    }

    const char *mode_name = "Bare";
    if (s_emu.mode == MOS6502_MODE_C64) mode_name = "C64";
    else if (s_emu.mode == MOS6502_MODE_VIC20) mode_name = "VIC-20";

    mos6502_t *c = &s_emu.cpu;
    snprintf(buffer, size,
        "MOS 6502 CPU State [%s mode]:\n"
        "  PC=$%04X  SP=$%02X  A=$%02X  X=$%02X  Y=$%02X\n"
        "  Status=$%02X [%c%c-%c%c%c%c%c]\n"
        "  Cycles: %llu  Instructions: %llu\n"
        "  Halted: %s\n",
        mode_name,
        c->pc, c->sp, c->a, c->x, c->y,
        c->status,
        (c->status & MOS6502_FLAG_N) ? 'N' : '.',
        (c->status & MOS6502_FLAG_V) ? 'V' : '.',
        (c->status & MOS6502_FLAG_B) ? 'B' : '.',
        (c->status & MOS6502_FLAG_D) ? 'D' : '.',
        (c->status & MOS6502_FLAG_I) ? 'I' : '.',
        (c->status & MOS6502_FLAG_Z) ? 'Z' : '.',
        (c->status & MOS6502_FLAG_C) ? 'C' : '.',
        (unsigned long long)c->cycles,
        (unsigned long long)c->instructions,
        c->halted ? "yes" : "no");
}

void mos6502_emu_dump_memory(uint32_t addr, uint32_t length) {
    if (!s_emu.initialized) {
        console_printf("6502 emulator not initialized\n");
        return;
    }

    if (addr >= MOS6502_MEM_SIZE) {
        console_printf("Address $%04X out of range\n", addr);
        return;
    }
    if (addr + length > MOS6502_MEM_SIZE) {
        length = MOS6502_MEM_SIZE - addr;
    }

    for (uint32_t i = 0; i < length; i += 16) {
        console_printf("%04X: ", (uint16_t)(addr + i));
        // Hex
        for (int j = 0; j < 16 && (i + j) < length; j++) {
            console_printf("%02X ", s_emu.memory[addr + i + j]);
        }
        // Padding if last line is short
        for (int j = (int)(length - i); j < 16 && (length - i) < 16; j++) {
            console_printf("   ");
        }
        // ASCII
        console_printf(" |");
        for (int j = 0; j < 16 && (i + j) < length; j++) {
            uint8_t ch = s_emu.memory[addr + i + j];
            console_printf("%c", (ch >= 0x20 && ch < 0x7F) ? ch : '.');
        }
        console_printf("|\n");
    }
}

void mos6502_emu_load_program(const uint8_t *data, uint32_t size, uint32_t addr) {
    if (!s_emu.initialized) {
        ESP_LOGE(TAG, "Emulator not initialized");
        return;
    }

    if (addr + size > MOS6502_MEM_SIZE) {
        ESP_LOGW(TAG, "Program too large (addr=$%04X, size=%u, max=$FFFF)", addr, size);
        size = MOS6502_MEM_SIZE - addr;
    }

    memcpy(&s_emu.memory[addr], data, size);
    ESP_LOGI(TAG, "Loaded %u bytes at $%04X", size, (uint16_t)addr);
}

void mos6502_emu_destroy(void) {
    if (!s_emu.initialized) return;

    if (s_emu.memory) {
        heap_caps_free(s_emu.memory);
        s_emu.memory = NULL;
    }
    if (s_emu.chargen_rom) {
        free(s_emu.chargen_rom);
        s_emu.chargen_rom = NULL;
    }
    memset(&s_emu.cpu, 0, sizeof(s_emu.cpu));
    s_emu.mode = MOS6502_MODE_BARE;
    s_emu.initialized = false;
    ESP_LOGI(TAG, "Emulator destroyed");
}

uint8_t mos6502_emu_read_memory(uint16_t addr) {
    if (!s_emu.initialized) return 0xFF;
    return s_emu.memory[addr];
}

void mos6502_emu_write_memory(uint16_t addr, uint8_t val) {
    if (!s_emu.initialized) return;
    s_emu.memory[addr] = val;
}

void mos6502_emu_trigger_irq(void) {
    if (!s_emu.initialized) return;
    mos6502_irq(&s_emu.cpu, true);
}

void mos6502_emu_trigger_nmi(void) {
    if (!s_emu.initialized) return;
    mos6502_nmi(&s_emu.cpu);
}

void mos6502_emu_render_screen(void) {
    if (!s_emu.initialized) return;
    c64_render_screen();
}

void mos6502_emu_poll_keyboard(void) {
    if (!s_emu.initialized) return;
    c64_poll_keyboard();
}

// ============================================================================
// Machine mode support (C64, VIC-20, bare 6502)
// ============================================================================

mos6502_machine_mode_t mos6502_emu_get_mode(void) {
    return s_emu.mode;
}

esp_err_t mos6502_emu_init_mode(mos6502_machine_mode_t mode) {
    // Initialize base emulator first
    esp_err_t err = mos6502_emu_init();
    if (err != ESP_OK) return err;

    s_emu.mode = mode;
    s_exit_requested = false;  // Reset exit flag

    switch (mode) {
    case MOS6502_MODE_C64:
        // C64 defaults: all ROMs banked in, I/O visible
        s_emu.c64_port_dir = 0x2F;  // Default DDR
        s_emu.c64_bank_cfg = 0x37;  // BASIC + KERNAL + I/O visible
        memset(s_emu.vic_regs, 0, sizeof(s_emu.vic_regs));
        memset(s_emu.sid_regs, 0, sizeof(s_emu.sid_regs));
        memset(s_emu.color_ram, 0, sizeof(s_emu.color_ram));

        // VIC-II default register values
        s_emu.vic_regs[0x11] = 0x1B;  // Screen on, 25 rows, bitmap mode off
        s_emu.vic_regs[0x16] = 0xC8;  // 40 columns, multicolor off
        s_emu.vic_regs[0x18] = 0x15;  // Screen at $0400, charset at $1000
        s_emu.vic_regs[0x20] = 0x0E;  // Border color: light blue
        s_emu.vic_regs[0x21] = 0x06;  // Background: blue
        s_emu.raster_line = 0;
        s_emu.vic_irq_status = 0;
        s_emu.vic_irq_enable = 0;

        // CIA1 defaults (keyboard matrix, Timer A for 60Hz IRQ)
        memset(&s_emu.cia1, 0, sizeof(cia_t));
        s_emu.cia1.port_a = 0xFF;    // No keyboard row selected
        s_emu.cia1.port_b = 0xFF;    // No keys pressed
        s_emu.cia1.ddr_a  = 0xFF;    // Port A = output (row select)
        s_emu.cia1.ddr_b  = 0x00;    // Port B = input (column read)
        s_emu.cia1.timer_a_latch = 0x4025; // ~60Hz at ~1MHz (KERNAL sets this)
        s_emu.cia1.timer_a = 0x4025;
        s_emu.cia1.cra = 0x01;       // Timer A running, continuous mode
        s_emu.cia1.icr_mask = 0x01;  // Timer A IRQ enabled (KERNAL sets this)

        // CIA2 defaults (NMI source, VIC bank select)
        memset(&s_emu.cia2, 0, sizeof(cia_t));
        s_emu.cia2.port_a = 0x03;    // VIC bank 0 ($0000-$3FFF)
        s_emu.cia2.ddr_a  = 0x3F;    // DDRA

        // Keyboard state
        memset(&s_emu.keyboard, 0, sizeof(c64_keyboard_t));
        memset(s_emu.keyboard.matrix, 0xFF, sizeof(s_emu.keyboard.matrix));

        // Screen tracking
        memset(s_emu.prev_screen, 0, sizeof(s_emu.prev_screen));
        s_emu.screen_dirty = false;
        s_emu.frame_count = 0;

        // Fill screen RAM with spaces (screen code 0x20)
        memset(&s_emu.memory[0x0400], 0x20, 1000);
        // Fill color RAM with light blue (0x0E)
        memset(s_emu.color_ram, 0x0E, sizeof(s_emu.color_ram));

        ESP_LOGI(TAG, "C64 mode initialized (bank config=$%02X)", s_emu.c64_bank_cfg);
        break;

    case MOS6502_MODE_VIC20:
        memset(s_emu.vic20_via1, 0, sizeof(s_emu.vic20_via1));
        memset(s_emu.vic20_via2, 0, sizeof(s_emu.vic20_via2));
        memset(s_emu.vic20_vic_regs, 0, sizeof(s_emu.vic20_vic_regs));
        // VIC chip defaults
        s_emu.vic20_vic_regs[0x00] = 0x0C;  // Interlace, horizontal offset
        s_emu.vic20_vic_regs[0x01] = 0x26;  // Vertical offset
        s_emu.vic20_vic_regs[0x02] = 0x16;  // Columns (22)
        s_emu.vic20_vic_regs[0x03] = 0x2E;  // Rows (23), 8x8 chars
        s_emu.vic20_vic_regs[0x05] = 0xF0;  // Video matrix at $1E00, char at $8000
        s_emu.vic20_vic_regs[0x0E] = 0x00;  // Aux color
        s_emu.vic20_vic_regs[0x0F] = 0x1B;  // Border=white, bg=cyan
        ESP_LOGI(TAG, "VIC-20 mode initialized");
        break;

    case MOS6502_MODE_BARE:
    default:
        s_emu.mode = MOS6502_MODE_BARE;
        ESP_LOGI(TAG, "Bare 6502 mode initialized");
        break;
    }

    return ESP_OK;
}

esp_err_t mos6502_emu_load_roms(void) {
    if (!s_emu.initialized) {
        ESP_LOGE(TAG, "Emulator not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    const char *rom_dir = NULL;
    if (s_emu.mode == MOS6502_MODE_C64) {
        rom_dir = "/sdcard/c64";
    } else if (s_emu.mode == MOS6502_MODE_VIC20) {
        rom_dir = "/sdcard/vic20";
    } else {
        ESP_LOGI(TAG, "Bare mode: no ROMs to load");
        return ESP_OK;
    }

    char path[128];
    FILE *f;
    size_t bytes_read;

    // Load KERNAL ROM
    snprintf(path, sizeof(path), "%s/kernal.rom", rom_dir);
    f = fopen(path, "rb");
    if (f) {
        uint16_t kernal_addr = (s_emu.mode == MOS6502_MODE_C64) ?
                                C64_KERNAL_ROM_ADDR : VIC20_KERNAL_ROM_ADDR;
        uint16_t kernal_size = (s_emu.mode == MOS6502_MODE_C64) ?
                                C64_KERNAL_ROM_SIZE : VIC20_KERNAL_ROM_SIZE;
        bytes_read = fread(&s_emu.memory[kernal_addr], 1, kernal_size, f);
        fclose(f);
        ESP_LOGI(TAG, "Loaded KERNAL ROM: %u bytes at $%04X", (unsigned)bytes_read, kernal_addr);
    } else {
        ESP_LOGW(TAG, "KERNAL ROM not found: %s", path);
        console_printf("Warning: KERNAL ROM not found at %s\n", path);
    }

    // Load BASIC ROM
    snprintf(path, sizeof(path), "%s/basic.rom", rom_dir);
    f = fopen(path, "rb");
    if (f) {
        uint16_t basic_addr = (s_emu.mode == MOS6502_MODE_C64) ?
                               C64_BASIC_ROM_ADDR : VIC20_BASIC_ROM_ADDR;
        uint16_t basic_size = (s_emu.mode == MOS6502_MODE_C64) ?
                               C64_BASIC_ROM_SIZE : VIC20_BASIC_ROM_SIZE;
        bytes_read = fread(&s_emu.memory[basic_addr], 1, basic_size, f);
        fclose(f);
        ESP_LOGI(TAG, "Loaded BASIC ROM: %u bytes at $%04X", (unsigned)bytes_read, basic_addr);
    } else {
        ESP_LOGW(TAG, "BASIC ROM not found: %s", path);
        console_printf("Warning: BASIC ROM not found at %s\n", path);
    }

    // Load Character ROM (separate buffer, not mapped to main address space directly)
    snprintf(path, sizeof(path), "%s/chargen.rom", rom_dir);
    f = fopen(path, "rb");
    if (f) {
        uint16_t chargen_size = (s_emu.mode == MOS6502_MODE_C64) ?
                                 C64_CHARGEN_ROM_SIZE : VIC20_CHARGEN_ROM_SIZE;
        if (!s_emu.chargen_rom) {
            s_emu.chargen_rom = heap_caps_malloc(chargen_size, MALLOC_CAP_SPIRAM);
            if (!s_emu.chargen_rom) {
                s_emu.chargen_rom = malloc(chargen_size);
            }
        }
        if (s_emu.chargen_rom) {
            bytes_read = fread(s_emu.chargen_rom, 1, chargen_size, f);
            ESP_LOGI(TAG, "Loaded Character ROM: %u bytes", (unsigned)bytes_read);
        } else {
            ESP_LOGE(TAG, "Failed to allocate character ROM buffer");
        }
        fclose(f);
    } else {
        ESP_LOGW(TAG, "Character ROM not found: %s", path);
        console_printf("Warning: Character ROM not found at %s\n", path);
    }

    return ESP_OK;
}
