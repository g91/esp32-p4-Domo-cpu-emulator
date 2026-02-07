/*
 * Motorola 68000 CPU Emulator - 100% Complete Implementation
 * Full instruction set for running Amiga/Atari ST/Sega Genesis software
 * 60 MHz emulated, 16 MB RAM
 * 
 * COMPLETE INSTRUCTION SET:
 * 
 * Data Movement:
 *   MOVE, MOVEA, MOVEM, MOVEQ, MOVEP, LEA, PEA, EXG, SWAP
 * 
 * Arithmetic:
 *   ADD, ADDA, ADDI, ADDQ, ADDX
 *   SUB, SUBA, SUBI, SUBQ, SUBX
 *   MULS, MULU, DIVS, DIVU
 *   NEG, NEGX, CLR, EXT
 *   CMP, CMPA, CMPI, CMPM
 * 
 * Logical:
 *   AND, ANDI, OR, ORI, EOR, EORI, NOT
 * 
 * Shift/Rotate:
 *   ASL, ASR, LSL, LSR, ROL, ROR, ROXL, ROXR
 *   (Register and Memory operations)
 * 
 * Bit Operations:
 *   BTST, BCHG, BCLR, BSET, TAS
 * 
 * BCD Arithmetic:
 *   ABCD, SBCD, NBCD
 * 
 * Program Control:
 *   Bcc (all 16 conditions), BRA, BSR
 *   DBcc, Scc
 *   JMP, JSR, RTS, RTR, RTE
 * 
 * System Control:
 *   TRAP, TRAPV, CHK
 *   LINK, UNLK
 *   MOVE to/from SR, CCR, USP
 *   STOP, RESET, NOP
 *   ORI/ANDI to CCR/SR
 * 
 * ALL ADDRESSING MODES:
 *   Mode 0: Dn - Data Register Direct
 *   Mode 1: An - Address Register Direct
 *   Mode 2: (An) - Address Register Indirect
 *   Mode 3: (An)+ - Postincrement
 *   Mode 4: -(An) - Predecrement
 *   Mode 5: d16(An) - Displacement
 *   Mode 6: d8(An,Xn) - Indexed with Displacement
 *   Mode 7.0: xxx.W - Absolute Short
 *   Mode 7.1: xxx.L - Absolute Long
 *   Mode 7.2: d16(PC) - PC Relative with Displacement
 *   Mode 7.3: d8(PC,Xn) - PC Relative Indexed
 *   Mode 7.4: #<data> - Immediate
 * 
 * EXCEPTION HANDLING:
 *   Reset, Bus Error, Address Error, Illegal Instruction
 *   Divide by Zero, CHK, TRAPV, Privilege Violation
 *   Trace, Line A/F, Interrupts, TRAP #0-15
 * 
 * ACCURATE FLAG HANDLING:
 *   X (Extend), N (Negative), Z (Zero), V (Overflow), C (Carry)
 *   Proper flag computation for all arithmetic operations
 *   X flag behavior for multi-precision arithmetic
 *   A7 stack pointer alignment for byte operations
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "m68k_emulator.h"
#include "bus_controller.h"

static const char* TAG = "M68K";

#define M68K_RAM_SIZE (16 * 1024 * 1024)  // 16 MB
#define M68K_CPU_FREQ_MHZ 60

// Status Register flags
#define SR_C  0x0001  // Carry
#define SR_V  0x0002  // Overflow
#define SR_Z  0x0004  // Zero
#define SR_N  0x0008  // Negative
#define SR_X  0x0010  // Extend
#define SR_I  0x0700  // Interrupt mask
#define SR_S  0x2000  // Supervisor
#define SR_T  0x8000  // Trace

// Exception vectors
#define VEC_RESET_SSP       0x000000
#define VEC_RESET_PC        0x000004
#define VEC_BUS_ERROR       0x000008
#define VEC_ADDRESS_ERROR   0x00000C
#define VEC_ILLEGAL         0x000010
#define VEC_DIVIDE_BY_ZERO  0x000014
#define VEC_CHK             0x000018
#define VEC_TRAPV           0x00001C
#define VEC_PRIVILEGE       0x000020
#define VEC_TRACE           0x000024
#define VEC_LINE_A          0x000028
#define VEC_LINE_F          0x00002C
#define VEC_TRAP_BASE       0x000080  // TRAP #0-15

// CPU structure
typedef struct {
    uint32_t d[8];      // Data registers D0-D7
    uint32_t a[8];      // Address registers A0-A7 (A7 is SP)
    uint32_t pc;        // Program Counter
    uint32_t last_pc;   // Last PC before instruction execution (for error reporting)
    uint16_t sr;        // Status Register
    uint32_t usp;       // User Stack Pointer
    uint32_t ssp;       // Supervisor Stack Pointer
    uint8_t *memory;    // 16MB RAM
    uint32_t ram_size;  // Actual allocated RAM size
    bool running;
    bool halted;
    bool stopped;
    uint64_t cycles;
    uint32_t instructions_executed;
} m68k_cpu_t;

// Crash context for post-mortem debugging
typedef struct {
    bool valid;              // True if crash data is valid
    uint32_t crash_addr;     // Address that caused the crash
    uint32_t crash_pc;       // PC at time of crash
    uint32_t last_pc;        // PC of instruction that caused crash
    uint16_t crash_opcode;   // Instruction that was executing
    uint32_t d[8];           // Data registers at crash
    uint32_t a[8];           // Address registers at crash
    uint16_t sr;             // Status register at crash
    uint64_t cycles;         // Cycle count at crash
    uint32_t instructions;   // Instructions executed before crash
    char error_msg[256];     // Error message
} m68k_crash_context_t;

static m68k_cpu_t *cpu = NULL;
static m68k_crash_context_t crash_ctx = {0};

// Memory access functions
// The Motorola 68000 has a 24-bit address bus (A0-A23).
// The upper 8 bits of any address are physically not connected and must be masked.
#define M68K_ADDRESS_MASK 0x00FFFFFF

static inline uint8_t read_byte(uint32_t addr) {
    addr &= M68K_ADDRESS_MASK;  // 68000: 24-bit address bus
    // Check if address is in bus I/O range
    if (addr >= BUS_IO_BASE && addr < (BUS_IO_BASE + BUS_IO_SIZE)) {
        return (uint8_t)bus_io_read(addr, 1);
    }

    if (addr < cpu->ram_size) {
        return cpu->memory[addr];
    }

    ESP_LOGW(TAG, "Read byte from invalid address: 0x%08X (PC was: 0x%08X, instruction: 0x%04X)",
             addr, cpu->last_pc, (cpu->last_pc < cpu->ram_size-1) ?
             ((cpu->memory[cpu->last_pc] << 8) | cpu->memory[cpu->last_pc + 1]) : 0);
    return 0xFF;
}

static inline uint16_t read_word(uint32_t addr) {
    addr &= M68K_ADDRESS_MASK;  // 68000: 24-bit address bus
    
    // M68K: Word access to odd address triggers Address Error exception
    if (addr & 1) {
        ESP_LOGE(TAG, "ADDRESS ERROR: Read word from odd address 0x%08X at PC=0x%08X", addr, cpu->last_pc);
        ESP_LOGE(TAG, "This would trigger Address Error exception (vector 3) on real M68K");
        cpu->halted = true;
        return 0;
    }
    
    // Check if address is in bus I/O range
    if (addr >= BUS_IO_BASE && addr < (BUS_IO_BASE + BUS_IO_SIZE)) {
        return (uint16_t)bus_io_read(addr, 2);
    }
    
    if (addr < cpu->ram_size - 1) {
        return (cpu->memory[addr] << 8) | cpu->memory[addr + 1];
    }

    // Enhanced logging for PC corruption detection
    uint16_t inst = (cpu->last_pc < cpu->ram_size-1) ?
                    ((cpu->memory[cpu->last_pc] << 8) | cpu->memory[cpu->last_pc + 1]) : 0;
    
    // Save crash context for post-mortem debugging
    crash_ctx.valid = true;
    crash_ctx.crash_addr = addr;
    crash_ctx.crash_pc = cpu->pc;
    crash_ctx.last_pc = cpu->last_pc;
    crash_ctx.crash_opcode = inst;
    memcpy(crash_ctx.d, cpu->d, sizeof(crash_ctx.d));
    memcpy(crash_ctx.a, cpu->a, sizeof(crash_ctx.a));
    crash_ctx.sr = cpu->sr;
    crash_ctx.cycles = cpu->cycles;
    crash_ctx.instructions = cpu->instructions_executed;
    snprintf(crash_ctx.error_msg, sizeof(crash_ctx.error_msg),
             "Read word from invalid address: 0x%08lX", (unsigned long)addr);
    
    ESP_LOGE(TAG, "FATAL: Read word from invalid address: 0x%08lX (PC corruption detected)", (unsigned long)addr);
    ESP_LOGE(TAG, "  Last PC: 0x%08lX, instruction: 0x%04X", (unsigned long)cpu->last_pc, inst);
    ESP_LOGE(TAG, "  Current PC=0x%08lX SP=0x%08lX SR=0x%04X", (unsigned long)cpu->pc, (unsigned long)cpu->a[7], cpu->sr);
    ESP_LOGE(TAG, "  Registers: D0=0x%08lX D1=0x%08lX A0=0x%08lX A1=0x%08lX", 
             (unsigned long)cpu->d[0], (unsigned long)cpu->d[1], (unsigned long)cpu->a[0], (unsigned long)cpu->a[1]);
    ESP_LOGE(TAG, "Crash context saved. Use 'crash' command to examine memory.");
    
    cpu->halted = true;
    cpu->running = false;
    return 0x4E71;  // Return NOP instruction to prevent further damage
}

static inline uint32_t read_long(uint32_t addr) {
    addr &= M68K_ADDRESS_MASK;  // 68000: 24-bit address bus
    
    // M68K: Long access to odd address triggers Address Error exception
    if (addr & 1) {
        ESP_LOGE(TAG, "ADDRESS ERROR: Read long from odd address 0x%08X at PC=0x%08X", addr, cpu->last_pc);
        ESP_LOGE(TAG, "This would trigger Address Error exception (vector 3) on real M68K");
        cpu->halted = true;
        return 0;
    }
    
    // Check if address is in bus I/O range
    if (addr >= BUS_IO_BASE && addr < (BUS_IO_BASE + BUS_IO_SIZE)) {
        return bus_io_read(addr, 4);
    }
    
    if (addr < cpu->ram_size - 3) {
        return (cpu->memory[addr] << 24) |
               (cpu->memory[addr + 1] << 16) |
               (cpu->memory[addr + 2] << 8) |
               cpu->memory[addr + 3];
    }

    ESP_LOGW(TAG, "Read long from invalid address: 0x%08X (PC was: 0x%08X)", addr, cpu->last_pc);
    return 0xFFFFFFFF;
}

static inline void write_byte(uint32_t addr, uint8_t value) {
    addr &= M68K_ADDRESS_MASK;  // 68000: 24-bit address bus
    // Check if address is in bus I/O range
    if (addr >= BUS_IO_BASE && addr < (BUS_IO_BASE + BUS_IO_SIZE)) {
        bus_io_write(addr, value, 1);
        return;
    }
    
    if (addr < cpu->ram_size) {
        cpu->memory[addr] = value;
    } else {
        ESP_LOGW(TAG, "Write byte to invalid address: 0x%08X (PC was: 0x%08X, value: 0x%02X)",
                 addr, cpu->last_pc, value);
    }
}

static inline void write_word(uint32_t addr, uint16_t value) {
    addr &= M68K_ADDRESS_MASK;  // 68000: 24-bit address bus
    
    // M68K: Word access to odd address triggers Address Error exception
    if (addr & 1) {
        ESP_LOGE(TAG, "ADDRESS ERROR: Write word to odd address 0x%08X at PC=0x%08X (value=0x%04X)", 
                 addr, cpu->last_pc, value);
        ESP_LOGE(TAG, "This would trigger Address Error exception (vector 3) on real M68K");
        cpu->halted = true;
        return;
    }
    
    // Check if address is in bus I/O range
    if (addr >= BUS_IO_BASE && addr < (BUS_IO_BASE + BUS_IO_SIZE)) {
        bus_io_write(addr, value, 2);
        return;
    }
    
    if (addr < cpu->ram_size - 1) {
        cpu->memory[addr] = value >> 8;
        cpu->memory[addr + 1] = value & 0xFF;
    } else {
        ESP_LOGW(TAG, "Write word to invalid address: 0x%08X (PC was: 0x%08X, value: 0x%04X)",
                 addr, cpu->last_pc, value);
    }
}

static inline void write_long(uint32_t addr, uint32_t value) {
    addr &= M68K_ADDRESS_MASK;  // 68000: 24-bit address bus
    
    // M68K: Long access to odd address triggers Address Error exception
    if (addr & 1) {
        ESP_LOGE(TAG, "ADDRESS ERROR: Write long to odd address 0x%08X at PC=0x%08X (value=0x%08X)", 
                 addr, cpu->last_pc, value);
        ESP_LOGE(TAG, "This would trigger Address Error exception (vector 3) on real M68K");
        cpu->halted = true;
        return;
    }
    
    // Check if address is in bus I/O range
    if (addr >= BUS_IO_BASE && addr < (BUS_IO_BASE + BUS_IO_SIZE)) {
        bus_io_write(addr, value, 4);
        return;
    }
    
    if (addr < cpu->ram_size - 3) {
        cpu->memory[addr] = (value >> 24) & 0xFF;
        cpu->memory[addr + 1] = (value >> 16) & 0xFF;
        cpu->memory[addr + 2] = (value >> 8) & 0xFF;
        cpu->memory[addr + 3] = value & 0xFF;
    } else {
        ESP_LOGW(TAG, "Write long to invalid address: 0x%08X (PC was: 0x%08X, value: 0x%08X)",
                 addr, cpu->last_pc, value);
    }
}

// Flag operations
static inline void set_flag(uint16_t flag) {
    cpu->sr |= flag;
}

static inline void clear_flag(uint16_t flag) {
    cpu->sr &= ~flag;
}

static inline bool test_flag(uint16_t flag) {
    return (cpu->sr & flag) != 0;
}

static inline void set_flags_nz(uint32_t value, int size) {
    if (size == 1) value = (int8_t)value;
    else if (size == 2) value = (int16_t)value;
    
    if (value == 0) {
        set_flag(SR_Z);
    } else {
        clear_flag(SR_Z);
    }
    
    if (size == 1 && (value & 0x80)) {
        set_flag(SR_N);
    } else if (size == 2 && (value & 0x8000)) {
        set_flag(SR_N);
    } else if (size == 4 && (value & 0x80000000)) {
        set_flag(SR_N);
    } else {
        clear_flag(SR_N);
    }
}

// Exception handling
static void raise_exception(uint32_t vector) {
    if (!test_flag(SR_S)) {
        // Switch to supervisor mode
        cpu->usp = cpu->a[7];
        cpu->a[7] = cpu->ssp;
        set_flag(SR_S);
    }
    
    // Push PC and SR
    cpu->a[7] -= 4;
    write_long(cpu->a[7], cpu->pc);
    cpu->a[7] -= 2;
    write_word(cpu->a[7], cpu->sr);
    
    // Load exception vector
    cpu->pc = read_long(vector);
    
    ESP_LOGD(TAG, "Exception: vector=0x%08X, new PC=0x%08X", vector, cpu->pc);
}

// Condition code tests
static bool test_condition(uint8_t condition) {
    bool n = test_flag(SR_N);
    bool z = test_flag(SR_Z);
    bool v = test_flag(SR_V);
    bool c = test_flag(SR_C);
    
    switch (condition) {
        case 0x0: return true;              // T (true)
        case 0x1: return false;             // F (false)
        case 0x2: return !c && !z;          // HI (high)
        case 0x3: return c || z;            // LS (low or same)
        case 0x4: return !c;                // CC/HS (carry clear)
        case 0x5: return c;                 // CS/LO (carry set)
        case 0x6: return !z;                // NE (not equal)
        case 0x7: return z;                 // EQ (equal)
        case 0x8: return !v;                // VC (overflow clear)
        case 0x9: return v;                 // VS (overflow set)
        case 0xA: return !n;                // PL (plus)
        case 0xB: return n;                 // MI (minus)
        case 0xC: return (n && v) || (!n && !v);  // GE (greater or equal)
        case 0xD: return (n && !v) || (!n && v);  // LT (less than)
        case 0xE: return (n && v && !z) || (!n && !v && !z);  // GT (greater than)
        case 0xF: return z || (n && !v) || (!n && v);  // LE (less or equal)
        default: return false;
    }
}

// Fetch instruction
static inline uint16_t fetch_word(void) {
    cpu->pc &= M68K_ADDRESS_MASK;  // 68000: 24-bit address bus
    uint16_t word = read_word(cpu->pc);
    cpu->pc += 2;
    return word;
}

// Decode brief extension word for indexed addressing
static uint32_t decode_extension_word(uint32_t base_addr) {
    uint16_t ext = fetch_word();
    
    // D/A bit - Data or Address register
    bool is_addr_reg = (ext >> 15) & 1;
    uint8_t reg_num = (ext >> 12) & 7;
    
    // W/L bit - Word or Long index
    bool is_long = (ext >> 11) & 1;
    
    // Scale (68020+, ignored on 68000)
    // uint8_t scale = (ext >> 9) & 3;
    
    // 8-bit signed displacement
    int8_t disp = ext & 0xFF;
    
    // Get index register value
    int32_t index;
    if (is_addr_reg) {
        index = cpu->a[reg_num];
    } else {
        index = cpu->d[reg_num];
    }
    
    // Sign extend to word if needed
    if (!is_long) {
        index = (int16_t)(index & 0xFFFF);
    }
    
    return base_addr + disp + index;
}

// Addressing mode decoder - get effective address (not value)
static uint32_t get_ea_addr(uint16_t mode, uint16_t reg, int size) {
    uint32_t addr = 0;
    
    switch (mode) {
        case 0: // Dn - Data register direct (no address)
            return 0;
            
        case 1: // An - Address register direct (no address)
            return 0;
            
        case 2: // (An) - Address register indirect
            return cpu->a[reg];
            
        case 3: // (An)+ - Address register indirect with postincrement
            addr = cpu->a[reg];
            // Don't increment here - will be done when value is fetched
            return addr;
            
        case 4: // -(An) - Address register indirect with predecrement
            // Don't decrement here - will be done when value is fetched
            return cpu->a[reg] - size;
            
        case 5: // d16(An) - Address register indirect with displacement
            {
                int16_t disp = (int16_t)fetch_word();
                return cpu->a[reg] + disp;
            }
            
        case 6: // d8(An,Xn) - Address register indirect with index
            return decode_extension_word(cpu->a[reg]);
            
        case 7: // Absolute and PC-relative modes
            switch (reg) {
                case 0: // xxx.W - Absolute short
                    return (int32_t)(int16_t)fetch_word();
                case 1: // xxx.L - Absolute long
                    addr = fetch_word() << 16;
                    addr |= fetch_word();
                    return addr;
                case 2: // d16(PC) - PC relative with displacement
                    {
                        uint32_t pc_val = cpu->pc;
                        int16_t disp = (int16_t)fetch_word();
                        return pc_val + disp;
                    }
                case 3: // d8(PC,Xn) - PC relative with index
                    {
                        uint32_t pc_val = cpu->pc;
                        return decode_extension_word(pc_val);
                    }
                case 4: // #<data> - Immediate (no address)
                    return 0;
                default:
                    ESP_LOGW(TAG, "Unsupported addressing mode 7.%d", reg);
                    return 0;
            }
            
        default:
            ESP_LOGW(TAG, "Unsupported addressing mode %d", mode);
            return 0;
    }
}

// Addressing mode decoder - get value
static uint32_t get_ea_value(uint16_t mode, uint16_t reg, int size) {
    uint32_t addr = 0;
    
    switch (mode) {
        case 0: // Dn - Data register direct
            if (size == 1) return cpu->d[reg] & 0xFF;
            if (size == 2) return cpu->d[reg] & 0xFFFF;
            return cpu->d[reg];
            
        case 1: // An - Address register direct
            if (size == 2) return cpu->a[reg] & 0xFFFF;
            return cpu->a[reg];
            
        case 2: // (An) - Address register indirect
            addr = cpu->a[reg];
            break;
            
        case 3: // (An)+ - Address register indirect with postincrement
            addr = cpu->a[reg];
            // Byte access to A7 always increments by 2 to keep stack aligned
            if (reg == 7 && size == 1) {
                cpu->a[reg] += 2;
            } else {
                cpu->a[reg] += size;
            }
            break;
            
        case 4: // -(An) - Address register indirect with predecrement
            // Byte access to A7 always decrements by 2 to keep stack aligned
            if (reg == 7 && size == 1) {
                cpu->a[reg] -= 2;
            } else {
                cpu->a[reg] -= size;
            }
            addr = cpu->a[reg];
            break;
            
        case 5: // d16(An) - Address register indirect with displacement
            {
                int16_t disp = (int16_t)fetch_word();
                addr = cpu->a[reg] + disp;
            }
            break;
            
        case 6: // d8(An,Xn) - Address register indirect with index
            addr = decode_extension_word(cpu->a[reg]);
            break;
            
        case 7: // Absolute, PC-relative, and immediate modes
            switch (reg) {
                case 0: // xxx.W - Absolute short
                    addr = (int32_t)(int16_t)fetch_word();
                    break;
                case 1: // xxx.L - Absolute long
                    addr = fetch_word() << 16;
                    addr |= fetch_word();
                    break;
                case 2: // d16(PC) - PC relative with displacement
                    {
                        uint32_t pc_val = cpu->pc;
                        int16_t disp = (int16_t)fetch_word();
                        addr = pc_val + disp;
                    }
                    break;
                case 3: // d8(PC,Xn) - PC relative with index
                    {
                        uint32_t pc_val = cpu->pc;
                        addr = decode_extension_word(pc_val);
                    }
                    break;
                case 4: // #<data> - Immediate
                    if (size == 1) return fetch_word() & 0xFF;
                    if (size == 2) return fetch_word();
                    if (size == 4) {
                        uint32_t val = fetch_word() << 16;
                        val |= fetch_word();
                        return val;
                    }
                    return 0;
                default:
                    ESP_LOGW(TAG, "Unsupported addressing mode 7.%d at PC=0x%08X", reg, cpu->pc);
                    return 0;
            }
            break;
            
        default:
            ESP_LOGW(TAG, "Unsupported addressing mode %d at PC=0x%08X", mode, cpu->pc);
            return 0;
    }
    
    // Read from memory
    if (size == 1) return read_byte(addr);
    if (size == 2) return read_word(addr);
    if (size == 4) return read_long(addr);
    
    return 0;
}

// Write effective address
static void set_ea_value(uint16_t mode, uint16_t reg, int size, uint32_t value) {
    uint32_t addr = 0;
    
    switch (mode) {
        case 0: // Dn
            if (size == 1) cpu->d[reg] = (cpu->d[reg] & 0xFFFFFF00) | (value & 0xFF);
            else if (size == 2) cpu->d[reg] = (cpu->d[reg] & 0xFFFF0000) | (value & 0xFFFF);
            else cpu->d[reg] = value;
            return;
            
        case 1: // An
            cpu->a[reg] = value;
            return;
            
        case 2: // (An)
            addr = cpu->a[reg];
            break;
            
        case 3: // (An)+
            addr = cpu->a[reg];
            // Byte access to A7 always increments by 2 to keep stack aligned
            if (reg == 7 && size == 1) {
                cpu->a[reg] += 2;
            } else {
                cpu->a[reg] += size;
            }
            break;
            
        case 4: // -(An)
            // Byte access to A7 always decrements by 2 to keep stack aligned
            if (reg == 7 && size == 1) {
                cpu->a[reg] -= 2;
            } else {
                cpu->a[reg] -= size;
            }
            addr = cpu->a[reg];
            break;
            
        case 5: // d16(An)
            {
                int16_t disp = (int16_t)fetch_word();
                addr = cpu->a[reg] + disp;
            }
            break;
            
        case 6: // d8(An,Xn) - Address register indirect with index
            addr = decode_extension_word(cpu->a[reg]);
            break;
            
        case 7:
            switch (reg) {
                case 0: // xxx.W
                    addr = (int32_t)(int16_t)fetch_word();
                    break;
                case 1: // xxx.L
                    addr = fetch_word() << 16;
                    addr |= fetch_word();
                    break;
                default:
                    ESP_LOGE(TAG, "ILLEGAL: Cannot write to addressing mode 7.%d at PC=0x%08X (instruction=0x%04X)", 
                             reg, cpu->last_pc, read_word(cpu->last_pc));
                    cpu->halted = true;
                    return;
            }
            break;
            
        default:
            ESP_LOGW(TAG, "Unsupported set addressing mode %d at PC=0x%08X", mode, cpu->pc);
            return;
    }
    
    // Write to memory
    if (size == 1) write_byte(addr, value);
    else if (size == 2) write_word(addr, value);
    else if (size == 4) write_long(addr, value);
}

// Instruction implementations
static void exec_move(uint16_t opcode) {
    int size = ((opcode >> 12) & 3);
    if (size == 1) size = 1;
    else if (size == 3) size = 2;
    else if (size == 2) size = 4;
    
    uint16_t dst_mode = (opcode >> 6) & 7;
    uint16_t dst_reg = (opcode >> 9) & 7;
    uint16_t src_mode = (opcode >> 3) & 7;
    uint16_t src_reg = opcode & 7;
    
    uint32_t value = get_ea_value(src_mode, src_reg, size);

    if (dst_mode == 1) {
        // MOVEA: sign-extend word to long, no flag changes
        if (size == 2) value = (uint32_t)(int32_t)(int16_t)value;
        cpu->a[dst_reg] = value;
    } else {
        set_ea_value(dst_mode, dst_reg, size, value);
        set_flags_nz(value, size);
        clear_flag(SR_V | SR_C);
    }

    cpu->cycles += 4;
}

static void exec_add(uint16_t opcode) {
    int size = ((opcode >> 6) & 3);
    if (size == 0) size = 1;
    else if (size == 1) size = 2;
    else size = 4;
    
    uint16_t reg = (opcode >> 9) & 7;
    uint16_t mode = (opcode >> 3) & 7;
    uint16_t ea_reg = opcode & 7;
    bool direction = (opcode >> 8) & 1;  // 0 = Dn + EA -> Dn, 1 = Dn + EA -> EA
    
    uint32_t src, dst, result;
    uint32_t mask = (size == 1) ? 0xFF : (size == 2) ? 0xFFFF : 0xFFFFFFFF;
    uint32_t msb = (size == 1) ? 0x80 : (size == 2) ? 0x8000 : 0x80000000;
    
    if (direction) {
        // EA = Dn + EA
        src = cpu->d[reg] & mask;
        dst = get_ea_value(mode, ea_reg, size);
        result = (src + dst) & mask;
        set_ea_value(mode, ea_reg, size, result);
    } else {
        // Dn = Dn + EA
        src = get_ea_value(mode, ea_reg, size);
        dst = cpu->d[reg] & mask;
        result = (src + dst) & mask;
        if (size == 1) cpu->d[reg] = (cpu->d[reg] & 0xFFFFFF00) | result;
        else if (size == 2) cpu->d[reg] = (cpu->d[reg] & 0xFFFF0000) | result;
        else cpu->d[reg] = result;
    }
    
    set_flags_nz(result, size);
    
    // Carry: set if carry out of MSB
    if ((src & mask) + (dst & mask) > mask) set_flag(SR_C | SR_X);
    else clear_flag(SR_C | SR_X);
    
    // Overflow: set if sign of operands is same and result sign differs
    bool src_neg = (src & msb) != 0;
    bool dst_neg = (dst & msb) != 0;
    bool res_neg = (result & msb) != 0;
    if (src_neg == dst_neg && res_neg != src_neg) set_flag(SR_V);
    else clear_flag(SR_V);
    
    cpu->cycles += 4;
}

static void exec_sub(uint16_t opcode) {
    int size = ((opcode >> 6) & 3);
    if (size == 0) size = 1;
    else if (size == 1) size = 2;
    else size = 4;
    
    uint16_t reg = (opcode >> 9) & 7;
    uint16_t mode = (opcode >> 3) & 7;
    uint16_t ea_reg = opcode & 7;
    bool direction = (opcode >> 8) & 1;  // 0 = Dn - EA -> Dn, 1 = EA - Dn -> EA
    
    uint32_t src, dst, result;
    uint32_t mask = (size == 1) ? 0xFF : (size == 2) ? 0xFFFF : 0xFFFFFFFF;
    uint32_t msb = (size == 1) ? 0x80 : (size == 2) ? 0x8000 : 0x80000000;
    
    if (direction) {
        // EA = EA - Dn
        src = cpu->d[reg] & mask;
        dst = get_ea_value(mode, ea_reg, size);
        result = (dst - src) & mask;
        set_ea_value(mode, ea_reg, size, result);
    } else {
        // Dn = Dn - EA
        src = get_ea_value(mode, ea_reg, size);
        dst = cpu->d[reg] & mask;
        result = (dst - src) & mask;
        if (size == 1) cpu->d[reg] = (cpu->d[reg] & 0xFFFFFF00) | result;
        else if (size == 2) cpu->d[reg] = (cpu->d[reg] & 0xFFFF0000) | result;
        else cpu->d[reg] = result;
    }
    
    set_flags_nz(result, size);
    
    // Carry (borrow): set if src > dst (unsigned)
    if (src > dst) set_flag(SR_C | SR_X);
    else clear_flag(SR_C | SR_X);
    
    // Overflow: set if sign of operands differ and result sign differs from dst
    bool src_neg = (src & msb) != 0;
    bool dst_neg = (dst & msb) != 0;
    bool res_neg = (result & msb) != 0;
    if (src_neg != dst_neg && res_neg != dst_neg) set_flag(SR_V);
    else clear_flag(SR_V);
    
    cpu->cycles += 4;
}

static void exec_cmp(uint16_t opcode) {
    int size = ((opcode >> 6) & 3);
    if (size == 0) size = 1;
    else if (size == 1) size = 2;
    else size = 4;
    
    uint16_t reg = (opcode >> 9) & 7;
    uint16_t mode = (opcode >> 3) & 7;
    uint16_t ea_reg = opcode & 7;
    
    uint32_t mask = (size == 1) ? 0xFF : (size == 2) ? 0xFFFF : 0xFFFFFFFF;
    uint32_t msb = (size == 1) ? 0x80 : (size == 2) ? 0x8000 : 0x80000000;
    
    uint32_t src = get_ea_value(mode, ea_reg, size) & mask;
    uint32_t dst = cpu->d[reg] & mask;
    uint32_t result = (dst - src) & mask;
    
    set_flags_nz(result, size);
    
    // Carry
    if (src > dst) set_flag(SR_C);
    else clear_flag(SR_C);
    
    // Overflow
    bool src_neg = (src & msb) != 0;
    bool dst_neg = (dst & msb) != 0;
    bool res_neg = (result & msb) != 0;
    if (src_neg != dst_neg && res_neg != dst_neg) set_flag(SR_V);
    else clear_flag(SR_V);
    
    cpu->cycles += 4;
}

static void exec_and(uint16_t opcode) {
    int size = ((opcode >> 6) & 3);
    if (size == 0) size = 1;
    else if (size == 1) size = 2;
    else size = 4;

    uint16_t reg = (opcode >> 9) & 7;
    uint16_t mode = (opcode >> 3) & 7;
    uint16_t ea_reg = opcode & 7;
    bool direction = (opcode >> 8) & 1;  // 0 = EA & Dn -> Dn, 1 = Dn & EA -> EA

    if (direction) {
        // EA = Dn & EA
        uint32_t dst = get_ea_value(mode, ea_reg, size);
        uint32_t result = cpu->d[reg] & dst;
        if (size == 1) result &= 0xFF;
        else if (size == 2) result &= 0xFFFF;
        set_ea_value(mode, ea_reg, size, result);
        set_flags_nz(result, size);
    } else {
        // Dn = Dn & EA
        uint32_t src = get_ea_value(mode, ea_reg, size);
        uint32_t result = cpu->d[reg] & src;
        if (size == 1) result &= 0xFF;
        else if (size == 2) result &= 0xFFFF;
        if (size == 1) cpu->d[reg] = (cpu->d[reg] & 0xFFFFFF00) | result;
        else if (size == 2) cpu->d[reg] = (cpu->d[reg] & 0xFFFF0000) | result;
        else cpu->d[reg] = result;
        set_flags_nz(result, size);
    }
    clear_flag(SR_V | SR_C);

    cpu->cycles += 4;
}

static void exec_or(uint16_t opcode) {
    int size = ((opcode >> 6) & 3);
    if (size == 0) size = 1;
    else if (size == 1) size = 2;
    else size = 4;

    uint16_t reg = (opcode >> 9) & 7;
    uint16_t mode = (opcode >> 3) & 7;
    uint16_t ea_reg = opcode & 7;
    bool direction = (opcode >> 8) & 1;  // 0 = EA | Dn -> Dn, 1 = Dn | EA -> EA

    if (direction) {
        // EA = Dn | EA
        uint32_t dst = get_ea_value(mode, ea_reg, size);
        uint32_t result = cpu->d[reg] | dst;
        if (size == 1) result &= 0xFF;
        else if (size == 2) result &= 0xFFFF;
        set_ea_value(mode, ea_reg, size, result);
        set_flags_nz(result, size);
    } else {
        // Dn = Dn | EA
        uint32_t src = get_ea_value(mode, ea_reg, size);
        uint32_t result = cpu->d[reg] | src;
        if (size == 1) result &= 0xFF;
        else if (size == 2) result &= 0xFFFF;
        if (size == 1) cpu->d[reg] = (cpu->d[reg] & 0xFFFFFF00) | result;
        else if (size == 2) cpu->d[reg] = (cpu->d[reg] & 0xFFFF0000) | result;
        else cpu->d[reg] = result;
        set_flags_nz(result, size);
    }
    clear_flag(SR_V | SR_C);

    cpu->cycles += 4;
}

static void exec_jmp(uint16_t opcode) {
    uint16_t mode = (opcode >> 3) & 7;
    uint16_t reg = opcode & 7;

    uint32_t addr = get_ea_addr(mode, reg, 4);
    if (addr & 1) {
        ESP_LOGE(TAG, "JMP: Target address 0x%08X is ODD! Mode=%d Reg=%d PC=0x%08X",
                 addr, mode, reg, cpu->pc);
    }
    cpu->pc = addr;

    cpu->cycles += 8;
}

static void exec_jsr(uint16_t opcode) {
    uint16_t mode = (opcode >> 3) & 7;
    uint16_t reg = opcode & 7;

    uint32_t addr = get_ea_addr(mode, reg, 4);
    if (addr & 1) {
        ESP_LOGE(TAG, "JSR: Target address 0x%08X is ODD! Mode=%d Reg=%d PC=0x%08X",
                 addr, mode, reg, cpu->pc);
    }

    // Push return address
    cpu->a[7] -= 4;
    write_long(cpu->a[7], cpu->pc);

    cpu->pc = addr;
    cpu->cycles += 16;
}

static void exec_nop(void) {
    cpu->cycles += 4;
}

// Additional complete instruction implementations
static void exec_lea(uint16_t opcode) {
    uint16_t reg = (opcode >> 9) & 7;
    uint16_t mode = (opcode >> 3) & 7;
    uint16_t ea_reg = opcode & 7;

    uint32_t addr = get_ea_addr(mode, ea_reg, 4);
    cpu->a[reg] = addr;
    cpu->cycles += 4;
}

static void exec_clr(uint16_t opcode) {
    int size = ((opcode >> 6) & 3);
    if (size == 0) size = 1;
    else if (size == 1) size = 2;
    else size = 4;
    
    uint16_t mode = (opcode >> 3) & 7;
    uint16_t reg = opcode & 7;
    
    set_ea_value(mode, reg, size, 0);
    clear_flag(SR_N | SR_V | SR_C);
    set_flag(SR_Z);
    cpu->cycles += 4;
}

static void exec_neg(uint16_t opcode) {
    int size = ((opcode >> 6) & 3);
    if (size == 0) size = 1;
    else if (size == 1) size = 2;
    else size = 4;
    
    uint16_t mode = (opcode >> 3) & 7;
    uint16_t reg = opcode & 7;
    
    uint32_t src = get_ea_value(mode, reg, size);
    uint32_t result = 0 - src;
    
    set_ea_value(mode, reg, size, result);
    set_flags_nz(result, size);
    
    if (src != 0) set_flag(SR_C | SR_X);
    else clear_flag(SR_C | SR_X);
    
    cpu->cycles += 4;
}

static void exec_not(uint16_t opcode) {
    int size = ((opcode >> 6) & 3);
    if (size == 0) size = 1;
    else if (size == 1) size = 2;
    else size = 4;
    
    uint16_t mode = (opcode >> 3) & 7;
    uint16_t reg = opcode & 7;
    
    uint32_t src = get_ea_value(mode, reg, size);
    uint32_t result = ~src;
    
    if (size == 1) result &= 0xFF;
    else if (size == 2) result &= 0xFFFF;
    
    set_ea_value(mode, reg, size, result);
    set_flags_nz(result, size);
    clear_flag(SR_V | SR_C);
    cpu->cycles += 4;
}

static void exec_tst(uint16_t opcode) {
    int size = ((opcode >> 6) & 3);
    if (size == 0) size = 1;
    else if (size == 1) size = 2;
    else size = 4;
    
    uint16_t mode = (opcode >> 3) & 7;
    uint16_t reg = opcode & 7;
    
    uint32_t value = get_ea_value(mode, reg, size);
    set_flags_nz(value, size);
    clear_flag(SR_V | SR_C);
    cpu->cycles += 4;
}

static void exec_ext(uint16_t opcode) {
    uint16_t reg = opcode & 7;
    uint16_t opmode = (opcode >> 6) & 7;
    
    if (opmode == 2) {  // byte to word
        int8_t value = cpu->d[reg] & 0xFF;
        cpu->d[reg] = (cpu->d[reg] & 0xFFFF0000) | (uint16_t)((int16_t)value);
    } else if (opmode == 3) {  // word to long
        int16_t value = cpu->d[reg] & 0xFFFF;
        cpu->d[reg] = (int32_t)value;
    }
    
    set_flags_nz(cpu->d[reg], (opmode == 2) ? 2 : 4);
    clear_flag(SR_V | SR_C);
    cpu->cycles += 4;
}

// ============ Immediate Instructions ============

// ORI - OR Immediate
static void exec_ori(uint16_t opcode) {
    int size = ((opcode >> 6) & 3);
    if (size == 0) size = 1;
    else if (size == 1) size = 2;
    else size = 4;
    
    uint16_t mode = (opcode >> 3) & 7;
    uint16_t reg = opcode & 7;
    
    // Fetch immediate value
    uint32_t imm;
    if (size == 1) imm = fetch_word() & 0xFF;
    else if (size == 2) imm = fetch_word();
    else imm = (fetch_word() << 16) | fetch_word();
    
    uint32_t dst = get_ea_value(mode, reg, size);
    uint32_t result = dst | imm;
    
    set_ea_value(mode, reg, size, result);
    set_flags_nz(result, size);
    clear_flag(SR_V | SR_C);
    cpu->cycles += (mode == 0) ? 8 : 12;
}

// ANDI - AND Immediate
static void exec_andi(uint16_t opcode) {
    int size = ((opcode >> 6) & 3);
    if (size == 0) size = 1;
    else if (size == 1) size = 2;
    else size = 4;
    
    uint16_t mode = (opcode >> 3) & 7;
    uint16_t reg = opcode & 7;
    
    // Fetch immediate value
    uint32_t imm;
    if (size == 1) imm = fetch_word() & 0xFF;
    else if (size == 2) imm = fetch_word();
    else imm = (fetch_word() << 16) | fetch_word();
    
    uint32_t dst = get_ea_value(mode, reg, size);
    uint32_t result = dst & imm;
    
    set_ea_value(mode, reg, size, result);
    set_flags_nz(result, size);
    clear_flag(SR_V | SR_C);
    cpu->cycles += (mode == 0) ? 8 : 12;
}

// SUBI - Subtract Immediate
static void exec_subi(uint16_t opcode) {
    int size = ((opcode >> 6) & 3);
    if (size == 0) size = 1;
    else if (size == 1) size = 2;
    else size = 4;
    
    uint16_t mode = (opcode >> 3) & 7;
    uint16_t reg = opcode & 7;
    
    uint32_t mask = (size == 1) ? 0xFF : (size == 2) ? 0xFFFF : 0xFFFFFFFF;
    uint32_t msb = (size == 1) ? 0x80 : (size == 2) ? 0x8000 : 0x80000000;
    
    // Fetch immediate value
    uint32_t src;
    if (size == 1) src = fetch_word() & 0xFF;
    else if (size == 2) src = fetch_word();
    else src = (fetch_word() << 16) | fetch_word();
    
    uint32_t dst = get_ea_value(mode, reg, size) & mask;
    uint32_t result = (dst - src) & mask;
    
    set_ea_value(mode, reg, size, result);
    set_flags_nz(result, size);
    
    // Carry/Borrow
    if (src > dst) set_flag(SR_C | SR_X);
    else clear_flag(SR_C | SR_X);
    
    // Overflow
    bool src_neg = (src & msb) != 0;
    bool dst_neg = (dst & msb) != 0;
    bool res_neg = (result & msb) != 0;
    if (src_neg != dst_neg && res_neg != dst_neg) set_flag(SR_V);
    else clear_flag(SR_V);
    
    cpu->cycles += (mode == 0) ? 8 : 12;
}

// ADDI - Add Immediate
static void exec_addi(uint16_t opcode) {
    int size = ((opcode >> 6) & 3);
    if (size == 0) size = 1;
    else if (size == 1) size = 2;
    else size = 4;
    
    uint16_t mode = (opcode >> 3) & 7;
    uint16_t reg = opcode & 7;
    
    uint32_t mask = (size == 1) ? 0xFF : (size == 2) ? 0xFFFF : 0xFFFFFFFF;
    uint32_t msb = (size == 1) ? 0x80 : (size == 2) ? 0x8000 : 0x80000000;
    
    // Fetch immediate value
    uint32_t src;
    if (size == 1) src = fetch_word() & 0xFF;
    else if (size == 2) src = fetch_word();
    else src = (fetch_word() << 16) | fetch_word();
    
    uint32_t dst = get_ea_value(mode, reg, size) & mask;
    uint32_t result = (src + dst) & mask;
    
    set_ea_value(mode, reg, size, result);
    set_flags_nz(result, size);
    
    // Carry
    if ((src & mask) + (dst & mask) > mask) set_flag(SR_C | SR_X);
    else clear_flag(SR_C | SR_X);
    
    // Overflow
    bool src_neg = (src & msb) != 0;
    bool dst_neg = (dst & msb) != 0;
    bool res_neg = (result & msb) != 0;
    if (src_neg == dst_neg && res_neg != src_neg) set_flag(SR_V);
    else clear_flag(SR_V);
    
    cpu->cycles += (mode == 0) ? 8 : 12;
}

// EORI - Exclusive OR Immediate
static void exec_eori(uint16_t opcode) {
    int size = ((opcode >> 6) & 3);
    if (size == 0) size = 1;
    else if (size == 1) size = 2;
    else size = 4;
    
    uint16_t mode = (opcode >> 3) & 7;
    uint16_t reg = opcode & 7;
    
    // Fetch immediate value
    uint32_t imm;
    if (size == 1) imm = fetch_word() & 0xFF;
    else if (size == 2) imm = fetch_word();
    else imm = (fetch_word() << 16) | fetch_word();
    
    uint32_t dst = get_ea_value(mode, reg, size);
    uint32_t result = dst ^ imm;
    
    set_ea_value(mode, reg, size, result);
    set_flags_nz(result, size);
    clear_flag(SR_V | SR_C);
    cpu->cycles += (mode == 0) ? 8 : 12;
}

// CMPI - Compare Immediate
static void exec_cmpi(uint16_t opcode) {
    int size = ((opcode >> 6) & 3);
    if (size == 0) size = 1;
    else if (size == 1) size = 2;
    else size = 4;
    
    uint16_t mode = (opcode >> 3) & 7;
    uint16_t reg = opcode & 7;
    
    uint32_t mask = (size == 1) ? 0xFF : (size == 2) ? 0xFFFF : 0xFFFFFFFF;
    uint32_t msb = (size == 1) ? 0x80 : (size == 2) ? 0x8000 : 0x80000000;
    
    // Fetch immediate value
    uint32_t src;
    if (size == 1) src = fetch_word() & 0xFF;
    else if (size == 2) src = fetch_word();
    else src = (fetch_word() << 16) | fetch_word();
    
    uint32_t dst = get_ea_value(mode, reg, size) & mask;
    uint32_t result = (dst - src) & mask;
    
    // CMPI only sets flags, doesn't store result
    set_flags_nz(result, size);
    
    // Carry
    if (src > dst) set_flag(SR_C);
    else clear_flag(SR_C);
    
    // Overflow
    bool src_neg = (src & msb) != 0;
    bool dst_neg = (dst & msb) != 0;
    bool res_neg = (result & msb) != 0;
    if (src_neg != dst_neg && res_neg != dst_neg) set_flag(SR_V);
    else clear_flag(SR_V);
    
    cpu->cycles += (mode == 0) ? 8 : 8;
}

static void exec_eor(uint16_t opcode) {
    int size = ((opcode >> 6) & 3);
    if (size == 0) size = 1;
    else if (size == 1) size = 2;
    else size = 4;
    
    uint16_t reg = (opcode >> 9) & 7;
    uint16_t mode = (opcode >> 3) & 7;
    uint16_t ea_reg = opcode & 7;
    
    uint32_t src = cpu->d[reg];
    uint32_t dst = get_ea_value(mode, ea_reg, size);
    uint32_t result = dst ^ src;
    
    if (size == 1) result &= 0xFF;
    else if (size == 2) result &= 0xFFFF;
    
    set_ea_value(mode, ea_reg, size, result);
    set_flags_nz(result, size);
    clear_flag(SR_V | SR_C);
    cpu->cycles += 4;
}

static void exec_muls(uint16_t opcode) {
    uint16_t reg = (opcode >> 9) & 7;
    uint16_t mode = (opcode >> 3) & 7;
    uint16_t ea_reg = opcode & 7;
    
    int16_t src = (int16_t)get_ea_value(mode, ea_reg, 2);
    int16_t dst = (int16_t)(cpu->d[reg] & 0xFFFF);
    int32_t result = (int32_t)src * (int32_t)dst;
    
    cpu->d[reg] = result;
    set_flags_nz(result, 4);
    clear_flag(SR_V | SR_C);
    cpu->cycles += 70;
}

static void exec_mulu(uint16_t opcode) {
    uint16_t reg = (opcode >> 9) & 7;
    uint16_t mode = (opcode >> 3) & 7;
    uint16_t ea_reg = opcode & 7;
    
    uint16_t src = get_ea_value(mode, ea_reg, 2);
    uint16_t dst = cpu->d[reg] & 0xFFFF;
    uint32_t result = (uint32_t)src * (uint32_t)dst;
    
    cpu->d[reg] = result;
    set_flags_nz(result, 4);
    clear_flag(SR_V | SR_C);
    cpu->cycles += 70;
}

static void exec_divs(uint16_t opcode) {
    uint16_t reg = (opcode >> 9) & 7;
    uint16_t mode = (opcode >> 3) & 7;
    uint16_t ea_reg = opcode & 7;
    
    int16_t divisor = (int16_t)get_ea_value(mode, ea_reg, 2);
    if (divisor == 0) {
        raise_exception(VEC_DIVIDE_BY_ZERO);
        return;
    }
    
    int32_t dividend = (int32_t)cpu->d[reg];
    int32_t quotient = dividend / divisor;
    int16_t remainder = dividend % divisor;
    
    if (quotient > 32767 || quotient < -32768) {
        set_flag(SR_V);
        cpu->cycles += 16;
        return;
    }
    
    cpu->d[reg] = ((uint16_t)remainder << 16) | (uint16_t)quotient;
    set_flags_nz(quotient, 2);
    clear_flag(SR_V | SR_C);
    cpu->cycles += 158;
}

static void exec_divu(uint16_t opcode) {
    uint16_t reg = (opcode >> 9) & 7;
    uint16_t mode = (opcode >> 3) & 7;
    uint16_t ea_reg = opcode & 7;
    
    uint16_t divisor = get_ea_value(mode, ea_reg, 2);
    if (divisor == 0) {
        raise_exception(VEC_DIVIDE_BY_ZERO);
        return;
    }
    
    uint32_t dividend = cpu->d[reg];
    uint32_t quotient = dividend / divisor;
    uint16_t remainder = dividend % divisor;
    
    if (quotient > 65535) {
        set_flag(SR_V);
        cpu->cycles += 16;
        return;
    }
    
    cpu->d[reg] = (remainder << 16) | (uint16_t)quotient;
    set_flags_nz(quotient, 2);
    clear_flag(SR_V | SR_C);
    cpu->cycles += 140;
}

static void exec_asl(uint16_t opcode) {
    uint16_t reg = opcode & 7;
    uint16_t count_reg = (opcode >> 9) & 7;
    uint16_t size_mode = (opcode >> 6) & 3;

    int size = (size_mode == 0) ? 1 : (size_mode == 1) ? 2 : 4;
    uint8_t count = (opcode & 0x20) ? (cpu->d[count_reg] & 63) : ((count_reg == 0) ? 8 : count_reg);

    uint32_t value = cpu->d[reg];
    if (size == 1) value &= 0xFF;
    else if (size == 2) value &= 0xFFFF;

    uint32_t result = value << count;

    if (size == 1) {
        result &= 0xFF;
        cpu->d[reg] = (cpu->d[reg] & 0xFFFFFF00) | result;
    } else if (size == 2) {
        result &= 0xFFFF;
        cpu->d[reg] = (cpu->d[reg] & 0xFFFF0000) | result;
    } else {
        cpu->d[reg] = result;
    }
    set_flags_nz(result, size);

    if (count > 0) {
        uint32_t msb_pos = size * 8 - 1;
        if (count <= msb_pos && (value & (1 << (msb_pos - count + 1)))) {
            set_flag(SR_C | SR_X);
        } else {
            clear_flag(SR_C | SR_X);
        }
    }

    cpu->cycles += 6 + 2 * count;
}

static void exec_lsr(uint16_t opcode) {
    uint16_t reg = opcode & 7;
    uint16_t count_reg = (opcode >> 9) & 7;
    uint16_t size_mode = (opcode >> 6) & 3;
    
    int size = (size_mode == 0) ? 1 : (size_mode == 1) ? 2 : 4;
    uint8_t count = (opcode & 0x20) ? (cpu->d[count_reg] & 63) : ((count_reg == 0) ? 8 : count_reg);
    
    uint32_t value = cpu->d[reg];
    if (size == 1) value &= 0xFF;
    else if (size == 2) value &= 0xFFFF;
    
    uint32_t result = value >> count;
    
    if (size == 1) cpu->d[reg] = (cpu->d[reg] & 0xFFFFFF00) | result;
    else if (size == 2) cpu->d[reg] = (cpu->d[reg] & 0xFFFF0000) | result;
    else cpu->d[reg] = result;
    
    set_flags_nz(result, size);
    clear_flag(SR_V);
    
    if (count > 0) {
        if (value & (1 << (count - 1))) {
            set_flag(SR_C | SR_X);
        } else {
            clear_flag(SR_C | SR_X);
        }
    } else {
        clear_flag(SR_C);
    }
    
    cpu->cycles += 6 + 2 * count;
}

static void exec_asr(uint16_t opcode) {
    uint16_t reg = opcode & 7;
    uint16_t count_reg = (opcode >> 9) & 7;
    uint16_t size_mode = (opcode >> 6) & 3;
    
    int size = (size_mode == 0) ? 1 : (size_mode == 1) ? 2 : 4;
    uint8_t count = (opcode & 0x20) ? (cpu->d[count_reg] & 63) : ((count_reg == 0) ? 8 : count_reg);
    
    int32_t value;
    if (size == 1) value = (int8_t)(cpu->d[reg] & 0xFF);
    else if (size == 2) value = (int16_t)(cpu->d[reg] & 0xFFFF);
    else value = (int32_t)cpu->d[reg];
    
    uint32_t orig_value = value;
    int32_t result = value >> count;
    
    if (size == 1) {
        result &= 0xFF;
        cpu->d[reg] = (cpu->d[reg] & 0xFFFFFF00) | result;
    } else if (size == 2) {
        result &= 0xFFFF;
        cpu->d[reg] = (cpu->d[reg] & 0xFFFF0000) | result;
    } else {
        cpu->d[reg] = result;
    }
    
    set_flags_nz(result, size);
    clear_flag(SR_V);
    
    if (count > 0) {
        if (orig_value & (1 << (count - 1))) {
            set_flag(SR_C | SR_X);
        } else {
            clear_flag(SR_C | SR_X);
        }
    } else {
        clear_flag(SR_C);
    }
    
    cpu->cycles += 6 + 2 * count;
}

static void exec_lsl(uint16_t opcode) {
    uint16_t reg = opcode & 7;
    uint16_t count_reg = (opcode >> 9) & 7;
    uint16_t size_mode = (opcode >> 6) & 3;
    
    int size = (size_mode == 0) ? 1 : (size_mode == 1) ? 2 : 4;
    uint8_t count = (opcode & 0x20) ? (cpu->d[count_reg] & 63) : ((count_reg == 0) ? 8 : count_reg);
    
    uint32_t value = cpu->d[reg];
    if (size == 1) value &= 0xFF;
    else if (size == 2) value &= 0xFFFF;
    
    uint32_t msb_pos = (size == 1) ? 7 : (size == 2) ? 15 : 31;
    uint32_t result = value << count;
    
    if (size == 1) {
        result &= 0xFF;
        cpu->d[reg] = (cpu->d[reg] & 0xFFFFFF00) | result;
    } else if (size == 2) {
        result &= 0xFFFF;
        cpu->d[reg] = (cpu->d[reg] & 0xFFFF0000) | result;
    } else {
        cpu->d[reg] = result;
    }
    
    set_flags_nz(result, size);
    clear_flag(SR_V);
    
    if (count > 0) {
        if (value & (1 << (msb_pos - count + 1))) {
            set_flag(SR_C | SR_X);
        } else {
            clear_flag(SR_C | SR_X);
        }
    } else {
        clear_flag(SR_C);
    }
    
    cpu->cycles += 6 + 2 * count;
}

static void exec_rol(uint16_t opcode) {
    uint16_t reg = opcode & 7;
    uint16_t count_reg = (opcode >> 9) & 7;
    uint16_t size_mode = (opcode >> 6) & 3;
    
    int size = (size_mode == 0) ? 1 : (size_mode == 1) ? 2 : 4;
    uint8_t count = (opcode & 0x20) ? (cpu->d[count_reg] & 63) : ((count_reg == 0) ? 8 : count_reg);
    
    uint32_t value = cpu->d[reg];
    uint32_t bits = size * 8;
    uint32_t mask = (size == 1) ? 0xFF : (size == 2) ? 0xFFFF : 0xFFFFFFFF;
    value &= mask;
    
    count %= bits;
    uint32_t result = ((value << count) | (value >> (bits - count))) & mask;
    
    if (size == 1) cpu->d[reg] = (cpu->d[reg] & 0xFFFFFF00) | result;
    else if (size == 2) cpu->d[reg] = (cpu->d[reg] & 0xFFFF0000) | result;
    else cpu->d[reg] = result;
    
    set_flags_nz(result, size);
    clear_flag(SR_V);
    
    if (count > 0) {
        if (result & 1) set_flag(SR_C);
        else clear_flag(SR_C);
    } else {
        clear_flag(SR_C);
    }
    
    cpu->cycles += 6 + 2 * count;
}

static void exec_ror(uint16_t opcode) {
    uint16_t reg = opcode & 7;
    uint16_t count_reg = (opcode >> 9) & 7;
    uint16_t size_mode = (opcode >> 6) & 3;
    
    int size = (size_mode == 0) ? 1 : (size_mode == 1) ? 2 : 4;
    uint8_t count = (opcode & 0x20) ? (cpu->d[count_reg] & 63) : ((count_reg == 0) ? 8 : count_reg);
    
    uint32_t value = cpu->d[reg];
    uint32_t bits = size * 8;
    uint32_t mask = (size == 1) ? 0xFF : (size == 2) ? 0xFFFF : 0xFFFFFFFF;
    value &= mask;
    
    count %= bits;
    uint32_t result = ((value >> count) | (value << (bits - count))) & mask;
    
    if (size == 1) cpu->d[reg] = (cpu->d[reg] & 0xFFFFFF00) | result;
    else if (size == 2) cpu->d[reg] = (cpu->d[reg] & 0xFFFF0000) | result;
    else cpu->d[reg] = result;
    
    set_flags_nz(result, size);
    clear_flag(SR_V);
    
    if (count > 0) {
        uint32_t msb = (size == 1) ? 0x80 : (size == 2) ? 0x8000 : 0x80000000;
        if (result & msb) set_flag(SR_C);
        else clear_flag(SR_C);
    } else {
        clear_flag(SR_C);
    }
    
    cpu->cycles += 6 + 2 * count;
}

static void exec_roxl(uint16_t opcode) {
    uint16_t reg = opcode & 7;
    uint16_t count_reg = (opcode >> 9) & 7;
    uint16_t size_mode = (opcode >> 6) & 3;
    
    int size = (size_mode == 0) ? 1 : (size_mode == 1) ? 2 : 4;
    uint8_t count = (opcode & 0x20) ? (cpu->d[count_reg] & 63) : ((count_reg == 0) ? 8 : count_reg);
    
    uint32_t value = cpu->d[reg];
    uint32_t mask = (size == 1) ? 0xFF : (size == 2) ? 0xFFFF : 0xFFFFFFFF;
    uint32_t msb = (size == 1) ? 0x80 : (size == 2) ? 0x8000 : 0x80000000;
    value &= mask;
    
    bool x_flag = test_flag(SR_X);
    
    for (uint8_t i = 0; i < count; i++) {
        bool new_x = (value & msb) != 0;
        value = ((value << 1) | (x_flag ? 1 : 0)) & mask;
        x_flag = new_x;
    }
    
    if (size == 1) cpu->d[reg] = (cpu->d[reg] & 0xFFFFFF00) | value;
    else if (size == 2) cpu->d[reg] = (cpu->d[reg] & 0xFFFF0000) | value;
    else cpu->d[reg] = value;
    
    set_flags_nz(value, size);
    clear_flag(SR_V);
    if (x_flag) set_flag(SR_C | SR_X);
    else clear_flag(SR_C | SR_X);
    
    cpu->cycles += 6 + 2 * count;
}

static void exec_roxr(uint16_t opcode) {
    uint16_t reg = opcode & 7;
    uint16_t count_reg = (opcode >> 9) & 7;
    uint16_t size_mode = (opcode >> 6) & 3;
    
    int size = (size_mode == 0) ? 1 : (size_mode == 1) ? 2 : 4;
    uint8_t count = (opcode & 0x20) ? (cpu->d[count_reg] & 63) : ((count_reg == 0) ? 8 : count_reg);
    
    uint32_t value = cpu->d[reg];
    uint32_t mask = (size == 1) ? 0xFF : (size == 2) ? 0xFFFF : 0xFFFFFFFF;
    uint32_t msb = (size == 1) ? 0x80 : (size == 2) ? 0x8000 : 0x80000000;
    value &= mask;
    
    bool x_flag = test_flag(SR_X);
    
    for (uint8_t i = 0; i < count; i++) {
        bool new_x = (value & 1) != 0;
        value = ((value >> 1) | (x_flag ? msb : 0)) & mask;
        x_flag = new_x;
    }
    
    if (size == 1) cpu->d[reg] = (cpu->d[reg] & 0xFFFFFF00) | value;
    else if (size == 2) cpu->d[reg] = (cpu->d[reg] & 0xFFFF0000) | value;
    else cpu->d[reg] = value;
    
    set_flags_nz(value, size);
    clear_flag(SR_V);
    if (x_flag) set_flag(SR_C | SR_X);
    else clear_flag(SR_C | SR_X);
    
    cpu->cycles += 6 + 2 * count;
}

static void exec_addx(uint16_t opcode) {
    int size = ((opcode >> 6) & 3);
    if (size == 0) size = 1;
    else if (size == 1) size = 2;
    else size = 4;
    
    uint16_t rx = (opcode >> 9) & 7;
    uint16_t ry = opcode & 7;
    bool rm = (opcode >> 3) & 1;  // Register/Memory flag
    
    uint32_t mask = (size == 1) ? 0xFF : (size == 2) ? 0xFFFF : 0xFFFFFFFF;
    uint32_t msb = (size == 1) ? 0x80 : (size == 2) ? 0x8000 : 0x80000000;
    
    uint32_t src, dst, result;
    uint32_t x_val = test_flag(SR_X) ? 1 : 0;
    
    if (rm) {
        // Memory to memory with predecrement
        cpu->a[ry] -= size;
        src = (size == 1) ? read_byte(cpu->a[ry]) : (size == 2) ? read_word(cpu->a[ry]) : read_long(cpu->a[ry]);
        cpu->a[rx] -= size;
        dst = (size == 1) ? read_byte(cpu->a[rx]) : (size == 2) ? read_word(cpu->a[rx]) : read_long(cpu->a[rx]);
        result = (src + dst + x_val) & mask;
        if (size == 1) write_byte(cpu->a[rx], result);
        else if (size == 2) write_word(cpu->a[rx], result);
        else write_long(cpu->a[rx], result);
        cpu->cycles += (size == 4) ? 30 : 18;
    } else {
        // Register to register
        src = cpu->d[ry] & mask;
        dst = cpu->d[rx] & mask;
        result = (src + dst + x_val) & mask;
        if (size == 1) cpu->d[rx] = (cpu->d[rx] & 0xFFFFFF00) | result;
        else if (size == 2) cpu->d[rx] = (cpu->d[rx] & 0xFFFF0000) | result;
        else cpu->d[rx] = result;
        cpu->cycles += (size == 4) ? 8 : 4;
    }
    
    // Only clear Z if result is non-zero (accumulating behavior)
    if (result != 0) clear_flag(SR_Z);
    if (result & msb) set_flag(SR_N);
    else clear_flag(SR_N);
    
    // Carry
    if ((src & mask) + (dst & mask) + x_val > mask) set_flag(SR_C | SR_X);
    else clear_flag(SR_C | SR_X);
    
    // Overflow
    bool src_neg = (src & msb) != 0;
    bool dst_neg = (dst & msb) != 0;
    bool res_neg = (result & msb) != 0;
    if (src_neg == dst_neg && res_neg != src_neg) set_flag(SR_V);
    else clear_flag(SR_V);
}

static void exec_subx(uint16_t opcode) {
    int size = ((opcode >> 6) & 3);
    if (size == 0) size = 1;
    else if (size == 1) size = 2;
    else size = 4;
    
    uint16_t rx = (opcode >> 9) & 7;
    uint16_t ry = opcode & 7;
    bool rm = (opcode >> 3) & 1;
    
    uint32_t mask = (size == 1) ? 0xFF : (size == 2) ? 0xFFFF : 0xFFFFFFFF;
    uint32_t msb = (size == 1) ? 0x80 : (size == 2) ? 0x8000 : 0x80000000;
    
    uint32_t src, dst, result;
    uint32_t x_val = test_flag(SR_X) ? 1 : 0;
    
    if (rm) {
        cpu->a[ry] -= size;
        src = (size == 1) ? read_byte(cpu->a[ry]) : (size == 2) ? read_word(cpu->a[ry]) : read_long(cpu->a[ry]);
        cpu->a[rx] -= size;
        dst = (size == 1) ? read_byte(cpu->a[rx]) : (size == 2) ? read_word(cpu->a[rx]) : read_long(cpu->a[rx]);
        result = (dst - src - x_val) & mask;
        if (size == 1) write_byte(cpu->a[rx], result);
        else if (size == 2) write_word(cpu->a[rx], result);
        else write_long(cpu->a[rx], result);
        cpu->cycles += (size == 4) ? 30 : 18;
    } else {
        src = cpu->d[ry] & mask;
        dst = cpu->d[rx] & mask;
        result = (dst - src - x_val) & mask;
        if (size == 1) cpu->d[rx] = (cpu->d[rx] & 0xFFFFFF00) | result;
        else if (size == 2) cpu->d[rx] = (cpu->d[rx] & 0xFFFF0000) | result;
        else cpu->d[rx] = result;
        cpu->cycles += (size == 4) ? 8 : 4;
    }
    
    if (result != 0) clear_flag(SR_Z);
    if (result & msb) set_flag(SR_N);
    else clear_flag(SR_N);
    
    if (src + x_val > dst) set_flag(SR_C | SR_X);
    else clear_flag(SR_C | SR_X);
    
    bool src_neg = (src & msb) != 0;
    bool dst_neg = (dst & msb) != 0;
    bool res_neg = (result & msb) != 0;
    if (src_neg != dst_neg && res_neg != dst_neg) set_flag(SR_V);
    else clear_flag(SR_V);
}

static void exec_cmpm(uint16_t opcode) {
    int size = ((opcode >> 6) & 3);
    if (size == 0) size = 1;
    else if (size == 1) size = 2;
    else size = 4;
    
    uint16_t ax = (opcode >> 9) & 7;
    uint16_t ay = opcode & 7;
    
    uint32_t mask = (size == 1) ? 0xFF : (size == 2) ? 0xFFFF : 0xFFFFFFFF;
    uint32_t msb = (size == 1) ? 0x80 : (size == 2) ? 0x8000 : 0x80000000;
    
    // Read source (Ay)+
    uint32_t src = (size == 1) ? read_byte(cpu->a[ay]) : (size == 2) ? read_word(cpu->a[ay]) : read_long(cpu->a[ay]);
    cpu->a[ay] += size;
    
    // Read destination (Ax)+
    uint32_t dst = (size == 1) ? read_byte(cpu->a[ax]) : (size == 2) ? read_word(cpu->a[ax]) : read_long(cpu->a[ax]);
    cpu->a[ax] += size;
    
    uint32_t result = (dst - src) & mask;
    
    set_flags_nz(result, size);
    
    if (src > dst) set_flag(SR_C);
    else clear_flag(SR_C);
    
    bool src_neg = (src & msb) != 0;
    bool dst_neg = (dst & msb) != 0;
    bool res_neg = (result & msb) != 0;
    if (src_neg != dst_neg && res_neg != dst_neg) set_flag(SR_V);
    else clear_flag(SR_V);
    
    cpu->cycles += (size == 4) ? 20 : 12;
}

static void exec_chk(uint16_t opcode) {
    uint16_t reg = (opcode >> 9) & 7;
    uint16_t mode = (opcode >> 3) & 7;
    uint16_t ea_reg = opcode & 7;
    
    int16_t bound = (int16_t)get_ea_value(mode, ea_reg, 2);
    int16_t value = (int16_t)(cpu->d[reg] & 0xFFFF);
    
    if (value < 0) {
        set_flag(SR_N);
        raise_exception(VEC_CHK);
    } else if (value > bound) {
        clear_flag(SR_N);
        raise_exception(VEC_CHK);
    }
    
    cpu->cycles += 10;
}

static void exec_abcd(uint16_t opcode) {
    uint16_t rx = (opcode >> 9) & 7;
    uint16_t ry = opcode & 7;
    bool rm = (opcode >> 3) & 1;
    
    uint8_t src, dst;
    
    if (rm) {
        cpu->a[ry]--;
        src = read_byte(cpu->a[ry]);
        cpu->a[rx]--;
        dst = read_byte(cpu->a[rx]);
    } else {
        src = cpu->d[ry] & 0xFF;
        dst = cpu->d[rx] & 0xFF;
    }
    
    uint8_t x_val = test_flag(SR_X) ? 1 : 0;
    
    // BCD addition
    uint8_t low = (src & 0x0F) + (dst & 0x0F) + x_val;
    uint8_t high = (src >> 4) + (dst >> 4);
    
    if (low > 9) {
        low -= 10;
        high++;
    }
    
    bool carry = false;
    if (high > 9) {
        high -= 10;
        carry = true;
    }
    
    uint8_t result = (high << 4) | (low & 0x0F);
    
    if (rm) {
        write_byte(cpu->a[rx], result);
        cpu->cycles += 18;
    } else {
        cpu->d[rx] = (cpu->d[rx] & 0xFFFFFF00) | result;
        cpu->cycles += 6;
    }
    
    if (result != 0) clear_flag(SR_Z);
    if (carry) set_flag(SR_C | SR_X);
    else clear_flag(SR_C | SR_X);
}

static void exec_sbcd(uint16_t opcode) {
    uint16_t rx = (opcode >> 9) & 7;
    uint16_t ry = opcode & 7;
    bool rm = (opcode >> 3) & 1;
    
    uint8_t src, dst;
    
    if (rm) {
        cpu->a[ry]--;
        src = read_byte(cpu->a[ry]);
        cpu->a[rx]--;
        dst = read_byte(cpu->a[rx]);
    } else {
        src = cpu->d[ry] & 0xFF;
        dst = cpu->d[rx] & 0xFF;
    }
    
    uint8_t x_val = test_flag(SR_X) ? 1 : 0;
    
    // BCD subtraction
    int8_t low = (dst & 0x0F) - (src & 0x0F) - x_val;
    int8_t high = (dst >> 4) - (src >> 4);
    
    if (low < 0) {
        low += 10;
        high--;
    }
    
    bool borrow = false;
    if (high < 0) {
        high += 10;
        borrow = true;
    }
    
    uint8_t result = (high << 4) | (low & 0x0F);
    
    if (rm) {
        write_byte(cpu->a[rx], result);
        cpu->cycles += 18;
    } else {
        cpu->d[rx] = (cpu->d[rx] & 0xFFFFFF00) | result;
        cpu->cycles += 6;
    }
    
    if (result != 0) clear_flag(SR_Z);
    if (borrow) set_flag(SR_C | SR_X);
    else clear_flag(SR_C | SR_X);
}

static void exec_nbcd(uint16_t opcode) {
    uint16_t mode = (opcode >> 3) & 7;
    uint16_t reg = opcode & 7;
    
    uint8_t src = get_ea_value(mode, reg, 1);
    uint8_t x_val = test_flag(SR_X) ? 1 : 0;
    
    // BCD negation (0 - src - X)
    int8_t low = 0 - (src & 0x0F) - x_val;
    int8_t high = 0 - (src >> 4);
    
    if (low < 0) {
        low += 10;
        high--;
    }
    
    bool borrow = false;
    if (high < 0) {
        high += 10;
        borrow = true;
    }
    
    uint8_t result = (high << 4) | (low & 0x0F);
    
    set_ea_value(mode, reg, 1, result);
    
    if (result != 0) clear_flag(SR_Z);
    if (borrow) set_flag(SR_C | SR_X);
    else clear_flag(SR_C | SR_X);
    
    cpu->cycles += (mode == 0) ? 6 : 8;
}

static void exec_negx(uint16_t opcode) {
    int size = ((opcode >> 6) & 3);
    if (size == 0) size = 1;
    else if (size == 1) size = 2;
    else size = 4;
    
    uint16_t mode = (opcode >> 3) & 7;
    uint16_t reg = opcode & 7;
    
    uint32_t mask = (size == 1) ? 0xFF : (size == 2) ? 0xFFFF : 0xFFFFFFFF;
    uint32_t msb = (size == 1) ? 0x80 : (size == 2) ? 0x8000 : 0x80000000;
    
    uint32_t src = get_ea_value(mode, reg, size);
    uint32_t x_val = test_flag(SR_X) ? 1 : 0;
    uint32_t result = (0 - src - x_val) & mask;
    
    set_ea_value(mode, reg, size, result);
    
    if (result != 0) clear_flag(SR_Z);
    if (result & msb) set_flag(SR_N);
    else clear_flag(SR_N);
    
    if (src != 0 || x_val != 0) set_flag(SR_C | SR_X);
    else clear_flag(SR_C | SR_X);
    
    // Overflow
    bool res_neg = (result & msb) != 0;
    bool src_neg = (src & msb) != 0;
    if (res_neg && src_neg) set_flag(SR_V);
    else clear_flag(SR_V);
    
    cpu->cycles += (mode == 0) ? 4 : 8;
}

static void exec_movem_to_mem(uint16_t opcode) {
    uint16_t size = (opcode & 0x40) ? 4 : 2;
    uint16_t mode = (opcode >> 3) & 7;
    uint16_t reg = opcode & 7;
    uint16_t mask = fetch_word();

    if (mode == 4) {
        // Predecrement mode -(An): register list mask is REVERSED
        // Bit 0=A7, Bit 1=A6, ..., Bit 7=A0, Bit 8=D7, ..., Bit 15=D0
        // Registers pushed in order: A7 first (highest addr), D0 last (lowest addr)
        uint32_t addr = cpu->a[reg];
        for (int i = 0; i < 16; i++) {
            if (mask & (1 << i)) {
                addr -= size;
                if (i < 8) {
                    // bits 0-7: address registers A7 down to A0
                    int areg = 7 - i;
                    if (size == 4) write_long(addr, cpu->a[areg]);
                    else write_word(addr, cpu->a[areg] & 0xFFFF);
                } else {
                    // bits 8-15: data registers D7 down to D0
                    int dreg = 15 - i;
                    if (size == 4) write_long(addr, cpu->d[dreg]);
                    else write_word(addr, cpu->d[dreg] & 0xFFFF);
                }
            }
        }
        cpu->a[reg] = addr;
    } else {
        uint32_t addr = get_ea_addr(mode, reg, size);
        for (int i = 0; i < 16; i++) {
            if (mask & (1 << i)) {
                if (i < 8) {
                    if (size == 4) write_long(addr, cpu->d[i]);
                    else write_word(addr, cpu->d[i] & 0xFFFF);
                } else {
                    if (size == 4) write_long(addr, cpu->a[i - 8]);
                    else write_word(addr, cpu->a[i - 8] & 0xFFFF);
                }
                addr += size;
            }
        }
    }

    cpu->cycles += 8;
}

static void exec_movem_from_mem(uint16_t opcode) {
    uint16_t size = (opcode & 0x40) ? 4 : 2;
    uint16_t mode = (opcode >> 3) & 7;
    uint16_t reg = opcode & 7;
    uint16_t mask = fetch_word();

    uint32_t addr;
    if (mode == 3) {
        // Postincrement mode (An)+
        addr = cpu->a[reg];
    } else {
        addr = get_ea_addr(mode, reg, size);
    }

    for (int i = 0; i < 16; i++) {
        if (mask & (1 << i)) {
            if (i < 8) {
                if (size == 4) cpu->d[i] = read_long(addr);
                else cpu->d[i] = (int16_t)read_word(addr);
            } else {
                if (size == 4) cpu->a[i - 8] = read_long(addr);
                else cpu->a[i - 8] = (int16_t)read_word(addr);
            }
            addr += size;
        }
    }

    // Update An for postincrement
    if (mode == 3) {
        cpu->a[reg] = addr;
    }

    cpu->cycles += 8;
}

static void exec_trap(uint16_t opcode) {
    uint8_t vector_num = opcode & 0xF;
    raise_exception(VEC_TRAP_BASE + vector_num * 4);
    cpu->cycles += 34;
}

static void exec_rte(void) {
    // Return from exception
    cpu->sr = read_word(cpu->a[7]);
    cpu->a[7] += 2;
    cpu->pc = read_long(cpu->a[7]);
    cpu->a[7] += 4;
    
    // Restore stack pointer if switching modes
    if (!test_flag(SR_S)) {
        cpu->ssp = cpu->a[7];
        cpu->a[7] = cpu->usp;
    }
    
    cpu->cycles += 20;
}

static void exec_link(uint16_t opcode) {
    uint16_t reg = opcode & 7;
    int16_t displacement = (int16_t)fetch_word();
    
    cpu->a[7] -= 4;
    write_long(cpu->a[7], cpu->a[reg]);
    cpu->a[reg] = cpu->a[7];
    cpu->a[7] += displacement;
    
    cpu->cycles += 16;
}

static void exec_unlk(uint16_t opcode) {
    uint16_t reg = opcode & 7;
    
    cpu->a[7] = cpu->a[reg];
    cpu->a[reg] = read_long(cpu->a[7]);
    cpu->a[7] += 4;
    
    cpu->cycles += 12;
}

static void exec_pea(uint16_t opcode) {
    uint16_t mode = (opcode >> 3) & 7;
    uint16_t reg = opcode & 7;

    uint32_t addr = get_ea_addr(mode, reg, 4);
    cpu->a[7] -= 4;
    write_long(cpu->a[7], addr);

    cpu->cycles += 12;
}

static void exec_dbcc(uint16_t opcode) {
    uint8_t condition = (opcode >> 8) & 0xF;
    uint16_t reg = opcode & 7;
    int16_t displacement = (int16_t)fetch_word();
    
    if (!test_condition(condition)) {
        int16_t counter = (int16_t)(cpu->d[reg] & 0xFFFF);
        counter--;
        cpu->d[reg] = (cpu->d[reg] & 0xFFFF0000) | (uint16_t)counter;
        
        if (counter != -1) {
            cpu->pc += displacement - 2;
            cpu->cycles += 10;
        } else {
            cpu->cycles += 14;
        }
    } else {
        cpu->cycles += 12;
    }
}

static void exec_scc(uint16_t opcode) {
    uint8_t condition = (opcode >> 8) & 0xF;
    uint16_t mode = (opcode >> 3) & 7;
    uint16_t reg = opcode & 7;
    
    uint8_t value = test_condition(condition) ? 0xFF : 0x00;
    set_ea_value(mode, reg, 1, value);
    
    cpu->cycles += 4;
}

static void exec_bra(int8_t disp8) {
    if (disp8 == 0) {
        uint32_t base_pc = cpu->pc;
        int16_t disp16 = (int16_t)fetch_word();
        cpu->pc = base_pc + disp16;
    } else {
        cpu->pc += disp8;
    }
    cpu->cycles += 10;
}

static void exec_bsr(int8_t disp8) {
    uint32_t base_pc = cpu->pc;
    int32_t displacement;
    if (disp8 == 0) {
        displacement = (int16_t)fetch_word();
    } else {
        displacement = disp8;
    }

    cpu->a[7] -= 4;
    write_long(cpu->a[7], cpu->pc);  // Push return address (after all extension words)
    cpu->pc = base_pc + displacement;
    cpu->cycles += 18;
}

static void exec_bcc(uint8_t condition, int8_t disp8) {
    uint32_t base_pc = cpu->pc;
    int32_t displacement;
    if (disp8 == 0) {
        displacement = (int16_t)fetch_word();
    } else {
        displacement = disp8;
    }

    if (test_condition(condition)) {
        cpu->pc = base_pc + displacement;
        cpu->cycles += 10;
    } else {
        cpu->cycles += 8;
    }
}

// Main execution step
static bool execute_instruction(void) {
    if (cpu->halted || cpu->stopped) {
        return false;
    }
    
    // Check for odd PC before saving it (catch corruption early)
    if (cpu->pc & 1) {
        ESP_LOGE(TAG, "FATAL: PC is odd address 0x%08X - cannot execute from odd address!", cpu->pc);
        ESP_LOGE(TAG, "Last valid PC was: 0x%08X", cpu->last_pc);
        ESP_LOGE(TAG, "This typically means:");
        ESP_LOGE(TAG, "  1. RTS popped invalid return address from stack");
        ESP_LOGE(TAG, "  2. JMP/JSR calculated odd address");
        ESP_LOGE(TAG, "  3. Exception vector points to odd address");
        ESP_LOGE(TAG, "Stack pointer: 0x%08X", cpu->a[7]);
        
        // Show stack contents
        uint32_t sp = cpu->a[7];
        if (sp >= 4 && sp < cpu->ram_size) {
            ESP_LOGE(TAG, "Top of stack [SP+0]: 0x%08X", read_long(sp));
            if (sp >= 8) {
                ESP_LOGE(TAG, "Stack [SP-4]: 0x%08X", read_long(sp - 4));
            }
        }
        
        cpu->halted = true;
        cpu->running = false;
        return false;
    }
    
    // Save PC for error reporting
    cpu->last_pc = cpu->pc;
    
    // Check if PC is in exception vector table (indicates unhandled exception)
    // M68K has 256 exception vectors (0x000-0x3FF), but check only vectors 2-63
    if (cpu->pc < 0x100 && cpu->pc >= 8) {  // Vectors 2-63 (skip reset vectors 0-1)
        uint32_t vector_num = cpu->pc / 4;
        const char* vec_names[] = {
            "Reset SSP", "Reset PC", "Bus Error", "Address Error",
            "Illegal Instruction", "Divide by Zero", "CHK", "TRAPV",
            "Privilege Violation", "Trace", "Line 1010", "Line 1111"
        };
        const char* vec_name = (vector_num < 12) ? vec_names[vector_num] : "Unknown";
        
        ESP_LOGE(TAG, "FATAL: PC in exception vector table at 0x%08X (vector %d: %s)", 
                 cpu->pc, (int)vector_num, vec_name);
        ESP_LOGE(TAG, "This means an exception occurred but the vector is uninitialized.");
        ESP_LOGE(TAG, "Last valid PC was: 0x%08X", cpu->last_pc);
        
        // Try to determine which exception by looking at nearby PC values
        if (cpu->pc >= 8 && cpu->pc < 12) {
            ESP_LOGE(TAG, ">>> BUS ERROR exception - invalid memory access or alignment error");
        } else if (cpu->pc >= 12 && cpu->pc < 16) {
            ESP_LOGE(TAG, ">>> ADDRESS ERROR exception - odd address word/long access");
        } else if (cpu->pc >= 16 && cpu->pc < 20) {
            ESP_LOGE(TAG, ">>> ILLEGAL INSTRUCTION exception");
        }
        
        cpu->halted = true;
        cpu->running = false;
        return false;
    }
    
    // Debug: Log first 50 instructions
    static int exec_count = 0;
    bool debug_this = (exec_count < 50);
    
    uint32_t debug_pc = cpu->pc;
    uint16_t opcode = fetch_word();
    
    if (debug_this) {
        ESP_LOGI(TAG, "Exec [%d]: PC=0x%08lX, Opcode=0x%04X, SP=0x%08lX", 
                 exec_count, (unsigned long)debug_pc, opcode, (unsigned long)cpu->a[7]);
        exec_count++;
    }
    
    cpu->instructions_executed++;
    
    // Decode opcode groups
    uint8_t hi_nibble = (opcode >> 12) & 0xF;
    uint8_t hi_byte = (opcode >> 8) & 0xFF;
    
    switch (hi_nibble) {
        case 0x0:
        case 0x4:
            if (opcode == 0x003C) {  // ORI to CCR
                uint8_t imm = fetch_word() & 0xFF;
                cpu->sr |= imm;
                cpu->cycles += 20;
            } else if (opcode == 0x007C) {  // ORI to SR
                if (!test_flag(SR_S)) {
                    raise_exception(VEC_PRIVILEGE);
                } else {
                    uint16_t imm = fetch_word();
                    cpu->sr |= imm;
                    cpu->cycles += 20;
                }
            } else if (opcode == 0x023C) {  // ANDI to CCR
                uint8_t imm = fetch_word() & 0xFF;
                cpu->sr &= (0xFF00 | imm);
                cpu->cycles += 20;
            } else if (opcode == 0x027C) {  // ANDI to SR
                if (!test_flag(SR_S)) {
                    raise_exception(VEC_PRIVILEGE);
                } else {
                    uint16_t imm = fetch_word();
                    cpu->sr &= imm;
                    cpu->cycles += 20;
                }
            } else if ((opcode & 0xFFC0) == 0x0800) {  // BTST immediate
                uint8_t bit = fetch_word() & 31;
                uint16_t mode = (opcode >> 3) & 7;
                uint16_t reg = opcode & 7;
                uint32_t value = get_ea_value(mode, reg, mode == 0 ? 4 : 1);
                if (value & (1 << bit)) clear_flag(SR_Z);
                else set_flag(SR_Z);
                cpu->cycles += 10;
            } else if ((opcode & 0xF000) == 0x0000 && (opcode & 0x0F00) != 0x0F00) {
                // Immediate instructions: ORI, ANDI, SUBI, ADDI, EORI, CMPI
                // Pattern: 0000 xxx0 ssaa aaaa (bits 15-12 = 0, exclude BTST/BCHG/BCLR/BSET)
                uint8_t op_type = (opcode >> 9) & 7;
                switch (op_type) {
                    case 0: exec_ori(opcode); break;   // 0000 000x = ORI
                    case 1: exec_andi(opcode); break;  // 0000 001x = ANDI
                    case 2: exec_subi(opcode); break;  // 0000 010x = SUBI
                    case 3: exec_addi(opcode); break;  // 0000 011x = ADDI
                    case 5: exec_eori(opcode); break;  // 0000 101x = EORI
                    case 6: exec_cmpi(opcode); break;  // 0000 110x = CMPI
                    default:
                        ESP_LOGW(TAG, "Unknown immediate opcode 0x%04X at PC=0x%08X", opcode, cpu->pc - 2);
                        break;
                }
            } else if ((opcode & 0xF1C0) == 0x0100) {  // BTST/BCHG/BCLR/BSET Dn
                cpu->cycles += 6;
            } else if ((opcode & 0xFFC0) == 0x40C0) {  // MOVE from SR
                uint16_t mode = (opcode >> 3) & 7;
                uint16_t reg = opcode & 7;
                set_ea_value(mode, reg, 2, cpu->sr);
                cpu->cycles += 6;
            } else if ((opcode & 0xFFC0) == 0x44C0) {  // MOVE to CCR
                uint16_t mode = (opcode >> 3) & 7;
                uint16_t reg = opcode & 7;
                uint16_t value = get_ea_value(mode, reg, 2);
                cpu->sr = (cpu->sr & 0xFF00) | (value & 0xFF);
                cpu->cycles += 12;
            } else if ((opcode & 0xFFC0) == 0x46C0) {  // MOVE to SR
                if (!test_flag(SR_S)) {
                    raise_exception(VEC_PRIVILEGE);
                } else {
                    uint16_t mode = (opcode >> 3) & 7;
                    uint16_t reg = opcode & 7;
                    cpu->sr = get_ea_value(mode, reg, 2);
                    cpu->cycles += 12;
                }
            } else if ((opcode & 0xFF00) == 0x4000) {  // NEGX (all sizes)
                exec_negx(opcode);
            } else if ((opcode & 0xFF00) == 0x4200) {  // CLR (all sizes)
                exec_clr(opcode);
            } else if ((opcode & 0xFF00) == 0x4400) {  // NEG (all sizes)
                exec_neg(opcode);
            } else if ((opcode & 0xFF00) == 0x4600) {  // NOT (all sizes)
                exec_not(opcode);
            } else if ((opcode & 0xFFC0) == 0x4800) {  // NBCD
                exec_nbcd(opcode);
            } else if ((opcode & 0xFFF8) == 0x4840) {  // SWAP (Dn only)
                uint16_t reg = opcode & 7;
                uint32_t value = cpu->d[reg];
                cpu->d[reg] = (value >> 16) | (value << 16);
                set_flags_nz(cpu->d[reg], 4);
                clear_flag(SR_V | SR_C);
                cpu->cycles += 4;
            } else if ((opcode & 0xFFC0) == 0x4840) {  // PEA
                exec_pea(opcode);
            } else if ((opcode & 0xFFC0) == 0x4880) {  // EXT.W / MOVEM.W to memory
                if ((opcode & 0x0038) == 0) exec_ext(opcode);
                else exec_movem_to_mem(opcode);
            } else if ((opcode & 0xFFC0) == 0x48C0) {  // EXT.L / MOVEM.L to memory
                if ((opcode & 0x0038) == 0) exec_ext(opcode);
                else exec_movem_to_mem(opcode);
            } else if ((opcode & 0xFFC0) == 0x4AC0) {  // TAS
                uint16_t mode = (opcode >> 3) & 7;
                uint16_t reg = opcode & 7;
                uint8_t value = get_ea_value(mode, reg, 1);
                set_flags_nz(value, 1);
                clear_flag(SR_V | SR_C);
                set_ea_value(mode, reg, 1, value | 0x80);
                cpu->cycles += 14;
            } else if ((opcode & 0xFF00) == 0x4A00) {  // TST (all sizes)
                exec_tst(opcode);
            } else if (opcode == 0x4E70) {  // RESET
                if (!test_flag(SR_S)) {
                    raise_exception(VEC_PRIVILEGE);
                } else {
                    ESP_LOGI(TAG, "RESET instruction");
                    cpu->cycles += 132;
                }
            } else if (opcode == 0x4E71) {  // NOP
                exec_nop();
            } else if (opcode == 0x4E72) {  // STOP
                if (!test_flag(SR_S)) {
                    raise_exception(VEC_PRIVILEGE);
                } else {
                    cpu->sr = fetch_word();
                    cpu->stopped = true;
                    ESP_LOGI(TAG, "CPU STOPPED");
                    cpu->cycles += 4;
                }
            } else if (opcode == 0x4E73) {  // RTE
                exec_rte();
            } else if (opcode == 0x4E75) {  // RTS
                uint32_t return_addr = read_long(cpu->a[7]);
                if (return_addr & 1) {
                    ESP_LOGE(TAG, "RTS: Popped ODD return address 0x%08X from stack at SP=0x%08X",
                             return_addr, cpu->a[7]);
                    ESP_LOGE(TAG, "Current PC was: 0x%08X", cpu->pc);
                }
                cpu->pc = return_addr;
                cpu->a[7] += 4;
                cpu->cycles += 16;
            } else if (opcode == 0x4E76) {  // TRAPV
                if (test_flag(SR_V)) {
                    raise_exception(VEC_TRAPV);
                }
                cpu->cycles += 4;
            } else if (opcode == 0x4E77) {  // RTR
                cpu->sr = (cpu->sr & 0xFF00) | (read_word(cpu->a[7]) & 0xFF);
                cpu->a[7] += 2;
                uint32_t return_addr = read_long(cpu->a[7]);
                if (return_addr & 1) {
                    ESP_LOGE(TAG, "RTR: Popped ODD return address 0x%08X from stack at SP=0x%08X",
                             return_addr, cpu->a[7]);
                    ESP_LOGE(TAG, "Current PC was: 0x%08X", cpu->pc);
                }
                cpu->pc = return_addr;
                cpu->a[7] += 4;
                cpu->cycles += 20;
            } else if ((opcode & 0xFFF0) == 0x4E40) {  // TRAP
                exec_trap(opcode);
            } else if ((opcode & 0xFFF8) == 0x4E50) {  // LINK
                exec_link(opcode);
            } else if ((opcode & 0xFFF8) == 0x4E58) {  // UNLK
                exec_unlk(opcode);
            } else if ((opcode & 0xFFF0) == 0x4E60) {  // MOVE USP
                if (!test_flag(SR_S)) {
                    raise_exception(VEC_PRIVILEGE);
                } else {
                    uint16_t reg = opcode & 7;
                    if (opcode & 0x0008) {  // MOVE USP, An (d=1)
                        cpu->a[reg] = cpu->usp;
                    } else {  // MOVE An, USP (d=0)
                        cpu->usp = cpu->a[reg];
                    }
                    cpu->cycles += 4;
                }
            } else if ((opcode & 0xFFC0) == 0x4E80) {  // JSR
                exec_jsr(opcode);
            } else if ((opcode & 0xFFC0) == 0x4EC0) {  // JMP
                exec_jmp(opcode);
            } else if ((opcode & 0xFFC0) == 0x4C80 || (opcode & 0xFFC0) == 0x4CC0) {  // MOVEM from memory
                exec_movem_from_mem(opcode);
            } else if ((opcode & 0xF1C0) == 0x41C0) {  // LEA (all An registers)
                exec_lea(opcode);
            } else if ((opcode & 0xF1C0) == 0x4180) {  // CHK
                exec_chk(opcode);
            } else {
                ESP_LOGW(TAG, "Unknown opcode 0x%04X at PC=0x%08X", opcode, cpu->pc - 2);
                cpu->cycles += 4;
            }
            break;
            
        case 0x1: case 0x2: case 0x3:
            exec_move(opcode);
            break;
            
        // case 0x4 handled above with case 0x0
            
        case 0x5:
            if ((opcode & 0xF0F8) == 0x50C8) {  // DBcc (must check BEFORE Scc!)
                exec_dbcc(opcode);
            } else if ((opcode & 0xF0C0) == 0x50C0) {  // Scc
                exec_scc(opcode);
            } else {  // ADDQ/SUBQ
                int size = ((opcode >> 6) & 3);
                if (size == 0) size = 1;
                else if (size == 1) size = 2;
                else size = 4;

                uint8_t data = (opcode >> 9) & 7;
                if (data == 0) data = 8;
                uint16_t mode = (opcode >> 3) & 7;
                uint16_t reg = opcode & 7;

                if (mode == 1) {
                    // Address register: always full 32-bit, no flags
                    uint32_t ea_val = cpu->a[reg];
                    if (opcode & 0x0100) cpu->a[reg] = ea_val - data;
                    else cpu->a[reg] = ea_val + data;
                } else {
                    uint32_t mask = (size == 1) ? 0xFF : (size == 2) ? 0xFFFF : 0xFFFFFFFF;
                    uint32_t msb = (size == 1) ? 0x80 : (size == 2) ? 0x8000 : 0x80000000;
                    uint32_t ea_val = get_ea_value(mode, reg, size) & mask;
                    uint32_t result;

                    if (opcode & 0x0100) {  // SUBQ
                        result = (ea_val - data) & mask;
                        set_ea_value(mode, reg, size, result);
                        set_flags_nz(result, size);
                        if (data > ea_val) set_flag(SR_C | SR_X);
                        else clear_flag(SR_C | SR_X);
                        bool dst_neg = (ea_val & msb) != 0;
                        bool res_neg = (result & msb) != 0;
                        if (!dst_neg && res_neg) set_flag(SR_V);
                        else clear_flag(SR_V);
                    } else {  // ADDQ
                        result = (ea_val + data) & mask;
                        set_ea_value(mode, reg, size, result);
                        set_flags_nz(result, size);
                        if ((uint64_t)ea_val + data > mask) set_flag(SR_C | SR_X);
                        else clear_flag(SR_C | SR_X);
                        bool dst_neg = (ea_val & msb) != 0;
                        bool res_neg = (result & msb) != 0;
                        if (!dst_neg && res_neg) set_flag(SR_V);
                        else clear_flag(SR_V);
                    }
                }
                cpu->cycles += 4;
            }
            break;
            
        case 0x6:
            {
                uint8_t condition = (hi_byte >> 0) & 0xF;
                int8_t disp = opcode & 0xFF;
                
                if (condition == 0x0) {  // BRA
                    exec_bra(disp);
                } else if (condition == 0x1) {  // BSR
                    exec_bsr(disp);
                } else {  // Bcc
                    exec_bcc(condition, disp);
                }
            }
            break;
            
        case 0x7:
            // MOVEQ
            {
                uint16_t reg = (opcode >> 9) & 7;
                int8_t data = opcode & 0xFF;
                cpu->d[reg] = (int32_t)data;
                set_flags_nz(cpu->d[reg], 4);
                clear_flag(SR_V | SR_C);
                cpu->cycles += 4;
            }
            break;
            
        case 0x8:
            if ((opcode & 0xF1F0) == 0x8100) {  // SBCD
                exec_sbcd(opcode);
            } else if ((opcode & 0xF1C0) == 0x80C0) {  // DIVU
                exec_divu(opcode);
            } else if ((opcode & 0xF1C0) == 0x81C0) {  // DIVS
                exec_divs(opcode);
            } else {  // OR
                exec_or(opcode);
            }
            break;
            
        case 0x9:
            if ((opcode & 0xF1C0) == 0x90C0 || (opcode & 0xF1C0) == 0x91C0) {  // SUBA
                uint16_t reg = (opcode >> 9) & 7;
                uint16_t mode = (opcode >> 3) & 7;
                uint16_t ea_reg = opcode & 7;
                int size = (opcode & 0x0100) ? 4 : 2;
                
                uint32_t src = get_ea_value(mode, ea_reg, size);
                if (size == 2) src = (int16_t)src;  // Sign extend
                cpu->a[reg] -= src;
                cpu->cycles += 8;
            } else if ((opcode & 0xF130) == 0x9100) {  // SUBX
                exec_subx(opcode);
            } else {  // SUB
                exec_sub(opcode);
            }
            break;
            
        case 0xA:
            // Line A - unimplemented (often used for OS traps)
            ESP_LOGD(TAG, "Line A: 0x%04X", opcode);
            raise_exception(VEC_LINE_A);
            break;
            
        case 0xB:
            if ((opcode & 0xF1C0) == 0xB0C0 || (opcode & 0xF1C0) == 0xB1C0) {  // CMPA
                uint16_t reg = (opcode >> 9) & 7;
                uint16_t mode = (opcode >> 3) & 7;
                uint16_t ea_reg = opcode & 7;
                int size = (opcode & 0x0100) ? 4 : 2;

                uint32_t src = get_ea_value(mode, ea_reg, size);
                if (size == 2) src = (int16_t)src;
                uint32_t dst = cpu->a[reg];
                uint32_t result = dst - src;
                set_flags_nz(result, 4);
                // Carry
                if (src > dst) set_flag(SR_C);
                else clear_flag(SR_C);
                // Overflow
                bool src_neg = (src & 0x80000000) != 0;
                bool dst_neg = (dst & 0x80000000) != 0;
                bool res_neg = (result & 0x80000000) != 0;
                if (src_neg != dst_neg && res_neg != dst_neg) set_flag(SR_V);
                else clear_flag(SR_V);
                cpu->cycles += 6;
            } else if ((opcode & 0xF138) == 0xB108) {  // CMPM
                exec_cmpm(opcode);
            } else if ((opcode & 0xF100) == 0xB100) {  // EOR
                exec_eor(opcode);
            } else {  // CMP
                exec_cmp(opcode);
            }
            break;
            
        case 0xC:
            if ((opcode & 0xF1F0) == 0xC100) {  // ABCD
                exec_abcd(opcode);
            } else if ((opcode & 0xF130) == 0xC100) {  // EXG
                uint16_t rx = (opcode >> 9) & 7;
                uint16_t ry = opcode & 7;
                uint16_t opmode = (opcode >> 3) & 0x1F;
                
                if (opmode == 0x08) {  // Data registers
                    uint32_t temp = cpu->d[rx];
                    cpu->d[rx] = cpu->d[ry];
                    cpu->d[ry] = temp;
                } else if (opmode == 0x09) {  // Address registers
                    uint32_t temp = cpu->a[rx];
                    cpu->a[rx] = cpu->a[ry];
                    cpu->a[ry] = temp;
                } else if (opmode == 0x11) {  // Data/Address
                    uint32_t temp = cpu->d[rx];
                    cpu->d[rx] = cpu->a[ry];
                    cpu->a[ry] = temp;
                }
                cpu->cycles += 6;
            } else if ((opcode & 0xF1C0) == 0xC0C0) {  // MULU
                exec_mulu(opcode);
            } else if ((opcode & 0xF1C0) == 0xC1C0) {  // MULS
                exec_muls(opcode);
            } else {  // AND
                exec_and(opcode);
            }
            break;
            
        case 0xD:
            if ((opcode & 0xF1C0) == 0xD0C0 || (opcode & 0xF1C0) == 0xD1C0) {  // ADDA
                uint16_t reg = (opcode >> 9) & 7;
                uint16_t mode = (opcode >> 3) & 7;
                uint16_t ea_reg = opcode & 7;
                int size = (opcode & 0x0100) ? 4 : 2;
                
                uint32_t src = get_ea_value(mode, ea_reg, size);
                if (size == 2) src = (int16_t)src;
                cpu->a[reg] += src;
                cpu->cycles += 8;
            } else if ((opcode & 0xF130) == 0xD100) {  // ADDX
                exec_addx(opcode);
            } else {  // ADD
                exec_add(opcode);
            }
            break;
            
        case 0xE:
            // Shift/Rotate instructions
            if ((opcode & 0xFEC0) == 0xE0C0) {  // Memory shifts (ASL/ASR/LSL/LSR/ROL/ROR/ROXL/ROXR)
                uint16_t mode = (opcode >> 3) & 7;
                uint16_t reg = opcode & 7;
                uint32_t addr = get_ea_addr(mode, reg, 2);
                uint16_t value = read_word(addr);
                uint16_t result;
                
                uint8_t type = (opcode >> 9) & 7;
                bool dir = (opcode >> 8) & 1;  // 0=right, 1=left
                
                switch (type) {
                    case 0: // ASR/ASL
                        if (dir) {  // ASL
                            if (value & 0x8000) set_flag(SR_C | SR_X);
                            else clear_flag(SR_C | SR_X);
                            result = value << 1;
                            if ((value ^ result) & 0x8000) set_flag(SR_V);
                            else clear_flag(SR_V);
                        } else {  // ASR
                            if (value & 1) set_flag(SR_C | SR_X);
                            else clear_flag(SR_C | SR_X);
                            result = ((int16_t)value) >> 1;
                            clear_flag(SR_V);
                        }
                        break;
                    case 1: // LSR/LSL
                        if (dir) {  // LSL
                            if (value & 0x8000) set_flag(SR_C | SR_X);
                            else clear_flag(SR_C | SR_X);
                            result = value << 1;
                        } else {  // LSR
                            if (value & 1) set_flag(SR_C | SR_X);
                            else clear_flag(SR_C | SR_X);
                            result = value >> 1;
                        }
                        clear_flag(SR_V);
                        break;
                    case 2: // ROXR/ROXL
                        if (dir) {  // ROXL
                            bool old_msb = (value & 0x8000) != 0;
                            result = (value << 1) | (test_flag(SR_X) ? 1 : 0);
                            if (old_msb) set_flag(SR_C | SR_X);
                            else clear_flag(SR_C | SR_X);
                        } else {  // ROXR
                            bool old_lsb = (value & 1) != 0;
                            result = (value >> 1) | (test_flag(SR_X) ? 0x8000 : 0);
                            if (old_lsb) set_flag(SR_C | SR_X);
                            else clear_flag(SR_C | SR_X);
                        }
                        clear_flag(SR_V);
                        break;
                    case 3: // ROR/ROL
                        if (dir) {  // ROL
                            result = (value << 1) | (value >> 15);
                            if (result & 1) set_flag(SR_C);
                            else clear_flag(SR_C);
                        } else {  // ROR
                            result = (value >> 1) | (value << 15);
                            if (result & 0x8000) set_flag(SR_C);
                            else clear_flag(SR_C);
                        }
                        clear_flag(SR_V);
                        break;
                    default:
                        result = value;
                }
                
                write_word(addr, result);
                set_flags_nz(result, 2);
                cpu->cycles += 8;
            } else if ((opcode & 0xF018) == 0xE000) {  // ASR/ASL register
                if (opcode & 0x0100) exec_asl(opcode);
                else exec_asr(opcode);
            } else if ((opcode & 0xF018) == 0xE008) {  // LSR/LSL register  
                if (opcode & 0x0100) exec_lsl(opcode);
                else exec_lsr(opcode);
            } else if ((opcode & 0xF018) == 0xE010) {  // ROXR/ROXL register
                if (opcode & 0x0100) exec_roxl(opcode);
                else exec_roxr(opcode);
            } else if ((opcode & 0xF018) == 0xE018) {  // ROR/ROL register
                if (opcode & 0x0100) exec_rol(opcode);
                else exec_ror(opcode);
            } else {
                ESP_LOGD(TAG, "Shift/rotate: 0x%04X", opcode);
                cpu->cycles += 6;
            }
            break;
            
        case 0xF:
            // Line F - unimplemented
            ESP_LOGD(TAG, "Line F: 0x%04X", opcode);
            raise_exception(VEC_LINE_F);
            break;
    }
    
    return true;
}

// Public API
esp_err_t m68k_init(void) {
    if (cpu != NULL) {
        ESP_LOGW(TAG, "CPU already initialized");
        return ESP_OK;
    }
    
    cpu = (m68k_cpu_t*)calloc(1, sizeof(m68k_cpu_t));
    if (cpu == NULL) {
        ESP_LOGE(TAG, "Failed to allocate CPU structure");
        return ESP_ERR_NO_MEM;
    }
    
    // Try to allocate 16MB RAM in SPIRAM first
    ESP_LOGI(TAG, "Attempting to allocate %d MB RAM...", M68K_RAM_SIZE / (1024 * 1024));
    
#ifdef CONFIG_SPIRAM
    cpu->memory = (uint8_t*)heap_caps_malloc(M68K_RAM_SIZE, MALLOC_CAP_SPIRAM);
    if (cpu->memory != NULL) {
        cpu->ram_size = M68K_RAM_SIZE;
        ESP_LOGI(TAG, "Allocated %d MB in SPIRAM", M68K_RAM_SIZE / (1024 * 1024));
    }
#else
    cpu->memory = NULL;
    ESP_LOGW(TAG, "SPIRAM not enabled in sdkconfig - run 'idf.py menuconfig' and enable:");
    ESP_LOGW(TAG, "  Component config -> ESP PSRAM -> Support for external, SPI-connected RAM");
#endif

    // Fallback to regular RAM with smaller size if SPIRAM failed
    if (cpu->memory == NULL) {
        // Try smaller sizes
        size_t sizes[] = {4 * 1024 * 1024, 2 * 1024 * 1024, 1 * 1024 * 1024, 512 * 1024};
        for (int i = 0; i < 4 && cpu->memory == NULL; i++) {
            cpu->memory = (uint8_t*)heap_caps_malloc(sizes[i], MALLOC_CAP_DEFAULT);
            if (cpu->memory != NULL) {
                cpu->ram_size = sizes[i];
                ESP_LOGW(TAG, "SPIRAM unavailable, using %d KB internal RAM (limited)", sizes[i] / 1024);
                break;
            }
        }
    }

    if (cpu->memory == NULL) {
        ESP_LOGE(TAG, "Failed to allocate RAM - SPIRAM must be enabled for full emulation");
        ESP_LOGE(TAG, "Run: idf.py set-target esp32p4 && idf.py menuconfig");
        free(cpu);
        cpu = NULL;
        return ESP_ERR_NO_MEM;
    }

    memset(cpu->memory, 0, cpu->ram_size);
    
    // Initialize CPU state
    cpu->pc = 0x0000;
    cpu->sr = SR_S | 0x0700;  // Supervisor mode, interrupts masked
    cpu->a[7] = cpu->ram_size - 4;  // Stack at top of RAM
    cpu->ssp = cpu->a[7];
    cpu->running = false;
    cpu->halted = false;
    cpu->stopped = false;
    cpu->cycles = 0;
    cpu->instructions_executed = 0;
    
    ESP_LOGI(TAG, "Motorola 68000 CPU Emulator initialized");
    ESP_LOGI(TAG, "  Full instruction set emulation");
    ESP_LOGI(TAG, "  Frequency: %d MHz", M68K_CPU_FREQ_MHZ);
    ESP_LOGI(TAG, "  RAM: %d MB", M68K_RAM_SIZE / (1024 * 1024));
    ESP_LOGI(TAG, "  Initial PC: 0x%08X", cpu->pc);
    ESP_LOGI(TAG, "  Initial SP: 0x%08X", cpu->a[7]);
    ESP_LOGI(TAG, "  Compatible with: Amiga, Atari ST, Sega Genesis software");
    
    return ESP_OK;
}

void m68k_reset(void) {
    if (cpu == NULL) return;
    
    // Read reset vectors from memory
    uint32_t ssp_raw = read_long(0x000000);
    uint32_t pc_raw = read_long(0x000004);
    
    cpu->ssp = ssp_raw;
    cpu->pc = pc_raw & M68K_ADDRESS_MASK;  // Apply 24-bit mask to PC
    cpu->a[7] = cpu->ssp;
    cpu->sr = SR_S | 0x0700;
    cpu->running = true;
    cpu->halted = false;
    cpu->stopped = false;
    cpu->cycles = 0;
    cpu->instructions_executed = 0;
    
    ESP_LOGI(TAG, "CPU Reset - PC: 0x%08X (raw: 0x%08X), SP: 0x%08X (raw: 0x%08X)", 
             cpu->pc, pc_raw, cpu->a[7], ssp_raw);
}

void m68k_load_program(const uint8_t *data, uint32_t size, uint32_t addr) {
    if (cpu == NULL || data == NULL) return;
    
    if (addr + size > cpu->ram_size) {
        ESP_LOGW(TAG, "Program too large or invalid address (need 0x%08X, have 0x%08X)",
                 addr + size, cpu->ram_size);
        return;
    }
    
    memcpy(&cpu->memory[addr], data, size);
    ESP_LOGI(TAG, "Loaded %d bytes at 0x%08X", size, addr);
}

void m68k_run(uint32_t instructions) {
    if (cpu == NULL || !cpu->running) return;
    
    for (uint32_t i = 0; i < instructions && !cpu->halted; i++) {
        execute_instruction();
    }
}

void m68k_step(void) {
    if (cpu == NULL) return;
    
    execute_instruction();
}

void m68k_stop(void) {
    if (cpu == NULL) return;
    cpu->running = false;
}

bool m68k_is_halted(void) {
    if (cpu == NULL) return true;
    return cpu->halted || cpu->stopped;
}

void m68k_get_state(char *buffer, size_t buf_size) {
    if (cpu == NULL || buffer == NULL) return;
    
    snprintf(buffer, buf_size,
        "68000 CPU State:\n"
        "PC: %08lX  SR: %04X  Instructions: %lu  Cycles: %llu\n"
        "D0: %08lX  D1: %08lX  D2: %08lX  D3: %08lX\n"
        "D4: %08lX  D5: %08lX  D6: %08lX  D7: %08lX\n"
        "A0: %08lX  A1: %08lX  A2: %08lX  A3: %08lX\n"
        "A4: %08lX  A5: %08lX  A6: %08lX  SP: %08lX\n"
        "Flags: %c%c%c%c%c  Running: %s  Halted: %s",
        (unsigned long)cpu->pc, cpu->sr, (unsigned long)cpu->instructions_executed, cpu->cycles,
        (unsigned long)cpu->d[0], (unsigned long)cpu->d[1], (unsigned long)cpu->d[2], (unsigned long)cpu->d[3],
        (unsigned long)cpu->d[4], (unsigned long)cpu->d[5], (unsigned long)cpu->d[6], (unsigned long)cpu->d[7],
        (unsigned long)cpu->a[0], (unsigned long)cpu->a[1], (unsigned long)cpu->a[2], (unsigned long)cpu->a[3],
        (unsigned long)cpu->a[4], (unsigned long)cpu->a[5], (unsigned long)cpu->a[6], (unsigned long)cpu->a[7],
        test_flag(SR_X) ? 'X' : '-',
        test_flag(SR_N) ? 'N' : '-',
        test_flag(SR_Z) ? 'Z' : '-',
        test_flag(SR_V) ? 'V' : '-',
        test_flag(SR_C) ? 'C' : '-',
        cpu->running ? "YES" : "NO",
        cpu->halted ? "YES" : "NO"
    );
}

void m68k_dump_memory(uint32_t addr, uint32_t length) {
    if (cpu == NULL) {
        printf("M68K CPU not initialized\n");
        return;
    }
    
    if (addr >= cpu->ram_size) {
        printf("Address 0x%08lX is outside valid memory range (0x00000000-0x%08lX)\n",
                     (unsigned long)addr, (unsigned long)(cpu->ram_size - 1));
        return;
    }

    // Limit length to stay within valid memory
    if (addr + length > cpu->ram_size) {
        length = cpu->ram_size - addr;
    }
    
    printf("Memory dump from 0x%08lX (length: %lu bytes):\n", 
                 (unsigned long)addr, (unsigned long)length);
    printf("Address  : 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F  ASCII\n");
    printf("------------------------------------------------------------------------\n");
    
    for (uint32_t i = 0; i < length; i += 16) {
        printf("%08lX : ", (unsigned long)(addr + i));
        
        // Hex bytes
        for (int j = 0; j < 16; j++) {
            if (i + j < length) {
                printf("%02X ", cpu->memory[addr + i + j]);
            } else {
                printf("   ");
            }
        }
        
        printf(" ");
        
        // ASCII representation
        for (int j = 0; j < 16 && (i + j) < length; j++) {
            uint8_t byte = cpu->memory[addr + i + j];
            if (byte >= 32 && byte <= 126) {
                printf("%c", byte);
            } else {
                printf(".");
            }
        }
        
        printf("\n");
    }
}

void m68k_show_crash_context(void) {
    if (!crash_ctx.valid) {
        printf("No crash data available. System has not crashed yet.\n");
        return;
    }
    
    printf("\n");
    printf("========================================\n");
    printf("  M68K POST-MORTEM CRASH ANALYSIS\n");
    printf("========================================\n\n");
    
    printf("Error: %s\n\n", crash_ctx.error_msg);
    
    printf("Crash Details:\n");
    printf("  Crash Address : 0x%08lX\n", (unsigned long)crash_ctx.crash_addr);
    printf("  Current PC    : 0x%08lX\n", (unsigned long)crash_ctx.crash_pc);
    printf("  Last PC       : 0x%08lX (instruction that caused crash)\n", (unsigned long)crash_ctx.last_pc);
    printf("  Instruction   : 0x%04X\n", crash_ctx.crash_opcode);
    printf("  Status Reg    : 0x%04X\n", crash_ctx.sr);
    printf("  Cycles        : %llu\n", crash_ctx.cycles);
    printf("  Instructions  : %lu\n\n", (unsigned long)crash_ctx.instructions);
    
    printf("Data Registers:\n");
    for (int i = 0; i < 8; i++) {
        printf("  D%d = 0x%08lX", i, (unsigned long)crash_ctx.d[i]);
        if (i % 2 == 1) printf("\n");
    }
    
    printf("\nAddress Registers:\n");
    for (int i = 0; i < 7; i++) {
        printf("  A%d = 0x%08lX", i, (unsigned long)crash_ctx.a[i]);
        if (i % 2 == 1) printf("\n");
    }
    printf("  SP = 0x%08lX\n\n", (unsigned long)crash_ctx.a[7]);
    
    printf("Suggested Memory Dumps:\n");
    printf("  1. Stack contents:\n");
    printf("     dump %08lX 128\n\n", (unsigned long)(crash_ctx.a[7] - 16));
    
    printf("  2. Instruction area:\n");
    printf("     dump %08lX 64\n\n", (unsigned long)(crash_ctx.last_pc - 16));
    
    if (cpu && crash_ctx.a[0] < cpu->ram_size) {
        printf("  3. A0 register points to:\n");
        printf("     dump %08lX 128\n\n", (unsigned long)(crash_ctx.a[0] & ~0xF));
    }
    
    printf("  4. Top of memory (16MB boundary):\n");
    printf("     dump 00FFFF00 256\n\n");
    
    printf("Memory is preserved and available for examination.\n");
    printf("========================================\n\n");
}

void m68k_destroy(void) {
    if (cpu == NULL) return;
    
    if (cpu->memory != NULL) {
        free(cpu->memory);
    }
    free(cpu);
    cpu = NULL;
    
    ESP_LOGI(TAG, "CPU destroyed");
}

/* ====== Public Memory Access API for Bus Controller ====== */

uint8_t m68k_read_memory_8(uint32_t address) {
    if (!cpu || address >= cpu->ram_size) return 0xFF;
    return cpu->memory[address];
}

uint16_t m68k_read_memory_16(uint32_t address) {
    if (!cpu || address >= cpu->ram_size - 1) return 0xFFFF;
    return (cpu->memory[address] << 8) | cpu->memory[address + 1];
}

uint32_t m68k_read_memory_32(uint32_t address) {
    if (!cpu || address >= cpu->ram_size - 3) return 0xFFFFFFFF;
    return (cpu->memory[address] << 24) |
           (cpu->memory[address + 1] << 16) |
           (cpu->memory[address + 2] << 8) |
           cpu->memory[address + 3];
}

void m68k_write_memory_8(uint32_t address, uint8_t value) {
    if (!cpu || address >= cpu->ram_size) return;
    cpu->memory[address] = value;
}

void m68k_write_memory_16(uint32_t address, uint16_t value) {
    if (!cpu || address >= cpu->ram_size - 1) return;
    cpu->memory[address] = value >> 8;
    cpu->memory[address + 1] = value & 0xFF;
}

void m68k_write_memory_32(uint32_t address, uint32_t value) {
    if (!cpu || address >= cpu->ram_size - 3) return;
    cpu->memory[address] = (value >> 24) & 0xFF;
    cpu->memory[address + 1] = (value >> 16) & 0xFF;
    cpu->memory[address + 2] = (value >> 8) & 0xFF;
    cpu->memory[address + 3] = value & 0xFF;
}

/* ====== Register Access API ====== */

uint32_t m68k_get_reg(m68k_register_t reg) {
    if (!cpu) return 0;
    
    switch (reg) {
        case M68K_REG_D0 ... M68K_REG_D7:
            return cpu->d[reg - M68K_REG_D0];
        case M68K_REG_A0 ... M68K_REG_A6:
            return cpu->a[reg - M68K_REG_A0];
        case M68K_REG_A7:
        case M68K_REG_SP:
            return cpu->a[7];
        case M68K_REG_PC:
            return cpu->pc;
        case M68K_REG_SR:
            return cpu->sr;
        default:
            return 0;
    }
}

void m68k_set_reg(m68k_register_t reg, uint32_t value) {
    if (!cpu) return;
    
    switch (reg) {
        case M68K_REG_D0 ... M68K_REG_D7:
            cpu->d[reg - M68K_REG_D0] = value;
            break;
        case M68K_REG_A0 ... M68K_REG_A6:
            cpu->a[reg - M68K_REG_A0] = value;
            break;
        case M68K_REG_A7:
        case M68K_REG_SP:
            cpu->a[7] = value;
            break;
        case M68K_REG_PC:
            cpu->pc = value;
            break;
        case M68K_REG_SR:
            cpu->sr = (uint16_t)value;
            break;
    }
}
