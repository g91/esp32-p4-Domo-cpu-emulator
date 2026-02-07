// Minimal i386 CPU emulator stub for ESP32-P4
// This is a placeholder until we port the full tiny386 CPU core
// For now, implements basic structure and test functionality

#include "i386_core.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_attr.h"

static const char *TAG = "i386";

// CPU structure (simplified from tiny386)
struct CPUI386 {
    // General purpose registers
    union {
        u32 r32;
        u16 r16;
        u8 r8[2];
    } gprx[8];
    
    uword ip, next_ip;
    uword flags;
    uword flags_mask;
    int cpl;
    bool code16;
    uword sp_mask;
    bool halt;
    
    FPU *fpu;
    
    // Segment registers
    struct {
        uword sel;
        uword base;
        uword limit;
        uword flags;
    } seg[8];
    
    // Descriptor tables
    struct {
        uword base;
        uword limit;
    } idt, gdt;
    
    // Control registers
    uword cr0, cr2, cr3;
    uword dr[8];
    
    // TLB cache
    struct {
        int size;
        struct tlb_entry {
            uword lpgno;
            uword xaddr;
            void *pte_lookup;
            u8 *ppte;
        } *tab;
    } tlb;
    
    // Physical memory
    u8 *phys_mem;
    long phys_mem_size;
    
    long cycle;
    int excno;
    uword excerr;
    bool intr;
    
    CPU_CB cb;
    int gen;
};

// FLAGS bits
#define CF 0x1
#define PF 0x4
#define AF 0x10
#define ZF 0x40
#define SF 0x80
#define TF 0x100
#define IF 0x200
#define DF 0x400
#define OF 0x800
#define IOPL_MASK 0x3000
#define NT 0x4000
#define RF 0x10000
#define VM 0x20000
#define AC 0x40000
#define VIF 0x80000
#define VIP 0x100000
#define ID 0x200000

// Control register bits
#define CR0_PE 0x1      // Protected mode
#define CR0_MP 0x2      // Monitor coprocessor
#define CR0_EM 0x4      // Emulation
#define CR0_TS 0x8      // Task switched
#define CR0_ET 0x10     // Extension type
#define CR0_NE 0x20     // Numeric error
#define CR0_WP 0x10000  // Write protect
#define CR0_AM 0x40000  // Alignment mask
#define CR0_NW 0x20000000  // Not write-through
#define CR0_CD 0x40000000  // Cache disable
#define CR0_PG 0x80000000  // Paging

CPUI386 *cpui386_new(int gen, char *phys_mem, long phys_mem_size, CPU_CB **cb)
{
    ESP_LOGI(TAG, "Creating i386 CPU (gen=%d, RAM=%ld MB)", gen, phys_mem_size / (1024*1024));
    
    CPUI386 *cpu = heap_caps_calloc(1, sizeof(CPUI386), MALLOC_CAP_INTERNAL);
    if (!cpu) {
        ESP_LOGE(TAG, "Failed to allocate CPU structure");
        return NULL;
    }
    
    cpu->gen = gen;
    cpu->phys_mem = (u8 *)phys_mem;
    cpu->phys_mem_size = phys_mem_size;
    
    // Allocate TLB in internal RAM for speed
    cpu->tlb.size = 256;
    cpu->tlb.tab = heap_caps_calloc(cpu->tlb.size, sizeof(struct tlb_entry), MALLOC_CAP_INTERNAL);
    if (!cpu->tlb.tab) {
        ESP_LOGE(TAG, "Failed to allocate TLB");
        free(cpu);
        return NULL;
    }
    
    // FPU support for gen >= 4
    cpu->fpu = NULL;  // Will be enabled separately if needed
    
    // Return callback structure
    *cb = &cpu->cb;
    
    ESP_LOGI(TAG, "i386 CPU created successfully");
    return cpu;
}

void cpui386_delete(CPUI386 *cpu)
{
    if (!cpu) return;
    
    if (cpu->tlb.tab) {
        heap_caps_free(cpu->tlb.tab);
    }
    
    // Note: phys_mem is not freed here - it's managed by the caller
    free(cpu);
    ESP_LOGI(TAG, "i386 CPU deleted");
}

void cpui386_enable_fpu(CPUI386 *cpu)
{
    if (!cpu) return;
    // TODO: Initialize FPU
    ESP_LOGI(TAG, "FPU enabled (stub)");
}

void cpui386_reset(CPUI386 *cpu)
{
    if (!cpu) return;
    
    ESP_LOGI(TAG, "Resetting i386 CPU to real mode");
    
    // Real mode reset state (Intel 386 manual)
    memset(cpu->gprx, 0, sizeof(cpu->gprx));
    
    // CS:IP = F000:FFF0 (BIOS entry point)
    cpu->seg[SEG_CS].sel = 0xF000;
    cpu->seg[SEG_CS].base = 0xFFFF0000;
    cpu->seg[SEG_CS].limit = 0xFFFF;
    cpu->seg[SEG_CS].flags = 0x93;  // Present, readable, accessed
    cpu->ip = 0xFFF0;
    
    // Other segments
    for (int i = 0; i < 8; i++) {
        if (i != SEG_CS) {
            cpu->seg[i].sel = 0;
            cpu->seg[i].base = 0;
            cpu->seg[i].limit = 0xFFFF;
            cpu->seg[i].flags = 0x93;
        }
    }
    
    // FLAGS = 0x0002 (bit 1 always set)
    cpu->flags = 0x0002;
    cpu->flags_mask = 0x3F7FFF;
    
    // CR0 = 0 (real mode, no paging)
    cpu->cr0 = 0;
    cpu->cr2 = 0;
    cpu->cr3 = 0;
    
    // Real mode by default
    cpu->code16 = true;
    cpu->sp_mask = 0xFFFF;
    cpu->cpl = 0;
    cpu->halt = false;
    cpu->intr = false;
    cpu->excno = 0;
    cpu->cycle = 0;
    
    // Clear TLB
    memset(cpu->tlb.tab, 0, cpu->tlb.size * sizeof(struct tlb_entry));
    
    ESP_LOGI(TAG, "CPU reset complete: CS:IP = %04X:%04X", cpu->seg[SEG_CS].sel, cpu->ip);
}

void cpui386_reset_pm(CPUI386 *cpu, uint32_t start_addr)
{
    if (!cpu) return;
    
    ESP_LOGI(TAG, "Resetting to protected mode at 0x%08X", start_addr);
    
    cpui386_reset(cpu);
    
    // Enable protected mode
    cpu->cr0 |= CR0_PE;
    cpu->code16 = false;
    cpu->sp_mask = 0xFFFFFFFF;
    
    // Set CS:IP to start address
    cpu->seg[SEG_CS].base = 0;
    cpu->ip = start_addr;
    
    ESP_LOGI(TAG, "Protected mode reset complete");
}

// Placeholder execution loop
void IRAM_ATTR cpui386_step(CPUI386 *cpu, int stepcount)
{
    if (!cpu) return;
    
    // TODO: Implement full instruction decoder and executor
    // For now, just increment cycle counter
    for (int i = 0; i < stepcount; i++) {
        if (cpu->halt) {
            break;
        }
        
        // Placeholder: just advance IP
        cpu->ip++;
        cpu->cycle++;
        
        // Check for interrupts every 100 cycles
        if (cpu->intr && (cpu->flags & IF) && (cpu->cycle % 100 == 0)) {
            if (cpu->cb.pic && cpu->cb.pic_read_irq) {
                int irq = cpu->cb.pic_read_irq(cpu->cb.pic);
                if (irq >= 0) {
                    // Handle interrupt (stub)
                    cpu->intr = false;
                }
            }
        }
    }
}

void cpui386_raise_irq(CPUI386 *cpu)
{
    if (cpu) {
        cpu->intr = true;
    }
}

void cpui386_set_gpr(CPUI386 *cpu, int i, u32 val)
{
    if (cpu && i >= 0 && i < 8) {
        cpu->gprx[i].r32 = val;
    }
}

long cpui386_get_cycle(CPUI386 *cpu)
{
    return cpu ? cpu->cycle : 0;
}

// Memory access stubs
bool cpu_load8(CPUI386 *cpu, int seg, uword addr, u8 *res)
{
    if (!cpu || !res) return false;
    
    // Simplified: no segmentation/paging yet
    uword linear = cpu->seg[seg].base + addr;
    if (linear >= cpu->phys_mem_size) {
        cpu->excno = 13; // GP fault
        return false;
    }
    
    *res = cpu->phys_mem[linear];
    return true;
}

bool cpu_store8(CPUI386 *cpu, int seg, uword addr, u8 val)
{
    if (!cpu) return false;
    
    uword linear = cpu->seg[seg].base + addr;
    if (linear >= cpu->phys_mem_size) {
        cpu->excno = 13;
        return false;
    }
    
    cpu->phys_mem[linear] = val;
    return true;
}

bool cpu_load16(CPUI386 *cpu, int seg, uword addr, u16 *res)
{
    if (!cpu || !res) return false;
    
    uword linear = cpu->seg[seg].base + addr;
    if (linear + 1 >= cpu->phys_mem_size) {
        cpu->excno = 13;
        return false;
    }
    
    *res = *(u16 *)(cpu->phys_mem + linear);
    return true;
}

bool cpu_store16(CPUI386 *cpu, int seg, uword addr, u16 val)
{
    if (!cpu) return false;
    
    uword linear = cpu->seg[seg].base + addr;
    if (linear + 1 >= cpu->phys_mem_size) {
        cpu->excno = 13;
        return false;
    }
    
    *(u16 *)(cpu->phys_mem + linear) = val;
    return true;
}

bool cpu_load32(CPUI386 *cpu, int seg, uword addr, u32 *res)
{
    if (!cpu || !res) return false;
    
    uword linear = cpu->seg[seg].base + addr;
    if (linear + 3 >= cpu->phys_mem_size) {
        cpu->excno = 13;
        return false;
    }
    
    *res = *(u32 *)(cpu->phys_mem + linear);
    return true;
}

bool cpu_store32(CPUI386 *cpu, int seg, uword addr, u32 val)
{
    if (!cpu) return false;
    
    uword linear = cpu->seg[seg].base + addr;
    if (linear + 3 >= cpu->phys_mem_size) {
        cpu->excno = 13;
        return false;
    }
    
    *(u32 *)(cpu->phys_mem + linear) = val;
    return true;
}

void cpu_setax(CPUI386 *cpu, u16 ax)
{
    if (cpu) {
        cpu->gprx[REG_EAX].r16 = ax;
    }
}

u16 cpu_getax(CPUI386 *cpu)
{
    return cpu ? cpu->gprx[REG_EAX].r16 : 0;
}

void cpu_setexc(CPUI386 *cpu, int excno, uword excerr)
{
    if (cpu) {
        cpu->excno = excno;
        cpu->excerr = excerr;
    }
}

void cpu_setflags(CPUI386 *cpu, uword set_mask, uword clear_mask)
{
    if (cpu) {
        cpu->flags = (cpu->flags & ~clear_mask) | set_mask;
    }
}

uword cpu_getflags(CPUI386 *cpu)
{
    return cpu ? cpu->flags : 0;
}

void cpu_abort(CPUI386 *cpu, int code)
{
    if (cpu) {
        ESP_LOGE(TAG, "CPU abort: code=%d", code);
        cpu->halt = true;
    }
}

void cpui386_dump_regs(CPUI386 *cpu)
{
    if (!cpu) return;
    
    printf("=== i386 CPU State ===\n");
    printf("EAX=%08lX  EBX=%08lX  ECX=%08lX  EDX=%08lX\n",
           (unsigned long)cpu->gprx[0].r32, (unsigned long)cpu->gprx[3].r32, (unsigned long)cpu->gprx[1].r32, (unsigned long)cpu->gprx[2].r32);
    printf("ESI=%08lX  EDI=%08lX  EBP=%08lX  ESP=%08lX\n",
           (unsigned long)cpu->gprx[6].r32, (unsigned long)cpu->gprx[7].r32, (unsigned long)cpu->gprx[5].r32, (unsigned long)cpu->gprx[4].r32);
    printf("CS:IP=%04lX:%08lX  FLAGS=%08lX\n",
           (unsigned long)cpu->seg[SEG_CS].sel, (unsigned long)cpu->ip, (unsigned long)cpu->flags);
    printf("DS=%04lX  ES=%04lX  SS=%04lX  FS=%04lX  GS=%04lX\n",
           (unsigned long)cpu->seg[SEG_DS].sel, (unsigned long)cpu->seg[SEG_ES].sel, (unsigned long)cpu->seg[SEG_SS].sel,
           (unsigned long)cpu->seg[SEG_FS].sel, (unsigned long)cpu->seg[SEG_GS].sel);
    printf("CR0=%08lX  CR2=%08lX  CR3=%08lX  CPL=%d\n",
           (unsigned long)cpu->cr0, (unsigned long)cpu->cr2, (unsigned long)cpu->cr3, cpu->cpl);
    printf("Cycles=%ld  Halt=%d  Intr=%d  Exc=%d\n",
           cpu->cycle, cpu->halt, cpu->intr, cpu->excno);
    printf("Mode: %s\n", (cpu->cr0 & CR0_PE) ? "Protected" : "Real");
    if (cpu->cr0 & CR0_PG) printf("  Paging enabled\n");
    printf("=====================\n");
}

const char *cpui386_get_status(CPUI386 *cpu)
{
    static char status[256];
    if (!cpu) return "CPU not initialized";
    
    snprintf(status, sizeof(status),
             "CS:IP=%04lX:%08lX EAX=%08lX FLAGS=%08lX Cycles=%ld %s",
             (unsigned long)cpu->seg[SEG_CS].sel, (unsigned long)cpu->ip, (unsigned long)cpu->gprx[0].r32, (unsigned long)cpu->flags,
             cpu->cycle, cpu->halt ? "HALT" : "RUN");
    return status;
}
