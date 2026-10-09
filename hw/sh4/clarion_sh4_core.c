/* Minimal SH-4A core register windows used by Clarion machines. */
#include "qemu/osdep.h"
#include "hw/sh4/clarion_sh4_core.h"
#include "target/sh4/cpu.h"
#include "hw/core/cpu.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "exec/cputlb.h"

typedef struct ClarionSH4Core {
    SuperHCPU *cpu;
    MemoryRegion regs;
    MemoryRegion tlb;
    uint32_t ccr;
} ClarionSH4Core;

static uint64_t core_read(void *opaque, hwaddr addr, unsigned size)
{
    ClarionSH4Core *s = opaque;
    CPUSH4State *e = &s->cpu->env;

    if (size != 4) {
        return 0;
    }
    switch (addr) {
    case 0x00:
        return e->pteh;
    case 0x04:
        return e->ptel;
    case 0x08:
        return e->ttb;
    case 0x0c:
        return e->tea;
    case 0x10:
        return e->mmucr;
    case 0x1c:
        return s->ccr;
    case 0x20:
        return e->tra;
    case 0x24:
        return e->expevt;
    case 0x28:
        return e->intevt;
    case 0x30:
        return SUPERH_CPU_GET_CLASS(s->cpu)->pvr;
    case 0x34:
        return e->ptea;
    case 0x40:
        return SUPERH_CPU_GET_CLASS(s->cpu)->cvr;
    case 0x44:
        return SUPERH_CPU_GET_CLASS(s->cpu)->prr;
    default:
        return 0;
    }
}

static void core_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    ClarionSH4Core *s = opaque;
    CPUSH4State *e = &s->cpu->env;
    uint32_t v = value;

    if (size != 4) {
        return;
    }
    switch (addr) {
    case 0x00:
        if ((e->pteh ^ v) & 0xff) {
            tlb_flush(CPU(s->cpu));
        }
        e->pteh = v;
        break;
    case 0x04:
        e->ptel = v;
        break;
    case 0x08:
        e->ttb = v;
        break;
    case 0x0c:
        e->tea = v;
        break;
    case 0x10:
        if (v & MMUCR_TI) {
            cpu_sh4_invalidate_tlb(e);
        }
        e->mmucr = v & ~MMUCR_TI;
        break;
    case 0x1c:
        s->ccr = v;
        break;
    case 0x20:
        e->tra = v & 0x7ff;
        break;
    case 0x24:
        e->expevt = v & 0x7ff;
        break;
    case 0x28:
        e->intevt = v & 0x7ff;
        break;
    case 0x34:
        e->ptea = v & 0xf;
        break;
    }
}

static const MemoryRegionOps core_ops = {
    .read = core_read,
    .write = core_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static uint64_t tlb_read(void *opaque, hwaddr addr, unsigned size)
{
    ClarionSH4Core *s = opaque;
    unsigned type = (addr & 0x07000000) >> 24;

    if (size != 4) {
        return 0;
    }
    switch (type) {
    case 2:
        return cpu_sh4_read_mmaped_itlb_addr(&s->cpu->env, addr);
    case 3:
        return cpu_sh4_read_mmaped_itlb_data(&s->cpu->env, addr);
    case 6:
        return cpu_sh4_read_mmaped_utlb_addr(&s->cpu->env, addr);
    case 7:
        return cpu_sh4_read_mmaped_utlb_data(&s->cpu->env, addr);
    default:
        return 0;
    }
}

static void tlb_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    ClarionSH4Core *s = opaque;
    unsigned type = (addr & 0x07000000) >> 24;

    if (size != 4) {
        return;
    }
    switch (type) {
    case 2:
        cpu_sh4_write_mmaped_itlb_addr(&s->cpu->env, addr, value);
        break;
    case 3:
        cpu_sh4_write_mmaped_itlb_data(&s->cpu->env, addr, value);
        break;
    case 6:
        cpu_sh4_write_mmaped_utlb_addr(&s->cpu->env, addr, value);
        break;
    case 7:
        cpu_sh4_write_mmaped_utlb_data(&s->cpu->env, addr, value);
        break;
    }
}

static const MemoryRegionOps tlb_ops = {
    .read = tlb_read,
    .write = tlb_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

void clarion_sh4_core_init(SuperHCPU *cpu, MemoryRegion *sysmem)
{
    ClarionSH4Core *s = g_new0(ClarionSH4Core, 1);
    s->cpu = cpu;
    memory_region_init_io(&s->regs, NULL, &core_ops, s,
                          "clarion-sh4-core-registers", 0x1000);
    memory_region_add_subregion(sysmem, 0xff000000, &s->regs);
    memory_region_init_io(&s->tlb, NULL, &tlb_ops, s, "clarion-sh4-core-tlb",
                          0x08000000);
    memory_region_add_subregion(sysmem, 0xf0000000, &s->tlb);
}
