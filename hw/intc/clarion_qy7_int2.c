/*
 * Clarion QY7 (QY7221NL) interrupt controller at 0xFF804000.
 *
 * Every property below is taken from accesses the QY7221NL kernel and its
 * bootloaders make to the block; anything the guest code does not show is
 * left UNKNOWN (reads return zero, writes are stored and logged).  See the
 * register table in the project report (docs/60-qy7-int2-controller.md).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/bitops.h"
#include "hw/intc/clarion_qy7_int2.h"
#include "hw/sh4/sh_intc.h"
#include "exec/cpu-interrupt.h"
#include "system/reset.h"

#define INT2_SIZE 0x100

/* Offsets proven by guest accesses (NK1 and the IP12/EH12 bootloaders). */
#define INT2_PRIO0      0x00 /* group priorities, one byte per group */
#define INT2_PRIO1      0x04
#define INT2_MASK_SET   0x40 /* write 1: mask the group, read: mask */
#define INT2_MASK_CLR   0x44 /* write 1: unmask the group */
#define INT2_TMU_SRC    0x58 /* bit n: TMU channel n request line */

/*
 * INFERENCE: the timer group is mask bit 8 and the top byte of PRIO0.  NK1's
 * ISR for vector 0x580 masks 0x100 on entry (0x8803F1BC), the unmask
 * switch of NK1 (0x8803E554) writes 0x100 to MASK_CLR for ids 12..14, and
 * NK1 writes PRIO0 = 0x08020202 with 8 equal to the priority it assigns to
 * vector 0x580 in the table at 0x8A64280C.
 */
#define INT2_TIMER_GROUP_BIT  8
#define INT2_TIMER_PRIO_SHIFT 24

#define INT2_TMU_CHANNELS 3
#define INT2_UNK_SLOTS    (INT2_SIZE / 4)

struct ClarionQy7Int2 {
    MemoryRegion iomem;
    CPUState *cpu;
    struct intc_desc desc;
    struct intc_source src[INT2_TMU_CHANNELS];
    qemu_irq tmu_irq[INT2_TMU_CHANNELS];
    uint32_t prio[2];
    uint32_t mask;
    uint32_t tmu_lines;
    uint32_t unk[INT2_UNK_SLOTS];
};

static void int2_update(ClarionQy7Int2 *s)
{
    uint32_t prio = (s->prio[0] >> INT2_TIMER_PRIO_SHIFT) & 0xff;
    bool open = !(s->mask & BIT(INT2_TIMER_GROUP_BIT)) && prio > 0;
    int pending = 0;

    for (int i = 0; i < INT2_TMU_CHANNELS; i++) {
        s->src[i].priority = prio;
        s->src[i].pending = open && (s->tmu_lines & BIT(i));
        pending += s->src[i].pending;
    }
    s->desc.pending = pending;
    if (pending) {
        cpu_interrupt(s->cpu, CPU_INTERRUPT_HARD);
    } else {
        cpu_reset_interrupt(s->cpu, CPU_INTERRUPT_HARD);
    }
}

static void int2_tmu_set(void *opaque, int n, int level)
{
    ClarionQy7Int2 *s = opaque;

    s->tmu_lines = deposit32(s->tmu_lines, n, 1, !!level);
    int2_update(s);
}

static uint64_t int2_read(void *opaque, hwaddr addr, unsigned size)
{
    ClarionQy7Int2 *s = opaque;

    switch (addr) {
    case INT2_PRIO0:
        return s->prio[0];
    case INT2_PRIO1:
        return s->prio[1];
    case INT2_MASK_SET:
        return s->mask;
    case INT2_TMU_SRC:
        return s->tmu_lines;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "clarion-qy7-int2: unimp read +0x%02" HWADDR_PRIx
                      " (returns 0)\n", addr);
        return 0;
    }
}

static void int2_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    ClarionQy7Int2 *s = opaque;

    switch (addr) {
    case INT2_PRIO0:
        s->prio[0] = val;
        break;
    case INT2_PRIO1:
        s->prio[1] = val;
        break;
    case INT2_MASK_SET:
        s->mask |= val;
        break;
    case INT2_MASK_CLR:
        s->mask &= ~val;
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "clarion-qy7-int2: unimp write +0x%02" HWADDR_PRIx
                      " = 0x%08" PRIx64 " (stored)\n", addr, val);
        s->unk[addr / 4] = val;
        return;
    }
    int2_update(s);
}

static const MemoryRegionOps int2_ops = {
    .read = int2_read,
    .write = int2_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void int2_reset(void *opaque)
{
    ClarionQy7Int2 *s = opaque;

    /*
     * INFERENCE: every QY7221NL image writes 0xFFFFFFFF to MASK_SET before
     * anything else, so all groups start masked.  Priorities start at 0.
     */
    s->mask = 0xffffffff;
    s->prio[0] = s->prio[1] = 0;
    memset(s->unk, 0, sizeof(s->unk));
    int2_update(s);
}

ClarionQy7Int2 *clarion_qy7_int2_init(MemoryRegion *sysmem, hwaddr base,
                                      CPUState *cpu, void **intc_handle)
{
    ClarionQy7Int2 *s = g_new0(ClarionQy7Int2, 1);
    /* INTEVT of TUNI0..2, same as in the SH7785 manual and the NK1 ISR */
    static const unsigned short vect[INT2_TMU_CHANNELS] = {
        0x580, 0x5a0, 0x5c0,
    };

    s->cpu = cpu;
    s->desc.nr_sources = INT2_TMU_CHANNELS;
    s->desc.sources = s->src;
    for (int i = 0; i < INT2_TMU_CHANNELS; i++) {
        s->src[i].parent = &s->desc;
        s->src[i].vect = vect[i];
    }
    *intc_handle = &s->desc;

    for (int i = 0; i < INT2_TMU_CHANNELS; i++) {
        s->tmu_irq[i] = qemu_allocate_irq(int2_tmu_set, s, i);
    }
    memory_region_init_io(&s->iomem, NULL, &int2_ops, s, "qy7.int2",
                          INT2_SIZE);
    memory_region_add_subregion(sysmem, base, &s->iomem);
    qemu_register_reset(int2_reset, s);
    return s;
}

qemu_irq clarion_qy7_int2_timer_irq(ClarionQy7Int2 *s, int channel)
{
    g_assert(channel >= 0 && channel < INT2_TMU_CHANNELS);
    return s->tmu_irq[channel];
}
