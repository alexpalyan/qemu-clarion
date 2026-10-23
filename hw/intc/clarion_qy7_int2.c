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
#include "target/sh4/cpu.h"
#include "qemu/timer.h"
#include "trace.h"

#define INT2_SIZE 0x100

/* Offsets proven by guest accesses (NK1 and the IP12/EH12 bootloaders). */
#define INT2_PRIO0      0x00 /* group priorities, stored, no effect yet */
#define INT2_PRIO1      0x04
#define INT2_MASK_SET   0x40 /* write 1: mask the group, read: mask */
#define INT2_MASK_CLR   0x44 /* write 1: unmask the group */

/*
 * INFERENCE: the TMU group is mask bit 1.  The kernel writes 2 to MASK_CLR
 * right after it installs the tick ISR (0x8803D698), and IP12/EH12, which
 * only need the timer, unmask exactly 2.
 */
#define INT2_TMU_GROUP_BIT  1

/* QY7's per-vector software priority tables do not configure this IRQ level. */
#define INT2_QY7_PRIORITY   1

#define INT2_HPB_GROUP_BIT  19
#define INT2_HPB_CHANNELS   32
#define INT2_HPB_INDEX_BASE 0x4B
/*
 * HPB-DMAC channels 0..11 use ISR table slots 75..86; channels 12..20 use
 * slots 93..101, i.e. 6 slots further (slots 87..92 hold no channel ISR).
 * The ISR of slot 93 + k clears and tests bit 12 + k of the DMAC status and
 * enable registers (0xFFC0880C, 0xFFC08814), so the slot is the one of the
 * channel whose status bit is raised.  Channels above 20 have no such ISR
 * in the live table and keep the old (unverified) slot.
 */
#define INT2_HPB_SPLIT_CHANNEL 12
#define INT2_HPB_LAST_CHANNEL  20
#define INT2_HPB_SPLIT_GAP     6

#define INT2_TMU_CHANNELS 3
#define INT2_DISPLAY_SOURCES 2
#define INT2_SDHI_SOURCES 2
#define INT2_DISPLAY_INDEX (INT2_TMU_CHANNELS + INT2_HPB_CHANNELS)
#define INT2_SDHI_INDEX (INT2_DISPLAY_INDEX + INT2_DISPLAY_SOURCES)
#define INT2_UART_INDEX (INT2_SDHI_INDEX + INT2_SDHI_SOURCES)
#define INT2_RCAN_INDEX (INT2_UART_INDEX + 1)
#define INT2_SCIF_CHANNELS 6
#define INT2_SCIF_INDEX (INT2_RCAN_INDEX + 1)
#define INT2_SOURCE_COUNT (INT2_TMU_CHANNELS + INT2_HPB_CHANNELS + \
                           INT2_DISPLAY_SOURCES + INT2_SDHI_SOURCES + \
                           2 + INT2_SCIF_CHANNELS + 1)
#define INT2_EXTIRQ_INDEX (INT2_SOURCE_COUNT - 1)
#define INT2_EXTIRQ_GROUP_BIT 18
#define INT2_EXTIRQ_PRIORITY 11
/*
 * Sub-CPU link UART (CAN.dll registers SYSINTR 26 for it).  The ISRs that
 * return SYSINTR 26 sit in slots 63 (vector 0x9e0) and 65 (vector 0xa20);
 * both mask INT2 group bit 18.  INFERENCE: the UART uses slot 63.
 */
#define INT2_UART_GROUP_BIT 18
#define INT2_UART_SLOT      63
#define INT2_RCAN_GROUP_BIT 17
#define INT2_RCAN_SLOT      52
#define INT2_SCIF_GROUP_BIT 14
#define INT2_SCIF_CHANNEL_COUNT 8
#define INT2_UNK_SLOTS    (INT2_SIZE / 4)

static int int2_hpb_slot(int channel)
{
    return INT2_HPB_INDEX_BASE + channel +
           (channel >= INT2_HPB_SPLIT_CHANNEL &&
            channel <= INT2_HPB_LAST_CHANNEL ? INT2_HPB_SPLIT_GAP : 0);
}

struct ClarionQy7Int2 {
    MemoryRegion iomem;
    CPUState *cpu;
    struct intc_desc desc;
    struct intc_source src[INT2_SOURCE_COUNT];
    qemu_irq tmu_irq[INT2_TMU_CHANNELS];
    qemu_irq hpb_irq[INT2_HPB_CHANNELS];
    uint32_t prio[2];
    uint32_t mask;
    uint32_t tmu_lines;
    uint32_t hpb_pending;
    qemu_irq g2d_irq;
    bool g2d_pending;
    qemu_irq du_irq;
    bool du_pending;
    qemu_irq sdhi_irq[INT2_SDHI_SOURCES];
    uint32_t sdhi_pending;
    qemu_irq uart_irq;
    bool uart_pending;
    qemu_irq rcan_irq;
    bool rcan_pending;
    qemu_irq extirq;
    bool extirq_pending;
    qemu_irq scif_irq[INT2_SCIF_CHANNEL_COUNT];
    uint32_t scif_pending;
    uint32_t unk[INT2_UNK_SLOTS];
};

static void int2_update(ClarionQy7Int2 *s)
{
    bool open = !(s->mask & BIT(INT2_TMU_GROUP_BIT));
    int pending = 0;

    for (int i = 0; i < INT2_TMU_CHANNELS; i++) {
        s->src[i].priority = INT2_QY7_PRIORITY;
        s->src[i].pending = open && (s->tmu_lines & BIT(i));
        pending += s->src[i].pending;
    }
    for (int i = 0; i < INT2_HPB_CHANNELS; i++) {
        int index = int2_hpb_slot(i);
        struct intc_source *source = &s->src[INT2_TMU_CHANNELS + i];

        source->vect = 0x200 + index * 0x20;
        source->priority = INT2_QY7_PRIORITY;
        source->pending = !(s->mask & BIT(INT2_HPB_GROUP_BIT)) &&
                          (s->hpb_pending & BIT(i));
        pending += source->pending;
    }
    {
        struct intc_source *source = &s->src[INT2_DISPLAY_INDEX];

        source->vect = 0x780;
        source->priority = INT2_QY7_PRIORITY;
        source->pending = s->g2d_pending && !(s->mask & BIT(12));
        pending += source->pending;
    }
    {
        struct intc_source *source = &s->src[INT2_DISPLAY_INDEX + 1];

        source->vect = 0x3e0;
        source->priority = INT2_QY7_PRIORITY;
        source->pending = s->du_pending && !(s->mask & BIT(0));
        pending += source->pending;
    }
    for (int i = 0; i < INT2_SDHI_SOURCES; i++) {
        struct intc_source *source = &s->src[INT2_SDHI_INDEX + i];

        source->vect = 0xae0 + i * 0x20;
        source->priority = INT2_QY7_PRIORITY;
        source->pending = (s->sdhi_pending & BIT(i)) &&
                          !(s->mask & BIT(29));
        pending += source->pending;
    }
    {
        struct intc_source *source = &s->src[INT2_UART_INDEX];

        source->vect = 0x200 + INT2_UART_SLOT * 0x20;
        source->priority = INT2_QY7_PRIORITY;
        source->pending = s->uart_pending &&
                          !(s->mask & BIT(INT2_UART_GROUP_BIT));
        pending += source->pending;
    }
    {
        struct intc_source *source = &s->src[INT2_RCAN_INDEX];

        source->vect = 0x200 + INT2_RCAN_SLOT * 0x20;
        source->priority = INT2_QY7_PRIORITY;
        source->pending = s->rcan_pending &&
                          !(s->mask & BIT(INT2_RCAN_GROUP_BIT));
        pending += source->pending;
    }
    {
        static const uint8_t channels[INT2_SCIF_CHANNELS] = {
            1, 2, 3, 5, 6, 7,
        };
        static const uint16_t vectors[INT2_SCIF_CHANNELS] = {
            0x8e0, 0x900, 0x920, 0x960, 0x980, 0x9a0,
        };
        for (int i = 0; i < INT2_SCIF_CHANNELS; i++) {
            struct intc_source *source = &s->src[INT2_SCIF_INDEX + i];

            source->vect = vectors[i];
            source->priority = INT2_QY7_PRIORITY;
            source->pending = (s->scif_pending & BIT(channels[i])) &&
                              !(s->mask & BIT(INT2_SCIF_GROUP_BIT));
            pending += source->pending;
        }
    }
    {
        struct intc_source *source = &s->src[INT2_EXTIRQ_INDEX];

        source->vect = 0x240;
        source->priority = INT2_EXTIRQ_PRIORITY;
        source->pending = s->extirq_pending &&
                          !(s->mask & BIT(INT2_EXTIRQ_GROUP_BIT));
        pending += source->pending;
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

    if (!!(s->tmu_lines & BIT(n)) == !!level) {
        return;
    }
    s->tmu_lines = deposit32(s->tmu_lines, n, 1, !!level);
    int2_update(s);
}

static void int2_hpb_set(void *opaque, int n, int level)
{
    ClarionQy7Int2 *s = opaque;

    if (!!(s->hpb_pending & BIT(n)) == !!level) {
        return;
    }
    s->hpb_pending = deposit32(s->hpb_pending, n, 1, !!level);
    int2_update(s);
}

static void int2_g2d_set(void *opaque, int n, int level)
{
    ClarionQy7Int2 *s = opaque;

    (void)n;
    if (s->g2d_pending == !!level) {
        return;
    }
    s->g2d_pending = !!level;
    int2_update(s);
}

static void int2_du_set(void *opaque, int n, int level)
{
    ClarionQy7Int2 *s = opaque;

    (void)n;
    if (s->du_pending == !!level) {
        return;
    }
    s->du_pending = !!level;
    int2_update(s);
}

static void int2_sdhi_set(void *opaque, int n, int level)
{
    ClarionQy7Int2 *s = opaque;

    trace_clarion_qy7_int2_sdhi(
        n, level, SUPERH_CPU(s->cpu)->env.pc,
        (SUPERH_CPU(s->cpu)->env.sr >> 4) & 0xf, s->mask);
    if (!!(s->sdhi_pending & BIT(n)) == !!level) {
        if (level) {
            int2_update(s);
        }
        return;
    }
    s->sdhi_pending = deposit32(s->sdhi_pending, n, 1, !!level);
    int2_update(s);
}

static void int2_uart_set(void *opaque, int n, int level)
{
    ClarionQy7Int2 *s = opaque;

    (void)n;
    if (s->uart_pending == !!level) {
        return;
    }
    s->uart_pending = !!level;
    int2_update(s);
}

static void int2_rcan_set(void *opaque, int n, int level)
{
    ClarionQy7Int2 *s = opaque;

    (void)n;
    if (s->rcan_pending == !!level) {
        return;
    }
    s->rcan_pending = !!level;
    int2_update(s);
}

static void int2_extirq_set(void *opaque, int n, int level)
{
    ClarionQy7Int2 *s = opaque;

    (void)n;
    if (s->extirq_pending == !!level) {
        return;
    }
    s->extirq_pending = !!level;
    int2_update(s);
}

static void int2_scif_set(void *opaque, int n, int level)
{
    ClarionQy7Int2 *s = opaque;

    if (!!(s->scif_pending & BIT(n)) == !!level) {
        return;
    }
    s->scif_pending = deposit32(s->scif_pending, n, 1, !!level);
    int2_update(s);
}

static uint64_t int2_reg_read(ClarionQy7Int2 *s, hwaddr addr)
{
    switch (addr) {
    case INT2_PRIO0:
        return s->prio[0];
    case INT2_PRIO1:
        return s->prio[1];
    case 0x3c:
        return (s->scif_pending ? BIT(INT2_SCIF_GROUP_BIT) : 0) |
               (s->sdhi_pending ? BIT(29) : 0) |
               (s->uart_pending ? BIT(INT2_UART_GROUP_BIT) : 0) |
               (s->rcan_pending ? BIT(INT2_RCAN_GROUP_BIT) : 0);
    case INT2_MASK_SET:
        return s->mask;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "clarion-qy7-int2: unimp read +0x%02" HWADDR_PRIx
                      " (returns 0)\n", addr);
        return 0;
    }
}

static uint64_t int2_read(void *opaque, hwaddr addr, unsigned size)
{
    uint64_t val = int2_reg_read(opaque, addr);

    trace_clarion_qy7_int2_read(addr, val);
    return val;
}

static void int2_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    ClarionQy7Int2 *s = opaque;

    trace_clarion_qy7_int2_write(
        addr, val, SUPERH_CPU(s->cpu)->env.pc,
        qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL));
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
    s->hpb_pending = 0;
    s->g2d_pending = false;
    s->du_pending = false;
    s->sdhi_pending = 0;
    s->uart_pending = false;
    s->rcan_pending = false;
    s->extirq_pending = false;
    s->scif_pending = 0;
    memset(s->unk, 0, sizeof(s->unk));
    int2_update(s);
}

ClarionQy7Int2 *clarion_qy7_int2_init(MemoryRegion *sysmem, hwaddr base,
                                      CPUState *cpu, void **intc_handle)
{
    ClarionQy7Int2 *s = g_new0(ClarionQy7Int2, 1);
    /* Vectors the guest installs for TMU0..2 (tick ISR at vector 0x400). */
    static const unsigned short vect[INT2_TMU_CHANNELS] = {
        0x400, 0x420, 0x440,
    };

    s->cpu = cpu;
    s->desc.nr_sources = INT2_SOURCE_COUNT;
    s->desc.sources = s->src;
    for (int i = 0; i < INT2_SOURCE_COUNT; i++) {
        s->src[i].parent = &s->desc;
        if (i < INT2_TMU_CHANNELS) {
            s->src[i].vect = vect[i];
        } else if (i == INT2_RCAN_INDEX) {
            s->src[i].vect = 0x200 + INT2_RCAN_SLOT * 0x20;
        } else {
            int index = int2_hpb_slot(i - INT2_TMU_CHANNELS -
                                      (i > INT2_RCAN_INDEX));
            s->src[i].vect = 0x200 + index * 0x20;
        }
    }
    *intc_handle = &s->desc;

    for (int i = 0; i < INT2_TMU_CHANNELS; i++) {
        s->tmu_irq[i] = qemu_allocate_irq(int2_tmu_set, s, i);
    }
    for (int i = 0; i < INT2_HPB_CHANNELS; i++) {
        s->hpb_irq[i] = qemu_allocate_irq(int2_hpb_set, s, i);
    }
    for (int i = 0; i < INT2_SCIF_CHANNEL_COUNT; i++) {
        s->scif_irq[i] = qemu_allocate_irq(int2_scif_set, s, i);
    }
    s->g2d_irq = qemu_allocate_irq(int2_g2d_set, s, 0);
    s->du_irq = qemu_allocate_irq(int2_du_set, s, 0);
    for (int i = 0; i < INT2_SDHI_SOURCES; i++) {
        s->sdhi_irq[i] = qemu_allocate_irq(int2_sdhi_set, s, i);
    }
    s->uart_irq = qemu_allocate_irq(int2_uart_set, s, 0);
    s->rcan_irq = qemu_allocate_irq(int2_rcan_set, s, 0);
    s->extirq = qemu_allocate_irq(int2_extirq_set, s, 0);
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

qemu_irq clarion_qy7_int2_hpb_irq(ClarionQy7Int2 *s)
{
    return clarion_qy7_int2_hpb_channel_irq(s, 0);
}

qemu_irq clarion_qy7_int2_hpb_channel_irq(ClarionQy7Int2 *s, int channel)
{
    g_assert(channel >= 0 && channel < INT2_HPB_CHANNELS);
    return s->hpb_irq[channel];
}

qemu_irq clarion_qy7_int2_scif_irq(ClarionQy7Int2 *s, int channel)
{
    g_assert(channel >= 0 && channel < INT2_SCIF_CHANNEL_COUNT);
    return s->scif_irq[channel];
}


qemu_irq clarion_qy7_int2_g2d_irq(ClarionQy7Int2 *s)
{
    return s->g2d_irq;
}

qemu_irq clarion_qy7_int2_du_irq(ClarionQy7Int2 *s)
{
    return s->du_irq;
}

qemu_irq clarion_qy7_int2_sdhi_irq(ClarionQy7Int2 *s, int unit)
{
    g_assert(unit >= 0 && unit < INT2_SDHI_SOURCES);
    return s->sdhi_irq[unit];
}

qemu_irq clarion_qy7_int2_uart_irq(ClarionQy7Int2 *s)
{
    return s->uart_irq;
}

qemu_irq clarion_qy7_int2_rcan_irq(ClarionQy7Int2 *s)
{
    return s->rcan_irq;
}

qemu_irq clarion_qy7_int2_extirq(ClarionQy7Int2 *s)
{
    return s->extirq;
}
