/* Clarion QY7 board registers in SH7785 Area 6. */
#include "qemu/osdep.h"
#include "hw/misc/clarion_boards.h"
#include "hw/misc/clarion_qy7_regs.h"
#include "qemu/log.h"
#include "system/runstate.h"

#define QY7_REGS_BASE 0x19000000
#define QY7_REGS_SIZE 0x00800100
#define QY7_STATUS0 0x19400000
#define QY7_DIPSW 0x19400002
#define QY7_STATUS1 0x19800000
#define QY7_POWER_HOLD (QY7_REGS_BASE + 0x00100000)
#define QY7_EXTINT 0x00300000
#define QY7_EXTINT_END 0x00300026

static const uint8_t ext_status_offsets[] = { 0x00, 0x06, 0x08 };
static const uint8_t ext_mask_offsets[] = { 0x10, 0x14, 0x18 };
#define QY7_EXTINT_POWER_BITS 0x0280

typedef struct QY7BoardRegs {
    MemoryRegion iomem;
    uint8_t dipsw;
    uint16_t status0_inputs;
    bool extint_model;
    qemu_irq extirq;
    uint16_t ext_status[3];
    uint16_t ext_mask[3];
    uint16_t ext_aux[0x13];
    uint16_t ext_delivered;
    bool ext_power_enabled;
    bool power_hold;
} QY7BoardRegs;

static int qy7_ext_group(hwaddr addr, const uint8_t *offsets, size_t count)
{
    for (size_t i = 0; i < count; i++) {
        if (addr == QY7_EXTINT + offsets[i]) {
            return i;
        }
    }
    return -1;
}

static void qy7_extirq_update(QY7BoardRegs *s)
{
    uint16_t pending = s->ext_status[1] & ~s->ext_mask[1];

    qemu_set_irq(s->extirq, pending != 0);
}

static void qy7_ext_power_rearm(QY7BoardRegs *s)
{
    uint16_t active = s->status0_inputs & QY7_EXTINT_POWER_BITS;

    /* Deliver once per active input while guest edge/level mode is unknown. */
    if (s->ext_power_enabled) {
        s->ext_status[1] |= active & ~s->ext_delivered;
    }
    qy7_extirq_update(s);
}

static bool qy7_known(hwaddr addr)
{
    if (addr == QY7_STATUS1 - QY7_REGS_BASE) {
        return true;
    }
    switch (addr) {
    case 0x00000010:
    case 0x00000012:
    case 0x00100000:
    case 0x00200000:
    case 0x00200002:
    case 0x00200004:
    case 0x00300000:
    case 0x00300006:
    case 0x00300008:
    case 0x00300010:
    case 0x00300012:
    case 0x00300014:
    case 0x00300016:
    case 0x00300018:
    case 0x00300020:
    case 0x00300022:
    case 0x00400000:
    case 0x00400002:
    case 0x00400008:
    case 0x0040000a:
    case 0x0040000c:
    case 0x0040000e:
    case 0x00400010:
    case 0x00800000:
        return true;
    default:
        return false;
    }
}

static uint64_t qy7_regs_read(void *opaque, hwaddr addr, unsigned size)
{
    QY7BoardRegs *s = opaque;
    uint16_t value = 0;
    int group;

    if (s->extint_model && size == 2 && addr >= QY7_EXTINT &&
        addr < QY7_EXTINT_END) {
        group = qy7_ext_group(addr, ext_status_offsets,
                              G_N_ELEMENTS(ext_status_offsets));
        if (group >= 0) {
            return s->ext_status[group];
        }
        group = qy7_ext_group(addr, ext_mask_offsets,
                              G_N_ELEMENTS(ext_mask_offsets));
        if (group >= 0) {
            return s->ext_mask[group];
        }
        return s->ext_aux[(addr - QY7_EXTINT) / 2];
    }
    if (size != 2 || !qy7_known(addr)) {
        qemu_log_mask(LOG_UNIMP,
                      "clarion-qy7-regs: unimp read addr=0x%08" HWADDR_PRIx
                      " width=%u\n",
                      QY7_REGS_BASE + addr, size);
        return 0;
    }
    if (addr == QY7_STATUS0 - QY7_REGS_BASE) {
        value = 0x0001 | s->status0_inputs;
    } else if (addr == QY7_STATUS1 - QY7_REGS_BASE) {
        value = 0x0010;
    } else if (addr == QY7_DIPSW - QY7_REGS_BASE) {
        value = s->dipsw << 12;
    }
    qemu_log_mask(LOG_UNIMP,
                  "clarion-qy7-regs: read addr=0x%08" HWADDR_PRIx
                  " width=%u value=0x%04x\n",
                  QY7_REGS_BASE + addr, size, value);
    return value;
}

static void qy7_regs_write(void *opaque, hwaddr addr, uint64_t value,
                           unsigned size)
{
    QY7BoardRegs *s = opaque;
    int group;

    if (s->extint_model && size == 2 && addr >= QY7_EXTINT &&
        addr < QY7_EXTINT_END) {
        group = qy7_ext_group(addr, ext_status_offsets,
                              G_N_ELEMENTS(ext_status_offsets));
        if (group >= 0) {
            s->ext_status[group] &= ~(uint16_t)value;
            if (group == 1 && value == QY7_EXTINT_POWER_BITS &&
                !s->ext_power_enabled) {
                s->ext_power_enabled = true;
            } else if (group == 1 && s->ext_power_enabled) {
                s->ext_delivered |= (uint16_t)value &
                                    QY7_EXTINT_POWER_BITS;
            }
            qy7_ext_power_rearm(s);
            return;
        }
        group = qy7_ext_group(addr, ext_mask_offsets,
                              G_N_ELEMENTS(ext_mask_offsets));
        if (group >= 0) {
            s->ext_mask[group] &= ~(uint16_t)value;
            qy7_ext_power_rearm(s);
            return;
        }
        s->ext_aux[(addr - QY7_EXTINT) / 2] = value;
        return;
    }

    if (size != 2 || !qy7_known(addr)) {
        qemu_log_mask(LOG_UNIMP,
                      "clarion-qy7-regs: unimp write addr=0x%08" HWADDR_PRIx
                      " width=%u value=0x%" PRIx64 "\n",
                      QY7_REGS_BASE + addr, size, value);
        return;
    }
    qemu_log_mask(LOG_UNIMP,
                  "clarion-qy7-regs: write addr=0x%08" HWADDR_PRIx
                  " width=%u value=0x%" PRIx64 "\n",
                  QY7_REGS_BASE + addr, size, value);

    if (addr == QY7_POWER_HOLD - QY7_REGS_BASE && size == 2) {
        if (value == 1) {
            s->power_hold = true;
        } else if (value == 0 && s->power_hold) {
            /* Kernel OFFP clears power hold at these guest code addresses. */
            /* X110: 0x88047a24; X112: 0x88047fb4. */
            s->power_hold = false;
            qemu_log_mask(LOG_GUEST_ERROR,
                          "clarion-qy7-regs: power-hold released; "
                          "requesting power cycle\n");
            qemu_system_shutdown_request_with_code(
                SHUTDOWN_CAUSE_GUEST_SHUTDOWN,
                CLARION_POWER_CYCLE_EXIT_CODE);
        }
    }
}

static const MemoryRegionOps qy7_regs_ops = {
    .read = qy7_regs_read,
    .write = qy7_regs_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

void clarion_qy7_regs_init(MemoryRegion *sysmem, uint8_t dipsw,
                           uint16_t status0_inputs, bool extint_model,
                           qemu_irq extirq)
{
    QY7BoardRegs *s = g_new0(QY7BoardRegs, 1);

    s->dipsw = dipsw;
    s->status0_inputs = status0_inputs;
    s->extint_model = extint_model;
    s->extirq = extirq;
    for (size_t i = 0; i < G_N_ELEMENTS(s->ext_mask); i++) {
        s->ext_mask[i] = 0xffff;
    }
    memory_region_init_io(&s->iomem, NULL, &qy7_regs_ops, s, "clarion-qy7-regs",
                          QY7_REGS_SIZE);
    memory_region_add_subregion(sysmem, QY7_REGS_BASE, &s->iomem);
}
