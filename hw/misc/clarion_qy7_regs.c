/* Clarion QY7 board registers in SH7785 Area 6. */
#include "qemu/osdep.h"
#include "hw/misc/clarion_qy7_regs.h"
#include "qemu/log.h"

#define QY7_REGS_BASE 0x19000000
#define QY7_REGS_SIZE 0x00800100
#define QY7_STATUS0 0x19400000
#define QY7_DIPSW 0x19400002
#define QY7_STATUS1 0x19800000

typedef struct QY7BoardRegs {
    MemoryRegion iomem;
    uint8_t dipsw;
} QY7BoardRegs;

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

    if (size != 2 || !qy7_known(addr)) {
        qemu_log_mask(LOG_UNIMP,
                      "clarion-qy7-regs: unimp read addr=0x%08" HWADDR_PRIx
                      " width=%u\n",
                      QY7_REGS_BASE + addr, size);
        return 0;
    }
    if (addr == QY7_STATUS0 - QY7_REGS_BASE) {
        value = 0x0001;
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
    (void)opaque;

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

void clarion_qy7_regs_init(MemoryRegion *sysmem, uint8_t dipsw)
{
    QY7BoardRegs *s = g_new0(QY7BoardRegs, 1);

    s->dipsw = dipsw;
    memory_region_init_io(&s->iomem, NULL, &qy7_regs_ops, s, "clarion-qy7-regs",
                          QY7_REGS_SIZE);
    memory_region_add_subregion(sysmem, QY7_REGS_BASE, &s->iomem);
}
