/*
 * Minimal Clarion SH core initialization-end model.
 *
 * The trigger, 100 ms virtual-clock delay, and HPB status-bit meanings are
 * SYNTHETIC. Shared-memory message queues are not modeled.
 */
#include "qemu/osdep.h"
#include "hw/misc/clarion_shcore.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/timer.h"

#define CLARION_SHCORE_SIZE 0x3c
#define CLARION_SHCORE_REGS (CLARION_SHCORE_SIZE / sizeof(uint32_t))
#define CLARION_SHCORE_START 0x04
#define CLARION_SHCORE_ZERO 0x08
#define CLARION_SHCORE_STATUS 0x1c
#define CLARION_SHCORE_POLL 0x30
#define CLARION_SHCORE_INIT_END 0x4
#define CLARION_SHCORE_ACK 0x100
#define CLARION_SHCORE_DELAY_NS 100000000

struct ClarionSHCoreState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;
    uint32_t regs[CLARION_SHCORE_REGS];
};

static void clarion_shcore_update_irq(ClarionSHCoreState *s)
{
    qemu_set_irq(s->irq, !!(s->regs[CLARION_SHCORE_STATUS / 4] &
                            CLARION_SHCORE_INIT_END));
}

static void clarion_shcore_init_end(void *opaque)
{
    ClarionSHCoreState *s = opaque;

    s->regs[CLARION_SHCORE_STATUS / 4] |= CLARION_SHCORE_INIT_END;
    clarion_shcore_update_irq(s);
    qemu_log_mask(LOG_UNIMP, "clarion-shcore: init end status=%08x irq=1\n",
                  s->regs[CLARION_SHCORE_STATUS / 4]);
}

static uint64_t clarion_shcore_read(void *opaque, hwaddr offset, unsigned size)
{
    ClarionSHCoreState *s = opaque;
    uint32_t value = 0;

    if (size == 4 && offset < CLARION_SHCORE_SIZE && !(offset & 3)) {
        value = s->regs[offset / 4];
        if (offset == CLARION_SHCORE_ZERO) {
            value = 0;
        } else if (offset == CLARION_SHCORE_POLL) {
            value &= ~1U;
        }
    }
    qemu_log_mask(LOG_UNIMP,
                  "clarion-shcore: read offset=%02" HWADDR_PRIx
                  " size=%u value=%08x\n",
                  offset, size, value);
    return value;
}

static void clarion_shcore_write(void *opaque, hwaddr offset, uint64_t value,
                                 unsigned size)
{
    ClarionSHCoreState *s = opaque;

    qemu_log_mask(LOG_UNIMP,
                  "clarion-shcore: write offset=%02" HWADDR_PRIx
                  " size=%u value=%08" PRIx64 "\n",
                  offset, size, value);
    if (size != 4 || offset >= CLARION_SHCORE_SIZE || (offset & 3)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "clarion-shcore: invalid write offset=%02" HWADDR_PRIx
                      " size=%u value=%08" PRIx64 "\n",
                      offset, size, value);
        return;
    }

    if (offset == CLARION_SHCORE_STATUS && (value & CLARION_SHCORE_ACK)) {
        if (s->regs[CLARION_SHCORE_STATUS / 4] & CLARION_SHCORE_INIT_END) {
            s->regs[CLARION_SHCORE_STATUS / 4] &= ~CLARION_SHCORE_INIT_END;
            clarion_shcore_update_irq(s);
            qemu_log_mask(LOG_UNIMP, "clarion-shcore: init end acknowledged\n");
        }
        return;
    }

    if (offset == CLARION_SHCORE_STATUS) {
        uint32_t init_end = s->regs[offset / 4] & CLARION_SHCORE_INIT_END;

        s->regs[offset / 4] = (value & ~CLARION_SHCORE_INIT_END) | init_end;
        clarion_shcore_update_irq(s);
        return;
    }

    s->regs[offset / 4] = value;
    if (offset == CLARION_SHCORE_START && (value & 1)) {
        qemu_log_mask(LOG_UNIMP, "clarion-shcore: init end armed delay_ns=%u\n",
                      CLARION_SHCORE_DELAY_NS);
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                                CLARION_SHCORE_DELAY_NS);
    }
}

static const MemoryRegionOps clarion_shcore_ops = {
    .read = clarion_shcore_read,
    .write = clarion_shcore_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4, .unaligned = false },
    .impl = { .min_access_size = 4, .max_access_size = 4, .unaligned = false },
};

static void clarion_shcore_reset(DeviceState *dev)
{
    ClarionSHCoreState *s = CLARION_SHCORE(dev);

    memset(s->regs, 0, sizeof(s->regs));
    timer_del(s->timer);
    qemu_set_irq(s->irq, 0);
}

static void clarion_shcore_realize(DeviceState *dev, Error **errp)
{
    ClarionSHCoreState *s = CLARION_SHCORE(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    (void)errp;
    memory_region_init_io(&s->iomem, OBJECT(s), &clarion_shcore_ops, s,
                          TYPE_CLARION_SHCORE, CLARION_SHCORE_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, clarion_shcore_init_end, s);
}

static const VMStateDescription vmstate_clarion_shcore = {
    .name = TYPE_CLARION_SHCORE,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields =
        (const VMStateField[]){
            VMSTATE_UINT32_ARRAY(regs, ClarionSHCoreState, CLARION_SHCORE_REGS),
            VMSTATE_TIMER_PTR(timer, ClarionSHCoreState),
            VMSTATE_END_OF_LIST() }
};

static void clarion_shcore_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    (void)data;
    dc->realize = clarion_shcore_realize;
    device_class_set_legacy_reset(dc, clarion_shcore_reset);
    dc->vmsd = &vmstate_clarion_shcore;
}

static const TypeInfo clarion_shcore_info = {
    .name = TYPE_CLARION_SHCORE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ClarionSHCoreState),
    .class_init = clarion_shcore_class_init,
};

static void clarion_shcore_register_types(void)
{
    type_register_static(&clarion_shcore_info);
}

type_init(clarion_shcore_register_types)
