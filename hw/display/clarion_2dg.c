/*
 * Minimal Clarion 2DG completion model.
 *
 * The command list is observed but never executed. The 1 ms completion
 * delay and the +0x0c enable-bit meaning are SYNTHETIC.
 */
#include "qemu/osdep.h"
#include "hw/display/clarion_2dg.h"
#include "system/address-spaces.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/timer.h"

#define CLARION_2DG_SIZE 0x200
#define CLARION_2DG_REGS (CLARION_2DG_SIZE / sizeof(uint32_t))
#define CLARION_2DG_STATUS 0x04
#define CLARION_2DG_ACK 0x08
#define CLARION_2DG_IRQ_ENABLE 0x0c
#define CLARION_2DG_LIST 0x48
#define CLARION_2DG_DONE 1
#define CLARION_2DG_DELAY_NS 1000000
#define CLARION_2DG_LOG_WORDS 256

struct Clarion2DGState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;
    uint32_t regs[CLARION_2DG_REGS];
    char *log_path;
};

static void clarion_2dg_update_irq(Clarion2DGState *s)
{
    qemu_set_irq(s->irq, (s->regs[CLARION_2DG_STATUS / 4] & CLARION_2DG_DONE) &&
                             (s->regs[CLARION_2DG_IRQ_ENABLE / 4] & 1));
}

static void clarion_2dg_log_list(Clarion2DGState *s, uint32_t address)
{
    FILE *file;
    uint32_t words[CLARION_2DG_LOG_WORDS];
    char *json;
    GString *line;
    unsigned i;

    if (!s->log_path || !*s->log_path) {
        return;
    }
    for (i = 0; i < CLARION_2DG_LOG_WORDS; i++) {
        uint32_t word = 0;

        address_space_read(
            &address_space_memory, (hwaddr)address + i * sizeof(word),
            MEMTXATTRS_UNSPECIFIED, (uint8_t *)&word, sizeof(word));
        words[i] = le32_to_cpu(word);
    }
    line = g_string_new(NULL);
    g_string_append_printf(
        line, "{\"time_ns\":%" PRId64 ",\"address\":%u,\"words\":[",
        qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL), address);
    for (i = 0; i < CLARION_2DG_LOG_WORDS; i++) {
        g_string_append_printf(line, "%s%u", i ? "," : "", words[i]);
    }
    g_string_append(line, "]}\n");
    json = g_string_free(line, FALSE);
    file = fopen(s->log_path, "a");
    if (file) {
        fputs(json, file);
        fclose(file);
    } else {
        qemu_log_mask(LOG_GUEST_ERROR, "clarion-2dg: cannot write %s: %s\n",
                      s->log_path, strerror(errno));
    }
    g_free(json);
}

static void clarion_2dg_complete(void *opaque)
{
    Clarion2DGState *s = opaque;

    s->regs[CLARION_2DG_STATUS / 4] |= CLARION_2DG_DONE;
    clarion_2dg_update_irq(s);
    qemu_log_mask(LOG_UNIMP, "clarion-2dg: completion status=%08x irq=%u\n",
                  s->regs[CLARION_2DG_STATUS / 4],
                  !!((s->regs[CLARION_2DG_STATUS / 4] & CLARION_2DG_DONE) &&
                     (s->regs[CLARION_2DG_IRQ_ENABLE / 4] & 1)));
}

static uint64_t clarion_2dg_read(void *opaque, hwaddr offset, unsigned size)
{
    Clarion2DGState *s = opaque;
    uint32_t value = 0;

    if (size == 4 && offset < CLARION_2DG_SIZE && !(offset & 3)) {
        value = s->regs[offset / 4];
    }
    qemu_log_mask(LOG_UNIMP,
                  "clarion-2dg: read offset=%03" HWADDR_PRIx
                  " size=%u value=%08x\n",
                  offset, size, value);
    return value;
}

static void clarion_2dg_write(void *opaque, hwaddr offset, uint64_t value,
                              unsigned size)
{
    Clarion2DGState *s = opaque;
    uint32_t old;

    if (size != 4 || offset >= CLARION_2DG_SIZE || (offset & 3)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "clarion-2dg: invalid write offset=%03" HWADDR_PRIx
                      " size=%u value=%08" PRIx64 "\n",
                      offset, size, value);
        return;
    }
    qemu_log_mask(LOG_UNIMP,
                  "clarion-2dg: write offset=%03" HWADDR_PRIx
                  " value=%08" PRIx64 "\n",
                  offset, value);
    if (offset == CLARION_2DG_STATUS) {
        return;
    }
    if (offset == CLARION_2DG_ACK) {
        if ((value & CLARION_2DG_DONE) &&
            (s->regs[CLARION_2DG_STATUS / 4] & CLARION_2DG_DONE)) {
            s->regs[CLARION_2DG_STATUS / 4] &= ~CLARION_2DG_DONE;
            clarion_2dg_update_irq(s);
            qemu_log_mask(LOG_UNIMP, "clarion-2dg: acknowledge\n");
        }
        s->regs[offset / 4] = value;
        return;
    }
    old = s->regs[offset / 4];
    s->regs[offset / 4] = value;
    if (offset == CLARION_2DG_IRQ_ENABLE) {
        clarion_2dg_update_irq(s);
    }
    if (offset == 0 && (value & 1)) {
        if (timer_pending(s->timer)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "clarion-2dg: start while previous blit pending\n");
        }
        qemu_log_mask(LOG_UNIMP, "clarion-2dg: start list=%08x first_words=",
                      s->regs[CLARION_2DG_LIST / 4]);
        for (unsigned i = 0; i < 16; i++) {
            uint32_t word = 0;

            address_space_read(&address_space_memory,
                               s->regs[CLARION_2DG_LIST / 4] + i * 4,
                               MEMTXATTRS_UNSPECIFIED, (uint8_t *)&word, 4);
            qemu_log_mask(LOG_UNIMP, "%s%08x", i ? "," : " ",
                          le32_to_cpu(word));
        }
        qemu_log_mask(LOG_UNIMP, "\n");
        clarion_2dg_log_list(s, s->regs[CLARION_2DG_LIST / 4]);
        s->regs[CLARION_2DG_STATUS / 4] &= ~CLARION_2DG_DONE;
        clarion_2dg_update_irq(s);
        timer_mod(s->timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + CLARION_2DG_DELAY_NS);
    }
    if (old != s->regs[offset / 4]) {
        clarion_2dg_update_irq(s);
    }
}

static const MemoryRegionOps clarion_2dg_ops = {
    .read = clarion_2dg_read,
    .write = clarion_2dg_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4, .unaligned = false },
    .impl = { .min_access_size = 4, .max_access_size = 4, .unaligned = false },
};

static void clarion_2dg_reset(DeviceState *dev)
{
    Clarion2DGState *s = CLARION_2DG(dev);

    memset(s->regs, 0, sizeof(s->regs));
    timer_del(s->timer);
    qemu_set_irq(s->irq, 0);
}

static void clarion_2dg_realize(DeviceState *dev, Error **errp)
{
    Clarion2DGState *s = CLARION_2DG(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    (void)errp;
    memory_region_init_io(&s->iomem, OBJECT(s), &clarion_2dg_ops, s,
                          TYPE_CLARION_2DG, CLARION_2DG_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, clarion_2dg_complete, s);
}

static const VMStateDescription vmstate_clarion_2dg = {
    .name = TYPE_CLARION_2DG,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields =
        (const VMStateField[]){
            VMSTATE_UINT32_ARRAY(regs, Clarion2DGState, CLARION_2DG_REGS),
            VMSTATE_TIMER_PTR(timer, Clarion2DGState), VMSTATE_END_OF_LIST() }
};

static const Property clarion_2dg_properties[] = {
    DEFINE_PROP_STRING("log", Clarion2DGState, log_path),
};

static void clarion_2dg_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    (void)data;
    dc->realize = clarion_2dg_realize;
    device_class_set_legacy_reset(dc, clarion_2dg_reset);
    dc->vmsd = &vmstate_clarion_2dg;
    device_class_set_props(dc, clarion_2dg_properties);
}

static const TypeInfo clarion_2dg_info = {
    .name = TYPE_CLARION_2DG,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Clarion2DGState),
    .class_init = clarion_2dg_class_init,
};

static void clarion_2dg_register_types(void)
{
    type_register_static(&clarion_2dg_info);
}

type_init(clarion_2dg_register_types)
