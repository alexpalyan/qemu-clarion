/*
 * Clarion QY7 SH-4A watchdog timer.
 *
 * The register layout and counter widths follow the SH7785 Hardware Manual,
 * section 16. QY7 is an SH77722; the peripheral clock used by its WDT has not
 * been measured yet. The machine property wdt-pclk therefore supplies a
 * documented provisional value and permits board-specific overrides.
 *
 * Copyright (C) 2026 Clarion QEMU contributors
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 */

#include "qemu/osdep.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "system/runstate.h"
#include "system/reset.h"
#include "system/system.h"
#include "trace.h"
#include "qom/object.h"

#define TYPE_CLARION_QY7_WDT "clarion-qy7-wdt"
OBJECT_DECLARE_SIMPLE_TYPE(ClarionQy7WdtState, CLARION_QY7_WDT)

#define QY7_WDT_SIZE 0x1000
#define QY7_WDT_WDTST 0x00
#define QY7_WDT_WTCSR 0x04
#define QY7_WDT_WTBST 0x08
#define QY7_WDT_WDTCNT 0x10
#define QY7_WDT_WDTBCNT 0x18

#define QY7_WDT_TME 0x80
#define QY7_WDT_WT 0x40
#define QY7_WDT_RSTS 0x20
#define QY7_WDT_WOVF 0x10
#define QY7_WDT_IOVF 0x08
#define QY7_WDT_WTCSR_MASK 0xf8

#define QY7_WDT_WDTST_MASK 0x00000fff
#define QY7_WDT_WTBST_MASK 0x0003ffff
#define QY7_WDT_WDTCNT_MASK 0x00000fff
#define QY7_WDT_WDTBCNT_MASK 0x0003ffff

#define QY7_WDT_WDTST_KEY 0x5a000000
#define QY7_WDT_WTCSR_KEY 0xa5000000
#define QY7_WDT_WTBST_KEY 0x55000000

#define QY7_WDT_WDTBCNT_BITS 18
#define QY7_WDT_WDTCNT_BITS 12

struct ClarionQy7WdtState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    QEMUTimer *timer;
    uint32_t pclk_hz;
    bool enabled;
    uint32_t wdtst;
    uint32_t wtcsr;
    uint32_t wtbst;
    uint32_t wdtcnt;
    uint32_t wdtbcnt;
    uint64_t pclk_fraction;
    int64_t last_update_ns;
    bool reset_initialized;
    bool overflow_pending;
};

static uint32_t qy7_wdt_base_limit(const ClarionQy7WdtState *s)
{
    return s->wtbst ? s->wtbst : (1U << QY7_WDT_WDTBCNT_BITS);
}

static uint32_t qy7_wdt_stop_limit(const ClarionQy7WdtState *s)
{
    return s->wdtst ? s->wdtst : (1U << QY7_WDT_WDTCNT_BITS);
}

static void qy7_wdt_schedule(ClarionQy7WdtState *s)
{
    uint64_t base_limit;
    uint64_t stop_limit;
    uint64_t base_overflows;
    uint64_t ticks_left;
    uint64_t scaled_ns;
    int64_t now_ns;

    timer_del(s->timer);
    if (!s->enabled || !(s->wtcsr & QY7_WDT_TME) || !s->pclk_hz) {
        return;
    }

    base_limit = qy7_wdt_base_limit(s);
    stop_limit = qy7_wdt_stop_limit(s);
    base_overflows = stop_limit - s->wdtcnt;
    ticks_left = base_limit - s->wdtbcnt + (base_overflows - 1) * base_limit;

    /* pclk_fraction is the fractional Pck numerator left by the last sync. */
    scaled_ns = ticks_left * NANOSECONDS_PER_SECOND - s->pclk_fraction;
    scaled_ns = (scaled_ns + s->pclk_hz - 1) / s->pclk_hz;
    now_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    timer_mod(s->timer, now_ns + MAX(scaled_ns, 1));
}

static void qy7_wdt_expire(ClarionQy7WdtState *s)
{
    bool manual_reset = (s->wtcsr & QY7_WDT_RSTS) != 0;

    s->wtcsr |= QY7_WDT_WOVF;
    s->overflow_pending = true;
    timer_del(s->timer);
    trace_qy7_wdt_timeout(manual_reset, s->pclk_hz, s->wdtst, s->wtbst);

    /*
     * Both reset selections restart QY7 at its IPL vector in this model.
     * The SH7785 distinguishes their external reset signalling; QEMU does not
     * model those board pins, so both use the machine reset path.
     */
    qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
}

static void qy7_wdt_sync(ClarionQy7WdtState *s, int64_t now_ns)
{
    uint64_t elapsed_ns;
    uint64_t scaled_cycles;
    uint64_t pclk_cycles;
    uint64_t base_limit;
    uint64_t stop_limit;
    uint64_t base_total;
    uint64_t overflows;
    uint64_t stop_total;

    if (!s->enabled || !(s->wtcsr & QY7_WDT_TME)) {
        s->last_update_ns = now_ns;
        return;
    }

    elapsed_ns = MAX(now_ns - s->last_update_ns, 0);
    scaled_cycles = elapsed_ns * s->pclk_hz + s->pclk_fraction;
    pclk_cycles = scaled_cycles / NANOSECONDS_PER_SECOND;
    s->pclk_fraction = scaled_cycles % NANOSECONDS_PER_SECOND;
    s->last_update_ns = now_ns;

    base_limit = qy7_wdt_base_limit(s);
    stop_limit = qy7_wdt_stop_limit(s);
    base_total = s->wdtbcnt + pclk_cycles;
    overflows = base_total / base_limit;
    s->wdtbcnt = base_total % base_limit;
    stop_total = s->wdtcnt + overflows;

    if (stop_total >= stop_limit) {
        if (s->wtcsr & QY7_WDT_WT) {
            qy7_wdt_expire(s);
            return;
        }
        s->wtcsr |= QY7_WDT_IOVF;
        s->wdtcnt = stop_total % stop_limit;
    } else {
        s->wdtcnt = stop_total;
    }
    qy7_wdt_schedule(s);
}

static void qy7_wdt_timer_expired(void *opaque)
{
    ClarionQy7WdtState *s = opaque;

    qy7_wdt_sync(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
}

static uint32_t qy7_wdt_read_register(ClarionQy7WdtState *s, hwaddr offset)
{
    switch (offset) {
    case QY7_WDT_WDTST:
        return s->wdtst;
    case QY7_WDT_WTCSR:
        return s->wtcsr & QY7_WDT_WTCSR_MASK;
    case QY7_WDT_WTBST:
        return s->wtbst;
    case QY7_WDT_WDTCNT:
        return s->wdtcnt & QY7_WDT_WDTCNT_MASK;
    case QY7_WDT_WDTBCNT:
        return s->wdtbcnt & QY7_WDT_WDTBCNT_MASK;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "qy7.wdt: invalid read at offset 0x%" HWADDR_PRIx "\n",
                      offset);
        return 0;
    }
}

static uint64_t qy7_wdt_read(void *opaque, hwaddr offset, unsigned size)
{
    ClarionQy7WdtState *s = CLARION_QY7_WDT(opaque);

    if (size != 4 || (offset & 3)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "qy7.wdt: invalid read size %u at offset 0x%" HWADDR_PRIx
                      "\n",
                      size, offset);
        return 0;
    }

    qy7_wdt_sync(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    return qy7_wdt_read_register(s, offset);
}

static bool qy7_wdt_valid_write(ClarionQy7WdtState *s, hwaddr offset,
                                uint64_t value, unsigned size)
{
    if (size != 4 || (offset & 3)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "qy7.wdt: invalid write size %u at offset 0x%" HWADDR_PRIx
                      "\n",
                      size, offset);
        return false;
    }

    switch (offset) {
    case QY7_WDT_WDTST:
        if ((value & 0xff000000) != QY7_WDT_WDTST_KEY || (value & 0x00fff000)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "qy7.wdt: invalid WDTST write 0x%08" PRIx64 "\n",
                          value);
            return false;
        }
        break;
    case QY7_WDT_WTCSR:
        if ((value & 0xff000000) != QY7_WDT_WTCSR_KEY || (value & 0x00ffff07)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "qy7.wdt: invalid WTCSR write 0x%08" PRIx64 "\n",
                          value);
            return false;
        }
        break;
    case QY7_WDT_WTBST:
        if ((value & 0xff000000) != QY7_WDT_WTBST_KEY || (value & 0x00fc0000)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "qy7.wdt: invalid WTBST write 0x%08" PRIx64 "\n",
                          value);
            return false;
        }
        break;
    default:
        qemu_log_mask(
            LOG_GUEST_ERROR,
            "qy7.wdt: write to read-only or invalid offset 0x%" HWADDR_PRIx
            "\n",
            offset);
        return false;
    }
    return true;
}

static void qy7_wdt_write(void *opaque, hwaddr offset, uint64_t value,
                          unsigned size)
{
    ClarionQy7WdtState *s = CLARION_QY7_WDT(opaque);
    uint32_t old_wtcsr;
    int64_t now_ns;

    if (!qy7_wdt_valid_write(s, offset, value, size)) {
        return;
    }

    now_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    qy7_wdt_sync(s, now_ns);
    if (s->overflow_pending) {
        return;
    }

    switch (offset) {
    case QY7_WDT_WDTST:
        s->wdtst = value & QY7_WDT_WDTST_MASK;
        s->wdtcnt = 0;
        break;
    case QY7_WDT_WTCSR:
        old_wtcsr = s->wtcsr;
        s->wtcsr = value & QY7_WDT_WTCSR_MASK;
        if (!(s->wtcsr & QY7_WDT_TME) || !(old_wtcsr & QY7_WDT_TME)) {
            s->last_update_ns = now_ns;
            s->pclk_fraction = 0;
        }
        break;
    case QY7_WDT_WTBST:
        s->wtbst = value & QY7_WDT_WTBST_MASK;
        s->wdtbcnt = 0;
        s->last_update_ns = now_ns;
        s->pclk_fraction = 0;
        trace_qy7_wdt_feed(s->wdtst, s->wtbst);
        break;
    }
    qy7_wdt_schedule(s);
}

static const MemoryRegionOps qy7_wdt_ops = {
    .read = qy7_wdt_read,
    .write = qy7_wdt_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void qy7_wdt_reset_enter(Object *obj, ResetType type)
{
    ClarionQy7WdtState *s = CLARION_QY7_WDT(obj);

    timer_del(s->timer);
    if (!s->reset_initialized) {
        s->wdtst = 0;
        s->wtcsr = 0;
        s->wtbst = 0;
        s->wdtcnt = 0;
        s->wdtbcnt = 0;
        s->pclk_fraction = 0;
        s->reset_initialized = true;
    } else if (s->overflow_pending) {
        /* SH7785 clears the counters when leaving a WDT reset sequence. */
        s->wdtcnt = 0;
        s->wdtbcnt = 0;
        s->pclk_fraction = 0;
    }
    s->overflow_pending = false;
    s->last_update_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    (void)type;
}

static void qy7_wdt_reset_exit(Object *obj, ResetType type)
{
    ClarionQy7WdtState *s = CLARION_QY7_WDT(obj);

    qy7_wdt_schedule(s);
    (void)type;
}

static void qy7_wdt_realize(DeviceState *dev, Error **errp)
{
    ClarionQy7WdtState *s = CLARION_QY7_WDT(dev);

    if (!s->pclk_hz) {
        error_setg(errp, "qy7.wdt: pclk-hz must be greater than zero");
        return;
    }
    memory_region_init_io(&s->iomem, OBJECT(s), &qy7_wdt_ops, s, "qy7.wdt",
                          QY7_WDT_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    /* Virtual time pauses while the VM is stopped, including under GDB. */
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, qy7_wdt_timer_expired, s);
    s->last_update_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static const Property qy7_wdt_properties[] = {
    DEFINE_PROP_UINT32("pclk-hz", ClarionQy7WdtState, pclk_hz, 24000000),
    DEFINE_PROP_BOOL("enabled", ClarionQy7WdtState, enabled, true),
};

static void qy7_wdt_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = qy7_wdt_realize;
    dc->desc = "Clarion QY7 SH-4A watchdog timer";
    device_class_set_props(dc, qy7_wdt_properties);
    rc->phases.enter = qy7_wdt_reset_enter;
    rc->phases.exit = qy7_wdt_reset_exit;
}

static const TypeInfo qy7_wdt_type_info = {
    .name = TYPE_CLARION_QY7_WDT,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ClarionQy7WdtState),
    .class_init = qy7_wdt_class_init,
};

static void qy7_wdt_register_types(void)
{
    type_register_static(&qy7_wdt_type_info);
}

type_init(qy7_wdt_register_types)
