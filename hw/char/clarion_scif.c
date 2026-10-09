/*
 * Shared Renesas SCIF model used by Clarion QY7 and QY8.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/char/clarion_scif.h"
#include "system/system.h"

#define SCIF_SCSMR 0x00
#define SCIF_SCBRR 0x04
#define SCIF_SCSCR 0x08
#define SCIF_SCFTDR 0x0C
#define SCIF_SCFSR 0x10
#define SCIF_SCFRDR 0x14
#define SCIF_SCFCR 0x18
#define SCIF_SCFDR 0x1C
#define SCIF_SCSPTR 0x20
#define SCIF_SCLSR 0x24

#define SCFSR_DR 0x0001
#define SCFSR_RDF 0x0002
#define SCFSR_TDFE 0x0020
#define SCFSR_TEND 0x0040

#define SCFCR_RFRST 0x0002
#define CLARION_SCIF_IDLE_NS 200000
#define SCSCR_RIE 0x0040
#define SCSCR_TIE 0x0080

static unsigned clarion_scif_rtrg(ClarionScif *s)
{
    static const unsigned lvl[4] = { 1, 4, 8, 14 };
    return lvl[(s->scfcr >> 6) & 3];
}

static bool clarion_scif_rdf(ClarionScif *s)
{
    return s->fifo_len >= clarion_scif_rtrg(s);
}

static void clarion_scif_update_irq(ClarionScif *s)
{

    if (s->irq) {
        qemu_set_irq(s->irq, ((clarion_scif_rdf(s) || s->dr) &&
                              (s->scscr & SCSCR_RIE)) ||
                                 (s->txi && (s->scscr & SCSCR_TIE)));
    }
}

static void clarion_scif_rx_pump(ClarionScif *s)
{
    unsigned taken = 0;

    if (s->dmac && (s->fifo_len >= clarion_scif_rtrg(s) || s->dr)) {
        while (taken < s->fifo_len &&
               s->dma_feed(s->dmac, s->base + SCIF_SCFRDR, s->fifo[taken])) {
            taken++;
        }
    }
    if (taken) {
        memmove(s->fifo, s->fifo + taken, s->fifo_len - taken);
        s->fifo_len -= taken;
        if (!s->fifo_len) {
            s->dr = false;
        }
    }
    clarion_scif_update_irq(s);
}

static void clarion_scif_idle_expire(void *opaque)
{
    ClarionScif *s = opaque;

    if (s->fifo_len) {
        s->dr = true;
        clarion_scif_rx_pump(s);
    }

    if (s->dmac) {
        if (s->dma_eod) {
            s->dma_eod(s->dmac, s->base + SCIF_SCFRDR);
        }
    }
    if (s->fifo_len) {

        timer_mod(s->idle,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + CLARION_SCIF_IDLE_NS);
    }
}

static uint64_t clarion_scif_read(void *opaque, hwaddr addr, unsigned size)
{
    ClarionScif *s = opaque;

    switch (addr) {
    case SCIF_SCSMR:
        return s->scsmr;
    case SCIF_SCSCR:
        return s->scscr;
    case SCIF_SCFSR:

        return SCFSR_TDFE | SCFSR_TEND | (clarion_scif_rdf(s) ? SCFSR_RDF : 0) |
               (s->dr ? SCFSR_DR : 0);
    case SCIF_SCFRDR: {
        uint8_t v = s->fifo_len ? s->fifo[0] : 0;

        if (s->fifo_len) {
            memmove(s->fifo, s->fifo + 1, --s->fifo_len);
        }
        if (!s->fifo_len) {
            s->dr = false;
        }
        clarion_scif_update_irq(s);
        return v;
    }
    case SCIF_SCFCR:
        return s->scfcr;
    case SCIF_SCFDR:

        return s->fifo_len;
    case SCIF_SCSPTR:
        return 0;
    case SCIF_SCLSR:
        return 0;
    default:
        return 0;
    }
}

static void clarion_scif_write(void *opaque, hwaddr addr, uint64_t val,
                               unsigned size)
{
    ClarionScif *s = opaque;
    uint8_t ch;

    switch (addr) {
    case SCIF_SCSMR:
        s->scsmr = val;
        break;
    case SCIF_SCSCR:
        s->scscr = val;
        clarion_scif_update_irq(s);
        break;
    case SCIF_SCFTDR:
        ch = val & 0xff;

        if (s->micom) {
            s->micom_tx(s->micom, ch);
            break;
        }
        if (s->dispmicom) {
            s->dispmicom_tx(s->dispmicom, ch);
            break;
        }
        if (s->ublox) {
            s->ublox_tx(s->ublox, ch);
            break;
        }

        qemu_chr_fe_write_all(&s->chr, &ch, 1);
        break;
    case SCIF_SCFCR:
        s->scfcr = val;
        if (val & SCFCR_RFRST) {
            s->fifo_len = 0;
            s->dr = false;
        }
        clarion_scif_update_irq(s);
        break;
    case SCIF_SCFSR:

        if (!(val & SCFSR_DR)) {
            s->dr = false;
        }
        clarion_scif_update_irq(s);
        break;
    default:
        break;
    }
}

static const MemoryRegionOps clarion_scif_ops = {
    .read = clarion_scif_read,
    .write = clarion_scif_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

int clarion_scif_can_receive(void *opaque)
{
    ClarionScif *s = opaque;
    return CLARION_SCIF_FIFO - s->fifo_len;
}

void clarion_scif_receive(void *opaque, const uint8_t *buf, int size)
{
    ClarionScif *s = opaque;

    int i;

    for (i = 0; i < size && s->fifo_len < CLARION_SCIF_FIFO; i++) {
        s->fifo[s->fifo_len++] = buf[i];
    }
    s->dr = false;
    clarion_scif_rx_pump(s);
    timer_mod(s->idle,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + CLARION_SCIF_IDLE_NS);
}

void clarion_scif_init(ClarionScif *s, MemoryRegion *sysmem, hwaddr base,
                       const char *name, int index, int serial_index,
                       qemu_irq irq, DeviceState *dmac, bool txi)
{
    s->index = index;
    s->txi = txi;
    s->base = base;
    s->irq = irq;
    s->dmac = dmac;
    s->idle = timer_new_ns(QEMU_CLOCK_VIRTUAL, clarion_scif_idle_expire, s);
    memory_region_init_io(&s->mr, NULL, &clarion_scif_ops, s, name, 0x100);
    memory_region_add_subregion(sysmem, base, &s->mr);
    if (serial_hd(serial_index)) {
        qemu_chr_fe_init(&s->chr, serial_hd(serial_index), &error_abort);
        qemu_chr_fe_set_handlers(&s->chr, clarion_scif_can_receive,
                                 clarion_scif_receive, NULL, NULL, s, NULL,
                                 true);
    }
}
