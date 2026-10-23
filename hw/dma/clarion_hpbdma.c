/*
 * Renesas HPB-DMAC (High Performance Bus DMA controller, R8A7778) —
 * model for the Clarion QY8XXX board.
 *
 * The block is identified by an exact match of the full register map, not
 * similar addresses: the guest writes seven registers at a 0x40 stride, and
 * every offset matches drivers/dma/sh/rcar-hpbdma.c. Bases and interrupts
 * come from arch/arm/mach-shmobile/setup-r8a7778.c (hpb_dmae_resources[]),
 * the platform file for this SoC:
 *
 *   channel registers 0xffc08000/0x1000, shared registers 0xffc09000/0x170,
 *   async reset 0xffc00300, async mode 0xffc00400,
 *   interrupts gic_iid(0x7b), 5 lines, IRQ 123..127.
 *
 * Why WinCE needs this model: firmware sets up two SCIF4 channels:
 *
 *   channel 4: DSAR0/DSAR1 = 0xffe44014 (SCIF4 SCFRDR), DDAR0/DDAR1 are
 *              two 0x80-byte RAM buffers, DCR = CT|DIP|SMDL
 *              (continuous receive into a double buffer);
 *   channel 6: DSAR0 = RAM buffer, DDAR0 = 0xffe4400c (SCIF4 SCFTDR),
 *              DTCR0 = 8, DCR = DMDL (transmit).
 *
 * The TX buffer contains 10 02 | 02 08 00 | 10 03 | 19: DLE STX / data /
 * DLE ETX / checksum. SCIF4 is therefore the link to the board's companion
 * MCU (the firmware includes cpucom.dll and several *MicomUpdate.exe tools),
 * not a debug console.
 *
 * A channel's module side is selected by CHANNEL BINDING (table
 * hpb_chan_module below), not by guest-written DSAR/DDAR values. The model
 * previously matched DSAR against peripheral addresses, which happened to
 * work for SCIF but could never work for SDHI. See the table comment.
 *
 * ⚠ Scope: memory-to-module transfers happen immediately because the modeled
 * SCIF is always ready; this matches the model rather than approximating a
 * wait. Module-to-memory transfers are NOT performed blindly: the channel is
 * armed and SCIF supplies bytes via clarion_hpbdma_feed(), avoiding invented
 * data. Until SCIF4 receives a response, its RX channel remains active but
 * transfers nothing.
 *
 * Deliberate omissions (sources do not establish behavior):
 *   DPTR: firmware writes 4 and 0x400, Linux driver does not use it; store
 *         without interpretation;
 *   DTIMR, DMASPR, DMLVLR, DMSHPT: named in the driver only;
 *   hardware transfer rate: model transfers everything in one tick.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/dma/clarion_hpbdma.h"
#include "hw/core/qdev-properties.h"
#include "system/address-spaces.h"
#include "qom/object.h"
#include "trace.h"
#include "hw/core/cpu.h"

/* --- channel registers (offsets from rcar-hpbdma.c) -------------------- */

#define HPB_DSAR0       0x00
#define HPB_DDAR0       0x04
#define HPB_DTCR0       0x08
#define HPB_DSAR1       0x0C
#define HPB_DDAR1       0x10
#define HPB_DTCR1       0x14
#define HPB_DSASR       0x18
#define HPB_DDASR       0x1C
#define HPB_DTCSR       0x20
#define HPB_DPTR        0x24
#define HPB_DCR         0x28
#define HPB_DCMDR       0x2C
#define HPB_DSTPR       0x30
#define HPB_DSTSR       0x34
#define HPB_DDBGR       0x38
#define HPB_DDBGR2      0x3C
#define HPB_CHAN_STRIDE 0x40

/* DCMDR */
#define DCMDR_BDOUT     (1u << 7)
#define DCMDR_DQSPD     (1u << 6)
#define DCMDR_DQSPC     (1u << 5)
#define DCMDR_DMSPD     (1u << 4)
#define DCMDR_DMSPC     (1u << 3)
#define DCMDR_DQEND     (1u << 2)
#define DCMDR_DNXT      (1u << 1)
#define DCMDR_DMEN      (1u << 0)

/* DSTPR / DSTSR */
#define DSTPR_DMSTP     (1u << 0)
#define DSTSR_NDP1      (1u << 6)
#define DSTSR_NDP0      (1u << 5)
#define DSTSR_DQSTS     (1u << 2)
#define DSTSR_DRSTS     (1u << 1)
#define DSTSR_DMSTS     (1u << 0)

/* DCR (include/linux/platform_data/dma-rcar-hpbdma.h) */
#define DCR_CT          (1u << 18)
#define DCR_DIP         (1u << 16)
#define DCR_SMDL        (1u << 13)
#define DCR_SPDS_MASK   (3u << 8)
#define DCR_SPDS_SHIFT  8
#define DCR_DMDL        (1u << 5)
#define DCR_DPDS_MASK   (3u << 0)
#define DCR_DPDS_SHIFT  0

/* --- shared registers -------------------------------------------------- */

#define HPB_DTIMR       0x00
#define HPB_DINTSR0     0x0C
#define HPB_DINTSR1     0x10
#define HPB_DINTCR0     0x14
#define HPB_DINTCR1     0x18
#define HPB_DINTMR0     0x1C
#define HPB_DINTMR1     0x20
#define HPB_QY7_DTE0    0x810
#define HPB_QY7_DTE1    0x814
#define HPB_DACTSR0     0x24
#define HPB_DACTSR1     0x28
#define HPB_HSRSTR(n)   (0x40 + (n) * 4)

typedef struct ClarionHpbChan {
    uint32_t sar[2], dar[2], tcr[2];    /* two planes (DIP) */
    uint32_t dsasr, ddasr, dtcsr;
    uint32_t dptr, dcr, dstsr;
    uint32_t left;                      /* remaining transfer units */
    unsigned plane;                     /* current plane */
    bool next_requested;                /* DNXT: next plane requested */
    bool active;
} ClarionHpbChan;

/*
 * Module that buffers its own data and raises a DMA request (see header).
 * This board has two: SDHI0 and SDHI1; four slots provide headroom.
 */
#define HPB_MAX_MODULE  4

typedef struct ClarionHpbModule {
    hwaddr addr;                        /* module data register */
    ClarionHpbModuleReady ready;        /* "is data ready now?" */
    void *opaque;
} ClarionHpbModule;

struct ClarionHpbDmaState {
    SysBusDevice parent_obj;

    MemoryRegion chan_mr;
    MemoryRegion comm_mr;
    qemu_irq irq[CLARION_HPBDMA_NUM_IRQ];

    ClarionHpbChan ch[CLARION_HPBDMA_NUM_CHAN];
    uint32_t dintsr[2];                 /* interrupt status */
    uint32_t dintmr[2];                 /* masks */
    uint32_t dtimr;
    uint32_t qy7_dte[2];

    ClarionHpbModule module[HPB_MAX_MODULE];
    unsigned n_module;
    bool servicing;                     /* re-entry guard */
    bool qy7_mode;
    /* channels already reported, so a log line comes once per channel */
    uint64_t port_widths_logged;
    uint64_t no_binding_logged;
    uint64_t no_irq_line_logged;
};

OBJECT_DECLARE_SIMPLE_TYPE(ClarionHpbDmaState, CLARION_HPBDMA)

/*
 * Channel -> interrupt line.
 *
 * The hpb_dmae_channels[] table in setup-r8a7778.c gives ch_irq for Linux
 * channels: 0x7c (IRQ 124) for 14/15 (USB function), 0x7e (IRQ 126) for
 * 21/22 (SDHI0), and 0x7f (IRQ 127) for 28..36 (SSI/HPBIF). Linux does not
 * use channels 4 and 6, but the guest binds them: it uses 4/6 (micom) and
 * 21/22 (SDHI), and enables only IRQ 123 and 126 in the GIC. Since 126 maps
 * to SDHI, 123 maps to channels 4/6, the lowest of the five lines.
 *
 * Channels 16/17 are HSCIF0 (base 0xFFE48000). Their line is also derived
 * from guest behavior rather than guessed:
 *   - DINTMR0 enables exactly channels 4, 16 and 17;
 *   - of the five DMAC lines, the guest enables IRQ 123, 125 and 126 in GIC;
 *   - as above, 123 belongs to channels 4/6 (micom), and 126 to 21/22 (SDHI);
 *   - 125 (line 2) is therefore the only unassigned line, while 16/17 are
 *     the only channels without a line.
 * A test confirmed the alternative wrong: while channel 17 used line 0, the
 * SPI 91 handler found no matching channel, did not write DINTCR0, and the
 * level stayed high, causing ~4100 IRQ/s and starving other work.
 *
 * No source identifies lines for other channels; use line 0 and log it.
 */
static int hpb_chan_irq_line(ClarionHpbDmaState *s, int ch)
{
    if (ch == 14 || ch == 15) {
        return 1;                       /* 0x7c = IRQ 124 */
    }
    if (ch == 21 || ch == 22) {
        return 3;                       /* 0x7e = IRQ 126 */
    }
    if (ch >= 28 && ch <= 36) {
        return 4;                       /* 0x7f = IRQ 127 */
    }
    if (ch == 16 || ch == 17) {
        return 2;                       /* IRQ 125, HSCIF0; see above */
    }
    if (ch != 4 && ch != 6 && !(s->no_irq_line_logged & BIT_ULL(ch))) {
        s->no_irq_line_logged |= BIT_ULL(ch);
        qemu_log_mask(LOG_UNIMP,
                      "clarion-hpbdma: channel %d has no interrupt line in "
                      "available sources; using the lowest line\n", ch);
    }
    return 0;                           /* gic_iid(0x7b) = IRQ 123 */
}

static void hpb_update_irq(ClarionHpbDmaState *s)
{
    bool raised[CLARION_HPBDMA_NUM_IRQ] = { false };
    int ch, i;

    if (s->qy7_mode) {
        uint32_t active = s->dintsr[0] & s->qy7_dte[1];

        for (i = 0; i < CLARION_HPBDMA_QY7_NUM_IRQ; i++) {
            qemu_set_irq(s->irq[i], !!(active & (1u << i)));
        }
        return;
    }

    for (ch = 0; ch < CLARION_HPBDMA_NUM_CHAN; ch++) {
        int w = ch / 32, b = ch % 32;

        if ((s->dintsr[w] & s->dintmr[w]) & (1u << b)) {
            raised[hpb_chan_irq_line(s, ch)] = true;
        }
    }
    for (i = 0; i < CLARION_HPBDMA_QY8_NUM_IRQ; i++) {
        qemu_set_irq(s->irq[i], raised[i]);
    }
}

static void hpb_chan_complete(ClarionHpbDmaState *s, int ch)
{
    int w = ch / 32, b = ch % 32;

    trace_clarion_hpbdma_complete(ch, s->ch[ch].dcr, s->ch[ch].dsasr,
                                  s->ch[ch].ddasr, s->ch[ch].dtcsr);
    s->dintsr[w] |= 1u << b;
    hpb_update_irq(s);
}

/*
 * --- Channel-to-peripheral bindings -----------------------------------
 *
 * HPB-DMAC selects a channel's peripheral ("module") side by CHANNEL, not by
 * DSAR/DDAR address. The guest declares these bindings in ASYNCRSTR
 * (0xFFC00300) and ASYNCMDR (0xFFC00400), with one bit per channel.
 *
 * Why the model no longer matches DSAR to the peripheral address:
 * clarion_hpbdma_feed() previously searched for `dsasr == periph_addr`. This
 * worked BY ACCIDENT for SCIF because EdaDrv stores the register's physical
 * address there. SDHC.dll stores a VIRTUAL SD_BUF0 address in DSAR, correctly:
 * its VA->PA translation works for memory buffers and is deliberately not
 * applied to the module side. An address match for SDHI would never occur.
 *
 * ⚠ We do NOT know how hardware uses guest-written DSAR/DDAR values on the
 * module side; available sources do not describe it. The model makes no
 * assumption: known channels use register addresses from this table, and
 * guest-written module-side values are not interpreted.
 *
 * Binding sources vary by row:
 *
 *   4, 6    SCIF4   - guest behavior: DSAR0=0xFFE44014 (SCFRDR) on channel
 *                     4 and DDAR0=0xFFE4400C (SCFTDR) on channel 6.
 *   16, 17  HSCIF0  - guest behavior: 0xFFE48014 / 0xFFE4800C.
 *   21, 22  SDHI0   - two sources. setup-r8a7778.c (v4.0): SDHI0_TX
 *                     .dma_ch=21, SDHI0_RX .dma_ch=22, both .addr =
 *                     0xffe4c000 + 0x30. The guest also sets ASYNCRSTR bit 1
 *                     (ASRST21) and ASYNCMDR ASMD21=MULTI, the bits Linux
 *                     labels "SDHI0".
 *   14, 15  USB function - setup-r8a7778.c only; this firmware does not use
 *                     them, so these entries are not verified by a live guest.
 *
 * Channel does NOT determine direction: DCR.SMDL/DMDL does. The guest uses
 * channel 21 for reads (SMDL), although Linux labels channel 21 TX.
 */
#define HPB_NO_MODULE   0

static const hwaddr hpb_chan_module[CLARION_HPBDMA_NUM_CHAN] = {
    [3]  = 0xFFE43014,      /* SCIF3 SCFRDR */
    [4]  = 0xFFE44014,      /* SCIF4  SCFRDR  - guest                    */
    [6]  = 0xFFE4400C,      /* SCIF4  SCFTDR  - guest                    */
    [14] = 0xFFE60018,      /* USB function D0 - Linux, unverified       */
    [15] = 0xFFE6001C,      /* USB function D1 - Linux, unverified       */
    [16] = 0xFFE48014,      /* HSCIF0 HSRDR   - guest                    */
    [17] = 0xFFE4800C,      /* HSCIF0 HSTDR   - guest                    */
    [21] = 0xFFE4C030,      /* SDHI0  SD_BUF0 - Linux + guest ASYNCMDR   */
    [22] = 0xFFE4C030,      /* SDHI0  SD_BUF0 — Linux                   */
};

/*
 * Return the module-side address for a channel. Unknown channels return
 * HPB_NO_MODULE; no binding is invented.
 */
static hwaddr hpb_module_addr(ClarionHpbDmaState *s, int ch)
{
    if (s->qy7_mode && ch == 1) {
        /* QY7221NL serial_scif.dll configures SCI1 RX on channel 1. */
        return 0xffe41014;
    }
    if (s->qy7_mode && ch == 10) {
        /* QY7221NL serial_scif.dll configures SCI1 TX on channel 10. */
        return 0xffe4100c;
    }
    /* QY7 SDHI0 uses HPB channel 8 in WinCE and channel 9 in EH12. */
    if (s->qy7_mode && (ch == 8 || ch == 9)) {
        return 0xffe4c030;
    }
    return hpb_chan_module[ch];
}

/* Transfer unit size from SPDS/DPDS: 0=8 bits, 1=16, 2=32. */
static unsigned hpb_unit(ClarionHpbDmaState *s, int ch)
{
    uint32_t dcr = s->ch[ch].dcr;
    unsigned spds = (dcr & DCR_SPDS_MASK) >> DCR_SPDS_SHIFT;
    unsigned dpds = (dcr & DCR_DPDS_MASK) >> DCR_DPDS_SHIFT;

    if (spds != dpds && !(s->port_widths_logged & BIT_ULL(ch))) {
        s->port_widths_logged |= BIT_ULL(ch);
        qemu_log_mask(LOG_UNIMP,
                      "clarion-hpbdma: channel %d has different port widths "
                      "(SPDS=%u, DPDS=%u); using the larger\n", ch, spds, dpds);
    }
    return 1u << MAX(spds > 2 ? 2 : spds, dpds > 2 ? 2 : dpds);
}

static hwaddr hpb_dma_addr(ClarionHpbDmaState *s, hwaddr addr)
{
    if (s->qy7_mode && addr >= 0xa0000000 && addr < 0xc0000000) {
        return addr & 0x1fffffff;
    }
    return addr;
}

/*
 * Transfer one unit. The side marked MDL is the peripheral module, whose
 * address does not advance; the other side is incremented.
 */
static void hpb_move_one(ClarionHpbDmaState *s, int ch)
{
    ClarionHpbChan *c = &s->ch[ch];
    unsigned unit = hpb_unit(s, ch);
    hwaddr mod = hpb_module_addr(s, ch);
    hwaddr src = c->dsasr, dst = c->ddasr;
    bool sd_burst = s->qy7_mode && (ch == 8 || ch == 9) &&
                    (c->dcr & BIT(21)) &&
                    (c->dcr & DCR_SMDL) && !(c->dcr & DCR_DMDL);
    bool qy7_scif1_tx = s->qy7_mode && ch == 10;
    unsigned bytes = sd_burst ? 16 : unit;
    uint8_t buf[16];

    /*
     * The MDL side uses the channel binding. Ignore the guest-written value
     * on that side; it need not be an address (see hpb_chan_module comment).
     */
    if ((c->dcr & DCR_SMDL) && mod != HPB_NO_MODULE) {
        src = mod;
    }
    if ((c->dcr & DCR_DMDL) && mod != HPB_NO_MODULE) {
        dst = mod;
    }
    /* QY7 channel 10 targets SCIF1 SCFTDR with DMDL clear. */
    if (qy7_scif1_tx) {
        dst = mod;
    }
    if ((c->dcr & (DCR_SMDL | DCR_DMDL)) && mod == HPB_NO_MODULE &&
        !(s->no_binding_logged & BIT_ULL(ch))) {
        s->no_binding_logged |= BIT_ULL(ch);
        qemu_log_mask(LOG_UNIMP, "clarion-hpbdma: channel %d accesses a module "
                      "with no known binding; using register address\n", ch);
    }

    /* Burst mode counts 16 bytes while the SD FIFO port stays 16-bit. */
    for (unsigned offset = 0; offset < bytes; offset += unit) {
        hwaddr read_addr = src + ((c->dcr & DCR_SMDL) ? 0 : offset);

        address_space_read(&address_space_memory, hpb_dma_addr(s, read_addr),
                           MEMTXATTRS_UNSPECIFIED, buf + offset, unit);
    }
    address_space_write(&address_space_memory, hpb_dma_addr(s, dst),
                        MEMTXATTRS_UNSPECIFIED, buf, bytes);

    if (!(c->dcr & DCR_SMDL)) {
        c->dsasr += bytes;
    }
    if (!(c->dcr & DCR_DMDL) && !qy7_scif1_tx) {
        c->ddasr += bytes;
    }
    c->left--;
    c->dtcsr = c->left;
}

static void hpb_chan_load_plane(ClarionHpbDmaState *s, int ch, unsigned plane)
{
    ClarionHpbChan *c = &s->ch[ch];

    c->plane = plane;
    c->dsasr = c->sar[plane];
    c->ddasr = c->dar[plane];
    c->left = c->tcr[plane];
    c->dtcsr = c->left;
}

/*
 * Transfer count completed. Shared completion path: raise interrupt, then
 * start the next plane (CT, if software has set DNXT) or deactivate channel.
 */
static void hpb_chan_finish(ClarionHpbDmaState *s, int ch)
{
    ClarionHpbChan *c = &s->ch[ch];

    hpb_chan_complete(s, ch);
    if (c->dcr & DCR_CT) {
        /*
         * CT does not unconditionally auto-switch. Start the next set only
         * if software has already set DNXT; otherwise remain active in the
         * command-wait state.
         */
        if (c->next_requested) {
            c->next_requested = false;
            hpb_chan_load_plane(s, ch, (c->dcr & DCR_DIP) ? c->plane ^ 1 : 0);
        }
    } else {
        c->active = false;
        c->dstsr &= ~DSTSR_DMSTS;
    }
}

static uint32_t hpb_chan_status(const ClarionHpbChan *c)
{
    uint32_t status = c->dstsr & DSTSR_DQSTS;
    unsigned next_plane = (c->dcr & DCR_DIP) && c->active ? c->plane ^ 1 : 0;

    status |= next_plane ? DSTSR_NDP1 : DSTSR_NDP0;
    if (c->next_requested) {
        status |= DSTSR_DRSTS;
    }
    if (c->active) {
        status |= DSTSR_DMSTS;
    }
    return status;
}

/* TEMPORARY: virtual-address probe enabled by QY8_DMA_VA_PROBE. */
static bool hpb_va_probe(void)
{
    static int on = -1;
    if (on < 0) {
        on = getenv("QY8_DMA_VA_PROBE") != NULL;
    }
    return on;
}

/* TEMPORARY: channel log enabled by QY8_DMA_LOG. */
static bool hpb_dbg(void)
{
    static int on = -1;
    if (on < 0) {
        on = getenv("QY8_DMA_LOG") != NULL;
    }
    return on;
}

/*
 * TEMPORARY (QY8_DMA_LOG; remove with the other probes).
 *
 * Show what reached GUEST memory at the destination address. Read it back
 * from memory instead of echoing the just-written data, or the probe would
 * only confirm itself. These lines established the first SD block read.
 */
static void hpb_dbg_dest(ClarionHpbDmaState *s, int ch, hwaddr addr,
                         uint32_t dar,
                         uint32_t moved, uint32_t asked)
{
    uint8_t back[16];
    unsigned k;

    if (!hpb_dbg()) {
        return;
    }
    address_space_read(&address_space_memory, hpb_dma_addr(s, dar),
                       MEMTXATTRS_UNSPECIFIED, back, sizeof(back));
    fprintf(stderr, "[dma] ch%d module %08x -> %08x: moved %u units of %u, "
            "memory:", ch, (unsigned)addr, dar, moved, asked);
    for (k = 0; k < sizeof(back); k++) {
        fprintf(stderr, " %02x", back[k]);
    }
    fprintf(stderr, "\n");
}

/* --- module with its own data buffer (SDHI) ---------------------------- */

static ClarionHpbModule *hpb_module_find(ClarionHpbDmaState *s, hwaddr addr)
{
    unsigned i;

    for (i = 0; i < s->n_module; i++) {
        if (s->module[i].addr == addr) {
            return &s->module[i];
        }
    }
    return NULL;
}

/*
 * Transfer to memory all data that the module at addr can provide now. Data
 * comes from reading its register (hpb_move_one), so the model invents none:
 * it transfers exactly what the module returns.
 *
 * Check "is more data ready?" before every transfer unit. A module may refill
 * its buffer during this read (SDHI does this for multi-block reads), and the
 * loop will pick up the new data.
 */
static void hpb_service_module(ClarionHpbDmaState *s, hwaddr addr)
{
    ClarionHpbModule *m = hpb_module_find(s, addr);
    int ch;

    if (!m || s->servicing) {
        return;
    }
    s->servicing = true;

    for (ch = 0; ch < CLARION_HPBDMA_NUM_CHAN; ch++) {
        ClarionHpbChan *c = &s->ch[ch];
        uint32_t dar0, n0;

        bool module_to_memory = (c->dcr & DCR_SMDL) &&
                                !(c->dcr & DCR_DMDL);
        bool memory_to_module = !(c->dcr & DCR_SMDL) &&
                                (c->dcr & DCR_DMDL);

        if (!c->active || !c->left || hpb_module_addr(s, ch) != addr ||
            (!module_to_memory && !memory_to_module)) {
            continue;
        }

        dar0 = c->ddasr;
        n0 = c->left;
        trace_clarion_hpbdma_module_service(ch, (uint32_t)addr, c->dsasr,
                                            c->ddasr, c->left);
        while (c->left && m->ready(m->opaque)) {
            hpb_move_one(s, ch);
        }
        if (!c->left) {
            hpb_chan_finish(s, ch);
        }

        hpb_dbg_dest(s, ch, addr, dar0, n0 - c->left, n0);
        break;                          /* one channel per address */
    }

    s->servicing = false;
}

void clarion_hpbdma_module_attach(DeviceState *dev, hwaddr periph_addr,
                                  ClarionHpbModuleReady ready, void *opaque)
{
    ClarionHpbDmaState *s = CLARION_HPBDMA(dev);

    if (s->n_module >= HPB_MAX_MODULE) {
        qemu_log_mask(LOG_UNIMP, "clarion-hpbdma: more than %d buffered "
                      "modules are not supported\n", HPB_MAX_MODULE);
        return;
    }
    s->module[s->n_module].addr = periph_addr;
    s->module[s->n_module].ready = ready;
    s->module[s->n_module].opaque = opaque;
    s->n_module++;
}

void clarion_hpbdma_module_poke(DeviceState *dev, hwaddr periph_addr)
{
    hpb_service_module(CLARION_HPBDMA(dev), periph_addr);
}

static void hpb_chan_start(ClarionHpbDmaState *s, int ch, bool next)
{
    ClarionHpbChan *c = &s->ch[ch];
    hwaddr mod = hpb_module_addr(s, ch);
    unsigned plane = 0;

    /* DMEN activates an idle channel; repeated DMEN does not restart it. */
    if (c->active) {
        return;
    }
    if ((c->dcr & DCR_DIP) && next) {
        plane = c->plane ^ 1;
    }
    hpb_chan_load_plane(s, ch, plane);
    c->active = true;
    trace_clarion_hpbdma_start(ch, plane, c->dcr, c->dsasr, c->ddasr, c->left);
    if (hpb_dbg() && (ch == 4 || ch == 6)) {
        fprintf(stderr, "[dma] start ch%d plane%u dcr=%08x dar=%08x tcr=%u\n",
                ch, plane, c->dcr, c->ddasr, c->left);
    }

    if ((c->dcr & DCR_SMDL) && !(c->dcr & DCR_DMDL)) {
        /*
         * Module to memory. Do not invent data. A buffered module (SDHI) may
         * have data ready before DMEN, so query it immediately. For other
         * modules (SCIF), leave the channel armed; the peripheral supplies
         * bytes through clarion_hpbdma_feed().
         */
        hpb_service_module(s, hpb_module_addr(s, ch));
        return;
    }
    if ((c->dcr & DCR_SMDL) && (c->dcr & DCR_DMDL)) {
        qemu_log_mask(LOG_UNIMP, "clarion-hpbdma: channel %d module-to-module "
                      "transfer has no source\n", ch);
        return;
    }

    if ((c->dcr & DCR_DMDL) && hpb_module_find(s, mod)) {
        /* Wait for the peripheral to enter its receive-data state. */
        hpb_service_module(s, mod);
        return;
    }

    /* Memory -> module (or memory -> memory) with no module handshake. */
    while (c->left) {
        hpb_move_one(s, ch);
    }
    c->active = false;
    c->dstsr &= ~DSTSR_DMSTS;
    hpb_chan_complete(s, ch);
}

/*
 * A byte arrived at peripheral register periph_addr. If an armed channel is
 * waiting, store it in memory and raise an interrupt when the count completes.
 * Return true if DMA consumed the byte.
 */
bool clarion_hpbdma_feed(DeviceState *dev, hwaddr periph_addr, uint8_t val)
{
    ClarionHpbDmaState *s = CLARION_HPBDMA(dev);
    int ch;

    for (ch = 0; ch < CLARION_HPBDMA_NUM_CHAN; ch++) {
        ClarionHpbChan *c = &s->ch[ch];

        /*
         * Find the channel by BINDING, not guest-written DSAR. Keep the old
         * fallback for channels without a known binding; otherwise we could
         * silently stop handling uncharacterized SCIF-like cases.
         */
        hwaddr mod = hpb_module_addr(s, ch);

        if (!c->active || !c->left || !(c->dcr & DCR_SMDL)) {
            continue;
        }
        if (mod != HPB_NO_MODULE ? mod != periph_addr
                                 : c->dsasr != periph_addr) {
            continue;
        }
        address_space_write(&address_space_memory,
                            hpb_dma_addr(s, c->ddasr),
                            MEMTXATTRS_UNSPECIFIED, &val, 1);
        c->ddasr++;
        c->left--;
        c->dtcsr = c->left;

        if (!c->left) {
            if (hpb_dbg()) {
                fprintf(stderr, "[dma] ch%d buffer complete (plane%u)\n",
                        ch, c->plane);
            }
            hpb_chan_finish(s, ch);
        }
        return true;
    }
    return false;
}

/*
 * The module receiver became idle. End the active set for any channel that
 * transferred data in this burst. DTCSR retains the remaining count, so the
 * driver sees exactly the number of bytes that arrived.
 *
 * Why the model needs this: SCIF delivers received data through HPB-DMAC,
 * not interrupts. The guest keeps RIE = 0 on all SCIFs (seen at boot:
 * SCSCR 0x003E/0x00BE), so RXI is not used. The driver learns of data only
 * from the DMA channel interrupt. Its buffer is 128 bytes while link frames
 * are short (the panel response is 20 bytes), so without early completion the
 * set would never finish and the frame would go unnoticed. Hardware uses the
 * SCIF receiver-empty (DR) signal for this purpose.
 */
void clarion_hpbdma_eod(DeviceState *dev, hwaddr periph_addr)
{
    ClarionHpbDmaState *s = CLARION_HPBDMA(dev);
    int ch;

    for (ch = 0; ch < CLARION_HPBDMA_NUM_CHAN; ch++) {
        ClarionHpbChan *c = &s->ch[ch];
        hwaddr mod = hpb_module_addr(s, ch);

        if (!c->active || !(c->dcr & DCR_SMDL)) {
            continue;
        }
        if (mod != HPB_NO_MODULE ? mod != periph_addr
                                 : c->dsasr != periph_addr) {
            continue;
        }
        /* No empty set to finish; do not interrupt without data. */
        if (!c->left || c->left == c->tcr[c->plane]) {
            continue;
        }
        c->dtcsr = c->left;
        hpb_chan_finish(s, ch);
        return;                         /* one channel per address */
    }
}

/* --- channel registers ------------------------------------------------- */

static uint64_t hpb_chan_read(void *opaque, hwaddr addr, unsigned size)
{
    ClarionHpbDmaState *s = opaque;
    if (s->qy7_mode && addr == 0x80C) {
        return s->dintsr[0];
    }
    if (s->qy7_mode && addr == HPB_QY7_DTE0) {
        return s->qy7_dte[0];
    }
    if (s->qy7_mode && addr == HPB_QY7_DTE1) {
        return s->qy7_dte[1];
    }
    int ch = addr / HPB_CHAN_STRIDE;
    hwaddr off = addr % HPB_CHAN_STRIDE;

    if (ch >= (s->qy7_mode ? 32 : CLARION_HPBDMA_NUM_CHAN)) {
        return 0;
    }

    if (hpb_dbg() && ch == 4 &&
        (off == HPB_DDASR || off == HPB_DTCSR || off == HPB_DSTSR)) {
        fprintf(stderr, "[dma] READ ch4 +%02x "
                "(plane%u dstsr=%08x ddasr=%08x left=%u)\n",
                (unsigned)off, s->ch[ch].plane, hpb_chan_status(&s->ch[ch]),
                s->ch[ch].ddasr, s->ch[ch].left);
    }
    switch (off) {
    case HPB_DSAR0: return s->ch[ch].sar[0];
    case HPB_DDAR0: return s->ch[ch].dar[0];
    case HPB_DTCR0: return s->ch[ch].tcr[0];
    case HPB_DSAR1: return s->ch[ch].sar[1];
    case HPB_DDAR1: return s->ch[ch].dar[1];
    case HPB_DTCR1: return s->ch[ch].tcr[1];
    case HPB_DSASR: return s->ch[ch].dsasr;
    case HPB_DDASR: return s->ch[ch].ddasr;
    case HPB_DTCSR: return s->ch[ch].dtcsr;
    case HPB_DPTR:  return s->ch[ch].dptr;
    case HPB_DCR:   return s->ch[ch].dcr;
    case HPB_DCMDR: return 0;           /* command register reads as zero */
    case HPB_DSTPR: return 0;
    case HPB_DSTSR: return hpb_chan_status(&s->ch[ch]);
    default:        return 0;
    }
}

static void hpb_chan_write(void *opaque, hwaddr addr, uint64_t val,
                           unsigned size)
{
    ClarionHpbDmaState *s = opaque;
    if (s->qy7_mode && addr == HPB_QY7_DTE0) {
        s->dintsr[0] &= ~(uint32_t)val;
        hpb_update_irq(s);
        s->qy7_dte[0] = val;
        return;
    }
    if (s->qy7_mode && addr == HPB_QY7_DTE1) {
        s->qy7_dte[1] = val;
        hpb_update_irq(s);
        return;
    }
    int ch = addr / HPB_CHAN_STRIDE;
    hwaddr off = addr % HPB_CHAN_STRIDE;

    if (ch >= (s->qy7_mode ? 32 : CLARION_HPBDMA_NUM_CHAN)) {
        return;
    }

    trace_clarion_hpbdma_chan_write(ch, (uint32_t)off, val);

    /*
     * TEMPORARY (QY8_DMA_VA_PROBE; remove after the 2026-09-24 experiment).
     *
     * Experiment question: why do channel 21 (SDHI0) DSAR/DDAR values fall
     * outside the board's physical map while channels 4/6/16/17 use valid
     * physical addresses? Translate the written value as a VIRTUAL address
     * in the context of the same CPU. A meaningful VA -> PA result means the
     * driver wrote a virtual address rather than a physical one.
     */
    if (hpb_va_probe() && (off == HPB_DSAR0 || off == HPB_DDAR0 ||
                           off == HPB_DSAR1 || off == HPB_DDAR1)) {
        CPUState *cs = current_cpu;
        TranslateForDebugResult tres;

        if (cs && cpu_translate_for_debug(cs, val, &tres)) {
            fprintf(stderr, "[va] ch%-2d +0x%02x <- %08" PRIx64
                    "   as VA -> PA %08" HWADDR_PRIx "\n",
                    ch, (unsigned)off, val, tres.physaddr);
        } else {
            fprintf(stderr, "[va] ch%-2d +0x%02x <- %08" PRIx64
                    "   VA does not translate\n", ch, (unsigned)off, val);
        }
    }

    switch (off) {
    case HPB_DSAR0: s->ch[ch].sar[0] = val; break;
    case HPB_DDAR0: s->ch[ch].dar[0] = val; break;
    case HPB_DTCR0: s->ch[ch].tcr[0] = val; break;
    case HPB_DSAR1: s->ch[ch].sar[1] = val; break;
    case HPB_DDAR1: s->ch[ch].dar[1] = val; break;
    case HPB_DTCR1: s->ch[ch].tcr[1] = val; break;
    case HPB_DPTR:  s->ch[ch].dptr = val; break;   /* store, do not interpret */
    case HPB_DCR:   s->ch[ch].dcr = val; break;

    case HPB_DCMDR:
        if (hpb_dbg() && (ch == 4 || ch == 6)) {
            fprintf(stderr, "[dma] DCMDR ch%d <- %08" PRIx64
                    " (active=%d plane%u left=%u)\n",
                    ch, val, s->ch[ch].active, s->ch[ch].plane,
                    s->ch[ch].left);
        }
        if (val & DCMDR_DMEN) {
            hpb_chan_start(s, ch, !!(val & DCMDR_DNXT));
        }
        if ((val & DCMDR_DNXT) && (s->ch[ch].dcr & DCR_CT)) {
            ClarionHpbChan *c = &s->ch[ch];

            c->next_requested = true;
            if (c->active && !c->left) {
                c->next_requested = false;
                hpb_chan_load_plane(s, ch,
                                    (c->dcr & DCR_DIP) ? c->plane ^ 1 : 0);
            }
        }
        if (val & (DCMDR_DQEND | DCMDR_DQSPD | DCMDR_DMSPD)) {
            s->ch[ch].active = false;
            s->ch[ch].next_requested = false;
        }
        break;

    case HPB_DSTPR:
        if (val & DSTPR_DMSTP) {
            s->ch[ch].active = false;
            s->ch[ch].next_requested = false;
        }
        break;

    default:
        break;
    }
}

static const MemoryRegionOps hpb_chan_ops = {
    .read = hpb_chan_read,
    .write = hpb_chan_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* --- shared registers -------------------------------------------------- */

static uint64_t hpb_comm_read(void *opaque, hwaddr addr, unsigned size)
{
    ClarionHpbDmaState *s = opaque;
    uint32_t act[2] = { 0, 0 };
    int ch;

    switch (addr) {
    case HPB_DTIMR:   return s->dtimr;
    case HPB_DINTSR0: return s->dintsr[0];
    case HPB_DINTSR1: return s->dintsr[1];
    case HPB_DINTMR0: return s->dintmr[0];
    case HPB_DINTMR1: return s->dintmr[1];
    case HPB_DACTSR0:
    case HPB_DACTSR1:
        for (ch = 0; ch < CLARION_HPBDMA_NUM_CHAN; ch++) {
            if (s->ch[ch].active) {
                act[ch / 32] |= 1u << (ch % 32);
            }
        }
        return addr == HPB_DACTSR0 ? act[0] : act[1];
    default:
        return 0;
    }
}

static void hpb_comm_write(void *opaque, hwaddr addr, uint64_t val,
                           unsigned size)
{
    ClarionHpbDmaState *s = opaque;
    int ch;

    switch (addr) {
    case HPB_DTIMR:
        s->dtimr = val;
        break;
    case HPB_DINTCR0:                   /* writing 1 clears status */
        s->dintsr[0] &= ~(uint32_t)val;
        hpb_update_irq(s);
        break;
    case HPB_DINTCR1:
        s->dintsr[1] &= ~(uint32_t)val;
        hpb_update_irq(s);
        break;
    case HPB_DINTMR0:
        s->dintmr[0] = val;
        hpb_update_irq(s);
        break;
    case HPB_DINTMR1:
        s->dintmr[1] = val;
        hpb_update_irq(s);
        break;
    default:
        /* HSRSTR(n): reset channel n (hpb_dmae_reset) */
        if (addr >= HPB_HSRSTR(0) &&
            addr < HPB_HSRSTR(CLARION_HPBDMA_NUM_CHAN) && (val & 1)) {
            ch = (addr - HPB_HSRSTR(0)) / 4;
            memset(&s->ch[ch], 0, sizeof(s->ch[ch]));
            s->dintsr[ch / 32] &= ~(1u << (ch % 32));
            hpb_update_irq(s);
        }
        break;
    }
}

static const MemoryRegionOps hpb_comm_ops = {
    .read = hpb_comm_read,
    .write = hpb_comm_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* --- device ------------------------------------------------------------ */

static void clarion_hpbdma_reset_hold(Object *obj, ResetType type)
{
    ClarionHpbDmaState *s = CLARION_HPBDMA(obj);

    memset(s->ch, 0, sizeof(s->ch));
    s->dintsr[0] = s->dintsr[1] = 0;
    s->dintmr[0] = s->dintmr[1] = 0;
    s->dtimr = 0;
    s->qy7_dte[0] = s->qy7_dte[1] = 0;
    hpb_update_irq(s);
}

static void clarion_hpbdma_realize(DeviceState *dev, Error **errp)
{
    ClarionHpbDmaState *s = CLARION_HPBDMA(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    int i;

    memory_region_init_io(&s->chan_mr, OBJECT(dev), &hpb_chan_ops, s,
                          "clarion-hpbdma.chan", CLARION_HPBDMA_CHAN_SIZE);
    memory_region_init_io(&s->comm_mr, OBJECT(dev), &hpb_comm_ops, s,
                          "clarion-hpbdma.comm", CLARION_HPBDMA_COMM_SIZE);
    sysbus_init_mmio(sbd, &s->chan_mr);
    sysbus_init_mmio(sbd, &s->comm_mr);

    for (i = 0; i < CLARION_HPBDMA_NUM_IRQ; i++) {
        sysbus_init_irq(sbd, &s->irq[i]);
    }
}

static void clarion_hpbdma_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    static const Property props[] = {
        DEFINE_PROP_BOOL("qy7-mode", ClarionHpbDmaState, qy7_mode, false),
    };

    dc->realize = clarion_hpbdma_realize;
    dc->desc = "Renesas HPB-DMAC (R-Car Gen1)";
    device_class_set_props(dc, props);
    rc->phases.hold = clarion_hpbdma_reset_hold;
}

static const TypeInfo clarion_hpbdma_types[] = {
    {
        .name          = TYPE_CLARION_HPBDMA,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(ClarionHpbDmaState),
        .class_init    = clarion_hpbdma_class_init,
    },
};

DEFINE_TYPES(clarion_hpbdma_types)
