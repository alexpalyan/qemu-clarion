/*
 * Renesas CAN controller (R-Car Gen1, R8A7778) — QY8XXX @ 0xFFFD1000.
 *
 * This is a CONTROLLER, not a vehicle. The model implements its modes
 * (reset / halt / operation / sleep) and mailbox memory only. There is no CAN
 * bus here: transmission and reception are not implemented, and the model
 * does not invent substitutes.
 *
 * --- Register map sources ------------------------------------------------
 *
 * 1. Live QEMU trace (2026-09-24): 64 groups of 16 bytes at +0x000, 64
 *    single-byte registers at +0x800, followed by separate registers.
 *    +0x840, +0x842, +0x844/846, +0x84C, +0x856, +0x858.
 * 2. Driver `CAN.dll` (section 0, VA 0xEF621000) provides the semantics;
 *    no public datasheet for this block is available.
 * 3. Register names come from Linux driver's `struct rcar_can_regs` in
 *    `drivers/net/can/rcar/rcar_can.c`; its map matches (1) byte for byte:
 *    byte for byte: mb[64]×16, mkr/fidcr/mier at +0x400, mctl[64] at +0x800,
 *    CTLR +0x840, STR +0x842, BCR +0x844..846.
 *
 * --- Established driver behavior -----------------------------------------
 *
 *   0xEF628E80  wake from sleep: if CTLR & 0x400, clear the bit and wait for
 *               STR & 0x400 to clear.
 *   0xEF629308  initialization:
 *                 CTLR = (CTLR & ~0x300) | 0x100      - request reset mode
 *                 poll STR & 0x100 up to 100 times   - else error 0x21
 *                 clear 64 mailboxes, MCTL[i] = 0x40
 *                 BCR: 0xF004 -> +0x844, 0x12 -> +0x846 (bit timing)
 *                 CTLR = (CTLR & ~0x300) | 0x200      - request halt mode
 *                 poll STR & 0x200 up to 100 times   - else error 0x21
 *                 +0x858 = (+0x858 & 0x20) | 0x07, +0x84C = 0x08
 *                 CTLR &= ~0x300                      - request operation mode
 *                 wait up to 100 times for both STR 0x100 and 0x200 to clear
 *   0xEF6294BC  mailbox memory test: write 0x5555 and 0xAAAA to all 64
 *               mailboxes at offsets 0x0, 0x2, 0x6, 0x8, 0xA, 0xC, then
 *               verify masked reads (table at 0xEF6216AC). First halfword
 *               mask is 0xDFFF; the rest use 0xFFFF.
 *
 * --- Model scope ---------------------------------------------------------
 *
 * * The CTLR reset value is not confirmed by a datasheet. Use 0x0100, reset
 *   mode after power-on as expected for a CAN controller. The driver forces
 *   CANM itself before waiting for confirmation.
 * * Real hardware enters operation mode after 11 recessive bus bits. No
 *   receiver is connected, so the bus is idle and those bits elapse
 *   immediately. This models an idle bus rather than stubbing the transition.
 * * Error counters (RECR/TECR/ECSR) read zero because there is no bus and no
 *   errors. MSSR returns SEST=1 (no mailbox found) because there is nothing to
 *   receive.
 * * Bit 13 of the first mailbox halfword reads zero; it is the only bit the
 *   driver excludes from its memory test (mask 0xDFFF). In the Linux map this
 *   corresponds to reserved bit 29 of the 32-bit ID field.
 * * Receive is the only interrupt: ISR (+0x861) bit 0x02 while a mailbox has
 *   RECREQ|NEWDATA and is enabled in MIER0/1; the line is ISR & IER (+0x860).
 *   The GIC number comes from the OAL table, see clarion_qy8.c (QY8_CAN_SPI).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <stdarg.h>
#include "qemu/log.h"
#include "qemu/cutils.h"
#include "qemu/timer.h"
#include "accel/tcg/cpu-loop.h"
#include "exec/target_page.h"
#include "hw/core/cpu.h"
#include "qemu/module.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/net/renesas_can.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "trace.h"

OBJECT_DECLARE_SIMPLE_TYPE(RenesasCANState, RENESAS_CAN)

/* --- block map ---------------------------------------------------------- */

#define CAN_MB_BASE     0x000   /* 64 mailboxes, 16 bytes each            */
#define CAN_MB_SIZE     0x400
#define CAN_MKR_BASE    0x400   /* MKR2..9, FIDCR0/1, MKIVLR*, MIER*      */
#define CAN_MKR_SIZE    0x040
#define CAN_CTL_BASE    0x800   /* MCTL[64] and control registers         */
#define CAN_CTL_SIZE    0x100

#define CAN_CTLR        0x840   /* Control Register, 16 bits              */
#define CAN_STR         0x842   /* Status Register, 16 bits, read-only    */

/* CTLR */
#define CTLR_CANM       0x0300  /* controller mode                        */
#define CTLR_CANM_OPER  0x0000
#define CTLR_CANM_RESET 0x0100
#define CTLR_CANM_HALT  0x0200
#define CTLR_CANM_FRST  0x0300  /* force reset                           */
#define CTLR_SLPM       0x0400  /* sleep request                           */

/* STR */
#define STR_RSTST       0x0100
#define STR_HLTST       0x0200
#define STR_SLPST       0x0400

/* Registers with behavior beyond storage (offsets from CAN_CTL_BASE). */
#define CTL_OFF(x)      ((x) - CAN_CTL_BASE)
#define CTL_MSSR        CTL_OFF(0x852)
#define CTL_IER         CTL_OFF(0x860)
#define CTL_ISR         CTL_OFF(0x861)
/* CAN.dll scans mailbox MCTL state; no ISR read appears in that path. */
#define ISR_RX          0x02
#define MKR_MIER1       0x2c    /* interrupt enable, mailboxes 32..63     */
#define MKR_MIER0       0x3c    /* mailboxes 0..31                        */
#define MSSR_SEST       0x80    /* 1 = no search result                   */

/*
 * QY8652NB CAN.dll observations: initialization writes 0x40 to each MCTL
 * (0xEF629308); the receive scan tests bit 0 (0xEF62965C), and the mailbox
 * reader clears it after consuming the frame (0xEF6291C8). A helper at
 * 0xEF62960C also tests bit 0 for mailbox 63 completion, gated by bit 1.
 * Bit 2 is independently tested and cleared at 0xEF629194. The names below
 * follow the Renesas register convention where the guest writes the request
 * bits; request-bit semantics beyond these observations remain inferred.
 */
#define MCTL_TRMREQ     0x80    /* I: transmit request (set by guest)   */
#define MCTL_RECREQ     0x40    /* I: mailbox set up for receive        */
#define MCTL_DATA       0x01    /* I: "data ready": TX done and RX new  */

enum {
    CAN_MODE_RESET,
    CAN_MODE_HALT,
    CAN_MODE_OPERATION,
};

/* one replay entry: a frame repeated every period_ms */
typedef struct CanRxReplay {
    QEMUTimer *timer;
    struct RenesasCANState *s;
    uint32_t period_ms;
    uint32_t id;
    bool ext;
    uint8_t dlc;
    uint8_t data[8];
} CanRxReplay;

struct RenesasCANState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;

    uint8_t mb[CAN_MB_SIZE];    /* mailbox memory                         */
    uint8_t mkr[CAN_MKR_SIZE];  /* masks/filters/interrupt enables        */
    uint8_t ctl[CAN_CTL_SIZE];  /* MCTL[64] + control registers            */

    uint8_t mode;               /* CAN_MODE_*                            */
    bool sleep;

    /* QY8_CAN_PC=1 adds caller pc/lr to each trace line. */
    bool trace_pc;

    /* optional file receiving frame and mode/filter records */
    FILE *canlog;

    /* periodic receive frames loaded from can-replay */
    CanRxReplay *replay;
    int n_replay;
    char *replay_path;
};

static void can_log(RenesasCANState *s, const char *fmt, ...)
    G_GNUC_PRINTF(2, 3);

static const char *can_mode_name(uint8_t mode)
{
    switch (mode) {
    case CAN_MODE_RESET:     return "reset";
    case CAN_MODE_HALT:      return "halt";
    case CAN_MODE_OPERATION: return "operation";
    default:                 return "?";
    }
}

/*
 * Derive STR from controller state instead of storing it, avoiding a fake
 * "read back the expected value" implementation.
 */
static uint16_t can_str(RenesasCANState *s)
{
    uint16_t str = 0;

    switch (s->mode) {
    case CAN_MODE_RESET:
        str |= STR_RSTST;
        break;
    case CAN_MODE_HALT:
        str |= STR_HLTST;
        break;
    default:
        break;
    }
    if (s->sleep) {
        str |= STR_SLPST;
    }
    return str;
}

static void can_ctlr_write(RenesasCANState *s, uint16_t ctlr)
{
    uint8_t old = s->mode;
    bool oldsleep = s->sleep;

    switch (ctlr & CTLR_CANM) {
    case CTLR_CANM_RESET:
    case CTLR_CANM_FRST:
        s->mode = CAN_MODE_RESET;
        break;
    case CTLR_CANM_HALT:
        s->mode = CAN_MODE_HALT;
        break;
    default:
        /*
         * Request operation mode. Hardware joins the bus after observing 11
         * recessive bits; the bus is idle here, so this happens immediately.
         */
        s->mode = CAN_MODE_OPERATION;
        break;
    }
    s->sleep = (ctlr & CTLR_SLPM) != 0;

    if (s->mode != old || s->sleep != oldsleep) {
        trace_renesas_can_mode(can_mode_name(old), can_mode_name(s->mode),
                               s->sleep, can_str(s));
    can_log(s, "event=mode old=%s new=%s sleep=%d", can_mode_name(old),
               can_mode_name(s->mode), s->sleep);
    }
}

/* --- access byte arrays at any width ----------------------------------- */

static uint64_t can_ld(const uint8_t *p, unsigned size)
{
    uint64_t v = 0;
    unsigned i;

    for (i = 0; i < size; i++) {
        v |= (uint64_t)p[i] << (8 * i);
    }
    return v;
}

static void can_st(uint8_t *p, unsigned size, uint64_t v)
{
    unsigned i;

    for (i = 0; i < size; i++) {
        p[i] = v >> (8 * i);
    }
}

/*
 * Identify the caller. As in qy8.periph, cpu_unwind_state_data provides the
 * exact PC (not cpu_restore_state, which writes unwound state back to env and
 * changes the guest's execution path). Only used with QY8_CAN_PC=1 because
 * unwinding state is expensive.
 */
static void can_ctx(RenesasCANState *s, char *buf, size_t len)
{
    CPUState *cs = current_cpu;
    uint64_t data[4];
    uint64_t pc;

    buf[0] = '\0';
    if (!s->trace_pc || !cs) {
        return;
    }
    if (cpu_unwind_state_data(cs, cs->mem_io_pc, data)) {
        /*
         * With CF_PCREL, data[0] is an offset within the page; use high bits
         * from the current PC because a translation block does not cross a
         * page boundary.
         */
        pc = data[0];
        if (pc < qemu_target_page_size()) {
            pc |= CPU_GET_CLASS(cs)->get_pc(cs) & qemu_target_page_mask();
        }
    } else {
        pc = CPU_GET_CLASS(cs)->get_pc(cs);   /* fallback: start of block */
    }
    snprintf(buf, len, " pc=0x%08" PRIx64 " t=%" PRId64, pc,
             qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL));
}

static void can_log(RenesasCANState *s, const char *fmt, ...)
{
    va_list ap;

    if (!s->canlog) {
        return;
    }
    fprintf(s->canlog, "guest_ms=%" PRId64 " ",
            qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL));
    va_start(ap, fmt);
    vfprintf(s->canlog, fmt, ap);
    va_end(ap);
    fputc('\n', s->canlog);
    fflush(s->canlog);
}

/*
 * Mailbox layout (16 bytes), from the guest's self-test mask (0xEF6294BC
 * above) and from what the driver writes itself: halfword +0x0 =
 * IDE(15) RTR(14) 0(13) SID(12..2) EID17..16(1..0), +0x2 = EID15..0,
 * +0x4 DLC, +0x6..+0xD data. Measured on ZE0 062: mailboxes 0..2 hold
 * 0x1a04/0x1d1c/0x1ab0, i.e. 0x681/0x747/0x6AC, and the driver compares IDs
 * as (h0 & 0x1ffc) == id << 2 (CAN.dll 0xEF6585FC).
 */
static void can_frame_decode(const uint8_t *mb, uint32_t *id, bool *ext,
                             uint8_t *dlc, uint8_t data[8])
{
    uint16_t w0 = can_ld(mb, 2), w1 = can_ld(mb + 2, 2);

    *ext = (w0 & 0x8000) != 0;
    *id = (w0 >> 2) & 0x7ff;
    if (*ext) {
        *id = (*id << 18) | ((w0 & 3) << 16) | w1;
    }
    *dlc = mb[4] & 0xf;
    memcpy(data, mb + 6, 8);
}

static void can_frame_encode(uint8_t *mb, uint32_t id, bool ext, uint8_t dlc,
                             const uint8_t *data)
{
    if (ext) {
        can_st(mb, 2, 0x8000 | ((id >> 18) & 0x7ff) << 2 | ((id >> 16) & 3));
        can_st(mb + 2, 2, id & 0xffff);
    } else {
        can_st(mb, 2, (id & 0x7ff) << 2);
    }
    mb[4] = dlc & 0xf;
    memcpy(mb + 6, data, 8);
}

static void can_log_frame(RenesasCANState *s, const char *tag, int mbidx,
                          uint32_t id, bool ext, uint8_t dlc,
                          const uint8_t *data)
{
    char hex[2 * 8 + 1];
    int i;

    for (i = 0; i < 8 && i < dlc; i++) {
        snprintf(hex + 2 * i, 3, "%02x", data[i]);
    }
    hex[2 * MIN(dlc, 8)] = '\0';
    can_log(s, "direction=%s mailbox=%d id=0x%08x format=%s dlc=%u data=%s",
            tag, mbidx, id, ext ? "extended" : "standard", dlc, hex);
}

/*
 * Put a frame in receive mailbox mbidx and set "data ready" (MCTL_DATA).
 * Used by both can-rx and periodic replay.
 */
static uint8_t can_isr(RenesasCANState *s)
{
    uint32_t mier0 = can_ld(&s->mkr[MKR_MIER0], 4);
    uint32_t mier1 = can_ld(&s->mkr[MKR_MIER1], 4);
    int i;

    for (i = 0; i < 64; i++) {
        bool en = i < 32 ? mier0 & BIT(i) : mier1 & BIT(i - 32);

        if (en && (s->ctl[i] & (MCTL_RECREQ | MCTL_DATA)) ==
                  (MCTL_RECREQ | MCTL_DATA)) {
            return ISR_RX;
        }
    }
    return 0;
}

static void can_update_irq(RenesasCANState *s)
{
    qemu_set_irq(s->irq, can_isr(s) & s->ctl[CTL_IER]);
}

static void can_deliver_rx(RenesasCANState *s, int mbidx, uint32_t id,
                           bool ext, uint8_t dlc, const uint8_t *data)
{
    can_frame_encode(&s->mb[mbidx * 16], id, ext, dlc, data);
    s->ctl[mbidx] |= MCTL_DATA;
    can_log_frame(s, "RX", mbidx, id, ext, dlc, data);
    can_update_irq(s);
}

/* Receive mailbox with the same ID; filters are not modelled. */
static int can_find_rx_mailbox(RenesasCANState *s, uint32_t id, bool ext)
{
    int i;

    for (i = 0; i < 64; i++) {
        if (!(s->ctl[i] & MCTL_RECREQ)) {
            continue;
        }
        {
            uint32_t cid;
            bool cext;
            uint8_t cdlc, cdata[8];

            can_frame_decode(&s->mb[i * 16], &cid, &cext, &cdlc, cdata);
            if (cid == id && cext == ext) {
                return i;
            }
        }
    }
    return -1;
}

static uint64_t can_read(void *opaque, hwaddr addr, unsigned size)
{
    RenesasCANState *s = opaque;
    char ctx[80];
    uint64_t val;

    if (addr < CAN_MB_BASE + CAN_MB_SIZE) {
        val = can_ld(&s->mb[addr - CAN_MB_BASE], size);
    } else if (addr >= CAN_MKR_BASE && addr < CAN_MKR_BASE + CAN_MKR_SIZE) {
        val = can_ld(&s->mkr[addr - CAN_MKR_BASE], size);
    } else if (addr == CAN_STR && size == 2) {
        val = can_str(s);
    } else if (addr == CAN_CTL_BASE + CTL_ISR && size == 1) {
        val = can_isr(s);
    } else if (addr >= CAN_CTL_BASE && addr < CAN_CTL_BASE + CAN_CTL_SIZE) {
        val = can_ld(&s->ctl[addr - CAN_CTL_BASE], size);
    } else {
        qemu_log_mask(LOG_UNIMP, "renesas-can: read outside map +0x%"
                      HWADDR_PRIx " (%u)\n", addr, size);
        val = 0;
    }

    can_ctx(s, ctx, sizeof(ctx));
    trace_renesas_can_read(addr, size, val, ctx);
    return val;
}

static void can_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    RenesasCANState *s = opaque;
    char ctx[80];

    can_ctx(s, ctx, sizeof(ctx));
    trace_renesas_can_write(addr, size, val, ctx);

    if (addr < CAN_MB_BASE + CAN_MB_SIZE) {
        unsigned i;

        for (i = 0; i < size; i++) {
            hwaddr a = addr - CAN_MB_BASE + i;
            uint8_t b = val >> (8 * i);

            /*
             * The second mailbox byte is the high half of the 16-bit word at
             * +0x00; bit 5 (word bit 13) is not written because the driver
             * excludes it from its memory test.
             */
            if ((a & 0xf) == 1) {
                b &= 0xdf;
            }
            s->mb[a] = b;
        }
        return;
    }
    if (addr >= CAN_MKR_BASE && addr < CAN_MKR_BASE + CAN_MKR_SIZE) {
        can_st(&s->mkr[addr - CAN_MKR_BASE], size, val);
        can_log(s, "event=filter offset=0x%03" HWADDR_PRIx " value=0x%"
                PRIx64, addr, val);
        can_update_irq(s);
        return;
    }
    if (addr == CAN_STR && size == 2) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "renesas-can: write to read-only STR 0x%" PRIx64 "\n",
                      val);
        return;
    }
    if (addr >= CAN_CTL_BASE && addr < CAN_CTL_BASE + CAN_CTL_SIZE) {
        /*
         * MCTL[0..63] sit in the first 64 bytes of this block. Keep the old
         * value: a 0->1 edge on TRMREQ/RECREQ is the "guest did something"
         * event, which we complete at once since there is no bus.
         */
        hwaddr base = addr - CAN_CTL_BASE;

        if (base < 64) {
            unsigned i;

            for (i = 0; i < size && base + i < 64; i++) {
                unsigned mbidx = base + i;
                uint8_t old = s->ctl[mbidx];
                uint8_t nv = val >> (8 * i);

                s->ctl[mbidx] = nv;
                if ((nv & MCTL_TRMREQ) && !(old & MCTL_TRMREQ)) {
                    uint32_t id;
                    bool ext;
                    uint8_t dlc, data[8];

                    can_frame_decode(&s->mb[mbidx * 16], &id, &ext, &dlc,
                                     data);
                    can_log_frame(s, "TX", mbidx, id, ext, dlc, data);
                    /* "sent" at once: TRMREQ cleared, data ready */
                    s->ctl[mbidx] = (s->ctl[mbidx] & ~MCTL_TRMREQ) |
                                    MCTL_DATA;
                }
                if ((nv & MCTL_RECREQ) && !(old & MCTL_RECREQ)) {
                    can_log(s, "RXCFG mb=%u", mbidx);
                }
            }
            can_update_irq(s);
        } else {
            can_st(&s->ctl[base], size, val);
            if (addr <= CAN_CTLR && addr + size > CAN_CTLR) {
                can_ctlr_write(s, can_ld(&s->ctl[CTL_OFF(CAN_CTLR)], 2));
            }
            can_update_irq(s);
            if (addr <= 0x846 && addr + size > 0x844) {
                can_log(s, "event=bitrate value=0x%04x",
                        (unsigned)can_ld(&s->ctl[CTL_OFF(0x844)], 2));
            }
        }
        return;
    }

    qemu_log_mask(LOG_UNIMP, "renesas-can: write outside map +0x%" HWADDR_PRIx
                  " (%u) = 0x%" PRIx64 "\n", addr, size, val);
}

static const MemoryRegionOps can_ops = {
    .read = can_read,
    .write = can_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static void can_reset_hold(Object *obj, ResetType type)
{
    RenesasCANState *s = RENESAS_CAN(obj);

    memset(s->mb, 0, sizeof(s->mb));
    memset(s->mkr, 0, sizeof(s->mkr));
    memset(s->ctl, 0, sizeof(s->ctl));

    /* Controller starts in reset mode after power-on. */
    s->mode = CAN_MODE_RESET;
    s->sleep = false;
    can_st(&s->ctl[CTL_OFF(CAN_CTLR)], 2, CTLR_CANM_RESET);
    /* No mailbox search has run, so there is no result. */
    s->ctl[CTL_MSSR] = MSSR_SEST;
}

/*
 * "ID#DATA" in hex, e.g. "5c5#0102030405060708". An ID > 0x7FF is taken as
 * extended (29 bits), the same split can_frame_decode uses.
 */
static bool can_parse_frame(const char *str, uint32_t *id, bool *ext,
                            uint8_t *dlc, uint8_t data[8])
{
    const char *hashp = strchr(str, '#');
    char idbuf[16];
    const char *end;
    unsigned long parsed_id;
    const char *p;

    if (!hashp || hashp == str || hashp - str >= sizeof(idbuf)) {
        return false;
    }
    memcpy(idbuf, str, hashp - str);
    idbuf[hashp - str] = '\0';
    if (qemu_strtoul(idbuf, &end, 16, &parsed_id) || end == idbuf || *end ||
        parsed_id > 0x1fffffff) {
        return false;
    }
    *id = parsed_id;
    *ext = *id > 0x7ff;
    *dlc = 0;
    memset(data, 0, 8);
    p = hashp + 1;
    while (*p) {
        int high, low;

        while (g_ascii_isspace(*p) || *p == ',') {
            p++;
        }
        if (!*p) {
            break;
        }
        if (!p[1] || *dlc == 8) {
            return false;
        }
        high = g_ascii_xdigit_value(p[0]);
        low = g_ascii_xdigit_value(p[1]);
        p += 2;
        if (high < 0 || low < 0 ||
            (*p && !g_ascii_isspace(*p) && *p != ',' &&
             !g_ascii_isxdigit(*p))) {
            return false;
        }
        data[(*dlc)++] = high * 16 + low;
    }
    return *dlc != 0;
}

static void can_inject(RenesasCANState *s, uint32_t id, bool ext,
                       uint8_t dlc, const uint8_t *data)
{
    int mbidx = can_find_rx_mailbox(s, id, ext);

    if (mbidx < 0) {
        can_log_frame(s, "RX", -1, id, ext, dlc, data);
        return;
    }
    can_deliver_rx(s, mbidx, id, ext, dlc, data);
}

static void can_rx_set(Object *obj, const char *value, Error **errp)
{
    RenesasCANState *s = RENESAS_CAN(obj);
    uint32_t id;
    bool ext;
    uint8_t dlc, data[8];

    if (!can_parse_frame(value, &id, &ext, &dlc, data)) {
        error_setg(errp, "renesas-can: can-rx expects ID#DATA in hex");
        return;
    }
    can_inject(s, id, ext, dlc, data);
}

static char *can_rx_get(Object *obj, Error **errp)
{
    return g_strdup("");
}

static void can_replay_tick(void *opaque)
{
    CanRxReplay *r = opaque;

    can_inject(r->s, r->id, r->ext, r->dlc, r->data);
    timer_mod(r->timer,
             qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + r->period_ms);
}

/*
 * can-replay: one frame per line, "period_ms ID#DATA". Blank lines and
 * lines starting with # are skipped.
 */
static void can_load_rx_file(RenesasCANState *s, const char *path)
{
    gchar *contents;
    gsize len;
    GError *gerr = NULL;
    gchar **lines;
    int i, n;

    for (i = 0; i < s->n_replay; i++) {
        timer_del(s->replay[i].timer);
        timer_free(s->replay[i].timer);
    }
    g_free(s->replay);
    s->replay = NULL;
    s->n_replay = 0;
    if (!path || !*path) {
        return;
    }
    if (!g_file_get_contents(path, &contents, &len, &gerr)) {
        warn_report("can-replay: %s", gerr->message);
        g_error_free(gerr);
        return;
    }
    lines = g_strsplit(contents, "\n", 0);
    g_free(contents);

    for (n = 0; lines[n]; n++) {
        /* just counting */
    }
    s->replay = g_new0(CanRxReplay, n);
    s->n_replay = 0;

    for (i = 0; lines[i]; i++) {
        gchar *line = g_strstrip(lines[i]);
        unsigned period;
        char iddata[64];
        CanRxReplay *r;

        if (!*line || *line == '#') {
            continue;
        }
        if (sscanf(line, "%u %63s", &period, iddata) != 2) {
            warn_report("can-replay: bad line %s", line);
            continue;
        }
        r = &s->replay[s->n_replay];
        if (!can_parse_frame(iddata, &r->id, &r->ext, &r->dlc, r->data)) {
            warn_report("can-replay: bad frame %s", iddata);
            continue;
        }
        r->s = s;
        if (!period) {
            warn_report("can-replay: period must be nonzero: %s", line);
            continue;
        }
        r->period_ms = period;
        r->timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, can_replay_tick, r);
        timer_mod(r->timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + period);
        s->n_replay++;
    }
    g_strfreev(lines);
}

static void can_log_set(Object *obj, const char *path, Error **errp)
{
    RenesasCANState *s = RENESAS_CAN(obj);
    FILE *file = NULL;

    if (path && *path) {
        file = fopen(path, "a");
        if (!file) {
            error_setg_errno(errp, errno, "cannot open can-log '%s'", path);
            return;
        }
    }
    if (s->canlog) {
        fclose(s->canlog);
    }
    s->canlog = file;
}

static void can_replay_set(Object *obj, const char *path, Error **errp)
{
    RenesasCANState *s = RENESAS_CAN(obj);

    g_free(s->replay_path);
    s->replay_path = g_strdup(path);
    can_load_rx_file(s, path);
}

static void can_init(Object *obj)
{
    RenesasCANState *s = RENESAS_CAN(obj);

    s->trace_pc = getenv("QY8_CAN_PC") != NULL;
    object_property_add_str(obj, "rx", can_rx_get, can_rx_set);
    object_property_add_str(obj, "can-log", NULL, can_log_set);
    object_property_add_str(obj, "can-replay", NULL, can_replay_set);
    memory_region_init_io(&s->iomem, obj, &can_ops, s, "renesas-can",
                          RENESAS_CAN_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const VMStateDescription vmstate_renesas_can = {
    .name = "renesas-can",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(mb, RenesasCANState, CAN_MB_SIZE),
        VMSTATE_UINT8_ARRAY(mkr, RenesasCANState, CAN_MKR_SIZE),
        VMSTATE_UINT8_ARRAY(ctl, RenesasCANState, CAN_CTL_SIZE),
        VMSTATE_UINT8(mode, RenesasCANState),
        VMSTATE_BOOL(sleep, RenesasCANState),
        VMSTATE_END_OF_LIST()
    },
};

static void can_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->desc = "Renesas CAN controller (R-Car Gen1)";
    dc->vmsd = &vmstate_renesas_can;
    rc->phases.hold = can_reset_hold;
}

static const TypeInfo renesas_can_types[] = {
    {
        .name          = TYPE_RENESAS_CAN,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(RenesasCANState),
        .instance_init = can_init,
        .class_init    = can_class_init,
    },
};

DEFINE_TYPES(renesas_can_types)
