/*
 * Clarion QY7 RCAN-TL1 compatible register window.
 *
 * The register layout and mailbox count follow the QY7 CAN.dll accesses.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/cutils.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/misc/clarion_rcan.h"
#include "system/reset.h"
#include "target/sh4/cpu.h"
#include "trace.h"

#define RCAN_SIZE 0x1000
#define RCAN_MCR 0x000
#define RCAN_GSR 0x002
#define RCAN_BCR1 0x004
#define RCAN_BCR0 0x006
#define RCAN_IRR 0x008
#define RCAN_IMR 0x00a
#define RCAN_TXACK1 0x030
#define RCAN_TXACK0 0x032
#define RCAN_TXPR1 0x020
#define RCAN_TXPR0 0x022
#define RCAN_RXPR1 0x040
#define RCAN_RXPR0 0x042
#define RCAN_MBIMR1 0x050
#define RCAN_MBIMR0 0x052
#define RCAN_MAILBOX_BASE 0x100
#define RCAN_MAILBOX_STRIDE 0x20
#define RCAN_MAILBOX_COUNT 32
#define RCAN_MAILBOX_END \
    (RCAN_MAILBOX_BASE + RCAN_MAILBOX_STRIDE * RCAN_MAILBOX_COUNT)

#define RCAN_MCR_RESET BIT(0)
#define RCAN_MCR_HALT BIT(1)
#define RCAN_GSR_RESET BIT(3)
#define RCAN_GSR_HALT BIT(4)
#define RCAN_GSR_NOT_BUS_OFF BIT(2)
#define RCAN_IRR_RESET_HALT BIT(0)
#define RCAN_IRR_RX BIT(1)
#define RCAN_IRR_TX BIT(8)

struct ClarionRcan {
    MemoryRegion iomem;
    qemu_irq irq;
    uint16_t regs[RCAN_SIZE / sizeof(uint16_t)];
    bool reset_requested;
    bool halt_requested;
    FILE *canlog;
    GPtrArray *replay;
};

typedef struct RcanReplay {
    ClarionRcan *s;
    QEMUTimer *timer;
    uint32_t period_ms;
    uint16_t id;
    uint8_t len;
    uint8_t data[8];
} RcanReplay;

static void rcan_log(ClarionRcan *s, const char *fmt, ...)
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

static void rcan_log_frame(ClarionRcan *s, const char *direction,
                           int mailbox, uint16_t id, const uint8_t *data,
                           unsigned len)
{
    char hex[17];
    unsigned data_len = MIN(len, 8);

    for (unsigned i = 0; i < data_len; i++) {
        snprintf(hex + i * 2, 3, "%02x", data[i]);
    }
    hex[data_len * 2] = '\0';
    rcan_log(s, "direction=%s mailbox=%d id=0x%08x format=standard "
             "dlc=%u data=%s", direction, mailbox, id, len, hex);
}

static const char *rcan_mode_name(bool reset, bool halt)
{
    return reset ? "reset" : halt ? "halt" : "operation";
}

static uint16_t *rcan_reg(ClarionRcan *s, hwaddr offset)
{
    return &s->regs[offset / sizeof(uint16_t)];
}

static uint32_t rcan_rxpr(ClarionRcan *s)
{
    return *rcan_reg(s, RCAN_RXPR0) |
           ((uint32_t)*rcan_reg(s, RCAN_RXPR1) << 16);
}

static uint32_t rcan_mbimr(ClarionRcan *s)
{
    return *rcan_reg(s, RCAN_MBIMR0) |
           ((uint32_t)*rcan_reg(s, RCAN_MBIMR1) << 16);
}

static uint32_t rcan_txack(ClarionRcan *s)
{
    return *rcan_reg(s, RCAN_TXACK0) |
           ((uint32_t)*rcan_reg(s, RCAN_TXACK1) << 16);
}

static uint16_t rcan_mailbox_id(ClarionRcan *s, hwaddr base)
{
    uint16_t control0 = *rcan_reg(s, base);

    return (control0 >> 2) & 0x7ff;
}

static uint16_t rcan_mailbox_id_mask(ClarionRcan *s, hwaddr base)
{
    uint16_t mask = *rcan_reg(s, base + 4);

    return (mask >> 2) & 0x7ff;
}

static unsigned rcan_mailbox_config(ClarionRcan *s, hwaddr base)
{
    return (*rcan_reg(s, base + 0x10) >> 8) & 7;
}

static void rcan_update_rx_flag(ClarionRcan *s)
{
    uint16_t *irr = rcan_reg(s, RCAN_IRR);

    if (rcan_rxpr(s) & ~rcan_mbimr(s)) {
        *irr |= RCAN_IRR_RX;
    } else {
        *irr &= ~RCAN_IRR_RX;
    }
}

static void rcan_update_irq(ClarionRcan *s)
{
    uint32_t unmasked_txack = rcan_txack(s) & ~rcan_mbimr(s);
    uint16_t *irr = rcan_reg(s, RCAN_IRR);
    uint16_t imr = *rcan_reg(s, RCAN_IMR);
    bool pending;

    rcan_update_rx_flag(s);
    if (unmasked_txack) {
        *irr |= RCAN_IRR_TX;
    } else {
        *irr &= ~RCAN_IRR_TX;
    }
    pending = (*irr & ~imr) != 0;
    qemu_set_irq(s->irq, pending);
}

static void rcan_reset_registers(ClarionRcan *s, uint16_t mcr)
{
    memset(s->regs, 0, sizeof(s->regs));
    *rcan_reg(s, RCAN_MCR) = mcr;
    *rcan_reg(s, RCAN_GSR) = RCAN_GSR_RESET | RCAN_GSR_NOT_BUS_OFF;
    *rcan_reg(s, RCAN_IRR) = RCAN_IRR_RESET_HALT;
    *rcan_reg(s, RCAN_IMR) = 0xffff;
    *rcan_reg(s, RCAN_MBIMR0) = 0xffff;
    *rcan_reg(s, RCAN_MBIMR1) = 0xffff;
    s->reset_requested = true;
    s->halt_requested = false;
    qemu_set_irq(s->irq, 0);
}

static void rcan_start_transmit(ClarionRcan *s, uint32_t requested)
{
    uint32_t txack = rcan_txack(s);

    for (int i = 0; i < RCAN_MAILBOX_COUNT; i++) {
        hwaddr base = RCAN_MAILBOX_BASE + i * RCAN_MAILBOX_STRIDE;
        uint32_t bit = BIT(i);
        unsigned config;

        if (!(requested & bit)) {
            continue;
        }
        config = rcan_mailbox_config(s, base);
        if (s->reset_requested || s->halt_requested || config > 1) {
            continue;
        }
        {
            uint8_t data[8];
            unsigned dlc = *rcan_reg(s, base + 0x10) & 0xf;
            unsigned len = MIN(dlc, 8);

            for (unsigned j = 0; j < len; j++) {
                uint16_t word = *rcan_reg(s, base + 0x08 + (j & ~1u));

                data[j] = (j & 1) ? word : word >> 8;
            }
            rcan_log_frame(s, "TX", i, rcan_mailbox_id(s, base), data, dlc);
        }
        txack |= bit;
    }
    *rcan_reg(s, RCAN_TXACK0) = txack;
    *rcan_reg(s, RCAN_TXACK1) = txack >> 16;
    *rcan_reg(s, RCAN_TXPR0) &= ~requested;
    *rcan_reg(s, RCAN_TXPR1) &= ~(requested >> 16);
    rcan_update_irq(s);
}

static void rcan_write_mcr(ClarionRcan *s, uint16_t value)
{
    uint16_t *gsr = rcan_reg(s, RCAN_GSR);

    bool old_reset = s->reset_requested;
    bool old_halt = s->halt_requested;
    const char *old_mode = rcan_mode_name(old_reset, old_halt);

    if ((value & RCAN_MCR_RESET) && !old_reset) {
        rcan_reset_registers(s, value);
    }
    *rcan_reg(s, RCAN_MCR) = value;
    s->reset_requested = value & RCAN_MCR_RESET;
    s->halt_requested = value & RCAN_MCR_HALT;
    if (s->reset_requested) {
        *gsr = RCAN_GSR_RESET | RCAN_GSR_NOT_BUS_OFF;
        *gsr &= ~RCAN_GSR_HALT;
    } else {
        *gsr = RCAN_GSR_NOT_BUS_OFF;
        if (s->halt_requested) {
            *gsr |= RCAN_GSR_HALT;
        } else {
            *gsr &= ~RCAN_GSR_HALT;
        }
    }
    if (old_reset != s->reset_requested || old_halt != s->halt_requested) {
        *rcan_reg(s, RCAN_IRR) |= RCAN_IRR_RESET_HALT;
        rcan_log(s, "event=mode old=%s new=%s sleep=0", old_mode,
                 rcan_mode_name(s->reset_requested, s->halt_requested));
    }
    rcan_update_irq(s);
}

static uint64_t rcan_read(void *opaque, hwaddr addr, unsigned size)
{
    ClarionRcan *s = opaque;
    uint64_t value = 0;

    if (addr + size > RCAN_SIZE) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "clarion-rcan: out of range read +0x%03" HWADDR_PRIx
                      " size %u\n",
                      addr, size);
        return 0;
    }
    if (size == 2 && !(addr & 1)) {
        value = *rcan_reg(s, addr);
    } else if (size == 1) {
        uint16_t word = *rcan_reg(s, addr & ~1);
        bool mailbox = addr >= RCAN_MAILBOX_BASE && addr < RCAN_MAILBOX_END;
        bool high_byte = mailbox ? !(addr & 1) : (addr & 1);

        value = high_byte ? word >> 8 : word & 0xff;
    } else {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "clarion-rcan: unsupported read +0x%03" HWADDR_PRIx
                      " size %u\n",
                      addr, size);
    }
    trace_clarion_rcan_read(qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL), addr,
                            value);
    return value;
}

static void rcan_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    ClarionRcan *s = opaque;
    uint16_t old_value;
    uint16_t previous_value;

    if (addr + size > RCAN_SIZE) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "clarion-rcan: out of range write +0x%03" HWADDR_PRIx
                      " size %u\n",
                      addr, size);
        return;
    }
    previous_value = *rcan_reg(s, addr & ~1);
    if (size == 1) {
        uint16_t *reg = rcan_reg(s, addr & ~1);
        bool mailbox = addr >= RCAN_MAILBOX_BASE && addr < RCAN_MAILBOX_END;
        bool high_byte = mailbox ? !(addr & 1) : (addr & 1);
        uint16_t byte_mask = high_byte ? 0xff00 : 0x00ff;
        uint16_t byte_value = high_byte ? (value & 0xff) << 8 : value & 0xff;

        *reg = (*reg & ~byte_mask) | byte_value;
    } else if (size == 4 && addr == RCAN_TXPR1) {
        *rcan_reg(s, RCAN_TXPR1) |= value;
        *rcan_reg(s, RCAN_TXPR0) |= value >> 16;
        rcan_start_transmit(s, value);
    } else if (size == 2 && !(addr & 1)) {
        if (addr == RCAN_MCR) {
            rcan_write_mcr(s, value);
        } else if (addr == RCAN_RXPR0 || addr == RCAN_RXPR1) {
            old_value = *rcan_reg(s, addr);
            *rcan_reg(s, addr) = old_value & ~value;
        } else if (addr == RCAN_IRR) {
            old_value = *rcan_reg(s, addr);
            old_value &= ~value;
            *rcan_reg(s, addr) = old_value;
        } else if (addr == RCAN_TXPR0 || addr == RCAN_TXPR1) {
            *rcan_reg(s, addr) |= value;
            rcan_start_transmit(s, addr == RCAN_TXPR0 ? value : value << 16);
        } else if (addr == RCAN_TXACK0 || addr == RCAN_TXACK1) {
            old_value = *rcan_reg(s, addr);
            *rcan_reg(s, addr) = old_value & ~value;
        } else {
            *rcan_reg(s, addr) = value;
        }
        if (addr == RCAN_RXPR0 || addr == RCAN_RXPR1 || addr == RCAN_MBIMR0 ||
            addr == RCAN_MBIMR1 || addr == RCAN_IMR || addr == RCAN_IRR ||
            addr == RCAN_TXACK0 || addr == RCAN_TXACK1) {
            rcan_update_irq(s);
        }
    } else {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "clarion-rcan: unsupported write +0x%03" HWADDR_PRIx
                      " size %u\n",
                      addr, size);
        return;
    }
    if (previous_value != *rcan_reg(s, addr & ~1)) {
        if (addr == RCAN_MBIMR0 || addr == RCAN_MBIMR1) {
            rcan_log(s, "event=filter field=mailbox-mask offset=0x%03"
                     HWADDR_PRIx " value=0x%04x", addr,
                     *rcan_reg(s, addr));
        } else if (addr >= RCAN_MAILBOX_BASE && addr < RCAN_MAILBOX_END) {
            hwaddr base = RCAN_MAILBOX_BASE +
                ((addr - RCAN_MAILBOX_BASE) / RCAN_MAILBOX_STRIDE) *
                RCAN_MAILBOX_STRIDE;
            hwaddr field = (addr - base) & ~1;
            const char *name = field == 0 ? "id" :
                               field == 4 ? "id-mask" :
                               field == 0x10 ? "config" : NULL;

            if (name) {
                rcan_log(s, "event=filter mailbox=%u field=%s value=0x%04x",
                         (unsigned)((base - RCAN_MAILBOX_BASE) /
                                    RCAN_MAILBOX_STRIDE), name,
                         *rcan_reg(s, addr & ~1));
            }
        }
    }
    trace_clarion_rcan_write(qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL),
                             SUPERH_CPU(current_cpu)->env.pc, size, addr,
                             value);
}

static const MemoryRegionOps rcan_ops = {
    .read = rcan_read,
    .write = rcan_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void rcan_reset(void *opaque)
{
    ClarionRcan *s = opaque;

    rcan_reset_registers(s, 0x8001);
}

ClarionRcan *clarion_rcan_init(MemoryRegion *sysmem, hwaddr base, qemu_irq irq)
{
    ClarionRcan *s = g_new0(ClarionRcan, 1);

    s->irq = irq;
    memory_region_init_io(&s->iomem, NULL, &rcan_ops, s, "clarion.rcan",
                          RCAN_SIZE);
    memory_region_add_subregion(sysmem, base, &s->iomem);
    qemu_register_reset(rcan_reset, s);
    return s;
}

bool clarion_rcan_inject(ClarionRcan *s, uint16_t id, const uint8_t *data,
                         unsigned len)
{
    g_assert(len <= 8);
    for (int i = RCAN_MAILBOX_COUNT - 1; i >= 0; i--) {
        hwaddr base = RCAN_MAILBOX_BASE + i * RCAN_MAILBOX_STRIDE;
        uint32_t bit = BIT(i);
        unsigned config = rcan_mailbox_config(s, base);

        if ((config != 2 && config != 3) ||
            ((rcan_mailbox_id(s, base) ^ id) & ~rcan_mailbox_id_mask(s, base) &
             0x7ff)) {
            continue;
        }
        for (unsigned j = 0; j < len; j += 2) {
            uint16_t word = (uint16_t)data[j] << 8;

            if (j + 1 < len) {
                word |= data[j + 1];
            }
            *rcan_reg(s, base + 0x08 + j) = word;
        }
        for (unsigned j = len; j < 8; j++) {
            uint16_t *word = rcan_reg(s, base + 0x08 + (j & ~1u));

            if (j & 1) {
                *word &= 0xff00;
            } else {
                *word &= 0x00ff;
            }
        }
        *rcan_reg(s, base) =
            (*rcan_reg(s, base) & ~0x1ffc) | ((id & 0x7ff) << 2);
        *rcan_reg(s, base + 0x10) =
            (*rcan_reg(s, base + 0x10) & 0xff00) | (len & 0x000f);
        if (i < 16) {
            *rcan_reg(s, RCAN_RXPR0) |= bit;
        } else {
            *rcan_reg(s, RCAN_RXPR1) |= bit >> 16;
        }
        rcan_log_frame(s, "RX", i, id, data, len);
        rcan_update_irq(s);
        qemu_log("clarion-rcan: rx id=%03x mailbox=%u len=%u\n", id, i, len);
        return true;
    }
    rcan_log_frame(s, "RX", -1, id, data, len);
    return false;
}

static bool rcan_parse_frame(const char *text, uint16_t *id, uint8_t data[8],
                             unsigned *len)
{
    const char *hash = strchr(text, '#');
    char idbuf[9];
    const char *end;
    unsigned long parsed;
    const char *p;

    if (!hash || hash == text || hash - text > 8) {
        return false;
    }
    memcpy(idbuf, text, hash - text);
    idbuf[hash - text] = '\0';
    if (qemu_strtoul(idbuf, &end, 16, &parsed) || end == idbuf || *end ||
        parsed > 0x7ff) {
        return false;
    }
    *len = 0;
    p = hash + 1;
    while (*p) {
        int hi, lo;

        while (g_ascii_isspace(*p) || *p == ',') {
            p++;
        }
        if (!*p) {
            break;
        }
        if (!p[0] || !p[1]) {
            return false;
        }
        hi = g_ascii_xdigit_value(p[0]);
        lo = g_ascii_xdigit_value(p[1]);
        p += 2;
        if (hi < 0 || lo < 0 || *len == 8 ||
            (*p && !g_ascii_isspace(*p) && *p != ',' &&
             !g_ascii_isxdigit(*p))) {
            return false;
        }
        data[(*len)++] = hi * 16 + lo;
    }
    *id = parsed;
    return *len != 0;
}

static void rcan_replay_tick(void *opaque)
{
    RcanReplay *r = opaque;

    clarion_rcan_inject(r->s, r->id, r->data, r->len);
    timer_mod(r->timer,
              qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + r->period_ms);
}

void clarion_rcan_set_log(ClarionRcan *s, const char *path)
{
    if (s->canlog) {
        fclose(s->canlog);
        s->canlog = NULL;
    }
    if (path && *path) {
        s->canlog = fopen(path, "a");
        if (!s->canlog) {
            warn_report("can-log: %s: %s", path, strerror(errno));
        }
    }
}

void clarion_rcan_set_replay(ClarionRcan *s, const char *path)
{
    gchar *contents, **lines;
    GError *error = NULL;

    if (s->replay) {
        for (guint i = 0; i < s->replay->len; i++) {
            RcanReplay *r = g_ptr_array_index(s->replay, i);

            timer_del(r->timer);
            timer_free(r->timer);
            g_free(r);
        }
        g_ptr_array_free(s->replay, true);
        s->replay = NULL;
    }
    if (!path || !*path) {
        return;
    }
    if (!g_file_get_contents(path, &contents, NULL, &error)) {
        warn_report("can-replay: %s", error->message);
        g_error_free(error);
        return;
    }
    s->replay = g_ptr_array_new();
    lines = g_strsplit(contents, "\n", -1);
    g_free(contents);
    for (guint i = 0; lines[i]; i++) {
        gchar *line = g_strstrip(lines[i]);
        RcanReplay *r;
        char frame[64];
        const char *end;
        unsigned long period;
        unsigned len;

        if (!*line || *line == '#') {
            continue;
        }
        if (qemu_strtoul(line, &end, 10, &period) || end == line ||
            *end == '\0' || !g_ascii_isspace(*end) ||
            !period || period > UINT32_MAX ||
            sscanf(end, "%63s", frame) != 1) {
            warn_report("can-replay: invalid line: %s", line);
            continue;
        }
        r = g_new0(RcanReplay, 1);
        r->s = s;
        r->period_ms = period;
        if (!rcan_parse_frame(frame, &r->id, r->data, &len)) {
            warn_report("can-replay: invalid frame: %s", frame);
            g_free(r);
            continue;
        }
        r->len = len;
        r->timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, rcan_replay_tick, r);
        timer_mod(r->timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + period);
        g_ptr_array_add(s->replay, r);
    }
    g_strfreev(lines);
}
