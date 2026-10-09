/*
 * Cypress TMA616 touch controller of the Clarion QY8202NA board,
 * modelled from what TouchPaneldrv.dll does on I2C4.
 *
 * The application answers on 0x67 and the bootloader on 0x69, one at a
 * time. The driver resets the chip through GPIO4 bit 10, waits for the
 * bootloader's interrupt, queries it (0x38), leaves it (0x3B), waits for
 * the application's interrupt, reads sysinfo and switches to operating
 * mode. Every step it waits for gets a short low pulse on the interrupt
 * line (GPIO4 bit 11). A pointer bound to the display drives one touch,
 * reported bottom to top like the TMA460's.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/core/irq.h"
#include "hw/i2c/i2c.h"
#include "hw/i2c/clarion_rcar_i2c.h"
#include "ui/input.h"
#include "trace.h"

#define TMA616_APP_ADDR     0x67
#define TMA616_BL_ADDR      0x69
#define TMA616_WIDTH        800
#define TMA616_HEIGHT       480
#define TMA616_PULSE_NS     (100 * SCALE_US)

/* hst_mode, register 0 */
#define HST_MODE_MASK       0x70
#define HST_MODE_OP         0x00
#define HST_MODE_SYSINFO    0x10
#define HST_MODE_CONFIG     0x20
#define HST_CHANGE          0x08
#define HST_RESET           0x01

#define TMA616_CMD          2
#define TMA616_CMD_COMPLETE 0x40
#define TMA616_TCH          11      /* touch count; 7-byte records follow */
#define TMA616_REC_SIZE     7

typedef enum ClarionTma616Mode {
    TMA616_OFF,                     /* held in reset */
    TMA616_BL,
    TMA616_SYSINFO,
    TMA616_OPERATING,
    TMA616_CONFIG,
} ClarionTma616Mode;

typedef enum ClarionTma616Action {
    TMA616_ACT_NONE,
    TMA616_ACT_PULSE,               /* bootloader answer ready */
    TMA616_ACT_BL_READY,            /* out of reset */
    TMA616_ACT_APP_READY,           /* application started after 0x3B */
} ClarionTma616Action;

struct ClarionTma616 {
    I2CSlave parent_obj;
    qemu_irq irq;
    QEMUTimer *pulse_timer;
    QEMUTimer *act_timer;
    QemuInputHandlerState *pointer_input;

    ClarionTma616Mode mode;
    ClarionTma616Action act;
    bool cur_bl;                    /* this transfer addressed 0x69 */
    bool writing;
    bool have_off;
    uint8_t off;
    uint8_t wlen;
    uint8_t wbuf[256];
    uint8_t regs[256];
    uint8_t bl_resp[16];

    bool btn;
    bool was_down;
    uint16_t x;
    uint16_t y;
};

/* Parameter sizes, from the driver's block 0 table */
static const uint8_t clarion_tma616_param_size[0x3d] = {
    1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 1, 2, 2, 2, 2,
    1, 2, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 2, 1,
    2, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 1, 1, 2, 1,
    1, 1, 2, 2, 1, 1, 2, 2, 2, 2, 1, 2, 2,
};

static void clarion_tma616_pulse(ClarionTma616 *s)
{
    trace_clarion_tma616_pulse(s->mode);
    qemu_set_irq(s->irq, 0);
    timer_mod(s->pulse_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + TMA616_PULSE_NS);
}

static void clarion_tma616_pulse_end(void *opaque)
{
    ClarionTma616 *s = opaque;

    qemu_set_irq(s->irq, 1);
}

static void clarion_tma616_later(ClarionTma616 *s, ClarionTma616Action act,
                                 int64_t us)
{
    s->act = act;
    timer_mod(s->act_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + us * SCALE_US);
}

static void clarion_tma616_set_mode(ClarionTma616 *s, ClarionTma616Mode mode,
                                    uint8_t hst)
{
    s->mode = mode;
    memset(s->regs, 0, sizeof(s->regs));
    if (mode == TMA616_SYSINFO) {
        /* the driver only checks hst_mode and the gesture bit */
        s->regs[0] = HST_MODE_SYSINFO;
    } else if (mode == TMA616_CONFIG) {
        s->regs[0] = HST_MODE_CONFIG;
    }
    /* keep the host's toggle and other non-mode bits */
    s->regs[0] |= hst & ~(HST_MODE_MASK | HST_CHANGE | HST_RESET);
}

static void clarion_tma616_act(void *opaque)
{
    ClarionTma616 *s = opaque;
    ClarionTma616Action act = s->act;

    s->act = TMA616_ACT_NONE;
    switch (act) {
    case TMA616_ACT_BL_READY:
        s->mode = TMA616_BL;
        memset(s->bl_resp, 0, sizeof(s->bl_resp));
        break;
    case TMA616_ACT_APP_READY:
        clarion_tma616_set_mode(s, TMA616_SYSINFO, 0);
        break;
    case TMA616_ACT_PULSE:
        break;
    default:
        return;
    }
    clarion_tma616_pulse(s);
}

static void clarion_tma616_bl_answer(ClarionTma616 *s, const uint8_t *data,
                                     int len)
{
    uint8_t *p = s->bl_resp;

    memset(p, 0, sizeof(s->bl_resp));
    p[0] = 0x01;                    /* SOP, status 0 */
    p[2] = len;
    if (len) {
        memcpy(p + 4, data, len);
    }
    p[4 + len + 2] = 0x17;          /* EOP; the driver doesn't check the CRC */
}

static void clarion_tma616_bl_packet(ClarionTma616 *s, const uint8_t *pkt,
                                     int len)
{
    /* silicon id (4), revision, bootloader version */
    static const uint8_t info[8] = { 0x05, 0xa2, 0x11, 0x69, 0x01, 1, 0, 0 };

    if (len < 7 || pkt[0] != 0x01) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "clarion-tma616: bad bootloader packet\n");
        return;
    }
    trace_clarion_tma616_bl_cmd(pkt[1]);
    switch (pkt[1]) {
    case 0x38:                      /* enter, get info */
        clarion_tma616_bl_answer(s, info, sizeof(info));
        clarion_tma616_later(s, TMA616_ACT_PULSE, 200);
        break;
    case 0x3b:                      /* exit to the application */
        clarion_tma616_bl_answer(s, NULL, 0);
        clarion_tma616_later(s, TMA616_ACT_APP_READY, 2000);
        break;
    default:
        clarion_tma616_bl_answer(s, NULL, 0);
        clarion_tma616_later(s, TMA616_ACT_PULSE, 200);
        break;
    }
}

static void clarion_tma616_command(ClarionTma616 *s)
{
    uint8_t *r = s->regs;
    uint8_t cmd = r[TMA616_CMD] & 0x3f;

    trace_clarion_tma616_command(cmd, s->mode);
    if (cmd == 0x02) {
        /* get parameter: id echoed, then its size */
        uint8_t id = r[3];

        memset(r + 3, 0, 7);
        r[3] = id;
        r[4] = id < sizeof(clarion_tma616_param_size) ?
               clarion_tma616_param_size[id] : 1;
    } else if (cmd == 0x26) {
        /* the driver expects the argument back at +6 */
        r[8] = r[3];
    }
    r[TMA616_CMD] = TMA616_CMD_COMPLETE | cmd;
    clarion_tma616_pulse(s);
}

static void clarion_tma616_hst_write(ClarionTma616 *s, uint8_t v)
{
    if (v & HST_CHANGE) {
        switch (v & HST_MODE_MASK) {
        case HST_MODE_OP:
            clarion_tma616_set_mode(s, TMA616_OPERATING, v);
            break;
        case HST_MODE_SYSINFO:
            clarion_tma616_set_mode(s, TMA616_SYSINFO, v);
            break;
        case HST_MODE_CONFIG:
            clarion_tma616_set_mode(s, TMA616_CONFIG, v);
            break;
        default:
            s->regs[0] = v & ~HST_CHANGE;
            break;
        }
        clarion_tma616_pulse(s);
    } else if (v & HST_RESET) {
        s->mode = TMA616_OFF;
        clarion_tma616_later(s, TMA616_ACT_BL_READY, 2000);
    } else {
        /* toggle bit: the host acknowledges a report */
        s->regs[0] = v;
    }
}

static void clarion_tma616_write_done(ClarionTma616 *s)
{
    trace_clarion_tma616_write(s->cur_bl, s->mode, s->off, s->wlen);

    s->writing = false;
    if (!s->have_off) {
        return;
    }
    if (s->cur_bl) {
        /* a lone byte only sets the bootloader's read pointer */
        if (s->wlen) {
            uint8_t pkt[257];

            pkt[0] = s->off;
            memcpy(pkt + 1, s->wbuf, s->wlen);
            clarion_tma616_bl_packet(s, pkt, s->wlen + 1);
            s->off = 0;
        }
        return;
    }
    if (s->mode == TMA616_BL || !s->wlen) {
        return;
    }
    for (int i = 0; i < s->wlen; i++) {
        uint8_t reg = s->off + i;

        if (reg == 0) {
            clarion_tma616_hst_write(s, s->wbuf[i]);
        } else {
            s->regs[reg] = s->wbuf[i];
        }
    }
    if (s->mode != TMA616_SYSINFO &&
        s->off <= TMA616_CMD && s->off + s->wlen > TMA616_CMD) {
        clarion_tma616_command(s);
    }
    s->off += s->wlen;
}

static bool clarion_tma616_match(I2CSlave *candidate, uint8_t address,
                                 bool broadcast, I2CNodeList *current_devs)
{
    ClarionTma616 *s = CLARION_TMA616(candidate);
    bool bl = address == TMA616_BL_ADDR;
    I2CNode *node;

    if (address != candidate->address && !bl && !broadcast) {
        return false;
    }
    s->cur_bl = bl;

    node = g_new(I2CNode, 1);
    node->elt = candidate;
    QLIST_INSERT_HEAD(current_devs, node, next);
    return true;
}

static int clarion_tma616_event(I2CSlave *slave, enum i2c_event event)
{
    ClarionTma616 *s = CLARION_TMA616(slave);

    switch (event) {
    case I2C_START_SEND:
    case I2C_START_RECV:
        if (s->writing) {
            clarion_tma616_write_done(s);
        }
        /* nothing answers in reset; the two addresses take turns */
        if (s->mode == TMA616_OFF || s->cur_bl != (s->mode == TMA616_BL)) {
            return 1;
        }
        if (event == I2C_START_SEND) {
            s->writing = true;
            s->have_off = false;
            s->wlen = 0;
        }
        break;
    case I2C_FINISH:
        if (s->writing) {
            clarion_tma616_write_done(s);
        }
        break;
    default:
        break;
    }
    return 0;
}

static int clarion_tma616_send(I2CSlave *slave, uint8_t data)
{
    ClarionTma616 *s = CLARION_TMA616(slave);

    if (!s->have_off) {
        s->off = data;
        s->have_off = true;
    } else if (s->wlen < sizeof(s->wbuf) - 1) {
        s->wbuf[s->wlen++] = data;
    }
    return 0;
}

static uint8_t clarion_tma616_recv(I2CSlave *slave)
{
    ClarionTma616 *s = CLARION_TMA616(slave);
    uint8_t off = s->off++;

    if (s->mode == TMA616_BL) {
        return off < sizeof(s->bl_resp) ? s->bl_resp[off] : 0;
    }
    return s->regs[off];
}

/* One touch in the operating mode report, then an interrupt */
static void clarion_tma616_report(ClarionTma616 *s, int event)
{
    uint8_t *r = s->regs;
    uint8_t *rec = r + TMA616_TCH + 1;

    r[TMA616_TCH - 1] = 0;
    r[TMA616_TCH] = 1;
    memset(rec, 0, TMA616_REC_SIZE);
    rec[0] = s->x >> 8;
    rec[1] = s->x;
    rec[2] = s->y >> 8;
    rec[3] = s->y;
    rec[4] = event == 3 ? 0 : 0x40;
    rec[5] = event << 5;
    rec[6] = 0x10;
    trace_clarion_tma616_report(event, s->x, s->y);
    clarion_tma616_pulse(s);
}

static void clarion_tma616_pointer_event(DeviceState *dev, QemuConsole *src,
                                         QemuInputEvent *evt)
{
    ClarionTma616 *s = CLARION_TMA616(dev);

    switch (evt->type) {
    case INPUT_EVENT_KIND_ABS:
        if (evt->abs.axis == INPUT_AXIS_X) {
            s->x = qemu_input_scale_axis(evt->abs.value, INPUT_EVENT_ABS_MIN,
                                         INPUT_EVENT_ABS_MAX,
                                         0, TMA616_WIDTH - 1);
        } else if (evt->abs.axis == INPUT_AXIS_Y) {
            /* kepdrv.dll mirrors Y here too, as on the QY8652NB */
            s->y = TMA616_HEIGHT - 1 -
                   qemu_input_scale_axis(evt->abs.value, INPUT_EVENT_ABS_MIN,
                                         INPUT_EVENT_ABS_MAX,
                                         0, TMA616_HEIGHT - 1);
        }
        break;
    case INPUT_EVENT_KIND_BTN:
        if (evt->btn.button == INPUT_BUTTON_LEFT) {
            s->btn = evt->btn.down;
        }
        break;
    default:
        break;
    }
}

static void clarion_tma616_pointer_sync(DeviceState *dev)
{
    ClarionTma616 *s = CLARION_TMA616(dev);

    if (s->mode != TMA616_OPERATING) {
        s->was_down = false;
        return;
    }
    if (s->btn) {
        clarion_tma616_report(s, s->was_down ? 2 : 1);
    } else if (s->was_down) {
        clarion_tma616_report(s, 3);
    }
    s->was_down = s->btn;
}

static const QemuInputHandler clarion_tma616_pointer_handler = {
    .name = "Clarion TMA616 absolute pointer",
    .mask = INPUT_EVENT_MASK_ABS | INPUT_EVENT_MASK_BTN,
    .event = clarion_tma616_pointer_event,
    .sync = clarion_tma616_pointer_sync,
};

void clarion_tma616_bind_pointer_input(DeviceState *dev,
                                       const char *display_id,
                                       Error **errp)
{
    ERRP_GUARD();
    ClarionTma616 *s = CLARION_TMA616(dev);

    if (s->pointer_input) {
        return;
    }
    s->pointer_input = qemu_input_handler_register(dev,
                                      &clarion_tma616_pointer_handler);
    qemu_input_handler_bind(s->pointer_input, display_id, 0, errp);
    if (*errp) {
        qemu_input_handler_unregister(s->pointer_input);
        s->pointer_input = NULL;
    }
}

/* GPIO4.OUTDT bit 10, active low like the TMA460's */
void clarion_tma616_set_reset(DeviceState *dev, bool gpio_level)
{
    ClarionTma616 *s = CLARION_TMA616(dev);

    trace_clarion_tma616_reset(gpio_level);
    if (!gpio_level) {
        s->mode = TMA616_OFF;
        s->act = TMA616_ACT_NONE;
        timer_del(s->act_timer);
    } else if (s->mode == TMA616_OFF && s->act != TMA616_ACT_BL_READY) {
        clarion_tma616_later(s, TMA616_ACT_BL_READY, 2000);
    }
}

static void clarion_tma616_reset(DeviceState *dev)
{
    ClarionTma616 *s = CLARION_TMA616(dev);

    timer_del(s->pulse_timer);
    timer_del(s->act_timer);
    /* powered with reset released, bootloader idle */
    s->mode = TMA616_BL;
    s->act = TMA616_ACT_NONE;
    s->writing = false;
    s->have_off = false;
    s->btn = false;
    s->was_down = false;
    memset(s->regs, 0, sizeof(s->regs));
    memset(s->bl_resp, 0, sizeof(s->bl_resp));
    qemu_set_irq(s->irq, 1);
}

static void clarion_tma616_realize(DeviceState *dev, Error **errp)
{
    ClarionTma616 *s = CLARION_TMA616(dev);

    s->pulse_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                  clarion_tma616_pulse_end, s);
    s->act_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, clarion_tma616_act, s);
}

static void clarion_tma616_instance_init(Object *obj)
{
    ClarionTma616 *s = CLARION_TMA616(obj);

    I2C_SLAVE(obj)->address = TMA616_APP_ADDR;
    qdev_init_gpio_out(DEVICE(s), &s->irq, 1);
}

static void clarion_tma616_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(klass);

    dc->realize = clarion_tma616_realize;
    device_class_set_legacy_reset(dc, clarion_tma616_reset);
    sc->event = clarion_tma616_event;
    sc->send = clarion_tma616_send;
    sc->recv = clarion_tma616_recv;
    sc->match_and_add = clarion_tma616_match;
}

static const TypeInfo clarion_tma616_type_info = {
    .name = TYPE_CLARION_TMA616,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(ClarionTma616),
    .instance_init = clarion_tma616_instance_init,
    .class_init = clarion_tma616_class_init,
};

static void clarion_tma616_register_types(void)
{
    type_register_static(&clarion_tma616_type_info);
}

type_init(clarion_tma616_register_types)
