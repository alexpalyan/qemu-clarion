/*
 * Clarion QY7221NL sub-CPU transport.
 *
 * The CAN driver of the QY7221NL image (CAN.dll) does not touch a CAN
 * controller for the BCM/HVAC/steering traffic it carries on "CAN1:": after
 * the three IOCTL-driven init passes on 0xFFFD0000, it talks over a UART
 * at physical 0x19B00000 (hw/char/clarion_qy7_uart.c). This file models only
 * the framing of that link; command semantics (the actual CAN/BCM protocol
 * riding on top) belong to a separate responder object that attaches
 * through clarion_scpu_set_body_sink().
 *
 * Framing is the one already modelled for the display link
 * (hw/misc/clarion_dispmicom.c), confirmed on this link by a live capture:
 * guest-tx "10 02 23 10 03 30", XOR(23 10 03) = 0x30:
 *
 *     10 02 | DATA... (DLE stuffed) | 10 03 | BCC
 *     BCC = XOR of every byte after 10 02, trailing 10 03 included
 *
 * Both sides acknowledge every good frame with the two bytes 10 06 (DLE
 * ACK); a bad checksum gets 10 15 (DLE NAK). PROVEN: dispatcher at
 * CAN.dll+0x30DC (guest VA 0x03DD30DC) switches on body[0] after the same
 * checksum discipline docs/21-antitheft.md describes for the BCM challenge
 * (byte 5 = sum of bytes 0..4); one observed guest frame
 * "51 30 00 07 1E 14 17 F0 59 ED 61 FF FF" carries CAN ID 0x071E (= 0x71E,
 * the antitheft TX id) and, at offset 5, the exact `0x14 seed 0xF0 .. ..`
 * challenge of docs/21 with a matching checksum byte — confirming this UART
 * is the antitheft CAN1: transport.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/core/qdev-properties.h"
#include "hw/char/clarion_qy7_uart.h"
#include "hw/misc/clarion_scpu.h"

#define DLE 0x10
#define STX 0x02
#define ETX 0x03
#define ACK 0x06
#define NAK 0x15

#define SCPU_MAX_DATA 160

/* Message codes of the CAN.dll <-> sub-CPU protocol (CAN.dll+0x30DC). */
#define MSG_HELLO       0x23    /* guest: who are you */
#define MSG_VERSION     0x24    /* sub-CPU: version block */
#define MSG_NOTICE_REG  0x41    /* register a receive ID, acked with 0x41 */
#define MSG_CAN_TX      0x51    /* guest: send a frame, acked with 0x51 */
#define MSG_CAN_RX      0x65    /* sub-CPU: received frame(s) for an ID */

/* Pause between two frames and delay for refilling a busy UART FIFO. */
#define SCPU_GAP_NS   (5 * 1000 * 1000)
#define SCPU_FILL_NS  (1 * 1000 * 1000)

typedef enum {
    RX_IDLE, RX_DLE, RX_BODY, RX_BODY_DLE, RX_BCC,
} ScpuRxState;

struct ClarionScpuState {
    DeviceState parent_obj;

    DeviceState *uart;
    ClarionScpuBodyFn body_fn;
    void *body_opaque;
    ClarionScpuCanTxFn can_fn;
    void *can_opaque;

    QEMUTimer *tx;
    GQueue *frames;
    GByteArray *cur;
    unsigned cur_off;

    ScpuRxState rx_state;
    uint8_t body[SCPU_MAX_DATA];
    unsigned body_len;
    uint8_t bcc;
};

/*
 * Bytes [hide_from, hide_to) of the trace are printed as "xx": the second
 * step of the antitheft exchange carries a vehicle-specific code that must
 * not end up in logs.
 */
static void scpu_trace_hide(const char *dir, const uint8_t *data, unsigned len,
                            unsigned hide_from, unsigned hide_to)
{
    g_autoptr(GString) body = g_string_new(NULL);

    for (unsigned i = 0; i < len; i++) {
        if (i >= hide_from && i < hide_to) {
            g_string_append_printf(body, "%sxx", i ? " " : "");
        } else {
            g_string_append_printf(body, "%s%02x", i ? " " : "", data[i]);
        }
    }
    qemu_log_mask(LOG_GUEST_ERROR,
                  "clarion-scpu: trace virtual-ms=%" PRId64
                  " dir=%s body=%s\n",
                  qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL), dir, body->str);
}

static void scpu_trace(const char *dir, const uint8_t *data, unsigned len)
{
    scpu_trace_hide(dir, data, len, 0, 0);
}

static GByteArray *scpu_build(const uint8_t *data, unsigned len)
{
    GByteArray *f = g_byte_array_new();
    uint8_t b, bcc = 0;

    b = DLE; g_byte_array_append(f, &b, 1);
    b = STX; g_byte_array_append(f, &b, 1);
    for (unsigned i = 0; i < len; i++) {
        uint8_t v = data[i];

        g_byte_array_append(f, &v, 1);
        bcc ^= v;
        if (v == DLE) {
            g_byte_array_append(f, &v, 1);
            bcc ^= v;
        }
    }
    b = DLE; g_byte_array_append(f, &b, 1); bcc ^= b;
    b = ETX; g_byte_array_append(f, &b, 1); bcc ^= b;
    g_byte_array_append(f, &bcc, 1);
    return f;
}

static void scpu_arm(ClarionScpuState *s, int64_t delay)
{
    timer_mod(s->tx, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + delay);
}

void clarion_scpu_send(DeviceState *dev, const uint8_t *data, unsigned len)
{
    ClarionScpuState *s = CLARION_SCPU(dev);

    g_queue_push_tail(s->frames, scpu_build(data, len));
    if (!s->cur) {
        scpu_arm(s, SCPU_FILL_NS);
    }
}

static void scpu_queue_ack(ClarionScpuState *s, uint8_t code)
{
    GByteArray *f = g_byte_array_new();
    uint8_t b = DLE;

    g_byte_array_append(f, &b, 1);
    g_byte_array_append(f, &code, 1);
    g_queue_push_head(s->frames, f);
    if (!s->cur) {
        scpu_arm(s, SCPU_FILL_NS);
    }
}

static void scpu_tx_expire(void *opaque)
{
    ClarionScpuState *s = opaque;
    int took;

    if (!s->cur) {
        s->cur = g_queue_pop_head(s->frames);
        s->cur_off = 0;
        if (!s->cur) {
            return;
        }
    }
    took = clarion_qy7_uart_rx(s->uart, s->cur->data + s->cur_off,
                               s->cur->len - s->cur_off);
    if (took > 0) {
        if (!s->cur_off) {
            if (s->cur->len == 2) {
                scpu_trace("sub-tx", s->cur->data, 2);
            } else {
                uint8_t data[SCPU_MAX_DATA];
                unsigned len = 0;

                for (unsigned i = 2; i < s->cur->len - 3; i++) {
                    data[len++] = s->cur->data[i];
                    if (s->cur->data[i] == DLE) {
                        i++;
                    }
                }
                if (len >= 10 && data[0] == MSG_CAN_RX) {
                    /* hide data bytes 1..5 (code and checksum) of received frames */
                    scpu_trace_hide("sub-tx", data, len, 6, 11);
                } else {
                    scpu_trace("sub-tx", data, len);
                }
            }
        }
        s->cur_off += took;
    }
    if (s->cur_off >= s->cur->len) {
        g_byte_array_free(s->cur, TRUE);
        s->cur = NULL;
        s->cur_off = 0;
        if (!g_queue_is_empty(s->frames)) {
            scpu_arm(s, SCPU_GAP_NS);
        }
    } else {
        scpu_arm(s, SCPU_FILL_NS);
    }
}

/* Synthetic version block: the guest only logs it. */
static const uint8_t scpu_version[] = {
    MSG_VERSION, 0x01, 0x20, 0x14, 0x01, 0x01,
    'Q', 'E', 'M', 'U', 'S', 'C', 'P', 'U', '0', '0', '1', 0x00,
};

void clarion_scpu_can_rx(DeviceState *dev, uint16_t id, const uint8_t *data,
                         unsigned len)
{
    uint8_t msg[5 + 8];

    g_assert(len <= 8);
    msg[0] = MSG_CAN_RX;
    msg[1] = id >> 8;
    msg[2] = id & 0xff;
    msg[3] = 1;             /* one record */
    msg[4] = len;
    memcpy(msg + 5, data, len);
    clarion_scpu_send(dev, msg, 5 + len);
}

void clarion_scpu_set_can_sink(DeviceState *dev, ClarionScpuCanTxFn fn,
                               void *opaque)
{
    ClarionScpuState *s = CLARION_SCPU(dev);

    s->can_fn = fn;
    s->can_opaque = opaque;
}

static void scpu_controller(ClarionScpuState *s)
{
    const uint8_t *b = s->body;
    unsigned len = s->body_len;

    switch (b[0]) {
    case MSG_HELLO:
        clarion_scpu_send(DEVICE(s), scpu_version, sizeof(scpu_version));
        break;
    case MSG_NOTICE_REG:
        if (len >= 7) {
            /* echo the tag bytes, the receive ID moves to offset 6..7 */
            uint8_t ack[8] = { MSG_NOTICE_REG, b[1], b[2], b[3], b[4], 0,
                               b[5], b[6] };

            clarion_scpu_send(DEVICE(s), ack, sizeof(ack));
        }
        break;
    case MSG_CAN_TX:
        if (len >= 5) {
            uint8_t ack[3] = { MSG_CAN_TX, b[1], b[2] };
            uint16_t id = (b[3] << 8) | b[4];

            clarion_scpu_send(DEVICE(s), ack, sizeof(ack));
            if (s->can_fn && len - 5 <= 8) {
                s->can_fn(s->can_opaque, id, b + 5, len - 5);
            }
        }
        break;
    default:
        break;
    }
}

static void scpu_frame(ClarionScpuState *s)
{
    if (s->body_len >= 10 && s->body[0] == MSG_CAN_TX && s->body[5] == 0x24) {
        /* antitheft step 2: data bytes 1..4 are the vehicle code, 5 its sum */
        scpu_trace_hide("guest-tx", s->body, s->body_len, 6, 11);
    } else {
        scpu_trace("guest-tx", s->body, s->body_len);
    }
    scpu_queue_ack(s, ACK);
    if (!s->body_len) {
        return;
    }
    scpu_controller(s);
    if (s->body_fn) {
        s->body_fn(s->body_opaque, s->body, s->body_len);
    }
}

void clarion_scpu_rx_byte(DeviceState *dev, uint8_t b)
{
    ClarionScpuState *s = CLARION_SCPU(dev);

    switch (s->rx_state) {
    case RX_IDLE:
        if (b == DLE) {
            s->rx_state = RX_DLE;
        }
        break;
    case RX_DLE:
        if (b == STX) {
            s->body_len = 0;
            s->bcc = 0;
            s->rx_state = RX_BODY;
        } else if (b != DLE) {
            /* DLE ACK / DLE NAK: acknowledgement of one of our frames */
            s->rx_state = RX_IDLE;
        }
        break;
    case RX_BODY:
        s->bcc ^= b;
        if (b == DLE) {
            s->rx_state = RX_BODY_DLE;
        } else if (s->body_len < sizeof(s->body)) {
            s->body[s->body_len++] = b;
        }
        break;
    case RX_BODY_DLE:
        s->bcc ^= b;
        if (b == DLE) {
            if (s->body_len < sizeof(s->body)) {
                s->body[s->body_len++] = DLE;
            }
            s->rx_state = RX_BODY;
        } else if (b == ETX) {
            s->rx_state = RX_BCC;
        } else {
            s->rx_state = RX_IDLE;
        }
        break;
    case RX_BCC:
        if (b != s->bcc) {
            qemu_log_mask(LOG_GUEST_ERROR, "clarion-scpu: BCC %#x, expected "
                          "%#x\n", b, s->bcc);
            scpu_queue_ack(s, NAK);
        } else {
            scpu_frame(s);
        }
        s->rx_state = RX_IDLE;
        break;
    }
}

void clarion_scpu_attach_uart(DeviceState *dev, DeviceState *uart)
{
    CLARION_SCPU(dev)->uart = uart;
}

void clarion_scpu_set_body_sink(DeviceState *dev, ClarionScpuBodyFn fn,
                                void *opaque)
{
    ClarionScpuState *s = CLARION_SCPU(dev);

    s->body_fn = fn;
    s->body_opaque = opaque;
}

static void scpu_reset_hold(Object *obj, ResetType type)
{
    ClarionScpuState *s = CLARION_SCPU(obj);

    while (!g_queue_is_empty(s->frames)) {
        g_byte_array_free(g_queue_pop_head(s->frames), TRUE);
    }
    if (s->cur) {
        g_byte_array_free(s->cur, TRUE);
        s->cur = NULL;
    }
    s->cur_off = 0;
    s->rx_state = RX_IDLE;
    s->body_len = 0;
    s->bcc = 0;
}

static void scpu_realize(DeviceState *dev, Error **errp)
{
    ClarionScpuState *s = CLARION_SCPU(dev);

    s->frames = g_queue_new();
    s->tx = timer_new_ns(QEMU_CLOCK_VIRTUAL, scpu_tx_expire, s);
}

static void scpu_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = scpu_realize;
    dc->desc = "Clarion QY7221NL sub-CPU transport";
    rc->phases.hold = scpu_reset_hold;
}

static const TypeInfo scpu_types[] = {
    {
        .name = TYPE_CLARION_SCPU,
        .parent = TYPE_DEVICE,
        .instance_size = sizeof(ClarionScpuState),
        .class_init = scpu_class_init,
    },
};

DEFINE_TYPES(scpu_types)
