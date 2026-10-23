/*
 * Micom, the Clarion QY8XXX board's companion microcontroller.
 *
 * On the real board this is a separate chip connected to the main SoC over
 * SCIF4. Without its response, the WinCE kernel cannot initialize: the device
 * manager waits on the link and boot does not reach GWES.
 *
 * The frame format was derived from EdaDrv.dll (builder @0xef78f164), not
 * from documentation:
 *
 *     10 02 | LEN DATA... (stuffing: 10 -> 10 10) | 10 03 | BCC
 *     BCC = XOR of all bytes after DLE STX, including trailing 10 03
 *     (eor/uxtb loop @0xef78ee58)
 *
 * Verified against a frame from the kernel itself: 10 02 02 08 00 10 03 19,
 * XOR(02 08 00 10 03) = 0x19 ✔
 *
 * Received-frame parsing (class accessor vtable @0xef783490): SetFrame
 * @0xef78e96c requires len(frame) - 6 == frame[2], stores LEN in obj[0x18]
 * and DATA in obj[0x1c]. Dispatcher @0xef78baa4 uses DATA[0] as the command
 * code; handler 0x08 @0xef78bb04 uses DATA[1] as the subcode.
 *
 * Handshake sequence:
 *
 *     HU  -> 08 00              reset (SendCmd08(0) @0xef78b528)  state 1
 *     HU  <- 08 01              micom alive                       state 2
 *     HU  <- 08 0a <codes>      micom command set                 state 3
 *     HU  -> 08 0b              command set accepted
 *
 * ⚠ Model scope: this implements only behavior established from EdaDrv.dll:
 * framing, checksum, the 1->2->3 state machine, and two commands in the 0x60
 * family decoded in the same source. Other commands receive only an ACK
 * containing "<code> 00", as in the original host-side responder. Undecoded
 * commands are deliberately not invented. To experiment with them, disable
 * the micom (-M clarion-qy8,micom=off) and connect a serial responder to SCIF4.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/misc/clarion_micom.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"

#define DLE     0x10
#define STX     0x02
#define ETX     0x03
#define ACK     0x06
#define NAK     0x15

#define MICOM_MAX_DATA      256

/*
 * Gap between frames. The host-side responder uses 50 ms of wall time; here
 * it is virtual time, so boot no longer depends on host load. The gap is still
 * needed because the SCIF receiver has a 16-byte FIFO and the driver must ACK
 * the previous frame before the next one arrives.
 */
#define MICOM_GAP_NS        (50 * 1000 * 1000)
/* Refill the remainder of a long frame as DMA frees the FIFO. */
#define MICOM_FILL_NS       (1 * 1000 * 1000)

typedef enum {
    RX_IDLE,            /* waiting for DLE */
    RX_DLE,             /* saw DLE outside a frame */
    RX_BODY,            /* inside a frame */
    RX_BODY_DLE,        /* inside a frame, saw DLE */
    RX_BCC,             /* next byte is the checksum */
} MicomRxState;

struct ClarionMicomState {
    DeviceState parent_obj;

    ClarionMicomSink sink;
    void *sink_opaque;

    QEMUTimer *tx;
    GQueue *frames;             /* queue of ready GByteArray * frames */
    GByteArray *cur;            /* frame currently being delivered */
    unsigned cur_off;           /* number of bytes already accepted */

    MicomRxState rx_state;
    uint8_t body[MICOM_MAX_DATA + 2];
    unsigned body_len;
    uint8_t bcc;

    bool handshake_done;        /* saw 08 0b from the kernel */
    bool cenet_audio_off;
    bool cenet_antitheft_release;
    bool cenet_antitheft_release_sent;
    unsigned handshake_count;
    bool generic_reply;
    uint8_t generic_reply_subcode;
};

/* --- frame construction ------------------------------------------------ */

static GByteArray *micom_build(const uint8_t *data, unsigned len)
{
    GByteArray *f = g_byte_array_new();
    uint8_t b, bcc = 0;
    unsigned i;

    b = DLE; g_byte_array_append(f, &b, 1);
    b = STX; g_byte_array_append(f, &b, 1);

    /* Body = LEN + DATA with DLE stuffing; BCC covers the stuffed bytes. */
    for (i = 0; i <= len; i++) {
        uint8_t v = (i == 0) ? (uint8_t)len : data[i - 1];

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

static void micom_arm(ClarionMicomState *s, int64_t delay)
{
    timer_mod(s->tx, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + delay);
}

static void micom_queue(ClarionMicomState *s, const uint8_t *data, unsigned len)
{
    g_queue_push_tail(s->frames, micom_build(data, len));
    if (!s->cur) {
        micom_arm(s, MICOM_FILL_NS);
    }
}

static void micom_queue_raw2(ClarionMicomState *s, uint8_t a, uint8_t b)
{
    GByteArray *f = g_byte_array_new();

    g_byte_array_append(f, &a, 1);
    g_byte_array_append(f, &b, 1);
    g_queue_push_head(s->frames, f);        /* ACK precedes queued frames */
    if (!s->cur) {
        micom_arm(s, MICOM_FILL_NS);
    }
}

/* --- deliver bytes to SCIF --------------------------------------------- */

static void micom_tx_expire(void *opaque)
{
    ClarionMicomState *s = opaque;
    int took;

    if (!s->cur) {
        s->cur = g_queue_pop_head(s->frames);
        s->cur_off = 0;
        if (!s->cur) {
            return;
        }
    }
    if (!s->sink) {
        return;
    }
    took = s->sink(s->sink_opaque, s->cur->data + s->cur_off,
                   s->cur->len - s->cur_off);
    if (took > 0) {
        s->cur_off += took;
    }
    if (s->cur_off >= s->cur->len) {
        g_byte_array_free(s->cur, TRUE);
        s->cur = NULL;
        s->cur_off = 0;
        if (!g_queue_is_empty(s->frames)) {
            micom_arm(s, MICOM_GAP_NS);     /* inter-frame gap */
        }
    } else {
        micom_arm(s, MICOM_FILL_NS);        /* FIFO full; refill later */
    }
}

/* --- process a frame from the kernel ----------------------------------- */

/*
 * Command set announced by micom in frame 08 0a. The first list byte is the
 * subcode 0x0a itself: RegisterSupported @0xef78bc2c reads DATA+1 for N-1
 * bytes, so the 0x0a flag is set as well. Byte 0xff marks an error and stops
 * parsing, so the list uses values 0x01..0x3e.
 */
static void micom_send_cmd_set(ClarionMicomState *s)
{
    uint8_t data[2 + 0x3e];
    unsigned i;

    data[0] = 0x08;
    data[1] = 0x0a;
    for (i = 0; i < 0x3e; i++) {
        data[2 + i] = (uint8_t)(0x01 + i);
    }
    micom_queue(s, data, 2 + 0x3e);
}

/* CEdaCtl init @0xef787888: subscribes to async family 0x3a; no response
 * is required (the sender creates no waiter). */
static const uint8_t cmd_60_3a[] = { 0x60, 0x0d, 0x3a, 0, 0, 0, 0xe3 };
/* CEdaCtl init @0xef787930: RX handler @0xef789ec0 requires DATA[0] = 0x10,
 * DATA[1] = 0x3f, with status in DATA[5]; 0 means success. */
static const uint8_t cmd_60_3f[] = { 0x60, 0x0d, 0x3f, 0, 0, 0, 0x01 };
static const uint8_t rsp_10_3f[] = { 0x10, 0x3f, 0, 0, 0, 0 };

/*
 * QY7221NL: CeNet "audio mode off" notification, as read from anwdrv.dll
 * (CModeMgr, QY7221NL image). Decoded message, DATA[0..]:
 *   [0] = 0x40  CeNet notification family (handler vt+0x378 -> 0x37c)
 *   [2] = 0x71  "system information" message class
 *   [6] = 0x01  sub-type: audio mode off ("ANWDRV >> Audio Mode off.")
 * The driver then reports LA=0xff, on/off=0, intr LA=0xff through
 * IOCTL_CEN_GET_LATEST_SYS_MODE; without any such message the driver keeps
 * the initial all-0xff buffer and SystemCC never starts the audio resource
 * switch. SYNTHETIC: the real audio unit may report a different state.
 */
static const uint8_t cenet_audio_off[] = {
    0x40, 0x00, 0x71, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00
};

/*
 * QY7 antitheft release (SYNTHETIC). Two CeNet frames, in this order; both
 * are decoded by anwdrv.dll (the NOR build, runtime base 0x03E00000):
 *
 * 1. DATA[1] == 0 selects the Mode module. 0x03E1348C (DATA[0] == 0x40) calls
 *    the Mode handler 0x03E134D4 which, for DATA[2] == 0x0D and DATA[6] in
 *    {0x20, 0x21}, calls SetEvent() (vtbl+16 = 0x03E24C24) on the
 *    ANT_RCV_SECURITY_RELEASE_OK handle (this+100) and then on
 *    TMC_RCV_POWER_ON (this+104).
 * 2. The Antitheft module (classifier 0x03E1FA74: DATA[1] == 0, DATA[0] ==
 *    0x60, DATA[6] == 0x85) calls SetEvent() on the ANT_RCV_SECURITY_LOCK
 *    handle (this+32) in 0x03E23768; DATA[8] is stored at this+56.
 *
 * Antitheft.exe only acts on the RELEASE_OK event after it has seen LOCK
 * (0x18330 != 0 in its 0x14D64), so LOCK goes first. Which value of
 * DATA[6] (0x20/0x21) and DATA[8] the real micom sends is UNKNOWN.
 */
static const uint8_t cenet_antitheft_lock[] = {
    0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x85, 0x00, 0x00, 0x00
};
static const uint8_t cenet_antitheft_release[] = {
    0x40, 0x00, 0x0d, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00
};

static void micom_frame(ClarionMicomState *s)
{
    const uint8_t *data = s->body + 1;
    unsigned len = s->body_len ? s->body_len - 1 : 0;
    uint8_t cmd, sub, ack[2];
    const char *action = "ack-only";

    if (s->body_len && s->body[0] != len) {
        qemu_log_mask(LOG_GUEST_ERROR, "clarion-micom: LEN %u does not match "
                      "DATA length %u\n", s->body[0], len);
    }
    /* Send a DLE ACK for every frame, as on the real link. */
    ack[0] = DLE;
    ack[1] = ACK;
    micom_queue_raw2(s, ack[0], ack[1]);

    if (!len) {
        return;
    }
    cmd = data[0];
    sub = (len > 1) ? data[1] : 0;

    if (cmd == 0x08 && sub == 0x00) {           /* reset -> states 2, 3 */
        const uint8_t alive[] = { 0x08, 0x01 };

        action = "alive+command-set";
        s->handshake_done = false;
        micom_queue(s, alive, sizeof(alive));
        micom_send_cmd_set(s);
    } else if (cmd == 0x08 && sub == 0x0b) {    /* command set accepted */
        s->handshake_done = true;
        s->handshake_count++;
        if (s->cenet_audio_off) {
            action = "ack-only+cenet-audio-off";
            micom_queue(s, cenet_audio_off, sizeof(cenet_audio_off));
        }
        if (s->cenet_antitheft_release &&
            !s->cenet_antitheft_release_sent && s->handshake_count > 1) {
            action = "ack-only+cenet-antitheft-release";
            micom_queue(s, cenet_antitheft_lock,
                        sizeof(cenet_antitheft_lock));
            micom_queue(s, cenet_antitheft_release,
                        sizeof(cenet_antitheft_release));
            s->cenet_antitheft_release_sent = true;
        }
    } else if (len == sizeof(cmd_60_3a) &&
               !memcmp(data, cmd_60_3a, sizeof(cmd_60_3a))) {
        /* no response required */
        action = "ack-only-async";
    } else if (len == sizeof(cmd_60_3f) &&
               !memcmp(data, cmd_60_3f, sizeof(cmd_60_3f))) {
        action = "reply-10-3f";
        micom_queue(s, rsp_10_3f, sizeof(rsp_10_3f));
    } else if (s->generic_reply) {
        /* QY8 fallback; QY7 disables synthetic replies by default. */
        const uint8_t recv[2] = { cmd, s->generic_reply_subcode };

        action = "generic-reply";
        micom_queue(s, recv, sizeof(recv));
    }

    qemu_log_mask(LOG_GUEST_ERROR,
                  "clarion-micom: rx virtual-ms=%" PRId64
                  " len=%u cmd=%02x sub=%02x action=%s\n",
                  qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL), len, cmd, sub,
                  action);
    if (cmd != 0x08) {
        /* Payload bytes of non-handshake frames (no secrets pass here). */
        char hex[3 * 32 + 1];
        unsigned i;

        for (i = 0; i < len && i < 32; i++) {
            snprintf(hex + 3 * i, 4, "%02x ", data[i]);
        }
        hex[3 * i] = 0;
        qemu_log_mask(LOG_GUEST_ERROR, "clarion-micom: data %s\n", hex);
    }
}

/* --- parse the stream from the kernel --------------------------------- */

void clarion_micom_rx_byte(DeviceState *dev, uint8_t b)
{
    ClarionMicomState *s = CLARION_MICOM(dev);

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
        } else if (b == DLE) {
            s->rx_state = RX_DLE;           /* 10 10 outside a frame; stay */
        } else {
            /* DLE ACK / DLE NAK from the kernel acknowledge our frames */
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
        if (b == DLE) {                     /* stuffed literal 0x10 */
            if (s->body_len < sizeof(s->body)) {
                s->body[s->body_len++] = DLE;
            }
            s->rx_state = RX_BODY;
        } else if (b == ETX) {
            s->rx_state = RX_BCC;
        } else {
            s->rx_state = RX_IDLE;          /* malformed frame; discard */
        }
        break;

    case RX_BCC:
        if (b != s->bcc) {
            qemu_log_mask(LOG_GUEST_ERROR, "clarion-micom: BCC 0x%x, expected "
                          "0x%x; processing frame anyway\n", b, s->bcc);
        }
        micom_frame(s);
        s->rx_state = RX_IDLE;
        break;
    }
}

void clarion_micom_set_sink(DeviceState *dev, ClarionMicomSink fn, void *opaque)
{
    ClarionMicomState *s = CLARION_MICOM(dev);

    s->sink = fn;
    s->sink_opaque = opaque;
}

/* --- QOM --------------------------------------------------------------- */

static void clarion_micom_reset_hold(Object *obj, ResetType type)
{
    ClarionMicomState *s = CLARION_MICOM(obj);

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
    s->handshake_done = false;
    s->handshake_count = 0;
    s->cenet_antitheft_release_sent = false;
}

static void clarion_micom_realize(DeviceState *dev, Error **errp)
{
    ClarionMicomState *s = CLARION_MICOM(dev);

    s->frames = g_queue_new();
    s->tx = timer_new_ns(QEMU_CLOCK_VIRTUAL, micom_tx_expire, s);
}

static const Property clarion_micom_props[] = {
    DEFINE_PROP_BOOL("generic-reply", ClarionMicomState,
                     generic_reply, true),
    DEFINE_PROP_UINT8("generic-reply-subcode", ClarionMicomState,
                      generic_reply_subcode, 0),
    DEFINE_PROP_BOOL("cenet-audio-off", ClarionMicomState,
                     cenet_audio_off, false),
    DEFINE_PROP_BOOL("cenet-antitheft-release", ClarionMicomState,
                     cenet_antitheft_release, false),
};

static void clarion_micom_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = clarion_micom_realize;
    dc->desc = "Clarion QY8XXX companion micom (SCIF4 link)";
    device_class_set_props(dc, clarion_micom_props);
    rc->phases.hold = clarion_micom_reset_hold;
}

static const TypeInfo clarion_micom_types[] = {
    {
        .name = TYPE_CLARION_MICOM,
        .parent = TYPE_DEVICE,
        .instance_size = sizeof(ClarionMicomState),
        .class_init = clarion_micom_class_init,
    },
};

DEFINE_TYPES(clarion_micom_types)
