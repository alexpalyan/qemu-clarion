/*
 * Display Micom, the Clarion QY8XXX panel microcontroller.
 *
 * On the real board, this is a separate chip in the display unit, connected
 * to the main SoC over SCIF1. It is distinct from the board's companion MCU
 * on SCIF4 (hw/misc/clarion_micom.c): the link, guest driver, and frame body
 * format differ. Without a panel response, `lcddrv.dll` never signals ACM
 * module 129, and DrawOi renders its first frame only after the wait times out.
 *
 * The protocol was derived from lcddrv.dll, not documentation.
 *
 * Framing (receiver CLcdDrv::StartReadData @0xEF717578, transmitter
 * CLcdDrv_Send @0xEF71A5D0):
 *
 *     10 02 | DATA... (stuffing: 10 -> 10 10) | 10 03 | BCC
 *     BCC = XOR of all bytes AFTER 10 02, including trailing 10 03,
 *     computed over stuffed bytes (eor/uxtb @0xEF7177B8, @0xEF71A65C)
 *
 * ⚠ Unlike the board link, there is NO LEN byte here: the body starts with
 * the command code. Verified against a real guest frame captured from
 * WriteFile: 10 02 23 10 03 30, XOR(23 10 03) = 0x30 ✔
 *
 * ACKs are two bytes outside the frame: DLE ACK = 10 06, DLE NAK = 10 15
 * (StartSendData_AckNack @0xEF71A4A0 stores 0x10 at [this+0x552] and the code
 * at [this+0x553], then calls WriteFile for 2 bytes). The guest receiver
 * StartReadData @0xEF7176D8.
 *
 * Startup sequence (CLcdDrv::RcvAnalysisData @0xEF718554, dispatcher for
 * DATA[0]):
 *
 *     HU  -> 23                 panel information request (link probe)
 *     HU  <- 10 06              ACK
 *     HU  <- 24 VV UU P[12]     version, UnitID, PartNum
 *                               handler @0xEF7185B0 stores DATA[1] at
 *                               [this+0x374], DATA[2] at [this+0x375],
 *                               and DATA[3..14] at [this+0x376],
 *                               then Event(this, 1) -> link state 0
 *                               -> CLcdDrv_Send::LinkStart @0xEF71A6B0:
 *                                  m_LinkState := 1 and queues frame 0x25
 *     HU  -> 25                 link up
 *     HU  <- 10 06              ACK
 *     HU  <- 40                 handler @0xEF718674: if m_LinkState == 1,
 *                               sets it to 2 and calls
 *                               CApi_ModuleInitializeComplete(129)
 *
 * Framing, ACKs and startup replies follow the guest lcddrv code. Ready
 * 0x40 reads no payload at ARM 0xef718674. Version/UnitID/PartNum are
 * SYNTHETIC: the ARM guest stores and prints them at 0xef718694,
 * 0xef7186a0 and 0xef7186ac..0xef7186d4 without comparing their values.
 *
 * Payload-free control completions are also supported: keepalive 0x71
 * (SH4 0x03cf6874, ARM 0xef718764), settings ready 0x42 (SH4 0x03cf677c)
 * and settings report completion 0x43 (SH4 0x03cf679e). Their receive
 * handlers read no response payload. Unsupported commands receive ACK.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/misc/clarion_dispmicom.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "standard-headers/linux/input-event-codes.h"
#include "ui/input.h"
#include "trace.h"

#define DLE     0x10
#define STX     0x02
#define ETX     0x03
#define ACK     0x06
#define NAK     0x15

#define DISP_MAX_DATA       64
#define DISP_ACK_TIMEOUT_NS (500 * 1000 * 1000)
#define DISP_ACK_RETRIES    3

typedef struct DispFrame {
    GByteArray *bytes;
    bool wait_ack;
} DispFrame;

/* Command codes decoded from lcddrv.dll. */
#define CMD_INFO_REQ        0x23    /* HU -> panel: information request */
#define CMD_INFO_RSP        0x24    /* panel -> HU: version/UnitID/PartNum */
#define CMD_LINK_UP         0x25    /* HU -> panel: link up */
#define CMD_READY           0x40    /* panel -> HU: ready; guest signals 129 */
#define CMD_SETTINGS        0x42
#define CMD_REPORT          0x43
#define CMD_KEEPALIVE       0x71
#define CMD_TOUCH           0x51
#define CMD_PANEL_KEY       0x34
/* kepdrv InputLcdKey (QY8605NB @0xEF880F1C): DATA[1] = matrix code, bit 7 = pressed */
#define CMD_KEY             CMD_PANEL_KEY
#define KEY_PRESSED         0x80
#define CMD_ROTARY          0x35
/* VOL push (power): kepdrv takes a bare 37 as key id f0, VK 05, pressed */
#define CMD_POWER           0x37

/*
 * Inter-frame gap, as in the board MCU model: the SCIF receiver has a 16-byte
 * FIFO, and the driver must drain the previous frame. Virtual time keeps boot
 * independent of host load.
 */
#define DISP_GAP_NS         (20 * 1000 * 1000)
/* Refill the rest of the frame as the driver frees the FIFO. */
#define DISP_FILL_NS        (1 * 1000 * 1000)

/*
 * SYNTHETIC values for frame 0x24 fields. The guest does not validate them;
 * it only prints them. These values make the model identifiable in logs:
 * Version 0x01, UnitID 0x01, PartNum "QEMUDISP001" (11 ASCII bytes, matching
 * print format @0xEF7141F8), followed by a twelfth byte that the guest copies
 * but never reads: 0x00.
 */
#define DISP_VERSION        0x01
#define DISP_UNIT_ID        0x01
static const char disp_partnum[11] = "QEMUDISP001";

typedef enum {
    RX_IDLE,            /* waiting for DLE */
    RX_DLE,             /* saw DLE outside a frame */
    RX_BODY,            /* inside a frame */
    RX_BODY_DLE,        /* inside a frame, saw DLE */
    RX_BCC,             /* next byte is the checksum */
} DispRxState;

struct ClarionDispMicomState {
    DeviceState parent_obj;

    ClarionDispMicomSink sink;
    void *sink_opaque;

    QEMUTimer *tx;
    GQueue *frames;             /* queued panel frames */
    DispFrame *cur;             /* frame currently being sent */
    DispFrame *awaiting_ack;    /* sent key frame awaiting DLE ACK */
    unsigned cur_off;           /* number of bytes already accepted */
    unsigned ack_retries;

    DispRxState rx_state;
    uint8_t body[DISP_MAX_DATA];
    unsigned body_len;
    uint8_t bcc;

    bool linked;                /* saw 0x25 from the guest */
    bool touch_input;
    QemuInputHandlerState *input_handler;
    uint16_t touch_x;
    uint16_t touch_y;
    bool touch_down;
    bool touch_dirty;
    bool key_input;
    bool can_key_input;
    QemuInputHandlerState *key_input_handler;
    uint32_t key_down_mask;
    ClarionDispMicomCanKey can_key;
    void *can_key_opaque;

    QEMUTimer *key_up;          /* release of a key injected from the monitor */
    uint8_t key_held;

    bool panel_jpn;             /* code table: false = row (0x10180201), true = jpn (0x10180202) */
};

static void disp_frame_free(DispFrame *frame)
{
    if (frame) {
        g_byte_array_free(frame->bytes, TRUE);
        g_free(frame);
    }
}

/* --- frame construction ------------------------------------------------ */

static GByteArray *disp_build(const uint8_t *data, unsigned len)
{
    GByteArray *f = g_byte_array_new();
    uint8_t b, bcc = 0;
    unsigned i;

    b = DLE; g_byte_array_append(f, &b, 1);
    b = STX; g_byte_array_append(f, &b, 1);

    /* Body is DATA (no LEN); DLE stuffing, BCC over stuffed bytes. */
    for (i = 0; i < len; i++) {
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

static void disp_arm(ClarionDispMicomState *s, int64_t delay)
{
    timer_mod(s->tx, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + delay);
}

static void disp_trace(const char *direction, const uint8_t *data,
                       unsigned len)
{
    unsigned i;
    g_autoptr(GString) body = g_string_new(NULL);

    for (i = 0; i < len; i++) {
        g_string_append_printf(body, "%s%02x", i ? " " : "", data[i]);
    }
    trace_clarion_dispmicom_frame(qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL),
                                  direction, body->str);
}

static void disp_queue(ClarionDispMicomState *s, const uint8_t *data,
                       unsigned len)
{
    DispFrame *frame = g_new0(DispFrame, 1);

    frame->bytes = disp_build(data, len);
    g_queue_push_tail(s->frames, frame);
    if (!s->cur && !s->awaiting_ack) {
        disp_arm(s, DISP_FILL_NS);
    }
}

static void disp_queue_key(ClarionDispMicomState *s, const uint8_t *data,
                           unsigned len)
{
    DispFrame *frame = g_new0(DispFrame, 1);

    frame->bytes = disp_build(data, len);
    frame->wait_ack = true;
    g_queue_push_tail(s->frames, frame);
    if (!s->cur && !s->awaiting_ack) {
        disp_arm(s, DISP_FILL_NS);
    }
}

static void disp_queue_ack(ClarionDispMicomState *s, uint8_t code)
{
    DispFrame *frame = g_new0(DispFrame, 1);
    uint8_t b = DLE;

    frame->bytes = g_byte_array_new();
    g_byte_array_append(frame->bytes, &b, 1);
    g_byte_array_append(frame->bytes, &code, 1);
    g_queue_push_head(s->frames, frame);
    if (!s->cur && !s->awaiting_ack) {
        disp_arm(s, DISP_FILL_NS);
    }
}

/* --- deliver bytes to SCIF --------------------------------------------- */

static void disp_tx_expire(void *opaque)
{
    ClarionDispMicomState *s = opaque;
    int took;

    if (!s->cur) {
        if (s->awaiting_ack) {
            if (s->ack_retries >= DISP_ACK_RETRIES) {
                trace_clarion_dispmicom_key_ack_timeout(s->ack_retries);
                disp_frame_free(s->awaiting_ack);
                s->awaiting_ack = NULL;
                s->ack_retries = 0;
                if (!g_queue_is_empty(s->frames)) {
                    disp_arm(s, DISP_GAP_NS);
                }
                return;
            }
            s->cur = s->awaiting_ack;
            s->awaiting_ack = NULL;
            s->cur_off = 0;
            s->ack_retries++;
            trace_clarion_dispmicom_key_ack_retry(s->ack_retries);
        } else {
            s->cur = g_queue_pop_head(s->frames);
        }
        s->cur_off = 0;
        if (!s->cur) {
            return;
        }
    }
    if (!s->sink) {
        return;
    }
    took = s->sink(s->sink_opaque, s->cur->bytes->data + s->cur_off,
                   s->cur->bytes->len - s->cur_off);
    if (took > 0) {
        if (!s->cur_off) {
            if (s->cur->bytes->len == 2) {
                disp_trace("panel-tx", s->cur->bytes->data, 2);
            } else {
                uint8_t data[DISP_MAX_DATA];
                unsigned len = 0;
                unsigned i;

                for (i = 2; i < s->cur->bytes->len - 3; i++) {
                    data[len++] = s->cur->bytes->data[i];
                    if (s->cur->bytes->data[i] == DLE) {
                        i++;
                    }
                }
                disp_trace("panel-tx", data, len);
            }
        }
        s->cur_off += took;
    }
    if (s->cur_off >= s->cur->bytes->len) {
        if (s->cur->wait_ack) {
            s->awaiting_ack = s->cur;
            s->cur = NULL;
            s->cur_off = 0;
            disp_arm(s, DISP_ACK_TIMEOUT_NS);
            return;
        }
        disp_frame_free(s->cur);
        s->cur = NULL;
        s->cur_off = 0;
        if (!g_queue_is_empty(s->frames)) {
            disp_arm(s, DISP_GAP_NS);       /* inter-frame gap */
        }
    } else {
        disp_arm(s, DISP_FILL_NS);          /* FIFO full; refill later */
    }
}

static void disp_guest_ack(ClarionDispMicomState *s, uint8_t code)
{
    if (!s->awaiting_ack) {
        return;
    }
    timer_del(s->tx);
    if (code == ACK) {
        trace_clarion_dispmicom_key_ack_accepted(s->ack_retries);
        disp_frame_free(s->awaiting_ack);
        s->awaiting_ack = NULL;
        s->ack_retries = 0;
        if (!g_queue_is_empty(s->frames)) {
            disp_arm(s, DISP_GAP_NS);
        }
    } else {
        trace_clarion_dispmicom_key_nak(s->ack_retries);
        if (s->ack_retries >= DISP_ACK_RETRIES) {
            disp_frame_free(s->awaiting_ack);
            s->awaiting_ack = NULL;
            s->ack_retries = 0;
            if (!g_queue_is_empty(s->frames)) {
                disp_arm(s, DISP_GAP_NS);
            }
        } else {
            s->cur = s->awaiting_ack;
            s->awaiting_ack = NULL;
            s->cur_off = 0;
            s->ack_retries++;
            disp_arm(s, DISP_FILL_NS);
        }
    }
}

/* --- process a frame from the guest ------------------------------------ */

static void disp_send_info(ClarionDispMicomState *s)
{
    uint8_t data[3 + 12];

    data[0] = CMD_INFO_RSP;
    data[1] = DISP_VERSION;
    data[2] = DISP_UNIT_ID;
    memcpy(&data[3], disp_partnum, sizeof(disp_partnum));
    data[3 + sizeof(disp_partnum)] = 0x00;  /* guest copies but never reads */
    disp_queue(s, data, sizeof(data));
}

static void disp_frame(ClarionDispMicomState *s)
{
    uint8_t cmd;

    disp_trace("guest-tx", s->body, s->body_len);

    /* Send a DLE ACK for every frame, as on the real link. */
    disp_queue_ack(s, ACK);

    if (!s->body_len) {
        return;
    }
    cmd = s->body[0];

    switch (cmd) {
    case CMD_INFO_REQ:
        s->linked = false;
        disp_send_info(s);
        break;

    case CMD_LINK_UP:
        s->linked = true;
        disp_queue(s, (const uint8_t[]){ CMD_READY }, 1);
        break;

    case CMD_KEEPALIVE:
        /*
         * Keepalive completion: SH4 handler 0x03cf6874 and ARM handler
         * 0xef718764 clear the outstanding flag and cancel timer 2.
         * Neither handler reads a payload byte.
         */
        disp_queue(s, (const uint8_t[]){ CMD_KEEPALIVE }, 1);
        break;

    case CMD_SETTINGS:
        /*
         * SH4 handler 0x03cf677c accepts ready 0x42 in link state 1,
         * reads no payload, and queues the 0x43 settings report.
         */
        disp_queue(s, (const uint8_t[]){ CMD_SETTINGS }, 1);
        break;

    case CMD_REPORT:
        /*
         * SH4 handler 0x03cf679e changes link state 1 to 2. It logs
         * the previously saved panel identity, not a response payload.
         */
        disp_queue(s, (const uint8_t[]){ CMD_REPORT }, 1);
        break;

    default:
        /* Other panel commands are undecoded; ACK only, without guessing. */
        break;
    }
}

/* --- synthetic QY7 touch input ---------------------------------------- */

static void disp_touch_event(DeviceState *dev, QemuConsole *src,
                             QemuInputEvent *evt)
{
    ClarionDispMicomState *s = CLARION_DISPMICOM(dev);

    (void)src;

    switch (evt->type) {
    case INPUT_EVENT_KIND_ABS:
        if (evt->abs.axis == INPUT_AXIS_X) {
            s->touch_x =
                qemu_input_scale_axis(evt->abs.value, INPUT_EVENT_ABS_MIN,
                                      INPUT_EVENT_ABS_MAX, 0, 800);
            s->touch_dirty = true;
        } else if (evt->abs.axis == INPUT_AXIS_Y) {
            s->touch_y = 480 -
                qemu_input_scale_axis(evt->abs.value, INPUT_EVENT_ABS_MIN,
                                      INPUT_EVENT_ABS_MAX, 0, 479);
            s->touch_dirty = true;
        }
        break;
    case INPUT_EVENT_KIND_BTN:
        if (evt->btn.button == INPUT_BUTTON_LEFT) {
            s->touch_down = evt->btn.down;
            s->touch_dirty = true;
        }
        break;
    default:
        break;
    }
}

static void disp_touch_sync(DeviceState *dev)
{
    ClarionDispMicomState *s = CLARION_DISPMICOM(dev);
    uint8_t data[] = {
        CMD_TOUCH,         s->touch_x >> 8, s->touch_x & 0xff,
        s->touch_y >> 8, s->touch_y & 0xff,   s->touch_down ? '1' : '0',
    };

    if (!s->touch_dirty) {
        return;
    }
    s->touch_dirty = false;
    trace_clarion_dispmicom_synthetic_touch(
        qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL), s->touch_x, s->touch_y,
        s->touch_down);
    disp_trace("synthetic-touch", data + 1, sizeof(data) - 1);
    disp_queue(s, data, sizeof(data));
}

static const QemuInputHandler disp_touch_handler = {
    .name = "qy7-touch",
    .mask = INPUT_EVENT_MASK_ABS | INPUT_EVENT_MASK_BTN,
    .event = disp_touch_event,
    .sync = disp_touch_sync,
};


static void disp_key_up(void *opaque)
{
    ClarionDispMicomState *s = opaque;

    disp_queue(s, (const uint8_t[]){ CMD_KEY, s->key_held }, 2);
}

/* qom-set key="CODE[,HOLD_MS]": press a panel key by its matrix code */
static void disp_set_key(Object *obj, const char *value, Error **errp)
{
    ClarionDispMicomState *s = CLARION_DISPMICOM(obj);
    unsigned long code, hold = 150;
    char *end;

    code = strtoul(value, &end, 16);
    if (end != value && *end == ',') {
        hold = strtoul(end + 1, &end, 10);
    }
    if (end == value || code > 0x7f || *end) {
        error_setg(errp, "key: expected CODE[,HOLD_MS], CODE hex 00..7f");
        return;
    }
    if (timer_pending(s->key_up)) {
        timer_del(s->key_up);
        disp_key_up(s);
    }
    s->key_held = code;
    disp_queue(s, (const uint8_t[]){ CMD_KEY, code | KEY_PRESSED }, 2);
    timer_mod(s->key_up, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              (int64_t)hold * 1000000);
}

/* qom-set frame="34 05": raw panel frame, for experiments with other commands */
static void disp_set_frame(Object *obj, const char *value, Error **errp)
{
    ClarionDispMicomState *s = CLARION_DISPMICOM(obj);
    uint8_t data[DISP_MAX_DATA];
    unsigned len = 0;
    const char *p = value;
    char *end;

    while (*p) {
        unsigned long v = strtoul(p, &end, 16);

        if (end == p || v > 0xff || len == sizeof(data)) {
            error_setg(errp, "frame: expected hex bytes");
            return;
        }
        data[len++] = v;
        p = end + strspn(end, " ,:");
    }
    if (len) {
        disp_queue(s, data, len);
    }
}

/* sign and magnitude: 0x81 is one step back, 0xff is 127 back */
static uint8_t disp_detents(int n)
{
    return n < 0 ? 0x80 | MIN(-n, 0x7f) : MIN(n, 0x7f);
}

/* kepdrv swaps the bytes by panel: jpn sends 35 VOL TUNE, row 35 TUNE VOL */
static void disp_rotary(ClarionDispMicomState *s, int vol, int tune)
{
    uint8_t v = disp_detents(vol), t = disp_detents(tune);
    uint8_t a = s->panel_jpn ? v : t, b = s->panel_jpn ? t : v;

    disp_queue(s, (const uint8_t[]){ CMD_ROTARY, a, b }, 3);
}

/* the touch controller owns the pointer buttons, so the wheel arrives here */
static ClarionDispMicomState *disp_panel;

void clarion_dispmicom_wheel(int detents)
{
    if (disp_panel && detents) {
        /* wheel up moves the selection up, like PageUp */
        disp_rotary(disp_panel, 0, -detents);
    }
}


/*
 * Host keyboard -> panel keys. One table for every unit, so a host key presses
 * the same function on QY7 and QY8. Each row carries the QY7 code (panel frame
 * 0x34, or MCAN PUSH_SW when qy7_can) and the QY8 codes for the row and jpn
 * panels, from the kepdrv InputLcdKey tables. NO_CODE: no proven code on that
 * unit. F1..F9 are the original QY8 aliases.
 */
#define NO_CODE (-1)

typedef struct DispKeyMap {
    unsigned int host;          /* Linux input code */
    int qy7;
    bool qy7_can;
    int qy8_row;
    int qy8_jpn;
} DispKeyMap;

static const DispKeyMap disp_key_map[] = {
    /* host            QY7      MCAN   QY8 row  QY8 jpn */
    { KEY_M,           0x0c,    true,  0x00,    0x32 },     /* MAP/VOICE */
    { KEY_N,           0x44,    true,  0x13,    0x01 },     /* MENU */
    { KEY_T,           0x31,    true,  NO_CODE, NO_CODE },  /* STATUS */
    { KEY_Z,           0x4c,    true,  NO_CODE, NO_CODE },  /* ZERO EMISSION */
    { KEY_O,           0xf0,    true,  NO_CODE, NO_CODE },  /* POWER */
    { KEY_D,           0x01,    false, NO_CODE, NO_CODE },  /* DAY/NIGHT */
    { KEY_B,           0x21,    false, 0x20,    0x31 },     /* brightness / display off */
    { KEY_ESC,         NO_CODE, false, 0x30,    0x03 },     /* BACK */
    { KEY_ENTER,       NO_CODE, false, 0x21,    0x10 },     /* TUNE press */
    { KEY_KPENTER,     NO_CODE, false, 0x21,    0x10 },     /* TUNE press */
    { KEY_C,           NO_CODE, false, 0x01,    0x13 },     /* CAMERA */
    { KEY_A,           NO_CODE, false, 0x03,    0x30 },     /* AUDIO */
    { KEY_LEFTBRACE,   NO_CODE, false, 0x31,    0x00 },     /* SEEK back */
    { KEY_RIGHTBRACE,  NO_CODE, false, 0x32,    0x20 },     /* SEEK forward */
    { KEY_F1,          NO_CODE, false, 0x13,    0x01 },     /* MENU */
    { KEY_F2,          NO_CODE, false, 0x30,    0x03 },     /* BACK */
    { KEY_F3,          NO_CODE, false, 0x20,    0x31 },     /* display off */
    { KEY_F4,          NO_CODE, false, 0x00,    0x32 },     /* MAP */
    { KEY_F5,          NO_CODE, false, 0x01,    0x13 },     /* CAMERA */
    { KEY_F6,          NO_CODE, false, 0x03,    0x30 },     /* AUDIO */
    { KEY_F7,          NO_CODE, false, 0x31,    0x00 },     /* SEEK back */
    { KEY_F8,          NO_CODE, false, 0x32,    0x20 },     /* SEEK forward */
    { KEY_F9,          NO_CODE, false, 0x21,    0x10 },     /* TUNE press */
};

/*
 * QY8 knobs from the host keyboard: Up/Down (or PageUp/PageDown) turn TUNE,
 * =/- turn VOL. Arrows because laptop keyboards have no PageUp/PageDown.
 */
static bool disp_knob_key(ClarionDispMicomState *s, QemuInputEvent *evt)
{
    switch (evt->key.key) {
    case KEY_UP:
    case KEY_DOWN:
    case KEY_PAGEUP:
    case KEY_PAGEDOWN:
        if (evt->key.down) {
            /* Up moves the selection up, i.e. one step back */
            disp_rotary(s, 0, (evt->key.key == KEY_UP ||
                               evt->key.key == KEY_PAGEUP) ? -1 : 1);
        }
        return true;
    case KEY_EQUAL:
    case KEY_MINUS:
        if (evt->key.down) {
            disp_rotary(s, evt->key.key == KEY_EQUAL ? 1 : -1, 0);
        }
        return true;
    case KEY_O:
        if (evt->key.down) {
            disp_queue(s, (const uint8_t[]){ CMD_POWER }, 1);
        }
        return true;
    }
    return false;
}

/*
 * key-input=on selects the QY7 codes: panel keys wait for the guest's DLE ACK,
 * MCAN keys need can-key-input=on. Otherwise the QY8 codes for panel=row|jpn.
 */
static void disp_key_event(DeviceState *dev, QemuConsole *src,
                           QemuInputEvent *evt)
{
    ClarionDispMicomState *s = CLARION_DISPMICOM(dev);
    const DispKeyMap *k = NULL;
    uint8_t data[2] = { CMD_PANEL_KEY, 0 };
    uint32_t bit;
    unsigned int i;
    int code;

    (void)src;
    if (evt->type != INPUT_EVENT_KIND_KEY) {
        return;
    }
    if (!s->key_input && disp_knob_key(s, evt)) {
        return;
    }
    for (i = 0; i < ARRAY_SIZE(disp_key_map); i++) {
        if (evt->key.key == disp_key_map[i].host) {
            k = &disp_key_map[i];
            break;
        }
    }
    if (!k) {
        return;
    }
    if (s->key_input) {
        code = k->qy7;
        if (code == NO_CODE || (k->qy7_can && !s->can_key_input)) {
            return;
        }
    } else {
        code = s->panel_jpn ? k->qy8_jpn : k->qy8_row;
        if (code == NO_CODE) {
            return;
        }
    }

    bit = 1U << i;
    if (evt->key.down) {
        if (s->key_down_mask & bit) {
            trace_clarion_dispmicom_panel_key_repeat_suppressed(code);
            return;
        }
        s->key_down_mask |= bit;
    } else {
        if (!(s->key_down_mask & bit)) {
            return;
        }
        s->key_down_mask &= ~bit;
    }

    if (s->key_input && k->qy7_can) {
        if (s->can_key) {
            s->can_key(s->can_key_opaque, code, evt->key.down);
        }
        return;
    }
    data[1] = code | (evt->key.down ? KEY_PRESSED : 0);
    trace_clarion_dispmicom_panel_key(code, evt->key.down);
    if (s->key_input) {
        disp_queue_key(s, data, sizeof(data));
    } else {
        disp_queue(s, data, sizeof(data));
    }
}

static const QemuInputHandler disp_key_handler = {
    .name  = "Clarion panel keys",
    .mask  = INPUT_EVENT_MASK_KEY,
    .event = disp_key_event,
};

/* --- parse the stream from the guest ----------------------------------- */

void clarion_dispmicom_rx_byte(DeviceState *dev, uint8_t b)
{
    ClarionDispMicomState *s = CLARION_DISPMICOM(dev);

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
            if (b == ACK || b == NAK) {
                disp_trace("guest-tx", (const uint8_t[]){ DLE, b }, 2);
                disp_guest_ack(s, b);
            }
            /* DLE ACK / DLE NAK from guest acknowledge our frames */
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
            qemu_log_mask(LOG_GUEST_ERROR, "clarion-dispmicom: BCC %#x, "
                          "expected %#x; frame discarded\n", b, s->bcc);
            disp_queue_ack(s, NAK);
        } else {
            disp_frame(s);
        }
        s->rx_state = RX_IDLE;
        break;
    }
}

static void disp_set_key_raw(Object *obj, const char *value, Error **errp)
{
    ClarionDispMicomState *s = CLARION_DISPMICOM(obj);
    uint8_t data[DISP_MAX_DATA];
    const char *p = value;
    unsigned len = 0;

    if (!s->key_input) {
        error_setg(errp, "raw panel keys are enabled only on QY7");
        return;
    }
    while (*p) {
        char *end;
        unsigned long byte;

        while (g_ascii_isspace(*p) || *p == ',' || *p == ':') {
            p++;
        }
        if (!*p) {
            break;
        }
        if (!g_ascii_isxdigit(*p) || len == DISP_MAX_DATA) {
            error_setg(errp, "key-raw expects 1 to %u hex bytes",
                       DISP_MAX_DATA);
            return;
        }
        errno = 0;
        byte = g_ascii_strtoull(p, &end, 16);
        if (errno || end == p || byte > 0xff ||
            (*end && !g_ascii_isspace(*end) && *end != ',' && *end != ':')) {
            error_setg(errp, "invalid key-raw byte near '%s'", p);
            return;
        }
        data[len++] = byte;
        p = end;
    }
    if (!len) {
        error_setg(errp, "key-raw requires at least a command byte");
        return;
    }
    disp_trace("key-raw", data, len);
    disp_queue_key(s, data, len);
}

void clarion_dispmicom_set_sink(DeviceState *dev, ClarionDispMicomSink fn,
                                void *opaque)
{
    ClarionDispMicomState *s = CLARION_DISPMICOM(dev);

    s->sink = fn;
    s->sink_opaque = opaque;
}

void clarion_dispmicom_set_can_key_sink(DeviceState *dev,
                                        ClarionDispMicomCanKey fn, void *opaque)
{
    ClarionDispMicomState *s = CLARION_DISPMICOM(dev);

    s->can_key = fn;
    s->can_key_opaque = opaque;
}

/* --- QOM --------------------------------------------------------------- */

static void clarion_dispmicom_reset_hold(Object *obj, ResetType type)
{
    ClarionDispMicomState *s = CLARION_DISPMICOM(obj);

    while (!g_queue_is_empty(s->frames)) {
        disp_frame_free(g_queue_pop_head(s->frames));
    }
    if (s->cur) {
        disp_frame_free(s->cur);
        s->cur = NULL;
    }
    disp_frame_free(s->awaiting_ack);
    s->awaiting_ack = NULL;
    s->cur_off = 0;
    s->ack_retries = 0;
    s->rx_state = RX_IDLE;
    s->body_len = 0;
    s->bcc = 0;
    s->linked = false;
    s->key_down_mask = 0;
    timer_del(s->key_up);
}

static void clarion_dispmicom_realize(DeviceState *dev, Error **errp)
{
    ClarionDispMicomState *s = CLARION_DISPMICOM(dev);

    s->frames = g_queue_new();
    s->tx = timer_new_ns(QEMU_CLOCK_VIRTUAL, disp_tx_expire, s);
    s->key_input_handler = qemu_input_handler_register(dev, &disp_key_handler);
    qemu_input_handler_activate(s->key_input_handler);
    if (s->key_input) {
        object_property_add_str(OBJECT(s), "key-raw", NULL,
                                disp_set_key_raw);
    }
    if (s->touch_input) {
        s->input_handler =
            qemu_input_handler_register(dev, &disp_touch_handler);
        qemu_input_handler_activate(s->input_handler);
    }
    s->key_up = timer_new_ns(QEMU_CLOCK_VIRTUAL, disp_key_up, s);
    disp_panel = s;
}

static void clarion_dispmicom_unrealize(DeviceState *dev)
{
    ClarionDispMicomState *s = CLARION_DISPMICOM(dev);

    qemu_input_handler_unregister(s->key_input_handler);
    if (disp_panel == s) {
        disp_panel = NULL;
    }
}

/* panel=jpn|row: which QY8 code column disp_key_event uses */
static char *disp_panel_get(Object *obj, Error **errp)
{
    return g_strdup(CLARION_DISPMICOM(obj)->panel_jpn ? "jpn" : "row");
}

static void disp_panel_set(Object *obj, const char *value, Error **errp)
{
    ClarionDispMicomState *s = CLARION_DISPMICOM(obj);

    if (!strcmp(value, "jpn")) {
        s->panel_jpn = true;
    } else if (!strcmp(value, "row")) {
        s->panel_jpn = false;
    } else {
        error_setg(errp, "panel: expected jpn or row");
    }
}

static const Property clarion_dispmicom_props[] = {
    DEFINE_PROP_BOOL("touch-input", ClarionDispMicomState, touch_input, false),
    DEFINE_PROP_BOOL("key-input", ClarionDispMicomState, key_input, false),
    DEFINE_PROP_BOOL("can-key-input", ClarionDispMicomState, can_key_input,
                     false),
};

static void clarion_dispmicom_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = clarion_dispmicom_realize;
    dc->desc = "Clarion display panel micom (QY7 SCIF3 or QY8 SCIF1)";
    device_class_set_props(dc, clarion_dispmicom_props);
    dc->unrealize = clarion_dispmicom_unrealize;
    rc->phases.hold = clarion_dispmicom_reset_hold;
    object_class_property_add_str(klass, "key", NULL, disp_set_key);
    object_class_property_add_str(klass, "frame", NULL, disp_set_frame);
    object_class_property_add_str(klass, "panel", disp_panel_get,
                                  disp_panel_set);
}

static const TypeInfo clarion_dispmicom_types[] = {
    {
        .name = TYPE_CLARION_DISPMICOM,
        .parent = TYPE_DEVICE,
        .instance_size = sizeof(ClarionDispMicomState),
        .class_init = clarion_dispmicom_class_init,
    },
};

DEFINE_TYPES(clarion_dispmicom_types)
