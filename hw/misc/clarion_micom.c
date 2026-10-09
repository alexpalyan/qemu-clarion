/*
 * Micom — супутній мікроконтролер плати Clarion QY8XXX.
 *
 * На живій платі це окрема мікросхема, під'єднана до SCIF4 головного SoC.
 * Без відповіді від неї ядро WinCE не проходить ініціалізацію: диспетчер
 * пристроїв стоїть на лінку, і бут не доходить до GWES.
 *
 * Формат кадру знято з коду EdaDrv.dll (побудовник @0xef78f164), а не з
 * документації:
 *
 *     10 02 | LEN DATA... (стафінг: 10 -> 10 10) | 10 03 | BCC
 *     BCC = XOR усіх байтів після DLE STX, разом із хвостовими 10 03
 *     (цикл eor/uxtb @0xef78ee58)
 *
 * Перевірка на кадрі самого ядра: 10 02 02 08 00 10 03 19,
 * XOR(02 08 00 10 03) = 0x19 ✔
 *
 * Розбір прийнятого кадру (клас-аксесор vtable @0xef783490): SetFrame
 * @0xef78e96c вимагає len(кадр) - 6 == кадр[2], кладе LEN у obj[0x18], а
 * DATA — у obj[0x1c]. Диспетчер @0xef78baa4 бере DATA[0] як код команди,
 * обробник 0x08 @0xef78bb04 — DATA[1] як підкод.
 *
 * Автомат рукостискання (docs/20-qemu-board.md, розділ «M3b-3»):
 *
 *     HU  -> 08 00              скидання (SendCmd08(0) @0xef78b528)  стан 1
 *     HU  <- 08 01              micom живий                          стан 2
 *     HU  <- 08 0a <коди>       набір команд micom                   стан 3
 *     HU  -> 08 0b              набір прийнято
 *
 * ⚠ Межа чесності. Тут відтворено рівно те, що доведене кодом EdaDrv.dll:
 * кадрування, контрольна сума, автомат 1->2->3 і дві команди сімейства
 * 0x60, розібрані в тому ж джерелі. Усе інше отримує лише квитанцію
 * «<код> 00» — як і в tools/qy8_micom.py, з якого цю модель портовано.
 * Нерозібрані команди свідомо НЕ вигадуються: щоб експериментувати з ними,
 * micom вимикається (-M clarion-qy8,micom=off) і до SCIF4 чіпляється той
 * самий Python-відповідач через -serial.
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
 * Пауза між кадрами. У Python-відповідачі це 50 мс хостового часу; тут —
 * віртуального, тому бут перестає залежати від завантаження машини.
 * Причина самої паузи та сама: приймач SCIF має FIFO на 16 байтів, і
 * драйвер мусить устигнути підтвердити попередній кадр.
 */
#define MICOM_GAP_NS        (50 * 1000 * 1000)
/* Доливання решти довгого кадру, поки DMA звільняє FIFO. */
#define MICOM_FILL_NS       (1 * 1000 * 1000)

typedef enum {
    RX_IDLE,            /* чекаємо DLE */
    RX_DLE,             /* бачили DLE поза кадром */
    RX_BODY,            /* усередині кадру */
    RX_BODY_DLE,        /* усередині кадру, бачили DLE */
    RX_BCC,             /* наступний байт — контрольна сума */
} MicomRxState;

struct ClarionMicomState {
    DeviceState parent_obj;

    ClarionMicomSink sink;
    void *sink_opaque;

    QEMUTimer *tx;
    GQueue *frames;             /* черга GByteArray * — готові кадри */
    GByteArray *cur;            /* кадр, який зараз віддаємо */
    unsigned cur_off;           /* скільки з нього вже прийнято */

    MicomRxState rx_state;
    uint8_t body[MICOM_MAX_DATA + 2];
    unsigned body_len;
    uint8_t bcc;

    bool handshake_done;        /* бачили 08 0b від ядра */
};

/* --- побудова кадру ---------------------------------------------------- */

static GByteArray *micom_build(const uint8_t *data, unsigned len)
{
    GByteArray *f = g_byte_array_new();
    uint8_t b, bcc = 0;
    unsigned i;

    b = DLE; g_byte_array_append(f, &b, 1);
    b = STX; g_byte_array_append(f, &b, 1);

    /* тіло = LEN + DATA, зі стафінгом DLE; BCC рахується по стафнутих байтах */
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
    g_queue_push_head(s->frames, f);        /* квитанція йде поперед черги */
    if (!s->cur) {
        micom_arm(s, MICOM_FILL_NS);
    }
}

/* --- віддавання байтів у SCIF ------------------------------------------ */

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
            micom_arm(s, MICOM_GAP_NS);     /* пауза між кадрами */
        }
    } else {
        micom_arm(s, MICOM_FILL_NS);        /* FIFO повний — доллємо згодом */
    }
}

/* --- обробка кадру від ядра -------------------------------------------- */

/*
 * Набір команд, який micom оголошує кадром 08 0a. Перший байт списку — сам
 * підкод 0x0a: RegisterSupported @0xef78bc2c бере DATA+1 завдовжки N-1,
 * тож прапорець 0x0a виставляється заодно. Байт 0xff у списку — ознака
 * помилки, після нього розбір припиняється, тому список іде 0x01..0x3e.
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

/* CEdaCtl init @0xef787888: підписка на async-сімейство 0x3a, відповіді не
 * потребує (sender не створює waiter). */
static const uint8_t cmd_60_3a[] = { 0x60, 0x0d, 0x3a, 0, 0, 0, 0xe3 };
/* CEdaCtl init @0xef787930: RX-handler @0xef789ec0 вимагає DATA[0] = 0x10,
 * DATA[1] = 0x3f, статус у DATA[5]; 0 = успіх. */
static const uint8_t cmd_60_3f[] = { 0x60, 0x0d, 0x3f, 0, 0, 0, 0x01 };
static const uint8_t rsp_10_3f[] = { 0x10, 0x3f, 0, 0, 0, 0 };

static void micom_frame(ClarionMicomState *s)
{
    const uint8_t *data = s->body + 1;
    unsigned len = s->body_len ? s->body_len - 1 : 0;
    uint8_t cmd, sub, ack[2];

    if (s->body_len && s->body[0] != len) {
        qemu_log_mask(LOG_GUEST_ERROR, "clarion-micom: LEN %u не збігається з "
                      "довжиною DATA %u\n", s->body[0], len);
    }
    /* На кожен кадр — DLE ACK, як у живому лінку. */
    ack[0] = DLE;
    ack[1] = ACK;
    micom_queue_raw2(s, ack[0], ack[1]);

    if (!len) {
        return;
    }
    cmd = data[0];
    sub = (len > 1) ? data[1] : 0;

    if (cmd == 0x08 && sub == 0x00) {           /* скидання -> стан 2, 3 */
        const uint8_t alive[] = { 0x08, 0x01 };

        s->handshake_done = false;
        micom_queue(s, alive, sizeof(alive));
        micom_send_cmd_set(s);
    } else if (cmd == 0x08 && sub == 0x0b) {    /* набір прийнято */
        s->handshake_done = true;
    } else if (len == sizeof(cmd_60_3a) &&
               !memcmp(data, cmd_60_3a, sizeof(cmd_60_3a))) {
        /* відповіді не потребує */
    } else if (len == sizeof(cmd_60_3f) &&
               !memcmp(data, cmd_60_3f, sizeof(cmd_60_3f))) {
        micom_queue(s, rsp_10_3f, sizeof(rsp_10_3f));
    } else {
        const uint8_t recv[2] = { cmd, 0x00 };  /* квитанція на решту */

        micom_queue(s, recv, sizeof(recv));
    }
}

/* --- розбір потоку від ядра -------------------------------------------- */

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
            s->rx_state = RX_DLE;           /* 10 10 поза кадром — тримаємось */
        } else {
            /* DLE ACK / DLE NAK від ядра — підтвердження наших кадрів */
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
        if (b == DLE) {                     /* стафінг: один літеральний 0x10 */
            if (s->body_len < sizeof(s->body)) {
                s->body[s->body_len++] = DLE;
            }
            s->rx_state = RX_BODY;
        } else if (b == ETX) {
            s->rx_state = RX_BCC;
        } else {
            s->rx_state = RX_IDLE;          /* поламаний кадр — кидаємо */
        }
        break;

    case RX_BCC:
        if (b != s->bcc) {
            qemu_log_mask(LOG_GUEST_ERROR, "clarion-micom: BCC %#x, чекали "
                          "%#x — кадр усе одно обробляємо\n", b, s->bcc);
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
}

static void clarion_micom_realize(DeviceState *dev, Error **errp)
{
    ClarionMicomState *s = CLARION_MICOM(dev);

    s->frames = g_queue_new();
    s->tx = timer_new_ns(QEMU_CLOCK_VIRTUAL, micom_tx_expire, s);
}

static void clarion_micom_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = clarion_micom_realize;
    dc->desc = "Clarion QY8XXX companion micom (SCIF4 link)";
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
