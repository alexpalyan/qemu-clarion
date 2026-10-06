/*
 * Bounded u-blox receiver model for Clarion QY8 SCIF2.
 * UBX framing and NMEA formats follow the u-blox protocol. Satellite count,
 * signal levels, and accuracy values are synthetic.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/bswap.h"
#include "system/rtc.h"
#include "hw/misc/clarion_ublox.h"
#include "migration/vmstate.h"
#include <math.h>

#define UBX_MAX 128
#define UBX_TICK_NS (NANOSECONDS_PER_SECOND)
#define UBX_TX_RETRY_NS (10 * 1000 * 1000)

static void ublox_trace_tx(const uint8_t *buf, unsigned len)
{
    char hex[UBX_MAX * 3 + 1];
    unsigned i;

    for (i = 0; i < len; i++) {
        g_snprintf(hex + i * 3, sizeof(hex) - i * 3, "%02x%s", buf[i],
                   i + 1 == len ? "" : " ");
    }
    hex[len * 3] = '\0';
    qemu_log_mask(LOG_UNIMP, "clarion-ublox: TX %s\n", hex);
}

struct ClarionUbloxState {
    DeviceState parent_obj;
    ClarionUbloxSink sink;
    void *sink_opaque;
    QEMUTimer *tick;
    QEMUTimer *tx_timer;
    GQueue *tx_queue;
    GByteArray *tx_current;
    unsigned tx_offset;
    uint8_t rx[UBX_MAX];
    unsigned rx_len;
    double lat, lon, speed, course;
    bool quiet;
    bool startup_pending;
    int64_t quiet_until;
    uint32_t nav_ubx;
};

static void ubx_send(ClarionUbloxState *s, uint8_t cls, uint8_t id,
                     const uint8_t *payload, unsigned len);

static void put32(uint8_t *p, uint32_t v)
{
    stl_le_p(p, v);
}

static void put16(uint8_t *p, uint16_t v)
{
    stw_le_p(p, v);
}

static void ublox_tx_pump(void *opaque)
{
    ClarionUbloxState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    while (s->sink) {
        int remaining, accepted;

        if (!s->tx_current) {
            s->tx_current = g_queue_pop_head(s->tx_queue);
            s->tx_offset = 0;
            if (!s->tx_current) {
                return;
            }
        }
        remaining = s->tx_current->len - s->tx_offset;
        accepted = s->sink(s->sink_opaque, s->tx_current->data + s->tx_offset,
                           remaining);
        if (accepted <= 0) {
            timer_mod(s->tx_timer, now + UBX_TX_RETRY_NS);
            return;
        }
        s->tx_offset += accepted;
        if (s->tx_offset < s->tx_current->len) {
            timer_mod(s->tx_timer, now + UBX_TX_RETRY_NS);
            return;
        }
        g_byte_array_unref(s->tx_current);
        s->tx_current = NULL;
    }
}

static void ublox_queue_tx(ClarionUbloxState *s, const uint8_t *buf,
                           unsigned len)
{
    GByteArray *frame = g_byte_array_sized_new(len);

    g_byte_array_append(frame, buf, len);
    g_queue_push_tail(s->tx_queue, frame);
    if (s->sink) {
        timer_mod(s->tx_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1);
    }
}

static void put_i32(uint8_t *p, int32_t v)
{
    stl_le_p(p, v);
}

static void ublox_send_nav(ClarionUbloxState *s, const struct tm *tm)
{
    const double a = 6378137.0;
    const double f = 1.0 / 298.257223563;
    double la = s->lat * M_PI / 180.0;
    double lo = s->lon * M_PI / 180.0;
    double e2 = f * (2.0 - f);
    double n = a / sqrt(1.0 - e2 * sin(la) * sin(la));
    double h = 40.0;
    int year = tm->tm_year + 1900;
    int64_t years = year - 1980;
    int64_t gps_days = years * 365 + (year - 1977) / 4 - (year - 1901) / 100 +
                       (year - 1601) / 400 - 5 + tm->tm_yday;
    int64_t elapsed = gps_days * 86400 + tm->tm_hour * 3600 + tm->tm_min * 60 +
                      tm->tm_sec + 18;
    uint16_t week = elapsed / 604800;
    uint32_t tow = (elapsed % 604800) * 1000;
    uint8_t p[64] = { 0 };
    double speed = s->speed * 1852.0 / 3600.0;
    double heading = s->course * M_PI / 180.0;
    int32_t x = (n + h) * cos(la) * cos(lo) * 100;
    int32_t y = (n + h) * cos(la) * sin(lo) * 100;
    int32_t z = (n * (1.0 - e2) + h) * sin(la) * 100;

    if (s->nav_ubx & (1u << 1)) {
        put32(p, tow);
        put_i32(p + 4, x);
        put_i32(p + 8, y);
        put_i32(p + 12, z);
        put32(p + 16, 300);
        ubx_send(s, 0x01, 0x01, p, 20);
    }
    if (s->nav_ubx & (1u << 2)) {
        memset(p, 0, sizeof(p));
        put32(p, tow);
        put_i32(p + 4, s->lon * 1e7);
        put_i32(p + 8, s->lat * 1e7);
        put_i32(p + 12, 40000);
        put_i32(p + 16, 40000);
        put32(p + 20, 3000);
        put32(p + 24, 4000);
        ubx_send(s, 0x01, 0x02, p, 28);
    }
    if (s->nav_ubx & (1u << 0x12)) {
        memset(p, 0, sizeof(p));
        put32(p, tow);
        put_i32(p + 4, speed * cos(heading) * 100);
        put_i32(p + 8, speed * sin(heading) * 100);
        put_i32(p + 12, 0);
        put32(p + 16, speed * 100);
        put32(p + 20, speed * 100);
        put_i32(p + 24, s->course * 1e5);
        put32(p + 28, 50);
        put32(p + 32, 200000);
        ubx_send(s, 0x01, 0x12, p, 36);
    }
    memset(p, 0, sizeof(p));
    put32(p, tow);
    p[4] = 3;
    p[5] = 0x0d;
    put32(p + 8, 12000);
    put32(p + 12, tow);
    ubx_send(s, 0x01, 0x03, p, 16);

    memset(p, 0, sizeof(p));
    put32(p, tow);
    put16(p + 8, week);
    p[10] = 3;
    p[11] = 0x0d;
    put_i32(p + 12, x);
    put_i32(p + 16, y);
    put_i32(p + 20, z);
    put32(p + 24, 300);
    put32(p + 40, 50);
    put16(p + 44, 150);
    p[47] = 11;
    ubx_send(s, 0x01, 0x06, p, 52);
}

static void ubx_send(ClarionUbloxState *s, uint8_t cls, uint8_t id,
                     const uint8_t *payload, unsigned len)
{
    uint8_t b[UBX_MAX], a = 0, c = 0;
    unsigned i, n = len + 8;

    if (n > sizeof(b)) {
        return;
    }
    b[0] = 0xb5;
    b[1] = 0x62;
    b[2] = cls;
    b[3] = id;
    b[4] = len;
    b[5] = len >> 8;
    memcpy(b + 6, payload, len);
    for (i = 2; i < len + 6; i++) {
        a += b[i];
        c += a;
    }
    b[len + 6] = a;
    b[len + 7] = c;
    ublox_trace_tx(b, n);
    ublox_queue_tx(s, b, n);
}

static void nmea_send(ClarionUbloxState *s, const char *body)
{
    char line[UBX_MAX];
    unsigned i;
    uint8_t sum = 0;

    for (i = 0; body[i]; i++) {
        sum ^= body[i];
    }
    i = snprintf(line, sizeof(line), "$%s*%02X\r\n", body, sum);
    if (i < sizeof(line)) {
        ublox_trace_tx((const uint8_t *)line, i);
        ublox_queue_tx(s, (const uint8_t *)line, i);
    }
}

static void ublox_banner(ClarionUbloxState *s)
{
    nmea_send(s, "GPTXT,01,01,02,u-blox ag - www.u-blox.com");
    nmea_send(s, "GPTXT,01,01,02,HW  UBX-G70xx   00070000");
    nmea_send(s, "GPTXT,01,01,02,ROM CORE 1.00 (59842) Jun 27 2012 17:43:52");
}

static void ublox_ack(ClarionUbloxState *s, uint8_t cls, uint8_t id)
{
    uint8_t payload[2] = { cls, id };

    ubx_send(s, 0x05, 0x01, payload, sizeof(payload));
}

static void ublox_handle(ClarionUbloxState *s, uint8_t cls, uint8_t id,
                         const uint8_t *payload, unsigned len)
{
    qemu_log_mask(LOG_UNIMP,
                  "clarion-ublox: RX UBX class=%02x id=%02x len=%u\n", cls, id,
                  len);
    if (cls == 0x06 && id == 0x04) {
        qemu_log_mask(LOG_UNIMP, "clarion-ublox: reset\n");
        if (s->tx_current) {
            g_byte_array_unref(s->tx_current);
            s->tx_current = NULL;
            s->tx_offset = 0;
        }
        g_queue_clear_full(s->tx_queue, (GDestroyNotify)g_byte_array_unref);
        ublox_ack(s, cls, id);
        s->quiet = true;
        s->startup_pending = true;
        s->nav_ubx = 0;
        s->quiet_until =
            qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + NANOSECONDS_PER_SECOND;
        timer_mod(s->tick, s->quiet_until);
    } else if (cls == 0x06) {
        if (id == 0x01 && len >= 3 && payload[0] == 0x01 && payload[1] < 32) {
            unsigned i;
            bool enabled = false;

            for (i = 2; i < len; i++) {
                enabled |= payload[i] != 0;
            }
            if (enabled) {
                s->nav_ubx |= 1u << payload[1];
            } else {
                s->nav_ubx &= ~(1u << payload[1]);
            }
        }
        ublox_ack(s, cls, id);
    } else if (cls == 0x0a && id == 0x04) {
        uint8_t version[40] = { 0 };

        memcpy(version, "1.00 (59842)", 12);
        memcpy(version + 30, "00070000", 8);
        ubx_send(s, 0x0a, 0x04, version, sizeof(version));
    }
}

void clarion_ublox_rx_byte(DeviceState *dev, uint8_t byte)
{
    ClarionUbloxState *s = CLARION_UBLOX(dev);
    unsigned length;

    if (s->rx_len == sizeof(s->rx)) {
        s->rx_len = 0;
    }
    s->rx[s->rx_len++] = byte;
    while (s->rx_len >= 8) {
        if (s->rx[0] != 0xb5 || s->rx[1] != 0x62) {
            memmove(s->rx, s->rx + 1, --s->rx_len);
            continue;
        }
        length = s->rx[4] | (s->rx[5] << 8);
        if (length + 8 > sizeof(s->rx)) {
            s->rx_len = 0;
            return;
        }
        if (s->rx_len < length + 8) {
            return;
        }
        {
            uint8_t a = 0, c = 0;
            unsigned i;

            for (i = 2; i < length + 6; i++) {
                a += s->rx[i];
                c += a;
            }
            if (a == s->rx[length + 6] && c == s->rx[length + 7]) {
                ublox_handle(s, s->rx[2], s->rx[3], s->rx + 6, length);
            }
        }
        memmove(s->rx, s->rx + length + 8, s->rx_len - length - 8);
        s->rx_len -= length + 8;
    }
}

void clarion_ublox_set_sink(DeviceState *dev, ClarionUbloxSink fn, void *opaque)
{
    ClarionUbloxState *s = CLARION_UBLOX(dev);

    s->sink = fn;
    s->sink_opaque = opaque;
    if (fn && (!g_queue_is_empty(s->tx_queue) || s->tx_current)) {
        timer_mod(s->tx_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1);
    }
}

void clarion_ublox_set_position(DeviceState *dev, double lat, double lon,
                                double speed, double course)
{
    ClarionUbloxState *s = CLARION_UBLOX(dev);

    s->lat = lat;
    s->lon = lon;
    s->speed = speed;
    s->course = course;
}

static void ublox_tick(void *opaque)
{
    ClarionUbloxState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    struct tm tm;
    int year;
    char hms[16], dmy[16], lat[20], lon[20], lahemi, lohemi, body[UBX_MAX];
    double av;

    if (s->quiet) {
        if (now < s->quiet_until) {
            timer_mod(s->tick, s->quiet_until);
            return;
        }
        s->quiet = false;
    }
    if (s->startup_pending) {
        if (s->sink) {
            ublox_banner(s);
        }
        s->startup_pending = false;
        timer_mod(s->tick, now + UBX_TICK_NS);
        return;
    }
    qemu_get_timedate(&tm, 0);
    qemu_log_mask(LOG_UNIMP, "clarion-ublox: tick\n");
    ublox_send_nav(s, &tm);
    year = tm.tm_year % 100;
    snprintf(hms, sizeof(hms), "%02d%02d%02d.00", tm.tm_hour, tm.tm_min,
             tm.tm_sec);
    snprintf(dmy, sizeof(dmy), "%02d%02d%02d", tm.tm_mday, tm.tm_mon + 1, year);
    av = fabs(s->lat);
    snprintf(lat, sizeof(lat), "%02d%07.4f", (int)av, (av - (int)av) * 60);
    lahemi = s->lat < 0 ? 'S' : 'N';
    av = fabs(s->lon);
    snprintf(lon, sizeof(lon), "%03d%07.4f", (int)av, (av - (int)av) * 60);
    lohemi = s->lon < 0 ? 'W' : 'E';
    snprintf(body, sizeof(body),
             "GPGGA,%s,%s,%c,%s,%c,1,11,0.9,40.0,M,45.0,M,,", hms, lat, lahemi,
             lon, lohemi);
    nmea_send(s, body);
    nmea_send(s, "GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,,2.0,0.9,1.2");
    nmea_send(
        s, "GPGSV,3,1,11,01,20,000,38,02,27,031,41,03,34,062,44,04,41,093,47");
    nmea_send(
        s, "GPGSV,3,2,11,05,48,124,38,06,55,155,41,07,62,186,44,08,29,217,47");
    nmea_send(s, "GPGSV,3,3,11,09,36,248,38,10,43,279,41,11,50,310,44");
    snprintf(body, sizeof(body), "GPRMC,%s,A,%s,%c,%s,%c,%.2f,%.2f,%s,,,A", hms,
             lat, lahemi, lon, lohemi, s->speed, s->course, dmy);
    nmea_send(s, body);
    snprintf(body, sizeof(body), "GPVTG,%.2f,T,,M,%.2f,N,%.2f,K,A", s->course,
             s->speed, s->speed * 1.852);
    nmea_send(s, body);
    snprintf(body, sizeof(body),
             "PUBX,00,%s,%s,%c,%s,%c,40.000,G3,2.0,2.0,%.3f,%.2f,0.000,,0.9,1."
             "2,0.8,11,0,0",
             hms, lat, lahemi, lon, lohemi, s->speed * 1.852, s->course);
    nmea_send(s, body);
    snprintf(body, sizeof(body), "GPGLL,%s,%c,%s,%c,%s,A,A", lat, lahemi, lon,
             lohemi, hms);
    nmea_send(s, body);
    if (s->speed > 0) {
        double distance = s->speed * 1852.0 / 3600.0;
        double bearing = s->course * M_PI / 180.0;
        s->lat += (distance * cos(bearing) / 6371000.0) * 180.0 / M_PI;
        s->lon += (distance * sin(bearing) /
                   (6371000.0 * cos(s->lat * M_PI / 180.0))) *
                  180.0 / M_PI;
    }
    timer_mod(s->tick, now + UBX_TICK_NS);
}

static void ublox_realize(DeviceState *dev, Error **errp)
{
    ClarionUbloxState *s = CLARION_UBLOX(dev);

    (void)errp;
    s->lat = 50.4501;
    s->lon = 30.5234;
    s->tick = timer_new_ns(QEMU_CLOCK_VIRTUAL, ublox_tick, s);
    s->tx_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, ublox_tx_pump, s);
    s->tx_queue = g_queue_new();
    s->startup_pending = true;
    timer_mod(s->tick, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + UBX_TICK_NS);
}

static void ublox_finalize(Object *obj)
{
    ClarionUbloxState *s = CLARION_UBLOX(obj);

    timer_free(s->tick);
    timer_free(s->tx_timer);
    if (s->tx_current) {
        g_byte_array_unref(s->tx_current);
    }
    g_queue_free_full(s->tx_queue, (GDestroyNotify)g_byte_array_unref);
}

static void ublox_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    (void)data;
    dc->realize = ublox_realize;
}

static const TypeInfo ublox_types[] = {
    {
        .name = TYPE_CLARION_UBLOX,
        .parent = TYPE_DEVICE,
        .instance_size = sizeof(ClarionUbloxState),
        .instance_finalize = ublox_finalize,
        .class_init = ublox_class_init,
    },
};

DEFINE_TYPES(ublox_types)
