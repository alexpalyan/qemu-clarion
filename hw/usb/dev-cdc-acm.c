/*
 * Sierra Wireless USB CDC ACM modem used by the Clarion QY8 TCU model.
 *
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/bswap.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/usb/usb.h"
#include "migration/vmstate.h"
#include "chardev/char-fe.h"
#include "qom/object.h"
#include "qapi/visitor.h"
#include "desc.h"

#define TYPE_USB_CDC_ACM "usb-cdc-acm"
OBJECT_DECLARE_SIMPLE_TYPE(USBCDCACMState, USB_CDC_ACM)

#define CDC_ACM_SET_LINE_CODING 0x20
#define CDC_ACM_GET_LINE_CODING 0x21
#define CDC_ACM_SET_CONTROL_LINE 0x22
#define CDC_ACM_SEND_BREAK 0x23
#define CDC_ACM_SERIAL_STATE 0x20
#define CDC_ACM_RX_CARRIER (1u << 0)
#define CDC_ACM_TX_CARRIER (1u << 1)
#define CDC_ACM_RX_SIZE 8192
#define CDC_ACM_EP_INTERRUPT 4
#define CDC_ACM_EP_BULK_IN 5
#define CDC_ACM_EP_BULK_OUT 4
#define CDC_ACM_EP_SECONDARY_BULK_IN 7
#define CDC_ACM_EP_SECONDARY_BULK_OUT 5
#define CDC_ACM_EP_INTERRUPT_SECONDARY 6
#define CDC_ACM_EP_INTERRUPT_SIZE 64
#define CDC_ACM_NOTIFICATION_SIZE 10

struct USBCDCACMState {
    USBDevice dev;
    USBEndpoint *intr;
    USBEndpoint *intr_secondary;
    USBEndpoint *bulk_in;
    USBEndpoint *bulk_in_secondary;
    CharFrontend chr;
    uint8_t rx_buf[CDC_ACM_RX_SIZE];
    uint32_t rx_head;
    uint32_t rx_count;
    uint32_t baud;
    uint8_t stop_bits;
    uint8_t parity;
    uint8_t data_bits;
    uint8_t dtr;
    uint8_t rts;
    uint16_t serial_state;
    bool serial_state_pending;
};

enum {
    STR_CONFIGURATION = 1,
    STR_PRODUCT,
    STR_MANUFACTURER,
};

static const USBDescStrings cdc_acm_strings = {
    [STR_CONFIGURATION] = "Sierra Configuration",
    [STR_PRODUCT] = "AR8550",
    [STR_MANUFACTURER] = "Sierra Wireless, Incorporated",
};

static const USBDescIface cdc_acm_ifaces[] = {
    {
        .bInterfaceNumber = 0,
        .bNumEndpoints = 2,
        .bInterfaceClass = USB_CLASS_VENDOR_SPEC,
        .bInterfaceSubClass = USB_CLASS_VENDOR_SPEC,
        .bInterfaceProtocol = USB_CLASS_VENDOR_SPEC,
        .eps =
            (USBDescEndpoint[]){
                {
                    .bEndpointAddress = USB_DIR_IN | 1,
                    .bmAttributes = USB_ENDPOINT_XFER_BULK,
                    .wMaxPacketSize = 512,
                },
                {
                    .bEndpointAddress = USB_DIR_OUT | 1,
                    .bmAttributes = USB_ENDPOINT_XFER_BULK,
                    .wMaxPacketSize = 512,
                },
            },
    },
    {
        .bInterfaceNumber = 1,
        .bNumEndpoints = 2,
        .bInterfaceClass = USB_CLASS_VENDOR_SPEC,
        .bInterfaceSubClass = USB_CLASS_VENDOR_SPEC,
        .bInterfaceProtocol = USB_CLASS_VENDOR_SPEC,
        .eps =
            (USBDescEndpoint[]){
                {
                    .bEndpointAddress = USB_DIR_IN | 2,
                    .bmAttributes = USB_ENDPOINT_XFER_BULK,
                    .wMaxPacketSize = 512,
                },
                {
                    .bEndpointAddress = USB_DIR_OUT | 2,
                    .bmAttributes = USB_ENDPOINT_XFER_BULK,
                    .wMaxPacketSize = 512,
                },
            },
    },
    {
        .bInterfaceNumber = 2,
        .bNumEndpoints = 2,
        .bInterfaceClass = USB_CLASS_VENDOR_SPEC,
        .bInterfaceSubClass = USB_CLASS_VENDOR_SPEC,
        .bInterfaceProtocol = USB_CLASS_VENDOR_SPEC,
        .eps =
            (USBDescEndpoint[]){
                {
                    .bEndpointAddress = USB_DIR_IN | 3,
                    .bmAttributes = USB_ENDPOINT_XFER_BULK,
                    .wMaxPacketSize = 512,
                },
                {
                    .bEndpointAddress = USB_DIR_OUT | 3,
                    .bmAttributes = USB_ENDPOINT_XFER_BULK,
                    .wMaxPacketSize = 512,
                },
            },
    },
    {
        .bInterfaceNumber = 3,
        .bNumEndpoints = 3,
        .bInterfaceClass = USB_CLASS_COMM,
        .bInterfaceSubClass = 2,
        .bInterfaceProtocol = 1,
        .eps =
            (USBDescEndpoint[]){
                {
                    .bEndpointAddress = USB_DIR_IN | CDC_ACM_EP_INTERRUPT,
                    .bmAttributes = USB_ENDPOINT_XFER_INT,
                    .wMaxPacketSize = CDC_ACM_EP_INTERRUPT_SIZE,
                    .bInterval = 5,
                },
                {
                    .bEndpointAddress = USB_DIR_IN | CDC_ACM_EP_BULK_IN,
                    .bmAttributes = USB_ENDPOINT_XFER_BULK,
                    .wMaxPacketSize = 512,
                },
                {
                    .bEndpointAddress = USB_DIR_OUT | CDC_ACM_EP_BULK_OUT,
                    .bmAttributes = USB_ENDPOINT_XFER_BULK,
                    .wMaxPacketSize = 512,
                },
            },
    },
    {
        .bInterfaceNumber = 4,
        .bNumEndpoints = 3,
        .bInterfaceClass = USB_CLASS_COMM,
        .bInterfaceSubClass = 2,
        .bInterfaceProtocol = 1,
        .eps =
            (USBDescEndpoint[]){
                {
                    .bEndpointAddress =
                        USB_DIR_IN | CDC_ACM_EP_INTERRUPT_SECONDARY,
                    .bmAttributes = USB_ENDPOINT_XFER_INT,
                    .wMaxPacketSize = CDC_ACM_EP_INTERRUPT_SIZE,
                    .bInterval = 5,
                },
                {
                    .bEndpointAddress =
                        USB_DIR_IN | CDC_ACM_EP_SECONDARY_BULK_IN,
                    .bmAttributes = USB_ENDPOINT_XFER_BULK,
                    .wMaxPacketSize = 512,
                },
                {
                    .bEndpointAddress =
                        USB_DIR_OUT | CDC_ACM_EP_SECONDARY_BULK_OUT,
                    .bmAttributes = USB_ENDPOINT_XFER_BULK,
                    .wMaxPacketSize = 512,
                },
            },
    },
    {
        .bInterfaceNumber = 5,
        .bNumEndpoints = 3,
        .bInterfaceClass = USB_CLASS_VENDOR_SPEC,
        .bInterfaceSubClass = USB_CLASS_VENDOR_SPEC,
        .bInterfaceProtocol = USB_CLASS_VENDOR_SPEC,
        .eps =
            (USBDescEndpoint[]){
                {
                    .bEndpointAddress = USB_DIR_IN | 8,
                    .bmAttributes = USB_ENDPOINT_XFER_INT,
                    .wMaxPacketSize = 64,
                    .bInterval = 5,
                },
                {
                    .bEndpointAddress = USB_DIR_IN | 9,
                    .bmAttributes = USB_ENDPOINT_XFER_BULK,
                    .wMaxPacketSize = 512,
                },
                {
                    .bEndpointAddress = USB_DIR_OUT | 6,
                    .bmAttributes = USB_ENDPOINT_XFER_BULK,
                    .wMaxPacketSize = 512,
                },
            },
    },
    {
        .bInterfaceNumber = 6,
        .bNumEndpoints = 3,
        .bInterfaceClass = USB_CLASS_VENDOR_SPEC,
        .bInterfaceSubClass = USB_CLASS_VENDOR_SPEC,
        .bInterfaceProtocol = USB_CLASS_VENDOR_SPEC,
        .eps =
            (USBDescEndpoint[]){
                {
                    .bEndpointAddress = USB_DIR_IN | 10,
                    .bmAttributes = USB_ENDPOINT_XFER_INT,
                    .wMaxPacketSize = 64,
                    .bInterval = 5,
                },
                {
                    .bEndpointAddress = USB_DIR_IN | 11,
                    .bmAttributes = USB_ENDPOINT_XFER_BULK,
                    .wMaxPacketSize = 512,
                },
                {
                    .bEndpointAddress = USB_DIR_OUT | 7,
                    .bmAttributes = USB_ENDPOINT_XFER_BULK,
                    .wMaxPacketSize = 512,
                },
            },
    },
};

static const USBDescDevice cdc_acm_device = {
    .bcdUSB = 0x0200,
    .bDeviceClass = 0,
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = 64,
    .bNumConfigurations = 1,
    .confs =
        (USBDescConfig[]){
            {
                .bNumInterfaces = ARRAY_SIZE(cdc_acm_ifaces),
                .bConfigurationValue = 1,
                .iConfiguration = STR_CONFIGURATION,
                .bmAttributes = USB_CFG_ATT_ONE,
                .bMaxPower = 0,
                .nif = ARRAY_SIZE(cdc_acm_ifaces),
                .ifs = cdc_acm_ifaces,
            },
        },
};

static const USBDesc cdc_acm_desc = {
    .id = {
        .idVendor = 0x1199,
        .idProduct = 0x68ab,
        .bcdDevice = 0x0006,
        .iManufacturer = STR_MANUFACTURER,
        .iProduct = STR_PRODUCT,
        .iSerialNumber = 0,
    },
    .full = &cdc_acm_device,
    .high = &cdc_acm_device,
    .str = cdc_acm_strings,
};

static char *cdc_acm_hex(const uint8_t *buf, size_t len)
{
    GString *str = g_string_sized_new(len * 2 + 1);
    size_t i;

    for (i = 0; i < len; i++) {
        g_string_append_printf(str, "%02x", buf[i]);
    }
    return g_string_free(str, false);
}

static void cdc_acm_log(const char *event, const char *fields)
{
    qemu_log_mask(LOG_GUEST_ERROR,
                  "{\"tick_ns\":%" PRId64 ",\"event\":\"%s\"%s}\n",
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL), event, fields);
}

static void cdc_acm_log_state(const char *event, const char *format, ...)
{
    va_list ap;
    char *fields;

    va_start(ap, format);
    fields = g_strdup_vprintf(format, ap);
    va_end(ap);
    cdc_acm_log(event, fields);
    g_free(fields);
}

static void cdc_acm_log_control(USBPacket *p, int request, int value, int index,
                                int length, const uint8_t *data,
                                size_t data_len)
{
    char *hex = cdc_acm_hex(data, data_len);
    char *fields =
        g_strdup_printf(",\"bmRequestType\":%u,\"bRequest\":%u,\"wValue\":%u"
                        ",\"wIndex\":%u,\"wLength\":%u,\"data\":\"%s\""
                        ",\"status\":%d",
                        (request >> 8) & 0xff, request & 0xff, value & 0xffff,
                        index & 0xffff, length & 0xffff, hex, p->status);

    cdc_acm_log("control", fields);
    g_free(fields);
    g_free(hex);
}

static void cdc_acm_log_bytes(const char *event, const char *direction,
                              const uint8_t *data, size_t len)
{
    char *hex = cdc_acm_hex(data, len);
    char *fields =
        g_strdup_printf(",\"direction\":\"%s\",\"length\":%zu,\"data\":\"%s\"",
                        direction, len, hex);

    cdc_acm_log(event, fields);
    g_free(fields);
    g_free(hex);
}

static void cdc_acm_set_serial_state(USBCDCACMState *s, uint16_t state)
{
    state &= CDC_ACM_RX_CARRIER | CDC_ACM_TX_CARRIER;
    if (state == s->serial_state) {
        return;
    }

    s->serial_state = state;
    s->serial_state_pending = true;
    cdc_acm_log_state("serial_state_set", ",\"dcd\":%u,\"dsr\":%u,\"bits\":%u",
                      !!(state & CDC_ACM_RX_CARRIER),
                      !!(state & CDC_ACM_TX_CARRIER), state);
    usb_wakeup(s->intr, 0);
    usb_wakeup(s->intr_secondary, 0);
}

static void cdc_acm_get_state(Object *obj, Visitor *v, const char *name,
                              void *opaque, Error **errp)
{
    USBCDCACMState *s = USB_CDC_ACM(obj);
    uint32_t state = s->serial_state;

    (void)opaque;
    visit_type_uint32(v, name, &state, errp);
}

static void cdc_acm_set_state(Object *obj, Visitor *v, const char *name,
                              void *opaque, Error **errp)
{
    USBCDCACMState *s = USB_CDC_ACM(obj);
    uint32_t state;

    (void)opaque;
    if (!visit_type_uint32(v, name, &state, errp)) {
        return;
    }
    if (state & ~(CDC_ACM_RX_CARRIER | CDC_ACM_TX_CARRIER)) {
        error_setg(errp,
                   "serial-state accepts only DCD (bit 0) and DSR (bit 1)");
        return;
    }
    cdc_acm_set_serial_state(s, state);
}

static void cdc_acm_reset(USBDevice *dev)
{
    USBCDCACMState *s = USB_CDC_ACM(dev);

    s->rx_head = 0;
    s->rx_count = 0;
    s->baud = 9600;
    s->stop_bits = 0;
    s->parity = 0;
    s->data_bits = 8;
    s->dtr = 0;
    s->rts = 0;
    s->serial_state = 0;
    s->serial_state_pending = false;
    cdc_acm_log("reset", "");
}

static void cdc_acm_handle_control(USBDevice *dev, USBPacket *p, int request,
                                   int value, int index, int length,
                                   uint8_t *data)
{
    USBCDCACMState *s = USB_CDC_ACM(dev);
    size_t data_len = 0;
    int ret;

    ret = usb_desc_handle_control(dev, p, request, value, index, length, data);
    if (ret >= 0) {
        data_len = (request >> 8) & USB_DIR_IN ? p->actual_length : length;
        cdc_acm_log_control(p, request, value, index, length, data, data_len);
        return;
    }

    switch (request) {
    case ClassInterfaceOutRequest | CDC_ACM_SET_LINE_CODING:
        if ((index != 3 && index != 4) || length != 7) {
            goto stall;
        }
        s->baud = ldl_le_p(data);
        s->stop_bits = data[4];
        s->parity = data[5];
        s->data_bits = data[6];
        data_len = length;
        cdc_acm_log_control(p, request, value, index, length, data, data_len);
        cdc_acm_log_state("set_line_coding",
                          ",\"baud\":%u,\"stop_bits\":%u,\"parity\":%u"
                          ",\"data_bits\":%u",
                          s->baud, s->stop_bits, s->parity, s->data_bits);
        return;
    case ClassInterfaceRequest | CDC_ACM_GET_LINE_CODING:
        if ((index != 3 && index != 4) || length < 7) {
            goto stall;
        }
        stl_le_p(data, s->baud);
        data[4] = s->stop_bits;
        data[5] = s->parity;
        data[6] = s->data_bits;
        p->actual_length = 7;
        cdc_acm_log_control(p, request, value, index, length, data, 7);
        cdc_acm_log_state("get_line_coding",
                          ",\"baud\":%u,\"stop_bits\":%u,\"parity\":%u"
                          ",\"data_bits\":%u",
                          s->baud, s->stop_bits, s->parity, s->data_bits);
        return;
    case ClassInterfaceOutRequest | CDC_ACM_SET_CONTROL_LINE:
        if (index != 3 && index != 4) {
            goto stall;
        }
        s->dtr = value & 1;
        s->rts = (value >> 1) & 1;
        cdc_acm_log_control(p, request, value, index, length, NULL, 0);
        cdc_acm_log_state("set_control_line_state", ",\"dtr\":%u,\"rts\":%u",
                          s->dtr, s->rts);
        return;
    case ClassInterfaceOutRequest | CDC_ACM_SEND_BREAK:
        cdc_acm_log_control(p, request, value, index, length, NULL, 0);
        cdc_acm_log_state("send_break", ",\"duration\":%u", value & 0xffff);
        return;
    case InterfaceRequest | USB_REQ_GET_INTERFACE:
        data[0] = 0;
        p->actual_length = 1;
        cdc_acm_log_control(p, request, value, index, length, data, 1);
        return;
    case InterfaceOutRequest | USB_REQ_SET_INTERFACE:
    case EndpointOutRequest | USB_REQ_CLEAR_FEATURE:
        cdc_acm_log_control(p, request, value, index, length, NULL, 0);
        return;
    default:
        break;
    }

stall:
    p->status = USB_RET_STALL;
    cdc_acm_log_control(p, request, value, index, length, data,
                        (request >> 8) & USB_DIR_IN ? p->actual_length
                                                    : length);
}

static void cdc_acm_handle_data(USBDevice *dev, USBPacket *p)
{
    USBCDCACMState *s = USB_CDC_ACM(dev);
    uint8_t ep = p->ep->nr;
    uint8_t buf[CDC_ACM_RX_SIZE];
    size_t len;

    if (p->pid == USB_TOKEN_OUT &&
        (ep == CDC_ACM_EP_BULK_OUT || ep == CDC_ACM_EP_SECONDARY_BULK_OUT)) {
        len = usb_packet_size(p);
        if (len > sizeof(buf)) {
            p->status = USB_RET_STALL;
            return;
        }
        usb_packet_copy(p, buf, len);
        p->actual_length = len;
        cdc_acm_log_bytes("bulk", "out", buf, len);
        qemu_chr_fe_write_all(&s->chr, buf, len);
        return;
    }

    if (p->pid == USB_TOKEN_IN &&
        (ep == CDC_ACM_EP_BULK_IN || ep == CDC_ACM_EP_SECONDARY_BULK_IN)) {
        uint8_t out[CDC_ACM_RX_SIZE];

        if (!s->rx_count) {
            p->status = USB_RET_NAK;
            return;
        }
        len = MIN((uint32_t)usb_packet_size(p), s->rx_count);
        for (size_t i = 0; i < len; i++) {
            out[i] = s->rx_buf[(s->rx_head + i) % CDC_ACM_RX_SIZE];
        }
        usb_packet_copy(p, out, len);
        s->rx_head = (s->rx_head + len) % CDC_ACM_RX_SIZE;
        s->rx_count -= len;
        p->actual_length = len;
        cdc_acm_log_bytes("bulk", "in", out, len);
        return;
    }

    if (p->pid == USB_TOKEN_IN &&
        (ep == CDC_ACM_EP_INTERRUPT || ep == CDC_ACM_EP_INTERRUPT_SECONDARY)) {
        uint8_t notification[CDC_ACM_NOTIFICATION_SIZE] = {
            0xa1, CDC_ACM_SERIAL_STATE, 0, 0, 0, 0, 2, 0, 0, 0,
        };

        if (!s->serial_state_pending) {
            p->status = USB_RET_NAK;
            return;
        }
        stw_le_p(notification + 8, s->serial_state);
        usb_packet_copy(p, notification, sizeof(notification));
        p->actual_length = sizeof(notification);
        s->serial_state_pending = false;
        cdc_acm_log_bytes("interrupt", "in", notification,
                          sizeof(notification));
        cdc_acm_log_state(
            "serial_state_sent", ",\"dcd\":%u,\"dsr\":%u,\"bits\":%u",
            !!(s->serial_state & CDC_ACM_RX_CARRIER),
            !!(s->serial_state & CDC_ACM_TX_CARRIER), s->serial_state);
        return;
    }

    p->status = USB_RET_STALL;
}

static int cdc_acm_can_read(void *opaque)
{
    USBCDCACMState *s = opaque;

    return CDC_ACM_RX_SIZE - s->rx_count;
}

static void cdc_acm_read(void *opaque, const uint8_t *buf, int size)
{
    USBCDCACMState *s = opaque;
    uint32_t tail = (s->rx_head + s->rx_count) % CDC_ACM_RX_SIZE;
    uint32_t first;

    size = MIN(size, CDC_ACM_RX_SIZE - s->rx_count);
    first = MIN((uint32_t)size, CDC_ACM_RX_SIZE - tail);
    memcpy(s->rx_buf + tail, buf, first);
    if ((uint32_t)size > first) {
        memcpy(s->rx_buf, buf + first, size - first);
    }
    s->rx_count += size;
    cdc_acm_log_bytes("chardev_rx", "host_to_guest", buf, size);
    usb_wakeup(s->bulk_in, 0);
    usb_wakeup(s->bulk_in_secondary, 0);
}

static void cdc_acm_event(void *opaque, QEMUChrEvent event)
{
    (void)opaque;
    if (event == CHR_EVENT_BREAK) {
        cdc_acm_log("chardev_break", "");
    }
}

static void cdc_acm_realize(USBDevice *dev, Error **errp)
{
    USBCDCACMState *s = USB_CDC_ACM(dev);

    usb_desc_init(dev);
    if (!qemu_chr_fe_backend_connected(&s->chr)) {
        error_setg(errp, "Property chardev is required");
        return;
    }
    s->intr = usb_ep_get(dev, USB_TOKEN_IN, CDC_ACM_EP_INTERRUPT);
    s->intr_secondary =
        usb_ep_get(dev, USB_TOKEN_IN, CDC_ACM_EP_INTERRUPT_SECONDARY);
    s->bulk_in = usb_ep_get(dev, USB_TOKEN_IN, CDC_ACM_EP_BULK_IN);
    s->bulk_in_secondary =
        usb_ep_get(dev, USB_TOKEN_IN, CDC_ACM_EP_SECONDARY_BULK_IN);
    qemu_chr_fe_set_handlers(&s->chr, cdc_acm_can_read, cdc_acm_read,
                             cdc_acm_event, NULL, s, NULL, true);
    cdc_acm_log("attach", ",\"vid\":4505,\"pid\":26795");
}

static const VMStateDescription vmstate_usb_cdc_acm = {
    .name = "usb-cdc-acm",
    .unmigratable = 1,
};

static const Property cdc_acm_properties[] = {
    DEFINE_PROP_CHR("chardev", USBCDCACMState, chr),
};

static void cdc_acm_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    USBDeviceClass *uc = USB_DEVICE_CLASS(klass);

    (void)data;
    uc->realize = cdc_acm_realize;
    uc->handle_reset = cdc_acm_reset;
    uc->handle_control = cdc_acm_handle_control;
    uc->handle_data = cdc_acm_handle_data;
    uc->product_desc = "Sierra Wireless CDC ACM modem";
    uc->usb_desc = &cdc_acm_desc;
    dc->vmsd = &vmstate_usb_cdc_acm;
    device_class_set_props(dc, cdc_acm_properties);
    object_class_property_add(klass, "serial-state", "uint32",
                              cdc_acm_get_state, cdc_acm_set_state, NULL, NULL);
    object_class_property_set_description(
        klass, "serial-state",
        "CDC SERIAL_STATE bits: DCD bit 0 and DSR bit 1");
}

static const TypeInfo cdc_acm_type_info = {
    .name = TYPE_USB_CDC_ACM,
    .parent = TYPE_USB_DEVICE,
    .instance_size = sizeof(USBCDCACMState),
    .class_init = cdc_acm_class_init,
};

static void cdc_acm_register_types(void)
{
    type_register_static(&cdc_acm_type_info);
}

type_init(cdc_acm_register_types)
