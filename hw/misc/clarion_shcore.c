/*
 * Synthetic Clarion SH core initialization and CPUCOM transport model.
 *
 * Bluetooth replies are synthetic and only cover local stack startup. They do
 * not model a real SH core, Bluetooth controller, profiles, or radio traffic.
 */
#include "qemu/osdep.h"
#include "hw/misc/clarion_shcore.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/timer.h"

#define CLARION_SHCORE_SIZE 0x3c
#define CLARION_SHCORE_SHARED_SIZE 0x2000
#define CLARION_SHCORE_REGS (CLARION_SHCORE_SIZE / sizeof(uint32_t))
#define CLARION_SHCORE_START 0x04
#define CLARION_SHCORE_ZERO 0x08
#define CLARION_SHCORE_STATUS 0x1c
#define CLARION_SHCORE_POLL 0x30
#define CLARION_SHCORE_INIT_END 0x4
#define CLARION_SHCORE_RX_PENDING 0x2
#define CLARION_SHCORE_FATAL 0x40
#define CLARION_SHCORE_ACK 0x100
#define CLARION_SHCORE_INIT_DELAY_NS 100000000
#define CLARION_SHCORE_IRQ_DELAY_NS 1000000
#define CLARION_SHCORE_MAX_PACKET 64
#define CLARION_SHCORE_RX_QUEUE_SIZE 16
#define CLARION_SHCORE_RX_QUEUE_BYTES                                          \
    (CLARION_SHCORE_RX_QUEUE_SIZE * CLARION_SHCORE_MAX_PACKET)

struct ClarionSHCoreState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    MemoryRegion shared_iomem;
    qemu_irq irq;
    QEMUTimer *timer;
    QEMUTimer *transport_timer;
    QEMUTimer *rx_timer;
    uint32_t regs[CLARION_SHCORE_REGS];
    uint8_t shared[CLARION_SHCORE_SHARED_SIZE];
    uint8_t rx_packet[CLARION_SHCORE_MAX_PACKET];
    uint8_t rx_queue[CLARION_SHCORE_RX_QUEUE_BYTES];
    uint32_t rx_queue_lengths[CLARION_SHCORE_RX_QUEUE_SIZE];
    uint32_t rx_length;
    uint32_t rx_queue_head;
    uint32_t rx_queue_count;
    bool init_only;
    bool transport_irq_pending;
    bool tx_pending;
    bool tx_complete_deferred;
    bool rx_busy;
    bool irq_epilogue_pending;
};

static void clarion_shcore_update_irq(ClarionSHCoreState *s)
{
    uint32_t status = s->regs[CLARION_SHCORE_STATUS / 4];

    qemu_set_irq(s->irq, s->transport_irq_pending ||
                             (status & CLARION_SHCORE_INIT_END));
}

static void clarion_shcore_start_queued_rx(ClarionSHCoreState *s);

static void clarion_shcore_init_end(void *opaque)
{
    ClarionSHCoreState *s = opaque;

    s->regs[CLARION_SHCORE_STATUS / 4] |= CLARION_SHCORE_INIT_END;
    clarion_shcore_update_irq(s);
    qemu_log_mask(LOG_UNIMP, "clarion-shcore: init end status=%08x irq=1\n",
                  s->regs[CLARION_SHCORE_STATUS / 4]);
}

static uint64_t clarion_shcore_read(void *opaque, hwaddr offset, unsigned size)
{
    ClarionSHCoreState *s = opaque;
    uint32_t value = 0;

    if (size == 4 && offset < CLARION_SHCORE_SIZE && !(offset & 3)) {
        value = s->regs[offset / 4];
        if (offset == CLARION_SHCORE_ZERO) {
            value = 0;
        } else if (offset == CLARION_SHCORE_POLL) {
            value &= ~1U;
        }
    }
    qemu_log_mask(LOG_UNIMP,
                  "clarion-shcore: read offset=%02" HWADDR_PRIx
                  " size=%u value=%08x\n",
                  offset, size, value);
    return value;
}

static void clarion_shcore_queue_packet(ClarionSHCoreState *s,
                                        const uint8_t *packet, uint32_t length)
{
    uint32_t tail;

    if (s->rx_queue_count == CLARION_SHCORE_RX_QUEUE_SIZE) {
        qemu_log_mask(LOG_UNIMP, "clarion-shcore: RX queue full, drop len=%u\n",
                      length);
        return;
    }

    tail =
        (s->rx_queue_head + s->rx_queue_count) % CLARION_SHCORE_RX_QUEUE_SIZE;
    memcpy(s->rx_queue + tail * CLARION_SHCORE_MAX_PACKET, packet, length);
    s->rx_queue_lengths[tail] = length;
    s->rx_queue_count++;
    qemu_log_mask(LOG_UNIMP, "clarion-shcore: RX queued len=%u depth=%u\n",
                  length, s->rx_queue_count);
    clarion_shcore_start_queued_rx(s);
}

static void clarion_shcore_make_rx(ClarionSHCoreState *s, uint16_t opcode,
                                   const uint8_t *body, uint32_t body_length);

static void clarion_shcore_start_queued_rx(ClarionSHCoreState *s)
{
    uint32_t length;

    if (s->rx_busy || s->tx_pending || s->transport_irq_pending ||
        s->tx_complete_deferred || s->rx_queue_count == 0) {
        return;
    }
    length = s->rx_queue_lengths[s->rx_queue_head];
    memcpy(s->rx_packet,
           s->rx_queue + s->rx_queue_head * CLARION_SHCORE_MAX_PACKET, length);
    s->rx_length = length;
    s->rx_queue_lengths[s->rx_queue_head] = 0;
    s->rx_queue_head = (s->rx_queue_head + 1) % CLARION_SHCORE_RX_QUEUE_SIZE;
    s->rx_queue_count--;
    s->rx_busy = true;
    timer_mod(s->rx_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                               CLARION_SHCORE_IRQ_DELAY_NS);
}

static void clarion_shcore_make_rx(ClarionSHCoreState *s, uint16_t opcode,
                                   const uint8_t *body, uint32_t body_length)
{
    /* SYNTHETIC: minimal CPUCOM envelope and frame around the supplied body. */
    uint32_t frame_length = body_length + 6 + (body_length & 1);
    uint32_t payload_length = frame_length + 28;
    uint32_t total_length = payload_length + 12;
    uint8_t packet[CLARION_SHCORE_MAX_PACKET] = { 0 };

    if (total_length > sizeof(packet)) {
        qemu_log_mask(LOG_UNIMP,
                      "clarion-shcore: synthetic RX too large len=%u\n",
                      total_length);
        return;
    }

    stl_le_p(packet, total_length);
    stl_le_p(packet + 4, 1);
    stl_le_p(packet + 8, payload_length);
    stl_le_p(packet + 12, 0xca);
    stl_le_p(packet + 16, 0x6504);
    packet[36] = 1;
    packet[37] = 1;
    stw_le_p(packet + 40, frame_length / 2);
    packet[42] = opcode >= 0x1000 ? 0xa5 : 0x55;
    packet[43] = (body_length & 1) ? 1 : 2;
    stw_le_p(packet + 44, opcode);
    memcpy(packet + 46, body, body_length);

    clarion_shcore_queue_packet(s, packet, total_length);
}

static void clarion_shcore_rx_irq(void *opaque)
{
    ClarionSHCoreState *s = opaque;
    uint32_t status = s->regs[CLARION_SHCORE_STATUS / 4];
    uint16_t opcode = lduw_le_p(s->rx_packet + 44);

    memcpy(s->shared + 0xc00, s->rx_packet, s->rx_length);
    s->regs[CLARION_SHCORE_POLL / 4] = 0x82;
    s->regs[CLARION_SHCORE_STATUS / 4] =
        (status & CLARION_SHCORE_INIT_END) | CLARION_SHCORE_RX_PENDING;
    s->transport_irq_pending = true;
    clarion_shcore_update_irq(s);
    qemu_log_mask(LOG_UNIMP,
                  "clarion-shcore: RX issued t_ms=%" PRId64
                  " len=%u opcode=%04x task=65 function=6504"
                  " status=%08x irq=1\n",
                  qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL), s->rx_length, opcode,
                  s->regs[CLARION_SHCORE_STATUS / 4]);
}

static void clarion_shcore_transport_done(void *opaque)
{
    ClarionSHCoreState *s = opaque;
    uint32_t status = s->regs[CLARION_SHCORE_STATUS / 4];

    s->tx_pending = false;
    s->regs[CLARION_SHCORE_POLL / 4] &= ~1U;
    if (s->rx_busy) {
        s->tx_complete_deferred = true;
        qemu_log_mask(LOG_UNIMP,
                      "clarion-shcore: Tx completion deferred during RX\n");
        return;
    }
    s->regs[CLARION_SHCORE_STATUS / 4] = status & ~CLARION_SHCORE_FATAL;
    s->transport_irq_pending = true;
    s->irq_epilogue_pending = true;
    clarion_shcore_update_irq(s);
    qemu_log_mask(LOG_UNIMP, "clarion-shcore: Tx complete status=%08x irq=1\n",
                  s->regs[CLARION_SHCORE_STATUS / 4]);
}

static void clarion_shcore_receive_ack(ClarionSHCoreState *s)
{
    uint32_t status = s->regs[CLARION_SHCORE_STATUS / 4];

    s->transport_irq_pending = false;
    s->regs[CLARION_SHCORE_STATUS / 4] =
        status & ~(CLARION_SHCORE_RX_PENDING | CLARION_SHCORE_FATAL);
    s->regs[CLARION_SHCORE_POLL / 4] = 0;
    if (s->tx_pending) {
        s->regs[CLARION_SHCORE_POLL / 4] = 1;
    }
    s->rx_busy = false;
    s->rx_length = 0;
    s->irq_epilogue_pending = true;
    clarion_shcore_update_irq(s);
    qemu_log_mask(LOG_UNIMP, "clarion-shcore: RX acknowledged status=%08x\n",
                  s->regs[CLARION_SHCORE_STATUS / 4]);
}

static void clarion_shcore_process_tx(ClarionSHCoreState *s)
{
    uint32_t words[9];
    uint32_t total;
    uint32_t payload_length;
    uint32_t body_length;
    uint32_t task;
    uint32_t function;
    uint32_t opcode = 0xffff;
    uint8_t *body;
    uint8_t first[16] = { 0 };
    uint32_t first_length;
    int i;

    for (i = 0; i < 9; i++) {
        words[i] = ldl_le_p(s->shared + i * 4);
    }
    total = words[0];
    payload_length = words[2];
    task = words[3] >> 1;
    function = words[4];
    if (total < 36 || total > CLARION_SHCORE_SHARED_SIZE ||
        payload_length < 24 || payload_length > total - 12 ||
        total != payload_length + 12) {
        qemu_log_mask(LOG_UNIMP,
                      "clarion-shcore: invalid CPUCOM Tx total=%u"
                      " payload=%u\n",
                      total, payload_length);
        return;
    }

    body_length = total - 36;
    body = s->shared + 36;
    first_length = MIN(body_length, sizeof(first));
    memcpy(first, body, first_length);
    if (body_length >= 10 && body[0] == 1 && body[1] == 1) {
        opcode = lduw_le_p(body + 8);
    }
    qemu_log_mask(LOG_UNIMP,
                  "clarion-shcore: Tx t_ms=%" PRId64
                  " task=%02x function=%04x opcode=%04x len=%u"
                  " first16=%02x%02x%02x%02x%02x%02x%02x%02x"
                  "%02x%02x%02x%02x%02x%02x%02x%02x\n",
                  qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL), task, function, opcode,
                  body_length, first[0], first[1], first[2], first[3], first[4],
                  first[5], first[6], first[7], first[8], first[9], first[10],
                  first[11], first[12], first[13], first[14], first[15]);

    if (task != 0x65 || function != 0x6503 || body_length < 10 ||
        body[0] != 1 || body[1] != 1) {
        return;
    }

    switch (lduw_le_p(body + 8)) {
    case 0x1006: {
        if (body[6] != 0xa5) {
            break;
        }
        /* SYNTHETIC: success status; the frame builder supplies padding. */
        static const uint8_t cfm[] = { 0 };
        clarion_shcore_make_rx(s, 0x1006, cfm, sizeof(cfm));
        break;
    }
    case 0x1001: {
        if (body[6] != 0xa5) {
            break;
        }
        /* SYNTHETIC: success status; the frame builder supplies padding. */
        static const uint8_t cfm[] = { 0 };
        /* SYNTHETIC: zero metadata and a locally administered placeholder. */
        static const uint8_t app_init[] = {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00,
            0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
        };

        clarion_shcore_make_rx(s, 0x1001, cfm, sizeof(cfm));
        clarion_shcore_make_rx(s, 0x0030, app_init, sizeof(app_init));
        break;
    }
    case 0x1002: {
        if (body[6] != 0xa5) {
            break;
        }
        /* SYNTHETIC: success status; the frame builder supplies padding. */
        static const uint8_t cfm[] = { 0 };
        clarion_shcore_make_rx(s, 0x1002, cfm, sizeof(cfm));
        break;
    }
    case 0x0012: {
        if (body[6] != 0x55) {
            break;
        }
        /* SYNTHETIC: matched CFM; receiver does not inspect a status value. */
        static const uint8_t cfm[] = { 0 };

        clarion_shcore_make_rx(s, 0x0012, cfm, sizeof(cfm));
        break;
    }
    case 0x0009: {
        if (body[6] != 0x55) {
            break;
        }
        /* SYNTHETIC: matched CFM; receiver does not inspect a status value. */
        static const uint8_t cfm[] = { 0 };

        clarion_shcore_make_rx(s, 0x0009, cfm, sizeof(cfm));
        break;
    }
    case 0x002b: {
        if (body[6] != 0x55) {
            break;
        }
        /* SYNTHETIC: matched local SSP capability CFM; payload is ignored. */
        static const uint8_t cfm[] = { 0 };

        clarion_shcore_make_rx(s, 0x002b, cfm, sizeof(cfm));
        break;
    }
    default:
        break;
    }
}

static void clarion_shcore_write(void *opaque, hwaddr offset, uint64_t value,
                                 unsigned size)
{
    ClarionSHCoreState *s = opaque;

    qemu_log_mask(LOG_UNIMP,
                  "clarion-shcore: write offset=%02" HWADDR_PRIx
                  " size=%u value=%08" PRIx64 "\n",
                  offset, size, value);
    if (size != 4 || offset >= CLARION_SHCORE_SIZE || (offset & 3)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "clarion-shcore: invalid write offset=%02" HWADDR_PRIx
                      " size=%u value=%08" PRIx64 "\n",
                      offset, size, value);
        return;
    }

    if (offset == CLARION_SHCORE_STATUS && (value & CLARION_SHCORE_ACK)) {
        if (s->regs[CLARION_SHCORE_STATUS / 4] & CLARION_SHCORE_INIT_END) {
            s->regs[CLARION_SHCORE_STATUS / 4] &= ~CLARION_SHCORE_INIT_END;
            clarion_shcore_update_irq(s);
            qemu_log_mask(LOG_UNIMP, "clarion-shcore: init end acknowledged\n");
        } else {
            s->regs[CLARION_SHCORE_STATUS / 4] = 0;
            s->transport_irq_pending = false;
            if (s->irq_epilogue_pending) {
                s->irq_epilogue_pending = false;
                if (s->tx_complete_deferred) {
                    s->tx_complete_deferred = false;
                    s->transport_irq_pending = true;
                    s->irq_epilogue_pending = true;
                    clarion_shcore_update_irq(s);
                    qemu_log_mask(LOG_UNIMP,
                                  "clarion-shcore: deferred Tx completion\n");
                    return;
                }
                clarion_shcore_start_queued_rx(s);
            }
            clarion_shcore_update_irq(s);
        }
        return;
    }

    if (offset == CLARION_SHCORE_STATUS) {
        uint32_t init_end = s->regs[offset / 4] & CLARION_SHCORE_INIT_END;

        s->regs[offset / 4] = (value & ~CLARION_SHCORE_INIT_END) | init_end;
        clarion_shcore_update_irq(s);
        return;
    }

    s->regs[offset / 4] = value;
    if (offset == CLARION_SHCORE_START && (value & 1)) {
        qemu_log_mask(LOG_UNIMP, "clarion-shcore: init end armed delay_ns=%u\n",
                      CLARION_SHCORE_INIT_DELAY_NS);
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                                CLARION_SHCORE_INIT_DELAY_NS);
    } else if (offset == CLARION_SHCORE_POLL && (value & 0x83) == 0x83) {
        if (!s->init_only && !s->tx_pending) {
            s->tx_pending = true;
            clarion_shcore_process_tx(s);
            timer_mod(s->transport_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                          CLARION_SHCORE_IRQ_DELAY_NS);
        }
    } else if (offset == CLARION_SHCORE_POLL && (value & 1)) {
        if (!s->init_only && s->rx_busy) {
            clarion_shcore_receive_ack(s);
        }
    }
}

static uint64_t clarion_shcore_shared_read(void *opaque, hwaddr offset,
                                           unsigned size)
{
    ClarionSHCoreState *s = opaque;
    uint64_t value = 0;
    unsigned i;

    if (size > 4 || offset + size > CLARION_SHCORE_SHARED_SIZE) {
        return 0;
    }
    for (i = 0; i < size; i++) {
        value |= (uint64_t)s->shared[offset + i] << (i * 8);
    }
    return value;
}

static void clarion_shcore_shared_write(void *opaque, hwaddr offset,
                                        uint64_t value, unsigned size)
{
    ClarionSHCoreState *s = opaque;
    unsigned i;

    if (size > 4 || offset + size > CLARION_SHCORE_SHARED_SIZE) {
        qemu_log_mask(
            LOG_GUEST_ERROR,
            "clarion-shcore: invalid shared write offset=%04" HWADDR_PRIx
            " size=%u\n",
            offset, size);
        return;
    }
    for (i = 0; i < size; i++) {
        s->shared[offset + i] = value >> (i * 8);
    }
}

static const MemoryRegionOps clarion_shcore_ops = {
    .read = clarion_shcore_read,
    .write = clarion_shcore_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4, .unaligned = false },
    .impl = { .min_access_size = 4, .max_access_size = 4, .unaligned = false },
};

static const MemoryRegionOps clarion_shcore_shared_ops = {
    .read = clarion_shcore_shared_read,
    .write = clarion_shcore_shared_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4, .unaligned = true },
    .impl = { .min_access_size = 1, .max_access_size = 4, .unaligned = true },
};

static void clarion_shcore_reset(DeviceState *dev)
{
    ClarionSHCoreState *s = CLARION_SHCORE(dev);

    memset(s->regs, 0, sizeof(s->regs));
    memset(s->shared, 0, sizeof(s->shared));
    memset(s->rx_packet, 0, sizeof(s->rx_packet));
    memset(s->rx_queue, 0, sizeof(s->rx_queue));
    memset(s->rx_queue_lengths, 0, sizeof(s->rx_queue_lengths));
    s->rx_length = 0;
    s->rx_queue_head = 0;
    s->rx_queue_count = 0;
    s->transport_irq_pending = false;
    s->tx_pending = false;
    s->tx_complete_deferred = false;
    s->rx_busy = false;
    s->irq_epilogue_pending = false;
    timer_del(s->timer);
    timer_del(s->transport_timer);
    timer_del(s->rx_timer);
    qemu_set_irq(s->irq, 0);
}

static void clarion_shcore_realize(DeviceState *dev, Error **errp)
{
    ClarionSHCoreState *s = CLARION_SHCORE(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    (void)errp;
    memory_region_init_io(&s->iomem, OBJECT(s), &clarion_shcore_ops, s,
                          TYPE_CLARION_SHCORE, CLARION_SHCORE_SIZE);
    memory_region_init_io(&s->shared_iomem, OBJECT(s),
                          &clarion_shcore_shared_ops, s,
                          "clarion-shcore.shared", CLARION_SHCORE_SHARED_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_mmio(sbd, &s->shared_iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, clarion_shcore_init_end, s);
    s->transport_timer =
        timer_new_ns(QEMU_CLOCK_VIRTUAL, clarion_shcore_transport_done, s);
    s->rx_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, clarion_shcore_rx_irq, s);
}

static const VMStateDescription vmstate_clarion_shcore = {
    .name = TYPE_CLARION_SHCORE,
    .version_id = 3,
    .minimum_version_id = 3,
    .fields =
        (const VMStateField[]){
            VMSTATE_UINT32_ARRAY(regs, ClarionSHCoreState, CLARION_SHCORE_REGS),
            VMSTATE_TIMER_PTR(timer, ClarionSHCoreState),
            VMSTATE_TIMER_PTR_V(transport_timer, ClarionSHCoreState, 2),
            VMSTATE_TIMER_PTR_V(rx_timer, ClarionSHCoreState, 2),
            VMSTATE_UINT8_ARRAY_V(shared, ClarionSHCoreState,
                                  CLARION_SHCORE_SHARED_SIZE, 2),
            VMSTATE_UINT8_ARRAY_V(rx_packet, ClarionSHCoreState,
                                  CLARION_SHCORE_MAX_PACKET, 2),
            VMSTATE_UINT32_V(rx_length, ClarionSHCoreState, 2),
            VMSTATE_UINT8_ARRAY_V(rx_queue, ClarionSHCoreState,
                                  CLARION_SHCORE_RX_QUEUE_BYTES, 3),
            VMSTATE_UINT32_ARRAY_V(rx_queue_lengths, ClarionSHCoreState,
                                   CLARION_SHCORE_RX_QUEUE_SIZE, 3),
            VMSTATE_UINT32_V(rx_queue_head, ClarionSHCoreState, 3),
            VMSTATE_UINT32_V(rx_queue_count, ClarionSHCoreState, 3),
            VMSTATE_BOOL_V(tx_complete_deferred, ClarionSHCoreState, 3),
            VMSTATE_BOOL_V(transport_irq_pending, ClarionSHCoreState, 2),
            VMSTATE_BOOL_V(tx_pending, ClarionSHCoreState, 2),
            VMSTATE_BOOL_V(rx_busy, ClarionSHCoreState, 2),
            VMSTATE_BOOL_V(irq_epilogue_pending, ClarionSHCoreState, 2),
            VMSTATE_END_OF_LIST() }
};

static const Property clarion_shcore_properties[] = {
    DEFINE_PROP_BOOL("init-only", ClarionSHCoreState, init_only, false),
};

static void clarion_shcore_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    (void)data;
    dc->realize = clarion_shcore_realize;
    device_class_set_legacy_reset(dc, clarion_shcore_reset);
    dc->vmsd = &vmstate_clarion_shcore;
    device_class_set_props(dc, clarion_shcore_properties);
}

static const TypeInfo clarion_shcore_info = {
    .name = TYPE_CLARION_SHCORE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ClarionSHCoreState),
    .class_init = clarion_shcore_class_init,
};

static void clarion_shcore_register_types(void)
{
    type_register_static(&clarion_shcore_info);
}

type_init(clarion_shcore_register_types)
