/*
 * Bounded R-Car I2C4 model for the Clarion QY8 board.
 * Implements only the target accesses documented by project T142.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/irq.h"
#include "hw/i2c/i2c.h"
#include "hw/i2c/clarion_rcar_i2c.h"
#include "hw/misc/clarion_dispmicom.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "ui/input.h"

#define CLARION_LOG(mask, fmt, ...) \
    qemu_log_mask(mask, "virtual-ms=%" PRId64 " " fmt, \
                  qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL), ##__VA_ARGS__)

#define RCAR_I2C4_MMIO_SIZE 0x1000
#define TMA460_PROFILE_POINT_SIZE 10
#define TMA460_TOUCH_X_OFFSET 14
#define TMA460_TOUCH_Y_OFFSET 9
#define TMA460_TOUCH_REPORT_STALE_MS 500

#define ICMCR  0x04
#define ICMSR  0x0c
#define ICMIER 0x14
#define ICMAR  0x20
#define ICRXTX 0x24

#define ICMCR_FSDA BIT(5)
#define ICMCR_ESG  BIT(0)
#define ICMCR_MDBS BIT(7)
#define ICMCR_MIE  BIT(3)

#define ICMSR_MASK 0x7f
#define ICMSR_MNR  BIT(6)
#define ICMSR_MDE  BIT(3)
#define ICMSR_MDT  BIT(2)
#define ICMSR_MDR  BIT(1)
#define ICMSR_MAT  BIT(0)

typedef enum ClarionRcarI2C4Phase {
    RCAR_I2C4_IDLE,
    RCAR_I2C4_WRITE_ADDRESS,
    RCAR_I2C4_WRITE_DATA,
    RCAR_I2C4_READ_ADDRESS,
    RCAR_I2C4_READ_DATA_WAIT,
} ClarionRcarI2C4Phase;

struct ClarionRcarI2C4State {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    I2CBus *bus;
    qemu_irq irq;
    uint32_t icmcr;
    uint32_t icmsr;
    uint32_t icmier;
    uint32_t icmar;
    uint32_t icrxtx;
    ClarionRcarI2C4Phase phase;
    bool bus_active;
    bool icrxtx_written;
    bool read_last_pending;
};

struct ClarionI2C4Recorder {
    I2CSlave parent_obj;
};

struct ClarionTma460 {
    I2CSlave parent_obj;
    qemu_irq irq;
    uint8_t selector;
    uint8_t command[9];
    uint8_t command_len;
    uint8_t response_index;
    uint8_t response_len;
    bool selector_valid;
    bool command_valid;
    bool command_pending;
    bool response_active;
    bool exit_response_delivered;
    bool reset_released;
    bool ready_pulsed;
    bool synthetic_profile;
    bool ist_status_pending;
    QEMUTimer ist_irq_timer;
    bool system_header_active;
    bool profile_read_active;
    bool mode_write_pending;
    QemuInputHandlerState *pointer_input;
    int pointer_x;
    int pointer_y;
    bool pointer_valid;
    bool pointer_dirty;
    bool pointer_button_down;
    bool reported_button_down;
    bool reported_pointer_valid;
    bool ignore_pointer_until_release;
    uint16_t reported_pointer_x;
    uint16_t reported_pointer_y;
    bool touch_report_pending;
    int64_t touch_report_time;
    uint8_t touch_report[TMA460_PROFILE_POINT_SIZE];
    uint8_t mode_register;
    uint8_t mode_write_value;
};

#define TMA460_PROFILE_A1 0x20 /* SYNTHETIC: selected first block address. */
#define TMA460_PROFILE_L  0x10 /* TARGET: block length used by derived reads. */
#define TMA460_PROFILE_A2 0x4a /* SYNTHETIC: second block address. */
#define TMA460_PROFILE_A3 0x50 /* SYNTHETIC: third block address. */
#define TMA460_PROFILE_A4 0x60 /* SYNTHETIC: configuration block address. */
#define TMA460_PROFILE_A5 0x70 /* SYNTHETIC: cfg[0] selector. */
#define TMA460_PROFILE_A6 0x71 /* SYNTHETIC: cfg[1] selector. */
#define TMA460_PROFILE_TOUCH_COUNT 0x90 /* SYNTHETIC: separate count selector. */
#define TMA460_PROFILE_TOUCH_DATA (TMA460_PROFILE_TOUCH_COUNT + 1)
#define TMA460_PROFILE_MODE_ANY 0xff
#define TMA460_PROFILE_MODE_SYSTEM 0x10
#define TMA460_PROFILE_MODE_WORKING 0x00

G_STATIC_ASSERT(TMA460_PROFILE_TOUCH_DATA + 100 < 0x100);

/* TARGET: bit 0x10 selects System Mode; other fields satisfy parser constraints. */
static const uint8_t clarion_tma460_profile_sysinfo[] = {
    0x10, 0x00, 0x00, 0x01, 0x00, TMA460_PROFILE_A1,
    0x00, TMA460_PROFILE_A2, 0x00, TMA460_PROFILE_A3,
    0x00, TMA460_PROFILE_A4, 0x00, TMA460_PROFILE_A5,
    0x00, TMA460_PROFILE_A6,
};

typedef enum ClarionTma460ProfileData {
    TMA460_PROFILE_REGISTER_DATA,
    TMA460_PROFILE_MODE_DATA,
    TMA460_PROFILE_TOUCH_COUNT_DATA,
    TMA460_PROFILE_TOUCH_REPORT_DATA,
} ClarionTma460ProfileData;

typedef struct ClarionTma460ProfileMap {
    uint8_t selector;
    uint8_t mode;
    uint8_t length;
    ClarionTma460ProfileData kind;
    const uint8_t *data;
    const uint8_t *system_header;
    uint8_t system_header_length;
} ClarionTma460ProfileMap;

/* SYNTHETIC register bytes; TARGET constraints are marked at each field. */
static const uint8_t clarion_tma460_profile_registers[256] = {
    [TMA460_PROFILE_A1 + 0x12] = TMA460_PROFILE_L, /* TARGET: L input. */
    [TMA460_PROFILE_A4] = TMA460_PROFILE_A5, /* SYNTHETIC: cfg[0]. */
    [TMA460_PROFILE_A4 + 1] = TMA460_PROFILE_A6, /* SYNTHETIC: cfg[1]. */
    [TMA460_PROFILE_A4 + 4] = 0x00, /* TARGET: cfg[4] avoids extra reads. */
    [TMA460_PROFILE_A4 + 5] = TMA460_PROFILE_TOUCH_COUNT, /* SYNTHETIC: cfg[5]. */
    [TMA460_PROFILE_A4 + 6] = 0x00, /* TARGET: cfg[6] disables gesture path. */
    [TMA460_PROFILE_A4 + 8] = 0x0a, /* TARGET: cfg[8] is record stride. */
};

/* SYNTHETIC: minimum accepted selector-2/3 handshake response fields. */
static const uint8_t clarion_tma460_profile_handshake_status[10] = {
    [0] = 0x40,
};
static const uint8_t clarion_tma460_profile_handshake_data[0x80] = {
    [5] = 0x80,
    [0x1d] = 0x01,
};

static const uint8_t clarion_tma460_profile_zero[256];

/* One selector/length/data table serves System Mode and working-mode reads. */
static const ClarionTma460ProfileMap clarion_tma460_profile_map[] = {
    { 0x00, TMA460_PROFILE_MODE_ANY, 2, TMA460_PROFILE_MODE_DATA, NULL,
      clarion_tma460_profile_sysinfo, sizeof(clarion_tma460_profile_sysinfo) },
    { TMA460_PROFILE_A1, TMA460_PROFILE_MODE_SYSTEM,
      TMA460_PROFILE_A2 - TMA460_PROFILE_A1, TMA460_PROFILE_REGISTER_DATA,
      clarion_tma460_profile_registers + TMA460_PROFILE_A1 },
    { TMA460_PROFILE_A1 + 0x13, TMA460_PROFILE_MODE_SYSTEM,
      TMA460_PROFILE_L, TMA460_PROFILE_REGISTER_DATA,
      clarion_tma460_profile_registers + TMA460_PROFILE_A1 + 0x13 },
    { TMA460_PROFILE_A1 + TMA460_PROFILE_L + 0x15,
      TMA460_PROFILE_MODE_SYSTEM, 5, TMA460_PROFILE_REGISTER_DATA,
      clarion_tma460_profile_registers + TMA460_PROFILE_A1 +
          TMA460_PROFILE_L + 0x15 },
    { TMA460_PROFILE_A2, TMA460_PROFILE_MODE_SYSTEM, 2,
      TMA460_PROFILE_REGISTER_DATA,
      clarion_tma460_profile_registers + TMA460_PROFILE_A2 },
    { TMA460_PROFILE_A4 - 0x10, TMA460_PROFILE_MODE_SYSTEM, 0x0d,
      TMA460_PROFILE_REGISTER_DATA,
      clarion_tma460_profile_registers + TMA460_PROFILE_A4 - 0x10 },
    { TMA460_PROFILE_A4, TMA460_PROFILE_MODE_SYSTEM, 0x22,
      TMA460_PROFILE_REGISTER_DATA,
      clarion_tma460_profile_registers + TMA460_PROFILE_A4 },
    { TMA460_PROFILE_A5, TMA460_PROFILE_MODE_SYSTEM, 1,
      TMA460_PROFILE_REGISTER_DATA,
      clarion_tma460_profile_registers + TMA460_PROFILE_A5 },
    { TMA460_PROFILE_A6, TMA460_PROFILE_MODE_SYSTEM, 2,
      TMA460_PROFILE_REGISTER_DATA,
      clarion_tma460_profile_registers + TMA460_PROFILE_A6 },
    { 0x01, TMA460_PROFILE_MODE_ANY, 1, TMA460_PROFILE_REGISTER_DATA,
      clarion_tma460_profile_zero + 0x01 },
    { 0x02, TMA460_PROFILE_MODE_ANY, 10, TMA460_PROFILE_REGISTER_DATA,
      clarion_tma460_profile_handshake_status },
    { 0x03, TMA460_PROFILE_MODE_ANY, 0x80, TMA460_PROFILE_REGISTER_DATA,
      clarion_tma460_profile_handshake_data },
    { TMA460_PROFILE_A5, TMA460_PROFILE_MODE_WORKING, 1,
      TMA460_PROFILE_REGISTER_DATA,
      clarion_tma460_profile_registers + TMA460_PROFILE_A5 },
    { TMA460_PROFILE_A6, TMA460_PROFILE_MODE_WORKING, 2,
      TMA460_PROFILE_REGISTER_DATA,
      clarion_tma460_profile_registers + TMA460_PROFILE_A6 },
    { TMA460_PROFILE_A6 + 3, TMA460_PROFILE_MODE_WORKING, 10,
      TMA460_PROFILE_REGISTER_DATA,
      clarion_tma460_profile_zero + TMA460_PROFILE_A6 + 3 },
    { TMA460_PROFILE_TOUCH_COUNT, TMA460_PROFILE_MODE_WORKING, 1,
      TMA460_PROFILE_TOUCH_COUNT_DATA, NULL },
    { TMA460_PROFILE_TOUCH_DATA, TMA460_PROFILE_MODE_WORKING,
      TMA460_PROFILE_POINT_SIZE, TMA460_PROFILE_TOUCH_REPORT_DATA, NULL },
};

static const ClarionTma460ProfileMap *
clarion_tma460_profile_map_find(uint8_t selector, uint8_t mode)
{
    const ClarionTma460ProfileMap *fallback = NULL;
    size_t i;

    for (i = 0; i < ARRAY_SIZE(clarion_tma460_profile_map); i++) {
        const ClarionTma460ProfileMap *entry =
            &clarion_tma460_profile_map[i];

        if (entry->selector != selector) {
            continue;
        }
        if (entry->mode == mode) {
            return entry;
        }
        if (entry->mode == TMA460_PROFILE_MODE_ANY) {
            fallback = entry;
        }
    }
    return fallback;
}

static bool clarion_tma460_pointer_mode_ready(const ClarionTma460 *s)
{
    return s->synthetic_profile && s->exit_response_delivered &&
           s->mode_register == TMA460_PROFILE_MODE_WORKING;
}

static void clarion_tma460_delayed_ist_irq(void *opaque)
{
    ClarionTma460 *s = opaque;

    if (!s->synthetic_profile || !s->reset_released) {
        return;
    }

    qemu_irq_raise(s->irq);
    qemu_irq_lower(s->irq);
    CLARION_LOG(LOG_UNIMP,
                  "clarion-tma460: profile delayed IST IRQ pulse\n");
}

static void clarion_tma460_schedule_ist_irq(ClarionTma460 *s,
                                            int64_t delay_ms)
{
    timer_mod(&s->ist_irq_timer,
              qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + delay_ms);
}

static void clarion_tma460_pointer_sync_state(ClarionTma460 *s);

static const uint8_t clarion_tma460_enter_active[9] = {
    0x00, 0xff, 0x01, 0x38, 0x00, 0x00, 0xa0, 0x09, 0x17,
};

static const uint8_t clarion_tma460_exit_bootloader[9] = {
    0x00, 0xff, 0x01, 0x3b, 0x00, 0x00, 0x4f, 0x6d, 0x17,
};

/* Target-constrained selector 0x03 handshake payload. */
static const uint8_t clarion_tma460_profile_handshake_write[5] = {
    0x00, 0x00, 0x00, 0x80, 0x00,
};

static int clarion_i2c4_recorder_event(I2CSlave *slave, enum i2c_event event)
{
    const char *name;

    switch (event) {
    case I2C_START_SEND:
        name = "start(0x24, write)";
        break;
    case I2C_START_RECV:
        name = "start(0x24, read)";
        break;
    case I2C_FINISH:
        name = "finish";
        break;
    case I2C_NACK:
        name = "nack";
        break;
    default:
        name = "other-event";
        break;
    }

    CLARION_LOG(LOG_UNIMP, "clarion-i2c4-recorder: %s\n", name);
    return 0;
}

static int clarion_i2c4_recorder_send(I2CSlave *slave, uint8_t data)
{
    CLARION_LOG(LOG_UNIMP,
                  "clarion-i2c4-recorder: send(0x%02x) ACK\n", data);
    return 0;
}

static uint8_t clarion_i2c4_recorder_recv(I2CSlave *slave)
{
    CLARION_LOG(LOG_UNIMP,
                  "clarion-i2c4-recorder: ERROR unexpected recv; B must stop before data\n");
    return 0xff;
}

static void clarion_i2c4_recorder_class_init(ObjectClass *klass,
                                              const void *data)
{
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(klass);

    sc->event = clarion_i2c4_recorder_event;
    sc->send = clarion_i2c4_recorder_send;
    sc->recv = clarion_i2c4_recorder_recv;
}

static const TypeInfo clarion_i2c4_recorder_type_info = {
    .name = TYPE_CLARION_I2C4_RECORDER,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(ClarionI2C4Recorder),
    .class_init = clarion_i2c4_recorder_class_init,
};

static int clarion_tma460_event(I2CSlave *slave, enum i2c_event event)
{
    ClarionTma460 *s = CLARION_TMA460(slave);
    bool touch_report_consumed = false;

    switch (event) {
    case I2C_START_SEND:
        s->selector_valid = false;
        s->command_valid = false;
        s->command_len = 0;
        s->response_index = 0;
        s->response_active = false;
        s->profile_read_active = false;
        s->system_header_active = false;
        s->mode_write_pending = false;
        CLARION_LOG(LOG_UNIMP,
                      "clarion-tma460: start(0x24, write)\n");
        break;
    case I2C_START_RECV:
        s->response_index = 0;
        s->profile_read_active = false;
        s->system_header_active = false;
        s->response_active = s->command_pending && s->selector_valid &&
                             s->selector == 0x00 && s->command_len == 1;
        if (s->exit_response_delivered && s->selector_valid &&
            s->command_len == 1) {
            if (!s->synthetic_profile) {
                if (s->selector == 0x00) {
                    CLARION_LOG(LOG_UNIMP,
                                  "clarion-tma460: NACK unsupported read request selector=0x00 read_len=16 (length from target callsite; not present on I2C wire) after ExitBootloader response\n");
                    return 1;
                }
            } else {
                s->system_header_active = s->selector == 0x00 &&
                                     s->mode_register == 0x10;
                s->profile_read_active = true;
            }
        }
        if (!s->reset_released || !s->selector_valid || s->command_len != 1 ||
            (s->command_pending && !s->response_active)) {
            CLARION_LOG(LOG_UNIMP,
                          "clarion-tma460: NACK read start selector=0x%02x write_len=%u\n",
                          s->selector, s->command_len);
            return 1;
        }
        CLARION_LOG(LOG_UNIMP,
                      "clarion-tma460: start(0x24, read) selector=0x%02x write_len=%u command=%s\n",
                      s->selector, s->command_len,
                      s->response_active ? "EnterActiveState" : "selector-read");
        if (s->response_active) {
            s->response_len = s->command[3] == 0x3b ? 7 : 15;
        } else if (s->profile_read_active) {
            const ClarionTma460ProfileMap *entry =
                clarion_tma460_profile_map_find(s->selector, s->mode_register);

            if (!entry) {
                CLARION_LOG(LOG_UNIMP,
                              "clarion-tma460: NACK unsupported profile selector=0x%02x\n",
                              s->selector);
                return 1;
            }
            if (s->system_header_active) {
                s->response_len = entry->system_header_length;
            } else if (entry->kind == TMA460_PROFILE_TOUCH_REPORT_DATA) {
                s->response_len = s->touch_report_pending ? entry->length : 0;
            } else {
                s->response_len = entry->length;
            }
        } else {
            s->response_len = 1;
        }
        break;
    case I2C_FINISH:
        if (s->synthetic_profile &&
            ((s->selector == 0x02 && s->command_len == 2 &&
              s->command[1] == 0x03) ||
             (s->selector == 0x03 && s->command_len == 6))) {
            /* SYNTHETIC: satisfy the driver's bounded handshake completion. */
            s->ist_status_pending = true;
            clarion_tma460_schedule_ist_irq(s, 25);
            CLARION_LOG(LOG_UNIMP,
                          "clarion-tma460: handshake completion IRQ scheduled\n");
        }
        if (s->command_valid && s->command_len == sizeof(s->command)) {
            s->command_pending = true;
            if (s->synthetic_profile && s->command[3] == 0x3b) {
                s->mode_register = 0x10;
                s->ist_status_pending = true;
                /* The driver waits on [+0x1b0] after issuing this command. */
                clarion_tma460_schedule_ist_irq(s, 30);
                CLARION_LOG(LOG_UNIMP,
                              "clarion-tma460: ExitBootloader IST IRQ scheduled\n");
            }
        }
        if (s->response_active) {
            if (s->command[3] == 0x3b) {
                s->exit_response_delivered = true;
            }
            s->command_pending = false;
            s->response_active = false;
        }
        if (s->profile_read_active && s->touch_report_pending) {
            const ClarionTma460ProfileMap *entry =
                clarion_tma460_profile_map_find(s->selector,
                                                s->mode_register);

            if (entry && entry->kind == TMA460_PROFILE_TOUCH_REPORT_DATA &&
                s->response_index == s->response_len) {
                s->touch_report_pending = false;
                touch_report_consumed = true;
                CLARION_LOG(LOG_UNIMP,
                              "clarion-tma460: touch report consumed\n");
            }
        }
        if (s->mode_write_pending) {
            uint8_t mode = s->mode_write_value & 0x78;

            if (s->selector == 0x00 &&
                (mode == 0x08 || mode == 0x18 || mode == 0x28)) {
                /* TARGET: §2 register-0 command constants and readback modes. */
                s->mode_register = mode & 0x70;
                s->ist_status_pending = true;
                CLARION_LOG(LOG_UNIMP,
                              "clarion-tma460: profile register 0 mode write=0x%02x readback=0x%02x\n",
                              s->mode_write_value, s->mode_register);
                /* TARGET: §2 mode transition wakes event [+0x1ac]. */
                clarion_tma460_schedule_ist_irq(s, 25);
                CLARION_LOG(LOG_UNIMP,
                              "clarion-tma460: profile mode-change IRQ scheduled\n");
            }
            s->mode_write_pending = false;
        }
        if (touch_report_consumed) {
            clarion_tma460_pointer_sync_state(s);
        }
        CLARION_LOG(LOG_UNIMP, "clarion-tma460: finish\n");
        break;
    case I2C_NACK:
        CLARION_LOG(LOG_UNIMP, "clarion-tma460: nack\n");
        break;
    default:
        CLARION_LOG(LOG_UNIMP,
                      "clarion-tma460: unimplemented I2C event %d\n",
                      event);
        break;
    }
    return 0;
}

static int clarion_tma460_send(I2CSlave *slave, uint8_t data)
{
    ClarionTma460 *s = CLARION_TMA460(slave);

    if (!s->reset_released) {
        CLARION_LOG(LOG_UNIMP,
                      "clarion-tma460: NACK write byte=0x%02x reset_released=0\n",
                      data);
        return 1;
    }

    if (!s->command_len) {
        if (data != 0x00 && data != 0x01 &&
            !(s->synthetic_profile &&
              clarion_tma460_profile_map_find(data, s->mode_register))) {
            CLARION_LOG(LOG_UNIMP,
                          "clarion-tma460: NACK unsupported selector=0x%02x\n",
                          data);
            return 1;
        }
        s->selector = data;
        s->selector_valid = true;
        s->command[0] = data;
        s->command_len = 1;
        CLARION_LOG(LOG_UNIMP,
                      "clarion-tma460: selector 0x%02x ACK\n", data);
        return 0;
    }

    if (s->synthetic_profile && s->selector == 0x00 &&
        s->command_len == 1 &&
        ((data & 0x78) == 0x08 || (data & 0x78) == 0x18 ||
         (data & 0x78) == 0x28)) {
        s->mode_write_value = data;
        s->mode_write_pending = true;
        s->command[1] = data;
        s->command_len = 2;
        CLARION_LOG(LOG_UNIMP,
                      "clarion-tma460: profile register 0 write byte=0x%02x ACK\n",
                      data);
        return 0;
    }

    if (s->synthetic_profile && s->selector == 0x03 &&
        s->command_len >= 1 && s->command_len <= 5 &&
        data == clarion_tma460_profile_handshake_write[s->command_len - 1]) {
        /* TARGET: §2 handshake-enable transaction writes exactly 5 bytes. */
        s->command[s->command_len++] = data;
        CLARION_LOG(LOG_UNIMP,
                      "clarion-tma460: profile handshake selector 0x03 payload[%u]=0x%02x ACK\n",
                      s->command_len - 2, data);
        return 0;
    }

    if (s->synthetic_profile && s->selector == 0x02 &&
        s->command_len == 1 && data == 0x03) {
        /* TARGET: §2 handshake-enable transaction writes selector 2 = 3. */
        s->command[1] = data;
        s->command_len = 2;
        CLARION_LOG(LOG_UNIMP,
                      "clarion-tma460: profile handshake selector 0x02 payload=0x03 ACK\n");
        return 0;
    }

    const uint8_t *expected = s->command_len < sizeof(s->command) &&
                              s->command_len > 3 && s->command[3] == 0x3b ?
                              clarion_tma460_exit_bootloader :
                              clarion_tma460_enter_active;
    if (s->selector != 0x00 || s->command_len >= sizeof(s->command) ||
        (s->command_len == 3 ? (data != 0x38 && data != 0x3b) :
                               data != expected[s->command_len])) {
        CLARION_LOG(LOG_UNIMP,
                      "clarion-tma460: NACK command byte[%u]=0x%02x\n",
                      s->command_len, data);
        s->command_valid = false;
        return 1;
    }

    s->command[s->command_len++] = data;
    s->command_valid = s->command_len == sizeof(s->command);
    CLARION_LOG(LOG_UNIMP,
                  "clarion-tma460: command byte[%u]=0x%02x ACK\n",
                  s->command_len - 1, data);
    if (s->command_valid) {
        CLARION_LOG(LOG_UNIMP,
                      "clarion-tma460: Enter Active State command accepted\n");
    }
    return 0;
}

static uint8_t clarion_tma460_recv(I2CSlave *slave)
{
    ClarionTma460 *s = CLARION_TMA460(slave);

    if (!s->reset_released || !s->selector_valid ||
        (s->command_len != 1 && !s->command_valid)) {
        CLARION_LOG(LOG_UNIMP,
                      "clarion-tma460: ERROR read without supported selector\n");
        return 0xff;
    }

    if ((s->response_active || s->profile_read_active) &&
        s->response_index >= s->response_len) {
        CLARION_LOG(LOG_UNIMP,
                      "clarion-tma460: ERROR response overread index=%u\n",
                      s->response_index);
        return 0xff;
    }

    if (!s->response_active && !s->profile_read_active &&
        s->response_index >= 1) {
        CLARION_LOG(LOG_UNIMP,
                      "clarion-tma460: ERROR selector response overread index=%u\n",
                      s->response_index);
        return 0xff;
    }

    if (s->synthetic_profile && s->profile_read_active) {
        const ClarionTma460ProfileMap *entry =
            clarion_tma460_profile_map_find(s->selector, s->mode_register);
        uint8_t value = 0;

        if (s->selector == 0x00 && s->response_index == 0 &&
            s->ist_status_pending) {
            /* SYNTHETIC: IST accepts 1 before signaling [+0x1ac]/[+0x1b0]. */
            value = 1;
            s->ist_status_pending = false;
        } else if (s->system_header_active && entry && entry->system_header) {
            value = entry->system_header[s->response_index];
        } else if (entry && entry->kind == TMA460_PROFILE_MODE_DATA) {
            value = s->response_index == 0 ? s->mode_register : 0;
        } else if (entry &&
                   entry->kind == TMA460_PROFILE_TOUCH_COUNT_DATA) {
            value = s->touch_report_pending ? 1 : 0;
        } else if (entry &&
                   entry->kind == TMA460_PROFILE_TOUCH_REPORT_DATA &&
                   s->touch_report_pending) {
            value = s->touch_report[s->response_index];
        } else if (entry && entry->data) {
            value = entry->data[s->response_index];
        }
        CLARION_LOG(LOG_UNIMP,
                      "clarion-tma460: profile read selector=0x%02x response[%u]=0x%02x\n",
                      s->selector, s->response_index, value);
        s->response_index++;
        return value;
    }

    /* T143 selector reads and the T144 checked fields are zero. */
    CLARION_LOG(LOG_UNIMP,
                  "clarion-tma460: read selector=0x%02x response[%u]=0x00\n",
                  s->selector, s->response_index);
    s->response_index++;
    return 0x00;
}

static bool clarion_tma460_emit_touch(ClarionTma460 *s, uint8_t event_id,
                                      int pointer_x, int pointer_y)
{
    uint16_t x = pointer_x + TMA460_TOUCH_X_OFFSET;
    uint16_t y = pointer_y + TMA460_TOUCH_Y_OFFSET;

    if (!clarion_tma460_pointer_mode_ready(s)) {
        CLARION_LOG(LOG_UNIMP,
                      "clarion-tma460: pointer report ignored before working mode (profile=%d exit_response=%d mode=0x%02x)\n",
                      s->synthetic_profile, s->exit_response_delivered,
                      s->mode_register);
        return false;
    }
    if (s->touch_report_pending) {
        CLARION_LOG(LOG_UNIMP,
                      "clarion-tma460: pointer report deferred while prior report is unread\n");
        return false;
    }

    memset(s->touch_report, 0, sizeof(s->touch_report));
    s->touch_report[0] = x >> 8;
    s->touch_report[1] = x;
    s->touch_report[2] = y >> 8;
    s->touch_report[3] = y;
    s->touch_report[4] = 0; /* SYNTHETIC: reserved report byte. */
    s->touch_report[5] = event_id << 4; /* TARGET: EVTID occupies bits 5:4. */
    s->touch_report_pending = true;
    s->touch_report_time = qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL);
    CLARION_LOG(LOG_UNIMP,
                  "clarion-tma460: pointer report evt=%u x=%d y=%d raw_x=%u raw_y=%u\n",
                  event_id, pointer_x, pointer_y, x, y);
    qemu_irq_raise(s->irq);
    qemu_irq_lower(s->irq);
    return true;
}

static void clarion_tma460_pointer_event(DeviceState *dev, QemuConsole *src,
                                         QemuInputEvent *evt)
{
    ClarionTma460 *s = CLARION_TMA460(dev);

    switch (evt->type) {
    case INPUT_EVENT_KIND_ABS:
        if (evt->abs.axis == INPUT_AXIS_X) {
            s->pointer_x = qemu_input_scale_axis(evt->abs.value,
                                                 INPUT_EVENT_ABS_MIN,
                                                 INPUT_EVENT_ABS_MAX,
                                                 0, 799);
            s->pointer_valid = true;
            s->pointer_dirty = true;
        } else if (evt->abs.axis == INPUT_AXIS_Y) {
            /* kepdrv.dll mirrors Y before it posts WM_LBUTTON* */
            s->pointer_y = 479 - qemu_input_scale_axis(evt->abs.value,
                                                       INPUT_EVENT_ABS_MIN,
                                                       INPUT_EVENT_ABS_MAX,
                                                       0, 479);
            s->pointer_valid = true;
            s->pointer_dirty = true;
        }
        break;
    case INPUT_EVENT_KIND_BTN:
        if (evt->btn.button == INPUT_BUTTON_LEFT &&
            s->pointer_button_down != evt->btn.down) {
            s->pointer_button_down = evt->btn.down;
            s->pointer_dirty = true;
        } else if (evt->btn.down &&
                   (evt->btn.button == INPUT_BUTTON_WHEEL_UP ||
                    evt->btn.button == INPUT_BUTTON_WHEEL_DOWN)) {
            clarion_dispmicom_wheel(evt->btn.button == INPUT_BUTTON_WHEEL_UP ?
                                    1 : -1);
        }
        break;
    default:
        break;
    }
}

static void clarion_tma460_pointer_sync_state(ClarionTma460 *s)
{
    uint8_t event_id;

    if (!clarion_tma460_pointer_mode_ready(s)) {
        s->pointer_dirty = false;
        s->reported_button_down = false;
        s->reported_pointer_valid = false;
        s->ignore_pointer_until_release = s->pointer_button_down;
        return;
    }
    /*
     * Early in boot the driver acks the IRQ without reading the report;
     * drop it rather than hold back every later one.
     */
    if (s->touch_report_pending &&
        qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) - s->touch_report_time >
            TMA460_TOUCH_REPORT_STALE_MS) {
        CLARION_LOG(LOG_UNIMP,
                      "clarion-tma460: dropping unread touch report\n");
        s->touch_report_pending = false;
        s->reported_button_down = false;
        s->reported_pointer_valid = false;
    }
    if (s->ignore_pointer_until_release) {
        s->pointer_dirty = false;
        s->reported_button_down = false;
        s->reported_pointer_valid = false;
        if (!s->pointer_button_down) {
            s->ignore_pointer_until_release = false;
        }
        return;
    }
    if (!s->pointer_dirty || !s->pointer_valid || s->touch_report_pending) {
        return;
    }

    if (s->pointer_button_down && !s->reported_button_down) {
        event_id = 1;
    } else if (!s->pointer_button_down && s->reported_button_down) {
        event_id = 3;
    } else if (s->pointer_button_down && s->reported_button_down &&
               s->reported_pointer_valid &&
               (s->reported_pointer_x !=
                    s->pointer_x + TMA460_TOUCH_X_OFFSET ||
                s->reported_pointer_y !=
                    s->pointer_y + TMA460_TOUCH_Y_OFFSET)) {
        event_id = 2;
    } else {
        event_id = 0;
    }
    if (!event_id) {
        s->pointer_dirty = false;
        return;
    }
    if (clarion_tma460_emit_touch(s, event_id, s->pointer_x,
                                   s->pointer_y)) {
        s->pointer_dirty = false;
        s->reported_button_down = s->pointer_button_down;
        s->reported_pointer_x = s->pointer_x + TMA460_TOUCH_X_OFFSET;
        s->reported_pointer_y = s->pointer_y + TMA460_TOUCH_Y_OFFSET;
        s->reported_pointer_valid = true;
    }
}

static void clarion_tma460_pointer_sync(DeviceState *dev)
{
    clarion_tma460_pointer_sync_state(CLARION_TMA460(dev));
}

static const QemuInputHandler clarion_tma460_pointer_handler = {
    .name = "Clarion TMA460 absolute pointer",
    .mask = INPUT_EVENT_MASK_ABS | INPUT_EVENT_MASK_BTN,
    .event = clarion_tma460_pointer_event,
    .sync = clarion_tma460_pointer_sync,
};

void clarion_tma460_bind_pointer_input(DeviceState *dev,
                                       const char *display_id,
                                       Error **errp)
{
    ERRP_GUARD();
    ClarionTma460 *s = CLARION_TMA460(dev);

    if (!s->synthetic_profile || s->pointer_input) {
        return;
    }
    s->pointer_input = qemu_input_handler_register(dev,
                                      &clarion_tma460_pointer_handler);
    qemu_input_handler_bind(s->pointer_input, display_id, 0, errp);
    if (*errp) {
        qemu_input_handler_unregister(s->pointer_input);
        s->pointer_input = NULL;
    }
}

static void clarion_tma460_instance_init(Object *obj)
{
    ClarionTma460 *s = CLARION_TMA460(obj);

    s->reset_released = false;
    s->ready_pulsed = false;
    timer_init_ms(&s->ist_irq_timer, QEMU_CLOCK_VIRTUAL,
                  clarion_tma460_delayed_ist_irq, s);
    qdev_init_gpio_out(DEVICE(s), &s->irq, 1);
}

static void clarion_tma460_class_init(ObjectClass *klass, const void *data)
{
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(klass);

    sc->event = clarion_tma460_event;
    sc->send = clarion_tma460_send;
    sc->recv = clarion_tma460_recv;
}

static const TypeInfo clarion_tma460_type_info = {
    .name = TYPE_CLARION_TMA460,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(ClarionTma460),
    .instance_init = clarion_tma460_instance_init,
    .class_init = clarion_tma460_class_init,
};

void clarion_tma460_set_reset(DeviceState *dev, bool gpio_level)
{
    ClarionTma460 *s = CLARION_TMA460(dev);
    bool release = gpio_level;

    if (s->reset_released == release) {
        return;
    }
    s->reset_released = release;
    s->selector_valid = false;
    CLARION_LOG(LOG_UNIMP,
                  "clarion-tma460: reset %s (GPIO4.OUTDT bit10=%d)\n",
                  release ? "release" : "assert", gpio_level);

    if (!release) {
        timer_del(&s->ist_irq_timer);
        s->ready_pulsed = false;
        s->exit_response_delivered = false;
        s->pointer_valid = false;
        s->pointer_x = 0;
        s->pointer_y = 0;
        s->pointer_dirty = false;
        s->pointer_button_down = false;
        s->reported_button_down = false;
        s->reported_pointer_valid = false;
        s->ignore_pointer_until_release = false;
        s->reported_pointer_x = 0;
        s->reported_pointer_y = 0;
        s->touch_report_pending = false;
        memset(s->touch_report, 0, sizeof(s->touch_report));
        s->mode_register = 0; /* SYNTHETIC: chosen initial register-0 status. */
        s->ist_status_pending = s->synthetic_profile;
        return;
    }

    if (!s->ready_pulsed) {
        s->ready_pulsed = true;
        s->ist_status_pending = s->synthetic_profile;
        CLARION_LOG(LOG_UNIMP,
                      "clarion-tma460: ready IRQ scheduled after reset release\n");
        if (s->synthetic_profile) {
            clarion_tma460_schedule_ist_irq(s, 1);
        } else {
            qemu_irq_raise(s->irq);
            qemu_irq_lower(s->irq);
        }
    }
}

void clarion_tma460_set_synthetic_profile(DeviceState *dev, bool enabled)
{
    ClarionTma460 *s = CLARION_TMA460(dev);

    s->synthetic_profile = enabled;
    if (enabled) {
        CLARION_LOG(LOG_UNIMP,
                      "clarion-tma460: profile enabled\n");
    }
}

static void clarion_rcar_i2c4_update_irq(ClarionRcarI2C4State *s)
{
    bool level = !!(s->icmsr & s->icmier & ICMSR_MASK);

    qemu_set_irq(s->irq, level);
}

static void clarion_rcar_i2c4_start(ClarionRcarI2C4State *s)
{
    uint8_t address = (s->icmar >> 1) & 0x7f;
    bool read = !!(s->icmar & 1);
    int ret;

    ret = i2c_start_transfer(s->bus, address, read);
    /* QEMU's i2c_start_transfer() returns 1 when no slave ACKs. */
    if (ret != 0) {
        s->icmsr |= ICMSR_MNR;
        s->phase = RCAR_I2C4_IDLE;
        s->bus_active = false;
        CLARION_LOG(LOG_UNIMP,
                      "clarion-i2c4: address phase NACK addr=0x%02x %s\n",
                      address, read ? "read" : "write");
    } else {
        s->icmsr |= ICMSR_MAT;
        s->bus_active = true;
        s->phase = read ? RCAR_I2C4_READ_ADDRESS :
                          RCAR_I2C4_WRITE_ADDRESS;
        CLARION_LOG(LOG_UNIMP,
                      "clarion-i2c4: address phase ACK addr=0x%02x %s\n",
                      address, read ? "read" : "write");
    }
    clarion_rcar_i2c4_update_irq(s);
}

static uint64_t clarion_rcar_i2c4_read(void *opaque, hwaddr offset,
                                       unsigned size)
{
    ClarionRcarI2C4State *s = opaque;
    uint32_t value;

    switch (offset) {
    case ICMCR:
        /* FSDA reads as low while the modeled bus is idle. */
        value = s->icmcr & ~ICMCR_FSDA;
        break;
    case ICMSR:
        value = s->icmsr;
        break;
    case ICMIER:
        value = s->icmier;
        break;
    case ICMAR:
        value = s->icmar;
        break;
    case ICRXTX:
        value = s->icrxtx;
        break;
    default:
        CLARION_LOG(LOG_UNIMP,
                      "clarion-i2c4: unimplemented read offset=0x%03"
                      HWADDR_PRIx " size=%u\n",
                      offset, size);
        return 0;
    }

    CLARION_LOG(LOG_UNIMP,
                  "clarion-i2c4: read +0x%02" HWADDR_PRIx " = 0x%08x\n",
                  offset, value);
    return value;
}

static void clarion_rcar_i2c4_write(void *opaque, hwaddr offset,
                                    uint64_t value, unsigned size)
{
    ClarionRcarI2C4State *s = opaque;
    uint32_t v = value;

    CLARION_LOG(LOG_UNIMP,
                  "clarion-i2c4: write +0x%02" HWADDR_PRIx " = 0x%08x\n",
                  offset, v);
    switch (offset) {
    case ICMCR:
        s->icmcr = v;
        if ((v & (ICMCR_MDBS | ICMCR_MIE | ICMCR_ESG)) ==
            (ICMCR_MDBS | ICMCR_MIE | ICMCR_ESG)) {
            clarion_rcar_i2c4_start(s);
        } else if (s->phase == RCAR_I2C4_WRITE_ADDRESS &&
                   !(v & ICMCR_ESG) && (s->icmsr & ICMSR_MAT)) {
            /* Target row 0xefa828cc clears ESG on the first MAT. */
            s->phase = RCAR_I2C4_WRITE_DATA;
        }
        break;
    case ICMSR:
        /* R-Car master status is write-zero-to-clear. */
        {
            bool mat_cleared = (s->icmsr & ICMSR_MAT) &&
                               !(v & ICMSR_MAT);
            bool send_ready_cleared =
                (s->icmsr & (ICMSR_MDE | ICMSR_MDT)) ==
                    (ICMSR_MDE | ICMSR_MDT) &&
                !(v & ICMSR_MDE) && !(v & ICMSR_MDT);
            bool receive_ready_cleared = (s->icmsr & ICMSR_MDR) &&
                                         !(v & ICMSR_MDR);

            /* Apply the target's W0C before modeling the hardware transition. */
            s->icmsr &= v | ~ICMSR_MASK;

            if (s->phase == RCAR_I2C4_WRITE_DATA && mat_cleared) {
                int ret = i2c_send(s->bus, s->icrxtx);
                s->icrxtx_written = false;

                if (ret) {
                    s->icmsr |= ICMSR_MNR;
                    CLARION_LOG(LOG_UNIMP,
                                  "clarion-i2c4: ICRXTX send NACK byte=0x%02x\n",
                                  s->icrxtx);
                } else {
                    s->icmsr |= ICMSR_MDE | ICMSR_MDT;
                    CLARION_LOG(LOG_UNIMP,
                              "clarion-i2c4: W0C MAT -> send ICRXTX=0x%02x; set MDE|MDT\n",
                              s->icrxtx);
                }
            } else if (s->phase == RCAR_I2C4_WRITE_DATA &&
                       send_ready_cleared && s->icrxtx_written) {
                int ret = i2c_send(s->bus, s->icrxtx);
                s->icrxtx_written = false;

                if (ret) {
                    s->icmsr |= ICMSR_MNR;
                    CLARION_LOG(LOG_UNIMP,
                                  "clarion-i2c4: MDE|MDT send NACK byte=0x%02x\n",
                                  s->icrxtx);
                } else {
                    s->icmsr |= ICMSR_MDE | ICMSR_MDT;
                    CLARION_LOG(LOG_UNIMP,
                              "clarion-i2c4: W0C MDE|MDT -> send ICRXTX=0x%02x; set MDE|MDT\n",
                              s->icrxtx);
                }
            } else if (s->phase == RCAR_I2C4_WRITE_DATA &&
                       send_ready_cleared && !s->icrxtx_written &&
                       (s->icmcr & BIT(1))) {
                i2c_end_transfer(s->bus);
                s->bus_active = false;
                s->phase = RCAR_I2C4_IDLE;
                s->icmsr |= BIT(4); /* MST: write STOP completion. */
                CLARION_LOG(LOG_UNIMP,
                              "clarion-i2c4: write FSB completed; set MST\n");
            } else if (s->phase == RCAR_I2C4_READ_ADDRESS && mat_cleared) {
                s->phase = RCAR_I2C4_READ_DATA_WAIT;
                s->icrxtx = i2c_recv(s->bus);
                s->icmsr |= ICMSR_MDR;
                s->read_last_pending = !!(s->icmcr & BIT(1));
                CLARION_LOG(LOG_UNIMP,
                              "clarion-i2c4: read byte supplied=0x%02x; set MDR\n",
                              s->icrxtx);
            }

            if (s->phase == RCAR_I2C4_READ_DATA_WAIT &&
                receive_ready_cleared) {
                if (s->read_last_pending) {
                    i2c_end_transfer(s->bus);
                    s->bus_active = false;
                    s->phase = RCAR_I2C4_IDLE;
                    s->read_last_pending = false;
                    s->icmsr |= BIT(4); /* MST: FSB completion. */
                    CLARION_LOG(LOG_UNIMP,
                                  "clarion-i2c4: FSB completed; set MST\n");
                } else {
                    s->icrxtx = i2c_recv(s->bus);
                    s->icmsr |= ICMSR_MDR;
                    s->read_last_pending = !!(s->icmcr & BIT(1));
                    CLARION_LOG(LOG_UNIMP,
                                  "clarion-i2c4: MDR clear without FSB -> read byte 0x%02x; set MDR\n",
                                  s->icrxtx);
                }
            }
        }
        clarion_rcar_i2c4_update_irq(s);
        break;
    case ICMIER:
        s->icmier = v & ICMSR_MASK;
        clarion_rcar_i2c4_update_irq(s);
        break;
    case ICMAR:
        s->icmar = v;
        break;
    case ICRXTX:
        s->icrxtx = v & 0xff;
        s->icrxtx_written = true;
        break;
    default:
        CLARION_LOG(LOG_UNIMP,
                      "clarion-i2c4: unimplemented write offset=0x%03"
                      HWADDR_PRIx " value=0x%08x\n",
                      offset, v);
        break;
    }
}

static const MemoryRegionOps clarion_rcar_i2c4_ops = {
    .read = clarion_rcar_i2c4_read,
    .write = clarion_rcar_i2c4_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void clarion_rcar_i2c4_realize(DeviceState *dev, Error **errp)
{
    ClarionRcarI2C4State *s = CLARION_RCAR_I2C4(dev);

    s->bus = i2c_init_bus(dev, "i2c");
    memory_region_init_io(&s->iomem, OBJECT(dev), &clarion_rcar_i2c4_ops, s,
                          TYPE_CLARION_RCAR_I2C4, RCAR_I2C4_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
}

static void clarion_rcar_i2c4_class_init(ObjectClass *klass,
                                         const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = clarion_rcar_i2c4_realize;
}

static const TypeInfo clarion_rcar_i2c4_type_info = {
    .name = TYPE_CLARION_RCAR_I2C4,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ClarionRcarI2C4State),
    .class_init = clarion_rcar_i2c4_class_init,
};

static void clarion_rcar_i2c4_register_types(void)
{
    type_register_static(&clarion_rcar_i2c4_type_info);
    type_register_static(&clarion_i2c4_recorder_type_info);
    type_register_static(&clarion_tma460_type_info);
}

type_init(clarion_rcar_i2c4_register_types)
