/* Clarion QY7 bootloader machine. */
#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "hw/core/boards.h"
#include "system/block-backend.h"
#include "hw/block/flash.h"
#include "hw/char/clarion_scif.h"
#include "hw/char/clarion_qy7_uart.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/dma/clarion_hpbdma.h"
#include "hw/misc/unimp.h"
#include "hw/misc/clarion_qy7_regs.h"
#include "hw/misc/clarion_usbphy.h"
#include "hw/misc/clarion_dispmicom.h"
#include "hw/misc/clarion_micom.h"
#include "hw/misc/clarion_scpu.h"
#include "hw/misc/clarion_rcan.h"
#include "hw/misc/clarion_bcm.h"
#include "hw/misc/clarion_boards.h"
#include "hw/display/clarion_du.h"
#include "hw/display/clarion_2dg.h"
#include "hw/sh4/clarion_sh4_core.h"
#include "hw/sh4/sh.h"
#include "hw/intc/clarion_qy7_int2.h"
#include "hw/timer/tmu012.h"
#include "hw/sd/renesas_sdhi.h"
#include "hw/sd/sd.h"
#include "hw/core/irq.h"
#include "target/sh4/cpu-qom.h"
#include "target/sh4/cpu.h"
#include "qom/object.h"
#include "system/address-spaces.h"
#include "system/reset.h"
#include "system/system.h"

#define QY7_FLASH_SIZE (8 * MiB)
#define QY7_RAM_BASE 0x08000000
/*
 * The kernel's memory table (NOR 0x2acc90) is a list of (base, size)
 * pairs in P1: main RAM, the NK2 image, a 1 MiB block at 0x8e600000 and
 * a 25 MiB block at 0x8e700000 that KernelIoControl hands to the display
 * drivers, and a second bank (0x90000000, 0x08000000). At run time the
 * kernel builds a free-memory section for that bank with its page-use
 * map at 0x90000000, so RAM covers physical 0x08000000..0x18000000.
 */
#define QY7_RAM_SIZE (256 * MiB)
/*
 * TMU input clock. The CE kernel selects the /16 prescaler (TCR0 = 0x21)
 * and counts 2625 ticks per millisecond, so the clock it was built for is
 * 2625000 * 16 Hz. Taken from the guest, not measured on a board.
 */
#define QY7_TMU_CLOCK 42000000
#define QY7_SCIF_BASE 0xffe46000
/*
 * serial_scif.dll keeps a table of eight SCIF channels at 0xffe40000 +
 * 0x1000 * n. The guest maps channels 1, 3, 6 and 7 at boot (VirtualCopy
 * from 0xffe41000, 0xffe43000, 0xffe46000, 0xffe47000).
 */
#define QY7_SCIF0_BASE 0xffe40000
/* Index past any -serial the command line can reasonably carry. */
#define QY7_NO_CHARDEV 31
#define QY7_SCIF1_BASE 0xffe41000
#define QY7_SCIF2_BASE 0xffe42000
#define QY7_SCIF3_BASE 0xffe43000
#define QY7_SCIF5_BASE 0xffe45000
#define QY7_SCIF7_BASE 0xffe47000
/*
 * The ninth serial_scif.dll instance (registry key Drivers\BuiltIn\HSCIF,
 * Index 8, Prefix SCI) is opened by the device manager at boot.  Its
 * HWSetCommState (serial_scif.dll 0x03C88C40) polls SCFSR.TEND at
 * 0xffe48010; the context's register table covers offsets 0x00..0x24, 0x30
 * and 0x34 only.
 */
#define QY7_HSCIF_BASE 0xffe48000
#define QY7_USBPHY_BASE 0xffe70800
#define QY7_INT2_BASE 0xff804000
#define QY7_DU_BASE 0xfff80000
#define QY7_2DG_BASE 0xffe80000

typedef struct QY7MachineState {
    MachineState parent_obj;
    uint8_t dipsw;
    uint16_t status0_inputs;
    bool extint_model;
    bool paired_card;
    bool micom_on;
    bool bcm_on;
    bool micom_generic_reply;
    uint8_t micom_generic_reply_subcode;
    bool micom_audio_off;
    bool micom_antitheft_release;
    char *can_log_path;
    char *can_replay_path;
    DeviceState *scpu;
    QEMUTimer *mcan_refresh;
    QEMUTimer *mcan_key_timer;
    bool mcan_keyboard_started;
    uint8_t mcan_sequence;
    uint8_t mcan_key_code;
    unsigned mcan_key_stage;
    ClarionRcan *rcan;
    uint32_t du_dotclk;
    uint32_t du_panel_width;
    uint32_t du_panel_height;
} QY7MachineState;

typedef struct QY7DispSink {
    ClarionScif *scif;
} QY7DispSink;

static void qy7_mcan_refresh(void *opaque);
static void qy7_mcan_key_timer(void *opaque);

static void qy7_uart_tx(void *opaque, uint8_t byte)
{
    clarion_scpu_rx_byte(opaque, byte);
}

static int qy7_dispmicom_sink(void *opaque, const uint8_t *buf, int len)
{
    QY7DispSink *sink = opaque;
    int done = 0;

    while (done < len) {
        int room = clarion_scif_can_receive(sink->scif);
        int n;

        if (room <= 0) {
            break;
        }
        n = MIN(room, len - done);
        clarion_scif_receive(sink->scif, buf + done, n);
        done += n;
    }
    return done;
}

static int qy7_micom_sink(void *opaque, const uint8_t *buf, int len)
{
    ClarionScif *scif = opaque;
    int done = 0;

    while (done < len) {
        int room = clarion_scif_can_receive(scif);
        int n;

        if (room <= 0) {
            break;
        }
        n = MIN(room, len - done);
        clarion_scif_receive(scif, buf + done, n);
        done += n;
    }
    return done;
}

static void qy7_machine_instance_init(Object *obj)
{
    QY7MachineState *s = (QY7MachineState *)obj;

    s->dipsw = 7;
    /* Assumed asserted PowerMonitor input levels (STATUS0 bits 7 and 9). */
    s->status0_inputs = 0x0280;
    s->extint_model = true;
    s->paired_card = true;
    s->micom_on = true;
    s->bcm_on = true;
    s->micom_generic_reply = false;
    s->micom_generic_reply_subcode = 0;
    s->micom_audio_off = true;
    s->micom_antitheft_release = true;
    s->mcan_refresh = timer_new_ns(QEMU_CLOCK_VIRTUAL, qy7_mcan_refresh, s);
    s->mcan_key_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, qy7_mcan_key_timer, s);
    object_property_add_uint8_ptr(obj, "micom-generic-reply-subcode",
        &s->micom_generic_reply_subcode, OBJ_PROP_FLAG_READWRITE);
    /*
     * DU dot clock. 33333333 Hz is an assumed reference clock, not a
     * measured board value. Zero disables frame ticks.
     */
    s->du_dotclk = 33333333;
    s->du_panel_width = 800;
    s->du_panel_height = 480;
    object_property_add_uint32_ptr(obj, "du-panel-width",
                                   &s->du_panel_width,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_set_description(obj, "du-panel-width",
                                    "QY7 display panel width in pixels");
    object_property_add_uint32_ptr(obj, "du-panel-height",
                                   &s->du_panel_height,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_set_description(obj, "du-panel-height",
                                    "QY7 display panel height in pixels");
    object_property_add_uint32_ptr(obj, "du-dotclk", &s->du_dotclk,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_set_description(obj, "du-dotclk",
        "External DU pixel clock in Hz (0 disables vertical blank timing)");
}

static bool qy7_get_paired_card(Object *obj, Error **errp)
{
    return ((QY7MachineState *)obj)->paired_card;
}

static void qy7_set_paired_card(Object *obj, bool value, Error **errp)
{
    ((QY7MachineState *)obj)->paired_card = value;
}

static bool qy7_get_micom(Object *obj, Error **errp)
{
    return ((QY7MachineState *)obj)->micom_on;
}

static void qy7_set_micom(Object *obj, bool value, Error **errp)
{
    ((QY7MachineState *)obj)->micom_on = value;
}

static bool qy7_get_bcm(Object *obj, Error **errp)
{
    return ((QY7MachineState *)obj)->bcm_on;
}

static void qy7_set_bcm(Object *obj, bool value, Error **errp)
{
    ((QY7MachineState *)obj)->bcm_on = value;
}

static void qy7_inject_can(QY7MachineState *s, uint16_t id, const uint8_t *data,
                           unsigned len)
{
    if (!clarion_rcan_inject(s->rcan, id, data, len)) {
        clarion_scpu_can_rx(s->scpu, id, data, len);
    }
}

static void qy7_mcan_refresh(void *opaque)
{
    QY7MachineState *s = opaque;
    static const uint8_t frame[8] = { 0xf0, 0x00, 0x00, 0x01 };

    if (!s->mcan_keyboard_started) {
        return;
    }
    qy7_inject_can(s, 0x681, frame, sizeof(frame));
    timer_mod(s->mcan_refresh,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 140 * 1000 * 1000);
}

static uint8_t qy7_mcan_next_sequence(QY7MachineState *s)
{
    s->mcan_sequence = (s->mcan_sequence + 0x10) & 0x70;
    if (!s->mcan_sequence) {
        s->mcan_sequence = 0x10;
    }
    return s->mcan_sequence;
}

#define MCAN_CODE_AUDIO_ONOFF 0xf0
#define MCAN_FUNC_AUDIO_ONOFF 0x20

static void qy7_mcan_key_timer(void *opaque)
{
    QY7MachineState *s = opaque;
    static const uint8_t ack[8] = { 0xe0 };
    static const uint8_t state[8] = { 0x03, 0x40, 0x00, 0x2c };
    uint8_t frame[8] = { 0x04, 0, 0x40, 0x0d };

    if (s->mcan_key_stage == 3) {
        qy7_inject_can(s, 0x681, ack, sizeof(ack));
        s->mcan_key_stage = 4;
        timer_mod(s->mcan_key_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 20 * 1000 * 1000);
        return;
    }
    if (s->mcan_key_stage == 4) {
        qy7_inject_can(s, 0x681, state, sizeof(state));
        s->mcan_key_stage = 1;
        timer_mod(s->mcan_key_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1600 * 1000 * 1000);
        return;
    }
    if (s->mcan_key_stage == 1) {
        frame[1] = qy7_mcan_next_sequence(s);
        if (s->mcan_key_code == MCAN_CODE_AUDIO_ONOFF) {
            frame[2] = MCAN_FUNC_AUDIO_ONOFF;
            frame[4] = 0x00;
            qy7_inject_can(s, 0x681, frame, sizeof(frame));
            s->mcan_key_stage = 0;
            qemu_log_mask(LOG_GUEST_ERROR,
                          "clarion-qy7: MCAN audio on/off key code=%02x\n",
                          s->mcan_key_code);
            return;
        }
        frame[4] = s->mcan_key_code | 0x80;
        qy7_inject_can(s, 0x681, frame, sizeof(frame));
        s->mcan_key_stage = 2;
        timer_mod(s->mcan_key_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 150 * 1000 * 1000);
        return;
    }
    if (s->mcan_key_stage == 2) {
        frame[1] = qy7_mcan_next_sequence(s);
        frame[4] = s->mcan_key_code;
        qy7_inject_can(s, 0x681, frame, sizeof(frame));
        s->mcan_key_stage = 0;
        qemu_log_mask(LOG_GUEST_ERROR,
                      "clarion-qy7: MCAN panel key pair code=%02x\n",
                      s->mcan_key_code);
    }
}

static void qy7_mcan_key(void *opaque, uint8_t code, bool down)
{
    QY7MachineState *s = opaque;
    static const uint8_t start[8] = { 0xf0, 0x00, 0x00, 0x01 };
    int64_t delay_ms;

    if (!down) {
        return;
    }
    if (s->mcan_key_stage) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "clarion-qy7: MCAN panel key busy; ignored code=%02x\n",
                      code);
        return;
    }
    if (!s->mcan_keyboard_started) {
        s->mcan_keyboard_started = true;
        s->mcan_sequence = 0x40;
        qy7_inject_can(s, 0x681, start, sizeof(start));
        timer_mod(s->mcan_refresh,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 140 * 1000 * 1000);
        delay_ms = 40;
        s->mcan_key_stage = 3;
    } else {
        delay_ms = 200;
        s->mcan_key_stage = 1;
    }
    s->mcan_key_code = code;
    timer_mod(s->mcan_key_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + delay_ms * 1000 * 1000);
}

static bool qy7_parse_can_frame(const char *value, uint16_t *id,
                                uint8_t data[8], unsigned *len)
{
    const char *sep = strchr(value, '#');
    char *id_text;
    char *end;
    unsigned long parsed_id;
    const char *p;

    if (!sep || sep == value || sep - value > 8) {
        return false;
    }
    id_text = g_strndup(value, sep - value);
    errno = 0;
    parsed_id = g_ascii_strtoull(id_text, &end, 16);
    if (errno || end == id_text || *end || parsed_id > 0x7ff) {
        g_free(id_text);
        return false;
    }
    g_free(id_text);

    p = sep + 1;
    while (*p) {
        while (g_ascii_isspace(*p) || *p == ',') {
            p++;
        }
        if (!*p) {
            break;
        }
        if (!g_ascii_isxdigit(p[0]) || !g_ascii_isxdigit(p[1]) ||
            *len == 8) {
            return false;
        }
        data[(*len)++] = g_ascii_xdigit_value(p[0]) * 16 +
                         g_ascii_xdigit_value(p[1]);
        p += 2;
        if (*p && !g_ascii_isspace(*p) && *p != ',' &&
            !g_ascii_isxdigit(*p)) {
            return false;
        }
    }
    if (!*len) {
        return false;
    }
    *id = parsed_id;
    return true;
}

static void qy7_set_can_rx(Object *obj, const char *value, Error **errp)
{
    QY7MachineState *s = (QY7MachineState *)obj;
    uint16_t id;
    uint8_t data[8];
    unsigned len = 0;

    if (!s->scpu) {
        error_setg(errp, "can-rx is available after QY7 initialization");
        return;
    }
    if (!qy7_parse_can_frame(value, &id, data, &len)) {
        error_setg(errp, "can-rx expects ID#DATA in hexadecimal");
        return;
    }

    qemu_log_mask(LOG_GUEST_ERROR,
                  "clarion-qy7: can-rx inject id=%03x len=%u\n", id, len);
    if (clarion_rcan_inject(s->rcan, id, data, len)) {
        return;
    }
    clarion_scpu_can_rx(s->scpu, id, data, len);
}

static void qy7_set_can_log(Object *obj, const char *value, Error **errp)
{
    QY7MachineState *s = (QY7MachineState *)obj;

    g_free(s->can_log_path);
    s->can_log_path = g_strdup(value);
    if (s->rcan) {
        clarion_rcan_set_log(s->rcan, value);
    }
}

static void qy7_set_can_replay(Object *obj, const char *value, Error **errp)
{
    QY7MachineState *s = (QY7MachineState *)obj;

    g_free(s->can_replay_path);
    s->can_replay_path = g_strdup(value);
    if (s->rcan) {
        clarion_rcan_set_replay(s->rcan, value);
    }
}

static bool qy7_get_micom_generic_reply(Object *obj, Error **errp)
{
    return ((QY7MachineState *)obj)->micom_generic_reply;
}

static bool qy7_get_extint_model(Object *obj, Error **errp)
{
    return ((QY7MachineState *)obj)->extint_model;
}

static void qy7_set_extint_model(Object *obj, bool value, Error **errp)
{
    ((QY7MachineState *)obj)->extint_model = value;
}

static void qy7_set_micom_generic_reply(Object *obj, bool value, Error **errp)
{
    ((QY7MachineState *)obj)->micom_generic_reply = value;
}

static bool qy7_get_micom_audio_off(Object *obj, Error **errp)
{
    return ((QY7MachineState *)obj)->micom_audio_off;
}

static void qy7_set_micom_audio_off(Object *obj, bool value, Error **errp)
{
    ((QY7MachineState *)obj)->micom_audio_off = value;
}

static bool qy7_get_micom_antitheft_release(Object *obj, Error **errp)
{
    return ((QY7MachineState *)obj)->micom_antitheft_release;
}

static void qy7_set_micom_antitheft_release(Object *obj, bool value,
                                             Error **errp)
{
    ((QY7MachineState *)obj)->micom_antitheft_release = value;
}

typedef struct QY7ResetData {
    SuperHCPU *cpu;
} QY7ResetData;

static void qy7_reset(void *opaque)
{
    QY7ResetData *reset = opaque;

    cpu_reset(CPU(reset->cpu));
    reset->cpu->env.pc = 0xa0000000;
}

static void qy7_report_invalid_model(const char *model)
{
    error_report("clarion-qy7: flash model %s is not a QY7 unit; "
                 "use qemu-clarion", model);
}

/*
 * The guest reaches SCIF channels through user mappings (VirtualCopy),
 * whose TLB entries hold a 29-bit physical address: 0xffe41010 arrives as
 * 0x1fe41010 (area 7). Besides the P4 address, each channel therefore
 * needs an alias at A7ADDR(base), as the timer has.
 */
static ClarionScif *qy7_scif_with_alias_serial(MemoryRegion *sysmem,
                                               hwaddr base, const char *name,
                                               int index, int serial_index,
                                               qemu_irq irq, bool txi)
{
    ClarionScif *scif = g_new0(ClarionScif, 1);
    MemoryRegion *alias = g_new0(MemoryRegion, 1);
    g_autofree char *alias_name = g_strdup_printf("%s-a7", name);

    clarion_scif_init(scif, sysmem, base, name, index, serial_index, irq,
                      NULL, txi);
    memory_region_init_alias(alias, NULL, alias_name, &scif->mr, 0, 0x100);
    memory_region_add_subregion(sysmem, A7ADDR(base), alias);
    return scif;
}

static ClarionScif *qy7_scif_with_alias(MemoryRegion *sysmem, hwaddr base,
                                        const char *name, int index,
                                        qemu_irq irq)
{
    return qy7_scif_with_alias_serial(sysmem, base, name, index, index, irq,
                                      false);
}

static void qy7_init(MachineState *machine)
{
    QY7MachineState *qmachine = (QY7MachineState *)machine;
    MemoryRegion *sysmem = get_system_memory();
    MemoryRegion *ram = g_new0(MemoryRegion, 1);
    SuperHCPU *cpu = SUPERH_CPU(cpu_create(machine->cpu_type));
    QY7ResetData *reset;
    ClarionQy7Int2 *int2;
    DeviceState *dispmicom;
    DeviceState *micom;
    DeviceState *dmac;
    ClarionScif *scif1;
    ClarionScif *scif3;
    QY7DispSink *disp_sink;
    DeviceState *du;
    DeviceState *g2d;
    MemoryRegion *du_alias;
    MemoryRegion *g2d_alias;
    MemoryRegion *hpb_chan_alias;
    MemoryRegion *hpb_comm_alias;
    DriveInfo *dinfo;
    char model[CLARION_PROD_MODEL_LEN + 1] = "";
    const ClarionSeries *board;
    hwaddr prod_off = 0;
    bool found = false;

    dinfo = drive_get(IF_PFLASH, 0, 0);
    if (dinfo) {
        BlockBackend *blk = blk_by_legacy_dinfo(dinfo);
        for (int i = 0; i < ARRAY_SIZE(clarion_prod_offsets); i++) {
            uint8_t sig[4];
            uint8_t raw[CLARION_PROD_MODEL_LEN];
            hwaddr off = clarion_prod_offsets[i];

            if (blk_pread(blk, off, sizeof(sig), sig, 0) < 0 ||
                memcmp(sig, "PROD", sizeof(sig)) ||
                blk_pread(blk, off + CLARION_PROD_MODEL_OFF, sizeof(raw),
                         raw, 0) < 0) {
                continue;
            }
            for (int j = 0; j < sizeof(raw); j++) {
                model[j] = g_ascii_isgraph(raw[j]) ? raw[j] : '.';
            }
            model[sizeof(raw)] = '\0';
            found = true;
            prod_off = off;
            break;
        }
    }
    board = found ? clarion_series_find(model) : NULL;
    if (!board || strcmp(board->machine, "clarion-qy7")) {
        qy7_report_invalid_model(found ? model : "<no PROD>");
        exit(1);
    }

    if (!cpu) {
        error_report("Unable to create SH7785 CPU");
        exit(1);
    }
    if (qmachine->dipsw > 7) {
        error_report("dipsw must be in the range 0..7");
        exit(1);
    }
    int2 = clarion_qy7_int2_init(sysmem, QY7_INT2_BASE, CPU(cpu),
                                 &cpu->env.intc_handle);
    dmac = qdev_new(TYPE_CLARION_HPBDMA);
    qdev_prop_set_bit(dmac, "qy7-mode", true);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dmac), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dmac), 0, CLARION_HPBDMA_CHAN_BASE);
    sysbus_mmio_map(SYS_BUS_DEVICE(dmac), 1, CLARION_HPBDMA_COMM_BASE);
    hpb_chan_alias = g_new0(MemoryRegion, 1);
    memory_region_init_alias(hpb_chan_alias, NULL, "qy7.hpb-channel-a7",
                             sysbus_mmio_get_region(SYS_BUS_DEVICE(dmac), 0),
                             0, CLARION_HPBDMA_CHAN_SIZE);
    memory_region_add_subregion(sysmem, A7ADDR(CLARION_HPBDMA_CHAN_BASE),
                                hpb_chan_alias);
    hpb_comm_alias = g_new0(MemoryRegion, 1);
    memory_region_init_alias(hpb_comm_alias, NULL, "qy7.hpb-common-a7",
                             sysbus_mmio_get_region(SYS_BUS_DEVICE(dmac), 1),
                             0, CLARION_HPBDMA_COMM_SIZE);
    memory_region_add_subregion(sysmem, A7ADDR(CLARION_HPBDMA_COMM_BASE),
                                hpb_comm_alias);
    for (int i = 0; i < CLARION_HPBDMA_QY7_NUM_IRQ; i++) {
        sysbus_connect_irq(SYS_BUS_DEVICE(dmac), i,
                           clarion_qy7_int2_hpb_channel_irq(int2, i));
    }
    tmu012_init(sysmem, 0xffd80000,
                TMU012_FEAT_TOCR | TMU012_FEAT_3CHAN | TMU012_FEAT_EXTCLK,
                QY7_TMU_CLOCK, clarion_qy7_int2_timer_irq(int2, 0),
                clarion_qy7_int2_timer_irq(int2, 1),
                clarion_qy7_int2_timer_irq(int2, 2), NULL);
    reset = g_new0(QY7ResetData, 1);
    reset->cpu = cpu;
    qemu_register_reset(qy7_reset, reset);
    memory_region_init_ram(ram, NULL, "qy7.ram", QY7_RAM_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, QY7_RAM_BASE, ram);
    clarion_qy7_regs_init(sysmem, qmachine->dipsw,
                          qmachine->status0_inputs,
                          qmachine->extint_model,
                          clarion_qy7_int2_extirq(int2));

    qmachine->rcan = clarion_rcan_init(sysmem, 0x1ffd0000,
                                       clarion_qy7_int2_rcan_irq(int2));
    if (qmachine->can_log_path) {
        clarion_rcan_set_log(qmachine->rcan, qmachine->can_log_path);
    }
    if (qmachine->can_replay_path) {
        clarion_rcan_set_replay(qmachine->rcan, qmachine->can_replay_path);
    }

    pflash_cfi02_register(0, "qy7.flash", QY7_FLASH_SIZE,
                          dinfo ? blk_by_legacy_dinfo(dinfo) : NULL, 64 * KiB,
                          1, 2, 0x0001, 0x227e, 0x2220, 0x2200, 0x555, 0x2aa,
                          0);

    clarion_sh4_core_init(cpu, sysmem);
    du = qdev_new(TYPE_CLARION_DU);
    qdev_prop_set_uint32(du, "dotclk", qmachine->du_dotclk);
    qdev_prop_set_uint32(du, "panel-width", qmachine->du_panel_width);
    qdev_prop_set_uint32(du, "panel-height", qmachine->du_panel_height);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(du), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(du), 0, QY7_DU_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(du), 0,
                       clarion_qy7_int2_du_irq(int2));
    du_alias = g_new0(MemoryRegion, 1);
    memory_region_init_alias(du_alias, NULL, "qy7.du-a7",
                             sysbus_mmio_get_region(SYS_BUS_DEVICE(du), 0),
                             0, CLARION_DU_SIZE);
    memory_region_add_subregion(sysmem, A7ADDR(QY7_DU_BASE), du_alias);
    g2d = qdev_new(TYPE_CLARION_2DG);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(g2d), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(g2d), 0, QY7_2DG_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(g2d), 0,
                       clarion_qy7_int2_g2d_irq(int2));
    g2d_alias = g_new0(MemoryRegion, 1);
    memory_region_init_alias(g2d_alias, NULL, "qy7.2dg-a7",
                             sysbus_mmio_get_region(SYS_BUS_DEVICE(g2d), 0),
                             0, 0x200);
    memory_region_add_subregion(sysmem, A7ADDR(QY7_2DG_BASE), g2d_alias);
    /* SCIF6 carries the IPL console on the first -serial chardev. */
    qy7_scif_with_alias_serial(sysmem, QY7_SCIF_BASE, "qy7.scif", 6, 0,
                               clarion_qy7_int2_scif_irq(int2, 6), true);
    /*
     * SCIF0: CelaviDrv.dll opens "SCI0:" and serial_scif.dll then polls
     * SCFSR.TEND at 0xffe40010.  The first -serial belongs to the block at
     * 0xffe46000 above, so this one gets no host character device.
     */
    qy7_scif_with_alias_serial(sysmem, QY7_SCIF0_BASE, "qy7.scif0", 0,
                               QY7_NO_CHARDEV, NULL, false);
    qy7_scif_with_alias(sysmem, QY7_SCIF2_BASE, "qy7.scif2", 2,
                        clarion_qy7_int2_scif_irq(int2, 2));
    qy7_scif_with_alias(sysmem, QY7_SCIF5_BASE, "qy7.scif5", 5,
                        clarion_qy7_int2_scif_irq(int2, 5));
    /*
     * SCIF1: the routine at serial_scif.dll+0x8c18 polls SCFSR.TEND
     * (bit 6, offset 0x10) of this channel.
     */
    scif1 = qy7_scif_with_alias(sysmem, QY7_SCIF1_BASE, "qy7.scif1", 1,
                                clarion_qy7_int2_scif_irq(int2, 1));
    scif1->dmac = dmac;
    scif1->dma_feed = clarion_hpbdma_feed;
    scif1->dma_eod = clarion_hpbdma_eod;
    if (qmachine->micom_on) {
        micom = qdev_new(TYPE_CLARION_MICOM);
        qdev_prop_set_bit(micom, "generic-reply",
                          qmachine->micom_generic_reply);
        qdev_prop_set_uint8(micom, "generic-reply-subcode",
                            qmachine->micom_generic_reply_subcode);
        qdev_prop_set_bit(micom, "cenet-audio-off",
                          qmachine->micom_audio_off);
        qdev_prop_set_bit(micom, "cenet-antitheft-release",
                          qmachine->micom_antitheft_release);
        qdev_realize_and_unref(micom, NULL, &error_fatal);
        scif1->micom = micom;
        scif1->micom_tx = clarion_micom_rx_byte;
        clarion_micom_set_sink(micom, qy7_micom_sink, scif1);
    }
    /* SCIF3: the same polling routine, SCFSR at VA 0x2b0010. */
    scif3 = qy7_scif_with_alias(sysmem, QY7_SCIF3_BASE, "qy7.scif3", 3,
                                clarion_qy7_int2_scif_irq(int2, 3));
    scif3->dmac = dmac;
    scif3->dma_feed = clarion_hpbdma_feed;
    scif3->dma_eod = clarion_hpbdma_eod;
    dispmicom = qdev_new(TYPE_CLARION_DISPMICOM);
    qdev_prop_set_bit(dispmicom, "touch-input", true);
    qdev_prop_set_bit(dispmicom, "key-input", true);
    qdev_prop_set_bit(dispmicom, "can-key-input", true);
    object_property_add_child(qdev_get_machine(), "qy7-dispmicom",
                              OBJECT(dispmicom));
    qdev_realize_and_unref(dispmicom, NULL, &error_fatal);
    disp_sink = g_new0(QY7DispSink, 1);
    disp_sink->scif = scif3;
    scif3->dispmicom = dispmicom;
    scif3->dispmicom_tx = clarion_dispmicom_rx_byte;
    clarion_dispmicom_set_sink(dispmicom, qy7_dispmicom_sink, disp_sink);
    clarion_dispmicom_set_can_key_sink(dispmicom, qy7_mcan_key, qmachine);
    /* SCIF7: the same polling routine, SCFSR at VA 0x2f0010. */
    qy7_scif_with_alias(sysmem, QY7_SCIF7_BASE, "qy7.scif7", 7,
                        clarion_qy7_int2_scif_irq(int2, 7));
    /* HSCIF: the same polling routine, SCFSR at VA 0x00e40010 (area 7). */
    qy7_scif_with_alias(sysmem, QY7_HSCIF_BASE, "qy7.hscif", 8, NULL);
    clarion_usbphy_init(sysmem, QY7_USBPHY_BASE, 2, "qy7.usbphy");

    create_unimplemented_device("qy7.dbsc", 0xfe800000, 0x10000);
    create_unimplemented_device("qy7.lbsc", 0xff800200, 0x200);
    /* 0xff804000..0xff8040ff is the INT2 controller (qy7.int2) */
    create_unimplemented_device("qy7.lbsc-window", 0xff801000, 0x3000);
    create_unimplemented_device("qy7.lbsc-window2", 0xff804100, 0xf00);
    create_unimplemented_device("qy7.gpio", 0xffc40000, 0x4000);
    create_unimplemented_device("qy7.cpg", 0xffc80000, 0x1000);
    create_unimplemented_device("qy7.wdt", 0xffcc0000, 0x1000);
    {
        DeviceState *sdhi = qdev_new(TYPE_RENESAS_SDHI);
        DriveInfo *di = drive_get(IF_SD, 0, 0);
        MemoryRegion *alias = g_new0(MemoryRegion, 1);

        object_property_set_link(OBJECT(sdhi), "dmac", OBJECT(dmac),
                                 &error_fatal);
        qdev_prop_set_uint64(sdhi, "dma-buf-addr", 0xffe4c030);
        /* The firmware SD host driver uses a 42000 kHz input clock. */
        qdev_prop_set_uint32(sdhi, "input-clock-hz", 42000000);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(sdhi), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(sdhi), 0, 0xffe4c000);
        sysbus_connect_irq(SYS_BUS_DEVICE(sdhi), 0,
                           clarion_qy7_int2_sdhi_irq(int2, 0));
        memory_region_init_alias(alias, NULL, "qy7.sdhi0-a7",
                                 sysbus_mmio_get_region(
                                     SYS_BUS_DEVICE(sdhi), 0),
                                 0, RENESAS_SDHI_SIZE);
        memory_region_add_subregion(sysmem, A7ADDR(0xffe4c000), alias);
        if (di) {
            DeviceState *card = qdev_new(TYPE_SD_CARD);

            qdev_prop_set_drive_err(card, "drive",
                                   blk_by_legacy_dinfo(di), &error_fatal);
            qdev_realize_and_unref(card, qdev_get_child_bus(sdhi, "sd-bus"),
                                   &error_fatal);
            if (qmachine->paired_card && dinfo && found) {
                /*
                 * The card is paired with this unit: it comes up locked
                 * with the password kept in PROD.  The guest unlocks it with
                 * CMD42 (SDHC.dll 0x03CA6BDC, password from KernelIoControl
                 * 0x01013008).  The password stays in this buffer only.
                 */
                uint8_t pwd[CLARION_PROD_CARD_PWD_LEN];
                bool blank = true;

                if (blk_pread(blk_by_legacy_dinfo(dinfo),
                              prod_off + CLARION_PROD_CARD_PWD_OFF,
                              sizeof(pwd), pwd, 0) >= 0) {
                    for (int i = 0; i < sizeof(pwd); i++) {
                        blank &= pwd[i] == 0x00 || pwd[i] == 0xff;
                    }
                    if (!blank) {
                        sd_card_set_pairing(SD_CARD(card), pwd, sizeof(pwd));
                    }
                }
                memset(pwd, 0, sizeof(pwd));
            }
        }
    }
    create_unimplemented_device("qy7.sdhi1", 0xffe4d000, 0x1000);
    {
        /* Sub-CPU link UART on the external bus (area 6), SYSINTR 26. */
        DeviceState *uart = qdev_new(TYPE_CLARION_QY7_UART);
        DeviceState *scpu;

        sysbus_realize_and_unref(SYS_BUS_DEVICE(uart), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(uart), 0, CLARION_QY7_UART_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(uart), 0,
                           clarion_qy7_int2_uart_irq(int2));
        scpu = qdev_new(TYPE_CLARION_SCPU);
        qmachine->scpu = scpu;
        clarion_scpu_attach_uart(scpu, uart);
        qdev_realize_and_unref(scpu, NULL, &error_fatal);
        clarion_qy7_uart_set_tx(uart, qy7_uart_tx, scpu);
        if (qmachine->bcm_on) {
            DeviceState *bcm = qdev_new(TYPE_CLARION_BCM);

            qdev_realize_and_unref(bcm, NULL, &error_fatal);
            clarion_bcm_attach(bcm, scpu);
        }
    }
    create_unimplemented_device("qy7.pfc", 0xfffc0000, 0x10000);
    create_unimplemented_device("qy7.intc", 0xffd00000, 0x1000);
    create_unimplemented_device("qy7.intc-usermask", 0xffd30000, 0x1000);
    create_unimplemented_device("qy7.intc2", 0xffd40000, 0x1000);
}

static void qy7_machine_init(MachineClass *mc)
{
    mc->desc = "Clarion QY7 (SH7785)";
    mc->init = qy7_init;
    mc->default_cpu_type = TYPE_SH7785_CPU;
    mc->ignore_memory_transaction_failures = true;
}

static void qy7_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    qy7_machine_init(mc);
    object_class_property_add_uint8_ptr(
        oc, "dipsw", offsetof(QY7MachineState, dipsw),
        OBJ_PROP_FLAG_READWRITE);
    object_class_property_add_uint16_ptr(
        oc, "status0-inputs", offsetof(QY7MachineState, status0_inputs),
        OBJ_PROP_FLAG_READWRITE);
    object_class_property_add_bool(oc, "extint-model",
                                   qy7_get_extint_model,
                                   qy7_set_extint_model);
    object_class_property_set_description(oc, "extint-model",
        "Enable the QY7 external interrupt controller model (default on)");
    object_class_property_set_description(
        oc, "status0-inputs",
        "Assumed input level bitmap for board STATUS0 (default 0x0280)");
    object_class_property_add_bool(oc, "paired-card", qy7_get_paired_card,
                                   qy7_set_paired_card);
    object_class_property_set_description(oc, "paired-card",
        "The SD card is locked with the PROD card password at power-up, "
        "like a card paired with this unit (default on)");
    object_class_property_add_bool(oc, "micom", qy7_get_micom,
                                   qy7_set_micom);
    object_class_property_set_description(oc, "micom",
        "The companion micom connected to SCIF1 (default on)");
    object_class_property_add_bool(oc, "bcm", qy7_get_bcm, qy7_set_bcm);
    object_class_property_set_description(oc, "bcm",
        "Synthetic body control module answering the antitheft CAN "
        "exchange (default on)");
    object_class_property_add_str(oc, "can-rx", NULL, qy7_set_can_rx);
    object_class_property_set_description(oc, "can-rx",
        "Inject one CAN frame on QY7 as ID#DATA in hexadecimal");
    object_class_property_add_str(oc, "can-log", NULL, qy7_set_can_log);
    object_class_property_set_description(oc, "can-log",
        "Append QY7 CAN frames and mode changes to this file");
    object_class_property_add_str(oc, "can-replay", NULL,
                                  qy7_set_can_replay);
    object_class_property_set_description(oc, "can-replay",
        "Replay period_ms ID#DATA lines from this file");
    object_class_property_add_bool(oc, "micom-generic-reply",
                                   qy7_get_micom_generic_reply,
                                   qy7_set_micom_generic_reply);
    object_class_property_set_description(oc, "micom-generic-reply",
        "Reply to commands not modeled by the micom (default off)");
    object_class_property_add_bool(oc, "micom-audio-off",
                                   qy7_get_micom_audio_off,
                                   qy7_set_micom_audio_off);
    object_class_property_set_description(oc, "micom-audio-off",
        "The micom sends the CeNet 'audio mode off' notification after the "
        "link handshake, so the CEN driver reports a defined audio mode "
        "(synthetic, default on)");
    object_class_property_add_bool(oc, "micom-antitheft-release",
                                   qy7_get_micom_antitheft_release,
                                   qy7_set_micom_antitheft_release);
    object_class_property_set_description(oc, "micom-antitheft-release",
        "After the second link handshake the micom sends two synthetic CeNet "
        "frames (antitheft LOCK, then Mode RELEASE_OK) that make anwdrv "
        "signal the Antitheft events (default on)");
}

static const TypeInfo qy7_machine_typeinfo = {
    .name = MACHINE_TYPE_NAME("clarion-qy7"),
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(QY7MachineState),
    .instance_init = qy7_machine_instance_init,
    .class_init = qy7_machine_class_init,
};

static void qy7_machine_register_types(void)
{
    type_register_static(&qy7_machine_typeinfo);
}

type_init(qy7_machine_register_types)
