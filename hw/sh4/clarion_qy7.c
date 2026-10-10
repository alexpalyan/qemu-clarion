/* Clarion QY7 bootloader machine. */
#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "hw/core/boards.h"
#include "system/block-backend.h"
#include "hw/block/flash.h"
#include "hw/char/clarion_scif.h"
#include "hw/misc/unimp.h"
#include "hw/misc/clarion_qy7_regs.h"
#include "hw/misc/clarion_usbphy.h"
#include "hw/misc/clarion_boards.h"
#include "hw/sh4/clarion_sh4_core.h"
#include "hw/sh4/sh.h"
#include "hw/intc/clarion_qy7_int2.h"
#include "hw/timer/tmu012.h"
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
 * The kernel's memory table lists regions up to 0x90000000 (P1), i.e.
 * physical 0x08000000..0x10000000: main RAM, then the NK2 image, then a
 * 1 MiB block at 0x0e600000 and a 25 MiB block at 0x0e700000 that
 * KernelIoControl hands to the display drivers.
 */
#define QY7_RAM_SIZE (128 * MiB)
#define QY7_SCIF_BASE 0xffe46000
/*
 * serial_scif.dll keeps a table of eight SCIF channels at 0xffe40000 +
 * 0x1000 * n. The guest maps channels 1, 3, 6 and 7 at boot (VirtualCopy
 * from 0xffe41000, 0xffe43000, 0xffe46000, 0xffe47000).
 */
#define QY7_SCIF1_BASE 0xffe41000
#define QY7_SCIF3_BASE 0xffe43000
#define QY7_USBPHY_BASE 0xffe70800
#define QY7_INT2_BASE 0xff804000

typedef struct QY7MachineState {
    MachineState parent_obj;
    uint8_t dipsw;
} QY7MachineState;

static void qy7_machine_instance_init(Object *obj)
{
    QY7MachineState *s = (QY7MachineState *)obj;

    s->dipsw = 7;
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

static const ClarionBoardInfo *qy7_board_by_model(const char *model)
{
    for (int i = 0; i < ARRAY_SIZE(clarion_boards); i++) {
        if (!strcmp(model, clarion_boards[i].model)) {
            return &clarion_boards[i];
        }
    }
    return NULL;
}

static void qy7_report_invalid_model(const char *model)
{
    GString *supported = g_string_new(NULL);

    for (int i = 0; i < ARRAY_SIZE(clarion_boards); i++) {
        if (!strcmp(clarion_boards[i].machine, "clarion-qy7")) {
            g_string_append_printf(supported, "%s%s",
                                   supported->len ? ", " : "",
                                   clarion_boards[i].model);
        }
    }
    error_report("clarion-qy7: flash model %s is not supported; "
                 "supported: %s; use qemu-clarion",
                 model, supported->str);
    g_string_free(supported, true);
}

/*
 * The guest reaches SCIF channels through user mappings (VirtualCopy),
 * whose TLB entries hold a 29-bit physical address: 0xffe41010 arrives as
 * 0x1fe41010 (area 7). Besides the P4 address, each channel therefore
 * needs an alias at A7ADDR(base), as the timer has.
 */
static void qy7_scif_with_alias(MemoryRegion *sysmem, hwaddr base,
                                const char *name, int index)
{
    ClarionScif *scif = g_new0(ClarionScif, 1);
    MemoryRegion *alias = g_new0(MemoryRegion, 1);
    g_autofree char *alias_name = g_strdup_printf("%s-a7", name);

    clarion_scif_init(scif, sysmem, base, name, index, index, NULL, NULL,
                      false);
    memory_region_init_alias(alias, NULL, alias_name, &scif->mr, 0, 0x100);
    memory_region_add_subregion(sysmem, A7ADDR(base), alias);
}

static void qy7_init(MachineState *machine)
{
    QY7MachineState *qmachine = (QY7MachineState *)machine;
    MemoryRegion *sysmem = get_system_memory();
    MemoryRegion *ram = g_new0(MemoryRegion, 1);
    SuperHCPU *cpu = SUPERH_CPU(cpu_create(machine->cpu_type));
    QY7ResetData *reset;
    ClarionQy7Int2 *int2;
    DriveInfo *dinfo;
    char model[CLARION_PROD_MODEL_LEN + 1] = "";
    const ClarionBoardInfo *board;
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
            break;
        }
    }
    board = found ? qy7_board_by_model(model) : NULL;
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
    tmu012_init(sysmem, 0xffd80000,
                TMU012_FEAT_TOCR | TMU012_FEAT_3CHAN | TMU012_FEAT_EXTCLK,
                33333333, clarion_qy7_int2_timer_irq(int2, 0),
                clarion_qy7_int2_timer_irq(int2, 1),
                clarion_qy7_int2_timer_irq(int2, 2), NULL);
    reset = g_new0(QY7ResetData, 1);
    reset->cpu = cpu;
    qemu_register_reset(qy7_reset, reset);
    memory_region_init_ram(ram, NULL, "qy7.ram", QY7_RAM_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, QY7_RAM_BASE, ram);
    clarion_qy7_regs_init(sysmem, qmachine->dipsw);

    pflash_cfi02_register(0, "qy7.flash", QY7_FLASH_SIZE,
                          dinfo ? blk_by_legacy_dinfo(dinfo) : NULL, 64 * KiB,
                          1, 2, 0x0001, 0x227e, 0x2220, 0x2200, 0x555, 0x2aa,
                          0);

    clarion_sh4_core_init(cpu, sysmem);
    clarion_scif_init(g_new0(ClarionScif, 1), sysmem, QY7_SCIF_BASE, "qy7.scif",
                      0, 0, NULL, NULL, false);
    /*
     * SCIF1: the routine at serial_scif.dll+0x8c18 polls SCFSR.TEND
     * (bit 6, offset 0x10) of this channel.
     */
    qy7_scif_with_alias(sysmem, QY7_SCIF1_BASE, "qy7.scif1", 1);
    /* SCIF3: the same polling routine, SCFSR at VA 0x2b0010. */
    qy7_scif_with_alias(sysmem, QY7_SCIF3_BASE, "qy7.scif3", 3);
    clarion_usbphy_init(sysmem, QY7_USBPHY_BASE, 2, "qy7.usbphy");

    create_unimplemented_device("qy7.dbsc", 0xfe800000, 0x10000);
    create_unimplemented_device("qy7.lbsc", 0xff800200, 0x200);
    /* 0xff804000..0xff8040ff is the INT2 controller (qy7.int2) */
    create_unimplemented_device("qy7.lbsc-window", 0xff801000, 0x3000);
    create_unimplemented_device("qy7.lbsc-window2", 0xff804100, 0xf00);
    create_unimplemented_device("qy7.hpb", 0xffc08000, 0x1000);
    create_unimplemented_device("qy7.gpio", 0xffc40000, 0x4000);
    create_unimplemented_device("qy7.cpg", 0xffc80000, 0x1000);
    create_unimplemented_device("qy7.wdt", 0xffcc0000, 0x1000);
    create_unimplemented_device("qy7.sdhi0", 0xffe4c000, 0x1000);
    create_unimplemented_device("qy7.sdhi1", 0xffe4d000, 0x1000);
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
