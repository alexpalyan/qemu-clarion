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
#include "hw/misc/clarion_boards.h"
#include "hw/sh4/clarion_sh4_core.h"
#include "hw/sh4/sh_intc.h"
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
#define QY7_RAM_SIZE (96 * MiB)
#define QY7_SCIF_BASE 0xffe46000

typedef struct QY7MachineState {
    MachineState parent_obj;
    uint8_t dipsw;
} QY7MachineState;

static void qy7_machine_instance_init(Object *obj)
{
    QY7MachineState *s = (QY7MachineState *)obj;

    s->dipsw = 7;
}

static struct intc_desc qy7_intc;
static struct intc_source qy7_tmu0_source;

static void qy7_tmu0_irq(void *opaque, int n, int level)
{
    qy7_tmu0_source.pending = level;
    qy7_intc.pending = level;
    if (level) {
        cpu_interrupt(first_cpu, CPU_INTERRUPT_HARD);
    } else {
        cpu_reset_interrupt(first_cpu, CPU_INTERRUPT_HARD);
    }
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

static void qy7_init(MachineState *machine)
{
    QY7MachineState *qmachine = (QY7MachineState *)machine;
    MemoryRegion *sysmem = get_system_memory();
    MemoryRegion *ram = g_new0(MemoryRegion, 1);
    SuperHCPU *cpu = SUPERH_CPU(cpu_create(machine->cpu_type));
    QY7ResetData *reset;
    DriveInfo *dinfo;
    char model[CLARION_PROD_MODEL_LEN + 1] = "";
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
    if (!found || strcmp(model, "QY7221NL")) {
        error_report("clarion-qy7: flash model %s is not supported; "
                     "supported: QY7221NL; use qemu-clarion",
                     found ? model : "<no PROD>");
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
    qy7_intc.nr_sources = 1;
    qy7_intc.sources = &qy7_tmu0_source;
    qy7_tmu0_source.parent = &qy7_intc;
    qy7_tmu0_source.vect = 0x560;
    cpu->env.intc_handle = &qy7_intc;
    tmu012_init(sysmem, 0xffd80000,
                TMU012_FEAT_TOCR | TMU012_FEAT_3CHAN | TMU012_FEAT_EXTCLK,
                33333333, qemu_allocate_irq(qy7_tmu0_irq, NULL, 0), NULL, NULL,
                NULL);
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

    create_unimplemented_device("qy7.dbsc", 0xfe800000, 0x10000);
    create_unimplemented_device("qy7.lbsc", 0xff800200, 0x200);
    create_unimplemented_device("qy7.lbsc-window", 0xff801000, 0x4000);
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
