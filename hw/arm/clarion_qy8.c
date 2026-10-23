/*
 * Clarion QY8XXX head unit (Nissan Leaf) board emulation.
 *
 * SoC: Renesas R8A7778 (R-Car M1A), single-core Cortex-A9.
 * OS: Windows CE, XIP directly from flash on CS0 (reset vector @PA 0).
 *
 * Memory map extracted from the bootloader's OEMAddressTable
 * (@0x11c2c), not guessed:
 *
 *   VA 80000000 -> PA 00000000  64 MB   CS0 flash (same image)
 *   VA 84000000 -> PA 04000000   8 MB   CS1
 *   VA 84800000 -> PA 18000000   8 MB
 *   VA 85000000 -> PA 18800000   8 MB
 *   VA 85800000 -> PA f0000000   8 MB   L2C (PL310 @f0100000)
 *   VA 88000000 -> PA 08000000 128 MB   DDR (kernel load address)
 *   VA 90000000 -> PA 10000000 128 MB   DDR (eboot execution address)
 *   VA 9c000000 -> PA fc000000  64 MB   peripherals
 *
 * Peripherals are standard Renesas blocks; addresses match r8a7778.dtsi
 * and constants in the eboot code.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <math.h>
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/units.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/core/boards.h"
#include "hw/misc/clarion_boards.h"
#include "hw/core/sysbus.h"
#include "hw/core/loader.h"
#include "hw/core/qdev-properties.h"
#include "hw/display/clarion_2dg.h"
#include "hw/arm/boot.h"
#include "hw/intc/arm_gic.h"
#include "hw/display/clarion_du.h"
#ifdef CONFIG_PLUGIN
#include "qemu/plugin.h"
#include "hw/display/clarion_qy8_render.h"
#endif
#include "monitor/qdev.h"
#include "hw/display/clarion_sgx.h"
#include "hw/misc/clarion_micom.h"
#include "hw/misc/clarion_usbphy.h"
#include "hw/misc/clarion_shcore.h"
#include "hw/misc/clarion_dispmicom.h"
#include "hw/misc/clarion_ublox.h"
#include "hw/dma/clarion_hpbdma.h"
#include "hw/dma/clarion_lbdma.h"
#include "hw/sd/sd.h"
#include "hw/net/renesas_can.h"
#include "hw/i2c/clarion_rcar_i2c.h"
#include "hw/sd/renesas_sdhi.h"
#include "hw/misc/unimp.h"
#include "hw/char/clarion_scif.h"
#include "hw/usb/hcd-ehci.h"
#include "hw/usb/hcd-ohci.h"
#include "hw/block/flash.h"
#include "system/block-backend.h"
#include "system/blockdev.h"
#include "system/system.h"
#include "system/address-spaces.h"
#include "chardev/char-fe.h"
#include "hw/core/ptimer.h"
#include "hw/core/irq.h"
#include "target/arm/cpu-qom.h"
#include "target/arm/cpu.h"
#include "accel/tcg/cpu-loop.h"
#include "exec/target_page.h"
#include "qom/object.h"

/* --- board physical map ------------------------------------------------ */

#define QY8_FLASH_BASE      0x00000000
#define QY8_FLASH_SIZE      (64 * MiB)

/*
 * The CS0 chip is parallel NOR, not NAND (the dump filename is historical).
 * The bootloader confirms this: `NCG2K2_FlashRom` (flash 0x1114c) reads
 * the ID with the AMD sequence `AA`@0xAAAA, `55`@0x5554, `90`@0xAAAA, then
 * words at 0xA0000000/0xA0000002. Erase (0x113d8) adds `80`,`AA`,`55` and
 * `30` at the sector address. The detector (0x1127c) prints `ManuID/DevID`
 * and maps them to g_wSelectFlashType:
 *
 *   ManuID 0x89 or 0x1F + DevID 0x227E -> 0x30  "MICRON_PC28F512"
 *   ManuID 0x20          + DevID 0x227E -> 0x227E
 *   ManuID 0x01                          -> 0     "NORMAL"
 *
 * Use the first variant: 0x89/0x227E identifies a Micron PC28F512M29EW,
 * 512 Mbit (exactly 64 MB), x16, with uniform 128 KB sectors and AMD commands.
 * The sector stride matches the monitor: `backupcr` walks blocks in steps of
 * 0x20000 for type 0x30 (0x11804..0x11828), and 0x10000 for other types.
 */
#define QY8_FLASH_SECTOR    (128 * KiB)
#define QY8_FLASH_MANUF_ID  0x0089      /* Micron/Intel */
#define QY8_FLASH_DEVICE_ID 0x227e      /* PC28F512M29EW */

#define QY8_CS1_BASE        0x04000000
#define QY8_CS1_SIZE        (8 * MiB)

#define QY8_DDR0_BASE       0x08000000      /* kernel image, VA 0x88000000 */
#define QY8_DDR0_SIZE       (128 * MiB)
#define QY8_DDR1_BASE       0x10000000      /* eboot, VA 0x90000000      */
#define QY8_DDR1_SIZE       (128 * MiB)

#define QY8_SRAM0_BASE      0x18000000
#define QY8_SRAM0_SIZE      (8 * MiB)
#define QY8_SRAM1_BASE      0x18800000
#define QY8_SRAM1_SIZE      (8 * MiB)

/*
 * Main DDR window.
 *
 * Reset stage @PA 0x17e0 writes 0x0f030a02 to DBCONF0 (DBSC3 + 0x24; name
 * taken from the eboot monitor table at flash 0x24088). OAL @0x88011ef0 reads
 * the same register and selects a branch from field [28:24]: 0x0d -> 256 MB,
 * 0x0e -> 512 MB, otherwise -> 1024 MB. The remaining fields (15 rows, 8
 * banks, 10 columns, 32-bit bus) yield exactly 1 GiB, consistent with 1024.
 *
 * The controller has one DBCONF0 and one DBRNK0, hence one contiguous region.
 * The three extension DRAM windows from the OAL's 1024 branch (0x44000000
 * +64 MB, 0x58000000 +128 MB, 0x60000000 +480 MB) cover [0x44000000,
 * 0x7e000000). The only naturally aligned 1 GiB window containing them
 * starts at 0x40000000. The bootloader self-test (DIPSW=4, CTP_DramBank)
 * independently tests 0x40000000 and 0x58000000 as DRAM banks.
 *
 * ⚠️ This is not yet a final board model. qy8.ddr0 (PA 0x08000000) and
 * qy8.ddr1 (PA 0x10000000) remain separate memory regions, although on
 * hardware they are likely aliases of this DDR: the 256 MB hole at
 * 0x48000000..0x58000000 is excluded from both branches and exactly matches
 * the 256 MB mapped low by the kernel OEMAddressTable. This link is inferred,
 * not proven, so for now the guest sees more memory than the board has.
 */
#define QY8_DDR_BASE        0x40000000
#define QY8_DDR_SIZE        (1 * GiB)
/* A/B control: 0 disables creation of this window. */
#define QY8_DDR_ENABLED     1

/* eboot executes here (VA 0x97C00000 -> PA 0x17C00000) */
#define QY8_EBOOT_PA        0x17C00000

/* GIC as in r8a7778.dtsi; both bases appear in eboot code */
#define QY8_GIC_CPU_BASE    0xFE430000
#define QY8_GIC_DIST_BASE   0xFE438000
#define QY8_NUM_IRQ         256

#define QY8_L2C_BASE        0xF0100000

/*
 * Display Unit — display@fff80000 reg = <0xfff80000 0x40000> (r8a7779.dtsi).
 * Model: hw/display/clarion_du.c. The BOOTLOADER programs the display: it
 * enables 800x480 on channel 0 and leaves DOOR = 0. It does not write any
 * plane registers, so the display remains black until M3b; this is the system
 * state, not a model defect.
 */
#define QY8_DU_BASE         0xFFF80000
/*
 * DU interrupt line is **GIC_SPI 31**, confirmed 2026-09-24: the
 * OEMInterruptHandler has a dedicated case for GIC ID 63, and live guest OAL
 * tables map g_oalIrq2SysIntr[63] = 37, the SYSINTR used by ddi_ncg.dll. The
 * OAL appeared not to react to SPI 31 because the handler reached the INTC2
 * demux at 0xFE782048 and stalled there; that register was not modeled.
 *
 * Keep the number as the machine property `du-spi` for experiments.
 */
#define QY8_DU_SPI          31          /* confirmed by OAL tables */
#define QY8_I2C4_SPI        77          /* GIC ID 109 = SPI 77 */
#define QY8_GPIO4_SPI       103         /* GIC ID 135 = SPI 103 */
#define QY8_SHCORE_SPI      54          /* GIC INTID 86 = SPI 54 */
#define QY8_SHCORE_BASE     0xFE700000
#define QY8_SHCORE_SHARED_BASE 0xFE790000

#define QY8_SCIF_BASE       0xFFE40000      /* scif0..scif5, stride 0x1000 */
#define QY8_SCIF_STRIDE     0x1000
#define QY8_NUM_SCIF        6

/*
 * The bootloader console is SCIF3 (0xFFE43000), not SCIF0. This is confirmed
 * by OEMInitDebugSerial @VA 0x97c14758, which builds the base with two
 * instructions (mov r3,#0x3000; sub r3,r3,#0x1c0000 -> 0xFFE43000) and stores
 * it in global 0x97cc249c. putchar @0x97c14784 reads base+0x10 (SCFSR.TDFE)
 * and base+0x0c (SCFTDR). Therefore connect the first -serial (stdio with
 * -nographic) to SCIF3 and subsequent ports in order.
 */
#define QY8_SCIF_DEBUG      3

/* SCIF4 is the board companion MCU link, not the console. */
#define QY8_SCIF_MICOM      4

/*
 * SCIF1 is the Display Micom link, not the console. Guest registry entry
 * Drivers\Launch\SCIF1 -> serial_scif.dll, Prefix 'SCI', Index 1 gives stream
 * name `SCI1:`, opened by CLcdDrv::SCIF_Init @0xEF71724C.
 */
#define QY8_SCIF_DISPMICOM  1

/* serial@ffe4n000 interrupts = <GIC_SPI 70+n> (r8a7778.dtsi) */
#define QY8_SCIF_SPI0       70

static int qy8_scif_chr_index(int scif)
{
    return scif == QY8_SCIF_DEBUG ? 0 :
           scif < QY8_SCIF_DEBUG ? scif + 1 : scif;
}

/*
 * SDHI0/SDHI1 are the two SD slots (mmc@ffe4c000 / mmc@ffe4d000 in
 * r8a7778.dtsi). The device registry uses Drivers\SD\Card (PortNumber 0,
 * profile SDProfile, "Main Slot SD Card") and Drivers\SD\Card2
 * (PortNumber 1, SDProfile2, "Sub Slot SD Card"). Both use SDHC.dll.
 *
 * Interrupts are required: the driver starts its polling path (WaitEvent
 * @0xefa318d8, mode 1) with a zero timeout, so it fails immediately and waits
 * for command completion only through the ISR.
 */
#define QY8_CAN_BASE        0xFFFD1000
#define QY8_SDHI_BASE       0xFFE4C000
#define QY8_SDHI_STRIDE     0x1000
#define QY8_NUM_SDHI        2
/* SD_BUF0 is the data port read by DMAC (hpb_chan_module[21/22]). */
#define QY8_SDHI_BUF0       0x30

/*
 * GIC lines. OALIntrRequestIrqs (nk.exe @0x88012b34) assigns a logical IRQ
 * by device physical base: 0xFFE4C000 -> 119 (@0x88012fc0), 0xFFE4D000 ->
 * 120 (@0x88012fe8). Here the logical IRQ equals the GIC ID, and SPI n = ID
 * n + 32, so SDHI0 = SPI 87 and SDHI1 = SPI 88, matching r8a7778.dtsi for
 * mmc@ffe4c000 / mmc@ffe4d000. The OAL then maps IRQ 119 -> SYSINTR 20 and
 * IRQ 120 -> SYSINTR 23 (g_oalIrq2SysIntr table).
 */
#define QY8_SDHI_SPI0       87

/* --- board controller on CS bus @0x18800000 --------------------------- */

#define QY8_BCTL_BASE       0x18800000
#define QY8_BCTL_SIZE       0x40

#define BCTL_STATUS         0x00
#define BCTL_DIPSW          0x02

/* Status bits polled by the bootloader in loop @0x1148 (see below). */
#define BCTL_ST_PWR         0x0080
#define BCTL_ST_BOOT        0x0400

/*
 * Vehicle inputs in the same word, from GPIO.dll's register reader (QY8202NA
 * @0xEF6F31E0, QY8652NB @0xEF6C3204):
 * IOCTL_GPIO_ACCESS_* -> ldrh [base] >> bit.
 * GPIO.dll polls them and signals EVT_GPIO_* on a change. All active low.
 * ST_PWR is IGN (bit 7) and ST_BOOT is NAVI_ON (bit 10) in the same table.
 */
#define BCTL_ST_PKB         0x0020      /* bit 5: 0 = parking brake on */
#define BCTL_ST_ILL         0x0200      /* bit 9: ILL_MR, 0 = lights on */
#define BCTL_ST_RV          0x1000      /* bit 12: 0 = in reverse */

/*
 * Card-detect for both SD slots shares the 16-bit word with DIPSW.
 * Derived from `GPIO.dll` bit accessors (read vtbl+0x1C0 @VA 0xEF6C3204,
 * write vtbl+0x1C4 @0xEF6C3850; base = mapped PA 0x18800000):
 *
 *   IOCTL 0x800A2174 -> ldrh [base+0x02] >> 5 & 1   slot 0 card-detect
 *   IOCTL 0x800A20EC -> ldrh [base+0x02] >> 6 & 1   slot 1 card-detect
 *
 * `SDHC.dll` accesses these through `SD_Sense_Card` (@0xEFA2D888), which reads
 * the bit via `GPI1:`, and `CSDHCAccess::SDHC_CardCheck` (@0xEFA29ED4), which
 * reports "card present" only when the bit is ZERO (@0xEFA2D904:
 * `*out = (bit == 1) ? 2 : 1`; @0xEFA29FDC: result 1 only when `*out == 1`).
 * The line is active-low: an empty slot with pull-up reads as 1. The firmware
 * also treats an IOCTL failure as "no card", its safe default.
 *
 * ⚠ Word +0x02 is input-only: the write accessor has no branch for it (all
 * 19 output latches are at +0x04 and +0x00). Bits can therefore be overlaid
 * on reads without overwriting guest-written state.
 */
#define BCTL_CD0            0x0020      /* bit 5: slot 0, 0 = card present */
#define BCTL_CD1            0x0040      /* bit 6: slot 1, 0 = card present */

/* DIPSW modes from the eboot transition table @VA 0x97c07e28 */
#define QY8_DIPSW_NORM_RES  5           /* "NORM(RES)>>", as in TEST_B1 */

/* --- HSCIF0 (high-speed SCIF) @0xFFE48000 ------------------------------ */

/*
 * Separate block, not a seventh SCIF. It has its own node in r8a7778.dtsi:
 *
 *     hscif0: serial@ffe48000 {
 *             compatible = "renesas,hscif-r8a7778",
 *                          "renesas,rcar-gen1-hscif", "renesas,hscif";
 *             reg = <0xffe48000 96>;
 *             interrupts = <GIC_SPI 118 IRQ_TYPE_LEVEL_HIGH>;
 *     };
 *
 * Its base is outside the SCIF0..SCIF5 window (0xFFE40000..0xFFE45FFF), it
 * has size 0x60 and its own interrupt. Previously these 96 bytes fell into
 * the broad qy8.periph region (prio -1000), which returns zero on reads.
 *
 * Why this model is needed: when the extension DDR window is enabled, the
 * kernel starts a SECOND serial_scif.dll instance at PA 0xFFE48000.
 * HWSetCommState @VA 0xefa48158 begins by waiting for transmission to finish:
 *
 *     0xefa48190  ldr  r0, [r5, #0x10]   ; VA SCFSR
 *     0xefa48194  bl   READ_REGISTER_USHORT
 *     0xefa48198  uxth r3, r0
 *     0xefa4819c  tst  r3, #0x40         ; TEND
 *     0xefa481a0  bne  0xefa481a8        ; exit, then SetBaudRate, etc.
 *     0xefa481a4  b    0xefa48190        ; otherwise loop, without Sleep
 *
 * This is a spin loop that never yields, and the thread has priority 50.
 * Returning zero from qy8.periph leaves TEND = 0 forever: 11,754,863 reads
 * of 0xFFE48010 in 30 seconds, starving the device manager and preventing
 * ActivateDeviceEx("Drivers\BuiltIn") from returning.
 *
 * Register map: SCIx_HSCIF_REGTYPE from drivers/tty/serial/sh-sci.c:
 * SCSMR 0x00/16, SCBRR 0x04/8, SCSCR 0x08/16, SCxTDR 0x0c/8, SCxSR 0x10/16,
 * SCxRDR 0x14/8, SCFCR 0x18/16, SCFDR 0x1c/16, SCSPTR 0x20/16, SCLSR 0x24/16,
 * SCDL 0x30/16, SCCKS 0x34/16, HSSRR 0x40/16, HSRTRGR 0x54/16, HSTTRGR 0x58/16;
 * It also sets fifosize = 128 (16 for SCIF).
 *
 * ⚠ TEND and TDFE are NOT constant here; they reflect transmit FIFO
 * occupancy. After reset the FIFO is empty, so TEND = 1 immediately, as the
 * driver requires. When the guest writes to SCFTDR, TEND clears until the
 * model drains the bytes. Thus a TEND stall remains observable instead of
 * being hidden by a constant 0x40.
 *
 * Measured during a live boot (55 accesses for the entire boot, down from
 * 11,754,864):
 *
 *     r 0x20                 ; SCSPTR - one-time probe, as before
 *     r 0x10                 ; SCFSR: TEND = 1 -> loop exits on FIRST read
 *     w 0x08 = 0             ; SCSCR: disable TE/RE
 *     w 0x18 = 0x6           ; SCFCR: RFRST|TFRST
 *     r/w 0x10 = 0           ; clear flags
 *     r/w 0x24 = 0           ; SCLSR: clear ORER
 *     w 0x40 = 0x800f        ; HSSRR = SRE | SRCYC, HSCIF-only register
 *     w 0x34 = 0             ; SCCKS
 *     w 0x30 = 0x30          ; SCDL divider
 *     w 0x50 = 0x64          ; outside sh-sci.c map (see below)
 *     w 0x54 = 0x40          ; HSRTRGR = 64, the HSCIF sci_port rx_trigger
 *     w 0x58 = 0             ; HSTTRGR = 0, transmitter threshold
 *     w 0x08 = 0x2e, 0x3e    ; TE|RE|REIE|CKE1; neither RIE nor TIE
 *     w 0x0c ×6              ; 10 02 23 10 03 30, one frame on the link
 *
 * Three model details come from this measurement rather than guesswork:
 *  - HSSRR @0x40 with SRE confirms this is HSCIF, not a seventh SCIF;
 *  - HSTTRGR = 0, so TDFE means "transmit FIFO empty", as computed by
 *    qy8_hscif_tdfe();
 *  - the guest enables neither RIE nor TIE, so no transmitter interrupt is
 *    needed here.
 *
 * ⚠ Offset 0x50 (value 0x64 = 100) is absent from the sh-sci.c map; the guest
 * writes it once and never reads it. It is deliberately left UNMODELED and
 * reported via LOG_UNIMP as qy8.hscif0. There is no evidence for inventing
 * semantics. If needed later, this is the place to investigate.
 *
 * Deliberate omissions:
 *  - DMA channel: HPB-DMAC serves SCIF4 (the micom link); there is no evidence
 *    that this port enables DMA, and a slave ID must not be invented;
 *  - transmitter interrupt: as in the SCIF model, only the receiver raises
 *    the line, and only with RIE enabled;
 *  - nonzero reset values: all register storage starts at 0. Unmapped
 *    accesses go to LOG_UNIMP under their own name so the next blocker is
 *    identifiable instead of hidden in qy8.periph noise.
 */

#define QY8_HSCIF_BASE      0xFFE48000
#define QY8_HSCIF_SIZE      0x60        /* reg = <0xffe48000 96> */
#define QY8_HSCIF_SPI       118         /* interrupts = <GIC_SPI 118> */

#define HSCIF_SCSMR     0x00
#define HSCIF_SCBRR     0x04
#define HSCIF_SCSCR     0x08
#define HSCIF_SCFTDR    0x0C
#define HSCIF_SCFSR     0x10
#define HSCIF_SCFRDR    0x14
#define HSCIF_SCFCR     0x18
#define HSCIF_SCFDR     0x1C
#define HSCIF_SCSPTR    0x20
#define HSCIF_SCLSR     0x24
#define HSCIF_SCDL      0x30
#define HSCIF_SCCKS     0x34
#define HSCIF_HSSRR     0x40
#define HSCIF_HSRTRGR   0x54
#define HSCIF_HSTTRGR   0x58

/* SCFCR: reset TX/RX FIFOs (same map as SCIF) */
#define SCFCR_TFRST     0x0004
#define HSCIF_SCFCR_RFRST 0x0002
#define HSCIF_SCSCR_RIE   0x0040
#define HSCIF_SCFSR_DR   0x0001
#define HSCIF_SCFSR_RDF  0x0002
#define HSCIF_SCFSR_TDFE 0x0020
#define HSCIF_SCFSR_TEND 0x0040
#define QY8_SCIF_IDLE_NS 200000

#define QY8_HSCIF_FIFO  128             /* sh-sci.c: fifosize = 128 */

/*
 * Virtual transmission time per byte. Baud rate cannot be derived: the guest
 * programs the divider after this wait, and SCBRR is 0 for the same sequence
 * on SCIF. As with QY8_SCIF_IDLE_NS, use a fixed value. Its precision is less
 * important than ensuring the transmitter stays busy for nonzero time and
 * TEND follows the device state.
 */
#define QY8_HSCIF_TX_NS     10000

typedef struct Qy8Hscif {
    MemoryRegion mr;
    CharFrontend chr;
    qemu_irq irq;
    QEMUTimer *tx;              /* drain transmit FIFO */
    QEMUTimer *idle;            /* line idle -> DR, as in SCIF */
    uint16_t scsmr, scscr, scfcr, scsptr, scdl, sccks, hssrr;
    uint16_t hsrtrgr, hsttrgr, sclsr;
    uint8_t scbrr;
    uint8_t rx[QY8_HSCIF_FIFO];
    unsigned rx_len;
    unsigned tx_len;            /* bytes in transmit FIFO */
    bool dr;
} Qy8Hscif;

/* RTRG[7:6] in SCFCR; HSCIF can also get the threshold from HSRTRGR */
static unsigned qy8_hscif_rtrg(Qy8Hscif *h)
{
    static const unsigned lvl[4] = { 1, 4, 8, 14 };

    /* sh-sci.c uses HSRTRGR as the receive threshold when programmed */
    if (h->hsrtrgr) {
        return MIN(h->hsrtrgr, QY8_HSCIF_FIFO);
    }
    return lvl[(h->scfcr >> 6) & 3];
}

static bool qy8_hscif_rdf(Qy8Hscif *h)
{
    return h->rx_len >= qy8_hscif_rtrg(h);
}

/* TEND: transmitter empty, with nothing in the FIFO or "on the wire" */
static bool qy8_hscif_tend(Qy8Hscif *h)
{
    return h->tx_len == 0;
}

/*
 * TDFE means "transmit FIFO occupancy is at or below the threshold".
 * HSTTRGR supplies the threshold; after reset it is 0, so TDFE means the FIFO
 * is empty. This is stricter than hardware, where the threshold is
 * programmable, but requires no invented reset value and never permits a
 * write to a full FIFO.
 */
static bool qy8_hscif_tdfe(Qy8Hscif *h)
{
    return h->tx_len <= MIN(h->hsttrgr, QY8_HSCIF_FIFO - 1);
}

static void qy8_hscif_update_irq(Qy8Hscif *h)
{
    /* Receiver only, and only with RIE enabled; see the SCIF model note. */
    qemu_set_irq(h->irq, (qy8_hscif_rdf(h) || h->dr) && (h->scscr & HSCIF_SCSCR_RIE));
}

static void qy8_hscif_tx_expire(void *opaque)
{
    Qy8Hscif *h = opaque;

    if (h->tx_len) {
        h->tx_len--;
    }
    if (h->tx_len) {
        timer_mod(h->tx, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                         QY8_HSCIF_TX_NS);
    }
}

static void qy8_hscif_idle_expire(void *opaque)
{
    Qy8Hscif *h = opaque;

    if (h->rx_len) {
        h->dr = true;
        qy8_hscif_update_irq(h);
    }
}

static uint64_t qy8_hscif_read(void *opaque, hwaddr addr, unsigned size)
{
    Qy8Hscif *h = opaque;

    switch (addr) {
    case HSCIF_SCSMR:
        return h->scsmr;
    case HSCIF_SCBRR:
        return h->scbrr;
    case HSCIF_SCSCR:
        return h->scscr;
    case HSCIF_SCFSR:
        return (qy8_hscif_tdfe(h) ? HSCIF_SCFSR_TDFE : 0) |
               (qy8_hscif_tend(h) ? HSCIF_SCFSR_TEND : 0) |
               (qy8_hscif_rdf(h) ? HSCIF_SCFSR_RDF : 0) |
               (h->dr ? HSCIF_SCFSR_DR : 0);
    case HSCIF_SCFRDR: {
        uint8_t v = h->rx_len ? h->rx[0] : 0;

        if (h->rx_len) {
            memmove(h->rx, h->rx + 1, --h->rx_len);
        }
        if (!h->rx_len) {
            h->dr = false;
        }
        qy8_hscif_update_irq(h);
        return v;
    }
    case HSCIF_SCFCR:
        return h->scfcr;
    case HSCIF_SCFDR:
        /* High byte: TX FIFO occupancy; low byte: RX FIFO occupancy. */
        return ((h->tx_len & 0xff) << 8) | (h->rx_len & 0xff);
    case HSCIF_SCSPTR:
        return h->scsptr;
    case HSCIF_SCLSR:
        return h->sclsr;
    case HSCIF_SCDL:
        return h->scdl;
    case HSCIF_SCCKS:
        return h->sccks;
    case HSCIF_HSSRR:
        return h->hssrr;
    case HSCIF_HSRTRGR:
        return h->hsrtrgr;
    case HSCIF_HSTTRGR:
        return h->hsttrgr;
    default:
        qemu_log_mask(LOG_UNIMP, "qy8.hscif0: read outside map at %#" HWADDR_PRIx
                      " (%u bytes)\n", addr, size);
        return 0;
    }
}

static void qy8_hscif_write(void *opaque, hwaddr addr, uint64_t val,
                            unsigned size)
{
    Qy8Hscif *h = opaque;
    uint8_t ch;

    switch (addr) {
    case HSCIF_SCSMR:
        h->scsmr = val;
        break;
    case HSCIF_SCBRR:
        h->scbrr = val;
        break;
    case HSCIF_SCSCR:
        h->scscr = val;
        qy8_hscif_update_irq(h);
        break;
    case HSCIF_SCFTDR:
        ch = val & 0xff;
        if (h->tx_len < QY8_HSCIF_FIFO) {
            if (!h->tx_len) {
                timer_mod(h->tx, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                                 QY8_HSCIF_TX_NS);
            }
            h->tx_len++;
        }
        /*
         * Output is synchronous, as in the SCIF model; only the occupancy
         * counter remains in the FIFO and determines TEND/TDFE.
         */
        qemu_chr_fe_write_all(&h->chr, &ch, 1);
        break;
    case HSCIF_SCFCR:
        h->scfcr = val;
        if (val & HSCIF_SCFCR_RFRST) {
            h->rx_len = 0;
            h->dr = false;
        }
        if (val & SCFCR_TFRST) {
            h->tx_len = 0;
            timer_del(h->tx);
        }
        qy8_hscif_update_irq(h);
        break;
    case HSCIF_SCFSR:
        /* Clear flags by writing zero to the corresponding bit. */
        if (!(val & HSCIF_SCFSR_DR)) {
            h->dr = false;
        }
        qy8_hscif_update_irq(h);
        break;
    case HSCIF_SCLSR:
        h->sclsr &= val;        /* ORER is cleared by writing zero. */
        break;
    case HSCIF_SCSPTR:
        h->scsptr = val;
        break;
    case HSCIF_SCDL:
        h->scdl = val;
        break;
    case HSCIF_SCCKS:
        h->sccks = val;
        break;
    case HSCIF_HSSRR:
        h->hssrr = val;
        break;
    case HSCIF_HSRTRGR:
        h->hsrtrgr = val;
        qy8_hscif_update_irq(h);
        break;
    case HSCIF_HSTTRGR:
        h->hsttrgr = val;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "qy8.hscif0: write outside map at %#" HWADDR_PRIx
                      " = %#" PRIx64 " (%u bytes)\n", addr, val, size);
        break;
    }
}

static const MemoryRegionOps qy8_hscif_ops = {
    .read = qy8_hscif_read,
    .write = qy8_hscif_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static int qy8_hscif_can_receive(void *opaque)
{
    Qy8Hscif *h = opaque;
    return QY8_HSCIF_FIFO - h->rx_len;
}

static void qy8_hscif_receive(void *opaque, const uint8_t *buf, int size)
{
    Qy8Hscif *h = opaque;
    int i;

    for (i = 0; i < size && h->rx_len < QY8_HSCIF_FIFO; i++) {
        h->rx[h->rx_len++] = buf[i];
    }
    h->dr = false;
    qy8_hscif_update_irq(h);
    timer_mod(h->idle, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                       QY8_SCIF_IDLE_NS);
}

/*
 * Deliver bytes from either MCU (board on SCIF4 or panel on SCIF1).
 * Return the number accepted.
 *
 * A frame must reach the receiver as ONE burst. A naive "send what fits in
 * the 16-byte FIFO, then send the rest later" splits longer frames because
 * the SCIF idle timer fires between chunks (QY8_SCIF_IDLE_NS = 200 us, while
 * the MCU refills after *_FILL_NS = 1 ms). `qy8_scif_idle_expire()` does more
 * than set DR: it also closes the DMA set via `clarion_hpbdma_eod()`. The guest
 * would receive several separate 16-byte fragments instead of one frame.
 *
 * This does not happen on a live link: bytes arrive continuously while DMA
 * drains the FIFO. In the model draining is immediate (qy8_scif_receive ->
 * qy8_scif_rx_pump), so refill while space remains. Virtual time does not
 * advance inside the loop, the idle timer does not fire, and the frame stays
 * one burst.
 *
 * Observed symptoms: the panel MCU's 0x24 frame (20 bytes) split into two
 * incomplete frames and the guest replied with NAK; the board MCU's command
 * set frame `08 0a` (~70 bytes) never reached the EdaDrv parser, leaving the
 * link permanently in state 2 and ACM-ID 61 unset.
 */
static int qy8_micom_burst(ClarionScif *s, const uint8_t *buf, int len)
{
    int done = 0;

    while (done < len) {
        int room = clarion_scif_can_receive(s);
        int n;

        if (room <= 0) {
            break;                  /* DMA not armed; MCU will send the rest */
        }
        n = MIN(room, len - done);
        clarion_scif_receive(s, buf + done, n);
        done += n;
    }
    return done;
}

static int qy8_micom_sink(void *opaque, const uint8_t *buf, int len)
{
    return qy8_micom_burst(opaque, buf, len);
}

static int qy8_dispmicom_sink(void *opaque, const uint8_t *buf, int len)
{
    return qy8_micom_burst(opaque, buf, len);
}

static int qy8_ublox_sink(void *opaque, const uint8_t *buf, int len)
{
    return qy8_micom_burst(opaque, buf, len);
}

/* --- TMU (Renesas timers, register map as in SH TMU) ------------------- */

#define QY8_TMU_BASE        0xFFD80000
#define QY8_TMU_CHANS       3
#define QY8_TMU_SPI0        32          /* channel n -> SPI 32+n -> IRQ 64+n;
                                           verified: eboot enables IRQ 65
                                           when channel 1 starts */
#define QY8_PCLK            65000000    /* P-clock R8A7778 */

#define TMU_TOCR    0x00
#define TMU_TSTR    0x04
#define TMU_CH(n)   (0x08 + (n) * 0x0c)   /* TCOR, +4 TCNT, +8 TCR */

#define TCR_TPSC    0x0007
#define TCR_UNIE    0x0020
#define TCR_UNF     0x0100

typedef struct Qy8TmuChan {
    ptimer_state *ptimer;
    qemu_irq irq;
    uint32_t tcor;
    uint16_t tcr;
    bool running;
} Qy8TmuChan;

typedef struct Qy8Tmu {
    MemoryRegion mr;
    Qy8TmuChan ch[QY8_TMU_CHANS];
    uint8_t tstr;
    uint8_t tocr;
} Qy8Tmu;

static void qy8_tmu_chan_update_irq(Qy8TmuChan *c)
{
    qemu_set_irq(c->irq, (c->tcr & TCR_UNF) && (c->tcr & TCR_UNIE));
}

static void qy8_tmu_tick(void *opaque)
{
    Qy8TmuChan *c = opaque;

    c->tcr |= TCR_UNF;
    qy8_tmu_chan_update_irq(c);
}

static uint32_t qy8_tmu_freq(const Qy8TmuChan *c)
{
    /* TPSC: 0=P/4 1=P/16 2=P/64 3=P/256 4=P/1024 */
    static const uint32_t div[8] = { 4, 16, 64, 256, 1024, 1024, 1024, 1024 };
    return QY8_PCLK / div[c->tcr & TCR_TPSC];
}

static void qy8_tmu_chan_restart(Qy8TmuChan *c, bool run)
{
    ptimer_transaction_begin(c->ptimer);
    if (run) {
        ptimer_set_freq(c->ptimer, qy8_tmu_freq(c));
        ptimer_set_limit(c->ptimer, c->tcor ? c->tcor : 1, 0);
        ptimer_run(c->ptimer, 0);
    } else {
        ptimer_stop(c->ptimer);
    }
    ptimer_transaction_commit(c->ptimer);
    c->running = run;
}

static uint64_t qy8_tmu_read(void *opaque, hwaddr addr, unsigned size)
{
    Qy8Tmu *t = opaque;
    int n;

    if (addr == TMU_TOCR) {
        return t->tocr;
    }
    if (addr == TMU_TSTR) {
        return t->tstr;
    }
    for (n = 0; n < QY8_TMU_CHANS; n++) {
        hwaddr b = TMU_CH(n);
        if (addr == b) {
            return t->ch[n].tcor;
        }
        if (addr == b + 4) {
            return ptimer_get_count(t->ch[n].ptimer);
        }
        if (addr == b + 8) {
            return t->ch[n].tcr;
        }
    }
    return 0;
}

static void qy8_tmu_write(void *opaque, hwaddr addr, uint64_t val,
                          unsigned size)
{
    Qy8Tmu *t = opaque;
    int n;

    if (addr == TMU_TOCR) {
        t->tocr = val;
        return;
    }
    if (addr == TMU_TSTR) {
        t->tstr = val;
        for (n = 0; n < QY8_TMU_CHANS; n++) {
            bool run = (val >> n) & 1;
            if (run != t->ch[n].running) {
                qy8_tmu_chan_restart(&t->ch[n], run);
            }
        }
        return;
    }
    for (n = 0; n < QY8_TMU_CHANS; n++) {
        Qy8TmuChan *c = &t->ch[n];
        hwaddr b = TMU_CH(n);

        if (addr == b) {
            c->tcor = val;
            ptimer_transaction_begin(c->ptimer);
            ptimer_set_limit(c->ptimer, val ? val : 1, 0);
            ptimer_transaction_commit(c->ptimer);
            return;
        }
        if (addr == b + 4) {
            ptimer_transaction_begin(c->ptimer);
            ptimer_set_count(c->ptimer, val);
            ptimer_transaction_commit(c->ptimer);
            return;
        }
        if (addr == b + 8) {
            /* Clear UNF by writing zero to this bit. */
            c->tcr = (val & ~TCR_UNF) | (c->tcr & val & TCR_UNF);
            if (c->running) {
                ptimer_transaction_begin(c->ptimer);
                ptimer_set_freq(c->ptimer, qy8_tmu_freq(c));
                ptimer_transaction_commit(c->ptimer);
            }
            qy8_tmu_chan_update_irq(c);
            return;
        }
    }
}

static const MemoryRegionOps qy8_tmu_ops = {
    .read = qy8_tmu_read,
    .write = qy8_tmu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

/* --- board controller (external CS chip, 16-bit registers) ------------- */

/*
 * This is a separate chip on the CS bus, not an SoC block. The firmware uses
 * it for DIPSW and startup-enable signals. Reads below were derived from code:
 *
 *   +0x00  status. The bootloader reads it twice in loop @0x1148 (external
 *          bus debounce) and checks bit 0x400, "leave standby". Without it,
 *          counter r7 never reaches zero and watchdog reset occurs at
 *          @0x11e0. Bit 0x80 controls a second counter (r6); @0x1a6c then
 *          writes 0x8000 or 0 to +0x04.
 *   +0x02  DIPSW. The first stage @0x12c4 compares the low 3 bits with 4
 *          (flash update mode at offset 0x60000); eboot prints "DIPSW=%x"
 *          @0x97c0686c and selects the boot mode from these bits.
 *   +0x04, +0x06, +0x08, +0x0c, +0x14 are written during initialization.
 */
typedef struct Qy8Bctl {
    MemoryRegion mr;
    uint16_t reg[QY8_BCTL_SIZE / 2];
    SDBus *sd[QY8_NUM_SDHI];    /* SDHI buses provide card-detect state */
} Qy8Bctl;

/*
 * Card-detect is not stored in reg[]; derive it on every read from QEMU's
 * actual topology. `sdbus_get_inserted()` is true only when the bus has a card
 * device with a block backend (`get_card()` returns NULL for an empty bus;
 * `sd_get_inserted()` checks `blk && blk_is_inserted(blk)`). This represents
 * an inserted card, not an existing controller: both SDHI controllers always
 * exist in the model, and `-drive if=sd,index=N` inserts a card into a slot.
 */
static uint16_t qy8_bctl_card_detect(Qy8Bctl *b)
{
    static const uint16_t cd[QY8_NUM_SDHI] = { BCTL_CD0, BCTL_CD1 };
    uint16_t bits = 0;
    int i;

    for (i = 0; i < QY8_NUM_SDHI; i++) {
        if (!b->sd[i] || !sdbus_get_inserted(b->sd[i])) {
            bits |= cd[i];          /* empty -> 1 (active-low line) */
        }
    }
    return bits;
}

static uint64_t qy8_bctl_read(void *opaque, hwaddr addr, unsigned size)
{
    Qy8Bctl *b = opaque;
    uint16_t v = b->reg[addr >> 1];

    if (addr == BCTL_DIPSW) {
        v = (v & ~(BCTL_CD0 | BCTL_CD1)) | qy8_bctl_card_detect(b);
    }
    return v;
}

static void qy8_bctl_write(void *opaque, hwaddr addr, uint64_t val,
                           unsigned size)
{
    Qy8Bctl *b = opaque;

    /*
     * +0x00 is status on read and a different register on write: stage one
     * @0x1a6c writes 0x2000 there, then immediately reads the same address,
     * expecting bits 0x80/0x400 to remain set. If the write overwrote the
     * read value, the next watchdog poll @0x97c10e84 would see 0x400 clear,
     * set state 1, and reset on the following tick at @0x97c10d38 ("_p").
     * Read and write views therefore differ; writes do not alter status.
     */
    if (addr == BCTL_STATUS) {
        return;
    }
    b->reg[addr >> 1] = val;
}

static const MemoryRegionOps qy8_bctl_ops = {
    .read = qy8_bctl_read,
    .write = qy8_bctl_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 2,
    .impl.max_access_size = 2,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};



/* --- unmapped address space -------------------------------------------- */

/*
 * By default, QEMU responds to an unmapped physical address with an external
 * data/prefetch abort. This board does not: the WinCE kernel probes memory.
 *
 * Routine @0x880489ec takes (start, length, working VA), maps a candidate page
 * at 0xC04B0000 and binary-searches for the top of RAM: it saves two words,
 * writes signatures 0x6a08bc95/0xfd1247e3, reads them back, then moves the
 * lower bound up on a match and the upper bound down on a mismatch.
 *
 * The first call probes real RAM (PA 0x0d6f0000, size 0x9e0000, matching
 * ulRAMStart..ulRAMEnd in the kernel ROMHDR) and succeeds. The second probes
 * a possible 64 MB bank at PA 0x44000000, which is absent from every kernel
 * mapping (L1 table: flash @0, 256 MB DDR @0x08000000, windows at 0x18000000,
 * 0xf0000000, and 0xfc000000). Probing this unmapped space is expected; reads
 * must return arbitrary data for the algorithm to work. Here it caused an
 * abort, sending the kernel to vector @0xffff0010.
 *
 * Provide a low-priority "void" region: reads return zero and writes are
 * ignored. Its priority is below all other regions, including qy8.periph
 * (-1000), so it cannot intercept real registers; accesses remain visible
 * with -d unimp.
 */
static uint64_t qy8_void_read(void *opaque, hwaddr addr, unsigned size)
{
    qemu_log_mask(LOG_UNIMP, "qy8.void: read from unmapped address %#" HWADDR_PRIx
                  " (%u bytes)\n", addr, size);
    return 0;
}

static void qy8_void_write(void *opaque, hwaddr addr, uint64_t val,
                           unsigned size)
{
    qemu_log_mask(LOG_UNIMP, "qy8.void: write to unmapped address %#" HWADDR_PRIx
                  " = %#" PRIx64 "\n", addr, val);
}

static const MemoryRegionOps qy8_void_ops = {
    .read = qy8_void_read,
    .write = qy8_void_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
};

/* --- USB host: EHCI @0xFFE70000, OHCI @0xFFE70400 --------------------- */

/*
 * Bases are the ones MQUSBH prints and the OAL's base->IRQ table uses
 * (nk.exe QY8202NA @0x8801275c, QY8652NB @0x88013028): EHCI -> IRQ 165,
 * OHCI -> 164.
 * Both share GIC ID 76 (SPI 44); the OAL demux reads INTC2 word 0xFE782058,
 * bit 1 for EHCI and bit 0 for OHCI (QY8202NA @0x88011734,
 * QY8652NB @0x88011e14).
 * Vendor registers inside the EHCI window (EIIBC1/2 at +0x94/+0x9c, written
 * by the OAL) fall through to qy8.periph.
 */
#define QY8_EHCI_BASE       0xFFE70000
#define QY8_OHCI_BASE       0xFFE70400
#define QY8_USB_SPI         44
#define QY8_USBPHY_BASE     0xFFE70800
#define QY8_INT2_USB_OHCI   (1u << 0)
#define QY8_INT2_USB_EHCI   (1u << 1)

/* --- GPIO (R-Car, 6 banks of 0x1000) ---------------------------------- */


#define QY8_GPIO_BASE       0xFFC40000
#define QY8_GPIO_BANKS      6
#define QY8_GPIO_STRIDE     0x1000

#define GPIO_IOINTSEL   0x00
#define GPIO_INOUTSEL   0x04
#define GPIO_OUTDT      0x08
#define GPIO_INDT       0x0c
#define GPIO_INTDT      0x10
#define GPIO_INTCLR     0x14
#define GPIO_INTMSK     0x18
#define GPIO_MSKCLR     0x1c
#define GPIO_POSNEG     0x20
#define GPIO_EDGLEVEL   0x24
#define GPIO_FILONOFF   0x28

typedef struct Qy8Gpio {
    MemoryRegion mr;
    uint32_t iointsel, inoutsel, outdt;
    uint32_t intdt, intmsk, posneg, edglevel, filonoff;
    uint32_t in_level;          /* input line levels */
    qemu_irq parent_irq;
    DeviceState *tma460_reset_target;
    bool tma460_irq_enabled;
    int bank;
} Qy8Gpio;

#define GPIO4_TMA_RESET_BIT BIT(10)
#define GPIO4_TMA_IRQ_BIT   BIT(11)

static void qy8_gpio4_update_parent_irq(Qy8Gpio *g)
{
    if (g->bank == 4 && g->tma460_irq_enabled) {
        bool pending = (g->intdt & GPIO4_TMA_IRQ_BIT) &&
                       !(g->intmsk & GPIO4_TMA_IRQ_BIT);
        qemu_set_irq(g->parent_irq, pending);
    }
}

static bool qy8_gpio_reset_level(Qy8Gpio *g)
{
    return !!(g->outdt & g->inoutsel & GPIO4_TMA_RESET_BIT);
}

static void qy8_gpio4_tma_input(void *opaque, int n, int level)
{
    Qy8Gpio *g = opaque;
    bool old_level = !!(g->in_level & GPIO4_TMA_IRQ_BIT);
    bool new_level = !!level;

    if (new_level) {
        g->in_level |= GPIO4_TMA_IRQ_BIT;
    } else {
        g->in_level &= ~GPIO4_TMA_IRQ_BIT;
    }

    if (old_level == new_level) {
        return;
    }

    if (new_level && g->tma460_irq_enabled) {
        bool configured = (g->iointsel & GPIO4_TMA_IRQ_BIT) &&
                          (g->edglevel & GPIO4_TMA_IRQ_BIT) &&
                          !(g->posneg & GPIO4_TMA_IRQ_BIT) &&
                          !(g->inoutsel & GPIO4_TMA_IRQ_BIT);

        if (!configured) {
            qemu_log_mask(LOG_UNIMP,
                          "qy8.gpio4: TMA line rising with unsupported config IOINTSEL=%08x INOUTSEL=%08x POSNEG=%08x EDGLEVEL=%08x\n",
                          g->iointsel, g->inoutsel, g->posneg,
                          g->edglevel);
            return;
        }

        g->intdt |= GPIO4_TMA_IRQ_BIT;
        qemu_log_mask(LOG_UNIMP,
                      "qy8.gpio4: TMA rising edge latched INTDT[11]\n");
        qy8_gpio4_update_parent_irq(g);
    }
}

static uint64_t qy8_gpio_read(void *opaque, hwaddr addr, unsigned size)
{
    Qy8Gpio *g = opaque;

    switch (addr) {
    case GPIO_IOINTSEL:
        return g->iointsel;
    case GPIO_INOUTSEL:
        return g->inoutsel;
    case GPIO_OUTDT:
        return g->outdt;
    case GPIO_INDT:
        /* Outputs read back their written value; inputs read the line level. */
        return (g->outdt & g->inoutsel) | (g->in_level & ~g->inoutsel);
    case GPIO_INTMSK:
        return g->intmsk;
    case GPIO_INTDT:
        return g->bank == 4 && g->tma460_irq_enabled ? g->intdt : 0;
    case GPIO_POSNEG:
        return g->posneg;
    case GPIO_EDGLEVEL:
        return g->edglevel;
    case GPIO_FILONOFF:
        return g->filonoff;
    default:
        return 0;
    }
}

static void qy8_gpio_write(void *opaque, hwaddr addr, uint64_t val,
                           unsigned size)
{
    Qy8Gpio *g = opaque;
    bool old_reset_level = qy8_gpio_reset_level(g);

    switch (addr) {
    case GPIO_IOINTSEL:
        g->iointsel = val;
        break;
    case GPIO_INOUTSEL:
        g->inoutsel = val;
        break;
    case GPIO_OUTDT:
        g->outdt = val;
        break;
    case GPIO_INTMSK:
        g->intmsk = val;
        qy8_gpio4_update_parent_irq(g);
        break;
    case GPIO_INTCLR:
        if (g->bank == 4 && g->tma460_irq_enabled) {
            g->intdt &= ~(uint32_t)val;
            qemu_log_mask(LOG_UNIMP,
                          "qy8.gpio4: INTCLR write=0x%08x INTDT=0x%08x\n",
                          (uint32_t)val, g->intdt);
            qy8_gpio4_update_parent_irq(g);
        }
        break;
    case GPIO_MSKCLR:
        g->intmsk &= ~val;
        qy8_gpio4_update_parent_irq(g);
        break;
    case GPIO_POSNEG:
        g->posneg = val;
        break;
    case GPIO_EDGLEVEL:
        g->edglevel = val;
        break;
    case GPIO_FILONOFF:
        g->filonoff = val;
        break;
    default:
        break;
    }

    if (g->bank == 4 && g->tma460_reset_target &&
        (addr == GPIO_OUTDT || addr == GPIO_INOUTSEL)) {
        bool new_reset_level = qy8_gpio_reset_level(g);

        if (old_reset_level != new_reset_level) {
            qemu_log_mask(LOG_UNIMP,
                          "qy8.gpio4: reset OUTDT[10] %d -> %d\n",
                          old_reset_level, new_reset_level);
            if (object_dynamic_cast(OBJECT(g->tma460_reset_target),
                                    TYPE_CLARION_TMA616)) {
                clarion_tma616_set_reset(g->tma460_reset_target,
                                         new_reset_level);
            } else {
                clarion_tma460_set_reset(g->tma460_reset_target,
                                         new_reset_level);
            }
        }
    }
}

static const MemoryRegionOps qy8_gpio_ops = {
    .read = qy8_gpio_read,
    .write = qy8_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

/*
 * Levels on the board's input lines.
 *
 * GPIO1 bit 0 is the "stay in standby" signal. Loop @0x1148 reads INDT twice
 * and, if the bit is set, branches to 0x1188: r8 becomes 1, and when counter
 * r7 reaches zero, @0x120c checks r8 and starts watchdog reset @0x11e0. A one
 * on this pin therefore means "do not boot", not "board ready"; boot proceeds
 * only when the line is low. The same pin receives GPIO1.INTCLR bit 0 at
 * @0x1bfc, so it is also an interrupt source.
 */
static const uint32_t qy8_gpio_in_level[QY8_GPIO_BANKS] = {
    [1] = 0x00000000,
};


/* --- qy8.periph: broad intercept with caller context ------------------- */

/*
 * Like `unimplemented-device`, but also answers "who did this". Guest-visible
 * behavior is unchanged: reads return 0 and writes are discarded. Only a log
 * line is added.
 *
 * QY8_UNIMP_PC=N enables the extended format for the first N accesses to
 * each address:
 *
 *   qy8.periph: unimplemented device write (size 4, offset 0x0100730,
 *       value 0x00000000) pc=0x88011a2c lr=0xefd8b118 mode=0x13 t=61234
 *
 * `pc` is exact: it comes from translation block unwind data
 * (`cpu_unwind_state_data`); otherwise env would contain the block's start PC.
 * `t` is guest time in ms, using the same clock as the kernel console, so the
 * line can be correlated with a boot stage.
 *
 * Without the environment variable, the line format is byte-for-byte the
 * same as standard unimplemented-device output, preserving existing parsers.
 */
#define QY8_PERIPH_SEEN 2048        /* hash table: offset -> count */

#define QY8_PERIPH_BASE 0xF0000000
#define QY8_PERIPH_RANGES 8         /* number of ranges to always trace */

typedef struct Qy8Periph {
    MemoryRegion mr;
    unsigned trace_pc;          /* number of initial accesses to trace */
    hwaddr seen_off[QY8_PERIPH_SEEN];
    unsigned seen_cnt[QY8_PERIPH_SEEN];
    /* QY8_UNIMP_PC_ALL: physical address ranges with unlimited tracing */
    unsigned nranges;
    hwaddr rlo[QY8_PERIPH_RANGES], rhi[QY8_PERIPH_RANGES];
} Qy8Periph;

/*
 * Unwinding CPU state is expensive: doing it for every access slows the guest
 * by about 10x (A/B measurement: 5910 log lines in 45 s vs. 687). It is only
 * needed to identify the caller on the first few accesses to each address;
 * subsequent lines use the normal fast path.
 */
static bool qy8_periph_want_ctx(Qy8Periph *p, hwaddr addr)
{
    unsigned i = (unsigned)((addr >> 2) * 2654435761u) % QY8_PERIPH_SEEN;
    unsigned probe;

    for (probe = 0; probe < p->nranges; probe++) {
        hwaddr pa = QY8_PERIPH_BASE + addr;

        if (pa >= p->rlo[probe] && pa < p->rhi[probe]) {
            return true;        /* this range is under study; no limit */
        }
    }
    if (!p->trace_pc) {
        return false;
    }
    for (probe = 0; probe < 8; probe++) {
        unsigned k = (i + probe) % QY8_PERIPH_SEEN;

        if (p->seen_cnt[k] && p->seen_off[k] != addr) {
            continue;           /* collision; continue probing */
        }
        p->seen_off[k] = addr;
        p->seen_cnt[k]++;
        return p->seen_cnt[k] <= p->trace_pc;
    }
    return false;               /* table full: prefer fast path */
}

static void qy8_periph_ctx(char *buf, size_t len)
{
    CPUState *cs = current_cpu;
    CPUARMState *env;

    uint64_t data[4];    /* TARGET_INSN_START_WORDS is 3 for ARM */
    uint32_t pc;

    buf[0] = '\0';
    if (!cs) {
        return;
    }
    env = cpu_env(cs);
    /*
     * Exact PC of the instruction making the access. Use
     * cpu_unwind_state_data, NOT cpu_restore_state: the latter writes the
     * unwound state back to env, and the translation block continues with
     * altered registers. This was verified: the guest then takes a different
     * branch (A/B diverges around log line 680). This function only reads the
     * unwind data.
     */
    if (cpu_unwind_state_data(cs, cs->mem_io_pc, data)) {
        /*
         * With CF_PCREL (typical for system emulation), data[0] is an offset
         * within the page, not a full PC. Take the high bits from env because
         * a translation block never crosses a page boundary.
         */
        pc = (uint32_t)data[0];
        if (pc < qemu_target_page_size()) {
            pc |= (uint32_t)env->regs[15] & (uint32_t)qemu_target_page_mask();
        }
    } else {
        pc = (uint32_t)env->regs[15];   /* fallback: start of the block */
    }
    snprintf(buf, len, " pc=0x%08x lr=0x%08x mode=0x%02x t=%" PRId64,
             pc, (uint32_t)env->regs[14],
             (unsigned)(env->uncached_cpsr & CPSR_M),
             qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL));
}

static uint64_t qy8_periph_read(void *opaque, hwaddr addr, unsigned size)
{
    Qy8Periph *p = opaque;
    char ctx[96] = "";

    if (qy8_periph_want_ctx(p, addr)) {
        qy8_periph_ctx(ctx, sizeof(ctx));
    }
    qemu_log_mask(LOG_UNIMP, "qy8.periph: unimplemented device read  "
                  "(size %d, offset 0x%07" HWADDR_PRIx ")%s\n",
                  size, addr, ctx);
    return 0;
}

static void qy8_periph_write(void *opaque, hwaddr addr, uint64_t val,
                             unsigned size)
{
    Qy8Periph *p = opaque;
    char ctx[96] = "";

    if (qy8_periph_want_ctx(p, addr)) {
        qy8_periph_ctx(ctx, sizeof(ctx));
    }
    qemu_log_mask(LOG_UNIMP, "qy8.periph: unimplemented device write "
                  "(size %d, offset 0x%07" HWADDR_PRIx ", value 0x%0*" PRIx64
                  ")%s\n", size, addr, size << 1, val, ctx);
}

static const MemoryRegionOps qy8_periph_ops = {
    .read = qy8_periph_read,
    .write = qy8_periph_write,
    .impl.min_access_size = 1,
    .impl.max_access_size = 8,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

/* --- DBSC3 (DDR3 SDRAM Controller) @0xFE800000 ----------------------- */

/*
 * DDR3 controller. The boot reset stage (PC 0x17e0) writes DDR PHY
 * configuration, including 0x0f030a02 to DBPDCNT3 (offset 0x24). Later the
 * OAL (PC 0x88011f04) reads the same address and expects the value to persist.
 * Without this register, the write fell through to qy8.periph (-1000) and
 * was discarded.
 *
 * Other registers are not needed yet and are logged via LOG_UNIMP.
 */
#define QY8_DBSC3_BASE      0xFE800000
#define QY8_DBSC3_SIZE      0x10000

#define DBSC3_DBPDCNT3      0x24        /* PHY control register 3 */

typedef struct Qy8Dbsc3 {
    MemoryRegion mr;
    uint32_t dbpdcnt3;          /* offset 0x24 */
} Qy8Dbsc3;

static uint64_t qy8_dbsc3_read(void *opaque, hwaddr addr, unsigned size)
{
    Qy8Dbsc3 *d = opaque;

    switch (addr) {
    case DBSC3_DBPDCNT3:
        return d->dbpdcnt3;
    default:
        qemu_log_mask(LOG_UNIMP, "qy8.dbsc3: read  offset %#" HWADDR_PRIx
                      " (%u B)\n", addr, size);
        return 0;
    }
}

static void qy8_dbsc3_write(void *opaque, hwaddr addr, uint64_t val,
                             unsigned size)
{
    Qy8Dbsc3 *d = opaque;

    switch (addr) {
    case DBSC3_DBPDCNT3:
        d->dbpdcnt3 = val;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "qy8.dbsc3: write offset %#" HWADDR_PRIx
                      " = %#" PRIx64 "\n", addr, val);
        break;
    }
}

static const MemoryRegionOps qy8_dbsc3_ops = {
    .read = qy8_dbsc3_read,
    .write = qy8_dbsc3_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

/* --- CPG: MSTPCR latches (Module Stop Control) ------------------------ */

/*
 * MSTPCR0/1/3/4/5/6 control module clocks: bit 1 means "module stopped", 0
 * means "clocked". Names and addresses are not guessed; they appear in the
 * bootloader's register-name table (@0x24010) and match mstp0..5_clks@ffc800xx
 * in r8a7778.dtsi.
 *
 * Why this model exists: both the bootloader and OAL use read-modify-write:
 *
 *     v = MSTPCRn;  v &= ~mask;  MSTPCRn = v;   // start module
 *     if (MSTPCRn & mask) fail;                 // verify
 *     v = MSTPCRn;  v |=  mask;  MSTPCRn = v;   // stop module
 *     if ((MSTPCRn & mask) != mask) fail;       // verify
 *
 * When these addresses fell through to qy8.periph, writes were lost and
 * reads returned 0. This broke the RMW sequence between the bootloader and
 * WinCE, and the check after "stop" always failed.
 *
 * The model is a plain latch: reads return the last written value. Clocking
 * is not modeled, so it has no other effects.
 *
 * ⚠️ MSTPCR reset values are UNKNOWN (not present in available sources), so
 * the model starts at zero. This does not reproduce the SoC power-on state;
 * it only preserves values written by the guest. If reset values are found,
 * update this array.
 *
 * Deliberately leave status registers MSTPSR1/4/6 (0xFFC80044/48/4C) alone:
 * they continue to read as zero through qy8.periph, and zero means "module
 * running", which the OAL expects after starting a module.
 */
typedef struct Qy8Mstp {
    MemoryRegion mr;
    uint32_t val;
    hwaddr pa;
    const char *name;
} Qy8Mstp;

#define QY8_NUM_MSTP 6

static const struct { hwaddr pa; const char *name; } qy8_mstp_regs[QY8_NUM_MSTP] = {
    { 0xFFC80030, "MSTPCR0" },
    { 0xFFC80034, "MSTPCR1" },
    { 0xFFC8003C, "MSTPCR3" },
    { 0xFFC80050, "MSTPCR4" },
    { 0xFFC80054, "MSTPCR5" },
    { 0xFFC80058, "MSTPCR6" },
};

static uint64_t qy8_mstp_read(void *opaque, hwaddr addr, unsigned size)
{
    Qy8Mstp *p = opaque;

    return p->val;
}

static void qy8_mstp_write(void *opaque, hwaddr addr, uint64_t val,
                           unsigned size)
{
    Qy8Mstp *p = opaque;

    if (p->val != (uint32_t)val) {
        qemu_log_mask(LOG_UNIMP, "qy8.cpg: %s (%#" HWADDR_PRIx ") %#x -> %#"
                      PRIx64 "\n", p->name, p->pa, p->val, val);
    }
    p->val = val;
}

static const MemoryRegionOps qy8_mstp_ops = {
    .read = qy8_mstp_read,
    .write = qy8_mstp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

/* --- INTC2: demux status words ----------------------------------------- */

/*
 * Three 4-byte INTC2 words: 0xFE782048 (DU), 0xFE7820F4 (SDHI0),
 * 0xFE7820F8 (SDHI1).
 *
 * Why they are needed: none of these GIC lines lead directly to a driver.
 * OEMInterruptHandler (nk.exe @0x8800b5a0) has a case for each line; it first
 * masks the line in GICD_ICENABLER, then calls a demultiplexer. The demux
 * reads the status word, and only a nonzero value produces a logical IRQ:
 *
 *   GIC ID 63  (SPI 31, DU)    case 0x8800b6d8 -> demux 0x88011e94:
 *       v = MMIO32(0xFE782048);
 *       if (v & 0x0000FFFF) return OALIntrTranslateIrq(63);   // -> SYSINTR 37
 *       if (v & 0xFFFF0000) return OALIntrTranslateIrq(167);
 *       return SYSINTR_NOP;
 *
 *   GIC ID 119 (SPI 87, SDHI0) case 0x8800bb9c -> demux 0x8800da90:
 *       if (MMIO32(0xFE700008)) { MMIO32(0xFE782280) = 0x100;
 *                                 return OALIntrTranslateIrq(119); }
 *       if (MMIO32(0xFE7820F4) & 0xF) return OALIntrTranslateIrq(119);
 *       return SYSINTR_NOP;                                   // -> SYSINTR 20
 *
 *   GIC ID 120 (SPI 88, SDHI1) case 0x8800bbc4 -> demux 0x8800db68:
 *       same with word 0xFE7820F8, ACK 0x200 and IRQ 120. // -> SYSINTR 23
 *
 * Without these words, reads return zero, the demux returns SYSINTR_NOP, the
 * kernel does not call InterruptDone, and the line remains masked forever.
 *
 * The "device base -> logical IRQ" mapping comes from OALIntrRequestIrqs
 * (nk.exe @0x88012b34): 0xFFE4C000 -> 0x77 (119) @0x88012fc0,
 * 0xFFE4D000 -> 0x78 (120) @0x88012fe8, 0xFFE4F000 -> 0x76 (118).
 * The following values come from live OAL tables: g_oalIrq2SysIntr[119] = 20,
 * g_oalIrq2SysIntr[120] = 23, g_oalIrq2SysIntr[63] = 37.
 *
 * Not established: which exact bit in each word belongs to the device. The
 * OAL checks the whole half-word (DU) or bits 0..3 (SDHI), so the model sets
 * bit 0. This does not force the guest forward; the word reflects the REAL
 * line state rather than a constant.
 *
 * Deliberately leave branch 0xFE700008 at zero: this block is not modeled,
 * and the demux's second branch produces the same logical IRQ without writes.
 *
 * These words are read-only: nk.exe never writes them.
 */
#define QY8_INT2_STATUS_DU      0xFE782048
#define QY8_INT2_STATUS_SDHI0   0xFE7820F4
#define QY8_INT2_STATUS_SDHI1   0xFE7820F8
#define QY8_INT2_STATUS_USB     0xFE782058
#define QY8_INT2_STATUS_SHCORE 0xFE78208C
#define QY8_INT2_BIT            (1u << 0)   /* word bit is not established */
#define QY8_INT2_SHCORE_BIT     0x10000    /* confirmed for IRQ 86 */

typedef struct Qy8Int2Line {
    MemoryRegion mr;
    const char *name;
    hwaddr pa;
    uint32_t pending;           /* source bits holding the GIC line high */
} Qy8Int2Line;

static uint64_t qy8_int2_read(void *opaque, hwaddr addr, unsigned size)
{
    Qy8Int2Line *l = opaque;

    return l->pending;
}

static void qy8_int2_write(void *opaque, hwaddr addr, uint64_t val,
                           unsigned size)
{
    Qy8Int2Line *l = opaque;

    qemu_log_mask(LOG_UNIMP, "qy8.int2: write to status register %s %#"
                  HWADDR_PRIx " = %#" PRIx64 " (ignored)\n",
                  l->name, l->pa + addr, val);
}

static const MemoryRegionOps qy8_int2_ops = {
    .read = qy8_int2_read,
    .write = qy8_int2_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void qy8_int2_line_init(MemoryRegion *sysmem, Qy8Int2Line *l,
                               const char *name, hwaddr pa)
{
    l->name = name;
    l->pa = pa;
    l->pending = 0;
    memory_region_init_io(&l->mr, NULL, &qy8_int2_ops, l, name, 4);
    memory_region_add_subregion_overlap(sysmem, pa, &l->mr, 1);
}

/* --- machine ----------------------------------------------------------- */

#define TYPE_QY8_MACHINE MACHINE_TYPE_NAME("clarion-qy8")
OBJECT_DECLARE_SIMPLE_TYPE(Qy8MachineState, QY8_MACHINE)

typedef struct Qy8BoardProperties {
    const char *name;
    bool tma616;
} Qy8BoardProperties;

static const Qy8BoardProperties qy8_board_properties[] = {
    { "qy8652nb", false },
    { "qy8202na", true },
    { "qy8602nb", false },
    { "qy8600ne", false },
    { "qy8202pb", true },
    { "qy8202nc", true },
    { "qy8252nf", true },
};

/*
 * The NK2 image header names the firmware build, e.g. "G214ELNI.066". G214
 * builds are for the older board with a TMA616 touch controller (QY82xx:
 * 2014-2017 Leaf, e-NV200); G218 builds are for the ZE1 board with a TMA460.
 * A unit missing from the table above takes its board from this name.
 */
#define QY8_NK2_BUILD_OFFSET 0xac0016

static const Qy8BoardProperties qy8_firmware_boards[] = {
    { "G214", true },
    { "G218", false },
};

struct Qy8MachineState {
    MachineState parent;

    ARMCPU *cpu;
    DeviceState *gic;
    DeviceState *du;
    DeviceState *dmac;
    DeviceState *lbdma;      /* NOR read DMA @0xFF801000 */
    ClarionScif scif[QY8_NUM_SCIF];
    Qy8Hscif hscif0;
    DeviceState *micom;
    DeviceState *dispmicom;
    DeviceState *ublox;
    DeviceState *sdhi[QY8_NUM_SDHI];
    DeviceState *can;
    DeviceState *i2c4;
    DeviceState *tma460;
    DeviceState *ehci;
    DeviceState *ohci;
    DeviceState *sgx;         /* PowerVR SGX @0xFCE00000 */
    DeviceState *g2d;
    DeviceState *shcore;
    Qy8Tmu tmu;
    Qy8Gpio gpio[QY8_GPIO_BANKS];
    Qy8Bctl bctl;
    Qy8Dbsc3 dbsc3;
    Qy8Int2Line int2_du;
    Qy8Int2Line int2_sdhi[QY8_NUM_SDHI];
    Qy8Int2Line int2_usb;
    Qy8Int2Line int2_shcore;
    Qy8Mstp mstp[QY8_NUM_MSTP];
    Qy8Periph periph;
    MemoryRegion voidmr;

    uint8_t dipsw;              /* boot mode, machine property */
    uint32_t du_spi;            /* DU GIC line, machine property */
    uint32_t du_dotclk;         /* DU dot clock in Hz; 0 = no clock */
    bool micom_on;              /* built-in board MCU on SCIF4 */
    bool dispmicom_on;          /* built-in panel MCU on SCIF1 */
    bool gps_on;
    char *gps_lat;
    char *gps_lon;
    char *gps_speed;
    char *gps_course;
    bool i2c4_on;               /* opt-in bounded R-Car I2C4 model */
    bool i2c4_recorder_on;      /* opt-in I2C4 transaction recorder */
    bool tma460_on;             /* opt-in bounded TMA460 model */
    bool tma460_synthetic_profile_on; /* opt-in synthetic profile */
    bool i2c_empty_on;          /* opt-in: I2C0..I2C2 with empty buses */
    char *board;                /* property value: "auto" or canonical name */
    const ClarionBoardInfo *board_info; /* resolved in qy8_init() */
    const Qy8BoardProperties *board_properties;
    bool reverse;               /* RV input, machine property */
    bool g2d_on;
    bool shcore_on;
    bool shcore_init_only;
    char *shcore_mode;
    char *g2d_log;
    char *render;
    char *render_lib;
    char *render_log;
    char *render_dump_dir;

    MemoryRegion flash;          /* only when flash is provided as ROM */
    DriveInfo *flash_drive;      /* -drive if=pflash: writable copy */
    MemoryRegion ddr, ddr0, ddr1;
    MemoryRegion sram0, sram1;
    MemoryRegion ddr1_shadow;
};

/*
 * DU line: GIC SPI 31 plus a bit in the INTC2 status word. The OAL reads this
 * word in its demultiplexer (see qy8_int2_read), so raising the GIC line alone
 * is insufficient.
 */
static void qy8_du_irq(void *opaque, int n, int level)
{
    Qy8MachineState *s = opaque;

    s->int2_du.pending = level ? QY8_INT2_BIT : 0;
    qemu_set_irq(qdev_get_gpio_in(s->gic, s->du_spi), level);
}

/*
 * SDHI lines: like DU, each uses GIC SPI 87/88 and a bit in its INTC2 status
 * word; without it, the OAL demux returns SYSINTR_NOP.
 */
static void qy8_sdhi_irq(void *opaque, int n, int level)
{
    Qy8MachineState *s = opaque;

    s->int2_sdhi[n].pending = level ? QY8_INT2_BIT : 0;
    qemu_set_irq(qdev_get_gpio_in(s->gic, QY8_SDHI_SPI0 + n), level);
}

/* SH-core IRQ 86 is demultiplexed through the confirmed INTC2 status bit. */
static void qy8_shcore_irq(void *opaque, int n, int level)
{
    Qy8MachineState *s = opaque;

    (void)n;
    s->int2_shcore.pending = level ? QY8_INT2_SHCORE_BIT : 0;
    qemu_set_irq(qdev_get_gpio_in(s->gic, QY8_SHCORE_SPI), level);
}

/* n = 0 OHCI, 1 EHCI: one GIC line, the demux tells them apart by bit */
static void qy8_usb_irq(void *opaque, int n, int level)
{
    Qy8MachineState *s = opaque;
    uint32_t bit = n ? QY8_INT2_USB_EHCI : QY8_INT2_USB_OHCI;

    if (level) {
        s->int2_usb.pending |= bit;
    } else {
        s->int2_usb.pending &= ~bit;
    }
    qemu_set_irq(qdev_get_gpio_in(s->gic, QY8_USB_SPI),
                 s->int2_usb.pending != 0);
}

/*
 * Boards are named after the unit model, not after the car: one car
 * generation was fitted with units of more than one family.  The model is
 * stored in the NOR image, in a block that starts with "PROD": 8 ASCII
 * characters at +0x40.  The block is not at the same place in every unit
 * family, so a short list of offsets is probed.
 */
static const ClarionBoardInfo *qy8_board_by_name(const char *name)
{
    for (int i = 0; i < ARRAY_SIZE(clarion_boards); i++) {
        if (!g_strcmp0(name, clarion_boards[i].name)) {
            return clarion_boards[i].target &&
                   !strcmp(clarion_boards[i].target, "arm") ?
                   &clarion_boards[i] : NULL;
        }
    }
    return NULL;
}

static const Qy8BoardProperties *qy8_properties_by_name(const char *name)
{
    for (int i = 0; i < ARRAY_SIZE(qy8_board_properties); i++) {
        if (!strcmp(name, qy8_board_properties[i].name)) {
            return &qy8_board_properties[i];
        }
    }
    return NULL;
}

static const ClarionBoardInfo *qy8_board_by_model(const char *model)
{
    int untested;
    const ClarionBoardInfo *board = clarion_board_find(model, &untested);

    if (!board || strcmp(board->target, "arm")) {
        return NULL;
    }
    if (untested) {
        warn_report("clarion-qy8: unit model %s has not been tested",
                    model);
    }
    return board;
}

/* Read from the flash image before any device is built on top of it */
static bool qy8_flash_peek(MachineState *machine, hwaddr offset,
                           uint8_t *buf, size_t len)
{
    DriveInfo *dinfo = drive_get(IF_PFLASH, 0, 0);
    bool ok = false;

    if (dinfo) {
        BlockBackend *blk = blk_by_legacy_dinfo(dinfo);

        return blk_getlength(blk) >= offset + len &&
               blk_pread(blk, offset, len, buf, 0) >= 0;
    }
    if (machine->firmware) {
        FILE *f = fopen(machine->firmware, "rb");

        if (f) {
            ok = !fseeko(f, offset, SEEK_SET) && fread(buf, 1, len, f) == len;
            fclose(f);
        }
    }
    return ok;
}

/* Find the PROD block; on success @model is a printable C string */
static bool qy8_flash_model(MachineState *machine, hwaddr *prod_offset,
                            char model[CLARION_PROD_MODEL_LEN + 1],
                            uint8_t first_bytes[4])
{
    for (int i = 0; i < ARRAY_SIZE(clarion_prod_offsets); i++) {
        hwaddr off = clarion_prod_offsets[i];
        uint8_t sig[4] = { 0 };

        if (!qy8_flash_peek(machine, off, sig, sizeof(sig))) {
            continue;
        }
        if (i == 0) {
            memcpy(first_bytes, sig, sizeof(sig));
        }
        if (memcmp(sig, "PROD", 4) ||
            !qy8_flash_peek(machine, off + CLARION_PROD_MODEL_OFF,
                            (uint8_t *)model, CLARION_PROD_MODEL_LEN)) {
            continue;
        }
        for (int j = 0; j < CLARION_PROD_MODEL_LEN; j++) {
            if (!g_ascii_isgraph(model[j])) {
                model[j] = '.';
            }
        }
        model[CLARION_PROD_MODEL_LEN] = '\0';
        *prod_offset = off;
        return true;
    }
    return false;
}

static void qy8_resolve_board(Qy8MachineState *s, MachineState *machine)
{
    static const char hint[] = "choose one with "
        "board=qy8652nb|qy8602nb|qy8600ne|qy8202na|qy8202pb|qy8202nc|"
        "qy8252nf";
    const ClarionBoardInfo *detected = NULL;
    char model[CLARION_PROD_MODEL_LEN + 1] = "";
    uint8_t first[4] = { 0 };
    bool is_auto = !strcmp(s->board, "auto");
    hwaddr prod = 0;
    bool found;

    if (!drive_get(IF_PFLASH, 0, 0) && !machine->firmware) {
        return;                 /* qy8_init() reports the missing image */
    }

    found = qy8_flash_model(machine, &prod, model, first);
    if (found) {
        detected = qy8_board_by_model(model);
    }

    if (!is_auto) {
        s->board_info = qy8_board_by_name(s->board);
        if (found && detected != s->board_info) {
            warn_report("clarion-qy8: board=%s does not match unit model "
                        "%s in the flash image", s->board, model);
        }
    } else if (!found) {
        error_report("clarion-qy8: no PROD block in the flash image "
                     "(%02x %02x %02x %02x at 0x%" HWADDR_PRIx "); %s",
                     first[0], first[1], first[2], first[3],
                     clarion_prod_offsets[0], hint);
        exit(1);
    } else if (g_str_has_prefix(model, "QY7")) {
        error_report("clarion-qy8: unit model %s is a QY7-series unit "
                     "(SH-4); this machine does not support it", model);
        exit(1);
    } else if (!detected) {
        error_report("clarion-qy8: unknown unit model \"%s\" in the PROD "
                     "block at 0x%" HWADDR_PRIx "; %s", model, prod, hint);
        exit(1);
    } else {
        s->board_info = detected;
    }

    if (found) {
        info_report("clarion-qy8: unit model %s (PROD block at 0x%"
                    HWADDR_PRIx "), board %s (%s)", model, prod,
                    s->board_info->name, is_auto ? "auto" : "set explicitly");
    } else {
        info_report("clarion-qy8: no PROD block in the flash image, "
                    "board %s (set explicitly)", s->board_info->name);
    }
    s->board_properties = qy8_properties_by_name(s->board_info->name);
    if (!s->board_properties) {
        char build[13] = "";

        if (qy8_flash_peek(machine, QY8_NK2_BUILD_OFFSET, (uint8_t *)build,
                           sizeof(build) - 1)) {
            for (int i = 0; i < ARRAY_SIZE(qy8_firmware_boards); i++) {
                if (!strncmp(build, qy8_firmware_boards[i].name, 4)) {
                    s->board_properties = &qy8_firmware_boards[i];
                }
            }
        }
        for (int i = 0; i < sizeof(build) - 1; i++) {
            if (!g_ascii_isgraph(build[i])) {
                build[i] = '.';
            }
        }
        if (s->board_properties) {
            info_report("clarion-qy8: firmware %s, %s touch controller",
                        build, s->board_properties->tma616 ? "TMA616" :
                        "TMA460");
        } else {
            warn_report("clarion-qy8: firmware build \"%s\" not recognised; "
                        "using the TMA460 touch controller", build);
        }
    }
}

static bool qy8_has_tma616(Qy8MachineState *s)
{
    return s->board_properties && s->board_properties->tma616;
}

/* Parking brake on, lights off; reverse as the property says */
static void qy8_bctl_set_inputs(Qy8MachineState *s)
{
    uint16_t *st = &s->bctl.reg[BCTL_STATUS >> 1];

    *st = (*st & ~(BCTL_ST_PKB | BCTL_ST_RV)) | BCTL_ST_ILL;
    if (!s->reverse) {
        *st |= BCTL_ST_RV;
    }
}

static bool qy8_reverse_get(Object *obj, Error **errp)
{
    return QY8_MACHINE(obj)->reverse;
}

/* GPIO.dll polls the word, so qom-set at run time shifts into reverse */
static void qy8_reverse_set(Object *obj, bool value, Error **errp)
{
    Qy8MachineState *s = QY8_MACHINE(obj);

    s->reverse = value;
    qy8_bctl_set_inputs(s);
}

static char *qy8_render_get(Object *obj, Error **errp)
{
    (void)errp;
    return g_strdup(QY8_MACHINE(obj)->render);
}

static void qy8_render_set(Object *obj, const char *value, Error **errp)
{
    Qy8MachineState *s = QY8_MACHINE(obj);

    if (strcmp(value, "cpu") && strcmp(value, "angle") &&
        strcmp(value, "off")) {
        error_setg(errp, "render must be cpu, angle, or off");
        return;
    }
    g_free(s->render);
    s->render = g_strdup(value);
}

static char *qy8_render_lib_get(Object *obj, Error **errp)
{
    (void)errp;
    return g_strdup(QY8_MACHINE(obj)->render_lib);
}

static void qy8_render_lib_set(Object *obj, const char *value, Error **errp)
{
    Qy8MachineState *s = QY8_MACHINE(obj);

    (void)errp;
    g_free(s->render_lib);
    s->render_lib = g_strdup(value);
}

static char *qy8_render_log_get(Object *obj, Error **errp)
{
    (void)errp;
    return g_strdup(QY8_MACHINE(obj)->render_log);
}

static void qy8_render_log_set(Object *obj, const char *value, Error **errp)
{
    Qy8MachineState *s = QY8_MACHINE(obj);

    (void)errp;
    g_free(s->render_log);
    s->render_log = g_strdup(value);
}

static char *qy8_render_dump_dir_get(Object *obj, Error **errp)
{
    (void)errp;
    return g_strdup(QY8_MACHINE(obj)->render_dump_dir);
}

static void qy8_render_dump_dir_set(Object *obj, const char *value,
                                    Error **errp)
{
    Qy8MachineState *s = QY8_MACHINE(obj);

    (void)errp;
    g_free(s->render_dump_dir);
    s->render_dump_dir = g_strdup(value);
}

static void qy8_add_ram(MemoryRegion *sysmem, MemoryRegion *mr,
                        const char *name, hwaddr base, uint64_t size)
{
    memory_region_init_ram(mr, NULL, name, size, &error_fatal);
    memory_region_add_subregion(sysmem, base, mr);
}

static void qy8_init(MachineState *machine)
{
    Qy8MachineState *s = QY8_MACHINE(machine);
    MemoryRegion *sysmem = get_system_memory();
    SysBusDevice *gicbusdev;
    char *fname;
    ssize_t sz;
    int i;

    if (s->tma460_on && !s->i2c4_on) {
        error_report("clarion-qy8: tma460=on requires i2c4=on");
        exit(1);
    }
    if (s->tma460_on && s->i2c4_recorder_on) {
        error_report("clarion-qy8: tma460 and i2c4-recorder are mutually exclusive at 0x24");
        exit(1);
    }
    qy8_resolve_board(s, machine);

#ifdef CONFIG_PLUGIN
    if (strcmp(s->render, "off")) {
        clarion_qy8_render_configure(s->render, s->render_lib, s->render_log,
                                     s->render_dump_dir);
        qemu_plugin_load_builtin("clarion-qy8-render",
                                 clarion_qy8_render_install, &error_fatal);
    }
#endif

    s->cpu = ARM_CPU(object_new(machine->cpu_type));
    object_property_set_bool(OBJECT(s->cpu), "has_el3", false, &error_fatal);
    /* Reset starts at 0x0: CS0 flash, XIP. */
    object_property_set_int(OBJECT(s->cpu), "rvbar", 0, NULL);
    qdev_realize(DEVICE(s->cpu), NULL, &error_fatal);

    /* --- memory --- */

    /* Bottom layer: unmapped addresses read as zero instead of aborting. */
    memory_region_init_io(&s->voidmr, NULL, &qy8_void_ops, s,
                          "qy8.void", 0x100000000ULL);
    memory_region_add_subregion_overlap(sysmem, 0, &s->voidmr, -1500);

    /*
     * Provide CS0 flash to the guest in either of two ways.
     *
     * Without `-drive if=pflash`, use ROM populated from the dump by `-bios`.
     * Guest writes are discarded, flash IDs are unavailable, and erases are
     * impossible; this is sufficient for read-only tracing.
     *
     * With `-drive if=pflash,format=raw,file=COPY.bin`, use the chip model
     * (`cfi.pflash02`): the guest can read IDs, erase sectors and write words;
     * changes go to the COPY FILE. The source dump is not used or modified.
     */
    s->flash_drive = drive_get(IF_PFLASH, 0, 0);
    if (s->flash_drive) {
        DeviceState *fl = qdev_new(TYPE_PFLASH_CFI02);

        qdev_prop_set_drive(fl, "drive",
                            blk_by_legacy_dinfo(s->flash_drive));
        qdev_prop_set_uint32(fl, "num-blocks",
                             QY8_FLASH_SIZE / QY8_FLASH_SECTOR);
        qdev_prop_set_uint32(fl, "sector-length", QY8_FLASH_SECTOR);
        qdev_prop_set_uint8(fl, "width", 2);        /* x16 */
        qdev_prop_set_uint8(fl, "mappings", 0);
        qdev_prop_set_uint8(fl, "big-endian", 0);
        qdev_prop_set_uint16(fl, "id0", QY8_FLASH_MANUF_ID);
        qdev_prop_set_uint16(fl, "id1", QY8_FLASH_DEVICE_ID);
        qdev_prop_set_uint16(fl, "id2", 0x22a3); /* PC28F512M29AWxB ext. ID */
        qdev_prop_set_uint16(fl, "id3", 0x2201);
        /*
         * Unlock addresses. The bootloader writes byte addresses 0xAAAA and
         * 0x5554; the x16 model divides by 2 and keeps 11 bits, yielding
         * 0x555 and 0x2AA, the standard AMD command values.
         */
        qdev_prop_set_uint16(fl, "unlock-addr0", 0x555);
        qdev_prop_set_uint16(fl, "unlock-addr1", 0x2aa);
        qdev_prop_set_string(fl, "name", "qy8.flash");
        sysbus_realize_and_unref(SYS_BUS_DEVICE(fl), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(fl), 0, QY8_FLASH_BASE);
    } else {
        memory_region_init_rom(&s->flash, NULL, "qy8.flash",
                               QY8_FLASH_SIZE, &error_fatal);
        memory_region_add_subregion(sysmem, QY8_FLASH_BASE, &s->flash);
    }

#if QY8_DDR_ENABLED
    qy8_add_ram(sysmem, &s->ddr, "qy8.ddr", QY8_DDR_BASE, QY8_DDR_SIZE);
#endif
    qy8_add_ram(sysmem, &s->ddr0, "qy8.ddr0", QY8_DDR0_BASE, QY8_DDR0_SIZE);
    qy8_add_ram(sysmem, &s->ddr1, "qy8.ddr1", QY8_DDR1_BASE, QY8_DDR1_SIZE);
    qy8_add_ram(sysmem, &s->sram0, "qy8.sram0", QY8_SRAM0_BASE, QY8_SRAM0_SIZE);
    qy8_add_ram(sysmem, &s->sram1, "qy8.sram1", QY8_SRAM1_BASE, QY8_SRAM1_SIZE);

    /*
     * DDR1 shadow at PA 0x90000000 is a pipeline workaround, not a board feature.
     *
     * The bootloader's `Launch` (VA 0x97c1204c) disables the MMU and branches
     * to the physical trampoline on the very next instruction:
     *
     *      97c12118  mcr p15,0,r1,c1,c0,0   ; SCTLR.M := 0 - MMU disabled
     *      97c1211c  mov pc, r0             ; r0 = VAtoPA(0x97c12130) = 0x17c12130
     *      97c12120  nop x4                 ; pipeline flush padding
     *
     * On a real Cortex-A9, `mov pc,r0` is already in the pipeline while the
     * MMU is enabled, so instruction fetch uses the old mapping; the ARM ARM
     * does not guarantee SCTLR.M changes take effect before a context-
     * synchronizing operation. QEMU applies the write immediately, ends the
     * translation block, then fetches 0x97c1211c with the MMU disabled, as
     * PA 0x97C1211C. That address is unmapped here: prefetch abort -> PC=0xc
     * -> zero-slide through the vector table to the ASCII name at @0x28 ->
     * data abort -> infinite loop. This was observed during boot.
     * `info registers` (abt32, PC=0x10).
     *
     * Make those few words reachable at physical addresses matching their
     * VAs: DDR1 (PA 0x10000000, containing eboot) is also visible at
     * 0x90000000, matching the OEMAddressTable mapping VA 0x90000000 -> PA
     * 0x10000000. This window is needed only while the MMU is disabled but PC
     * is still virtual; execution then continues at real PAs (0x17c12130,
     * followed by the kernel at @0x08001000).
     */
    memory_region_init_alias(&s->ddr1_shadow, NULL, "qy8.ddr1-shadow",
                             &s->ddr1, 0, QY8_DDR1_SIZE);
    memory_region_add_subregion(sysmem, 0x90000000, &s->ddr1_shadow);

    /* --- flash contents: board dump via -bios --- */
    if (s->flash_drive) {
        /*
         * Contents already come from the `-drive if=pflash` block backend,
         * which receives guest erases and writes. `-bios` is unnecessary and
         * unsafe here: it would silently overwrite the copy with another file.
         */
        if (machine->firmware) {
            error_report("clarion-qy8: -bios and -drive if=pflash cannot be "
                         "used together; flash contents come from the copy");
            exit(1);
        }
    } else {
        if (!machine->firmware) {
            error_report("clarion-qy8: specify -bios <64 MB flash dump> or "
                         "-drive if=pflash,format=raw,file=<copy>");
            exit(1);
        }
        fname = g_strdup(machine->firmware);
        sz = load_image_mr(fname, &s->flash);
        if (sz < 0) {
            error_report("clarion-qy8: failed to read %s", fname);
            exit(1);
        }
        /*
         * Any CS0 dump is accepted, stock, modified or from another unit: the
         * machine does not parse it, but places the bytes at the reset vector.
         * It must be exactly 64 MB (without OOB data), or the rest of the
         * window stays zero and boot will fail.
         */
        if (sz != QY8_FLASH_SIZE) {
            warn_report("clarion-qy8: %s is %d bytes, not 64 MB; "
                        "the rest of the flash window remains zero", fname, (int)sz);
        }
        g_free(fname);
    }

    /* --- GIC (separate from the A9 R-Car Gen1 private region) --- */
    s->gic = qdev_new(TYPE_ARM_GIC);
    qdev_prop_set_uint32(s->gic, "revision", 2);
    qdev_prop_set_uint32(s->gic, "num-cpu", 1);
    qdev_prop_set_uint32(s->gic, "num-irq", QY8_NUM_IRQ);
    gicbusdev = SYS_BUS_DEVICE(s->gic);
    sysbus_realize_and_unref(gicbusdev, &error_fatal);
    sysbus_mmio_map(gicbusdev, 0, QY8_GIC_DIST_BASE);
    sysbus_mmio_map(gicbusdev, 1, QY8_GIC_CPU_BASE);
    sysbus_connect_irq(gicbusdev, 0,
                       qdev_get_gpio_in(DEVICE(s->cpu), ARM_CPU_IRQ));
    sysbus_connect_irq(gicbusdev, 1,
                       qdev_get_gpio_in(DEVICE(s->cpu), ARM_CPU_FIQ));

    if (s->g2d_on) {
        s->g2d = qdev_new(TYPE_CLARION_2DG);
        if (s->g2d_log && *s->g2d_log) {
            qdev_prop_set_string(s->g2d, "log", s->g2d_log);
        }
        sysbus_realize_and_unref(SYS_BUS_DEVICE(s->g2d), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(s->g2d), 0, 0xffe80000);
        sysbus_connect_irq(SYS_BUS_DEVICE(s->g2d), 0,
                           qdev_get_gpio_in(s->gic, 60));
    }

    if (s->shcore_on) {
        s->shcore = qdev_new(TYPE_CLARION_SHCORE);
        qdev_prop_set_bit(s->shcore, "init-only", s->shcore_init_only);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(s->shcore), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(s->shcore), 0, QY8_SHCORE_BASE);
        if (!s->shcore_init_only) {
            sysbus_mmio_map(SYS_BUS_DEVICE(s->shcore), 1,
                            QY8_SHCORE_SHARED_BASE);
        }
        sysbus_connect_irq(SYS_BUS_DEVICE(s->shcore), 0,
                           qemu_allocate_irq(qy8_shcore_irq, s, 0));
    }

    /*
     * --- I2C0..I2C2: same limited controller model, buses EMPTY ---
     *
     * Diagnostic mode: without a model, clients (tuner library, Usb.exe) wait
     * 3000 ms for an interrupt on every transaction. An empty bus immediately
     * returns NACK (MNR), its truthful response. Devices exist on the real
     * board, so this is a fast failure in place of a slow one and is disabled
     * by default. GIC IDs from OEMInterruptHandler are 99, 110, 108
     * (SPI = ID - 32).
     */
    if (s->i2c_empty_on) {
        static const struct { hwaddr base; int spi; } ctl[] = {
            { 0xffc70000, 67 }, { 0xffc71000, 78 }, { 0xffc72000, 76 },
        };
        for (int i = 0; i < ARRAY_SIZE(ctl); i++) {
            DeviceState *d = qdev_new(TYPE_CLARION_RCAR_I2C4);
            sysbus_realize_and_unref(SYS_BUS_DEVICE(d), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(d), 0, ctl[i].base);
            sysbus_connect_irq(SYS_BUS_DEVICE(d), 0,
                               qdev_get_gpio_in(s->gic, ctl[i].spi));
        }
    }

    /* --- I2C4: bounded model; other controllers stay unmodeled ------------ */
    if (s->i2c4_on) {
        I2CBus *bus;

        s->i2c4 = qdev_new(TYPE_CLARION_RCAR_I2C4);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(s->i2c4), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(s->i2c4), 0, 0xffc73000);
        /* GIC input indices are SPI numbers: ID 109 maps to SPI 77. */
        sysbus_connect_irq(SYS_BUS_DEVICE(s->i2c4), 0,
                           qdev_get_gpio_in(s->gic, QY8_I2C4_SPI));
        bus = I2C_BUS(qdev_get_child_bus(s->i2c4, "i2c"));
        if (s->i2c4_recorder_on) {
            i2c_slave_create_simple(bus, TYPE_CLARION_I2C4_RECORDER, 0x24);
        }
        if (s->tma460_on && qy8_has_tma616(s)) {
            /*
             * The QY8202NA unit has a TMA616 in the same place: application
             * on 0x67, bootloader on 0x69, same reset and interrupt pins.
             */
            s->tma460 = DEVICE(i2c_slave_create_simple(bus,
                                                       TYPE_CLARION_TMA616,
                                                       0x67));
        } else if (s->tma460_on) {
            s->tma460 = DEVICE(i2c_slave_create_simple(bus,
                                                       TYPE_CLARION_TMA460,
                                                       0x24));
            clarion_tma460_set_synthetic_profile(
                s->tma460, s->tma460_synthetic_profile_on);
        }
    }

    /*
     * --- HPB-DMAC ---
     *
     * Bases and interrupts from hpb_dmae_resources[] in setup-r8a7778.c:
     * channel registers 0xffc08000/0x1000, shared registers 0xffc09000/0x170,
     * gic_iid(0x7b), and 5 consecutive lines (IRQ 123..127 = SPI 91..95).
     * Place before SCIF because SCIF receivers deliver bytes here.
     */
    s->dmac = qdev_new(TYPE_CLARION_HPBDMA);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(s->dmac), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(s->dmac), 0, CLARION_HPBDMA_CHAN_BASE);
    sysbus_mmio_map(SYS_BUS_DEVICE(s->dmac), 1, CLARION_HPBDMA_COMM_BASE);
    for (i = 0; i < CLARION_HPBDMA_QY8_NUM_IRQ; i++) {
        sysbus_connect_irq(SYS_BUS_DEVICE(s->dmac), i,
                           qdev_get_gpio_in(s->gic,
                                            CLARION_HPBDMA_IRQ_BASE_SPI + i));
    }

    /*
     * --- Board companion MCU (micom) on SCIF4 ---
     *
     * This is a separate chip on the board and is enabled by default. Without
     * it, WinCE cannot initialize the link and boot does not reach GWES.
     * Disable with `-M clarion-qy8,micom=off`; SCIF4 then acts as a normal
     * port for an external responder connected through -serial.
     */
    if (s->micom_on) {
        s->micom = qdev_new(TYPE_CLARION_MICOM);
        qdev_realize_and_unref(s->micom, NULL, &error_fatal);
    }

    /*
     * --- Display panel MCU (Display Micom) on SCIF1 ---
     *
     * A separate chip in the display unit, also present on the real vehicle,
     * so enabled by default. Without it, `lcddrv.dll` opens `SCI1:`, sends
     * 10 02 23 10 03 30 and waits forever in WaitCommEvent: ACM module 129 is
     * never signaled, and DrawOi renders its first frame only after the 5 s
     * wait for 129 expires. Disable with `-M clarion-qy8,dispmicom=off`; SCIF1
     * then acts as a normal port for an external responder on -serial.
     */
    if (s->dispmicom_on) {
        s->dispmicom = qdev_new(TYPE_CLARION_DISPMICOM);
        /* stable path /machine/panel-micom for injecting keys with qom-set */
        object_property_add_child(OBJECT(s), "panel-micom",
                                  OBJECT(s->dispmicom));
        qdev_realize_and_unref(s->dispmicom, NULL, &error_fatal);
    }

    if (s->gps_on) {
        s->ublox = qdev_new(TYPE_CLARION_UBLOX);
        qdev_realize_and_unref(s->ublox, NULL, &error_fatal);
        clarion_ublox_set_position(s->ublox,
                                   g_ascii_strtod(s->gps_lat, NULL),
                                   g_ascii_strtod(s->gps_lon, NULL),
                                   g_ascii_strtod(s->gps_speed, NULL),
                                   g_ascii_strtod(s->gps_course, NULL));
    }

    /* --- SCIF --- */
    for (i = 0; i < QY8_NUM_SCIF; i++) {
        ClarionScif *sc = &s->scif[i];
        char *name = g_strdup_printf("qy8.scif%d", i);

        clarion_scif_init(sc, sysmem,
                          QY8_SCIF_BASE + i * QY8_SCIF_STRIDE, name, i,
                          qy8_scif_chr_index(i),
                          qdev_get_gpio_in(s->gic, QY8_SCIF_SPI0 + i),
                          s->dmac,
                          i == QY8_SCIF_DEBUG &&
                          g_strcmp0(getenv("QY8_SCIF3_TXI"), "0") != 0);
        sc->dma_feed = clarion_hpbdma_feed;
        sc->dma_eod = clarion_hpbdma_eod;
        sc->micom_tx = clarion_micom_rx_byte;
        sc->dispmicom_tx = clarion_dispmicom_rx_byte;
        sc->ublox_tx = clarion_ublox_rx_byte;
        if (i == QY8_SCIF_MICOM && s->micom) {
            sc->micom = s->micom;
            clarion_micom_set_sink(s->micom, qy8_micom_sink, sc);
        }
        if (i == QY8_SCIF_DISPMICOM && s->dispmicom) {
            sc->dispmicom = s->dispmicom;
            clarion_dispmicom_set_sink(s->dispmicom, qy8_dispmicom_sink, sc);
        }
        if (i == 2 && s->ublox) {
            sc->ublox = s->ublox;
            clarion_ublox_set_sink(s->ublox, qy8_ublox_sink, sc);
        }
        g_free(name);
    }

    /* --- HSCIF0 --- */
    /*
     * Narrow 0x60 window with priority 1: overrides broad qy8.periph (-1000)
     * without affecting anything else. Three unused kilobytes lie between
     * SCIF5 (0xFFE45FFF) and HSCIF0. The seventh -serial (index 6, after six
     * SCIF ports) is connected here when specified; otherwise it has no peer,
     * like an unconnected board connector.
     */
    s->hscif0.tx = timer_new_ns(QEMU_CLOCK_VIRTUAL, qy8_hscif_tx_expire,
                                &s->hscif0);
    s->hscif0.idle = timer_new_ns(QEMU_CLOCK_VIRTUAL, qy8_hscif_idle_expire,
                                  &s->hscif0);
    s->hscif0.irq = qdev_get_gpio_in(s->gic, QY8_HSCIF_SPI);
    memory_region_init_io(&s->hscif0.mr, NULL, &qy8_hscif_ops, &s->hscif0,
                          "qy8.hscif0", QY8_HSCIF_SIZE);
    memory_region_add_subregion_overlap(sysmem, QY8_HSCIF_BASE,
                                        &s->hscif0.mr, 1);
    if (serial_hd(QY8_NUM_SCIF)) {
        qemu_chr_fe_init(&s->hscif0.chr, serial_hd(QY8_NUM_SCIF),
                         &error_abort);
        qemu_chr_fe_set_handlers(&s->hscif0.chr, qy8_hscif_can_receive,
                                 qy8_hscif_receive, NULL, NULL,
                                 &s->hscif0, NULL, true);
    }

    /* --- TMU --- */
    for (i = 0; i < QY8_TMU_CHANS; i++) {
        Qy8TmuChan *c = &s->tmu.ch[i];

        c->irq = qdev_get_gpio_in(s->gic, QY8_TMU_SPI0 + i);
        c->ptimer = ptimer_init(qy8_tmu_tick, c, PTIMER_POLICY_LEGACY);
        c->tcor = 0xffffffff;
    }
    memory_region_init_io(&s->tmu.mr, NULL, &qy8_tmu_ops, &s->tmu,
                          "qy8.tmu", 0x30);
    memory_region_add_subregion(sysmem, QY8_TMU_BASE, &s->tmu.mr);

    /* --- GPIO --- */
    for (i = 0; i < QY8_GPIO_BANKS; i++) {
        Qy8Gpio *g = &s->gpio[i];
        char *name = g_strdup_printf("qy8.gpio%d", i);

        g->bank = i;
        g->in_level = qy8_gpio_in_level[i];
        memory_region_init_io(&g->mr, NULL, &qy8_gpio_ops, g, name, 0x1000);
        memory_region_add_subregion(sysmem,
                                    QY8_GPIO_BASE + i * QY8_GPIO_STRIDE,
                                    &g->mr);
        g_free(name);
    }

    if (s->tma460_on) {
        Qy8Gpio *g = &s->gpio[4];

        g->tma460_irq_enabled = true;
        g->parent_irq = qdev_get_gpio_in(s->gic, QY8_GPIO4_SPI);
        g->tma460_reset_target = s->tma460;
        qdev_connect_gpio_out(s->tma460, 0,
                              qemu_allocate_irq(qy8_gpio4_tma_input,
                                                g, 11));
    }

    /* --- board controller: DIPSW and standby exit enable --- */
    s->bctl.reg[BCTL_STATUS >> 1] = BCTL_ST_PWR | BCTL_ST_BOOT;
    qy8_bctl_set_inputs(s);
    s->bctl.reg[BCTL_DIPSW >> 1] = s->dipsw & 7;
    memory_region_init_io(&s->bctl.mr, NULL, &qy8_bctl_ops, &s->bctl,
                          "qy8.bctl", QY8_BCTL_SIZE);
    /* Overrides the sram1 window because the board uses the same CS. */
    memory_region_add_subregion_overlap(sysmem, QY8_BCTL_BASE,
                                        &s->bctl.mr, 1);

    /* USB-PHY: overlays the broad qy8.periph window below. */
    clarion_usbphy_init(sysmem, QY8_USBPHY_BASE, 2, "qy8.usbphy");

    /*
     * The EHCI window is 4 KiB and would cover OHCI and the PHY, so those
     * two sit one priority higher. OHCI is the companion for full/low speed.
     */
    s->ehci = qdev_new(TYPE_PLATFORM_EHCI);
    qdev_prop_set_bit(s->ehci, "companion-enable", true);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(s->ehci), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(s->ehci), 0, QY8_EHCI_BASE, 1);
    sysbus_connect_irq(SYS_BUS_DEVICE(s->ehci), 0,
                       qemu_allocate_irq(qy8_usb_irq, s, 1));

    s->ohci = qdev_new(TYPE_SYSBUS_OHCI);
    qdev_prop_set_string(s->ohci, "masterbus",
                         SYS_BUS_EHCI(s->ehci)->ehci.bus.qbus.name);
    qdev_prop_set_uint32(s->ohci, "num-ports", EHCI_PORTS);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(s->ohci), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(s->ohci), 0, QY8_OHCI_BASE, 2);
    sysbus_connect_irq(SYS_BUS_DEVICE(s->ohci), 0,
                       qemu_allocate_irq(qy8_usb_irq, s, 0));

    /* --- Display Unit: actual QEMU window --- */
    /*
     * Interrupt line GIC_SPI 31. The model derives frame period from HCR/VCR,
     * programmed by the guest; dot clock comes from property `dotclk`
     * (ESCR02 = 0 selects an external clock, which is not in the registers).
     */
    s->du = qdev_new(TYPE_CLARION_DU);
    qdev_set_id(s->du, g_strdup("qy8-du"), &error_fatal);
    qdev_prop_set_uint32(s->du, "dotclk", s->du_dotclk);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(s->du), &error_fatal);
    if (s->tma460 && qy8_has_tma616(s)) {
        clarion_tma616_bind_pointer_input(s->tma460, "qy8-du", &error_fatal);
    } else if (s->tma460 && s->tma460_synthetic_profile_on) {
        clarion_tma460_bind_pointer_input(s->tma460, "qy8-du", &error_fatal);
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(s->du), 0, QY8_DU_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(s->du), 0,
                       qemu_allocate_irq(qy8_du_irq, s, 0));

    /*
     * INTC2 demux status words: priority 1 overrides the unmapped space
     * (qy8.void, -1500), which previously returned zero here.
     */
    qy8_int2_line_init(sysmem, &s->int2_du, "qy8.int2.du",
                       QY8_INT2_STATUS_DU);
    qy8_int2_line_init(sysmem, &s->int2_sdhi[0], "qy8.int2.sdhi0",
                       QY8_INT2_STATUS_SDHI0);
    qy8_int2_line_init(sysmem, &s->int2_sdhi[1], "qy8.int2.sdhi1",
                       QY8_INT2_STATUS_SDHI1);
    qy8_int2_line_init(sysmem, &s->int2_usb, "qy8.int2.usb",
                       QY8_INT2_STATUS_USB);
    if (s->shcore_on) {
        qy8_int2_line_init(sysmem, &s->int2_shcore, "qy8.int2.shcore",
                           QY8_INT2_STATUS_SHCORE);
    }

    /*
     * CPG MSTPCR: six separate 4-byte windows with priority 1 over broad
     * qy8.periph (-1000). Separate windows preserve the existing behavior
     * for other CPG registers (FRQCR, MSTPSR), which remain visible in
     * `-d unimp`.
     */
    for (int i = 0; i < QY8_NUM_MSTP; i++) {
        char nm[32];

        s->mstp[i].val = 0;         /* reset value unknown; see comment */
        s->mstp[i].pa = qy8_mstp_regs[i].pa;
        s->mstp[i].name = qy8_mstp_regs[i].name;
        snprintf(nm, sizeof(nm), "qy8.cpg.%s", qy8_mstp_regs[i].name);
        memory_region_init_io(&s->mstp[i].mr, NULL, &qy8_mstp_ops,
                              &s->mstp[i], nm, 4);
        memory_region_add_subregion_overlap(sysmem, qy8_mstp_regs[i].pa,
                                            &s->mstp[i].mr, 1);
    }

    /*
     * --- SDHI0/SDHI1: SD controllers ---
     *
     * Both controllers always exist, even without an image: they are part of
     * the SoC, not the card. An empty slot stays empty; commands receive no
     * response and the controller sets CMDTIMEOUT.
     *
     * Insert a card with `-drive if=sd,index=N,format=raw,file=...` (N = 0
     * for the main slot, 1 for the second) or the short form `-sd FILE`.
     */
    for (i = 0; i < QY8_NUM_SDHI; i++) {
        DriveInfo *di = drive_get(IF_SD, 0, i);

        s->sdhi[i] = qdev_new(TYPE_RENESAS_SDHI);
        qdev_prop_set_uint8(s->sdhi[i], "unit", i);
        /*
         * Data transfers use HPB-DMAC: the controller signals that a block is
         * ready, and DMAC reads it from this controller's SD_BUF0 address
         * (hpb_chan_module[] maps the channel to that address).
         */
        object_property_set_link(OBJECT(s->sdhi[i]), "dmac", OBJECT(s->dmac),
                                 &error_fatal);
        qdev_prop_set_uint64(s->sdhi[i], "dma-buf-addr",
                             QY8_SDHI_BASE + i * QY8_SDHI_STRIDE +
                             QY8_SDHI_BUF0);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(s->sdhi[i]), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(s->sdhi[i]), 0,
                        QY8_SDHI_BASE + i * QY8_SDHI_STRIDE);
        sysbus_connect_irq(SYS_BUS_DEVICE(s->sdhi[i]), 0,
                           qemu_allocate_irq(qy8_sdhi_irq, s, i));
        if (di) {
            DeviceState *card = qdev_new(TYPE_SD_CARD);

            qdev_prop_set_drive_err(card, "drive",
                                    blk_by_legacy_dinfo(di), &error_fatal);
            qdev_realize_and_unref(card,
                                   qdev_get_child_bus(s->sdhi[i], "sd-bus"),
                                   &error_fatal);
        }
        /*
         * The board chip reads card-detect from these buses (see
         * qy8_bctl_read). Bind them here, rather than when creating bctl,
         * because the controllers are created later; the guest reads the
         * register after machine initialization.
         */
        s->bctl.sd[i] = SD_BUS(qdev_get_child_bus(s->sdhi[i], "sd-bus"));
    }

    /*
     * --- LBSC DMAC @0xFF801000 ---------------------------------------
     *
     * DMA engine used by `Flash.dll` to read parallel NOR: IOCTL `0x01112020`
     * (`FlashReadToPhysMem`) sends no CFI commands to the chip; it programs
     * {SAR, DAR, TCR} here and starts the transfer. Without the model,
     * qy8.periph discarded the request and left the destination zero, while
     * the driver waited 5 seconds for an interrupt that could never arrive.
     *
     * GIC line is taken from firmware, not analogy: its OAL assigns IRQ 0x6F
     * for `LogicalLoc = 0xFF801000`. The OAL's logical IRQ is the GIC INTID,
     * so SPI = 111 - 32 = 79 (see clarion_lbdma.h).
     */
    s->lbdma = qdev_new(TYPE_CLARION_LBDMA);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(s->lbdma), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(s->lbdma), 0, CLARION_LBDMA_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(s->lbdma), 0,
                       qdev_get_gpio_in(s->gic, CLARION_LBDMA_SPI));

    /*
     * --- CAN @0xFFFD1000 ---------------------------------------------
     *
     * Board controller accessed by `CAN.dll` and `AntiTheft.exe` through
     * `CAN1:`. The model implements only the controller's own modes; it does
     * not include vehicle buses (BCM, meter, HVAC). No interrupt is connected:
     * the CAN GIC SPI number is unproven, and the model raises no interrupt.
     */
    /*
     * QY8_CAN=off removes the model, so the block falls through to qy8.periph
     * and `CAN.dll` times out again. This switch is for before/after
     * comparison only, so it is an environment variable, not a machine property.
     */
    if (g_strcmp0(getenv("QY8_CAN"), "off") != 0) {
        s->can = qdev_new(TYPE_RENESAS_CAN);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(s->can), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(s->can), 0, QY8_CAN_BASE);
    }

    /*
     * --- PowerVR SGX @0xFCE00000 -------------------------------------
     *
     * The SGX block has its own window and device model instead of the broad
     * interceptor. Guest-visible behavior is unchanged: reads still return
     * zero, there are no interrupts, and `ui32InitStatus` is untouched, so
     * `SGXInitialise` reaches the same timeout. On `EUR_CR_EVENT_KICK2`, the
     * model reports visible state: address-carrying registers, a full walk of
     * the SGX page directory, and the object at `PDS_EXEC_BASE + reg 0x0A68`.
     * Rationale and deferred work are documented in clarion_sgx.c.
     */
    if (g_strcmp0(getenv("QY8_SGX"), "off") != 0) {
        s->sgx = qdev_new(TYPE_CLARION_SGX);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(s->sgx), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(s->sgx), 0, CLARION_SGX_BASE);
    }

    /* --- remaining peripherals: log accesses only (-d unimp) --- */
    create_unimplemented_device("qy8.cs1",   QY8_CS1_BASE, QY8_CS1_SIZE);
    create_unimplemented_device("qy8.l2c",   QY8_L2C_BASE, 0x1000);
    /* --- DBSC3: DDR3 controller, above qy8.periph priority (-1000) --- */
    s->dbsc3.dbpdcnt3 = 0;      /* reset: offset 0x24 reads as zero */
    memory_region_init_io(&s->dbsc3.mr, NULL, &qy8_dbsc3_ops, &s->dbsc3,
                          "qy8.dbsc3", QY8_DBSC3_SIZE);
    memory_region_add_subregion_overlap(sysmem, QY8_DBSC3_BASE,
                                        &s->dbsc3.mr, 1);
    create_unimplemented_device("qy8.intc2", 0xFE780000, 0x1000);
    create_unimplemented_device("qy8.cpg",   0xFFC80000, 0x1000);
    create_unimplemented_device("qy8.pfc",   0xFFFC0000, 0x1000);
    create_unimplemented_device("qy8.rst",   0xFFCC0000, 0x1000);
    /* Broad interceptor: log everything else from 0xF0000000 upward. */
    /*
     * Broad interceptor for other accesses from 0xF0000000 upward. This model
     * replaces `unimplemented-device` only to let QY8_UNIMP_PC=1 add caller
     * pc/lr/time; guest-visible behavior is identical.
     */
    /*
     * QY8_UNIMP_PC=N traces pc/lr/time for the first N accesses to EACH
     * address (empty or 1 means once). Later lines use the normal format, so
     * state-unwind cost is paid once per address, not on every access.
     */
    {
        const char *e = getenv("QY8_UNIMP_PC");
        const char *r = getenv("QY8_UNIMP_PC_ALL");

        s->periph.trace_pc = e ? (atoi(e) > 0 ? atoi(e) : 1) : 0;
        /*
         * QY8_UNIMP_PC_ALL="0xffe80000-0xffe81000,0xfff18000-0xfff19000" —
         * Always include context for accesses in these PHYSICAL ranges,
         * regardless of count. This allows focused tracing of one block
         * without unwinding state for hot addresses in other peripherals.
         */
        while (r && *r && s->periph.nranges < QY8_PERIPH_RANGES) {
            char *end;
            uint64_t lo = strtoull(r, &end, 0);

            if (end == r || *end != '-') {
                break;
            }
            r = end + 1;
            s->periph.rhi[s->periph.nranges] = strtoull(r, &end, 0);
            s->periph.rlo[s->periph.nranges] = lo;
            if (end == r) {
                break;
            }
            s->periph.nranges++;
            r = (*end == ',') ? end + 1 : end;
        }
    }
    memory_region_init_io(&s->periph.mr, NULL, &qy8_periph_ops, &s->periph,
                          "qy8.periph", 0x10000000);
    memory_region_add_subregion_overlap(sysmem, 0xF0000000,
                                        &s->periph.mr, -1000);
}

static char *qy8_board_get(Object *obj, Error **errp)
{
    return g_strdup(QY8_MACHINE(obj)->board);
}

static void qy8_board_set(Object *obj, const char *value, Error **errp)
{
    Qy8MachineState *s = QY8_MACHINE(obj);

    if (g_strcmp0(value, "auto") && !qy8_board_by_name(value)) {
        error_setg(errp, "board must be auto, qy8652nb, qy8602nb, "
                   "qy8600ne, qy8202na, qy8202pb, qy8202nc or qy8252nf");
        return;
    }
    g_free(s->board);
    s->board = g_strdup(value);
}

static bool qy8_gps_get(Object *obj, Error **errp)
{
    (void)errp;
    return QY8_MACHINE(obj)->gps_on;
}

static void qy8_gps_set(Object *obj, bool value, Error **errp)
{
    (void)errp;
    QY8_MACHINE(obj)->gps_on = value;
}

static void qy8_gps_position_update(Qy8MachineState *s)
{
    if (s->ublox) {
        clarion_ublox_set_position(s->ublox,
                                   g_ascii_strtod(s->gps_lat, NULL),
                                   g_ascii_strtod(s->gps_lon, NULL),
                                   g_ascii_strtod(s->gps_speed, NULL),
                                   g_ascii_strtod(s->gps_course, NULL));
    }
}

static char *qy8_gps_lat_get(Object *obj, Error **errp)
{
    (void)errp;
    return g_strdup(QY8_MACHINE(obj)->gps_lat);
}

static void qy8_gps_lat_set(Object *obj, const char *value, Error **errp)
{
    Qy8MachineState *s = QY8_MACHINE(obj);
    char *end;
    double number = g_ascii_strtod(value, &end);

    if (end == value || *end || !isfinite(number) || fabs(number) > 90) {
        error_setg(errp, "gps-lat must be a decimal latitude in [-90, 90]");
        return;
    }
    g_free(s->gps_lat);
    s->gps_lat = g_strdup(value);
    qy8_gps_position_update(s);
}

static char *qy8_gps_lon_get(Object *obj, Error **errp)
{
    (void)errp;
    return g_strdup(QY8_MACHINE(obj)->gps_lon);
}

static void qy8_gps_lon_set(Object *obj, const char *value, Error **errp)
{
    Qy8MachineState *s = QY8_MACHINE(obj);
    char *end;
    double number = g_ascii_strtod(value, &end);

    if (end == value || *end || !isfinite(number) || fabs(number) > 180) {
        error_setg(errp, "gps-lon must be a decimal longitude in [-180, 180]");
        return;
    }
    g_free(s->gps_lon);
    s->gps_lon = g_strdup(value);
    qy8_gps_position_update(s);
}

static char *qy8_gps_speed_get(Object *obj, Error **errp)
{
    (void)errp;
    return g_strdup(QY8_MACHINE(obj)->gps_speed);
}

static void qy8_gps_speed_set(Object *obj, const char *value, Error **errp)
{
    Qy8MachineState *s = QY8_MACHINE(obj);
    char *end;
    double number = g_ascii_strtod(value, &end);

    if (end == value || *end || !isfinite(number) || number < 0) {
        error_setg(errp, "gps-speed must be a non-negative speed in knots");
        return;
    }
    g_free(s->gps_speed);
    s->gps_speed = g_strdup(value);
    qy8_gps_position_update(s);
}

static char *qy8_gps_course_get(Object *obj, Error **errp)
{
    (void)errp;
    return g_strdup(QY8_MACHINE(obj)->gps_course);
}

static void qy8_gps_course_set(Object *obj, const char *value, Error **errp)
{
    Qy8MachineState *s = QY8_MACHINE(obj);
    char *end;
    double number = g_ascii_strtod(value, &end);

    if (end == value || *end || !isfinite(number) || number < 0 ||
        number >= 360) {
        error_setg(errp, "gps-course must be in [0, 360) degrees");
        return;
    }
    g_free(s->gps_course);
    s->gps_course = g_strdup(value);
    qy8_gps_position_update(s);
}

static bool qy8_micom_get(Object *obj, Error **errp)
{
    return QY8_MACHINE(obj)->micom_on;
}

static void qy8_micom_set(Object *obj, bool value, Error **errp)
{
    QY8_MACHINE(obj)->micom_on = value;
}

static bool qy8_dispmicom_get(Object *obj, Error **errp)
{
    return QY8_MACHINE(obj)->dispmicom_on;
}

static void qy8_dispmicom_set(Object *obj, bool value, Error **errp)
{
    QY8_MACHINE(obj)->dispmicom_on = value;
}

static bool qy8_i2c4_get(Object *obj, Error **errp)
{
    return QY8_MACHINE(obj)->i2c4_on;
}

static void qy8_i2c4_set(Object *obj, bool value, Error **errp)
{
    QY8_MACHINE(obj)->i2c4_on = value;
}

static bool qy8_i2c4_recorder_get(Object *obj, Error **errp)
{
    return QY8_MACHINE(obj)->i2c4_recorder_on;
}

static void qy8_i2c4_recorder_set(Object *obj, bool value, Error **errp)
{
    QY8_MACHINE(obj)->i2c4_recorder_on = value;
}

static bool qy8_i2c_empty_get(Object *obj, Error **errp)
{
    return QY8_MACHINE(obj)->i2c_empty_on;
}

static void qy8_i2c_empty_set(Object *obj, bool value, Error **errp)
{
    QY8_MACHINE(obj)->i2c_empty_on = value;
}

static bool qy8_tma460_get(Object *obj, Error **errp)
{
    return QY8_MACHINE(obj)->tma460_on;
}

static void qy8_tma460_set(Object *obj, bool value, Error **errp)
{
    QY8_MACHINE(obj)->tma460_on = value;
}

static bool qy8_tma460_profile_get(Object *obj, Error **errp)
{
    return QY8_MACHINE(obj)->tma460_synthetic_profile_on;
}

static void qy8_tma460_profile_set(Object *obj, bool value, Error **errp)
{
    QY8_MACHINE(obj)->tma460_synthetic_profile_on = value;
}

static bool qy8_g2d_get(Object *obj, Error **errp)
{
    return QY8_MACHINE(obj)->g2d_on;
}

static void qy8_g2d_set(Object *obj, bool value, Error **errp)
{
    QY8_MACHINE(obj)->g2d_on = value;
}

static char *qy8_shcore_get(Object *obj, Error **errp)
{
    return g_strdup(QY8_MACHINE(obj)->shcore_mode);
}

static void qy8_shcore_set(Object *obj, const char *value, Error **errp)
{
    Qy8MachineState *s = QY8_MACHINE(obj);

    if (strcmp(value, "on") && strcmp(value, "init") && strcmp(value, "off")) {
        error_setg(errp, "invalid shcore mode '%s' (expected on, init, or off)",
                   value);
        return;
    }
    g_free(s->shcore_mode);
    s->shcore_mode = g_strdup(value);
    s->shcore_on = strcmp(value, "off") != 0;
    s->shcore_init_only = strcmp(value, "init") == 0;
}

static char *qy8_g2d_log_get(Object *obj, Error **errp)
{
    return g_strdup(QY8_MACHINE(obj)->g2d_log);
}

static void qy8_g2d_log_set(Object *obj, const char *value, Error **errp)
{
    Qy8MachineState *s = QY8_MACHINE(obj);

    g_free(s->g2d_log);
    s->g2d_log = g_strdup(value);
}

static void qy8_machine_instance_init(Object *obj)
{
    Qy8MachineState *s = QY8_MACHINE(obj);

    s->render = g_strdup("cpu");
    s->render_lib = g_strdup("");
    s->render_log = g_strdup("");
    s->render_dump_dir = g_strdup("");
    object_property_add_str(obj, "render", qy8_render_get, qy8_render_set);
    object_property_set_description(obj, "render",
        "render backend: cpu, angle, or off");
    object_property_add_str(obj, "render-lib", qy8_render_lib_get,
                            qy8_render_lib_set);
    object_property_set_description(obj, "render-lib",
        "path to the qy8r shared library for render=angle");
    object_property_add_str(obj, "render-log", qy8_render_log_get,
                            qy8_render_log_set);
    object_property_set_description(obj, "render-log",
        "optional renderer JSONL log path");
    object_property_add_str(obj, "render-dump-dir", qy8_render_dump_dir_get,
                            qy8_render_dump_dir_set);
    object_property_set_description(obj, "render-dump-dir",
        "optional directory for renderer frame and draw dumps");

    /*
     * Default 5 = "NORM(RES)>>", normal boot with full debug output; on the
     * real board this matches grounding TEST_B1. The stock value on an
     * unmodified board is 7 (quiet boot).
     */
    s->du_spi = QY8_DU_SPI;
    object_property_add_uint32_ptr(obj, "du-spi", &s->du_spi,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_set_description(obj, "du-spi",
        "GIC line number (SPI) for the DU frame interrupt");

    /*
     * DU dot clock. 33333333 Hz is an assumed reference clock, not a
     * measured board value; with it the guest reaches its consent screen
     * without extra machine options. Zero disables frame ticks.
     */
    s->du_dotclk = 33333333;
    object_property_add_uint32_ptr(obj, "du-dotclk", &s->du_dotclk,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_set_description(obj, "du-dotclk",
        "DU dot clock frequency in Hz (0 = frame clock disabled)");

    object_property_add_bool(obj, "reverse", qy8_reverse_get,
                             qy8_reverse_set);
    object_property_set_description(obj, "reverse",
        "reverse gear input (RV), off by default");

    s->board = g_strdup("auto");
    object_property_add_str(obj, "board", qy8_board_get, qy8_board_set);
    object_property_set_description(obj, "board",
        "board peripherals by unit model: auto (default, read from the "
        "flash image), qy8652nb, qy8602nb, qy8600ne, qy8202na, "
        "qy8202pb, qy8202nc or qy8252nf");

    s->dipsw = QY8_DIPSW_NORM_RES;
    object_property_add_uint8_ptr(obj, "dipsw", &s->dipsw,
                                  OBJ_PROP_FLAG_READWRITE);
    object_property_set_description(obj, "dipsw",
        "boot mode from board controller DIPSW, 0..7 (5 = NORM(RES) with log)");

    /*
     * Board companion MCU. Enabled by default because the chip is present on
     * hardware and the kernel cannot initialize the link without its response.
     */
    s->micom_on = true;
    object_property_add_bool(obj, "micom", qy8_micom_get, qy8_micom_set);
    object_property_set_description(obj, "micom",
        "built-in board companion MCU on SCIF4 (off to connect an external "
        "responder through -serial)");

    /*
     * Display panel MCU. Enabled by default because it is present on hardware;
     * without it, lcddrv does not signal ACM module 129.
     */
    s->dispmicom_on = true;
    object_property_add_bool(obj, "dispmicom", qy8_dispmicom_get,
                             qy8_dispmicom_set);
    object_property_set_description(obj, "dispmicom",
        "built-in display panel MCU on SCIF1 (off to connect an external "
        "responder through -serial)");

    s->gps_on = true;
    s->gps_lat = g_strdup("50.4501");
    s->gps_lon = g_strdup("30.5234");
    s->gps_speed = g_strdup("0");
    s->gps_course = g_strdup("0");
    object_property_add_bool(obj, "gps", qy8_gps_get, qy8_gps_set);
    object_property_set_description(obj, "gps",
                                    "built-in u-blox receiver on SCIF2");
    object_property_add_str(obj, "gps-lat", qy8_gps_lat_get, qy8_gps_lat_set);
    object_property_add_str(obj, "gps-lon", qy8_gps_lon_get, qy8_gps_lon_set);
    object_property_add_str(obj, "gps-speed", qy8_gps_speed_get,
                            qy8_gps_speed_set);
    object_property_add_str(obj, "gps-course", qy8_gps_course_get,
                            qy8_gps_course_set);

    s->i2c4_on = true;
    object_property_add_bool(obj, "i2c4", qy8_i2c4_get, qy8_i2c4_set);
    object_property_set_description(obj, "i2c4",
        "Bounded I2C4 controller model at 0xffc73000 (on by default; off disables it)");

    s->i2c4_recorder_on = false;
    object_property_add_bool(obj, "i2c4-recorder", qy8_i2c4_recorder_get,
                             qy8_i2c4_recorder_set);
    object_property_set_description(obj, "i2c4-recorder",
        "opt-in transaction recorder on I2C4 address 0x24");

    s->i2c_empty_on = false;
    object_property_add_bool(obj, "i2c-empty", qy8_i2c_empty_get,
                             qy8_i2c_empty_set);
    object_property_set_description(obj, "i2c-empty",
        "opt-in diagnostic mode: I2C0..I2C2 with empty buses (NACK instead "
        "of a 3-second timeout)");

    s->tma460_on = true;
    object_property_add_bool(obj, "tma460", qy8_tma460_get,
                             qy8_tma460_set);
    object_property_set_description(obj, "tma460",
        "Bounded TMA460 bootloader model at I2C4 address 0x24 (on by default; requires i2c4=on); "
        "TMA616 at 0x67 with board=qy8202na");

    s->tma460_synthetic_profile_on = true;
    object_property_add_bool(obj, "tma460-profile",
                             qy8_tma460_profile_get,
                             qy8_tma460_profile_set);
    object_property_set_description(obj, "tma460-profile",
        "Minimal synthetic TMA460 System Mode profile with pointer-to-touch input (on by default)");

    s->g2d_on = true;
    object_property_add_bool(obj, "g2d", qy8_g2d_get, qy8_g2d_set);
    object_property_set_description(obj, "g2d",
        "minimal synthetic 2DG completion model (on by default)");

    s->shcore_mode = g_strdup("on");
    s->shcore_on = true;
    object_property_add_str(obj, "shcore", qy8_shcore_get, qy8_shcore_set);
    object_property_set_description(
        obj, "shcore",
        "SH model: on enables synthetic CPUCOM startup, init keeps only "
        "init-end, off disables it");
    s->g2d_log = g_strdup("");
    object_property_add_str(obj, "g2d-log", qy8_g2d_log_get,
                            qy8_g2d_log_set);
    object_property_set_description(obj, "g2d-log",
        "optional JSONL log of 2DG command lists");
}

static void qy8_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    static const char * const valid_cpu_types[] = {
        ARM_CPU_TYPE_NAME("cortex-a9"),
        NULL
    };

    mc->desc = "Clarion QY8XXX head unit (Renesas R8A7778, WinCE)";
    mc->init = qy8_init;
    mc->min_cpus = 1;
    mc->max_cpus = 1;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-a9");
    mc->valid_cpu_types = valid_cpu_types;
    mc->default_ram_size = QY8_DDR0_SIZE + QY8_DDR1_SIZE;
    mc->no_floppy = 1;
    mc->no_cdrom = 1;
    mc->no_parallel = 1;
    /* Machine creates RAM itself, following the OEMAddressTable map. */
    mc->default_ram_id = NULL;
}

static const TypeInfo qy8_machine_types[] = {
    {
        .name           = TYPE_QY8_MACHINE,
        .parent         = TYPE_MACHINE,
        .instance_size  = sizeof(Qy8MachineState),
        .instance_init  = qy8_machine_instance_init,
        .class_init     = qy8_machine_class_init,
    },
};

DEFINE_TYPES(qy8_machine_types)
