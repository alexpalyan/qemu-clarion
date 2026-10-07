/*
 * Clarion QY8XXX head unit (Nissan Leaf ZE1) — емуляція плати.
 *
 * SoC: Renesas R8A7778 (R-Car M1A), одноядерний Cortex-A9.
 * ОС:  Windows CE, XIP просто з флеш на CS0 (reset-вектор @PA 0).
 *
 * Карта пам'яті знята з OEMAddressTable самого завантажувача
 * (nand_20260827.bin @0x11c2c), а не вгадана:
 *
 *   VA 80000000 -> PA 00000000  64 MB   флеш CS0 (той самий дамп)
 *   VA 84000000 -> PA 04000000   8 MB   CS1
 *   VA 84800000 -> PA 18000000   8 MB
 *   VA 85000000 -> PA 18800000   8 MB
 *   VA 85800000 -> PA f0000000   8 MB   L2C (PL310 @f0100000)
 *   VA 88000000 -> PA 08000000 128 MB   DDR (сюди вантажиться ядро)
 *   VA 90000000 -> PA 10000000 128 MB   DDR (тут виконується eboot)
 *   VA 9c000000 -> PA fc000000  64 MB   периферія
 *
 * Периферія — стандартні блоки Renesas; адреси збігаються з r8a7778.dtsi
 * і з константами, що лежать у коді eboot.
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

/* --- фізична карта плати --------------------------------------------- */

#define QY8_FLASH_BASE      0x00000000
#define QY8_FLASH_SIZE      (64 * MiB)

/*
 * Мікросхема CS0 — паралельна NOR, а не NAND (ім'я файлів дампа історичне).
 * Це видно з самого завантажувача: `NCG2K2_FlashRom` (флеш 0x1114c) читає
 * ID послідовністю AMD — `AA`@0xAAAA, `55`@0x5554, `90`@0xAAAA — і потім
 * слова за 0xA0000000/0xA0000002, а стирання (0x113d8) додає `80`,`AA`,`55`
 * і `30` за адресою сектора. Детектор (0x1127c) друкує
 * `ManuID/DevID` і зводить g_wSelectFlashType:
 *
 *   ManuID 0x89 або 0x1F + DevID 0x227E -> 0x30  "MICRON_PC28F512"
 *   ManuID 0x20          + DevID 0x227E -> 0x227E
 *   ManuID 0x01                          -> 0     "NORMAL"
 *
 * Береться перший варіант: 0x89/0x227E — це Micron PC28F512M29EW, 512 Мбіт
 * (рівно 64 МБ дампа), x16, однорідні сектори по 128 КБ, набір команд AMD.
 * Крок сектора збігається з тим, що робить сам монітор: `backupcr` іде по
 * блоках із кроком 0x20000 саме для типу 0x30 (0x11804..0x11828), і для
 * решти типів — 0x10000.
 */
#define QY8_FLASH_SECTOR    (128 * KiB)
#define QY8_FLASH_MANUF_ID  0x0089      /* Micron/Intel */
#define QY8_FLASH_DEVICE_ID 0x227e      /* PC28F512M29EW */

#define QY8_CS1_BASE        0x04000000
#define QY8_CS1_SIZE        (8 * MiB)

#define QY8_DDR0_BASE       0x08000000      /* образ ядра, VA 0x88000000 */
#define QY8_DDR0_SIZE       (128 * MiB)
#define QY8_DDR1_BASE       0x10000000      /* eboot, VA 0x90000000      */
#define QY8_DDR1_SIZE       (128 * MiB)

#define QY8_SRAM0_BASE      0x18000000
#define QY8_SRAM0_SIZE      (8 * MiB)
#define QY8_SRAM1_BASE      0x18800000
#define QY8_SRAM1_SIZE      (8 * MiB)

/*
 * Головне вікно DDR.
 *
 * reset-stage @PA 0x17e0 пише в DBCONF0 (DBSC3 + 0x24; ім'я взято з таблиці
 * монітора самого eboot, флеш 0x24088) значення 0x0f030a02. OAL @0x88011ef0
 * читає той самий регістр і за полем [28:24] обирає гілку: 0x0d -> 256 МБ,
 * 0x0e -> 512 МБ, інакше -> 1024 МБ. Решта полів (rows 15, banks 8, cols 10,
 * шина 32 біт) дає рівно 1 ГіБ, тож гілка 1024 узгоджена сама з собою.
 *
 * У контролера є лише один DBCONF0 і один DBRNK0 -> одна безперервна область.
 * Три вікна, які OAL роздає як extension DRAM у гілці 1024 (0x44000000 +64 МБ,
 * 0x58000000 +128 МБ, 0x60000000 +480 МБ), разом займають [0x44000000,
 * 0x7e000000); єдине природно вирівняне вікно на 1 ГіБ, що їх вміщує,
 * починається з 0x40000000. Самотест завантажувача (DIPSW=4, CTP_DramBank)
 * незалежно перевіряє 0x40000000 і 0x58000000 як банки DRAM.
 *
 * ⚠️ Це ще не остаточна модель плати. qy8.ddr0 (PA 0x08000000) і qy8.ddr1
 * (PA 0x10000000) лишаються тут окремою пам'яттю, хоча на залізі вони,
 * найпевніше, alias цієї ж DDR: діра 0x48000000..0x58000000 розміром рівно
 * 256 МБ виключена з обох гілок і точно дорівнює тим 256 МБ, які
 * OEMAddressTable ядра відображає низько. Але цей зв'язок лише виведено, а
 * не доведено, тож поки що гість бачить більше пам'яті, ніж є на платі.
 */
#define QY8_DDR_BASE        0x40000000
#define QY8_DDR_SIZE        (1 * GiB)
/* Перемикач для контрольного A/B: 0 — вікно не створюється взагалі. */
#define QY8_DDR_ENABLED     1

/* eboot виконується звідси (VA 0x97C00000 -> PA 0x17C00000) */
#define QY8_EBOOT_PA        0x17C00000

/* GIC — як у r8a7778.dtsi; обидві бази присутні в коді eboot */
#define QY8_GIC_CPU_BASE    0xFE430000
#define QY8_GIC_DIST_BASE   0xFE438000
#define QY8_NUM_IRQ         256

#define QY8_L2C_BASE        0xF0100000

/*
 * Display Unit — display@fff80000 reg = <0xfff80000 0x40000> (r8a7779.dtsi).
 * Модель — hw/display/clarion_du.c. Дисплей програмує ЗАВАНТАЖУВАЧ: він
 * вмикає 800x480 на каналі 0 і лишає DOOR = 0. Регістрів площини він не
 * пише взагалі, тож вікно до M3b лишається чорним — це стан системи, а не
 * вада моделі (docs/20, розділ «M4»).
 */
#define QY8_DU_BASE         0xFFF80000
/*
 * Лінія переривання DU — **GIC_SPI 31**, підтверджено 24.09.2026 (див.
 * docs/04-journal.md і docs/20-qemu-board.md, «Топологія переривань»):
 * OEMInterruptHandler має для GIC id 63 власний case, а таблиці OAL із
 * живого гостя дають g_oalIrq2SysIntr[63] = 37 — рівно той SYSINTR, який
 * бере ddi_ncg.dll. Раніше здавалося, що OAL на SPI 31 не реагує: насправді
 * обробник доходив до демукса INTC2 (0xFE782048) і вмирав там, бо моделі
 * цього слова не було.
 *
 * Номер лишається властивістю машини `du-spi` — зручно для дослідів.
 */
#define QY8_DU_SPI          31          /* підтверджено таблицями OAL */
#define QY8_I2C4_SPI        77          /* GIC ID 109 = SPI 77 */
#define QY8_GPIO4_SPI       103         /* GIC ID 135 = SPI 103 */
#define QY8_SHCORE_SPI      54          /* GIC INTID 86 = SPI 54 */
#define QY8_SHCORE_BASE     0xFE700000

#define QY8_SCIF_BASE       0xFFE40000      /* scif0..scif5, крок 0x1000 */
#define QY8_SCIF_STRIDE     0x1000
#define QY8_NUM_SCIF        6

/*
 * Консоль завантажувача — SCIF3 (0xFFE43000), а не SCIF0. Це не здогад:
 * OEMInitDebugSerial @VA 0x97c14758 будує базу двома інструкціями
 * (mov r3,#0x3000; sub r3,r3,#0x1c0000 -> 0xFFE43000) і кладе її в глобал
 * 0x97cc249c, з якого putchar @0x97c14784 бере base+0x10 (SCFSR.TDFE) і
 * base+0x0c (SCFTDR). Тому перший -serial (при -nographic це stdio)
 * чіпляємо саме до SCIF3, решту — до наступних за порядком.
 */
#define QY8_SCIF_DEBUG      3

/* SCIF4 — лінк до супутнього МК плати (docs/20, «M3b»), не консоль. */
#define QY8_SCIF_MICOM      4

/*
 * SCIF1 — лінк до МК панелі дисплея (Display Micom), не консоль. Реєстр
 * гостя: Drivers\Launch\SCIF1 -> serial_scif.dll, Prefix 'SCI', Index 1,
 * тобто ім'я потоку `SCI1:`, яке відкриває CLcdDrv::SCIF_Init @0xEF71724C.
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
 * SDHI0/SDHI1 — два слоти SD (mmc@ffe4c000 / mmc@ffe4d000 у r8a7778.dtsi).
 * У реєстрі пристрою це Drivers\SD\Card (PortNumber 0, профіль SDProfile,
 * «Main Slot SD Card») і Drivers\SD\Card2 (PortNumber 1, SDProfile2,
 * «Sub Slot SD Card»). Обидва обслуговує SDHC.dll.
 *
 * Переривання обов'язкове: свою гілку опитування (WaitEvent @0xefa318d8,
 * режим 1) драйвер запускає з нульовим таймаутом, тож вона завершується
 * помилкою одразу, і завершення команди він чекає лише від ISR.
 */
#define QY8_CAN_BASE        0xFFFD1000
#define QY8_SDHI_BASE       0xFFE4C000
#define QY8_SDHI_STRIDE     0x1000
#define QY8_NUM_SDHI        2
/* SD_BUF0 — порт даних; саме його читає DMAC (hpb_chan_module[21/22]) */
#define QY8_SDHI_BUF0       0x30

/*
 * Лінії GIC. OALIntrRequestIrqs (nk.exe @0x88012b34) дає логічний IRQ за
 * фізичною базою пристрою: 0xFFE4C000 -> 119 (@0x88012fc0),
 * 0xFFE4D000 -> 120 (@0x88012fe8). Логічний IRQ тут тотожний GIC ID, а
 * SPI n = ID n + 32, тобто SDHI0 = SPI 87, SDHI1 = SPI 88 — рівно те, що
 * стоїть у r8a7778.dtsi для mmc@ffe4c000 / mmc@ffe4d000. Далі OAL мапить
 * IRQ 119 -> SYSINTR 20 і IRQ 120 -> SYSINTR 23 (дамп g_oalIrq2SysIntr).
 */
#define QY8_SDHI_SPI0       87

/* --- контролер плати на CS-шині @0x18800000 --------------------------- */

#define QY8_BCTL_BASE       0x18800000
#define QY8_BCTL_SIZE       0x40

#define BCTL_STATUS         0x00
#define BCTL_DIPSW          0x02

/* Біти статусу, які опитує завантажувач у циклі @0x1148 (див. нижче). */
#define BCTL_ST_PWR         0x0080
#define BCTL_ST_BOOT        0x0400

/*
 * Vehicle inputs in the same word, from GPIO.dll's register reader (ZE0
 * @0xEF6F31E0, ZE1 @0xEF6C3204): IOCTL_GPIO_ACCESS_* -> ldrh [base] >> bit.
 * GPIO.dll polls them and signals EVT_GPIO_* on a change. All active low.
 * ST_PWR is IGN (bit 7) and ST_BOOT is NAVI_ON (bit 10) in the same table.
 */
#define BCTL_ST_PKB         0x0020      /* bit 5: 0 = parking brake on */
#define BCTL_ST_ILL         0x0200      /* bit 9: ILL_MR, 0 = lights on */
#define BCTL_ST_RV          0x1000      /* bit 12: 0 = in reverse */

/*
 * Card-detect обох слотів SD живе в тому самому 16-бітному слові, що й DIPSW.
 * Знято з бітових аксесорів `GPIO.dll` (читання vtbl+0x1C0 @VA 0xEF6C3204,
 * запис vtbl+0x1C4 @0xEF6C3850; base = змаплений PA 0x18800000):
 *
 *   IOCTL 0x800A2174 -> ldrh [base+0x02] >> 5 & 1   card-detect слота 0
 *   IOCTL 0x800A20EC -> ldrh [base+0x02] >> 6 & 1   card-detect слота 1
 *
 * До них ходить `SDHC.dll`: `SD_Sense_Card` (@0xEFA2D888) читає біт через
 * `GPI1:`, а `CSDHCAccess::SDHC_CardCheck` (@0xEFA29ED4) віддає «картка є»
 * лише коли біт дорівнює НУЛЮ (@0xEFA2D904: `*out = (bit == 1) ? 2 : 1`,
 * @0xEFA29FDC: результат 1 лише при `*out == 1`). Тобто лінія активно-низька:
 * порожній слот із підтяжкою читається як 1. Збій IOCTL прошивка теж трактує
 * як «немає картки» — це її безпечний дефолт.
 *
 * ⚠ Слово +0x02 суто вхідне: у записувальному аксесорі для нього немає жодної
 * гілки (усі 19 вихідних защіпок лежать у +0x04 і +0x00). Тому тут можна
 * накладати біти при читанні, не боячись затерти щось, що пише гість.
 */
#define BCTL_CD0            0x0020      /* біт 5: слот 0, 0 = картка є */
#define BCTL_CD1            0x0040      /* біт 6: слот 1, 0 = картка є */

/* Режими DIPSW — таблиця переходів eboot @VA 0x97c07e28 */
#define QY8_DIPSW_NORM_RES  5           /* "NORM(RES)>>" — як із TEST_B1 */

/* --- SCIF (Renesas serial, 16-бітні регістри) ------------------------- */

#define SCIF_SCSMR      0x00
#define SCIF_SCBRR      0x04
#define SCIF_SCSCR      0x08
#define SCIF_SCFTDR     0x0C
#define SCIF_SCFSR      0x10
#define SCIF_SCFRDR     0x14
#define SCIF_SCFCR      0x18
#define SCIF_SCFDR      0x1C
#define SCIF_SCSPTR     0x20
#define SCIF_SCLSR      0x24

/* SCFSR (мапа sh-sci.h: DR, RDF, PER, FER, BRK, TDFE, TEND, ER) */
#define SCFSR_DR        0x0001
#define SCFSR_RDF       0x0002
#define SCFSR_TDFE      0x0020
#define SCFSR_TEND      0x0040

/* SCFCR: RTRG[7:6] — рівень запуску приймача, RFRST — скидання FIFO прийому */
#define SCFCR_RFRST     0x0002
#define QY8_SCIF_FIFO   16

/*
 * DR («receive data ready») SCIF зводить, коли у FIFO є байти, їх менше за
 * рівень запуску, і лінія мовчить ~15 бітових інтервалів. Швидкість гість
 * задає не через SCBRR (пише туди 0), тож беремо сталу паузу — важлива не
 * її точність, а сам факт «посилка скінчилася».
 */
#define QY8_SCIF_IDLE_NS    200000

typedef struct Qy8Scif {
    MemoryRegion mr;
    CharFrontend chr;
    qemu_irq irq;
    DeviceState *dmac;          /* кому віддавати прийняті байти */
    DeviceState *micom;         /* супутній МК на тому ж дроті, якщо є */
    DeviceState *dispmicom;     /* МК панелі на тому ж дроті, якщо є */
    DeviceState *ublox;         /* приймач GNSS на тому ж дроті, якщо є */
    hwaddr base;                /* фізична база — щоб назвати SCFRDR для DMA */
    uint16_t scsmr, scscr, scfcr;
    uint8_t fifo[QY8_SCIF_FIFO];
    unsigned fifo_len;
    bool dr;                    /* прийом завершився паузою */
    QEMUTimer *idle;
    int index;
    bool txi;                   /* зводити TXI при TIE (лише SCIF3) */
} Qy8Scif;

/* SCSCR — дозволи переривань (мапа sh-sci.h) */
#define SCSCR_RIE       0x0040
#define SCSCR_TIE       0x0080

/* RTRG[7:6] -> скільки байтів у FIFO запускають запит DMA (мапа SCIF) */
static unsigned qy8_scif_rtrg(Qy8Scif *s)
{
    static const unsigned lvl[4] = { 1, 4, 8, 14 };
    return lvl[(s->scfcr >> 6) & 3];
}

static bool qy8_scif_rdf(Qy8Scif *s)
{
    return s->fifo_len >= qy8_scif_rtrg(s);
}

static void qy8_scif_update_irq(Qy8Scif *s)
{
    /*
     * Переривання приймача просять RDF і DR, і обидва — лише при RIE.
     * ⚠ Перевірено дослідом: якщо підняти лінію по DR без RIE (є спокуса
     * тлумачити біт 2 SCSCR як TOIE з sh-sci.h), драйвер SCIF4 входить в
     * обробник, читає SCFSR/SCSCR/SCLSR, нічого не бере з SCFRDR і не гасить
     * причину — виходить нескінченний шторм (219 тис. входів за 16 с). Тобто
     * такого джерела на цьому SCIF драйвер не знає.
     *
     * Передавач у нас завжди порожній (TDFE стоїть постійно), тож TXI — це
     * просто TIE. serial_scif.dll передає по перериваннях: кладе байт,
     * вмикає TIE і чекає TXI; коли черга спорожніла, сам знімає TIE. Без TXI
     * кожен запис у SCI3: висить до наступного вхідного байта, а модулі, що
     * пишуть у debug shell, блокуються й бут не доходить до AUI (перевірено
     * дослідом, nissan-can-explore docs/32). Вмикаємо лише на SCIF3: на
     * решті портів поведінку драйвера з TIE не перевірено.
     */
    qemu_set_irq(s->irq,
                 ((qy8_scif_rdf(s) || s->dr) && (s->scscr & SCSCR_RIE)) ||
                 (s->txi && (s->scscr & SCSCR_TIE)));
}

/*
 * Віддати байти DMA. Запит DMA приймача — це той самий RXI, і SCIF зводить
 * його або по RDF (набрався рівень запуску RTRG), або по DR (посилка
 * скінчилася, у FIFO лишилося менше за рівень). Що RDF і DR — одне й те саме
 * джерело RXI, видно з `sh-sci.c`: в `sci_rx_interrupt()` драйвер гасить
 * причину як `ssr & ~(SCIF_DR | SCxSR_RDxF(port))`, тобто обидва біти разом.
 * Що цей самий RXI йде в DMAC, у джерелах Linux не записано (каналів SCIF у
 * `hpb_dmae_slaves[]` немає) — це рішення моделі; без нього коротка
 * відповідь micom назавжди лишалася б у FIFO, чого на живій платі не буває.
 */
static void qy8_scif_rx_pump(Qy8Scif *s)
{
    unsigned taken = 0;

    if (s->dmac && (s->fifo_len >= qy8_scif_rtrg(s) || s->dr)) {
        while (taken < s->fifo_len &&
               clarion_hpbdma_feed(s->dmac, s->base + SCIF_SCFRDR,
                                   s->fifo[taken])) {
            taken++;
        }
    }
    if (taken) {
        memmove(s->fifo, s->fifo + taken, s->fifo_len - taken);
        s->fifo_len -= taken;
        if (!s->fifo_len) {
            s->dr = false;
        }
    }
    qy8_scif_update_irq(s);
}

static void qy8_scif_idle_expire(void *opaque)
{
    Qy8Scif *s = opaque;

    if (s->fifo_len) {
        s->dr = true;
        qy8_scif_rx_pump(s);        /* DR теж просить DMA — див. rx_pump */
    }
    /*
     * Посилка скінчилася: те, що канал DMA уже переніс, — усе, що буде.
     * Кажемо йому завершити set, інакше короткий кадр мовчки лежав би в
     * буфері гостя до кінця 128-байтного DTCR (див. clarion_hpbdma_eod).
     */
    if (s->dmac) {
        clarion_hpbdma_eod(s->dmac, s->base + SCIF_SCFRDR);
    }
    if (s->fifo_len) {
        /*
         * Канал DMA зараз не озброєний — запит лишається висіти, як у
         * залізі, і байти чекають у FIFO. Перевіряємо ще раз згодом:
         * інакше про них ніхто б не згадав до наступного прийнятого байта.
         */
        timer_mod(s->idle, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                           QY8_SCIF_IDLE_NS);
    }
}

static uint64_t qy8_scif_read(void *opaque, hwaddr addr, unsigned size)
{
    Qy8Scif *s = opaque;

    switch (addr) {
    case SCIF_SCSMR:
        return s->scsmr;
    case SCIF_SCSCR:
        return s->scscr;
    case SCIF_SCFSR:
        /* передавач завжди готовий; приймач — за станом FIFO */
        return SCFSR_TDFE | SCFSR_TEND |
               (qy8_scif_rdf(s) ? SCFSR_RDF : 0) | (s->dr ? SCFSR_DR : 0);
    case SCIF_SCFRDR: {
        uint8_t v = s->fifo_len ? s->fifo[0] : 0;

        if (s->fifo_len) {
            memmove(s->fifo, s->fifo + 1, --s->fifo_len);
        }
        if (!s->fifo_len) {
            s->dr = false;
        }
        qy8_scif_update_irq(s);
        return v;
    }
    case SCIF_SCFCR:
        return s->scfcr;
    case SCIF_SCFDR:
        /* старший байт — заповненість TX FIFO (0), молодший — RX */
        return s->fifo_len;
    case SCIF_SCSPTR:
        return 0;
    case SCIF_SCLSR:
        return 0;
    default:
        return 0;
    }
}

static void qy8_scif_write(void *opaque, hwaddr addr, uint64_t val,
                           unsigned size)
{
    Qy8Scif *s = opaque;
    uint8_t ch;

    switch (addr) {
    case SCIF_SCSMR:
        s->scsmr = val;
        break;
    case SCIF_SCSCR:
        s->scscr = val;
        qy8_scif_update_irq(s);
        break;
    case SCIF_SCFTDR:
        ch = val & 0xff;
        /*
         * Якщо до цього SCIF під'єднано супутній МК, байт іде йому — це той
         * самий дріт, а не додатковий канал. Відповіді micom лягають у FIFO
         * приймача через qy8_micom_sink() нижче, тобто тим самим шляхом, що
         * й байти від -serial: DMA і прапорець DR працюють як є.
         */
        if (s->micom) {
            clarion_micom_rx_byte(s->micom, ch);
            break;
        }
        if (s->dispmicom) {
            clarion_dispmicom_rx_byte(s->dispmicom, ch);
            break;
        }
        if (s->ublox) {
            clarion_ublox_rx_byte(s->ublox, ch);
            break;
        }
        /* синхронний вивід: без нього ранні рядки буту губляться */
        qemu_chr_fe_write_all(&s->chr, &ch, 1);
        break;
    case SCIF_SCFCR:
        s->scfcr = val;
        if (val & SCFCR_RFRST) {            /* скидання FIFO прийому */
            s->fifo_len = 0;
            s->dr = false;
        }
        qy8_scif_update_irq(s);
        break;
    case SCIF_SCFSR:
        /* прапорці скидаються записом нуля у відповідний біт */
        if (!(val & SCFSR_DR)) {
            s->dr = false;
        }
        qy8_scif_update_irq(s);
        break;
    default:
        break;
    }
}

static const MemoryRegionOps qy8_scif_ops = {
    .read = qy8_scif_read,
    .write = qy8_scif_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static int qy8_scif_can_receive(void *opaque)
{
    Qy8Scif *s = opaque;
    return QY8_SCIF_FIFO - s->fifo_len;
}

static void qy8_scif_receive(void *opaque, const uint8_t *buf, int size)
{
    Qy8Scif *s = opaque;

    /*
     * Байти лягають у FIFO приймача, як у залізі. DMA забирає їх лише коли
     * назбирався рівень запуску RTRG (гість ставить 14) — саме тому коротка
     * відповідь від micom лишається у FIFO, і про неї повідомляє DR.
     */
    int i;

    for (i = 0; i < size && s->fifo_len < QY8_SCIF_FIFO; i++) {
        s->fifo[s->fifo_len++] = buf[i];
    }
    s->dr = false;
    qy8_scif_rx_pump(s);
    timer_mod(s->idle, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                       QY8_SCIF_IDLE_NS);
}

/* --- HSCIF0 (високошвидкісний SCIF) @0xFFE48000 ----------------------- */

/*
 * Окремий блок, а не сьомий SCIF. У r8a7778.dtsi це власний вузол:
 *
 *     hscif0: serial@ffe48000 {
 *             compatible = "renesas,hscif-r8a7778",
 *                          "renesas,rcar-gen1-hscif", "renesas,hscif";
 *             reg = <0xffe48000 96>;
 *             interrupts = <GIC_SPI 118 IRQ_TYPE_LEVEL_HIGH>;
 *     };
 *
 * тобто база поза вікном SCIF0..SCIF5 (0xFFE40000..0xFFE45FFF), розмір 0x60
 * і власна лінія. Донедавна ці 96 байтів потрапляли у широкий qy8.periph
 * (prio −1000), який на читання віддає нулі.
 *
 * Навіщо це знадобилося. При ввімкненому вікні extension DDR ядро піднімає
 * ДРУГИЙ екземпляр serial_scif.dll із базою PA 0xFFE48000. Його
 * HWSetCommState @VA 0xefa48158 починає з очікування кінця передачі:
 *
 *     0xefa48190  ldr  r0, [r5, #0x10]   ; VA SCFSR
 *     0xefa48194  bl   READ_REGISTER_USHORT
 *     0xefa48198  uxth r3, r0
 *     0xefa4819c  tst  r3, #0x40         ; TEND
 *     0xefa481a0  bne  0xefa481a8        ; вихід — далі SetBaudRate тощо
 *     0xefa481a4  b    0xefa48190        ; інакше назад, без Sleep
 *
 * Це spin без виходу з планувальника, а потік має пріоритет 50. Нуль від
 * qy8.periph означає TEND = 0 назавжди: 11 754 863 читання 0xFFE48010 за
 * 30 с, менеджер пристроїв голодує, ActivateDeviceEx("Drivers\BuiltIn") не
 * повертається (docs/04-journal.md, запис від 2026-09-23).
 *
 * Регістрова мапа — SCIx_HSCIF_REGTYPE з drivers/tty/serial/sh-sci.c:
 * SCSMR 0x00/16, SCBRR 0x04/8, SCSCR 0x08/16, SCxTDR 0x0c/8, SCxSR 0x10/16,
 * SCxRDR 0x14/8, SCFCR 0x18/16, SCFDR 0x1c/16, SCSPTR 0x20/16, SCLSR 0x24/16,
 * SCDL 0x30/16, SCCKS 0x34/16, HSSRR 0x40/16, HSRTRGR 0x54/16, HSTTRGR 0x58/16;
 * там же fifosize = 128 (у SCIF — 16).
 *
 * ⚠ TEND і TDFE тут НЕ сталі: вони рахуються з наповненості передавального
 * FIFO. Після скидання FIFO порожній, тож TEND = 1 одразу — саме цього
 * драйверові й бракувало. Але щойно гість покладе байти у SCFTDR, TEND
 * зійде, доки модель їх не віддасть. Так зупинка на TEND лишається
 * спостережуваною, а не замаскованою константою 0x40.
 *
 * Виміряно на живому буті (увесь бут — 55 звернень замість 11 754 864):
 *
 *     r 0x20                 ; SCSPTR — одноразовий зонд, як і раніше
 *     r 0x10                 ; SCFSR: TEND = 1 -> spin виходить з ПЕРШОГО читання
 *     w 0x08 = 0             ; SCSCR: вимкнути TE/RE
 *     w 0x18 = 0x6           ; SCFCR: RFRST|TFRST
 *     r/w 0x10 = 0           ; погасити прапорці
 *     r/w 0x24 = 0           ; SCLSR: погасити ORER
 *     w 0x40 = 0x800f        ; HSSRR = SRE | SRCYC — суто HSCIF-ний регістр
 *     w 0x34 = 0             ; SCCKS
 *     w 0x30 = 0x30          ; SCDL — подільник
 *     w 0x50 = 0x64          ; поза мапою sh-sci.c (див. нижче)
 *     w 0x54 = 0x40          ; HSRTRGR = 64 — рівно sci_port rx_trigger HSCIF
 *     w 0x58 = 0             ; HSTTRGR = 0 — рівень передавача
 *     w 0x08 = 0x2e, 0x3e    ; TE|RE|REIE|CKE1; ні RIE, ні TIE
 *     w 0x0c ×6              ; 10 02 23 10 03 30 — один кадр у лінію
 *
 * Звідси три речі, які модель НЕ вгадує, а бере з цього заміру:
 *  - HSSRR @0x40 із SRE — підтверджує, що це справді HSCIF, а не сьомий SCIF;
 *  - HSTTRGR = 0, тобто TDFE = «FIFO передавача порожній» — саме так його й
 *    рахує qy8_hscif_tdfe();
 *  - гість НЕ вмикає ні RIE, ні TIE, тож відсутність переривання передавача
 *    тут нічого не ламає.
 *
 * ⚠ offset 0x50 (значення 0x64 = 100) у мапі sh-sci.c немає; гість пише його
 * один раз і ніколи не читає. Тому він свідомо лишається НЕзмодельованим і
 * падає в LOG_UNIMP під іменем qy8.hscif0 — вигадувати йому семантику не
 * було б на чому. Якщо колись знадобиться — це єдине місце, куди дивитися.
 *
 * Чого тут НЕМАЄ і чому:
 *  - каналу DMA: HPB-DMAC обслуговує SCIF4 (лінк до micom); жодного доказу,
 *    що цей порт озброює DMA, немає, а вигадувати slave-id не можна;
 *  - переривання передавача: як і в моделі SCIF, лінію піднімає тільки
 *    приймач і тільки при RIE (див. застереження про шторм вище);
 *  - будь-яких ненульових reset-значень: усі регістри-сховища стартують з 0,
 *    а все, що поза мапою, іде в LOG_UNIMP під власним іменем — щоб
 *    наступний блокер було видно поіменно, а не серед шуму qy8.periph.
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

/* SCFCR: скидання FIFO передавача/приймача (та сама мапа, що в SCIF) */
#define SCFCR_TFRST     0x0004

#define QY8_HSCIF_FIFO  128             /* sh-sci.c: fifosize = 128 */

/*
 * Скільки віртуального часу «летить» один байт. Швидкість вивести нізвідки:
 * гість програмує подільник уже після цього очікування, а SCBRR при
 * такій самій послідовності на SCIF отримує 0. Тому, як і з
 * QY8_SCIF_IDLE_NS, береться стала: важлива не її точність, а те, що
 * передавач має ненульовий час зайнятості й TEND випливає зі стану.
 */
#define QY8_HSCIF_TX_NS     10000

typedef struct Qy8Hscif {
    MemoryRegion mr;
    CharFrontend chr;
    qemu_irq irq;
    QEMUTimer *tx;              /* злив передавального FIFO */
    QEMUTimer *idle;            /* пауза в лінії -> DR, як у SCIF */
    uint16_t scsmr, scscr, scfcr, scsptr, scdl, sccks, hssrr;
    uint16_t hsrtrgr, hsttrgr, sclsr;
    uint8_t scbrr;
    uint8_t rx[QY8_HSCIF_FIFO];
    unsigned rx_len;
    unsigned tx_len;            /* байти в передавальному FIFO */
    bool dr;
} Qy8Hscif;

/* RTRG[7:6] у SCFCR; для HSCIF рівень може приходити і з HSRTRGR */
static unsigned qy8_hscif_rtrg(Qy8Hscif *h)
{
    static const unsigned lvl[4] = { 1, 4, 8, 14 };

    /* sh-sci.c бере рівень приймача з HSRTRGR, коли той запрограмований */
    if (h->hsrtrgr) {
        return MIN(h->hsrtrgr, QY8_HSCIF_FIFO);
    }
    return lvl[(h->scfcr >> 6) & 3];
}

static bool qy8_hscif_rdf(Qy8Hscif *h)
{
    return h->rx_len >= qy8_hscif_rtrg(h);
}

/* TEND — передавач порожній: ні у FIFO, ні «в дроті» нічого немає */
static bool qy8_hscif_tend(Qy8Hscif *h)
{
    return h->tx_len == 0;
}

/*
 * TDFE — «у передавальному FIFO даних не більше за рівень запуску».
 * Рівень беремо з HSTTRGR; після скидання він 0, тобто TDFE = FIFO порожній.
 * Це строгіше за залізо (там рівень програмований), зате не вимагає
 * вигаданого reset-значення й ніколи не дозволить запис у повний FIFO.
 */
static bool qy8_hscif_tdfe(Qy8Hscif *h)
{
    return h->tx_len <= MIN(h->hsttrgr, QY8_HSCIF_FIFO - 1);
}

static void qy8_hscif_update_irq(Qy8Hscif *h)
{
    /* Тільки приймач і тільки при RIE — див. застереження в моделі SCIF. */
    qemu_set_irq(h->irq, (qy8_hscif_rdf(h) || h->dr) && (h->scscr & SCSCR_RIE));
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
        return (qy8_hscif_tdfe(h) ? SCFSR_TDFE : 0) |
               (qy8_hscif_tend(h) ? SCFSR_TEND : 0) |
               (qy8_hscif_rdf(h) ? SCFSR_RDF : 0) |
               (h->dr ? SCFSR_DR : 0);
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
        /* старший байт — заповненість FIFO передавача, молодший — приймача */
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
        qemu_log_mask(LOG_UNIMP, "qy8.hscif0: читання поза мапою %#" HWADDR_PRIx
                      " (%u Б)\n", addr, size);
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
         * Вивід синхронний — як у моделі SCIF; у FIFO лишається тільки
         * лічильник зайнятості, з якого й випливають TEND/TDFE.
         */
        qemu_chr_fe_write_all(&h->chr, &ch, 1);
        break;
    case HSCIF_SCFCR:
        h->scfcr = val;
        if (val & SCFCR_RFRST) {
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
        /* прапорці гасяться записом нуля у відповідний біт */
        if (!(val & SCFSR_DR)) {
            h->dr = false;
        }
        qy8_hscif_update_irq(h);
        break;
    case HSCIF_SCLSR:
        h->sclsr &= val;        /* ORER гаситься записом нуля */
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
        qemu_log_mask(LOG_UNIMP, "qy8.hscif0: запис поза мапою %#" HWADDR_PRIx
                      " = %#" PRIx64 " (%u Б)\n", addr, val, size);
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
 * Куди МК (обидва — плати на SCIF4 і панелі на SCIF1) кладуть свої байти.
 * Повертаємо, скільки прийнято.
 *
 * Кадр мусить лягти в приймач ОДНІЄЮ посилкою. Наївне «скільки влізло у
 * 16-байтовий FIFO, решту наступного разу» розриває будь-який довший кадр, бо
 * між шматками встигає спрацювати таймер тиші SCIF (QY8_SCIF_IDLE_NS = 200
 * мкс, а МК доливає через *_FILL_NS = 1 мс). А `qy8_scif_idle_expire()` — це
 * не просто «підняти DR»: він ще й закриває набір DMA через
 * `clarion_hpbdma_eod()`. Тобто гість отримує не один кадр, а кілька
 * 16-байтових огризків, кожен як окрему завершену посилку.
 *
 * На живому лінку такого немає: байти йдуть безперервним потоком, а DMA
 * вигрібає FIFO на ходу. У моделі вигрібання миттєве
 * (qy8_scif_receive -> qy8_scif_rx_pump), тому просто доливаємо, доки є
 * місце — віртуальний час усередині циклу не рухається, таймер тиші не
 * спрацьовує, і кадр лишається однією посилкою.
 *
 * Симптоми, з яких це знайдено: МК панелі — кадр 0x24 (20 Б) розривався на
 * два недокадри й гість відповідав NAK; МК плати — кадр «набір команд»
 * `08 0a` (~70 Б) не доходив до розбирача EdaDrv взагалі, лінк назавжди
 * лишався у стані 2, ACM-ID 61 ніколи не виставлявся (docs/24 у
 * nissan-can-explore).
 */
static int qy8_micom_burst(Qy8Scif *s, const uint8_t *buf, int len)
{
    int done = 0;

    while (done < len) {
        int room = qy8_scif_can_receive(s);
        int n;

        if (room <= 0) {
            break;                  /* DMA не озброєний — решту доллє МК */
        }
        n = MIN(room, len - done);
        qy8_scif_receive(s, buf + done, n);
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

/* --- TMU (таймери Renesas, регістрова мапа як у SH TMU) --------------- */

#define QY8_TMU_BASE        0xFFD80000
#define QY8_TMU_CHANS       3
#define QY8_TMU_SPI0        32          /* канал n -> SPI 32+n -> IRQ 64+n;
                                           перевірено: eboot вмикає IRQ 65
                                           одночасно зі стартом каналу 1 */
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
            /* UNF скидається записом нуля в цей біт */
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

/* --- контролер плати (зовнішня мікросхема на CS, 16-бітні регістри) ---- */

/*
 * Це не блок SoC, а окремий чип на шині CS; у прошивці він — джерело DIPSW
 * і сигналів дозволу старту. Що саме з нього читають, знято з коду:
 *
 *   +0x00  статус. Завантажувач у циклі @0x1148 читає його двічі поспіль
 *          (антидребезг зовнішньої шини) і дивиться біт 0x400 — «можна
 *          виходити зі standby»; без нього лічильник r7 не спадає до нуля
 *          і машина йде на скидання по watchdog @0x11e0. Біт 0x80 керує
 *          другим лічильником (r6), за яким @0x1a6c у +0x04 пишеться
 *          0x8000 або 0.
 *   +0x02  DIPSW. Молодші 3 біти: перша стадія @0x12c4 порівнює з 4
 *          (режим оновлення з флеш-офсета 0x60000), а eboot друкує
 *          "DIPSW=%x" @0x97c0686c і обирає за ними режим буту.
 *   +0x04, +0x06, +0x08, +0x0c, +0x14 — пишуться під час ініціалізації.
 */
typedef struct Qy8Bctl {
    MemoryRegion mr;
    uint16_t reg[QY8_BCTL_SIZE / 2];
    SDBus *sd[QY8_NUM_SDHI];    /* шини SDHI — джерело card-detect */
} Qy8Bctl;

/*
 * Card-detect не зберігається в reg[]: його рахуємо на кожному читанні з
 * фактичної топології QEMU. `sdbus_get_inserted()` віддає true лише коли на
 * шині є пристрій-картка І в нього вставлений блочний backend (hw/sd/core.c:
 * `get_card()` повертає NULL для порожньої шини; hw/sd/sd.c:
 * `sd_get_inserted()` = `blk && blk_is_inserted(blk)`). Тобто це саме
 * «картка під'єднана», а не «контролер існує»: обидва SDHI в моделі є завжди,
 * картку в слот вставляє лише `-drive if=sd,index=N`.
 */
static uint16_t qy8_bctl_card_detect(Qy8Bctl *b)
{
    static const uint16_t cd[QY8_NUM_SDHI] = { BCTL_CD0, BCTL_CD1 };
    uint16_t bits = 0;
    int i;

    for (i = 0; i < QY8_NUM_SDHI; i++) {
        if (!b->sd[i] || !sdbus_get_inserted(b->sd[i])) {
            bits |= cd[i];          /* порожньо -> 1 (активно-низька лінія) */
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
     * +0x00 при читанні — статус, при записі — щось інше: перша стадія
     * @0x1a6c кладе туди 0x2000 і одразу після того читає той самий
     * регістр, очікуючи цілі біти 0x80/0x400. Якщо дати запису затерти
     * читану половину, наступний тік нагляду @0x97c10e84 бачить 0x400
     * скинутим, ставить стан 1, а ще через тік іде на скидання по
     * watchdog @0x97c10d38 ("_p"). Тобто плани читання й запису тут
     * різні — запис у статус просто не чіпає те, що читається.
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



/* --- простір, який ні в що не декодується ----------------------------- */

/*
 * QEMU за замовчуванням відповідає на доступ до нічийної фізичної адреси
 * зовнішнім аварійним завершенням (data/prefetch abort). Ця плата так не
 * поводиться, і доказ тому — сама прошивка: ядро WinCE ЗОНДУЄ пам'ять.
 *
 * Процедура @0x880489ec отримує (початок, довжина, робоча VA), відображає
 * сторінку-кандидат у вікно 0xC04B0000 і двійковим пошуком шукає верхню межу
 * ОЗП: зберігає два слова, пише підпис 0x6a08bc95/0xfd1247e3, читає назад і
 * за збігом рухає нижню межу вгору, а за розбіжністю — верхню вниз.
 *
 * Перший виклик іде по справжньому ОЗП (PA 0x0d6f0000, 0x9e0000 — це
 * ulRAMStart..ulRAMEnd із ROMHDR ядра) і проходить. Другий питає про
 * можливий додатковий банк PA 0x44000000 розміром 64 МБ — адреси, якої
 * немає в жодному відображенні самого ядра (його таблиця L1: флеш @0,
 * DDR @0x08000000 256 МБ, вікна 0x18000000, 0xf0000000, 0xfc000000).
 * Тобто зондування нічийного простору тут — штатний хід, і читання звідти
 * мусить просто повернути сміття, інакше алгоритм узагалі не міг би
 * працювати. У нас же воно давало abort -> ядро падало у вектор @0xffff0010.
 *
 * Тому нижнім шаром кладемо «порожнечу»: читання дає нулі, запис гине.
 * Пріоритет нижчий за все інше, зокрема за qy8.periph (-1000), тож жоден
 * справжній регістр вона не перехоплює, а доступи все одно видно з -d unimp.
 */
static uint64_t qy8_void_read(void *opaque, hwaddr addr, unsigned size)
{
    qemu_log_mask(LOG_UNIMP, "qy8.void: читання з нічийної адреси %#" HWADDR_PRIx
                  " (%u Б)\n", addr, size);
    return 0;
}

static void qy8_void_write(void *opaque, hwaddr addr, uint64_t val,
                           unsigned size)
{
    qemu_log_mask(LOG_UNIMP, "qy8.void: запис у нічийну адресу %#" HWADDR_PRIx
                  " = %#" PRIx64 "\n", addr, val);
}

static const MemoryRegionOps qy8_void_ops = {
    .read = qy8_void_read,
    .write = qy8_void_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
};

/* --- USB-PHY (R-Car Gen1) @0xFFE70800 --------------------------------- */


/*
 * Перший блок, на якому спіткнулося саме ЯДРО, а не завантажувач.
 *
 * Регістрова мапа взята з Linux (`drivers/usb/phy/phy-rcar-usb.c`,
 * сумісність "renesas,usb-phy-r8a7778"), а не вгадана; там же вузол
 * usb-phy@ffe70800 має другим вікном 0xffe76000.
 *
 *   +0x00 USBPCTRL0
 *   +0x04 USBPCTRL1 — біт0 PHY_ENB, біт1 PLL_ENB, біт2 PHY_RST
 *   +0x08 USBST     — тільки читання: біт31 ST_ACT, біт30 ST_PLL
 *
 * Прошивка робить рівно те, що й драйвер Linux: пише в USBPCTRL1 спершу
 * 1 (PHY_ENB), потім 3 (PHY_ENB|PLL_ENB) і чекає, поки в USBST стануть
 * обидва біти — «PLL захопився». Поки регістр читався нулем, ядро
 * крутилося в цьому опитуванні вічно (5,3 млн читань за 20 с у `-d unimp`).
 */
#define QY8_USBPHY_BASE     0xFFE70800
#define QY8_USBPHY_SIZE     0x100

#define USBPCTRL1           0x04
#define USBST               0x08

#define USBPCTRL1_PHY_ENB   (1u << 0)
#define USBPCTRL1_PLL_ENB   (1u << 1)
#define USBST_ACT           (1u << 31)
#define USBST_PLL           (1u << 30)

typedef struct Qy8UsbPhy {
    MemoryRegion mr;
    uint32_t reg[QY8_USBPHY_SIZE / 4];
} Qy8UsbPhy;

static uint64_t qy8_usbphy_read(void *opaque, hwaddr addr, unsigned size)
{
    Qy8UsbPhy *u = opaque;
    uint32_t ctrl1 = u->reg[USBPCTRL1 / 4];

    if (addr == USBST) {
        /* PLL «захоплюється» миттєво, щойно ввімкнено PHY і PLL */
        if ((ctrl1 & (USBPCTRL1_PHY_ENB | USBPCTRL1_PLL_ENB)) ==
            (USBPCTRL1_PHY_ENB | USBPCTRL1_PLL_ENB)) {
            return USBST_ACT | USBST_PLL;
        }
        return 0;
    }
    return u->reg[addr / 4];
}

static void qy8_usbphy_write(void *opaque, hwaddr addr, uint64_t val,
                             unsigned size)
{
    Qy8UsbPhy *u = opaque;

    if (addr == USBST) {          /* статус — тільки читання */
        return;
    }
    u->reg[addr / 4] = val;
}

static const MemoryRegionOps qy8_usbphy_ops = {
    .read = qy8_usbphy_read,
    .write = qy8_usbphy_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

/* --- USB host: EHCI @0xFFE70000, OHCI @0xFFE70400 --------------------- */

/*
 * Bases are the ones MQUSBH prints and the OAL's base->IRQ table uses
 * (nk.exe ZE0 @0x8801275c, ZE1 @0x88013028): EHCI -> IRQ 165, OHCI -> 164.
 * Both share GIC ID 76 (SPI 44); the OAL demux reads INTC2 word 0xFE782058,
 * bit 1 for EHCI and bit 0 for OHCI (ZE0 @0x88011734, ZE1 @0x88011e14).
 * Vendor registers inside the EHCI window (EIIBC1/2 at +0x94/+0x9c, written
 * by the OAL) fall through to qy8.periph.
 */
#define QY8_EHCI_BASE       0xFFE70000
#define QY8_OHCI_BASE       0xFFE70400
#define QY8_USB_SPI         44
#define QY8_INT2_USB_OHCI   (1u << 0)
#define QY8_INT2_USB_EHCI   (1u << 1)

/* --- GPIO (R-Car, 6 банків по 0x1000) --------------------------------- */


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
    uint32_t in_level;          /* рівні на вхідних лініях */
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
        /* виходи читаються як те, що ми туди записали; входи — рівень лінії */
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
 * Рівні на вхідних лініях плати.
 *
 * GPIO1 біт 0 — сигнал «лишатися в standby». Цикл @0x1148 читає INDT двічі
 * поспіль і, якщо біт стоїть, іде на 0x1188: там r8:=1, і коли лічильник
 * r7 добігає нуля, @0x120c перевіряє r8 і заводить watchdog @0x11e0 на
 * скидання. Тобто одиниця на цьому піні — це не «плата готова», а «не
 * вантажитися»; щоб бут пішов далі, лінія має бути в нулі. Той самий пін
 * @0x1bfc отримує GPIO1.INTCLR біт 0, тобто він ще й джерело переривання.
 */
static const uint32_t qy8_gpio_in_level[QY8_GPIO_BANKS] = {
    [1] = 0x00000000,
};


/* --- qy8.periph: широкий перехоплювач із контекстом викликача ---------- */

/*
 * Те саме, що `unimplemented-device`, але з відповіддю на питання «хто це
 * робить». Поведінка для гостя НЕ змінюється: читання так само повертає 0,
 * запис так само нікуди не йде. Додається лише рядок журналу.
 *
 * QY8_UNIMP_PC=N у середовищі вмикає розширений формат для перших N
 * доступів до кожної адреси:
 *
 *   qy8.periph: unimplemented device write (size 4, offset 0x0100730,
 *       value 0x00000000) pc=0x88011a2c lr=0xefd8b118 mode=0x13 t=61234
 *
 * `pc` точний: він береться з даних розгортання блока трансляції
 * (`cpu_unwind_state_data`), інакше в env лежав би PC початку блока.
 * `t` — гостьовий час у мс, той самий годинник, що друкує консоль ядра,
 * тож рядок можна покласти поруч із етапом буту.
 *
 * Без змінної середовища формат рядка байт-у-байт такий самий, як у
 * стандартного unimplemented-device, щоб наявні розбирачі логів працювали.
 */
#define QY8_PERIPH_SEEN 2048        /* хеш-таблиця «offset -> скільки разів» */

#define QY8_PERIPH_BASE 0xF0000000
#define QY8_PERIPH_RANGES 8         /* скільки діапазонів «трасувати завжди» */

typedef struct Qy8Periph {
    MemoryRegion mr;
    unsigned trace_pc;          /* скільки перших разів на адресу показувати */
    hwaddr seen_off[QY8_PERIPH_SEEN];
    unsigned seen_cnt[QY8_PERIPH_SEEN];
    /* QY8_UNIMP_PC_ALL: діапазони ФІЗИЧНИХ адрес без обмеження на кількість */
    unsigned nranges;
    hwaddr rlo[QY8_PERIPH_RANGES], rhi[QY8_PERIPH_RANGES];
} Qy8Periph;

/*
 * Розгортання стану процесора коштує дорого: якщо робити його на кожному
 * доступі, гість сповільнюється приблизно вдесятеро (виміряно A/B: 5910
 * рядків логу за 45 с проти 687). А потрібне воно лише щоб назвати
 * викликача, тобто перші кілька разів на кожну адресу. Далі адреса вже
 * відома й рядок пишеться звичайним швидким шляхом.
 */
static bool qy8_periph_want_ctx(Qy8Periph *p, hwaddr addr)
{
    unsigned i = (unsigned)((addr >> 2) * 2654435761u) % QY8_PERIPH_SEEN;
    unsigned probe;

    for (probe = 0; probe < p->nranges; probe++) {
        hwaddr pa = QY8_PERIPH_BASE + addr;

        if (pa >= p->rlo[probe] && pa < p->rhi[probe]) {
            return true;        /* цей блок зараз вивчаємо — без обмежень */
        }
    }
    if (!p->trace_pc) {
        return false;
    }
    for (probe = 0; probe < 8; probe++) {
        unsigned k = (i + probe) % QY8_PERIPH_SEEN;

        if (p->seen_cnt[k] && p->seen_off[k] != addr) {
            continue;           /* колізія — далі по таблиці */
        }
        p->seen_off[k] = addr;
        p->seen_cnt[k]++;
        return p->seen_cnt[k] <= p->trace_pc;
    }
    return false;               /* таблиця переповнена: краще швидко, ніж ніяк */
}

static void qy8_periph_ctx(char *buf, size_t len)
{
    CPUState *cs = current_cpu;
    CPUARMState *env;

    uint64_t data[4];    /* TARGET_INSN_START_WORDS для ARM = 3 */
    uint32_t pc;

    buf[0] = '\0';
    if (!cs) {
        return;
    }
    env = cpu_env(cs);
    /*
     * Точний PC інструкції, що робить доступ. Саме cpu_unwind_state_data,
     * а НЕ cpu_restore_state: той пише розгорнутий стан назад у env, і
     * блок трансляції продовжує виконуватися з підміненими регістрами —
     * перевірено, гість після цього йде іншою гілкою (A/B розходиться на
     * ~680-му рядку логу). Ця ж функція лише читає дані розгортання.
     */
    if (cpu_unwind_state_data(cs, cs->mem_io_pc, data)) {
        /*
         * З CF_PCREL (типово для системної емуляції) data[0] — це зсув
         * усередині сторінки, а не повний PC; старші біти беремо з env,
         * бо блок трансляції ніколи не перетинає межу сторінки.
         */
        pc = (uint32_t)data[0];
        if (pc < qemu_target_page_size()) {
            pc |= (uint32_t)env->regs[15] & (uint32_t)qemu_target_page_mask();
        }
    } else {
        pc = (uint32_t)env->regs[15];   /* запасний варіант: початок блока */
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
 * Контролер DDR3. Перша стадія буту (reset-stage, PC 0x17e0) записує
 * конфігурацію DDR-фізичного рівня; одне зі слів — 0x0f030a02 у регістр
 * DBPDCNT3 (offset 0x24). Пізніше OAL (PC 0x88011f04) перечитує ту саму
 * адресу й очікує збережене значення. Без цього регістра запис ішов у
 * широкий qy8.periph (-1000), який його відкидав.
 *
 * Решта регістрів поки не потрібна — логуємо через LOG_UNIMP.
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

/* --- CPG: засувки MSTPCR (Module Stop Control) ------------------------ */

/*
 * MSTPCR0/1/3/4/5/6 — керування подачею такту на модулі: біт 1 = «модуль
 * зупинено», 0 = «тактується». Імена й адреси НЕ вгадані: вони є в таблиці
 * імен регістрів самого завантажувача (nand @0x24010) і збігаються з
 * mstp0..5_clks@ffc800xx у r8a7778.dtsi.
 *
 * Навіщо модель. І завантажувач, і OAL роблять із ними read-modify-write:
 *
 *     v = MSTPCRn;  v &= ~mask;  MSTPCRn = v;   // пустити модуль
 *     if (MSTPCRn & mask) fail;                 // перевірка
 *     v = MSTPCRn;  v |=  mask;  MSTPCRn = v;   // зупинити модуль
 *     if ((MSTPCRn & mask) != mask) fail;       // перевірка
 *
 * Доки ці адреси падали в широкий qy8.periph, запис губився, а читання
 * давало 0. Тому RMW між завантажувачем і WinCE був розірваний, а перевірка
 * після «зупинити» завжди провалювалася.
 *
 * Модель — рівно засувка: що записали, те й читається. Тактування ми не
 * моделюємо, тож жодних інших наслідків у неї немає.
 *
 * ⚠️ Reset-значення MSTPCR нам НЕ відоме (в доступних джерелах його немає),
 * тому модель стартує з нулів. Це свідомо НЕ відтворення power-on стану
 * SoC — мета лише в тому, щоб зберігалися значення, які пише сам гість.
 * Якщо reset-значення колись знайдеться, змінити треба саме цей масив.
 *
 * Статусні регістри MSTPSR1/4/6 (0xFFC80044/48/4C) навмисно НЕ чіпаємо:
 * вони й далі читаються нулем через qy8.periph, і саме нуль означає
 * «модуль працює», якого OAL і чекає після «пустити».
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

/* --- INTC2: статусні слова демукса ------------------------------------ */

/*
 * Три слова в INTC2, кожне на 4 байти: 0xFE782048 (DU), 0xFE7820F4 (SDHI0),
 * 0xFE7820F8 (SDHI1).
 *
 * Навіщо. Жодна з цих ліній GIC не веде до драйвера напряму:
 * OEMInterruptHandler (nk.exe @0x8800b5a0) має для кожної власний case, який
 * спершу маскує лінію в GICD_ICENABLER, а потім кличе демультиплексор. Той
 * читає статусне слово, і лише ненульове значення дає логічний IRQ:
 *
 *   GIC ID 63  (SPI 31, DU)    case 0x8800b6d8 -> демукс 0x88011e94:
 *       v = MMIO32(0xFE782048);
 *       if (v & 0x0000FFFF) return OALIntrTranslateIrq(63);   // -> SYSINTR 37
 *       if (v & 0xFFFF0000) return OALIntrTranslateIrq(167);
 *       return SYSINTR_NOP;
 *
 *   GIC ID 119 (SPI 87, SDHI0) case 0x8800bb9c -> демукс 0x8800da90:
 *       if (MMIO32(0xFE700008)) { MMIO32(0xFE782280) = 0x100;
 *                                 return OALIntrTranslateIrq(119); }
 *       if (MMIO32(0xFE7820F4) & 0xF) return OALIntrTranslateIrq(119);
 *       return SYSINTR_NOP;                                   // -> SYSINTR 20
 *
 *   GIC ID 120 (SPI 88, SDHI1) case 0x8800bbc4 -> демукс 0x8800db68:
 *       те саме зі словом 0xFE7820F8, ACK 0x200 і IRQ 120. // -> SYSINTR 23
 *
 * Без цих слів читається нуль, демукс повертає SYSINTR_NOP, ядро не кличе
 * InterruptDone — і лінія лишається замаскованою назавжди. Саме так
 * поводилася модель до 24.09.2026 (див. docs/04-journal.md).
 *
 * Прив'язка «база пристрою -> логічний IRQ» знята з OALIntrRequestIrqs
 * (nk.exe @0x88012b34): 0xFFE4C000 -> 0x77 (119) @0x88012fc0,
 * 0xFFE4D000 -> 0x78 (120) @0x88012fe8, 0xFFE4F000 -> 0x76 (118).
 * Далі — дамп живих таблиць OAL: g_oalIrq2SysIntr[119] = 20,
 * g_oalIrq2SysIntr[120] = 23, g_oalIrq2SysIntr[63] = 37.
 *
 * Що НЕ доведено: котрий саме біт кожного слова належить пристрою. OAL
 * перевіряє половину слова цілком (DU) або біти 0..3 (SDHI), тож модель ставить
 * біт 0. Це не «підганяння, щоб гість пішов далі»: слово віддзеркалює
 * РЕАЛЬНИЙ стан лінії, а не константу.
 *
 * Гілку 0xFE700008 свідомо лишено нулем: цього блоку ми не моделюємо, а
 * друга гілка демукса дає той самий логічний IRQ без запису куди-небудь.
 *
 * Слова тільки на читання: у всьому nk.exe їх ніхто не пише.
 */
#define QY8_INT2_STATUS_DU      0xFE782048
#define QY8_INT2_STATUS_SDHI0   0xFE7820F4
#define QY8_INT2_STATUS_SDHI1   0xFE7820F8
#define QY8_INT2_STATUS_USB     0xFE782058
#define QY8_INT2_STATUS_SHCORE 0xFE78208C
#define QY8_INT2_BIT            (1u << 0)   /* біт у слові — не доведений */
#define QY8_INT2_SHCORE_BIT     0x10000    /* підтверджено для IRQ 86 */

typedef struct Qy8Int2Line {
    MemoryRegion mr;
    const char *name;
    hwaddr pa;
    uint32_t pending;           /* біти джерел, що тримають лінію GIC */
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

    qemu_log_mask(LOG_UNIMP, "qy8.int2: запис у статусний регістр %s %#"
                  HWADDR_PRIx " = %#" PRIx64 " (ігнорується)\n",
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

/* --- машина ----------------------------------------------------------- */

#define TYPE_QY8_MACHINE MACHINE_TYPE_NAME("clarion-qy8")
OBJECT_DECLARE_SIMPLE_TYPE(Qy8MachineState, QY8_MACHINE)

struct Qy8MachineState {
    MachineState parent;

    ARMCPU *cpu;
    DeviceState *gic;
    DeviceState *du;
    DeviceState *dmac;
    DeviceState *lbdma;      /* DMA читання NOR @0xFF801000 */
    Qy8Scif scif[QY8_NUM_SCIF];
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
    Qy8UsbPhy usbphy;
    Qy8Dbsc3 dbsc3;
    Qy8Int2Line int2_du;
    Qy8Int2Line int2_sdhi[QY8_NUM_SDHI];
    Qy8Int2Line int2_usb;
    Qy8Int2Line int2_shcore;
    Qy8Mstp mstp[QY8_NUM_MSTP];
    Qy8Periph periph;
    MemoryRegion voidmr;

    uint8_t dipsw;              /* режим буту, властивість машини */
    uint32_t du_spi;            /* лінія GIC для DU, властивість машини */
    uint32_t du_dotclk;         /* точкова частота DU, Гц; 0 = без такту */
    bool micom_on;              /* вбудований супутній МК на SCIF4 */
    bool dispmicom_on;          /* вбудований МК панелі на SCIF1 */
    bool gps_on;
    char *gps_lat;
    char *gps_lon;
    char *gps_speed;
    char *gps_course;
    bool i2c4_on;               /* opt-in bounded R-Car I2C4 model */
    bool i2c4_recorder_on;      /* opt-in I2C4 transaction recorder */
    bool tma460_on;             /* opt-in bounded TMA460 model */
    bool tma460_synthetic_profile_on; /* opt-in synthetic profile */
    bool i2c_empty_on;          /* opt-in: I2C0..I2C2 з порожньою шиною */
    char *board;                /* property value: "auto", a model, an alias */
    const struct Qy8BoardInfo *board_info; /* resolved in qy8_init() */
    bool reverse;               /* RV input, machine property */
    bool g2d_on;
    bool shcore_on;
    char *g2d_log;
    char *render;
    char *render_lib;
    char *render_log;
    char *render_dump_dir;

    MemoryRegion flash;          /* лише коли флеш подано як ROM */
    DriveInfo *flash_drive;      /* -drive if=pflash: записувана копія */
    MemoryRegion ddr, ddr0, ddr1;
    MemoryRegion sram0, sram1;
    MemoryRegion ddr1_shadow;
};

/*
 * Лінія DU: одночасно GIC SPI 31 і біт у статусному слові INTC2. OAL читає
 * це слово в демультиплексорі (див. qy8_int2_read), тож без нього підняття
 * самої лінії GIC нічого не дає.
 */
static void qy8_du_irq(void *opaque, int n, int level)
{
    Qy8MachineState *s = opaque;

    s->int2_du.pending = level ? QY8_INT2_BIT : 0;
    qemu_set_irq(qdev_get_gpio_in(s->gic, s->du_spi), level);
}

/*
 * Лінії SDHI: так само, як у DU — одночасно GIC SPI 87/88 і біт у власному
 * статусному слові INTC2, без якого демукс OAL поверне SYSINTR_NOP.
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
typedef struct Qy8BoardInfo {
    const char *model;          /* as stored in the PROD block */
    const char *name;           /* canonical board= value */
    const char *alias;          /* deprecated board= value */
    bool tma616;                /* touch controller: TMA616, not TMA460 */
} Qy8BoardInfo;

/* One row per supported unit model */
static const Qy8BoardInfo qy8_boards[] = {
    { "QY8652NB", "qy8652nb", "ze1", false },
    { "QY8202NA", "qy8202na", "ze0", true },
};

static const hwaddr qy8_prod_offsets[] = { 0x40000, 0xa000 };

#define QY8_PROD_MODEL_OFF  0x40
#define QY8_PROD_MODEL_LEN  8

static const Qy8BoardInfo *qy8_board_by_name(const char *name)
{
    for (int i = 0; i < ARRAY_SIZE(qy8_boards); i++) {
        if (!g_strcmp0(name, qy8_boards[i].name) ||
            !g_strcmp0(name, qy8_boards[i].alias)) {
            return &qy8_boards[i];
        }
    }
    return NULL;
}

static const Qy8BoardInfo *qy8_board_by_model(const char *model)
{
    for (int i = 0; i < ARRAY_SIZE(qy8_boards); i++) {
        if (!strcmp(model, qy8_boards[i].model)) {
            return &qy8_boards[i];
        }
    }
    return NULL;
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
                            char model[QY8_PROD_MODEL_LEN + 1],
                            uint8_t first_bytes[4])
{
    for (int i = 0; i < ARRAY_SIZE(qy8_prod_offsets); i++) {
        hwaddr off = qy8_prod_offsets[i];
        uint8_t sig[4] = { 0 };

        if (!qy8_flash_peek(machine, off, sig, sizeof(sig))) {
            continue;
        }
        if (i == 0) {
            memcpy(first_bytes, sig, sizeof(sig));
        }
        if (memcmp(sig, "PROD", 4) ||
            !qy8_flash_peek(machine, off + QY8_PROD_MODEL_OFF,
                            (uint8_t *)model, QY8_PROD_MODEL_LEN)) {
            continue;
        }
        for (int j = 0; j < QY8_PROD_MODEL_LEN; j++) {
            if (!g_ascii_isgraph(model[j])) {
                model[j] = '.';
            }
        }
        model[QY8_PROD_MODEL_LEN] = '\0';
        *prod_offset = off;
        return true;
    }
    return false;
}

static void qy8_resolve_board(Qy8MachineState *s, MachineState *machine)
{
    static const char hint[] = "choose one with board=qy8652nb|qy8202na";
    const Qy8BoardInfo *detected = NULL;
    char model[QY8_PROD_MODEL_LEN + 1] = "";
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
                     qy8_prod_offsets[0], hint);
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
}

/* QY8202NA, the unit albertbm brought up as "ze0": TMA616 touch controller */
static bool qy8_is_ze0(Qy8MachineState *s)
{
    return s->board_info && s->board_info->tma616;
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
    /* скидання починається з 0x0 — флеш CS0, XIP */
    object_property_set_int(OBJECT(s->cpu), "rvbar", 0, NULL);
    qdev_realize(DEVICE(s->cpu), NULL, &error_fatal);

    /* --- пам'ять --- */

    /* найнижчий шар: нічийні адреси читаються нулями, а не дають abort */
    memory_region_init_io(&s->voidmr, NULL, &qy8_void_ops, s,
                          "qy8.void", 0x100000000ULL);
    memory_region_add_subregion_overlap(sysmem, 0, &s->voidmr, -1500);

    /*
     * Флеш CS0 подається гостю двома способами.
     *
     * Без `-drive if=pflash` — як було: ROM, куди `-bios` кладе байти дампа.
     * Гостьові записи туди нікуди не йдуть, ID флеш не читаються, стирання
     * неможливе; для читального трасування цього досить.
     *
     * З `-drive if=pflash,format=raw,file=КОПІЯ.bin` — справжня модель
     * мікросхеми (`cfi.pflash02`): гість бачить ID, може стерти сектор і
     * записати слово, а зміни лягають у ФАЙЛ КОПІЇ. Вихідний дамп при цьому
     * не потрібен і не чіпається.
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
         * Адреси розблокування. Завантажувач пише за БАЙТОВИМИ 0xAAAA і
         * 0x5554; модель для x16 ділить на 2 і лишає 11 біт, тобто 0x555 і
         * 0x2AA — рівно типові значення набору команд AMD.
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
     * Тінь DDR1 за PA 0x90000000 — милиця під конвеєр, а не деталь плати.
     *
     * `Launch` завантажувача (VA 0x97c1204c) вимикає MMU і ОДРАЗУ наступною
     * інструкцією стрибає на фізичну адресу трампліна:
     *
     *      97c12118  mcr p15,0,r1,c1,c0,0   ; SCTLR.M := 0 — MMU вимкнено
     *      97c1211c  mov pc, r0             ; r0 = VAtoPA(0x97c12130) = 0x17c12130
     *      97c12120  nop x4                 ; набивка під злив конвеєра
     *
     * На живому Cortex-A9 `mov pc,r0` уже вибрано в конвеєр, поки MMU був
     * увімкнений, тож вибірка за старим відображенням; ARM ARM і не обіцяє,
     * що зміна SCTLR.M побачиться до context-synchronizing operation. QEMU ж
     * застосовує запис миттєво: обриває блок трансляції й перевибирає
     * 0x97c1211c уже з вимкненим MMU, тобто за PA 0x97C1211C. Там у нас нічого
     * немає -> prefetch abort -> PC=0xc -> «нульовий слайд» по таблиці векторів
     * до ASCII-імені @0x28 -> data abort -> вічна петля. Саме це й ловив
     * `info registers` (abt32, PC=0x10).
     *
     * Тому робимо ті кілька слів досяжними за фізичною адресою, рівною їхній
     * VA: DDR1 (PA 0x10000000, там живе eboot) ще раз видно за 0x90000000 —
     * рівно так, як його ж OEMAddressTable ставить VA 0x90000000 -> PA
     * 0x10000000. Це вікно потрібне тільки на час «MMU вже вимкнено, а PC ще
     * віртуальний»; далі виконання йде за справжніми PA (0x17c12130, потім
     * ядро @0x08001000).
     */
    memory_region_init_alias(&s->ddr1_shadow, NULL, "qy8.ddr1-shadow",
                             &s->ddr1, 0, QY8_DDR1_SIZE);
    memory_region_add_subregion(sysmem, 0x90000000, &s->ddr1_shadow);

    /* --- вміст флеш: дамп плати через -bios --- */
    if (s->flash_drive) {
        /*
         * Вміст уже прийшов із блочного бекенда `-drive if=pflash`, і саме
         * туди підуть гостьові стирання й записи. `-bios` тут зайвий і
         * небезпечний: він мовчки перекрив би копію байтами іншого файлу.
         */
        if (machine->firmware) {
            error_report("clarion-qy8: -bios і -drive if=pflash разом не "
                         "можна — вміст флеш береться з копії");
            exit(1);
        }
    } else {
        if (!machine->firmware) {
            error_report("clarion-qy8: треба -bios <дамп флеш 64 МБ> "
                         "або -drive if=pflash,format=raw,file=<копія>");
            exit(1);
        }
        fname = g_strdup(machine->firmware);
        sz = load_image_mr(fname, &s->flash);
        if (sz < 0) {
            error_report("clarion-qy8: не вдалося прочитати %s", fname);
            exit(1);
        }
        /*
         * Годиться будь-який дамп CS0 — стоковий, чужий чи власноруч
         * патчений: машина нічого з нього не розбирає, вона просто кладе
         * байти під reset-вектор. Розмір має бути рівно 64 МБ (дамп без OOB),
         * інакше решта вікна лишиться нулями і бут піде не туди.
         */
        if (sz != QY8_FLASH_SIZE) {
            warn_report("clarion-qy8: %s має %d байт, а не 64 МБ — "
                        "решта флеш-вікна лишиться нулями", fname, (int)sz);
        }
        g_free(fname);
    }

    /* --- GIC (в A9 R-Car Gen1 він окремий, не в private region) --- */
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
        sysbus_realize_and_unref(SYS_BUS_DEVICE(s->shcore), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(s->shcore), 0, QY8_SHCORE_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(s->shcore), 0,
                           qemu_allocate_irq(qy8_shcore_irq, s, 0));
    }

    /*
     * --- I2C0..I2C2: та сама обмежена модель контролера, шина ПОРОЖНЯ ---
     *
     * Діагностичний режим (docs/31): без моделі клієнти цих контролерів
     * (бібліотека тюнера, Usb.exe) на кожній транзакції чекають переривання
     * 3000 мс. Порожня шина дає чесний для неї результат — NACK адреси
     * (MNR) — одразу. На справжній платі пристрої там є, тож це не
     * поведінка заліза, а швидка помилка замість повільної; типово вимкнено.
     * GIC ID за таблицею OEMInterruptHandler: 99, 110, 108 (SPI = ID - 32).
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

    /* --- I2C4: bounded T142/T143 models; other controllers stay unmodeled --- */
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
        if (s->tma460_on && qy8_is_ze0(s)) {
            /*
             * The ZE0 board has a TMA616 in the same place: application
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
     * Бази й переривання — з hpb_dmae_resources[] у setup-r8a7778.c:
     * канальні регістри 0xffc08000/0x1000, спільні 0xffc09000/0x170,
     * gic_iid(0x7b) і 5 ліній поспіль (IRQ 123..127 = SPI 91..95).
     * Ставимо до SCIF, бо приймачі SCIF віддають байти саме сюди.
     */
    s->dmac = qdev_new(TYPE_CLARION_HPBDMA);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(s->dmac), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(s->dmac), 0, CLARION_HPBDMA_CHAN_BASE);
    sysbus_mmio_map(SYS_BUS_DEVICE(s->dmac), 1, CLARION_HPBDMA_COMM_BASE);
    for (i = 0; i < CLARION_HPBDMA_NUM_IRQ; i++) {
        sysbus_connect_irq(SYS_BUS_DEVICE(s->dmac), i,
                           qdev_get_gpio_in(s->gic,
                                            CLARION_HPBDMA_IRQ_BASE_SPI + i));
    }

    /*
     * --- Супутній МК (micom) на SCIF4 ---
     *
     * На платі це окрема мікросхема, тож типово вона є. Без неї ядро WinCE
     * не проходить ініціалізацію лінку й бут не доходить до GWES. Вимкнути —
     * `-M clarion-qy8,micom=off`: тоді SCIF4 поводиться як звичайний порт і
     * до нього можна причепити зовнішній відповідач через -serial
     * (tools/qy8_micom.py), щоб гратися з нерозібраними командами.
     */
    if (s->micom_on) {
        s->micom = qdev_new(TYPE_CLARION_MICOM);
        qdev_realize_and_unref(s->micom, NULL, &error_fatal);
    }

    /*
     * --- МК панелі дисплея (Display Micom) на SCIF1 ---
     *
     * Окрема мікросхема в блоці дисплея, теж розпаяна на живому авто, тож
     * типово вона є. Без неї `lcddrv.dll` відкриває `SCI1:`, шле свій
     * 10 02 23 10 03 30 і назавжди стоїть у WaitCommEvent: ACM-модуль 129
     * ніколи не виставляється, і DrawOi малює перший кадр аж після
     * 5-секундного таймауту очікування на 129. Вимкнути —
     * `-M clarion-qy8,dispmicom=off`: тоді SCIF1 поводиться як звичайний
     * порт і до нього можна причепити власний відповідач через -serial.
     */
    if (s->dispmicom_on) {
        s->dispmicom = qdev_new(TYPE_CLARION_DISPMICOM);
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
        Qy8Scif *sc = &s->scif[i];
        char *name = g_strdup_printf("qy8.scif%d", i);

        sc->index = i;
        /* QY8_SCIF3_TXI=0 повертає стару поведінку (A/B) */
        sc->txi = i == QY8_SCIF_DEBUG &&
                  g_strcmp0(getenv("QY8_SCIF3_TXI"), "0") != 0;
        sc->base = QY8_SCIF_BASE + i * QY8_SCIF_STRIDE;
        sc->idle = timer_new_ns(QEMU_CLOCK_VIRTUAL, qy8_scif_idle_expire, sc);
        sc->dmac = s->dmac;
        sc->irq = qdev_get_gpio_in(s->gic, QY8_SCIF_SPI0 + i);
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
        memory_region_init_io(&sc->mr, NULL, &qy8_scif_ops, sc, name, 0x100);
        memory_region_add_subregion(sysmem,
                                    QY8_SCIF_BASE + i * QY8_SCIF_STRIDE,
                                    &sc->mr);
        if (serial_hd(qy8_scif_chr_index(i))) {
            qemu_chr_fe_init(&sc->chr, serial_hd(qy8_scif_chr_index(i)),
                             &error_abort);
            qemu_chr_fe_set_handlers(&sc->chr, qy8_scif_can_receive,
                                     qy8_scif_receive, NULL, NULL,
                                     sc, NULL, true);
        }
        g_free(name);
    }

    /* --- HSCIF0 --- */
    /*
     * Вузьке вікно 0x60 з пріоритетом 1: перекриває широкий qy8.periph
     * (-1000), але нічого іншого не зачіпає — між SCIF5 (0xFFE45FFF) і
     * HSCIF0 лежать три незайняті кілобайти. Сьомий -serial (індекс 6,
     * після шести SCIF) чіпляється сюди, якщо його задали; без нього порт
     * просто не має співрозмовника, як непідключений роз'єм на платі.
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

    /* --- контролер плати: DIPSW і дозвіл виходу зі standby --- */
    s->bctl.reg[BCTL_STATUS >> 1] = BCTL_ST_PWR | BCTL_ST_BOOT;
    qy8_bctl_set_inputs(s);
    s->bctl.reg[BCTL_DIPSW >> 1] = s->dipsw & 7;
    memory_region_init_io(&s->bctl.mr, NULL, &qy8_bctl_ops, &s->bctl,
                          "qy8.bctl", QY8_BCTL_SIZE);
    /* перекриває вікно sram1, бо на платі це той самий CS */
    memory_region_add_subregion_overlap(sysmem, QY8_BCTL_BASE,
                                        &s->bctl.mr, 1);

    /* USB-PHY: перекриває широке вікно qy8.periph (див. нижче) */
    memory_region_init_io(&s->usbphy.mr, NULL, &qy8_usbphy_ops, &s->usbphy,
                          "qy8.usbphy", QY8_USBPHY_SIZE);
    memory_region_add_subregion_overlap(sysmem, QY8_USBPHY_BASE,
                                        &s->usbphy.mr, 2);

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

    /* --- Display Unit: справжнє вікно QEMU --- */
    /*
     * Лінія переривання GIC_SPI 31. Кадровий період модель бере з регістрів
     * HCR/VCR, які програмує сам гість; точкову частоту — з властивості
     * `dotclk` (ESCR02 = 0, такт зовнішній, у регістрах його немає).
     */
    s->du = qdev_new(TYPE_CLARION_DU);
    qdev_set_id(s->du, g_strdup("qy8-du"), &error_fatal);
    qdev_prop_set_uint32(s->du, "dotclk", s->du_dotclk);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(s->du), &error_fatal);
    if (s->tma460 && qy8_is_ze0(s)) {
        clarion_tma616_bind_pointer_input(s->tma460, "qy8-du", &error_fatal);
    } else if (s->tma460 && s->tma460_synthetic_profile_on) {
        clarion_tma460_bind_pointer_input(s->tma460, "qy8-du", &error_fatal);
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(s->du), 0, QY8_DU_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(s->du), 0,
                       qemu_allocate_irq(qy8_du_irq, s, 0));

    /*
     * Статусні слова демукса INTC2: пріоритет 1, щоб перекрити нічийний
     * простір (qy8.void, -1500), який досі віддавав звідти нуль.
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
     * CPG MSTPCR: шість окремих чотирибайтових вікон із пріоритетом 1 над
     * широким qy8.periph (-1000). Саме окремі вікна, а не одне на весь блок:
     * так усе інше в CPG (FRQCR, MSTPSR) лишається як було й далі видно в
     * `-d unimp`.
     */
    for (int i = 0; i < QY8_NUM_MSTP; i++) {
        char nm[32];

        s->mstp[i].val = 0;         /* reset-значення невідоме — див. коментар */
        s->mstp[i].pa = qy8_mstp_regs[i].pa;
        s->mstp[i].name = qy8_mstp_regs[i].name;
        snprintf(nm, sizeof(nm), "qy8.cpg.%s", qy8_mstp_regs[i].name);
        memory_region_init_io(&s->mstp[i].mr, NULL, &qy8_mstp_ops,
                              &s->mstp[i], nm, 4);
        memory_region_add_subregion_overlap(sysmem, qy8_mstp_regs[i].pa,
                                            &s->mstp[i].mr, 1);
    }

    /*
     * --- SDHI0/SDHI1: контролери SD ---
     *
     * Обидва існують завжди, навіть коли жодного образу не подано: на платі
     * це мікросхема SoC, а не картка. Порожній слот поводиться як порожній —
     * команда не отримує відповіді й контролер виставляє CMDTIMEOUT.
     *
     * Картку в слот вставляє `-drive if=sd,index=N,format=raw,file=...`
     * (N = 0 для переднього слота, 1 для другого) або коротке `-sd файл`.
     */
    for (i = 0; i < QY8_NUM_SDHI; i++) {
        DriveInfo *di = drive_get(IF_SD, 0, i);

        s->sdhi[i] = qdev_new(TYPE_RENESAS_SDHI);
        qdev_prop_set_uint8(s->sdhi[i], "unit", i);
        /*
         * Передача даних іде через HPB-DMAC: контролер каже йому «в буфері
         * є блок», а читає DMAC сам, за адресою власного SD_BUF0 цього
         * контролера (прив'язку каналу до неї тримає hpb_chan_module[]).
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
         * Чип плати читає card-detect із цих самих шин (див. qy8_bctl_read).
         * Прив'язку робимо тут, а не при створенні bctl, бо контролери
         * створюються пізніше; гість читає регістр уже після init машини.
         */
        s->bctl.sd[i] = SD_BUS(qdev_get_child_bus(s->sdhi[i], "sd-bus"));
    }

    /*
     * --- LBSC DMAC @0xFF801000 ---------------------------------------
     *
     * Двигун, яким `Flash.dll` реально читає паралельну NOR: IOCTL
     * `0x01112020` (`FlashReadToPhysMem`) не шле мікросхемі жодної команди
     * CFI, а програмує сюди {SAR, DAR, TCR} і дає START. Без моделі запити
     * ковтав широкий `qy8.periph`, призначення лишалося нульовим, а драйвер
     * 5 секунд чекав переривання завершення, яке нізвідки взятися не могло.
     *
     * Лінію GIC узято не з аналогії: OAL самої прошивки віддає драйверу
     * IRQ 0x6F для `LogicalLoc = 0xFF801000`, а «логічний IRQ» OAL — це GIC
     * INTID, тож SPI = 111 - 32 = 79 (докладно — clarion_lbdma.h).
     */
    s->lbdma = qdev_new(TYPE_CLARION_LBDMA);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(s->lbdma), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(s->lbdma), 0, CLARION_LBDMA_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(s->lbdma), 0,
                       qdev_get_gpio_in(s->gic, CLARION_LBDMA_SPI));

    /*
     * --- CAN @0xFFFD1000 ---------------------------------------------
     *
     * Контролер плати, до якого ходить `CAN.dll` (а через `CAN1:` —
     * `AntiTheft.exe`). Модель знає тільки про власні режими контролера;
     * шини авто (BCM, метр, HVAC) за нею немає — див. renesas_can.c.
     * Лінію переривання не під'єднуємо: номер GIC SPI для CAN нічим не
     * доведено, а модель поки нічого й не піднімає.
     */
    /*
     * QY8_CAN=off прибирає модель зовсім: блок знову падає в qy8.periph і
     * `CAN.dll` знову впирається в таймаут. Це потрібно лише для порівняння
     * «до/після», тому вимикач env, а не властивість машини.
     */
    if (g_strcmp0(getenv("QY8_CAN"), "off") != 0) {
        s->can = qdev_new(TYPE_RENESAS_CAN);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(s->can), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(s->can), 0, QY8_CAN_BASE);
    }

    /*
     * --- PowerVR SGX @0xFCE00000 -------------------------------------
     *
     * Щабель M1 карти docs/28-sgx-roadmap.md: блок отримує власне вікно й
     * прилад замість широкого перехоплювача. Поведінково для гостя це НІЩО —
     * читання так само віддають нулі, переривань немає, `ui32InitStatus`
     * модель не торкається, тож `SGXInitialise` доходить до того самого
     * таймауту. Уся користь у тому, що на `EUR_CR_EVENT_KICK2` модель друкує
     * розбір видимого стану: регістри-носії адрес, повний обхід каталогу
     * сторінок SGX і вміст об'єкта за коренем `PDS_EXEC_BASE + рег 0x0A68`.
     * Чому саме так і чому M3 поки не роблять — clarion_sgx.c і
     * docs/sgx/09-edm-boot-locator.md.
     */
    if (g_strcmp0(getenv("QY8_SGX"), "off") != 0) {
        s->sgx = qdev_new(TYPE_CLARION_SGX);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(s->sgx), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(s->sgx), 0, CLARION_SGX_BASE);
    }

    /* --- решта периферії: поки лише лог доступів (-d unimp) --- */
    create_unimplemented_device("qy8.cs1",   QY8_CS1_BASE, QY8_CS1_SIZE);
    create_unimplemented_device("qy8.l2c",   QY8_L2C_BASE, 0x1000);
    /* --- DBSC3: DDR3 controller, пріоритет вищий за qy8.periph (-1000) --- */
    s->dbsc3.dbpdcnt3 = 0;      /* reset: offset 0x24 читається нулем */
    memory_region_init_io(&s->dbsc3.mr, NULL, &qy8_dbsc3_ops, &s->dbsc3,
                          "qy8.dbsc3", QY8_DBSC3_SIZE);
    memory_region_add_subregion_overlap(sysmem, QY8_DBSC3_BASE,
                                        &s->dbsc3.mr, 1);
    create_unimplemented_device("qy8.intc2", 0xFE780000, 0x1000);
    create_unimplemented_device("qy8.cpg",   0xFFC80000, 0x1000);
    create_unimplemented_device("qy8.pfc",   0xFFFC0000, 0x1000);
    create_unimplemented_device("qy8.rst",   0xFFCC0000, 0x1000);
    /* широкий перехоплювач: усе інше згори 0xF0000000 логується */
    /*
     * Широкий перехоплювач: усе інше згори 0xF0000000 логується. Власна
     * модель замість `unimplemented-device` — лише щоб QY8_UNIMP_PC=1
     * додавало pc/lr/час викликача; гість бачить рівно те саме.
     */
    /*
     * QY8_UNIMP_PC=N — показувати pc/lr/час для перших N доступів до КОЖНОЇ
     * адреси (порожнє або 1 = один раз). Далі рядок звичайний, тож ціна
     * розгортання стану платиться один раз на адресу, а не на кожен доступ.
     */
    {
        const char *e = getenv("QY8_UNIMP_PC");
        const char *r = getenv("QY8_UNIMP_PC_ALL");

        s->periph.trace_pc = e ? (atoi(e) > 0 ? atoi(e) : 1) : 0;
        /*
         * QY8_UNIMP_PC_ALL="0xffe80000-0xffe81000,0xfff18000-0xfff19000" —
         * для цих ФІЗИЧНИХ діапазонів контекст пишеться на кожному доступі,
         * скільки б їх не було. Так вивчають один конкретний блок, не
         * платячи за розгортання стану на гарячих адресах решти периферії.
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
        error_setg(errp, "board must be auto, qy8652nb or qy8202na");
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

static bool qy8_shcore_get(Object *obj, Error **errp)
{
    return QY8_MACHINE(obj)->shcore_on;
}

static void qy8_shcore_set(Object *obj, bool value, Error **errp)
{
    QY8_MACHINE(obj)->shcore_on = value;
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
     * За замовчуванням 5 = "NORM(RES)>>" — звичайний бут із повним
     * налагоджувальним виводом; на живій платі це те саме, що заземлити
     * TEST_B1 (docs/16). Стокове значення непаяної плати — 7 (тихий бут).
     */
    s->du_spi = QY8_DU_SPI;
    object_property_add_uint32_ptr(obj, "du-spi", &s->du_spi,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_set_description(obj, "du-spi",
        "номер лінії GIC (SPI) для кадрового переривання DU");

    /*
     * Точкова частота DU. У регістрах її немає (ESCR02 = 0 — такт зовнішній,
     * від TCON), виміряної теж немає, тож типово 0 = кадрового такту немає.
     * 33333333 — опорний EXTAL плат R-Car M1A, правдоподібне, але НЕ
     * доведене значення; вмикати свідомо.
     */
    s->du_dotclk = 0;
    object_property_add_uint32_ptr(obj, "du-dotclk", &s->du_dotclk,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_set_description(obj, "du-dotclk",
        "точкова частота DU в Гц (0 = кадровий такт вимкнено)");

    object_property_add_bool(obj, "reverse", qy8_reverse_get,
                             qy8_reverse_set);
    object_property_set_description(obj, "reverse",
        "reverse gear input (RV), off by default");

    s->board = g_strdup("auto");
    object_property_add_str(obj, "board", qy8_board_get, qy8_board_set);
    object_property_set_description(obj, "board",
        "board peripherals by unit model: auto (default, read from the "
        "flash image), qy8652nb, qy8202na; ze1 and ze0 are deprecated "
        "aliases of the last two");

    s->dipsw = QY8_DIPSW_NORM_RES;
    object_property_add_uint8_ptr(obj, "dipsw", &s->dipsw,
                                  OBJ_PROP_FLAG_READWRITE);
    object_property_set_description(obj, "dipsw",
        "режим буту з DIPSW контролера плати, 0..7 (5 = NORM(RES) з логом)");

    /*
     * Супутній МК плати. Типово ввімкнений: на залізі він розпаяний, і без
     * відповіді від нього ядро не проходить ініціалізацію лінку.
     */
    s->micom_on = true;
    object_property_add_bool(obj, "micom", qy8_micom_get, qy8_micom_set);
    object_property_set_description(obj, "micom",
        "вбудований супутній МК на SCIF4 (off — щоб причепити свій "
        "відповідач через -serial)");

    /*
     * МК панелі дисплея. Теж типово ввімкнений: на залізі він розпаяний, і
     * без нього lcddrv не виставляє ACM-модуль 129.
     */
    s->dispmicom_on = true;
    object_property_add_bool(obj, "dispmicom", qy8_dispmicom_get,
                             qy8_dispmicom_set);
    object_property_set_description(obj, "dispmicom",
        "вбудований МК панелі дисплея на SCIF1 (off — щоб причепити свій "
        "відповідач через -serial)");

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
        "opt-in transaction recorder на I2C4 address 0x24");

    s->i2c_empty_on = false;
    object_property_add_bool(obj, "i2c-empty", qy8_i2c_empty_get,
                             qy8_i2c_empty_set);
    object_property_set_description(obj, "i2c-empty",
        "opt-in діагностика: I2C0..I2C2 з порожньою шиною (NACK замість "
        "3-секундного таймауту)");

    s->tma460_on = true;
    object_property_add_bool(obj, "tma460", qy8_tma460_get,
                             qy8_tma460_set);
    object_property_set_description(obj, "tma460",
        "Bounded TMA460 bootloader model at I2C4 address 0x24 (on by default; requires i2c4=on); "
        "TMA616 at 0x67 with board=ze0");

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

    s->shcore_on = true;
    object_property_add_bool(obj, "shcore", qy8_shcore_get, qy8_shcore_set);
    object_property_set_description(obj, "shcore",
        "synthetic SH initialization-end response (on by default)");
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
    /* RAM машина створює сама — за картою з OEMAddressTable */
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
