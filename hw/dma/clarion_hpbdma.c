/*
 * Renesas HPB-DMAC (High Performance Bus DMA controller, R8A7778) —
 * модель для плати Clarion QY8XXX.
 *
 * Блок ототожнено не за схожістю адрес, а за тим, що збіглася ВСЯ
 * регістрова мапа: гість пише сімку регістрів із кроком 0x40, і кожен
 * зсув лягає на drivers/dma/sh/rcar-hpbdma.c один-в-один. Бази й
 * переривання — з arch/arm/mach-shmobile/setup-r8a7778.c
 * (hpb_dmae_resources[]), тобто з платформенного файлу САМЕ нашого SoC:
 *
 *   канальні регістри 0xffc08000/0x1000, спільні 0xffc09000/0x170,
 *   async reset 0xffc00300, async mode 0xffc00400,
 *   переривання gic_iid(0x7b) — 5 ліній, IRQ 123..127.
 *
 * Навіщо це ядру WinCE. Прошивка піднімає два канали на SCIF4:
 *
 *   канал 4: DSAR0/DSAR1 = 0xffe44014 (SCIF4 SCFRDR), DDAR0/DDAR1 —
 *            два буфери в ОЗП по 0x80 Б, DCR = CT|DIP|SMDL
 *            (безперервний прийом у подвійний буфер);
 *   канал 6: DSAR0 = буфер в ОЗП, DDAR0 = 0xffe4400c (SCIF4 SCFTDR),
 *            DTCR0 = 8, DCR = DMDL (передача).
 *
 * У буфері передачі лежить 10 02 | 02 08 00 | 10 03 | 19 — кадр DLE STX
 * / дані / DLE ETX / сума. Тобто SCIF4 — це лінк до підпорядкованого
 * мікроконтролера плати (у прошивці є cpucom.dll і ціла родина
 * *MicomUpdate.exe), а не налагоджувальна консоль.
 *
 * Модульний бік каналу адресується ПРИВ'ЯЗКОЮ КАНАЛУ (таблиця
 * hpb_chan_module нижче), а не тим, що гість записав у DSAR/DDAR. Раніше
 * модель порівнювала DSAR з адресою периферії; для SCIF це працювало
 * випадково, для SDHI не спрацювало б ніколи. Докладно — у коментарі до
 * таблиці.
 *
 * ⚠ Межа чесності. Напрямок «пам'ять -> модуль» виконується одразу:
 * наш SCIF завжди готовий приймати, тож миттєва передача — це не
 * спрощення, а точний опис цієї моделі. Напрямок «модуль -> пам'ять»
 * НЕ виконується наосліп: канал лише озброюється, і байти в нього
 * подає сам SCIF через clarion_hpbdma_feed() — інакше модель вигадала
 * б дані, яких на шині немає. Поки на SCIF4 ніхто не відповідає,
 * канал прийому чесно стоїть активним і нічого не переносить.
 *
 * Чого свідомо немає (джерела не кажуть — не вигадуємо):
 *   DPTR: прошивка пише туди 4 і 0x400, драйвер Linux його не чіпає —
 *         зберігаємо, не тлумачимо;
 *   DTIMR, DMASPR, DMLVLR, DMSHPT: у драйвері лише названі;
 *   апаратна швидкість передачі — переносимо все за один такт.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/dma/clarion_hpbdma.h"
#include "system/address-spaces.h"
#include "qom/object.h"
#include "trace.h"
#include "hw/core/cpu.h"

/* --- канальні регістри (зсуви з rcar-hpbdma.c) ------------------------ */

#define HPB_DSAR0       0x00
#define HPB_DDAR0       0x04
#define HPB_DTCR0       0x08
#define HPB_DSAR1       0x0C
#define HPB_DDAR1       0x10
#define HPB_DTCR1       0x14
#define HPB_DSASR       0x18
#define HPB_DDASR       0x1C
#define HPB_DTCSR       0x20
#define HPB_DPTR        0x24
#define HPB_DCR         0x28
#define HPB_DCMDR       0x2C
#define HPB_DSTPR       0x30
#define HPB_DSTSR       0x34
#define HPB_DDBGR       0x38
#define HPB_DDBGR2      0x3C
#define HPB_CHAN_STRIDE 0x40

/* DCMDR */
#define DCMDR_BDOUT     (1u << 7)
#define DCMDR_DQSPD     (1u << 6)
#define DCMDR_DQSPC     (1u << 5)
#define DCMDR_DMSPD     (1u << 4)
#define DCMDR_DMSPC     (1u << 3)
#define DCMDR_DQEND     (1u << 2)
#define DCMDR_DNXT      (1u << 1)
#define DCMDR_DMEN      (1u << 0)

/* DSTPR / DSTSR */
#define DSTPR_DMSTP     (1u << 0)
#define DSTSR_NDP1      (1u << 6)
#define DSTSR_NDP0      (1u << 5)
#define DSTSR_DQSTS     (1u << 2)
#define DSTSR_DRSTS     (1u << 1)
#define DSTSR_DMSTS     (1u << 0)

/* DCR (include/linux/platform_data/dma-rcar-hpbdma.h) */
#define DCR_CT          (1u << 18)
#define DCR_DIP         (1u << 16)
#define DCR_SMDL        (1u << 13)
#define DCR_SPDS_MASK   (3u << 8)
#define DCR_SPDS_SHIFT  8
#define DCR_DMDL        (1u << 5)
#define DCR_DPDS_MASK   (3u << 0)
#define DCR_DPDS_SHIFT  0

/* --- спільні регістри ------------------------------------------------- */

#define HPB_DTIMR       0x00
#define HPB_DINTSR0     0x0C
#define HPB_DINTSR1     0x10
#define HPB_DINTCR0     0x14
#define HPB_DINTCR1     0x18
#define HPB_DINTMR0     0x1C
#define HPB_DINTMR1     0x20
#define HPB_DACTSR0     0x24
#define HPB_DACTSR1     0x28
#define HPB_HSRSTR(n)   (0x40 + (n) * 4)

typedef struct ClarionHpbChan {
    uint32_t sar[2], dar[2], tcr[2];    /* дві «площини» (DIP) */
    uint32_t dsasr, ddasr, dtcsr;
    uint32_t dptr, dcr, dstsr;
    uint32_t left;                      /* скільки одиниць лишилося */
    unsigned plane;                     /* яка площина зараз */
    bool next_requested;                /* DNXT: наступна площина поставлена */
    bool active;
} ClarionHpbChan;

/*
 * Модуль, який сам тримає дані і піднімає запит DMA (див. заголовок).
 * Таких на цій платі рівно два — SDHI0 і SDHI1; чотири з запасом.
 */
#define HPB_MAX_MODULE  4

typedef struct ClarionHpbModule {
    hwaddr addr;                        /* регістр даних модуля */
    ClarionHpbModuleReady ready;        /* «дані є просто зараз?» */
    void *opaque;
} ClarionHpbModule;

struct ClarionHpbDmaState {
    SysBusDevice parent_obj;

    MemoryRegion chan_mr;
    MemoryRegion comm_mr;
    qemu_irq irq[CLARION_HPBDMA_NUM_IRQ];

    ClarionHpbChan ch[CLARION_HPBDMA_NUM_CHAN];
    uint32_t dintsr[2];                 /* стан переривань */
    uint32_t dintmr[2];                 /* маски */
    uint32_t dtimr;

    ClarionHpbModule module[HPB_MAX_MODULE];
    unsigned n_module;
    bool servicing;                     /* захист від повторного входу */
};

OBJECT_DECLARE_SIMPLE_TYPE(ClarionHpbDmaState, CLARION_HPBDMA)

/*
 * Канал -> лінія переривання.
 *
 * Таблиця hpb_dmae_channels[] у setup-r8a7778.c дає ch_irq для тих
 * каналів, які використовує Linux: 0x7c (IRQ 124) для 14/15 (USB-функція),
 * 0x7e (IRQ 126) для 21/22 (SDHI0), 0x7f (IRQ 127) для 28..36 (SSI/HPBIF).
 * Каналів 4 і 6 у Linux немає — але їх прив'язує сам гість: він
 * користується каналами 4, 6 (micom) і 21, 22 (SDHI) і вмикає в GIC
 * рівно IRQ 123 та 126. 126 належить SDHI за таблицею вище, отже 123 —
 * це канали 4/6, тобто найнижча з п'яти ліній.
 *
 * Канали 16/17 — HSCIF0 (база 0xFFE48000). Лінію для них теж виведено, а не
 * вгадано, і саме з поведінки гостя:
 *   - у масці DINTMR0 він пропускає рівно канали 4, 16, 17;
 *   - із п'яти ліній DMAC вмикає в GIC рівно IRQ 123, 125, 126;
 *   - 123 за міркуванням вище належить каналам 4/6 (micom), 126 — 21/22 (SDHI);
 *   - отже 125 (лінія 2) лишається єдиною незайнятою, а 16/17 — єдиними
 *     каналами без лінії.
 * Доказ від протилежного знято дослідом: доки 17 віддавав лінію 0, обробник
 * SPI 91 не знаходив «свого» каналу, не писав DINTCR0, і рівень лишався
 * піднятим назавжди — шторм ~4100 IRQ/с, усе інше голодувало (docs/04-journal).
 *
 * Для решти каналів джерела немає; віддаємо лінію 0 і кажемо про це в лог.
 */
static int hpb_chan_irq_line(ClarionHpbDmaState *s, int ch)
{
    if (ch == 14 || ch == 15) {
        return 1;                       /* 0x7c = IRQ 124 */
    }
    if (ch == 21 || ch == 22) {
        return 3;                       /* 0x7e = IRQ 126 */
    }
    if (ch >= 28 && ch <= 36) {
        return 4;                       /* 0x7f = IRQ 127 */
    }
    if (ch == 16 || ch == 17) {
        return 2;                       /* IRQ 125 — HSCIF0, див. вище */
    }
    if (ch != 4 && ch != 6) {
        qemu_log_mask(LOG_UNIMP,
                      "clarion-hpbdma: канал %d — лінії переривання для "
                      "нього немає в жодному джерелі, беремо нижню\n", ch);
    }
    return 0;                           /* gic_iid(0x7b) = IRQ 123 */
}

static void hpb_update_irq(ClarionHpbDmaState *s)
{
    bool raised[CLARION_HPBDMA_NUM_IRQ] = { false };
    int ch, i;

    for (ch = 0; ch < CLARION_HPBDMA_NUM_CHAN; ch++) {
        int w = ch / 32, b = ch % 32;

        if ((s->dintsr[w] & s->dintmr[w]) & (1u << b)) {
            raised[hpb_chan_irq_line(s, ch)] = true;
        }
    }
    for (i = 0; i < CLARION_HPBDMA_NUM_IRQ; i++) {
        qemu_set_irq(s->irq[i], raised[i]);
    }
}

static void hpb_chan_complete(ClarionHpbDmaState *s, int ch)
{
    int w = ch / 32, b = ch % 32;

    s->dintsr[w] |= 1u << b;
    hpb_update_irq(s);
}

/*
 * --- Прив'язка каналу до периферії ------------------------------------
 *
 * У HPB-DMAC периферійний («модульний») бік каналу задає САМЕ КАНАЛ, а не
 * адреса в DSAR/DDAR. Гість оголошує цю прив'язку окремими регістрами —
 * ASYNCRSTR (0xFFC00300) і ASYNCMDR (0xFFC00400), де кожному каналу
 * належить свій біт.
 *
 * Чому модель більше не порівнює DSAR з адресою периферії. Раніше
 * clarion_hpbdma_feed() шукав канал за збігом `dsasr == periph_addr`. Для
 * SCIF це працювало ВИПАДКОВО: EdaDrv кладе туди фізичну адресу регістра.
 * SDHC.dll кладе в DSAR ВІРТУАЛЬНУ адресу SD_BUF0 — і це не його збій:
 * доведено (docs/qy8-sdhi-dma-addr-20260924.txt у репозиторії
 * nissan-can-explore), що механізм VA->PA у драйвера є, працює й
 * застосований до буфера в пам'яті, а на модульний бік не застосовується
 * навмисно. Тож збіг за адресою для SDHI не настав би ніколи.
 *
 * ⚠ Що саме залізо робить із записаним у модульний бік DSAR/DDAR, ми НЕ
 * знаємо (розділу про це в наших джерелах немає). Тому модель нічого про
 * це не припускає: для відомих каналів вона бере адресу регістра з цієї
 * таблиці, а записане гостем значення не тлумачить узагалі.
 *
 * Джерела прив'язки — для кожного рядка своє:
 *
 *   4, 6    SCIF4   — з поведінки самого гостя: DSAR0=0xFFE44014 (SCFRDR)
 *                     на каналі 4 і DDAR0=0xFFE4400C (SCFTDR) на каналі 6;
 *                     розбір лінка — docs/20-qemu-board.md, «M3b».
 *   16, 17  HSCIF0  — так само з гостя: 0xFFE48014 / 0xFFE4800C.
 *   21, 22  SDHI0   — подвійне джерело. setup-r8a7778.c (v4.0):
 *                     SDHI0_TX .dma_ch=21, SDHI0_RX .dma_ch=22, обидва
 *                     .addr = 0xffe4c000 + 0x30. І сам гість: SDHC.dll
 *                     пише ASYNCRSTR біт 1 (ASRST21) та ASYNCMDR
 *                     ASMD21=MULTI — ті самі біти, що Linux позначає
 *                     коментарем "SDHI0".
 *   14, 15  USB-функція — лише з setup-r8a7778.c; ця прошивка їх не чіпає,
 *                     тож рядки не перевірені живим гостем.
 *
 * Напрямок каналом НЕ задається: його визначають DCR.SMDL/DMDL. Гість
 * користується каналом 21 для читання (SMDL), хоча в Linux 21 — це TX.
 */
#define HPB_NO_MODULE   0

static const hwaddr hpb_chan_module[CLARION_HPBDMA_NUM_CHAN] = {
    [4]  = 0xFFE44014,      /* SCIF4  SCFRDR  — з гостя                 */
    [6]  = 0xFFE4400C,      /* SCIF4  SCFTDR  — з гостя                 */
    [14] = 0xFFE60018,      /* USB-функція D0 — Linux, не перевірено    */
    [15] = 0xFFE6001C,      /* USB-функція D1 — Linux, не перевірено    */
    [16] = 0xFFE48014,      /* HSCIF0 HSRDR   — з гостя                 */
    [17] = 0xFFE4800C,      /* HSCIF0 HSTDR   — з гостя                 */
    [21] = 0xFFE4C030,      /* SDHI0  SD_BUF0 — Linux + ASYNCMDR гостя  */
    [22] = 0xFFE4C030,      /* SDHI0  SD_BUF0 — Linux                   */
};

/*
 * Адреса модульного боку каналу. Для невідомого каналу повертає
 * HPB_NO_MODULE: прив'язки ми не знаємо, і вигадувати її не будемо.
 */
static hwaddr hpb_module_addr(int ch)
{
    return hpb_chan_module[ch];
}

/* Розмір одиниці передачі: SPDS/DPDS, 0=8 біт, 1=16, 2=32. */
static unsigned hpb_unit(ClarionHpbDmaState *s, int ch)
{
    uint32_t dcr = s->ch[ch].dcr;
    unsigned spds = (dcr & DCR_SPDS_MASK) >> DCR_SPDS_SHIFT;
    unsigned dpds = (dcr & DCR_DPDS_MASK) >> DCR_DPDS_SHIFT;

    if (spds != dpds) {
        qemu_log_mask(LOG_UNIMP,
                      "clarion-hpbdma: канал %d має різну ширину портів "
                      "(SPDS=%u, DPDS=%u) — беремо ширшу\n", ch, spds, dpds);
    }
    return 1u << MAX(spds > 2 ? 2 : spds, dpds > 2 ? 2 : dpds);
}

/*
 * Перенести одну одиницю. Бік, позначений «MDL», — це модуль (периферія),
 * його адреса не рухається; інший бік інкрементується.
 */
static void hpb_move_one(ClarionHpbDmaState *s, int ch)
{
    ClarionHpbChan *c = &s->ch[ch];
    unsigned unit = hpb_unit(s, ch);
    hwaddr mod = hpb_module_addr(ch);
    hwaddr src = c->dsasr, dst = c->ddasr;
    uint8_t buf[4];

    /*
     * Бік, позначений MDL, адресується прив'язкою каналу. Записане гостем
     * значення для цього боку не використовуємо: воно не зобов'язане бути
     * адресою (див. коментар до hpb_chan_module).
     */
    if ((c->dcr & DCR_SMDL) && mod != HPB_NO_MODULE) {
        src = mod;
    }
    if ((c->dcr & DCR_DMDL) && mod != HPB_NO_MODULE) {
        dst = mod;
    }
    if ((c->dcr & (DCR_SMDL | DCR_DMDL)) && mod == HPB_NO_MODULE) {
        qemu_log_mask(LOG_UNIMP, "clarion-hpbdma: канал %d ходить у модуль, "
                      "але його прив'язки ми не знаємо — беремо адресу з "
                      "регістра, як було\n", ch);
    }

    address_space_read(&address_space_memory, src,
                       MEMTXATTRS_UNSPECIFIED, buf, unit);
    address_space_write(&address_space_memory, dst,
                        MEMTXATTRS_UNSPECIFIED, buf, unit);

    if (!(c->dcr & DCR_SMDL)) {
        c->dsasr += unit;
    }
    if (!(c->dcr & DCR_DMDL)) {
        c->ddasr += unit;
    }
    c->left--;
    c->dtcsr = c->left;
}

static void hpb_chan_load_plane(ClarionHpbDmaState *s, int ch, unsigned plane)
{
    ClarionHpbChan *c = &s->ch[ch];

    c->plane = plane;
    c->dsasr = c->sar[plane];
    c->ddasr = c->dar[plane];
    c->left = c->tcr[plane];
    c->dtcsr = c->left;
}

/*
 * Лічильник добіг. Спільна кінцівка для всіх напрямків: подія переривання,
 * а далі — або наступна площина (CT, якщо software уже подало DNXT), або
 * канал стає неактивним.
 */
static void hpb_chan_finish(ClarionHpbDmaState *s, int ch)
{
    ClarionHpbChan *c = &s->ch[ch];

    hpb_chan_complete(s, ch);
    if (c->dcr & DCR_CT) {
        /*
         * CT не означає безумовне автоперемикання. Наступний set
         * запускається лише якщо software уже подало DNXT; інакше канал
         * лишається active у command-wait state.
         */
        if (c->next_requested) {
            c->next_requested = false;
            hpb_chan_load_plane(s, ch, (c->dcr & DCR_DIP) ? c->plane ^ 1 : 0);
        }
    } else {
        c->active = false;
        c->dstsr &= ~DSTSR_DMSTS;
    }
}

static uint32_t hpb_chan_status(const ClarionHpbChan *c)
{
    uint32_t status = c->dstsr & DSTSR_DQSTS;
    unsigned next_plane = (c->dcr & DCR_DIP) && c->active ? c->plane ^ 1 : 0;

    status |= next_plane ? DSTSR_NDP1 : DSTSR_NDP0;
    if (c->next_requested) {
        status |= DSTSR_DRSTS;
    }
    if (c->active) {
        status |= DSTSR_DMSTS;
    }
    return status;
}

/* ТИМЧАСОВО: зонд «а чи це віртуальна адреса?» під QY8_DMA_VA_PROBE */
static bool hpb_va_probe(void)
{
    static int on = -1;
    if (on < 0) {
        on = getenv("QY8_DMA_VA_PROBE") != NULL;
    }
    return on;
}

/* ТИМЧАСОВО: журнал роботи каналів під QY8_DMA_LOG (знести після досліду) */
static bool hpb_dbg(void)
{
    static int on = -1;
    if (on < 0) {
        on = getenv("QY8_DMA_LOG") != NULL;
    }
    return on;
}

/*
 * ТИМЧАСОВО (QY8_DMA_LOG, знести разом з рештою зондів).
 *
 * Показати, що лягло в ГОСТЬОВУ пам'ять за адресою призначення. Читаємо
 * саме з пам'яті, а не з того, що щойно писали: інакше зонд підтверджував
 * би сам себе. Саме цими рядками доведено перше читання блоку SD
 * (docs/qy8-sdhi-dma-read-20260924.log у репозиторії nissan-can-explore).
 */
static void hpb_dbg_dest(int ch, hwaddr addr, uint32_t dar,
                         uint32_t moved, uint32_t asked)
{
    uint8_t back[16];
    unsigned k;

    if (!hpb_dbg()) {
        return;
    }
    address_space_read(&address_space_memory, dar, MEMTXATTRS_UNSPECIFIED,
                       back, sizeof(back));
    fprintf(stderr, "[dma] ch%d модуль %08x -> %08x: перенесено %u од. з %u, "
            "у пам'яті:", ch, (unsigned)addr, dar, moved, asked);
    for (k = 0; k < sizeof(back); k++) {
        fprintf(stderr, " %02x", back[k]);
    }
    fprintf(stderr, "\n");
}

/* --- модуль, який сам тримає дані (SDHI) ------------------------------ */

static ClarionHpbModule *hpb_module_find(ClarionHpbDmaState *s, hwaddr addr)
{
    unsigned i;

    for (i = 0; i < s->n_module; i++) {
        if (s->module[i].addr == addr) {
            return &s->module[i];
        }
    }
    return NULL;
}

/*
 * Перекачати в пам'ять усе, що модуль за адресою addr готовий віддати
 * просто зараз. Дані беруться ЧИТАННЯМ його регістра (hpb_move_one), тож
 * модель нічого не вигадує: скільки модуль віддасть, стільки й піде.
 *
 * Питання «чи є ще дані» ставиться перед кожною одиницею передачі —
 * модуль може дозаправити свій буфер просто всередині нашого читання
 * (так робить SDHI на багатоблоковому читанні), і цикл це підхопить.
 */
static void hpb_service_module(ClarionHpbDmaState *s, hwaddr addr)
{
    ClarionHpbModule *m = hpb_module_find(s, addr);
    int ch;

    if (!m || s->servicing) {
        return;
    }
    s->servicing = true;

    for (ch = 0; ch < CLARION_HPBDMA_NUM_CHAN; ch++) {
        ClarionHpbChan *c = &s->ch[ch];
        uint32_t dar0, n0;

        /* лише озброєний канал «модуль -> пам'ять», прив'язаний до addr */
        if (!c->active || !c->left || hpb_module_addr(ch) != addr) {
            continue;
        }
        if (!(c->dcr & DCR_SMDL) || (c->dcr & DCR_DMDL)) {
            continue;
        }

        dar0 = c->ddasr;
        n0 = c->left;
        trace_clarion_hpbdma_module_service(ch, (uint32_t)addr, c->ddasr,
                                            c->left);
        while (c->left && m->ready(m->opaque)) {
            hpb_move_one(s, ch);
        }
        if (!c->left) {
            hpb_chan_finish(s, ch);
        }

        hpb_dbg_dest(ch, addr, dar0, n0 - c->left, n0);
        break;                          /* один канал на одну адресу */
    }

    s->servicing = false;
}

void clarion_hpbdma_module_attach(DeviceState *dev, hwaddr periph_addr,
                                  ClarionHpbModuleReady ready, void *opaque)
{
    ClarionHpbDmaState *s = CLARION_HPBDMA(dev);

    if (s->n_module >= HPB_MAX_MODULE) {
        qemu_log_mask(LOG_UNIMP, "clarion-hpbdma: більше ніж %d модулів "
                      "з власним буфером не передбачено\n", HPB_MAX_MODULE);
        return;
    }
    s->module[s->n_module].addr = periph_addr;
    s->module[s->n_module].ready = ready;
    s->module[s->n_module].opaque = opaque;
    s->n_module++;
}

void clarion_hpbdma_module_poke(DeviceState *dev, hwaddr periph_addr)
{
    hpb_service_module(CLARION_HPBDMA(dev), periph_addr);
}

static void hpb_chan_start(ClarionHpbDmaState *s, int ch, bool next)
{
    ClarionHpbChan *c = &s->ch[ch];
    unsigned plane = 0;

    /* DMEN активує idle channel; повторний DMEN не перезапускає transfer. */
    if (c->active) {
        return;
    }
    if ((c->dcr & DCR_DIP) && next) {
        plane = c->plane ^ 1;
    }
    hpb_chan_load_plane(s, ch, plane);
    c->active = true;
    trace_clarion_hpbdma_start(ch, plane, c->dcr, c->dsasr, c->ddasr, c->left);
    if (hpb_dbg() && (ch == 4 || ch == 6)) {
        fprintf(stderr, "[dma] start ch%d plane%u dcr=%08x dar=%08x tcr=%u\n",
                ch, plane, c->dcr, c->ddasr, c->left);
    }

    if ((c->dcr & DCR_SMDL) && !(c->dcr & DCR_DMDL)) {
        /*
         * Модуль -> пам'ять. Самі даних не вигадуємо. Модуль із власним
         * буфером (SDHI) міг набрати їх ще до DMEN — у нього питаємо одразу;
         * решті (SCIF) канал лишається просто озброєним, і байти подасть
         * сама периферія через clarion_hpbdma_feed().
         */
        hpb_service_module(s, hpb_module_addr(ch));
        return;
    }
    if ((c->dcr & DCR_SMDL) && (c->dcr & DCR_DMDL)) {
        qemu_log_mask(LOG_UNIMP, "clarion-hpbdma: канал %d — модуль у модуль, "
                      "джерела на таку передачу немає\n", ch);
        return;
    }

    /* Пам'ять -> модуль (або пам'ять -> пам'ять): наш приймач завжди готовий. */
    while (c->left) {
        hpb_move_one(s, ch);
    }
    c->active = false;
    c->dstsr &= ~DSTSR_DMSTS;
    hpb_chan_complete(s, ch);
}

/*
 * Байт прийшов на периферійний регістр periph_addr. Якщо його чекає
 * озброєний канал — покласти в пам'ять і, коли лічильник добіг,
 * підняти переривання. Повертає true, якщо байт забрав DMA.
 */
bool clarion_hpbdma_feed(DeviceState *dev, hwaddr periph_addr, uint8_t val)
{
    ClarionHpbDmaState *s = CLARION_HPBDMA(dev);
    int ch;

    for (ch = 0; ch < CLARION_HPBDMA_NUM_CHAN; ch++) {
        ClarionHpbChan *c = &s->ch[ch];

        /*
         * Канал шукаємо за ПРИВ'ЯЗКОЮ, а не за тим, що гість записав у
         * DSAR. Для каналу без відомої прив'язки лишається старий шлях —
         * інакше ми б мовчки перестали обслуговувати SCIF-подібні випадки,
         * яких ще не розібрали.
         */
        hwaddr mod = hpb_module_addr(ch);

        if (!c->active || !c->left || !(c->dcr & DCR_SMDL)) {
            continue;
        }
        if (mod != HPB_NO_MODULE ? mod != periph_addr
                                 : c->dsasr != periph_addr) {
            continue;
        }
        address_space_write(&address_space_memory, c->ddasr,
                            MEMTXATTRS_UNSPECIFIED, &val, 1);
        c->ddasr++;
        c->left--;
        c->dtcsr = c->left;

        if (!c->left) {
            if (hpb_dbg()) {
                fprintf(stderr, "[dma] ch%d буфер добіг (plane%u)\n",
                        ch, c->plane);
            }
            hpb_chan_finish(s, ch);
        }
        return true;
    }
    return false;
}

/*
 * Приймач модуля замовк. Завершуємо активний set того каналу, який уже щось
 * у цій посилці переніс: DTCSR лишається з недобраним залишком, і драйвер
 * бачить рівно стільки байтів, скільки прийшло.
 *
 * Чому це потрібно моделі. SCIF віддає прийняте в HPB-DMAC, а не в
 * переривання: гість тримає RIE = 0 на всіх SCIF (перевірено на буті —
 * SCSCR 0x003E/0x00BE), тож RXI не працює як джерело. Драйвер дізнається
 * про дані лише з переривання каналу DMA. Буфер у нього 128 байтів, а
 * кадри лінків короткі (відповідь панелі — 20 байтів), тож без дострокового
 * завершення set ніколи не добігав би до кінця і про кадр ніхто б не
 * дізнався. На залізі цю роль грає сигнал «приймач порожній» (DR) SCIF.
 */
void clarion_hpbdma_eod(DeviceState *dev, hwaddr periph_addr)
{
    ClarionHpbDmaState *s = CLARION_HPBDMA(dev);
    int ch;

    for (ch = 0; ch < CLARION_HPBDMA_NUM_CHAN; ch++) {
        ClarionHpbChan *c = &s->ch[ch];
        hwaddr mod = hpb_module_addr(ch);

        if (!c->active || !(c->dcr & DCR_SMDL)) {
            continue;
        }
        if (mod != HPB_NO_MODULE ? mod != periph_addr
                                 : c->dsasr != periph_addr) {
            continue;
        }
        /* Порожній set завершувати нема чого — переривання без даних. */
        if (!c->left || c->left == c->tcr[c->plane]) {
            continue;
        }
        c->dtcsr = c->left;
        hpb_chan_finish(s, ch);
        return;                         /* один канал на одну адресу */
    }
}

/* --- канальні регістри ------------------------------------------------ */

static uint64_t hpb_chan_read(void *opaque, hwaddr addr, unsigned size)
{
    ClarionHpbDmaState *s = opaque;
    int ch = addr / HPB_CHAN_STRIDE;
    hwaddr off = addr % HPB_CHAN_STRIDE;

    if (ch >= CLARION_HPBDMA_NUM_CHAN) {
        return 0;
    }

    if (hpb_dbg() && ch == 4 &&
        (off == HPB_DDASR || off == HPB_DTCSR || off == HPB_DSTSR)) {
        fprintf(stderr, "[dma] ЧИТАННЯ ch4 +%02x "
                "(plane%u dstsr=%08x ddasr=%08x left=%u)\n",
                (unsigned)off, s->ch[ch].plane, hpb_chan_status(&s->ch[ch]),
                s->ch[ch].ddasr, s->ch[ch].left);
    }
    switch (off) {
    case HPB_DSAR0: return s->ch[ch].sar[0];
    case HPB_DDAR0: return s->ch[ch].dar[0];
    case HPB_DTCR0: return s->ch[ch].tcr[0];
    case HPB_DSAR1: return s->ch[ch].sar[1];
    case HPB_DDAR1: return s->ch[ch].dar[1];
    case HPB_DTCR1: return s->ch[ch].tcr[1];
    case HPB_DSASR: return s->ch[ch].dsasr;
    case HPB_DDASR: return s->ch[ch].ddasr;
    case HPB_DTCSR: return s->ch[ch].dtcsr;
    case HPB_DPTR:  return s->ch[ch].dptr;
    case HPB_DCR:   return s->ch[ch].dcr;
    case HPB_DCMDR: return 0;           /* команда, читається нулем */
    case HPB_DSTPR: return 0;
    case HPB_DSTSR: return hpb_chan_status(&s->ch[ch]);
    default:        return 0;
    }
}

static void hpb_chan_write(void *opaque, hwaddr addr, uint64_t val,
                           unsigned size)
{
    ClarionHpbDmaState *s = opaque;
    int ch = addr / HPB_CHAN_STRIDE;
    hwaddr off = addr % HPB_CHAN_STRIDE;

    if (ch >= CLARION_HPBDMA_NUM_CHAN) {
        return;
    }

    trace_clarion_hpbdma_chan_write(ch, (uint32_t)off, val);

    /*
     * ТИМЧАСОВО (QY8_DMA_VA_PROBE, знести після досліду 24.09.2026).
     *
     * Питання досліду: чому в DSAR/DDAR каналу 21 (SDHI0) потрапляють
     * значення, яких немає у фізичній карті плати, тоді як канали 4/6/16/17
     * дістають правильні фізичні адреси. Зонд перекладає записане значення
     * як ВІРТУАЛЬНУ адресу в контексті того самого CPU, що робить запис.
     * Якщо VA -> PA дає осмислену фізичну адресу — значить драйвер пише
     * віртуальну адресу, а не фізичну.
     */
    if (hpb_va_probe() && (off == HPB_DSAR0 || off == HPB_DDAR0 ||
                           off == HPB_DSAR1 || off == HPB_DDAR1)) {
        CPUState *cs = current_cpu;
        TranslateForDebugResult tres;

        if (cs && cpu_translate_for_debug(cs, val, &tres)) {
            fprintf(stderr, "[va] ch%-2d +0x%02x <- %08" PRIx64
                    "   як VA -> PA %08" HWADDR_PRIx "\n",
                    ch, (unsigned)off, val, tres.physaddr);
        } else {
            fprintf(stderr, "[va] ch%-2d +0x%02x <- %08" PRIx64
                    "   як VA не транслюється\n", ch, (unsigned)off, val);
        }
    }

    switch (off) {
    case HPB_DSAR0: s->ch[ch].sar[0] = val; break;
    case HPB_DDAR0: s->ch[ch].dar[0] = val; break;
    case HPB_DTCR0: s->ch[ch].tcr[0] = val; break;
    case HPB_DSAR1: s->ch[ch].sar[1] = val; break;
    case HPB_DDAR1: s->ch[ch].dar[1] = val; break;
    case HPB_DTCR1: s->ch[ch].tcr[1] = val; break;
    case HPB_DPTR:  s->ch[ch].dptr = val; break;   /* зберігаємо, не тлумачимо */
    case HPB_DCR:   s->ch[ch].dcr = val; break;

    case HPB_DCMDR:
        if (hpb_dbg() && (ch == 4 || ch == 6)) {
            fprintf(stderr, "[dma] DCMDR ch%d <- %08" PRIx64
                    " (active=%d plane%u left=%u)\n",
                    ch, val, s->ch[ch].active, s->ch[ch].plane,
                    s->ch[ch].left);
        }
        if (val & DCMDR_DMEN) {
            hpb_chan_start(s, ch, !!(val & DCMDR_DNXT));
        }
        if ((val & DCMDR_DNXT) && (s->ch[ch].dcr & DCR_CT)) {
            ClarionHpbChan *c = &s->ch[ch];

            c->next_requested = true;
            if (c->active && !c->left) {
                c->next_requested = false;
                hpb_chan_load_plane(s, ch,
                                    (c->dcr & DCR_DIP) ? c->plane ^ 1 : 0);
            }
        }
        if (val & (DCMDR_DQEND | DCMDR_DQSPD | DCMDR_DMSPD)) {
            s->ch[ch].active = false;
            s->ch[ch].next_requested = false;
        }
        break;

    case HPB_DSTPR:
        if (val & DSTPR_DMSTP) {
            s->ch[ch].active = false;
            s->ch[ch].next_requested = false;
        }
        break;

    default:
        break;
    }
}

static const MemoryRegionOps hpb_chan_ops = {
    .read = hpb_chan_read,
    .write = hpb_chan_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* --- спільні регістри ------------------------------------------------- */

static uint64_t hpb_comm_read(void *opaque, hwaddr addr, unsigned size)
{
    ClarionHpbDmaState *s = opaque;
    uint32_t act[2] = { 0, 0 };
    int ch;

    switch (addr) {
    case HPB_DTIMR:   return s->dtimr;
    case HPB_DINTSR0: return s->dintsr[0];
    case HPB_DINTSR1: return s->dintsr[1];
    case HPB_DINTMR0: return s->dintmr[0];
    case HPB_DINTMR1: return s->dintmr[1];
    case HPB_DACTSR0:
    case HPB_DACTSR1:
        for (ch = 0; ch < CLARION_HPBDMA_NUM_CHAN; ch++) {
            if (s->ch[ch].active) {
                act[ch / 32] |= 1u << (ch % 32);
            }
        }
        return addr == HPB_DACTSR0 ? act[0] : act[1];
    default:
        return 0;
    }
}

static void hpb_comm_write(void *opaque, hwaddr addr, uint64_t val,
                           unsigned size)
{
    ClarionHpbDmaState *s = opaque;
    int ch;

    switch (addr) {
    case HPB_DTIMR:
        s->dtimr = val;
        break;
    case HPB_DINTCR0:                   /* запис одиниці скидає стан */
        s->dintsr[0] &= ~(uint32_t)val;
        hpb_update_irq(s);
        break;
    case HPB_DINTCR1:
        s->dintsr[1] &= ~(uint32_t)val;
        hpb_update_irq(s);
        break;
    case HPB_DINTMR0:
        s->dintmr[0] = val;
        hpb_update_irq(s);
        break;
    case HPB_DINTMR1:
        s->dintmr[1] = val;
        hpb_update_irq(s);
        break;
    default:
        /* HSRSTR(n): скидання каналу n (hpb_dmae_reset) */
        if (addr >= HPB_HSRSTR(0) &&
            addr < HPB_HSRSTR(CLARION_HPBDMA_NUM_CHAN) && (val & 1)) {
            ch = (addr - HPB_HSRSTR(0)) / 4;
            memset(&s->ch[ch], 0, sizeof(s->ch[ch]));
            s->dintsr[ch / 32] &= ~(1u << (ch % 32));
            hpb_update_irq(s);
        }
        break;
    }
}

static const MemoryRegionOps hpb_comm_ops = {
    .read = hpb_comm_read,
    .write = hpb_comm_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* --- пристрій --------------------------------------------------------- */

static void clarion_hpbdma_reset_hold(Object *obj, ResetType type)
{
    ClarionHpbDmaState *s = CLARION_HPBDMA(obj);

    memset(s->ch, 0, sizeof(s->ch));
    s->dintsr[0] = s->dintsr[1] = 0;
    s->dintmr[0] = s->dintmr[1] = 0;
    s->dtimr = 0;
    hpb_update_irq(s);
}

static void clarion_hpbdma_realize(DeviceState *dev, Error **errp)
{
    ClarionHpbDmaState *s = CLARION_HPBDMA(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    int i;

    memory_region_init_io(&s->chan_mr, OBJECT(dev), &hpb_chan_ops, s,
                          "clarion-hpbdma.chan", CLARION_HPBDMA_CHAN_SIZE);
    memory_region_init_io(&s->comm_mr, OBJECT(dev), &hpb_comm_ops, s,
                          "clarion-hpbdma.comm", CLARION_HPBDMA_COMM_SIZE);
    sysbus_init_mmio(sbd, &s->chan_mr);
    sysbus_init_mmio(sbd, &s->comm_mr);

    for (i = 0; i < CLARION_HPBDMA_NUM_IRQ; i++) {
        sysbus_init_irq(sbd, &s->irq[i]);
    }
}

static void clarion_hpbdma_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = clarion_hpbdma_realize;
    dc->desc = "Renesas HPB-DMAC (R-Car Gen1)";
    rc->phases.hold = clarion_hpbdma_reset_hold;
}

static const TypeInfo clarion_hpbdma_types[] = {
    {
        .name          = TYPE_CLARION_HPBDMA,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(ClarionHpbDmaState),
        .class_init    = clarion_hpbdma_class_init,
    },
};

DEFINE_TYPES(clarion_hpbdma_types)
