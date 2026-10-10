/*
 * Renesas R-Car Display Unit (Gen1, R8A7778/R8A7779) —
 * модель для плати Clarion QY8XXX.
 *
 * Блок ототожнено не за схожістю адрес, а за магічними кодами регістрів:
 * прошивка кладе 0x66000001 у +0xe0 (DEFR5_CODE = 0x66 << 24), 0x77780010
 * у +0xe8 (DEFR6_CODE = 0x7778 << 16) і 0x77730011 у +0x20 (DEFR_CODE
 * 0x7773) — усе один-в-один із drivers/gpu/drm/renesas/rcar-du/rcar_du_regs.h.
 * Вузол у r8a7779.dtsi: display@fff80000 reg = <0xfff80000 0x40000>,
 * interrupts = <GIC_SPI 31>.
 *
 * Розкладка всередині блока (з rcar_du_regs.h і з того, як адресується
 * драйвер Linux — rcar_du_crtc.c / rcar_du_group.c / rcar_du_plane.c):
 *
 *   +0x00000  канал 0: DSYSR..DEWR, DOOR/BPOR, DEFR5/DDLTR/DEFR6  (CRTC 0)
 *   +0x00100  площини 1..8 групи 0, крок PLANE_OFF = 0x100
 *   +0x01000  палітри CP1..CP4 групи 0 (по 256 слів)
 *   +0x0a100  «A»-площини групи 0
 *   +0x10000  ESCR02 / OTAR02 — регістри синхронізації каналу 0
 *   +0x11000  DORCR / DPTSR / DAPTSR, +0x11020 DS1PR, +0x11024 DS2PR
 *   +0x30000  канал 1: та сама мапа, що й у каналу 0 (DU1_REG_OFFSET)
 *   +0x31000  ESCR13 / OTAR13 — регістри синхронізації каналу 1
 *
 * Завантажувач плати вмикає 800x480 (повний кадр 1055x524) на обох
 * каналах, DOOR = 0, DORCR = PG1T|DK1S|PG1D_DS1, DSYSR каналу 0 = DEN;
 * канал 1 лишається вимкненим. Площини пізніше програмує ddi_ncg.dll у
 * користувацькій частині WinCE. T132 зафіксував register programming P1/P2/P8,
 * а T133 перевірив діагностичне відображення інжектованих кадрів; це не
 * підтверджує поведінку фізичного QY8 DU чи реальний scanout.
 *
 * ⚠ Межа чесності. Рендер площин, DPPR та програмування регістрів
 * реалізовані за rcar_du_regs.h і драйвером Linux. T133 також додав
 * діагностичні color-key та alpha правила за encoding Linux v6.6
 * (перемикач blend); ABIT_1 є inference. Ці результати перевірені лише у
 * QEMU diagnostic captures, а color key/alpha та решта відповідної DU
 * поведінки НЕ ПЕРЕВІРЕНІ на фізичному QY8 hardware. Де поведінка не
 * підтверджена апаратурою, це позначено коментарями біля реалізації.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/sysbus.h"
#include "hw/display/clarion_du.h"
#include "qemu/timer.h"
#include "trace.h"
#include "hw/core/cpu.h"
#include "ui/console.h"
#include "ui/pixel_ops.h"
#include "system/address-spaces.h"
#include "qom/object.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include <sys/mman.h>

/* --- регістри (імена й зсуви — з rcar_du_regs.h) ---------------------- */

#define DU0_REG_OFFSET      0x00000
#define DU1_REG_OFFSET      0x30000
#define DU_NUM_CHAN         2

#define DSYSR               0x00000
#define DSYSR_DRES          (1 << 9)
#define DSYSR_DEN           (1 << 8)

#define DSSR                0x00008     /* статус, тільки читання */
#define DSRCR               0x0000c     /* скидання бітів статусу */
#define DIER                0x00010     /* дозвіл переривань */

/*
 * Біт 11 у DSSR/DIER — кадрова синхронізація (vertical blanking).
 * Іменування й номер біта — з rcar_du_regs.h (`DSSR_VBK`, `DIER_VBE`);
 * підтверджено з обох боків: драйвер HU пише в DIER рівно 0x800, а його
 * потік обробки переривання (ddi_ncg.dll+0x12e78) читає DSSR, перевіряє
 * `tst r3, #0x800` і скидає біт записом 0x800 у DSRCR.
 */
#define DSSR_VBK            (1 << 11)
#define DIER_VBE            (1 << 11)

#define HCR                 0x00050     /* повна ширина кадру - 1 */
#define VCR                 0x00058     /* повна висота кадру - 1 */

#define DPPR                0x00018     /* пріоритети й вибір площин */
/*
 * Кожен нібл DPPR — один рівень пріоритету n = 1..8 (нібл n-1):
 *   DPE(n)    = 1 << (n*4 - 1)        — біт 3 нібла, «рівень увімкнено»
 *   DPS(n, p) = (p - 1) << (n-1)*4    — біти 2..0, номер площини 1..8
 * (rcar_du_regs.h). Завантажувач кладе 0x76543210 — усі вісім рівнів
 * розписані по площинах, але ЖОДНОГО DPE, тобто нічого не ввімкнено.
 */
#define DPPR_DPE(n)         (1u << ((n) * 4 - 1))
#define DPPR_NIB(v, n)      (((v) >> (((n) - 1) * 4)) & 0xf)

/* Генератор таймінгів */
#define HDSR                0x00040
#define HDER                0x00044
#define VDSR                0x00048
#define VDER                0x0004c

/* Атрибути виводу. DOOR — колір, коли дисплей вимкнено; BPOR — колір
 * фонової площини під усіма площинами (rcar_du_crtc.c: «Set display off
 * and background to black»). Обидва — 8 біт на канал: (r<<18)|(g<<10)|(b<<2). */
#define DOOR                0x00090
#define BPOR                0x00098

/* Площини групи: площина n (1..8) — це group + n * PLANE_OFF */
#define PLANE_OFF           0x00100
#define DU_NUM_PLANES       8

#define PnMR                0x000
#define PnMR_CPSL_SHIFT     8
#define PnMR_SPIM_SHIFT     12
#define PnMR_SPIM_MASK      (3u << PnMR_SPIM_SHIFT)
#define PnMR_SPIM_TP         (0u << PnMR_SPIM_SHIFT)
#define PnMR_SPIM_ALP        (1u << PnMR_SPIM_SHIFT)
#define PnMR_SPIM_EOR        (2u << PnMR_SPIM_SHIFT)
#define PnMR_SPIM_TP_OFF     (1u << 14)
#define PnMR_TC              (1u << 17)
#define PnMR_WAE             (1u << 16)
#define PnMR_DDDF_MASK      3
#define PnMR_DDDF_8BPP      0
#define PnMR_DDDF_16BPP     1
#define PnMR_DDDF_ARGB      2
#define PnMR_DDDF_YC        3
#define PnMWR               0x004       /* крок рядка В ПІКСЕЛЯХ */
#define PnDSXR              0x010
#define PnDSYR              0x014
#define PnDPXR              0x018
#define PnDPYR              0x01c
#define PnDSA0R             0x020
#define PnDSA_MASK          0xfffffff0
#define PnALPHAR            0x008
#define PnALPHAR_ALPHA_MASK 0xff
#define PnALPHAR_ABIT_SHIFT 12
#define PnALPHAR_ABIT_MASK  (3u << PnALPHAR_ABIT_SHIFT)
#define PnALPHAR_ABIT_1     (0u << PnALPHAR_ABIT_SHIFT)
#define PnALPHAR_ABIT_0     (1u << PnALPHAR_ABIT_SHIFT)
#define PnALPHAR_ABIT_X     (2u << PnALPHAR_ABIT_SHIFT)
#define PnTC1R              0x044
#define PnTC2R              0x048
#define PnTC3R              0x04c
#define PnSPXR              0x030
#define PnSPYR              0x034
#define PnSWAPR             0x080
#define PnDDCR4             0x090
#define PnDDCR4_EDF_MASK    7
#define PnDDCR4_EDF_NONE    0
#define PnDDCR4_EDF_ARGB8888 1
#define PnDDCR4_EDF_RGB888  2
#define PnDDCR4_EDF_RGB666  3

/* Палітри групи */
#define CP1_000R            0x01000
#define CP_STRIDE           0x01000

/* Керування подвійним виводом (регістри групи) */
#define DORCR               0x11000
#define DORCR_PG0D_MASK     (3 << 16)
#define DORCR_PG0D_DS0      (0 << 16)
#define DORCR_PG0D_DS1      (1 << 16)
#define DORCR_PG0D_FIX0     (2 << 16)
#define DORCR_PG0D_DOOR     (3 << 16)
#define DPTSR               0x11004
#define DPTSR_PnTS(n)       (1 << (n))  /* n — 0-based індекс площини */
#define DS1PR               0x11020     /* склад суперпозиції 0 */
#define DS2PR               0x11024     /* склад суперпозиції 1 */

/* --- стан ------------------------------------------------------------- */

OBJECT_DECLARE_SIMPLE_TYPE(ClarionDuState, CLARION_DU)

struct ClarionDuState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    QemuConsole *con;

    uint32_t *reg;              /* увесь блок як масив слів */
    int cols, rows;
    bool invalidate;

    qemu_irq irq;               /* GIC_SPI 31 */
    QEMUTimer *vbk;             /* кадровий такт */
    uint32_t dotclk;            /* точкова частота, Гц (властивість) */

    /* щоб не засмічувати лог однаковими скаргами */
    bool warned_chan1;
    bool warned_yc;
    bool warned_swap;
    bool warned_dppr;
    bool warned_bpp;

    /* frame rendered by the qy8gl plugin, blended over the planes */
    int gl_fd;
    uint8_t *gl_map;
    size_t gl_len;
    uint32_t gl_frame;
    bool warned_du_semantics;
    bool blend;
};

static inline uint32_t du_rd(ClarionDuState *s, hwaddr off)
{
    return s->reg[off / 4];
}

static inline hwaddr du_chan_base(int ch)
{
    return ch ? DU1_REG_OFFSET : DU0_REG_OFFSET;
}

/* DOOR/BPOR/палітра -> 0x00RRGGBB */
static inline uint32_t du_color(uint32_t v)
{
    return (((v >> 18) & 0xff) << 16) |
           (((v >> 10) & 0xff) << 8) |
           ((v >> 2) & 0xff);
}

/*
 * Розмір кадру береться з генератора таймінгів, а не зашивається:
 * ширина = HDER - HDSR, висота = VDER - VDSR. Для цієї плати
 * завантажувач кладе 984-184 = 800 і 511-31 = 480.
 */
static bool du_geometry(ClarionDuState *s, int ch, int *w, int *h)
{
    hwaddr b = du_chan_base(ch);
    int32_t cw = (int32_t)du_rd(s, b + HDER) - (int32_t)du_rd(s, b + HDSR);
    int32_t chh = (int32_t)du_rd(s, b + VDER) - (int32_t)du_rd(s, b + VDSR);

    if (cw <= 0 || chh <= 0 || cw > 4096 || chh > 4096) {
        return false;
    }
    *w = cw;
    *h = chh;
    return true;
}

/* --- площини (НЕПЕРЕВІРЕНО: гість їх ще не програмує) ------------------ */

typedef enum {
    DU_FMT_PAL8,
    DU_FMT_RGB565,
    DU_FMT_ARGB1555,
    DU_FMT_XRGB8888,
    DU_FMT_ARGB8888,
    DU_FMT_UNSUPPORTED,
} DuFormat;

static DuFormat du_plane_format(ClarionDuState *s, int plane, int *bpp)
{
    hwaddr pb = plane * PLANE_OFF;
    uint32_t pnmr = du_rd(s, pb + PnMR);
    uint32_t edf = du_rd(s, pb + PnDDCR4) & PnDDCR4_EDF_MASK;

    /*
     * PnMR.DDDF розрізняє 8bpp / «16 або 32 біти» / ARGB1555 / YC, а який
     * саме з «16 або 32» — каже EDF у PnDDCR4 (rcar_du_kms.c, таблиця
     * rcar_du_format_infos: RGB565 = DDDF_16BPP + EDF_NONE, XRGB8888 =
     * DDDF_16BPP + EDF_RGB888, ARGB8888 = DDDF_16BPP + EDF_ARGB8888).
     * Драйвер пише PnDDCR4 і на Gen1 теж — там gen < 3 іде в
     * rcar_du_plane_setup_format_gen2().
     */
    switch (pnmr & PnMR_DDDF_MASK) {
    case PnMR_DDDF_8BPP:
        *bpp = 1;
        return DU_FMT_PAL8;
    case PnMR_DDDF_ARGB:
        *bpp = 2;
        return DU_FMT_ARGB1555;
    case PnMR_DDDF_16BPP:
        switch (edf) {
        case PnDDCR4_EDF_ARGB8888:
            *bpp = 4;
            return DU_FMT_ARGB8888;
        case PnDDCR4_EDF_RGB888:
            *bpp = 4;
            return DU_FMT_XRGB8888;
        case PnDDCR4_EDF_NONE:
            *bpp = 2;
            return DU_FMT_RGB565;
        default:
            break;
        }
        break;
    default:
        break;
    }

    *bpp = 0;
    return DU_FMT_UNSUPPORTED;
}

static uint32_t du_decode_pixel(ClarionDuState *s, DuFormat fmt,
                                const uint8_t *p, uint32_t palette_base)
{
    uint32_t v;

    switch (fmt) {
    case DU_FMT_PAL8:
        return du_color(du_rd(s, palette_base + p[0] * 4));
    case DU_FMT_RGB565:
        v = p[0] | (p[1] << 8);
        return (((v >> 11) & 0x1f) * 255 / 31) << 16 |
               (((v >> 5) & 0x3f) * 255 / 63) << 8 |
               ((v & 0x1f) * 255 / 31);
    case DU_FMT_ARGB1555:
        v = p[0] | (p[1] << 8);
        return (((v >> 10) & 0x1f) * 255 / 31) << 16 |
               (((v >> 5) & 0x1f) * 255 / 31) << 8 |
               ((v & 0x1f) * 255 / 31);
    case DU_FMT_XRGB8888:
    case DU_FMT_ARGB8888:
        /* слово в пам'яті мало-endian: 0xAARRGGBB */
        return p[0] | (p[1] << 8) | (p[2] << 16);
    default:
        return 0;
    }
}

/*
 * Список площин суперпозиції, знизу вгору.
 *
 * Джерело — DS1PR (суперпозиція 0) / DS2PR (суперпозиція 1): драйвер
 * (rcar_du_crtc_update_planes) кладе туди по нібблу на площину, значення
 * = номер площини 1..8, 0 = порожньо; причому НИЖНІЙ нібл — найверхній
 * шар (prio рахується від кількості площин униз). Тому обхід від старшого
 * ніббла до молодшого і дає порядок «знизу вгору».
 *
 * Якщо DS1PR/DS2PR порожні, дивимося DPPR — саме ним користується рідний
 * драйвер плати ddi_ncg.dll (див. нижче).
 */
static int du_plane_list(ClarionDuState *s, int sp, int *order)
{
    uint32_t dspr = du_rd(s, sp ? DS2PR : DS1PR);
    uint32_t dptsr = du_rd(s, DPTSR);
    int n = 0;

    for (int nib = 7; nib >= 0; nib--) {
        int plane = (dspr >> (nib * 4)) & 0xf;

        if (plane < 1 || plane > DU_NUM_PLANES) {
            continue;
        }
        /* DPTSR: 0 -> площина живить суперпозицію 0, 1 -> суперпозицію 1 */
        if (!!(dptsr & DPTSR_PnTS(plane - 1)) != !!sp) {
            continue;
        }
        order[n++] = plane;
    }

    /*
     * Запасний шлях: DPPR.
     *
     * Драйвер Linux цей регістр не чіпає, тому раніше ми його не тлумачили.
     * Але ddi_ncg.dll (рідний драйвер плати) користується САМЕ ним: DS1PR і
     * DS2PR він лишає нульовими, а в DPPR міняє 0x76543210 -> 0x76543218,
     * тобто ставить DPE на рівні 1 з DPS = 0 -> площина 1. Виміряно на
     * живому буті (docs/04-journal.md, запис «Перший кадр»); там же видно,
     * що площину 1 він при цьому повністю програмує (800x480, ARGB1555,
     * PnDSA0R = 0x10119400), і в тому буфері лежать ненульові пікселі.
     *
     * Порядок обходу — той самий, що й для DS1PR: від старшого рівня до
     * молодшого, тож молодший лягає зверху.
     */
    if (!n) {
        uint32_t dppr = du_rd(s, DPPR);

        for (int lvl = DU_NUM_PLANES; lvl >= 1; lvl--) {
            int plane;

            if (!(dppr & DPPR_DPE(lvl))) {
                continue;               /* рівень вимкнено */
            }
            plane = (int)(DPPR_NIB(dppr, lvl) & 7) + 1;
            if (!!(dptsr & DPTSR_PnTS(plane - 1)) != !!sp) {
                continue;
            }
            order[n++] = plane;
        }
    }
    return n;
}

/* Малювання однієї площини поверх уже готового рядка кадру. */
static void du_draw_plane(ClarionDuState *s, int plane,
                          uint32_t *fb, int cols, int rows)
{
    hwaddr pb = plane * PLANE_OFF;
    int bpp;
    DuFormat fmt = du_plane_format(s, plane, &bpp);
    uint32_t dsa = du_rd(s, pb + PnDSA0R) & PnDSA_MASK;
    uint32_t pnmr = du_rd(s, pb + PnMR);
    uint32_t spim = pnmr & PnMR_SPIM_MASK;
    uint32_t alphar = du_rd(s, pb + PnALPHAR);
    uint32_t tc2 = du_rd(s, pb + PnTC2R);
    uint32_t mwr = du_rd(s, pb + PnMWR);        /* крок рядка в пікселях */
    int dsx = du_rd(s, pb + PnDSXR);
    int dsy = du_rd(s, pb + PnDSYR);
    int dpx = du_rd(s, pb + PnDPXR);
    int dpy = du_rd(s, pb + PnDPYR);
    int spx = du_rd(s, pb + PnSPXR);
    int spy = du_rd(s, pb + PnSPYR);
    uint32_t palette;
    g_autofree uint8_t *line = NULL;

    if (fmt == DU_FMT_UNSUPPORTED) {
        if (!s->warned_yc) {
            s->warned_yc = true;
            qemu_log_mask(LOG_UNIMP, "clarion-du: площина %d у форматі YC/"
                          "невідомому (PnMR %#x, PnDDCR4 %#x) — не малюємо\n",
                          plane, du_rd(s, pb + PnMR), du_rd(s, pb + PnDDCR4));
        }
        return;
    }
    /* Поля/режими нижче мають підтримку лише за Linux v6.6 driver encoding;
     * поведінка кремнію QY8 цим не підтверджується. */
    if (!s->warned_du_semantics &&
        ((pnmr & (PnMR_TC | PnMR_WAE)) || spim == PnMR_SPIM_EOR ||
         ((pnmr & PnMR_DDDF_MASK) == PnMR_DDDF_8BPP) ||
         fmt == DU_FMT_XRGB8888 || fmt == DU_FMT_ARGB8888)) {
        s->warned_du_semantics = true;
        qemu_log_mask(LOG_UNIMP, "clarion-du: частина семантики DU "
                      "(TC/WAE/EOR, 8/32bpp key) не моделюється\n");
    }
    if (du_rd(s, pb + PnSWAPR) && !s->warned_swap) {
        s->warned_swap = true;
        qemu_log_mask(LOG_UNIMP, "clarion-du: PnSWAPR площини %d = %#x — "
                      "перестановку байтів не моделюємо\n",
                      plane, du_rd(s, pb + PnSWAPR));
    }
    if (!dsa || dsx <= 0 || dsy <= 0 || mwr == 0) {
        return;
    }

    /* PnMR.CPSL обирає палітру CP1..CP4 групи */
    palette = CP1_000R + ((du_rd(s, pb + PnMR) >> PnMR_CPSL_SHIFT) & 3)
              * CP_STRIDE;

    line = g_malloc(dsx * (size_t)bpp);

    for (int y = 0; y < dsy; y++) {
        int dy = dpy + y;
        hwaddr src;
        uint32_t *dst;

        if (dy < 0 || dy >= rows) {
            continue;
        }
        /*
         * НЕПЕРЕВІРЕНО. PnMWR — крок у пікселях, PnSPXR/PnSPYR — зсув у
         * джерелі (піксель / растровий рядок). Драйвер Linux для 32bpp
         * подвоює SPY «згідно з документацією R8A7790»; на повноекранній
         * поверхні SPY = 0, тож тут беремо пряме тлумачення і лишаємо це
         * питання відкритим до M3b.
         */
        src = dsa + (hwaddr)(spy + y) * mwr * bpp + (hwaddr)spx * bpp;
        if (address_space_read(&address_space_memory, src,
                               MEMTXATTRS_UNSPECIFIED, line,
                               dsx * (size_t)bpp) != MEMTX_OK) {
            return;
        }

        dst = fb + (size_t)dy * cols;
        for (int x = 0; x < dsx; x++) {
            int dx = dpx + x;

            if (dx < 0 || dx >= cols) {
                continue;
            }
            const uint8_t *pixel = line + (size_t)x * bpp;
            uint32_t src = du_decode_pixel(s, fmt, pixel, palette);

            if (s->blend && !(pnmr & PnMR_SPIM_TP_OFF) &&
                (fmt == DU_FMT_ARGB1555 || fmt == DU_FMT_RGB565)) {
                /* За драйвером Linux v6.6; на фізичному QY8 hardware
                 * не перевірено.
                 * Чинність ключа в SPIM=ALP лишається окремим питанням. */
                uint16_t raw = pixel[0] | ((uint16_t)pixel[1] << 8);
                uint16_t key_mask = fmt == DU_FMT_ARGB1555 ? 0x7fff : 0xffff;
                if ((raw & key_mask) == (tc2 & key_mask)) {
                    continue;
                }
            }

            if (s->blend && fmt == DU_FMT_ARGB1555 &&
                spim == PnMR_SPIM_ALP) {
                /* За драйвером Linux v6.6, на фізичному QY8 hardware не
                 * перевірено: коефіцієнт PnALPHAR і ABIT_0; ABIT_1 —
                 * симетрична inference. */
                uint32_t abit = alphar & PnALPHAR_ABIT_MASK;
                bool a_bit = !!(pixel[1] & 0x80);
                bool blend_pixel =
                    (abit == PnALPHAR_ABIT_X) ||
                    (abit == PnALPHAR_ABIT_1 && a_bit) ||
                    (abit == PnALPHAR_ABIT_0 && !a_bit);
                if (blend_pixel) {
                    uint32_t a = alphar & PnALPHAR_ALPHA_MASK;
                    uint32_t dstc = dst[dx];
                    uint32_t r = (((src >> 16) & 0xff) * a +
                                  ((dstc >> 16) & 0xff) * (255 - a)) / 255;
                    uint32_t g = (((src >> 8) & 0xff) * a +
                                  ((dstc >> 8) & 0xff) * (255 - a)) / 255;
                    uint32_t b = ((src & 0xff) * a +
                                  (dstc & 0xff) * (255 - a)) / 255;
                    src = (r << 16) | (g << 8) | b;
                }
            }
            dst[dx] = src;
        }
    }
}

/* --- вивід у вікно ---------------------------------------------------- */

/*
 * The qy8gl plugin draws the AUI's GL calls itself and publishes each frame
 * in the file named by QY8_GL_FRAME: "QYGL", frame count, width, height, then
 * RGBA rows bottom-up as GL keeps them. It covers the planes completely.
 */
static bool du_gl_overlay(ClarionDuState *s, uint32_t *fb, int w, int h,
                          bool draw)
{
    const char *path = getenv("QY8_GL_FRAME");
    struct stat st;
    uint32_t hdr[4];

    if (!path) {
        return false;
    }
    if (!s->gl_map) {
        s->gl_fd = open(path, O_RDONLY);
        if (s->gl_fd < 0) {
            return false;
        }
        if (fstat(s->gl_fd, &st) < 0 || st.st_size < 16) {
            close(s->gl_fd);
            return false;
        }
        s->gl_len = st.st_size;
        s->gl_map = mmap(NULL, s->gl_len, PROT_READ, MAP_SHARED, s->gl_fd, 0);
        if (s->gl_map == MAP_FAILED) {
            s->gl_map = NULL;
            close(s->gl_fd);
            return false;
        }
    }
    memcpy(hdr, s->gl_map, sizeof(hdr));
    if (hdr[0] != 0x4c475951 || !hdr[1] ||
        16 + (size_t)hdr[2] * hdr[3] * 4 > s->gl_len) {
        return false;
    }
    if (!draw) {
        return hdr[1] != s->gl_frame;
    }
    s->gl_frame = hdr[1];
    for (int y = 0; y < h && y < (int)hdr[3]; y++) {
        const uint8_t *src = s->gl_map + 16 +
                             (size_t)(hdr[3] - 1 - y) * hdr[2] * 4;
        uint32_t *row = fb + (size_t)y * w;

        /* the AUI keeps a plane-mixing mask in alpha, not opacity */
        for (int x = 0; x < w && x < (int)hdr[2]; x++, src += 4) {
            row[x] = src[0] << 16 | src[1] << 8 | src[2];
        }
    }
    return true;
}

static bool clarion_du_gfx_update(void *opaque)
{
    ClarionDuState *s = opaque;
    DisplaySurface *surface = qemu_console_surface(s->con);
    int order[DU_NUM_PLANES];
    int w, h, nplanes, sp, src_ch;
    uint32_t dorcr, bg;
    g_autofree uint32_t *fb = NULL;
    uint8_t *dest;

    /* Поки завантажувач не поклав таймінги — показувати нічого. */
    if (!du_geometry(s, 0, &w, &h)) {
        return true;
    }
    if (w != s->cols || h != s->rows) {
        qemu_console_resize(s->con, w, h);
        surface = qemu_console_surface(s->con);
        s->cols = w;
        s->rows = h;
        s->invalidate = true;
    }
    if (surface_bits_per_pixel(surface) != 32) {
        if (!s->warned_bpp) {
            s->warned_bpp = true;
            qemu_log_mask(LOG_UNIMP, "clarion-du: поверхня %d біт на піксель — "
                          "модель уміє лише 32\n",
                          surface_bits_per_pixel(surface));
        }
        return true;
    }

    /*
     * DORCR.PG0D каже, що саме виходить на DPAD0 — єдиний вихід, до якого
     * на цій платі під'єднана панель: суперпозиція 0, суперпозиція 1,
     * постійний нуль або колір DOOR.
     */
    dorcr = du_rd(s, DORCR);
    switch (dorcr & DORCR_PG0D_MASK) {
    case DORCR_PG0D_DS1:
        sp = 1;
        src_ch = 1;
        break;
    case DORCR_PG0D_FIX0:
    case DORCR_PG0D_DOOR:
        sp = -1;
        src_ch = 0;
        break;
    default:
        sp = 0;
        src_ch = 0;
        break;
    }

    if (!(du_rd(s, du_chan_base(src_ch) + DSYSR) & DSYSR_DEN)) {
        /* Дисплей вимкнено — на вихід іде колір DOOR (rcar_du_crtc.c). */
        sp = -1;
    }

    if (sp < 0) {
        bg = ((dorcr & DORCR_PG0D_MASK) == DORCR_PG0D_FIX0)
             ? 0 : du_color(du_rd(s, du_chan_base(src_ch) + DOOR));
        nplanes = 0;
    } else {
        bg = du_color(du_rd(s, du_chan_base(src_ch) + BPOR));
        nplanes = du_plane_list(s, sp, order);
    }

    /* Без площин кадр — константа: перемальовуємо лише коли щось змінилось. */
    if (!nplanes && !s->invalidate && !du_gl_overlay(s, NULL, w, h, false)) {
        return true;
    }

    fb = g_new(uint32_t, (size_t)w * h);
    for (size_t i = 0; i < (size_t)w * h; i++) {
        fb[i] = bg;
    }
    for (int i = 0; i < nplanes; i++) {
        du_draw_plane(s, order[i], fb, w, h);
    }
    du_gl_overlay(s, fb, w, h, true);

    dest = surface_data(surface);
    for (int y = 0; y < h; y++) {
        uint32_t *row = (uint32_t *)(dest + (size_t)y * surface_stride(surface));

        for (int x = 0; x < w; x++) {
            uint32_t c = fb[(size_t)y * w + x];

            row[x] = rgb_to_pixel32((c >> 16) & 0xff, (c >> 8) & 0xff, c & 0xff);
        }
    }

    qemu_console_update(s->con, 0, 0, w, h);
    s->invalidate = false;
    return true;
}

static void clarion_du_invalidate(void *opaque)
{
    ClarionDuState *s = opaque;

    s->invalidate = true;
}

static const GraphicHwOps clarion_du_gfx_ops = {
    .invalidate = clarion_du_invalidate,
    .gfx_update = clarion_du_gfx_update,
};

/* --- MMIO ------------------------------------------------------------- */

static uint64_t clarion_du_read(void *opaque, hwaddr addr, unsigned size)
{
    ClarionDuState *s = opaque;

    /*
     * DSSR (статус кадру: VBK, HBK, FRM, ...) читається нулем.
     *
     * Завантажувач вмикає DIER = DIER_VBE, тобто переривання кадрової
     * синхронізації (у r8a7779.dtsi це GIC_SPI 31). Щоб його генерувати,
     * треба знати точкову частоту, а вона в DU не лежить: ESCR02 тут
     * дорівнює 0, тобто DCLKSEL = DCLKIN — зовнішній такт, джерела на
     * його частоту в нас немає. Вигадувати її ми не стали, тож ні
     * лічильника кадрів, ні лінії переривання модель не має. Відкрите
     * питання до M3b: чи чекає на VBK користувацька частина.
     */
    if ((addr & 0xffff) == DSSR) {
        /*
         * Лічильник читань статусу кадру. Потрібен, щоб відрізнити «драйвер
         * узагалі не цікавиться кадровою синхронізацією» від «драйвер її
         * чекає, а модель мовчить». DIER=VBK завантажувач вмикає на 917 мс.
         */
        static uint32_t dssr_reads;
        trace_clarion_du_dssr_read(
            (uint32_t)(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) / SCALE_MS),
            (uint32_t)addr, ++dssr_reads);
    }

    return s->reg[addr / 4];
}

/*
 * Ім'я регістра для журналу. Площини (`P1DSA0R` тощо) складаються на льоту:
 * площина n живе за group + n * PLANE_OFF, тож номер відновлюється діленням.
 */
static const char *du_reg_name(hwaddr addr)
{
    static char buf[16];
    hwaddr off = addr & 0xffff;

    switch (off) {
    case DSYSR:   return "DSYSR";
    case DSSR:    return "DSSR";
    case DSRCR:   return "DSRCR";
    case DIER:    return "DIER";
    case DPPR:    return "DPPR";
    case DOOR:    return "DOOR";
    case BPOR:    return "BPOR";
    case HDSR:    return "HDSR";
    case HDER:    return "HDER";
    case VDSR:    return "VDSR";
    case VDER:    return "VDER";
    case DORCR:   return "DORCR";
    case DPTSR:   return "DPTSR";
    case DS1PR:   return "DS1PR";
    case DS2PR:   return "DS2PR";
    default:      break;
    }

    if (off >= PLANE_OFF && off < (DU_NUM_PLANES + 1) * PLANE_OFF) {
        int plane = off / PLANE_OFF;
        hwaddr po = off % PLANE_OFF;
        const char *r = NULL;

        switch (po) {
        case PnMR:     r = "MR";     break;
        case PnMWR:    r = "MWR";    break;
        case PnALPHAR: r = "ALPHAR"; break;
        case PnTC1R:   r = "TC1R";   break;
        case PnTC2R:   r = "TC2R";   break;
        case PnTC3R:   r = "TC3R";   break;
        case 0x084:    r = "DDCR";   break;
        case 0x088:    r = "DDCR2";  break;
        case PnDSXR:   r = "DSXR";   break;
        case PnDSYR:   r = "DSYR";   break;
        case PnDPXR:   r = "DPXR";   break;
        case PnDPYR:   r = "DPYR";   break;
        case PnDSA0R:  r = "DSA0R";  break;
        case PnSPXR:   r = "SPXR";   break;
        case PnSPYR:   r = "SPYR";   break;
        case PnSWAPR:  r = "SWAPR";  break;
        case PnDDCR4:  r = "DDCR4";  break;
        default:       break;
        }
        if (r) {
            snprintf(buf, sizeof(buf), "P%d%s", plane, r);
            return buf;
        }
    }

    snprintf(buf, sizeof(buf), "?%05x", (unsigned)off);
    return buf;
}

/* --- кадрова синхронізація (VBK) -------------------------------------- */

/*
 * Лінія переривання зведена, поки є хоч один дозволений і незнятий біт
 * статусу. Обробник у драйвері знімає біт записом у DSRCR — і лінія падає
 * сама, без окремого «ack» від моделі.
 */
static void du_irq_update(ClarionDuState *s)
{
    qemu_set_irq(s->irq, !!(du_rd(s, DSSR) & du_rd(s, DIER)));
}

/*
 * Період кадру береться з РЕГІСТРІВ, які запрограмував сам гість:
 * повний кадр = (HCR + 1) x (VCR + 1) точок. На HU це 1056 x 525.
 *
 * Єдине число, якого немає в регістрах, — сама точкова частота: ESCR02
 * лишається 0, тобто DCLKSEL = DCLKIN, зовнішній такт від TCON
 * (PA 0xFFF18000). Його джерела ми не знаємо, тому це ВЛАСТИВІСТЬ машини
 * `dotclk` зі значенням за замовчуванням 33 333 333 Гц — опорний EXTAL
 * плат R-Car M1A. Для кадру 1056 x 525 це дає 60.1 Гц.
 */
static void du_vbk_resched(ClarionDuState *s)
{
    uint64_t total = (uint64_t)(du_rd(s, HCR) + 1) * (du_rd(s, VCR) + 1);
    bool on = (du_rd(s, DSYSR) & DSYSR_DEN) && (du_rd(s, DIER) & DIER_VBE);

    if (!on || total < 2 || !s->dotclk) {
        timer_del(s->vbk);
        return;
    }
    timer_mod_ns(s->vbk, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                 muldiv64(total, NANOSECONDS_PER_SECOND, s->dotclk));
}

static void du_vbk_tick(void *opaque)
{
    ClarionDuState *s = opaque;

    s->reg[DSSR / 4] |= DSSR_VBK;
    du_irq_update(s);
    du_vbk_resched(s);
}

static void clarion_du_write(void *opaque, hwaddr addr, uint64_t val,
                             unsigned size)
{
    ClarionDuState *s = opaque;
    hwaddr off = addr & 0xffff;

    /*
     * Журнал іде ПЕРЕД обробкою DSSR/DSRCR: інакше записи в DSRCR (скидання
     * бітів статусу — саме те, що робить обробник переривання) губилися б у
     * ранньому `return` і виглядали б як «їх не було».
     */
    if (off == DSSR || off == DSRCR || s->reg[addr / 4] != (uint32_t)val) {
        /*
         * Журнал змін регістрів — щоб бачити, КОЛИ саме змінюється картинка
         * і який саме регістр за це відповідає. Пишемо лише справжні зміни:
         * драйвер переписує ті самі значення щокадру, і без фільтра трас
         * тоне в повторах. Час — віртуальний, у мілісекундах, той самий, за
         * яким мітить рядки консоль гостя.
         */
        CPUState *cs = current_cpu;
        uint64_t pc = 0;

        /*
         * PC гостя. У TCG він оновлюється на межах блоків трансляції, тож
         * це адреса З ТОЧНІСТЮ ДО БЛОКУ, а не сама інструкція `str`. Для
         * відповіді «який модуль пише» цього досить: бази модулів XIP
         * рознесені на десятки кілобайтів.
         */
        if (cs) {
            pc = CPU_GET_CLASS(cs)->get_pc(cs);
        }
        trace_clarion_du_reg_write(
            (uint32_t)(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) / SCALE_MS),
            du_reg_name(addr), (uint32_t)addr,
            s->reg[addr / 4], (uint32_t)val, pc);
    }

    if (off == DSSR) {
        return;                 /* статус — тільки читання */
    }
    if (off == DSRCR) {
        s->reg[DSSR / 4] &= ~(uint32_t)val;
        du_irq_update(s);
        return;
    }

    s->reg[addr / 4] = val;

    /* Усе, від чого залежить кадровий такт каналу 0. */
    if (addr == DSYSR || addr == DIER || addr == HCR || addr == VCR) {
        du_vbk_resched(s);
        if (addr == DIER) {
            du_irq_update(s);
        }
    }

    if (addr == DU1_REG_OFFSET + DSYSR && (val & DSYSR_DEN) &&
        !s->warned_chan1) {
        s->warned_chan1 = true;
        qemu_log_mask(LOG_UNIMP, "clarion-du: увімкнено канал 1 (DU1) — "
                      "у вікно виводиться лише DPAD0\n");
    }

    s->invalidate = true;
}

static const MemoryRegionOps clarion_du_ops = {
    .read = clarion_du_read,
    .write = clarion_du_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void clarion_du_reset_hold(Object *obj, ResetType type)
{
    ClarionDuState *s = CLARION_DU(obj);

    memset(s->reg, 0, CLARION_DU_SIZE);
    s->cols = 0;
    s->rows = 0;
    s->invalidate = true;
    if (s->vbk) {
        timer_del(s->vbk);
    }
    qemu_set_irq(s->irq, 0);
}

static void clarion_du_realize(DeviceState *dev, Error **errp)
{
    ClarionDuState *s = CLARION_DU(dev);

    s->reg = g_malloc0(CLARION_DU_SIZE);
    memory_region_init_io(&s->iomem, OBJECT(dev), &clarion_du_ops, s,
                          "clarion-du", CLARION_DU_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
    s->vbk = timer_new_ns(QEMU_CLOCK_VIRTUAL, du_vbk_tick, s);

    s->con = qemu_graphic_console_create(dev, 0, &clarion_du_gfx_ops, s);
}

static const Property clarion_du_props[] = {
    /* Diagnostic renderer switch; Linux v6.6 field semantics are not QY8 verified. */
    DEFINE_PROP_BOOL("blend", ClarionDuState, blend, true),
    /*
     * Точкова частота DCLKIN у герцах — період кадру = (HCR+1)*(VCR+1)/dotclk.
     * У регістрах DU її немає: ESCR02 = 0, тобто такт зовнішній, від TCON
     * (PA 0xFFF18000).
     *
     * ⚠ Типово 0 — кадрове переривання НЕ генерується. Не тому, що модель
     * неправильна, а тому, що НЕ ВСТАНОВЛЕНО, якою лінією GIC воно доходить
     * до цієї прошивки: драйвер бере SYSINTR = 37 з власної константи, а
     * перебір усіх 17 ліній, які OAL узагалі вмикає, не розбудив його
     * потік обробки (docs/04-journal.md, 24.09). Поки лінія не відома,
     * генерувати переривання означало б стукати в чужі двері.
     *
     * Коли лінію знайдуть: `-global clarion-du.dotclk=33333333` (опорний
     * EXTAL плат R-Car M1A; з кадром 1056 x 525 це 60.1 Гц) разом із
     * `-M clarion-qy8,du-spi=<номер>`.
     */
    DEFINE_PROP_UINT32("dotclk", ClarionDuState, dotclk, 0),
};

static void clarion_du_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = clarion_du_realize;
    dc->desc = "Renesas R-Car Display Unit (Gen1)";
    device_class_set_props(dc, clarion_du_props);
    rc->phases.hold = clarion_du_reset_hold;
}

static const TypeInfo clarion_du_types[] = {
    {
        .name          = TYPE_CLARION_DU,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(ClarionDuState),
        .class_init    = clarion_du_class_init,
    },
};

DEFINE_TYPES(clarion_du_types)
