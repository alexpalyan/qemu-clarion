/*
 * Renesas SDHI — SD/MMC Host Interface (ядро TMIO) з R-Car Gen1 (R8A7778).
 *
 * На платі Clarion QY8XXX таких контролерів два:
 *
 *     0xFFE4C000  SDHI0  — передній слот, «Main Slot SD Card»  (DSK1)
 *     0xFFE4D000  SDHI1  — другий слот,  «Sub Slot SD Card»    (DSK11)
 *
 * Імена слотів узято з реєстру самого пристрою
 * (SYSTEM\StorageManager\Profiles\SDProfile / SDProfile2), адреси — з
 * r8a7778.dtsi (mmc@ffe4c000, mmc@ffe4d000) і з живої траси QEMU, де до
 * них звертається `SDHC.dll` (vbase 0xefa20000).
 *
 * --- Звідки взята семантика регістрів -------------------------------------
 *
 * Не з даташита, а з коду драйвера `SDHC.dll` (дизасембльована секція 0,
 * VA 0xefa21000). Доведені точки:
 *
 *   0xefa30dd4  WaitForSclkDivEn:  ldrh r3,[base+0x1e]; tst r3,#0x2000
 *               — дві спроби з паузою 1 мс, інакше помилка 0xFFFFFBB2
 *               (mvn r3,#0x400; eor r0,r3,#0x4d @0xefa30f54).
 *   0xefa30e20  Init: (rd16(0x20) & 0x0018) | 0x0305 -> 0x20,
 *               0x8b7f -> 0x22, 0xc007 -> 0x38, 0 -> 0xd8.
 *   0xefa30654  Reset: 0 -> 0xe0, пауза, 1 -> 0xe0.
 *   0xefa30f28  SetClock: після WaitForSclkDivEn править 0x24
 *               (біт 8 = SCLKEN, біти 7..0 = дільник; 0xFF = особливий).
 *   0xefa307f0  (rd16(0x28) & 0xFFF0) | TOP  — поле таймауту SD_OPTION;
 *   0xefa3081c  (rd16(0x28) & 0x00FF) | 0x8000 — ширина шини 1 біт.
 *   0xefa318d8  WaitEvent(mask32): опитує 0x1c (молодші 16) і 0x1e (старші
 *               16), знайдене гасить записом ~bits, причому в 0x1e завжди
 *               лишає 0x0800 — тобто цей біт апаратно завжди одиниця.
 *   0xefa31c00  Команда: 0x0a=лічильник блоків, 0x08=0x100 (SEC),
 *               0x26=довжина блоку, 0x04/0x06=аргумент, 0x00=SD_CMD
 *               (біти 5..0 індекс, 6 ACMD, 11 дані, 12 читання, 13 мульти).
 *   0xefa32340/0xefa32534  PIO через 0x30, по 16 бітів.
 *
 * --- Межа чесності --------------------------------------------------------
 *
 * Контролер тут СПРАВЖНІЙ: він виконує команди через шину QEMU sd-bus, а не
 * підробляє успіх. Зокрема SD_INFO2.SCLKDIVEN не «завжди 1»: цей біт
 * виводиться зі стану контролера (немає незавершеної передачі), рівно як і
 * CMD_BUSY та DAT0. Порожній слот поводиться як порожній: команда не
 * отримує відповіді й контролер виставляє CMDTIMEOUT.
 *
 * Reset-значення регістрів даташитом не підтверджені. Там, де драйвер
 * робить RMW, обрано безпечне значення, і це позначено біля масиву.
 *
 * Лінія переривання виведена з моделі, але плата її поки НЕ під'єднує:
 * номер GIC SPI для SDHI не встановлено жодним доказом, а помилкова лінія
 * дала б шторм. Драйвер це переживає — свою основну гілку очікування
 * (0xefa318d8, режим 1) він крутить опитуванням зі Sleep(10).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "hw/core/sysbus.h"
#include "hw/sd/sd.h"
#include "hw/sd/renesas_sdhi.h"
#include "hw/dma/clarion_hpbdma.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "trace.h"

#define TYPE_RENESAS_SDHI_BUS "renesas-sdhi-bus"
DECLARE_INSTANCE_CHECKER(SDBus, RENESAS_SDHI_BUS, TYPE_RENESAS_SDHI_BUS)

OBJECT_DECLARE_SIMPLE_TYPE(RenesasSDHIState, RENESAS_SDHI)

/* --- регістри (16-бітні, як у TMIO) ----------------------------------- */

#define SD_CMD              0x00
#define SD_PORTSEL          0x02
#define SD_ARG0             0x04
#define SD_ARG1             0x06
#define SD_STOP             0x08
#define SD_SECCNT           0x0a
#define SD_RSP0             0x0c    /* 0x0c..0x1a — вісім слів відповіді */
#define SD_RSP7             0x1a
#define SD_INFO1            0x1c
#define SD_INFO2            0x1e
#define SD_INFO1_MASK       0x20
#define SD_INFO2_MASK       0x22
#define SD_CLK_CTRL         0x24
#define SD_SIZE             0x26
#define SD_OPTION           0x28
#define SD_ERR_STS1         0x2c
#define SD_ERR_STS2         0x2e
#define SD_BUF0             0x30
#define SDIO_MODE           0x34
#define SDIO_INFO1          0x36
#define SDIO_INFO1_MASK     0x38
#define CC_EXT_MODE         0xd8
#define SD_SOFT_RST         0xe0
#define SD_VERSION          0xe2
#define SD_HOST_MODE        0xe4
#define SD_SDIF_MODE        0xe6

/* SD_CMD */
#define CMD_IDX             0x003f
#define CMD_ACMD            0x0040
#define CMD_RSPTP           0x0700
#define CMD_RSPTP_NONE      0x0300
#define CMD_DATA            0x0800
#define CMD_READ            0x1000
#define CMD_MULTI           0x2000
#define CMD_NOAUTOSTOP      0x4000      /* 0 = controller sends CMD12 itself */

/* SD_STOP */
#define STOP_STP            0x0001
#define STOP_SEC            0x0100

/* SD_INFO1 — біти, що гасяться записом нуля */
#define INFO1_RESPEND       0x0001
#define INFO1_ACCEND        0x0004
#define INFO1_REMOVE        0x0008
#define INFO1_INSERT        0x0010
#define INFO1_SIGSTATE      0x0020      /* рівень лінії CD, не подія */
#define INFO1_WRPROTECT     0x0080      /* рівень лінії WP, не подія */
#define INFO1_REMOVE_A      0x0100
#define INFO1_INSERT_A      0x0200
#define INFO1_SIGSTATE_A    0x0400      /* рівень, не подія */

#define INFO1_W0C   (INFO1_RESPEND | INFO1_ACCEND | INFO1_REMOVE | \
                     INFO1_INSERT | INFO1_REMOVE_A | INFO1_INSERT_A)
#define INFO1_IRQ   INFO1_W0C

/* SD_INFO2 */
#define INFO2_CMD_IDX_ERR   0x0001
#define INFO2_CRCFAIL       0x0002
#define INFO2_STOPBIT_ERR   0x0004
#define INFO2_DATATIMEOUT   0x0008
#define INFO2_RXOVERFLOW    0x0010
#define INFO2_TXUNDERRUN    0x0020
#define INFO2_CMDTIMEOUT    0x0040
#define INFO2_DAT0          0x0080      /* рівень DAT0: 1 = картка вільна */
#define INFO2_RXRDY         0x0100
#define INFO2_TXRQ          0x0200
#define INFO2_ALWAYS        0x0800      /* апаратно завжди 1 (див. вище) */
#define INFO2_SCLKDIVEN     0x2000      /* дільник такту можна міняти */
#define INFO2_CMD_BUSY      0x4000
#define INFO2_ILL_ACCESS    0x8000

#define INFO2_ERRORS    (INFO2_CMD_IDX_ERR | INFO2_CRCFAIL | \
                         INFO2_STOPBIT_ERR | INFO2_DATATIMEOUT | \
                         INFO2_RXOVERFLOW | INFO2_TXUNDERRUN | \
                         INFO2_CMDTIMEOUT)
#define INFO2_W0C       (INFO2_ERRORS | INFO2_RXRDY | INFO2_TXRQ | \
                         INFO2_ILL_ACCESS)
/*
 * Джерела переривання — лише події. DAT0, ALWAYS, SCLKDIVEN і CMD_BUSY це
 * рівні: вони ніколи не піднімають лінію, хоч драйвер і лишає їх
 * незамаскованими (SD_INFO2_MASK = 0x8b7f гасить біт 11, але не 13/14).
 */
#define INFO2_IRQ       INFO2_W0C

/* SD_CLK_CTRL */
#define CLK_SCLKEN          0x0100

/* SD_SOFT_RST: біт 0 = 0 тримає контролер у скиданні */
#define SOFT_RST_RELEASE    0x0001

/* CC_EXT_MODE: біт 1 = передача даних через DMA, а не через SD_BUF0 */
#define CC_EXT_DMASDRW      0x0002

#define SDHI_MAX_BLOCK      0x800

struct RenesasSDHIState {
    SysBusDevice parent_obj;

    SDBus sdbus;
    MemoryRegion iomem;
    qemu_irq irq;

    uint8_t unit;               /* номер контролера, лише для трас */

    /*
     * Передача даних через DMA (CC_EXT_MODE.DMASDRW). На цій платі за неї
     * відповідає HPB-DMAC, і зв'язок однобічний: контролер лише каже йому
     * «в буфері є дані». Читає їх DMAC сам — звичайним читанням SD_BUF0,
     * тобто рівно тим самим шляхом, що й PIO. Без цих двох властивостей
     * модель поводиться як раніше (PIO і чесний лог про невідомий DMA).
     */
    DeviceState *dmac;
    uint64_t dma_buf_addr;      /* фізична адреса власного SD_BUF0 */
    char *read_log_path;
    uint32_t read_sector;
    uint32_t read_index;
    uint32_t read_total;

    uint16_t cmd, portsel, arg0, arg1, stop, seccnt;
    uint16_t rsp[8];
    uint16_t info1, info2;
    uint16_t info1_mask, info2_mask;
    uint16_t clk_ctrl, size, option;
    uint16_t err_sts1, err_sts2;
    uint16_t sdio_mode, sdio_info1, sdio_info1_mask;
    uint16_t cc_ext_mode, soft_rst, host_mode, sdif_mode;

    /* стан передачі даних (PIO через SD_BUF0) */
    uint8_t  buf[SDHI_MAX_BLOCK];
    uint32_t blen;              /* довжина блоку, байтів */
    uint32_t pos;               /* скільки байтів блоку вже пройшло */
    uint32_t blocks;            /* скільки блоків лишилося, разом із цим */
    bool     reading, writing;
};

/* --- допоміжне -------------------------------------------------------- */

static bool sdhi_idle(RenesasSDHIState *s)
{
    return !s->reading && !s->writing;
}

static bool sdhi_card_present(RenesasSDHIState *s)
{
    return sdbus_get_inserted(&s->sdbus);
}

/* SD_INFO1 з доданими рівнями ліній */
static uint16_t sdhi_info1_live(RenesasSDHIState *s)
{
    uint16_t v = s->info1 & INFO1_W0C;

    if (sdhi_card_present(s)) {
        v |= INFO1_SIGSTATE | INFO1_SIGSTATE_A;
        if (sdbus_get_readonly(&s->sdbus)) {
            v |= INFO1_WRPROTECT;
        }
    }
    return v;
}

/* SD_INFO2 з доданими рівнями */
static uint16_t sdhi_info2_live(RenesasSDHIState *s)
{
    uint16_t v = (s->info2 & INFO2_W0C) | INFO2_ALWAYS;

    if (sdhi_idle(s)) {
        /*
         * Незавершеної передачі немає — дільник такту можна міняти, шина
         * не зайнята. Саме на цей біт чекає WaitForSclkDivEn.
         */
        v |= INFO2_SCLKDIVEN;
        if (sdhi_card_present(s)) {
            v |= INFO2_DAT0;
        }
    } else {
        v |= INFO2_CMD_BUSY;
    }
    return v;
}

static void sdhi_update_irq(RenesasSDHIState *s)
{
    int level = ((sdhi_info1_live(s) & INFO1_IRQ & ~s->info1_mask) ||
                 (sdhi_info2_live(s) & INFO2_IRQ & ~s->info2_mask) ||
                 (s->sdio_info1 & ~s->sdio_info1_mask & 0x0001));

    trace_renesas_sdhi_irq(s->unit, level);
    qemu_set_irq(s->irq, level);
}

static void sdhi_abort_data(RenesasSDHIState *s)
{
    s->reading = s->writing = false;
    s->pos = 0;
    s->blocks = 0;
    s->info2 &= ~(INFO2_RXRDY | INFO2_TXRQ);
}

/* --- команди ---------------------------------------------------------- */

/*
 * Драйвер майже завжди лишає RSPTP = 000 («автоматично»), тобто тип
 * відповіді визначає саме контролер за індексом команди. Тут — той самий
 * перелік команд без відповіді, що й у специфікації SD.
 */
static bool sdhi_cmd_has_response(unsigned idx)
{
    switch (idx) {
    case 0:     /* GO_IDLE_STATE */
    case 4:     /* SET_DSR */
    case 15:    /* GO_INACTIVE_STATE */
        return false;
    default:
        return true;
    }
}

static void sdhi_fill_block(RenesasSDHIState *s)
{
    bool ready = sdbus_data_ready(&s->sdbus);

    memset(s->buf, 0, s->blen);
    if (ready) {
        sdbus_read_data(&s->sdbus, s->buf, s->blen);
    } else {
        /* картка даних не дає — це обрив передачі, а не тиша */
        s->info2 |= INFO2_DATATIMEOUT;
    }

    if (s->read_log_path && *s->read_log_path &&
        ((s->cmd & CMD_IDX) == 17 || (s->cmd & CMD_IDX) == 18)) {
        FILE *file = fopen(s->read_log_path, "a");
        bool all_zero = ready;

        for (uint32_t i = 0; i < s->blen; i++) {
            all_zero &= s->buf[i] == 0;
        }
        if (file) {
            fprintf(file,
                    "{\"unit\":%u,\"cmd\":%u,\"block\":%u,"
                    "\"block_index\":%u,\"blocks\":%u,"
                    "\"bytes\":%u,\"ready\":%s,\"all_zero\":%s,"
                    "\"result\":\"%s\"}\n",
                    s->unit, s->cmd & CMD_IDX, s->read_sector,
                    s->read_index, s->read_total, s->blen,
                    ready ? "true" : "false",
                    all_zero ? "true" : "false",
                    ready ? "ok" : "not-ready");
            fclose(file);
        }
    }
    if (!ready) {
        sdhi_abort_data(s);
        return;
    }

    s->read_sector++;
    s->read_index++;
    s->pos = 0;
    s->info2 |= INFO2_RXRDY;
}

/*
 * Запит DMA від контролера: «в буфері просто зараз є що віддати».
 *
 * Це саме запит, а не дозвіл: DMAC питає його перед кожною одиницею
 * передачі й читає SD_BUF0 рівно доти, доки він істинний. Тому модель не
 * тримає жодного лічильника поза самим буфером і не може віддати байтів
 * більше, ніж картка справді дала.
 */
static bool sdhi_dma_ready(void *opaque)
{
    RenesasSDHIState *s = opaque;

    return s->dmac && s->reading && (s->cc_ext_mode & CC_EXT_DMASDRW) &&
           s->pos < s->blen;
}

static void sdhi_start_data(RenesasSDHIState *s)
{
    s->blen = s->size ? s->size : 512;
    if (s->blen > SDHI_MAX_BLOCK) {
        qemu_log_mask(LOG_GUEST_ERROR, "renesas-sdhi: довжина блоку %u > %u\n",
                      s->blen, SDHI_MAX_BLOCK);
        s->blen = SDHI_MAX_BLOCK;
    }
    s->blocks = (s->stop & STOP_SEC) ? s->seccnt : 1;
    if (!s->blocks) {
        s->blocks = 1;
    }
    s->pos = 0;

    if ((s->cc_ext_mode & CC_EXT_DMASDRW) && !s->dmac) {
        /*
         * DMA просять, а контролера DMA нам не під'єднали (машина без
         * HPB-DMAC або властивість не задано). Мовчки вдавати успіх не
         * можна: даних ніхто не перенесе. Драйвер свій PIO-шлях
         * (0xefa32340) має завжди, тож далі йдемо ним.
         */
        qemu_log_mask(LOG_UNIMP, "renesas-sdhi: передачу через DMA просять, "
                      "але DMAC не під'єднано (CC_EXT_MODE=%#x)\n",
                      s->cc_ext_mode);
    }

    if (s->cmd & CMD_READ) {
        s->reading = true;
        sdhi_fill_block(s);
        /*
         * Блок уже в буфері, а канал DMAC на цій платі озброюють ПІСЛЯ
         * команди (траса: DCR/DSAR/DDAR/DTCR -> CMD17 -> DCMDR.DMEN).
         * Тож штовхаємо DMAC і на випадок зворотного порядку: якщо канал
         * уже озброєно, він забере блок тут і зараз, якщо ні — забере на
         * DMEN, спитавши sdhi_dma_ready().
         */
        if (sdhi_dma_ready(s)) {
            clarion_hpbdma_module_poke(s->dmac, s->dma_buf_addr);
        }
    } else {
        s->writing = true;
        s->info2 |= INFO2_TXRQ;
    }
}

static void sdhi_send_command(RenesasSDHIState *s)
{
    unsigned idx = s->cmd & CMD_IDX;
    SDRequest req;
    uint8_t rsp[16];
    size_t rlen;
    bool want_rsp;
    int i;

    if (!(s->soft_rst & SOFT_RST_RELEASE)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "renesas-sdhi: CMD%u під час скидання\n", idx);
        return;
    }

    sdhi_abort_data(s);
    s->info2 &= ~INFO2_ERRORS;
    memset(s->rsp, 0, sizeof(s->rsp));

    req.cmd = idx;
    req.arg = ((uint32_t)s->arg1 << 16) | s->arg0;
    if ((s->cmd & (CMD_DATA | CMD_READ)) == (CMD_DATA | CMD_READ)) {
        s->read_sector = req.arg;
        s->read_index = 0;
        s->read_total = (s->stop & STOP_SEC) ? s->seccnt : 1;
        if (!s->read_total) {
            s->read_total = 1;
        }
    }
    trace_renesas_sdhi_command(s->unit, idx, req.arg, s->cmd);

    want_rsp = (s->cmd & CMD_RSPTP) != CMD_RSPTP_NONE &&
               sdhi_cmd_has_response(idx);

    if (!sdhi_card_present(s)) {
        /* порожній слот: на шині нікому відповідати */
        if (want_rsp) {
            s->info2 |= INFO2_CMDTIMEOUT;
        } else {
            s->info1 |= INFO1_RESPEND;
        }
        sdhi_update_irq(s);
        return;
    }

    rlen = sdbus_do_command(&s->sdbus, &req, rsp, sizeof(rsp));

    if (rlen == 4) {
        uint32_t v = ldl_be_p(&rsp[0]);

        s->rsp[0] = v & 0xffff;
        s->rsp[1] = v >> 16;
    } else if (rlen == 16) {
        /*
         * R2: the registers hold bits [127:8] without the CRC, right-aligned;
         * tmio_mmc_cmd_irq() shifts them back left by 8 bits.
         */
        uint8_t r2[16] = { 0 };

        memcpy(&r2[1], rsp, 15);
        for (i = 0; i < 4; i++) {
            uint32_t v = ldl_be_p(&r2[12 - 4 * i]);

            s->rsp[2 * i] = v & 0xffff;
            s->rsp[2 * i + 1] = v >> 16;
        }
    } else if (want_rsp) {
        trace_renesas_sdhi_timeout(s->unit, idx);
        s->info2 |= INFO2_CMDTIMEOUT;
        sdhi_update_irq(s);
        return;
    }

    s->info1 |= INFO1_RESPEND;

    if (s->cmd & CMD_DATA) {
        sdhi_start_data(s);
    }
    sdhi_update_irq(s);
}

/* --- порт даних ------------------------------------------------------- */

static void sdhi_block_done(RenesasSDHIState *s)
{
    s->blocks--;
    if (s->blocks) {
        if (s->reading) {
            sdhi_fill_block(s);
        } else {
            s->pos = 0;
        }
        return;
    }
    if ((s->cmd & CMD_MULTI) && (s->stop & STOP_SEC) &&
        !(s->cmd & CMD_NOAUTOSTOP)) {
        SDRequest req = { .cmd = 12, .arg = 0 };
        uint8_t rsp[16];

        sdbus_do_command(&s->sdbus, &req, rsp, sizeof(rsp));
    }
    sdhi_abort_data(s);
    s->info1 |= INFO1_ACCEND;
}

static uint64_t sdhi_buf_read(RenesasSDHIState *s, unsigned size)
{
    uint64_t v = 0;
    unsigned i;

    if (!s->reading) {
        s->info2 |= INFO2_ILL_ACCESS;
        sdhi_update_irq(s);
        return 0;
    }
    for (i = 0; i < size; i++) {
        v |= (uint64_t)s->buf[s->pos++] << (8 * i);
        if (s->pos >= s->blen) {
            sdhi_block_done(s);
            break;
        }
    }
    sdhi_update_irq(s);
    return v;
}

static void sdhi_buf_write(RenesasSDHIState *s, uint64_t val, unsigned size)
{
    unsigned i;

    if (!s->writing) {
        s->info2 |= INFO2_ILL_ACCESS;
        sdhi_update_irq(s);
        return;
    }
    for (i = 0; i < size; i++) {
        s->buf[s->pos++] = (val >> (8 * i)) & 0xff;
        if (s->pos >= s->blen) {
            sdbus_write_data(&s->sdbus, s->buf, s->blen);
            sdhi_block_done(s);
            break;
        }
    }
    sdhi_update_irq(s);
}

/* --- MMIO ------------------------------------------------------------- */

static uint64_t sdhi_read(void *opaque, hwaddr addr, unsigned size)
{
    RenesasSDHIState *s = opaque;
    uint64_t v;

    if (addr == SD_BUF0) {
        return sdhi_buf_read(s, size);
    }

    switch (addr) {
    case SD_CMD:            v = s->cmd;             break;
    case SD_PORTSEL:        v = s->portsel;         break;
    case SD_ARG0:           v = s->arg0;            break;
    case SD_ARG1:           v = s->arg1;            break;
    case SD_STOP:           v = s->stop;            break;
    case SD_SECCNT:         v = s->seccnt;          break;
    case SD_RSP0 ... SD_RSP7:
        v = s->rsp[(addr - SD_RSP0) / 2];
        break;
    case SD_INFO1:          v = sdhi_info1_live(s); break;
    case SD_INFO2:          v = sdhi_info2_live(s); break;
    case SD_INFO1_MASK:     v = s->info1_mask;      break;
    case SD_INFO2_MASK:     v = s->info2_mask;      break;
    case SD_CLK_CTRL:       v = s->clk_ctrl;        break;
    case SD_SIZE:           v = s->size;            break;
    case SD_OPTION:         v = s->option;          break;
    case SD_ERR_STS1:       v = s->err_sts1;        break;
    case SD_ERR_STS2:       v = s->err_sts2;        break;
    case SDIO_MODE:         v = s->sdio_mode;       break;
    case SDIO_INFO1:        v = s->sdio_info1;      break;
    case SDIO_INFO1_MASK:   v = s->sdio_info1_mask; break;
    case CC_EXT_MODE:       v = s->cc_ext_mode;     break;
    case SD_SOFT_RST:       v = s->soft_rst;        break;
    case SD_VERSION:        v = 0;                  break;
    case SD_HOST_MODE:      v = s->host_mode;       break;
    case SD_SDIF_MODE:      v = s->sdif_mode;       break;
    default:
        qemu_log_mask(LOG_UNIMP, "renesas-sdhi: читання невідомого "
                      "регістра +%#" HWADDR_PRIx "\n", addr);
        v = 0;
        break;
    }
    trace_renesas_sdhi_read(s->unit, addr, size, v);
    return v;
}

static void sdhi_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    RenesasSDHIState *s = opaque;

    trace_renesas_sdhi_write(s->unit, addr, size, val);

    if (addr == SD_BUF0) {
        sdhi_buf_write(s, val, size);
        return;
    }

    switch (addr) {
    case SD_CMD:
        s->cmd = val;
        sdhi_send_command(s);
        break;
    case SD_PORTSEL:        s->portsel = val;   break;
    case SD_ARG0:           s->arg0 = val;      break;
    case SD_ARG1:           s->arg1 = val;      break;
    case SD_STOP:
        s->stop = val;
        if (val & STOP_STP) {
            /* примусова зупинка передачі */
            sdhi_abort_data(s);
            sdhi_update_irq(s);
        }
        break;
    case SD_SECCNT:         s->seccnt = val;    break;
    case SD_INFO1:
        /* запис нуля гасить біт, запис одиниці лишає як є */
        s->info1 &= (val | ~INFO1_W0C);
        sdhi_update_irq(s);
        break;
    case SD_INFO2:
        s->info2 &= (val | ~INFO2_W0C);
        sdhi_update_irq(s);
        break;
    case SD_INFO1_MASK:
        s->info1_mask = val;
        sdhi_update_irq(s);
        break;
    case SD_INFO2_MASK:
        s->info2_mask = val;
        sdhi_update_irq(s);
        break;
    case SD_CLK_CTRL:       s->clk_ctrl = val;  break;
    case SD_SIZE:           s->size = val;      break;
    case SD_OPTION:         s->option = val;    break;
    case SDIO_MODE:         s->sdio_mode = val; break;
    case SDIO_INFO1:
        s->sdio_info1 &= val;
        sdhi_update_irq(s);
        break;
    case SDIO_INFO1_MASK:
        s->sdio_info1_mask = val;
        sdhi_update_irq(s);
        break;
    case CC_EXT_MODE:       s->cc_ext_mode = val;   break;
    case SD_SOFT_RST:
        s->soft_rst = val;
        if (!(val & SOFT_RST_RELEASE)) {
            /* контролер у скиданні: стан передачі й події зникають */
            sdhi_abort_data(s);
            s->info1 = 0;
            s->info2 = 0;
            sdhi_update_irq(s);
        }
        break;
    case SD_HOST_MODE:      s->host_mode = val; break;
    case SD_SDIF_MODE:      s->sdif_mode = val; break;
    case SD_RSP0 ... SD_RSP7:
    case SD_ERR_STS1:
    case SD_ERR_STS2:
    case SD_VERSION:
        qemu_log_mask(LOG_GUEST_ERROR, "renesas-sdhi: запис у регістр лише "
                      "для читання +%#" HWADDR_PRIx "\n", addr);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "renesas-sdhi: запис у невідомий регістр "
                      "+%#" HWADDR_PRIx " = %#" PRIx64 "\n", addr, val);
        break;
    }
}

static const MemoryRegionOps sdhi_ops = {
    .read = sdhi_read,
    .write = sdhi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    /*
     * Усі регістри 16-бітні, і драйвер ходить саме півсловами. Однобайтовий
     * доступ приймаємо лише щоб випадкове звернення не дало помилку шини:
     * QEMU складе його з нашого 16-бітного.
     */
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 2,
    .impl.max_access_size = 4,
};

/* --- зміна стану слота ------------------------------------------------ */

static void sdhi_set_inserted(DeviceState *dev, bool inserted)
{
    RenesasSDHIState *s = RENESAS_SDHI(dev);

    trace_renesas_sdhi_inserted(s->unit, inserted);
    s->info1 |= inserted ? (INFO1_INSERT | INFO1_INSERT_A)
                         : (INFO1_REMOVE | INFO1_REMOVE_A);
    if (!inserted) {
        sdhi_abort_data(s);
    }
    sdhi_update_irq(s);
}

/*
 * WRPROTECT у SD_INFO1 — це рівень лінії WP, тож модель бере його прямо з
 * шини (див. sdhi_info1_live). Тут лишається тільки переоцінити лінію.
 */
static void sdhi_set_readonly(DeviceState *dev, bool readonly)
{
    RenesasSDHIState *s = RENESAS_SDHI(dev);

    sdhi_update_irq(s);
}

/* --- QOM -------------------------------------------------------------- */

static void sdhi_reset_hold(Object *obj, ResetType type)
{
    RenesasSDHIState *s = RENESAS_SDHI(obj);

    s->cmd = s->portsel = s->arg0 = s->arg1 = s->stop = s->seccnt = 0;
    memset(s->rsp, 0, sizeof(s->rsp));
    s->info1 = 0;
    s->info2 = 0;
    /*
     * ⚠ Reset-значення масок даташитом не підтверджені. Обрано «усе
     * замасковано»: це єдиний варіант, що не може дати помилкового
     * переривання до того, як драйвер налаштує контролер. Драйвер усе одно
     * приходить до свого значення: (0xffff & 0x0018) | 0x0305 = 0x031d
     * (0xefa30e38), а SD_INFO2_MASK він пише цілим словом 0x8b7f.
     */
    s->info1_mask = 0xffff;
    s->info2_mask = 0xffff;
    /*
     * ⚠ Так само невідомі reset-значення SD_CLK_CTRL, SD_OPTION і SD_SIZE.
     * Нулі тут — не відтворення стану після подачі живлення; драйвер
     * програмує всі три поля до першої передачі.
     */
    s->clk_ctrl = 0;
    s->size = 0;
    s->option = 0;
    s->err_sts1 = s->err_sts2 = 0;
    s->sdio_mode = s->sdio_info1 = 0;
    s->sdio_info1_mask = 0xffff;
    s->cc_ext_mode = 0;
    s->soft_rst = SOFT_RST_RELEASE;
    s->host_mode = s->sdif_mode = 0;
    sdhi_abort_data(s);
    s->blen = 0;
    qemu_set_irq(s->irq, 0);
}

static void sdhi_init(Object *obj)
{
    RenesasSDHIState *s = RENESAS_SDHI(obj);

    qbus_init(&s->sdbus, sizeof(s->sdbus), TYPE_RENESAS_SDHI_BUS,
              DEVICE(s), "sd-bus");
    memory_region_init_io(&s->iomem, obj, &sdhi_ops, s,
                          TYPE_RENESAS_SDHI, RENESAS_SDHI_SIZE);
    /* an SD_CMD write wakes the DMAC, which reads SD_BUF0 inside that write */
    s->iomem.disable_reentrancy_guard = true;
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
}

static const VMStateDescription vmstate_renesas_sdhi = {
    .name = "renesas-sdhi",
    .version_id = 2,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16(cmd, RenesasSDHIState),
        VMSTATE_UINT16(portsel, RenesasSDHIState),
        VMSTATE_UINT16(arg0, RenesasSDHIState),
        VMSTATE_UINT16(arg1, RenesasSDHIState),
        VMSTATE_UINT16(stop, RenesasSDHIState),
        VMSTATE_UINT16(seccnt, RenesasSDHIState),
        VMSTATE_UINT16_ARRAY(rsp, RenesasSDHIState, 8),
        VMSTATE_UINT16(info1, RenesasSDHIState),
        VMSTATE_UINT16(info2, RenesasSDHIState),
        VMSTATE_UINT16(info1_mask, RenesasSDHIState),
        VMSTATE_UINT16(info2_mask, RenesasSDHIState),
        VMSTATE_UINT16(clk_ctrl, RenesasSDHIState),
        VMSTATE_UINT16(size, RenesasSDHIState),
        VMSTATE_UINT16(option, RenesasSDHIState),
        VMSTATE_UINT16(err_sts1, RenesasSDHIState),
        VMSTATE_UINT16(err_sts2, RenesasSDHIState),
        VMSTATE_UINT16(sdio_mode, RenesasSDHIState),
        VMSTATE_UINT16(sdio_info1, RenesasSDHIState),
        VMSTATE_UINT16(sdio_info1_mask, RenesasSDHIState),
        VMSTATE_UINT16(cc_ext_mode, RenesasSDHIState),
        VMSTATE_UINT16(soft_rst, RenesasSDHIState),
        VMSTATE_UINT16(host_mode, RenesasSDHIState),
        VMSTATE_UINT16(sdif_mode, RenesasSDHIState),
        VMSTATE_UINT8_ARRAY(buf, RenesasSDHIState, SDHI_MAX_BLOCK),
        VMSTATE_UINT32(blen, RenesasSDHIState),
        VMSTATE_UINT32(pos, RenesasSDHIState),
        VMSTATE_UINT32(blocks, RenesasSDHIState),
        VMSTATE_BOOL(reading, RenesasSDHIState),
        VMSTATE_BOOL(writing, RenesasSDHIState),
        VMSTATE_UINT32_V(read_sector, RenesasSDHIState, 2),
        VMSTATE_UINT32_V(read_index, RenesasSDHIState, 2),
        VMSTATE_UINT32_V(read_total, RenesasSDHIState, 2),
        VMSTATE_END_OF_LIST()
    }
};

static void sdhi_realize(DeviceState *dev, Error **errp)
{
    RenesasSDHIState *s = RENESAS_SDHI(dev);

    if (s->dmac) {
        if (!s->dma_buf_addr) {
            error_setg(errp, "renesas-sdhi: разом із dmac треба задати "
                             "dma-buf-addr (фізична адреса SD_BUF0)");
            return;
        }
        /*
         * Реєструємо себе як модуль із власним буфером: DMAC питатиме
         * sdhi_dma_ready() і сам читатиме SD_BUF0 за цією адресою.
         */
        clarion_hpbdma_module_attach(s->dmac, s->dma_buf_addr,
                                     sdhi_dma_ready, s);
    }
}

static const Property sdhi_properties[] = {
    DEFINE_PROP_UINT8("unit", RenesasSDHIState, unit, 0),
    /*
     * Контролер DMA плати і фізична адреса власного SD_BUF0. Обидві
     * необов'язкові: без них модель працює як раніше, лише через PIO.
     */
    DEFINE_PROP_LINK("dmac", RenesasSDHIState, dmac, TYPE_DEVICE,
                     DeviceState *),
    DEFINE_PROP_UINT64("dma-buf-addr", RenesasSDHIState, dma_buf_addr, 0),
    DEFINE_PROP_STRING("read-log", RenesasSDHIState, read_log_path),
};

static void sdhi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    device_class_set_props(dc, sdhi_properties);
    dc->realize = sdhi_realize;
    dc->desc = "Renesas SDHI (TMIO) SD/MMC host controller";
    dc->vmsd = &vmstate_renesas_sdhi;
    rc->phases.hold = sdhi_reset_hold;
}

static void sdhi_bus_class_init(ObjectClass *klass, const void *data)
{
    SDBusClass *sbc = SD_BUS_CLASS(klass);

    sbc->set_inserted = sdhi_set_inserted;
    sbc->set_readonly = sdhi_set_readonly;
}

static const TypeInfo renesas_sdhi_types[] = {
    {
        .name           = TYPE_RENESAS_SDHI,
        .parent         = TYPE_SYS_BUS_DEVICE,
        .instance_size  = sizeof(RenesasSDHIState),
        .instance_init  = sdhi_init,
        .class_init     = sdhi_class_init,
    },
    {
        .name           = TYPE_RENESAS_SDHI_BUS,
        .parent         = TYPE_SD_BUS,
        .instance_size  = sizeof(SDBus),
        .class_init     = sdhi_bus_class_init,
    },
};

DEFINE_TYPES(renesas_sdhi_types)
