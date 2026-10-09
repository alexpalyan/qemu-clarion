/*
 * LBSC DMAC — DMA-двигун читання паралельної NOR плати Clarion QY8XXX
 * (WinCE 7.0).
 *
 * Навіщо. `Flash.dll` не читає NOR послідовністю команд CFI. IOCTL
 * `0x01112020` (`CFlashMain::FlashReadToPhysMem`) обчислює фізичний зсув у
 * мікросхемі й доручає перенесення окремому двигуну: той сам читає масив NOR
 * і пише в ОЗП за фізичною адресою. Поки блок не змодельовано, запити
 * ковтає широкий перехоплювач плати, у призначенні лишаються нулі, а
 * `CFlashDmacCtl::Transfer` усе одно віддає TRUE — бо результат очікування
 * завершення там просто відкидається. Наслідок у гості: порожня версія NK2,
 * `Version Error` і, через 5-секундний таймаут очікування переривання,
 * гонка, що валить AppLaunch у `memcpy` з нульового джерела.
 *
 * Звідки взято мапу регістрів. Не з аналогії з чужим SoC, а з коду самого
 * драйвера (`Flash.dll`, vbase 0xEFA00000) і з живого прогону: драйвер
 * тримає масив ВКАЗІВНИКІВ на регістри, і цей масив знято з гостя
 * (probe tools/qy8_flash_dma_probe.py репозиторію nissan-can-explore,
 * структура @0xC483D310):
 *
 *   +0x00 = 0            номер каналу       -> біт 1<<0 у 0x410/0x414/0x418
 *   +0x04 = 2            режим (гілка «не 1» у dmac_IsFinished)
 *   +0x10..+0x18 -> 0x00/0x04/0x08   набір 0 {SAR, DAR, TCR}
 *   +0x1c..+0x24 -> 0x0c/0x10/0x14   набір 1
 *   +0x28..+0x30 -> 0x18/0x1c/0x20   набір 2
 *   +0x38 -> 0x28        CTRL   (`CFlashDmacCtl::Transfer` пише 0x00203515)
 *   +0x3c -> 0x2c        START  (запис 1)
 *   +0x40 -> 0x30        STOP   (запис 1)
 *   +0x44 -> 0x34        STATUS (біт 0 = «зайнятий»)
 *   +0x5c -> 0x410       INTSTAT (читання, біт на канал)
 *   +0x68 -> 0x414       INTCLR  (запис 1 у біт каналу)
 *   +0x74 -> 0x418       INTMASK (дозвіл переривання, біт на канал)
 *
 * Порядок, який виконує прошивка (перевірено `-d unimp` на чистому буті):
 *
 *   SAR <- 0x00AC0000 ; DAR <- 0x70196000 ; TCR <- 0x20
 *   CTRL <- 0x00203515
 *   ResetEvent(m_hFlashInterEvent)
 *   INTMASK |= 1                      ; @0xEFA0F590
 *   START <- 1                        ; @0xEFA0F514, і лише тепер іде обмін
 *   WaitForSingleObject(подія, 5000)  ; @0xEFA0E9F0 FlashDmacIsFinish
 *   ... переривання -> демукс OAL знімає INTMASK біт 0, дає SYSINTR 0x13 ...
 *   InterruptDone
 *   STOP <- 1 ; чекати STATUS.біт0 == 0 (до 100 мс)   ; @0xEFA0E9AC
 *   перевірити INTSTAT.біт0, записати 1 у INTCLR      ; @0xEFA0F8D8
 *
 * Саме тому модель піднімає переривання НА ЗАПИС У START, а не раніше:
 * дозвіл переривання гість виставляє між CTRL і START, і рівнева логіка
 * `INTSTAT & INTMASK` відтворює це без жодних припущень про порядок.
 *
 * Що модель робить. На запис 1 у START переносить `TCR * 16` байтів із
 * `SAR` у `DAR` через фізичний простір (тобто реальні байти NOR, які віддає
 * модель мікросхеми), просуває SAR/DAR і обнуляє TCR, знімає «зайнятий» і
 * виставляє біт каналу в INTSTAT. Жодних знань про блок 0x56, адресу
 * 0xAC0000, заголовок NK2 чи AppLaunch тут немає — лише те, що гість сам
 * поклав у регістри.
 *
 * Чого свідомо немає (джерела не кажуть — не вигадуємо):
 *   CTRL (0x28): значення 0x00203515 зберігається, але не тлумачиться —
 *       які там поля, з коду драйвера не видно (він пише константу);
 *   0x400/0x404/0x40c: драйвер тримає на них вказівники, але в режимі 2
 *       (наш) не читає жодного разу;
 *   набори 1 і 2: прошивка на шляху читання користується лише набором 0;
 *       модель обслуговує будь-який набір із ненульовим TCR, бо це поведінка
 *       контролера, а не здогад про призначення наборів;
 *   канали, крім 0: сторінок інших каналів гість не чіпає;
 *   реальна тривалість обміну — переносимо все за один такт.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/dma/clarion_lbdma.h"
#include "system/address-spaces.h"
#include "qom/object.h"
#include "trace.h"

/* Зсуви канальної сторінки. */
#define LBDMA_SAR(i)        (0x00 + (i) * 0x0C)
#define LBDMA_DAR(i)        (0x04 + (i) * 0x0C)
#define LBDMA_TCR(i)        (0x08 + (i) * 0x0C)
#define LBDMA_CTRL          0x28
#define LBDMA_START         0x2C
#define LBDMA_STOP          0x30
#define LBDMA_STATUS        0x34
#define LBDMA_INTSTAT       0x410
#define LBDMA_INTCLR        0x414
#define LBDMA_INTMASK       0x418

#define LBDMA_STATUS_BUSY   (1u << 0)

/*
 * Сторінка 0xFF801000 — це канал 0: драйвер бере номер каналу з власної
 * структури (поле +0x00 = 0) і ним же індексує біти 0x410/0x414/0x418.
 */
#define LBDMA_CH            0
#define LBDMA_CH_BIT        (1u << LBDMA_CH)

struct ClarionLbDmaState {
    SysBusDevice parent_obj;

    MemoryRegion mr;
    qemu_irq irq;

    uint32_t sar[CLARION_LBDMA_NDESC];
    uint32_t dar[CLARION_LBDMA_NDESC];
    uint32_t tcr[CLARION_LBDMA_NDESC];
    uint32_t ctrl;
    uint32_t status;
    uint32_t intstat;
    uint32_t intmask;
};
typedef struct ClarionLbDmaState ClarionLbDmaState;

OBJECT_DECLARE_SIMPLE_TYPE(ClarionLbDmaState, CLARION_LBDMA)

static void lbdma_update_irq(ClarionLbDmaState *s)
{
    qemu_set_irq(s->irq, (s->intstat & s->intmask) != 0);
}

/* Один набір {SAR, DAR, TCR}. Повертає перенесені байти. */
static uint32_t lbdma_run_desc(ClarionLbDmaState *s, int i)
{
    uint8_t buf[256];
    uint32_t left = s->tcr[i] * CLARION_LBDMA_UNIT;
    uint32_t done = 0;

    if (!left) {
        return 0;
    }
    trace_clarion_lbdma_run(i, s->sar[i], s->dar[i], s->tcr[i], left);

    while (left) {
        uint32_t n = MIN(left, sizeof(buf));

        address_space_read(&address_space_memory, s->sar[i] + done,
                           MEMTXATTRS_UNSPECIFIED, buf, n);
        address_space_write(&address_space_memory, s->dar[i] + done,
                            MEMTXATTRS_UNSPECIFIED, buf, n);
        done += n;
        left -= n;
    }

    /*
     * Лічильник вичерпано, адреси просунуто — так поводиться контролер,
     * і так гість не перенесе той самий набір удруге, якщо START прийде
     * ще раз без переініціалізації.
     */
    s->sar[i] += done;
    s->dar[i] += done;
    s->tcr[i] = 0;
    return done;
}

static void lbdma_start(ClarionLbDmaState *s)
{
    uint32_t total = 0;
    int i;

    for (i = 0; i < CLARION_LBDMA_NDESC; i++) {
        total += lbdma_run_desc(s, i);
    }
    trace_clarion_lbdma_start(s->ctrl, total);

    /*
     * Обмін у моделі миттєвий, тож «зайнятий» ніколи не лишається піднятим:
     * драйвер опитує STATUS.біт0 після STOP і чекає саме нуля.
     */
    s->status &= ~LBDMA_STATUS_BUSY;
    s->intstat |= LBDMA_CH_BIT;
    lbdma_update_irq(s);
}

static uint64_t lbdma_read(void *opaque, hwaddr addr, unsigned size)
{
    ClarionLbDmaState *s = opaque;
    int i;

    for (i = 0; i < CLARION_LBDMA_NDESC; i++) {
        if (addr == LBDMA_SAR(i)) {
            return s->sar[i];
        }
        if (addr == LBDMA_DAR(i)) {
            return s->dar[i];
        }
        if (addr == LBDMA_TCR(i)) {
            return s->tcr[i];
        }
    }

    switch (addr) {
    case LBDMA_CTRL:
        return s->ctrl;
    case LBDMA_STATUS:
        return s->status;
    case LBDMA_INTSTAT:
        return s->intstat;
    case LBDMA_INTMASK:
        return s->intmask;
    case LBDMA_START:
    case LBDMA_STOP:
    case LBDMA_INTCLR:
        return 0;               /* лише на запис */
    default:
        qemu_log_mask(LOG_UNIMP,
                      "clarion-lbdma: читання невідомого регістра +0x%03"
                      HWADDR_PRIx "\n", addr);
        return 0;
    }
}

static void lbdma_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    ClarionLbDmaState *s = opaque;
    int i;

    trace_clarion_lbdma_write((uint32_t)addr, val);

    for (i = 0; i < CLARION_LBDMA_NDESC; i++) {
        if (addr == LBDMA_SAR(i)) {
            s->sar[i] = val;
            return;
        }
        if (addr == LBDMA_DAR(i)) {
            s->dar[i] = val;
            return;
        }
        if (addr == LBDMA_TCR(i)) {
            s->tcr[i] = val;
            return;
        }
    }

    switch (addr) {
    case LBDMA_CTRL:
        s->ctrl = val;
        break;
    case LBDMA_START:
        if (val & 1) {
            lbdma_start(s);
        }
        break;
    case LBDMA_STOP:
        if (val & 1) {
            s->status &= ~LBDMA_STATUS_BUSY;
        }
        break;
    case LBDMA_INTCLR:
        s->intstat &= ~(uint32_t)val;
        lbdma_update_irq(s);
        break;
    case LBDMA_INTMASK:
        s->intmask = val;
        lbdma_update_irq(s);
        break;
    case LBDMA_STATUS:
    case LBDMA_INTSTAT:
        break;                  /* лише на читання */
    default:
        qemu_log_mask(LOG_UNIMP,
                      "clarion-lbdma: запис невідомого регістра +0x%03"
                      HWADDR_PRIx " <- 0x%08" PRIx64 "\n", addr, val);
        break;
    }
}

static const MemoryRegionOps lbdma_ops = {
    .read = lbdma_read,
    .write = lbdma_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void clarion_lbdma_reset_hold(Object *obj, ResetType type)
{
    ClarionLbDmaState *s = CLARION_LBDMA(obj);

    memset(s->sar, 0, sizeof(s->sar));
    memset(s->dar, 0, sizeof(s->dar));
    memset(s->tcr, 0, sizeof(s->tcr));
    s->ctrl = 0;
    s->status = 0;
    s->intstat = 0;
    s->intmask = 0;
    lbdma_update_irq(s);
}

static void clarion_lbdma_realize(DeviceState *dev, Error **errp)
{
    ClarionLbDmaState *s = CLARION_LBDMA(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->mr, OBJECT(dev), &lbdma_ops, s,
                          "clarion-lbdma", CLARION_LBDMA_SIZE);
    sysbus_init_mmio(sbd, &s->mr);
    sysbus_init_irq(sbd, &s->irq);
}

static void clarion_lbdma_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = clarion_lbdma_realize;
    dc->desc = "Clarion QY8XXX LBSC NOR DMA";
    rc->phases.hold = clarion_lbdma_reset_hold;
}

static const TypeInfo clarion_lbdma_types[] = {
    {
        .name          = TYPE_CLARION_LBDMA,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(ClarionLbDmaState),
        .class_init    = clarion_lbdma_class_init,
    },
};

DEFINE_TYPES(clarion_lbdma_types)
