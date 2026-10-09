/*
 * Clarion shared USB PHY model.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "system/memory.h"
#include "hw/misc/clarion_usbphy.h"

#define USBPHY_SIZE 0x100
#define USBPCTRL1 0x04
#define USBST 0x08
#define USBPCTRL1_PHY_ENB (1u << 0)
#define USBPCTRL1_PLL_ENB (1u << 1)
#define USBST_ACT (1u << 31)
#define USBST_PLL (1u << 30)

typedef struct ClarionUsbPhy {
    MemoryRegion mr;
    uint32_t reg[USBPHY_SIZE / 4];
} ClarionUsbPhy;

static uint64_t clarion_usbphy_read(void *opaque, hwaddr addr, unsigned size)
{
    ClarionUsbPhy *s = opaque;
    uint32_t ctrl1 = s->reg[USBPCTRL1 / 4];

    if (addr == USBST) {
        if ((ctrl1 & (USBPCTRL1_PHY_ENB | USBPCTRL1_PLL_ENB)) ==
            (USBPCTRL1_PHY_ENB | USBPCTRL1_PLL_ENB)) {
            return USBST_ACT | USBST_PLL;
        }
        return 0;
    }
    return s->reg[addr / 4];
}

static void clarion_usbphy_write(void *opaque, hwaddr addr, uint64_t val,
                                 unsigned size)
{
    ClarionUsbPhy *s = opaque;

    if (addr != USBST) {
        s->reg[addr / 4] = val;
    }
}

static const MemoryRegionOps clarion_usbphy_ops = {
    .read = clarion_usbphy_read,
    .write = clarion_usbphy_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

void clarion_usbphy_init(MemoryRegion *sysmem, hwaddr base, int priority,
                         const char *name)
{
    ClarionUsbPhy *s = g_new0(ClarionUsbPhy, 1);

    memory_region_init_io(&s->mr, NULL, &clarion_usbphy_ops, s, name,
                          USBPHY_SIZE);
    memory_region_add_subregion_overlap(sysmem, base, &s->mr, priority);
}
