/*
 * Clarion shared USB PHY model.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_CLARION_USBPHY_H
#define HW_MISC_CLARION_USBPHY_H

#include "system/memory.h"

void clarion_usbphy_init(MemoryRegion *sysmem, hwaddr base, int priority,
                         const char *name);

#endif
