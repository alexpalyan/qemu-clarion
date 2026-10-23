/*
 * Clarion QY7 integrated RCAN register block.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_CLARION_RCAN_H
#define HW_MISC_CLARION_RCAN_H

#include "hw/core/irq.h"
#include "system/memory.h"

typedef struct ClarionRcan ClarionRcan;

ClarionRcan *clarion_rcan_init(MemoryRegion *sysmem, hwaddr base, qemu_irq irq);

bool clarion_rcan_inject(ClarionRcan *s, uint16_t id, const uint8_t *data,
                         unsigned len);
void clarion_rcan_set_log(ClarionRcan *s, const char *path);
void clarion_rcan_set_replay(ClarionRcan *s, const char *path);

#endif
