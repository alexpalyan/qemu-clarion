/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_CHAR_CLARION_SCIF_H
#define HW_CHAR_CLARION_SCIF_H

#include "chardev/char-fe.h"
#include "hw/core/sysbus.h"
#include "qemu/timer.h"

#define CLARION_SCIF_FIFO 16

typedef struct ClarionScif {

    MemoryRegion mr;
    CharFrontend chr;
    qemu_irq irq;
    DeviceState *dmac;
    DeviceState *micom;
    DeviceState *dispmicom;
    DeviceState *ublox;
    hwaddr base;
    uint16_t scsmr, scscr, scfcr;
    uint8_t fifo[CLARION_SCIF_FIFO];
    unsigned fifo_len;
    bool dr;
    QEMUTimer *idle;
    int index;
    bool txi;
} ClarionScif;

void clarion_scif_init(ClarionScif *s, MemoryRegion *sysmem, hwaddr base,
                       const char *name, int index, int serial_index,
                       qemu_irq irq, DeviceState *dmac, bool txi);
int clarion_scif_can_receive(void *opaque);
void clarion_scif_receive(void *opaque, const uint8_t *buf, int size);

#endif
