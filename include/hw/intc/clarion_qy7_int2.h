/*
 * Clarion QY7 (QY7221NL) interrupt controller at 0xFF804000.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_INTC_CLARION_QY7_INT2_H
#define HW_INTC_CLARION_QY7_INT2_H

#include "hw/core/cpu.h"
#include "hw/core/irq.h"
#include "system/memory.h"

typedef struct ClarionQy7Int2 ClarionQy7Int2;

/*
 * Create the controller at @base and make it the interrupt source of @cpu
 * (CPUSH4State.intc_handle).  The timer group has three request lines
 * (TMU channels 0..2); fetch them with clarion_qy7_int2_timer_irq().
 */
ClarionQy7Int2 *clarion_qy7_int2_init(MemoryRegion *sysmem, hwaddr base,
                                      CPUState *cpu, void **intc_handle);

qemu_irq clarion_qy7_int2_timer_irq(ClarionQy7Int2 *s, int channel);

#endif
