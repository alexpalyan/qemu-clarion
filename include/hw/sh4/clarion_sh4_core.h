/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_SH4_CLARION_SH4_CORE_H
#define HW_SH4_CLARION_SH4_CORE_H

#include "target/sh4/cpu.h"
#include "system/memory.h"

void clarion_sh4_core_init(SuperHCPU *cpu, MemoryRegion *sysmem);

#endif
