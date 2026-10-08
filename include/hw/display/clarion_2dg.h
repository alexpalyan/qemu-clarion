/*
 * Clarion 2DG blitter model.
 *
 * Command lists (M2DG format) are executed into guest memory; property
 * "exec" (default on) disables execution, "log" records the lists.
 */
#ifndef HW_DISPLAY_CLARION_2DG_H
#define HW_DISPLAY_CLARION_2DG_H

#include "hw/core/sysbus.h"

#define TYPE_CLARION_2DG "clarion-2dg"
OBJECT_DECLARE_SIMPLE_TYPE(Clarion2DGState, CLARION_2DG)

#endif
