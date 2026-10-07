/*
 * Clarion SH core host-interface model.
 *
 * The model answers the SH initialization-end notification only. It does not
 * implement the SH shared-memory message queues.
 */
#ifndef HW_MISC_CLARION_SHCORE_H
#define HW_MISC_CLARION_SHCORE_H

#include "hw/core/sysbus.h"

#define TYPE_CLARION_SHCORE "clarion-shcore"
OBJECT_DECLARE_SIMPLE_TYPE(ClarionSHCoreState, CLARION_SHCORE)

#endif
