/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_MISC_CLARION_UBLOX_H
#define HW_MISC_CLARION_UBLOX_H

#include "hw/core/qdev.h"
#include "qom/object.h"

#define TYPE_CLARION_UBLOX "clarion-ublox"
OBJECT_DECLARE_SIMPLE_TYPE(ClarionUbloxState, CLARION_UBLOX)

typedef int (*ClarionUbloxSink)(void *opaque, const uint8_t *buf, int len);
void clarion_ublox_set_sink(DeviceState *dev, ClarionUbloxSink fn,
                            void *opaque);
void clarion_ublox_rx_byte(DeviceState *dev, uint8_t byte);
void clarion_ublox_set_position(DeviceState *dev, double lat, double lon,
                                double speed, double course);

#endif
