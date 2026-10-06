#ifndef HW_I2C_CLARION_RCAR_I2C_H
#define HW_I2C_CLARION_RCAR_I2C_H

#include "hw/core/qdev.h"
#include "hw/i2c/i2c.h"

typedef struct Error Error;

#define TYPE_CLARION_RCAR_I2C4 "clarion-rcar-i2c4"
OBJECT_DECLARE_SIMPLE_TYPE(ClarionRcarI2C4State, CLARION_RCAR_I2C4)

#define TYPE_CLARION_I2C4_RECORDER "clarion-i2c4-recorder"
OBJECT_DECLARE_SIMPLE_TYPE(ClarionI2C4Recorder, CLARION_I2C4_RECORDER)

#define TYPE_CLARION_TMA460 "clarion-tma460"
OBJECT_DECLARE_SIMPLE_TYPE(ClarionTma460, CLARION_TMA460)

#define TYPE_CLARION_TMA616 "clarion-tma616"
OBJECT_DECLARE_SIMPLE_TYPE(ClarionTma616, CLARION_TMA616)

/* GPIO4.OUTDT bit 10 is the active-low reset signal observed by this device. */
void clarion_tma460_set_reset(DeviceState *dev, bool gpio_level);
void clarion_tma460_set_synthetic_profile(DeviceState *dev, bool enabled);
void clarion_tma460_bind_pointer_input(DeviceState *dev,
                                      const char *display_id,
                                      Error **errp);
void clarion_tma616_set_reset(DeviceState *dev, bool gpio_level);
void clarion_tma616_bind_pointer_input(DeviceState *dev,
                                       const char *display_id,
                                       Error **errp);

#endif
