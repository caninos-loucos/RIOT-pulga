/*
 * Copyright (C) ?
 */

/**
 * @ingroup     drivers_bmx160
 * @{
 *
 * @file
 * @brief       Device driver implementation for BME160 and BMP160 sensors
 *
 * @author      Geovane Fedrecheski <geonnave@gmail.com>
 *
 * @}
 */

#include <string.h>
#include <math.h>

#include "log.h"
#include "assert.h"
#include "bmx160.h"
#include "bmx160_internals.h"

#define ENABLE_DEBUG        1
#include "debug.h"


int bmx160_init(struct bmi160_dev bmi)
{
    (void) bmi;
    i2c_acquire(bmi.i2c_dev);

    i2c_release(bmi.i2c_dev);
    DEBUG("[bmx160] device initialized\n");
    return 0;
}
