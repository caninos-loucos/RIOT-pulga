/*
 * Copyright (C) ?
 */

/**
 * @ingroup     tests
 * @{
 *
 * @file
 * @brief       Test application for the BMX160 temperature, pressure, and
 *              humidity sensor driver
 *
 * @author      Kees Bakker <kees@sodaq.com>
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * @}
 */

#include <stdlib.h>
#include <stdio.h>

#include "bmx160_params.h"
#include "bmx160.h"
// #include "bmx160.h"
// #include "xtimer.h"
// #include "fmt.h"

#define MAINLOOP_DELAY  (2)         /* read sensor every 2 seconds */

int main(void)
{
    // bmx160_t dev;
    struct bmi160_dev bmi;
    // i2c_init(BMX160_PARAM_I2C_DEV);

    bmi.i2c_dev = BMX160_PARAM_I2C_DEV;
    bmx160_init(bmi);

    return 0;
}
