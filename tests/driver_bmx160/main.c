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
#include "ztimer.h"
// #include "fmt.h"

#define MAINLOOP_DELAY  (2)         /* read sensor every 2 seconds */

int main(void)
{
    printf("Will initialize in 2 secs...\n");
    ztimer_sleep(ZTIMER_MSEC, 2000);
    printf("Will initialize now\n");

    struct bmi160_dev bmi; // TODO: change to bmx160_t dev;
    bmi.i2c_dev = BMX160_PARAM_I2C_DEV;
    bmx160_init(bmi);

    return 0;
}
