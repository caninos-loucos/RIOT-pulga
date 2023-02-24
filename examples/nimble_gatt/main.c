/*
 * Copyright (C) 2018 Freie Universität Berlin
 *               2018 Codecoup
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       BLE peripheral example using NimBLE
 *
 * Have a more detailed look at the api here:
 * https://mynewt.apache.org/latest/tutorials/ble/bleprph/bleprph.html
 *
 * More examples (not ready to run on RIOT) can be found here:
 * https://github.com/apache/mynewt-nimble/tree/master/apps
 *
 * Test this application e.g. with Nordics "nRF Connect"-App
 * iOS: https://itunes.apple.com/us/app/nrf-connect/id1054362403
 * Android: https://play.google.com/store/apps/details?id=no.nordicsemi.android.mcp
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Andrzej Kaczmarek <andrzej.kaczmarek@codecoup.pl>
 * @author      Hendrik van Essen <hendrik.ve@fu-berlin.de>
 *
 * @}
 */

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "my_gatt.c"

#include "nimble_riot.h"
#include "nimble_autoadv.h"

#include "thread.h"
#include "shell.h"
#include "ztimer.h"

#include "periph/i2c.h"

#include "bmi160.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "host/ble_gatt.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

/* Macros for frames to be read */

#define ACC_FRAMES	10 /* 10 Frames are available every 100ms @ 100Hz */ //Datasheet says you can take measurementes at 1600 Hz
#define GYR_FRAMES	10
#define MAG_FRAMES	10
/* 10 frames containing a 1 byte header, 6 bytes of accelerometer,
 * 6 bytes of gyroscope and 8 bytes of magnetometer data. This results in
 * 21 bytes per frame. Additional 40 bytes in case sensor time readout is enabled */
#define FIFO_SIZE	250

/* Variable declarations */

struct bmi160_dev bmi;

uint8_t fifo_buff[FIFO_SIZE];
struct bmi160_fifo_frame fifo_frame;
struct bmi160_sensor_data gyro_data[GYR_FRAMES], accel_data[ACC_FRAMES];

int8_t rslt;

typedef struct {
    int16_t accelerometerX;
    int16_t accelerometerY;
    int16_t accelerometerZ;
    uint16_t dummy;
} reading_t;

#define MAX_READINGS 4096
int16_t readings_bufferX[MAX_READINGS];
int16_t readings_bufferY[MAX_READINGS];
int16_t readings_bufferZ[MAX_READINGS];
size_t rlen = 0;

reading_t get_reading(void);
void right_shift_readings_buffer(void);
void do_read(void);
void log_readings(void);
void read_and_show_Acc_values(void);

int dev = I2C_DEV(0);

int8_t user_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    // printf("i2c_read_regs(%d, %x, %x, ...)\n", dev, dev_addr, reg_addr);
    return i2c_read_regs(dev, dev_addr, reg_addr, data, len, 0);
}

int8_t user_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    // printf("i2c_write_regs(%d, %x, %x, ...)\n", dev, dev_addr, reg_addr);
    return i2c_write_regs(dev, dev_addr, reg_addr, data, len, 0);
}

void user_delay(uint32_t period)
{
    ztimer_sleep(ZTIMER_MSEC, period);
}

/* accel params and conversion constants */
#define AC 2048.0 // for 16G
// #define AC 16384.0 // for 2G
/* gyro params and conversion constants */
#define GC 16.4 // for 2000 DPS
// #define GC 131.2 // for 250 DPS

#define GATT_DEVICE_INFO_UUID                   0x180A
#define GATT_MANUFACTURER_NAME_UUID             0x2A29
#define GATT_MODEL_NUMBER_UUID                  0x2A24

#define STR_ANSWER_BUFFER_SIZE 100


int main(void)
{

    (void) puts("Welcome to RIOT!");

    i2c_init(dev);
    i2c_acquire(dev);
    
    /* Initialize your host interface to the BMI160 */

    /* This example uses I2C as the host interface */
    bmi.id = BMI160_I2C_ADDR;
    bmi.read = user_i2c_read;
    bmi.write = user_i2c_write;
    bmi.delay_ms = user_delay;
    bmi.intf = BMI160_I2C_INTF;

    rslt = bmi160_init(&bmi);
    if (rslt == BMI160_OK) {
        printf("Success initializing BMI160 - Chip ID 0x%X\n", bmi.chip_id);
    } else if (rslt == BMI160_E_DEV_NOT_FOUND) {
        printf("Error initializing BMI160: device not found\n");
        return 1;
    } else {
        printf("Error initializing BMI160 - %d\n", rslt);
        return 1;
    }

    /* Select the Output data rate, range of accelerometer sensor */
    bmi.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
    bmi.accel_cfg.range = BMI160_ACCEL_RANGE_16G;
    // bmi.accel_cfg.range = BMI160_ACCEL_RANGE_2G;
    bmi.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;

    /* Select the power mode of accelerometer sensor */
    bmi.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    /* Select the Output data rate, range of Gyroscope sensor */
    bmi.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
    bmi.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    // bmi.gyro_cfg.range = BMI160_GYRO_RANGE_250_DPS;
    bmi.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

    /* Select the power mode of Gyroscope sensor */
    bmi.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

    /* Set the sensor configuration */
    rslt = bmi160_set_sens_conf(&bmi);
    if (rslt != BMI160_OK) {
        printf("Error configuring BMI160 - %d\n", rslt);
        return 1;
    }

    /* Link the FIFO memory location */
    fifo_frame.data = fifo_buff;
    fifo_frame.length = FIFO_SIZE;
    bmi.fifo = &fifo_frame;
    /* Clear all existing FIFO configurations */
    rslt = bmi160_set_fifo_config(BMI160_FIFO_CONFIG_1_MASK , BMI160_DISABLE, &bmi);
    if (rslt != BMI160_OK) {
        printf("Error clearing fifo - %d\n", rslt);
        return 1;
    }

    uint8_t fifo_config = BMI160_FIFO_HEADER |  BMI160_FIFO_ACCEL | BMI160_FIFO_GYRO;
    rslt = bmi160_set_fifo_config(fifo_config, BMI160_ENABLE, &bmi);
    if (rslt != BMI160_OK) {
        printf("Error enabling fifo - %d\n", rslt);
        return 1;
    }
    /* Check rslt for any error codes */
    i2c_release(dev);

    my_gatt_start1();

    while(rslt == 0 ) {
        /* Wait for 100ms for the FIFO to fill */
        user_delay(10);

        /* It is VERY important to reload the length of the FIFO memory as after the
         * call to bmi160_get_fifo_data(), the bmi.fifo->length contains the
         * number of bytes read from the FIFO */
        bmi.fifo->length = FIFO_SIZE;
        i2c_acquire(dev);
        rslt = bmi160_get_fifo_data(&bmi);
        if (rslt != BMI160_OK) {
            printf("Error getting fifo data - %d\n", rslt);
            return 1;
        }
        i2c_release(dev);

        uint8_t acc_inst = ACC_FRAMES;

        rslt = bmi160_extract_accel(accel_data, &acc_inst, &bmi);
        if (rslt != BMI160_OK) {
            printf("Error extracting accel data - %d\n", rslt);
            return 1;
        }

    

        read_and_show_Acc_values();

    }

    //int lenki = sprintf(rm_demo_write_data, "%f, %f, %f", (accel_data[1].x / AC), (accel_data[1].y /AC), (accel_data[1].z /AC));
    //printf("%d \n", lenki);


    return 0;
}


void right_shift_readings_buffer(void)
{
    size_t max = rlen < MAX_READINGS ? rlen++ : MAX_READINGS-1;
    for (size_t i = max; i > 0; i--) {
        readings_bufferX[i] = readings_bufferX[i-ACC_FRAMES];
        readings_bufferY[i] = readings_bufferY[i-ACC_FRAMES];
        readings_bufferZ[i] = readings_bufferZ[i-ACC_FRAMES];
    }
}

void do_read(void)
{
#ifdef PULGA_USE_RINGBUFFER
    right_shift_readings_buffer();
#endif
    for(size_t i=0; i<ACC_FRAMES; i++){
        readings_bufferX[i] = accel_data[i].x;
        readings_bufferY[i] = accel_data[i].y;
        readings_bufferZ[i] = accel_data[i].z;
    }
}

void log_readings(void)
{
#ifdef PULGA_USE_RINGBUFFER
    for (size_t i = 0; i < rlen; i++) {
        printf("[Acc_readings] readings_buffer[%d]: ", i);
        printf("Acc_x: %f ", ((float)readings_bufferX[i])/ AC);
        printf("Acc_y: %f ", ((float)readings_bufferY[i])/ AC);
        printf("Acc_z: %f ", ((float)readings_bufferZ[i])/ AC);
        printf("\n %c", 13);
    }
#else
    for (size_t i = 0; i < ACC_FRAMES; i++) {
        printf("[Acc_readings] readings_buffer[%d]: ", i);
        printf("Acc_x: %f ", ((float)readings_bufferX[i])/ AC);
        printf("Acc_y: %f ", ((float)readings_bufferY[i])/ AC);
        printf("Acc_z: %f ", ((float)readings_bufferZ[i])/ AC);
        printf("\n %c", 13);
    }
#endif
}
void read_and_show_Acc_values(void)
{
    do_read();
//#if(LOG_LEVEL==4)
//    log_readings();
//#endif
}