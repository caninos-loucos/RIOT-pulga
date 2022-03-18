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
#include "ztimer.h"

#define ENABLE_DEBUG        1
#include "debug.h"

i2c_t i2c_dev;

int8_t user_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    // DEBUG("i2c_read_regs(%d, %x, %x, ...)\n", i2c_dev, dev_addr, reg_addr);
    return i2c_read_regs(i2c_dev, dev_addr, reg_addr, data, len, 0);
}

int8_t user_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    // DEBUG("i2c_write_regs(%d, %x, %x, ...)\n", i2c_dev, dev_addr, reg_addr);
    return i2c_write_regs(i2c_dev, dev_addr, reg_addr, data, len, 0);
}

void user_delay(uint32_t period)
{
    ztimer_sleep(ZTIMER_MSEC, period);
}



/*!
 * @brief This internal API is used to validate the device structure pointer for
 * null conditions.
 */
static int8_t null_ptr_check(const struct bmi160_dev *dev)
{
    int8_t rslt;

    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_ms == NULL))
    {
        rslt = BMI160_E_NULL_PTR;
    }
    else
    {
        /* Device structure is fine */
        rslt = BMI160_OK;
    }

    return rslt;
}

/*!
 * @brief This API sets the default configuration parameters of accel & gyro.
 * Also maintain the previous state of configurations.
 */
static void default_param_settg(struct bmi160_dev *dev)
{
    /* Initializing accel and gyro params with
     * default values */
    dev->accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
    dev->accel_cfg.odr = BMI160_ACCEL_ODR_100HZ;
    dev->accel_cfg.power = BMI160_ACCEL_SUSPEND_MODE;
    dev->accel_cfg.range = BMI160_ACCEL_RANGE_2G;
    dev->gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
    dev->gyro_cfg.odr = BMI160_GYRO_ODR_100HZ;
    dev->gyro_cfg.power = BMI160_GYRO_SUSPEND_MODE;
    dev->gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;

    /* To maintain the previous state of accel configuration */
    dev->prev_accel_cfg = dev->accel_cfg;

    /* To maintain the previous state of gyro configuration */
    dev->prev_gyro_cfg = dev->gyro_cfg;
}

/*!
 * @brief This API reads the data from the given register address
 * of sensor.
 */
int8_t bmi160_get_regs(uint8_t reg_addr, uint8_t *data, uint16_t len, const struct bmi160_dev *dev)
{
    int8_t rslt = BMI160_OK;

    /* Null-pointer check */
    if ((dev == NULL) || (dev->read == NULL))
    {
        rslt = BMI160_E_NULL_PTR;
    }
    else if (len == 0)
    {
        rslt = BMI160_E_READ_WRITE_LENGTH_INVALID;
    }
    else
    {
        /* Configuring reg_addr for SPI Interface */
        if (dev->intf == BMI160_SPI_INTF)
        {
            reg_addr = (reg_addr | BMI160_SPI_RD_MASK);
        }

        rslt = dev->read(dev->id, reg_addr, data, len);
    }

    return rslt;
}

/*!
 * @brief This API writes the given data to the register address
 * of sensor.
 */
int8_t bmi160_set_regs(uint8_t reg_addr, uint8_t *data, uint16_t len, const struct bmi160_dev *dev)
{
    int8_t rslt = BMI160_OK;
    uint8_t count = 0;

    /* Null-pointer check */
    if ((dev == NULL) || (dev->write == NULL))
    {
        rslt = BMI160_E_NULL_PTR;
    }
    else if (len == 0)
    {
        rslt = BMI160_E_READ_WRITE_LENGTH_INVALID;
    }
    else
    {
        /* Configuring reg_addr for SPI Interface */
        if (dev->intf == BMI160_SPI_INTF)
        {
            reg_addr = (reg_addr & BMI160_SPI_WR_MASK);
        }

        if ((dev->prev_accel_cfg.power == BMI160_ACCEL_NORMAL_MODE) ||
            (dev->prev_gyro_cfg.power == BMI160_GYRO_NORMAL_MODE))
        {
            rslt = dev->write(dev->id, reg_addr, data, len);

            /* Kindly refer bmi160 data sheet section 3.2.4 */
            dev->delay_ms(1);

        }
        else
        {
            /*Burst write is not allowed in
             * suspend & low power mode */
            for (; count < len; count++)
            {
                rslt = dev->write(dev->id, reg_addr, &data[count], 1);
                reg_addr++;

                /* Kindly refer bmi160 data sheet section 3.2.4 */
                dev->delay_ms(1);

            }
        }

        if (rslt != BMI160_OK)
        {
            rslt = BMI160_E_COM_FAIL;
        }
    }

    return rslt;
}

/*!
 * @brief This API resets and restarts the device.
 * All register values are overwritten with default parameters.
 */
int8_t bmi160_soft_reset(struct bmi160_dev *dev)
{
    int8_t rslt;
    uint8_t data = BMI160_SOFT_RESET_CMD;

    /* Null-pointer check */
    if ((dev == NULL) || (dev->delay_ms == NULL))
    {
        rslt = BMI160_E_NULL_PTR;
    }
    else
    {
        /* Reset the device */
        rslt = bmi160_set_regs(BMI160_COMMAND_REG_ADDR, &data, 1, dev);
        dev->delay_ms(BMI160_SOFT_RESET_DELAY_MS);
        if ((rslt == BMI160_OK) && (dev->intf == BMI160_SPI_INTF))
        {
            /* Dummy read of 0x7F register to enable SPI Interface
             * if SPI is used */
            rslt = bmi160_get_regs(BMI160_SPI_COMM_TEST_ADDR, &data, 1, dev);
        }

        if (rslt == BMI160_OK)
        {
            /* Update the default parameters */
            default_param_settg(dev);
        }
    }

    return rslt;
}

int8_t _bmi160_init(struct bmi160_dev *dev)
{
    int8_t rslt;
    uint8_t data;
    uint8_t try = 3;

    /* Null-pointer check */
    rslt = null_ptr_check(dev);

    /* Dummy read of 0x7F register to enable SPI Interface
     * if SPI is used */
    if ((rslt == BMI160_OK) && (dev->intf == BMI160_SPI_INTF))
    {
        rslt = bmi160_get_regs(BMI160_SPI_COMM_TEST_ADDR, &data, 1, dev);
    }

    if (rslt == BMI160_OK)
    {
        /* Assign chip id as zero */
        dev->chip_id = 0;

        while ((try--) && (dev->chip_id != BMI160_CHIP_ID))
        {
            /* Read chip_id */
            rslt = bmi160_get_regs(BMI160_CHIP_ID_ADDR, &dev->chip_id, 1, dev);
        }

        if ((rslt == BMI160_OK) && (dev->chip_id == BMI160_CHIP_ID))
        {
            dev->any_sig_sel = BMI160_BOTH_ANY_SIG_MOTION_DISABLED;

            /* Soft reset */
            rslt = bmi160_soft_reset(dev);
        }
        else
        {
            rslt = BMI160_E_DEV_NOT_FOUND;
        }
    }

    return rslt;
}

int bmx160_init(struct bmi160_dev bmi)
{
    (void) bmi;
    i2c_dev = bmi.i2c_dev;
    i2c_acquire(bmi.i2c_dev);

    bmi.id = BMI160_I2C_ADDR;
    bmi.read = user_i2c_read;
    bmi.write = user_i2c_write;
    bmi.delay_ms = user_delay;
    bmi.intf = BMI160_I2C_INTF;

    int8_t rslt = _bmi160_init(&bmi);
    if (rslt == BMI160_OK) {
        DEBUG("Success initializing BMI160 - Chip ID 0x%X\n", bmi.chip_id);
    } else if (rslt == BMI160_E_DEV_NOT_FOUND) {
        DEBUG("Error initializing BMI160: device not found\n");
        return 1;
    } else {
        DEBUG("Error initializing BMI160 - %d\n", rslt);
        return 1;
    }

    i2c_release(bmi.i2c_dev);
    DEBUG("[bmx160] device initialized\n");
    return 0;
}
