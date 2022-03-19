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


/* ================ Initialization and Configuration ==================== */

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

/*!
 * @brief This API process the accel odr.
 */
static int8_t process_accel_odr(uint8_t *data, const struct bmi160_dev *dev)
{
    int8_t rslt = 0;
    uint8_t temp = 0;
    uint8_t odr = 0;

    if (dev->accel_cfg.odr <= BMI160_ACCEL_ODR_1600HZ)
    {
        if (dev->accel_cfg.odr != dev->prev_accel_cfg.odr)
        {
            odr = (uint8_t)dev->accel_cfg.odr;
            temp = *data & ~BMI160_ACCEL_ODR_MASK;

            /* Adding output data rate */
            *data = temp | (odr & BMI160_ACCEL_ODR_MASK);
        }
    }
    else
    {
        rslt = BMI160_E_OUT_OF_RANGE;
    }

    return rslt;
}

/*!
 * @brief This API process the accel bandwidth.
 */
static int8_t process_accel_bw(uint8_t *data, const struct bmi160_dev *dev)
{
    int8_t rslt = 0;
    uint8_t temp = 0;
    uint8_t bw = 0;

    if (dev->accel_cfg.bw <= BMI160_ACCEL_BW_RES_AVG128)
    {
        if (dev->accel_cfg.bw != dev->prev_accel_cfg.bw)
        {
            bw = (uint8_t)dev->accel_cfg.bw;
            temp = *data & ~BMI160_ACCEL_BW_MASK;

            /* Adding bandwidth */
            *data = temp | ((bw << 4) & BMI160_ACCEL_BW_MASK);
        }
    }
    else
    {
        rslt = BMI160_E_OUT_OF_RANGE;
    }

    return rslt;
}

/*!
 * @brief This API process the accel range.
 */
static int8_t process_accel_range(uint8_t *data, const struct bmi160_dev *dev)
{
    int8_t rslt = 0;
    uint8_t temp = 0;
    uint8_t range = 0;

    if (dev->accel_cfg.range <= BMI160_ACCEL_RANGE_16G)
    {
        if (dev->accel_cfg.range != dev->prev_accel_cfg.range)
        {
            range = (uint8_t)dev->accel_cfg.range;
            temp = *data & ~BMI160_ACCEL_RANGE_MASK;

            /* Adding range */
            *data = temp | (range & BMI160_ACCEL_RANGE_MASK);
        }
    }
    else
    {
        rslt = BMI160_E_OUT_OF_RANGE;
    }

    return rslt;
}

/*!
 * @brief This API check the accel configuration.
 */
static int8_t check_accel_config(uint8_t *data, const struct bmi160_dev *dev)
{
    int8_t rslt;

    /* read accel Output data rate and bandwidth */
    rslt = bmi160_get_regs(BMI160_ACCEL_CONFIG_ADDR, data, 2, dev);
    if (rslt == BMI160_OK)
    {
        rslt = process_accel_odr(&data[0], dev);
        if (rslt == BMI160_OK)
        {
            rslt = process_accel_bw(&data[0], dev);
            if (rslt == BMI160_OK)
            {
                rslt = process_accel_range(&data[1], dev);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API set the accel configuration.
 */
static int8_t set_accel_conf(struct bmi160_dev *dev)
{
    int8_t rslt;
    uint8_t data[2] = { 0 };

    rslt = check_accel_config(data, dev);
    if (rslt == BMI160_OK)
    {
        /* Write output data rate and bandwidth */
        rslt = bmi160_set_regs(BMI160_ACCEL_CONFIG_ADDR, &data[0], 1, dev);
        if (rslt == BMI160_OK)
        {
            dev->prev_accel_cfg.odr = dev->accel_cfg.odr;
            dev->prev_accel_cfg.bw = dev->accel_cfg.bw;

            /* write accel range */
            rslt = bmi160_set_regs(BMI160_ACCEL_RANGE_ADDR, &data[1], 1, dev);
            if (rslt == BMI160_OK)
            {
                dev->prev_accel_cfg.range = dev->accel_cfg.range;
            }
        }
    }

    DEBUG("[bmx160] set_accel_conf: %d\n", rslt);
    return rslt;
}

/*!
 * @brief This API process the gyro odr.
 */
static int8_t process_gyro_odr(uint8_t *data, const struct bmi160_dev *dev)
{
    int8_t rslt = 0;
    uint8_t temp = 0;
    uint8_t odr = 0;

    if (dev->gyro_cfg.odr <= BMI160_GYRO_ODR_3200HZ)
    {
        if (dev->gyro_cfg.odr != dev->prev_gyro_cfg.odr)
        {
            odr = (uint8_t)dev->gyro_cfg.odr;
            temp = (*data & ~BMI160_GYRO_ODR_MASK);

            /* Adding output data rate */
            *data = temp | (odr & BMI160_GYRO_ODR_MASK);
        }
    }
    else
    {
        rslt = BMI160_E_OUT_OF_RANGE;
    }

    return rslt;
}

/*!
 * @brief This API process the gyro bandwidth.
 */
static int8_t process_gyro_bw(uint8_t *data, const struct bmi160_dev *dev)
{
    int8_t rslt = 0;
    uint8_t temp = 0;
    uint8_t bw = 0;

    if (dev->gyro_cfg.bw <= BMI160_GYRO_BW_NORMAL_MODE)
    {
        bw = (uint8_t)dev->gyro_cfg.bw;
        temp = *data & ~BMI160_GYRO_BW_MASK;

        /* Adding bandwidth */
        *data = temp | ((bw << 4) & BMI160_GYRO_BW_MASK);
    }
    else
    {
        rslt = BMI160_E_OUT_OF_RANGE;
    }

    return rslt;
}

/*!
 * @brief This API process the gyro range.
 */
static int8_t process_gyro_range(uint8_t *data, const struct bmi160_dev *dev)
{
    int8_t rslt = 0;
    uint8_t temp = 0;
    uint8_t range = 0;

    if (dev->gyro_cfg.range <= BMI160_GYRO_RANGE_125_DPS)
    {
        if (dev->gyro_cfg.range != dev->prev_gyro_cfg.range)
        {
            range = (uint8_t)dev->gyro_cfg.range;
            temp = *data & ~BMI160_GYRO_RANGE_MASK;

            /* Adding range */
            *data = temp | (range & BMI160_GYRO_RANGE_MASK);
        }
    }
    else
    {
        rslt = BMI160_E_OUT_OF_RANGE;
    }

    return rslt;
}

/*!
 * @brief This API check the gyro configuration.
 */
static int8_t check_gyro_config(uint8_t *data, const struct bmi160_dev *dev)
{
    int8_t rslt;

    /* read gyro Output data rate and bandwidth */
    rslt = bmi160_get_regs(BMI160_GYRO_CONFIG_ADDR, data, 2, dev);
    if (rslt == BMI160_OK)
    {
        rslt = process_gyro_odr(&data[0], dev);
        if (rslt == BMI160_OK)
        {
            rslt = process_gyro_bw(&data[0], dev);
            if (rslt == BMI160_OK)
            {
                rslt = process_gyro_range(&data[1], dev);
            }
        }
    }

    return rslt;
}

static int8_t set_gyro_conf(struct bmi160_dev *dev)
{
    int8_t rslt;
    uint8_t data[2] = { 0 };

    rslt = check_gyro_config(data, dev);
    if (rslt == BMI160_OK)
    {
        /* Write output data rate and bandwidth */
        rslt = bmi160_set_regs(BMI160_GYRO_CONFIG_ADDR, &data[0], 1, dev);
        if (rslt == BMI160_OK)
        {
            dev->prev_gyro_cfg.odr = dev->gyro_cfg.odr;
            dev->prev_gyro_cfg.bw = dev->gyro_cfg.bw;

            /* Write gyro range */
            rslt = bmi160_set_regs(BMI160_GYRO_RANGE_ADDR, &data[1], 1, dev);
            if (rslt == BMI160_OK)
            {
                dev->prev_gyro_cfg.range = dev->gyro_cfg.range;
            }
        }
    }

    DEBUG("[bmx160] set_gyro_conf: %d\n", rslt);
    return rslt;
}

/*!
 * @brief This API process the undersampling setting of Accel.
 */
static int8_t process_under_sampling(uint8_t *data, const struct bmi160_dev *dev)
{
    int8_t rslt;
    uint8_t temp = 0;
    uint8_t pre_filter[2] = { 0 };

    rslt = bmi160_get_regs(BMI160_ACCEL_CONFIG_ADDR, data, 1, dev);
    if (rslt == BMI160_OK)
    {
        if (dev->accel_cfg.power == BMI160_ACCEL_LOWPOWER_MODE)
        {
            temp = *data & ~BMI160_ACCEL_UNDERSAMPLING_MASK;

            /* Set under-sampling parameter */
            *data = temp | ((1 << 7) & BMI160_ACCEL_UNDERSAMPLING_MASK);

            /* Write data */
            rslt = bmi160_set_regs(BMI160_ACCEL_CONFIG_ADDR, data, 1, dev);

            /* Disable the pre-filter data in low power mode */
            if (rslt == BMI160_OK)
            {
                /* Disable the Pre-filter data*/
                rslt = bmi160_set_regs(BMI160_INT_DATA_0_ADDR, pre_filter, 2, dev);
            }
        }
        else if (*data & BMI160_ACCEL_UNDERSAMPLING_MASK)
        {
            temp = *data & ~BMI160_ACCEL_UNDERSAMPLING_MASK;

            /* Disable under-sampling parameter if already enabled */
            *data = temp;

            /* Write data */
            rslt = bmi160_set_regs(BMI160_ACCEL_CONFIG_ADDR, data, 1, dev);
        }
    }

    return rslt;
}

/*!
 * @brief This API sets the accel power.
 */
static int8_t set_accel_pwr(struct bmi160_dev *dev)
{
    int8_t rslt = 0;
    uint8_t data = 0;

    if ((dev->accel_cfg.power >= BMI160_ACCEL_SUSPEND_MODE) && (dev->accel_cfg.power <= BMI160_ACCEL_LOWPOWER_MODE))
    {
        if (dev->accel_cfg.power != dev->prev_accel_cfg.power)
        {
            rslt = process_under_sampling(&data, dev);
            if (rslt == BMI160_OK)
            {
                /* Write accel power */
                rslt = bmi160_set_regs(BMI160_COMMAND_REG_ADDR, &dev->accel_cfg.power, 1, dev);

                /* Add delay of 3.8 ms - refer data sheet table 24*/
                if (dev->prev_accel_cfg.power == BMI160_ACCEL_SUSPEND_MODE)
                {
                    dev->delay_ms(BMI160_ACCEL_DELAY_MS);
                }

                dev->prev_accel_cfg.power = dev->accel_cfg.power;
            }
        }
    }
    else
    {
        rslt = BMI160_E_INVALID_CONFIG;
    }

    return rslt;
}

/*!
 * @brief This API sets the gyro power mode.
 */
static int8_t set_gyro_pwr(struct bmi160_dev *dev)
{
    int8_t rslt = 0;

    if ((dev->gyro_cfg.power == BMI160_GYRO_SUSPEND_MODE) || (dev->gyro_cfg.power == BMI160_GYRO_NORMAL_MODE) ||
        (dev->gyro_cfg.power == BMI160_GYRO_FASTSTARTUP_MODE))
    {
        if (dev->gyro_cfg.power != dev->prev_gyro_cfg.power)
        {
            /* Write gyro power */
            rslt = bmi160_set_regs(BMI160_COMMAND_REG_ADDR, &dev->gyro_cfg.power, 1, dev);
            if (dev->prev_gyro_cfg.power == BMI160_GYRO_SUSPEND_MODE)
            {
                /* Delay of 80 ms - datasheet Table 24 */
                dev->delay_ms(BMI160_GYRO_DELAY_MS);
            }
            else if ((dev->prev_gyro_cfg.power == BMI160_GYRO_FASTSTARTUP_MODE) &&
                     (dev->gyro_cfg.power == BMI160_GYRO_NORMAL_MODE))
            {
                /* This delay is required for transition from
                 * fast-startup mode to normal mode - datasheet Table 3 */
                dev->delay_ms(10);
            }
            else
            {
                /* do nothing */
            }

            dev->prev_gyro_cfg.power = dev->gyro_cfg.power;
        }
    }
    else
    {
        rslt = BMI160_E_INVALID_CONFIG;
    }

    return rslt;
}

/*!
 * @brief This API sets the power mode of the sensor.
 */
int8_t bmi160_set_power_mode(struct bmi160_dev *dev)
{
    int8_t rslt = 0;

    /* Null-pointer check */
    if ((dev == NULL) || (dev->delay_ms == NULL))
    {
        rslt = BMI160_E_NULL_PTR;
    }
    else
    {
        rslt = set_accel_pwr(dev);
        if (rslt == BMI160_OK)
        {
            rslt = set_gyro_pwr(dev);
        }
    }

    return rslt;
}

/*!
 * @brief This API checks the invalid settings for ODR & Bw for
 * Accel and Gyro.
 */
static int8_t check_invalid_settg(const struct bmi160_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0;

    /* read the error reg */
    rslt = bmi160_get_regs(BMI160_ERROR_REG_ADDR, &data, 1, dev);
    data = data >> 1;
    data = data & BMI160_ERR_REG_MASK;
    if (data == 1)
    {
        rslt = BMI160_E_ACCEL_ODR_BW_INVALID;
    }
    else if (data == 2)
    {
        rslt = BMI160_E_GYRO_ODR_BW_INVALID;
    }
    else if (data == 3)
    {
        rslt = BMI160_E_LWP_PRE_FLTR_INT_INVALID;
    }
    else if (data == 7)
    {
        rslt = BMI160_E_LWP_PRE_FLTR_INVALID;
    }

    return rslt;
}








/* ================ FIFO functions ==================== */

/*!
 *  @brief This API is used to reset the FIFO related configurations
 *  in the fifo_frame structure.
 */
static void reset_fifo_data_structure(const struct bmi160_dev *dev)
{
    /*Prepare for next FIFO read by resetting FIFO's
     * internal data structures*/
    dev->fifo->accel_byte_start_idx = 0;
    dev->fifo->gyro_byte_start_idx = 0;
    dev->fifo->aux_byte_start_idx = 0;
    dev->fifo->sensor_time = 0;
    dev->fifo->skipped_frame_count = 0;
}

/*!
 *  @brief This API is used to read fifo_byte_counter value (i.e)
 *  current fill-level in Fifo buffer.
 */
static int8_t get_fifo_byte_counter(uint16_t *bytes_to_read, struct bmi160_dev const *dev)
{
    int8_t rslt = 0;
    uint8_t data[2];
    uint8_t addr = BMI160_FIFO_LENGTH_ADDR;

    rslt |= bmi160_get_regs(addr, data, 2, dev);
    data[1] = data[1] & BMI160_FIFO_BYTE_COUNTER_MASK;

    /* Available data in FIFO is stored in bytes_to_read*/
    *bytes_to_read = (((uint16_t)data[1] << 8) | ((uint16_t)data[0]));

    return rslt;
}

/*!
 * @brief This API reads the data from fifo buffer.
 */
int8_t bmi160_get_fifo_data(struct bmi160_dev const *dev)
{
    int8_t rslt = 0;
    uint16_t bytes_to_read = 0;
    uint16_t user_fifo_len = 0;

    /* check the bmi160 structure as NULL*/
    if ((dev == NULL) || (dev->fifo->data == NULL))
    {
        rslt = BMI160_E_NULL_PTR;
    }
    else
    {
        reset_fifo_data_structure(dev);

        /* get current FIFO fill-level*/
        rslt = get_fifo_byte_counter(&bytes_to_read, dev);
        if (rslt == BMI160_OK)
        {
            user_fifo_len = dev->fifo->length;
            if ((dev->fifo->length > bytes_to_read))
            {
                /* Handling the case where user requests
                 * more data than available in FIFO */
                dev->fifo->length = bytes_to_read;
            }

            if ((dev->fifo->fifo_time_enable == BMI160_FIFO_TIME_ENABLE) &&
                (bytes_to_read + BMI160_FIFO_BYTES_OVERREAD <= user_fifo_len))
            {
                /* Handling case of sensor time availability*/
                dev->fifo->length = dev->fifo->length + BMI160_FIFO_BYTES_OVERREAD;
            }

            /* read only the filled bytes in the FIFO Buffer */
            rslt = bmi160_get_regs(BMI160_FIFO_DATA_ADDR, dev->fifo->data, dev->fifo->length, dev);
        }
    }

    return rslt;
}

/*!
 *  @brief This API computes the number of bytes of gyro FIFO data
 *  which is to be parsed in header-less mode
 */
static void get_gyro_len_to_parse(uint16_t *data_index,
                                  uint16_t *data_read_length,
                                  const uint8_t *gyro_frame_count,
                                  const struct bmi160_dev *dev)
{
    /* Data start index */
    *data_index = dev->fifo->gyro_byte_start_idx;
    if (dev->fifo->fifo_data_enable == BMI160_FIFO_G_ENABLE)
    {
        *data_read_length = (*gyro_frame_count) * BMI160_FIFO_G_LENGTH;
    }
    else if (dev->fifo->fifo_data_enable == BMI160_FIFO_G_A_ENABLE)
    {
        *data_read_length = (*gyro_frame_count) * BMI160_FIFO_GA_LENGTH;
    }
    else if (dev->fifo->fifo_data_enable == BMI160_FIFO_M_G_ENABLE)
    {
        *data_read_length = (*gyro_frame_count) * BMI160_FIFO_MG_LENGTH;
    }
    else if (dev->fifo->fifo_data_enable == BMI160_FIFO_M_G_A_ENABLE)
    {
        *data_read_length = (*gyro_frame_count) * BMI160_FIFO_MGA_LENGTH;
    }
    else
    {
        /* When gyro is not enabled ,there will be no gyro data.
         * so we update the data index as complete */
        *data_index = dev->fifo->length;
    }

    if (*data_read_length > dev->fifo->length)
    {
        /* Handling the case where more data is requested
         * than that is available*/
        *data_read_length = dev->fifo->length;
    }
}

/*!
 *  @brief This API checks the presence of non-valid frames in the read fifo data.
 */
static void check_frame_validity(uint16_t *data_index, const struct bmi160_dev *dev)
{
    if ((*data_index + 2) < dev->fifo->length)
    {
        /* Check if FIFO is empty */
        if ((dev->fifo->data[*data_index] == FIFO_CONFIG_MSB_CHECK) &&
            (dev->fifo->data[*data_index + 1] == FIFO_CONFIG_LSB_CHECK))
        {
            /*Update the data index as complete*/
            *data_index = dev->fifo->length;
        }
    }
}

/*!
 *  @brief This API is used to parse the gyro data from the
 *  FIFO data and store it in the instance of the structure bmi160_sensor_data.
 */
static void unpack_gyro_data(struct bmi160_sensor_data *gyro_data,
                             uint16_t data_start_index,
                             const struct bmi160_dev *dev)
{
    uint16_t data_lsb;
    uint16_t data_msb;

    /* Gyro raw x data */
    data_lsb = dev->fifo->data[data_start_index++];
    data_msb = dev->fifo->data[data_start_index++];
    gyro_data->x = (int16_t)((data_msb << 8) | data_lsb);

    /* Gyro raw y data */
    data_lsb = dev->fifo->data[data_start_index++];
    data_msb = dev->fifo->data[data_start_index++];
    gyro_data->y = (int16_t)((data_msb << 8) | data_lsb);

    /* Gyro raw z data */
    data_lsb = dev->fifo->data[data_start_index++];
    data_msb = dev->fifo->data[data_start_index++];
    gyro_data->z = (int16_t)((data_msb << 8) | data_lsb);
}

/*!
 *  @brief This API is used to parse the gyroscope's data from the
 *  FIFO data in both header mode and header-less mode.
 *  It updates the idx value which is used to store the index of
 *  the current data byte which is parsed.
 */
static void unpack_gyro_frame(struct bmi160_sensor_data *gyro,
                              uint16_t *idx,
                              uint8_t *gyro_idx,
                              uint8_t frame_info,
                              const struct bmi160_dev *dev)
{
    switch (frame_info)
    {
        case BMI160_FIFO_HEAD_G:
        case BMI160_FIFO_G_ENABLE:

            /*Partial read, then skip the data*/
            if ((*idx + BMI160_FIFO_G_LENGTH) > dev->fifo->length)
            {
                /*Update the data index as complete*/
                *idx = dev->fifo->length;
                break;
            }

            /*Unpack the data array into structure instance "gyro"*/
            unpack_gyro_data(&gyro[*gyro_idx], *idx, dev);

            /*Move the data index*/
            (*idx) = (*idx) + BMI160_FIFO_G_LENGTH;
            (*gyro_idx)++;
            break;
        case BMI160_FIFO_HEAD_G_A:
        case BMI160_FIFO_G_A_ENABLE:

            /*Partial read, then skip the data*/
            if ((*idx + BMI160_FIFO_GA_LENGTH) > dev->fifo->length)
            {
                /*Update the data index as complete*/
                *idx = dev->fifo->length;
                break;
            }

            /* Unpack the data array into structure instance "gyro" */
            unpack_gyro_data(&gyro[*gyro_idx], *idx, dev);

            /* Move the data index */
            *idx = *idx + BMI160_FIFO_GA_LENGTH;
            (*gyro_idx)++;
            break;
        case BMI160_FIFO_HEAD_M_G_A:
        case BMI160_FIFO_M_G_A_ENABLE:

            /*Partial read, then skip the data*/
            if ((*idx + BMI160_FIFO_MGA_LENGTH) > dev->fifo->length)
            {
                /*Update the data index as complete*/
                *idx = dev->fifo->length;
                break;
            }

            /*Unpack the data array into structure instance "gyro"*/
            unpack_gyro_data(&gyro[*gyro_idx], *idx + BMI160_FIFO_M_LENGTH, dev);

            /*Move the data index*/
            *idx = *idx + BMI160_FIFO_MGA_LENGTH;
            (*gyro_idx)++;
            break;
        case BMI160_FIFO_HEAD_M_A:
        case BMI160_FIFO_M_A_ENABLE:

            /* Move the data index */
            *idx = *idx + BMI160_FIFO_MA_LENGTH;
            break;
        case BMI160_FIFO_HEAD_M:
        case BMI160_FIFO_M_ENABLE:
            (*idx) = (*idx) + BMI160_FIFO_M_LENGTH;
            break;
        case BMI160_FIFO_HEAD_M_G:
        case BMI160_FIFO_M_G_ENABLE:

            /*Partial read, then skip the data*/
            if ((*idx + BMI160_FIFO_MG_LENGTH) > dev->fifo->length)
            {
                /*Update the data index as complete*/
                *idx = dev->fifo->length;
                break;
            }

            /*Unpack the data array into structure instance "gyro"*/
            unpack_gyro_data(&gyro[*gyro_idx], *idx + BMI160_FIFO_M_LENGTH, dev);

            /*Move the data index*/
            (*idx) = (*idx) + BMI160_FIFO_MG_LENGTH;
            (*gyro_idx)++;
            break;
        case BMI160_FIFO_HEAD_A:
        case BMI160_FIFO_A_ENABLE:

            /*Move the data index*/
            *idx = *idx + BMI160_FIFO_A_LENGTH;
            break;
        default:
            break;
    }
}

/*!
 *  @brief This API is used to parse and store the sensor time from the
 *  FIFO data in the structure instance dev.
 */
static void unpack_sensortime_frame(uint16_t *data_index, const struct bmi160_dev *dev)
{
    uint32_t sensor_time_byte3 = 0;
    uint16_t sensor_time_byte2 = 0;
    uint8_t sensor_time_byte1 = 0;

    /*Partial read, then move the data index to last data*/
    if ((*data_index + BMI160_SENSOR_TIME_LENGTH) > dev->fifo->length)
    {
        /*Update the data index as complete*/
        *data_index = dev->fifo->length;
    }
    else
    {
        sensor_time_byte3 = dev->fifo->data[(*data_index) + BMI160_SENSOR_TIME_MSB_BYTE] << 16;
        sensor_time_byte2 = dev->fifo->data[(*data_index) + BMI160_SENSOR_TIME_XLSB_BYTE] << 8;
        sensor_time_byte1 = dev->fifo->data[(*data_index)];

        /* Sensor time */
        dev->fifo->sensor_time = (uint32_t)(sensor_time_byte3 | sensor_time_byte2 | sensor_time_byte1);
        *data_index = (*data_index) + BMI160_SENSOR_TIME_LENGTH;
    }
}

/*!
 *  @brief This API is used to parse and store the skipped_frame_count from
 *  the FIFO data in the structure instance dev.
 */
static void unpack_skipped_frame(uint16_t *data_index, const struct bmi160_dev *dev)
{
    /*Partial read, then move the data index to last data*/
    if (*data_index >= dev->fifo->length)
    {
        /*Update the data index as complete*/
        *data_index = dev->fifo->length;
    }
    else
    {
        dev->fifo->skipped_frame_count = dev->fifo->data[*data_index];

        /*Move the data index*/
        *data_index = (*data_index) + 1;
    }
}

/*!
 *  @brief This API is used to move the data index ahead of the
 *  current_frame_length parameter when unnecessary FIFO data appears while
 *  extracting the user specified data.
 */
static void move_next_frame(uint16_t *data_index, uint8_t current_frame_length, const struct bmi160_dev *dev)
{
    /*Partial read, then move the data index to last data*/
    if ((*data_index + current_frame_length) > dev->fifo->length)
    {
        /*Update the data index as complete*/
        *data_index = dev->fifo->length;
    }
    else
    {
        /*Move the data index to next frame*/
        *data_index = *data_index + current_frame_length;
    }
}

/*!
 *  @brief This API is used to parse the gyro data from the
 *  FIFO data in header mode.
 */
static void extract_gyro_header_mode(struct bmi160_sensor_data *gyro_data,
                                     uint8_t *gyro_length,
                                     const struct bmi160_dev *dev)
{
    uint8_t frame_header = 0;
    uint16_t data_index;
    uint8_t gyro_index = 0;

    for (data_index = dev->fifo->gyro_byte_start_idx; data_index < dev->fifo->length;)
    {
        /* extracting Frame header */
        frame_header = (dev->fifo->data[data_index] & BMI160_FIFO_TAG_INTR_MASK);

        /*Index is moved to next byte where the data is starting*/
        data_index++;
        switch (frame_header)
        {
            /* GYRO frame */
            case BMI160_FIFO_HEAD_G:
            case BMI160_FIFO_HEAD_G_A:
            case BMI160_FIFO_HEAD_M_G:
            case BMI160_FIFO_HEAD_M_G_A:
                unpack_gyro_frame(gyro_data, &data_index, &gyro_index, frame_header, dev);
                break;
            case BMI160_FIFO_HEAD_A:
                move_next_frame(&data_index, BMI160_FIFO_A_LENGTH, dev);
                break;
            case BMI160_FIFO_HEAD_M:
                move_next_frame(&data_index, BMI160_FIFO_M_LENGTH, dev);
                break;
            case BMI160_FIFO_HEAD_M_A:
                move_next_frame(&data_index, BMI160_FIFO_M_LENGTH, dev);
                break;

            /* Sensor time frame */
            case BMI160_FIFO_HEAD_SENSOR_TIME:
                unpack_sensortime_frame(&data_index, dev);
                break;

            /* Skip frame */
            case BMI160_FIFO_HEAD_SKIP_FRAME:
                unpack_skipped_frame(&data_index, dev);
                break;

            /* Input config frame */
            case BMI160_FIFO_HEAD_INPUT_CONFIG:
                move_next_frame(&data_index, 1, dev);
                break;
            case BMI160_FIFO_HEAD_OVER_READ:

                /* Update the data index as complete in case of over read */
                data_index = dev->fifo->length;
                break;
            default:
                break;
        }
        if (*gyro_length == gyro_index)
        {
            /*Number of frames to read completed*/
            break;
        }
    }

    /*Update number of gyro data read*/
    *gyro_length = gyro_index;

    /*Update the gyro frame index*/
    dev->fifo->gyro_byte_start_idx = data_index;
}

/*!
 *  @brief This API parses and extracts the gyro frames from
 *  FIFO data read by the "bmi160_get_fifo_data" API and stores it in
 *  the "gyro_data" structure instance.
 */
int8_t bmi160_extract_gyro(struct bmi160_sensor_data *gyro_data, uint8_t *gyro_length, struct bmi160_dev const *dev)
{
    int8_t rslt = 0;
    uint16_t data_index = 0;
    uint16_t data_read_length = 0;
    uint8_t gyro_index = 0;
    uint8_t fifo_data_enable = 0;

    if (dev == NULL || dev->fifo->data == NULL)
    {
        rslt = BMI160_E_NULL_PTR;
    }
    else
    {
        /* Parsing the FIFO data in header-less mode */
        if (dev->fifo->fifo_header_enable == 0)
        {
            /* Number of bytes to be parsed from FIFO */
            get_gyro_len_to_parse(&data_index, &data_read_length, gyro_length, dev);
            for (; data_index < data_read_length;)
            {
                /*Check for the availability of next two bytes of FIFO data */
                check_frame_validity(&data_index, dev);
                fifo_data_enable = dev->fifo->fifo_data_enable;
                unpack_gyro_frame(gyro_data, &data_index, &gyro_index, fifo_data_enable, dev);
            }

            /* update number of gyro data read */
            *gyro_length = gyro_index;

            /* update the gyro byte index */
            dev->fifo->gyro_byte_start_idx = data_index;
        }
        else
        {
            /* Parsing the FIFO data in header mode */
            extract_gyro_header_mode(gyro_data, gyro_length, dev);
        }
    }

    return rslt;
}

/*!
 *  @brief This API is used to compute the number of bytes of accel FIFO data
 *  which is to be parsed in header-less mode
 */
static void get_accel_len_to_parse(uint16_t *data_index,
                                   uint16_t *data_read_length,
                                   const uint8_t *acc_frame_count,
                                   const struct bmi160_dev *dev)
{
    /* Data start index */
    *data_index = dev->fifo->accel_byte_start_idx;
    if (dev->fifo->fifo_data_enable == BMI160_FIFO_A_ENABLE)
    {
        *data_read_length = (*acc_frame_count) * BMI160_FIFO_A_LENGTH;
    }
    else if (dev->fifo->fifo_data_enable == BMI160_FIFO_G_A_ENABLE)
    {
        *data_read_length = (*acc_frame_count) * BMI160_FIFO_GA_LENGTH;
    }
    else if (dev->fifo->fifo_data_enable == BMI160_FIFO_M_A_ENABLE)
    {
        *data_read_length = (*acc_frame_count) * BMI160_FIFO_MA_LENGTH;
    }
    else if (dev->fifo->fifo_data_enable == BMI160_FIFO_M_G_A_ENABLE)
    {
        *data_read_length = (*acc_frame_count) * BMI160_FIFO_MGA_LENGTH;
    }
    else
    {
        /* When accel is not enabled ,there will be no accel data.
         * so we update the data index as complete */
        *data_index = dev->fifo->length;
    }

    if (*data_read_length > dev->fifo->length)
    {
        /* Handling the case where more data is requested
         * than that is available*/
        *data_read_length = dev->fifo->length;
    }
}

/*!
 *  @brief This API is used to parse the accelerometer data from the
 *  FIFO data and store it in the instance of the structure bmi160_sensor_data.
 */
static void unpack_accel_data(struct bmi160_sensor_data *accel_data,
                              uint16_t data_start_index,
                              const struct bmi160_dev *dev)
{
    uint16_t data_lsb;
    uint16_t data_msb;

    /* Accel raw x data */
    data_lsb = dev->fifo->data[data_start_index++];
    data_msb = dev->fifo->data[data_start_index++];
    accel_data->x = (int16_t)((data_msb << 8) | data_lsb);

    /* Accel raw y data */
    data_lsb = dev->fifo->data[data_start_index++];
    data_msb = dev->fifo->data[data_start_index++];
    accel_data->y = (int16_t)((data_msb << 8) | data_lsb);

    /* Accel raw z data */
    data_lsb = dev->fifo->data[data_start_index++];
    data_msb = dev->fifo->data[data_start_index++];
    accel_data->z = (int16_t)((data_msb << 8) | data_lsb);
}

/*!
 *  @brief This API is used to parse the accelerometer data from the
 *  FIFO data in both header mode and header-less mode.
 *  It updates the idx value which is used to store the index of
 *  the current data byte which is parsed.
 */
static void unpack_accel_frame(struct bmi160_sensor_data *acc,
                               uint16_t *idx,
                               uint8_t *acc_idx,
                               uint8_t frame_info,
                               const struct bmi160_dev *dev)
{
    switch (frame_info)
    {
        case BMI160_FIFO_HEAD_A:
        case BMI160_FIFO_A_ENABLE:

            /*Partial read, then skip the data*/
            if ((*idx + BMI160_FIFO_A_LENGTH) > dev->fifo->length)
            {
                /*Update the data index as complete*/
                *idx = dev->fifo->length;
                break;
            }

            /*Unpack the data array into the structure instance "acc" */
            unpack_accel_data(&acc[*acc_idx], *idx, dev);

            /*Move the data index*/
            *idx = *idx + BMI160_FIFO_A_LENGTH;
            (*acc_idx)++;
            break;
        case BMI160_FIFO_HEAD_G_A:
        case BMI160_FIFO_G_A_ENABLE:

            /*Partial read, then skip the data*/
            if ((*idx + BMI160_FIFO_GA_LENGTH) > dev->fifo->length)
            {
                /*Update the data index as complete*/
                *idx = dev->fifo->length;
                break;
            }

            /*Unpack the data array into structure instance "acc"*/
            unpack_accel_data(&acc[*acc_idx], *idx + BMI160_FIFO_G_LENGTH, dev);

            /*Move the data index*/
            *idx = *idx + BMI160_FIFO_GA_LENGTH;
            (*acc_idx)++;
            break;
        case BMI160_FIFO_HEAD_M_A:
        case BMI160_FIFO_M_A_ENABLE:

            /*Partial read, then skip the data*/
            if ((*idx + BMI160_FIFO_MA_LENGTH) > dev->fifo->length)
            {
                /*Update the data index as complete*/
                *idx = dev->fifo->length;
                break;
            }

            /*Unpack the data array into structure instance "acc"*/
            unpack_accel_data(&acc[*acc_idx], *idx + BMI160_FIFO_M_LENGTH, dev);

            /*Move the data index*/
            *idx = *idx + BMI160_FIFO_MA_LENGTH;
            (*acc_idx)++;
            break;
        case BMI160_FIFO_HEAD_M_G_A:
        case BMI160_FIFO_M_G_A_ENABLE:

            /*Partial read, then skip the data*/
            if ((*idx + BMI160_FIFO_MGA_LENGTH) > dev->fifo->length)
            {
                /*Update the data index as complete*/
                *idx = dev->fifo->length;
                break;
            }

            /*Unpack the data array into structure instance "acc"*/
            unpack_accel_data(&acc[*acc_idx], *idx + BMI160_FIFO_MG_LENGTH, dev);

            /*Move the data index*/
            *idx = *idx + BMI160_FIFO_MGA_LENGTH;
            (*acc_idx)++;
            break;
        case BMI160_FIFO_HEAD_M:
        case BMI160_FIFO_M_ENABLE:
            (*idx) = (*idx) + BMI160_FIFO_M_LENGTH;
            break;
        case BMI160_FIFO_HEAD_G:
        case BMI160_FIFO_G_ENABLE:
            (*idx) = (*idx) + BMI160_FIFO_G_LENGTH;
            break;
        case BMI160_FIFO_HEAD_M_G:
        case BMI160_FIFO_M_G_ENABLE:
            (*idx) = (*idx) + BMI160_FIFO_MG_LENGTH;
            break;
        default:
            break;
    }
}

/*!
 *  @brief This API is used to parse the accelerometer data from the
 *  FIFO data in header mode.
 */
static void extract_accel_header_mode(struct bmi160_sensor_data *accel_data,
                                      uint8_t *accel_length,
                                      const struct bmi160_dev *dev)
{
    uint8_t frame_header = 0;
    uint16_t data_index;
    uint8_t accel_index = 0;

    for (data_index = dev->fifo->accel_byte_start_idx; data_index < dev->fifo->length;)
    {
        /* extracting Frame header */
        frame_header = (dev->fifo->data[data_index] & BMI160_FIFO_TAG_INTR_MASK);

        /*Index is moved to next byte where the data is starting*/
        data_index++;
        switch (frame_header)
        {
            /* Accel frame */
            case BMI160_FIFO_HEAD_A:
            case BMI160_FIFO_HEAD_M_A:
            case BMI160_FIFO_HEAD_G_A:
            case BMI160_FIFO_HEAD_M_G_A:
                unpack_accel_frame(accel_data, &data_index, &accel_index, frame_header, dev);
                break;
            case BMI160_FIFO_HEAD_M:
                move_next_frame(&data_index, BMI160_FIFO_M_LENGTH, dev);
                break;
            case BMI160_FIFO_HEAD_G:
                move_next_frame(&data_index, BMI160_FIFO_G_LENGTH, dev);
                break;
            case BMI160_FIFO_HEAD_M_G:
                move_next_frame(&data_index, BMI160_FIFO_MG_LENGTH, dev);
                break;

            /* Sensor time frame */
            case BMI160_FIFO_HEAD_SENSOR_TIME:
                unpack_sensortime_frame(&data_index, dev);
                break;

            /* Skip frame */
            case BMI160_FIFO_HEAD_SKIP_FRAME:
                unpack_skipped_frame(&data_index, dev);
                break;

            /* Input config frame */
            case BMI160_FIFO_HEAD_INPUT_CONFIG:
                move_next_frame(&data_index, 1, dev);
                break;
            case BMI160_FIFO_HEAD_OVER_READ:

                /* Update the data index as complete in case of Over read */
                data_index = dev->fifo->length;
                break;
            default:
                break;
        }
        if (*accel_length == accel_index)
        {
            /* Number of frames to read completed */
            break;
        }
    }

    /*Update number of accel data read*/
    *accel_length = accel_index;

    /*Update the accel frame index*/
    dev->fifo->accel_byte_start_idx = data_index;
}

/*!
 *  @brief This API parses and extracts the accelerometer frames from
 *  FIFO data read by the "bmi160_get_fifo_data" API and stores it in
 *  the "accel_data" structure instance.
 */
int8_t bmi160_extract_accel(struct bmi160_sensor_data *accel_data, uint8_t *accel_length, struct bmi160_dev const *dev)
{
    int8_t rslt = 0;
    uint16_t data_index = 0;
    uint16_t data_read_length = 0;
    uint8_t accel_index = 0;
    uint8_t fifo_data_enable = 0;

    if (dev == NULL || dev->fifo == NULL || dev->fifo->data == NULL)
    {
        rslt = BMI160_E_NULL_PTR;
    }
    else
    {
        /* Parsing the FIFO data in header-less mode */
        if (dev->fifo->fifo_header_enable == 0)
        {
            /* Number of bytes to be parsed from FIFO */
            get_accel_len_to_parse(&data_index, &data_read_length, accel_length, dev);
            for (; data_index < data_read_length;)
            {
                /*Check for the availability of next two bytes of FIFO data */
                check_frame_validity(&data_index, dev);
                fifo_data_enable = dev->fifo->fifo_data_enable;
                unpack_accel_frame(accel_data, &data_index, &accel_index, fifo_data_enable, dev);
            }

            /* update number of accel data read*/
            *accel_length = accel_index;

            /*update the accel byte index*/
            dev->fifo->accel_byte_start_idx = data_index;
        }
        else
        {
            /* Parsing the FIFO data in header mode */
            extract_accel_header_mode(accel_data, accel_length, dev);
        }
    }

    return rslt;
}








/*!
 * @brief This API configures the power mode, range and bandwidth
 * of sensor.
 */
int8_t bmi160_set_params(struct bmi160_dev *dev)
{
    int8_t rslt = BMI160_OK;

    /* Null-pointer check */
    if ((dev == NULL) || (dev->delay_ms == NULL))
    {
        rslt = BMI160_E_NULL_PTR;
    }
    else
    {
        rslt = set_accel_conf(dev);
        if (rslt == BMI160_OK)
        {
            rslt = set_gyro_conf(dev);
            if (rslt == BMI160_OK)
            {
                /* write power mode for accel and gyro */
                rslt = bmi160_set_power_mode(dev);
                if (rslt == BMI160_OK)
                {
                    rslt = check_invalid_settg(dev);
                }
            }
        }
    }

    DEBUG("[bmx160] bmi160_set_params status: %d\n", rslt);
    return rslt;
}

/*!
 * @brief This API sets the FIFO configuration in the sensor.
 */
int8_t bmi160_set_fifo_config(uint8_t config, uint8_t enable, struct bmi160_dev const *dev)
{
    int8_t rslt = 0;
    uint8_t data = 0;
    uint8_t reg_addr = BMI160_FIFO_CONFIG_1_ADDR;
    uint8_t fifo_config = config & BMI160_FIFO_CONFIG_1_MASK;

    /* Check the bmi160_dev structure for NULL address*/
    if (dev == NULL)
    {
        rslt = BMI160_E_NULL_PTR;
    }
    else
    {
        rslt = bmi160_get_regs(reg_addr, &data, BMI160_ONE, dev);
        if (rslt == BMI160_OK)
        {
            if (fifo_config > 0)
            {
                if (enable == BMI160_ENABLE)
                {
                    data = data | fifo_config;
                }
                else
                {
                    data = data & (~fifo_config);
                }
            }

            /* write fifo frame content configuration*/
            rslt = bmi160_set_regs(reg_addr, &data, BMI160_ONE, dev);
            if (rslt == BMI160_OK)
            {
                /* read fifo frame content configuration*/
                rslt = bmi160_get_regs(reg_addr, &data, BMI160_ONE, dev);
                if (rslt == BMI160_OK)
                {
                    /* extract fifo header enabled status */
                    dev->fifo->fifo_header_enable = data & BMI160_FIFO_HEAD_ENABLE;

                    /* extract accel/gyr/aux. data enabled status */
                    dev->fifo->fifo_data_enable = data & BMI160_FIFO_M_G_A_ENABLE;

                    /* extract fifo sensor time enabled status */
                    dev->fifo->fifo_time_enable = data & BMI160_FIFO_TIME_ENABLE;
                }
            }
        }
    }

    DEBUG("[bmx160] bmi160_set_fifo_config status: %d\n", rslt);
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

int bmx160_init(struct bmi160_dev *bmi)
{
    (void) bmi;
    i2c_dev = bmi->i2c_dev;
    i2c_acquire(bmi->i2c_dev);

    bmi->id = BMI160_I2C_ADDR;
    bmi->read = user_i2c_read;
    bmi->write = user_i2c_write;
    bmi->delay_ms = user_delay;
    bmi->intf = BMI160_I2C_INTF;

    int8_t rslt = _bmi160_init(bmi);
    if (rslt == BMI160_OK) {
        DEBUG("Success initializing BMI160 - Chip ID 0x%X\n", bmi->chip_id);
    } else if (rslt == BMI160_E_DEV_NOT_FOUND) {
        DEBUG("Error initializing BMI160: device not found\n");
        return 1;
    } else {
        DEBUG("Error initializing BMI160 - %d\n", rslt);
        return 1;
    }

    i2c_release(bmi->i2c_dev);
    DEBUG("[bmx160] device initialized\n");
    return 0;
}
