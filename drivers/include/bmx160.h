/*
 * Copyright (C) 2016 Kees Bakker, SODAQ
 *               2017 Inria
 *               2018 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_bmx160 BMP160/BME160 temperature, pressure and humidity sensor
 * @ingroup     drivers_sensors
 * @brief       Device driver interface for the Bosch BMP160 and BME160 sensors
 *

 */

#ifndef BMX160_H
#define BMX160_H

#include <stdint.h>
#include "saul.h"

#if defined(MODULE_BMX160_SPI) || defined(MODULE_BMI160_SPI)
#define BMX160_USE_SPI
#include "periph/spi.h"
#else
#include "periph/i2c.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif


/*****************************************************************************/
/* type definitions */

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific read functions of the user
 */
typedef int8_t (*bmi160_read_fptr_t)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific write functions of the user
 */
typedef int8_t (*bmi160_write_fptr_t)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *read_data, uint16_t len);
typedef void (*bmi160_delay_fptr_t)(uint32_t period);

/*************************** Data structures *********************************/

/*!
 * @brief bmi160 interrupt status selection enum.
 */
enum bmi160_int_status_sel {
    BMI160_INT_STATUS_0 = 1,
    BMI160_INT_STATUS_1 = 2,
    BMI160_INT_STATUS_2 = 4,
    BMI160_INT_STATUS_3 = 8,
    BMI160_INT_STATUS_ALL = 15
};

/*!
 * @brief bmi160 interrupt status bits structure
 */
struct bmi160_int_status_bits
{
#ifdef LITTLE_ENDIAN

    uint32_t step : 1;
    uint32_t sigmot : 1;
    uint32_t anym : 1;

    /* pmu trigger will be handled later */
    uint32_t pmu_trigger_reserved : 1;
    uint32_t d_tap : 1;
    uint32_t s_tap : 1;
    uint32_t orient : 1;
    uint32_t flat_int : 1;
    uint32_t reserved : 2;
    uint32_t high_g : 1;
    uint32_t low_g : 1;
    uint32_t drdy : 1;
    uint32_t ffull : 1;
    uint32_t fwm : 1;
    uint32_t nomo : 1;
    uint32_t anym_first_x : 1;
    uint32_t anym_first_y : 1;
    uint32_t anym_first_z : 1;
    uint32_t anym_sign : 1;
    uint32_t tap_first_x : 1;
    uint32_t tap_first_y : 1;
    uint32_t tap_first_z : 1;
    uint32_t tap_sign : 1;
    uint32_t high_first_x : 1;
    uint32_t high_first_y : 1;
    uint32_t high_first_z : 1;
    uint32_t high_sign : 1;
    uint32_t orient_1_0 : 2;
    uint32_t orient_2 : 1;
    uint32_t flat : 1;
#else
    uint32_t high_first_x : 1;
    uint32_t high_first_y : 1;
    uint32_t high_first_z : 1;
    uint32_t high_sign : 1;
    uint32_t orient_1_0 : 2;
    uint32_t orient_2 : 1;
    uint32_t flat : 1;
    uint32_t anym_first_x : 1;
    uint32_t anym_first_y : 1;
    uint32_t anym_first_z : 1;
    uint32_t anym_sign : 1;
    uint32_t tap_first_x : 1;
    uint32_t tap_first_y : 1;
    uint32_t tap_first_z : 1;
    uint32_t tap_sign : 1;
    uint32_t reserved : 2;
    uint32_t high_g : 1;
    uint32_t low_g : 1;
    uint32_t drdy : 1;
    uint32_t ffull : 1;
    uint32_t fwm : 1;
    uint32_t nomo : 1;
    uint32_t step : 1;
    uint32_t sigmot : 1;
    uint32_t anym : 1;

    /* pmu trigger will be handled later */
    uint32_t pmu_trigger_reserved : 1;
    uint32_t d_tap : 1;
    uint32_t s_tap : 1;
    uint32_t orient : 1;
    uint32_t flat_int : 1;
#endif
};

/*!
 * @brief bmi160 interrupt status structure
 */
union bmi160_int_status
{
    uint8_t data[4];
    struct bmi160_int_status_bits bit;
};

/*!
 * @brief bmi160 sensor data structure which comprises of accel data
 */
struct bmi160_sensor_data
{
    /*! X-axis sensor data */
    int16_t x;

    /*! Y-axis sensor data */
    int16_t y;

    /*! Z-axis sensor data */
    int16_t z;

    /*! sensor time */
    uint32_t sensortime;
};

/*!
 * @brief bmi160 aux data structure which comprises of 8 bytes of accel data
 */
struct bmi160_aux_data
{
    /*! Auxiliary data */
    uint8_t data[8];
};

/*!
 * @brief bmi160 FOC configuration structure
 */
struct bmi160_foc_conf
{
    /*! Enabling FOC in gyro
     * Assignable macros :
     *  - BMI160_ENABLE
     *  - BMI160_DISABLE
     */
    uint8_t foc_gyr_en;

    /*! Accel FOC configurations
     * Assignable macros :
     *  - BMI160_FOC_ACCEL_DISABLED
     *  - BMI160_FOC_ACCEL_POSITIVE_G
     *  - BMI160_FOC_ACCEL_NEGATIVE_G
     *  - BMI160_FOC_ACCEL_0G
     */
    uint8_t foc_acc_x;
    uint8_t foc_acc_y;
    uint8_t foc_acc_z;

    /*! Enabling offset compensation for accel in data registers
     * Assignable macros :
     *  - BMI160_ENABLE
     *  - BMI160_DISABLE
     */
    uint8_t acc_off_en;

    /*! Enabling offset compensation for gyro in data registers
     * Assignable macros :
     *  - BMI160_ENABLE
     *  - BMI160_DISABLE
     */
    uint8_t gyro_off_en;
};

/*!
 * @brief bmi160 accel gyro offsets
 */
struct bmi160_offsets
{
    /*! Accel offset for x axis */
    int8_t off_acc_x;

    /*! Accel offset for y axis */
    int8_t off_acc_y;

    /*! Accel offset for z axis */
    int8_t off_acc_z;

    /*! Gyro offset for x axis */
    int16_t off_gyro_x;

    /*! Gyro offset for y axis */
    int16_t off_gyro_y;

    /*! Gyro offset for z axis */
    int16_t off_gyro_z;
};

/*!
 * @brief FIFO aux. sensor data structure
 */
struct bmi160_aux_fifo_data
{
    /*! The value of aux. sensor x LSB data */
    uint8_t aux_x_lsb;

    /*! The value of aux. sensor x MSB data */
    uint8_t aux_x_msb;

    /*! The value of aux. sensor y LSB data */
    uint8_t aux_y_lsb;

    /*! The value of aux. sensor y MSB data */
    uint8_t aux_y_msb;

    /*! The value of aux. sensor z LSB data */
    uint8_t aux_z_lsb;

    /*! The value of aux. sensor z MSB data */
    uint8_t aux_z_msb;

    /*! The value of aux. sensor r for BMM150 LSB data */
    uint8_t aux_r_y2_lsb;

    /*! The value of aux. sensor r for BMM150 MSB data */
    uint8_t aux_r_y2_msb;
};

/*!
 * @brief bmi160 sensor select structure
 */
enum bmi160_select_sensor {
    BMI160_ACCEL_ONLY = 1,
    BMI160_GYRO_ONLY,
    BMI160_BOTH_ACCEL_AND_GYRO
};

/*!
 * @brief bmi160 sensor step detector mode structure
 */
enum bmi160_step_detect_mode {
    BMI160_STEP_DETECT_NORMAL,
    BMI160_STEP_DETECT_SENSITIVE,
    BMI160_STEP_DETECT_ROBUST,

    /*! Non recommended User defined setting */
    BMI160_STEP_DETECT_USER_DEFINE
};

/*!
 * @brief enum for auxiliary burst read selection
 */
enum bmi160_aux_read_len {
    BMI160_AUX_READ_LEN_0,
    BMI160_AUX_READ_LEN_1,
    BMI160_AUX_READ_LEN_2,
    BMI160_AUX_READ_LEN_3
};

/*!
 * @brief bmi160 sensor configuration structure
 */
struct bmi160_cfg
{
    /*! power mode */
    uint8_t power;

    /*! output data rate */
    uint8_t odr;

    /*! range */
    uint8_t range;

    /*! bandwidth */
    uint8_t bw;
};

/*!
 * @brief Aux sensor configuration structure
 */
struct bmi160_aux_cfg
{
    /*! Aux sensor, 1 - enable 0 - disable */
    uint8_t aux_sensor_enable : 1;

    /*! Aux manual/auto mode status */
    uint8_t manual_enable : 1;

    /*! Aux read burst length */
    uint8_t aux_rd_burst_len : 2;

    /*! output data rate */
    uint8_t aux_odr : 4;

    /*! i2c addr of auxiliary sensor */
    uint8_t aux_i2c_addr;
};

/*!
 * @brief bmi160 interrupt channel selection structure
 */
enum bmi160_int_channel {
    /*! Un-map both channels */
    BMI160_INT_CHANNEL_NONE,

    /*! interrupt Channel 1 */
    BMI160_INT_CHANNEL_1,

    /*! interrupt Channel 2 */
    BMI160_INT_CHANNEL_2,

    /*! Map both channels */
    BMI160_INT_CHANNEL_BOTH
};
enum bmi160_int_types {
    /*! Slope/Any-motion interrupt */
    BMI160_ACC_ANY_MOTION_INT,

    /*! Significant motion interrupt */
    BMI160_ACC_SIG_MOTION_INT,

    /*! Step detector interrupt */
    BMI160_STEP_DETECT_INT,

    /*! double tap interrupt */
    BMI160_ACC_DOUBLE_TAP_INT,

    /*! single tap interrupt */
    BMI160_ACC_SINGLE_TAP_INT,

    /*! orientation interrupt */
    BMI160_ACC_ORIENT_INT,

    /*! flat interrupt */
    BMI160_ACC_FLAT_INT,

    /*! high-g interrupt */
    BMI160_ACC_HIGH_G_INT,

    /*! low-g interrupt */
    BMI160_ACC_LOW_G_INT,

    /*! slow/no-motion interrupt */
    BMI160_ACC_SLOW_NO_MOTION_INT,

    /*! data ready interrupt  */
    BMI160_ACC_GYRO_DATA_RDY_INT,

    /*! fifo full interrupt */
    BMI160_ACC_GYRO_FIFO_FULL_INT,

    /*! fifo watermark interrupt */
    BMI160_ACC_GYRO_FIFO_WATERMARK_INT,

    /*! fifo tagging feature support */
    BMI160_FIFO_TAG_INT_PIN
};

/*!
 * @brief bmi160 active state of any & sig motion interrupt.
 */
enum bmi160_any_sig_motion_active_interrupt_state {
    /*! Both any & sig motion are disabled */
    BMI160_BOTH_ANY_SIG_MOTION_DISABLED = -1,

    /*! Any-motion selected */
    BMI160_ANY_MOTION_ENABLED,

    /*! Sig-motion selected */
    BMI160_SIG_MOTION_ENABLED
};
struct bmi160_acc_tap_int_cfg
{
#ifdef LITTLE_ENDIAN

    /*! tap threshold */
    uint16_t tap_thr : 5;

    /*! tap shock */
    uint16_t tap_shock : 1;

    /*! tap quiet */
    uint16_t tap_quiet : 1;

    /*! tap duration */
    uint16_t tap_dur : 3;

    /*! data source 0- filter & 1 pre-filter*/
    uint16_t tap_data_src : 1;

    /*! tap enable, 1 - enable, 0 - disable */
    uint16_t tap_en : 1;
#else

    /*! tap enable, 1 - enable, 0 - disable */
    uint16_t tap_en : 1;

    /*! data source 0- filter & 1 pre-filter*/
    uint16_t tap_data_src : 1;

    /*! tap duration */
    uint16_t tap_dur : 3;

    /*! tap quiet */
    uint16_t tap_quiet : 1;

    /*! tap shock */
    uint16_t tap_shock : 1;

    /*! tap threshold */
    uint16_t tap_thr : 5;
#endif
};
struct bmi160_acc_any_mot_int_cfg
{
#ifdef LITTLE_ENDIAN

    /*! 1 any-motion enable, 0 - any-motion disable */
    uint8_t anymotion_en : 1;

    /*! slope interrupt x, 1 - enable, 0 - disable */
    uint8_t anymotion_x : 1;

    /*! slope interrupt y, 1 - enable, 0 - disable */
    uint8_t anymotion_y : 1;

    /*! slope interrupt z, 1 - enable, 0 - disable */
    uint8_t anymotion_z : 1;

    /*! slope duration */
    uint8_t anymotion_dur : 2;

    /*! data source 0- filter & 1 pre-filter*/
    uint8_t anymotion_data_src : 1;

    /*! slope threshold */
    uint8_t anymotion_thr;
#else

    /*! slope threshold */
    uint8_t anymotion_thr;

    /*! data source 0- filter & 1 pre-filter*/
    uint8_t anymotion_data_src : 1;

    /*! slope duration */
    uint8_t anymotion_dur : 2;

    /*! slope interrupt z, 1 - enable, 0 - disable */
    uint8_t anymotion_z : 1;

    /*! slope interrupt y, 1 - enable, 0 - disable */
    uint8_t anymotion_y : 1;

    /*! slope interrupt x, 1 - enable, 0 - disable */
    uint8_t anymotion_x : 1;

    /*! 1 any-motion enable, 0 - any-motion disable */
    uint8_t anymotion_en : 1;
#endif
};
struct bmi160_acc_sig_mot_int_cfg
{
#ifdef LITTLE_ENDIAN

    /*! skip time of sig-motion interrupt */
    uint8_t sig_mot_skip : 2;

    /*! proof time of sig-motion interrupt */
    uint8_t sig_mot_proof : 2;

    /*! data source 0- filter & 1 pre-filter*/
    uint8_t sig_data_src : 1;

    /*! 1 - enable sig, 0 - disable sig & enable anymotion */
    uint8_t sig_en : 1;

    /*! sig-motion threshold */
    uint8_t sig_mot_thres;
#else

    /*! sig-motion threshold */
    uint8_t sig_mot_thres;

    /*! 1 - enable sig, 0 - disable sig & enable anymotion */
    uint8_t sig_en : 1;

    /*! data source 0- filter & 1 pre-filter*/
    uint8_t sig_data_src : 1;

    /*! proof time of sig-motion interrupt */
    uint8_t sig_mot_proof : 2;

    /*! skip time of sig-motion interrupt */
    uint8_t sig_mot_skip : 2;
#endif
};
struct bmi160_acc_step_detect_int_cfg
{
#ifdef LITTLE_ENDIAN

    /*! 1- step detector enable, 0- step detector disable */
    uint16_t step_detector_en : 1;

    /*! minimum threshold */
    uint16_t min_threshold : 2;

    /*! minimal detectable step time */
    uint16_t steptime_min : 3;

    /*! enable step counter mode setting */
    uint16_t step_detector_mode : 2;

    /*! minimum step buffer size*/
    uint16_t step_min_buf : 3;
#else

    /*! minimum step buffer size*/
    uint16_t step_min_buf : 3;

    /*! enable step counter mode setting */
    uint16_t step_detector_mode : 2;

    /*! minimal detectable step time */
    uint16_t steptime_min : 3;

    /*! minimum threshold */
    uint16_t min_threshold : 2;

    /*! 1- step detector enable, 0- step detector disable */
    uint16_t step_detector_en : 1;
#endif
};
struct bmi160_acc_no_motion_int_cfg
{
#ifdef LITTLE_ENDIAN

    /*! no motion interrupt x */
    uint16_t no_motion_x : 1;

    /*! no motion interrupt y */
    uint16_t no_motion_y : 1;

    /*! no motion interrupt z */
    uint16_t no_motion_z : 1;

    /*! no motion duration */
    uint16_t no_motion_dur : 6;

    /*! no motion sel , 1 - enable no-motion ,0- enable slow-motion */
    uint16_t no_motion_sel : 1;

    /*! data source 0- filter & 1 pre-filter*/
    uint16_t no_motion_src : 1;

    /*! no motion threshold */
    uint8_t no_motion_thres;
#else

    /*! no motion threshold */
    uint8_t no_motion_thres;

    /*! data source 0- filter & 1 pre-filter*/
    uint16_t no_motion_src : 1;

    /*! no motion sel , 1 - enable no-motion ,0- enable slow-motion */
    uint16_t no_motion_sel : 1;

    /*! no motion duration */
    uint16_t no_motion_dur : 6;

    /* no motion interrupt z */
    uint16_t no_motion_z : 1;

    /*! no motion interrupt y */
    uint16_t no_motion_y : 1;

    /*! no motion interrupt x */
    uint16_t no_motion_x : 1;
#endif
};
struct bmi160_acc_orient_int_cfg
{
#ifdef LITTLE_ENDIAN

    /*! thresholds for switching between the different orientations */
    uint16_t orient_mode : 2;

    /*! blocking_mode */
    uint16_t orient_blocking : 2;

    /*! Orientation interrupt hysteresis */
    uint16_t orient_hyst : 4;

    /*! Orientation interrupt theta */
    uint16_t orient_theta : 6;

    /*! Enable/disable Orientation interrupt */
    uint16_t orient_ud_en : 1;

    /*! exchange x- and z-axis in algorithm ,0 - z, 1 - x */
    uint16_t axes_ex : 1;

    /*! 1 - orient enable, 0 - orient disable */
    uint8_t orient_en : 1;
#else

    /*! 1 - orient enable, 0 - orient disable */
    uint8_t orient_en : 1;

    /*! exchange x- and z-axis in algorithm ,0 - z, 1 - x */
    uint16_t axes_ex : 1;

    /*! Enable/disable Orientation interrupt */
    uint16_t orient_ud_en : 1;

    /*! Orientation interrupt theta */
    uint16_t orient_theta : 6;

    /*! Orientation interrupt hysteresis */
    uint16_t orient_hyst : 4;

    /*! blocking_mode */
    uint16_t orient_blocking : 2;

    /*! thresholds for switching between the different orientations */
    uint16_t orient_mode : 2;
#endif
};
struct bmi160_acc_flat_detect_int_cfg
{
#ifdef LITTLE_ENDIAN

    /*! flat threshold */
    uint16_t flat_theta : 6;

    /*! flat interrupt hysteresis */
    uint16_t flat_hy : 3;

    /*! delay time for which the flat value must remain stable for the
     * flat interrupt to be generated */
    uint16_t flat_hold_time : 2;

    /*! 1 - flat enable, 0 - flat disable */
    uint16_t flat_en : 1;
#else

    /*! 1 - flat enable, 0 - flat disable */
    uint16_t flat_en : 1;

    /*! delay time for which the flat value must remain stable for the
     * flat interrupt to be generated */
    uint16_t flat_hold_time : 2;

    /*! flat interrupt hysteresis */
    uint16_t flat_hy : 3;

    /*! flat threshold */
    uint16_t flat_theta : 6;
#endif
};
struct bmi160_acc_low_g_int_cfg
{
#ifdef LITTLE_ENDIAN

    /*! low-g interrupt trigger delay */
    uint8_t low_dur;

    /*! low-g interrupt trigger threshold */
    uint8_t low_thres;

    /*! hysteresis of low-g interrupt */
    uint8_t low_hyst : 2;

    /*! 0 - single-axis mode ,1 - axis-summing mode */
    uint8_t low_mode : 1;

    /*! data source 0- filter & 1 pre-filter */
    uint8_t low_data_src : 1;

    /*! 1 - enable low-g, 0 - disable low-g */
    uint8_t low_en : 1;
#else

    /*! 1 - enable low-g, 0 - disable low-g */
    uint8_t low_en : 1;

    /*! data source 0- filter & 1 pre-filter */
    uint8_t low_data_src : 1;

    /*! 0 - single-axis mode ,1 - axis-summing mode */
    uint8_t low_mode : 1;

    /*! hysteresis of low-g interrupt */
    uint8_t low_hyst : 2;

    /*! low-g interrupt trigger threshold */
    uint8_t low_thres;

    /*! low-g interrupt trigger delay */
    uint8_t low_dur;
#endif
};
struct bmi160_acc_high_g_int_cfg
{
#ifdef LITTLE_ENDIAN

    /*! High-g interrupt x, 1 - enable, 0 - disable */
    uint8_t high_g_x : 1;

    /*! High-g interrupt y, 1 - enable, 0 - disable */
    uint8_t high_g_y : 1;

    /*! High-g interrupt z, 1 - enable, 0 - disable */
    uint8_t high_g_z : 1;

    /*! High-g hysteresis  */
    uint8_t high_hy : 2;

    /*! data source 0- filter & 1 pre-filter */
    uint8_t high_data_src : 1;

    /*! High-g threshold */
    uint8_t high_thres;

    /*! High-g duration */
    uint8_t high_dur;
#else

    /*! High-g duration */
    uint8_t high_dur;

    /*! High-g threshold */
    uint8_t high_thres;

    /*! data source 0- filter & 1 pre-filter */
    uint8_t high_data_src : 1;

    /*! High-g hysteresis  */
    uint8_t high_hy : 2;

    /*! High-g interrupt z, 1 - enable, 0 - disable */
    uint8_t high_g_z : 1;

    /*! High-g interrupt y, 1 - enable, 0 - disable */
    uint8_t high_g_y : 1;

    /*! High-g interrupt x, 1 - enable, 0 - disable */
    uint8_t high_g_x : 1;
#endif
};
struct bmi160_int_pin_settg
{
#ifdef LITTLE_ENDIAN

    /*! To enable either INT1 or INT2 pin as output.
     * 0- output disabled ,1- output enabled */
    uint16_t output_en : 1;

    /*! 0 - push-pull 1- open drain,only valid if output_en is set 1 */
    uint16_t output_mode : 1;

    /*! 0 - active low , 1 - active high level.
     * if output_en is 1,this applies to interrupts,else PMU_trigger */
    uint16_t output_type : 1;

    /*! 0 - level trigger , 1 - edge trigger  */
    uint16_t edge_ctrl : 1;

    /*! To enable either INT1 or INT2 pin as input.
     * 0 - input disabled ,1 - input enabled */
    uint16_t input_en : 1;

    /*! latch duration*/
    uint16_t latch_dur : 4;
#else

    /*! latch duration*/
    uint16_t latch_dur : 4;

    /*! Latched,non-latched or temporary interrupt modes */
    uint16_t input_en : 1;

    /*! 1 - edge trigger, 0 - level trigger */
    uint16_t edge_ctrl : 1;

    /*! 0 - active low , 1 - active high level.
     * if output_en is 1,this applies to interrupts,else PMU_trigger */
    uint16_t output_type : 1;

    /*! 0 - push-pull , 1 - open drain,only valid if output_en is set 1 */
    uint16_t output_mode : 1;

    /*! To enable either INT1 or INT2 pin as output.
     * 0 - output disabled , 1 - output enabled */
    uint16_t output_en : 1;
#endif
};
union bmi160_int_type_cfg
{
    /*! Tap interrupt structure */
    struct bmi160_acc_tap_int_cfg acc_tap_int;

    /*! Slope interrupt structure */
    struct bmi160_acc_any_mot_int_cfg acc_any_motion_int;

    /*! Significant motion interrupt structure */
    struct bmi160_acc_sig_mot_int_cfg acc_sig_motion_int;

    /*! Step detector interrupt structure */
    struct bmi160_acc_step_detect_int_cfg acc_step_detect_int;

    /*! No motion interrupt structure */
    struct bmi160_acc_no_motion_int_cfg acc_no_motion_int;

    /*! Orientation interrupt structure */
    struct bmi160_acc_orient_int_cfg acc_orient_int;

    /*! Flat interrupt structure */
    struct bmi160_acc_flat_detect_int_cfg acc_flat_int;

    /*! Low-g interrupt structure */
    struct bmi160_acc_low_g_int_cfg acc_low_g_int;

    /*! High-g interrupt structure */
    struct bmi160_acc_high_g_int_cfg acc_high_g_int;
};
struct bmi160_int_settg
{
    /*! Interrupt channel */
    enum bmi160_int_channel int_channel;

    /*! Select Interrupt */
    enum bmi160_int_types int_type;

    /*! Structure configuring Interrupt pins */
    struct bmi160_int_pin_settg int_pin_settg;

    /*! Union configures required interrupt */
    union bmi160_int_type_cfg int_type_cfg;

    /*! FIFO FULL INT 1-enable, 0-disable */
    uint8_t fifo_full_int_en : 1;

    /*! FIFO WTM INT 1-enable, 0-disable */
    uint8_t fifo_wtm_int_en : 1;
};

/*!
 *  @brief This structure holds the information for usage of
 *  FIFO by the user.
 */
struct bmi160_fifo_frame
{
    /*! Data buffer of user defined length is to be mapped here */
    uint8_t *data;

    /*! While calling the API  "bmi160_get_fifo_data" , length stores
     *  number of bytes in FIFO to be read (specified by user as input)
     *  and after execution of the API ,number of FIFO data bytes
     *  available is provided as an output to user
     */
    uint16_t length;

    /*! FIFO time enable */
    uint8_t fifo_time_enable;

    /*! Enabling of the FIFO header to stream in header mode */
    uint8_t fifo_header_enable;

    /*! Streaming of the Accelerometer, Gyroscope
     * sensor data or both in FIFO */
    uint8_t fifo_data_enable;

    /*! Will be equal to length when no more frames are there to parse */
    uint16_t accel_byte_start_idx;

    /*! Will be equal to length when no more frames are there to parse */
    uint16_t gyro_byte_start_idx;

    /*! Will be equal to length when no more frames are there to parse */
    uint16_t aux_byte_start_idx;

    /*! Value of FIFO sensor time time */
    uint32_t sensor_time;

    /*! Value of Skipped frame counts */
    uint8_t skipped_frame_count;
};
struct bmi160_dev
{
    /*! Chip Id */
    uint8_t chip_id;

    /*! Device Id */
    uint8_t id;

    /*! 0 - I2C , 1 - SPI Interface */
    uint8_t intf;

// #ifdef BMX160_USE_I2C
    i2c_t i2c_dev;
// #endif

    /*! Hold active interrupts status for any and sig motion
     *  0 - Any-motion enable, 1 - Sig-motion enable,
     *  -1 neither any-motion nor sig-motion selected */
    enum bmi160_any_sig_motion_active_interrupt_state any_sig_sel;

    /*! Structure to configure Accel sensor */
    struct bmi160_cfg accel_cfg;

    /*! Structure to hold previous/old accel config parameters.
     * This is used at driver level to prevent overwriting of same
     * data, hence user does not change it in the code */
    struct bmi160_cfg prev_accel_cfg;

    /*! Structure to configure Gyro sensor */
    struct bmi160_cfg gyro_cfg;

    /*! Structure to hold previous/old gyro config parameters.
     * This is used at driver level to prevent overwriting of same
     * data, hence user does not change it in the code */
    struct bmi160_cfg prev_gyro_cfg;

    /*! Structure to configure the auxiliary sensor */
    struct bmi160_aux_cfg aux_cfg;

    /*! Structure to hold previous/old aux config parameters.
     * This is used at driver level to prevent overwriting of same
     * data, hence user does not change it in the code */
    struct bmi160_aux_cfg prev_aux_cfg;

    /*! FIFO related configurations */
    struct bmi160_fifo_frame *fifo;

    /*! Read function pointer */
    bmi160_read_fptr_t read;

    /*! Write function pointer */
    bmi160_write_fptr_t write;

    /*!  Delay function pointer */
    bmi160_delay_fptr_t delay_ms;

    /*! User set read/write length */
    uint16_t read_write_len;
};


/**
 * @brief   Initialize the given BMX160 device
 *
 * @param[out] dev      device descriptor of the given BMX160 device
 * @param[in]  params   static configuration parameters
 *
 * @return  BMX160_OK on success
 * @return  BMX160_ERR_BUS on bus error
 * @return  BMX160_ERR_NODEV if no corresponding device was found on the bus
 */
int bmx160_init(struct bmi160_dev bmi);



#ifdef __cplusplus
}
#endif

#endif /* BMX160_H */
/** @} */
