/*
 * Copyright (C) ?
 *
 */

/**
 * @ingroup     drivers_bmx160
 *
 * @{
 * @file
 * @brief       Default configuration for BMX160
 *
 * @author      Geovane Fedrecheski <geonnave@gmail.com>
 */

#ifndef BMX_160_PARAMS_H
#define BMX_160_PARAMS_H

#include "board.h"
#include "bmx160.h"
#include "bmx160_internals.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Set default configuration parameters for the BMX160
 * @{
 */
#ifdef BMX160_USE_SPI
/* SPI configuration */
// #ifndef BMX160_PARAM_SPI
// #define BMX160_PARAM_SPI            SPI_DEV(0)
// #endif
// #ifndef BMX160_PARAM_CLK
// #define BMX160_PARAM_CLK            SPI_CLK_5MHZ
// #endif
// #ifndef BMX160_PARAM_CS
// #define BMX160_PARAM_CS             GPIO_PIN(0, 0)
// #endif
#else
/* I2C configuration */
#ifndef BMX160_PARAM_I2C_DEV
#define BMX160_PARAM_I2C_DEV        I2C_DEV(0)
#endif
#ifndef BMX160_PARAM_I2C_ADDR
#define BMX160_PARAM_I2C_ADDR       (0x69)
#endif
#endif

#ifndef BMX160_PARAM_ACCEL_FACTOR
#define BMX160_PARAM_ACCEL_FACTOR 2048.0
#endif
#ifndef BMX160_PARAM_GYRO_FACTOR
#define BMX160_PARAM_GYRO_FACTOR 16.4
#endif

// #define BMX160_PARAM_MISC                   \//
//         .t_sb = BMX160_SB_0_5,              \//
//         .filter = BMX160_FILTER_OFF,        \//
//         .run_mode = BMX160_MODE_FORCED,     \//
//         .temp_oversample = BMX160_OSRS_X1,  \//
//         .press_oversample = BMX160_OSRS_X1, \//
//         .humid_oversample = BMX160_OSRS_X1, \//

// /* Defaults for Weather Monitoring */
// #ifndef BMX160_PARAMS
// #ifdef BMX160_USE_SPI
// #define BMX160_PARAMS                       \//
//     {                                       \//
//         .spi = BMX160_PARAM_SPI,            \//
//         .clk = BMX160_PARAM_CLK,            \//
//         .cs  = BMX160_PARAM_CS,             \//
//         BMX160_PARAM_MISC                   \//
//     }
// #else
// #define BMX160_PARAMS                       \//
//     {                                       \//
//         .i2c_dev  = BMX160_PARAM_I2C_DEV,   \//
//         .i2c_addr = BMX160_PARAM_I2C_ADDR,  \//
//         BMX160_PARAM_MISC                   \//
//     }
// #endif
// #endif
/**@}*/

// /**
//  * @brief   Configure BMX160
//  */
// static const bmx160_params_t bmx160_params[] =
// {
//     BMX160_PARAMS
// };

// /**
//  * @brief   The number of configured sensors
//  */
// #define BMX160_NUMOF    ARRAY_SIZE(bmx160_params)

// /**
//  * @brief   Configuration details of SAUL registry entries
//  *
//  * This two dimensional array contains static details of the sensors
//  * for each device. Please be aware that the indexes are used in
//  * aut160nit_bmx160, so make sure the indexes match.
//  */
// static const saul_reg_info_t bmx160_saul_reg_info[BMX160_NUMOF] =
// {
// #if defined(MODULE_BMX160_SPI) || defined(MODULE_BMX160_I2C)
//         { .name = "bmx160" }
// #else
//         { .name = "bmi160" }
// #endif
// };

#ifdef __cplusplus
}
#endif

#endif /* BMX_160_PARAMS_H */