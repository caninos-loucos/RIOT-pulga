/*
 * Copyright (C) ?
 */

/**
 * @ingroup     drivers_bmx160
 * @brief       Internal addresses, registers, constants for the BMX160 family sensors.
 * @{
 * @file
 * @brief       Internal addresses, registers, constants for the BMX160 family sensors.
 *
 * @author      Geovane Fedrecheski <geonnave@gmail.com>
 */

#ifndef BMX160_INTERNALS_H
#define BMX160_INTERNALS_H

/*************************** C types headers *****************************/
#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/kernel.h>
#else
#include <stdint.h>
#include <stddef.h>
#endif

/*************************** Common macros   *****************************/

#if !defined(UINT8_C) && !defined(INT8_C)
#define INT8_C(x)                                 S8_C(x)
#define UINT8_C(x)                                U8_C(x)
#endif

#if !defined(UINT16_C) && !defined(INT16_C)
#define INT16_C(x)                                S16_C(x)
#define UINT16_C(x)                               U16_C(x)
#endif

#if !defined(INT32_C) && !defined(UINT32_C)
#define INT32_C(x)                                S32_C(x)
#define UINT32_C(x)                               U32_C(x)
#endif

#if !defined(INT64_C) && !defined(UINT64_C)
#define INT64_C(x)                                S64_C(x)
#define UINT64_C(x)                               U64_C(x)
#endif

/**@}*/
/**\name C standard macros */
#ifndef NULL
#ifdef __cplusplus
#define NULL                                      0
#else
#define NULL                                      ((void *) 0)
#endif
#endif

/*************************** Sensor macros   *****************************/
/* Test for an endian machine */
#ifndef __ORDER_LITTLE_ENDIAN__
#define __ORDER_LITTLE_ENDIAN__                   0
#endif

#ifndef __BYTE_ORDER__
#define __BYTE_ORDER__                            __ORDER_LITTLE_ENDIAN__
#endif

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#ifndef LITTLE_ENDIAN
#define LITTLE_ENDIAN                             1
#endif
#elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
#ifndef BIG_ENDIAN
#define BIG_ENDIAN                                1
#endif
#else
#error "Code does not support Endian format of the processor"
#endif

/** Mask definitions */
#define BMI160_ACCEL_BW_MASK                      UINT8_C(0x70)
#define BMI160_ACCEL_ODR_MASK                     UINT8_C(0x0F)
#define BMI160_ACCEL_UNDERSAMPLING_MASK           UINT8_C(0x80)
#define BMI160_ACCEL_RANGE_MASK                   UINT8_C(0x0F)
#define BMI160_GYRO_BW_MASK                       UINT8_C(0x30)
#define BMI160_GYRO_ODR_MASK                      UINT8_C(0x0F)
#define BMI160_GYRO_RANGE_MASK                    UINT8_C(0x07)

#define BMI160_ACCEL_BW_POS                       UINT8_C(4)
#define BMI160_GYRO_BW_POS                        UINT8_C(4)

/** Mask definitions for INT_EN registers */
#define BMI160_ANY_MOTION_X_INT_EN_MASK           UINT8_C(0x01)
#define BMI160_HIGH_G_X_INT_EN_MASK               UINT8_C(0x01)
#define BMI160_NO_MOTION_X_INT_EN_MASK            UINT8_C(0x01)
#define BMI160_ANY_MOTION_Y_INT_EN_MASK           UINT8_C(0x02)
#define BMI160_HIGH_G_Y_INT_EN_MASK               UINT8_C(0x02)
#define BMI160_NO_MOTION_Y_INT_EN_MASK            UINT8_C(0x02)
#define BMI160_ANY_MOTION_Z_INT_EN_MASK           UINT8_C(0x04)
#define BMI160_HIGH_G_Z_INT_EN_MASK               UINT8_C(0x04)
#define BMI160_NO_MOTION_Z_INT_EN_MASK            UINT8_C(0x04)
#define BMI160_SIG_MOTION_INT_EN_MASK             UINT8_C(0x07)
#define BMI160_ANY_MOTION_ALL_INT_EN_MASK         UINT8_C(0x07)
#define BMI160_STEP_DETECT_INT_EN_MASK            UINT8_C(0x08)
#define BMI160_DOUBLE_TAP_INT_EN_MASK             UINT8_C(0x10)
#define BMI160_SINGLE_TAP_INT_EN_MASK             UINT8_C(0x20)
#define BMI160_FIFO_FULL_INT_EN_MASK              UINT8_C(0x20)
#define BMI160_ORIENT_INT_EN_MASK                 UINT8_C(0x40)
#define BMI160_FIFO_WATERMARK_INT_EN_MASK         UINT8_C(0x40)
#define BMI160_LOW_G_INT_EN_MASK                  UINT8_C(0x08)
#define BMI160_STEP_DETECT_EN_MASK                UINT8_C(0x08)
#define BMI160_FLAT_INT_EN_MASK                   UINT8_C(0x80)
#define BMI160_DATA_RDY_INT_EN_MASK               UINT8_C(0x10)

/** PMU status Macros */
#define BMI160_AUX_PMU_SUSPEND                    UINT8_C(0x00)
#define BMI160_AUX_PMU_NORMAL                     UINT8_C(0x01)
#define BMI160_AUX_PMU_LOW_POWER                  UINT8_C(0x02)

#define BMI160_GYRO_PMU_SUSPEND                   UINT8_C(0x00)
#define BMI160_GYRO_PMU_NORMAL                    UINT8_C(0x01)
#define BMI160_GYRO_PMU_FSU                       UINT8_C(0x03)

#define BMI160_ACCEL_PMU_SUSPEND                  UINT8_C(0x00)
#define BMI160_ACCEL_PMU_NORMAL                   UINT8_C(0x01)
#define BMI160_ACCEL_PMU_LOW_POWER                UINT8_C(0x02)

/** Mask definitions for INT_OUT_CTRL register */
#define BMI160_INT1_EDGE_CTRL_MASK                UINT8_C(0x01)
#define BMI160_INT1_OUTPUT_MODE_MASK              UINT8_C(0x04)
#define BMI160_INT1_OUTPUT_TYPE_MASK              UINT8_C(0x02)
#define BMI160_INT1_OUTPUT_EN_MASK                UINT8_C(0x08)
#define BMI160_INT2_EDGE_CTRL_MASK                UINT8_C(0x10)
#define BMI160_INT2_OUTPUT_MODE_MASK              UINT8_C(0x40)
#define BMI160_INT2_OUTPUT_TYPE_MASK              UINT8_C(0x20)
#define BMI160_INT2_OUTPUT_EN_MASK                UINT8_C(0x80)

/** Mask definitions for INT_LATCH register */
#define BMI160_INT1_INPUT_EN_MASK                 UINT8_C(0x10)
#define BMI160_INT2_INPUT_EN_MASK                 UINT8_C(0x20)
#define BMI160_INT_LATCH_MASK                     UINT8_C(0x0F)

/** Mask definitions for INT_MAP register */
#define BMI160_INT1_LOW_G_MASK                    UINT8_C(0x01)
#define BMI160_INT1_HIGH_G_MASK                   UINT8_C(0x02)
#define BMI160_INT1_SLOPE_MASK                    UINT8_C(0x04)
#define BMI160_INT1_NO_MOTION_MASK                UINT8_C(0x08)
#define BMI160_INT1_DOUBLE_TAP_MASK               UINT8_C(0x10)
#define BMI160_INT1_SINGLE_TAP_MASK               UINT8_C(0x20)
#define BMI160_INT1_FIFO_FULL_MASK                UINT8_C(0x20)
#define BMI160_INT1_FIFO_WM_MASK                  UINT8_C(0x40)
#define BMI160_INT1_ORIENT_MASK                   UINT8_C(0x40)
#define BMI160_INT1_FLAT_MASK                     UINT8_C(0x80)
#define BMI160_INT1_DATA_READY_MASK               UINT8_C(0x80)
#define BMI160_INT2_LOW_G_MASK                    UINT8_C(0x01)
#define BMI160_INT1_LOW_STEP_DETECT_MASK          UINT8_C(0x01)
#define BMI160_INT2_LOW_STEP_DETECT_MASK          UINT8_C(0x01)
#define BMI160_INT2_HIGH_G_MASK                   UINT8_C(0x02)
#define BMI160_INT2_FIFO_FULL_MASK                UINT8_C(0x02)
#define BMI160_INT2_FIFO_WM_MASK                  UINT8_C(0x04)
#define BMI160_INT2_SLOPE_MASK                    UINT8_C(0x04)
#define BMI160_INT2_DATA_READY_MASK               UINT8_C(0x08)
#define BMI160_INT2_NO_MOTION_MASK                UINT8_C(0x08)
#define BMI160_INT2_DOUBLE_TAP_MASK               UINT8_C(0x10)
#define BMI160_INT2_SINGLE_TAP_MASK               UINT8_C(0x20)
#define BMI160_INT2_ORIENT_MASK                   UINT8_C(0x40)
#define BMI160_INT2_FLAT_MASK                     UINT8_C(0x80)

/** Mask definitions for INT_DATA register */
#define BMI160_TAP_SRC_INT_MASK                   UINT8_C(0x08)
#define BMI160_LOW_HIGH_SRC_INT_MASK              UINT8_C(0x80)
#define BMI160_MOTION_SRC_INT_MASK                UINT8_C(0x80)

/** Mask definitions for INT_MOTION register */
#define BMI160_SLOPE_INT_DUR_MASK                 UINT8_C(0x03)
#define BMI160_NO_MOTION_INT_DUR_MASK             UINT8_C(0xFC)
#define BMI160_NO_MOTION_SEL_BIT_MASK             UINT8_C(0x01)

/** Mask definitions for INT_TAP register */
#define BMI160_TAP_DUR_MASK                       UINT8_C(0x07)
#define BMI160_TAP_SHOCK_DUR_MASK                 UINT8_C(0x40)
#define BMI160_TAP_QUIET_DUR_MASK                 UINT8_C(0x80)
#define BMI160_TAP_THRES_MASK                     UINT8_C(0x1F)

/** Mask definitions for INT_FLAT register */
#define BMI160_FLAT_THRES_MASK                    UINT8_C(0x3F)
#define BMI160_FLAT_HOLD_TIME_MASK                UINT8_C(0x30)
#define BMI160_FLAT_HYST_MASK                     UINT8_C(0x07)

/** Mask definitions for INT_LOWHIGH register */
#define BMI160_LOW_G_HYST_MASK                    UINT8_C(0x03)
#define BMI160_LOW_G_LOW_MODE_MASK                UINT8_C(0x04)
#define BMI160_HIGH_G_HYST_MASK                   UINT8_C(0xC0)

/** Mask definitions for INT_SIG_MOTION register */
#define BMI160_SIG_MOTION_SEL_MASK                UINT8_C(0x02)
#define BMI160_SIG_MOTION_SKIP_MASK               UINT8_C(0x0C)
#define BMI160_SIG_MOTION_PROOF_MASK              UINT8_C(0x30)

/** Mask definitions for INT_ORIENT register */
#define BMI160_ORIENT_MODE_MASK                   UINT8_C(0x03)
#define BMI160_ORIENT_BLOCK_MASK                  UINT8_C(0x0C)
#define BMI160_ORIENT_HYST_MASK                   UINT8_C(0xF0)
#define BMI160_ORIENT_THETA_MASK                  UINT8_C(0x3F)
#define BMI160_ORIENT_UD_ENABLE                   UINT8_C(0x40)
#define BMI160_AXES_EN_MASK                       UINT8_C(0x80)

/** Mask definitions for FIFO_CONFIG register */
#define BMI160_FIFO_GYRO                          UINT8_C(0x80)
#define BMI160_FIFO_ACCEL                         UINT8_C(0x40)
#define BMI160_FIFO_AUX                           UINT8_C(0x20)
#define BMI160_FIFO_TAG_INT1                      UINT8_C(0x08)
#define BMI160_FIFO_TAG_INT2                      UINT8_C(0x04)
#define BMI160_FIFO_TIME                          UINT8_C(0x02)
#define BMI160_FIFO_HEADER                        UINT8_C(0x10)
#define BMI160_FIFO_CONFIG_1_MASK                 UINT8_C(0xFE)

/** Mask definitions for STEP_CONF register */
#define BMI160_STEP_COUNT_EN_BIT_MASK             UINT8_C(0x08)
#define BMI160_STEP_DETECT_MIN_THRES_MASK         UINT8_C(0x18)
#define BMI160_STEP_DETECT_STEPTIME_MIN_MASK      UINT8_C(0x07)
#define BMI160_STEP_MIN_BUF_MASK                  UINT8_C(0x07)

/** Mask definition for FIFO Header Data Tag */
#define BMI160_FIFO_TAG_INTR_MASK                 UINT8_C(0xFC)

/** Fifo byte counter mask definitions */
#define BMI160_FIFO_BYTE_COUNTER_MASK             UINT8_C(0x07)

/** Enable/disable bit value */
#define BMI160_ENABLE                             UINT8_C(0x01)
#define BMI160_DISABLE                            UINT8_C(0x00)

/** Latch Duration */
#define BMI160_LATCH_DUR_NONE                     UINT8_C(0x00)
#define BMI160_LATCH_DUR_312_5_MICRO_SEC          UINT8_C(0x01)
#define BMI160_LATCH_DUR_625_MICRO_SEC            UINT8_C(0x02)
#define BMI160_LATCH_DUR_1_25_MILLI_SEC           UINT8_C(0x03)
#define BMI160_LATCH_DUR_2_5_MILLI_SEC            UINT8_C(0x04)
#define BMI160_LATCH_DUR_5_MILLI_SEC              UINT8_C(0x05)
#define BMI160_LATCH_DUR_10_MILLI_SEC             UINT8_C(0x06)
#define BMI160_LATCH_DUR_20_MILLI_SEC             UINT8_C(0x07)
#define BMI160_LATCH_DUR_40_MILLI_SEC             UINT8_C(0x08)
#define BMI160_LATCH_DUR_80_MILLI_SEC             UINT8_C(0x09)
#define BMI160_LATCH_DUR_160_MILLI_SEC            UINT8_C(0x0A)
#define BMI160_LATCH_DUR_320_MILLI_SEC            UINT8_C(0x0B)
#define BMI160_LATCH_DUR_640_MILLI_SEC            UINT8_C(0x0C)
#define BMI160_LATCH_DUR_1_28_SEC                 UINT8_C(0x0D)
#define BMI160_LATCH_DUR_2_56_SEC                 UINT8_C(0x0E)
#define BMI160_LATCHED                            UINT8_C(0x0F)

/** BMI160 Register map */
#define BMI160_CHIP_ID_ADDR                       UINT8_C(0x00)
#define BMI160_ERROR_REG_ADDR                     UINT8_C(0x02)
#define BMI160_PMU_STATUS_ADDR                    UINT8_C(0x03)
#define BMI160_AUX_DATA_ADDR                      UINT8_C(0x04)
#define BMI160_GYRO_DATA_ADDR                     UINT8_C(0x0C)
#define BMI160_ACCEL_DATA_ADDR                    UINT8_C(0x12)
#define BMI160_STATUS_ADDR                        UINT8_C(0x1B)
#define BMI160_INT_STATUS_ADDR                    UINT8_C(0x1C)
#define BMI160_FIFO_LENGTH_ADDR                   UINT8_C(0x22)
#define BMI160_FIFO_DATA_ADDR                     UINT8_C(0x24)
#define BMI160_ACCEL_CONFIG_ADDR                  UINT8_C(0x40)
#define BMI160_ACCEL_RANGE_ADDR                   UINT8_C(0x41)
#define BMI160_GYRO_CONFIG_ADDR                   UINT8_C(0x42)
#define BMI160_GYRO_RANGE_ADDR                    UINT8_C(0x43)
#define BMI160_AUX_ODR_ADDR                       UINT8_C(0x44)
#define BMI160_FIFO_DOWN_ADDR                     UINT8_C(0x45)
#define BMI160_FIFO_CONFIG_0_ADDR                 UINT8_C(0x46)
#define BMI160_FIFO_CONFIG_1_ADDR                 UINT8_C(0x47)
#define BMI160_AUX_IF_0_ADDR                      UINT8_C(0x4B)
#define BMI160_AUX_IF_1_ADDR                      UINT8_C(0x4C)
#define BMI160_AUX_IF_2_ADDR                      UINT8_C(0x4D)
#define BMI160_AUX_IF_3_ADDR                      UINT8_C(0x4E)
#define BMI160_AUX_IF_4_ADDR                      UINT8_C(0x4F)
#define BMI160_INT_ENABLE_0_ADDR                  UINT8_C(0x50)
#define BMI160_INT_ENABLE_1_ADDR                  UINT8_C(0x51)
#define BMI160_INT_ENABLE_2_ADDR                  UINT8_C(0x52)
#define BMI160_INT_OUT_CTRL_ADDR                  UINT8_C(0x53)
#define BMI160_INT_LATCH_ADDR                     UINT8_C(0x54)
#define BMI160_INT_MAP_0_ADDR                     UINT8_C(0x55)
#define BMI160_INT_MAP_1_ADDR                     UINT8_C(0x56)
#define BMI160_INT_MAP_2_ADDR                     UINT8_C(0x57)
#define BMI160_INT_DATA_0_ADDR                    UINT8_C(0x58)
#define BMI160_INT_DATA_1_ADDR                    UINT8_C(0x59)
#define BMI160_INT_LOWHIGH_0_ADDR                 UINT8_C(0x5A)
#define BMI160_INT_LOWHIGH_1_ADDR                 UINT8_C(0x5B)
#define BMI160_INT_LOWHIGH_2_ADDR                 UINT8_C(0x5C)
#define BMI160_INT_LOWHIGH_3_ADDR                 UINT8_C(0x5D)
#define BMI160_INT_LOWHIGH_4_ADDR                 UINT8_C(0x5E)
#define BMI160_INT_MOTION_0_ADDR                  UINT8_C(0x5F)
#define BMI160_INT_MOTION_1_ADDR                  UINT8_C(0x60)
#define BMI160_INT_MOTION_2_ADDR                  UINT8_C(0x61)
#define BMI160_INT_MOTION_3_ADDR                  UINT8_C(0x62)
#define BMI160_INT_TAP_0_ADDR                     UINT8_C(0x63)
#define BMI160_INT_TAP_1_ADDR                     UINT8_C(0x64)
#define BMI160_INT_ORIENT_0_ADDR                  UINT8_C(0x65)
#define BMI160_INT_ORIENT_1_ADDR                  UINT8_C(0x66)
#define BMI160_INT_FLAT_0_ADDR                    UINT8_C(0x67)
#define BMI160_INT_FLAT_1_ADDR                    UINT8_C(0x68)
#define BMI160_FOC_CONF_ADDR                      UINT8_C(0x69)
#define BMI160_CONF_ADDR                          UINT8_C(0x6A)

#define BMI160_IF_CONF_ADDR                       UINT8_C(0x6B)
#define BMI160_SELF_TEST_ADDR                     UINT8_C(0x6D)
#define BMI160_OFFSET_ADDR                        UINT8_C(0x71)
#define BMI160_OFFSET_CONF_ADDR                   UINT8_C(0x77)
#define BMI160_INT_STEP_CNT_0_ADDR                UINT8_C(0x78)
#define BMI160_INT_STEP_CONFIG_0_ADDR             UINT8_C(0x7A)
#define BMI160_INT_STEP_CONFIG_1_ADDR             UINT8_C(0x7B)
#define BMI160_COMMAND_REG_ADDR                   UINT8_C(0x7E)
#define BMI160_SPI_COMM_TEST_ADDR                 UINT8_C(0x7F)
#define BMI160_INTL_PULLUP_CONF_ADDR              UINT8_C(0x85)

/** Error code definitions */
#define BMI160_OK                                 INT8_C(0)
#define BMI160_E_NULL_PTR                         INT8_C(-1)
#define BMI160_E_COM_FAIL                         INT8_C(-2)
#define BMI160_E_DEV_NOT_FOUND                    INT8_C(-3)
#define BMI160_E_OUT_OF_RANGE                     INT8_C(-4)
#define BMI160_E_INVALID_INPUT                    INT8_C(-5)
#define BMI160_E_ACCEL_ODR_BW_INVALID             INT8_C(-6)
#define BMI160_E_GYRO_ODR_BW_INVALID              INT8_C(-7)
#define BMI160_E_LWP_PRE_FLTR_INT_INVALID         INT8_C(-8)
#define BMI160_E_LWP_PRE_FLTR_INVALID             INT8_C(-9)
#define BMI160_E_AUX_NOT_FOUND                    INT8_C(-10)
#define BMI160_E_FOC_FAILURE                      INT8_C(-11)
#define BMI160_E_READ_WRITE_LENGTH_INVALID        INT8_C(-12)
#define BMI160_E_INVALID_CONFIG                   INT8_C(-13)

/**\name API warning codes */
#define BMI160_W_GYRO_SELF_TEST_FAIL              INT8_C(1)
#define BMI160_W_ACCEl_SELF_TEST_FAIL             INT8_C(2)

/** BMX160 unique chip identifier (BMI160 is 0xD1) */
#define BMI160_CHIP_ID                            UINT8_C(0xD8)

/** Soft reset command */
#define BMI160_SOFT_RESET_CMD                     UINT8_C(0xb6)
#define BMI160_SOFT_RESET_DELAY_MS                UINT8_C(1)

/** Start FOC command */
#define BMI160_START_FOC_CMD                      UINT8_C(0x03)

/** NVM backup enabling command */
#define BMI160_NVM_BACKUP_EN                      UINT8_C(0xA0)

/* Delay in ms settings */
#define BMI160_ACCEL_DELAY_MS                     UINT8_C(5)
#define BMI160_GYRO_DELAY_MS                      UINT8_C(80)
#define BMI160_ONE_MS_DELAY                       UINT8_C(1)
#define BMI160_AUX_COM_DELAY                      UINT8_C(10)
#define BMI160_GYRO_SELF_TEST_DELAY               UINT8_C(20)
#define BMI160_ACCEL_SELF_TEST_DELAY              UINT8_C(50)

/** Self test configurations */
#define BMI160_ACCEL_SELF_TEST_CONFIG             UINT8_C(0x2C)
#define BMI160_ACCEL_SELF_TEST_POSITIVE_EN        UINT8_C(0x0D)
#define BMI160_ACCEL_SELF_TEST_NEGATIVE_EN        UINT8_C(0x09)
#define BMI160_ACCEL_SELF_TEST_LIMIT              UINT16_C(8192)

/** Power mode settings */
/* Accel power mode */
#define BMI160_ACCEL_NORMAL_MODE                  UINT8_C(0x11)
#define BMI160_ACCEL_LOWPOWER_MODE                UINT8_C(0x12)
#define BMI160_ACCEL_SUSPEND_MODE                 UINT8_C(0x10)

/* Gyro power mode */
#define BMI160_GYRO_SUSPEND_MODE                  UINT8_C(0x14)
#define BMI160_GYRO_NORMAL_MODE                   UINT8_C(0x15)
#define BMI160_GYRO_FASTSTARTUP_MODE              UINT8_C(0x17)

/* Aux power mode */
#define BMI160_AUX_SUSPEND_MODE                   UINT8_C(0x18)
#define BMI160_AUX_NORMAL_MODE                    UINT8_C(0x19)
#define BMI160_AUX_LOWPOWER_MODE                  UINT8_C(0x1A)

/** Range settings */
/* Accel Range */
#define BMI160_ACCEL_RANGE_2G                     UINT8_C(0x03)
#define BMI160_ACCEL_RANGE_4G                     UINT8_C(0x05)
#define BMI160_ACCEL_RANGE_8G                     UINT8_C(0x08)
#define BMI160_ACCEL_RANGE_16G                    UINT8_C(0x0C)

/* Gyro Range */
#define BMI160_GYRO_RANGE_2000_DPS                UINT8_C(0x00)
#define BMI160_GYRO_RANGE_1000_DPS                UINT8_C(0x01)
#define BMI160_GYRO_RANGE_500_DPS                 UINT8_C(0x02)
#define BMI160_GYRO_RANGE_250_DPS                 UINT8_C(0x03)
#define BMI160_GYRO_RANGE_125_DPS                 UINT8_C(0x04)

/** Bandwidth settings */
/* Accel Bandwidth */
#define BMI160_ACCEL_BW_OSR4_AVG1                 UINT8_C(0x00)
#define BMI160_ACCEL_BW_OSR2_AVG2                 UINT8_C(0x01)
#define BMI160_ACCEL_BW_NORMAL_AVG4               UINT8_C(0x02)
#define BMI160_ACCEL_BW_RES_AVG8                  UINT8_C(0x03)
#define BMI160_ACCEL_BW_RES_AVG16                 UINT8_C(0x04)
#define BMI160_ACCEL_BW_RES_AVG32                 UINT8_C(0x05)
#define BMI160_ACCEL_BW_RES_AVG64                 UINT8_C(0x06)
#define BMI160_ACCEL_BW_RES_AVG128                UINT8_C(0x07)

#define BMI160_GYRO_BW_OSR4_MODE                  UINT8_C(0x00)
#define BMI160_GYRO_BW_OSR2_MODE                  UINT8_C(0x01)
#define BMI160_GYRO_BW_NORMAL_MODE                UINT8_C(0x02)

/* Output Data Rate settings */
/* Accel Output data rate */
#define BMI160_ACCEL_ODR_RESERVED                 UINT8_C(0x00)
#define BMI160_ACCEL_ODR_0_78HZ                   UINT8_C(0x01)
#define BMI160_ACCEL_ODR_1_56HZ                   UINT8_C(0x02)
#define BMI160_ACCEL_ODR_3_12HZ                   UINT8_C(0x03)
#define BMI160_ACCEL_ODR_6_25HZ                   UINT8_C(0x04)
#define BMI160_ACCEL_ODR_12_5HZ                   UINT8_C(0x05)
#define BMI160_ACCEL_ODR_25HZ                     UINT8_C(0x06)
#define BMI160_ACCEL_ODR_50HZ                     UINT8_C(0x07)
#define BMI160_ACCEL_ODR_100HZ                    UINT8_C(0x08)
#define BMI160_ACCEL_ODR_200HZ                    UINT8_C(0x09)
#define BMI160_ACCEL_ODR_400HZ                    UINT8_C(0x0A)
#define BMI160_ACCEL_ODR_800HZ                    UINT8_C(0x0B)
#define BMI160_ACCEL_ODR_1600HZ                   UINT8_C(0x0C)
#define BMI160_ACCEL_ODR_RESERVED0                UINT8_C(0x0D)
#define BMI160_ACCEL_ODR_RESERVED1                UINT8_C(0x0E)
#define BMI160_ACCEL_ODR_RESERVED2                UINT8_C(0x0F)

/* Gyro Output data rate */
#define BMI160_GYRO_ODR_RESERVED                  UINT8_C(0x00)
#define BMI160_GYRO_ODR_25HZ                      UINT8_C(0x06)
#define BMI160_GYRO_ODR_50HZ                      UINT8_C(0x07)
#define BMI160_GYRO_ODR_100HZ                     UINT8_C(0x08)
#define BMI160_GYRO_ODR_200HZ                     UINT8_C(0x09)
#define BMI160_GYRO_ODR_400HZ                     UINT8_C(0x0A)
#define BMI160_GYRO_ODR_800HZ                     UINT8_C(0x0B)
#define BMI160_GYRO_ODR_1600HZ                    UINT8_C(0x0C)
#define BMI160_GYRO_ODR_3200HZ                    UINT8_C(0x0D)

/* Auxiliary sensor Output data rate */
#define BMI160_AUX_ODR_RESERVED                   UINT8_C(0x00)
#define BMI160_AUX_ODR_0_78HZ                     UINT8_C(0x01)
#define BMI160_AUX_ODR_1_56HZ                     UINT8_C(0x02)
#define BMI160_AUX_ODR_3_12HZ                     UINT8_C(0x03)
#define BMI160_AUX_ODR_6_25HZ                     UINT8_C(0x04)
#define BMI160_AUX_ODR_12_5HZ                     UINT8_C(0x05)
#define BMI160_AUX_ODR_25HZ                       UINT8_C(0x06)
#define BMI160_AUX_ODR_50HZ                       UINT8_C(0x07)
#define BMI160_AUX_ODR_100HZ                      UINT8_C(0x08)
#define BMI160_AUX_ODR_200HZ                      UINT8_C(0x09)
#define BMI160_AUX_ODR_400HZ                      UINT8_C(0x0A)
#define BMI160_AUX_ODR_800HZ                      UINT8_C(0x0B)

/** FIFO_CONFIG Definitions */
#define BMI160_FIFO_TIME_ENABLE                   UINT8_C(0x02)
#define BMI160_FIFO_TAG_INT2_ENABLE               UINT8_C(0x04)
#define BMI160_FIFO_TAG_INT1_ENABLE               UINT8_C(0x08)
#define BMI160_FIFO_HEAD_ENABLE                   UINT8_C(0x10)
#define BMI160_FIFO_M_ENABLE                      UINT8_C(0x20)
#define BMI160_FIFO_A_ENABLE                      UINT8_C(0x40)
#define BMI160_FIFO_M_A_ENABLE                    UINT8_C(0x60)
#define BMI160_FIFO_G_ENABLE                      UINT8_C(0x80)
#define BMI160_FIFO_M_G_ENABLE                    UINT8_C(0xA0)
#define BMI160_FIFO_G_A_ENABLE                    UINT8_C(0xC0)
#define BMI160_FIFO_M_G_A_ENABLE                  UINT8_C(0xE0)

/* Macro to specify the number of bytes over-read from the
 * FIFO in order to get the sensor time at the end of FIFO */
#ifndef BMI160_FIFO_BYTES_OVERREAD
#define BMI160_FIFO_BYTES_OVERREAD                UINT8_C(25)
#endif

/* Accel, gyro and aux. sensor length and also their combined
 * length definitions in FIFO */
#define BMI160_FIFO_G_LENGTH                      UINT8_C(6)
#define BMI160_FIFO_A_LENGTH                      UINT8_C(6)
#define BMI160_FIFO_M_LENGTH                      UINT8_C(8)
#define BMI160_FIFO_GA_LENGTH                     UINT8_C(12)
#define BMI160_FIFO_MA_LENGTH                     UINT8_C(14)
#define BMI160_FIFO_MG_LENGTH                     UINT8_C(14)
#define BMI160_FIFO_MGA_LENGTH                    UINT8_C(20)

/** FIFO Header Data definitions */
#define BMI160_FIFO_HEAD_SKIP_FRAME               UINT8_C(0x40)
#define BMI160_FIFO_HEAD_SENSOR_TIME              UINT8_C(0x44)
#define BMI160_FIFO_HEAD_INPUT_CONFIG             UINT8_C(0x48)
#define BMI160_FIFO_HEAD_OVER_READ                UINT8_C(0x80)
#define BMI160_FIFO_HEAD_A                        UINT8_C(0x84)
#define BMI160_FIFO_HEAD_G                        UINT8_C(0x88)
#define BMI160_FIFO_HEAD_G_A                      UINT8_C(0x8C)
#define BMI160_FIFO_HEAD_M                        UINT8_C(0x90)
#define BMI160_FIFO_HEAD_M_A                      UINT8_C(0x94)
#define BMI160_FIFO_HEAD_M_G                      UINT8_C(0x98)
#define BMI160_FIFO_HEAD_M_G_A                    UINT8_C(0x9C)

/** FIFO sensor time length definitions */
#define BMI160_SENSOR_TIME_LENGTH                 UINT8_C(3)

/** FIFO DOWN selection */
/* Accel fifo down-sampling values*/
#define  BMI160_ACCEL_FIFO_DOWN_ZERO              UINT8_C(0x00)
#define  BMI160_ACCEL_FIFO_DOWN_ONE               UINT8_C(0x10)
#define  BMI160_ACCEL_FIFO_DOWN_TWO               UINT8_C(0x20)
#define  BMI160_ACCEL_FIFO_DOWN_THREE             UINT8_C(0x30)
#define  BMI160_ACCEL_FIFO_DOWN_FOUR              UINT8_C(0x40)
#define  BMI160_ACCEL_FIFO_DOWN_FIVE              UINT8_C(0x50)
#define  BMI160_ACCEL_FIFO_DOWN_SIX               UINT8_C(0x60)
#define  BMI160_ACCEL_FIFO_DOWN_SEVEN             UINT8_C(0x70)

/* Gyro fifo down-smapling values*/
#define  BMI160_GYRO_FIFO_DOWN_ZERO               UINT8_C(0x00)
#define  BMI160_GYRO_FIFO_DOWN_ONE                UINT8_C(0x01)
#define  BMI160_GYRO_FIFO_DOWN_TWO                UINT8_C(0x02)
#define  BMI160_GYRO_FIFO_DOWN_THREE              UINT8_C(0x03)
#define  BMI160_GYRO_FIFO_DOWN_FOUR               UINT8_C(0x04)
#define  BMI160_GYRO_FIFO_DOWN_FIVE               UINT8_C(0x05)
#define  BMI160_GYRO_FIFO_DOWN_SIX                UINT8_C(0x06)
#define  BMI160_GYRO_FIFO_DOWN_SEVEN              UINT8_C(0x07)

/* Accel Fifo filter enable*/
#define  BMI160_ACCEL_FIFO_FILT_EN                UINT8_C(0x80)

/* Gyro Fifo filter enable*/
#define  BMI160_GYRO_FIFO_FILT_EN                 UINT8_C(0x08)

/** Definitions to check validity of FIFO frames */
#define FIFO_CONFIG_MSB_CHECK                     UINT8_C(0x80)
#define FIFO_CONFIG_LSB_CHECK                     UINT8_C(0x00)

/*! BMI160 accel FOC configurations */
#define BMI160_FOC_ACCEL_DISABLED                 UINT8_C(0x00)
#define BMI160_FOC_ACCEL_POSITIVE_G               UINT8_C(0x01)
#define BMI160_FOC_ACCEL_NEGATIVE_G               UINT8_C(0x02)
#define BMI160_FOC_ACCEL_0G                       UINT8_C(0x03)

/** Array Parameter DefinItions */
#define BMI160_SENSOR_TIME_LSB_BYTE               UINT8_C(0)
#define BMI160_SENSOR_TIME_XLSB_BYTE              UINT8_C(1)
#define BMI160_SENSOR_TIME_MSB_BYTE               UINT8_C(2)

/** Interface settings */
#define BMI160_SPI_INTF                           UINT8_C(1)
#define BMI160_I2C_INTF                           UINT8_C(0)
#define BMI160_SPI_RD_MASK                        UINT8_C(0x80)
#define BMI160_SPI_WR_MASK                        UINT8_C(0x7F)

/* Sensor & time select definition*/
#define BMI160_ACCEL_SEL                          UINT8_C(0x01)
#define BMI160_GYRO_SEL                           UINT8_C(0x02)
#define BMI160_TIME_SEL                           UINT8_C(0x04)

/* Sensor select mask*/
#define BMI160_SEN_SEL_MASK                       UINT8_C(0x07)

/* Error code mask */
#define BMI160_ERR_REG_MASK                       UINT8_C(0x0F)

/* BMX160 I2C address: 0x69 instead of 0x68 because SDO is connected to VDD */
#define BMI160_I2C_ADDR                           UINT8_C(0x69)

/* BMI160 secondary IF address */
#define BMI160_AUX_BMM150_I2C_ADDR                UINT8_C(0x10)

/** BMI160 Length definitions */
#define BMI160_ONE                                UINT8_C(1)
#define BMI160_TWO                                UINT8_C(2)
#define BMI160_THREE                              UINT8_C(3)
#define BMI160_FOUR                               UINT8_C(4)
#define BMI160_FIVE                               UINT8_C(5)

/** BMI160 fifo level Margin */
#define BMI160_FIFO_LEVEL_MARGIN                  UINT8_C(16)

/** BMI160 fifo flush Command */
#define BMI160_FIFO_FLUSH_VALUE                   UINT8_C(0xB0)

/** BMI160 offset values for xyz axes of accel */
#define BMI160_ACCEL_MIN_OFFSET                   INT8_C(-128)
#define BMI160_ACCEL_MAX_OFFSET                   INT8_C(127)

/** BMI160 offset values for xyz axes of gyro */
#define BMI160_GYRO_MIN_OFFSET                    INT16_C(-512)
#define BMI160_GYRO_MAX_OFFSET                    INT16_C(511)

/** BMI160 fifo full interrupt position and mask */
#define BMI160_FIFO_FULL_INT_POS                  UINT8_C(5)
#define BMI160_FIFO_FULL_INT_MSK                  UINT8_C(0x20)
#define BMI160_FIFO_WTM_INT_POS                   UINT8_C(6)
#define BMI160_FIFO_WTM_INT_MSK                   UINT8_C(0x40)

#define BMI160_FIFO_FULL_INT_PIN1_POS             UINT8_C(5)
#define BMI160_FIFO_FULL_INT_PIN1_MSK             UINT8_C(0x20)
#define BMI160_FIFO_FULL_INT_PIN2_POS             UINT8_C(1)
#define BMI160_FIFO_FULL_INT_PIN2_MSK             UINT8_C(0x02)

#define BMI160_FIFO_WTM_INT_PIN1_POS              UINT8_C(6)
#define BMI160_FIFO_WTM_INT_PIN1_MSK              UINT8_C(0x40)
#define BMI160_FIFO_WTM_INT_PIN2_POS              UINT8_C(2)
#define BMI160_FIFO_WTM_INT_PIN2_MSK              UINT8_C(0x04)

#define BMI160_MANUAL_MODE_EN_POS                 UINT8_C(7)
#define BMI160_MANUAL_MODE_EN_MSK                 UINT8_C(0x80)
#define BMI160_AUX_READ_BURST_POS                 UINT8_C(0)
#define BMI160_AUX_READ_BURST_MSK                 UINT8_C(0x03)

#define BMI160_GYRO_SELF_TEST_POS                 UINT8_C(4)
#define BMI160_GYRO_SELF_TEST_MSK                 UINT8_C(0x10)
#define BMI160_GYRO_SELF_TEST_STATUS_POS          UINT8_C(1)
#define BMI160_GYRO_SELF_TEST_STATUS_MSK          UINT8_C(0x02)

#define BMI160_GYRO_FOC_EN_POS                    UINT8_C(6)
#define BMI160_GYRO_FOC_EN_MSK                    UINT8_C(0x40)

#define BMI160_ACCEL_FOC_X_CONF_POS               UINT8_C(4)
#define BMI160_ACCEL_FOC_X_CONF_MSK               UINT8_C(0x30)

#define BMI160_ACCEL_FOC_Y_CONF_POS               UINT8_C(2)
#define BMI160_ACCEL_FOC_Y_CONF_MSK               UINT8_C(0x0C)

#define BMI160_ACCEL_FOC_Z_CONF_MSK               UINT8_C(0x03)

#define BMI160_FOC_STATUS_POS                     UINT8_C(3)
#define BMI160_FOC_STATUS_MSK                     UINT8_C(0x08)

#define BMI160_GYRO_OFFSET_X_MSK                  UINT8_C(0x03)

#define BMI160_GYRO_OFFSET_Y_POS                  UINT8_C(2)
#define BMI160_GYRO_OFFSET_Y_MSK                  UINT8_C(0x0C)

#define BMI160_GYRO_OFFSET_Z_POS                  UINT8_C(4)
#define BMI160_GYRO_OFFSET_Z_MSK                  UINT8_C(0x30)

#define BMI160_GYRO_OFFSET_EN_POS                 UINT8_C(7)
#define BMI160_GYRO_OFFSET_EN_MSK                 UINT8_C(0x80)

#define BMI160_ACCEL_OFFSET_EN_POS                UINT8_C(6)
#define BMI160_ACCEL_OFFSET_EN_MSK                UINT8_C(0x40)

#define BMI160_GYRO_OFFSET_POS                    UINT16_C(8)
#define BMI160_GYRO_OFFSET_MSK                    UINT16_C(0x0300)

#define BMI160_NVM_UPDATE_POS                     UINT8_C(1)
#define BMI160_NVM_UPDATE_MSK                     UINT8_C(0x02)

#define BMI160_NVM_STATUS_POS                     UINT8_C(4)
#define BMI160_NVM_STATUS_MSK                     UINT8_C(0x10)

#define BMI160_MAG_POWER_MODE_MSK                 UINT8_C(0x03)

#define BMI160_ACCEL_POWER_MODE_MSK               UINT8_C(0x30)
#define BMI160_ACCEL_POWER_MODE_POS               UINT8_C(4)

#define BMI160_GYRO_POWER_MODE_MSK                UINT8_C(0x0C)
#define BMI160_GYRO_POWER_MODE_POS                UINT8_C(2)

/* BIT SLICE GET AND SET FUNCTIONS */
#define BMI160_GET_BITS(regvar, bitname) \
    ((regvar & bitname##_MSK) >> bitname##_POS)
#define BMI160_SET_BITS(regvar, bitname, val) \
    ((regvar & ~bitname##_MSK) | \
     ((val << bitname##_POS) & bitname##_MSK))

#define BMI160_SET_BITS_POS_0(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | \
     (data & bitname##_MSK))

#define BMI160_GET_BITS_POS_0(reg_data, bitname)  (reg_data & (bitname##_MSK))

/**\name UTILITY MACROS */
#define BMI160_SET_LOW_BYTE                       UINT16_C(0x00FF)
#define BMI160_SET_HIGH_BYTE                      UINT16_C(0xFF00)

#define BMI160_GET_LSB(var)                       (uint8_t)(var & BMI160_SET_LOW_BYTE)
#define BMI160_GET_MSB(var)                       (uint8_t)((var & BMI160_SET_HIGH_BYTE) >> 8)

#endif /* BMX160_INTERNALS_H */
