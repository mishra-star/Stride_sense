//*****************************************************************************
//
//! @file bmi323.h
//!
//! @brief Driver for the Bosch BMI323 6-axis IMU (accel + gyro) over SPI.
//!        Target: Apollo510B EVB — AmbiqSuite SDK 5.1.0
//!
//! IOM assignment in merged project:
//!   BMI323 has IOM0 (SPI) to itself. BMP390 has moved to its own bus,
//!   IOM1 (I2C), so the two sensors no longer time-share a single IOM.
//!   SensorTask opens both once at startup via bmi323_reinit_spi() /
//!   bmp390_init() and leaves them open — no more per-cycle
//!   release/reinit dance.
//!
//! BMI323 wiring (IOM0 SPI — same connector as original standalone project):
//!   SCK  -> pin 5
//!   MOSI -> pin 6  (BMI323 SDI)
//!   MISO -> pin 7  (BMI323 SDO)
//!   CS   -> AM_BSP_IOM0_CS_CHNL
//!   VDD / VDDIO -> 3.3 V
//!   GND  -> GND
//
//*****************************************************************************

#ifndef BMI323_H
#define BMI323_H

#include <stdint.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

#ifdef __cplusplus
extern "C" {
#endif

//*****************************************************************************
// IOM — BMI323 uses IOM0 in SPI mode (same module as BMP390 I2C;
// the task switches the IOM mode between the two sensor reads).
//*****************************************************************************
#define BMI323_IOM_MODULE          0
#define BMI323_IOM_CS_CHNL         AM_BSP_IOM0_CS_CHNL
#define BMI323_IOM_SPI_SPEED       AM_HAL_IOM_8MHZ

//*****************************************************************************
// Register map
//*****************************************************************************
#define BMI323_REG_CHIP_ID             0x00
#define BMI323_REG_ERR_REG             0x01
#define BMI323_REG_STATUS              0x02
#define BMI323_REG_ACC_DATA_X          0x03
#define BMI323_REG_ACC_DATA_Y          0x04
#define BMI323_REG_ACC_DATA_Z          0x05
#define BMI323_REG_GYR_DATA_X          0x06
#define BMI323_REG_GYR_DATA_Y          0x07
#define BMI323_REG_GYR_DATA_Z          0x08
#define BMI323_REG_TEMP_DATA           0x09
#define BMI323_REG_SENSOR_TIME_0       0x0A
#define BMI323_REG_SENSOR_TIME_1       0x0B
#define BMI323_REG_ACC_CONF            0x20
#define BMI323_REG_GYR_CONF            0x21
#define BMI323_REG_CMD                 0x7E

//*****************************************************************************
// CHIP_ID
//*****************************************************************************
#define BMI323_CHIP_ID_VALUE           0x0043

//*****************************************************************************
// STATUS register bits
//*****************************************************************************
#define BMI323_STATUS_DRDY_ACC         (1u << 5)
#define BMI323_STATUS_DRDY_GYR         (1u << 6)

//*****************************************************************************
// ODR
//*****************************************************************************
#define BMI323_ODR_50HZ        0x07
#define BMI323_ODR_100HZ       0x08
#define BMI323_ODR_200HZ       0x09


//*****************************************************************************
// Accel range
//*****************************************************************************
#define BMI323_ACC_RANGE_2G    0x00
#define BMI323_ACC_RANGE_4G    0x01
#define BMI323_ACC_RANGE_8G    0x02
#define BMI323_ACC_RANGE_16G   0x03

#define BMI323_ACC_SENS_2G_LSB_PER_G     16384.0f
#define BMI323_ACC_SENS_4G_LSB_PER_G      8192.0f
#define BMI323_ACC_SENS_8G_LSB_PER_G      4096.0f
#define BMI323_ACC_SENS_16G_LSB_PER_G     2048.0f

//*****************************************************************************
// Gyro range
//*****************************************************************************
#define BMI323_GYR_RANGE_125DPS    0x00
#define BMI323_GYR_RANGE_250DPS    0x01
#define BMI323_GYR_RANGE_500DPS    0x02
#define BMI323_GYR_RANGE_1000DPS   0x03
#define BMI323_GYR_RANGE_2000DPS   0x04

#define BMI323_GYR_SENS_125DPS_LSB_PER_DPS    262.144f
#define BMI323_GYR_SENS_250DPS_LSB_PER_DPS    131.072f
#define BMI323_GYR_SENS_500DPS_LSB_PER_DPS     65.536f
#define BMI323_GYR_SENS_1000DPS_LSB_PER_DPS    32.768f
#define BMI323_GYR_SENS_2000DPS_LSB_PER_DPS    16.384f

//*****************************************************************************
// Bandwidth / average
//*****************************************************************************
#define BMI323_BW_ODR_OVER_2   0x00
#define BMI323_BW_ODR_OVER_4   0x01
#define BMI323_AVG_NONE        0x00

//*****************************************************************************
// Operating modes
//*****************************************************************************
#define BMI323_ACC_MODE_DISABLE    0x00
#define BMI323_ACC_MODE_NORMAL     0x04
#define BMI323_ACC_MODE_HIGH_PERF  0x07
#define BMI323_GYR_MODE_DISABLE    0x00
#define BMI323_GYR_MODE_NORMAL     0x04
#define BMI323_GYR_MODE_HIGH_PERF  0x07

//*****************************************************************************
// CMD
//*****************************************************************************
#define BMI323_CMD_SOFT_RESET      0xDEAF

//*****************************************************************************
// Internal register field shifts
//*****************************************************************************
#define BMI323_SPI_READ_BIT         0x80
#define BMI323_CONF_ODR_SHIFT       0
#define BMI323_CONF_RANGE_SHIFT     4
#define BMI323_CONF_BW_SHIFT        7
#define BMI323_CONF_AVG_SHIFT       8
#define BMI323_CONF_MODE_SHIFT      12

//*****************************************************************************
// Data types
//*****************************************************************************
typedef struct
{
    int16_t i16X;
    int16_t i16Y;
    int16_t i16Z;
} bmi323_axis_data_t;

typedef bmi323_axis_data_t bmi323_raw_data_t;

typedef struct
{
    bmi323_axis_data_t sAccel;
    bmi323_axis_data_t sGyro;
} bmi323_raw_imu_data_t;

typedef struct
{
    uint32_t ui32IOMModule;
    uint32_t ui32CSChannel;
    uint32_t ui32SpiSpeed;
    uint8_t  ui8AccOdr;
    uint8_t  ui8AccRange;
    uint8_t  ui8AccBw;
    uint8_t  ui8AccAvgNum;
    uint8_t  ui8AccMode;
    uint8_t  ui8GyrOdr;
    uint8_t  ui8GyrRange;
    uint8_t  ui8GyrBw;
    uint8_t  ui8GyrAvgNum;
    uint8_t  ui8GyrMode;
} bmi323_config_t;

//*****************************************************************************
// Public API
//*****************************************************************************
uint32_t bmi323_init(const bmi323_config_t *psCfg);
void     bmi323_deinit(void);

// Re-initialise SPI on IOM0 (call after bmp390_deinit releases IOM0).
uint32_t bmi323_reinit_spi(void);

// Release IOM0 SPI (call before bmp390_init re-claims IOM0 as I2C).
void     bmi323_release_spi(void);

uint32_t bmi323_who_am_i(uint16_t *pui16ChipId);
uint32_t bmi323_read_status(uint16_t *pui16Status);
uint32_t bmi323_read_imu_raw(bmi323_raw_imu_data_t *psData);
uint32_t bmi323_get_sensor_time(uint32_t *pui32Time);

void bmi323_convert_accel(const bmi323_axis_data_t *psRaw, float fSensitivity,
                           float *pfX, float *pfY, float *pfZ);
void bmi323_convert_gyro (const bmi323_axis_data_t *psRaw, float fSensitivity,
                           float *pfX, float *pfY, float *pfZ);

// Internal helpers (used within bmi323.c)
uint32_t bmi323_read_registers(uint8_t ui8StartReg, uint16_t *pui16Val, uint32_t ui32NumRegs);
uint32_t bmi323_read_register (uint8_t ui8Reg, uint16_t *pui16Val);
uint32_t bmi323_write_register(uint8_t ui8Reg, uint16_t ui16Val);
uint32_t bmi323_soft_reset(void);
uint32_t bmi323_read_accel_raw(bmi323_raw_data_t *psData);
uint32_t bmi323_read_gyro_raw (bmi323_raw_data_t *psData);

#ifdef __cplusplus
}
#endif

// IOM time-sharing API
extern uint32_t bmi323_store_config(const bmi323_config_t *psConfig);
extern uint32_t bmi323_reinit_spi(void);
extern void     bmi323_release_spi(void);

#endif // BMI323_H