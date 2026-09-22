//*****************************************************************************
//
// bmp390.h
//
// Driver for Bosch BMP390 Digital Pressure & Temperature sensor over I2C,
// for use with the Apollo510B EVB.
//
//*****************************************************************************

#ifndef BMP390_H
#define BMP390_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

//*****************************************************************************
//
// BMP390 I2C address 
//
//*****************************************************************************
#define BMP390_I2C_ADDRESS               0x77

//*****************************************************************************
//
// IOM module used to communicate with the BMP390 on the Apollo510B EVB.
//
// BMP390 now has its OWN dedicated bus (IOM1, I2C) so it no longer time-
// shares IOM0 with the BMI323 (SPI). This removes the need to open/close
// the IOM every 100 ms sample cycle.
//
// IOM1 I2C pins (from am_bsp_pins.h — already wired, no new pin config
// needed since am_bsp_iom_pins_enable()/disable() already dispatch on
// module number generically):
//   SCL -> pin 8  (AM_BSP_GPIO_IOM1_SCL_CB, open-drain, 1.5K pull-up)
//   SDA -> pin 9  (AM_BSP_GPIO_IOM1_SDA_CB, open-drain, 1.5K pull-up)
//
//*****************************************************************************
#define BMP390_IOM_MODULE                1

//*****************************************************************************
//
// I2C bus speed
//
//*****************************************************************************
#define BMP390_I2C_SPEED                 AM_HAL_IOM_400KHZ

//*****************************************************************************
//
// BMP390 Register map definitions
//
//*****************************************************************************
#define BMP390_REG_CHIP_ID               0x00
#define BMP390_REG_STATUS                0x03
#define BMP390_REG_DATA_0                0x04
#define BMP390_REG_PWR_CTRL              0x1B
#define BMP390_REG_OSR                   0x1C
#define BMP390_REG_ODR                   0x1D
#define BMP390_REG_CONFIG                0x1F
#define BMP390_REG_CALIB_DATA            0x31
#define BMP390_REG_CMD                   0x7E

//*****************************************************************************
//
// Command and bit mask codes
//
//*****************************************************************************
#define BMP390_CHIP_ID_VALUE             0x60
#define BMP390_CMD_SOFT_RESET            0xB6

#define BMP390_PRESS_EN                  (1 << 0)
#define BMP390_TEMP_EN                   (1 << 1)
#define BMP390_MODE_NORMAL               (0x03 << 4)

//*****************************************************************************
//
// Sensor configuration enums (Oversampling, IIR Filter, and ODR rates)
//
//*****************************************************************************
typedef enum
{
    BMP390_OVERSAMPLING_1X  = 0x00,
    BMP390_OVERSAMPLING_2X  = 0x01,
    BMP390_OVERSAMPLING_4X  = 0x02,
    BMP390_OVERSAMPLING_8X  = 0x03,
    BMP390_OVERSAMPLING_16X = 0x04
} bmp390_oversampling_e;

typedef enum
{
    BMP390_IIR_FILTER_OFF    = 0x00,
    BMP390_IIR_FILTER_COEF1  = 0x01,
    BMP390_IIR_FILTER_COEF3  = 0x02,
    BMP390_IIR_FILTER_COEF7  = 0x03,
    BMP390_IIR_FILTER_COEF15 = 0x04
} bmp390_iir_filter_e;

typedef enum
{
    BMP390_ODR_50_HZ         = 0x02,
    BMP390_ODR_25_HZ         = 0x03,
    BMP390_ODR_12p5_HZ       = 0x04
} bmp390_odr_e;

//*****************************************************************************
//
// Return / error codes
//
//*****************************************************************************
typedef enum
{
    BMP390_OK              = 0,
    BMP390_ERR_I2C_FAIL    = 1,
    BMP390_ERR_CHIP_ID     = 2
} bmp390_status_e;

//*****************************************************************************
//
// Measurement result structure
//
//*****************************************************************************
typedef struct
{
    float fTemperatureC;        // Temperature in degrees Celsius
    float fPressurePa;          // Atmospheric pressure in Pascals
    float fAltitudeM;           // Calculated relative altitude in meters
} bmp390_data_t;

//*****************************************************************************
//
// Internal NVM Calibration parameters layout
//
//*****************************************************************************
typedef struct
{
    float par_t1; float par_t2; float par_t3;
    float par_p1; float par_p2; float par_p3; float par_p4;
    float par_p5; float par_p6; float par_p7; float par_p8;
    float par_p9; float par_p10; float par_p11;
    float t_lin;
} bmp390_calib_data_t;

//*****************************************************************************
//
// Public APIs
//
//*****************************************************************************
extern bmp390_status_e bmp390_init(void);
extern bmp390_status_e bmp390_load_calibration(void); // <-- Add this line

extern void            bmp390_deinit(void);
extern bmp390_status_e bmp390_soft_reset(void);
extern bmp390_status_e bmp390_read_chip_id(uint8_t *pui8ChipId);
extern bmp390_status_e bmp390_configure_normal_mode(void);
extern bool            bmp390_data_ready(void);
extern bmp390_status_e bmp390_measure(bmp390_data_t *psData, float fBaselineAltitude);
extern float           bmp390_calculate_raw_altitude(float fPressurePa);



#ifdef __cplusplus
}
#endif

#endif // BMP390_H