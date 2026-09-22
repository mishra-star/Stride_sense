//*****************************************************************************
//
//  lsm6dsl.h
//! @file
//!
//! @brief Driver for the ST LSM6DSL 6-axis IMU (accel + gyro) over SPI.
//!        Target: Apollo510B EVB (AmbiqSuite SDK 5.1.0)
//!
//*****************************************************************************

#ifndef LSM6DSL_H
#define LSM6DSL_H

//*****************************************************************************
//
//  lsm6dsl_i2c.h
//! @file
//!
//! @brief Driver for the ST LSM6DSL 6-axis IMU over I2C (IOM).
//!        Target: Ambiq Apollo510B  –  AmbiqSuite SDK 5.1.0
//!
//! Two acquisition modes are supported, selected at init time via
//! lsm6dsl_i2c_config_t.eMode:
//!
//!   LSM6DSL_MODE_NORMAL
//!     – STATUS_REG polling, direct output register burst read.
//!     – Suitable for low-sample-rate / interactive use.
//!
//!   LSM6DSL_MODE_FIFO
//!     – FIFO Continuous mode (FIFO_MODE[2:0] = 110b).
//!     – Accel + Gyro each decimated ×1 into FIFO.
//!     – Watermark threshold (FTH) configurable.
//!     – Caller polls lsm6dsl_i2c_fifo_watermark_reached(), then drains
//!       FIFO words via lsm6dsl_i2c_fifo_read_set().
//!
//! I2C address (datasheet section 5.6):
//!   SA0 pin → GND : 0x6A  (LSM6DSL_I2C_ADDR_SA0_LOW)
//!   SA0 pin → VDD : 0x6B  (LSM6DSL_I2C_ADDR_SA0_HIGH)
//!
//! Wiring (IOM1 example – adjust IOM module in config as needed):
//!   SDA  → AM_BSP_GPIO_IOM1_SDA   (open-drain, 4.7 kΩ pull-up to 3.3 V)
//!   SCL  → AM_BSP_GPIO_IOM1_SCL   (open-drain, 4.7 kΩ pull-up to 3.3 V)
//!   SA0  → GND (I2C address 0x6A)
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

#ifdef __cplusplus
extern "C" {
#endif

//*****************************************************************************
// I2C configuration
//*****************************************************************************

//! IOM module wired to the LSM6DSL SDA/SCL lines.
#define LSM6DSL_I2C_IOM_MODULE          1           // IOM1

//! I2C clock speed
#define LSM6DSL_I2C_SPEED               AM_HAL_IOM_400KHZ

//! 7-bit I2C device addresses (datasheet section 5.6)
#define LSM6DSL_I2C_ADDR_SA0_LOW        0x6A        // SA0 pin → GND
#define LSM6DSL_I2C_ADDR_SA0_HIGH       0x6B        // SA0 pin → VDD

//! Default address used when not specified
#define LSM6DSL_I2C_ADDR_DEFAULT        LSM6DSL_I2C_ADDR_SA0_LOW

//*****************************************************************************
// Register map (datasheet section 9)
//*****************************************************************************
#define LSM6DSL_REG_FUNC_CFG_ACCESS     0x01
#define LSM6DSL_REG_FIFO_CTRL1          0x06    // FTH[7:0]
#define LSM6DSL_REG_FIFO_CTRL2          0x07    // FTH[10:8] + TIMER_PEDO_FIFO + STOP_ON_FTH
#define LSM6DSL_REG_FIFO_CTRL3          0x08    // DEC_FIFO_GYRO + DEC_FIFO_XL
#define LSM6DSL_REG_FIFO_CTRL4          0x09    // DEC3 + DEC4 + ONLY_HIGH_DATA + STOP_ON_FTH
#define LSM6DSL_REG_FIFO_CTRL5          0x0A    // ODR_FIFO + FIFO_MODE
#define LSM6DSL_REG_DRDY_PULSE_CFG      0x0B
#define LSM6DSL_REG_INT1_CTRL           0x0D
#define LSM6DSL_REG_INT2_CTRL           0x0E
#define LSM6DSL_REG_WHO_AM_I            0x0F    // Fixed = 0x6A
#define LSM6DSL_REG_CTRL1_XL            0x10    // Accel ODR + FS
#define LSM6DSL_REG_CTRL2_G             0x11    // Gyro ODR + FS
#define LSM6DSL_REG_CTRL3_C             0x12    // BDU, IF_INC, SW_RESET
#define LSM6DSL_REG_CTRL4_C             0x13    // I2C_disable bit (bit2)
#define LSM6DSL_REG_CTRL5_C             0x14
#define LSM6DSL_REG_CTRL6_C             0x15
#define LSM6DSL_REG_CTRL7_G             0x16
#define LSM6DSL_REG_CTRL8_XL            0x17
#define LSM6DSL_REG_CTRL9_XL            0x18
#define LSM6DSL_REG_CTRL10_C            0x19
#define LSM6DSL_REG_STATUS_REG          0x1E
#define LSM6DSL_REG_OUT_TEMP_L          0x20
#define LSM6DSL_REG_OUT_TEMP_H          0x21
#define LSM6DSL_REG_OUTX_L_G            0x22
#define LSM6DSL_REG_OUTX_H_G            0x23
#define LSM6DSL_REG_OUTY_L_G            0x24
#define LSM6DSL_REG_OUTY_H_G            0x25
#define LSM6DSL_REG_OUTZ_L_G            0x26
#define LSM6DSL_REG_OUTZ_H_G            0x27
#define LSM6DSL_REG_OUTX_L_XL           0x28
#define LSM6DSL_REG_OUTX_H_XL           0x29
#define LSM6DSL_REG_OUTY_L_XL           0x2A
#define LSM6DSL_REG_OUTY_H_XL           0x2B
#define LSM6DSL_REG_OUTZ_L_XL           0x2C
#define LSM6DSL_REG_OUTZ_H_XL           0x2D
#define LSM6DSL_REG_FIFO_STATUS1        0x3A    // DIFF_FIFO[7:0]
#define LSM6DSL_REG_FIFO_STATUS2        0x3B    // FTH flag, OVER_RUN, FULL_SMART, EMPTY, DIFF[10:8]
#define LSM6DSL_REG_FIFO_STATUS3        0x3C    // FIFO pattern word[7:0]
#define LSM6DSL_REG_FIFO_STATUS4        0x3D    // FIFO pattern word[9:8]
#define LSM6DSL_REG_FIFO_DATA_OUT_L     0x3E
#define LSM6DSL_REG_FIFO_DATA_OUT_H     0x3F
#define LSM6DSL_REG_STEP_COUNTER_L      0x4B    // Step counter output, LSbyte (r)
#define LSM6DSL_REG_STEP_COUNTER_H      0x4C    // Step counter output, MSbyte (r)
#define LSM6DSL_REG_FUNC_SRC1           0x53    // Pedometer/tilt/sig-motion status (r, clear-on-read)

//*****************************************************************************
// WHO_AM_I
//*****************************************************************************
#define LSM6DSL_WHO_AM_I_VALUE          0x6A

//*****************************************************************************
// CTRL3_C bit masks
//*****************************************************************************
#define LSM6DSL_CTRL3_BDU               (1u << 6)   // Block data update
#define LSM6DSL_CTRL3_IF_INC            (1u << 2)   // Register auto-increment
#define LSM6DSL_CTRL3_SW_RESET          (1u << 0)   // Software reset

//*****************************************************************************
// STATUS_REG bit masks
//*****************************************************************************
#define LSM6DSL_STATUS_TDA              (1u << 2)   // Temperature data available
#define LSM6DSL_STATUS_GDA              (1u << 1)   // Gyro data available
#define LSM6DSL_STATUS_XLDA             (1u << 0)   // Accel data available

//*****************************************************************************
// FIFO_STATUS2 bit masks
//*****************************************************************************
#define LSM6DSL_FIFO_FTH                (1u << 7)   // Watermark reached
#define LSM6DSL_FIFO_OVER_RUN           (1u << 6)   // FIFO overrun
#define LSM6DSL_FIFO_FULL_SMART         (1u << 5)   // FIFO full
#define LSM6DSL_FIFO_EMPTY              (1u << 4)   // FIFO empty

//*****************************************************************************
// FIFO_CTRL3 decimation values (no decimation = store every sample)
//*****************************************************************************
#define LSM6DSL_FIFO_DEC_BYPASS         0x0         // Sensor not in FIFO
#define LSM6DSL_FIFO_DEC_1              0x1         // No decimation

//*****************************************************************************
// FIFO_CTRL5 FIFO mode bits [2:0]
//*****************************************************************************
#define LSM6DSL_FIFO_MODE_BYPASS        0x00        // FIFO disabled
#define LSM6DSL_FIFO_MODE_FIFO          0x01        // FIFO mode (stops when full)
#define LSM6DSL_FIFO_MODE_CONTINUOUS    0x06        // Continuous – discard oldest

//*****************************************************************************
// FIFO_CTRL5 ODR bits [6:3]  (must match sensor ODR for correct timestamps)
//*****************************************************************************
#define LSM6DSL_FIFO_ODR_12_5HZ        (0x1u << 3)
#define LSM6DSL_FIFO_ODR_26HZ          (0x2u << 3)
#define LSM6DSL_FIFO_ODR_52HZ          (0x3u << 3)
#define LSM6DSL_FIFO_ODR_104HZ         (0x4u << 3)
#define LSM6DSL_FIFO_ODR_208HZ         (0x5u << 3)
#define LSM6DSL_FIFO_ODR_416HZ         (0x6u << 3)

//*****************************************************************************
// Accelerometer ODR / FS  (CTRL1_XL)
//*****************************************************************************
#define LSM6DSL_XL_ODR_OFF              (0x0u << 4)
#define LSM6DSL_XL_ODR_12_5HZ          (0x1u << 4)
#define LSM6DSL_XL_ODR_26HZ            (0x2u << 4)
#define LSM6DSL_XL_ODR_52HZ            (0x3u << 4)
#define LSM6DSL_XL_ODR_104HZ           (0x4u << 4)
#define LSM6DSL_XL_ODR_208HZ           (0x5u << 4)
#define LSM6DSL_XL_ODR_416HZ           (0x6u << 4)

#define LSM6DSL_XL_FS_2G               (0x0u << 2)
#define LSM6DSL_XL_FS_16G              (0x1u << 2)
#define LSM6DSL_XL_FS_4G               (0x2u << 2)
#define LSM6DSL_XL_FS_8G               (0x3u << 2)

//*****************************************************************************
// Gyroscope ODR / FS  (CTRL2_G)
//*****************************************************************************
#define LSM6DSL_G_ODR_OFF               (0x0u << 4)
#define LSM6DSL_G_ODR_12_5HZ           (0x1u << 4)
#define LSM6DSL_G_ODR_26HZ             (0x2u << 4)
#define LSM6DSL_G_ODR_52HZ             (0x3u << 4)
#define LSM6DSL_G_ODR_104HZ            (0x4u << 4)
#define LSM6DSL_G_ODR_208HZ            (0x5u << 4)
#define LSM6DSL_G_ODR_416HZ            (0x6u << 4)

#define LSM6DSL_G_FS_250DPS            (0x0u << 2)
#define LSM6DSL_G_FS_500DPS            (0x1u << 2)
#define LSM6DSL_G_FS_1000DPS           (0x2u << 2)
#define LSM6DSL_G_FS_2000DPS           (0x3u << 2)

//*****************************************************************************
// Sensitivity constants
//*****************************************************************************
#define LSM6DSL_XL_SENS_2G              0.061f      // mg/LSB
#define LSM6DSL_XL_SENS_4G              0.122f
#define LSM6DSL_XL_SENS_8G              0.244f
#define LSM6DSL_XL_SENS_16G             0.488f

#define LSM6DSL_G_SENS_250DPS           8.75f       // mdps/LSB
#define LSM6DSL_G_SENS_500DPS           17.50f
#define LSM6DSL_G_SENS_1000DPS          35.00f
#define LSM6DSL_G_SENS_2000DPS          70.00f

#define LSM6DSL_TEMP_SENS               16.0f       // LSB/°C
#define LSM6DSL_TEMP_OFFSET             25.0f       // °C at raw=0

//*****************************************************************************
// CTRL10_C (0x19) embedded-function enable bits (datasheet Tables 76/77).
// Needed for the pedometer/step counter; other CTRL10_C bits (wrist tilt,
// timer, tilt, significant motion) are unused by this driver but defined
// here for completeness.
//*****************************************************************************
#define LSM6DSL_CTRL10_SIGN_MOTION_EN   (0x1u << 0)
#define LSM6DSL_CTRL10_PEDO_RST_STEP    (0x1u << 1)
#define LSM6DSL_CTRL10_FUNC_EN          (0x1u << 2)
#define LSM6DSL_CTRL10_TILT_EN          (0x1u << 3)
#define LSM6DSL_CTRL10_PEDO_EN          (0x1u << 4)
#define LSM6DSL_CTRL10_TIMER_EN         (0x1u << 5)
#define LSM6DSL_CTRL10_WRIST_TILT_EN    (0x1u << 7)

//*****************************************************************************
// FUNC_SRC1 (0x53) status bits (datasheet Tables 177/178). Reading this
// register clears its latched status bits.
//*****************************************************************************
#define LSM6DSL_FUNC_SRC1_SENSORHUB_END_OP    (0x1u << 0)
#define LSM6DSL_FUNC_SRC1_SI_END_OP           (0x1u << 1)
#define LSM6DSL_FUNC_SRC1_HI_FAIL             (0x1u << 2)
#define LSM6DSL_FUNC_SRC1_STEP_OVERFLOW       (0x1u << 3)
#define LSM6DSL_FUNC_SRC1_STEP_DETECTED       (0x1u << 4)
#define LSM6DSL_FUNC_SRC1_TILT_IA             (0x1u << 5)
#define LSM6DSL_FUNC_SRC1_SIGN_MOTION_IA      (0x1u << 6)
#define LSM6DSL_FUNC_SRC1_STEP_COUNT_DELTA_IA (0x1u << 7)

//*****************************************************************************
// Data structures
//*****************************************************************************

//! Raw 3-axis reading
typedef struct
{
    int16_t i16X;
    int16_t i16Y;
    int16_t i16Z;
} lsm6dsl_i2c_raw_t;

//! Acquisition mode selector
typedef enum
{
    LSM6DSL_MODE_NORMAL = 0,    //!< Poll STATUS_REG, read output regs directly
    LSM6DSL_MODE_FIFO           //!< FIFO Continuous mode with watermark
} lsm6dsl_i2c_mode_t;

//! One decoded FIFO word-set (gyro + accel set, in FIFO order)
//! The LSM6DSL FIFO interleaves gyro then accel words (when both decimation
//! factors are 1).  Each call to lsm6dsl_i2c_fifo_read_set() returns one
//! such pair.
typedef struct
{
    lsm6dsl_i2c_raw_t sGyro;   //!< Raw gyro word from FIFO
    lsm6dsl_i2c_raw_t sAccel;  //!< Raw accel word from FIFO
} lsm6dsl_i2c_fifo_set_t;

//! Driver configuration  – pass to lsm6dsl_i2c_init()
typedef struct
{
    uint32_t            ui32IOMModule;  //!< IOM module number (e.g. 1)
    uint32_t            ui32I2CSpeed;   //!< e.g. AM_HAL_IOM_400KHZ
    uint8_t             ui8DevAddr;     //!< 7-bit I2C address (0x6A or 0x6B)

    // Sensor ODR / FS
    uint8_t             ui8XLOdr;       //!< LSM6DSL_XL_ODR_xxx
    uint8_t             ui8XLFs;        //!< LSM6DSL_XL_FS_xxx
    uint8_t             ui8GOdr;        //!< LSM6DSL_G_ODR_xxx
    uint8_t             ui8GFs;         //!< LSM6DSL_G_FS_xxx

    // Acquisition mode
    lsm6dsl_i2c_mode_t eMode;

    // FIFO-mode parameters (ignored when eMode == LSM6DSL_MODE_NORMAL)
    uint16_t            ui16FifoWatermark; //!< FTH threshold in 16-bit words
                                            //   (e.g. 12 = 2 sets of gyro+accel)
    uint8_t             ui8FifoOdr;        //!< LSM6DSL_FIFO_ODR_xxx (must match sensor ODR)
} lsm6dsl_i2c_config_t;

//*****************************************************************************
// Public API
//*****************************************************************************

//! @brief  Initialise IOM I2C, reset and configure LSM6DSL.
//! @return AM_HAL_STATUS_SUCCESS or IOM error code.
uint32_t lsm6dsl_i2c_init(const lsm6dsl_i2c_config_t *psConfig);

//! @brief  Power down sensors, disable IOM.
void     lsm6dsl_i2c_deinit(void);

//! @brief  Read WHO_AM_I register (expected 0x6A).
uint32_t lsm6dsl_i2c_who_am_i(uint8_t *pui8Id);

// ── Normal mode ─────────────────────────────────────────────────────────────

//! @brief  Read STATUS_REG.  Mask result with LSM6DSL_STATUS_XLDA / GDA / TDA.
uint32_t lsm6dsl_i2c_get_status(uint8_t *pui8Status);

//! @brief  Burst-read raw accelerometer (6 bytes, OUTX_L_XL..OUTZ_H_XL).
uint32_t lsm6dsl_i2c_read_accel_raw(lsm6dsl_i2c_raw_t *psData);

//! @brief  Burst-read raw gyroscope (6 bytes, OUTX_L_G..OUTZ_H_G).
uint32_t lsm6dsl_i2c_read_gyro_raw(lsm6dsl_i2c_raw_t *psData);

//! @brief  Burst-read gyro + accel in a single 12-byte transfer.
uint32_t lsm6dsl_i2c_read_gyro_accel_raw(lsm6dsl_i2c_raw_t *psGyro,
                                           lsm6dsl_i2c_raw_t *psAccel);

//! @brief  Read raw temperature (OUT_TEMP_L/H).
uint32_t lsm6dsl_i2c_read_temp_raw(int16_t *pi16Temp);

// ── FIFO mode ────────────────────────────────────────────────────────────────

//! @brief  Return true when FIFO watermark flag (FTH) is set.
//! @param  pbReached  OUT: true = watermark reached.
uint32_t lsm6dsl_i2c_fifo_watermark_reached(bool *pbReached);

//! @brief  Return current number of 16-bit words stored in FIFO.
//! @param  pui16Count  OUT: word count (0..4096).
uint32_t lsm6dsl_i2c_fifo_word_count(uint16_t *pui16Count);

//! @brief  Read one gyro+accel set (6 words = 12 bytes) from FIFO.
//!         Each call consumes 6 FIFO words: Gx,Gy,Gz,Ax,Ay,Az.
//!         Call lsm6dsl_i2c_fifo_word_count() first; divide by 6 to know
//!         how many complete sets are available.
//! @param  psSet  OUT: decoded gyro + accel raw values.
uint32_t lsm6dsl_i2c_fifo_read_set(lsm6dsl_i2c_fifo_set_t *psSet);

//! @brief  Reset FIFO by toggling Bypass → Continuous.
//!         Call this after an overrun or before restarting acquisition.
uint32_t lsm6dsl_i2c_fifo_reset(void);

// ── Conversion helpers ───────────────────────────────────────────────────────

//! @brief  Convert raw accel counts to mg.
void lsm6dsl_i2c_convert_accel(const lsm6dsl_i2c_raw_t *psRaw, float fSens,
                                 float *pfX, float *pfY, float *pfZ);

//! @brief  Convert raw gyro counts to mdps.
void lsm6dsl_i2c_convert_gyro(const lsm6dsl_i2c_raw_t *psRaw, float fSens,
                                float *pfX, float *pfY, float *pfZ);

//! @brief  Convert raw temperature to °C.  T = 25 + raw/16.
float lsm6dsl_i2c_convert_temp(int16_t i16Raw);

// ── Pedometer / step counter ─────────────────────────────────────────────────

//! @brief  Enable the embedded pedometer algorithm (CTRL10_C: FUNC_EN + PEDO_EN).
//!         Uses the sensor's power-on-reset defaults for peak threshold
//!         (CONFIG_PEDO_THS_MIN, Bank A 0x0F = 0x10) and debounce
//!         (PEDO_DEB_REG, Bank A 0x14 = 0x6E) - the datasheet lists both as
//!         sane out-of-the-box values, so no Bank-A register access (which
//!         would require the device to be in power-down mode) is needed.
uint32_t lsm6dsl_i2c_pedometer_enable(void);

//! @brief  Reset the internal step counter back to 0 (pulses PEDO_RST_STEP
//!         high then low, per CTRL10_C). Call once right after
//!         lsm6dsl_i2c_pedometer_enable() to start a session's step count
//!         at 0, or any time you want to zero the count without disabling
//!         the algorithm.
uint32_t lsm6dsl_i2c_pedometer_reset(void);

//! @brief  Read the current 16-bit step count (STEP_COUNTER_L/H, 0x4B/0x4C).
//!         The count saturates (does not wrap) at 65535 - see
//!         lsm6dsl_i2c_step_overflowed() to detect that condition.
//! @param  pui16Steps  OUT: total steps since the last reset.
uint32_t lsm6dsl_i2c_read_step_count(uint16_t *pui16Steps);

//! @brief  Read FUNC_SRC1 and report whether the step counter has
//!         overflowed (reached 65535) since the last reset. NOTE: reading
//!         FUNC_SRC1 clears its latched status bits, so call this at most
//!         once per polling cycle alongside any other FUNC_SRC1 use.
//! @param  pbOverflowed  OUT: true if the step counter has saturated.
uint32_t lsm6dsl_i2c_step_overflowed(bool *pbOverflowed);

// ── Low-level register access (available for debug / extension) ──────────────
uint32_t lsm6dsl_i2c_write_reg(uint8_t ui8Reg, uint8_t ui8Val);
uint32_t lsm6dsl_i2c_read_reg(uint8_t ui8Reg, uint8_t *pui8Val);
uint32_t lsm6dsl_i2c_read_burst(uint8_t ui8StartReg, uint8_t *pui8Buf,
                                  uint32_t ui32Len);

#ifdef __cplusplus
}
#endif


#endif // LSM6DSL_H