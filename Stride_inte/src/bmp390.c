//*****************************************************************************
//
// bmp390.c
//
// Driver for Bosch BMP390 Digital Pressure & Temperature sensor over I2C.
//
//*****************************************************************************

#include "bmp390.h"
#include <math.h>
#include <string.h>

//*****************************************************************************
//
// Static module variables
//
//*****************************************************************************
static void                *g_BMP390IomHandle;
static bmp390_calib_data_t  g_sBmp390Calib;

static am_hal_iom_config_t  g_sBMP390IomCfg =
{
    .eInterfaceMode       = AM_HAL_IOM_I2C_MODE,
    .ui32ClockFreq        = BMP390_I2C_SPEED,
};

//*****************************************************************************
//
// Local helper: write multiple bytes to a BMP390 register.
//
//*****************************************************************************
static bmp390_status_e
bmp390_write_bytes(uint8_t ui8Reg, const uint8_t *pui8TxBuf, uint32_t ui32NumBytes)
{
    if ( ui32NumBytes > 15 ) return BMP390_ERR_I2C_FAIL;

    AM_SHARED_RW static uint32_t ui32TxWordBuf[4];
    uint8_t *pui8DataPtr = (uint8_t *)ui32TxWordBuf;

    pui8DataPtr[0] = ui8Reg;
    for ( uint32_t i = 0; i < ui32NumBytes; i++ )
    {
        pui8DataPtr[1 + i] = pui8TxBuf[i];
    }

    am_hal_iom_transfer_t sXfer =
    {
        .uPeerInfo.ui32I2CDevAddr = BMP390_I2C_ADDRESS,
        .ui32InstrLen             = 0,
        .ui64Instr                = 0,
        .eDirection               = AM_HAL_IOM_TX,
        .ui32NumBytes             = ui32NumBytes + 1,
        .pui32TxBuffer            = ui32TxWordBuf,
        .bContinue                = false,
        .ui8RepeatCount           = 0,
        .ui8Priority              = 1,
        .ui32PauseCondition       = 0,
        .ui32StatusSetClr         = 0,
    };

    if ( am_hal_iom_blocking_transfer(g_BMP390IomHandle, &sXfer) != AM_HAL_STATUS_SUCCESS )
    {
        return BMP390_ERR_I2C_FAIL;
    }

    return BMP390_OK;
}

//*****************************************************************************
//
// Local helper: read a number of bytes from a BMP390 register into a buffer.
//
//*****************************************************************************
static bmp390_status_e
bmp390_read_bytes(uint8_t ui8Reg, uint8_t *pui8RxBuf, uint32_t ui32NumBytes)
{
    if ( ui32NumBytes > 24 ) return BMP390_ERR_I2C_FAIL;

    AM_SHARED_RW static uint32_t ui32RxWordBuf[6];
    memset(ui32RxWordBuf, 0, sizeof(ui32RxWordBuf));

    am_hal_iom_transfer_t sXfer =
    {
        .uPeerInfo.ui32I2CDevAddr = BMP390_I2C_ADDRESS,
        .ui32InstrLen             = 1,
        .ui64Instr                = ui8Reg,
        .eDirection               = AM_HAL_IOM_RX,
        .ui32NumBytes             = ui32NumBytes,
        .pui32RxBuffer            = ui32RxWordBuf,
        .bContinue                = false,
        .ui8RepeatCount           = 0,
        .ui8Priority              = 1,
        .ui32PauseCondition       = 0,
        .ui32StatusSetClr         = 0,
    };

    if ( am_hal_iom_blocking_transfer(g_BMP390IomHandle, &sXfer) != AM_HAL_STATUS_SUCCESS )
    {
        return BMP390_ERR_I2C_FAIL;
    }

    // Direct byte alignment copy to prevent mathematical shifts over the word boundaries
    uint8_t *pui8ByteSrc = (uint8_t *)ui32RxWordBuf;
    for ( uint32_t i = 0; i < ui32NumBytes; i++ )
    {
        pui8RxBuf[i] = pui8ByteSrc[i];
    }

    return BMP390_OK;
}

//*****************************************************************************
//
// Local helper: Fetch factory calibration coefficients from the sensor's NVM.
//
//*****************************************************************************
static bmp390_status_e
bmp390_read_calibration_data(void)
{
    uint8_t calib[21];
    if ( bmp390_read_bytes(BMP390_REG_CALIB_DATA, calib, sizeof(calib)) != BMP390_OK )
    {
        return BMP390_ERR_I2C_FAIL;
    }

    uint16_t nvm_par_t1 = (uint16_t)(calib[0]  | ((uint16_t)calib[1]  << 8));
    uint16_t nvm_par_t2 = (uint16_t)(calib[2]  | ((uint16_t)calib[3]  << 8));
    int8_t   nvm_par_t3 = (int8_t)calib[4];
    int16_t  nvm_par_p1 = (int16_t)(calib[5]   | ((uint16_t)calib[6]  << 8));
    int16_t  nvm_par_p2 = (int16_t)(calib[7]   | ((uint16_t)calib[8]  << 8));
    int8_t   nvm_par_p3 = (int8_t)calib[9];
    int8_t   nvm_par_p4 = (int8_t)calib[10];
    uint16_t nvm_par_p5 = (uint16_t)(calib[11] | ((uint16_t)calib[12] << 8));
    uint16_t nvm_par_p6 = (uint16_t)(calib[13] | ((uint16_t)calib[14] << 8));
    int8_t   nvm_par_p7 = (int8_t)calib[15];
    int8_t   nvm_par_p8 = (int8_t)calib[16];
    int16_t  nvm_par_p9 = (int16_t)(calib[17]  | ((uint16_t)calib[18] << 8));
    int8_t   nvm_par_p10 = (int8_t)calib[19];
    int8_t   nvm_par_p11 = (int8_t)calib[20];

    g_sBmp390Calib.par_t1 = (float)nvm_par_t1  * 256.0f;
    g_sBmp390Calib.par_t2 = (float)nvm_par_t2  / 1073741824.0f;
    g_sBmp390Calib.par_t3 = (float)nvm_par_t3  / 281474976710656.0f;
    g_sBmp390Calib.par_p1 = ((float)nvm_par_p1 - 16384.0f) / 1048576.0f;
    g_sBmp390Calib.par_p2 = ((float)nvm_par_p2 - 16384.0f) / 536870912.0f;
    g_sBmp390Calib.par_p3 = (float)nvm_par_p3   / 4294967296.0f;
    g_sBmp390Calib.par_p4 = (float)nvm_par_p4   / 137438953472.0f;
    g_sBmp390Calib.par_p5 = (float)nvm_par_p5   * 8.0f;
    g_sBmp390Calib.par_p6 = (float)nvm_par_p6   / 64.0f;
    g_sBmp390Calib.par_p7 = (float)nvm_par_p7   / 256.0f;
    g_sBmp390Calib.par_p8 = (float)nvm_par_p8   / 32768.0f;
    g_sBmp390Calib.par_p9 = (float)nvm_par_p9   / 281474976710656.0f;
    g_sBmp390Calib.par_p10 = (float)nvm_par_p10 / 281474976710656.0f;
    g_sBmp390Calib.par_p11 = (float)nvm_par_p11 / 36893488147419103232.0f;
    g_sBmp390Calib.t_lin   = 0.0f;

    return BMP390_OK;
}

//*****************************************************************************
//
// Calibration trigger wrapper accessible externally
//
//*****************************************************************************
bmp390_status_e
bmp390_load_calibration(void)
{
    return bmp390_read_calibration_data();
}

//*****************************************************************************
//
// Data compensation routines from Bosch datasheet equations.
//
//*****************************************************************************
static float
bmp390_compensate_temperature(uint32_t uncomp_temp)
{
    float pd1 = (float)uncomp_temp - g_sBmp390Calib.par_t1;
    float pd2 = pd1 * g_sBmp390Calib.par_t2;
    g_sBmp390Calib.t_lin = pd2 + (pd1 * pd1) * g_sBmp390Calib.par_t3;
    return g_sBmp390Calib.t_lin;
}

static float
bmp390_compensate_pressure(uint32_t uncomp_press)
{
    float pd1 = g_sBmp390Calib.par_p6 * g_sBmp390Calib.t_lin;
    float pd2 = g_sBmp390Calib.par_p7 * (g_sBmp390Calib.t_lin * g_sBmp390Calib.t_lin);
    float pd3 = g_sBmp390Calib.par_p8 * (g_sBmp390Calib.t_lin * g_sBmp390Calib.t_lin * g_sBmp390Calib.t_lin);
    float po1 = g_sBmp390Calib.par_p5 + pd1 + pd2 + pd3;

    pd1 = g_sBmp390Calib.par_p2 * g_sBmp390Calib.t_lin;
    pd2 = g_sBmp390Calib.par_p3 * (g_sBmp390Calib.t_lin * g_sBmp390Calib.t_lin);
    pd3 = g_sBmp390Calib.par_p4 * (g_sBmp390Calib.t_lin * g_sBmp390Calib.t_lin * g_sBmp390Calib.t_lin);
    float po2 = (float)uncomp_press * (g_sBmp390Calib.par_p1 + pd1 + pd2 + pd3);

    pd1 = (float)uncomp_press * (float)uncomp_press;
    pd2 = g_sBmp390Calib.par_p9 + g_sBmp390Calib.par_p10 * g_sBmp390Calib.t_lin;
    pd3 = pd1 * pd2;
    float pd4 = pd3 + ((float)uncomp_press * (float)uncomp_press * (float)uncomp_press) * g_sBmp390Calib.par_p11;

    return po1 + po2 + pd4;
}

//*****************************************************************************
//
// Initialize the I2C interface and pins for the BMP390 sensor.
//
//*****************************************************************************
bmp390_status_e
bmp390_init(void)
{
    am_bsp_iom_pins_enable(BMP390_IOM_MODULE, AM_HAL_IOM_I2C_MODE);

    if ( am_hal_iom_initialize(BMP390_IOM_MODULE, &g_BMP390IomHandle) )
    {
        return BMP390_ERR_I2C_FAIL;
    }

    if ( am_hal_iom_power_ctrl(g_BMP390IomHandle, AM_HAL_SYSCTRL_WAKE, false) )
    {
        return BMP390_ERR_I2C_FAIL;
    }

    if ( am_hal_iom_configure(g_BMP390IomHandle, &g_sBMP390IomCfg) )
    {
        return BMP390_ERR_I2C_FAIL;
    }

    if ( am_hal_iom_enable(g_BMP390IomHandle) )
    {
        return BMP390_ERR_I2C_FAIL;
    }

    am_util_delay_ms(10);
    return BMP390_OK;
}

void
bmp390_deinit(void)
{
    am_hal_iom_disable(g_BMP390IomHandle);
    am_hal_iom_power_ctrl(g_BMP390IomHandle, AM_HAL_SYSCTRL_DEEPSLEEP, false);
    am_hal_iom_uninitialize(g_BMP390IomHandle);
    am_bsp_iom_pins_disable(BMP390_IOM_MODULE, AM_HAL_IOM_I2C_MODE);
}

bmp390_status_e
bmp390_soft_reset(void)
{
    uint8_t ui8ResetCmd = BMP390_CMD_SOFT_RESET;
    bmp390_status_e eStatus = bmp390_write_bytes(BMP390_REG_CMD, &ui8ResetCmd, 1);
    
    if ( eStatus != BMP390_OK )
    {
        return eStatus;
    }

    am_util_delay_ms(15); // Essential boot-up delay following a hardware reset
    return BMP390_OK;
}

bmp390_status_e
bmp390_read_chip_id(uint8_t *pui8ChipId)
{
    return bmp390_read_bytes(BMP390_REG_CHIP_ID, pui8ChipId, 1);
}

bmp390_status_e
bmp390_configure_normal_mode(void)
{
    uint8_t ui8ConfigRegisterVal;

    ui8ConfigRegisterVal = (uint8_t)((BMP390_OVERSAMPLING_1X << 3) | BMP390_OVERSAMPLING_4X);
    if ( bmp390_write_bytes(BMP390_REG_OSR, &ui8ConfigRegisterVal, 1) != BMP390_OK )
    {
        return BMP390_ERR_I2C_FAIL;
    }

    ui8ConfigRegisterVal = (uint8_t)(BMP390_ODR_50_HZ & 0x1F);
    if ( bmp390_write_bytes(BMP390_REG_ODR, &ui8ConfigRegisterVal, 1) != BMP390_OK )
    {
        return BMP390_ERR_I2C_FAIL;
    }

    ui8ConfigRegisterVal = (uint8_t)(BMP390_IIR_FILTER_COEF3 << 1);
    if ( bmp390_write_bytes(BMP390_REG_CONFIG, &ui8ConfigRegisterVal, 1) != BMP390_OK )
    {
        return BMP390_ERR_I2C_FAIL;
    }

    ui8ConfigRegisterVal = (uint8_t)(BMP390_MODE_NORMAL | BMP390_TEMP_EN | BMP390_PRESS_EN);
    return bmp390_write_bytes(BMP390_REG_PWR_CTRL, &ui8ConfigRegisterVal, 1);
}

bool
bmp390_data_ready(void)
{
    uint8_t ui8Status = 0;
    if ( bmp390_read_bytes(BMP390_REG_STATUS, &ui8Status, 1) != BMP390_OK )
    {
        return false;
    }
    return ((ui8Status & 0x60) == 0x60);
}

float
bmp390_calculate_raw_altitude(float fPressurePa)
{
    return 44330.0f * (1.0f - powf(fPressurePa / 101325.0f, 0.190294957f));
}

bmp390_status_e
bmp390_measure(bmp390_data_t *psData, float fBaselineAltitude)
{
    uint8_t ui8RawDataBuf[6];
    if ( bmp390_read_bytes(BMP390_REG_DATA_0, ui8RawDataBuf, sizeof(ui8RawDataBuf)) != BMP390_OK )
    {
        return BMP390_ERR_I2C_FAIL;
    }

    uint32_t uncomp_press = (uint32_t)ui8RawDataBuf[0] | ((uint32_t)ui8RawDataBuf[1] << 8) | ((uint32_t)ui8RawDataBuf[2] << 16);
    uint32_t uncomp_temp  = (uint32_t)ui8RawDataBuf[3] | ((uint32_t)ui8RawDataBuf[4] << 8) | ((uint32_t)ui8RawDataBuf[5] << 16);

    psData->fTemperatureC = bmp390_compensate_temperature(uncomp_temp);
    psData->fPressurePa   = bmp390_compensate_pressure(uncomp_press);
    
    float fRawAltitude = bmp390_calculate_raw_altitude(psData->fPressurePa);
    psData->fAltitudeM  = fRawAltitude - fBaselineAltitude;

    return BMP390_OK;
}