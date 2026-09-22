//*****************************************************************************
//
//! @file bmi323.c
//!
//! @brief BMI323 6-axis IMU driver (SPI on IOM0, shared with BMP390 I2C).
//!
//! Because IOM0 is time-shared with BMP390 (I2C), this driver does NOT hold
//! the IOM open permanently.  Instead:
//!
//!   bmi323_store_config()  — called once at startup; saves config, no IOM touch
//!   bmi323_reinit_spi()    — opens IOM0 as SPI, runs power-on/reset sequence
//!   bmi323_read_imu_raw()  — reads accel + gyro in one burst
//!   bmi323_release_spi()   — disables IOM0 and tristates SPI pins
//!
//! Sequence per 100 ms cycle (rtos.c SensorTask):
//!   bmp390_init()       -> IOM0 I2C open
//!   bmp390_measure()
//!   bmp390_deinit()     -> IOM0 released
//!   bmi323_reinit_spi() -> IOM0 SPI open  (power-on dummy read + CHIP_ID check)
//!   bmi323_read_imu_raw()
//!   bmi323_release_spi()-> IOM0 released
//!
//! Key SPI protocol points (BMI323 datasheet section 7.2.3):
//!   - First read after power-on OR soft-reset: TWO dummy bytes before data.
//!   - All subsequent reads: ONE dummy byte before data.
//!   - Writes: no dummy byte.
//!   - Data registers are 16-bit, LSB first on the wire.
//
//*****************************************************************************

#include <string.h>
#include "bmi323.h"

//*****************************************************************************
// Module state
//*****************************************************************************
static void                *g_pIOMHandle    = NULL;
static am_hal_iom_config_t  g_sIOMConfig;
static uint32_t              g_ui32IOMModule;
static uint32_t              g_ui32CSChannel;

// Saved config for reinit each cycle
static bmi323_config_t       g_sSavedConfig;
static bool                  g_bConfigStored = false;

#define BMI323_BUF_WORDS        4
#define BMI323_MAX_BURST_REGS   7

//*****************************************************************************
// Valid operating-mode values (Table 35, BMI323 datasheet).
// Reserved values (0x01, 0x02, 0x05, 0x06) keep the sensor disabled -> all zeros.
//*****************************************************************************
static bool bmi323_mode_valid(uint8_t mode)
{
    return (mode == 0x00u || mode == 0x03u || mode == 0x04u || mode == 0x07u);
}

//*****************************************************************************
// Standard multi-register read — 1 dummy byte discarded.
// Used for ALL reads EXCEPT the very first one after power-on/reset.
//*****************************************************************************
uint32_t
bmi323_read_registers(uint8_t ui8StartReg, uint16_t *pui16Val, uint32_t ui32NumRegs)
{
    uint32_t              ui32Status;
    am_hal_iom_transfer_t Transaction;
    uint32_t              ui32RxBuf[BMI323_BUF_WORDS];
    uint8_t               pui8Buf[2 * BMI323_MAX_BURST_REGS + 1];
    uint32_t              ui32NumBytes;
    uint32_t              i;

    if (!pui16Val || ui32NumRegs == 0 || ui32NumRegs > BMI323_MAX_BURST_REGS)
        return AM_HAL_STATUS_INVALID_ARG;

    ui32NumBytes = 1 + (2 * ui32NumRegs);   // 1 dummy + 2 bytes per reg

    memset(&Transaction, 0, sizeof(Transaction));
    Transaction.ui32InstrLen                = 1;
    Transaction.ui64Instr                   = (uint64_t)(BMI323_SPI_READ_BIT | (ui8StartReg & 0x7F));
    Transaction.eDirection                  = AM_HAL_IOM_RX;
    Transaction.ui32NumBytes                = ui32NumBytes;
    Transaction.pui32RxBuffer               = ui32RxBuf;
    Transaction.uPeerInfo.ui32SpiChipSelect = g_ui32CSChannel;

    ui32Status = am_hal_iom_blocking_transfer(g_pIOMHandle, &Transaction);
    if (ui32Status != AM_HAL_STATUS_SUCCESS) return ui32Status;

    for (i = 0; i < ui32NumBytes; i++)
        pui8Buf[i] = (uint8_t)((ui32RxBuf[i / 4] >> ((i % 4) * 8)) & 0xFF);

    // Byte 0 is dummy — discard.
    for (i = 0; i < ui32NumRegs; i++)
        pui16Val[i] = (uint16_t)(((uint16_t)pui8Buf[2 + (2*i)] << 8) | pui8Buf[1 + (2*i)]);

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
// Power-on read — 2 dummy bytes discarded (first read after power-on or reset).
//*****************************************************************************
static uint32_t
bmi323_read_poweron(uint8_t ui8Reg, uint16_t *pui16Val)
{
    uint32_t              ui32Status;
    am_hal_iom_transfer_t Transaction;
    uint32_t              ui32RxBuf[BMI323_BUF_WORDS];
    uint8_t               pui8Buf[4];   // 2 dummies + LSB + MSB

    if (!pui16Val) return AM_HAL_STATUS_INVALID_ARG;

    memset(&Transaction, 0, sizeof(Transaction));
    Transaction.ui32InstrLen                = 1;
    Transaction.ui64Instr                   = (uint64_t)(BMI323_SPI_READ_BIT | (ui8Reg & 0x7F));
    Transaction.eDirection                  = AM_HAL_IOM_RX;
    Transaction.ui32NumBytes                = 4;   // 2 dummies + 2 data bytes
    Transaction.pui32RxBuffer               = ui32RxBuf;
    Transaction.uPeerInfo.ui32SpiChipSelect = g_ui32CSChannel;

    ui32Status = am_hal_iom_blocking_transfer(g_pIOMHandle, &Transaction);
    if (ui32Status != AM_HAL_STATUS_SUCCESS) return ui32Status;

    for (uint32_t i = 0; i < 4; i++)
        pui8Buf[i] = (uint8_t)((ui32RxBuf[i / 4] >> ((i % 4) * 8)) & 0xFF);

    // Bytes 0,1 = dummy.  Bytes 2 (LSB), 3 (MSB) = register value.
    *pui16Val = (uint16_t)(((uint16_t)pui8Buf[3] << 8) | pui8Buf[2]);
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
// Multi-register write
//*****************************************************************************
static uint32_t
bmi323_write_registers(uint8_t ui8StartReg, const uint16_t *pui16Val, uint32_t ui32NumRegs)
{
    uint32_t              ui32Status;
    am_hal_iom_transfer_t Transaction;
    uint32_t              ui32TxBuf[BMI323_BUF_WORDS];
    uint8_t               pui8Buf[2 * BMI323_MAX_BURST_REGS];
    uint32_t              ui32NumBytes;
    uint32_t              i;

    if (!pui16Val || ui32NumRegs == 0 || ui32NumRegs > BMI323_MAX_BURST_REGS)
        return AM_HAL_STATUS_INVALID_ARG;

    ui32NumBytes = 2 * ui32NumRegs;

    for (i = 0; i < ui32NumRegs; i++)
    {
        pui8Buf[2*i]   = (uint8_t)(pui16Val[i] & 0xFF);
        pui8Buf[2*i+1] = (uint8_t)((pui16Val[i] >> 8) & 0xFF);
    }

    memset(ui32TxBuf, 0, sizeof(ui32TxBuf));
    for (i = 0; i < ui32NumBytes; i++)
        ui32TxBuf[i/4] |= ((uint32_t)pui8Buf[i]) << ((i%4)*8);

    memset(&Transaction, 0, sizeof(Transaction));
    Transaction.ui32InstrLen                = 1;
    Transaction.ui64Instr                   = (uint64_t)(ui8StartReg & 0x7F);
    Transaction.eDirection                  = AM_HAL_IOM_TX;
    Transaction.ui32NumBytes                = ui32NumBytes;
    Transaction.pui32TxBuffer               = ui32TxBuf;
    Transaction.uPeerInfo.ui32SpiChipSelect = g_ui32CSChannel;

    return am_hal_iom_blocking_transfer(g_pIOMHandle, &Transaction);
}

//*****************************************************************************
// Single-register helpers
//*****************************************************************************
uint32_t bmi323_read_register(uint8_t ui8Reg, uint16_t *pui16Val)
    { return bmi323_read_registers(ui8Reg, pui16Val, 1); }

uint32_t bmi323_write_register(uint8_t ui8Reg, uint16_t ui16Val)
    { return bmi323_write_registers(ui8Reg, &ui16Val, 1); }

uint32_t bmi323_who_am_i(uint16_t *pui16Id)
    { return bmi323_read_register(BMI323_REG_CHIP_ID, pui16Id); }

uint32_t bmi323_read_status(uint16_t *pui16Status)
    { return bmi323_read_register(BMI323_REG_STATUS, pui16Status); }

uint32_t bmi323_soft_reset(void)
    { return bmi323_write_register(BMI323_REG_CMD, BMI323_CMD_SOFT_RESET); }

//*****************************************************************************
// Data reads
//*****************************************************************************
uint32_t
bmi323_read_imu_raw(bmi323_raw_imu_data_t *psData)
{
    uint32_t ui32Status;
    uint16_t pui16Buf[6];

    if (!psData) return AM_HAL_STATUS_INVALID_ARG;

    // Single 6-register burst: ACC_X(0x03)..GYR_Z(0x08)
    ui32Status = bmi323_read_registers(BMI323_REG_ACC_DATA_X, pui16Buf, 6);
    if (ui32Status != AM_HAL_STATUS_SUCCESS) return ui32Status;

    psData->sAccel.i16X = (int16_t)pui16Buf[0];
    psData->sAccel.i16Y = (int16_t)pui16Buf[1];
    psData->sAccel.i16Z = (int16_t)pui16Buf[2];
    psData->sGyro.i16X  = (int16_t)pui16Buf[3];
    psData->sGyro.i16Y  = (int16_t)pui16Buf[4];
    psData->sGyro.i16Z  = (int16_t)pui16Buf[5];
    return AM_HAL_STATUS_SUCCESS;
}

uint32_t
bmi323_read_accel_raw(bmi323_raw_data_t *psData)
{
    uint16_t buf[3];
    uint32_t s = bmi323_read_registers(BMI323_REG_ACC_DATA_X, buf, 3);
    if (s != AM_HAL_STATUS_SUCCESS) return s;
    psData->i16X = (int16_t)buf[0];
    psData->i16Y = (int16_t)buf[1];
    psData->i16Z = (int16_t)buf[2];
    return AM_HAL_STATUS_SUCCESS;
}

uint32_t
bmi323_read_gyro_raw(bmi323_raw_data_t *psData)
{
    uint16_t buf[3];
    uint32_t s = bmi323_read_registers(BMI323_REG_GYR_DATA_X, buf, 3);
    if (s != AM_HAL_STATUS_SUCCESS) return s;
    psData->i16X = (int16_t)buf[0];
    psData->i16Y = (int16_t)buf[1];
    psData->i16Z = (int16_t)buf[2];
    return AM_HAL_STATUS_SUCCESS;
}

void bmi323_convert_accel(const bmi323_raw_data_t *psRaw, float fSens,
                           float *pfX, float *pfY, float *pfZ)
    { *pfX = psRaw->i16X/fSens; *pfY = psRaw->i16Y/fSens; *pfZ = psRaw->i16Z/fSens; }

void bmi323_convert_gyro(const bmi323_raw_data_t *psRaw, float fSens,
                          float *pfX, float *pfY, float *pfZ)
    { *pfX = psRaw->i16X/fSens; *pfY = psRaw->i16Y/fSens; *pfZ = psRaw->i16Z/fSens; }

//*****************************************************************************
//
//  bmi323_store_config() — save config without touching IOM.
//  Called once at startup from SensorTask.
//
//*****************************************************************************
uint32_t
bmi323_store_config(const bmi323_config_t *psConfig)
{
    if (!psConfig) return AM_HAL_STATUS_INVALID_ARG;

    if (!bmi323_mode_valid(psConfig->ui8AccMode) ||
        !bmi323_mode_valid(psConfig->ui8GyrMode))
    {
        am_util_debug_printf(
            "ERROR: BMI323 invalid mode acc=0x%02X gyr=0x%02X. Valid: 0x00,0x03,0x04,0x07\r\n",
            psConfig->ui8AccMode, psConfig->ui8GyrMode);
        return AM_HAL_STATUS_INVALID_ARG;
    }

    g_sSavedConfig  = *psConfig;
    g_ui32IOMModule = psConfig->ui32IOMModule;
    g_ui32CSChannel = psConfig->ui32CSChannel;
    g_bConfigStored = true;

    g_sIOMConfig.eInterfaceMode     = AM_HAL_IOM_SPI_MODE;
    g_sIOMConfig.ui32ClockFreq      = psConfig->ui32SpiSpeed;
    g_sIOMConfig.eSpiMode           = AM_HAL_IOM_SPI_MODE_0;
    g_sIOMConfig.pNBTxnBuf          = NULL;
    g_sIOMConfig.ui32NBTxnBufLength = 0;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//  bmi323_reinit_spi()
//
//  Opens IOM0 as SPI, runs the BMI323 startup sequence, writes ACC/GYR config.
//  Call this AFTER bmp390_deinit() has released IOM0.
//
//  On the very first call, performs a full power-on sequence (dummy read + reset).
//  On subsequent calls, skips the reset to save time (~15 ms) because sensor
//  state is preserved between cycles.
//
//*****************************************************************************
uint32_t
bmi323_reinit_spi(void)
{
    uint32_t ui32Status;
    uint16_t ui16Dummy;
    uint16_t ui16ChipId;
    uint16_t ui16AccConf;
    uint16_t ui16GyrConf;
    static bool s_bFirstInit = true;

    if (!g_bConfigStored) return AM_HAL_STATUS_INVALID_OPERATION;
    if (g_pIOMHandle != NULL)  return AM_HAL_STATUS_SUCCESS;  // already open

    // Open IOM as SPI
    ui32Status = am_hal_iom_initialize(g_ui32IOMModule, &g_pIOMHandle);
    if (ui32Status != AM_HAL_STATUS_SUCCESS) return ui32Status;

    ui32Status = am_hal_iom_power_ctrl(g_pIOMHandle, AM_HAL_SYSCTRL_WAKE, false);
    if (ui32Status != AM_HAL_STATUS_SUCCESS) goto fail;

    ui32Status = am_hal_iom_configure(g_pIOMHandle, &g_sIOMConfig);
    if (ui32Status != AM_HAL_STATUS_SUCCESS) goto fail;

    am_bsp_iom_pins_enable(g_ui32IOMModule, AM_HAL_IOM_SPI_MODE);

    ui32Status = am_hal_iom_enable(g_pIOMHandle);
    if (ui32Status != AM_HAL_STATUS_SUCCESS) goto fail;

    am_util_delay_ms(5);   // VDD settle

    if (s_bFirstInit)
    {
        // First-ever open: full power-on sequence with reset
        // Step 1: 2-dummy-byte read to force SPI mode 
        ui32Status = bmi323_read_poweron(BMI323_REG_CHIP_ID, &ui16Dummy);
        if (ui32Status != AM_HAL_STATUS_SUCCESS) goto fail;
        am_util_delay_ms(3);

        // Step 2: confirm CHIP_ID
        ui32Status = bmi323_who_am_i(&ui16ChipId);
        if (ui32Status != AM_HAL_STATUS_SUCCESS) goto fail;
        if ((ui16ChipId & 0xFF) != (BMI323_CHIP_ID_VALUE & 0xFF))
        {
            am_util_debug_printf("BMI323 CHIP_ID=0x%04X (expect 0x0043) — check wiring\r\n", ui16ChipId);
            ui32Status = AM_HAL_STATUS_FAIL;
            goto fail;
        }
        am_util_debug_printf("BMI323 CHIP_ID OK: 0x%04X\r\n", ui16ChipId);

        // Step 3: soft reset
        bmi323_soft_reset();
        am_util_delay_ms(10);   // datasheet: ~2 ms; 10 ms for margin

        // Step 4: re-force SPI mode after reset (another 2-dummy read)
        ui32Status = bmi323_read_poweron(BMI323_REG_CHIP_ID, &ui16Dummy);
        if (ui32Status != AM_HAL_STATUS_SUCCESS) goto fail;
        am_util_delay_ms(3);

        // Step 5: verify CHIP_ID post-reset
        ui32Status = bmi323_who_am_i(&ui16ChipId);
        if (ui32Status != AM_HAL_STATUS_SUCCESS) goto fail;
        if ((ui16ChipId & 0xFF) != (BMI323_CHIP_ID_VALUE & 0xFF))
        {
            ui32Status = AM_HAL_STATUS_FAIL;
            goto fail;
        }

        s_bFirstInit = false;
    }
    else
    {
        // Subsequent opens: sensor retains config; just re-force SPI mode.
        // Only 1 dummy byte needed here because sensor has been powered
        // continuously (no reset between cycles).
        ui32Status = bmi323_read_register(BMI323_REG_CHIP_ID, &ui16ChipId);
        if (ui32Status != AM_HAL_STATUS_SUCCESS) goto fail;
        if ((ui16ChipId & 0xFF) != (BMI323_CHIP_ID_VALUE & 0xFF))
        {
            // Unexpected — sensor lost power; fall back to full init next time
            s_bFirstInit = true;
            ui32Status = AM_HAL_STATUS_FAIL;
            goto fail;
        }
    }

    // Write ACC_CONF and GYR_CONF every open so config is always current
    ui16AccConf = (uint16_t)(
        ((g_sSavedConfig.ui8AccOdr    & 0x0F) << BMI323_CONF_ODR_SHIFT)   |
        ((g_sSavedConfig.ui8AccRange  & 0x07) << BMI323_CONF_RANGE_SHIFT) |
        ((g_sSavedConfig.ui8AccBw     & 0x01) << BMI323_CONF_BW_SHIFT)    |
        ((g_sSavedConfig.ui8AccAvgNum & 0x07) << BMI323_CONF_AVG_SHIFT)   |
        ((g_sSavedConfig.ui8AccMode   & 0x07) << BMI323_CONF_MODE_SHIFT)
    );
    ui32Status = bmi323_write_register(BMI323_REG_ACC_CONF, ui16AccConf);
    if (ui32Status != AM_HAL_STATUS_SUCCESS) goto fail;

    ui16GyrConf = (uint16_t)(
        ((g_sSavedConfig.ui8GyrOdr    & 0x0F) << BMI323_CONF_ODR_SHIFT)   |
        ((g_sSavedConfig.ui8GyrRange  & 0x07) << BMI323_CONF_RANGE_SHIFT) |
        ((g_sSavedConfig.ui8GyrBw     & 0x01) << BMI323_CONF_BW_SHIFT)    |
        ((g_sSavedConfig.ui8GyrAvgNum & 0x07) << BMI323_CONF_AVG_SHIFT)   |
        ((g_sSavedConfig.ui8GyrMode   & 0x07) << BMI323_CONF_MODE_SHIFT)
    );
    ui32Status = bmi323_write_register(BMI323_REG_GYR_CONF, ui16GyrConf);
    if (ui32Status != AM_HAL_STATUS_SUCCESS) goto fail;

    am_util_delay_ms(5);   // let config take effect
    return AM_HAL_STATUS_SUCCESS;

fail:
    bmi323_release_spi();
    return ui32Status;
}

//*****************************************************************************
//
//  bmi323_release_spi()
//
//  Disables IOM0 SPI and tristates the SPI pins so bmp390_init() can
//  reconfigure them as I2C immediately after.
//
//*****************************************************************************
void
bmi323_release_spi(void)
{
    if (g_pIOMHandle != NULL)
    {
        am_hal_iom_disable(g_pIOMHandle);
        am_bsp_iom_pins_disable(g_ui32IOMModule, AM_HAL_IOM_SPI_MODE);
        am_hal_iom_power_ctrl(g_pIOMHandle, AM_HAL_SYSCTRL_DEEPSLEEP, false);
        am_hal_iom_uninitialize(g_pIOMHandle);
        g_pIOMHandle = NULL;
    }
}

//*****************************************************************************
// Kept for backward-compat with bmi323.h declaration.
// In time-shared mode use bmi323_store_config() + bmi323_reinit_spi() instead.
//*****************************************************************************
uint32_t
bmi323_init(const bmi323_config_t *psConfig)
{
    uint32_t s = bmi323_store_config(psConfig);
    if (s != AM_HAL_STATUS_SUCCESS) return s;
    return bmi323_reinit_spi();
}

void
bmi323_deinit(void)
{
    bmi323_write_register(BMI323_REG_ACC_CONF, 0x0000);
    bmi323_write_register(BMI323_REG_GYR_CONF, 0x0000);
    bmi323_release_spi();
}

//*****************************************************************************
// Sensor timestamp
//*****************************************************************************
uint32_t
bmi323_get_sensor_time(uint32_t *pui32SensorTime)
{
    uint16_t ui16Time[2];
    uint32_t s = bmi323_read_registers(BMI323_REG_SENSOR_TIME_0, ui16Time, 2);
    if (s != AM_HAL_STATUS_SUCCESS) return s;
    *pui32SensorTime = ((uint32_t)(ui16Time[1] & 0x00FF) << 16) |
                        (uint32_t)(ui16Time[0] & 0xFFFF);
    return AM_HAL_STATUS_SUCCESS;
}