//*****************************************************************************
//
//  lsm6dsl_i2c.c
//! @file
//!
//! @brief LSM6DSL I2C driver for Ambiq Apollo510B  –  AmbiqSuite SDK 5.1.0
//!
//! I2C protocol notes (datasheet section 5.6)
//! ------------------------------------------
//! • Standard/Fast mode up to 400 kHz.
//! • 7-bit device address: 0x6A (SA0=GND) or 0x6B (SA0=VDD).
//! • Write: START | ADDR+W | SUB | DATA ... | STOP
//! • Read : START | ADDR+W | SUB | RESTART | ADDR+R | DATA ... | STOP
//!   The SUB byte is sent as the IOM "instruction" (ui32InstrLen=1,
//!   ui64Instr=register_address); the IOM HAL handles the repeated START
//!   automatically for reads.
//!
//! Apollo510B IOM I2C HAL notes
//! ----------------------------
//! • uPeerInfo.ui32I2CDevAddr carries the 7-bit address (not shifted).
//! • ui32InstrLen=1 and ui64Instr=register address selects the sub-address.
//! • RX buffer must be 4-byte aligned; bytes are packed little-endian per
//!   32-bit word in the same way as SPI:
//!     word[0] bits [7:0]  = byte 0, [15:8] = byte 1, [23:16] = byte 2, ...
//! • am_bsp_iom_pins_enable(module, AM_HAL_IOM_I2C_MODE) configures
//!   SDA and SCL GPIOs with the correct pin-function and pull-up settings.
//!
//! FIFO notes (datasheet section 5.5)
//! -----------------------------------
//! • FIFO holds up to 4096 × 16-bit words.
//! • Continuous mode (FIFO_MODE[2:0]=110): oldest data overwritten when full.
//! • With DEC_FIFO_GYRO[2:0]=001 and DEC_FIFO_XL[2:0]=001 (both ×1),
//!   the FIFO interleaves: Gx, Gy, Gz, Ax, Ay, Az, Gx, Gy, Gz, Ax, Ay, Az …
//!   i.e., each "set" = 6 words = 12 bytes.
//! • FTH watermark flag (FIFO_STATUS2 bit 7) asserts when unread words ≥ FTH.
//! • FIFO_STATUS1[7:0] + FIFO_STATUS2[3:0] = 12-bit unread-word count.
//! • FIFO_DATA_OUT_L/H (0x3E/0x3F): read one word per pair of bytes.
//!   With IF_INC=1, a burst starting at 0x3E advances address back to 0x3E
//!   every 2 bytes (the FIFO register pair auto-reloads), so we read 2 bytes
//!   per word using repeated single-register reads, or read the full byte
//!   count in one burst (the HAL keeps CS low / repeated-START pattern).
//!
//*****************************************************************************

#include <string.h>
#include "lsm6dsl.h"

//*****************************************************************************
// Private state
//*****************************************************************************
static void    *g_pIOMHandle   = NULL;
static uint8_t  g_ui8DevAddr   = LSM6DSL_I2C_ADDR_DEFAULT;
static uint32_t g_ui32IOMModule;

// FIFO config saved during init (needed by lsm6dsl_i2c_fifo_reset)
static uint8_t  g_ui8FifoCtrl5;    // FIFO_CTRL5 value (ODR | CONTINUOUS)

//*****************************************************************************
// Private helpers
//*****************************************************************************

//! Unpack Apollo IOM receive buffer (packed 32-bit words) into a byte array.
//! Byte i → word [i/4], shift [(i%4)*8].
static void prv_unpack(const uint32_t *pui32Buf, uint8_t *pui8Dst, uint32_t ui32Len)
{
    uint32_t i;
    for (i = 0; i < ui32Len; i++)
    {
        pui8Dst[i] = (uint8_t)((pui32Buf[i / 4] >> ((i % 4) * 8)) & 0xFFu);
    }
}

//*****************************************************************************
// Low-level I2C register access
//*****************************************************************************

uint32_t lsm6dsl_i2c_write_reg(uint8_t ui8Reg, uint8_t ui8Val)
{
    uint32_t              ui32Status;
    am_hal_iom_transfer_t sXfer;
    uint32_t              ui32TxBuf = (uint32_t)ui8Val;

    memset(&sXfer, 0, sizeof(sXfer));
    sXfer.uPeerInfo.ui32I2CDevAddr = g_ui8DevAddr;
    sXfer.ui32InstrLen             = 1;
    sXfer.ui64Instr                = ui8Reg;
    sXfer.eDirection               = AM_HAL_IOM_TX;
    sXfer.ui32NumBytes             = 1;
    sXfer.pui32TxBuffer            = &ui32TxBuf;
    sXfer.bContinue                = false;

    ui32Status = am_hal_iom_blocking_transfer(g_pIOMHandle, &sXfer);
    return ui32Status;
}

uint32_t lsm6dsl_i2c_read_reg(uint8_t ui8Reg, uint8_t *pui8Val)
{
    uint32_t              ui32Status;
    am_hal_iom_transfer_t sXfer;
    uint32_t              ui32RxBuf = 0;

    memset(&sXfer, 0, sizeof(sXfer));
    sXfer.uPeerInfo.ui32I2CDevAddr = g_ui8DevAddr;
    sXfer.ui32InstrLen             = 1;
    sXfer.ui64Instr                = ui8Reg;
    sXfer.eDirection               = AM_HAL_IOM_RX;
    sXfer.ui32NumBytes             = 1;
    sXfer.pui32RxBuffer            = &ui32RxBuf;
    sXfer.bContinue                = false;

    ui32Status = am_hal_iom_blocking_transfer(g_pIOMHandle, &sXfer);
    if (ui32Status == AM_HAL_STATUS_SUCCESS)
    {
        *pui8Val = (uint8_t)(ui32RxBuf & 0xFFu);
    }
    return ui32Status;
}

uint32_t lsm6dsl_i2c_read_burst(uint8_t ui8StartReg,
                                  uint8_t *pui8Buf,
                                  uint32_t ui32Len)
{
    uint32_t              ui32Status;
    am_hal_iom_transfer_t sXfer;
    // Max burst used internally is 12 bytes (gyro+accel) → 3 words.
    // Allocate 16 bytes (4 words) for safety and alignment.
    uint32_t              ui32RxBuf[4];

    if (ui32Len == 0 || ui32Len > 16u)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    memset(ui32RxBuf, 0, sizeof(ui32RxBuf));
    memset(&sXfer, 0, sizeof(sXfer));

    sXfer.uPeerInfo.ui32I2CDevAddr = g_ui8DevAddr;
    sXfer.ui32InstrLen             = 1;
    sXfer.ui64Instr                = ui8StartReg;
    sXfer.eDirection               = AM_HAL_IOM_RX;
    sXfer.ui32NumBytes             = ui32Len;
    sXfer.pui32RxBuffer            = ui32RxBuf;
    sXfer.bContinue                = false;

    ui32Status = am_hal_iom_blocking_transfer(g_pIOMHandle, &sXfer);
    if (ui32Status == AM_HAL_STATUS_SUCCESS)
    {
        prv_unpack(ui32RxBuf, pui8Buf, ui32Len);
    }
    return ui32Status;
}

//*****************************************************************************
// Initialisation
//*****************************************************************************

uint32_t lsm6dsl_i2c_init(const lsm6dsl_i2c_config_t *psConfig)
{
    uint32_t ui32Status;
    uint8_t  ui8Val;

    if (psConfig == NULL) { return AM_HAL_STATUS_INVALID_ARG; }

    g_ui32IOMModule = psConfig->ui32IOMModule;
    g_ui8DevAddr    = psConfig->ui8DevAddr;

    //------------------------------------------------------------------
    // 1. Power on IOM clock
    //------------------------------------------------------------------
    // IOM power peripherals: IOM0 = AM_HAL_PWRCTRL_PERIPH_IOM0, etc.
    // IOM1 = AM_HAL_PWRCTRL_PERIPH_IOM0 + 1 (contiguous enum values).
    ui32Status = am_hal_pwrctrl_periph_enable(
                     (am_hal_pwrctrl_periph_e)(AM_HAL_PWRCTRL_PERIPH_IOM0 +
                                               g_ui32IOMModule));
    if (ui32Status != AM_HAL_STATUS_SUCCESS) { return ui32Status; }

    //------------------------------------------------------------------
    // 2. Initialise IOM handle
    //------------------------------------------------------------------
    ui32Status = am_hal_iom_initialize(g_ui32IOMModule, &g_pIOMHandle);
    if (ui32Status != AM_HAL_STATUS_SUCCESS) { return ui32Status; }

    //------------------------------------------------------------------
    // 3. Wake IOM
    //------------------------------------------------------------------
    ui32Status = am_hal_iom_power_ctrl(g_pIOMHandle, AM_HAL_SYSCTRL_WAKE, false);
    if (ui32Status != AM_HAL_STATUS_SUCCESS) { return ui32Status; }

    //------------------------------------------------------------------
    // 4. Configure I2C master
    //------------------------------------------------------------------
    am_hal_iom_config_t sIomCfg =
    {
        .eInterfaceMode     = AM_HAL_IOM_I2C_MODE,
        .ui32ClockFreq      = psConfig->ui32I2CSpeed,
        .pNBTxnBuf          = NULL,
        .ui32NBTxnBufLength = 0,
    };
    ui32Status = am_hal_iom_configure(g_pIOMHandle, &sIomCfg);
    if (ui32Status != AM_HAL_STATUS_SUCCESS) { return ui32Status; }

    //------------------------------------------------------------------
    // 5. Configure SDA / SCL GPIO pins via BSP helper
    //------------------------------------------------------------------
    am_bsp_iom_pins_enable(g_ui32IOMModule, AM_HAL_IOM_I2C_MODE);

    //------------------------------------------------------------------
    // 6. Enable IOM
    //------------------------------------------------------------------
    ui32Status = am_hal_iom_enable(g_pIOMHandle);
    if (ui32Status != AM_HAL_STATUS_SUCCESS) { return ui32Status; }

    //------------------------------------------------------------------
    // 7. Software reset LSM6DSL  (CTRL3_C bit 0 self-clears after reset)
    //------------------------------------------------------------------
    ui32Status = lsm6dsl_i2c_write_reg(LSM6DSL_REG_CTRL3_C, LSM6DSL_CTRL3_SW_RESET);
    if (ui32Status != AM_HAL_STATUS_SUCCESS) { return ui32Status; }

    {
        uint32_t ui32Timeout = 200;     // 200 × 1 ms = 200 ms max
        do
        {
            am_util_delay_ms(1);
            ui32Status = lsm6dsl_i2c_read_reg(LSM6DSL_REG_CTRL3_C, &ui8Val);
            if (ui32Status != AM_HAL_STATUS_SUCCESS) { return ui32Status; }
        } while ((ui8Val & LSM6DSL_CTRL3_SW_RESET) && (--ui32Timeout));

        if (ui32Timeout == 0) { return AM_HAL_STATUS_TIMEOUT; }
    }

    //------------------------------------------------------------------
    // 8. Verify WHO_AM_I
    //------------------------------------------------------------------
    ui32Status = lsm6dsl_i2c_read_reg(LSM6DSL_REG_WHO_AM_I, &ui8Val);
    if (ui32Status != AM_HAL_STATUS_SUCCESS) { return ui32Status; }
    if (ui8Val != LSM6DSL_WHO_AM_I_VALUE)    { return AM_HAL_STATUS_FAIL; }

    //------------------------------------------------------------------
    // 9. BDU + IF_INC (auto-increment for burst reads)
    //------------------------------------------------------------------
    ui32Status = lsm6dsl_i2c_write_reg(LSM6DSL_REG_CTRL3_C,
                                        LSM6DSL_CTRL3_BDU | LSM6DSL_CTRL3_IF_INC);
    if (ui32Status != AM_HAL_STATUS_SUCCESS) { return ui32Status; }

    //------------------------------------------------------------------
    // 10. Accelerometer ODR + FS
    //------------------------------------------------------------------
    ui32Status = lsm6dsl_i2c_write_reg(LSM6DSL_REG_CTRL1_XL,
                                        psConfig->ui8XLOdr | psConfig->ui8XLFs);
    if (ui32Status != AM_HAL_STATUS_SUCCESS) { return ui32Status; }

    //------------------------------------------------------------------
    // 11. Gyroscope ODR + FS
    //------------------------------------------------------------------
    ui32Status = lsm6dsl_i2c_write_reg(LSM6DSL_REG_CTRL2_G,
                                        psConfig->ui8GOdr | psConfig->ui8GFs);
    if (ui32Status != AM_HAL_STATUS_SUCCESS) { return ui32Status; }

    //------------------------------------------------------------------
    // 12. Mode-specific setup
    //------------------------------------------------------------------
    if (psConfig->eMode == LSM6DSL_MODE_FIFO)
    {
        uint16_t fth = psConfig->ui16FifoWatermark;

        // FIFO_CTRL1: FTH[7:0]
        ui32Status = lsm6dsl_i2c_write_reg(LSM6DSL_REG_FIFO_CTRL1,
                                            (uint8_t)(fth & 0xFFu));
        if (ui32Status != AM_HAL_STATUS_SUCCESS) { return ui32Status; }

        // FIFO_CTRL2: FTH[10:8] in bits [2:0], rest 0
        ui32Status = lsm6dsl_i2c_write_reg(LSM6DSL_REG_FIFO_CTRL2,
                                            (uint8_t)((fth >> 8) & 0x07u));
        if (ui32Status != AM_HAL_STATUS_SUCCESS) { return ui32Status; }

        // FIFO_CTRL3: DEC_FIFO_GYRO[5:3]=001, DEC_FIFO_XL[2:0]=001
        //   Store every gyro and every accel sample (no decimation).
        ui32Status = lsm6dsl_i2c_write_reg(LSM6DSL_REG_FIFO_CTRL3,
                                            (LSM6DSL_FIFO_DEC_1 << 3) |
                                            (LSM6DSL_FIFO_DEC_1));
        if (ui32Status != AM_HAL_STATUS_SUCCESS) { return ui32Status; }

        // FIFO_CTRL4: leave at reset (no extra datasets, no STOP_ON_FTH)
        ui32Status = lsm6dsl_i2c_write_reg(LSM6DSL_REG_FIFO_CTRL4, 0x00u);
        if (ui32Status != AM_HAL_STATUS_SUCCESS) { return ui32Status; }

        // FIFO_CTRL5: ODR_FIFO + FIFO_MODE = Continuous
        g_ui8FifoCtrl5 = (uint8_t)(psConfig->ui8FifoOdr | LSM6DSL_FIFO_MODE_CONTINUOUS);
        ui32Status = lsm6dsl_i2c_write_reg(LSM6DSL_REG_FIFO_CTRL5, g_ui8FifoCtrl5);
        if (ui32Status != AM_HAL_STATUS_SUCCESS) { return ui32Status; }
    }

    //------------------------------------------------------------------
    // 13. Let the accel/gyro digital filters settle after the ODR/FS
    //     change, then flush anything that accumulated in the FIFO
    //     during power-up and this settling delay. Without this, the
    //     first drained FIFO set contains pre-settling transient data,
    //     which shows up as an implausibly huge spike on set #1.
    //------------------------------------------------------------------
    am_util_delay_ms(50);

    if (psConfig->eMode == LSM6DSL_MODE_FIFO)
    {
        ui32Status = lsm6dsl_i2c_fifo_reset();
        if (ui32Status != AM_HAL_STATUS_SUCCESS) { return ui32Status; }
    }

    return AM_HAL_STATUS_SUCCESS;
}

void lsm6dsl_i2c_deinit(void)
{
    if (g_pIOMHandle == NULL) { return; }

    // Power-down sensors
    (void)lsm6dsl_i2c_write_reg(LSM6DSL_REG_CTRL1_XL, 0x00u);
    (void)lsm6dsl_i2c_write_reg(LSM6DSL_REG_CTRL2_G,  0x00u);
    // Stop FIFO
    (void)lsm6dsl_i2c_write_reg(LSM6DSL_REG_FIFO_CTRL5, LSM6DSL_FIFO_MODE_BYPASS);

    am_hal_iom_disable(g_pIOMHandle);
    am_bsp_iom_pins_disable(g_ui32IOMModule, AM_HAL_IOM_I2C_MODE);
    am_hal_iom_power_ctrl(g_pIOMHandle, AM_HAL_SYSCTRL_DEEPSLEEP, false);
    am_hal_iom_uninitialize(g_pIOMHandle);
    g_pIOMHandle = NULL;
}

//*****************************************************************************
// WHO_AM_I
//*****************************************************************************

uint32_t lsm6dsl_i2c_who_am_i(uint8_t *pui8Id)
{
    return lsm6dsl_i2c_read_reg(LSM6DSL_REG_WHO_AM_I, pui8Id);
}

//*****************************************************************************
// Normal mode reads
//*****************************************************************************

uint32_t lsm6dsl_i2c_get_status(uint8_t *pui8Status)
{
    return lsm6dsl_i2c_read_reg(LSM6DSL_REG_STATUS_REG, pui8Status);
}

uint32_t lsm6dsl_i2c_read_accel_raw(lsm6dsl_i2c_raw_t *psData)
{
    uint8_t  buf[6];
    uint32_t ui32Status;

    if (psData == NULL) { return AM_HAL_STATUS_INVALID_ARG; }

    ui32Status = lsm6dsl_i2c_read_burst(LSM6DSL_REG_OUTX_L_XL, buf, 6);
    if (ui32Status != AM_HAL_STATUS_SUCCESS) { return ui32Status; }

    psData->i16X = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);
    psData->i16Y = (int16_t)((uint16_t)buf[3] << 8 | buf[2]);
    psData->i16Z = (int16_t)((uint16_t)buf[5] << 8 | buf[4]);
    return AM_HAL_STATUS_SUCCESS;
}

uint32_t lsm6dsl_i2c_read_gyro_raw(lsm6dsl_i2c_raw_t *psData)
{
    uint8_t  buf[6];
    uint32_t ui32Status;

    if (psData == NULL) { return AM_HAL_STATUS_INVALID_ARG; }

    ui32Status = lsm6dsl_i2c_read_burst(LSM6DSL_REG_OUTX_L_G, buf, 6);
    if (ui32Status != AM_HAL_STATUS_SUCCESS) { return ui32Status; }

    psData->i16X = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);
    psData->i16Y = (int16_t)((uint16_t)buf[3] << 8 | buf[2]);
    psData->i16Z = (int16_t)((uint16_t)buf[5] << 8 | buf[4]);
    return AM_HAL_STATUS_SUCCESS;
}

uint32_t lsm6dsl_i2c_read_gyro_accel_raw(lsm6dsl_i2c_raw_t *psGyro,
                                           lsm6dsl_i2c_raw_t *psAccel)
{
    uint8_t  buf[12];
    uint32_t ui32Status;

    if (psGyro == NULL || psAccel == NULL) { return AM_HAL_STATUS_INVALID_ARG; }

    // 12-byte burst: [0x22..0x2D] = Gx,Gy,Gz (6 bytes) then Ax,Ay,Az (6 bytes)
    ui32Status = lsm6dsl_i2c_read_burst(LSM6DSL_REG_OUTX_L_G, buf, 12);
    if (ui32Status != AM_HAL_STATUS_SUCCESS) { return ui32Status; }

    psGyro->i16X  = (int16_t)((uint16_t)buf[1]  << 8 | buf[0]);
    psGyro->i16Y  = (int16_t)((uint16_t)buf[3]  << 8 | buf[2]);
    psGyro->i16Z  = (int16_t)((uint16_t)buf[5]  << 8 | buf[4]);
    psAccel->i16X = (int16_t)((uint16_t)buf[7]  << 8 | buf[6]);
    psAccel->i16Y = (int16_t)((uint16_t)buf[9]  << 8 | buf[8]);
    psAccel->i16Z = (int16_t)((uint16_t)buf[11] << 8 | buf[10]);
    return AM_HAL_STATUS_SUCCESS;
}

uint32_t lsm6dsl_i2c_read_temp_raw(int16_t *pi16Temp)
{
    uint8_t  buf[2];
    uint32_t ui32Status;

    if (pi16Temp == NULL) { return AM_HAL_STATUS_INVALID_ARG; }

    ui32Status = lsm6dsl_i2c_read_burst(LSM6DSL_REG_OUT_TEMP_L, buf, 2);
    if (ui32Status != AM_HAL_STATUS_SUCCESS) { return ui32Status; }

    *pi16Temp = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
// FIFO mode
//*****************************************************************************

uint32_t lsm6dsl_i2c_fifo_word_count(uint16_t *pui16Count)
{
    uint8_t  s1, s2;
    uint32_t ui32Status;

    if (pui16Count == NULL) { return AM_HAL_STATUS_INVALID_ARG; }

    ui32Status = lsm6dsl_i2c_read_reg(LSM6DSL_REG_FIFO_STATUS1, &s1);
    if (ui32Status != AM_HAL_STATUS_SUCCESS) { return ui32Status; }

    ui32Status = lsm6dsl_i2c_read_reg(LSM6DSL_REG_FIFO_STATUS2, &s2);
    if (ui32Status != AM_HAL_STATUS_SUCCESS) { return ui32Status; }

    // DIFF_FIFO[10:8] in FIFO_STATUS2[3:0], DIFF_FIFO[7:0] in FIFO_STATUS1
    *pui16Count = (uint16_t)(((uint16_t)(s2 & 0x0Fu) << 8) | s1);
    return AM_HAL_STATUS_SUCCESS;
}

uint32_t lsm6dsl_i2c_fifo_watermark_reached(bool *pbReached)
{
    uint8_t  s2;
    uint32_t ui32Status;

    if (pbReached == NULL) { return AM_HAL_STATUS_INVALID_ARG; }

    ui32Status = lsm6dsl_i2c_read_reg(LSM6DSL_REG_FIFO_STATUS2, &s2);
    if (ui32Status != AM_HAL_STATUS_SUCCESS) { return ui32Status; }

    *pbReached = ((s2 & LSM6DSL_FIFO_FTH) != 0u);
    return AM_HAL_STATUS_SUCCESS;
}

uint32_t lsm6dsl_i2c_fifo_read_set(lsm6dsl_i2c_fifo_set_t *psSet)
{
    //
    // Read 6 FIFO words = 12 bytes from FIFO_DATA_OUT_L (0x3E).
    //
    // The FIFO output register pair (0x3E / 0x3F) auto-reloads in the
    // sensor each time a full 16-bit word has been clocked out, so a
    // 12-byte burst starting at 0x3E correctly reads 6 consecutive words.
    //
    // Word order with DEC_FIFO_GYRO=1, DEC_FIFO_XL=1 (set in init):
    //   word 0: Gx   word 1: Gy   word 2: Gz
    //   word 3: Ax   word 4: Ay   word 5: Az
    //
    uint8_t  buf[12];
    uint32_t ui32Status;

    if (psSet == NULL) { return AM_HAL_STATUS_INVALID_ARG; }

    ui32Status = lsm6dsl_i2c_read_burst(LSM6DSL_REG_FIFO_DATA_OUT_L, buf, 12);
    if (ui32Status != AM_HAL_STATUS_SUCCESS) { return ui32Status; }

    psSet->sGyro.i16X  = (int16_t)((uint16_t)buf[1]  << 8 | buf[0]);
    psSet->sGyro.i16Y  = (int16_t)((uint16_t)buf[3]  << 8 | buf[2]);
    psSet->sGyro.i16Z  = (int16_t)((uint16_t)buf[5]  << 8 | buf[4]);
    psSet->sAccel.i16X = (int16_t)((uint16_t)buf[7]  << 8 | buf[6]);
    psSet->sAccel.i16Y = (int16_t)((uint16_t)buf[9]  << 8 | buf[8]);
    psSet->sAccel.i16Z = (int16_t)((uint16_t)buf[11] << 8 | buf[10]);

    return AM_HAL_STATUS_SUCCESS;
}

uint32_t lsm6dsl_i2c_fifo_reset(void)
{
    uint32_t ui32Status;

    // Step 1: Bypass mode clears all stored FIFO data
    ui32Status = lsm6dsl_i2c_write_reg(LSM6DSL_REG_FIFO_CTRL5,
                                        LSM6DSL_FIFO_MODE_BYPASS);
    if (ui32Status != AM_HAL_STATUS_SUCCESS) { return ui32Status; }

    am_util_delay_ms(1);

    // Step 2: Restore Continuous mode
    return lsm6dsl_i2c_write_reg(LSM6DSL_REG_FIFO_CTRL5, g_ui8FifoCtrl5);
}

//*****************************************************************************
// Pedometer / step counter
//*****************************************************************************

uint32_t lsm6dsl_i2c_pedometer_enable(void)
{
    // FUNC_EN + PEDO_EN, using the sensor's default peak threshold
    // (CONFIG_PEDO_THS_MIN, Bank A 0x0F) and debounce (PEDO_DEB_REG, Bank
    // A 0x14) - both default to sane values (0x10 and 0x6E respectively
    // per the datasheet), so no Bank-A access - and no power-down-mode
    // dance that would require - is needed here.
    return lsm6dsl_i2c_write_reg(LSM6DSL_REG_CTRL10_C,
                                  LSM6DSL_CTRL10_FUNC_EN | LSM6DSL_CTRL10_PEDO_EN);
}

uint32_t lsm6dsl_i2c_pedometer_reset(void)
{
    uint32_t ui32Status;

    // Pulse PEDO_RST_STEP high (keeping FUNC_EN/PEDO_EN set so the
    // algorithm stays running), then clear it again.
    ui32Status = lsm6dsl_i2c_write_reg(LSM6DSL_REG_CTRL10_C,
                                        LSM6DSL_CTRL10_FUNC_EN |
                                        LSM6DSL_CTRL10_PEDO_EN |
                                        LSM6DSL_CTRL10_PEDO_RST_STEP);
    if (ui32Status != AM_HAL_STATUS_SUCCESS) { return ui32Status; }

    am_util_delay_ms(1);

    return lsm6dsl_i2c_write_reg(LSM6DSL_REG_CTRL10_C,
                                  LSM6DSL_CTRL10_FUNC_EN | LSM6DSL_CTRL10_PEDO_EN);
}

uint32_t lsm6dsl_i2c_read_step_count(uint16_t *pui16Steps)
{
    uint32_t ui32Status;
    uint8_t  pui8Buf[2];

    // STEP_COUNTER_L (0x4B) / STEP_COUNTER_H (0x4C) are adjacent, and
    // IF_INC (auto-increment) is already enabled in CTRL3_C at init, so a
    // 2-byte burst read works the same as the other multi-byte registers
    // in this driver.
    ui32Status = lsm6dsl_i2c_read_burst(LSM6DSL_REG_STEP_COUNTER_L, pui8Buf, 2);
    if (ui32Status != AM_HAL_STATUS_SUCCESS) { return ui32Status; }

    *pui16Steps = ((uint16_t)pui8Buf[1] << 8) | pui8Buf[0];
    return AM_HAL_STATUS_SUCCESS;
}

uint32_t lsm6dsl_i2c_step_overflowed(bool *pbOverflowed)
{
    uint32_t ui32Status;
    uint8_t  ui8Src;

    ui32Status = lsm6dsl_i2c_read_reg(LSM6DSL_REG_FUNC_SRC1, &ui8Src);
    if (ui32Status != AM_HAL_STATUS_SUCCESS) { return ui32Status; }

    *pbOverflowed = (ui8Src & LSM6DSL_FUNC_SRC1_STEP_OVERFLOW) != 0;
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
// Conversion helpers
//*****************************************************************************

// fSens is in mg/LSB (LSM6DSL_XL_SENS_xx), so raw * fSens = mg directly.
// (Datasheet Table 3: LA_So — e.g. FS=±4g -> 0.122 mg/LSB.)
void lsm6dsl_i2c_convert_accel(const lsm6dsl_i2c_raw_t *psRaw, float fSens,
                                 float *pfX, float *pfY, float *pfZ)
{
    *pfX = (float)psRaw->i16X * fSens;
    *pfY = (float)psRaw->i16Y * fSens;
    *pfZ = (float)psRaw->i16Z * fSens;
}

// fSens is in mdps/LSB (LSM6DSL_G_SENS_xx), so raw * fSens = mdps directly.
// (Datasheet Table 3: G_So — e.g. FS=±500dps -> 17.50 mdps/LSB.)
void lsm6dsl_i2c_convert_gyro(const lsm6dsl_i2c_raw_t *psRaw, float fSens,
                                float *pfX, float *pfY, float *pfZ)
{
    *pfX = (float)psRaw->i16X * fSens;
    *pfY = (float)psRaw->i16Y * fSens;
    *pfZ = (float)psRaw->i16Z * fSens;
}

float lsm6dsl_i2c_convert_temp(int16_t i16Raw)
{
    return LSM6DSL_TEMP_OFFSET + ((float)i16Raw / LSM6DSL_TEMP_SENS);
}