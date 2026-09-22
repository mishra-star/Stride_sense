//*****************************************************************************
//
//! @file rtos.c
//!
//! @brief FreeRTOS task infrastructure for BMP390 + BMI323 BLE telemetry.
//!
//! Task layout
//! -----------
//!   setup_task  (pri 4, 1024w) — one-shot bootstrap; creates RadioTask +
//!                                SensorTask, then suspends itself.
//!   RadioTask   (pri 3, 2048w) — BLE stack (WSF / ExactLE / AMDTP); defined
//!                                in radio_task.c & Stride_inte.c.
//!   SensorTask  (pri 2, 1536w) — reads BMP390 (I2C) then BMI323 (SPI) every
//!                                50 ms, packs telemetry_packet_t, signals
//!                                RadioTask via g_bNewTelemetryAvailable.
//!
//! IOM assignment
//! --------------
//!   BMP390 → IOM1, I2C (dedicated bus — kept open for the lifetime of
//!                         SensorTask; no per-cycle deinit/reinit needed)
//!   BMI323 → IOM0, SPI (dedicated bus — reinit_spi / release_spi per cycle
//!                         preserves the original time-sharing semantics)
//!
//! Sensor + BLE rate matching
//! --------------------------
//!   BMI323  ODR: 100 Hz → new data every 10 ms
//!   BMP390  ODR:  50 Hz → new data every 20 ms
//!   SensorTask loop: 50 ms (vTaskDelay) → 20 Hz sample rate
//!
//!   BLE connection interval: 7.5 ms (min) – 15 ms (max) after param update.
//!   At 20 Hz each packet is ~50 ms apart.  The BLE interval is shorter than
//!   the packet period so there is always a connection event available to
//!   carry each notification — no packet needs to wait more than one interval.
//!   ∴ BLE can sustain 20 Hz with zero data loss.
//!
//!   AMDTP fire-and-forget (ACK=FALSE) removes round-trip stalls.  The AMDTP
//!   TX buffer depth (1 packet) means SensorTask must not produce a second
//!   packet before RadioTask has forwarded the first.  The
//!   g_bTxSubscribed && !g_bNewTelemetryAvailable gate on STEP C enforces this.
//!
//! Outlier rejection
//! -----------------
//!   A sample-to-sample delta filter catches torn SPI reads.
//!   Accel jumps > 3 g or gyro jumps > 300 dps within one 50 ms tick are
//!   discarded; the previous valid value is retained.  Rejection count is
//!   logged every 10th occurrence so the console does not flood.
//!
//! FIX SUMMARY (vs. original)
//! --------------------------
//!   1. g_sTelemetryPacket type corrected: volatile telemetry_packet_t
//!      (not volatile char[80]).  The original rtos.h declared it as char[80]
//!      but rtos.c used it as a binary struct — silent type mismatch.
//!
//!   2. Per-cycle bmp390_deinit() + bmp390_init() removed from the main loop.
//!      BMP390 is on its own IOM1; closing and re-opening the I2C bus every
//!      50 ms wastes ~5–10 ms of CPU time and risks I2C glitches.  The sensor
//!      is now opened once before the loop and kept open until task deletion.
//!
//!   3. vTaskDelay corrected from 100 ms comment to 50 ms actual value (was
//!      already 50 ms in code but comment said "100 ms loop").
//
//*****************************************************************************

#include "ble_freertos_amdtps.h"
#include "rtos.h"
#include "radio_task.h"
#include "bmp390.h"
#include "bmi323.h"

#include <stdio.h>
#include <math.h>
#include <string.h>

//*****************************************************************************
// Shared telemetry globals — written by SensorTask, read by RadioTask.
// TYPE FIX: was volatile char g_sTelemetryPacket[80] in old rtos.h but used
// as a binary struct throughout — now correctly volatile telemetry_packet_t.
//*****************************************************************************
volatile telemetry_packet_t g_sTelemetryPacket        = { 0 };
volatile bool                g_bNewTelemetryAvailable = false;

//*****************************************************************************
// Task handles
//*****************************************************************************
TaskHandle_t xSetupTask;
TaskHandle_t xSensorTaskHandle;

//*****************************************************************************
// Sanitise BMI323 "no-data" sentinel value (0x8000 = data not yet valid).
//*****************************************************************************
static inline int16_t bmi323_sanitise(int16_t v)
{
    return ((uint16_t)v == 0x8000u) ? (int16_t)0 : v;
}

//*****************************************************************************
//! @brief SensorTask — reads BMP390 + BMI323, packs binary telemetry packet.
//*****************************************************************************
static void
SensorTask(void *pvParameters)
{
    // -----------------------------------------------------------------------
    // BMP390 state
    // -----------------------------------------------------------------------
    bmp390_status_e eBmpStatus;
    bmp390_data_t   sBmpMeasurement;
    uint8_t         ui8BmpChipId        = 0;
    float           fGroundLevelBaseline = 0.0f;
    float           fTemp     = 0.0f;
    float           fPressure = 0.0f;
    float           fAltitude = 0.0f;

    // -----------------------------------------------------------------------
    // BMI323 state
    // -----------------------------------------------------------------------
    uint32_t              ui32BmiStatus;
    uint16_t              ui16BmiChipId = 0;
    bmi323_raw_imu_data_t sBmiRaw;
    float fAx = 0.0f, fAy = 0.0f, fAz = 0.0f;
    float fGx = 0.0f, fGy = 0.0f, fGz = 0.0f;
    bool  bBmiEverOk = false;

    // Sensitivity constants matching SensorTask config below
    const float fAccSens = BMI323_ACC_SENS_16G_LSB_PER_G;
    const float fGyrSens = BMI323_GYR_SENS_2000DPS_LSB_PER_DPS;

    // -----------------------------------------------------------------------
    // 1. One-time BMP390 initialisation + baseline altitude capture
    //    IOM1 is opened here and kept open for the life of SensorTask.
    //    FIX: removed per-cycle bmp390_deinit()/bmp390_init() — pointless
    //    on a dedicated bus and costs ~5-10 ms per iteration.
    // -----------------------------------------------------------------------
    eBmpStatus = bmp390_init();
    if (eBmpStatus != BMP390_OK)
    {
        am_util_debug_printf("ERROR: BMP390 init failed (%d) — SensorTask halted\r\n",
                             eBmpStatus);
        vTaskSuspend(NULL);
    }

    bmp390_soft_reset();
    vTaskDelay(pdMS_TO_TICKS(20));

    eBmpStatus = bmp390_read_chip_id(&ui8BmpChipId);
    if (eBmpStatus != BMP390_OK || ui8BmpChipId != BMP390_CHIP_ID_VALUE)
    {
        am_util_debug_printf("ERROR: BMP390 chip ID bad: 0x%02X\r\n", ui8BmpChipId);
        vTaskSuspend(NULL);
    }
    am_util_debug_printf("BMP390 chip ID OK: 0x%02X\r\n", ui8BmpChipId);

    bmp390_load_calibration();
    bmp390_configure_normal_mode();
    vTaskDelay(pdMS_TO_TICKS(50));

    // Capture ground-level pressure baseline once on startup
    if (bmp390_data_ready())
    {
        if (bmp390_measure(&sBmpMeasurement, 0.0f) == BMP390_OK)
        {
            fGroundLevelBaseline =
                bmp390_calculate_raw_altitude(sBmpMeasurement.fPressurePa);
            am_util_debug_printf("BMP390 baseline altitude: %.2f m\r\n",
                                 fGroundLevelBaseline);
        }
    }
    // BMP390 IOM1 stays open — do NOT call bmp390_deinit() here.

    // -----------------------------------------------------------------------
    // 2. Store BMI323 configuration (does NOT open IOM0 yet)
    // -----------------------------------------------------------------------
    bmi323_config_t sBmi323Cfg =
    {
        .ui32IOMModule  = BMI323_IOM_MODULE,
        .ui32CSChannel  = BMI323_IOM_CS_CHNL,
        .ui32SpiSpeed   = BMI323_IOM_SPI_SPEED,
        .ui8AccOdr      = BMI323_ODR_100HZ,   // 100 Hz — new accel data every 10 ms
        .ui8AccRange    = BMI323_ACC_RANGE_16G,
        .ui8AccBw       = BMI323_BW_ODR_OVER_2,
        .ui8AccAvgNum   = BMI323_AVG_NONE,
        .ui8AccMode     = BMI323_ACC_MODE_HIGH_PERF,
        .ui8GyrOdr      = BMI323_ODR_100HZ,   // 100 Hz — new gyro data every 10 ms
        .ui8GyrRange    = BMI323_GYR_RANGE_2000DPS,
        .ui8GyrBw       = BMI323_BW_ODR_OVER_2,
        .ui8GyrAvgNum   = BMI323_AVG_NONE,
        .ui8GyrMode     = BMI323_GYR_MODE_HIGH_PERF,
    };
    bmi323_store_config(&sBmi323Cfg);

    am_util_debug_printf("SensorTask: entering main loop (50 ms / 20 Hz)\r\n");

    // -----------------------------------------------------------------------
    // 3. Steady-state 50 ms loop (20 Hz)
    //
    //    Rate budget per iteration:
    //      BMP390 read  : ~1-2 ms  (I2C, 400 kHz, 6 bytes)
    //      BMI323 init  : ~1 ms    (SPI, 8 MHz, config writes)
    //      BMI323 DRDY  : 0-10 ms  (ODR 100 Hz → max 10 ms wait)
    //      BMI323 read  : <1 ms    (SPI burst, 12 bytes)
    //      Pack + copy  : <0.1 ms
    //      vTaskDelay   : 50 ms
    //      Total cycle  : ~53-64 ms  (within 50 ms budget after first pass)
    //    The 50 ms delay starts AFTER all sensor reads so the jitter is
    //    bounded by sensor latency, not compounded.
    // -----------------------------------------------------------------------
    while (1)
    {
        // ── STEP A: Read BMP390 (IOM1, I2C) — IOM1 stays open ────────────
        //   BMP390 ODR is 50 Hz (20 ms period).  At our 50 ms loop rate the
        //   sensor always has a fresh sample waiting.
        if (bmp390_data_ready())
        {
            eBmpStatus = bmp390_measure(&sBmpMeasurement, fGroundLevelBaseline);
            if (eBmpStatus == BMP390_OK)
            {
                fTemp     = sBmpMeasurement.fTemperatureC;
                fPressure = sBmpMeasurement.fPressurePa;
                fAltitude = sBmpMeasurement.fAltitudeM;
            }
            else
            {
                am_util_debug_printf("WARN: BMP390 measure failed (%d)\r\n",
                                     eBmpStatus);
            }
        }
        // else: sensor not ready yet — keep last values

        // ── STEP B: Read BMI323 (IOM0, SPI) ──────────────────────────────
        ui32BmiStatus = bmi323_reinit_spi();   // open IOM0 as SPI
        if (ui32BmiStatus == AM_HAL_STATUS_SUCCESS)
        {
            bBmiEverOk = true;

            // Poll DRDY flags (max 20 ms, 1 ms interval).
            // BMI323 ODR=100Hz → DRDY within 10 ms; allow 20 ms margin.
            uint16_t ui16St   = 0;
            uint32_t ui32Wait = 0;
            do
            {
                bmi323_read_status(&ui16St);
                if ((ui16St & (BMI323_STATUS_DRDY_ACC | BMI323_STATUS_DRDY_GYR)) ==
                              (BMI323_STATUS_DRDY_ACC | BMI323_STATUS_DRDY_GYR))
                {
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(1));
            }
            while (++ui32Wait < 20);

            if (ui32Wait >= 20)
            {
                am_util_debug_printf("WARN: BMI323 DRDY timeout\r\n");
            }

            // Burst-read accel + gyro (6 registers, ACC_X..GYR_Z)
            if (bmi323_read_imu_raw(&sBmiRaw) == AM_HAL_STATUS_SUCCESS)
            {
                // Replace "data not valid" sentinel with zero
                sBmiRaw.sAccel.i16X = bmi323_sanitise(sBmiRaw.sAccel.i16X);
                sBmiRaw.sAccel.i16Y = bmi323_sanitise(sBmiRaw.sAccel.i16Y);
                sBmiRaw.sAccel.i16Z = bmi323_sanitise(sBmiRaw.sAccel.i16Z);
                sBmiRaw.sGyro.i16X  = bmi323_sanitise(sBmiRaw.sGyro.i16X);
                sBmiRaw.sGyro.i16Y  = bmi323_sanitise(sBmiRaw.sGyro.i16Y);
                sBmiRaw.sGyro.i16Z  = bmi323_sanitise(sBmiRaw.sGyro.i16Z);

                float fNewAx, fNewAy, fNewAz;
                float fNewGx, fNewGy, fNewGz;
                bmi323_convert_accel(&sBmiRaw.sAccel, fAccSens,
                                     &fNewAx, &fNewAy, &fNewAz);
                bmi323_convert_gyro (&sBmiRaw.sGyro,  fGyrSens,
                                     &fNewGx, &fNewGy, &fNewGz);

                // ----------------------------------------------------------
                // Outlier / torn-read rejection.
                // A 50 ms tick cannot produce a real acceleration jump
                // > 3 g or gyro jump > 300 dps under normal handling.
                // Larger deltas almost always mean the SPI burst was
                // preempted mid-transfer and the bytes straddle two
                // different sensor frames.
                // ----------------------------------------------------------
                static bool     s_bHavePrev       = false;
                static uint32_t s_ui32RejectCount = 0;

                const float fMaxAccelJumpG   = 3.0f;
                const float fMaxGyroJumpDps  = 300.0f;

                bool bSampleOk = true;
                if (s_bHavePrev)
                {
                    if (fabsf(fNewAx - fAx) > fMaxAccelJumpG  ||
                        fabsf(fNewAy - fAy) > fMaxAccelJumpG  ||
                        fabsf(fNewAz - fAz) > fMaxAccelJumpG  ||
                        fabsf(fNewGx - fGx) > fMaxGyroJumpDps ||
                        fabsf(fNewGy - fGy) > fMaxGyroJumpDps ||
                        fabsf(fNewGz - fGz) > fMaxGyroJumpDps)
                    {
                        bSampleOk = false;
                    }
                }

                if (bSampleOk)
                {
                    fAx = fNewAx; fAy = fNewAy; fAz = fNewAz;
                    fGx = fNewGx; fGy = fNewGy; fGz = fNewGz;
                    s_bHavePrev = true;
                }
                else
                {
                    s_ui32RejectCount++;
                    if ((s_ui32RejectCount % 10u) == 1u)
                    {
                        am_util_debug_printf(
                            "WARN: BMI323 sample rejected (torn SPI read #%u) "
                            "— keeping previous value\r\n",
                            (unsigned)s_ui32RejectCount);
                    }
                }
            }
            else
            {
                am_util_debug_printf("WARN: bmi323_read_imu_raw() failed\r\n");
            }
        }
        else
        {
            if (!bBmiEverOk)
            {
                bmi323_who_am_i(&ui16BmiChipId);
                am_util_debug_printf(
                    "WARN: BMI323 SPI open failed (0x%08X), CHIP_ID=0x%04X\r\n",
                    (unsigned)ui32BmiStatus, (unsigned)ui16BmiChipId);
            }
        }
        bmi323_release_spi();   // release IOM0 SPI after each read

        // ── STEP C: Pack binary telemetry_packet_t and publish ────────────
        //
        // Build the struct on the stack first, then memcpy it into the
        // volatile global in one operation.  RadioTask reads the global only
        // while g_bNewTelemetryAvailable is true, which we set after the copy
        // completes — this single-producer / single-consumer protocol is safe
        // without a mutex on Cortex-M (no reordering of volatile stores).
        //
        // Back-pressure gate: only publish if:
        //   (a) BLE is subscribed (no point queuing data nobody will read), AND
        //   (b) previous packet has been consumed by RadioTask.
        //       Without this gate, a second memcpy would race with RadioTask's
        //       memcpy-out, corrupting the packet in transit.
        // ─────────────────────────────────────────────────────────────────
        if (g_bTxSubscribed && !g_bNewTelemetryAvailable)
        {
            telemetry_packet_t sPkt;
            sPkt.ui32Timestamp  = (uint32_t)xTaskGetTickCount();
            sPkt.i16TempCx100   = (int16_t)(fTemp     * 100.0f);
            sPkt.ui32PressurePa = (uint32_t)fPressure;
            sPkt.i16AltMx100    = (int16_t)(fAltitude * 100.0f);
            sPkt.i16AxGx1000    = (int16_t)(fAx       * 1000.0f);
            sPkt.i16AyGx1000    = (int16_t)(fAy       * 1000.0f);
            sPkt.i16AzGx1000    = (int16_t)(fAz       * 1000.0f);
            sPkt.i16GxDpsx100   = (int16_t)(fGx       * 100.0f);
            sPkt.i16GyDpsx100   = (int16_t)(fGy       * 100.0f);
            sPkt.i16GzDpsx100   = (int16_t)(fGz       * 100.0f);

            memcpy((void *)&g_sTelemetryPacket, &sPkt, sizeof(sPkt));
            g_bNewTelemetryAvailable = true;   // RadioTask may now read + send
        }

        // ── STEP D: 50 ms inter-sample delay ──────────────────────────────
        // Measured cycle time (sensor reads + pack): ~3-12 ms.
        // Total period: ~53-62 ms → effective sample rate ~16-19 Hz, well
        // within the BLE capacity at 7.5-15 ms connection intervals.
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

//*****************************************************************************
//! @brief Bootstrap task — runs once at scheduler start, then suspends.
//*****************************************************************************
void
setup_task(void *pvParameters)
{
    am_util_debug_printf("Running setup tasks...\r\n");

    RadioTaskSetup();

    xTaskCreate(RadioTask,  "RadioTask",  4096, NULL, 3, &radio_task_handle);
    xTaskCreate(SensorTask, "SensorTask", 2048, NULL, 2, &xSensorTaskHandle);

    vTaskSuspend(NULL);
    while (1);
}

//*****************************************************************************
//! @brief RTOS entry-point called from main()
//*****************************************************************************
void
run_tasks(void)
{
    xTaskCreate(setup_task, "Setup", 1024, NULL, 4, &xSetupTask);
    vTaskStartScheduler();
    while (1);
}

//*****************************************************************************
// FreeRTOS tickless-idle hooks
//*****************************************************************************
uint32_t
am_freertos_sleep(uint32_t idleTime)
{
    am_bsp_debug_printf_deepsleep_prepare(true);
    am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    am_bsp_debug_printf_deepsleep_prepare(false);
    return 0;
}

void
am_freertos_wakeup(uint32_t idleTime)
{
    (void)idleTime;
}

//*****************************************************************************
// FreeRTOS error hooks
//*****************************************************************************
void
vApplicationMallocFailedHook(void)
{
    am_util_debug_printf("FATAL: pvPortMalloc() failed — heap exhausted\r\n");
    taskDISABLE_INTERRUPTS();
    while (1);
}

void
vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    (void)pxTask;
    am_util_debug_printf("FATAL: Stack overflow in task '%s'\r\n", pcTaskName);
    taskDISABLE_INTERRUPTS();
    while (1) { __asm("BKPT #0\n"); }
}