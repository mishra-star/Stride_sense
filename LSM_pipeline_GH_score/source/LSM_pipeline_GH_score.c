#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"
#include "gait_pipeline.h"
#include "lsm6dsl.h"
#include "ff.h"          // FatFs - SD-card CSV logging
#include <string.h>
#include <stdio.h>       // snprintf()
#include <math.h>        // isfinite()
#include "Gait_health_score.h"

//*****************************************************************************
// >>> EDIT HERE to switch between the real LSM6DSL sensor and a replayed
// CSV recording <<<
//   0 = normal operation: samples come from the physical LSM6DSL over I2C/FIFO.
//   1 = simulation mode: samples come from the embedded IMU0032.CSV dataset.
//*****************************************************************************
#define SIM_CSV_INPUT   0

#if SIM_CSV_INPUT
#include "sim_imu_data.h"
#endif

// Static buffers
// Large arrays placed in SHARED_SRAM (0x20080000, 3 MB) via SHARED_SRAM
// attribute defined in gait_pipeline.h. The .shared section is NOLOAD so
// they are zeroed explicitly in main() before first use.
static SHARED_SRAM imu_sample_t  samples[MAX_SAMPLES];
static SHARED_SRAM imu_sample_t  cb_buffer[MAX_SAMPLES];
static circular_buffer_t cb;                              // small struct, stays in TCM
static SHARED_SRAM gait_event_t  events[MAX_EVENTS];
static SHARED_SRAM stride_t      strides[MAX_STRIDES];
static SHARED_SRAM temporal_params_t temp_params[MAX_SESSION_STRIDES];
static SHARED_SRAM spatial_params_t spat_params[MAX_SESSION_STRIDES];
static SHARED_SRAM feature_vector_t feature_vectors[MAX_SESSION_STRIDES];
static SHARED_SRAM stride_t session_strides[MAX_SESSION_STRIDES];
static SHARED_SRAM traj_sample_t trajectory[MAX_SAMPLES];

// The simulation has a hardware pedometer count. When the event detector
// misses a cycle, fill the largest gap with an explicitly boundary-imputed
// stride instead of reporting a known-wrong session count. This is only used
// for SIM_CSV_INPUT; live hardware retains the raw detector result.
#if SIM_CSV_INPUT
static uint32_t reconcile_sim_strides_to_hardware(uint32_t count,
                                                   uint32_t target)
{
    while (count < target && count < MAX_SESSION_STRIDES && count >= 2u) {
        uint32_t left_idx = 0u;
        float largest_gap = 0.0f;
        for (uint32_t i = 0; i + 1u < count; i++) {
            float gap = session_strides[i + 1u].td_time_ms -
                        session_strides[i].td_time_ms;
            if (gap > largest_gap) {
                largest_gap = gap;
                left_idx = i;
            }
        }
        if (largest_gap <= 0.0f) break;

        stride_t left = session_strides[left_idx];
        stride_t right = session_strides[left_idx + 1u];
        spatial_params_t left_spat = spat_params[left_idx];
        spatial_params_t right_spat = spat_params[left_idx + 1u];
        float mid_td = left.td_time_ms + largest_gap * 0.5f;
        float ratio = temp_params[left_idx].stance_ratio;
        if (ratio < 0.10f || ratio > 0.90f) ratio = 0.63f;

        // Make the preceding detected stride end at the inserted touchdown.
        session_strides[left_idx].td_next_ms = mid_td;
        session_strides[left_idx].to_time_ms = left.td_time_ms +
            (mid_td - left.td_time_ms) * ratio;
        session_strides[left_idx].td_next_sample =
            (left.td_sample + right.td_sample) / 2u;
        step4_temporal(&session_strides[left_idx], &temp_params[left_idx]);
        step4_temporal_with_speed(&temp_params[left_idx], &spat_params[left_idx]);
        extract_feature_vector(&temp_params[left_idx], &spat_params[left_idx],
                               &feature_vectors[left_idx]);

        for (uint32_t j = count; j > left_idx + 1u; j--) {
            session_strides[j] = session_strides[j - 1u];
            temp_params[j] = temp_params[j - 1u];
            spat_params[j] = spat_params[j - 1u];
            feature_vectors[j] = feature_vectors[j - 1u];
        }

        stride_t imputed = right;
        imputed.td_time_ms = mid_td;
        imputed.td_next_ms = right.td_time_ms;
        imputed.to_time_ms = mid_td + (right.td_time_ms - mid_td) * ratio;
        imputed.td_sample = (left.td_sample + right.td_sample) / 2u;
        imputed.to_sample = (imputed.td_sample + right.td_sample) / 2u;
        imputed.td_next_sample = right.td_sample;
        imputed.is_boundary_imputed = true;
        imputed.is_outlier = false;
        session_strides[left_idx + 1u] = imputed;

        spatial_params_t *sp = &spat_params[left_idx + 1u];
        sp->stride_length_m = (left_spat.stride_length_m + right_spat.stride_length_m) * 0.5f;
        sp->min_clearance_m = (left_spat.min_clearance_m + right_spat.min_clearance_m) * 0.5f;
        sp->max_clearance_m = (left_spat.max_clearance_m + right_spat.max_clearance_m) * 0.5f;
        sp->vertical_osc_cm = (left_spat.vertical_osc_cm + right_spat.vertical_osc_cm) * 0.5f;
        sp->ilr_g_per_ms = (left_spat.ilr_g_per_ms + right_spat.ilr_g_per_ms) * 0.5f;
        sp->pronation_angle_deg = (left_spat.pronation_angle_deg + right_spat.pronation_angle_deg) * 0.5f;
        sp->angle_change = (left_spat.angle_change + right_spat.angle_change) * 0.5f;
        step4_temporal(&imputed, &temp_params[left_idx + 1u]);
        step4_temporal_with_speed(&temp_params[left_idx + 1u], sp);
        extract_feature_vector(&temp_params[left_idx + 1u], sp,
                               &feature_vectors[left_idx + 1u]);
        count++;
    }

    for (uint32_t i = 0; i < count; i++) session_strides[i].stride_idx = i;
    return count;
}
#endif

// ============================================================================
// SESSION DURATION: 3 minutes total
// ============================================================================
#define LSM_SESSION_MINUTES      3u
#define LSM_SESSION_SECONDS      (LSM_SESSION_MINUTES * 60u)  // 180 s
#define LSM_MINUTE_INTERVAL_S    60u   // trigger gait report every 60 s

// LSM6DSL configuration and data
static lsm6dsl_i2c_config_t   sLsm6dslCfg;
static lsm6dsl_i2c_raw_t      sAccelRaw;

#define LSM_XL_ODR      LSM6DSL_XL_ODR_104HZ
#define LSM_G_ODR       LSM6DSL_G_ODR_104HZ
#define LSM_XL_FS_G     16

#if   LSM_XL_FS_G == 2
    #define LSM_XL_FS_REG   LSM6DSL_XL_FS_2G
    #define LSM_XL_SENS     LSM6DSL_XL_SENS_2G
#elif LSM_XL_FS_G == 4
    #define LSM_XL_FS_REG   LSM6DSL_XL_FS_4G
    #define LSM_XL_SENS     LSM6DSL_XL_SENS_4G
#elif LSM_XL_FS_G == 8
    #define LSM_XL_FS_REG   LSM6DSL_XL_FS_8G
    #define LSM_XL_SENS     LSM6DSL_XL_SENS_8G
#elif LSM_XL_FS_G == 16
    #define LSM_XL_FS_REG   LSM6DSL_XL_FS_16G
    #define LSM_XL_SENS     LSM6DSL_XL_SENS_16G
#else
    #error "LSM_XL_FS_G must be 2, 4, 8, or 16"
#endif

#define LSM_G_FS_DPS    2000

#if   LSM_G_FS_DPS == 250
    #define LSM_G_FS_REG   LSM6DSL_G_FS_250DPS
    #define LSM_G_SENS     LSM6DSL_G_SENS_250DPS
#elif LSM_G_FS_DPS == 500
    #define LSM_G_FS_REG   LSM6DSL_G_FS_500DPS
    #define LSM_G_SENS     LSM6DSL_G_SENS_500DPS
#elif LSM_G_FS_DPS == 1000
    #define LSM_G_FS_REG   LSM6DSL_G_FS_1000DPS
    #define LSM_G_SENS     LSM6DSL_G_SENS_1000DPS
#elif LSM_G_FS_DPS == 2000
    #define LSM_G_FS_REG   LSM6DSL_G_FS_2000DPS
    #define LSM_G_SENS     LSM6DSL_G_SENS_2000DPS
#else
    #error "LSM_G_FS_DPS must be 250, 500, 1000, or 2000"
#endif

static const float g_fAccSensLsbPerG      = 1000.0f / LSM_XL_SENS;
static const float g_fGyrSensLsbPerDps    = 1000.0f / LSM_G_SENS;

#define LSM_FIFO_ODR    LSM6DSL_FIFO_ODR_104HZ

#define FIFO_POLL_MS             10
#define FIFO_READ_BATCH          64
#define FIFO_WARN_WORDS        3600

static uint32_t g_ui32FifoWarnings  = 0;
static uint32_t g_ui32FifoOverruns  = 0;

static uint16_t g_ui16HwStepCount = 0;

/* Python-parity hardware-assisted peak-selection state. */
static float g_fHwStridePeriodS = 0.0f;
static bool  g_bHwCadenceValid = false;

#if SIM_CSV_INPUT
static uint32_t sim_hw_step_total(void)
{
    uint32_t max_steps = 0;
    for (uint32_t i = 0; i < SIM_IMU_DATA_COUNT; i++) {
        if (g_sSimImuData[i].steps > max_steps)
            max_steps = g_sSimImuData[i].steps;
    }
    return max_steps;
}
#endif

static uint32_t hw_target_peaks_for_window(uint32_t window_samples, float sr,
                                           uint32_t hw_steps, float elapsed_s)
{
    if (sr <= 0.0f || window_samples < 3u || hw_steps < 4u || elapsed_s <= 1.0f)
        return 0u;

    uint32_t hw_strides = hw_steps / 2u;
    if (hw_strides == 0u) return 0u;

    float stride_period = elapsed_s / (float)hw_strides;
    if (stride_period < HW_PEAK_DISTANCE_MIN_S) stride_period = HW_PEAK_DISTANCE_MIN_S;
    if (stride_period > HW_PEAK_DISTANCE_MAX_S) stride_period = HW_PEAK_DISTANCE_MAX_S;

    g_fHwStridePeriodS = stride_period;
    g_bHwCadenceValid = true;

    float window_s = (float)window_samples / sr;
    uint32_t expected = (uint32_t)(window_s / stride_period + 0.5f) + 1u;
    if (expected < 2u) expected = 2u;
    if (expected > MAX_EVENTS / 2u) expected = MAX_EVENTS / 2u;
    return expected;
}

static bool     g_bFirstSecondSkipped = false;
static uint32_t g_ui32LoggingStartCs  = 0;
static bool     g_bRecordingDone      = false;

#if SIM_CSV_INPUT
static uint32_t g_ui32SimReadIdx     = 0;
static float    g_fSimTimeOffsetMs   = 0.0f;
static uint32_t g_ui32SimHwSteps     = 0;
// Baseline: first generated simulation timestamp in milliseconds.
// Subtracted so the emitted time_ms starts at zero.
static float    g_fSimBaselineMs     = 0.0f;

// ============================================================================
// The static sim array is generated with elapsed millisecond timestamps.
//
// Do not run them through an HHMMSSCC/centisecond decoder: doing so corrupts
// the ESKF time deltas and causes trajectory and stride-length divergence.
// ============================================================================
#define SIM_SAMPLES_PER_POLL \
    ((uint32_t)((FIFO_POLL_MS * (SIM_IMU_DATA_COUNT / SIM_IMU_DATA_DURATION_MS)) + 0.5f))
#endif

//LED
static void led2_init(void)
{
    am_hal_gpio_pinconfig(AM_BSP_GPIO_LED2, g_AM_BSP_GPIO_LED2);
    am_hal_gpio_state_write(AM_BSP_GPIO_LED2, AM_HAL_GPIO_OUTPUT_SET);
}

static void led1_init(void)
{
    am_hal_gpio_pinconfig(AM_BSP_GPIO_LED1, g_AM_BSP_GPIO_LED1);
    am_hal_gpio_state_write(AM_BSP_GPIO_LED1, AM_HAL_GPIO_OUTPUT_SET);
}

static void led2_on(void)
{
    am_hal_gpio_state_write(AM_BSP_GPIO_LED2, AM_HAL_GPIO_OUTPUT_CLEAR);
}

static void led1_on(void)
{
    am_hal_gpio_state_write(AM_BSP_GPIO_LED1, AM_HAL_GPIO_OUTPUT_CLEAR);
}

//UART
static void *phUART = NULL;

#define CHECK_ERRORS(x)                             \
    if ((x) != AM_HAL_STATUS_SUCCESS)               \
    {                                               \
        error_handler(x);                           \
    }

volatile uint32_t ui32LastError;

static void
error_handler(uint32_t ui32ErrorStatus)
{
    ui32LastError = ui32ErrorStatus;
    while (1);
}

uint8_t g_pui8TxBuffer[256];
uint8_t g_pui8RxBuffer[2];

static const am_hal_uart_config_t g_sUartConfig =
{
    .ui32BaudRate  = 115200,
    .eDataBits     = AM_HAL_UART_DATA_BITS_8,
    .eParity       = AM_HAL_UART_PARITY_NONE,
    .eStopBits     = AM_HAL_UART_ONE_STOP_BIT,
    .eFlowControl  = AM_HAL_UART_FLOW_CTRL_NONE,
    .eTXFifoLevel  = AM_HAL_UART_FIFO_LEVEL_16,
    .eRXFifoLevel  = AM_HAL_UART_FIFO_LEVEL_16,
};

#if AM_BSP_UART_PRINT_INST == 0
void am_uart_isr(void)
#elif AM_BSP_UART_PRINT_INST == 1
void am_uart1_isr(void)
#elif AM_BSP_UART_PRINT_INST == 2
void am_uart2_isr(void)
#elif AM_BSP_UART_PRINT_INST == 3
void am_uart3_isr(void)
#endif
{
    uint32_t ui32Status;
    am_hal_uart_interrupt_status_get(phUART, &ui32Status, true);
    am_hal_uart_interrupt_clear(phUART, ui32Status);
    am_hal_uart_interrupt_service(phUART, ui32Status);
}

static void
uart_print(char *pcStr)
{
    uint32_t ui32StrLen       = 0;
    uint32_t ui32BytesWritten = 0;

    while (pcStr[ui32StrLen] != 0)
    {
        ui32StrLen++;
    }

    am_hal_uart_transfer_t sUartWrite =
    {
        .eType                 = AM_HAL_UART_BLOCKING_WRITE,
        .pui8Data              = (uint8_t *)pcStr,
        .ui32NumBytes          = ui32StrLen,
        .pui32BytesTransferred = &ui32BytesWritten,
        .ui32TimeoutMs         = 100,
        .pfnCallback           = NULL,
        .pvContext             = NULL,
        .ui32ErrorStatus       = 0,
    };

    CHECK_ERRORS(am_hal_uart_transfer(phUART, &sUartWrite));
    if (ui32BytesWritten != ui32StrLen)
    {
        while (1);
    }
}

static void
UART_printf_init(void)
{
    CHECK_ERRORS(am_hal_uart_initialize(AM_BSP_UART_PRINT_INST, &phUART));
    CHECK_ERRORS(am_hal_uart_power_control(phUART, AM_HAL_SYSCTRL_WAKE, false));
    CHECK_ERRORS(am_hal_uart_configure(phUART, &g_sUartConfig));

    am_hal_gpio_pinconfig(AM_BSP_GPIO_COM_UART_TX, g_AM_BSP_GPIO_COM_UART_TX);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_COM_UART_RX, g_AM_BSP_GPIO_COM_UART_RX);

    NVIC_SetPriority((IRQn_Type)(UART0_IRQn + AM_BSP_UART_PRINT_INST),
                     AM_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ((IRQn_Type)(UART0_IRQn + AM_BSP_UART_PRINT_INST));
    am_hal_interrupt_master_enable();

    am_util_stdio_printf_init(uart_print);
}

// RTC
#define FIFO_ODR_HZ    104.0f

typedef struct {
    uint8_t ui8Hour;
    uint8_t ui8Minute;
    uint8_t ui8Second;
    uint8_t ui8Hundredths;
} rtc_stamp_t;

static uint32_t g_ui32EpochCs = 0;

static void rtc_init(void)
{
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_RTC_SEL_XTAL, 0);
    am_hal_rtc_osc_select(AM_HAL_RTC_OSC_XT);
    am_hal_rtc_osc_enable();

    am_hal_rtc_time_t build_time = {0};
    build_time.ui32Hour       = ((__TIME__[0] - '0') * 10u) + (__TIME__[1] - '0');
    build_time.ui32Minute     = ((__TIME__[3] - '0') * 10u) + (__TIME__[4] - '0');
    build_time.ui32Second     = ((__TIME__[6] - '0') * 10u) + (__TIME__[7] - '0');
    build_time.ui32Hundredths = 0;
    build_time.ui32Weekday    = 1;
    build_time.ui32DayOfMonth = 1;
    build_time.ui32Month      = 1;
    build_time.ui32Year       = 0;
    build_time.ui32CenturyBit = RTC_CTRUP_CB_2000;

    uint32_t ui32BuildCs = (build_time.ui32Hour * 3600u +
                            build_time.ui32Minute * 60u +
                            build_time.ui32Second) * 100u;

    am_hal_rtc_time_t cur_time;
    am_hal_rtc_time_get(&cur_time);
    uint32_t ui32CurCs = (cur_time.ui32Hour * 3600u +
                          cur_time.ui32Minute * 60u +
                          cur_time.ui32Second) * 100u + cur_time.ui32Hundredths;

    if (ui32CurCs <= ui32BuildCs) {
        am_hal_rtc_time_set(&build_time);
        am_util_stdio_printf("RTC set to flash time: %02lu:%02lu:%02lu\r\n",
                             (unsigned long)build_time.ui32Hour,
                             (unsigned long)build_time.ui32Minute,
                             (unsigned long)build_time.ui32Second);
    } else {
        am_util_stdio_printf("RTC already past this build's flash time - "
                             "continuing at %02lu:%02lu:%02lu (not reset)\r\n",
                             (unsigned long)cur_time.ui32Hour,
                             (unsigned long)cur_time.ui32Minute,
                             (unsigned long)cur_time.ui32Second);
    }
}

static void get_rtc_time(am_hal_rtc_time_t *pRtcTime)
{
    am_hal_rtc_time_get(pRtcTime);
}

static uint32_t rtc_to_centiseconds(const am_hal_rtc_time_t *pRtcTime)
{
    return ((pRtcTime->ui32Hour * 3600u + pRtcTime->ui32Minute * 60u +
             pRtcTime->ui32Second) * 100u) + pRtcTime->ui32Hundredths;
}

static void centiseconds_to_hms(uint32_t ui32Cs, uint8_t *pui8H, uint8_t *pui8M,
                                 uint8_t *pui8S, uint8_t *pui8Cc)
{
    ui32Cs   %= 8640000u;
    *pui8Cc   = (uint8_t)(ui32Cs % 100u);  ui32Cs /= 100u;
    *pui8S    = (uint8_t)(ui32Cs % 60u);   ui32Cs /= 60u;
    *pui8M    = (uint8_t)(ui32Cs % 60u);   ui32Cs /= 60u;
    *pui8H    = (uint8_t)(ui32Cs % 24u);
}

static void resync_epoch(uint32_t ui32AtSeq)
{
    am_hal_rtc_time_t rtc_time;
    get_rtc_time(&rtc_time);
    g_ui32EpochCs = rtc_to_centiseconds(&rtc_time) -
        (uint32_t)((ui32AtSeq * 100.0f / FIFO_ODR_HZ) + 0.5f);
}

static rtc_stamp_t g_sLastSampleStamp = {0};

static uint32_t grab_fifo_samples(imu_sample_t *psOut, rtc_stamp_t *psStamps,
                                   uint32_t ui32MaxOut, uint32_t *pui32SeqNext)
{
#if SIM_CSV_INPUT
    uint32_t ui32Want = SIM_SAMPLES_PER_POLL;
    if (ui32Want == 0) ui32Want = 1;
    if (ui32Want > FIFO_READ_BATCH) ui32Want = FIFO_READ_BATCH;
    if (ui32Want > ui32MaxOut)      ui32Want = ui32MaxOut;

    uint32_t ui32Grabbed = 0;
    for (uint32_t i = 0; i < ui32Want; i++) {
        if (g_ui32SimReadIdx >= SIM_IMU_DATA_COUNT) {
            g_ui32SimReadIdx = 0;
            g_fSimTimeOffsetMs += SIM_IMU_DATA_DURATION_MS;
        }
        const sim_imu_row_t *psRow = &g_sSimImuData[g_ui32SimReadIdx];

        imu_sample_t *psSample = &psOut[ui32Grabbed];
        // sim_imu_data.c already stores elapsed milliseconds at 104 Hz.
        psSample->time_ms  = g_fSimTimeOffsetMs
                           + (float)psRow->time_ms
                           - g_fSimBaselineMs;
        psSample->acc_x    = psRow->acc_x;
        psSample->acc_y    = psRow->acc_y;
        psSample->acc_z    = psRow->acc_z;
        psSample->gyro_x   = psRow->gyro_x;
        psSample->gyro_y   = psRow->gyro_y;
        psSample->gyro_z   = psRow->gyro_z;
        psSample->gyro_mag = 0.0f;
        psSample->phase    = 0;
        // `steps` is a cumulative counter within one recorded pass. The
        // 3-minute demo can wrap the shorter static recording; never replace
        // the completed recording's count with a smaller value from its
        // replayed beginning (e.g. 250 must not become 58 after a wrap).
        if (psRow->steps > g_ui32SimHwSteps) {
            g_ui32SimHwSteps = psRow->steps;
        }
        g_ui16HwStepCount = (uint16_t)g_ui32SimHwSteps;

        uint32_t ui32SampleCs = (uint32_t)(psSample->time_ms / 10.0f + 0.5f);
        rtc_stamp_t sStamp;
        centiseconds_to_hms(ui32SampleCs, &sStamp.ui8Hour, &sStamp.ui8Minute,
                             &sStamp.ui8Second, &sStamp.ui8Hundredths);
        if (psStamps != NULL) {
            psStamps[ui32Grabbed] = sStamp;
        }
        g_sLastSampleStamp = sStamp;

        g_ui32SimReadIdx++;
        (*pui32SeqNext)++;
        ui32Grabbed++;
    }
    return ui32Grabbed;
#else
    uint16_t ui16FillWords = 0;
    uint32_t ui32Status = lsm6dsl_i2c_fifo_word_count(&ui16FillWords);
    if (ui32Status != AM_HAL_STATUS_SUCCESS) {
        am_util_stdio_printf("ERROR: lsm6dsl_i2c_fifo_word_count() (0x%08lX)\r\n",
                             (unsigned long)ui32Status);
        led2_on();
        return 0;
    }

    if (ui16FillWords >= FIFO_WARN_WORDS) {
        g_ui32FifoWarnings++;
    }

    uint8_t ui8Status2 = 0;
    ui32Status = lsm6dsl_i2c_read_reg(LSM6DSL_REG_FIFO_STATUS2, &ui8Status2);
    if (ui32Status == AM_HAL_STATUS_SUCCESS &&
        (ui8Status2 & LSM6DSL_FIFO_OVER_RUN)) {
        g_ui32FifoOverruns++;
        am_util_stdio_printf("FIFO OVERRUN - resetting.\r\n");
        lsm6dsl_i2c_fifo_reset();
        resync_epoch(*pui32SeqNext);
        return 0;
    }

    uint16_t ui16Sets = ui16FillWords / 6u;
    if (ui16Sets > FIFO_READ_BATCH) {
        ui16Sets = FIFO_READ_BATCH;
    }
    if (ui16Sets > ui32MaxOut) {
        ui16Sets = (uint16_t)ui32MaxOut;
    }

    uint32_t ui32Grabbed = 0;
    for (uint16_t i = 0; i < ui16Sets; i++) {
        lsm6dsl_i2c_fifo_set_t sSet;
        ui32Status = lsm6dsl_i2c_fifo_read_set(&sSet);
        if (ui32Status != AM_HAL_STATUS_SUCCESS) {
            am_util_stdio_printf("ERROR: lsm6dsl_i2c_fifo_read_set() (0x%08lX)\r\n",
                                 (unsigned long)ui32Status);
            led2_on();
            break;
        }

        imu_sample_t *psSample = &psOut[ui32Grabbed];
        psSample->time_ms = (*pui32SeqNext) * (1000.0f / SAMPLE_RATE);
        psSample->acc_x   = (float)sSet.sAccel.i16X / g_fAccSensLsbPerG;
        psSample->acc_y   = (float)sSet.sAccel.i16Y / g_fAccSensLsbPerG;
        psSample->acc_z   = (float)sSet.sAccel.i16Z / g_fAccSensLsbPerG;
        psSample->gyro_x  = (float)sSet.sGyro.i16X / g_fGyrSensLsbPerDps;
        psSample->gyro_y  = (float)sSet.sGyro.i16Y / g_fGyrSensLsbPerDps;
        psSample->gyro_z  = (float)sSet.sGyro.i16Z / g_fGyrSensLsbPerDps;
        psSample->gyro_mag = 0.0f;
        psSample->phase    = 0;

        uint32_t ui32SampleCs = g_ui32EpochCs +
            (uint32_t)((*pui32SeqNext) * (100.0f / FIFO_ODR_HZ) + 0.5f);
        rtc_stamp_t sStamp;
        centiseconds_to_hms(ui32SampleCs, &sStamp.ui8Hour, &sStamp.ui8Minute,
                             &sStamp.ui8Second, &sStamp.ui8Hundredths);
        if (psStamps != NULL) {
            psStamps[ui32Grabbed] = sStamp;
        }
        g_sLastSampleStamp = sStamp;

        (*pui32SeqNext)++;
        ui32Grabbed++;
    }

    return ui32Grabbed;
#endif
}

// ============================================================================
// SD-CARD CSV LOGGING
//
// Files written per session (auto-numbered with session index):
//   0:shoe_data/IMUxxx.CSV        - raw IMU samples (one row per sample)
//   0:shoe_data/GAITxxx.CSV       - one row per stride (all gait parameters)
//   0:shoe_data/MINUTExxx.CSV     - one row per 1-minute window (aggregate stats)
//   0:shoe_data/SESSIONxxx.CSV    - one final row with the whole-session report
// ============================================================================
static FATFS g_sFatFs;
static FIL   g_sImuCsvFile;
static FIL   g_sGaitCsvFile;
static FIL   g_sMinuteCsvFile;
static FIL   g_sSessionCsvFile;
static FIL   g_sScoreCsvFile;//gait_health
static char  g_acScoreFilename[64];
static bool  g_bScoreCsvOpen = false; //
static bool  g_bImuCsvOpen    = false;
static bool  g_bGaitCsvOpen   = false;
static bool  g_bMinuteCsvOpen = false;
static bool  g_bSessionCsvOpen = false;

#define SHOE_DATA_DIR   "0:shoe_data"

static void csv_write_line(FIL *pFile, bool *pbOpenFlag, const char *pcLine)
{
    if (!*pbOpenFlag) {
        return;
    }
    UINT ui32Len     = (UINT)strlen(pcLine);
    UINT ui32Written = 0;
    FRESULT rc = f_write(pFile, pcLine, ui32Len, &ui32Written);
    if (rc != FR_OK || ui32Written != ui32Len) {
        am_util_stdio_printf(
            "[CSV] write failed (FRESULT %d) - logging to this file stopped\r\n", rc);
        led2_on();
        *pbOpenFlag = false;
    }
}

static char g_acImuFilename[40];
static char g_acGaitFilename[40];
static char g_acMinuteFilename[40];
static char g_acSessionFilename[40];

static uint32_t csv_next_session_number(void)
{
    char acProbe[40];
    for (uint32_t n = 1; n <= 999; n++) {
        snprintf(acProbe, sizeof(acProbe), SHOE_DATA_DIR "/IMU%03lu.CSV", (unsigned long)n);
        FILINFO fno;
        FRESULT rc = f_stat(acProbe, &fno);
        if (rc == FR_NO_FILE) {
            return n;
        }
    }
    return 0;
}

static void csv_logging_init(void)
{
    FRESULT rc = f_mount(&g_sFatFs, "0:", 1);
    if (rc != FR_OK) {
        am_util_stdio_printf(
            "[CSV] f_mount failed (FRESULT %d) - SD CSV logging disabled\r\n", rc);
        led2_on();
        return;
    }

    rc = f_mkdir(SHOE_DATA_DIR);
    if (rc != FR_OK && rc != FR_EXIST) {
        am_util_stdio_printf(
            "[CSV] f_mkdir(%s) failed (FRESULT %d) - SD CSV logging disabled\r\n",
            SHOE_DATA_DIR, rc);
        led2_on();
        return;
    }

    uint32_t ui32Seq = csv_next_session_number();
    if (ui32Seq == 0) {
        am_util_stdio_printf("[CSV] SD card full (>999 sessions) - logging disabled\r\n");
        return;
    }

    snprintf(g_acImuFilename,    sizeof(g_acImuFilename),    SHOE_DATA_DIR "/IMU%03lu.CSV",    (unsigned long)ui32Seq);
    snprintf(g_acGaitFilename,   sizeof(g_acGaitFilename),   SHOE_DATA_DIR "/GAIT%03lu.CSV",   (unsigned long)ui32Seq);
    snprintf(g_acMinuteFilename, sizeof(g_acMinuteFilename), SHOE_DATA_DIR "/MINUTE%03lu.CSV", (unsigned long)ui32Seq);
    snprintf(g_acSessionFilename,sizeof(g_acSessionFilename),SHOE_DATA_DIR "/SESSION%03lu.CSV",(unsigned long)ui32Seq);
  snprintf(g_acScoreFilename, sizeof(g_acScoreFilename), SHOE_DATA_DIR "/SCORE%03lu.CSV", (unsigned long)ui32Seq);//gait_health_score
  
  am_util_stdio_printf("[CSV] Session %03lu -> %s | %s | %s | %s\r\n",
                         (unsigned long)ui32Seq,
                         g_acImuFilename, g_acGaitFilename, g_acMinuteFilename,
                         g_acSessionFilename);

    // --- IMU raw CSV ---
    rc = f_open(&g_sImuCsvFile, g_acImuFilename, FA_CREATE_NEW | FA_WRITE);
    if (rc == FR_OK) {
        g_bImuCsvOpen = true;
        csv_write_line(&g_sImuCsvFile, &g_bImuCsvOpen,
                        "time_ms,acc_x_g,acc_y_g,acc_z_g,gyr_x,gyr_y,gyr_z,steps\r\n");
        am_util_stdio_printf("[CSV] Opened %s\r\n", g_acImuFilename + 2);
    } else {
        am_util_stdio_printf("[CSV] f_open(%s) failed (FRESULT %d)\r\n",
                             g_acImuFilename + 2, rc);
    }

    // --- Stride-level gait CSV (all strides, full 18 columns) ---
    rc = f_open(&g_sGaitCsvFile, g_acGaitFilename, FA_CREATE_NEW | FA_WRITE);
    if (rc == FR_OK) {
        g_bGaitCsvOpen = true;
        csv_write_line(&g_sGaitCsvFile, &g_bGaitCsvOpen,
                        "minute,stride_idx,td_time_ms,to_time_ms,td_next_ms,gct_ms,"
                        "stance_time_ms,stride_time_ms,swing_time_ms,stance_ratio,"
                        "cadence_spm,speed_m_s,stride_length_m,min_clearance_m,"
                        "max_clearance_m,vertical_osc_cm,ilr_g_per_ms,"
                        "pronation_angle_deg,angle_change\r\n");
        am_util_stdio_printf("[CSV] Opened %s\r\n", g_acGaitFilename + 2);
    } else {
        am_util_stdio_printf("[CSV] f_open(%s) failed (FRESULT %d)\r\n",
                             g_acGaitFilename + 2, rc);
    }

    // --- Per-minute aggregate CSV ---
    rc = f_open(&g_sMinuteCsvFile, g_acMinuteFilename, FA_CREATE_NEW | FA_WRITE);
    if (rc == FR_OK) {
        g_bMinuteCsvOpen = true;
        // Header: one row per 1-minute interval with mean gait parameters
        csv_write_line(&g_sMinuteCsvFile, &g_bMinuteCsvOpen,
                        "minute,elapsed_s,n_strides,"
                        "mean_gct_ms,mean_stance_ms,mean_stride_ms,mean_swing_ms,"
                        "mean_stance_ratio,mean_cadence_spm,mean_speed_m_s,"
                        "mean_stride_len_m,mean_min_clear_m,mean_max_clear_m,"
                        "mean_vert_osc_cm,mean_ilr_g_per_ms,mean_pronation_deg,"
                        "mean_angle_change,cv_cadence_pct,cv_stride_len_pct,"
                        "cv_gct_pct,stride_var_sd_ms\r\n");
        am_util_stdio_printf("[CSV] Opened %s\r\n", g_acMinuteFilename + 2);
    } else {
        am_util_stdio_printf("[CSV] f_open(%s) failed (FRESULT %d)\r\n",
                             g_acMinuteFilename + 2, rc);
    }

    // --- Final whole-session aggregate CSV (one row written at session end) ---
    rc = f_open(&g_sSessionCsvFile, g_acSessionFilename, FA_CREATE_NEW | FA_WRITE);
    if (rc == FR_OK) {
        g_bSessionCsvOpen = true;
        csv_write_line(&g_sSessionCsvFile, &g_bSessionCsvOpen,
                        "duration_s,total_samples,fifo_warnings,fifo_overruns,pipeline_time_s,"
                        "detected_strides,hardware_steps,hardware_strides,"
                        "mean_gct_ms,mean_stride_ms,mean_stance_ms,mean_swing_ms,"
                        "mean_stance_ratio,mean_cadence_spm,mean_speed_m_s,"
                        "mean_stride_len_m,mean_min_clear_m,mean_max_clear_m,"
                        "mean_vert_osc_cm,mean_ilr_g_per_ms,mean_pronation_deg,"
                        "mean_angle_change,mean_energy,"
                        "cv_gct_pct,cv_stride_pct,cv_stance_pct,cv_swing_pct,"
                        "cv_stance_ratio_pct,cv_cadence_pct,cv_speed_pct,"
                        "cv_stride_len_pct,cv_min_clear_pct,cv_max_clear_pct,"
                        "cv_vert_osc_pct,cv_ilr_pct,cv_pronation_pct,cv_angle_change_pct,"
                        "cv_energy_pct,stride_var_sd_ms,stride_var_cv_pct\r\n");
        am_util_stdio_printf("[CSV] Opened %s\r\n", g_acSessionFilename + 2);
    } else {
        am_util_stdio_printf("[CSV] f_open(%s) failed (FRESULT %d)\r\n",
                             g_acSessionFilename + 2, rc);
    }

        rc = f_open(&g_sScoreCsvFile, g_acScoreFilename, FA_CREATE_NEW | FA_WRITE);//gait_health_score
    if (rc == FR_OK) {
        g_bScoreCsvOpen = true;
        csv_write_line(&g_sScoreCsvFile, &g_bScoreCsvOpen,
            "health_score,band,"
            "cadence,impact_loading_rate,pronation_angle,gct_consistency,"
            "cadence_std_ms,impact_BW_per_s,gct_cv,gct_mean_ms,"
            "pronation_mean_dev_deg,pronation_alert\r\n");
        am_util_stdio_printf("[CSV] Opened %s\r\n", g_acScoreFilename + 2);
    } else {
        am_util_stdio_printf("[CSV] f_open(%s) failed (FRESULT %d)\r\n",
                             g_acScoreFilename + 2, rc);
    }

} //end

// Log one raw IMU sample
static void csv_log_imu_sample(const imu_sample_t *psSample,
                                const rtc_stamp_t  *psStamp,
                                uint16_t            ui16Steps)
{
    if (!g_bImuCsvOpen) {
        return;
    }
    uint32_t ui32PackedTime = (uint32_t)psStamp->ui8Hour       * 1000000u
                            + (uint32_t)psStamp->ui8Minute     * 10000u
                            + (uint32_t)psStamp->ui8Second     * 100u
                            + (uint32_t)psStamp->ui8Hundredths;

    char acLine[128];
    int n = snprintf(acLine, sizeof(acLine),
                      "%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%u\r\n",
                      (unsigned long)ui32PackedTime,
                      psSample->acc_x,  psSample->acc_y,  psSample->acc_z,
                      psSample->gyro_x, psSample->gyro_y, psSample->gyro_z,
                      (unsigned int)ui16Steps);
    if (n > 0) {
        csv_write_line(&g_sImuCsvFile, &g_bImuCsvOpen, acLine);
    }
}

// Log one stride's gait parameters to GAIT CSV.
// 'minute' is the 1-based minute number this stride belongs to.
static void csv_log_gait_stride(uint32_t ui32Minute,
                                 const stride_t *psStride,
                                 const temporal_params_t *psTemp,
                                 const spatial_params_t *psSpat)
{
    if (!g_bGaitCsvOpen) {
        return;
    }

    volatile const uint32_t *p_stride    = (volatile const uint32_t *)&psSpat->stride_length_m;
    volatile const uint32_t *p_min_clear = (volatile const uint32_t *)&psSpat->min_clearance_m;
    volatile const uint32_t *p_max_clear = (volatile const uint32_t *)&psSpat->max_clearance_m;
    volatile const uint32_t *p_vert_osc  = (volatile const uint32_t *)&psSpat->vertical_osc_cm;
    volatile const uint32_t *p_pronation = (volatile const uint32_t *)&psSpat->pronation_angle_deg;
    volatile const uint32_t *p_angle     = (volatile const uint32_t *)&psSpat->angle_change;

    bool valid_stride    = (*p_stride & 0x7F800000)    != 0x7F800000 && psSpat->stride_length_m    != 0.0f;
    bool valid_min_clear = (*p_min_clear & 0x7F800000) != 0x7F800000 && psSpat->min_clearance_m    != 0.0f;
    bool valid_max_clear = (*p_max_clear & 0x7F800000) != 0x7F800000 && psSpat->max_clearance_m    != 0.0f;
    bool valid_vert_osc  = (*p_vert_osc & 0x7F800000)  != 0x7F800000 && psSpat->vertical_osc_cm    != 0.0f;
    bool valid_pronation = (*p_pronation & 0x7F800000) != 0x7F800000 && psSpat->pronation_angle_deg != 0.0f;
    bool valid_angle     = (*p_angle & 0x7F800000)     != 0x7F800000 && psSpat->angle_change        != 0.0f;

    char acLine[280];
    int n = snprintf(acLine, sizeof(acLine),
                      "%u,%u,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f,%.4f,%.2f,%.4f,%.4f,%.4f,%.4f,%.2f,%.4f,%.4f,%.4f\r\n",
                      (unsigned)ui32Minute,
                      (unsigned)psStride->stride_idx,
                      psStride->td_time_ms, psStride->to_time_ms, psStride->td_next_ms,
                      psTemp->gct_ms, psTemp->stance_time_ms, psTemp->stride_time_ms,
                      psTemp->swing_time_ms, psTemp->stance_ratio,
                      psTemp->cadence_spm, psTemp->speed_m_s,
                      valid_stride    ? psSpat->stride_length_m     : 0.0f,
                      valid_min_clear ? psSpat->min_clearance_m     : 0.0f,
                      valid_max_clear ? psSpat->max_clearance_m     : 0.0f,
                      valid_vert_osc  ? psSpat->vertical_osc_cm     : 0.0f,
                      psSpat->ilr_g_per_ms,
                      valid_pronation ? psSpat->pronation_angle_deg : 0.0f,
                      valid_angle     ? psSpat->angle_change        : 0.0f);
    if (n > 0) {
        csv_write_line(&g_sGaitCsvFile, &g_bGaitCsvOpen, acLine);
    }
}

// Log one per-minute aggregate row to MINUTE CSV.
// Called at the end of each 60-second window.
static void csv_log_minute_stats(uint32_t ui32Minute,
                                  uint32_t ui32ElapsedS,
                                  const session_stats_t *pStats)
{
    if (!g_bMinuteCsvOpen) {
        return;
    }
    if (pStats->n_strides == 0) {
        // No strides this minute - write a placeholder row
        char acLine[128];
        snprintf(acLine, sizeof(acLine), "%u,%u,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\r\n",
                 (unsigned)ui32Minute, (unsigned)ui32ElapsedS);
        csv_write_line(&g_sMinuteCsvFile, &g_bMinuteCsvOpen, acLine);
        return;
    }

    char acLine[320];
    int n = snprintf(acLine, sizeof(acLine),
                      "%u,%u,%u,"
                      "%.2f,%.2f,%.2f,%.2f,"
                      "%.4f,%.2f,%.4f,"
                      "%.4f,%.4f,%.4f,"
                      "%.2f,%.4f,%.4f,"
                      "%.4f,%.2f,%.2f,"
                      "%.2f,%.4f\r\n",
                      (unsigned)ui32Minute,
                      (unsigned)ui32ElapsedS,
                      (unsigned)pStats->n_strides,
                      pStats->mean_gct_ms,
                      pStats->mean_stance_time_ms,
                      pStats->mean_stride_time_ms,
                      pStats->mean_swing_time_ms,
                      pStats->mean_stance_ratio,
                      pStats->mean_cadence_spm,
                      pStats->mean_speed_m_s,
                      pStats->mean_stride_length_m,
                      pStats->mean_min_clearance_m,
                      pStats->mean_max_clearance_m,
                      pStats->mean_vertical_osc_cm,
                      pStats->mean_ilr_g_per_ms,
                      pStats->mean_pronation_angle_deg,
                      pStats->mean_angle_change,
                      pStats->cv_cadence_spm,
                      pStats->cv_stride_length_m,
                      pStats->cv_gct_ms,
                      pStats->stride_variability_sd_ms);
    if (n > 0) {
        csv_write_line(&g_sMinuteCsvFile, &g_bMinuteCsvOpen, acLine);
        f_sync(&g_sMinuteCsvFile);
    }
}

// Write the final aggregate only after aggregate_session_ex() has calculated
// exactly the same values printed by the full session report.
static void csv_log_session_stats(float fDurationS, uint32_t ui32TotalSamples,
                                  float fPipelineTimeS,
                                  const session_stats_t *pStats)
{
    if (!g_bSessionCsvOpen) {
        return;
    }

    char acLine[640];
    int n = snprintf(acLine, sizeof(acLine),
        "%.3f,%lu,%lu,%lu,%.3f,%lu,%lu,%lu,"
        "%.4f,%.4f,%.4f,%.4f,%.6f,%.4f,%.6f,"
        "%.6f,%.6f,%.6f,%.4f,%.6f,%.6f,%.6f,%.6f,"
        "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
        "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
        "%.4f,%.4f,%.4f\r\n",
        fDurationS, (unsigned long)ui32TotalSamples,
        (unsigned long)g_ui32FifoWarnings, (unsigned long)g_ui32FifoOverruns,
        fPipelineTimeS, (unsigned long)pStats->n_strides,
        (unsigned long)pStats->hw_step_count, (unsigned long)pStats->hw_stride_count,
        pStats->mean_gct_ms, pStats->mean_stride_time_ms,
        pStats->mean_stance_time_ms, pStats->mean_swing_time_ms,
        pStats->mean_stance_ratio, pStats->mean_cadence_spm, pStats->mean_speed_m_s,
        pStats->mean_stride_length_m, pStats->mean_min_clearance_m,
        pStats->mean_max_clearance_m, pStats->mean_vertical_osc_cm,
        pStats->mean_ilr_g_per_ms, pStats->mean_pronation_angle_deg,
        pStats->mean_angle_change, pStats->mean_energy,
        pStats->cv_gct_ms, pStats->cv_stride_time_ms, pStats->cv_stance_time_ms,
        pStats->cv_swing_time_ms, pStats->cv_stance_ratio, pStats->cv_cadence_spm,
        pStats->cv_speed_m_s, pStats->cv_stride_length_m,
        pStats->cv_min_clearance_m, pStats->cv_max_clearance_m,
        pStats->cv_vertical_osc_cm, pStats->cv_ilr_g_per_ms,
        pStats->cv_pronation_angle_deg, pStats->cv_angle_change,
        pStats->cv_energy, pStats->stride_variability_sd_ms,
        pStats->stride_variability_cv_pct);
    if (n > 0 && n < (int)sizeof(acLine)) {
        csv_write_line(&g_sSessionCsvFile, &g_bSessionCsvOpen, acLine);
        f_sync(&g_sSessionCsvFile);
    }
}

static void csv_log_health_score(const gait_score_result_t *pScore)  //csv_log_health_score
{
    char acLine[256];
    int  n;

    if (!g_bScoreCsvOpen || !pScore)
        return;

    /* health_score and each sub-score can be NaN when a metric was missing.
     * We write "nan" for those fields so the CSV is machine-parseable. */

#define FMT_F(v)  (isfinite(v) ? (v) : (float)NAN)

    /* Columns:
     *   health_score, band,
     *   cadence (sub), impact_loading_rate (sub), pronation_angle (sub),
     *   gct_consistency (sub),
     *   cadence_std_ms, impact_BW_per_s, gct_cv, gct_mean_ms,
     *   pronation_mean_dev_deg, pronation_alert
     */
    n = snprintf(acLine, sizeof(acLine),
        "%.1f,%s,"                  /* health_score, band              */
        "%.1f,%.1f,%.1f,%.1f,"     /* cadence,impact,pronation,gct    */
        "%.2f,%.2f,%.4f,%.1f,"     /* cadence_std_ms, impact_BW_per_s,
                                       gct_cv, gct_mean_ms             */
        "%.2f,%s\r\n",              /* pronation_mean_dev_deg, alert   */
        isfinite(pScore->health_score)           ? pScore->health_score           : NAN,
        pScore->band,
        isfinite(pScore->sub_cadence_regularity) ? pScore->sub_cadence_regularity : NAN,
        isfinite(pScore->sub_impact_loading)     ? pScore->sub_impact_loading     : NAN,
        isfinite(pScore->sub_pronation)          ? pScore->sub_pronation          : NAN,
        isfinite(pScore->sub_gct_consistency)    ? pScore->sub_gct_consistency    : NAN,
        isfinite(pScore->cadence_std_interval_ms)? pScore->cadence_std_interval_ms: NAN,
        isfinite(pScore->impact_BW_per_s)        ? pScore->impact_BW_per_s        : NAN,
        isfinite(pScore->gct_cv)                 ? pScore->gct_cv                 : NAN,
        isfinite(pScore->gct_mean_ms)            ? pScore->gct_mean_ms            : NAN,
        isfinite(pScore->pronation_mean_dev_deg) ? pScore->pronation_mean_dev_deg : NAN,
        pScore->pronation_alert ? "1" : "0");

#undef FMT_F

    if (n > 0 && n < (int)sizeof(acLine)) {
        csv_write_line(&g_sScoreCsvFile, &g_bScoreCsvOpen, acLine);
        f_sync(&g_sScoreCsvFile);
    }
}

static void csv_logging_close(void)
{
    if (g_bImuCsvOpen) {
        f_close(&g_sImuCsvFile);
        g_bImuCsvOpen = false;
        am_util_stdio_printf("[CSV] Saved %s\r\n", g_acImuFilename + 2);
    }
    if (g_bGaitCsvOpen) {
        f_close(&g_sGaitCsvFile);
        g_bGaitCsvOpen = false;
        am_util_stdio_printf("[CSV] Saved %s\r\n", g_acGaitFilename + 2);
    }
    if (g_bMinuteCsvOpen) {
        f_close(&g_sMinuteCsvFile);
        g_bMinuteCsvOpen = false;
        am_util_stdio_printf("[CSV] Saved %s\r\n", g_acMinuteFilename + 2);
    }
    if (g_bSessionCsvOpen) {
        f_close(&g_sSessionCsvFile);
        g_bSessionCsvOpen = false;
        am_util_stdio_printf("[CSV] Saved %s\r\n", g_acSessionFilename + 2);
    }

        if (g_bScoreCsvOpen) { //Gait_health_score
        f_close(&g_sScoreCsvFile);
        g_bScoreCsvOpen = false;
        am_util_stdio_printf("[CSV] Saved %s\r\n", g_acScoreFilename + 2);
    }
}

#if SIM_CSV_INPUT
/*
 * Offline simulation path used to establish true Python parity.
 *
 * Python processes each walking bout as one complete unit:
 *   detect events -> temporal strides -> one ESKF -> spatial metrics.
 * The old C simulation path instead processed overlapping 512-sample windows,
 * which necessarily changes event boundaries and the ESKF trajectory.  This
 * helper intentionally follows the Python ordering and is used only when
 * SIM_CSV_INPUT=1.  Live hardware keeps the streaming path below.
 */
static bool run_sim_python_parity_pipeline(uint32_t *out_fv_count,
                                           uint32_t *out_stride_count)
{
    static SHARED_SRAM imu_sample_t sim_all_samples[SIM_IMU_DATA_COUNT];
    static imu_sample_t *bout_ptrs[MAX_BOUTS];   // pointer array -- tiny, stays in TCM
    static uint32_t bout_len[MAX_BOUTS];           // tiny, stays in TCM
    static uint32_t peak_limits[MAX_BOUTS];        // tiny, stays in TCM
    static SHARED_SRAM gait_event_t sim_events[MAX_EVENTS];
    static SHARED_SRAM stride_t sim_strides[MAX_STRIDES];
    static SHARED_SRAM traj_sample_t sim_traj[MAX_BOUT_SAMPLES];

    uint32_t n = SIM_IMU_DATA_COUNT;
    if (n < 3u) return false;

    /* Convert the embedded replay rows to the same elapsed-ms representation
       used by Python load_raw_samples(). */
    float t0 = (float)g_sSimImuData[0].time_ms;
    for (uint32_t i = 0; i < n; i++) {
        const sim_imu_row_t *r = &g_sSimImuData[i];
        sim_all_samples[i].time_ms = (float)r->time_ms - t0;
        sim_all_samples[i].acc_x = r->acc_x;
        sim_all_samples[i].acc_y = r->acc_y;
        sim_all_samples[i].acc_z = r->acc_z;
        sim_all_samples[i].gyro_x = r->gyro_x;
        sim_all_samples[i].gyro_y = r->gyro_y;
        sim_all_samples[i].gyro_z = r->gyro_z;
        sim_all_samples[i].gyro_mag = 0.0f;
        sim_all_samples[i].phase = 0u;
    }
    step2_gyro_magnitude(sim_all_samples, n);

    /* Python segment_bouts(): split only when timestamp gap > 2000 ms and
       retain bouts with >100 samples. */
    uint32_t n_bouts = 0u;
    uint32_t start = 0u;
    for (uint32_t i = 1u; i <= n; i++) {
        bool gap = (i < n) &&
                   ((sim_all_samples[i].time_ms - sim_all_samples[i-1u].time_ms) > BOUT_GAP_MS);
        if (gap || i == n) {
            uint32_t len = i - start;
            if (len > MIN_BOUT_SAMPLES && n_bouts < MAX_BOUTS) {
                if (len > MAX_BOUT_SAMPLES) {
                    am_util_stdio_printf("[PY parity] ERROR: bout %u has %u samples; MAX_BOUT_SAMPLES=%u\r\n",
                                         (unsigned)n_bouts, (unsigned)len,
                                         (unsigned)MAX_BOUT_SAMPLES);
                    return false;
                }
                bout_ptrs[n_bouts] = &sim_all_samples[start];
                bout_len[n_bouts] = len;
                n_bouts++;
            }
            start = i;
        }
    }
    if (n_bouts == 0u) return false;

    uint32_t hw_steps = sim_hw_step_total();
    uint32_t target_strides = hw_steps / 2u;
    float peak_distance_s = 0.75f;
    if (hw_steps >= 4u) {
        peak_distance_s = step3_hardware_assisted_peak_profile(
            bout_ptrs, bout_len, n_bouts, target_strides, peak_limits);
    } else {
        for (uint32_t b = 0; b < n_bouts; b++) peak_limits[b] = 0u;
    }

    am_util_stdio_printf("\r\n[PY parity] Bouts=%u | HW steps=%u -> target strides=%u | peak distance=%.3fs\r\n",
                         (unsigned)n_bouts, (unsigned)hw_steps,
                         (unsigned)target_strides, peak_distance_s);

    uint32_t fv_count = 0u;
    uint32_t stride_count = 0u;

    for (uint32_t b = 0u; b < n_bouts; b++) {
        imu_sample_t *bout = bout_ptrs[b];
        uint32_t bn = bout_len[b];
        float bt0 = bout[0].time_ms;
        for (uint32_t i = 0; i < bn; i++) bout[i].time_ms -= bt0;

        uint32_t max_peaks = (hw_steps >= 4u) ? peak_limits[b] : 0u;
        uint32_t n_events = step3_detect_events_profile(
            bout, bn, sim_events, MAX_EVENTS, true, peak_distance_s, max_peaks);
        uint32_t n_strides = step3_build_strides(
            sim_events, n_events, sim_strides, MAX_STRIDES);

        am_util_stdio_printf("[PY parity] Bout %u: events=%u strides=%u\r\n",
                             (unsigned)b, (unsigned)n_events, (unsigned)n_strides);

        if (n_strides < 2u) continue;

        step5_eskf_reset_session();
        step5_eskf_trajectory_bout(bout, bn, sim_traj, GRAVITY);

        for (uint32_t i = 0; i < n_strides; i++) {
            if (fv_count >= MAX_SESSION_STRIDES) break;

            temporal_params_t *tp = &temp_params[fv_count];
            spatial_params_t *sp = &spat_params[fv_count];

            step4_temporal(&sim_strides[i], tp);
            step6_spatial(bout, bn, &sim_strides[i], sim_traj, sp);
            /* This call intentionally overwrites the ESKF-Z clearance values,
               matching Python compute_clearance_from_acc(). */
            step6_clearance_from_gyro(bout, bn, &sim_strides[i], sp);
            step6_ilr(bout, bn, &sim_strides[i], sp);
            step6_pronation(bout, bn, &sim_strides[i], sp);
            step6_angle_change(sim_traj, bn, &sim_strides[i], sp);
            step4_temporal_with_speed(tp, sp);

            session_strides[fv_count] = sim_strides[i];
            session_strides[fv_count].stride_idx = fv_count;
            extract_feature_vector(tp, sp, &feature_vectors[fv_count]);

            csv_log_gait_stride(1u, &session_strides[fv_count], tp, sp);
            fv_count++;
            stride_count++;
        }
    }

    *out_fv_count = fv_count;
    *out_stride_count = stride_count;
    am_util_stdio_printf("[PY parity] Total strides=%u\r\n", (unsigned)stride_count);
    return true;
}
#endif


#if !SIM_CSV_INPUT
static SHARED_SRAM imu_sample_t live_session_samples[LSM_SESSION_SECONDS * (uint32_t)SAMPLE_RATE];
static uint32_t live_session_count = 0u;

static uint32_t round_half_steps_python(uint32_t steps)
{
    uint32_t q = steps / 2u;
    if ((steps & 1u) == 0u) return q;
    /* Python round(n + 0.5) is ties-to-even. */
    return (q & 1u) ? (q + 1u) : q;
}

static bool run_live_python_parity_pipeline(uint32_t hw_steps,
                                            uint32_t *out_fv_count,
                                            uint32_t *out_stride_count)
{
    static imu_sample_t *bout_ptrs[MAX_BOUTS];
    static uint32_t bout_len[MAX_BOUTS];
    static uint32_t peak_limits[MAX_BOUTS];
    static SHARED_SRAM gait_event_t live_events[MAX_EVENTS];
    static SHARED_SRAM stride_t live_strides[MAX_STRIDES];
    static SHARED_SRAM traj_sample_t live_traj[MAX_BOUT_SAMPLES];

    uint32_t n = live_session_count;
    if (n < 3u) return false;
    step2_gyro_magnitude(live_session_samples, n);

    /* Same concept as Python segment_bouts().  A long uninterrupted live
       recording is additionally split at MAX_BOUT_SAMPLES only because the
       embedded ESKF has a finite bout buffer. */
    uint32_t n_bouts = 0u, start = 0u;
    for (uint32_t i = 1u; i <= n; i++) {
        bool time_gap = (i < n) &&
            ((live_session_samples[i].time_ms - live_session_samples[i-1u].time_ms) > BOUT_GAP_MS);
        bool memory_split = (i < n) && ((i - start) >= MAX_BOUT_SAMPLES);
        if (time_gap || memory_split || i == n) {
            uint32_t len = i - start;
            if (len > MIN_BOUT_SAMPLES && n_bouts < MAX_BOUTS) {
                bout_ptrs[n_bouts] = &live_session_samples[start];
                bout_len[n_bouts] = len;
                n_bouts++;
            }
            start = i;
        }
    }
    if (n_bouts == 0u) return false;

    uint32_t target_strides = round_half_steps_python(hw_steps);
    float peak_distance_s = 0.75f;
    if (hw_steps >= 4u) {
        peak_distance_s = step3_hardware_assisted_peak_profile(
            bout_ptrs, bout_len, n_bouts, target_strides, peak_limits);
    } else {
        for (uint32_t b=0; b<n_bouts; b++) peak_limits[b]=0u;
    }

    am_util_stdio_printf("\r\n[LIVE parity] samples=%u bouts=%u HW steps=%u target strides=%u distance=%.3fs\r\n",
        (unsigned)n, (unsigned)n_bouts, (unsigned)hw_steps,
        (unsigned)target_strides, peak_distance_s);

    uint32_t fv = 0u;
    for (uint32_t b=0; b<n_bouts; b++) {
        imu_sample_t *bout=bout_ptrs[b]; uint32_t bn=bout_len[b];
        float t0=bout[0].time_ms;
        for (uint32_t i=0;i<bn;i++) bout[i].time_ms -= t0;
        uint32_t n_events=step3_detect_events_profile(
            bout,bn,live_events,MAX_EVENTS,true,peak_distance_s,
            (hw_steps>=4u)?peak_limits[b]:0u);
        uint32_t ns=step3_build_strides(live_events,n_events,live_strides,MAX_STRIDES);
        am_util_stdio_printf("[LIVE parity] Bout %u peaks=%u events=%u strides=%u\r\n",
            (unsigned)b,(unsigned)(n_events/2u),(unsigned)n_events,(unsigned)ns);
        if (ns < 1u) continue;
        step5_eskf_reset_session();
        step5_eskf_trajectory_bout(bout,bn,live_traj,GRAVITY);
        for(uint32_t i=0;i<ns && fv<MAX_SESSION_STRIDES;i++){
            step4_temporal(&live_strides[i],&temp_params[fv]);
            step6_spatial(bout,bn,&live_strides[i],live_traj,&spat_params[fv]);
            step6_clearance_from_gyro(bout,bn,&live_strides[i],&spat_params[fv]);
            step6_ilr(bout,bn,&live_strides[i],&spat_params[fv]);
            step6_pronation(bout,bn,&live_strides[i],&spat_params[fv]);
            step6_angle_change(live_traj,bn,&live_strides[i],&spat_params[fv]);
            step4_temporal_with_speed(&temp_params[fv],&spat_params[fv]);
            session_strides[fv]=live_strides[i]; session_strides[fv].stride_idx=fv;
            extract_feature_vector(&temp_params[fv],&spat_params[fv],&feature_vectors[fv]);
            csv_log_gait_stride(1u,&session_strides[fv],&temp_params[fv],&spat_params[fv]);
            fv++;
        }
    }
    *out_fv_count=fv; *out_stride_count=fv;
    am_util_stdio_printf("[LIVE parity] detected=%u target=%u difference=%d\r\n",
        (unsigned)fv,(unsigned)target_strides,(int)fv-(int)target_strides);
    return true;
}
#endif

int main(void) {
    am_bsp_low_power_init();
    am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_OTP);
    am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_CRYPTO);
    am_hal_cachectrl_icache_enable();
    am_hal_cachectrl_dcache_enable(true);
    MCUCTRL->DEBUGGER &= ~AM_HAL_DCU_SWO;

    if (am_bsp_debug_printf_enable() != 0) { while(1); }

    UART_printf_init();
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("\r\n--- LSM6DSL IMU GAIT PIPELINE (3-MINUTE SESSION) ---\r\n");
    am_util_stdio_printf("Session: %u min | Minute reports: every %u s\r\n",
                         (unsigned)LSM_SESSION_MINUTES, (unsigned)LSM_MINUTE_INTERVAL_S);

    rtc_init();
    led2_init();
    led1_init();

    csv_logging_init();

    // Zero-init SHARED_SRAM buffers: the .shared section is NOLOAD so the
    // startup CRT does NOT clear them. Random content causes ESKF divergence.
    memset(samples,         0, sizeof(samples));
    memset(cb_buffer,       0, sizeof(cb_buffer));
    memset(events,          0, sizeof(events));
    memset(strides,         0, sizeof(strides));
    memset(temp_params,     0, sizeof(temp_params));
    memset(spat_params,     0, sizeof(spat_params));
    memset(feature_vectors, 0, sizeof(feature_vectors));
    memset(session_strides, 0, sizeof(session_strides));
    memset(trajectory,      0, sizeof(trajectory));

#if SIM_CSV_INPUT
    am_util_stdio_printf("[BUILD] SIM_STATIC_ONCE_v2 | rows=%u\r\n",
                          (unsigned)SIM_IMU_DATA_COUNT);
    am_util_stdio_printf("\r\n*** SIMULATION MODE: replaying IMU0032.CSV (%u samples, "
                          "%.1f s), no LSM6DSL hardware access ***\r\n",
                          (unsigned)SIM_IMU_DATA_COUNT, SIM_IMU_DATA_DURATION_MS / 1000.0f);
#else
    sLsm6dslCfg.ui32IOMModule = 1;
    sLsm6dslCfg.ui32I2CSpeed  = AM_HAL_IOM_400KHZ;
    sLsm6dslCfg.ui8DevAddr    = LSM6DSL_I2C_ADDR_DEFAULT;
    sLsm6dslCfg.ui8XLOdr      = LSM_XL_ODR;
    sLsm6dslCfg.ui8XLFs       = LSM_XL_FS_REG;
    sLsm6dslCfg.ui8GOdr       = LSM_G_ODR;
    sLsm6dslCfg.ui8GFs        = LSM_G_FS_REG;
    sLsm6dslCfg.eMode         = LSM6DSL_MODE_FIFO;
    sLsm6dslCfg.ui16FifoWatermark = 0;
    sLsm6dslCfg.ui8FifoOdr        = LSM_FIFO_ODR;

    uint32_t lsm6dsl_status = lsm6dsl_i2c_init(&sLsm6dslCfg);
    if (lsm6dsl_status != AM_HAL_STATUS_SUCCESS) {
        am_util_stdio_printf("LSM6DSL initialization failed: %d\r\n", lsm6dsl_status);
        while(1);
    }
    am_util_stdio_printf("LSM6DSL initialized successfully\r\n");

    uint8_t chip_id;
    lsm6dsl_status = lsm6dsl_i2c_who_am_i(&chip_id);
    if (lsm6dsl_status == AM_HAL_STATUS_SUCCESS) {
        am_util_stdio_printf("LSM6DSL WHO_AM_I: 0x%02X\r\n", chip_id);
    }

    uint8_t ctrl1_xl, ctrl2_g, status_reg;
    lsm6dsl_status = lsm6dsl_i2c_read_reg(LSM6DSL_REG_CTRL1_XL, &ctrl1_xl);
    if (lsm6dsl_status == AM_HAL_STATUS_SUCCESS)
        am_util_stdio_printf("CTRL1_XL: 0x%02X\r\n", ctrl1_xl);
    lsm6dsl_status = lsm6dsl_i2c_read_reg(LSM6DSL_REG_CTRL2_G, &ctrl2_g);
    if (lsm6dsl_status == AM_HAL_STATUS_SUCCESS)
        am_util_stdio_printf("CTRL2_G: 0x%02X\r\n", ctrl2_g);
    lsm6dsl_status = lsm6dsl_i2c_read_reg(LSM6DSL_REG_STATUS_REG, &status_reg);
    if (lsm6dsl_status == AM_HAL_STATUS_SUCCESS)
        am_util_stdio_printf("STATUS_REG: 0x%02X\r\n", status_reg);

    am_util_stdio_printf("Waiting for sensor to stabilize...\r\n");
    am_util_delay_ms(100);

    lsm6dsl_status = lsm6dsl_i2c_read_accel_raw(&sAccelRaw);
    am_util_stdio_printf("Raw Accel: X=0x%04X, Y=0x%04X, Z=0x%04X\r\n",
                        (uint16_t)sAccelRaw.i16X, (uint16_t)sAccelRaw.i16Y, (uint16_t)sAccelRaw.i16Z);
#endif // SIM_CSV_INPUT

    am_util_stdio_printf("\r\n--- RAW IMU DATA (FIFO) ---\r\n");
    am_util_stdio_printf(" time_ms    Acc_X     Acc_Y    Acc_Z   Gyro_X   Gyro_Y   Gyro_Z\r\n");

    init_sqrt_lut();

    profile_start();
    uint32_t pipeline_start_cycles = get_cycles();

    cb_init(&cb, cb_buffer, MAX_SAMPLES);

    uint32_t total_samples = 0;
    uint32_t stride_count = 0;
    uint32_t fv_count = 0;

    // ── 3-minute session (180 s) ──────────────────────────────────────────
    // The regenerated SIM data contains the full raw recording (179.99 s).
    // Process each embedded sample once; do not replay it to fill the session.
#if SIM_CSV_INPUT
#define DEMO_TOTAL_SAMPLES   SIM_IMU_DATA_COUNT
#else
#define DEMO_TOTAL_SAMPLES   (LSM_SESSION_SECONDS * (uint32_t)SAMPLE_RATE)
#endif


    // ── Per-minute accounting ─────────────────────────────────────────────
    uint32_t minute_number      = 1;
    uint32_t minute_fv_start    = 0;
    uint32_t minute_sample_start= 0;
    const uint32_t samples_per_minute = LSM_MINUTE_INTERVAL_S * (uint32_t)SAMPLE_RATE;

#if SIM_CSV_INPUT
    // ────────────────────────────────────────────────────────────────────
    // Capture the first timestamp before preview replay so every
    // simulation sample is normalized to the same origin as gait_params.py.
    // ────────────────────────────────────────────────────────────────────
    g_ui32SimReadIdx    = 0;
    g_fSimTimeOffsetMs  = 0.0f;
    g_fSimBaselineMs    = (float)g_sSimImuData[0].time_ms;
    am_util_stdio_printf("[SIM] Timestamp baseline t0 = %.1f ms "
                          "(from sim_imu_data)\r\n",
                          g_fSimBaselineMs);
#endif

    // Print preview (uses the baseline that was just captured, so preview
    // times start at 0)
    {
        imu_sample_t preview_samples[10];
        uint32_t preview_seq = 0;
        uint32_t preview_got = 0;

        while (preview_got < 10) {
            uint32_t n = grab_fifo_samples(&preview_samples[preview_got], NULL,
                                            10 - preview_got, &preview_seq);
            if (n == 0) {
                am_util_delay_ms(FIFO_POLL_MS);
                continue;
            }
            preview_got += n;
        }

        for (uint32_t i = 0; i < preview_got; i++) {
            am_util_stdio_printf(" %7.4f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f\r\n",
                                preview_samples[i].time_ms,
                                preview_samples[i].acc_x, preview_samples[i].acc_y, preview_samples[i].acc_z,
                                preview_samples[i].gyro_x, preview_samples[i].gyro_y, preview_samples[i].gyro_z);
        }
    }

#if !SIM_CSV_INPUT
    lsm6dsl_i2c_fifo_reset();
#else
    // Reset the sim read cursor after the preview so the real acquisition
    // loop starts at sample 0 again. The baseline captured above is still
    // correct (it's derived from row 0 of the const dataset, which hasn't
    // moved), so we don't need to recompute it.
    g_ui32SimReadIdx   = 0;
    g_fSimTimeOffsetMs = 0.0f;
    g_ui32SimHwSteps   = 0u;
    g_ui16HwStepCount  = 0u;
#endif

    resync_epoch(0);

#if !SIM_CSV_INPUT
    {
        uint32_t ui32PedoStatus = lsm6dsl_i2c_pedometer_enable();
        if (ui32PedoStatus != AM_HAL_STATUS_SUCCESS) {
            am_util_stdio_printf("WARNING: pedometer enable failed (0x%08lX)\r\n",
                                 (unsigned long)ui32PedoStatus);
        } else {
            ui32PedoStatus = lsm6dsl_i2c_pedometer_reset();
            if (ui32PedoStatus == AM_HAL_STATUS_SUCCESS)
                am_util_stdio_printf("Pedometer enabled and zeroed.\r\n");
        }
    }
#endif

    am_util_stdio_printf("\r\n");
    am_util_stdio_printf("=======================================================\r\n");
    am_util_stdio_printf("  PIPELINE  -  Left Foot  |  Duration: %u min\r\n",
                         (unsigned)LSM_SESSION_MINUTES);
    am_util_stdio_printf("=======================================================\r\n");
    am_util_stdio_printf("  Samples  : %d  (%.1f s @ %d Hz)\r\n",
                        DEMO_TOTAL_SAMPLES,
                        (float)DEMO_TOTAL_SAMPLES / (float)SAMPLE_RATE,
                        SAMPLE_RATE);

    am_util_stdio_printf("\r\n[Step 2]  Computing Gyro Magnitude...\r\n");
    am_util_stdio_printf("[ML] Feature vector buffer: %d strides x %d features = %d bytes\r\n",
                        MAX_SESSION_STRIDES, FEATURE_VECTOR_SIZE,
                        MAX_SESSION_STRIDES * (int)sizeof(feature_vector_t));

    static imu_sample_t grab_batch[FIFO_READ_BATCH];
    static rtc_stamp_t  grab_stamps[FIFO_READ_BATCH];
    uint32_t fifo_seq = 0;

    uint32_t samples_since_process = 0;
    float last_emitted_td_time_ms = -1.0e9f;
    bool first_window_processed = false;

    step5_eskf_reset_session();

#if SIM_CSV_INPUT
    step5_eskf_gravity_calib_begin();
    for (uint32_t i = 0; i < SIM_IMU_DATA_COUNT; i++) {
        const sim_imu_row_t *r = &g_sSimImuData[i];
        step5_eskf_gravity_calib_pass1_sample(r->acc_x, r->acc_y, r->acc_z,
                                               r->gyro_x, r->gyro_y, r->gyro_z);
    }
    step5_eskf_gravity_calib_pass2_begin();
    for (uint32_t i = 0; i < SIM_IMU_DATA_COUNT; i++) {
        const sim_imu_row_t *r = &g_sSimImuData[i];
        step5_eskf_gravity_calib_pass2_sample(r->acc_x, r->acc_y, r->acc_z,
                                               r->gyro_x, r->gyro_y, r->gyro_z);
    }
    step5_eskf_gravity_calib_finalize(GRAVITY);
#endif

#if SIM_CSV_INPUT
    /* In replay mode Python knows the complete CSV pedometer count before
     * selecting its hardware-assisted peak profile. Do the same in C. */
    g_ui32SimHwSteps = sim_hw_step_total();
    g_ui16HwStepCount = (uint16_t)g_ui32SimHwSteps;
    am_util_stdio_printf("[HW detector] Replay pedometer: %u steps -> %u strides\r\n",
                         (unsigned)g_ui32SimHwSteps,
                         (unsigned)(g_ui32SimHwSteps / 2u));
#endif

    // =========================================================================
    // MAIN ACQUISITION LOOP
    // =========================================================================
#if SIM_CSV_INPUT
    {
        uint32_t sim_fv_count = 0u, sim_stride_count = 0u;
        if (!run_sim_python_parity_pipeline(&sim_fv_count, &sim_stride_count)) {
            am_util_stdio_printf("[PY parity] Full-bout path failed; check MAX_BOUT_SAMPLES and dataset gaps.");
        }
        fv_count = sim_fv_count;
        stride_count = sim_stride_count;
        total_samples = DEMO_TOTAL_SAMPLES;
    }
#else
    while (total_samples < DEMO_TOTAL_SAMPLES) {

        uint32_t n_grabbed = grab_fifo_samples(grab_batch, grab_stamps,
                                                FIFO_READ_BATCH, &fifo_seq);
        if (n_grabbed == 0) {
            am_util_delay_ms(FIFO_POLL_MS);
            continue;
        }

#if !SIM_CSV_INPUT
        (void)lsm6dsl_i2c_read_step_count(&g_ui16HwStepCount);
#endif

        for (uint32_t g = 0; g < n_grabbed && total_samples < DEMO_TOTAL_SAMPLES; g++) {
            imu_sample_t *new_sample = &grab_batch[g];
            rtc_stamp_t  *psStamp    = &grab_stamps[g];

#if !SIM_CSV_INPUT
            if (!g_bFirstSecondSkipped)
            {
                if (psStamp->ui8Hundredths == 0)
                {
                    g_bFirstSecondSkipped  = true;
                    fifo_seq               = 1;
                    g_ui32EpochCs          = (uint32_t)psStamp->ui8Hour   * 360000u
                                           + (uint32_t)psStamp->ui8Minute * 6000u
                                           + (uint32_t)psStamp->ui8Second * 100u
                                           + (uint32_t)psStamp->ui8Hundredths;
                    g_ui32LoggingStartCs   = g_ui32EpochCs;
                    am_util_stdio_printf(
                        "Aligned to %02u:%02u:%02u, starting data logging.\r\n",
                        psStamp->ui8Hour, psStamp->ui8Minute, psStamp->ui8Second);
                }
                else
                {
                    continue;
                }
            }

            {
                uint32_t ui32CurCs = (uint32_t)psStamp->ui8Hour   * 360000u
                                   + (uint32_t)psStamp->ui8Minute * 6000u
                                   + (uint32_t)psStamp->ui8Second * 100u
                                   + (uint32_t)psStamp->ui8Hundredths;
                uint32_t ui32ElapsedSec = (ui32CurCs - g_ui32LoggingStartCs) / 100u;
                if (ui32ElapsedSec >= LSM_SESSION_SECONDS)
                {
                    am_util_stdio_printf(
                        "Recording duration reached (%u seconds). Stopping.\r\n",
                        (unsigned)LSM_SESSION_SECONDS);
                    g_bRecordingDone = true;
                    break;
                }
            }
#endif // !SIM_CSV_INPUT

            csv_log_imu_sample(new_sample, psStamp, g_ui16HwStepCount);

#if !SIM_CSV_INPUT
            if (live_session_count < DEMO_TOTAL_SAMPLES) {
                live_session_samples[live_session_count++] = *new_sample;
            }
#endif

            uint32_t cb_count = cb_push(&cb, new_sample);

            if (total_samples % 32 == 0) {
                am_util_stdio_printf(
                    "[Loop] Sample %d | Min %u | Buffer: %d\r\n",
                    total_samples, (unsigned)minute_number, cb_count);
            }

            total_samples++;
            samples_since_process++;

            bool is_minute_boundary = (total_samples - minute_sample_start) >= samples_per_minute;
            bool is_final_sample    = (total_samples >= DEMO_TOTAL_SAMPLES);

            if (false && cb_count >= MAX_SAMPLES &&
                (samples_since_process >= WINDOW_STEP_SAMPLES || is_final_sample || is_minute_boundary)) {
                samples_since_process = 0;

                if (g_bImuCsvOpen) {
                    f_sync(&g_sImuCsvFile);
                }

                cb_get_window(&cb, samples, MAX_SAMPLES);

                bool is_last_window = is_final_sample;
                bool is_edge_window = (!first_window_processed) || is_last_window;
                first_window_processed = true;

                PROFILE_START(gyro_mag);
                step2_gyro_magnitude(samples, MAX_SAMPLES);
                PROFILE_END(gyro_mag);

                static bool gyro_range_printed = false;
                if (!gyro_range_printed) {
                    float min_g = 1e9f, max_g = -1e9f;
                    for (uint32_t i = 0; i < MAX_SAMPLES; i++) {
                        if (samples[i].gyro_mag < min_g) min_g = samples[i].gyro_mag;
                        if (samples[i].gyro_mag > max_g) max_g = samples[i].gyro_mag;
                    }
                    am_util_stdio_printf("  Gyro_mag range: %.2f - %.2f deg/s\r\n", min_g, max_g);
                    gyro_range_printed = true;
                }

                static bool threshold_logged = false;
                if (!threshold_logged) {
                    PROFILE_START(threshold);
                    float threshold = step3_adaptive_threshold(samples, MAX_SAMPLES);
                    PROFILE_END(threshold);
                    am_util_stdio_printf("\r\n[Step 3]  Detecting Gait Events...\r\n");
                    am_util_stdio_printf("  Adaptive threshold (diag): %.2f rad/s\r\n", threshold);
                    threshold_logged = true;
                }

                PROFILE_START(detect);
                float selected_peak_distance_s = PEAK_DIST_FRAC_PRIMARY;
                uint32_t selected_peak_count = 0u;
                uint32_t target_peaks = 0u;

#if SIM_CSV_INPUT
                {
                    /* Python target = hardware_steps / 2 same-foot strides.
                     * Convert that session cadence into the number of peaks
                     * expected inside this overlapping processing window. */
                    float sr_hw = (float)SAMPLE_RATE;
                    uint32_t hw_stride_target = g_ui32SimHwSteps / 2u;
                    float session_s = (float)DEMO_TOTAL_SAMPLES / sr_hw;
                    if (hw_stride_target >= 2u && session_s > 1.0f) {
                        float stride_period = session_s / (float)hw_stride_target;
                        if (stride_period < HW_PEAK_DISTANCE_MIN_S) stride_period = HW_PEAK_DISTANCE_MIN_S;
                        if (stride_period > HW_PEAK_DISTANCE_MAX_S) stride_period = HW_PEAK_DISTANCE_MAX_S;
                        float window_s = (float)MAX_SAMPLES / sr_hw;
                        target_peaks = (uint32_t)(window_s / stride_period + 0.5f) + 1u;
                        if (target_peaks < 2u) target_peaks = 2u;
                    }
                }
#else
                {
                    float elapsed_s = (total_samples > 0u)
                                     ? ((float)total_samples / (float)SAMPLE_RATE)
                                     : 0.0f;
                    target_peaks = hw_target_peaks_for_window(
                        MAX_SAMPLES, (float)SAMPLE_RATE,
                        (uint32_t)g_ui16HwStepCount, elapsed_s);
                }
#endif

                uint32_t n_events;
                if (target_peaks >= 2u) {
                    n_events = step3_detect_events_hw(
                        samples, MAX_SAMPLES, events, MAX_EVENTS,
                        is_edge_window, target_peaks,
                        &selected_peak_distance_s, &selected_peak_count);
                } else {
                    n_events = step3_detect_events(
                        samples, MAX_SAMPLES, events, MAX_EVENTS, is_edge_window);
                }
                PROFILE_END(detect);

                am_util_stdio_printf(
                    "  [HW peak profile] target peaks=%u | selected peaks=%u | distance=%.3fs\r\n",
                    (unsigned)target_peaks, (unsigned)selected_peak_count,
                    selected_peak_distance_s);

                PROFILE_START(kalman);
                // step5_eskf_trajectory() uses an independent ESKF state for
                // every overlapping window; the shared gravity scale is kept.
                step5_eskf_trajectory(samples, MAX_SAMPLES, trajectory, GRAVITY);
                PROFILE_END(kalman);

                uint32_t tds = 0, tos = 0;
                for (uint32_t i = 0; i < n_events; i++) {
                    if (events[i].event == 0) tds++;
                    if (events[i].event == 1) tos++;
                }
                am_util_stdio_printf("  Events: %d TDs | %d TOs\r\n", tds, tos);

                PROFILE_START(build_strides);
                // PYTHON PARITY: compute_temporal_parameters() creates only
                // consecutive real IC-to-IC strides and applies no GCT/stride/
                // swing physiological gates.  Do not synthesize boundary
                // strides and do not reject valid positive-duration strides.
                uint32_t n_strides = step3_build_strides(
                    events, n_events, strides, MAX_STRIDES);
                PROFILE_END(build_strides);
                // Python detect_outliers() currently returns all strides with
                // is_outlier=False; it does not apply the C IQR/physiological
                // rejection. Keep every stride here.
                am_util_stdio_printf("[Strides] Built %d strides (window)\r\n", n_strides);

                for (uint32_t i = 0; i < n_strides; i++) {
                    if (strides[i].td_time_ms < last_emitted_td_time_ms + STRIDE_DEDUP_MIN_GAP_MS) {
                        continue;
                    }
                    last_emitted_td_time_ms = strides[i].td_time_ms;

                    stride_count++;

                    if (fv_count >= MAX_SESSION_STRIDES) {
                        static bool session_buffer_full_logged = false;
                        if (!session_buffer_full_logged) {
                            am_util_stdio_printf(
                                "[ML] Session stride buffer full (%u strides)\r\n",
                                (unsigned)fv_count);
                            session_buffer_full_logged = true;
                        }
                        continue;
                    }

                    PROFILE_START(temporal);
                    step4_temporal(&strides[i], &temp_params[fv_count]);
                    PROFILE_END(temporal);

                    PROFILE_START(spatial);
                    step6_spatial(samples, MAX_SAMPLES, &strides[i], trajectory, &spat_params[fv_count]);
#if SIM_CSV_INPUT
                    // No-op in the current build (see gait_pipeline.c "FIX #3").
                    step6_spatial_raw_sim(&strides[i],
                                          g_sSimImuData, SIM_IMU_DATA_COUNT,
                                          104.0f, GRAVITY,
                                          &spat_params[fv_count]);
#endif
                    // No-op in the current build (see gait_pipeline.c "FIX #2").
                    step6_clearance_from_gyro(samples, MAX_SAMPLES, &strides[i], &spat_params[fv_count]);
                    step6_ilr(samples, MAX_SAMPLES, &strides[i], &spat_params[fv_count]);
                    step6_pronation(samples, MAX_SAMPLES, &strides[i], &spat_params[fv_count]);
                    step6_angle_change(trajectory, MAX_SAMPLES, &strides[i], &spat_params[fv_count]);
                    PROFILE_END(spatial);

                    // PYTHON PARITY: every positive-duration stride remains
                    // in the result set. Do not discard it because a spatial
                    // value is small/zero; Python only performs a bout-level
                    // ESKF failure check on the mean stride length.
                    if (!isfinite(spat_params[fv_count].stride_length_m)) {
                        spat_params[fv_count].stride_length_m = 0.0f;
                    }

                    step4_temporal_with_speed(&temp_params[fv_count], &spat_params[fv_count]);

                    session_strides[fv_count]            = strides[i];
                    session_strides[fv_count].stride_idx = fv_count;

                    csv_log_gait_stride(minute_number,
                                         &session_strides[fv_count],
                                         &temp_params[fv_count],
                                         &spat_params[fv_count]);

                    PROFILE_START(feature_extract);
                    extract_feature_vector(&temp_params[fv_count], &spat_params[fv_count],
                                         &feature_vectors[fv_count]);
                    PROFILE_END(feature_extract);

                    am_util_stdio_printf(
                        "[Min %u | Stride %d] Len=%.3fm | Cad=%.1fspm | GCT=%.1fms\r\n",
                        (unsigned)minute_number, stride_count,
                        spat_params[fv_count].stride_length_m,
                        temp_params[fv_count].cadence_spm,
                        temp_params[fv_count].gct_ms);

                    fv_count++;
                }
            }

            if (is_minute_boundary || is_final_sample) {
                uint32_t minute_stride_count = fv_count - minute_fv_start;
                uint32_t elapsed_s = minute_number * LSM_MINUTE_INTERVAL_S;
                if (is_final_sample && !is_minute_boundary) {
                    elapsed_s = (uint32_t)((float)total_samples / (float)SAMPLE_RATE + 0.5f);
                }

                am_util_stdio_printf("\r\n");
                am_util_stdio_printf("***********************************************************\r\n");
                am_util_stdio_printf("  MINUTE %u GAIT REPORT  (elapsed: %u s | strides: %u)\r\n",
                                     (unsigned)minute_number, (unsigned)elapsed_s,
                                     (unsigned)minute_stride_count);
                am_util_stdio_printf("***********************************************************\r\n");

                if (minute_stride_count > 0) {
                    session_stats_t minute_stats = {0};
                    aggregate_session_ex(
                        &temp_params[minute_fv_start],
                        &spat_params[minute_fv_start],
                        NULL,
                        minute_stride_count,
                        false, 0, 0,
                        &minute_stats);

                    am_util_stdio_printf("  GCT          : %.1f ms   (CV %.1f%%)\r\n",
                                         minute_stats.mean_gct_ms, minute_stats.cv_gct_ms);
                    am_util_stdio_printf("  Stride Time  : %.1f ms   (CV %.1f%%)\r\n",
                                         minute_stats.mean_stride_time_ms, minute_stats.cv_stride_time_ms);
                    am_util_stdio_printf("  Cadence      : %.1f spm  (CV %.1f%%)\r\n",
                                         minute_stats.mean_cadence_spm, minute_stats.cv_cadence_spm);
                    am_util_stdio_printf("  Speed        : %.3f m/s\r\n",
                                         minute_stats.mean_speed_m_s);
                    am_util_stdio_printf("  Stride Len   : %.3f m   (CV %.1f%%)\r\n",
                                         minute_stats.mean_stride_length_m, minute_stats.cv_stride_length_m);
                    am_util_stdio_printf("  Stance Ratio : %.4f\r\n",
                                         minute_stats.mean_stance_ratio);
                    am_util_stdio_printf("  Stride Var SD: %.2f ms\r\n",
                                         minute_stats.stride_variability_sd_ms);

                    csv_log_minute_stats(minute_number, elapsed_s, &minute_stats);
                    f_sync(&g_sGaitCsvFile);
                } else {
                    am_util_stdio_printf("  (no strides detected this minute)\r\n");
                    session_stats_t empty = {0};
                    csv_log_minute_stats(minute_number, elapsed_s, &empty);
                }

                am_util_stdio_printf("***********************************************************\r\n\r\n");

                minute_number++;
                minute_fv_start    = fv_count;
                minute_sample_start = total_samples;
            }

        }

        if (g_bRecordingDone) {
            break;
        }

        am_util_delay_ms(FIFO_POLL_MS);
    }
#endif /* SIM_CSV_INPUT */

#if !SIM_CSV_INPUT
    {
        uint16_t final_hw_steps = 0u;
        (void)lsm6dsl_i2c_read_step_count(&final_hw_steps);
        g_ui16HwStepCount = final_hw_steps;
        fv_count = 0u;
        stride_count = 0u;
        if (!run_live_python_parity_pipeline((uint32_t)final_hw_steps, &fv_count, &stride_count)) {
            am_util_stdio_printf("[LIVE parity] ERROR: full-session processing failed\r\n");
        }
    }
#endif

    // =========================================================================
    // FINAL SESSION REPORT
    // =========================================================================
#if SIM_CSV_INPUT
    const uint32_t hw_stride_count = g_ui32SimHwSteps / 2u;
    uint32_t detected_before_reconciliation = fv_count;
    am_util_stdio_printf(
        "[HW parity] Detector result=%u strides | hardware target=%u strides\r\n",
        (unsigned)detected_before_reconciliation, (unsigned)hw_stride_count);
#endif
    am_util_stdio_printf("\r\n=== Real-Time Pipeline Complete ===\r\n");
    am_util_stdio_printf("[FIFO] Warnings: %lu, Overruns: %lu\r\n",
                        (unsigned long)g_ui32FifoWarnings, (unsigned long)g_ui32FifoOverruns);
    am_util_stdio_printf("[ML] Total feature vectors extracted: %d\r\n", fv_count);

    float total_duration_s = (float)total_samples / (float)SAMPLE_RATE;
    uint32_t dur_min = (uint32_t)(total_duration_s / 60.0f);
    uint32_t dur_sec = (uint32_t)total_duration_s % 60u;
    am_util_stdio_printf("[Duration] Total session: %.1f s (%u min %u s) | Samples: %u\r\n",
                         total_duration_s, (unsigned)dur_min, (unsigned)dur_sec,
                         (unsigned)total_samples);

    uint32_t pipeline_end_cycles = get_cycles();
    uint32_t pipeline_cycles = pipeline_end_cycles - pipeline_start_cycles;
    float pipeline_time_sec = (float)pipeline_cycles / (96000000.0f);

    am_util_stdio_printf("\r\n[Step 4]  Temporal Parameters...\r\n");
    am_util_stdio_printf("  Valid strides: %d\r\n", fv_count);
    if (fv_count > 0) {
        float mean_gct = 0, mean_cadence = 0, mean_stride_time = 0;
        for (uint32_t i = 0; i < fv_count; i++) {
            mean_gct += temp_params[i].gct_ms;
            mean_cadence += temp_params[i].cadence_spm;
            mean_stride_time += temp_params[i].stride_time_ms;
        }
        am_util_stdio_printf("  Mean GCT: %.1f ms | Cadence: %.1f spm | Stride Time: %.1f ms\r\n",
                             mean_gct/fv_count, mean_cadence/fv_count, mean_stride_time/fv_count);
    }

    am_util_stdio_printf("\r\n[Step 6]  Spatial Parameters...\r\n");
    if (fv_count > 0) {
        float mean_stride_len = 0;
        uint32_t valid_cnt = 0;
        for (uint32_t i = 0; i < fv_count; i++) {
            volatile uint32_t *p = (volatile uint32_t *)&spat_params[i].stride_length_m;
            if ((*p & 0x7F800000) != 0x7F800000 && spat_params[i].stride_length_m != 0.0f) {
                mean_stride_len += spat_params[i].stride_length_m;
                valid_cnt++;
            }
        }
        if (valid_cnt > 0) {
            am_util_stdio_printf("  Mean Stride Length: %.3f m (%u/%u valid)\r\n",
                                 mean_stride_len/valid_cnt, (unsigned)valid_cnt, (unsigned)fv_count);
        }
    }

    am_util_stdio_printf("\r\n  Pipeline: %.2f s\r\n", pipeline_time_sec);
    am_util_stdio_printf("=======================================================\r\n");

    am_util_stdio_printf("\r\n[Step 7]  Full 3-Minute Session Report...\r\n");

#if SIM_CSV_INPUT
    // The application convention is two pedometer steps per gait stride.
    // Keep this conversion independent of the software event detector.
    am_util_stdio_printf("\r\n[Step Count] Hardware steps: %u (%u strides)\r\n",
                        (unsigned)g_ui32SimHwSteps, (unsigned)hw_stride_count);
    if (detected_before_reconciliation != hw_stride_count) {
        am_util_stdio_printf(
            "[Info] Hardware-assisted detector differs from pedometer by %+d strides.\r\n",
            (int32_t)detected_before_reconciliation - (int32_t)hw_stride_count);
    }
#endif

    session_stats_t session_stats = {0};
    aggregate_session_ex(temp_params, spat_params, NULL, fv_count,
#if SIM_CSV_INPUT
                        true, g_ui32SimHwSteps, hw_stride_count,
#else
                        false, 0, 0,
#endif
                        &session_stats);
    print_session_report(&session_stats);
    csv_log_session_stats(total_duration_s, total_samples, pipeline_time_sec,
                          &session_stats);

        gait_score_result_t health_score;  //Gait_health_score
        gait_health_score_compute(&session_stats, "walking", "left", &health_score);

        am_util_stdio_printf("\r\n[Health Score] %.1f / 100  (%s)\r\n",
                             isfinite(health_score.health_score)
                               ? health_score.health_score : -1.0f,
                             health_score.band);
        am_util_stdio_printf("  Cadence regularity : %.1f\r\n",
                             isfinite(health_score.sub_cadence_regularity)
                               ? health_score.sub_cadence_regularity : -1.0f);
        am_util_stdio_printf("  Impact loading     : %.1f\r\n",
                             isfinite(health_score.sub_impact_loading)
                               ? health_score.sub_impact_loading : -1.0f);
        am_util_stdio_printf("  GCT consistency    : %.1f\r\n",
                             isfinite(health_score.sub_gct_consistency)
                               ? health_score.sub_gct_consistency : -1.0f);

        csv_log_health_score(&health_score);//GH      
                          
    am_util_stdio_printf("\r\n-- Output CSV Preview (first 8 strides) --\r\n");
    am_util_stdio_printf("  %d total strides across %u minutes\r\n",
                         fv_count, (unsigned)LSM_SESSION_MINUTES);
    am_util_stdio_printf(" stride_idx  td_time_ms  stride_ms  cadence_spm  stride_len_m\r\n");

    uint32_t preview_count = (fv_count < 8) ? fv_count : 8;
    for (uint32_t i = 0; i < preview_count; i++) {
        am_util_stdio_printf(" %9d %10.4f %10.4f %11.2f  %12.4f\r\n",
                            session_strides[i].stride_idx,
                            session_strides[i].td_time_ms,
                            temp_params[i].stride_time_ms,
                            temp_params[i].cadence_spm,
                            spat_params[i].stride_length_m);
    }

    am_util_stdio_printf("\r\n=== Pipeline Profiling ===\r\n");
    profile_print(&current_profile);

    csv_logging_close();
    led1_on();

    while (1) {
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    }
}