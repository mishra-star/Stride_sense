#ifndef SIM_IMU_DATA_H
#define SIM_IMU_DATA_H

// Auto-generated from IMU093_normal_.CSV by gen_sim_data.py - DO NOT HAND-EDIT.
// Regenerate instead if the source CSV changes.
//
// time_ms: elapsed milliseconds normalized to the first source row. The raw
// CSV stores its clock as packed HHMMSSCC; values here are decoded with the
// same HHMMSSCC-to-ms conversion used by gait_params 6.py. gait_pipeline.c
// uses these deltas directly for GCT, stride time, cadence, and ESKF dt.
//
// steps: carries the CSV's own "steps" column verbatim - the LSM6DSL
// on-chip pedometer's cumulative step count. Ends at 308 for this
// recording. aggregate_session_ex() reads the latest value and reports
// hw_step_count / hw_stride_count (steps / 2).

#include <stdint.h>

typedef struct {
    uint32_t time_ms;   // decoded elapsed milliseconds from the raw CSV
    float acc_x, acc_y, acc_z;    // g
    float gyro_x, gyro_y, gyro_z; // deg/s
    uint32_t steps;               // cumulative pedometer count
} sim_imu_row_t;

#define SIM_IMU_DATA_COUNT        16592u
// Duration spanned by one full pass through the data (ms).
// Derived from the decoded first and final HHMMSSCC timestamps.
#define SIM_IMU_DATA_DURATION_MS  179990.0f

extern const sim_imu_row_t g_sSimImuData[SIM_IMU_DATA_COUNT];

#endif // SIM_IMU_DATA_H
