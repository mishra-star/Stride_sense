#ifndef GAIT_PIPELINE_H
#define GAIT_PIPELINE_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "sim_imu_data.h"

// SAMPLE_RATE now matches the LSM6DSL/BMI323 sensor ODR actually configured in
// stridesense_pipeline.c (LSM_XL_ODR / LSM_G_ODR / FIFO_ODR_HZ = 104 Hz), and
// matches gait_params_1.py's SAMPLE_RATE = 104.0.
#define SAMPLE_RATE      104
#define DT               (1.0f / SAMPLE_RATE)
#define GRAVITY          9.81f
#define CALIB_SAMPLES    128

// MAX_SAMPLES is the live streaming window size. Python-parity simulation
// uses MAX_BOUT_SAMPLES for complete walking bouts instead of this window.
#define MAX_SAMPLES      512
#define MAX_EVENTS       384
#define MAX_STRIDES      192
#define MAX_BOUTS        16

// Overlapping-window streaming: re-run the window pipeline every
// WINDOW_STEP_SAMPLES new samples instead of every MAX_SAMPLES. Overlap =
// MAX_SAMPLES - WINDOW_STEP_SAMPLES must exceed the longest expected
// stride so no stride is clipped by every window it appears in. See
// STRIDE_DEDUP_MIN_GAP_MS for the caller-side dedup guard.
#define WINDOW_STEP_SAMPLES        (MAX_SAMPLES / 4)
#define STRIDE_DEDUP_MIN_GAP_MS    300.0f

// Session-wide accumulator size (main-loop temp_params[]/spat_params[]/
// feature_vectors[]).  The bundled IMU093 recording alone contains 308
// pedometer steps, so the former 256-entry limit silently truncated a normal
// session and made the final report always stop at "Detected Strides: 256".
// 384 leaves headroom for the complete recording and its boundary strides.
#define MAX_SESSION_STRIDES  384

// MAX_BOUT_SAMPLES: upper bound for a single walking bout buffer.
// Reduced from 10000 to 2900 (~27 s at 104 Hz) so the large ESKF static
// arrays (P[], C[], v[], s[], etc.) fit inside SHARED_SRAM alongside all
// other pipeline buffers. The embedded recording produces bouts well under
// 27 s after BOUT_GAP_MS segmentation; live hardware is similar.
#define MAX_BOUT_SAMPLES     2900

// ---------------------------------------------------------------------------
// SHARED_SRAM placement attribute
// Applied to every large static array in gait_pipeline.c and LSM_sdcard.c
// so the linker places them in SHARED_SRAM (0x20080000, 3 MB) instead of
// MCU_TCM (496 KB). The .shared section is NOLOAD -- arrays must be zeroed
// explicitly before first use (see the memset block in main()).
// ---------------------------------------------------------------------------
#define SHARED_SRAM  __attribute__((section(".shared")))

#define DEG2RAD_F        (3.14159265358979323846f / 180.0f)

// PYTHON PARITY: compute_temporal_parameters() applies NO physiological
// GCT/stride/swing gating. It only requires stride_time / stance_time /
// swing_time > 0. The C stride builders follow the same rule.

// ── detect_gait_events() peak-detection parameters ──
// Primary/fallback use the same min-distance. Python uses round() when
// converting peak_distance_s * sample_rate to samples.
#define PEAK_HEIGHT_PRIMARY        40.0f
#define PEAK_DIST_FRAC_PRIMARY     0.75f
#define PEAK_PROMINENCE_PRIMARY    15.0f
#define PEAK_HEIGHT_FALLBACK       40.0f
#define PEAK_DIST_FRAC_FALLBACK    0.75f
#define PEAK_PROMINENCE_FALLBACK   15.0f

// Python hardware-assisted detector searches the peak separation from
// 0.35 s through 1.00 s in 0.005 s steps and selects the separation whose
// peak count is closest to the hardware-pedometer target.
#define HW_PEAK_DISTANCE_MIN_S     0.35f
#define HW_PEAK_DISTANCE_MAX_S     1.00f
#define HW_PEAK_DISTANCE_STEP_S    0.005f
#define SWING_WINDOW_FRAC          0.20f
#define SWING_WINDOW_FLOOR_FRAC    0.08f
#define SWING_WINDOW_CEIL_FRAC     0.22f

// ── segment_bouts() parameters ──
#define BOUT_GAP_MS                2000.0f
#define MIN_BOUT_SAMPLES           100

// ── error_state_kalman_filter() PROCESS-NOISE sigmas ──
// These match gait_params_4.py's module-level ACC_SIGMA / GYRO_SIGMA and
// are used by the F * P * F^T + Q propagation.
#define ESKF_ACC_SIGMA             0.05f
#define ESKF_GYRO_SIGMA            0.02f

// ── error_state_kalman_filter() ZUPT MEASUREMENT sigmas ──
// IMPORTANT: these header defines are HISTORICAL / DOCUMENTATION ONLY.
// step5_eskf_trajectory() in gait_pipeline.c IGNORES them and uses
// hard-coded 1e-4f for both, matching Python's IN-FUNCTION override:
//     v_meas_sigma = 1e-4
//     z_meas_sigma = 1e-4
// (Python's error_state_kalman_filter() body overrides the module-level
// V_MEAS_SIGMA=0.01 / Z_MEAS_SIGMA=1e-3 constants at the top of the
// function; the C port now mirrors that override exactly.)
// Do NOT change these defines expecting the ESKF to react - the C code
// path no longer reads them.
#define ESKF_V_MEAS_SIGMA          1e-4f
#define ESKF_Z_MEAS_SIGMA          1e-4f

// ── compute_spatial_parameters() / compute_ilr() parameters ──
#define CLEARANCE_PROMINENCE       0.002f  // m

// STRIDE_SPATIAL_SCALE: multiplies the raw ESKF XY-displacement between
// consecutive ICs to produce stride_length_m. Matches Python's
// compute_spatial_parameters() `raw_len * STRIDE_SPATIAL_SCALE` line
// (Python module-level constant = 3.3).
#define STRIDE_SPATIAL_SCALE       3.3f

// STRIDE_SPATIAL_SCALE_RAW_SIM: was used by step6_spatial_raw_sim(), which
// is now a no-op (see gait_pipeline.c "FIX #3" comment). Kept for source
// compatibility with any external references; has no runtime effect.
#define STRIDE_SPATIAL_SCALE_RAW_SIM 0.86f

#define ILR_WINDOW_MS              50.0f

typedef struct {
    float    time_ms;
    float    acc_x, acc_y, acc_z;
    float    gyro_x, gyro_y, gyro_z;
    float    gyro_mag;
    uint8_t  phase;
} imu_sample_t;

typedef struct {
    uint8_t  event;
    float    time_ms;
    uint32_t sample_idx;
} gait_event_t;

typedef struct {
    uint32_t stride_idx;
    float    td_time_ms;
    float    to_time_ms;
    float    td_next_ms;
    uint32_t td_sample;
    uint32_t to_sample;
    uint32_t td_next_sample;
    bool     is_boundary_imputed;
    bool     is_outlier;
} stride_t;

typedef struct {
    float gct_ms;
    float stance_time_ms;
    float stride_time_ms;
    float swing_time_ms;
    float stance_ratio;
    float cadence_spm;
    float speed_m_s;
    bool  is_boundary_imputed;
    bool  is_outlier;
} temporal_params_t;

typedef struct {
    uint32_t head;
    uint32_t tail;
    uint32_t count;
    uint32_t max_size;
    imu_sample_t *buffer;
} circular_buffer_t;

typedef struct {
    float stride_length_m;
    float min_clearance_m;
    float max_clearance_m;
    float vertical_osc_cm;
    float ilr_g_per_ms;
    float pronation_angle_deg;
    float angle_change;
} spatial_params_t;

typedef struct {
    float w, x, y, z;
} quat_t;

typedef struct {
    float   pos[3];
    float   vel[3];
    quat_t  quat;
    bool    stance;
} traj_sample_t;

typedef struct {
    float gct_ms;
    float stride_time_ms;
    float stance_time_ms;
    float swing_time_ms;
    float stance_ratio;
    float cadence_spm;
    float speed_m_s;

    float stride_length_m;
    float min_clearance_m;
    float max_clearance_m;
    float vertical_osc_cm;
    float ilr_g_per_ms;
    float pronation_angle_deg;

    float cadence_cv;
    float stride_length_cv;
    float gct_cv;
    float stride_time_cv;
    float speed_cv;
} feature_vector_t;

typedef struct {
    float mean_gct_ms;
    float mean_stance_time_ms;
    float mean_stride_time_ms;
    float mean_swing_time_ms;
    float mean_stance_ratio;
    float mean_cadence_spm;
    float mean_speed_m_s;

    float mean_stride_length_m;
    float mean_min_clearance_m;
    float mean_max_clearance_m;
    float mean_vertical_osc_cm;
    float mean_ilr_g_per_ms;
    float mean_pronation_angle_deg;
    float mean_angle_change;

    float cv_gct_ms;
    float cv_stride_time_ms;
    float cv_stance_time_ms;
    float cv_cadence_spm;
    float cv_swing_time_ms;
    float cv_stance_ratio;
    float cv_speed_m_s;

    float cv_stride_length_m;
    float cv_min_clearance_m;
    float cv_max_clearance_m;
    float cv_vertical_osc_cm;
    float cv_ilr_g_per_ms;
    float cv_pronation_angle_deg;
    float cv_angle_change;

    float mean_energy;
    float cv_energy;

    float stride_variability_sd_ms;
    float stride_variability_cv_pct;

    uint32_t n_strides;

    bool     hw_stride_count_available;
    uint32_t hw_step_count;
    uint32_t hw_stride_count;
} session_stats_t;

#define FEATURE_VECTOR_SIZE 18

#define SQRT_LUT_SIZE 1024
extern float sqrt_lut[SQRT_LUT_SIZE];
void init_sqrt_lut(void);
float fast_sqrt(float x);
float gait_decode_packed_time_ms(uint32_t packed_time);

typedef struct {
    uint32_t gyro_mag_us;
    uint32_t threshold_us;
    uint32_t detect_us;
    uint32_t build_strides_us;
    uint32_t temporal_us;
    uint32_t kalman_us;
    uint32_t spatial_us;
    uint32_t feature_extract_us;
    uint32_t total_us;
} pipeline_profile_t;

uint32_t get_cycles(void);
uint32_t cycles_to_us(uint32_t cycles);
void profile_start(void);
void profile_end(pipeline_profile_t *profile);
void profile_print(pipeline_profile_t *profile);

extern pipeline_profile_t current_profile;

#define PROFILE_START(name) \
    uint32_t profile_##name##_start = get_cycles();

#define PROFILE_END(name) \
    current_profile.name##_us = cycles_to_us(get_cycles() - profile_##name##_start);

void     step2_gyro_magnitude(imu_sample_t *samples, uint32_t n);

// Diagnostic-only (Python computes but never gates on it).
float    step3_adaptive_threshold(imu_sample_t *samples, uint32_t n);

// clamp_at_edges: true only for the true first/last window of the whole
// session; false for interior streaming windows (see gait_pipeline.c).
uint32_t step3_detect_events(imu_sample_t *samples, uint32_t n, gait_event_t *events, uint32_t max_events,
                              bool clamp_at_edges);

// Python-parity hardware-assisted event detector. target_peaks is the number
// of gyro swing peaks desired in this processing window. The selected peak
// separation is searched over 0.35..1.00 s exactly like the Python helper.
uint32_t step3_detect_events_hw(imu_sample_t *samples, uint32_t n,
                                gait_event_t *events, uint32_t max_events,
                                bool clamp_at_edges, uint32_t target_peaks,
                                float *selected_peak_distance_s,
                                uint32_t *selected_peak_count);

// Python-parity full-bout detector using a preselected hardware-assisted
// peak distance and the same max_peaks/prominence selection as Python.
uint32_t step3_detect_events_profile(imu_sample_t *samples, uint32_t n,
                                     gait_event_t *events, uint32_t max_events,
                                     bool clamp_at_edges, float peak_distance_s,
                                     uint32_t max_peaks);

// Select the hardware-assisted peak separation across all walking bouts,
// matching Python hardware_assisted_peak_profile().
float step3_hardware_assisted_peak_profile(imu_sample_t **bouts,
                                           const uint32_t *lengths,
                                           uint32_t n_bouts,
                                           uint32_t target_strides,
                                           uint32_t *peak_limits);

uint32_t step3_build_strides(gait_event_t *events, uint32_t n_events, stride_t *strides, uint32_t max_strides);

uint32_t step3_build_strides_ex(gait_event_t *events, uint32_t n_events,
                                 imu_sample_t *samples, uint32_t n_samples,
                                 stride_t *strides, uint32_t max_strides,
                                 bool include_boundary_strides);

uint32_t step2_segment_bouts(imu_sample_t *samples, uint32_t n,
                              uint32_t *bout_start, uint32_t *bout_end,
                              uint32_t max_bouts,
                              float gap_threshold_ms, uint32_t min_bout_samples);

void     step4_temporal(stride_t *stride, temporal_params_t *out);
void     step4_temporal_with_speed(temporal_params_t *temp, spatial_params_t *spat);

// 9-state ESKF + ZUPT + RTS smoother. Cross-window C/v/s/P state persists
// across calls (see step5_eskf_reset_session()).
void     step5_eskf_trajectory(imu_sample_t *samples, uint32_t n, traj_sample_t *traj_out, float g);
void     step5_eskf_reset_session(void);

// Bout-level ESKF: runs the full ESKF over an entire bout in one pass,
// producing a continuous global pos[] trajectory exactly as Python does.
// traj_out must have at least n entries (n <= MAX_BOUT_SAMPLES).
// Call step5_eskf_reset_session() before each bout, then call this once.
// step6_spatial() can then use td_sample/td_next_sample to index traj_out
// directly — no window boundary resets, no position drift across windows.
void     step5_eskf_trajectory_bout(imu_sample_t *samples, uint32_t n,
                                     traj_sample_t *traj_out, float g);

// Whole-session gravity calibration (2-pass streaming API). Call once
// before the first step5_eskf_trajectory().
void     step5_eskf_gravity_calib_begin(void);
void     step5_eskf_gravity_calib_pass1_sample(float acc_x_g, float acc_y_g, float acc_z_g,
                                                float gyro_x_dps, float gyro_y_dps, float gyro_z_dps);
void     step5_eskf_gravity_calib_pass2_begin(void);
void     step5_eskf_gravity_calib_pass2_sample(float acc_x_g, float acc_y_g, float acc_z_g,
                                                float gyro_x_dps, float gyro_y_dps, float gyro_z_dps);
void     step5_eskf_gravity_calib_finalize(float g);

// Spatial: stride length from ESKF XY displacement (x STRIDE_SPATIAL_SCALE),
// clearance / vertical osc from ESKF pos_z prominence peaks.
void     step6_spatial(imu_sample_t *samples, uint32_t n, stride_t *stride, traj_sample_t *traj, spatial_params_t *out);

// Final clearance pass matching gait_params.py's active gyro-magnitude proxy.
void step6_clearance_from_gyro(imu_sample_t *samples, uint32_t n,
                                stride_t *stride, spatial_params_t *out);

// NO-OP in the current build (see gait_pipeline.c "FIX #3" comment).
// Python has no equivalent code path - Python's stride_length_m always
// comes from step6_spatial()'s ESKF-trajectory XY displacement. Kept as a
// symbol so existing SIM_CSV_INPUT call sites still link; leaving it in
// the call chain has no effect on outputs.
void     step6_spatial_raw_sim(const stride_t *stride,
                                const sim_imu_row_t *sim_data, uint32_t n_sim,
                                float odr_hz, float g_mss,
                                spatial_params_t *out);

void     step6_ilr(imu_sample_t *samples, uint32_t n, stride_t *stride, spatial_params_t *out);
void     step6_pronation(imu_sample_t *samples, uint32_t n, stride_t *stride, spatial_params_t *out);
void     step6_angle_change(traj_sample_t *traj, uint32_t n, stride_t *stride, spatial_params_t *out);

void     cb_init(circular_buffer_t *cb, imu_sample_t *buffer, uint32_t size);
uint32_t cb_push(circular_buffer_t *cb, imu_sample_t *sample);
uint32_t cb_get_count(circular_buffer_t *cb);
void     cb_get_window(circular_buffer_t *cb, imu_sample_t *out, uint32_t n);

void     extract_feature_vector(temporal_params_t *tp, spatial_params_t *sp, feature_vector_t *fv);
void     extract_feature_vector_batch(stride_t *strides, uint32_t n_strides,
                                      temporal_params_t *tp_arr, spatial_params_t *sp_arr,
                                      feature_vector_t *fv_out, uint32_t max_fv);

// Pass-through stub matching Python's detect_outliers() (clears is_outlier
// on every stride; no z-score logic is applied in the reference either).
void     step7_detect_outliers(stride_t *strides, uint32_t n_strides);

void     aggregate_session(temporal_params_t *temp_arr, spatial_params_t *spat_arr,
                           uint32_t n_strides, session_stats_t *stats);

void     aggregate_session_ex(temporal_params_t *temp_arr, spatial_params_t *spat_arr,
                              const bool *is_outlier, uint32_t n_strides,
                              bool hw_stride_count_available,
                              uint32_t hw_step_count, uint32_t hw_stride_count,
                              session_stats_t *stats);

void     print_session_report(session_stats_t *stats);

#endif