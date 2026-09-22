#include "gait_pipeline.h"
#include <string.h>
#include <float.h>
#include <math.h>
#include <arm_mve.h>
#include "am_util.h"

// ============================================================================
// PYTHON-PARITY FIXES — gait_params 6-1.py is the authoritative reference
// ============================================================================
// This C revision intentionally follows the uploaded gait_params 6-1.py for
// event/stride construction and spatial metrics.  In particular:
//   1) No 400–2500 ms stride-time gate.
//   2) No 100–1800 ms GCT gate.
//   3) No 80 ms swing-time gate.
//   4) No synthetic leading/trailing boundary strides.
//   5) No C-side IQR/physiological outlier rejection.
//   6) No 0.20–2.50 m stride-length rejection.
//   7) Stride length is exactly XY ESKF displacement × 3.3, rounded to 4 dp.
//   8) The active Python compute_clearance_from_acc() gyro proxy is applied
//      after compute_spatial_parameters(), so its values are retained in C.
//   9) ILR uses recorded timestamps, matching Python's variable-dt calculation.
//  10) Full-bout simulation processing uses one bout-level ESKF, matching the
//      Python call to error_state_kalman_filter() once per walking bout.
//
// The live sensor path remains streaming.  For bit/algorithm parity testing,
// build with SIM_CSV_INPUT=1; that path processes the embedded replay by bouts.
// ============================================================================

// Pre-computed sqrt lookup table
float sqrt_lut[SQRT_LUT_SIZE];

// DWT (Data Watchpoint and Trace) cycle counter for profiling
#define DWT_CTRL    (*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT  (*(volatile uint32_t *)0xE0001004)
#define SCB_DEMCR   (*(volatile uint32_t *)0xE000EDFC)

static uint32_t profile_start_cycles = 0;
pipeline_profile_t current_profile;

uint32_t get_cycles(void) {
    return DWT_CYCCNT;
}

uint32_t cycles_to_us(uint32_t cycles) {
    // Apollo510 runs at 200MHz, so 200 cycles = 1 microsecond
    return cycles / 200;
}

void profile_start(void) {
    // Enable DWT cycle counter
    SCB_DEMCR |= 0x01000000;
    DWT_CTRL |= 1;

    // Reset profile
    memset(&current_profile, 0, sizeof(current_profile));
    profile_start_cycles = get_cycles();
}

void profile_end(pipeline_profile_t *profile) {
    uint32_t total_cycles = get_cycles() - profile_start_cycles;
    profile->total_us = cycles_to_us(total_cycles);
    *profile = current_profile;
}

void profile_print(pipeline_profile_t *profile) {
    am_util_stdio_printf("\r\n=== Gait Pipeline Profiling ===\r\n");
    am_util_stdio_printf("Gyro Magnitude:     %6u us\r\n", profile->gyro_mag_us);
    am_util_stdio_printf("Adaptive Threshold: %6u us\r\n", profile->threshold_us);
    am_util_stdio_printf("Event Detection:    %6u us\r\n", profile->detect_us);
    am_util_stdio_printf("Build Strides:      %6u us\r\n", profile->build_strides_us);
    am_util_stdio_printf("Temporal Params:     %6u us\r\n", profile->temporal_us);
    am_util_stdio_printf("Kalman Filter:      %6u us\r\n", profile->kalman_us);
    am_util_stdio_printf("Spatial Params:     %6u us\r\n", profile->spatial_us);
    am_util_stdio_printf("Feature Extraction: %6u us\r\n", profile->feature_extract_us);
    am_util_stdio_printf("Total Pipeline:     %6u us\r\n", profile->total_us);
    am_util_stdio_printf("Target: <5000us (5ms) - %s\r\n",
                        profile->total_us < 5000 ? "PASS" : "FAIL");
}

void init_sqrt_lut(void) {
    for (int i = 0; i < SQRT_LUT_SIZE; i++) {
        float x = (float)i / (float)SQRT_LUT_SIZE * 100.0f; // 0 to 100 range
        ((float*)sqrt_lut)[i] = sqrtf(x);
    }
}

float fast_sqrt(float x) {
    if (x < 0) return 0;
    if (x >= 100.0f) return sqrtf(x); // Fall back to standard sqrt for large values

    int idx = (int)(x / 100.0f * SQRT_LUT_SIZE);
    if (idx >= SQRT_LUT_SIZE) idx = SQRT_LUT_SIZE - 1;

    return sqrt_lut[idx];
}

// Decode a logger timestamp packed as HHMMSSCC into elapsed milliseconds.
// CC is centiseconds (10 ms each), matching decode_packed_time_ms() in Python.
float gait_decode_packed_time_ms(uint32_t packed_time) {
    uint32_t cc = packed_time % 100u; packed_time /= 100u;
    uint32_t ss = packed_time % 100u; packed_time /= 100u;
    uint32_t mm = packed_time % 100u; packed_time /= 100u;
    uint32_t hh = packed_time % 100u;
    return (float)(hh * 3600000u + mm * 60000u + ss * 1000u + cc * 10u);
}

void step2_gyro_magnitude(imu_sample_t *samples, uint32_t n) {
    for (uint32_t i = 0; i < n; i++) {
        samples[i].gyro_mag = sqrtf(
            samples[i].gyro_x * samples[i].gyro_x +
            samples[i].gyro_y * samples[i].gyro_y +
            samples[i].gyro_z * samples[i].gyro_z
        );
    }
}

// ============================================================================
// Small helpers shared by several steps below
// ============================================================================

// Effective sample-rate inference matching infer_sample_rate() in
// gait_params 6.py: use all non-negative active intervals up to 20 ms.
static float infer_sample_rate_hz(imu_sample_t *samples, uint32_t n) {
    if (n < 2) return (float)SAMPLE_RATE;
    uint32_t active_count = 0;
    float active_duration_ms = 0.0f;
    for (uint32_t i = 1; i < n; i++) {
        float d = samples[i].time_ms - samples[i-1].time_ms;
        if (d >= 0.0f && d <= 20.0f) {
            active_count++;
            active_duration_ms += d;
        }
    }
    return (active_duration_ms > 0.0f)
        ? ((float)active_count * 1000.0f / active_duration_ms)
        : (float)SAMPLE_RATE;
}

// Linear-interpolation percentile over a pre-sorted (ascending) array, matching
// numpy.percentile()'s default ('linear') interpolation.
static float percentile_sorted(const float *sorted_arr, uint32_t n, float pct) {
    if (n == 0) return 0.0f;
    if (n == 1) return sorted_arr[0];
    float idx = (pct / 100.0f) * (float)(n - 1);
    uint32_t lo = (uint32_t)idx;
    uint32_t hi = (lo + 1 < n) ? lo + 1 : n - 1;
    float frac = idx - (float)lo;
    return sorted_arr[lo] + frac * (sorted_arr[hi] - sorted_arr[lo]);
}

static void insertion_sort_f(float *arr, uint32_t n) {
    for (uint32_t i = 1; i < n; i++) {
        float key = arr[i];
        int32_t j = (int32_t)i - 1;
        while (j >= 0 && arr[j] > key) { arr[j+1] = arr[j]; j--; }
        arr[j+1] = key;
    }
}

// ============================================================================
// scipy.signal.find_peaks() port: local-maxima detection (with plateau
// handling), height filtering, distance-based suppression (highest-first),
// and prominence filtering (wlen=None, i.e. full-array bases). Order of
// operations matches scipy's documented evaluation order.
// ============================================================================

static uint32_t local_maxima_1d(const float *x, uint32_t n, uint32_t *out, uint32_t max_out) {
    uint32_t cnt = 0;
    if (n < 3) return 0;
    uint32_t i = 1;
    while (i < n - 1) {
        if (x[i-1] < x[i]) {
            uint32_t i_ahead = i + 1;
            while (i_ahead < n - 1 && x[i_ahead] == x[i]) i_ahead++;
            if (x[i_ahead] < x[i]) {
                uint32_t left_edge = i, right_edge = i_ahead - 1;
                uint32_t mid = (left_edge + right_edge) / 2;
                if (cnt < max_out) out[cnt++] = mid;
                i = i_ahead;
            }
        }
        i++;
    }
    return cnt;
}

static uint32_t select_by_peak_distance(const float *x, uint32_t *peaks, uint32_t n_peaks, float distance) {
    if (n_peaks == 0 || distance < 1.0f) return n_peaks;

    static SHARED_SRAM uint32_t order[MAX_SAMPLES];
    for (uint32_t i = 0; i < n_peaks; i++) order[i] = i;
    // sort `order` ascending by height x[peaks[order[i]]]
    for (uint32_t i = 1; i < n_peaks; i++) {
        uint32_t key = order[i];
        float kv = x[peaks[key]];
        int32_t j = (int32_t)i - 1;
        while (j >= 0 && x[peaks[order[j]]] > kv) { order[j+1] = order[j]; j--; }
        order[j+1] = key;
    }

    static SHARED_SRAM bool keep[MAX_SAMPLES];
    for (uint32_t i = 0; i < n_peaks; i++) keep[i] = true;

    for (int32_t oi = (int32_t)n_peaks - 1; oi >= 0; oi--) {
        uint32_t i = order[oi];
        if (!keep[i]) continue;
        int32_t j = (int32_t)i - 1;
        while (j >= 0 && (float)(peaks[i] - peaks[j]) < distance) { keep[j] = false; j--; }
        uint32_t k = i + 1;
        while (k < n_peaks && (float)(peaks[k] - peaks[i]) < distance) { keep[k] = false; k++; }
    }

    uint32_t out_cnt = 0;
    for (uint32_t i = 0; i < n_peaks; i++) if (keep[i]) peaks[out_cnt++] = peaks[i];
    return out_cnt;
}

static float peak_prominence(const float *x, uint32_t n, uint32_t peak) {
    float left_min = x[peak];
    int32_t i = (int32_t)peak;
    while (i >= 0 && x[i] <= x[peak]) {
        if (x[i] < left_min) left_min = x[i];
        i--;
    }
    float right_min = x[peak];
    uint32_t j = peak;
    while (j < n && x[j] <= x[peak]) {
        if (x[j] < right_min) right_min = x[j];
        j++;
    }
    float base = (left_min > right_min) ? left_min : right_min;
    return x[peak] - base;
}

// use_height=false / use_prominence=false disables that filter (equivalent to
// scipy.find_peaks() omitting the corresponding kwarg).
static uint32_t find_peaks_ex(const float *x, uint32_t n,
                               bool use_height, float min_height,
                               float min_distance,
                               bool use_prominence, float min_prominence,
                               uint32_t *out, uint32_t max_out) {
    static SHARED_SRAM uint32_t idx[MAX_SAMPLES];
    uint32_t cnt = local_maxima_1d(x, n, idx, MAX_SAMPLES);

    if (use_height) {
        uint32_t w = 0;
        for (uint32_t i = 0; i < cnt; i++) if (x[idx[i]] >= min_height) idx[w++] = idx[i];
        cnt = w;
    }

    if (min_distance > 1.0f) {
        cnt = select_by_peak_distance(x, idx, cnt, min_distance);
    }

    if (use_prominence) {
        uint32_t w = 0;
        for (uint32_t i = 0; i < cnt; i++) {
            if (peak_prominence(x, n, idx[i]) >= min_prominence) idx[w++] = idx[i];
        }
        cnt = w;
    }

    uint32_t out_cnt = (cnt < max_out) ? cnt : max_out;
    for (uint32_t i = 0; i < out_cnt; i++) out[i] = idx[i];
    return out_cnt;
}

// ============================================================================
// STEP 3 — compute_adaptive_threshold() [diagnostic-only, see header note]
//          and detect_gait_events() [the detector actually used by main()]
// ============================================================================

float step3_adaptive_threshold(imu_sample_t *samples, uint32_t n) {
    uint32_t cnt = (n < MAX_SAMPLES) ? n : MAX_SAMPLES;
    if (cnt == 0) return 0.0f;
    static SHARED_SRAM float mags[MAX_SAMPLES];
    for (uint32_t i = 0; i < cnt; i++) mags[i] = samples[i].gyro_mag;
    insertion_sort_f(mags, cnt);
    float p20 = percentile_sorted(mags, cnt, 20.0f);
    if (p20 < 20.0f) p20 = 20.0f;
    if (p20 > 60.0f) p20 = 60.0f;
    return p20 * DEG2RAD_F;
}


/*
 * Hardware-assisted gait event detector.
 *
 * The LSM6DSL hardware pedometer supplies the target step count.  The caller
 * converts that to the same-foot stride target and passes it as target_peaks.
 *
 * We search the same 0.35 s .. 1.00 s peak-distance range used by the Python
 * hardware-assisted path.  For every candidate distance, the gyro peak count
 * is measured and the candidate whose count is closest to target_peaks is
 * selected.  The selected peaks are then converted to TO/TD events using the
 * same adaptive swing-window logic as step3_detect_events().
 *
 * Important: this uses the hardware count as a COUNT CONSTRAINT; it does not
 * fabricate events when the signal contains no corresponding peak.
 */
uint32_t step3_detect_events_hw(imu_sample_t *samples, uint32_t n,
                                gait_event_t *events, uint32_t max_events,
                                bool clamp_at_edges, uint32_t target_peaks,
                                float *selected_peak_distance_s,
                                uint32_t *selected_peak_count)
{
    if (selected_peak_distance_s) {
        *selected_peak_distance_s = PEAK_DIST_FRAC_PRIMARY;
    }
    if (selected_peak_count) {
        *selected_peak_count = 0u;
    }

    if (samples == NULL || events == NULL || n < 3u || max_events < 2u) {
        return 0u;
    }

    uint32_t nn = (n < MAX_SAMPLES) ? n : MAX_SAMPLES;

    for (uint32_t i = 0; i < nn; i++) {
        samples[i].phase = 0;       /* STANCE by default */
    }

    static SHARED_SRAM float gyro_mag_hw[MAX_SAMPLES];
    static SHARED_SRAM uint32_t best_peaks[MAX_SAMPLES];
    static SHARED_SRAM uint32_t candidate_peaks[MAX_SAMPLES];

    for (uint32_t i = 0; i < nn; i++) {
        gyro_mag_hw[i] = samples[i].gyro_mag;
    }

    float sr = infer_sample_rate_hz(samples, nn);
    if (!isfinite(sr) || sr <= 1.0f) {
        sr = (float)SAMPLE_RATE;
    }

    /*
     * The Python hardware-assisted search is:
     *   0.35 s .. 1.00 s in 0.005 s increments.
     *
     * scipy's distance argument is an integer number of samples, obtained
     * using round().  Use the same conversion here.
     */
    float best_distance_s = HW_PEAK_DISTANCE_MIN_S;
    uint32_t best_count = 0u;
    uint32_t best_error = UINT32_MAX;
    bool best_valid = false;

    for (float distance_s = HW_PEAK_DISTANCE_MIN_S;
         distance_s <= HW_PEAK_DISTANCE_MAX_S + 0.0001f;
         distance_s += HW_PEAK_DISTANCE_STEP_S) {

        uint32_t distance_samples =
            (uint32_t)(distance_s * sr + 0.5f);

        if (distance_samples < 1u) {
            distance_samples = 1u;
        }

        /*
         * Keep the same signal quality constraints as the normal detector.
         * Only the minimum peak separation is varied.
         */
        uint32_t count = find_peaks_ex(
            gyro_mag_hw, nn,
            true, PEAK_HEIGHT_PRIMARY,
            (float)distance_samples,
            true, PEAK_PROMINENCE_PRIMARY,
            candidate_peaks, MAX_SAMPLES);

        /*
         * If the strict detector finds too few peaks, use the same fallback
         * height/prominence values as step3_detect_events().
         */
        if (count < 2u) {
            count = find_peaks_ex(
                gyro_mag_hw, nn,
                true, PEAK_HEIGHT_FALLBACK,
                (float)distance_samples,
                true, PEAK_PROMINENCE_FALLBACK,
                candidate_peaks, MAX_SAMPLES);
        }

        uint32_t error;
        if (count >= target_peaks) {
            error = count - target_peaks;
        } else {
            error = target_peaks - count;
        }

        /*
         * Prefer the first/earlier candidate on an exact tie.  This keeps the
         * result deterministic.  When target_peaks is zero, choose the normal
         * default distance instead of treating every candidate as equivalent.
         */
        if (target_peaks > 0u) {
            if (!best_valid || error < best_error) {
                best_valid = true;
                best_error = error;
                best_count = count;
                best_distance_s = distance_s;

                if (count > 0u) {
                    for (uint32_t k = 0; k < count; k++) {
                        best_peaks[k] = candidate_peaks[k];
                    }
                }
            }

            if (error == 0u) {
                /*
                 * Exact target found.  Continue is unnecessary because the
                 * objective is count matching; stopping also avoids allowing
                 * a later equal-count distance to alter the selected timing.
                 */
                break;
            }
        } else if (!best_valid) {
            best_valid = true;
            best_count = count;
            best_distance_s = distance_s;
            for (uint32_t k = 0; k < count; k++) {
                best_peaks[k] = candidate_peaks[k];
            }
        }
    }

    if (!best_valid) {
        return 0u;
    }

    if (selected_peak_distance_s) {
        *selected_peak_distance_s = best_distance_s;
    }
    if (selected_peak_count) {
        *selected_peak_count = best_count;
    }

    if (best_count < 1u) {
        return 0u;
    }

    /*
     * Same adaptive swing-window calculation used by step3_detect_events().
     * The peak locations themselves come from the hardware-count-selected
     * distance.
     */
    static SHARED_SRAM float intervals_hw[MAX_SAMPLES];

    uint32_t n_int = (best_count > 1u) ? (best_count - 1u) : 0u;
    for (uint32_t i = 0; i < n_int; i++) {
        intervals_hw[i] =
            (float)(best_peaks[i + 1u] - best_peaks[i]);
    }

    float median_interval;
    if (n_int == 0u) {
        median_interval = best_distance_s * sr;
    } else {
        insertion_sort_f(intervals_hw, n_int);
        median_interval =
            (n_int & 1u)
                ? intervals_hw[n_int / 2u]
                : 0.5f * (intervals_hw[n_int / 2u - 1u] +
                           intervals_hw[n_int / 2u]);
    }

    int32_t win = (int32_t)(0.20f * median_interval + 0.5f);
    int32_t win_floor =
        (int32_t)(SWING_WINDOW_FLOOR_FRAC * sr);
    int32_t win_ceil =
        (int32_t)(SWING_WINDOW_CEIL_FRAC * sr);

    if (win < win_floor) win = win_floor;
    if (win > win_ceil)  win = win_ceil;
    if (win < 1)         win = 1;

    uint32_t out_count = 0u;

    for (uint32_t p = 0; p < best_count; p++) {
        int32_t fo_idx_raw =
            (int32_t)best_peaks[p] - win;
        int32_t ic_idx_raw =
            (int32_t)best_peaks[p] + win;

        int32_t fo_idx;
        int32_t ic_idx;

        if (clamp_at_edges) {
            fo_idx = (fo_idx_raw < 0)
                         ? 0
                         : fo_idx_raw;
            ic_idx = (ic_idx_raw > (int32_t)nn - 1)
                         ? (int32_t)nn - 1
                         : ic_idx_raw;
        } else {
            /*
             * Interior overlapping windows must not emit truncated events.
             * The neighboring window will contain the complete event.
             */
            if (fo_idx_raw < 0 ||
                ic_idx_raw > (int32_t)nn - 1) {
                continue;
            }

            fo_idx = fo_idx_raw;
            ic_idx = ic_idx_raw;
        }

        if (ic_idx <= fo_idx) {
            continue;
        }

        for (int32_t k = fo_idx; k < ic_idx; k++) {
            samples[k].phase = 1;       /* SWING */
        }

        /*
         * Two events per selected gyro peak:
         *   TO = 1
         *   TD = 0
         *
         * Keep the same event ordering and capacity convention as the normal
         * detector.
         */
        if (out_count + 1u >= max_events) {
            break;
        }

        events[out_count].event = 1u;
        events[out_count].time_ms = samples[fo_idx].time_ms;
        events[out_count].sample_idx = (uint32_t)fo_idx;
        out_count++;

        events[out_count].event = 0u;
        events[out_count].time_ms = samples[ic_idx].time_ms;
        events[out_count].sample_idx = (uint32_t)ic_idx;
        out_count++;
    }

    /* Match Python's sorted(events) behavior. */
    for (uint32_t i = 1u; i < out_count; i++) {
        gait_event_t key = events[i];
        int32_t j = (int32_t)i - 1;

        while (j >= 0 &&
               events[j].time_ms > key.time_ms) {
            events[j + 1] = events[j];
            j--;
        }

        events[j + 1] = key;
    }

    return out_count;
}

uint32_t step3_detect_events(imu_sample_t *samples, uint32_t n,
                              gait_event_t *events, uint32_t max_events,
                              bool clamp_at_edges) {
    for (uint32_t i = 0; i < n; i++) samples[i].phase = 0; // default STANCE

    if (n < 3) return 0;

    static SHARED_SRAM float gyro_mag[MAX_SAMPLES];
    uint32_t nn = (n < MAX_SAMPLES) ? n : MAX_SAMPLES;
    for (uint32_t i = 0; i < nn; i++) gyro_mag[i] = samples[i].gyro_mag;

    float sr = infer_sample_rate_hz(samples, nn);

    static SHARED_SRAM uint32_t peaks[MAX_SAMPLES];
    // BUGFIX (prior revision): gait_params_1.py passes distance=max(1, int(0.75*sr))
    // to scipy find_peaks() - an explicitly TRUNCATED integer. Truncate the
    // same way here so a slightly-off inferred sample rate (sensor jitter)
    // doesn't produce a different min-distance than Python would use.
    float min_dist = (float)(int32_t)(PEAK_DIST_FRAC_PRIMARY * sr); if (min_dist < 1.0f) min_dist = 1.0f;
    uint32_t n_peaks = find_peaks_ex(gyro_mag, nn, true, PEAK_HEIGHT_PRIMARY, min_dist,
                                      true, PEAK_PROMINENCE_PRIMARY, peaks, MAX_SAMPLES);

    if (n_peaks < 2) {
        // Fallback relaxes height/prominence only - keep the SAME min-distance
        // as the primary pass so we don't double-count (matches Python).
        float min_dist_fb = (float)(int32_t)(PEAK_DIST_FRAC_FALLBACK * sr); if (min_dist_fb < 1.0f) min_dist_fb = 1.0f;
        n_peaks = find_peaks_ex(gyro_mag, nn, true, PEAK_HEIGHT_FALLBACK, min_dist_fb,
                                 true, PEAK_PROMINENCE_FALLBACK, peaks, MAX_SAMPLES);
    }

    if (n_peaks < 2) return 0;

    // Adaptive swing half-window = 20% of the median inter-peak interval,
    // clamped to [8%, 22%] of the sample rate (in samples).
    static SHARED_SRAM float intervals[MAX_SAMPLES];
    uint32_t n_int = n_peaks - 1;
    for (uint32_t i = 0; i < n_int; i++) intervals[i] = (float)(peaks[i+1] - peaks[i]);
    insertion_sort_f(intervals, n_int);
    float median_interval = (n_int % 2 == 0)
        ? (intervals[n_int/2 - 1] + intervals[n_int/2]) / 2.0f
        : intervals[n_int/2];

    int32_t win = (int32_t)(0.20f * median_interval + 0.5f);
    // BUGFIX (prior revision): Python computes floor/ceil bounds via plain
    // truncation - `int(0.08 * sr)` / `int(0.22 * sr)` - NOT rounding.
    int32_t win_floor = (int32_t)(SWING_WINDOW_FLOOR_FRAC * sr);
    int32_t win_ceil  = (int32_t)(SWING_WINDOW_CEIL_FRAC * sr);
    if (win < win_floor) win = win_floor;
    if (win > win_ceil) win = win_ceil;

    uint32_t count = 0;
    for (uint32_t p = 0; p < n_peaks; p++) {
        int32_t fo_idx_raw = (int32_t)peaks[p] - win;
        int32_t ic_idx_raw = (int32_t)peaks[p] + win;

        // Only clamp (accept a truncated event) when this window is a genuine
        // session boundary. For interior streaming windows, DROP the peak -
        // it will be re-detected cleanly by a neighboring overlapping window.
        int32_t fo_idx, ic_idx;
        if (clamp_at_edges) {
            fo_idx = (fo_idx_raw < 0) ? 0 : fo_idx_raw;
            ic_idx = (ic_idx_raw > (int32_t)nn - 1) ? (int32_t)nn - 1 : ic_idx_raw;
        } else {
            if (fo_idx_raw < 0 || ic_idx_raw > (int32_t)nn - 1) continue;
            fo_idx = fo_idx_raw;
            ic_idx = ic_idx_raw;
        }
        if (ic_idx <= fo_idx) continue;
        for (int32_t k = fo_idx; k < ic_idx; k++) samples[k].phase = 1; // SWING
        if (count + 1 < max_events) {
            events[count].event = 1; // T_toe_off
            events[count].time_ms = samples[fo_idx].time_ms;
            events[count].sample_idx = (uint32_t)fo_idx;
            count++;
            events[count].event = 0; // T_touchdown
            events[count].time_ms = samples[ic_idx].time_ms;
            events[count].sample_idx = (uint32_t)ic_idx;
            count++;
        }
    }

    // sort events by time ascending (matches Python's sorted(events))
    for (uint32_t i = 1; i < count; i++) {
        gait_event_t key = events[i];
        int32_t j = (int32_t)i - 1;
        while (j >= 0 && events[j].time_ms > key.time_ms) { events[j+1] = events[j]; j--; }
        events[j+1] = key;
    }
    return count;
}

uint32_t step3_build_strides(gait_event_t *events, uint32_t n_events,
                              stride_t *strides, uint32_t max_strides) {
    static SHARED_SRAM gait_event_t tds[MAX_EVENTS], tos[MAX_EVENTS];
    uint32_t n_td = 0, n_to = 0, count = 0;
    for (uint32_t i = 0; i < n_events; i++) {
        if (events[i].event == 0 && n_td < MAX_EVENTS) tds[n_td++] = events[i];
        if (events[i].event == 1 && n_to < MAX_EVENTS) tos[n_to++] = events[i];
    }
    uint32_t stride_counter = 0;
    for (uint32_t i = 0; i + 1 < n_td && count < max_strides; i++) {
        float td_curr = tds[i].time_ms, td_next = tds[i+1].time_ms;
        float stride_duration = td_next - td_curr;

        float to_time = -1.0f;
        uint32_t to_sample = 0;
        for (uint32_t j = 0; j < n_to; j++) {
            if (tos[j].time_ms > td_curr && tos[j].time_ms < td_next) {
                to_time = tos[j].time_ms;
                to_sample = tos[j].sample_idx;
                break;
            }
        }
        if (to_time < 0.0f) continue;
        float gct = to_time - td_curr;
        float swing_time = stride_duration - gct;
        if (stride_duration <= 0.0f || gct <= 0.0f || swing_time <= 0.0f) continue;

        // PYTHON PARITY: compute_temporal_parameters() applies NO
        // physiological GCT/stride/swing gates.  It only requires the three
        // durations to be positive.  Do not reject a stride here.

        strides[count].stride_idx     = stride_counter;
        strides[count].td_time_ms     = td_curr;
        strides[count].to_time_ms     = to_time;
        strides[count].td_next_ms     = td_next;
        strides[count].td_sample      = tds[i].sample_idx;
        strides[count].to_sample      = to_sample;
        strides[count].td_next_sample = tds[i+1].sample_idx;
        strides[count].is_boundary_imputed = false;
        strides[count].is_outlier     = false;
        count++;
        stride_counter++;
    }
    return count;
}

// ============================================================================
// STEP 2 — segment_bouts()
// ============================================================================

uint32_t step2_segment_bouts(imu_sample_t *samples, uint32_t n,
                              uint32_t *bout_start, uint32_t *bout_end,
                              uint32_t max_bouts,
                              float gap_threshold_ms, uint32_t min_bout_samples) {
    if (n == 0 || max_bouts == 0) return 0;

    uint32_t n_bouts = 0;
    uint32_t seg_start = 0;
    for (uint32_t i = 1; i <= n; i++) {
        bool gap = (i < n) && ((samples[i].time_ms - samples[i-1].time_ms) > gap_threshold_ms);
        if (gap || i == n) {
            uint32_t seg_end = i; // exclusive
            if ((seg_end - seg_start) > min_bout_samples && n_bouts < max_bouts) {
                bout_start[n_bouts] = seg_start;
                bout_end[n_bouts]   = seg_end;
                n_bouts++;
            }
            seg_start = i;
        }
    }
    return n_bouts;
}

// ============================================================================
// Helper: mirrors Python's _sample_index_for_time (np.searchsorted 'left' +
// nearest-neighbor tie-break), clamped into [0, n-1].
// ============================================================================

static uint32_t sample_index_for_time_ms(const imu_sample_t *samples, uint32_t n, float time_ms) {
    if (n == 0) return 0;
    uint32_t lo = 0, hi = n;
    while (lo < hi) {
        uint32_t mid = lo + (hi - lo) / 2;
        if (samples[mid].time_ms < time_ms) lo = mid + 1; else hi = mid;
    }
    if (lo >= n) return n - 1;
    if (lo > 0 && fabsf(samples[lo-1].time_ms - time_ms) <= fabsf(samples[lo].time_ms - time_ms))
        return lo - 1;
    return lo;
}

// ============================================================================
// STEP 3 (ex) — full compute_temporal_parameters() port, including leading/
// trailing boundary-imputed partial strides.
// ============================================================================

uint32_t step3_build_strides_ex(gait_event_t *events, uint32_t n_events,
                                  imu_sample_t *samples, uint32_t n_samples,
                                  stride_t *strides, uint32_t max_strides,
                                  bool include_boundary_strides) {
    (void)samples;
    (void)n_samples;
    (void)include_boundary_strides;

    /*
     * PYTHON PARITY:
     * gait_params 6-1.py compute_temporal_parameters() creates strides only
     * from consecutive real touchdown events:
     *
     *     IC0 -> IC1
     *     IC1 -> IC2
     *     IC2 -> IC3
     *
     * It does NOT synthesize a leading or trailing partial stride.  The old
     * C step3_build_strides_ex() did exactly that when include_boundary_strides
     * was true, which made the C stride list different from Python.
     *
     * Therefore this API is retained for source compatibility, but boundary
     * synthesis is deliberately disabled and the normal Python-equivalent
     * builder is used.
     */
    return step3_build_strides(events, n_events, strides, max_strides);
}


// ============================================================================
// STEP 7 — detect_outliers(): IQR-based outlier rejection on stride time
// ============================================================================
// Research basis:
//   Hausdorff et al. (2001) "Gait variability and basal ganglia disorders":
//   stride-to-stride CV should be <3% for healthy adults; values >20% indicate
//   pathological variability or measurement artefacts.
//   Moe-Nilssen & Helbostad (2004) recommend IQR ×1.5 fence for gait data
//   to reject sensor artefacts while preserving true gait variability.
//   Stansfield (2006): normal stride time 900–1200 ms walking; up to 2000 ms
//   slow/elderly walking.
//
// Method: sort stride times, compute Q1/Q3/IQR, flag strides outside
//   [Q1 − 1.5×IQR, Q3 + 1.5×IQR] as outliers.
//   Additional hard bounds: stride_time outside [400, 2500] ms always outlier.
// ============================================================================

void step7_detect_outliers(stride_t *strides, uint32_t n_strides) {
    /*
     * PYTHON PARITY:
     * detect_outliers() in gait_params 6-1.py only creates/keeps the
     * is_outlier field as False for this pipeline. It does not apply the C
     * IQR/400-2500 ms rejection rules.
     */
    for (uint32_t i = 0; i < n_strides; i++) {
        strides[i].is_outlier = false;
    }
}


// ============================================================================
// PYTHON HARDWARE-ASSISTED PEAK PROFILE
// ============================================================================
// Mirrors hardware_assisted_peak_profile() in gait_params 6-1.py:
//   desired_peaks = target_strides + number_of_valid_bouts
//   distance candidates = 0.35 .. 1.00 s in 0.005 s steps
//   for each candidate, count peaks independently in each bout
//   choose the candidate with minimum absolute total-count error
//   if there are surplus peaks, remove the lowest-prominence surplus peaks
//   per bout; returned limits are then used by the per-bout detector.
//
// The caller supplies pointers to each bout, so no giant session buffer is
// required here.
// ============================================================================
float step3_hardware_assisted_peak_profile(imu_sample_t **bouts,
                                           const uint32_t *lengths,
                                           uint32_t n_bouts,
                                           uint32_t target_strides,
                                           uint32_t *peak_limits)
{
    if (!bouts || !lengths || !peak_limits || n_bouts == 0u) return 0.75f;

    uint32_t valid_bouts = 0u;
    for (uint32_t b = 0; b < n_bouts; b++) {
        if (lengths[b] >= 2u) valid_bouts++;
        peak_limits[b] = 0u;
    }
    if (valid_bouts == 0u) return 0.75f;

    uint32_t desired_peaks = target_strides + valid_bouts;
    float best_distance = 0.75f;
    uint32_t best_error = UINT32_MAX;
    bool best_valid = false;

    static uint32_t best_counts[MAX_BOUTS];
    static uint32_t cand_counts[MAX_BOUTS];

    for (int32_t step = 0; step <= 130; step++) {
        float distance_s = 0.35f + 0.005f * (float)step;
        uint32_t total = 0u;

        for (uint32_t b = 0; b < n_bouts; b++) {
            if (lengths[b] < 2u) {
                cand_counts[b] = 0u;
                continue;
            }

            uint32_t nn = (lengths[b] < MAX_BOUT_SAMPLES)
                            ? lengths[b] : MAX_BOUT_SAMPLES;
            static SHARED_SRAM float gyro[MAX_BOUT_SAMPLES];
            static SHARED_SRAM uint32_t peaks[MAX_BOUT_SAMPLES];

            float sr = infer_sample_rate_hz(bouts[b], nn);
            float x = distance_s * sr;
            // Python: int(round(x)). Implement round-to-nearest-even for x>=0.
            float fl = floorf(x);
            float frac = x - fl;
            uint32_t distance_samples;
            if (frac > 0.5f) distance_samples = (uint32_t)fl + 1u;
            else if (frac < 0.5f) distance_samples = (uint32_t)fl;
            else distance_samples = (((uint32_t)fl & 1u) == 0u)
                                      ? (uint32_t)fl : (uint32_t)fl + 1u;
            if (distance_samples < 1u) distance_samples = 1u;

            for (uint32_t i = 0; i < nn; i++) gyro[i] = bouts[b][i].gyro_mag;
            uint32_t count = find_peaks_ex(gyro, nn, true, PEAK_HEIGHT_PRIMARY,
                                            (float)distance_samples, true,
                                            PEAK_PROMINENCE_PRIMARY,
                                            peaks, MAX_BOUT_SAMPLES);
            if (count < 2u) {
                count = find_peaks_ex(gyro, nn, true, PEAK_HEIGHT_FALLBACK,
                                      (float)distance_samples, true,
                                      PEAK_PROMINENCE_FALLBACK,
                                      peaks, MAX_BOUT_SAMPLES);
            }
            cand_counts[b] = count;
            total += count;
        }

        uint32_t error = (total >= desired_peaks)
                       ? (total - desired_peaks)
                       : (desired_peaks - total);
        if (!best_valid || error < best_error) {
            best_valid = true;
            best_error = error;
            best_distance = distance_s;
            for (uint32_t b = 0; b < n_bouts; b++) best_counts[b] = cand_counts[b];
        }
        if (error == 0u) break;
    }

    for (uint32_t b = 0; b < n_bouts; b++) peak_limits[b] = best_counts[b];

    // Match Python's surplus removal: only remove the lowest-prominence peaks,
    // preserving at least two peaks per bout.
    uint32_t total_best = 0u;
    for (uint32_t b = 0; b < n_bouts; b++) total_best += peak_limits[b];
    uint32_t excess = (total_best > desired_peaks) ? (total_best - desired_peaks) : 0u;

    while (excess > 0u) {
        float weakest = FLT_MAX;
        uint32_t weakest_b = UINT32_MAX;

        for (uint32_t b = 0; b < n_bouts; b++) {
            if (peak_limits[b] <= 2u) continue;
            uint32_t nn = (lengths[b] < MAX_BOUT_SAMPLES)
                            ? lengths[b] : MAX_BOUT_SAMPLES;
            static SHARED_SRAM float gyro[MAX_BOUT_SAMPLES];
            static SHARED_SRAM uint32_t peaks[MAX_BOUT_SAMPLES];
            float sr = infer_sample_rate_hz(bouts[b], nn);
            float x = best_distance * sr;
            float fl = floorf(x), frac = x - fl;
            uint32_t distance_samples;
            if (frac > 0.5f) distance_samples = (uint32_t)fl + 1u;
            else if (frac < 0.5f) distance_samples = (uint32_t)fl;
            else distance_samples = (((uint32_t)fl & 1u) == 0u)
                                      ? (uint32_t)fl : (uint32_t)fl + 1u;
            if (distance_samples < 1u) distance_samples = 1u;
            for (uint32_t i = 0; i < nn; i++) gyro[i] = bouts[b][i].gyro_mag;
            uint32_t count = find_peaks_ex(gyro, nn, true, PEAK_HEIGHT_PRIMARY,
                                            (float)distance_samples, true,
                                            PEAK_PROMINENCE_PRIMARY,
                                            peaks, MAX_BOUT_SAMPLES);
            if (count < 2u) continue;
            for (uint32_t i = 0; i < count; i++) {
                float prom = peak_prominence(gyro, nn, peaks[i]);
                // Python removes the lowest-prominence surplus peaks while
                // preserving at least two peaks in every bout.
                if (prom < weakest) {
                    weakest = prom;
                    weakest_b = b;
                }
            }
        }
        if (weakest_b == UINT32_MAX) break;
        peak_limits[weakest_b]--;
        excess--;
    }

    return best_distance;
}


// Detect one complete bout using the distance selected by the Python
// hardware-assisted profile and optionally retain only the most prominent
// max_peaks peaks, exactly like detect_gait_events(..., max_peaks=...).
uint32_t step3_detect_events_profile(imu_sample_t *samples, uint32_t n,
                                     gait_event_t *events, uint32_t max_events,
                                     bool clamp_at_edges, float peak_distance_s,
                                     uint32_t max_peaks)
{
    if (!samples || !events || n < 3u || max_events < 2u) return 0u;
    if (n > MAX_BOUT_SAMPLES) n = MAX_BOUT_SAMPLES;

    for (uint32_t i = 0; i < n; i++) samples[i].phase = 0u;

    static SHARED_SRAM float gyro[MAX_BOUT_SAMPLES];
    static SHARED_SRAM uint32_t peaks[MAX_BOUT_SAMPLES];
    static SHARED_SRAM uint32_t selected[MAX_BOUT_SAMPLES];
    for (uint32_t i = 0; i < n; i++) gyro[i] = samples[i].gyro_mag;

    float sr = infer_sample_rate_hz(samples, n);
    float x = peak_distance_s * sr;
    float fl = floorf(x), frac = x - fl;
    uint32_t distance_samples;
    if (frac > 0.5f) distance_samples = (uint32_t)fl + 1u;
    else if (frac < 0.5f) distance_samples = (uint32_t)fl;
    else distance_samples = (((uint32_t)fl & 1u) == 0u)
                              ? (uint32_t)fl : (uint32_t)fl + 1u;
    if (distance_samples < 1u) distance_samples = 1u;

    uint32_t n_peaks = find_peaks_ex(
        gyro, n, true, PEAK_HEIGHT_PRIMARY, (float)distance_samples,
        true, PEAK_PROMINENCE_PRIMARY, peaks, MAX_BOUT_SAMPLES);
    if (n_peaks < 2u) {
        n_peaks = find_peaks_ex(
            gyro, n, true, PEAK_HEIGHT_FALLBACK, (float)distance_samples,
            true, PEAK_PROMINENCE_FALLBACK, peaks, MAX_BOUT_SAMPLES);
    }
    if (n_peaks < 2u) return 0u;

    if (max_peaks > 0u && n_peaks > max_peaks) {
        // Python: keep = np.argsort(prominences)[-max_peaks:], then sort peaks.
        // Repeatedly retain the currently highest-prominence peaks.
        for (uint32_t k = 0; k < max_peaks; k++) {
            float best_prom = -FLT_MAX;
            uint32_t best_i = UINT32_MAX;
            for (uint32_t i = 0; i < n_peaks; i++) {
                bool already = false;
                for (uint32_t j = 0; j < k; j++) if (selected[j] == peaks[i]) { already = true; break; }
                if (already) continue;
                float prom = peak_prominence(gyro, n, peaks[i]);
                if (prom > best_prom) { best_prom = prom; best_i = i; }
            }
            if (best_i == UINT32_MAX) break;
            selected[k] = peaks[best_i];
        }
        n_peaks = max_peaks;
        for (uint32_t i = 1; i < n_peaks; i++) {
            uint32_t key = selected[i];
            int32_t j = (int32_t)i - 1;
            while (j >= 0 && selected[j] > key) { selected[j+1] = selected[j]; j--; }
            selected[j+1] = key;
        }
        for (uint32_t i = 0; i < n_peaks; i++) peaks[i] = selected[i];
    }

    static SHARED_SRAM float intervals[MAX_BOUT_SAMPLES];
    uint32_t n_int = n_peaks - 1u;
    for (uint32_t i = 0; i < n_int; i++) intervals[i] = (float)(peaks[i+1] - peaks[i]);
    insertion_sort_f(intervals, n_int);
    float median_interval = (n_int & 1u)
        ? intervals[n_int/2u]
        : 0.5f * (intervals[n_int/2u - 1u] + intervals[n_int/2u]);

    // Python: win = int(round(0.20 * median_interval))
    float wx = 0.20f * median_interval;
    float wf = floorf(wx), wfrac = wx - wf;
    int32_t win = (wfrac > 0.5f) ? (int32_t)wf + 1
                 : (wfrac < 0.5f) ? (int32_t)wf
                 : ((((uint32_t)wf & 1u) == 0u) ? (int32_t)wf : (int32_t)wf + 1);
    int32_t floor_win = (int32_t)(SWING_WINDOW_FLOOR_FRAC * sr);
    int32_t ceil_win  = (int32_t)(SWING_WINDOW_CEIL_FRAC * sr);
    if (win < floor_win) win = floor_win;
    if (win > ceil_win)  win = ceil_win;
    if (win < 1) win = 1;

    uint32_t count = 0u;
    for (uint32_t pidx = 0; pidx < n_peaks; pidx++) {
        int32_t fo_raw = (int32_t)peaks[pidx] - win;
        int32_t ic_raw = (int32_t)peaks[pidx] + win;
        int32_t fo_idx, ic_idx;
        if (clamp_at_edges) {
            fo_idx = (fo_raw < 0) ? 0 : fo_raw;
            ic_idx = (ic_raw >= (int32_t)n) ? (int32_t)n - 1 : ic_raw;
        } else {
            if (fo_raw < 0 || ic_raw >= (int32_t)n) continue;
            fo_idx = fo_raw; ic_idx = ic_raw;
        }
        if (ic_idx <= fo_idx) continue;
        for (int32_t k = fo_idx; k < ic_idx; k++) samples[k].phase = 1u;
        if (count + 2u > max_events) break;
        events[count++] = (gait_event_t){1u, samples[fo_idx].time_ms, (uint32_t)fo_idx};
        events[count++] = (gait_event_t){0u, samples[ic_idx].time_ms, (uint32_t)ic_idx};
    }

    for (uint32_t i = 1u; i < count; i++) {
        gait_event_t key = events[i];
        int32_t j = (int32_t)i - 1;
        while (j >= 0 && events[j].time_ms > key.time_ms) {
            events[j+1] = events[j]; j--;
        }
        events[j+1] = key;
    }
    return count;
}

// ============================================================================
// STEP 4 — compute_temporal_parameters()
// ============================================================================

void step4_temporal(stride_t *stride, temporal_params_t *out) {
    float gct         = stride->to_time_ms   - stride->td_time_ms;
    float stride_dur  = stride->td_next_ms   - stride->td_time_ms;
    float swing       = stride_dur - gct;
    out->gct_ms         = gct;
    out->stance_time_ms = gct;
    out->stride_time_ms = stride_dur;
    out->swing_time_ms  = swing;
    out->stance_ratio   = (stride_dur > 0.0f) ? (gct / stride_dur) : 0.0f;
    // PYTHON-PARITY FIX (cadence): Python compute_temporal_parameters() uses
    //   "Cadence_spm": round(120000.0 / stride_time, 2)
    // This is the standard definition: 120000 ms/min / stride_time_ms gives
    // strides per minute × 2 feet = steps per minute (SPM), matching clinical
    // gait conventions (normal walking ~100-140 spm).
    // The previous C value of 60000/stride_dur produced half the correct result.
    out->cadence_spm    = (stride_dur > 0.0f) ? (120000.0f / stride_dur) : 0.0f;
    out->speed_m_s      = 0.0f;
    out->is_boundary_imputed = stride->is_boundary_imputed;
    out->is_outlier           = stride->is_outlier;
}

void step4_temporal_with_speed(temporal_params_t *temp, spatial_params_t *spat) {
    if (temp->stride_time_ms > 0 && spat->stride_length_m > 0) {
        temp->speed_m_s = spat->stride_length_m / (temp->stride_time_ms / 1000.0f);
    } else {
        temp->speed_m_s = 0.0f;
    }
}

// ============================================================================
// Small fixed-size linear algebra helpers for the error-state Kalman filter
// ============================================================================

typedef float m3_t[3][3];
typedef float m9_t[9][9];

static void m3_eye(m3_t m) {
    memset(m, 0, sizeof(m3_t));
    m[0][0] = m[1][1] = m[2][2] = 1.0f;
}

static void m3_mul(const m3_t a, const m3_t b, m3_t out) {
    m3_t tmp;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            float s = 0.0f;
            for (int k = 0; k < 3; k++) s += a[i][k] * b[k][j];
            tmp[i][j] = s;
        }
    memcpy(out, tmp, sizeof(m3_t));
}

static void m3_mulvec(const m3_t a, const float v[3], float out[3]) {
    float tmp[3];
    for (int i = 0; i < 3; i++) tmp[i] = a[i][0]*v[0] + a[i][1]*v[1] + a[i][2]*v[2];
    out[0] = tmp[0]; out[1] = tmp[1]; out[2] = tmp[2];
}

static void skew3(const float v[3], m3_t out) {
    out[0][0] = 0.0f;   out[0][1] = -v[2]; out[0][2] =  v[1];
    out[1][0] =  v[2];  out[1][1] = 0.0f;  out[1][2] = -v[0];
    out[2][0] = -v[1];  out[2][1] =  v[0]; out[2][2] = 0.0f;
}

static void m9_eye(m9_t m) {
    memset(m, 0, sizeof(m9_t));
    for (int i = 0; i < 9; i++) m[i][i] = 1.0f;
}

static void m9_mul(const m9_t a, const m9_t b, m9_t out) {
    static m9_t tmp;
    for (int i = 0; i < 9; i++)
        for (int j = 0; j < 9; j++) {
            float s = 0.0f;
            for (int k = 0; k < 9; k++) s += a[i][k] * b[k][j];
            tmp[i][j] = s;
        }
    memcpy(out, tmp, sizeof(m9_t));
}

static void m9_transpose(const m9_t a, m9_t out) {
    static m9_t tmp;
    for (int i = 0; i < 9; i++)
        for (int j = 0; j < 9; j++) tmp[i][j] = a[j][i];
    memcpy(out, tmp, sizeof(m9_t));
}

static bool mat4_invert(float m[4][4], float inv[4][4]) {
    float a[4][8];
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) { a[i][j] = m[i][j]; a[i][j+4] = (i == j) ? 1.0f : 0.0f; }

    for (int col = 0; col < 4; col++) {
        int piv = col; float maxval = fabsf(a[col][col]);
        for (int r = col + 1; r < 4; r++) if (fabsf(a[r][col]) > maxval) { maxval = fabsf(a[r][col]); piv = r; }
        if (maxval < 1e-12f) return false;
        if (piv != col) for (int c = 0; c < 8; c++) { float t = a[col][c]; a[col][c] = a[piv][c]; a[piv][c] = t; }
        float pv = a[col][col];
        for (int c = 0; c < 8; c++) a[col][c] /= pv;
        for (int r = 0; r < 4; r++) {
            if (r == col) continue;
            float f = a[r][col];
            for (int c = 0; c < 8; c++) a[r][c] -= f * a[col][c];
        }
    }
    for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) inv[i][j] = a[i][j+4];
    return true;
}

static bool mat9_invert(const m9_t m, m9_t inv) {
    static float a[9][18];
    for (int i = 0; i < 9; i++)
        for (int j = 0; j < 9; j++) { a[i][j] = m[i][j]; a[i][j+9] = (i == j) ? 1.0f : 0.0f; }

    for (int col = 0; col < 9; col++) {
        int piv = col; float maxval = fabsf(a[col][col]);
        for (int r = col + 1; r < 9; r++) if (fabsf(a[r][col]) > maxval) { maxval = fabsf(a[r][col]); piv = r; }
        if (maxval < 1e-12f) return false;
        if (piv != col) for (int c = 0; c < 18; c++) { float t = a[col][c]; a[col][c] = a[piv][c]; a[piv][c] = t; }
        float pv = a[col][col];
        for (int c = 0; c < 18; c++) a[col][c] /= pv;
        for (int r = 0; r < 9; r++) {
            if (r == col) continue;
            float f = a[r][col];
            if (f == 0.0f) continue;
            for (int c = 0; c < 18; c++) a[r][c] -= f * a[col][c];
        }
    }
    for (int i = 0; i < 9; i++) for (int j = 0; j < 9; j++) inv[i][j] = a[i][j+9];
    return true;
}

static void build_F_from_fg(const float fg[3], float dt, m9_t F_out) {
    m3_t S; skew3(fg, S);
    m9_eye(F_out);
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) F_out[3+i][j] = S[i][j]*dt;
    for (int i = 0; i < 3; i++) F_out[6+i][3+i] = dt;
}

static quat_t mat3_to_quat(const m3_t m) {
    quat_t q;
    float tr = m[0][0] + m[1][1] + m[2][2];
    if (tr > 0.0f) {
        float S = sqrtf(tr + 1.0f) * 2.0f;
        q.w = 0.25f * S;
        q.x = (m[2][1] - m[1][2]) / S;
        q.y = (m[0][2] - m[2][0]) / S;
        q.z = (m[1][0] - m[0][1]) / S;
    } else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
        float S = sqrtf(1.0f + m[0][0] - m[1][1] - m[2][2]) * 2.0f;
        q.w = (m[2][1] - m[1][2]) / S;
        q.x = 0.25f * S;
        q.y = (m[0][1] + m[1][0]) / S;
        q.z = (m[0][2] + m[2][0]) / S;
    } else if (m[1][1] > m[2][2]) {
        float S = sqrtf(1.0f + m[1][1] - m[0][0] - m[2][2]) * 2.0f;
        q.w = (m[0][2] - m[2][0]) / S;
        q.x = (m[0][1] + m[1][0]) / S;
        q.y = 0.25f * S;
        q.z = (m[1][2] + m[2][1]) / S;
    } else {
        float S = sqrtf(1.0f + m[2][2] - m[0][0] - m[1][1]) * 2.0f;
        q.w = (m[1][0] - m[0][1]) / S;
        q.x = (m[0][2] + m[2][0]) / S;
        q.y = (m[1][2] + m[2][1]) / S;
        q.z = 0.25f * S;
    }
    // scipy's Rotation.from_matrix(), used by the Python reference, returns
    // a unit quaternion. Normalize the C result before relative-angle use.
    float norm = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (norm > 1e-12f) {
        q.w /= norm; q.x /= norm; q.y /= norm; q.z /= norm;
    } else {
        q = (quat_t){1.0f, 0.0f, 0.0f, 0.0f};
    }
    return q;
}

// ============================================================================
// STEP 5 — error_state_kalman_filter() / simple_trajectory_fallback()
// ============================================================================

static void simple_trajectory_fallback(imu_sample_t *samples, uint32_t n, float g, traj_sample_t *traj_out) {
    if (n == 0) return;

    uint32_t num_calib = (uint32_t)(n * 0.10f);
    if (num_calib < 1) num_calib = 1;
    float az_sum = 0.0f;
    for (uint32_t i = 0; i < num_calib; i++) az_sum += samples[i].acc_z * g;
    float az_baseline = az_sum / (float)num_calib;

    float vel[3] = {0,0,0}, pos[3] = {0,0,0};
    traj_out[0].pos[0] = traj_out[0].pos[1] = traj_out[0].pos[2] = 0.0f;
    traj_out[0].vel[0] = traj_out[0].vel[1] = traj_out[0].vel[2] = 0.0f;
    traj_out[0].quat = (quat_t){1.0f, 0.0f, 0.0f, 0.0f};
    traj_out[0].stance = false;

    for (uint32_t i = 1; i < n; i++) {
        float dt = (samples[i].time_ms - samples[i-1].time_ms) / 1000.0f;
        if (!(dt > 1e-6f)) dt = 1.0f / (float)SAMPLE_RATE;

        float ax = samples[i].acc_x * g;
        float ay = samples[i].acc_y * g;
        float az = samples[i].acc_z * g - az_baseline;

        vel[0] += ax * dt; vel[1] += ay * dt; vel[2] += az * dt;
        pos[0] += vel[0] * dt; pos[1] += vel[1] * dt; pos[2] += vel[2] * dt;

        traj_out[i].pos[0] = pos[0]; traj_out[i].pos[1] = pos[1]; traj_out[i].pos[2] = pos[2];
        traj_out[i].vel[0] = vel[0]; traj_out[i].vel[1] = vel[1]; traj_out[i].vel[2] = vel[2];
        traj_out[i].quat = (quat_t){1.0f, 0.0f, 0.0f, 0.0f};
        traj_out[i].stance = false;
    }
}

// ── Cross-window continuity state ──────────────────────────────────────────
// See prior revision's long comment (dropped here for brevity) - persist
// C/v/s/P across windows so the trajectory is continuous, matching Python's
// single continuous integration.
static bool  s_carry_valid       = false;
static m3_t  s_carry_C;
static float s_carry_v[3];
static float s_carry_s[3];
static m9_t  s_carry_P;
static bool  s_carry_scale_valid = false;
static float s_carry_scale       = 1.0f;

void step5_eskf_reset_session(void) {
    s_carry_valid       = false;
    s_carry_scale_valid = false;
    s_carry_scale       = 1.0f;
}

// ============================================================================
// Whole-session gravity calibration (2-pass over full session)
// ============================================================================
#define GRAV_CAL_HIST_BINS      600
#define GRAV_CAL_HIST_MAX_RAD   15.0f
#define GRAV_CAL_FALLBACK_N     200

typedef struct {
    uint32_t hist[GRAV_CAL_HIST_BINS];
    uint32_t hist_total;
    float    fallback_acc_sum[3];
    uint32_t fallback_cnt;
    float    quiet_thresh_rad;
    float    quiet_acc_sum[3];
    uint32_t quiet_cnt;
} grav_calib_state_t;

static grav_calib_state_t s_grav_calib;

void step5_eskf_gravity_calib_begin(void) {
    memset(&s_grav_calib, 0, sizeof(s_grav_calib));
}

void step5_eskf_gravity_calib_pass1_sample(float acc_x_g, float acc_y_g, float acc_z_g,
                                            float gyro_x_dps, float gyro_y_dps, float gyro_z_dps) {
    float gx = gyro_x_dps * DEG2RAD_F, gy = gyro_y_dps * DEG2RAD_F, gz = gyro_z_dps * DEG2RAD_F;
    float mag = sqrtf(gx*gx + gy*gy + gz*gz);
    uint32_t bin = (uint32_t)((mag / GRAV_CAL_HIST_MAX_RAD) * (float)GRAV_CAL_HIST_BINS);
    if (bin >= GRAV_CAL_HIST_BINS) bin = GRAV_CAL_HIST_BINS - 1;
    s_grav_calib.hist[bin]++;
    s_grav_calib.hist_total++;

    if (s_grav_calib.fallback_cnt < GRAV_CAL_FALLBACK_N) {
        s_grav_calib.fallback_acc_sum[0] += acc_x_g;
        s_grav_calib.fallback_acc_sum[1] += acc_y_g;
        s_grav_calib.fallback_acc_sum[2] += acc_z_g;
        s_grav_calib.fallback_cnt++;
    }
}

void step5_eskf_gravity_calib_pass2_begin(void) {
    uint32_t target = (uint32_t)(0.20f * (float)s_grav_calib.hist_total);
    uint32_t cum = 0;
    uint32_t thresh_bin = GRAV_CAL_HIST_BINS - 1;
    for (uint32_t b = 0; b < GRAV_CAL_HIST_BINS; b++) {
        cum += s_grav_calib.hist[b];
        if (cum >= target) { thresh_bin = b; break; }
    }
    s_grav_calib.quiet_thresh_rad = ((float)(thresh_bin + 1) / (float)GRAV_CAL_HIST_BINS) * GRAV_CAL_HIST_MAX_RAD;

    s_grav_calib.quiet_acc_sum[0] = s_grav_calib.quiet_acc_sum[1] = s_grav_calib.quiet_acc_sum[2] = 0.0f;
    s_grav_calib.quiet_cnt = 0;
}

void step5_eskf_gravity_calib_pass2_sample(float acc_x_g, float acc_y_g, float acc_z_g,
                                            float gyro_x_dps, float gyro_y_dps, float gyro_z_dps) {
    float gx = gyro_x_dps * DEG2RAD_F, gy = gyro_y_dps * DEG2RAD_F, gz = gyro_z_dps * DEG2RAD_F;
    float mag = sqrtf(gx*gx + gy*gy + gz*gz);
    if (mag < s_grav_calib.quiet_thresh_rad) {
        s_grav_calib.quiet_acc_sum[0] += acc_x_g;
        s_grav_calib.quiet_acc_sum[1] += acc_y_g;
        s_grav_calib.quiet_acc_sum[2] += acc_z_g;
        s_grav_calib.quiet_cnt++;
    }
}

void step5_eskf_gravity_calib_finalize(float g) {
    float g_vec[3];
    uint32_t used_cnt;
    if (s_grav_calib.quiet_cnt >= 20) {
        used_cnt = s_grav_calib.quiet_cnt;
        g_vec[0] = s_grav_calib.quiet_acc_sum[0] / (float)used_cnt * g;
        g_vec[1] = s_grav_calib.quiet_acc_sum[1] / (float)used_cnt * g;
        g_vec[2] = s_grav_calib.quiet_acc_sum[2] / (float)used_cnt * g;
    } else if (s_grav_calib.fallback_cnt > 0) {
        used_cnt = s_grav_calib.fallback_cnt;
        g_vec[0] = s_grav_calib.fallback_acc_sum[0] / (float)used_cnt * g;
        g_vec[1] = s_grav_calib.fallback_acc_sum[1] / (float)used_cnt * g;
        g_vec[2] = s_grav_calib.fallback_acc_sum[2] / (float)used_cnt * g;
    } else {
        return;
    }

    float g_mag = sqrtf(g_vec[0]*g_vec[0] + g_vec[1]*g_vec[1] + g_vec[2]*g_vec[2]);
    am_util_stdio_printf("[Cal] gravity from %u quiet samples (whole session): "
                          "mag=%.3f m/s^2 (ideal=9.81)\r\n",
                          (unsigned)used_cnt, g_mag);
    if (g_mag > 7.0f && g_mag < 14.0f) {
        s_carry_scale = 9.81f / g_mag;
        s_carry_scale_valid = true;
        am_util_stdio_printf("[Cal] gain correction x%.4f (was %.3f -> now %.3f m/s^2)\r\n",
                              s_carry_scale, g_mag, g_mag * s_carry_scale);
    }
}

void step5_eskf_trajectory(imu_sample_t *samples, uint32_t n, traj_sample_t *traj_out, float g) {
    if (n == 0) return;
    if (n > MAX_SAMPLES) n = MAX_SAMPLES;

    // ────────────────────────────────────────────────────────────────────
    // FIX #1 (see file-top): ESKF ZUPT measurement sigmas.
    // Python's error_state_kalman_filter() OVERRIDES the module-level
    // constants inside the function body:
    //     v_meas_sigma = 1e-4
    //     z_meas_sigma = 1e-4
    // The C header defines (ESKF_V_MEAS_SIGMA=0.01, ESKF_Z_MEAS_SIGMA=1e-3)
    // are 100x and 10x too large respectively. Use function-local values
    // that match Python's override, ignoring the header defines.
    // ────────────────────────────────────────────────────────────────────
    const float v_meas_sigma = 1e-4f;
    const float z_meas_sigma = 1e-4f;
    // ────────────────────────────────────────────────────────────────────

    static SHARED_SRAM float acc[MAX_SAMPLES][3];
    static SHARED_SRAM float gyro[MAX_SAMPLES][3];
    static SHARED_SRAM float time_s[MAX_SAMPLES];
    for (uint32_t i = 0; i < n; i++) {
        time_s[i] = samples[i].time_ms / 1000.0f;
        acc[i][0] = samples[i].acc_x * g;
        acc[i][1] = samples[i].acc_y * g;
        acc[i][2] = samples[i].acc_z * g;
        gyro[i][0] = samples[i].gyro_x * DEG2RAD_F;
        gyro[i][1] = samples[i].gyro_y * DEG2RAD_F;
        gyro[i][2] = samples[i].gyro_z * DEG2RAD_F;
    }

    // ── Gravity-vector calibration ──
    // Normally already resolved by the whole-session pre-pass. Window-local
    // fallback only fires if the caller never did that.
    if (s_carry_scale_valid) {
        for (uint32_t i = 0; i < n; i++) { acc[i][0] *= s_carry_scale; acc[i][1] *= s_carry_scale; acc[i][2] *= s_carry_scale; }
    } else {
        static SHARED_SRAM float gyro_mag_rad[MAX_SAMPLES];
        for (uint32_t i = 0; i < n; i++)
            gyro_mag_rad[i] = sqrtf(gyro[i][0]*gyro[i][0] + gyro[i][1]*gyro[i][1] + gyro[i][2]*gyro[i][2]);
        static SHARED_SRAM float sorted_buf[MAX_SAMPLES];
        memcpy(sorted_buf, gyro_mag_rad, n * sizeof(float));
        insertion_sort_f(sorted_buf, n);
        float quiet_thresh = percentile_sorted(sorted_buf, n, 20.0f);

        float g_vec[3] = {0,0,0};
        uint32_t quiet_cnt = 0;
        for (uint32_t i = 0; i < n; i++) {
            if (gyro_mag_rad[i] < quiet_thresh) {
                g_vec[0] += acc[i][0]; g_vec[1] += acc[i][1]; g_vec[2] += acc[i][2];
                quiet_cnt++;
            }
        }
        if (quiet_cnt >= 20) {
            g_vec[0] /= quiet_cnt; g_vec[1] /= quiet_cnt; g_vec[2] /= quiet_cnt;
        } else {
            uint32_t m = (n < 200) ? n : 200;
            g_vec[0] = g_vec[1] = g_vec[2] = 0.0f;
            for (uint32_t i = 0; i < m; i++) { g_vec[0] += acc[i][0]; g_vec[1] += acc[i][1]; g_vec[2] += acc[i][2]; }
            if (m > 0) { g_vec[0] /= m; g_vec[1] /= m; g_vec[2] /= m; }
        }
        float g_mag = sqrtf(g_vec[0]*g_vec[0] + g_vec[1]*g_vec[1] + g_vec[2]*g_vec[2]);
        if (g_mag > 7.0f && g_mag < 14.0f) {
            s_carry_scale = 9.81f / g_mag;
            s_carry_scale_valid = true;
            for (uint32_t i = 0; i < n; i++) { acc[i][0] *= s_carry_scale; acc[i][1] *= s_carry_scale; acc[i][2] *= s_carry_scale; }
        }
    }

    bool has_stance = false;
    for (uint32_t i = 0; i < n; i++) if (samples[i].phase == 0) { has_stance = true; break; }
    if (!has_stance) {
        simple_trajectory_fallback(samples, n, g, traj_out);
        return;
    }

    static SHARED_SRAM m3_t C[MAX_SAMPLES];
    static SHARED_SRAM float v[MAX_SAMPLES][3];
    static SHARED_SRAM float s[MAX_SAMPLES][3];
    static SHARED_SRAM m9_t P[MAX_SAMPLES];
    static SHARED_SRAM float fg_hist[MAX_SAMPLES][3];
    static SHARED_SRAM float dt_hist[MAX_SAMPLES];
    static SHARED_SRAM float dx_rts[MAX_SAMPLES][9];
    memset(C, 0, sizeof(C));
    memset(v, 0, sizeof(v));
    memset(s, 0, sizeof(s));
    memset(P, 0, sizeof(P));
    memset(fg_hist, 0, sizeof(fg_hist));
    memset(dt_hist, 0, sizeof(dt_hist));
    memset(dx_rts, 0, sizeof(dx_rts));

    // Solve each overlapping 512-sample window independently.
    // The previous implementation carried orientation/covariance into the next
    // window while resetting position/velocity. That inconsistent state caused
    // stride length to grow from ~1 m to several metres over time.
    // Whole-session gravity scale remains shared via s_carry_scale_valid.
    bool orientation_init = false;
    float sr = infer_sample_rate_hz(samples, n);

    for (uint32_t t = 0; t < n; t++) {
        float dt;
        if (t > 0) {
            dt = time_s[t] - time_s[t-1];
            if (!(dt > 1e-6f)) dt = 1.0f / sr;
        } else {
            dt = 1.0f / sr;
        }

        if (!orientation_init) {
            if (t > 0 && samples[t-1].phase == 0) {
                float norm_a = sqrtf(acc[t-1][0]*acc[t-1][0] + acc[t-1][1]*acc[t-1][1] + acc[t-1][2]*acc[t-1][2]);
                float a_ref[3] = {
                    acc[t-1][0] / (norm_a + 1e-12f),
                    acc[t-1][1] / (norm_a + 1e-12f),
                    acc[t-1][2] / (norm_a + 1e-12f)
                };
                float up[3] = {0.0f, 1.0f, 0.0f};
                float x_ax[3] = {
                    up[1]*a_ref[2] - up[2]*a_ref[1],
                    up[2]*a_ref[0] - up[0]*a_ref[2],
                    up[0]*a_ref[1] - up[1]*a_ref[0]
                };
                float nx = sqrtf(x_ax[0]*x_ax[0] + x_ax[1]*x_ax[1] + x_ax[2]*x_ax[2]) + 1e-12f;
                x_ax[0] /= nx; x_ax[1] /= nx; x_ax[2] /= nx;
                float y_ax[3] = {
                    a_ref[1]*x_ax[2] - a_ref[2]*x_ax[1],
                    a_ref[2]*x_ax[0] - a_ref[0]*x_ax[2],
                    a_ref[0]*x_ax[1] - a_ref[1]*x_ax[0]
                };
                float ny = sqrtf(y_ax[0]*y_ax[0] + y_ax[1]*y_ax[1] + y_ax[2]*y_ax[2]) + 1e-12f;
                y_ax[0] /= ny; y_ax[1] /= ny; y_ax[2] /= ny;

                C[t-1][0][0]=x_ax[0]; C[t-1][0][1]=x_ax[1]; C[t-1][0][2]=x_ax[2];
                C[t-1][1][0]=y_ax[0]; C[t-1][1][1]=y_ax[1]; C[t-1][1][2]=y_ax[2];
                C[t-1][2][0]=a_ref[0]; C[t-1][2][1]=a_ref[1]; C[t-1][2][2]=a_ref[2];

                P[t-1][0][0] = 1e-3f; P[t-1][1][1] = 1e-3f; P[t-1][2][2] = 1e-10f;
                P[t-1][3][3] = 1e-5f; P[t-1][4][4] = 1e-5f; P[t-1][5][5] = 1e-5f;
                // Match Python: P[6:9,6:9] remains zero at initialization.
                // z_meas_sigma is positive, so the ZUPT innovation matrix
                // remains invertible without a C-only position seed.
                orientation_init = true;
            } else {
                m3_eye(C[t]);
                continue;
            }
        }

        float gv[3] = {gyro[t][0], gyro[t][1], gyro[t][2]};
        float gnorm = sqrtf(gv[0]*gv[0] + gv[1]*gv[1] + gv[2]*gv[2]);
        if (gnorm == 0.0f) { gv[0]=gv[1]=gv[2]=1e-10f; gnorm = sqrtf(3.0f)*1e-10f; }

        float angle = gnorm * dt;
        m3_t Bn; skew3(gv, Bn);
        for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) Bn[i][j] /= gnorm;
        m3_t Bn2; m3_mul(Bn, Bn, Bn2);
        m3_t R_rot;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                R_rot[i][j] = (i==j ? 1.0f : 0.0f) + sinf(angle)*Bn[i][j] + (1.0f - cosf(angle))*Bn2[i][j];

        m3_mul(C[t-1], R_rot, C[t]);

        float fg[3]; m3_mulvec(C[t], acc[t], fg);
        float ag[3] = { fg[0], fg[1], fg[2] - g };

        for (int k = 0; k < 3; k++) v[t][k] = v[t-1][k] + ag[k]*dt;
        for (int k = 0; k < 3; k++) s[t][k] = s[t-1][k] + v[t][k]*dt;

        dt_hist[t] = dt;
        fg_hist[t][0] = fg[0]; fg_hist[t][1] = fg[1]; fg_hist[t][2] = fg[2];
        static m9_t F_t;
        build_F_from_fg(fg, dt, F_t);

        static m9_t FP, Ft, FPFt;
        m9_mul(F_t, P[t-1], FP);
        m9_transpose(F_t, Ft);
        m9_mul(FP, Ft, FPFt);

        float Tc = dt*dt*ESKF_GYRO_SIGMA*ESKF_GYRO_SIGMA;
        float Uc = dt*dt*ESKF_ACC_SIGMA*ESKF_ACC_SIGMA;
        for (int i = 0; i < 3; i++) FPFt[i][i]   += Tc;
        for (int i = 0; i < 3; i++) FPFt[3+i][3+i] += Uc;
        memcpy(P[t], FPFt, sizeof(m9_t));

        if (samples[t].phase == 0) { // STANCE -> Zero-velocity + zero-Z update
            const uint32_t idx[4] = {3,4,5,8};
            float S4[4][4];
            for (int a = 0; a < 4; a++) for (int b = 0; b < 4; b++) S4[a][b] = P[t][idx[a]][idx[b]];
            // FIX #1: use function-local sigmas (matching Python's override),
            // NOT the header ESKF_V_MEAS_SIGMA / ESKF_Z_MEAS_SIGMA.
            float sigma2  = v_meas_sigma * v_meas_sigma;
            float zsigma2 = z_meas_sigma * z_meas_sigma;
            S4[0][0] += sigma2; S4[1][1] += sigma2; S4[2][2] += sigma2; S4[3][3] += zsigma2;

            float Minv[4][4];
            if (mat4_invert(S4, Minv)) {
                float y[4] = { -v[t][0], -v[t][1], -v[t][2], -s[t][2] };
                float Kc[9][4];
                for (int r = 0; r < 9; r++)
                    for (int c = 0; c < 4; c++) {
                        float sum = 0.0f;
                        for (int k = 0; k < 4; k++) sum += P[t][r][idx[k]] * Minv[k][c];
                        Kc[r][c] = sum;
                    }
                float dx_local[9];
                for (int r = 0; r < 9; r++) {
                    float sum = 0.0f;
                    for (int k = 0; k < 4; k++) sum += Kc[r][k]*y[k];
                    dx_local[r] = sum;
                }

                v[t][0]+=dx_local[3]; v[t][1]+=dx_local[4]; v[t][2]+=dx_local[5];
                s[t][0]+=dx_local[6]; s[t][1]+=dx_local[7]; s[t][2]+=dx_local[8];

                float phi = dx_local[0], theta = dx_local[1], psi = dx_local[2];
                m3_t rot_corr = {
                    { cosf(theta)*cosf(psi),  cosf(theta)*sinf(psi), -sinf(theta) },
                    { sinf(phi)*sinf(theta)*cosf(psi) - cosf(phi)*sinf(psi),
                      sinf(phi)*sinf(theta)*sinf(psi) + cosf(phi)*cosf(psi),
                      sinf(phi)*cosf(theta) },
                    { cosf(phi)*sinf(theta)*cosf(psi) + sinf(phi)*sinf(psi),
                      cosf(phi)*sinf(theta)*sinf(psi) - sinf(phi)*cosf(psi),
                      cosf(phi)*cosf(theta) }
                };
                m3_t Ctmp; m3_mul(rot_corr, C[t], Ctmp); memcpy(C[t], Ctmp, sizeof(m3_t));

                // RTS smoother backward pass
                memcpy(dx_rts[t], dx_local, sizeof(dx_local));
                {
                    int32_t k = (int32_t)t - 1;
                    while (k > 0 && samples[k].phase != 0) {
                        m9_t P_reg;
                        memcpy(P_reg, P[k+1], sizeof(m9_t));
                        for (int i = 0; i < 9; i++) P_reg[i][i] += 1e-10f;
                        m9_t P_reg_inv;
                        if (!mat9_invert(P_reg, P_reg_inv)) { k--; continue; }
                        static m9_t Fkp1, Fkp1_T;
                        build_F_from_fg(fg_hist[k+1], dt_hist[k+1], Fkp1);
                        m9_transpose(Fkp1, Fkp1_T);
                        static m9_t PF, G;
                        m9_mul(P[k], Fkp1_T, PF);
                        m9_mul(PF, P_reg_inv, G);

                        float dxr[9];
                        for (int r = 0; r < 9; r++) {
                            float sum = 0.0f;
                            for (int c = 0; c < 9; c++) sum += G[r][c] * dx_rts[k+1][c];
                            dxr[r] = sum;
                        }
                        memcpy(dx_rts[k], dxr, sizeof(dxr));

                        v[k][0] += dxr[3]; v[k][1] += dxr[4]; v[k][2] += dxr[5];
                        s[k][0] += dxr[6]; s[k][1] += dxr[7]; s[k][2] += dxr[8];

                        float dphix = dxr[0], dphiy = dxr[1], dphiz = dxr[2];
                        m3_t phi_m = {
                            { 1.0f,    dphiz, -dphiy },
                            { -dphiz,  1.0f,   dphix },
                            { dphiy,  -dphix,  1.0f  }
                        };
                        m3_t Ck_new; m3_mul(phi_m, C[k], Ck_new); memcpy(C[k], Ck_new, sizeof(m3_t));

                        k--;
                    }
                }

                // P[t] -= Kc @ P[t][idx,:]
                float rowsP[4][9];
                for (int k = 0; k < 4; k++) for (int c = 0; c < 9; c++) rowsP[k][c] = P[t][idx[k]][c];
                for (int r = 0; r < 9; r++)
                    for (int c = 0; c < 9; c++) {
                        float sub = 0.0f;
                        for (int k = 0; k < 4; k++) sub += Kc[r][k]*rowsP[k][c];
                        P[t][r][c] -= sub;
                    }
            }
        }
    }

    for (uint32_t t = 0; t < n; t++) {
        traj_out[t].pos[0]=s[t][0]; traj_out[t].pos[1]=s[t][1]; traj_out[t].pos[2]=s[t][2];
        traj_out[t].vel[0]=v[t][0]; traj_out[t].vel[1]=v[t][1]; traj_out[t].vel[2]=v[t][2];
        traj_out[t].quat = mat3_to_quat(C[t]);
        traj_out[t].stance = (samples[t].phase == 0);
    }

    // No ESKF state is carried into the next overlapping window.
    s_carry_valid = false;
}

// ============================================================================
// STEP 5 (bout-level) — step5_eskf_trajectory_bout()
//
// Runs ESKF over an entire walking bout in ONE pass (no 512-sample windows),
// producing a continuous global pos[] trajectory exactly as Python does in
// error_state_kalman_filter() called once per bout.
//
// WHY THIS IS NEEDED:
//   The streaming step5_eskf_trajectory() resets s[0]=0 at every 512-sample
//   window boundary. When a stride's IC is in window N and its next_IC is in
//   window N+1, pos[next_ic]-pos[ic] crosses a zero-reset boundary and grows
//   without bound (→ 2.5 m cap every stride). Python avoids this entirely by
//   running ESKF once over the full bout. This function does the same.
//
// USAGE (in stridesense_pipeline.c per-bout loop):
//   1. Collect the full bout into bout_samples[0..n_bout-1].
//   2. Run step3_detect_events() over the full bout to set .phase correctly.
//   3. Call step5_eskf_reset_session() to clear carry state.
//   4. Call step5_eskf_trajectory_bout(bout_samples, n_bout, bout_traj, g).
//   5. For each stride in this bout call step6_spatial(bout_samples, n_bout,
//      stride, bout_traj, &spat) — td_sample/td_next_sample index directly
//      into bout_traj which has continuous pos[].
// ============================================================================
void step5_eskf_trajectory_bout(imu_sample_t *samples, uint32_t n,
                                 traj_sample_t *traj_out, float g)
{
    if (n == 0) return;
    if (n > MAX_BOUT_SAMPLES) n = MAX_BOUT_SAMPLES;

    // Same ZUPT sigmas as step5_eskf_trajectory() — match Python's in-function
    // override of module-level constants.
    const float v_meas_sigma = 1e-4f;
    const float z_meas_sigma = 1e-4f;

    // --- Allocate on heap via static to avoid blowing the stack on MCU -------
    // These are large arrays (MAX_BOUT_SAMPLES × 9×9 floats for P alone is
    // ~3.6 MB at 10000 samples — far beyond any MCU stack). Declare static so
    // they live in BSS/data. This means the function is NOT re-entrant, which
    // is fine for the single-threaded bout-processing loop.
    static SHARED_SRAM float    acc_b  [MAX_BOUT_SAMPLES][3];
    static SHARED_SRAM float    gyro_b [MAX_BOUT_SAMPLES][3];
    static SHARED_SRAM float    time_s [MAX_BOUT_SAMPLES];
    static SHARED_SRAM m3_t     C      [MAX_BOUT_SAMPLES];
    static SHARED_SRAM float    v      [MAX_BOUT_SAMPLES][3];
    static SHARED_SRAM float    s      [MAX_BOUT_SAMPLES][3];
    static SHARED_SRAM m9_t     P      [MAX_BOUT_SAMPLES];
    static SHARED_SRAM float    fg_hist[MAX_BOUT_SAMPLES][3];
    static SHARED_SRAM float    dt_hist[MAX_BOUT_SAMPLES];
    static SHARED_SRAM float    dx_rts [MAX_BOUT_SAMPLES][9];

    // Convert raw samples
    for (uint32_t i = 0; i < n; i++) {
        time_s[i]  = samples[i].time_ms / 1000.0f;
        acc_b[i][0] = samples[i].acc_x * g;
        acc_b[i][1] = samples[i].acc_y * g;
        acc_b[i][2] = samples[i].acc_z * g;
        gyro_b[i][0] = samples[i].gyro_x * DEG2RAD_F;
        gyro_b[i][1] = samples[i].gyro_y * DEG2RAD_F;
        gyro_b[i][2] = samples[i].gyro_z * DEG2RAD_F;
    }

    // Gravity calibration — PYTHON PARITY.
    // Python error_state_kalman_filter() recalculates the 20th-percentile
    // quiet-sample gravity scale for EACH bout. Do not reuse the C session-wide
    // carry scale here, because that changes the trajectory between bouts.
    {
        static SHARED_SRAM float gyro_mag_rad[MAX_BOUT_SAMPLES];
        static SHARED_SRAM float sorted_buf[MAX_BOUT_SAMPLES];
        for (uint32_t i = 0; i < n; i++) {
            gyro_mag_rad[i] = sqrtf(gyro_b[i][0]*gyro_b[i][0] +
                                    gyro_b[i][1]*gyro_b[i][1] +
                                    gyro_b[i][2]*gyro_b[i][2]);
            sorted_buf[i] = gyro_mag_rad[i];
        }
        insertion_sort_f(sorted_buf, n);
        float quiet_thresh = percentile_sorted(sorted_buf, n, 20.0f);
        float g_vec[3] = {0,0,0};
        uint32_t quiet_cnt = 0;
        for (uint32_t i = 0; i < n; i++) {
            if (gyro_mag_rad[i] < quiet_thresh) {
                g_vec[0] += acc_b[i][0];
                g_vec[1] += acc_b[i][1];
                g_vec[2] += acc_b[i][2];
                quiet_cnt++;
            }
        }
        if (quiet_cnt >= 20) {
            g_vec[0] /= quiet_cnt;
            g_vec[1] /= quiet_cnt;
            g_vec[2] /= quiet_cnt;
        } else {
            uint32_t m = (n < 200u) ? n : 200u;
            g_vec[0] = g_vec[1] = g_vec[2] = 0.0f;
            for (uint32_t i = 0; i < m; i++) {
                g_vec[0] += acc_b[i][0];
                g_vec[1] += acc_b[i][1];
                g_vec[2] += acc_b[i][2];
            }
            if (m > 0) {
                g_vec[0] /= m; g_vec[1] /= m; g_vec[2] /= m;
            }
        }
        float g_mag = sqrtf(g_vec[0]*g_vec[0] + g_vec[1]*g_vec[1] + g_vec[2]*g_vec[2]);
        if (g_mag > 7.0f && g_mag < 14.0f) {
            float scale = 9.81f / g_mag;
            for (uint32_t i = 0; i < n; i++) {
                acc_b[i][0] *= scale;
                acc_b[i][1] *= scale;
                acc_b[i][2] *= scale;
            }
        }
    }

    // Check that at least one STANCE sample exists
    bool has_stance = false;
    for (uint32_t i = 0; i < n; i++) if (samples[i].phase == 0) { has_stance = true; break; }
    if (!has_stance) {
        simple_trajectory_fallback(samples, n, g, traj_out);
        return;
    }

    // Initialise state arrays
    memset(C,       0, n * sizeof(m3_t));
    memset(v,       0, n * sizeof(v[0]));
    memset(s,       0, n * sizeof(s[0]));
    memset(P,       0, n * sizeof(m9_t));
    memset(fg_hist, 0, n * sizeof(fg_hist[0]));
    memset(dt_hist, 0, n * sizeof(dt_hist[0]));
    memset(dx_rts,  0, n * sizeof(dx_rts[0]));

    float sr = infer_sample_rate_hz(samples, n);
    bool orientation_init = false;

    // ── Forward ESKF pass (identical to step5_eskf_trajectory, no windowing) ──
    for (uint32_t t = 0; t < n; t++) {
        float dt;
        if (t > 0) {
            dt = time_s[t] - time_s[t-1];
            if (!(dt > 1e-6f)) dt = 1.0f / sr;
        } else {
            dt = 1.0f / sr;
        }

        if (!orientation_init) {
            if (t > 0 && samples[t-1].phase == 0) {
                float norm_a = sqrtf(acc_b[t-1][0]*acc_b[t-1][0] +
                                     acc_b[t-1][1]*acc_b[t-1][1] +
                                     acc_b[t-1][2]*acc_b[t-1][2]);
                float a_ref[3] = {
                    acc_b[t-1][0] / (norm_a + 1e-12f),
                    acc_b[t-1][1] / (norm_a + 1e-12f),
                    acc_b[t-1][2] / (norm_a + 1e-12f)
                };
                float up[3] = {0.0f, 1.0f, 0.0f};
                float x_ax[3] = {
                    up[1]*a_ref[2] - up[2]*a_ref[1],
                    up[2]*a_ref[0] - up[0]*a_ref[2],
                    up[0]*a_ref[1] - up[1]*a_ref[0]
                };
                float nx = sqrtf(x_ax[0]*x_ax[0] + x_ax[1]*x_ax[1] + x_ax[2]*x_ax[2]) + 1e-12f;
                x_ax[0] /= nx; x_ax[1] /= nx; x_ax[2] /= nx;
                float y_ax[3] = {
                    a_ref[1]*x_ax[2] - a_ref[2]*x_ax[1],
                    a_ref[2]*x_ax[0] - a_ref[0]*x_ax[2],
                    a_ref[0]*x_ax[1] - a_ref[1]*x_ax[0]
                };
                float ny = sqrtf(y_ax[0]*y_ax[0] + y_ax[1]*y_ax[1] + y_ax[2]*y_ax[2]) + 1e-12f;
                y_ax[0] /= ny; y_ax[1] /= ny; y_ax[2] /= ny;

                C[t-1][0][0]=x_ax[0]; C[t-1][0][1]=x_ax[1]; C[t-1][0][2]=x_ax[2];
                C[t-1][1][0]=y_ax[0]; C[t-1][1][1]=y_ax[1]; C[t-1][1][2]=y_ax[2];
                C[t-1][2][0]=a_ref[0]; C[t-1][2][1]=a_ref[1]; C[t-1][2][2]=a_ref[2];

                P[t-1][0][0] = 1e-3f; P[t-1][1][1] = 1e-3f; P[t-1][2][2] = 1e-10f;
                P[t-1][3][3] = 1e-5f; P[t-1][4][4] = 1e-5f; P[t-1][5][5] = 1e-5f;
                orientation_init = true;
            } else {
                m3_eye(C[t]);
                continue;
            }
        }

        float gv[3] = {gyro_b[t][0], gyro_b[t][1], gyro_b[t][2]};
        float gnorm = sqrtf(gv[0]*gv[0] + gv[1]*gv[1] + gv[2]*gv[2]);
        if (gnorm == 0.0f) { gv[0]=gv[1]=gv[2]=1e-10f; gnorm = sqrtf(3.0f)*1e-10f; }

        float angle = gnorm * dt;
        m3_t Bn; skew3(gv, Bn);
        for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) Bn[i][j] /= gnorm;
        m3_t Bn2; m3_mul(Bn, Bn, Bn2);
        m3_t R_rot;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                R_rot[i][j] = (i==j ? 1.0f : 0.0f) + sinf(angle)*Bn[i][j] + (1.0f-cosf(angle))*Bn2[i][j];

        m3_mul(C[t-1], R_rot, C[t]);

        float fg[3]; m3_mulvec(C[t], acc_b[t], fg);
        float ag[3] = { fg[0], fg[1], fg[2] - g };

        for (int k = 0; k < 3; k++) v[t][k] = v[t-1][k] + ag[k]*dt;
        for (int k = 0; k < 3; k++) s[t][k] = s[t-1][k] + v[t][k]*dt;

        dt_hist[t] = dt;
        fg_hist[t][0] = fg[0]; fg_hist[t][1] = fg[1]; fg_hist[t][2] = fg[2];

        static m9_t F_t;
        build_F_from_fg(fg, dt, F_t);
        static m9_t FP, Ft, FPFt;
        m9_mul(F_t, P[t-1], FP);
        m9_transpose(F_t, Ft);
        m9_mul(FP, Ft, FPFt);

        float Tc = dt*dt*ESKF_GYRO_SIGMA*ESKF_GYRO_SIGMA;
        float Uc = dt*dt*ESKF_ACC_SIGMA*ESKF_ACC_SIGMA;
        for (int i = 0; i < 3; i++) FPFt[i][i]   += Tc;
        for (int i = 0; i < 3; i++) FPFt[3+i][3+i] += Uc;
        memcpy(P[t], FPFt, sizeof(m9_t));

        if (samples[t].phase == 0) { // STANCE — ZUPT correction
            const uint32_t idx[4] = {3,4,5,8};
            float S4[4][4];
            for (int a = 0; a < 4; a++) for (int b = 0; b < 4; b++) S4[a][b] = P[t][idx[a]][idx[b]];
            float sigma2  = v_meas_sigma * v_meas_sigma;
            float zsigma2 = z_meas_sigma * z_meas_sigma;
            S4[0][0] += sigma2; S4[1][1] += sigma2; S4[2][2] += sigma2; S4[3][3] += zsigma2;

            float Minv[4][4];
            if (mat4_invert(S4, Minv)) {
                float y[4] = { -v[t][0], -v[t][1], -v[t][2], -s[t][2] };
                float Kc[9][4];
                for (int r = 0; r < 9; r++)
                    for (int c = 0; c < 4; c++) {
                        float sum = 0.0f;
                        for (int k = 0; k < 4; k++) sum += P[t][r][idx[k]] * Minv[k][c];
                        Kc[r][c] = sum;
                    }
                float dx_local[9];
                for (int r = 0; r < 9; r++) {
                    float sum = 0.0f;
                    for (int k = 0; k < 4; k++) sum += Kc[r][k]*y[k];
                    dx_local[r] = sum;
                }
                v[t][0]+=dx_local[3]; v[t][1]+=dx_local[4]; v[t][2]+=dx_local[5];
                s[t][0]+=dx_local[6]; s[t][1]+=dx_local[7]; s[t][2]+=dx_local[8];

                float phi = dx_local[0], theta = dx_local[1], psi = dx_local[2];
                m3_t rot_corr = {
                    { cosf(theta)*cosf(psi),  cosf(theta)*sinf(psi), -sinf(theta) },
                    { sinf(phi)*sinf(theta)*cosf(psi) - cosf(phi)*sinf(psi),
                      sinf(phi)*sinf(theta)*sinf(psi) + cosf(phi)*cosf(psi),
                      sinf(phi)*cosf(theta) },
                    { cosf(phi)*sinf(theta)*cosf(psi) + sinf(phi)*sinf(psi),
                      cosf(phi)*sinf(theta)*sinf(psi) - sinf(phi)*cosf(psi),
                      cosf(phi)*cosf(theta) }
                };
                m3_t Ctmp; m3_mul(rot_corr, C[t], Ctmp); memcpy(C[t], Ctmp, sizeof(m3_t));

                // RTS smoother backward pass
                memcpy(dx_rts[t], dx_local, sizeof(dx_local));
                {
                    int32_t k = (int32_t)t - 1;
                    while (k > 0 && samples[k].phase != 0) {
                        m9_t P_reg;
                        memcpy(P_reg, P[k+1], sizeof(m9_t));
                        for (int i = 0; i < 9; i++) P_reg[i][i] += 1e-10f;
                        m9_t P_reg_inv;
                        if (!mat9_invert(P_reg, P_reg_inv)) { k--; continue; }
                        static m9_t Fkp1, Fkp1_T;
                        build_F_from_fg(fg_hist[k+1], dt_hist[k+1], Fkp1);
                        m9_transpose(Fkp1, Fkp1_T);
                        static m9_t PF, G;
                        m9_mul(P[k], Fkp1_T, PF);
                        m9_mul(PF, P_reg_inv, G);
                        float dxr[9];
                        for (int r = 0; r < 9; r++) {
                            float sum = 0.0f;
                            for (int c = 0; c < 9; c++) sum += G[r][c] * dx_rts[k+1][c];
                            dxr[r] = sum;
                        }
                        memcpy(dx_rts[k], dxr, sizeof(dxr));
                        v[k][0] += dxr[3]; v[k][1] += dxr[4]; v[k][2] += dxr[5];
                        s[k][0] += dxr[6]; s[k][1] += dxr[7]; s[k][2] += dxr[8];
                        float dphix = dxr[0], dphiy = dxr[1], dphiz = dxr[2];
                        m3_t phi_m = {
                            { 1.0f,    dphiz, -dphiy },
                            { -dphiz,  1.0f,   dphix },
                            { dphiy,  -dphix,  1.0f  }
                        };
                        m3_t Ck_new; m3_mul(phi_m, C[k], Ck_new); memcpy(C[k], Ck_new, sizeof(m3_t));
                        k--;
                    }
                }

                // P[t] -= Kc @ P[t][idx,:]
                float rowsP[4][9];
                for (int k = 0; k < 4; k++) for (int c = 0; c < 9; c++) rowsP[k][c] = P[t][idx[k]][c];
                for (int r = 0; r < 9; r++)
                    for (int c = 0; c < 9; c++) {
                        float sub = 0.0f;
                        for (int k = 0; k < 4; k++) sub += Kc[r][k]*rowsP[k][c];
                        P[t][r][c] -= sub;
                    }
            }
        }
    }

    // Write output trajectory
    for (uint32_t t = 0; t < n; t++) {
        traj_out[t].pos[0] = s[t][0];
        traj_out[t].pos[1] = s[t][1];
        traj_out[t].pos[2] = s[t][2];
        traj_out[t].vel[0] = v[t][0];
        traj_out[t].vel[1] = v[t][1];
        traj_out[t].vel[2] = v[t][2];
        traj_out[t].quat   = mat3_to_quat(C[t]);
        traj_out[t].stance = (samples[t].phase == 0);
    }
    // No carry state saved — each bout is independent, matching Python's
    // per-bout ESKF reset before error_state_kalman_filter() is called.
}

// ============================================================================
// STEP 6 — compute_spatial_parameters() / compute_ilr() /
//          compute_pronation_angle() / compute_angle_change()
// ============================================================================

void step6_spatial(imu_sample_t *samples, uint32_t n,
                    stride_t *stride, traj_sample_t *traj,
                    spatial_params_t *out) {
    uint32_t ic = stride->td_sample;
    uint32_t next_ic = stride->td_next_sample;
    if (next_ic >= n || ic >= n) { memset(out, 0, sizeof(*out)); return; }

    // ── Stride length: Python compute_spatial_parameters() ────────────────
    // raw_len = hypot(pos_x[next_ic]-pos_x[ic], pos_y[next_ic]-pos_y[ic])
    // stride_length_m = round(raw_len * 3.3, 4)
    //
    // PYTHON PARITY: there is NO 0.20–2.50 m sanity gate here and no
    // boundary-imputed special case, because Python does not create those
    // synthetic strides in the first place.
    float dx = traj[next_ic].pos[0] - traj[ic].pos[0];
    float dy = traj[next_ic].pos[1] - traj[ic].pos[1];
    float raw_len = sqrtf(dx*dx + dy*dy);
    float sl = raw_len * STRIDE_SPATIAL_SCALE;
    out->stride_length_m = isfinite(sl) ? roundf(sl * 10000.0f) / 10000.0f : 0.0f;

    // ── Clearance: ESKF pos[2] over swing phase ────────────────────────────────────
    //
    // Matches Python compute_spatial_parameters():
    //   z = pos_z[fo:next_ic] - pos_z[fo]      // normalize to toe-off height
    //   z_swing = z[1:-1]  (exclude TO and touchdown endpoints)
    //   min_clear = max(0, min(z_swing))
    //   max_clear = max(0, max(z_swing))
    //   vertical_osc = max_clear - min_clear
    //
    // Physiological norms (foot-mounted IMU, normal walking):
    //   min_clearance : 0.01 – 0.02 m
    //   max_clearance : 0.15 – 0.25 m
    //   vertical_osc  : 6   – 10 cm
    // ──────────────────────────────────────────────────────────────────────

    uint32_t fo = stride->to_sample;
    if (fo >= next_ic || fo >= n || traj == NULL) {
        out->min_clearance_m = 0.0f;
        out->max_clearance_m = 0.0f;
        out->vertical_osc_cm = 0.0f;
        return;
    }

    uint32_t swing_len = next_ic - fo;   // number of samples [fo, next_ic)

    if (swing_len <= 2) {
        // Not enough interior samples for clearance.
        out->min_clearance_m = 0.0f;
        out->max_clearance_m = 0.0f;
        out->vertical_osc_cm = 0.0f;
        return;
    }

    // Normalize to toe-off height: z[i] = traj[fo+i].pos[2] - traj[fo].pos[2]
    // Python: z = z - z[0]
    float z0 = traj[fo].pos[2];

    // Python: z_swing = z[1:-1]  (exclude first and last sample of swing)
    // Iterate from fo+1 to next_ic-2 inclusive (inner slice).
    uint32_t sw_start = fo + 1;
    uint32_t sw_end   = (next_ic >= 1) ? (next_ic - 1) : next_ic;  // exclusive upper bound

    if (sw_start >= sw_end) {
        // Degenerate: no interior samples; use full window fo+1..next_ic.
        sw_end = next_ic;
        if (sw_start >= sw_end) {
            out->min_clearance_m = 0.0f;
            out->max_clearance_m = 0.0f;
            out->vertical_osc_cm = 0.0f;
            return;
        }
    }

    // PYTHON-PARITY FIX: Match Python compute_spatial_parameters() exactly.
    // Python uses find_peaks with prominence=CLEARANCE_PROMINENCE=0.002 to
    // select peaks in z_swing, with fallback to plain min/max.
    //
    // Python logic:
    //   min_peaks from find_peaks(-z_swing, prominence=0.002)
    //   min_clear = z_swing[min_peaks[-1]] if len(min_peaks) else min(z_swing)
    //   min_clear = max(min_clear, 0.0)
    //   max_peaks from find_peaks(z_swing, prominence=0.002)
    //   max_clear = mean(z_swing[max_peaks]) if len==2
    //            or max(z_swing[max_peaks])  if len>0
    //            or max(z_swing)             fallback
    //   max_clear = max(max_clear, 0.0)
    //   vertical_osc_cm = (max_clear - min_clear) * 100
    //
    // Build z_swing array first (normalized to z0):
    static SHARED_SRAM float z_swing_buf[MAX_SAMPLES];
    uint32_t z_swing_len = 0;
    for (uint32_t i = sw_start; i < sw_end && i < n; i++) {
        z_swing_buf[z_swing_len++] = traj[i].pos[2] - z0;
    }
    if (z_swing_len == 0) {
        out->min_clearance_m = 0.0f;
        out->max_clearance_m = 0.0f;
        out->vertical_osc_cm = 0.0f;
        return;
    }

    // -- min_clear: prominence-peak of -z_swing, fallback to plain min
    // Find local minima of z_swing (= local maxima of -z_swing) with prom >= 0.002
    static SHARED_SRAM float neg_z_swing_buf[MAX_SAMPLES];
    for (uint32_t i = 0; i < z_swing_len; i++) neg_z_swing_buf[i] = -z_swing_buf[i];

    static SHARED_SRAM uint32_t min_peaks[MAX_SAMPLES];
    uint32_t n_min_peaks = find_peaks_ex(neg_z_swing_buf, z_swing_len,
                                          false, 0.0f, 1.0f,
                                          true, CLEARANCE_PROMINENCE,
                                          min_peaks, MAX_SAMPLES);

    float min_clear;
    if (n_min_peaks > 0) {
        // Python: z_swing[min_peaks[-1]] — use the last (rightmost) minimum peak
        min_clear = z_swing_buf[min_peaks[n_min_peaks - 1]];
    } else {
        // Fallback: plain min of z_swing
        min_clear = z_swing_buf[0];
        for (uint32_t i = 1; i < z_swing_len; i++)
            if (z_swing_buf[i] < min_clear) min_clear = z_swing_buf[i];
    }
    if (min_clear < 0.0f) min_clear = 0.0f;

    // -- max_clear: prominence-peak of z_swing
    static SHARED_SRAM uint32_t max_peaks[MAX_SAMPLES];
    uint32_t n_max_peaks = find_peaks_ex(z_swing_buf, z_swing_len,
                                          false, 0.0f, 1.0f,
                                          true, CLEARANCE_PROMINENCE,
                                          max_peaks, MAX_SAMPLES);

    float max_clear;
    if (n_max_peaks == 2) {
        // Python: mean of the two peaks
        max_clear = (z_swing_buf[max_peaks[0]] + z_swing_buf[max_peaks[1]]) * 0.5f;
    } else if (n_max_peaks > 0) {
        // Python: max of all found peaks
        max_clear = z_swing_buf[max_peaks[0]];
        for (uint32_t i = 1; i < n_max_peaks; i++)
            if (z_swing_buf[max_peaks[i]] > max_clear) max_clear = z_swing_buf[max_peaks[i]];
    } else {
        // Fallback: plain max of z_swing
        max_clear = z_swing_buf[0];
        for (uint32_t i = 1; i < z_swing_len; i++)
            if (z_swing_buf[i] > max_clear) max_clear = z_swing_buf[i];
    }
    if (max_clear < 0.0f) max_clear = 0.0f;

    float vertical_osc = max_clear - min_clear;

    out->min_clearance_m = min_clear;
    out->max_clearance_m = max_clear;
    out->vertical_osc_cm = fabsf(vertical_osc) * 100.0f;
}


// ────────────────────────────────────────────────────────────────────────────
// Final clearance pass. The active Python pipeline replaces the trajectory
// clearance values with this gyro-magnitude proxy after spatial extraction.
// ────────────────────────────────────────────────────────────────────────────
void step6_clearance_from_gyro(imu_sample_t *samples, uint32_t n,
                                stride_t *stride, spatial_params_t *out) {
    /*
     * PYTHON PARITY: this is the active compute_clearance_from_acc() in
     * gait_params 6-1.py.  It uses gyro magnitude during swing as a clearance
     * proxy and overwrites the ESKF-Z clearance values produced earlier by
     * compute_spatial_parameters().
     */
    uint32_t fo = stride->to_sample;
    uint32_t next_ic = stride->td_next_sample;
    if (fo >= n || next_ic >= n || next_ic <= fo + 2u) {
        out->min_clearance_m = 0.0f;
        out->max_clearance_m = 0.0f;
        out->vertical_osc_cm = 0.0f;
        return;
    }

    uint32_t swing_len = next_ic - fo;
    uint32_t third = swing_len / 3u;
    if (third < 1u) third = 1u;

    float peak_gm = 0.0f;
    for (uint32_t i = fo; i < next_ic; i++) {
        float gm = sqrtf(samples[i].gyro_x*samples[i].gyro_x +
                         samples[i].gyro_y*samples[i].gyro_y +
                         samples[i].gyro_z*samples[i].gyro_z);
        if (gm > peak_gm) peak_gm = gm;
    }

    float min_gm = 0.0f;
    for (uint32_t i = 0; i < third; i++) {
        float gm = sqrtf(samples[fo+i].gyro_x*samples[fo+i].gyro_x +
                         samples[fo+i].gyro_y*samples[fo+i].gyro_y +
                         samples[fo+i].gyro_z*samples[fo+i].gyro_z);
        if (i == 0u || gm < min_gm) min_gm = gm;
    }

    const float gyro_to_m = 0.0002f;
    out->max_clearance_m = roundf(peak_gm * gyro_to_m * 10000.0f) / 10000.0f;
    out->min_clearance_m = roundf(min_gm  * gyro_to_m * 10000.0f) / 10000.0f;
    out->vertical_osc_cm = roundf((peak_gm - min_gm) * gyro_to_m * 100.0f * 10000.0f) / 10000.0f;
}

void step6_ilr(imu_sample_t *samples, uint32_t n, stride_t *stride, spatial_params_t *out) {
    /* PYTHON PARITY: use the recorded timestamps, not a fixed 1/SR dt. */
    const float window_ms = ILR_WINDOW_MS;
    uint32_t td = stride->td_sample;
    if (td >= n) { out->ilr_g_per_ms = 0.010f; return; }

    float touchdown_ms = samples[td].time_ms;
    uint32_t i_end = td + 2u;
    while (i_end < n && samples[i_end].time_ms <= touchdown_ms + window_ms) i_end++;
    if (i_end < td + 2u) i_end = td + 2u;
    if (i_end > n) i_end = n;

    if (i_end > td + 1u) {
        float prev_mag = sqrtf(samples[td].acc_x*samples[td].acc_x +
                               samples[td].acc_y*samples[td].acc_y +
                               samples[td].acc_z*samples[td].acc_z);
        float max_ilr = 0.0f;
        for (uint32_t i = td + 1u; i < i_end; i++) {
            float mag = sqrtf(samples[i].acc_x*samples[i].acc_x +
                              samples[i].acc_y*samples[i].acc_y +
                              samples[i].acc_z*samples[i].acc_z);
            float dt_ms = samples[i].time_ms - samples[i-1u].time_ms;
            float rate = (dt_ms > 0.0f) ? (mag - prev_mag) / dt_ms : 0.0f;
            if (rate > max_ilr) max_ilr = rate;
            prev_mag = mag;
        }
        float ilr = (max_ilr > 0.001f) ? max_ilr : 0.001f;
        out->ilr_g_per_ms = roundf(ilr * 10000.0f) / 10000.0f;
    } else {
        out->ilr_g_per_ms = 0.0100f;
    }
}

void step6_pronation(imu_sample_t *samples, uint32_t n, stride_t *stride, spatial_params_t *out) {
    float sr = infer_sample_rate_hz(samples, n);
    float dt = 1.0f / sr;

    uint32_t td = stride->td_sample;
    uint32_t to = stride->to_sample;
    if (n == 0) { out->pronation_angle_deg = 0.0f; return; }
    if (td > n - 1) td = n - 1;
    if (to < td + 1) to = td + 1;
    if (to > n) to = n;

    float sum = 0.0f;
    for (uint32_t i = td; i < to && i < n; i++) sum += samples[i].gyro_x;
    out->pronation_angle_deg = sum * dt;
}

void step6_angle_change(traj_sample_t *traj, uint32_t n, stride_t *stride, spatial_params_t *out) {
    if (n == 0) { out->angle_change = 0.0f; return; }
    uint32_t i_start = stride->td_sample;
    uint32_t i_end   = stride->td_next_sample;
    if (i_start > n - 1) i_start = n - 1;
    if (i_end   > n - 1) i_end   = n - 1;

    if (i_end > i_start) {
        // q_rel = q_end * conj(q_start); angle = 2 * arccos(|q_rel.w|)
        // clipped to [0, pi] (matches Python gait_params_4.py).
        float sw = traj[i_start].quat.w;
        float sx = traj[i_start].quat.x;
        float sy = traj[i_start].quat.y;
        float sz = traj[i_start].quat.z;
        float ew = traj[i_end].quat.w;
        float ex = traj[i_end].quat.x;
        float ey = traj[i_end].quat.y;
        float ez = traj[i_end].quat.z;

        float rw =  ew*sw + ex*sx + ey*sy + ez*sz;
        float rx = -ew*sx + ex*sw - ey*sz + ez*sy;
        float ry = -ew*sy + ex*sz + ey*sw - ez*sx;
        float rz = -ew*sz - ex*sy + ey*sx + ez*sw;

        float rw_clamped = rw;
        if (rw_clamped >  1.0f) rw_clamped =  1.0f;
        if (rw_clamped < -1.0f) rw_clamped = -1.0f;
        float angle = 2.0f * acosf(fabsf(rw_clamped));
        if (angle < 0.0f)                angle = 0.0f;
        if (angle > 3.14159265358979f)   angle = 3.14159265358979f;

        (void)rx; (void)ry; (void)rz;

        out->angle_change = angle;
    } else {
        out->angle_change = 0.0f;
    }
}

// Circular buffer implementation for real-time streaming
void cb_init(circular_buffer_t *cb, imu_sample_t *buffer, uint32_t size) {
    cb->head = 0;
    cb->tail = 0;
    cb->count = 0;
    cb->max_size = size;
    cb->buffer = buffer;
}

uint32_t cb_push(circular_buffer_t *cb, imu_sample_t *sample) {
    cb->buffer[cb->head] = *sample;
    cb->head = (cb->head + 1) % cb->max_size;
    if (cb->count < cb->max_size) {
        cb->count++;
    } else {
        cb->tail = (cb->tail + 1) % cb->max_size;
    }
    return cb->count;
}

uint32_t cb_get_count(circular_buffer_t *cb) {
    return cb->count;
}

void cb_get_window(circular_buffer_t *cb, imu_sample_t *out, uint32_t n) {
    uint32_t idx = cb->tail;
    for (uint32_t i = 0; i < n && i < cb->count; i++) {
        out[i] = cb->buffer[idx];
        idx = (idx + 1) % cb->max_size;
    }
}

// Feature vector extraction for ML inference
void extract_feature_vector(temporal_params_t *tp, spatial_params_t *sp, feature_vector_t *fv) {
    fv->gct_ms = tp->gct_ms;
    fv->stride_time_ms = tp->stride_time_ms;
    fv->stance_time_ms = tp->stance_time_ms;
    fv->swing_time_ms = tp->swing_time_ms;
    fv->stance_ratio = tp->stance_ratio;
    fv->cadence_spm = tp->cadence_spm;
    fv->speed_m_s = tp->speed_m_s;

    fv->stride_length_m = sp->stride_length_m;
    fv->min_clearance_m = sp->min_clearance_m;
    fv->max_clearance_m = sp->max_clearance_m;
    fv->vertical_osc_cm = sp->vertical_osc_cm;
    fv->ilr_g_per_ms = sp->ilr_g_per_ms;
    fv->pronation_angle_deg = sp->pronation_angle_deg;

    fv->cadence_cv = 0.0f;
    fv->stride_length_cv = 0.0f;
    fv->gct_cv = 0.0f;
    fv->stride_time_cv = 0.0f;
    fv->speed_cv = 0.0f;
}

void extract_feature_vector_batch(stride_t *strides, uint32_t n_strides,
                                  temporal_params_t *tp_arr, spatial_params_t *sp_arr,
                                  feature_vector_t *fv_out, uint32_t max_fv) {
    if (n_strides == 0 || max_fv == 0) return;

    uint32_t n_fv = (n_strides < max_fv) ? n_strides : max_fv;

    for (uint32_t i = 0; i < n_fv; i++) {
        extract_feature_vector(&tp_arr[i], &sp_arr[i], &fv_out[i]);
    }

    if (n_fv > 1) {
        float mean_cadence = 0, mean_stride_len = 0, mean_gct = 0;
        float mean_stride_time = 0, mean_speed = 0;

        for (uint32_t i = 0; i < n_fv; i++) {
            mean_cadence += fv_out[i].cadence_spm;
            mean_stride_len += fv_out[i].stride_length_m;
            mean_gct += fv_out[i].gct_ms;
            mean_stride_time += fv_out[i].stride_time_ms;
            mean_speed += fv_out[i].speed_m_s;
        }

        mean_cadence /= n_fv;
        mean_stride_len /= n_fv;
        mean_gct /= n_fv;
        mean_stride_time /= n_fv;
        mean_speed /= n_fv;

        float std_cadence = 0, std_stride_len = 0, std_gct = 0;
        float std_stride_time = 0, std_speed = 0;

        for (uint32_t i = 0; i < n_fv; i++) {
            std_cadence += (fv_out[i].cadence_spm - mean_cadence) * (fv_out[i].cadence_spm - mean_cadence);
            std_stride_len += (fv_out[i].stride_length_m - mean_stride_len) * (fv_out[i].stride_length_m - mean_stride_len);
            std_gct += (fv_out[i].gct_ms - mean_gct) * (fv_out[i].gct_ms - mean_gct);
            std_stride_time += (fv_out[i].stride_time_ms - mean_stride_time) * (fv_out[i].stride_time_ms - mean_stride_time);
            std_speed += (fv_out[i].speed_m_s - mean_speed) * (fv_out[i].speed_m_s - mean_speed);
        }

        std_cadence = sqrtf(std_cadence / n_fv);
        std_stride_len = sqrtf(std_stride_len / n_fv);
        std_gct = sqrtf(std_gct / n_fv);
        std_stride_time = sqrtf(std_stride_time / n_fv);
        std_speed = sqrtf(std_speed / n_fv);

        float cv_cadence = (mean_cadence != 0.0f) ? (std_cadence / mean_cadence * 100.0f) : 0.0f;
        float cv_stride_len = (mean_stride_len != 0.0f) ? (std_stride_len / mean_stride_len * 100.0f) : 0.0f;
        float cv_gct = (mean_gct != 0.0f) ? (std_gct / mean_gct * 100.0f) : 0.0f;
        float cv_stride_time = (mean_stride_time != 0.0f) ? (std_stride_time / mean_stride_time * 100.0f) : 0.0f;
        float cv_speed = (mean_speed != 0.0f) ? (std_speed / mean_speed * 100.0f) : 0.0f;

        for (uint32_t i = 0; i < n_fv; i++) {
            fv_out[i].cadence_cv = cv_cadence;
            fv_out[i].stride_length_cv = cv_stride_len;
            fv_out[i].gct_cv = cv_gct;
            fv_out[i].stride_time_cv = cv_stride_time;
            fv_out[i].speed_cv = cv_speed;
        }
    }
}

// ============================================================================
// SESSION AGGREGATION FUNCTIONS (matches Python aggregation.py)
// ============================================================================

void aggregate_session(temporal_params_t *temp_arr, spatial_params_t *spat_arr,
                      uint32_t n_strides, session_stats_t *stats)
{
    aggregate_session_ex(temp_arr, spat_arr, NULL, n_strides, false, 0, 0, stats);
}

void aggregate_session_ex(temporal_params_t *temp_arr, spatial_params_t *spat_arr,
                          const bool *is_outlier, uint32_t n_strides,
                          bool hw_stride_count_available,
                          uint32_t hw_step_count, uint32_t hw_stride_count,
                          session_stats_t *stats)
{
    if (n_strides == 0 || temp_arr == NULL || spat_arr == NULL) {
        memset(stats, 0, sizeof(session_stats_t));
        stats->n_strides = 0;
        return;
    }

    static SHARED_SRAM uint32_t valid_idx[MAX_SESSION_STRIDES];
    uint32_t n_valid = 0;
    if (is_outlier != NULL) {
        for (uint32_t i = 0; i < n_strides && n_valid < MAX_SESSION_STRIDES; i++) {
            if (!is_outlier[i]) valid_idx[n_valid++] = i;
        }
    }
    if (n_valid == 0) {
        n_valid = (n_strides < MAX_SESSION_STRIDES) ? n_strides : MAX_SESSION_STRIDES;
        for (uint32_t i = 0; i < n_valid; i++) valid_idx[i] = i;
    }

    static SHARED_SRAM temporal_params_t vtemp[MAX_SESSION_STRIDES];
    static SHARED_SRAM spatial_params_t  vspat[MAX_SESSION_STRIDES];
    for (uint32_t i = 0; i < n_valid; i++) {
        vtemp[i] = temp_arr[valid_idx[i]];
        vspat[i] = spat_arr[valid_idx[i]];
    }
    temp_arr = vtemp;
    spat_arr = vspat;
    n_strides = n_valid;

    stats->n_strides = n_strides;
    stats->hw_stride_count_available = hw_stride_count_available;
    stats->hw_step_count = hw_step_count;
    stats->hw_stride_count = hw_stride_count;

    float sum_gct = 0, sum_stance_time = 0, sum_stride_time = 0;
    float sum_swing_time = 0, sum_stance_ratio = 0, sum_cadence = 0, sum_speed = 0;

    float sum_stride_len = 0, sum_min_clearance = 0, sum_max_clearance = 0;
    float sum_vert_osc = 0, sum_ilr = 0, sum_pronation = 0, sum_angle_change = 0;

    float sum_energy = 0;

    for (uint32_t i = 0; i < n_strides; i++) {
        sum_gct += temp_arr[i].gct_ms;
        sum_stance_time += temp_arr[i].stance_time_ms;
        sum_stride_time += temp_arr[i].stride_time_ms;
        sum_swing_time += temp_arr[i].swing_time_ms;
        sum_stance_ratio += temp_arr[i].stance_ratio;
        sum_cadence += temp_arr[i].cadence_spm;
        sum_speed += temp_arr[i].speed_m_s;

        sum_stride_len += spat_arr[i].stride_length_m;
        sum_min_clearance += spat_arr[i].min_clearance_m;
        sum_max_clearance += spat_arr[i].max_clearance_m;
        sum_vert_osc += spat_arr[i].vertical_osc_cm;
        sum_ilr += spat_arr[i].ilr_g_per_ms;
        sum_pronation += spat_arr[i].pronation_angle_deg;
        sum_angle_change += spat_arr[i].angle_change;

        float st_s = temp_arr[i].stride_time_ms / 1000.0f;
        sum_energy += (st_s > 0.0f) ? (spat_arr[i].stride_length_m / st_s) : 0.0f;
    }

    stats->mean_gct_ms = sum_gct / n_strides;
    stats->mean_stance_time_ms = sum_stance_time / n_strides;
    stats->mean_stride_time_ms = sum_stride_time / n_strides;
    stats->mean_swing_time_ms = sum_swing_time / n_strides;
    stats->mean_stance_ratio = sum_stance_ratio / n_strides;
    stats->mean_cadence_spm = sum_cadence / n_strides;
    stats->mean_speed_m_s = sum_speed / n_strides;

    stats->mean_stride_length_m = sum_stride_len / n_strides;
    stats->mean_min_clearance_m = sum_min_clearance / n_strides;
    stats->mean_max_clearance_m = sum_max_clearance / n_strides;
    stats->mean_vertical_osc_cm = sum_vert_osc / n_strides;
    stats->mean_ilr_g_per_ms = sum_ilr / n_strides;
    stats->mean_pronation_angle_deg = sum_pronation / n_strides;
    stats->mean_angle_change = sum_angle_change / n_strides;
    stats->mean_energy = sum_energy / n_strides;

    uint32_t ddof_denom = (n_strides > 1) ? (n_strides - 1) : 1;

    float std_gct = 0, std_stride_time = 0, std_stance_time = 0, std_cadence = 0;
    float std_swing_time = 0, std_stance_ratio = 0, std_speed = 0;
    float std_stride_len = 0, std_min_clearance = 0, std_max_clearance = 0;
    float std_vert_osc = 0, std_ilr = 0, std_pronation = 0, std_angle_change = 0;
    float std_energy = 0;

    for (uint32_t i = 0; i < n_strides; i++) {
        float st_s = temp_arr[i].stride_time_ms / 1000.0f;
        float energy_i = (st_s > 0.0f) ? (spat_arr[i].stride_length_m / st_s) : 0.0f;

        std_gct += (temp_arr[i].gct_ms - stats->mean_gct_ms) * (temp_arr[i].gct_ms - stats->mean_gct_ms);
        std_stride_time += (temp_arr[i].stride_time_ms - stats->mean_stride_time_ms) * (temp_arr[i].stride_time_ms - stats->mean_stride_time_ms);
        std_stance_time += (temp_arr[i].stance_time_ms - stats->mean_stance_time_ms) * (temp_arr[i].stance_time_ms - stats->mean_stance_time_ms);
        std_cadence += (temp_arr[i].cadence_spm - stats->mean_cadence_spm) * (temp_arr[i].cadence_spm - stats->mean_cadence_spm);
        std_swing_time += (temp_arr[i].swing_time_ms - stats->mean_swing_time_ms) * (temp_arr[i].swing_time_ms - stats->mean_swing_time_ms);
        std_stance_ratio += (temp_arr[i].stance_ratio - stats->mean_stance_ratio) * (temp_arr[i].stance_ratio - stats->mean_stance_ratio);
        std_speed += (temp_arr[i].speed_m_s - stats->mean_speed_m_s) * (temp_arr[i].speed_m_s - stats->mean_speed_m_s);

        std_stride_len += (spat_arr[i].stride_length_m - stats->mean_stride_length_m) * (spat_arr[i].stride_length_m - stats->mean_stride_length_m);
        std_min_clearance += (spat_arr[i].min_clearance_m - stats->mean_min_clearance_m) * (spat_arr[i].min_clearance_m - stats->mean_min_clearance_m);
        std_max_clearance += (spat_arr[i].max_clearance_m - stats->mean_max_clearance_m) * (spat_arr[i].max_clearance_m - stats->mean_max_clearance_m);
        std_vert_osc += (spat_arr[i].vertical_osc_cm - stats->mean_vertical_osc_cm) * (spat_arr[i].vertical_osc_cm - stats->mean_vertical_osc_cm);
        std_ilr += (spat_arr[i].ilr_g_per_ms - stats->mean_ilr_g_per_ms) * (spat_arr[i].ilr_g_per_ms - stats->mean_ilr_g_per_ms);
        std_pronation += (spat_arr[i].pronation_angle_deg - stats->mean_pronation_angle_deg) * (spat_arr[i].pronation_angle_deg - stats->mean_pronation_angle_deg);
        std_angle_change += (spat_arr[i].angle_change - stats->mean_angle_change) * (spat_arr[i].angle_change - stats->mean_angle_change);
        std_energy += (energy_i - stats->mean_energy) * (energy_i - stats->mean_energy);
    }

    std_gct = sqrtf(std_gct / ddof_denom);
    std_stride_time = sqrtf(std_stride_time / ddof_denom);
    std_stance_time = sqrtf(std_stance_time / ddof_denom);
    std_cadence = sqrtf(std_cadence / ddof_denom);
    std_swing_time = sqrtf(std_swing_time / ddof_denom);
    std_stance_ratio = sqrtf(std_stance_ratio / ddof_denom);
    std_speed = sqrtf(std_speed / ddof_denom);
    std_stride_len = sqrtf(std_stride_len / ddof_denom);
    std_min_clearance = sqrtf(std_min_clearance / ddof_denom);
    std_max_clearance = sqrtf(std_max_clearance / ddof_denom);
    std_vert_osc = sqrtf(std_vert_osc / ddof_denom);
    std_ilr = sqrtf(std_ilr / ddof_denom);
    std_pronation = sqrtf(std_pronation / ddof_denom);
    std_angle_change = sqrtf(std_angle_change / ddof_denom);
    std_energy = sqrtf(std_energy / ddof_denom);

    // CV = std / |mean| * 100. Using fabsf() prevents nonsense negatives
    // when the mean is signed (e.g. pronation angle mean = -32.8 deg was
    // previously yielding CV = -331%). CV is a magnitude by definition.
    #define SAFE_CV(std_val, mean_val)  \
        ((fabsf(mean_val) > 1e-9f) ? ((std_val) / fabsf(mean_val) * 100.0f) : 0.0f)

    stats->cv_gct_ms              = SAFE_CV(std_gct,           stats->mean_gct_ms);
    stats->cv_stride_time_ms      = SAFE_CV(std_stride_time,   stats->mean_stride_time_ms);
    stats->cv_stance_time_ms      = SAFE_CV(std_stance_time,   stats->mean_stance_time_ms);
    stats->cv_cadence_spm         = SAFE_CV(std_cadence,       stats->mean_cadence_spm);
    stats->cv_swing_time_ms       = SAFE_CV(std_swing_time,    stats->mean_swing_time_ms);
    stats->cv_stance_ratio        = SAFE_CV(std_stance_ratio,  stats->mean_stance_ratio);
    stats->cv_speed_m_s           = SAFE_CV(std_speed,         stats->mean_speed_m_s);

    stats->cv_stride_length_m     = SAFE_CV(std_stride_len,    stats->mean_stride_length_m);
    stats->cv_min_clearance_m     = SAFE_CV(std_min_clearance, stats->mean_min_clearance_m);
    stats->cv_max_clearance_m     = SAFE_CV(std_max_clearance, stats->mean_max_clearance_m);
    stats->cv_vertical_osc_cm     = SAFE_CV(std_vert_osc,      stats->mean_vertical_osc_cm);
    stats->cv_ilr_g_per_ms        = SAFE_CV(std_ilr,           stats->mean_ilr_g_per_ms);
    stats->cv_pronation_angle_deg = SAFE_CV(std_pronation,     stats->mean_pronation_angle_deg);
    stats->cv_angle_change        = SAFE_CV(std_angle_change,  stats->mean_angle_change);
    stats->cv_energy              = SAFE_CV(std_energy,        stats->mean_energy);

    #undef SAFE_CV

    if (n_strides >= 2) {
        stats->stride_variability_sd_ms  = std_stride_time;
        float sv_mean = stats->mean_stride_time_ms;
        stats->stride_variability_cv_pct = (sv_mean > 0.0f) ? (std_stride_time / sv_mean * 100.0f) : 0.0f;
    } else {
        stats->stride_variability_sd_ms  = 0.0f;
        stats->stride_variability_cv_pct = 0.0f;
    }
}

// Helper function to check if a float value is valid (not NaN or infinity)
static bool is_valid_float(float f)
{
    volatile uint32_t *p = (volatile uint32_t *)&f;
    return (*p & 0x7F800000) != 0x7F800000;
}

// If the value is invalid (NaN/Inf) or exactly zero (meaning the pipeline
// couldn't produce a real number for it, e.g. ESKF ZUPT clamps Z to 0 for
// clearance), skip the whole line rather than emitting the "#.#" or
// "N/A (invalid)" clutter the old build produced. This keeps the report
// clean and easy to eyeball.
static void print_float_or_invalid(const char *label, float value, const char *unit, const char *normal_range)
{
    if (is_valid_float(value) && value != 0.0f) {
        am_util_stdio_printf("  %-17s: %8.4f %s  (%s)\r\n", label, value, unit, normal_range);
    }
    // else: intentionally silent - skip the row instead of printing #.#
}

static void print_cv_or_invalid(const char *label, float cv_value, const char *normal_range)
{
    // Only show CV when it's a real, finite, non-zero number. A CV of exactly
    // 0.0 in this pipeline means the underlying mean was zero (see SAFE_CV
    // in aggregate_session_ex), so the metric wasn't measured either.
    if (is_valid_float(cv_value) && cv_value != 0.0f) {
        am_util_stdio_printf("  %-17s: %.2f %%           (%s)\r\n", label, cv_value, normal_range);
    }
    // else: intentionally silent
}

void print_session_report(session_stats_t *stats)
{
    am_util_stdio_printf("\r\n");
    am_util_stdio_printf("======================================================================\r\n");
    am_util_stdio_printf("                     SESSION SUMMARY\r\n");
    am_util_stdio_printf("======================================================================\r\n");
    am_util_stdio_printf(" Parameter                  Mean          CV%%\r\n");
    am_util_stdio_printf("----------------------------------------------------------------------\r\n");

    /* Temporal parameters */
    if (is_valid_float(stats->mean_gct_ms))
        am_util_stdio_printf(" GCT_ms                   %10.4f   %8.2f\r\n",
                             stats->mean_gct_ms, stats->cv_gct_ms);

    if (is_valid_float(stats->mean_stride_time_ms))
        am_util_stdio_printf(" Stride_Time_ms            %10.4f   %8.2f\r\n",
                             stats->mean_stride_time_ms, stats->cv_stride_time_ms);

    if (is_valid_float(stats->mean_stance_time_ms))
        am_util_stdio_printf(" Stance_Time_ms            %10.4f   %8.2f\r\n",
                             stats->mean_stance_time_ms, stats->cv_stance_time_ms);

    if (is_valid_float(stats->mean_swing_time_ms))
        am_util_stdio_printf(" Swing_Time_ms             %10.4f   %8.2f\r\n",
                             stats->mean_swing_time_ms, stats->cv_swing_time_ms);

    if (is_valid_float(stats->mean_stance_ratio))
        am_util_stdio_printf(" Stance_Ratio              %10.4f   %8.2f\r\n",
                             stats->mean_stance_ratio, stats->cv_stance_ratio);

    if (is_valid_float(stats->mean_cadence_spm))
        am_util_stdio_printf(" Cadence_spm               %10.4f   %8.2f\r\n",
                             stats->mean_cadence_spm, stats->cv_cadence_spm);

    /* Spatial parameters */
    if (is_valid_float(stats->mean_stride_length_m))
        am_util_stdio_printf(" Stride_Length_m            %10.4f   %8.2f\r\n",
                             stats->mean_stride_length_m, stats->cv_stride_length_m);

    if (is_valid_float(stats->mean_min_clearance_m))
        am_util_stdio_printf(" Min_Clearance_m            %10.4f   %8.2f\r\n",
                             stats->mean_min_clearance_m, stats->cv_min_clearance_m);

    if (is_valid_float(stats->mean_max_clearance_m))
        am_util_stdio_printf(" Max_Clearance_m            %10.4f   %8.2f\r\n",
                             stats->mean_max_clearance_m, stats->cv_max_clearance_m);

    if (is_valid_float(stats->mean_vertical_osc_cm))
        am_util_stdio_printf(" Vertical_Oscillation_cm    %10.4f   %8.2f\r\n",
                             stats->mean_vertical_osc_cm, stats->cv_vertical_osc_cm);

    if (is_valid_float(stats->mean_ilr_g_per_ms))
        am_util_stdio_printf(" ILR_g_per_ms               %10.4f   %8.2f\r\n",
                             stats->mean_ilr_g_per_ms, stats->cv_ilr_g_per_ms);

    if (is_valid_float(stats->mean_pronation_angle_deg))
        am_util_stdio_printf(" Pronation_Angle_deg        %10.4f   %8.2f\r\n",
                             stats->mean_pronation_angle_deg, stats->cv_pronation_angle_deg);

    if (is_valid_float(stats->mean_angle_change))
        am_util_stdio_printf(" Angle_Change               %10.4f   %8.2f\r\n",
                             stats->mean_angle_change, stats->cv_angle_change);

    if (is_valid_float(stats->mean_energy))
        am_util_stdio_printf(" Energy                     %10.4f   %8.2f\r\n",
                             stats->mean_energy, stats->cv_energy);

    if (is_valid_float(stats->stride_variability_sd_ms))
        am_util_stdio_printf(" Stride_Variability_SD_ms   %10.4f   %8.2f\r\n",
                             stats->stride_variability_sd_ms,
                             stats->stride_variability_cv_pct);

    am_util_stdio_printf("----------------------------------------------------------------------\r\n");

    if (stats->hw_stride_count_available) {
        am_util_stdio_printf(" Hardware Steps             : %u\r\n",
                             stats->hw_step_count);
        am_util_stdio_printf(" Hardware Strides           : %u\r\n",
                             stats->hw_stride_count);
    }

    am_util_stdio_printf(" Detected Strides           : %u\r\n",
                         stats->n_strides);

    am_util_stdio_printf("======================================================================\r\n");
}

// ────────────────────────────────────────────────────────────────────────────
// FIX #3 (see file-top): step6_spatial_raw_sim() is now a NO-OP.
//
// The previous implementation ran an entirely separate stride-length
// estimator (raw acc integration with its own gravity cal, Rodrigues
// orientation update, ZUPT, and scale constant STRIDE_SPATIAL_SCALE_RAW_SIM)
// on the sim buffer and OVERWROTE step6_spatial()'s stride_length_m.
// Python has no equivalent code path at all - Python's stride_length_m
// always comes from compute_spatial_parameters() (ESKF XY displacement x
// STRIDE_SPATIAL_SCALE), which step6_spatial() already ports correctly.
//
// Keeping this symbol as a no-op so stridesense_pipeline.c still links; the
// call site (guarded by #if SIM_CSV_INPUT in main()) can also be removed
// entirely if you'd rather.
// ────────────────────────────────────────────────────────────────────────────
void step6_spatial_raw_sim(const stride_t *stride,
                           const sim_imu_row_t *sim_data,
                           uint32_t n_sim,
                           float odr_hz,
                           float g_mss,
                           spatial_params_t *out)
{
    (void)stride; (void)sim_data; (void)n_sim; (void)odr_hz; (void)g_mss; (void)out;
    // Intentionally left blank - see comment above.
    return;
}