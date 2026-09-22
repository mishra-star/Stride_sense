/*
 * Gait_health_score.h
 *
 * Exact embedded C implementation of the uploaded Python gait-health-score
 * reference where the required per-stride inputs are available.
 *
 * Python parity:
 *   - GCT bounds: 350..1100 ms
 *   - cadence interval: 120000 / Cadence_spm
 *   - sample standard deviation (N-1)
 *   - ILR_g_per_ms -> BW/s by *1000
 *   - pronation: mean absolute deviation from each bout median
 *   - GCT CV: sample_std / mean
 *   - numpy.interp-style piecewise linear anchors
 *   - weighted composite with weight renormalization
 *
 * No dynamic allocation and no stdio.
 */
#ifndef GAIT_HEALTH_SCORE_H
#define GAIT_HEALTH_SCORE_H

#include <stdint.h>
#include <stdbool.h>
#include "gait_pipeline.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float health_score;
    char  band[12];

    float sub_cadence_regularity;
    float sub_impact_loading;
    float sub_pronation;
    float sub_gct_consistency;

    bool has_cadence;
    bool has_impact;
    bool has_pronation;
    bool has_gct;
    bool pronation_alert;

    float cadence_std_interval_ms;
    float impact_BW_per_s;
    float gct_cv;
    float gct_mean_ms;
    float pronation_mean_dev_deg;
    float pronation_neutral_used;
    uint32_t n_used;
} gait_score_result_t;

/*
 * Exact scorer. temp_arr/spat_arr/session_strides must describe the same
 * per-stride records in the same order.
 *
 * session_bout_index[i] is used for Python's
 * deviation_from_bout_median mode. These metadata arrays are separate so
 * gait_pipeline.h / stride_t remain unchanged.
 */
void gait_health_score_compute_ex(
    const temporal_params_t *temp_arr,
    const spatial_params_t *spat_arr,
    const bool *session_outlier,
    const uint32_t *session_bout_index,
    const bool *session_turning_step,
    uint32_t n_strides,
    const char *activity,
    const char *foot,
    gait_score_result_t *result);

/*
 * Compatibility wrapper for callers that only have session aggregates.
 * This cannot reproduce Python's per-stride cadence/pronation calculations
 * exactly; use gait_health_score_compute_ex() for Python parity.
 */
void gait_health_score_compute(
    const session_stats_t *stats,
    const char *activity,
    const char *foot,
    gait_score_result_t *result);

#ifdef __cplusplus
}
#endif
#endif