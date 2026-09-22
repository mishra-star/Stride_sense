/*
 * Gait_health_score.c
 *
 * Exact port of the executable scoring path in the uploaded Python reference.
 * This file intentionally computes the statistics from per-stride arrays,
 * because the Python reference uses each Cadence_spm/GCT/Pronation value,
 * not only session means/CVs.
 */
#include "Gait_health_score.h"

#include <math.h>
#include <string.h>

#define W_CADENCE   0.30f
#define W_IMPACT    0.30f
#define W_PRONATION 0.15f
#define W_GCT       0.25f

#define GCT_MIN_MS 350.0f
#define GCT_MAX_MS 1100.0f
#define PRONATION_ALERT_DEG 8.0f

typedef struct { float x, y; } anchor_t;

static const anchor_t k_impact_walk[] = {
    {0,0},{5,30},{10,70},{15,90},{22,100},{35,100},
    {50,80},{70,45},{90,15},{110,0}
};
static const anchor_t k_impact_run[] = {
    {0,0},{10,30},{20,65},{30,85},{40,100},{80,100},
    {100,90},{120,75},{150,50},{180,25},{220,0}
};
static const anchor_t k_pronation[] = {
    {0,100},{3,95},{5,85},{8,65},{10,50},{13,25},{16,0}
};
static const anchor_t k_gct_cv[] = {
    {0,100},{0.03f,98},{0.05f,90},{0.10f,75},
    {0.15f,55},{0.20f,35},{0.30f,0}
};
static const anchor_t k_cad_std[] = {
    {0,100},{30,85},{50,70},{80,45},{120,20},{150,0}
};

static float pw(float v, const anchor_t *a, uint32_t n)
{
    uint32_t i;
    if (n == 0u) return 0.0f;
    if (v <= a[0].x) return a[0].y;
    if (v >= a[n-1u].x) return a[n-1u].y;

    for (i = 1u; i < n; ++i) {
        if (v <= a[i].x) {
            float f = (v - a[i-1u].x) / (a[i].x - a[i-1u].x);
            return a[i-1u].y + f * (a[i].y - a[i-1u].y);
        }
    }
    return a[n-1u].y;
}

static float round1(float v) { return roundf(v * 10.0f) / 10.0f; }
static float round2(float v) { return roundf(v * 100.0f) / 100.0f; }
static float round4(float v) { return roundf(v * 10000.0f) / 10000.0f; }

static const char *score_band(float s)
{
    if (!isfinite(s)) return "Unknown";
    if (s >= 85.0f) return "Excellent";
    if (s >= 70.0f) return "Good";
    if (s >= 50.0f) return "Fair";
    return "Poor";
}

static void init_result(gait_score_result_t *r)
{
    memset(r, 0, sizeof(*r));
    r->health_score = NAN;
    r->sub_cadence_regularity = NAN;
    r->sub_impact_loading = NAN;
    r->sub_pronation = NAN;
    r->sub_gct_consistency = NAN;
    r->cadence_std_interval_ms = NAN;
    r->impact_BW_per_s = NAN;
    r->gct_cv = NAN;
    r->gct_mean_ms = NAN;
    r->pronation_mean_dev_deg = NAN;
    r->pronation_neutral_used = NAN;
    strcpy(r->band, "Unknown");
}

/* Insertion-sort a small temporary float array. */
static void sort_float(float *x, uint32_t n)
{
    uint32_t i;
    for (i = 1u; i < n; ++i) {
        float key = x[i];
        uint32_t j = i;
        while (j > 0u && x[j-1u] > key) {
            x[j] = x[j-1u];
            --j;
        }
        x[j] = key;
    }
}

static float median_sorted(const float *x, uint32_t n)
{
    if (n == 0u) return NAN;
    if (n & 1u) return x[n/2u];
    return 0.5f * (x[n/2u - 1u] + x[n/2u]);
}

static float sample_std(const float *x, uint32_t n)
{
    uint32_t i;
    double sum = 0.0, ss = 0.0;
    double mean, d;

    if (n < 2u) return NAN;
    for (i = 0u; i < n; ++i) sum += x[i];
    mean = sum / (double)n;
    for (i = 0u; i < n; ++i) {
        d = (double)x[i] - mean;
        ss += d * d;
    }
    return (float)sqrt(ss / (double)(n - 1u));
}

static bool valid_python_row(const temporal_params_t *t,
                             const spatial_params_t *s,
                             bool is_outlier,
                             bool turning_step)
{
    /*
     * Python load_clean():
     *   numeric conversion -> remove outlier/turning rows
     *   -> require all four metric columns finite
     *   -> GCT between 350 and 1100 ms.
     *
     * turning_step and is_outlier are supplied as parallel arrays so that
     * gait_pipeline.h does NOT need to be modified.
     */
    if (!isfinite(t->cadence_spm) ||
        !isfinite(s->ilr_g_per_ms) ||
        !isfinite(s->pronation_angle_deg) ||
        !isfinite(t->gct_ms))
        return false;

    if (is_outlier || turning_step) return false;
    if (t->gct_ms < GCT_MIN_MS || t->gct_ms > GCT_MAX_MS)
        return false;

    return true;
}

static bool compute_pronation_exact(
    const temporal_params_t *t, const spatial_params_t *s,
    const bool *outlier, const uint32_t *bout_index,
    const bool *turning_step, uint32_t n,
    float *mean_dev, float *neutral)
{
    /*
     * Exact Python default:
     *   group by bout_index
     *   median(pronation) for each bout
     *   abs(angle - bout_median) for every retained row
     *   neutral_used = mean(all bout medians)
     *
     * The bout index is passed separately from LSM_pipeline_GH_score.c.
     * This avoids adding bout_index to stride_t / gait_pipeline.h.
     */
    uint32_t i = 0u, total = 0u, bout_count = 0u;
    double dev_sum = 0.0, neut_sum = 0.0;

    while (i < n) {
        uint32_t j = i + 1u;
        uint32_t k, m = 0u;
        float vals[256];
        float med;

        /* Records are appended bout-by-bout in the existing LSM pipeline. */
        if (bout_index) {
            while (j < n && bout_index[j] == bout_index[i])
                ++j;
        } else {
            /* No bout information: treat the complete session as one bout. */
            j = n;
        }

        for (k = i; k < j; ++k) {
            bool o = outlier ? outlier[k] : false;
            bool tr = turning_step ? turning_step[k] : false;

            if (valid_python_row(&t[k], &s[k], o, tr)) {
                if (m < 256u)
                    vals[m++] = s[k].pronation_angle_deg;
            }
        }

        if (m > 0u) {
            sort_float(vals, m);
            med = median_sorted(vals, m);

            if (isfinite(med)) {
                neut_sum += med;
                ++bout_count;

                for (k = i; k < j; ++k) {
                    bool o = outlier ? outlier[k] : false;
                    bool tr = turning_step ? turning_step[k] : false;

                    if (valid_python_row(&t[k], &s[k], o, tr)) {
                        dev_sum += fabs((double)s[k].pronation_angle_deg -
                                        (double)med);
                        ++total;
                    }
                }
            }
        }

        i = j;
    }

    if (total == 0u || bout_count == 0u)
        return false;

    *mean_dev = (float)(dev_sum / (double)total);
    *neutral = (float)(neut_sum / (double)bout_count);
    return true;
}

void gait_health_score_compute_ex(
    const temporal_params_t *temp_arr,
    const spatial_params_t *spat_arr,
    const bool *session_outlier,
    const uint32_t *session_bout_index,
    const bool *session_turning_step,
    uint32_t n_strides,
    const char *activity,
    const char *foot,
    gait_score_result_t *r)
{
    /*
     * No changes are required to gait_pipeline.c or gait_pipeline.h.
     * The LSM caller supplies outlier/bout/turning metadata as parallel arrays.
     * Cadence/GCT use streaming accumulators; pronation uses a 256-value
     * per-bout scratch buffer.
     */
    uint32_t i, n = 0u, n_cad = 0u, n_gct = 0u, n_ilr = 0u;
    double cad_sum = 0.0, cad_ss = 0.0;
    double gct_sum = 0.0, gct_ss = 0.0;
    double ilr_sum = 0.0;
    float pron_dev, pron_neutral;
    float cad_std, gct_cv, ilr_bw_s;
    float used_sum = 0.0f, total_w = 0.0f;
    bool running;

    (void)foot;
    init_result(r);

    if (!temp_arr || !spat_arr || n_strides == 0u) return;

    running = (activity &&
               (activity[0] == 'r' || activity[0] == 'R'));

    /*
     * The Python cleaned dataframe is the common base for all four metrics.
     * A row is retained only if all four metric columns are finite, it is not
     * an outlier, and GCT is inside the hard bounds.
     */
    for (i = 0u; i < n_strides; ++i) {
        const temporal_params_t *t = &temp_arr[i];
        const spatial_params_t *s = &spat_arr[i];
        bool o = session_outlier ? session_outlier[i] : false;
        bool tr = session_turning_step ? session_turning_step[i] : false;

        if (!valid_python_row(t, s, o, tr))
            continue;
        ++n;

        if (t->cadence_spm > 0.0f) {
            double interval = 120000.0 / (double)t->cadence_spm;
            cad_sum += interval;
            cad_ss += interval * interval;
            ++n_cad;
        }

        if (s->ilr_g_per_ms >= 0.0f) {
            ilr_sum += (double)s->ilr_g_per_ms;
            ++n_ilr;
        }

        if (t->gct_ms > 0.0f) {
            gct_sum += (double)t->gct_ms;
            gct_ss += (double)t->gct_ms * (double)t->gct_ms;
            ++n_gct;
        }
    }

    r->n_used = n;


    /* Python cad = std(120000/cad, ddof=1). */
    if (n_cad >= 2u) {
        double mean = cad_sum / (double)n_cad;
        double var = (cad_ss - (double)n_cad * mean * mean) /
                     (double)(n_cad - 1u);
        if (var < 0.0 && var > -1e-9) var = 0.0;
        cad_std = (float)sqrt(var);
        r->cadence_std_interval_ms = round2(cad_std);
        r->sub_cadence_regularity =
            round1(pw(cad_std, k_cad_std,
                      (uint32_t)(sizeof(k_cad_std)/sizeof(k_cad_std[0]))));
        r->has_cadence = true;
    }

    /* Python: mean(ILR_g_per_ms * 1000), non-negative only. */
    if (n_ilr > 0u) {
        ilr_bw_s = (float)(ilr_sum * 1000.0 / (double)n_ilr);
        r->impact_BW_per_s = round2(ilr_bw_s);
        if (isfinite(ilr_bw_s)) {
            const anchor_t *a = running ? k_impact_run : k_impact_walk;
            uint32_t an = running
                ? (uint32_t)(sizeof(k_impact_run)/sizeof(k_impact_run[0]))
                : (uint32_t)(sizeof(k_impact_walk)/sizeof(k_impact_walk[0]));
            r->sub_impact_loading = round1(pw(ilr_bw_s, a, an));
            r->has_impact = true;
        }
    }

    /*
     * Exact Python default pronation mode:
     * deviation_from_bout_median.
     */
    if (compute_pronation_exact(temp_arr, spat_arr,
                                 session_outlier, session_bout_index,
                                 session_turning_step, n_strides,
                                 &pron_dev, &pron_neutral)) {
        r->pronation_mean_dev_deg = round2(pron_dev);
        r->pronation_neutral_used = round2(pron_neutral);
        r->sub_pronation = round1(
            pw(pron_dev, k_pronation,
               (uint32_t)(sizeof(k_pronation)/sizeof(k_pronation[0]))));
        r->has_pronation = true;
        r->pronation_alert =
            (W_PRONATION > 0.0f) && (pron_dev > PRONATION_ALERT_DEG);
    }

    /* Python: sample std / mean, ddof=1. */
    if (n_gct >= 2u) {
        double mean = gct_sum / (double)n_gct;
        double var = (gct_ss - (double)n_gct * mean * mean) /
                     (double)(n_gct - 1u);
        if (var < 0.0 && var > -1e-9) var = 0.0;
        gct_cv = (mean > 0.0) ? (float)(sqrt(var) / mean) : NAN;
        if (isfinite(gct_cv)) {
            r->gct_cv = round4(gct_cv);
            r->gct_mean_ms = round1((float)mean);
            r->sub_gct_consistency = round1(
                pw(gct_cv, k_gct_cv,
                   (uint32_t)(sizeof(k_gct_cv)/sizeof(k_gct_cv[0]))));
            r->has_gct = true;
        }
    }

    if (r->has_cadence) {
        used_sum += W_CADENCE * r->sub_cadence_regularity;
        total_w += W_CADENCE;
    }
    if (r->has_impact) {
        used_sum += W_IMPACT * r->sub_impact_loading;
        total_w += W_IMPACT;
    }
    if (r->has_pronation) {
        used_sum += W_PRONATION * r->sub_pronation;
        total_w += W_PRONATION;
    }
    if (r->has_gct) {
        used_sum += W_GCT * r->sub_gct_consistency;
        total_w += W_GCT;
    }

    if (total_w > 0.0f) r->health_score = round1(used_sum / total_w);
    strncpy(r->band, score_band(r->health_score), sizeof(r->band)-1u);
    r->band[sizeof(r->band)-1u] = '\0';
}

/*
 * Aggregate-only compatibility implementation.
 * This is retained for existing callers. The exact pipeline calls
 * gait_health_score_compute_ex() with per-stride arrays.
 */
void gait_health_score_compute(
    const session_stats_t *stats,
    const char *activity,
    const char *foot,
    gait_score_result_t *r)
{
    float used_sum = 0.0f, total_w = 0.0f;
    bool running = activity &&
                   (activity[0] == 'r' || activity[0] == 'R');

    (void)foot;
    init_result(r);
    if (!stats) return;

    if (stats->mean_cadence_spm > 0.0f &&
        isfinite(stats->cv_cadence_spm) &&
        stats->n_strides >= 2u) {
        /*
         * Fallback only. Exact Python requires std of every 120000/cadence.
         */
        float mean_interval = 120000.0f / stats->mean_cadence_spm;
        float std_ms = (stats->cv_cadence_spm / 100.0f) * mean_interval;
        r->cadence_std_interval_ms = round2(std_ms);
        r->sub_cadence_regularity = round1(
            pw(std_ms, k_cad_std,
               (uint32_t)(sizeof(k_cad_std)/sizeof(k_cad_std[0]))));
        r->has_cadence = true;
    }

    if (isfinite(stats->mean_ilr_g_per_ms) && stats->mean_ilr_g_per_ms >= 0.0f) {
        float x = stats->mean_ilr_g_per_ms * 1000.0f;
        const anchor_t *a = running ? k_impact_run : k_impact_walk;
        uint32_t an = running
            ? (uint32_t)(sizeof(k_impact_run)/sizeof(k_impact_run[0]))
            : (uint32_t)(sizeof(k_impact_walk)/sizeof(k_impact_walk[0]));
        r->impact_BW_per_s = round2(x);
        r->sub_impact_loading = round1(pw(x, a, an));
        r->has_impact = true;
    }

    if (isfinite(stats->mean_gct_ms) &&
        stats->mean_gct_ms > 0.0f &&
        isfinite(stats->cv_gct_ms)) {
        float cv = stats->cv_gct_ms / 100.0f;
        r->gct_cv = round4(cv);
        r->gct_mean_ms = round1(stats->mean_gct_ms);
        r->sub_gct_consistency = round1(
            pw(cv, k_gct_cv,
               (uint32_t)(sizeof(k_gct_cv)/sizeof(k_gct_cv[0]))));
        r->has_gct = true;
    }

    if (isfinite(stats->mean_pronation_angle_deg) &&
        isfinite(stats->cv_pronation_angle_deg)) {
        float dev = (stats->cv_pronation_angle_deg / 100.0f) *
                    fabsf(stats->mean_pronation_angle_deg);
        r->pronation_mean_dev_deg = round2(dev);
        r->pronation_neutral_used = round2(stats->mean_pronation_angle_deg);
        r->sub_pronation = round1(
            pw(dev, k_pronation,
               (uint32_t)(sizeof(k_pronation)/sizeof(k_pronation[0]))));
        r->has_pronation = true;
        r->pronation_alert = (W_PRONATION > 0.0f &&
                              dev > PRONATION_ALERT_DEG);
    }

    if (r->has_cadence) { used_sum += W_CADENCE*r->sub_cadence_regularity; total_w += W_CADENCE; }
    if (r->has_impact)  { used_sum += W_IMPACT*r->sub_impact_loading; total_w += W_IMPACT; }
    if (r->has_pronation) { used_sum += W_PRONATION*r->sub_pronation; total_w += W_PRONATION; }
    if (r->has_gct) { used_sum += W_GCT*r->sub_gct_consistency; total_w += W_GCT; }

    if (total_w > 0.0f) r->health_score = round1(used_sum/total_w);
    strncpy(r->band, score_band(r->health_score), sizeof(r->band)-1u);
    r->band[sizeof(r->band)-1u] = '\0';
}