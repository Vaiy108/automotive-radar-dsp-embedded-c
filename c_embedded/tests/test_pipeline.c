/*
 * test_pipeline.c -- end-to-end integration test.
 *
 * Runs the full range FFT -> Doppler FFT (multi-antenna non-coherent
 * integration) -> CA-CFAR -> clustering chain on the exact test
 * vectors exported by python_model/radar_pipeline_demo.py, and checks
 * the result against the two known target truths baked into that
 * export (range_m=18.0/velocity_mps=12.0 and range_m=30.0/
 * velocity_mps=-8.0). 
 */

#include <math.h>
#include "test_framework.h"
#include "radar_types.h"
#include "range_fft.h"
#include "doppler_fft.h"
#include "cfar_ca.h"
#include "test_vectors.h"

static chirp_matrix_t g_matrix;
static float g_power_accum[RADAR_NUM_CHIRPS][RADAR_NUM_SAMPLES];
static rd_map_t g_rd_map;
static detection_list_t g_detections;

static void load_test_vector_antenna(unsigned ant) {
    unsigned idx = ant * RADAR_NUM_CHIRPS * RADAR_NUM_SAMPLES;
    for (unsigned c = 0; c < RADAR_NUM_CHIRPS; c++) {
        for (unsigned s = 0; s < RADAR_NUM_SAMPLES; s++) {
            g_matrix[c][s].re = TV_REAL[idx];
            g_matrix[c][s].im = TV_IMAG[idx];
            idx++;
        }
    }
}

/* Same greedy non-max suppression as radar_pipeline.c -- duplicated
 * here rather than shared via a header so this test exercises the
 * exact same source-level logic the production binary ships, not an
 * extracted/refactored copy that could silently drift from it. */
static void cluster_detections(detection_list_t *list, unsigned min_sep) {
    for (unsigned i = 1; i < list->count; i++) {
        detection_t key = list->detections[i];
        unsigned j = i;
        while (j > 0 && list->detections[j - 1].magnitude < key.magnitude) {
            list->detections[j] = list->detections[j - 1];
            j--;
        }
        list->detections[j] = key;
    }

    unsigned char suppressed[RADAR_MAX_DETECTIONS] = {0};
    detection_list_t peaks;
    peaks.count = 0;

    for (unsigned i = 0; i < list->count; i++) {
        if (suppressed[i]) continue;
        detection_t peak = list->detections[i];
        if (peaks.count < RADAR_MAX_DETECTIONS) {
            peaks.detections[peaks.count++] = peak;
        }
        for (unsigned j = 0; j < list->count; j++) {
            int dd = (int)list->detections[j].doppler_idx - (int)peak.doppler_idx;
            int dr = (int)list->detections[j].range_idx - (int)peak.range_idx;
            if (dd < 0) dd = -dd;
            if (dr < 0) dr = -dr;
            if ((unsigned)dd <= min_sep && (unsigned)dr <= min_sep) {
                suppressed[j] = 1;
            }
        }
    }
    *list = peaks;
}

static int find_detection_near(float target_range_m, float target_vel_mps,
                                float range_tol, float vel_tol,
                                float *out_range_m, float *out_vel_mps) {
    const int half = RADAR_NUM_CHIRPS / 2;
    for (unsigned i = 0; i < g_detections.count; i++) {
        detection_t *d = &g_detections.detections[i];
        float range_m = (float)d->range_idx * RADAR_RANGE_RESOLUTION_M;
        float vel_mps = ((float)d->doppler_idx - (float)half) * RADAR_VELOCITY_RESOLUTION_MPS;
        if (fabsf(range_m - target_range_m) <= range_tol &&
            fabsf(vel_mps - target_vel_mps) <= vel_tol) {
            *out_range_m = range_m;
            *out_vel_mps = vel_mps;
            return 1;
        }
    }
    return 0;
}

static void test_full_pipeline_matches_known_targets(void) {
    for (unsigned ant = 0; ant < RADAR_NUM_RX; ant++) {
        load_test_vector_antenna(ant);
        range_fft_process(&g_matrix);
        doppler_fft_accumulate_power(&g_matrix, g_power_accum, ant == 0);
    }
    doppler_fft_finalize(g_power_accum, &g_rd_map);

    cfar_config_t cfg = { .num_train = 8, .num_guard = 4, .pfa = 1e-3f };
    cfar_ca_process(&g_rd_map, &cfg, &g_detections);
    TF_ASSERT_EQ_INT(g_detections.overflowed, 0u);

    cluster_detections(&g_detections, 3);
    TF_ASSERT_EQ_INT(g_detections.count, 2u);

    /* Tolerances are one bin in each dimension: RADAR_RANGE_RESOLUTION_M
     * and RADAR_VELOCITY_RESOLUTION_MPS respectively. */
    float r_m, v_mps;
    int found_t1 = find_detection_near(18.0f, 12.0f, 0.5f, 1.0f, &r_m, &v_mps);
    TF_ASSERT_TRUE(found_t1);
    if (found_t1) {
        TF_ASSERT_NEAR(r_m, 18.0f, 0.5f);
        TF_ASSERT_NEAR(v_mps, 12.0f, 1.0f);
    }

    int found_t2 = find_detection_near(30.0f, -8.0f, 0.5f, 1.0f, &r_m, &v_mps);
    TF_ASSERT_TRUE(found_t2);
    if (found_t2) {
        TF_ASSERT_NEAR(r_m, 30.0f, 0.5f);
        TF_ASSERT_NEAR(v_mps, -8.0f, 1.0f);
    }
}

int main(void) {
    TF_RUN(test_full_pipeline_matches_known_targets);
    return tf_summary();
}
