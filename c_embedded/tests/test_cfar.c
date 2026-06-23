/*
 * test_cfar.c -- unit tests for cfar_ca.c
 *
 * Includes a deliberate regression test (test_overflow_flag_is_set)
 See * docs/embedded_architecture.md for the full writeup.
 */

#include <string.h>
#include "test_framework.h"
#include "cfar_ca.h"
#include "radar_types.h"

static rd_map_t g_map;

static void fill_flat_noise(float level) {
    for (unsigned d = 0; d < RADAR_NUM_CHIRPS; d++) {
        for (unsigned r = 0; r < RADAR_NUM_SAMPLES; r++) {
            g_map[d][r] = level;
        }
    }
}

static void test_flat_noise_yields_no_detections(void) {
    /* A perfectly flat map has zero contrast against its own local
     * mean, so CA-CFAR (which only fires on cells that stand out
     * above their neighbourhood) must report nothing. */
    fill_flat_noise(100.0f);

    cfar_config_t cfg = { .num_train = 8, .num_guard = 4, .pfa = 1e-3f };
    detection_list_t out;
    cfar_ca_process(&g_map, &cfg, &out);

    TF_ASSERT_EQ_INT(out.count, 0u);
    TF_ASSERT_EQ_INT(out.overflowed, 0u);
}

static void test_single_strong_target_is_detected(void) {
    /* One cell far above a flat noise floor must be flagged, and its
     * exact (doppler, range) location must be reported. */
    fill_flat_noise(10.0f);
    const unsigned d0 = 64, r0 = 128;
    g_map[d0][r0] = 5000.0f;

    cfar_config_t cfg = { .num_train = 8, .num_guard = 4, .pfa = 1e-3f };
    detection_list_t out;
    cfar_ca_process(&g_map, &cfg, &out);

    int found = 0;
    for (unsigned i = 0; i < out.count; i++) {
        if (out.detections[i].doppler_idx == d0 && out.detections[i].range_idx == r0) {
            found = 1;
        }
    }
    TF_ASSERT_TRUE(found);
    TF_ASSERT_EQ_INT(out.overflowed, 0u);
}

static void test_edge_band_is_excluded(void) {
    /* Cells inside the (num_train + num_guard) margin from any edge
     * can't form a full training window and must never be scanned --
     * even an extreme spike there should produce no detection. */
    fill_flat_noise(10.0f);
    g_map[0][0] = 1.0e9f;
    g_map[RADAR_NUM_CHIRPS - 1][RADAR_NUM_SAMPLES - 1] = 1.0e9f;

    cfar_config_t cfg = { .num_train = 8, .num_guard = 4, .pfa = 1e-3f };
    detection_list_t out;
    cfar_ca_process(&g_map, &cfg, &out);

    for (unsigned i = 0; i < out.count; i++) {
        TF_ASSERT_TRUE(out.detections[i].doppler_idx != 0);
        TF_ASSERT_TRUE(out.detections[i].range_idx != 0);
    }
}

static void test_overflow_flag_is_set(void) {
    /* Regression test for the real bug described in the file header.
     */
    fill_flat_noise(0.001f);
    const int spacing = 8;
    for (unsigned d = 20; d < RADAR_NUM_CHIRPS - 20; d += spacing) {
        for (unsigned r = 20; r < RADAR_NUM_SAMPLES - 20; r += spacing) {
            g_map[d][r] = 9999.0f;
        }
    }

    cfar_config_t cfg = { .num_train = 8, .num_guard = 4, .pfa = 1e-3f };
    detection_list_t out;
    cfar_ca_process(&g_map, &cfg, &out);

    TF_ASSERT_EQ_INT(out.overflowed, 1u);
    TF_ASSERT_EQ_INT(out.count, RADAR_MAX_DETECTIONS);
}

int main(void) {
    TF_RUN(test_flat_noise_yields_no_detections);
    TF_RUN(test_single_strong_target_is_detected);
    TF_RUN(test_edge_band_is_excluded);
    TF_RUN(test_overflow_flag_is_set);
    return tf_summary();
}
