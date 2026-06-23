/*
 * radar_pipeline.c
 *
 * End-to-end embedded pipeline:
 *     test_vectors.h (exported from the Python model)
 *       --> range_fft_process
 *       --> doppler_fft_process
 *       --> cfar_ca_process
 *       --> printed object list
 *
 * This is the C-side equivalent of python_model/radar_pipeline_demo.py,
 * run on the exact same input data

 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "radar_types.h"
#include "range_fft.h"
#include "doppler_fft.h"
#include "cfar_ca.h"
#include "test_vectors.h"

static chirp_matrix_t g_matrix;                                   /* one antenna at a time */
static float g_power_accum[RADAR_NUM_CHIRPS][RADAR_NUM_SAMPLES];  /* non-coherent sum across antennas */
static rd_map_t g_rd_map;
static detection_list_t g_detections;

/* Loads antenna `ant` of the exported [antenna][chirp][sample] test
 * cube into g_matrix. Only one antenna is ever resident in RAM --
 */
static void load_test_vector_antenna(unsigned ant) {
#if (TV_NUM_RX != RADAR_NUM_RX) || (TV_NUM_CHIRPS != RADAR_NUM_CHIRPS) || (TV_NUM_SAMPLES != RADAR_NUM_SAMPLES)
#error "test_vectors.h dimensions do not match radar_types.h -- regenerate with radar_pipeline_demo.py"
#endif
    unsigned base = ant * RADAR_NUM_CHIRPS * RADAR_NUM_SAMPLES;
    unsigned idx = base;
    for (unsigned c = 0; c < RADAR_NUM_CHIRPS; c++) {
        for (unsigned s = 0; s < RADAR_NUM_SAMPLES; s++) {
            g_matrix[c][s].re = TV_REAL[idx];
            g_matrix[c][s].im = TV_IMAG[idx];
            idx++;
        }
    }
}

/* Greedy non-max suppression: collapses each target's mainlobe
 * footprint of adjacent CFAR cells down to its single strongest cell.
 * Mirrors python_model/cfar.py::cluster_detections -- sorting by
  */
static unsigned cluster_detections(detection_list_t *list, unsigned min_sep) {
    /* simple descending insertion sort by magnitude (count is small,
     * bounded by RADAR_MAX_DETECTIONS) */
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
    return list->count;
}

int main(void) {
    for (unsigned ant = 0; ant < RADAR_NUM_RX; ant++) {
        load_test_vector_antenna(ant);
        range_fft_process(&g_matrix);
        doppler_fft_accumulate_power(&g_matrix, g_power_accum, ant == 0);
    }
    doppler_fft_finalize(g_power_accum, &g_rd_map);

    cfar_config_t cfar_cfg = { .num_train = 8, .num_guard = 4, .pfa = 1e-3f };
    cfar_ca_process(&g_rd_map, &cfar_cfg, &g_detections);
    if (g_detections.overflowed) {
        printf("WARNING: raw CFAR detection buffer overflowed (RADAR_MAX_DETECTIONS=%u too small)\n",
               (unsigned)RADAR_MAX_DETECTIONS);
    }
    cluster_detections(&g_detections, 3);

    printf("C pipeline: %u detection(s) after clustering\n", g_detections.count);
    printf("%-10s %-10s %-10s\n", "range_m", "vel_mps", "magnitude");

    const int half = RADAR_NUM_CHIRPS / 2;
    for (unsigned i = 0; i < g_detections.count; i++) {
        detection_t *d = &g_detections.detections[i];
        float range_m = (float)d->range_idx * RADAR_RANGE_RESOLUTION_M;
        float vel_mps = ((float)d->doppler_idx - (float)half) * RADAR_VELOCITY_RESOLUTION_MPS;
        printf("%-10.2f %-10.2f %-10.2f\n", range_m, vel_mps, d->magnitude);
    }

    return 0;
}
