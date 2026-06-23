/*
 * cfar_ca.c -- see cfar_ca.h. Algorithmic twin of python_model/cfar.py.
 *
 * Deliberately uses the same "recompute the window sum per cell"
 * approach as the Python reference (rather than a sliding-sum /
 * summed-area-table optimization) so the two implementations similar
  in the benchmark -- the speedup
 * results/benchmark_table.csv comes from C vs. interpreted
 * Python on identical algorithmic complexity, not from a smarter
 * algorithm on one side.
 */

#include "cfar_ca.h"
#include <math.h>

static float cfar_alpha(unsigned num_training_cells, float pfa) {
    float n = (float)num_training_cells;
    return n * (powf(pfa, -1.0f / n) - 1.0f);
}

void cfar_ca_process(const rd_map_t *rd_map, const cfar_config_t *cfg,
                      detection_list_t *out_list) {
    out_list->count = 0;
    out_list->overflowed = 0;

    const unsigned g = cfg->num_guard;
    const unsigned t = cfg->num_train;
    const unsigned win = g + t;

    const unsigned win_side = 2u * win + 1u;
    const unsigned guard_side = 2u * g + 1u;
    const unsigned num_train_cells = win_side * win_side - guard_side * guard_side;
    const float alpha = cfar_alpha(num_train_cells, cfg->pfa);

    for (unsigned d = win; d < RADAR_NUM_CHIRPS - win; d++) {
        for (unsigned r = win; r < RADAR_NUM_SAMPLES - win; r++) {

            float window_sum = 0.0f;
            for (unsigned dd = d - win; dd <= d + win; dd++) {
                for (unsigned rr = r - win; rr <= r + win; rr++) {
                    float v = (*rd_map)[dd][rr];
                    window_sum += v * v;
                }
            }

            float guard_sum = 0.0f;
            for (unsigned dd = d - g; dd <= d + g; dd++) {
                for (unsigned rr = r - g; rr <= r + g; rr++) {
                    float v = (*rd_map)[dd][rr];
                    guard_sum += v * v;
                }
            }

            float train_sum = window_sum - guard_sum;
            float noise_est = train_sum / (float)num_train_cells;
            float threshold_power = alpha * noise_est;

            float cell_v = (*rd_map)[d][r];
            float cell_power = cell_v * cell_v;

            if (cell_power > threshold_power) {
                if (out_list->count < RADAR_MAX_DETECTIONS) {
                    detection_t *det = &out_list->detections[out_list->count];
                    det->doppler_idx = (uint16_t)d;
                    det->range_idx = (uint16_t)r;
                    det->magnitude = cell_v;
                    out_list->count++;
                } else {
                    out_list->overflowed = 1;
                }
            }
        }
    }
}
