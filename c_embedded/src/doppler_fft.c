/*
 * doppler_fft.c -- see doppler_fft.h. Algorithmic twin of
 * python_model/doppler_fft.py.
 */

#include "doppler_fft.h"
#include "fft_core.h"
#include <math.h>

void doppler_fft_process(chirp_matrix_t *range_fft_out, rd_map_t *rd_map_out) {
    static float win[RADAR_NUM_CHIRPS];
    static int win_ready = 0;
    static cplx_t column[RADAR_NUM_CHIRPS];

    if (!win_ready) {
        hann_window(win, RADAR_NUM_CHIRPS);
        win_ready = 1;
    }

    const unsigned half = RADAR_NUM_CHIRPS / 2u;

    for (unsigned r = 0; r < RADAR_NUM_SAMPLES; r++) {
        /* gather column (one range bin across all chirps) */
        for (unsigned c = 0; c < RADAR_NUM_CHIRPS; c++) {
            column[c].re = (*range_fft_out)[c][r].re * win[c];
            column[c].im = (*range_fft_out)[c][r].im * win[c];
        }

        fft_inplace(column, RADAR_NUM_CHIRPS, 0);

        /* fft-shift + magnitude, written straight into the output map */
        for (unsigned c = 0; c < RADAR_NUM_CHIRPS; c++) {
            unsigned src = (c + half) % RADAR_NUM_CHIRPS;
            float re = column[src].re;
            float im = column[src].im;
            (*rd_map_out)[c][r] = sqrtf(re * re + im * im);
        }
    }
}

void doppler_fft_accumulate_power(chirp_matrix_t *range_fft_out,
                                   float power_accum[RADAR_NUM_CHIRPS][RADAR_NUM_SAMPLES],
                                   int reset) {
    static float win[RADAR_NUM_CHIRPS];
    static int win_ready = 0;
    static cplx_t column[RADAR_NUM_CHIRPS];

    if (!win_ready) {
        hann_window(win, RADAR_NUM_CHIRPS);
        win_ready = 1;
    }

    const unsigned half = RADAR_NUM_CHIRPS / 2u;

    for (unsigned r = 0; r < RADAR_NUM_SAMPLES; r++) {
        for (unsigned c = 0; c < RADAR_NUM_CHIRPS; c++) {
            column[c].re = (*range_fft_out)[c][r].re * win[c];
            column[c].im = (*range_fft_out)[c][r].im * win[c];
        }

        fft_inplace(column, RADAR_NUM_CHIRPS, 0);

        for (unsigned c = 0; c < RADAR_NUM_CHIRPS; c++) {
            unsigned src = (c + half) % RADAR_NUM_CHIRPS;
            float re = column[src].re;
            float im = column[src].im;
            float power = re * re + im * im;

            if (reset) {
                power_accum[c][r] = power;
            } else {
                power_accum[c][r] += power;
            }
        }
    }
}

void doppler_fft_finalize(float power_accum[RADAR_NUM_CHIRPS][RADAR_NUM_SAMPLES],
                           rd_map_t *rd_map_out) {
    for (unsigned c = 0; c < RADAR_NUM_CHIRPS; c++) {
        for (unsigned r = 0; r < RADAR_NUM_SAMPLES; r++) {
            (*rd_map_out)[c][r] = sqrtf(power_accum[c][r]);
        }
    }
}
