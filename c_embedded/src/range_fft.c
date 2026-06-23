/*
 * range_fft.c -- see range_fft.h. Algorithmic twin of
 * python_model/range_fft.py.
 */

#include "range_fft.h"
#include "fft_core.h"

void range_fft_process(chirp_matrix_t *matrix) {
    static float win[RADAR_NUM_SAMPLES];
    static int win_ready = 0;

    if (!win_ready) {
        hann_window(win, RADAR_NUM_SAMPLES);
        win_ready = 1;
    }

    for (unsigned chirp = 0; chirp < RADAR_NUM_CHIRPS; chirp++) {
        cplx_t *row = (*matrix)[chirp];

        for (unsigned s = 0; s < RADAR_NUM_SAMPLES; s++) {
            row[s].re *= win[s];
            row[s].im *= win[s];
        }

        fft_inplace(row, RADAR_NUM_SAMPLES, 0);
    }
}
