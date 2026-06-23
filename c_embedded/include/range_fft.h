/*
 * range_fft.h
 *
 * Stage 1 of the embedded DSP pipeline: fast-time (range) FFT.
 * Mirrors python_model/range_fft.py exactly: Hann window along the
 * sample axis, then a forward FFT per chirp row.
 */

#ifndef RANGE_FFT_H
#define RANGE_FFT_H

#include "radar_types.h"

/* In/out: chirp matrix is processed in place, row by row (one FFT per
 * chirp, length RADAR_NUM_SAMPLES). */
void range_fft_process(chirp_matrix_t *matrix);

#endif /* RANGE_FFT_H */
