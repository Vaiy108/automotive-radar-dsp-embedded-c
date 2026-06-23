/*
 * doppler_fft.h
 *
 * Stage 2 of the embedded DSP pipeline: slow-time (Doppler) FFT.
 * Mirrors python_model/doppler_fft.py: Hann window along the chirp
 * axis, FFT per range bin (column), then fft-shift so index
 * RADAR_NUM_CHIRPS/2 corresponds to zero velocity -- this keeps the
 * bin layout identical to the Python reference model for easy
 * cross-validation.
 */

#ifndef DOPPLER_FFT_H
#define DOPPLER_FFT_H

#include "radar_types.h"

/* In: range-FFT output (chirp_matrix_t, complex).
 * Out: rd_map populated with the non-coherent magnitude
 *      (here: single-antenna magnitude, since the demo pipeline
 *      processes one antenna -- see docs/embedded_architecture.md
 *      for how this generalizes to RADAR_NUM_RX antennas). */
void doppler_fft_process(chirp_matrix_t *range_fft_out, rd_map_t *rd_map_out);

/* Multi-antenna non-coherent integration without holding all
 * RADAR_NUM_RX antennas in RAM simultaneously: call once per antenna
 * with the same power_accum buffer, reset=1 on the first antenna and
 * reset=0 on the rest. Call doppler_fft_finalize() once afterwards to
 * turn the accumulated power into the final rd_map_t magnitude map --
 * this is the same non-coherent sum used by
 * python_model/doppler_fft.py::range_doppler_magnitude(). */
void doppler_fft_accumulate_power(chirp_matrix_t *range_fft_out,
                                   float power_accum[RADAR_NUM_CHIRPS][RADAR_NUM_SAMPLES],
                                   int reset);

void doppler_fft_finalize(float power_accum[RADAR_NUM_CHIRPS][RADAR_NUM_SAMPLES],
                           rd_map_t *rd_map_out);

#endif /* DOPPLER_FFT_H */
