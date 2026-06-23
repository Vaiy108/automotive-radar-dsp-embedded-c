/*
 * fft_core.h
 *
 * Minimal, dependency-free, in-place iterative radix-2 Cooley-Tukey
 * FFT for power-of-two lengths, plus a Hann window helper. Both
 * range_fft.c and doppler_fft.c are thin, semantically-named wrappers
 * around this shared core -- range FFT runs it along the fast-time
 * (sample) axis, Doppler FFT runs it along the slow-time (chirp) axis.
 *
 * Single-precision float throughout: sufficient dynamic range for a
 * radar magnitude pipeline and the natural choice on automotive
 * DSP/MCU cores with a hardware FPU.
 */

#ifndef FFT_CORE_H
#define FFT_CORE_H

#include <stddef.h>
#include "radar_types.h"

/* In-place FFT of a power-of-two length complex buffer.
 * inverse = 0 -> forward FFT, inverse = 1 -> inverse FFT (unnormalized * N).
 * Returns 0 on success, -1 if n is not a power of two. */
int fft_inplace(cplx_t *buf, size_t n, int inverse);

/* Fills win[0..n-1] with Hann window coefficients. */
void hann_window(float *win, size_t n);

/* Bit-reverses x assuming it is log2_n bits wide (used internally,
 * exposed for unit testing). */
unsigned fft_bit_reverse(unsigned x, unsigned log2_n);

#endif /* FFT_CORE_H */
