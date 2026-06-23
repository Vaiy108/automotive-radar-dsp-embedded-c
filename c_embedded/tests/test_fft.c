/*
 * test_fft.c -- unit tests for fft_core.c
 *
 * Validates the building block shared by both range_fft.c and
 * doppler_fft.c against known-closed-form FFT results (DC tone,
 * single complex exponential, forward/inverse round trip) rather
 * than against the radar pipeline itself, so a failure here points
 * straight at the FFT primitive instead of anywhere downstream.
 */

#include <math.h>
#include <stddef.h>
#include "test_framework.h"
#include "fft_core.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static void test_bit_reverse_3bit(void) {
    /* n = 8 -> log2_n = 3. 3-bit reversal: 001 <-> 100, 011 <-> 110, etc. */
    TF_ASSERT_EQ_INT(fft_bit_reverse(0, 3), 0u);
    TF_ASSERT_EQ_INT(fft_bit_reverse(1, 3), 4u);
    TF_ASSERT_EQ_INT(fft_bit_reverse(2, 3), 2u);
    TF_ASSERT_EQ_INT(fft_bit_reverse(3, 3), 6u);
    TF_ASSERT_EQ_INT(fft_bit_reverse(4, 3), 1u);
    TF_ASSERT_EQ_INT(fft_bit_reverse(7, 3), 7u);
}

static void test_hann_window_shape(void) {
    float win[16];
    hann_window(win, 16);

    /* Hann window is zero at both ends and symmetric. */
    TF_ASSERT_NEAR(win[0], 0.0f, 1e-5f);
    TF_ASSERT_NEAR(win[15], 0.0f, 1e-5f);
    for (int i = 0; i < 16; i++) {
        TF_ASSERT_NEAR(win[i], win[15 - i], 1e-5f);
    }

    /* Peak near the center should approach (but not exceed) 1.0. */
    float peak = 0.0f;
    for (int i = 0; i < 16; i++) {
        if (win[i] > peak) peak = win[i];
    }
    TF_ASSERT_TRUE(peak > 0.9f && peak <= 1.0001f);
}

static void test_fft_rejects_non_power_of_two(void) {
    cplx_t buf[6] = {{0}};
    int rc = fft_inplace(buf, 6, 0);
    TF_ASSERT_EQ_INT(rc, -1);
}

static void test_fft_dc_input(void) {
    /* A constant (DC) signal should produce all of its energy in
     * bin 0, with every other bin at zero. */
    const size_t n = 8;
    cplx_t buf[8];
    for (size_t i = 0; i < n; i++) {
        buf[i].re = 1.0f;
        buf[i].im = 0.0f;
    }

    int rc = fft_inplace(buf, n, 0);
    TF_ASSERT_EQ_INT(rc, 0);

    TF_ASSERT_NEAR(buf[0].re, (float)n, 1e-3f);
    TF_ASSERT_NEAR(buf[0].im, 0.0f, 1e-3f);
    for (size_t k = 1; k < n; k++) {
        TF_ASSERT_NEAR(buf[k].re, 0.0f, 1e-3f);
        TF_ASSERT_NEAR(buf[k].im, 0.0f, 1e-3f);
    }
}

static void test_fft_single_tone(void) {
    /* A complex exponential at bin k0=2 of an 8-point FFT must show
     * up as a single spike of magnitude n at exactly bin 2. */
    const size_t n = 8;
    const int k0 = 2;
    cplx_t buf[8];
    for (size_t i = 0; i < n; i++) {
        double phase = 2.0 * M_PI * k0 * (double)i / (double)n;
        buf[i].re = (float)cos(phase);
        buf[i].im = (float)sin(phase);
    }

    fft_inplace(buf, n, 0);

    for (size_t k = 0; k < n; k++) {
        float mag = sqrtf(buf[k].re * buf[k].re + buf[k].im * buf[k].im);
        if ((int)k == k0) {
            TF_ASSERT_NEAR(mag, (float)n, 1e-2f);
        } else {
            TF_ASSERT_NEAR(mag, 0.0f, 1e-2f);
        }
    }
}

static void test_fft_forward_inverse_round_trip(void) {
    /* fft_inplace's documented inverse is unnormalized (scaled by n),
     * so inverse(forward(x)) should reconstruct n * x. */
    const size_t n = 16;
    cplx_t original[16], buf[16];
    for (size_t i = 0; i < n; i++) {
        original[i].re = (float)(i % 5) - 2.0f;
        original[i].im = (float)(i % 3) - 1.0f;
        buf[i] = original[i];
    }

    fft_inplace(buf, n, 0);
    fft_inplace(buf, n, 1);

    for (size_t i = 0; i < n; i++) {
        TF_ASSERT_NEAR(buf[i].re, original[i].re * (float)n, 1e-2f);
        TF_ASSERT_NEAR(buf[i].im, original[i].im * (float)n, 1e-2f);
    }
}

int main(void) {
    TF_RUN(test_bit_reverse_3bit);
    TF_RUN(test_hann_window_shape);
    TF_RUN(test_fft_rejects_non_power_of_two);
    TF_RUN(test_fft_dc_input);
    TF_RUN(test_fft_single_tone);
    TF_RUN(test_fft_forward_inverse_round_trip);
    return tf_summary();
}
