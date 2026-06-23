/*
 * fft_core.c -- see fft_core.h for design notes.
 */

#include "fft_core.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static int is_power_of_two(size_t n) {
    return n != 0 && (n & (n - 1)) == 0;
}

static unsigned log2_u(size_t n) {
    unsigned r = 0;
    while (n > 1u) {
        n >>= 1;
        r++;
    }
    return r;
}

unsigned fft_bit_reverse(unsigned x, unsigned log2_n) {
    unsigned result = 0;
    for (unsigned i = 0; i < log2_n; i++) {
        result = (result << 1) | (x & 1u);
        x >>= 1;
    }
    return result;
}

void hann_window(float *win, size_t n) {
    if (n == 1) {
        win[0] = 1.0f;
        return;
    }
    for (size_t i = 0; i < n; i++) {
        win[i] = 0.5f - 0.5f * cosf((float)(2.0 * M_PI * (double)i / (double)(n - 1)));
    }
}

/*
 * Precomputed twiddle-factor table.
  *
 * Build a single base table of cos/sin(2*pi*i/MAX_FFT_LEN) once,
 * lazily, on first use, sized MAX_FFT_LEN/2. Every FFT length and
 * every stage size m used in this project is a power of two no larger
 * than MAX_FFT_LEN, so m always divides MAX_FFT_LEN, and the twiddle
 * for stage size m at index j is exactly base[j * (MAX_FFT_LEN / m)]
 * -- no recomputation, no malloc, no runtime growth. This is the same
 * fixed-table approach real FFT libraries use on DSP/MCU cores.
 */
#define MAX_FFT_LEN 256u /* must be >= the largest n passed to fft_inplace */

static float g_twiddle_cos[MAX_FFT_LEN / 2];
static float g_twiddle_sin[MAX_FFT_LEN / 2]; /* sin(-2*pi*i/MAX_FFT_LEN), i.e. the forward-transform sign */
static int g_twiddle_ready = 0;

static void build_twiddle_table(void) {
    for (unsigned i = 0; i < MAX_FFT_LEN / 2; i++) {
        float angle = -2.0f * (float)M_PI * (float)i / (float)MAX_FFT_LEN; /* forward convention */
        g_twiddle_cos[i] = cosf(angle);
        g_twiddle_sin[i] = sinf(angle);
    }
    g_twiddle_ready = 1;
}

int fft_inplace(cplx_t *buf, size_t n, int inverse) {
    if (!is_power_of_two(n) || n > MAX_FFT_LEN) {
        return -1;
    }
    if (!g_twiddle_ready) {
        build_twiddle_table();
    }

    unsigned log2_n = log2_u(n);

    /* Bit-reversal permutation */
    for (unsigned i = 0; i < n; i++) {
        unsigned j = fft_bit_reverse(i, log2_n);
        if (j > i) {
            cplx_t tmp = buf[i];
            buf[i] = buf[j];
            buf[j] = tmp;
        }
    }

    const unsigned stride = (unsigned)(MAX_FFT_LEN / n); /* table index step for this FFT length */
    const float inv_sign = inverse ? -1.0f : 1.0f;        /* flips sin() sign for the inverse transform */

    /* Iterative Cooley-Tukey, stage by stage */
    for (unsigned stage = 1; stage <= log2_n; stage++) {
        unsigned m = 1u << stage;          /* size of each sub-DFT this stage */
        unsigned half_m = m >> 1;
        unsigned base_step = stride * ((unsigned)n / m); /* index multiplier: j -> j*base_step into the shared table */

        for (unsigned k = 0; k < n; k += m) {
            for (unsigned j = 0; j < half_m; j++) {
                unsigned t_idx = j * base_step;
                cplx_t w = { g_twiddle_cos[t_idx], inv_sign * g_twiddle_sin[t_idx] };

                cplx_t *a = &buf[k + j];
                cplx_t *b = &buf[k + j + half_m];

                /* t = w * b */
                cplx_t t;
                t.re = w.re * b->re - w.im * b->im;
                t.im = w.re * b->im + w.im * b->re;

                cplx_t a_old = *a;
                a->re = a_old.re + t.re;
                a->im = a_old.im + t.im;
                b->re = a_old.re - t.re;
                b->im = a_old.im - t.im;
            }
        }
    }

    return 0;
}
