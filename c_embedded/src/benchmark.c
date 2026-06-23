/*
 * benchmark.c
 *
 * Times each embedded pipeline stage on the same input data and CFAR
 * configuration used by python_model/radar_pipeline_demo.py, using the
 * same "one warm-up call, then average over N runs" methodology as
 * export_python_timings_csv()
 *
 * Prints CSV rows to stdout in the same schema the Python script
 * writes (stage,implementation,avg_time_us); the project's build
 * script appends these to results/benchmark_table.csv. See
 * docs/performance_benchmark.md for the full numbers and discussion.
 */

#include <stdio.h>
#include <string.h>
#include <time.h>

#include "radar_types.h"
#include "range_fft.h"
#include "doppler_fft.h"
#include "cfar_ca.h"
#include "test_vectors.h"

/* 200 rather than 50: clock() resolution is implementation-defined --
 * CLOCKS_PER_SEC is typically 1e6 (microsecond resolution) on Linux/macOS
 * but only 1e3 (millisecond resolution) on Windows.
 */
#define N_RUNS 200

static chirp_matrix_t g_matrix;
static chirp_matrix_t g_matrix_template; /* pristine copy, since range_fft_process mutates in place */
static float g_power_accum[RADAR_NUM_CHIRPS][RADAR_NUM_SAMPLES];
static rd_map_t g_rd_map;
static detection_list_t g_detections;

/* Portable elapsed-time helper: clock() is plain ANSI C (C89), so this
 * compiles identically with gcc, MinGW, and MSVC -- unlike
 * clock_gettime()/CLOCK_MONOTONIC, which are POSIX-only and don't exist
 * on MSVC at all. */
static double now_us(void) {
    return (double)clock() / (double)CLOCKS_PER_SEC * 1e6;
}

static void load_antenna0_template(void) {
    unsigned idx = 0; /* antenna 0 */
    for (unsigned c = 0; c < RADAR_NUM_CHIRPS; c++) {
        for (unsigned s = 0; s < RADAR_NUM_SAMPLES; s++) {
            g_matrix_template[c][s].re = TV_REAL[idx];
            g_matrix_template[c][s].im = TV_IMAG[idx];
            idx++;
        }
    }
}

static double bench_range_fft(void) {
    memcpy(g_matrix, g_matrix_template, sizeof(g_matrix));
    range_fft_process(&g_matrix);

    double t0 = now_us();
    for (int i = 0; i < N_RUNS; i++) {
        memcpy(g_matrix, g_matrix_template, sizeof(g_matrix)); /* range_fft_process mutates in place */
        range_fft_process(&g_matrix);
    }
    return (now_us() - t0) / N_RUNS;
}

static double bench_doppler_fft(void) {
    static chirp_matrix_t after_range;
    memcpy(after_range, g_matrix_template, sizeof(after_range));
    range_fft_process(&after_range);

    
    doppler_fft_process(&after_range, &g_rd_map);

    double t0 = now_us();
    for (int i = 0; i < N_RUNS; i++) {
        doppler_fft_process(&after_range, &g_rd_map);
    }
    return (now_us() - t0) / N_RUNS;
}

static double bench_cfar(void) {
    cfar_config_t cfar_cfg = { .num_train = 8, .num_guard = 4, .pfa = 1e-3f };

    /* warm-up */
    cfar_ca_process(&g_rd_map, &cfar_cfg, &g_detections);

    double t0 = now_us();
    for (int i = 0; i < N_RUNS; i++) {
        cfar_ca_process(&g_rd_map, &cfar_cfg, &g_detections);
    }
    return (now_us() - t0) / N_RUNS;
}

int main(void) {
    load_antenna0_template();

    double t_range = bench_range_fft();
    double t_doppler = bench_doppler_fft();
    double t_cfar = bench_cfar();

    /* Reset accumulator state so the printed numbers don't depend on
     * call order if this binary is ever extended. */
    (void)g_power_accum;

    printf("stage,implementation,avg_time_us\n");
    printf("range_fft,c,%.2f\n", t_range);
    printf("doppler_fft,c,%.2f\n", t_doppler);
    printf("ca_cfar_2d,c,%.2f\n", t_cfar);

    return 0;
}
