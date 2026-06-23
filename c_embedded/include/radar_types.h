/*
 * radar_types.h
 *
 * Fixed-size, allocation-free type definitions shared by every stage
 * of the embedded DSP pipeline. Sizes are compile-time constants
 * (matching python_model/generate_fmcw_data.py::RadarConfig exactly)
 * so every buffer in the pipeline is statically sized -- no malloc/free
 * anywhere in this codebase, which is a hard requirement on most
 * automotive radar MCU/DSP targets.
 */

#ifndef RADAR_TYPES_H
#define RADAR_TYPES_H

#include <stdint.h>

/* ---- Must match python_model/generate_fmcw_data.py::RadarConfig ---- */
#define RADAR_NUM_SAMPLES   256u   /* ADC samples per chirp (range FFT size)  */
#define RADAR_NUM_CHIRPS    128u   /* chirps per frame (Doppler FFT size)     */
#define RADAR_NUM_RX        4u     /* virtual receive antennas                */

#define RADAR_RANGE_RESOLUTION_M     0.3f
#define RADAR_VELOCITY_RESOLUTION_MPS 0.7609f

typedef struct {
    float re;
    float im;
} cplx_t;

/* Single-antenna chirp matrix: [chirp][sample] */
typedef cplx_t chirp_matrix_t[RADAR_NUM_CHIRPS][RADAR_NUM_SAMPLES];

/* Range-Doppler magnitude map (non-coherent power, single antenna here) */
typedef float rd_map_t[RADAR_NUM_CHIRPS][RADAR_NUM_SAMPLES];

typedef struct {
    uint16_t doppler_idx;
    uint16_t range_idx;
    float    magnitude;
} detection_t;

/* Raw CA-CFAR output on this scene size/config measures ~40-45 hits
 * before clustering (each target's mainlobe + near sidelobes light up
 * several adjacent cells). 32 was too tight and silently dropped the
 * true peak of one target once the fixed buffer filled -- see
 * docs/embedded_architecture.md for the full writeup of this bug and
 * why a fixed-size bound still needs a measured safety margin rather
 * than a round-number guess. */
#define RADAR_MAX_DETECTIONS 128u

typedef struct {
    detection_t detections[RADAR_MAX_DETECTIONS];
    uint16_t    count;
    uint8_t     overflowed;  /* 1 if more raw hits existed than the buffer
                               * could hold -- see cfar_ca.c. A fixed-size
                               * embedded buffer must never truncate silently. */
} detection_list_t;

#endif /* RADAR_TYPES_H */
