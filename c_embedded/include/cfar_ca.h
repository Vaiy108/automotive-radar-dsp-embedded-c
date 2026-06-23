/*
 * cfar_ca.h
 *
 * Stage 3 of the embedded DSP pipeline: Cell-Averaging CFAR.
 * Algorithmic twin of python_model/cfar.py -- same training/guard
 * window geometry and the same closed-form alpha = N*(Pfa^(-1/N) - 1)
 * threshold factor, so a detection list produced here should match
 * the Python reference model bin-for-bin on identical input.
 */

#ifndef CFAR_CA_H
#define CFAR_CA_H

#include "radar_types.h"

typedef struct {
    unsigned num_train;  /* training cells per side, per axis */
    unsigned num_guard;  /* guard cells per side, per axis     */
    float    pfa;        /* desired probability of false alarm */
} cfar_config_t;

/* Runs 2-D CA-CFAR over rd_map and appends detections (up to
 * RADAR_MAX_DETECTIONS) into out_list. out_list->count is reset to 0
 * at the start of the call. */
void cfar_ca_process(const rd_map_t *rd_map, const cfar_config_t *cfg,
                      detection_list_t *out_list);

#endif /* CFAR_CA_H */
