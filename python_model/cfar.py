"""
cfar.py

Stage 3 of the radar DSP chain: Cell-Averaging CFAR (CA-CFAR) detection
on the 2-D range-Doppler magnitude map.

For each cell under test (CUT), the local noise floor is estimated by
averaging the power of the surrounding "training" cells (excluding a
"guard" band immediately around the CUT, which protects against a
target's own energy leaking into the noise estimate). A detection is
declared if the CUT power exceeds (noise_estimate * alpha), where
alpha is derived from the desired probability of false alarm (Pfa)
and the number of training cells -- this is the same closed-form
threshold factor used in the C implementation, so Python and C should
produce numerically matching detections on the same input.

This is deliberately implemented with explicit nested loops (rather
than a vectorized convolution) because the C port in
c_embedded/src/cfar_ca.c mirrors this exact loop structure -- the goal
is a 1:1 algorithmic match between prototype and embedded code, not
the fastest possible Python implementation.
"""

import numpy as np
from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class CfarConfig:
    num_train: int = 8     # training cells on each side, per axis
    num_guard: int = 4     # guard cells on each side, per axis
    pfa: float = 1e-3      # desired probability of false alarm


def cfar_alpha(num_training_cells: int, pfa: float) -> float:
    """Closed-form CA-CFAR threshold multiplier for square-law detectors:
        alpha = N * (Pfa^(-1/N) - 1)
    """
    n = num_training_cells
    return n * (pfa ** (-1.0 / n) - 1.0)


def ca_cfar_2d(mag_map: np.ndarray, cfg: CfarConfig) -> Tuple[List[Tuple[int, int]], np.ndarray]:
    """Run 2-D CA-CFAR over a (Ndoppler, Nrange) magnitude map.

    Returns:
        detections: list of (doppler_idx, range_idx) integer index pairs
        threshold_map: the per-cell threshold actually used (for plotting)
    """
    power_map = mag_map.astype(np.float64) ** 2
    n_dop, n_rng = power_map.shape

    g, t = cfg.num_guard, cfg.num_train
    win = g + t

    num_train_cells = (2 * win + 1) ** 2 - (2 * g + 1) ** 2
    alpha = cfar_alpha(num_train_cells, cfg.pfa)

    threshold_map = np.zeros_like(power_map)
    detections = []

    for d in range(win, n_dop - win):
        for r in range(win, n_rng - win):
            window_block = power_map[d - win:d + win + 1, r - win:r + win + 1]
            guard_block = power_map[d - g:d + g + 1, r - g:r + g + 1]
            train_sum = window_block.sum() - guard_block.sum()
            noise_est = train_sum / num_train_cells
            threshold = alpha * noise_est
            threshold_map[d, r] = threshold

            if power_map[d, r] > threshold:
                detections.append((d, r))

    return detections, np.sqrt(threshold_map)


def cluster_detections(
    detections: List[Tuple[int, int]],
    mag_map: np.ndarray,
    min_separation: int = 2,
) -> List[Tuple[int, int]]:
    """Greedy non-max suppression: a target's energy typically lights
    up several adjacent CFAR cells (its mainlobe footprint), so we
    collapse each footprint down to its single strongest cell.

    Sorting by magnitude and suppressing neighbours of the strongest
    remaining cell first (rather than arbitrarily merging whichever
    cells happen to be visited first) makes the result deterministic
    and guarantees the reported detection is the true local peak --
    this also matches the equivalent logic in c_embedded/src/radar_pipeline.c,
    so both implementations pick the same representative cell.
    """
    if not detections:
        return []

    ordered = sorted(detections, key=lambda dr: mag_map[dr[0], dr[1]], reverse=True)
    suppressed = set()
    peaks = []

    for d, r in ordered:
        if (d, r) in suppressed:
            continue
        peaks.append((d, r))
        for d2, r2 in detections:
            if abs(d2 - d) <= min_separation and abs(r2 - r) <= min_separation:
                suppressed.add((d2, r2))

    return peaks


if __name__ == "__main__":
    from generate_fmcw_data import RadarConfig, generate_iq_cube, Target
    from range_fft import range_fft
    from doppler_fft import doppler_fft, range_doppler_magnitude

    cfg = RadarConfig()
    targets = [
        Target(range_m=18.0, velocity_mps=12.0, amplitude=4.0),
        Target(range_m=30.0, velocity_mps=-8.0, amplitude=2.5),
    ]
    cube = generate_iq_cube(cfg, targets)
    mag_map = range_doppler_magnitude(doppler_fft(range_fft(cube, cfg), cfg))

    cfar_cfg = CfarConfig()
    dets, _ = ca_cfar_2d(mag_map, cfar_cfg)
    dets = cluster_detections(dets, mag_map)
    print(f"Raw CFAR detections (clustered): {len(dets)}")
    for d, r in dets:
        print(f"  doppler_idx={d}, range_idx={r}")
