"""
angle_estimation.py

Stage 4 of the radar DSP chain: angle-of-arrival (AoA) estimation.

For each CFAR detection cell, we have one complex sample per virtual
receive antenna. A uniform linear array (ULA) with d = lambda/2
spacing produces a linear phase progression across antennas that is
proportional to sin(theta). The classic, cheap way to estimate theta
on an embedded sensor is a zero-padded FFT across the antenna axis
(equivalent to conventional/Bartlett beamforming) -- no matrix
inversion required, which is why this step is included in the Python
model but deliberately *not* ported to the embedded C demo: it shows
the full sensor signal chain while keeping the C deliverable focused
on the FFT/CFAR core the JD calls out explicitly.
"""

import numpy as np
from typing import List, Tuple

try:
    from generate_fmcw_data import RadarConfig
except ImportError:
    from .generate_fmcw_data import RadarConfig


def estimate_angle(
    range_doppler_cube: np.ndarray,
    detection: Tuple[int, int],
    cfg: RadarConfig,
    n_angle_bins: int = 180,
) -> float:
    """Estimate angle-of-arrival in degrees for a single (doppler_idx,
    range_idx) detection using a zero-padded FFT across the antenna
    (Nrx) axis of the range-Doppler cube.
    """
    d_idx, r_idx = detection
    antenna_samples = range_doppler_cube[:, d_idx, r_idx]  # shape (Nrx,)

    spectrum = np.fft.fftshift(np.fft.fft(antenna_samples, n=n_angle_bins))
    peak_bin = int(np.argmax(np.abs(spectrum)))

    # map bin index -> normalized spatial frequency -> angle via asin
    norm_freq = (peak_bin - n_angle_bins / 2) / n_angle_bins  # in [-0.5, 0.5)
    sin_theta = norm_freq / cfg.rx_spacing_lambda
    sin_theta = np.clip(sin_theta, -1.0, 1.0)
    return float(np.degrees(np.arcsin(sin_theta)))


def estimate_angles(
    range_doppler_cube: np.ndarray,
    detections: List[Tuple[int, int]],
    cfg: RadarConfig,
) -> List[float]:
    return [estimate_angle(range_doppler_cube, det, cfg) for det in detections]


if __name__ == "__main__":
    from generate_fmcw_data import generate_iq_cube, Target
    from range_fft import range_fft
    from doppler_fft import doppler_fft, range_doppler_magnitude
    from cfar import CfarConfig, ca_cfar_2d, cluster_detections

    cfg = RadarConfig()
    targets = [Target(range_m=18.0, velocity_mps=12.0, angle_deg=10.0, amplitude=4.0)]
    cube = generate_iq_cube(cfg, targets)
    rd_cube = doppler_fft(range_fft(cube, cfg), cfg)
    mag_map = range_doppler_magnitude(rd_cube)

    dets, _ = ca_cfar_2d(mag_map, CfarConfig())
    dets = cluster_detections(dets, mag_map)
    angles = estimate_angles(rd_cube, dets, cfg)
    for (d, r), a in zip(dets, angles):
        print(f"Detection (doppler={d}, range={r}) -> estimated angle {a:.1f} deg "
              f"(true 10.0 deg)")
