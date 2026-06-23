"""
doppler_fft.py

Stage 2 of the radar DSP chain: slow-time (Doppler) FFT.

Applied along the chirp axis of the range-FFT output to resolve the
phase progression between consecutive chirps into a velocity bin.
A second FFT then turns the (chirp, range) data into a full
range-Doppler map, which is the classic 2-D representation used for
CFAR detection on automotive radar sensors.

Input:  range_cube      complex64 (Nrx, Nchirp, Nrange_bins)
Output: range_doppler    complex64 (Nrx, Ndoppler_bins, Nrange_bins)
        doppler_axis_mps  float64  (Ndoppler_bins,)
"""

import numpy as np

try:
    from generate_fmcw_data import RadarConfig
except ImportError:
    from .generate_fmcw_data import RadarConfig


def doppler_fft(range_cube: np.ndarray, cfg: RadarConfig, window: bool = True) -> np.ndarray:
    """Apply windowed FFT along the slow-time (chirp) axis and
    fft-shift so that zero velocity sits in the centre of the map,
    matching how a range-Doppler map is conventionally displayed.
    """
    n_chirps = range_cube.shape[1]
    data = range_cube
    if window:
        win = np.hanning(n_chirps).astype(np.float32)
        data = data * win[None, :, None]

    spectrum = np.fft.fft(data, axis=1)
    spectrum = np.fft.fftshift(spectrum, axes=1)
    return spectrum.astype(np.complex64)


def doppler_axis(cfg: RadarConfig) -> np.ndarray:
    bin_idx = np.arange(-cfg.num_chirps // 2, cfg.num_chirps // 2)
    return bin_idx * cfg.velocity_resolution


def range_doppler_magnitude(range_doppler_cube: np.ndarray) -> np.ndarray:
    """Non-coherently integrate (sum power) across the receive antenna
    axis to get a single 2-D magnitude map for CFAR detection, which
    mirrors how a real sensor's detection stage works on the combined
    channel energy.
    """
    power = np.abs(range_doppler_cube) ** 2
    integrated = np.sum(power, axis=0)
    return np.sqrt(integrated)


if __name__ == "__main__":
    from generate_fmcw_data import generate_iq_cube, Target
    from range_fft import range_fft, range_axis

    cfg = RadarConfig()
    targets = [Target(range_m=18.0, velocity_mps=12.0, amplitude=4.0)]
    cube = generate_iq_cube(cfg, targets)
    rfft = range_fft(cube, cfg)
    rd = doppler_fft(rfft, cfg)
    mag_map = range_doppler_magnitude(rd)

    peak_flat = int(np.argmax(mag_map))
    d_idx, r_idx = np.unravel_index(peak_flat, mag_map.shape)
    print(f"Range-Doppler map shape: {mag_map.shape}")
    print(f"Peak at range {range_axis(cfg)[r_idx]:.2f} m, "
          f"velocity {doppler_axis(cfg)[d_idx]:.2f} m/s "
          f"(true: 18.00 m, 12.00 m/s)")
