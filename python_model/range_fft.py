"""
range_fft.py

Stage 1 of the radar DSP chain: fast-time (range) FFT.

Applied along the sample axis of the IQ cube to resolve the beat
frequency of each target into a range bin. A Hann window is applied
first to suppress spectral leakage / sidelobes, a standard
practice in automotive radar processing chains.

Input:  iq_cube      complex64 (Nrx, Nchirp, Nsample)
Output: range_cube    complex64 (Nrx, Nchirp, Nrange_bins)
        range_axis_m  float64  (Nrange_bins,)
"""

import numpy as np

try:
    from generate_fmcw_data import RadarConfig
except ImportError:  # allow running as part of a package
    from .generate_fmcw_data import RadarConfig


def range_fft(iq_cube: np.ndarray, cfg: RadarConfig, window: bool = True) -> np.ndarray:
    """Apply windowed FFT along the fast-time (sample) axis.

    Only the positive-beat-frequency half of the spectrum is physically
    meaningful for a real FMCW beat signal, but since the simulated
    cube is already complex baseband IQ,  the full FFT length is kept
    and `range_axis_m` describe the corresponding physical ranges.
    """
    n_samples = iq_cube.shape[-1]
    data = iq_cube
    if window:
        win = np.hanning(n_samples).astype(np.float32)
        data = data * win[None, None, :]

    spectrum = np.fft.fft(data, axis=-1)
    return spectrum.astype(np.complex64)


def range_axis(cfg: RadarConfig) -> np.ndarray:
    bin_idx = np.arange(cfg.num_samples)
    return bin_idx * cfg.range_resolution


if __name__ == "__main__":
    from generate_fmcw_data import generate_iq_cube, Target

    cfg = RadarConfig()
    targets = [Target(range_m=18.0, velocity_mps=12.0, amplitude=4.0)]
    cube = generate_iq_cube(cfg, targets)
    rfft = range_fft(cube, cfg)
    mag = np.abs(rfft[0, 0, :])
    peak_bin = int(np.argmax(mag))
    axis = range_axis(cfg)
    print(f"Range FFT output shape: {rfft.shape}")
    print(f"Peak bin {peak_bin} -> estimated range {axis[peak_bin]:.2f} m "
          f"(true range 18.00 m)")
