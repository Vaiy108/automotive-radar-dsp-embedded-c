"""
generate_fmcw_data.py

Simulates the raw IQ data a 77 GHz automotive FMCW radar front-end would
hand to the digital signal processing chain. This stands in for the
ADC capture stage of a real sensor: everything downstream (range FFT,
Doppler FFT, CFAR, angle estimation) consumes exactly the same data
shape that a real radar front-end (e.g. an AFE + ADC on the sensor PCB)
would produce.

Data cube convention: complex64 array of shape (Nrx, Nchirp, Nsample)
  - Nrx      : number of (virtual) receive antennas
  - Nchirp   : number of chirps per frame (slow time / Doppler axis)
  - Nsample  : ADC samples per chirp (fast time / range axis)
"""

import numpy as np
from dataclasses import dataclass, field
from typing import List, Tuple

C_LIGHT = 299_792_458.0  # m/s


@dataclass
class RadarConfig:
    fc: float = 77.0e9          # carrier frequency [Hz]
    bandwidth: float = 500.0e6  # chirp sweep bandwidth [Hz]
    chirp_time: float = 20.0e-6  # active chirp duration [s]
    num_samples: int = 256      # ADC samples per chirp (range FFT size)
    num_chirps: int = 128       # chirps per frame (Doppler FFT size)
    num_rx: int = 4             # virtual receive antennas (angle axis)
    rx_spacing_lambda: float = 0.5  # antenna spacing in units of wavelength

    @property
    def wavelength(self) -> float:
        return C_LIGHT / self.fc

    @property
    def slope(self) -> float:
        """Chirp slope S = B / Tc [Hz/s]."""
        return self.bandwidth / self.chirp_time

    @property
    def sample_rate(self) -> float:
        return self.num_samples / self.chirp_time

    @property
    def range_resolution(self) -> float:
        return C_LIGHT / (2.0 * self.bandwidth)

    @property
    def max_range(self) -> float:
        # Because the simulated front-end produces complex (IQ) baseband
        # samples rather than a real-valued ADC capture, the beat-frequency
        # spectrum has no Hermitian symmetry to discard: all num_samples
        # FFT bins map to distinct unambiguous ranges.
        return self.range_resolution * self.num_samples

    @property
    def velocity_resolution(self) -> float:
        frame_time = self.num_chirps * self.chirp_time
        return self.wavelength / (2.0 * frame_time)

    @property
    def max_velocity(self) -> float:
        return self.velocity_resolution * self.num_chirps / 2.0


@dataclass
class Target:
    range_m: float
    velocity_mps: float
    angle_deg: float = 0.0
    amplitude: float = 1.0


def generate_iq_cube(
    cfg: RadarConfig,
    targets: List[Target],
    snr_db: float = 15.0,
    seed: int = 42,
) -> np.ndarray:
    """Build the simulated raw IQ cube (Nrx, Nchirp, Nsample) for a list
    of point targets, including thermal noise.

    The single-target beat signal model used here is the standard FMCW
    approximation:

        s(n, m, k) = A * exp(j*2*pi*(2*S*R/c)*n/fs)      -- range (fast time)
                       * exp(j*4*pi*v*fc*Tc*m/c)          -- Doppler (slow time)
                       * exp(j*2*pi*k*d_lambda*sin(theta)) -- angle (antenna)

    where n indexes ADC samples, m indexes chirps, k indexes RX antennas.
    """
    rng = np.random.default_rng(seed)
    n = np.arange(cfg.num_samples)
    m = np.arange(cfg.num_chirps)
    k = np.arange(cfg.num_rx)

    cube = np.zeros((cfg.num_rx, cfg.num_chirps, cfg.num_samples), dtype=np.complex64)

    for tgt in targets:
        f_beat = 2.0 * cfg.slope * tgt.range_m / C_LIGHT
        range_phase = np.exp(1j * 2 * np.pi * f_beat * n / cfg.sample_rate)

        doppler_phase = np.exp(
            1j * 4 * np.pi * tgt.velocity_mps * cfg.fc * cfg.chirp_time * m / C_LIGHT
        )

        theta = np.deg2rad(tgt.angle_deg)
        angle_phase = np.exp(
            1j * 2 * np.pi * cfg.rx_spacing_lambda * k * np.sin(theta)
        )

        # outer product across (rx, chirp, sample)
        contribution = (
            tgt.amplitude
            * angle_phase[:, None, None]
            * doppler_phase[None, :, None]
            * range_phase[None, None, :]
        )
        cube += contribution.astype(np.complex64)

    # Additive complex white Gaussian noise, scaled relative to the
    # strongest target so snr_db is meaningful.
    peak_amp = max((t.amplitude for t in targets), default=1.0)
    signal_power = peak_amp ** 2 / 2.0
    noise_power = signal_power / (10 ** (snr_db / 10.0))
    noise_std = np.sqrt(noise_power)
    noise = noise_std * (
        rng.standard_normal(cube.shape) + 1j * rng.standard_normal(cube.shape)
    )
    cube += noise.astype(np.complex64)

    return cube


if __name__ == "__main__":
    cfg = RadarConfig()
    targets = [
        Target(range_m=18.0, velocity_mps=12.0, angle_deg=10.0, amplitude=4.0),
        Target(range_m=30.0, velocity_mps=-8.0, angle_deg=-15.0, amplitude=2.5),
    ]
    cube = generate_iq_cube(cfg, targets)
    print("Radar config:")
    print(f"  range resolution   : {cfg.range_resolution:.3f} m")
    print(f"  max range          : {cfg.max_range:.1f} m")
    print(f"  velocity resolution: {cfg.velocity_resolution:.3f} m/s")
    print(f"  max velocity       : {cfg.max_velocity:.1f} m/s")
    print(f"IQ cube shape (Nrx, Nchirp, Nsample): {cube.shape}, dtype={cube.dtype}")
