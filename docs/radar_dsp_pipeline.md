# Radar DSP Pipeline

This document explains the signal processing chain implemented in both
`python_model/` and `c_embedded/`, end to end, from raw ADC samples to a
list of detected objects with range, velocity, and angle.

## 1. FMCW waveform and raw data

An FMCW (Frequency-Modulated Continuous Wave) radar transmits a sequence of
linear chirps -- frequency ramps from `f_start` to `f_start + bandwidth`
over `chirp_time` seconds. The receiver mixes the echo with the transmitted
chirp, producing a low-frequency "beat" signal whose frequency is
proportional to the target's range, and whose phase drifts chirp-to-chirp
in proportion to the target's radial velocity. With multiple receive
antennas, the phase also varies antenna-to-antenna in proportion to the
target's angle off boresight.

`python_model/generate_fmcw_data.py` synthesizes this beat signal directly
(skipping RF/mixing simulation, since only the resulting digitized cube
matters for the DSP chain) for one or more point targets, plus additive
receiver noise, producing a `(num_rx, num_chirps, num_samples)` complex
data cube -- the same shape every real radar front-end hands to the DSP
chain.

## 2. Range FFT (fast-time)

A Hann-windowed FFT along the sample axis turns each chirp's beat
frequency into a range bin: range resolution is `c / (2 * bandwidth)`, and
the maximum unambiguous range is set by the ADC sample rate and chirp
slope. Windowing trades a slightly wider mainlobe for much lower sidelobes,
which matters once two targets of very different reflectivity are close
together in range.

Implemented in `python_model/range_fft.py` and `c_embedded/src/range_fft.c`
(via the shared `fft_core` radix-2 FFT).

## 3. Doppler FFT (slow-time)

A second, Hann-windowed FFT along the chirp axis turns the phase
progression across chirps into a velocity bin: velocity resolution is set
by the wavelength and total frame time (`num_chirps * chirp_time`). The
output of range FFT + Doppler FFT is the classic 2-D range-Doppler map; see
`results/range_doppler_map.png` for an example with two targets clearly
visible above the noise floor.

With multiple receive antennas, the per-antenna range-Doppler maps are
combined non-coherently (sum of squared magnitudes, then square root)
before detection, since CFAR operates on a single 2-D magnitude map.
`doppler_fft_accumulate_power()` in the C implementation does this
one antenna at a time rather than holding all antennas in RAM
simultaneously -- see `docs/embedded_architecture.md`.

## 4. CA-CFAR detection

Cell-Averaging Constant False Alarm Rate detection scans every cell of the
range-Doppler map and compares its power against a locally-estimated noise
floor (the average power of a ring of "training" cells around it, excluding
a "guard" band immediately adjacent to avoid a target's own energy
contaminating the estimate). A detection is declared when the cell exceeds
`alpha * noise_estimate`, where `alpha = N * (Pfa^(-1/N) - 1)` is the
closed-form threshold factor for the desired false-alarm probability `Pfa`
and `N` training cells. This adapts the threshold to the local noise level
automatically, which is essential in automotive scenes where the
background can vary by tens of dB between a quiet highway and a cluttered
urban intersection.

Implemented identically (same loop structure, same alpha formula) in
`python_model/cfar.py` and `c_embedded/src/cfar_ca.c` so the two are a fair
benchmark comparison -- see `docs/performance_benchmark.md`.

## 5. Clustering

A real target's energy spreads across several adjacent range-Doppler cells
(its mainlobe footprint plus near sidelobes), so raw CFAR output typically
has several hits per target. Both implementations run a greedy non-max
suppression pass: sort hits by magnitude, keep the strongest, and suppress
any other hit within a small range/Doppler radius of it, repeating until
no hits remain. This collapses each target down to one representative
detection.

## 6. Angle estimation

For each surviving detection, a zero-padded FFT across the (virtual)
receive-antenna axis estimates angle of arrival: the antenna-to-antenna
phase progression caused by a target off boresight is itself a complex
exponential, so an FFT finds its spatial frequency, which maps to angle via
`sin(theta) = (normalized spatial frequency) / (antenna spacing in
wavelengths)`. This stays Python-only in this project (see
`python_model/angle_estimation.py`) since it needs full per-antenna phase
information rather than the integrated power the C demo carries, and the
JD's C/embedded emphasis is on the FFT/CFAR core rather than the angle
stage specifically.

## 7. Validation

`python_model/radar_pipeline_demo.py` runs the full chain in Python on a
two-target synthetic scene, exports the raw cube as
`c_embedded/include/test_vectors.h`, and writes the resulting object list,
range-Doppler plots, and Python-side timings to `results/`.
`c_embedded/src/radar_pipeline.c` runs the same chain in C on that exact
exported data, and `c_embedded/tests/test_pipeline.c` asserts the result
matches the same two ground-truth targets -- so the Python model and the
embedded C port are validated against each other, not just against
themselves.
