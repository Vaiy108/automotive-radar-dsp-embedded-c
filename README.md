# Automotive Radar DSP Pipeline: Python Prototyping to Embedded C

A complete FMCW automotive radar signal-processing pipeline, built twice:
once in Python for algorithm design and validation, once in dependency-free
embedded C for the target platform -- with both validated against each
other on identical input data, and benchmarked against each other on
identical algorithms.

Part of a broader radar perception pipeline that also includes a separate
radar-camera fusion project -- this piece is the front-end signal
processing stage: turning raw FMCW ADC samples into a validated,
benchmarked range/velocity/angle object list, in both Python and embedded
C, ready to feed into a tracker or fusion layer downstream.

## Pipeline

```
simulated raw IQ cube  -->  range FFT  -->  Doppler FFT  -->  CA-CFAR  -->  clustering  -->  angle estimation  -->  object list
   (synthetic FMCW)         (fast-time)     (slow-time)    (detection)   (non-max suppr.)    (Python only)
```

Both implementations run this chain (the C side omits angle estimation by
design -- see [`docs/radar_dsp_pipeline.md`](docs/radar_dsp_pipeline.md)).
Full explanation of the radar physics and each stage's algorithm is in
that doc.

![Range-Doppler map with two targets](results/range_doppler_map.png)

## Repository layout

```
automotive-radar-dsp-embedded-c/
├── python_model/          Algorithm design & validation (numpy/matplotlib)
│   ├── generate_fmcw_data.py   FMCW radar raw-data simulator
│   ├── range_fft.py            Stage 1: fast-time FFT
│   ├── doppler_fft.py          Stage 2: slow-time FFT + non-coherent integration
│   ├── cfar.py                 Stage 3: CA-CFAR detection + clustering
│   ├── angle_estimation.py     Stage 4: angle-of-arrival via spatial FFT
│   └── radar_pipeline_demo.py  End-to-end demo; generates results/ and test_vectors.h
│
├── c_embedded/             Embedded C port (no malloc, no OS, no external deps)
│   ├── include/                 Shared headers + auto-generated test_vectors.h
│   ├── src/                     fft_core, range_fft, doppler_fft, cfar_ca, radar_pipeline, benchmark
│   └── tests/                   Standalone unit + integration test binaries
│
├── docs/
│   ├── radar_dsp_pipeline.md       Full pipeline explanation
│   ├── embedded_architecture.md    Embedded design decisions + a real bug fixed
│   └── performance_benchmark.md    Python vs. C methodology and results
│
├── results/                Generated plots, object list, benchmark CSV
└── CMakeLists.txt
```

## Building and running

### Python model

```bash
cd python_model
pip install numpy scipy matplotlib
python3 radar_pipeline_demo.py
```

Prints the detected objects and Python-side stage timings, and writes
`results/range_doppler_map.png`, `results/cfar_detection.png`,
`results/object_list.txt`, `results/benchmark_table.csv`, and
`c_embedded/include/test_vectors.h`.

### C pipeline and tests

```bash
mkdir build && cd build
cmake ..
cmake --build .
./radar_pipeline       # runs the same scene as the Python demo, prints detections
./benchmark             # appends C-side timings to results/benchmark_table.csv
ctest                   # runs test_fft, test_cfar, test_pipeline
```

(Validated in development with direct `gcc`/`make` invocations rather than
CMake, since the development sandbox didn't have CMake installed; the
`CMakeLists.txt` mirrors that same build with no special flags, so
`cmake .. && cmake --build .` is expected to behave identically on a
standard toolchain. See individual `gcc` commands in
[`docs/performance_benchmark.md`](docs/performance_benchmark.md) if you
want to reproduce the exact validation commands.)

### Windows

The C code only depends on `<math.h>`, `<string.h>`, `<stdio.h>`,
`<stdint.h>`, and ANSI `<time.h>` -- no POSIX-only calls -- so it builds
with either MSVC (Visual Studio) or MinGW-w64/gcc. From an
**x64 Native Tools Command Prompt for VS** (installed alongside Visual
Studio's C++ workload):

```bat
cd c_embedded
cl /O2 /std:c17 /I include src\fft_core.c src\range_fft.c src\doppler_fft.c src\cfar_ca.c src\radar_pipeline.c /Fe:radar_pipeline.exe
radar_pipeline.exe
```

`/std:c17` requires Visual Studio 2019 16.8+; drop the flag on older
versions (designated initializers in this codebase still build fine
under MSVC's default C dialect). MSVC links the math library
automatically, so no `-lm` equivalent is needed.

## Validation

The C pipeline is checked against the Python reference model on identical
input, not just against itself:

- `c_embedded/include/test_vectors.h` is exported directly from the Python
  model's exact synthetic scene.
- `c_embedded/tests/test_pipeline.c` asserts the C output matches the same
  two ground-truth targets the Python script reports.
- Running both produces near-identical detections (range/velocity within
  a fraction of a bin, magnitude matching to 3 significant figures).

## Performance

| Stage | Python | C | Speedup |
|---|---|---|---|
| Range FFT | ~1.3 ms | ~0.6 ms | ~2.2x |
| Doppler FFT | ~2.1 ms | ~0.6 ms | ~3.5x |
| CA-CFAR | ~131 ms | ~20 ms | ~6.5x |

Full methodology, why the FFT speedup isn't larger, and a documented
twiddle-factor-caching optimization that doubled FFT throughput partway
through development: [`docs/performance_benchmark.md`](docs/performance_benchmark.md).

## Notable design decisions

- **No dynamic memory anywhere in `c_embedded/`** -- every buffer is a
  fixed-size static/stack array, matching how most automotive radar
  MCU/DSP targets handle the signal-processing task.
- **A real bug, found and fixed**: an early, too-small CFAR detection
  buffer silently dropped a true-peak detection. Full write-up, the fix,
  and the regression test that now guards against it:
  [`docs/embedded_architecture.md`](docs/embedded_architecture.md).
- **One antenna resident in RAM at a time** during multi-antenna
  non-coherent integration, rather than holding every antenna's full chirp
  matrix simultaneously.
