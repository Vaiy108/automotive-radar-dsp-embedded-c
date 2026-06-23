# Performance Benchmark: Python vs. C

## Methodology

Both languages run the exact same algorithm (same window function, same
FFT length, same CFAR training/guard geometry, same closed-form alpha) on
the exact same input data, so the comparison isolates the effect of
compiled C vs. interpreted Python rather than any algorithmic difference.
Each stage is timed with one untimed warm-up call followed by an average
over multiple repeated calls:

- Python: `time.perf_counter()` around each call in
  `python_model/radar_pipeline_demo.py::export_python_timings_csv()`,
  averaged over 20 runs.
- C: `clock()` (plain ANSI C, `<time.h>`) around each call in
  `c_embedded/src/benchmark.c`, averaged over 200 runs so the measured
  interval stays well-resolved even on platforms where `clock()` has
  millisecond granularity (Windows) rather than microsecond (Linux/macOS).

An earlier version of benchmark.c used POSIX `clock_gettime`/`CLOCK_MONOTONIC`,
which doesn't exist on MSVC at all. `clock()` compiles identically across
gcc, MinGW, and MSVC, which matters for a project that targets multiple
toolchains.

Both write to the same `results/benchmark_table.csv` schema
(`stage,implementation,avg_time_us`) so the numbers sit side by side.

These numbers are measured on development hardware, not on actual radar
sensor silicon, so the absolute microsecond values won't transfer 1:1
to a real DSP/MCU target -- what matters, and what does transfer, is the
relative speedup and *why* it occurs.

## Results

Measured on **Windows 10, Intel x64, MSVC 19.50 /O2 vs. Python 3.12
(Anaconda)**:

| Stage | Python (us) | C (us) | Speedup |
|---|---|---|---|
| Range FFT | 4779.7 | 695.0 | **6.9x** |
| Doppler FFT | 5976.4 | 780.0 | **7.7x** |
| CA-CFAR | 226468.1 | 22340.0 | **10.1x** |


The CA-CFAR speedup is the most consistent and most meaningful number
across platforms -- see the section below for why.

## Why the FFT speedup varies so much between platforms (~7x on Windows)


On **Windows with Anaconda**, numpy carries more per-call overhead
(DLL loading, memory allocator behavior, Python/C boundary crossing) and
MSVC's `/O2` auto-vectorizer is aggressive on x64. The result is that
the same C code wins by a larger margin against the same algorithm running
in Python, even though the underlying work hasn't changed.

Neither set of numbers is wrong -- they're accurate measurements of their
respective environments. The Linux numbers are the more conservative and
more credible lower bound for "how much faster is compiled C."

One historical note: the first version of `fft_core.c`'s butterfly loop
computed `cosf()`/`sinf()` for every `(block, j)` pair in each FFT stage,
even though the twiddle angle depends only on `j`, not on which block is
being processed -- so the same handful of twiddle values were being
recomputed repeatedly. That version benchmarked essentially even with numpy
on Linux. Replacing it with a single precomputed twiddle-factor lookup table
(built once, shared across both the 256-point range FFT and the 128-point
Doppler FFT -- see `c_embedded/src/fft_core.c`) removed that bottleneck.
Identical detection output was verified by `test_fft.c` and `test_pipeline.c`
before and after the change.



## Why CA-CFAR shows the largest and most consistent speedup

CFAR has no numpy-equivalent shortcut available: both `python_model/cfar.py`
and `c_embedded/src/cfar_ca.c` use the same nested-loop,
recompute-the-window-sum-per-cell approach. With identical O(window_area)
per-cell algorithms on both sides, the gap is a direct measurement of
"interpreted nested Python loops" vs. "compiled C nested loops" on exactly
the same work -- with no vectorized library call on either side to blur the
comparison.

This is also the most practically relevant number: CFAR-style detection
logic is exactly the kind of per-cell, control-flow-heavy code that ends
up on the host CPU core (the Cortex-M7 or Cortex-A53 on a target like the
NXP S32R45) rather than on a hardware accelerator. Knowing it runs ~10x
faster in C than in Python on this hardware is a direct answer to "what
does the Python-to-C porting workflow actually buy"
