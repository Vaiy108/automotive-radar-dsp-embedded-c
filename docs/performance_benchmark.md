# Performance Benchmark: Python vs. C

## Methodology

Both languages run the exact same algorithm (same window function, same
FFT length, same CFAR training/guard geometry, same closed-form alpha) on
the exact same input data, so the comparison isolates the effect of
compiled C vs. interpreted Python rather than any algorithmic difference.
Each stage is timed with one untimed warm-up call followed by an average
over multiple repeated calls (20 runs in Python, 50 in C, since the C
calls are individually much shorter):

- Python: `time.perf_counter()` around each call in
  `python_model/radar_pipeline_demo.py::export_python_timings_csv()`.
- C: `clock()` (plain ANSI C, `<time.h>`) around each call in
  `c_embedded/src/benchmark.c`, averaged over 200 runs rather than 50 so
  the measured interval stays well-resolved even on platforms where
  `clock()` only has millisecond granularity (Windows) rather than
  microsecond (Linux/macOS). An earlier version used POSIX
  `clock_gettime`/`CLOCK_MONOTONIC`, which doesn't exist on MSVC at all;
  `clock()` compiles identically across gcc, MinGW, and MSVC, which
  matters for a project that claims to target "multiple sensor
  platforms."

Both write to the same `results/benchmark_table.csv` schema
(`stage,implementation,avg_time_us`) so the numbers sit side by side.

These numbers were measured on a shared cloud development sandbox, not on
actual radar sensor silicon, so the absolute microsecond values won't
transfer 1:1 to a real DSP/MCU target -- what matters, and what does
transfer, is the relative speedup and *why* it occurs.

## Results

| Stage | Python (us) | C (us) | Speedup |
|---|---|---|---|
| Range FFT | 1347.1 | 611.8 | ~2.2x |
| Doppler FFT | 2090.8 | 593.5 | ~3.5x |
| CA-CFAR | 131101.3 | 20252.3 | ~6.5x |

(Re-run `python3 python_model/radar_pipeline_demo.py` then the
`benchmark` binary to regenerate `results/benchmark_table.csv` with
current numbers; run-to-run variance on a shared sandbox is real --
repeated runs during development saw range-FFT speedup anywhere from
~1.5x to ~2.6x and Doppler-FFT speedup from ~2.8x to ~4x -- but CA-CFAR's
~6-7x speedup was consistent across every run, and the qualitative
finding below (a redundant trig computation, not algorithmic complexity,
was the FFT bottleneck) held in every measurement.)

## Why the FFT speedup is "only" ~2-3x, not larger

A naive expectation is that compiled C should beat interpreted Python by
one or two orders of magnitude. For the FFT stages here, it doesn't,
because numpy's `fft.fft()` is itself a highly optimized, vectorized C/Fortran
routine (pocketfft) operating on a batch of chirps in one call -- it is not
"slow Python" being compared against "fast C," it's "optimized C, called
from Python" against "this project's own scalar C FFT."

The first version of `fft_core.c`'s butterfly loop computed `cosf()`/`sinf()`
for every `(block, j)` pair in each FFT stage, even though the twiddle
angle depends only on `j`, not on which block is being processed -- so for
a stage with many blocks, the same handful of twiddle values were being
recomputed from scratch, repeatedly, via expensive transcendental function
calls. That version benchmarked essentially even with numpy. Replacing it
with a single precomputed twiddle-factor lookup table (built once, shared
across both the 256-point range FFT and the 128-point Doppler FFT, see the
comment block in `c_embedded/src/fft_core.c`) removed that bottleneck and
produced the ~2-3x numbers above, with identical detection output verified
by `c_embedded/tests/test_fft.c` and `test_pipeline.c`.

The remaining gap to numpy is expected and not a bug: numpy's FFT also
uses a mixed-radix algorithm with further-optimized memory access
patterns and SIMD, while this project's FFT is a straightforward
radix-2 implementation chosen for clarity and direct C-portability to a
target without a vendor DSP library, not for maximum desktop throughput.

## Why CA-CFAR shows the largest speedup

CFAR has no numpy-equivalent shortcut in the Python reference: both
`python_model/cfar.py` and `c_embedded/src/cfar_ca.c` use the same
nested-loop, recompute-the-window-sum-per-cell approach (see
`docs/embedded_architecture.md` for why that algorithmic choice was kept
on both sides rather than optimized only in C). With both implementations
using the same O(window_area) per-cell algorithm, the ~6x gap is a fair
read of "interpreted nested Python loops" vs. "compiled C nested loops" on
identical work -- and is the more representative number for what porting
this kind of detection logic from a Python prototype to embedded C
actually buys.
