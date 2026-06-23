# Embedded Architecture Notes

The `c_embedded/` code is written as if it had to run on an actual radar
sensor's MCU/DSP core, not just "C that happens to implement the algorithm
on a desktop." This document explains the embedded-specific decisions and
walks through a real bug that came out of taking that constraint
seriously.

## No dynamic memory, anywhere

Every buffer in the pipeline is a fixed-size, compile-time-sized static or
stack array (`radar_types.h` defines `chirp_matrix_t`, `rd_map_t`, and
`detection_list_t` as plain arrays/structs, no pointers to heap memory).
Most automotive radar MCU/DSP targets either don't have a heap at all in
the signal-processing task, or treat malloc/free as something to avoid for
determinism and fragmentation reasons -- a missed detection because of a
late-arriving allocation failure is not an acceptable failure mode in a
safety-relevant sensor.

## One antenna resident in RAM at a time

A real sensor's antenna count and frame size mean holding every receive
antenna's full chirp matrix in RAM simultaneously is wasteful, and on a
memory-constrained target it may not fit at all. `doppler_fft_accumulate_power()`
processes one antenna's range-FFT output at a time, accumulating power
into a single `(num_chirps, num_samples)` buffer across antennas
(`reset=1` on the first antenna, `reset=0` after), so peak RAM usage is one
antenna's chirp matrix plus one shared power accumulator -- not
`num_rx` chirp matrices at once. `radar_pipeline.c` and
`benchmark.c` both follow this same one-antenna-at-a-time loading
pattern.

## Case study: a real CFAR buffer-overflow bug

`detection_list_t` bounds CFAR's raw output to `RADAR_MAX_DETECTIONS`
entries -- necessary on a fixed-memory target, since "however many
detections happen to occur" is not a valid array size. The first version
of this buffer was sized to a round number (32) that looked generous for a
two-target test scene.

It wasn't. CA-CFAR doesn't produce one hit per target; it produces a hit
for every cell whose power clears the local threshold, which on a real
target's mainlobe-plus-sidelobe footprint is often several dozen cells
spread across both targets combined. With the buffer capped at 32, the
detector silently stopped recording new hits once full -- and the cell it
dropped, on one run, happened to be the true peak of one target's
footprint. The clustering step then picked the strongest *remaining*
cell as that target's representative detection: a real hit, just not the
true peak, which showed up as a small but real disagreement with the
Python reference model's velocity and magnitude for that target.

Two fixes, not one:

1. **Size the buffer to a measured number with margin**, not a guess.
   Printing the actual raw CFAR hit count on the project's test scenes
   showed roughly 40-45 hits before clustering; `RADAR_MAX_DETECTIONS` is
   now 128, comfortably above that with room for noisier scenes.
2. **Make overflow observable instead of silent.** `cfar_ca_process()` now
   sets `out_list->overflowed = 1` if the raw hit count would have
   exceeded the buffer, and `radar_pipeline.c` checks and prints a warning
   on that flag rather than continuing as if nothing happened. A fixed-size
   buffer on an embedded target is a legitimate design choice; a fixed-size
   buffer that fails *silently* is not.

`c_embedded/tests/test_cfar.c::test_overflow_flag_is_set` is a regression
test for exactly this scenario, and
`c_embedded/tests/test_pipeline.c::test_full_pipeline_matches_known_targets`
is the integration-level check that would have caught the original
symptom (a wrong velocity on one target) even without knowing the root
cause yet.

## Why the algorithmic structure mirrors Python so closely

`cfar_ca.c` deliberately recomputes each cell's training-window sum from
scratch (an O(window_area) operation per cell) rather than using a
sliding-sum or summed-area-table optimization, even though that
optimization is well known and would make the C version faster still. The
goal of this particular comparison is to isolate the speedup that comes
from compiled C vs. interpreted Python on *identical* algorithmic
complexity (see `docs/performance_benchmark.md`); a smarter algorithm on
only one side would conflate "C is faster" with "this version uses a
better algorithm," which is a different (also real, but separate) point.
A production version of this detector would use the sliding-sum
optimization on the embedded target; it's noted here as a known follow-up
rather than implemented, to keep the benchmark honest.
