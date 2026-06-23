cmake_minimum_required(VERSION 3.15)
project(automotive_radar_dsp_embedded_c C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_C_FLAGS_RELEASE "-O2")

include_directories(c_embedded/include)

# ---------------------------------------------------------------------------
# Shared DSP core (FFT, range FFT, Doppler FFT, CA-CFAR) -- linked into the
# pipeline demo, the benchmark binary, and the test binaries so there is a
# single compiled copy of the algorithm under test everywhere.
# ---------------------------------------------------------------------------
add_library(radar_dsp STATIC
  c_embedded/src/fft_core.c
  c_embedded/src/range_fft.c
  c_embedded/src/doppler_fft.c
  c_embedded/src/cfar_ca.c
)
target_link_libraries(radar_dsp m)

# ---------------------------------------------------------------------------
# End-to-end demo binary: runs the pipeline on the Python-exported test
# vectors and prints the detected object list.
# ---------------------------------------------------------------------------
add_executable(radar_pipeline c_embedded/src/radar_pipeline.c)
target_link_libraries(radar_pipeline radar_dsp m)

# ---------------------------------------------------------------------------
# Benchmark binary: per-stage timing, results appended to
# results/benchmark_table.csv alongside the Python timings.
# ---------------------------------------------------------------------------
add_executable(benchmark c_embedded/src/benchmark.c)
target_link_libraries(benchmark radar_dsp m)

# ---------------------------------------------------------------------------
# Unit / integration tests -- each test_*.c is a standalone binary with its
# own main() and a non-zero exit code on failure, registered with ctest.
# ---------------------------------------------------------------------------
enable_testing()
include_directories(c_embedded/tests)

add_executable(test_fft c_embedded/tests/test_fft.c c_embedded/src/fft_core.c)
target_link_libraries(test_fft m)
add_test(NAME test_fft COMMAND test_fft)

add_executable(test_cfar c_embedded/tests/test_cfar.c c_embedded/src/cfar_ca.c)
target_link_libraries(test_cfar m)
add_test(NAME test_cfar COMMAND test_cfar)

add_executable(test_pipeline c_embedded/tests/test_pipeline.c)
target_link_libraries(test_pipeline radar_dsp m)
add_test(NAME test_pipeline COMMAND test_pipeline)
