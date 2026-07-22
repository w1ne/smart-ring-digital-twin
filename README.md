# Smart Ring Platform, homework
An open-source smart-ring platform on the Nordic **nRF54L15**:

system architecture, a portable C11 sensor manager, and a debugging methodology.

See

**Live write-up (interactive 3D + diagrams): https://w1ne.github.io/smart-ring-digital-twin/**

for demos

## Deliverables

| Part | Where |
|---|---|
| 1. System Architecture | [`docs/part1-architecture.md`](docs/part1-architecture.md) |
| 2. Firmware (sensor manager) | [`src/`](src/), [`tests/`](tests/), [`app/`](app/) |
| 2. On simulated silicon | [`firmware/nrf54l15/`](firmware/nrf54l15/) |
| 3. Debugging & Investigation | [`docs/part3-debugging.md`](docs/part3-debugging.md) |
| Assumptions, risks, unknowns | [Part 1 §7](docs/part1-architecture.md#7-assumptions-risks-and-unknowns) |

## Build & run

CMake ≥ 3.16 and any C11 compiler. No third-party dependencies.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j
ctest --test-dir build --output-on-failure
```

Demo (simulated sensors + BLE link):

```sh
./build/sensor_demo --seconds 10           # normal operation
./build/sensor_demo --seconds 9 --stall 3  # phone drops out, buffer overflows, gap reported
```

Optional build with memory-bug and race detectors turned on: ASan/UBSan for the single-threaded tests, ThreadSanitizer for the multi-threaded stress test.

```sh
cmake -S . -B build-san -DCMAKE_BUILD_TYPE=Debug -DSM_SANITIZERS=ON
cmake --build build-san -j
ctest --test-dir build-san --output-on-failure
```

Under Zephyr (Ztest), the same 43 cases run on `native_sim`
and on-target via Twister; the host+TSan build above is kept for race detection:

```sh
west build -b native_sim tests/ztest -t run
```

## Layout

```
src/                sensor manager, ring buffers, rate control, retention, stats,
                    BLE batch encoder; port layer (posix / single-thread / zephyr sketch)
app/                demo: acquisition thread + BLE consumer thread
tests/              43 cases, 4 suites (host ctest + TSan); tests/ztest/ = Zephyr Ztest
firmware/nrf54l15/  the same src/ core cross-compiled onto a simulated nRF54L15
docs/               Part 1 (architecture) and Part 3 (debugging)
```

## Requirements coverage

| Requirement | Implementation | Test |
|---|---|---|
| IMU 50 Hz / temp 1 Hz | drift-free `sm_rate_t` | `rate_is_drift_free_over_ten_seconds` |
| Thread-safe ring buffers | `sm_ringbuf` + port lock | `test_concurrency` (TSan) |
| Retain latest 100 IMU / 20 temp | `SM_IMU_CAPACITY` / `SM_TEMP_CAPACITY` | `imu_retains_latest_100` |
| BLE retrieves buffered data | `sm_read`/`sm_peek` + `ble_batch` | `test_ble_batch` |
| Graceful overflow | overwrite-oldest + drop counter + sequence gap | `overflow_leaves_a_detectable_sequence_gap` |
| Unit tests | `tests/` | 4 suites, all green |

## Scope

Runs on host and on a simulated nRF54L15.
