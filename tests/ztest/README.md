# Ztest suite — sensor manager

Zephyr-native (Ztest) port of the sensor-manager unit tests. Same portable core
as the host `cmake`/`ctest` build and the nRF54L15 firmware app
(`../../src/*.c` + `port/sm_port_zephyr.c`), linked **unmodified**; only the test
harness is Zephyr's. Runs on `native_sim` (host) and, via Twister, on target.

## Layout

```
tests/ztest/
  CMakeLists.txt        links the unmodified core with SM_PORT_ZEPHYR
  prj.conf              CONFIG_ZTEST=y (everything else is native_sim default)
  testcase.yaml         Twister metadata (native_sim + nrf54l15dk)
  src/
    test_ringbuf.c          ZTEST_SUITE(ringbuf)          13 cases
    test_sensor_manager.c   ZTEST_SUITE(sensor_manager)   17 cases
    test_ble_batch.c        ZTEST_SUITE(ble_batch)        10 cases
    test_concurrency.c      ZTEST_SUITE(concurrency)       3 cases
```

Suites map 1:1 to the four modules; assertion intent is identical to the
original `tests/*.c` (only `TEST/CHECK` → `ZTEST/zassert_*`).

## Run

```sh
source ~/zephyrproject/.venv/bin/activate
export ZEPHYR_BASE=~/zephyrproject/zephyr ZEPHYR_TOOLCHAIN_VARIANT=host
west build -b native_sim -t run tests/ztest       # 64-bit host: native_sim/native/64
```

Twister:

```sh
west twister -T tests/ztest -p native_sim
```

> `native_sim`'s POSIX arch is **Linux-only**. On macOS build it in a Linux
> container/VM (host gcc, no Zephyr SDK needed); the on-target `nrf54l15dk`
> qualifier builds natively with the `gnuarmemb` toolchain.

## Concurrency & ThreadSanitizer

The concurrency suite runs the producer/consumer/overflow logic on **Zephyr
threads** (`k_thread`) under the real firmware port (`SM_PORT_ZEPHYR`,
`k_spinlock`). It verifies **accounting integrity** — every submitted sample is
delivered, dropped, or still buffered, exactly once — plus record coherence and
sequence monotonicity, which no sanitizer can check.

**TSan coverage is not preserved on `native_sim`, by design.** ThreadSanitizer
instruments host **pthreads**; Zephyr threads on `native_sim` are scheduled by
the Zephyr kernel inside one host process, so TSan sees no cross-thread races to
analyse. Data-race coverage therefore stays with the original host pthreads
build, kept alongside this suite:

```sh
cmake -S . -B build-san -DSM_SANITIZERS=ON && cmake --build build-san
ctest --test-dir build-san -R test_concurrency_tsan --output-on-failure
```

So the two concerns are split cleanly: **Ztest/native_sim** owns functional and
accounting correctness on the real port; **host+TSan** owns race detection.
