# nRF54L15 firmware simulation

This target compiles the same `sensor_manager.c`, `sm_ringbuf.c` and
`ble_batch.c` used by the host tests, then runs them on LabWired's nRF54L15
model. Target-specific code provides startup, a PRIMASK critical section,
GRTC time, UART output and sensor drivers.

## Build and verify

Install an `arm-none-eabi-gcc` toolchain, Python 3 and the LabWired CLI. A
`labwired-core` checkout supplies the smart-ring system manifest and models.
From this directory:

```sh
make
make test LABWIRED=/path/to/labwired-core
```

`make run` is an alias for the same verification. Select a particular CLI
binary or cross compiler when required:

```sh
make test LABWIRED=/path/to/labwired-core \
  LABWIRED_CLI=/path/to/labwired \
  CROSS=/path/to/arm-none-eabi-
```

The check runs the real ELF through `labwired run --system`, writes
`build/uart.log`, and requires all sensor initialization markers, successful
ring-buffer recovery, the BLE-load overflow verdict and the final completion
marker. Missing or inconclusive evidence exits nonzero. The 600-million-step
limit covers all application phases. The former 120-million-step limit could
stop during the initial recovery experiment while returning success.

LabWired v0.25.0 limits its `test` subcommand to 50 million steps for this chip.
The bounded `run` command plus `tests/check_uart.py` supports the complete
experiment with that released CLI.

## Hardware modeled

| Function | Device or peripheral | Evidence |
|---|---|---|
| Motion | BMI270 at I²C address `0x68` | `CHIP_ID=0x24`, configuration upload and initialization status |
| PPG | MAX30102 at `0x57` | `PART_ID=0x15`, 1600 Hz sample source and FIFO overflow counter |
| Temperature | TMP117 at `0x48` | `DEVICE_ID=0x0117`, external temperature register reads |
| Haptic output | DRV2605 at `0x5A` | Driver initialization and GO command |
| Touch input | GPIO P1.13 | GPIO input read |
| Time | GRTC and SysTick | Acquisition deadlines and periodic wake interrupts |

The source uses TWIM EasyDMA transactions for the I²C devices. It issues a
repeated START for register reads. UART carries the measurements and verdicts.

## Application phases

1. **Ring-buffer overflow and recovery.** The IMU producer runs at 50 Hz. The
   consumer drains for 0.3 s, pauses for 2.4 s, then resumes for 0.4 s. A
   100-sample buffer must retain the newest samples and count every overwritten
   sample. The first resumed sequence number must account for the same loss.
2. **PPG acquisition with the CPU available.** The driver drains the MAX30102
   FIFO for 0.6 s. Its overflow counter must remain zero.
3. **PPG acquisition under simulated BLE load.** SysTick records a 25 ms drain
   pause every 30 ms. The same acquisition loop runs for another 0.6 s. The
   32-sample FIFO must report overflow under this load.

The firmware uses WFI between service ticks so the simulator can advance idle
periods. The BLE load represents CPU unavailability and drain jitter. BLE
packets, RF current, analog supply noise, battery discharge and physical sensor
accuracy are outside this experiment. BLE batch encoding still uses the
portable implementation exercised by the host tests.

## Reproduced result

On 2026-11-10, ARM GCC 15.2.1 built the current firmware with all warnings
enabled as errors: 6,392 bytes of text and 4,032 bytes of BSS. LabWired CLI
v0.25.0 with core manifest revision `9f9a5d89` completed all phases:

| Measurement | Observed result |
|---|---:|
| IMU samples pushed during recovery experiment | 154 |
| Samples delivered | 133 |
| Samples overwritten | 21 |
| Peak retained buffer | 100 / 100 |
| Sequence gap reported on resume | 21 |
| PPG samples drained with BLE load off | 952 |
| PPG overflow count with BLE load off | 0 |
| Simulated BLE events with load on | 20 |
| PPG overflow count with load on | 31, counter saturated |

The recovery accounting closes: `133 + 21 = 154`, and the sequence gap equals
the overwrite count. The PPG check requires `off.overflow == 0` and
`on.overflow > 0`; it does not depend on an exact drain count at a tick boundary.
All seven required UART evidence markers passed the checker.

These results establish behavior in the modeled system. Host concurrency and
buffer tests cover different properties and remain separate:

```sh
cmake -S ../.. -B ../../build -DCMAKE_BUILD_TYPE=Debug
cmake --build ../../build --parallel 2
ctest --test-dir ../../build --output-on-failure
```

## Files

| File | Purpose |
|---|---|
| `Makefile` | Cross build and checked simulation |
| `nrf54l15.ld` | RRAM and SRAM layout |
| `src/startup.c` | Vector table, data initialization and SysTick handler |
| `src/libc_shim.c`, `shim/string.h` | Freestanding memory routines |
| `src/nrf54l15.h` | Register subset and timing constants |
| `src/board.c` | UART, TWIM, GRTC and sensor drivers |
| `src/sm_port_nrf54l15.c` | PRIMASK critical sections and GRTC clock |
| `src/main.c` | Acquisition, recovery and BLE-load experiments |
| `tests/check_uart.py` | Completion and verdict checks |

The port restores the prior PRIMASK state when a critical section ends. The
build links `libgcc` for 64-bit division but uses no libc startup or dynamic
allocation. The simulator chip profile and sensor models live in
[LabWired Core](https://github.com/w1ne/labwired-core).
