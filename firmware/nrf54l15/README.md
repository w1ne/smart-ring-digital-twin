# Part 2, on silicon: the sensor manager running on an nRF54L15

The host build proves the sensor manager is *correct*. This proves it is
*portable* — and it is the difference between claiming a clean port layer and
demonstrating one.

`../../src/sensor_manager.c`, `../../src/sm_ringbuf.c` and
`../../src/ble_batch.c` are compiled here **unmodified** — byte-for-byte the
sources the host unit tests build. There is no vendored copy and no `#ifdef` in
the core. Only two files are new: the port (`src/sm_port_nrf54l15.c`, ~60 lines)
and the drivers (`src/board.c`). If the core had a hidden host dependency, this
build would not link.

It runs on a simulated nRF54L15 in [LabWired](https://labwired.com), against
behavioural models of the actual ring sensors: an MPU-6050 IMU, a MAX30102 PPG
front end, a CAP1188 touch controller and a DRV2605 haptic driver, all on one
TWIM (I²C) bus, plus the SoC's on-die temperature sensor and the GRTC.

> The nRF54L15 was not supported by LabWired before this assessment. The chip
> profile, the GRTC model and the three sensor models were written as part of
> this work; see "Provenance" below.

## Build and run

```sh
brew install arm-none-eabi-gcc          # toolchain (no Zephyr, no nRF SDK, no CMSIS)
make                                    # build the ELF
make run                                # run it in the simulator
```

`make run` expects a LabWired checkout at `$LABWIRED`
(default `/Users/andrii/Projects/lw-nrf54l15`) with `cargo build -p labwired-cli
--release` already done.

## Result

```
=== smart ring / nRF54L15 ===
sensor manager: imu 50Hz keep 100, temp 1Hz keep 20
[init] imu    MPU6050  ok (WHO_AM_I=0x68)
[init] ppg    MAX30102 ok (PART_ID=0x15)
[init] haptic DRV2605  ok
grtc t0 us    : 205
grtc dt us    : 1096

--- after 1.2 s ---
imu  pushed   : 60        <- 50 Hz x 1.2 s, exactly
imu  dropped  : 0
imu  buffered : 3
temp pushed   : 1         <-  1 Hz x 1.2 s, exactly
temp buffered : 1
ppg  reads    : 60
ble  frames   : 23
ble  samples  : 57        <- 57 delivered + 3 buffered = 60 pushed
touch status  : 0
haptic        : GO asserted
=== done ===
```

Every `[init]` line is a device **ID register read back over I²C**, not an
assumption: `WHO_AM_I = 0x68`, `PART_ID = 0x15`. A silent bus failure cannot be
mistaken for a working sensor.

Three numbers are worth pausing on:

- **60 IMU samples in 1.2 s and 1 temperature sample** — both rates exact, zero
  drift. The same property `rate_is_drift_free_over_ten_seconds` asserts on the
  host, now observed against a real 1 MHz hardware counter instead of injected
  virtual time.
- **57 delivered + 3 buffered = 60 pushed, 0 dropped.** The accounting identity
  the concurrency test checks on the host holds on the target too.
- **23 notifications for 57 samples** ≈ 2.5 samples per radio event, because the
  loop drains every simulated 50 ms connection interval. That is the expected
  consequence of a short connection interval, and it is the argument for the
  400–1000 ms intervals the power budget assumes — at which the 16-samples-per
  frame packing actually fills.

The run costs ~330 M simulated cycles and takes a couple of minutes of wall
clock; `RUN_DURATION_US` in `src/main.c` trades run length against patience.

## What is real and what is not

| Real | Not real |
|---|---|
| Cortex-M33 instruction execution | The BLE radio — LabWired does not model the nRF54L radio |
| RRAM/SRAM map, reset SP `0x2004_0000` | The battery and charger |
| TWIM EasyDMA I²C transactions, register by register | Skin contact — the PPG waveform is synthetic |
| GRTC 1 MHz SYSCOUNTER | |
| Sensor register maps and FIFOs | |
| On-die TEMP peripheral | |

The BLE "notification" is encoded into a real wire frame by the same
`ble_batch_encode_imu()` the host tests exercise, and then printed rather than
transmitted. Everything up to the radio is genuine.

**The on-die TEMP sensor measures the die, not skin.** It is used here because
it needs no extra model. On a real ring it is a poor proxy for body
temperature — the die is heated by the MCU, the PPG LEDs and the motor — which
is exactly the thermal-coupling problem raised in the architecture document. A
shipping design puts a dedicated sensor on the skin-contact surface.

## Three things this caught that the host build could not

**1. `-nostdlib` means no `memcpy`.** The Homebrew ARM toolchain ships without
newlib, and this firmware links `-nostdlib` by choice — a wearable does not want
newlib's malloc and locale tables pulled in because one file included
`<string.h>`. `src/libc_shim.c` provides `memcpy`/`memmove`/`memset`, which the
compiler also emits implicit calls to when copying structs.

**2. 64-bit timestamps cost a libgcc call.** `sm_rate_poll` divides 64-bit
microsecond values, and a 32-bit core has no 64-bit divide, so GCC emits
`__aeabi_uldivmod` — hence `-lgcc` even with `-nostdlib`. Each call is on the
order of a hundred cycles and runs on every fire. Irrelevant at 50 Hz; the first
thing to replace with repeated subtraction if acquisition ever reaches kHz.

**3. An I²C register read needs a repeated START, not a STOP.** The first
version of `board_i2c_read_reg()` did the obvious thing — write the register
pointer, STOP, then STARTRX — and the IMU returned garbage while the simpler
devices happened to work. A STOP ends the transaction, and a slave is entitled
to treat that as the end of the addressing phase. The nRF fix is the
`LASTTX_STARTRX` short, which chains TX→RX in hardware so the repeated start
has correct timing regardless of CPU latency. This is precisely the class of
bug a host test can never find.

## Files

```
Makefile                  cross build; compiles ../../src unmodified
nrf54l15.ld               RRAM 1524K @ 0x0, RAM 256K @ 0x20000000
shim/string.h             freestanding <string.h> (toolchain has no newlib)
src/startup.c             ARMv8-M vector table, .data/.bss init
src/libc_shim.c           memcpy / memmove / memset
src/nrf54l15.h            audited register subset (not generated CMSIS)
src/board.c               UARTE console, TWIM I2C, GRTC time, sensor drivers
src/sm_port_nrf54l15.c    THE PORT — PRIMASK critical sections + GRTC time
src/main.c                acquisition loop + BLE-batch consumer
```

`sm_port_nrf54l15.c` is worth reading first: it is the entire platform
dependency of the sensor manager, and it uses **interrupt masking rather than a
mutex**, because on the target the producer is an ISR and a mutex taken in ISR
context deadlocks. It saves and restores PRIMASK rather than blindly enabling
interrupts on release, so the lock composes inside an already-masked region.

## Provenance

The nRF54L15 support this depends on was written for this assessment and is a
separate contribution to LabWired (branch `feat/nrf54l15-onboarding`):

- chip profile + nRF54L15-DK system manifest, sourced from the Zephyr devicetree
- a GRTC behavioural model (52-bit SYSCOUNTER, 12 CC channels, anti-tearing
  L/H read pair)
- MAX30102, CAP1188 and DRV2605 component models (51 unit tests)
- 7 bus-level conformance tests + 2 ELF boot tests

It also surfaced two bugs in existing code: GPIO peripheral bases were mapped
`0x500` too high on **both** the new nRF54L15 profile and the pre-existing
nRF5340 one (a Nordic GPIO devicetree node points at the OUT register, not the
peripheral base), and a TWIM bookkeeping gap that attached kit-registry devices
twice. Full suite after the changes: 2107 passed, 0 failed.
