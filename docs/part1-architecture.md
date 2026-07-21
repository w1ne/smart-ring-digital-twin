# Part 1 — System Architecture

Scope: next-revision open-source smart ring — nRF54L15, IMU, PPG, temperature, cap-touch, LRA/ERM motor, BLE 5.x, ~20 mAh Li-Po. A separate team builds the app in parallel, so the BLE contract is a hard, versioned, discoverable interface.

All figures below are **order-of-magnitude estimates, not datasheet values** — each needs bench validation on DVT (§7 names the measurement for each open number).

---

## 1. Firmware Architecture

### 1.1 RTOS choice: Zephyr via nRF Connect SDK

**Decision: Zephyr + nRF Connect SDK (NCS).** In order of weight:

- First-class NCS target: maintained BLE controller, PPR/FLPR support, nrfx drivers. Bare-metal means owning radio timing.
- Devicetree + Kconfig give board variants for free — critical for §5, where a new sensor is an overlay, not a fork.
- `native_sim` runs the app layer on host; with the portable core (§1.2) that's hardware-free CI unit testing.
- MCUboot + SMP/mcumgr gives signed DFU out of the box; "roll your own bootloader" is indefensible for a third-party-developer platform. (Zephyr's `pm_device`/soft-off machinery also serves §4 directly.)

**Alternative: bare-metal `nrfx` superloop** wins on RAM and idle determinism but loses at this feature count (six mixed-rate streams, timing-sensitive BLE, offline logging with wear management, DFU) — an RTOS-shaped problem. Tickless idle means the RTOS isn't the power problem; the radio and PPG LEDs are.

Cost: NCS version churn and a contributor-hostile build system — mitigated by a pinned NCS SHA in a west manifest plus a container build.

### 1.2 Layering

```
┌──────────────────────────────────────────────┐
│  BLE service layer  (GATT, batching, backfill)│   ← DRAINS buffers
├──────────────────────────────────────────────┤
│  Sensor manager core   (portable C11)         │   ← no RTOS types
│  ring buffers · seq numbers · drop counters   │
├──────────────────────────────────────────────┤
│  Port layer   sm_lock_t · sm_time_now()       │   ← ~40 lines per platform
├──────────────────────────────────────────────┤
│  HAL / drivers   Zephyr sensor.h · raw nrfx   │
└──────────────────────────────────────────────┘
```

The **sensor manager core** is pure C11, no Zephyr headers: one static ring buffer per channel, **overwrite-oldest** on overflow, monotonic 32-bit sequence number, per-channel drop counter. Capacities are arbitrary (products: 100 and 20), so wrapping is modular not mask-based — a test pins that. No `malloc`, no unbounded queues: on a 256 KB device with a real-time radio, dynamic allocation is a latent failure, and blocking a producer to avoid a drop is worse — it stalls sampling and corrupts the timebase for *every* channel. Overwrite-oldest suits a wearable: after a six-hour gap the newest data beats the oldest, and the drop counter lets the app quantify loss instead of silently interpolating.

The **port layer** is the entire RTOS dependency: opaque `sm_lock_t` (lock/unlock) and `sm_time_now()` (monotonic µs). Zephyr → `k_spinlock` / `k_uptime_ticks()`; host → pthread mutex / `CLOCK_MONOTONIC`. That makes the core host-testable: wraparound, overflow drain, and sequence continuity run in CI under sanitizers, not on flashed hardware.

Above the core, the **BLE layer is a consumer, never a callee**: drivers never call BLE; the BLE thread bulk-copies N records when a connection event needs filling — killing priority-inversion/reentrancy bugs and letting the radio, not the sensor, decide when it runs.

### 1.3 The PPR split — headline decision

nRF54L15 carries a Cortex-M33 @128 MHz plus a **PPR (Peripheral Processor, a RISC-V "VPR" core)** and an **FLPR (fast lightweight processor)**. Architect around this:

- **PPR** runs the always-on acquisition loop — service INT lines, drain FIFOs over TWIM/SPIM, timestamp, push to shared ring buffers — from a low-power clock domain, M33 asleep.
- **M33** stays in System OFF / idle with RAM retention, waking only for BLE connection events, batch/signal processing, and user interaction.
- **FLPR** reserved for tightly-timed peripheral work (future nonstandard-bus sensor, motor waveform generation).

Win: the always-on 25–50 Hz sampling never wakes the 128 MHz M33, whose wake pattern collapses to roughly the BLE connection interval.

**Costs:** an IPC/shared-memory protocol — coherency on the shared SRAM and a cross-core lock (`k_spinlock` is thread-only; need an inter-core mutex or SPSC lock-free indices). Dual-image DFU: PPR and M33 must version together or the shared layout silently corrupts — embed a layout hash in both, refuse to boot on mismatch.

**Unknown:** unbenchmarked — PPR wake latency and per-transaction energy vs DPPI/PPI-driven EasyDMA that never wakes the M33. PPI + autonomous TWIM/SPIM may get 80% of the benefit at 20% of the complexity. **First EVK measurement**, before committing to the split.

### 1.4 Concurrency model

| Context | Priority | Job |
|---|---|---|
| Sensor INT / timer | IRQ or PPR | Timestamp, kick DMA |
| Acquisition thread | High (cooperative) | Drain FIFO → ring buffer |
| BLE stack | Zephyr-managed, highest effective | Radio timing — never blocked by us |
| Consumer thread | Low | Bulk drain → GATT notify / flash log |
| Idle | — | Tickless, PM enters System ON-IDLE / OFF |

Ring buffers are the *only* shared mutable state. Critical sections are short and bounded — index arithmetic plus a `memcpy` of a contiguous span, never per-record calls under lock. Worst-case hold (N × record_size) must sit well under the BLE guard time — target < 20 µs, asserted by GPIO toggle on a logic analyzer.

---

## 2. Hardware Architecture

**PCB.** Rigid-flex: a flex spine with two or three rigid islands (MCU+radio, PPG+optics, battery/charge) wrapped into the band. Area binds everything in §5, and component *height* matters as much as footprint — inner-diameter clearance is unforgiving.

**Antenna.** Dominant RF problem: a lossy, high-permittivity finger millimetres away detunes it — resonance drops, several dB lost.
- Hard keep-out; no battery, copper pour, or motor under it.
- Tune with a **finger phantom / loaded fixture**, never free space — free-space-tuned wearable antennas are a classic field failure.
- Lean **ceramic chip antenna** over a printed trace: smaller, easier to match on curved rigid-flex where trace geometry varies across the bend, at BOM and Q cost. Decide from a measured DVT comparison.

**Power tree.** Li-Po (3.0–4.2 V) → nRF54L15 internal DC/DC (VDD direct), plus three load-switched rails: `VDD_SENSOR_DIG` (IMU, temp, touch), `VDD_ANALOG` (PPG AFE, separately filtered), `VDD_MOTOR` (driver, local bulk cap). Rail gating isn't optional: a "standby" sensor still draws µA and leaks — three at 3 µA is 7.5% of the week budget for nothing. Tradeoff: re-init/re-calibration on power-up; for some parts a cold start costs more than standby, a per-part crossover.

**PPG gets its own filtered analog rail.** BLE PA transients (tens of mA at connection cadence) coupled into a microvolt-scale photodiode front end are a synchronous artifact indistinguishable from a real physiological signal. Separate rail, local bulk, star ground, PPG sampling scheduled around radio events.

**Bus resilience.** All four sensors on one shared I²C bus (TWIM21) is the fragile point: one wedged device — brownout mid-transfer, ESD, a clock-stretch hang — takes down *all* sensing, and it's the same root as Issue A. Rev 2 splits the **PPG onto its own SPI bus** (isolation + the throughput it needs); firmware carries an I²C recovery routine (nine clock pulses to unstick a slave holding SDA → TWIM reset → per-transaction timeout → re-init) plus **per-sensor power gating**, so a hung device self-heals and is flagged instead of bricking the ring.

**Charging.** Contact pads, not Qi (a Qi secondary costs volume we lack and couples poorly at this size). Contact costs: sweat corrosion/electrolysis (gold-over-nickel, pads equipotential when idle), ESD (TVS on both pads, non-negotiable), cradle alignment. Add a charge-inhibit path so a shorted/wet cradle can't dump into the cell.

**Fuel gauging.** ADC voltage is free but useless in Li-Po's flat 30–80% band; firmware coulomb counting needs accurate µA current sense (hard); a gauge IC with a model costs a part and ~1–3 µA. Recommendation: **gauge IC** — a wearable that lies about battery is a support burden, and 2 µA of 119 µA is affordable. Area-constrained fallback: voltage + a learned discharge model, flagged low-confidence.

**Interrupts.** Every sensor INT routes to a GPIO that wakes from System OFF; polling is the enemy — a 1 Hz poll waking the M33 costs more than the sensor it reads. IMU wake-on-motion is the master gate for §4's adaptive sampling.

**Thermal.** The temperature sensor should read *skin* but reads a mix of skin, MCU self-heating, PPG LED dissipation, and motor — the most under-appreciated BOM failure mode. Mitigations: thermally isolated island (flex neck, slotted copper, minimal pour) far from PPG/motor, coupled to the skin-side wall; a firmware blackout window around PPG bursts and haptics. Accuracy stays poor — document for *trends and deltas only*, not core temperature.

**Motor.** A mechanical and electrical aggressor: back-EMF needs a flyback diode and a real driver; inrush sags the rail (browns out PPG, disturbs coulomb counting); mechanically it saturates the IMU and destroys PPG while running. Firmware flags IMU/PPG samples spanning a haptic event invalid via a record flag bit (§3).

**Test/debug.** SWD pads on the panel (ideally survivable on the assembled unit — open platform), RTT logging (no UART pins burned), a pogo-pin production fixture on SWD + charge/test pads, and a self-test firmware mode that exercises every sensor over BLE.

---

## 3. BLE Interface Design

### 3.1 Service map

| Service | Use |
|---|---|
| Device Information (SIG) | Model, HW rev, FW rev, serial |
| Battery Service (SIG) | Battery level — free integration with every OS |
| Current Time (SIG) | Coarse time sync |
| **Ring Data Service (custom, 128-bit UUID)** | Everything else |

Custom service characteristics:

| Char | Props | Purpose |
|---|---|---|
| Capability descriptor | Read | Generated from the sensor table (§5): sensor IDs, rates, record sizes, protocol version |
| Stream control | Write | Start/stop per channel, rate, mode (week / research) |
| **Sensor data** | Notify | Batched packed binary records |
| Configuration | Read/Write | Thresholds, calibration constants, log policy |
| Control point | Write + Indicate | Haptic buzz, calibrate, time sync, backfill request, factory reset — indicate carries the result code |
| Log / backfill | Notify | Historical records drained from flash |

### 3.2 Batching is the whole design

50 Hz IMU × 12 B ≈ **600 B/s** — trivial for 2M PHY. The problem is **wakeups**: one notification per sample is 50 radio wakes/s, each ramping up, and ramp-up energy dwarfs payload. Batching cuts wakes proportionally for identical data.

So fill the negotiated ATT MTU. With **DLE and a 247-byte ATT MTU** (244 B payload after the header), the format packs a 16-byte batch header plus 14-byte IMU records: **16 records per notification** — one every 320 ms at 50 Hz, not fifty per second. A longer connection interval collapses radio duty further.

Implemented in `src/ble_batch.c`, asserted by `payload_budget_yields_sixteen_imu_samples` in `tests/test_ble_batch.c`.

Rules: **2M PHY** (halves airtime/energy), **DLE**, negotiate MTU at connect, *derive* batch size from it — Android and iOS grant different values, and a hardcoded 244 silently fragments.

### 3.3 Record framing

What `src/ble_batch.c` emits: little-endian, field-by-field — never a struct `memcpy`, since compiler padding and host endianness aren't part of a cross-team contract.

```
off  size  field
0    1     type       0x01 IMU batch, 0x02 temperature batch
1    1     count      records in this frame
2    4     first_seq  monotonic sequence number of record 0
6    8     base_t_us  absolute device time of record 0
14   2     dropped    lifetime drop count, saturating at 0xFFFF
16   ...   records

IMU record  (14 B): dt_us:u16, accel[3]:i16, gyro[3]:i16
Temp record ( 6 B): dt_ms:u16, milli_celsius:i32
```

`first_seq` + `count` let the phone reconstruct ordering and detect gaps; `dropped` comes straight from the ring buffer's drop counter, so the app *quantifies* loss. It saturates, not wraps — "at least 65535 lost" is true; a wrapped small number lies.

Two encoder subtleties (both caught by the payload-budget test):

- **`dt` is the inter-record gap, not a `base_t_us` offset.** A cumulative u16 of µs overflows at 65.5 ms — three samples at 50 Hz — closing batches before the payload fills; the gap need only span one sample period.
- **Unit is per-stream** — µs (50 Hz IMU), ms (1 Hz temp); a µs field can't hold one 1 Hz period. Hence typed records, not generic.

An over-large gap closes the batch early rather than emit a wrong delta; the rest buffer for the next frame.

**Next:** a `flags` byte (backfill / haptic-corrupt / overflow) and a `sensor_id` into the §5 capability table, so a new sensor gets a new type tag and old apps skip it rather than misparse.

### 3.4 Connection parameters

The **peripheral must request** its preferred parameters (`bt_conn_le_param_update`); central defaults are fine for a phone and ruinous for a 20 mAh cell.

| Mode | Conn interval | Slave latency | Effect |
|---|---|---|---|
| Week | 400–1000 ms | 4–10 | Radio nearly idle; latency in seconds, fine for background logging |
| Research | 15–50 ms | 0 | Low latency, high throughput, ~7× the radio energy |

iOS rejects parameters outside Apple's accessory guidelines outright. Test both platforms early, not at integration.

### 3.5 Security and privacy

LESC pairing with bonding; reject legacy pairing. **Resolvable private addresses with rotation** — a static MAC is a passive tracking beacon following the wearer through every BLE scanner in a city; not optional. Accept-list filter once bonded; encrypt-and-authenticate the data and control-point characteristics.

### 3.6 Offline logging and backfill

Non-negotiable for a ring (showers, sleep, phone left behind): with no connection the consumer thread drains ring buffers into a flash log (Zephyr NVS or a circular RRAM/external-flash log). On reconnect the app sends a backfill request with its last sequence number; the device streams the gap over the log characteristic with `flags.backfill` set, rate-limited so it doesn't starve live data. The log is itself overwrite-oldest with drop accounting.

### 3.7 DFU

MCUboot + SMP/mcumgr over BLE, dual-slot with revert-on-failure — a bricked ring needs disassembly to recover. Open platform: **unlocked but signed**, with a documented, replaceable key so a researcher can build and sign their own image while stock devices refuse unsigned firmware.

---

## 4. Power Management Strategy

### 4.1 The two budgets

Assume a **20 mAh usable** Li-Po (below nameplate after cutoff voltage and aging margin). Every decision below is measured against these:

| Target | Average current budget |
|---|---|
| **1 week** (168 h) | 20 mAh / 168 h = **~119 µA** |
| **24 hours** | 20 mAh / 24 h = **~833 µA** |

> **Probably optimistic — the first thing I'd nail down.** A public Oura Ring 5
> teardown (a shipping ring in this class) puts its cell at **10.2 mAh**, half my
> assumption. That makes the budgets **~61 µA** and **~425 µA**, and the §4.2 week
> design (~108 µA) delivers **under 4 days**, not 7.7. I keep the 20 mAh arithmetic
> because the *method* rescales linearly — but a 7-day ring at this feature set is
> harder than the numbers suggest, and cell capacity is the first measurement, not
> an assumption. See A1.

### 4.2 Estimated current budget

Order-of-magnitude, bench-validate. Per-subsystem average current, µA. **Week mode**: IMU WoM+12.5 Hz accel, PPG 15 s/15 min, temp 1/min, 1000 ms conn, batched. **Research mode**: IMU 50 Hz accel+gyro, PPG continuous 25 Hz, temp 1 Hz, 30 ms conn.

| Subsystem | Week | Research |
|---|---|---|
| M33 (OFF+retention wakes → continuous) | 12 | 120 |
| PPR acquisition | 15 | 60 |
| IMU (12.5 Hz accel → 50 Hz accel+gyro) | 15 | 150 |
| **PPG** (1.8 mA LED; 1.67% duty → continuous) | **30** | **400** |
| Temperature | 1 | 3 |
| Cap touch | 5 | 10 |
| BLE (1 s batched → 30 ms) | 20 | 120 |
| Quiescent + gauge + leakage | 10 | 15 |
| **Total** | **~108 µA → ~7.7 days** | **~878 µA → 22.8 h** |

Budget was 119 µA (week) / 833 µA (24 h). **Research mode misses 24 h by ~5%** — a real finding, not rounding. Fix by trimming PPG duty (25→20 Hz or shorter LED pulse), gating gyro to motion windows, or relaxing conn to 50 ms. I won't claim 24 h until measured.

### 4.3 PPG dominates — everything else is noise

PPG energy ≈ **LED current × pulse width × sample rate** — all three firmware-controllable, spanning two orders of magnitude. Levers, best first:

1. **Adaptive sampling gated on IMU.** Measure HR only when still — motion artifact makes PPG-during-movement largely unusable, so this discards already-poor data (a moving hand is the worst-case PPG environment).
2. **Closed-loop LED drive.** Use the AFE's ambient/gain loop for the minimum LED current clearing the SNR threshold for *this* finger, not a fixed worst case — skin tone, perfusion, and fit move it several-fold.
3. **Duty-cycle the LED, not just the sample rate.** Shorter pulses at higher instantaneous current hold SNR at lower average energy, up to the AFE settling limit.

MCU clock/sleep/compiler tuning comes only after: shaving 5 µA off the M33 is meaningless while PPG spends 400.

### 4.4 Mechanism

Zephyr tickless PM; M33 in System OFF + RAM retention between BLE events; device runtime PM + load-switch gating on unused rails; all wakeups interrupt-driven (only periodic timer: the low-power RTC). Consolidate wake sources — align batch processing to BLE connection events.

---

## 5. Future Sensor Expansion

### 5.1 Bus allocation

| Bus | Sensors | Why |
|---|---|---|
| SPIM (high speed) | IMU, PPG | Burst FIFO reads; SPI @8 MHz empties a 512-sample FIFO far faster than TWIM, and radio-on time is energy |
| TWIM (I²C) | Temp, fuel gauge, future slow sensors (SpO₂ aux, ambient light, magnetometer) | Cheap on pins; trivial rates |
| QSPI / external flash | Offline log storage, if RRAM insufficient | |
| Direct GPIO / AIN | Cap touch (nRF54L15 comparator), motor drive, all INT lines | |

Rule of thumb: FIFO + burst-read → SPI; ≤ 10 Hz → I²C. Reserve two spare GPIOs to test pads and one spare I²C address block.

### 5.2 Capability/descriptor-driven design — the key point

Sensors register into a static table:

```c
struct sensor_desc {
    uint8_t  id;
    uint8_t  record_type;      /* TLV tag */
    uint16_t record_size;
    uint16_t default_rate_mhz;
    const struct sensor_vtable *ops;  /* init, start, stop, read, power_gate */
    struct ring_buffer *rb;
};
```

The BLE **capability characteristic is generated from this table at build time**. Adding a sensor is *a table entry plus a driver*: the GATT layout doesn't change, the protocol version doesn't bump, and — critically — **the app needs no coupled release**. It reads the descriptor at connect, discovers `sensor_id = 7, record_type = 0x21, record_size = 6`, and renders it generically or ignores it. An app built before the sensor existed keeps working.

The alternative — a hand-written GATT layout per sensor set — forces lockstep firmware/app releases for every hardware variant, and an open platform will have many variants we don't control.

### 5.3 Variants and compatibility

- **Devicetree overlays per variant.** `ring_rev_c.overlay` adds the node; the table populates from devicetree, so the image self-describes.
- **Versioned protocol** in the capability descriptor; the app negotiates down.
- **Forward-compatible framing.** TLV/type-tagged records make unknown types skippable, not fatal. Never reuse a type tag for a different layout.
- **Dual-image versioning** (with PPR, the table lives in shared memory): both images build together, gated on the §1.3 layout hash.

### 5.4 Physical limits

Board area, not software, is the real constraint. Reserve pads and INT routing now even for sensors we won't ship (unpopulated footprints are near-free); keep the antenna keep-out sacred (a part near it costs range); any optical sensor needs a skin-facing window plus optical isolation from PPG — mechanical before electrical. Adding one sensor may force a full mechanical re-spin.

---

## 6. Battery Life Trade-Offs: 24-Hour vs 1-Week

Not two settings of one product — two products sharing a BOM. The gap is roughly **7×** in average current (119 µA vs 833 µA), spent almost entirely on PPG and radio duty cycle. Per-sensor config is in §4.2; the higher-level contrast:

| Dimension | Week mode | 24-h research mode |
|---|---|---|
| BLE interval | 400–1000 ms, latency 4–10 | 15–50 ms, latency 0 |
| Data delivery | Batched, delay-tolerant (seconds) | Near-real-time streaming |
| Offline logging | Primary path | Secondary |
| Est. average / runtime @ 20 mAh | ~108 µA → ~7.7 days | ~878 µA → ~22.8 h (misses) |
| Data yield | Sparse, trend-quality | Dense, raw, publication-quality |

What's traded is **data density for time**: raw continuous PPG and 50 Hz IMU for the researcher, a week of sleep/step trends for the consumer. Ship both as explicit named modes over the stream-control characteristic, the capability descriptor advertising supported modes — not a vague "power saving" slider that muddies data provenance.

- **Battery capacity isn't the lever.** 20 → 25 mAh buys 25% (and ring volume likely gives *less*, not more — §4.1); halving PPG duty buys ~30% of the week budget. Optimise duty cycle; treat the cell as fixed by mechanical design.
- **Research mode assumes charging discipline.** Design the cradle and low-battery UX around a daily charge; firmware refuses research mode below ~40% SoC rather than dying mid-study.

---

## 7. Assumptions, Risks and Unknowns

### Assumptions

| # | Assumption | If wrong |
|---|---|---|
| A1 | 20 mAh usable cell | Budgets scale linearly, but **likely optimistic** (see §4.1) — the binding constraint on the whole design; first thing to measure. |
| A2 | Current figures are estimates; no IMU/PPG/temp parts fixed | Datasheets could move any line ±3×, PPG especially |
| A3 | PPG is the dominant consumer | If the AFE is far better, BLE becomes the top lever and §4.3 reorders |
| A5 | RRAM (~1.5 MB) suffices for offline logging | Add external QSPI flash — area and power not budgeted |

### Risks

Top risks (secondary risks — motor transients, iOS/Android param divergence, charge-pad corrosion/ESD, NCS churn — are addressed inline in §1–§3).

| Risk | Sev | Mitigation |
|---|---|---|
| Antenna detuning by the finger; poor field range | High | Finger-phantom fixture from day one; measure TRP on a hand; budget a chip-antenna alternative |
| Research mode misses 24 h (est. 22.8 h) | High | Trim PPG rate / gate gyro; measure before committing |
| Skin temperature corrupted by self-heating | High | Thermal isolation, placement, blackout windows; document as relative-not-absolute |
| PPG motion artifact makes HR unreliable in activity | High | Adaptive sampling; document when HR is valid |
| PPR/M33 shared-memory corruption on version skew | Med | Layout-hash boot gate; SPSC lock-free indices; fall back to PPI/DMA-only |

### Unknowns — and how I would resolve each

1. **Does PPR actually beat PPI + DMA on the M33?** Instrument both paths on the EVK with a PPK II, same load and wall time, compare mAh. Gates §1.3.
2. **Real PPG energy per measurement** for the chosen AFE/LEDs at the SNR we need, on real fingers across skin tones — bench sweep of LED current × pulse width × rate vs a reference HR monitor. Decides whether either budget is achievable.
3. **Cold-start vs standby crossover per rail-gated sensor.** Power-up + re-init + settle energy vs standby draw over the gating interval; per-part, decides §2's gating policy.
4. **Antenna performance under load.** Measured TRP/TIS with the ring worn, both antenna candidates.
5. **Actual regulator and leakage quiescent on assembled hardware** — easily 3× the estimate, which matters on a 119 µA budget.
