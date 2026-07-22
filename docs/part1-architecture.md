# Part 1 — System Architecture

**Product:** next-revision open-source smart ring  
**MCU:** Nordic nRF54L15  
**Sensors / IO:** IMU, PPG (heart rate), temperature, capacitive touch, vibrator motor  
**Connectivity:** BLE 5.x  
**Power:** rechargeable Li-Po (~20 mAh usable assumed)  
**App:** built by a parallel team → the BLE contract must be clear, versioned, and discoverable

All current and battery figures are **order-of-magnitude estimates**. They need bench validation on hardware (see §7).

---

## 1. Firmware architecture

### 1.1 RTOS: Zephyr (nRF Connect SDK)

**Choice: Zephyr + NCS.**

| Why | Detail |
|---|---|
| Nordic stack | Maintained BLE controller and drivers |
| Board variants | Devicetree / Kconfig — new sensors as overlays, not forks |
| Host testing | `native_sim` + portable core → CI without hardware |
| Updates | MCUboot + SMP/mcumgr → signed DFU over BLE |

**Not bare-metal:** six mixed-rate streams, BLE timing, offline log, and DFU are an RTOS-shaped problem. Power will be dominated by radio and PPG LEDs, not the RTOS tick.

**Cost to accept:** NCS version churn. Mitigate with a pinned SDK revision and a containerized build.

### 1.2 Software layers

```
BLE layer            pulls data, notifies phone, handles backfill
Sensor manager       ring buffers, sequence numbers, drop counts  (portable C11)
Port layer           lock + monotonic time only  (~40 lines per platform)
Drivers / HAL        Zephyr sensor API or nrfx
```

**Sensor manager rules:**
- Pure C11 — no Zephyr types → host unit tests and sanitizers
- No `malloc`; fixed static buffers
- Buffer full → **overwrite oldest**, keep newest (right trade for a wearable after a long disconnect)
- Every drop is counted; each sample has a sequence number so the phone can see gaps
- Capacities are product numbers (100 IMU / 20 temp), not power-of-two masks — wrap with modulo

**Port layer** is the only OS dependency: lock acquire/release + `time_now_us()`.  
Zephyr → spinlock / uptime; host → pthread / `CLOCK_MONOTONIC`.

**BLE is a consumer only.** Drivers never call BLE. The radio thread drains batches when a connection event needs filling. That avoids priority inversion and keeps sensor timing independent of the link.

### 1.3 Dual-core (PPR) — measure before committing

nRF54L15 has a main Cortex-M33 plus smaller co-processors (PPR / FLPR).

| Approach | Idea |
|---|---|
| **PPR path** | Small core runs continuous sampling; M33 mostly sleeps until BLE / UI work |
| **Simpler path** | Keep sampling on M33 with DMA + hardware triggers (PPI/DPPI) so the CPU still sleeps most of the time |

**PPR win:** always-on 25–50 Hz sampling without waking the big core every sample.  
**PPR cost:** shared memory protocol, cross-core locking, dual firmware images, version skew can corrupt the layout.

**Decision rule:** first EVK power measurement of PPR vs DMA-only. DMA may deliver most of the energy win at far less complexity. Do not commit the split on paper alone.

### 1.4 Concurrency

| Context | Job |
|---|---|
| Sensor IRQ / timer | Timestamp, kick DMA |
| Acquisition | FIFO → ring buffer |
| BLE stack | Radio timing — never blocked by app code |
| Consumer | Bulk drain → GATT notify or flash log |
| Idle | Tickless; deep sleep when idle |

Ring buffers are the only shared mutable state. Critical sections stay short: index math + one bulk `memcpy`, not per-record work under the lock. Target worst-case hold well under BLE timing margins (check with a GPIO toggle on a logic analyzer).

---

## 2. Hardware architecture

### PCB
Rigid-flex: flex spine with rigid islands (MCU/radio, PPG/optics, battery/charge).  
Inner-diameter clearance is tight — **height** matters as much as footprint.

### Antenna
A finger is lossy and detunes resonance. Several dB of range is easy to lose.

- Hard keep-out: no battery, pour, or motor under the antenna
- Tune on a **finger phantom / worn fixture**, never free space only
- Prefer a small **chip antenna** if printed trace geometry is unreliable on curved flex  
  Final pick from measured DVT comparison

### Power tree
Li-Po (≈3.0–4.2 V) → nRF54L15 internal DCDC, plus three load-switched rails:

| Rail | Supplies |
|---|---|
| `VDD_SENSOR_DIG` | IMU, temp, touch |
| `VDD_ANALOG` | PPG AFE (filtered, separate) |
| `VDD_MOTOR` | Haptic driver + local bulk cap |

Rail gating is not optional on a ~100 µA week budget: three sensors at 3 µA standby is already a large fraction of the budget.  
Tradeoff: cold-start + re-init energy vs standby — decide **per part** after measurement.

**PPG gets its own filtered analog rail.** BLE TX bursts (tens of mA) on a shared rail couple into a microvolt photodiode front-end and look like physiology. Separate rail, local bulk, star ground; schedule sampling away from radio events where possible.

### Buses
One shared I²C for every sensor is fragile: one hung slave (brownout, ESD, clock stretch) kills all sensing.

**Rev plan:**
- PPG (and preferably IMU) on **SPI** for isolation and FIFO drain speed
- Slow parts (temp, fuel gauge, future light/mag) on **I²C**
- Firmware: I²C recovery (clock pulses → controller reset → timeout → re-init) + per-sensor power kill

### Charging and battery gauge
- **Contact pads**, not Qi (volume and coupling are poor at ring size)
- Gold-over-nickel pads, equipotential when idle; **TVS** on both pads; charge-inhibit if cradle is wet/shorted
- Voltage-only SoC is weak in the flat Li-Po region → prefer a **fuel-gauge IC** (~1–3 µA). Fallback: voltage + learned model, marked low-confidence

### Interrupts, thermal, motor, test
- Every sensor INT can wake from deep sleep. Avoid polling.
- IMU wake-on-motion gates adaptive PPG (see §4).
- Temperature reads a mix of skin + MCU + LED + motor heat. Isolate thermally; document as **trends/deltas only**, not core body temp.
- Motor is electrical and mechanical noise: flyback + real driver; flag IMU/PPG samples during haptics invalid.
- SWD pads (ideally reachable after assembly), RTT not UART, pogo production fixture, self-test mode over BLE.

---

## 3. BLE interface design

### 3.1 Services

| Service | Role |
|---|---|
| Device Information (SIG) | Model, HW/FW rev, serial |
| Battery Service (SIG) | Level for the OS |
| Current Time (SIG) | Coarse clock sync |
| **Ring Data Service (custom UUID)** | Everything product-specific |

**Custom characteristics:**

| Characteristic | Props | Purpose |
|---|---|---|
| Capability | Read | Sensor list, rates, sizes, protocol version |
| Stream control | Write | Start/stop, rate, week vs research mode |
| Sensor data | Notify | Live batched binary records |
| Configuration | R/W | Thresholds, calibration, log policy |
| Control point | Write + Indicate | Haptic, calibrate, time sync, backfill, factory reset |
| Log / backfill | Notify | History after disconnect |

### 3.2 Batching is the energy design

50 Hz IMU is only ~600 B/s — easy for 2M PHY. The expensive part is **radio wakeups**.

- One notification per sample = ~50 wakes/s → bad for a 20 mAh cell
- Fill the negotiated ATT MTU; pack many samples per notification  
  Example: 16-byte header + 14-byte IMU records → **~16 samples per notify** ≈ one event every 320 ms at 50 Hz
- Use **2M PHY**, **DLE**, negotiate MTU at connect, **derive** batch size (Android/iOS grant different MTUs; hardcoding 244 fragments)

Implemented in `src/ble_batch.c`; covered by `tests/test_ble_batch.c`.

### 3.3 Wire format (summary)

Little-endian, field-by-field writers — **never** `memcpy` a C struct onto the air (padding/endian are not a cross-team contract).

```
Header:
  type, count, first_seq, base_t_us, dropped (saturates at 0xFFFF)

IMU record:  dt_us, accel[3], gyro[3]
Temp record: dt_ms, milli_celsius
```

- Sequence + count → order and gap detection
- Drop count → phone quantifies loss
- `dt` is **inter-sample gap**, not cumulative offset from base (avoids u16 overflow)
- Units differ per stream (µs for IMU, ms for temp)

Later: `flags` (backfill / haptic-corrupt / overflow) and `sensor_id` for forward-compatible types.

### 3.4 Connection parameters

Peripheral **requests** params (`bt_conn_le_param_update`). Phone defaults are often too aggressive for a tiny cell.

| Mode | Conn interval | Slave latency | Intent |
|---|---|---|---|
| Week | 400–1000 ms | 4–10 | Background logging, low radio duty |
| Research | 15–50 ms | 0 | Low latency stream, ~7× radio energy |

Test **iOS and Android** early — iOS rejects parameters outside its accessory rules.

### 3.5 Security, offline log, DFU

- LESC pairing + bonding; reject legacy pairing
- Resolvable private addresses with rotation (static MAC = tracking beacon)
- Encrypt/authenticate data and control characteristics; accept-list after bond
- No phone → drain ring buffers to flash/RRAM log; on reconnect, backfill from last sequence, rate-limited so live data still flows
- MCUboot dual-slot + revert on failure; **signed but documented keys** so researchers can ship their own images while stock units reject unsigned firmware

---

## 4. Power management

### 4.1 Budgets (20 mAh usable)

| Target | Average current budget |
|---|---|
| 1 week (168 h) | **~119 µA** |
| 24 hours | **~833 µA** |

**Likely optimistic.** Public teardowns of shipping rings in this class show cells near **~10 mAh**. If so, budgets become ~61 µA / ~425 µA, and a ~108 µA week design lasts **under 4 days**. Keep the 20 mAh math because the method scales linearly — but **measure cell capacity first**.

### 4.2 Rough current split (µA average)

| Subsystem | Week mode | Research mode |
|---|---|---|
| Main CPU (sleep-heavy → more active) | 12 | 120 |
| Acquisition (PPR or equiv.) | 15 | 60 |
| IMU | 15 | 150 |
| **PPG** | **30** | **400** |
| Temperature | 1 | 3 |
| Cap touch | 5 | 10 |
| BLE | 20 | 120 |
| Quiescent + gauge + leakage | 10 | 15 |
| **Total** | **~108 µA → ~7.7 days** | **~878 µA → ~22.8 h** |

Week mode assumptions: light IMU, sparse PPG, slow BLE, batched.  
Research: full IMU, continuous PPG, fast connection.

**Research mode misses 24 h by ~5% on this estimate.** Fix candidates: lower PPG rate, gate gyro to motion, 50 ms conn. Do not claim 24 h until measured.

### 4.3 Save energy where it matters

PPG energy ≈ LED current × pulse width × sample rate. Optimize that before MCU micro-optimizations.

1. **Sample HR when still** (motion artifact often makes active PPG useless anyway)
2. **Closed-loop LED current** — minimum that meets SNR for this finger
3. **Shorter LED pulses** if the AFE still settles
4. Then: batch BLE, long week-mode interval, rail gating, deep sleep

### 4.4 Mechanisms
Tickless Zephyr PM; deep sleep + RAM retention between BLE events; runtime PM + load switches; interrupt-driven wake; align batch work to connection events.

---

## 5. Future sensor expansion

### 5.1 Bus map

| Bus | Sensors | Why |
|---|---|---|
| SPI | IMU, PPG | Fast FIFO drain |
| I²C | Temp, gauge, future slow sensors | Cheap, low rate |
| QSPI / flash | Offline log if needed | Capacity |
| GPIO / AIN | Touch, motor, all IRQs | Direct |

Rule of thumb: FIFO + burst → SPI; ≤10 Hz → I²C. Leave spare GPIOs and I²C address space.

### 5.2 Discoverable sensor table

```c
struct sensor_desc {
    uint8_t  id;
    uint8_t  record_type;   /* wire tag */
    uint16_t record_size;
    uint16_t default_rate_mhz;
    const struct sensor_vtable *ops;
    struct ring_buffer *rb;
};
```

BLE **capability** is generated from this table. Adding a sensor = table entry + driver. GATT layout stays stable; old apps ignore unknown types. Avoid lockstep app/firmware releases for every hardware variant.

### 5.3 Compatibility
- Devicetree overlays per board revision
- Protocol version in capability; app negotiates down
- Type-tagged records; never reuse a tag for a new layout
- If dual-core: shared layout hash at boot; refuse mismatch

### 5.4 Physical limit
Board area and antenna keep-out beat software. Reserve unpopulated pads early. Optical sensors need a skin window and isolation from PPG — mechanical first.

---

## 6. 24-hour vs 1-week trade-off

Two **named product modes**, not a vague “battery saver” slider (provenance of data matters for researchers).

| | Week mode | Research mode |
|---|---|---|
| BLE | 400–1000 ms, latency OK | 15–50 ms, low latency |
| Delivery | Batched, delay-tolerant | Near real-time |
| Offline log | Primary | Secondary |
| Est. @ 20 mAh | ~108 µA → ~7.7 days | ~878 µA → ~22.8 h (short) |
| Data | Sparse trends | Dense raw streams |

**Trade:** data density for runtime.  
Cell size is mostly fixed by mechanics; **duty cycle** (especially PPG) is the real lever.  
Research mode needs daily charge UX; refuse start below ~40% SoC rather than die mid-study.

---

## 7. Assumptions, risks, unknowns

### Assumptions

| # | Assumption | If wrong |
|---|---|---|
| A1 | ~20 mAh usable cell | Likely high; all runtimes scale down — measure first |
| A2 | Current table is estimates; parts not frozen | Any line can move a lot, PPG especially |
| A3 | PPG is the dominant load | If not, BLE becomes the top optimization |
| A4 | On-chip log memory enough | May need external flash (area + power) |

### Top risks

| Risk | Severity | Mitigation |
|---|---|---|
| Antenna detuned by finger | High | Finger phantom; protect keep-out; chip-antenna option |
| Research mode &lt; 24 h | High | Cut PPG / gate gyro / slow BLE; measure before commit |
| Temperature self-heating | High | Placement, isolation, blackout windows; relative only |
| Bad HR while moving | High | Adaptive sampling; quality flags |
| Dual-core version skew | Med | Layout hash at boot; or stay single-core + DMA |

Secondary risks (called out inline above): motor rail sag, iOS/Android param policy, charge-pad corrosion/ESD, NCS churn.

### Unknowns — how to close them

1. **PPR vs DMA energy** — same workload on EVK, PPK II, compare mAh  
2. **Real PPG energy at usable SNR** — LED current × width × rate on real fingers / skin tones vs reference HR  
3. **Gating vs standby crossover** — per sensor, over realistic off intervals  
4. **Worn antenna** — TRP/TIS on finger for both antenna options  
5. **Assembled leakage / regulator IQ** — can be several× the spreadsheet; matters on a 119 µA budget  

---

## Design in one paragraph

Use Zephyr; keep a portable no-alloc sensor manager with sequence numbers and drop accounting; make BLE a pure consumer that batches to the negotiated MTU. On hardware, isolate PPG power and buses, design the antenna for a worn finger, and gate unused rails. Spend power budget on the few controls that matter (PPG duty, BLE wake rate), with explicit **week** and **research** modes. Expand sensors through a discoverable capability table. Treat every µA number as a hypothesis until measured on EVK and DVT.
