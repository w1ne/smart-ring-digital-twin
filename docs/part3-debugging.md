
**Make the system measurable first; then the root cause is usually obvious.**

# Part 3. Debugging Field Issues

## Method (all three)

**Cheapest discriminating test first**. one experiment that kills several hypotheses.
**Reproduce deterministically before changing anything**, a fix without a confirmed repro is a guess.
**Bisect**, electrical vs firmware vs algorithm; change one variable at a time.
Every hypothesis needs a **predicted signature** (“if true, I will see X”).

---

## Issue A. PPG unreliable when BLE throughput rises

**Clue:** scales with radio activity → power coupling, RF/EMI, or timing (CPU/bus/IRQ). 
Those three are separable with cheap tests.

### Likely causes

| # | Cause | If true, you see… | Cheap test |
|---|---|---|---|
| A1 | **Shared power rail** sag on BLE TX | Noise with LEDs off; rail ripple at connection events | Power PPG from a **bench supply**; if fixed → conducted coupling |
| A2 | **Bus / FIFO timing** delayed by BLE | Jitter peaks at conn interval; FIFO overflow flags | Log sample timestamps + FIFO status; GPIO on radio-active |
| A3 | **CPU starvation** (BLE preempts PPG) | PPG thread late during radio slots | Replace radio with **CPU busy-loop**, radio off. Still broken → CPU/bus |
| A4 | **IRQ latency** (long critical sections) | DRDY→ISR delay spikes at radio windows | GPIO in DRDY ISR + logic analyzer |
| A5 | **RF / EMI** into AFE or flex | Survives bench supply + dark PD; scales with **TX power** more than packet rate | Sweep TX power and PHY (1M vs 2M) independently |
| A6 | **Thermal** from sustained TX | Slow baseline drift| Log temp + PPG DC while stepping radio load |

### Process

1. **Fixed repro:** BLE load ladder (idle → short interval → max notify), fixed PPG rate, **optical phantom, niot live finger**. Metrics: SNR + sample loss/jitter. Need monotonic degradation vs throughput.
2. **Partition:** A3 (busy-loop) + A1 (bench supply) → electrical vs time vs RF.
3. **Dark reading:** LEDs off, max BLE. Non-zero “signal” = electrical.
4. **Time correlation:** PPG timestamps vs sniffer connection events. Phase-lock to radio anchors = proof; “worse when busy” alone is weak.
5. **Algorithm last.** Jittered timestamps look like noise to HR estimators. Don’t blame the algo until the time base is clean.

### Tools
Scope on analog rail (**AC couple, mV/div** , DC at 1 V/div misses bursts), PPK II, logic analyzer (bus + IRQs), SystemView/Zephyr trace, **RTT**, BLE sniffer, near-field probe if EMI suspected, host capture of raw samples.

### Design changes
- Filtered **dedicated analog rail** for AFE (PSRR at >10 kHz matters for ~1 ms TX bursts; 1 kHz datasheet PSRR is not enough); separate LED path; star ground
- PPG on **dedicated SPI + DMA + FIFO watermark**
- **Hardware timestamps** (sensor or timer capture on DRDY) — kills software-latency classes A2–A4
- Optional: acquisition on **PPR** so BLE on M33 cannot preempt it
- Interleave PPG windows with known connection anchors
- **Reject/flag** high-jitter samples — wrong confident HR is worse than no HR

---

## Issue B. Battery life ~40% below expectation

**First question:** was “expected” measured, or a spreadsheet? Spreadsheets use guessed duty cycles and “typical @ 25 °C” numbers. **Audit the model first** often that *is* the bug. 40% also looks like “one forgotten subsystem”;

### Likely causes

| # | Cause | Signature | Cheap test |
|---|---|---|---|
| B1 | **Wrong model** | Measured total matches a corrected line-by-line budget | Rebuild budget from measurements |
| B2 | **Never reaches deep sleep** | Floor 10–100× high; flat baseline | PPK floor, all app suspended; check PM residency. **Unplug debugger** (classic false alarm) |
| B3 | **Bad GPIO** (float / fighting pulls) | Tens of µA DC; touch/humidity sensitive | GPIO audit; force unused pins to known state |
| B4 | **Sensor not fully off** / rail left on | Floor steps down when that rail is cut | Gate/cut one rail at a time |
| B5 | **Negotiated BLE interval ≠ requested** | Radio wakes faster than intended | **Sniffer** on a real phone (iOS + Android) |
| B6 | **Too many wakeups** (timers, per-sample notify) | Many small spikes, little work | Count wakes/min vs workload need |
| B7 | **Cell not delivering rated mAh** | Device current OK, runtime still short | Battery cycler at real load/cutoff |
| B8 | **Back-power via ESD diodes** | “Off” rail sits ~0.5 V | Measure gated rail voltage; float bus pins too |
| B9 | **Moisture / flux leakage** | Worse when worn/humid; better after clean/bake | Clean board vs field return; humidity sweep |
| B10 | **Regulator bad at µA load** | Input current ≫ output × ratio at sleep | Measure Vin and Vout current at sleep load |

### Process

**Incremental attribution, not one total number.**

1. Debugger off → bare-board floor  
2. Enable one subsystem at a time (clocks → sensors → adv → connected → full app); log each **delta** vs the model line  
3. GPIO markers on state transitions so every current spike has a name  
4. Average over a **full duty cycle** (minutes–hour), not a 100 ms snapshot  
5. Sniff real-phone connection interval  
6. Validate cell capacity separately from device current  

### Tools
Nordic **PPK II**, SMU/Joulescope for long µA captures, GPIO markers, Zephyr PM residency, BLE sniffer, battery cycler.

### Design changes
- Automated **GPIO state check** per power mode  
- Load switches on every sensor; **tri-state bus before gating** (closes B8)  
- **Power residency telemetry** over BLE (fleet-visible energy map)  
- **CI power test on real hardware + PPK** with sleep-floor / cycle-average limits (catches the commit that regressed)  
- Design for measurement: sense shunts, 0 Ω rail cuts  

---

## Issue C. 12% first-pass yield fail in manufacturing

**12% means nothing.** One mode at 11% ≠ twelve modes at 1%. 
No engineering fix before the failure breakdown.

### First two actions (before root-cause theater)

**Pareto** Anakyzer failure by test step, station, shift, operator, panel position, serial, supplier lot  
**Bad product vs bad test?** Retest fails. High retest pass rate → fixture/test. **Gage R&R:** golden units through the station repeatedly. If goldens fail 5%, that *is* most of your 12%.

### Likely causes

| # | Cause | Signature | Cheap test |
|---|---|---|---|
| C1 | **Fixture / pogo contact** | Clusters by station; follows fixture swap | Retest + swap fixtures |
| C2 | **RF test in open factory** | RF step dominates; varies by time of day | RF step in a **shield box** |
| C3 | **Limits too tight** | Distribution fine; failures just outside limit | **Cpk** per step from parametric data |
| C4 | **Tester SW / timing** | Step change on a date | Correlate to tester version history |
| C5 | **Solder defects** | Panel position / reflow correlation; bimodal | X-ray / cross-section sample |
| C6 | **Flex interconnect** | Flex-related tests; handling sensitive | Flex cycle + retest |
| C7 | **MSD / ESD / handling** | Scattered multi-mode | ESD + MSD audit vs fail timeline |
| C8 | **Antenna detuned after assembly** | Pass board RF, fail final RF, same serials | Serial correlation across stages (needs traceability) |
| C9 | **Incoming lot shift** | Step change on date code | Join fails to lot IDs |

### Process

1. Log **measured values per step per serial**
2. Histograms + limits → margin vs separate defect  
3. Physical FA on a fail sample (X-ray, cross-section, AOI)  
4. Time-clustered fails → process; uniform fails → design margin  

### Tools
X-ray/cross-section, AOI, shield box / RF chamber, SWD/BIST structural test, simple parametric dashboard.

### Design changes
- **DFT:** rail test points; firmware **self-test** that returns parametric sensor values over SWD  
- **Serial traceability** panel → final  
- Limits from measured Cpk
- **Shielded RF test** for any 2.4 GHz product  
- **Staged test:** bare board before battery/enclosure (cheaper fails earlier)  
- Fewer hand steps; more mechanical margin on antenna and flex  

---
