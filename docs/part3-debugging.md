# Part 3 — Debugging Three Field Issues

**Method note, applied to all three issues.** 
I order investigation by *cheapest-and-most-discriminating first*. 
A cheap experiment that can kill three hypotheses at once beats an expensive one that confirms the favourite. 
Every hypothesis is paired with a predicted outcome, what I would specifically see if it were true.

Second rule: **reproduce deterministically before changing anything.** A fix applied without a confirmed reproduction is a guess.

Third rule: **bisect the problem space** - electrical vs firmware vs algorithmic - and change one variable at a time.

---

## Issue A - PPG becomes unreliable when BLE throughput increases

The correlation with throughput is the strongest clue: 
something that scales with radio activity is corrupting an analog acquisition. 
That leaves power, radiated, or time (CPU/bus/IRQ) coupling. Those three are separable with cheap experiments.

### Hypothesis matrix

| # | Hypothesis | Prediction if true | Cheapest experiment |
|---|---|---|---|
| A1 | **Power-rail coupling.** BLE TX bursts (≈5–15 mA, ~1 ms slots) sag a rail shared with the PPG AFE/LED driver; | Noise present with **LEDs off / photodiode dark**; ripple bursts on the analog rail time-aligned to connection events; amplitude scales with TX power | Power the PPG AFE from a **clean bench supply**, cut the shared rail. If the problem vanishes → conducted coupling. Highest-information single test|
| A2 | **Bus contention / FIFO timing.** PPG FIFO reads on a shared I²C/SPI bus are delayed by BLE work; FIFO overflows or sample timestamps jitter | Sample-arrival jitter histogram, peaks at BLE connection-interval; FIFO overflow/watermark flags set; noise appears in the *frequency* domain even though raw sample values look sane | Log sample-arrival timestamps + FIFO status; sniffer or a GPIO toggled at radio-active |
| A3 | **CPU starvation / priority inversion.** BLE host+controller preempt the PPG processing thread; deadlines missed | Thread-level: PPG thread runtime shifts late by ~ the radio slot; SystemView shows preemption immediately before each corrupted block | Replace the radio with an **equivalent CPU busy-load** at the same duty cycle, radio off. If corruption persists → CPU/bus, not RF/power. This one test kills A1, A4, A5 or kills A2/A3 |
| A4 | **Interrupt latency.** Long critical sections in the controller delay the PPG DRDY IRQ | GPIO-in-ISR trace shows DRDY→ISR-entry latency spikes of tens of µs aligned to radio windows; jitter, not loss | Toggle a GPIO in the DRDY ISR, capture DRDY line and GPIO on a logic analyzer, histogram the delta |
| A5 | **RF / EMI rectification.** 2.4 GHz energy rectified in the AFE input or antenna coupling into the PPG flex | Noise persists on bench supply (kills A1) *and* persists with dark photodiode, *and* scales with **TX power** but not with number of transmissions | Vary **TX power** and **PHY (1M vs 2M)** independently of throughput. Energy-per-TX vs number-of-wakeups separate cleanly here |
| A6 | **Thermal / self-heating.** Sustained TX warms the die/skin interface; PPG DC baseline drifts | Corruption ramps over tens of seconds and does not track individual events; recovers slowly after radio stops | Thermocouple or on-die temp sensor logged alongside PPG baseline; step the radio load and look for a time constant |

### Debugging process

1. **Build the deterministic repro first.** Fixed BLE throughput ladder (idle → 15 ms interval → max-throughput notification stream), fixed PPG rate, fixed optical target (a static phantom or matte grey card, *not* a finger — a finger adds motion and perfusion variance and destroys determinism). Metric: SNR in the HR band, plus a hard sample-loss/jitter count. If I cannot get a monotonic degradation vs throughput on the bench, I do not yet understand the bug.
2. **Cut the space in three.** Run experiment A3 (radio→busy-loop) and A1 (bench supply). Two experiments partition electrical vs temporal vs RF. Everything after this is refinement inside one third.
3. **Dark-reading test.** LEDs off, photodiode dark, BLE at max. Any signal that is not dark is electrical, full stop. This is a five-minute test that removes the entire "optical/algorithmic" branch, which is why it goes early despite not being my leading hypothesis.
4. **Time correlation.** Log PPG sample-arrival timestamps and cross-correlate the jitter series with on-air connection-event timing from a sniffer. **Correlation in time is the smoking gun** — noise that is merely "worse when busy" is weak evidence; noise phase-locked to connection anchor points is proof.
5. **Only then** touch the algorithm. Jittered timestamps look *identical* to a noisy signal to a frequency-domain HR/HRV estimator; I will not accept "the algorithm is noisy" until I have proven the time base is clean.

### Tools

Oscilloscope with a low-noise probe on the analog rail, **AC-coupled at high vertical sensitivity** (mV/div; expect µV-to-low-mV bursts at connection-event cadence — DC-coupled at 1 V/div you will see nothing and wrongly exonerate the rail). Spectrum analyzer + near-field H-field probe over the AFE and PPG flex for A5. Current probe or **Nordic PPK II** for the burst current profile. Logic analyzer / Saleae on the sensor bus plus a GPIO toggled inside the PPG ISR and another toggled at radio-active. **SEGGER SystemView / Zephyr tracing** for thread and IRQ timing. **RTT logging, never UART** — UART at these rates perturbs exactly the timing under investigation and manufactures a different bug. **BLE sniffer** (nRF Sniffer, or Ellisys if available) to correlate on-air events with corrupted blocks. Raw-sample capture to a host for offline analysis, so signal work is reproducible on recorded data rather than re-run on live hardware.

### Design changes I would consider

- **Dedicated, filtered analog rail with its own LDO** for the AFE and LED driver. PSRR is the specification that matters, and cheap LDO PSRR collapses above ~10 kHz — which is precisely where a ~1 ms burst's spectral content lives. A "60 dB PSRR" part quoted at 1 kHz buys nothing here. Add local bulk plus an LC/ferrite pre-filter and star-ground the analog return.
- Separate LED drive current path from the AFE supply; LED pulses are the largest local dI/dt and should not share impedance with the signal chain.
- **PPG on a dedicated SPI with DMA and FIFO watermark-driven burst reads**, so acquisition timing stops depending on CPU responsiveness.
- **Hardware-timestamp samples at the sensor** (or capture a DRDY edge with a timer capture channel) rather than timestamping at read time. This decouples the time base from software latency entirely and makes A2/A3/A4 structurally impossible.
- **Run acquisition on the RISC-V PPR coprocessor**, so BLE work on the M33 cannot perturb it. This is the architecturally clean answer on nRF54L15 and I would push for it.
- Product-level mitigation: schedule PPG acquisition windows to **interleave with, not collide with**, BLE connection events (the connection anchor is known to firmware).
- **Flag or reject samples whose inter-sample jitter exceeds a threshold** rather than silently emitting a heart rate derived from a corrupted time base. Emitting a confidently wrong HR is worse than emitting none.

---

## Issue B — Battery life ~40% below expectation

**"40% lower than expected" is only meaningful if "expected" was itself measured.** The expectation here is a ~119 µA average budget built from a spreadsheet. Spreadsheets contain duty cycles someone guessed and datasheet "typical at 25 °C" figures that no shipping unit ever achieves. The first suspect is the model, and I say that plainly because auditing it costs an afternoon and it is very often the actual bug. 40% is also suspiciously close to "one subsystem you forgot" or "the connection interval is not what you asked for."

### Hypothesis matrix

| # | Hypothesis | Predicted signature if true | Cheapest discriminating experiment |
|---|---|---|---|
| B1 | **The model is wrong**, not the hardware | Measured average matches a corrected model line-by-line; no single anomalous current feature in the trace | Rebuild the budget as a *measured* budget (below). Kills or confirms everything else's framing |
| B2 | **MCU never reaches the budgeted deep sleep state** (peripheral still clocked, HFCLK request never released, debug/UART left enabled) | Sleep-floor current is 10–100× spec; flat elevated baseline with no structure; `pm_state` residency counters show System-ON idle, not the deep state | PPK II floor measurement with all app threads suspended; read residency counters. **Detach the debugger first** — an attached debugger alone can cost hundreds of µA and is the classic false alarm that wastes a day |
| B3 | **Floating / misconfigured GPIO**, or a pull-up fighting an external pull-down | Baseline elevated by tens of µA; sensitive to touching the board or to humidity; a static DC current with no time structure | Full GPIO state audit against schematic; drive all unused pins to a defined state and re-measure the floor |
| B4 | **Sensor in standby-but-not-shutdown**, or a rail never gated | Floor drops by a discrete, identifiable step when that sensor's rail is cut | Cut/lift each sensor rail one at a time and re-measure — incremental attribution |
| B5 | **Negotiated BLE connection interval ≠ requested.** The central (phone) may reject the peripheral's parameters; iOS and Android apply their own policies | Radio wakeup cadence in the current trace is faster than the requested interval | **Sniffer** — read the actual interval on air. Never trust the requested value or the app's log |
| B6 | **Excess wakeups.** Timer waking far more often than the workload needs; per-sample notifications instead of batched | Trace shows many small current spikes with no useful work between them | Count wakeups per minute in the trace and divide by the workload's actual requirement |
| B7 | **Battery model error.** Overstated capacity, high internal resistance, self-discharge, protection-IC quiescent current, charge termination short of full, temperature, aging | Device current matches budget but runtime still short; delivered mAh below rating on a cycler | Discharge a cell on a **battery cycler** at the real average load and measure actual delivered capacity to the real cutoff. A 20 mAh cell has very little margin for any of these |
| B8 | **Leakage via ESD diodes into an unpowered rail.** A gated-off sensor is back-powered through its I/O clamp diodes from a still-driven bus | Gated rail sits at ~0.5 V instead of 0 V; current disappears when the MCU's bus pins are also floated | Measure the gated rail voltage with the sensor "off". Nasty, common, and invisible in firmware |
| B9 | **Moisture / flux-residue leakage.** A ring is worn on skin and sweated on | Current rises with humidity/wear; drops after cleaning and bake | Measure a fresh, cleaned board vs a field-returned unit; humidity chamber sweep |
| B10 | **Regulator quiescent current / poor µA-load efficiency.** 90% efficient at 10 mA can be dismal at 100 µA | Input current far exceeds output current × ratio at low load | Measure regulator input and output current simultaneously at the real sleep load |

### Debugging process

The central move is **incremental attribution, not staring at the total**. Staring at 200 µA tells you nothing; watching it go 3 → 11 → 46 → 198 µA as you enable subsystems tells you everything.

1. Detach the debugger. Measure a **bare board** floor.
2. Add one subsystem at a time — clocks, then each sensor, then BLE advertising, then BLE connected, then the full application — recording the delta at each step. Compare each delta against the corresponding *line* of the model. The goal is to name the wrong line, not to reduce the total.
3. **Instrument with GPIO markers** at every state transition so every current feature in the trace is attributable to a code path. Unattributed current is the thing to hunt.
4. Measure over a **full duty cycle** — averages over minutes, ideally an hour, not a millisecond snapshot. A once-per-minute 20 mA burst is invisible in a 100 ms capture and dominates the budget.
5. Read the **actual** negotiated connection interval on air, against a real phone of each OS, not a dev-board central.
6. Separately validate the cell on a cycler. Device current and battery capacity are two independent failure domains and must be measured independently before being blamed.

### Tools

**Nordic Power Profiler Kit II** (the natural instrument for this MCU and this range), an SMU or Joulescope for long µA-resolution captures with sub-µA floor, GPIO instrumentation, Zephyr `pm_state` hooks and power-state residency counters, an on-air BLE sniffer, and a battery cycler / coulomb counter.

### Design changes I would consider

- **GPIO state audit as an automated test** — a build-time or on-target check that every pin is in its intended state in every power mode.
- **Hardware rail gating with load switches** for every sensor, with bus pins driven low or tri-stated before gating, to close the B8 leakage path by construction.
- **In-firmware power-state residency counters reported over BLE as telemetry**, so field units self-report where their energy went. This turns Issue B from a lab problem into a fleet statistic.
- **A low-power regression test in CI on real hardware with a PPK.** This is the actual fix. Power regressions do not arrive in one bad commit; they creep in one commit at a time until the total is 40% off and nobody knows which change did it. A CI job that fails the build when sleep-floor or average-over-a-cycle exceeds a threshold catches the responsible commit on the day it lands.
- **Design for measurement**: current-sense shunts and rail-cut jumpers/0 Ω links on each rail so any rail can be measured in isolation without a scalpel.

---

## Issue C — 12% first-pass test failure rate in manufacturing

**12% is a number without meaning until it is decomposed.** One dominant failure mode at 11% is a completely different problem from twelve modes at 1% each — the first is a fixable defect, the second is a margin problem across the design. Without the test log I cannot tell you which this is, and I would refuse to propose an engineering fix before seeing it.

**First action — Pareto.** Failure distribution by: test step, station, date/shift, operator, panel position, board serial, and component supplier lot. This is a data pull, costs nothing, and typically collapses the problem by an order of magnitude before any engineering starts.

**Second action — the crucial split: bad product, or bad test?** False failures are extremely common, and a marginal test costs yield and hides real defects simultaneously. Retest every failed unit. If a meaningful fraction passes on retest, the test or fixture is the primary suspect. Quantify with a **Gage R&R-style study**: run known-good golden units through repeatedly and count how often they fail. If a golden unit fails 5% of the time, you have found most of your 12%.

### Hypothesis matrix

| # | Hypothesis | Predicted signature if true | Cheapest discriminating experiment |
|---|---|---|---|
| C1 | **Fixture / contact.** Pogo-pin wear or contamination, poor contact on gold pads, high fixture ground impedance | Failures cluster by station and rise with pins-since-service; retest pass rate high; failures correlate with fixture position | Retest study + swap fixtures between stations. If failures follow the fixture, not the units, it is the fixture |
| C2 | **RF test environment.** Golden-unit-referenced RF measurement in an unshielded factory with nearby Wi-Fi | RF step dominates Pareto; failures cluster by time of day/shift; measured RSSI/TX-power distribution wide and shifting | Repeat the RF step in a **shield box**. A ring's RF test is my leading test-side suspect |
| C3 | **Limits too tight.** Test limits set by wish rather than measured process spread | Parametric distribution centred fine but Cpk < 1.33; failures sit just outside a limit, not in a separate population | Compute **Cpk per test step** from parametric data. Any step with no margin is a limits problem, not a defect |
| C4 | **Tester firmware / timing marginality.** Version mismatch, borderline timeouts | Failures start abruptly at a date boundary; step-specific | Correlate failure onset with tester software change log |
| C5 | **Solder defects.** Tombstoning on small passives, insufficient paste on flex, head-in-pillow under the CSP/BGA MCU | Failures cluster by panel position and paste-stencil/reflow-profile changes; a bimodal population, not a tail | X-ray a sample of failures; cross-section the worst |
| C6 | **Flex assembly / interconnect.** Connector or ACF bond, placement on a curved substrate | Failures concentrate at the flex-dependent tests; sensitive to handling and flexing | Flex-cycle a sample; re-test before/after |
| C7 | **MSD handling / reflow damage, ESD in handling.** A ring is handled a lot | Scattered, multi-mode, no clean cluster; sensor or MCU parametric shifts | Audit ESD controls and MSD floor-life records; check whether failures track handling steps |
| C8 | **Antenna detuning by enclosure/potting.** Units pass at board test, fail RF only after final assembly | RF pass at board test, fail at final test on the same serials | **Serial-number correlation across test stages.** Requires traceability — if it does not exist, that is finding #1 |
| C9 | **Incoming lot variation.** Component or cell lot with different parametrics | Failures track supplier date code; step change on a lot boundary | Join failure data to incoming lot IDs |

### Debugging process

**Get the parametric data, not pass/fail.** This is the single highest-leverage change in this entire section. Pass/fail data can only tell you a unit failed; parametric data — measured value per test step per serial number — shows a distribution drifting toward a limit *before* it fails, tells you whether failures are a tail of one population (margin problem) or a separate population (defect), and makes Cpk computable. If the station is not logging measured values against serial numbers, fixing that comes before any other engineering.

Then: failure-mode analysis on a physical sample of failed units — X-ray under the MCU package, cross-section, optical/AOI review. Correlate to panel position and supplier lot. Check whether failures **cluster in time** (a process shift — chase the process) or are **uniform** (a design margin problem — chase the design). Compute Cpk per test to find limits with no margin.

### Tools

X-ray and cross-sectioning, AOI/AXI data, a shielded RF chamber or at minimum a shield box, boundary-scan / SWD-based structural test, a test-data analytics dashboard (even a parametric histogram per test step, with limits overlaid, is transformative), and DOE for process parameters once a candidate cause is identified.

### Design changes I would consider

- **DFT/DFM**: test points on every rail; a firmware **BIST/self-test mode** that exercises every sensor and reports *parametric* values over SWD, so the tester reads real measurements instead of inferring from a pass/fail bit.
- **Structured serial-number traceability** from panel through final assembly. Without it, C8-class questions are unanswerable.
- **Guard-banded limits derived from measured Cpk**, not guessed.
- **Shielded RF test**, non-negotiable for a 2.4 GHz product in a factory.
- **Staged testing**: test the bare board before adding the expensive battery and enclosure. A defect that reaches final test costs roughly 10× one caught at board test, and final-test failures are also harder to localise. Moving coverage earlier is the highest-ROI change on this list after parametric logging.
- Reduce hand-assembly steps; widen real margin (antenna tuning tolerance, relaxed component tolerances, thicker/reworkable flex interconnects).

---

## What I'd put in place across all three

- **Parametric data beats pass/fail, everywhere.** The same principle appears in all three issues: PPG samples need a jitter/quality metric rather than a silent HR output; power needs residency counters rather than "it lasted a week"; manufacturing needs measured values rather than a verdict. In every case the pass/fail version of the data can only tell you that something broke, never that it was about to.
- **Instrument first, hypothesize second, fix third.** Reversing that order is how a team spends three weeks on a plausible theory that was never tested against a measurement.
- **Every fix ships with a regression test that would have caught the bug**: a CI power test on real hardware with a PPK, a CI PPG-under-BLE-load test asserting SNR and jitter bounds, and an SPC chart on the line for the manufacturing mode. A fix without a regression test is a temporary fix.
- **The common thread is observability.** None of these three issues is fundamentally about a wrong component or a wrong line of code — each is about a system that could not tell anyone what it was doing. The correct fix in each case includes making the system measurable, not just making it work.
