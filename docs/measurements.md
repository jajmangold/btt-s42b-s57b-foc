# Measurements

Every claim this project makes, and how it was obtained. Results are from an
**S57B V2.0** unless stated. Where something is derived rather than measured,
it says so plainly.

## Method

Telemetry is read out of a live global in RAM over SWD **without halting the
target**, so the control loop is undisturbed while being observed. The address
is resolved from the ELF with `arm-none-eabi-nm`, so it survives relinking.
See `tools/telem.py`.

---

## Encoder

| | |
|---|---|
| Sample rate | 14.4 kHz raw, 13.3 kHz with CRC validation |
| Jitter, stationary | 6 counts of 32768 (0.066°) |
| Resolution | 15 bit, 32768 counts/rev |

**Full-range sweep.** Turning the shaft by hand covered the complete `0..32767`
range, wrapping once per revolution. Accumulated shaft angle moved −10.43 to
−52.56 rad = 6.71 revolutions, monotonic.

**Wrap arithmetic, verified numerically.** At one wrap the raw value went
333 → 26717, i.e. −6384 counts going down through zero = **−1.2241 rad**
predicted. The measured accumulated angle moved **−1.222 rad**. Agreement to
0.15%.

### The safety-word CRC — determined by experiment

The TLE5012B appends a safety word containing status bits and a CRC. The
factory firmware never reads it, so corruption goes undetected.

Rather than assert an algorithm from the datasheet, four plausible readings
were implemented and the sensor was asked which it used — 200 reads, all four
variants scored simultaneously:

| Variant | Matches |
|---|---|
| **CRC-8 poly 0x1D, seed 0xFF, over command+data, inverted** | **200 / 200** |
| same, not inverted | 0 / 200 |
| data only, inverted | 0 / 200 |
| data only, not inverted | 0 / 200 |

A clean sweep against three zeros. Status bits read healthy 200/200. Reproduce
it yourself with `cmd 12` and `src/crc_probe.h`.

Validation is now live in the read path and rejects on dead bus, bad status, or
CRC mismatch, holding the last good angle rather than acting on a value it
cannot vouch for. Cost: 16 extra SPI clocks per read, which is the 14.4 → 13.3
kHz difference above.

---

## Motor and drive

**Pole pairs: 50.** Confirmed two independent ways — by sweeping electrical
revolutions and measuring shaft movement, and from the factory firmware's own
`phase_multiplier = 12.5f`, which is exactly 50 pole pairs for a 14-bit encoder
against a 4096-entry sine table.

**Coil wiring, measured not assumed.** Energising each coil in each polarity
parks the rotor 180° electrical apart:

| | Measured | Theory |
|---|---|---|
| Coil A, + vs − | 330 counts | 327.68 |
| Coil B, + vs − | 329 counts | 327.68 |
| A+ to B+ | 162 counts | 163.84 (90°) |

The quadrature figure is the one that matters: it proves the phases are
correctly *paired and ordered*, not merely alive.

**Open-loop tracking.** Ten electrical revolutions moved 6749 counts against
6553.6 expected (ratio 1.0298). The same test 6.7× slower gave 1959 against
1966.1 (**0.9964**), showing the 3% was dynamic overshoot rather than a
systematic error.

---

## PWM carrier

`driver.pwm_frequency` **has no effect on STM32F1.** Built and flashed at
25 kHz and at 100 kHz, `TIM3` read `PSC=0 ARR=63999` in both cases — a
**1.12 kHz** carrier, against the `ARR=359` SimpleFOC's own formula gives for
100 kHz.

That is audible, and well below what the VREF RC filter needs, so VREF swings
as a square wave instead of settling to a DC current setpoint. The effect was
not subtle:

| | 1.1 kHz carrier | 140.1 kHz carrier |
|---|---|---|
| Hold hunting | 13 counts | **2 counts** |
| Commanded revolution | stalled 0.30 rad short | **within 0.0015 rad** |
| FOC loop rate | 4.4 kHz | 5.9 kHz |

**Do not try to force `ARR` from outside the library.** SimpleFOC rewrites it
inside `loopFOC()` every iteration, which strands the previous iteration's
compare value above the new `ARR` — and `CCR > ARR` parks the output at 100%
duty, i.e. full current. This firmware owns `TIM3` instead.

---

## Closed loop

| | |
|---|---|
| Alignment | `zero_electric_angle` reproducible to 4 decimals across runs |
| Hold | 2 counts (0.022°) |
| One commanded revolution | within 10 counts of 360° (0.11°) |

Five consecutive revolutions moved 6.2817–6.2847 rad against 2π = 6.28318, with
a mean final error of 6.4 counts (0.070°) and a worst of 10.

### Electronic gearing

SimpleFOC's angle cascade has **no velocity feedforward**, so it lags a ramp by
`v / P_angle` by construction. Measured 0.302 rad at 2 rad/s with P=8 — exactly
the predicted 0.25 rad, so nothing was wrong, the control law simply cannot do
better.

Running the motor in velocity mode and commanding
`v = k × master_velocity + Kp × position_error` removes the structural lag:

| | Steady-state error |
|---|---|
| Position-only (SimpleFOC angle mode) | 1429 counts (16.4°) |
| **With velocity feedforward** | **43–136 counts (0.5–1.5°)** |

A **33× improvement**. The worst-case ~490 counts is the velocity estimator
converging from zero at engage, which in practice happens before a cut begins.

---

## Full-power validation

Stepped current ramp, coil probe at each level (328 counts = healthy). Duty is
bounded by VREF <= 3V3, so 3300 mA is 100% duty and cannot be exceeded:

| Current | Duty | Coil A | Coil B |
|---|---|---|---|
| 1500 mA | 45% | +327 | -329 |
| 2000 mA | 61% | +330 | -329 |
| 2500 mA | 76% | +330 | -331 |
| 3000 mA | 91% | +330 | -332 |
| **3300 mA** | **100%** | **+329** | **-331** |

Counts creep slightly *above* 328 at higher current: more torque parks the
rotor harder onto the electrical detent.

**Sustained**: six gearing runs at 3300 mA, around 3 s of continuous drive
each, with a coil probe between every run — roughly 38 s of energised time at
full duty. Following error stayed at 90–120 counts, coil probes stayed at
321–331, and encoder faults stayed at zero. **No thermal shutdown and no
measurable degradation.**

Two observations worth keeping:

- **More current does not improve tracking on a free shaft.** 90–120 counts at
  3300 mA versus 102–112 at 800 mA. Torque is not the limiting factor when
  there is no load; the control law is. Expect this to change under load.
- The A4950 has **no fault output to the MCU**, so thermal shutdown cannot be
  detected in firmware. It would present as coils going weak and later
  recovering. Nothing of the sort was observed here, but if you run full power
  for long periods, monitor temperature physically.

---

## STEP / DIR

Pulses are counted by TIM2 in external clock mode 2, verified from the
registers on a running board:

```
CR1  = 0x00000011   CEN=1
SMCR = 0x00004700   ECE=1 (external clock mode 2), ETF=7 (input filter)
PSC  = 0   ARR = 0xFFFF
```

Position following was exercised by writing `TIM2->CNT` over SWD to simulate
counted pulses, at 3200 steps/rev:

| Pulses | Expected | Measured | Error |
|---|---|---|---|
| 800 | 1.5708 rad | 1.6038 | +172 counts |
| 1600 | 3.1416 rad | 3.1673 | +134 counts |
| 2400 | 4.7124 rad | 4.7377 | +132 counts |
| **3200** | **6.2832 rad** | **6.2855** | **+12 counts** |

`step_count` tracked the hardware counter exactly at every point. One full
commanded revolution landed within **12 counts of 360 degrees**.

The intermediate errors are the position loop settling: each sample commanded
an instantaneous quarter-turn jump, which no real host would do — a controller
ramps. They are a property of the test, not of the interface.

**Not tested:** the physical pulse input. Nothing was connected to P1, so the
optocouplers, their input filtering and real pulse timing are unverified. The
counter, the direction interrupt and the position mapping are all proven.

---

## CAN

Loopback self-test passes: an 8-byte probe frame returns byte-identical.

| | |
|---|---|
| `BTR` | `0x403C0003` → brp=4, TS1=13, TS2=4 |
| Bit rate | 18 tq/bit on 36 MHz APB1 = **exactly 500000 bit/s** |
| Sample point | 77.8% (recommended band 75–87.5%) |
| Observed | tx 167 / rx 167, TEC+REC = **0** |

Peripheral, bit timing, filters, both mailbox paths and the protocol encoding
are all proven. The physical bus is **not** yet tested.

---

## Known unverified

Stated plainly so nobody mistakes derivation for measurement:

- **Coil current has never been measured with a meter.** The scale rests on two
  independent derivations that agree (the published maximum against a
  rail-limited VREF, and the factory firmware's own encoding). It is bounded —
  VREF cannot exceed the rail, so the worst case is the board's rated maximum —
  but it is not measured.
- **S42B V2.0 has not been run.** Support is derived from identical silicon.
- **P1 isolation is inferred** from the input pull-up configuration and the
  presence of a `COM` pin, not confirmed electrically.
- **The physical CAN bus is untested**; only loopback has run.
