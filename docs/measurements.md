# Measurements

Every claim this project makes, and how it was obtained. Results are from an
**S57B V2.0** unless stated. Where something is derived rather than measured,
it says so plainly.

## The system clock -- read this before trusting any rate

The Arduino core's default configuration for a generic STM32F103 board runs the
PLL from **HSI/2**, giving 4 MHz x 16 = **64 MHz**, and never enables the
crystal at all. That is a reasonable default for a board that might not have
one. These boards do, and the difference is not cosmetic:

| | assumed (72 MHz) | actual (64 MHz) | error |
|---|---|---|---|
| UART baud | 38400 | 43191 | 12.5% |
| **CAN bit rate** | 500000 | **444444** | **11%** |
| PWM carrier | 140.1 kHz | 124.5 kHz | harmless |

**CAN loopback cannot detect this**, because both ends of a loopback share the
same wrong clock. The fault only appears the first time the drive meets a
correctly-clocked peer on a real bus — at which point nothing works and the
wiring gets the blame.

The internal RC is also only +-1-2% over temperature, against +-50 ppm for a
crystal. CAN needs the crystal regardless of whether the arithmetic is fixed.

`src/ClockConfig.cpp` overrides the core default to run HSE x9 = 72 MHz, with a
fallback to the 64 MHz HSI configuration if the crystal fails to start rather
than hanging. Verified on hardware:

```
HSEON=1 HSERDY=1   PLLSRC=HSE   PLLMUL=x9   SYSCLK = 72 MHz
UART   BRR=0x0753 -> 38400 baud, 0.00% error
CAN    brp=4      -> exactly 500000 bit/s
PWM    ARR=513    -> 140.1 kHz
```

Every rate in this firmware is now derived from the clock **actually achieved**
at runtime, not from an assumed core frequency, and `clock_ok` in telemetry
reports whether the crystal came up.

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

## UART control

Verified end to end through an ESP32 serial bridge on header P7 at 38400 8N1.

| Command | Response |
|---|---|
| `DC 1200` | `current 1200 mA` |
| **`DC 9999`** | **`current 3300 mA`** — clamped to the board maximum |
| `D5` | `align ok` |
| `D1` / `D2` | `hold` / `step/dir` |
| `D4 2.0` | `jog 2.000` |
| `D3 1.0` | `gear k=1.0000` |
| `D?` | `ready=1 shaft=11.474 rad mA=800 faults=0 steps=0` |

SimpleFOC's `Commander` verbs work alongside, so existing tooling is unchanged:
`MVP` returns `PID vel| P: 0.050`, matching the configured gain.

The `DC 9999` result is the one worth noting: a current value arriving over a
wire cannot exceed the board rating, and that is enforced rather than assumed.

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

## Encoder linearity, ballscrew axis (2026-10-16)

The ballscrew axis would not run closed loop. It moved 1–2 mm and locked solid,
in every control mode, at any current. Open-loop drive was flawless throughout.

Ruled out by measurement, in order: mechanical obstruction (free by hand),
torque (1800 mA moved *less* than 1200 mA), the position controller (velocity
mode stalls identically), motor and drivers (coil probe gives four park
positions at 1.77/1.85/1.69° spacing, confirming 1.8°/step and 50 pole pairs),
`sensor_direction` (forced `+1` and `−1` both stall), magnet strength (replaced,
no change), and coil wiring order (parks are monotonic once the electrical wrap
is accounted for).

The cause was encoder nonlinearity. Command 19 steps the field open-loop through
200 positions spanning one mechanical revolution and records the encoder at each,
with no feedback anywhere in the path. Against a least-squares line:

| | counts | mech deg | **elec deg** |
|---|---|---|---|
| peak deviation | 207 | 2.279 | **114** |
| rms deviation | 101 | 1.111 | **55** |

With 50 pole pairs every mechanical degree of encoder error becomes fifty
electrical degrees. FOC needs roughly ±20–30°; at 90° torque reaches zero and
beyond that it reverses. At 114° peak the drive actively brakes the rotor at
some shaft positions, which is exactly the observed "moves, then locks".

This also explains why raising current made it worse — more current pushing a
field that is pointing the wrong way — and why open-loop never showed a problem.

The TLE5012B is specified around ±0.5–1°, so 2.28° peak is 3–4× out of spec and
points at magnet mounting (centring, tilt, air gap) rather than the magnet.

It is also why the factory firmware keeps a per-motor calibration table and
prints `Please calibrate`: this part needs correcting against the motor it is
mounted on. The command-19 sweep produces exactly that data.

### Correcting it

`tools/calibrate.py apply` sweeps open loop, fits a line, and uploads a
256-bucket additive correction indexed by the raw reading and interpolated
between buckets. Applied inside `getSensorAngle()`, so every consumer sees
corrected angles.

| | peak | rms |
|---|---|---|
| uncorrected | 128.2 elec° | 66.1 elec° |
| corrected | 53.1 elec° | **4.9 elec°** |

Closed-loop motion went from **0 of 10** moves completing to **9 of 10**,
accurate to ±0.05 mm on a 4 mm move.

Two things worth knowing. Verification must sweep through `getSensorAngle()`,
not `readRaw()` — `readRaw()` bypasses the table, so a sweep taken that way is
blind to the thing it is measuring, and the first verification run here was
wrong for exactly that reason.

And refinement plateaus around 48–53 elec° peak while rms stays near 5°. That
residual is **stiction, not encoder error**: the rotor settles where friction
allows rather than at the true field angle, so a single-direction sweep measures
friction alongside the thing it is after. Sweeping both directions and averaging
would cancel it, which is what SimpleFOC's `CalibratedSensor` does.

**The table currently lives in RAM and is lost on reset.** Persisting it to
flash is the obvious next step.

### Status: not yet reliable, and why

The correction demonstrably works — one table produced **9 of 10** completed
moves at ±0.05 mm, from a baseline of 0 of 10. It has not been reproducible.
Successive tables from the same procedure give anywhere from 9/10 to 0/10.

The reason is that the *measurement* is unstable. Peak error across six sweeps
of the same hardware:

| sweep | 1 | 2 | 3 | 4 | 5 | 6 |
|---|---|---|---|---|---|---|
| peak (elec°) | 114 | 110 | 128 | 145 | 134 | 151 |

Two causes identified and partly addressed:

**Hysteresis.** Measured at 30.9 counts, 17 electrical degrees, between forward
and reverse passes. The sweep is now bidirectional and averaged, which removes
it from the average but not from the noise.

**The fit did not close the loop.** Stepping a 50-pole-pair motor through 50
electrical revolutions must return it exactly where it started, so a sweep spans
exactly 32768 counts. Fitting the slope freely let endpoint noise tilt the line,
producing a residual of +244 at the start and −89 at the end of the same
revolution — a 311-count step across a seam that is physically continuous. The
slope is now fixed by geometry and only the offset is fitted, which drops the
seam to 30 counts.

**The likely remaining cause is that calibration is being done with the motor
coupled to the ballscrew.** Open-loop parking assumes the rotor settles where
the field points. On a loaded axis it settles where field torque balances load
and friction, which varies with position and direction — so the sweep measures
the mechanism as much as the encoder. The correction table is a property of the
magnet, rotor and sensor alone, so it should be measured with the motor
**decoupled and unloaded**, then used once re-coupled.

### First table that passes

With the closure-constrained fit and the motor mostly decoupled from the screw:

| | peak | rms |
|---|---|---|
| uncorrected | 124.0 elec° | 66.1 elec° |
| corrected | **22.3 elec°** | **3.8 elec°** |

First sweep to land inside the ±20–30° FOC needs. Closed-loop moves: **11 of 12**
on a free shaft, and **5 of 6** after a hard reset using only the flash-stored
table with no host involvement.

Two corrections to earlier reasoning in this document, both worth keeping:

**Decoupling was not what fixed it.** Hysteresis barely changed, 30.0 → 27.4
counts, so it is magnetic detent in the motor rather than friction in the
mechanism. The closure-constrained fit was the actual fix.

**Completed moves land ~0.7% short** (0.993 rev for 1.000 commanded),
consistently and in both directions. That is steady-state error in a
proportional angle controller against detent torque, not a calibration defect,
and it will not integrate away without an I term.

### Resolved

Recalibrated on a fully decoupled shaft:

| | peak | rms |
|---|---|---|
| uncorrected | 123.7 elec° | 65.4 elec° |
| corrected | **23.8 elec°** | **6.2 elec°** |

**16 of 16** closed-loop moves completed, mean error 1.68° on a full
revolution; **8 of 8** after a hard reset on the flash-stored table alone.

Decoupling made no measurable difference, which settles an open question in the
section above. Against the partially-coupled run: hysteresis 30.0 → 31.4
counts, uncorrected peak 124.0 → 123.7, rms 66.1 → 65.4. The load was never
corrupting the sweep. The hysteresis is magnetic detent in the motor, and the
closure-constrained fit was the entire fix — the earlier advice to decouple was
based on a theory the data does not support.

Remaining: completed moves still land ~0.5% short in the positive direction
while the negative direction slightly overshoots, giving a drift of about
0.01 rev per out-and-back cycle. That is proportional-controller steady-state
error against detent torque. It needs an integral term, not more calibration.

## Homing seeks stall intermittently (open)

Homing completed once, cleanly: sought 13 mm, found the sensor, backed off until
it released, parked 1.4 mm clear, `homed = 1`. It is not repeatable. Subsequent
seeks cover a few millimetres and stop at an arbitrary position — −2.96 mm one
run, −7.96 mm another, −5.76 mm a third.

Ruled out by measurement:

| Suspect | Evidence against |
|---|---|
| Drive watchdog | `drive_started` observed incrementing throughout; `home_state` stays `SEEK` |
| Velocity mode itself | jog at 2 rad/s travels 3.70 mm fine |
| Seek current | stalls identically at 800 and 1500 mA |
| The state machine | `home_state`, `home_dir`, `home_vel` all correct during a stalled seek |

What is left is the same intermittent stall measured at roughly 1 in 16 on
point-to-point moves. A seek is one long continuous move, so it meets the
problem nearly every time where a 5 mm positioning move usually does not.

The common factor with every earlier stall is residual commutation error. The
corrected table measures 23.8 electrical degrees peak, which is inside the
±20–30° guideline but not far inside, and that measurement was taken free-shaft
and statically. Under load and in motion the effective error is plausibly worse.

### Two measurement traps hit while chasing this

**Writing commands over SWD with the target halted.** A sequence of `mww`
writes between one `halt` and one `resume` never executes — the firmware sees
only the final value, once. Two separate velocity-mode tests "failed" this way
and both conclusions were wrong. Interleave `resume` between writes.

**`home_state` read after the test script's own cleanup.** The script ends with
`allOff()`, which calls `homeCancel()` and sets the state to idle, so a state
read afterwards reports idle regardless of how the seek ended. It made a stalled
seek look like a cancelled one and sent the first diagnosis down the wrong path.

## Resolved: filter the calibration to low-order harmonics

Homing seeks stalled about half the time with a table built from the raw sweep
residual. The cause was the table correcting something it cannot represent.

Encoder angle error is **low-order** — a few cycles per mechanical revolution,
from magnet eccentricity and tilt. Detent torque is not: it repeats once per
full step, **200x per revolution, every 164 counts**. An open-loop park settles
where detent allows rather than where the field points, so the sweep measures
both.

A 256-bucket table spans 128 counts per bucket and cannot represent a 164-count
period at all. Feeding it the raw residual aliased the detent component into
the table and corrupted the correction everywhere — not just near the detent.
And detent is not an encoder error, so correcting it means fighting the motor's
own magnetic preference.

Fitting 10 harmonics and discarding the rest separates them cleanly:

| component | peak | rms |
|---|---|---|
| total measured | 128.4 elec° | 65.6 |
| encoder, correctable | 119.1 | 65.3 |
| detent + noise, discarded | **17.3** | **6.4** |

That discarded 17.3° is the floor this method can reach, and the corrected
result lands right on it:

| | corrected peak | homing | repeatability |
|---|---|---|---|
| raw residual table | 24.8 elec° | **3 / 6** | — |
| harmonic-filtered | **17.7 elec°** | **6 / 6** | 16 µm spread, 5 µm sd |

After a hard reset, running purely from the flash-stored table: **3/3 homed,
5 µm spread.**

Note the counter-intuitive part: a *finer* 800-point sweep alone did not help
(24.8° vs 23.8° at 400 points). More samples cannot fix a table being asked to
represent a frequency above its own resolution. Filtering did.
