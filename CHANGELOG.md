# Changelog

All notable changes to this project are documented here.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- **STEP/DIR/EN input on P1.** Pulses are counted by TIM2 in hardware via its
  external clock input, so step rate costs no CPU and cannot be missed by a
  busy control loop. Direction changes drive an interrupt that flips the
  counter, taking effect immediately. This is the drop-in path for FluidNC,
  LinuxCNC, GRBL or any standard breakout.

### Planned

- Physical pulse input validation (the counter and mapping are proven, the
  optocoupler path is not)
- Physical CAN bus validation against a second node (only loopback is proven)
- Coil current measured with a meter rather than derived
- S42B V2.0 validation on real hardware

## [0.1.0] - 2026-10-15

First release. Closed-loop FOC, CAN, and electronic gearing all verified on an
S57B V2.0.

### Added

- SimpleFOC-based closed-loop position and velocity control for the
  BIGTREETECH S42B V2.0 and S57B V2.0
- `TLE5012B` sensor driver over 3-wire half-duplex SSC, with **safety-word and
  CRC validation** — the CRC algorithm was determined experimentally rather
  than assumed (200/200 against three variants at 0/200)
- `S57BDriver`, a `StepperDriver` implementation that owns `TIM3` directly and
  runs the PWM carrier at 140.1 kHz
- `S57BCan`, register-level bxCAN at 500 kbit/s with a documented axis protocol
- Electronic gearing with velocity feedforward, so a follower can track a
  master position at a fixed ratio
- Velocity jog mode with a 150 ms deadman watchdog
- `tools/telem.py`, which reads live state out of RAM over SWD without halting
  the target
- `tools/swd.py`, a bit-banged SWD master for MicroPython, as a fallback for
  anyone without a probe
- Dual board support with a compile-time selector, because the S42B and S57B
  differ 2:1 in current scale

### Fixed

Relative to the stock firmware and to common practice on these boards:

- **PWM carrier.** `driver.pwm_frequency` has no effect on STM32F1; `TIM3` ends
  up at 1.12 kHz regardless. That is audible and starves the A4950 current
  regulation. Owning the timer directly cut hold hunting from 13 counts to 2,
  and took a commanded revolution from stalling 0.30 rad short to repeating
  within 0.0015 rad.
- **Unit mismatch.** `StepperDriver` treats duty as a winding voltage, but on
  this hardware duty sets a current limit. Declaring `voltage_power_supply` as
  the full-duty current in amps makes every `voltage_limit` read directly as a
  current limit.
- **Encoder corruption went undetected.** The stock firmware never reads the
  TLE5012B safety word, so a corrupted angle silently commands current.
- **Ramp tracking.** SimpleFOC has no velocity feedforward, so its angle
  cascade lags a moving target by `v / P_angle` by construction. The gearing
  law supplies feedforward, improving steady-state following error 33x.
- **PB4 is `NJTRST`** and must be released by the SWJ NOJTAG remap before it
  can drive PWM. Without it, coil A is dead.

### Known limitations

- Coil current scale is derived from two agreeing calculations, not measured
- S42B V2.0 support is derived from identical silicon but untested
- P1 optocoupler isolation is inferred, not confirmed electrically
- CAN is proven in loopback only; the physical bus is untested
- All results were taken with no load on the motor

[Unreleased]: https://github.com/jajmangold/btt-s42b-s57b-foc/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/jajmangold/btt-s42b-s57b-foc/releases/tag/v0.1.0
