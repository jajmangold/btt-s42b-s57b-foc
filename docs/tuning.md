# Tuning

## Start low

`BRINGUP_mA` defaults to half the board factory default — 800 mA on an S57B,
400 mA on an S42B. Raise it deliberately, after motion works, not as a first
reflex.

The current scale is **derived, not measured** (see
[measurements.md](measurements.md)). It is bounded, because VREF cannot exceed
the 3.3 V rail, so you cannot exceed the board rating even if the scale is off.
If you want certainty, put a meter in series with one phase while holding a
fixed electrical angle. The winding inductance smooths the current, so a DC
ammeter reads the regulated value accurately — more accurately than inferring
it from the chopped voltage across the coil.

## If the motor will not move

The most common cause is **not enough current**, not a broken install. Detent
torque varies a lot between motors, and `BRINGUP_mA` is deliberately
conservative — on some motors it is below what is needed to overcome detent
from rest.

Diagnose it with the coil probe (`cmd 3`), which energises each coil in each
polarity and reports where the rotor parks:

| Reading | Meaning |
|---|---|
| Both coils ~328 counts between polarities | healthy |
| One coil near 0 counts | that phase is open — check wiring and crimps |
| Both coils well under 328 | not enough current; raise and retry |

A real example from development: at 800 mA a motor gave 245 and -10 counts
(looks like a dead phase); the same motor at 1500 mA gave 322 and -321
(perfectly healthy). The same motor had worked at 600 mA a few hours earlier,
which is how we knew to suspect mechanical load rather than the firmware.

**Raise current before concluding anything is broken**, then investigate why
the motor needs more than it did.

## Position loop

Defaults, which are a reasonable starting point rather than an optimum:

```cpp
motor.P_angle.P       = 8.0f;
motor.PID_velocity.P  = 0.05f;
motor.PID_velocity.I  = 1.0f;
motor.PID_velocity.D  = 0.0f;
motor.LPF_velocity.Tf = 0.02f;
```

**On the velocity filter.** Six counts of encoder jitter differentiated at
14 kHz produces roughly ±1.4 rad/s of phantom velocity on a stationary shaft.
Without filtering, the velocity loop chases noise.

**On `P_angle`.** Raising it tightens hold, but it will not fix tracking of a
*moving* target — a proportional position loop lags a ramp by `v / P` by
construction. Chasing that with gain alone buys instability, not accuracy.
That is what feedforward is for.

## Gearing

```cpp
static const float GEAR_KP = 25.0f;   // rad/s of correction per rad of error
```

The feedforward term carries the motion and `GEAR_KP` only trims, so it does
not need to be large. Raising it amplifies encoder noise into the velocity
command.

Master velocity is differentiated from successive positions and filtered with
alpha 0.15. A raw difference over a 1 ms window is far too noisy to feed
forward. If your master broadcasts at a different rate, that filter constant is
the first thing to revisit.

The startup transient — roughly 490 counts — is the velocity estimator
converging from zero at engage. If you need to engage *inside* a cut rather
than in air, seed it from a known master velocity.

## What good looks like

Measured on an S57B at 600–800 mA with a free shaft:

| | |
|---|---|
| Hold | 2–3 counts |
| Revolution accuracy | within 10 counts of 360 degrees |
| Gearing at 2 rad/s | 43–136 counts following error |

All of these were taken with **no load**. Expect to revisit the gains once
there is something to push against — in particular `PID_velocity.I`, which is
what rejects a steady disturbance.

## Current and torque

Torque is roughly proportional to current, so a drive at 25% of rated current
has roughly 25% of rated torque. Two consequences worth planning around:

- Bring-up numbers above were taken at well under half current. They are not
  a prediction of behaviour under load.
- The driver, not the motor, is usually the binding limit. A 4 A motor on an
  S57B is capped at 3.3 A; the same motor on an S42B is capped at 1.65 A.
