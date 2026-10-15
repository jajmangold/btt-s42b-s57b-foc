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

Two causes, and they look identical from the outside. Distinguish them with the
coil probe (`cmd 3`), which energises each coil in each polarity and reports
where the rotor parks:

| Reading | Meaning |
|---|---|
| Both coils ~328 counts between polarities | healthy |
| Both coils well under 328, consistently | not enough current; raise and retry |
| One or both weak **sometimes** | intermittent connection — see below |

### Not enough current

Detent torque varies between motors and `BRINGUP_mA` is deliberately
conservative. Sweeping current on a development motor put the threshold between
400 and 600 mA:

| | coil A | coil B |
|---|---|---|
| 400 mA | +13 | -315 |
| 600 mA | +325 | -321 |
| 1500 mA | +330 | -326 |

Note the asymmetry at 400 mA — one coil failing while the other looks fine does
**not** prove that phase is broken. It can simply be the one fighting detent
from a worse starting position.

### Intermittent connection

If the probe passes, then fails, then passes again at the same current, suspect
a crimp rather than the firmware. During development a motor read `A 245 /
B -10` at 800 mA, and twenty minutes later the same board at the same current
read `A 327 / B -323`, with a full current sweep showing 800 mA comfortably
above threshold. Nothing had been touched.

This is worth taking seriously because it mimics every other fault: it looks
like low torque, a dead phase, a tuning problem, or a firmware regression
depending on when you happen to test. **Run the probe three times before
concluding anything**, and re-seat the motor connector before changing code.

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
