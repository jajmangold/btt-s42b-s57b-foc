# Contributing

Issues and pull requests are welcome.

## The one rule that matters

**Measure it, or say you didn't.**

This project's value is that its numbers are real and their provenance is
recorded. If you add a claim, add how you established it. If something is
derived, inferred, or copied from a datasheet rather than observed on hardware,
label it as such — `docs/measurements.md` has a "Known unverified" section
precisely so that uncertainty has somewhere honest to live.

A pull request that says "this reduced following error from 136 to 40 counts,
here is the test" is worth far more than one that says "improved tuning".

## Especially wanted

- **S42B V2.0 test reports.** Support for that board is derived from
  BIGTREETECH's own sources showing identical silicon, but it has never been
  run. A report either way is genuinely useful — open an issue even if it just
  works.
- **A coil-current measurement.** The current scale is the last derived-but-
  unmeasured number in the system. A meter in series with one phase at a known
  commanded current would close it.
- **P1 isolation confirmation.** Drive PA2 high and low and measure the EN pin
  at the connector. If the voltage follows, the inputs are not isolated and
  bidirectional use of that connector becomes possible.

## Building

```
pio run -e s57b        # or -e s42b
pio run                # both
```

Both environments must build before a pull request is merged; CI enforces it.

## Hardware changes

If you change anything that touches the motor:

1. Verify the coil probe still shows both phases driving in both polarities,
   roughly 328 counts apart, and 90 degrees apart from each other
2. Verify closed-loop hold is still in the low single-digit counts
3. Say what current you tested at — a number taken at 600 mA on a free shaft
   is not a claim about behaviour under load

## Safety

Changes that could increase commanded current, remove a limit, or alter a
watchdog need an explicit note in the pull request explaining why that is safe.
The deadman on jog and the clamp on CAN current commands exist because a value
arriving over a wire should not get to set motor current unchecked.

## Style

Match the surrounding code. Comments should explain *why*, especially where
the code works around something non-obvious — most of the awkward parts of this
firmware exist for a documented hardware reason, and a future reader needs that
reason more than they need a restatement of the syntax.
