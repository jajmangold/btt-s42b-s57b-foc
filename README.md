# Open FOC firmware for the BIGTREETECH S42B / S57B V2.0

Replacement firmware for BIGTREETECH's discontinued closed-loop stepper driver
boards, built on [SimpleFOC](https://github.com/simplefoc/Arduino-FOC). It turns
them into properly controllable servo drives with **real telemetry** — position,
following error, and encoder fault detection — over STEP/DIR, CAN, or UART.

These boards are end-of-life and the stock firmware is a black box: you get a
step input and a single alarm wire, with no way to ask *why* it faulted. The
hardware is good. This makes it usable.

**Every number in this repository was measured on hardware, and the method is
recorded.** Where something is derived rather than measured, it says so.

---

## ⚠️ Read before flashing

This firmware drives a motor at up to 3.3 A from a 12–24 V supply. Mistakes
can destroy the driver, damage a machine, or injure someone.

- **Back up the factory firmware first.** Per-motor encoder calibration lives
  at `0x08018000` — **above 64 KB** — and is *not* present in any published
  BIGTREETECH binary. A 64 KB dump silently misses it. See
  [docs/flashing.md](docs/flashing.md).
- **Select the correct board.** The S42B and S57B differ 2:1 in current scale.
  Choosing wrong commands double the current you intended.
- **Flashing is one-way without a probe.** There is no USB bootloader; SWD is
  the only way in, and the only way back.
- Provided as-is under the MIT licence, with no warranty. See [LICENSE](LICENSE).

---

## Supported hardware

| | S42B V2.0 | S57B V2.0 |
|---|---|---|
| MCU | STM32F103CBT6, Cortex-M3 72 MHz | same |
| Driver | 2 × A4950 | same |
| Encoder | TLE5012B, 3-wire half-duplex SSC | same |
| Maximum current | 1650 mA | 3300 mA |
| Current sense | 0.2 Ω | 0.1 Ω |
| PlatformIO env | `s42b` | `s57b` |

Both boards run the same factory firmware and are electrically the same design.
They differ only in the current-sense resistor — hence the 2:1 current scale,
and hence the board selection.

> **Testing status.** All hardware results below were measured on an **S57B
> V2.0**. S42B support is derived from BIGTREETECH's own sources and manuals,
> which document identical silicon, but has **not been run on an S42B**. If you
> try it, please open an issue either way.

The **V1.0** boards are *not* supported — they use an STM32F030 (Cortex-M0) and
a different toolchain.

---

## Verified on hardware

| Measurement | Result |
|---|---|
| Encoder sample rate | 14.4 kHz (13.3 kHz with CRC validation) |
| Encoder jitter, stationary | 6 counts of 32768 (0.066°) |
| Closed-loop hold | 2 counts (0.022°) |
| Commanded revolution | within 10 counts of 360° (0.11°) |
| Pole pairs | 50, confirmed two independent ways |
| PWM carrier | 140.1 kHz |
| CAN | 500 kbit/s exact, 77.8% sample point, zero errors |
| Electronic gearing | 43–136 counts following error at 2 rad/s |
| Full power | validated to 3300 mA (100% duty), sustained, no degradation |

See [docs/measurements.md](docs/measurements.md) for how each was obtained.

---

## Why not just use SimpleFOC's stock driver?

Two things on this hardware defeat the library's defaults, and both are fixed
here:

**The PWM frequency setting is ignored.** `driver.pwm_frequency` has no effect
on STM32F1 — built at 25 kHz and at 100 kHz, `TIM3` came out at `ARR=63999`
both times, i.e. **1.1 kHz**. That is audible, and far below what the board's
VREF filter needs, so the A4950's current regulation is starved. This firmware
owns `TIM3` directly at 140.1 kHz. Fixing it reduced hold hunting from 13 counts
to 2, and took a commanded revolution from stalling 0.30 rad short to repeating
within 0.0015 rad.

**The units lie.** `StepperDriver` treats duty as a winding *voltage*. On this
board duty sets a current *limit* via VREF. Declaring
`voltage_power_supply` as the full-duty current in amps makes SimpleFOC's
"volts" numerically equal to amps, so every `voltage_limit` reads directly as a
current limit.

---

## Quick start

```bash
git clone https://github.com/<you>/btt-s42b-s57b-foc
cd btt-s42b-s57b-foc

# 1. BACK UP THE FACTORY FIRMWARE -- do not skip this
#    see docs/flashing.md for the full 128 KB dump procedure

# 2. Build for your board
pio run -e s57b          # or -e s42b

# 3. Flash over SWD
pio run -e s57b -t upload
```

Wiring an ST-Link: `SWCLK → SWCK`, `SWDIO → SWDO`, `GND → GND` on header P7.
Leave the probe's 3.3 V disconnected and power the board normally from VIM.
Full detail, including clone-probe quirks, in [docs/flashing.md](docs/flashing.md).

---

## Control interfaces

| Interface | Connector | Use |
|---|---|---|
| **STEP / DIR / EN** | P1 | Drop-in replacement for stock. Works with FluidNC, LinuxCNC, GRBL, anything. |
| **CAN** | J2 | Digital control plus telemetry. 500 kbit/s. |
| **UART** | P7 | Configuration and diagnostics. 38400 8N1. |

CAN is where this firmware earns its keep: over STEP/DIR a drive is a black box
with one alarm wire, whereas CAN reports actual position, following error, and
the specific reason for a fault. Protocol in [docs/protocol.md](docs/protocol.md).

### Electronic gearing

One drive can follow another's position at a fixed ratio
(`follower = k × master`), which is how you cut a helix or run a gantry in
sync. The follower tracks the master's **measured position**, never a
time-based profile — so a late update makes it trail and catch up while the
*ratio* stays exact. Timing jitter cannot corrupt the geometry.

---

## Documentation

- [docs/hardware.md](docs/hardware.md) — the reverse-engineered board: pin map, registers, how each was established
- [docs/flashing.md](docs/flashing.md) — backup, flashing, recovery, probe quirks
- [docs/protocol.md](docs/protocol.md) — CAN protocol specification
- [docs/measurements.md](docs/measurements.md) — every number and how it was measured
- [docs/tuning.md](docs/tuning.md) — gains, current limits, gearing

## Tools

`tools/telem.py` reads live state out of the running firmware over SWD without
halting it, so you can watch position and following error with no serial cable
attached. `tools/swd.py` is a bit-banged SWD master for MicroPython, kept as a
fallback for anyone without a probe.

## Contributing

Issues and pull requests welcome — particularly S42B test reports, since that
path is currently unverified. See [CONTRIBUTING.md](CONTRIBUTING.md).

## Licence

MIT. Builds on [SimpleFOC](https://github.com/simplefoc/Arduino-FOC) (MIT).
Hardware details were established from BIGTREETECH's own published sources and
from direct measurement.
