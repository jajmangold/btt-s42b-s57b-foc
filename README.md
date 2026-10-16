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
| Closed-loop hold | 2–3 counts (0.022–0.033°) |
| Commanded revolution | within 10 counts of 360° (0.11°) |
| Pole pairs | 50, confirmed two independent ways |
| PWM carrier | 140.1 kHz |
| CAN | 500 kbit/s exact, 77.8% sample point, zero errors |
| Electronic gearing | 90–136 counts following error at 2 rad/s |
| Full power | validated to 3300 mA (100% duty), sustained, no degradation |
| STEP/DIR | one commanded revolution within 12 counts of 360° |
| UART | full command set verified, current clamped to board maximum |

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

**The clock is wrong by default.** The core's generic STM32F103 configuration
runs the PLL from the internal RC at 64 MHz and never enables the crystal. CAN
then runs at 444 kbit/s while believing it is at 500, and UART at 43191 baud
while believing it is at 38400. Loopback cannot catch it — both ends share the
error — so it would first appear as "CAN doesn't work" against a real peer.
This firmware configures HSE x9 = 72 MHz and derives every rate from the clock
actually achieved.

**The units lie.** `StepperDriver` treats duty as a winding *voltage*. On this
board duty sets a current *limit* via VREF. Declaring
`voltage_power_supply` as the full-duty current in amps makes SimpleFOC's
"volts" numerically equal to amps, so every `voltage_limit` reads directly as a
current limit.

---

## Quick start

```bash
git clone https://github.com/jajmangold/btt-s42b-s57b-foc
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
| **STEP / DIR / EN** | P1 | Drop-in replacement for stock. Works with FluidNC, LinuxCNC, GRBL, anything. Pulses counted in **hardware**. |
| **CAN** | J2 | Digital control plus telemetry. 500 kbit/s. |
| **UART** | P7 | Configuration and diagnostics. 38400 8N1. |

CAN is where this firmware earns its keep: over STEP/DIR a drive is a black box
with one alarm wire, whereas CAN reports actual position, following error, and
the specific reason for a fault. Protocol in [docs/protocol.md](docs/protocol.md).

### UART

38400 8N1 on header P7. SimpleFOC's `Commander` provides the standard motor
verbs under `M`, so existing SimpleFOC tooling and anyone's muscle memory work
unchanged. `D` adds this firmware's drive modes:

```
D?          help, plus live status
D5          align (find the electrical zero)
D1          hold position
D2          follow STEP/DIR from P1
D3 <k>      engage electronic gearing at ratio k
D4 <v>      jog at v rad/s
DC <mA>     set current limit, clamped to the board maximum
M...        full SimpleFOC motor verbs: PID gains, limits, target, monitoring
```

Current values arriving over the wire are clamped to the board maximum, and
`Commander` cannot be used to exceed it in any case — duty is
`|U| / voltage_power_supply`, and `voltage_power_supply` *is* the full-duty
current, so the hardware bound holds whatever anyone types.

### STEP / DIR

Step pulses are counted by **TIM2 in hardware** (PA0 is the timer's external
clock input), so the step rate costs no CPU and cannot be missed by a busy
control loop. Direction changes drive an interrupt that flips the counter
between up and down, so a direction change takes effect immediately rather
than at the next poll.

Set `STEP_DIR_STEPS_PER_REV` to match the microstepping your controller is
configured for — it is the only number that has to agree between host and
drive. Under FOC, microstepping is a host-side fiction rather than something
the drive does.

`ENABLE` is ignored unless you define `STEP_DIR_USE_ENABLE`. That is
deliberate: the polarity of that input has not been confirmed on hardware, and
guessing it wrong in the permissive direction would energise a motor the host
believes is released.

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

**`tools/telem.py`** reads live state out of the running firmware over SWD
without halting it, so you can watch position, following error and fault
counters with no serial cable attached.

**`tools/esp32-bridge/`** turns any ESP32 dev board into a USB-to-serial
adapter for the drive's P7 console — useful when you have one to hand and no
dedicated USB-TTL cable. Three jumpers, and the drive's command set is then
available from any terminal. The same board has a TWAI controller, so this is
also a reasonable starting point for a CAN host.

**`tools/pendant.py`** turns any game controller into a jog pendant. It sends
the drive's own `D4 <v>` commands, so it is not a privileged channel — just a
convenient way to drive the same interface you can type into. Inside the stick
deadzone it sends nothing rather than a zero, so every return to centre
re-proves the deadman watchdog rather than exercising it only on failure.

**`tools/swd.py`** is a bit-banged SWD master for MicroPython, kept as a
fallback for anyone without a debug probe.

## Examples

**`examples/esp32-can-host/`** is a complete, readable implementation of the
CAN protocol for an ESP32 — send commands, decode the 50 Hz telemetry
broadcast, and drive an axis from a serial console. It relays the drive's UART
console on separate pins too, so one board covers both paths during bring-up.

The ESP32's TWAI controller speaks plain CAN 2.0, so no external CAN
controller is needed — only a 3.3 V transceiver such as an SN65HVD230. The
drive already has one onboard behind J2.

It also joins Wi-Fi and serves a live status page — axis angle, velocity,
ready/live/fault flags and CAN error counters — plus `/status` as JSON for
scripting. Copy `src/secrets.example.h` to `src/secrets.h` and fill in your
network; `secrets.h` is gitignored and must stay that way.

Send `t` on its console to self-test with **no transceiver and no peer**: it
routes TX to RX through the GPIO matrix and uses self-reception, which proves
bit timing, filters and protocol encoding before any wiring exists.

## Upstream

Work from this project that belongs in its dependencies is sent there rather
than kept here:

- **[simplefoc/Arduino-FOC-drivers#106](https://github.com/simplefoc/Arduino-FOC-drivers/pull/106)** —
  safety-word and CRC validation for SimpleFOC's own TLE5012B driver. That
  driver already clocks the safety word off the wire and discards it, so
  checking it costs nothing; the CRC algorithm contributed there is the one
  determined by measurement in [docs/measurements.md](docs/measurements.md).

## Contributing

Issues and pull requests welcome — particularly S42B test reports, since that
path is currently unverified. See [CONTRIBUTING.md](CONTRIBUTING.md).

## Licence

MIT. Builds on [SimpleFOC](https://github.com/simplefoc/Arduino-FOC) (MIT).
Hardware details were established from BIGTREETECH's own published sources and
from direct measurement.
