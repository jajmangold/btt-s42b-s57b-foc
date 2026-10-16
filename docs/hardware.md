# The board, reverse-engineered

Everything here was established from BIGTREETECH's own published firmware
sources and manuals, or measured directly. Where a claim is inference, it is
labelled as such.

## MCU and peripherals

| | |
|---|---|
| MCU | STM32F103CBT6, Cortex-M3, 72 MHz, **128 KB flash** |
| Motor driver | 2 × Allegro A4950 full-bridge |
| Encoder | Infineon TLE5012B, 15-bit magnetic |

**On the flash size.** BIGTREETECH's repository README says `STM32F103C8T6`
(64 KB). That is wrong, and four independent checks agree it is a **CB** part:

1. The `F_SIZE` register at `0x1FFFF7E0` reads **128**
2. OpenOCD's own probe reports `flash size = 128 KiB`
3. The factory firmware writes to `0x08018000` — 96 KB in — which a true 64 KB
   part does not have
4. Both the S42B V2.0 and S57B V2.0 *manuals* say `STM32F103CBT6`

## Pin map

### Encoder — SPI1, 3-wire half-duplex SSC

| Pin | Function |
|---|---|
| PA5 | SCK |
| PA6 | MISO |
| PA7 | MOSI |
| PA4 | CS |

MOSI and MISO are **joined on the PCB**. The master must tri-state MOSI after
sending the command word or it fights the sensor for the shared line. On
STM32F1 that is `GPIOA->CRL` bits 31:28 — `0x4` for floating input, `0xB` for
AF push-pull.

Bus is **SPI mode 1** (CPOL=0, CPHA=1), 16-bit frames, MSB first, software NSS.
Command words: `STAT 0x8001`, `AVAL 0x8021`, `ASPD 0x8031`.

The factory firmware clocks this at 9 MHz; the datasheet maximum is 8 MHz.
This firmware uses 4 MHz.

### Motor — 2 PWM + 4 direction

| Pin | Function |
|---|---|
| PB4 | `TIM3_CH1` PWM → coil A VREF |
| PB8 / PB9 | coil A direction |
| PB5 | `TIM3_CH2` PWM → coil B VREF |
| PB6 / PB7 | coil B direction |

The PWM pin does **not** drive the winding. It feeds the A4950's VREF through
an RC filter, so duty is a **current setpoint** and the direction pins set
polarity. This is why `StepperDriver2PWM` is the right model and
`StepperDriver4PWM` is not.

`TIM3` uses **partial** remap, which places CH1 on PB4 and CH2 on PB5.

**PB4 is `NJTRST`.** It is a JTAG pin out of reset and cannot be used as GPIO or
PWM until `SWJ_CFG` selects "JTAG disabled, SW-DP enabled" (`0b010`). Without
that, coil A is simply dead. Never select `0b100` (both disabled) — that kills
SW-DP and with it your only way back into the chip.

**Write `AFIO_MAPR` once, explicitly. Never read-modify-write it.** Two
separate hazards make the obvious approach unsafe:

1. `SWJ_CFG` is **write-only** on STM32F1, so a read-modify-write cannot
   preserve it reliably.
2. ST's own `AFIO_REMAP_PARTIAL()` macro — which `__HAL_AFIO_REMAP_TIM3_PARTIAL()`
   expands to — does `tmpreg |= AFIO_MAPR_SWJ_CFG`, ORing in **all three**
   `SWJ_CFG` bits. That is `0b111`: JTAG *and* SWD disabled.

So calling the TIM3 remap macro after setting NOJTAG attacks the exact field
PB4 depends on. In practice `0b111` is reserved and appears to be ignored by
the silicon, which is why this does not bite immediately — but it is luck, not
design. This firmware sets both fields in a single store:

```c
AFIO->MAPR = (0x2u << AFIO_MAPR_SWJ_CFG_Pos)      // JTAG off, SW-DP alive
           | (0x2u << AFIO_MAPR_TIM3_REMAP_Pos);  // partial: CH1=PB4, CH2=PB5
```

Verify with a debugger: `AFIO_MAPR` should read `0x02000800`.

### Host interface — P1

| P1 pin | MCU | Function |
|---|---|---|
| CLK/STEP | PA0 | `TIM2` ETR external clock — pulses counted in **hardware** |
| DIR | PA1 | `EXTI1`, both edges; handler flips `TIM2` count direction |
| EN | PA2 | read as a level |
| GND, COM, VIM | — | ground, input common (5 V/3.3 V selectable), 12–24 V motor supply |

This resolves an ambiguity that earlier ports could not: STEP and DIR appear to
be the same pin in the factory sources because **DIR owns the interrupt while
STEP owns the hardware clock**, so both show up in one code path.

All three inputs are configured input-with-pull-up, which is the signature of an
**open-collector optocoupler** output. Together with P1 having a separate `COM`
pin alongside GND, that strongly suggests the inputs are isolated and therefore
**receive-only** — the MCU could not drive the connector even if asked.
*Inference, not confirmed with a meter.*

### DIP switches

Four switches, read by the factory firmware in `SetModeCheck()`. Their factory
meanings are gone here — microstep is meaningless under FOC and open/closed
loop is not a choice — so **this firmware uses them as the CAN node address**,
`switches + 1`, giving 1..16. See `docs/protocol.md`. Two of the pins also
matter for a second reason, below.

| Pin | DIP | Factory meaning |
|---|---|---|
| PB10 | 1 | microstep select, with DIP 2 |
| PB11 | 2 | microstep select, with DIP 1 |
| PB3 | 3 | low = closed loop |
| PA15 | 4 | low = run encoder calibration at boot |

All four are input-with-pull-up in the factory firmware, so a switch that is ON
reads 0. SET1/SET2 map to microstep as: both high = 16, (0,1) = 8, (1,0) = 4,
both low = 32.

**PA15 is JTDI and PB3 is JTDO.** Like PB4, they belong to the JTAG peripheral
out of reset and are only released once `SWJ_CFG` is set. Since DIP 3 shorts
PB3 to ground when ON, leaving it as a JTAG pin longer than necessary is worth
avoiding — another reason the `AFIO_MAPR` write happens before anything else.

You can read the switch positions without a firmware change by configuring the
pins as input-with-pull over SWD and sampling under both pull directions: a pin
that reads 0 under a pull-up is driven low, i.e. switch ON.

### Home / limit sensor

| Pin | Function |
|---|---|
| PB15 | hall-effect sensor input, internal pull-up, active low |

PB15 was chosen for two reasons. It is **5 V tolerant** — unlike `PA0`–`PA7`
and `PB0`–`PB1`, which are the ADC-capable pins and are not — so the usual 5 V
hall module cannot over-volt it. And nothing else in this firmware uses it.

Measured on hardware with a 3-wire hall module: **1 with no magnet, 0 with one
present**, each stable across six consecutive samples. That is the standard
open-collector-plus-pull-up output, so the idle high comes from the module's
own pull-up; the internal pull-up is enabled as well so the pin still has a
defined level if the module is unplugged.

`DS` reports the sensor without moving anything. `DH<±v>` runs a homing seek,
which **does** move the axis — see `docs/flashing.md` before using it on a
ballscrew.

### CAN and debug

| Pin | Function |
|---|---|
| PA11 / PA12 | bxCAN RX/TX → onboard transceiver → J2 as CANH/CANL |
| PA9 / PA10 | UART1 TX/RX on header P7 |
| PA13 / PA14 | SWDIO / SWCLK on header P7 |
| PC13 | LED |
| PA3, PB0, PB1 | buttons |

The boards have a CAN transceiver onboard, so **no external transceiver module
is needed at the drive end**. The CAN hardware is present but the factory
firmware never used it — its entire CAN block sits inside `#if 0`.

## Current scaling

The A4950 limits current by `Itrip = VREF / (10 × Rsense)`. VREF comes from PWM
through an RC filter and **cannot exceed the 3.3 V rail**, so 100% duty is the
board maximum by construction:

| Board | Rsense | VREF max | Max current |
|---|---|---|---|
| S42B V2.0 | 0.2 Ω | 3.3 V | 1.65 A |
| S57B V2.0 | 0.1 Ω | 3.3 V | 3.3 A |

Both match the published manuals exactly, which is why the derivation is
trusted. It is still a *derivation* — see [measurements.md](measurements.md).

A useful safety consequence: because VREF is rail-limited, **the current cannot
exceed the board's rating even if the scale is wrong**. Getting the board
selection wrong is an accuracy error, not a destructive one.

## Factory firmware flash map

| Range | Contents |
|---|---|
| `0x08000000`–`0x080086EB` | firmware, 33.7 KB |
| `0x08017C00`–`0x08017C1B` | settings, 28 bytes |
| `0x08018000`–`0x0801FFFF` | **encoder calibration, 32 KB** |

The settings block is 14 little-endian halfwords: marker `0xAACC`, current,
microstep, direction, enable mode, and PID gains.

The 32 KB calibration table is generated by the factory calibration routine, is
specific to **your** motor, and is not in any published binary. Back it up.
