# CAN protocol

500 kbit/s by default. The drive has an onboard transceiver, so J2 CANH/CANL go
straight to the bus — **no transceiver module is needed at the drive end**.

## Bit timing

APB1 runs at 36 MHz. The timing is **18 tq/bit** (SYNC 1 + TS1 13 + TS2 4),
giving a **77.8% sample point**, inside the recommended 75–87.5% band. The
prescaler selects the rate:

| Rate | Prescaler |
|---|---|
| 1 Mbit/s | 2 |
| **500 kbit/s** | **4** |
| 250 kbit/s | 8 |
| 125 kbit/s | 16 |

Because 36 MHz divides exactly, 500 kbit/s is achieved with **no rounding
error**.

## Addressing

11-bit standard identifiers:

```
ID = (node_address << 4) | message_type
```

7 bits of address, 4 bits of message type. A single accept-all filter is
installed and addressing is done in software, so additional axes need no
filter-bank changes.

### Node address

Taken from the four DIP switches at boot, as `switches + 1`, giving **1..16**.
The switches are input-with-pull-up so ON reads 0, and the address inverts
that: ON = 1. A board with every switch OFF is node 1.

Build with `-D CAN_NODE_ID_FIXED=n` to ignore the switches.

The switches are read immediately after the `SWJ_CFG` NOJTAG remap, and cannot
be read before it: DIP 3 is PB3 (JTDO) and DIP 4 is PA15 (JTDI), which belong
to the JTAG peripheral out of reset and read as such whatever `pinMode()` is
asked for.

### Group address

**Address 0 is a group address.** Every node acts on a frame sent to it, and no
node ever transmits from it — replies and telemetry always carry the node's own
address, so two drives can never answer as one.

| Use | Why it needs the group address |
|---|---|
| Ganged axis | One `TARGET` commands both drives in the same instant, instead of one frame each leaving the second ~230 µs behind at 500 kbit/s |
| Gearing | One `GEAR_X` master broadcast reaches every follower |
| Stop | "All axes off" as a single frame |
| Enumeration | A group `PING` makes every node reply with its own address |

Group frames occupy IDs `0x000`–`0x00F`, numerically below every node's
telemetry. Since the lower ID wins CAN arbitration, a stop command cannot be
delayed behind drive chatter.

## Host to drive

| Type | Name | Payload |
|---|---|---|
| `0x1` | `ENABLE` | `uint8` 0 or 1 |
| `0x2` | `TARGET` | `float32` shaft angle, rad |
| `0x3` | `CURRENT` | `uint16` mA |
| `0x4` | `PING` | none |
| `0x5` | `JOG` | `float32` rad/s — **watchdogged** |
| `0x6` | `GEAR_K` | `float32` ratio; engages gearing at current position |
| `0x7` | `GEAR_X` | `float32` master position, rad |

`CURRENT` is **clamped to the board maximum on arrival**. A value that came off
a wire does not get to set motor current unchecked.

## Drive to host

| Type | Name | Payload |
|---|---|---|
| `0x8` | `STATE` | `float32` angle, `int16` velocity x100, `uint8` flags, `uint8` seq |
| `0x9` | `PONG` | 4-byte board tag |

`STATE` is broadcast at 50 Hz. Flags: bit 0 FOC ready, bit 1 energised, bit 2
encoder link fault.

This is the half a STEP/DIR link cannot give you. Over step pulses a drive is a
black box with one alarm wire; here you get actual position, following error
and the specific reason for a fault.

## The jog watchdog

`JOG` is the one command where the link sits in the control path, so it is
deadman-gated. Motion continues **only while fresh `JOG` frames arrive**. After
`JOG_WATCHDOG_MS` (150 ms) without one, velocity is zeroed and a counter
increments — so a flaky link shows up as a number rather than as mystery
stutter. Send at 20–50 Hz.

This is not a substitute for a hardware E-stop. The watchdog handles dropouts;
an E-stop handles everything else.

## Electronic gearing

`GEAR_K` engages, capturing the current follower and master positions as the
zero point. `GEAR_X` then feeds master positions and the follower servos to:

```
target = shaft_at_engage + k * (master - master_at_engage)
```

The follower tracks the master **measured position**, never a time-based
profile. A late update makes it trail and catch up while the *ratio* stays
exact — and for a helix or a synchronised gantry, the ratio is what sets the
geometry.

It is also inherently fail-safe: no fresh master position means the target
stops advancing, so the follower holds rather than running away.

Broadcast master position at around 1 kHz. That is one frame per cycle, roughly
120 kbit/s, or 24% of a 500 kbit/s bus.

## A note on bandwidth

Do not stream cyclic position setpoints to every axis at 1 kHz. Four axes
bidirectional is about 960 kbit/s and will not fit — it saturates 500 kbit/s
and leaves no arbitration headroom even at 1 Mbit/s.

It is also unnecessary. These drives close their own position loop at several
kHz. Send *moves*, or gear one axis to another, and the bus is nearly idle. The
streaming model exists for dumb follower drives; this is not one.

## Homing over SWD

Command 21 runs the homing seek, taking its direction from the sign of
`g_telem.nudge_rev`. Added because homing otherwise existed only as the UART
`DH` command, which is unavailable on a board reached only through SWD.

Verified on the ballscrew axis: sought 13 mm, found the sensor, backed off
until it released, and parked 1.4 mm clear with `homed = 1` and the trip point
recorded at −13.487 mm.
