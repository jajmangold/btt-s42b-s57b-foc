#pragma once
#include <Arduino.h>
#include <stdint.h>

// ---------------------------------------------------------------------------
// bxCAN on PA11 (RX) / PA12 (TX), straight to the board's onboard transceiver
// and out on J2 as CANH/CANL. Host side is an ESP32 (native TWAI) plus an
// SN65HVD230.
//
// Programmed at register level deliberately: this board's HAL/library layers
// have already silently ignored one configuration request today (SimpleFOC's
// pwm_frequency), and CAN bit timing is not something to discover is wrong on
// a bus. Everything here is verifiable from the registers.
//
// NOTE: the stock BTT firmware never used CAN -- its entire CAN block sits
// inside `#if 0` -- so none of this hardware path has prior art on this board.
//
// Bit timing, APB1 = 36 MHz, 18 tq/bit (SYNC 1 + TS1 13 + TS2 4):
//   sample point 14/18 = 77.8%, inside the recommended 75-87.5% band.
//   prescaler 2 = 1 Mbit, 4 = 500 kbit, 8 = 250 kbit, 16 = 125 kbit.
// ---------------------------------------------------------------------------

#define CAN_ID_OF(a, t)    (uint16_t)((((a) & 0x7F) << 4) | ((t) & 0x0F))
#define CAN_ADDR_OF(id)    (uint8_t)(((id) >> 4) & 0x7F)
#define CAN_TYPE_OF(id)    (uint8_t)((id) & 0x0F)

// --- addressing ------------------------------------------------------------
// 11-bit standard ID, split 7 bits of address and 4 bits of message type.
//
// Address 0 is the GROUP address: every node acts on a frame sent to it, and
// no node ever transmits from it. One TARGET frame to address 0 therefore
// commands a ganged pair in the same instant, rather than one frame each with
// the second drive a frame-time behind -- about 230 us at 500 kbit/s, which is
// harmless but needless. It is also the clean way to reach every follower with
// a single GEAR_X master broadcast, and gives "all axes off" as one frame.
//
// Group frames win arbitration, as it happens: address 0 puts them at IDs
// 0x000-0x00F, numerically below every node's telemetry, and on CAN the lower
// ID wins. A stop command cannot be delayed by chatter from the drives.
#define CAN_ADDR_BROADCAST 0
#define CAN_NODE_ID_MIN    1
#define CAN_NODE_ID_MAX    16

// Read the DIP switches and latch this node's address. MUST be called after
// a4950_claim_pins(), which performs the SWJ NOJTAG remap that frees PB3 and
// PA15; before that, two of the four switches read as JTAG pins, not inputs.
void    canNodeIdInit();

// This node's address, 1..16. Returns the compile-time default until
// canNodeIdInit() has run, so a caller that gets the order wrong gets the old
// single-drive behaviour rather than a wrong address.
uint8_t canNodeId();

// Raw switch pattern behind the address, for the banner and telemetry.
uint8_t canNodeDipBits();

// host -> node
enum : uint8_t {
  CAN_CMD_ENABLE  = 0x1,   // data[0] = 0/1
  CAN_CMD_TARGET  = 0x2,   // float32 shaft angle, radians
  CAN_CMD_CURRENT = 0x3,   // uint16 mA
  CAN_CMD_PING    = 0x4,   // no payload
  CAN_CMD_JOG     = 0x5,   // float32 rad/s. WATCHDOGGED -- see below.
  CAN_CMD_GEAR_K  = 0x6,   // float32 ratio k; engages gearing at current pos
  CAN_CMD_GEAR_X  = 0x7,   // float32 master position, rad. Broadcast ~1 kHz.
  CAN_CMD_STEPDIR = 0xA,   // uint8 0/1: follow STEP/DIR pulses on P1
};

// Jog is the ONE command where the link sits in the control path, so it is
// deadman-gated: motion continues only while fresh JOG frames keep arriving.
// Stop sending (or drop the link) and the axis ramps to a stop within this
// window. A hardware E-stop still covers everything this cannot.
static const uint32_t JOG_WATCHDOG_MS = 150;
// node -> host
enum : uint8_t {
  CAN_MSG_STATE   = 0x8,   // float32 angle, int16 vel*100, uint8 flags, uint8 seq
  CAN_MSG_PONG    = 0x9,
};

enum : uint8_t {
  CAN_FLAG_FOC_READY = 1 << 0,
  CAN_FLAG_ENERGISED = 1 << 1,
  CAN_FLAG_LINK_FAIL = 1 << 2,
};

struct CanFrame {
  uint16_t id;
  uint8_t  len;
  uint8_t  data[8];
};

class AxisCan {
 public:
  // bitrate_hz: 1000000 / 500000 / 250000 / 125000. loopback keeps the frames
  // internal so the whole path can be proven with no transceiver and no peer.
  bool init(uint32_t bitrate_hz = 500000, bool loopback = false);
  bool send(uint16_t id, const void* data, uint8_t len);
  bool receive(CanFrame* out);          // false if nothing waiting

  uint32_t txCount() const { return tx_; }
  uint32_t rxCount() const { return rx_; }
  uint32_t errCount() const;            // from CAN_ESR
  uint8_t  lastError() const;           // LEC field
  uint32_t btr() const;                 // for verifying timing over SWD

 private:
  uint32_t tx_ = 0, rx_ = 0;
};
