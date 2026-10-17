#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>
#include "config.h"

// ---------------------------------------------------------------------------
// Infineon TLE5012B angle sensor, wired on the S42B / S57B V2.0 as 3-wire half-duplex
// SSC: MOSI (PA7) and MISO (PA6) are joined on the PCB. The master must
// TRI-STATE MOSI after sending the command word, or it fights the sensor for
// the shared data line.
//
// Transaction (verified against BTT's own ReadValue(), which runs on this
// exact board):
//     CS low -> send 16-bit command -> tri-state MOSI
//            -> clock 16 dummy bits -> read data -> CS high -> restore MOSI
//
// Bus: SPI MODE 1 (CPOL=0, CPHA=1), 16-bit frames, MSB first, software NSS.
// ---------------------------------------------------------------------------

// SSC command words (from BTT mydef.h, standard TLE5012B encoding:
// bit15 RW=1 read, bits 9:4 register address, bits 3:0 word count).
static const uint16_t TLE_CMD_READ_STATUS = 0x8001;  // STAT, 1 word
// Write command: RW=0, same address field. Writing STAT clears its latched
// error flags -- reading alone does not, which is why a power-on system-error
// flag can persist for the life of the session.
static const uint16_t TLE_CMD_WRITE_STATUS = 0x0001;
static const uint16_t TLE_CMD_READ_ANGLE  = 0x8021;  // AVAL, 1 word
static const uint16_t TLE_CMD_READ_SPEED  = 0x8031;  // ASPD, 1 word

// Datasheet maximum SSC clock is 8 MHz. BTT shipped /8 of a 72 MHz APB2 =
// 9 MHz, i.e. slightly over spec. Start inside spec; raise only with evidence.
static const uint32_t TLE_SPI_HZ = 4000000;

// ---------------------------------------------------------------------------
//  Encoder linearity correction
// ---------------------------------------------------------------------------
// This part is specified around +/-0.5-1 degrees of angle error, and that error
// is systematic -- a property of this magnet on this shaft, repeatable turn
// after turn. On a 2-pole motor you would ignore it. On a 50-pole-pair stepper
// every mechanical degree becomes FIFTY electrical degrees, and FOC starts
// losing torque past about 30 and reverses past 90.
//
// Measured on the ballscrew axis: 2.28 degrees peak, which is 114 electrical.
// The axis moved a millimetre or two and locked solid, because at some shaft
// positions the drive was braking the rotor rather than turning it.
//
// The table holds an additive correction in encoder counts, indexed by the top
// bits of the raw reading, and is interpolated between buckets -- a stepped
// correction would simply trade one smooth error for 256 sharp ones.
//
// It is specific to this magnet in this position on this motor. Re-run the
// sweep after any mechanical change.
#define TLE_CAL_BUCKETS   256
#define TLE_CAL_SHIFT     7               // 32768 / 256
extern volatile int16_t tle_cal_corr[TLE_CAL_BUCKETS];
extern volatile uint8_t tle_cal_enable;

class TLE5012B : public Sensor {
 public:
  explicit TLE5012B(int csPin = ENC_CS);

  void init(SPIClass* spi = &SPI);

  // --- Sensor interface (SimpleFOC) ---
  float getSensorAngle() override;   // 0 .. 2*PI
  int   needsSearch() override { return 0; }   // absolute sensor, no index hunt

  // --- Raw access / diagnostics ---
  uint16_t readRaw();       // 15-bit angle, 0..32767, one turn
  uint16_t readStatus();    // STAT register

  // Clear STAT's latched error flags. The safety word's SYS_ERR bit mirrors
  // them, and this firmware rejects any read whose safety word reports an
  // error -- so a latched power-on flag means every angle is discarded and
  // getSensorAngle() returns the same held value forever. Observed on a board
  // whose safety word read 0xBE36: SYS_ERR low, but the angle flagged VALID
  // and the CRC correct. Call once at init, after the bus is up.
  void     clearStatus();
  int16_t  readSpeed();     // ASPD register, signed

  // Link health. NOTE: there is deliberately NO per-read value check -- a
  // dead bus reads 0x0000 or 0x7FFF, but so does a perfectly healthy sensor
  // when the shaft sits at those angles, which it does once per revolution.
  // Validity must come from the STAT register, not from the angle value.
  bool     checkLink();                 // true if STAT looks sane
  uint32_t linkFails() const { return link_fails_; }
  uint32_t crcFails() const { return crc_fails_; }
  uint32_t statusFails() const { return status_fails_; }

  // Read a register AND the trailing safety word. The SSC protocol appends a
  // safety word after the data; BTT never clocked it, so corruption went
  // undetected. Returns false only if the bus looks dead.
  bool readSafe(uint16_t cmd, uint16_t* data, uint16_t* safety);

  // Safety-word status bits are ACTIVE LOW on this part: 1 = healthy.
  //
  //   bit 15  STAT_RES  sensor reset / status not yet read
  //   bit 14  SYS_ERR   internal system error
  //   bit 13  IF_ERR    interface access error
  //   bit 12  INV_ANG   angle value invalid
  //
  // Only the lower two speak to whether THIS ANGLE is trustworthy. The upper
  // two are latched condition flags, and that distinction matters: this
  // originally demanded all four, which meant a single latched SYS_ERR
  // discarded every angle for the life of the session with no way to clear it.
  // Writing STAT does not clear it. A board was measured reading 0xBE36 --
  // SYS_ERR low, but CRC correct, interface fine, and the sensor itself
  // flagging the angle VALID, while the encoder tracked 302 degrees of hand
  // rotation perfectly. The firmware was throwing away good data and the drive
  // looked mechanically dead.
  //
  // A latched flag that permanently disables a drive whose data is sound is a
  // worse failure than the one it guards against. CRC still catches corruption
  // and INV_ANG still catches angles the sensor itself distrusts; SYS_ERR is
  // counted and reported instead of acted on.
  static bool statusOk(uint16_t safety) { return (safety & 0x3000) == 0x3000; }
  static bool systemWarn(uint16_t safety) { return (safety & 0x4000) == 0; }

  uint32_t sysWarns() const { return sys_warns_; }

  // The last safety word actually received. Without this, a read rejected by
  // statusOk() is indistinguishable from any other -- the angle is simply held
  // and nothing says why. That cost a wrong diagnosis on board 3.
  uint16_t lastSafety() const { return last_safety_; }
  uint16_t lastStat() const { return last_stat_; }
  uint16_t lastRaw()  const { return last_raw_; }   // no bus traffic
  void     resetStats() { link_fails_ = 0; }

 private:
  uint16_t transfer(uint16_t cmd);

  int        cs_;
  SPIClass*  spi_ = nullptr;
  SPISettings settings_{TLE_SPI_HZ, MSBFIRST, SPI_MODE1};
  uint32_t   link_fails_ = 0;
  uint32_t   crc_fails_ = 0;
  uint32_t   status_fails_ = 0;
  uint16_t   last_stat_ = 0;
  uint16_t   last_safety_ = 0;
  uint32_t   sys_warns_ = 0;
  uint16_t   last_raw_ = 0;
};
