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
static const uint16_t TLE_CMD_READ_ANGLE  = 0x8021;  // AVAL, 1 word
static const uint16_t TLE_CMD_READ_SPEED  = 0x8031;  // ASPD, 1 word

// Datasheet maximum SSC clock is 8 MHz. BTT shipped /8 of a 72 MHz APB2 =
// 9 MHz, i.e. slightly over spec. Start inside spec; raise only with evidence.
static const uint32_t TLE_SPI_HZ = 4000000;

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
  static bool statusOk(uint16_t safety) { return (safety & 0xF000) == 0xF000; }
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
  uint16_t   last_raw_ = 0;
};
