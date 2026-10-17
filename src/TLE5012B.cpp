#include "TLE5012B.h"

// ---------------------------------------------------------------------------
// Half-duplex line turnaround.
//
// PA7 (MOSI) occupies bits 31:28 of GPIOA->CRL on STM32F1.
//   0x4 = CNF 01 / MODE 00 -> floating input   (release the shared line)
//   0xB = CNF 10 / MODE 11 -> AF push-pull 50MHz (drive it again)
// Lifted verbatim from BTT's SPI_TX_OFF / SPI_TX_ON, which are proven on this
// board. STM32F1-only by construction -- this board is an F103 and the SWD
// pins are the only way in, so there is no portability story to protect.
// ---------------------------------------------------------------------------
#if !defined(STM32F1xx)
#error "TLE5012B.cpp assumes STM32F1 GPIO CRL layout (S42B / S57B V2.0 = STM32F103)"
#endif

static inline void mosiRelease() {
  GPIOA->CRL = (GPIOA->CRL & 0x0FFFFFFFu) | 0x40000000u;
}
static inline void mosiDrive() {
  GPIOA->CRL = (GPIOA->CRL & 0x0FFFFFFFu) | 0xB0000000u;
}

TLE5012B::TLE5012B(int csPin) : cs_(csPin) {}

void TLE5012B::init(SPIClass* spi) {
  spi_ = spi;

  pinMode(cs_, OUTPUT);
  digitalWrite(cs_, HIGH);        // idle high, sensor deselected

  spi_->begin();
  mosiDrive();                    // SPI.begin() leaves PA7 as AF; be explicit

  // Give the sensor its power-on settling time before the first transaction.
  delay(10);
}

// One 16-bit frame. Caller owns CS and the MOSI direction.
uint16_t TLE5012B::transfer(uint16_t cmd) {
  return spi_->transfer16(cmd);
}

uint16_t TLE5012B::readRaw() {
  spi_->beginTransaction(settings_);
  digitalWrite(cs_, LOW);

  transfer(TLE_CMD_READ_ANGLE);   // command out; return value is bus echo, junk
  mosiRelease();                  // hand the shared line to the sensor
  uint16_t v = transfer(0xFFFF);  // clock the data word back
  digitalWrite(cs_, HIGH);
  mosiDrive();                    // take the line back

  spi_->endTransaction();

  // AVAL bit15 is the RD_AV update flag, bits 14:0 are the angle.
  // BTT additionally shifted >>1 and used 14 bits; we keep all 15.
  last_raw_ = v & 0x7FFF;
  return last_raw_;
}

uint16_t TLE5012B::readStatus() {
  spi_->beginTransaction(settings_);
  digitalWrite(cs_, LOW);
  transfer(TLE_CMD_READ_STATUS);
  mosiRelease();
  uint16_t v = transfer(0xFFFF);
  digitalWrite(cs_, HIGH);
  mosiDrive();
  spi_->endTransaction();
  return v;
}

void TLE5012B::clearStatus() {
  spi_->beginTransaction(settings_);
  digitalWrite(cs_, LOW);
  transfer(TLE_CMD_WRITE_STATUS);
  transfer(0x0000);               // all flags cleared
  digitalWrite(cs_, HIGH);
  spi_->endTransaction();
  // MOSI stays driven: a write never hands the line to the sensor.
}

int16_t TLE5012B::readSpeed() {
  spi_->beginTransaction(settings_);
  digitalWrite(cs_, LOW);
  transfer(TLE_CMD_READ_SPEED);
  mosiRelease();
  uint16_t v = transfer(0xFFFF);
  digitalWrite(cs_, HIGH);
  mosiDrive();
  spi_->endTransaction();

  // 15-bit two's complement, sign-extend from bit 14.
  int16_t s = (int16_t)(v & 0x7FFF);
  if (s & 0x4000) s |= 0x8000;
  return s;
}

// CRC-8, polynomial 0x1D, seed 0xFF, over [cmd_hi, cmd_lo, data_hi, data_lo],
// result INVERTED. MEASURED, not assumed: of four plausible readings of the
// datasheet, this one matched the sensor 200/200 and the others 0/200.
static uint8_t tle_crc(uint16_t cmd, uint16_t data) {
  const uint8_t b[4] = { (uint8_t)(cmd >> 8),  (uint8_t)(cmd & 0xFF),
                         (uint8_t)(data >> 8), (uint8_t)(data & 0xFF) };
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < 4; i++) {
    crc ^= b[i];
    for (uint8_t k = 0; k < 8; k++)
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x1D) : (uint8_t)(crc << 1);
  }
  return (uint8_t)~crc;
}

volatile int16_t tle_cal_corr[TLE_CAL_BUCKETS];
volatile uint8_t tle_cal_enable = 0;

// Linear interpolation between adjacent buckets, wrapping at the top because
// the angle does. Returns a corrected 15-bit count.
static uint16_t tleApplyCal(uint16_t raw) {
  uint16_t idx  = raw >> TLE_CAL_SHIFT;                 // 0..255
  uint16_t frac = raw & ((1u << TLE_CAL_SHIFT) - 1);    // 0..127
  int32_t  c0   = tle_cal_corr[idx];
  int32_t  c1   = tle_cal_corr[(idx + 1) & (TLE_CAL_BUCKETS - 1)];
  int32_t  corr = c0 + ((c1 - c0) * (int32_t)frac >> TLE_CAL_SHIFT);
  return (uint16_t)(((int32_t)raw + corr) & 0x7FFF);
}

float TLE5012B::getSensorAngle() {
  // Validated read. Every 15-bit value is a legal angle, so the data word
  // alone can never reveal corruption -- only the safety word can. A bad
  // angle now commands real current, so this matters in a way it did not at
  // milestone 1. On any failure, hold the last good angle rather than act on
  // a value we cannot vouch for.
  uint16_t d, sw;
  if (!readSafe(TLE_CMD_READ_ANGLE, &d, &sw)) { link_fails_++;   return angle_prev; }
  last_safety_ = sw;
  if (systemWarn(sw)) sys_warns_++;   // reported, not acted on -- see the header
  if (!statusOk(sw))                          { status_fails_++; return angle_prev; }
  if ((uint8_t)(sw & 0xFF) != tle_crc(TLE_CMD_READ_ANGLE, d)) {
    crc_fails_++;
    return angle_prev;
  }
  uint16_t raw = (uint16_t)(d & 0x7FFF);
  if (tle_cal_enable) raw = tleApplyCal(raw);
  return (float)raw * _2PI / 32768.0f;
}

bool TLE5012B::checkLink() {
  uint16_t st = readStatus();
  last_stat_ = st;
  // A dead SSC bus reads all-ones (idle pull-up) or all-zeros (held low).
  // A live TLE5012B never reports either in STAT.
  if (st == 0xFFFF || st == 0x0000) {
    link_fails_++;
    return false;
  }
  return true;
}


// ---------------------------------------------------------------------------
// Safety word. Transaction is: command -> tri-state -> data word -> safety
// word -> CS high. Identical to a normal read plus 16 more clocks.
// ---------------------------------------------------------------------------
bool TLE5012B::readSafe(uint16_t cmd, uint16_t* data, uint16_t* safety) {
  spi_->beginTransaction(settings_);
  digitalWrite(cs_, LOW);
  transfer(cmd);
  mosiRelease();
  uint16_t d = transfer(0xFFFF);
  uint16_t s = transfer(0xFFFF);
  digitalWrite(cs_, HIGH);
  mosiDrive();
  spi_->endTransaction();
  *data = d;
  *safety = s;
  last_raw_ = d & 0x7FFF;
  return !(s == 0xFFFF || s == 0x0000);
}
