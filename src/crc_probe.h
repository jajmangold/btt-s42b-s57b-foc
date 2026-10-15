#pragma once
#include <stdint.h>

// CRC-8 used by Infineon SSC parts: polynomial 0x1D (x^8+x^4+x^3+x^2+1).
// The seed and which bytes are covered are what we are trying to establish, so
// both are parameters rather than assumptions.
static inline uint8_t crc8_1d(const uint8_t* p, uint8_t n, uint8_t seed) {
  uint8_t crc = seed;
  for (uint8_t i = 0; i < n; i++) {
    crc ^= p[i];
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x1D) : (uint8_t)(crc << 1);
  }
  return crc;
}

// Four plausible readings of the datasheet. The sensor decides which is right.
//   A: command + data, inverted      C: data only, inverted
//   B: command + data, plain        D: data only, plain
static inline void crc_variants(uint16_t cmd, uint16_t data, uint8_t out[4]) {
  uint8_t full[4] = { (uint8_t)(cmd >> 8),  (uint8_t)(cmd & 0xFF),
                      (uint8_t)(data >> 8), (uint8_t)(data & 0xFF) };
  uint8_t just[2] = { (uint8_t)(data >> 8), (uint8_t)(data & 0xFF) };
  uint8_t f = crc8_1d(full, 4, 0xFF);
  uint8_t j = crc8_1d(just, 2, 0xFF);
  out[0] = (uint8_t)~f;
  out[1] = f;
  out[2] = (uint8_t)~j;
  out[3] = j;
}
