#include "CalStore.h"

// Layout in the page. Header first so a blank or foreign page is rejected
// before anything is copied anywhere.
struct CalHeader {
  uint32_t magic;       // 'S','F','C','1'
  uint16_t buckets;     // must equal TLE_CAL_BUCKETS
  uint16_t version;
  uint32_t crc;         // over the table bytes only
};
static const uint32_t CAL_MAGIC   = 0x31434653UL;   // "SFC1" little-endian
static const uint16_t CAL_VERSION = 1;

static bool cal_stored = false;
bool calStored() { return cal_stored; }

// Plain bitwise CRC-32. Run once at boot over 512 bytes; a table would cost
// more flash than the speed is worth here.
static uint32_t crc32(const uint8_t* p, uint32_t n) {
  uint32_t c = 0xFFFFFFFFUL;
  while (n--) {
    c ^= *p++;
    for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320UL & (-(int32_t)(c & 1)));
  }
  return ~c;
}

bool calLoad() {
  const CalHeader* h = (const CalHeader*)CAL_FLASH_ADDR;
  if (h->magic != CAL_MAGIC) return false;          // blank or never written
  if (h->buckets != TLE_CAL_BUCKETS) return false;  // table geometry changed
  if (h->version != CAL_VERSION) return false;

  const uint8_t* tbl = (const uint8_t*)(CAL_FLASH_ADDR + sizeof(CalHeader));
  if (crc32(tbl, TLE_CAL_BUCKETS * 2) != h->crc) return false;

  const int16_t* src = (const int16_t*)tbl;
  for (uint16_t k = 0; k < TLE_CAL_BUCKETS; k++) tle_cal_corr[k] = src[k];
  tle_cal_enable = 1;
  cal_stored = true;
  return true;
}

bool calSave() {
  // Belt and braces on top of the static_asserts: refuse at runtime too,
  // because the cost of erasing the wrong page is a board that can never be
  // calibrated again.
  if (CAL_FLASH_ADDR >= CAL_FACTORY_ADDR) return false;
  if (CAL_FLASH_ADDR + CAL_FLASH_PAGE_SZ > CAL_FACTORY_ADDR) return false;

  uint8_t buf[TLE_CAL_BUCKETS * 2];
  int16_t* t = (int16_t*)buf;
  for (uint16_t k = 0; k < TLE_CAL_BUCKETS; k++) t[k] = tle_cal_corr[k];

  CalHeader h;
  h.magic   = CAL_MAGIC;
  h.buckets = TLE_CAL_BUCKETS;
  h.version = CAL_VERSION;
  h.crc     = crc32(buf, sizeof(buf));

  HAL_FLASH_Unlock();

  FLASH_EraseInitTypeDef er = {};
  er.TypeErase   = FLASH_TYPEERASE_PAGES;
  er.PageAddress = CAL_FLASH_ADDR;
  er.NbPages     = 1;
  uint32_t bad = 0;
  if (HAL_FLASHEx_Erase(&er, &bad) != HAL_OK) { HAL_FLASH_Lock(); return false; }

  // STM32F1 programs 16 bits at a time.
  const uint16_t* hw = (const uint16_t*)&h;
  uint32_t addr = CAL_FLASH_ADDR;
  for (uint32_t k = 0; k < sizeof(CalHeader) / 2; k++, addr += 2)
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, hw[k]) != HAL_OK) {
      HAL_FLASH_Lock(); return false;
    }
  const uint16_t* tw = (const uint16_t*)buf;
  for (uint32_t k = 0; k < sizeof(buf) / 2; k++, addr += 2)
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, tw[k]) != HAL_OK) {
      HAL_FLASH_Lock(); return false;
    }

  HAL_FLASH_Lock();

  // Read it back through the same path that will load it at boot. Reporting
  // success on a write we have not verified is how a table that silently
  // failed to program gets discovered later, on a moving machine.
  cal_stored = false;
  return calLoad();
}
