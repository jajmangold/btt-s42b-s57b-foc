#pragma once
#include <Arduino.h>
#include "TLE5012B.h"

// ---------------------------------------------------------------------------
//  Encoder calibration table, stored in flash
// ---------------------------------------------------------------------------
// The correction table is specific to one magnet in one position on one motor,
// and without it this drive cannot run closed loop at all -- see
// docs/measurements.md. Keeping it only in RAM would mean recalibrating after
// every power cycle, so it lives in one 1 KB flash page.
//
// WHERE IT LIVES, AND WHY THAT EXACT PAGE. The STM32F103CB has 128 KB of flash
// in 1 KB pages. Page 95 onwards (0x17C00) holds the FACTORY per-motor
// calibration, which is unique to each board and cannot be recovered once
// erased -- that is the single most destructive mistake available on this
// hardware. This table goes in page 94, immediately below it: as far from the
// growing firmware as possible while still never touching the factory data.
//
// The guards below are not decoration. A page number computed wrongly here
// erases either the running firmware or the irreplaceable factory table.
#define CAL_FLASH_ADDR    0x08017800UL      // page 94
#define CAL_FLASH_PAGE_SZ 1024UL
#define CAL_FACTORY_ADDR  0x08017C00UL      // page 95 -- NEVER erase at or above

static_assert(CAL_FLASH_ADDR + CAL_FLASH_PAGE_SZ <= CAL_FACTORY_ADDR,
              "calibration page would overlap the factory calibration");
static_assert(CAL_FLASH_ADDR >= 0x08010000UL,
              "calibration page is too close to the firmware");

// Load the table from flash into tle_cal_corr and enable correction.
// Returns false (leaving correction OFF) if the page is blank or fails its CRC,
// so an uncalibrated board runs uncorrected rather than on garbage.
bool calLoad();

// Write the current contents of tle_cal_corr to flash. Returns false on any
// erase or program failure, or if the guards reject the address.
bool calSave();

// True if the stored table passed its CRC at boot.
bool calStored();
