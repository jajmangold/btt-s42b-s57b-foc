#include <Arduino.h>

// ---------------------------------------------------------------------------
// System clock: 8 MHz crystal -> PLL x9 -> 72 MHz.
//
// WHY THIS FILE EXISTS. The Arduino core's default configuration for a generic
// STM32F103 board runs the PLL from HSI/2, giving 4 MHz x 16 = 64 MHz, and
// never enables HSE at all. That is a sane default for a board that might not
// have a crystal. These boards do have one, and the difference is not cosmetic:
//
//   * CAN bit timing is derived from APB1. At 64 MHz SYSCLK, APB1 is 32 MHz
//     and a nominal 500 kbit/s becomes 444 kbit/s. Loopback still passes,
//     because both ends share the error, so the fault only appears when the
//     drive first meets a correctly-clocked peer on a real bus.
//   * UART baud is derived from APB2. 38400 becomes 43191, a 12% error, which
//     no receiver will tolerate.
//   * The internal RC is +-1-2% over temperature. A crystal is +-50 ppm. CAN
//     needs the crystal regardless of the arithmetic.
//
// This override is weak-linked over the core's version, so simply defining it
// replaces the default.
//
// If HSE fails to start -- an unpopulated or dead crystal -- we deliberately
// fall back to the core's 64 MHz HSI configuration rather than hang. Everything
// downstream derives its timing from the clock actually achieved, so the drive
// still runs; only CAN and UART are affected, and clockIsCorrect() reports it.
// ---------------------------------------------------------------------------

extern "C" void SystemClock_Config(void) {
  RCC_OscInitTypeDef osc = {};
  RCC_ClkInitTypeDef clk = {};

  // --- preferred: 8 MHz crystal, PLL x9 -> 72 MHz --------------------------
  osc.OscillatorType      = RCC_OSCILLATORTYPE_HSE;
  osc.HSEState            = RCC_HSE_ON;
  osc.HSEPredivValue      = RCC_HSE_PREDIV_DIV1;
  osc.PLL.PLLState        = RCC_PLL_ON;
  osc.PLL.PLLSource       = RCC_PLLSOURCE_HSE;
  osc.PLL.PLLMUL          = RCC_PLL_MUL9;

  bool hse_ok = (HAL_RCC_OscConfig(&osc) == HAL_OK);

  if (!hse_ok) {
    // --- fallback: internal RC, PLL x16 -> 64 MHz --------------------------
    osc = RCC_OscInitTypeDef{};
    osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState            = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState        = RCC_PLL_ON;
    osc.PLL.PLLSource       = RCC_PLLSOURCE_HSI_DIV2;
    osc.PLL.PLLMUL          = RCC_PLL_MUL16;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) return;   // leave whatever boot gave us
  }

  clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV2;   // APB1 max is 36 MHz
  clk.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2);
}

// True when the crystal came up and the core is at the intended 72 MHz.
// Anything deriving a bit rate should check this, or better, derive from the
// actual clock rather than assuming.
bool clockIsCorrect() {
  return (HAL_RCC_GetSysClockFreq() > 70000000UL);
}

// Timer clock for peripherals on APB1 (TIM2, TIM3, CAN's parent bus).
// When the APB1 prescaler is not 1, timers run at twice PCLK1.
uint32_t apb1TimerClock() {
  uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
  return (pclk1 == HAL_RCC_GetHCLKFreq()) ? pclk1 : pclk1 * 2;
}
