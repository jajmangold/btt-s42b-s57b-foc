#include "StepDir.h"

#if !defined(STM32F1xx)
#error "StepDir.cpp programs STM32F1 TIM2/EXTI registers directly"
#endif

// DIR changes are infrequent compared to step pulses, so an interrupt on the
// direction pin costs nothing and reacts immediately -- whereas polling it
// would apply a direction change late, after the hardware counter had already
// moved some pulses the wrong way.
static void dirChanged() {
  if (digitalRead(P1_DIR_PIN) == STEP_DIR_REVERSE_LEVEL)
    TIM2->CR1 |= TIM_CR1_DIR;        // count down
  else
    TIM2->CR1 &= ~TIM_CR1_DIR;       // count up
}

void StepDir::init() {
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE();

  // PA0 = ETR, PA1 = DIR, PA2 = ENABLE. All are optocoupler outputs on this
  // board, i.e. open collector, so they need a pull-up to read as high when
  // the opto is dark. CRL nibbles 0..2; 0x8 = input with pull, ODR selects up.
  GPIOA->CRL = (GPIOA->CRL & ~0x00000FFFUL) | 0x00000888UL;
  GPIOA->ODR |= 0x0007;                       // pull-ups on PA0, PA1, PA2

  // TIM2 in external clock mode 2: the counter advances on every edge of ETR,
  // entirely in hardware. Filter set to 0b0111 to reject the ringing that a
  // long step cable will inevitably pick up.
  TIM2->CR1  = 0;
  TIM2->PSC  = 0;
  TIM2->ARR  = 0xFFFF;                        // free-running, we difference it
  TIM2->SMCR = TIM_SMCR_ECE                   // external clock mode 2
             | (0x7UL << TIM_SMCR_ETF_Pos);   // digital filter
  TIM2->CNT  = 0;
  TIM2->EGR  = TIM_EGR_UG;
  TIM2->CR1 |= TIM_CR1_CEN;

  raw_prev_ = 0;
  steps_    = 0;

  // Direction: set the initial state, then follow it on both edges.
  dirChanged();
  attachInterrupt(digitalPinToInterrupt(P1_DIR_PIN), dirChanged, CHANGE);
}

void StepDir::update() {
  uint16_t raw = (uint16_t)TIM2->CNT;
  // 16-bit wrapping subtraction: correct across the 0xFFFF/0x0000 boundary in
  // both directions, which is why the counter can be left free-running.
  steps_   += (int16_t)(raw - raw_prev_);
  raw_prev_ = raw;
}

bool StepDir::hostEnabled() const {
#ifdef STEP_DIR_USE_ENABLE
  return digitalRead(P1_EN_PIN) != STEP_DIR_DISABLE_LEVEL;
#else
  return true;     // ENABLE ignored unless explicitly opted in -- see config.h
#endif
}
