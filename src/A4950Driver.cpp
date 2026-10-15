#include "A4950Driver.h"

#if !defined(STM32F1xx)
#error "A4950Driver programs STM32F1 TIM3/GPIOB registers directly"
#endif

// 72 MHz / (ARR+1) with PSC=0. 513 -> 140.1 kHz, matching the factory
// carrier and comfortably above hearing. Keep ARR >= 255 or duty resolution
// gets coarse. Defined in config.h so docs and code cannot drift apart.
static const uint32_t CARRIER_ARR = PWM_CARRIER_ARR;

uint32_t A4950Driver::carrierHz() const {
  return 72000000UL / ((TIM3->PSC + 1) * (TIM3->ARR + 1));
}

int A4950Driver::init() {
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();

  // TIM3 partial remap puts CH1 on PB4 and CH2 on PB5.
  __HAL_AFIO_REMAP_TIM3_PARTIAL();

  // Direction pins are plain outputs, parked low (coast) until commanded.
  const int dirs[4] = {COIL_A_DIR_1, COIL_A_DIR_2, COIL_B_DIR_1, COIL_B_DIR_2};
  for (int i = 0; i < 4; i++) { pinMode(dirs[i], OUTPUT); digitalWrite(dirs[i], LOW); }

  // PB4/PB5 -> alternate function push-pull, 50 MHz (CRL nibbles 4 and 5).
  GPIOB->CRL = (GPIOB->CRL & ~0x00FF0000UL) | 0x00BB0000UL;

  TIM3->CR1   = 0;                       // stop while reconfiguring
  TIM3->PSC   = 0;
  TIM3->ARR   = CARRIER_ARR;
  TIM3->CCMR1 = (6UL << 4)  | TIM_CCMR1_OC1PE     // CH1 PWM mode 1, preloaded
              | (6UL << 12) | TIM_CCMR1_OC2PE;    // CH2 PWM mode 1, preloaded
  TIM3->CCER  = TIM_CCER_CC1E | TIM_CCER_CC2E;
  TIM3->CCR1  = 0;
  TIM3->CCR2  = 0;
  TIM3->EGR   = TIM_EGR_UG;              // latch PSC/ARR/CCMR before enabling
  TIM3->CR1   = TIM_CR1_ARPE | TIM_CR1_CEN;

  if (!_isset(voltage_limit) || voltage_limit > voltage_power_supply)
    voltage_limit = voltage_power_supply;

  initialized = true;
  return 1;
}

void A4950Driver::enable() {
  phase_a_on_ = phase_b_on_ = true;
  setPwm(0, 0);
}

void A4950Driver::disable() {
  TIM3->CCR1 = 0;
  TIM3->CCR2 = 0;
  digitalWrite(COIL_A_DIR_1, LOW); digitalWrite(COIL_A_DIR_2, LOW);
  digitalWrite(COIL_B_DIR_1, LOW); digitalWrite(COIL_B_DIR_2, LOW);
}

void A4950Driver::setPhaseState(PhaseState sa, PhaseState sb) {
  phase_a_on_ = (sa != PHASE_OFF);
  phase_b_on_ = (sb != PHASE_OFF);
  if (!phase_a_on_) {
    TIM3->CCR1 = 0;
    digitalWrite(COIL_A_DIR_1, LOW); digitalWrite(COIL_A_DIR_2, LOW);
  }
  if (!phase_b_on_) {
    TIM3->CCR2 = 0;
    digitalWrite(COIL_B_DIR_1, LOW); digitalWrite(COIL_B_DIR_2, LOW);
  }
}

// Duty is computed against the LIVE ARR, so the carrier can change without
// stranding a stale compare value above it (the hazard that made the earlier
// outside-the-library fix unsafe).
void A4950Driver::coilRaw(volatile uint32_t* ccr, int d1, int d2,
                         float duty, bool positive) {
  digitalWrite(d1, positive ? HIGH : LOW);
  digitalWrite(d2, positive ? LOW  : HIGH);
  uint32_t top = TIM3->ARR + 1;
  uint32_t v = (uint32_t)(duty * top);
  if (v > top) v = top;                 // never leave CCR above ARR
  *ccr = v;
}

void A4950Driver::setPwm(float Ua, float Ub) {
  Ua = _constrain(Ua, -voltage_limit, voltage_limit);
  Ub = _constrain(Ub, -voltage_limit, voltage_limit);

  // voltage_power_supply is SIMPLEFOC_VPS (3.2), so these "volts" are amps.
  float da = _constrain(fabsf(Ua) / voltage_power_supply, 0.0f, 1.0f);
  float db = _constrain(fabsf(Ub) / voltage_power_supply, 0.0f, 1.0f);

  if (phase_a_on_) coilRaw(&TIM3->CCR1, COIL_A_DIR_1, COIL_A_DIR_2, da, Ua >= 0);
  if (phase_b_on_) coilRaw(&TIM3->CCR2, COIL_B_DIR_1, COIL_B_DIR_2, db, Ub >= 0);
}
