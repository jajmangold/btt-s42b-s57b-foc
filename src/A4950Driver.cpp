#include "A4950Driver.h"

#if !defined(STM32F1xx)
#error "A4950Driver programs STM32F1 TIM3/GPIOB registers directly"
#endif

// Target carrier, well above hearing and matching the factory value, which is
// what the board's VREF RC filter was designed around. ARR is computed from
// the TIMER CLOCK AT RUNTIME rather than assuming a 72 MHz core, so the
// carrier stays correct even if the crystal fails and the part falls back to
// the internal RC. Keep ARR >= 255 or duty resolution gets coarse.
extern uint32_t apb1TimerClock();
static const uint32_t CARRIER_HZ = 140000;
static uint32_t carrierArr() {
  uint32_t arr = (apb1TimerClock() / CARRIER_HZ) - 1;
  if (arr < 255)   arr = 255;
  if (arr > 65535) arr = 65535;
  return arr;
}

uint32_t A4950Driver::carrierHz() const {
  return apb1TimerClock() / ((TIM3->PSC + 1) * (TIM3->ARR + 1));
}

void a4950_claim_pins() {
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // ---- AFIO_MAPR: ONE EXPLICIT WRITE, NEVER READ-MODIFY-WRITE ------------
  // PB4 is NJTRST. It only becomes usable as PWM once SWJ_CFG selects
  // "JTAG disabled, SW enabled" (0b010). Until then the pin belongs to the
  // JTAG peripheral and pinMode() on it does nothing -- which means the coil
  // cannot be put in a safe state either. So this must come FIRST.
  //
  // It also frees PB3 (JTDO) and PA15 (JTDI), which carry DIP switches on
  // this board. DIP 3 shorts PB3 to ground, so leaving it as a JTAG output
  // any longer than necessary is asking for trouble.
  //
  // ST's own remap macros are hostile here: AFIO_REMAP_PARTIAL() does
  //     tmpreg = AFIO->MAPR; ... tmpreg |= AFIO_MAPR_SWJ_CFG; AFIO->MAPR = tmpreg;
  // which ORs in ALL THREE SWJ_CFG bits (0b111 = JTAG *and* SWD disabled).
  // SWJ_CFG is write-only on STM32F1 anyway, so read-modify-write cannot
  // preserve it. Write the whole register once; nothing else needs a remap.
  AFIO->MAPR = (0x2u << AFIO_MAPR_SWJ_CFG_Pos)      // JTAG off, SW-DP alive
             | (0x2u << AFIO_MAPR_TIM3_REMAP_Pos);  // partial: CH1=PB4, CH2=PB5

  // Now PB4 is ours, the coils can actually be safed. Zero duty with both
  // direction pins low is coast on an A4950: no current path at all.
  const int pins[6] = { COIL_A_PWM, COIL_A_DIR_1, COIL_A_DIR_2,
                        COIL_B_PWM, COIL_B_DIR_1, COIL_B_DIR_2 };
  for (int i = 0; i < 6; i++) { pinMode(pins[i], OUTPUT); digitalWrite(pins[i], LOW); }
}

int A4950Driver::init() {
  a4950_claim_pins();                 // idempotent; safe if already called
  __HAL_RCC_TIM3_CLK_ENABLE();

  // Direction pins are plain outputs, parked low (coast) until commanded.
  const int dirs[4] = {COIL_A_DIR_1, COIL_A_DIR_2, COIL_B_DIR_1, COIL_B_DIR_2};
  for (int i = 0; i < 4; i++) { pinMode(dirs[i], OUTPUT); digitalWrite(dirs[i], LOW); }

  // PB4/PB5 -> alternate function push-pull, 50 MHz (CRL nibbles 4 and 5).
  GPIOB->CRL = (GPIOB->CRL & ~0x00FF0000UL) | 0x00BB0000UL;

  TIM3->CR1   = 0;                       // stop while reconfiguring
  TIM3->PSC   = 0;
  TIM3->ARR   = carrierArr();
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
