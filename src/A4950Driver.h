#pragma once
#include <Arduino.h>
#include <SimpleFOC.h>
#include "config.h"

// ---------------------------------------------------------------------------
// Stepper driver for the BIGTREETECH S42B / S57B V2.0, owning TIM3 directly.
//
// WHY NOT StepperDriver2PWM: on this target SimpleFOC ignores
// driver.pwm_frequency entirely -- building with 25 kHz and with 100 kHz both
// leave TIM3 at PSC=0 ARR=63999, i.e. a 1.12 kHz carrier. That is audible, and
// far below the ~140 kHz the board's VREF RC filter was designed for, so VREF
// never settles to a DC current setpoint. Forcing ARR from outside is UNSAFE
// (SimpleFOC resets it mid-loop, stranding a stale CCR > ARR = 100% duty).
// So we own the timer instead, and keep SimpleFOC's FOC/PID/sensor stack.
//
// The register values below reproduce BTT's own TIM3 setup, which we read off
// the stock firmware: CR1=0x81, CCMR1=0x6868, CCER=0x11, partial remap.
//
// Pin pairing is MEASURED, not assumed (see CHANGELOG 2026-10-15):
//   CH1 = PB4 drives the A4950 whose direction pins are PB8/PB9  (coil A)
//   CH2 = PB5 drives the A4950 whose direction pins are PB6/PB7  (coil B)
// ---------------------------------------------------------------------------

// MUST be the first thing main() does, before ANY pin on this board is
// touched. PB4 is NJTRST and belongs to the JTAG peripheral until SWJ_CFG
// says otherwise -- pinMode() on it before this call silently does nothing,
// so the coils cannot even be safed. See A4950Driver.cpp.
void a4950_claim_pins();

class A4950Driver : public StepperDriver {
 public:
  A4950Driver() = default;

  int  init() override;
  void enable() override;
  void disable() override;
  void setPwm(float Ua, float Ub) override;
  void setPhaseState(PhaseState sa, PhaseState sb) override;

  // Actual carrier, computed from the registers we programmed. Reported rather
  // than assumed, because "the frequency I asked for" was the whole problem.
  uint32_t carrierHz() const;

 private:
  void coilRaw(volatile uint32_t* ccr, int d1, int d2, float duty, bool positive);
  bool phase_a_on_ = true, phase_b_on_ = true;
};
