#pragma once
#include <Arduino.h>
#include "config.h"

// Local, so this header does not have to pull in all of SimpleFOC.
static const float STEPDIR_2PI = 6.28318530718f;

// ---------------------------------------------------------------------------
// STEP / DIR / ENABLE input on the P1 connector.
//
// This is the interface most hosts speak -- FluidNC, LinuxCNC, GRBL and every
// standard breakout emit step pulses -- so it is the drop-in path for using
// these boards as closed-loop drives without changing anything upstream.
//
// Pulses are counted in HARDWARE: PA0 is TIM2's external clock input (ETR), so
// the step rate costs no CPU whatsoever and cannot be missed by a busy control
// loop. PA1 (DIR) drives EXTI1, whose handler flips TIM2 between up- and
// down-counting. This is exactly how the factory firmware does it, and it is
// strictly better than counting pulses in an interrupt.
//
// The counter is 16-bit and free-running; absolute position is accumulated by
// differencing it with wrapping arithmetic. At the loop rates this firmware
// runs, a wrap would need step rates in the hundreds of megahertz to be missed.
// ---------------------------------------------------------------------------

class StepDir {
 public:
  void init();

  // Accumulate the hardware counter into an absolute step position. Cheap:
  // one register read plus a subtraction. Call from the main loop.
  void update();

  int32_t steps() const { return steps_; }
  float   angle() const { return (float)steps_ * STEPDIR_2PI / (float)STEP_DIR_STEPS_PER_REV; }

  // Zero the step position at the current shaft angle, so the host's notion of
  // position and ours agree from here on.
  void zero() { steps_ = 0; raw_prev_ = (uint16_t)TIM2->CNT; }

  // ENABLE input. Returns true when the host is asking the drive to hold.
  // With nothing connected the optocoupler is dark and the pin reads high, so
  // the default polarity treats "not connected" as enabled -- a host that
  // never wires ENABLE still gets a working drive.
  bool hostEnabled() const;

  uint16_t rawCount() const { return (uint16_t)TIM2->CNT; }

 private:
  int32_t  steps_ = 0;
  uint16_t raw_prev_ = 0;
};
