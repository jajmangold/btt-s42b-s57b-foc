#pragma once
// ===========================================================================
//  BOARD SELECTION  -- READ THIS BEFORE YOU FLASH
// ===========================================================================
// The S42B V2.0 and S57B V2.0 are electrically the same design: STM32F103CBT6,
// two A4950 full-bridges, a TLE5012B encoder, and identical firmware from the
// factory. They differ in ONE respect that matters to software:
//
//        S42B V2.0  maximum 1650 mA   (current sense 0.2 ohm)
//        S57B V2.0  maximum 3300 mA   (current sense 0.1 ohm)
//
// Exactly 2:1. Select the WRONG board here and every current command is out by
// a factor of two -- on an S42B that means asking for double what the driver
// can deliver. Nothing else in this firmware depends on the choice.
//
// (If you have seen CURRENT_SENSE_RESISTOR 0.2 in other S42B-derived projects:
//  that value is correct, for the S42B. It is only wrong on an S57B.)
// ---------------------------------------------------------------------------
#if !defined(BOARD_S42B_V2) && !defined(BOARD_S57B_V2)
  #error "Define BOARD_S42B_V2 or BOARD_S57B_V2 (see platformio.ini env sections)"
#endif

// The STM32 core's variant header also defines BOARD_NAME, so drop its value
// before taking the name over. Without this every translation unit that
// includes both emits a redefinition warning.
#ifdef BOARD_NAME
  #undef BOARD_NAME
#endif

#if defined(BOARD_S57B_V2)
  #define BOARD_NAME            "BIGTREETECH S57B V2.0"
  #define BOARD_TAG             "S57B"
  #define SENSE_RESISTOR_OHM    0.1f
  #define DRIVER_MAX_mA         3300    // manual, Product Parameters
  #define FACTORY_DEFAULT_mA    1600
#else
  #define BOARD_NAME            "BIGTREETECH S42B V2.0"
  #define BOARD_TAG             "S42B"
  #define SENSE_RESISTOR_OHM    0.2f
  #define DRIVER_MAX_mA         1650
  #define FACTORY_DEFAULT_mA     800
#endif

// ---------------------------------------------------------------------------
//  MOTOR current limit -- separate from the BOARD limit above
// ---------------------------------------------------------------------------
// DRIVER_MAX_mA is what the board can deliver. MOTOR_MAX_mA is what the motor
// on the end of it can survive, and it is the lower of the two that matters.
// An S57B will drive 3300 mA into a motor rated 2800 mA without complaint; the
// motor is what fails, slowly, as insulation damage you do not see until it
// shorts.
//
// Set this to the motor's rated phase current. Nothing in the firmware will
// command more, whatever arrives over CAN or UART.
//
//   JK57HS76-2804  NEMA 23, 76 mm, 2.8 A/phase  ->  2800
//
#ifndef MOTOR_MAX_mA
#define MOTOR_MAX_mA            DRIVER_MAX_mA   // default: trust the board limit
#endif

// The effective ceiling: whichever of board and motor gives out first.
#define CURRENT_CEILING_mA  (MOTOR_MAX_mA < DRIVER_MAX_mA ? MOTOR_MAX_mA : DRIVER_MAX_mA)

// Conservative starting current for bring-up. Raise deliberately, not by habit.
#define BRINGUP_mA              (FACTORY_DEFAULT_mA / 2)

// ---------------------------------------------------------------------------
//  Encoder: Infineon TLE5012B on SPI1, 3-wire HALF-DUPLEX SSC
// ---------------------------------------------------------------------------
// MOSI and MISO are joined on the PCB, so MOSI must be tri-stated after the
// command word or master and sensor fight for the shared line. See TLE5012B.cpp.
#define ENC_SCK   PA5
#define ENC_MISO  PA6
#define ENC_MOSI  PA7
#define ENC_CS    PA4

// ---------------------------------------------------------------------------
//  Motor: 2x A4950, driven as 2 PWM + 4 direction
// ---------------------------------------------------------------------------
// The "PWM" pin feeds the A4950 VREF through an RC filter, so it is a CURRENT
// SETPOINT, not a winding voltage. The direction pins set H-bridge polarity.
// TIM3 partial remap puts CH1 on PB4 and CH2 on PB5.
//   CH1 = PB4 pairs with PB8/PB9      CH2 = PB5 pairs with PB6/PB7
// That pairing is measured, not assumed -- see docs/measurements.md.
#define COIL_A_PWM    PB4
#define COIL_A_DIR_1  PB8
#define COIL_A_DIR_2  PB9
#define COIL_B_PWM    PB5
#define COIL_B_DIR_1  PB6
#define COIL_B_DIR_2  PB7

// ---------------------------------------------------------------------------
//  CAN (bxCAN) -> onboard transceiver -> J2 as CANH/CANL
// ---------------------------------------------------------------------------
// Present on both boards but NEVER used by the factory firmware: its entire
// CAN block is inside `#if 0`. No prior art, so treat as newly brought up.
#define CAN_RX  PA11
#define CAN_TX  PA12

// ---------------------------------------------------------------------------
//  Host interface connector (P1) and debug
// ---------------------------------------------------------------------------
// Resolved from the factory firmware. PA0 clocks TIM2 as an external clock
// (step pulses counted in hardware); PA1 drives EXTI1 and its handler flips
// TIM2's count direction. All three are input-with-pull-up, the signature of
// an open-collector optocoupler -- so P1 is very likely isolated and
// RECEIVE-ONLY. Not confirmed with a meter.
#define P1_STEP_PIN   PA0
#define P1_DIR_PIN    PA1
#define P1_EN_PIN     PA2

// --- STEP/DIR behaviour ----------------------------------------------------
// Steps per shaft revolution, as the HOST believes it. Set this to match the
// microstepping your controller is configured for; it is the only number that
// has to agree between the two. The factory firmware took this from the DIP
// switches (2/4/8/16/32 microsteps x 200 full steps); here it is explicit,
// because under FOC microstepping is a host-side fiction rather than anything
// the drive does.
#ifndef STEP_DIR_STEPS_PER_REV
#define STEP_DIR_STEPS_PER_REV  3200    // 200 x 16, a common default
#endif

// Level on DIR that means "count down". Flip if your axis runs backwards --
// this is the one-line fix, rather than rewiring or inverting in the host.
#ifndef STEP_DIR_REVERSE_LEVEL
#define STEP_DIR_REVERSE_LEVEL  HIGH
#endif

// ENABLE is IGNORED unless STEP_DIR_USE_ENABLE is defined. That is deliberate:
// the polarity of this input has not been confirmed on hardware, and guessing
// it wrong in the "enable" direction would energise a motor that the host
// believes is released. Opting in is a decision the integrator should make
// knowingly, after checking their own board.
// #define STEP_DIR_USE_ENABLE
#ifndef STEP_DIR_DISABLE_LEVEL
#define STEP_DIR_DISABLE_LEVEL  LOW
#endif

#define UART1_TX  PA9     // header P7, 38400 8N1 under factory firmware
#define UART1_RX  PA10
// SWD: PA13 = SWDIO, PA14 = SWCLK, on header P7. THE ONLY WAY IN. Do not remap.
// PB4/PB3/PA15 are JTAG pins out of reset -- the firmware frees them with
// SWJ NOJTAG, which keeps SW-DP alive. COIL_A_PWM is PB4, so this is required.

// ---------------------------------------------------------------------------
//  DIP switches -- CAN node address
// ---------------------------------------------------------------------------
// Four switches. The factory firmware used them for microstep and open/closed
// loop; neither means anything under FOC, so this firmware repurposes them as
// the CAN node address. That keeps the address in the hardware, where it
// belongs: two drives on one bus must differ, and having to remember which
// binary went on which board is exactly how you end up with two node 1s.
//
// All four are input-with-pull-up, so a switch that is ON reads 0. The address
// inverts that, so ON = 1 and a board with every switch OFF is node 1 -- the
// same address the firmware used when it was hardcoded, so existing
// single-drive setups are unaffected by this change.
//
// ORDERING TRAP: DIP 3 is PB3 (JTDO) and DIP 4 is PA15 (JTDI). Both belong to
// the JTAG peripheral out of reset and read nothing useful until SWJ_CFG is
// set to NOJTAG. a4950_claim_pins() does that remap, so the switches MUST NOT
// be read before it has run. This is the same trap that made an earlier
// pinMode(PB4) silently do nothing.
#define DIP1_PIN  PB10
#define DIP2_PIN  PB11
#define DIP3_PIN  PB3
#define DIP4_PIN  PA15

// ---------------------------------------------------------------------------
//  Home / limit sensor
// ---------------------------------------------------------------------------
// A 3-wire hall-effect module on PB15. The pin is a good choice for a reason
// worth recording: PB15 is 5 V tolerant, unlike PA0-PA7 and PB0-PB1, which are
// the ADC-capable pins and are not. A module powered from 5 V -- most of them
// are -- cannot over-volt it. Nothing else in this firmware touches PB15.
//
// These modules are open-collector with a pull-up, so the output idles HIGH and
// is pulled LOW by the magnet. Measured on hardware: 1 with no magnet, 0 with,
// both stable across repeated samples. The internal pull-up is enabled too, so
// the pin still has a defined level if the module is unplugged.
//
// FAILURE MODE, READ THIS BEFORE HOMING. With an active-low sensor, a broken
// signal wire reads exactly like "not triggered". During a seek that means the
// axis keeps driving until something physical stops it -- and on a ballscrew
// the thing that stops it may be the ball nut leaving the end of the thread,
// which scatters the balls and scraps the nut. HOME_MAX_TRAVEL_RAD is what
// makes that survivable: the seek gives up after a bounded distance instead of
// trusting a sensor that may no longer be connected.
#define HOME_SENSOR_PIN         PB15
#define HOME_SENSOR_ACTIVE_LOW  1
#define HOME_DEBOUNCE_SAMPLES   4     // consecutive agreeing reads to accept

// Homing motion. Slow and weak deliberately: this is the one operation that
// drives on purpose towards something solid. Force scales with current, so the
// seek current is what decides whether a missed trigger dents a bearing race
// or bends a bracket.
#define HOME_SEEK_VEL_RAD_S     1.0f
#define HOME_SEEK_mA            400
#define HOME_MAX_TRAVEL_RAD     125.0f   // ~20 turns, then abort
#define HOME_BACKOFF_RAD        0.25f    // clear of the trigger, towards travel

#define LED_PIN     PC13
#define BTN_DOWN    PA3
#define BTN_BACK    PB0
#define BTN_SELECT  PB1

// ---------------------------------------------------------------------------
//  Motor mechanical
// ---------------------------------------------------------------------------
#define MOTOR_POLE_PAIRS  50   // 1.8 deg/step hybrid = 200 steps/rev / 4

// ---------------------------------------------------------------------------
//  Current scaling: PWM duty -> VREF -> coil current
// ---------------------------------------------------------------------------
// A4950: Itrip = VREF / (10 * Rsense). VREF comes from PWM through an RC
// filter and cannot exceed the 3V3 rail, so 100% duty == the board maximum:
//    S57B  3.3 V / (10 * 0.1) = 3.3 A        S42B  3.3 V / (10 * 0.2) = 1.65 A
// Both match the manuals exactly, which is why we trust the derivation.
// NOTE: this scale is DERIVED, not measured with a meter. It is bounded, since
// VREF cannot exceed the rail, so the worst case is the board's rated maximum.
#define FULL_DUTY_mA     DRIVER_MAX_mA
#define PWM_CARRIER_ARR  513     // 72 MHz / 514 = 140.1 kHz, see A4950Driver.cpp
#define MA_TO_DUTY(mA)   ((int)((mA) * (long)(PWM_CARRIER_ARR + 1) / FULL_DUTY_mA))

// --- SimpleFOC unit bridge -------------------------------------------------
// StepperDriver computes duty = |U| / voltage_power_supply and treats U as a
// winding voltage. Here duty sets a CURRENT LIMIT, so declaring
// voltage_power_supply = the full-duty current in amps makes SimpleFOC's
// "volts" numerically EQUAL TO AMPS, and every voltage_limit reads directly as
// a current limit. See docs/measurements.md.
#define SIMPLEFOC_VPS   (FULL_DUTY_mA / 1000.0f)
#define AMPS(mA)        ((mA) / 1000.0f)
