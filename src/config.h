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

#define UART1_TX  PA9     // header P7, 38400 8N1 under factory firmware
#define UART1_RX  PA10
// SWD: PA13 = SWDIO, PA14 = SWCLK, on header P7. THE ONLY WAY IN. Do not remap.
// PB4/PB3/PA15 are JTAG pins out of reset -- the firmware frees them with
// SWJ NOJTAG, which keeps SW-DP alive. COIL_A_PWM is PB4, so this is required.

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
