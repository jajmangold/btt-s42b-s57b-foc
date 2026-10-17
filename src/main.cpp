// ---------------------------------------------------------------------------
// BIGTREETECH S42B / S57B V2.0 -- SimpleFOC firmware.
//
// UNITS: driver.voltage_power_supply is deliberately set to SIMPLEFOC_VPS
// (3.2), so every SimpleFOC "volt" IS an amp of coil current. voltage_limit
// therefore reads directly as a current limit. See config.h.
//
// SAFETY: FOC is off until commanded. Any energised state is bounded by
// DRIVE_TIMEOUT_MS. Current is capped at BRINGUP_mA, far below the driver's
// 3300 mA ceiling.
//
// Commands poked into g_cmd over SWD (no reflash per experiment):
//   3 = per-coil polarity probe     5 = initFOC (aligns, moves the shaft)
//   6 = closed-loop hold            7 = closed-loop +1 revolution
//   9 = all off
// ---------------------------------------------------------------------------
#include <Arduino.h>
#include <SimpleFOC.h>
#include "config.h"
#include "TLE5012B.h"
#include "A4950Driver.h"
#include "AxisCan.h"
#include "StepDir.h"
#include "CalStore.h"
#include "crc_probe.h"

static const uint32_t DRIVE_TIMEOUT_MS = 15000;

TLE5012B enc(ENC_CS);
StepDir  stepdir;
Commander command = Commander(Serial);
A4950Driver driver;                 // owns TIM3; see A4950Driver.h for why
AxisCan    can;
StepperMotor motor = StepperMotor(MOTOR_POLE_PAIRS);

struct Telem {
  uint32_t magic, seq;
  uint16_t raw, stat, raw_min, raw_max;
  uint32_t linkfail;
  float    angle, shaft, vel;
  uint32_t cmd, cmd_done;
  float    theta_e;
  int32_t  move_counts;
  float    expect_counts;
  uint16_t duty_amp, drive_mA;
  int32_t  probe[4];
  uint16_t set_mA, _pad;
  float    zero_elec;       // motor.zero_electric_angle after alignment
  int32_t  sensor_dir;      // motor.sensor_direction  (+1 / -1)
  float    target;          // commanded shaft angle, rad
  uint32_t foc_ready;       // 1 once initFOC has succeeded
  uint32_t can_ok;          // 1 once bxCAN reached normal/loopback mode
  uint32_t can_btr;         // raw BTR, so bit timing is verifiable over SWD
  uint32_t can_tx, can_rx;
  uint32_t can_err;         // TEC + REC
  uint32_t can_selftest;    // 1 = loopback frame went out and came back intact
  uint32_t crc_fails;       // safety-word CRC rejections
  uint32_t status_fails;    // safety-word status-bit rejections
  float    jog_vel;         // commanded jog velocity, rad/s
  uint32_t jog_stops;       // times the deadman watchdog halted a jog
  float    gear_k;          // gearing ratio, follower = k * master
  float    gear_master;     // last master position received, rad
  float    gear_err;        // current following error, rad
  float    gear_err_max;    // worst following error since engage, rad
  uint32_t gear_on;         // 1 = gearing engaged
  int32_t  step_count;      // accumulated STEP pulses, signed by DIR
  uint32_t step_raw;        // TIM2 counter, for checking the hardware directly
  uint32_t step_on;         // 1 = following STEP/DIR
  uint32_t home_raw;        // live sensor pin level, before debounce
  uint32_t home_trig;       // 1 = debounced sensor says triggered
  uint32_t home_state;      // 0 idle 1 seeking 2 backing off 3 homed 4 failed
  uint32_t homed;           // 1 = home_angle is valid
  float    home_angle;      // shaft angle, rad, at the moment of trigger
  uint32_t sysclk_hz;       // measured, not assumed
  uint32_t clock_ok;        // 1 = crystal came up and core is at 72 MHz
  float    nudge_rev;       // host-written: revolutions for command 16
  uint32_t cal_done;        // non-zero once the open-loop sweep has filled
  uint32_t cal_saved;       // 1 = last calSave() wrote and verified
  uint32_t cal_active;      // 1 = correction is being applied to every read
  float    nudge_vel;       // host-written traverse speed, rad/s. 0 = default.
  uint32_t safety;          // last TLE5012B safety word, for diagnosing rejects
};
volatile Telem g_telem __attribute__((used));

static uint32_t drive_started = 0;
static bool     energised = false;
static bool     foc_running = false;
static bool     jog_mode = false;        // velocity control rather than position
static uint32_t jog_deadline = 0;        // deadman: jog dies when this passes

// --- electronic gearing ----------------------------------------------------
// follower_target = shaft_at_engage + k * (master - master_at_engage)
//
// The follower tracks the master's MEASURED POSITION, not a time-based
// profile. That is the whole point: if a master update is late, the follower
// trails slightly and catches up, but the RATIO stays exact -- and the ratio
// is what sets the tooth flank. Timing jitter cannot corrupt the geometry.
//
// It is also inherently fail-safe: no fresh master position means the target
// simply stops advancing, so the follower holds rather than running away.
// SimpleFOC 2.4.0 has feed_forward_voltage/current but NO velocity
// feedforward, so its angle cascade can only ever lag a ramp by v/P_angle --
// measured 0.30 rad at 2 rad/s with P=8, exactly as theory predicts. So the
// gearing law is ours: run the motor in VELOCITY mode and command
//
//     v = k * master_velocity  +  GEAR_KP * position_error
//
// The feedforward term carries the motion; the error term only trims. That
// removes the structural lag instead of papering over it with a huge P.
static const float GEAR_KP = 25.0f;      // rad/s of correction per rad of error
static bool  step_mode = false;      // following step pulses from the host
static bool  gear_mode = false;
static float gear_shaft0 = 0.0f, gear_master0 = 0.0f;
static float step_shaft0 = 0.0f;   // shaft angle when STEP/DIR was engaged
static float gear_master_prev = 0.0f, gear_vel_f = 0.0f;
static uint32_t gear_t_prev = 0;

static void markEnergised();      // defined below

// Defined with the homing block below. allOff() is the single place every
// stop path converges on, so cancelling a seek has to happen here too --
// otherwise D0 stops the motor but leaves the state machine armed, and the
// next enable would quietly resume seeking.
static void homeCancel();

// Set the motion limits together, because they are not independent.
//
// P_angle's output IS the velocity setpoint, so its clamp must match
// velocity_limit. SimpleFOC sets P_angle.limit from velocity_limit once, in
// init(); changing velocity_limit later leaves the clamp behind. With an
// integral term that is a windup bug waiting to happen -- the integrator would
// be free to accumulate toward the stale, larger limit while the actual
// command is held at the smaller one, and every move would end with a wound-up
// integrator discharging into the next.
//
// The integral is also reset here. Carrying accumulated error from a finished
// move into a fresh one makes the axis lurch at the start of the new move,
// which on a machine tool is a surprise with a cutter attached.
static void setMotionLimits(float vel_rad_s, float amps) {
  motor.velocity_limit  = vel_rad_s;
  motor.P_angle.limit   = vel_rad_s;
  motor.voltage_limit   = amps;
  motor.PID_velocity.limit = amps;
  motor.P_angle.reset();
  motor.PID_velocity.reset();
}

static void allOff() {
  motor.disable();
  driver.setPwm(0, 0);
  digitalWrite(COIL_A_DIR_1, LOW); digitalWrite(COIL_A_DIR_2, LOW);
  digitalWrite(COIL_B_DIR_1, LOW); digitalWrite(COIL_B_DIR_2, LOW);
  energised = false;
  foc_running = false;
  jog_mode = false;
  gear_mode = false;
  step_mode = false;
  g_telem.step_on = 0;
  g_telem.gear_on = 0;
  g_telem.jog_vel = 0.0f;
  homeCancel();
  g_telem.drive_mA = 0;
}

// Follow step pulses from the host. Position control, with the host's step
// count mapped straight onto shaft angle. Engaging zeroes the step counter at
// the current position, so the drive does not lurch to wherever a stale count
// happened to point.
// ---------------------------------------------------------------------------
//  Shaft angle IN THE CONTROL FRAME -- use this for anything compared against
//  g_telem.target. Never enc.getAngle().
// ---------------------------------------------------------------------------
// SimpleFOC does not control on the raw encoder angle. It controls on
//
//     shaftAngle() = sensor_direction * sensor->getAngle() - sensor_offset
//
// and alignment sets sensor_direction to -1 whenever it finds the encoder
// counting the opposite way to the motor. On such a board the control frame is
// NEGATED relative to the encoder, so a target built from enc.getAngle() is in
// the wrong frame entirely.
//
// This is not theoretical. Measured on the ballscrew axis, which aligned to
// sensor_direction = -1: a request to move +1.257 rad produced a 6.9 rad
// position error and drove the axis BACKWARDS. It stopped only because the
// 500 mA nudge current was too weak to continue. At a working current it would
// have run until it hit something. The same bug sat in "hold position" and in
// the CAN TARGET handler, where it would have been worse.
//
// Before alignment sensor_direction is UNKNOWN, which is 0, so shaftAngle()
// would return ~0 and every relative move would start from the wrong origin.
// Fall back to the raw angle until FOC is ready; every caller that commands
// motion already requires foc_ready, so the fallback only feeds display paths.
static float shaftNow() {
  return g_telem.foc_ready ? motor.shaftAngle() : enc.getAngle();
}


static void stepDirEngage(float amps) {
  if (!g_telem.foc_ready) return;
  motor.controller = MotionControlType::angle;
  jog_mode = false;
  gear_mode = false;
  motor.voltage_limit = amps;
  stepdir.zero();
  step_shaft0 = shaftNow();
  g_telem.target = step_shaft0;
  step_mode = true;
  g_telem.step_on = 1;
  markEnergised();
  foc_running = true;
  motor.enable();
}

static void gearEngage(float k, float amps) {
  if (!g_telem.foc_ready) return;
  motor.controller = MotionControlType::velocity;   // we close position ourselves
  jog_mode = false;
  motor.voltage_limit = amps;
  g_telem.gear_k = k;
  gear_master_prev = g_telem.gear_master;
  gear_vel_f = 0.0f;
  gear_t_prev = micros();
  gear_shaft0  = shaftNow();
  gear_master0 = g_telem.gear_master;
  g_telem.target = gear_shaft0;
  g_telem.gear_err_max = 0.0f;
  gear_mode = true;
  g_telem.gear_on = 1;
  markEnergised();
  foc_running = true;
  motor.enable();
}

// Master position arrives here, whether from CAN or the bench test. Velocity
// is differentiated from successive positions and lightly filtered -- a raw
// difference over a 1 ms window is far too noisy to feed forward.
static void gearMaster(float master) {
  uint32_t now = micros();
  float dt = (float)(now - gear_t_prev) * 1e-6f;
  if (dt > 1e-5f && dt < 0.1f) {
    float v = (master - gear_master_prev) / dt;
    gear_vel_f += 0.15f * (v - gear_vel_f);
  }
  gear_master_prev = master;
  gear_t_prev = now;
  g_telem.gear_master = master;
  if (!gear_mode) return;
  g_telem.target = gear_shaft0 + g_telem.gear_k * (master - gear_master0);
}

// The gearing control law. Returns the velocity command.
static float gearVelocityCmd() {
  float err = g_telem.target - shaftNow();
  g_telem.gear_err = err;
  float a = err < 0 ? -err : err;
  if (a > g_telem.gear_err_max) g_telem.gear_err_max = a;
  return g_telem.gear_k * gear_vel_f + GEAR_KP * err;
}

// Enter velocity control at `v` rad/s and refresh the deadman. Called both by
// the CAN handler and the local bench test, so both share the same watchdog.
static void jogCommand(float v, float amps) {
  if (!g_telem.foc_ready) return;
  if (!jog_mode) {
    motor.controller = MotionControlType::velocity;
    jog_mode = true;
    motor.enable();
  }
  motor.voltage_limit = amps;
  g_telem.jog_vel = _constrain(v, -motor.velocity_limit, motor.velocity_limit);
  markEnergised();
  foc_running = true;
  jog_deadline = millis() + JOG_WATCHDOG_MS;
}

static void markEnergised() { drive_started = millis(); energised = true; }

// --- diagnostics, now routed through SimpleFOC's driver --------------------
// Kept from milestone 2 so a rewire can be re-verified without reflashing an
// older build. Uses setPwm, so it obeys the same unit bridge as the FOC path.
static int32_t pos_counts = 0;
static uint16_t last_raw = 0;
static void updatePos() {
  uint16_t r = enc.readRaw();
  int32_t d = (int32_t)r - (int32_t)last_raw;
  if (d >  16384) d -= 32768;
  if (d < -16384) d += 32768;
  pos_counts += d;
  last_raw = r;
  g_telem.raw = r;
  if (r < g_telem.raw_min) g_telem.raw_min = r;
  if (r > g_telem.raw_max) g_telem.raw_max = r;
}

static int32_t parkAndRead(float ua, float ub, uint32_t settle_ms) {
  markEnergised();
  driver.setPwm(ua, ub);
  uint32_t t0 = millis();
  while (millis() - t0 < settle_ms) { updatePos(); delay(1); }
  updatePos();
  return pos_counts;
}

// Open-loop sweep buffer (diagnostic, command 19). 200 full steps is one
// mechanical revolution on a 1.8 deg/step motor.
// 800 points over one mechanical revolution: 0.45 deg of field per step, and
// bidirectionally that is ~6 samples per correction bucket rather than ~3. The
// limit on this table is measurement noise, not bucket count, so spending the
// RAM on more samples per bucket beats spending it on more buckets.
static const uint16_t CAL_STEPS     = 800;
static const uint32_t CAL_SETTLE_MS = 60;
// volatile, and deliberately so: nothing in the firmware ever reads this back,
// so without it the compiler correctly proves the stores are dead and removes
// the array entirely. It is read over SWD, which the compiler cannot see.
// TWO sweeps: forward then reverse, stored back to back. Averaging them
// cancels hysteresis. A single-direction sweep records where friction let the
// rotor settle, not where the field actually pointed, so it measures stiction
// alongside encoder error -- and on this axis that noise was large enough that
// successive sweeps disagreed by 35 electrical degrees and a table built from
// one bad pass stalled the motor completely.
static volatile uint16_t cal_raw[CAL_STEPS * 2];

static void probeCoils(float amps) {
  g_telem.drive_mA = (uint16_t)(amps * 1000);
  g_telem.probe[0] = parkAndRead( amps, 0, 700);
  g_telem.probe[1] = parkAndRead(-amps, 0, 700);
  g_telem.probe[2] = parkAndRead(0,  amps, 700);
  g_telem.probe[3] = parkAndRead(0, -amps, 700);
  allOff();
}

// Broadcast one state frame. 8 bytes: angle, velocity, flags, sequence.
static void canSendState() {
  uint8_t d[8];
  float a = g_telem.shaft;
  memcpy(d, &a, 4);
  int16_t v = (int16_t)(g_telem.vel * 100.0f);
  d[4] = (uint8_t)(v & 0xFF);
  d[5] = (uint8_t)(v >> 8);
  d[6] = (g_telem.foc_ready ? CAN_FLAG_FOC_READY : 0)
       | (energised        ? CAN_FLAG_ENERGISED : 0)
       | (g_telem.linkfail ? CAN_FLAG_LINK_FAIL : 0);
  d[7] = (uint8_t)g_telem.seq;
  can.send(CAN_ID_OF(canNodeId(), CAN_MSG_STATE), d, 8);
}

// Act on a host frame. Addressing is in software so more axes drop in later.
static void canHandle(const CanFrame& f) {
  // Act on frames addressed to us, and on group frames. Replies and telemetry
  // always go out under our OWN address below -- a node must never transmit
  // from the group address, or two drives would answer as one.
  const uint8_t addr = CAN_ADDR_OF(f.id);
  if (addr != canNodeId() && addr != CAN_ADDR_BROADCAST) return;
  switch (CAN_TYPE_OF(f.id)) {
    case CAN_CMD_ENABLE:
      if (f.len >= 1 && f.data[0] && g_telem.foc_ready) {
        g_telem.target = shaftNow();
        markEnergised(); foc_running = true; motor.enable();
      } else {
        allOff();
      }
      break;
    case CAN_CMD_TARGET:
      if (f.len >= 4 && g_telem.foc_ready) {
        float t; memcpy(&t, f.data, 4);
        g_telem.target = t;
        markEnergised(); foc_running = true; motor.enable();
      }
      break;
    case CAN_CMD_CURRENT:
      if (f.len >= 2) {
        uint16_t ma = (uint16_t)(f.data[0] | (f.data[1] << 8));
        if (ma > CURRENT_CEILING_mA) ma = CURRENT_CEILING_mA;  // never trust the bus
        g_telem.set_mA = ma;
        motor.voltage_limit = AMPS(ma);
      }
      break;
    case CAN_CMD_JOG:
      if (f.len >= 4) {
        float v; memcpy(&v, f.data, 4);
        jogCommand(v, AMPS(g_telem.set_mA ? g_telem.set_mA : BRINGUP_mA));
      }
      break;
    case CAN_CMD_GEAR_K:
      if (f.len >= 4) {
        float k; memcpy(&k, f.data, 4);
        gearEngage(k, AMPS(g_telem.set_mA ? g_telem.set_mA : BRINGUP_mA));
      }
      break;
    case CAN_CMD_GEAR_X:
      if (f.len >= 4) {
        float x; memcpy(&x, f.data, 4);
        gearMaster(x);
      }
      break;
    case CAN_CMD_STEPDIR:
      if (f.len >= 1 && f.data[0])
        stepDirEngage(AMPS(g_telem.set_mA ? g_telem.set_mA : BRINGUP_mA));
      else
        allOff();
      break;
    case CAN_CMD_PING:
      can.send(CAN_ID_OF(canNodeId(), CAN_MSG_PONG), BOARD_TAG, 4);
      break;
    default: break;
  }
}

// ---------------------------------------------------------------------------
// UART control, on P7 at 38400 8N1.
//
// SimpleFOC's Commander provides the standard motor verbs -- target, PID
// gains, limits, monitoring -- so anyone who has used SimpleFOC already knows
// this interface and existing tooling works unchanged. 'D' adds the drive
// modes this firmware provides on top.
//
// Safety note: Commander cannot be used to exceed the board's rating. Duty is
// |U| / voltage_power_supply and voltage_power_supply IS the full-duty
// current, so the hardware bound holds whatever a user types.
// ---------------------------------------------------------------------------
static void onMotor(char* cmd) { command.motor(&motor, cmd); }

// ---------------------------------------------------------------------------
//  Home sensor and homing
// ---------------------------------------------------------------------------
// Homing is a STATE MACHINE driven from the main loop, not a blocking routine.
// That is not a style preference: loopFOC() has to keep running at full rate
// for the motor to stay under control, so a seek that blocked inside a command
// handler would stall the very control loop holding the axis.

// Stages: FAST finds the sensor quickly, RETREAT backs clear of it, SEEK
// re-approaches slowly and is the one that records the reference, BACKOFF
// parks clear so the next home can start.
enum : uint8_t { HOME_IDLE = 0, HOME_SEEK, HOME_BACKOFF, HOME_DONE, HOME_FAIL,
                 HOME_FAST, HOME_RETREAT };

static uint8_t home_state  = HOME_IDLE;
static float   home_start  = 0.0f;    // shaft angle when the seek began
static float   home_dir    = 1.0f;    // +1 or -1
static float   home_vel    = 0.0f;    // commanded seek velocity, rad/s
static bool    home_released = false; // sensor let go during backoff
static float   home_rel_ang  = 0.0f;  // shaft angle at the release point
static float   home_still_at = 0.0f;  // position when motion last progressed
static uint32_t home_still_ms = 0;    // ...and when that was
static uint8_t home_count  = 0;       // debounce accumulator
static bool    home_level  = false;   // debounced sensor state

static void homeSensorInit() {
  pinMode(HOME_SENSOR_PIN, INPUT_PULLUP);
}

// Debounce by agreement: a new level is only accepted after it has been read
// HOME_DEBOUNCE_SAMPLES times running. Called every loop, so this is a few
// hundred microseconds of filtering, not a delay.
static void homeSensorPoll() {
  int raw = digitalRead(HOME_SENSOR_PIN);
  g_telem.home_raw = (uint32_t)raw;
#if HOME_SENSOR_ACTIVE_LOW
  bool now = (raw == LOW);
#else
  bool now = (raw == HIGH);
#endif
  if (now == home_level) { home_count = 0; }
  else if (++home_count >= HOME_DEBOUNCE_SAMPLES) { home_level = now; home_count = 0; }
  g_telem.home_trig = home_level ? 1 : 0;
}

static bool homeTriggered() { return home_level; }

// True once the axis has stopped making progress for HOME_STALL_MS while a
// seek is commanding motion. At full seek current this is the thing bounding
// how hard a missed trigger can push, so it matters more than the travel limit.
static bool homeStalled(float pos) {
  if (fabsf(pos - home_still_at) > HOME_STALL_RAD) {
    home_still_at = pos;
    home_still_ms = millis();
    return false;
  }
  return (millis() - home_still_ms) > HOME_STALL_MS;
}

// Drop out of any in-flight seek. Callers that want a specific terminal state
// (HOME_DONE, HOME_FAIL) set it after their allOff().
static void homeCancel() {
  home_vel = 0.0f;
  home_released = false;
  if (home_state == HOME_SEEK || home_state == HOME_BACKOFF) {
    home_state = HOME_IDLE;
    g_telem.home_state = HOME_IDLE;
  }
}

static void homeAbort(const __FlashStringHelper* why) {
  home_vel = 0.0f;
  allOff();
  home_state = HOME_FAIL;
  g_telem.home_state = HOME_FAIL;
  Serial.print(F("home FAILED: ")); Serial.println(why);
}

static void homeStart(float dir_vel) {
  if (!g_telem.foc_ready) { Serial.println(F("align first: D5")); return; }
  if (homeTriggered()) {
    // Already on the sensor. Seeking further in would only drive deeper into
    // whatever the sensor marks, so refuse and let the operator back it off.
    Serial.println(F("home refused: sensor already triggered, move clear first"));
    return;
  }
  home_dir   = (dir_vel < 0) ? -1.0f : 1.0f;
  home_start = shaftNow();
  g_telem.homed = 0;

  // Deliberately weak: a missed trigger should stall, not break something.
  // setMotionLimits() rather than setting voltage_limit alone, so the velocity
  // clamp and both PID limits belong to the seek instead of being inherited
  // from whatever mode ran last.
  // Seek current is host-settable via set_mA so it can be characterised without
  // a reflash. It matters more than it looks: PID_velocity's output limit IS
  // the current limit, so raising the current raises the loop's authority --
  // and this loop is not tuned for an arbitrary one.
  uint16_t seek_ma = g_telem.set_mA ? g_telem.set_mA : HOME_SEEK_mA;
  if (seek_ma > CURRENT_CEILING_mA) seek_ma = CURRENT_CEILING_mA;
  g_telem.drive_mA = seek_ma;
  setMotionLimits(HOME_FAST_VEL_RAD_S, AMPS(seek_ma));
  motor.controller    = MotionControlType::velocity;
  jog_mode = gear_mode = step_mode = false;
  markEnergised(); foc_running = true; motor.enable();
  home_vel = home_dir * HOME_FAST_VEL_RAD_S;

  home_released = false;
  home_still_at = home_start;
  home_still_ms = millis();
  home_state = HOME_FAST;
  g_telem.home_state = HOME_FAST;
  Serial.print(F("homing ")); Serial.print(home_dir > 0 ? F("+") : F("-"));
  Serial.print(F(" at ")); Serial.print(g_telem.drive_mA); Serial.print(F(" mA, "));
  Serial.print(RAD_TO_MM(HOME_SEEK_VEL_RAD_S), 2); Serial.print(F(" mm/s, abort after "));
  Serial.print(RAD_TO_MM(HOME_MAX_TRAVEL_RAD), 1); Serial.println(F(" mm"));
}

static void homeUpdate() {
  if (home_state == HOME_IDLE || home_state == HOME_DONE || home_state == HOME_FAIL) return;

  // Keep the drive watchdog fed while a seek is genuinely in progress.
  //
  // DRIVE_TIMEOUT_MS exists to stop the motor being left energised and
  // forgotten, and it is 15 s. A seek at HOME_SEEK_VEL_RAD_S can legitimately
  // need 125 s to cover HOME_MAX_TRAVEL_RAD, so without this the watchdog
  // always wins and homing can never complete -- observed as the state machine
  // dropping to idle after exactly 15 s, having found nothing.
  //
  // This does not weaken the watchdog: a seek is bounded by its own travel
  // limit and ends on its own, and every other path still times out normally.
  markEnergised();
  float pos = shaftNow();
  float travelled = pos - home_start;

  // Any stage that commands motion must give up if motion stops. At full seek
  // current this is what keeps a missed trigger from driving into a hard stop.
  if (home_state != HOME_RETREAT && homeStalled(pos)) {
    homeAbort(F("axis stopped moving -- obstruction, or a sensor that never fired"));
    return;
  }

  // --- stage 1: fast coarse search ----------------------------------------
  if (home_state == HOME_FAST) {
    if (homeTriggered()) {
      // Found it, but at speed the trip point is blurred by detection latency.
      // Retreat clear and let the slow stage make the actual measurement.
      setMotionLimits(HOME_FAST_VEL_RAD_S, AMPS(g_telem.drive_mA));
      home_vel      = -home_dir * HOME_FAST_VEL_RAD_S;
      home_start    = pos;
      home_still_at = pos;
      home_still_ms = millis();
      home_state = HOME_RETREAT;
      g_telem.home_state = HOME_RETREAT;
      return;
    }
    if (fabsf(travelled) > HOME_MAX_TRAVEL_RAD)
      homeAbort(F("no trigger within the travel limit -- sensor wiring?"));
    return;
  }

  // --- stage 2: retreat clear of the sensor --------------------------------
  if (home_state == HOME_RETREAT) {
    if (!homeTriggered() && fabsf(travelled) >= HOME_RESEEK_BACKOFF_RAD) {
      setMotionLimits(HOME_SEEK_VEL_RAD_S, AMPS(g_telem.drive_mA));
      home_vel      = home_dir * HOME_SEEK_VEL_RAD_S;
      home_start    = pos;
      home_still_at = pos;
      home_still_ms = millis();
      home_state = HOME_SEEK;
      g_telem.home_state = HOME_SEEK;
      return;
    }
    if (fabsf(travelled) > HOME_RESEEK_BACKOFF_RAD * 4.0f)
      homeAbort(F("sensor never released while retreating"));
    return;
  }

  if (home_state == HOME_SEEK) {
    if (homeTriggered()) {
      g_telem.home_angle = shaftNow();
      g_telem.homed      = 1;
      // Back off far enough to release the sensor, so a subsequent home starts
      // from a known-clear position rather than from on top of the trigger.
      home_vel     = -home_dir * HOME_SEEK_VEL_RAD_S;
      home_start   = shaftNow();
      home_state   = HOME_BACKOFF;
      g_telem.home_state = HOME_BACKOFF;
      Serial.print(F("home found at ")); Serial.print(g_telem.home_angle, 4);
      Serial.println(F(" rad"));
      return;
    }
    // The sensor may be unplugged, or the magnet may have moved. Either way,
    // stop before the mechanics do it for us.
    // The slow stage only has to recover the retreat distance, so its bound is
    // tight: if the sensor fired once at speed and not again on the careful
    // approach, something is wrong and it must not keep going.
    if (fabsf(travelled) > HOME_RESEEK_BACKOFF_RAD * 3.0f) {
      homeAbort(F("sensor did not fire again on the slow approach"));
    }
    return;
  }

  // HOME_BACKOFF -- reverse until the sensor RELEASES, then a margin past it.
  if (!home_released) {
    if (!homeTriggered()) {
      home_released = true;
      home_rel_ang  = shaftNow();
      Serial.print(F("sensor released after "));
      Serial.print(fabsf(RAD_TO_MM(travelled)), 2); Serial.println(F(" mm"));
    } else if (fabsf(travelled) > HOME_BACKOFF_MAX_RAD) {
      // Still triggered after a sane backoff: the sensor is stuck on, or the
      // magnet is far larger than assumed. Either way, stop rather than keep
      // reversing on the strength of a reading we no longer trust.
      homeAbort(F("sensor never released during backoff"));
    }
    return;
  }
  if (fabsf(shaftNow() - home_rel_ang) >= HOME_BACKOFF_EXTRA_RAD) {
    home_vel = 0.0f;
    allOff();
    home_state = HOME_DONE;
    g_telem.home_state = HOME_DONE;
    Serial.print(F("home done at ")); Serial.print(g_telem.home_angle, 4);
    Serial.println(F(" rad, axis off and clear of the sensor"));
  }
}

static void onDrive(char* cmd) {
  float amps = AMPS(g_telem.set_mA ? g_telem.set_mA : BRINGUP_mA);
  switch (cmd[0]) {
    case '0': allOff();                     Serial.println(F("off")); break;
    case '1': if (!g_telem.foc_ready) { Serial.println(F("align first: D5")); break; }
              motor.controller = MotionControlType::angle;
              jog_mode = gear_mode = step_mode = false;
              motor.voltage_limit = amps;
              g_telem.target = shaftNow();
              markEnergised(); foc_running = true; motor.enable();
              Serial.println(F("hold")); break;
    case '2': stepDirEngage(amps);          Serial.println(F("step/dir")); break;
    case '3': gearEngage(atof(cmd + 1), amps);
              Serial.print(F("gear k=")); Serial.println(g_telem.gear_k, 4); break;
    case '4': jogCommand(atof(cmd + 1), amps);
              Serial.print(F("jog ")); Serial.println(g_telem.jog_vel, 3); break;
    case '5': markEnergised(); motor.initFOC();
              g_telem.zero_elec  = motor.zero_electric_angle;
              g_telem.sensor_dir = (int32_t)motor.sensor_direction;
              g_telem.foc_ready  = (motor.motor_status == FOCMotorStatus::motor_ready) ? 1 : 0;
              allOff();
              Serial.print(F("align ")); Serial.println(g_telem.foc_ready ? F("ok") : F("FAILED"));
              break;
    case 'S':   // sensor status -- reads only, never moves the motor
      Serial.print(F("home sensor: pin=")); Serial.print(g_telem.home_raw);
      Serial.print(F(" -> ")); Serial.print(homeTriggered() ? F("TRIGGERED") : F("clear"));
      Serial.print(F("  homed=")); Serial.print(g_telem.homed);
      if (g_telem.homed) { Serial.print(F(" at ")); Serial.print(g_telem.home_angle, 4);
                           Serial.print(F(" rad")); }
      Serial.println();
      break;
    case 'H':   // MOVES THE MOTOR. sign of the argument picks the direction.
      homeStart(atof(cmd + 1));
      break;
    case 'C': { int ma = atoi(cmd + 1);
                if (ma > CURRENT_CEILING_mA) ma = CURRENT_CEILING_mA;  // never trust input
                if (ma > 0) g_telem.set_mA = ma;
                Serial.print(F("current ")); Serial.print(g_telem.set_mA
                  ? g_telem.set_mA : BRINGUP_mA); Serial.println(F(" mA")); break; }
    case '?':
    default:
      Serial.println(F("D0 off | D1 hold | D2 step/dir | D3<k> gear | D4<v> jog"));
      Serial.println(F("D5 align | DC<mA> current | D? help"));
      Serial.println(F("DS sensor status | DH<+/-v> home seek (MOVES THE AXIS)"));
      Serial.print(F("  ready="));   Serial.print(g_telem.foc_ready);
      Serial.print(F(" shaft="));    Serial.print(shaftNow(), 3);
      Serial.print(F(" rad  mA="));  Serial.print(g_telem.set_mA ? g_telem.set_mA : BRINGUP_mA);
      Serial.print(F(" faults="));   Serial.print(g_telem.linkfail + g_telem.crc_fails);
      Serial.print(F(" steps="));    Serial.println(g_telem.step_count);
      break;
  }
}

void setup() {
  // PB4/PB3/PA15 are JTAG pins at reset. NOJTAG frees them and keeps SW-DP,
  // so the ST-Link stays alive. Must precede any pin setup. See CHANGELOG.
  // ---- CLAIM AND SAFE THE MOTOR PINS BEFORE ANYTHING ELSE ---------------
  // Order matters and is not obvious: PB4 is NJTRST and belongs to the JTAG
  // peripheral out of reset, so it cannot be driven -- or even safed -- until
  // AFIO_MAPR releases it. a4950_claim_pins() does the remap and then parks
  // all six motor pins in coast, in that order. Everything after this point
  // (Serial.begin, its delay, encoder init) takes well over 200 ms, and the
  // coils must not be floating for that window.
  a4950_claim_pins();

  // Only now can the DIP switches be read: two of the four are JTAG pins until
  // the remap above frees them. This sets the CAN node address.
  canNodeIdInit();
  homeSensorInit();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  Serial.begin(38400);
  delay(200);
  Serial.println();
  Serial.print(F("=== ")); Serial.print(F(BOARD_NAME));
  Serial.println(F(" : SimpleFOC firmware, idle ==="));
  Serial.print(F("CAN node ")); Serial.print(canNodeId());
  Serial.print(F("  (DIP ")); Serial.print(canNodeDipBits(), BIN);
  Serial.println(F(", group address 0 also accepted)"));

  command.add('M', onMotor, "motor");
  command.add('D', onDrive, "drive mode");
  Serial.println(F("UART control ready. 'D?' for help, 'M' for SimpleFOC motor verbs."));

  enc.init();
  // Clear any latched sensor error from power-up before anything consumes an
  // angle. Without this a latched flag makes every validated read fail, the
  // angle freezes at its first value, and the drive looks like a mechanical
  // fault rather than a status bit.
  enc.clearStatus();

  // Load the encoder correction table before anything can command motion.
  // Without it this drive may not turn at all -- 50 pole pairs multiply the
  // part's angle error into electrical degrees. See docs/measurements.md.
  if (calLoad()) {
    Serial.println(F("encoder calibration: loaded from flash, correction ON"));
  } else {
    Serial.println(F("encoder calibration: NONE STORED -- run tools/calibrate.py"));
  }
  stepdir.init();
  g_telem.raw_min = 0xFFFF;
  g_telem.raw_max = 0;
  g_telem.stat    = enc.readStatus();
  last_raw        = enc.readRaw();

  // "volts" == amps, see config.h
  driver.voltage_power_supply = SIMPLEFOC_VPS;
  driver.voltage_limit        = AMPS(CURRENT_CEILING_mA);
  driver.init();

  Serial.print(F("PWM carrier = ")); Serial.print(driver.carrierHz() / 1000);
  Serial.println(F(" kHz"));

  motor.linkDriver(&driver);
  motor.linkSensor(&enc);
  motor.voltage_limit   = AMPS(BRINGUP_mA);     // 0.6 "V" == 600 mA
  motor.velocity_limit  = 20.0f;                // rad/s, shaft
  motor.controller      = MotionControlType::angle;
  motor.foc_modulation  = FOCModulationType::SinePWM;

  // 6 counts of jitter differentiated at ~14 kHz is +-1.4 rad/s of phantom
  // velocity on a stationary shaft. Filter it or the velocity loop chases noise.
  motor.LPF_velocity.Tf = 0.02f;
  motor.P_angle.P       = ANGLE_PID_P;
  motor.P_angle.I       = ANGLE_PID_I;
  motor.PID_velocity.P  = 0.05f;
  motor.PID_velocity.I  = 1.0f;
  motor.PID_velocity.D  = 0.0f;

  motor.init();
  allOff();                                    // never idle energised
  extern bool clockIsCorrect();
  g_telem.sysclk_hz = HAL_RCC_GetSysClockFreq();
  g_telem.clock_ok  = clockIsCorrect() ? 1 : 0;
  Serial.print(F("SYSCLK ")); Serial.print(g_telem.sysclk_hz / 1000000);
  Serial.println(g_telem.clock_ok ? F(" MHz (crystal)") : F(" MHz -- HSE FAILED, CAN/UART rates affected"));

  // Apply the power-up current before anything can command motion.
  g_telem.set_mA = DEFAULT_RUN_mA;
  Serial.print(F("run current ")); Serial.print(DEFAULT_RUN_mA);
  Serial.print(F(" mA (ceiling ")); Serial.print(CURRENT_CEILING_mA);
  Serial.println(F(" mA)"));

  g_telem.magic = 0x53354237;
}

void loop() {
  if (!foc_running) { enc.update(); updatePos(); }
  // During FOC, loopFOC() already read the sensor -- reuse that value instead
  // of issuing another SPI transaction. Previously raw froze during FOC and
  // made the hold look perfect when it was simply not being sampled.
  if (foc_running) {
    uint16_t r = enc.lastRaw();
    g_telem.raw = r;
    if (r < g_telem.raw_min) g_telem.raw_min = r;
    if (r > g_telem.raw_max) g_telem.raw_max = r;
  }
  // These were declared and never assigned, so they read 0 on every board and
  // every run -- which looks exactly like a healthy encoder. That dead
  // telemetry cost a wrong diagnosis: a sweep returning a constant angle was
  // read as "reads are fine, the rotor must not be turning", when the counters
  // simply had no way to say otherwise.
  g_telem.safety       = enc.lastSafety();
  g_telem.crc_fails    = enc.crcFails();
  g_telem.status_fails = enc.statusFails();
  g_telem.linkfail     = enc.linkFails();
  g_telem.cal_active = tle_cal_enable ? 1 : 0;
  g_telem.angle = enc.getAngle();      // multi-turn, no extra bus traffic
  g_telem.shaft = shaftNow();   // CONTROL frame; .angle stays raw
  g_telem.vel   = enc.getVelocity();
  g_telem.seq++;

  if (energised && millis() - drive_started > DRIVE_TIMEOUT_MS) allOff();

  uint32_t c = g_telem.cmd;
  // Writing 0 re-arms the dispatcher. Without this, resending the SAME command
  // is silently ignored because cmd_done still matches -- which made a
  // repeatability test look like the motor had stopped moving entirely.
  if (c == 0) g_telem.cmd_done = 0;
  if (c && c != g_telem.cmd_done) {
    float amps = AMPS(g_telem.set_mA ? g_telem.set_mA : BRINGUP_mA);
    g_telem.raw_min = 0xFFFF; g_telem.raw_max = 0;   // fresh window per test
    switch (c) {
      case 3:
        probeCoils(amps);
        break;
      case 5:                                   // align: finds zero electric angle
        motor.voltage_limit = amps;
        markEnergised();
        motor.initFOC();
        g_telem.zero_elec  = motor.zero_electric_angle;
        g_telem.sensor_dir = (int32_t)motor.sensor_direction;
        g_telem.foc_ready  = (motor.motor_status == FOCMotorStatus::motor_ready) ? 1 : 0;
        allOff();
        break;
      case 14: {                                // gearing test, synthetic master
        // Ramp a fake master position and watch the follower track it. Proves
        // the gearing maths and the servo response without a second axis.
        // Zero the master BEFORE engaging. gearEngage() captures master0 from
        // the current value, and the ramp below restarts at 0 -- so a stale
        // master from a previous run would be seen as an instantaneous step of
        // the whole previous travel. That spike is a test artifact, not a
        // control failure, and it masked the real transient until it was fixed.
        g_telem.gear_master = 0.0f;
        gear_master_prev    = 0.0f;
        gearEngage(1.0f, amps);
        const float rate = 2.0f;                // master rad/s
        const float secs = 3.0f;
        uint32_t t0 = millis();
        uint32_t last = t0;
        while (millis() - t0 < (uint32_t)(secs * 1000)) {
          uint32_t now = millis();
          if (now != last) {                    // ~1 kHz, like a CAN broadcast
            last = now;
            gearMaster((now - t0) * 0.001f * rate);
          }
          enc.update();
          motor.loopFOC();
          motor.move(gearVelocityCmd());
          if (millis() - drive_started > DRIVE_TIMEOUT_MS) break;
        }
        allOff();
        break;
      }
      case 13:                                  // bench jog test, no CAN needed
        jogCommand(2.0f, amps);                 // 2 rad/s, deadman still applies
        break;
      case 6:                                   // hold where it currently sits
        if (g_telem.foc_ready) {
          motor.controller = MotionControlType::angle; jog_mode = false;
          motor.voltage_limit = amps;
          g_telem.target = shaftNow();
          markEnergised(); foc_running = true; motor.enable();
        }
        break;
      case 21:   // run the homing seek. Direction from the sign of nudge_rev.
        homeStart(g_telem.nudge_rev < 0 ? -1.0f : 1.0f);
        break;
      case 20:   // persist the current correction table to flash
        g_telem.cal_saved = calSave() ? 1 : 0;
        break;
      case 19: {
        // DIAGNOSTIC: open-loop electrical sweep. Steps the field through
        // CAL_STEPS positions spanning one full MECHANICAL revolution, with no
        // feedback at all, and records what the encoder reads at each.
        //
        // This is the measurement that separates the two remaining stories. If
        // the encoder is sound, its reading against commanded position is a
        // straight line. If the magnet or its mounting is off, the line bends,
        // and with 50 pole pairs even a degree of bend is 50 degrees of
        // electrical error -- enough to stall commutation at some positions
        // while leaving open-loop parking, which never consults the encoder,
        // looking perfect.
        float a = amps;
        // forward, then reverse over the same points
        for (uint16_t n = 0; n < CAL_STEPS * 2; n++) {
          uint16_t k = (n < CAL_STEPS) ? n : (CAL_STEPS * 2 - 1 - n);
          float elec = (float)k * _2PI * (float)MOTOR_POLE_PAIRS / (float)CAL_STEPS;
          parkAndRead(a * _cos(elec), a * _sin(elec), CAL_SETTLE_MS);
          // Record through getSensorAngle(), NOT readRaw(): readRaw bypasses
          // the correction table, so a verification sweep taken that way is
          // blind to the very thing it is meant to measure. With the table
          // disabled the two agree, so one path serves both roles.
          cal_raw[n] = (uint16_t)(lroundf(enc.getSensorAngle() * 32768.0f / _2PI) & 0x7FFF);
        }
        allOff();
        g_telem.cal_done = CAL_STEPS * 2;
        break;
      }
      case 17:   // DIAGNOSTIC: force sensor_direction, then align.
      case 18: {
        // Presetting sensor_direction before initFOC() makes SimpleFOC skip
        // its own direction detection and keep the value given, while still
        // searching for the electrical zero. Used to test whether automatic
        // detection picked the wrong sign -- an inverted feedback sign makes
        // FOC push the rotor the wrong way, so the error grows and the axis
        // locks solid after a short move, which is exactly the symptom here.
        motor.sensor_direction = (c == 17) ? Direction::CW : Direction::CCW;
        motor.voltage_limit = amps;
        markEnergised();
        motor.initFOC();
        g_telem.zero_elec  = motor.zero_electric_angle;
        g_telem.sensor_dir = (int32_t)motor.sensor_direction;
        g_telem.foc_ready  = (motor.motor_status == FOCMotorStatus::motor_ready) ? 1 : 0;
        allOff();
        break;
      }
      case 16: {
        // Bounded relative move, both directions, for positioning by hand over
        // SWD -- creeping a gantry up to a sensor, for instance. The distance
        // comes from g_telem.nudge_rev, written by the host before the command,
        // and is CLAMPED here: a host that writes a wild value, or forgets to
        // write one at all, must not launch the axis across the machine.
        if (!g_telem.foc_ready) break;
        float rev = g_telem.nudge_rev;
        if (!(rev == rev)) rev = 0.0f;                       // NaN guard
        if (rev >  NUDGE_MAX_REV) rev =  NUDGE_MAX_REV;
        if (rev < -NUDGE_MAX_REV) rev = -NUDGE_MAX_REV;
        if (rev == 0.0f) break;
        motor.controller    = MotionControlType::angle;
        jog_mode = gear_mode = step_mode = false;
        // Traverse speed and current are both host-settable, because
        // positioning by hand near something fragile and crossing half a metre
        // of empty axis want very different numbers. Both are clamped: the
        // point of this command is that a bad value cannot launch the axis.
        float v = g_telem.nudge_vel;
        if (!(v == v) || v <= 0.0f) v = NUDGE_VEL_RAD_S;      // NaN or unset
        if (v > NUDGE_MAX_VEL_RAD_S) v = NUDGE_MAX_VEL_RAD_S;
        uint16_t ma = g_telem.set_mA ? g_telem.set_mA : NUDGE_mA;
        if (ma > CURRENT_CEILING_mA) ma = CURRENT_CEILING_mA;
        setMotionLimits(v, AMPS(ma));
        g_telem.target = shaftNow() + rev * _2PI;
        markEnergised(); foc_running = true; motor.enable();
        break;
      }
      case 7:                                   // closed-loop +1 shaft revolution
        if (g_telem.foc_ready) {
          motor.controller = MotionControlType::angle; jog_mode = false;
          motor.voltage_limit = amps;
          g_telem.target = shaftNow() + _2PI;
          markEnergised(); foc_running = true; motor.enable();
        }
        break;
      case 10: {                                // CAN self-test, LOOPBACK
        g_telem.can_ok = can.init(500000, true) ? 1 : 0;
        g_telem.can_btr = can.btr();
        g_telem.can_selftest = 0;
        if (g_telem.can_ok) {
          const uint8_t probe[8] = {0xDE,0xAD,0xBE,0xEF,0x01,0x02,0x03,0x04};
          uint16_t id = CAN_ID_OF(canNodeId(), CAN_MSG_STATE);
          can.send(id, probe, 8);
          uint32_t t0 = millis();
          CanFrame f;
          while (millis() - t0 < 50) {
            if (can.receive(&f)) {
              bool ok = (f.id == id) && (f.len == 8);
              for (int i = 0; i < 8 && ok; i++) ok = (f.data[i] == probe[i]);
              g_telem.can_selftest = ok ? 1 : 2;    // 2 = came back corrupted
              break;
            }
          }
        }
        g_telem.can_tx = can.txCount();
        g_telem.can_rx = can.rxCount();
        g_telem.can_err = can.errCount();
        break;
      }
      case 11:                                   // CAN live on the bus
        g_telem.can_ok = can.init(500000, false) ? 1 : 0;
        g_telem.can_btr = can.btr();
        break;
      case 12: {                                // which CRC does the part use?
        // Reuse probe[0..3] as match counters for the four variants and
        // move_counts as the trial total -- no struct change needed.
        for (int i = 0; i < 4; i++) g_telem.probe[i] = 0;
        int trials = 0, status_ok = 0;
        for (int n = 0; n < 200; n++) {
          uint16_t d, sw;
          if (!enc.readSafe(TLE_CMD_READ_ANGLE, &d, &sw)) continue;
          trials++;
          if (TLE5012B::statusOk(sw)) status_ok++;
          uint8_t cand[4];
          crc_variants(TLE_CMD_READ_ANGLE, d, cand);
          uint8_t got = (uint8_t)(sw & 0xFF);
          for (int i = 0; i < 4; i++) if (cand[i] == got) g_telem.probe[i]++;
          delay(2);
        }
        g_telem.move_counts   = trials;
        g_telem.expect_counts = (float)status_ok;
        break;
      }
      case 15:                                  // follow STEP/DIR from P1
        stepDirEngage(amps);
        break;
      case 9:
      default:
        allOff();
        break;
    }
    g_telem.cmd_done = c;
  }

  if (jog_mode && (int32_t)(millis() - jog_deadline) >= 0 && g_telem.jog_vel != 0.0f) {
    // DEADMAN EXPIRED: no fresh jog command arrived. Stop, and count it so a
    // flaky link shows up as a number instead of as mystery stutter.
    g_telem.jog_vel = 0.0f;
    g_telem.jog_stops++;
  }

  if (step_mode) {
    stepdir.update();
    g_telem.step_count = stepdir.steps();
    g_telem.step_raw   = stepdir.rawCount();
    if (stepdir.hostEnabled()) {
      g_telem.target = step_shaft0 + stepdir.angle();
    } else {
      // Host has released the drive. Stop holding, but keep following the
      // counter so position is still correct when it enables again.
      allOff();
    }
  }

  homeSensorPoll();
  homeUpdate();

  if (foc_running) {
    // Conditional integration: the angle integral is enabled only close to the
    // target. See ANGLE_I_BAND_RAD in config.h for why.
    if (motor.controller == MotionControlType::angle) {
      float err = fabsf(g_telem.target - shaftNow());
      motor.P_angle.I = (err < ANGLE_I_BAND_RAD) ? ANGLE_PID_I : 0.0f;
    }
    motor.loopFOC();
    // EVERY homing stage that commands motion must be listed here. Miss one
    // and the loop falls through to motor.move(g_telem.target) -- a stale
    // position target from whatever ran last -- so the axis drives off to an
    // unrelated place at full authority. That is exactly what adding the fast
    // and retreat stages without updating this line did.
    bool homing = (home_state == HOME_FAST    || home_state == HOME_RETREAT ||
                   home_state == HOME_SEEK    || home_state == HOME_BACKOFF);
    if (homing)         motor.move(home_vel);
    else if (gear_mode) motor.move(gearVelocityCmd());
    else if (jog_mode)  motor.move(g_telem.jog_vel);
    else                motor.move(g_telem.target);
  }

  if (g_telem.can_ok) {
    CanFrame f;
    while (can.receive(&f)) canHandle(f);
    static uint32_t can_last = 0;
    if (millis() - can_last >= 20) {             // 50 Hz state broadcast
      can_last = millis();
      canSendState();
    }
    g_telem.can_tx = can.txCount();
    g_telem.can_rx = can.rxCount();
    g_telem.can_err = can.errCount();
  }

  command.run();        // non-blocking; returns immediately with no input

  static uint32_t last = 0;
  if (millis() - last >= 200) {
    last = millis();
    if (!foc_running) { enc.checkLink(); g_telem.linkfail = enc.linkFails(); g_telem.stat = enc.lastStat(); }
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
}
