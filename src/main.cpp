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
#include "crc_probe.h"

static const uint32_t DRIVE_TIMEOUT_MS = 15000;

TLE5012B enc(ENC_CS);
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
static bool  gear_mode = false;
static float gear_shaft0 = 0.0f, gear_master0 = 0.0f;
static float gear_master_prev = 0.0f, gear_vel_f = 0.0f;
static uint32_t gear_t_prev = 0;

static void markEnergised();      // defined below

static void allOff() {
  motor.disable();
  driver.setPwm(0, 0);
  digitalWrite(COIL_A_DIR_1, LOW); digitalWrite(COIL_A_DIR_2, LOW);
  digitalWrite(COIL_B_DIR_1, LOW); digitalWrite(COIL_B_DIR_2, LOW);
  energised = false;
  foc_running = false;
  jog_mode = false;
  gear_mode = false;
  g_telem.gear_on = 0;
  g_telem.jog_vel = 0.0f;
  g_telem.drive_mA = 0;
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
  gear_shaft0  = enc.getAngle();
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
  float err = g_telem.target - enc.getAngle();
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
  can.send(CAN_ID_OF(CAN_NODE_ID, CAN_MSG_STATE), d, 8);
}

// Act on a host frame. Addressing is in software so more axes drop in later.
static void canHandle(const CanFrame& f) {
  if (CAN_ADDR_OF(f.id) != CAN_NODE_ID) return;
  switch (CAN_TYPE_OF(f.id)) {
    case CAN_CMD_ENABLE:
      if (f.len >= 1 && f.data[0] && g_telem.foc_ready) {
        g_telem.target = enc.getAngle();
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
        if (ma > DRIVER_MAX_mA) ma = DRIVER_MAX_mA;   // never trust the bus
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
    case CAN_CMD_PING:
      can.send(CAN_ID_OF(CAN_NODE_ID, CAN_MSG_PONG), BOARD_TAG, 4);
      break;
    default: break;
  }
}

void setup() {
  // PB4/PB3/PA15 are JTAG pins at reset. NOJTAG frees them and keeps SW-DP,
  // so the ST-Link stays alive. Must precede any pin setup. See CHANGELOG.
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_AFIO_REMAP_SWJ_NOJTAG();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  Serial.begin(38400);
  delay(200);
  Serial.println();
  Serial.print(F("=== ")); Serial.print(F(BOARD_NAME));
  Serial.println(F(" : SimpleFOC firmware, idle ==="));

  enc.init();
  g_telem.raw_min = 0xFFFF;
  g_telem.raw_max = 0;
  g_telem.stat    = enc.readStatus();
  last_raw        = enc.readRaw();

  // "volts" == amps, see config.h
  driver.voltage_power_supply = SIMPLEFOC_VPS;
  driver.voltage_limit        = AMPS(DRIVER_MAX_mA);
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
  motor.P_angle.P       = 8.0f;
  motor.PID_velocity.P  = 0.05f;
  motor.PID_velocity.I  = 1.0f;
  motor.PID_velocity.D  = 0.0f;

  motor.init();
  allOff();                                    // never idle energised
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
  g_telem.angle = enc.getAngle();      // multi-turn, no extra bus traffic
  g_telem.shaft = enc.getAngle();
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
          g_telem.target = enc.getAngle();
          markEnergised(); foc_running = true; motor.enable();
        }
        break;
      case 7:                                   // closed-loop +1 shaft revolution
        if (g_telem.foc_ready) {
          motor.controller = MotionControlType::angle; jog_mode = false;
          motor.voltage_limit = amps;
          g_telem.target = enc.getAngle() + _2PI;
          markEnergised(); foc_running = true; motor.enable();
        }
        break;
      case 10: {                                // CAN self-test, LOOPBACK
        g_telem.can_ok = can.init(500000, true) ? 1 : 0;
        g_telem.can_btr = can.btr();
        g_telem.can_selftest = 0;
        if (g_telem.can_ok) {
          const uint8_t probe[8] = {0xDE,0xAD,0xBE,0xEF,0x01,0x02,0x03,0x04};
          uint16_t id = CAN_ID_OF(CAN_NODE_ID, CAN_MSG_STATE);
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

  if (foc_running) {
    motor.loopFOC();
    if (gear_mode)      motor.move(gearVelocityCmd());
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

  static uint32_t last = 0;
  if (millis() - last >= 200) {
    last = millis();
    if (!foc_running) { enc.checkLink(); g_telem.linkfail = enc.linkFails(); g_telem.stat = enc.lastStat(); }
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
}
