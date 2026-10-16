// ---------------------------------------------------------------------------
// ESP32 CAN host for the drive.
//
// A minimal, complete implementation of the drive's CAN protocol, meant to be
// read as much as run. Talk to one or more drives, see their telemetry, and
// drive the axis from a serial console.
//
// The ESP32's TWAI controller speaks plain CAN 2.0, so no external controller
// is needed -- only a 3.3 V transceiver such as an SN65HVD230. The DRIVE end
// already has one onboard, wired to J2, so a two-node bus is:
//
//     ESP32 ---- transceiver ---- CANH/CANL ---- drive J2
//
// Terminate at BOTH physical ends with 120 ohm and nowhere else. With power
// off, CANH-CANL across the assembled bus should read about 60 ohm. If it
// reads 40, something has a third terminator fitted -- many transceiver
// breakouts have one soldered on permanently.
//
// SELF-TEST WITHOUT HARDWARE: TWAI_MODE_NO_ACK lets the controller transmit
// and receive its own frames with no peer and no transceiver, which verifies
// bit timing, filters and the protocol encoding before any wiring exists.
// Send 't' on the console to run it.
// ---------------------------------------------------------------------------
#include <Arduino.h>
#include "driver/twai.h"
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include "secrets.h"

// Hostname on the LAN. With mDNS this resolves as <HOSTNAME>.local, which
// saves hunting for a DHCP address every time the board reboots.
#ifndef HOSTNAME
#define HOSTNAME "gearcutter"
#endif

static WebServer http(80);

// Last telemetry seen from each axis, so the web view can answer immediately
// rather than waiting for the next 50 Hz broadcast.
struct AxisState { float angle; float vel; uint8_t flags; uint32_t age_ms; bool seen; };
static AxisState axes[8] = {};

// The drive's UART console is relayed too, on separate pins, so one board can
// do both during bring-up: CAN for control and telemetry, UART for the
// human-readable command set. Lines prefixed '>' go to the UART verbatim;
// everything else is a CAN host command.
#ifndef UART_TX_PIN
#define UART_TX_PIN 17
#endif
#ifndef UART_RX_PIN
#define UART_RX_PIN 16
#endif
#ifndef DRIVE_BAUD
#define DRIVE_BAUD 38400
#endif

#ifndef CAN_TX_PIN
#define CAN_TX_PIN 5
#endif
#ifndef CAN_RX_PIN
#define CAN_RX_PIN 4
#endif

// --- the drive's protocol (mirrors src/AxisCan.h in the firmware) ----------
#define NODE_ID            1
#define CAN_ID_OF(a, t)    (uint16_t)((((a) & 0x7F) << 4) | ((t) & 0x0F))
#define CAN_ADDR_OF(id)    (uint8_t)(((id) >> 4) & 0x7F)
#define CAN_TYPE_OF(id)    (uint8_t)((id) & 0x0F)

enum : uint8_t {
  CMD_ENABLE = 0x1, CMD_TARGET = 0x2, CMD_CURRENT = 0x3, CMD_PING = 0x4,
  CMD_JOG = 0x5, CMD_GEAR_K = 0x6, CMD_GEAR_X = 0x7, CMD_STEPDIR = 0xA,
};
enum : uint8_t { MSG_STATE = 0x8, MSG_PONG = 0x9 };
enum : uint8_t { FLAG_FOC_READY = 1 << 0, FLAG_ENERGISED = 1 << 1, FLAG_LINK_FAIL = 1 << 2 };

static bool can_up = false;

// loopback: point RX at the TX pin through the GPIO matrix so the controller
// hears its own output. The ESP32's TWAI has no internal TX-to-RX path, so
// without this a self-test with no transceiver attached transmits into a
// floating RX and never receives anything.
static bool canStart(twai_mode_t mode, bool loopback = false) {
  if (can_up) { twai_stop(); twai_driver_uninstall(); can_up = false; }
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
      (gpio_num_t)CAN_TX_PIN,
      (gpio_num_t)(loopback ? CAN_TX_PIN : CAN_RX_PIN), mode);
  twai_timing_config_t  t = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
  if (twai_start() != ESP_OK) return false;
  can_up = true;
  return true;
}

static bool send(uint8_t type, const void* data, uint8_t len, bool self_rx = false) {
  twai_message_t m = {};
  m.self = self_rx ? 1 : 0;   // required for self-test; ignored otherwise
  m.identifier = CAN_ID_OF(NODE_ID, type);
  m.data_length_code = len;
  if (len) memcpy(m.data, data, len);
  return twai_transmit(&m, pdMS_TO_TICKS(50)) == ESP_OK;
}
static bool sendF(uint8_t type, float v) { return send(type, &v, 4); }
static bool sendU8(uint8_t type, uint8_t v) { return send(type, &v, 1); }

// Drive telemetry, broadcast at 50 Hz.
static void onState(const twai_message_t& m) {
  if (m.data_length_code < 8) return;
  float angle; memcpy(&angle, m.data, 4);
  int16_t vel = (int16_t)(m.data[4] | (m.data[5] << 8));
  uint8_t flags = m.data[6];
  Serial.printf("axis %u: %.4f rad  %.2f rad/s  [%s%s%s] seq %u\n",
                CAN_ADDR_OF(m.identifier), angle, vel / 100.0f,
                (flags & FLAG_FOC_READY) ? "ready " : "",
                (flags & FLAG_ENERGISED) ? "live "  : "",
                (flags & FLAG_LINK_FAIL) ? "ENCODER-FAULT" : "",
                m.data[7]);
}

static void selfTest() {
  Serial.println("self-test: NO_ACK mode, no transceiver or peer needed");
  if (!canStart(TWAI_MODE_NO_ACK, true)) { Serial.println("  driver install FAILED"); return; }
  const uint8_t probe[8] = {0xDE,0xAD,0xBE,0xEF,1,2,3,4};
  send(MSG_STATE, probe, 8, true);
  twai_message_t rx;
  if (twai_receive(&rx, pdMS_TO_TICKS(100)) == ESP_OK) {
    bool ok = rx.data_length_code == 8;
    for (int i = 0; i < 8 && ok; i++) ok = rx.data[i] == probe[i];
    Serial.printf("  frame returned %s\n", ok ? "INTACT -- PASS" : "CORRUPTED -- FAIL");
  } else {
    Serial.println("  no frame returned -- FAIL");
  }
  twai_status_info_t s; twai_get_status_info(&s);
  Serial.printf("  tx %u  rx %u  bus errors %u\n", s.msgs_to_tx, s.msgs_to_rx, s.bus_error_count);
  canStart(TWAI_MODE_NORMAL);
  Serial.println("  back in NORMAL mode");
}

// --- web interface --------------------------------------------------------
static String statusJson() {
  String j = R"({"uptime_ms":)" + String(millis());
  twai_status_info_t st; twai_get_status_info(&st);
  j += R"(,"can":{"state":)" + String((int)st.state);
  j += R"(,"tx_err":)" + String(st.tx_error_counter);
  j += R"(,"rx_err":)" + String(st.rx_error_counter);
  j += R"(,"bus_err":)" + String(st.bus_error_count) + "}";
  j += R"(,"axes":[)";
  bool first = true;
  for (int i = 0; i < 8; i++) {
    if (!axes[i].seen) continue;
    if (!first) j += ",";
    first = false;
    j += R"({"id":)" + String(i);
    j += R"(,"angle":)" + String(axes[i].angle, 5);
    j += R"(,"vel":)" + String(axes[i].vel, 3);
    j += R"(,"ready":)"  + String((axes[i].flags & FLAG_FOC_READY) ? "true" : "false");
    j += R"(,"live":)"   + String((axes[i].flags & FLAG_ENERGISED) ? "true" : "false");
    j += R"(,"fault":)"  + String((axes[i].flags & FLAG_LINK_FAIL) ? "true" : "false");
    j += R"(,"age_ms":)" + String(millis() - axes[i].age_ms) + "}";
  }
  j += "]}";
  return j;
}

static const char INDEX_HTML[] PROGMEM = R"HTML(<!doctype html>
<title>drive status</title>
<style>
 body{font:14px system-ui;margin:2rem;max-width:40rem}
 td{padding:.25rem .75rem .25rem 0;font-variant-numeric:tabular-nums}
 .f{color:#b00} .o{color:#888}
</style>
<h2>drive status</h2>
<div id=b>connecting...</div>
<script>
async function tick(){
 try{
  const r = await fetch('/status'); const d = await r.json();
  let h = '<table>';
  if(!d.axes.length) h += '<tr><td class=o>no telemetry yet - is CAN wired?</td></tr>';
  for(const a of d.axes){
   h += '<tr><td>axis '+a.id+'</td><td>'+a.angle.toFixed(4)+' rad</td>'
      + '<td>'+a.vel.toFixed(2)+' rad/s</td>'
      + '<td>'+(a.ready?'ready ':'')+(a.live?'live ':'')
      + (a.fault?'<span class=f>ENCODER FAULT</span>':'')+'</td>'
      + '<td class=o>'+a.age_ms+' ms ago</td></tr>';
  }
  h += '</table><p class=o>CAN bus errors: '+d.can.bus_err+'</p>';
  document.getElementById('b').innerHTML = h;
 }catch(e){ document.getElementById('b').textContent = 'lost connection'; }
}
tick(); setInterval(tick, 500);
</script>)HTML";

static void startWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HOSTNAME);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("wifi: connecting to ");
  Serial.println(WIFI_SSID);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) { delay(250); Serial.print("."); }
  Serial.println();
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("wifi: FAILED -- continuing without it; CAN and UART are unaffected");
    return;
  }
  Serial.print("wifi: ");
  Serial.print(WiFi.localIP());
  Serial.print("  rssi ");
  Serial.println(WiFi.RSSI());
  if (MDNS.begin(HOSTNAME)) {
    MDNS.addService("http", "tcp", 80);
    Serial.print("mdns: http://");
    Serial.print(HOSTNAME);
    Serial.println(".local");
  }
  http.on("/", []() { http.send_P(200, "text/html", INDEX_HTML); });
  http.on("/status", []() { http.send(200, "application/json", statusJson()); });
  http.begin();
  Serial.println("http: serving on port 80");
}

static void help() {
  Serial.println("t self-test | p ping | e/d enable,disable");
  Serial.println("j<v> jog rad/s | g<k> gear | x<pos> gear master | s step/dir");
  Serial.println("c<mA> current | T<rad> target | ? status");
  Serial.println(">...  send a line to the drive's UART console (e.g. >D?)");
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.printf("ESP32 CAN host -- TX GPIO%d, RX GPIO%d, 500 kbit/s\n", CAN_TX_PIN, CAN_RX_PIN);
  Serial1.begin(DRIVE_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  Serial.printf("UART console relay -- TX GPIO%d, RX GPIO%d, %d 8N1\n",
                UART_TX_PIN, UART_RX_PIN, DRIVE_BAUD);
  if (!canStart(TWAI_MODE_NORMAL)) Serial.println("TWAI install FAILED");
  startWifi();
  help();
}

void loop() {
  http.handleClient();

  twai_message_t rx;
  while (twai_receive(&rx, 0) == ESP_OK) {
    if (CAN_TYPE_OF(rx.identifier) == MSG_STATE) onState(rx);
    else if (CAN_TYPE_OF(rx.identifier) == MSG_PONG) Serial.println("pong");
  }

  while (Serial1.available()) Serial.write(Serial1.read());   // drive console

  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (!line.length()) return;
  if (line[0] == '>') {                      // relay to the drive's UART
    Serial1.print(line.substring(1)); Serial1.print('\n');
    return;
  }
  char c = line[0];
  float arg = line.substring(1).toFloat();

  switch (c) {
    case 't': selfTest(); break;
    case 'p': send(CMD_PING, nullptr, 0); break;
    case 'e': sendU8(CMD_ENABLE, 1); break;
    case 'd': sendU8(CMD_ENABLE, 0); break;
    case 'j': sendF(CMD_JOG, arg); break;
    case 'g': sendF(CMD_GEAR_K, arg); break;
    case 'x': sendF(CMD_GEAR_X, arg); break;
    case 'T': sendF(CMD_TARGET, arg); break;
    case 's': sendU8(CMD_STEPDIR, 1); break;
    case 'c': { uint16_t ma = (uint16_t)arg; send(CMD_CURRENT, &ma, 2); break; }
    case '?': {
      twai_status_info_t s; twai_get_status_info(&s);
      Serial.printf("state %d  tx-q %u  rx-q %u  tx-err %u  rx-err %u  bus-err %u\n",
                    (int)s.state, s.msgs_to_tx, s.msgs_to_rx,
                    s.tx_error_counter, s.rx_error_counter, s.bus_error_count);
      break;
    }
    default: help(); break;
  }
}
