// USB <-> UART bridge to the drive's P7 console.
//
// Deliberately dumb: it relays bytes and does not interpret them, so the
// drive's own command set is the only thing you have to learn. Anything typed
// into the ESP32's serial port reaches the drive unchanged.
#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  Serial1.begin(DRIVE_BAUD, SERIAL_8N1, BRIDGE_RX, BRIDGE_TX);
  delay(200);
  Serial.println();
  Serial.println("ESP32 <-> drive bridge");
  Serial.printf("  drive on GPIO%d (TX) / GPIO%d (RX) at %d 8N1\n",
                BRIDGE_TX, BRIDGE_RX, DRIVE_BAUD);
  Serial.println("  type 'D?' for the drive's help, 'M' for SimpleFOC motor verbs");
  Serial.println();
}

void loop() {
  while (Serial.available())  Serial1.write(Serial.read());
  while (Serial1.available()) Serial.write(Serial1.read());
}
