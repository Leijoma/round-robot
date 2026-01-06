/*
 * SoftwareSerial Test - Send data continuously to verify TX works
 */

#include <Arduino.h>
#include <SoftwareSerial.h>

const uint8_t PIN_SOFT_RX = A0;  // Arduino RX (connected to ESP32 GPIO27 TX)
const uint8_t PIN_SOFT_TX = A1;  // Arduino TX (connected to ESP32 GPIO26 RX)

SoftwareSerial softSerial(PIN_SOFT_RX, PIN_SOFT_TX);

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println(F("\n=== SoftwareSerial Test ==="));
  Serial.println(F("Testing A0/A1 communication to ESP32"));
  Serial.println(F("A0 (RX) <- ESP32 GPIO27 (TX)"));
  Serial.println(F("A1 (TX) -> ESP32 GPIO26 (RX)"));
  Serial.println();

  softSerial.begin(9600);
  delay(100);

  Serial.println(F("SoftwareSerial started at 9600 baud"));
  Serial.println(F("Sending test messages every second..."));
  Serial.println();
}

uint32_t counter = 0;

void loop() {
  // Send test message
  counter++;

  // Send simple text
  softSerial.print("TEST ");
  softSerial.println(counter);

  Serial.print(F("Sent: TEST "));
  Serial.println(counter);

  // Check if we receive anything back
  if (softSerial.available()) {
    Serial.print(F("Received: "));
    while (softSerial.available()) {
      char c = softSerial.read();
      Serial.write(c);
    }
    Serial.println();
  }

  delay(1000);
}
