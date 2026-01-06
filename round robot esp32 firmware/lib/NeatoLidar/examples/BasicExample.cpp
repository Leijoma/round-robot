/**
 * Neato Lidar Basic Example
 *
 * This example demonstrates basic usage of the NeatoLidar library.
 * It reads Lidar data and prints distance measurements.
 *
 * Hardware connections:
 * - Lidar TX (Orange wire) -> ESP32 GPIO 16
 * - Lidar Motor PWM -> ESP32 GPIO 21
 * - Lidar GND -> ESP32 GND
 * - Lidar VCC (Red wire) -> 5V power supply
 */

#include <Arduino.h>
#include <NeatoLidar.h>

// Pin definitions
#define LIDAR_RX_PIN 16
#define LIDAR_PWM_PIN 21

// Create Lidar instance
NeatoLidar lidar(LIDAR_RX_PIN, LIDAR_PWM_PIN);

// Variables for statistics
uint32_t lastStatsTime = 0;
const uint32_t STATS_INTERVAL = 5000;  // Print stats every 5 seconds

// Callback function for each Lidar packet
void onLidarPacket(const NeatoPacket& packet) {
  // Print first measurement from each packet (every 4 degrees)
  uint16_t angle = packet.index * 4;  // Each packet covers 4 degrees

  // Get first data point
  const NeatoDataPoint& point = packet.data[0];

  // Only print valid measurements with reasonable signal strength
  if (point.isValid && point.signalStrength > 100) {
    Serial.printf("Angle: %3d° | Distance: %4d mm | Strength: %4d\n",
                  angle, point.distance, point.signalStrength);
  }
}

// Callback function for each complete revolution
void onLidarRevolution(uint32_t totalPackets, uint16_t rpm) {
  Serial.printf("\n[REVOLUTION] Complete! Total packets: %lu | RPM: %u\n\n",
                totalPackets, rpm);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n======================================");
  Serial.println("   Neato Lidar Basic Example");
  Serial.println("======================================\n");

  // Enable debug output
  lidar.setDebug(true);

  // Initialize Lidar
  if (!lidar.begin()) {
    Serial.println("ERROR: Failed to initialize Lidar!");
    while (1) delay(1000);
  }

  // Set callbacks
  lidar.onPacket(onLidarPacket);
  lidar.onRevolution(onLidarRevolution);

  // Start motor
  Serial.println("Starting Lidar motor...\n");
  lidar.startMotor(255);  // Full speed

  Serial.println("Lidar initialized! Waiting for data...\n");
}

void loop() {
  // Update Lidar (process incoming data)
  lidar.update();

  // Print statistics periodically
  uint32_t now = millis();
  if (now - lastStatsTime >= STATS_INTERVAL) {
    lastStatsTime = now;

    Serial.println("\n--- Statistics ---");
    Serial.printf("Packets received: %lu\n", lidar.getPacketsReceived());
    Serial.printf("Packets invalid: %lu\n", lidar.getPacketsInvalid());
    Serial.printf("Bytes received: %lu\n", lidar.getBytesReceived());
    Serial.printf("Motor speed: %d/255\n", lidar.getMotorSpeed());
    Serial.printf("Motor running: %s\n", lidar.isMotorRunning() ? "YES" : "NO");
    Serial.println("------------------\n");
  }

  delay(1);  // Small delay to prevent watchdog issues
}
