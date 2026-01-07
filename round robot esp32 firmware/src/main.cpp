#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "RobotLink.h"
#include "RobotLinkMessages.h"
#include "RobotLinkUDP.h"
#include "NeatoLidar.h"

// WiFi Configuration
const char* WIFI_SSID = "SurfsUp";
const char* WIFI_PASSWORD = "MollyLou123!";
const uint16_t UDP_PORT = 5000;

// Arduino Serial Connection
#define ARDUINO_RX_PIN 26  // ESP32 RX <- Arduino TX
#define ARDUINO_TX_PIN 27  // ESP32 TX -> Arduino RX
#define ARDUINO_BAUD 9600

// LIDAR Configuration
#define LIDAR_RX_PIN 16    // ESP32 RX <- Neato LIDAR TX (orange wire)
#define LIDAR_PWM_PIN 21   // ESP32 PWM -> LIDAR Motor Control
#define LIDAR_TARGET_RPM 240  // 4.0 Hz scan rate, 2.5:1 ratio with 10 Hz odometry

// Global objects
HardwareSerial arduinoSerial(1);
RobotLink::Link* arduinoLink = nullptr;

WiFiUDP udp;
RobotLink::UDPStream* udpStream = nullptr;
RobotLink::Link* hostLink = nullptr;

NeatoLidar* lidar = nullptr;

// LIDAR scan batching (batch 10 Neato packets = 40 readings per UDP message)
#define LIDAR_BATCH_SIZE 40
RobotLink::LidarReading lidarBatchBuffer[LIDAR_BATCH_SIZE];
uint8_t lidarBatchCount = 0;
uint16_t lidarBatchStartAngle = 0;
uint16_t lidarBatchRpm = 0;
uint32_t lidarBatchTimestamp = 0;
uint32_t lidarScansComplete = 0;

void setupWiFi() {
  Serial.println("\n========================================");
  Serial.println("WiFi Setup");
  Serial.println("========================================");

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to ");
  Serial.print(WIFI_SSID);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(" Connected!");
    Serial.print("  IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("  UDP Port: ");
    Serial.println(UDP_PORT);
  } else {
    Serial.println(" FAILED!");
    Serial.println("  Continuing without WiFi...");
  }
}

void setupArduino() {
  Serial.println("\n========================================");
  Serial.println("Arduino Serial Setup");
  Serial.println("========================================");

  // CRITICAL: Increase RX buffer to prevent data loss during WiFi operations
  // Default 256 bytes fills up quickly at 20Hz ODOM rate
  // Using 512 bytes (2x default) - larger values may cause ESP32 to crash
  arduinoSerial.setRxBufferSize(512);

  // Initialize Serial1 for Arduino
  arduinoSerial.begin(ARDUINO_BAUD, SERIAL_8N1, ARDUINO_RX_PIN, ARDUINO_TX_PIN);
  delay(100);

  Serial.printf("  Serial1: GPIO26(RX) GPIO27(TX) @ %d baud\n", ARDUINO_BAUD);
  Serial.println("  RX Buffer: 512 bytes (2x default)");

  // Initialize RobotLink protocol for Arduino
  RobotLink::Config cfg;
  cfg.maxPayload = 64;
  cfg.rejectOversize = true;
  arduinoLink = new RobotLink::Link(arduinoSerial, cfg);

  Serial.println("  RobotLink initialized");
  Serial.println("  Waiting 2 seconds for Arduino boot...");
  delay(2000);
  Serial.println("  Arduino ready - waiting for host to enable streaming");
}

void setupUDP() {
  Serial.println("\n========================================");
  Serial.println("UDP Setup");
  Serial.println("========================================");

  if (!udp.begin(UDP_PORT)) {
    Serial.println("  UDP begin FAILED!");
    return;
  }

  Serial.printf("  UDP listening on port %d\n", UDP_PORT);

  // Initialize RobotLink protocol for Host
  udpStream = new RobotLink::UDPStream(udp);

  RobotLink::Config cfg;
  cfg.maxPayload = 64;
  cfg.rejectOversize = true;
  hostLink = new RobotLink::Link(*udpStream, cfg);

  Serial.println("  RobotLinkUDP initialized");
}

// Helper function to send batched LIDAR scan data
void sendLidarBatch() {
  if (lidarBatchCount == 0 || !hostLink || !udpStream || !udpStream->hasClient()) {
    return;  // Nothing to send or no client connected
  }

  // Prepare scan payload header
  RobotLink::LidarScanPayload scanHeader;
  scanHeader.timestamp = lidarBatchTimestamp;
  scanHeader.rpm = lidarBatchRpm;
  scanHeader.startAngle = lidarBatchStartAngle;
  scanHeader.numReadings = lidarBatchCount;
  scanHeader.reserved[0] = 0;
  scanHeader.reserved[1] = 0;
  scanHeader.reserved[2] = 0;

  // Calculate total payload size
  uint8_t totalLen = sizeof(scanHeader) + (lidarBatchCount * sizeof(RobotLink::LidarReading));

  // Create temporary buffer for full payload
  uint8_t payload[totalLen];
  memcpy(payload, &scanHeader, sizeof(scanHeader));
  memcpy(payload + sizeof(scanHeader), lidarBatchBuffer, lidarBatchCount * sizeof(RobotLink::LidarReading));

  // Send via RobotLink
  hostLink->sendFrame(RobotLink::MSG_LIDAR_SCAN, payload, totalLen);

  // Reset batch
  lidarBatchCount = 0;
}

void setupLidar() {
  Serial.println("\n========================================");
  Serial.println("LIDAR Setup");
  Serial.println("========================================");

  // Create LIDAR instance on Serial2 (UART 2)
  lidar = new NeatoLidar(LIDAR_RX_PIN, LIDAR_PWM_PIN, 2);

  if (!lidar->begin()) {
    Serial.println("  ERROR: LIDAR initialization failed!");
    return;
  }

  Serial.printf("  Serial2: GPIO%d(RX) @ 115200 baud\n", LIDAR_RX_PIN);
  Serial.printf("  PWM Motor: GPIO%d @ 25kHz\n", LIDAR_PWM_PIN);
  Serial.printf("  Target RPM: %d\n", LIDAR_TARGET_RPM);

  // Enable RPM control (PD-controller for stable rotation)
  lidar->enableRpmControl(true, LIDAR_TARGET_RPM);

  // Set up packet callback for real-time scan batching
  lidar->onPacket([](const NeatoPacket& packet) {
    // Each Neato packet contains 4 measurements
    for (int i = 0; i < 4; i++) {
      const NeatoDataPoint& point = packet.data[i];

      // Calculate angle for this measurement
      // Each packet covers 4 degrees (360/90), each point is 1 degree apart
      uint16_t angle = (packet.index * 4 + i) % 360;

      // Start new batch if buffer is empty
      if (lidarBatchCount == 0) {
        lidarBatchStartAngle = angle;
        lidarBatchTimestamp = millis();
        lidarBatchRpm = packet.rpm;
      }

      // Add reading to batch buffer
      RobotLink::LidarReading& reading = lidarBatchBuffer[lidarBatchCount];
      reading.distance_mm = point.distance;
      reading.signalStrength = point.signalStrength;
      reading.flags = 0;
      if (!point.isValid) reading.flags |= 0x01;
      if (point.strengthWarning) reading.flags |= 0x02;

      lidarBatchCount++;

      // Send batch when full (40 readings = 10 Neato packets)
      if (lidarBatchCount >= LIDAR_BATCH_SIZE) {
        sendLidarBatch();
      }
    }
  });

  // Set up revolution callback for statistics and batch flushing
  lidar->onRevolution([](uint32_t packetsReceived, uint16_t rpm) {
    lidarScansComplete++;

    // Flush any remaining batch at end of revolution
    sendLidarBatch();

    // Print status every 10 revolutions
    if (lidarScansComplete % 10 == 0) {
      Serial.printf("[LIDAR] Scan #%lu complete: %d RPM, %lu packets\n",
                    lidarScansComplete, rpm, packetsReceived);
    }
  });

  Serial.println("  LIDAR initialized");
  Serial.println("  Motor: STOPPED (use MSG_LIDAR_ENABLE to start)");
}

void setup() {
  // USB Serial for debugging
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n");
  Serial.println("========================================");
  Serial.println("ESP32 Robot Bridge - INCREMENTAL BUILD");
  Serial.println("========================================");
  Serial.println("Building on working test code");
  Serial.println("Adding: WiFi + UDP + Message Forwarding");
  Serial.println("========================================\n");

  setupWiFi();
  setupArduino();
  setupUDP();
  setupLidar();

  Serial.println("\n========================================");
  Serial.println("System Ready!");
  Serial.println("========================================");
  Serial.println("Waiting for Host to connect...\n");
}

void loop() {
  // Poll for messages FROM Arduino
  arduinoLink->poll([](uint8_t type, const uint8_t* payload, uint8_t len) {
    // Forward to Host if we have a connection
    if (udpStream && udpStream->hasClient()) {
      hostLink->sendFrame(type, payload, len);
    }
  });

  // Poll for messages FROM Host
  if (hostLink) {
    hostLink->poll([](uint8_t type, const uint8_t* payload, uint8_t len) {
      // Check if this is a new client
      if (udpStream->hasClient()) {
        static IPAddress lastIP;
        static uint16_t lastPort = 0;
        IPAddress clientIP = udpStream->getClientIP();
        uint16_t clientPort = udpStream->getClientPort();

        if (clientPort != lastPort || clientIP != lastIP) {
          lastIP = clientIP;
          lastPort = clientPort;
          Serial.printf("\nHost connected: %s:%d\n", clientIP.toString().c_str(), clientPort);
        }
      }

      // Handle LIDAR control messages
      if (type == RobotLink::MSG_LIDAR_ENABLE) {
        if (lidar && len >= 1) {
          RobotLink::LidarEnablePayload* cmd = (RobotLink::LidarEnablePayload*)payload;
          if (cmd->enable) {
            lidar->startMotor();
            Serial.println("  Host -> LIDAR: START");
          } else {
            lidar->stopMotor();
            Serial.println("  Host -> LIDAR: STOP");
          }
        }
      } else if (type == RobotLink::MSG_LIDAR_SET_RPM) {
        if (lidar && len >= 2) {
          RobotLink::LidarSetRpmPayload* cmd = (RobotLink::LidarSetRpmPayload*)payload;
          lidar->enableRpmControl(true, cmd->targetRpm);
          Serial.printf("  Host -> LIDAR: SET_RPM %d\n", cmd->targetRpm);
        }
      } else {
        // Forward all other messages to Arduino
        arduinoLink->sendFrame(type, payload, len);

        // Debug output for important messages
        if (type == RobotLink::MSG_ENABLE_STREAM) {
          Serial.println("  Host -> Arduino: ENABLE_STREAM");
        } else if (type == RobotLink::MSG_SET_VEL) {
          Serial.println("  Host -> Arduino: SET_VEL");
        } else if (type == RobotLink::MSG_STOP) {
          Serial.println("  Host -> Arduino: STOP");
        }
      }
    });
  }

  // Process LIDAR data
  if (lidar) {
    lidar->update();
  }

  delay(1);  // Small delay to prevent watchdog issues
}
