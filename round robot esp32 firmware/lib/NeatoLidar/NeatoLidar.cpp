/**
 * NeatoLidar Library for ESP32 - Implementation
 */

#include "NeatoLidar.h"

NeatoLidar::NeatoLidar(uint8_t rxPin, uint8_t pwmPin, uint8_t uartNum)
  : rxPin(rxPin),
    pwmPin(pwmPin),
    uartNum(uartNum),
    serial(nullptr),
    motorSpeed(0),
    motorRunning(false),
    rpmControlEnabled(false),
    targetRpm(220),
    currentRpm(0),
    previousRpm(0),
    rpmKp(0.5f),   // Proportional gain
    rpmKd(2.0f),   // Derivative gain for damping
    lastRpmUpdate(0),
    bufferIndex(0),
    packetStartFound(false),
    packetsReceived(0),
    packetsInvalid(0),
    bytesReceived(0),
    lastPacketTime(0),
    revolutionPacketCount(0),
    lastRpm(0),
    packetCallback(nullptr),
    revolutionCallback(nullptr),
    debugEnabled(false) {

  // Create HardwareSerial instance
  if (uartNum == 1) {
    serial = new HardwareSerial(1);
  } else if (uartNum == 2) {
    serial = new HardwareSerial(2);
  }
}

NeatoLidar::~NeatoLidar() {
  if (serial) {
    serial->end();
    delete serial;
  }
}

bool NeatoLidar::begin() {
  if (!serial) {
    if (debugEnabled) {
      Serial.println("[NeatoLidar] ERROR: Invalid UART number");
    }
    return false;
  }

  // Initialize serial communication
  // Neato Lidar TX is NOT inverted (tested with XV-11)
  serial->begin(NEATO_BAUD_RATE, SERIAL_8N1, rxPin, -1, false);

  // Initialize PWM for motor control
  ledcAttach(pwmPin, pwmFreq, pwmResolution);
  ledcWrite(pwmPin, 0);

  if (debugEnabled) {
    Serial.printf("[NeatoLidar] Initialized on RX=%d, PWM=%d, UART%d\n",
                  rxPin, pwmPin, uartNum);
  }

  return true;
}

void NeatoLidar::update() {
  readSerialData();
  updateRpmControl();
}

void NeatoLidar::startMotor(uint8_t speed) {
  // Start with a reasonable PWM to get motor spinning
  // P-controller will adjust to target RPM
  if (speed == 255) {
    speed = 180;  // Initial PWM - P-controller will adjust to target
  }
  setMotorSpeed(speed);
  motorRunning = true;

  if (debugEnabled) {
    Serial.printf("[NeatoLidar] Motor started at PWM=%d (P-controller will adjust)\n", speed);
  }
}

void NeatoLidar::stopMotor() {
  setMotorSpeed(0);
  motorRunning = false;

  if (debugEnabled) {
    Serial.println("[NeatoLidar] Motor stopped");
  }
}

void NeatoLidar::setMotorSpeed(uint8_t speed) {
  motorSpeed = speed;
  ledcWrite(pwmPin, motorSpeed);
}

void NeatoLidar::resetStats() {
  packetsReceived = 0;
  packetsInvalid = 0;
  bytesReceived = 0;
  revolutionPacketCount = 0;
}

void NeatoLidar::readSerialData() {
  if (!serial) return;

  while (serial->available()) {
    uint8_t byte = serial->read();
    bytesReceived++;

    // Look for packet start
    if (!packetStartFound) {
      if (byte == NEATO_PACKET_START) {
        packetStartFound = true;
        bufferIndex = 0;
        packetBuffer[bufferIndex++] = byte;
      }
    } else {
      // Add byte to buffer
      packetBuffer[bufferIndex++] = byte;

      // Validate index byte (2nd byte) to avoid false starts
      if (bufferIndex == 2) {
        uint8_t index = byte;
        // Valid index is 0xA0 or higher (160-249)
        if (index < NEATO_MIN_INDEX) {
          // Invalid index - reset and check if this byte is a start
          packetStartFound = false;
          bufferIndex = 0;
          if (byte == NEATO_PACKET_START) {
            packetStartFound = true;
            packetBuffer[bufferIndex++] = byte;
          }
        }
      }

      // Check if packet is complete
      if (bufferIndex >= NEATO_PACKET_SIZE) {
        processPacket(packetBuffer);
        packetStartFound = false;
        bufferIndex = 0;
      }
    }
  }
}

void NeatoLidar::processPacket(const uint8_t* packet) {
  // Verify start byte
  if (packet[0] != NEATO_PACKET_START) {
    packetsInvalid++;
    return;
  }

  // Calculate and verify checksum
  uint16_t calculatedChecksum = calculateChecksum(packet, NEATO_PACKET_SIZE - 2);
  uint16_t receivedChecksum = packet[20] | (packet[21] << 8);
  bool checksumValid = (calculatedChecksum == receivedChecksum);

  if (!checksumValid) {
    packetsInvalid++;
  }

  // Parse packet even if checksum fails (data is often still valid)
  NeatoPacket parsedPacket;
  parsePacket(packet, parsedPacket);
  parsedPacket.checksumValid = checksumValid;

  // Update statistics
  packetsReceived++;
  lastPacketTime = millis();
  revolutionPacketCount++;
  lastRpm = parsedPacket.rpm;

  // Check for revolution complete (90 packets = 360 degrees)
  if (revolutionPacketCount >= NEATO_PACKETS_PER_REV) {
    if (revolutionCallback) {
      revolutionCallback(packetsReceived, parsedPacket.rpm);
    }
    revolutionPacketCount = 0;
  }

  // Call packet callback
  if (packetCallback) {
    packetCallback(parsedPacket);
  }
}

uint16_t NeatoLidar::calculateChecksum(const uint8_t* data, uint8_t len) {
  uint32_t chk32 = 0;

  // Sum data in 16-bit words
  for (uint8_t i = 0; i < len; i += 2) {
    uint16_t word = data[i] | (data[i + 1] << 8);
    chk32 = (chk32 << 1) + word;
  }

  // Fold carry bits back
  uint16_t checksum = (chk32 & 0xFFFF) + (chk32 >> 16);
  checksum = (checksum & 0x7FFF);  // Clear top bit

  return checksum;
}

void NeatoLidar::parsePacket(const uint8_t* rawPacket, NeatoPacket& packet) {
  // Extract index (packet number 0-89)
  packet.index = rawPacket[1] - NEATO_MIN_INDEX;

  // Extract speed (bytes 2-3, little-endian)
  // Speed is in 64ths of RPM
  uint16_t speedRaw = rawPacket[2] | (rawPacket[3] << 8);
  packet.rpm = speedRaw / 64;

  // Update current RPM with exponential smoothing
  if (currentRpm == 0) {
    currentRpm = packet.rpm;  // Initialize
  } else {
    // Exponential moving average: alpha = 0.1
    currentRpm = (uint16_t)(0.9f * currentRpm + 0.1f * packet.rpm);
  }

  // Extract 4 data points (4 bytes each)
  for (int i = 0; i < 4; i++) {
    int offset = 4 + (i * 4);

    // Distance: bytes 0-1 of data point (little-endian)
    uint16_t distRaw = rawPacket[offset] | (rawPacket[offset + 1] << 8);

    packet.data[i].distance = distRaw & 0x3FFF;           // Lower 14 bits
    packet.data[i].isValid = !(distRaw & 0x4000);         // Bit 14 (inverted)
    packet.data[i].strengthWarning = (distRaw & 0x8000);  // Bit 15

    // Signal strength: bytes 2-3 of data point (little-endian)
    packet.data[i].signalStrength = rawPacket[offset + 2] | (rawPacket[offset + 3] << 8);
  }
}

void NeatoLidar::enableRpmControl(bool enable, uint16_t targetRpm) {
  rpmControlEnabled = enable;
  this->targetRpm = targetRpm;

  if (debugEnabled) {
    if (enable) {
      Serial.printf("[NeatoLidar] RPM control enabled, target=%d RPM\n", targetRpm);
    } else {
      Serial.println("[NeatoLidar] RPM control disabled");
    }
  }
}

void NeatoLidar::updateRpmControl() {
  if (!rpmControlEnabled || !motorRunning || currentRpm == 0) {
    return;
  }

  // Update at 10 Hz (every 100ms)
  uint32_t now = millis();
  if (now - lastRpmUpdate < 100) {
    return;
  }
  lastRpmUpdate = now;

  // Calculate error (proportional term)
  int16_t error = targetRpm - currentRpm;

  // Calculate derivative (rate of change of RPM)
  int16_t derivative = currentRpm - previousRpm;
  previousRpm = currentRpm;

  // PD-controller: P term pushes toward target, D term dampens overshoot
  float pTerm = error * rpmKp;
  float dTerm = -derivative * rpmKd;  // Negative because we want to oppose changes
  int16_t adjustment = (int16_t)(pTerm + dTerm);

  // Apply adjustment with limits
  int16_t newSpeed = motorSpeed + adjustment;
  newSpeed = constrain(newSpeed, 100, 255);  // Keep within safe limits

  // Update motor speed if changed significantly (>1 PWM unit)
  if (abs(newSpeed - motorSpeed) >= 1) {
    setMotorSpeed((uint8_t)newSpeed);

    if (debugEnabled && (packetsReceived % 900 == 0)) {  // Print every 10 revolutions
      Serial.printf("[RPM-Ctrl] Target:%d Current:%d Error:%d D:%d PWM:%d\n",
                    targetRpm, currentRpm, error, derivative, motorSpeed);
    }
  }
}

