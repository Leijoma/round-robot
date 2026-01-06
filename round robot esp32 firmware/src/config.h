#pragma once
#include <Arduino.h>
#include <EEPROM.h>
#include "RobotLink.h"

// EEPROM configuration storage for robot parameters
// Layout designed to be compact yet extensible

namespace Config {

// EEPROM layout constants
static const uint16_t MAGIC_NUMBER = 0xAA55;
static const uint8_t VERSION = 1;
static const uint16_t EEPROM_SIZE = 64;

// EEPROM structure (64 bytes total)
struct EEPROMData {
  uint16_t magic;           // 0-1: Magic number for validation (0xAA55)
  uint8_t version;          // 2: Version number
  uint8_t flags;            // 3: Reserved flags

  // Robot physical parameters
  float wheelDiameter;      // 4-7: meters
  float wheelbase;          // 8-11: meters (distance between wheels)
  float ticksPerRev;        // 12-15: encoder resolution

  // PID gains (both motors use same gains)
  float Kp;                 // 16-19
  float Ki;                 // 20-23
  float Kd;                 // 24-27

  // Deadband compensation (PWM offsets)
  float deadbandLF;         // 28-31: Left forward
  float deadbandLR;         // 32-35: Left reverse
  float deadbandRF;         // 36-39: Right forward
  float deadbandRR;         // 40-43: Right reverse

  // Encoder balancing
  uint8_t balanceEnable;    // 44: Enable flag (0 or 1)
  float balanceGain;        // 45-48: Proportional gain

  // Reserved for future use
  uint8_t reserved[13];     // 49-61

  // Integrity check
  uint16_t crc;             // 62-63: CRC16 of bytes 0-61
} __attribute__((packed));

// Default configuration values
struct Defaults {
  static constexpr float WHEEL_DIAMETER = 0.082f;  // 82mm wheels
  static constexpr float WHEELBASE = 0.150f;        // 150mm between wheels (NEEDS MEASUREMENT)
  static constexpr float TICKS_PER_REV = 360.0f;

  static constexpr float KP = 0.25f;
  static constexpr float KI = 0.05f;
  static constexpr float KD = 0.0f;

  static constexpr float DEADBAND_LF = 12.0f;
  static constexpr float DEADBAND_LR = 12.0f;
  static constexpr float DEADBAND_RF = 10.0f;
  static constexpr float DEADBAND_RR = 10.0f;

  static constexpr uint8_t BALANCE_ENABLE = 1;
  static constexpr float BALANCE_GAIN = 0.02f;
};

// Calculate CRC16 for EEPROM data (excludes the CRC field itself)
inline uint16_t calculateCRC(const EEPROMData& data) {
  const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&data);
  size_t len = sizeof(EEPROMData) - sizeof(uint16_t);  // Exclude CRC field
  return RobotLink::Link::crc16_ccitt_false(bytes, len);
}

// Initialize EEPROM data with default values
inline void setDefaults(EEPROMData& data) {
  data.magic = MAGIC_NUMBER;
  data.version = VERSION;
  data.flags = 0;

  data.wheelDiameter = Defaults::WHEEL_DIAMETER;
  data.wheelbase = Defaults::WHEELBASE;
  data.ticksPerRev = Defaults::TICKS_PER_REV;

  data.Kp = Defaults::KP;
  data.Ki = Defaults::KI;
  data.Kd = Defaults::KD;

  data.deadbandLF = Defaults::DEADBAND_LF;
  data.deadbandLR = Defaults::DEADBAND_LR;
  data.deadbandRF = Defaults::DEADBAND_RF;
  data.deadbandRR = Defaults::DEADBAND_RR;

  data.balanceEnable = Defaults::BALANCE_ENABLE;
  data.balanceGain = Defaults::BALANCE_GAIN;

  memset(data.reserved, 0, sizeof(data.reserved));

  data.crc = calculateCRC(data);
}

// Validate EEPROM data
inline bool validate(const EEPROMData& data) {
  // Check magic number
  if (data.magic != MAGIC_NUMBER) {
    return false;
  }

  // Check version (for now, only version 1 is supported)
  if (data.version != VERSION) {
    return false;
  }

  // Verify CRC
  uint16_t calculatedCRC = calculateCRC(data);
  if (data.crc != calculatedCRC) {
    return false;
  }

  // Sanity check values
  if (data.wheelDiameter <= 0.0f || data.wheelDiameter > 1.0f) return false;
  if (data.wheelbase <= 0.0f || data.wheelbase > 2.0f) return false;
  if (data.ticksPerRev <= 0.0f || data.ticksPerRev > 10000.0f) return false;

  return true;
}

// Load configuration from EEPROM
inline bool load(EEPROMData& data) {
  // Read from EEPROM
  EEPROM.get(0, data);

  // Validate
  if (!validate(data)) {
    // Invalid data, load defaults
    setDefaults(data);
    return false;
  }

  return true;
}

// Save configuration to EEPROM
inline void save(EEPROMData& data) {
  // Ensure magic number and version are set
  data.magic = MAGIC_NUMBER;
  data.version = VERSION;

  // Calculate and set CRC
  data.crc = calculateCRC(data);

  // Write to EEPROM
  EEPROM.put(0, data);
}

} // namespace Config
