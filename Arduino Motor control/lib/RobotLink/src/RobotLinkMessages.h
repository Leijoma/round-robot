#pragma once
#include <Arduino.h>

// Payload structures for RobotLink protocol messages
// Message type codes are defined in RobotLink.h
namespace RobotLink {

// ============================================================
// Payload Structures (all little-endian, packed)
// ============================================================

// MSG_SET_VEL (0x10): Set motor velocities
struct SetVelPayload {
  float velLeft;    // m/s
  float velRight;   // m/s
} __attribute__((packed));

// MSG_SET_PID (0x11): Set PID parameters (applied to both motors)
struct SetPidPayload {
  float Kp;
  float Ki;
  float Kd;
} __attribute__((packed));

// MSG_SET_DEADBAND (0x12): Set deadband PWM offsets
struct SetDeadbandPayload {
  float leftForward;   // PWM offset
  float leftReverse;   // PWM offset
  float rightForward;  // PWM offset
  float rightReverse;  // PWM offset
} __attribute__((packed));

// MSG_SET_CONFIG (0x13) & MSG_CONFIG_RESP (0x16): Robot configuration
struct ConfigPayload {
  float wheelDiameter;   // meters
  float wheelbase;       // meters (distance between wheel centers)
  float ticksPerRev;     // encoder ticks per wheel revolution
  uint8_t invertLeft;    // 0 or 1
  uint8_t invertRight;   // 0 or 1
  uint8_t balanceEnable; // 0 or 1
  uint8_t reserved;      // padding for alignment
  float balanceGain;     // encoder balance gain
} __attribute__((packed));

// MSG_ODOM (0x01): Odometry data
struct OdomPayload {
  int32_t encoderLeft;   // total ticks since startup/zero
  int32_t encoderRight;  // total ticks since startup/zero
  float velLeft;         // m/s
  float velRight;        // m/s
  int16_t pwmLeft;       // current PWM value
  int16_t pwmRight;      // current PWM value
  uint32_t timestamp;    // milliseconds since startup
} __attribute__((packed));

// MSG_STATUS (0x03): Current robot status
struct StatusPayload {
  float Kp, Ki, Kd;           // PID gains (12 bytes)
  float deadband[4];          // L_fwd, L_rev, R_fwd, R_rev (16 bytes)
  uint8_t pidEnabled;         // 1 byte
  uint8_t streamEnabled;      // 1 byte
  uint16_t streamInterval;    // milliseconds (2 bytes)
  uint32_t framesReceived;    // stats (4 bytes)
  uint32_t framesSent;        // stats (4 bytes)
  uint32_t uptime;            // milliseconds (4 bytes)
} __attribute__((packed));

// MSG_ENABLE_STREAM (0x14): Control continuous odometry updates
struct EnableStreamPayload {
  uint8_t enable;          // 0=disable, 1=enable
  uint16_t intervalMs;     // update interval in milliseconds
} __attribute__((packed));

// MSG_PING (0x7E) / MSG_PONG (0x7F): Connection test
struct PingPongPayload {
  uint32_t timestamp;      // echo back for latency measurement
} __attribute__((packed));

// ============================================================
// Lidar Message Payloads
// ============================================================

// MSG_LIDAR_STATUS (0x21): Current lidar status
struct LidarStatusPayload {
  uint16_t currentRpm;        // Current motor RPM
  uint16_t targetRpm;         // Target motor RPM
  uint8_t motorRunning;       // 0=off, 1=on
  uint8_t rpmControlEnabled;  // 0=manual, 1=PD-controller
  uint8_t motorSpeed;         // Current PWM (0-255)
  uint8_t reserved;           // Padding
  uint32_t packetsReceived;   // Total valid packets
  uint32_t packetsInvalid;    // Total invalid packets
  uint32_t scansComplete;     // Total complete 360° scans
  uint32_t timestamp;         // milliseconds since startup
} __attribute__((packed));

// MSG_LIDAR_SCAN (0x20): Single lidar reading (send multiple for full scan)
// To reduce UDP packet size, we send readings in batches
struct LidarScanPayload {
  uint32_t timestamp;         // milliseconds since startup
  uint16_t rpm;               // RPM at time of scan
  uint16_t startAngle;        // Starting angle (0-359)
  uint8_t numReadings;        // Number of readings in this packet (max ~60)
  uint8_t reserved[3];        // Padding for alignment
  // Followed by array of readings (not included in struct - sent separately)
  // Each reading: uint16_t distance_mm, uint16_t signalStrength, uint8_t flags
} __attribute__((packed));

// Single lidar reading (for serialization)
struct LidarReading {
  uint16_t distance_mm;       // Distance in millimeters
  uint16_t signalStrength;    // Signal strength
  uint8_t flags;              // bit 0: invalid, bit 1: warning
} __attribute__((packed));

// MSG_LIDAR_ENABLE (0x22): Enable/disable lidar motor
struct LidarEnablePayload {
  uint8_t enable;             // 0=disable, 1=enable
} __attribute__((packed));

// MSG_LIDAR_SET_RPM (0x23): Set target RPM
struct LidarSetRpmPayload {
  uint16_t targetRpm;         // Target RPM (typically 200-300)
} __attribute__((packed));

} // namespace RobotLink
