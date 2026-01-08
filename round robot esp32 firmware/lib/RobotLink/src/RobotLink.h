#pragma once
#include <Arduino.h>

// RobotLink - binary framed protocol over Stream (HardwareSerial etc)
//
// Frame format (little-endian):
//  [0]   0xAA
//  [1]   0x55
//  [2]   type        (uint8_t)
//  [3]   len         (uint8_t) payload bytes, 0..255
//  [4..] payload     (len bytes)
//  [..]  crc16_lo    (uint8_t) CRC16/CCITT-FALSE over (type,len,payload)
//  [..]  crc16_hi    (uint8_t)
//
// CRC params: poly=0x1021 init=0xFFFF xorout=0x0000 refin=false refout=false
//
namespace RobotLink {

static const uint8_t SOF0 = 0xAA;
static const uint8_t SOF1 = 0x55;

enum MsgType : uint8_t {
  MSG_ODOM    = 0x01, // Arduino -> ESP32
  MSG_CMD_VEL = 0x02, // ESP32 -> Arduino
  MSG_STATUS  = 0x03, // optional

  // Extended message types for robot motor controller
  MSG_SET_VEL      = 0x10,  // Set motor velocities (m/s)
  MSG_SET_PID      = 0x11,  // Set PID parameters
  MSG_SET_DEADBAND = 0x12,  // Set deadband values
  MSG_SET_CONFIG   = 0x13,  // Set robot configuration
  MSG_ENABLE_STREAM = 0x14, // Enable/disable continuous updates
  MSG_GET_CONFIG   = 0x15,  // Request configuration
  MSG_CONFIG_RESP  = 0x16,  // Configuration response
  MSG_SAVE_CONFIG  = 0x17,  // Save config to EEPROM
  MSG_LOAD_CONFIG  = 0x18,  // Load config from EEPROM
  MSG_ZERO_ENCODERS = 0x19, // Zero encoder counts
  MSG_STOP         = 0x1A,  // Emergency stop
  MSG_RESET_POSE   = 0x1B,  // Reset pose to (0, 0, 0) without zeroing encoders
  MSG_SET_PID_PER_MOTOR = 0x1C, // Set PID parameters per motor (left/right separate)

  // Lidar message types (ESP32 -> Host)
  MSG_LIDAR_SCAN   = 0x20,  // Complete 360° scan data
  MSG_LIDAR_STATUS = 0x21,  // Lidar status (RPM, packets, etc.)
  MSG_LIDAR_ENABLE = 0x22,  // Enable/disable lidar motor (Host -> ESP32)
  MSG_LIDAR_SET_RPM = 0x23, // Set target RPM (Host -> ESP32)

  MSG_PING    = 0x7E, // optional
  MSG_PONG    = 0x7F  // optional
};

// Callback signature
typedef void (*FrameHandler)(uint8_t type, const uint8_t* payload, uint8_t len);

struct Config {
  // Max payload you want to accept (<=255). Default 64 to save RAM on AVR.
  uint8_t maxPayload = 64;

  // If true, reader discards frames with len > maxPayload (recommended).
  bool rejectOversize = true;
};

class Link {
public:
  explicit Link(Stream& io, const Config& cfg = Config());

  // Call frequently in loop() to parse incoming bytes.
  // Returns true if at least one valid frame was delivered to handler.
  bool poll(FrameHandler handler);

  // Low-level send
  bool sendFrame(uint8_t type, const uint8_t* payload, uint8_t len);

  // Convenience: send from a POD struct
  template <typename T>
  bool sendStruct(uint8_t type, const T& data) {
    return sendFrame(type, reinterpret_cast<const uint8_t*>(&data), (uint8_t)sizeof(T));
  }

  // Stats
  uint32_t framesOk() const { return _framesOk; }
  uint32_t framesBadCrc() const { return _framesBadCrc; }
  uint32_t framesBadLen() const { return _framesBadLen; }
  uint32_t bytesDropped() const { return _bytesDropped; }

  // CRC utility (public for host tooling / tests)
  static uint16_t crc16_ccitt_false(const uint8_t* data, size_t len);

  // Packing helpers (little-endian)
  static void wr_u16_le(uint8_t* p, uint16_t v);
  static void wr_i16_le(uint8_t* p, int16_t v);
  static void wr_u32_le(uint8_t* p, uint32_t v);
  static void wr_i32_le(uint8_t* p, int32_t v);

  static uint16_t rd_u16_le(const uint8_t* p);
  static int16_t  rd_i16_le(const uint8_t* p);
  static uint32_t rd_u32_le(const uint8_t* p);
  static int32_t  rd_i32_le(const uint8_t* p);

private:
  enum RxState : uint8_t {
    S_WAIT_SOF0 = 0,
    S_WAIT_SOF1,
    S_READ_TYPE,
    S_READ_LEN,
    S_READ_PAYLOAD,
    S_READ_CRC_LO,
    S_READ_CRC_HI
  };

  Stream& _io;
  Config _cfg;

  RxState _st = S_WAIT_SOF0;
  uint8_t _type = 0;
  uint8_t _len = 0;
  uint8_t _idx = 0;
  uint16_t _rxCrc = 0;
  uint16_t _calcCrc = 0;

  // Payload buffer sized by cfg.maxPayload (allocated at runtime)
  uint8_t* _buf = nullptr;

  uint32_t _framesOk = 0;
  uint32_t _framesBadCrc = 0;
  uint32_t _framesBadLen = 0;
  uint32_t _bytesDropped = 0;

  void resetRx();
  void dropByte();
};

} // namespace RobotLink
