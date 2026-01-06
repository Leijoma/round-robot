/**
 * NeatoLidar Library for ESP32
 *
 * Interface for Neato XV-11 Lidar scanner
 *
 * Hardware connections:
 * - Lidar TX (Orange) -> ESP32 RX pin (configurable)
 * - Lidar Motor Control -> ESP32 PWM pin (configurable)
 * - Lidar GND -> ESP32 GND
 * - Lidar VCC (Red) -> 5V power supply
 *
 * @author Claude Code
 * @date 2026-01-05
 */

#ifndef NEATO_LIDAR_H
#define NEATO_LIDAR_H

#include <Arduino.h>
#include <HardwareSerial.h>

// Neato Lidar Constants
#define NEATO_BAUD_RATE 115200
#define NEATO_PACKET_START 0xFA
#define NEATO_PACKET_SIZE 22
#define NEATO_PACKETS_PER_REV 90
#define NEATO_MIN_INDEX 0xA0

/**
 * Structure for a single Lidar distance measurement
 */
struct NeatoDataPoint {
  uint16_t distance;        // Distance in mm (bits 0-13)
  uint16_t signalStrength;  // Signal strength
  bool isValid;             // True if measurement is valid
  bool strengthWarning;     // True if signal strength is low
};

/**
 * Structure for a complete Lidar packet (4 measurements)
 */
struct NeatoPacket {
  uint8_t index;            // Packet index (0-89)
  uint16_t rpm;             // Motor speed in RPM
  NeatoDataPoint data[4];   // 4 measurements per packet
  bool checksumValid;       // True if checksum passed
};

/**
 * Callback function type for receiving Lidar packets
 * @param packet The received and parsed packet
 */
typedef void (*NeatoPacketCallback)(const NeatoPacket& packet);

/**
 * Callback function type for revolution complete events
 * @param packetsReceived Total packets received in this revolution
 * @param rpm Current motor RPM
 */
typedef void (*NeatoRevolutionCallback)(uint32_t packetsReceived, uint16_t rpm);

/**
 * Main NeatoLidar class
 */
class NeatoLidar {
public:
  /**
   * Constructor
   * @param rxPin GPIO pin connected to Lidar TX
   * @param pwmPin GPIO pin for motor PWM control
   * @param uartNum UART number to use (1 or 2, default 1)
   */
  NeatoLidar(uint8_t rxPin, uint8_t pwmPin, uint8_t uartNum = 1);

  /**
   * Destructor
   */
  ~NeatoLidar();

  /**
   * Initialize the Lidar (call in setup())
   * @return true if initialization successful
   */
  bool begin();

  /**
   * Process incoming Lidar data (call in loop())
   */
  void update();

  /**
   * Start the Lidar motor
   * @param speed PWM duty cycle (0-255), default 255 (full speed)
   */
  void startMotor(uint8_t speed = 255);

  /**
   * Stop the Lidar motor
   */
  void stopMotor();

  /**
   * Set motor speed
   * @param speed PWM duty cycle (0-255)
   */
  void setMotorSpeed(uint8_t speed);

  /**
   * Get current motor speed
   * @return PWM duty cycle (0-255)
   */
  uint8_t getMotorSpeed() const { return motorSpeed; }

  /**
   * Enable/disable automatic RPM control
   * @param enable true to enable P-controller
   * @param targetRpm Target RPM (default 220)
   */
  void enableRpmControl(bool enable, uint16_t targetRpm = 220);

  /**
   * Get target RPM
   * @return Target RPM for P-controller
   */
  uint16_t getTargetRpm() const { return targetRpm; }

  /**
   * Get current RPM (smoothed average)
   * @return Current motor RPM
   */
  uint16_t getCurrentRpm() const { return currentRpm; }

  /**
   * Check if motor is running
   * @return true if motor is running
   */
  bool isMotorRunning() const { return motorRunning; }

  /**
   * Set callback for packet reception
   * @param callback Function to call when packet is received
   */
  void onPacket(NeatoPacketCallback callback) { packetCallback = callback; }

  /**
   * Set callback for revolution complete
   * @param callback Function to call when revolution completes
   */
  void onRevolution(NeatoRevolutionCallback callback) { revolutionCallback = callback; }

  /**
   * Get total packets received
   * @return Total valid packets received
   */
  uint32_t getPacketsReceived() const { return packetsReceived; }

  /**
   * Get total invalid packets
   * @return Total invalid packets
   */
  uint32_t getPacketsInvalid() const { return packetsInvalid; }

  /**
   * Get total bytes received
   * @return Total bytes received
   */
  uint32_t getBytesReceived() const { return bytesReceived; }

  /**
   * Reset statistics
   */
  void resetStats();

  /**
   * Enable/disable debug output
   * @param enable true to enable debug output
   */
  void setDebug(bool enable) { debugEnabled = enable; }

private:
  // Hardware configuration
  uint8_t rxPin;
  uint8_t pwmPin;
  uint8_t uartNum;
  HardwareSerial* serial;

  // Motor control
  uint8_t motorSpeed;
  bool motorRunning;
  const uint32_t pwmFreq = 25000;      // 25 kHz
  const uint8_t pwmResolution = 8;     // 8-bit (0-255)

  // RPM control (PD-controller)
  bool rpmControlEnabled;
  uint16_t targetRpm;
  uint16_t currentRpm;
  uint16_t previousRpm;                // For derivative calculation
  float rpmKp;                         // Proportional gain
  float rpmKd;                         // Derivative gain
  uint32_t lastRpmUpdate;

  // Packet parsing
  uint8_t packetBuffer[NEATO_PACKET_SIZE];
  uint8_t bufferIndex;
  bool packetStartFound;

  // Statistics
  uint32_t packetsReceived;
  uint32_t packetsInvalid;
  uint32_t bytesReceived;
  uint32_t lastPacketTime;
  uint32_t revolutionPacketCount;
  uint16_t lastRpm;

  // Callbacks
  NeatoPacketCallback packetCallback;
  NeatoRevolutionCallback revolutionCallback;

  // Debug
  bool debugEnabled;

  // Private methods
  void readSerialData();
  void processPacket(const uint8_t* packet);
  uint16_t calculateChecksum(const uint8_t* data, uint8_t len);
  void parsePacket(const uint8_t* rawPacket, NeatoPacket& packet);
  void updateRpmControl();
};

#endif // NEATO_LIDAR_H
