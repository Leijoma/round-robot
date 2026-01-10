#pragma once
#include <Arduino.h>

/**
 * Encoder - robust quadrature decoder for Arduino Uno with A-only interrupts.
 *
 * - Attach interrupt on pinA with CHANGE.
 * - In ISR, read both A and B and use a transition table (Gray code).
 * - Optional microsecond glitch reject to kill EMI spikes.
 *
 * Sign convention:
 * - Set reverse=false so forward motion yields positive counts.
 * - Flip reverse per wheel if needed.
 */
class Encoder {
public:
  Encoder(uint8_t pinA, uint8_t pinB, bool reverse = false);

  // If you use external pullups (recommended), pass false.
  void begin(bool useInternalPullups = false);

  // Atomic read
  int32_t getCount() const;

  void reset();
  void setCount(int32_t count);

  // Counts invalid transitions (useful diagnostic)
  uint32_t getErrorCount() const;

  // Glitch reject: ignore interrupts closer than this many microseconds (0 disables).
  void setMinPulseUs(uint16_t us);

  // Call this from your ISR attached to pinA CHANGE
  void handleInterrupt();

private:
  uint8_t _pinA;
  uint8_t _pinB;
  bool _reverse;

  volatile int32_t _count;
  volatile uint8_t _lastAB;          // last state: (A<<1)|B
  volatile uint32_t _errorCount;

  volatile uint32_t _lastInterruptUs;
  volatile uint16_t _minPulseUs;     // glitch reject window

  static const int8_t _transitionTable[16];
};
