#include "Encoder.h"

// Transition table for quadrature decoding.
// Index = (prev<<2) | curr, where prev/curr are 2-bit AB states.
// Values: +1, -1 for valid steps, 0 for invalid/no movement.
const int8_t Encoder::_transitionTable[16] = {
  // prev=00: curr 00,01,10,11
   0,  +1,  -1,   0,
  // prev=01
  -1,   0,   0,  +1,
  // prev=10
  +1,   0,   0,  -1,
  // prev=11
   0,  -1,  +1,   0
};

Encoder::Encoder(uint8_t pinA, uint8_t pinB, bool reverse)
: _pinA(pinA),
  _pinB(pinB),
  _reverse(reverse),
  _count(0),
  _lastAB(0),
  _errorCount(0),
  _lastInterruptUs(0),
  _minPulseUs(0) // disabled by default
{
}

void Encoder::begin(bool useInternalPullups) {
  if (useInternalPullups) {
    pinMode(_pinA, INPUT_PULLUP);
    pinMode(_pinB, INPUT_PULLUP);
  } else {
    pinMode(_pinA, INPUT);
    pinMode(_pinB, INPUT);
  }

  _count = 0;
  _errorCount = 0;
  _lastInterruptUs = micros();

  // Initialize last state
  uint8_t a = (uint8_t)digitalRead(_pinA);
  uint8_t b = (uint8_t)digitalRead(_pinB);
  _lastAB = (a << 1) | b;
}

int32_t Encoder::getCount() const {
  noInterrupts();
  int32_t c = _count;
  interrupts();
  return c;
}

void Encoder::reset() {
  noInterrupts();
  _count = 0;
  interrupts();
}

void Encoder::setCount(int32_t count) {
  noInterrupts();
  _count = count;
  interrupts();
}

uint32_t Encoder::getErrorCount() const {
  noInterrupts();
  uint32_t e = _errorCount;
  interrupts();
  return e;
}

void Encoder::setMinPulseUs(uint16_t us) {
  noInterrupts();
  _minPulseUs = us;
  interrupts();
}

void Encoder::handleInterrupt() {
  // Glitch reject (microseconds). Helps with EMI without dropping real ticks.
  if (_minPulseUs) {
    uint32_t now = micros();
    uint32_t dt = now - _lastInterruptUs;
    if (dt < _minPulseUs) {
      return;
    }
    _lastInterruptUs = now;
  }

  // Read both channels
  uint8_t a = (uint8_t)digitalRead(_pinA);
  uint8_t b = (uint8_t)digitalRead(_pinB);
  uint8_t curr = (a << 1) | b;

  uint8_t prev = _lastAB;
  uint8_t idx = (prev << 2) | curr;
  int8_t delta = _transitionTable[idx];

  if (delta == 0 && curr != prev) {
    // Invalid transition (often noise/edge skew)
    _errorCount++;
  }

  _lastAB = curr;

  if (_reverse) delta = -delta;
  _count += delta;
}
