/**
 * Encoder.cpp - 2X Quadrature Encoder Implementation
 *
 * Uses state machine lookup table for robust quadrature decoding.
 * More tolerant of slow rise times and noise than simple XOR method.
 */

#include "Encoder.h"

Encoder::Encoder(uint8_t pinA, uint8_t pinB, bool reverse)
    : _pinA(pinA), _pinB(pinB), _reverse(reverse), _count(0), _lastState(0), _errorCount(0) {
}

void Encoder::begin() {
    pinMode(_pinA, INPUT_PULLUP);
    pinMode(_pinB, INPUT_PULLUP);
    _count = 0;
    _errorCount = 0;

    // Initialize state by reading current encoder position
    uint8_t a = digitalRead(_pinA);
    uint8_t b = digitalRead(_pinB);
    _lastState = (a << 1) | b;
}

int32_t Encoder::getCount() {
    noInterrupts();
    int32_t count = _count;
    interrupts();
    return count;
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

uint32_t Encoder::getErrorCount() {
    noInterrupts();
    uint32_t errors = _errorCount;
    interrupts();
    return errors;
}

void Encoder::handleInterrupt() {
    // State machine 2X quadrature decoder
    // Read current encoder state
    uint8_t a = digitalRead(_pinA);
    uint8_t b = digitalRead(_pinB);
    uint8_t currentState = (a << 1) | b;  // 2-bit state: 00, 01, 10, 11

    // State transition lookup table for 2X quadrature decoding
    // Maps (prevState << 2 | currentState) to count delta
    // Valid Gray code transitions: +1 (forward) or -1 (backward)
    // Invalid transitions (both bits flip): 0
    static const int8_t transitionTable[16] = {
         0, +1, -1,  0,   // From state 00 (A=0, B=0)
        -1,  0,  0, +1,   // From state 01 (A=0, B=1)
        +1,  0,  0, -1,   // From state 10 (A=1, B=0)
         0, -1, +1,  0    // From state 11 (A=1, B=1)
    };

    // Look up count delta based on state transition
    int8_t delta = transitionTable[(_lastState << 2) | currentState];

    // Detect invalid transitions for diagnostics
    if (delta == 0 && _lastState != currentState) {
        // Invalid transition: both channels changed simultaneously
        // This indicates signal quality issues or missed interrupts
        _errorCount++;
    }

    // Apply direction reversal if configured
    if (_reverse) {
        _count -= delta;
    } else {
        _count += delta;
    }

    // Save current state for next interrupt
    _lastState = currentState;
}
