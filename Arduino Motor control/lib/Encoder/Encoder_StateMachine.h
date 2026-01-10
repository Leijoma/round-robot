/**
 * Encoder_StateMachine.h - State Machine Quadrature Encoder Library
 *
 * Uses state transition lookup table for more robust 2X decoding.
 * More tolerant of slow rise times and noise than simple XOR method.
 */

#ifndef ENCODER_STATEMACHINE_H
#define ENCODER_STATEMACHINE_H

#include <Arduino.h>

class Encoder {
public:
    /**
     * Constructor
     * @param pinA Encoder channel A pin (must support interrupts)
     * @param pinB Encoder channel B pin (any digital pin)
     * @param reverse Set true to reverse counting direction
     */
    Encoder(uint8_t pinA, uint8_t pinB, bool reverse = false);

    /**
     * Initialize encoder pins and attach interrupt.
     * Call this in setup() after Serial.begin().
     */
    void begin();

    /**
     * Get current encoder count (signed)
     */
    int32_t getCount();

    /**
     * Reset encoder count to zero
     */
    void reset();

    /**
     * Set encoder count to specific value
     */
    void setCount(int32_t count);

    /**
     * ISR handler - call this from your interrupt service routine
     */
    void handleInterrupt();

    /**
     * Get count of invalid state transitions (for diagnostics)
     */
    uint32_t getErrorCount();

private:
    uint8_t _pinA;
    uint8_t _pinB;
    bool _reverse;
    volatile int32_t _count;
    volatile uint8_t _lastState;
    volatile uint32_t _errorCount;  // Track invalid transitions

    // State transition lookup table for 2X quadrature decoding
    // Maps (prevState << 2 | currentState) to count delta
    // Valid Gray code transitions: +1 or -1
    // Invalid transitions: 0 (and increment error counter)
    static const int8_t _transitionTable[16];
};

#endif
