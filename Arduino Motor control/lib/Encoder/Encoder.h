/**
 * Encoder.h - 2X Quadrature Encoder Library
 *
 * Provides 2X decoding using state machine lookup table on CHANGE interrupts.
 * More robust than XOR method - validates state transitions and tracks errors.
 * Tested and verified with 714 ticks/rev.
 */

#ifndef ENCODER_H
#define ENCODER_H

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
     * Get count of invalid state transitions (for diagnostics)
     * High error count indicates encoder signal quality issues
     */
    uint32_t getErrorCount();

    /**
     * ISR handler - call this from your interrupt service routine
     */
    void handleInterrupt();

private:
    uint8_t _pinA;
    uint8_t _pinB;
    bool _reverse;
    volatile int32_t _count;
    volatile uint8_t _lastState;      // Previous encoder state (2 bits: A<<1 | B)
    volatile uint32_t _errorCount;    // Count of invalid state transitions
};

#endif
