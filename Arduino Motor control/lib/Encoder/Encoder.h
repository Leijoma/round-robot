/**
 * Encoder.h - 2X Quadrature Encoder Library
 *
 * Provides 2X decoding using XOR logic on CHANGE interrupts.
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
     * ISR handler - call this from your interrupt service routine
     */
    void handleInterrupt();

private:
    uint8_t _pinA;
    uint8_t _pinB;
    bool _reverse;
    volatile int32_t _count;
};

#endif
