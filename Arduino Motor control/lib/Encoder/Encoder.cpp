/**
 * Encoder.cpp - 2X Quadrature Encoder Implementation
 */

#include "Encoder.h"

Encoder::Encoder(uint8_t pinA, uint8_t pinB, bool reverse)
    : _pinA(pinA), _pinB(pinB), _reverse(reverse), _count(0) {
}

void Encoder::begin() {
    pinMode(_pinA, INPUT_PULLUP);
    pinMode(_pinB, INPUT_PULLUP);
    _count = 0;
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

void Encoder::handleInterrupt() {
    // 2X decoding with XOR logic
    // When A == B, one direction; when A != B, other direction
    bool a = digitalRead(_pinA);
    bool b = digitalRead(_pinB);

    if (_reverse) {
        // Reversed counting direction
        if (a == b) {
            _count--;
        } else {
            _count++;
        }
    } else {
        // Normal counting direction
        if (a == b) {
            _count++;
        } else {
            _count--;
        }
    }
}
