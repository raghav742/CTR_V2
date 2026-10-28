#pragma once
// ============================================================
//  Rotary encoder: rotation + push button (non-blocking)
//  Rotation: polled quadrature, one count per detent.
//  Button:   debounced; events for press, release, long press.
//  Polarity and pull-up come from the hardware profile.
// ============================================================
#include <Arduino.h>

namespace encoder {
void begin();
void update();                // call every loop pass

int16_t takeDelta();          // rotation counts since last call (+ = clockwise)
bool    takePress();          // true once per press  (event, cleared on read)
bool    takeRelease();        // true once per release
bool    takeLongPress();      // true once when held >= LONG_PRESS_MS
bool    isDown();             // current debounced state
long    position();           // total count since startup (for diagnostics)

constexpr uint16_t DEBOUNCE_MS   = 20;
constexpr uint16_t LONG_PRESS_MS = 1000;
}  // namespace encoder
