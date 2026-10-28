#pragma once
// ============================================================
//  Status LED on the rotary encoder (non-blocking blink)
//  V1: no red (pin 49 is the encoder supply, see decision D2).
//  Red requests show cyan on V1 (combine with a blink pattern).
// ============================================================
#include <Arduino.h>

namespace status_led {
enum class Color   : uint8_t { Off, Green, Blue, Cyan, Red };
enum class Pattern : uint8_t { Solid, SlowBlink, FastBlink };

void begin();
void update();                                  // call every loop pass
void set(Color c, Pattern p = Pattern::Solid);
Color   color();
Pattern pattern();
const __FlashStringHelper* colorName(Color c);
}  // namespace status_led
