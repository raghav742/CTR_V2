#pragma once
// ============================================================
//  Buzzer (step 3.1): Casper's melodies, played without waiting.
//  play() starts a melody; update() advances it from the loop.
//  tone() uses Timer2 (Timer1 is the step interrupt).
//  Also plays sounds on state changes by itself: homing done,
//  fault.
// ============================================================
#include <Arduino.h>

namespace buzzer {
enum class Sound : uint8_t { Click, Beep, Ok, Boot, Error, Home };
void begin();
void update();             // call every loop pass
void play(Sound s);
void setMuted(bool m);
bool muted();
}  // namespace buzzer
