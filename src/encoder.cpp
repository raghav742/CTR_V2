#include "encoder.h"
#include "hw_profile.h"

namespace encoder {
namespace {
  // rotation
  uint8_t lastA      = HIGH;
  int16_t delta      = 0;
  long    total      = 0;
  // button
  bool     rawLast    = false;   // last raw reading (true = pressed)
  bool     stable     = false;   // debounced state
  uint32_t rawSince   = 0;       // time of last raw change
  uint32_t downSince  = 0;
  bool     longFired  = false;
  bool     evPress = false, evRelease = false, evLong = false;

  bool readPressed() {
    bool level = digitalRead(hw::PIN_ENC_SW) == HIGH;
    return hw::ENC_SW_ACTIVE_HIGH ? level : !level;
  }
}

void begin() {
  pinMode(hw::PIN_ENC_A, INPUT_PULLUP);
  pinMode(hw::PIN_ENC_B, INPUT_PULLUP);
  pinMode(hw::PIN_ENC_SW, hw::ENC_SW_PULLUP ? INPUT_PULLUP : INPUT);
  lastA   = digitalRead(hw::PIN_ENC_A);
  rawLast = stable = readPressed();
}

void update() {
  const uint32_t now = millis();

  // --- rotation: count on the falling edge of A, direction from B ---
  uint8_t a = digitalRead(hw::PIN_ENC_A);
  if (a != lastA) {
    if (a == LOW) {
      int8_t d = (digitalRead(hw::PIN_ENC_B) != a) ? +1 : -1;
      delta += d;
      total += d;
    }
    lastA = a;
  }

  // --- button: accept a new state only after DEBOUNCE_MS without change ---
  bool raw = readPressed();
  if (raw != rawLast) { rawLast = raw; rawSince = now; }
  if (raw != stable && (now - rawSince) >= DEBOUNCE_MS) {
    stable = raw;
    if (stable) { evPress = true; downSince = now; longFired = false; }
    else        { evRelease = true; }
  }
  if (stable && !longFired && (now - downSince) >= LONG_PRESS_MS) {
    longFired = true;
    evLong = true;
  }
}

int16_t takeDelta()     { int16_t d = delta; delta = 0; return d; }
bool    takePress()     { bool e = evPress;   evPress = false;   return e; }
bool    takeRelease()   { bool e = evRelease; evRelease = false; return e; }
bool    takeLongPress() { bool e = evLong;    evLong = false;    return e; }
bool    isDown()        { return stable; }
long    position()      { return total; }
}  // namespace encoder
