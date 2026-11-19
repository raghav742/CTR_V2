#include "joystick.h"
#include "hw_profile.h"

namespace joystick {
namespace {
  int8_t   jx = 0, jy = 0;
#if HW_HAS_JOYSTICK
  constexpr uint8_t  READ_MS    = 10;
  constexpr uint8_t  DEBOUNCE   = 3;      // switched: same reading 3x (30 ms)
  constexpr int16_t  DEADZONE   = 60;     // analog: counts around the centre (of 512)
  constexpr int16_t  FULL_SCALE = 400;    // analog: counts from centre for 100 %
  uint32_t lastRead = 0;
  int8_t   candX = 0, candY = 0;
  uint8_t  stable = 0;
  int16_t  centreX = 512, centreY = 512;  // measured at start (stick must be released)

  int8_t fromAnalog(int16_t raw, int16_t centre) {
    int16_t d = raw - centre;
    if (d > -DEADZONE && d < DEADZONE) return 0;
    d += (d > 0) ? -DEADZONE : DEADZONE;
    long pct = (long)d * 100 / (FULL_SCALE - DEADZONE);
    return pct > 100 ? 100 : pct < -100 ? -100 : (int8_t)pct;
  }
#endif
}

void begin() {
#if HW_HAS_JOYSTICK
  if (hw::JOYSTICK_TYPE == JoystickType::Switched) {
    pinMode(hw::PIN_JOY_UP, INPUT_PULLUP);       // contact to GND = pressed
    pinMode(hw::PIN_JOY_DOWN, INPUT_PULLUP);
    pinMode(hw::PIN_JOY_LEFT, INPUT_PULLUP);
    pinMode(hw::PIN_JOY_RIGHT, INPUT_PULLUP);
  } else if (hw::JOYSTICK_TYPE == JoystickType::Analog) {
    centreX = analogRead(hw::PIN_JOY_X);
    centreY = analogRead(hw::PIN_JOY_Y);
  }
#endif
}

void update() {
#if HW_HAS_JOYSTICK
  uint32_t now = millis();
  if (now - lastRead < READ_MS) return;
  lastRead = now;
  if (hw::JOYSTICK_TYPE == JoystickType::Switched) {
    int8_t nx = 0, ny = 0;
    if (!digitalRead(hw::PIN_JOY_UP))    ny += 100;
    if (!digitalRead(hw::PIN_JOY_DOWN))  ny -= 100;
    if (!digitalRead(hw::PIN_JOY_RIGHT)) nx += 100;
    if (!digitalRead(hw::PIN_JOY_LEFT))  nx -= 100;
    if (nx == candX && ny == candY) {
      if (stable < DEBOUNCE) stable++;
    } else {
      candX = nx; candY = ny; stable = 0;
    }
    if (stable >= DEBOUNCE) { jx = candX; jy = candY; }
  } else if (hw::JOYSTICK_TYPE == JoystickType::Analog) {
    // analogRead takes ~0.1 ms each; two per 10 ms is negligible
    jx = fromAnalog(analogRead(hw::PIN_JOY_X), centreX);
    jy = fromAnalog(analogRead(hw::PIN_JOY_Y), centreY);
  }
#endif
}

int8_t x() { return jx; }
int8_t y() { return jy; }
bool centred() { return jx == 0 && jy == 0; }
}  // namespace joystick
