#include "status_led.h"
#include "hw_profile.h"

namespace status_led {
namespace {
  Color   curColor   = Color::Off;
  Pattern curPattern = Pattern::Solid;
  bool    phaseOn    = true;
  uint32_t lastToggle = 0;

  void writeChannel(uint8_t pin, bool on) {
    digitalWrite(pin, (on != hw::LED_ACTIVE_LOW) ? HIGH : LOW);
  }

  void apply(bool on) {
    bool g = false, b = false;
    if (on) {
      switch (curColor) {
        case Color::Green: g = true; break;
        case Color::Blue:  b = true; break;
        case Color::Cyan:  g = b = true; break;
        case Color::Red:
          // No red on V1: show cyan (blink pattern makes it distinct)
          if (!hw::LED_HAS_RED) { g = b = true; }
          break;
        default: break;
      }
    }
    writeChannel(hw::PIN_LED_G, g);
    writeChannel(hw::PIN_LED_B, b);
  }
}

void begin() {
  // Encoder "+" supply must be HIGH, otherwise LEDs and button are dead (D2)
  pinMode(hw::PIN_ENC_SUPPLY, OUTPUT);
  digitalWrite(hw::PIN_ENC_SUPPLY, HIGH);
  pinMode(hw::PIN_LED_G, OUTPUT);
  pinMode(hw::PIN_LED_B, OUTPUT);
  apply(false);
}

void set(Color c, Pattern p) {
  curColor = c; curPattern = p;
  phaseOn = true; lastToggle = millis();
  apply(true);
}

void update() {
  if (curPattern == Pattern::Solid) return;
  uint16_t half = (curPattern == Pattern::SlowBlink) ? 500 : 120;
  uint32_t now = millis();
  if (now - lastToggle >= half) {
    lastToggle = now;
    phaseOn = !phaseOn;
    apply(phaseOn);
  }
}

Color   color()   { return curColor; }
Pattern pattern() { return curPattern; }

const __FlashStringHelper* colorName(Color c) {
  switch (c) {
    case Color::Green: return F("green");
    case Color::Blue:  return F("blue");
    case Color::Cyan:  return F("cyan");
    case Color::Red:   return F("red");
    default:           return F("off");
  }
}
}  // namespace status_led
