// ============================================================
//  CTR V2 firmware
//  Rule: no module may wait. Every module has begin() (once)
//  and update() (every loop pass, returns quickly).
// ============================================================
#include <Arduino.h>
#include "hw_profile.h"
#include "version.h"
#include "loop_stats.h"
#include "encoder.h"
#include "limit_switches.h"
#include "status_led.h"
#include "console.h"

// Print input events while "watch on" is active (test aid)
static void reportEvents() {
  if (!console::watching()) { encoder::takeDelta(); encoder::takePress();
                              encoder::takeRelease(); encoder::takeLongPress();
                              for (uint8_t i = 0; i < hw::NUM_SWITCHES; i++) limit_switches::changed(i);
                              return; }
  int16_t d = encoder::takeDelta();
  if (d)                         { Serial.print(F("EV enc ")); Serial.println(d); }
  if (encoder::takePress())      Serial.println(F("EV btn press"));
  if (encoder::takeRelease())    Serial.println(F("EV btn release"));
  if (encoder::takeLongPress())  Serial.println(F("EV btn long"));
  for (uint8_t i = 0; i < hw::NUM_SWITCHES; i++) {
    if (limit_switches::changed(i)) {
      Serial.print(F("EV switch ")); Serial.print(hw::SWITCHES[i].name);
      Serial.println(limit_switches::pressed(i) ? F(" pressed") : F(" released"));
    }
  }
}

void setup() {
  Serial.begin(115200);
  status_led::begin();
  encoder::begin();
  limit_switches::begin();
  console::begin();

  status_led::set(status_led::Color::Green);   // idle
  Serial.print(F("CTR V2 firmware ")); Serial.print(FW_VERSION);
  Serial.print(F(" | profile: ")); Serial.println(hw::PROFILE_NAME);
  Serial.println(F("Type 'help' for commands."));
}

void loop() {
  loop_stats::update();
  encoder::update();
  limit_switches::update();
  status_led::update();
  console::update();
  reportEvents();
}
