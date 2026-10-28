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
#include "drivers.h"
#include "steppers.h"
#include "motion.h"

// Handle input events once per pass. An encoder press always stops
// motion (temporary until the state machine in step 1.4).
// With "watch on", events are also printed (test aid).
static void handleEvents() {
  const bool w = console::watching();
  int16_t d = encoder::takeDelta();
  if (w && d) { Serial.print(F("EV enc ")); Serial.println(d); }
  if (encoder::takePress()) {
    if (w) Serial.println(F("EV btn press"));
    if (motion::anyBusy()) motion::stopAll(F("encoder button"));
  }
  if (encoder::takeRelease()   && w) Serial.println(F("EV btn release"));
  if (encoder::takeLongPress() && w) Serial.println(F("EV btn long"));
  for (uint8_t i = 0; i < hw::NUM_SWITCHES; i++) {
    if (limit_switches::changed(i) && w) {
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
  drivers::begin();
  steppers::begin();
  motion::begin();

  status_led::set(status_led::Color::Green);   // idle
  Serial.print(F("CTR V2 firmware ")); Serial.print(FW_VERSION);
  Serial.print(F(" | profile: ")); Serial.println(hw::PROFILE_NAME);
  drivers::report();
  Serial.println(F("Type 'help' for commands."));
}

void loop() {
  loop_stats::update();
  encoder::update();
  limit_switches::update();
  status_led::update();
  motion::update();
  console::update();
  handleEvents();
}
