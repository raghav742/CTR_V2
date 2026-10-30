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
#include "system_state.h"
#include "safety.h"
#include "homing.h"
#include "calib.h"
#include "limits.h"
#include "settings.h"
#include "rotation_zero.h"
#include "serial_io.h"

// Handle input events once per pass.
//   encoder press during motion -> stop (no fault)
//   encoder long press (1 s) in FAULT -> acknowledge
// With "watch on", events are also printed (test aid).
static void handleEvents() {
  const bool w = console::watching();
  int16_t d = encoder::takeDelta();
  if (w && d) { Out.print(F("EV enc ")); Out.println(d); }
  if (encoder::takePress()) {
    if (w) Out.println(F("EV btn press"));
    safety::userStop(F("encoder button"), encoder::pressEdgeUs());
  }
  if (encoder::takeRelease()   && w) Out.println(F("EV btn release"));
  if (encoder::takeLongPress()) {
    if (w) Out.println(F("EV btn long"));
    if (sys::get() == sys::State::Fault) sys::acknowledge();
  }
  for (uint8_t i = 0; i < hw::NUM_SWITCHES; i++) {
    if (limit_switches::changed(i) && w) {
      Out.print(F("EV switch ")); Out.print(hw::SWITCHES[i].name);
      Out.println(limit_switches::pressed(i) ? F(" pressed") : F(" released"));
    }
  }
}

void setup() {
  safety::begin();            // first: reset cause, watchdog off
  Serial.begin(CONSOLE_BAUD);
  status_led::begin();
  sys::begin();
  encoder::begin();
  limit_switches::begin();
  console::begin();
  drivers::begin();
  steppers::begin();
  motion::begin();
  homing::begin();
  calib::begin();
  settings::begin();          // EEPROM: before limits and rotation zero
  limits::begin();
  rotzero::begin();

  Out.print(F("CTR V2 firmware ")); Out.print(FW_VERSION);
  Out.print(F(" | profile: ")); Out.println(hw::PROFILE_NAME);
  drivers::report();
  Out.println(F("Type 'help' for commands."));
  safety::armWatchdog();      // last: from here the loop must keep running
}

void loop() {
  loop_stats::update();
  safety::update();
  encoder::update();
  limit_switches::update();
  homing::update();           // right after the switches: stop on the edge this pass
  calib::update();
  rotzero::update();
  status_led::update();
  motion::update();
  sys::update();
  console::update();
  handleEvents();
}
