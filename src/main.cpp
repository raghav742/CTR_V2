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
#include "display.h"
#include "buzzer.h"
#include "ui.h"
#include "demo.h"
#include "serial_io.h"

// Handle input events once per pass.
//   encoder turn                    -> UI (menu selection)
//   press while anything moves      -> STOP at once (press edge, no fault)
//   short press otherwise           -> UI select (on release, so a long press
//                                      never also selects)
//   long press (1 s)                -> UI back; on the error screen: confirm
// With "watch on", events are also printed (test aid).
static bool pressArmed = false;   // press started while idle: select on release

static void handleEvents() {
  const bool w = console::watching();
  int16_t d = encoder::takeDelta();
  if (d) {
    if (w) { Out.print(F("EV enc ")); Out.println(d); }
    ui::onTurn(d);
  }
  if (encoder::takePress()) {
    if (w) Out.println(F("EV btn press"));
    if (ui::motionActive()) {
      safety::userStop(F("encoder button"), encoder::pressEdgeUs());
      if (demo::active()) demo::stop();          // also between two demo moves
      pressArmed = false;
    } else {
      pressArmed = true;
    }
  }
  if (encoder::takeLongPress()) {
    if (w) Out.println(F("EV btn long"));
    if (pressArmed) ui::onBack();
    pressArmed = false;
  }
  if (encoder::takeRelease()) {
    if (w) Out.println(F("EV btn release"));
    if (pressArmed) ui::onSelect();
    pressArmed = false;
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
  demo::begin();
  calib::begin();
  settings::begin();          // EEPROM: before limits and rotation zero
  limits::begin();
  rotzero::begin();
  buzzer::begin();
  bool oled = display::begin();
  ui::begin();

  Out.print(F("CTR V2 firmware ")); Out.print(FW_VERSION);
  Out.print(F(" | profile: ")); Out.println(hw::PROFILE_NAME);
  drivers::report();
  Out.println(oled ? F("Display: OLED found") : F("Display: not found (check I2C SDA 20 / SCL 21)"));
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
  demo::update();
  buzzer::update();
  display::update();          // sends one small piece of the current frame
  ui::update();
  status_led::update();
  motion::update();
  sys::update();
  console::update();
  handleEvents();
}
