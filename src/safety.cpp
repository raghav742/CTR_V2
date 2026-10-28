#include "safety.h"
#include "serial_io.h"
#include <avr/wdt.h>
#include "hw_profile.h"
#include "motion_config.h"
#include "limit_switches.h"
#include "drivers.h"
#include "motion.h"
#include "system_state.h"

// Survive a reset (not cleared by startup code): lets us tell after a
// restart that the watchdog fired.
static volatile uint16_t wdtMagic __attribute__((section(".noinit")));
static constexpr uint16_t WDT_MAGIC = 0xBEEF;

namespace safety {
namespace {
  bool     swPrev[hw::NUM_SWITCHES];
  bool     swExpected[hw::NUM_SWITCHES];
  uint32_t lastDrvCheck = 0;
  uint8_t  drvAxis = 0;
  uint32_t lastReactionUs = 0;
  enum Cause : uint8_t { C_UNKNOWN, C_WATCHDOG, C_BROWNOUT, C_EXTERNAL, C_POWERON };
  uint8_t  resetCause = C_UNKNOWN;

  const __FlashStringHelper* causeText() {
    switch (resetCause) {
      case C_WATCHDOG: return F("watchdog");
      case C_BROWNOUT: return F("brown-out (supply dip)");
      case C_EXTERNAL: return F("reset button / upload");
      case C_POWERON:  return F("power-on");
      default:         return F("unknown");
    }
  }

  bool isTra(uint8_t i) { return hw::AXES[i].type == AxisType::Translation; }

  void reportReaction(uint32_t edgeUs) {
    lastReactionUs = micros() - edgeUs;
    Out.print(F("EV reaction_ms=")); Out.println(lastReactionUs / 1000.0f, 1);
  }
}

// ------------------------------------------------------------
void begin() {
  uint8_t mcusr = MCUSR;
  MCUSR = 0;
  wdt_disable();                       // in case we restarted from the watchdog
  if (wdtMagic == WDT_MAGIC)   resetCause = C_WATCHDOG;
  else if (mcusr & _BV(BORF))  resetCause = C_BROWNOUT;
  else if (mcusr & _BV(EXTRF)) resetCause = C_EXTERNAL;
  else if (mcusr & _BV(PORF))  resetCause = C_POWERON;
  wdtMagic = 0;

  for (uint8_t s = 0; s < hw::NUM_SWITCHES; s++) { swPrev[s] = false; swExpected[s] = false; }
}

void armWatchdog() {
  // Interrupt + reset mode, 1 s: first timeout runs ISR(WDT_vect) (stops
  // step pulses, marks the cause), the next timeout resets the board.
  cli();
  wdt_reset();
  WDTCSR = _BV(WDCE) | _BV(WDE);
  WDTCSR = _BV(WDIE) | _BV(WDE) | _BV(WDP2) | _BV(WDP1);   // 1 s
  sei();
  for (uint8_t s = 0; s < hw::NUM_SWITCHES; s++) swPrev[s] = limit_switches::pressed(s);
  Out.print(F("EV boot reset_cause=")); Out.println(causeText());
}

// ------------------------------------------------------------
bool checkMove(uint8_t i, float amount) {
  const char* name = hw::AXES[i].name;
  if (!drivers::present(i)) { Out.print(F("ERR ")); Out.print(name); Out.println(F(" driver not found")); return false; }
  if (!drivers::enabled(i)) { Out.print(F("ERR ")); Out.print(name); Out.println(F(" disabled")); return false; }
  if (!isTra(i)) return true;

  // Until homing exists, positions are only counted steps
  if (!sys::homed() && fabsf(amount) > motion_cfg::TRA_MAX_MOVE_UNHOMED) {
    Out.print(F("ERR translation moves limited to "));
    Out.print(motion_cfg::TRA_MAX_MOVE_UNHOMED, 0); Out.println(F(" mm until homed"));
    return false;
  }
  // No moving further into a switch that is already pressed
  int8_t home = limit_switches::findHome(i), far = limit_switches::findFarEnd(i);
  if (amount < 0 && home >= 0 && limit_switches::pressed(home)) {
    Out.print(F("ERR ")); Out.print(name); Out.println(F(" home switch pressed")); return false; }
  if (amount > 0 && far >= 0 && limit_switches::pressed(far)) {
    Out.print(F("ERR ")); Out.print(name); Out.println(F(" far-end switch pressed")); return false; }
  return true;
}

void expectSwitch(int8_t s, bool expected) {
  if (s >= 0 && s < (int8_t)hw::NUM_SWITCHES) swExpected[s] = expected;
}

void userStop(const __FlashStringHelper* source, uint32_t edgeUs) {
  if (!motion::anyBusy()) return;
  motion::stopAll(source);
  reportReaction(edgeUs);
}

// ------------------------------------------------------------
void update() {
  wdt_reset();   // "still alive": must happen at least once per second

  // --- limit switches: a new press while anything moves ---
  for (uint8_t s = 0; s < hw::NUM_SWITCHES; s++) {
    bool p = limit_switches::pressed(s);
    if (p && !swPrev[s] && motion::anyBusy() && !swExpected[s]) {
      swPrev[s] = p;
      char buf[24];
      snprintf(buf, sizeof(buf), "%s hit", hw::SWITCHES[s].name);
      sys::fault(buf);                                  // stops all motion
      reportReaction(limit_switches::rawEdgeUs(s));
      return;
    }
    swPrev[s] = p;
  }

  // --- driver health: one driver every DRV_CHECK_MS (one check blocks ~9 ms, D7) ---
  uint32_t now = millis();
  if (now - lastDrvCheck >= motion_cfg::DRV_CHECK_MS && sys::get() != sys::State::Fault) {
    lastDrvCheck = now;
    uint8_t i = drvAxis;
    drvAxis = (drvAxis + 1) % hw::NUM_AXES;
    if (drivers::present(i)) {
      char buf[32];
      if (!drivers::alive(i)) {
        snprintf(buf, sizeof(buf), "%s driver not answering", hw::AXES[i].name);
        sys::fault(buf);
      } else if (drivers::overTemp(i)) {
        snprintf(buf, sizeof(buf), "%s driver overheated", hw::AXES[i].name);
        sys::fault(buf);
      }
    }
  }
}

void report() {
  Out.print(F("OK reset_cause=")); Out.print(causeText());
  Out.print(F(" last_reaction_ms=")); Out.print(lastReactionUs / 1000.0f, 1);
  Out.print(F(" expected_switches="));
  bool any = false;
  for (uint8_t s = 0; s < hw::NUM_SWITCHES; s++)
    if (swExpected[s]) { if (any) Out.print(','); Out.print(hw::SWITCHES[s].name); any = true; }
  if (!any) Out.print(F("none"));
  Out.println(F(" watchdog=1s"));
}

void watchdogTest() {
  if (motion::anyBusy()) { Out.println(F("ERR stop motion first")); return; }
  Out.println(F("OK freezing now; watchdog should restart the board in ~2 s"));
  Serial.flush();
  for (;;) { }   // never feeds the watchdog
}
}  // namespace safety

// First watchdog timeout: stop all step pulses immediately and remember why.
// The second timeout (1 s later) resets the board.
ISR(WDT_vect) {
  TIMSK1 &= ~_BV(OCIE1A);   // Timer1 step interrupt off -> no more step pulses
  wdtMagic = WDT_MAGIC;
}
