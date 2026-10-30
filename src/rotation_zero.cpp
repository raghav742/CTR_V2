#include "rotation_zero.h"
#include "serial_io.h"
#include "hw_profile.h"
#include "motion_config.h"
#include "motion.h"
#include "steppers.h"
#include "settings.h"
#include "homing.h"
#include "system_state.h"

namespace rotzero {
namespace {
  constexpr uint8_t N = hw::NUM_AXES;
  bool     ref[N];
  bool     parkRunning = false;
  uint8_t  parkMask = 0;
  uint32_t stopsAtStart = 0;

  bool isRot(uint8_t i) { return hw::AXES[i].type == AxisType::Rotation; }

  bool allRotReferenced() {
    for (uint8_t i = 0; i < N; i++) if (isRot(i) && !ref[i]) return false;
    return true;
  }
}

void begin() {
  for (uint8_t i = 0; i < N; i++) ref[i] = false;
  if (settings::parked()) {
    for (uint8_t i = 0; i < N; i++) if (isRot(i)) ref[i] = true;
    Out.println(F("EV rotation zero restored (robot was parked)"));
  } else {
    Out.println(F("EV rotation zero unknown (not parked): align and use 'zero'"));
  }
}

bool referenced(uint8_t i) { return i < N && isRot(i) && ref[i]; }
bool parking() { return parkRunning; }

bool setZero(int8_t only, bool quiet) {
  bool any = false;
  for (uint8_t i = 0; i < N; i++) {
    if (!isRot(i) || (only >= 0 && only != i)) continue;
    if (motion::busy(i)) { Out.print(F("ERR ")); Out.print(hw::AXES[i].name); Out.println(F(" busy")); return false; }
    steppers::setPosition(i, 0);
    ref[i] = true;
    any = true;
  }
  if (!any) { Out.println(F("ERR usage: zero <1|3|5|all> (rotation axes)")); return false; }
  Out.print(quiet ? F("EV zero set") : F("OK zero"));
  for (uint8_t i = 0; i < N; i++)
    if (isRot(i) && (only < 0 || only == i)) { Out.print(' '); Out.print(hw::AXES[i].name); }
  Out.println();
  return true;
}

bool park(bool quiet) {
  if (parkRunning)       { Out.println(F("ERR park already running")); return false; }
  if (motion::anyBusy()) { Out.println(F("ERR stop motion first")); return false; }

  float amount[N] = { 0 };
  parkMask = 0;
  for (uint8_t i = 0; i < N; i++) {
    float p = motion::position(i);
    if (isRot(i) && ref[i]) {
      amount[i] = roundf(p / 360.0f) * 360.0f - p;        // nearest full turn of zero
    } else if (!isRot(i) && homing::axisHomed(i)) {
      amount[i] = motion_cfg::CALIB_REST_MM - p;          // carriages to rest
    } else continue;
    parkMask |= _BV(i);
  }
  if (!parkMask) { Out.println(F("ERR nothing referenced to park (home / zero first)")); return false; }
  for (uint8_t i = 0; i < N; i++)
    if (isRot(i) && !ref[i]) { Out.print(F("EV park ")); Out.print(hw::AXES[i].name); Out.println(F(" has no zero: not parked")); }

  // one coordinated move: carriages stay in proportion (limits checked as usual)
  if (!motion::moveGroup(amount, parkMask, motion_cfg::PARK_SPEED_PCT, 0, 0, quiet)) return false;
  parkRunning = true;
  stopsAtStart = motion::stopCount();
  return true;
}

void update() {
  // The first rotation move after a park makes the stored zeros invalid
  if (settings::parked() && !parkRunning)
    for (uint8_t i = 0; i < N; i++)
      if (isRot(i) && motion::busy(i)) { settings::setParked(false); break; }

  if (!parkRunning) return;
  if (motion::stopCount() != stopsAtStart) { parkRunning = false; Out.println(F("EV park aborted")); return; }
  for (uint8_t i = 0; i < N; i++) if ((parkMask & _BV(i)) && motion::busy(i)) return;

  // Done: rotations now exactly on a full turn -> call that 0
  parkRunning = false;
  for (uint8_t i = 0; i < N; i++) if (isRot(i) && ref[i]) steppers::setPosition(i, 0);
  if (allRotReferenced()) {
    settings::setParked(true);
    Out.println(F("EV park done: safe to power off (rotation zeros kept)"));
  } else {
    Out.println(F("EV park done, but not all rotations have a zero: they will be unknown after power-off"));
  }
}
}  // namespace rotzero
