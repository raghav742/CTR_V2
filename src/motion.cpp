#include "motion.h"
#include "hw_profile.h"
#include "motion_config.h"
#include "steppers.h"
#include "drivers.h"
#include "limit_switches.h"

namespace motion {
namespace {
  struct Ramp {
    float v;        // current speed (steps/s)
    float vMax;     // cruise speed for this move (steps/s)
    float vMin;     // start/stop speed (steps/s)
    float accel;    // steps/s^2
    bool  moving;   // a move is in progress (for the "done" event)
  };
  Ramp     ramp[hw::NUM_AXES];
  uint32_t lastRampUs = 0;
  bool     swPrev[hw::NUM_SWITCHES];

  bool isTra(uint8_t i) { return hw::AXES[i].type == AxisType::Translation; }

  // speed in user units -> steps/s  (rpm -> deg/s = rpm * 6)
  float toSps(uint8_t i, float speed) {
    return isTra(i) ? speed * stepsPerUnit(i) : speed * 6.0f * stepsPerUnit(i);
  }

  void printDone(uint8_t i, bool stopped) {
    Serial.print(stopped ? F("EV stopped ") : F("EV done "));
    Serial.print(i + 1); Serial.print(' '); Serial.print(hw::AXES[i].name);
    Serial.print(F(" pos=")); Serial.print(position(i), 2); Serial.println(unit(i));
  }
}

float stepsPerUnit(uint8_t i) {
  float ms = drivers::microsteps(i);
  return isTra(i) ? (hw::FULLSTEPS_PER_REV * ms) / hw::LEAD_MM
                  : (hw::FULLSTEPS_PER_REV * ms) / 360.0f;
}

const __FlashStringHelper* unit(uint8_t i) { return isTra(i) ? F("mm") : F("deg"); }

float position(uint8_t i) { return steppers::position(i) / stepsPerUnit(i); }

void begin() {
  for (uint8_t i = 0; i < hw::NUM_AXES; i++) ramp[i] = Ramp{0, 0, 0, 0, false};
  for (uint8_t s = 0; s < hw::NUM_SWITCHES; s++) swPrev[s] = limit_switches::pressed(s);
  lastRampUs = micros();
}

bool busy(uint8_t i) { return steppers::running(i); }

bool anyBusy() {
  for (uint8_t i = 0; i < hw::NUM_AXES; i++) if (steppers::running(i)) return true;
  return false;
}

bool moveRelative(uint8_t i, float amount, float speed) {
  if (i >= hw::NUM_AXES)       { Serial.println(F("ERR axis 1..6")); return false; }
  if (!drivers::present(i))    { Serial.println(F("ERR driver not found")); return false; }
  if (!drivers::enabled(i))    { Serial.println(F("ERR axis disabled")); return false; }
  if (busy(i))                 { Serial.println(F("ERR axis busy")); return false; }

  // Temporary safety net: refuse moving into a switch that is already pressed
  if (isTra(i) && fabsf(amount) > motion_cfg::TRA_MAX_MOVE_UNHOMED) {
    Serial.print(F("ERR translation moves limited to "));
    Serial.print(motion_cfg::TRA_MAX_MOVE_UNHOMED, 0); Serial.println(F(" mm until homing exists"));
    return false;
  }
  if (isTra(i)) {
    int8_t home = limit_switches::findHome(i), far = limit_switches::findFarEnd(i);
    if (amount < 0 && home >= 0 && limit_switches::pressed(home)) { Serial.println(F("ERR home switch pressed")); return false; }
    if (amount > 0 && far  >= 0 && limit_switches::pressed(far))  { Serial.println(F("ERR far-end switch pressed")); return false; }
  }

  float def  = isTra(i) ? motion_cfg::TRA_SPEED_DEFAULT : motion_cfg::ROT_SPEED_DEFAULT;
  float vmax = isTra(i) ? motion_cfg::TRA_SPEED_MAX     : motion_cfg::ROT_SPEED_MAX;
  if (speed <= 0)   speed = def;
  if (speed > vmax) speed = vmax;

  Ramp& r = ramp[i];
  r.vMax  = toSps(i, speed);
  r.vMin  = toSps(i, isTra(i) ? motion_cfg::TRA_SPEED_MIN : motion_cfg::ROT_SPEED_MIN);
  r.accel = isTra(i) ? motion_cfg::TRA_ACCEL * stepsPerUnit(i)
                     : motion_cfg::ROT_ACCEL * 6.0f * stepsPerUnit(i);
  if (r.vMin > r.vMax) r.vMin = r.vMax;
  r.v = r.vMin;
  r.moving = true;

  int32_t target = steppers::position(i) + (int32_t)lround(amount * stepsPerUnit(i));
  steppers::setRate(i, r.v);
  steppers::start(i, target);
  Serial.print(F("OK move ")); Serial.print(i + 1); Serial.print(' ');
  Serial.print(amount, 2); Serial.print(unit(i)); Serial.print(F(" at "));
  Serial.print(speed, 1); Serial.println(isTra(i) ? F(" mm/s") : F(" rpm"));
  return true;
}

void stopAll(const __FlashStringHelper* reason) {
  bool any = anyBusy();
  steppers::stopAll();
  for (uint8_t i = 0; i < hw::NUM_AXES; i++) {
    if (ramp[i].moving) { ramp[i].moving = false; printDone(i, true); }
  }
  if (any) { Serial.print(F("EV stop ")); Serial.println(reason); }
}

void update() {
  // --- temporary safety net: a switch becoming pressed while moving ---
  for (uint8_t s = 0; s < hw::NUM_SWITCHES; s++) {
    bool p = limit_switches::pressed(s);
    if (p && !swPrev[s] && anyBusy()) {
      swPrev[s] = p;
      Serial.print(F("EV switch ")); Serial.print(hw::SWITCHES[s].name); Serial.println(F(" hit"));
      stopAll(F("limit switch"));
      return;
    }
    swPrev[s] = p;
  }

  // --- ramps: recalculated every RAMP_UPDATE_US ---
  uint32_t now = micros();
  uint32_t dtUs = now - lastRampUs;
  if (dtUs < motion_cfg::RAMP_UPDATE_US) return;
  lastRampUs = now;
  float dt = dtUs * 1e-6f;

  for (uint8_t i = 0; i < hw::NUM_AXES; i++) {
    Ramp& r = ramp[i];
    if (!r.moving) continue;
    if (!steppers::running(i)) {             // interrupt reached the target
      r.moving = false;
      printDone(i, false);
      continue;
    }
    // Speed for this tick: accelerate, cruise, or brake in time.
    // Braking speed from the distance left: v = sqrt(2 * a * d)
    float vUp   = r.v + r.accel * dt;
    float vStop = sqrtf(2.0f * r.accel * (float)steppers::remaining(i));
    float v = vUp;
    if (v > r.vMax)  v = r.vMax;
    if (v > vStop)   v = vStop;
    if (v < r.vMin)  v = r.vMin;
    r.v = v;
    steppers::setRate(i, v);
  }
}
}  // namespace motion
