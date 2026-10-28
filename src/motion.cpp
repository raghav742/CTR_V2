#include "motion.h"
#include "hw_profile.h"
#include "motion_config.h"
#include "steppers.h"
#include "drivers.h"
#include "limit_switches.h"
#include "system_state.h"
#include "safety.h"

// ============================================================
//  Coordinated motion with a shared "progress" profile.
//  A move (group) has one progress value p: 0 -> 1. The profile
//  of p (accelerate, cruise, brake) is shared by all axes in the
//  group; each axis' speed = dp/dt * its own distance. So every
//  axis starts, accelerates and finishes together.
//  Limits: the progress speed and acceleration are chosen so no
//  axis exceeds its own maximum speed or acceleration.
// ============================================================
namespace motion {
namespace {
  constexpr uint8_t N = hw::NUM_AXES;

  struct Group {
    bool     active;
    uint8_t  mask;        // bit i = axis i belongs to this group
    uint8_t  master;      // axis with the longest distance (in steps)
    float    d[N];        // distance per axis in microsteps (absolute)
    float    vp;          // current progress speed (1/s)
    float    vpMax;       // cruise progress speed
    float    vpMin;       // start/stop progress speed
    float    ap;          // progress acceleration (1/s^2)
    uint32_t startMs;
  };
  Group    groups[N];               // at most one group per axis
  int8_t   axisGroup[N];            // group of each axis, -1 = idle
  uint32_t lastRampUs = 0;

  bool isTra(uint8_t i) { return hw::AXES[i].type == AxisType::Translation; }

  // Per-axis limits in microsteps/s and microsteps/s^2
  float vMaxSps(uint8_t i) { return isTra(i) ? motion_cfg::TRA_SPEED_MAX * stepsPerUnit(i)
                                             : motion_cfg::ROT_SPEED_MAX * 6.0f * stepsPerUnit(i); }
  float vMinSps(uint8_t i) { return isTra(i) ? motion_cfg::TRA_SPEED_MIN * stepsPerUnit(i)
                                             : motion_cfg::ROT_SPEED_MIN * 6.0f * stepsPerUnit(i); }
  float accSps(uint8_t i)  { return isTra(i) ? motion_cfg::TRA_ACCEL * stepsPerUnit(i)
                                             : motion_cfg::ROT_ACCEL * 6.0f * stepsPerUnit(i); }
  // user speed (mm/s or rpm) -> microsteps/s
  float toSps(uint8_t i, float speed) { return isTra(i) ? speed * stepsPerUnit(i)
                                                        : speed * 6.0f * stepsPerUnit(i); }

  // Time of a trapezoid over progress 1 with given cruise speed and acceleration
  float profileTime(float vp, float ap) {
    if (vp * vp / ap <= 1.0f) return 1.0f / vp + vp / ap;   // reaches cruise
    return 2.0f * sqrtf(1.0f / ap);                        // triangle
  }

  void printDone(Group& g, bool stopped) {
    Serial.print(stopped ? F("EV stopped") : F("EV done"));
    Serial.print(F(" t=")); Serial.print((millis() - g.startMs) / 1000.0f, 2); Serial.print('s');
    for (uint8_t i = 0; i < N; i++) {
      if (!(g.mask & _BV(i))) continue;
      Serial.print(' '); Serial.print(hw::AXES[i].name); Serial.print('=');
      Serial.print(position(i), 2);
    }
    Serial.println();
  }

  void finishGroup(uint8_t k, bool stopped) {
    Group& g = groups[k];
    if (!g.active) return;
    printDone(g, stopped);
    for (uint8_t i = 0; i < N; i++) if (g.mask & _BV(i)) axisGroup[i] = -1;
    g.active = false;
  }

  // Validate one axis: busy here, everything else in the safety manager
  bool axisOk(uint8_t i, float amount) {
    if (axisGroup[i] >= 0) { Serial.print(F("ERR ")); Serial.print(hw::AXES[i].name); Serial.println(F(" busy")); return false; }
    return safety::checkMove(i, amount);
  }
}

// ------------------------------------------------------------
float stepsPerUnit(uint8_t i) {
  float ms = drivers::microsteps(i);
  return isTra(i) ? (hw::FULLSTEPS_PER_REV * ms) / hw::LEAD_MM
                  : (hw::FULLSTEPS_PER_REV * ms) / 360.0f;
}
const __FlashStringHelper* unit(uint8_t i) { return isTra(i) ? F("mm") : F("deg"); }
float position(uint8_t i) { return steppers::position(i) / stepsPerUnit(i); }

void begin() {
  for (uint8_t k = 0; k < N; k++) groups[k].active = false;
  for (uint8_t i = 0; i < N; i++) axisGroup[i] = -1;
  lastRampUs = micros();
}

bool busy(uint8_t i) { return i < N && axisGroup[i] >= 0; }
bool anyBusy() {
  for (uint8_t i = 0; i < N; i++) if (axisGroup[i] >= 0) return true;
  return false;
}

// ------------------------------------------------------------
//  Start a coordinated move.
//  amount[i]: relative distance (mm or deg) for axes in 'mask'.
//  speedPct:  0 < pct <= 100 of the fastest allowed (default 100)
//  duration:  > 0 = take exactly this many seconds (if possible)
//  singleSpeed: user speed for a single-axis move (0 = ignore)
// ------------------------------------------------------------
bool moveGroup(const float amount[], uint8_t mask, float speedPct, float duration, float singleSpeed) {
  if (!sys::motionAllowed()) {
    Serial.print(F("ERR not allowed in state ")); Serial.print(sys::name(sys::get()));
    if (sys::get() == sys::State::Fault) { Serial.print(F(": ")); Serial.print(sys::faultReason()); Serial.print(F(" (ack first)")); }
    Serial.println();
    return false;
  }
  if (!mask) { Serial.println(F("ERR no axes")); return false; }
  for (uint8_t i = 0; i < N; i++)
    if ((mask & _BV(i)) && !axisOk(i, amount[i])) return false;

  int8_t k = -1;
  for (uint8_t j = 0; j < N; j++) if (!groups[j].active) { k = j; break; }
  if (k < 0) { Serial.println(F("ERR no free move slot")); return false; }
  Group& g = groups[k];

  // Distances in steps; derive the shared progress limits
  float vpLimit = 1e9f, apLimit = 1e9f, dMax = 0;
  int32_t target[N];
  g.mask = 0; g.master = 0;
  for (uint8_t i = 0; i < N; i++) {
    g.d[i] = 0;
    if (!(mask & _BV(i))) continue;
    int32_t steps = (int32_t)lround(amount[i] * stepsPerUnit(i));
    if (steps == 0) continue;
    g.d[i] = fabsf((float)steps);
    target[i] = steppers::position(i) + steps;
    g.mask |= _BV(i);
    float vmax = (singleSpeed > 0) ? toSps(i, singleSpeed) : vMaxSps(i);
    if (vmax > vMaxSps(i)) vmax = vMaxSps(i);
    vpLimit = fminf(vpLimit, vmax / g.d[i]);
    apLimit = fminf(apLimit, accSps(i) / g.d[i]);
    if (g.d[i] > dMax) { dMax = g.d[i]; g.master = i; }
  }
  if (!g.mask) { Serial.println(F("OK nothing to move")); return true; }

  if (speedPct <= 0 || speedPct > 100) speedPct = 100;
  g.ap    = apLimit;
  g.vpMax = vpLimit * speedPct / 100.0f;

  if (duration > 0) {
    // Solve t = 1/vp + vp/ap for vp (smaller root = slower cruise)
    float disc = g.ap * g.ap * duration * duration - 4.0f * g.ap;
    float tMin = profileTime(vpLimit, g.ap);
    if (disc < 0 || duration < tMin) {
      Serial.print(F("ERR too short, minimum t=")); Serial.print(tMin, 2); Serial.println('s');
      return false;
    }
    g.vpMax = (g.ap * duration - sqrtf(disc)) / 2.0f;
  }

  g.vpMin = fminf(vMinSps(g.master) / g.d[g.master], g.vpMax);
  g.vp    = g.vpMin;
  g.startMs = millis();
  g.active  = true;

  for (uint8_t i = 0; i < N; i++) {
    if (!(g.mask & _BV(i))) continue;
    axisGroup[i] = k;
    steppers::setRate(i, g.vp * g.d[i]);
    steppers::start(i, target[i]);
  }

  Serial.print(F("OK mv"));
  for (uint8_t i = 0; i < N; i++) {
    if (!(g.mask & _BV(i))) continue;
    Serial.print(' '); Serial.print(hw::AXES[i].name); Serial.print('=');
    Serial.print(amount[i], 2); Serial.print(unit(i));
  }
  Serial.print(F(" est_t=")); Serial.print(profileTime(g.vpMax, g.ap), 2); Serial.println('s');
  return true;
}

bool moveRelative(uint8_t i, float amount, float speed) {
  if (i >= N) { Serial.println(F("ERR axis 1..6")); return false; }
  float a[N] = { 0 };
  a[i] = amount;
  if (speed <= 0) speed = isTra(i) ? motion_cfg::TRA_SPEED_DEFAULT : motion_cfg::ROT_SPEED_DEFAULT;
  return moveGroup(a, _BV(i), 100, 0, speed);
}

void stopAll(const __FlashStringHelper* reason) {
  bool any = anyBusy();
  steppers::stopAll();
  for (uint8_t k = 0; k < N; k++) finishGroup(k, true);
  if (any) { Serial.print(F("EV stop ")); Serial.println(reason); }
}

// ------------------------------------------------------------
void update() {
  // --- shared progress ramps, every RAMP_UPDATE_US ---
  uint32_t now = micros();
  uint32_t dtUs = now - lastRampUs;
  if (dtUs < motion_cfg::RAMP_UPDATE_US) return;
  lastRampUs = now;
  float dt = dtUs * 1e-6f;

  for (uint8_t k = 0; k < N; k++) {
    Group& g = groups[k];
    if (!g.active) continue;

    bool anyRunning = false;
    for (uint8_t i = 0; i < N; i++) if ((g.mask & _BV(i)) && steppers::running(i)) anyRunning = true;
    if (!anyRunning) { finishGroup(k, false); continue; }

    // Remaining progress, measured on the longest axis
    float remP = steppers::remaining(g.master) / g.d[g.master];
    float vUp   = g.vp + g.ap * dt;
    float vStop = sqrtf(2.0f * g.ap * remP);
    float v = vUp;
    if (v > g.vpMax) v = g.vpMax;
    if (v > vStop)   v = vStop;
    if (v < g.vpMin) v = g.vpMin;
    g.vp = v;

    for (uint8_t i = 0; i < N; i++)
      if ((g.mask & _BV(i)) && steppers::running(i)) steppers::setRate(i, v * g.d[i]);
  }
}
}  // namespace motion
