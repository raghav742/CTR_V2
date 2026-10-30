#include "demo.h"
#include "serial_io.h"
#include "hw_profile.h"
#include "motion_config.h"
#include "motion.h"
#include "limits.h"
#include "safety.h"
#include "rotation_zero.h"
#include "system_state.h"

namespace demo {
namespace {
  constexpr uint8_t C = hw::NUM_HOMED_AXES;
  constexpr int16_t KEEP = -32768;        // rotation: leave as it is

  struct Step {
    int16_t base, ext2, ext3;             // per mille (see demo.h)
    int16_t rot[3];                       // degrees, KEEP = unchanged
    uint16_t ms;                          // move takes at least this long
    uint16_t pause;                       // wait after the move
  };

  // ---------------- the programs ----------------
  // 1. Telescope: the assembly moves forward, then each inner tube extends,
  //    the tubes twist, and everything retracts again.
  const Step TELESCOPE[] PROGMEM = {
    {   0,   0,   0, {   0,    0,    0 }, 2500, 600 },   // collapsed at the back
    { 500,   0,   0, {   0,    0,    0 }, 3500, 300 },   // whole assembly forward
    { 500, 800,   0, {   0,    0,    0 }, 2500, 300 },   // T2 extends out of T1
    { 500, 800, 800, {   0,    0,    0 }, 2500, 700 },   // T3 extends out of T2
    { 500, 800, 800, { 360, -360,  720 }, 3500, 500 },   // twist
    { 500,   0,   0, {   0,    0,    0 }, 3500, 300 },   // retract and untwist
  };
  // 2. Wave: continuous flowing motion, tubes out of phase
  const Step WAVE[] PROGMEM = {
    { 300, 300, 300, {   0,    0,    0 }, 2500,   0 },
    { 600, 500, 200, {  90,  -90,  180 }, 1800,   0 },
    { 300, 700, 600, { -90,   90, -180 }, 1800,   0 },
    { 600, 300, 800, {  90,  -90,  180 }, 1800,   0 },
    { 300, 500, 400, {   0,    0,    0 }, 1800,   0 },
  };
  // 3. Spin: mostly rotation, small breathing of the tubes
  const Step SPIN[] PROGMEM = {
    { 200, 200, 200, {   0,    0,    0 }, 2000, 300 },
    { 200, 200, 200, { 720, -720, 1080 }, 4000, 300 },
    { 400, 400, 600, { 720, -720, 1080 }, 2500, 300 },
    { 200, 200, 200, {   0,    0,    0 }, 4000, 600 },
  };
  struct Program { const Step* steps; uint8_t n; };
  const Program PROGRAMS[] = {
    { TELESCOPE, sizeof(TELESCOPE) / sizeof(Step) },
    { WAVE,      sizeof(WAVE)      / sizeof(Step) },
    { SPIN,      sizeof(SPIN)      / sizeof(Step) },
  };
  constexpr uint8_t NPROG = sizeof(PROGRAMS) / sizeof(PROGRAMS[0]);

  // ---------------- state ----------------
  enum class Phase : uint8_t { Idle, Moving, Pausing };
  Phase    phase = Phase::Idle;
  uint8_t  prog = 0, cur = 0;
  uint16_t loopCount = 0;
  uint32_t pauseStart = 0, stopsAtStart = 0;
  float    rotBase[hw::NUM_AXES];          // rotation zero used by this run
  float    frontMax, extMax[C - 1];         // geometry, read at start

  Step stepAt(uint8_t i) { Step s; memcpy_P(&s, &PROGRAMS[prog].steps[i], sizeof(Step)); return s; }

  void finish(const __FlashStringHelper* why) {
    phase = Phase::Idle;
    safety::forceSpanRule(false);
    Out.print(F("EV demo stopped (")); Out.print(why); Out.print(F(") loops="));
    Out.println(loopCount);
    if (sys::get() == sys::State::Demo) sys::request(sys::State::Ready);
  }

  // Pose -> absolute target per axis -> one coordinated move
  bool moveTo(const Step& s) {
    float target[hw::NUM_AXES], amount[hw::NUM_AXES] = { 0 };
    uint8_t mask = 0;
    // translations: T1 = 2 mm + base * stroke; each next = previous + ext * usable gap
    float t = 2.0f + s.base / 1000.0f * (frontMax - 2.0f);
    target[hw::CARRIAGE_ORDER[0]] = t;
    const int16_t ext[2] = { s.ext2, s.ext3 };
    for (uint8_t k = 1; k < C; k++) {
      t += ext[k - 1] / 1000.0f * extMax[k - 1];
      target[hw::CARRIAGE_ORDER[k]] = t;
    }
    // rotations: tube k = axis 2k (V1 axis order: Rot, Tra per tube)
    for (uint8_t k = 0; k < 3; k++)
      if (s.rot[k] != KEEP) target[2 * k] = rotBase[2 * k] + s.rot[k];
      else target[2 * k] = motion::position(2 * k);
    for (uint8_t i = 0; i < hw::NUM_AXES; i++) {
      amount[i] = target[i] - motion::position(i);
      if (fabsf(amount[i]) >= 0.01f) mask |= _BV(i);
    }
    if (!mask) return true;                     // already there
    return motion::moveGroup(amount, mask, motion_cfg::DEMO_SPEED_PCT, -(s.ms / 1000.0f), 0, true);
  }

  void startStep() {
    Step s = stepAt(cur);
    if (!moveTo(s)) { finish(F("move refused, see ERR")); return; }
    phase = Phase::Moving;
  }
}

// ------------------------------------------------------------
void begin() { phase = Phase::Idle; }
uint8_t count() { return NPROG; }
bool active() { return phase != Phase::Idle; }
uint8_t program() { return prog; }
uint8_t step() { return cur; }
uint8_t steps() { return PROGRAMS[prog].n; }
uint16_t loops() { return loopCount; }

const __FlashStringHelper* name(uint8_t p) {
  switch (p) {
    case 0:  return F("Telescope");
    case 1:  return F("Wave");
    default: return F("Spin");
  }
}

bool start(uint8_t p, bool quiet) {
  if (p >= NPROG)        { Out.println(F("ERR demo 1..3")); return false; }
  if (active())          { Out.println(F("ERR demo already running")); return false; }
  if (motion::anyBusy()) { Out.println(F("ERR stop motion first")); return false; }
  if (!limits::geometry(frontMax, extMax)) { Out.println(F("ERR home first (limits not active)")); return false; }
  if (!sys::request(sys::State::Demo)) return false;

  // Rotations without a zero: use the current angle as zero for this run
  for (uint8_t i = 0; i < hw::NUM_AXES; i++)
    rotBase[i] = (hw::AXES[i].type == AxisType::Rotation && !rotzero::referenced(i)) ? motion::position(i) : 0;

  prog = p; cur = 0; loopCount = 0;
  stopsAtStart = motion::stopCount();
  safety::forceSpanRule(true);                 // big automatic moves: keep tubes supported
  if (!quiet) { Out.print(F("OK demo ")); Out.println(name(p)); }
  Out.print(F("EV demo start ")); Out.println(name(p));
  startStep();
  return active();
}

void stop() {
  if (!active()) { Out.println(F("OK demo not running")); return; }
  motion::stopAll(F("demo stop"));             // update() then reports the stop
}

void update() {
  if (!active()) return;
  if (motion::stopCount() != stopsAtStart) { finish(F("stop")); return; }
  if (sys::get() != sys::State::Demo)      { finish(F("state changed")); return; }

  if (phase == Phase::Moving) {
    if (motion::anyBusy()) return;
    phase = Phase::Pausing;
    pauseStart = millis();
  }
  if (phase == Phase::Pausing) {
    if (millis() - pauseStart < stepAt(cur).pause) return;
    if (++cur >= PROGRAMS[prog].n) { cur = 0; loopCount++; }
    startStep();
  }
}
}  // namespace demo
