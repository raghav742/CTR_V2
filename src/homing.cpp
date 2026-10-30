#include "homing.h"
#include "serial_io.h"
#include "hw_profile.h"
#include "motion_config.h"
#include "limit_switches.h"
#include "motion.h"
#include "steppers.h"
#include "safety.h"
#include "system_state.h"

namespace homing {
namespace {
  constexpr uint8_t N = hw::NUM_AXES;

  // ---- Part 1: fast approach, all carriages together (Phase::Group) ----
  // All carriages that have not found their switch yet move backward in ONE
  // coordinated move, at the same speed, so the gaps between them stay the
  // same (no tube is left hanging). When any of them reaches its switch, the
  // whole move stops; the others then continue together. Two carriages can
  // only close in when the one behind has already stopped at its block, and
  // because there is a gap between all carriages at home, the one in front
  // always reaches its own block first: the switch can only be pressed by the
  // block, never by the carriage behind.
  //
  // ---- Part 2: slow approach, one carriage at a time (back carriage first) ----
  //   Clear  : switch pressed -> move forward until it releases
  //   Extra  : then a bit further (2 mm before the slow approach, 1 mm at the end)
  //   Seek   : move backward until the switch presses
  // stage 0: no fast hit for this carriage (do its own fast approach first),
  //       1: fast hit -> slow approach next, 2: zero set -> clear and finish
  enum class Phase : uint8_t { Idle, Group, Clear, Extra, Seek };
  Phase    phase = Phase::Idle;
  uint8_t  queue[N];
  uint8_t  qLen = 0, qPos = 0;
  uint8_t  axis = 0;
  int8_t   sw   = -1;
  uint8_t  stage  = 0;               // see above
  bool     found[N];                 // fast approach reached the switch
  uint32_t stopsAtStart = 0;         // motion::stopCount() when homing started
  uint32_t startMs = 0;
  bool     homedAxis[N];

  void name() { Out.print(hw::AXES[axis].name); }

  bool allHomed() {
    for (uint8_t k = 0; k < hw::NUM_HOMED_AXES; k++)
      if (!homedAxis[hw::HOME_ORDER[k]]) return false;
    return true;
  }

  // Internal move: allowed past the limits, prints nothing on success
  bool move(float mm, float speed = motion_cfg::HOME_SPEED) {
    safety::setHomingMove(true);
    bool ok = motion::moveRelative(axis, mm, speed, true);
    safety::setHomingMove(false);
    return ok;
  }

  void expectAll(bool on) {
    for (uint8_t k = 0; k < qLen; k++) safety::expectSwitch(limit_switches::findHome(queue[k]), on);
  }

  void reset() {
    expectAll(false);
    sw = -1;
    phase = Phase::Idle;
  }

  // Homing could not complete: hardware problem -> FAULT
  void fail(const char* why) {
    char buf[40];
    snprintf(buf, sizeof(buf), "%s home: %s", hw::AXES[axis].name, why);
    Out.print(F("EV home failed ")); Out.println(buf);
    reset();
    sys::fault(buf);
  }

  void startSeek() {
    bool ok = (stage == 0)
      ? move(-motion_cfg::HOME_SEARCH_MM, motion_cfg::HOME_FAST_SPEED)                     // fast, far
      : move(-(motion_cfg::HOME_REAPPROACH_MM + motion_cfg::HOME_CLEAR_MM), motion_cfg::HOME_SPEED); // slow, short
    if (!ok) { fail("seek refused"); return; }
    phase = Phase::Seek;
  }

  void startClear() {
    if (!move(motion_cfg::HOME_CLEAR_MM)) { fail("clear refused"); return; }
    phase = Phase::Clear;
  }

  // Part 2 for one carriage
  void startAxis() {
    axis = queue[qPos];
    sw   = limit_switches::findHome(axis);
    if (sw < 0) { fail("no home switch"); return; }
    // Normally the switch is pressed after the group approach: back off and come in slowly.
    // If not (e.g. it was pressed by the carriage behind at the start), do a fast approach first.
    stage = limit_switches::pressed(sw) ? 1 : 0;
    if (stage == 1) startClear();
    else            startSeek();
  }

  void axisDone() {
    homedAxis[axis] = true;
    Out.print(F("EV home ")); name(); Out.print(F(" ok pos=")); Out.println(motion::position(axis), 2);
    if (++qPos < qLen) { startAxis(); return; }

    reset();
    Out.print(F("EV home done t=")); Out.print((millis() - startMs) / 1000.0f, 1); Out.println('s');
    sys::setHomed(allHomed());
    sys::request(allHomed() ? sys::State::Ready : sys::State::Idle);
  }

  // Part 1: one coordinated backward move of every carriage still searching
  void startGroup() {
    float amount[N] = { 0 };
    uint8_t mask = 0;
    for (uint8_t k = 0; k < qLen; k++) {
      uint8_t a = queue[k];
      if (found[a]) continue;
      amount[a] = -motion_cfg::HOME_SEARCH_MM;   // equal distances + one speed = equal speeds
      mask |= _BV(a);
    }
    if (!mask) {                                 // all at their switch: slow approaches
      qPos = 0;
      startAxis();
      return;
    }
    safety::setHomingMove(true);
    bool ok = motion::moveGroup(amount, mask, 100, 0, motion_cfg::HOME_FAST_SPEED, true);
    safety::setHomingMove(false);
    if (!ok) { fail("group move refused"); return; }
    phase = Phase::Group;
  }

  void updateGroup() {
    bool anyHit = false, anyMoving = false;
    for (uint8_t k = 0; k < qLen; k++) {
      uint8_t a = queue[k];
      if (found[a]) continue;
      if (motion::busy(a)) anyMoving = true;
      if (limit_switches::pressed(limit_switches::findHome(a))) anyHit = true;
    }
    if (anyHit) {
      // stop the whole group, note every carriage now at its switch
      for (uint8_t k = 0; k < qLen; k++) if (!found[queue[k]]) { motion::stopGroupOf(queue[k]); break; }
      for (uint8_t k = 0; k < qLen; k++) {
        uint8_t a = queue[k];
        if (!found[a] && limit_switches::pressed(limit_switches::findHome(a))) {
          found[a] = true;
          Out.print(F("EV home ")); Out.print(hw::AXES[a].name);
          Out.print(F(" switch reached (fast) at ")); Out.print(motion::position(a), 2); Out.println(F(" mm"));
        }
      }
      startGroup();                              // the rest continue together
    } else if (!anyMoving) {
      for (uint8_t k = 0; k < qLen; k++) if (!found[queue[k]]) { axis = queue[k]; break; }
      fail("switch not found");
    }
  }
}

// ------------------------------------------------------------
void begin() {
  for (uint8_t i = 0; i < N; i++) homedAxis[i] = false;
  phase = Phase::Idle;
}

bool active() { return phase != Phase::Idle; }
bool axisHomed(uint8_t i) { return i < N && homedAxis[i]; }

void invalidate(uint8_t i) {
  if (i >= N || !homedAxis[i]) return;
  homedAxis[i] = false;
  sys::setHomed(false);
  Out.print(F("EV home ")); Out.print(hw::AXES[i].name); Out.println(F(" lost (zero no longer valid)"));
}

bool start(int8_t only, bool quiet) {
  if (active() || sys::get() == sys::State::Homing) { Out.println(F("ERR homing or calib already running")); return false; }
  if (motion::anyBusy()) { Out.println(F("ERR stop motion first")); return false; }

  qLen = 0; qPos = 0;
  for (uint8_t k = 0; k < hw::NUM_HOMED_AXES; k++)
    if (only < 0 || hw::HOME_ORDER[k] == only) queue[qLen++] = hw::HOME_ORDER[k];
  if (!qLen) { Out.println(F("ERR axis has no homing (use 2, 4 or 6)")); return false; }

  if (!sys::request(sys::State::Homing)) return false;   // prints the reason
  sys::setHomed(false);

  if (!quiet) {
    Out.print(F("OK home"));
    for (uint8_t k = 0; k < qLen; k++) { Out.print(k ? ',' : ' '); Out.print(hw::AXES[queue[k]].name); }
    Out.println();
  }

  startMs = millis();
  stopsAtStart = motion::stopCount();
  for (uint8_t k = 0; k < qLen; k++) {
    uint8_t a = queue[k];
    homedAxis[a] = false;
    found[a] = limit_switches::pressed(limit_switches::findHome(a));   // already at the switch
  }
  axis = queue[0];
  expectAll(true);
  startGroup();
  return true;
}

// ------------------------------------------------------------
void update() {
  if (phase == Phase::Idle) return;

  // Stopped from outside (stop command, encoder, fault)?
  if (motion::stopCount() != stopsAtStart) {
    Out.print(F("EV home aborted ")); name(); Out.println();
    reset();
    if (sys::get() == sys::State::Homing) sys::request(sys::State::Idle);
    return;
  }

  if (phase == Phase::Group) { updateGroup(); return; }

  const bool pressed = limit_switches::pressed(sw);
  const bool moving  = motion::busy(axis);

  switch (phase) {
    case Phase::Clear:                     // forward until the switch releases
      if (!pressed) {
        motion::stopGroupOf(axis);
        if (stage == 2) {
          Out.print(F("EV home ")); name();
          Out.print(F(" switch released at ")); Out.print(motion::position(axis), 2); Out.println(F(" mm"));
        }
        if (!move(stage == 1 ? motion_cfg::HOME_REAPPROACH_MM : motion_cfg::HOME_BACKOFF_MM)) { fail("backoff refused"); return; }
        phase = Phase::Extra;
      } else if (!moving) {
        fail("switch stays pressed");
      }
      break;

    case Phase::Extra:                     // backoff finished
      if (!moving) {
        if (stage == 2) axisDone();
        else            startSeek();       // stage 1: slow approach (stage 0 never gets here)
      }
      break;

    case Phase::Seek:                      // backward until the switch presses
      if (pressed && stage == 0) {         // own fast approach hit: back off, then come in slowly
        motion::stopGroupOf(axis);
        stage = 1;
        startClear();
      } else if (pressed) {                // slow approach hit: this is the zero
        motion::stopGroupOf(axis);
        float was = motion::position(axis);
        steppers::setPosition(axis, 0);
        stage = 2;
        Out.print(F("EV home ")); name();
        Out.print(F(" zero set (was at ")); Out.print(was, 2); Out.println(F(" mm)"));
        startClear();
      } else if (!moving) {
        fail(stage == 0 ? "switch not found" : "switch not found again (slow approach)");
      }
      break;

    default: break;
  }
}
}  // namespace homing
