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

  // Phases of one axis:
  //   Clear  : switch pressed -> move forward until it releases
  //   Extra  : then HOME_BACKOFF_MM further
  //   Seek   : move backward until the switch presses -> zero
  //   (after zero: Clear + Extra again, then the axis is done)
  enum class Phase : uint8_t { Idle, Clear, Extra, Seek };
  Phase    phase = Phase::Idle;
  uint8_t  queue[N];
  uint8_t  qLen = 0, qPos = 0;
  uint8_t  axis = 0;
  int8_t   sw   = -1;
  bool     zeroed = false;           // current axis already has its zero
  uint32_t stopsAtStart = 0;         // motion::stopCount() when homing started
  uint32_t startMs = 0;
  bool     homedAxis[N];

  void name() { Out.print(hw::AXES[axis].name); }

  bool allHomed() {
    for (uint8_t k = 0; k < hw::NUM_HOMED_AXES; k++)
      if (!homedAxis[hw::HOME_ORDER[k]]) return false;
    return true;
  }

  // Internal move: allowed past the "not homed" limit, prints nothing on success
  bool move(float mm) {
    safety::setHomingMove(true);
    bool ok = motion::moveRelative(axis, mm, motion_cfg::HOME_SPEED, true);
    safety::setHomingMove(false);
    return ok;
  }

  void reset() {
    if (sw >= 0) safety::expectSwitch(sw, false);
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
    if (!move(-motion_cfg::HOME_SEARCH_MM)) { fail("seek refused"); return; }
    phase = Phase::Seek;
  }

  void startClear() {
    if (!move(motion_cfg::HOME_CLEAR_MM)) { fail("clear refused"); return; }
    phase = Phase::Clear;
  }

  void startAxis() {
    axis   = queue[qPos];
    sw     = limit_switches::findHome(axis);
    zeroed = false;
    homedAxis[axis] = false;
    if (sw < 0) { fail("no home switch"); return; }
    safety::expectSwitch(sw, true);
    Out.print(F("EV home ")); name(); Out.println(F(" start"));
    if (limit_switches::pressed(sw)) startClear();   // e.g. T3 resting on its block
    else                             startSeek();
  }

  void axisDone() {
    safety::expectSwitch(sw, false);
    sw = -1;
    homedAxis[axis] = true;
    Out.print(F("EV home ")); name(); Out.print(F(" ok pos=")); Out.println(motion::position(axis), 2);
    if (++qPos < qLen) { startAxis(); return; }

    phase = Phase::Idle;
    Out.print(F("EV home done t=")); Out.print((millis() - startMs) / 1000.0f, 1); Out.println('s');
    sys::setHomed(allHomed());
    sys::request(allHomed() ? sys::State::Ready : sys::State::Idle);
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

bool start(int8_t only) {
  if (active())          { Out.println(F("ERR homing already running")); return false; }
  if (motion::anyBusy()) { Out.println(F("ERR stop motion first")); return false; }

  qLen = 0; qPos = 0;
  for (uint8_t k = 0; k < hw::NUM_HOMED_AXES; k++)
    if (only < 0 || hw::HOME_ORDER[k] == only) queue[qLen++] = hw::HOME_ORDER[k];
  if (!qLen) { Out.println(F("ERR axis has no homing (use 2, 4 or 6)")); return false; }

  if (!sys::request(sys::State::Homing)) return false;   // prints the reason
  sys::setHomed(false);

  Out.print(F("OK home"));
  for (uint8_t k = 0; k < qLen; k++) { Out.print(k ? ',' : ' '); Out.print(hw::AXES[queue[k]].name); }
  Out.println();

  startMs = millis();
  stopsAtStart = motion::stopCount();
  startAxis();
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

  const bool pressed = limit_switches::pressed(sw);
  const bool moving  = motion::busy(axis);

  switch (phase) {
    case Phase::Clear:                     // forward until the switch releases
      if (!pressed) {
        motion::stopGroupOf(axis);
        if (zeroed) {
          Out.print(F("EV home ")); name();
          Out.print(F(" switch released at ")); Out.print(motion::position(axis), 2); Out.println(F(" mm"));
        }
        if (!move(motion_cfg::HOME_BACKOFF_MM)) { fail("backoff refused"); return; }
        phase = Phase::Extra;
      } else if (!moving) {
        fail("switch stays pressed");
      }
      break;

    case Phase::Extra:                     // backoff finished
      if (!moving) {
        if (zeroed) axisDone();
        else        startSeek();
      }
      break;

    case Phase::Seek:                      // backward until the switch presses
      if (pressed) {
        motion::stopGroupOf(axis);
        float found = motion::position(axis);
        steppers::setPosition(axis, 0);
        zeroed = true;
        Out.print(F("EV home ")); name();
        Out.print(F(" zero set (was at ")); Out.print(found, 2); Out.println(F(" mm)"));
        startClear();
      } else if (!moving) {
        fail("switch not found");
      }
      break;

    default: break;
  }
}
}  // namespace homing
