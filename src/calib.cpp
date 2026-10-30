#include "calib.h"
#include "serial_io.h"
#include "hw_profile.h"
#include "motion_config.h"
#include "limit_switches.h"
#include "motion.h"
#include "safety.h"
#include "system_state.h"
#include "homing.h"
#include "limits.h"
#include "settings.h"

namespace calib {
namespace {
  constexpr uint8_t C = hw::NUM_HOMED_AXES;

  // One operation of the sequence
  enum Kind : uint8_t {
    SEEK,     // carriages in 'mask' forward together (same speed) up to 'dist' mm
              // until switch 'sw' presses; 'axis' is the one measured (record if rec >= 0)
    CLEAR,    // 'axis' backward until switch 'sw' releases, then 'dist' mm further
    MOVETO    // carriages in 'mask' together to absolute position 'dist' (mm)
  };
  struct Op { Kind kind; uint8_t axis; uint8_t mask; int8_t sw; float speed; float dist; int8_t rec; };
  // rec: -1 = nothing, 0 = front max, k>0 = gap between carriage k-1 and k

  Op       ops[8 * C];
  uint8_t  nOps = 0, cur = 0;
  bool     running = false;
  bool     backingOff = false;    // CLEAR: switch released, doing the extra mm
  float    frontMax = 0;
  float    gap[C - 1];
  uint32_t stopsAtStart = 0, startMs = 0;

  void add(Kind k, uint8_t axis, uint8_t mask, int8_t sw, float speed, float dist, int8_t rec) {
    ops[nOps++] = { k, axis, mask, sw, speed, dist, rec };
  }
  // Two-speed approach onto a switch: the carriages in 'mask' move together
  // fast until the switch clicks; then only 'axis' backs off 2 mm and comes in
  // slowly (records). The short refine move barely changes any gap.
  void addApproach(uint8_t axis, uint8_t mask, int8_t sw, float fast, float slow, int8_t rec) {
    add(SEEK,  axis, mask,      sw, fast, motion_cfg::HOME_SEARCH_MM, -1);
    add(CLEAR, axis, _BV(axis), sw, 0, motion_cfg::HOME_REAPPROACH_MM, -1);
    add(SEEK,  axis, _BV(axis), sw, slow, motion_cfg::HOME_REAPPROACH_MM + motion_cfg::HOME_CLEAR_MM, rec);
    add(CLEAR, axis, _BV(axis), sw, 0, motion_cfg::CALIB_SEPARATE_MM, -1);
  }

  // Coordinated internal move of several carriages (no OK/EV lines)
  bool groupMove(const float amount[], uint8_t mask, float speed) {
    safety::setHomingMove(true);
    bool ok = motion::moveGroup(amount, mask, 100, 0, speed, true);
    safety::setHomingMove(false);
    return ok;
  }

  bool move(uint8_t axis, float mm, float speed) {
    safety::setHomingMove(true);
    bool ok = motion::moveRelative(axis, mm, speed, true);
    safety::setHomingMove(false);
    return ok;
  }

  void endRun() {
    for (uint8_t s = 0; s < hw::NUM_SWITCHES; s++) safety::expectSwitch(s, false);
    running = false;
  }

  void fail(const char* why) {
    char buf[40];
    snprintf(buf, sizeof(buf), "calib %s: %s", hw::AXES[ops[cur].axis].name, why);
    Out.print(F("EV calib failed ")); Out.println(buf);
    endRun();
    sys::fault(buf);
  }

  // Compare one measured value with the hardware profile value (0 = unknown)
  bool plausible(const __FlashStringHelper* what, float measuredV, float expected) {
    if (expected <= 0) return true;
    float dev = measuredV - expected;
    if (fabsf(dev) <= motion_cfg::CALIB_MAX_DEVIATION_MM) return true;
    Out.print(F("EV calib rejected ")); Out.print(what); Out.print('=');
    Out.print(measuredV, 2); Out.print(F(" expected ")); Out.print(expected, 2);
    Out.print(F(" (")); Out.print(dev, 2); Out.println(F(" mm, lost steps?)"));
    return false;
  }

  void finish() {
    endRun();
    Out.print(F("EV calib measured t=")); Out.print((millis() - startMs) / 1000.0f, 1);
    Out.print(F("s front_max=")); Out.print(frontMax, 2);
    for (uint8_t k = 0; k + 1 < C; k++) { Out.print(F(" gap")); Out.print(k + 1); Out.print('='); Out.print(gap[k], 2); }
    Out.println();

    bool ok = plausible(F("front_max"), frontMax, hw::CALIB_DEFAULT_FRONT_MAX_MM);
    for (uint8_t k = 0; k + 1 < C; k++) ok &= plausible(k ? F("gap2") : F("gap1"), gap[k], hw::CALIB_DEFAULT_GAP_MM[k]);
    if (ok) {
      limits::set(frontMax, gap);
      settings::saveCalib(frontMax, gap);
      Out.println(F("EV calib stored (also in EEPROM)"));
      // Furthest each carriage can physically go (all carriages in front at their maximum),
      // before the margins of the limits
      Out.print(F("EV calib reach"));
      float m = frontMax;
      for (uint8_t k = 0; k < C; k++) {
        if (k) m += gap[k - 1];
        Out.print(' '); Out.print(hw::AXES[hw::CARRIAGE_ORDER[k]].name); Out.print('='); Out.print(m, 2);
      }
      Out.println();
    } else {
      Out.println(F("EV calib not stored (previous values kept)"));
    }

    // Steps may have been lost during the long moves: home again so positions are right
    sys::request(sys::State::Ready);
    Out.println(F("EV calib re-homing"));
    homing::start(-1, true);
  }

  void startOp() {
    if (cur >= nOps) { finish(); return; }
    Op& o = ops[cur];
    backingOff = false;
    switch (o.kind) {
      case SEEK: {
        if (limit_switches::pressed(o.sw)) { fail("switch pressed before seek"); return; }
        safety::expectSwitch(o.sw, true);
        float amount[hw::NUM_AXES] = { 0 };
        for (uint8_t i = 0; i < hw::NUM_AXES; i++) if (o.mask & _BV(i)) amount[i] = o.dist;  // equal = same speed
        if (!groupMove(amount, o.mask, o.speed)) fail("seek refused");
        break;
      }
      case CLEAR:
        safety::expectSwitch(o.sw, true);
        if (!move(o.axis, -motion_cfg::HOME_CLEAR_MM, motion_cfg::HOME_SPEED)) fail("back-off refused");
        break;
      case MOVETO: {
        float amount[hw::NUM_AXES] = { 0 };
        uint8_t m = 0;
        for (uint8_t i = 0; i < hw::NUM_AXES; i++) {
          if (!(o.mask & _BV(i))) continue;
          amount[i] = o.dist - motion::position(i);
          if (fabsf(amount[i]) >= 0.01f) m |= _BV(i);
        }
        if (!m) { cur++; startOp(); return; }
        // one coordinated move: all arrive together, every gap changes linearly
        if (!groupMove(amount, m, motion_cfg::CALIB_MOVE_SPEED)) fail("move refused");
        break;
      }
    }
  }

  void next() { cur++; startOp(); }
}

// ------------------------------------------------------------
void begin() { running = false; }
bool active() { return running; }

bool start(bool quiet) {
  if (running || homing::active()) { Out.println(F("ERR homing or calib already running")); return false; }
  if (motion::anyBusy())            { Out.println(F("ERR stop motion first")); return false; }
  for (uint8_t k = 0; k < C; k++)
    if (!homing::axisHomed(hw::CARRIAGE_ORDER[k])) { Out.println(F("ERR home first")); return false; }

  // Build the sequence from the hardware profile (front to back)
  nOps = 0;
  const uint8_t front = hw::CARRIAGE_ORDER[0];
  const int8_t  farSw = limit_switches::findFarEnd(front);
  if (farSw < 0) { Out.println(F("ERR front carriage has no far-end switch")); return false; }
  // Front carriage to its far switch, and it stays there (just off the switch).
  // Each next carriage drives up to the one in front and stays there too:
  // every carriage is measured over its real range of motion.
  // A carriage may never get further ahead of the one behind it than at home
  // (the tube between them would hang unsupported and buckle). So carriage k
  // always moves together with every carriage behind it:
  //   all three forward together until T1 reaches its far switch,
  //   T2 + T3 together until T2 touches T1, then T3 alone onto T2
  //   (moving the back carriage alone only shortens the free span).
  uint8_t all = 0;
  for (uint8_t k = 0; k < C; k++) all |= _BV(hw::CARRIAGE_ORDER[k]);
  uint8_t mask = all;
  addApproach(front, mask, farSw, motion_cfg::HOME_FAST_SPEED, motion_cfg::HOME_SPEED, 0);
  for (uint8_t k = 1; k < C; k++) {
    mask &= ~_BV(hw::CARRIAGE_ORDER[k - 1]);     // the one in front stays where it is
    // carriage k drives onto carriage k-1: switch on the back of k-1 (its home switch)
    const uint8_t a  = hw::CARRIAGE_ORDER[k];
    const int8_t  sw = limit_switches::findHome(hw::CARRIAGE_ORDER[k - 1]);
    if (sw < 0) { Out.println(F("ERR carriage has no switch on its back")); return false; }
    // From the third carriage on: first bring everything back together to the
    // park position, so this carriage touches the next one well inside its range
    // (the gap is the same wherever it is measured).
    if (k >= 2) add(MOVETO, front, all, -1, 0, motion_cfg::CALIB_PARK_MM, -1);
    addApproach(a, mask, sw, motion_cfg::CALIB_CONTACT_FAST, motion_cfg::CALIB_CONTACT_SPEED, k);
  }
  add(MOVETO, front, all, -1, 0, motion_cfg::CALIB_REST_MM, -1);   // all back together

  if (!sys::request(sys::State::Homing)) return false;
  if (!quiet) Out.println(F("OK calib"));
  running = true;
  cur = 0;
  startMs = millis();
  stopsAtStart = motion::stopCount();
  startOp();
  return true;
}

void update() {
  if (!running) return;
  if (motion::stopCount() != stopsAtStart) {
    Out.println(F("EV calib aborted (previous values kept)"));
    endRun();
    if (sys::get() == sys::State::Homing) sys::request(sys::State::Ready);
    return;
  }

  Op& o = ops[cur];
  const bool moving = motion::busy(o.axis);
  switch (o.kind) {
    case SEEK:
      if (limit_switches::pressed(o.sw)) {
        motion::stopGroupOf(o.axis);
        float p = motion::position(o.axis);
        if (o.rec > 0) {                  // check before printing anything
          float g = p - motion::position(hw::CARRIAGE_ORDER[o.rec - 1]);
          if (g < motion_cfg::LIMIT_MARGIN_MM) { fail("gap implausible (<margin)"); return; }
        }
        Out.print(F("EV calib ")); Out.print(hw::AXES[o.axis].name);
        if (o.rec < 0) {
          Out.print(F(" switch at ")); Out.print(p, 2); Out.println(F(" mm (fast), approaching slowly"));
        } else if (o.rec == 0) {
          frontMax = p;
          Out.print(F(" far switch at ")); Out.print(p, 2); Out.println(F(" mm"));
        } else {
          uint8_t frontAxis = hw::CARRIAGE_ORDER[o.rec - 1];
          float pf = motion::position(frontAxis);
          gap[o.rec - 1] = p - pf;
          Out.print(F(" touches ")); Out.print(hw::AXES[frontAxis].name);
          Out.print(F(" at ")); Out.print(p, 2); Out.print(F(" mm (")); Out.print(hw::AXES[frontAxis].name);
          Out.print(F(" at ")); Out.print(pf, 2); Out.print(F(") gap=")); Out.println(gap[o.rec - 1], 2);
        }
        next();
      } else if (!moving) {
        fail("switch not found");
      }
      break;

    case CLEAR:
      if (!backingOff) {
        if (!limit_switches::pressed(o.sw)) {
          motion::stopGroupOf(o.axis);
          if (!move(o.axis, -o.dist, motion_cfg::HOME_SPEED)) { fail("back-off refused"); return; }
          backingOff = true;
        } else if (!moving) {
          fail("switch stays pressed");
        }
      } else if (!moving) {
        safety::expectSwitch(o.sw, false);
        next();
      }
      break;

    case MOVETO:
      if (!moving) next();
      break;
  }
}
}  // namespace calib
