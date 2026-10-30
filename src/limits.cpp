#include "limits.h"
#include "serial_io.h"
#include "hw_profile.h"
#include "motion_config.h"
#include "motion.h"
#include "homing.h"
#include "settings.h"
#include "safety.h"

namespace limits {
namespace {
  constexpr uint8_t C = hw::NUM_HOMED_AXES;      // number of carriages
  bool  valid = false;
  uint8_t source = 0;                // 0 = profile, 1 = calib (this session), 2 = EEPROM
  float fMax = 0;
  float gaps[C - 1];

  uint8_t ax(uint8_t k) { return hw::CARRIAGE_ORDER[k]; }
  const char* nm(uint8_t k) { return hw::AXES[ax(k)].name; }
  bool homedPair(uint8_t k) { return homing::axisHomed(ax(k)) && homing::axisHomed(ax(k + 1)); }

  // Far end (front carriage) or contact with the carriage in front, minus margins
  float contactLimit(uint8_t k, const float p[]) {
    float m = motion_cfg::LIMIT_MARGIN_MM + hw::CARRIAGE_EXTRA_MARGIN_MM[k];
    return (k == 0) ? fMax - m : p[k - 1] + gaps[k - 1] - m;
  }

  // Upper / lower limit of carriage k for given positions of all carriages
  float upper(uint8_t k, const float p[]) {
    float u = 1e9f;
    if (active()) u = contactLimit(k, p);
    if ((motion_cfg::SPAN_RULE_FOR_MOVES || safety::spanRuleForced()) && k + 1 < C && homedPair(k))
      u = fminf(u, p[k + 1] + motion_cfg::LIMIT_SPAN_TOL_MM);
    return u;
  }
  float lower(uint8_t k, const float p[]) {
    float l = homing::axisHomed(ax(k)) ? motion_cfg::TRA_SOFT_MIN_MM : -1e9f;
    if ((motion_cfg::SPAN_RULE_FOR_MOVES || safety::spanRuleForced()) && k > 0 && homedPair(k - 1))
      l = fmaxf(l, p[k - 1] - motion_cfg::LIMIT_SPAN_TOL_MM);
    return l;
  }
}

void begin() {
  // Known values from the hardware profile (0 = unknown -> no limits until calib)
  valid = hw::CALIB_DEFAULT_FRONT_MAX_MM > 0;
  fMax = hw::CALIB_DEFAULT_FRONT_MAX_MM;
  for (uint8_t k = 0; k + 1 < C; k++) {
    gaps[k] = hw::CALIB_DEFAULT_GAP_MM[k];
    if (gaps[k] <= 0) valid = false;
  }
  source = 0;

  // A calib result stored in EEPROM replaces them, if it is plausible
  if (settings::calibStored()) {
    float fm, g[C - 1];
    settings::calibValues(fm, g);
    bool ok = fabsf(fm - hw::CALIB_DEFAULT_FRONT_MAX_MM) <= motion_cfg::CALIB_MAX_DEVIATION_MM ||
              hw::CALIB_DEFAULT_FRONT_MAX_MM <= 0;
    for (uint8_t k = 0; k + 1 < C; k++)
      ok &= fabsf(g[k] - hw::CALIB_DEFAULT_GAP_MM[k]) <= motion_cfg::CALIB_MAX_DEVIATION_MM ||
            hw::CALIB_DEFAULT_GAP_MM[k] <= 0;
    if (ok) {
      fMax = fm;
      for (uint8_t k = 0; k + 1 < C; k++) gaps[k] = g[k];
      valid = true;
      source = 2;
    } else {
      Out.println(F("EV limits stored calib values implausible: using profile values"));
    }
  }
}

void set(float frontMax, const float gap[]) {
  fMax = frontMax;
  for (uint8_t k = 0; k + 1 < C; k++) gaps[k] = gap[k];
  valid = true;
  source = 1;
}

bool calibrated() { return valid; }

bool active() {
  if (!valid) return false;
  for (uint8_t k = 0; k < C; k++) if (!homing::axisHomed(ax(k))) return false;
  return true;
}

bool checkMove(const float amount[], uint8_t mask) {
  // End position of every carriage; refuse if a neighbour of a moving carriage
  // is busy in another move (its end position is not known here).
  float endPos[C];
  for (uint8_t k = 0; k < C; k++) {
    bool inMove = mask & _BV(ax(k));
    endPos[k] = motion::position(ax(k)) + (inMove ? amount[ax(k)] : 0);
    if (!inMove || !homing::axisHomed(ax(k))) continue;
    for (int8_t n = k - 1; n <= k + 1; n += 2) {
      if (n < 0 || n >= C) continue;
      if (!(mask & _BV(ax(n))) && motion::busy(ax(n))) {
        Out.print(F("ERR ")); Out.print(nm(n));
        Out.println(F(" is moving in another move; combine them in one mv"));
        return false;
      }
    }
  }

  for (uint8_t k = 0; k < C; k++) {
    bool involved = (mask & _BV(ax(k))) ||
                    (k > 0 && (mask & _BV(ax(k - 1)))) || (k + 1 < C && (mask & _BV(ax(k + 1))));
    if (!involved) continue;

    // span: not further ahead of the carriage behind than at home
    if ((motion_cfg::SPAN_RULE_FOR_MOVES || safety::spanRuleForced()) && k + 1 < C && homedPair(k) &&
        endPos[k] > endPos[k + 1] + motion_cfg::LIMIT_SPAN_TOL_MM + 0.005f) {
      Out.print(F("ERR ")); Out.print(nm(k)); Out.print(F(" would be "));
      Out.print(endPos[k] - endPos[k + 1], 2); Out.print(F(" mm ahead of ")); Out.print(nm(k + 1));
      Out.print(F(" (tube unsupported, max ")); Out.print(motion_cfg::LIMIT_SPAN_TOL_MM, 1);
      Out.println(F("): move the carriages behind it along in the same mv"));
      return false;
    }
    // contact / far end (needs calibration)
    if (active()) {
      float lim = contactLimit(k, endPos);
      if (endPos[k] > lim + 0.005f) {
        Out.print(F("ERR ")); Out.print(nm(k)); Out.print(F(" would end at "));
        Out.print(endPos[k], 2); Out.print(F(" mm, limit ")); Out.print(lim, 2);
        if (k == 0) Out.println(F(" mm (far end)"));
        else { Out.print(F(" mm (")); Out.print(nm(k - 1)); Out.println(')'); }
        return false;
      }
    }
  }
  return true;
}

bool geometry(float& frontMaxUsable, float extMax[]) {
  if (!active()) return false;
  frontMaxUsable = fMax - motion_cfg::LIMIT_MARGIN_MM - hw::CARRIAGE_EXTRA_MARGIN_MM[0];
  for (uint8_t k = 1; k < C; k++)
    extMax[k - 1] = gaps[k - 1] - motion_cfg::LIMIT_MARGIN_MM - hw::CARRIAGE_EXTRA_MARGIN_MM[k];
  return true;
}

bool range(uint8_t axis, float& lo, float& hi) {
  int8_t k = -1;
  for (uint8_t j = 0; j < C; j++) if (ax(j) == axis) k = j;
  if (k < 0) return false;
  float p[C];
  for (uint8_t j = 0; j < C; j++) p[j] = motion::position(ax(j));
  lo = lower(k, p); hi = upper(k, p);
  if (lo < -1e8f) lo = NAN;
  if (hi > 1e8f)  hi = NAN;
  return true;
}

void report() {
  Out.print(F("OK limits calibrated=")); Out.print(valid ? 1 : 0);
  if (valid) {
    Out.print(F(" source="));
    Out.print(source == 1 ? F("calib") : source == 2 ? F("eeprom") : F("profile"));
  }
  Out.print(F(" active=")); Out.print(active() ? 1 : 0);
  if (valid) {
    Out.print(F(" front_max=")); Out.print(fMax, 2);
    for (uint8_t k = 0; k + 1 < C; k++) {
      Out.print(F(" gap_")); Out.print(nm(k)); Out.print('/'); Out.print(nm(k + 1));
      Out.print('=');       Out.print(gaps[k], 2);
    }
    Out.print(F(" margin=")); Out.print(motion_cfg::LIMIT_MARGIN_MM, 1);
    Out.print(F(" extra="));
    for (uint8_t k = 0; k < C; k++) { if (k) Out.print(','); Out.print(hw::CARRIAGE_EXTRA_MARGIN_MM[k], 0); }
  }
  Out.print(F(" span_rule=")); Out.print(motion_cfg::SPAN_RULE_FOR_MOVES ? F("on") : F("homing/calib only"));
  // Range each carriage could move to on its own right now
  float p[C];
  for (uint8_t k = 0; k < C; k++) p[k] = motion::position(ax(k));
  Out.print(F(" range_now="));
  for (uint8_t k = 0; k < C; k++) {
    if (k) Out.print(',');
    Out.print(nm(k)); Out.print(':');
    float lo = lower(k, p), hi = upper(k, p);
    if (lo < -1e8f) Out.print('?'); else Out.print(lo, 2);
    Out.print(F(".."));
    if (hi > 1e8f)  Out.print('?'); else Out.print(hi, 2);
  }
  Out.println();
}
}  // namespace limits
