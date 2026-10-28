#include "drivers.h"
#include "serial_io.h"
#include <TMCStepper.h>
#include "hw_profile.h"
#include "motion_config.h"

namespace drivers {
namespace {
  static_assert(hw::NUM_AXES == 6, "driver table below assumes 6 axes");

  HardwareSerial* bus(uint8_t n) {
    return n == 1 ? &Serial1 : n == 2 ? &Serial2 : &Serial3;
  }

  // One driver object per axis, wired to the bus + address from the profile
  TMC2209Stepper tmc[hw::NUM_AXES] = {
    TMC2209Stepper(bus(hw::AXES[0].uartBus), hw::R_SENSE, hw::AXES[0].uartAddr),
    TMC2209Stepper(bus(hw::AXES[1].uartBus), hw::R_SENSE, hw::AXES[1].uartAddr),
    TMC2209Stepper(bus(hw::AXES[2].uartBus), hw::R_SENSE, hw::AXES[2].uartAddr),
    TMC2209Stepper(bus(hw::AXES[3].uartBus), hw::R_SENSE, hw::AXES[3].uartAddr),
    TMC2209Stepper(bus(hw::AXES[4].uartBus), hw::R_SENSE, hw::AXES[4].uartAddr),
    TMC2209Stepper(bus(hw::AXES[5].uartBus), hw::R_SENSE, hw::AXES[5].uartAddr),
  };

  bool     isPresent[hw::NUM_AXES];
  bool     isEnabled[hw::NUM_AXES];
  uint16_t msCur[hw::NUM_AXES];
  Chopper  chopCur[hw::NUM_AXES];
  float    hybridCur[hw::NUM_AXES];

  uint16_t msFor(uint8_t i) {
    return hw::AXES[i].type == AxisType::Translation ? hw::MICROSTEPS_TRA : hw::MICROSTEPS_ROT;
  }

  const __FlashStringHelper* chopName(Chopper c) {
    return c == Chopper::Stealth ? F("stealth") : c == Chopper::Spread ? F("spread") : F("hybrid");
  }
}

void begin() {
  Serial1.begin(hw::TMC_BAUD);
  Serial2.begin(hw::TMC_BAUD);
  Serial3.begin(hw::TMC_BAUD);

  for (uint8_t i = 0; i < hw::NUM_AXES; i++) {
    const AxisConfig& a = hw::AXES[i];
    msCur[i] = msFor(i); chopCur[i] = Chopper::Stealth; hybridCur[i] = 0;
    pinMode(a.pinStep, OUTPUT); digitalWrite(a.pinStep, LOW);
    pinMode(a.pinDir,  OUTPUT); digitalWrite(a.pinDir,  LOW);
    pinMode(a.pinEn,   OUTPUT); digitalWrite(a.pinEn,   HIGH);   // disabled until configured
    pinMode(a.pinDiag, INPUT);

#ifdef SIM_FAKE_DRIVERS
    // Simulator build only: no drivers attached, pretend they answered
    isPresent[i] = true; enable(i, true); continue;
#endif
    tmc[i].begin();
    isPresent[i] = (tmc[i].test_connection() == 0);
    if (isPresent[i]) {
      tmc[i].rms_current(a.currentMa);   // run current; hold = 50 % (library default)
      tmc[i].microsteps(msFor(i));
      tmc[i].intpol(true);               // interpolate to 256 microsteps
      tmc[i].en_spreadCycle(false);      // StealthChop: quiet
      tmc[i].pwm_autoscale(true);
      enable(i, true);                   // hold position (as V1 did)
    }
  }
  // Startup chopper mode per axis type (see motion_config.h, decision D4)
  for (uint8_t i = 0; i < hw::NUM_AXES; i++) {
    bool tra = hw::AXES[i].type == AxisType::Translation;
    uint8_t mode = tra ? motion_cfg::TRA_CHOPPER : motion_cfg::ROT_CHOPPER;
    float hybridSps = tra ? motion_cfg::TRA_HYBRID_MMPS * hw::FULLSTEPS_PER_REV * msCur[i] / hw::LEAD_MM : 0;
    setChopper(i, mode == 1 ? Chopper::Spread : mode == 2 ? Chopper::Hybrid : Chopper::Stealth, hybridSps);
  }
}

bool     present(uint8_t i)    { return i < hw::NUM_AXES && isPresent[i]; }
uint16_t microsteps(uint8_t i) { return msCur[i]; }

void setMicrosteps(uint8_t i, uint16_t ms) {
  if (i >= hw::NUM_AXES) return;
  msCur[i] = ms;
#ifndef SIM_FAKE_DRIVERS
  if (isPresent[i]) tmc[i].microsteps(ms);
#endif
}

void setChopper(uint8_t i, Chopper mode, float hybridSps) {
  if (i >= hw::NUM_AXES) return;
  chopCur[i] = mode; hybridCur[i] = hybridSps;
#ifndef SIM_FAKE_DRIVERS
  if (!isPresent[i]) return;
  if (mode == Chopper::Spread) {
    tmc[i].en_spreadCycle(true);
    tmc[i].TPWMTHRS(0);
  } else {
    tmc[i].en_spreadCycle(false);
    uint32_t thr = 0;   // 0 = StealthChop at all speeds
    if (mode == Chopper::Hybrid && hybridSps > 0) {
      // TSTEP is measured in 12 MHz clocks per 1/256 microstep.
      // StealthChop is used while TSTEP >= TPWMTHRS (i.e. below the speed).
      float f256 = hybridSps * 256.0f / msCur[i];
      thr = (uint32_t)(12000000.0f / f256);
      if (thr > 0xFFFFF) thr = 0xFFFFF;
    }
    tmc[i].TPWMTHRS(thr);
  }
#endif
}

void setCurrent(uint8_t i, uint16_t mA) {
  if (i >= hw::NUM_AXES) return;
#ifndef SIM_FAKE_DRIVERS
  if (isPresent[i]) tmc[i].rms_current(mA);
#endif
}
bool     enabled(uint8_t i)    { return i < hw::NUM_AXES && isEnabled[i]; }

void enable(uint8_t i, bool on) {
  if (i >= hw::NUM_AXES) return;
  digitalWrite(hw::AXES[i].pinEn, on ? LOW : HIGH);
  isEnabled[i] = on && isPresent[i];
}

#ifdef SIM_FAKE_DRIVERS
bool alive(uint8_t)           { return true; }
bool overTemp(uint8_t)        { return false; }
bool overTempWarning(uint8_t) { return false; }
#else
bool alive(uint8_t i)           { return isPresent[i] && tmc[i].test_connection() == 0; }
bool overTemp(uint8_t i)        { return isPresent[i] && tmc[i].ot(); }
bool overTempWarning(uint8_t i) { return isPresent[i] && tmc[i].otpw(); }
#endif

void report() {
#ifdef SIM_FAKE_DRIVERS
  Out.println(F("OK drv simulated")); return;
#endif
  for (uint8_t i = 0; i < hw::NUM_AXES; i++) {
    Out.print(F("OK drv ")); Out.print(i + 1);
    Out.print(' ');         Out.print(hw::AXES[i].name);
    if (!isPresent[i]) { Out.println(F(" not found")); continue; }
    Out.print(F(" I="));    Out.print(tmc[i].rms_current());
    Out.print(F("mA ms=")); Out.print(tmc[i].microsteps());
    Out.print(F(" en="));   Out.print(isEnabled[i] ? 1 : 0);
    Out.print(F(" chop=")); Out.print(chopName(chopCur[i]));
    if (chopCur[i] == Chopper::Hybrid) { Out.print('@'); Out.print(hybridCur[i], 0); Out.print(F("sps")); }
    if (tmc[i].ot())        Out.print(F(" OVERTEMP"));
    else if (tmc[i].otpw()) Out.print(F(" warm"));
    Out.println();
  }
}
}  // namespace drivers
