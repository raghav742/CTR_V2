#include "limit_switches.h"
#include "hw_profile.h"

namespace limit_switches {
namespace {
  bool     stable[hw::NUM_SWITCHES];
  bool     rawLast[hw::NUM_SWITCHES];
  uint32_t rawSince[hw::NUM_SWITCHES];
  bool     evChanged[hw::NUM_SWITCHES];
  uint32_t edgeUs[hw::NUM_SWITCHES];

  bool readRaw(uint8_t i) {
    bool level = digitalRead(hw::SWITCHES[i].pin) == HIGH;
    return hw::SWITCHES[i].activeHigh ? level : !level;
  }

  int8_t find(uint8_t axis, SwitchRole role) {
    for (uint8_t i = 0; i < hw::NUM_SWITCHES; i++)
      if (hw::SWITCHES[i].axis == axis && hw::SWITCHES[i].role == role) return i;
    return -1;
  }
}

void begin() {
  for (uint8_t i = 0; i < hw::NUM_SWITCHES; i++) {
    pinMode(hw::SWITCHES[i].pin, INPUT);   // driven by the Schmitt trigger: no pull-up
    stable[i] = rawLast[i] = readRaw(i);
    rawSince[i] = millis();
    evChanged[i] = false;
  }
}

void update() {
  const uint32_t now = millis();
  for (uint8_t i = 0; i < hw::NUM_SWITCHES; i++) {
    bool raw = readRaw(i);
    if (raw != rawLast[i]) { rawLast[i] = raw; rawSince[i] = now; edgeUs[i] = micros(); }
    if (raw != stable[i] && (now - rawSince[i]) >= DEBOUNCE_MS) {
      stable[i] = raw;
      evChanged[i] = true;
    }
  }
}

bool   pressed(uint8_t i)       { return i < hw::NUM_SWITCHES && stable[i]; }
uint32_t rawEdgeUs(uint8_t i)   { return i < hw::NUM_SWITCHES ? edgeUs[i] : 0; }
bool   changed(uint8_t i)       { if (i >= hw::NUM_SWITCHES) return false;
                                  bool e = evChanged[i]; evChanged[i] = false; return e; }
int8_t findHome(uint8_t axis)   { return find(axis, SwitchRole::Home); }
int8_t findFarEnd(uint8_t axis) { return find(axis, SwitchRole::FarEnd); }
}  // namespace limit_switches
