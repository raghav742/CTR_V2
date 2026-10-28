#pragma once
// ============================================================
//  Limit switches (debounced, polarity from hardware profile)
//  Hardware already filters (RC + Schmitt trigger); software adds
//  a short debounce so a single noise spike never counts.
// ============================================================
#include <Arduino.h>

namespace limit_switches {
void begin();
void update();                    // call every loop pass

bool    pressed(uint8_t idx);     // debounced state of SWITCHES[idx]
bool    changed(uint8_t idx);     // true once after a state change (event)
uint32_t rawEdgeUs(uint8_t idx);  // micros() of the last raw level change (reaction time)
int8_t  findHome(uint8_t axis);   // switch index of an axis' home switch, -1 = none
int8_t  findFarEnd(uint8_t axis); // switch index of an axis' far-end switch, -1 = none

constexpr uint8_t DEBOUNCE_MS = 5;
}  // namespace limit_switches
