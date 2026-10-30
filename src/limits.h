#pragma once
// ============================================================
//  Travel limits of the translation carriages (steps 2.2, 2.3)
//  Carriages front to back: CARRIAGE_ORDER (T1, T2, T3).
//  Two rules between each carriage and the one behind it:
//   span    : front <= back + tolerance
//             (never further apart than at home: the tube between
//             them would hang unsupported and buckle). Needs homing.
//   contact : back <= front + gap - margin
//             (gap = distance at home, measured by "calib").
//   far end : front carriage <= frontMax - margin (calib).
//  Positions are in mm from each carriage's own zero.
//  Coordinated moves are proportional, so every difference changes
//  linearly during a move: checking the end positions covers the
//  whole path.
// ============================================================
#include <Arduino.h>

namespace limits {
void  begin();                  // profile values, or a plausible calib result from EEPROM
void  set(float frontMax, const float gap[]);   // from calib
bool  calibrated();
bool  active();                 // calibrated AND all carriages homed
// Would this move (relative amounts for the axes in mask) break a limit?
// Prints "ERR ..." and returns false if so.
bool  checkMove(const float amount[], uint8_t mask);
void  report();                 // console "limits"
}  // namespace limits
