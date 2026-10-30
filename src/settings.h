#pragma once
// ============================================================
//  Settings kept in EEPROM (step 2.5)
//   - last accepted calib result (front_max, gaps)
//   - "parked" flag: the robot was parked before power-off, so
//     the rotation zeros are still valid at the next start
//  Written only on calib, park and the first rotation move after
//  a park (EEPROM lasts ~100,000 writes per cell).
//  Protected by a magic number, a version and a checksum; bad or
//  missing data is ignored (the hardware profile values are used).
// ============================================================
#include <Arduino.h>

namespace settings {
void begin();                                   // read EEPROM (call before limits/rotation)
bool calibStored();
void calibValues(float& frontMax, float gap[]);  // only if calibStored()
void saveCalib(float frontMax, const float gap[]);
void forgetCalib();                             // back to the profile values
bool parked();
void setParked(bool p);                         // writes only if it changes
void report();                                  // console "settings"
}  // namespace settings
