#pragma once
// ============================================================
//  Rotation zero and parking (step 2.4)
//  The rotation axes have no sensor. Their zero is set by hand:
//  align a tube by eye, then "zero <axis>". "park" turns every
//  referenced rotation to the nearest full turn of its zero and
//  the homed carriages to the rest position (one coordinated
//  move), then stores "parked" in EEPROM. At the next power-on
//  the rotation zeros are restored from that (a motor that is
//  switched off stays where it is, within half a full step,
//  0.9 deg). The first rotation move after that clears the flag.
// ============================================================
#include <Arduino.h>

namespace rotzero {
void begin();                    // after settings::begin()
void update();                   // call every loop pass
bool referenced(uint8_t axis);   // rotation axis has a valid zero
bool setZero(int8_t axis, bool quiet = false);   // -1 = all rotation axes
bool park(bool quiet = false);                   // quiet: no OK lines (menu)
bool parking();
}  // namespace rotzero
