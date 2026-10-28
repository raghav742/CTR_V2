#pragma once
// ============================================================
//  Motion: physical units, acceleration ramps, move commands.
//  Converts mm / degrees to microsteps, ramps the speed up and
//  down from the main loop, and hands rates to the step interrupt.
//
//  Step 1.2 also contains a TEMPORARY safety net (replaced by the
//  safety manager in step 1.5):
//    - any limit switch becoming pressed during motion stops all axes
//    - moves into an already-pressed home / far-end switch are refused
// ============================================================
#include <Arduino.h>

namespace motion {
void  begin();
void  update();                                   // call every loop pass

// amount: mm (translation) or degrees (rotation)
// speed:  mm/s (translation) or rpm (rotation); 0 = default
bool  moveRelative(uint8_t axis, float amount, float speed = 0);
void  stopAll(const __FlashStringHelper* reason);
bool  busy(uint8_t axis);
bool  anyBusy();

float position(uint8_t axis);                     // mm or degrees
const __FlashStringHelper* unit(uint8_t axis);    // "mm" or "deg"
float stepsPerUnit(uint8_t axis);
}  // namespace motion
