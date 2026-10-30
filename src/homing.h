#pragma once
// ============================================================
//  Homing (step 2.1)
//  Finds the zero of each translation axis: drive backward fast
//  (15 mm/s) until the carriage's own switch touches its block on
//  the frame, back off 2 mm, approach again slowly (5 mm/s), set
//  position 0 there, then move forward until the switch releases
//  (+1 mm). Order from the hardware profile
//  (back carriage first). Runs step by step from update(), so
//  the main loop and safety keep running.
//  Only the switch being searched is "expected"; any other
//  switch still causes a FAULT. A stop (command, encoder) aborts.
// ============================================================
#include <Arduino.h>

namespace homing {
void begin();
void update();                  // call every loop pass
bool start(int8_t axis, bool quiet = false);   // -1 = all in HOME_ORDER; else one axis.
                                               // quiet: no "OK" line (started by calib)
bool active();
bool axisHomed(uint8_t axis);   // translation axis has a valid zero
void invalidate(uint8_t axis);  // zero no longer trusted (e.g. driver disabled)
}  // namespace homing
