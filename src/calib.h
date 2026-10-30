#pragma once
// ============================================================
//  Calibration of the front limits (step 2.2)
//  Needs all carriages homed. Runs as a list of operations:
//   1. front carriage (T1) forward to its far switch -> frontMax;
//      it stays there (1 mm off the switch)
//   2. T2 forward until it touches T1 (switch on T1's back)
//      -> gap T1/T2 = T2 pos - T1 pos at contact; T2 stays there
//   3. all three back together to CALIB_PARK_MM (50 mm), then
//      T3 forward until it touches T2 -> gap T2/T3 (at ~400 mm, not at
//      T3's full extension)
//   4. all carriages back to CALIB_REST_MM (T3, T2, T1)
//  Each approach: fast, back off 2 mm, slow (the slow one records).
//  Non-blocking like homing; uses the HOMING state. "stop" aborts
//  (previous values kept); switch not found -> FAULT.
// ============================================================
#include <Arduino.h>

namespace calib {
void begin();
void update();     // call every loop pass
bool start(bool quiet = false);   // quiet: no "OK" line (started from the menu)
bool active();
}  // namespace calib
