#pragma once
// ============================================================
//  Joystick input (step 5.3, prepared for V2, UNTESTED ON HARDWARE)
//  APEM NZ 2-axis joystick. Two possible types (hardware profile):
//   Switched: one contact per direction -> deflection is 0 or full
//   Analog:   two voltages (X, Y)       -> deflection proportional
//  x() / y(): -100 .. +100 (percent), 0 = centre (inside dead zone).
//  Y = up/down, X = left/right. Debounced (switched) or filtered
//  (analog), read every 10 ms. Without a joystick both return 0.
//  Use (ui.cpp, Jog screen): Y moves the selected tube in and out,
//  X rotates it, in speed mode (motion::jogStart etc.).
// ============================================================
#include <Arduino.h>

namespace joystick {
void   begin();
void   update();        // every loop pass (reads every 10 ms)
int8_t x();
int8_t y();
bool   centred();       // both axes at 0
}  // namespace joystick
