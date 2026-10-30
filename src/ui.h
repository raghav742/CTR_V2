#pragma once
// ============================================================
//  Screens and menu on the OLED (steps 3.1, 3.2)
//  Encoder: turn = choose, short press (on release) = select,
//  long press = back. While anything moves, main.cpp turns a
//  press into STOP before the UI sees it.
//  Main menu: Demo, Jog, Home, Status, Service
//  Service:   Calibrate limits, Zero rotations, Park, Driver scan,
//             Sound on/off, Back
//  The UI also follows the robot: FAULT -> error screen
//  (press = confirm), homing / calib / park -> progress screen
//  (also when started from the console), 2 min idle -> status.
// ============================================================
#include <Arduino.h>

namespace ui {
void begin();          // after display::begin()
void update();         // call every loop pass
bool motionActive();   // something moves / a sequence runs: a press means STOP
void onTurn(int16_t d);
void onSelect();       // short press (released before the long-press time)
void onBack();         // long press
}  // namespace ui
