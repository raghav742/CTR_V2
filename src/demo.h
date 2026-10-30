#pragma once
// ============================================================
//  Demo sequencer (step 3.4)
//  A demo is a list of poses played as coordinated moves, in a
//  loop, until stopped (encoder press, "demo stop", fault).
//  A pose describes the tubes the way a concentric tube robot is
//  described: where the outer tube is, and how far each inner tube
//  sticks out beyond the one around it:
//    base  T1 position, 0..1000 = 0..100 % of its usable stroke
//    ext2  T2 beyond T1, 0..1000 = 0..100 % of the usable gap
//    ext3  T3 beyond T2, same
//    rot   rotations in degrees (from the rotation zero)
//  Every pose is inside the limits by construction (ext >= 0: span
//  rule; ext <= 100 %: no contact), and the same demo scales to V2.
//  Each step: move to the pose taking at least 'ms', then pause.
// ============================================================
#include <Arduino.h>

namespace demo {
void begin();
void update();                        // call every loop pass
uint8_t count();                      // number of programs
const __FlashStringHelper* name(uint8_t prog);
bool start(uint8_t prog, bool quiet = false);
void stop();
bool active();
uint8_t program();                    // running program
uint8_t step();                       // current step (0-based)
uint8_t steps();                      // steps in the running program
uint16_t loops();                     // completed loops
}  // namespace demo
