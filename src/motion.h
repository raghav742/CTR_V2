#pragma once
// ============================================================
//  Motion: physical units, acceleration ramps, move commands.
//  Converts mm / degrees to microsteps, ramps the speed up and
//  down from the main loop, and hands rates to the step interrupt.
//  Moves are coordinated: axes in one move share one speed profile
//  and finish together (step 1.3).
//  Every move is checked by the safety manager (safety.h) first.
// ============================================================
#include <Arduino.h>

namespace motion {
void  begin();
void  update();                                   // call every loop pass

// amount: mm (translation) or degrees (rotation)
// speed:  mm/s (translation) or rpm (rotation); 0 = default
bool  moveRelative(uint8_t axis, float amount, float speed = 0, bool quiet = false);

// Coordinated move: all axes in 'mask' start and finish together.
// amount[i] relative (mm or deg); speedPct of the fastest allowed;
// duration > 0 = take exactly that many seconds; singleSpeed: see .cpp
// duration > 0: take exactly this long (ERR if impossible); < 0: at least |duration|
bool  moveGroup(const float amount[], uint8_t mask, float speedPct = 100,
                float duration = 0, float singleSpeed = 0, bool quiet = false);
void  stopAll(const __FlashStringHelper* reason);
void  stopGroupOf(uint8_t axis);                  // stop just the move this axis is in
uint32_t stopCount();                             // +1 on every stopAll (lets homing see a stop)
bool  busy(uint8_t axis);
bool  anyBusy();

// ---- Speed mode (step 5.3 prep, for the V2 joystick; tested on V1 via "jog") ----
// The axis moves toward its travel limit in 'dir' (+1 / -1) at speedFrac (0..1)
// of its maximum speed. The speed can be changed while it moves; release brakes
// along the normal ramp and stops. The move ends by itself at the travel limit
// (translation) or after JOG_ROT_MAX_DEG (rotation). All normal safety checks apply.
bool  jogStart(uint8_t axis, int8_t dir, float speedFrac, bool quiet = true);
void  jogSpeed(uint8_t axis, float speedFrac);   // change speed of a running jog
void  jogRelease(uint8_t axis);                   // brake to a stop
bool  jogging(uint8_t axis);                      // axis is in a speed-mode move

float position(uint8_t axis);                     // mm or degrees
const __FlashStringHelper* unit(uint8_t axis);    // "mm" or "deg"
float stepsPerUnit(uint8_t axis);
}  // namespace motion
