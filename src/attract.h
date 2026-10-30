#pragma once
// ============================================================
//  Attract mode (step 3.5)
//  When the robot has been idle for ATTRACT_IDLE_MS (homed,
//  READY, nothing moving, no encoder or console input), a demo
//  starts by itself; each time the next program. Any touch of the
//  encoder stops it (a press is STOP anyway; a turn also stops).
// ============================================================
#include <Arduino.h>

namespace attract {
void begin();
void update();          // call every loop pass
bool enabled();
void setEnabled(bool on);
bool running();         // the running demo was started by attract mode
}  // namespace attract
