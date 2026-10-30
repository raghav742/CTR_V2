#pragma once
// ============================================================
//  Serial console = protocol v1 (docs/protocol.md)
//  Reads characters as they arrive and runs a command when a
//  full line is in. Never waits.
//  Line types: replies "OK"/"ERR", events "EV", status "ST".
//  "#<id> <command>" tags every reply line with "#<id> ".
// ============================================================
#include <Arduino.h>

namespace console {
void begin();
void update();          // call every loop pass
bool watching();        // "watch on": print input events live
uint32_t lastCommandMs(); // millis() of the last command (attract mode: activity)
void updateStream();    // ST line every <ms> if "stream" is on (called by update)
}  // namespace console
