#pragma once
// ============================================================
//  Serial console (non-blocking)
//  Reads characters as they arrive and runs a command when a
//  full line is in. Never waits. Becomes the laptop protocol
//  in step 1.6.
//  Replies start with "OK" or "ERR"; events start with "EV".
// ============================================================
#include <Arduino.h>

namespace console {
void begin();
void update();          // call every loop pass
bool watching();        // "watch on": print input events live
}  // namespace console
