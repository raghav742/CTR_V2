#pragma once
// ============================================================
//  Main-loop statistics: proves the loop never blocks.
//  Reports loops per second and the slowest single pass (us)
//  over the last full second.
// ============================================================
#include <Arduino.h>

namespace loop_stats {
void     update();          // call once per loop pass (first thing in loop())
uint32_t loopsPerSecond();
uint32_t maxLoopUs();       // slowest pass in the last second
}  // namespace loop_stats
