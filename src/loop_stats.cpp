#include "loop_stats.h"

namespace loop_stats {
namespace {
  uint32_t lastPassUs  = 0;
  uint32_t windowStart = 0;
  uint32_t count       = 0;
  uint32_t worstUs     = 0;
  uint32_t reportLps   = 0;
  uint32_t reportWorst = 0;
}

void update() {
  const uint32_t nowUs = micros();
  if (lastPassUs != 0) {
    uint32_t pass = nowUs - lastPassUs;
    if (pass > worstUs) worstUs = pass;
  }
  lastPassUs = nowUs;
  count++;

  const uint32_t nowMs = millis();
  if (nowMs - windowStart >= 1000) {
    reportLps   = count;
    reportWorst = worstUs;
    count = 0; worstUs = 0; windowStart = nowMs;
  }
}

uint32_t loopsPerSecond() { return reportLps; }
uint32_t maxLoopUs()      { return reportWorst; }
}  // namespace loop_stats
