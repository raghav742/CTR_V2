#pragma once
// ============================================================
//  Step pulse generation from a timer interrupt (Timer1).
//  The interrupt runs STEP_TICK_HZ times per second. For each
//  moving axis it adds the axis' rate to an accumulator; when the
//  accumulator overflows, that axis gets one step pulse. So one
//  timer drives six axes at independent speeds.
//  The main loop only sets targets and rates; it never pulses.
//  Units here: microsteps.
// ============================================================
#include <Arduino.h>
#include "motion_config.h"

namespace steppers {
void    begin();                                 // cache pin ports, start Timer1
void    start(uint8_t axis, int32_t target);     // set direction, run until target
void    setRate(uint8_t axis, float stepsPerSec);
// Faster variant for the ramp loop: the raw per-tick increment
// (steps/s x INC_PER_SPS). Saves float work every millisecond.
constexpr float INC_PER_SPS = 65536.0f / motion_cfg::STEP_TICK_HZ;
void    setIncrement(uint8_t axis, uint16_t inc);
void    stop(uint8_t axis);                      // immediate stop
void    stopAll();
bool    running(uint8_t axis);
int32_t position(uint8_t axis);
int32_t remaining(uint8_t axis);                // microsteps left in this move
void    setPosition(uint8_t axis, int32_t pos);  // only while stopped

float   isrMaxUs();                              // longest interrupt (us) since last reset
void    resetIsrStats();
}  // namespace steppers
