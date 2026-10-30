#pragma once
// ============================================================
//  Safety manager: the single gatekeeper for motion.
//   - checkMove(): every move from any source asks here first
//   - unexpected limit switch during motion -> FAULT
//   - driver over-temperature / driver not answering -> FAULT
//   - user stop (encoder, console) with measured reaction time
//   - hardware watchdog: if the program freezes for 1 s, the
//     watchdog interrupt stops all step pulses, then resets
//   - reset cause reported at startup
// ============================================================
#include <Arduino.h>

namespace safety {
void begin();            // FIRST thing in setup(): reset cause, watchdog off
void armWatchdog();      // LAST thing in setup()
void update();           // every loop pass (feeds the watchdog)

// Pre-move check for one axis; prints "ERR ..." and returns false if refused
bool checkMove(uint8_t axis, float amount);
bool checkGroup(const float amount[], uint8_t mask);   // travel limits for the whole move

// Homing (phase 2) marks the switch it is looking for as expected
void expectSwitch(int8_t switchIdx, bool expected);
void setHomingMove(bool on);
void forceSpanRule(bool on);    // demos: apply the span rule to their moves too
bool spanRuleForced();    // homing's own moves: allowed while HOMING, no 50 mm cap

// Stop requested by a person; edgeUs = micros() of the input edge
void userStop(const __FlashStringHelper* source, uint32_t edgeUs);

void report();           // console "safety" command
void watchdogTest();     // console "wdtest": freeze on purpose
}  // namespace safety
