#pragma once
// ============================================================
//  System state machine.
//  One place that knows what the robot is doing and which
//  actions are allowed. Every change is logged as an EV line.
//
//  BOOT -> IDLE (not homed) -> HOMING -> READY (homed)
//  READY -> MANUAL / DEMO / ATTRACT / VISITOR / REMOTE -> READY
//  any  -> FAULT  (needs acknowledgement: 'ack' or 1 s encoder press)
//  HOMING, MANUAL, DEMO, ATTRACT, VISITOR, REMOTE are filled in by
//  later steps; in step 1.4 they exist so transitions are defined.
// ============================================================
#include <Arduino.h>

namespace sys {
enum class State : uint8_t {
  Boot, Idle, Homing, Ready, Manual, Demo, Attract, Visitor, Remote, Fault
};

void   begin();                 // enters BOOT
void   update();                // call every loop pass (fault checks, LED)
State  get();
const __FlashStringHelper* name(State s);

bool   request(State to);       // false (+ ERR line) if not allowed from here
void   fault(const char* reason);   // stop everything, enter FAULT
bool   acknowledge();           // leave FAULT (to READY if homed, else IDLE)
const char* faultReason();

bool   homed();                 // set by homing (phase 2)
void   setHomed(bool h);

bool   motionAllowed();         // may a move command start now?
void   report();                // "OK state=..." line for the console
}  // namespace sys
