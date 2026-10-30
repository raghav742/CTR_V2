#pragma once
// ============================================================
//  Record & replay (step 3.6)
//  Stores up to MAX_POSES poses (all 6 axes, absolute, 0.1 mm or
//  0.1 deg) taken from the current position, e.g. after jogging.
//  Kept in EEPROM (own block after the settings), so a recorded
//  choreography survives power-off. Played back by the demo
//  sequencer (demo::startRecorded).
// ============================================================
#include <Arduino.h>
#include "hw_profile.h"

namespace record {
constexpr uint8_t MAX_POSES = 10;
void    begin();                    // read EEPROM
uint8_t count();
bool    add();                      // current position as the next pose
void    clear();
const int16_t* pose(uint8_t i);     // hw::NUM_AXES values, 0.1 mm / 0.1 deg
void    report();                   // console "rec"
}  // namespace record
