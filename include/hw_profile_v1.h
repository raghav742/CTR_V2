#pragma once
#include "hw_types.h"

namespace hw {
constexpr const char* PROFILE_NAME = "V1 (Casper demonstrator)";

// --- Axes ---
constexpr uint8_t NUM_AXES = 6;
constexpr AxisConfig AXES[NUM_AXES] = {
  { "T1-Rot", AxisType::Rotation, 3, 4, 5, 6, 1, 0b00, 1500, false },
  // TODO: the other five
};

// --- Limit switches ---
constexpr uint8_t NUM_SWITCHES = 4;
constexpr SwitchConfig SWITCHES[NUM_SWITCHES] = {
  // TODO
};

// --- Mechanics ---
// TODO: full steps per rev, lead (mm), travel (mm), R_SENSE

// --- Interface ---
// TODO: encoder pins, RGB pins + polarity, buzzer, display type, OLED address, joystick type
}