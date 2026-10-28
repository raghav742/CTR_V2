#pragma once
// ============================================================
//  Hardware profile: V1 robot (Casper de Ruiter demonstrator)
//  Arduino Mega 2560 + MSCTR shield, 6x TMC2209.
//  Values verified against the PCB schematic (Eindverslag,
//  Bijlage A) and Casper's tested code (reference_v1/Config.h).
// ============================================================
#include "hw_types.h"

namespace hw {
constexpr const char* PROFILE_NAME = "V1 (Casper demonstrator)";

// --- Axes ---------------------------------------------------
// Each tube has one rotation and one translation axis.
// dirInvert: Casper's DIR_FWD_LEVEL was LOW for M2 and M4.
// Currents: Casper's values (rotation DINGS 17HS2048, translation Nanotec LA561S20).
constexpr uint8_t NUM_AXES = 6;
constexpr AxisConfig AXES[NUM_AXES] = {
  //  name      type                   step dir  en diag bus addr  mA    invert
  { "T1-Rot", AxisType::Rotation,      3,   4,   5,  6,  1, 0b00, 1500, false },
  { "T1-Tra", AxisType::Translation,   8,   9,  10, 11,  1, 0b10, 1900, true  },
  { "T2-Rot", AxisType::Rotation,     22,  23,  24, 25,  2, 0b00, 1500, false },
  { "T2-Tra", AxisType::Translation,  26,  27,  28, 29,  2, 0b10, 1900, true  },
  { "T3-Rot", AxisType::Rotation,     30,  31,  32, 33,  3, 0b00, 1500, false },
  { "T3-Tra", AxisType::Translation,  34,  35,  36, 37,  3, 0b10, 1900, false },
};

// --- Limit switches -----------------------------------------
// Signal path: switch -> RC filter -> SN74AHCT14 (inverting) -> pin.
// Normally-open switches, so a pressed switch reads HIGH.
// Pins 41 and 43 are wired on the PCB but have no switch on V1.
constexpr uint8_t NUM_SWITCHES = 4;
constexpr SwitchConfig SWITCHES[NUM_SWITCHES] = {
  //  name        pin  role                axis activeHigh
  { "T1-home",    38, SwitchRole::Home,    1,   true },   // mounted on T1 carriage
  { "T1-far",     39, SwitchRole::FarEnd,  1,   true },   // end of the rail
  { "T2-home",    40, SwitchRole::Home,    3,   true },
  { "T3-home",    42, SwitchRole::Home,    5,   true },
};

// --- Mechanics ----------------------------------------------
constexpr uint16_t FULLSTEPS_PER_REV = 200;     // 1.8 deg motors
constexpr float    LEAD_MM           = 2.0f;    // lead screw: 0.01 mm per full step (Nanotec)
constexpr float    TRAVEL_MM         = 200.0f;  // usable translation stroke
constexpr float    R_SENSE           = 0.11f;   // TMC2209 breakout sense resistor (ohm)
constexpr uint32_t TMC_BAUD          = 115200;  // UART to the drivers

// Microstepping per axis type. Driver interpolates to 256 internally (smooth).
// TRA 8: 20 mm/s = 16,000 steps/s, fits under the 20 kHz step timer.
constexpr uint16_t MICROSTEPS_TRA    = 8;
constexpr uint16_t MICROSTEPS_ROT    = 16;

// --- Interface ----------------------------------------------
constexpr uint8_t PIN_ENC_A    = 46;   // encoder CLK
constexpr uint8_t PIN_ENC_B    = 45;   // encoder DT

// Encoder push button: measured 2026-10-28. The encoder board has its own
// pull-down, and the button connects to the encoder "+" pin -> pressed = HIGH.
// Use WITHOUT the internal pull-up (with it, idle sits near 1 V = noise-prone).
constexpr uint8_t PIN_ENC_SW         = 44;
constexpr bool    ENC_SW_ACTIVE_HIGH = true;
constexpr bool    ENC_SW_PULLUP      = false;

// RGB LED: measured 2026-10-28. On V1 the "+" (common anode) and red wires
// are swapped between PCB connector P1 and the encoder:
//   pin 49 (PCB "ENC_R") actually feeds the encoder "+" = LED supply AND button.
//   It must be driven HIGH at all times, or LEDs and button stop working.
//   Red is not available on V1. Colours: green, blue, cyan.
constexpr uint8_t PIN_ENC_SUPPLY = 49;
constexpr bool    LED_HAS_RED    = false;
constexpr uint8_t PIN_LED_G      = 48;   // LOW = on
constexpr uint8_t PIN_LED_B      = 50;   // LOW = on (also SPI MISO)
constexpr bool    LED_ACTIVE_LOW = true;
constexpr uint8_t PIN_BUZZER     = 47;

constexpr DisplayType  DISPLAY_TYPE  = DisplayType::OLED_SSD1306;
constexpr uint8_t      OLED_ADDR     = 0x3C;
constexpr JoystickType JOYSTICK_TYPE = JoystickType::None;
}  // namespace hw
