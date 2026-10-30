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

// Homing (step 2.1). Each carriage carries its own switch, which presses
// against a fixed block on the frame when the carriage moves backward
// (negative mm). Blocks are staggered: T3 at the back end, T2 further forward,
// T1 near the front. Order: back carriage first, so each carriage has the most
// room behind it when it moves backward.
constexpr uint8_t NUM_HOMED_AXES = 3;
constexpr uint8_t HOME_ORDER[NUM_HOMED_AXES] = { 5, 3, 1 };   // T3-Tra, T2-Tra, T1-Tra

// Carriages front to back (step 2.2/2.3). The front carriage stops at its
// far-end switch. Every other carriage's front limit is the carriage in front
// of it: touching it presses the home switch on the BACK of that carriage
// (switch 38 on T1 is pressed by T1's block or by T2; switch 40 on T2 by T2's
// block or by T3). With all carriages homed there is a gap between them.
constexpr uint8_t CARRIAGE_ORDER[NUM_HOMED_AXES] = { 1, 3, 5 };   // T1, T2, T3
// Extra distance each carriage keeps from its measured upper limit, on top of
// LIMIT_MARGIN_MM (same order). T3: 50 mm (2026-10-30): at its full measured range
// (~615 mm) T3 disturbed the front of the robot (T1 far switch was hit).
constexpr float CARRIAGE_EXTRA_MARGIN_MM[NUM_HOMED_AXES] = { 0.0f, 0.0f, 50.0f };

// Known travel values (measured with "calib" on 2026-10-30; smallest value of the
// good runs = safe side). Loaded at startup, so the limits work right after
// "home" without running calib. calib rejects a new measurement that differs
// from these by more than CALIB_MAX_DEVIATION_MM (lost steps). 0 = unknown.
//   T1 stroke: 151.42, 151.19, 151.37 | gap T1/T2: 118.03, 118.07, 117.84
//   gap T2/T3: 349.13 (runs with T1 slipping gave 354.5, 354.4)
constexpr float CALIB_DEFAULT_FRONT_MAX_MM = 151.2f;
constexpr float CALIB_DEFAULT_GAP_MM[NUM_HOMED_AXES - 1] = { 117.8f, 349.1f };

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
