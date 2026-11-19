#pragma once
// ============================================================
//  Hardware profile: V2 robot
//
//  *** PREPARED BUT UNTESTED ON HARDWARE (2026-11) ***
//  The V2 hardware was not built during the internship. This
//  profile is filled in from the V2 design (BoM, pin plan in
//  docs/pinmap.md) so the v2 environment compiles. Every value
//  marked TODO must be checked on the real robot during bring-up
//  (docs/HANDOVER.md, "V2 bring-up").
//
//  Differences from V1:
//   - Nanotec SCA4218M1804-L rotation motors (1.8 A rated)
//   - 2.8" TFT (Adafruit 1770, ILI9341) instead of the OLED;
//     touch NOT used: the encoder operates the menu as on V1
//   - APEM NZ 2-axis joystick for manual control
//   - encoder cable corrected: "+" on 5 V, red LED on pin 49
//   - no V1 restrictions: full speeds, limit calibration on
// ============================================================
#include "hw_types.h"

namespace hw {
constexpr const char* PROFILE_NAME = "V2 (prepared, UNTESTED)";

// --- Axes ---------------------------------------------------
// Same MSCTR PCB pinout as V1.
// TODO(V2): check every direction on the robot ("move <n> 5": translation
//           must move FORWARD for +, rotation turn the agreed way) and flip
//           dirInvert where needed.
// Currents: rotation SCA4218M1804-L is rated 1.8 A per winding; start at
// 1500 mA (cooler, like V1) and raise with "cur" only if a rotation skips.
// Translation: same Nanotec LA561S20 as V1 (2 A rated, V1 ran 1.9 A).
constexpr uint8_t NUM_AXES = 6;
constexpr AxisConfig AXES[NUM_AXES] = {
  //  name      type                   step dir  en diag bus addr  mA    invert
  { "T1-Rot", AxisType::Rotation,      3,   4,   5,  6,  1, 0b00, 1500, false },  // TODO(V2) dir
  { "T1-Tra", AxisType::Translation,   8,   9,  10, 11,  1, 0b10, 1900, true  },  // TODO(V2) dir
  { "T2-Rot", AxisType::Rotation,     22,  23,  24, 25,  2, 0b00, 1500, false },  // TODO(V2) dir
  { "T2-Tra", AxisType::Translation,  26,  27,  28, 29,  2, 0b10, 1900, true  },  // TODO(V2) dir
  { "T3-Rot", AxisType::Rotation,     30,  31,  32, 33,  3, 0b00, 1500, false },  // TODO(V2) dir
  { "T3-Tra", AxisType::Translation,  34,  35,  36, 37,  3, 0b10, 1900, false },  // TODO(V2) dir
};

// --- Limit switches -----------------------------------------
// Same 4 switches and same arrangement as V1 (confirmed 2026-11-19):
// a home switch on the back of each carriage, a far-end switch for T1.
// TODO(V2): the V2 plan mentions normally-CLOSED wiring (a broken wire then
//           looks like a pressed switch = safe). If the switches are wired NC,
//           set activeHigh = false for all four. Check with "inputs" + "watch on".
constexpr uint8_t NUM_SWITCHES = 4;
constexpr SwitchConfig SWITCHES[NUM_SWITCHES] = {
  //  name        pin  role                axis activeHigh
  { "T1-home",    38, SwitchRole::Home,    1,   true },
  { "T1-far",     39, SwitchRole::FarEnd,  1,   true },
  { "T2-home",    40, SwitchRole::Home,    3,   true },
  { "T3-home",    42, SwitchRole::Home,    5,   true },
};

constexpr uint8_t NUM_HOMED_AXES = 3;
constexpr uint8_t HOME_ORDER[NUM_HOMED_AXES]     = { 5, 3, 1 };   // T3, T2, T1 (back first)
constexpr uint8_t CARRIAGE_ORDER[NUM_HOMED_AXES] = { 1, 3, 5 };   // T1, T2, T3 (front to back)

// No extra margins: V2 is expected not to slip. TODO(V2): if calibration shows
// slip or a carriage disturbs others near full extension, add margin here
// (V1 needed 20 / 20 / 50 mm).
constexpr float CARRIAGE_EXTRA_MARGIN_MM[NUM_HOMED_AXES] = { 0.0f, 0.0f, 0.0f };

// Travel values: unknown until measured. 0 = unknown -> the travel limits stay
// inactive (demos refused, moves capped) until "calib" has run once.
// TODO(V2): run "home" then "calib" 3x; copy the smallest good values here so
//           the limits work right after homing, even with an empty EEPROM.
constexpr float CALIB_DEFAULT_FRONT_MAX_MM = 0.0f;
constexpr float CALIB_DEFAULT_GAP_MM[NUM_HOMED_AXES - 1] = { 0.0f, 0.0f };

// --- No V1 restrictions --------------------------------------
constexpr float TRA_SPEED_CAP_MMPS = 20.0f;   // = motion_cfg::TRA_SPEED_MAX (requirement FRT_6)
constexpr float ROT_SPEED_CAP_RPM  = 120.0f;  // = motion_cfg::ROT_SPEED_MAX
constexpr float ACCEL_SCALE        = 1.0f;
constexpr bool  CALIB_ENABLED      = true;

// --- Mechanics ----------------------------------------------
constexpr uint16_t FULLSTEPS_PER_REV = 200;     // both motor types 1.8 deg
constexpr float    LEAD_MM           = 2.0f;    // same Nanotec lead screw
constexpr float    TRAVEL_MM         = 200.0f;  // informative only; real limits come from calib
constexpr float    R_SENSE           = 0.11f;   // TODO(V2): sense resistor of the V2 driver boards
constexpr uint32_t TMC_BAUD          = 115200;
constexpr uint16_t MICROSTEPS_TRA    = 8;
constexpr uint16_t MICROSTEPS_ROT    = 16;

// --- Interface ----------------------------------------------
constexpr uint8_t PIN_ENC_A    = 46;
constexpr uint8_t PIN_ENC_B    = 45;
constexpr uint8_t PIN_ENC_SW         = 44;
constexpr bool    ENC_SW_ACTIVE_HIGH = true;    // button to encoder "+" (now 5 V), board pull-down
constexpr bool    ENC_SW_PULLUP      = false;

// Encoder RGB LED with the corrected cable: "+" to 5 V, red cathode on pin 49.
// Blue: pin 50 is SPI MISO, which the TFT's SPI bus takes over (in SPI master
// mode the pin is forced to input), so blue cannot work on pin 50 on V2.
// Without blue the status colours are mapped to red/green (status_led.cpp).
// TODO(V2): to keep blue, wire it to a free pin (e.g. A8) and set it here.
constexpr uint8_t PIN_ENC_SUPPLY = 0xFF;        // none: "+" is on 5 V now
constexpr bool    LED_HAS_RED    = true;
constexpr bool    LED_HAS_BLUE   = false;
constexpr uint8_t PIN_LED_R      = 49;          // LOW = on
constexpr uint8_t PIN_LED_G      = 48;          // LOW = on
constexpr uint8_t PIN_LED_B      = 0xFF;        // not connected (see above)
constexpr bool    LED_ACTIVE_LOW = true;
constexpr uint8_t PIN_BUZZER     = 47;

// Display: Adafruit 1770, 2.8" 320x240 TFT, ILI9341, hardware SPI.
// Used as a larger copy of the V1 screens (no touch).
constexpr DisplayType DISPLAY_TYPE = DisplayType::TFT_ILI9341;
constexpr uint8_t OLED_ADDR    = 0x3C;          // unused on V2 (kept for the shared code)
constexpr uint8_t PIN_TFT_CS   = 53;            // SPI MOSI 51, SCK 52 (fixed by the Mega)
constexpr uint8_t PIN_TFT_DC   = 12;
constexpr uint8_t PIN_TFT_RST  = 13;
constexpr uint8_t PIN_TFT_LITE = 7;             // backlight, HIGH = on

// Joystick: APEM NZ series, 2 axes. Most likely SWITCHED (a contact per
// direction, no proportional output). TODO(V2): confirm on the part.
//   Switched: 4 contacts to GND, internal pull-ups -> pins below (UP/DOWN/LEFT/RIGHT)
//   Analog:   X on A0, Y on A1 (centre ~512)  -> set JOYSTICK_TYPE = Analog
constexpr JoystickType JOYSTICK_TYPE = JoystickType::Switched;
constexpr uint8_t PIN_JOY_UP    = A0;
constexpr uint8_t PIN_JOY_DOWN  = A1;
constexpr uint8_t PIN_JOY_LEFT  = A2;
constexpr uint8_t PIN_JOY_RIGHT = A3;
constexpr uint8_t PIN_JOY_X     = A0;           // analog type only
constexpr uint8_t PIN_JOY_Y     = A1;           // analog type only
}  // namespace hw

// Optional hardware compiled in
#define HW_HAS_TFT      1
#define HW_HAS_JOYSTICK 1
