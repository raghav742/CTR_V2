#pragma once
#include <Arduino.h>

enum class AxisType     : uint8_t { Rotation, Translation };
enum class DisplayType  : uint8_t { None, OLED_SSD1306, TFT_ILI9341 };
enum class JoystickType : uint8_t { None, Switched, Analog };
enum class SwitchRole   : uint8_t { Home, FarEnd };

struct AxisConfig {
  const char* name;       // e.g. "T1-Rot"
  AxisType    type;
  uint8_t     pinStep, pinDir, pinEn, pinDiag;
  uint8_t     uartBus;    // 1..3 = Serial1..Serial3
  uint8_t     uartAddr;   // TMC2209 address on that bus
  uint16_t    currentMa;  // RMS motor current
  bool        dirInvert;  // true = flip direction
};

struct SwitchConfig {
  const char* name;
  uint8_t     pin;
  SwitchRole  role;
  uint8_t     axis;       // index into AXES[]
  bool        activeHigh; // pin level when the switch is pressed
};