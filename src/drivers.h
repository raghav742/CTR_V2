#pragma once
// ============================================================
//  TMC2209 drivers: detection and configuration over UART.
//  Configuration follows Casper's tested setup (StealthChop,
//  interpolation to 256, automatic PWM scaling).
// ============================================================
#include <Arduino.h>

namespace drivers {
void     begin();                 // start UART buses, detect + configure all
bool     present(uint8_t axis);   // driver answered on UART
uint16_t microsteps(uint8_t axis);

// Chopper mode: Stealth = quiet (StealthChop), Spread = full torque (SpreadCycle),
// Hybrid = StealthChop below hybridSps steps/s, SpreadCycle above (TPWMTHRS).
enum class Chopper : uint8_t { Stealth, Spread, Hybrid };
void     setMicrosteps(uint8_t axis, uint16_t ms);   // caller rescales position
void     setChopper(uint8_t axis, Chopper mode, float hybridSps = 0);
void     setCurrent(uint8_t axis, uint16_t mA);
void     enable(uint8_t axis, bool on);   // EN pin (LOW = enabled)
bool     enabled(uint8_t axis);
void     report();                // print one line per driver (console)
bool     overTemp(uint8_t axis);        // over-temperature shutdown flag
bool     overTempWarning(uint8_t axis); // pre-warning flag
}  // namespace drivers
