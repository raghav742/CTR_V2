// ============================================================
//  CTR V2 firmware - Step 0.3 test
//  Prints the selected hardware profile at startup so it can be
//  checked against the real robot. Replaced in Phase 1.
// ============================================================
#include <Arduino.h>
#include "hw_profile.h"

static const __FlashStringHelper* typeName(AxisType t) {
  return t == AxisType::Rotation ? F("ROT") : F("TRA");
}
static const __FlashStringHelper* roleName(SwitchRole r) {
  return r == SwitchRole::Home ? F("HOME") : F("FAR-END");
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.print(F("Profile: ")); Serial.println(hw::PROFILE_NAME);
  Serial.println();

  for (uint8_t i = 0; i < hw::NUM_AXES; i++) {
    const AxisConfig& a = hw::AXES[i];
    Serial.print(F("Axis "));   Serial.print(i);
    Serial.print(' ');          Serial.print(a.name);
    Serial.print(' ');          Serial.print(typeName(a.type));
    Serial.print(F(" step="));  Serial.print(a.pinStep);
    Serial.print(F(" dir="));   Serial.print(a.pinDir);
    Serial.print(F(" en="));    Serial.print(a.pinEn);
    Serial.print(F(" diag="));  Serial.print(a.pinDiag);
    Serial.print(F(" bus="));   Serial.print(a.uartBus);
    Serial.print(F(" addr="));  Serial.print(a.uartAddr);
    Serial.print(F(" I="));     Serial.print(a.currentMa);
    Serial.print(F("mA inv=")); Serial.println(a.dirInvert);
  }
  Serial.println();

  for (uint8_t i = 0; i < hw::NUM_SWITCHES; i++) {
    const SwitchConfig& s = hw::SWITCHES[i];
    Serial.print(F("Switch ")); Serial.print(i);
    Serial.print(' ');          Serial.print(s.name);
    Serial.print(F(" pin="));   Serial.print(s.pin);
    Serial.print(' ');          Serial.print(roleName(s.role));
    Serial.print(F(" axis="));  Serial.print(s.axis);
    Serial.print(F(" activeHigh=")); Serial.println(s.activeHigh);
  }
  Serial.println();

  Serial.print(F("Lead "));   Serial.print(hw::LEAD_MM);
  Serial.print(F(" mm, travel ")); Serial.print(hw::TRAVEL_MM);
  Serial.print(F(" mm, ")); Serial.print(hw::FULLSTEPS_PER_REV);
  Serial.println(F(" full steps/rev"));
  Serial.println(F("Profile check done."));
}

void loop() {}
