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

// LED test helper: LOW = on. V1 has no red (pin 49 is the encoder supply).
static void setLed(char c) {
  bool g = (c == 'g' || c == 'c'), b = (c == 'b' || c == 'c');
  digitalWrite(hw::PIN_LED_G, g ? LOW : HIGH);
  digitalWrite(hw::PIN_LED_B, b ? LOW : HIGH);
  Serial.print(F("LED -> ")); Serial.println(c);
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
  Serial.println();

  // --- Input test: report every change on encoder and switches ---
  // Encoder supply (pin 49) must be HIGH before LEDs or button work.
  pinMode(hw::PIN_ENC_SUPPLY, OUTPUT);
  digitalWrite(hw::PIN_ENC_SUPPLY, HIGH);
  pinMode(hw::PIN_ENC_A, INPUT_PULLUP);
  pinMode(hw::PIN_ENC_B, INPUT_PULLUP);
  pinMode(hw::PIN_ENC_SW, hw::ENC_SW_PULLUP ? INPUT_PULLUP : INPUT);
  pinMode(hw::PIN_LED_G, OUTPUT);
  pinMode(hw::PIN_LED_B, OUTPUT);
  setLed('g');
  for (uint8_t i = 0; i < hw::NUM_SWITCHES; i++) pinMode(hw::SWITCHES[i].pin, INPUT);
  Serial.println(F("Input test: turn/press the encoder, press each limit switch."));
  Serial.println(F("Type g / b / c (cyan) / o (off) + Enter to change the LED."));
  Serial.print(F("Idle levels: ENC_SW=")); Serial.print(digitalRead(hw::PIN_ENC_SW));
  for (uint8_t i = 0; i < hw::NUM_SWITCHES; i++) {
    Serial.print(' '); Serial.print(hw::SWITCHES[i].name);
    Serial.print('='); Serial.print(digitalRead(hw::SWITCHES[i].pin));
  }
  Serial.println();
}

// Previous levels, to print only changes
static int8_t lastSw = -1, lastA = -1;
static int8_t lastLim[hw::NUM_SWITCHES];
static bool   limInit = false;
static long   encCount = 0;

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'g' || c == 'b' || c == 'c' || c == 'o') setLed(c);
  }
  int sw = digitalRead(hw::PIN_ENC_SW);
  if (sw != lastSw) {
    if (lastSw != -1) { Serial.print(F("ENC_SW pin ")); Serial.print(hw::PIN_ENC_SW);
                        Serial.print(F(" -> ")); Serial.println(sw ? F("HIGH") : F("LOW")); }
    lastSw = sw;
  }

  int a = digitalRead(hw::PIN_ENC_A);
  if (a != lastA) {
    if (lastA != -1 && a == LOW) {
      encCount += (digitalRead(hw::PIN_ENC_B) != a) ? 1 : -1;
      Serial.print(F("Encoder count ")); Serial.println(encCount);
    }
    lastA = a;
  }

  for (uint8_t i = 0; i < hw::NUM_SWITCHES; i++) {
    int v = digitalRead(hw::SWITCHES[i].pin);
    if (!limInit) lastLim[i] = v;
    if (v != lastLim[i]) {
      Serial.print(F("Switch ")); Serial.print(hw::SWITCHES[i].name);
      Serial.print(F(" pin ")); Serial.print(hw::SWITCHES[i].pin);
      Serial.print(F(" -> ")); Serial.println(v ? F("HIGH") : F("LOW"));
      lastLim[i] = v;
    }
  }
  limInit = true;
}
