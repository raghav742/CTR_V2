// ============================================================
//  Safety.ino  -  Eindschakelaars, grensbewaking en temperatuur
// ============================================================
#include "Config.h"

// Lees een schakelaar. Schmitt-trigger is inverterend -> HIGH = geraakt.
// pin < 0 betekent 'geen schakelaar' -> nooit geraakt.
bool switchPressed(int pin) {
  if (pin < 0) return false;
  int v = digitalRead(pin);
  return LIMIT_ACTIVE_HIGH ? (v == HIGH) : (v == LOW);
}
bool homeSwitchHit(int i) { return switchPressed(HOME_SWITCH_PIN[i]); }
bool farStopHit(int i)    { return switchPressed(FARSTOP_PIN[i]); }

// Log een statuswijziging van een schakelaar (debug + zichtbaarheid)
void logSwitch(int pin, bool hit, const char* name, const __FlashStringHelper* role) {
  int idx = pin - LIMIT_PIN_FIRST;
  if (idx < 0 || idx >= LIMIT_PIN_COUNT) return;
  if (hit != limitState[idx]) {
    limitState[idx] = hit;
    Serial.print(F("[LIMIT] ")); Serial.print(name); Serial.print(' ');
    Serial.print(role);
    Serial.print(F(" (pin ")); Serial.print(pin); Serial.print(')');
    Serial.println(hit ? F(" -> GERAAKT") : F(" -> vrij"));
  }
}

void checkLimitSwitches() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (HOME_SWITCH_PIN[i] >= 0) logSwitch(HOME_SWITCH_PIN[i], homeSwitchHit(i), MOTOR_NAMES[i], F("home"));
    if (FARSTOP_PIN[i]     >= 0) logSwitch(FARSTOP_PIN[i],     farStopHit(i),    MOTOR_NAMES[i], F("eindstop"));
  }
}

// (Veiligheid tijdens beweging zit nu in de blokkerende bewegingslus:
//  motionCheck() in Motion.ino controleert ver-eindstop + gebruiker-stop.)

// ============================================================
//  NOODSTOP / ERROR-state
// ============================================================
void triggerError(String msg) {
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].running   = false;
    motors[i].hasTarget = false;
    if (motors[i].connected) disableMotor(i);
  }
  systemMode = MODE_ERROR;
  errorMsg   = msg;
  g_stopFlag = false;   // wis stale vlag — bevestiging vereist vers drukken
  rgbWait();
  triggerBuzzer(BUZ_ERROR);
  Serial.print(F("[ERROR] ")); Serial.println(msg);
  updateOLED();
}

void handleError() {
  if (systemMode != MODE_ERROR) return;

  // g_stopFlag wordt gezet door monitorEncoderButton() in loop().
  // loop() draait normaal in MODE_ERROR (niet blokkerend) -> werkt betrouwbaar.
  if (!g_stopFlag) return;
  g_stopFlag = false;
  swEdge = false; swRelease = false;

  for (int i = 0; i < NUM_MOTORS; i++)
    if (motors[i].connected) enableMotor(i);
  systemMode = MODE_MENU;
  menuIndex  = 0;
  encDelta   = 0;
  rgbIdle();
  Serial.println(F("[ERROR] bevestigd -> menu"));
  waitButtonRelease();
  updateOLED();
}

// ============================================================
//  TEMPERATUURBEWAKING  (TMC2209 over-temperature flags)
//    otpw() = pre-warning, ot() = afslag
// ============================================================
void monitorTemperature() {
  static unsigned long last = 0;
  if (millis() - last < 5000) return;
  last = millis();

  for (int i = 0; i < NUM_MOTORS; i++) {
    if (!motors[i].connected) continue;
    if (tmc[i].ot()) {
      triggerError(String(F("M")) + String(i + 1) + F(" overheated"));
      return;
    }
    if (tmc[i].otpw()) {
      Serial.print(F("[TEMP] M")); Serial.print(i + 1);
      Serial.println(F(": waarschuwing (warm)"));
    }
  }
}
