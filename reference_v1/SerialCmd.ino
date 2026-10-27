// ============================================================
//  SerialCmd.ino  -  Seriele commando's (debug / handmatig testen)
// ------------------------------------------------------------
//  Commando's (115200 baud, regel afsluiten met newline):
//    MENU            terug naar hoofdmenu
//    CALIB           start kalibratie (homing translatie-assen)
//    DEMO            start demo (vereist kalibratie)
//    SCAN            opnieuw drivers detecteren
//    STATUS          toon status van alle assen
//    STOP            (tijdens beweging) stop / afbreken
//    M<n> <waarde>   beweeg motor n  (mm voor TRA, graden voor ROT)
//                    bv. "M2 50"  = T1-Tra 50 mm,  "M1 -90" = T1-Rot -90 graden
//    EN<n> / DIS<n>  motor in-/uitschakelen (vrij bewegen)
// ============================================================
#include "Config.h"

void printStatus() {
  Serial.println(F("---------- STATUS ----------"));
  for (int i = 0; i < NUM_MOTORS; i++) {
    StepperMotor &m = motors[i];
    Serial.print('M'); Serial.print(i + 1);
    Serial.print(F(" ")); Serial.print(MOTOR_NAMES[i]); Serial.print(F(": "));
    if (!m.connected) { Serial.println(F("N/A")); continue; }

    Serial.print(m.type == AX_TRA ? F("[TRA] ") : F("[ROT] "));
    switch (m.state) {
      case CAL_IDLE:      Serial.print(F("idle  ")); break;
      case CAL_FIND_ZERO: Serial.print(F("homing")); break;
      case CAL_DONE:      Serial.print(F("klaar ")); break;
      default:            Serial.print(F("--    ")); break;
    }
    Serial.print(F(" pos="));
    if (m.type == AX_TRA) { Serial.print(fullStepsToMM(m.position), 2);  Serial.print(F("mm")); }
    else                  { Serial.print(fullStepsToDeg(m.position), 1); Serial.print(F("deg")); }
    if (m.type == AX_TRA) { Serial.print(F(" slag=")); Serial.print(fullStepsToMM(m.max_position), 0); Serial.print(F("mm")); }
    Serial.print(F(" I=")); Serial.print(tmc[i].rms_current()); Serial.print(F("mA"));
    if (tmc[i].ot())        Serial.print(F(" OT!"));
    else if (tmc[i].otpw()) Serial.print(F(" warm"));
    Serial.println();
  }
  Serial.println(F("----------------------------"));
}

void handleSerial() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;

  String up = cmd; up.toUpperCase();

  // --- zonder argument ---
  if (up == "MENU")   { systemMode = MODE_MENU; menuIndex = 0; rgbIdle(); updateOLED(); return; }
  if (up == "CALIB")  { runCalibration(); updateOLED(); return; }
  if (up == "DEMO")   { runDemo();        updateOLED(); return; }
  if (up == "SCAN")   { scanDrivers();    return; }
  if (up == "STATUS") { printStatus();    return; }
  if (up == "STOP")   { for (int i = 0; i < NUM_MOTORS; i++) disableMotor(i);
                        Serial.println(F("[CMD] motoren uit")); return; }

  // --- met index + (optioneel) waarde ---
  int sp = cmd.indexOf(' ');
  String tok = (sp < 0) ? up : up.substring(0, sp);   // bv "M2", "EN1"
  float val  = (sp < 0) ? 0  : cmd.substring(sp + 1).toFloat();

  uint8_t k = 0;
  while (k < tok.length() && isAlpha(tok[k])) k++;
  String pfx = tok.substring(0, k);
  int idx = tok.substring(k).toInt() - 1;

  if (idx < 0 || idx >= NUM_MOTORS) { Serial.println(F("? motor 1..6")); return; }
  if (!motors[idx].connected)       { Serial.print(F("M")); Serial.print(idx+1); Serial.println(F(" niet verbonden")); return; }

  if (pfx == "M") {                                   // beweeg (blokkerend)
    if (motors[idx].type == AX_TRA) {
      long delta = mmToFullSteps(val);
      if (motors[idx].state == CAL_DONE) {            // klamp binnen slag
        long target = motors[idx].position + delta;
        long lo = 0, hi = motors[idx].max_position;
        if (hi < lo) { long t = lo; lo = hi; hi = t; }
        if (target < lo) target = lo;
        if (target > hi) target = hi;
        delta = target - motors[idx].position;
      }
      moveAxisFull(idx, delta, MANUAL_TRA_MMPS * FULLSTEPS_PER_MM, MS_MANUAL, CURRENT_TRA_MA, SPREAD_TRA);
    } else {
      rotateDeg(idx, val, MANUAL_ROT_RPM, MS_MANUAL);
    }
    if (g_stopReason == 2) triggerError(g_safetyMsg);
    Serial.print(F("[CMD] M")); Serial.print(idx + 1);
    Serial.print(F(" -> ")); Serial.print(val);
    Serial.println(motors[idx].type == AX_TRA ? F(" mm") : F(" deg"));
    updateOLED();
  }
  else if (pfx == "EN")  { enableMotor(idx);  Serial.print(F("[CMD] M")); Serial.print(idx+1); Serial.println(F(" aan")); }
  else if (pfx == "DIS") { disableMotor(idx); Serial.print(F("[CMD] M")); Serial.print(idx+1); Serial.println(F(" uit")); }
  else { Serial.print(F("? onbekend: ")); Serial.println(cmd); }
}
