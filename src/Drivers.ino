// ============================================================
//  Drivers.ino  -  TMC2209 detectie en configuratie (via UART)
// ============================================================
#include "Config.h"

// Basisconfiguratie van de driver (per beweging stelt applyDriver() in
// Motion.ino microstepping/stroom/modus opnieuw in).
void configMotorTMC(int i) {
  uint16_t cur    = (motors[i].type == AX_ROT) ? CURRENT_ROT_MA : CURRENT_TRA_MA;
  bool     spread = (motors[i].type == AX_ROT) ? SPREAD_ROT     : SPREAD_TRA;

  tmc[i].rms_current(cur);
  tmc[i].microsteps(MS_MANUAL);
#if USE_INTERPOLATION
  tmc[i].intpol(true);            // interpoleer naar 256 (MicroPlyer): vloeiend
#endif
  tmc[i].en_spreadCycle(spread);
  tmc[i].pwm_autoscale(true);
}

bool testDriver(int i) {
  return tmc[i].test_connection() == 0;  // 0 = OK
}

// Eenmalige detectie + basisconfiguratie bij opstart
void initDrivers() {
  int found = 0;
  for (int i = 0; i < NUM_MOTORS; i++) {
    oled.clearDisplay();
    oledTitle("DRIVER INIT");
    oled.setCursor(3, 22);
    oled.print(MOTOR_NAMES[i]); oled.print(F(" ..."));
    oled.display();

    tmc[i].begin();
    bool ok = testDriver(i);
    motors[i].connected = ok;

    if (ok) {
      found++;
      // Motorpinnen instellen
      pinMode(MOTOR_PINS[i][P_STEP], OUTPUT);
      pinMode(MOTOR_PINS[i][P_DIR],  OUTPUT);
      pinMode(MOTOR_PINS[i][P_EN],   OUTPUT);
      pinMode(MOTOR_PINS[i][P_DIAG], INPUT);
      digitalWrite(MOTOR_PINS[i][P_STEP], LOW);
      digitalWrite(MOTOR_PINS[i][P_DIR],  LOW);
      digitalWrite(MOTOR_PINS[i][P_EN],   LOW);   // LOW = ingeschakeld (houdt positie)

      configMotorTMC(i);

      Serial.print(F("[INIT] M")); Serial.print(i + 1);
      Serial.print(F(" (")); Serial.print(MOTOR_NAMES[i]);
      Serial.print(F("): OK  I_rms=")); Serial.print(tmc[i].rms_current());
      Serial.println(F("mA"));
    } else {
      Serial.print(F("[INIT] M")); Serial.print(i + 1);
      Serial.print(F(" (")); Serial.print(MOTOR_NAMES[i]);
      Serial.println(F("): not found"));
    }
    delay(60);
  }
  Serial.print(F("[INIT] "));
  Serial.print(found); Serial.print('/'); Serial.print(NUM_MOTORS);
  Serial.println(F(" drivers found."));
}

// Herhaalbare scan (menu-item / serieel commando)
void scanDrivers() {
  Serial.println(F("[SCAN] Driver scan..."));
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oledTitle("DRIVER SCAN");

  int found = 0;
  for (int i = 0; i < NUM_MOTORS; i++) {
    bool ok = testDriver(i);
    motors[i].connected = ok;
    if (ok) { found++; configMotorTMC(i); }

    oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(3, 16 + i * 8);
    oled.print(MOTOR_NAMES[i]); oled.print(':');
    oled.print(ok ? F(" OK") : F(" --"));
    if (ok) { oled.print(F(" SG:")); oled.print(tmc[i].SG_RESULT()); }
    oled.display();

    Serial.print(F("[SCAN] ")); Serial.print(MOTOR_NAMES[i]); Serial.print(F(": "));
    if (ok) {
      Serial.print(F("OK | SG:"));   Serial.print(tmc[i].SG_RESULT());
      Serial.print(F(" | I_rms:")); Serial.print(tmc[i].rms_current());
      Serial.println(F("mA"));
    } else {
      Serial.println(F("not found"));
    }
    delay(80);
  }

  Serial.print(F("[SCAN] Result: "));
  Serial.print(found); Serial.print('/'); Serial.print(NUM_MOTORS); Serial.println();

  delay(2000);
  triggerBuzzer(found == NUM_MOTORS ? BUZ_OK : BUZ_BEEP);
  systemMode = MODE_MENU;
  menuIndex  = 0;
  rgbIdle();
  updateOLED();
}
