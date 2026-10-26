// ============================================================
//  MSCTR  -  Mega Shield Concentric Tube Robot  -  Demonstrator
// ------------------------------------------------------------
//  Besturingssoftware voor de CTR-demonstrator (afstudeerproject
//  Casper de Ruiter, Benchmark / Saxion / UT).
//
//  Toestandsmachine (zie Flowcharts 1-5 in het eindverslag):
//      INIT  ->  MENU/IDLE  ->  KALIBRATIE  ->  DEMO
//                     \------------ ERROR ------------/
//  Statusled:  groen = idle/klaar, rood = wacht/kalibratie/error,
//              blauw = demo, geel = handmatige motorbesturing.
//
//  Code is opgedeeld in tabs:
//      Config.h     - pinnen, mapping, parameters (PAS HIER AAN)
//      Drivers.ino  - TMC2209 init / scan / configuratie
//      Motion.ino   - stapgeneratie, kalibratie/homing, demo
//      Interface.ino- OLED, encoder, RGB-led, buzzer
//      Safety.ino   - eindschakelaars, grensbewaking, temperatuur
//      SerialCmd.ino- seriele commando's (debug/test)
//
//  Vereiste libraries (Library Manager):
//      TMCStepper (teemuatlut), Adafruit GFX, Adafruit SSD1306
// ============================================================

#include <TMCStepper.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Config.h"

// ============================================================
//  GLOBALE OBJECTEN  (hier gedefinieerd zodat alle tabs ze zien)
// ============================================================
StepperMotor motors[NUM_MOTORS];

// Koppeling motor -> seriele bus + UART-adres
//   M1/M2 = Serial1, M3/M4 = Serial2, M5/M6 = Serial3
TMC2209Stepper tmc[NUM_MOTORS] = {
  TMC2209Stepper(&Serial1, R_SENSE, 0b00),  // M1  T1-Rot
  TMC2209Stepper(&Serial1, R_SENSE, 0b10),  // M2  T1-Tra
  TMC2209Stepper(&Serial2, R_SENSE, 0b00),  // M3  T2-Rot
  TMC2209Stepper(&Serial2, R_SENSE, 0b10),  // M4  T2-Tra
  TMC2209Stepper(&Serial3, R_SENSE, 0b00),  // M5  T3-Rot
  TMC2209Stepper(&Serial3, R_SENSE, 0b10),  // M6  T3-Tra
};

Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);

// ============================================================
//  GEDEELDE DEMO-VARIABELEN
// ============================================================
volatile bool  g_stopFlag    = false;   // gezet door monitorEncoderButton in loop()
unsigned long  g_demoStartMs = 0;       // tijdstip van demo-start (voor opstart-grace)

// ============================================================
//  SYSTEEM-TOESTAND
// ============================================================
SystemMode  systemMode  = MODE_INIT;
int         menuIndex   = 0;
int         selectedMotor = 0;
String      errorMsg    = "";

// Demo
int           demoStep   = 0;
unsigned long demoPhaseStart = 0;
bool          demoRunning = false;

// ============================================================
//  ENCODER-TOESTAND
// ============================================================
int  lastCLK   = HIGH;
int  encDelta  = 0;           // +/- stappen sinds laatste uitlezing
// Drukknop (debounce + flankdetectie)
int  swStatePrev = HIGH;
int  swRaw       = HIGH;
int  swRawPrev   = HIGH;
unsigned long swDebounceMs = 0;
bool swEdge    = false;       // eenmalige druk-puls (neergaande flank)
bool swRelease = false;       // eenmalige loslaat-puls (stijgende flank)

// ============================================================
//  EINDSCHAKELAAR-TOESTAND
// ============================================================
bool limitState[NUM_MOTORS]       = { false };
bool limitWaitRelease[NUM_MOTORS] = { false };

// ============================================================
//  OLED kalibratie-cache (alleen hertekenen bij wijziging)
// ============================================================
CalibState lastCalibState[NUM_MOTORS];

// ============================================================
//  BUZZER melodie-sequencer
// ============================================================
const Note* melBuffer = nullptr;
uint8_t       melLength = 0;
uint8_t       melIndex  = 0;
unsigned long melStart  = 0;
bool          melActive = false;

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial1.begin(TMC_BAUD);
  Serial2.begin(TMC_BAUD);
  Serial3.begin(TMC_BAUD);

  Wire.begin();
  Wire.setClock(400000);
  delay(100);

  // --- OLED ---
  bool oledOK = oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  Serial.println(oledOK ? F("[OLED] OK") : F("[OLED] not found (SDA=20 SCL=21)"));

  // --- Interface (encoder, RGB, buzzer, limit pins) ---
  initInterface();
  rgbWait();

  // --- Animated boot splash (logo + progress bar) ---
  bootSplash();

  // --- Detect + configure drivers ---
  initDrivers();

  // --- Init motor structures ---
  initMotors();

  // --- Ready -> main menu ---
  systemMode = MODE_MENU;
  menuIndex  = 0;
  rgbIdle();
  updateOLED();

  Serial.println(F("--- MSCTR ready ---"));
  Serial.println(F("Commands: MENU CALIB DEMO STOP SCAN STATUS"));
  Serial.println(F("          M<n> <mm|deg>   EN<n> / DIS<n>"));
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
  // 1. Continu-taken (achtergrond)
  handleBuzzer();
  readEncoder();
  monitorEncoderButton();   // raw ENC_SW loggen (debug)
  checkLimitSwitches();
  handleSerial();

  // 2. Toestand-afhandeling.
  //    Kalibratie en demo draaien BLOKKEREND (geramde, vloeiende
  //    stapgeneratie) en worden vanuit het menu gestart; ze keren
  //    daarna zelf terug naar MODE_MENU / MODE_ERROR.
  switch (systemMode) {
    case MODE_MENU:  handleMenu();  break;
    case MODE_ERROR: handleError(); break;
    default: break;
  }

  // 3. Bewaking + weergave (intern afgeknepen op tijd)
  monitorTemperature();
  refreshOLED();
}
