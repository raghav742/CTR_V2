// ============================================================
//  MSCTR  -  Mega Shield Concentric Tube Robot
//  Config.h  -  Pinnen, motor-mapping en instelbare parameters
// ------------------------------------------------------------
//  Alle hardware-afhankelijke instellingen staan in DIT bestand.
//  Pas hier aan; de rest van de code is data-gestuurd.
//
//  Hardware : Arduino Mega 2560 + custom shield "MSCTR"
//             6x TMC2209 (BigTreeTech BOB) via hardware-UART
//             OLED 128x64 I2C, RGB rotary encoder, piezo buzzer,
//             eindschakelaars via RC-filter + Schmitt-trigger.
//  Bron     : Eindverslag CTR + PCB-netlist (ArduinoShieldForCTR)
// ============================================================
#ifndef MSCTR_CONFIG_H
#define MSCTR_CONFIG_H

#include <Arduino.h>

// ============================================================
//  SYSTEEM
// ============================================================
#define NUM_MOTORS     6
#define R_SENSE        0.11f    // BigTreeTech TMC2209 sense-weerstand
#define SERIAL_BAUD    115200   // USB seriele monitor
#define TMC_BAUD       115200   // UART naar de drivers

// ============================================================
//  MOTOR-PINNEN   [STEP, DIR, EN, DIAG]   (uit PCB-netlist)
//  index 0..5  =  M1..M6
// ============================================================
const uint8_t MOTOR_PINS[NUM_MOTORS][4] = {
  {  3,  4,  5,  6 },   // M1
  {  8,  9, 10, 11 },   // M2
  { 22, 23, 24, 25 },   // M3
  { 26, 27, 28, 29 },   // M4
  { 30, 31, 32, 33 },   // M5
  { 34, 35, 36, 37 },   // M6
};
// Index in bovenstaande rijen:
#define P_STEP 0
#define P_DIR  1
#define P_EN   2
#define P_DIAG 3

// ============================================================
//  MOTOR-FUNCTIE  (bevestigd: zoals in geteste code)
//    M1/M3/M5 = ROTATIE  van tube 1/2/3   (continu, geen eindstop)
//    M2/M4/M6 = TRANSLATIE van tube 1/2/3  (spindel, wordt gehomed)
// ============================================================
enum AxisType { AX_ROT, AX_TRA };

const AxisType AXIS_TYPE[NUM_MOTORS] = {
  AX_ROT,  // M1  T1-Rotatie
  AX_TRA,  // M2  T1-Translatie
  AX_ROT,  // M3  T2-Rotatie
  AX_TRA,  // M4  T2-Translatie
  AX_ROT,  // M5  T3-Rotatie
  AX_TRA,  // M6  T3-Translatie
};

const char* const MOTOR_NAMES[NUM_MOTORS] = {
  "T1-Rot", "T1-Tra", "T2-Rot", "T2-Tra", "T3-Rot", "T3-Tra"
};

// ============================================================
//  UART-ADRESSEN  (2 drivers per seriele bus)
//  Pin 6 van J1_M zet MS2:  M1/M3/M5 -> 0b00,  M2/M4/M6 -> 0b10
//  De seriele poort per motor wordt in MSCTR_CODE.ino gekoppeld.
// ============================================================
const uint8_t TMC_ADDR[NUM_MOTORS] = { 0b00, 0b10, 0b00, 0b10, 0b00, 0b10 };

// ============================================================
//  EINDSCHAKELAARS  (4 stuks; Schmitt-trigger INVERTEREND: HIGH = geraakt)
// ------------------------------------------------------------
//  Werkelijke bedrading (opgegeven door gebruiker):
//    pin 38  schakelaar OP T1     -> HOME van T1-translatie (M2)
//                                    + collision-bewaking T2->T1 (demo)
//    pin 39  eind van het pad     -> ver-eind VEILIGHEIDSSTOP T1 (M2)
//    pin 40  beginpunt T2         -> HOME van T2-translatie (M4)
//                                    + collision-bewaking T3->T2 (demo)
//    pin 42  beginpunt T3         -> HOME van T3-translatie (M6) (referentie)
//    pin 41, 43                   -> niet gebruikt
//  Tube-volgorde: T1 = dikste/buitenste ... T3 = dunste/binnenste.
//  Alle tubes starten de demo op hun gekalibreerde nulpunt.
// ============================================================
#define LIMIT_ACTIVE_HIGH 1     // 1 = HIGH betekent geraakt (Schmitt invert)
#define LIMIT_PIN_FIRST   38    // ingangen 38..43 worden als INPUT gezet
#define LIMIT_PIN_COUNT   6

// HOME-schakelaarpin per motor-index (-1 = homed niet op een schakelaar)
const int8_t HOME_SWITCH_PIN[NUM_MOTORS] = {
  -1,   // M1  T1-Rot
  38,   // M2  T1-Tra   home op schakelaar (gemonteerd op T1)
  -1,   // M3  T2-Rot
  40,   // M4  T2-Tra   home op beginpunt T2
  -1,   // M5  T3-Rot
  42,   // M6  T3-Tra   home op beginpunt T3 (referentie)
};

// Ver-eind VEILIGHEIDSSTOP per motor-index (-1 = geen). Trigger -> noodstop.
const int8_t FARSTOP_PIN[NUM_MOTORS] = {
  -1, 39, -1, -1, -1, -1,   // alleen T1-translatie (M2): pin 39
};

// Collision-bewaking: een home-schakelaar die tijdens bedrijf triggert terwijl
// de tube NIET thuis is -> de volgende (binnenste) tube botst ertegenaan.
const bool COLLISION_GUARD[NUM_MOTORS] = {
  false, true,  false, true,  false, false,
  //     M2:T2->T1    M4:T3->T2          M6: alleen referentie
};
#define COLLISION_HOME_TOL_MM  5.0f   // < deze afstand van nul = 'thuis' (geen alarm)

// Homing-richting per as (false = DIR LOW, true = DIR HIGH richting nulpunt).
// Flip deze als een as de verkeerde kant op homet.
//   idx: 0=M1(T1-Rot) 1=M2(T1-Tra) 2=M3(T2-Rot) 3=M4(T2-Tra) 4=M5(T3-Rot) 5=M6(T3-Tra)
const bool HOME_DIR[NUM_MOTORS] = {
  false, true, false, true, false, false
  //     ^M2          ^M4   omgedraaid: T1 en T2 homeden de verkeerde kant op
};

// ============================================================
//  INTERFACE-PINNEN  (uit netlist)
// ============================================================
#define ENC_CLK   46
#define ENC_DT    45
#define ENC_SW    44
#define ENC_R     49     // RGB rotary encoder, common cathode (LOW = aan)
#define ENC_G     48     // groen/blauw omgewisseld t.o.v. eerste mapping:
#define ENC_B     50     //   pin 48 = groen, pin 50 = blauw (op hardware gemeten)
#define BUZZER_PIN 47    // piezo via transistor Q1

// OLED (I2C op Mega: SDA=20, SCL=21)
#define OLED_WIDTH   128
#define OLED_HEIGHT  64
#define OLED_RESET   -1
#define OLED_ADDR    0x3C
#define OLED_TITLE_Y    2    // gele zone (titelbalk)
#define OLED_CONTENT_Y  18   // blauwe zone (inhoud)

// ============================================================
//  MECHANICA  -  positie wordt in VOLLE stappen bijgehouden,
//  zodat microstepping per beweging mag verschillen.
// ============================================================
#define FULLSTEPS_PER_REV   200      // 1.8 graden per volle stap
#define LEAD_MM             2.0f     // spoed spindel (mm per omwenteling)
#define TRAVEL_MM           200.0f   // maximale slag translatie

#define FULLSTEPS_PER_MM    ((float)FULLSTEPS_PER_REV / LEAD_MM)   // 100
#define FULLSTEPS_PER_DEG   ((float)FULLSTEPS_PER_REV / 360.0f)    // 0.5556

// ============================================================
//  DRAAIRICHTING
//  DIR-pinniveau voor POSITIEVE beweging:
//    translatie = weg van home (+mm),  rotatie = voorwaarts (+graden).
//  Homing beweegt omgekeerd (naar de home-schakelaar toe).
//  >>> Flip een waarde als die as de verkeerde kant op gaat. <<<
//    idx: 0=M1 1=M2(T1-Tra) 2=M3 3=M4(T2-Tra) 4=M5 5=M6(T3-Tra)
// ============================================================
const bool DIR_FWD_LEVEL[NUM_MOTORS] = { HIGH, LOW, HIGH, LOW, HIGH, HIGH };

// ============================================================
//  MICROSTEPPING per bewerking
//  Laag (2) = hogere snelheid mogelijk; hoog (16..256) = fijner/zachter
//  bij lage snelheid. Interpolatie naar 256 maakt ook lage MS zacht.
// ============================================================
#define MS_HOME    16     // homing
#define MS_TRA     16     // translatie demo
#define MS_ROT     16     // rotatie demo
#define MS_MANUAL  16     // handmatige seriele bewegingen
#define USE_INTERPOLATION 1   // interpoleer naar 256 microstappen (zachter)

// ============================================================
//  DRIVERMODUS per astype
//  SpreadCycle (true)  = sterker/vloeiender bij hogere snelheid
//  StealthChop (false) = stiller bij lage snelheid
// ============================================================
#define SPREAD_TRA   false    // StealthChop — stil
#define SPREAD_ROT   false    // StealthChop — stil
#define SPREAD_HOME  false    // StealthChop — stil

// ============================================================
//  MOTORSTROOM (RMS, mA)   >>> zet per motor-datasheet <<<
//  Rotatie: DINGS 17HS2048  |  Translatie: Nanotec LA561S20
// ============================================================
#define CURRENT_ROT_MA   1500
#define CURRENT_TRA_MA   1900

// ============================================================
//  SNELHEDEN
// ============================================================
#define HOME_SPEED_MMPS    4.0f     // homing — stil (120 RPM, diep onder resonantie)
#define DEMO_TRA_MMPS      4.0f     // translatie demo — stil
#define DEMO_ROT_RPM       10.0f    // rotatie demo — stil
#define MANUAL_TRA_MMPS    8.0f     // handmatige translatie
#define MANUAL_ROT_RPM     20.0f    // handmatige rotatie

// ============================================================
//  ACCELERATIE (trapezium-ramp, zoals in de testcode)
// ============================================================
#define ACCEL_MS          600UL    // ramp
#define START_SPEED_FSPS  50UL     // zacht opstarten (15 RPM)
#define MIN_PULSE_US      3UL      // ondergrens pulsinterval (microstap)

// ============================================================
//  DEMO
// ============================================================
#define DEMO_EXTEND_FRAC   1.00f   // translatie tot 100% van de slag
#define DEMO_ROT_REVS      1.0f    // omwentelingen per demostap

// ============================================================
//  RUNTIME-TYPEN  (in deze header zodat alle .ino-tabs ze kennen)
// ============================================================
enum CalibState { CAL_IDLE, CAL_FIND_ZERO, CAL_FIND_MAX, CAL_DONE, CAL_GO_HOME };
enum SystemMode { MODE_INIT, MODE_CALIB, MODE_MENU, MODE_DEMO, MODE_ERROR };

// Buzzer: melodie = reeks noten (freq in Hz, 0 = pauze; dur in ms)
struct Note { uint16_t freq; uint16_t dur; };
enum BuzzerSound { BUZ_CLICK, BUZ_BEEP, BUZ_OK, BUZ_BOOT, BUZ_ERROR, BUZ_HOME };

struct StepperMotor {
  bool          connected;          // driver gevonden op UART
  AxisType      type;               // AX_ROT of AX_TRA
  CalibState    state;
  long          position;           // huidige positie in (micro)stappen
  long          max_position;       // bovengrens, alleen TRA na homing
  long          target;             // doelpositie voor target-beweging
  bool          hasTarget;          // beweegt naar 'target'?
  bool          direction;          // huidige DIR-waarde
  bool          running;
  unsigned long stepInterval;       // us tussen stappen
  unsigned long lastStepTime;
  long          steps_since_start;
  int           stall_counter;
  unsigned long stall_time;
  unsigned long last_stall_event;
};

#endif // MSCTR_CONFIG_H
