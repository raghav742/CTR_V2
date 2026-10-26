// ============================================================
//  Motion.ino  -  Blokkerende, geramde stapgeneratie (vloeiend)
// ------------------------------------------------------------
//  Bewegingen zijn GELIJKTIJDIG (alle motoren van één fase
//  bewegen tegelijk via runPhaseSimultaneous).
//  Positie wordt in VOLLE stappen bijgehouden (MS-onafhankelijk).
// ============================================================
#include "Config.h"

// Stopreden van de laatste beweging: 0=klaar, 1=gebruiker-stop, 2=veiligheid
int    g_stopReason = 0;
String g_safetyMsg  = "";

// Translatie- en rotatie-assen in tube-volgorde
const int TRA_ORDER[3] = { 1, 3, 5 };   // M2(T1), M4(T2), M6(T3)
const int ROT_IDX[3]   = { 0, 2, 4 };   // M1(T1), M3(T2), M5(T3)

// ------------------------------------------------------------
//  Eenheden <-> volle stappen
// ------------------------------------------------------------
long  mmToFullSteps(float mm) { return lround(mm * FULLSTEPS_PER_MM); }
long  degToFullSteps(float d) { return lround(d * FULLSTEPS_PER_DEG); }
float fullStepsToMM(long s)   { return s / FULLSTEPS_PER_MM; }
float fullStepsToDeg(long s)  { return s / FULLSTEPS_PER_DEG; }

// ------------------------------------------------------------
//  Lage-niveau
// ------------------------------------------------------------
void enableMotor(int i)  { if (motors[i].connected) digitalWrite(MOTOR_PINS[i][P_EN], LOW);  }
void disableMotor(int i) { if (motors[i].connected) digitalWrite(MOTOR_PINS[i][P_EN], HIGH); }

void applyDriver(int i, uint16_t ms, uint16_t current_ma, bool spread) {
  tmc[i].rms_current(current_ma);
  tmc[i].microsteps(ms);
#if USE_INTERPOLATION
  tmc[i].intpol(true);
#endif
  tmc[i].en_spreadCycle(spread);
  tmc[i].pwm_autoscale(true);
}

inline void pulseStep(uint8_t pin, unsigned long iv) {
  digitalWrite(pin, HIGH);
  delayMicroseconds(2);
  digitalWrite(pin, LOW);
  if (iv > 2) delayMicroseconds(iv - 2);
}

// ------------------------------------------------------------
//  Onderbreking: gebruiker-stop (encoder/serieel) of veiligheid
// ------------------------------------------------------------
bool encStop() {
  // Tijdgebaseerde debounce: pin moet >= 200 ms aaneengesloten LOW zijn.
  // Voorkomt valse stop door EMI van de motoren op pin 44.
  static unsigned long lowSince = 0;
  if (digitalRead(ENC_SW) == LOW) {
    if (lowSince == 0) lowSince = micros();
    if (micros() - lowSince > 200000UL) { lowSince = 0; return true; }
  } else {
    lowSince = 0;
  }
  return false;
}
bool serialStop() {
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim(); s.toUpperCase();
    if (s == "STOP") return true;
  }
  return false;
}
// 0 = door, 1 = gebruiker-stop (serieel STOP), 2 = veiligheid (ver-eindstop)
// encStop() is bewust NIET meegenomen: EMI van de motoren trekt pin 44
// omlaag en zou anders na ~200 ms elke beweging afbreken.
int motionCheck() {
  if (serialStop()) return 1;
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (FARSTOP_PIN[i] >= 0 && farStopHit(i)) {
      g_safetyMsg = String(MOTOR_NAMES[i]) + F(" limit hit");
      return 2;
    }
  }
  return 0;
}

// ------------------------------------------------------------
//  Trapezium-ramp; retourneert het aantal voltooide pulsen.
//  Zet g_stopReason als afgebroken.
// ------------------------------------------------------------
long rampPulse(int i, long nPulses, unsigned long cruiseIv, unsigned long startIv) {
  g_stopReason = 0;
  if (nPulses <= 0) return 0;
  if (startIv < cruiseIv) startIv = cruiseIv;

  unsigned long avg = (startIv + cruiseIv) / 2; if (avg < 1) avg = 1;
  long rampSteps = (long)((ACCEL_MS * 1000UL) / avg);
  if (rampSteps > nPulses / 2) rampSteps = nPulses / 2;
  if (rampSteps < 1) rampSteps = 1;
  long cruise = nPulses - 2 * rampSteps; if (cruise < 0) cruise = 0;

  const uint8_t pin = MOTOR_PINS[i][P_STEP];
  long done = 0;

  for (long s = 0; s < rampSteps; s++) {                 // op
    unsigned long iv = startIv - ((startIv - cruiseIv) * (unsigned long)s) / (unsigned long)rampSteps;
    if (iv < MIN_PULSE_US) iv = MIN_PULSE_US;
    pulseStep(pin, iv); done++;
    if ((s & 0xFF) == 0) { int r = motionCheck(); if (r) { g_stopReason = r; return done; } }
  }
  for (long s = 0; s < cruise; s++) {                    // cruise
    pulseStep(pin, cruiseIv); done++;
    if ((s & 0xFF) == 0) { int r = motionCheck(); if (r) { g_stopReason = r; return done; } }
  }
  for (long s = 0; s < rampSteps; s++) {                 // af
    unsigned long iv = cruiseIv + ((startIv - cruiseIv) * (unsigned long)s) / (unsigned long)rampSteps;
    if (iv < MIN_PULSE_US) iv = MIN_PULSE_US;
    pulseStep(pin, iv); done++;
    if ((s & 0xFF) == 0) { int r = motionCheck(); if (r) { g_stopReason = r; return done; } }
  }
  return done;
}

// ------------------------------------------------------------
//  Beweeg een as 'deltaFull' volle stappen (gesigned).
//  Werkt de positie bij (ook bij vroegtijdige stop).
//  true = volledig voltooid.
// ------------------------------------------------------------
bool moveAxisFull(int i, long deltaFull, float fullPerSec, uint16_t ms, uint16_t cur, bool spread) {
  if (deltaFull == 0) return true;
  bool forward = (deltaFull > 0);
  applyDriver(i, ms, cur, spread);
  digitalWrite(MOTOR_PINS[i][P_DIR], forward ? DIR_FWD_LEVEL[i] : !DIR_FWD_LEVEL[i]);
  enableMotor(i);
  delay(3);                                              // DIR laten settelen

  long pulses = labs(deltaFull) * (long)ms;
  if (fullPerSec < 1) fullPerSec = 1;
  unsigned long cruiseIv = (unsigned long)(1000000.0f / (fullPerSec * ms));
  if (cruiseIv < MIN_PULSE_US) cruiseIv = MIN_PULSE_US;
  unsigned long startIv = (unsigned long)(1000000.0f / ((float)START_SPEED_FSPS * ms));

  long done = rampPulse(i, pulses, cruiseIv, startIv);
  long fullDone = done / (long)ms;
  motors[i].position += forward ? fullDone : -fullDone;
  return (done == pulses);
}

// Demo-/handmatige wrappers in fysieke eenheden
bool moveTubeMM(int i, float mm, float mmps, uint16_t ms) {
  return moveAxisFull(i, mmToFullSteps(mm), mmps * FULLSTEPS_PER_MM, ms, CURRENT_TRA_MA, SPREAD_TRA);
}
bool rotateDeg(int i, float deg, float rpm, uint16_t ms) {
  float fullPerSec = (rpm / 60.0f) * FULLSTEPS_PER_REV;
  return moveAxisFull(i, degToFullSteps(deg), fullPerSec, ms, CURRENT_ROT_MA, SPREAD_ROT);
}

// ------------------------------------------------------------
//  Homing ALLE 3 translatie-assen TEGELIJK (micros-gebaseerde polling-lus)
//  Elke as krijgt een eigen timer; ze bewegen onafhankelijk en stoppen
//  individueel zodra hun home-schakelaar aanslaat.
// ------------------------------------------------------------
bool homeAllAxesSimultaneous() {
  g_stopReason = 0;

  unsigned long cruiseIv = (unsigned long)(1000000.0f / (HOME_SPEED_MMPS * FULLSTEPS_PER_MM * MS_HOME));
  if (cruiseIv < MIN_PULSE_US) cruiseIv = MIN_PULSE_US;
  unsigned long startIv  = (unsigned long)(1000000.0f / ((float)START_SPEED_FSPS * MS_HOME));
  if (startIv < cruiseIv) startIv = cruiseIv;
  long rampPulses = (long)((ACCEL_MS * 1000UL) / ((startIv + cruiseIv) / 2));
  if (rampPulses < 1) rampPulses = 1;
  long maxPulses = (long)(TRAVEL_MM * 1.5f * FULLSTEPS_PER_MM) * (long)MS_HOME;

  // Per-as toestand
  bool          active[3]    = { false, false, false };
  unsigned long lastPulse[3] = { 0, 0, 0 };
  long          pulsesDone[3]= { 0, 0, 0 };

  // Alle verbonden translatie-assen klaarzetten
  for (int t = 0; t < 3; t++) {
    int i = TRA_ORDER[t];
    if (!motors[i].connected || HOME_SWITCH_PIN[i] < 0) {
      motors[i].state = CAL_DONE;  // geen schakelaar -> meteen klaar
      continue;
    }
    if (homeSwitchHit(i)) {
      // Staat al op de schakelaar -> meteen nulpunt
      motors[i].position = 0; motors[i].max_position = mmToFullSteps(TRAVEL_MM);
      motors[i].state = CAL_DONE;
      Serial.print(F("[CALIB] ")); Serial.print(MOTOR_NAMES[i]); Serial.println(F(" al op nulpunt"));
      continue;
    }
    applyDriver(i, MS_HOME, CURRENT_TRA_MA, SPREAD_HOME);
    digitalWrite(MOTOR_PINS[i][P_DIR], !DIR_FWD_LEVEL[i]);  // richting home
    enableMotor(i);
    motors[i].state = CAL_FIND_ZERO;
    active[t] = true;
  }
  delay(5);   // DIR laten settelen

  unsigned long checkTimer = micros();

  while (true) {
    bool anyActive = false;
    unsigned long now = micros();

    for (int t = 0; t < 3; t++) {
      if (!active[t]) continue;
      anyActive = true;
      int i = TRA_ORDER[t];

      // Ramp-interval berekenen
      long p = pulsesDone[t];
      unsigned long iv = (p < rampPulses)
          ? startIv - ((startIv - cruiseIv) * (unsigned long)p) / (unsigned long)rampPulses
          : cruiseIv;
      if (iv < cruiseIv) iv = cruiseIv;

      // Puls op tijd
      if (now - lastPulse[t] >= iv) {
        digitalWrite(MOTOR_PINS[i][P_STEP], HIGH);
        delayMicroseconds(2);
        digitalWrite(MOTOR_PINS[i][P_STEP], LOW);
        lastPulse[t] = micros();
        pulsesDone[t]++;

        // Time-out
        if (pulsesDone[t] > maxPulses) {
          triggerError(String(MOTOR_NAMES[i]) + F(" homing timeout"));
          return false;
        }

        // Home-schakelaar checken elke 8 pulsen.
        // GEEN blokkerende beep()/updateOLED() hier: die zouden de andere
        // assen ~85 ms laten stilstaan -> stall + lawaai. Alleen stoppen.
        if ((pulsesDone[t] & 0x07) == 0 && homeSwitchHit(i)) {
          motors[i].position     = 0;
          motors[i].max_position = mmToFullSteps(TRAVEL_MM);
          motors[i].state        = CAL_DONE;
          active[t]              = false;
          Serial.print(F("[CALIB] ")); Serial.print(MOTOR_NAMES[i]); Serial.println(F(" nulpunt"));
        }
      }
    }

    if (!anyActive) break;

    // Gebruiker/veiligheid checken elke ~50 ms
    if (micros() - checkTimer > 50000UL) {
      checkTimer = micros();
      int r = motionCheck();
      if (r == 1) {
        g_stopReason = 1;
        Serial.println(F("[CALIB] afgebroken"));
        for (int t = 0; t < 3; t++) if (active[t]) motors[TRA_ORDER[t]].state = CAL_IDLE;
        return false;
      }
      if (r == 2) { g_stopReason = 2; return false; }
    }
  }
  return true;
}

// ------------------------------------------------------------
//  Kalibratie / Home / Demo  (blokkerend, vanuit het menu)
// ------------------------------------------------------------
bool allCalibrated() {
  for (int i = 0; i < NUM_MOTORS; i++)
    if (motors[i].connected && motors[i].type == AX_TRA && motors[i].state != CAL_DONE)
      return false;
  return true;
}

void runCalibration() {
  stopBuzzer();
  systemMode = MODE_CALIB;
  rgbWait();
  for (int t = 0; t < 3; t++) if (motors[TRA_ORDER[t]].connected) motors[TRA_ORDER[t]].state = CAL_IDLE;
  for (int t = 0; t < 3; t++) if (motors[ROT_IDX[t]].connected)   motors[ROT_IDX[t]].state   = CAL_DONE;
  updateOLED();
  Serial.println(F("[CALIB] gestart — alle assen tegelijk"));

  if (!homeAllAxesSimultaneous()) {
    if (g_stopReason == 2) triggerError(g_safetyMsg);
    waitButtonRelease();
    systemMode = (g_stopReason == 1) ? MODE_MENU : systemMode;
    if (g_stopReason == 1) { rgbIdle(); updateOLED(); }
    return;
  }

  // Toon kort welke tubes nu gekalibreerd zijn (nog in MODE_CALIB -> vinkjes)
  updateOLED();
  beep(1568, 150);
  Serial.println(F("[CALIB] klaar"));
  delay(1300);

  systemMode = MODE_MENU; rgbIdle();
  waitButtonRelease();
  updateOLED();
}

// Stop-check voor demo (blokkerend).
// Pin 44 wordt gecontroleerd elke 1000 ms (zie demo-lus).
// Extra beveiliging: pin moet >= 200 ms aaneengesloten LOW zijn.
// Samen: 1000 ms + 200 ms = min. 1200 ms drukken om te stoppen.
// EMI-glitches zijn < 10 ms en worden volledig genegeerd.
bool demoAbort() {
  if (serialStop()) return true;
  // ENC_SW is active-HIGH: drukken = HIGH, loslaten = LOW.
  // 200 ms debounce filtert EMI-spikes (< 10 ms) eruit.
  static unsigned long highSince = 0;
  if (digitalRead(ENC_SW) == HIGH) {
    if (highSince == 0) highSince = millis();
    if (millis() - highSince >= 200) { highSince = 0; return true; }
  } else {
    highSince = 0;
  }
  return false;
}

// ============================================================
//  GELIJKTIJDIGE MEERFASE-BEWEGING
// ------------------------------------------------------------
//  Beweegt tot 6 motoren tegelijk met één gedeelde trapeziumramp.
//  mIdx[count]   : motorindices
//  targetFS[count]: doelpositie in VOLLE stappen (absoluut)
//  ms, cur, spread, fullPerSec: rijparameters (zelfde voor alle assen)
//  Retourneert false bij afbreken (g_stopReason is dan gezet).
// ============================================================
bool runPhaseSimultaneous(int* mIdx, int count, long* targetFS,
                           uint16_t ms, uint16_t cur, bool spread, float fullPerSec) {
  g_stopReason = 0;

  long pulsesNeeded[6] = {};
  bool fwd[6]          = {};
  bool active[6]       = {};
  long done[6]         = {};
  long maxPulses       = 0;

  for (int k = 0; k < count; k++) {
    int i = mIdx[k];
    if (!motors[i].connected) continue;
    long delta      = targetFS[k] - motors[i].position;
    fwd[k]          = (delta >= 0);
    pulsesNeeded[k] = labs(delta) * (long)ms;
    active[k]       = (pulsesNeeded[k] > 0);
    if (pulsesNeeded[k] > maxPulses) maxPulses = pulsesNeeded[k];
    applyDriver(i, ms, cur, spread);
    digitalWrite(MOTOR_PINS[i][P_DIR], fwd[k] ? DIR_FWD_LEVEL[i] : !DIR_FWD_LEVEL[i]);
    if (active[k]) enableMotor(i);
  }
  if (maxPulses == 0) return true;
  delay(5);

  unsigned long cruiseIv = (unsigned long)(1000000.0f / (fullPerSec * (float)ms));
  if (cruiseIv < MIN_PULSE_US) cruiseIv = MIN_PULSE_US;
  unsigned long startIv  = (unsigned long)(1000000.0f / ((float)START_SPEED_FSPS * (float)ms));
  if (startIv < cruiseIv) startIv = cruiseIv;
  long rampPulses = (long)((ACCEL_MS * 1000UL) / ((startIv + cruiseIv) / 2));
  if (rampPulses > maxPulses / 2) rampPulses = maxPulses / 2;
  if (rampPulses < 1) rampPulses = 1;

  long          tick        = 0;
  unsigned long lastTick    = micros();
  unsigned long checkTimer  = micros();

  while (true) {
    bool any = false;
    for (int k = 0; k < count; k++) if (active[k]) { any = true; break; }
    if (!any) break;

    unsigned long now = micros();

    // Ramp omhoog / cruise / ramp omlaag op basis van de langste as
    unsigned long iv;
    long rampDown = maxPulses - rampPulses - tick;
    if      (tick < rampPulses)
      iv = startIv - ((startIv - cruiseIv) * (unsigned long)tick)      / (unsigned long)rampPulses;
    else if (rampDown <= 0)
      iv = cruiseIv + ((startIv - cruiseIv) * (unsigned long)(-rampDown)) / (unsigned long)rampPulses;
    else
      iv = cruiseIv;
    if (iv < cruiseIv) iv = cruiseIv;
    if (iv > startIv)  iv = startIv;

    if (now - lastTick >= iv) {
      lastTick = micros();
      tick++;
      for (int k = 0; k < count; k++) {
        if (!active[k]) continue;
        uint8_t pin = MOTOR_PINS[mIdx[k]][P_STEP];
        digitalWrite(pin, HIGH); delayMicroseconds(2); digitalWrite(pin, LOW);
        done[k]++;
        if (done[k] >= pulsesNeeded[k]) active[k] = false;
      }
    }

    // Check elke ~50 ms. Tijdens beweging ALLEEN serieel STOP: de encoderlijn
    // (pin 44) pikt motor-EMI op en zou anders vals afbreken. Encoder-stop wordt
    // tussen de fases gecontroleerd, wanneer de motoren stilstaan.
    if (micros() - checkTimer > 50000UL) {
      checkTimer = micros();
      if (serialStop()) { g_stopReason = 1; break; }
      for (int i = 0; i < NUM_MOTORS; i++) {
        if (FARSTOP_PIN[i] >= 0 && farStopHit(i)) {
          g_safetyMsg = String(MOTOR_NAMES[i]) + F(" limit hit");
          g_stopReason = 2; break;
        }
      }
      if (g_stopReason) break;
    }
  }

  // Posities bijwerken op basis van werkelijk gezette stappen
  for (int k = 0; k < count; k++) {
    if (motors[mIdx[k]].connected)
      motors[mIdx[k]].position += fwd[k] ? (done[k]/(long)ms) : -(done[k]/(long)ms);
  }
  return (g_stopReason == 0);
}

// ============================================================
//  DEMO  —  continue bouncing loop (alle 6 assen tegelijk)
// ------------------------------------------------------------
//  TRA-assen: bewegen heen en weer. Bij raken van een eindschakelaar
//  (homeSwitchHit of farStopHit) OF bij het bereiken van de positielimiet
//  keert de as zelfstandig om. Zo loopt de demo eindeloos door.
//  ROT-assen: draaien continu in één richting.
//  Stoppen: encoder 25 ms indrukken, of serieel STOP sturen.
// ============================================================
void runDemo() {
  stopBuzzer();
  if (!allCalibrated()) {
    oledMessage("NOT READY", "Calibration", "required first", nullptr);
    beep(400, 250); delay(1100);
    systemMode = MODE_MENU; updateOLED();
    return;
  }
  systemMode = MODE_DEMO; rgbDemo();
  demoStep = 1;
  updateOLED();
  waitButtonRelease();
  g_stopFlag    = false;
  g_demoStartMs = millis();   // start opstart-grace (500 ms EMI-venster)
  Serial.println(F("[DEMO] gestart — druk encoder (>200 ms) of stuur STOP"));

  // --- Eigen beweging per tube ("Deep Tide / Mid Bridge / Tip Flicker") ---
  //  Elke translatie-as krijgt een EIGEN slagband [lo..hi] (% van de slag) en een
  //  eigen snelheid. Doordat de drie buizen verschillende amplitudes, gebieden en
  //  ritmes hebben, bewegen ze zichtbaar onafhankelijk en bouwen ze samen één
  //  continu veranderende 3D-vorm op — precies de kracht van het CTR-concept.
  //    T1 (buitenste): volledige, langzame diepe slag    (0..85%, traag)
  //    T2 (midden):    middenband, raakt nooit thuis/eind (30..75%, gemiddeld)
  //    T3 (binnenste): korte, snelle tip-flikkering hoog  (60..93%, snel)
  //  De korte band van T3 zorgt dat hij ~5x vaker heen-en-weer gaat dan T1, dus
  //  hij oogt veel sneller zonder de lineaire actuator te overbelasten (stall).
  const float TRA_LO_PCT[3]       = {  0.0f, 30.0f, 60.0f };  // ondergrens slagband
  const float TRA_HI_PCT[3]       = { 85.0f, 75.0f, 93.0f };  // bovengrens slagband
  const float TRA_SPEED_FACTOR[3] = { 0.80f, 1.30f, 1.60f };  // translatie-snelheid
  const float ROT_SPEED_FACTOR[3] = { 0.80f, 1.30f, 2.20f };  // rotatie-snelheid

  // --- Front-/botsschakelaar per buis (gecheckt tijdens UITSCHUIVEN) ---
  //  Elke buis kan bij het uitschuiven tegen de vorige (buitenliggende) buis
  //  botsen. De schakelaar die dat afvangt:
  //    T1 -> pin 39  (eigen eind-veiligheidsstop)
  //    T2 -> pin 38  (zit op T1; T2 dreigt tegen T1 te botsen)
  //    T3 -> pin 40  (zit op T2; T3 dreigt tegen T2 te botsen)
  //  pin 38/40 zijn TEVENS de home-schakelaar van de vorige buis, dus die
  //  tellen alleen als botsing wanneer die vorige buis niet zelf thuis staat.
  const int  FRONT_SW[3]  = { FARSTOP_PIN[TRA_ORDER[0]],       // 39
                              HOME_SWITCH_PIN[TRA_ORDER[0]],   // 38 (home van T1)
                              HOME_SWITCH_PIN[TRA_ORDER[1]] }; // 40 (home van T2)
  const long collisionTol = (long)(COLLISION_HOME_TOL_MM * FULLSTEPS_PER_MM) * (long)MS_TRA;

  // --- Basisintervallen ---
  float traIvBase = 1000000.0f / (DEMO_TRA_MMPS * FULLSTEPS_PER_MM * (float)MS_TRA);
  float rotIvBase = 1000000.0f / ((DEMO_ROT_RPM / 60.0f) * (float)FULLSTEPS_PER_REV * (float)MS_ROT);

  // --- Per-tube intervallen (sneller = korter interval = delen door factor) ---
  unsigned long traIv[3], rotIv[3];
  for (int t = 0; t < 3; t++) {
    traIv[t] = (unsigned long)(traIvBase / TRA_SPEED_FACTOR[t]);
    rotIv[t] = (unsigned long)(rotIvBase / ROT_SPEED_FACTOR[t]);
    if (traIv[t] < MIN_PULSE_US) traIv[t] = MIN_PULSE_US;
    if (rotIv[t] < MIN_PULSE_US) rotIv[t] = MIN_PULSE_US;
  }

  // --- TRA toestand (µstap-positie + eigen slagband per as) ---
  long traPos[3], traLo[3], traHi[3];
  bool traFwd[3];   // true = uitschuiven (weg van home)
  bool traOn[3];    // as actief (verbonden + gekalibreerd)

  for (int t = 0; t < 3; t++) {
    int i = TRA_ORDER[t];
    if (!motors[i].connected || motors[i].state != CAL_DONE) { traOn[t] = false; continue; }
    traOn[t] = true;
    long stroke = (long)(DEMO_EXTEND_FRAC * (float)motors[i].max_position) * (long)MS_TRA;
    traLo[t] = (long)(TRA_LO_PCT[t] / 100.0f * (float)stroke);
    traHi[t] = (long)(TRA_HI_PCT[t] / 100.0f * (float)stroke);
    traPos[t] = motors[i].position * (long)MS_TRA;   // begin vanuit huidige positie (0 na kalibratie)
    traFwd[t] = (traPos[t] < traHi[t]);              // eerste beweging = uitschuiven

    applyDriver(i, MS_TRA, CURRENT_TRA_MA, SPREAD_TRA);
    digitalWrite(MOTOR_PINS[i][P_DIR], traFwd[t] ? DIR_FWD_LEVEL[i] : !DIR_FWD_LEVEL[i]);
    enableMotor(i);
  }

  // --- ROT instellen ---
  for (int t = 0; t < 3; t++) {
    int i = ROT_IDX[t];
    if (!motors[i].connected) continue;
    applyDriver(i, MS_ROT, CURRENT_ROT_MA, SPREAD_ROT);
    digitalWrite(MOTOR_PINS[i][P_DIR], DIR_FWD_LEVEL[i]);
    enableMotor(i);
  }
  delay(5);

  // --- Timers per as ---
  unsigned long lastTra[3] = {}, lastRot[3] = {};
  unsigned long checkTimer  = micros();
  g_stopReason = 0;

  while (true) {
    unsigned long now = micros();

    // --- TRA: heen-en-weer ---
    for (int t = 0; t < 3; t++) {
      int i = TRA_ORDER[t];
      if (!traOn[t]) continue;
      if (now - lastTra[t] < traIv[t]) continue;
      lastTra[t] = micros();

      // --- Botsing-/eindschakelaar VOOR deze buis (alleen bij uitschuiven) ---
      bool frontHit = false;
      int fpin = FRONT_SW[t];
      if (fpin >= 0 && switchPressed(fpin)) {
        if (t == 0)          frontHit = true;                         // eigen eindstop T1
        else if (traOn[t-1]) frontHit = (traPos[t-1] > collisionTol); // vorige buis niet thuis
      }

      // Keer om: bij uitschuiven op front-schakelaar/bovengrens,
      //          bij intrekken op home-schakelaar/ondergrens.
      bool hitFar  = traFwd[t]  && (frontHit         || traPos[t] >= traHi[t]);
      bool hitHome = !traFwd[t] && (homeSwitchHit(i) || traPos[t] <= traLo[t]);
      if (hitFar || hitHome) {
        traFwd[t] = !traFwd[t];
        digitalWrite(MOTOR_PINS[i][P_DIR], traFwd[t] ? DIR_FWD_LEVEL[i] : !DIR_FWD_LEVEL[i]);
        Serial.print(F("[DEMO] "));
        Serial.print(MOTOR_NAMES[i]);
        if (frontHit) Serial.print(F(" [schakelaar voor]"));
        Serial.println(traFwd[t] ? F(" keert: uitschuiven") : F(" keert: terugtrekken"));
        continue;   // geen puls deze tick; eerst richting settelen
      }

      uint8_t pin = MOTOR_PINS[i][P_STEP];
      digitalWrite(pin, HIGH); delayMicroseconds(2); digitalWrite(pin, LOW);
      traPos[t] += traFwd[t] ? 1 : -1;
    }

    // --- ROT: continu ---
    for (int t = 0; t < 3; t++) {
      int i = ROT_IDX[t];
      if (!motors[i].connected) continue;
      if (now - lastRot[t] < rotIv[t]) continue;
      lastRot[t] = micros();
      uint8_t pin = MOTOR_PINS[i][P_STEP];
      digitalWrite(pin, HIGH); delayMicroseconds(2); digitalWrite(pin, LOW);
    }

    // --- Stop-check elke 1000 ms ---
    // 1000 ms interval = natuurlijke grace-period na motorstart (EMI piek).
    // demoAbort() vereist ook >= 200 ms aaneengesloten LOW -> geen valse stops.
    if (micros() - checkTimer > 1000000UL) {
      checkTimer = micros();
      if (demoAbort()) { g_stopReason = 1; break; }
    }
  }

  // Positie terugschrijven
  for (int t = 0; t < 3; t++) {
    int i = TRA_ORDER[t];
    if (traOn[t]) motors[i].position = traPos[t] / (long)MS_TRA;
  }

  systemMode = MODE_MENU; rgbIdle();
  beep(784, 120);
  Serial.println(F("[DEMO] gestopt"));
  waitButtonRelease();
  updateOLED();
}

// ------------------------------------------------------------
//  MENU
// ------------------------------------------------------------
void handleMenu() {
  if (systemMode != MODE_MENU) return;

  if (encDelta != 0) {
    menuIndex = constrain(menuIndex + encDelta, 0, (int)MENU_COUNT - 1);
    encDelta  = 0;
    updateOLED();
  }
  if (swEdge) {
    swEdge = false;
    Serial.print(F("[MENU] -> ")); Serial.println(MENU_ITEMS[menuIndex]);
    switch (menuIndex) {
      case 0: runCalibration(); break;
      case 1: runDemo();        break;
      case 2: printStatus();    break;
      case 3: scanDrivers();    break;
    }
    updateOLED();
  }
}

// ------------------------------------------------------------
//  INIT motorstructuren (na driverdetectie)
// ------------------------------------------------------------
void initMotors() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].type         = AXIS_TYPE[i];
    motors[i].position     = 0;
    motors[i].max_position = mmToFullSteps(TRAVEL_MM);
    motors[i].state        = (motors[i].type == AX_TRA) ? CAL_IDLE : CAL_DONE;
  }
}
