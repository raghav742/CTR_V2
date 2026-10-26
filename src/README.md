# MSCTR — Besturingssoftware Concentric Tube Robot demonstrator

Firmware voor de **Arduino Mega 2560 + custom shield (MSCTR)** met 6× TMC2209,
OLED, RGB rotary encoder, buzzer en eindschakelaars. Gebaseerd op het eindverslag
(CTR-demonstrator) en de PCB-netlist `ArduinoShieldForCTR`.

## Bestandsstructuur (Arduino-tabs)

| Bestand          | Inhoud |
|------------------|--------|
| `MSCTR_CODE.ino` | hoofdbestand: globale objecten, `setup()`, `loop()` |
| `Config.h`       | **alle pinnen, motor-mapping en instelbare parameters** (pas hier aan) |
| `Drivers.ino`    | TMC2209 detectie + configuratie (StealthChop, CoolStep; StallGuard alleen uitleesbaar) |
| `Motion.ino`     | stapgeneratie, kalibratie/homing, demo, menu |
| `Interface.ino`  | OLED, encoder, RGB-led, buzzer |
| `Safety.ino`     | eindschakelaars, grensbewaking, noodstop/error, temperatuur |
| `SerialCmd.ino`  | seriële commando's (debug / handmatig testen) |

## Vereiste libraries (Library Manager)
- **TMCStepper** (teemuatlut)
- **Adafruit GFX** + **Adafruit SSD1306** (+ Adafruit BusIO)

Board: **Arduino Mega or Mega 2560** (`arduino:avr:mega`). Upload via USB.

## Toestandsmachine (Flowcharts 1–5 uit verslag)
`INIT → MENU → KALIBRATIE → DEMO`, met een ERROR-state die op elk moment kan
worden bereikt en met de encoder bevestigd wordt.

**Statusled:** groen = idle/klaar · rood = wacht/kalibratie/error · blauw = demo.

## Bediening
- **Encoder draaien** = door menu scrollen, **drukken** = kiezen.
- Menu: `Kalibreren` · `Start Demo` · `Status` · `Driver Scan`.
- Tijdens kalibratie of demo: **drukken = afbreken/stoppen**.

## Seriële commando's (115200 baud)
```
MENU  CALIB  DEMO  STOP  SCAN  STATUS
M<n> <waarde>   beweeg motor n   (mm voor translatie, ° voor rotatie)
                 bv. "M2 50"  = T1-Tra 50 mm vooruit
                     "M1 -90" = T1-Rot 90° terug
EN<n> / DIS<n>  motor in-/uitschakelen
```

## Motor-mapping (bevestigd, = geteste code)
| Motor | Functie       | STEP | DIR | EN | DIAG | UART     |
|-------|---------------|------|-----|----|------|----------|
| M1    | T1 Rotatie    | 3    | 4   | 5  | 6    | Serial1  |
| M2    | T1 Translatie | 8    | 9   | 10 | 11   | Serial1  |
| M3    | T2 Rotatie    | 22   | 23  | 24 | 25   | Serial2  |
| M4    | T2 Translatie | 26   | 27  | 28 | 29   | Serial2  |
| M5    | T3 Rotatie    | 30   | 31  | 32 | 33   | Serial3  |
| M6    | T3 Translatie | 34   | 35  | 36 | 37   | Serial3  |

Encoder: CLK 46 · DT 45 · SW 44 · R 49 · G 50 · B 48 · Buzzer 47 · OLED SDA 20 / SCL 21.
UART-adressen: M1/M3/M5 = `0b00`, M2/M4/M6 = `0b10`.

Rotatie-assen (M1/M3/M5) draaien continu en worden **niet** gehomed; alleen de
translatie-assen (M2/M4/M6) krijgen een kalibratie (nulpunt + vaste slag).

### Eindschakelaars (4 stuks — werkelijke bedrading)
| Pin | Locatie | Homing | Veiligheid (bedrijf) |
|-----|---------|--------|----------------------|
| 38  | op T1        | home T1-translatie (M2) | botsing T2→T1 (afgaan terwijl T1 niet thuis) |
| 39  | eind van pad | —                       | ver-eind veiligheidsstop T1 |
| 40  | beginpunt T2 | home T2-translatie (M4) | botsing T3→T2 |
| 42  | beginpunt T3 | home T3-translatie (M6) | referentie |

Pin 41 en 43 zijn niet gebruikt. Alle tubes starten de demo op hun gekalibreerde
nulpunt; daardoor betekent een home-schakelaar die afgaat terwijl de tube níet thuis
is, dat de binnenste tube ertegenaan botst → noodstop. Schmitt-trigger = inverterend
(`LIMIT_ACTIVE_HIGH 1`, HIGH = geraakt).

## ⚠️ Nog instellen vóór gebruik op hardware (`Config.h`)

1. **Eindschakelaars — al ingevuld** (`HOME_SWITCH_PIN`, `FARSTOP_PIN`,
   `COLLISION_GUARD`) volgens bovenstaande tabel. Controleer alleen of `HOME_DIR` per
   as naar de juiste schakelaar toe beweegt, en `COLLISION_HOME_TOL_MM` (nu 5 mm).
2. **Homing-richting — `HOME_DIR[]`.** Flip per as als die de verkeerde kant op homed.
3. **Motorstroom — `CURRENT_ROT_MA` / `CURRENT_TRA_MA`.** Zet volgens datasheet
   (rotatie = DINGS 17HS2048, translatie = Nanotec LA561S20).
4. **StallGuard — niet geïmplementeerd.** Homing draait volledig op de eindschakelaars.
   StallGuard is hardwarematig voorbereid (DIAG-pinnen bedraad, `SG_RESULT()` uitleesbaar via
   de driver-scan), maar wordt niet als homing- of noodstopcriterium gebruikt. Sensorloos
   homen op StallGuard is een mogelijke vervolgstap. CoolStep blijft wel actief.
5. **Snelheden / microstepping.** Demo-snelheden bewust gematigd voor 6 gelijktijdige
   assen op de Mega; los opvoeren voor CTQ-tests (20 mm/s translatie, 0,5° rotatie).
6. **OLED.** Code gebruikt SSD1306 (zoals in TAR getest). Bij beeld-verschuiving/ruis
   is het paneel mogelijk SH1106 → overstappen op U8g2 (al geïnstalleerd).

## Mechanica (uit verslag)
Translatie: spindel 2 mm/omw, 200 stappen/omw. Slag 200 mm. Rotatie: direct, 360°.
Resoluties bij default microstepping ruim binnen de eisen (0,25 mm / 0,5°).
