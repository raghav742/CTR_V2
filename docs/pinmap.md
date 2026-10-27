# Pin map

Source of truth: `include/hw_profile_v1.h` (V1) and `include/hw_profile_v2.h` (V2).
V1 values verified against the PCB schematic (Casper's report, Bijlage A) and his tested code.

## Motors (TMC2209 via MSCTR shield)

| Axis | Name | Function | STEP | DIR | EN | DIAG | UART | Addr | Current (mA RMS) | DIR inverted |
|---|---|---|---|---|---|---|---|---|---|---|
| 0 | T1-Rot | Tube 1 rotation | 3 | 4 | 5 | 6 | Serial1 (18/19) | 0b00 | 1500 | no |
| 1 | T1-Tra | Tube 1 translation | 8 | 9 | 10 | 11 | Serial1 (18/19) | 0b10 | 1900 | yes |
| 2 | T2-Rot | Tube 2 rotation | 22 | 23 | 24 | 25 | Serial2 (16/17) | 0b00 | 1500 | no |
| 3 | T2-Tra | Tube 2 translation | 26 | 27 | 28 | 29 | Serial2 (16/17) | 0b10 | 1900 | yes |
| 4 | T3-Rot | Tube 3 rotation | 30 | 31 | 32 | 33 | Serial3 (14/15) | 0b00 | 1500 | no |
| 5 | T3-Tra | Tube 3 translation | 34 | 35 | 36 | 37 | Serial3 (14/15) | 0b10 | 1900 | no |

## Limit switches (RC filter + inverting Schmitt trigger: pressed = HIGH)

| Pin | Name | Role | Axis |
|---|---|---|---|
| 38 | T1-home | Home of T1-Tra; also detects T2 hitting T1 | 1 |
| 39 | T1-far | Far-end safety stop of T1-Tra | 1 |
| 40 | T2-home | Home of T2-Tra; also detects T3 hitting T2 | 3 |
| 41 | — | Wired on PCB, no switch on V1 | — |
| 42 | T3-home | Home of T3-Tra | 5 |
| 43 | — | Wired on PCB, no switch on V1 | — |

## Interface

| Pin | Function | Note |
|---|---|---|
| 20 / 21 | OLED SDA / SCL (I2C, 0x3C) | 4.7 kΩ pull-ups on PCB |
| 44 | Encoder push button | Polarity to verify (step 1.1) |
| 45 | Encoder DT (B) | Not an interrupt pin |
| 46 | Encoder CLK (A) | Not an interrupt pin |
| 47 | Buzzer (via 2N2222A) | 24 V piezo |
| 48 | RGB LED green | LOW = on; report says blue |
| 49 | RGB LED red | LOW = on |
| 50 | RGB LED blue | LOW = on; also SPI MISO, so no SPI reads |

## Free pins

2 (interrupt, reserved for E-stop on V2), 7, 12, 13, 41*, 43*, 51, 52, 53, A0–A15.
\* PCB switch inputs without a switch on V1.

## V2 additions (planned, see firmware plan)

| Pin | Function |
|---|---|
| 51 / 52 / 53 | TFT MOSI / SCK / CS (hardware SPI) |
| 12 / 13 | TFT D/C / reset |
| 7 | TFT backlight (PWM) |
| A2–A5 | Resistive touch |
| A0/A1 or 4 digital inputs | APEM joystick (type to confirm) |
| 2 | Emergency-stop input (if fitted) |
