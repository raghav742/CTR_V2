# Development log

## 2026-10-27 — Step 0.1: Project setup
**What:** Git repository created (github.com/raghav742/CTR_V2). Casper's V1 firmware committed unchanged and tagged `v1-baseline`.
**Why:** Every V2 change is traceable, and V1 can always be compared or restored.
**Test:** Repository pushed; tag visible on GitHub.

## 2026-10-27 — Step 0.2: Project structure
**What:** Casper's code moved to `reference_v1/` (read-only, not compiled). Empty `src/` for the new firmware. `docs/` added with devlog, decisions and pin map.
**Why:** The V1 motion code blocks the whole program, and that pattern runs through every file; a fresh structure is faster than converting it. Good parts (OLED screens, buzzer, driver setup) are reused from `reference_v1/` when needed.
**Test:** PlatformIO no longer compiles the V1 files.

## 2026-10-27 — Step 0.3: Hardware profile
**What:** `hw_types.h` (data types), `hw_profile_v1.h` (V1 values), `hw_profile_v2.h` (placeholder), `hw_profile.h` (selector). Two build environments, `v1` and `v2`, in `platformio.ini`. Test program prints the profile at startup.
**Why:** Program code never contains pin numbers; moving to V2 means filling in one file (decision D1).
**Test:** `v1` builds and prints the profile; `v2` stops with "V2 profile not defined yet".
**Result on robot (2026-10-28):** Profile printout correct. Encoder rotation and all limit switches report changes.
**Findings:**
- Encoder button is active HIGH and only works while pin 49 is HIGH. Pin 49 (PCB "ENC_R") actually feeds the encoder "+" (LED supply and button); the "+" and red wires are swapped. Red LED unavailable on V1. Profile updated (decision D2).
- With the internal pull-up enabled, the released button idles near 1 V (encoder pull-down vs. internal pull-up): likely cause of the "EMI on pin 44" in V1. Pull-up now disabled.
- At rest, only T3-home reads HIGH: T1 and T2 carriages rest just short of their switches (checked). Switch wiring and polarity correct.

## 2026-10-28 — Step 1.1: Non-blocking main loop and module structure
**What:** Modules `encoder`, `status_led`, `limit_switches`, `console`, `loop_stats`, each with `begin()` and `update()`; `main.cpp` only calls them. Serial console reads character by character (commands: help, status, inputs, watch, led). Firmware version in `include/version.h`.
**Why:** Nothing may wait, so inputs, safety and display keep running while motors move (Casper's code blocked everything during motion).
**Test:** `status` shows loops/s and the slowest loop pass; `watch on` prints encoder and switch events; `led` changes colour without stopping anything.
**Result on robot:** All results as expected. Encoder, limit switches everything prints changes.

## 2026-10-28 — Step 1.2: Timer-interrupt step generation
**What:** `drivers` (TMC2209 setup over UART, Casper's settings), `steppers` (Timer1 interrupt at 20 kHz drives all six STEP pins), `motion` (mm/degrees, acceleration ramps, move commands). Microstepping 8 for translation, 16 for rotation (driver interpolates to 256). New commands: drivers, move, stop, pos, en, dis. Temporary safety net: a switch becoming pressed during motion, an encoder press or `stop` halts all axes.
**Why:** Step pulses from an interrupt keep running while the main loop handles inputs, safety and display (decision D3).
**Simulation (simavr, all 6 axes at full speed):** first version used 85 % CPU in the interrupt; optimised (unrolled, 16-bit accumulators) to 45 %. Loop still ~3,900 passes/s under that worst case. All moves end at the exact target.
**Test on robot (2026-10-28):**
- All 6 drivers detected and configured. Rotation axes (StealthChop): correct angles at 30 and 60 rpm, smooth, all three together OK; encoder press stops instantly.
- Translation in StealthChop **slips** at 10, 15, 20 mm/s and also at 5 mm/s (counts drift; T2 lost about 16 mm, T3 about 5 mm). Diagnosed with runtime commands `chop`, `ms`, `cur` (v1.2.1): SpreadCycle fixes it, microstepping was not the cause. Translation now SpreadCycle by default (decision D4, v1.2.3).
- Repeatability T3 in SpreadCycle against home switch: reference -54.65 mm; after 20 mm/s cycles -54.67 / -54.67; return at 20 mm/s -54.82 (= ~0.17 mm stopping distance, about 8 ms reaction); slow 3 mm/s -54.57. No steps lost.
- Load with three translation axes at 20 mm/s: 9,016 loops/s, longest interrupt 29.5 us (tick = 50 us).
- Added temporary 50 mm cap per translation move until homing exists (a 1200 mm typo was accepted before).
**Open:** occasional slow loop pass (~4 ms) while printing to USB, to address with a larger serial buffer.
