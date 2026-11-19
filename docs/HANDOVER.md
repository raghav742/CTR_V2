# CTR V2 — Handover

Firmware and laptop software for the Concentric Tube Robot (CTR) V2 demonstrator,
Benchmark Electronics Almelo. Written by Raghavendran Srinivasan (internship, Oct–Nov 2026).

This document is for whoever continues the project: what exists, what was tested, how to
build and run it, and how to bring up the V2 robot step by step.

---

## 1. Status at the end of the internship

The firmware was developed and tested on the **V1 robot** (Casper de Ruiter's demonstrator,
hardware unchanged). The **V2 robot was not built**: its design and BoM are finished, but the
components and the enclosure were delayed. At the very end of the internship the V1 robot was
no longer available either.

- **Last version tested on a robot: firmware 4.3.1** (V1, phases 0–4). Git tag
  `v4.3.1-tested` (see section 3).
- **Firmware 5.0.0** adds the phase 5 preparation. It compiles for V1 and V2 and was reviewed
  and checked off-robot, but **has not run on any robot**. It also touches code shared with
  V1 (the motion ramp for the speed mode, the status LED), so treat 5.0.0 as untested on V1 too
  until test V1-1 / B7 has passed.

| Part | Status |
|---|---|
| Phase 0–1: structure, step generation, coordinated moves, states, safety, watchdog, serial protocol | Done, tested on V1 |
| Phase 2: homing, travel limits, limit calibration, rotation zero, parking, EEPROM | Done, tested on V1 (calibration works but is switched off on V1 because V1 slips) |
| Phase 3: OLED menu, jog, demos, attract mode, record & replay, visitor mode, safety demo | Done, tested on V1 |
| Phase 4: laptop control centre, heartbeat, CSV recording | Done, tested on V1 |
| V1 restrictions (10 mm/s, 100 rpm, 40 % acceleration, calib off, extra margins) | In `hw_profile_v1.h` only |
| Phase 5: speed mode (joystick motion) | Written, **untested on a robot**; testable on V1 or V2 with the `jog` command (test V1-1 below) |
| Phase 5: V2 hardware profile, TFT display, joystick | **Prepared, compiles, UNTESTED on hardware** |
| Phase 6: StallGuard jam detection, 3D view on the laptop | Not started — see Recommendations |

Firmware **5.0.0** (last robot-tested: 4.3.1), control centre **4.4.0**. Everything untested
is marked `UNTESTED` or `TODO(V2)` in the code.

**If 5.0.0 misbehaves on a robot:** go back to tag `v4.3.1-tested` for V1, or compare with it
(`git diff v4.3.1-tested -- src include`) to find the cause; the changes are listed in
`devlog.md` ("Phase 5 prepared without hardware").

---

## 2. What is where

```
CTRV2.0/
├─ platformio.ini        build environments v1 and v2
├─ include/              hardware profiles (hw_profile_v1.h, hw_profile_v2.h), motion settings, version
├─ src/                  firmware modules (one .h/.cpp pair per job)
├─ laptop/               control centre (Python), see laptop/README.md
├─ reference_v1/         Casper's original V1 firmware, untouched (git tag v1-baseline)
└─ docs/
   ├─ HANDOVER.md        this file
   ├─ USER_MANUAL.md     how to operate the robot
   ├─ architecture.md    how the firmware works and why (start here for the code)
   ├─ decisions.md       design decisions D1–D17 with alternatives
   ├─ devlog.md          every step: what, why, test results on the robot
   ├─ protocol.md        serial protocol for programs
   └─ pinmap.md          all pins, V1 and V2, and the V2 wiring corrections
```

**Key idea of the code:** nothing may wait. Every module has `begin()` (once) and `update()`
(every loop pass, returns quickly). Step pulses come from a 20 kHz timer interrupt; every move
passes the safety manager. Everything that differs between robots is in the hardware profile.
Read `architecture.md` sections 1–3 before changing anything.

---

## 3. Getting started

### Tools
- **VS Code + PlatformIO** extension (builds and uploads the firmware; libraries are fetched
  automatically from `platformio.ini`).
- **Python 3.10+** for the control centre (`laptop/README.md`).
- Git. The repository: `github.com/raghav742/CTR_V2`. Tags: `v1-baseline` (Casper's original),
  `v4.3.1-tested` (last version tested on the V1 robot), `v5.0.0-handover` (end of internship).

### Build and upload
- V1 robot: environment **`v1`** (PlatformIO: *env:v1 → Upload*, or `pio run -e v1 -t upload`).
- V2 robot: environment **`v2`** (`pio run -e v2 -t upload`).
- **Close the serial monitor before uploading** (only one program can use the port). If the
  upload fails with an avrdude write error: close every serial program, unplug and replug USB.

### Talk to the robot
- PlatformIO serial monitor: 250000 baud (set in `platformio.ini`). Type `help`.
- Useful first commands: `status`, `info`, `inputs`, `drivers`, `limits`, `pos`.
- Or the control centre: `laptop/run_control_centre.bat`. It has a **Fake robot** option to
  try it without hardware.

---

## 4. First test of firmware 5.0.0 (on V1 if available, otherwise at V2 bring-up B7)

Start with a quick regression check that 5.0.0 behaves like 4.3.1: `home`, a demo, a knob STOP,
`status` (status LED colours as in `USER_MANUAL.md`).

**V1-1 — speed mode** (the motion the V2 joystick will use). After `home`
(speeds below are for V1; on V2 they double):

| Step | Command | Pass if |
|---|---|---|
| 1 | `jog 2 + 50` | T1 moves forward smoothly at about 5 mm/s (50 % of the V1 cap) |
| 2 | `jog 2 100` while it moves | speeds up smoothly to about 10 mm/s |
| 3 | `jog 2 20` | slows down smoothly (no jerk) to about 2 mm/s |
| 4 | `jog 2 off` | brakes and stops within a fraction of a second; `EV done` |
| 5 | `jog 2 + 100`, let it run | stops by itself at the travel limit, nothing hit |
| 6 | `jog 1 + 30`, then press the knob | stops immediately (STOP), `EV reaction_ms` printed |
| 7 | `jog 4 - 50`, then `stop` | stops immediately |

Write the result in `devlog.md` (entry "Phase 5 prepared without hardware").
If the robot behaves differently from 4.3.1 in normal use, compare with tag `v4.3.1-tested`.

Two older entries in `devlog.md` also still say *"Test on robot: to fill in"* (V1 restrictions,
CSV recording); both were used successfully but never formally written down.

---

## 5. V2 bring-up, step by step

Do the steps in order and write each result in `devlog.md`. Before the first power-on with
motors, keep a hand near the power switch.

### B0 — Hardware checks (before flashing)
- Wiring against `pinmap.md`, including the **V2 corrections**: encoder "+" to 5 V and red LED
  to pin 49; TFT on 51/52/53 (SPI) + 12/13/7; joystick on A0–A3 (switched) or A0/A1 (analog).
- **Joystick type:** the APEM NZ in the BoM is most likely *switched* (one contact per
  direction). Check the part: several tabs per direction (common/NO/NC) = switched; three per
  axis (+, −, wiper) = analog. If analog, set `JOYSTICK_TYPE = JoystickType::Analog` in
  `hw_profile_v2.h`.
- **Limit switches:** same 4 switches and arrangement as V1. Decide NO or NC wiring (NC is
  safer: a broken wire looks like a pressed switch). For NC set `activeHigh = false` for all
  four in `hw_profile_v2.h`.
- Driver boards: check the sense resistor (`R_SENSE`, 0.11 Ω on V1).

### B1 — First start (motor power off)
- Upload env `v2`. Serial monitor: banner shows `profile: V2 (prepared, UNTESTED)`.
- Pass: the TFT shows the header *Concentric Tube Robot* and the menu, twice the size of the
  V1 OLED, in the middle of the screen; turning and pressing the encoder works as on V1.
- If the screen stays white or black: check the TFT wiring (CS 53, D/C 12, RST 13, backlight 7).

### B2 — Inputs and LED
- `watch on`, then press each limit switch by hand: the right name appears with
  *pressed/released*. Fix polarity in the profile if reversed.
- Encoder turn/press/long press: `EV enc`, `EV btn ...`.
- Status LED: green blinking = not homed. On V2 there is no blue (pin 50 is used by the TFT's
  SPI bus): blue states show yellow. To get blue back, wire it to a free pin and set
  `PIN_LED_B` / `LED_HAS_BLUE` (see `pinmap.md`).

### B3 — Drivers, directions, currents (motor power on)
- `drivers`: all 6 found, currents and microsteps as in the profile.
- One axis at a time, small moves: `move 2 5`, `move 2 -5`, … Translation `+` must move the
  carriage **forward** (away from its home block); rotations should all turn the same way for
  `+`. Fix with `dirInvert` in the profile.
- Rotation motors (SCA4218M1804-L, rated 1.8 A) start at 1500 mA. If a rotation skips at
  120 rpm, raise with `cur <axis> <mA>` (max 1800) and put the value in the profile.

### B4 — Homing
- `home`. Pass: all three carriages reach their switch, zero is set, `EV home done`.
- Repeat 5–10 times; the zero should be found within about 0.1 mm (`EV home ... zero set
  (was at ...)` shows the difference). V1 reached this.

### B5 — Limit calibration
- `calib` (or Service → Calibrate limits) after homing, three times. It measures T1's far end
  and the gap between each pair of carriages.
- Put the **smallest** good values into `CALIB_DEFAULT_FRONT_MAX_MM` and `CALIB_DEFAULT_GAP_MM`
  in `hw_profile_v2.h` and rebuild. From then on the limits work right after homing.
- If values differ by more than a few mm between runs, the mechanics slip (as on V1): fix that
  first, or add `CARRIAGE_EXTRA_MARGIN_MM`.

### B6 — Limits and speeds
- `limits` shows the ranges. Try to move each carriage past its limit: the move must be refused
  or stop exactly at the limit (Jog screen shows *LIMIT*).
- Full speed: `mv 2=50 4=50 6=50` (20 mm/s design speed, requirement FRT_6) and back; watch for
  vibration and lost steps (home again and compare the zero). The V2 profile has no speed caps;
  if V2 needs some, set `TRA_SPEED_CAP_MMPS` etc. like V1 did.

### B7 — Joystick
- First run section 4 (regression check and test V1-1) on V2 with the `jog` command — this is
  also the first robot test of the 5.0.0 speed mode.
- Menu → Jog → pick a tube. Up/down = translation of that tube, left/right = its rotation.
- Pass: moves while held, brakes on release, stops at the limit with a beep, a knob press
  stops it, and after a stop it does not move again until the stick has been centred.
- Switched joystick speed: `JOY_SWITCHED_SPEED_PCT` in `motion_config.h` (50 %).

### B8 — Display load and RAM
- `status` while all six axes move (`mv 1=360 2=30 3=360 4=30 5=360 6=30`): `maxloop_us` should
  stay below ~10 ms (the 1 s driver check alone is ~9 ms); `free_ram` about 2 KB.
- If the screen feels slow, that is expected for large changes (each changed 8x8 block is
  drawn in its own loop pass); numbers update several times per second.

### B9 — Demos, park, full test round
- Demos (`demo 1..3` or the menu) once the limits are active; tune speeds
  (`DEMO_SPEED_PCT`) and the poses in `demo.cpp` for the V2 mechanics.
- `park`, power off/on: rotation zeros restored.
- Re-run the phase 1–4 tests from `devlog.md` and the validation tests of the report
  (reaction times, repeatability, speeds) with the control centre's CSV recording.

---

## 6. Known issues and limits

- **V1 mechanics:** carriages slip and vibrate at higher speed, so V1 is capped (10 mm/s,
  100 rpm, 40 % acceleration) and the limit calibration is off. T1's home switch lever is
  probably bent (releases at ~0.8 mm; the soft minimum is 1.0 mm).
- **9 ms loop pause once per second** from the driver health check (blocking UART reads in the
  TMCStepper library). Accepted for a demonstrator (decision D7).
- **Encoder stop reaction ~21 ms**, mostly the 20 ms debounce. Could be halved by stopping on
  the first edge (a press always means stop).
- **Rotation has no sensor:** the zero is set by hand and kept only if the robot was parked
  before power-off.
- **V2 status LED has no blue** unless rewired (pin 50 conflict).
- **No emergency-stop button** on V2. The knob, the `stop` command, the laptop STOP button and
  the main power switch stop the robot.

---

## 7. Recommendations

1. **Bring up V2 with section 5**, then validate against the requirements (speeds, resolution,
   repeatability, reaction times) and compare with V1.
2. **Hardware emergency stop** in the motor power path (or at least on the free pin 2),
   independent of the firmware, before the robot is shown publicly.
3. **StallGuard as a jam detector** (phase 6): the TMC2209 can detect a stalled motor. Use it
   as an extra safety layer that stops the robot if a carriage jams or tubes bind — not as a
   replacement for the switches. Needs tuning per axis.
4. **Native colour screens** on the TFT instead of the 2x copy of the OLED screens (larger
   text, colours for states and faults). Keep the rule: draw in small pieces per loop pass.
5. **3D view on the laptop** (phase 6): the control centre already receives all six positions
   10 times per second; a tube model drawn from them would show the robot's shape live.
6. **Rotation index sensor** (e.g. a Hall sensor and magnet per tube) for a real rotation zero.
7. **Non-blocking driver reads** to remove the 9 ms pause.
8. Demo choreography: refine on V2 (V1 slip made fine tuning pointless).
