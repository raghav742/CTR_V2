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

## 2026-10-28 — Step 1.3: Coordinated multi-axis moves
**What:** `motion` rewritten around move groups with one shared "progress" profile (accelerate, cruise, brake from 0 to 1). Each axis' speed = progress speed x its own distance, so all axes in a move start, accelerate and finish together. Progress limits chosen so no axis exceeds its own max speed or acceleration. New command `mv <axis>=<amount> ... [t=<s>] [v=<%>]`; `move` is now a one-axis group. Done events report elapsed time.
**Why:** Smooth, coordinated tube motion for demos, record and replay, and laptop control.
**Simulation:** `mv 2=30 4=30 6=30` predicted 2.00 s, ran 1.98 s; `t=5` ran 4.98 s. Proportionality during a move (30 mm / 10 mm / 3 mm / 90 deg): every axis at 50.7 % of its distance at mid-move.
**Test on robot (2026-10-28):** All coordinated moves stay in proportion: when the T1 far-end switch stopped a mixed move, every axis (3 rotations, 2 translations) was at 94.7 % of its distance; an encoder stop left all three carriages at the same 8.21 mm. Elapsed times match predictions (1.98/2.00 s, 0.98/1.00 s, 2.11/2.13 s, 2.98/3.00 s). T3 hit its home switch at -0.27 mm twice from different moves (no slip). T1 sits near its far end (far switch after ~19 mm), to be measured during homing.

## 2026-10-28 — Step 1.4: State machine
**What:** New module `system_state` (namespace `sys`): states BOOT, IDLE (not homed), HOMING, READY (homed), MANUAL, DEMO, ATTRACT, VISITOR, REMOTE, FAULT, with an explicit table of allowed transitions and an LED pattern per state. Moves are refused in BOOT and FAULT. An unexpected limit switch during motion and driver over-temperature now enter FAULT (motion stopped, reason stored); `ack` or a 1 s encoder press acknowledges. Encoder press or `stop` during a normal move only stops, no fault. New commands `state`, `ack`; `status` shows the state. Driver temperature read one driver every 200 ms (UART reads take 1-2 ms each).
**Why:** One place decides what is allowed; the later modes (homing, demo, manual, laptop) plug into the same table.
**Simulation:** switch pressed during a move -> stop + "EV state IDLE -> FAULT (T3-home hit)"; moves refused with reason; `ack` -> IDLE; moves work again.
**Test on robot (2026-10-28):** BOOT -> IDLE at startup. T2-home pressed by hand during a rotation -> stop + "IDLE -> FAULT (T2-home hit)"; moves refused with reason; cleared with `ack` and with a 1 s encoder hold. Short encoder press during motion stops without fault. `status` shows state. Note: drivers report 1,767 mA for translation (requested 1,900; nearest TMC2209 current step with R_sense 0.11 ohm) and 1,491 mA for rotation.

## 2026-10-28 — Step 1.5: Safety manager
**What:** New module `safety`, the single gatekeeper: every move from any source passes `safety::checkMove()` (driver present/enabled, 50 mm cap until homed, no moving into a pressed switch). Watches limit switches (unexpected press during motion -> FAULT; homing can mark a switch as expected) and driver health round-robin every 200 ms (over-temperature and "driver not answering" -> FAULT). User stops (encoder, `stop`) and switch faults report the measured reaction time from the input edge. Hardware watchdog (1 s): first timeout stops the step interrupt, second resets the board; reset cause reported at startup. New commands `safety`, `wdtest`. Safety code removed from `motion` and `sys`.
**Why:** One place for all safety rules, so no motion source can bypass them; measured stop response for validation; a frozen program can never leave motors running.
**Simulation:** switch fault reaction 7.4 ms (5 ms debounce + loop); encoder stop 20.7 ms (20 ms debounce); `wdtest` -> board restarts, reports reset_cause=watchdog.
**Test on robot (2026-10-28):** `wdtest` -> board restarted by the watchdog, reset cause reported. Measured stop reaction times (input edge -> all motion stopped):
| Input | Reaction | Made up of |
|---|---|---|
| Limit switch (T1-far hit during a move -> FAULT) | 10.2 ms | 5 ms debounce + up to ~9 ms driver check (see 1.6) |
| `stop` command | 4.3 ms (twice) | command parsing + stop |
| Encoder button | 20.7 ms, 21.9 ms | 20 ms debounce + loop |
The encoder figure is dominated by its 20 ms debounce; can be reduced later by stopping on the first press edge if needed (a press always means stop, so a bounce cannot cause a wrong action).

## 2026-10-28 — Step 1.6: Serial protocol v1
**What:** The console becomes a protocol a laptop program can rely on (`docs/protocol.md`). Line types `OK`/`ERR` (replies), `EV` (events), `ST` (status stream). Request ids: `#12 pos` tags every line of the reply with `#12 `. New commands `info` (protocol version, firmware, axis names and units, switch names), `ping`, `st` (one status line) and `stream <ms>|off` (status line every 20–10000 ms). All modules print through one output object (`serial_io`, `Out`) instead of `Serial`. Serial speed 115200 -> 250000 baud and send buffer 64 -> 256 bytes; the stream skips a line (counted as `st_skipped`) instead of waiting when the buffer is full. Fast decimal printing (Arduino's float printing took ~0.5 ms per number). Ramp update now uses precomputed per-axis factors (one multiply per axis per ms). `platformio.ini`: monitor 250000, `monitor_echo`/`monitor_eol` moved to `[env]` (they were only under `v2`).
**Why:** Phase 4 laptop control centre needs replies it can match to commands and live positions; the occasional 4–7 ms loop stall came from `Serial.print` waiting for the 64-byte buffer at 115200 baud.
**Simulation (all 6 axes at maximum speed, the worst case):** ids, `info`, `ping`, `st`, `stream` correct while moving; `#9 stop` -> all lines tagged, reaction 3.8 ms. Printing no longer waits. Measured costs: 6 numbers 3.3 ms -> 1.6 ms; ramp update 1.0 ms -> 0.74 ms per run under full load. Worst single loop passes still ~3–8 ms when a long line is built (e.g. `OK mv` reply, `EV done`), now CPU time rather than waiting. RAM 41 %.
**Test on robot (2026-10-28):** `info`, `ping`, request ids, `st`, `stream` all correct. Ids also tag the events a command causes (`#10 EV stopped`, `#10 EV reaction_ms=4.3`, `#10 OK stop`). Stream at 200 ms during a full-rotation-speed 6-axis move: lines exactly every 200 ms, `st_skipped=0`. Move `mv 1=720 2=6 3=720 4=6 5=720 6=6`: 1.97 s (predicted 2.00); return move back to exactly 0.00 / -7.00; stop mid-move left all axes at the same point (706.73 deg, -1.11 mm). `stop` reaction 4.3 ms (twice). Longest interrupt 32 us. A move from unknown start position hit the T1 far-end switch -> FAULT "T1-far hit", reaction 10.2 ms, moves refused until `ack`, then refused into the pressed switch (correct).
**Findings:**
- `maxloop_us` ~8,950 every second, identical idle, moving and streaming: not printing. Cause: the safety driver check (step 1.5) does two UART reads per check and TMCStepper waits ~4.5 ms inside each read (2 ms fixed delays + reply). The 1-2 ms per read assumed in step 1.4 was wrong; this was most likely also the "slow pass while printing" seen in step 1.2. Decision D7: check one driver every 1 s instead of every 200 ms (v1.6.1); 9 ms switch-detection delay accepted for the prototype.
- PlatformIO monitor sends arrow keys and Delete as escape codes that ended up inside commands. v1.6.1 ignores escape sequences and handles Backspace.

## 2026-10-30 — Step 2.1: Homing
**Mechanics (confirmed on the robot):** each translation carriage carries its own home switch, which presses against a fixed block on the frame when the carriage moves backward (negative mm). Blocks are staggered: T3 at the back end, T2 further forward, T1 near the front; the T1 far-end switch is on the frame at the front. Casper's notes (switches 38/40 also detect carriage-to-carriage contact) do not match this mounting.
**What:** New module `homing` (non-blocking). `home` homes T3, T2, T1 in that order (`HOME_ORDER` in the hardware profile); `home 2|4|6` homes one axis. Per axis: if the switch is already pressed, move forward until it releases (+1 mm); move backward at 5 mm/s until the switch presses; set position 0; move forward until it releases (+1 mm). Only that switch is "expected"; any other switch -> FAULT. Not found within 210 mm, or switch stays pressed -> FAULT. `stop` / encoder abort homing (-> IDLE). All axes homed -> READY. Per-axis homed flags: the 50 mm limit now only applies to axes that are not homed; homed axes cannot be sent below `TRA_SOFT_MIN_MM` = 1.0 mm. Disabling a driver discards that axis' zero. While HOMING, other move commands are refused. Homing's own moves print no `OK`/`EV done` lines (protocol: `OK` only answers a command). `st` and the stream gain `homed=`. Firmware 2.1.0.
**Why:** Real positions instead of counted steps: removes the 50 mm limit and the "where is T1 today" problem, and is the basis for soft limits (2.2) and collision protection (2.3). Back carriage first gives each carriage the most room behind it.
**Simulation:** full sequence T3 -> T2 -> T1 -> READY (T3 starting on its switch handled); 50 mm limit gone after homing; `stop` during homing -> aborted -> IDLE; T2 switch pressed while T3 was homing -> FAULT "T2-home hit"; `home 4` alone -> IDLE with `homed=000100`; `home 5` refused (rotation); move below 1.0 mm refused with the largest allowed move.
**Test on robot:** _to fill in_
