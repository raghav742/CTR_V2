## D1 — Develop on V1, switch hardware via profiles
**Decision:** All hardware differences live in one profile file; V1 and V2 are separate build environments.
**Why:** Firmware is developed on the unchanged V1 robot; moving to V2 should mean changing a profile, not code.
**Alternatives:** Separate branches per robot (rejected: changes would have to be made twice).

## D2 — V1 encoder: pin 49 is the encoder supply, no red LED
**Decision:** On V1, pin 49 is driven HIGH permanently as the encoder's "+" supply. Status colours use green, blue and cyan only. The button is read active HIGH without the internal pull-up.
**Why:** Measured 2026-10-28: driving pin 49 LOW switches off all LEDs and the button; red never lights. The "+" and red wires are swapped between PCB connector P1 and the encoder. No hardware changes are allowed on V1.
**V2:** Swap the two wires back; full RGB becomes available (profile setting `LED_HAS_RED`).

## D3 — Step pulses from one Timer1 interrupt, 20 kHz
**Decision:** A single Timer1 interrupt at 20 kHz generates STEP pulses for all six axes using a 16-bit phase accumulator per axis. Ramps are calculated in the main loop (every 1 ms) and passed to the interrupt as a rate. Microstepping: 8 (translation), 16 (rotation), with driver interpolation to 256.
**Why:** V1 generated pulses in blocking loops, freezing inputs and safety during motion. One timer for all axes keeps CPU use predictable; the accumulator gives each axis an independent speed. At 8 microsteps, 20 mm/s = 16,000 steps/s, under the 20,000 steps/s limit.
**Measured (simulation):** worst case (all axes at max speed) 45 % CPU in the interrupt.
**Alternatives:** One hardware timer per axis (the Mega has only 4 usable 16-bit timers); AccelStepper/FastAccelStepper (blocking-free but pin restrictions on AVR; V1 step pins don't fit).

## D4 — Translation axes use SpreadCycle
**Decision:** Translation drivers run in SpreadCycle at all speeds. Rotation stays in StealthChop (quiet). Settings in `motion_config.h` (`TRA_CHOPPER`, `ROT_CHOPPER`).
**Why:** Measured 2026-10-28. StealthChop on translation loses steps at 10, 15 and 20 mm/s (T1) and also at 5 mm/s (T3, about 5 mm lost over one cycle). Hybrid mode with a 5 mm/s switch-over therefore still slipped on slow moves. In SpreadCycle, T3 repeatability against its home switch: reference -54.65 mm; after fast (20 mm/s) cycles -54.67, -54.67; slow (3 mm/s) -54.57; i.e. within 0.1 mm, no steps lost. StealthChop has less torque under load; the lead-screw nut plus tube friction needs SpreadCycle.
**Cost:** Translation motors are slightly more audible at low speed.
**Alternatives:** Hybrid with a lower switch-over (not tested; StealthChop already slipped at 5 mm/s), higher current (already 1.9 A of the 2 A rating).

## D5 — Hardware watchdog in interrupt-then-reset mode
**Decision:** The ATmega2560 watchdog runs with a 1 s timeout in interrupt + reset mode. The main loop feeds it every pass. On the first timeout the watchdog interrupt switches off the step interrupt (no more step pulses) and marks the cause in RAM that survives a reset; one second later the board resets and reports "reset_cause=watchdog".
**Why:** A frozen program must never leave motors moving. Stopping the pulses in the interrupt works even if the board then fails to restart (older Mega bootloaders can hang after a watchdog reset).
**Alternatives:** Plain reset mode (no chance to stop pulses first; cause not reportable on bootloaders that clear MCUSR); no watchdog (V1).

## D6 — One text protocol for humans and programs
**Decision:** Plain-text lines with fixed prefixes (`OK`, `ERR`, `EV`, `ST`), `key=value` fields, optional request ids (`#<n>`), 250000 baud, 256-byte send buffer. The status stream skips a line rather than wait. Versioned with `PROTO_VERSION`; documented in `docs/protocol.md`.
**Why:** The same commands work in the Serial Monitor and from the laptop program, so everything the program does can be tried by hand. Ids let a program match replies while events and stream lines arrive in between. `info` makes the laptop program configure itself from the robot, so it works on V1 and V2 unchanged. Never waiting for USB keeps the safety loop fast.
**Alternatives:** Binary protocol (smaller, but unreadable when debugging and gains little at 250000 baud); JSON (readable, but large lines and slow to build on an 8-bit CPU).

## D7 — Driver health check once per second, blocking reads accepted
**Decision:** The safety manager checks one TMC2209 per second (round-robin, each driver every 6 s) using the TMCStepper library's blocking reads. Setting `DRV_CHECK_MS` in `motion_config.h`.
**Why:** Measured 2026-10-28: one check (alive + temperature) blocks the main loop for ~9 ms because TMCStepper waits inside each UART read. At the original 200 ms interval this happened 5 times a second. A 9 ms worst-case delay in noticing a limit switch is acceptable for the prototype (stop reaction measured 10.2 ms including it); checking less often makes it rare. The TMC2209 also protects itself (thermal shutdown in hardware).
**Cost:** Over-temperature or a lost driver is reported up to 6 s later; the 9 ms pause still occurs once a second.
**Alternatives:** Own non-blocking UART read (send request, collect reply over later loop passes; no pause at all, ~1 h work; revisit if switch reaction or display smoothness needs it); one read per check (halves the pause).
