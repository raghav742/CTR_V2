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

## D4 — Translation axes use hybrid chopper mode
**Decision:** Translation drivers start in hybrid mode: StealthChop below 5 mm/s, SpreadCycle above (TMC2209 TPWMTHRS). Rotation stays in StealthChop. Settings in `motion_config.h`.
**Why:** Measured 2026-10-28 on T1: in StealthChop the translation motor slips at 10, 15 and 20 mm/s (position lost, measured against the home switch). In SpreadCycle 10, 15 and 20 mm/s run without loss. StealthChop has less torque at speed; the lead-screw nut plus tube friction needs it. Hybrid keeps slow moves quiet.
**Alternatives:** SpreadCycle always (louder at low speed); higher current (already 1.9 A of 2 A).

