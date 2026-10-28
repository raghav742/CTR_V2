## D1 — Develop on V1, switch hardware via profiles
**Decision:** All hardware differences live in one profile file; V1 and V2 are separate build environments.
**Why:** Firmware is developed on the unchanged V1 robot; moving to V2 should mean changing a profile, not code.
**Alternatives:** Separate branches per robot (rejected: changes would have to be made twice).

## D2 — V1 encoder: pin 49 is the encoder supply, no red LED
**Decision:** On V1, pin 49 is driven HIGH permanently as the encoder's "+" supply. Status colours use green, blue and cyan only. The button is read active HIGH without the internal pull-up.
**Why:** Measured 2026-10-28: driving pin 49 LOW switches off all LEDs and the button; red never lights. The "+" and red wires are swapped between PCB connector P1 and the encoder. No hardware changes are allowed on V1.
**V2:** Swap the two wires back; full RGB becomes available (profile setting `LED_HAS_RED`).

