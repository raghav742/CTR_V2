## D1 — Develop on V1, switch hardware via profiles
**Decision:** All hardware differences live in one profile file; V1 and V2 are separate build environments.
**Why:** Firmware is developed on the unchanged V1 robot; moving to V2 should mean changing a profile, not code.
**Alternatives:** Separate branches per robot (rejected: changes would have to be made twice).