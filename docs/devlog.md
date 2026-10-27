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
**Test:** `v1` builds and prints the profile; `v2` stops with "V2 profile not defined yet". Result on robot: _to fill in_
