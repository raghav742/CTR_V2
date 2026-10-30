# CTR V2 Firmware — Architecture and Design Explanation

This document explains **how the firmware works and why it is built this way**.
It complements:
- `devlog.md` — what changed per step, with test results
- `decisions.md` — design decisions (D1, D2, ...) with alternatives
- `pinmap.md` — all pins, V1 and planned V2

Firmware version at last update: **1.5.0** (Phase 1, step 5 of 6).

---

## 1. Starting point: the V1 firmware

The V1 firmware (Casper de Ruiter, kept unchanged in `reference_v1/`, git tag `v1-baseline`)
worked but had one structural weakness: **motion was generated in blocking loops**.
While a motor moved, the program sat inside a loop pulsing step pins, so nothing else ran:
no button reading, no safety checks, no display updates. This pattern ran through every
file (`Motion.ino`, `Safety.ino`, `SerialCmd.ino`, `Interface.ino`), which is why V2 was
rebuilt with a new structure rather than edited in place. Good V1 parts (OLED graphics,
buzzer melodies, TMC2209 setup) are reused when their phase comes.

Other V1 issues found during analysis and testing:
- Serial commands read with a 1 s blocking timeout.
- Demo limited to 4 mm/s; 20 mm/s only possible for a single axis.
- Encoder button polarity used inconsistently; "EMI on pin 44" workaround of 200 ms.
- Several configuration values unused (`HOME_DIR`, `COLLISION_GUARD`).
- Report pin tables partly wrong (code was correct; see `pinmap.md`).

---

## 2. Project structure

```
CTRV2.0/
├─ platformio.ini      build recipe: board, libraries, environments v1 and v2
├─ include/            descriptions and settings (no logic)
│   ├─ hw_types.h        data types: what an axis or a switch is
│   ├─ hw_profile.h      selects v1 or v2 profile from the build environment
│   ├─ hw_profile_v1.h   V1 robot: pins, currents, switches, homing order, mechanics, interface
│   ├─ hw_profile_v2.h   V2 robot (filled in at Phase 5)
│   ├─ motion_config.h   tuning: speeds, accelerations, chopper modes
│   └─ version.h         firmware version, protocol version, console baud rate
├─ src/                the program (one module = one .h + one .cpp)
├─ reference_v1/       Casper's code, read-only, not compiled
└─ docs/               this file, devlog, decisions, pin map, protocol
```

**Hardware profile (decision D1).** Program code never contains a pin number. It always
asks the profile, e.g. `hw::AXES[2].pinStep`. Building environment `v1` gives pin 22;
environment `v2` gives whatever `hw_profile_v2.h` says. Moving to V2 means filling in one
file and building the other environment. `hw_profile_v2.h` currently contains `#error`
so it cannot be flashed by accident.

**Why `.cpp`/`.h` instead of Arduino `.ino` tabs.** The Arduino IDE glues all `.ino` tabs
into one file, so everything is global and any tab can change any variable (in V1,
`g_stopFlag` was set in one tab, cleared in another and read in a third). With separate
modules, each `.h` is the module's public interface and everything inside
`namespace { ... }` in the `.cpp` is private. A bug in the encoder can only come from
`encoder.cpp`. It is still Arduino: `Arduino.h`, `setup()` and `loop()` work as before.

---

## 3. Core rule: nothing may wait (Step 1.1)

Every module has two functions:
- `begin()` — called once in `setup()`
- `update()` — called on every pass of `loop()`; checks whether there is something to do,
  does it quickly, and returns

`loop()` only calls the `update()` functions. It runs about 16,000 times per second idle
and about 9,000 times per second with three axes moving at 20 mm/s (measured).

Two techniques make "never wait" possible:
1. **Clocks instead of `delay()`.** A module stores a timestamp and checks on later passes
   whether enough time has passed (`if (now - last >= 500) ...`). Used for LED blinking,
   debouncing, long-press detection, driver checks.
2. **Events that are "taken".** When the encoder sees a press it sets a flag. Whoever needs
   it calls `encoder::takePress()`, which returns `true` once and clears the flag. Events
   are never lost and never counted twice.

### Modules from Step 1.1
| Module | Function |
|---|---|
| `encoder` | Rotation (count on falling edge of A, direction from B); button debounced 20 ms with press, release, long press (1 s) events. Polarity from the profile. |
| `limit_switches` | Debounced (5 ms) switch states; polarity per switch from the profile; `findHome(axis)`, `findFarEnd(axis)`. |
| `status_led` | Green / blue / cyan / off, solid or blinking, non-blocking. Drives pin 49 HIGH first (encoder supply, decision D2). |
| `console` | Serial commands read character by character; runs a command when a full line has arrived; never waits. Replies start with `OK`/`ERR`, events with `EV`. |
| `loop_stats` | Loop passes per second and slowest single pass: proof that nothing blocks. |

### Hardware findings during Phase 0 (V1 robot)
- **Encoder wiring (decision D2).** The encoder's "+" pin (LED supply and button common)
  is wired to Arduino pin 49 (PCB net "ENC_R"); the red LED wire went where "+" should be.
  Pin 49 must stay HIGH, otherwise LEDs and button are dead. Red is unavailable on V1.
- **Encoder button** is active HIGH; the encoder board has its own pull-down. With the
  Mega's internal pull-up enabled, the released button sat near 1 V, close to the logic
  threshold: very likely the cause of V1's "EMI on pin 44". V2 firmware reads it without
  pull-up (clean 0 V).
- **Limit switches** go through an RC filter and an inverting Schmitt trigger
  (SN74AHCT14): a pressed normally-open switch reads HIGH.

---

## 4. Motion: conductor and drummer (Step 1.2)

Motion is split into two parts:
- **The conductor** (`motion`, in the main loop) decides *what* should happen: distances,
  speeds, acceleration ramps. It recalculates every 1 ms.
- **The drummer** (`steppers`, a timer interrupt) only produces step pulses at the rate it
  is given. It runs on a hardware timer, so it never misses a beat even while the main loop
  is busy.

```
console ──► motion ─────────────────► steppers (interrupt, 20,000x/s) ──► STEP pins ──► TMC2209 ──► motor
              "how fast right now?"      "pulse now or not?"
              every 1 ms                 every 50 us
drivers ──► TMC2209 settings over UART, once at startup
```

### `drivers` — TMC2209 configuration
At startup, for each driver: check it answers on UART (`test_connection`), set RMS current,
microstepping, interpolation to 256, StealthChop/SpreadCycle, automatic PWM scaling, then
enable (EN pin LOW: holds position). Mostly Casper's tested setup.
Measured actual currents: rotation 1,491 mA (requested 1,500), translation 1,767 mA
(requested 1,900; nearest current step the TMC2209 can set with R_sense = 0.11 ohm).

### `steppers` — the step interrupt (decision D3)
**Hardware timer.** Timer1 counts at 2 MHz up to 100, i.e. every 50 us, and raises an
interrupt: the CPU pauses the loop, runs the interrupt function, and continues.
That is 20,000 interrupts per second.

**One timer, six speeds: the phase accumulator.** Each axis has a 16-bit counter and an
increment that represents its speed. On every tick:
```
counter = counter + increment
if the counter overflowed past 65,535  ->  one step pulse
```
Increment 32,768 overflows every 2nd tick (10,000 steps/s); increment 6,554 every 10th
tick (2,000 steps/s). Each axis' speed is one number; all six share one timer.
Maximum: 20,000 steps/s per axis.

**Pulses.** On a step the pin goes HIGH; it goes LOW on the next tick (50 us later).
The TMC2209 needs only 0.1 us.

**Position and stopping.** On each step the position counts +1/-1 and "steps remaining"
counts down. At zero the axis stops by itself, exactly on target, independent of the main
loop.

**Why the interrupt code looks unusual.** It runs 20,000 times per second on an 8-bit CPU.
The first version (array of structs, 32-bit counters) used 85 % of the CPU in the
simulator. The final version is unrolled per axis with templates (all addresses fixed at
compile time), uses 16-bit accumulators and keeps interrupt-only data non-volatile:
45 % worst case (all six axes at full speed, simulation); on the robot with three axes at
20 mm/s the longest interrupt was 29.5 us of the 50 us tick.

**Self-measurement.** At the end of each interrupt, Timer1's counter shows how long the
interrupt took; `status` shows the maximum as `isr_max_us`.

### Microstepping
- Translation: 8 microsteps -> 200 x 8 / 2 mm = **800 steps/mm**. 20 mm/s = 16,000 steps/s,
  under the 20,000 limit. (V1's 16 microsteps would need 32,000.)
  Resolution 0.00125 mm per step, 200x finer than the 0.25 mm requirement.
- Rotation: 16 microsteps -> 200 x 16 / 360 = **8.9 steps/degree**.
- The driver interpolates to 256 internally, so motion is smooth either way.
- Lead screw: 2 mm per revolution (Nanotec: 0.01 mm per full step).

### Ramps
Every 1 ms the speed is recalculated:
- speed up: add acceleration x elapsed time
- never faster than the requested speed
- brake in time: the highest speed from which it can still stop in the remaining distance
  is `sqrt(2 x acceleration x remaining distance)`
This gives a trapezoid: up, cruise, down. Translation acceleration 40 mm/s^2
(20 mm/s reached in 0.5 s); rotation 120 rpm/s. Values in `motion_config.h`.

### Chopper mode: StealthChop vs SpreadCycle (decision D4)
The TMC2209 has two current-control modes:
- **StealthChop**: very quiet, but less torque at speed and under load.
- **SpreadCycle**: full torque, slightly more audible.

Measured on the robot: in StealthChop the translation motors lost steps at 10, 15, 20 mm/s
and also at 5 mm/s (T2 lost about 16 mm, T3 about 5 mm). Diagnosed with runtime commands
`chop`, `ms`, `cur` (no reflash needed): SpreadCycle fixed it; microstepping was not the
cause. Repeatability test (T3 against its home switch, SpreadCycle): reference -54.65 mm,
after 20 mm/s cycles -54.67 / -54.67, slow 3 mm/s -54.57: within 0.1 mm, no steps lost.
Result: **translation SpreadCycle, rotation StealthChop.**

**Measuring slip with the home switch.** Positions are counted steps, so a slipping motor
keeps counting while the carriage stops. Method: drive back slowly until the home switch
stops the carriage, note the count (reference); do a fast move out and back; the switch
must trigger at the same count. A difference = lost steps.

---

## 5. Coordinated moves (Step 1.3)

**Problem.** After Step 1.2 each axis ramped independently, so axes started together but
finished at different times (rotation done after 1 s, translation after 2 s).

**Idea: one shared progress value.** A move ("group") has one progress value p from 0 to 1.
p follows one speed curve (accelerate, cruise, brake). Each axis' speed =
**p's speed x its own distance**. Example: p advances 0.5/s; T1-Tra (30 mm) runs at
15 mm/s, T1-Rot (360 deg) at 180 deg/s; both are always at the same percentage.

**Choosing the speed.** For each axis in the group:
```
p-speed limit = axis max speed / axis distance
p-accel limit = axis max acceleration / axis distance
```
The group uses the smallest of these, so no axis ever exceeds its own limits.
Example `mv 2=30 4=30 6=30`: p-speed 20/30 = 0.667 /s, p-accel 40/30 = 1.33 /s^2,
time = 1/0.667 + 0.667/1.33 = 1.5 + 0.5 = **2.00 s** (robot measured 1.98 s).
- `v=25` scales the p-speed to 25 %.
- `t=5` solves the time formula backwards for the p-speed; refused if faster than allowed.

**Braking** uses the longest axis ("master"): remaining progress = its remaining steps /
its total steps, with the same sqrt formula, so the whole group brakes together.

**The interrupt did not change.** Only the rate calculation in the main loop changed.

**Verified on the robot:** when the T1 far switch stopped a mixed 5-axis move, every axis
was at 94.7 % of its distance; times matched predictions (1.98/2.00, 0.98/1.00, 2.11/2.13,
2.98/3.00 s).

Up to 6 groups can run at once (e.g. independent single-axis moves). `move` is a group
with one axis; `mv` is the coordinated command.

---

## 6. State machine (Step 1.4)

**Problem.** The robot did not know what it was doing; every command worked at any time.

**Idea.** The robot is always in exactly one state and can only change state along
defined paths (like a traffic light).

| State | Meaning | Moves allowed | LED (V1) |
|---|---|---|---|
| BOOT | starting up | no | cyan |
| IDLE | ready, not homed | yes (50 mm cap) | green, slow blink |
| HOMING | homing running (Phase 2) | yes | cyan, slow blink |
| READY | homed, positions real | yes | green |
| MANUAL | manual control (Phase 3) | yes | blue |
| DEMO / ATTRACT / VISITOR | operating modes (Phase 3) | yes | blue, slow blink |
| REMOTE | laptop control (Phase 4) | yes | cyan |
| FAULT | something went wrong | **no**, until acknowledged | fast blink (cyan on V1) |

**Implementation (`system_state.cpp`).**
1. **Transition table.** For each state, a 16-bit number with one bit per allowed target
   state (`B(State::Homing) | B(State::Remote)`). `request(to)` checks the bit and refuses
   with `ERR not allowed: IDLE -> DEMO`. Extra rule: READY requires homed.
2. **One function makes every change, `enter()`**: sets the state, sets the LED, prints
   `EV state IDLE -> FAULT (reason)`. Every change is logged; LED always matches.
3. **FAULT.** `sys::fault("reason")` stops all motion, stores the reason, enters FAULT.
   `sys::acknowledge()` (command `ack` or 1 s encoder press) returns to READY if homed,
   else IDLE. A fault is never cleared by accident.
4. **`motionAllowed()`**: moves refused in BOOT and FAULT, with the fault reason.

**Stopping on purpose vs a fault.** An encoder press or `stop` during a move only stops.
An unexpected limit switch or a driver problem is a FAULT.

**Verified on the robot:** switch pressed during a rotation -> stop + FAULT; moves refused;
cleared with `ack` and with a 1 s encoder press; short press only stops.

---

## 7. Safety manager (Step 1.5)

**Problem.** Safety rules were scattered over `motion`, `system_state` and `main`. Every
future motion source (menu, demo, joystick, laptop) would have had to repeat them.

**Idea: one gatekeeper.**
```
console / menu / demo / laptop ──► motion::moveGroup() ──► safety::checkMove() ──► steppers
limit switches, drivers, watchdog ──► safety::update() ──► sys::fault() ──► stop everything
```
- The **safety manager notices** problems.
- The **state machine handles the consequence** (FAULT, refuse motion, wait for ack).
- **Motion only moves**, after the safety manager says yes.

### Functions (`safety.cpp`)
1. **`checkMove(axis, amount)`** — every move from any source passes it. Refuses if the
   driver is missing or disabled; a translation move is over 50 mm while not homed; or the
   axis would move further into a switch that is already pressed.
2. **Switch monitoring** — a switch becoming pressed while anything moves -> FAULT.
   **Expected switches**: homing (Phase 2) marks the switch it is looking for, so that
   switch does not cause a fault.
3. **Reaction time** — `limit_switches` records `rawEdgeUs` (time of the raw signal change,
   before debouncing); `encoder` records `pressEdgeUs`. At a stop, safety prints
   `EV reaction_ms = now - edge`. Simulation: switch 7.4 ms (5 ms debounce + loop),
   encoder 20.7 ms (20 ms debounce + loop).
4. **Driver health** — every 200 ms one driver in turn: still answering on UART
   (`drivers::alive`)? over-temperature? Either -> FAULT. One at a time because each UART
   read takes 1–2 ms.
5. **`userStop()`** — encoder press and `stop` command: stop all motion (no fault) and
   report the reaction time.

### Hardware watchdog (decision D5)
A separate timer inside the ATmega2560, independent of the program. The program must reset
("feed") it regularly; if the program freezes, the timer runs out.
- Timeout 1 s; `safety::update()` feeds it on every loop pass.
- **Interrupt-then-reset mode.** First timeout: the watchdog interrupt switches off the
  Timer1 step interrupt, so no more step pulses (motors stop). One second later the board
  resets.
- **Reset cause.** A variable in the `.noinit` RAM section survives the reset; the
  watchdog interrupt writes a marker (0xBEEF) there. At startup `safety::begin()` reports
  `reset_cause=watchdog`. It also reads the chip's reset register (MCUSR) for power-on,
  reset button/upload and brown-out (supply dip).
- **Verified on the robot:** `wdtest` froze the program; the board restarted by itself and
  reported `reset_cause=watchdog`. The bootloader does not have the known hang-after-
  watchdog bug of older Mega bootloaders.

### Startup order (`main.cpp`)
```
setup():  safety::begin()        first: read reset cause, switch any old watchdog off
          ... all modules start (driver setup takes a while) ...
          safety::armWatchdog()  last: 1 s countdown starts only now
loop():   safety::update()       early in every pass: feed watchdog, check switches/drivers
```

---

## 8. Serial protocol (Step 1.6, decision D6)

The console from Step 1.1 is now a protocol: the same text lines work for a person in the
Serial Monitor and for the phase 4 laptop program. Full reference: `docs/protocol.md`.

### Line types
Every line from the robot starts with a word that says what it is: `OK` / `ERR` (reply to a
command), `EV` (something happened), `ST` (status stream). A program only has to look at the
first word. Data is always `key=value`, so new fields can be added later without breaking
an older program.

### Request ids
A laptop sends commands while events and stream lines keep arriving, so it needs to know
which line answers which command. Writing `#12 pos` makes every line printed during that
command start with `#12 `. Implementation: `console::execute()` strips the `#12`, calls
`Out.beginReply(12)`, runs the command, then `Out.endReply()`. Events caused by the command
(e.g. `EV stopped` after `#9 stop`) carry the id too, which is correct: the command caused them.

### `serial_io` — the output object `Out`
All modules print through `Out` instead of `Serial` (same `print`/`println` calls). `Out`
is a small class derived from Arduino's `Print`; only its `write(c)` is new: at the start of
a line it inserts the `#id ` tag when a request id is active, then passes the character to
`Serial`. Having one output point also made two speed fixes possible in one place:
- **Fast decimals.** Arduino prints a float digit by digit with float maths (~0.5 ms per
  number). `Out.print(x, 2)` scales once to an integer (12.345 -> 1235), splits whole part
  and fraction with 16-bit integer maths when the value fits, and sends the text in one go.
- **`room()`** returns free space in the send buffer, used by the stream.

### Why printing used to stall the loop
`Serial.print` only copies characters into a send buffer; an interrupt sends them over USB.
At 115200 baud one character takes 87 us, and the buffer held 64 characters. A 130-character
reply therefore filled the buffer and `print` waited ~5.7 ms for space; the whole loop
(safety checks included) waited with it. Fixes:
1. 250000 baud: 40 us per character, and exact on a 16 MHz clock (115200 is 2 % off).
2. Send buffer 256 bytes (`-DSERIAL_TX_BUFFER_SIZE=256` in `platformio.ini`; costs 768 bytes
   of RAM because the core uses the same size for Serial1-3).
3. The stream checks `Out.room()` first and **skips** a line if it does not fit (`st_skipped`).
Only very long human listings (`help`) can still wait.

**Robot result:** printing no longer shows up in `maxloop_us`. The ~9 ms that remained was
traced to the driver health check from Step 1.5: TMCStepper waits ~4.5 ms inside every UART
read (fixed 2 ms delays plus the reply), and one check reads twice. Decision D7 runs the
check once per second instead of five times.

**Terminal keys (v1.6.1):** the PlatformIO monitor sends arrow keys and Delete as escape
sequences (`ESC [ C`, `ESC [ 3 ~`). The console now skips these and treats Backspace
(0x08 / 0x7F) as "delete last character".

Remaining cost is CPU time, not waiting: the Arduino serial driver costs ~10 us per
character (including its send interrupt), so an `ST` line costs ~1.5 ms; at 10 lines/s that
is 1.5 % of the CPU.

### Status stream
`stream 100` prints `ST t=... state=... busy=110000 pos=...` every 100 ms from
`console::updateStream()` (called at the end of `console::update()`). `st` gives the same
fields once, as an `OK` reply. `info` describes the axes (names and units) and switches, so
the laptop program builds its display from the robot itself (works for V1 and V2).

### Ramp update optimisation
Profiling in the simulator showed `motion::update()` taking ~1 ms of wall time per run when
all six axes ran at maximum speed (the step interrupt then uses most of the CPU, so all main
loop work runs slower). Per axis it did speed x distance, x scale, float-to-integer, every
millisecond. Now `k[i] = distance x scale` is computed once when the move starts and
`steppers::setIncrement()` takes the integer directly: one multiply per axis. 1.0 -> 0.74 ms.

## 9. Homing (Step 2.1, decision D8)

### The mechanics
```
 BACK                                                            FRONT
 |[T3 blk]                 [T2 blk]           [T1 blk]         [far sw]|
 |   <-sw T3 carriage   <-sw T2 carriage   <-sw T1 carriage     (39)   |
 |      (42)               (40)               (38)                     |
   <-- negative mm = backward = homing          positive mm = forward -->
```
Each translation carriage carries its own switch. Moving backward, the switch presses
against a fixed block on the frame. So each carriage's zero is fixed to the frame and
does not depend on where the other carriages are.

### The tube constraint
The tubes are concentric: each carriage holds a tube that runs forward into the tube of the
carriage in front. Between two carriages that tube is unsupported. At home the span is the
largest that is safe; if a carriage gets further ahead of the one behind it, the slender
tube hangs over a longer distance and buckles. So carriages must move **together** whenever
a span would grow (decision D8, D9). In positions (each from its own zero): T1 ≤ T2 ≤ T3.

### The procedure (`homing.cpp`, v2.3.3)
**Part 1 — fast, all together.** All carriages still searching move backward in **one
coordinated move** at 15 mm/s (equal distances and one speed limit = identical speeds), so
every gap stays the same. The switches are *expected* in the safety manager. When any of
them reaches its switch (`switch reached (fast)`), the whole move stops and the others
continue together. Two carriages can only close in once the one behind has stopped at its
block; since there is a gap between all carriages at home, the one in front reaches its own
block first, so the dual-role switch is always pressed by the block.

**Part 2 — slow, one at a time** (T3, T2, T1), only a few mm each:
1. **Clear** — forward until the switch releases, then 2 mm further.
2. **Slow seek and zero** — backward at 5 mm/s; the pass in which the debounced switch
   reads pressed, the move is stopped and the step counter set to 0. Debounce (5 ms) at
   5 mm/s = 0.025 mm. Only this approach sets the zero, so part 1 can be fast.
3. **Clear** — forward until the switch releases (`switch released at`), +1 mm.
4. If a carriage's switch is *not* pressed at the start of part 2 (e.g. it was pressed by
   the carriage behind when homing started), that carriage first does its own fast approach.

State machine: `Phase` Group / Clear / Extra / Seek, plus `stage` (0 = needs its own fast
approach, 1 = slow approach next, 2 = zero set), advanced by `homing::update()` every loop
pass, right after `limit_switches::update()` so a switch edge stops the move in the same pass.
Nothing waits, so the safety manager, the encoder and the console keep working.
History: v2.1 one carriage at a time, slow only; v2.3.1 one at a time, two speeds.

### How homing fits the other modules
- **State machine:** `home` requests HOMING (allowed from IDLE and READY). The end result is
  READY if all three axes are homed, IDLE if only some are (`home 4` alone).
- **Safety:** homing's own moves set `safety::setHomingMove(true)`: they may exceed the
  50 mm "not homed" limit, and they are the only moves allowed while HOMING. Any switch
  other than the expected one still causes a FAULT.
- **Aborting:** every `motion::stopAll()` (command `stop`, encoder press, fault) increments
  `motion::stopCount()`. Homing remembers the count when it starts; if it changes, homing
  aborts (`EV home aborted`) and returns to IDLE (unless already in FAULT).
- **Quiet moves:** homing calls `motion::moveGroup(..., quiet=true)` / `moveRelative`, which
  suppress the `OK mv` and `EV done` lines, because an `OK` line must only ever answer a
  command. `motion::stopGroupOf(axis)` stops just homing's move when a switch is found.
- **Failure:** switch not found within 1000 mm (practically unlimited: only a broken switch
  or cable), or a switch that does not release within 10 mm -> `EV home failed ...` and FAULT.
- `home 2|4|6` homes one carriage alone (for testing); it can briefly break the span rule.

### After homing
- Per-axis homed flags (`homing::axisHomed(i)`, `homed=` in `st` and the stream).
- The span rule (T1 ≤ T2 ≤ T3 + 1 mm) applies as soon as two neighbours are homed.
- A homed axis cannot be sent below `TRA_SOFT_MIN_MM` (1.0 mm): at 0 the switch is
  pressed, and just behind it is the block.
- The 50 mm single-move limit applies until `calib` has measured the upper limits.
- Disabling a driver (`dis`) discards that axis' zero (the carriage could be moved by hand).

## 9b. Calibration and travel limits (Steps 2.2 + 2.3, decision D9)

### Why
After homing the robot knows each carriage's zero, but not how far forward it may go. T1
has a far switch. T2 and T3 can only be stopped by the carriage in front: switch 38 on the
back of T1 is pressed by T1's block **or** by T2 touching T1; switch 40 on the back of T2 by
T2's block **or** by T3. These switches are a backup only: if T1 rests on its block, switch
38 is already pressed, and T2 touching T1 gives no new edge, so nothing would stop T2.

### What `calib` measures (`calib.cpp`)
A list of operations built from the hardware profile (`CARRIAGE_ORDER` front to back) and
run one by one from `calib::update()`, like homing:

| Op | Moving | Until | Result |
|---|---|---|---|
| Approach 15 → 5 mm/s | T1 + T2 + T3 together | far switch 39 | `front_max` = T1 position; T1 stays there |
| Approach 10 → 3 mm/s | T2 + T3 together | switch 38 (T2 touches T1) | `gap1` = T2 − T1 |
| MOVETO 50 mm | all three together | | park, so T3 is measured well inside its range |
| Approach 10 → 3 mm/s | T3 alone (span only shrinks) | switch 40 (T3 touches T2) | `gap2` = T3 − T2 (contact at ~400 mm) |
| MOVETO 2 mm, 20 mm/s | all three in one coordinated move | | |

Each "approach" is four operations: SEEK fast (the group), then for the measured carriage
only: CLEAR (release + 2 mm), SEEK slow (records), CLEAR (release + 5 mm, `CALIB_SEPARATE_MM`, so a push of the
next carriage cannot reach the far switch). These short refinements are the only moves of
one carriage alone.
History: v2.3.0 parked carriages at 30 mm and measured one at a time; v2.3.2 (Raghav's
procedure) let each carriage stay at its maximum; v2.3.3 moves carriages together because
of the tube constraint; v2.3.5 parks all three at 50 mm before T3 is measured, because at T3's
full extension (~615 mm) other switches were disturbed (39 and 42 hit, twice at that moment). Robot so far: front_max 151.19 / 151.37 mm, gap T1/T2 118.03 / 118.07 / 117.84 mm,
gap T2/T3 349.13 mm.

Why "gap at contact" works: every carriage's position is measured from its own zero. When
T2 touches T1, `T2 − T1` is the same number wherever along the rail it happens: it is the
gap the two have when both are at zero. So one measurement describes contact everywhere.

### The rules (`limits.cpp`)
Between each carriage and the one behind it:
- **Span**: `front <= back + 1 mm` — never further apart than at home. Applied by the
  *sequences* (homing and calib move the carriages together); not applied to normal moves
  since v2.5.1 (`SPAN_RULE_FOR_MOVES = false`): manual moves are short and may move each
  tube individually.
- **Contact** (needs calib): `back <= front + gap − margin − extra` (margin 2 mm; extra per
  carriage from `CARRIAGE_EXTRA_MARGIN_MM`: T3 50 mm, because at its full range it disturbed
  the front of the robot).
- Front carriage: `position <= front_max − margin`.

For normal moves each carriage is therefore limited by the far end or by the carriage in
front (contact), plus the soft minimum of 1 mm.
- Checked by `safety::checkGroup()` inside `motion::moveGroup()`, on the **end positions**
  of the move. Coordinated moves are proportional, so each gap changes linearly between start
  and end: if it is fine at both ends it is fine all the way.
- A carriage whose neighbour is busy in a *different* move cannot be checked (the neighbour's
  end is unknown), so that is refused: combine them in one `mv`.
- Homing and calib bypass the rules (`setHomingMove`), because they drive to switches on purpose.
- Until limits are active (calibrated + all three homed), the 50 mm per-move limit applies.
- **Known values:** `hw_profile_v1.h` holds the values measured on the robot
  (`CALIB_DEFAULT_FRONT_MAX_MM`, `CALIB_DEFAULT_GAP_MM`); `limits::begin()` loads them at
  startup, so the limits are active right after `home`. `calib` re-measures; a value more
  than 5 mm from the profile value is rejected (lost steps on V1), and `calib` always ends
  with an automatic `home` so positions are right afterwards (v2.3.6).
- `limits` prints the measured values, `source=profile|calib`, and `range_now`: the lowest..highest position each
  carriage could move to on its own right now.

## 9c. Rotation zero, parking and EEPROM (Steps 2.4 + 2.5, decision D10)

### Rotation zero (`rotation_zero.cpp`)
The rotation axes have no sensor, so their zero is set by hand: turn a tube with `move`
until it is aligned (e.g. a mark), then `zero <axis>` (or `zero all`) makes that angle 0.
The rotations turn freely, so there is no limit; the zero only gives a repeatable start
orientation for demos and for record & replay.

### Parking
`park` builds one coordinated move: every referenced rotation to the nearest full turn of
its zero (`round(pos / 360) * 360`, the shortest way) and every homed carriage to 2 mm. The
move goes through the normal limit checks. When it is done, the rotation positions are set
to exactly 0 and the "parked" flag is written to EEPROM: the robot can be switched off.
At the next start `rotzero::begin()` reads the flag: parked -> rotation zeros restored (a
switched-off stepper stays where it is, at most settling 0.9 deg to the nearest full step);
not parked -> zeros unknown. The first rotation move clears the flag again.

### EEPROM (`settings.cpp`)
One small record at address 0: magic number, version, the last accepted calib result,
the parked flag and a checksum. `EEPROM.put()` only writes bytes that changed. Read once in
`setup()` before `limits::begin()` and `rotzero::begin()`. Empty, old or corrupt data is
ignored (profile values, rotation unknown). A stored calib result is used only if it is
within 5 mm of the profile values. Writes happen only on calib, park and the first rotation
move after a park. Commands: `settings` (show), `forget` (erase the stored calib result).

## 10. Development and test method

- Each step: design explained -> code -> compiled -> simulator (where possible) -> robot test
  with pass criteria -> devlog entry + git commit.
- **Simulator.** The firmware is also built for `simavr` (an ATmega2560 simulator) with
  `-DSIM_FAKE_DRIVERS` (pretends the TMC2209s answer). A harness sends serial commands at
  set times, can drive input pins (e.g. press a switch) and measures the exact cycle count
  of the step interrupt. Used to measure interrupt cost (85 % -> 45 %), verify coordinated
  timing and proportionality, and test fault, encoder stop and watchdog before the robot.
- **Runtime diagnostic commands** (`chop`, `ms`, `cur`) allow testing driver settings
  without reflashing; this is how the StealthChop slip was diagnosed in minutes.

## 11. Console commands (firmware 2.5.0)
```
help  status  state  ack  safety  wdtest  inputs  watch on|off  led g|b|c|r|off [slow|fast]
drivers  move <1-6> <amount> [speed]  mv <axis>=<amount> ... [t=<s>] [v=<%>]
stop  pos  en <1-6|all>  dis <1-6|all>  isr reset  home [2|4|6]  calib  limits
zero <1|3|5|all>  park  settings  forget
info  ping  st  stream <ms>|off          (protocol, Step 1.6; prefix any command with #<id>)
chop <1-6> stealth|spread|hybrid [speed]  ms <1-6> <2..32>  cur <1-6> <mA>
```
Axes: 1 T1-Rot, 2 T1-Tra, 3 T2-Rot, 4 T2-Tra, 5 T3-Rot, 6 T3-Tra.
Units: translation mm and mm/s; rotation degrees and rpm.
Order front to back: T1, T2, T3. Moving forward, T2 approaches T1 and T3 approaches T2;
moving backward, T1 approaches T2 and T2 approaches T3. Contact is prevented by the limits
(switches 38/40 also detect contact; the software limits of step 2.3 prevent it).

## 12. Open items
- Longest loop pass on the robot is ~9 ms, caused by the driver health check (two blocking
  TMCStepper UART reads, ~4.5 ms each), not by printing. Accepted for the prototype and made
  rare: once per second (decision D7). A non-blocking UART reader would remove it.
- Building a long line still costs CPU (~10 us per character): with all six axes at maximum
  speed the simulator shows single passes of 3–8 ms when a move starts or ends.
- With all six axes at maximum speed the step interrupt uses most of the CPU (simulator:
  ~900 loop passes/s). Fine for now; revisit if demos need all axes at full speed together.
- Rotation zero is kept across power cycles only if the robot was parked (no sensor).
- Homing from far away is slow (up to ~30 s per carriage at 5 mm/s). Option: faster
  approach, then a slow final approach.
