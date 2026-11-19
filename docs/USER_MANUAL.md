# CTR V2 — User manual

How to operate the Concentric Tube Robot demonstrator. Written for the V1 robot with firmware
5.0; on V2 everything works the same, with a larger screen and a joystick (marked **V2**).

---

## Safety first

- **A press on the knob always stops the robot** while anything moves — in every screen and
  every mode. The STOP button in the laptop program (or the Esc key) does the same.
- Keep hands away from the carriages and tubes while the robot moves.
- If something looks wrong, press the knob, then switch off the main power.
- The robot only knows where the carriages are after **homing**. Before homing, moves are
  limited to 50 mm and the travel limits are not active.

---

## Controls

| Action | Effect |
|---|---|
| Turn the knob | Choose a menu item / change a value |
| Short press | Select (on release) |
| Long press (1 s) | Back |
| Press while moving | **STOP** |
| **V2:** joystick (in the Jog screen) | Up/down: move the selected tube in/out; left/right: rotate it |

**Status LED** (on the knob)

| LED | Meaning |
|---|---|
| Green, blinking | Running, not homed |
| Cyan, blinking | Homing / calibrating |
| Green | Homed, ready |
| Blue | Manual control (jog) |
| Blue, blinking | Demo, attract mode or visitor mode |
| Cyan | Controlled from the laptop |
| Fast blinking (cyan on V1, red on V2) | **Fault** — see the screen |

On V2 the LED has no blue: blue shows as yellow (unless the LED was rewired).

---

## Starting up

1. Switch on the main power. The robot beeps, shows the start screen, then the menu.
2. If the robot was **parked** before it was switched off, the rotation zeros are restored.
   Otherwise the angles show `?` until they are set (see *Rotation zero*).
3. Choose **Home**. All carriages move backward together until they reach their switches,
   then each one approaches its switch slowly. The status LED turns green when done
   (about 10–15 s).

---

## Main menu

| Item | What it does |
|---|---|
| **Demo** | Choose a demonstration: *Telescope*, *Wave* or *Spin*. It runs in a loop until you press the knob. Needs homing first. |
| **Jog** | Move one axis by hand. Choose the axis, then turn the knob: each click moves one step. A short press changes the step size (0.1 / 1 / 5 mm or 1 / 10 / 45°). A carriage stops exactly at its limit (beep, *LIMIT*). **V2:** the joystick moves the selected tube continuously while held. |
| **Record** | *Add pose* stores the current position (up to 10). *Play* moves through the stored poses in a loop. *Clear all* deletes them. Poses are kept after power-off. |
| **Home** | Homing (see above). |
| **Status** | Positions of all tubes. `?` = not homed / no rotation zero, `*` = moving. |
| **Safety demo** | The tubes turn slowly; a visitor presses the knob and the screen shows how fast the robot stopped (about 21 ms). |
| **Service** | Settings and maintenance (below). |

**Service menu**

| Item | What it does |
|---|---|
| Calibrate limits | Measures how far the carriages may travel (switched off on V1, which uses stored values) |
| Zero rotations | The current angle of every tube becomes 0 (align the tubes by eye first) |
| Park (power off) | Turns the tubes to their zero and moves the carriages to rest; then switch off |
| Driver scan | Checks the six motor drivers |
| Sound on/off | Buzzer |
| Attract on/off | When on, a demo starts by itself after 3 minutes without input; any touch stops it. Off after every power-on. |
| Visitor mode | Short menu for visitors (Demo, Safety demo, Status). **To leave: hold the knob for 5 seconds.** |

---

## Rotation zero

The rotations have no sensor. To set the zero: use **Jog** to turn each tube to its marked
position, then **Service → Zero rotations**. Always **Park** before switching off — then the
zero is still valid at the next start.

---

## Switching off

**Service → Park (power off)**, wait until it is done, then switch off the main power.

---

## Faults

If the robot detects a problem (an unexpected limit switch, a motor driver not answering or
too hot), it stops, shows an error screen with the reason and plays an error sound. Remove
the cause, then **press the knob** to confirm. Home again if the carriages may have moved.

| Problem | What to do |
|---|---|
| "T1-far hit" (or another switch name + "hit") | A carriage touched a switch unexpectedly. Check nothing is blocked; confirm, then Home. |
| "… driver not answering" / "… driver overheated" | Check motor power and cables, let it cool down; Service → Driver scan. |
| "… home: switch not found" | Homing could not find a switch: check the switch and its cable. |
| A move or demo is refused | Usually not homed, or the move would pass a limit. Home first. |
| Screen dark (V1 OLED) | Check the display cable (I2C SDA 20 / SCL 21). |
| Robot stops when the laptop program closes | Normal: the heartbeat stops motion if the laptop goes silent. |

---

## Laptop control centre (optional)

The robot works without a laptop. With one, start `laptop/run_control_centre.bat`, choose the
robot's COM port and **Connect**. You get a STOP button, Home / Zero / Park buttons, demos,
jog, live positions and plots, a command line, and CSV recording of measurements. Details:
`laptop/README.md`. Serial commands for experts: type `help` in the command line.
