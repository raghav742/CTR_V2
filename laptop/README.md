# CTR V2 Control Centre (laptop)

Desktop app to operate and monitor the CTR V2 robot over USB, using the serial
protocol of the firmware (`../docs/protocol.md`). The robot works without it; the
laptop is an extra control and monitoring station.

## Install (once)
1. Install Python 3.10 or newer from https://www.python.org (tick "Add python.exe to PATH").
2. In a terminal, in this `laptop` folder:
   ```
   pip install -r requirements.txt
   ```

## Start
- Double-click `run_control_centre.bat`, or
- in a terminal in this folder: `python -m ctr_control`

Close any serial monitor (PlatformIO / Arduino IDE) first: only one program can use the port.

## Use
- **Port:** choose the Arduino's COM port, or **Fake robot (no hardware)** to try the app
  without the robot. Then **Connect**. Connecting restarts the Arduino (normal); the app
  waits for its start message, asks `info` and starts the status stream (10 lines/s).
- **STOP** (or the **Esc** key anywhere in the window) stops all motion.
- **Robot:** Home, Acknowledge fault, Zero rotations, Park, Status, Limits.
- **Demo:** choose a program, Run / Stop demo.
- **Jog:** axis, step (mm or deg), − / +.
- **Command line:** any console command (`help` lists them); ↑ / ↓ for history.
- **Plots:** translation and rotation of the three tubes over the last 30 s.
- **Link indicator** (top bar): "link OK · <ms>" = round-trip time of the pings; red
  "link: NO REPLY" if the robot does not answer. While connected the robot watches this link
  (heartbeat): if the laptop goes silent for 1.5 s while the robot moves, the robot stops.
  **Disconnect** switches the watch off, so a demo started from the laptop then keeps running.
- **Measurement (CSV):** choose a rate (10/20/50 per second), optionally *Excel NL/DE format*
  (`;` between columns, `,` as decimal point; needed for Excel with Dutch or German settings),
  then **Record**. Every status line and every event (stops, reaction times, faults, demo
  start/stop) is written to `recordings/rec_<date>_<time>.csv`; **Stop recording** closes it.
  Columns: `pc_time_s, robot_ms, state, T1-Rot … T3-Tra, busy, event`.
- **Log:** every line sent and received is written to `logs/session_<date>_<time>.log`
  (pings excluded).

## Files
| File | Content |
|---|---|
| `ctr_control/app.py` | Main window |
| `ctr_control/protocol.py` | Serial link: line parsing (OK/ERR/EV/ST), request ids, background reader |
| `ctr_control/fake_robot.py` | Simulated robot with the same protocol (no hardware needed) |
| `ctr_control/recorder.py` | CSV recording of status stream and events |
| `requirements.txt` | Python packages: PySide6, pyqtgraph, pyserial |
