# Serial protocol v1

The Arduino is always in control. A laptop (Serial Monitor or the phase 4 control
program) connects over USB to send commands, read replies, follow events and stream
status. The same protocol is used by humans and programs: every line is plain text.

## Connection
| Setting | Value |
|---|---|
| Baud rate | 250000 (exact on a 16 MHz Mega; 115200 is 2 % off) |
| Line ending sent to the robot | LF (`\n`) or CR (`\r`); both accepted |
| Max command length | 95 characters |
| Protocol version | `proto=1` in the `info` reply |

Opening the port resets the Mega (standard Arduino behaviour). A program waits for the
`EV boot` line, then sends `info`.

## Line types (first word of each line from the robot)
| Prefix | Meaning | When |
|---|---|---|
| `OK` | Command succeeded; data follows as `key=value` | Reply to a command |
| `ERR` | Command refused or wrong; human-readable reason follows | Reply to a command |
| `EV` | Event: something happened (state change, move done, stop, fault, boot) | Any time |
| `ST` | Status line from the stream | Every `<ms>` while `stream` is on |
| other | Startup banner and the `help` / `drivers` listings | For humans |

Every command produces exactly one `OK` or `ERR` line (multi-line `help` / `drivers`
listings start with an `OK` line).

## Request ids
Put `#<number>` before a command, and every line printed while that command runs
starts with `#<number> `:
```
> #12 pos
< #12 OK T1-Rot=0.00 T1-Tra=0.00 ...
> #13 stop
< #13 EV stopped t=0.89s ...        (events caused by the command carry the id too)
< #13 EV reaction_ms=3.8
< #13 OK stop
```
Lines without an id (events, `ST`) are not replies. A program can send a command, then
wait for the `OK`/`ERR` line with its id, ignoring everything else in between.

## Commands for programs
| Command | Reply |
|---|---|
| `info` | `OK info proto=1 fw=1.6.0 profile="..." axes=T1-Rot:deg,T1-Tra:mm,... switches=T1-home,...` |
| `ping` | `OK pong` (link check; basis of the heartbeat in step 4.4) |
| `st` | `OK t=<ms> state=<STATE> busy=<6 x 0/1> pos=<p1>,...,<p6> homed=<6 x 0/1>` |
| `stream <ms>` / `stream off` / `stream` | Start (20–10000 ms), stop, or show the status stream |
| `status` | firmware, profile, state, loops/s, maxloop_us, isr_max_us, uptime_s, st_skipped |
| `settings` | `OK settings eeprom=valid|empty calib=<0/1> [front_max=.. gap1=.. gap2=..] parked=<0/1>` |
| `zero <1|3|5|all>` / `park` / `forget` | rotation zero, park before power-off, erase stored calib |
| `limits` | `OK limits calibrated=<0/1> source=profile|calib|eeprom active=<0/1> front_max=.. gap_<A>/<B>=.. margin=.. extra=<mm per carriage> span_tol=.. range_now=<axis>:<lo>..<hi>,...` (`?` = no limit known) |

Status line (stream):
```
ST t=1249 state=READY busy=110000 pos=157.95,13.17,0.00,0.00,0.00,0.00 homed=010101
```
- `t` milliseconds since start (for plotting, and to detect a restart).
- `busy` one digit per axis in axis order, 1 = moving.
- `pos` positions in axis order, in the units from `info` (mm or degrees, 2 decimals).
- `homed` one digit per axis, 1 = position is measured from a real zero (translation: homed,
  fw 2.1+; rotation: zero set or restored after parking, fw 2.5+).

If the USB send buffer does not have room for a whole `ST` line, that line is skipped
(and counted in `st_skipped`) instead of making the robot wait. A program must therefore
not assume every period produces a line; use `t`.

## All commands
See `help` on the robot, or `docs/architecture.md` section 11. Axes are numbered 1–6 in
the order of `info`. Units: translation mm and mm/s; rotation degrees and rpm.

## Events
| Event | Meaning |
|---|---|
| `EV boot reset_cause=<cause>` | Startup finished; cause = power-on, reset button / upload, watchdog, brown-out |
| `EV state <FROM> -> <TO> (<why>)` | State machine changed state |
| `EV done t=<s> <axis>=<pos> ...` | Move finished at its target |
| `EV stopped t=<s> <axis>=<pos> ...` | Move stopped early |
| `EV stop <reason>` | All motion stopped (command, encoder button, ...) |
| `EV reaction_ms=<ms>` | Time from the stop input to motors stopped |
| `EV enc`, `EV btn`, `EV switch` | Input events (only with `watch on`) |
| `EV home <axis> switch reached (fast) at <mm> mm` | Fast approach (all together) reached this carriage's switch |
| `EV home <axis> zero set (was at <mm> mm)` | Slow approach found the switch; position set to 0 (old counted position shown) |
| `EV home <axis> switch released at <mm> mm` | Where the switch lets go when moving forward again |
| `EV home <axis> ok pos=<mm>` | Axis homed, resting just in front of its switch |
| `EV home done t=<s>` | All requested axes homed |
| `EV home aborted <axis>` | Homing stopped by `stop`, encoder or a fault |
| `EV home failed <axis> home: <why>` | Switch not found / stays pressed; followed by FAULT |
| `EV home <axis> lost (zero no longer valid)` | Zero discarded (driver disabled) |
| `EV calib <axis> switch at <mm> mm (fast), approaching slowly` | Fast approach hit; slow approach follows |
| `EV calib <axis> far switch at <mm> mm` | Front carriage reached its far switch |
| `EV calib <axis> touches <axis> at <mm> mm (<axis> at <mm>) gap=<mm>` | Carriage touched the one in front; gap at home |
| `EV calib reach <axis>=<mm> ...` | Furthest each carriage can physically go (far end + gaps), before margins |
| `EV calib measured t=<s> front_max=<mm> gap1=<mm> gap2=<mm>` | All values measured |
| `EV calib rejected <name>=<mm> expected <mm> (<dev> mm, lost steps?)` | Value too far from the profile value |
| `EV calib stored` / `EV calib not stored (previous values kept)` | Result of the check |
| `EV settings none stored (using profile values)` | EEPROM empty or invalid at startup |
| `EV rotation zero restored (robot was parked)` / `EV rotation zero unknown ...` | Startup: rotation zeros |
| `EV park done: safe to power off ...` / `EV park aborted` | Result of `park` |
| `EV calib re-homing` | calib finished; automatic `home` follows (its EV home lines) |
| `EV calib aborted (previous values kept)` | Stopped by `stop`, encoder or fault |
| `EV calib failed calib <axis>: <why>` | Switch not found / implausible gap; followed by FAULT |

## Rules for changes
- New keys may be added at the end of a reply; existing keys keep their meaning.
- Removing or renaming a key, or changing a line type, increases `PROTO_VERSION`
  (`include/version.h`).
