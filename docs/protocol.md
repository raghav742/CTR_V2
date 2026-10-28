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
| `st` | `OK t=<ms> state=<STATE> busy=<6 x 0/1> pos=<p1>,...,<p6>` |
| `stream <ms>` / `stream off` / `stream` | Start (20–10000 ms), stop, or show the status stream |
| `status` | firmware, profile, state, loops/s, maxloop_us, isr_max_us, uptime_s, st_skipped |

Status line (stream):
```
ST t=1249 state=READY busy=110000 pos=157.95,13.17,0.00,0.00,0.00,0.00
```
- `t` milliseconds since start (for plotting, and to detect a restart).
- `busy` one digit per axis in axis order, 1 = moving.
- `pos` positions in axis order, in the units from `info` (mm or degrees, 2 decimals).

If the USB send buffer does not have room for a whole `ST` line, that line is skipped
(and counted in `st_skipped`) instead of making the robot wait. A program must therefore
not assume every period produces a line; use `t`.

## All commands
See `help` on the robot, or `docs/architecture.md` section 10. Axes are numbered 1–6 in
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

## Rules for changes
- New keys may be added at the end of a reply; existing keys keep their meaning.
- Removing or renaming a key, or changing a line type, increases `PROTO_VERSION`
  (`include/version.h`).
