"""
Fake robot: behaves like the CTR V2 firmware on the serial protocol, so the
control centre can be developed and shown without hardware.
Simplified: translations at 10 mm/s, rotations at 100 rpm (V1 caps), homing
takes a few seconds, demos move the tubes in a loop, limits are approximate.
"""
import math
import queue
import re
import threading
import time

AXES = ["T1-Rot", "T1-Tra", "T2-Rot", "T2-Tra", "T3-Rot", "T3-Tra"]
UNITS = ["deg", "mm", "deg", "mm", "deg", "mm"]
TRA_SPEED = 10.0          # mm/s
ROT_SPEED = 600.0         # deg/s (100 rpm)
FRONT_MAX = 129.2         # T1 limit with the V1 margins
DEMOS = ["Telescope", "Wave", "Spin"]


class FakeRobot:
    name = "fake robot"

    def __init__(self):
        self.out = queue.Queue()
        self.pos = [0.0] * 6
        self.target = [0.0] * 6
        self.homed = [False] * 6
        self.state = "IDLE"
        self.stream_ms = 0
        self.t0 = time.time()
        self.demo = None            # (index, step, start time)
        self.home_until = 0
        self._tag = None
        self.hb_ms = 0
        self.last_cmd = time.time()
        self._lock = threading.Lock()
        self._run = True
        self._emit("CTR V2 firmware 4.3.0 | profile: FAKE robot (laptop simulation)")
        self._emit("Display: not found (fake robot)")
        self._emit("Type 'help' for commands.")
        self._emit("EV boot reset_cause=power-on")
        self._emit("EV state BOOT -> IDLE (startup complete)")
        threading.Thread(target=self._sim, daemon=True).start()

    # ---------------- transport interface ----------------
    def readline(self):
        try:
            return self.out.get(timeout=0.1) + "\n"
        except queue.Empty:
            return ""

    def write(self, text):
        for line in text.splitlines():
            if line.strip():
                with self._lock:
                    self._command(line.strip())

    def close(self):
        self._run = False

    # ---------------- helpers ----------------
    def _ms(self):
        return int((time.time() - self.t0) * 1000)

    def _emit(self, s):
        self.out.put((f"#{self._tag} " if self._tag is not None else "") + s)

    def _busy(self, i):
        return abs(self.pos[i] - self.target[i]) > 1e-3

    def _status_fields(self):
        busy = "".join("1" if self._busy(i) else "0" for i in range(6))
        pos = ",".join(f"{p:.2f}" for p in self.pos)
        homed = "".join("1" if h else "0" for h in self.homed)
        return f"t={self._ms()} state={self.state} busy={busy} pos={pos} homed={homed}"

    def _set_state(self, new, why=None):
        if new != self.state:
            self._emit(f"EV state {self.state} -> {new}" + (f" ({why})" if why else ""))
            self.state = new

    def _limit_ok(self, i, value):
        if i % 2 == 0:
            return True                       # rotations are free
        if not self.homed[i]:
            return abs(value - self.pos[i]) <= 50
        if value < 1.0:
            return False
        if i == 1:
            return value <= FRONT_MAX
        if i == 3:
            return value <= self.target[1] + 117.8 - 22
        return value <= self.target[3] + 349.1 - 52

    # ---------------- commands ----------------
    def _command(self, line):
        self.last_cmd = time.time()
        m = re.match(r"^#(\d+) (.*)$", line)
        self._tag = int(m.group(1)) if m else None
        cmd = (m.group(2) if m else line).split()
        try:
            self._dispatch(cmd[0].lower(), cmd[1:])
        except Exception as e:
            self._emit(f"ERR {e}")
        self._tag = None

    def _dispatch(self, c, a):
        if c == "info":
            axes = ",".join(f"{n}:{u}" for n, u in zip(AXES, UNITS))
            self._emit(f'OK info proto=1 fw=4.3.0 profile="FAKE robot" axes={axes} '
                       "switches=T1-home,T1-far,T2-home,T3-home")
        elif c == "ping":
            self._emit("OK pong")
        elif c == "hb":
            self.hb_ms = int(a[0])
            self._emit(f"OK hb={self.hb_ms}")
        elif c == "st":
            self._emit("OK " + self._status_fields())
        elif c == "stream":
            self.stream_ms = 0 if (a and a[0] == "off") else int(a[0]) if a else self.stream_ms
            self._emit(f"OK stream={self.stream_ms}")
        elif c == "status":
            self._emit(f'OK fw=4.3.0 profile="FAKE robot" state={self.state} loops/s=8700 '
                       f"maxloop_us=900 isr_max_us=30.0 uptime_s={self._ms() // 1000} "
                       "st_skipped=0 free_ram=2300")
        elif c == "state":
            self._emit(f"OK state={self.state} homed={1 if all(self.homed[1::2]) else 0}")
        elif c == "pos":
            self._emit("OK " + " ".join(f"{n}={p:.2f}{'*' if self._busy(i) else ''}"
                                        for i, (n, p) in enumerate(zip(AXES, self.pos))))
        elif c == "home":
            self._set_state("HOMING")
            self._emit("OK home T3-Tra,T2-Tra,T1-Tra")
            for i in (1, 3, 5):
                self.target[i] = 1.4
            self.home_until = time.time() + max(2.0, max(self.pos[1::2]) / TRA_SPEED)
        elif c in ("move", "mv"):
            amounts = {}
            if c == "move":
                amounts[int(a[0]) - 1] = float(a[1])
            else:
                for kv in a:
                    k, v = kv.split("=")
                    if k.isdigit():
                        amounts[int(k) - 1] = float(v)
            if self.state in ("FAULT", "HOMING", "BOOT"):
                self._emit(f"ERR not allowed in state {self.state}")
                return
            for i, d in amounts.items():
                if not self._limit_ok(i, self.target[i] + d):
                    self._emit(f"ERR {AXES[i]} would end at {self.target[i] + d:.2f} {UNITS[i]}, limit (fake)")
                    return
            for i, d in amounts.items():
                self.target[i] += d
            self._emit("OK mv " + " ".join(f"{AXES[i]}={d:.2f}{UNITS[i]}" for i, d in amounts.items()))
        elif c == "stop":
            moving = any(self._busy(i) for i in range(6)) or self.demo is not None
            self.target = self.pos[:]
            if moving:
                self._emit("EV stop command")
                self._emit("EV reaction_ms=0.4")
            if self.demo is not None:
                self._emit("EV demo stopped (stop) loops=0")
                self.demo = None
                self._set_state("READY")
            self._emit("OK stop")
        elif c == "demo":
            if not a:
                self._emit("OK demos " + " ".join(f"{i + 1}={n}" for i, n in enumerate(DEMOS)))
            elif a[0] == "stop":
                self._dispatch("stop", [])
            else:
                if not all(self.homed[1::2]):
                    self._emit("ERR home first (limits not active)")
                    return
                n = int(a[0]) - 1
                self._set_state("DEMO")
                self._emit(f"OK demo {DEMOS[n]}")
                self._emit(f"EV demo start {DEMOS[n]}")
                self.demo = (n, time.time())
        elif c == "park":
            for i in range(6):
                self.target[i] = 2.0 if i % 2 else round(self.pos[i] / 360) * 360
            self._emit("OK mv (park)")
        elif c == "zero":
            for i in (0, 2, 4):
                self.pos[i] = self.target[i] = 0.0
            self._emit("OK zero T1-Rot T2-Rot T3-Rot")
        elif c == "ack":
            if self.state == "FAULT":
                self._set_state("READY" if all(self.homed[1::2]) else "IDLE", "acknowledged")
                self._emit("OK acknowledged")
            else:
                self._emit("ERR no fault active")
        elif c == "limits":
            self._emit("OK limits calibrated=1 source=profile active=1 front_max=151.20 calib=off "
                       "speed_cap=10.0mm/s,100rpm (fake)")
        elif c == "help":
            self._emit("OK commands: (fake robot: info ping st stream status state pos home move mv "
                       "stop demo park zero ack limits)")
        else:
            self._emit(f"ERR unknown command: {c}")

    # ---------------- simulation ----------------
    def _sim(self):
        last = time.time()
        last_st = 0
        while self._run:
            time.sleep(0.02)
            now = time.time()
            dt = now - last
            last = now
            with self._lock:
                if self.demo is not None:
                    n, t0 = self.demo
                    ph = now - t0
                    w = [0.25, 0.6, 0.35][n]
                    base = 40 + 30 * math.sin(w * ph)
                    self.target[1] = base
                    self.target[3] = base + 40 + 25 * math.sin(w * ph + 1)
                    self.target[5] = self.target[3] + 60 + 50 * math.sin(w * ph + 2)
                    for k, i in enumerate((0, 2, 4)):
                        amp = [90, 45, 720][n]
                        self.target[i] = amp * math.sin(w * ph + k)
                for i in range(6):
                    v = (ROT_SPEED if i % 2 == 0 else TRA_SPEED) * dt
                    d = self.target[i] - self.pos[i]
                    self.pos[i] = self.target[i] if abs(d) <= v else self.pos[i] + math.copysign(v, d)
                if self.hb_ms and (now - self.last_cmd) * 1000 > self.hb_ms:
                    moving = any(self._busy(i) for i in range(6)) or self.demo is not None
                    if moving:
                        self.target = self.pos[:]
                        self.demo = None
                        self.out.put("EV stop heartbeat lost")
                        self.out.put(f"EV heartbeat lost: no command for {self.hb_ms} ms, motion stopped")
                        if self.state == "DEMO":
                            self.state = "READY"
                    else:
                        self.out.put("EV heartbeat lost: watch off")
                    self.hb_ms = 0
                if self.state == "HOMING" and now >= self.home_until:
                    for i in (1, 3, 5):
                        self.pos[i] = self.target[i] = 1.4
                        self.homed[i] = True
                        self._emit(f"EV home {AXES[i]} ok pos=1.40")
                    self._emit("EV home done t=3.0s")
                    self._set_state("READY")
                if self.stream_ms and self._ms() - last_st >= self.stream_ms:
                    last_st = self._ms()
                    self.out.put("ST " + self._status_fields())
