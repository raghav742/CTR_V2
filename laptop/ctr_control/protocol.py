"""
Serial protocol v1 of the CTR V2 firmware (docs/protocol.md), laptop side.

Every line from the robot starts with a type word:
    OK / ERR   reply to a command        (may start with "#<id> ")
    EV         event (something happened)
    ST         status stream line
    other      banner / human listings
The link runs a background thread that reads lines; the GUI collects them with
poll() from its own thread (Qt widgets must only be touched by the GUI thread).
"""
import queue
import re
import threading
import time
from dataclasses import dataclass, field

BAUD = 250000
_ID_RE = re.compile(r"^#(\d+) (.*)$")


@dataclass
class Line:
    kind: str            # OK, ERR, EV, ST or TXT
    text: str            # line without the "#id " tag
    id: int = None       # request id if the line was tagged
    raw: str = ""
    t: float = field(default_factory=time.time)


def parse_line(raw: str) -> Line:
    s = raw.strip()
    rid = None
    m = _ID_RE.match(s)
    if m:
        rid, s = int(m.group(1)), m.group(2)
    word = s.split(" ", 1)[0]
    kind = word if word in ("OK", "ERR", "EV", "ST") else "TXT"
    return Line(kind, s, rid, raw.rstrip("\r\n"))


def parse_kv(text: str) -> dict:
    """key=value pairs of a line; values in quotes may contain spaces."""
    out = {}
    for m in re.finditer(r'(\w[\w/\-]*)=("([^"]*)"|\S+)', text):
        out[m.group(1)] = m.group(3) if m.group(3) is not None else m.group(2)
    return out


def parse_status(text: str):
    """Fields of an ST line or an 'OK t=...' reply to 'st'.
    Returns dict(t, state, busy[list of bool], pos[list of float], homed[list of bool])."""
    kv = parse_kv(text)
    if "pos" not in kv:
        return None
    pos = [float(v) for v in kv["pos"].split(",")]
    busy = [c == "1" for c in kv.get("busy", "")]
    homed = [c == "1" for c in kv.get("homed", "")]
    return {"t": int(kv.get("t", 0)), "state": kv.get("state", "?"),
            "busy": busy, "pos": pos, "homed": homed}


class SerialTransport:
    """USB serial port (pyserial). Opening the port resets the Arduino."""

    def __init__(self, port: str, baud: int = BAUD):
        import serial
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.name = port

    def readline(self) -> str:
        b = self.ser.readline()
        return b.decode("ascii", errors="replace") if b else ""

    def write(self, text: str):
        self.ser.write(text.encode("ascii", errors="replace"))

    def close(self):
        self.ser.close()


def list_ports():
    try:
        from serial.tools import list_ports as lp
        return [(p.device, p.description) for p in lp.comports()]
    except Exception:
        return []


class RobotLink:
    """Reads lines in a background thread; sends commands with request ids."""

    def __init__(self, transport, log=None):
        self.tr = transport
        self.log = log                      # callable(direction, text) or None
        self.q = queue.Queue()
        self._next_id = 1
        self._lock = threading.Lock()
        self.pending = {}                   # id -> (command, time sent)
        self._run = True
        self._th = threading.Thread(target=self._reader, daemon=True)
        self._th.start()

    def _reader(self):
        while self._run:
            try:
                raw = self.tr.readline()
            except Exception as e:          # cable pulled, port gone
                self.q.put(Line("ERR", f"ERR link lost: {e}", raw=""))
                self._run = False
                break
            if raw and raw.strip():
                if self.log:
                    self.log("<", raw.rstrip("\r\n"))
                self.q.put(parse_line(raw))

    def send(self, cmd: str, tagged: bool = True) -> int:
        """Send a command; returns its request id (0 if untagged)."""
        with self._lock:
            rid = self._next_id if tagged else 0
            if tagged:
                self._next_id += 1
                self.pending[rid] = (cmd, time.time())
        line = f"#{rid} {cmd}" if tagged else cmd
        if self.log:
            self.log(">", line)
        self.tr.write(line + "\n")
        return rid

    def poll(self):
        """All lines received since the last call (call from the GUI thread)."""
        out = []
        while True:
            try:
                ln = self.q.get_nowait()
            except queue.Empty:
                break
            if ln.id is not None and ln.kind in ("OK", "ERR"):
                self.pending.pop(ln.id, None)
            out.append(ln)
        return out

    def close(self):
        self._run = False
        try:
            self.tr.close()
        except Exception:
            pass
