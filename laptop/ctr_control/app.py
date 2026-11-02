"""
CTR V2 control centre (phase 4): desktop window for the laptop.

    python -m ctr_control            (from the 'laptop' folder)

Connect to the robot over USB (or to the built-in fake robot), see the state
and positions live, plot them, send commands with buttons or by typing, and
keep a log file of everything that was sent and received.
"""
import collections
import os
import sys
import time

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont, QColor, QTextCursor
from PySide6.QtWidgets import (
    QApplication, QComboBox, QDoubleSpinBox, QGridLayout, QGroupBox, QHBoxLayout,
    QLabel, QLineEdit, QMainWindow, QMessageBox, QPlainTextEdit, QPushButton,
    QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget, QCheckBox, QHeaderView,
)
import pyqtgraph as pg

from . import __version__
from .protocol import RobotLink, SerialTransport, list_ports, parse_kv, parse_status
from .fake_robot import FakeRobot
from .recorder import Recorder

STREAM_MS = 100                 # status lines per second: 10
HEARTBEAT_MS = 1500             # robot stops motion if it hears nothing for this long (step 4.3)
PING_MS = 500                   # we send a ping this often
PLOT_SECONDS = 30
TUBE_COLOURS = ["#1f77b4", "#d62728", "#2ca02c"]   # T1, T2, T3
STATE_COLOURS = {"READY": "#2e7d32", "IDLE": "#757575", "HOMING": "#00838f",
                 "DEMO": "#1565c0", "FAULT": "#c62828"}


class Logger:
    """Session log: every line sent (>) and received (<) with a timestamp."""

    def __init__(self):
        folder = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs")
        os.makedirs(folder, exist_ok=True)
        self.path = os.path.join(folder, time.strftime("session_%Y%m%d_%H%M%S.log"))
        self.f = open(self.path, "a", encoding="utf-8", buffering=1)

    def __call__(self, direction, text):
        self.f.write(f"{time.strftime('%H:%M:%S')}.{int(time.time() * 1000) % 1000:03d} {direction} {text}\n")


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle(f"CTR V2 Control Centre {__version__}")
        self.resize(1200, 780)
        self.link = None
        self.logger = Logger()
        self.axes = ["T1-Rot", "T1-Tra", "T2-Rot", "T2-Tra", "T3-Rot", "T3-Tra"]
        self.units = ["deg", "mm"] * 3
        self.hist_t = collections.deque(maxlen=PLOT_SECONDS * 1000 // STREAM_MS)
        self.hist = [collections.deque(maxlen=self.hist_t.maxlen) for _ in range(6)]
        self.waiting_boot = False
        self._build()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(20)
        # Heartbeat: a ping every PING_MS keeps the robot's watch satisfied and
        # measures the round-trip time for the link indicator
        self.ping_timer = QTimer(self)
        self.ping_timer.timeout.connect(self._ping)
        self.ping_sent = {}             # request id -> time sent
        self.last_pong = 0.0
        self.rtt_ms = None
        self._set_connected(False)

    # ------------------------------------------------------------ layout
    def _build(self):
        root = QWidget()
        self.setCentralWidget(root)
        lay = QVBoxLayout(root)

        # connection bar
        bar = QHBoxLayout()
        self.port = QComboBox()
        self.port.setMinimumWidth(260)
        self._refresh_ports()
        btn_refresh = QPushButton("↻")
        btn_refresh.setFixedWidth(32)
        btn_refresh.clicked.connect(self._refresh_ports)
        self.btn_connect = QPushButton("Connect")
        self.btn_connect.clicked.connect(self._toggle_connect)
        self.lbl_robot = QLabel("not connected")
        self.lbl_link = QLabel("")
        self.lbl_link.setMinimumWidth(150)
        self.lbl_state = QLabel("—")
        self.lbl_state.setAlignment(Qt.AlignCenter)
        self.lbl_state.setMinimumWidth(110)
        f = QFont(); f.setBold(True); f.setPointSize(12)
        self.lbl_state.setFont(f)
        bar.addWidget(QLabel("Port:"))
        bar.addWidget(self.port)
        bar.addWidget(btn_refresh)
        bar.addWidget(self.btn_connect)
        bar.addSpacing(12)
        bar.addWidget(self.lbl_robot, 1)
        bar.addWidget(self.lbl_link)
        bar.addWidget(self.lbl_state)
        lay.addLayout(bar)

        mid = QHBoxLayout()
        lay.addLayout(mid, 1)

        # left: controls
        left = QVBoxLayout()
        mid.addLayout(left)

        self.btn_stop = QPushButton("STOP")
        self.btn_stop.setMinimumHeight(70)
        self.btn_stop.setStyleSheet("QPushButton{background:#c62828;color:white;font-size:22px;"
                                    "font-weight:bold;border-radius:8px}"
                                    "QPushButton:disabled{background:#e0a0a0}")
        self.btn_stop.clicked.connect(lambda: self.send("stop"))
        left.addWidget(self.btn_stop)

        g = QGroupBox("Robot")
        gl = QGridLayout(g)
        self.cmd_buttons = []
        for k, (txt, cmd) in enumerate([("Home", "home"), ("Acknowledge fault", "ack"),
                                        ("Zero rotations", "zero all"), ("Park (power off)", "park"),
                                        ("Status", "status"), ("Limits", "limits")]):
            b = QPushButton(txt)
            b.clicked.connect(lambda _=False, c=cmd: self.send(c))
            gl.addWidget(b, k // 2, k % 2)
            self.cmd_buttons.append(b)
        left.addWidget(g)

        g = QGroupBox("Demo")
        gl = QHBoxLayout(g)
        self.demo_box = QComboBox()
        self.demo_box.addItems(["1  Telescope", "2  Wave", "3  Spin"])
        b_run = QPushButton("Run")
        b_run.clicked.connect(lambda: self.send(f"demo {self.demo_box.currentIndex() + 1}"))
        b_end = QPushButton("Stop demo")
        b_end.clicked.connect(lambda: self.send("demo stop"))
        gl.addWidget(self.demo_box, 1)
        gl.addWidget(b_run)
        gl.addWidget(b_end)
        self.cmd_buttons += [b_run, b_end]
        left.addWidget(g)

        g = QGroupBox("Jog")
        gl = QGridLayout(g)
        self.jog_axis = QComboBox()
        self.jog_axis.addItems([f"{i + 1}  {n}" for i, n in enumerate(self.axes)])
        self.jog_axis.setCurrentIndex(1)
        self.jog_step = QDoubleSpinBox()
        self.jog_step.setRange(0.1, 360.0)
        self.jog_step.setValue(5.0)
        self.jog_step.setSuffix("  mm / deg")
        b_minus = QPushButton("◀  −")
        b_plus = QPushButton("+  ▶")
        b_minus.clicked.connect(lambda: self._jog(-1))
        b_plus.clicked.connect(lambda: self._jog(+1))
        gl.addWidget(QLabel("Axis"), 0, 0)
        gl.addWidget(self.jog_axis, 0, 1, 1, 2)
        gl.addWidget(QLabel("Step"), 1, 0)
        gl.addWidget(self.jog_step, 1, 1, 1, 2)
        gl.addWidget(b_minus, 2, 1)
        gl.addWidget(b_plus, 2, 2)
        self.cmd_buttons += [b_minus, b_plus]
        left.addWidget(g)

        g = QGroupBox("Positions")
        gl = QVBoxLayout(g)
        self.table = QTableWidget(6, 4)
        self.table.setHorizontalHeaderLabels(["Axis", "Position", "Zero", "Moving"])
        self.table.verticalHeader().setVisible(False)
        hh = self.table.horizontalHeader()
        hh.setSectionResizeMode(QHeaderView.ResizeToContents)
        hh.setSectionResizeMode(1, QHeaderView.Stretch)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        for i, n in enumerate(self.axes):
            self.table.setItem(i, 0, QTableWidgetItem(n))
            for c in (1, 2, 3):
                self.table.setItem(i, c, QTableWidgetItem("—"))
        self.table.setMinimumHeight(215)
        gl.addWidget(self.table)
        left.addWidget(g)
        g = QGroupBox("Measurement (CSV)")
        gl = QGridLayout(g)
        self.rec_rate = QComboBox()
        self.rec_rate.addItems(["10 per second", "20 per second", "50 per second"])
        self.rec_excel = QCheckBox("Excel NL/DE format ( ; and , )")
        self.btn_rec = QPushButton("●  Record")
        self.btn_rec.clicked.connect(self._toggle_record)
        self.lbl_rec = QLabel("")
        gl.addWidget(QLabel("Rate"), 0, 0)
        gl.addWidget(self.rec_rate, 0, 1)
        gl.addWidget(self.rec_excel, 1, 0, 1, 2)
        gl.addWidget(self.btn_rec, 2, 0)
        gl.addWidget(self.lbl_rec, 2, 1)
        self.cmd_buttons.append(self.btn_rec)
        self.recorder = None
        left.addWidget(g)
        left.addStretch(1)

        # right: plots + log + command line
        right = QVBoxLayout()
        mid.addLayout(right, 2)
        pg.setConfigOptions(antialias=True, background="w", foreground="k")
        self.plot_tra = pg.PlotWidget(title="Translation (mm)")
        self.plot_rot = pg.PlotWidget(title="Rotation (deg)")
        self.curves = []
        for p in (self.plot_tra, self.plot_rot):
            p.showGrid(x=True, y=True, alpha=0.3)
            p.setLabel("bottom", "time (s)")
            p.addLegend(offset=(5, 5))
        for i in range(6):
            tube = i // 2
            plot = self.plot_tra if i % 2 else self.plot_rot
            self.curves.append(plot.plot(name=f"T{tube + 1}", pen=pg.mkPen(TUBE_COLOURS[tube], width=2)))
        right.addWidget(self.plot_tra, 1)
        right.addWidget(self.plot_rot, 1)

        self.logview = QPlainTextEdit()
        self.logview.setReadOnly(True)
        self.logview.setMaximumBlockCount(3000)
        self.logview.setFont(QFont("Consolas", 9))
        right.addWidget(self.logview, 1)
        opts = QHBoxLayout()
        self.show_st = QCheckBox("show status lines")
        opts.addWidget(self.show_st)
        opts.addStretch(1)
        opts.addWidget(QLabel(f"log file: {self.logger.path}"))
        right.addLayout(opts)

        cl = QHBoxLayout()
        self.cmdline = QLineEdit()
        self.cmdline.setPlaceholderText("command, e.g.  mv 2=20 4=20 6=20   (Enter to send, ↑ for history)")
        self.cmdline.returnPressed.connect(self._send_typed)
        self.cmdline.installEventFilter(self)
        self.history, self.hpos = [], 0
        cl.addWidget(self.cmdline, 1)
        right.addLayout(cl)

    # ------------------------------------------------------------ connection
    def _refresh_ports(self):
        cur = self.port.currentData() if self.port.count() else None
        self.port.clear()
        self.port.addItem("Fake robot (no hardware)", "FAKE")
        for dev, desc in list_ports():
            self.port.addItem(f"{dev}  –  {desc}", dev)
        idx = self.port.findData(cur) if cur else -1
        # prefer a real Arduino if one is plugged in
        if idx < 0:
            for k in range(self.port.count()):
                if "arduino" in self.port.itemText(k).lower() or "mega" in self.port.itemText(k).lower():
                    idx = k
        self.port.setCurrentIndex(max(idx, 0))

    def _toggle_connect(self):
        if self.link:
            self._disconnect()
            return
        dev = self.port.currentData()
        try:
            tr = FakeRobot() if dev == "FAKE" else SerialTransport(dev)
        except Exception as e:
            QMessageBox.warning(self, "Connect", f"Could not open {dev}:\n{e}")
            return
        self.link = RobotLink(tr, log=self._log_filtered)
        self.logger("#", f"connected to {tr.name}")
        self._append(f"--- connected to {tr.name} ---", "#555")
        # Opening the USB port restarts the Arduino: wait for its start message
        self.waiting_boot = True
        self.boot_deadline = time.time() + 4.0
        self.hist_t.clear()
        for h in self.hist:
            h.clear()
        self._set_connected(True)

    def _log_filtered(self, direction, text):
        # pings and pongs twice a second would drown the log file
        if "ping" in text or "OK pong" in text:
            return
        self.logger(direction, text)

    # ------------------------------------------------------------ measurement (step 4.4)
    def _toggle_record(self):
        if self.recorder:
            self._stop_record()
            return
        folder = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "recordings")
        self.recorder = Recorder(folder, self.axes, excel_eu=self.rec_excel.isChecked())
        ms = [100, 50, 20][self.rec_rate.currentIndex()]
        self.send(f"stream {ms}")
        self.btn_rec.setText("■  Stop recording")
        self.btn_rec.setStyleSheet("QPushButton{background:#c62828;color:white;font-weight:bold}")
        self.rec_rate.setEnabled(False)
        self.rec_excel.setEnabled(False)
        self._append(f"--- recording to {self.recorder.path} ---", "#555")

    def _stop_record(self):
        rec, self.recorder = self.recorder, None
        rec.close()
        if self.link:
            self.send(f"stream {STREAM_MS}")
        self.btn_rec.setText("●  Record")
        self.btn_rec.setStyleSheet("")
        self.rec_rate.setEnabled(True)
        self.rec_excel.setEnabled(True)
        self.lbl_rec.setText(f"{rec.rows} rows saved")
        self._append(f"--- recording saved: {rec.path} ({rec.rows} rows) ---", "#555")

    def _disconnect(self):
        if self.recorder:
            self._stop_record()
        self.ping_timer.stop()
        try:
            self.link.send("hb 0", tagged=False)        # leaving on purpose: robot need not watch
            self.link.send("stream off", tagged=False)
        except Exception:
            pass
        self.link.close()
        self.link = None
        self.logger("#", "disconnected")
        self._append("--- disconnected ---", "#555")
        self._set_connected(False)

    def _after_boot(self):
        self.waiting_boot = False
        self.send("info")
        self.send(f"stream {STREAM_MS}")
        self.send(f"hb {HEARTBEAT_MS}")
        self.last_pong = time.time()
        self.ping_timer.start(PING_MS)

    def _ping(self):
        if not self.link:
            return
        rid = self.link.send("ping")
        self.ping_sent[rid] = time.time()
        for k in [k for k, t in self.ping_sent.items() if time.time() - t > 5]:
            del self.ping_sent[k]
        age = time.time() - self.last_pong
        if age > HEARTBEAT_MS / 1000.0:
            self._show_link("link: NO REPLY", "#c62828")
        elif self.rtt_ms is not None:
            self._show_link(f"link OK · {self.rtt_ms:.0f} ms", "#2e7d32")

    def _show_link(self, text, colour):
        self.lbl_link.setText(text)
        self.lbl_link.setStyleSheet(f"QLabel{{color:{colour};font-weight:bold}}")

    def _set_connected(self, on):
        self.btn_connect.setText("Disconnect" if on else "Connect")
        self.port.setEnabled(not on)
        for b in self.cmd_buttons + [self.btn_stop]:
            b.setEnabled(on)
        self.cmdline.setEnabled(on)
        if not on:
            self.lbl_link.setText("")
            self.lbl_robot.setText("not connected")
            self._show_state("—")

    # ------------------------------------------------------------ sending
    def send(self, cmd):
        if not self.link:
            return
        self.link.send(cmd)
        self._append(f"> {cmd}", "#555")

    def _send_typed(self):
        cmd = self.cmdline.text().strip()
        if not cmd:
            return
        self.history.append(cmd)
        self.hpos = len(self.history)
        self.cmdline.clear()
        self.send(cmd)

    def eventFilter(self, obj, ev):
        if obj is self.cmdline and ev.type() == ev.Type.KeyPress and self.history:
            if ev.key() == Qt.Key_Up:
                self.hpos = max(0, self.hpos - 1)
                self.cmdline.setText(self.history[self.hpos])
                return True
            if ev.key() == Qt.Key_Down:
                self.hpos = min(len(self.history), self.hpos + 1)
                self.cmdline.setText(self.history[self.hpos] if self.hpos < len(self.history) else "")
                return True
        return super().eventFilter(obj, ev)

    def _jog(self, sign):
        axis = self.jog_axis.currentIndex() + 1
        self.send(f"move {axis} {sign * self.jog_step.value():g}")

    def keyPressEvent(self, ev):
        if ev.key() == Qt.Key_Escape and self.link:     # Esc = STOP, anywhere in the window
            self.send("stop")
        super().keyPressEvent(ev)

    # ------------------------------------------------------------ receiving
    def _tick(self):
        if not self.link:
            return
        if self.waiting_boot and time.time() > self.boot_deadline:
            self._after_boot()                          # no start message: maybe no reset
        for ln in self.link.poll():
            if self.waiting_boot and (ln.text.startswith("EV boot") or "Type 'help'" in ln.text):
                self._after_boot()
            if self.recorder and ln.kind == "EV":
                self.recorder.event(ln.text, ln.t)
            if ln.kind == "ST":
                st = parse_status(ln.text)
                self._on_status(st)
                if self.recorder and st:
                    self.recorder.status(st, ln.t)
                if self.show_st.isChecked():
                    self._append(ln.raw, "#999")
                continue
            if ln.kind == "OK" and ln.text.startswith("OK info"):
                self._on_info(parse_kv(ln.text))
            if ln.kind == "OK" and " t=" in ln.text and "pos=" in ln.text:
                self._on_status(parse_status(ln.text))
            if ln.kind == "OK" and ln.text == "OK pong":
                sent = self.ping_sent.pop(ln.id, None)
                if sent:
                    self.rtt_ms = (time.time() - sent) * 1000
                self.last_pong = time.time()
                continue                                # not shown in the log
            colour = {"OK": "#1b5e20", "ERR": "#b71c1c", "EV": "#0d47a1"}.get(ln.kind, "#333")
            self._append(ln.raw, colour)
            if ln.kind == "ERR" and ln.text.startswith("ERR link lost"):
                self._disconnect()
                QMessageBox.warning(self, "Connection", ln.text)
                return
        self._update_plots()
        if self.recorder:
            self.lbl_rec.setText(f"{self.recorder.rows} rows · {time.time() - self.recorder.t0:.0f} s")

    def _on_info(self, kv):
        axes = kv.get("axes", "")
        if axes:
            parts = [a.split(":") for a in axes.split(",")]
            self.axes = [p[0] for p in parts]
            self.units = [p[1] if len(p) > 1 else "" for p in parts]
            for i, n in enumerate(self.axes[:6]):
                self.table.item(i, 0).setText(n)
        self.lbl_robot.setText(f"{kv.get('profile', '?')}   ·   firmware {kv.get('fw', '?')}"
                               f"   ·   protocol {kv.get('proto', '?')}")

    def _on_status(self, st):
        if not st:
            return
        self._show_state(st["state"])
        for i, p in enumerate(st["pos"][:6]):
            self.table.item(i, 1).setText(f"{p:.2f} {self.units[i]}")
            homed = st["homed"][i] if i < len(st["homed"]) else False
            self.table.item(i, 2).setText("✓" if homed else "?")
            busy = st["busy"][i] if i < len(st["busy"]) else False
            it = self.table.item(i, 3)
            it.setText("●" if busy else "")
            it.setForeground(QColor("#1565c0"))
        self.hist_t.append(st["t"] / 1000.0)
        for i, p in enumerate(st["pos"][:6]):
            self.hist[i].append(p)

    def _update_plots(self):
        if len(self.hist_t) < 2:
            return
        t = list(self.hist_t)
        for i in range(6):
            self.curves[i].setData(t, list(self.hist[i]))

    def _show_state(self, s):
        self.lbl_state.setText(s)
        col = STATE_COLOURS.get(s, "#424242")
        self.lbl_state.setStyleSheet(f"QLabel{{background:{col};color:white;border-radius:6px;padding:4px}}")

    def _append(self, text, colour):
        self.logview.appendHtml(f'<span style="color:{colour}">{text.replace("<", "&lt;")}</span>')
        self.logview.moveCursor(QTextCursor.End)

    def closeEvent(self, ev):
        if self.link:
            self._disconnect()
        super().closeEvent(ev)


def main():
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
