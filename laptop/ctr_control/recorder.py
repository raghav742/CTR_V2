"""
Measurement recording (step 4.4): writes the status stream and the events to a
CSV file, for plots and tables in reports.

Columns: pc_time_s (since the start of the recording), robot_ms (robot clock),
state, the six positions (mm / deg), busy (one 0/1 per axis), event (text of an
EV line; on those rows the position columns are empty).

Two formats:
  standard  ',' between columns, '.' as decimal point  (Python, MATLAB, LibreOffice)
  excel_eu  ';' between columns, ',' as decimal point  (Excel with Dutch/German settings)
"""
import csv
import os
import time


class Recorder:
    def __init__(self, folder, axes, excel_eu=False):
        os.makedirs(folder, exist_ok=True)
        self.path = os.path.join(folder, time.strftime("rec_%Y%m%d_%H%M%S.csv"))
        self.f = open(self.path, "w", newline="", encoding="utf-8")
        self.excel_eu = excel_eu
        self.w = csv.writer(self.f, delimiter=";" if excel_eu else ",")
        self.t0 = time.time()
        self.rows = 0
        self.w.writerow(["pc_time_s", "robot_ms", "state"] + list(axes) + ["busy", "event"])

    def _num(self, v, dec):
        s = f"{v:.{dec}f}"
        return s.replace(".", ",") if self.excel_eu else s

    def status(self, st, pc_time=None):
        t = (pc_time or time.time()) - self.t0
        busy = "".join("1" if b else "0" for b in st["busy"])
        self.w.writerow([self._num(t, 3), st["t"], st["state"]] +
                        [self._num(p, 2) for p in st["pos"]] + [busy, ""])
        self.rows += 1

    def event(self, text, pc_time=None):
        t = (pc_time or time.time()) - self.t0
        self.w.writerow([self._num(t, 3), "", ""] + [""] * 6 + ["", text])
        self.rows += 1

    def close(self):
        self.f.close()
