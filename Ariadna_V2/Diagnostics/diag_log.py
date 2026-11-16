"""
Letový záznam — stažení, filtrování, export a grafy záznamu z flash Mastera.

Master zapisuje 96bajtové záznamy: 200 Hz za letu (ASCENT..LANDED, příznak
flight_rate), 10 Hz na zemi. Tato záložka:

  * zjistí obsah (LOG INFO) a najde lety přímo v raketě (LOG FLIGHTS - Master
    projde flash sám, přes USB jde jen seznam),
  * stáhne buď jen vybrané lety (s pár sekundami okolí), nebo celý záznam
    (LOG DUMP od počet),
  * filtruje stažená data (jen let / celý, číslo letu, časové okno),
  * vykreslí grafy a exportuje CSV (stejný formát jako tools/daidalos_log.py).

Dekódování záznamu je převzaté z tools/daidalos_log.py - jeden formát, jedno
místo.
"""
from __future__ import annotations

import csv
import importlib.util
import math
import os

from PyQt6 import QtCore, QtWidgets

import actuator_gui as ag
from actuator_gui import button, label, Card
from daidalos_widgets import StripChart

HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location(
    "daidalos_log", os.path.join(HERE, "..", "..", "tools", "daidalos_log.py"))
dlog = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(dlog)

CONTEXT_S = 3.0          # s of 10 Hz ground data kept before/after a flight
PLOT_POINTS = 2500       # per series, decimated


class LogTab(QtWidgets.QWidget):
    def __init__(self, win):
        super().__init__()
        self.win = win
        self.rows: list[dict] = []          # decoded, everything downloaded
        self.segs: list[dict] = []
        self.queue: list[tuple[int, int]] = []
        self.expect = 0
        self.got_in_job = 0
        self._pending: list[dict] = []
        lay = QtWidgets.QGridLayout(self)
        lay.setContentsMargins(20, 16, 20, 20)
        lay.setSpacing(14)

        # ---- on the board
        bc = Card("V raketě", "Flash Mastera (W25Q128)")
        self.info = label("Obsah neznámý — Zjistit obsah", "muted")
        bc.lay.addWidget(self.info)
        r = QtWidgets.QHBoxLayout()
        for txt, fn, tip in (("Zjistit obsah", lambda: self.win._cmd("LOG INFO"), "LOG INFO"),
                             ("Najít lety", self.find_flights, "Master projde záznam a vrátí seznam letů (LOG FLIGHTS, pár s)"),
                             ("Stáhnout vybrané lety", self.dl_selected, "Jen záznamy z letu + %g s okolí" % CONTEXT_S),
                             ("Stáhnout vše", self.dl_all, "Celý záznam včetně 10 Hz dat ze země"),
                             ("Zastavit", self.stop, "LOG STOP")):
            b = button(txt, "primary" if txt.startswith("Stáhnout vybr") else "", tip)
            b.clicked.connect(fn)
            r.addWidget(b)
        r.addStretch(1)
        b = button("Smazat záznam v raketě…", tip="LOG ERASE (parametry zůstanou), jen IDLE/TEST")
        b.clicked.connect(self.erase)
        r.addWidget(b)
        bc.lay.addLayout(r)
        self.seg_tab = QtWidgets.QTableWidget(0, 6)
        self.seg_tab.setHorizontalHeaderLabels(["", "let", "záznamů", "délka", "max výška", "od záznamu"])
        self.seg_tab.verticalHeader().setVisible(False)
        self.seg_tab.horizontalHeader().setStretchLastSection(True)
        self.seg_tab.setMaximumHeight(170)
        bc.lay.addWidget(self.seg_tab)
        self.prog = QtWidgets.QProgressBar()
        self.prog.setFormat("%v / %m záznamů")
        self.prog.setValue(0)
        bc.lay.addWidget(self.prog)
        lay.addWidget(bc, 0, 0)

        # ---- downloaded data
        dc = Card("Stažená data", "Filtr, export, grafy")
        r = QtWidgets.QHBoxLayout()
        r.addWidget(label("Zobrazit", "muted"))
        self.f_mode = QtWidgets.QComboBox()
        self.f_mode.addItems(["jen let (200 Hz)", "let + okolí", "vše"])
        self.f_mode.setCurrentIndex(1)
        r.addWidget(self.f_mode)
        r.addWidget(label("let č.", "muted"))
        self.f_flight = QtWidgets.QComboBox()
        r.addWidget(self.f_flight)
        r.addWidget(label("čas od", "muted"))
        self.f_t0 = QtWidgets.QDoubleSpinBox()
        self.f_t0.setRange(-1e6, 1e6)
        self.f_t0.setSuffix(" s")
        self.f_t0.setSpecialValueText("začátek")
        self.f_t0.setValue(-1e6)
        r.addWidget(self.f_t0)
        r.addWidget(label("do", "muted"))
        self.f_t1 = QtWidgets.QDoubleSpinBox()
        self.f_t1.setRange(-1e6, 1e6)
        self.f_t1.setSuffix(" s")
        self.f_t1.setSpecialValueText("konec")
        self.f_t1.setValue(-1e6)
        r.addWidget(self.f_t1)
        for w in (self.f_mode, self.f_flight):
            w.currentIndexChanged.connect(self.replot)
        for w in (self.f_t0, self.f_t1):
            w.valueChanged.connect(self.replot)
        r.addStretch(1)
        for txt, fn in (("Exportovat CSV…", self.export), ("Otevřít CSV…", self.open_csv),
                        ("Vymazat", self.clear)):
            b = button(txt)
            b.clicked.connect(fn)
            r.addWidget(b)
        dc.lay.addLayout(r)
        self.summary = label("—", "muted")
        self.summary.setWordWrap(True)
        dc.lay.addWidget(self.summary)
        lay.addWidget(dc, 1, 0)

        # ---- plots (time axis: s from the start of the selection)
        pw = QtWidgets.QWidget()
        g = QtWidgets.QGridLayout(pw)
        g.setContentsMargins(0, 0, 0, 0)
        self.charts = {
            "alt": StripChart("Výška", ["h"], "m"),
            "vel": StripChart("Rychlost", ["vz", "vx", "vy"], "m/s"),
            "tilt": StripChart("Náklon osy tahu", ["náklon"], "°"),
            "tvc": StripChart("TVC a klapka", ["TVC 1", "TVC 2", "klapka×10"], "°"),
            "thr": StripChart("Tah (odhad)", ["tah"], "N"),
            "cal": StripChart("Kalibrace: odpor / tah / křidélka", ["odpor", "tah/tab.", "křidélka"], "×"),
            "pwr": StripChart("Napájení", ["baterie", "proud"], "V / A"),
            "st": StripChart("Stav / fáze / odpočet", ["stav", "fáze", "odpočet"], ""),
        }
        for i, c_ in enumerate(self.charts.values()):
            c_.setMinimumHeight(170)
            g.addWidget(c_, i // 2, i % 2)
        lay.addWidget(pw, 2, 0)
        lay.setRowStretch(2, 1)

    # ------------------------------------------------------------ commands
    def find_flights(self) -> None:
        self.segs = []
        self.seg_tab.setRowCount(0)
        self.win._cmd("LOG FLIGHTS")

    def dl_selected(self) -> None:
        jobs = []
        for i, s in enumerate(self.segs):
            it = self.seg_tab.item(i, 0)
            if it is not None and it.checkState() == QtCore.Qt.CheckState.Checked:
                ctx = int(CONTEXT_S * 10)               # ground records are 10 Hz
                start = max(0, s["start"] - ctx)
                jobs.append((start, (s["start"] - start) + s["count"] + ctx))
        if not jobs:
            QtWidgets.QMessageBox.information(self, "Záznam", "Nejdřív Najít lety a zaškrtnout, které stáhnout.")
            return
        self._start(jobs)

    def dl_all(self) -> None:
        self._start([(0, 0)])

    def _start(self, jobs) -> None:
        self.queue = list(jobs)
        self.prog.setValue(0)
        self._next_job()

    def _next_job(self) -> None:
        if not self.queue:
            self._decoded_done()
            return
        start, cnt = self.queue.pop(0)
        self.got_in_job = 0
        self.win._cmd(f"LOG DUMP {start} {cnt}" if cnt else f"LOG DUMP {start}")

    def stop(self) -> None:
        self.queue = []
        self.win._cmd("LOG STOP")

    def erase(self) -> None:
        if QtWidgets.QMessageBox.question(
                self, "Smazat záznam",
                "Smazat celý letový záznam ve flash Mastera?\nParametry zůstanou. Nejde vrátit.") \
                != QtWidgets.QMessageBox.StandardButton.Yes:
            return
        self.win._cmd("LOG ERASE")

    # ------------------------------------------------------------ lines from the Master
    def on_line(self, line: str) -> None:
        if line.startswith("LOGR "):
            d = dlog.decode_line(line)
            if d:
                n, row = d
                row["rec"] = n
                self._pending.append(row)
                self.got_in_job += 1
                if self.got_in_job % 200 == 0:
                    self.prog.setValue(self.prog.value() + 200)
            return
        p = line.split()
        if line.startswith("OK LOG INFO") and len(p) >= 5:
            used, n = int(p[3]), int(p[4])
            self.info.setText(f"{n} záznamů, {used / 1024:.0f} kB z 16 384 kB "
                              f"({100 * used / (16 * 1024 * 1024):.1f} %)")
        elif line.startswith("OK LOG SCAN"):
            self.info.setText(f"hledám lety v {p[3]} záznamech…")
        elif line.startswith("LOGF ") and len(p) >= 7:
            s = {"i": int(p[1]), "start": int(p[2]), "count": int(p[3]), "t0": int(p[4]),
                 "t1": int(p[5]), "hmax": float(p[6])}
            self.segs.append(s)
            r = self.seg_tab.rowCount()
            self.seg_tab.insertRow(r)
            chk = QtWidgets.QTableWidgetItem()
            chk.setCheckState(QtCore.Qt.CheckState.Checked)
            self.seg_tab.setItem(r, 0, chk)
            for c, v in enumerate([str(s["i"] + 1), str(s["count"]), f"{(s['t1'] - s['t0']) / 1000:.1f} s",
                                   f"{s['hmax']:.1f} m", str(s["start"])], 1):
                self.seg_tab.setItem(r, c, QtWidgets.QTableWidgetItem(v))
        elif line.startswith("OK LOG FLIGHTS"):
            self.info.setText(f"nalezeno letů: {p[3]}")
        elif line.startswith("OK LOG BEGIN"):
            self.expect = int(p[3])
            self.prog.setMaximum(self.prog.value() + self.expect)
        elif line.startswith("OK LOG DUMP") or line.startswith("OK LOG STOP"):
            self._next_job() if line.startswith("OK LOG DUMP") else self._decoded_done()
        elif line.startswith("OK LOG ERASE"):
            self.info.setText("záznam smazán")
            self.segs = []
            self.seg_tab.setRowCount(0)
        elif line.startswith("ERR LOG"):
            self.info.setText(line)
            self.queue = []

    def _decoded_done(self) -> None:
        if not self._pending:
            return
        seen = {r["rec"] for r in self.rows}
        self.rows += [r for r in self._pending if r["rec"] not in seen]
        self._pending = []
        self.rows.sort(key=lambda r: r["rec"])
        self._number_flights()
        self.prog.setValue(self.prog.maximum())
        self.win.log(f"# záznam: staženo {len(self.rows)} záznamů")
        self.replot()

    # ------------------------------------------------------------ data
    def _number_flights(self) -> None:
        """Flight number for every row: each run of flight-rate records is a
        flight; ground records belong to the flight they precede (within
        CONTEXT_S) or else to the one before them."""
        starts = [i for i, r in enumerate(self.rows)
                  if r["flight_rate"] and (i == 0 or not self.rows[i - 1]["flight_rate"])]
        k = -1
        for i, r in enumerate(self.rows):
            while k + 1 < len(starts) and starts[k + 1] <= i:
                k += 1
            f = max(k, 0)
            if not r["flight_rate"] and k + 1 < len(starts):
                nxt = self.rows[starts[k + 1]]
                if 0 <= nxt["t_ms"] - r["t_ms"] <= CONTEXT_S * 1000:
                    f = k + 1
            r["flight"] = f
        self.f_flight.blockSignals(True)
        cur = self.f_flight.currentText()
        self.f_flight.clear()
        self.f_flight.addItem("všechny")
        for f in sorted({r["flight"] for r in self.rows}):
            n = sum(1 for r in self.rows if r["flight"] == f and r["flight_rate"])
            self.f_flight.addItem(f"{f + 1}" + ("" if n else " (jen země)"))
        i = self.f_flight.findText(cur)
        self.f_flight.setCurrentIndex(i if i >= 0 else (1 if self.f_flight.count() > 1 else 0))
        self.f_flight.blockSignals(False)

    def selected(self) -> list[dict]:
        rows = self.rows
        if self.f_flight.currentIndex() > 0:
            f = int(self.f_flight.currentText().split()[0]) - 1
            rows = [r for r in rows if r["flight"] == f]
        mode = self.f_mode.currentIndex()
        if mode == 0:
            rows = [r for r in rows if r["flight_rate"]]
        elif mode == 1 and any(r["flight_rate"] for r in rows):
            fl = [r["t_ms"] for r in rows if r["flight_rate"]]
            a, b = min(fl) - CONTEXT_S * 1000, max(fl) + CONTEXT_S * 1000
            rows = [r for r in rows if a <= r["t_ms"] <= b]
        if not rows:
            return rows
        base = rows[0]["t_ms"]
        t0 = self.f_t0.value() if self.f_t0.value() > -1e6 else -1e9
        t1 = self.f_t1.value() if self.f_t1.value() > -1e6 else 1e9
        return [r for r in rows if t0 <= (r["t_ms"] - base) / 1000.0 <= t1]

    def replot(self) -> None:
        rows = self.selected()
        for c_ in self.charts.values():
            c_.clear()
        if not rows:
            self.summary.setText("Žádná data (nebo filtr nic nepropustil).")
            return
        base = rows[0]["t_ms"]
        span = max((rows[-1]["t_ms"] - base) / 1000.0, 1.0)
        step = max(1, len(rows) // PLOT_POINTS)
        for c_ in self.charts.values():
            c_.span = span * 1.001
        for r in rows[::step]:
            t = (r["t_ms"] - base) / 1000.0
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, abs(r["uz"])))))
            self.charts["alt"].data[0].append((t, r["h"]))
            for i, k in enumerate(("vz", "vx", "vy")):
                self.charts["vel"].data[i].append((t, r[k]))
            self.charts["tilt"].data[0].append((t, tilt))
            for i, v in enumerate((r["tvc_x"], r["tvc_y"], r["clamp"] * 10)):
                self.charts["tvc"].data[i].append((t, v))
            self.charts["thr"].data[0].append((t, r["thrust_est"]))
            for i, k in enumerate(("drag", "tscale", "fin_gain")):
                if r[k] > 0:
                    self.charts["cal"].data[i].append((t, r[k]))
            self.charts["pwr"].data[0].append((t, r["vbat"]))
            self.charts["pwr"].data[1].append((t, r["current"]))
            for i, v in enumerate((r["state"], r["nav_phase"], r["t_event"])):
                self.charts["st"].data[i].append((t, v))
        for c_ in self.charts.values():
            c_.update()
        fl = [r for r in rows if r["flight_rate"]]
        txt = (f"{len(rows)} záznamů, {span:.1f} s, max výška {max(r['h'] for r in rows):.1f} m, "
               f"min vz {min(r['vz'] for r in rows):.1f} m/s")
        if fl:
            desc = [r for r in fl if r["state_name"] == "DESCENT" and r["drag"] > 0]
            burn = sorted(r["tscale"] for r in fl if r["nav_phase"] == 4 and r["tscale"] > 0)
            last = fl[-1]
            txt += (f"  ·  dosednutí vz {last['vz']:.2f} m/s"
                    + (f"  ·  odpor/model {desc[-1]['drag']:.2f}" if desc else "")
                    + (f"  ·  tah/tabulka {burn[len(burn) // 2]:.2f}" if burn else "")
                    + (f"  ·  křidélka {last['fin_gain']:.2f}" if last["fin_gain"] > 0 else "")
                    + "   (co s tím: CALIBRATION_MANUAL.md, kap. 10)")
        self.summary.setText(txt)

    def export(self) -> None:
        rows = self.selected()
        if not rows:
            return
        f, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Export záznamu", "let.csv", "CSV (*.csv)")
        if not f:
            return
        cols = ["flight", "rec"] + [k for k in rows[0] if k not in ("flight", "rec")]
        with open(f, "w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=cols, extrasaction="ignore")
            w.writeheader()
            w.writerows(rows)
        self.win.log(f"# záznam: {len(rows)} řádků -> {f}")

    def open_csv(self) -> None:
        f, _ = QtWidgets.QFileDialog.getOpenFileName(self, "CSV záznamu", "", "CSV (*.csv)")
        if f:
            self.load_csv(f)

    def load_csv(self, path: str) -> None:
        rows = []
        with open(path, encoding="utf-8") as fh:
            for i, r in enumerate(csv.DictReader(fh)):
                d = {}
                for k, v in r.items():
                    try:
                        d[k] = float(v) if "." in v or "e" in v.lower() else int(v)
                    except (ValueError, AttributeError):
                        d[k] = v
                d.setdefault("rec", i)
                d.setdefault("flight_rate", 0)
                rows.append(d)
        self.rows = rows
        self._number_flights()
        self.replot()

    def clear(self) -> None:
        self.rows = []
        self._pending = []
        self.f_flight.clear()
        self.replot()
