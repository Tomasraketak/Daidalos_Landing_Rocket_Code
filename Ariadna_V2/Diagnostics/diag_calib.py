"""
"Kalibrace" tab of the diagnostic console: TriSense calibration, the pad
position and the servo connection check.

  * GYRO          the Position MCU averages the gyro for 1 s with the vehicle
                  still ("CAL GYRO"). Done at every boot as well. After a
                  "CAL GYRO" from here, the offset the board now uses (the
                  "goff" of the R line, fw 3.27) is saved as imu.gyr_off_*;
                  at boot the Position MCU falls back to it when its own
                  boot calibration is far from it (moved while booting).
  * ACCELEROMETER six or more STILL orientations, any directions (ideally
                  each axis once up and once down). An axis-aligned ellipsoid
                  is fitted to the averaged readings:
                      a = (raw - off) * scale,  |a| = 1 g
                  -> imu.acc_off_* / imu.acc_scale_*.
  * MAGNETOMETER  the board is turned slowly through every direction for
                  ~1 minute; a general ellipsoid is fitted to the RAW samples
                      m = SI * (raw - HI),  |m| = const
                  -> hard iron imu.mag_hi_*, soft iron imu.mag_si_* (the
                  MotionCal convention INSLIB uses).
  * PAD           the current GPS position, averaged for 10 s, becomes the
                  pad (pos.pad_lat_deg / _lon_deg): the navigation origin no
                  longer moves to wherever the vehicle is when it is armed.
  * SERVOS        "SERVOCHECK": the Master moves each servo 5 deg and back
                  and watches the battery current (also runs automatically
                  when the RBF tag comes out, mst.servo_check_on_rbf).

All values go to the Master as ordinary parameters (PARAM SET) and are made
permanent with PARAM SAVE; the Master streams them to the Position MCU,
which applies them at once. Everything works on the raw-data stream ("R"
lines), switched on automatically while it is needed.

The fits live in calib_fit.py (numpy only) and are tested by
test/check_calib.py.
"""
from __future__ import annotations

import json
import math
import time

try:
    import numpy as np
    from calib_fit import fit_accel, fit_mag, sphere_coverage
except ImportError:          # the tab says so instead of failing to start
    np = None

from PyQt6 import QtCore, QtWidgets

from actuator_gui import button, label, Card

SERVO_NAMES = ["TVC X", "TVC Y", "Klapka (škrcení)", "Křidélko 1", "Křidélko 2",
               "Křidélko 3", "Křidélko 4", "Aerokryt (zámek)"]


# ============================================================================
#  the tab
# ============================================================================
class CalibTab(QtWidgets.QWidget):
    ACC_SAMPLES = 50          # 2 s of R lines at 25 Hz
    PAD_SECONDS = 10.0
    GYRO_SETTLE_S = 2.5       # CAL GYRO: 1 s average + filter restart, then read "goff"
    GYRO_GIVE_UP_S = 8.0

    def __init__(self, win):
        super().__init__()
        self.win = win
        self.acc_points: list[list[float]] = []
        self.acc_buf: list[tuple] | None = None          # capture in progress
        self.acc_fit: dict | None = None
        self.mag_pts: list[list[float]] = []
        self.mag_on = False
        self.mag_fit: dict | None = None
        self.pad_buf: list[tuple] | None = None
        self.pad_t0 = 0.0
        self.gyro_t0: float | None = None                # CAL GYRO sent, offset not saved yet
        self.gyro_msg_until = 0.0                        # keep the result line this long

        g = QtWidgets.QGridLayout(self)
        g.setContentsMargins(20, 16, 20, 20)
        g.setSpacing(16)
        intro = label("Kalibrace se ukládá jako parametry do flash Masteru a Position MCU ji použije "
                      "okamžitě (není potřeba nic nahrávat). Při sběru se samo zapne posílání "
                      "surových dat. Vše jen na zemi (IDLE / TEST).", "muted")
        intro.setWordWrap(True)
        g.addWidget(intro, 0, 0, 1, 2)
        if np is None:
            warn = label("Chybí balík numpy - výpočty kalibrace nepůjdou. "
                         "python -m pip install numpy", "muted")
            g.addWidget(warn, 1, 0, 1, 2)

        # ---------------------------------------------------------- gyro
        gy = Card("Gyroskop", "Nulová odchylka")
        t = label("Raketou (deskou) 2 s nehýbat. Výsledek se sám uloží do flash (imu.gyr_off_*). "
                  "Kalibruje se i při každém zapnutí; když se ale raketa při zapnutí hýbala "
                  "(výsledek se liší od uloženého o víc než 1 °/s), použije se uložený. INSLIB pak "
                  "odchylku dál dolaďuje za běhu. |ω| v klidu po kalibraci má být < 0,2 °/s.", "muted")
        t.setWordWrap(True)
        gy.lay.addWidget(t)
        row = QtWidgets.QHBoxLayout()
        self.gyro_btn = button("Kalibrovat gyro", "primary")
        self.gyro_btn.clicked.connect(self.gyro_start)
        row.addWidget(self.gyro_btn)
        self.gyro_lbl = label("|ω| = —", "muted")
        row.addWidget(self.gyro_lbl)
        row.addStretch(1)
        gy.lay.addLayout(row)
        self.gyro_off_lbl = label("Offset: —", "muted")
        self.gyro_off_lbl.setWordWrap(True)
        gy.lay.addWidget(self.gyro_off_lbl)
        g.addWidget(gy, 2, 0)

        # ---------------------------------------------------------- pad
        pd = Card("Rampa", "Poloha rampy = počátek navigace")
        t = label("Raketa na rampě, GPS fix 3D. Poloha se 10 s průměruje a uloží jako "
                  "pos.pad_lat_deg / pos.pad_lon_deg. Pak je počátek (cíl přistání, souřadnice "
                  "kotev UWB) pevně tady, i když se raketa zastrčí jinde. 0 / 0 = nenastaveno "
                  "(počátek = poloha při zastrčení).", "muted")
        t.setWordWrap(True)
        pd.lay.addWidget(t)
        row = QtWidgets.QHBoxLayout()
        self.pad_btn = button("Rampa = tady", "primary")
        self.pad_btn.clicked.connect(self.pad_start)
        row.addWidget(self.pad_btn)
        b = button("Zrušit polohu rampy")
        b.clicked.connect(self.pad_clear)
        row.addWidget(b)
        row.addStretch(1)
        pd.lay.addLayout(row)
        self.pad_lbl = label("—", "muted")
        self.pad_lbl.setWordWrap(True)
        pd.lay.addWidget(self.pad_lbl)
        g.addWidget(pd, 2, 1)

        # ---------------------------------------------------------- accel
        ac = Card("Akcelerometr", "Offset a škála - aspoň 6 poloh v klidu")
        t = label("Položte desku/raketu do klidu v libovolné poloze a stiskněte „Zachytit polohu“ "
                  "(2 s průměr; při pohybu se zachycení zopakuje). Nejlepší je 6 stran - každá osa "
                  "jednou nahoru a jednou dolů - ale fungují i jiné směry, jen ať jsou rozmanité. "
                  "Další polohy navíc přesnost zlepší.", "muted")
        t.setWordWrap(True)
        ac.lay.addWidget(t)
        row = QtWidgets.QHBoxLayout()
        self.acc_btn = button("Zachytit polohu", "primary")
        self.acc_btn.clicked.connect(self.acc_capture)
        row.addWidget(self.acc_btn)
        b = button("Smazat poslední")
        b.clicked.connect(self.acc_undo)
        row.addWidget(b)
        b = button("Smazat vše")
        b.clicked.connect(self.acc_reset)
        row.addWidget(b)
        row.addStretch(1)
        ac.lay.addLayout(row)
        self.acc_table = QtWidgets.QTableWidget(0, 4)
        self.acc_table.setHorizontalHeaderLabels(["X [g]", "Y [g]", "Z [g]", "|a| [g]"])
        self.acc_table.verticalHeader().setVisible(False)
        self.acc_table.setMinimumHeight(170)
        self.acc_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.Stretch)
        ac.lay.addWidget(self.acc_table)
        self.acc_lbl = label("Zachyceno 0 poloh.", "muted")
        self.acc_lbl.setWordWrap(True)
        ac.lay.addWidget(self.acc_lbl)
        self.acc_apply = button("Použít a uložit do flash", "primary")
        self.acc_apply.setEnabled(False)
        self.acc_apply.clicked.connect(self.acc_save)
        ac.lay.addWidget(self.acc_apply)
        g.addWidget(ac, 3, 0)

        # ---------------------------------------------------------- mag
        mg = Card("Magnetometr", "Hard iron + soft iron")
        t = label("Kalibrujte v sestavené raketě (železo a proudy v ní jsou to, co se kalibruje), "
                  "daleko od kovu a magnetů. „Start“, pak raketou pomalu otáčejte do všech směrů "
                  "(osmičky, převracení přes všechny strany) asi minutu, až pokrytí ≥ 85 %, pak "
                  "„Stop a spočítat“.", "muted")
        t.setWordWrap(True)
        mg.lay.addWidget(t)
        row = QtWidgets.QHBoxLayout()
        self.mag_btn = button("Start sběru", "primary")
        self.mag_btn.clicked.connect(self.mag_toggle)
        row.addWidget(self.mag_btn)
        row.addStretch(1)
        mg.lay.addLayout(row)
        self.mag_cov = QtWidgets.QProgressBar()
        self.mag_cov.setRange(0, 100)
        self.mag_cov.setFormat("pokrytí směrů %p %")
        mg.lay.addWidget(self.mag_cov)
        self.mag_lbl = label("—", "muted")
        self.mag_lbl.setWordWrap(True)
        mg.lay.addWidget(self.mag_lbl)
        self.mag_apply = button("Použít a uložit do flash", "primary")
        self.mag_apply.setEnabled(False)
        self.mag_apply.clicked.connect(self.mag_save)
        mg.lay.addWidget(self.mag_apply)
        g.addWidget(mg, 3, 1)

        # ---------------------------------------------------------- servos
        sv = Card("Serva", "Kontrola připojení")
        t = label("Master pohne jedním servem po druhém o 5° a zpět a sleduje proud z baterie. "
                  "Když proud vzroste aspoň o mst.servo_check_ma, servo je připojené. Spouští se "
                  "i samo po vytažení RBF (mst.servo_check_on_rbf, vypnout = 0); zastrčení počká "
                  "na konec. Jen v IDLE (ne v TEST), trvá ~9 s.", "muted")
        t.setWordWrap(True)
        sv.lay.addWidget(t)
        row = QtWidgets.QHBoxLayout()
        b = button("Zkontrolovat serva", "primary")
        b.clicked.connect(lambda: self.win._cmd("SERVOCHECK"))
        row.addWidget(b)
        self.srv_lbl = label("—", "muted")
        row.addWidget(self.srv_lbl)
        row.addStretch(1)
        sv.lay.addLayout(row)
        self.srv_table = QtWidgets.QTableWidget(len(SERVO_NAMES), 3)
        self.srv_table.setHorizontalHeaderLabels(["Servo", "Nárůst proudu", "Výsledek"])
        self.srv_table.verticalHeader().setVisible(False)
        self.srv_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.Stretch)
        self.srv_table.setMinimumHeight(290)
        for i, n in enumerate(SERVO_NAMES):
            for c_, txt in enumerate((n, "—", "—")):
                self.srv_table.setItem(i, c_, QtWidgets.QTableWidgetItem(txt))
        sv.lay.addWidget(self.srv_table)
        g.addWidget(sv, 4, 0, 1, 2)
        g.setColumnStretch(0, 1)
        g.setColumnStretch(1, 1)

    # ------------------------------------------------------------------ helpers
    def _need_raw(self) -> bool:
        if not self.win.link.is_open:
            QtWidgets.QMessageBox.warning(self, "Kalibrace", "Není připojena raketa.")
            return False
        if np is None:
            QtWidgets.QMessageBox.warning(self, "Kalibrace", "Chybí numpy: python -m pip install numpy")
            return False
        if not self.win.raw_btn.isChecked():
            self.win.raw_btn.setChecked(True)          # sends RAW 1
        return True

    def _need_link_raw(self) -> bool:
        """Like _need_raw(), for the steps that need no numpy."""
        if not self.win.link.is_open:
            QtWidgets.QMessageBox.warning(self, "Kalibrace", "Není připojena raketa.")
            return False
        if not self.win.raw_btn.isChecked():
            self.win.raw_btn.setChecked(True)          # sends RAW 1
        return True

    def _param_set(self, values: dict[str, float]) -> None:
        for k, v in values.items():
            self.win.link.send(f"PARAM SET {k} {v:.7g}")
        self.win.link.send("PARAM SAVE")
        self.win.log(f"> PARAM SET ×{len(values)} + PARAM SAVE")

    # ------------------------------------------------------------------ input
    def on_raw(self, d: dict) -> None:
        gyr = d.get("gyr") or [0, 0, 0]
        gm = math.sqrt(sum(c * c for c in gyr))
        self.gyro_lbl.setText(f"|ω| = {gm:.2f} °/s")
        goff = d.get("goff")
        if self.gyro_t0 is not None:
            self._gyro_poll(goff)
        elif goff is not None and time.monotonic() > self.gyro_msg_until:
            self.gyro_off_lbl.setText("Offset v platnosti: " + self._fmt_goff(goff) + " °/s")
        if self.acc_buf is not None:
            self.acc_buf.append((d.get("accr") or d.get("acc") or [0, 0, 0], gm))
            n = len(self.acc_buf)
            self.acc_lbl.setText(f"Zachytávám… {n}/{self.ACC_SAMPLES}")
            if n >= self.ACC_SAMPLES:
                self._acc_finish()
        if self.mag_on:
            m = d.get("mag") or [0, 0, 0]
            if d.get("mf", 1) and (not self.mag_pts or
                                   sum((a - b) ** 2 for a, b in zip(m, self.mag_pts[-1])) > 0.25):
                self.mag_pts.append(list(m))
                if len(self.mag_pts) % 10 == 0:
                    cov = sphere_coverage(self.mag_pts) if np is not None else 0.0
                    self.mag_cov.setValue(int(100 * cov))
                    self.mag_lbl.setText(f"Vzorků: {len(self.mag_pts)}")

    def on_status(self, P: dict) -> None:
        """P line (Position status), from the window's refresh."""
        if self.pad_buf is None:
            return
        if P and P.get("fix", 0) >= 3 and P.get("lat") and P.get("lon"):
            self.pad_buf.append((P["lat"] * 1e-7, P["lon"] * 1e-7, P.get("hdop", 99) / 10.0))
        left = self.PAD_SECONDS - (time.monotonic() - self.pad_t0)
        if left > 0:
            self.pad_lbl.setText(f"Průměruji GPS… {left:.0f} s, vzorků {len(self.pad_buf)}")
            return
        buf, self.pad_buf = self.pad_buf, None
        self.pad_btn.setEnabled(True)
        if len(buf) < 5:
            self.pad_lbl.setText("Bez 3D fixu GPS - poloha rampy NEnastavena.")
            return
        lat = sum(b[0] for b in buf) / len(buf)
        lon = sum(b[1] for b in buf) / len(buf)
        hd = sum(b[2] for b in buf) / len(buf)
        if QtWidgets.QMessageBox.question(
                self, "Rampa", f"Nastavit rampu na\n{lat:.7f}, {lon:.7f}\n(HDOP {hd:.1f}, {len(buf)} vzorků)\n"
                "a uložit do flash?") != QtWidgets.QMessageBox.StandardButton.Yes:
            self.pad_lbl.setText("Zrušeno.")
            return
        self._param_set({"pos.pad_lat_deg": lat, "pos.pad_lon_deg": lon})
        self.pad_lbl.setText(f"Rampa: {lat:.7f}, {lon:.7f} (uloženo)")

    def on_srvchk(self, d: dict) -> None:
        res, ma = d.get("res", []), d.get("ma", [])
        for i in range(min(len(SERVO_NAMES), len(res))):
            r = res[i]
            self.srv_table.item(i, 1).setText(f"{ma[i]:.0f} mA" if r else "—")
            it = self.srv_table.item(i, 2)
            it.setText({0: "—", 1: "připojeno ✓", 2: "NEREAGUJE ✗"}.get(r, "?"))
            it.setForeground(QtCore.Qt.GlobalColor.green if r == 1 else
                             (QtCore.Qt.GlobalColor.red if r == 2 else QtCore.Qt.GlobalColor.gray))
        if d.get("run"):
            self.srv_lbl.setText(f"Probíhá… servo {d.get('ch', 0) + 1}/8 (práh {d.get('thr', 0):.0f} mA)")
        else:
            bad = [SERVO_NAMES[i] for i, r in enumerate(res) if r == 2]
            self.srv_lbl.setText(("Nereaguje: " + ", ".join(bad)) if bad else
                                 ("Všechna serva reagují." if d.get("done") else "—"))

    # ------------------------------------------------------------------ gyro
    @staticmethod
    def _fmt_goff(g) -> str:
        return " / ".join(f"{v:+.3f}".replace(".", ",") for v in g)

    def gyro_start(self) -> None:
        if not self._need_link_raw():
            return
        self.win._cmd("CAL GYRO")
        self.gyro_t0 = time.monotonic()
        self.gyro_btn.setEnabled(False)
        self.gyro_off_lbl.setText("Kalibruji… nehýbat.")
        # also when the R lines never come (old firmware, raw stream off)
        QtCore.QTimer.singleShot(int(self.GYRO_GIVE_UP_S * 1000) + 100, lambda: self._gyro_poll(None))

    def _gyro_poll(self, goff) -> None:
        if self.gyro_t0 is None:
            return
        dt = time.monotonic() - self.gyro_t0
        if goff is not None and dt >= self.GYRO_SETTLE_S:
            self.gyro_t0 = None
            self.gyro_btn.setEnabled(True)
            if all(abs(v) < 1e-6 for v in goff):
                self.gyro_off_lbl.setText("Deska hlásí nulový offset - neuloženo (kalibrace neproběhla?).")
                return
            g = [max(-10.0, min(10.0, float(v))) for v in goff]
            self._param_set({"imu.gyr_off_x": g[0], "imu.gyr_off_y": g[1], "imu.gyr_off_z": g[2]})
            self.gyro_off_lbl.setText("Offset " + self._fmt_goff(g) + " °/s uložen do flash ✓")
            self.gyro_msg_until = time.monotonic() + 15.0
        elif dt >= self.GYRO_GIVE_UP_S:
            self.gyro_t0 = None
            self.gyro_btn.setEnabled(True)
            self.gyro_off_lbl.setText("Kalibrace proběhla, ale deska offset nehlásí (firmware < 3.27?) - "
                                      "NEuloženo, platí jen do vypnutí.")

    # ------------------------------------------------------------------ pad
    def pad_start(self) -> None:
        if not self.win.link.is_open:
            return
        self.pad_buf = []
        self.pad_t0 = time.monotonic()
        self.pad_btn.setEnabled(False)

    def pad_clear(self) -> None:
        if self.win.link.is_open:
            self._param_set({"pos.pad_lat_deg": 0.0, "pos.pad_lon_deg": 0.0})
            self.pad_lbl.setText("Poloha rampy zrušena: počátek = poloha při zastrčení.")

    # ------------------------------------------------------------------ accel
    def acc_capture(self) -> None:
        if not self._need_raw():
            return
        self.acc_buf = []
        self.acc_btn.setEnabled(False)

    def _acc_finish(self) -> None:
        buf, self.acc_buf = self.acc_buf, None
        self.acc_btn.setEnabled(True)
        A = np.array([b[0] for b in buf], dtype=float)
        gmax = max(b[1] for b in buf)
        sd = A.std(axis=0).max()
        if sd > 0.01 or gmax > 3.0:
            self.acc_lbl.setText(f"Deska se hýbala (σ {sd:.3f} g, |ω| až {gmax:.1f} °/s) - zachyťte znovu.")
            return
        m = A.mean(axis=0)
        u = m / np.linalg.norm(m)
        for p in self.acc_points:
            q = np.array(p) / np.linalg.norm(p)
            if float(u @ q) > math.cos(math.radians(20)):
                self.acc_lbl.setText("Skoro stejná poloha jako už zachycená - otočte desku jinak.")
                return
        self.acc_points.append([float(c) for c in m])
        self._acc_refresh()

    def acc_undo(self) -> None:
        if self.acc_points:
            self.acc_points.pop()
        self._acc_refresh()

    def acc_reset(self) -> None:
        self.acc_points.clear()
        self._acc_refresh()

    def _acc_refresh(self) -> None:
        self.acc_table.setRowCount(len(self.acc_points))
        for r, p in enumerate(self.acc_points):
            for c_, v in enumerate(p + [math.sqrt(sum(x * x for x in p))]):
                self.acc_table.setItem(r, c_, QtWidgets.QTableWidgetItem(f"{v:+.4f}"))
        self.acc_fit = None
        self.acc_apply.setEnabled(False)
        n = len(self.acc_points)
        if n < 6:
            self.acc_lbl.setText(f"Zachyceno {n} poloh, potřeba aspoň 6.")
            return
        try:
            f = fit_accel(self.acc_points)
        except ValueError as exc:
            self.acc_lbl.setText(f"Zachyceno {n} poloh. {exc}")
            return
        ok = f["resid_rms"] < 0.01 and f["cond"] < 1e5 and np.all(np.abs(f["off"]) < 0.5) \
            and np.all(np.abs(f["scale"] - 1) < 0.2)
        self.acc_fit = f
        o, s = f["off"], f["scale"]
        exact = "  (se 6 polohami vyjde chyba vždy 0 - 7. a další poloha fit ověří)" if n == 6 else ""
        self.acc_lbl.setText(
            f"Zachyceno {n} poloh. Offset {o[0]:+.4f} / {o[1]:+.4f} / {o[2]:+.4f} g, "
            f"škála {s[0]:.4f} / {s[1]:.4f} / {s[2]:.4f}. Zbytková chyba |a|−1: "
            f"{1000 * f['resid_rms']:.1f} mg" + exact + ("" if ok else
            "  ⚠ fit je nejistý (polohy málo rozmanité nebo se deska hýbala) - přidejte polohy."))
        self.acc_apply.setEnabled(bool(ok))

    def acc_save(self) -> None:
        f = self.acc_fit
        if not f:
            return
        o, s = f["off"], f["scale"]
        self._param_set({"imu.acc_off_x": o[0], "imu.acc_off_y": o[1], "imu.acc_off_z": o[2],
                         "imu.acc_scale_x": s[0], "imu.acc_scale_y": s[1], "imu.acc_scale_z": s[2]})
        self.acc_lbl.setText(self.acc_lbl.text() + "  → uloženo.")

    # ------------------------------------------------------------------ mag
    def mag_toggle(self) -> None:
        if not self.mag_on:
            if not self._need_raw():
                return
            self.mag_pts = []
            self.mag_fit = None
            self.mag_apply.setEnabled(False)
            self.mag_cov.setValue(0)
            self.mag_on = True
            self.mag_btn.setText("Stop a spočítat")
            self.mag_lbl.setText("Otáčejte raketou do všech směrů…")
            return
        self.mag_on = False
        self.mag_btn.setText("Start sběru")
        try:
            f = fit_mag(self.mag_pts)
        except ValueError as exc:
            self.mag_lbl.setText(f"Vzorků {len(self.mag_pts)}: {exc}")
            return
        cov = sphere_coverage(self.mag_pts)
        self.mag_cov.setValue(int(100 * cov))
        h, S = f["hard"], f["soft"]
        ok = cov >= 0.85 and f["resid_after"] < 0.05 and 15.0 < f["field"] < 80.0
        self.mag_fit = f
        self.mag_lbl.setText(
            f"Vzorků {len(self.mag_pts)}, pokrytí {100 * cov:.0f} %. Hard iron {h[0]:+.1f} / {h[1]:+.1f} / "
            f"{h[2]:+.1f} µT, pole {f['field']:.1f} µT. Rozptyl |B|: {100 * f['resid_before']:.1f} % → "
            f"{100 * f['resid_after']:.1f} %. Soft iron:\n" +
            "\n".join("   " + "  ".join(f"{S[r][c]:+.4f}" for c in range(3)) for r in range(3)) +
            ("" if ok else "\n⚠ málo pokrytí nebo nevěrohodné pole - nasbírejte znovu, víc směrů."))
        self.mag_apply.setEnabled(bool(ok))

    def mag_save(self) -> None:
        f = self.mag_fit
        if not f:
            return
        h, S = f["hard"], f["soft"]
        v = {"imu.mag_hi_x": h[0], "imu.mag_hi_y": h[1], "imu.mag_hi_z": h[2]}
        for r in range(3):
            for c in range(3):
                v[f"imu.mag_si_{r}{c}"] = S[r][c]
        self._param_set(v)
        self.mag_lbl.setText(self.mag_lbl.text() + "\n→ uloženo.")


def parse_srvchk(line: str) -> dict | None:
    try:
        return json.loads(line.split(" ", 1)[1])
    except (IndexError, ValueError):
        return None
