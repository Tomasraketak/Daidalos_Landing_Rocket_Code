"""
"Zážeh a LandSim" tab of the diagnostic console.

Where the ignition altitude comes from, and how to change it:

  1. Enter the vehicle as it really is (mass, propellant, Cd, motor scale,
     diameter, igniter pad). These are the g.* parameters.
  2. "Spočítat v LandSim_Light" runs tools/gen_guidance_tables.py against
     your local LandSim_Light checkout with that vehicle. The simulation
     flies its own receding-horizon landing controller from every ignition
     altitude and fall speed and returns the BAND of altitudes [h_min, h_max]
     from which the landing still closes, against the fall speed |vz|.
  3. "Odeslat do rakety" sends the vehicle and the band (tbl.* parameters);
     "Uložit do flash" on the parameter tab makes it permanent.

In flight the Navigation MCU propagates its real fall with the drag it
measures, ten times a second, and ignites where that fall enters the band
(plus the igniter pad). The preview table on this tab runs the same
calculation here, so you can see the ignition altitude for each apogee
before you fly.
"""
from __future__ import annotations

import json
import math
import os
import re
import sys
import tempfile

from PyQt6 import QtCore, QtGui, QtWidgets

import actuator_gui as ag
from actuator_gui import button, label, Card

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
GEN = os.path.join(ROOT, "tools", "gen_guidance_tables.py")
TABLES_H = os.path.join(ROOT, "Ariadna_V2", "common", "guidance_tables.h")

VEHICLE_KEYS = [
    ("g.mass_gross_kg", "Hmotnost s palivem", "kg"),
    ("g.mass_prop_kg", "Palivo přistávacího motoru", "kg"),
    ("g.cd_body", "Cd trupu (bez křidélek)", "-"),
    ("g.thrust_scale", "Násobek tahové křivky", "×"),
    ("g.diameter_m", "Průměr", "m"),
    ("g.ign_delay_pad_s", "Rezerva na zpoždění roznětu", "s"),
    ("g.plan_target_mps", "Cílová rychlost dosednutí", "m/s"),
    ("fin.brake", "Křidélka jako brzda (0/1)", ""),
    ("g.fin_dcd_zero", "ΔCd křidélek, neutrál (z LandSim)", "-"),
    ("g.fin_dcd_brake", "ΔCd křidélek, aerobrzda (z LandSim)", "-"),
]
N_TBL = 32

# vehicle.json for landsim_export.py: our parameter key -> LandSim's key
LANDSIM_KEYS = {
    "g.mass_gross_kg": "mass_gross_kg", "g.mass_prop_kg": "mass_prop_kg",
    "g.cd_body": "cd_body", "g.thrust_scale": "thrust_scale",
    "g.diameter_m": "diameter_m", "g.ign_delay_pad_s": "delay_pad_s",
    "g.plan_target_mps": "plan_target",
}
# geometry LandSim needs but the rocket does not (0 = LandSim's default)
EXTRA_KEYS = [
    ("fin_area_m2", "Plocha jednoho křidélka", "m²", 4),
    ("fin_arm_m", "Rameno křidélek (těžiště → křidélko)", "m", 3),
    ("fin_stall_deg", "Úhel odtržení křidélka", "°", 1),
    ("inertia_t", "Moment setrvačnosti příčný (plná)", "kg·m²", 4),
    ("inertia_r", "Moment setrvačnosti podélný (plná)", "kg·m²", 5),
    ("ign_delay_max_s", "Max. zpoždění roznětu", "s", 3),
]
# band.json "gains" -> parameter keys
GAIN_KEYS = {"wn_tvc": "g.wn_tvc", "zeta_tvc": "g.zeta_tvc", "wn_fin": "g.wn_fin",
             "zeta_fin": "g.zeta_fin", "roll_gain": "g.roll_gain", "sched_tvc": "g.sched_tvc",
             "sched_fin": "g.sched_fin", "att_ki": "g.att_ki"}


def fill_no_band(hmin: list, hmax: list) -> list:
    """landsim_export writes h_min = 0 where no altitude lands. Replace it with
    the best effort the Navigation MCU would use anyway (hmin_at() in
    guidance.cpp): the last positive h_min extrapolated along its slope."""
    out = list(hmin)
    for i, v in enumerate(out):
        if v > 0:
            continue
        j = i
        while j > 0 and out[j] <= 0:
            j -= 1
        if out[j] <= 0:
            continue
        slope = max(out[j] - out[j - 1], 0.0) if j > 0 and out[j - 1] > 0 else 0.0
        out[i] = out[j] + slope * (i - j)
    return out


def header_const(name: str, default: float) -> float:
    try:
        m = re.search(rf"#define\s+{name}\s+([-\d.eE]+)f?", open(TABLES_H, encoding="utf-8").read())
        return float(m.group(1)) if m else default
    except OSError:
        return default


FIN_CD_ZERO = header_const("DG_FIN_CD_ZERO", 0.143)
FIN_CD_BRAKE = header_const("DG_FIN_CD_BRAKE", 0.447)


def band_at(tbl: dict, vz_abs: float):
    """dg_ign_band(): (h_min, h_max) for a fall speed, 0 = no band."""
    vz0, dvz = tbl["vz0"], max(tbl["dvz"], 0.1)
    f = min(max((vz_abs - vz0) / dvz, 0.0), N_TBL - 1)
    i = min(int(f), N_TBL - 2)
    t = f - i
    lo = tbl["hmin"][i] + t * (tbl["hmin"][i + 1] - tbl["hmin"][i])
    a, b = tbl["hmax"][i], tbl["hmax"][i + 1]
    hi = 0.0 if (a <= 0 or b <= 0) else a + t * (b - a)
    return lo, hi


def cd_free_of(veh: dict) -> float:
    """Free-fall Cd with the airbrake fully out, as the rocket models it."""
    z = veh.get("g.fin_dcd_zero") or FIN_CD_ZERO
    b = veh.get("g.fin_dcd_brake") or FIN_CD_BRAKE
    return veh["g.cd_body"] + (b if veh.get("fin.brake", 1) > 0.5 else z)


def ign_compute(tbl: dict, veh: dict, h0: float, vz0: float = 0.0, vh0: float = 0.0,
                drag_scale: float = 1.0):
    """The Navigation MCU's dg_ign_compute(), line for line. Returns
    (h_cmd, band_lo, band_hi, vz at h_cmd, time to h_cmd)."""
    cd_free = cd_free_of(veh)
    area = math.pi * (veh["g.diameter_m"] / 2) ** 2
    kd = 0.5 * 1.225 * area * cd_free * min(max(drag_scale, 0.4), 2.5) / veh["g.mass_gross_kg"]
    dt, g = 0.005, 9.80665

    def step(h, vx, vz):
        v = math.hypot(vx, vz)
        ax, az = -kd * v * vx, -kd * v * vz - g
        vxm, vzm = vx + 0.5 * ax * dt, vz + 0.5 * az * dt
        vm = math.hypot(vxm, vzm)
        return h + vzm * dt, vx - kd * vm * vxm * dt, vz + (-kd * vm * vzm - g) * dt

    h, vx, vz = h0, abs(vh0), vz0
    h_lo, v_lo, h_hi, no_band = None, 0.0, None, False
    _, hmx = band_at(tbl, abs(vz))
    if hmx > 0 and h <= hmx:
        h_hi = h
    for _ in range(8000):
        if h <= 0.5:
            break
        hmin, hmx = band_at(tbl, abs(vz))
        if h_hi is None and hmx > 0 and h <= hmx:
            h_hi = h
        if h <= hmin:
            h_lo, v_lo, no_band = h, vz, hmx <= 0
            break
        h, vx, vz = step(h, vx, vz)
    if h_lo is None:
        h_lo, v_lo = max(h, 0.5), vz
    d = veh["g.ign_delay_pad_s"]
    h_cmd = h_lo + abs(v_lo) * d + 0.5 * g * d * d
    cap = h0 - 1.0 if (no_band or h_hi is None) else h_hi
    cap = max(cap, h_lo)
    h_cmd = min(h_cmd, cap)
    # speed and time at h_cmd
    h, vx, vz, t = h0, abs(vh0), vz0, 0.0
    for _ in range(8000):
        if h <= h_cmd:
            break
        h, vx, vz = step(h, vx, vz)
        t += dt
    return h_cmd, h_lo, (0.0 if (no_band or h_hi is None) else h_hi), vz, t


class BandPlot(QtWidgets.QWidget):
    """The ignition band against the fall speed, and one fall drawn into it."""

    def __init__(self):
        super().__init__()
        self.setMinimumSize(420, 300)
        self.tables: list[tuple[str, dict, str]] = []     # (name, table, colour)
        self.fall: list[tuple[float, float]] = []          # (|vz|, h)
        self.cmd: tuple[float, float] | None = None

    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), QtGui.QColor("#101418"))
        w, h = self.width(), self.height()
        L, T, R, B = 48, 14, 12, 34
        pw, ph = w - L - R, h - T - B
        vmax, hmax = 66.0, 100.0
        X = lambda v: L + pw * v / vmax
        Y = lambda hh: T + ph * (1 - min(hh, hmax) / hmax)
        f = p.font()
        f.setPointSize(8)
        p.setFont(f)
        for v in range(0, 70, 10):
            p.setPen(QtGui.QPen(QtGui.QColor("#2a343c"), 1))
            p.drawLine(QtCore.QPointF(X(v), T), QtCore.QPointF(X(v), T + ph))
            p.setPen(QtGui.QColor("#9fb0bb"))
            p.drawText(QtCore.QPointF(X(v) - 6, T + ph + 14), f"{v}")
        for hh in range(0, 101, 20):
            p.setPen(QtGui.QPen(QtGui.QColor("#2a343c"), 1))
            p.drawLine(QtCore.QPointF(L, Y(hh)), QtCore.QPointF(L + pw, Y(hh)))
            p.setPen(QtGui.QColor("#9fb0bb"))
            p.drawText(QtCore.QPointF(6, Y(hh) + 4), f"{hh} m")
        p.drawText(QtCore.QPointF(L + pw / 2 - 70, h - 4), "rychlost pádu při zážehu |vz| [m/s]")

        for name, tbl, col in self.tables:
            c = QtGui.QColor(col)
            vzs = [tbl["vz0"] + tbl["dvz"] * i for i in range(N_TBL)]
            # feasible area
            poly = QtGui.QPolygonF()
            for v, lo, hi in zip(vzs, tbl["hmin"], tbl["hmax"]):
                if hi > 0:
                    poly.append(QtCore.QPointF(X(v), Y(hi)))
            for v, lo, hi in reversed(list(zip(vzs, tbl["hmin"], tbl["hmax"]))):
                if hi > 0:
                    poly.append(QtCore.QPointF(X(v), Y(lo)))
            fill = QtGui.QColor(c)
            fill.setAlpha(45)
            p.setPen(QtCore.Qt.PenStyle.NoPen)
            p.setBrush(fill)
            p.drawPolygon(poly)
            p.setBrush(QtCore.Qt.BrushStyle.NoBrush)
            pen = QtGui.QPen(c, 1.6)
            p.setPen(pen)
            for arr in (tbl["hmin"], tbl["hmax"]):
                path = QtGui.QPainterPath()
                first = True
                for v, hv in zip(vzs, arr):
                    if hv <= 0:
                        first = True
                        continue
                    if first:
                        path.moveTo(X(v), Y(hv))
                        first = False
                    else:
                        path.lineTo(X(v), Y(hv))
                p.drawPath(path)
        y_leg = T + 4
        for name, _, col in self.tables:
            p.setPen(QtGui.QColor(col))
            p.drawText(QtCore.QPointF(L + pw - 190, y_leg + 10), "■ " + name)
            y_leg += 14
        if self.fall:
            path = QtGui.QPainterPath()
            path.moveTo(X(self.fall[0][0]), Y(self.fall[0][1]))
            for v, hh in self.fall[1:]:
                path.lineTo(X(v), Y(hh))
            p.setPen(QtGui.QPen(QtGui.QColor("#e8eef2"), 1.2, QtCore.Qt.PenStyle.DashLine))
            p.drawPath(path)
        if self.cmd:
            p.setPen(QtCore.Qt.PenStyle.NoPen)
            p.setBrush(QtGui.QColor("#ff7a1a"))
            p.drawEllipse(QtCore.QPointF(X(self.cmd[0]), Y(self.cmd[1])), 5, 5)
        p.end()


class IgnitionTab(QtWidgets.QWidget):
    def __init__(self, win):
        super().__init__()
        self.win = win
        self.computed: dict | None = None
        self.gains: dict | None = None
        self.fin_dcd: tuple | None = None
        self.last_vehicle: dict | None = None
        self.mode = "export"
        self.settings = QtCore.QSettings("VoltinoLabs", "DaidalosDiag")
        lay = QtWidgets.QGridLayout(self)
        lay.setContentsMargins(20, 16, 20, 20)
        lay.setSpacing(16)

        # ---- vehicle
        vc = Card("Vozidlo", "Parametry pro výpočet zážehu")
        note = label("Tyto hodnoty čte i raketa (g.*). Po změně přepočítejte tabulku v LandSim "
                     "a obojí odešlete. Cd se za letu navíc měří (g.drag_adapt), takže "
                     "chyba v Cd posune hlavně odhad, ne přistání.", "muted")
        note.setWordWrap(True)
        vc.lay.addWidget(note)
        form = QtWidgets.QFormLayout()
        self.fields: dict[str, QtWidgets.QDoubleSpinBox] = {}
        for key, name, unit in VEHICLE_KEYS:
            sp = QtWidgets.QDoubleSpinBox()
            sp.setDecimals(3)
            sp.setRange(-10, 1000)
            sp.setSingleStep(0.01)
            sp.setSuffix(f" {unit}" if unit else "")
            sp.valueChanged.connect(self.preview)
            self.fields[key] = sp
            form.addRow(name, sp)
        vc.lay.addLayout(form)
        row = QtWidgets.QHBoxLayout()
        b = button("Načíst z tabulky parametrů", tip="Vezme hodnoty z rakety / tabulky parametrů")
        b.clicked.connect(self.load_from_params)
        row.addWidget(b)
        row.addStretch(1)
        vc.lay.addLayout(row)
        lay.addWidget(vc, 0, 0)

        # ---- LandSim
        sc = Card("LandSim_Light", "Výpočet pásma zážehu")
        r = QtWidgets.QHBoxLayout()
        # DAIDALOS_LANDSIM is set by the Windows launcher (tools/windows), which
        # keeps LandSim_Light up to date next to the repository; it wins over a
        # folder remembered from an older checkout.
        env_sim = os.environ.get("DAIDALOS_LANDSIM", "")
        default_sim = env_sim if env_sim and os.path.isdir(env_sim) else \
            self.settings.value("landsim_path", os.path.join(ROOT, "..", "LandSim_Light"))
        self.sim_path = QtWidgets.QLineEdit(default_sim)
        self.sim_path.setToolTip("Složka s tvc_sim.py (git clone https://github.com/Tomasraketak/LandSim_Light)")
        r.addWidget(label("Složka", "muted"))
        r.addWidget(self.sim_path, 1)
        b = button("…")
        b.clicked.connect(self.browse)
        r.addWidget(b)
        sc.lay.addLayout(r)
        ef = QtWidgets.QFormLayout()
        self.extra: dict[str, QtWidgets.QDoubleSpinBox] = {}
        for key, name, unit, dec in EXTRA_KEYS:
            sp = QtWidgets.QDoubleSpinBox()
            sp.setDecimals(dec)
            sp.setRange(0, 100)
            sp.setSingleStep(10 ** -dec)
            sp.setSpecialValueText("výchozí LandSim")
            sp.setSuffix(f" {unit}")
            try:
                sp.setValue(float(self.settings.value(f"landsim/{key}", 0.0)))
            except (TypeError, ValueError):
                pass
            self.extra[key] = sp
            ef.addRow(name, sp)
        sc.lay.addLayout(ef)
        r = QtWidgets.QHBoxLayout()
        r.addWidget(label("Tahová křivka (CSV t,N)", "muted"))
        self.curve_path = QtWidgets.QLineEdit(self.settings.value("landsim/curve", ""))
        self.curve_path.setPlaceholderText("prázdné = křivka v LandSim")
        r.addWidget(self.curve_path, 1)
        b = button("…")
        b.clicked.connect(self.browse_curve)
        r.addWidget(b)
        sc.lay.addLayout(r)
        self.fit_gains = QtWidgets.QCheckBox("Přefitovat zisky regulátorů (--gains, řádově minuty)")
        sc.lay.addWidget(self.fit_gains)
        r = QtWidgets.QHBoxLayout()
        self.run_btn = button("Spočítat v LandSim_Light", "primary",
                              "Spustí landsim_export.py s tímto vozidlem (~15–60 s, se zisky déle)")
        self.run_btn.clicked.connect(self.run_sim)
        r.addWidget(self.run_btn)
        self.send_btn = button("Odeslat vozidlo + tabulku do rakety", "primary",
                               "PARAM SET pro g.*, ΔCd křidélek, tbl.* (a zisky, pokud byly "
                               "přefitovány); pak Uložit do flash")
        self.send_btn.clicked.connect(self.send)
        self.send_btn.setEnabled(False)
        r.addWidget(self.send_btn)
        b = button("Uložit do flash", tip="PARAM SAVE")
        b.clicked.connect(lambda: self.win._cmd("PARAM SAVE"))
        r.addWidget(b)
        b = button("Uložit vehicle.json…", tip="Popis vozidla pro LandSim (landsim_export.py, "
                   "make tables VEHICLE=...) - pro přegenerování celé tabulky v firmwaru")
        b.clicked.connect(self.save_vehicle)
        r.addWidget(b)
        r.addStretch(1)
        sc.lay.addLayout(r)
        self.sim_log = QtWidgets.QPlainTextEdit()
        self.sim_log.setReadOnly(True)
        self.sim_log.setMaximumBlockCount(400)
        self.sim_log.setMinimumHeight(170)
        sc.lay.addWidget(self.sim_log)
        lay.addWidget(sc, 1, 0)

        # ---- band plot
        pc = Card("Pásmo", "Odkud ještě jde přistát")
        expl = label("Plocha = výšky zážehu, ze kterých simulace přistane (pod 2,3 m/s), podle "
                     "rychlosti pádu. Čárkovaně pád z apogea zvoleného níže, oranžově kde raketa "
                     "zapálí (vstup do pásma + rezerva na zpoždění roznětu, ne nad horní hranou).", "muted")
        expl.setWordWrap(True)
        pc.lay.addWidget(expl)
        self.plot = BandPlot()
        pc.lay.addWidget(self.plot, 1)
        r = QtWidgets.QHBoxLayout()
        r.addWidget(label("Apogeum", "muted"))
        self.apo = QtWidgets.QDoubleSpinBox()
        self.apo.setRange(30, 400)
        self.apo.setValue(150)
        self.apo.setSuffix(" m")
        self.apo.valueChanged.connect(self.preview)
        r.addWidget(self.apo)
        r.addWidget(label("drift", "muted"))
        self.drift = QtWidgets.QDoubleSpinBox()
        self.drift.setRange(0, 20)
        self.drift.setSuffix(" m/s")
        self.drift.valueChanged.connect(self.preview)
        r.addWidget(self.drift)
        r.addWidget(label("Cd měřené / model", "muted"))
        self.dscale = QtWidgets.QDoubleSpinBox()
        self.dscale.setRange(0.4, 2.5)
        self.dscale.setSingleStep(0.05)
        self.dscale.setValue(1.0)
        self.dscale.valueChanged.connect(self.preview)
        r.addWidget(self.dscale)
        r.addStretch(1)
        pc.lay.addLayout(r)
        self.result = label("—", "big")
        pc.lay.addWidget(self.result)
        lay.addWidget(pc, 0, 1, 2, 1)

        # ---- apogee preview table
        tc = Card("Náhled", "Výška zážehu podle apogea")
        self.ptab = QtWidgets.QTableWidget(0, 6)
        self.ptab.setHorizontalHeaderLabels(["apogeum", "zážeh", "pásmo od", "pásmo do",
                                             "|vz| při zážehu", "čas do zážehu"])
        self.ptab.verticalHeader().setVisible(False)
        self.ptab.horizontalHeader().setStretchLastSection(True)
        tc.lay.addWidget(self.ptab)
        lay.addWidget(tc, 2, 0, 1, 2)
        lay.setColumnStretch(0, 1)
        lay.setColumnStretch(1, 1)

        self.proc = QtCore.QProcess(self)
        self.proc.readyReadStandardError.connect(self._proc_out)
        self.proc.readyReadStandardOutput.connect(self._proc_out)
        self.proc.finished.connect(self._proc_done)
        self.tmp = tempfile.mkdtemp(prefix="daidalos_ign_")
        self.load_from_params()

    # -------------------------------------------------------------- data
    def params_by_key(self) -> dict:
        return {p["key"]: self.win.param_edits.get(p["id"], p["val"]) for p in self.win.params}

    def board_table(self) -> dict | None:
        pk = self.params_by_key()
        if "tbl.vz0" not in pk:
            return None
        return {"vz0": pk["tbl.vz0"], "dvz": pk["tbl.dvz"],
                "hmin": [pk.get(f"tbl.hmin_{i:02d}", 0.0) for i in range(N_TBL)],
                "hmax": [pk.get(f"tbl.hmax_{i:02d}", 0.0) for i in range(N_TBL)]}

    def vehicle(self) -> dict:
        return {k: self.fields[k].value() for k, _, _ in VEHICLE_KEYS}

    def load_from_params(self) -> None:
        pk = self.params_by_key()
        for k, sp in self.fields.items():
            if k in pk:
                sp.blockSignals(True)
                sp.setValue(pk[k])
                sp.blockSignals(False)
        self.preview()

    def browse_curve(self) -> None:
        f, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Tahová křivka", ag.user_dir(), "CSV (*.csv *.txt)")
        if f:
            self.curve_path.setText(f)

    def landsim_vehicle(self) -> dict:
        """vehicle.json in landsim_export.py's words. Daidalos flies NO D9
        booster: LandSim's default rocket carries one, so say so explicitly."""
        veh = self.vehicle()
        out = {"comment": "written by the Daidalos diagnostic console",
               "n_boosters": 0, "use_booster_rule": False,
               "fin_enable": 1, "fin_brake": 1 if veh.get("fin.brake", 1) > 0.5 else 0}
        for k, lk in LANDSIM_KEYS.items():
            out[lk] = float(veh[k])
        it = {}
        for key, sp in self.extra.items():
            self.settings.setValue(f"landsim/{key}", sp.value())
            if sp.value() <= 0:
                continue
            if key == "inertia_t":
                it["transverse"] = sp.value()
            elif key == "inertia_r":
                it["roll"] = sp.value()
            else:
                out[key] = sp.value()
        if it:
            out["inertia"] = it
        cp = self.curve_path.text().strip()
        self.settings.setValue("landsim/curve", cp)
        if cp:
            rows = []
            with open(cp, encoding="utf-8") as fh:
                for ln in fh:
                    parts = re.split(r"[,;\s]+", ln.strip())
                    try:
                        rows.append([float(parts[0]), float(parts[1])])
                    except (ValueError, IndexError):
                        continue                    # header / comment line
            if len(rows) < 3:
                raise ValueError(f"{cp}: méně než 3 řádky t,N")
            out["motor_curve"] = rows
        return out

    def save_vehicle(self) -> None:
        try:
            veh = self.landsim_vehicle()
        except (OSError, ValueError) as exc:
            QtWidgets.QMessageBox.warning(self, "LandSim", str(exc))
            return
        f, _ = QtWidgets.QFileDialog.getSaveFileName(self, "vehicle.json", ag.user_path("vehicle.json"), "JSON (*.json)")
        if f:
            with open(f, "w", encoding="utf-8") as fh:
                json.dump(veh, fh, indent=1)
            self.sim_log.appendPlainText(f"# uloženo {f}")

    def browse(self) -> None:
        d = QtWidgets.QFileDialog.getExistingDirectory(self, "Složka LandSim_Light", self.sim_path.text())
        if d:
            self.sim_path.setText(d)

    # -------------------------------------------------------------- simulation
    def _python_ready(self) -> bool:
        """LandSim needs numpy in the SAME interpreter that runs this GUI
        (the computation is started with sys.executable). Say so in plain
        words instead of leaving a traceback in the log."""
        import importlib.util
        if importlib.util.find_spec("numpy") is not None:
            return True
        cmd = f'"{sys.executable}" -m pip install numpy'
        self.sim_log.clear()
        self.sim_log.appendPlainText("# CHYBA: v Pythonu, ve kterém běží diagnostika, chybí numpy "
                                     "(LandSim_Light ho potřebuje).")
        self.sim_log.appendPlainText(f"# {sys.executable}")
        self.sim_log.appendPlainText("# Nainstalujte ho (pak diagnostiku restartujte):")
        self.sim_log.appendPlainText(cmd)
        QtWidgets.QMessageBox.warning(self, "LandSim", "Chybí balík numpy.\n\nSpusťte v příkazovém "
                                      f"řádku:\n{cmd}\n\na diagnostiku restartujte.")
        return False

    def run_sim(self) -> None:
        sim = self.sim_path.text().strip()
        if not self._python_ready():
            return
        exporter = os.path.join(sim, "landsim_export.py")
        if os.path.isfile(exporter):
            self.settings.setValue("landsim_path", sim)
            try:
                veh = self.landsim_vehicle()
            except (OSError, ValueError) as exc:
                QtWidgets.QMessageBox.warning(self, "LandSim", str(exc))
                return
            self.last_vehicle = veh
            vpath = os.path.join(self.tmp, "vehicle.json")
            self.out_path = os.path.join(self.tmp, "band.json")
            with open(vpath, "w", encoding="utf-8") as fh:
                json.dump(veh, fh, indent=1)
            if os.path.exists(self.out_path):
                os.remove(self.out_path)
            args = [exporter, "--vehicle", vpath, "--out", self.out_path, "--n", str(N_TBL)]
            if self.fit_gains.isChecked():
                args.append("--gains")
            self.mode = "export"
            self.sim_log.clear()
            self.sim_log.appendPlainText("> landsim_export.py " + " ".join(args[1:]))
            self.run_btn.setEnabled(False)
            self.proc.setWorkingDirectory(sim)
            self.proc.start(sys.executable, args)
            return
        if not os.path.isfile(os.path.join(sim, "tvc_sim.py")):
            QtWidgets.QMessageBox.warning(self, "LandSim", f"V {sim} není tvc_sim.py.\n\n"
                                          "git clone https://github.com/Tomasraketak/LandSim_Light\n"
                                          "python -m pip install -r requirements.txt")
            return
        self.settings.setValue("landsim_path", sim)
        vpath = os.path.join(self.tmp, "vehicle.json")
        self.out_path = os.path.join(self.tmp, "table.json")
        with open(vpath, "w", encoding="utf-8") as fh:
            json.dump(self.vehicle(), fh)
        if os.path.exists(self.out_path):
            os.remove(self.out_path)
        self.sim_log.clear()
        self.sim_log.appendPlainText(f"> {os.path.basename(GEN)} --sim {sim}   "
                                     "(starý LandSim bez landsim_export.py)")
        self.sim_log.appendPlainText("# Tento LandSim_Light je starý: bez landsim_export.py se nespočítá "
                                     "ΔCd křidélek ani zisky. Aktualizujte ho: v jeho složce "
                                     "'git pull origin main'.")
        self.mode = "bridge"
        self.run_btn.setEnabled(False)
        self.proc.start(sys.executable, [GEN, "--sim", sim, "--vehicle", vpath,
                                         "--table-json", self.out_path])

    def _proc_out(self) -> None:
        txt = bytes(self.proc.readAllStandardError()).decode(errors="replace") + \
              bytes(self.proc.readAllStandardOutput()).decode(errors="replace")
        for ln in txt.splitlines():
            if ln.strip():
                self.sim_log.appendPlainText(ln)

    def _proc_done(self, code: int, _status) -> None:
        self.run_btn.setEnabled(True)
        if code != 0 or not os.path.exists(self.out_path):
            self.sim_log.appendPlainText(f"# CHYBA (kód {code})")
            return
        with open(self.out_path, encoding="utf-8") as fh:
            d = json.load(fh)
        self.gains = None
        self.fin_dcd = None
        if getattr(self, "mode", "bridge") == "export":
            if d.get("schema") != 1 or int(d.get("n", 0)) != N_TBL:
                self.sim_log.appendPlainText(f"# CHYBA: band.json schema {d.get('schema')} / n "
                                             f"{d.get('n')} (čekám schema 1, n {N_TBL})")
                return
            cd = float(d["vehicle"]["cd_body"])
            self.fin_dcd = (float(d["fin_cd_zero"]) - cd, float(d["fin_cd_brake"]) - cd)
            self.computed = {"vz0": d["vz0"], "dvz": d["dvz"],
                             "hmin": fill_no_band(d["hmin"], d["hmax"]), "hmax": d["hmax"]}
            if "gains" in d:
                self.gains = {GAIN_KEYS[k]: float(v) for k, v in d["gains"].items() if k in GAIN_KEYS}
            git = str(d.get("landsim_git", "?"))[:10] + (" (upravený)" if d.get("landsim_dirty") else "")
            self.sim_log.appendPlainText(f"# LandSim {git}; ΔCd křidélek neutrál {self.fin_dcd[0]:.3f}, "
                                         f"aerobrzda {self.fin_dcd[1]:.3f}; rezerva zpoždění "
                                         f"{d['delay_pad_s']:.2f} s")
            for k, sp in (("g.fin_dcd_zero", self.fin_dcd[0]), ("g.fin_dcd_brake", self.fin_dcd[1]),
                          ("g.ign_delay_pad_s", float(d["delay_pad_s"]))):
                self.fields[k].blockSignals(True)
                self.fields[k].setValue(sp)
                self.fields[k].blockSignals(False)
            if self.gains:
                self.sim_log.appendPlainText("# zisky: " + ", ".join(f"{k} {v:.3f}" for k, v in self.gains.items()))
            if "motor_curve" in (self.last_vehicle or {}):
                self.sim_log.appendPlainText(
                    "# POZOR: vlastní tahová křivka platí pro pásmo, ale tabulka motoru v raketě je "
                    "zakompilovaná - přegenerujte guidance_tables.h (make tables VEHICLE=...) a nahrajte "
                    "firmware Navigation MCU.")
        else:
            self.computed = {"vz0": d["tbl.vz0"], "dvz": d["tbl.dvz"], "hmin": d["h_min"], "hmax": d["h_max"]}
        n_no = sum(1 for v in self.computed["hmax"] if v <= 0)
        self.sim_log.appendPlainText(f"# hotovo: pásmo pro {N_TBL - n_no} z {N_TBL} rychlostí pádu"
                                     + (f", nad {self.computed['vz0'] + self.computed['dvz'] * (N_TBL - n_no):.0f} m/s "
                                        "přistání nejde uzavřít" if n_no else ""))
        self.send_btn.setEnabled(True)
        self.preview()

    def send(self) -> None:
        if not self.computed or not self.win.link.is_open:
            return
        cmds = [f"PARAM SET {k} {v:.6g}" for k, v in self.vehicle().items()]
        cmds += [f"PARAM SET tbl.vz0 {self.computed['vz0']:.6g}", f"PARAM SET tbl.dvz {self.computed['dvz']:.6g}"]
        cmds += [f"PARAM SET tbl.hmin_{i:02d} {v:.6g}" for i, v in enumerate(self.computed["hmin"])]
        cmds += [f"PARAM SET tbl.hmax_{i:02d} {v:.6g}" for i, v in enumerate(self.computed["hmax"])]
        if getattr(self, "gains", None):
            cmds += [f"PARAM SET {k} {v:.6g}" for k, v in self.gains.items()]
        # paced: the Master answers each one, and its console reads 64 bytes a tick
        self._queue = cmds
        self._tick = QtCore.QTimer(self, interval=30, timeout=self._send_next)
        self._tick.start()
        self.win.log(f"> odesílám {len(cmds)} parametrů (vozidlo + tabulka zážehu)")

    def _send_next(self) -> None:
        if not self._queue or not self.win.link.is_open:
            self._tick.stop()
            QtCore.QTimer.singleShot(400, lambda: self.win._cmd("PARAM LIST"))
            return
        self.win.link.send(self._queue.pop(0))

    # -------------------------------------------------------------- preview
    def preview(self) -> None:
        veh = self.vehicle()
        if veh["g.mass_gross_kg"] <= 0 or veh["g.diameter_m"] <= 0:
            return
        tabs = []
        bt = self.board_table()
        if bt:
            tabs.append(("v raketě (tbl.*)", bt, "#4aa3ff"))
        if self.computed:
            tabs.append(("nově spočítané", self.computed, "#35d07f"))
        self.plot.tables = tabs
        use = self.computed or bt
        if not use:
            self.plot.update()
            return
        h_cmd, lo, hi, vz_at, t_to = ign_compute(use, veh, self.apo.value(), 0.0, self.drift.value(),
                                                 self.dscale.value())
        # the fall itself, for the plot
        fall = []
        cd_free = cd_free_of(veh)
        kd = 0.5 * 1.225 * math.pi * (veh["g.diameter_m"] / 2) ** 2 * cd_free * self.dscale.value() / veh["g.mass_gross_kg"]
        h, vz = self.apo.value(), 0.0
        while h > 0:
            fall.append((abs(vz), h))
            vz += (-9.80665 + kd * vz * vz) * 0.02
            h += vz * 0.02
        self.plot.fall = fall
        self.plot.cmd = (abs(vz_at), h_cmd)
        self.plot.update()
        self.result.setText(f"Zážeh v {h_cmd:.1f} m  ·  pád {abs(vz_at):.1f} m/s  ·  "
                            f"{t_to:.1f} s po apogeu  ·  pásmo na této dráze {lo:.1f}–"
                            + (f"{hi:.1f} m" if hi > 0 else "ŽÁDNÉ (nejlepší pokus)"))
        rows = []
        for apo in range(80, 241, 10):
            hc, l_, h_, v_, t_ = ign_compute(use, veh, float(apo), 0.0, self.drift.value(), self.dscale.value())
            rows.append((apo, hc, l_, h_, abs(v_), t_))
        self.ptab.setRowCount(len(rows))
        for r_, row in enumerate(rows):
            vals = [f"{row[0]} m", f"{row[1]:.1f} m", f"{row[2]:.1f} m",
                    f"{row[3]:.1f} m" if row[3] > 0 else "žádné", f"{row[4]:.1f} m/s", f"{row[5]:.1f} s"]
            for c_, v in enumerate(vals):
                it = QtWidgets.QTableWidgetItem(v)
                if row[3] <= 0:
                    it.setForeground(QtGui.QColor(ag.C["danger"]))
                self.ptab.setItem(r_, c_, it)
