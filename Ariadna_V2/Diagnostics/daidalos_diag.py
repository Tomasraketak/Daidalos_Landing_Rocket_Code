#!/usr/bin/env python3
"""
Daidalos — diagnostic and test console (Ariadna V2, flight firmware)

Plug the MASTER MCU into the laptop with a USB cable and this window shows
the state of the whole vehicle: all three flight computers, their firmware
versions, every error the software can detect, attitude (3-D model, tilt
bull's-eye, numbers), position relative to the pad, live graphs, and the
complete parameter table — which can be edited and saved to the Master's
flash (W25Q128) from here, including the timing of every flight event, and
the ignition band recomputed in LandSim_Light for the vehicle you enter.

It is built ON TOP OF the Actuator Console (Examples/Actuator_Console/gui):
that window is the "Aktuátory a pyro" tab, unchanged, with every function it
had. With the flight firmware those controls work only in TEST mode, which
this window switches on and off (button "TEST režim").

THE USB LINK NEVER COSTS THE FLIGHT ANYTHING. The Master sends nothing
unless a host has the port open, queues text in RAM, and hands the USB
stack only as much as it will take without waiting. In flight there is no
cable and the whole diagnostic path is two comparisons per control tick.

    pip install -r requirements.txt
    python daidalos_diag.py                 # pick the port in the window
    python daidalos_diag.py --port COM5     # connect straight away
    python daidalos_diag.py --demo          # no hardware: a simulated vehicle

The protocol it speaks is described in Master_MCU.ino, section 13, and in
USER_MANUAL.md, chapter "Diagnostická konzole".
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import random
import re
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ACT_GUI = os.path.join(HERE, "..", "Examples", "Actuator_Console", "gui")
PARAMS_H = os.path.join(HERE, "..", "common", "ariadna_params.h")
sys.path.insert(0, ACT_GUI)
sys.path.insert(0, HERE)

from PyQt6 import QtCore, QtGui, QtWidgets          # noqa: E402
from PyQt6.QtCore import Qt                          # noqa: E402

import actuator_gui as ag                            # noqa: E402
from actuator_gui import button, chip, label, set_chip, Card   # noqa: E402
from daidalos_widgets import (RocketView, TiltBubble, StripChart, PadMap,   # noqa: E402
                              attitude_summary, pad_quaternion)
from diag_log import LogTab                            # noqa: E402
from diag_ignition import IgnitionTab                # noqa: E402
from diag_calib import CalibTab, parse_srvchk          # noqa: E402

APP_NAME = "Daidalos · Diagnostika"

STATE_NAMES = {0: "BOOT", 1: "IDLE", 2: "ARMED", 3: "ASCENT", 4: "APOGEE", 5: "DESCENT",
               6: "GUIDANCE", 7: "LANDED", 8: "RECOVERY", 9: "ABORT", 10: "TEST", 11: "ERROR"}
STATE_CZ = {0: "start", 1: "na rampě, bezpečno", 2: "ZASTRČENO – připraveno ke startu",
            3: "stoupání", 4: "apogeum", 5: "pád", 6: "přistávací zážeh", 7: "přistáno",
            8: "vyhledávání", 9: "PŘERUŠENO", 10: "TEST režim", 11: "CHYBA"}
PHASE_NAMES = {0: "IDLE", 1: "ASCENT", 2: "DESCENT", 3: "IGNITING", 4: "BURN", 5: "DONE"}
EVENT_BITS = [(0, "Apogeum"), (1, "Aerokryt"), (2, "Zážeh 1"), (3, "Tah cítit"),
              (4, "Zážeh 2"), (5, "Nohy"), (6, "Lidar"), (7, "Dosed")]
NAV_FLAGS = [(0, "VALID"), (1, "BARO"), (2, "IMU"), (3, "MAG"), (4, "GPS fix"), (5, "PAD ref"),
             (6, "UWB rádio"), (7, "LIDAR"), (8, "ACC sat"), (9, "DEADRECK"), (10, "Origin"),
             (11, "GPS aided"), (12, "ZUPT"), (13, "UWB aided")]
SENS_BITS = [(0, "IMU"), (1, "BARO"), (2, "MAG"), (3, "GPS"), (4, "UWB"), (5, "LIDAR"), (6, "INS"),
             (7, "LIDAR povolen")]
INS_MODES = {0: "žádný", 1: "jen orientace", 2: "bez GPS (coasting)", 3: "plný (GPS)", 255: "nezkompilováno"}
UWB_MODES = {0: "—", 1: "1 kotva (LOS)", 2: "2-D", 3: "3-D"}
HEALTH_ERR = {1: "POST selhal", 2: "spoj s Position MCU", 3: "spoj s Navigation MCU",
              4: "flash log", 5: "sběrnice serv", 6: "nízká baterie", 7: "parametry nesouhlasí"}

# What every parameter does, for the editor's tooltip. Keys from ariadna_params.h.
PARAM_HELP = {
    "seq.cover_lead_s": "Kolik sekund PŘED příkazem k zážehu se odjistí aerokryt (servo, kanál 7).",
    "seq.backup_win_s": "Okno po příkazu k zážehu: když v něm palubní počítač necítí skok zrychlení, "
                        "odpálí záložní roznět na PYRO2.",
    "seq.legs_lead_s": "Kolik sekund PŘED plánovaným dosednutím se pálí gumička nohou (PYRO3).",
    "seq.legs_pulse_s": "Jak dlouho teče proud do přepalovacího drátku nohou (nejméně 0,5 s).",
    "seq.ign_pulse_s": "Délka proudového pulzu do roznětu (PYRO1 i PYRO2).",
    "seq.cover_enable": "1 = aerokryt se odjišťuje automaticky.",
    "seq.backup_enable": "1 = záložní roznět smí odpálit.",
    "seq.legs_enable": "1 = nohy se uvolňují automaticky.",
    "seq.legs_h_max_m": "Nad touto výškou se nohy nikdy neuvolní, ať odpočet říká cokoli.",
    "seq.apogee_min_m": "Apogeum se uzná, až když raketa byla aspoň takhle vysoko.",
    "g.mass_gross_kg": "Vzletová hmotnost přistávací části VČETNĚ paliva přistávacího motoru.",
    "g.mass_prop_kg": "Hmotnost paliva přistávacího motoru.",
    "g.wn_tvc": "Šířka pásma náklonové smyčky na trysce (rad/s). Fitováno v LandSim_Light.",
    "g.zeta_tvc": "Tlumení náklonové smyčky na trysce.",
    "g.sched_tvc": "Exponent plánování zisku podle skutečného tahu.",
    "g.att_ki": "Integrátor náklonu (jen tryska). Vyrovnává vychýlenou osu tahu.",
    "g.att_i_lim": "Omezení integrátoru.",
    "g.wn_fin": "Šířka pásma smyčky na křidélkách (volný pád).",
    "g.zeta_fin": "Tlumení smyčky na křidélkách.",
    "g.sched_fin": "Exponent plánování zisku křidélek podle dynamického tlaku.",
    "g.roll_gain": "Zisk tlumiče rotace kolem podélné osy (jen křidélka).",
    "g.roll_max_deg": "Kolik výchylky smí roll kanál spotřebovat.",
    "g.tilt_min_deg": "Kužel povoleného náklonu u země.",
    "g.tilt_slope": "Jak rychle se kužel rozšiřuje s výškou (°/m).",
    "g.tilt_cap_deg": "Největší povolený náklon.",
    "g.alt_pad_m": "O kolik metrů se navedení počítá NÍŽ, dokud lidar nevidí zem.",
    "g.h_ign_bias_m": "Posun výšky zážehu z tabulky (+ = výš).",
    "g.h_ign_override_m": "Je-li > 0, zážeh v této výšce místo tabulky. Jen pro zkoušky!",
    "g.plan_target_mps": "Rychlost dosednutí, na kterou plánovač míří.",
    "g.plan_margin": "Podíl odhadnutého tahu, se kterým plánovač nepočítá.",
    "g.plan_hold_s": "Jak dlouho plán drží úroveň škrcení, pak předpokládá plný tah.",
    "g.onset_bump": "O kolik m/s² musí vzrůst osové zrychlení, aby to byl zážeh.",
    "g.onset_timeout_s": "Když zážeh do té doby necítí, předpokládá ho.",
    "g.onset_assumed_s": "Předpokládané zpoždění zážehu v tom případě.",
    "fin.enable": "1 = křidélka jsou řízená.",
    "fin.max_deg": "Mez výchylky křidélka.",
    "fin.brake": "1 = ve volném pádu se křidélka roztáhnou jako aerodynamická brzda.",
    "fin.min_airspeed": "Pod touto rychlostí se křidélka neřídí (nemají účinek).",
    "fin.drift_gain": "Náklon proti driftu ve volném pádu (°/(m/s)). 0 = držet svisle.",
    "fin.drift_max_deg": "Maximální náklon proti driftu.",
    "fin.ctrl_lead_m": "Výška nad zážehem, kde se přechází z držení svislice na navigační zákon.",
    "fin.ascent_mode": "Křidélka při stoupání: 0 neutrál, 1 jen tlumit rotaci, 2 tlumit + držet svislici.",
    "fin.adapt": "1 = za letu se odhaduje skutečná účinnost a offset křidélek.",
    "fin.gain0": "Počáteční účinnost křidélek (1 = podle CFD).",
    "srv.tvc_x_trim_us": "Pulz, při kterém je tryska v ose X rovně.",
    "srv.tvc_y_trim_us": "Pulz, při kterém je tryska v ose Y rovně.",
    "srv.tvc_us_per_deg": "µs pulzu na stupeň serva TVC.",
    "srv.tvc_x_sign": "Znaménko serva X. ŠPATNÉ ZNAMÉNKO = KLADNÁ ZPĚTNÁ VAZBA.",
    "srv.tvc_y_sign": "Znaménko serva Y. ŠPATNÉ ZNAMÉNKO = KLADNÁ ZPĚTNÁ VAZBA.",
    "srv.clamp_closed_us": "Pulz serva škrticí klapky: plně zavřeno (nejméně tahu).",
    "srv.clamp_open_us": "Pulz serva škrticí klapky: plně otevřeno.",
    "srv.fin_us_per_deg": "µs pulzu na stupeň křidélka.",
    "srv.fin1_trim_us": "Pulz, při kterém stojí křidélko 1 rovně (neutrál).",
    "srv.fin2_trim_us": "Pulz, při kterém stojí křidélko 2 rovně (neutrál).",
    "srv.fin3_trim_us": "Pulz, při kterém stojí křidélko 3 rovně (neutrál).",
    "srv.fin4_trim_us": "Pulz, při kterém stojí křidélko 4 rovně (neutrál).",
    "srv.fin1_sign": "Znaménko křidélka 1: kladná výchylka musí tlačit křidélko ve směru b × r (manuál).",
    "srv.fin2_sign": "Znaménko křidélka 2: kladná výchylka musí tlačit křidélko ve směru b × r (manuál).",
    "srv.fin3_sign": "Znaménko křidélka 3: kladná výchylka musí tlačit křidélko ve směru b × r (manuál).",
    "srv.fin4_sign": "Znaménko křidélka 4: kladná výchylka musí tlačit křidélko ve směru b × r (manuál).",
    "srv.cover_lock_us": "Pulz serva aerokrytu: ZAJIŠTĚNO.",
    "srv.cover_unlock_us": "Pulz serva aerokrytu: ODJIŠTĚNO.",
    "mst.launch_g": "Start se pozná i bez breakwiru, když zrychlení přesáhne tuto hodnotu.",
    "mst.auto_arm": "1 = zastrčí se samo po vytažení RBF; 0 = navíc potřeba tlačítko „Zastrčit“.",
    "pos.gps_latency_ms": "O kolik ms jsou data z GPS opožděná. Filtr je porovnává s minulým stavem.",
    "pos.lidar_enable": "1 = TF-Luna lidar se používá.",
    "pos.lidar_offset_m": "Výška lidaru nad zemí, když raketa stojí na nohou.",
    "pos.lidar_max_m": "Lidar se do výšky započítává jen pod touto vzdáleností.",
    "pos.att_gyro_only": "1 = za letu orientace jen z gyra (bez akcelerometru a magnetometru).",
    "pos.uwb_enable": "1 = UWB kotvy se používají.",
    "pos.lidar_delay_ms": "Lidar je na vyklápěcí noze: do fúze se započítá až tolik ms po aktivaci nohou (PYRO3).",
    "pos.lidar_tilt_x_deg": "Statická odchylka montáže lidaru: náklon paprsku kolem tělesové osy X.",
    "imu.acc_off_x": "Offset akcelerometru (osa X desky), g. Z kalibrace v záložce Kalibrace.",
    "imu.acc_off_y": "Offset akcelerometru (osa Y desky), g.",
    "imu.acc_off_z": "Offset akcelerometru (osa Z desky), g.",
    "imu.acc_scale_x": "Škála akcelerometru (osa X): a = (raw − off) · scale.",
    "imu.acc_scale_y": "Škála akcelerometru (osa Y).",
    "imu.acc_scale_z": "Škála akcelerometru (osa Z).",
    "imu.mag_hi_x": "Hard iron magnetometru X, µT. Z kalibrace v záložce Kalibrace.",
    "imu.mag_hi_y": "Hard iron magnetometru Y, µT.",
    "imu.mag_hi_z": "Hard iron magnetometru Z, µT.",
    "pos.pad_lat_deg": "Zeměpisná šířka rampy (počátek navigace). 0 a 0 = nenastaveno: počátek = poloha při zastrčení. Nastavuje „Rampa = tady“.",
    "pos.pad_lon_deg": "Zeměpisná délka rampy. Viz pos.pad_lat_deg.",
    "mst.servo_check_on_rbf": "1 = po vytažení RBF Master zkontroluje připojení všech serv (pohyb 5° a proud), zastrčení počká; 0 = jen tlačítkem.",
    "mst.servo_check_ma": "Minimální nárůst proudu, při kterém se servo považuje za připojené, mA (1 krok ADC ≈ 16 mA).",
    "pos.lidar_tilt_y_deg": "Statická odchylka montáže lidaru: náklon paprsku kolem tělesové osy Z (osa rakety je Y).",
    "g.cd_body": "Cd samotného trupu. Křidélka se přičítají z CFD. Za letu se odpor navíc měří.",
    "g.thrust_scale": "Násobek katalogové tahové křivky přistávacího motoru (statický zážeh).",
    "g.ign_delay_pad_s": "Na jak dlouhé zpoždění roznětu se výška zážehu jistí (nejhorší případ + ~0,1 s).",
    "g.drag_adapt": "1 = odpor se ve volném pádu měří akcelerometrem a výpočet zážehu ho používá.",
    "g.plan_ts_tau_s": "Plánovač klapky čeká, že odchylka tahu od tabulky odezní s touto časovou konstantou (LandSim: 0,7 s). 0 = odchylka trvá do vyhoření.",
    "g.fin_dcd_zero": "Odpor křidélek v neutrálu jako přírůstek k Cd trupu (LandSim band.json fin_cd_zero − cd_body). Platí při hoření.",
    "g.fin_dcd_brake": "Odpor křidélek roztažených jako aerobrzda (+,−,+,−) při pádu, přírůstek k Cd trupu (fin_cd_brake − cd_body). Raketa ho škáluje podle skutečného roztažení brzdy.",
    "g.diameter_m": "Průměr trupu (referenční plocha odporu).",
    "tbl.vz0": "Tabulka zážehu: první rychlost pádu. Počítá se v LandSim (záložka Zážeh a LandSim).",
    "tbl.dvz": "Tabulka zážehu: krok rychlosti pádu.",
}


def load_param_table(path: str = PARAMS_H) -> list[dict]:
    """The parameter table straight from ariadna_params.h (the same file the
    firmware compiles), so the editor works offline and the demo is honest."""
    out = []
    try:
        src = open(path, encoding="utf-8").read()
    except OSError:
        return out
    # the ignition band table is generated into its own header
    tbl = os.path.join(os.path.dirname(path), "ariadna_params_tbl.h")
    if os.path.exists(tbl):
        src += open(tbl, encoding="utf-8").read()
    rx = re.compile(r'X\(\s*(\d+),\s*(\w+),\s*"([^"]+)",\s*([-\d.eE]+)f?,\s*([-\d.eE]+)f?,'
                    r'\s*([-\d.eE]+)f?,\s*"([^"]*)",\s*\'(\w)\'\)')
    for m in rx.finditer(src):
        out.append({"id": int(m.group(1)), "name": m.group(2), "key": m.group(3),
                    "def": float(m.group(4)), "lo": float(m.group(5)), "hi": float(m.group(6)),
                    "unit": m.group(7), "owner": m.group(8)})
    for p in out:
        p["val"] = p["def"]
    return out


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


# ---------------------------------------------------------------------------
#  Simulated vehicle, for --demo
# ---------------------------------------------------------------------------
class FlightSimBoard(ag.SimBoard):
    """The Actuator Console's simulated board, extended with the flight
    firmware's diagnostic protocol: TEST mode, the parameter table, and the
    M / N / P / G status lines of a vehicle standing on the pad."""

    def __init__(self):
        super().__init__()
        self.test = False
        self.state = 1
        self.params = load_param_table()
        self.next_diag = 0.0
        self.which = 0
        self.list_queue: list[str] = []
        self.raw = False
        self.next_raw = 0.0
        self.srv_t0 = None
        self.srv_last = -1

    def crc(self) -> int:
        import struct
        v = [0.0] * 200
        for p in self.params:
            v[p["id"]] = p["val"]
        return crc16(struct.pack("<200f", *v))

    def _cmd(self, line: str) -> None:
        if not line:
            return
        p = line.split()
        c = p[0].upper()
        if c == "HELLO":
            super()._cmd(line)
            self.q.put("I DAIDALOS 325 FLIGHT guidance 220 params %d" % len(self.params))
            return
        if c == "TEST":
            self.test = len(p) > 1 and p[1] == "1"
            self.state = 10 if self.test else 1
            if not self.test:
                self.armed_until, self.fire_ch = 0, -1
            self.q.put(f"OK TEST {1 if self.test else 0}")
            return
        if c == "PARAM":
            sub = p[1].upper() if len(p) > 1 else ""
            if sub == "LIST":
                for q in self.params:
                    self.list_queue.append(f"PRM {q['id']} {q['key']} {q['val']:.6g} {q['def']:.6g} "
                                           f"{q['lo']:.6g} {q['hi']:.6g} {q['unit']} {q['owner']}")
                self.list_queue.append(f"OK PARAM LIST {len(self.params)} {self.crc()}")
            elif sub == "SET" and len(p) >= 4:
                for q in self.params:
                    if q["key"] == p[2] or str(q["id"]) == p[2]:
                        q["val"] = max(q["lo"], min(q["hi"], float(p[3].replace(",", "."))))
                        self.q.put(f"OK PARAM {q['id']} {q['key']} {q['val']:.6g}")
                        break
                else:
                    self.q.put(f"ERR PARAM unknown '{p[2]}'")
            elif sub == "SAVE":
                self.q.put(f"OK PARAM SAVE {self.crc()}")
            elif sub == "DEFAULTS":
                for q in self.params:
                    q["val"] = q["def"]
                self.q.put(f"OK PARAM DEFAULTS {self.crc()} (not saved)")
            elif sub == "LOAD":
                self.q.put(f"OK PARAM LOAD eeprom {self.crc()}")
            return
        if c == "LOG":
            self._log_cmd(p)
            return
        if c == "SERVOCHECK":
            self.srv_t0, self.srv_last = time.monotonic(), -1
            self.q.put("OK SERVOCHECK")
            return
        if c == "RAW":
            self.raw = len(p) > 1 and p[1] == "1"
            self.q.put(f"OK RAW {1 if self.raw else 0}")
            return
        if c in ("PADREF", "CAL", "FLIGHTARM"):
            self.q.put(f"OK {line.upper()}")
            return
        if c in ("SET", "OFF", "IDLE", "LIM", "PWM", "ARM", "FIRE") and not self.test:
            if c != "SET":
                self.q.put(f"ERR {c} needs TEST mode (send TEST 1)")
            return
        super()._cmd(line)

    # ---- a synthetic flight log: pad at 10 Hz, one flight at 200 Hz, recovery
    def _sim_log(self) -> list[bytes]:
        if getattr(self, "_log", None) is not None:
            return self._log
        import struct
        from diag_log import dlog
        recs, t = [], 0
        def rec(t_ms, st, ph, h, vz, fl, thr=0.0, k=1.0, drag=0, ts=0, fg=100, tvc=0.0):
            return struct.pack(dlog.FMT, t_ms, 0.0, 0.0, h, 0.1, -0.05, vz, 0.0, 0.0,
                               1.0 if st >= 5 else -1.0, 0.0, 0.0, 0.0, 500000000, 140000000,
                               tvc, -tvc / 2, k, thr, k, 7900, 900, 0x1F, st, ph, 0x07, 7, 0, 0,
                               fg, drag, ts, fl)
        for i in range(600):                       # 60 s on the pad, 10 Hz
            recs.append(rec(t, 2 if i > 300 else 1, 0, 0.0, 0.0, 0)); t += 100
        h, vz = 0.0, 0.0
        for i in range(200 * 26):                  # 26 s of flight, 200 Hz
            tt = i / 200.0
            if tt < 1.2:   a, st, ph = 60.0, 3, 1
            elif vz > 0:   a, st, ph = -9.81 - 0.004 * vz * vz, 3, 1
            elif h > 55:   a, st, ph = -9.81 + 0.0085 * vz * vz, 5, 2
            elif h > 0.05: a, st, ph = 18.0 + 0.002 * vz * vz, 6, 4
            else:          a, st, ph = 0.0, 7, 5
            vz += a * 0.005; h = max(h + vz * 0.005, 0.0)
            if h <= 0.0 and ph >= 4: vz = 0.0
            burn = ph == 4
            recs.append(rec(t, st, ph, h, vz, 1, thr=95.0 if burn else 0.0, k=0.7 if burn else 1.0,
                            drag=103 if ph == 2 else 0, ts=97 if burn else 0, fg=112,
                            tvc=1.5 * math.sin(tt * 9) if burn else 0.0))
            t += 5
        for i in range(300):                       # recovery, 10 Hz
            recs.append(rec(t, 8, 5, 0.0, 0.0, 0)); t += 100
        self._log = recs
        return recs

    def _log_cmd(self, p) -> None:
        sub = p[1].upper() if len(p) > 1 else "INFO"
        log = self._sim_log()
        if sub == "INFO":
            self.q.put(f"OK LOG INFO {len(log) * 96} {len(log)} 96")
        elif sub == "FLIGHTS":
            self.q.put(f"OK LOG SCAN {len(log)}")
            import struct
            from diag_log import dlog
            fl = [i for i, r in enumerate(log) if r[-1] & 1]
            if fl:
                t0 = struct.unpack_from("<I", log[fl[0]])[0]; t1 = struct.unpack_from("<I", log[fl[-1]])[0]
                hm = max(struct.unpack(dlog.FMT, log[i])[3] for i in fl)
                self.list_queue.append(f"LOGF 0 {fl[0]} {len(fl)} {t0} {t1} {hm:.1f}")
            self.list_queue.append(f"OK LOG FLIGHTS {1 if fl else 0}")
        elif sub == "DUMP":
            a = int(p[2]) if len(p) > 2 else 0
            n = int(p[3]) if len(p) > 3 else len(log)
            part = log[a:a + n]
            self.q.put(f"OK LOG BEGIN {len(part)} 96")
            self.list_queue += [f"LOGR {a + i} {r.hex()}" for i, r in enumerate(part)]
            self.list_queue.append(f"OK LOG DUMP {a + len(part)}")
        elif sub == "STOP":
            self.list_queue = [l for l in self.list_queue if not l.startswith("LOGR")]
            self.q.put("OK LOG STOP")
        elif sub == "ERASE":
            self._log = []
            self.q.put("OK LOG ERASING 0")
            self.list_queue.append("OK LOG ERASE")

    def readline(self, timeout: float = 0.05):
        now = time.monotonic()
        if self.list_queue:
            return self.list_queue.pop(0)
        if self.srv_t0 is not None:
            k = int((now - self.srv_t0) / 1.1)          # one servo per 1.1 s, as on the Master
            if k != self.srv_last:
                self.srv_last = k
                res = [1 if i < k else 0 for i in range(8)]
                if k >= 8:
                    res[6] = 2                           # demo: fin 4 "unplugged"
                ma = [180 + 17 * i if r == 1 else (8 if r == 2 else 0) for i, r in enumerate(res)]
                d = {"run": 0 if k >= 8 else 1, "done": 1 if k >= 8 else 0, "ch": min(k, 7),
                     "res": res, "ma": ma, "thr": 60}
                if k >= 8:
                    self.srv_t0 = None
                return "SRVCHK " + json.dumps(d, separators=(",", ":"))
        if self.raw and now >= self.next_raw:
            self.next_raw = now + 0.04
            return self._raw(now - self.t0)
        if now >= self.next_diag:
            self.next_diag = now + 0.02
            self.which = (self.which + 1) % 5
            if self.which:
                return self._diag(self.which, now - self.t0)
        return super().readline(timeout)

    def _sim_tilt(self, t: float):
        # standing on the pad, nose up, swaying a little
        return math.radians(2.0 + 1.5 * math.sin(t * 0.7)), t * 0.3

    def _raw(self, t: float) -> str:
        """An R line: the TriSense at rest with board -Y up (fw 3.24 mount)."""
        tilt, az = self._sim_tilt(t)
        n = lambda s_: round(random.gauss(0.0, s_), 4)      # noqa: E731
        # specific force points UP (+1 g along the nose, board -Y), plus the lean
        ax, az_ = math.sin(tilt) * math.cos(az), math.sin(tilt) * math.sin(az)
        acc = [round(ax + n(0.003), 4), round(-math.cos(tilt) + n(0.003), 4), round(az_ + n(0.003), 4)]
        d = {"t": int(t * 1000), "acc": acc,
             "accr": [round(acc[0] / 0.995 + 0.012, 5), round(acc[1] / 1.004 - 0.008, 5),
                      round(acc[2] / 0.998 + 0.020, 5)],
             "gyr": [n(0.05), n(0.05), n(0.05)], "mag": [round(21.0 + n(0.3), 2), round(-38.5 + n(0.3), 2),
                                                        round(12.2 + n(0.3), 2)],
             "magc": [round(18.0 + n(0.3), 2), round(-41.0 + n(0.3), 2), round(9.5 + n(0.3), 2)],
             "p": round(97234.0 + n(2.0), 1), "temp": 24.6, "src": 1, "mf": 1, "hz": 200}
        return "R " + json.dumps(d, separators=(",", ":"))

    def _diag(self, which: int, t: float) -> str:
        crc = self.crc()
        tilt, az = self._sim_tilt(t)
        qw, qx, qy, qz = pad_quaternion(tilt, az)
        # thrust axis u = q applied to board +Y: down on the pad
        R = [[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
             [2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx)],
             [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy)]]
        u = [round(R[i][1], 3) for i in range(3)]
        if which == 1:
            d = {"t": int(t * 1000), "fw": 325, "gv": 220, "st": self.state, "fl": 0x12 | (8 if self.test else 0),
                 "tl": 0, "vbat": round(7.9 - 0.002 * (t % 100), 2), "i": 0.42, "rbf": 1, "bw": 0, "post": 1,
                 "servo": 1, "flash": 1, "lora": 1, "log": 18432, "logdrop": 0, "tlm": [int(t * 10), 0],
                 "pos": [int(t * 260), 0, 1], "nav": [int(t * 220), 0, 1], "pcrc": crc, "psync": 1,
                 "peep": 1, "cover": 0, "hl": 0xEF, "usbdrop": 0}
            return "M " + json.dumps(d, separators=(",", ":"))
        if which == 2:
            d = {"p": [round(0.05 * math.sin(t * 0.2), 2), round(0.04 * math.cos(t * 0.17), 2),
                       round(0.03 * math.sin(t), 2)],
                 "v": [0.0, 0.0, 0.0], "a": [0.0, 0.0, 0.0],
                 "q": [round(qw, 4), round(qx, 4), round(qy, 4), round(qz, 4)],
                 "u": u, "w": [round(0.01 * math.sin(t * 3), 3), 0.0, 0.0],
                 "g": 1.0, "fl": 0x3CF7, "ql": 230, "ok": 1}
            return "N " + json.dumps(d, separators=(",", ":"))
        if which == 3:
            d = {"ok": 1, "fw": 325, "pcrc": crc, "pseen": len(self.params),
                 "rpy": [0.0, round(90 - math.degrees(tilt), 1), round(math.degrees(az) % 360, 1)],
                 "lat": 497985411, "lon": 166912772, "galt": 412.3, "sats": 12, "fix": 3, "hdop": 9,
                 "temp": 21.4, "sig": [0.21, 0.05], "uwb": [4, 15, 3, 0], "uwbs": [0.14, 0.08],
                 "trim": 0.21, "lidar": [round(1.2 + 0.01 * math.sin(t), 2), 40], "ins": 3, "att": 1,
                 "hz": 200, "sens": 0x7F, "ovf": 0, "cal": 1, "rx": [int(t * 10), 0], "pp": 97230}
            return "P " + json.dumps(d, separators=(",", ":"))
        d = {"ok": 1, "fw": 325, "pcrc": crc, "pseen": len(self.params), "ph": 0, "k": 1.0, "thr": 0.0,
             "hign": 0.0, "tc": 0.0, "te": 0.0, "roll": 0.0, "tign": -1.0, "ttd": -1.0, "ts": 1.0,
             "fg": 1.0, "apo": 0.0, "res": 0.0, "rx": [int(t * 240), 0], "plan": int(t * 200),
             "loop": 64, "ovr": 0, "cont": 0x07, "fired": 0, "act": 0, "nfl": 0x2F, "ev": 0,
             "tvc": [0.0, 0.0], "cl": 1.0, "fin": [0.0, 0.0, 0.0, 0.0], "afl": 1,
             "drag": 1.0, "blo": 0.0, "bhi": 0.0}
        return "G " + json.dumps(d, separators=(",", ":"))


class DiagLink(ag.Link):
    """The Actuator Console's link, with the flight-firmware simulation."""

    def open(self, port: str) -> None:
        if port == "DEMO":
            self.close()
            self._dev = FlightSimBoard()
            import threading
            self._run = True
            self._thread = threading.Thread(target=self._reader, daemon=True)
            self._thread.start()
            self.state.emit(True, "DEMO")
            return
        super().open(port)

    def _reader(self) -> None:
        dev = self._dev
        if isinstance(dev, FlightSimBoard):
            while self._run and self._dev is dev:
                ln = dev.readline()
                if ln is not None:
                    self.line.emit(ln)
            return
        super()._reader()


# ---------------------------------------------------------------------------
#  small view helpers
# ---------------------------------------------------------------------------
class KV(QtWidgets.QWidget):
    """A two-column grid of name : value labels."""

    def __init__(self, names: list[str]):
        super().__init__()
        g = QtWidgets.QGridLayout(self)
        g.setContentsMargins(0, 0, 0, 0)
        g.setHorizontalSpacing(14)
        g.setVerticalSpacing(3)
        self.v: dict[str, QtWidgets.QLabel] = {}
        for i, n in enumerate(names):
            g.addWidget(label(n, "muted"), i, 0)
            lab = label("—", "value")
            lab.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
            g.addWidget(lab, i, 1)
            self.v[n] = lab
        g.setColumnStretch(1, 1)

    def set(self, name: str, text: str, color: str | None = None) -> None:
        lab = self.v[name]
        if lab.text() != text:
            lab.setText(text)
        style = f"color: {color};" if color else ""
        if lab.styleSheet() != style:
            lab.setStyleSheet(style)


class Flags(QtWidgets.QWidget):
    """A row of chips, one per bit."""

    def __init__(self, bits: list[tuple[int, str]]):
        super().__init__()
        lay = QtWidgets.QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(5)
        self.bits = bits
        self.chips = [chip(n) for _, n in bits]
        for c_ in self.chips:
            lay.addWidget(c_)
        lay.addStretch(1)

    def set(self, mask: int | None, bad_when_off: tuple[int, ...] = ()) -> None:
        for (bit, name), c_ in zip(self.bits, self.chips):
            if mask is None:
                set_chip(c_, name, "")
            elif mask & (1 << bit):
                set_chip(c_, name, "ok")
            else:
                set_chip(c_, name, "bad" if bit in bad_when_off else "")


GOOD, WARN, BAD, DIM = ag.C["positive"], ag.C["accent"], ag.C["danger"], ag.C["subtle"]


def fmt(v, nd=1, unit="", sign=False):
    if v is None:
        return "—"
    try:
        return ag.cz(float(v), nd, sign=sign) + (f" {unit}" if unit else "")
    except (TypeError, ValueError):
        return str(v)


# ---------------------------------------------------------------------------
#  the window
# ---------------------------------------------------------------------------
class DiagWindow(ag.MainWindow):
    def __init__(self, cfg_path: str, demo: bool, port: str | None):
        self.d: dict[str, dict] = {"M": {}, "N": {}, "P": {}, "G": {}, "R": {}}
        self.d_time: dict[str, float] = {"M": 0.0, "N": 0.0, "P": 0.0, "G": 0.0, "R": 0.0}
        self.flight_fw = False
        self.fw_info = ""
        self.test_on = False
        self.params = load_param_table()
        self.param_rows: dict[int, int] = {}
        self.param_edits: dict[int, float] = {}
        self.board_crc: int | None = None
        self.rec_file = None
        self.rec_writer = None
        self.t0 = time.monotonic()
        super().__init__(cfg_path, False, None)

        # The actuator console's link, replaced by one that also simulates
        # the flight firmware. Signals are re-wired to the same slots.
        self.link.close()
        self.link = DiagLink()
        self.link.line.connect(self.on_line)
        self.link.state.connect(self.on_link_state)

        self.setWindowTitle(APP_NAME)
        self.resize(1500, 980)
        root = self.centralWidget()
        outer = root.layout()
        act_scroll = outer.itemAt(1).widget()
        outer.removeWidget(act_scroll)
        outer.addWidget(self._build_diag_bar())
        self.tabs = QtWidgets.QTabWidget()
        self.tabs.setDocumentMode(True)
        self.tabs.addTab(self._scroll(self._build_overview()), "Přehled")
        self.tabs.addTab(self._scroll(self._build_nav_tab()), "Poloha a senzory")
        self.tabs.addTab(self._build_graphs(), "Grafy")
        self.tabs.addTab(self._scroll(self._build_raw_tab()), "Surová data IMU")
        self.calib_tab = CalibTab(self)
        self.tabs.addTab(self._scroll(self.calib_tab), "Kalibrace")
        self.tabs.addTab(act_scroll, "Aktuátory a pyro")
        self.tabs.addTab(self._build_params(), "Parametry a časování")
        self.ign_tab = IgnitionTab(self)
        self.tabs.addTab(self._scroll(self.ign_tab), "Zážeh a LandSim")
        self.log_tab = LogTab(self)
        self.tabs.addTab(self._scroll(self.log_tab), "Letový záznam")
        outer.addWidget(self.tabs, 1)
        self.tabs.setStyleSheet(
            f"QTabBar::tab {{ padding: 8px 18px; color: {ag.C['muted']}; border: none; }}"
            f"QTabBar::tab:selected {{ color: {ag.C['accent']}; border-bottom: 2px solid {ag.C['accent']}; }}")

        self.diag_timer = QtCore.QTimer(self, interval=200, timeout=self._refresh_diag)
        self.diag_timer.start()

        if demo:
            self.port_box.setCurrentIndex(self.port_box.findData("DEMO"))
            self.connect_toggle()
        elif port:
            idx = self.port_box.findData(port)
            if idx < 0:
                self.port_box.addItem(port, port)
                idx = self.port_box.count() - 1
            self.port_box.setCurrentIndex(idx)
            self.connect_toggle()

    @staticmethod
    def _scroll(w: QtWidgets.QWidget) -> QtWidgets.QScrollArea:
        s = QtWidgets.QScrollArea()
        s.setWidgetResizable(True)
        s.setWidget(w)
        return s

    # ------------------------------------------------------------ top bar
    def _build_diag_bar(self) -> QtWidgets.QWidget:
        bar = QtWidgets.QFrame()
        bar.setObjectName("header")
        lay = QtWidgets.QHBoxLayout(bar)
        lay.setContentsMargins(20, 8, 20, 8)
        lay.setSpacing(8)
        self.chip_state = chip("Stav: —")
        self.chip_fw = chip("Firmware: —")
        self.chip_sync = chip("Parametry: —")
        self.chip_err = chip("Chyby: —")
        for c_ in (self.chip_state, self.chip_fw, self.chip_sync, self.chip_err):
            lay.addWidget(c_)
        lay.addStretch(1)
        self.test_btn = button("TEST režim", "primary",
                               "Zapne/vypne TEST režim Masteru. Jen v něm fungují ruční ovládání "
                               "serv a pyro (záložka Aktuátory). Ze stavu ARMED vyjme.")
        self.test_btn.setCheckable(True)
        self.test_btn.clicked.connect(self.toggle_test)
        lay.addWidget(self.test_btn)
        b = button("Pad reference", tip="Teď zachytit referenční tlak a GPS počátek (raketa na rampě).")
        b.clicked.connect(lambda: self._cmd("PADREF"))
        lay.addWidget(b)
        b = button("Kalibrace gyra", tip="Kalibrace gyra a orientace. Raketou 2 s nehýbat!")
        b.clicked.connect(lambda: self._cmd("CAL GYRO"))
        lay.addWidget(b)
        self.farm_btn = button("Zastrčit", "danger",
                               "Žádost o zastrčení (ARMED) pro mst.auto_arm = 0. Podmínky: RBF venku, "
                               "navigace platná, raketa stojí, parametry synchronní.")
        self.farm_btn.clicked.connect(self._flightarm)
        lay.addWidget(self.farm_btn)
        b = button("Odstrčit", tip="Zpět do IDLE.")
        b.clicked.connect(lambda: self._cmd("FLIGHTARM 0"))
        lay.addWidget(b)
        self.rec_btn = button("Nahrávat", tip="Ukládá každou řádku z desky do CSV (čas, typ, obsah).")
        self.rec_btn.setCheckable(True)
        self.rec_btn.clicked.connect(self.toggle_record)
        lay.addWidget(self.rec_btn)
        return bar

    def _cmd(self, text: str) -> None:
        if self.link.is_open:
            self.link.send(text)
            self.log(f"> {text}")

    def _flightarm(self) -> None:
        r = QtWidgets.QMessageBox.warning(
            self, APP_NAME, "Opravdu požádat o ZASTRČENÍ?\n\nPo zastrčení může letový software "
            "sám odpálit pyrotechniku. Všichni za bezpečnostní čáru.",
            QtWidgets.QMessageBox.StandardButton.Yes | QtWidgets.QMessageBox.StandardButton.Cancel)
        if r == QtWidgets.QMessageBox.StandardButton.Yes:
            self._cmd("FLIGHTARM 1")

    def toggle_test(self) -> None:
        on = self.test_btn.isChecked()
        self._cmd(f"TEST {1 if on else 0}")

    def toggle_record(self) -> None:
        if self.rec_file:
            self.rec_file.close()
            self.rec_file = self.rec_writer = None
            self.rec_btn.setText("Nahrávat")
            return
        name = time.strftime("daidalos_diag_%Y%m%d_%H%M%S.csv")
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Nahrávat do", name, "CSV (*.csv)")
        if not path:
            self.rec_btn.setChecked(False)
            return
        self.rec_file = open(path, "w", newline="", encoding="utf-8")
        self.rec_writer = csv.writer(self.rec_file)
        self.rec_writer.writerow(["pc_time", "type", "line"])
        self.rec_btn.setText("Stop nahrávání")

    # ------------------------------------------------------------ overview
    def _build_overview(self) -> QtWidgets.QWidget:
        w = QtWidgets.QWidget()
        g = QtWidgets.QGridLayout(w)
        g.setContentsMargins(20, 16, 20, 20)
        g.setSpacing(16)

        # three boards
        self.board_kv = {}
        names = {
            "M": ("Master MCU", "Pico 2 W · stav mise, serva, log, LoRa",
                  ["Online", "Firmware", "POST", "Stav", "Baterie", "Proud", "RBF", "Breakwire",
                   "PCA9685", "Flash log", "LoRa", "Spoj Position", "Spoj Navigation", "Aerokryt",
                   "USB zahozeno"]),
            "P": ("Position MCU", "XIAO RP2350 · senzory, odhad polohy",
                  ["Online", "Firmware", "Parametry", "Senzory", "Orientace", "Fúze Hz",
                   "IMU přetečení", "Kalibrace", "Rx Master"]),
            "G": ("Navigation MCU", "XIAO RP2350 · navedení, pyro, řízení",
                  ["Online", "Firmware", "Parametry", "Fáze", "Smyčka max", "Přetečení",
                   "Plánovač", "Rx Master", "Kontinuita", "Odpáleno", "Proud teď"]),
        }
        for col, key in enumerate(("M", "P", "G")):
            title, kick, rows = names[key]
            card = Card(kick, title)
            kv = KV(rows)
            card.lay.addWidget(kv)
            card.lay.addStretch(1)
            self.board_kv[key] = kv
            g.addWidget(card, 0, col)

        # attitude
        att = Card("Orientace", "Jak je raketa nakloněná")
        row = QtWidgets.QHBoxLayout()
        self.rocket = RocketView()
        self.rocket.setMinimumSize(300, 300)
        row.addWidget(self.rocket, 3)
        col = QtWidgets.QVBoxLayout()
        self.bubble = TiltBubble()
        self.bubble.setMinimumSize(180, 180)
        col.addWidget(self.bubble, 1)
        self.att_kv = KV(["Náklon", "Směr náklonu", "Roll", "Pitch / heading", "Úhlové rychlosti",
                          "Zdroj orientace"])
        col.addWidget(self.att_kv)
        row.addLayout(col, 2)
        att.lay.addLayout(row)
        g.addWidget(att, 1, 0, 1, 2)

        # flight / guidance
        fl = Card("Let", "Navedení a sekvence")
        self.flight_kv = KV(["Fáze navedení", "Apogeum", "Výška zážehu", "Pásmo zážehu",
                             "Odpor měřený / model", "Do zážehu", "Do dosednutí",
                             "Odhad tahu", "Tah / tabulka", "Škrcení plán", "Plán rezerva",
                             "Účinnost křidélek", "Náklon příkaz / chyba"])
        fl.lay.addWidget(self.flight_kv)
        fl.lay.addWidget(label("Události", "subtle"))
        self.ev_flags = Flags(EVENT_BITS)
        fl.lay.addWidget(self.ev_flags)
        fl.lay.addStretch(1)
        g.addWidget(fl, 1, 2)

        # problems
        pr = Card("Kontrola", "Chyby a varování")
        self.problems = QtWidgets.QListWidget()
        self.problems.setMinimumHeight(170)
        self.problems.setStyleSheet("QListWidget { border: none; }")
        pr.lay.addWidget(self.problems)
        g.addWidget(pr, 2, 0, 1, 3)
        for i in range(3):
            g.setColumnStretch(i, 1)
        return w

    # ------------------------------------------------------------ nav tab
    def _build_nav_tab(self) -> QtWidgets.QWidget:
        w = QtWidgets.QWidget()
        g = QtWidgets.QGridLayout(w)
        g.setContentsMargins(20, 16, 20, 20)
        g.setSpacing(16)
        mp = Card("Poloha", "Vůči rampě (sever nahoru)")
        self.padmap = PadMap()
        self.padmap.setMinimumSize(340, 340)
        mp.lay.addWidget(self.padmap, 1)
        b = button("Vymazat stopu")
        b.clicked.connect(self.padmap.clear)
        mp.head.addWidget(b)
        g.addWidget(mp, 0, 0, 2, 1)

        st = Card("Stav", "Navigační řešení")
        self.nav_kv = KV(["Poloha E / N / výška", "Rychlost E / N / svislá", "Zrychlení (bez g)",
                          "|a| s gravitací", "Kvalita", "σ poloha / rychlost vodorovně"])
        st.lay.addWidget(self.nav_kv)
        st.lay.addWidget(label("Příznaky", "subtle"))
        self.nav_flags = Flags(NAV_FLAGS[:7])
        self.nav_flags2 = Flags(NAV_FLAGS[7:])
        st.lay.addWidget(self.nav_flags)
        st.lay.addWidget(self.nav_flags2)
        g.addWidget(st, 0, 1)

        se = Card("Senzory", "GPS · UWB · lidar · barometr")
        self.sens_kv = KV(["GPS", "Poloha (lat, lon)", "GPS výška", "UWB", "UWB přesnost / reziduál",
                           "Baro trim", "Pad tlak", "Lidar", "Teplota", "INS (INSLIB)"])
        se.lay.addWidget(self.sens_kv)
        self.sens_flags = Flags(SENS_BITS)
        se.lay.addWidget(self.sens_flags)
        g.addWidget(se, 1, 1)
        g.setColumnStretch(0, 1)
        g.setColumnStretch(1, 1)
        return w

    # ------------------------------------------------------------ graphs
    def _build_graphs(self) -> QtWidgets.QWidget:
        w = QtWidgets.QWidget()
        g = QtWidgets.QGridLayout(w)
        g.setContentsMargins(20, 16, 20, 20)
        g.setSpacing(12)
        top = QtWidgets.QHBoxLayout()
        top.addWidget(label("Okno grafů", "muted"))
        self.span_box = QtWidgets.QComboBox()
        for s in (10, 30, 60, 120, 300):
            self.span_box.addItem(f"{s} s", s)
        self.span_box.setCurrentIndex(1)
        self.span_box.currentIndexChanged.connect(self._span_changed)
        top.addWidget(self.span_box)
        b = button("Vymazat")
        b.clicked.connect(lambda: [c.clear() for c in self.charts])
        top.addWidget(b)
        self.pause_btn = button("Pozastavit")
        self.pause_btn.setCheckable(True)
        top.addWidget(self.pause_btn)
        top.addStretch(1)
        g.addLayout(top, 0, 0, 1, 2)
        self.ch_alt = StripChart("Výška nad rampou", ["pz"], "m")
        self.ch_vel = StripChart("Rychlost", ["vx", "vy", "vz"], "m/s")
        self.ch_tilt = StripChart("Náklon", ["skutečný", "příkaz"], "°")
        self.ch_rate = StripChart("Úhlová rychlost", ["wx", "wy", "wz"], "°/s")
        self.ch_act = StripChart("TVC a škrcení", ["TVC 1", "TVC 2", "klapka×10"], "°")
        self.ch_fin = StripChart("Křidélka", ["1", "2", "3", "4"], "°")
        self.ch_pwr = StripChart("Napájení", ["baterie", "proud"], "V / A")
        self.ch_loop = StripChart("Navigation MCU smyčka", ["max µs"], "µs")
        self.charts = [self.ch_alt, self.ch_vel, self.ch_tilt, self.ch_rate, self.ch_act,
                       self.ch_fin, self.ch_pwr, self.ch_loop]
        for i, c_ in enumerate(self.charts):
            g.addWidget(c_, 1 + i // 2, i % 2)
        for r in range(1, 5):
            g.setRowStretch(r, 1)
        return w

    # ------------------------------------------------------------ raw IMU
    def _build_raw_tab(self) -> QtWidgets.QWidget:
        w = QtWidgets.QWidget()
        g = QtWidgets.QGridLayout(w)
        g.setContentsMargins(20, 16, 20, 20)
        g.setSpacing(12)
        top = QtWidgets.QHBoxLayout()
        self.raw_btn = button("Zapnout surová data", "primary")
        self.raw_btn.setCheckable(True)
        self.raw_btn.toggled.connect(lambda on: self.link.is_open and self.link.send(f"RAW {1 if on else 0}"))
        top.addWidget(self.raw_btn)
        top.addWidget(label("Position MCU posílá ~25×/s, jen na zemi. Osy desky TriSense, jak jsou "
                            "natištěné: na rampě míří −Y nahoru (k nosu), při přistání +Y nahoru. "
                            "V klidu na rampě má akcelerometr ukazovat ≈ (0, −1, 0) g.", "muted"))
        top.addStretch(1)
        g.addLayout(top, 0, 0, 1, 2)

        kv = Card("Hodnoty", "Poslední vzorek")
        self.raw_kv = KV(["Akcelerometr X / Y / Z", "|a|", "Gyroskop X / Y / Z", "|ω|",
                          "Magnetometr surový X / Y / Z", "Magnetometr kalibrovaný X / Y / Z", "|B| kalibr.",
                          "Barometr", "Teplota", "Zdroj / epochy"])
        kv.lay.addWidget(self.raw_kv)
        g.addWidget(kv, 1, 0, 3, 1, Qt.AlignmentFlag.AlignTop)
        self.ch_raw_acc = StripChart("Akcelerometr", ["X", "Y", "Z"], "g", span_s=10.0)
        self.ch_raw_gyr = StripChart("Gyroskop", ["X", "Y", "Z"], "°/s", span_s=10.0)
        self.ch_raw_mag = StripChart("Magnetometr (kalibrovaný)", ["X", "Y", "Z"], "µT", span_s=10.0)
        for i, c_ in enumerate((self.ch_raw_acc, self.ch_raw_gyr, self.ch_raw_mag)):
            c_.setMinimumHeight(180)
            g.addWidget(c_, 1 + i, 1)
        g.setColumnStretch(0, 1)
        g.setColumnStretch(1, 2)
        for r in (1, 2, 3):
            g.setRowStretch(r, 1)
        return w

    def _refresh_raw(self, d: dict) -> None:
        def v3(k, nd):
            x = d.get(k) or [0.0, 0.0, 0.0]
            return " / ".join(f"{c:+.{nd}f}" for c in x), math.sqrt(sum(c * c for c in x))
        a, am = v3("acc", 3)
        gy, gm = v3("gyr", 2)
        m, _ = v3("mag", 1)
        mc, mcm = v3("magc", 1)
        kv = self.raw_kv
        kv.set("Akcelerometr X / Y / Z", a + " g")
        kv.set("|a|", f"{am:.3f} g", GOOD if abs(am - 1.0) < 0.04 else None)
        kv.set("Gyroskop X / Y / Z", gy + " °/s")
        kv.set("|ω|", f"{gm:.2f} °/s", GOOD if gm < 1.0 else None)
        kv.set("Magnetometr surový X / Y / Z", m + " µT" + ("" if d.get("mf") else "  (bez nového vzorku)"))
        kv.set("Magnetometr kalibrovaný X / Y / Z", mc + " µT")
        kv.set("|B| kalibr.", f"{mcm:.1f} µT", GOOD if 25.0 < mcm < 65.0 else WARN)
        kv.set("Barometr", f"{d.get('p', 0.0):.1f} Pa")
        kv.set("Teplota", f"{d.get('temp', 0.0):.2f} °C")
        kv.set("Zdroj / epochy", ("INSLIB" if d.get("src") else "TriSense fusion") + f" · {d.get('hz', 0)} Hz")

    def _span_changed(self) -> None:
        s = float(self.span_box.currentData())
        for c_ in self.charts:
            c_.span = s
            c_.update()

    # ------------------------------------------------------------ params
    def _build_params(self) -> QtWidgets.QWidget:
        w = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(w)
        lay.setContentsMargins(20, 16, 20, 20)
        lay.setSpacing(10)
        note = label(
            "Tabulka parametrů žije ve flash W25Q128 Masteru a ten ji na rampě posílá Position i Navigation MCU. "
            "Měnit lze jen v IDLE nebo TEST. Postup: Načíst z desky → upravit hodnoty (žlutě) → "
            "Odeslat změny → Uložit do flash. Tabulka zážehu má vlastní záložku. Zastrčit lze jen, když obě desky drží stejnou tabulku "
            "(CRC souhlasí). Najetím myši na řádek se zobrazí, co parametr dělá.", "muted")
        note.setWordWrap(True)
        lay.addWidget(note)
        row = QtWidgets.QHBoxLayout()
        for txt, kind, fn, tip in (
                ("Načíst z desky", "primary", lambda: self._cmd("PARAM LIST"), "PARAM LIST"),
                ("Odeslat změny", "primary", self.param_send, "PARAM SET pro každou žlutou hodnotu"),
                ("Uložit do flash", "", lambda: self._cmd("PARAM SAVE"), "PARAM SAVE: přežije restart (W25Q128)"),
                ("Načíst z flash", "", lambda: self._cmd("PARAM LOAD"), "PARAM LOAD: zahodí neuložené změny"),
                ("Výchozí hodnoty", "danger", self.param_defaults, "PARAM DEFAULTS (neuloží)"),
                ("Export profilu…", "", self.param_export, "Uloží tabulku do JSON na počítači"),
                ("Import profilu…", "", self.param_import, "Načte JSON a označí rozdíly k odeslání")):
            b = button(txt, kind, tip)
            b.clicked.connect(fn)
            row.addWidget(b)
        row.addStretch(1)
        self.filter = QtWidgets.QLineEdit()
        self.filter.setPlaceholderText("filtr (např. seq. nebo fin)")
        self.filter.textChanged.connect(self._param_filter)
        row.addWidget(self.filter)
        lay.addLayout(row)
        self.ptable = QtWidgets.QTableWidget(0, 9)
        self.ptable.setHorizontalHeaderLabels(["id", "parametr", "hodnota", "výchozí", "min", "max",
                                               "jednotka", "deska", "co dělá"])
        self.ptable.verticalHeader().setVisible(False)
        self.ptable.horizontalHeader().setStretchLastSection(True)
        self.ptable.setWordWrap(False)
        self.ptable.itemChanged.connect(self._param_item_changed)
        lay.addWidget(self.ptable, 1)
        self._param_fill()
        return w

    def _param_fill(self) -> None:
        self.ptable.blockSignals(True)
        self.ptable.setRowCount(len(self.params))
        self.param_rows.clear()
        owners = {"N": "Navigation", "M": "Master", "P": "Position"}
        for r, p in enumerate(self.params):
            self.param_rows[p["id"]] = r
            vals = [str(p["id"]), p["key"], f"{p['val']:.6g}", f"{p['def']:.6g}", f"{p['lo']:.6g}",
                    f"{p['hi']:.6g}", p["unit"], owners.get(p["owner"], p["owner"]),
                    PARAM_HELP.get(p["key"], "")]
            for c_, v in enumerate(vals):
                it = QtWidgets.QTableWidgetItem(v)
                if c_ != 2:
                    it.setFlags(it.flags() & ~Qt.ItemFlag.ItemIsEditable)
                it.setToolTip(PARAM_HELP.get(p["key"], p["key"]))
                self.ptable.setItem(r, c_, it)
            self._param_color(p["id"])
        self.ptable.blockSignals(False)
        for c_ in range(8):
            self.ptable.resizeColumnToContents(c_)
        self._param_filter()

    def _param_color(self, pid: int) -> None:
        r = self.param_rows.get(pid)
        if r is None:
            return
        p = self.params[r]
        it = self.ptable.item(r, 2)
        if pid in self.param_edits:
            it.setBackground(QtGui.QColor(245, 165, 28, 70))
        elif abs(p["val"] - p["def"]) > 1e-6 * (1 + abs(p["def"])):
            it.setBackground(QtGui.QColor(74, 163, 255, 45))      # differs from default
        else:
            it.setBackground(QtGui.QBrush())

    def _param_item_changed(self, it: QtWidgets.QTableWidgetItem) -> None:
        if it.column() != 2:
            return
        p = self.params[it.row()]
        try:
            v = float(it.text().replace(",", "."))
        except ValueError:
            it.setText(f"{p['val']:.6g}")
            return
        v = max(p["lo"], min(p["hi"], v))
        self.param_edits[p["id"]] = v
        self.ptable.blockSignals(True)
        it.setText(f"{v:.6g}")
        self.ptable.blockSignals(False)
        self._param_color(p["id"])

    def _param_filter(self, txt: str = "") -> None:
        txt = self.filter.text() if hasattr(self, "filter") else txt
        for r, p in enumerate(self.params):
            # the 64 band entries live on their own tab
            hide = p["key"].startswith("tbl.hm") or (bool(txt) and txt.lower() not in p["key"].lower())
            self.ptable.setRowHidden(r, hide)

    def param_send(self) -> None:
        if not self.link.is_open:
            return
        risky = [k for k in self.param_edits
                 if self.params[self.param_rows[k]]["key"] in ("srv.tvc_x_sign", "srv.tvc_y_sign")]
        if risky:
            r = QtWidgets.QMessageBox.warning(
                self, APP_NAME, "Měníte ZNAMÉNKO serva TVC. Špatné znaménko = kladná zpětná vazba a "
                "pád do 1 s. Po změně VŽDY udělejte zkoušku znamének (manuál, kap. Předletové testy).",
                QtWidgets.QMessageBox.StandardButton.Ok | QtWidgets.QMessageBox.StandardButton.Cancel)
            if r != QtWidgets.QMessageBox.StandardButton.Ok:
                return
        for pid, v in sorted(self.param_edits.items()):
            self.link.send(f"PARAM SET {pid} {v:.6g}")
        self.log(f"> PARAM SET × {len(self.param_edits)}")

    def param_defaults(self) -> None:
        r = QtWidgets.QMessageBox.question(self, APP_NAME, "Vrátit všechny parametry na výchozí? "
                                           "(do flash se zapíše až tlačítkem Uložit)")
        if r == QtWidgets.QMessageBox.StandardButton.Yes:
            self._cmd("PARAM DEFAULTS")
            QtCore.QTimer.singleShot(300, lambda: self._cmd("PARAM LIST"))

    def param_export(self) -> None:
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Export", "daidalos_params.json", "JSON (*.json)")
        if path:
            with open(path, "w", encoding="utf-8") as fh:
                json.dump({p["key"]: self.param_edits.get(p["id"], p["val"]) for p in self.params}, fh, indent=2)

    def param_import(self) -> None:
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Import", "", "JSON (*.json)")
        if not path:
            return
        with open(path, encoding="utf-8") as fh:
            d = json.load(fh)
        for p in self.params:
            if p["key"] in d and abs(float(d[p["key"]]) - p["val"]) > 1e-9:
                self.param_edits[p["id"]] = max(p["lo"], min(p["hi"], float(d[p["key"]])))
        self._param_fill()
        for pid, v in self.param_edits.items():
            self.ptable.item(self.param_rows[pid], 2).setText(f"{v:.6g}")
        self.log(f"# import: {len(self.param_edits)} rozdílů, odešlete tlačítkem „Odeslat změny“")

    # ------------------------------------------------------------ lines
    def on_link_state(self, up: bool, msg: str) -> None:
        super().on_link_state(up, msg)
        self.flight_fw = False
        self.test_on = False
        self.test_btn.setChecked(False)
        if hasattr(self, "raw_btn"):
            self.raw_btn.blockSignals(True)
            self.raw_btn.setChecked(False)
            self.raw_btn.blockSignals(False)
            self.raw_btn.setText("Zapnout surová data")
        if up:
            QtCore.QTimer.singleShot(500, lambda: self.link.is_open and self.link.send("PARAM LIST"))

    def on_line(self, line: str) -> None:
        if self.rec_writer:
            self.rec_writer.writerow([f"{time.time():.3f}", line[:1], line])
        if len(line) > 2 and line[0] in "MNPGR" and line[1] == " " and line[2] == "{":
            try:
                self.d[line[0]] = json.loads(line[2:])
                self.d_time[line[0]] = time.monotonic()
            except ValueError:
                pass
            self._feed_charts(line[0])
            return
        if line.startswith("T "):
            super().on_line(line)
            try:
                self.test_on = bool(json.loads(line[2:]).get("test", 0)) if self.flight_fw else True
            except ValueError:
                pass
            return
        if line.startswith("PRM "):
            self._prm(line)
            return
        if line.startswith("SRVCHK "):
            d = parse_srvchk(line)
            if d is not None:
                self.calib_tab.on_srvchk(d)
            return
        if line.startswith(("LOGR ", "LOGF ", "OK LOG", "ERR LOG")):
            if hasattr(self, "log_tab"):
                self.log_tab.on_line(line)
            if not line.startswith("LOGR "):
                self.log(line)
            return
        if line.startswith("I DAIDALOS"):
            self.flight_fw = True
            self.fw_info = line[2:]
        elif line.startswith("OK PARAM LIST"):
            parts = line.split()
            self.board_crc = int(parts[-1]) if parts[-1].isdigit() else None
            self.param_edits.clear()
            self._param_fill()
            if hasattr(self, "ign_tab"):
                self.ign_tab.load_from_params()
        elif line.startswith("OK PARAM ") and len(line.split()) >= 5 and line.split()[2].isdigit():
            _, _, pid, _key, val = line.split()[:5]
            pid = int(pid)
            if pid in self.param_rows:
                self.params[self.param_rows[pid]]["val"] = float(val)
                self.param_edits.pop(pid, None)
                self.ptable.blockSignals(True)
                self.ptable.item(self.param_rows[pid], 2).setText(val)
                self.ptable.blockSignals(False)
                self._param_color(pid)
        elif line.startswith("OK TEST 1"):
            self.test_on = True
            # The console's PWM / IDLE / LIM are accepted only in TEST: send them now.
            self.link.send(f"PWM {self.pwm_box.currentData()}")
            self.link.send(f"IDLE {self.idle_spin.value()}")
            self.send_limits()
        elif line.startswith("OK TEST 0"):
            self.test_on = False
        elif line.startswith("OK RAW"):
            on = line.strip().endswith("1")
            self.raw_btn.blockSignals(True)
            self.raw_btn.setChecked(on)
            self.raw_btn.blockSignals(False)
            self.raw_btn.setText("Vypnout surová data" if on else "Zapnout surová data")
        elif line.startswith("ERR RAW"):
            self.raw_btn.blockSignals(True)
            self.raw_btn.setChecked(False)
            self.raw_btn.blockSignals(False)
            self.raw_btn.setText("Zapnout surová data")
        super().on_line(line)

    def _prm(self, line: str) -> None:
        p = line.split()
        if len(p) < 9:
            return
        pid = int(p[1])
        # the owner is always the last token; a unit never has a space, but be tolerant
        entry = {"id": pid, "key": p[2], "val": float(p[3]), "def": float(p[4]), "lo": float(p[5]),
                 "hi": float(p[6]), "unit": " ".join(p[7:-1]), "owner": p[-1], "name": p[2]}
        if pid in self.param_rows:
            self.params[self.param_rows[pid]].update(entry)
        else:
            self.params.append(entry)
            self.param_rows[pid] = len(self.params) - 1

    # ------------------------------------------------------------ refresh
    def _fresh_d(self, k: str) -> bool:
        return self.link.is_open and time.monotonic() - self.d_time[k] < 1.0

    def _feed_charts(self, k: str) -> None:
        if k == "R" and self.d["R"] and self.pause_btn.isChecked():
            self.calib_tab.on_raw(self.d["R"])        # calibration never pauses
        if self.pause_btn.isChecked():
            return
        t = time.monotonic() - self.t0
        d = self.d[k]
        if k == "N" and d:
            p, v, w = d.get("p", [0, 0, 0]), d.get("v", [0, 0, 0]), d.get("w", [0, 0, 0])
            self.ch_alt.add(t, [p[2]])
            self.ch_vel.add(t, v)
            self.ch_rate.add(t, [math.degrees(x) for x in w])
            q = d.get("q", [1, 0, 0, 0])
            tc = math.degrees(self.d["G"].get("tc", 0.0)) if self.d["G"] else None
            self.ch_tilt.add(t, [attitude_summary(*q)["tilt"], tc])
        elif k == "G" and d:
            tvc = d.get("tvc", [0, 0])
            self.ch_act.add(t, [tvc[0], tvc[1], 10.0 * d.get("cl", 1.0)])
            self.ch_fin.add(t, d.get("fin", [0, 0, 0, 0]))
            self.ch_loop.add(t, [d.get("loop", 0)])
        elif k == "M" and d:
            self.ch_pwr.add(t, [d.get("vbat"), d.get("i")])
        elif k == "R" and d:
            self.calib_tab.on_raw(d)
            self.ch_raw_acc.add(t, d.get("acc", [0, 0, 0]))
            self.ch_raw_gyr.add(t, d.get("gyr", [0, 0, 0]))
            self.ch_raw_mag.add(t, d.get("magc", [0, 0, 0]))
            self._refresh_raw(d)

    def _refresh_status(self) -> None:
        super()._refresh_status()
        if getattr(self, "flight_fw", False) and not getattr(self, "test_on", False):
            # Flight firmware outside TEST: the actuator page is read-only.
            self.pyro_note.setText("Letový firmware: ruční ovládání serv a pyro jen v TEST režimu "
                                   "(tlačítko nahoře). Mimo TEST ukazují ukazatele, co posílá letový software.")
            self.arm_btn.setEnabled(False)

    def _refresh_diag(self) -> None:
        M = self.d["M"] if self._fresh_d("M") else {}
        N = self.d["N"] if self._fresh_d("N") else {}
        P = self.d["P"] if self._fresh_d("P") else {}
        G = self.d["G"] if self._fresh_d("G") else {}
        problems: list[tuple[str, str]] = []
        self.calib_tab.on_status(P)

        # ---- top bar
        if not self.link.is_open:
            set_chip(self.chip_state, "Stav: odpojeno", "")
        elif not M:
            set_chip(self.chip_state, "Stav: —  (žádné M řádky: starý firmware?)", "warn")
        else:
            st = M.get("st", 0)
            set_chip(self.chip_state, f"Stav: {STATE_NAMES.get(st, st)} · {STATE_CZ.get(st, '')}",
                     "hot" if st == 2 else ("bad" if st in (9, 11) else ("warn" if st == 10 else "ok")))
        fws = {k: d.get("fw") for k, d in (("M", M), ("P", P), ("G", G)) if d}
        if fws:
            vals = set(fws.values())
            txt = " / ".join(f"{k}:{v / 100:.2f}" for k, v in fws.items())
            set_chip(self.chip_fw, f"Firmware {txt}", "ok" if len(vals) == 1 and len(fws) == 3 else "warn")
            if len(vals) > 1:
                problems.append((BAD, "Desky mají různé verze firmwaru – nahrajte stejnou verzi do všech tří."))
        else:
            set_chip(self.chip_fw, "Firmware: —", "")
        if M:
            sync = bool(M.get("psync"))
            set_chip(self.chip_sync, f"Parametry: {'synchronní' if sync else 'NESOUHLASÍ'} · CRC {M.get('pcrc', 0):04X}"
                     + ("" if M.get("peep") else " · výchozí (nic ve flash)"), "ok" if sync else "bad")
            if not sync:
                problems.append((BAD, "Position/Navigation nedrží stejnou tabulku parametrů jako Master – "
                                      "počkejte pár sekund; trvá-li to, restartujte desky. Bez shody NELZE zastrčit."))
            if self.param_edits:
                problems.append((WARN, f"{len(self.param_edits)} upravených parametrů není odeslaných."))
        self.test_btn.setChecked(bool(self.test_on and self.flight_fw))
        self.test_btn.setEnabled(self.link.is_open and self.flight_fw)

        # ---- boards
        kv = self.board_kv["M"]
        if M:
            kv.set("Online", "ano", GOOD)
            kv.set("Firmware", f"{M.get('fw', 0) / 100:.2f}  (navedení {M.get('gv', 0) / 100:.2f})")
            kv.set("POST", "OK" if M.get("post") else "SELHAL", GOOD if M.get("post") else BAD)
            st = M.get("st", 0)
            kv.set("Stav", f"{STATE_NAMES.get(st, st)}  {STATE_CZ.get(st, '')}")
            vb = M.get("vbat", 0.0)
            kv.set("Baterie", fmt(vb, 2, "V"), BAD if 0.5 < vb < 7.0 else (WARN if vb < 7.4 else GOOD))
            kv.set("Proud", fmt(M.get("i"), 2, "A"))
            kv.set("RBF", "vytažen" if M.get("rbf") else "zasunut", WARN if M.get("rbf") else GOOD)
            kv.set("Breakwire", "přerušen" if M.get("bw") else "celý", WARN if M.get("bw") else GOOD)
            kv.set("PCA9685", "OK" if M.get("servo") else "NEODPOVÍDÁ", GOOD if M.get("servo") else BAD)
            kv.set("Flash log", f"OK, {M.get('log', 0) // 1024} kB zapsáno, zahozeno {M.get('logdrop', 0)}"
                   if M.get("flash") else "CHYBÍ", GOOD if M.get("flash") else WARN)
            tl = M.get("tlm", [0, 0])
            kv.set("LoRa", f"OK, odesláno {tl[0]}, přeskočeno {tl[1]}" if M.get("lora") else "NEODPOVÍDÁ",
                   GOOD if M.get("lora") else BAD)
            for nm, k in (("Spoj Position", "pos"), ("Spoj Navigation", "nav")):
                v = M.get(k, [0, 0, 0])
                kv.set(nm, f"{'OK' if v[2] else 'MRTVÝ'}  rx {v[0]} / chyb {v[1]}", GOOD if v[2] else BAD)
                if not v[2]:
                    problems.append((BAD, f"Master neslyší {nm.split()[1]} MCU."))
            kv.set("Aerokryt", "ODJIŠTĚN" if M.get("cover") else "zajištěn", WARN if M.get("cover") else GOOD)
            kv.set("USB zahozeno", str(M.get("usbdrop", 0)))
            if not M.get("post"):
                problems.append((BAD, "Master: POST selhal (viz Záznam)."))
            if not M.get("servo"):
                problems.append((BAD, "PCA9685 neodpovídá: žádné TVC, křidélka ani aerokryt."))
            if 0.5 < vb < 7.0:
                problems.append((BAD, f"Baterie {vb:.2f} V – nabijte (2S, plná 8,4 V)."))
            elif 0.5 < vb < 7.4:
                problems.append((WARN, f"Baterie {vb:.2f} V – pod nominálem."))
            if not M.get("flash"):
                problems.append((WARN, "Flash log nefunguje – let nebude zaznamenán."))
            if not M.get("lora"):
                problems.append((BAD, "LoRa nefunguje – žádná telemetrie."))
        else:
            for n in kv.v:
                kv.set(n, "—", DIM)
            if self.link.is_open and self.flight_fw:
                problems.append((WARN, "Master neposílá stavové řádky."))

        kv = self.board_kv["P"]
        if P and P.get("ok"):
            kv.set("Online", "ano", GOOD)
            kv.set("Firmware", f"{P.get('fw', 0) / 100:.2f}")
            pc_ok = M and P.get("pcrc") == M.get("pcrc")
            kv.set("Parametry", f"{P.get('pseen', 0)} přijato, CRC {P.get('pcrc', 0):04X}", GOOD if pc_ok else WARN)
            kv.set("Senzory", " ".join(n for b, n in SENS_BITS if P.get("sens", 0) & (1 << b)) or "—")
            kv.set("Orientace", "INSLIB " + INS_MODES.get(P.get("ins", 255), "?") if P.get("att") else
                   "TriSense fúze")
            kv.set("Fúze Hz", fmt(P.get("hz"), 0, "Hz"), GOOD if P.get("hz", 0) > 150 else WARN)
            kv.set("IMU přetečení", str(P.get("ovf", 0)), WARN if P.get("ovf") else GOOD)
            kv.set("Kalibrace", {0: "nekalibrováno", 1: "gyro", 2: "gyro + znovu", 3: "plná"}.get(P.get("cal", 0), "?"))
            rx = P.get("rx", [0, 0])
            kv.set("Rx Master", f"{rx[0]} / chyb {rx[1]}")
            sens = P.get("sens", 0)
            for bit, name, sev in ((0, "IMU", BAD), (1, "barometr", BAD), (2, "magnetometr", WARN),
                                   (3, "GPS", WARN), (4, "UWB", WARN), (5, "lidar", WARN)):
                if not sens & (1 << bit):
                    problems.append((sev, f"Position MCU: {name} nefunguje."))
            if P.get("fix", 0) < 3:
                problems.append((WARN, f"GPS bez 3D fixu ({P.get('sats', 0)} družic)."))
            if P.get("uwb", [0])[0] < 3 and sens & 16:
                problems.append((WARN, f"UWB: jen {P.get('uwb', [0])[0]} kotvy – poloha horší."))
            if abs(P.get("trim", 0.0)) > 1.4:
                problems.append((WARN, "Baro trim na mezi: UWB kotvy jsou nejspíš špatně zaměřené."))
        else:
            for n in kv.v:
                kv.set(n, "—", DIM)
            if M:
                problems.append((BAD, "Position MCU neodpovídá."))

        kv = self.board_kv["G"]
        if G and G.get("ok"):
            kv.set("Online", "ano", GOOD)
            kv.set("Firmware", f"{G.get('fw', 0) / 100:.2f}")
            pc_ok = M and G.get("pcrc") == M.get("pcrc")
            kv.set("Parametry", f"{G.get('pseen', 0)} přijato, CRC {G.get('pcrc', 0):04X}", GOOD if pc_ok else WARN)
            kv.set("Fáze", PHASE_NAMES.get(G.get("ph", 0), "?"))
            kv.set("Smyčka max", f"{G.get('loop', 0)} µs (rozpočet 5000)", GOOD if G.get("loop", 0) < 2500 else WARN)
            kv.set("Přetečení", str(G.get("ovr", 0)), GOOD if not G.get("ovr") else WARN)
            kv.set("Plánovač", f"{G.get('plan', 0)} iterací", GOOD if G.get("plan") else BAD)
            rx = G.get("rx", [0, 0])
            kv.set("Rx Master", f"{rx[0]} / chyb {rx[1]}")
            cont = G.get("cont", 0)
            kv.set("Kontinuita", "  ".join(f"P{i + 1}:{'OK' if cont & (1 << i) else '—'}" for i in range(3)),
                   GOOD if cont & 1 else BAD)
            kv.set("Odpáleno", "  ".join(f"P{i + 1}" for i in range(3) if G.get("fired", 0) & (1 << i)) or "nic")
            kv.set("Proud teď", "  ".join(f"P{i + 1}" for i in range(3) if G.get("act", 0) & (1 << i)) or "nic",
                   BAD if G.get("act") else None)
            nfl = G.get("nfl", 0)
            if not nfl & 2:
                problems.append((BAD, "Navigation MCU: POST selhal – nezastrčovat."))
            for i, nm in enumerate(("hlavní roznět (PYRO1)", "záložní roznět (PYRO2)", "nohy (PYRO3)")):
                if not cont & (1 << i) and not G.get("fired", 0) & (1 << i):
                    problems.append((BAD if i == 0 else WARN, f"Bez kontinuity: {nm}."))
            if not nfl & 4:
                problems.append((WARN, "Navigation MCU nedostává navigační stav."))
            if G.get("apo", 0) > 190.0:
                problems.append((WARN, f"Apogeum {G['apo']:.0f} m je nad obálkou tabulky zážehu (190 m): "
                                       "přistávací motor tam přistání neuzavře, zážeh je jen nejlepší pokus."))
        else:
            for n in kv.v:
                kv.set(n, "—", DIM)
            if M:
                problems.append((BAD, "Navigation MCU neodpovídá."))

        # ---- attitude
        if N and N.get("q"):
            q = N["q"]
            a = attitude_summary(*q)
            self.rocket.set_quaternion(*q)
            tvc = G.get("tvc", [0, 0]) if G else [0, 0]
            self.rocket.set_actuators(fins=G.get("fin") if G else None, tvc=tvc,
                                      burning=bool(G) and G.get("ph") == 4)
            self.bubble.set_tilt(a["tilt"], a["tilt_az"], True,
                                 math.degrees(G.get("tc", 0)) if G and G.get("ph", 0) >= 2 else None)
            self.att_kv.set("Náklon", f"{a['tilt']:.2f}°  ({'špička nahoru' if a['nose_up'] else 'špička dolů'})",
                            GOOD if a["tilt"] < 4 else (WARN if a["tilt"] < 10 else BAD))
            self.att_kv.set("Směr náklonu", f"{a['tilt_az']:.0f}° (azimut, kam se naklání vršek)")
            self.att_kv.set("Roll", f"{a['roll']:.0f}°  (kam míří osa 1 / křidélko 1)")
            self.att_kv.set("Pitch / heading", f"{a['pitch']:.1f}° / {a['heading']:.0f}°")
            w = N.get("w", [0, 0, 0])
            self.att_kv.set("Úhlové rychlosti", " ".join(f"{math.degrees(x):+.1f}" for x in w) + " °/s")
            self.att_kv.set("Zdroj orientace", ("INSLIB · " + INS_MODES.get(P.get("ins", 255), "?"))
                            if P and P.get("att") else ("TriSense fúze" if P else "—"))
        else:
            self.rocket.set_quaternion(1, 0, 0, 0, valid=False)
            self.bubble.set_tilt(0, 0, False)

        # ---- flight
        fk = self.flight_kv
        if G:
            fk.set("Fáze navedení", PHASE_NAMES.get(G.get("ph", 0), "?"))
            fk.set("Apogeum", fmt(G.get("apo"), 1, "m"))
            fk.set("Výška zážehu", fmt(G.get("hign"), 1, "m") + "  (přepočet 10× za s)"
                   if G.get("hign") else "— (určí se v apogeu)")
            blo, bhi = G.get("blo", 0.0), G.get("bhi", 0.0)
            fk.set("Pásmo zážehu", (f"{blo:.1f} – {bhi:.1f} m" if bhi > 0 else
                                    (f"ŽÁDNÉ (nejlepší pokus {blo:.1f} m)" if blo else "—")),
                   None if bhi > 0 or not blo else BAD)
            dr = G.get("drag", 1.0)
            fk.set("Odpor měřený / model", fmt(dr, 2), GOOD if 0.8 < dr < 1.25 else WARN)
            fk.set("Do zážehu", fmt(G.get("tign"), 2, "s") if G.get("tign", -1) >= 0 else "—")
            fk.set("Do dosednutí", fmt(G.get("ttd"), 2, "s") if G.get("ttd", -1) >= 0 else "—")
            fk.set("Odhad tahu", fmt(G.get("thr"), 1, "N"))
            fk.set("Tah / tabulka", fmt(G.get("ts"), 3))
            fk.set("Škrcení plán", fmt(100 * G.get("k", 1.0), 0, "%"))
            res = G.get("res", 0.0)
            fk.set("Plán rezerva", fmt(res, 2, "m/s"), GOOD if res >= 0 else BAD)
            fk.set("Účinnost křidélek", fmt(G.get("fg"), 2) + "  (1 = podle CFD)")
            fk.set("Náklon příkaz / chyba", f"{math.degrees(G.get('tc', 0)):.1f}° / {math.degrees(G.get('te', 0)):.1f}°")
            self.ev_flags.set(G.get("ev", 0))
        else:
            for n in fk.v:
                fk.set(n, "—", DIM)
            self.ev_flags.set(None)

        # ---- nav tab
        if N:
            p, v, a_ = N.get("p", [0] * 3), N.get("v", [0] * 3), N.get("a", [0] * 3)
            self.nav_kv.set("Poloha E / N / výška", f"{p[0]:+.2f} / {p[1]:+.2f} / {p[2]:.2f} m")
            self.nav_kv.set("Rychlost E / N / svislá", f"{v[0]:+.2f} / {v[1]:+.2f} / {v[2]:+.2f} m/s")
            self.nav_kv.set("Zrychlení (bez g)", f"{a_[0]:+.2f} / {a_[1]:+.2f} / {a_[2]:+.2f} m/s²")
            self.nav_kv.set("|a| s gravitací", fmt(N.get("g"), 2, "g"))
            self.nav_kv.set("Kvalita", f"{N.get('ql', 0)} / 255", GOOD if N.get("ql", 0) > 150 else WARN)
            fl = N.get("fl", 0)
            self.nav_flags.set(fl, bad_when_off=(0, 1, 2))
            self.nav_flags2.set(fl)
            self.padmap.set_position(p[0], p[1], P.get("sig", [0])[0] if P else 0.0)
            if not fl & 1:
                problems.append((BAD, "Navigační řešení NENÍ platné (VALID) – nelze zastrčit."))
            if not fl & 32:
                problems.append((WARN, "Chybí reference rampy (Pad reference)."))
        if P:
            sig = P.get("sig", [0, 0])
            self.nav_kv.set("σ poloha / rychlost vodorovně", f"{sig[0]:.2f} m / {sig[1]:.2f} m/s")
            fix = P.get("fix", 0)
            self.sens_kv.set("GPS", f"fix {fix}D, {P.get('sats', 0)} družic, HDOP {P.get('hdop', 0) / 10:.1f}",
                             GOOD if fix >= 3 else WARN)
            self.sens_kv.set("Poloha (lat, lon)", f"{P.get('lat', 0) / 1e7:.7f}, {P.get('lon', 0) / 1e7:.7f}")
            self.sens_kv.set("GPS výška", fmt(P.get("galt"), 1, "m n. m."))
            u = P.get("uwb", [0, 0, 0, 0])
            self.sens_kv.set("UWB", f"{u[0]} kotvy (maska {u[1]:04b}), řešení {UWB_MODES.get(u[2], '?')}, "
                             f"odmítnuto {u[3]}", GOOD if u[0] >= 3 else WARN)
            us = P.get("uwbs", [0, 0])
            self.sens_kv.set("UWB přesnost / reziduál", f"{us[0]:.2f} m / {us[1]:.2f} m")
            self.sens_kv.set("Baro trim", fmt(P.get("trim"), 2, "m"), WARN if abs(P.get("trim", 0)) > 1.4 else None)
            self.sens_kv.set("Pad tlak", fmt(P.get("pp"), 0, "Pa"))
            li = P.get("lidar", [0, 0])
            self.sens_kv.set("Lidar", f"{li[0]:.2f} m, signál {li[1] * 256}" if li[0] else "neodpovídá / mimo rozsah",
                             GOOD if li[0] else WARN)
            self.sens_kv.set("Teplota", fmt(P.get("temp"), 1, "°C"))
            self.sens_kv.set("INS (INSLIB)", INS_MODES.get(P.get("ins", 255), "?"))
            self.sens_flags.set(P.get("sens", 0), bad_when_off=(0, 1))

        # ---- problems list
        self.problems.clear()
        if not self.link.is_open:
            problems = [(DIM, "Připojte Master USB kabelem a zvolte port (nebo Simulace).")]
        elif self.link.is_open and not self.flight_fw and not M:
            problems.insert(0, (WARN, "Deska neodpověděla jako letový firmware (I DAIDALOS). Je nahraný "
                                      "Examples/Actuator_Console? Pak funguje jen záložka Aktuátory."))
        if not problems:
            problems = [(GOOD, "Vše v pořádku.")]
        order = {BAD: 0, WARN: 1, GOOD: 2, DIM: 3}
        problems.sort(key=lambda x: order.get(x[0], 9))
        nbad = sum(1 for c_, _ in problems if c_ == BAD)
        nwarn = sum(1 for c_, _ in problems if c_ == WARN)
        for col, txt in problems:
            it = QtWidgets.QListWidgetItem(("✖  " if col == BAD else "▲  " if col == WARN else "✔  ") + txt)
            it.setForeground(QtGui.QColor(col))
            self.problems.addItem(it)
        set_chip(self.chip_err, f"Chyby: {nbad} · varování: {nwarn}",
                 "bad" if nbad else ("warn" if nwarn else ("ok" if self.link.is_open else "")))
        self.farm_btn.setEnabled(bool(M) and M.get("st") == 1)

    def closeEvent(self, e):
        if self.rec_file:
            self.rec_file.close()
        if self.test_on and self.link.is_open:
            self.link.send("TEST 0")
        super().closeEvent(e)


def main() -> int:
    ap = argparse.ArgumentParser(description=APP_NAME)
    ap.add_argument("--port", help="sériový port Masteru, např. COM5 nebo /dev/ttyACM0")
    ap.add_argument("--demo", action="store_true", help="simulovaná raketa, bez hardwaru")
    ap.add_argument("--config", default=ag.DEFAULT_CONFIG, help="konfigurace serv (JSON), sdílená s Actuator Console")
    ap.add_argument("--screenshot", help=argparse.SUPPRESS)
    ap.add_argument("--tab", type=int, default=0, help=argparse.SUPPRESS)
    args = ap.parse_args()

    QtWidgets.QApplication.setHighDpiScaleFactorRoundingPolicy(
        Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)
    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName(APP_NAME)
    ag.pick_font(app)
    app.setStyleSheet(ag.QSS)
    win = DiagWindow(args.config, args.demo, args.port)
    win.show()
    if args.screenshot:
        win.tabs.setCurrentIndex(args.tab)

        def shot():
            win.grab().save(args.screenshot)
            win._dirty = False
            app.quit()
        QtCore.QTimer.singleShot(3500, shot)
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
