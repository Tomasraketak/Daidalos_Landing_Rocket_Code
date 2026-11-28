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
from diag_hil import HilTab                            # noqa: E402
import hil_sim as hs                                   # noqa: E402

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
INS_MODES = {0: "žádný", 1: "jen orientace", 2: "bez GPS (coasting)", 3: "plný (GPS)",
             254: "HIL simulace", 255: "nezkompilováno"}
UWB_MODES = {0: "—", 1: "1 kotva (LOS)", 2: "2-D", 3: "3-D"}

# ---- vibration spectrum (fw 3.35): "V <t_ms> <hex>", the 88-byte ariadna_vib_t
VIB_FMT = "<4BH6H62B6B2B"
VIB_CH_NAMES = ["Vibrace osa (zrychlení)", "Vibrace bočně (zrychlení)",
                "Kmitání náklon (úhl. rychl.)", "Kmitání rotace (úhl. rychl.)"]
VIB_UNIT = ["g", "g", "°/s", "°/s"]


def vib_level(code: int, ch: int) -> float:
    """RMS of an 8-bit dB code (ariadna_link.h): g for channels 0-1, deg/s for 2-3."""
    if code == 0:
        return 0.0
    return 10.0 ** ((code * 0.5 - (100.0 if ch <= 1 else 70.0)) / 20.0)


def parse_vib(line: str) -> list[dict] | None:
    """A V line -> two channel dicts (rms, tones [(Hz, rms)], stat, fs), or None."""
    import struct
    p = line.split()
    if len(p) != 3:
        return None
    try:
        raw = bytes.fromhex(p[2])
        v = struct.unpack(VIB_FMT, raw)
    except (ValueError, struct.error):
        return None
    part, seq, stat, _n, fs = v[0:5]
    peak_f, peak_a, rms = v[5:11], v[73:79], v[79:81]
    out = []
    for j in range(2):
        ch = 2 * part + j
        if ch > 3:
            return None
        tones = [(peak_f[3 * j + i] / 100.0, vib_level(peak_a[3 * j + i], ch))
                 for i in range(3) if peak_f[3 * j + i]]
        out.append({"ch": ch, "seq": seq, "stat": stat, "fs": fs / 10.0,
                    "rms": vib_level(rms[j], ch), "tones": tones})
    return out
HEALTH_ERR = {1: "POST selhal", 2: "spoj s Position MCU", 3: "spoj s Navigation MCU",
              4: "flash log", 5: "sběrnice serv", 6: "nízká baterie", 7: "parametry nesouhlasí"}

# What every parameter does, for the editor's tooltip. Keys from ariadna_params.h.
PARAM_HELP = {
    "seq.cover_lead_s": "Kolik sekund PŘED příkazem k zážehu se odjistí aerokryt (servo, kanál 8).",
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
    "batt.cap_mah": "Kapacita baterie podle štítku, mAh (2S LiPo 2200 mAh).",
    "batt.health": "Kolik štítkové kapacity baterie ve skutečnosti dá (1,00 = tolik, co má na štítku; 0,85 = o 15 % méně). "
                   "Starší nebo levná baterie bývá níž. Ladění: vybijte ji známým proudem a porovnejte s výdrží, kterou GUI ukazuje.",
    "batt.r25_mohm": "Vnitřní odpor celé baterie včetně kabeláže při 25 °C, mΩ. Slouží k dopočtu poklesu napětí pod zátěží; "
                     "v zimě se sám zvětšuje (≈ 2× při 0 °C).",
    "batt.learn": "Co se baterie sama naučila: násobek batt.health. Master ho snižuje, když čas do limitu dlouhodobě ubývá rychleji než 1 min za 1 min, a ukládá do flash. Po výměně baterie nastavte 1,00.",
    "batt.cutoff_pct": "Při jakém nabití se počítá „výdrž do limitu“, %. Výchozí 20 %.",
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


PARAM_HELP["imu.gyr_off_x"] = ("Nulový offset gyra (osa X desky), °/s. Uloží se sám po „Kalibrovat gyro“. "
                               "Použije se jen, když se kalibrace při zapnutí liší o víc než 1 °/s.")
PARAM_HELP["imu.gyr_off_y"] = "Nulový offset gyra (osa Y desky), °/s."
PARAM_HELP["imu.gyr_off_z"] = "Nulový offset gyra (osa Z desky), °/s."
_SRV_NAMES = {"tvc_x": "TVC X", "tvc_y": "TVC Y", "clamp": "klapky", "cover": "aerokrytu",
              **{f"fin{i}": f"křidélka {i}" for i in range(1, 5)}}
for _k, _n in _SRV_NAMES.items():
    PARAM_HELP[f"srv.ch_{_k}"] = (f"Kanál PCA9685 (0–15) serva {_n}. Všech 8 letových serv musí mít různé kanály, "
                                  "jinak Master použije výchozí zapojení a nahlásí chybu serv (nelze zastrčit). "
                                  "Nastavuje se i z tabulky serv v záložce Aktuátory.")
    PARAM_HELP[f"srv.{_k}_min_us"] = f"Tvrdé minimum pulzu serva {_n}, µs: žádný příkaz ani trim ho nepřekročí."
    PARAM_HELP[f"srv.{_k}_max_us"] = f"Tvrdé maximum pulzu serva {_n}, µs. Min ≥ max = limit vypnutý (jen 500–2500)."
for _i in range(3):
    for _j in range(3):
        PARAM_HELP.setdefault(f"imu.mag_si_{_i}{_j}", f"Soft iron magnetometru, prvek matice [{_i}][{_j}]. "
                                                      "Z kalibrace v záložce Kalibrace.")

# The parameter editor's groups, in the order they are shown. A parameter
# goes to the FIRST group whose rule matches its key: an exact key, or a
# prefix ending in "." or "_". Anything new lands in "Ostatní" until it is
# given a home here.
PARAM_GROUPS: list[tuple[str, str, tuple[str, ...]]] = [
    ("Sekvence letu a pyro", "aerokryt, záložní roznět, nohy, apogeum (Navigation MCU)",
     ("seq.",)),
    ("Raketa a motor", "hmotnost, průměr, odpor, tah",
     ("g.mass_", "g.diameter_m", "g.cd_body", "g.fin_dcd_", "g.drag_adapt", "g.thrust_scale")),
    ("Zážeh a dosednutí", "kdy zapálit, plánovač klapky, detekce zážehu",
     ("g.h_ign_", "g.ign_delay_pad_s", "g.alt_pad_m", "g.plan_", "g.onset_", "tbl.")),
    ("Náklon a tryska (TVC)", "smyčka náklonu na trysce, povolený kužel náklonu",
     ("g.wn_tvc", "g.zeta_tvc", "g.sched_tvc", "g.att_", "g.tilt_")),
    ("Křidélka", "smyčka na křidélkách, brzda, drift, rotace",
     ("g.wn_fin", "g.zeta_fin", "g.sched_fin", "g.roll_", "fin.")),
    ("Serva – kalibrace", "trimy, µs/°, znaménka, klapka a aerokryt (Master)",
     ("srv.",)),
    ("Master – start a kontroly", "detekce startu, zastrčení, kontrola serv",
     ("mst.",)),
    ("Baterie (2S LiPo)", "kapacita, zdraví, odpor – z toho se počítá nabití a výdrž do limitu",
     ("batt.",)),
    ("Kalibrace senzorů a rampa", "akcelerometr, gyro, magnetometr, poloha rampy (z záložky Kalibrace)",
     ("imu.", "pos.pad_")),
    ("Poloha a senzory", "GPS, lidar, UWB, orientace za letu (Position MCU)",
     ("pos.",)),
]
PARAM_OTHER = ("Ostatní", "parametry bez skupiny", ())


def _param_id_max(path: str = PARAMS_H) -> int:
    try:
        m = re.search(r"ARIADNA_PARAM_ID_MAX\s*=\s*(\d+)", open(path, encoding="utf-8").read())
        return int(m.group(1)) if m else 232
    except OSError:
        return 232


PARAM_ID_MAX = _param_id_max()

# The servo config table of the actuator page <-> the Master's parameters
# (fw 3.28). Fields not listed (clamp / cover centre, the spare channels)
# exist only in the GUI's own JSON file. "reverse" is the servo's sign
# parameter (-1 = reversed); µs/° is shared by both TVC servos and by all fins.
SERVO_PARAMS: dict[str, dict[str, str]] = {
    "tvc_x": {"ch": "srv.ch_tvc_x", "min_us": "srv.tvc_x_min_us", "max_us": "srv.tvc_x_max_us",
              "center_us": "srv.tvc_x_trim_us", "reverse": "srv.tvc_x_sign", "us_per_deg": "srv.tvc_us_per_deg"},
    "tvc_y": {"ch": "srv.ch_tvc_y", "min_us": "srv.tvc_y_min_us", "max_us": "srv.tvc_y_max_us",
              "center_us": "srv.tvc_y_trim_us", "reverse": "srv.tvc_y_sign", "us_per_deg": "srv.tvc_us_per_deg"},
    "clamp": {"ch": "srv.ch_clamp", "min_us": "srv.clamp_min_us", "max_us": "srv.clamp_max_us"},
    "cover": {"ch": "srv.ch_cover", "min_us": "srv.cover_min_us", "max_us": "srv.cover_max_us",
              "lock_us": "srv.cover_lock_us", "unlock_us": "srv.cover_unlock_us"},
    **{f"fin{i}": {"ch": f"srv.ch_fin{i}", "min_us": f"srv.fin{i}_min_us", "max_us": f"srv.fin{i}_max_us",
                   "center_us": f"srv.fin{i}_trim_us", "reverse": f"srv.fin{i}_sign",
                   "us_per_deg": "srv.fin_us_per_deg"} for i in range(1, 5)},
}


def param_group(key: str) -> int:
    """Index into PARAM_GROUPS (len(PARAM_GROUPS) = "Ostatní")."""
    for i, (_, _, rules) in enumerate(PARAM_GROUPS):
        for r in rules:
            if (key.startswith(r) if r.endswith((".", "_")) else key == r):
                return i
    return len(PARAM_GROUPS)


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
    firmware's diagnostic protocol: TEST mode, HIL simulation mode, the
    parameter table, and the M / N / P / G / H status lines."""

    def __init__(self):
        super().__init__()
        self.test = False
        self.state = 1
        self.params = load_param_table()
        self.goff = [0.412, -0.287, 0.153]                 # deg/s, the "boot calibration"
        self.next_diag = 0.0
        self.which = 0
        self.list_queue: list[str] = []
        self.raw = False
        self.next_raw = 0.0
        self.next_uwb = 0.0
        self.srv_t0 = None
        self.srv_last = -1

        # HIL simulation state (emulates Position + Master + Navigation in --demo)
        import threading
        self._hil_lock = threading.Lock()
        self.hil = False
        self.hil_pyro = False
        self.polarity = False
        self._pol_uz = -1.0
        self._pol_thrust_n = 0.0
        self._pol_speed_ms = 0.0
        self._pol_demo_tilt: tuple[float, float, bool] | None = None
        self.next_h = 0.0
        self._hil_pos_t = 0.0
        self._hil_last_t = 0.0
        self._hil_t = 0.0
        self._hil_p0 = 97230.0
        self._hil_lat0 = 49.7985411
        self._hil_lon0 = 16.6912772
        self._hil_pos = [0.0, 0.0, 0.0]
        self._hil_vel = [0.0, 0.0, 0.0]
        self._hil_acc = [0.0, 0.0, 0.0]
        self._hil_q = [0.70710678, -0.70710678, 0.0, 0.0]
        self._hil_w = [0.0, 0.0, 0.0]
        self._hil_g = 1.0
        self._hil_lidar_m = 0.0
        self._hil_lat_1e7 = 497985411
        self._hil_lon_1e7 = 166912772
        self._hil_alt_msl = 412.3
        self._hil_temp = 21.4
        self._hil_guid: hs.ReferenceGuidance | None = None
        self._hil_out: dict = self._idle_hil_out()

    @staticmethod
    def _idle_hil_out() -> dict:
        return {
            "mst_state": 1, "nav_phase": 0, "nav_events": 0,
            "pyro_active": 0, "pyro_fired": 0, "cover_open": 0,
            "tvc_deg": (0.0, 0.0), "clamp": 1.0,
            "fin_deg": (0.0, 0.0, 0.0, 0.0), "act_flags": 1,
            "h_ign": 0.0, "thr_est": 0.0, "k_plan": 1.0,
            "t_to_ign": -1.0, "t_to_td": -1.0, "tilt_cmd": 0.0, "tilt_err": 0.0,
        }

    def _hil_servo_us(self, out: dict) -> list[int]:
        tvc = out.get("tvc_deg", (0.0, 0.0))
        cl = out.get("clamp", 1.0)
        cov = out.get("cover_open", 0)
        fin = out.get("fin_deg", (0.0, 0.0, 0.0, 0.0))
        return [
            int(round(1500.0 + tvc[0] * 10.0)),
            int(round(1500.0 + tvc[1] * 10.0)),
            int(round(1100.0 + cl * 800.0)),
            1900 if cov else 1100,
            int(round(1500.0 + fin[0] * 10.0)),
            int(round(1500.0 + fin[1] * 10.0)),
            int(round(1500.0 + fin[2] * 10.0)),
            int(round(1500.0 + fin[3] * 10.0)),
        ]

    def _hil_pos_fresh(self, now: float) -> bool:
        return (now - self._hil_pos_t) < 0.5

    def hil_feed_pos(self, sample: hs.HilSensorSample) -> None:
        """Emulate Position MCU receiving a binary ARIADNA_PKT_HIL_IN frame,
        running its estimator, and forwarding nav state to Master + Navigation."""
        now = time.monotonic()
        with self._hil_lock:
            dt = 0.01 if self._hil_last_t <= 0.0 else min(0.05, max(0.002, now - self._hil_last_t))
            self._hil_last_t = now
            self._hil_pos_t = now

            if sample.flags & hs.HIL_FLAG_SET_PAD_REF:
                self._hil_p0 = max(1000.0, float(sample.pressure_pa))
                self._hil_lat0 = sample.gps_lat_1e7 * 1e-7
                self._hil_lon0 = sample.gps_lon_1e7 * 1e-7
            if sample.flags & hs.HIL_FLAG_RESET_EST:
                self._hil_pos = [0.0, 0.0, 0.0]
                self._hil_vel = [0.0, 0.0, 0.0]
                self._hil_acc = [0.0, 0.0, 0.0]

            # Rotate synthetic body-frame IMU into pad-ENU (mirrors ariadna_hil_apply_imu)
            q = (sample.qw, sample.qx, sample.qy, sample.qz)
            qn = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]) or 1.0
            q = (q[0] / qn, q[1] / qn, q[2] / qn, q[3] / qn)
            R = hs.rot_from_quat(q)
            ab = sample.acc_body
            gb = sample.gyr_body
            sf_pad = (
                R[0][0] * ab[0] + R[0][1] * ab[1] + R[0][2] * ab[2],
                R[1][0] * ab[0] + R[1][1] * ab[1] + R[1][2] * ab[2],
                R[2][0] * ab[0] + R[2][1] * ab[1] + R[2][2] * ab[2],
            )
            a_pad = (sf_pad[0], sf_pad[1], sf_pad[2] - hs.G_ACCEL)
            w_pad = (
                R[0][0] * gb[0] + R[0][1] * gb[1] + R[0][2] * gb[2],
                R[1][0] * gb[0] + R[1][1] * gb[1] + R[1][2] * gb[2],
                R[2][0] * gb[0] + R[2][1] * gb[1] + R[2][2] * gb[2],
            )
            self._hil_q = list(q)
            self._hil_acc = list(a_pad)
            self._hil_w = list(w_pad)
            self._hil_g = math.sqrt(ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2]) / hs.G_ACCEL
            self._hil_lat_1e7 = sample.gps_lat_1e7
            self._hil_lon_1e7 = sample.gps_lon_1e7
            self._hil_alt_msl = sample.gps_alt_msl
            self._hil_temp = sample.temp_c_x10 * 0.1
            self._hil_lidar_m = sample.lidar_mm * 0.001 if (sample.flags & hs.HIL_FLAG_LIDAR_VALID) else 0.0

            # Lightweight complementary KF (fuses IMU + baro + GPS + lidar)
            for i in range(3):
                self._hil_vel[i] += a_pad[i] * dt
                self._hil_pos[i] += self._hil_vel[i] * dt
            if sample.flags & hs.HIL_FLAG_BARO_FRESH and sample.pressure_pa > 1000.0:
                h_baro = 44330.0 * (1.0 - (sample.pressure_pa / max(self._hil_p0, 1000.0)) ** 0.19029495)
                err_z = h_baro - self._hil_pos[2]
                self._hil_pos[2] += 0.22 * err_z
                self._hil_vel[2] += 0.04 * err_z
            if sample.flags & hs.HIL_FLAG_LIDAR_VALID and sample.lidar_mm > 0:
                cos_t = abs(R[2][1])
                if cos_t > 0.7:
                    h_lid = (sample.lidar_mm * 0.001) * cos_t
                    self._hil_pos[2] += 0.35 * (h_lid - self._hil_pos[2])
            if sample.flags & hs.HIL_FLAG_GPS_FRESH:
                m_lat, m_lon = hs.wgs84_radii(self._hil_lat0)
                pe = math.radians(sample.gps_lon_1e7 * 1e-7 - self._hil_lon0) * m_lon
                pn = math.radians(sample.gps_lat_1e7 * 1e-7 - self._hil_lat0) * m_lat
                self._hil_pos[0] += 0.25 * (pe - self._hil_pos[0])
                self._hil_pos[1] += 0.25 * (pn - self._hil_pos[1])
                self._hil_vel[0] += 0.20 * (sample.gps_ve_cms * 0.01 - self._hil_vel[0])
                self._hil_vel[1] += 0.20 * (sample.gps_vn_cms * 0.01 - self._hil_vel[1])
            if not self.hil and abs(self._hil_pos[2]) < 0.5 and math.hypot(*a_pad) < 0.8:
                # Pad ZUPT while waiting for launch
                for i in range(3):
                    self._hil_vel[i] *= 0.6
                    self._hil_pos[i] *= 0.85

            if self.hil and self._hil_guid is not None:
                self._hil_t += dt
                b_vec = (R[0][1], R[1][1], R[2][1])
                g_vec = (R[0][0], R[1][0], R[2][0])
                out = self._hil_guid.step(
                    self._hil_t, dt, self._hil_pos, self._hil_vel,
                    b_vec, g_vec, w_pad, ab[1], pyro_permitted=self.hil_pyro,
                )
                self._hil_out = out
                self.state = out["mst_state"]

    def crc(self) -> int:
        import struct
        v = [0.0] * PARAM_ID_MAX
        for p in self.params:
            v[p["id"]] = p["val"]
        return crc16(struct.pack(f"<{PARAM_ID_MAX}f", *v))

    def _hil_leave(self) -> None:
        with self._hil_lock:
            self.hil = False
            self.hil_pyro = False
            self._hil_guid = None
            self._hil_out = self._idle_hil_out()
            self.state = 1

    def _cmd(self, line: str) -> None:
        if not line:
            return
        p = line.split()
        c = p[0].upper()
        if c == "HELLO":
            super()._cmd(line)
            self.q.put("I DAIDALOS 332 FLIGHT guidance 220 params %d" % len(self.params))
            return
        if c == "TEST":
            on = len(p) > 1 and p[1] == "1"
            if on and self.hil:
                self.q.put("ERR TEST leave HIL first (send HIL 0)")
                return
            self.test = on
            self.state = 10 if self.test else 1
            if not self.test:
                self.armed_until, self.fire_ch = 0, -1
            self.q.put(f"OK TEST {1 if self.test else 0}")
            return
        if c == "HIL":
            sub = p[1].upper() if len(p) > 1 else "STATUS"
            if sub == "STATUS":
                self.q.put(f"OK HIL STATUS {1 if self.hil else 0} {1 if self.hil_pyro else 0} {self.state}")
                return
            if sub == "0":
                self._hil_leave()
                self.q.put("OK HIL 0")
                return
            if sub == "1":
                if self.hil:
                    self.q.put("ERR HIL already active (send HIL 0 first)")
                    return
                if self.state not in (1, 10):
                    self.q.put(f"ERR HIL not on the ground (state {self.state})")
                    return
                pyro_en = len(p) > 2 and p[2] == "1"
                with self._hil_lock:
                    self.test = False
                    self.polarity = False
                    self.hil = True
                    self.hil_pyro = pyro_en
                    self._hil_t = 0.0
                    self._hil_guid = hs.ReferenceGuidance(hs.HilConfig())
                    self._hil_out = self._idle_hil_out()
                    self._hil_out["mst_state"] = 2
                    self.state = 2
                    self.next_h = time.monotonic()
                self.q.put(f"OK HIL 1 {1 if pyro_en else 0}")
                return
            self.q.put("ERR HIL 1 [0|1] | 0 | STATUS")
            return
        if c == "POLARITY":
            sub = p[1].upper() if len(p) > 1 else "STATUS"
            if sub == "STATUS":
                self.q.put(f"OK POLARITY STATUS {1 if self.polarity else 0} "
                           f"{self._pol_thrust_n:.0f} {self._pol_speed_ms:.0f}")
                return
            if sub == "THRUST":
                if len(p) < 3:
                    self.q.put("ERR POLARITY THRUST <0|10..250>")
                    return
                try:
                    v = float(p[2])
                except ValueError:
                    self.q.put("ERR POLARITY THRUST <0|10..250>")
                    return
                self._pol_thrust_n = 0.0 if v <= 0.0 else max(10.0, min(250.0, v))
                self.q.put(f"OK POLARITY THRUST {self._pol_thrust_n:.0f}")
                return
            if sub == "SPEED":
                if len(p) < 3:
                    self.q.put("ERR POLARITY SPEED <0|5..70>")
                    return
                try:
                    v = float(p[2])
                except ValueError:
                    self.q.put("ERR POLARITY SPEED <0|5..70>")
                    return
                self._pol_speed_ms = 0.0 if v <= 0.0 else max(5.0, min(70.0, v))
                self.q.put(f"OK POLARITY SPEED {self._pol_speed_ms:.0f}")
                return
            if sub == "0":
                self.polarity = False
                self._pol_demo_tilt = None
                self.q.put("OK POLARITY 0")
                return
            if sub == "1":
                if self.test:
                    self.q.put("ERR POLARITY leave TEST first")
                    return
                if self.hil:
                    self.q.put("ERR POLARITY leave HIL first")
                    return
                if self.state != 1:
                    self.q.put(f"ERR POLARITY only in IDLE (state {self.state})")
                    return
                if len(p) >= 3:
                    try:
                        v = float(p[2])
                        self._pol_thrust_n = 0.0 if v <= 0.0 else max(10.0, min(250.0, v))
                    except ValueError:
                        pass
                if len(p) >= 4:
                    try:
                        v = float(p[3])
                        self._pol_speed_ms = 0.0 if v <= 0.0 else max(5.0, min(70.0, v))
                    except ValueError:
                        pass
                self.polarity = True
                self.q.put("OK POLARITY 1")
                return
            self.q.put("ERR POLARITY 1 [N] [m/s] | 0 | THRUST <0|10..250> | SPEED <0|5..70> | STATUS")
            return
        if c == "POLSIM" and len(p) >= 4:
            try:
                self._pol_demo_tilt = (float(p[1]), float(p[2]), p[3] == "1")
            except ValueError:
                pass
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
            if c == "CAL":     # a fresh gyro offset, a little off the last one
                self.goff = [round(v + random.gauss(0.0, 0.02), 3) for v in self.goff]
            if c == "FLIGHTARM" and (len(p) < 2 or p[1] == "0") and self.hil:
                self._hil_leave()
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

    def _hil_h_line(self, t: float) -> str:
        with self._hil_lock:
            out = dict(self._hil_out)
            pos = list(self._hil_pos)
            vel = list(self._hil_vel)
            q = list(self._hil_q)
            st = self.state
            fl = 0x12 | (1 if 2 <= st <= 6 else 0) | (0 if self.hil_pyro else hs.MST_PYRO_INHIBIT)
        us = self._hil_servo_us(out)
        tvc = out["tvc_deg"]
        fin = out["fin_deg"]
        d = {
            "t": int(t * 1000), "st": st, "ph": out["nav_phase"], "fl": fl,
            "ev": out["nav_events"], "pa": out["pyro_active"], "pf": out["pyro_fired"],
            "cov": out["cover_open"],
            "p": [round(pos[0], 2), round(pos[1], 2), round(pos[2], 2)],
            "v": [round(vel[0], 2), round(vel[1], 2), round(vel[2], 2)],
            "q": [round(q[0], 4), round(q[1], 4), round(q[2], 4), round(q[3], 4)],
            "tvc": [round(tvc[0], 2), round(tvc[1], 2)],
            "cl": round(out["clamp"], 2),
            "fin": [round(fin[0], 1), round(fin[1], 1), round(fin[2], 1), round(fin[3], 1)],
            "afl": out["act_flags"], "us": us,
            "hign": round(out["h_ign"], 1), "thr": round(out["thr_est"], 1),
            "k": round(out["k_plan"], 2), "tign": round(out["t_to_ign"], 2),
            "ttd": round(out["t_to_td"], 2),
            "tc": round(out["tilt_cmd"], 3), "te": round(out["tilt_err"], 3),
        }
        return "H " + json.dumps(d, separators=(",", ":"))

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
        if self.hil and now >= self.next_h:
            self.next_h = now + 0.02                     # 50 Hz H snapshots, matching Master_MCU.ino
            return self._hil_h_line(now - self.t0)
        if self.raw and now >= self.next_raw:
            self.next_raw = now + 0.04
            return self._raw(now - self.t0)
        if now >= self.next_uwb:
            self.next_uwb = now + 0.5
            self.vib_part = 0
            return self._uwb_diag(now - self.t0)
        if getattr(self, "vib_part", 2) < 2:
            self.vib_part += 1
            return self._vib_demo(self.vib_part - 1, now - self.t0)
        if now >= self.next_diag:
            self.next_diag = now + 0.02
            self.which = (self.which + 1) % 5
            if self.which:
                return self._diag(self.which, now - self.t0)
        return super().readline(timeout)

    @staticmethod
    def _uwb_diag(t: float) -> str:
        """Demo 2 Hz U line: 4 anchors around a 20 m pad square."""
        anchors = [
            [1, int(round((28.28 + 0.03 * math.sin(t)) * 100)), 22, -68, 3],
            [2, int(round((28.31 + 0.03 * math.cos(t * 0.9)) * 100)), 44, -71, 3],
            [3, int(round((28.25 + 0.02 * math.sin(t * 1.1)) * 100)), 68, -74, 3],
            [4, int(round((28.29 + 0.02 * math.cos(t * 0.8)) * 100)), 92, -79, 3],
        ]
        d = {"t": int(t * 1000), "ok": 1, "fl": 3, "err": 0, "rej": 0, "na": 4, "a": anchors}
        return "U " + json.dumps(d, separators=(",", ":"))

    @staticmethod
    def _vib_demo(part: int, t: float) -> str:
        """Demo V line: a 2 kHz IMU on the pad with a fan at 87 Hz and a slow sway."""
        import struct

        def code(rms, ch):
            off = 100.0 if ch <= 1 else 70.0
            return max(1, min(255, int(round(2 * (20 * math.log10(rms) + off)))))
        chs = (2 * part, 2 * part + 1)
        tones = {0: [(87.0, 0.012), (174.1, 0.004), (311.5, 0.002)],
                 1: [(87.0, 0.006), (23.4, 0.003), (1.3, 0.001)],
                 2: [(1.31, 0.4 + 0.1 * math.sin(t)), (87.0, 0.05), (23.4, 0.03)],
                 3: [(4.2, 0.08), (87.0, 0.02), (140.0, 0.01)]}
        floor = [0.0005, 0.0006, 0.02, 0.015]
        peak_f = [int(f * 100) for ch in chs for f, _ in tones[ch]]
        bands = [code(floor[ch] * (1.5 if b > 20 else 1.0), ch) for ch in chs for b in range(31)]
        peak_a = [code(a, ch) for ch in chs for _, a in tones[ch]]
        rms = [code(math.sqrt(sum(a * a for _, a in tones[ch]) + (3 * floor[ch]) ** 2), ch) for ch in chs]
        raw = struct.pack(VIB_FMT, part, int(t / 0.512) & 255, 3, 8, 20000, *peak_f, *bands, *peak_a, *rms)
        return f"V {int(t * 1000)} {raw.hex()}"

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
             "p": round(97234.0 + n(2.0), 1), "temp": 24.6, "src": 1, "mf": 1, "hz": 238,
             "goff": self.goff}
        return "R " + json.dumps(d, separators=(",", ":"))

    @staticmethod
    def _demo_bat(t: float) -> list:
        """[status, soc %, minutes to the limit, open-circuit V, window s, air C, capacity mAh, mean A]"""
        soc = max(0.0, 62.0 - 0.02 * t)
        i = 0.42
        left = max(0.0, soc - 20.0) * 0.01 * 1980.0
        return [1 if soc > 20.0 else 3, round(soc, 1), round(60.0 * left / 1000.0 / i, 1) if soc > 20.0 else 0.0,
                round(7.78 - 0.0003 * t, 2), int(min(300, t)), 21.5, 1980, i, 1.0, 0.0, 0]

    def _diag(self, which: int, t: float) -> str:
        crc = self.crc()
        now = time.monotonic()
        with self._hil_lock:
            pos_hil = self._hil_pos_fresh(now)
            hil_on = self.hil
            hil_pyro = self.hil_pyro
            st = self.state
            out = dict(self._hil_out)
            h_pos = list(self._hil_pos)
            h_vel = list(self._hil_vel)
            h_acc = list(self._hil_acc)
            h_q = list(self._hil_q)
            h_w = list(self._hil_w)
            h_g = self._hil_g
            h_lid = self._hil_lidar_m
            h_lat = self._hil_lat_1e7
            h_lon = self._hil_lon_1e7
            h_alt = self._hil_alt_msl
            h_tmp = self._hil_temp
            h_p0 = self._hil_p0

        if pos_hil:
            qw, qx, qy, qz = h_q
            R = hs.rot_from_quat(h_q)
            u = [round(R[i][1], 3) for i in range(3)]
            tilt = math.acos(max(-1.0, min(1.0, abs(R[2][1]))))
            az = math.atan2(R[1][1], R[0][1])
        elif self.polarity and self._pol_demo_tilt is not None:
            tx_deg, ty_deg, nose_down = self._pol_demo_tilt
            tx, ty = math.radians(tx_deg), math.radians(ty_deg)
            bz_sign = 1.0 if nose_down else -1.0
            bx = math.sin(tx)
            by = math.sin(ty)
            bz2 = max(0.01, 1.0 - min(0.99, bx * bx + by * by))
            b_vec = hs._unit((bx, by, bz_sign * math.sqrt(bz2)))
            g_vec = (1.0, 0.0, 0.0)
            R = hs.rot_from_bg(b_vec, g_vec)
            qw, qx, qy, qz = hs.quat_from_rot(R)
            u = [round(R[i][1], 3) for i in range(3)]
            tilt = math.acos(max(-1.0, min(1.0, abs(R[2][1]))))
            az = math.atan2(R[1][1], R[0][1])
        else:
            tilt, az = self._sim_tilt(t)
            qw, qx, qy, qz = pad_quaternion(tilt, az)
            # thrust axis u = q applied to board +Y: down on the pad
            R = [[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
                 [2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx)],
                 [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy)]]
            u = [round(R[i][1], 3) for i in range(3)]

        if which == 1:
            fl = 0x12 | (8 if self.test else 0)
            if hil_on:
                fl |= (1 if 2 <= st <= 6 else 0) | (0 if hil_pyro else hs.MST_PYRO_INHIBIT)
            d = {"t": int(t * 1000), "fw": 333, "gv": 220, "st": st, "fl": fl,
                 "tl": 0, "vbat": round(7.9 - 0.002 * (t % 100), 2), "i": 0.42, "rbf": 1, "bw": 0, "post": 1,
                 "servo": 1, "flash": 1, "lora": 1, "log": 18432, "logdrop": 0, "tlm": [int(t * 10), 0],
                 "pos": [int(t * 260), 0, 1], "nav": [int(t * 220), 0, 1], "pcrc": crc, "psync": 1,
                 "peep": 1, "cover": out["cover_open"] if hil_on else 0, "hl": 0xEF, "usbdrop": 0,
                 "bat": self._demo_bat(t), "su": [1, 0], "hil": (2 if hil_pyro else 1) if hil_on else 0,
                 "pol": 1 if self.polarity else 0}
            return "M " + json.dumps(d, separators=(",", ":"))
        if which == 2:
            if pos_hil:
                d = {"p": [round(h_pos[0], 2), round(h_pos[1], 2), round(h_pos[2], 2)],
                     "v": [round(h_vel[0], 2), round(h_vel[1], 2), round(h_vel[2], 2)],
                     "a": [round(h_acc[0], 2), round(h_acc[1], 2), round(h_acc[2], 2)],
                     "q": [round(qw, 4), round(qx, 4), round(qy, 4), round(qz, 4)],
                     "u": u, "w": [round(h_w[0], 3), round(h_w[1], 3), round(h_w[2], 3)],
                     "g": round(h_g, 2), "fl": 0x3CF7, "ql": 240, "ok": 1}
            else:
                d = {"p": [round(0.05 * math.sin(t * 0.2), 2), round(0.04 * math.cos(t * 0.17), 2),
                           round(0.03 * math.sin(t), 2)],
                     "v": [0.0, 0.0, 0.0], "a": [0.0, 0.0, 0.0],
                     "q": [round(qw, 4), round(qx, 4), round(qy, 4), round(qz, 4)],
                     "u": u, "w": [round(0.01 * math.sin(t * 3), 3), 0.0, 0.0],
                     "g": 1.0, "fl": 0x3CF7, "ql": 230, "ok": 1}
            return "N " + json.dumps(d, separators=(",", ":"))
        if which == 3:
            d = {"ok": 1, "fw": 333, "pcrc": crc, "pseen": len(self.params),
                 "rpy": [0.0, round(90 - math.degrees(tilt), 1), round(math.degrees(az) % 360, 1)],
                 "lat": h_lat if pos_hil else 497985411,
                 "lon": h_lon if pos_hil else 166912772,
                 "galt": round(h_alt, 1) if pos_hil else 412.3, "sats": 12, "fix": 3, "hdop": 9,
                 "temp": round(h_tmp, 1) if pos_hil else 21.4, "sig": [0.21, 0.05], "uwb": [4, 15, 3, 0],
                 "uwbs": [0.14, 0.08], "trim": 0.21,
                 "lidar": [round(h_lid, 2), 40] if pos_hil else [round(1.2 + 0.01 * math.sin(t), 2), 40],
                 "ins": 254 if pos_hil else 3, "att": 2 if pos_hil else 1,
                 "hz": 100 if pos_hil else 238, "sens": 0x17F, "ovf": 0, "cal": 1,
                 "rx": [int(t * 10), 0], "pp": round(h_p0) if pos_hil else 97230}
            return "P " + json.dumps(d, separators=(",", ":"))
        if hil_on:
            tvc = out["tvc_deg"]
            fin = out["fin_deg"]
            d = {"ok": 1, "fw": 333, "pcrc": crc, "pseen": len(self.params), "ph": out["nav_phase"],
                 "k": round(out["k_plan"], 2), "thr": round(out["thr_est"], 1),
                 "hign": round(out["h_ign"], 1), "tc": round(out["tilt_cmd"], 3),
                 "te": round(out["tilt_err"], 3), "roll": 0.0,
                 "tign": round(out["t_to_ign"], 2), "ttd": round(out["t_to_td"], 2),
                 "ts": 1.0, "fg": 1.0, "apo": round(self._hil_guid.h_max, 1) if self._hil_guid else 0.0,
                 "res": 0.0, "rx": [int(t * 240), 0], "plan": int(t * 200),
                 "loop": 64, "ovr": 0, "cont": 0x07, "fired": out["pyro_fired"],
                 "act": out["pyro_active"], "nfl": 0x2F, "ev": out["nav_events"],
                 "tvc": [round(tvc[0], 2), round(tvc[1], 2)], "cl": round(out["clamp"], 2),
                 "fin": [round(fin[0], 1), round(fin[1], 1), round(fin[2], 1), round(fin[3], 1)],
                 "afl": out["act_flags"], "drag": 1.0, "blo": 0.0, "bhi": 0.0}
        elif self.polarity:
            b_vec = (R[0][1], R[1][1], R[2][1])
            g_vec = (R[0][0], R[1][0], R[2][0])
            pol = hs.polarity_step(b_vec, g_vec, (0.0, 0.0, 0.0),
                                   uz_hint=self._pol_uz,
                                   ref_thrust_n=self._pol_thrust_n,
                                   ref_speed_ms=self._pol_speed_ms)
            self._pol_uz = pol["uz"]
            tvc = pol["tvc_deg"]
            fin = pol["fin_deg"]
            d = {"ok": 1, "fw": 333, "pcrc": crc, "pseen": len(self.params), "ph": 0, "k": 1.0,
                 "thr": round(pol["ref_thrust_n"], 1),
                 "hign": 0.0, "tc": 0.0, "te": round(pol["tilt_err"], 3), "roll": round(pol["roll_rate"], 2),
                 "tign": -1.0, "ttd": -1.0, "ts": 1.0, "fg": 1.0, "apo": 0.0, "res": 0.0,
                 "rx": [int(t * 240), 0], "plan": int(t * 200), "loop": 64, "ovr": 0, "cont": 0x07,
                 "fired": 0, "act": 0, "nfl": 0x2F, "ev": 0,
                 "tvc": [round(tvc[0], 2), round(tvc[1], 2)], "cl": 1.0,
                 "fin": [round(fin[0], 1), round(fin[1], 1), round(fin[2], 1), round(fin[3], 1)],
                 "afl": 3, "drag": 1.0, "blo": 0.0, "bhi": 0.0}
        else:
            d = {"ok": 1, "fw": 333, "pcrc": crc, "pseen": len(self.params), "ph": 0, "k": 1.0, "thr": 0.0,
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
        self.d: dict[str, dict] = {"M": {}, "N": {}, "P": {}, "G": {}, "R": {}, "U": {}}
        self.vib: dict[int, dict] = {}            # channel -> newest V-line decode
        self.vib_time = 0.0
        self.d_time: dict[str, float] = {"M": 0.0, "N": 0.0, "P": 0.0, "G": 0.0, "R": 0.0, "U": 0.0}
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
        # The console's body is laid out with a QGridLayout (no insertWidget),
        # so the polarity card goes into a new wrapper above the whole body.
        act_body = act_scroll.takeWidget()
        act_wrap = QtWidgets.QWidget()
        act_v = QtWidgets.QVBoxLayout(act_wrap)
        act_v.setContentsMargins(20, 16, 20, 0)
        act_v.setSpacing(0)
        act_v.addWidget(self._build_polarity_card())
        if act_body is not None:
            act_v.addWidget(act_body, 1)
        act_scroll.setWidget(act_wrap)
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
        self.hil_tab = HilTab(self)
        self.tabs.addTab(self._scroll(self.hil_tab), "HIL simulace")
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
        self.chip_hil = chip("")
        self.chip_hil.setVisible(False)
        self.chip_pol = chip("")
        self.chip_pol.setVisible(False)
        for c_ in (self.chip_state, self.chip_fw, self.chip_sync, self.chip_err, self.chip_hil, self.chip_pol):
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

    # ------------------------------------------------------------ polarity test card
    def _build_polarity_card(self) -> QtWidgets.QWidget:
        c = Card("Polarita", "Test směru TVC a křidélek (polarita)")
        cl = c.lay
        desc = QtWidgets.QLabel(
            "Čte reálný náklon z IMU Position MCU. TVC se naklání jako při přistání na 0° náklon, "
            "křidélka jako při startu na 0° náklon (+ tlumení rollu). Pyro je po celou dobu VYPNUTO."
        )
        desc.setWordWrap(True)
        desc.setStyleSheet(f"color: {ag.C['muted']};")
        cl.addWidget(desc)
        top = QtWidgets.QHBoxLayout()
        top.setSpacing(10)
        self.pol_btn = button(
            "Spustit test směru TVC + křidélek", "primary",
            "Zapne/vypne režim POLARITY (vyžaduje stav IDLE, mimo TEST/HIL). "
            "Nakloň raketu rukou a ověř, že TVC i křidélka tlačí raketu zpět do svislé polohy 0°.",
        )
        self.pol_btn.setCheckable(True)
        self.pol_btn.clicked.connect(self._toggle_polarity)
        self.pol_chip = chip("NEAKTIVNÍ")
        self.pol_orient_chip = chip("Orientace: —")
        top.addWidget(self.pol_btn)
        top.addWidget(self.pol_chip)
        top.addWidget(self.pol_orient_chip)
        top.addStretch(1)
        cl.addLayout(top)

        self._pol_default_thrust = hs.pol_ref_thrust()
        self._pol_thrust_custom = False
        self._pol_thr_timer = QtCore.QTimer(self)
        self._pol_thr_timer.setSingleShot(True)
        self._pol_thr_timer.setInterval(60)
        self._pol_thr_timer.timeout.connect(self._send_pol_thrust)

        thr_row = QtWidgets.QHBoxLayout()
        thr_row.setSpacing(10)
        thr_lbl_title = QtWidgets.QLabel("Referenční tah TVC:")
        thr_lbl_title.setToolTip(
            "Tah použitý pro výpočet výchylky TVC při testu polarity (10–250 N). "
            "Výchozí hodnota je průměr průměrného a mediánového tahu z referenční křivky."
        )
        self.pol_thr_slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.pol_thr_slider.setRange(10, 250)
        self.pol_thr_slider.setSingleStep(2)
        self.pol_thr_slider.setPageStep(10)
        self.pol_thr_slider.setValue(int(round(self._pol_default_thrust)))
        self.pol_thr_slider.setFixedWidth(220)
        self.pol_thr_lbl = QtWidgets.QLabel(f"{self._pol_default_thrust:.1f} N (výchozí)")
        self.pol_thr_lbl.setMinimumWidth(130)
        self.pol_thr_reset = button(
            "Výchozí", "",
            f"Vrátí referenční tah na výchozí hodnotu ({self._pol_default_thrust:.1f} N).",
        )
        self.pol_thr_reset.clicked.connect(self._reset_pol_thrust)
        self.pol_thr_slider.valueChanged.connect(self._on_pol_thrust_changed)
        thr_row.addWidget(thr_lbl_title)
        thr_row.addWidget(self.pol_thr_slider)
        thr_row.addWidget(self.pol_thr_lbl)
        thr_row.addWidget(self.pol_thr_reset)
        thr_row.addStretch(1)
        cl.addLayout(thr_row)

        self._pol_default_speed = 40
        self._pol_speed_custom = False
        self._pol_spd_timer = QtCore.QTimer(self)
        self._pol_spd_timer.setSingleShot(True)
        self._pol_spd_timer.setInterval(60)
        self._pol_spd_timer.timeout.connect(self._send_pol_speed)

        spd_row = QtWidgets.QHBoxLayout()
        spd_row.setSpacing(10)
        spd_lbl_title = QtWidgets.QLabel("Rychlost (křidélka):")
        spd_lbl_title.setMinimumWidth(thr_lbl_title.sizeHint().width())
        spd_lbl_title.setToolTip(
            "Rychlost proudění pro výpočet výchylky křidélek při testu polarity (5–70 m/s). "
            "Vyšší rychlost = vyšší dynamický tlak a menší potřebná výchylka. Výchozí: 40 m/s."
        )
        self.pol_spd_slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.pol_spd_slider.setRange(5, 70)
        self.pol_spd_slider.setSingleStep(1)
        self.pol_spd_slider.setPageStep(5)
        self.pol_spd_slider.setValue(self._pol_default_speed)
        self.pol_spd_slider.setFixedWidth(220)
        self.pol_spd_lbl = QtWidgets.QLabel(f"{self._pol_default_speed} m/s (výchozí)")
        self.pol_spd_lbl.setMinimumWidth(130)
        self.pol_spd_reset = button(
            "Výchozí", "",
            f"Vrátí rychlost rakety pro křidélka na výchozí hodnotu ({self._pol_default_speed} m/s).",
        )
        self.pol_spd_reset.clicked.connect(self._reset_pol_speed)
        self.pol_spd_slider.valueChanged.connect(self._on_pol_speed_changed)
        spd_row.addWidget(spd_lbl_title)
        spd_row.addWidget(self.pol_spd_slider)
        spd_row.addWidget(self.pol_spd_lbl)
        spd_row.addWidget(self.pol_spd_reset)
        spd_row.addStretch(1)
        cl.addLayout(spd_row)

        self.pol_kv = KV([
            "Reálný náklon",
            "Úhlová rychlost roll",
            "TVC (přistání → 0°)",
            "Křidélka F1–F4 (start → 0°)",
        ])
        cl.addWidget(self.pol_kv)

        hint = QtWidgets.QLabel(
            "Ověření rukou: nakloň raketu — tryska TVC i křidélka se musí vychýlit tak, aby vracely "
            "raketu do svislé polohy 0°; při otočení kolem podélné osy musí všechna 4 křidélka brzdit rotaci."
        )
        hint.setWordWrap(True)
        hint.setStyleSheet(f"color: {ag.C['muted']}; font-size: 11px;")
        cl.addWidget(hint)

        # Demo-only tilt controls (hidden on real hardware)
        self.pol_demo_box = QtWidgets.QWidget()
        dl = QtWidgets.QHBoxLayout(self.pol_demo_box)
        dl.setContentsMargins(0, 4, 0, 0)
        dl.setSpacing(10)
        dl.addWidget(QtWidgets.QLabel("DEMO náklon X:"))
        self.pol_sx = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.pol_sx.setRange(-25, 25)
        self.pol_sx.setValue(0)
        self.pol_sx.setFixedWidth(130)
        self.pol_sx_lbl = QtWidgets.QLabel("0°")
        self.pol_sx_lbl.setFixedWidth(36)
        dl.addWidget(self.pol_sx)
        dl.addWidget(self.pol_sx_lbl)
        dl.addWidget(QtWidgets.QLabel("Y:"))
        self.pol_sy = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.pol_sy.setRange(-25, 25)
        self.pol_sy.setValue(0)
        self.pol_sy.setFixedWidth(130)
        self.pol_sy_lbl = QtWidgets.QLabel("0°")
        self.pol_sy_lbl.setFixedWidth(36)
        dl.addWidget(self.pol_sy)
        dl.addWidget(self.pol_sy_lbl)
        self.pol_down_chk = QtWidgets.QCheckBox("Špičkou dolů (přistání)")
        dl.addWidget(self.pol_down_chk)
        dl.addStretch(1)
        self.pol_sx.valueChanged.connect(self._send_pol_demo)
        self.pol_sy.valueChanged.connect(self._send_pol_demo)
        self.pol_down_chk.toggled.connect(self._send_pol_demo)
        self.pol_demo_box.setVisible(False)
        cl.addWidget(self.pol_demo_box)
        return c

    def _on_pol_thrust_changed(self, val: int) -> None:
        self._pol_thrust_custom = True
        self.pol_thr_lbl.setText(f"{val} N")
        self._pol_thr_timer.start()

    def _reset_pol_thrust(self) -> None:
        self._pol_thrust_custom = False
        self.pol_thr_slider.blockSignals(True)
        self.pol_thr_slider.setValue(int(round(self._pol_default_thrust)))
        self.pol_thr_slider.blockSignals(False)
        self.pol_thr_lbl.setText(f"{self._pol_default_thrust:.1f} N (výchozí)")
        self._send_pol_thrust()

    def _send_pol_thrust(self) -> None:
        if self.link.is_open and self.flight_fw:
            arg = self.pol_thr_slider.value() if self._pol_thrust_custom else 0
            self.link.send(f"POLARITY THRUST {arg}")

    def _on_pol_speed_changed(self, val: int) -> None:
        self._pol_speed_custom = True
        self.pol_spd_lbl.setText(f"{val} m/s")
        self._pol_spd_timer.start()

    def _reset_pol_speed(self) -> None:
        self._pol_speed_custom = False
        self.pol_spd_slider.blockSignals(True)
        self.pol_spd_slider.setValue(self._pol_default_speed)
        self.pol_spd_slider.blockSignals(False)
        self.pol_spd_lbl.setText(f"{self._pol_default_speed} m/s (výchozí)")
        self._send_pol_speed()

    def _send_pol_speed(self) -> None:
        if self.link.is_open and self.flight_fw:
            arg = self.pol_spd_slider.value() if self._pol_speed_custom else 0
            self.link.send(f"POLARITY SPEED {arg}")

    def _toggle_polarity(self) -> None:
        on = self.pol_btn.isChecked()
        if on:
            thr = self.pol_thr_slider.value() if self._pol_thrust_custom else 0
            spd = self.pol_spd_slider.value() if self._pol_speed_custom else 0
            self._cmd(f"POLARITY 1 {thr} {spd}")
            if self.port_box.currentData() == "DEMO":
                self._send_pol_demo()
        else:
            self._cmd("POLARITY 0")

    def _send_pol_demo(self) -> None:
        tx = self.pol_sx.value()
        ty = self.pol_sy.value()
        down = 1 if self.pol_down_chk.isChecked() else 0
        self.pol_sx_lbl.setText(f"{tx:+d}°")
        self.pol_sy_lbl.setText(f"{ty:+d}°")
        if self.link.is_open and self.port_box.currentData() == "DEMO":
            self.link.send(f"POLSIM {tx} {ty} {down}")

    def _refresh_polarity(self, M: dict, N: dict, G: dict, hil_mode: int) -> None:
        pol_on = bool(M.get("pol", 0)) if M else False
        st = M.get("st", 0) if M else 0
        can_run = (self.link.is_open and self.flight_fw and hil_mode == 0
                   and not self.test_on and st == 1)
        self.pol_btn.blockSignals(True)
        self.pol_btn.setChecked(pol_on)
        self.pol_btn.setText("Zastavit test směru" if pol_on else "Spustit test směru TVC + křidélek")
        self.pol_btn.setEnabled(can_run or pol_on)
        self.pol_btn.blockSignals(False)

        if pol_on:
            set_chip(self.pol_chip, "AKTIVNÍ · pyro VYPNUTO", "warn")
        else:
            set_chip(self.pol_chip, "NEAKTIVNÍ", "")

        is_demo = self.link.is_open and self.port_box.currentData() == "DEMO"
        self.pol_demo_box.setVisible(is_demo and pol_on)

        u = N.get("u") if N else None
        if u and len(u) == 3:
            uz = float(u[2])
            if uz >= 0.0:
                set_chip(self.pol_orient_chip, "Špičkou DOLŮ (přistání / TVC)", "ok")
            else:
                set_chip(self.pol_orient_chip, "Špičkou NAHORU (start / křidélka)", "ok")
        else:
            set_chip(self.pol_orient_chip, "Orientace: —", "")

        if N and N.get("q"):
            a = attitude_summary(*N["q"])
            self.pol_kv.set(
                "Reálný náklon",
                f"{a['tilt']:.1f}°  (azimut {a['tilt_az']:.0f}°, {'špička nahoru' if a['nose_up'] else 'špička dolů'})",
                GOOD if a["tilt"] < 4 else (WARN if a["tilt"] < 15 else BAD),
            )
        else:
            self.pol_kv.set("Reálný náklon", "—", DIM)

        if G and G.get("ok"):
            rr = math.degrees(float(G.get("roll", 0.0)))
            self.pol_kv.set("Úhlová rychlost roll", f"{rr:+.1f} °/s")
            tvc = G.get("tvc", [0.0, 0.0])
            fin = G.get("fin", [0.0, 0.0, 0.0, 0.0])
            self.pol_kv.set("TVC (přistání → 0°)", f"X: {tvc[0]:+.2f}°   Y: {tvc[1]:+.2f}°",
                            WARN if pol_on and (abs(tvc[0]) > 0.2 or abs(tvc[1]) > 0.2) else None)
            self.pol_kv.set(
                "Křidélka F1–F4 (start → 0°)",
                f"F1:{fin[0]:+.1f}°  F2:{fin[1]:+.1f}°  F3:{fin[2]:+.1f}°  F4:{fin[3]:+.1f}°",
                WARN if pol_on and any(abs(v) > 0.2 for v in fin) else None,
            )
        else:
            self.pol_kv.set("Úhlová rychlost roll", "—", DIM)
            self.pol_kv.set("TVC (přistání → 0°)", "—", DIM)
            self.pol_kv.set("Křidélka F1–F4 (start → 0°)", "—", DIM)

    def toggle_record(self) -> None:
        if self.rec_file:
            self.rec_file.close()
            self.rec_file = self.rec_writer = None
            self.rec_btn.setText("Nahrávat")
            return
        name = time.strftime("daidalos_diag_%Y%m%d_%H%M%S.csv")
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Nahrávat do", ag.user_path(name), "CSV (*.csv)")
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
                  ["Online", "Firmware", "POST", "Stav", "Baterie", "Nabití baterie", "Výdrž do limitu",
                   "Kontrola po startu", "Proud", "RBF", "Breakwire", "PCA9685", "Flash log", "LoRa", "Spoj Position", "Spoj Navigation", "Aerokryt",
                   "USB zahozeno"]),
            "P": ("Position MCU", "XIAO RP2350 · senzory, odhad polohy",
                  ["Online", "Firmware", "Parametry", "Senzory", "Lidar kvalita", "Orientace", "Fúze Hz",
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
                           "UWB tag", "Kotva 1", "Kotva 2", "Kotva 3", "Kotva 4",
                           "Baro trim", "Pad tlak", "Lidar", "Teplota", "INS (INSLIB)"])
        se.lay.addWidget(self.sens_kv)
        self.sens_flags = Flags(SENS_BITS)
        se.lay.addWidget(self.sens_flags)
        g.addWidget(se, 1, 1)

        vc = Card("Vibrace", "0,5–500 Hz z IMU (2 kHz) · RMS celkem a 3 nejsilnější tóny · zapisuje se do logu")
        self.vib_kv = KV(VIB_CH_NAMES + ["Analyzátor"])
        vc.lay.addWidget(self.vib_kv)
        g.addWidget(vc, 2, 0, 1, 2)
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

    # ---------------------------------------------- servo table <-> params
    def _config_extra(self, card) -> None:
        b = button("Zapsat do rakety", "primary",
                   "Kanály, limity, trimy, reverz (znaménko), µs/° a polohy aerokrytu jako parametry "
                   "Masteru: PARAM SET + PARAM SAVE (trvale ve flash)")
        b.clicked.connect(self.servo_params_write)
        card.head.addWidget(b)
        b = button("Načíst z rakety", "", "Vyplní tabulku podle parametrů Masteru (PARAM LIST)")
        b.clicked.connect(lambda: self._cmd("PARAM LIST"))
        card.head.addWidget(b)
        self.srv_sync_lbl = label("", "muted")
        self.srv_sync_lbl.setWordWrap(True)
        card.lay.addWidget(self.srv_sync_lbl)

    def _pval(self, key: str) -> float | None:
        r = next((p for p in self.params if p["key"] == key), None)
        return None if r is None else self.param_edits.get(r["id"], r["val"])

    def _servo_wanted(self) -> dict[str, float]:
        """Parameter values the servo table describes."""
        out: dict[str, float] = {}
        for s in self.cfg.servos:
            for f, key in SERVO_PARAMS.get(s.key, {}).items():
                v = getattr(s, f)
                if v is None:
                    continue
                out[key] = (-1.0 if v else 1.0) if f == "reverse" else float(v)
        return out

    def _servo_diff(self) -> dict[str, float]:
        diff = {}
        for k, v in self._servo_wanted().items():
            cur = self._pval(k)
            if cur is not None and abs(cur - v) > 1e-3:
                diff[k] = v
        return diff

    def _board_has_servo_map(self) -> bool:
        return any(p["key"] == "srv.ch_tvc_x" for p in self.params)

    def servo_params_from_board(self) -> None:
        """After PARAM LIST: the flight firmware's servo setup into the table."""
        if not self._board_has_servo_map():
            self.srv_sync_lbl.setText("Firmware rakety < 3.28: kanály a limity serv v ní nejsou, tabulka je jen v PC.")
            return
        for s in self.cfg.servos:
            for f, key in SERVO_PARAMS.get(s.key, {}).items():
                v = self._pval(key)
                if v is None:
                    continue
                if f == "reverse":
                    s.reverse = v < 0
                elif f == "us_per_deg":
                    s.us_per_deg = float(v)
                else:
                    setattr(s, f, int(round(v)))
            if s.min_us >= s.max_us:                     # limit switched off on the board
                s.min_us, s.max_us = ag.ABS_US_MIN, ag.ABS_US_MAX
            ag.validate_servo(s)
        self._fill_config_table()
        for s in self.cfg.servos:
            ax = self.axes.get(s.key)
            if ax:
                ax.apply_cfg()
                ax.set_us(ax.value())
        self._sync_tvc_pad()
        self._update_cover_state()
        self._check_conflicts()
        if self.test_on:
            self.send_limits()
        self._servo_sync_note()

    def _servo_sync_note(self) -> None:
        if not hasattr(self, "srv_sync_lbl") or not self._board_has_servo_map():
            return
        d = self._servo_diff()
        self.srv_sync_lbl.setText(
            f"⚠ {len(d)} hodnot v tabulce se liší od rakety — „Zapsat do rakety“ je uloží trvale."
            if d else "Tabulka odpovídá parametrům rakety (kanály, limity, trimy, znaménka).")
        self.srv_sync_lbl.setStyleSheet(f"color: {ag.C['accent'] if d else ag.C['positive']};")

    def _cfg_changed(self, s, send_pos: bool) -> None:
        super()._cfg_changed(s, send_pos)
        self._servo_sync_note()

    def servo_params_write(self) -> None:
        if not self.link.is_open:
            return
        if not self._board_has_servo_map():
            QtWidgets.QMessageBox.warning(self, APP_NAME, "Nejdřív „Načíst z rakety“ (a firmware ≥ 3.28).")
            return
        chans = [s.ch for s in self.cfg.servos if s.key in SERVO_PARAMS]
        if len(set(chans)) != len(chans):
            QtWidgets.QMessageBox.warning(self, APP_NAME, "Dvě letová serva mají stejný kanál - opravte tabulku.")
            return
        d = self._servo_diff()
        if not d:
            self.log("# serva: raketa už má tyto hodnoty")
            return
        if any(k in d for k in ("srv.tvc_x_sign", "srv.tvc_y_sign")):
            r = QtWidgets.QMessageBox.warning(
                self, APP_NAME, "Měníte ZNAMÉNKO (reverz) serva TVC. Špatné znaménko = kladná zpětná vazba a "
                "pád do 1 s. Po změně VŽDY udělejte zkoušku znamének (manuál, kap. Předletové testy).",
                QtWidgets.QMessageBox.StandardButton.Ok | QtWidgets.QMessageBox.StandardButton.Cancel)
            if r != QtWidgets.QMessageBox.StandardButton.Ok:
                return
        for k, v in d.items():
            self.link.send(f"PARAM SET {k} {v:.6g}")
        self.link.send("PARAM SAVE")
        self.log(f"> serva: PARAM SET × {len(d)} + PARAM SAVE")

    # ------------------------------------------------------------ params
    # Columns of the parameter tree.
    PC_KEY, PC_VAL, PC_UNIT, PC_DEF, PC_RANGE, PC_OWNER, PC_HELP = range(7)

    def _build_params(self) -> QtWidgets.QWidget:
        w = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(w)
        lay.setContentsMargins(20, 16, 20, 20)
        lay.setSpacing(10)
        note = label(
            "Tabulka parametrů žije ve flash W25Q128 Masteru a ten ji na rampě posílá Position i Navigation MCU. "
            "Měnit lze jen v IDLE nebo TEST. Postup: Načíst z desky → klepnout na hodnotu a upravit (žlutě) → "
            "Odeslat změny → Uložit do flash. Modře = jiná než výchozí. Tabulka zážehu má vlastní záložku. "
            "Zastrčit lze jen, když obě desky drží stejnou tabulku (CRC souhlasí).", "muted")
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
        lay.addLayout(row)

        row = QtWidgets.QHBoxLayout()
        self.filter = QtWidgets.QLineEdit()
        self.filter.setPlaceholderText("hledat: název nebo popis (např. aerokryt, fin., lidar)")
        self.filter.setClearButtonEnabled(True)
        self.filter.textChanged.connect(self._param_filter)
        row.addWidget(self.filter, 1)
        self.param_changed_only = QtWidgets.QCheckBox("Jen změněné")
        self.param_changed_only.setToolTip("Jen hodnoty jiné než výchozí a neodeslané úpravy")
        self.param_changed_only.toggled.connect(self._param_filter)
        row.addWidget(self.param_changed_only)
        for txt, fn in (("Rozbalit vše", lambda: self.ptree.expandAll()),
                        ("Sbalit vše", lambda: self.ptree.collapseAll())):
            b = button(txt)
            b.clicked.connect(fn)
            row.addWidget(b)
        lay.addLayout(row)

        t = QtWidgets.QTreeWidget()
        t.setColumnCount(7)
        t.setHeaderLabels(["parametr", "hodnota", "jednotka", "výchozí", "rozsah", "deska", "co dělá"])
        t.setRootIsDecorated(True)
        t.setUniformRowHeights(True)
        t.setAlternatingRowColors(False)
        t.setWordWrap(False)
        t.setIndentation(14)
        t.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)  # value column only, below
        t.setSelectionMode(QtWidgets.QAbstractItemView.SelectionMode.SingleSelection)
        t.setStyleSheet(
            f"QTreeWidget {{ background: {ag.C['bg_raised']}; border: 1px solid {ag.C['line']}; "
            f"border-radius: 10px; }}"
            f"QTreeWidget::item {{ padding: 3px 2px; }}"
            f"QTreeWidget::item:selected {{ background: {ag.C['accent_tint']}; color: {ag.C['text']}; }}")
        hdr = t.header()
        hdr.setStretchLastSection(True)
        hdr.setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.Interactive)
        t.itemChanged.connect(self._param_item_changed)
        t.itemClicked.connect(self._param_item_clicked)
        t.itemDoubleClicked.connect(self._param_item_clicked)
        self.ptree = t
        self.param_items: dict[int, QtWidgets.QTreeWidgetItem] = {}
        self.param_group_items: list[QtWidgets.QTreeWidgetItem] = []
        lay.addWidget(t, 1)
        self._param_fill()
        return w

    @staticmethod
    def _num(v: float) -> str:
        return f"{v:.6g}"

    def _param_fill(self) -> None:
        t = self.ptree
        expanded = {g.text(0) for g in self.param_group_items if g.isExpanded()} if self.param_group_items else None
        t.blockSignals(True)
        t.clear()
        self.param_items.clear()
        self.param_rows = {p["id"]: i for i, p in enumerate(self.params)}
        owners = {"N": "Navigation", "M": "Master", "P": "Position"}
        groups = PARAM_GROUPS + [PARAM_OTHER]
        heads: list[QtWidgets.QTreeWidgetItem] = []
        bold = QtGui.QFont(t.font())
        bold.setBold(True)
        for name, sub, _ in groups:
            g = QtWidgets.QTreeWidgetItem([name, "", "", "", "", "", sub])
            g.setData(self.PC_HELP, Qt.ItemDataRole.UserRole, sub)
            g.setFlags(Qt.ItemFlag.ItemIsEnabled)
            g.setFont(0, bold)
            g.setForeground(0, QtGui.QColor(ag.C["accent"]))
            g.setForeground(self.PC_HELP, QtGui.QColor(ag.C["subtle"]))
            heads.append(g)
        for p in sorted(self.params, key=lambda q: q["id"]):
            if p["key"].startswith("tbl.hm"):        # the 64 band entries live on their own tab
                continue
            help_ = PARAM_HELP.get(p["key"], "")
            it = QtWidgets.QTreeWidgetItem([
                p["key"], self._num(self.param_edits.get(p["id"], p["val"])), p["unit"],
                self._num(p["def"]), f"{self._num(p['lo'])} … {self._num(p['hi'])}",
                owners.get(p["owner"], p["owner"]), help_])
            it.setData(0, Qt.ItemDataRole.UserRole, p["id"])
            it.setFlags(Qt.ItemFlag.ItemIsEnabled | Qt.ItemFlag.ItemIsSelectable | Qt.ItemFlag.ItemIsEditable)
            tip = f"{p['key']} (id {p['id']})" + (f"\n{help_}" if help_ else "")
            for c_ in range(7):
                it.setToolTip(c_, tip)
            for c_ in (self.PC_UNIT, self.PC_DEF, self.PC_RANGE, self.PC_OWNER, self.PC_HELP):
                it.setForeground(c_, QtGui.QColor(ag.C["muted"] if c_ == self.PC_HELP else ag.C["subtle"]))
            it.setTextAlignment(self.PC_VAL, Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            heads[param_group(p["key"])].addChild(it)
            self.param_items[p["id"]] = it
        self.param_group_items = [g for g in heads if g.childCount()]
        for g in self.param_group_items:
            t.addTopLevelItem(g)
            g.setFirstColumnSpanned(False)
            g.setExpanded(expanded is None or g.text(0) in expanded)
        for pid in self.param_items:
            self._param_color(pid)
        t.blockSignals(False)
        for c_ in range(self.PC_HELP):
            t.resizeColumnToContents(c_)
        t.setColumnWidth(self.PC_VAL, max(t.columnWidth(self.PC_VAL), 90))
        self._param_filter()

    def _param_show(self, pid: int, v: float) -> None:
        it = self.param_items.get(pid)
        if it is None:
            return
        self.ptree.blockSignals(True)
        it.setText(self.PC_VAL, self._num(v))
        self.ptree.blockSignals(False)
        self._param_color(pid)

    def _param_color(self, pid: int) -> None:
        it = self.param_items.get(pid)
        r = self.param_rows.get(pid)
        if it is None or r is None:
            return
        p = self.params[r]
        self.ptree.blockSignals(True)
        if pid in self.param_edits:
            it.setBackground(self.PC_VAL, QtGui.QColor(245, 165, 28, 70))
            it.setForeground(self.PC_VAL, QtGui.QColor(ag.C["accent_soft"]))
        elif abs(p["val"] - p["def"]) > 1e-6 * (1 + abs(p["def"])):
            it.setBackground(self.PC_VAL, QtGui.QColor(74, 163, 255, 45))      # differs from default
            it.setForeground(self.PC_VAL, QtGui.QColor(ag.C["text"]))
        else:
            it.setBackground(self.PC_VAL, QtGui.QBrush())
            it.setForeground(self.PC_VAL, QtGui.QColor(ag.C["text"]))
        self.ptree.blockSignals(False)
        self._param_group_counts()

    def _param_changed(self, p: dict) -> bool:
        return p["id"] in self.param_edits or abs(p["val"] - p["def"]) > 1e-6 * (1 + abs(p["def"]))

    def _param_group_counts(self) -> None:
        """'10 param. · 2 změněné · 1 k odeslání' in each group header."""
        if not getattr(self, "param_group_items", None):
            return
        self.ptree.blockSignals(True)
        for g in self.param_group_items:
            n = g.childCount()
            ch = sum(1 for i in range(n)
                     if self._param_changed(self.params[self.param_rows[g.child(i).data(0, Qt.ItemDataRole.UserRole)]]))
            ed = sum(1 for i in range(n) if g.child(i).data(0, Qt.ItemDataRole.UserRole) in self.param_edits)
            txt = f"{n} param." + (f" · {ch} jiné než výchozí" if ch else "") + (f" · {ed} k odeslání" if ed else "")
            g.setText(self.PC_HELP, txt + "  —  " + (g.data(self.PC_HELP, Qt.ItemDataRole.UserRole) or ""))
            g.setForeground(self.PC_HELP, QtGui.QColor(ag.C["accent_soft"] if ed else ag.C["subtle"]))
        self.ptree.blockSignals(False)

    def _param_item_clicked(self, it: QtWidgets.QTreeWidgetItem, col: int) -> None:
        if it.data(0, Qt.ItemDataRole.UserRole) is not None and col == self.PC_VAL:
            self.ptree.editItem(it, self.PC_VAL)

    def _param_item_changed(self, it: QtWidgets.QTreeWidgetItem, col: int) -> None:
        pid = it.data(0, Qt.ItemDataRole.UserRole)
        if col != self.PC_VAL or pid is None or pid not in self.param_rows:
            return
        p = self.params[self.param_rows[pid]]
        try:
            v = float(it.text(self.PC_VAL).replace(",", ".").strip())
        except ValueError:
            self._param_show(pid, self.param_edits.get(pid, p["val"]))
            return
        v = max(p["lo"], min(p["hi"], v))
        if abs(v - p["val"]) <= 1e-9 * (1 + abs(v)):
            self.param_edits.pop(pid, None)                 # typed back to the board's value
        else:
            self.param_edits[pid] = v
        self._param_show(pid, v)

    def _param_filter(self, *_args) -> None:
        if not hasattr(self, "ptree"):
            return
        txt = self.filter.text().strip().lower()
        only = self.param_changed_only.isChecked()
        for g in self.param_group_items:
            shown = 0
            for i in range(g.childCount()):
                it = g.child(i)
                p = self.params[self.param_rows[it.data(0, Qt.ItemDataRole.UserRole)]]
                hit = (not txt or txt in p["key"].lower() or txt in PARAM_HELP.get(p["key"], "").lower())
                hide = not hit or (only and not self._param_changed(p))
                it.setHidden(hide)
                shown += not hide
            g.setHidden(shown == 0)
            if (txt or only) and shown:
                g.setExpanded(True)

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
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Export", ag.user_path("daidalos_params.json"), "JSON (*.json)")
        if path:
            with open(path, "w", encoding="utf-8") as fh:
                json.dump({p["key"]: self.param_edits.get(p["id"], p["val"]) for p in self.params}, fh, indent=2)

    def param_import(self) -> None:
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Import", ag.user_dir(), "JSON (*.json)")
        if not path:
            return
        with open(path, encoding="utf-8") as fh:
            d = json.load(fh)
        for p in self.params:
            if p["key"] in d and abs(float(d[p["key"]]) - p["val"]) > 1e-9:
                self.param_edits[p["id"]] = max(p["lo"], min(p["hi"], float(d[p["key"]])))
        self._param_fill()
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
        if hasattr(self, "hil_tab"):
            self.hil_tab.on_master_link_state(up)
        if up:
            QtCore.QTimer.singleShot(500, lambda: self.link.is_open and self.link.send("PARAM LIST"))

    def on_line(self, line: str) -> None:
        if self.rec_writer:
            self.rec_writer.writerow([f"{time.time():.3f}", line[:1], line])
        if line.startswith("H {"):
            try:
                hd = json.loads(line[2:])
            except ValueError:
                hd = None
            if hd is not None and hasattr(self, "hil_tab"):
                self.hil_tab.on_h_line(hd)
            return
        if len(line) > 2 and line[0] in "MNPGRU" and line[1] == " " and line[2] == "{":
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
        if line.startswith("V "):
            for c in parse_vib(line) or []:
                self.vib[c["ch"]] = c
                self.vib_time = time.monotonic()
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
            self.servo_params_from_board()
        elif line.startswith("OK PARAM ") and len(line.split()) >= 5 and line.split()[2].isdigit():
            _, _, pid, _key, val = line.split()[:5]
            pid = int(pid)
            if pid in self.param_rows:
                self.params[self.param_rows[pid]]["val"] = float(val)
                self.param_edits.pop(pid, None)
                self._param_show(pid, float(val))
                self._servo_sync_note()
        elif line.startswith("OK TEST 1"):
            self.test_on = True
            # The console's PWM / IDLE / LIM are accepted only in TEST: send them now.
            self.link.send(f"PWM {self.pwm_box.currentData()}")
            self.link.send(f"IDLE {self.idle_spin.value()}")
            self.send_limits()
        elif line.startswith("OK TEST 0"):
            self.test_on = False
        elif line.startswith(("OK HIL", "ERR HIL")):
            if hasattr(self, "hil_tab"):
                self.hil_tab.on_master_hil_reply(line)
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
        limit = 1.5 if k == "U" else 1.0
        return self.link.is_open and time.monotonic() - self.d_time[k] < limit

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

    @staticmethod
    def fmt_minutes(m: float) -> str:
        m = int(round(m))
        return f"{m} min" if m < 120 else f"{m // 60} h {m % 60:02d} min"

    def _refresh_vib(self, pos_alive: bool) -> None:
        fresh = self.link.is_open and time.monotonic() - self.vib_time < 2.0
        if not fresh:
            for n in VIB_CH_NAMES:
                self.vib_kv.set(n, "—", DIM)
            self.vib_kv.set("Analyzátor",
                            "žádná spektra – Position MCU přeložený se starou Voltino_TriSense? (potřeba ≥ 1.7.1, výpis při startu)"
                            if pos_alive and self.flight_fw else "čeká na spektrum (2 Hz)", DIM)
            return
        stat, fs = 0, 0.0
        for ch, n in enumerate(VIB_CH_NAMES):
            c = self.vib.get(ch)
            if not c:
                self.vib_kv.set(n, "—", DIM)
                continue
            stat |= c["stat"]
            fs = c["fs"]
            u = VIB_UNIT[ch]
            tones = " · ".join(f"{f:.1f} Hz {a:.3g}" for f, a in c["tones"]) or "žádný výrazný tón"
            self.vib_kv.set(n, f"RMS {c['rms']:.3g} {u}   |   {tones}")
        notes = []
        if stat & 16:
            notes.append("IMU pod 1,8 kHz – nad 450 Hz nic")
        if stat & 8:
            notes.append("ztracené vzorky")
        self.vib_kv.set("Analyzátor", f"IMU {fs:.0f} Hz" + (" · " + ", ".join(notes) if notes else ""),
                        WARN if notes else GOOD)

    def _refresh_battery(self, kv, M: dict) -> None:
        """Charge and run time to the limit, from the Master's "bat" field
        [status, soc %, minutes, V open-circuit, window s, air C, capacity mAh, mean A, learn, last k, judged stretches].
        The Master recomputes it once a minute from a 5-minute mean."""
        bat = M.get("bat")
        if not bat or len(bat) < 8:
            kv.set("Nabití baterie", "firmware bez odhadu (starší než 3.29)", DIM)
            kv.set("Výdrž do limitu", "—", DIM)
            return
        st, soc, minutes, voc, win, tair, cap, iavg = bat[:8]
        learn = bat[8] if len(bat) > 8 else 1.0
        lim = self._pval("batt.cutoff_pct")
        lim = 20.0 if lim is None else lim
        if st == 0:
            kv.set("Nabití baterie", "bez baterie / čeká na data", DIM)
            kv.set("Výdrž do limitu", "—", DIM)
            return
        extra = f", zdraví se samo snížilo na ×{fmt(learn, 2)}" if learn < 0.995 else ""
        col = BAD if soc <= lim else (WARN if soc <= lim + 15.0 else GOOD)
        kv.set("Nabití baterie",
               f"{soc:.0f} %   (Voc {fmt(voc, 2, 'V')}, {tair:.0f} °C, využitelná kapacita {cap:.0f} mAh{extra})", col)
        if st == 3:
            kv.set("Výdrž do limitu", f"POD LIMITEM {lim:.0f} %", BAD)
        elif st == 2:
            kv.set("Výdrž do limitu", f"> {self.fmt_minutes(minutes)}  (zátěž {fmt(iavg, 2, 'A')} je příliš malá na odhad)", GOOD)
        else:
            # time left ABOVE the limit: orange under an hour, red under 20 minutes
            col = BAD if minutes < 20 else (WARN if minutes < 60 else GOOD)
            kv.set("Výdrž do limitu",
                   f"≈ {self.fmt_minutes(minutes)} do {lim:.0f} %   (při {fmt(iavg, 2, 'A')}, okno {int(win)} z 300 s)", col)

    SU_NAMES = [(0, "POST Masteru"), (1, "serva / PCA9685"), (2, "flash"), (3, "Position MCU mlčí"),
                (4, "Position: senzory / kalibrace gyra"), (5, "Navigation MCU mlčí"), (6, "parametry nesedí")]

    def _refresh_startup(self, kv, M: dict) -> None:
        """The buzzer's start-up verdict, [result, not-ready mask]: 0 still deciding, 1 OK, 2 FAIL, 3 skipped."""
        su = M.get("su")
        if not su or len(su) < 2:
            kv.set("Kontrola po startu", "—", DIM)
            return
        res, mask = int(su[0]), int(su[1])
        if res == 1:
            kv.set("Kontrola po startu", "OK (bzučák: dva stoupavé tóny a dlouhý vysoký)", GOOD)
        elif res == 2:
            what = ", ".join(n for b, n in self.SU_NAMES if mask & (1 << b)) or "?"
            kv.set("Kontrola po startu", f"SELHALA (bzučák: tři klesající hluboké tóny): {what}", BAD)
        elif res == 3:
            kv.set("Kontrola po startu", "přeskočena (restart za letu)", DIM)
        else:
            kv.set("Kontrola po startu", "probíhá…", WARN)

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
        hil_mode = int(M.get("hil", 0)) if M else 0
        pol_on = bool(M.get("pol", 0)) if M else False
        if hil_mode == 1:
            self.chip_hil.setVisible(True)
            set_chip(self.chip_hil, "HIL AKTIVNÍ (pyro blokováno)", "warn")
        elif hil_mode == 2:
            self.chip_hil.setVisible(True)
            set_chip(self.chip_hil, "HIL AKTIVNÍ · PYRO ŽIVÉ!", "bad")
        else:
            self.chip_hil.setVisible(False)
        if pol_on:
            self.chip_pol.setVisible(True)
            set_chip(self.chip_pol, "TEST SMĚRU (POLARITA · pyro OFF)", "warn")
        else:
            self.chip_pol.setVisible(False)
        self.test_btn.setChecked(bool(self.test_on and self.flight_fw))
        self.test_btn.setEnabled(self.link.is_open and self.flight_fw and hil_mode == 0 and not pol_on)
        self._refresh_polarity(M, N, G, hil_mode)

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
            self._refresh_battery(kv, M)
            self._refresh_startup(kv, M)
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
            sens = P.get("sens", 0)
            li = P.get("lidar", [0.0, 0])
            l_ans = bool(sens & (1 << 5))
            l_gate = bool(sens & (1 << 7))
            l_valid = bool(sens & (1 << 8))
            l_sat = bool(sens & (1 << 9))
            l_str = int(li[1]) * 256 if len(li) > 1 else 0
            if not l_ans:
                kv.set("Lidar kvalita", "neodpovídá", WARN)
            elif l_sat:
                kv.set("Lidar kvalita", "NEKVALITNÍ · přesyceno (nepoužívá se)", WARN)
            elif l_valid:
                kv.set("Lidar kvalita",
                       f"OK · {li[0]:.2f} m · signál ~{l_str} ({'fúze' if l_gate else 'čeká na nohy'})",
                       GOOD)
            else:
                kv.set("Lidar kvalita",
                       f"NEKVALITNÍ · slabý signál (~{l_str}) / mimo rozsah (nepoužívá se)",
                       WARN)
            att_src = P.get("att", 0)
            if att_src == 2:
                kv.set("Orientace", "HIL simulace (USB vstup)", WARN)
            elif att_src == 1:
                kv.set("Orientace", "INSLIB " + INS_MODES.get(P.get("ins", 255), "?"))
            else:
                kv.set("Orientace", "TriSense fúze")
            kv.set("Fúze Hz", fmt(P.get("hz"), 0, "Hz"), GOOD if P.get("hz", 0) > 200 else WARN)
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
            att_src = P.get("att", 0) if P else None
            if att_src == 2:
                self.att_kv.set("Zdroj orientace", "HIL simulace (USB vstup)", WARN)
            elif att_src == 1:
                self.att_kv.set("Zdroj orientace", "INSLIB · " + INS_MODES.get(P.get("ins", 255), "?"))
            elif P:
                self.att_kv.set("Zdroj orientace", "TriSense fúze")
            else:
                self.att_kv.set("Zdroj orientace", "—")
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
            U = self.d["U"] if self._fresh_d("U") else {}
            if U:
                tag_ok = bool(U.get("ok", 0))
                u_err = U.get("err", 0)
                u_rej = U.get("rej", 0)
                if tag_ok:
                    self.sens_kv.set("UWB tag",
                                     f"OK · rádio aktivní · I2C chyby {u_err} · odmítnuto {u_rej}",
                                     GOOD if u_err == 0 else WARN)
                else:
                    self.sens_kv.set("UWB tag",
                                     f"OFFLINE / rádio neběží · I2C chyby {u_err}", WARN)
                anchors = U.get("a", [])
                for idx in range(4):
                    key = f"Kotva {idx + 1}"
                    if idx >= len(anchors):
                        self.sens_kv.set(key, "—", DIM)
                        continue
                    addr, rcm, age_ms, rxp, afl = anchors[idx]
                    if age_ms >= 65535 or rcm >= 65535:
                        self.sens_kv.set(key, f"#{addr} · OFFLINE (nepřijata)", BAD)
                    elif not (afl & 1):
                        self.sens_kv.set(key,
                                         f"#{addr} · OFFLINE (naposled {rcm / 100:.2f} m, {rxp} dBm, před {age_ms / 1000:.1f} s)",
                                         WARN)
                    else:
                        used_s = " · v řešení" if (afl & 2) else ""
                        if afl & 8:
                            st_txt, col = "VELMI SLABÝ", WARN
                        elif afl & 4:
                            st_txt, col = "SLABÝ", WARN
                        else:
                            st_txt, col = "OK", GOOD
                        self.sens_kv.set(key,
                                         f"#{addr} · {st_txt} · {rcm / 100:.2f} m · {rxp} dBm · {age_ms} ms{used_s}",
                                         col)
            else:
                self.sens_kv.set("UWB tag", "čeká na diagnostiku (2 Hz)", DIM)
                for idx in range(4):
                    self.sens_kv.set(f"Kotva {idx + 1}", "—", DIM)
            self.sens_kv.set("Baro trim", fmt(P.get("trim"), 2, "m"), WARN if abs(P.get("trim", 0)) > 1.4 else None)
            self.sens_kv.set("Pad tlak", fmt(P.get("pp"), 0, "Pa"))
            li = P.get("lidar", [0, 0])
            sens_p = P.get("sens", 0)
            if not (sens_p & (1 << 5)):
                self.sens_kv.set("Lidar", "neodpovídá", WARN)
            elif sens_p & (1 << 9):
                self.sens_kv.set("Lidar", f"{li[0]:.2f} m · NEKVALITNÍ (přesyceno, nepoužívá se)", WARN)
            elif sens_p & (1 << 8):
                self.sens_kv.set("Lidar", f"{li[0]:.2f} m · OK (signál ~{li[1] * 256})", GOOD)
            else:
                self.sens_kv.set("Lidar", f"{li[0]:.2f} m · NEKVALITNÍ (signál ~{li[1] * 256}, nepoužívá se)", WARN)
            self.sens_kv.set("Teplota", fmt(P.get("temp"), 1, "°C"))
            self.sens_kv.set("INS (INSLIB)", INS_MODES.get(P.get("ins", 255), "?"))
            self.sens_flags.set(P.get("sens", 0), bad_when_off=(0, 1))
        self._refresh_vib(bool(P))

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
        self.farm_btn.setEnabled(bool(M) and M.get("st") == 1 and hil_mode == 0 and not pol_on)

    def closeEvent(self, e):
        if hasattr(self, "hil_tab"):
            self.hil_tab.shutdown()
        if self.rec_file:
            self.rec_file.close()
        if self.link.is_open:
            if self.d.get("M", {}).get("pol"):
                self.link.send("POLARITY 0")
            if self.test_on:
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

    ag.set_windows_app_id("VoltinoLabs.Daidalos.Diagnostika")
    QtWidgets.QApplication.setHighDpiScaleFactorRoundingPolicy(
        Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)
    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName(APP_NAME)
    icon = ag.load_app_icon()
    if not icon.isNull():
        app.setWindowIcon(icon)
    ag.pick_font(app)
    app.setStyleSheet(ag.QSS)
    win = DiagWindow(args.config, args.demo, args.port)
    if not icon.isNull():
        win.setWindowIcon(icon)
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
