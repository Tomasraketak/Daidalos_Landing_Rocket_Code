#!/usr/bin/env python3
"""
Daidalos — Actuator Console GUI (Ariadna V2, Master MCU)

One screen for every actuator on the Master's PCA9685 and the three pyro
channels on the Navigation MCU:

  * TVC X / Y      2-D pad, sliders, direct entry, centre
  * clamp          slider, closed / centre / open
  * aerocover      lock / unlock (two configured positions) + fine slider
  * 4 fins         sliders, gang mode, centre, motion test
  * pyro 1-3       continuity, arm with confirmation, hold-to-fire
  * configuration  channel, min / centre / max, reverse, us per degree and
                   the two aerocover positions, saved to JSON

Firmware: Examples/Actuator_Console on the Master, Examples/Pyro_Companion
on the Navigation MCU. Styled with the VoltinoLabs design system (dark
ground, one amber accent, Inter).

    pip install -r requirements.txt
    python actuator_gui.py                 # pick the port in the window
    python actuator_gui.py --port COM5     # connect straight away
    python actuator_gui.py --demo          # no hardware: a simulated board
"""
from __future__ import annotations

import argparse
import json
import os
import queue
import sys
import threading
import time
from dataclasses import asdict, dataclass, field

from PyQt6 import QtCore, QtGui, QtWidgets
from PyQt6.QtCore import Qt, pyqtSignal

try:
    import serial
    import serial.tools.list_ports
except ImportError:  # the demo still runs without pyserial
    serial = None

APP_NAME = "Daidalos · Aktuátory a pyro"
HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CONFIG = os.path.join(HERE, "actuator_config.json")
N_CH = 16
ABS_US_MIN, ABS_US_MAX = 500, 2500

# ---------------------------------------------------------------------------
#  VoltinoLabs design tokens (artifact "VoltinoLabs", tokens.json)
# ---------------------------------------------------------------------------
C = {
    "bg": "#161826",
    "bg_raised": "#1c1e2b",
    "surface": "#232532",
    "text": "#e9e9ed",
    "muted": "#b2b6ca",
    "subtle": "#75798c",
    "line": "rgba(233, 233, 237, 0.10)",
    "line_strong": "rgba(233, 233, 237, 0.18)",
    "accent": "#f5a51c",
    "accent_soft": "#ffcf7a",
    "accent_strong": "#ffb734",
    "accent_tint": "rgba(245, 165, 28, 0.12)",
    "positive": "#5dd39e",
    "danger": "#f2777a",
    "danger_tint": "rgba(242, 119, 122, 0.14)",
}

QSS = f"""
QWidget {{
    background: {C['bg']};
    color: {C['text']};
    font-size: 13px;
}}
QToolTip {{
    background: {C['surface']}; color: {C['text']};
    border: 1px solid {C['line_strong']}; padding: 6px;
}}
QFrame#card {{
    background: {C['bg_raised']};
    border: 1px solid {C['line']};
    border-radius: 14px;
}}
QFrame#card QWidget {{ background: transparent; }}
QFrame#header {{
    background: {C['bg']};
    border-bottom: 1px solid {C['line']};
}}
QLabel#kicker {{
    color: {C['accent_soft']};
    font-size: 11px; font-weight: 600;
    letter-spacing: 1.5px;
}}
QLabel#title {{ font-size: 22px; font-weight: 500; }}
QLabel#wordAccent {{ font-size: 22px; font-weight: 500; color: {C['accent']}; }}
QLabel#cardTitle {{ font-size: 16px; font-weight: 500; }}
QLabel#muted {{ color: {C['muted']}; }}
QLabel#subtle {{ color: {C['subtle']}; font-size: 12px; }}
QLabel#value {{ color: {C['text']}; font-size: 13px; font-family: "JetBrains Mono", Menlo, Consolas, monospace; }}
QLabel#big {{ font-size: 15px; font-weight: 600; }}

QLabel#chip {{
    border: 1px solid {C['line_strong']}; border-radius: 6px;
    padding: 3px 8px; color: {C['muted']}; font-size: 12px;
}}
QLabel#chip[state="ok"]   {{ color: {C['positive']}; border-color: rgba(93, 211, 158, 0.45); }}
QLabel#chip[state="bad"]  {{ color: {C['danger']};   border-color: rgba(242, 119, 122, 0.55); }}
QLabel#chip[state="warn"] {{ color: {C['accent']};   border-color: rgba(245, 165, 28, 0.55); }}
QLabel#chip[state="hot"]  {{ color: {C['bg']}; background: {C['danger']}; border-color: {C['danger']}; }}

QPushButton {{
    background: transparent; color: {C['muted']};
    border: 1px solid {C['line_strong']}; border-radius: 8px;
    padding: 6px 12px; font-weight: 500;
}}
QPushButton:hover {{ color: {C['text']}; border-color: rgba(233, 233, 237, 0.35); }}
QPushButton:focus {{ border: 2px solid {C['accent']}; padding: 5px 11px; }}
QPushButton:disabled {{ color: rgba(178, 182, 202, 0.45); border-color: rgba(233, 233, 237, 0.08); }}
QPushButton[kind="primary"] {{ color: {C['accent']}; border-color: {C['accent']}; }}
QPushButton[kind="primary"]:hover {{ background: {C['accent_tint']}; }}
QPushButton[kind="primary"]:disabled {{ color: rgba(245, 165, 28, 0.45); border-color: rgba(245, 165, 28, 0.25); }}
QPushButton[kind="danger"] {{ color: {C['danger']}; border-color: {C['danger']}; }}
QPushButton[kind="danger"]:hover {{ background: {C['danger_tint']}; }}
QPushButton[kind="danger"]:disabled {{ color: rgba(242, 119, 122, 0.4); border-color: rgba(242, 119, 122, 0.2); }}
QPushButton[size="big"] {{ padding: 10px 14px; font-size: 14px; }}
QPushButton[size="big"]:focus {{ padding: 9px 13px; }}
QPushButton[active="true"] {{ background: {C['accent_tint']}; color: {C['accent']}; border-color: {C['accent']}; }}

QComboBox, QSpinBox, QDoubleSpinBox, QLineEdit {{
    background: {C['surface']}; color: {C['text']};
    border: 1px solid {C['line_strong']}; border-radius: 6px;
    padding: 4px 6px; min-height: 20px;
    selection-background-color: {C['accent']}; selection-color: {C['bg']};
}}
QComboBox:focus, QSpinBox:focus, QDoubleSpinBox:focus, QLineEdit:focus {{ border: 1px solid {C['accent']}; }}
QComboBox QAbstractItemView {{ background: {C['surface']}; border: 1px solid {C['line_strong']}; }}
QSpinBox::up-button, QSpinBox::down-button {{ width: 14px; border: none; }}
QSpinBox#cell, QDoubleSpinBox#cell {{
    padding: 0 6px; min-height: 0; margin: 2px 3px; border-radius: 4px;
    font-family: "JetBrains Mono", Menlo, Consolas, monospace;
}}
QSpinBox#cell::up-button, QSpinBox#cell::down-button,
QDoubleSpinBox#cell::up-button, QDoubleSpinBox#cell::down-button {{
    subcontrol-origin: border; width: 0px; border: none;
}}

QCheckBox {{ color: {C['muted']}; spacing: 6px; background: transparent; }}
QCheckBox::indicator {{ width: 14px; height: 14px; border: 1px solid {C['line_strong']}; border-radius: 4px; background: {C['surface']}; }}
QCheckBox::indicator:checked {{ background: {C['accent']}; border-color: {C['accent']}; }}

QSlider::groove:horizontal {{ height: 4px; background: {C['surface']}; border-radius: 2px; }}
QSlider::groove:vertical   {{ width: 4px;  background: {C['surface']}; border-radius: 2px; }}
QSlider::sub-page:horizontal {{ background: rgba(245, 165, 28, 0.35); border-radius: 2px; }}
QSlider::add-page:vertical   {{ background: rgba(245, 165, 28, 0.35); border-radius: 2px; }}
QSlider::handle:horizontal, QSlider::handle:vertical {{
    background: {C['bg']}; border: 2px solid {C['accent']};
    width: 14px; height: 14px; margin: -7px 0; border-radius: 9px;
}}
QSlider::handle:vertical {{ margin: 0 -7px; }}
QSlider::handle:disabled {{ border-color: {C['subtle']}; }}

QTableWidget {{
    background: {C['bg_raised']}; gridline-color: {C['line']};
    border: none; border-radius: 0;
}}
QTableWidget::item {{ padding: 2px; }}
QHeaderView::section {{
    background: {C['bg_raised']}; color: {C['subtle']};
    border: none; border-bottom: 1px solid {C['line']};
    padding: 4px; font-size: 11px; font-weight: 600;
}}
QPlainTextEdit {{
    background: {C['surface']}; color: {C['muted']};
    border: 1px solid {C['line']}; border-radius: 8px;
    font-family: "JetBrains Mono", Menlo, Consolas, monospace; font-size: 12px;
}}
QScrollArea {{ border: none; }}
QScrollBar:vertical {{ background: {C['bg']}; width: 10px; }}
QScrollBar::handle:vertical {{ background: {C['surface']}; border-radius: 5px; min-height: 30px; }}
QScrollBar::add-line, QScrollBar::sub-line {{ height: 0; }}
QMessageBox QLabel {{ color: {C['text']}; }}
"""


def cz(x: float, nd: int = 1, sign: bool = True) -> str:
    """Czech number: decimal comma, real minus sign."""
    s = f"{x:+.{nd}f}" if sign else f"{x:.{nd}f}"
    return s.replace(".", ",").replace("-", "−")


# ---------------------------------------------------------------------------
#  Configuration
# ---------------------------------------------------------------------------
@dataclass
class ServoCfg:
    key: str
    name: str
    ch: int
    min_us: int
    center_us: int
    max_us: int
    reverse: bool = False
    us_per_deg: float = 10.0
    lock_us: int | None = None     # aerocover only
    unlock_us: int | None = None

    def clamp(self, us: float) -> int:
        return int(round(min(max(us, self.min_us), self.max_us)))

    def norm_to_us(self, n: float) -> int:
        """-1..+1 around the centre (asymmetric travel allowed)."""
        n = max(-1.0, min(1.0, -n if self.reverse else n))
        us = self.center_us + n * (self.max_us - self.center_us if n >= 0 else self.center_us - self.min_us)
        return self.clamp(us)

    def us_to_norm(self, us: float) -> float:
        span = (self.max_us - self.center_us) if us >= self.center_us else (self.center_us - self.min_us)
        n = 0.0 if span <= 0 else (us - self.center_us) / span
        return -n if self.reverse else n

    def deg(self, us: float) -> float:
        d = (us - self.center_us) / (self.us_per_deg or 10.0)
        return -d if self.reverse else d


def default_servos() -> list[ServoCfg]:
    # Channels and the clamp's end points follow Master_MCU.ino. TVC: the
    # flight code moves the servo +-10 deg at 10 us/deg, so +-100 us here.
    return [
        ServoCfg("tvc_x", "TVC X", 0, 1400, 1500, 1600),
        ServoCfg("tvc_y", "TVC Y", 1, 1400, 1500, 1600),
        ServoCfg("clamp", "Clamp", 2, 1150, 1500, 1900),
        ServoCfg("fin1", "Křidélko 1", 3, 1200, 1500, 1800),
        ServoCfg("fin2", "Křidélko 2", 4, 1200, 1500, 1800),
        ServoCfg("fin3", "Křidélko 3", 5, 1200, 1500, 1800),
        ServoCfg("fin4", "Křidélko 4", 6, 1200, 1500, 1800),
        ServoCfg("cover", "Aerokryt", 7, 1000, 1500, 2000, lock_us=1200, unlock_us=1800),
    ]


@dataclass
class AppConfig:
    servos: list[ServoCfg] = field(default_factory=default_servos)
    pwm_hz: int = 50
    pulse_ms: list[int] = field(default_factory=lambda: [250, 250, 250])
    port: str = ""

    def servo(self, key: str) -> ServoCfg:
        return next(s for s in self.servos if s.key == key)

    def to_json(self) -> dict:
        return {"version": 1, "pwm_hz": self.pwm_hz, "pulse_ms": self.pulse_ms,
                "port": self.port, "servos": [asdict(s) for s in self.servos]}

    @classmethod
    def from_json(cls, d: dict) -> "AppConfig":
        cfg = cls()
        cfg.pwm_hz = int(d.get("pwm_hz", 50))
        pm = list(d.get("pulse_ms", []))
        cfg.pulse_ms = [max(20, min(1000, int(x))) for x in (pm + [250, 250, 250])[:3]]
        cfg.port = d.get("port", "")
        by_key = {s.get("key"): s for s in d.get("servos", [])}
        for s in cfg.servos:  # unknown keys ignored, missing keys keep defaults
            src = by_key.get(s.key)
            if not src:
                continue
            if int(src.get("min_us", s.min_us)) >= int(src.get("max_us", s.max_us)):
                continue  # nonsense window in the file: keep the defaults
            for f in ("name", "ch", "min_us", "center_us", "max_us", "reverse",
                      "us_per_deg", "lock_us", "unlock_us"):
                if f in src and src[f] is not None:
                    setattr(s, f, type(getattr(s, f))(src[f]) if getattr(s, f) is not None else src[f])
            validate_servo(s)
        return cfg


def validate_servo(s: ServoCfg) -> None:
    s.ch = max(0, min(N_CH - 1, int(s.ch)))
    s.min_us = max(ABS_US_MIN, min(ABS_US_MAX - 2, int(s.min_us)))
    s.max_us = max(s.min_us + 2, min(ABS_US_MAX, int(s.max_us)))
    s.center_us = max(s.min_us, min(s.max_us, int(s.center_us)))
    if s.us_per_deg <= 0:
        s.us_per_deg = 10.0
    if s.lock_us is not None:
        s.lock_us = s.clamp(s.lock_us)
        s.unlock_us = s.clamp(s.unlock_us if s.unlock_us is not None else s.center_us)


# ---------------------------------------------------------------------------
#  Transport: a real serial port, or a simulated board for --demo
# ---------------------------------------------------------------------------
class SimBoard:
    """Speaks the Actuator_Console protocol well enough to try the GUI."""

    def __init__(self):
        self.q: queue.Queue[str] = queue.Queue()
        self.t0 = time.monotonic()
        self.us = [0] * N_CH
        self.lo = [900] * N_CH
        self.hi = [2100] * N_CH
        self.hz = 50
        self.armed_until = 0.0
        self.fire_ch = -1
        self.fire_until = 0.0
        self.fired = 0
        self.rej = 0
        self.last_rx = time.monotonic()
        self.next_tlm = 0.0
        self.rbf = 1

    def ms(self) -> int:
        return int((time.monotonic() - self.t0) * 1000)

    def write(self, data: bytes) -> None:
        for line in data.decode(errors="replace").splitlines():
            self._cmd(line.strip())

    def _cmd(self, line: str) -> None:
        if not line:
            return
        self.last_rx = time.monotonic()
        p = line.split()
        c = p[0].upper()
        now = time.monotonic()
        if c == "HELLO":
            self.q.put(f"I ACTCON 1 {N_CH}")
            self.q.put("# simulated board (--demo): no hardware is driven")
        elif c == "SET" and len(p) == 3:
            ch = int(p[1])
            self.us[ch] = max(self.lo[ch], min(self.hi[ch], int(p[2])))
        elif c == "OFF":
            if p[1].upper().startswith("A"):
                self.us = [0] * N_CH
            else:
                self.us[int(p[1])] = 0
            self.q.put(f"OK OFF {p[1]}")
        elif c == "LIM" and len(p) == 4:
            ch, lo, hi = int(p[1]), max(ABS_US_MIN, int(p[2])), min(ABS_US_MAX, int(p[3]))
            self.lo[ch], self.hi[ch] = lo, hi
            if self.us[ch]:
                self.us[ch] = max(lo, min(hi, self.us[ch]))
            self.q.put(f"OK LIM {ch} {lo} {hi}")
        elif c == "PWM":
            self.hz = int(p[1])
            self.q.put(f"OK PWM {self.hz}")
        elif c == "ARM":
            if not self.rbf:
                self.q.put("ERR ARM RBF pin is in")
            else:
                self.armed_until = now + 60
                self.q.put("OK ARM sent")
        elif c == "DISARM":
            self.armed_until = 0
            self.fire_ch = -1
            self.q.put("OK DISARM sent")
        elif c == "FIRE":
            ch, ms = int(p[1]), int(p[2]) if len(p) > 2 else 250
            if now >= self.armed_until:
                self.q.put("ERR FIRE not armed")
            else:
                ms = max(20, min(1000, ms))
                self.fire_ch, self.fire_until = ch - 1, now + ms / 1000
                self.fired |= 1 << (ch - 1)
                self.q.put(f"OK FIRE {ch} {ms} ms")

    def readline(self, timeout: float = 0.05) -> str | None:
        now = time.monotonic()
        if self.fire_ch >= 0 and now >= self.fire_until:
            self.fire_ch, self.armed_until = -1, 0
        if self.armed_until and now >= self.armed_until:
            self.armed_until = 0
        if self.armed_until and now - self.last_rx > 1.5:
            self.armed_until, self.fire_ch = 0, -1
            self.q.put("# GUI silent - pyros disarmed, servos hold")
        if now >= self.next_tlm:
            self.next_tlm = now + 0.1
            armed = 1 if (self.armed_until > now or self.fire_ch >= 0) else 0
            left = max(0.0, self.armed_until - now) if armed else 0.0
            noise = int((now * 37) % 9)
            t = {"t": self.ms(), "pca": 1, "hz": self.hz, "rbf": self.rbf, "vbat": 7.84,
                 "us": self.us, "nav": 1, "arm": armed, "arm_left": round(left, 1),
                 "cont": [2930 + noise, 2915 + noise, 41 + noise], "thr": 1200, "cmask": 3,
                 "fired": self.fired, "firing": (1 << self.fire_ch) if self.fire_ch >= 0 else 0,
                 "rej": self.rej}
            return "T " + json.dumps(t, separators=(",", ":"))
        try:
            return self.q.get(timeout=timeout)
        except queue.Empty:
            return None

    def close(self) -> None:
        pass


class Link(QtCore.QObject):
    """Line-based link to the board; reading happens on a worker thread."""
    line = pyqtSignal(str)
    state = pyqtSignal(bool, str)

    def __init__(self):
        super().__init__()
        self._dev = None
        self._thread: threading.Thread | None = None
        self._run = False
        self._lock = threading.Lock()

    @property
    def is_open(self) -> bool:
        return self._dev is not None

    def open(self, port: str) -> None:
        self.close()
        try:
            if port == "DEMO":
                self._dev = SimBoard()
            else:
                if serial is None:
                    raise RuntimeError("chybí pyserial: pip install pyserial")
                self._dev = serial.Serial(port, 115200, timeout=0.05, write_timeout=0.2)
        except Exception as e:  # noqa: BLE001 - shown to the user
            self._dev = None
            self.state.emit(False, f"Nelze otevřít {port}: {e}")
            return
        self._run = True
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()
        self.state.emit(True, port)

    def close(self) -> None:
        self._run = False
        if self._thread:
            self._thread.join(timeout=0.5)
        self._thread = None
        if self._dev is not None:
            try:
                self._dev.close()
            except Exception:  # noqa: BLE001
                pass
            self._dev = None
            self.state.emit(False, "Odpojeno")

    def send(self, text: str) -> None:
        dev = self._dev
        if dev is None:
            return
        try:
            with self._lock:
                dev.write((text + "\n").encode())
        except Exception as e:  # noqa: BLE001
            self.line.emit(f"# chyba zápisu: {e}")
            QtCore.QTimer.singleShot(0, self.close)

    def _reader(self) -> None:
        buf = b""
        while self._run and self._dev is not None:
            dev = self._dev
            try:
                if isinstance(dev, SimBoard):
                    ln = dev.readline()
                    if ln is not None:
                        self.line.emit(ln)
                    continue
                chunk = dev.read(dev.in_waiting or 1)
            except Exception as e:  # noqa: BLE001 - cable pulled
                self.line.emit(f"# port ztracen: {e}")
                QtCore.QTimer.singleShot(0, self.close)
                return
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                self.line.emit(raw.decode(errors="replace").strip())


# ---------------------------------------------------------------------------
#  Widgets
# ---------------------------------------------------------------------------
def repolish(w: QtWidgets.QWidget) -> None:
    w.style().unpolish(w)
    w.style().polish(w)


def chip(text: str = "", state: str = "") -> QtWidgets.QLabel:
    lab = QtWidgets.QLabel(text)
    lab.setObjectName("chip")
    lab.setProperty("state", state)
    return lab


def set_chip(lab: QtWidgets.QLabel, text: str, state: str) -> None:
    if lab.text() != text:
        lab.setText(text)
    if lab.property("state") != state:
        lab.setProperty("state", state)
        repolish(lab)


def button(text: str, kind: str = "", tip: str = "") -> QtWidgets.QPushButton:
    b = QtWidgets.QPushButton(text)
    if kind:
        b.setProperty("kind", kind)
    if tip:
        b.setToolTip(tip)
    b.setCursor(Qt.CursorShape.PointingHandCursor)
    return b


def label(text: str, name: str = "") -> QtWidgets.QLabel:
    lab = QtWidgets.QLabel(text)
    if name:
        lab.setObjectName(name)
    return lab


class Card(QtWidgets.QFrame):
    def __init__(self, kicker: str, title: str):
        super().__init__()
        self.setObjectName("card")
        self.lay = QtWidgets.QVBoxLayout(self)
        self.lay.setContentsMargins(16, 14, 16, 16)
        self.lay.setSpacing(10)
        head = QtWidgets.QHBoxLayout()
        col = QtWidgets.QVBoxLayout()
        col.setSpacing(2)
        col.addWidget(label(kicker.upper(), "kicker"))
        col.addWidget(label(title, "cardTitle"))
        head.addLayout(col)
        head.addStretch(1)
        self.head = head
        self.lay.addLayout(head)


class XYPad(QtWidgets.QWidget):
    """Square pad, -1..+1 on both axes, drag to move, double-click centres."""
    moved = pyqtSignal(float, float)

    def __init__(self):
        super().__init__()
        self.x = 0.0
        self.y = 0.0
        self.setMinimumSize(200, 200)
        self.setMaximumSize(250, 250)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.setCursor(Qt.CursorShape.CrossCursor)
        self.setToolTip("Táhněte myší · dvojklik = střed · šipky = krok 5 %")

    def sizeHint(self):
        return QtCore.QSize(250, 250)

    def hasHeightForWidth(self):
        return True

    def heightForWidth(self, w):
        return w

    def set_pos(self, x: float, y: float) -> None:
        self.x, self.y = max(-1, min(1, x)), max(-1, min(1, y))
        self.update()

    def _area(self) -> QtCore.QRectF:
        s = min(self.width(), self.height()) - 2
        return QtCore.QRectF((self.width() - s) / 2, (self.height() - s) / 2, s, s)

    def _from_mouse(self, p: QtCore.QPointF) -> None:
        a = self._area()
        x = (p.x() - a.center().x()) / (a.width() / 2 - 10)
        y = -(p.y() - a.center().y()) / (a.height() / 2 - 10)
        self.set_pos(x, y)
        self.moved.emit(self.x, self.y)

    def mousePressEvent(self, e):
        self._from_mouse(e.position())

    def mouseMoveEvent(self, e):
        if e.buttons() & Qt.MouseButton.LeftButton:
            self._from_mouse(e.position())

    def mouseDoubleClickEvent(self, e):
        self.set_pos(0, 0)
        self.moved.emit(0.0, 0.0)

    def keyPressEvent(self, e):
        step = 0.05
        k = e.key()
        dx = {Qt.Key.Key_Left: -step, Qt.Key.Key_Right: step}.get(k, 0)
        dy = {Qt.Key.Key_Down: -step, Qt.Key.Key_Up: step}.get(k, 0)
        if k in (Qt.Key.Key_Home, Qt.Key.Key_C):
            self.set_pos(0, 0)
            self.moved.emit(0.0, 0.0)
        elif dx or dy:
            self.set_pos(self.x + dx, self.y + dy)
            self.moved.emit(self.x, self.y)
        else:
            super().keyPressEvent(e)

    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        a = self._area()
        p.setPen(QtGui.QPen(QtGui.QColor(233, 233, 237, 46), 1))
        p.setBrush(QtGui.QColor(C["surface"]))
        p.drawRoundedRect(a, 8, 8)
        inner = a.adjusted(10, 10, -10, -10)
        grid = QtGui.QPen(QtGui.QColor(233, 233, 237, 22), 1)
        p.setPen(grid)
        for i in (1, 3):
            fx = inner.left() + inner.width() * i / 4
            fy = inner.top() + inner.height() * i / 4
            p.drawLine(QtCore.QPointF(fx, inner.top()), QtCore.QPointF(fx, inner.bottom()))
            p.drawLine(QtCore.QPointF(inner.left(), fy), QtCore.QPointF(inner.right(), fy))
        p.setPen(QtGui.QPen(QtGui.QColor(233, 233, 237, 60), 1))
        c = inner.center()
        p.drawLine(QtCore.QPointF(c.x(), inner.top()), QtCore.QPointF(c.x(), inner.bottom()))
        p.drawLine(QtCore.QPointF(inner.left(), c.y()), QtCore.QPointF(inner.right(), c.y()))
        p.setPen(QtGui.QColor(C["subtle"]))
        f = p.font()
        f.setPixelSize(11)
        p.setFont(f)
        p.drawText(QtCore.QRectF(inner.right() - 30, c.y() + 2, 28, 14), Qt.AlignmentFlag.AlignRight, "+X")
        p.drawText(QtCore.QRectF(c.x() + 4, inner.top(), 28, 14), Qt.AlignmentFlag.AlignLeft, "+Y")
        pt = QtCore.QPointF(c.x() + self.x * inner.width() / 2, c.y() - self.y * inner.height() / 2)
        acc = QtGui.QColor(C["accent"])
        glow = QtGui.QColor(acc)
        glow.setAlpha(50)
        p.setPen(QtGui.QPen(glow, 1, Qt.PenStyle.DashLine))
        p.drawLine(c, pt)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(glow)
        p.drawEllipse(pt, 14, 14)
        p.setPen(QtGui.QPen(acc, 2))
        p.setBrush(QtGui.QColor(C["bg"]))
        p.drawEllipse(pt, 8, 8)
        if self.hasFocus():
            p.setPen(QtGui.QPen(acc, 2))
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawRoundedRect(a.adjusted(1, 1, -1, -1), 8, 8)


class HoldButton(QtWidgets.QPushButton):
    """Fires only after being held down for `hold_ms`; shows the progress."""
    held = pyqtSignal()

    def __init__(self, text: str, hold_ms: int = 1200):
        super().__init__(text)
        self.hold_ms = hold_ms
        self.setProperty("kind", "danger")
        self._t0 = 0.0
        self._prog = 0.0
        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(16)
        self._timer.timeout.connect(self._tick)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setToolTip(f"Držte {hold_ms / 1000:.1f} s".replace(".", ","))

    def mousePressEvent(self, e):
        if e.button() == Qt.MouseButton.LeftButton and self.isEnabled():
            self._t0 = time.monotonic()
            self._timer.start()
        super().mousePressEvent(e)

    def mouseReleaseEvent(self, e):
        self._cancel()
        super().mouseReleaseEvent(e)

    def leaveEvent(self, e):
        self._cancel()
        super().leaveEvent(e)

    def changeEvent(self, e):
        if e.type() == QtCore.QEvent.Type.EnabledChange and not self.isEnabled():
            self._cancel()
        super().changeEvent(e)

    def _cancel(self):
        self._timer.stop()
        self._prog = 0.0
        self.update()

    def _tick(self):
        self._prog = (time.monotonic() - self._t0) * 1000 / self.hold_ms
        if self._prog >= 1.0:
            self._cancel()
            self.held.emit()
        self.update()

    def paintEvent(self, e):
        super().paintEvent(e)
        if self._prog > 0:
            p = QtGui.QPainter(self)
            col = QtGui.QColor(C["danger"])
            col.setAlpha(70)
            r = QtCore.QRectF(self.rect()).adjusted(1, 1, -1, -1)
            r.setWidth(r.width() * min(1.0, self._prog))
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(col)
            p.drawRoundedRect(r, 7, 7)


class AxisControl(QtWidgets.QWidget):
    """One servo: slider + direct entry + readouts. Horizontal or vertical."""
    changed = pyqtSignal(str, int)  # key, us

    def __init__(self, cfg: ServoCfg, vertical: bool = False, show_name: bool = True):
        super().__init__()
        self.cfg = cfg
        self._quiet = False
        orient = Qt.Orientation.Vertical if vertical else Qt.Orientation.Horizontal
        self.slider = QtWidgets.QSlider(orient)
        self.slider.setCursor(Qt.CursorShape.PointingHandCursor)
        self.spin = QtWidgets.QSpinBox()
        self.spin.setSuffix(" µs")
        self.spin.setKeyboardTracking(False)
        self.spin.setAccelerated(True)
        self.deg = label("", "value")
        self.board = label("deska: —", "subtle")
        self.name = label(cfg.name, "big")
        self.center_btn = button("Střed", tip="Na nastavený střed")
        self.center_btn.clicked.connect(lambda: self.set_us(self.cfg.center_us, emit=True))

        if vertical:
            self.slider.setFixedHeight(150)
            lay = QtWidgets.QVBoxLayout(self)
            lay.setContentsMargins(0, 0, 0, 0)
            lay.setSpacing(4)
            if show_name:
                lay.addWidget(self.name, alignment=Qt.AlignmentFlag.AlignHCenter)
            lay.addWidget(self.slider, 0, Qt.AlignmentFlag.AlignHCenter)
            lay.addWidget(self.spin)
            lay.addWidget(self.deg, alignment=Qt.AlignmentFlag.AlignHCenter)
            lay.addWidget(self.board, alignment=Qt.AlignmentFlag.AlignHCenter)
            lay.addWidget(self.center_btn)
            lay.addStretch(1)
        else:
            grid = QtWidgets.QGridLayout(self)
            grid.setContentsMargins(0, 0, 0, 0)
            grid.setHorizontalSpacing(10)
            grid.setVerticalSpacing(4)
            if show_name:
                grid.addWidget(self.name, 0, 0)
            grid.addWidget(self.deg, 0, 1, alignment=Qt.AlignmentFlag.AlignRight)
            grid.addWidget(self.board, 0, 2, alignment=Qt.AlignmentFlag.AlignRight)
            grid.addWidget(self.slider, 1, 0, 1, 2)
            grid.addWidget(self.spin, 1, 2)
            grid.addWidget(self.center_btn, 1, 3)
            grid.setColumnStretch(0, 1)
            self.spin.setMinimumWidth(96)

        self.slider.valueChanged.connect(lambda v: self._user(v))
        self.spin.valueChanged.connect(lambda v: self._user(v))
        self.apply_cfg()

    def apply_cfg(self) -> None:
        c = self.cfg
        self._quiet = True
        for w in (self.slider, self.spin):
            w.setRange(c.min_us, c.max_us)
        self.slider.setSingleStep(1)
        self.slider.setPageStep(max(1, (c.max_us - c.min_us) // 20))
        self.slider.setInvertedAppearance(c.reverse)
        self.slider.setInvertedControls(c.reverse)
        self.name.setText(c.name)
        self.setToolTip(f"{c.name} · kanál {c.ch} · {c.min_us}–{c.max_us} µs, střed {c.center_us} µs")
        self._quiet = False
        self._readout(self.spin.value())

    def value(self) -> int:
        return self.spin.value()

    def set_us(self, us: int, emit: bool = False) -> None:
        us = self.cfg.clamp(us)
        self._quiet = True
        self.slider.setValue(us)
        self.spin.setValue(us)
        self._quiet = False
        self._readout(us)
        if emit:
            self.changed.emit(self.cfg.key, us)

    def _user(self, v: int) -> None:
        if self._quiet:
            return
        self.set_us(v, emit=True)

    def _readout(self, us: int) -> None:
        self.deg.setText(f"≈ {cz(self.cfg.deg(us))}°")

    def set_board(self, us: int | None) -> None:
        txt = "deska: —" if us is None else ("deska: vyp." if us == 0 else f"deska: {us} µs")
        if self.board.text() != txt:
            self.board.setText(txt)


# ---------------------------------------------------------------------------
#  Main window
# ---------------------------------------------------------------------------
REJECT = {0: "", 1: "není aktivováno", 2: "špatný kanál", 3: "jiný kanál právě pálí",
          4: "ztráta spojení s Masterem", 5: "neznámý příkaz"}
PYRO_NAMES = ["Kanál 1 · přistávací motor", "Kanál 2 · záložní zapalovač", "Kanál 3 · rezerva"]
CFG_COLS = ["Servo", "Kanál", "Min µs", "Střed µs", "Max µs", "Reverz", "µs / °",
            "Zajištěno µs", "Odjištěno µs", "Deska"]


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, cfg_path: str, demo: bool, port: str | None):
        super().__init__()
        self.cfg_path = cfg_path
        self.cfg = self._load(cfg_path)
        self.link = Link()
        self.link.line.connect(self.on_line)
        self.link.state.connect(self.on_link_state)
        self.tlm: dict = {}
        self.tlm_time = 0.0
        self.pending: dict[int, int] = {}
        self.axes: dict[str, AxisControl] = {}
        self._fin_test_step = -1
        self._dirty = False

        self.setWindowTitle(APP_NAME)
        self.resize(1440, 940)
        root = QtWidgets.QWidget()
        outer = QtWidgets.QVBoxLayout(root)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)
        outer.addWidget(self._build_header())
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        body = QtWidgets.QWidget()
        scroll.setWidget(body)
        outer.addWidget(scroll, 1)
        self.setCentralWidget(root)

        grid = QtWidgets.QGridLayout(body)
        grid.setContentsMargins(20, 16, 20, 20)
        grid.setHorizontalSpacing(16)
        grid.setVerticalSpacing(16)
        # Row 0: everything that moves. Row 1: how it is wired, the cover, the log.
        grid.addWidget(self._build_tvc(), 0, 0)
        mid = QtWidgets.QVBoxLayout()
        mid.setSpacing(16)
        mid.addWidget(self._build_fins(), 1)
        mid.addWidget(self._build_clamp())
        grid.addLayout(mid, 0, 1)
        grid.addWidget(self._build_pyro(), 0, 2)
        grid.addWidget(self._build_config(), 1, 0, 1, 2)
        right = QtWidgets.QVBoxLayout()
        right.setSpacing(16)
        right.addWidget(self._build_cover())
        right.addWidget(self._build_log(), 1)
        grid.addLayout(right, 1, 2)
        for i in range(3):
            grid.setColumnStretch(i, 1)
        grid.setRowStretch(1, 1)

        for s in self.cfg.servos:
            if s.key in self.axes:
                self.axes[s.key].set_us(s.center_us)
        self._sync_tvc_pad()
        self._update_cover_state()

        self.flush_timer = QtCore.QTimer(self, interval=20, timeout=self._flush)
        self.flush_timer.start()
        self.hb_timer = QtCore.QTimer(self, interval=400, timeout=self._heartbeat)
        self.hb_timer.start()
        self.ui_timer = QtCore.QTimer(self, interval=250, timeout=self._refresh_status)
        self.ui_timer.start()
        self.fin_timer = QtCore.QTimer(self, interval=700, timeout=self._fin_test_tick)

        QtGui.QShortcut(QtGui.QKeySequence(Qt.Key.Key_Escape), self, activated=self.disarm)
        QtGui.QShortcut(QtGui.QKeySequence("Ctrl+S"), self, activated=self.save)

        self.refresh_ports()
        self._update_controls_enabled()
        self._refresh_status()
        self._check_conflicts()
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

    # ------------------------------------------------------------- config io
    def _load(self, path: str) -> AppConfig:
        try:
            with open(path, encoding="utf-8") as f:
                return AppConfig.from_json(json.load(f))
        except FileNotFoundError:
            return AppConfig()
        except Exception as e:  # noqa: BLE001
            msg = f"# konfiguraci nelze načíst ({e}), výchozí hodnoty"
            QtCore.QTimer.singleShot(0, lambda: self.log(msg))
            return AppConfig()

    def save(self) -> None:
        self.cfg.port = self.port_box.currentData() or ""
        try:
            with open(self.cfg_path, "w", encoding="utf-8") as f:
                json.dump(self.cfg.to_json(), f, ensure_ascii=False, indent=2)
            self._dirty = False
            self.log(f"# konfigurace uložena: {self.cfg_path}")
        except OSError as e:
            QtWidgets.QMessageBox.warning(self, APP_NAME, f"Konfiguraci nelze uložit:\n{e}")

    def save_as(self) -> None:
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Uložit konfiguraci", self.cfg_path, "JSON (*.json)")
        if path:
            self.cfg_path = path
            self.save()

    def load_from(self) -> None:
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Načíst konfiguraci", os.path.dirname(self.cfg_path), "JSON (*.json)")
        if not path:
            return
        try:
            with open(path, encoding="utf-8") as f:
                new = AppConfig.from_json(json.load(f))
        except Exception as e:  # noqa: BLE001
            QtWidgets.QMessageBox.warning(self, APP_NAME, f"Soubor nelze načíst:\n{e}")
            return
        self.cfg_path = path
        self._replace_cfg(new)
        self.log(f"# konfigurace načtena: {path}")

    def reset_defaults(self) -> None:
        if QtWidgets.QMessageBox.question(self, APP_NAME, "Vrátit kanály a limity všech serv na výchozí hodnoty?") \
                != QtWidgets.QMessageBox.StandardButton.Yes:
            return
        new = AppConfig()
        new.port = self.cfg.port
        self._replace_cfg(new)

    def _replace_cfg(self, new: AppConfig) -> None:
        for s_old, s_new in zip(self.cfg.servos, new.servos):
            for k, v in asdict(s_new).items():
                setattr(s_old, k, v)
        self.cfg.pwm_hz, self.cfg.pulse_ms = new.pwm_hz, new.pulse_ms
        self._fill_config_table()
        for s in self.cfg.servos:
            self._cfg_changed(s, send_pos=False)
        self.pwm_box.setCurrentIndex(max(0, self.pwm_box.findData(self.cfg.pwm_hz)))
        for i, sp in enumerate(self.pulse_spins):
            sp.setValue(self.cfg.pulse_ms[i])
        self._dirty = True

    # --------------------------------------------------------------- header
    def _build_header(self) -> QtWidgets.QWidget:
        h = QtWidgets.QFrame()
        h.setObjectName("header")
        lay = QtWidgets.QHBoxLayout(h)
        lay.setContentsMargins(20, 12, 20, 12)
        lay.setSpacing(10)
        brand = QtWidgets.QVBoxLayout()
        brand.setSpacing(0)
        brand.addWidget(label("ARIADNA V2 · MASTER · AKTUÁTORY A PYRO", "kicker"))
        word = QtWidgets.QHBoxLayout()
        word.setSpacing(0)
        word.addWidget(label("Daidalos", "title"))
        word.addWidget(label(" Console", "wordAccent"))
        word.addStretch(1)
        brand.addLayout(word)
        lay.addLayout(brand)
        lay.addSpacing(18)

        self.port_box = QtWidgets.QComboBox()
        self.port_box.setMinimumWidth(190)
        self.port_box.setToolTip("Sériový port Masteru (USB)")
        lay.addWidget(self.port_box)
        b = button("Obnovit", tip="Znovu vyhledat porty")
        b.clicked.connect(self.refresh_ports)
        lay.addWidget(b)
        self.conn_btn = button("Připojit", "primary")
        self.conn_btn.clicked.connect(self.connect_toggle)
        lay.addWidget(self.conn_btn)

        self.pwm_box = QtWidgets.QComboBox()
        self.pwm_box.addItem("50 Hz · všechna serva", 50)
        self.pwm_box.addItem("333 Hz · jen digitální", 333)
        self.pwm_box.addItem("339 Hz · letový (digitální)", 339)
        self.pwm_box.setCurrentIndex(max(0, self.pwm_box.findData(self.cfg.pwm_hz)))
        self.pwm_box.setToolTip("Frekvence PCA9685 – společná pro všech 16 kanálů.\n"
                                "Analogová serva na 333/339 Hz bzučí a přehřívají se.")
        self.pwm_box.currentIndexChanged.connect(self._pwm_changed)
        lay.addWidget(self.pwm_box)

        lay.addStretch(1)
        self.chip_board = chip("Deska")
        self.chip_pca = chip("PCA9685")
        self.chip_nav = chip("Navigation")
        self.chip_rbf = chip("RBF")
        self.chip_bat = chip("Baterie")
        for c_ in (self.chip_board, self.chip_pca, self.chip_nav, self.chip_rbf, self.chip_bat):
            lay.addWidget(c_)
        lay.addSpacing(8)
        self.all_center_btn = button("Vše na střed", "primary", "Všechna serva na nastavený střed (zapne výstupy)")
        self.all_center_btn.clicked.connect(self.all_center)
        lay.addWidget(self.all_center_btn)
        self.all_off_btn = button("Vypnout serva", tip="Zastaví pulzy na všech kanálech (serva povolí)")
        self.all_off_btn.clicked.connect(self.all_off)
        lay.addWidget(self.all_off_btn)
        return h

    # ------------------------------------------------------------------ TVC
    def _build_tvc(self) -> QtWidgets.QWidget:
        card = Card("Vektorování tahu", "TVC X / Y")
        b = button("Střed", "primary", "Obě osy na střed (Home)")
        b.clicked.connect(lambda: self._tvc_from_pad(0.0, 0.0, move_pad=True))
        card.head.addWidget(b)
        self.pad = XYPad()
        self.pad.moved.connect(lambda x, y: self._tvc_from_pad(x, y))
        card.lay.addWidget(self.pad, 0, Qt.AlignmentFlag.AlignHCenter)
        self.pad_label = label("", "subtle")
        self.pad_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        card.lay.addWidget(self.pad_label)
        for key in ("tvc_x", "tvc_y"):
            ax = AxisControl(self.cfg.servo(key))
            ax.changed.connect(self.on_axis)
            self.axes[key] = ax
            card.lay.addWidget(ax)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(label("Krok testu", "subtle"))
        for txt, pts in (("Kříž", [(1, 0), (-1, 0), (0, 1), (0, -1), (0, 0)]),
                         ("Kruh", None)):
            bb = button(txt, tip="Projede krajní polohy a vrátí se na střed")
            bb.clicked.connect(lambda _=False, p=pts: self._tvc_test(p))
            row.addWidget(bb)
        row.addStretch(1)
        card.lay.addLayout(row)
        self._tvc_seq: list[tuple[float, float]] = []
        self.tvc_timer = QtCore.QTimer(self, interval=60, timeout=self._tvc_test_tick)
        return card

    def _tvc_from_pad(self, x: float, y: float, move_pad: bool = False) -> None:
        if move_pad:
            self.pad.set_pos(x, y)
        sx, sy = self.cfg.servo("tvc_x"), self.cfg.servo("tvc_y")
        # The pad is in "nozzle" sense: reverse is already in norm_to_us.
        self.axes["tvc_x"].set_us(sx.norm_to_us(x), emit=True)
        self.axes["tvc_y"].set_us(sy.norm_to_us(y), emit=True)
        self._pad_text()

    def _sync_tvc_pad(self) -> None:
        sx, sy = self.cfg.servo("tvc_x"), self.cfg.servo("tvc_y")
        self.pad.set_pos(sx.us_to_norm(self.axes["tvc_x"].value()), sy.us_to_norm(self.axes["tvc_y"].value()))
        self._pad_text()

    def _pad_text(self) -> None:
        sx, sy = self.cfg.servo("tvc_x"), self.cfg.servo("tvc_y")
        self.pad_label.setText(f"X {cz(sx.deg(self.axes['tvc_x'].value()))}°   ·   "
                               f"Y {cz(sy.deg(self.axes['tvc_y'].value()))}°   (servo)")

    def _tvc_test(self, pts) -> None:
        import math
        if pts is None:
            pts = [(math.cos(a / 24 * 2 * math.pi), math.sin(a / 24 * 2 * math.pi)) for a in range(25)] + [(0, 0)]
            self.tvc_timer.setInterval(80)
        else:
            pts = [p for p in pts for _ in range(8)]  # hold each corner ~0.5 s
            self.tvc_timer.setInterval(60)
        self._tvc_seq = list(pts)
        self.tvc_timer.start()

    def _tvc_test_tick(self) -> None:
        if not self._tvc_seq:
            self.tvc_timer.stop()
            return
        x, y = self._tvc_seq.pop(0)
        self._tvc_from_pad(x * 0.9, y * 0.9, move_pad=True)

    # ----------------------------------------------------------------- fins
    def _build_fins(self) -> QtWidgets.QWidget:
        card = Card("Aerodynamika", "Naklápěcí křidélka")
        self.gang = QtWidgets.QCheckBox("Společně")
        self.gang.setToolTip("Pohyb jednoho křidélka posune všechna o stejný podíl rozsahu")
        card.head.addWidget(self.gang)
        b = button("Vše střed", "primary")
        b.clicked.connect(lambda: [self.axes[k].set_us(self.cfg.servo(k).center_us, emit=True)
                                   for k in ("fin1", "fin2", "fin3", "fin4")])
        card.head.addWidget(b)
        self.fin_test_btn = button("Test pohybu", tip="Všechna křidélka: střed → +50 % → −50 % → střed")
        self.fin_test_btn.clicked.connect(self._fin_test_start)
        card.head.addWidget(self.fin_test_btn)
        row = QtWidgets.QHBoxLayout()
        row.setSpacing(14)
        for key in ("fin1", "fin2", "fin3", "fin4"):
            ax = AxisControl(self.cfg.servo(key), vertical=True)
            ax.changed.connect(self.on_axis)
            ax.spin.setMinimumWidth(84)
            self.axes[key] = ax
            row.addWidget(ax)
        card.lay.addLayout(row, 1)
        return card

    def _fin_test_start(self) -> None:
        self._fin_test_step = 0
        self.fin_timer.start()
        self._fin_test_tick()

    def _fin_test_tick(self) -> None:
        seq = [0.0, 0.5, 0.0, -0.5, 0.0]
        if self._fin_test_step < 0 or self._fin_test_step >= len(seq):
            self.fin_timer.stop()
            self._fin_test_step = -1
            return
        n = seq[self._fin_test_step]
        for k in ("fin1", "fin2", "fin3", "fin4"):
            self.axes[k].set_us(self.cfg.servo(k).norm_to_us(n), emit=True)
        self._fin_test_step += 1

    # ---------------------------------------------------------------- clamp
    def _build_clamp(self) -> QtWidgets.QWidget:
        card = Card("Řízení tahu", "Clamp")
        ax = AxisControl(self.cfg.servo("clamp"), show_name=False)
        ax.changed.connect(self.on_axis)
        self.axes["clamp"] = ax
        card.lay.addWidget(ax)
        row = QtWidgets.QHBoxLayout()
        for txt, which in (("Zavřeno (min)", "min"), ("Střed", "center"), ("Otevřeno (max)", "max")):
            bb = button(txt)
            bb.clicked.connect(lambda _=False, w=which: self._clamp_to(w))
            row.addWidget(bb)
        card.lay.addLayout(row)
        return card

    def _clamp_to(self, which: str) -> None:
        s = self.cfg.servo("clamp")
        self.axes["clamp"].set_us({"min": s.min_us, "center": s.center_us, "max": s.max_us}[which], emit=True)

    # ------------------------------------------------------------ aerocover
    def _build_cover(self) -> QtWidgets.QWidget:
        card = Card("Nosový kužel", "Aerokryt")
        self.cover_state = chip("—")
        card.head.addWidget(self.cover_state)
        row = QtWidgets.QHBoxLayout()
        self.lock_btn = button("Zajistit", "", "Servo aerokrytu do polohy ZAJIŠTĚNO")
        self.unlock_btn = button("Odjistit", "", "Servo aerokrytu do polohy ODJIŠTĚNO")
        for b_ in (self.lock_btn, self.unlock_btn):
            b_.setProperty("size", "big")
        self.lock_btn.clicked.connect(lambda: self._cover_to(True))
        self.unlock_btn.clicked.connect(lambda: self._cover_to(False))
        row.addWidget(self.lock_btn)
        row.addWidget(self.unlock_btn)
        card.lay.addLayout(row)
        ax = AxisControl(self.cfg.servo("cover"), show_name=False)
        ax.center_btn.setText("Střed")
        ax.changed.connect(self.on_axis)
        ax.changed.connect(lambda *_: self._update_cover_state())
        self.axes["cover"] = ax
        card.lay.addWidget(ax)
        hint = QtWidgets.QHBoxLayout()
        hint.addWidget(label("Jemné doladění. Aktuální polohu uložíte jako:", "subtle"))
        hint.addStretch(1)
        for txt, lock in (("zajištěno", True), ("odjištěno", False)):
            bb = button(txt)
            bb.setToolTip(f"Uložit aktuální polohu jako {txt.upper()}")
            bb.clicked.connect(lambda _=False, lk=lock: self._cover_store(lk))
            hint.addWidget(bb)
        card.lay.addLayout(hint)
        return card

    def _cover_to(self, lock: bool) -> None:
        s = self.cfg.servo("cover")
        self.axes["cover"].set_us(s.lock_us if lock else s.unlock_us, emit=True)
        self._update_cover_state()

    def _cover_store(self, lock: bool) -> None:
        s = self.cfg.servo("cover")
        us = self.axes["cover"].value()
        if lock:
            s.lock_us = us
        else:
            s.unlock_us = us
        self._fill_config_table()
        self._update_cover_state()
        self._dirty = True
        self.log(f"# aerokryt: {'zajištěno' if lock else 'odjištěno'} = {us} µs (Ctrl+S uloží)")

    def _update_cover_state(self) -> None:
        s = self.cfg.servo("cover")
        us = self.axes["cover"].value()
        if us == s.lock_us:
            set_chip(self.cover_state, "ZAJIŠTĚNO", "ok")
        elif us == s.unlock_us:
            set_chip(self.cover_state, "ODJIŠTĚNO", "warn")
        else:
            set_chip(self.cover_state, f"mezipoloha {us} µs", "")
        for b_, on in ((self.lock_btn, us == s.lock_us), (self.unlock_btn, us == s.unlock_us)):
            if b_.property("active") != ("true" if on else "false"):
                b_.setProperty("active", "true" if on else "false")
                repolish(b_)
        self.lock_btn.setText(f"Zajistit · {s.lock_us} µs")
        self.unlock_btn.setText(f"Odjistit · {s.unlock_us} µs")

    # ----------------------------------------------------------------- pyro
    def _build_pyro(self) -> QtWidgets.QWidget:
        card = Card("Navigation MCU", "Pyro kanály")
        self.arm_chip = chip("NEAKTIVNÍ")
        card.head.addWidget(self.arm_chip)

        self.arm_btn = button("Aktivovat pyro…", "primary",
                              "Odjistí pyro kanály na Navigation MCU (60 s, pak se samy zajistí)")
        self.arm_btn.setProperty("size", "big")
        self.arm_btn.clicked.connect(self.arm_clicked)
        self.disarm_btn = button("Deaktivovat  (Esc)", "", "Okamžitě zajistí pyro kanály")
        self.disarm_btn.setProperty("size", "big")
        self.disarm_btn.clicked.connect(self.disarm)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(self.arm_btn, 1)
        row.addWidget(self.disarm_btn, 1)
        card.lay.addLayout(row)
        self.pyro_note = label("", "muted")
        self.pyro_note.setWordWrap(True)
        card.lay.addWidget(self.pyro_note)

        self.pyro_rows = []
        self.pulse_spins: list[QtWidgets.QSpinBox] = []
        for i in range(3):
            fr = QtWidgets.QFrame()
            fr.setStyleSheet(f"QFrame {{ border-top: 1px solid {C['line']}; }} QLabel, QPushButton, QSpinBox {{ border-top: none; }}")
            g = QtWidgets.QGridLayout(fr)
            g.setContentsMargins(0, 10, 0, 2)
            g.setHorizontalSpacing(8)
            g.setVerticalSpacing(6)
            head = QtWidgets.QHBoxLayout()
            head.addWidget(label(PYRO_NAMES[i], "big"))
            head.addStretch(1)
            cont = chip("—")
            fired = chip("")
            fired.setVisible(False)
            head.addWidget(fired)
            head.addWidget(cont)
            g.addLayout(head, 0, 0, 1, 3)
            raw = label("ADC —", "subtle")
            g.addWidget(raw, 1, 0)
            sp = QtWidgets.QSpinBox()
            sp.setRange(20, 1000)
            sp.setSingleStep(10)
            sp.setSuffix(" ms")
            sp.setValue(self.cfg.pulse_ms[i])
            sp.setToolTip("Délka pulzu (20–1000 ms, deska ji omezí taky)")
            sp.valueChanged.connect(lambda v, k=i: self._pulse_changed(k, v))
            self.pulse_spins.append(sp)
            fire = HoldButton(f"Podržet: ODPÁLIT {i + 1}")
            fire.held.connect(lambda k=i: self.fire(k + 1))
            g.addWidget(sp, 1, 1)
            g.addWidget(fire, 1, 2)
            g.setColumnStretch(2, 1)
            card.lay.addWidget(fr)
            self.pyro_rows.append({"cont": cont, "raw": raw, "fired": fired, "fire": fire})
        card.lay.addStretch(1)
        warn = label("Bez zapalovače testujte s LED nebo rezistorem. S motorem jen podle postupu "
                     "statického zážehu: lidé mimo, motor zajištěný, výhled na stanoviště.", "subtle")
        warn.setWordWrap(True)
        card.lay.addWidget(warn)
        return card

    def _pulse_changed(self, i: int, v: int) -> None:
        self.cfg.pulse_ms[i] = v
        self._dirty = True

    def arm_clicked(self) -> None:
        box = QtWidgets.QMessageBox(self)
        box.setIcon(QtWidgets.QMessageBox.Icon.Warning)
        box.setWindowTitle("Aktivovat pyro kanály")
        box.setText("Aktivovat pyro kanály na 60 sekund?")
        box.setInformativeText(
            "Po aktivaci lze podržením tlačítka odpálit kterýkoli kanál.\n\n"
            "• Nikdo není u rakety ani u motoru.\n"
            "• RBF je vytažený jen pro tento test.\n"
            "• Esc nebo „Deaktivovat“ vše okamžitě zajistí.")
        yes = box.addButton("Aktivovat", QtWidgets.QMessageBox.ButtonRole.AcceptRole)
        no = box.addButton("Zrušit", QtWidgets.QMessageBox.ButtonRole.RejectRole)
        box.setDefaultButton(no)
        box.exec()
        if box.clickedButton() is yes:
            self.link.send("ARM")
            self.log("> ARM")

    def disarm(self) -> None:
        if self.link.is_open:
            self.link.send("DISARM")
            self.log("> DISARM")

    def fire(self, ch: int) -> None:
        if not self.tlm.get("arm"):
            self.log("# odpálení zamítnuto: pyro není aktivované")
            return
        ms = self.pulse_spins[ch - 1].value()
        self.link.send(f"FIRE {ch} {ms}")
        self.log(f"> FIRE {ch} {ms}")

    # --------------------------------------------------------------- config
    def _build_config(self) -> QtWidgets.QWidget:
        card = Card("Nastavení", "Přiřazení kanálů a limity serv")
        for txt, fn, kind in (("Uložit", self.save, "primary"), ("Uložit jako…", self.save_as, ""),
                              ("Načíst…", self.load_from, ""), ("Výchozí", self.reset_defaults, ""),
                              ("Poslat limity do desky", self.send_limits, "")):
            b = button(txt, kind)
            b.clicked.connect(fn)
            card.head.addWidget(b)
        self.cfg_warn = label("", "muted")
        card.lay.addWidget(self.cfg_warn)
        t = QtWidgets.QTableWidget(len(self.cfg.servos), len(CFG_COLS))
        t.setHorizontalHeaderLabels(CFG_COLS)
        t.verticalHeader().setVisible(False)
        t.setSelectionMode(QtWidgets.QAbstractItemView.SelectionMode.NoSelection)
        t.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        t.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.Stretch)
        t.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeMode.ResizeToContents)
        t.verticalHeader().setDefaultSectionSize(30)
        t.setMinimumHeight(30 * len(self.cfg.servos) + 30)
        t.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.table = t
        card.lay.addWidget(t)
        card.lay.addWidget(label("Min/Max jsou tvrdé limity: deska je dostane a žádný pulz za ně nepustí. "
                                 "Střed = poloha tlačítek „Střed“. Reverz otočí smysl táhla a TVC plochy. "
                                 "µs / ° slouží jen pro zobrazení úhlu.", "subtle"))
        self._fill_config_table()
        return card

    def _spin(self, lo, hi, val, step=10, dbl=False) -> QtWidgets.QAbstractSpinBox:
        sp = QtWidgets.QDoubleSpinBox() if dbl else QtWidgets.QSpinBox()
        sp.setRange(lo, hi)
        sp.setSingleStep(step)
        sp.setValue(val)
        sp.setKeyboardTracking(False)
        sp.setButtonSymbols(QtWidgets.QAbstractSpinBox.ButtonSymbols.NoButtons)
        sp.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        sp.setObjectName("cell")
        return sp

    def _fill_config_table(self) -> None:
        t = self.table
        for r, s in enumerate(self.cfg.servos):
            name = QtWidgets.QTableWidgetItem(s.name)
            name.setFlags(Qt.ItemFlag.ItemIsEnabled)
            t.setItem(r, 0, name)
            ch = self._spin(0, N_CH - 1, s.ch, 1)
            mn = self._spin(ABS_US_MIN, ABS_US_MAX, s.min_us)
            ce = self._spin(ABS_US_MIN, ABS_US_MAX, s.center_us)
            mx = self._spin(ABS_US_MIN, ABS_US_MAX, s.max_us)
            rv = QtWidgets.QCheckBox()
            rv.setChecked(s.reverse)
            rv_w = QtWidgets.QWidget()
            rv_l = QtWidgets.QHBoxLayout(rv_w)
            rv_l.setContentsMargins(0, 0, 0, 0)
            rv_l.addWidget(rv, alignment=Qt.AlignmentFlag.AlignCenter)
            upd = self._spin(0.5, 100.0, s.us_per_deg, 0.5, dbl=True)
            upd.setDecimals(1)
            widgets = {"ch": ch, "min_us": mn, "center_us": ce, "max_us": mx, "us_per_deg": upd}
            for col, w in enumerate((ch, mn, ce, mx), start=1):
                t.setCellWidget(r, col, w)
            t.setCellWidget(r, 5, rv_w)
            t.setCellWidget(r, 6, upd)
            if s.lock_us is not None:
                lk = self._spin(ABS_US_MIN, ABS_US_MAX, s.lock_us)
                ul = self._spin(ABS_US_MIN, ABS_US_MAX, s.unlock_us)
                widgets["lock_us"], widgets["unlock_us"] = lk, ul
                t.setCellWidget(r, 7, lk)
                t.setCellWidget(r, 8, ul)
            else:
                for col in (7, 8):
                    it = QtWidgets.QTableWidgetItem("—")
                    it.setFlags(Qt.ItemFlag.ItemIsEnabled)
                    it.setForeground(QtGui.QColor(C["subtle"]))
                    it.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                    t.setItem(r, col, it)
            out = QtWidgets.QTableWidgetItem("—")
            out.setFlags(Qt.ItemFlag.ItemIsEnabled)
            out.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            t.setItem(r, 9, out)
            for f, w in widgets.items():
                w.valueChanged.connect(lambda v, s_=s, f_=f: self._cfg_edit(s_, f_, v))
            rv.toggled.connect(lambda v, s_=s: self._cfg_edit(s_, "reverse", v))

    def _cfg_edit(self, s: ServoCfg, f: str, v) -> None:
        old_ch = s.ch
        setattr(s, f, v)
        before = asdict(s)
        validate_servo(s)
        if asdict(s) != before:  # validation moved something: show the real values
            QtCore.QTimer.singleShot(0, self._fill_config_table)
        self._dirty = True
        if f == "ch" and old_ch != s.ch and self.link.is_open:
            if not any(o.ch == old_ch for o in self.cfg.servos):
                self.link.send(f"OFF {old_ch}")  # nobody uses the old channel now
        self._cfg_changed(s, send_pos=True)

    def _cfg_changed(self, s: ServoCfg, send_pos: bool) -> None:
        ax = self.axes.get(s.key)
        if ax:
            ax.apply_cfg()
            ax.set_us(ax.value())  # re-clamp into the new window
        if s.key.startswith("tvc"):
            self._sync_tvc_pad()
        if s.key == "cover":
            self._update_cover_state()
        if self.link.is_open:
            self._send_lim(s)
            if send_pos and ax and self._channel_live(s.ch):
                self.pending[s.ch] = ax.value()
        self._check_conflicts()

    def _channel_live(self, ch: int) -> bool:
        us = self.tlm.get("us") or []
        return ch < len(us) and us[ch] > 0

    def _check_conflicts(self) -> None:
        by_ch: dict[int, list[str]] = {}
        for s in self.cfg.servos:
            by_ch.setdefault(s.ch, []).append(s.name)
        dup = {ch: names for ch, names in by_ch.items() if len(names) > 1}
        for r, s in enumerate(self.cfg.servos):
            w = self.table.cellWidget(r, 1)
            if w is not None:
                w.setStyleSheet(f"border: 1px solid {C['danger']}; color: {C['danger']};" if s.ch in dup else "")
        if dup:
            txt = "; ".join(f"kanál {ch}: {', '.join(n)}" for ch, n in dup.items())
            self.cfg_warn.setText(f"Pozor, jeden kanál pro víc serv — {txt}. Na desku jde vždy poslední příkaz.")
            self.cfg_warn.setStyleSheet(f"color: {C['danger']};")
        else:
            self.cfg_warn.setText("")

    def _send_lim(self, s: ServoCfg) -> None:
        # Two servos on one channel (a mistake, flagged in the table) get the
        # INTERSECTION of their windows, so a mis-assigned servo can never
        # widen the travel of the one that is really plugged in.
        on_ch = [o for o in self.cfg.servos if o.ch == s.ch]
        lo = max(o.min_us for o in on_ch)
        hi = min(o.max_us for o in on_ch)
        if lo >= hi:
            self.log(f"# kanál {s.ch}: rozsahy serv se nepřekrývají, limit neposlán")
            return
        self.link.send(f"LIM {s.ch} {lo} {hi}")

    def send_limits(self) -> None:
        if not self.link.is_open:
            return
        for ch in sorted({s.ch for s in self.cfg.servos}):
            self._send_lim(next(s for s in self.cfg.servos if s.ch == ch))
        self.log("> LIM pro všechny použité kanály")

    # ------------------------------------------------------------------ log
    def _build_log(self) -> QtWidgets.QWidget:
        card = Card("Deska", "Záznam")
        b = button("Vymazat")
        card.head.addWidget(b)
        self.log_view = QtWidgets.QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(800)
        b.clicked.connect(self.log_view.clear)
        card.lay.addWidget(self.log_view, 1)
        self.cmd_line = QtWidgets.QLineEdit()
        self.cmd_line.setPlaceholderText("Příkaz pro desku, např. STATUS  (Enter)")
        self.cmd_line.returnPressed.connect(self._send_raw)
        card.lay.addWidget(self.cmd_line)
        return card

    def _send_raw(self) -> None:
        txt = self.cmd_line.text().strip()
        if txt:
            self.link.send(txt)
            self.log(f"> {txt}")
            self.cmd_line.clear()

    def log(self, text: str) -> None:
        self.log_view.appendPlainText(f"{time.strftime('%H:%M:%S')}  {text}")

    # ------------------------------------------------------------ link glue
    def refresh_ports(self) -> None:
        cur = self.port_box.currentData() or self.cfg.port
        self.port_box.clear()
        if serial is not None:
            for p in sorted(serial.tools.list_ports.comports(), key=lambda p: p.device):
                desc = p.description if p.description and p.description != "n/a" else ""
                self.port_box.addItem(f"{p.device}  {desc}".strip(), p.device)
        self.port_box.addItem("Simulace (bez hardwaru)", "DEMO")
        idx = self.port_box.findData(cur)
        if idx >= 0:
            self.port_box.setCurrentIndex(idx)

    def connect_toggle(self) -> None:
        if self.link.is_open:
            self.disarm()
            self.link.close()
        else:
            port = self.port_box.currentData()
            if port:
                self.link.open(port)

    def on_link_state(self, up: bool, msg: str) -> None:
        self.conn_btn.setText("Odpojit" if up else "Připojit")
        self.conn_btn.setProperty("kind", "" if up else "primary")
        repolish(self.conn_btn)
        self.log(f"# {'připojeno: ' + msg if up else msg}")
        self.tlm = {}
        self.tlm_time = 0.0
        if up:
            self.cfg.port = self.port_box.currentData() or ""
            self.link.send("HELLO")
            self.link.send(f"PWM {self.pwm_box.currentData()}")
            self.send_limits()
            self.log("# serva zůstávají vypnutá, dokud jimi nepohnete nebo nedáte „Vše na střed“")
        self._update_controls_enabled()
        self._refresh_status()

    def _pwm_changed(self) -> None:
        hz = self.pwm_box.currentData()
        self.cfg.pwm_hz = hz
        self._dirty = True
        if self.link.is_open:
            self.link.send(f"PWM {hz}")

    def on_line(self, line: str) -> None:
        if line.startswith("T "):
            try:
                self.tlm = json.loads(line[2:])
                self.tlm_time = time.monotonic()
            except ValueError:
                return
            self._apply_tlm()
            return
        if line:
            self.log(line)

    def on_axis(self, key: str, us: int) -> None:
        s = self.cfg.servo(key)
        if self.gang.isChecked() and key.startswith("fin"):
            n = s.us_to_norm(us)
            for k in ("fin1", "fin2", "fin3", "fin4"):
                if k != key:
                    o = self.cfg.servo(k)
                    self.axes[k].set_us(o.norm_to_us(n))
                    self.pending[o.ch] = self.axes[k].value()
        if key.startswith("tvc") and not self.tvc_timer.isActive():
            self._sync_tvc_pad()
        if key == "cover":
            self._update_cover_state()
        self.pending[s.ch] = us

    def _flush(self) -> None:
        if not self.pending or not self.link.is_open:
            self.pending.clear()
            return
        for ch, us in self.pending.items():
            self.link.send(f"SET {ch} {us}")
        self.pending.clear()

    def _heartbeat(self) -> None:
        if self.link.is_open:
            self.link.send("HB")

    def all_center(self) -> None:
        for s in self.cfg.servos:
            self.axes[s.key].set_us(s.center_us)
            self.pending[s.ch] = s.center_us
        self._sync_tvc_pad()
        self._update_cover_state()

    def all_off(self) -> None:
        if self.link.is_open:
            self.link.send("OFF ALL")
            self.log("> OFF ALL")

    # ------------------------------------------------------------ status UI
    def _fresh(self) -> bool:
        return self.link.is_open and time.monotonic() - self.tlm_time < 0.6

    def _apply_tlm(self) -> None:
        t = self.tlm
        us = t.get("us") or []
        for r, s in enumerate(self.cfg.servos):
            v = us[s.ch] if s.ch < len(us) else None
            self.axes[s.key].set_board(v)
            it = self.table.item(r, 9)
            if it is not None:
                txt = "—" if v is None else ("vyp." if v == 0 else f"{v} µs")
                if it.text() != txt:
                    it.setText(txt)
        cont = t.get("cont") or [0, 0, 0]
        cmask, fired, firing = t.get("cmask", 0), t.get("fired", 0), t.get("firing", 0)
        nav = bool(t.get("nav"))
        for i, row in enumerate(self.pyro_rows):
            if not nav:
                set_chip(row["cont"], "—", "")
                row["raw"].setText("ADC —")
            elif firing & (1 << i):
                set_chip(row["cont"], "PÁLÍ", "hot")
            elif cmask & (1 << i):
                set_chip(row["cont"], "Zapalovač OK", "ok")
            else:
                set_chip(row["cont"], "Rozpojeno", "")
            if nav:
                row["raw"].setText(f"ADC {cont[i]}  ·  práh {t.get('thr', '—')}")
            set_chip(row["fired"], "odpáleno" if fired & (1 << i) else "", "warn" if fired & (1 << i) else "")
            row["fired"].setVisible(bool(fired & (1 << i)))

    def _refresh_status(self) -> None:
        up = self.link.is_open
        fresh = self._fresh()
        t = self.tlm if fresh else {}
        if not up:
            set_chip(self.chip_board, "Deska: odpojeno", "")
        elif not fresh:
            set_chip(self.chip_board, "Deska: čekám na data", "warn")
        else:
            set_chip(self.chip_board, "Deska: OK", "ok")
        set_chip(self.chip_pca, "PCA9685: OK" if t.get("pca") else "PCA9685: —",
                 "ok" if t.get("pca") else ("bad" if fresh else ""))
        set_chip(self.chip_nav, "Navigation: OK" if t.get("nav") else "Navigation: —",
                 "ok" if t.get("nav") else ("bad" if fresh else ""))
        if fresh:
            set_chip(self.chip_rbf, "RBF: vytažen" if t.get("rbf") else "RBF: zasunut",
                     "warn" if t.get("rbf") else "ok")
            v = float(t.get("vbat", 0))
            set_chip(self.chip_bat, f"Baterie: {cz(v, 2, sign=False)} V",
                     "" if v < 0.5 else ("bad" if v < 7.0 else "ok"))
        else:
            set_chip(self.chip_rbf, "RBF: —", "")
            set_chip(self.chip_bat, "Baterie: —", "")

        armed = bool(t.get("arm"))
        if armed:
            left = float(t.get("arm_left", 0))
            set_chip(self.arm_chip, f"AKTIVNÍ · {cz(left, 0, sign=False)} s", "hot")
        else:
            set_chip(self.arm_chip, "NEAKTIVNÍ", "ok" if t.get("nav") else "")
        reasons = []
        if not up:
            reasons.append("Připojte Master.")
        elif not fresh:
            reasons.append("Deska neposílá data.")
        else:
            if not t.get("nav"):
                reasons.append("Navigation MCU neodpovídá – nahrajte do ní Examples/Pyro_Companion.")
            if not t.get("rbf"):
                reasons.append("RBF je zasunutý: aktivace není možná.")
        rej = int(t.get("rej", 0)) if fresh else 0
        if rej:
            reasons.append(f"Poslední příkaz odmítnut: {REJECT.get(rej, rej)}.")
        if armed:
            reasons.insert(0, "Pyro je AKTIVNÍ. Odpálení: podržte tlačítko kanálu.")
        self.pyro_note.setText(" ".join(reasons))
        can_arm = fresh and bool(t.get("nav")) and bool(t.get("rbf")) and not armed
        self.arm_btn.setEnabled(can_arm)
        self.disarm_btn.setEnabled(up)
        for row in self.pyro_rows:
            row["fire"].setEnabled(armed and fresh)
        self.setWindowTitle(APP_NAME + (" — AKTIVNÍ PYRO" if armed else "") + (" •" if self._dirty else ""))

    def _update_controls_enabled(self) -> None:
        up = self.link.is_open
        for w in (self.all_center_btn, self.all_off_btn):
            w.setEnabled(up)

    def closeEvent(self, e):
        self.disarm()
        if self._dirty:
            r = QtWidgets.QMessageBox.question(
                self, APP_NAME, "Uložit změny konfigurace?",
                QtWidgets.QMessageBox.StandardButton.Save | QtWidgets.QMessageBox.StandardButton.Discard
                | QtWidgets.QMessageBox.StandardButton.Cancel)
            if r == QtWidgets.QMessageBox.StandardButton.Cancel:
                e.ignore()
                return
            if r == QtWidgets.QMessageBox.StandardButton.Save:
                self.save()
        self.link.close()
        e.accept()


def pick_font(app: QtWidgets.QApplication) -> None:
    fams = set(QtGui.QFontDatabase.families())
    for name in ("Inter", "Inter Variable", "Segoe UI", "Helvetica Neue", "Noto Sans", "DejaVu Sans"):
        if name in fams:
            f = QtGui.QFont(name)
            f.setPixelSize(13)
            app.setFont(f)
            return


def main() -> int:
    ap = argparse.ArgumentParser(description=APP_NAME)
    ap.add_argument("--port", help="sériový port Masteru, např. COM5 nebo /dev/ttyACM0")
    ap.add_argument("--demo", action="store_true", help="simulovaná deska, bez hardwaru")
    ap.add_argument("--config", default=DEFAULT_CONFIG, help="soubor s konfigurací (JSON)")
    ap.add_argument("--screenshot", help=argparse.SUPPRESS)
    args = ap.parse_args()

    QtWidgets.QApplication.setHighDpiScaleFactorRoundingPolicy(
        Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)
    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName(APP_NAME)
    QtCore.QLocale.setDefault(QtCore.QLocale(QtCore.QLocale.Language.Czech, QtCore.QLocale.Country.CzechRepublic))
    pick_font(app)
    app.setStyleSheet(QSS)
    win = MainWindow(args.config, args.demo, args.port)
    win.show()
    if args.screenshot:
        def shot():
            win.grab().save(args.screenshot)
            win._dirty = False
            app.quit()
        QtCore.QTimer.singleShot(2500, shot)
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
