#!/usr/bin/env python3
# ==============================================================================
#  Ariadna V2 — Ground Station Visualiser
#
#  Space Carrots / Czech Rocket Challenge 2026
#  Vehicle: Daidalos, propulsive-landing rocket
#
#  Runs on a laptop, connected by USB to the ground station receiver
#  (Raspberry Pi Pico 2 + LoRa Ra-02). Parses the framed telemetry stream,
#  draws it, and records it.
#
#  ------------------------------------------------------------------------
#  WHAT CHANGED FROM THE V1 VISUALISER
#  ------------------------------------------------------------------------
#   * Framed parser. V1 read a fixed-size struct straight off the port with
#     no sync word, so a single dropped byte misaligned the stream
#     permanently — every subsequent packet decoded as garbage and the only
#     cure was restarting the app. This one hunts a sync word and checks a
#     CRC, so it self-heals on the next packet and never shows a corrupted
#     frame as if it were a reading.
#
#   * V2 telemetry: mission state machine, guidance phase, TVC deflection,
#     thrust clamp, thrust estimate, pyro continuity, per-board health.
#
#   * Subsystem and pyro strips. On a vehicle that fires an igniter in
#     flight, continuity is the pre-launch go/no-go, and it belongs on the
#     screen rather than in a log.
#
#  ------------------------------------------------------------------------
#  Install:
#      pip install pyserial PyQt6 PyQt6-WebEngine pyqtgraph haversine
#  Run:
#      python ground_visualizer.py
#
#  The packet layout below is pinned against the C definition by
#  ../../test/check_py_format.py. Run `cd ../../test && make pyfmt` after
#  touching either side.
# ==============================================================================

import sys
import os
import csv
import math
import time
import struct
import urllib.request
import concurrent.futures
from pathlib import Path
from datetime import datetime

import serial
import serial.tools.list_ports

from PyQt6 import QtWidgets, QtCore, QtGui
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtWebEngineCore import QWebEngineSettings
import pyqtgraph as pg
from haversine import haversine

# ==============================================================================
#  Configuration
# ==============================================================================
CACHE_DIR = os.path.join(os.getcwd(), "map_cache")

# Target pad location (Czech Rocket Challenge).
PAD_LAT = 49.7985411
PAD_LON = 16.6912772

# --- wire format --------------------------------------------------------------
# Must match ariadna_link.h. See the note at the top of the file.
SYNC0 = 0xA5
SYNC1 = 0x1D
PKT_TELEMETRY = 0x40
TLM_VERSION = 3

# Downlink rate the vehicle is configured for. Must match ARIADNA_TLM_HZ in
# ariadna_link.h — it is what the link-quality percentage is measured
# against, so a stale value here reads as a permanently bad link.
TLM_HZ = 10

#            t_ms lat lon | 11 x int16                | 13 x uint8      | rssi snr
GS_FORMAT = "<Iii" + "h" * 11 + "B" * 13 + "bb"
GS_SIZE = struct.calcsize(GS_FORMAT)
assert GS_SIZE == 49, f"packet layout drift: {GS_SIZE} != 49"

# ariadna_tlm_t.uwb, one byte of UWB constellation health. See the
# ARIADNA_TUWB_* block in ariadna_link.h for why it is worth the airtime.
UWB_MODE_NAMES = {0: "--", 1: "LOS", 2: "2D", 3: "3D"}

STATE_NAMES = {
    0: "BOOT", 1: "IDLE", 2: "ARMED", 3: "ASCENT", 4: "APOGEE", 5: "DESCENT",
    6: "GUIDANCE", 7: "LANDED", 8: "RECOVERY", 9: "ABORT", 10: "TEST", 11: "ERROR",
}

# dg_phase_t from guidance.h.
PHASE_NAMES = {0: "IDLE", 1: "COAST", 2: "BURN", 3: "DONE"}

# ariadna_tlm_t.health bits.
HEALTH_BITS = [
    (0, "POS"),    (1, "NAV"),   (2, "SOLN"),  (3, "FLASH"),
    (4, "LOG"),    (5, "SERVO"), (6, "GPS"),   (7, "POST"),
]

ERROR_NAMES = {
    0: "none", 1: "POST fail", 2: "position link", 3: "navigation link",
    4: "flash", 5: "servo bus", 6: "battery low", 7: "unknown",
}

# Google satellite tiles, cached for offline use at the field. Radius in km
# per zoom level — tight at high zoom so the download stays quick.
ZOOM_STRATEGY = {15: 2.0, 16: 1.5, 17: 1.0, 18: 0.5, 19: 0.4}


def crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT-FALSE. Bit-identical to ariadna_crc16() in the firmware."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def calculate_bearing(lat1, lon1, lat2, lon2):
    """Azimuth from one GPS point to another, degrees 0-360."""
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dlam = math.radians(lon2 - lon1)
    y = math.sin(dlam) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlam)
    return (math.degrees(math.atan2(y, x)) + 360) % 360


# ==============================================================================
#  Offline map downloader
# ==============================================================================
def deg2num(lat_deg, lon_deg, zoom):
    lat_rad = math.radians(lat_deg)
    n = 2.0 ** zoom
    xtile = int((lon_deg + 180.0) / 360.0 * n)
    ytile = int((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n)
    return xtile, ytile


class MapDownloader(QtCore.QThread):
    progress = QtCore.pyqtSignal(int, str)
    finished = QtCore.pyqtSignal()

    def __init__(self, lat, lon, strategy):
        super().__init__()
        self.lat, self.lon, self.strategy = lat, lon, strategy
        self.headers = {"User-Agent": "SpaceCarrots_Ariadna_Map/2.0"}

    def run(self):
        tiles = []
        for zoom, radius_km in self.strategy.items():
            dlat = radius_km / 111.0
            dlon = radius_km / (111.0 * math.cos(math.radians(self.lat)))
            xmin, ymax = deg2num(self.lat - dlat, self.lon - dlon, zoom)
            xmax, ymin = deg2num(self.lat + dlat, self.lon + dlon, zoom)
            for x in range(xmin, xmax + 1):
                for y in range(ymin, ymax + 1):
                    tiles.append((zoom, x, y))

        total = max(1, len(tiles))
        self.progress.emit(0, f"Scanning {total} satellite tiles...")

        def fetch(z, x, y):
            tile_dir = os.path.join(CACHE_DIR, "satellite", str(z), str(x))
            os.makedirs(tile_dir, exist_ok=True)
            path = os.path.join(tile_dir, f"{y}.png")
            if os.path.exists(path):
                return
            url = f"https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}"
            try:
                req = urllib.request.Request(url, headers=self.headers)
                with urllib.request.urlopen(req, timeout=3) as r, open(path, "wb") as f:
                    f.write(r.read())
            except Exception:
                pass  # a missing tile shows as a grey square, not a crash

        with concurrent.futures.ThreadPoolExecutor(max_workers=10) as ex:
            futures = []
            for idx, (z, x, y) in enumerate(tiles):
                futures.append(ex.submit(fetch, z, x, y))
                if idx % 10 == 0:
                    self.progress.emit(int(idx / total * 100), f"Downloading zoom {z}...")
            concurrent.futures.wait(futures)

        self.progress.emit(100, "Map cache ready.")
        self.finished.emit()


# ==============================================================================
#  Leaflet map
# ==============================================================================
LEAFLET_HTML = """
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>Daidalos Tracker</title>
    <style>
        body { padding: 0; margin: 0; background-color: #1e1e1e; }
        html, body, #map { height: 100%; width: 100%; background: #1e1e1e; }
        .custom-svg-icon { background: transparent; border: none; }
    </style>
    <script>
    // Leaflet is loaded from the local cache directory so the app works
    // with no internet at the launch site.
    </script>
    <link rel="stylesheet" href="LEAFLET_CSS_URI" />
    <script src="LEAFLET_JS_URI"></script>
</head>
<body>
    <div id="map"></div>
    <script>
        var map = L.map('map', {
            zoomControl: false, attributionControl: false,
            preferCanvas: true, zoomAnimation: true, fadeAnimation: true
        }).setView([PAD_LAT_V, PAD_LON_V], 18);

        var tileOptions = {
            maxZoom: 19, maxNativeZoom: 19, minZoom: 15, keepBuffer: 4,
            updateWhenZooming: false,
            errorTileUrl: 'data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7'
        };
        L.tileLayer('CACHE_URI/satellite/{z}/{x}/{y}.png', tileOptions).addTo(map);

        var padSvg = `<svg width="24" height="24" viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg">
            <circle cx="12" cy="12" r="10" stroke="#000" stroke-width="2" fill="rgba(255,255,255,0.5)"/>
            <path d="M 12 0 L 12 24 M 0 12 L 24 12" stroke="#000" stroke-width="2"/>
            <circle cx="12" cy="12" r="10" stroke="#00ff88" stroke-width="2" fill="none" stroke-dasharray="4,4"/>
            <circle cx="12" cy="12" r="2" fill="#000"/></svg>`;
        var padIcon = L.divIcon({ html: padSvg, className: 'custom-svg-icon',
                                  iconSize: [24,24], iconAnchor: [12,12] });
        var rocketIcon = L.divIcon({
            html: '<div style="background-color:#ff0000;width:14px;height:14px;border-radius:50%;border:2px solid white;box-shadow:0 0 4px rgba(0,0,0,0.8);"></div>',
            className: 'custom-svg-icon', iconSize: [14,14], iconAnchor: [7,7] });

        var padLat = PAD_LAT_V, padLon = PAD_LON_V, minRadiusDeg = 0.0010;
        var lastRocketPos = [padLat, padLon], lastViewUpdate = 0;
        var minPadBounds = L.latLngBounds(
            [[padLat + minRadiusDeg, padLon + minRadiusDeg],
             [padLat - minRadiusDeg, padLon - minRadiusDeg]]);

        L.marker([padLat, padLon], {icon: padIcon, zIndexOffset: 100}).addTo(map);
        var rocketMarker = L.marker([padLat, padLon], {icon: rocketIcon, zIndexOffset: 1000}).addTo(map);
        var trajectory = L.polyline([], {color:'#ff8c00', weight:3, opacity:0.9,
                                         lineCap:'round', lineJoin:'round'}).addTo(map);

        function updateRocketPosition(lat, lon) {
            var p = [lat, lon];
            lastRocketPos = p;
            rocketMarker.setLatLng(p);
            trajectory.addLatLng(p);
            var now = Date.now();
            if (now - lastViewUpdate > 500) { updateMapView(); lastViewUpdate = now; }
        }
        function clearTrajectory() { trajectory.setLatLngs([]); }
        function updateMapView() {
            var b = L.latLngBounds(minPadBounds);
            b.extend(lastRocketPos);
            map.fitBounds(b, {padding:[40,40], maxZoom:19, animate:true, duration:0.5});
        }
        setTimeout(updateMapView, 200);
    </script>
</body>
</html>
"""


class RocketMapWidget(QtWidgets.QWidget):
    def __init__(self, pad_lat=PAD_LAT, pad_lon=PAD_LON, parent=None):
        super().__init__(parent)
        self.pad_lat, self.pad_lon = pad_lat, pad_lon
        self.setFocusPolicy(QtCore.Qt.FocusPolicy.StrongFocus)
        self._init_ui()

    def _init_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self.browser = QWebEngineView(self)
        self.browser.page().setBackgroundColor(QtGui.QColor(30, 30, 30))
        s = self.browser.settings()
        s.setAttribute(QWebEngineSettings.WebAttribute.LocalContentCanAccessFileUrls, True)
        s.setAttribute(QWebEngineSettings.WebAttribute.LocalContentCanAccessRemoteUrls, True)
        layout.addWidget(self.browser)

        os.makedirs(CACHE_DIR, exist_ok=True)
        cache_uri = Path(CACHE_DIR).as_uri()

        # Leaflet itself is served from unpkg when online and from the cache
        # directory when it has been vendored there. Copy leaflet.js and
        # leaflet.css into map_cache/ before going to the field.
        css_local = os.path.join(CACHE_DIR, "leaflet.css")
        js_local = os.path.join(CACHE_DIR, "leaflet.js")
        css_uri = (Path(css_local).as_uri() if os.path.exists(css_local)
                   else "https://unpkg.com/leaflet@1.9.4/dist/leaflet.css")
        js_uri = (Path(js_local).as_uri() if os.path.exists(js_local)
                  else "https://unpkg.com/leaflet@1.9.4/dist/leaflet.js")

        html = (LEAFLET_HTML
                .replace("PAD_LAT_V", str(self.pad_lat))
                .replace("PAD_LON_V", str(self.pad_lon))
                .replace("CACHE_URI", cache_uri)
                .replace("LEAFLET_CSS_URI", css_uri)
                .replace("LEAFLET_JS_URI", js_uri))
        self.browser.setHtml(html, QtCore.QUrl.fromLocalFile(CACHE_DIR + "/"))

    def update_position(self, lat, lon):
        self.browser.page().runJavaScript(f"updateRocketPosition({lat:.7f}, {lon:.7f});")

    def clear_trajectory(self):
        self.browser.page().runJavaScript("clearTrajectory();")


# ==============================================================================
#  Serial reception
# ==============================================================================
class SerialWorker(QtCore.QThread):
    """
    Reads the framed telemetry stream from the ground station.

    The framer is the Python twin of ariadna_rx_byte(): hunt the sync word,
    read a length, verify a CRC. Anything that fails is counted and dropped
    — never displayed. A corrupted altitude looks exactly like a real one
    on screen, which is the whole reason the CRC is here.
    """

    data_received = QtCore.pyqtSignal(dict)
    link_stats = QtCore.pyqtSignal(int, int)      # good, bad
    connection_error = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.serial_port = None
        self.baud_rate = 115200
        self.is_running = False
        self.rx_good = 0
        self.rx_bad = 0

    def connect_port(self, port_name):
        try:
            self.serial_port = serial.Serial(port_name, self.baud_rate, timeout=0.1)
            self.serial_port.reset_input_buffer()
            self.is_running = True
            self.rx_good = self.rx_bad = 0
            self.start()
            return True
        except Exception as e:
            self.connection_error.emit(str(e))
            return False

    def disconnect_port(self):
        self.is_running = False
        self.wait(500)
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()

    def send_line(self, text):
        """Text command up to the receiver (test mode toggles)."""
        try:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.write((text + "\n").encode())
        except Exception:
            pass

    def run(self):
        buf = bytearray()
        while self.is_running and self.serial_port and self.serial_port.is_open:
            try:
                chunk = self.serial_port.read(256)
                if not chunk:
                    continue
                buf.extend(chunk)

                while True:
                    # --- hunt the sync word ---------------------------------
                    i = buf.find(bytes([SYNC0, SYNC1]))
                    if i < 0:
                        # Keep a trailing byte: it may be the first half of a
                        # sync pair split across two reads.
                        del buf[:max(0, len(buf) - 1)]
                        break
                    if i:
                        del buf[:i]
                    if len(buf) < 4:
                        break

                    ptype, plen = buf[2], buf[3]
                    total = 4 + plen + 2
                    if plen > 96:
                        del buf[:2]          # bogus header, resync past it
                        self.rx_bad += 1
                        continue
                    if len(buf) < total:
                        break

                    payload = bytes(buf[4:4 + plen])
                    got = buf[4 + plen] | (buf[5 + plen] << 8)
                    if crc16_ccitt(bytes(buf[2:4 + plen])) != got:
                        del buf[:2]
                        self.rx_bad += 1
                        continue

                    del buf[:total]
                    self.rx_good += 1
                    if ptype == PKT_TELEMETRY and plen == GS_SIZE:
                        self._emit(payload)

                self.link_stats.emit(self.rx_good, self.rx_bad)

            except Exception:
                # A disconnected adapter raises here; stop cleanly rather
                # than spinning on a dead handle.
                self.is_running = False

    def _emit(self, payload):
        u = struct.unpack(GS_FORMAT, payload)
        (t_ms, lat, lon,
         alt_dm, vz_dms, vh_dms, acc_cg, roll_d, pitch_d, yaw_d, temp_d,
         tvc_x_cd, tvc_y_cd, thrust_n,
         version, state, nav_phase, clamp_pct, vbat_cv, curr_da,
         hdop_x10, sats, pyro, health, flags, exp, uwb,
         rssi, snr) = u

        if version != TLM_VERSION:
            self.rx_bad += 1
            return

        self.data_received.emit({
            "timeMs": t_ms,
            "LAT": lat / 1e7, "LON": lon / 1e7,
            "ALT": alt_dm / 10.0,
            "VZ": vz_dms / 10.0,
            "HVEL": vh_dms / 10.0,
            "TOTVEL": math.hypot(vz_dms / 10.0, vh_dms / 10.0),
            "ACCEL": acc_cg / 100.0,
            "ROLL": roll_d / 10.0, "PITCH": pitch_d / 10.0, "YAW": yaw_d / 10.0,
            "TEMP": temp_d / 10.0,
            "TVC_X": tvc_x_cd / 100.0, "TVC_Y": tvc_y_cd / 100.0,
            "THRUST": float(thrust_n),
            "STATE": state, "PHASE": nav_phase,
            "CLAMP": clamp_pct,
            "BATTERY": (vbat_cv / 100.0) + 2.5,
            "CURRENT": curr_da / 10.0,
            "HDOP": hdop_x10 / 10.0, "SATS": sats,
            "PYRO_CONT": pyro & 0x0F, "PYRO_FIRED": (pyro >> 4) & 0x0F,
            "HEALTH": health,
            "FLAGS": flags,
            "ERROR": (flags >> 5) & 0x07,
            "EXP": exp,
            "UWB_N": uwb & 0x0F,
            "UWB_MODE": (uwb >> 4) & 0x03,
            "UWB_AIDED": bool(uwb & 0x40),
            "UWB_ALIVE": bool(uwb & 0x80),
            "RSSI": rssi, "SNR": snr,
            "sys_time": time.time(),
        })


# ==============================================================================
#  Widgets
# ==============================================================================
class TelemetryCard(QtWidgets.QFrame):
    def __init__(self, title, value="--"):
        super().__init__()
        self.setObjectName("TelemetryCard")
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(6, 3, 6, 3)
        lay.setSpacing(1)

        self.title_label = QtWidgets.QLabel(title)
        self.title_label.setObjectName("CardTitle")
        self.value_label = QtWidgets.QLabel(value)
        self.value_label.setObjectName("CardValue")
        lay.addWidget(self.title_label)
        lay.addWidget(self.value_label)
        self.set_status("#444444")

    def set_value(self, text):
        self.value_label.setText(text)

    def set_status(self, color):
        self.setStyleSheet(f"""
        QFrame#TelemetryCard {{
            background-color: #1e1e1e;
            border: 2px solid {color};
            border-radius: 6px;
        }}
        QLabel#CardTitle {{ color: #aaaaaa; font-size: 9px;  font-weight: bold; }}
        QLabel#CardValue {{ color: white;   font-size: 15px; font-weight: bold; }}
        """)


class IndicatorStrip(QtWidgets.QFrame):
    """
    A row of named lights.

    Used for subsystem health and pyro continuity — the two things that are
    binary, that matter before launch, and that a number cannot express.
    """

    def __init__(self, title, labels):
        super().__init__()
        self.setObjectName("TelemetryCard")
        self.setStyleSheet("""
        QFrame#TelemetryCard { background-color: #1e1e1e; border: 2px solid #444;
                               border-radius: 6px; }
        """)
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(6, 3, 6, 3)
        outer.setSpacing(2)

        cap = QtWidgets.QLabel(title)
        cap.setStyleSheet("color:#aaaaaa; font-size:9px; font-weight:bold;")
        outer.addWidget(cap)

        row = QtWidgets.QHBoxLayout()
        row.setSpacing(3)
        outer.addLayout(row)

        self.lamps = {}
        for name in labels:
            lamp = QtWidgets.QLabel(name)
            lamp.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
            lamp.setFixedHeight(18)
            self.lamps[name] = lamp
            row.addWidget(lamp)
            self.set(name, False)

    def set(self, name, on, warn=False):
        lamp = self.lamps.get(name)
        if lamp is None:
            return
        if warn:
            bg, fg = "#ffaa00", "#000000"
        elif on:
            bg, fg = "#00ff88", "#000000"
        else:
            bg, fg = "#3a1010", "#886666"
        lamp.setStyleSheet(
            f"background-color:{bg}; color:{fg}; font-size:9px; font-weight:bold;"
            f"border-radius:3px; padding:2px 4px;")


class TVCWidget(QtWidgets.QWidget):
    """
    Nozzle deflection, drawn as a dot in the gimbal's travel circle, with a
    thrust-clamp bar beside it.

    Two numbers would do, but the failure this is meant to catch is the
    nozzle pinned against a travel limit — and a dot on the edge of a circle
    shows that instantly, where a number does not.
    """

    MAX_DEG = 10.0      # DG_SERVO_MAX_DEG

    def __init__(self):
        super().__init__()
        self.x = 0.0
        self.y = 0.0
        self.clamp = 0
        self.phase = 0
        self.setMinimumHeight(150)

    def update_tvc(self, x, y, clamp, phase):
        self.x, self.y, self.clamp, self.phase = x, y, clamp, phase
        self.update()

    def paintEvent(self, _event):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        rect = self.rect()
        p.fillRect(rect, QtGui.QColor("#121212"))

        bar_w = 26
        size = min(rect.width() - bar_w - 24, rect.height() - 26)
        r = max(20, size // 2)
        cx = 12 + r
        cy = rect.height() // 2

        # travel limit and half-travel rings
        p.setBrush(QtCore.Qt.BrushStyle.NoBrush)
        p.setPen(QtGui.QPen(QtGui.QColor(90, 90, 90), 2))
        p.drawEllipse(QtCore.QPointF(cx, cy), float(r), float(r))
        p.setPen(QtGui.QPen(QtGui.QColor(60, 60, 60), 1, QtCore.Qt.PenStyle.DashLine))
        p.drawEllipse(QtCore.QPointF(cx, cy), r / 2.0, r / 2.0)
        p.drawLine(cx - r, cy, cx + r, cy)
        p.drawLine(cx, cy - r, cx, cy + r)

        # the commanded deflection
        mag = math.hypot(self.x, self.y)
        px = cx + (self.x / self.MAX_DEG) * r
        py = cy - (self.y / self.MAX_DEG) * r
        colour = QtGui.QColor("#ff2222") if mag > self.MAX_DEG * 0.9 else QtGui.QColor("#00ff88")
        p.setPen(QtGui.QPen(colour, 2))
        p.drawLine(QtCore.QPointF(cx, cy), QtCore.QPointF(px, py))
        p.setBrush(colour)
        p.setPen(QtCore.Qt.PenStyle.NoPen)
        p.drawEllipse(QtCore.QPointF(px, py), 5.0, 5.0)

        p.setPen(QtGui.QColor(150, 150, 150))
        p.setFont(QtGui.QFont("Consolas", 8, QtGui.QFont.Weight.Bold))
        p.drawText(QtCore.QRectF(0, rect.height() - 16, cx * 2, 14),
                   QtCore.Qt.AlignmentFlag.AlignCenter,
                   f"TVC {self.x:+.2f} / {self.y:+.2f}")

        # thrust clamp
        bx = cx + r + 18
        by0, by1 = 16, rect.height() - 22
        p.setPen(QtGui.QPen(QtGui.QColor(90, 90, 90), 2))
        p.setBrush(QtCore.Qt.BrushStyle.NoBrush)
        p.drawRect(bx, by0, bar_w, by1 - by0)

        frac = max(0.0, min(1.0, self.clamp / 100.0))
        fh = int((by1 - by0) * frac)
        p.setPen(QtCore.Qt.PenStyle.NoPen)
        p.setBrush(QtGui.QColor("#ff8c00") if self.phase == 2 else QtGui.QColor("#555555"))
        p.drawRect(bx + 2, by1 - fh + 1, bar_w - 4, max(0, fh - 2))

        p.setPen(QtGui.QColor(200, 200, 200))
        p.drawText(QtCore.QRectF(bx - 8, rect.height() - 16, bar_w + 16, 14),
                   QtCore.Qt.AlignmentFlag.AlignCenter, f"{self.clamp}%")


class NavBallWidget(QtWidgets.QWidget):
    """Native PyQt6 3D navball. Carried over from the V1 visualiser."""

    def __init__(self):
        super().__init__()
        self.pitch = self.roll = self.yaw = 0.0
        self.speed = 0.0
        self.setMinimumSize(260, 280)

        self.COLOR_SKY = QtGui.QColor(50, 130, 168)
        self.COLOR_GROUND = QtGui.QColor(168, 92, 50)
        self.COLOR_UI_BG = QtGui.QColor(30, 30, 30, 230)
        self.COLOR_GREEN = QtGui.QColor(50, 255, 50)
        self.COLOR_WHITE = QtGui.QColor(255, 255, 255)
        self.COLOR_BLACK = QtGui.QColor(0, 0, 0)
        self.COLOR_GREY = QtGui.QColor(150, 150, 150)
        self.COLOR_ORANGE = QtGui.QColor(255, 140, 0)
        self.COLOR_CHEVRON = QtGui.QColor(255, 180, 0)

    def update_attitude(self, pitch, roll, yaw=0.0, speed=0.0):
        self.pitch, self.roll = pitch, roll
        self.yaw = yaw % 360
        self.speed = speed
        self.update()

    def _project_3d(self, lat, lon):
        lat_r, lon_r = math.radians(lat), math.radians(lon)
        x = math.cos(lat_r) * math.sin(lon_r)
        y = math.sin(lat_r)
        z = math.cos(lat_r) * math.cos(lon_r)

        yaw_r = math.radians(-self.yaw)
        cy, sy = math.cos(yaw_r), math.sin(yaw_r)
        x1, y1, z1 = x * cy + z * sy, y, -x * sy + z * cy

        pitch_r = math.radians(90 - self.pitch)
        cp, sp = math.cos(pitch_r), math.sin(pitch_r)
        x2, y2, z2 = x1, y1 * cp - z1 * sp, y1 * sp + z1 * cp

        roll_r = math.radians(-self.roll)
        cr, sr = math.cos(roll_r), math.sin(roll_r)
        return x2 * cr - y2 * sr, x2 * sr + y2 * cr, z2

    def _draw_3d_line(self, painter, cx, cy, radius, pts, color, width=1):
        painter.setPen(QtGui.QPen(color, width))
        for i in range(len(pts) - 1):
            p1, p2 = pts[i], pts[i + 1]
            if p1[2] > 0 and p2[2] > 0:
                painter.drawLine(
                    QtCore.QPointF(float(cx + p1[0] * radius), float(cy - p1[1] * radius)),
                    QtCore.QPointF(float(cx + p2[0] * radius), float(cy - p2[1] * radius)))

    def _draw_oriented_text(self, painter, text, lat, lon, cx, cy, radius):
        px, py, pz = self._project_3d(lat, lon)
        if pz < 0.15:
            return
        pl = self._project_3d(lat, lon - 2)
        pr = self._project_3d(lat, lon + 2)
        angle = math.degrees(math.atan2((pr[1] - pl[1]) * -radius,
                                        (pr[0] - pl[0]) * radius))

        painter.save()
        painter.translate(cx + px * radius, cy - py * radius)
        painter.rotate(angle)
        painter.setFont(QtGui.QFont("Consolas", 8, QtGui.QFont.Weight.Bold))
        m = painter.fontMetrics()
        tw, th = m.horizontalAdvance(text), m.height()
        painter.setPen(self.COLOR_BLACK)
        for ox, oy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            painter.drawText(QtCore.QRectF(-tw / 2 + ox, -th / 2 + oy, tw, th),
                             QtCore.Qt.AlignmentFlag.AlignCenter, text)
        painter.setPen(self.COLOR_WHITE)
        painter.drawText(QtCore.QRectF(-tw / 2, -th / 2, tw, th),
                         QtCore.Qt.AlignmentFlag.AlignCenter, text)
        painter.restore()

    def paintEvent(self, _event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        rect = self.rect()
        cx, cy = rect.center().x(), rect.center().y()
        radius = min(cx, cy) - 46

        painter.setPen(QtCore.Qt.PenStyle.NoPen)
        painter.setBrush(self.COLOR_SKY)
        painter.drawEllipse(QtCore.QPointF(float(cx), float(cy)), float(radius), float(radius))

        path = QtGui.QPainterPath()
        path.addEllipse(QtCore.QPointF(float(cx), float(cy)), float(radius), float(radius))
        painter.setClipPath(path)

        painter.setBrush(self.COLOR_GROUND)
        painter.setPen(QtGui.QPen(self.COLOR_GROUND, 1))
        for lat in range(-90, 0, 5):
            for lon in range(0, 360, 5):
                q = [self._project_3d(lat, lon), self._project_3d(lat + 5, lon),
                     self._project_3d(lat + 5, lon + 5), self._project_3d(lat, lon + 5)]
                if sum(p[2] for p in q) > 0:
                    painter.drawPolygon(QtGui.QPolygonF(
                        [QtCore.QPointF(float(cx + p[0] * radius), float(cy - p[1] * radius))
                         for p in q]))

        painter.setClipping(False)
        painter.setBrush(QtCore.Qt.BrushStyle.NoBrush)
        painter.setPen(QtGui.QPen(QtGui.QColor(90, 90, 90), 16))
        painter.drawEllipse(QtCore.QPointF(float(cx), float(cy)),
                            float(radius + 8), float(radius + 8))
        painter.setPen(QtGui.QPen(QtGui.QColor(30, 30, 30), 4))
        painter.drawEllipse(QtCore.QPointF(float(cx), float(cy)), float(radius), float(radius))
        painter.setClipPath(path)
        painter.setClipping(True)

        for lon in range(0, 360, 45):
            self._draw_3d_line(painter, cx, cy, radius,
                               [self._project_3d(lat, lon) for lat in range(-90, 91, 5)],
                               self.COLOR_GREY, 1)
        self._draw_3d_line(painter, cx, cy, radius,
                           [self._project_3d(0, lon) for lon in range(0, 365, 5)],
                           self.COLOR_ORANGE, 3)

        for lon in range(0, 360, 10):
            self._draw_3d_line(painter, cx, cy, radius,
                               [self._project_3d(-2, lon), self._project_3d(2, lon)],
                               self.COLOR_WHITE, 2)
            if lon % 30 == 0:
                txt = {0: "N", 90: "E", 180: "S", 270: "W"}.get(lon, f"{lon:03d}")
                self._draw_oriented_text(painter, txt, -5, lon, cx, cy, radius)

        for center_lon in (0, 90, 180, 270):
            for lat in range(-80, 90, 10):
                if lat == 0:
                    continue
                cos_lat = max(math.cos(math.radians(lat)), 0.05)
                base = 12 if lat % 20 == 0 else 6
                span = min(base / cos_lat, 40)
                step = max(1, int(span / 5))
                self._draw_3d_line(
                    painter, cx, cy, radius,
                    [self._project_3d(lat, lon)
                     for lon in range(int(center_lon - span), int(center_lon + span) + 1, step)],
                    self.COLOR_WHITE, 2)
                off = 6.0 / cos_lat
                self._draw_oriented_text(painter, str(abs(lat)), lat,
                                         center_lon - span - off, cx, cy, radius)
                self._draw_oriented_text(painter, str(abs(lat)), lat,
                                         center_lon + span + off, cx, cy, radius)

        painter.setClipping(False)

        chevron = [QtCore.QPointF(float(cx - 32), float(cy)),
                   QtCore.QPointF(float(cx - 11), float(cy)),
                   QtCore.QPointF(float(cx), float(cy + 16)),
                   QtCore.QPointF(float(cx + 11), float(cy)),
                   QtCore.QPointF(float(cx + 32), float(cy))]
        painter.setPen(QtGui.QPen(self.COLOR_CHEVRON, 4))
        painter.setBrush(QtCore.Qt.BrushStyle.NoBrush)
        painter.drawPolyline(QtGui.QPolygonF(chevron))
        painter.setBrush(self.COLOR_CHEVRON)
        painter.drawEllipse(QtCore.QPointF(float(cx), float(cy)), 4.0, 4.0)

        f_small = QtGui.QFont("Consolas", 8, QtGui.QFont.Weight.Bold)
        f_large = QtGui.QFont("Consolas", 13, QtGui.QFont.Weight.Bold)

        sr = QtCore.QRectF(cx - 66, cy - radius - 42, 132, 38)
        painter.setBrush(self.COLOR_UI_BG)
        painter.setPen(QtGui.QPen(QtGui.QColor(120, 120, 120), 2))
        painter.drawRoundedRect(sr, 5, 5)
        painter.setPen(self.COLOR_GREEN)
        painter.setFont(f_small)
        painter.drawText(QtCore.QRectF(cx - 66, cy - radius - 39, 132, 14),
                         QtCore.Qt.AlignmentFlag.AlignHCenter, "Surface")
        painter.setFont(f_large)
        painter.drawText(QtCore.QRectF(cx - 66, cy - radius - 26, 132, 20),
                         QtCore.Qt.AlignmentFlag.AlignHCenter, f"{self.speed:.1f} m/s")

        hr = QtCore.QRectF(cx - 48, cy + radius + 5, 96, 32)
        painter.setBrush(self.COLOR_UI_BG)
        painter.setPen(QtGui.QPen(QtGui.QColor(120, 120, 120), 2))
        painter.drawRoundedRect(hr, 5, 5)
        painter.setPen(self.COLOR_GREEN)
        painter.setFont(f_large)
        painter.drawText(hr, QtCore.Qt.AlignmentFlag.AlignCenter, f"AZI {int(self.yaw):03d}°")


class AltimeterWidget(QtWidgets.QWidget):
    """
    Altitude tape, with the guidance handover altitude marked.

    The 85 m line is not decoration: it is the altitude at which the
    Navigation MCU takes authority, and watching the marker cross it is how
    you know the handover happened.
    """

    GUIDANCE_ENTRY = 85.0

    def __init__(self):
        super().__init__()
        self.altitude = 0.0
        self.max_alt = 500.0
        self.setMinimumWidth(135)

    def update_altitude(self, altitude):
        self.altitude = altitude
        if self.altitude > self.max_alt - 50:
            self.max_alt += 200
        self.update()

    def paintEvent(self, _event):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        rect = self.rect()
        p.fillRect(rect, QtGui.QColor("#121212"))

        top, bottom = 20, 40
        height = rect.height() - top - bottom
        x_axis = 42

        p.setPen(QtGui.QPen(QtGui.QColor("white"), 2))
        p.drawLine(x_axis, top, x_axis, top + height)

        p.setPen(QtGui.QColor(150, 150, 150))
        p.setFont(QtGui.QFont("Consolas", 10, QtGui.QFont.Weight.Bold))
        p.drawText(QtCore.QRectF(0, rect.height() - 25, rect.width(), 20),
                   QtCore.Qt.AlignmentFlag.AlignCenter, "ALTITUDE")

        p.setFont(QtGui.QFont("Consolas", 10, QtGui.QFont.Weight.Bold))
        p.setPen(QtGui.QColor(255, 255, 255))
        for alt in range(0, int(self.max_alt) + 1, 100):
            y = top + height - int(alt / self.max_alt * height)
            p.drawLine(x_axis - 5, y, x_axis + 5, y)
            p.drawText(QtCore.QRectF(0, y - 10, x_axis - 8, 20),
                       QtCore.Qt.AlignmentFlag.AlignRight | QtCore.Qt.AlignmentFlag.AlignVCenter,
                       f"{alt}")

        # guidance handover line
        gy = top + height - int(self.GUIDANCE_ENTRY / self.max_alt * height)
        p.setPen(QtGui.QPen(QtGui.QColor("#00ff88"), 1, QtCore.Qt.PenStyle.DashLine))
        p.drawLine(x_axis, gy, rect.width(), gy)
        p.setPen(QtGui.QColor("#00ff88"))
        p.setFont(QtGui.QFont("Consolas", 7, QtGui.QFont.Weight.Bold))
        p.drawText(x_axis + 6, gy - 3, "GUIDANCE 85")

        y = top + height - int(max(0.0, self.altitude) / self.max_alt * height)
        p.setBrush(QtGui.QColor("#ff5500"))
        p.setPen(QtCore.Qt.PenStyle.NoPen)
        p.drawPolygon(QtGui.QPolygonF([
            QtCore.QPointF(float(x_axis + 5), float(y)),
            QtCore.QPointF(float(x_axis + 15), float(y - 6)),
            QtCore.QPointF(float(x_axis + 15), float(y + 6))]))
        p.setPen(QtGui.QColor(255, 255, 255))
        p.setFont(QtGui.QFont("Consolas", 10, QtGui.QFont.Weight.Bold))
        p.drawText(int(x_axis + 20), int(y + 5), f"{self.altitude:.0f} m")


# ==============================================================================
#  Main window
# ==============================================================================
class GroundStation(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.packet_count = 0
        self.data_alt, self.data_accel, self.data_vel = [], [], []
        self.data_tvc_x, self.data_tvc_y = [], []

        self.boot_time = time.time()
        self.last_packet_time = time.time()
        self.rx_timestamps = []
        self.is_test_mode = False
        self.last_state = None

        self.is_recording = False
        self.csv_file = None
        self.csv_writer = None
        self.last_flush_time = 0.0

        self.sim_lat, self.sim_lon = PAD_LAT, PAD_LON
        self.test_t = 0.0

        self.setWindowTitle("Space Carrots — Daidalos / Ariadna V2 Ground Station")
        self.resize(1440, 860)

        self.progress_dialog = QtWidgets.QProgressDialog(
            "Caching offline maps...", "Skip", 0, 100, self)
        self.progress_dialog.setWindowTitle("Pre-flight setup")
        self.progress_dialog.setWindowModality(QtCore.Qt.WindowModality.WindowModal)
        self.progress_dialog.setMinimumDuration(0)

        self.downloader = MapDownloader(PAD_LAT, PAD_LON, ZOOM_STRATEGY)
        self.downloader.progress.connect(self.update_progress)
        self.downloader.finished.connect(self.launch_interface)
        self.downloader.start()

    # ---------------------------------------------------------------- setup
    def update_progress(self, val, text):
        self.progress_dialog.setValue(val)
        self.progress_dialog.setLabelText(text)
        if self.progress_dialog.wasCanceled():
            self.downloader.terminate()
            self.launch_interface()

    def launch_interface(self):
        if hasattr(self, "_launched"):
            return
        self._launched = True
        self.progress_dialog.close()

        self.serial_thread = SerialWorker()
        self.serial_thread.data_received.connect(self.process_telemetry)
        self.serial_thread.link_stats.connect(self.update_link_stats)

        self.setup_ui()

        self.watchdog_timer = QtCore.QTimer()
        self.watchdog_timer.timeout.connect(self.check_watchdog)
        self.watchdog_timer.start(100)

        self.test_data_timer = QtCore.QTimer()
        self.test_data_timer.timeout.connect(self.generate_test_data)

    def setup_ui(self):
        self.setStyleSheet("""
        QWidget { background-color: #121212; color: white; font-family: Consolas; }
        QComboBox { background-color: #1e1e1e; border: 1px solid #444; padding: 5px; }
        QPushButton { background-color: #ff5500; color: white; border: none;
                      padding: 6px 15px; font-weight: bold; border-radius: 4px; }
        QPushButton:hover { background-color: #ff7733; }
        """)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)

        # ---------------- toolbar ----------------
        bar = QtWidgets.QHBoxLayout()
        root.addLayout(bar)

        title = QtWidgets.QLabel("DAIDALOS · ARIADNA V2")
        title.setStyleSheet("font-size: 19px; font-weight: bold; color: #ff5500;")
        bar.addWidget(title)

        self.state_banner = QtWidgets.QLabel("— NO DATA —")
        self.state_banner.setStyleSheet(
            "font-size: 17px; font-weight: bold; color: white;"
            "background-color: #333; padding: 5px 18px; border-radius: 5px;")
        bar.addWidget(self.state_banner)

        self.test_mode_label = QtWidgets.QLabel("⚠️ TEST MODE ⚠️")
        self.test_mode_label.setStyleSheet(
            "font-size: 16px; font-weight: bold; color: yellow;"
            "background-color: #aa0000; padding: 5px 15px; border-radius: 5px;")
        self.test_mode_label.setVisible(False)
        bar.addWidget(self.test_mode_label)

        self.los_label = QtWidgets.QLabel("⚠️ LOSS OF SIGNAL ⚠️")
        self.los_label.setStyleSheet(
            "font-size: 16px; font-weight: bold; color: white;"
            "background-color: #ff0000; padding: 5px 15px; border-radius: 5px;")
        self.los_label.setVisible(False)
        bar.addWidget(self.los_label)

        bar.addStretch()

        self.record_btn = QtWidgets.QPushButton("⏺ RECORD")
        self.record_btn.setStyleSheet("background-color: #555555; color: white;")
        self.record_btn.clicked.connect(self.toggle_recording)
        bar.addWidget(self.record_btn)

        self.port_combo = QtWidgets.QComboBox()
        self.refresh_ports()
        bar.addWidget(self.port_combo)

        refresh = QtWidgets.QPushButton("Refresh")
        refresh.clicked.connect(self.refresh_ports)
        bar.addWidget(refresh)

        self.connect_btn = QtWidgets.QPushButton("CONNECT")
        self.connect_btn.clicked.connect(self.toggle_connection)
        bar.addWidget(self.connect_btn)

        # ---------------- body ----------------
        body = QtWidgets.QHBoxLayout()
        root.addLayout(body, stretch=1)

        # ---- left: cards, indicators, navball ----
        left = QtWidgets.QVBoxLayout()
        body.addLayout(left, stretch=1)

        grid = QtWidgets.QGridLayout()
        grid.setSpacing(5)
        left.addLayout(grid, stretch=0)

        self.cards = {}
        card_names = [
            "ALTITUDE", "MISSION STATE",
            "VERTICAL SPEED", "HORIZ SPEED",
            "ACCELERATION", "GUIDANCE PHASE",
            "THRUST EST", "THRUST CLAMP",
            "BATTERY", "CURRENT",
            "TEMPERATURE", "SATELLITES",
            "HDOP", "LORA SIGNAL",
            "LINK QUALITY", "ERROR",
            # The UWB constellation. Two cards rather than one because the
            # two facts fail independently: the anchors can be alive and
            # the filter can still be rejecting every fix they produce,
            # which is what a survey error looks like from the ground.
            "UWB ANCHORS", "UWB FIX",
        ]
        for i, name in enumerate(card_names):
            card = TelemetryCard(name)
            self.cards[name] = card
            grid.addWidget(card, i // 2, i % 2)

        self.health_strip = IndicatorStrip("SUBSYSTEMS", [n for _, n in HEALTH_BITS])
        left.addWidget(self.health_strip)

        self.pyro_strip = IndicatorStrip("PYRO  (continuity / fired)",
                                         ["C1", "C2", "C3", "F1", "F2", "F3",
                                          "ARM", "RBF"])
        left.addWidget(self.pyro_strip)

        self.navball = NavBallWidget()
        left.addWidget(self.navball, stretch=1)

        # ---- centre: graphs, TVC, map ----
        centre = QtWidgets.QVBoxLayout()
        body.addLayout(centre, stretch=3)

        graphs = QtWidgets.QHBoxLayout()
        centre.addLayout(graphs, stretch=1)

        pg.setConfigOption("background", "#1b1b1b")
        pg.setConfigOption("foreground", "w")

        self.alt_plot = pg.PlotWidget(title="Altitude (m)")
        self.accel_plot = pg.PlotWidget(title="Acceleration (g)")
        self.vel_plot = pg.PlotWidget(title="Vertical speed (m/s)")
        self.tvc_plot = pg.PlotWidget(title="TVC command (deg)")

        self.alt_curve = self.alt_plot.plot(pen=pg.mkPen("#ff5500", width=2))
        self.accel_curve = self.accel_plot.plot(pen=pg.mkPen("g", width=2))
        self.vel_curve = self.vel_plot.plot(pen=pg.mkPen("c", width=2))
        self.tvc_plot.addLegend(offset=(-10, 10))
        self.tvc_x_curve = self.tvc_plot.plot(pen=pg.mkPen("#ffaa00", width=2), name="X")
        self.tvc_y_curve = self.tvc_plot.plot(pen=pg.mkPen("#00aaff", width=2), name="Y")

        for plot in (self.alt_plot, self.accel_plot, self.vel_plot, self.tvc_plot):
            plot.showGrid(x=True, y=True, alpha=0.4)
            graphs.addWidget(plot)
        # Altitude and acceleration cannot go negative; vertical speed and
        # TVC very much can, so they keep a symmetric axis.
        self.alt_plot.getPlotItem().setLimits(yMin=0)
        self.accel_plot.getPlotItem().setLimits(yMin=0)

        lower = QtWidgets.QHBoxLayout()
        centre.addLayout(lower, stretch=2)

        self.map_widget = RocketMapWidget(PAD_LAT, PAD_LON)
        lower.addWidget(self.map_widget, stretch=3)

        self.tvc_widget = TVCWidget()
        lower.addWidget(self.tvc_widget, stretch=1)

        # ---- right: altimeter ----
        self.altimeter = AltimeterWidget()
        body.addWidget(self.altimeter, stretch=0)

        self.status = self.statusBar()
        self.status.showMessage("DISCONNECTED")

    # ------------------------------------------------------------ recording
    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key.Key_R:
            self.toggle_recording()
        super().keyPressEvent(event)

    def toggle_recording(self):
        if not self.is_recording:
            filename = datetime.now().strftime("%Y_%m_%d_%H_%M_ariadna.csv")
            self.csv_file = open(filename, "a", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            if os.stat(filename).st_size == 0:
                self.csv_writer.writerow([
                    "sys_time", "t_ms", "state", "phase", "lat", "lon", "alt",
                    "vz", "hvel", "accel", "roll", "pitch", "yaw", "temp",
                    "tvc_x", "tvc_y", "clamp", "thrust", "battery", "current",
                    "hdop", "sats", "pyro_cont", "pyro_fired", "health",
                    "flags", "error", "rssi", "snr",
                    "uwb_n", "uwb_mode", "uwb_aided", "uwb_alive"])
            self.is_recording = True
            self.record_btn.setText("⏹ STOP REC")
            self.record_btn.setStyleSheet("background-color: #ff0000; color: white;")
            self.last_flush_time = time.time()
            self.status.showMessage(f"RECORDING TO {filename}")
        else:
            self.is_recording = False
            if self.csv_file:
                self.csv_file.flush()
                os.fsync(self.csv_file.fileno())
                self.csv_file.close()
                self.csv_file = None
            self.csv_writer = None
            self.record_btn.setText("⏺ RECORD")
            self.record_btn.setStyleSheet("background-color: #555555; color: white;")
            self.status.showMessage("RECORDING STOPPED")

    # ----------------------------------------------------------- connection
    def refresh_ports(self):
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports:
            self.port_combo.addItem(p.device)
        if not ports:
            self.port_combo.addItem("No serial ports found")

    def toggle_connection(self):
        if self.serial_thread.is_running:
            self.serial_thread.disconnect_port()
            self.connect_btn.setText("CONNECT")
            self.connect_btn.setStyleSheet("background-color: #ff5500;")
            self.status.showMessage("DISCONNECTED")
            self.last_packet_time = time.time()
            self.los_label.setVisible(False)
        else:
            port = self.port_combo.currentText()
            if port and "No serial" not in port:
                if self.serial_thread.connect_port(port):
                    self.connect_btn.setText("DISCONNECT")
                    self.connect_btn.setStyleSheet("background-color: #aa0000;")
                    self.status.showMessage(f"CONNECTED TO {port}")
                    self.last_packet_time = time.time()
                    self.rx_timestamps.clear()
                    self.enable_test_mode(False)

    def update_link_stats(self, good, bad):
        self._link_good, self._link_bad = good, bad

    # ------------------------------------------------------------ watchdog
    def check_watchdog(self):
        now = time.time()
        self.rx_timestamps = [t for t in self.rx_timestamps if now - t <= 3.0]

        if not self.is_test_mode:
            # Packet delivery ratio against the vehicle's nominal downlink
            # rate. Anything under about 80 % on the pad means the link will
            # not survive the flight.
            expected = 3.0 * TLM_HZ     # 3 s window
            pdr = min(100.0, len(self.rx_timestamps) / expected * 100.0)
            self.cards["LINK QUALITY"].set_value(f"{pdr:.0f} %")
            self.cards["LINK QUALITY"].set_status(
                "#ff2222" if pdr < 50 else "#ffaa00" if pdr < 90 else "#00ff88")

        if getattr(self.serial_thread, "is_running", False):
            self.los_label.setVisible(now - self.last_packet_time > 3.0)
        else:
            self.los_label.setVisible(False)
            if now - self.boot_time > 5.0 and not self.is_test_mode:
                self.enable_test_mode(True)

    def enable_test_mode(self, enabled):
        if self.is_test_mode == enabled:
            return
        self.is_test_mode = enabled
        self.test_mode_label.setVisible(enabled)
        if enabled:
            self.test_data_timer.start(50)
            self.status.showMessage("NO SIGNAL — LOCAL SIMULATION")
            self.sim_lat, self.sim_lon = PAD_LAT, PAD_LON
            self.test_t = 0.0
        else:
            self.test_data_timer.stop()
            self.test_t = 0.0

    # ------------------------------------------------------- simulated data
    @staticmethod
    def tri(t, lo, hi, period):
        c = (t % period) / period
        return lo + (hi - lo) * (c * 2) if c < 0.5 else hi - (hi - lo) * ((c - 0.5) * 2)

    def generate_test_data(self):
        """
        Local simulation, for developing the UI with no hardware at all.

        Distinct from the ground station's own test mode: that one exercises
        the radio and the USB framing, this one exercises only the display.
        """
        self.test_t += 0.05
        t = self.test_t
        cyc = t % 40.0

        if cyc < 4:
            state, phase, alt, vz = 2, 0, 0.0, 0.0
        elif cyc < 12:
            state, phase, alt, vz = 3, 0, (cyc - 4) * 60.0, 60.0
        elif cyc < 14:
            state, phase, alt, vz = 4, 0, 480.0, 0.0
        elif cyc < 26:
            state, phase, alt, vz = 5, 1, 480.0 - (cyc - 14) * 33.0, -33.0
        elif cyc < 28:
            state, phase, alt, vz = 6, 1, 84.0 - (cyc - 26) * 30.0, -30.0
        elif cyc < 30:
            state, phase, alt, vz = 6, 2, 24.0 - (cyc - 28) * 12.0, -6.0
        elif cyc < 34:
            state, phase, alt, vz = 7, 3, 0.0, 0.0
        else:
            state, phase, alt, vz = 8, 3, 0.0, 0.0

        h_speed = 4.0 + math.sin(t) * 3.0
        azim = (t * 20) % 360
        d = h_speed * 0.05
        R = 6371000.0
        self.sim_lat += math.degrees(d * math.cos(math.radians(azim)) / R)
        self.sim_lon += math.degrees(d * math.sin(math.radians(azim))
                                     / (R * math.cos(math.radians(PAD_LAT))))
        if haversine((self.sim_lat, self.sim_lon), (PAD_LAT, PAD_LON)) * 1000 > 300:
            self.sim_lat, self.sim_lon = PAD_LAT, PAD_LON

        burn = (phase == 2)
        self.process_telemetry({
            "timeMs": int(t * 1000),
            "LAT": self.sim_lat, "LON": self.sim_lon,
            "ALT": max(0.0, alt), "VZ": vz, "HVEL": h_speed,
            "TOTVEL": math.hypot(vz, h_speed),
            "ACCEL": 4.5 if burn else 1.0,
            "ROLL": self.tri(t, -40, 40, 6),
            "PITCH": self.tri(t, -25, 25, 7),
            "YAW": azim,
            "TEMP": self.tri(t, -5, 35, 20),
            "TVC_X": math.sin(t * 6) * 3.5 if burn else 0.0,
            "TVC_Y": math.cos(t * 5) * 2.5 if burn else 0.0,
            "THRUST": 190 + math.sin(t * 4) * 35 if burn else 0.0,
            "STATE": state, "PHASE": phase,
            "CLAMP": int(55 + math.sin(t * 3) * 40) if burn else 60,
            "BATTERY": self.tri(t, 6.8, 8.3, 60),
            "CURRENT": 5.5 if burn else 1.2,
            "HDOP": self.tri(t, 0.6, 2.5, 12), "SATS": 13,
            "PYRO_CONT": 0x00 if phase >= 2 else 0x03,
            "PYRO_FIRED": 0x01 if phase >= 2 else 0x00,
            "HEALTH": 0xFF,
            "FLAGS": 0x0B if state >= 2 else 0x08,
            "ERROR": 0, "EXP": int(t / 5) % 8,
            "UWB_N": 4 if int(t) % 20 < 15 else 3,
            "UWB_MODE": 3 if alt > 5 else 2,
            "UWB_AIDED": True, "UWB_ALIVE": True,
            "RSSI": -60 + int(math.sin(t) * 25),
            "SNR": 9 + int(math.cos(t) * 4),
            "sys_time": time.time(),
        }, is_simulated=True)

    # ---------------------------------------------------------- the display
    def process_telemetry(self, d, is_simulated=False):
        if not is_simulated:
            self.last_packet_time = time.time()
            self.rx_timestamps.append(self.last_packet_time)
            self.enable_test_mode(False)

            if self.is_recording and self.csv_writer:
                self.csv_writer.writerow([
                    d["sys_time"], d["timeMs"], d["STATE"], d["PHASE"],
                    d["LAT"], d["LON"], d["ALT"], d["VZ"], d["HVEL"], d["ACCEL"],
                    d["ROLL"], d["PITCH"], d["YAW"], d["TEMP"],
                    d["TVC_X"], d["TVC_Y"], d["CLAMP"], d["THRUST"],
                    d["BATTERY"], d["CURRENT"], d["HDOP"], d["SATS"],
                    d["PYRO_CONT"], d["PYRO_FIRED"], d["HEALTH"],
                    d["FLAGS"], d["ERROR"], d["RSSI"], d["SNR"],
                    d["UWB_N"], d["UWB_MODE"],
                    int(d["UWB_AIDED"]), int(d["UWB_ALIVE"])])
                if time.time() - self.last_flush_time >= 10.0:
                    self.csv_file.flush()
                    os.fsync(self.csv_file.fileno())
                    self.last_flush_time = time.time()

        self.packet_count += 1

        # A new flight clears the old track rather than drawing a straight
        # line from wherever the last one ended.
        if d["STATE"] == 2 and self.last_state not in (None, 2):
            self.map_widget.clear_trajectory()
            self.data_alt.clear(); self.data_accel.clear(); self.data_vel.clear()
            self.data_tvc_x.clear(); self.data_tvc_y.clear()
        self.last_state = d["STATE"]

        for series, key in ((self.data_alt, "ALT"), (self.data_accel, "ACCEL"),
                            (self.data_vel, "VZ"), (self.data_tvc_x, "TVC_X"),
                            (self.data_tvc_y, "TVC_Y")):
            series.append(d[key])
            del series[:-400]

        self.alt_curve.setData(self.data_alt)
        self.accel_curve.setData(self.data_accel)
        self.vel_curve.setData(self.data_vel)
        self.tvc_x_curve.setData(self.data_tvc_x)
        self.tvc_y_curve.setData(self.data_tvc_y)

        self.alt_plot.setYRange(0, max(100, max(self.data_alt)))
        self.accel_plot.setYRange(0, max(2, max(self.data_accel)))
        self.vel_plot.setYRange(min(-10, min(self.data_vel)), max(10, max(self.data_vel)))
        self.tvc_plot.setYRange(-TVCWidget.MAX_DEG, TVCWidget.MAX_DEG)

        c = self.cards
        c["ALTITUDE"].set_value(f"{d['ALT']:.1f} m")
        c["MISSION STATE"].set_value(STATE_NAMES.get(d["STATE"], "?"))
        c["VERTICAL SPEED"].set_value(f"{d['VZ']:+.1f} m/s")
        c["HORIZ SPEED"].set_value(f"{d['HVEL']:.1f} m/s")
        c["ACCELERATION"].set_value(f"{d['ACCEL']:.2f} g")
        c["GUIDANCE PHASE"].set_value(PHASE_NAMES.get(d["PHASE"], "?"))
        c["THRUST EST"].set_value(f"{d['THRUST']:.0f} N")
        c["THRUST CLAMP"].set_value(f"{d['CLAMP']} %")
        c["BATTERY"].set_value(f"{d['BATTERY']:.2f} V")
        c["CURRENT"].set_value(f"{d['CURRENT']:.1f} A")
        c["TEMPERATURE"].set_value(f"{d['TEMP']:.1f} °C")
        c["SATELLITES"].set_value(f"{d['SATS']}")
        c["HDOP"].set_value(f"{d['HDOP']:.1f}")
        c["ERROR"].set_value(ERROR_NAMES.get(d["ERROR"], "?"))

        # UWB. "n/4 anchors" and what the filter is doing with them.
        c["UWB ANCHORS"].set_value(f"{d['UWB_N']} / 4")
        if not d["UWB_ALIVE"]:
            c["UWB FIX"].set_value("OFFLINE")
        elif d["UWB_AIDED"]:
            c["UWB FIX"].set_value(f"FUSED {UWB_MODE_NAMES.get(d['UWB_MODE'], '?')}")
        else:
            c["UWB FIX"].set_value("GATED OUT")

        c["LORA SIGNAL"].value_label.setTextFormat(QtCore.Qt.TextFormat.RichText)
        c["LORA SIGNAL"].set_value(
            f"<b>{d['RSSI']}</b><span style='font-size:10px;color:#aaa;'> dBm</span> "
            f"<b>{d['SNR']}</b><span style='font-size:10px;color:#aaa;'> dB</span>")

        # 2S lithium: 7.4 V nominal, 8.4 V full, 6.6 V empty.
        c["BATTERY"].set_status("#ff2222" if d["BATTERY"] < 7.0
                                else "#ffaa00" if d["BATTERY"] < 7.4 else "#00ff88")
        c["SATELLITES"].set_status("#ff2222" if d["SATS"] < 5 else "#00ff88")
        c["ERROR"].set_status("#ff2222" if d["ERROR"] else "#00ff88")
        c["HDOP"].set_status("#ff2222" if d["HDOP"] > 2.5
                             else "#ffaa00" if d["HDOP"] > 1.5 else "#00ff88")
        # Three anchors still navigates; two is a warning; one or none is
        # not a constellation any more.
        c["UWB ANCHORS"].set_status("#00ff88" if d["UWB_N"] >= 3
                                    else "#ffaa00" if d["UWB_N"] == 2 else "#ff2222")
        # Green only when the filter is ACCEPTING fixes. Alive but gated
        # out is amber, and it is the state worth acting on before launch.
        c["UWB FIX"].set_status("#00ff88" if d["UWB_AIDED"]
                                else "#ffaa00" if d["UWB_ALIVE"] else "#444444")
        c["LORA SIGNAL"].set_status("#00ff88" if d["RSSI"] > -90
                                    else "#ffaa00" if d["RSSI"] > -110 else "#ff2222")
        c["GUIDANCE PHASE"].set_status("#ff8c00" if d["PHASE"] == 2 else "#444444")
        c["ACCELERATION"].set_status("#ffaa00" if d["ACCEL"] > 8 else "#444444")

        # state banner
        name = STATE_NAMES.get(d["STATE"], "?")
        colour = {6: "#ff8c00", 7: "#00aa44", 8: "#0088cc",
                  9: "#cc0000", 11: "#cc0000"}.get(d["STATE"], "#333333")
        self.state_banner.setText(f"  {name}  ")
        self.state_banner.setStyleSheet(
            f"font-size:17px; font-weight:bold; color:white; background-color:{colour};"
            "padding:5px 18px; border-radius:5px;")

        # subsystem lights
        for bit, label in HEALTH_BITS:
            self.health_strip.set(label, bool(d["HEALTH"] & (1 << bit)))

        # pyro. Continuity going away after a channel has fired is expected,
        # so a fired channel with no continuity is green, not red.
        for i in range(3):
            cont = bool(d["PYRO_CONT"] & (1 << i))
            fired = bool(d["PYRO_FIRED"] & (1 << i))
            self.pyro_strip.set(f"C{i+1}", cont, warn=(not cont and not fired))
            self.pyro_strip.set(f"F{i+1}", fired)
        self.pyro_strip.set("ARM", bool(d["FLAGS"] & 0x01))
        self.pyro_strip.set("RBF", bool(d["FLAGS"] & 0x02))

        self.navball.update_attitude(d["PITCH"], d["ROLL"], d["YAW"], d["TOTVEL"])
        self.altimeter.update_altitude(d["ALT"])
        self.tvc_widget.update_tvc(d["TVC_X"], d["TVC_Y"], d["CLAMP"], d["PHASE"])

        if d["LAT"] or d["LON"]:
            self.map_widget.update_position(d["LAT"], d["LON"])

        if not self.is_recording:
            src = "SIMULATION" if is_simulated else self.port_combo.currentText()
            bad = getattr(self, "_link_bad", 0)
            self.status.showMessage(
                f"{src} | PACKETS: {self.packet_count} | DROPPED: {bad}")

    def closeEvent(self, event):
        if self.is_recording and self.csv_file:
            self.csv_file.flush()
            os.fsync(self.csv_file.fileno())
            self.csv_file.close()
        if hasattr(self, "serial_thread"):
            self.serial_thread.disconnect_port()
        super().closeEvent(event)


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    window = GroundStation()
    window.show()
    sys.exit(app.exec())
