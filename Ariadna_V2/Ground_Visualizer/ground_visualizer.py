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

# The 3-D rocket and the tilt bull's-eye are shared with the diagnostic
# console and live next to it.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "Diagnostics"))
from daidalos_widgets import (RocketView, TiltBubble, attitude_summary, pad_quaternion,
                              quat_from_smallest_three)

# ==============================================================================
#  Configuration
# ==============================================================================
# Map tiles and recordings live in <Documents>/Daidalos (or $DAIDALOS_DATA),
# shared with the diagnostic console - never in the code folder, which an
# update replaces. A map_cache/ left in the working directory by an older
# version is moved there once.
def _user_dir() -> str:
    base = os.environ.get("DAIDALOS_DATA", "")
    if not base:
        docs = QtCore.QStandardPaths.writableLocation(
            QtCore.QStandardPaths.StandardLocation.DocumentsLocation) or os.path.expanduser("~")
        base = os.path.join(docs, "Daidalos")
    os.makedirs(base, exist_ok=True)
    return base


USER_DIR = _user_dir()
CACHE_DIR = os.path.join(USER_DIR, "map_cache")
for _old in {os.path.join(os.getcwd(), "map_cache"),
             os.path.join(os.path.dirname(os.path.abspath(__file__)), "map_cache")}:
    if os.path.isdir(_old) and not os.path.exists(CACHE_DIR):
        try:
            import shutil
            shutil.move(_old, CACHE_DIR)
        except OSError:
            pass

# Target pad location (Czech Rocket Challenge). Only the starting point:
# the map follows the pad the ROCKET reports (its navigation origin, from the
# telemetry), remembers it in <user dir>/pad.json and downloads the map
# around it.
PAD_LAT = 49.7985411
PAD_LON = 16.6912772
PAD_FILE = os.path.join(USER_DIR, "pad.json")


def load_pad() -> tuple[float, float, str]:
    try:
        import json
        with open(PAD_FILE, encoding="utf-8") as f:
            d = json.load(f)
        return float(d["lat"]), float(d["lon"]), str(d.get("src", "saved"))
    except (OSError, ValueError, KeyError):
        return PAD_LAT, PAD_LON, "default"


def save_pad(lat: float, lon: float, src: str) -> None:
    try:
        import json
        with open(PAD_FILE, "w", encoding="utf-8") as f:
            json.dump({"lat": lat, "lon": lon, "src": src, "saved": datetime.now().isoformat()}, f)
    except OSError:
        pass

# --- wire format --------------------------------------------------------------
# Must match ariadna_link.h. See the note at the top of the file.
SYNC0 = 0xA5
SYNC1 = 0x1D
PKT_TELEMETRY = 0x40
TLM_VERSION = 4

# Downlink rate the vehicle is configured for. Must match ARIADNA_TLM_HZ in
# ariadna_link.h — it is what the link-quality percentage is measured
# against, so a stale value here reads as a permanently bad link.
TLM_HZ = 10

#            t_ms lat lon | 11 x int16 | 13 x uint8 | t_flight px py | rssi snr
GS_FORMAT = "<Iii" + "h" * 11 + "B" * 13 + "Bbb" + "bb"
GS_SIZE = struct.calcsize(GS_FORMAT)
assert GS_SIZE == 52, f"packet layout drift: {GS_SIZE} != 52"
T_UNKNOWN = -32768          # ARIADNA_TLM_T_UNKNOWN

# ariadna_tlm_t.uwb, one byte of UWB constellation health. See the
# ARIADNA_TUWB_* block in ariadna_link.h for why it is worth the airtime.
UWB_MODE_NAMES = {0: "--", 1: "LOS", 2: "2D", 3: "3D"}

STATE_NAMES = {
    0: "BOOT", 1: "IDLE", 2: "ARMED", 3: "ASCENT", 4: "APOGEE", 5: "DESCENT",
    6: "GUIDANCE", 7: "LANDED", 8: "RECOVERY", 9: "ABORT", 10: "TEST", 11: "ERROR",
}

# dg_phase_t from guidance.h.
PHASE_NAMES = {0: "IDLE", 1: "ASCENT", 2: "DESCENT", 3: "IGNITING", 4: "BURN", 5: "DONE"}
PHASE_BURN = 4

# ariadna_tlm_t.events: what the flight sequencer has done (ariadna_seq.h).
EVENT_BITS = [
    (0, "APOGEE"), (1, "COVER"), (2, "IGN 1"), (3, "THRUST"),
    (4, "IGN 2"),  (5, "LEGS"),  (6, "LIDAR"), (7, "DOWN"),
]

# ariadna_tlm_t.health bits.
HEALTH_BITS = [
    (0, "POS"),    (1, "NAV"),   (2, "SOLN"),  (3, "FLASH"),
    (4, "LOG"),    (5, "SERVO"), (6, "GPS"),   (7, "POST"),
]

ERROR_NAMES = {
    0: "none", 1: "POST fail", 2: "position link", 3: "navigation link",
    4: "flash", 5: "servo bus", 6: "battery low", 7: "params not synced",
}

# Bidirectional LoRa sync/Rampa/ACK packet (ARIADNA_PKT_GS_SYNC, fw 3.34).
PKT_GS_SYNC = 0x41
GS_SYNC_FORMAT = "<BBHIIHHiBBBBBBHbb"
GS_SYNC_SIZE = struct.calcsize(GS_SYNC_FORMAT)
assert GS_SYNC_SIZE == 30, f"sync packet layout drift: {GS_SYNC_SIZE} != 30"

RAMPA_STATE_NAMES = {
    0: "OFFLINE", 1: "IDLE", 2: "READY", 3: "COUNTDOWN",
    4: "COMMITTED", 5: "FIRED", 6: "ABORT", 7: "FAULT",
}
UL_RC_NAMES = {
    0: "—", 1: "OK", 2: "ODMÍTNUTO (LET)", 3: "ODMÍTNUTO (RAKETA)",
    4: "ODMÍTNUTO (RAMPA)", 5: "CHYBA TOKENU", 6: "ABORT POTVRZEN",
}

# Aerial imagery, cached for offline use at the field. The sharpest source
# for the Czech Republic is the CUZK orthophoto (Czech Office for Surveying,
# open data): 0.15 m per pixel at zoom 20, where Esri has no imagery at all
# and Google's is several times blurrier. Outside the Czech Republic CUZK
# answers 404 and the tile comes from Google instead. Radius in km per zoom
# level - tight at high zoom so the download stays around ~60 MB.
TILE_SOURCES = (
    ("CUZK ortofoto", "https://ags.cuzk.cz/arcgis1/rest/services/ORTOFOTO_WM/MapServer/tile/{z}/{y}/{x}"),
    ("Google", "https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}"),
)
TILE_DIR = "aerial"                    # under CACHE_DIR; the old "satellite" cache is not reused
MAX_NATIVE_ZOOM = 20
ZOOM_STRATEGY = {15: 4.0, 16: 2.5, 17: 1.5, 18: 1.0, 19: 0.6, 20: 0.35}


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
        self.headers = {"User-Agent": "Daidalos-GroundStation/2.1"}

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
            tile_dir = os.path.join(CACHE_DIR, TILE_DIR, str(z), str(x))
            os.makedirs(tile_dir, exist_ok=True)
            path = os.path.join(tile_dir, f"{y}.png")      # PNG or JPEG inside: browsers sniff it
            if os.path.exists(path):
                return
            for _name, tmpl in TILE_SOURCES:
                try:
                    req = urllib.request.Request(tmpl.format(z=z, x=x, y=y), headers=self.headers)
                    with urllib.request.urlopen(req, timeout=6) as r:
                        data = r.read()
                        ctype = r.headers.get("Content-Type", "")
                    if ctype.startswith("image/") and len(data) > 1000:
                        tmp = path + ".part"
                        with open(tmp, "wb") as f:
                            f.write(data)
                        os.replace(tmp, path)
                        return
                except Exception:
                    continue  # next source; a missing tile shows as the live layer or nothing

        with concurrent.futures.ThreadPoolExecutor(max_workers=8) as ex:
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

        var blank = 'data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7';
        var tileOptions = {
            maxZoom: 22, maxNativeZoom: MAX_NATIVE_ZOOM_V, minZoom: 13, keepBuffer: 4,
            updateWhenZooming: false, errorTileUrl: blank
        };
        // Live CUZK orthophoto underneath (when online), the offline cache on top.
        L.tileLayer('LIVE_TILE_URL', tileOptions).addTo(map);
        L.tileLayer('CACHE_URI/TILE_DIR_V/{z}/{x}/{y}.png', tileOptions).addTo(map);
        L.control.scale({imperial: false, position: 'bottomleft'}).addTo(map);
        var info = L.control({position: 'topright'});
        info.onAdd = function () {
            this._div = L.DomUtil.create('div');
            this._div.style.cssText = 'background:rgba(0,0,0,0.78);color:#fff;font:bold 13px Consolas,monospace;' +
                                      'padding:6px 9px;border-radius:6px;line-height:1.45;';
            return this._div;
        };
        info.addTo(map);
        var padSrc = 'PAD_SRC_V';

        var padSvg = `<svg width="24" height="24" viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg">
            <circle cx="12" cy="12" r="10" stroke="#000" stroke-width="2" fill="rgba(255,255,255,0.5)"/>
            <path d="M 12 0 L 12 24 M 0 12 L 24 12" stroke="#000" stroke-width="2"/>
            <circle cx="12" cy="12" r="10" stroke="#00ff88" stroke-width="2" fill="none" stroke-dasharray="4,4"/>
            <circle cx="12" cy="12" r="2" fill="#000"/></svg>`;
        var padIcon = L.divIcon({ html: padSvg, className: 'custom-svg-icon',
                                  iconSize: [24,24], iconAnchor: [12,12] });
        var rocketIcon = L.divIcon({
            html: '<div style="background-color:#ff0000;width:18px;height:18px;border-radius:50%;border:3px solid white;box-shadow:0 0 5px rgba(0,0,0,0.9);"></div>',
            className: 'custom-svg-icon', iconSize: [18,18], iconAnchor: [9,9] });

        var padLat = PAD_LAT_V, padLon = PAD_LON_V, minRadiusDeg = 0.0010;
        var lastRocketPos = [padLat, padLon], lastViewUpdate = 0;
        var minPadBounds = L.latLngBounds(
            [[padLat + minRadiusDeg, padLon + minRadiusDeg],
             [padLat - minRadiusDeg, padLon - minRadiusDeg]]);

        var padMarker = L.marker([padLat, padLon], {icon: padIcon, zIndexOffset: 100}).addTo(map);
        var rocketMarker = L.marker([padLat, padLon], {icon: rocketIcon, zIndexOffset: 1000}).addTo(map);
        var trajectory = L.polyline([], {color:'#ff8c00', weight:4, opacity:0.9,
                                         lineCap:'round', lineJoin:'round'}).addTo(map);

        function updateInfo() {
            var d = map.distance([padLat, padLon], lastRocketPos);
            info._div.innerHTML = 'PAD ' + padLat.toFixed(6) + ', ' + padLon.toFixed(6) +
                ' <span style="color:#9a9">(' + padSrc + ')</span><br>' +
                'ROCKET ' + lastRocketPos[0].toFixed(6) + ', ' + lastRocketPos[1].toFixed(6) +
                '<br>FROM PAD <b>' + d.toFixed(1) + ' m</b>';
        }
        function setPad(lat, lon, src) {
            padLat = lat; padLon = lon; padSrc = src;
            padMarker.setLatLng([lat, lon]);
            minPadBounds = L.latLngBounds([[lat + minRadiusDeg, lon + minRadiusDeg],
                                           [lat - minRadiusDeg, lon - minRadiusDeg]]);
            updateInfo();
            updateMapView();
        }
        function updateRocketPosition(lat, lon) {
            var p = [lat, lon];
            lastRocketPos = p;
            rocketMarker.setLatLng(p);
            trajectory.addLatLng(p);
            updateInfo();
            var now = Date.now();
            if (now - lastViewUpdate > 500) { updateMapView(); lastViewUpdate = now; }
        }
        function clearTrajectory() { trajectory.setLatLngs([]); }
        function updateMapView() {
            var b = L.latLngBounds(minPadBounds);
            b.extend(lastRocketPos);
            map.fitBounds(b, {padding:[40,40], maxZoom:21, animate:true, duration:0.5});
        }
        updateInfo();
        setTimeout(updateMapView, 200);
    </script>
</body>
</html>
"""


class RocketMapWidget(QtWidgets.QWidget):
    def __init__(self, pad_lat=PAD_LAT, pad_lon=PAD_LON, pad_src="default", parent=None):
        super().__init__(parent)
        self.pad_lat, self.pad_lon, self.pad_src = pad_lat, pad_lon, pad_src
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
                .replace("PAD_SRC_V", self.pad_src)
                .replace("MAX_NATIVE_ZOOM_V", str(MAX_NATIVE_ZOOM))
                .replace("TILE_DIR_V", TILE_DIR)
                .replace("LIVE_TILE_URL", TILE_SOURCES[0][1])
                .replace("CACHE_URI", cache_uri)
                .replace("LEAFLET_CSS_URI", css_uri)
                .replace("LEAFLET_JS_URI", js_uri))
        self.browser.setHtml(html, QtCore.QUrl.fromLocalFile(CACHE_DIR + "/"))

    def update_position(self, lat, lon):
        self.browser.page().runJavaScript(f"updateRocketPosition({lat:.7f}, {lon:.7f});")

    def set_pad(self, lat, lon, src):
        self.pad_lat, self.pad_lon, self.pad_src = lat, lon, src
        self.browser.page().runJavaScript(f"setPad({lat:.7f}, {lon:.7f}, '{src}');")

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
    sync_received = QtCore.pyqtSignal(dict)
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
                    elif ptype == PKT_GS_SYNC and plen == GS_SYNC_SIZE:
                        self._emit_sync(payload)

                self.link_stats.emit(self.rx_good, self.rx_bad)

            except Exception:
                # A disconnected adapter raises here; stop cleanly rather
                # than spinning on a dead handle.
                self.is_running = False

    def _emit_sync(self, payload):
        (magic, ver, _crc, rocket_t_ms, utc_ms, next_rx_in_ms, rx_dur_ms,
         t_launch_rem_ms, last_cmd_seq, last_cmd_rc, rampa_state, rampa_flags,
         rampa_fault, sync_flags, rampa_vbat_mv, rssi, snr) = struct.unpack(GS_SYNC_FORMAT, payload)
        if magic != 0xA6 or ver != 1:
            return
        self.sync_received.emit({
            "rocket_t_ms": rocket_t_ms,
            "utc_ms": None if utc_ms == 0xFFFFFFFF else utc_ms,
            "next_rx_in_ms": next_rx_in_ms,
            "rx_dur_ms": rx_dur_ms,
            "t_launch_rem_ms": t_launch_rem_ms,
            "last_cmd_seq": last_cmd_seq,
            "last_cmd_rc": last_cmd_rc,
            "rampa_state": rampa_state,
            "rampa_flags": rampa_flags,
            "rampa_fault": rampa_fault,
            "sync_flags": sync_flags,
            "rampa_vbat_v": rampa_vbat_mv / 1000.0,
            "rssi": rssi,
            "snr": snr,
            "rx_mono": time.monotonic(),
        })

    def _emit(self, payload):
        u = struct.unpack(GS_FORMAT, payload)
        (t_ms, lat, lon,
         alt_dm, vz_dms, vh_dms, acc_cg, qx, qy, qz,
         tvc_x_cdeg, tvc_y_cdeg, thrust_n, t_event_ds,
         version, state, nav_phase, clamp_pct, vbat_cv, curr_da,
         hdop_x10, sats, pyro, health, flags, events, uwb,
         t_flight_s, px_m, py_m,
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
            "QUAT": quat_from_smallest_three(qx, qy, qz),
            "T_EVENT": None if t_event_ds == T_UNKNOWN else t_event_ds / 10.0,
            "T_FLIGHT": t_flight_s,
            "PX": px_m, "PY": py_m,
            "EVENTS": events,
            "TVC_X": tvc_x_cdeg / 100.0, "TVC_Y": tvc_y_cdeg / 100.0,
            "THRUST": float(thrust_n),
            "STATE": state, "PHASE": nav_phase,
            "CLAMP": clamp_pct,
            "BATTERY": (vbat_cv / 50.0) + 5.0,
            "CURRENT": curr_da / 10.0,
            "HDOP": hdop_x10 / 10.0, "SATS": sats,
            "PYRO_CONT": pyro & 0x0F, "PYRO_FIRED": (pyro >> 4) & 0x0F,
            "HEALTH": health,
            "FLAGS": flags,
            "ERROR": (flags >> 5) & 0x07,
            "UWB_N": uwb & 0x0F,
            "UWB_MODE": (uwb >> 4) & 0x03,
            "UWB_AIDED": bool(uwb & 0x40),
            "UWB_ALIVE": bool(uwb & 0x80),
            "RSSI": rssi, "SNR": snr,
            "sys_time": time.time(),
        })


# ==============================================================================
#  Look: two themes
# ==============================================================================
# The station is used outdoors, on a laptop (Dell Vostro 7500: 15.6" FHD,
# ~300 nit matte panel, Windows at 125 % = 1536 x 864 logical px). In
# daylight a dark UI on a 300 nit panel drowns in reflections, so the default
# theme ("Venku") is light and high-contrast: near-black text on white, no
# pale-grey text anywhere, status shown by solid fills, not thin coloured
# borders. "Dark cockpit": a healthy value is a neutral tile with a green
# stripe; only a warning (solid amber) or an alarm (solid red) shouts.
# "Noc" is the dark theme, for dusk and night.
THEMES = {
    "light": dict(
        bg="#dfe4e9", panel="#ffffff", panel2="#eef1f4", border="#8995a1", dim="#6c7783",
        text="#070b0f", muted="#2f3943", accent="#c2410c",
        ok="#0d7a34", ok_text="#ffffff", warn="#f5ab00", warn_text="#000000",
        bad="#c8102e", bad_text="#ffffff", info="#1356b5", info_text="#ffffff",
        idle="#d2d8de", idle_text="#38424c",
        plot_bg="#ffffff", plot_fg="#070b0f", grid=0.35,
        s_alt="#c2410c", s_acc="#0d7a34", s_vz="#1356b5", s_tx="#d97706", s_ty="#1356b5"),
    "dark": dict(
        bg="#0a0e12", panel="#141a20", panel2="#1b232b", border="#3c4853", dim="#5c6b76",
        text="#f2f5f7", muted="#b1bcc6", accent="#ff7a1a",
        ok="#22c55e", ok_text="#03140a", warn="#fbbf24", warn_text="#000000",
        bad="#ef4444", bad_text="#ffffff", info="#3b82f6", info_text="#ffffff",
        idle="#25303a", idle_text="#9ba7b2",
        plot_bg="#141a20", plot_fg="#e7ecef", grid=0.22,
        s_alt="#ff7a1a", s_acc="#22c55e", s_vz="#38bdf8", s_tx="#fbbf24", s_ty="#38bdf8"),
}
THEME_LABEL = {"light": "VENKU", "dark": "NOC"}
T = dict(THEMES["light"])
T["key"] = "light"
UI_FONT = "Segoe UI"
MONO_FONT = "Consolas"


def C(key: str, alpha: int | None = None) -> QtGui.QColor:
    col = QtGui.QColor(T[key])
    if alpha is not None:
        col.setAlpha(alpha)
    return col


def ui_font(px: int, weight=QtGui.QFont.Weight.Bold, mono=False) -> QtGui.QFont:
    f = QtGui.QFont(MONO_FONT if mono else UI_FONT)
    f.setPixelSize(px)
    f.setWeight(weight)
    return f


def build_qss() -> str:
    t = T
    return f"""
    QWidget {{ background:{t['bg']}; color:{t['text']}; font-family:"{UI_FONT}"; font-size:13px; }}
    QMainWindow, QDialog, QMessageBox, QProgressDialog {{ background:{t['bg']}; }}
    QStatusBar {{ background:{t['panel']}; color:{t['text']}; border-top:1px solid {t['border']};
                  font-size:12px; font-weight:600; }}
    QStatusBar QLabel {{ background:transparent; }}
    QToolTip {{ background:{t['panel']}; color:{t['text']}; border:1px solid {t['border']};
                font-size:13px; padding:4px; }}
    QComboBox, QSpinBox {{ background:{t['panel']}; color:{t['text']}; border:2px solid {t['border']};
                border-radius:6px; padding:2px 8px; min-height:28px; font-size:14px; font-weight:700; }}
    QComboBox QAbstractItemView {{ background:{t['panel']}; color:{t['text']};
                selection-background-color:{t['info']}; selection-color:{t['info_text']}; }}
    QPushButton {{ background:{t['panel']}; color:{t['text']}; border:2px solid {t['border']};
                border-radius:6px; padding:3px 12px; min-height:28px; font-size:13px; font-weight:800; }}
    QPushButton:pressed {{ background:{t['idle']}; }}
    QPushButton[kind="primary"] {{ background:{t['accent']}; border-color:{t['accent']}; color:#ffffff; }}
    QPushButton[kind="info"] {{ background:{t['info']}; border-color:{t['info']}; color:{t['info_text']}; }}
    QPushButton[kind="warn"] {{ background:{t['warn']}; border-color:{t['warn']}; color:{t['warn_text']}; }}
    QPushButton[kind="danger"] {{ background:{t['bad']}; border-color:{t['bad']}; color:{t['bad_text']}; }}
    QPushButton[kind="abort"] {{ background:{t['bad']}; border:3px solid {t['text']}; color:{t['bad_text']};
                font-size:20px; font-weight:900; padding:2px 26px; min-height:36px; }}
    QPushButton:hover {{ border-color:{t['text']}; }}
    QPushButton:disabled {{ color:{t['dim']}; border-color:{t['idle']}; }}

    QFrame#Panel {{ background:{t['panel']}; border:1px solid {t['border']}; border-radius:8px; }}
    QFrame#Panel QLabel {{ background:transparent; }}
    QLabel#Caption {{ color:{t['muted']}; font-size:11px; font-weight:800; background:transparent; }}
    QLabel#Brand {{ color:{t['accent']}; font-size:20px; font-weight:900; }}
    QLabel#BrandSub {{ color:{t['muted']}; font-size:11px; font-weight:700; }}
    QLabel#Ack {{ color:{t['text']}; font-size:13px; font-weight:700; }}

    QFrame#Tile {{ background:{t['panel']}; border:1px solid {t['border']};
                border-left:8px solid {t['idle']}; border-radius:6px; }}
    QFrame#Tile[level="ok"] {{ border-left-color:{t['ok']}; }}
    QFrame#Tile[level="info"] {{ border-left-color:{t['info']}; }}
    QFrame#Tile[level="accent"] {{ border-left-color:{t['accent']}; }}
    QFrame#Tile[level="warn"] {{ background:{t['warn']}; border-color:{t['warn']}; }}
    QFrame#Tile[level="bad"] {{ background:{t['bad']}; border-color:{t['bad']}; }}
    QFrame#Tile QLabel {{ background:transparent; border:none; }}
    QLabel#TileTitle {{ color:{t['muted']}; font-size:11px; font-weight:800; }}
    QLabel#TileValue {{ color:{t['text']}; font-family:"{MONO_FONT}"; font-size:22px; font-weight:700; }}
    QLabel#TileValue[hero="true"] {{ font-size:34px; }}
    QLabel#TileSub {{ color:{t['muted']}; font-size:12px; font-weight:700; }}
    QFrame#Tile[level="warn"] QLabel {{ color:{t['warn_text']}; }}
    QFrame#Tile[level="bad"] QLabel {{ color:{t['bad_text']}; }}

    QLabel#Badge {{ background:{t['panel']}; color:{t['text']}; border:2px solid {t['border']};
                border-radius:6px; padding:2px 10px; min-height:26px; font-size:13px; font-weight:800; }}
    QLabel#Badge[level="ok"] {{ background:{t['ok']}; border-color:{t['ok']}; color:{t['ok_text']}; }}
    QLabel#Badge[level="info"] {{ border-color:{t['info']}; color:{t['info']}; }}
    QLabel#Badge[level="warn"] {{ background:{t['warn']}; border-color:{t['warn']}; color:{t['warn_text']}; }}
    QLabel#Badge[level="bad"] {{ background:{t['bad']}; border-color:{t['bad']}; color:{t['bad_text']}; }}
    QLabel#Badge[level="okline"] {{ border-color:{t['ok']}; color:{t['ok']}; }}

    QLabel#State {{ background:{t['idle']}; color:{t['idle_text']}; border-radius:8px;
                padding:0px 22px; min-height:44px; font-size:26px; font-weight:900; }}
    QLabel#State[level="ok"] {{ background:{t['ok']}; color:{t['ok_text']}; }}
    QLabel#State[level="info"] {{ background:{t['info']}; color:{t['info_text']}; }}
    QLabel#State[level="accent"] {{ background:{t['accent']}; color:#ffffff; }}
    QLabel#State[level="warn"] {{ background:{t['warn']}; color:{t['warn_text']}; }}
    QLabel#State[level="bad"] {{ background:{t['bad']}; color:{t['bad_text']}; }}
    QLabel#State[level="plain"] {{ background:{t['panel']}; color:{t['text']}; border:2px solid {t['border']}; }}

    QLabel#Clock {{ background:{t['panel']}; color:{t['text']}; border:2px solid {t['border']};
                border-radius:8px; padding:0px 14px; min-height:44px;
                font-family:"{MONO_FONT}"; font-size:28px; font-weight:700; }}
    QLabel#Clock[level="warn"] {{ background:{t['warn']}; border-color:{t['warn']}; color:{t['warn_text']}; }}
    QLabel#Clock[level="bad"] {{ background:{t['bad']}; border-color:{t['bad']}; color:{t['bad_text']}; }}
    QLabel#Clock[level="ok"] {{ border-color:{t['ok']}; color:{t['ok']}; }}

    QLabel#Lamp {{ background:{t['idle']}; color:{t['idle_text']}; border:2px solid {t['idle']};
                border-radius:4px; padding:1px 4px; font-size:12px; font-weight:800; }}
    QLabel#Lamp[kind="ok"] {{ background:{t['ok']}; border-color:{t['ok']}; color:{t['ok_text']}; }}
    QLabel#Lamp[kind="okline"] {{ background:{t['panel']}; border-color:{t['ok']}; color:{t['ok']}; }}
    QLabel#Lamp[kind="info"] {{ background:{t['info']}; border-color:{t['info']}; color:{t['info_text']}; }}
    QLabel#Lamp[kind="warn"] {{ background:{t['warn']}; border-color:{t['warn']}; color:{t['warn_text']}; }}
    QLabel#Lamp[kind="bad"] {{ background:{t['bad']}; border-color:{t['bad']}; color:{t['bad_text']}; }}
    """


def apply_theme(key: str) -> None:
    T.clear()
    T.update(THEMES.get(key, THEMES["light"]))
    T["key"] = key if key in THEMES else "light"
    app = QtWidgets.QApplication.instance()
    if app is not None:
        app.setStyleSheet(build_qss())


def set_level(widget: QtWidgets.QWidget, level: str, prop: str = "level") -> None:
    """Change a QSS state property and re-apply the style sheet to the widget
    and its labels (Qt does not do that on its own)."""
    if widget.property(prop) == level:
        return
    widget.setProperty(prop, level)
    st = widget.style()
    for w in [widget, *widget.findChildren(QtWidgets.QLabel)]:
        st.unpolish(w)
        st.polish(w)
    widget.update()


def kbutton(text: str, kind: str = "", tip: str = "") -> QtWidgets.QPushButton:
    b = QtWidgets.QPushButton(text)
    if kind:
        b.setProperty("kind", kind)
    if tip:
        b.setToolTip(tip)
    b.setCursor(QtCore.Qt.CursorShape.PointingHandCursor)
    return b


def badge(text: str, level: str = "") -> QtWidgets.QLabel:
    lab = QtWidgets.QLabel(text)
    lab.setObjectName("Badge")
    lab.setProperty("level", level)
    return lab


# ==============================================================================
#  Widgets
# ==============================================================================
class Tile(QtWidgets.QFrame):
    """
    One reading: a caption, a big number and a small second line.

    level: "idle" (no data / not relevant), "ok" (green stripe), "info",
    "accent", "warn" (solid amber), "bad" (solid red).
    """

    def __init__(self, title, value="--", hero=False):
        super().__init__()
        self.setObjectName("Tile")
        self.setProperty("level", "idle")
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(10, 4, 8, 5)
        lay.setSpacing(0)
        self.title_label = QtWidgets.QLabel(title.upper())
        self.title_label.setObjectName("TileTitle")
        self.value_label = QtWidgets.QLabel(value)
        self.value_label.setObjectName("TileValue")
        self.value_label.setProperty("hero", "true" if hero else "false")
        self.sub_label = QtWidgets.QLabel(" ")
        self.sub_label.setObjectName("TileSub")
        for w in (self.title_label, self.value_label, self.sub_label):
            w.setSizePolicy(QtWidgets.QSizePolicy.Policy.Ignored,
                            QtWidgets.QSizePolicy.Policy.Preferred)
            lay.addWidget(w)
        self.setSizePolicy(QtWidgets.QSizePolicy.Policy.Expanding,
                           QtWidgets.QSizePolicy.Policy.Fixed)

    def set_value(self, text, sub=None):
        self.value_label.setText(text)
        if sub is not None:
            self.sub_label.setText(sub or " ")

    def set_level(self, level):
        set_level(self, level)


class IndicatorStrip(QtWidgets.QFrame):
    """
    A row of named lights.

    Used for subsystem health, pyro continuity and the flight events — the
    things that are binary and that a number cannot express. on_kind / off_kind
    pick what "set" and "not set" look like (see the Lamp styles).
    """

    def __init__(self, title, labels, on_kind="ok", off_kind="idle"):
        super().__init__()
        self.setObjectName("Panel")
        self.on_kind, self.off_kind = on_kind, off_kind
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(8, 3, 8, 6)
        outer.setSpacing(3)

        cap = QtWidgets.QLabel(title.upper())
        cap.setObjectName("Caption")
        outer.addWidget(cap)

        row = QtWidgets.QHBoxLayout()
        row.setSpacing(4)
        outer.addLayout(row)

        self.lamps = {}
        for name in labels:
            lamp = QtWidgets.QLabel(name)
            lamp.setObjectName("Lamp")
            lamp.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
            lamp.setFixedHeight(26)
            lamp.setMinimumWidth(10)
            lamp.setSizePolicy(QtWidgets.QSizePolicy.Policy.Ignored,
                               QtWidgets.QSizePolicy.Policy.Fixed)
            lamp.setProperty("kind", off_kind)
            self.lamps[name] = lamp
            row.addWidget(lamp)

    def set(self, name, on, warn=False):
        lamp = self.lamps.get(name)
        if lamp is None:
            return
        set_level(lamp, "warn" if warn else (self.on_kind if on else self.off_kind), "kind")


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
        self.setMinimumSize(150, 150)

    def update_tvc(self, x, y, clamp, phase):
        self.x, self.y, self.clamp, self.phase = x, y, clamp, phase
        self.update()

    def paintEvent(self, _event):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        rect = self.rect()
        p.setPen(QtGui.QPen(C("border"), 1))
        p.setBrush(C("panel"))
        p.drawRoundedRect(QtCore.QRectF(rect).adjusted(0.5, 0.5, -0.5, -0.5), 8, 8)

        p.setPen(C("muted"))
        p.setFont(ui_font(11, QtGui.QFont.Weight.ExtraBold))
        p.drawText(QtCore.QRectF(10, 4, rect.width() - 20, 16),
                   QtCore.Qt.AlignmentFlag.AlignLeft, "TVC · CLAMP")

        bar_w = 22
        top, bottom = 24, 24
        size = min(rect.width() - bar_w - 34, rect.height() - top - bottom)
        r = max(20, size // 2)
        cx = 12 + r
        cy = top + (rect.height() - top - bottom) // 2

        # travel limit and half-travel rings
        p.setBrush(C("panel2"))
        p.setPen(QtGui.QPen(C("text"), 2))
        p.drawEllipse(QtCore.QPointF(cx, cy), float(r), float(r))
        p.setPen(QtGui.QPen(C("dim"), 1, QtCore.Qt.PenStyle.DashLine))
        p.setBrush(QtCore.Qt.BrushStyle.NoBrush)
        p.drawEllipse(QtCore.QPointF(cx, cy), r / 2.0, r / 2.0)
        p.drawLine(cx - r, cy, cx + r, cy)
        p.drawLine(cx, cy - r, cx, cy + r)

        # the commanded deflection
        mag = math.hypot(self.x, self.y)
        px = cx + (max(-1.0, min(1.0, self.x / self.MAX_DEG))) * r
        py = cy - (max(-1.0, min(1.0, self.y / self.MAX_DEG))) * r
        colour = C("bad") if mag > self.MAX_DEG * 0.9 else C("ok")
        p.setPen(QtGui.QPen(colour, 3))
        p.drawLine(QtCore.QPointF(cx, cy), QtCore.QPointF(px, py))
        p.setBrush(colour)
        p.setPen(QtGui.QPen(C("panel"), 2))
        p.drawEllipse(QtCore.QPointF(px, py), 7.0, 7.0)

        p.setPen(C("text"))
        p.setFont(ui_font(13, QtGui.QFont.Weight.Bold, mono=True))
        p.drawText(QtCore.QRectF(4, rect.height() - 22, cx * 2, 18),
                   QtCore.Qt.AlignmentFlag.AlignCenter,
                   f"{self.x:+.1f}° {self.y:+.1f}°")

        # thrust clamp
        bx = cx + r + 14
        by0, by1 = top, rect.height() - bottom
        p.setPen(QtGui.QPen(C("text"), 2))
        p.setBrush(C("panel2"))
        p.drawRect(bx, by0, bar_w, by1 - by0)

        frac = max(0.0, min(1.0, self.clamp / 100.0))
        fh = int((by1 - by0) * frac)
        p.setPen(QtCore.Qt.PenStyle.NoPen)
        p.setBrush(C("accent") if self.phase == PHASE_BURN else C("dim"))
        p.drawRect(bx + 3, by1 - fh + 2, bar_w - 5, max(0, fh - 3))

        p.setPen(C("text"))
        p.drawText(QtCore.QRectF(bx - 14, rect.height() - 22, bar_w + 28, 18),
                   QtCore.Qt.AlignmentFlag.AlignCenter, f"{self.clamp}%")
        p.end()


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
        painter.setFont(ui_font(11, QtGui.QFont.Weight.Bold))
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
        painter.setPen(QtGui.QPen(C("border"), 1))
        painter.setBrush(C("panel"))
        painter.drawRoundedRect(QtCore.QRectF(rect).adjusted(0.5, 0.5, -0.5, -0.5), 8, 8)
        self.COLOR_UI_BG = C("panel2")
        self.COLOR_GREEN = C("text")
        cx, cy = rect.center().x(), rect.center().y()
        radius = max(30, min(cx, cy - 34) - 22)

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
        painter.setPen(QtGui.QPen(C("dim"), 12))
        painter.drawEllipse(QtCore.QPointF(float(cx), float(cy)),
                            float(radius + 6), float(radius + 6))
        painter.setPen(QtGui.QPen(C("text"), 3))
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

        f_small = ui_font(12, QtGui.QFont.Weight.ExtraBold)
        f_large = ui_font(19, QtGui.QFont.Weight.Bold, mono=True)

        sr = QtCore.QRectF(cx - 80, cy - radius - 52, 160, 46)
        painter.setBrush(self.COLOR_UI_BG)
        painter.setPen(QtGui.QPen(C("border"), 2))
        painter.drawRoundedRect(sr, 6, 6)
        painter.setPen(C("muted"))
        painter.setFont(f_small)
        painter.drawText(QtCore.QRectF(sr.x(), sr.y() + 2, sr.width(), 16),
                         QtCore.Qt.AlignmentFlag.AlignHCenter, "RYCHLOST")
        painter.setPen(self.COLOR_GREEN)
        painter.setFont(f_large)
        painter.drawText(QtCore.QRectF(sr.x(), sr.y() + 17, sr.width(), 26),
                         QtCore.Qt.AlignmentFlag.AlignHCenter, f"{self.speed:.1f} m/s")

        hr = QtCore.QRectF(cx - 62, cy + radius + 8, 124, 36)
        painter.setBrush(self.COLOR_UI_BG)
        painter.setPen(QtGui.QPen(C("border"), 2))
        painter.drawRoundedRect(hr, 6, 6)
        painter.setPen(self.COLOR_GREEN)
        painter.setFont(f_large)
        painter.drawText(hr, QtCore.Qt.AlignmentFlag.AlignCenter, f"AZ {int(self.yaw):03d}°")


class AltimeterWidget(QtWidgets.QWidget):
    """
    Altitude tape, with the guidance handover altitude marked.

    The ignition line is not decoration: it is the altitude at which the
    Navigation MCU takes authority, and watching the marker cross it is how
    you know the handover happened.
    """

    GUIDANCE_ENTRY = 56.0

    def __init__(self):
        super().__init__()
        self.altitude = 0.0
        self.max_alt = 500.0
        self.setFixedWidth(118)

    def update_altitude(self, altitude):
        self.altitude = altitude
        if self.altitude > self.max_alt - 50:
            self.max_alt += 200
        self.update()

    def paintEvent(self, _event):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        rect = self.rect()
        p.setPen(QtGui.QPen(C("border"), 1))
        p.setBrush(C("panel"))
        p.drawRoundedRect(QtCore.QRectF(rect).adjusted(0.5, 0.5, -0.5, -0.5), 8, 8)

        top, bottom = 34, 18
        height = rect.height() - top - bottom
        x_axis = 44

        p.setPen(C("muted"))
        p.setFont(ui_font(11, QtGui.QFont.Weight.ExtraBold))
        p.drawText(QtCore.QRectF(0, 6, rect.width(), 16),
                   QtCore.Qt.AlignmentFlag.AlignCenter, "VÝŠKA [m]")

        p.setPen(QtGui.QPen(C("text"), 2))
        p.drawLine(x_axis, top, x_axis, top + height)

        p.setFont(ui_font(12, QtGui.QFont.Weight.Bold, mono=True))
        step = 100 if self.max_alt <= 900 else 200
        for alt in range(0, int(self.max_alt) + 1, step):
            y = top + height - int(alt / self.max_alt * height)
            p.drawLine(x_axis - 6, y, x_axis, y)
            p.drawText(QtCore.QRectF(0, y - 10, x_axis - 9, 20),
                       QtCore.Qt.AlignmentFlag.AlignRight | QtCore.Qt.AlignmentFlag.AlignVCenter,
                       f"{alt}")

        # guidance handover line
        gy = top + height - int(self.GUIDANCE_ENTRY / self.max_alt * height)
        p.setPen(QtGui.QPen(C("ok"), 2, QtCore.Qt.PenStyle.DashLine))
        p.drawLine(x_axis, gy, rect.width() - 6, gy)
        p.setPen(C("ok"))
        p.setFont(ui_font(11, QtGui.QFont.Weight.ExtraBold))
        p.drawText(x_axis + 4, gy - 4, "ZÁŽEH ~56")

        y = top + height - int(max(0.0, min(self.altitude, self.max_alt)) / self.max_alt * height)
        p.setBrush(C("accent"))
        p.setPen(QtGui.QPen(C("text"), 1))
        p.drawPolygon(QtGui.QPolygonF([
            QtCore.QPointF(float(x_axis + 2), float(y)),
            QtCore.QPointF(float(x_axis + 16), float(y - 9)),
            QtCore.QPointF(float(x_axis + 16), float(y + 9))]))
        p.setPen(C("text"))
        p.setFont(ui_font(14, QtGui.QFont.Weight.Bold, mono=True))
        p.drawText(QtCore.QRectF(x_axis + 18, y - 11, rect.width() - x_axis - 20, 22),
                   QtCore.Qt.AlignmentFlag.AlignLeft | QtCore.Qt.AlignmentFlag.AlignVCenter,
                   f"{self.altitude:.0f}")
        p.end()


# ==============================================================================
#  Main window
# ==============================================================================
class GroundStation(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.packet_count = 0
        self.data_alt, self.data_accel, self.data_vel = [], [], []
        self.data_tvc_x, self.data_tvc_y = [], []
        self.data_t = []                       # arrival time of each sample (monotonic s)

        self.boot_time = time.time()
        self.last_packet_time = time.time()
        self.rx_timestamps = []
        self.is_test_mode = False
        self.last_state = None

        self.is_recording = False
        self.csv_file = None
        self.csv_writer = None
        self.last_flush_time = 0.0

        self.pad_lat, self.pad_lon, self.pad_src = load_pad()
        self.pad_acc = []                      # recent pad estimates from the telemetry
        self.sim_lat, self.sim_lon = self.pad_lat, self.pad_lon
        self.test_t = 0.0

        self.setWindowTitle("Daidalos · Ariadna V2 — pozemní stanice")
        self.resize(1440, 860)

        self.progress_dialog = QtWidgets.QProgressDialog(
            "Caching offline maps...", "Skip", 0, 100, self)
        self.progress_dialog.setWindowTitle("Pre-flight setup")
        self.progress_dialog.setWindowModality(QtCore.Qt.WindowModality.WindowModal)
        self.progress_dialog.setMinimumDuration(0)

        self.downloader = MapDownloader(self.pad_lat, self.pad_lon, ZOOM_STRATEGY)
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
        self.serial_thread.sync_received.connect(self.process_sync)
        self.serial_thread.link_stats.connect(self.update_link_stats)

        self.last_sync = None
        self.setup_ui()

        self.watchdog_timer = QtCore.QTimer()
        self.watchdog_timer.timeout.connect(self.check_watchdog)
        self.watchdog_timer.start(100)

        self.utc_timer = QtCore.QTimer()
        self.utc_timer.timeout.connect(self.send_utc_sync)
        self.utc_timer.start(5000)

        self.test_data_timer = QtCore.QTimer()
        self.test_data_timer.timeout.connect(self.generate_test_data)

    def setup_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(10, 8, 10, 4)
        root.setSpacing(7)

        # ---------------- row 1: what the vehicle is doing, and the link ----------------
        bar = QtWidgets.QHBoxLayout()
        bar.setSpacing(8)
        root.addLayout(bar)

        brand = QtWidgets.QVBoxLayout()
        brand.setSpacing(0)
        b1 = QtWidgets.QLabel("DAIDALOS")
        b1.setObjectName("Brand")
        b2 = QtWidgets.QLabel("Ariadna V2 · pozemní stanice")
        b2.setObjectName("BrandSub")
        brand.addWidget(b1)
        brand.addWidget(b2)
        bar.addLayout(brand)
        bar.addSpacing(6)

        self.state_banner = QtWidgets.QLabel("BEZ DAT")
        self.state_banner.setObjectName("State")
        self.state_banner.setProperty("level", "idle")
        self.state_banner.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.state_banner.setMinimumWidth(210)
        self.state_banner.setToolTip("Stav letového automatu (mission state)")
        bar.addWidget(self.state_banner)

        # Launch countdown. The vehicle does not know when it will be
        # launched; the operator does. Before lift-off this shows T-, from
        # the rocket's synchronised countdown when there is one, else from
        # the local START T-0; in flight it shows the flight time.
        self.clock = QtWidgets.QLabel("T  --:--")
        self.clock.setObjectName("Clock")
        self.clock.setProperty("level", "")
        self.clock.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.clock.setMinimumWidth(190)
        self.clock.setToolTip("Odpočet do startu (T-) / čas letu (T+)")
        bar.addWidget(self.clock)

        self.test_mode_label = badge("SIMULACE · bez signálu", "warn")
        self.test_mode_label.setToolTip("Není připojená pozemní stanice - zobrazují se simulovaná data")
        self.test_mode_label.setVisible(False)
        bar.addWidget(self.test_mode_label)

        self.los_label = badge("ZTRÁTA SIGNÁLU", "bad")
        self.los_label.setStyleSheet("font-size:18px; padding:2px 16px;")
        self.los_label.setVisible(False)
        bar.addWidget(self.los_label)

        bar.addStretch()

        self.port_combo = QtWidgets.QComboBox()
        self.port_combo.setMinimumWidth(130)
        self.refresh_ports()
        bar.addWidget(self.port_combo)

        refresh = kbutton("⟳", "", "Znovu načíst seznam sériových portů")
        refresh.setFixedWidth(40)
        refresh.clicked.connect(self.refresh_ports)
        bar.addWidget(refresh)

        self.connect_btn = kbutton("PŘIPOJIT", "primary", "Připojit / odpojit pozemní stanici (USB)")
        self.connect_btn.setMinimumWidth(118)
        self.connect_btn.clicked.connect(self.toggle_connection)
        bar.addWidget(self.connect_btn)

        bar.addSpacing(8)
        self.record_btn = kbutton("● ZÁZNAM", "", "Nahrávat telemetrii do CSV v Dokumenty\\Daidalos (klávesa R)")
        self.record_btn.clicked.connect(self.toggle_recording)
        bar.addWidget(self.record_btn)

        self.map_btn = kbutton("MAPA ⭳", "", "Stáhnout letecké snímky (ČÚZK ortofoto, 15 cm/px) okolí "
                                            "rampy pro použití offline na místě")
        self.map_btn.clicked.connect(self.download_map_here)
        bar.addWidget(self.map_btn)

        self.theme_btn = kbutton("", "", "Přepnout vzhled: VENKU (světlý, na slunce) / NOC (tmavý)")
        self.theme_btn.setMinimumWidth(92)
        self.theme_btn.clicked.connect(self.toggle_theme)
        bar.addWidget(self.theme_btn)

        # ---------------- row 2: uplink & Rampa control (fw 3.34) ----------------
        ul = QtWidgets.QHBoxLayout()
        ul.setSpacing(8)
        root.addLayout(ul)

        self.ul_win_label = badge("RX OKNO: --")
        self.ul_win_label.setMinimumWidth(190)
        ul.addWidget(self.ul_win_label)

        self.rampa_label = badge("RAMPA: OFFLINE")
        ul.addWidget(self.rampa_label)

        self.ul_ack_label = QtWidgets.QLabel("ACK: —")
        self.ul_ack_label.setObjectName("Ack")
        ul.addWidget(self.ul_ack_label)

        ul.addStretch()

        t0_cap = QtWidgets.QLabel("T-0 ZA")
        t0_cap.setObjectName("Caption")
        ul.addWidget(t0_cap)
        self.t0_spin = QtWidgets.QSpinBox()
        self.t0_spin.setRange(5, 3600)
        self.t0_spin.setValue(60)
        self.t0_spin.setSuffix(" s")
        self.t0_spin.setMinimumWidth(86)
        ul.addWidget(self.t0_spin)
        self.t0_btn = kbutton("START T-0", "", "Jen místní odpočet na této obrazovce (nic neodesílá)")
        self.t0_btn.clicked.connect(self.toggle_countdown)
        ul.addWidget(self.t0_btn)
        self.t0_deadline = None
        self.t0_timer = QtCore.QTimer()
        self.t0_timer.timeout.connect(self.update_countdown)
        self.t0_timer.start(200)

        ul.addSpacing(10)
        self.ul_prep_btn = kbutton("PŘÍPRAVA (PREP)", "info",
                                   "Odešle pokyn Příprava / Start brzy (čerstvý PADREF + kontrola rampy)")
        self.ul_prep_btn.clicked.connect(self.send_uplink_prep)
        ul.addWidget(self.ul_prep_btn)

        self.ul_cd_btn = kbutton("ODPOČET → RAMPA", "warn",
                                 "Odešle synchronizovaný odpočet (dle pole T-0 ZA) raketě a přes pupeční I2C rampě")
        self.ul_cd_btn.clicked.connect(self.send_uplink_countdown)
        ul.addWidget(self.ul_cd_btn)

        self.ul_padref_btn = kbutton("PADREF", "", "Raketa si znovu změří referenční polohu rampy")
        self.ul_padref_btn.clicked.connect(lambda: self._send_ul("CMD:PADREF", "PADREF"))
        ul.addWidget(self.ul_padref_btn)

        self.ul_reboot_btn = kbutton("RESTART", "", "Vzdálený restart rakety (povoleno pouze na zemi)")
        self.ul_reboot_btn.clicked.connect(self.send_uplink_reboot)
        ul.addWidget(self.ul_reboot_btn)

        ul.addSpacing(18)
        self.ul_abort_btn = kbutton("✖  ABORT", "abort",
                                    "Okamžitě zařadí ABORT do nejbližšího RX okna (včetně nouzového okna T-1 s)")
        self.ul_abort_btn.clicked.connect(self.send_uplink_abort)
        ul.addWidget(self.ul_abort_btn)

        # ---------------- body ----------------
        body = QtWidgets.QHBoxLayout()
        body.setSpacing(10)
        root.addLayout(body, stretch=1)

        # ---- left: the graphs, stacked, each the full width of the column ----
        graphs = QtWidgets.QVBoxLayout()
        graphs.setSpacing(6)
        body.addLayout(graphs, stretch=30)

        # ---- centre: the numbers, the lights, attitude and the map ----
        centre = QtWidgets.QVBoxLayout()
        centre.setSpacing(7)
        body.addLayout(centre, stretch=66)

        self.tiles = {}

        def tile(key, title, hero=False):
            t = Tile(title, hero=hero)
            self.tiles[key] = t
            return t

        # The four numbers that matter during the flight, big.
        hero = QtWidgets.QHBoxLayout()
        hero.setSpacing(7)
        centre.addLayout(hero)
        for key, title in (("ALT", "Výška"), ("VZ", "Svislá rychlost"),
                           ("TILT", "Náklon"), ("NEXT", "Další událost")):
            hero.addWidget(tile(key, title, hero=True))

        grid = QtWidgets.QGridLayout()
        grid.setHorizontalSpacing(7)
        grid.setVerticalSpacing(7)
        centre.addLayout(grid)
        small = [
            ("HVEL", "Vodorovná rychlost"), ("ACCEL", "Zrychlení"), ("THRUST", "Tah · clamp"),
            ("POWER", "Baterie · proud"), ("PHASE", "Fáze navedení"),
            ("GPS", "GPS"),
            # The UWB constellation: anchors alive AND what the filter does
            # with them - they fail independently (a survey error looks like
            # live anchors whose every fix is gated out).
            ("UWB", "UWB kotvy · fúze"),
            ("LORA", "LoRa linka"), ("PAD", "Poloha vůči rampě [m]"), ("ERROR", "Chyba"),
        ]
        for i, (key, title) in enumerate(small):
            grid.addWidget(tile(key, title), i // 5, i % 5)
        for col in range(5):
            grid.setColumnStretch(col, 1)

        strips = QtWidgets.QHBoxLayout()
        strips.setSpacing(7)
        centre.addLayout(strips)
        self.health_strip = IndicatorStrip("Subsystémy", [n for _, n in HEALTH_BITS],
                                           on_kind="okline", off_kind="bad")
        strips.addWidget(self.health_strip, 1)
        self.pyro_strip = IndicatorStrip("Pyro · C = kontinuita, F = odpáleno",
                                         ["C1", "C2", "C3", "F1", "F2", "F3", "ARM", "RBF"],
                                         on_kind="ok", off_kind="idle")
        strips.addWidget(self.pyro_strip, 1)

        self.event_strip = IndicatorStrip("Události letu", [n for _, n in EVENT_BITS],
                                          on_kind="info", off_kind="idle")
        centre.addWidget(self.event_strip)

        # Attitude, three ways: the navball, a 3-D rocket in the pad frame,
        # and the tilt bull's-eye (where the top of the vehicle leans); then
        # the map and the nozzle.
        lower = QtWidgets.QHBoxLayout()
        lower.setSpacing(7)
        centre.addLayout(lower, stretch=1)
        self.navball = NavBallWidget()
        self.navball.setMinimumSize(230, 240)
        lower.addWidget(self.navball, stretch=3)
        att_r = QtWidgets.QVBoxLayout()
        att_r.setSpacing(7)
        lower.addLayout(att_r, stretch=2)
        self.rocket_view = RocketView()
        self.rocket_view.setMinimumSize(170, 150)
        self.rocket_view.show_readout = False      # the NÁKLON tile says it, larger
        att_r.addWidget(self.rocket_view, stretch=3)
        self.tilt_bubble = TiltBubble()
        self.tilt_bubble.setMinimumSize(130, 120)
        att_r.addWidget(self.tilt_bubble, stretch=2)

        self.map_widget = RocketMapWidget(self.pad_lat, self.pad_lon, self.pad_src)
        self.map_widget.setMinimumWidth(220)
        lower.addWidget(self.map_widget, stretch=4)

        self.tvc_widget = TVCWidget()
        self.tvc_widget.setFixedWidth(160)
        lower.addWidget(self.tvc_widget, stretch=0)

        self.alt_plot = pg.PlotWidget()
        self.accel_plot = pg.PlotWidget()
        self.vel_plot = pg.PlotWidget()
        self.tvc_plot = pg.PlotWidget()
        self.alt_curve = self.alt_plot.plot()
        self.accel_curve = self.accel_plot.plot()
        self.vel_curve = self.vel_plot.plot()
        self.tvc_plot.addLegend(offset=(-10, 6))
        self.tvc_x_curve = self.tvc_plot.plot(name="X")
        self.tvc_y_curve = self.tvc_plot.plot(name="Y")
        self._plots = ((self.alt_plot, "Výška", "m"), (self.accel_plot, "Zrychlení", "g"),
                       (self.vel_plot, "Svislá rychlost", "m/s"), (self.tvc_plot, "Výchylka TVC", "°"))
        for plot, _title, _unit in self._plots:
            plot.setMenuEnabled(False)
            plot.getAxis("left").setWidth(44)
            graphs.addWidget(plot)
        # Altitude and acceleration cannot go negative; vertical speed and
        # TVC very much can, so they keep a symmetric axis.
        self.alt_plot.getPlotItem().setLimits(yMin=0)
        self.accel_plot.getPlotItem().setLimits(yMin=0)

        # ---- right: altimeter ----
        self.altimeter = AltimeterWidget()
        body.addWidget(self.altimeter, stretch=0)

        self.status = self.statusBar()
        self.status.showMessage("ODPOJENO")
        self.flight_t = None
        self.apply_look()

    # ------------------------------------------------------------- the look
    def apply_look(self):
        """Re-colour everything that is not styled by the application style
        sheet: the plots, the 3-D rocket, the bull's-eye and the map."""
        fg = T["plot_fg"]
        axis_font = ui_font(12, QtGui.QFont.Weight.DemiBold)
        series = {self.alt_curve: T["s_alt"], self.accel_curve: T["s_acc"],
                  self.vel_curve: T["s_vz"], self.tvc_x_curve: T["s_tx"],
                  self.tvc_y_curve: T["s_ty"]}
        for plot, title, unit in self._plots:
            plot.setBackground(T["plot_bg"])
            pi = plot.getPlotItem()
            pi.setTitle(f"<span style='font-weight:800'>{title}</span>"
                        f" <span style='font-weight:600'>[{unit}]</span>",
                        color=fg, size="11pt")
            for name in ("left", "bottom"):
                ax = pi.getAxis(name)
                ax.setPen(pg.mkPen(fg, width=1))
                ax.setTextPen(pg.mkPen(fg))
                ax.setTickFont(axis_font)
            pi.getAxis("bottom").setLabel("s před teď", color=fg)
            pi.getAxis("bottom").label.setFont(axis_font)
            pi.showGrid(x=True, y=True, alpha=T["grid"])
        for curve, col in series.items():
            curve.setPen(pg.mkPen(col, width=3))
        legend = self.tvc_plot.getPlotItem().legend
        if legend is not None:
            legend.setLabelTextColor(fg)
            for _sample, lab in legend.items:
                lab.setText(lab.text, color=fg, size="10pt", bold=True)
            legend.setBrush(pg.mkBrush(T["plot_bg"]))
            legend.setPen(pg.mkPen(T["border"]))

        rv = self.rocket_view
        rv.bg, rv.fg, rv.dim = C("panel"), C("text"), C("dim")
        rv.accent, rv.good, rv.warn, rv.bad = C("accent"), C("ok"), C("warn"), C("bad")
        tb = self.tilt_bubble
        tb.bg, tb.ring, tb.label = C("panel"), C("dim"), C("text")
        tb.good, tb.warn, tb.bad = C("ok"), C("warn"), C("bad")
        tb.gate = C("ok", 60)
        self.map_widget.browser.page().setBackgroundColor(C("panel"))
        self.theme_btn.setText("☾ NOC" if T["key"] == "light" else "☀ VENKU")
        for w in (self.navball, self.rocket_view, self.tilt_bubble, self.altimeter, self.tvc_widget):
            w.update()

    def toggle_theme(self):
        key = "dark" if T["key"] == "light" else "light"
        apply_theme(key)
        QtCore.QSettings("VoltinoLabs", "DaidalosGroundStation").setValue("theme", key)
        self.apply_look()

    # ------------------------------------------------------------ countdown
    def toggle_countdown(self):
        if self.t0_deadline is None:
            self.t0_deadline = time.time() + self.t0_spin.value()
            self.t0_btn.setText("STOP T-0")
            set_level(self.t0_btn, "primary", "kind")
        else:
            self.t0_deadline = None
            self.t0_btn.setText("START T-0")
            set_level(self.t0_btn, "", "kind")
        self.update_countdown()

    @staticmethod
    def _fmt_t(left: float) -> str:
        sign = "−" if left >= 0 else "+"
        left = abs(left)
        return f"T{sign}{int(left // 60):02d}:{left % 60:04.1f}"

    def update_countdown(self):
        flying = self.last_state is not None and 3 <= self.last_state <= 8
        # Prefer rocket-synchronized countdown when active.
        rocket_rem = None
        if self.last_sync is not None:
            age_s = time.monotonic() - self.last_sync["rx_mono"]
            if age_s < 3.0 and self.last_sync["t_launch_rem_ms"] >= 0:
                rocket_rem = self.last_sync["t_launch_rem_ms"] / 1000.0 - age_s
            # Update RX window countdown badge
            win_left = (self.last_sync["next_rx_in_ms"] / 1000.0) - age_s
            while win_left < -0.35:
                win_left += 10.0
            is_final = bool(self.last_sync["sync_flags"] & 0x02)
            gps_utc = bool(self.last_sync["sync_flags"] & 0x01)
            src = "GPS+UTC" if gps_utc else "SYNC"
            if is_final and win_left >= -0.3:
                self.ul_win_label.setText(f"⚠ T-1 s ABORT OKNO: {max(0.0, win_left):.1f} s ({src})")
                set_level(self.ul_win_label, "bad")
            elif win_left <= 0.0:
                self.ul_win_label.setText(f"RX OKNO OTEVŘENO ({src})")
                set_level(self.ul_win_label, "ok")
            else:
                self.ul_win_label.setText(f"RX OKNO za {win_left:.1f} s ({src})")
                set_level(self.ul_win_label, "info")
        if flying:
            ft = self.flight_t
            self.clock.setText(f"T+{ft:d} s" if ft is not None else "LET")
            set_level(self.clock, "ok")
            return
        if rocket_rem is not None:
            left, mark = rocket_rem, " ⚡"
        elif self.t0_deadline is not None:
            left, mark = self.t0_deadline - time.time(), ""
        else:
            self.clock.setText("T  --:--")
            set_level(self.clock, "")
            return
        self.clock.setText(self._fmt_t(left) + mark)
        set_level(self.clock, "warn" if left > 10 else "bad")

    # ------------------------------------------------- bidirectional LoRa uplink
    def send_utc_sync(self):
        """Send PC UTC ms-of-day to Ground Station for <= 10 ms TDMA sync."""
        if not getattr(self, "serial_thread", None) or not self.serial_thread.is_running:
            return
        utc_ms = int((time.time() % 86400.0) * 1000.0)
        self.serial_thread.send_line(f"UTC:{utc_ms}")

    def process_sync(self, s: dict):
        self.last_sync = s
        rstate = RAMPA_STATE_NAMES.get(s["rampa_state"], "?")
        rf = s["rampa_flags"]
        key_s = "KLÍČ OK" if (rf & 0x01) else "KLÍČ OFF"
        c_s = "".join(str(i + 1) if (rf & (1 << (i + 1))) else "-" for i in range(3))
        det_s = " · ODPOJENO" if (s["sync_flags"] & 0x04) else ""
        if s["rampa_state"] == 0:
            self.rampa_label.setText("RAMPA: OFFLINE")
            set_level(self.rampa_label, "")
        else:
            self.rampa_label.setText(
                f"RAMPA: {rstate} · {key_s} · C {c_s} · {s['rampa_vbat_v']:.2f} V{det_s}")
            lvl = "okline" if s["rampa_state"] in (2, 3, 4) and s["rampa_fault"] == 0 else (
                "bad" if s["rampa_state"] in (6, 7) or s["rampa_fault"] else "warn")
            set_level(self.rampa_label, lvl)

        rc_txt = UL_RC_NAMES.get(s["last_cmd_rc"], "?")
        self.ul_ack_label.setText(f"ACK #{s['last_cmd_seq']}: {rc_txt}")
        self.update_countdown()

    def _send_ul(self, line: str, label: str):
        if not getattr(self, "serial_thread", None) or not self.serial_thread.is_running:
            self.status.showMessage("UPLINK: Ground Station není připojená")
            return
        self.send_utc_sync()
        self.serial_thread.send_line(line)
        self.ul_ack_label.setText(f"VE FRONTĚ → {label} (čeká na RX okno)")
        self.status.showMessage(f"UPLINK zařazen: {label}")

    def send_uplink_prep(self):
        self._send_ul("CMD:PREP", "START BRZY / PREP")

    def send_uplink_countdown(self):
        sec = int(self.t0_spin.value())
        reply = QtWidgets.QMessageBox.question(
            self, "Spustit odpočet startu?",
            f"Opravdu odeslat pokyn ke startu za T-{sec} s raketě a rampě?\n\n"
            f"• Raketa předá přesný čas rampě po pupeční I2C šňůře.\n"
            f"• V čase T-1 s proběhne poslední nouzové RX okno pro ABORT.\n"
            f"• V čase T-0,1 s se I2C softwarově odpojí a v T-0 rampa zažehne palník.",
            QtWidgets.QMessageBox.StandardButton.Yes | QtWidgets.QMessageBox.StandardButton.No,
            QtWidgets.QMessageBox.StandardButton.No)
        if reply == QtWidgets.QMessageBox.StandardButton.Yes:
            self.t0_deadline = time.time() + sec
            self._send_ul(f"CMD:CD:{sec}:1", f"ODPOČET T-{sec} s")

    def send_uplink_abort(self):
        self.t0_deadline = None
        self._send_ul("CMD:ABORT", "ABORT!")

    def send_uplink_reboot(self):
        reply = QtWidgets.QMessageBox.question(
            self, "Restartovat raketu?",
            "Opravdu odeslat vzdálený RESTART všech MCU rakety?\n"
            "(Raketa příkaz přijme pouze na zemi, za letu je blokován.)",
            QtWidgets.QMessageBox.StandardButton.Yes | QtWidgets.QMessageBox.StandardButton.No,
            QtWidgets.QMessageBox.StandardButton.No)
        if reply == QtWidgets.QMessageBox.StandardButton.Yes:
            self._send_ul("CMD:REBOOT:0", "RESTART RAKETY")

    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key.Key_R:
            self.toggle_recording()
        super().keyPressEvent(event)

    def toggle_recording(self):
        if not self.is_recording:
            filename = os.path.join(USER_DIR, datetime.now().strftime("%Y_%m_%d_%H_%M_ariadna.csv"))
            self.csv_file = open(filename, "a", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            if os.stat(filename).st_size == 0:
                self.csv_writer.writerow([
                    "sys_time", "t_ms", "state", "phase", "lat", "lon", "alt",
                    "vz", "hvel", "accel", "qw", "qx", "qy", "qz", "tilt",
                    "t_event", "t_flight", "px", "py", "events",
                    "tvc_x", "tvc_y", "clamp", "thrust", "battery", "current",
                    "hdop", "sats", "pyro_cont", "pyro_fired", "health",
                    "flags", "error", "rssi", "snr",
                    "uwb_n", "uwb_mode", "uwb_aided", "uwb_alive"])
            self.is_recording = True
            self.record_btn.setText("■ NAHRÁVÁ")
            set_level(self.record_btn, "danger", "kind")
            self.last_flush_time = time.time()
            self.status.showMessage(f"NAHRÁVÁM DO {filename}")
        else:
            self.is_recording = False
            if self.csv_file:
                self.csv_file.flush()
                os.fsync(self.csv_file.fileno())
                self.csv_file.close()
                self.csv_file = None
            self.csv_writer = None
            self.record_btn.setText("● ZÁZNAM")
            set_level(self.record_btn, "", "kind")
            self.status.showMessage("ZÁZNAM UKONČEN")

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
            self.connect_btn.setText("PŘIPOJIT")
            set_level(self.connect_btn, "primary", "kind")
            self.status.showMessage("ODPOJENO")
            self.last_packet_time = time.time()
            self.los_label.setVisible(False)
        else:
            port = self.port_combo.currentText()
            if port and "No serial" not in port:
                if self.serial_thread.connect_port(port):
                    self.connect_btn.setText("ODPOJIT")
                    set_level(self.connect_btn, "", "kind")
                    self.status.showMessage(f"PŘIPOJENO K {port}")
                    self.last_packet_time = time.time()
                    self.rx_timestamps.clear()
                    self.enable_test_mode(False)
                    self.send_utc_sync()

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
            self._pdr = pdr
            self._show_link()

        if getattr(self.serial_thread, "is_running", False):
            self.los_label.setVisible(now - self.last_packet_time > 3.0)
        else:
            self.los_label.setVisible(False)
            if now - self.boot_time > 5.0 and not self.is_test_mode:
                self.enable_test_mode(True)

    def _show_link(self):
        """LoRa tile: packet delivery ratio (measured here) and RSSI/SNR
        (reported by the receiver). The worse of the two sets the colour."""
        pdr = None if self.is_test_mode else getattr(self, "_pdr", None)
        lora = getattr(self, "_lora", None)
        sub = f"{lora[0]} dBm · SNR {lora[1]} dB" if lora else " "
        tile = self.tiles["LORA"]
        tile.set_value("SIM" if pdr is None else f"{pdr:.0f} %", sub)
        rank = 0
        if pdr is not None:
            rank = max(rank, 2 if pdr < 50 else 1 if pdr < 90 else 0)
        if lora:
            rank = max(rank, 0 if lora[0] > -90 else 1 if lora[0] > -110 else 2)
        tile.set_level(("ok", "warn", "bad")[rank])

    def enable_test_mode(self, enabled):
        if self.is_test_mode == enabled:
            return
        self.is_test_mode = enabled
        self.test_mode_label.setVisible(enabled)
        if enabled:
            self.test_data_timer.start(50)
            self.status.showMessage("ŽÁDNÝ SIGNÁL — MÍSTNÍ SIMULACE")
            self.sim_lat, self.sim_lon = self.pad_lat, self.pad_lon
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

        t_event, events = None, 0
        if cyc < 4:
            state, phase, alt, vz = 2, 0, 0.0, 0.0
        elif cyc < 9:
            state, phase, alt, vz = 3, 1, (cyc - 4) * 32.0, 32.0
        elif cyc < 10:
            state, phase, alt, vz = 4, 1, 160.0, 0.0
        elif cyc < 14:
            state, phase, alt, vz = 5, 2, 160.0 - (cyc - 10) * 26.0, -26.0
            t_event = 14 - cyc
            events = 0x01 | (0x02 if t_event < 0.6 else 0)
        elif cyc < 16.6:
            state, phase = 6, 4
            alt = 56.0 * (1 - (cyc - 14) / 2.6)
            vz = -30.0 * (1 - (cyc - 14) / 2.7)
            t_event = 16.6 - cyc
            events = 0x0F | (0x20 if t_event < 1.7 else 0) | (0x40 if alt < 7 else 0)
        elif cyc < 24:
            state, phase, alt, vz, events = 7, 5, 0.0, 0.0, 0xEF
        else:
            state, phase, alt, vz, events = 8, 5, 0.0, 0.0, 0xEF

        # attitude: nose up before the apogee, nose down after, a few
        # degrees of wobble either way
        tilt = math.radians(5 + 4 * math.sin(t * 0.9))
        az = t * 0.6
        ang = math.pi - tilt if phase >= 2 else tilt
        # board -Y is the nose (fw 3.24 mount): daidalos_widgets builds it
        quat = pad_quaternion(ang, az)
        if quat[0] < 0:
            quat = tuple(-c for c in quat)

        h_speed = 4.0 + math.sin(t) * 3.0
        azim = (t * 20) % 360
        d = h_speed * 0.05
        R = 6371000.0
        self.sim_lat += math.degrees(d * math.cos(math.radians(azim)) / R)
        self.sim_lon += math.degrees(d * math.sin(math.radians(azim))
                                     / (R * math.cos(math.radians(self.pad_lat))))
        if haversine((self.sim_lat, self.sim_lon), (self.pad_lat, self.pad_lon)) * 1000 > 300:
            self.sim_lat, self.sim_lon = self.pad_lat, self.pad_lon

        burn = (phase == PHASE_BURN)
        self.process_telemetry({
            "timeMs": int(t * 1000),
            "LAT": self.sim_lat, "LON": self.sim_lon,
            "ALT": max(0.0, alt), "VZ": vz, "HVEL": h_speed,
            "TOTVEL": math.hypot(vz, h_speed),
            "ACCEL": 4.5 if burn else 1.0,
            "QUAT": quat,
            "T_EVENT": t_event, "T_FLIGHT": int(max(0.0, cyc - 4)),
            "PX": int(math.sin(t * 0.3) * 6), "PY": int(math.cos(t * 0.2) * 4),
            "EVENTS": events,
            "TVC_X": math.sin(t * 6) * 3.5 if burn else 0.0,
            "TVC_Y": math.cos(t * 5) * 2.5 if burn else 0.0,
            "THRUST": 95 + math.sin(t * 4) * 20 if burn else 0.0,
            "STATE": state, "PHASE": phase,
            "CLAMP": int(55 + math.sin(t * 3) * 40) if burn else 100,
            "BATTERY": self.tri(t, 6.8, 8.3, 60),
            "CURRENT": 5.5 if burn else 1.2,
            "HDOP": self.tri(t, 0.6, 2.5, 12), "SATS": 13,
            "PYRO_CONT": 0x00 if phase >= 4 else 0x07,
            "PYRO_FIRED": (0x05 if events & 0x20 else 0x01) if phase >= 4 else 0x00,
            "HEALTH": 0xFF,
            "FLAGS": 0x0B if state >= 2 else 0x08,
            "ERROR": 0,
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
                    *d["QUAT"], attitude_summary(*d["QUAT"])["tilt"],
                    d["T_EVENT"], d["T_FLIGHT"], d["PX"], d["PY"], d["EVENTS"],
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
            self.data_tvc_x.clear(); self.data_tvc_y.clear(); self.data_t.clear()
        self.last_state = d["STATE"]

        for series, key in ((self.data_alt, "ALT"), (self.data_accel, "ACCEL"),
                            (self.data_vel, "VZ"), (self.data_tvc_x, "TVC_X"),
                            (self.data_tvc_y, "TVC_Y")):
            series.append(d[key])
            del series[:-400]
        now = time.monotonic()
        self.data_t.append(now)
        del self.data_t[:-400]
        tx = [ti - now for ti in self.data_t]

        self.alt_curve.setData(tx, self.data_alt)
        self.accel_curve.setData(tx, self.data_accel)
        self.vel_curve.setData(tx, self.data_vel)
        self.tvc_x_curve.setData(tx, self.data_tvc_x)
        self.tvc_y_curve.setData(tx, self.data_tvc_y)

        self.alt_plot.setYRange(0, max(100, max(self.data_alt)))
        self.accel_plot.setYRange(0, max(2, max(self.data_accel)))
        self.vel_plot.setYRange(min(-10, min(self.data_vel)), max(10, max(self.data_vel)))
        self.tvc_plot.setYRange(-TVCWidget.MAX_DEG, TVCWidget.MAX_DEG)

        c = self.tiles
        att = attitude_summary(*d["QUAT"])
        self.flight_t = d["T_FLIGHT"] if d["STATE"] >= 3 else None

        c["ALT"].set_value(f"{d['ALT']:.1f} m", f"max {max(self.data_alt):.0f} m")
        c["ALT"].set_level("ok" if d["STATE"] >= 1 else "idle")
        vz = d["VZ"]
        c["VZ"].set_value(f"{vz:+.1f} m/s", "↑ stoupá" if vz > 0.5 else "↓ klesá" if vz < -0.5 else "—")
        c["VZ"].set_level("ok")
        c["TILT"].set_value(f"{att['tilt']:.1f}°",
                            f"špička {'NAHORU' if att['nose_up'] else 'DOLŮ'} · směr {att['tilt_az']:.0f}°")
        c["TILT"].set_level("ok" if att["tilt"] < 4 else "warn" if att["tilt"] < 10 else "bad")
        # The countdown the vehicle itself computes: to the ignition command
        # while it descends, to the planned touchdown once the command is out.
        if d["T_EVENT"] is None:
            c["NEXT"].set_value("--", " ")
            c["NEXT"].set_level("idle")
        elif d["PHASE"] == 2:
            c["NEXT"].set_value(f"{d['T_EVENT']:.1f} s", "do ZÁŽEHU")
            c["NEXT"].set_level("warn")
        else:
            c["NEXT"].set_value(f"{d['T_EVENT']:.1f} s", "do PŘISTÁNÍ")
            c["NEXT"].set_level("accent")

        c["HVEL"].set_value(f"{d['HVEL']:.1f} m/s", f"celková {d['TOTVEL']:.1f} m/s")
        c["HVEL"].set_level("ok")
        c["ACCEL"].set_value(f"{d['ACCEL']:.2f} g", " ")
        c["ACCEL"].set_level("warn" if d["ACCEL"] > 8 else "ok")
        burn = d["PHASE"] == PHASE_BURN
        c["THRUST"].set_value(f"{d['THRUST']:.0f} N", f"clamp {d['CLAMP']} %")
        c["THRUST"].set_level("accent" if burn else "idle")
        # 2S lithium: 7.4 V nominal, 8.4 V full, 6.6 V empty.
        c["POWER"].set_value(f"{d['BATTERY']:.2f} V", f"{d['CURRENT']:.1f} A")
        c["POWER"].set_level("bad" if d["BATTERY"] < 7.0 else "warn" if d["BATTERY"] < 7.4 else "ok")
        c["PHASE"].set_value(PHASE_NAMES.get(d["PHASE"], "?"), " ")
        c["PHASE"].set_level("accent" if d["PHASE"] in (3, 4) else
                             "info" if d["PHASE"] == 2 else "idle")

        c["GPS"].set_value(f"{d['SATS']} sat", f"HDOP {d['HDOP']:.1f}")
        c["GPS"].set_level("bad" if d["SATS"] < 5 or d["HDOP"] > 2.5 else
                           "warn" if d["HDOP"] > 1.5 else "ok")
        # Green only when the filter is ACCEPTING fixes. Alive but gated out
        # is amber, and it is the state worth acting on before launch. Three
        # anchors still navigates; two is a warning; fewer is not a
        # constellation any more.
        if not d["UWB_ALIVE"]:
            fix = "OFFLINE"
        elif d["UWB_AIDED"]:
            fix = f"FUSED {UWB_MODE_NAMES.get(d['UWB_MODE'], '?')}"
        else:
            fix = "GATED OUT"
        c["UWB"].set_value(f"{d['UWB_N']} / 4", fix)
        if not d["UWB_ALIVE"]:
            c["UWB"].set_level("idle")
        elif d["UWB_N"] < 2:
            c["UWB"].set_level("bad")
        elif d["UWB_N"] == 2 or not d["UWB_AIDED"]:
            c["UWB"].set_level("warn")
        else:
            c["UWB"].set_level("ok")
        self._lora = (d["RSSI"], d["SNR"])
        self._show_link()
        c["PAD"].set_value(f"E{d['PX']:+d} N{d['PY']:+d}", " ")
        c["PAD"].set_level("ok")
        c["ERROR"].set_value("žádná" if not d["ERROR"] else ERROR_NAMES.get(d["ERROR"], "?"), f"kód {d['ERROR']}" if d["ERROR"] else " ")
        c["ERROR"].set_level("bad" if d["ERROR"] else "ok")

        # state banner
        name = STATE_NAMES.get(d["STATE"], "?")
        level = {2: "plain", 3: "accent", 4: "accent", 5: "accent", 6: "accent", 7: "ok",
                 8: "info", 9: "bad", 10: "warn", 11: "bad"}.get(d["STATE"], "idle")
        self.state_banner.setText(name)
        set_level(self.state_banner, level)

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

        for bit, label in EVENT_BITS:
            self.event_strip.set(label, bool(d["EVENTS"] & (1 << bit)))

        # The navball shows the NOSE: +90 pitch on the pad, -90 falling
        # nose-first. The 3-D view and the bull's-eye need no convention.
        self.navball.update_attitude(att["pitch"], 0.0, att["heading"], d["TOTVEL"])
        self.rocket_view.set_quaternion(*d["QUAT"])
        self.rocket_view.set_actuators(tvc=(d["TVC_X"], d["TVC_Y"]),
                                       burning=(d["PHASE"] == PHASE_BURN))
        self.tilt_bubble.set_tilt(att["tilt"], att["tilt_az"])
        self.altimeter.update_altitude(d["ALT"])
        self.tvc_widget.update_tvc(d["TVC_X"], d["TVC_Y"], d["CLAMP"], d["PHASE"])

        if d["LAT"] or d["LON"]:
            self.map_widget.update_position(d["LAT"], d["LON"])
            if not is_simulated:
                self.track_pad(d)

        if not self.is_recording:
            src = "SIMULACE" if is_simulated else self.port_combo.currentText()
            bad = getattr(self, "_link_bad", 0)
            self.status.showMessage(
                f"{src}  ·  paketů: {self.packet_count}  ·  zahozeno: {bad}")

    # ------------------------------------------------------------------ pad
    def track_pad(self, d):
        """The rocket's navigation origin, from its GPS position and its
        position relative to the pad (px/py, whole metres, saturating at
        +-127). Averaged on the ground, where it does not move."""
        if d["STATE"] not in (1, 2) or abs(d["PX"]) >= 120 or abs(d["PY"]) >= 120:
            return
        m_lat = 111320.0
        m_lon = 111320.0 * math.cos(math.radians(d["LAT"]))
        self.pad_acc.append((d["LAT"] - d["PY"] / m_lat, d["LON"] - d["PX"] / m_lon))
        del self.pad_acc[:-50]
        if len(self.pad_acc) < 20:
            return
        lat = sum(a for a, _ in self.pad_acc) / len(self.pad_acc)
        lon = sum(b for _, b in self.pad_acc) / len(self.pad_acc)
        moved = haversine((lat, lon), (self.pad_lat, self.pad_lon)) * 1000.0
        if moved > 2.0 or self.pad_src != "rocket":
            self.pad_lat, self.pad_lon, self.pad_src = lat, lon, "rocket"
            self.map_widget.set_pad(lat, lon, "rocket")
            save_pad(lat, lon, "rocket")
            if moved > 300.0:
                self.status.showMessage("Rampa se posunula o >300 m - stiskni MAPA ⭳ pro offline mapu tam")

    def download_map_here(self):
        if getattr(self, "map_dl", None) is not None and self.map_dl.isRunning():
            return
        self.map_dl = MapDownloader(self.pad_lat, self.pad_lon, ZOOM_STRATEGY)
        self.map_dl.progress.connect(lambda v, t: self.map_btn.setText(f"MAPA {v} %"))
        self.map_dl.finished.connect(lambda: self.map_btn.setText("MAPA ⭳"))
        self.map_dl.start()

    def closeEvent(self, event):
        if self.is_recording and self.csv_file:
            self.csv_file.flush()
            os.fsync(self.csv_file.fileno())
            self.csv_file.close()
        if hasattr(self, "serial_thread"):
            self.serial_thread.disconnect_port()
        super().closeEvent(event)


def _find_app_icon() -> str | None:
    root = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
    cands = [os.path.join(root, "tools", "windows", "daidalos.ico")]
    local = os.environ.get("LOCALAPPDATA", "")
    if local:
        cands.append(os.path.join(local, "Daidalos", "daidalos.ico"))
    for path in cands:
        if os.path.isfile(path):
            return path
    return None


if __name__ == "__main__":
    if sys.platform == "win32":
        try:
            import ctypes
            ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(
                "VoltinoLabs.Daidalos.GroundStation")
        except Exception:
            pass
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    _f = QtGui.QFont(UI_FONT)
    _f.setPixelSize(13)
    app.setFont(_f)
    apply_theme(str(QtCore.QSettings("VoltinoLabs", "DaidalosGroundStation").value("theme", "light")))
    _ico_path = _find_app_icon()
    _icon = QtGui.QIcon(_ico_path) if _ico_path else QtGui.QIcon()
    if not _icon.isNull():
        app.setWindowIcon(_icon)
    window = GroundStation()
    if not _icon.isNull():
        window.setWindowIcon(_icon)
    window.showMaximized()
    sys.exit(app.exec())
