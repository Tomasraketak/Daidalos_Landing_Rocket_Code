"""
Widgets shared by the two Daidalos laptop tools:

    Diagnostics/daidalos_diag.py          the USB diagnostic / test console
    Ground_Visualizer/ground_visualizer.py the LoRa flight display

Only PyQt6 is needed. Everything here is drawn with QPainter, so neither tool
needs OpenGL or a 3-D library to show which way the rocket is pointing.

Frames, as on the wire (ariadna_link.h): the attitude is a quaternion that
rotates BODY vectors into pad-ENU (x East, y North, z Up). The body's +Z axis
is the NOSE (it points up on the pad); the landing motor pushes along body -Z.
"""
from __future__ import annotations

import math

from PyQt6 import QtCore, QtGui, QtWidgets


# ----------------------------------------------------------------------------
#  quaternion helpers
# ----------------------------------------------------------------------------
def quat_matrix(w: float, x: float, y: float, z: float):
    """Rotation matrix (rows) of a body -> world quaternion."""
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
        (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
        (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
    )


def rotate(R, v):
    return (R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
            R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
            R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2])


def quat_from_smallest_three(qx: int, qy: int, qz: int):
    """The downlink's packing: x, y, z * 32767 with w >= 0."""
    x, y, z = qx / 32767.0, qy / 32767.0, qz / 32767.0
    w = math.sqrt(max(0.0, 1.0 - x * x - y * y - z * z))
    return w, x, y, z


def attitude_summary(w: float, x: float, y: float, z: float) -> dict:
    """
    What a person wants to know about the attitude, without Euler angles.

      tilt      deg, angle of the long axis from vertical (0 = upright OR
                exactly nose-down; `nose_up` says which)
      tilt_az   deg, compass direction the top of the vehicle leans toward
      nose_up   True when the nose is above the horizon
      roll      deg, compass heading of body axis 1 (fin 1 / TVC X), i.e.
                how far the airframe has turned about its own long axis
      pitch     deg, elevation of the nose above the horizon (+90 on the pad)
      heading   deg, compass heading of the nose's ground projection
    """
    R = quat_matrix(w, x, y, z)
    nose = rotate(R, (0.0, 0.0, 1.0))
    bx = rotate(R, (1.0, 0.0, 0.0))
    nz = max(-1.0, min(1.0, nose[2]))
    tilt = math.degrees(math.acos(abs(nz)))
    # the end that is on top leans toward...
    top = nose if nz >= 0 else (-nose[0], -nose[1], -nose[2])
    tilt_az = math.degrees(math.atan2(top[0], top[1])) % 360.0
    roll = math.degrees(math.atan2(bx[0], bx[1])) % 360.0
    horiz = math.hypot(nose[0], nose[1])
    heading = math.degrees(math.atan2(nose[0], nose[1])) % 360.0 if horiz > 0.02 else roll
    return {"tilt": tilt, "tilt_az": tilt_az, "nose_up": nz >= 0, "roll": roll,
            "pitch": math.degrees(math.asin(nz)), "heading": heading,
            "nose": nose, "top": top}


# ----------------------------------------------------------------------------
#  3-D rocket
# ----------------------------------------------------------------------------
class RocketView(QtWidgets.QWidget):
    """
    A wire-frame rocket in the pad frame, seen from the south-east and a
    little above. The ground grid and the compass letters do not move; the
    rocket does. The orange end is the LANDING MOTOR (the nose), the fins are
    at the other end, fin 1 is drawn brighter so roll is visible.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(220, 240)
        self._q = (1.0, 0.0, 0.0, 0.0)
        self._valid = False
        self._fins = (0.0, 0.0, 0.0, 0.0)
        self._tvc = (0.0, 0.0)
        self._burn = False
        self._cam_az = math.radians(150.0)      # camera stands south-south-east of the pad
        self._cam_el = math.radians(18.0)
        self.bg = QtGui.QColor("#101418")
        self.fg = QtGui.QColor("#e8eef2")
        self.dim = QtGui.QColor("#5c6b76")
        self.accent = QtGui.QColor("#ff7a1a")
        self.good = QtGui.QColor("#35d07f")
        self.warn = QtGui.QColor("#ffb020")
        self.bad = QtGui.QColor("#ff4d4d")

    # -- inputs ---------------------------------------------------------------
    def set_quaternion(self, w, x, y, z, valid=True):
        self._q = (w, x, y, z)
        self._valid = valid
        self.update()

    def set_actuators(self, fins=None, tvc=None, burning=False):
        if fins is not None:
            self._fins = tuple(fins)
        if tvc is not None:
            self._tvc = tuple(tvc)
        self._burn = burning
        self.update()

    # -- projection -----------------------------------------------------------
    def _project(self, p, cx, cy, scale):
        """Orthographic: world ENU point -> widget pixels."""
        ca, sa = math.cos(self._cam_az), math.sin(self._cam_az)
        ce, se = math.cos(self._cam_el), math.sin(self._cam_el)
        # camera right and up vectors in ENU
        # The camera stands at compass azimuth `_cam_az` from the pad and looks
        # back at it: points further away are drawn higher on the screen.
        right = (-ca, sa, 0.0)
        up = (-sa * se, -ca * se, ce)
        u = p[0] * right[0] + p[1] * right[1] + p[2] * right[2]
        v = p[0] * up[0] + p[1] * up[1] + p[2] * up[2]
        return QtCore.QPointF(cx + u * scale, cy - v * scale)

    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), self.bg)
        wdt, hgt = self.width(), self.height()
        cx, cy = wdt / 2.0, hgt * 0.56
        scale = min(wdt, hgt) * 0.36          # pixels per unit; the rocket is 2 long

        def line(a, b, pen):
            p.setPen(pen)
            p.drawLine(self._project(a, cx, cy, scale), self._project(b, cx, cy, scale))

        # ground grid and compass
        grid = QtGui.QPen(self.dim, 1)
        for i in range(-2, 3):
            line((i * 0.5, -1.0, -1.05), (i * 0.5, 1.0, -1.05), grid)
            line((-1.0, i * 0.5, -1.05), (1.0, i * 0.5, -1.05), grid)
        f = p.font()
        f.setPointSize(8)
        p.setFont(f)
        p.setPen(self.dim)
        for txt, pos in (("N", (0, 1.18, -1.05)), ("E", (1.18, 0, -1.05)),
                         ("S", (0, -1.18, -1.05)), ("W", (-1.18, 0, -1.05))):
            pt = self._project(pos, cx, cy, scale)
            p.drawText(QtCore.QRectF(pt.x() - 8, pt.y() - 8, 16, 16),
                       QtCore.Qt.AlignmentFlag.AlignCenter, txt)
        # the vertical, dashed: what "upright" looks like
        dash = QtGui.QPen(self.dim, 1, QtCore.Qt.PenStyle.DashLine)
        line((0, 0, -1.05), (0, 0, 1.25), dash)

        if not self._valid:
            p.setPen(self.dim)
            p.drawText(self.rect(), QtCore.Qt.AlignmentFlag.AlignCenter, "bez dat o orientaci")
            p.end()
            return

        R = quat_matrix(*self._q)
        info = attitude_summary(*self._q)

        def body(v):
            return rotate(R, v)

        # body: a cylinder of radius r from z = -0.8 (tail) to z = +0.7, cone to +1.0
        r = 0.09
        n_seg = 12
        ring_tail, ring_top = [], []
        for i in range(n_seg):
            a = 2 * math.pi * i / n_seg
            ring_tail.append(body((r * math.cos(a), r * math.sin(a), -0.8)))
            ring_top.append(body((r * math.cos(a), r * math.sin(a), 0.7)))
        hull = QtGui.QPen(self.fg, 1.6)
        for i in range(n_seg):
            j = (i + 1) % n_seg
            line(ring_tail[i], ring_tail[j], hull)
            line(ring_top[i], ring_top[j], hull)
            if i % 3 == 0:
                line(ring_tail[i], ring_top[i], hull)
        tip = body((0, 0, 1.0))
        nose_pen = QtGui.QPen(self.accent, 2.2)
        for i in range(0, n_seg, 2):
            line(ring_top[i], tip, nose_pen)

        # fins, at the tail. Fin i sits at i*90 deg from body +X; its
        # deflection is shown by twisting the fin's outer edge.
        for i in range(4):
            a = math.pi / 2 * i
            ca, sa = math.cos(a), math.sin(a)
            d = math.radians(self._fins[i]) * 2.5       # exaggerated to be visible
            # hinge is radial; the chord runs along z, twisted about the hinge
            root_a = (r * ca, r * sa, -0.80)
            root_b = (r * ca, r * sa, -0.50)
            tx, ty = -sa * math.sin(d) * 0.3, ca * math.sin(d) * 0.3
            tip_a = (0.34 * ca + tx, 0.34 * sa + ty, -0.82)
            tip_b = (0.34 * ca - tx * 0.3, 0.34 * sa - ty * 0.3, -0.62)
            pen = QtGui.QPen(self.good if i == 0 else self.fg, 2.0 if i == 0 else 1.4)
            pts = [body(root_a), body(tip_a), body(tip_b), body(root_b)]
            for k in range(3):
                line(pts[k], pts[k + 1], pen)

        # thrust: along body +Z out of the nose, tilted by the nozzle
        if self._burn:
            sx = math.sin(math.radians(self._tvc[0] / 2.0)) * 4.0
            sy = math.sin(math.radians(self._tvc[1] / 2.0)) * 4.0
            flame = QtGui.QPen(self.warn, 3.0)
            line(tip, body((sx * 0.35, sy * 0.35, 1.45)), flame)

        # read-outs
        f.setPointSize(10)
        f.setBold(True)
        p.setFont(f)
        tilt = info["tilt"]
        col = self.good if tilt < 4 else (self.warn if tilt < 10 else self.bad)
        p.setPen(col)
        p.drawText(QtCore.QRectF(8, 6, wdt - 16, 18), QtCore.Qt.AlignmentFlag.AlignLeft,
                   f"náklon {tilt:4.1f}°")
        f.setBold(False)
        f.setPointSize(8)
        p.setFont(f)
        p.setPen(self.fg)
        p.drawText(QtCore.QRectF(8, 24, wdt - 16, 14), QtCore.Qt.AlignmentFlag.AlignLeft,
                   f"směr náklonu {info['tilt_az']:3.0f}°   roll {info['roll']:3.0f}°")
        p.drawText(QtCore.QRectF(8, 6, wdt - 16, 18), QtCore.Qt.AlignmentFlag.AlignRight,
                   "špička NAHORU" if info["nose_up"] else "špička DOLŮ")
        p.end()


# ----------------------------------------------------------------------------
#  tilt bubble: the long axis seen from above
# ----------------------------------------------------------------------------
class TiltBubble(QtWidgets.QWidget):
    """
    A bull's-eye, North up. The dot is where the top of the vehicle leans:
    centre = perfectly vertical, the rings are 5 / 10 / 20 degrees. The
    green disc is the 4 degree touchdown gate.
    """

    RINGS = (5.0, 10.0, 20.0)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(150, 150)
        self._tilt = 0.0
        self._az = 0.0
        self._valid = False
        self._cmd = None                       # commanded tilt, deg (optional)

    def set_tilt(self, tilt_deg, az_deg, valid=True, cmd_deg=None):
        self._tilt, self._az, self._valid, self._cmd = tilt_deg, az_deg, valid, cmd_deg
        self.update()

    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), QtGui.QColor("#101418"))
        w, h = self.width(), self.height()
        cx, cy = w / 2.0, h / 2.0
        rad = min(w, h) / 2.0 - 12
        full = self.RINGS[-1]

        p.setPen(QtCore.Qt.PenStyle.NoPen)
        p.setBrush(QtGui.QColor(53, 208, 127, 50))
        g = rad * 4.0 / full
        p.drawEllipse(QtCore.QPointF(cx, cy), g, g)
        p.setBrush(QtCore.Qt.BrushStyle.NoBrush)
        f = p.font()
        f.setPointSize(7)
        p.setFont(f)
        for ring in self.RINGS:
            rr = rad * ring / full
            p.setPen(QtGui.QPen(QtGui.QColor("#5c6b76"), 1))
            p.drawEllipse(QtCore.QPointF(cx, cy), rr, rr)
            p.drawText(QtCore.QPointF(cx + 3, cy - rr + 9), f"{ring:.0f}°")
        p.drawLine(QtCore.QPointF(cx - rad, cy), QtCore.QPointF(cx + rad, cy))
        p.drawLine(QtCore.QPointF(cx, cy - rad), QtCore.QPointF(cx, cy + rad))
        p.setPen(QtGui.QColor("#9fb0bb"))
        p.drawText(QtCore.QRectF(cx - 8, 0, 16, 12), QtCore.Qt.AlignmentFlag.AlignCenter, "N")
        p.drawText(QtCore.QRectF(w - 12, cy - 6, 12, 12), QtCore.Qt.AlignmentFlag.AlignCenter, "E")

        if self._cmd is not None and self._valid:
            rr = rad * min(self._cmd, full) / full
            p.setPen(QtGui.QPen(QtGui.QColor("#4aa3ff"), 1, QtCore.Qt.PenStyle.DashLine))
            p.drawEllipse(QtCore.QPointF(cx, cy), rr, rr)

        if self._valid:
            t = min(self._tilt, full)
            a = math.radians(self._az)
            x = cx + rad * t / full * math.sin(a)
            y = cy - rad * t / full * math.cos(a)
            col = "#35d07f" if self._tilt < 4 else ("#ffb020" if self._tilt < 10 else "#ff4d4d")
            p.setPen(QtGui.QPen(QtGui.QColor(col), 2))
            p.drawLine(QtCore.QPointF(cx, cy), QtCore.QPointF(x, y))
            p.setBrush(QtGui.QColor(col))
            p.drawEllipse(QtCore.QPointF(x, y), 5, 5)
        p.end()


# ----------------------------------------------------------------------------
#  small strip chart, no external plotting library
# ----------------------------------------------------------------------------
class StripChart(QtWidgets.QWidget):
    """Scrolling line chart of up to four series, auto-scaled."""

    COLORS = ("#ff7a1a", "#4aa3ff", "#35d07f", "#e86bd6")

    def __init__(self, title: str, series: list[str], unit: str = "", span_s: float = 30.0,
                 parent=None):
        super().__init__(parent)
        self.title, self.names, self.unit, self.span = title, series, unit, span_s
        self.data = [[] for _ in series]       # lists of (t, value)
        self.setMinimumSize(240, 130)

    def add(self, t: float, values):
        for buf, v in zip(self.data, values):
            if v is None or (isinstance(v, float) and math.isnan(v)):
                continue
            buf.append((t, float(v)))
            while buf and buf[0][0] < t - self.span:
                buf.pop(0)
        self.update()

    def clear(self):
        for buf in self.data:
            buf.clear()
        self.update()

    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), QtGui.QColor("#101418"))
        w, h = self.width(), self.height()
        left, top, right, bottom = 44, 20, 8, 16
        pw, ph = w - left - right, h - top - bottom
        f = p.font()
        f.setPointSize(8)
        p.setFont(f)
        p.setPen(QtGui.QColor("#e8eef2"))
        p.drawText(QtCore.QRectF(left, 2, pw, 16), QtCore.Qt.AlignmentFlag.AlignLeft,
                   f"{self.title}" + (f" [{self.unit}]" if self.unit else ""))
        # legend
        x = w - right
        for i in reversed(range(len(self.names))):
            txt = self.names[i]
            tw = p.fontMetrics().horizontalAdvance(txt) + 14
            x -= tw
            p.setPen(QtGui.QColor(self.COLORS[i % 4]))
            p.drawText(QtCore.QRectF(x, 2, tw, 16), QtCore.Qt.AlignmentFlag.AlignLeft, "■ " + txt)

        allv = [v for buf in self.data for _, v in buf]
        p.setPen(QtGui.QPen(QtGui.QColor("#2a343c"), 1))
        p.drawRect(QtCore.QRectF(left, top, pw, ph))
        if not allv:
            p.end()
            return
        lo, hi = min(allv), max(allv)
        if hi - lo < 1e-6:
            lo, hi = lo - 1.0, hi + 1.0
        pad = 0.08 * (hi - lo)
        lo, hi = lo - pad, hi + pad
        t1 = max(buf[-1][0] for buf in self.data if buf)
        t0 = t1 - self.span
        p.setPen(QtGui.QColor("#9fb0bb"))
        p.drawText(QtCore.QRectF(0, top - 6, left - 4, 12),
                   QtCore.Qt.AlignmentFlag.AlignRight, f"{hi:.4g}")
        p.drawText(QtCore.QRectF(0, top + ph - 6, left - 4, 12),
                   QtCore.Qt.AlignmentFlag.AlignRight, f"{lo:.4g}")
        if lo < 0 < hi:
            y0 = top + ph * (hi - 0) / (hi - lo)
            p.setPen(QtGui.QPen(QtGui.QColor("#3a4750"), 1, QtCore.Qt.PenStyle.DashLine))
            p.drawLine(QtCore.QPointF(left, y0), QtCore.QPointF(left + pw, y0))
        for i, buf in enumerate(self.data):
            if len(buf) < 2:
                continue
            path = QtGui.QPainterPath()
            first = True
            for t, v in buf:
                px = left + pw * (t - t0) / self.span
                py = top + ph * (hi - v) / (hi - lo)
                if first:
                    path.moveTo(px, py)
                    first = False
                else:
                    path.lineTo(px, py)
            p.setPen(QtGui.QPen(QtGui.QColor(self.COLORS[i % 4]), 1.5))
            p.drawPath(path)
        p.end()


# ----------------------------------------------------------------------------
#  top-down position plot (pad frame)
# ----------------------------------------------------------------------------
class PadMap(QtWidgets.QWidget):
    """Where the vehicle is relative to the pad, North up, with its track."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(200, 200)
        self.track = []
        self.pos = None
        self.sigma = 0.0
        self.range_m = 10.0

    def set_position(self, east, north, sigma=0.0):
        self.pos = (east, north)
        self.sigma = sigma
        if not self.track or math.hypot(east - self.track[-1][0], north - self.track[-1][1]) > 0.05:
            self.track.append((east, north))
            del self.track[:-2000]
        need = max(abs(east), abs(north)) * 1.25
        for r in (5.0, 10.0, 20.0, 50.0, 100.0, 200.0, 500.0):
            if need <= r:
                self.range_m = r
                break
        self.update()

    def clear(self):
        self.track.clear()
        self.update()

    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), QtGui.QColor("#101418"))
        w, h = self.width(), self.height()
        cx, cy = w / 2.0, h / 2.0
        rad = min(w, h) / 2.0 - 14
        k = rad / self.range_m
        f = p.font()
        f.setPointSize(7)
        p.setFont(f)
        for frac in (0.25, 0.5, 0.75, 1.0):
            p.setPen(QtGui.QPen(QtGui.QColor("#33414b"), 1))
            p.drawEllipse(QtCore.QPointF(cx, cy), rad * frac, rad * frac)
            p.setPen(QtGui.QColor("#7f909b"))
            p.drawText(QtCore.QPointF(cx + 3, cy - rad * frac + 9), f"{self.range_m * frac:g} m")
        p.setPen(QtGui.QPen(QtGui.QColor("#33414b"), 1))
        p.drawLine(QtCore.QPointF(cx - rad, cy), QtCore.QPointF(cx + rad, cy))
        p.drawLine(QtCore.QPointF(cx, cy - rad), QtCore.QPointF(cx, cy + rad))
        p.setPen(QtGui.QColor("#9fb0bb"))
        p.drawText(QtCore.QRectF(cx - 8, 0, 16, 12), QtCore.Qt.AlignmentFlag.AlignCenter, "N")
        # the pad
        p.setPen(QtGui.QPen(QtGui.QColor("#ff7a1a"), 2))
        p.drawLine(QtCore.QPointF(cx - 6, cy), QtCore.QPointF(cx + 6, cy))
        p.drawLine(QtCore.QPointF(cx, cy - 6), QtCore.QPointF(cx, cy + 6))
        if len(self.track) > 1:
            path = QtGui.QPainterPath()
            path.moveTo(cx + self.track[0][0] * k, cy - self.track[0][1] * k)
            for e, n in self.track[1:]:
                path.lineTo(cx + e * k, cy - n * k)
            p.setPen(QtGui.QPen(QtGui.QColor("#4aa3ff"), 1.2))
            p.drawPath(path)
        if self.pos:
            x, y = cx + self.pos[0] * k, cy - self.pos[1] * k
            if self.sigma > 0:
                p.setPen(QtGui.QPen(QtGui.QColor(74, 163, 255, 120), 1))
                p.setBrush(QtGui.QColor(74, 163, 255, 40))
                p.drawEllipse(QtCore.QPointF(x, y), self.sigma * k, self.sigma * k)
            p.setPen(QtCore.Qt.PenStyle.NoPen)
            p.setBrush(QtGui.QColor("#35d07f"))
            p.drawEllipse(QtCore.QPointF(x, y), 4, 4)
        p.end()
