"""
Hardware-in-the-Loop (HIL) 6-DOF flight simulator, binary Position-MCU codec,
reference LandSim_Light controller, and flight-log recorder / verifier.

Pure Python (stdlib only — zero Qt or pyserial dependency) so both the PyQt6
diagnostic console (`diag_hil.py`) and automated workstation tests
(`test/check_hil_format.py`) execute the exact same codec and flight dynamics.

Frame & wire conventions (identical to Ariadna_V2/common/ariadna_hil.h,
Position_MCU.ino, Navigation_MCU.ino and LandSim_Light):
  * Pad-ENU world frame: +x East, +y North, +z Up; origin on the pad.
  * TriSense body axes:
      +X = BODY_FIN1   (fin 1 / TVC servo 1 radial axis, maps to world u1 / g)
      +Y = BODY_THRUST (landing motor thrust direction, maps to world b)
      -Y = BODY_NOSE   (nose / landing motor end: up on pad, down in descent)
      +Z = BODY_SIDE   (u1 x b = -u2)
  * Quaternion (qw, qx, qy, qz) rotates BODY vectors into pad-ENU.
  * Specific force acc_body_ms2 = R^T * (a_pad + [0, 0, +g]), so stationary
    upright on the pad gives acc_body = (0, -9.80665, 0) m/s^2.
"""
from __future__ import annotations

import csv
import math
import random
import struct
from dataclasses import dataclass, field

# ---------------------------------------------------------------------------
#  Wire protocol constants — pinned by test/check_hil_format.py against
#  Ariadna_V2/common/ariadna_link.h and Ariadna_V2/common/ariadna_hil.h
# ---------------------------------------------------------------------------
SYNC0 = 0xA5
SYNC1 = 0x1D
HIL_PKT_TYPE = 0x50
HIL_FMT = "<I11fiifhhHH4HhBBBBBB"
HIL_PAYLOAD_SIZE = struct.calcsize(HIL_FMT)   # 84 bytes
HIL_TIMEOUT_MS = 500

# ariadna_hil_in_t.flags
HIL_FLAG_BARO_FRESH = 1 << 0
HIL_FLAG_GPS_FRESH = 1 << 1
HIL_FLAG_LIDAR_VALID = 1 << 2
HIL_FLAG_UWB_FRESH = 1 << 3
HIL_FLAG_SET_PAD_REF = 1 << 4
HIL_FLAG_RESET_EST = 1 << 5

# ariadna_seq.h ARIADNA_EV_*
EV_APOGEE = 1 << 0
EV_COVER = 1 << 1
EV_IGN1 = 1 << 2
EV_THRUST = 1 << 3
EV_IGN2 = 1 << 4
EV_LEGS = 1 << 5
EV_LIDAR = 1 << 6
EV_TOUCHDOWN = 1 << 7

# ariadna_link.h ARIADNA_ACT_*
ACT_VALID = 1 << 0
ACT_IGNITE = 1 << 1
ACT_FINS_LIVE = 1 << 2
ACT_TVC_PARK = 1 << 3
ACT_COVER_OPEN = 1 << 4

# ariadna_link.h ARIADNA_MST_*
MST_ARMED = 1 << 0
MST_RBF_OUT = 1 << 1
MST_PYRO_INHIBIT = 1 << 7

# ---------------------------------------------------------------------------
#  Vehicle & motor constants — verbatim from Ariadna_V2/common/guidance_tables.h
#  and guidance.h (LandSim_Light export)
# ---------------------------------------------------------------------------
G_ACCEL = 9.80665
RHO_AIR = 1.225

MOTOR_T = (
    0.0, 0.0, 0.050000000000000044, 0.11430000000000007, 0.14139999999999997,
    0.3055000000000001, 0.4232, 0.4548000000000001, 1.2547, 1.4739000000000002,
    1.5869000000000002, 1.7916, 2.0936000000000003, 2.3341000000000003,
    2.5464, 2.6132999999999997, 2.613301,
)
MOTOR_F = (
    0.0, 6.02, 12.04, 30.1, 60.2, 90.31,
    108.37, 114.39, 120.41, 114.39, 108.37, 90.31,
    60.2, 30.1, 12.04, 6.02, 0.0,
)
MOTOR_CUM = (
    0.0, 0.0, 0.45150000000000035, 1.8063010000000008, 3.0298659999999966,
    15.379211500000006, 27.071529499999997, 30.591137500000006,
    124.49939749999999, 150.23347750000002, 162.81941750000001,
    183.1543155, 205.8813255, 216.7399005, 221.2130615,
    221.8171685, 221.81717151,
)
MOTOR_I_TOTAL = 221.81717151
MOTOR_BURN_T = 2.613301
MOTOR_PEAK_N = 120.41

DG_M_GROSS = 2.85
DG_M_PROP = 0.277
DG_DIAMETER = 0.105
DG_CD_BODY = 0.35
DG_FIN_CD_ZERO = 0.14306194708764752
DG_FIN_CD_BRAKE = 0.44720533587550426
DG_IT_PER_KG = 0.16333333333333333
DG_IA_PER_KG = 0.0013781249999999998
DG_L_GIMBAL = 0.70
DG_L_CP = -0.60
DG_CN_ALPHA = 2.0
DG_FIN_AREA = 0.006405
DG_FIN_ARM = 0.50
DG_FIN_ROLL_ARM = 0.0805
DG_FIN_CL_ALPHA = 1.8040488883710275
DG_FIN_K_INT = 1.4285714285714286
DG_FIN_MAX_DEG = 15.0
DG_FIN_STALL_DEG = 22.0
DG_FIN_STALL_MARGIN = 1.5
DG_SERVO_MAX_DEG = 10.0
DG_TVC_RATIO = 2.0
DG_CLAMP_MIN = 0.10
DG_CLAMP_MAX = 1.00
DG_CLAMP_RATE = 12.84
DG_THRUST_GATE = 0.30

DG_WN_TVC = 8.230423385206832
DG_ZETA_TVC = 1.039857442407118
DG_SCHED_TVC = 0.6
DG_WN_FIN = 2.0
DG_ZETA_FIN = 0.9941139158605563
DG_SCHED_FIN = -0.6
DG_ROLL_GAIN = 0.7485332009639065

IGN_VZ0 = 4.0
IGN_DVZ = 2.0
IGN_HMIN = (
    2.22265625, 3.126953125, 4.142578125, 5.185546875, 6.359375, 7.623046875,
    9.2578125, 10.78515625, 12.625, 14.509765625, 16.18359375, 18.166015625,
    20.6875, 22.748046875, 25.451171875, 28.03125, 29.875, 33.09765625,
    35.892578125, 38.830078125, 41.73046875, 44.751953125, 47.623046875, 50.884765625,
    53.962890625, 58.296875, 61.37101181295551, 65.31030307743995, 68.7403834312955,
    72.26966827508187, 76.19072818577749, 80.51929230567876,
)
IGN_HMAX = (
    17.12109375, 19.998046875, 22.939453125, 25.650390625, 28.53125, 31.005859375,
    33.51953125, 35.712890625, 38.28515625, 40.431640625, 42.421875, 44.78125,
    46.6875, 48.5703125, 50.05078125, 50.99609375, 53.390625, 54.509765625,
    55.595703125, 56.705078125, 57.556640625, 57.771484375, 52.998046875, 52.748046875,
    60.435546875, 59.59375, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
)

# Default UWB anchors in Position_MCU.ino (40 m square centred on the pad)
UWB_ANCHORS = (
    (-20.0, -20.0, 0.0),
    (20.0, -20.0, 0.0),
    (20.0, 20.0, 0.0),
    (-20.0, 20.0, 0.0),
)


# ---------------------------------------------------------------------------
#  CRC-16/CCITT-FALSE & binary framing
# ---------------------------------------------------------------------------
def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else (hi if v > hi else v)


def _clamp_i16(v: float) -> int:
    return int(_clamp(round(v), -32768, 32767))


def _clamp_u16(v: float) -> int:
    return int(_clamp(round(v), 0, 65535))


@dataclass
class HilSensorSample:
    """Synthetic sensor sample matching ariadna_hil_in_t field for field."""
    seq: int = 0
    qw: float = 0.70710678
    qx: float = -0.70710678
    qy: float = 0.0
    qz: float = 0.0
    acc_body: tuple[float, float, float] = (0.0, -G_ACCEL, 0.0)
    gyr_body: tuple[float, float, float] = (0.0, 0.0, 0.0)
    pressure_pa: float = 97230.0
    gps_lat_1e7: int = 497985411
    gps_lon_1e7: int = 166912772
    gps_alt_msl: float = 412.3
    gps_ve_cms: int = 0
    gps_vn_cms: int = 0
    lidar_mm: int = 0
    lidar_strength: int = 0
    uwb_range_cm: tuple[int, int, int, int] = (2828, 2828, 2828, 2828)
    temp_c_x10: int = 215
    flags: int = 0
    gps_sats: int = 12
    gps_fix: int = 3
    gps_hdop_x10: int = 9
    uwb_mask: int = 0x0F


def pack_hil_payload(s: HilSensorSample) -> bytes:
    """Pack HilSensorSample into the 84-byte ariadna_hil_in_t payload."""
    return struct.pack(
        HIL_FMT,
        int(s.seq) & 0xFFFFFFFF,
        float(s.qw), float(s.qx), float(s.qy), float(s.qz),
        float(s.acc_body[0]), float(s.acc_body[1]), float(s.acc_body[2]),
        float(s.gyr_body[0]), float(s.gyr_body[1]), float(s.gyr_body[2]),
        float(s.pressure_pa),
        int(s.gps_lat_1e7), int(s.gps_lon_1e7),
        float(s.gps_alt_msl),
        _clamp_i16(s.gps_ve_cms), _clamp_i16(s.gps_vn_cms),
        _clamp_u16(s.lidar_mm), _clamp_u16(s.lidar_strength),
        _clamp_u16(s.uwb_range_cm[0]), _clamp_u16(s.uwb_range_cm[1]),
        _clamp_u16(s.uwb_range_cm[2]), _clamp_u16(s.uwb_range_cm[3]),
        _clamp_i16(s.temp_c_x10),
        int(s.flags) & 0xFF,
        int(s.gps_sats) & 0xFF,
        int(s.gps_fix) & 0xFF,
        int(s.gps_hdop_x10) & 0xFF,
        int(s.uwb_mask) & 0xFF,
        0,
    )


def pack_hil_frame(s: HilSensorSample) -> bytes:
    """Wrap a HilSensorSample in an ariadna_link.h binary frame (90 bytes)."""
    pl = pack_hil_payload(s)
    body = bytes((HIL_PKT_TYPE, len(pl))) + pl
    crc = crc16_ccitt_false(body)
    return bytes((SYNC0, SYNC1)) + body + struct.pack("<H", crc)


def unpack_hil_payload(payload: bytes) -> HilSensorSample:
    """Unpack an 84-byte ariadna_hil_in_t payload into a HilSensorSample."""
    if len(payload) != HIL_PAYLOAD_SIZE:
        raise ValueError(f"Expected {HIL_PAYLOAD_SIZE} bytes, got {len(payload)}")
    (
        seq, qw, qx, qy, qz,
        ax, ay, az, gx, gy, gz,
        pressure_pa, gps_lat_1e7, gps_lon_1e7, gps_alt_msl,
        gps_ve_cms, gps_vn_cms, lidar_mm, lidar_strength,
        uwb0, uwb1, uwb2, uwb3, temp_c_x10,
        flags, gps_sats, gps_fix, gps_hdop_x10, uwb_mask, _pad,
    ) = struct.unpack(HIL_FMT, payload)
    return HilSensorSample(
        seq=seq, qw=qw, qx=qx, qy=qy, qz=qz,
        acc_body=(ax, ay, az), gyr_body=(gx, gy, gz),
        pressure_pa=pressure_pa,
        gps_lat_1e7=gps_lat_1e7, gps_lon_1e7=gps_lon_1e7,
        gps_alt_msl=gps_alt_msl,
        gps_ve_cms=gps_ve_cms, gps_vn_cms=gps_vn_cms,
        lidar_mm=lidar_mm, lidar_strength=lidar_strength,
        uwb_range_cm=(uwb0, uwb1, uwb2, uwb3),
        temp_c_x10=temp_c_x10,
        flags=flags, gps_sats=gps_sats, gps_fix=gps_fix,
        gps_hdop_x10=gps_hdop_x10, uwb_mask=uwb_mask,
    )


def unpack_hil_frame(frame: bytes) -> HilSensorSample:
    """Validate and unpack a 90-byte ariadna_link.h HIL frame."""
    if len(frame) != HIL_PAYLOAD_SIZE + 6:
        raise ValueError(f"Invalid HIL frame length {len(frame)}")
    if frame[0] != SYNC0 or frame[1] != SYNC1:
        raise ValueError("Invalid HIL frame sync bytes")
    if frame[2] != HIL_PKT_TYPE or frame[3] != HIL_PAYLOAD_SIZE:
        raise ValueError(f"Unexpected type/len: 0x{frame[2]:02X}/{frame[3]}")
    crc_got = struct.unpack_from("<H", frame, 4 + HIL_PAYLOAD_SIZE)[0]
    crc_exp = crc16_ccitt_false(frame[2:4 + HIL_PAYLOAD_SIZE])
    if crc_got != crc_exp:
        raise ValueError(f"CRC mismatch: got 0x{crc_got:04X}, expected 0x{crc_exp:04X}")
    return unpack_hil_payload(frame[4:4 + HIL_PAYLOAD_SIZE])


# ---------------------------------------------------------------------------
#  Vector, rotation, quaternion, atmosphere & geodetic helpers
# ---------------------------------------------------------------------------
def _dot(a, b) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _norm(a) -> float:
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def _unit(a, fallback=(0.0, 0.0, 1.0)) -> tuple[float, float, float]:
    n = _norm(a)
    if n < 1e-12:
        return fallback
    return (a[0] / n, a[1] / n, a[2] / n)


def _rodrigues(v, axis, angle_rad) -> tuple[float, float, float]:
    """Rotate vector v by angle_rad about unit vector axis."""
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    kdv = _dot(axis, v)
    kxv = _cross(axis, v)
    return (
        v[0] * c + kxv[0] * s + axis[0] * kdv * (1.0 - c),
        v[1] * c + kxv[1] * s + axis[1] * kdv * (1.0 - c),
        v[2] * c + kxv[2] * s + axis[2] * kdv * (1.0 - c),
    )


def gimbal_basis(b, g) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Orthonormal basis (u1, u2) perpendicular to b with u2 = b x u1."""
    d = _dot(g, b)
    ax, ay, az = g[0] - d * b[0], g[1] - d * b[1], g[2] - d * b[2]
    n = math.sqrt(ax * ax + ay * ay + az * az)
    if n < 1e-9:
        ax, ay, az = 1.0 - b[0] * b[0], -b[0] * b[1], -b[0] * b[2]
        n = math.sqrt(ax * ax + ay * ay + az * az)
        if n < 1e-9:
            ax, ay, az, n = 0.0, 1.0, 0.0, 1.0
    u1 = (ax / n, ay / n, az / n)
    u2 = _cross(b, u1)
    return u1, u2


def rot_from_bg(b, g):
    """
    Build body->pad-ENU rotation matrix R from thrust axis b (= R*[0,1,0])
    and fin-1 axis g (= R*[1,0,0]). Column 2 is u1 x b = -u2 (right-handed).
    """
    bn = _unit(b, (0.0, 0.0, -1.0))
    u1, u2 = gimbal_basis(bn, g)
    c2 = (-u2[0], -u2[1], -u2[2])
    return (
        (u1[0], bn[0], c2[0]),
        (u1[1], bn[1], c2[1]),
        (u1[2], bn[2], c2[2]),
    )


def quat_from_rot(R, prev_q=None) -> tuple[float, float, float, float]:
    """Shepperd's method: 3x3 proper rotation matrix -> unit quaternion (w,x,y,z)."""
    r00, r01, r02 = R[0]
    r10, r11, r12 = R[1]
    r20, r21, r22 = R[2]
    tr = r00 + r11 + r22
    if tr > 0.0:
        s = 2.0 * math.sqrt(max(1e-12, tr + 1.0))
        w = 0.25 * s
        x = (r21 - r12) / s
        y = (r02 - r20) / s
        z = (r10 - r01) / s
    elif r00 > r11 and r00 > r22:
        s = 2.0 * math.sqrt(max(1e-12, 1.0 + r00 - r11 - r22))
        w = (r21 - r12) / s
        x = 0.25 * s
        y = (r01 + r10) / s
        z = (r02 + r20) / s
    elif r11 > r22:
        s = 2.0 * math.sqrt(max(1e-12, 1.0 + r11 - r00 - r22))
        w = (r02 - r20) / s
        x = (r01 + r10) / s
        y = 0.25 * s
        z = (r12 + r21) / s
    else:
        s = 2.0 * math.sqrt(max(1e-12, 1.0 + r22 - r00 - r11))
        w = (r10 - r01) / s
        x = (r02 + r20) / s
        y = (r12 + r21) / s
        z = 0.25 * s
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    q = (w / n, x / n, y / n, z / n)
    if prev_q is not None:
        if q[0] * prev_q[0] + q[1] * prev_q[1] + q[2] * prev_q[2] + q[3] * prev_q[3] < 0.0:
            q = (-q[0], -q[1], -q[2], -q[3])
    return q


def rot_from_quat(q):
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)),
        (2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)),
        (2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)),
    )


def apply_rt(R, v) -> tuple[float, float, float]:
    """Rotate pad-ENU vector v into body frame: R^T * v."""
    return (
        R[0][0] * v[0] + R[1][0] * v[1] + R[2][0] * v[2],
        R[0][1] * v[0] + R[1][1] * v[1] + R[2][1] * v[2],
        R[0][2] * v[0] + R[1][2] * v[1] + R[2][2] * v[2],
    )


def tilt_deg_from_quat(q) -> float:
    """Angle of the vehicle long axis from vertical (0 = upright or nose-down)."""
    R = rot_from_quat(q)
    bz = _clamp(R[2][1], -1.0, 1.0)
    return math.degrees(math.acos(abs(bz)))


def roll_deg_from_quat(q) -> float:
    """Compass heading of body +X (fin 1) in degrees [0, 360)."""
    R = rot_from_quat(q)
    return math.degrees(math.atan2(R[0][0], R[1][0])) % 360.0


def baro_pressure_from_alt(h_m: float, p0_pa: float) -> float:
    """Exact inverse of Position_MCU's baro_altitude(): h = 44330*(1 - (p/p0)^0.1902949)."""
    ratio = max(0.05, 1.0 - h_m / 44330.0)
    return p0_pa * (ratio ** (1.0 / 0.1902949))


def wgs84_radii(lat_deg: float) -> tuple[float, float]:
    """WGS-84 meridian and prime-vertical*cos radii, matching Position_MCU.ino."""
    lat = math.radians(lat_deg)
    s = math.sin(lat)
    e2 = 6.69437999014e-3
    w = 1.0 - e2 * s * s
    a = 6378137.0
    rw = math.sqrt(w)
    m_per_rad_lon = (a / rw) * math.cos(lat)
    m_per_rad_lat = (a * (1.0 - e2)) / (w * rw)
    return m_per_rad_lat, m_per_rad_lon


# ---------------------------------------------------------------------------
#  Motor curve helper (default from guidance_tables.h or custom CSV)
# ---------------------------------------------------------------------------
class MotorCurve:
    def __init__(self, times=MOTOR_T, forces=MOTOR_F):
        if len(times) < 2 or len(times) != len(forces):
            raise ValueError("Motor curve needs >= 2 (t, F) points")
        pairs = sorted(zip(times, forces), key=lambda p: p[0])
        self.t = [float(p[0]) for p in pairs]
        self.f = [max(0.0, float(p[1])) for p in pairs]
        self.cum = [0.0]
        for i in range(1, len(self.t)):
            dt = self.t[i] - self.t[i - 1]
            self.cum.append(self.cum[-1] + 0.5 * (self.f[i - 1] + self.f[i]) * dt)
        self.burn_t = self.t[-1]
        self.i_total = max(self.cum[-1], 1e-6)
        self.peak_n = max(self.f)

    def thrust(self, tb: float) -> float:
        if tb <= self.t[0] or tb >= self.burn_t:
            return 0.0
        lo, hi = 0, len(self.t) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if self.t[mid] <= tb:
                lo = mid
            else:
                hi = mid
        dt = self.t[hi] - self.t[lo]
        if dt <= 1e-12:
            return self.f[hi]
        frac = (tb - self.t[lo]) / dt
        return self.f[lo] + frac * (self.f[hi] - self.f[lo])

    def burned(self, tb: float, m_prop: float) -> float:
        if tb <= 0.0:
            return 0.0
        if tb >= self.burn_t:
            return m_prop
        lo, hi = 0, len(self.t) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if self.t[mid] <= tb:
                lo = mid
            else:
                hi = mid
        dt = self.t[hi] - self.t[lo]
        if dt <= 1e-12:
            imp = self.cum[hi]
        else:
            frac = (tb - self.t[lo]) / dt
            f_now = self.f[lo] + frac * (self.f[hi] - self.f[lo])
            imp = self.cum[lo] + 0.5 * (self.f[lo] + f_now) * (tb - self.t[lo])
        return m_prop * (imp / self.i_total)


def load_thrust_csv(path: str) -> tuple[tuple[float, ...], tuple[float, ...]]:
    """Load a 2-column CSV (time_s, thrust_N) for a custom landing motor curve."""
    ts, fs = [], []
    with open(path, "r", encoding="utf-8-sig") as fh:
        for row in csv.reader(fh):
            if not row or len(row) < 2:
                continue
            try:
                t = float(row[0].strip().replace(",", "."))
                f = float(row[1].strip().replace(",", "."))
            except ValueError:
                continue
            ts.append(t)
            fs.append(max(0.0, f))
    if len(ts) < 2:
        raise ValueError("CSV tahové křivky musí mít alespoň 2 číselné řádky (čas_s, tah_N).")
    return tuple(ts), tuple(fs)


# ---------------------------------------------------------------------------
#  Configuration & Hardware Feedback structures
# ---------------------------------------------------------------------------
@dataclass
class HilConfig:
    mode: str = "closed"                    # "closed" (real HW commands) or "open" (replay)
    target_apogee_m: float = 150.0          # m above pad (nominal landing envelope <= 190 m)
    wind_e_mps: float = 1.5                 # constant wind East, m/s
    wind_n_mps: float = -0.5                # constant wind North, m/s
    disturb_tilt_deg: float = 3.0           # initial tilt after apogee flip, deg
    disturb_az_deg: float = 45.0            # azimuth of initial tilt, deg
    init_roll_dps: float = 15.0             # initial roll rate entering descent, deg/s
    ign_delay_s: float = 0.25               # igniter delay from HW command to thrust onset, s
    mass_gross_kg: float = DG_M_GROSS
    mass_prop_kg: float = DG_M_PROP
    diameter_m: float = DG_DIAMETER
    cd_body: float = DG_CD_BODY
    fin_dcd_zero: float = DG_FIN_CD_ZERO
    fin_dcd_brake: float = DG_FIN_CD_BRAKE
    thrust_scale: float = 1.0
    drag_mult: float = 1.0
    pad_lat_deg: float = 49.7985411
    pad_lon_deg: float = 16.6912772
    pad_alt_msl: float = 412.3
    pad_pressure_pa: float = 97230.0
    temp_c: float = 21.5
    lidar_offset_m: float = 0.0
    lidar_max_m: float = 7.5
    boost_burn_s: float = 1.2
    sensor_noise: bool = True
    rng_seed: int | None = None
    custom_motor_t: tuple[float, ...] | None = None
    custom_motor_f: tuple[float, ...] | None = None

    @property
    def area_m2(self) -> float:
        return math.pi * (0.5 * self.diameter_m) ** 2


@dataclass
class HwFeedback:
    """Latest high-rate snapshot received from Master MCU (`H {..}` line)."""
    rx_time: float = 0.0
    t_ms: int = 0
    mst_state: int = 1
    nav_phase: int = 0
    mst_flags: int = 0
    nav_events: int = 0
    pyro_active: int = 0
    pyro_fired: int = 0
    cover_open: int = 0
    est_p: tuple[float, float, float] = (0.0, 0.0, 0.0)
    est_v: tuple[float, float, float] = (0.0, 0.0, 0.0)
    est_q: tuple[float, float, float, float] = (0.7071, -0.7071, 0.0, 0.0)
    tvc_deg: tuple[float, float] = (0.0, 0.0)
    clamp: float = 1.0
    fin_deg: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    act_flags: int = 0
    servo_us: tuple[int, ...] = (1500, 1500, 2000, 1100, 1500, 1500, 1500, 1500)
    h_ign: float = 0.0
    thr_est: float = 0.0
    k_plan: float = 1.0
    t_to_ign: float = -1.0
    t_to_td: float = -1.0
    tilt_cmd: float = 0.0
    tilt_err: float = 0.0

    @classmethod
    def from_h_dict(cls, d: dict, rx_time: float = 0.0) -> "HwFeedback":
        p = d.get("p") or [0.0, 0.0, 0.0]
        v = d.get("v") or [0.0, 0.0, 0.0]
        q = d.get("q") or [0.7071, -0.7071, 0.0, 0.0]
        tvc = d.get("tvc") or [0.0, 0.0]
        fin = d.get("fin") or [0.0, 0.0, 0.0, 0.0]
        us = d.get("us") or [1500, 1500, 2000, 1100, 1500, 1500, 1500, 1500]
        return cls(
            rx_time=rx_time,
            t_ms=int(d.get("t", 0)),
            mst_state=int(d.get("st", 1)),
            nav_phase=int(d.get("ph", 0)),
            mst_flags=int(d.get("fl", 0)),
            nav_events=int(d.get("ev", 0)),
            pyro_active=int(d.get("pa", 0)),
            pyro_fired=int(d.get("pf", 0)),
            cover_open=int(d.get("cov", 0)),
            est_p=(float(p[0]), float(p[1]), float(p[2])),
            est_v=(float(v[0]), float(v[1]), float(v[2])),
            est_q=(float(q[0]), float(q[1]), float(q[2]), float(q[3])),
            tvc_deg=(float(tvc[0]), float(tvc[1])),
            clamp=float(d.get("cl", 1.0)),
            fin_deg=(float(fin[0]), float(fin[1]), float(fin[2]), float(fin[3])),
            act_flags=int(d.get("afl", 0)),
            servo_us=tuple(int(x) for x in us[:8]),
            h_ign=float(d.get("hign", 0.0)),
            thr_est=float(d.get("thr", 0.0)),
            k_plan=float(d.get("k", 1.0)),
            t_to_ign=float(d.get("tign", -1.0)),
            t_to_td=float(d.get("ttd", -1.0)),
            tilt_cmd=float(d.get("tc", 0.0)),
            tilt_err=float(d.get("te", 0.0)),
        )

    @property
    def ignite_commanded(self) -> bool:
        return bool(
            (self.act_flags & ACT_IGNITE)
            or (self.nav_events & (EV_IGN1 | EV_IGN2))
            or (self.nav_phase >= 3)
        )

    @property
    def fins_live(self) -> bool:
        return bool(self.act_flags & ACT_FINS_LIVE)


# ---------------------------------------------------------------------------
#  Reference LandSim_Light controller (for open-loop replay & --demo mode)
# ---------------------------------------------------------------------------
class ReferenceGuidance:
    """
    Lightweight faithful port of Ariadna_V2/common/guidance.cpp + ariadna_seq.h.
    Drives open-loop reference HIL runs and the --demo virtual flight computer.
    """

    def __init__(self, cfg: HilConfig, motor: MotorCurve | None = None):
        self.cfg = cfg
        if motor is not None:
            self.motor = motor
        elif cfg.custom_motor_t and cfg.custom_motor_f:
            self.motor = MotorCurve(cfg.custom_motor_t, cfg.custom_motor_f)
        else:
            self.motor = MotorCurve()
        self.reset()

    def reset(self) -> None:
        self.phase = 0           # 0 IDLE, 1 ASCENT, 2 DESCENT, 3 IGNITING, 4 BURN, 5 DONE
        self.h_max = 0.0
        self.h_ign = 0.0
        self.t_ign_cmd = -1.0
        self.t_burn0 = -1.0
        self.a_base = 0.0
        self.k_plan = 1.0
        self.k_act = 1.0
        self.gate = True
        self.t_to_ign = -1.0
        self.t_to_td = -1.0
        self.last_plan_t = -1.0
        self.events = 0
        self.cover_open = 0
        self.pyro_fired = 0
        self.pyro_active = 0
        self.pyro_t_on = [-1.0, -1.0, -1.0]
        self.tvc_deg = (0.0, 0.0)
        self.fin_deg = (0.0, 0.0, 0.0, 0.0)
        self.fins_live = False
        self.tilt_cmd = 0.0
        self.tilt_err = 0.0
        self.thr_est = 0.0
        self.e_int = [0.0, 0.0, 0.0]

    @staticmethod
    def _band_at(vz_abs: float) -> tuple[float, float]:
        f = _clamp((vz_abs - IGN_VZ0) / IGN_DVZ, 0.0, len(IGN_HMIN) - 1)
        i = min(int(f), len(IGN_HMIN) - 2)
        t = f - i
        lo = IGN_HMIN[i] + t * (IGN_HMIN[i + 1] - IGN_HMIN[i])
        a, b = IGN_HMAX[i], IGN_HMAX[i + 1]
        hi = 0.0 if (a <= 0.0 or b <= 0.0) else a + t * (b - a)
        return lo, hi

    def _compute_ign_alt(self, h0: float, vz0: float, vh0: float) -> tuple[float, float]:
        cd_free = self.cfg.cd_body + self.cfg.fin_dcd_brake
        kd = 0.5 * RHO_AIR * self.cfg.area_m2 * cd_free / self.cfg.mass_gross_kg
        dt = 0.01
        h, vx, vz = max(h0, 1.0), abs(vh0), vz0
        h_lo, v_lo, h_hi, no_band = None, vz0, None, False
        for _ in range(4000):
            if h <= 0.5:
                break
            hm, hmx = self._band_at(abs(vz))
            if h_hi is None and hmx > 0.0 and h <= hmx:
                h_hi = h
            if h <= hm:
                h_lo, v_lo, no_band = h, vz, hmx <= 0.0
                break
            v = math.hypot(vx, vz)
            vx += (-kd * v * vx) * dt
            vz += (-kd * v * vz - G_ACCEL) * dt
            h += vz * dt
        if h_lo is None:
            h_lo, v_lo = max(h, 0.5), vz
        d = 0.4
        h_cmd = h_lo + abs(v_lo) * d + 0.5 * G_ACCEL * d * d
        cap = h0 - 1.0 if (no_band or h_hi is None) else h_hi
        h_cmd = min(h_cmd, max(cap, h_lo))
        # Estimate time to h_cmd
        h, vx, vz, tt = h0, abs(vh0), vz0, 0.0
        for _ in range(4000):
            if h <= h_cmd:
                break
            v = math.hypot(vx, vz)
            vx += (-kd * v * vx) * dt
            vz += (-kd * v * vz - G_ACCEL) * dt
            h += vz * dt
            tt += dt
        return h_cmd, tt

    def _project_vz(self, h0: float, vz0: float, tb0: float, k_level: float) -> tuple[float, float]:
        kd = 0.5 * RHO_AIR * self.cfg.area_m2 * (self.cfg.cd_body + self.cfg.fin_dcd_zero)
        dt = 0.02
        h, vz, tb = h0, vz0, tb0
        n_hold = 15   # 0.3 s
        n = 0
        while h > 0.0 and n < 1200:
            tn = self.motor.thrust(tb) * self.cfg.thrust_scale * 0.94
            mass = self.cfg.mass_gross_kg - self.motor.burned(tb, self.cfg.mass_prop_kg)
            if h < 3.0 and vz > -3.0 and tn > 1.6 * mass * G_ACCEL:
                vz_ref = -_clamp(math.sqrt(2.0 * 8.0 * max(h, 0.05)), 0.8, 2.5)
                a_cmd = min(70.0, G_ACCEL + 3.0 * (vz_ref - vz))
                k = _clamp(mass * a_cmd / max(tn, 1.0), DG_CLAMP_MIN, DG_CLAMP_MAX)
            else:
                k = DG_CLAMP_MAX if n >= n_hold else k_level
            a = (tn * k - kd * vz * abs(vz)) / mass - G_ACCEL
            vn = vz + a * dt
            h += 0.5 * (vz + vn) * dt
            vz = vn
            tb += dt
            n += 1
        return vz, tb - tb0

    def _solve_plan(self, h: float, vz: float, tb: float) -> tuple[float, float]:
        target = -0.5
        best_k, best_f, best_t = DG_CLAMP_MIN, -1e9, 0.0
        lo = hi = -1.0
        prev_k = DG_CLAMP_MIN
        for i in range(10):
            k = DG_CLAMP_MIN + (DG_CLAMP_MAX - DG_CLAMP_MIN) * i / 9.0
            vtd, ttd = self._project_vz(h, vz, tb, k)
            f = vtd - target
            if f > best_f:
                best_f, best_k, best_t = f, k, ttd
            if f >= 0.0:
                if i == 0:
                    return DG_CLAMP_MIN, ttd
                lo, hi, best_t = prev_k, k, ttd
                break
            prev_k = k
        if lo < 0.0:
            return best_k, best_t
        for _ in range(8):
            mid = 0.5 * (lo + hi)
            vtd, ttd = self._project_vz(h, vz, tb, mid)
            if vtd - target >= 0.0:
                hi, best_t = mid, ttd
            else:
                lo = mid
        return hi, best_t

    @staticmethod
    def _guidance_dir(h: float, vx: float, vy: float, vz: float, mass: float,
                      area: float, cd_ax: float) -> tuple[float, float, float]:
        avz = abs(vz)
        t_go = max(0.25, 2.0 * max(h, 0.1) / max(avz, 0.5))
        ax = -vx / t_go
        ay = -vy / t_go
        az = avz / t_go + G_ACCEL
        v2 = vx * vx + vy * vy + vz * vz
        if v2 > 1e-4:
            v = math.sqrt(v2)
            df = 0.5 * RHO_AIR * area * cd_ax * v / mass
            ax += df * vx
            ay += df * vy
            az += df * vz
        az = max(0.1, az)
        max_tilt = math.radians(_clamp(h * 1.2 + 1.5, 0.0, 20.0))
        tmax = math.tan(max_tilt)
        rh = math.hypot(ax, ay)
        if rh > tmax * az and rh > 1e-12:
            s = tmax * az / rh
            ax *= s
            ay *= s
        return _unit((ax, ay, az), (0.0, 0.0, 1.0))

    @staticmethod
    def _fin_mix(b, u1, u2, v, tau, tau_roll: float, brake: float,
                 control: bool) -> tuple[float, float, float, float]:
        fin_max = DG_FIN_MAX_DEG
        amax = DG_FIN_STALL_DEG - DG_FIN_STALL_MARGIN
        v2 = _dot(v, v)
        q = 0.5 * RHO_AIR * v2
        k_t = 2.0 * DG_FIN_ARM * q * DG_FIN_AREA * DG_FIN_CL_ALPHA * DG_FIN_K_INT
        k_r = 4.0 * DG_FIN_ROLL_ARM * q * DG_FIN_AREA * DG_FIN_CL_ALPHA * DG_FIN_K_INT
        if math.sqrt(v2) < 8.0:
            k_t = k_r = 0.0
        if not control:
            k_t = 0.0
        a_cmd = _clamp(math.degrees(-_dot(tau, u1) / k_t), -fin_max, fin_max) if k_t > 1e-6 else 0.0
        b_cmd = _clamp(math.degrees(-_dot(tau, u2) / k_t), -fin_max, fin_max) if k_t > 1e-6 else 0.0
        d_roll = _clamp(math.degrees(tau_roll / k_r), -2.0, 2.0) if k_r > 1e-6 else 0.0
        va = max(3.0, -_dot(v, b))
        ctrl = [0.0] * 4
        room = fin_max
        for i in range(4):
            ca = 1.0 if i == 0 else (-1.0 if i == 2 else 0.0)
            sa = 1.0 if i == 1 else (-1.0 if i == 3 else 0.0)
            r = (ca * u1[0] + sa * u2[0], ca * u1[1] + sa * u2[1], ca * u1[2] + sa * u2[2])
            n = _cross(b, r)
            xflow = math.degrees(_dot(v, n) / va)
            xf_c = xflow if k_t > 1e-6 else 0.0
            c = d_roll + a_cmd * ca + b_cmd * sa + xf_c
            a_i = c - xflow
            if a_i > amax:
                c = xflow + amax
            elif a_i < -amax:
                c = xflow - amax
            c = _clamp(c, -fin_max, fin_max)
            ctrl[i] = c
            room = min(room, fin_max - abs(c), amax - abs(c - xflow))
        room = max(0.0, room)
        bk = min(brake, room)
        out = []
        for i in range(4):
            alt = 1.0 if (i % 2 == 0) else -1.0
            out.append(_clamp(ctrl[i] + bk * alt, -fin_max, fin_max))
        return (out[0], out[1], out[2], out[3])

    def step(self, t: float, dt: float, pos, vel, b, g, w, a_axial: float,
             pyro_permitted: bool = False) -> dict:
        """Advance reference guidance + sequencer by dt and return actuator/status dict."""
        h = pos[2]
        vx, vy, vz = vel
        if h > self.h_max:
            self.h_max = h

        # Phase transitions matching Navigation_MCU.ino
        if self.phase == 0 and (h > 1.0 or vz > 3.0):
            self.phase = 1
        if self.phase == 1 and vz < -3.0 and h < self.h_max - 1.5 and self.h_max >= 20.0:
            self.phase = 2
            self.events |= EV_APOGEE
            self.h_ign, self.t_to_ign = self._compute_ign_alt(h, vz, math.hypot(vx, vy))
            self.a_base = a_axial
            self.last_plan_t = t

        if self.phase == 2:
            if t - self.last_plan_t >= 0.1:
                self.h_ign, self.t_to_ign = self._compute_ign_alt(h, vz, math.hypot(vx, vy))
                self.last_plan_t = t
            self.a_base += (a_axial - self.a_base) * min(1.0, dt * 3.0)
            if not self.cover_open and 0.0 <= self.t_to_ign <= 0.6:
                self.cover_open = 1
                self.events |= EV_COVER
            if h <= self.h_ign + 1.0:
                self.phase = 3
                self.t_ign_cmd = t
                self.cover_open = 1
                self.events |= EV_COVER | EV_IGN1
                self.pyro_fired |= 0x01
                self.pyro_t_on[0] = t
                self.t_to_ign = 0.0

        if self.phase == 3:
            if (a_axial - self.a_base) > 8.0 or (t - self.t_ign_cmd) > 1.8:
                self.phase = 4
                self.t_burn0 = t
                self.events |= EV_THRUST
                self.k_plan, self.t_to_td = self._solve_plan(h, vz, 0.0)
                self.last_plan_t = t

        if self.phase == 4:
            tb = max(0.0, t - self.t_burn0)
            if t - self.last_plan_t >= 0.1:
                self.k_plan, self.t_to_td = self._solve_plan(max(h, 0.0), vz, tb)
                self.last_plan_t = t
            else:
                self.t_to_td = max(0.0, self.t_to_td - dt)
            if not (self.events & EV_LEGS) and h <= 45.0 and 0.0 <= self.t_to_td <= 1.7:
                self.events |= EV_LEGS
                self.pyro_fired |= 0x04
                self.pyro_t_on[2] = t
            if h <= 0.25 and vz > -0.5:
                self.phase = 5
                self.events |= EV_TOUCHDOWN

        # Update pyro active pulse levels (inhibited unless pyro_permitted)
        self.pyro_active = 0
        if pyro_permitted:
            durs = (0.35, 0.35, 0.60)
            for ch in range(3):
                if self.pyro_t_on[ch] >= 0.0 and (t - self.pyro_t_on[ch]) < durs[ch]:
                    self.pyro_active |= (1 << ch)

        u1, u2 = gimbal_basis(b, g)
        wr = _dot(w, b)
        wt = (w[0] - wr * b[0], w[1] - wr * b[1], w[2] - wr * b[2])
        tb = max(0.0, t - self.t_burn0) if self.phase == 4 else -1.0
        mass = self.cfg.mass_gross_kg - (self.motor.burned(tb, self.cfg.mass_prop_kg) if tb > 0.0 else 0.0)
        raw_t = self.motor.thrust(tb) * self.cfg.thrust_scale if self.phase == 4 else 0.0
        if self.phase == 4:
            if self.gate and raw_t >= DG_THRUST_GATE * self.motor.peak_n * self.cfg.thrust_scale:
                self.gate = False
            if self.gate:
                k_tgt = DG_CLAMP_MAX
            elif h < 3.0 and vz > -3.0 and raw_t > 1.6 * mass * G_ACCEL:
                vz_ref = -_clamp(math.sqrt(2.0 * 8.0 * max(h, 0.05)), 0.8, 2.5)
                a_cmd = min(70.0, G_ACCEL + 3.0 * (vz_ref - vz))
                k_tgt = _clamp(mass * a_cmd / max(raw_t, 1.0), DG_CLAMP_MIN, DG_CLAMP_MAX)
            else:
                k_tgt = self.k_plan
            self.k_act += _clamp(k_tgt - self.k_act, -DG_CLAMP_RATE * dt, DG_CLAMP_RATE * dt)
        else:
            self.k_act = DG_CLAMP_MAX

        self.thr_est = raw_t * self.k_act
        It = mass * DG_IT_PER_KG
        Ia = mass * DG_IA_PER_KG

        if self.phase in (2, 3):
            # Free-fall fin attitude + roll + airbrake control
            uc = (0.0, 0.0, 1.0)
            if h < self.h_ign + 25.0:
                uc = self._guidance_dir(h, vx, vy, vz, mass, self.cfg.area_m2,
                                        self.cfg.cd_body + self.cfg.fin_dcd_brake)
            self.tilt_cmd = math.acos(_clamp(uc[2], -1.0, 1.0))
            e = _cross(b, uc)
            en = _norm(e)
            if en > 0.3:
                sc = 0.3 / en
                e = (e[0] * sc, e[1] * sc, e[2] * sc)
                en = 0.3
            self.tilt_err = math.asin(_clamp(en, 0.0, 1.0))
            v2 = _dot(vel, vel)
            q_dyn = 0.5 * RHO_AIR * v2
            sched = _clamp((max(q_dyn, 50.0) / 700.0) ** DG_SCHED_FIN, 0.35, 2.0)
            wn = DG_WN_FIN * sched
            k_rate = 2.0 * DG_ZETA_FIN * wn
            k_th = wn / (2.0 * DG_ZETA_FIN)
            tau = (
                It * k_rate * (k_th * e[0] - wt[0]),
                It * k_rate * (k_th * e[1] - wt[1]),
                It * k_rate * (k_th * e[2] - wt[2]),
            )
            tau_roll = -Ia * DG_ROLL_GAIN * wr
            self.fin_deg = self._fin_mix(b, u1, u2, vel, tau, tau_roll, DG_FIN_MAX_DEG, True)
            self.fins_live = True
            self.tvc_deg = (0.0, 0.0)
        elif self.phase == 4 and self.thr_est >= 5.0:
            # Powered burn: TVC dynamic inversion + roll-damping neutral fins
            uc = self._guidance_dir(h, vx, vy, vz, mass, self.cfg.area_m2,
                                    self.cfg.cd_body + self.cfg.fin_dcd_zero)
            self.tilt_cmd = math.acos(_clamp(uc[2], -1.0, 1.0))
            e = _cross(b, uc)
            en = _norm(e)
            if en > 0.3:
                sc = 0.3 / en
                e = (e[0] * sc, e[1] * sc, e[2] * sc)
                en = 0.3
            ec = 1.0 + (en * en) / 6.0
            e = (e[0] * ec, e[1] * ec, e[2] * ec)
            self.tilt_err = en * ec
            sched = _clamp((max(self.thr_est, 20.0) / 100.0) ** DG_SCHED_TVC, 0.35, 2.0)
            wn = DG_WN_TVC * sched
            k_rate = 2.0 * DG_ZETA_TVC * wn
            k_th = wn / (2.0 * DG_ZETA_TVC)
            bw = _cross(b, wt)
            gyk = (It - Ia) * wr
            tau = [
                It * k_rate * (k_th * e[0] - wt[0]) + gyk * bw[0],
                It * k_rate * (k_th * e[1] - wt[1]) + gyk * bw[1],
                It * k_rate * (k_th * e[2] - wt[2]) + gyk * bw[2],
            ]
            # Aero feedforward
            v2 = _dot(vel, vel)
            if v2 > 1.0:
                vr = math.sqrt(v2)
                ah = (-vx / vr, -vy / vr, -vz / vr)
                ca = _dot(ah, b)
                ba = _cross(b, ah)
                kf = 0.5 * RHO_AIR * v2 * self.cfg.area_m2 * DG_CN_ALPHA * abs(ca) * DG_L_CP
                tau[0] -= kf * ba[0]
                tau[1] -= kf * ba[1]
                tau[2] -= kf * ba[2]
            t_inv = max(self.thr_est, 25.0)
            bxt = _cross(b, tau)
            s_vec = (bxt[0] / (t_inv * DG_L_GIMBAL),
                     bxt[1] / (t_inv * DG_L_GIMBAL),
                     bxt[2] / (t_inv * DG_L_GIMBAL))
            sd1 = _clamp(_dot(s_vec, u1), -0.0872, 0.0872)
            sd2 = _clamp(_dot(s_vec, u2), -0.0872, 0.0872)
            c1 = _clamp(math.degrees(math.asin(sd1)) * DG_TVC_RATIO, -DG_SERVO_MAX_DEG, DG_SERVO_MAX_DEG)
            c2 = _clamp(math.degrees(math.asin(sd2)) * DG_TVC_RATIO, -DG_SERVO_MAX_DEG, DG_SERVO_MAX_DEG)
            self.tvc_deg = (c1, c2)
            tau_roll = -Ia * DG_ROLL_GAIN * wr
            self.fin_deg = self._fin_mix(b, u1, u2, vel, (0.0, 0.0, 0.0), tau_roll, 0.0, False)
            self.fins_live = True
        else:
            self.tvc_deg = (0.0, 0.0)
            self.fin_deg = (0.0, 0.0, 0.0, 0.0)
            self.fins_live = False

        afl = ACT_VALID
        if self.fins_live:
            afl |= ACT_FINS_LIVE
        if self.phase >= 3:
            afl |= ACT_IGNITE
        if self.cover_open:
            afl |= ACT_COVER_OPEN
        if self.phase != 4:
            afl |= ACT_TVC_PARK

        # Map nav phase -> Master state
        mst_st = {0: 2, 1: 3, 2: 5, 3: 6, 4: 6, 5: 7}.get(self.phase, 1)
        return {
            "mst_state": mst_st,
            "nav_phase": self.phase,
            "nav_events": self.events,
            "pyro_active": self.pyro_active,
            "pyro_fired": self.pyro_fired,
            "cover_open": self.cover_open,
            "tvc_deg": self.tvc_deg,
            "clamp": self.k_act,
            "fin_deg": self.fin_deg,
            "act_flags": afl,
            "h_ign": self.h_ign,
            "thr_est": self.thr_est,
            "k_plan": self.k_plan,
            "t_to_ign": self.t_to_ign,
            "t_to_td": self.t_to_td,
            "tilt_cmd": self.tilt_cmd,
            "tilt_err": self.tilt_err,
            "ignite": self.phase >= 3,
            "fins_live": self.fins_live,
        }


def pol_ref_thrust(motor: MotorCurve | None = None, thrust_scale: float = 1.0) -> float:
    """Average of time-weighted mean and median thrust of the reference curve."""
    m = motor or MotorCurve()
    burn_t = m.burn_t
    if burn_t <= 1e-6:
        return 100.0
    mean_t = (m.i_total / burn_t) * thrust_scale
    n = 64
    samples = sorted(m.thrust((i + 0.5) * (burn_t / n)) * thrust_scale for i in range(n))
    med_t = 0.5 * (samples[n // 2 - 1] + samples[n // 2])
    return max(8.0, 0.5 * (mean_t + med_t))


def polarity_step(b, g, w, cfg: HilConfig | None = None, uz_hint: float = -1.0,
                  ref_thrust_n: float = 0.0) -> dict:
    """Exact Python mirror of C++ dg_step_polarity() (guidance.cpp).

    Drives TVC as in landing burn (0 deg vertical hold) and fins as in ascent
    (0 deg vertical hold + roll damping) from real attitude (b, g, w).
    ref_thrust_n <= 0 uses pol_ref_thrust() (avg of mean & median).
    """
    c = cfg or HilConfig()
    bn = _norm(b)
    if bn < 0.5:
        return {"tvc_deg": (0.0, 0.0), "fin_deg": (0.0, 0.0, 0.0, 0.0),
                "tilt_err": 0.0, "roll_rate": 0.0, "uz": uz_hint, "fins_live": False,
                "ref_thrust_n": _clamp(ref_thrust_n, 10.0, 250.0) if ref_thrust_n > 0.0 else pol_ref_thrust(thrust_scale=c.thrust_scale)}
    b = (b[0] / bn, b[1] / bn, b[2] / bn)
    uz = 1.0 if b[2] > 0.25 else (-1.0 if b[2] < -0.25 else uz_hint)
    u = (0.0, 0.0, uz)
    u1, u2 = gimbal_basis(b, g)

    e = _cross(b, u)
    cn = _norm(e)
    bu = _dot(b, u)
    if cn > 1e-9:
        ang = math.asin(_clamp(cn, -1.0, 1.0))
        if bu < 0.0:
            ang = math.pi - ang
        sc = ang / cn
        e = (e[0] * sc, e[1] * sc, e[2] * sc)
    else:
        e = (0.0, 0.0, 0.0)
        ang = math.pi if bu < 0.0 else 0.0

    w_roll = _dot(w, b)
    wt = (w[0] - w_roll * b[0], w[1] - w_roll * b[1], w[2] - w_roll * b[2])
    I_t = c.mass_gross_kg * DG_IT_PER_KG
    I_a = c.mass_gross_kg * DG_IA_PER_KG

    # 1) TVC: landing-burn 0 deg attitude-hold at reference thrust (default: avg of mean & median)
    t_est = (_clamp(ref_thrust_n, 10.0, 250.0) if ref_thrust_n > 0.0
             else pol_ref_thrust(thrust_scale=c.thrust_scale))
    servo_quant = 0.15
    fac_t = (t_est / 100.0) ** DG_SCHED_TVC if DG_SCHED_TVC != 0.0 else 1.0
    wn_t = DG_WN_TVC * _clamp(fac_t, 0.35, 2.0)
    k_rate_t = 2.0 * DG_ZETA_TVC * wn_t
    k_th_t = wn_t / (2.0 * DG_ZETA_TVC)
    tr_t = tuple(I_t * k_rate_t * (k_th_t * e[i] - wt[i]) for i in range(3))
    sq = _cross(b, tr_t)
    inv = 1.0 / (t_est * DG_L_GIMBAL)
    sd1 = _clamp(_dot(sq, u1) * inv, -0.999, 0.999)
    sd2 = _clamp(_dot(sq, u2) * inv, -0.999, 0.999)
    cmd1 = _clamp(math.degrees(math.asin(sd1)) * DG_TVC_RATIO, -DG_SERVO_MAX_DEG, DG_SERVO_MAX_DEG)
    cmd2 = _clamp(math.degrees(math.asin(sd2)) * DG_TVC_RATIO, -DG_SERVO_MAX_DEG, DG_SERVO_MAX_DEG)
    cmd1 = math.floor(cmd1 / servo_quant + 0.5) * servo_quant
    cmd2 = math.floor(cmd2 / servo_quant + 0.5) * servo_quant

    # 2) Fins: ascent 0 deg attitude-hold + roll damping at nominal axial airspeed
    v_ref = 22.0
    v = (-v_ref * b[0], -v_ref * b[1], -v_ref * b[2])
    q = 0.5 * RHO_AIR * v_ref * v_ref
    fac = (q / 700.0) ** DG_SCHED_FIN if DG_SCHED_FIN != 0.0 else 1.0
    wn_f = DG_WN_FIN * _clamp(fac, 0.35, 2.0)
    k_rate_f = 2.0 * DG_ZETA_FIN * wn_f
    k_th_f = wn_f / (2.0 * DG_ZETA_FIN)
    tr_f = tuple(I_t * k_rate_f * (k_th_f * e[i] - wt[i]) for i in range(3))
    tau_roll = I_a * DG_ROLL_GAIN * (0.0 - w_roll)
    fins = ReferenceGuidance._fin_mix(b, u1, u2, v, tr_f, tau_roll, 0.0, True)

    return {
        "tvc_deg": (cmd1, cmd2),
        "fin_deg": fins,
        "tilt_err": ang,
        "roll_rate": w_roll,
        "uz": uz,
        "fins_live": True,
        "ref_thrust_n": t_est,
    }


# ---------------------------------------------------------------------------
#  6-DOF LandSim_Light HIL flight plant
# ---------------------------------------------------------------------------
class HilPlant:
    """
    6-DOF rocket flight dynamics + synthetic sensor generator for HIL.

    Flight timeline:
      PAD         -> stationary upright on pad (used during 5 s countdown)
      ASCENT      -> booster burn (auto-calibrated to reach target_apogee_m)
                     + ballistic coast up to apogee
      APOGEE_FLIP -> smooth 1.4 s aerodynamic nose-over from nose-up to
                     nose-down with kinematically consistent gyro & quaternion
      DESCENT     -> 6-DOF free fall with wind, body aero & 4 all-moving fins
      IGNITING    -> hardware ignition command received, waiting ign_delay_s
      BURN        -> landing motor thrust curve + clamp + TVC + fins + mass burn
      LANDED      -> touchdown clamped at pz = 0
    """

    def __init__(self, cfg: HilConfig | None = None):
        self.cfg = cfg or HilConfig()
        self.reset(self.cfg)

    def reset(self, cfg: HilConfig | None = None) -> None:
        if cfg is not None:
            self.cfg = cfg
        c = self.cfg
        if c.custom_motor_t and c.custom_motor_f:
            self.motor = MotorCurve(c.custom_motor_t, c.custom_motor_f)
        else:
            self.motor = MotorCurve(MOTOR_T, MOTOR_F)
        self.rng = random.Random(c.rng_seed)
        self.m_per_rad_lat, self.m_per_rad_lon = wgs84_radii(c.pad_lat_deg)
        self.pad_lat_1e7 = int(round(c.pad_lat_deg * 1e7))
        self.pad_lon_1e7 = int(round(c.pad_lon_deg * 1e7))
        self.ref_ctrl = ReferenceGuidance(c, self.motor)

        self.t = 0.0
        self.seq = 0
        self.phase = "PAD"
        self.pos = [0.0, 0.0, 0.0]
        self.vel = [0.0, 0.0, 0.0]
        # Upright on pad: thrust axis b = (0,0,-1) (nose +z), fin1 g = (1,0,0)
        self.b = [0.0, 0.0, -1.0]
        self.g = [1.0, 0.0, 0.0]
        self.w = [0.0, 0.0, 0.0]
        self.q = quat_from_rot(rot_from_bg(self.b, self.g))
        self.a_pad = [0.0, 0.0, 0.0]
        self.a_axial = -G_ACCEL
        self.mass = c.mass_gross_kg
        self.thrust_n = 0.0
        self.apogee_m = 0.0
        self.td_vz = 0.0
        self.td_vh = 0.0
        self.td_tilt_deg = 0.0
        self.landed_time = -1.0

        # Actuator physical states + 15 ms transport delay FIFO (3 x 5 ms slots)
        self.srv1 = 0.0
        self.srv2 = 0.0
        self.defl = [0.0, 0.0, 0.0, 0.0]
        self.k_act = 1.0
        self._cmd_fifo = [(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)] * 2

        # Flip & ignition timers
        self._flip_t0 = -1.0
        self._flip_dur = 1.4
        self._flip_axis = (1.0, 0.0, 0.0)
        self._flip_angle = math.pi
        self._flip_b0 = (0.0, 0.0, -1.0)
        self._flip_g0 = (1.0, 0.0, 0.0)
        self._t_ign_cmd = -1.0
        self._t_burn0 = -1.0
        self.h_ign_true = 0.0
        self._tick = 0
        self._boost_thrust_n = self._calibrate_booster(c.target_apogee_m)

    def _calibrate_booster(self, target_h: float) -> float:
        """Find constant booster thrust over boost_burn_s that reaches target_apogee_m."""
        c = self.cfg
        target_h = _clamp(target_h, 25.0, 500.0)
        kd = 0.5 * RHO_AIR * c.area_m2 * (c.cd_body + c.fin_dcd_zero) * c.drag_mult / c.mass_gross_kg
        dt = 0.01

        def sim_apogee(f_boost: float) -> float:
            h, vz, t = 0.0, 0.0, 0.0
            while t < 30.0:
                a = (f_boost / c.mass_gross_kg if t < c.boost_burn_s else 0.0) - G_ACCEL - kd * vz * abs(vz)
                vz += a * dt
                h += vz * dt
                t += dt
                if t > c.boost_burn_s and vz <= 0.0:
                    break
            return h

        lo, hi = c.mass_gross_kg * G_ACCEL * 2.5, c.mass_gross_kg * G_ACCEL * 45.0
        for _ in range(28):
            mid = 0.5 * (lo + hi)
            if sim_apogee(mid) < target_h:
                lo = mid
            else:
                hi = mid
        return 0.5 * (lo + hi)

    def launch(self) -> None:
        """Transition from PAD into booster ASCENT."""
        if self.phase == "PAD":
            self.phase = "ASCENT"
            self.t = 0.0

    def _noise(self, sigma: float) -> float:
        return self.rng.gauss(0.0, sigma) if self.cfg.sensor_noise else 0.0

    def _build_sample(self, reset_est: bool = False, set_pad_ref: bool = False) -> HilSensorSample:
        """Synthesize a Position-MCU HIL sensor frame from the current true state."""
        c = self.cfg
        self.seq += 1
        self._tick += 1
        R = rot_from_bg(self.b, self.g)
        self.q = quat_from_rot(R, self.q)

        # Specific force in body axes: R^T * (a_pad + [0, 0, g])
        sf_pad = (self.a_pad[0], self.a_pad[1], self.a_pad[2] + G_ACCEL)
        acc_b = apply_rt(R, sf_pad)
        gyr_b = apply_rt(R, self.w)

        acc_noisy = (
            acc_b[0] + self._noise(0.04),
            acc_b[1] + self._noise(0.04),
            acc_b[2] + self._noise(0.04),
        )
        gyr_noisy = (
            gyr_b[0] + self._noise(0.002),
            gyr_b[1] + self._noise(0.002),
            gyr_b[2] + self._noise(0.002),
        )

        flags = 0
        if reset_est:
            flags |= HIL_FLAG_RESET_EST
        if set_pad_ref:
            flags |= HIL_FLAG_SET_PAD_REF

        # Barometer @ 50 Hz (every 2nd tick) + always on pad-ref / reset
        baro_fresh = reset_est or set_pad_ref or (self._tick % 2 == 0)
        p_pa = baro_pressure_from_alt(self.pos[2], c.pad_pressure_pa)
        if not (reset_est or set_pad_ref):
            p_pa += self._noise(1.5)
        if baro_fresh:
            flags |= HIL_FLAG_BARO_FRESH

        # GPS @ 5 Hz (every 20th tick) + always on pad-ref / reset
        gps_fresh = reset_est or set_pad_ref or (self._tick % 20 == 0)
        px_g = self.pos[0] + (0.0 if (reset_est or set_pad_ref) else self._noise(0.35))
        py_g = self.pos[1] + (0.0 if (reset_est or set_pad_ref) else self._noise(0.35))
        pz_g = self.pos[2] + (0.0 if (reset_est or set_pad_ref) else self._noise(0.60))
        dlat_deg = math.degrees(py_g / self.m_per_rad_lat)
        dlon_deg = math.degrees(px_g / self.m_per_rad_lon)
        lat_1e7 = self.pad_lat_1e7 + int(round(dlat_deg * 1e7))
        lon_1e7 = self.pad_lon_1e7 + int(round(dlon_deg * 1e7))
        ve_cms = _clamp_i16((self.vel[0] + self._noise(0.08)) * 100.0)
        vn_cms = _clamp_i16((self.vel[1] + self._noise(0.08)) * 100.0)
        if gps_fresh:
            flags |= HIL_FLAG_GPS_FRESH

        # UWB 4 anchor slant ranges @ 10 Hz (every 10th tick) + always on pad-ref
        uwb_fresh = reset_est or set_pad_ref or (self._tick % 10 == 0)
        uwb_cm = []
        uwb_mask = 0
        for idx, anc in enumerate(UWB_ANCHORS):
            rng_m = math.sqrt(
                (self.pos[0] - anc[0]) ** 2
                + (self.pos[1] - anc[1]) ** 2
                + (self.pos[2] - anc[2]) ** 2
            )
            if not (reset_est or set_pad_ref):
                rng_m += self._noise(0.06)
            if 0.05 < rng_m < 290.0:
                uwb_cm.append(_clamp_u16(rng_m * 100.0))
                uwb_mask |= (1 << idx)
            else:
                uwb_cm.append(0)
        if uwb_fresh and uwb_mask:
            flags |= HIL_FLAG_UWB_FRESH

        # TF-Luna lidar on the nose: valid when nose points downward (b[2] > 0.7)
        # and slant distance <= 7.8 m (Position_MCU gates fusion on EV_LEGS + delay)
        lidar_mm = 0
        lidar_strength = 0
        cos_beam = self.b[2]  # nose = -b, downward component = -(-b[2]) = b[2]
        if cos_beam > 0.70 and self.pos[2] >= 0.0:
            slant_m = (self.pos[2] - c.lidar_offset_m) / cos_beam + self._noise(0.015)
            if 0.20 <= slant_m <= 7.80:
                lidar_mm = _clamp_u16(slant_m * 1000.0)
                lidar_strength = 12000
                flags |= HIL_FLAG_LIDAR_VALID

        return HilSensorSample(
            seq=self.seq,
            qw=self.q[0], qx=self.q[1], qy=self.q[2], qz=self.q[3],
            acc_body=acc_noisy,
            gyr_body=gyr_noisy,
            pressure_pa=p_pa,
            gps_lat_1e7=lat_1e7,
            gps_lon_1e7=lon_1e7,
            gps_alt_msl=c.pad_alt_msl + pz_g,
            gps_ve_cms=ve_cms,
            gps_vn_cms=vn_cms,
            lidar_mm=lidar_mm,
            lidar_strength=lidar_strength,
            uwb_range_cm=(uwb_cm[0], uwb_cm[1], uwb_cm[2], uwb_cm[3]),
            temp_c_x10=_clamp_i16(c.temp_c * 10.0),
            flags=flags,
            gps_sats=12,
            gps_fix=3,
            gps_hdop_x10=9,
            uwb_mask=uwb_mask,
        )

    def make_pad_sample(self, reset_est: bool = False, set_pad_ref: bool = False) -> HilSensorSample:
        """Generate a stationary pad frame (called at 100 Hz during countdown)."""
        self.pos = [0.0, 0.0, 0.0]
        self.vel = [0.0, 0.0, 0.0]
        self.a_pad = [0.0, 0.0, 0.0]
        self.w = [0.0, 0.0, 0.0]
        self.b = [0.0, 0.0, -1.0]
        self.g = [1.0, 0.0, 0.0]
        self.thrust_n = 0.0
        self.a_axial = -G_ACCEL
        return self._build_sample(reset_est=reset_est, set_pad_ref=set_pad_ref)

    def _begin_flip(self) -> None:
        """Set up smooth nose-over rotation from current upright b to disturbed nose-down b."""
        c = self.cfg
        self.phase = "APOGEE_FLIP"
        self._flip_t0 = self.t
        self._flip_b0 = tuple(self.b)
        self._flip_g0 = tuple(self.g)
        tilt_r = math.radians(c.disturb_tilt_deg)
        az_r = math.radians(c.disturb_az_deg)
        # Target nose-down thrust axis b (pointing mostly +Z with disturbance tilt)
        b_tgt = (
            math.sin(tilt_r) * math.sin(az_r),
            math.sin(tilt_r) * math.cos(az_r),
            math.cos(tilt_r),
        )
        dot_val = _clamp(_dot(self._flip_b0, b_tgt), -1.0, 1.0)
        self._flip_angle = math.acos(dot_val)
        axis = _cross(self._flip_b0, b_tgt)
        if _norm(axis) < 1e-6:
            # Exactly 180 deg: pitch over about East-ish axis perpendicular to b0
            axis = (math.cos(az_r), -math.sin(az_r), 0.0)
        self._flip_axis = _unit(axis, (1.0, 0.0, 0.0))

    def step(self, dt: float = 0.01, hw: HwFeedback | None = None) -> HilSensorSample:
        """
        Advance 6-DOF flight dynamics by `dt` (typically 0.01 s = 100 Hz) and
        return the next HilSensorSample to send to Position MCU.
        """
        c = self.cfg
        if self.phase == "PAD":
            return self.make_pad_sample()
        if self.phase == "LANDED":
            self.t += dt
            self.vel = [0.0, 0.0, 0.0]
            self.w = [0.0, 0.0, 0.0]
            self.a_pad = [0.0, 0.0, 0.0]
            self.thrust_n = 0.0
            self.a_axial = G_ACCEL * self.b[2]
            return self._build_sample()

        # Decide control commands (closed-loop HW vs open-loop reference)
        if c.mode == "open" or hw is None:
            ref = self.ref_ctrl.step(
                self.t, dt, self.pos, self.vel, self.b, self.g, self.w, self.a_axial
            )
            cmd_tvc1, cmd_tvc2 = ref["tvc_deg"]
            cmd_fins = ref["fin_deg"] if ref["fins_live"] else (0.0, 0.0, 0.0, 0.0)
            cmd_clamp = ref["clamp"]
            cmd_ignite = ref["ignite"]
        else:
            cmd_tvc1, cmd_tvc2 = hw.tvc_deg
            cmd_fins = hw.fin_deg if hw.fins_live else (0.0, 0.0, 0.0, 0.0)
            cmd_clamp = hw.clamp
            cmd_ignite = hw.ignite_commanded

        # 15 ms transport delay FIFO (10 ms ticks -> 1..2 tick delay)
        self._cmd_fifo.append((cmd_tvc1, cmd_tvc2, cmd_fins[0], cmd_fins[1], cmd_fins[2], cmd_fins[3], cmd_clamp))
        act = self._cmd_fifo.pop(0)

        # Latch ignition command in DESCENT/APOGEE_FLIP
        if cmd_ignite and self._t_ign_cmd < 0.0 and self.phase in ("APOGEE_FLIP", "DESCENT"):
            self._t_ign_cmd = self.t
            self.h_ign_true = self.pos[2]
            self.phase = "IGNITING"

        n_sub = max(1, int(round(dt / 0.001)))
        sub_dt = dt / n_sub
        area = c.area_m2
        stall = math.radians(DG_FIN_STALL_DEG)

        for _ in range(n_sub):
            if self.phase == "LANDED":
                break

            # Slew-limit actuators
            def _slew(cur: float, tgt: float, rate: float) -> float:
                d = tgt - cur
                s = rate * sub_dt
                return cur + (s if d > s else (-s if d < -s else d))

            self.srv1 = _slew(self.srv1, act[0], 500.0)
            self.srv2 = _slew(self.srv2, act[1], 500.0)
            for i in range(4):
                self.defl[i] = _slew(self.defl[i], act[2 + i], 600.0)
            self.k_act = _slew(self.k_act, act[6], DG_CLAMP_RATE)

            # Check landing motor light-up after igniter delay
            if self._t_ign_cmd >= 0.0 and self._t_burn0 < 0.0:
                if (self.t - self._t_ign_cmd) >= c.ign_delay_s:
                    self._t_burn0 = self.t
                    self.phase = "BURN"

            tb = (self.t - self._t_burn0) if self._t_burn0 >= 0.0 else -1.0
            engine_on = (0.0 <= tb <= self.motor.burn_t)

            if self.phase == "ASCENT":
                # Booster pushes along NOSE (-b, upward); small weathercocking into wind
                boosting = self.t < c.boost_burn_s
                f_boost = self._boost_thrust_n if boosting else 0.0
                self.mass = c.mass_gross_kg
                self.thrust_n = f_boost
                rv = (self.vel[0] - c.wind_e_mps, self.vel[1] - c.wind_n_mps, self.vel[2])
                vr = _norm(rv)
                cd_asc = (c.cd_body + c.fin_dcd_zero) * c.drag_mult
                if vr > 0.05:
                    f_drag = 0.5 * RHO_AIR * vr * area * cd_asc
                    fx = -f_drag * rv[0] - f_boost * self.b[0]
                    fy = -f_drag * rv[1] - f_boost * self.b[1]
                    fz = -f_drag * rv[2] - f_boost * self.b[2]
                else:
                    fx, fy, fz = -f_boost * self.b[0], -f_boost * self.b[1], -f_boost * self.b[2]
                self.a_pad = [fx / self.mass, fy / self.mass, fz / self.mass - G_ACCEL]
                self.a_axial = _dot((fx, fy, fz), self.b) / self.mass

                # Slight natural wind tilt during ascent (max ~3 deg into wind)
                if boosting and vr > 5.0:
                    w_mag = math.hypot(c.wind_e_mps, c.wind_n_mps)
                    lean = math.radians(min(3.0, 0.35 * w_mag))
                    if w_mag > 0.1:
                        ux_w, uy_w = -c.wind_e_mps / w_mag, -c.wind_n_mps / w_mag
                        b_des = (-math.sin(lean) * ux_w, -math.sin(lean) * uy_w, -math.cos(lean))
                        err = _cross(self.b, b_des)
                        self.w = [2.0 * err[0], 2.0 * err[1], 2.0 * err[2]]
                    else:
                        self.w = [0.0, 0.0, 0.0]
                else:
                    self.w = [0.0, 0.0, 0.0]

                self.vel[0] += self.a_pad[0] * sub_dt
                self.vel[1] += self.a_pad[1] * sub_dt
                self.vel[2] += self.a_pad[2] * sub_dt
                self.pos[0] += self.vel[0] * sub_dt
                self.pos[1] += self.vel[1] * sub_dt
                self.pos[2] += self.vel[2] * sub_dt
                self.apogee_m = max(self.apogee_m, self.pos[2])

                db = _cross(self.w, self.b)
                dg = _cross(self.w, self.g)
                self.b = list(_unit((self.b[0] + db[0] * sub_dt, self.b[1] + db[1] * sub_dt, self.b[2] + db[2] * sub_dt)))
                u1, _ = gimbal_basis(self.b, (self.g[0] + dg[0] * sub_dt, self.g[1] + dg[1] * sub_dt, self.g[2] + dg[2] * sub_dt))
                self.g = list(u1)

                self.t += sub_dt
                if not boosting and self.vel[2] <= 1.0:
                    self._begin_flip()
                continue

            if self.phase == "APOGEE_FLIP":
                # Smooth cosine-bell pitch-over from upright to nose-down
                tau_f = _clamp((self.t - self._flip_t0) / self._flip_dur, 0.0, 1.0)
                theta = 0.5 * self._flip_angle * (1.0 - math.cos(math.pi * tau_f))
                dtheta_dt = (0.5 * math.pi * self._flip_angle / self._flip_dur) * math.sin(math.pi * tau_f)
                self.b = list(_rodrigues(self._flip_b0, self._flip_axis, theta))
                self.g = list(_rodrigues(self._flip_g0, self._flip_axis, theta))
                roll_r = math.radians(c.init_roll_dps) * tau_f
                self.w = [
                    self._flip_axis[0] * dtheta_dt + self.b[0] * roll_r,
                    self._flip_axis[1] * dtheta_dt + self.b[1] * roll_r,
                    self._flip_axis[2] * dtheta_dt + self.b[2] * roll_r,
                ]
                rv = (self.vel[0] - c.wind_e_mps, self.vel[1] - c.wind_n_mps, self.vel[2])
                vr = _norm(rv)
                f_drag = 0.5 * RHO_AIR * vr * area * (c.cd_body + c.fin_dcd_zero) * c.drag_mult
                fx, fy, fz = -f_drag * rv[0], -f_drag * rv[1], -f_drag * rv[2]
                self.mass = c.mass_gross_kg
                self.thrust_n = 0.0
                self.a_pad = [fx / self.mass, fy / self.mass, fz / self.mass - G_ACCEL]
                self.a_axial = _dot((fx, fy, fz), self.b) / self.mass
                self.vel[0] += self.a_pad[0] * sub_dt
                self.vel[1] += self.a_pad[1] * sub_dt
                self.vel[2] += self.a_pad[2] * sub_dt
                self.pos[0] += self.vel[0] * sub_dt
                self.pos[1] += self.vel[1] * sub_dt
                self.pos[2] += self.vel[2] * sub_dt
                self.apogee_m = max(self.apogee_m, self.pos[2])
                self.t += sub_dt
                if tau_f >= 1.0:
                    self.phase = "DESCENT"
                    self.w = [self.b[0] * roll_r, self.b[1] * roll_r, self.b[2] * roll_r]
                continue

            # ---- 6-DOF DESCENT / IGNITING / BURN (exact LandSim_Light / guid_test.cpp equations) ----
            self.mass = c.mass_gross_kg - (self.motor.burned(tb, c.mass_prop_kg) if tb > 0.0 else 0.0)
            self.thrust_n = self.motor.thrust(tb) * c.thrust_scale * self.k_act if engine_on else 0.0

            b = self.b
            u1, u2 = gimbal_basis(b, self.g)
            s1 = math.sin(math.radians(self.srv1 / DG_TVC_RATIO))
            s2 = math.sin(math.radians(self.srv2 / DG_TVC_RATIO))
            sv = (s1 * u1[0] + s2 * u2[0], s1 * u1[1] + s2 * u2[1], s1 * u1[2] + s2 * u2[2])
            ca_n = math.sqrt(max(0.0, 1.0 - _dot(sv, sv)))
            td = (ca_n * b[0] + sv[0], ca_n * b[1] + sv[1], ca_n * b[2] + sv[2])

            rv = (self.vel[0] - c.wind_e_mps, self.vel[1] - c.wind_n_mps, self.vel[2])
            v2 = _dot(rv, rv)
            vr = math.sqrt(v2)
            q_dyn = 0.5 * RHO_AIR * v2
            F = [0.0, 0.0, 0.0]
            T = [0.0, 0.0, 0.0]
            if vr > 0.05:
                ah = (-rv[0] / vr, -rv[1] / vr, -rv[2] / vr)
                ca = _dot(ah, b)
                bc = _cross(b, ah)
                fnk = q_dyn * area * DG_CN_ALPHA * abs(ca)
                cdamp = 0.5 * RHO_AIR * vr * area * DG_CN_ALPHA * DG_L_CP * DG_L_CP
                wr = _dot(self.w, b)
                sa = math.sqrt(max(0.0, 1.0 - ca * ca))
                fvk = q_dyn * 0.8 * (1.4 * c.diameter_m) * sa
                for i in range(3):
                    perp = ah[i] - ca * b[i]
                    F[i] += q_dyn * area * c.cd_body * c.drag_mult * ca * b[i] + (fnk + fvk) * perp
                    T[i] += DG_L_CP * fnk * bc[i] - cdamp * (self.w[i] - wr * b[i])

            # 4 all-moving fins
            for i in range(4):
                ca_f = 1.0 if i == 0 else (-1.0 if i == 2 else 0.0)
                sa_f = 1.0 if i == 1 else (-1.0 if i == 3 else 0.0)
                r = (ca_f * u1[0] + sa_f * u2[0], ca_f * u1[1] + sa_f * u2[1], ca_f * u1[2] + sa_f * u2[2])
                n = _cross(b, r)
                pp = (DG_FIN_ARM * b[0] + DG_FIN_ROLL_ARM * r[0],
                      DG_FIN_ARM * b[1] + DG_FIN_ROLL_ARM * r[1],
                      DG_FIN_ARM * b[2] + DG_FIN_ROLL_ARM * r[2])
                wxp = _cross(self.w, pp)
                lv_vec = (rv[0] + wxp[0], rv[1] + wxp[1], rv[2] + wxp[2])
                l2 = _dot(lv_vec, lv_vec)
                if l2 < 1.0:
                    continue
                lv = math.sqrt(l2)
                va = max(2.0, -_dot(lv_vec, b))
                alpha = math.radians(self.defl[i]) - _dot(lv_vec, n) / va
                if abs(alpha) <= stall:
                    cl = DG_FIN_CL_ALPHA * alpha
                else:
                    sgn = 1.0 if alpha > 0.0 else -1.0
                    cl = sgn * DG_FIN_CL_ALPHA * stall * math.exp(-(abs(alpha) - stall) * 1.5)
                cdf = 0.048 + 0.45 * cl * cl
                lift = 0.5 * RHO_AIR * l2 * DG_FIN_AREA * cl * DG_FIN_K_INT
                drag = 0.5 * RHO_AIR * l2 * DG_FIN_AREA * cdf * c.drag_mult
                ln = _dot(lv_vec, n) / l2
                pr = (n[0] - ln * lv_vec[0], n[1] - ln * lv_vec[1], n[2] - ln * lv_vec[2])
                pn = max(1e-9, _norm(pr))
                ff = (lift * pr[0] / pn - drag * lv_vec[0] / lv,
                      lift * pr[1] / pn - drag * lv_vec[1] / lv,
                      lift * pr[2] / pn - drag * lv_vec[2] / lv)
                tq = _cross(pp, ff)
                for k in range(3):
                    F[k] += ff[k]
                    T[k] += tq[k]

            # Thrust & gimbal torque (-L_GIMBAL * T * (b x s))
            bs = _cross(b, sv)
            fsum = (self.thrust_n * td[0] + F[0],
                    self.thrust_n * td[1] + F[1],
                    self.thrust_n * td[2] + F[2])
            self.a_axial = _dot(fsum, b) / self.mass
            self.a_pad = [fsum[0] / self.mass, fsum[1] / self.mass, fsum[2] / self.mass - G_ACCEL]

            It = self.mass * DG_IT_PER_KG
            Ia = self.mass * DG_IA_PER_KG
            wr = _dot(self.w, b)
            wt = (self.w[0] - wr * b[0], self.w[1] - wr * b[1], self.w[2] - wr * b[2])
            bw = _cross(b, wt)
            tot = (
                -DG_L_GIMBAL * self.thrust_n * bs[0] + T[0] - (It - Ia) * wr * bw[0],
                -DG_L_GIMBAL * self.thrust_n * bs[1] + T[1] - (It - Ia) * wr * bw[1],
                -DG_L_GIMBAL * self.thrust_n * bs[2] + T[2] - (It - Ia) * wr * bw[2],
            )
            trr = _dot(tot, b)
            al = (
                (tot[0] - trr * b[0]) / It + trr * b[0] / Ia,
                (tot[1] - trr * b[1]) / It + trr * b[1] / Ia,
                (tot[2] - trr * b[2]) / It + trr * b[2] / Ia,
            )

            # Integrate with exact touchdown clamping
            dts = sub_dt
            touchdown = (self.pos[2] + self.vel[2] * sub_dt <= 0.0) and (self.vel[2] <= 0.0)
            if touchdown and self.vel[2] < -1e-9:
                dts = min(sub_dt, max(0.0, self.pos[2] / (-self.vel[2])))

            for i in range(3):
                self.vel[i] += self.a_pad[i] * dts
                self.pos[i] += self.vel[i] * dts
                self.w[i] += al[i] * dts

            db = _cross(self.w, b)
            dg = _cross(self.w, self.g)
            self.b = list(_unit((b[0] + db[0] * dts, b[1] + db[1] * dts, b[2] + db[2] * dts)))
            u1_new, _ = gimbal_basis(self.b, (self.g[0] + dg[0] * dts, self.g[1] + dg[1] * dts, self.g[2] + dg[2] * dts))
            self.g = list(u1_new)
            self.t += dts

            if touchdown or self.pos[2] <= 0.0:
                self.pos[2] = 0.0
                self.td_vz = self.vel[2]
                self.td_vh = math.hypot(self.vel[0], self.vel[1])
                self.td_tilt_deg = math.degrees(math.acos(_clamp(abs(self.b[2]), 0.0, 1.0)))
                self.landed_time = self.t
                self.vel = [0.0, 0.0, 0.0]
                self.w = [0.0, 0.0, 0.0]
                self.a_pad = [0.0, 0.0, 0.0]
                self.thrust_n = 0.0
                self.phase = "LANDED"
                break

        return self._build_sample()


# ---------------------------------------------------------------------------
#  Synchronized HIL flight recorder & post-flight verification summary
# ---------------------------------------------------------------------------
CSV_COLUMNS = [
    # Simulation truth
    "t_sim", "sim_phase",
    "true_px", "true_py", "true_pz",
    "true_vx", "true_vy", "true_vz",
    "true_tilt_deg", "true_roll_deg", "true_mass_kg", "true_thrust_N", "true_a_axial",
    # All HIL sensor inputs injected into Position MCU
    "in_seq", "in_qw", "in_qx", "in_qy", "in_qz",
    "in_ax_body", "in_ay_body", "in_az_body",
    "in_gx_body", "in_gy_body", "in_gz_body",
    "in_p_pa", "in_gps_lat_1e7", "in_gps_lon_1e7", "in_gps_alt_msl",
    "in_gps_ve_cms", "in_gps_vn_cms", "in_lidar_mm",
    "in_uwb0_cm", "in_uwb1_cm", "in_uwb2_cm", "in_uwb3_cm", "in_flags",
    # Hardware estimated state & mission/guidance telemetry from Master H line
    "hw_t_ms", "mst_state", "nav_phase", "mst_flags", "nav_events",
    "pyro_active", "pyro_fired", "cover_open",
    "est_px", "est_py", "est_pz", "est_vx", "est_vy", "est_vz",
    "est_qw", "est_qx", "est_qy", "est_qz", "est_tilt_deg",
    "h_ign_m", "thr_est_N", "k_plan", "t_to_ign_s", "t_to_td_s",
    "tilt_cmd_deg", "tilt_err_deg",
    # All hardware commanded actuator values (deg/clamp + exact PCA9685 us)
    "cmd_tvc_x_deg", "cmd_tvc_y_deg", "cmd_clamp",
    "cmd_fin0_deg", "cmd_fin1_deg", "cmd_fin2_deg", "cmd_fin3_deg", "act_flags",
    "us_tvc_x", "us_tvc_y", "us_clamp", "us_cover",
    "us_fin0", "us_fin1", "us_fin2", "us_fin3",
]


@dataclass
class HilSummary:
    samples: int = 0
    duration_s: float = 0.0
    true_apogee_m: float = 0.0
    est_apogee_m: float = 0.0
    alt_rmse_m: float = 0.0
    alt_max_err_m: float = 0.0
    horiz_max_err_m: float = 0.0
    h_ign_true_m: float = 0.0
    h_ign_hw_m: float = 0.0
    td_vz_mps: float = 0.0
    td_vh_mps: float = 0.0
    td_tilt_deg: float = 0.0
    max_tvc_deg: float = 0.0
    max_fin_deg: float = 0.0
    min_clamp: float = 1.0
    max_clamp: float = 1.0
    events_seen: int = 0
    pyro_fired_mask: int = 0
    cover_opened: bool = False
    legs_fired: bool = False
    landed: bool = False
    verdict: str = "OK"


class HilRecorder:
    """Stores every tick's truth, HIL sensor input, HW estimate, and actuator PWM."""

    def __init__(self):
        self.rows: list[list] = []

    def clear(self) -> None:
        self.rows.clear()

    def record(self, plant: HilPlant, sample: HilSensorSample, hw: HwFeedback | None) -> None:
        h = hw or HwFeedback()
        true_tilt = tilt_deg_from_quat(plant.q)
        true_roll = roll_deg_from_quat(plant.q)
        est_tilt = tilt_deg_from_quat(h.est_q)
        us = h.servo_us if len(h.servo_us) >= 8 else (0,) * 8
        self.rows.append([
            round(plant.t, 4), plant.phase,
            round(plant.pos[0], 4), round(plant.pos[1], 4), round(plant.pos[2], 4),
            round(plant.vel[0], 4), round(plant.vel[1], 4), round(plant.vel[2], 4),
            round(true_tilt, 3), round(true_roll, 3),
            round(plant.mass, 4), round(plant.thrust_n, 2), round(plant.a_axial, 3),
            sample.seq,
            round(sample.qw, 6), round(sample.qx, 6), round(sample.qy, 6), round(sample.qz, 6),
            round(sample.acc_body[0], 4), round(sample.acc_body[1], 4), round(sample.acc_body[2], 4),
            round(sample.gyr_body[0], 5), round(sample.gyr_body[1], 5), round(sample.gyr_body[2], 5),
            round(sample.pressure_pa, 2),
            sample.gps_lat_1e7, sample.gps_lon_1e7, round(sample.gps_alt_msl, 3),
            sample.gps_ve_cms, sample.gps_vn_cms, sample.lidar_mm,
            sample.uwb_range_cm[0], sample.uwb_range_cm[1], sample.uwb_range_cm[2], sample.uwb_range_cm[3],
            sample.flags,
            h.t_ms, h.mst_state, h.nav_phase, h.mst_flags, h.nav_events,
            h.pyro_active, h.pyro_fired, h.cover_open,
            round(h.est_p[0], 3), round(h.est_p[1], 3), round(h.est_p[2], 3),
            round(h.est_v[0], 3), round(h.est_v[1], 3), round(h.est_v[2], 3),
            round(h.est_q[0], 5), round(h.est_q[1], 5), round(h.est_q[2], 5), round(h.est_q[3], 5),
            round(est_tilt, 3),
            round(h.h_ign, 2), round(h.thr_est, 2), round(h.k_plan, 3),
            round(h.t_to_ign, 3), round(h.t_to_td, 3),
            round(math.degrees(h.tilt_cmd), 2), round(math.degrees(h.tilt_err), 2),
            round(h.tvc_deg[0], 3), round(h.tvc_deg[1], 3), round(h.clamp, 3),
            round(h.fin_deg[0], 2), round(h.fin_deg[1], 2), round(h.fin_deg[2], 2), round(h.fin_deg[3], 2),
            h.act_flags,
            us[0], us[1], us[2], us[3], us[4], us[5], us[6], us[7],
        ])

    def save_csv(self, path: str) -> str:
        with open(path, "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(CSV_COLUMNS)
            w.writerows(self.rows)
        return path

    def compute_summary(self, plant: HilPlant) -> HilSummary:
        s = HilSummary(
            samples=len(self.rows),
            duration_s=plant.t,
            true_apogee_m=plant.apogee_m,
            h_ign_true_m=plant.h_ign_true,
            td_vz_mps=plant.td_vz,
            td_vh_mps=plant.td_vh,
            td_tilt_deg=plant.td_tilt_deg,
            landed=(plant.phase == "LANDED"),
        )
        if not self.rows:
            s.verdict = "BEZ DAT"
            return s

        idx = {name: i for i, name in enumerate(CSV_COLUMNS)}
        sq_alt = 0.0
        n_alt = 0
        min_cl, max_cl = 1.0, 0.0
        for r in self.rows:
            tpz = float(r[idx["true_pz"]])
            epz = float(r[idx["est_pz"]])
            tpx, tpy = float(r[idx["true_px"]]), float(r[idx["true_py"]])
            epx, epy = float(r[idx["est_px"]]), float(r[idx["est_py"]])
            s.est_apogee_m = max(s.est_apogee_m, epz)
            aerr = abs(epz - tpz)
            sq_alt += aerr * aerr
            n_alt += 1
            if aerr > s.alt_max_err_m:
                s.alt_max_err_m = aerr
            herr = math.hypot(epx - tpx, epy - tpy)
            if herr > s.horiz_max_err_m:
                s.horiz_max_err_m = herr
            s.max_tvc_deg = max(s.max_tvc_deg, abs(float(r[idx["cmd_tvc_x_deg"]])),
                                abs(float(r[idx["cmd_tvc_y_deg"]])))
            s.max_fin_deg = max(s.max_fin_deg,
                                abs(float(r[idx["cmd_fin0_deg"]])),
                                abs(float(r[idx["cmd_fin1_deg"]])),
                                abs(float(r[idx["cmd_fin2_deg"]])),
                                abs(float(r[idx["cmd_fin3_deg"]])))
            cl = float(r[idx["cmd_clamp"]])
            min_cl = min(min_cl, cl)
            max_cl = max(max_cl, cl)
            ev = int(r[idx["nav_events"]])
            s.events_seen |= ev
            s.pyro_fired_mask |= int(r[idx["pyro_fired"]])
            if int(r[idx["cover_open"]]):
                s.cover_opened = True
            if ev & EV_LEGS:
                s.legs_fired = True
            hi = float(r[idx["h_ign_m"]])
            if hi > 0.0:
                s.h_ign_hw_m = hi

        s.alt_rmse_m = math.sqrt(sq_alt / max(1, n_alt))
        s.min_clamp, s.max_clamp = min_cl, max_cl
        if not s.landed:
            s.verdict = "NEDOKONČENO / PŘERUŠENO"
        elif abs(s.td_vz_mps) <= 4.0 and s.td_tilt_deg <= 8.0:
            s.verdict = "OK — měkké dosednutí"
        elif abs(s.td_vz_mps) <= 8.0:
            s.verdict = "TVRDŠÍ DOSEDNUTÍ"
        else:
            s.verdict = "NEÚSPĚŠNÉ PŘISTÁNÍ"
        return s
