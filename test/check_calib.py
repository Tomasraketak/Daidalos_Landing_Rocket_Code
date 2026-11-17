"""
Calibration fits of the diagnostic console (Ariadna_V2/Diagnostics/calib_fit.py)
against synthetic sensors with KNOWN errors. Run by `make calib` / `make check`;
skipped without numpy.
"""
import math
import os
import sys

try:
    import numpy as np
except ImportError:
    print("calib: skipped (no numpy)")
    sys.exit(0)

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "Ariadna_V2", "Diagnostics"))
from calib_fit import fit_accel, fit_mag, sphere_coverage   # noqa: E402

fails = 0


def check(cond, msg):
    global fails
    print(("  [PASS] " if cond else "  [FAIL] ") + msg)
    if not cond:
        fails += 1


rng = np.random.default_rng(7)
print("calibration fits")

# ---- accelerometer: raw = a / scale + off ---------------------------------
off = np.array([0.031, -0.018, 0.052])
scale = np.array([0.987, 1.012, 1.004])
for name, dirs in (
        ("6 faces", np.array([[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]], float)),
        ("6 random", None), ("12 random", None)):
    if dirs is None:
        n = int(name.split()[0])
        while True:
            dirs = rng.normal(size=(n, 3))
            dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
            # a person spreads them out: reject sets that leave an axis sign unvisited
            if all((dirs[:, i] > 0.3).any() and (dirs[:, i] < -0.3).any() for i in range(3)):
                break
    raw = dirs / scale + off + rng.normal(0, 0.0015, size=dirs.shape)   # 1.5 mg of averaged noise
    f = fit_accel(raw)
    e_off = np.abs(f["off"] - off).max()
    e_scl = np.abs(f["scale"] - scale).max()
    check(e_off < 0.006 and e_scl < 0.008,
          f"accel {name}: offset err {1000 * e_off:.1f} mg, scale err {1000 * e_scl:.1f} ppt, "
          f"resid {1000 * f['resid_rms']:.2f} mg")

try:
    fit_accel([[0, 0, 1]] * 6)
    check(False, "accel: degenerate set rejected")
except ValueError:
    check(True, "accel: degenerate set rejected")

# ---- magnetometer: raw = SI^-1 m + HI ---------------------------------------
B = 48.0
HI = np.array([12.5, -30.2, 7.7])
Q, _ = np.linalg.qr(rng.normal(size=(3, 3)))
SI_inv = Q @ np.diag([1.10, 0.93, 1.02]) @ Q.T           # soft iron distortion
u = rng.normal(size=(1500, 3))
u /= np.linalg.norm(u, axis=1, keepdims=True)
raw = (u * B) @ SI_inv.T + HI + rng.normal(0, 0.4, size=u.shape)
f = fit_mag(raw)
cal = (raw - f["hard"]) @ f["soft"].T
check(np.abs(f["hard"] - HI).max() < 0.8, f"mag hard iron err {np.abs(f['hard'] - HI).max():.2f} uT")
check(abs(f["field"] - B) / B < 0.03, f"mag field {f['field']:.1f} uT (true {B})")
check(f["resid_after"] < 0.02 and f["resid_before"] > 0.04,
      f"mag |B| spread {100 * f['resid_before']:.1f} % -> {100 * f['resid_after']:.1f} %")
# directions preserved up to the global rotation sqrtm leaves: compare angles between samples
a0 = math.degrees(math.acos(np.clip(cal[0] @ cal[1] / np.linalg.norm(cal[0]) / np.linalg.norm(cal[1]), -1, 1)))
t0 = math.degrees(math.acos(np.clip(u[0] @ u[1], -1, 1)))
check(abs(a0 - t0) < 3.0, f"mag angle between two samples {a0:.1f} deg (true {t0:.1f})")
check(sphere_coverage(raw) > 0.95, f"coverage of a full sphere {100 * sphere_coverage(raw):.0f} %")
half = raw[(raw - HI)[:, 2] > 0]   # one side never shown
check(sphere_coverage(half) < 0.75, f"coverage of a half sphere {100 * sphere_coverage(half):.0f} %")

still = HI + rng.normal(0, 0.5, size=(300, 3))   # a board lying on the table
check(sphere_coverage(still) < 0.1, f"coverage of a board that is not turned {100 * sphere_coverage(still):.0f} %")

print(f"{'all' if not fails else fails} {'checks passed' if not fails else 'FAILED'}")
sys.exit(1 if fails else 0)
