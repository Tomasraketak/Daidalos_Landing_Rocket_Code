"""
Calibration fits for the diagnostic console (numpy only, no Qt), so they can
be tested on a workstation: test/check_calib.py.

  fit_accel()        axis-aligned ellipsoid, a = (raw - off) * scale, |a| = 1 g
  fit_mag()          general ellipsoid, m = SI * (raw - HI), |m| = const
  sphere_coverage()  how much of the sphere of directions the samples cover
"""
from __future__ import annotations

import math

import numpy as np


# ============================================================================
#  fitting - numpy only
# ============================================================================
def fit_accel(points):
    """
    Axis-aligned ellipsoid through averaged still readings (N x 3, g).
    Returns dict(off, scale, resid_rms, resid, cond) or raises ValueError.

    a x^2 + b y^2 + c z^2 + d x + e y + f z = 1, linear least squares;
    completing the squares gives off = -d/2a and scale = sqrt(a/G) with
    G = 1 + d^2/4a + e^2/4b + f^2/4c.
    """
    P = np.asarray(points, dtype=float)
    if P.ndim != 2 or P.shape[1] != 3 or len(P) < 6:
        raise ValueError("potřeba aspoň 6 poloh")
    D = np.column_stack([P[:, 0] ** 2, P[:, 1] ** 2, P[:, 2] ** 2, P[:, 0], P[:, 1], P[:, 2]])
    cond = float(np.linalg.cond(D))
    v, *_ = np.linalg.lstsq(D, np.ones(len(P)), rcond=None)
    a, b, c, d, e, f = v
    if min(a, b, c) <= 0:
        raise ValueError("polohy nepokrývají všechny osy (ellipsoid nevyšel) - přidejte další")
    G = 1.0 + d * d / (4 * a) + e * e / (4 * b) + f * f / (4 * c)
    if G <= 0:
        raise ValueError("fit nevyšel - přidejte další polohy")
    off = np.array([-d / (2 * a), -e / (2 * b), -f / (2 * c)])
    scale = np.sqrt(np.array([a, b, c]) / G)
    cal = (P - off) * scale
    resid = np.linalg.norm(cal, axis=1) - 1.0
    return {"off": off, "scale": scale, "resid": resid,
            "resid_rms": float(np.sqrt(np.mean(resid ** 2))), "cond": cond}


def fit_mag(points):
    """
    General ellipsoid through raw magnetometer samples (N x 3, uT).
    Returns dict(hard, soft, field, resid_before, resid_after) or raises.

    x'Ax + 2b'x = 1 by least squares (9 unknowns); centre c = -A^-1 b;
    M = A / (1 + c'Ac) maps the ellipsoid to the unit sphere, and
    soft = sqrtm(M) * B with B = det(M)^(-1/6) keeps the field strength
    (the soft-iron matrix then has determinant 1).
    """
    P = np.asarray(points, dtype=float)
    if P.ndim != 2 or P.shape[1] != 3 or len(P) < 30:
        raise ValueError("málo vzorků (potřeba aspoň 30, ideálně stovky)")
    x, y, z = P[:, 0], P[:, 1], P[:, 2]
    D = np.column_stack([x * x, y * y, z * z, 2 * x * y, 2 * x * z, 2 * y * z, 2 * x, 2 * y, 2 * z])
    v, *_ = np.linalg.lstsq(D, np.ones(len(P)), rcond=None)
    A = np.array([[v[0], v[3], v[4]], [v[3], v[1], v[5]], [v[4], v[5], v[2]]])
    bvec = v[6:9]
    try:
        c = -np.linalg.solve(A, bvec)
    except np.linalg.LinAlgError:
        raise ValueError("fit nevyšel - otáčejte deskou do více směrů")
    k = 1.0 + c @ A @ c
    M = A / k
    w, V = np.linalg.eigh(M)
    if np.any(w <= 0):
        raise ValueError("data nepokrývají kouli - otáčejte deskou do všech směrů")
    W = V @ np.diag(np.sqrt(w)) @ V.T                 # sqrtm(M), symmetric
    B = float(np.prod(w) ** (-1.0 / 6.0))
    soft = W * B
    cal = (P - c) @ soft.T
    r_after = np.linalg.norm(cal, axis=1)
    r_before = np.linalg.norm(P - P.mean(axis=0), axis=1)
    return {"hard": c, "soft": soft, "field": float(np.mean(r_after)),
            "resid_before": float(np.std(r_before) / max(np.mean(r_before), 1e-9)),
            "resid_after": float(np.std(r_after) / max(np.mean(r_after), 1e-9))}


def sphere_coverage(points, center=None, n_dirs: int = 26, max_angle_deg: float = 30.0,
                    min_radius: float = 10.0) -> float:
    """
    Fraction of n_dirs evenly spread directions that some sample points within
    max_angle_deg of, seen from `center`: did the board really face every way?
    Without a centre one is taken from a quick ellipsoid fit (the mean would
    be pulled toward wherever the samples bunch up and hide a missing side).
    """
    P = np.asarray(points, dtype=float)
    if len(P) < 3:
        return 0.0
    if center is None:
        try:
            center = fit_mag(P)["hard"] if len(P) >= 30 else None
        except ValueError:
            center = None
        if center is None:
            center = 0.5 * (P.max(axis=0) + P.min(axis=0))
    U = P - np.asarray(center, dtype=float)
    r = np.linalg.norm(U, axis=1)
    # Samples hugging the centre are noise around a board that is not being
    # turned, not directions: the Earth's field is 25-65 uT, so a real
    # sample sits far from the centre of the sphere it lies on.
    keep = r >= min_radius
    if keep.sum() < 3:
        return 0.0
    U = U[keep] / r[keep, None]
    i = np.arange(n_dirs) + 0.5
    phi = np.arccos(1 - 2 * i / n_dirs)
    th = math.pi * (1 + 5 ** 0.5) * i
    dirs = np.column_stack([np.cos(th) * np.sin(phi), np.sin(th) * np.sin(phi), np.cos(phi)])
    cosmax = math.cos(math.radians(max_angle_deg))
    hit = (U @ dirs.T).max(axis=0) >= cosmax
    return float(hit.mean())
