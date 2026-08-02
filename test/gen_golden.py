#!/usr/bin/env python3
"""Generate test/golden_vectors.h from the validated Python model.

The flight code in ../Daidalos_Landing_Guidance is a hand port: it uses a
different integrator (RK2 at 0.04 s instead of Euler at 0.02 s), a resampled
thrust table and a series approximation for asin. None of that is visible in a
compile, so the port is pinned to the model by comparing numbers rather than by
reading the two side by side.

Usage:
    python3 gen_golden.py --model /path/to/lander_v2.py > golden_vectors.h

The model is imported, not re-implemented, so these vectors cannot silently
drift away from the simulation they came from.
"""
import argparse
import importlib.util
import math
import os
import sys


def load_model(path):
    spec = importlib.util.spec_from_file_location("lander_v2", path)
    if spec is None or spec.loader is None:
        sys.exit(f"cannot import a model from {path}")
    mod = importlib.util.module_from_spec(spec)
    sys.modules["lander_v2"] = mod
    spec.loader.exec_module(mod)
    return mod


# The flight code runs one fixed configuration: drag compensation on, no
# position control, no terminal-descent mode, no wind trim. Every call below
# is made with that configuration so the golden values describe the algorithm
# that is actually flying.
DRAGCOMP = True
POS_CTL = False
TERMINAL = False
AX_BIAS = 0.0

THRUST_TAU = [0.0, 0.005, 0.05, 0.15, 0.3, 0.45, 0.5, 0.6, 0.75, 0.9, 0.96, 0.99, 1.0]

TILT_H = [40.0, 25.0, 12.0, 5.0, 2.0, 0.5]
TILT_VX = [0.0, 5.0, 12.0]
TILT_VZ = [-50.0, -30.0, -10.0, -2.0]

# h, vx, vz, tau — sampled along a representative descent, from just after
# ignition down to the last second of the burn.
SOLVE_STATES = [
    (35.0, 8.0, -52.0, 0.00),
    (28.0, 6.0, -45.0, 0.15),
    (20.0, 4.0, -34.0, 0.35),
    (12.0, 3.0, -24.0, 0.55),
    (6.0, 1.5, -14.0, 0.72),
    (2.5, 0.6, -7.0, 0.85),
    (30.0, 0.0, -40.0, 0.10),
    (24.0, 12.0, -38.0, 0.25),
]

IGN_VZ = [20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0, 55.0]


def main():
    ap = argparse.ArgumentParser()
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument("--model",
                    default=os.path.join(here, "..", "..",
                                         "Landing-Rocket-Sim", "lander_v2.py"),
                    help="path to lander_v2.py")
    args = ap.parse_args()
    m = load_model(args.model)

    out = []
    w = out.append
    w("// Golden vectors generated from the validated Python model (lander_v2.py).")
    w("// Regenerate with: python3 test/gen_golden.py > test/golden_vectors.h")
    w("#ifndef GOLDEN_VECTORS_H")
    w("#define GOLDEN_VECTORS_H")
    w("")

    # ---- 1. motor curve -------------------------------------------------
    w(f"#define N_THRUST {len(THRUST_TAU)}")
    w("static const float G_THRUST_TAU[] = {%s};"
      % ", ".join(f"{t}f" for t in THRUST_TAU))
    w("static const float G_THRUST_N[]   = {%s};"
      % ", ".join(f"{m.thrust_ref(t):.4f}f" for t in THRUST_TAU))
    w("")

    # ---- 2. guidance tilt ----------------------------------------------
    # The guidance law is planar even in 3-D: the commanded direction is the
    # planar solution rotated into the azimuth of the horizontal velocity. So
    # these vectors are taken with the crossrange channel zeroed, and the flight
    # code's own test checks the rotation separately.
    rows = []
    for h in TILT_H:
        for vx in TILT_VX:
            for vz in TILT_VZ:
                ux, _uy, uz, _t = m.guidance_dir(h, vx, 0.0, vz, 3.0, m.CD_AXIAL,
                                                 DRAGCOMP, POS_CTL, 0.0, 0.0,
                                                 AX_BIAS, 0.0)
                rows.append((h, vx, vz, math.atan2(ux, uz)))
    w(f"#define N_TILT {len(rows)}")
    w("static const float G_TILT[][4] = {")
    for h, vx, vz, th in rows:
        w(f"  {{{h}f, {vx}f, {vz}f, {th:.7f}f}},")
    w("};")
    w("")

    # ---- 3. throttle-plan root find -------------------------------------
    # act_k 0.7 and mass 3.0 match the host test's call. The plan always starts
    # at the CURRENT burn fraction, so xi_start = tau. Warm start off, so the
    # vector is a property of the state alone and not of the call history.
    w(f"#define N_SOLVE {len(SOLVE_STATES)}")
    w("static const float G_SOLVE[][5] = {")
    for h, vx, vz, tau in SOLVE_STATES:
        p, status = m.solve_p_illinois(h, vx, vz, tau, 3.0, 0.7, tau,
                                       1.0, 1.0, m.CD_AXIAL, 0.0,
                                       0.0, False, DRAGCOMP, POS_CTL, 0.0,
                                       TERMINAL, AX_BIAS)
        w(f"  {{{h}f, {vx}f, {vz}f, {tau}f, {p:.6f}f}},   // status {status}")
    w("};")
    w("")

    # ---- 4. ignition altitude -------------------------------------------
    # The MINIMUM altitude from which a near-full-throttle burn can still stop
    # the vehicle — the third return value, not the commanded altitude. The
    # flight code stores this table and adds its own delay pad at run time,
    # because it fires on altitude rather than on a clock.
    w(f"#define N_IGN {len(IGN_VZ)}")
    w("static const float G_IGN[][2] = {")
    for v in IGN_VZ:
        _, _, h_min = m.find_ignition(9.0, -v, m.CD_AXIAL, m.IGN_DELAY_HI,
                                      True, DRAGCOMP, TERMINAL)
        w(f"  {{{v}f, {h_min:.4f}f}},")
    w("};")
    w("")
    w("#endif")

    print("\n".join(out))


if __name__ == "__main__":
    main()
