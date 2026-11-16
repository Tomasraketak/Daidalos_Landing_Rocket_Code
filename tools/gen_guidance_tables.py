#!/usr/bin/env python3
"""Generate the flight tables and the test vectors from LandSim_Light.

    python3 tools/gen_guidance_tables.py --sim ../LandSim_Light

writes

    Ariadna_V2/common/guidance_tables.h    motor curve, vehicle numbers, fitted gains,
                                           and the DEFAULT ignition band table
    Ariadna_V2/common/ariadna_params_tbl.h the same band table as parameters (tbl.*),
                                           so it can be replaced from the diagnostic GUI
    test/guid_golden.h                     reference values the host test compares
                                           guidance.cpp against

THE IGNITION BAND TABLE. For every fall speed |vz| at the moment of ignition
(4, 6, ... 66 m/s) the simulation finds the band of altitudes [h_min, h_max]
from which its own receding-horizon clamp controller still lands softer than
FEASIBLE_VZ (-2.3 m/s). The vehicle does the rest in flight: ten times a second
during the descent it propagates its own fall from the CURRENT state with the
drag it MEASURES, finds where that trajectory enters the band, and adds the
igniter-delay pad. That is the simulation's find_ignition(), split into the part
that needs a PC (the band) and the part that needs the real flight (the fall).

THE BRIDGE TO THE DIAGNOSTIC GUI.

    python3 tools/gen_guidance_tables.py --sim ../LandSim_Light \
        --vehicle vehicle.json --table-json table.json

reads the vehicle the GUI exported (g.mass_gross_kg, g.cd_body, ...), flies the
simulation with it and writes only the band table, as JSON, for the GUI to
upload. No header is touched in that mode.

The script IMPORTS tvc_sim.py rather than re-implementing anything, so the
firmware tables cannot drift from the simulation they came from. Re-run it
after any change to the vehicle in the simulation (mass, motor, fins, gates,
igniter pad) and then `cd test && make check`.

The landing vehicle has no D9 booster channel (PYRO1 igniter, PYRO2 backup
igniter, PYRO3 leg release), so everything is generated with n_boosters = 0.
"""
import argparse
import json
import math
import os
import sys
import time


def f(x):
    """A C float literal."""
    s = repr(float(x))
    if "e" in s or "." in s:
        return s + "f"
    return s + ".0f"


def arr(name, values, per_line=6):
    out = [f"static const float {name}[{len(values)}] = {{"]
    for i in range(0, len(values), per_line):
        out.append("    " + ", ".join(f(v) for v in values[i:i + per_line]) + ",")
    out.append("};")
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim", default="../LandSim_Light",
                    help="folder with tvc_sim.py and tvc_gains.json")
    ap.add_argument("--root", default=os.path.join(os.path.dirname(__file__), ".."))
    ap.add_argument("--delay-pad", type=float, default=None,
                    help="igniter delay the ignition altitude pads for [s] "
                         "(default: the simulation's)")
    ap.add_argument("--vz0", type=float, default=4.0, help="first |vz| of the band table")
    ap.add_argument("--dvz", type=float, default=2.0, help="|vz| step of the band table")
    ap.add_argument("--vehicle", help="JSON from the diagnostic GUI: parameter key -> value")
    ap.add_argument("--table-json", help="write only the band table to this JSON file")
    ap.add_argument("--landsim-vehicle",
                    help="vehicle.json in LandSim's own format (landsim_export.py): regenerate "
                         "EVERYTHING - motor table, fin geometry, inertia, band - for it")
    args = ap.parse_args()

    sim_dir = os.path.abspath(args.sim)
    sys.path.insert(0, sim_dir)
    os.chdir(sim_dir)                      # tvc_gains.json is read from the cwd
    import numpy as np
    import tvc_sim as s

    cfg = s.TvcConfig(n_boosters=0, use_booster_rule=False)
    gains = s.load_gains()
    if gains:
        cfg = cfg.with_gains(gains)
    from dataclasses import replace
    if args.landsim_vehicle:
        import landsim_export as ex
        with open(args.landsim_vehicle, encoding="utf-8") as fh:
            lveh = json.load(fh)
        # Daidalos flies no D9; LandSim's default rocket carries one
        lveh.setdefault("n_boosters", 0)
        lveh.setdefault("use_booster_rule", False)
        cfg = ex.config_from_vehicle(lveh)
        if gains and not any(k in lveh for k in ("wn", "zeta", "wn_fin", "zeta_fin", "roll_gain")):
            cfg = cfg.with_gains(gains)
        print("LandSim vehicle:", ex.resolved_vehicle(cfg), file=sys.stderr)
    if args.delay_pad is not None:
        cfg = replace(cfg, delay_pad=args.delay_pad)
    if args.vehicle:
        with open(args.vehicle, encoding="utf-8") as fh:
            veh = json.load(fh)
        m_ = {"g.mass_gross_kg": "gross_mass", "g.mass_prop_kg": "propellant",
              "g.cd_body": "cd", "g.thrust_scale": "thrust_mult", "g.diameter_m": "diameter",
              "g.plan_target_mps": "plan_target", "g.ign_delay_pad_s": "delay_pad"}
        kw = {m_[k]: float(v) for k, v in veh.items() if k in m_}
        if "fin.brake" in veh:
            kw["fin_brake"] = "auto" if float(veh["fin.brake"]) > 0.5 else "off"
        if "fin.enable" in veh and float(veh["fin.enable"]) < 0.5:
            kw["fins"] = False
        # inertia scales with the mass the GUI gives (the geometry does not change)
        if "gross_mass" in kw:
            r = kw["gross_mass"] / cfg.gross_mass
            kw["mmoi_transverse"] = cfg.mmoi_transverse * r
            kw["mmoi_roll"] = cfg.mmoi_roll * r
        cfg = replace(cfg, **kw)
        print("vehicle:", kw, file=sys.stderr)

    m, b = cfg.tables()
    mt, mf, mc = s.motor_arrays(m)
    bt, bf, bb = s.booster_arrays(b)
    fins = cfg.fin_set()
    _, cd_free = cfg.fin_args()

    # ---------------- ignition band table ----------------
    dry = cfg.gross_mass - m.propellant_mass
    feasible_vz = s.FEASIBLE_VZ

    def touchdown(h, vz):
        return s.project_rh(float(h), -float(vz), 0.0, dry, mt, mf, mc, m.propellant_mass,
                            m.total_impulse, cfg.cd_burn, cfg.area, cfg.k_min, cfg.k_max,
                            bt, bf, bb, b.propellant_mass, 1.0e6, cfg.plan_target)

    def band(vz):
        """[h_min, h_max] for this fall speed; (h_best, 0) when no altitude works."""
        step = 2.0
        hs = [3.0 + step * i for i in range(int(157 / step))]
        lo = hi = None
        best_h, best_v = hs[0], -1e9
        for h in hs:
            v = touchdown(h, vz)
            if v > best_v:
                best_v, best_h = v, h
            if v >= feasible_vz:
                if lo is None:
                    lo = h
                hi = h
            elif lo is not None:
                break                       # the band is one interval
        if lo is None:
            a_, c_ = max(1.0, best_h - step), best_h + step
            for _ in range(14):
                m1, m2 = a_ + (c_ - a_) / 3, c_ - (c_ - a_) / 3
                if touchdown(m1, vz) < touchdown(m2, vz):
                    a_ = m1
                else:
                    c_ = m2
            return 0.5 * (a_ + c_), 0.0
        a_, c_ = max(0.5, lo - step), lo           # refine the lower edge
        for _ in range(10):
            mid = 0.5 * (a_ + c_)
            if touchdown(mid, vz) >= feasible_vz:
                c_ = mid
            else:
                a_ = mid
        h_min = c_
        a_, c_ = hi, hi + step                      # and the upper one
        for _ in range(10):
            mid = 0.5 * (a_ + c_)
            if touchdown(mid, vz) >= feasible_vz:
                a_ = mid
            else:
                c_ = mid
        return h_min, a_

    n_vz = 32
    vzs = [args.vz0 + args.dvz * i for i in range(n_vz)]
    t0 = time.time()
    h_min_t, h_max_t = [], []
    for vz in vzs:
        lo_, hi_ = band(vz)
        h_min_t.append(lo_)
        h_max_t.append(hi_)
        print(f"  |vz| {vz:5.1f} m/s: band {lo_:6.2f} .. {hi_:6.2f} m"
              + ("   (NO BAND: best effort)" if hi_ <= 0 else ""), file=sys.stderr)
    print(f"band table: {time.time() - t0:.0f} s", file=sys.stderr)

    if args.table_json:
        with open(args.table_json, "w", encoding="utf-8") as fh:
            json.dump({"tbl.vz0": args.vz0, "tbl.dvz": args.dvz,
                       "h_min": h_min_t, "h_max": h_max_t,
                       "fin_cd_zero": (cfg.cd_burn - cfg.cd),
                       "fin_cd_brake": (cd_free - cfg.cd),
                       "vehicle": {"gross_mass": cfg.gross_mass, "propellant": m.propellant_mass,
                                   "cd": cfg.cd, "thrust_mult": cfg.thrust_mult,
                                   "diameter": cfg.diameter, "delay_pad": cfg.delay_pad}},
                      fh, indent=1)
        print("wrote", args.table_json, file=sys.stderr)
        return

    root = os.path.abspath(args.root)
    out = []
    out.append("// =====================================================================")
    out.append("//  GENERATED by tools/gen_guidance_tables.py from LandSim_Light/tvc_sim.py")
    out.append("//  DO NOT EDIT BY HAND - change the simulation and re-run the tool.")
    out.append("//")
    out.append(f"//  vehicle: {cfg.gross_mass} kg gross, {m.propellant_mass} kg propellant, "
               f"motor '{cfg.motor}', no D9 booster")
    out.append(f"//  igniter pad: {cfg.delay_pad} s, plan target {cfg.plan_target} m/s, "
               f"feasible {s.FEASIBLE_VZ} m/s")
    out.append("// =====================================================================")
    out.append("#ifndef DAIDALOS_GUIDANCE_TABLES_H")
    out.append("#define DAIDALOS_GUIDANCE_TABLES_H")
    out.append("")
    out.append("// ---- motor: tabulated, un-clamped thrust and cumulative impulse ----")
    out.append(f"#define DG_MOTOR_N {len(mt)}")
    out.append(arr("DG_MOTOR_T", mt))
    out.append(arr("DG_MOTOR_F", mf))
    out.append(arr("DG_MOTOR_CUM", mc))
    out.append(f"#define DG_I_TOTAL        {f(m.total_impulse)}   // N.s")
    out.append(f"#define DG_BURN_T         {f(mt[-1])}   // s")
    out.append(f"#define DG_PEAK_THRUST    {f(max(mf))}   // N")
    out.append(f"#define DG_M_PROP         {f(m.propellant_mass)}   // kg")
    out.append(f"#define DG_M_GROSS        {f(cfg.gross_mass)}   // kg, with propellant")
    out.append("")
    out.append("// ---- airframe ----")
    out.append(f"#define DG_AREA           {f(cfg.area)}   // m^2")
    out.append(f"#define DG_CD_BODY        {f(cfg.cd)}   // bare body (default of g.cd_body)")
    out.append(f"#define DG_FIN_CD_ZERO    {f(cfg.cd_burn - cfg.cd)}   // fins at 0 deg, as body dCd")
    out.append(f"#define DG_FIN_CD_BRAKE   {f(cd_free - cfg.cd)}   // fins splayed as airbrake, as body dCd")
    out.append(f"#define DG_DIAMETER       {f(cfg.diameter)}   // m (default of g.diameter_m)")
    out.append(f"#define DG_IT_PER_KG      {f(cfg.mmoi_transverse / cfg.gross_mass)}   // transverse MMOI / mass")
    out.append(f"#define DG_IA_PER_KG      {f(cfg.mmoi_roll / cfg.gross_mass)}   // roll MMOI / mass")
    out.append(f"#define DG_L_GIMBAL       {f(cfg.l_gimbal)}   // CG -> nozzle pivot, m")
    out.append(f"#define DG_L_CP           {f(cfg.l_cp)}   // CG -> BODY normal force, + aft, m")
    out.append(f"#define DG_CN_ALPHA       {f(cfg.cn_alpha)}")
    out.append("")
    out.append("// ---- fins ----")
    out.append(f"#define DG_FIN_AREA       {f(fins.area)}   // m^2, one fin")
    out.append(f"#define DG_FIN_ARM        {f(fins.arm)}   // CG -> fin, aft, m")
    out.append(f"#define DG_FIN_ROLL_ARM   {f(fins.roll_arm)}   // m")
    out.append(f"#define DG_FIN_CL_ALPHA   {f(fins.cl_alpha)}   // 1/rad, CFD secant to 7 deg")
    out.append(f"#define DG_FIN_K_INT      {f(cfg.fin_k())}   // fin-body interference")
    out.append(f"#define DG_FIN_MAX_DEG    {f(fins.max_deflect)}")
    out.append(f"#define DG_FIN_STALL_DEG  {f(cfg.fin_stall)}")
    out.append("")
    out.append("// ---- fitted gains (tvc_gains.json) ----")
    out.append(f"#define DG_WN_TVC         {f(cfg.wn)}")
    out.append(f"#define DG_ZETA_TVC       {f(cfg.zeta)}")
    out.append(f"#define DG_WN_FIN         {f(cfg.wn_fin)}")
    out.append(f"#define DG_ZETA_FIN       {f(cfg.zeta_fin)}")
    out.append(f"#define DG_ROLL_GAIN      {f(cfg.roll_gain)}")
    out.append(f"#define DG_SCHED_TVC      {f(cfg.sched_tvc)}")
    out.append(f"#define DG_SCHED_FIN      {f(cfg.sched_fin)}")
    out.append("")
    out.append("// ---- clamp planner: thrust forecast (scatter decays back to the table) ----")
    out.append(f"#define DG_THRUST_TAU     {f(getattr(cfg, 'plan_thrust_tau', 0.0))}   // s, 0 = deviation lasts to burnout")
    out.append("")
    out.append("// ---- default ignition band: [h_min, h_max] against |vz| at ignition ----")
    out.append("// h_max = 0 marks a fall speed with no feasible band; h_min is then the")
    out.append("// least-bad altitude. The flight copy is the tbl.* parameters.")
    out.append(f"#define DG_IGN_N   {n_vz}")
    out.append(f"#define DG_IGN_VZ0 {f(args.vz0)}")
    out.append(f"#define DG_IGN_DVZ {f(args.dvz)}")
    out.append(arr("DG_IGN_HMIN", h_min_t))
    out.append(arr("DG_IGN_HMAX", h_max_t))
    out.append(f"#define DG_IGN_DELAY_PAD  {f(cfg.delay_pad)}   // s, default igniter pad")
    out.append("")
    out.append("#endif // DAIDALOS_GUIDANCE_TABLES_H")
    path = os.path.join(root, "Ariadna_V2", "common", "guidance_tables.h")
    with open(path, "w") as fh:
        fh.write("\n".join(out) + "\n")
    print("wrote", path, file=sys.stderr)

    # ---------------- the same table as parameters ----------------
    pt = ["// GENERATED by tools/gen_guidance_tables.py - the ignition band table as",
          "// parameters (ids 96..163). Included by ariadna_params.h. DO NOT EDIT.",
          "#ifndef ARIADNA_PARAMS_TBL_H", "#define ARIADNA_PARAMS_TBL_H",
          "#define ARIADNA_PARAM_LIST_TBL(X) \\",
          f"    X( 96, TBL_VZ0, \"tbl.vz0\", {f(args.vz0)}, 0.0f, 30.0f, \"m/s\", 'N') \\",
          f"    X( 97, TBL_DVZ, \"tbl.dvz\", {f(args.dvz)}, 0.5f, 10.0f, \"m/s\", 'N') \\"]
    for i, v in enumerate(h_min_t):
        pt.append(f"    X({100 + i:3d}, TBL_HMIN_{i:02d}, \"tbl.hmin_{i:02d}\", {f(v)}, 0.0f, 300.0f, \"m\", 'N') \\")
    for i, v in enumerate(h_max_t):
        pt.append(f"    X({132 + i:3d}, TBL_HMAX_{i:02d}, \"tbl.hmax_{i:02d}\", {f(v)}, 0.0f, 300.0f, \"m\", 'N') \\")
    pt[-1] = pt[-1][:-2]
    pt += ["#endif", ""]
    path = os.path.join(root, "Ariadna_V2", "common", "ariadna_params_tbl.h")
    with open(path, "w") as fh:
        fh.write("\n".join(pt))
    print("wrote", path, file=sys.stderr)

    # ---------------- golden vectors ----------------
    g = []
    g.append("// GENERATED by tools/gen_guidance_tables.py - reference values from tvc_sim.py")
    g.append("#ifndef GUID_GOLDEN_H")
    g.append("#define GUID_GOLDEN_H")

    # motor
    ts = [0.0, 0.02, 0.05, 0.12, 0.3, 0.45, 0.9, 1.3, 1.6, 2.0, 2.4, 2.6, 2.7]
    g.append("struct GoldMotor { float t, thrust, burned; };")
    g.append(f"static const GoldMotor GOLD_MOTOR[{len(ts)}] = {{")
    for t in ts:
        g.append(f"    {{ {f(t)}, {f(s.main_thrust(t, mt, mf))}, "
                 f"{f(s.main_burned(t, mt, mc, m.propellant_mass, m.total_impulse))} }},")
    g.append("};")

    # guidance direction
    cases = []
    for h in (60.0, 30.0, 12.0, 4.0, 1.0, 0.3):
        for (vx, vy) in ((0.0, 0.0), (6.0, 0.0), (-3.0, 4.0), (0.4, -0.3)):
            for vz in (-40.0, -15.0, -3.0, -0.8):
                for mass in (2.85, 2.6):
                    ux, uy, uz, tgo = s.guidance_dir(h, vx, vy, vz, mass, cfg.cd_burn, 0.0, 0.0,
                                                     cfg.area, cfg.tilt_min, cfg.tilt_slope,
                                                     cfg.tilt_cap, 1.0, 0.0)
                    cases.append((h, vx, vy, vz, mass, ux, uy, uz, tgo))
    g.append("struct GoldDir { float h, vx, vy, vz, mass, ux, uy, uz, tgo; };")
    g.append(f"static const GoldDir GOLD_DIR[{len(cases)}] = {{")
    for c in cases:
        g.append("    { " + ", ".join(f(v) for v in c) + " },")
    g.append("};")

    # projection and plan
    dry = cfg.gross_mass - m.propellant_mass
    kmin, kmax = cfg.k_min, cfg.k_max
    TAU = getattr(cfg, "plan_thrust_tau", 0.0) or 0.7
    proj = []
    for (h, vz, tb) in ((55.0, -31.0, 0.0), (40.0, -30.0, 0.5), (25.0, -22.0, 1.0),
                        (12.0, -12.0, 1.6), (5.0, -6.0, 2.0), (2.0, -2.5, 2.2),
                        (60.0, -28.0, 0.0), (30.0, -30.0, 0.2)):
        for k in (0.1, 0.4, 0.7, 1.0):
            # (t_scale, forecast target, forecast tau): held, and decaying
            for ts_, ti, tau in ((1.0, -1.0, 0.0), (0.9, -1.0, 0.0),
                                 (0.9, 0.97, TAU), (1.12, 0.97, TAU)):
                v = s.project_vz(h, vz, tb, dry, k, mt, mf, mc, m.propellant_mass,
                                 m.total_impulse, cfg.cd_burn, cfg.area, kmin, kmax,
                                 bt, bf, bb, b.propellant_mass, 1.0e6, 0.02, ts_,
                                 *((ti, tau) if tau > 0.0 else ()))
                proj.append((h, vz, tb, k, ts_, ti, tau, v))
    g.append("struct GoldProj { float h, vz, tb, k, ts, ts_inf, ts_tau, vz_td; };")
    g.append(f"static const GoldProj GOLD_PROJ[{len(proj)}] = {{")
    for c in proj:
        g.append("    { " + ", ".join(f(v) for v in c) + " },")
    g.append("};")

    plan = []
    for (h, vz, tb) in ((55.0, -31.0, 0.0), (50.0, -28.0, 0.0), (40.0, -30.0, 0.5),
                        (25.0, -22.0, 1.0), (12.0, -12.0, 1.6), (5.0, -6.0, 2.0),
                        (70.0, -25.0, 0.0), (20.0, -30.0, 0.3), (8.0, -3.0, 1.2)):
        for ts_, ti, tau in ((1.0, -1.0, 0.0), (0.9, -1.0, 0.0), (1.1, -1.0, 0.0),
                             (0.9, 0.97, TAU), (1.1, 0.97, TAU)):
            k, r = s.solve_plan(h, vz, tb, dry, mt, mf, mc, m.propellant_mass,
                                m.total_impulse, cfg.cd_burn, cfg.area, kmin, kmax,
                                bt, bf, bb, b.propellant_mass, 1.0e6, 0.02,
                                cfg.plan_target, ts_,
                                *((ti, tau) if tau > 0.0 else ()))
            plan.append((h, vz, tb, ts_, ti, tau, k, r))
    g.append("struct GoldPlan { float h, vz, tb, ts, ts_inf, ts_tau, k, resid; };")
    g.append(f"static const GoldPlan GOLD_PLAN[{len(plan)}] = {{")
    for c in plan:
        g.append("    { " + ", ".join(f(v) for v in c) + " },")
    g.append("};")

    # terminal law
    term = []
    for h in (2.5, 1.0, 0.2):
        for vz in (-2.5, -1.0):
            for mass in (2.6,):
                for T in (110.0, 60.0):
                    act = 1.0 if s.terminal_active(h, vz, T, mass) else 0.0
                    k = s.terminal_throttle(h, vz, mass, T, kmin, kmax)
                    term.append((h, vz, mass, T, act, k))
    g.append("struct GoldTerm { float h, vz, mass, thrust, active, k; };")
    g.append(f"static const GoldTerm GOLD_TERM[{len(term)}] = {{")
    for c in term:
        g.append("    { " + ", ".join(f(v) for v in c) + " },")
    g.append("};")

    # ignition altitude from an apogee state, exactly as find_ignition() solves it;
    # guidance.cpp must reproduce it from the band table and its own fall
    spots = []
    for (h0, vx) in ((150.0, 0.0), (170.0, 5.0), (120.0, 2.5), (100.0, 0.0), (185.0, 1.0)):
        spots.append((h0, vx, s.plan_ignition(cfg, h0, vx)))
    g.append("struct GoldIgn { float h0, vx, h_cmd; };")
    g.append(f"static const GoldIgn GOLD_IGN[{len(spots)}] = {{")
    for c in spots:
        g.append("    { " + ", ".join(f(v) for v in c) + " },")
    g.append("};")
    g.append("#endif")
    path = os.path.join(root, "test", "guid_golden.h")
    with open(path, "w") as fh:
        fh.write("\n".join(g) + "\n")
    print("wrote", path, file=sys.stderr)


if __name__ == "__main__":
    main()
