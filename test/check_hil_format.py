#!/usr/bin/env python3
"""
Pin Ariadna_V2/Diagnostics/hil_sim.py against the C protocol headers
(ariadna_hil.h, ariadna_link.h, ariadna_seq.h) and verify the end-to-end
6-DOF Hardware-in-the-Loop (HIL) simulator, binary framing, and CSV recorder.

Run:  cd test && make hilfmt
"""
from __future__ import annotations

import csv
import math
import os
import re
import struct
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
COMMON = os.path.join(HERE, "..", "Ariadna_V2", "common")
DIAG = os.path.join(HERE, "..", "Ariadna_V2", "Diagnostics")

HIL_H = os.path.join(COMMON, "ariadna_hil.h")
LINK_H = os.path.join(COMMON, "ariadna_link.h")
SEQ_H = os.path.join(COMMON, "ariadna_seq.h")

sys.path.insert(0, DIAG)
import hil_sim as hs  # noqa: E402

CTYPES = {
    "uint8_t": ("B", 1), "int8_t": ("b", 1),
    "uint16_t": ("H", 2), "int16_t": ("h", 2),
    "uint32_t": ("I", 4), "int32_t": ("i", 4),
    "float": ("f", 4),
}

failures: list[str] = []


def check(ok: bool, msg: str) -> None:
    if not ok:
        failures.append(msg)


def parse_struct(source: str, name: str):
    """Extract (field_name, expanded_struct_codes, byte_size) from `typedef struct { ... } name;`."""
    m = re.search(r"typedef struct \{([^{}]*)\}\s*" + name + r"\s*;", source, re.S)
    if not m:
        return None
    fields = []
    for line in m.group(1).splitlines():
        line = re.sub(r"//.*", "", line).strip()
        if not line or not line.endswith(";"):
            continue
        decl = line[:-1].strip()
        parts = decl.split(None, 1)
        if len(parts) != 2 or parts[0] not in CTYPES:
            continue
        code, size = CTYPES[parts[0]]
        for nm in parts[1].split(","):
            nm = nm.strip()
            arr = re.match(r"(\w+)\s*\[\s*(\d+)\s*\]", nm)
            if arr:
                n = int(arr.group(2))
                fields.append((arr.group(1), code * n, size * n))
            else:
                fields.append((nm.lstrip("*"), code, size))
    return fields


def expand_fmt(fmt: str) -> str:
    """Expand repeat counts like '<I11f4H' -> '<IfffffffffffHHHH' for canonical comparison."""
    assert fmt.startswith("<")
    out = ["<"]
    for num, ch in re.findall(r"(\d*)([A-Za-z])", fmt[1:]):
        out.append(ch * (int(num) if num else 1))
    return "".join(out)


def parse_c_const(source: str, name: str) -> int | None:
    """Parse a `#define NAME <int|hex|shift>` or enum entry `NAME = <int|hex>`."""
    m = re.search(r"#define\s+" + re.escape(name) + r"\s+\(\s*1u?\s*<<\s*(\d+)\s*\)", source)
    if m:
        return 1 << int(m.group(1))
    m = re.search(r"(?:#define\s+)?" + re.escape(name) + r"\s*=?\s*(0x[0-9A-Fa-f]+|\d+)u?\b", source)
    if m:
        return int(m.group(1), 0)
    return None


def check_c_parity() -> None:
    hil_src = open(HIL_H, encoding="utf-8").read()
    link_src = open(LINK_H, encoding="utf-8").read()
    seq_src = open(SEQ_H, encoding="utf-8").read()

    # ---- struct layout ----
    fields = parse_struct(hil_src, "ariadna_hil_in_t")
    check(fields is not None, "could not find ariadna_hil_in_t in ariadna_hil.h")
    if fields is None:
        return

    c_fmt_expanded = "<" + "".join(f[1] for f in fields)
    c_size = sum(f[2] for f in fields)
    check(c_size == 84, f"ariadna_hil_in_t C size is {c_size}, expected 84")
    check(struct.calcsize(c_fmt_expanded) == c_size,
          f"packed C format size {struct.calcsize(c_fmt_expanded)} != field sum {c_size}")

    py_fmt_expanded = expand_fmt(hs.HIL_FMT)
    check(py_fmt_expanded == c_fmt_expanded,
          f"HIL_FMT mismatch:\n      C header: {c_fmt_expanded}\n      hil_sim : {py_fmt_expanded} ({hs.HIL_FMT})")
    check(hs.HIL_PAYLOAD_SIZE == c_size,
          f"HIL_PAYLOAD_SIZE is {hs.HIL_PAYLOAD_SIZE}, expected {c_size}")

    # Verify unpack tuple length matches expanded scalar count
    n_scalars = len(c_fmt_expanded) - 1
    zero_unpacked = struct.unpack(hs.HIL_FMT, b"\x00" * c_size)
    check(len(zero_unpacked) == n_scalars,
          f"unpacked scalar count {len(zero_unpacked)} != C scalar count {n_scalars}")

    # ---- framing, flags, and event bit constants ----
    checks = [
        (link_src, "ARIADNA_SYNC0", hs.SYNC0),
        (link_src, "ARIADNA_SYNC1", hs.SYNC1),
        (link_src, "ARIADNA_PKT_HIL_IN", hs.HIL_PKT_TYPE),
        (link_src, "ARIADNA_MST_ARMED", hs.MST_ARMED),
        (link_src, "ARIADNA_MST_RBF_OUT", hs.MST_RBF_OUT),
        (link_src, "ARIADNA_MST_PYRO_INHIBIT", hs.MST_PYRO_INHIBIT),
        (link_src, "ARIADNA_ACT_VALID", hs.ACT_VALID),
        (link_src, "ARIADNA_ACT_IGNITE", hs.ACT_IGNITE),
        (link_src, "ARIADNA_ACT_FINS_LIVE", hs.ACT_FINS_LIVE),
        (link_src, "ARIADNA_ACT_TVC_PARK", hs.ACT_TVC_PARK),
        (link_src, "ARIADNA_ACT_COVER_OPEN", hs.ACT_COVER_OPEN),
        (hil_src, "ARIADNA_HIL_TIMEOUT_MS", hs.HIL_TIMEOUT_MS),
        (hil_src, "ARIADNA_HIL_BARO_FRESH", hs.HIL_FLAG_BARO_FRESH),
        (hil_src, "ARIADNA_HIL_GPS_FRESH", hs.HIL_FLAG_GPS_FRESH),
        (hil_src, "ARIADNA_HIL_LIDAR_VALID", hs.HIL_FLAG_LIDAR_VALID),
        (hil_src, "ARIADNA_HIL_UWB_FRESH", hs.HIL_FLAG_UWB_FRESH),
        (hil_src, "ARIADNA_HIL_SET_PAD_REF", hs.HIL_FLAG_SET_PAD_REF),
        (hil_src, "ARIADNA_HIL_RESET_EST", hs.HIL_FLAG_RESET_EST),
        (seq_src, "ARIADNA_EV_APOGEE", hs.EV_APOGEE),
        (seq_src, "ARIADNA_EV_COVER", hs.EV_COVER),
        (seq_src, "ARIADNA_EV_IGN1", hs.EV_IGN1),
        (seq_src, "ARIADNA_EV_THRUST", hs.EV_THRUST),
        (seq_src, "ARIADNA_EV_IGN2", hs.EV_IGN2),
        (seq_src, "ARIADNA_EV_LEGS", hs.EV_LEGS),
        (seq_src, "ARIADNA_EV_LIDAR", hs.EV_LIDAR),
        (seq_src, "ARIADNA_EV_TOUCHDOWN", hs.EV_TOUCHDOWN),
    ]
    for src, cname, pyval in checks:
        cval = parse_c_const(src, cname)
        check(cval is not None, f"could not parse {cname} from C header")
        if cval is not None:
            check(cval == pyval, f"{cname} mismatch: C header={cval} vs hil_sim={pyval}")

    fw_ver = parse_c_const(link_src, "ARIADNA_FW_VERSION")
    check(fw_ver is not None and fw_ver >= 332,
          f"ARIADNA_FW_VERSION must be >= 332 for HIL (got {fw_ver})")


def check_frame_codec() -> None:
    sample = hs.HilSensorSample(
        seq=123456,
        qw=0.7071068, qx=-0.7071068, qy=0.01, qz=-0.02,
        acc_body=(0.12, -9.80665, -0.34),
        gyr_body=(0.05, -0.15, 0.25),
        pressure_pa=96812.5,
        gps_lat_1e7=497985411,
        gps_lon_1e7=166912772,
        gps_alt_msl=455.25,
        gps_ve_cms=-235,
        gps_vn_cms=412,
        lidar_mm=3420,
        lidar_strength=11200,
        uwb_range_cm=(2830, 2910, 2780, 2855),
        temp_c_x10=224,
        flags=hs.HIL_FLAG_BARO_FRESH | hs.HIL_FLAG_GPS_FRESH | hs.HIL_FLAG_LIDAR_VALID | hs.HIL_FLAG_UWB_FRESH,
        gps_sats=13,
        gps_fix=3,
        gps_hdop_x10=8,
        uwb_mask=0x0F,
    )
    frame = hs.pack_hil_frame(sample)
    check(len(frame) == 90, f"packed HIL frame len is {len(frame)}, expected 90")
    check(frame[0] == 0xA5 and frame[1] == 0x1D, f"bad sync bytes: {frame[0]:02X} {frame[1]:02X}")
    check(frame[2] == 0x50 and frame[3] == 84, f"bad type/len: 0x{frame[2]:02X}/{frame[3]}")

    out = hs.unpack_hil_frame(frame)
    check(out.seq == sample.seq, f"seq mismatch: {out.seq} != {sample.seq}")
    check(abs(out.qw - sample.qw) < 1e-6 and abs(out.qx - sample.qx) < 1e-6, "quaternion roundtrip mismatch")
    check(abs(out.acc_body[1] - sample.acc_body[1]) < 1e-4, "acc_body roundtrip mismatch")
    check(abs(out.gyr_body[2] - sample.gyr_body[2]) < 1e-5, "gyr_body roundtrip mismatch")
    check(abs(out.pressure_pa - sample.pressure_pa) < 1e-2, "pressure_pa roundtrip mismatch")
    check(out.gps_lat_1e7 == sample.gps_lat_1e7 and out.gps_lon_1e7 == sample.gps_lon_1e7, "GPS lat/lon mismatch")
    check(out.gps_ve_cms == sample.gps_ve_cms and out.gps_vn_cms == sample.gps_vn_cms, "GPS vel mismatch")
    check(out.lidar_mm == sample.lidar_mm and out.lidar_strength == sample.lidar_strength, "lidar mismatch")
    check(out.uwb_range_cm == sample.uwb_range_cm, "UWB ranges mismatch")
    check(out.flags == sample.flags and out.uwb_mask == sample.uwb_mask, "flags/uwb_mask mismatch")

    # Corrupted payload byte must fail CRC
    bad = bytearray(frame)
    bad[10] ^= 0x01
    try:
        hs.unpack_hil_frame(bytes(bad))
        check(False, "corrupted HIL frame payload was accepted")
    except ValueError:
        pass

    # Corrupted CRC byte must fail
    bad_crc = bytearray(frame)
    bad_crc[-1] ^= 0x80
    try:
        hs.unpack_hil_frame(bytes(bad_crc))
        check(False, "corrupted HIL frame CRC was accepted")
    except ValueError:
        pass

    # Out-of-range integer clamping should not raise struct.error
    extreme = hs.HilSensorSample(gps_ve_cms=99999, gps_vn_cms=-99999, lidar_mm=99999, uwb_range_cm=(-5, 70000, 0, 1))
    eframe = hs.pack_hil_frame(extreme)
    eout = hs.unpack_hil_frame(eframe)
    check(eout.gps_ve_cms == 32767 and eout.gps_vn_cms == -32768, "int16 clamping failed")
    check(eout.lidar_mm == 65535 and eout.uwb_range_cm[:2] == (0, 65535), "uint16 clamping failed")


def check_sim_and_csv() -> None:
    # 1. Closed-loop flight with Master H-dict feedback and pyro INHIBITED (default safe mode)
    cfg = hs.HilConfig(
        mode="closed",
        target_apogee_m=150.0,
        wind_e_mps=2.0,
        wind_n_mps=-1.0,
        disturb_tilt_deg=4.0,
        disturb_az_deg=35.0,
        init_roll_dps=20.0,
        sensor_noise=False,
    )
    plant = hs.HilPlant(cfg)
    guid = hs.ReferenceGuidance(cfg)
    rec = hs.HilRecorder()

    pad_s = plant.make_pad_sample(reset_est=True, set_pad_ref=True)
    check((pad_s.flags & (hs.HIL_FLAG_RESET_EST | hs.HIL_FLAG_SET_PAD_REF)) ==
          (hs.HIL_FLAG_RESET_EST | hs.HIL_FLAG_SET_PAD_REF), "pad sample flags missing")
    check(abs(pad_s.acc_body[1] + hs.G_ACCEL) < 1e-3,
          f"pad upright specific force should be -g on body Y, got {pad_s.acc_body}")

    plant.launch()
    hw = hs.HwFeedback()
    dt = 0.01
    lidar_seen = False
    pyro_active_ever = 0
    for step_i in range(4000):
        samp = plant.step(dt, hw if step_i > 0 else None)
        frame = hs.pack_hil_frame(samp)
        check(len(frame) == 90, "frame size drift during flight")
        if samp.flags & hs.HIL_FLAG_LIDAR_VALID:
            lidar_seen = True

        out = guid.step(
            plant.t, dt, plant.pos, plant.vel, plant.b, plant.g, plant.w,
            plant.a_axial, pyro_permitted=False,
        )
        tvc = out["tvc_deg"]
        fin = out["fin_deg"]
        h_dict = {
            "t": int(plant.t * 1000),
            "st": out["mst_state"],
            "ph": out["nav_phase"],
            "fl": 0x12 | hs.MST_PYRO_INHIBIT,
            "ev": out["nav_events"],
            "pa": out["pyro_active"],
            "pf": out["pyro_fired"],
            "cov": out["cover_open"],
            "p": list(plant.pos),
            "v": list(plant.vel),
            "q": list(plant.q),
            "tvc": list(tvc),
            "cl": out["clamp"],
            "fin": list(fin),
            "afl": out["act_flags"],
            "us": [
                int(round(1500 + tvc[0] * 10)), int(round(1500 + tvc[1] * 10)),
                int(round(1100 + out["clamp"] * 800)), 1900 if out["cover_open"] else 1100,
                int(round(1500 + fin[0] * 10)), int(round(1500 + fin[1] * 10)),
                int(round(1500 + fin[2] * 10)), int(round(1500 + fin[3] * 10)),
            ],
            "hign": out["h_ign"],
            "thr": out["thr_est"],
            "k": out["k_plan"],
            "tign": out["t_to_ign"],
            "ttd": out["t_to_td"],
            "tc": out["tilt_cmd"],
            "te": out["tilt_err"],
        }
        hw = hs.HwFeedback.from_h_dict(h_dict)
        pyro_active_ever |= hw.pyro_active
        rec.record(plant, samp, hw)
        if plant.phase == "LANDED":
            break

    summ = rec.compute_summary(plant)
    check(summ.landed, f"closed-loop 150 m flight did not land (phase={plant.phase})")
    check(145.0 <= summ.true_apogee_m <= 155.0, f"apogee {summ.true_apogee_m:.2f} m out of [145, 155] m")
    check(abs(summ.td_vz_mps) <= 4.0, f"touchdown vz {summ.td_vz_mps:.2f} m/s too fast")
    check(summ.td_tilt_deg <= 8.0, f"touchdown tilt {summ.td_tilt_deg:.2f} deg too large")
    check(summ.cover_opened, "aerocover did not open during HIL flight")
    check(summ.legs_fired, "landing legs event not fired during HIL flight")
    check(summ.events_seen & hs.EV_IGN1, "ignition 1 event not seen")
    check(pyro_active_ever == 0, f"pyro_active was non-zero ({pyro_active_ever}) with pyro_permitted=False")
    check(summ.max_fin_deg > 1.0, f"fins never moved (max_fin_deg={summ.max_fin_deg:.2f})")
    check(summ.max_tvc_deg > 0.2, f"TVC never moved (max_tvc_deg={summ.max_tvc_deg:.2f})")
    check(lidar_seen, "synthetic nose lidar never became valid on final approach")

    # 2. Verify CSV export contains exact header, finite data, and actuator pulses
    with tempfile.TemporaryDirectory() as tmp:
        csv_path = os.path.join(tmp, "hil_test_run.csv")
        rec.save_csv(csv_path)
        with open(csv_path, encoding="utf-8") as fh:
            rows = list(csv.reader(fh))
        check(len(rows) == len(rec.rows) + 1, f"CSV row count {len(rows)} != {len(rec.rows) + 1}")
        check(rows[0] == hs.CSV_COLUMNS, "CSV header does not match hs.CSV_COLUMNS")
        for r_i, row in enumerate(rows[1:]):
            if len(row) != len(hs.CSV_COLUMNS):
                check(False, f"CSV row {r_i} has {len(row)} columns, expected {len(hs.CSV_COLUMNS)}")
                break
            for c_i, cell in enumerate(row):
                if c_i == 1:  # sim_phase string
                    continue
                v = float(cell)
                if not math.isfinite(v):
                    check(False, f"non-finite value in CSV row {r_i} col {hs.CSV_COLUMNS[c_i]}: {cell}")
                    break

    # 3. Verify live pyro mode (pyro_permitted=True) DOES energize pyro_active pulses
    g_live = hs.ReferenceGuidance(cfg)
    p_live = hs.HilPlant(hs.HilConfig(mode="open", target_apogee_m=120.0, sensor_noise=False))
    p_live.launch()
    live_pa = 0
    for _ in range(3500):
        p_live.step(dt)
        out = g_live.step(p_live.t, dt, p_live.pos, p_live.vel, p_live.b, p_live.g, p_live.w,
                          p_live.a_axial, pyro_permitted=True)
        live_pa |= out["pyro_active"]
        if p_live.phase == "LANDED":
            break
    check(bool(live_pa & 0x01), "live pyro mode did not energize PYRO1 (igniter)")
    check(bool(live_pa & 0x04), "live pyro mode did not energize PYRO3 (legs)")


def main() -> int:
    check_c_parity()
    check_frame_codec()
    check_sim_and_csv()
    if failures:
        print("HIL protocol / simulator verification FAILED:\n")
        for f in failures:
            print("  FAIL  " + f)
        print()
        return 1
    print("hil_sim.py matches ariadna_hil.h / ariadna_link.h and passes 6-DOF HIL + CSV tests")
    return 0


if __name__ == "__main__":
    sys.exit(main())
