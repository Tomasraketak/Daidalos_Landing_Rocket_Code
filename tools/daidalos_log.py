#!/usr/bin/env python3
"""
Download the Master's flight log over USB and write it as CSV.

    python tools/daidalos_log.py --port COM5 --out flight.csv      # whole log
    python tools/daidalos_log.py --port /dev/ttyACM0 --info         # how much is there
    python tools/daidalos_log.py --port COM5 --flights-only         # only the flights
    python tools/daidalos_log.py --decode dump.txt --out flight.csv # from a saved dump
    python tools/daidalos_log.py --port COM5 --erase                # after downloading!

Close the diagnostic console first: only one program can hold the port. The
vehicle must not be logging (IDLE, TEST or RECOVERY). Records are 96 bytes,
50 per second from ARMED to LANDED; several flights follow each other in the
log and are told apart here by the clock going backwards (a reboot) or a gap
of more than 5 s. Each gets a "flight" number in the CSV.

The calibration columns (CALIBRATION_MANUAL.md, chapter 9):
    drag      measured / modelled free-fall drag  -> correct g.cd_body
    tscale    measured / table thrust             -> correct g.thrust_scale
    fin_gain  estimated fin effectiveness         -> fin.gain0 for the next flight

Vibration spectrum (fw 3.35): the log also holds a 0.5-500 Hz spectrum of the
vehicle every 0.512 s (USER_MANUAL.md, chapter 17). Those records go to a
second CSV (--vib-out, default <out>_vib.csv: one row per frame and channel,
1/3-octave band RMS and the three strongest tones), and the summary lists the
dominant frequencies of every flight phase. --vib-plot draws a spectrogram
(needs matplotlib).
"""
from __future__ import annotations

import argparse
import csv
import struct
import sys
import time

# Master_MCU.ino, struct LogRecord (packed, little endian)
FMT = "<I2f4f3f3f2i3f2f2HH4BBbBBBB"
FIELDS = ["t_ms", "px", "py", "h", "vx", "vy", "vz", "ux", "uy", "uz",
          "roll", "pitch", "yaw", "lat", "lon", "tvc_x", "tvc_y", "clamp",
          "thrust_est", "k_plan", "vbat_mv", "current_ma", "nav_flags",
          "state", "nav_phase", "pyro", "health", "events", "t_event_ds",
          "fin_gain_pct", "drag_pct", "tscale_pct", "flags"]
assert struct.calcsize(FMT) == 96, struct.calcsize(FMT)
# Vibration record (Master_MCU.ino struct VibLogRecord = ariadna_vib_t in a
# log slot): bit 7 of the last byte tells it from a LogRecord.
VIB_FMT = "<I4BH6H62B6B2B3xB"
assert struct.calcsize(VIB_FMT) == 96, struct.calcsize(VIB_FMT)
VIB_FLAG = 0x80
VIB_BANDS_HZ = [0.5, 0.63, 0.8, 1.0, 1.25, 1.6, 2.0, 2.5, 3.15, 4.0, 5.0, 6.3, 8.0, 10.0,
                12.5, 16.0, 20.0, 25.0, 31.5, 40.0, 50.0, 63.0, 80.0, 100.0, 125.0, 160.0,
                200.0, 250.0, 315.0, 400.0, 500.0]
VIB_CH = ["acc_axial_g", "acc_lateral_g", "gyro_lateral_dps", "gyro_roll_dps"]
VIB_ST = {"fresh0": 1, "fresh1": 2, "fresh2": 4, "overrun": 8, "lowrate": 16, "flight": 32}
STATES = ["BOOT", "IDLE", "ARMED", "ASCENT", "APOGEE", "DESCENT", "GUIDANCE",
          "LANDED", "RECOVERY", "ABORT", "TEST", "ERROR"]


def vib_level(code: int, ch: int) -> float:
    """RMS of an 8-bit dB code (ariadna_link.h): g for channels 0-1, deg/s for 2-3."""
    if code == 0:
        return 0.0
    off = 100.0 if ch <= 1 else 70.0
    return 10.0 ** ((code * 0.5 - off) / 20.0)


def decode_vib(payload: bytes, t_ms: int, flags: int = 0):
    """An 88-byte ariadna_vib_t (+ its log time) -> two per-channel dicts."""
    v = struct.unpack("<4BH6H62B6B2B", payload)
    part, seq, stat, n_avg, fs_dhz = v[0:5]
    peak_f, band, peak_amp, rms = v[5:11], v[11:73], v[73:79], v[79:81]
    rows = []
    for j in range(2):
        ch = 2 * part + j
        if ch > 3:
            break
        r = {"kind": "vib", "t_ms": t_ms, "seq": seq, "channel": VIB_CH[ch], "ch": ch,
             "stat": stat, "n_avg": n_avg, "fs_hz": fs_dhz / 10.0,
             "flight_rate": flags & 1, "rms": vib_level(rms[j], ch)}
        for i in range(3):
            r[f"peak{i + 1}_hz"] = peak_f[3 * j + i] / 100.0
            r[f"peak{i + 1}_rms"] = vib_level(peak_amp[3 * j + i], ch)
        for b, fc in enumerate(VIB_BANDS_HZ):
            r[f"b{fc:g}"] = vib_level(band[31 * j + b], ch)
        rows.append(r)
    return rows


def decode_line(line: str):
    """'LOGR <n> <hex>' -> (n, dict) for a flight record, (n, [dict, dict]) for a
    vibration record, or None."""
    parts = line.split()
    if len(parts) != 3 or parts[0] != "LOGR":
        return None
    raw = bytes.fromhex(parts[2])
    if len(raw) != 96 or raw == b"\xff" * 96:
        return None
    if raw[95] & VIB_FLAG:
        t_ms = struct.unpack_from("<I", raw)[0]
        return int(parts[1]), decode_vib(raw[4:92], t_ms, raw[95])
    d = dict(zip(FIELDS, struct.unpack(FMT, raw)))
    d["flight_rate"] = d.pop("flags") & 1
    d["lat"] /= 1e7
    d["lon"] /= 1e7
    d["fin_gain"] = d.pop("fin_gain_pct") / 100.0
    d["drag"] = d.pop("drag_pct") / 100.0
    d["tscale"] = d.pop("tscale_pct") / 100.0
    d["vbat"] = d.pop("vbat_mv") / 1000.0
    d["current"] = d.pop("current_ma") / 1000.0
    d["t_event"] = d.pop("t_event_ds") / 10.0
    d["state_name"] = STATES[d["state"]] if d["state"] < len(STATES) else str(d["state"])
    return int(parts[1]), d


def write_csv(rows, path):
    flight, last_t = 0, None
    for r in rows:
        if last_t is not None and (r["t_ms"] < last_t or r["t_ms"] - last_t > 5000):
            flight += 1
        r["flight"] = flight
        last_t = r["t_ms"]
    cols = ["flight"] + [k for k in rows[0] if k != "flight"] if rows else []
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=cols)
        w.writeheader()
        w.writerows(rows)
    return flight + 1 if rows else 0


def split_vib(entries):
    """Log-order (n, dict | list) entries -> (rows, vib_rows). Vibration rows
    take the flight number and mission state of the flight record before them."""
    rows, vib = [], []
    flight, last_t, state, state_t0 = 0, None, "?", 0
    fresh2_t = {}                                  # channel -> last 0.5-4 Hz refresh
    for _, d in entries:
        if isinstance(d, list):
            for r in d:
                r["flight"] = flight
                r["state_name"] = state
                r["state_t0"] = state_t0
                if r["stat"] & VIB_ST["fresh2"] or (flight, r["ch"]) not in fresh2_t:
                    fresh2_t[(flight, r["ch"])] = r["t_ms"]
                # where each stage's analysis window starts [ms]
                r["w_lo"] = fresh2_t[(flight, r["ch"])] - 4100
                r["w_mid"] = r["t_ms"] - 1020
                r["w_hi"] = r["t_ms"] - 640
                vib.append(r)
            continue
        if last_t is not None and (d["t_ms"] < last_t or d["t_ms"] - last_t > 5000):
            flight += 1
            state = None
        last_t = d["t_ms"]
        if d["state_name"] != state:
            state, state_t0 = d["state_name"], d["t_ms"]
        rows.append(d)
    return rows, vib


def write_vib_csv(vib, path):
    if not vib:
        return
    cols = ["flight", "t_ms", "state_name", "seq", "channel", "rms",
            "peak1_hz", "peak1_rms", "peak2_hz", "peak2_rms", "peak3_hz", "peak3_rms",
            "fs_hz", "n_avg", "stat", "flight_rate"] + [f"b{fc:g}" for fc in VIB_BANDS_HZ]
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=cols, extrasaction="ignore")
        w.writeheader()
        w.writerows(vib)


# Mission states grouped into the phases a vibration question is asked about.
VIB_PHASES = [("pad (ARMED)", {"ARMED"}), ("ascent", {"ASCENT"}),
              ("apogee + free fall", {"APOGEE", "DESCENT"}), ("landing burn", {"GUIDANCE"}),
              ("landed", {"LANDED"})]


# A frame says something about a phase only where its analysis window lies
# inside that phase: 63-500 Hz averages the last ~0.64 s, 5-50 Hz is one
# 1.02 s window, 0.5-4 Hz one 4.1 s window refreshed every ~2 s. A little
# overlap with the previous phase is tolerated (10 %).
def vib_sees(r, f_hz: float, t_start) -> bool:
    if t_start is None:
        return True
    if f_hz < 4.49:
        return r["w_lo"] >= t_start - 400
    if f_hz < 56.2:
        return r["w_mid"] >= t_start - 100
    return r["w_hi"] >= t_start - 60


def vib_dominant(rs, ch, t_start=None):
    """The strongest tones of one channel over a set of frames, ranked by their
    mean level (power average over the frames that can see them, a tone absent
    from a frame counting as zero), with the largest single reading alongside.
    Peaks closer than 5 % are one tone. With t_start (ms, the phase start), a
    frame counts for a frequency only if its analysis window is in the phase.
    Returns [f_hz, mean_rms, max_rms, frames] strongest first."""
    sel = [r for r in rs if r["ch"] == ch]
    tones = []                                   # [f, max, sum_pow, count]
    for r in sel:
        for i in (1, 2, 3):
            f, a = r[f"peak{i}_hz"], r[f"peak{i}_rms"]
            if f <= 0 or a <= 0 or not vib_sees(r, f, t_start):
                continue
            for t in tones:
                if abs(t[0] - f) < 0.05 * f:
                    if a > t[1]:
                        t[0], t[1] = f, a
                    t[2] += a * a
                    t[3] += 1
                    break
            else:
                tones.append([f, a, a * a, 1])
    out = []
    for f, amax, sp, cnt in tones:
        n = sum(1 for r in sel if vib_sees(r, f, t_start))
        out.append([f, (sp / max(n, 1)) ** 0.5, amax, cnt])
    return sorted(out, key=lambda t: -t[1])


def vib_band_mean(rs, ch, t_start=None):
    """Mean band RMS (power average) of one channel over a set of frames,
    each band over the frames whose window lies after t_start."""
    out = []
    for fc in VIB_BANDS_HZ:
        sel = [r for r in rs if r["ch"] == ch and vib_sees(r, fc, t_start)]
        if sel:
            out.append((fc, (sum(r[f"b{fc:g}"] ** 2 for r in sel) / len(sel)) ** 0.5))
    return out


def summarise_vib(vib):
    by = {}
    for r in vib:
        by.setdefault(r["flight"], []).append(r)
    unit = ["g", "g", "deg/s", "deg/s"]
    for f, rs in by.items():
        flags = 0
        for r in rs:
            flags |= r["stat"]
        print(f"flight {f}: vibration, {len(rs) // 4} spectra"
              + ("  [IMU < 1.8 kHz: nothing above 450 Hz]" if flags & VIB_ST["lowrate"] else "")
              + ("  [samples lost on the Position MCU]" if flags & VIB_ST["overrun"] else ""))
        for name, states in VIB_PHASES:
            sel = [r for r in rs if r["state_name"] in states]
            if not sel:
                continue
            t0 = min(r["state_t0"] for r in sel)
            dur = (sel[-1]["t_ms"] - t0) / 1000.0
            print(f"  {name} ({dur:.1f} s):")
            for ch, cname in enumerate(VIB_CH):
                tones = vib_dominant(sel, ch, t0)[:3]
                bands = sorted(vib_band_mean(sel, ch, t0), key=lambda b: -b[1])[:2]
                # overall RMS from the bands whose windows are in the phase
                rms = [sum(r[f"b{fc:g}"] ** 2 for fc in VIB_BANDS_HZ if vib_sees(r, fc, t0)) ** 0.5
                       for r in sel if r["ch"] == ch and vib_sees(r, 100.0, t0)]
                if not rms:
                    continue
                t = ", ".join(f"{t[0]:.1f} Hz {t[1]:.3g} (max {t[2]:.3g})" for t in tones) or "-"
                b = ", ".join(f"{b[0]:g} Hz" for b in bands)
                print(f"    {cname:17s} RMS max {max(rms):.3g} {unit[ch]}; tones [{unit[ch]}]: {t}; "
                      f"loudest bands: {b}")


def plot_vib(vib, path):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import math
    except ImportError:
        print("--vib-plot needs matplotlib (pip install matplotlib)")
        return
    flights = sorted({r["flight"] for r in vib})
    fig, axes = plt.subplots(4, len(flights), figsize=(7 * len(flights), 11), squeeze=False)
    unit = ["dB re 1 g", "dB re 1 g", "dB re 1 deg/s", "dB re 1 deg/s"]
    for col, fl in enumerate(flights):
        for ch in range(4):
            sel = [r for r in vib if r["flight"] == fl and r["ch"] == ch]
            ax = axes[ch][col]
            if not sel:
                ax.set_visible(False)
                continue
            t0 = sel[0]["t_ms"]
            ts = [(r["t_ms"] - t0) / 1000.0 for r in sel]
            z = [[20 * math.log10(r[f"b{fc:g}"]) if r[f"b{fc:g}"] > 0 else float("nan")
                  for r in sel] for fc in VIB_BANDS_HZ]
            edges = [fc * 2 ** (-1 / 6) for fc in VIB_BANDS_HZ] + [VIB_BANDS_HZ[-1] * 2 ** (1 / 6)]
            dt = 0.512
            tx = [t - dt / 2 for t in ts] + [ts[-1] + dt / 2]
            m = ax.pcolormesh(tx, edges, z, shading="flat", cmap="viridis")
            ax.set_yscale("log")
            ax.set_ylim(edges[0], edges[-1])
            ax.set_ylabel("Hz")
            ax.set_title(f"flight {fl}: {VIB_CH[ch]}")
            fig.colorbar(m, ax=ax, label=unit[ch])
            # mission state changes as vertical lines
            last = None
            for t, r in zip(ts, sel):
                if r["state_name"] != last:
                    ax.axvline(t, color="w", lw=0.6, alpha=0.7)
                    ax.text(t, edges[-1], r["state_name"], color="w", fontsize=7, va="top", rotation=90)
                    last = r["state_name"]
        axes[-1][col].set_xlabel("s")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    print(f"wrote {path}")


def summarise(rows):
    """The numbers the calibration manual asks for, per flight."""
    by = {}
    for r in rows:
        by.setdefault(r["flight"], []).append(r)
    for f, rs in by.items():
        desc = [r for r in rs if r["state_name"] == "DESCENT" and r["drag"] > 0 and r["vz"] < -15]
        burn = [r for r in rs if r["nav_phase"] == 4 and r["tscale"] > 0]   # DG_PHASE_BURN
        apo = max(r["h"] for r in rs)
        print(f"flight {f}: {len(rs)} records, {(rs[-1]['t_ms'] - rs[0]['t_ms']) / 1000:.1f} s, "
              f"apogee {apo:.1f} m")
        if desc:
            print(f"  drag measured/model at the end of the free fall: {desc[-1]['drag']:.2f}"
                  f"   -> g.cd_body x {desc[-1]['drag']:.2f}")
        if burn:
            ts = sorted(r["tscale"] for r in burn)
            print(f"  thrust/table during the burn: median {ts[len(ts) // 2]:.2f}"
                  f"   -> g.thrust_scale x {ts[len(ts) // 2]:.2f}")
        fg = [r["fin_gain"] for r in rs if r["fin_gain"] > 0]
        if fg:
            print(f"  fin effectiveness at the end: {fg[-1]:.2f}   -> fin.gain0 {fg[-1]:.2f}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--port")
    ap.add_argument("--out", default="flight_log.csv")
    ap.add_argument("--raw", help="also save the raw LOGR lines here")
    ap.add_argument("--decode", help="decode a saved raw dump instead of reading the board")
    ap.add_argument("--from", dest="start", type=int, default=0, help="first record")
    ap.add_argument("--info", action="store_true")
    ap.add_argument("--flights-only", action="store_true",
                    help="only the flights (LOG FLIGHTS), each with 3 s of ground data around it")
    ap.add_argument("--erase", action="store_true", help="erase the log (parameters are kept)")
    ap.add_argument("--vib-out", help="vibration spectra CSV (default: <out>_vib.csv)")
    ap.add_argument("--vib-plot", help="also draw the vibration spectrogram to this PNG")
    args = ap.parse_args()

    lines = []
    if args.decode:
        lines = open(args.decode, encoding="utf-8").read().splitlines()
    else:
        if not args.port:
            ap.error("--port or --decode")
        import serial                                   # pyserial
        ser = serial.Serial(args.port, 115200, timeout=0.5)
        time.sleep(0.3)
        ser.reset_input_buffer()

        def cmd(c, until, timeout=10.0):
            ser.write((c + "\n").encode())
            t0 = time.time()
            while time.time() - t0 < timeout:
                ln = ser.readline().decode(errors="replace").strip()
                if ln.startswith(until) or ln.startswith("ERR"):
                    return ln
            return ""

        if args.erase:
            ln = cmd("LOG ERASE", "OK LOG ERASING")
            print(ln)
            if ln.startswith("OK"):
                print(cmd("HB", "OK LOG ERASE", timeout=600))
            return 0 if ln.startswith("OK") else 1
        info = cmd("LOG INFO", "OK LOG INFO")
        print(info or "no answer - is the console closed and the Master connected?")
        if args.info or not info.startswith("OK"):
            return 0 if info.startswith("OK") else 1
        n_total = int(info.split()[4])
        jobs = [(args.start, 0)]
        if args.flights_only:
            ser.write(b"LOG FLIGHTS\n")
            jobs, t0 = [], time.time()
            while time.time() - t0 < 60:
                ln = ser.readline().decode(errors="replace").strip()
                if ln.startswith("LOGF"):
                    p = ln.split()
                    st, cnt = int(p[2]), int(p[3])
                    a = max(0, st - 30)                      # 3 s of 10 Hz ground data
                    jobs.append((a, st - a + cnt + 30))
                    print(f"  flight {int(p[1]) + 1}: {cnt} records, max {p[6]} m")
                elif ln.startswith("OK LOG FLIGHTS") or ln.startswith("ERR"):
                    break
        for start, count in jobs:
            ln = cmd(f"LOG DUMP {start}" + (f" {count}" if count else ""), "OK LOG")
            if not ln.startswith("OK LOG BEGIN"):
                print(ln)
                return 1
            n = int(ln.split()[3])
            t0, last, got = time.time(), time.time(), 0
            while True:
                ln = ser.readline().decode(errors="replace").strip()
                if ln.startswith("LOGR"):
                    lines.append(ln)
                    got += 1
                    last = time.time()
                    if got % 500 == 0:
                        rate = got / max(time.time() - t0, 1e-3)
                        print(f"\r  {got}/{n} records, {rate:.0f}/s", end="", flush=True)
                elif ln.startswith("OK LOG DUMP") or ln.startswith("ERR"):
                    break
                elif time.time() - last > 10:
                    print("\n  timeout")
                    break
            print(f"\r  {got}/{n} records")
        print(f"  {len(lines)} records downloaded (log holds {n_total})")
    if args.raw:
        with open(args.raw, "w", encoding="utf-8") as fh:
            fh.write("\n".join(lines) + "\n")
    rows, vib = split_vib(filter(None, (decode_line(ln) for ln in lines)))
    nf = write_csv(rows, args.out)
    print(f"wrote {args.out}: {len(rows)} records, {nf} flight(s)")
    if vib:
        vpath = args.vib_out or (args.out[:-4] if args.out.lower().endswith(".csv") else args.out) + "_vib.csv"
        write_vib_csv(vib, vpath)
        print(f"wrote {vpath}: {len(vib) // 4} vibration spectra")
    if rows:
        summarise(rows)
    if vib:
        summarise_vib(vib)
        if args.vib_plot:
            plot_vib(vib, args.vib_plot)
    return 0


if __name__ == "__main__":
    sys.exit(main())
