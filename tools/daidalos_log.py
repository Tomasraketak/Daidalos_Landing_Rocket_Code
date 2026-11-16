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
STATES = ["BOOT", "IDLE", "ARMED", "ASCENT", "APOGEE", "DESCENT", "GUIDANCE",
          "LANDED", "RECOVERY", "ABORT", "TEST", "ERROR"]


def decode_line(line: str):
    """'LOGR <n> <hex>' -> (n, dict) or None."""
    parts = line.split()
    if len(parts) != 3 or parts[0] != "LOGR":
        return None
    raw = bytes.fromhex(parts[2])
    if len(raw) != 96 or raw == b"\xff" * 96:
        return None
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
    rows = [d for _, d in filter(None, (decode_line(ln) for ln in lines))]
    nf = write_csv(rows, args.out)
    print(f"wrote {args.out}: {len(rows)} records, {nf} flight(s)")
    if rows:
        summarise(rows)
    return 0


if __name__ == "__main__":
    sys.exit(main())
