#!/usr/bin/env python3
"""Vibration records end to end: frames from the C analyser (vib_test --dump),
packed into 96-byte log slots exactly as Master_MCU.ino's VibLogRecord, mixed
with flight records, decoded by tools/daidalos_log.py. The tones the dump put
into each phase must come out as that phase's dominant tones."""
import os
import struct
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "tools"))
import daidalos_log as dl  # noqa: E402

fails = 0


def check(cond, msg):
    global fails
    if not cond:
        fails += 1
        print("FAIL", msg)


with tempfile.TemporaryDirectory() as d:
    dump = os.path.join(d, "vib.txt")
    subprocess.run([os.path.join(HERE, "vib_test"), "--dump", dump], check=True)
    lines, n, last_t = [], 0, -1
    for ln in open(dump):
        _, t, st, hx = ln.split()
        t, st = int(t), int(st)
        if t != last_t:      # one flight record before each frame, like the 200 Hz stream
            vals = [t] + [0.0] * 12 + [0, 0] + [0.0] * 5 + [0, 0, 0, st, 0, 0, 0, 0, 0, 0, 0, 0, 1]
            lines.append(f"LOGR {n} {struct.pack(dl.FMT, *vals).hex()}")
            n += 1
            last_t = t
        raw = struct.pack("<I", t) + bytes.fromhex(hx) + b"\0\0\0" + bytes([dl.VIB_FLAG | 1])
        check(len(raw) == 96, "slot size")
        lines.append(f"LOGR {n} {raw.hex()}")
        n += 1

rows, vib = dl.split_vib(filter(None, (dl.decode_line(x) for x in lines)))
check(len(vib) >= 4 * 55, f"vib rows {len(vib)}")
check(all(r["fs_hz"] > 1990 for r in vib), "fs")


def top(state, ch):
    sel = [r for r in vib if r["state_name"] == state]
    t = dl.vib_dominant(sel, ch)
    return t[0][0] if t else 0.0


want = [("ASCENT", 0, 87.0), ("ASCENT", 1, 23.0), ("DESCENT", 2, 1.6),
        ("GUIDANCE", 3, 4.3), ("GUIDANCE", 1, 310.0)]
for st, ch, f in want:
    got = top(st, ch)
    check(abs(got - f) < 0.03 * f, f"{st} {dl.VIB_CH[ch]}: dominant {got:.2f} Hz, want {f} Hz")
# the band holding the motor tone is the loudest axial band of the ascent
bands = dl.vib_band_mean([r for r in vib if r["state_name"] == "ASCENT"], 0)
check(max(bands, key=lambda b: b[1])[0] == 80.0, "ascent axial loudest band")
lvl = dict(bands)[80.0]
check(abs(lvl - 2.0 / 2 ** 0.5) / (2.0 / 2 ** 0.5) < 0.2, f"87 Hz band level {lvl:.3f} g")

with tempfile.TemporaryDirectory() as d:
    p = os.path.join(d, "v.csv")
    dl.write_vib_csv(vib, p)
    check(sum(1 for _ in open(p)) == len(vib) + 1, "vib csv rows")

print(f"vibration log: {len(vib) // 4} spectra decoded, {'OK' if not fails else f'{fails} FAILED'}")
sys.exit(1 if fails else 0)
