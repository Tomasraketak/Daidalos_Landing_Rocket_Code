#!/usr/bin/env python3
"""
Pin ground_visualizer.py's packet decoder against the C definition.

The downlink packet is defined once, in ariadna_link.h, and decoded twice —
by ground_station.ino (which just memcpy's it, so it cannot be wrong) and by
ground_visualizer.py (which uses a struct format string, which very much
can). link_test.cpp checks the C side. This checks the Python side against
the same header, so the two cannot drift apart without a test failing.

The failure this prevents is not a crash. A format string one field out of
step still unpacks 48 bytes successfully; it just puts the battery voltage
in the altitude box. That is the kind of bug you find during a flight.

Run:  cd test && make pyfmt
"""

import os
import re
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
HEADER = os.path.join(HERE, "..", "Ariadna_V2", "common", "ariadna_link.h")
VIS = os.path.join(HERE, "..", "Ariadna_V2", "Ground_Visualizer", "ground_visualizer.py")

# C type -> (struct code, size)
CTYPES = {
    "uint8_t": ("B", 1), "int8_t": ("b", 1),
    "uint16_t": ("H", 2), "int16_t": ("h", 2),
    "uint32_t": ("I", 4), "int32_t": ("i", 4),
    "float": ("f", 4),
}

failures = []


def check(ok, msg):
    if not ok:
        failures.append(msg)


def parse_struct(source, name):
    """Pull the field list out of a `typedef struct { ... } name;` block."""
    # The body must not contain a brace of its own, otherwise a lazy match
    # starting at the FIRST `typedef struct {` in the file happily spans
    # every struct between there and this one.
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
                fields.append((arr.group(1), code * int(arr.group(2)),
                               size * int(arr.group(2))))
            else:
                fields.append((nm.lstrip("*"), code, size))
    return fields


def main():
    src = open(HEADER, encoding="utf-8").read()
    vis = open(VIS, encoding="utf-8").read()

    # ---- the C side ----------------------------------------------------
    tlm = parse_struct(src, "ariadna_tlm_t")
    check(tlm is not None, "could not find ariadna_tlm_t in ariadna_link.h")
    if tlm is None:
        return report()

    c_fmt = "<" + "".join(f[1] for f in tlm)
    c_size = sum(f[2] for f in tlm)
    check(c_size == 46, f"ariadna_tlm_t is {c_size} bytes, expected 46")
    check(struct.calcsize(c_fmt) == c_size,
          f"packed size {struct.calcsize(c_fmt)} != field sum {c_size}")

    # The ground station appends RSSI and SNR on the way to the laptop.
    c_gs_fmt = c_fmt + "bb"
    check(struct.calcsize(c_gs_fmt) == 48,
          f"gs serial packet is {struct.calcsize(c_gs_fmt)} bytes, expected 48")

    # ---- the Python side ------------------------------------------------
    m = re.search(r'^GS_FORMAT\s*=\s*(.+)$', vis, re.M)
    check(m is not None, "could not find GS_FORMAT in ground_visualizer.py")
    if m is None:
        return report()

    py_fmt = eval(m.group(1), {"__builtins__": {}}, {})
    check(py_fmt == c_gs_fmt,
          f"format mismatch\n      header: {c_gs_fmt}\n      python: {py_fmt}")
    check(struct.calcsize(py_fmt) == 48,
          f"python format is {struct.calcsize(py_fmt)} bytes, expected 48")

    # ---- field order ------------------------------------------------------
    # The format string alone cannot catch two same-typed fields being
    # swapped, so check that the visualiser unpacks into names matching the
    # header's order.
    um = re.search(r"\(([^)]*?)\)\s*=\s*u\b", vis, re.S)
    check(um is not None, "could not find the unpack tuple in ground_visualizer.py")
    if um is not None:
        py_names = [n.strip() for n in um.group(1).replace("\n", " ").split(",") if n.strip()]
        c_names = [f[0] for f in tlm] + ["rssi", "snr"]
        check(len(py_names) == len(c_names),
              f"unpack tuple has {len(py_names)} names, header has {len(c_names)}")

        # The visualiser renames as it unpacks (alt_dm -> alt_dm, roll_ddeg ->
        # roll_d and so on), so compare by position and type rather than by
        # spelling: every C name must be a prefix of, or match, the Python one.
        if len(py_names) == len(c_names):
            for i, (cn, pn) in enumerate(zip(c_names, py_names)):
                base = cn.rstrip("_dmscg")
                check(pn.startswith(base[:4]) or cn.startswith(pn[:4]),
                      f"field {i}: header '{cn}' vs python '{pn}' — order may be wrong")

    # ---- version ------------------------------------------------------------
    vm = re.search(r"#define\s+ARIADNA_TLM_VERSION\s+(\d+)", src)
    pm = re.search(r"^TLM_VERSION\s*=\s*(\d+)", vis, re.M)
    check(vm is not None and pm is not None, "could not read the packet version")
    if vm and pm:
        check(vm.group(1) == pm.group(1),
              f"version mismatch: header {vm.group(1)}, python {pm.group(1)}")

    # ---- downlink rate ----------------------------------------------------
    # The visualiser measures link quality against this. If it disagrees
    # with the vehicle, a perfect link reads as a bad one (or vice versa,
    # which is worse).
    cm = re.search(r"#define\s+ARIADNA_TLM_HZ\s+(\d+)", src)
    pm = re.search(r"^TLM_HZ\s*=\s*(\d+)", vis, re.M)
    check(cm is not None and pm is not None, "could not read the downlink rate")
    if cm and pm:
        check(cm.group(1) == pm.group(1),
              f"downlink rate mismatch: header {cm.group(1)} Hz, python {pm.group(1)} Hz")

    # ---- framing constants ----------------------------------------------------
    for cname, pname in (("ARIADNA_SYNC0", "SYNC0"), ("ARIADNA_SYNC1", "SYNC1"),
                         ("ARIADNA_PKT_TELEMETRY", "PKT_TELEMETRY")):
        cm = re.search(cname + r"\s*=?\s*(0x[0-9A-Fa-f]+)", src)
        pmm = re.search(r"\b" + pname + r"\s*=\s*(0x[0-9A-Fa-f]+)", vis)
        check(cm is not None and pmm is not None, f"could not read {cname}/{pname}")
        if cm and pmm:
            check(int(cm.group(1), 16) == int(pmm.group(1), 16),
                  f"{cname} mismatch: {cm.group(1)} vs {pmm.group(1)}")

    return report()


def report():
    if failures:
        print("ground_visualizer.py does NOT match ariadna_link.h:\n")
        for f in failures:
            print("  FAIL  " + f)
        print()
        return 1
    print("ground_visualizer.py decodes the current packet layout correctly")
    return 0


if __name__ == "__main__":
    sys.exit(main())
