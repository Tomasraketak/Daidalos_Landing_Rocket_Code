#!/usr/bin/env python3
"""
Pull the merged tables out of a dump taken from Ariadna.

Ariadna's `p` command prints, for every stored run, a CSV block (thrust, battery
voltage, current, clamp pulse and closure, per thrust sample) followed by the stand's
own pages in the stand's dump format. Capture it with the stand's tools, which speak
the same console:

    python static_fire.py --port COM7          # Ariadna's port; saves results/<date>/raw_dump.txt

then split it:

    python extract_csv.py results/20261125_101500/raw_dump.txt  [outdir]

Each block becomes run_<n>_stand_<m>.csv. The thrust analysis of the same file is
`python static_fire.py --from-file raw_dump.txt`, exactly as for a dump from the stand.
"""
import re
import sys
from pathlib import Path


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    src = Path(sys.argv[1])
    out = Path(sys.argv[2]) if len(sys.argv) > 2 else src.parent
    out.mkdir(parents=True, exist_ok=True)

    block, name, written = None, None, 0
    for line in src.read_text(encoding="utf-8", errors="replace").splitlines():
        line = line.rstrip()
        m = re.match(r"#BEGIN ARIADNA_CSV run=(\d+) stand_burn=(\d+)", line)
        if m:
            block, name = [], f"run_{m.group(1)}_stand_{m.group(2)}.csv"
        elif line.startswith("#END ARIADNA_CSV") and block is not None:
            (out / name).write_text("\n".join(block) + "\n", encoding="utf-8")
            print(f"{out / name}: {max(len(block) - 1, 0)} rows")
            block, written = None, written + 1
        elif block is not None and not line.startswith("#"):
            block.append(line)
    if not written:
        print("No ARIADNA_CSV block found in", src)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
