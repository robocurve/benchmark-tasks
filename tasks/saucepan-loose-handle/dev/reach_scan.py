"""Scan: best IK error for the left arm over a grid of TCP points at a height, approach leaning toward +x by 0-45 deg.
Usage: python dev/reach_scan.py <z above table>"""
import math, pathlib, sys
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from reach import err, frame  # noqa: E402
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import layout as L  # noqa: E402
z = float(sys.argv[1])
for close in ((1, 0, 0), (0, 1, 0)):
    print("close", close)
    for x in (0.40, 0.45, 0.50, 0.55):
        line = []
        for y in (0.30, 0.20, 0.10, 0.0, -0.05):
            best = 1e9
            for lean in (0, 15, 30, 45):
                t = math.radians(lean)
                r = frame((math.sin(t), 0, -math.cos(t)), close)
                best = min(best, err("left", (x, y, L.TOP + z), r)[0])
            line.append(f"{best:6.1f}")
        print(f"x {x:.2f}: " + " ".join(line))
