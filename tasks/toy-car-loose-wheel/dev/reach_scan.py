"""Development check: wheel pinch (closing along the car's length) by car heading, hand tilt and car place."""
import math, pathlib, sys
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import layout as L
from reach import err, frame


def pinch(arm, cx, cy, yaw, tilt, axle_x):
    s = 1.0 if arm == "left" else -1.0
    ux = np.array([math.cos(yaw), math.sin(yaw), 0.0])  # the car's length
    uy = np.array([-math.sin(yaw), math.cos(yaw), 0.0])  # its axles
    t = math.radians(tilt)
    a = -s * math.sin(t) * uy - math.cos(t) * np.array([0, 0, 1.0])
    c = np.array([cx, cy, L.TOP + L.WHEEL_R]) + axle_x * ux + s * (L.WHEEL_Y + 0.002) * uy
    return min(err(arm, c - a * L.TCP_ABOVE_AXLE, frame(a, sg * ux))[0] for sg in (1, -1))


for cx in (0.26, 0.32):
    for cy in (-0.12, 0.0, 0.12):
        for yaw in (-30, -15, 0, 15, 30):
            row = []
            for tilt in (20, 35, 50):
                e = max(pinch(arm, cx, cy, math.radians(yaw), tilt, ax) for arm in ("left", "right")
                        for ax in (L.AXLE_X, -L.AXLE_X))
                row.append(f"{e:6.1f}")
            print(f"car ({cx:.2f},{cy:+.2f}) yaw {yaw:+4d}: tilt 0/30/50/70 worst err {' '.join(row)}", flush=True)
