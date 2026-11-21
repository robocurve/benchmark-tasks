"""Offline check of the scorer's aim rule: turn the bottle (held where the demo holds it, lifted 35 mm) to each tilt and
heading, and print which headings the exit rule calls safe and why the others fail. Usage: python dev/aimcheck.py"""
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import sparkling_cork as t  # noqa: E402
from bodykit import state_from_pose  # noqa: E402

base = {n: state_from_pose(t.SPEC[n], p, q) for n, (p, q) in t.poses().items()}
grip = np.array([t.L.COASTER_XY[0], t.L.COASTER_XY[1], t.TOP + t.L.COASTER_FLOOR + t.GRIP_Z + t.LIFT])


def quat_xyzw(r):
    from bodykit import _quat_from_matrix
    return _quat_from_matrix(r)


for tilt in (0, 15, 30, 45, 60, 75):
    row = []
    for h in range(-180, 180, 15):
        hr = math.radians(h)
        u = np.array([math.cos(hr + math.pi / 2), math.sin(hr + math.pi / 2), 0.0])
        r = t._rot(u, math.radians(tilt))
        pos = grip - r @ np.array([0.0, 0.0, t.GRIP_Z])
        st = dict(base)
        st["bottle"] = state_from_pose(t.SPEC["bottle"], pos, quat_xyzw(r))
        a = t.aim(st)
        why = ("T" if not t.L.TILT_RANGE[0] <= a["tilt"] <= t.L.TILT_RANGE[1] else "") + \
              ("L" if a["lamp_deg"] < t.L.LAMP_CONE else "") + ("G" if a["guest_deg"] < t.L.GUEST_CONE else "") + \
              ("S" if a["self_deg"] < t.L.SELF_CONE else "")
        row.append(f"{h}:{why or 'ok'}")
    print(f"tilt {tilt:2d}: " + " ".join(row))
print("T tilt outside 30-60, L within 25 deg of the lamp, G heading within 45 deg of the guest's plate, "
      "S heading within 45 deg of the robot")
