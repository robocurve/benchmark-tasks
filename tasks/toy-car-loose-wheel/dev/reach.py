"""Development check (no simulation): can each arm reach every grasp and end pose of the plan, with joint limits?
Usage: python dev/reach.py"""
import math
import pathlib
import sys

import numpy as np
from yam_arena import ik, station
from yam_arena.constants import READY

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import layout as L  # noqa: E402


def frame(approach, close):
    z = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(close, float) - z * float(np.dot(close, z))
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def err(arm, p, r):
    best = (1e9, None)
    for q0 in (READY, [0.0, 1.5, 1.2, -0.3, 0.0, 0.0], [0.5, 1.2, 1.0, -0.5, 0.0, 1.5], [-0.5, 1.2, 1.0, -0.5, 0.0, -1.5]):
        q, pe, re = ik.solve(np.asarray(p, float), ik.matrix_to_quat_xyzw(r), np.array(q0, float),
                             base_pos=station.BASES[arm], iters=150)
        e = pe * 1000 + math.degrees(re)
        if e < best[0]:
            best = (e, q)
    return best


def tug_pose(arm, car_y, axle_x, pulled=0.0, tilt=L.TUG_TILT_DEG):
    s = 1.0 if arm == "left" else -1.0  # the left hand takes the +y wheel
    t = math.radians(tilt)
    a = np.array([0.0, -s * math.sin(t), -math.cos(t)])  # down, leaning in from outside
    c = np.array([L.CAR_X + axle_x, car_y + s * (L.WHEEL_Y + 0.002 + pulled), L.TOP + L.WHEEL_R])
    return c - a * L.TCP_ABOVE_AXLE, frame(a, (1.0, 0.0, 0.0))


def _main():
    print("bases", station.BASES)
    rows = []
    for tilt in (0.0, 25.0, 40.0):
        for y in L.SLOTS_Y:
            for ax in (L.AXLE_X, -L.AXLE_X):
                for arm in ("left", "right"):
                    for pulled in (0.0, L.TUG_PULL):
                        p, r = tug_pose(arm, y, ax, pulled, tilt)
                        rows.append((arm, f"tug tilt {tilt:.0f} car y {y:+.2f} axle {ax:+.3f} out {pulled*1000:.0f}", p, r))
    down_y = frame((0, 0, -1), (0, 1, 0))
    down_x = frame((0, 0, -1), (1, 0, 0))
    for arm in ("left", "right"):
        for y in L.SLOTS_Y:
            for z in (L.PICK_TCP_Z, L.LIFT_Z):
                rows.append((arm, f"pick car y {y:+.2f} z {z:.3f}", np.array([L.CAR_X, y, L.TOP + z]), down_y))
        for (x, y) in L.BASKET_SPOTS + (L.BOX_SPOT,):
            for z in (L.LIFT_Z, L.FLOOR + L.DROP_Z):
                for nm, r in (("y", down_y), ("x", down_x)):
                    rows.append((arm, f"drop ({x:.2f},{y:+.2f}) z {z:.3f} close {nm}", np.array([x, y, L.TOP + z]), r))
    for arm, label, p, r in rows:
        e, q = err(arm, p, r)
        flag = "" if e < 3 else "  <-- BAD"
        print(f"{arm:5s} {label:48s} err {e:7.1f}{flag}")


if __name__ == "__main__":
    _main()
