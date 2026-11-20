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
                             base_pos=station.BASES[arm], iters=120)
        e = pe * 1000 + math.degrees(re)
        if e < best[0]:
            best = (e, q)
    return best


def _main():
    rows = []
    noz = L.nozzle_tcp()
    for roll in (0, 30, 60, 90, -30, -60, -90):
        c = np.array([0.0, math.cos(math.radians(roll)), math.sin(math.radians(roll))])
        rows.append(("left", f"nozzle pinch, roll {roll}", noz, frame((1, 0, 0), c)))
    rows.append(("left", "nozzle, 60 mm back", noz - np.array([0.06, 0, 0]), frame((1, 0, 0), (0, 1, 0))))
    hold = L.neck_tcp()
    rh = frame((0, 1, 0), (1, 0, 0))
    rows.append(("right", "neck grip", hold, rh))
    rows.append(("right", "neck, 60 mm out", hold - np.array([0, 0.06, 0]), rh))
    lift = hold + np.array([0, 0, L.LIFT])
    rows.append(("right", "lifted", lift, rh))
    over = np.array([L.SLOT_XY[0], L.SLOT_XY[1], lift[2]])
    rows.append(("right", "over the free compartment", over, rh))
    down = np.array([L.SLOT_XY[0], L.SLOT_XY[1], hold[2] + L.CADDY_FLOOR + 0.004])
    rows.append(("right", "set down in the compartment", down, rh))
    rows.append(("right", "backed out", down - np.array([0, 0.07, 0]), rh))
    # one-arm plan: the right hand turns the nozzle too
    for roll in (0, 90, -90):
        c = np.array([0.0, math.cos(math.radians(roll)), math.sin(math.radians(roll))])
        rows.append(("right", f"one-arm: nozzle pinch, roll {roll}", noz, frame((1, 0, 0), c)))
    for arm, label, p, r in rows:
        e, q = err(arm, p, r)
        print(f"{arm:5s} {label:34s} at ({p[0]:.3f},{p[1]:.3f},{p[2]-L.TOP:.3f}) err {e:7.1f} mm+deg  q {np.round(q, 2)}")


if __name__ == "__main__":
    _main()
