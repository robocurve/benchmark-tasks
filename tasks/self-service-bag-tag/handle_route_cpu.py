"""CPU geometry for the under-handle handover (not physics).

``scan``: for side-grasp poses on either side of the handle (closing vertical,
approach along the handle axis), solve IK for each arm from several seeds and
report penetration of the posed PhysX convex pieces into the case shell, the
handle arch colliders, the scale and the table, plus minimum clearances.
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import task  # noqa: E402
import yam_hulls  # noqa: E402
from bagtag_probe import frame  # noqa: E402
from exit_guide_pickup_cpu import inside_any  # noqa: E402
from yam_arena import ik, station  # noqa: E402
from yam_arena.constants import FINGER_OPEN, READY  # noqa: E402


def box_rot(spec):
    if len(spec) > 4:
        yaw, tilt = float(spec[3]), float(spec[4])
        cy, sy, ct, st = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(tilt / 2), math.sin(tilt / 2)
        w, x, y, z = cy * ct, cy * st, sy * st, sy * ct
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return np.eye(3)


def obstacle_points(step=.004):
    groups = {
        "case": [(task.bag, b) for b in task.bag.boxes],
        "handle": [(task.bag, b) for b in task.bag.sliders[0].boxes],
        "scale": [(task.scale, b) for b in task.scale.boxes],
        "kiosk": [(task.kiosk, b) for b in task.kiosk.boxes],
    }
    out = {}
    for k, specs in groups.items():
        pts = []
        for prt, b in specs:
            s = np.asarray(b[1], float)
            n = tuple(max(2, int(math.ceil(v / step)) + 1) for v in s)
            c, sy = math.cos(prt.yaw), math.sin(prt.yaw)
            root_R = np.array([[c, -sy, 0], [sy, c, 0], [0, 0, 1]])
            pts.append(yam_hulls.box_points(np.asarray(prt.pos) + root_R @ np.asarray(b[0]), root_R @ box_rot(b), s, n))
        out[k] = np.concatenate(pts)
    out["table"] = yam_hulls.box_points((.45, 0, task.TOP - .005), np.eye(3), (.9, 1.2, .01), (226, 301, 3))
    return out


SEEDS = [READY, [0, 1.4, 1.2, -.4, 0, 0], [.6, 1.2, 1.0, -.3, 0, 0], [-.6, 1.2, 1.0, -.3, 0, 0],
         [0, 1.8, 1.6, -.4, 0, 1.57], [0, 1.0, .6, 0, 0, -1.57], [.8, 1.6, 1.4, -.6, .5, 0],
         [-.8, 1.6, 1.4, -.6, -.5, 0]]


def best_ik(side, pos, R):
    base = station.BASES[side]
    best = None
    for s in SEEDS:
        q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(R), np.asarray(s, float), base_pos=base, iters=300)
        score = pe + .02 * re
        if best is None or score < best[3]:
            best = (q, pe, re, score)
    return best[:3]


def pose_report(side, pos, R, obst, trees, finger=-.003):
    q, pe, re = best_ik(side, pos, R)
    h = yam_hulls.hulls(np.r_[q, finger, finger], side)
    allh = [x for xs in h.values() for x in xs]
    verts = np.concatenate([x.vertices for x in allh])
    row = {"side": side, "tcp": np.round(pos, 4).tolist(), "ik_mm": round(pe * 1000, 2),
           "ik_deg": round(math.degrees(re), 2), "q": np.round(q, 3).tolist()}
    row["inside"] = {k: int(inside_any(p, allh).sum()) for k, p in obst.items()}
    row["clear_mm"] = {k: round(float(t.query(verts)[0].min()) * 1000, 1) for k, t in trees.items()}
    return row


def scan():
    obst = obstacle_points()
    trees = {k: cKDTree(v) for k, v in obst.items()}
    rows = []
    yc = task.HANDLE_WORLD_Y
    for side, sign in (("left", -1), ("right", 1)):
        for xc in (.24, .26, .27, .28, .38, .39, .40, .42):
            for z in (1.115, 1.125, 1.14):
                for pitch in (0, 10):
                    a = np.array([0, sign, -math.tan(math.radians(pitch))])
                    R = frame(a, (0, 0, 1))
                    ty = yc - sign * .003
                    r = pose_report(side, np.array([xc, ty, z]), R, obst, trees)
                    r["pitch_down_deg"] = pitch
                    rows.append(r)
                    print(json.dumps({k: r[k] for k in ("side", "tcp", "pitch_down_deg", "ik_mm", "ik_deg",
                                                        "inside", "clear_mm")}))
    (HERE / "runs/handle-route-scan-cpu.json").write_text(json.dumps(rows, indent=1) + "\n")


if __name__ == "__main__":
    scan()
