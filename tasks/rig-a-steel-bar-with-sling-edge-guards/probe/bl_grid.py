"""Where can the left hand hold a bottom-left guard by its ends (pads closing along the edge) at its seat and 45 mm
outside it (5 mm low), and at its tray block? Tool point offsets (dy out, dz up from the inside corner), approach
angle psi below the outward horizontal, both finger rolls. python bl_grid.py > out/bl_grid.json"""

import json
import math
import os
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE.parent / "tools"))
sys.path.insert(0, str(HERE))

import guard_scan as G  # noqa: E402
import ik_scan as IS  # noqa: E402
import build_assets as BA  # noqa: E402
from rdb_sling_guards import fixed_scene as F  # noqa: E402
from rdb_sling_guards import layout as L  # noqa: E402
from rdb_sling_guards import scripted as S  # noqa: E402

EDGE = os.environ.get("EDGE", "BL")
NAME = {"BL": "guard_2", "TL": "guard_1"}[EDGE]


def job(j):
    if IS.HULLS is None:
        G.init()
    dy, dz, psi, flip = j
    seat, Rs = S.edge_point(EDGE), S.seat_frame(EDGE)
    n2 = Rs[:, 1]
    up = np.array((0., 0., 1.))
    a = -(math.cos(math.radians(psi)) * n2 + math.sin(math.radians(psi)) * up)
    R = S.grip_frame(a, Rs[:, 0])
    if flip:
        R = R @ np.diag((-1., -1., 1.))
    p = seat + dy * n2 + dz * up
    lp, lR = Rs.T @ (p - seat), Rs.T @ R
    pt, Rt = BA.tray_pose(NAME)
    pt = np.asarray(pt) - (0., 0., F.TOP)
    rows = {}
    hold = -(L.GUARD_LEN / 2) - .0005
    r, q = G.check("left", pt + Rt @ lp, Rt @ lR, -(L.GUARD_LEN / 2 + .006))
    rows["tray"] = r
    low = np.array((0., 0., -.005)) if EDGE == "BL" else np.zeros(3)
    out = .045 * n2 if EDGE == "BL" else .030 * Rs @ np.array((0., 1., 1.)) / math.sqrt(2.)
    seed = q
    for nm, cp in (("out", seat + out + low), ("mid", seat + out / 2 + low), ("seat", seat + .002 * n2 + low * .4)):
        r, qq = G.check("left", cp + Rs @ lp, R, hold, seed)
        rows[nm] = r
        seed = qq if qq is not None else seed
    r, _ = G.check("left", seat + .002 * n2 + low * .4 + Rs @ lp, R, -(L.GUARD_LEN / 2 + .006), seed)
    rows["open"] = r
    ok = all(v["ok"] for v in rows.values())
    w = {k: min(v.get(k, -99) for v in rows.values()) for k in ("margin_deg", "fingers_scene_mm", "body_scene_mm", "hook_mm")}
    return {"dy": dy, "dz": dz, "psi": psi, "flip": flip, "ok": ok, "worst": w,
            "fail": [k for k, v in rows.items() if not v["ok"]]}


def main():
    from multiprocessing import Pool

    G.init()
    if EDGE == "BL":
        grid = [(dy, dz) for dy in (.008, .012, .016, .020, .025) for dz in (-.020, -.012, -.006, 0., .008, .016)]
    else:
        grid = [(dy, dz) for dy in (.006, .010, .014, .018) for dz in (.006, .010, .014, .018)]
    psis = [int(v) for v in os.environ.get("PSIS", ",".join(str(p) for p in range(-10, 81, 10))).split(",")]
    jobs = [(dy, dz, psi, flip) for dy, dz in grid for psi in psis for flip in (False, True)]
    with Pool(int(os.environ.get("NP", "28")), initializer=G.init) as pool:
        res = pool.map(job, jobs, chunksize=2)
    print(json.dumps(res, indent=1, default=float))


if __name__ == "__main__":
    main()
