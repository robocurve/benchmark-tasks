"""Guard grips by the end tabs: for each left edge (the right ones mirror), the approach angle below the outward
horizontal (psi) and the tool point's place on the tab, check the arm at the tray pick, along the way in and at the
release point: joint margins, fingers and body against the bar, blocks, trays and table, and against the double hook.

python guard_scan.py > out/guard_scan.json   (run in the task's probe/ folder on a machine with yam_arena)"""

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

import ik_scan as IS  # noqa: E402
import build_assets as BA  # noqa: E402
from rdb_sling_guards import fixed_scene as F  # noqa: E402
from rdb_sling_guards import hook as HK  # noqa: E402
from rdb_sling_guards import layout as L  # noqa: E402
from rdb_sling_guards import scripted as S  # noqa: E402

PSIS = [int(v) for v in os.environ.get("PSIS", "0,15,30,45,60,75,90").split(",")]
BACKS = [float(v) for v in os.environ.get("BACKS", ".004,.008").split(",")]


def boxes():
    out = [((L.BAR_X[0], -L.HALF, L.Z_BOT), (L.BAR_X[1], L.HALF, L.Z_TOP))]
    for x0, x1 in L.BLOCK_X:
        out.append(((x0, -L.BLOCK_L / 2, 0.), (x1, L.BLOCK_L / 2, L.BLOCK_H)))
    for side in ("left", "right"):
        (cx, cy, cz), (hx, hy, hz) = F.tray_block(side)
        out.append(((cx - hx, cy - hy, cz - hz), (cx + hx, cy + hy, cz + hz)))
        x, y = L.TRAY_XY[side]
        sx, sy, fl = L.TRAY_SIZE
        out.append(((x - sx / 2, y - sy / 2, -.05), (x + sx / 2, y + sy / 2, fl)))
    out.append(((-1., -1., -.05), (1., 1., 0.)))                 # the table
    return out


BOXES = None
CAPS = None


def init():
    global BOXES, CAPS
    IS.init()
    BOXES = boxes()
    O = np.array(F.HOOK_O, float)
    O[2] += float(os.environ.get("HOOK_DZ", "0"))
    CAPS = [(O + np.array((0., a[0], a[1])), O + np.array((0., b[0], b[1])), r) for _, a, b, r in HK.hook_capsules()]


def grip(edge, corner, Rg, psi, back):
    """Tool point and rotation: approach in the plane across the edge, psi below the outward horizontal of the
    guard's side face, the tool point ``back`` behind the tabs' middle along the approach."""
    n2 = Rg[:, 1]                                         # the side face's outward normal (horizontal)
    up = np.array((0., 0., 1.))
    a = -(math.cos(math.radians(psi)) * n2 + math.sin(math.radians(psi)) * up)
    b = Rg @ np.array((0., 1., 1.)) / math.sqrt(2.)
    tab = corner + (L.TAB_FROM + L.TAB_TO) / 2 * b
    return tab - back * a, S.grip_frame(a, Rg[:, 0])


def check(arm, p, R, finger, seed=None):
    q, ok, err, rerr, margin = IS.solve(arm, p, R, seeds=[seed] if seed is not None else None)
    row = {"ok": bool(ok), "margin_deg": round(margin, 1)}
    if not ok:
        return row, None
    pts = IS.HULLS.points(q, IS.BASES[arm], finger)
    fing = np.concatenate([pts[k] for k in IS.Hulls.FINGERS])
    body = np.concatenate([v for k, v in pts.items() if k not in IS.Hulls.FINGERS])
    row["fingers_scene_mm"] = round(1000 * min(IS.box_dist(fing, *b) for b in BOXES), 1)
    row["body_scene_mm"] = round(1000 * min(IS.box_dist(body, *b) for b in BOXES), 1)
    allp = np.concatenate([fing, body])
    row["hook_mm"] = round(1000 * min(IS.capsule_dist(allp, a, b, r) for a, b, r in CAPS), 1)
    return row, q


def job(j):
    if IS.HULLS is None:
        init()
    name, psi, back, flip = j
    side, edge, _ = F.tray_slots()[name]
    arm = side
    seat, Rs = S.edge_point(edge), S.seat_frame(edge)
    pt, Rt = BA.tray_pose(name)
    pt = np.asarray(pt) - (0., 0., F.TOP)
    gp, gR = grip(edge, seat, Rs, psi, back)
    if flip:
        gR = gR @ np.diag((-1., -1., 1.))
    # the hand relative to the guard, the same at the tray
    lp, lR = Rs.T @ (gp - seat), Rs.T @ gR
    tray_p, tray_R = pt + Rt @ lp, Rt @ lR
    b = Rs @ np.array((0., 1., 1.)) / math.sqrt(2.)
    out_n = Rs[:, 1]
    if edge in ("TL", "TR"):
        path = [("in30", seat + .030 * b), ("in15", seat + .015 * b), ("release", seat + .0028 * b)]
    else:
        low = np.array((0., 0., -.005))
        path = [("out45", seat + .045 * out_n + low), ("out25", seat + .025 * out_n + low), ("out10", seat + .010 * out_n + low),
                ("under", seat + .002 * out_n + low), ("release", seat + .002 * out_n - np.array((0., 0., .002)))]
    rows = {}
    hold = -(L.GUARD_LEN / 2) - .0005
    r, q = check(arm, tray_p, tray_R, -(L.GUARD_LEN / 2 + .006))
    rows["tray_open"] = r
    r2, q2 = check(arm, tray_p, tray_R, hold, q)
    rows["tray_held"] = r2
    seed = q2
    for nm, cp in path:
        hp, hR = cp + Rs @ lp, Rs @ lR
        r, qq = check(arm, hp, hR, hold, seed)
        rows[nm] = r
        if qq is not None:
            seed = qq
    hp, hR = path[-1][1] + Rs @ lp, Rs @ lR
    rows["release_open"], _ = check(arm, hp, hR, -(L.GUARD_LEN / 2 + .006), seed)
    ok = all(v["ok"] for v in rows.values())
    worst = {k: min(v.get(k, -99) for v in rows.values()) for k in ("margin_deg", "fingers_scene_mm", "body_scene_mm", "hook_mm")}
    worst_held = min((v.get("fingers_scene_mm", -99) for k, v in rows.items() if k not in ("tray_held",)), default=-99)
    return {"guard": name, "edge": edge, "psi": psi, "back": back, "flip": flip, "ok": ok, "worst": worst, "rows": rows}


def main():
    from multiprocessing import Pool

    init()
    jobs = [(n, psi, back, flip) for n in ("guard_1", "guard_2") for psi in PSIS for back in BACKS for flip in (False, True)]
    with Pool(int(os.environ.get("NP", "28")), initializer=init) as pool:
        res = pool.map(job, jobs, chunksize=1)
    print(json.dumps(res, indent=1, default=float))


if __name__ == "__main__":
    main()
