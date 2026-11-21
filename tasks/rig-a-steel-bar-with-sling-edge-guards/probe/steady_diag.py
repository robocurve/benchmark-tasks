"""Why the left arm finds no steadying grip on the double hook while the right arm hangs (and the mirror).
python steady_diag.py > out/steady_diag.json"""

import json
import math
import os
import sys

import numpy as np

import ik_scan as S
import rams_ik as RI
import rams_model as R

C = R.RAM_B / 2


def cands(other):
    s = 1. if other == "left" else -1.
    mid = np.array((RI.XS, 0., RI.HZ + R.BOWL_R_IN))
    pts = []
    for th in (-60., -40., -15., 10., 25.):          # the other horn's outer wall, down to the bottom and up to the tip
        a = math.radians(th)
        pts.append((f"horn{int(th)}", mid + np.array((0., s * (C + R.RC * math.cos(a)), R.RC * math.sin(a)))))
    pts.append(("horn-bottom", mid + np.array((0., s * C, -R.RC))))
    for y, z in ((.035, .030), (.035, .055), (.030, .075), (.0, .088), (.020, .086)):
        pts.append((f"eye{int(1000*y)}_{int(1000*z)}", mid + np.array((0., s * y, z))))
    pts.append(("neck", mid + np.array((0., 0., -.010))))
    out = []
    only = os.environ.get("POINTS")
    for name, p in pts:
        if only and name not in only.split(","):
            continue
        for az in range(0, 360, 15):
            for tilt in [float(t) for t in os.environ.get("TILTS", "0,30,-30").split(",")]:
                a = S.rot((1., 0., 0.), math.radians(az)) @ np.array((0., 1., 0.))
                a = S.rot(np.cross(a, (1., 0., 0.)), math.radians(tilt)) @ a
                c = np.array((1., 0., 0.)) - a * a[0]
                for sg in (1., -1.):
                    out.append((other, name, p, S.frame(a, sg * c), az, tilt * sg if False else tilt))
    return out


def job(j):
    if S.HULLS is None:
        RI.init()
    side_hung, (arm, name, p, Rm, az, tilt) = j
    lay = RI.Rams(side_hung)
    row, _ = S.evaluate(lay, arm, p, Rm, -.010, f"steady/{name}")
    row.update(where=name, az=az, tilt=tilt, side_hung=side_hung)
    return row


def main():
    from multiprocessing import Pool

    RI.init()
    jobs = [(side, c) for side in os.environ.get("SIDES", "R,L").split(",") for c in cands("left" if side == "R" else "right")]
    with Pool(int(os.environ.get("NP", "20")), initializer=RI.init) as pool:
        rows = pool.map(job, jobs, chunksize=4)
    out = {}
    for side in os.environ.get("SIDES", "R,L").split(","):
        rs = [r for r in rows if r["side_hung"] == side]
        okr = [r for r in rs if r["ok"]]
        cell = {"n": len(rs), "ik_ok": len(okr)}
        by = {}
        for r in rs:
            w = r["where"]
            b = by.setdefault(w, {"ik_ok": 0, "best_margin_fail": None, "best": None, "worst_joints": {}})
            if r["ok"]:
                b["ik_ok"] += 1
                key = (min(r["hook_body_mm"], 20), r["margin_deg"])
                if b["best"] is None or key > (min(b["best"]["hook_body_mm"], 20), b["best"]["margin_deg"]):
                    b["best"] = {k: r[k] for k in ("az", "tilt", "margin_deg", "hook_body_mm", "hook_fingers_mm", "bar_body_mm",
                                                   "table_mm", "tcp_mm", "approach", "close", "q_deg")}
            else:
                b["worst_joints"][r["worst_joint"]] = b["worst_joints"].get(r["worst_joint"], 0) + 1
                f = (r["err_mm"], r["margin_deg"])
                if b["best_margin_fail"] is None or r["err_mm"] < b["best_margin_fail"][0]:
                    b["best_margin_fail"] = (r["err_mm"], r["rerr_deg"], r["margin_deg"], r["az"], r["tilt"])
        cell["by_point"] = by
        out[side] = cell
    print(json.dumps(out, indent=1, default=float))


if __name__ == "__main__":
    main()
