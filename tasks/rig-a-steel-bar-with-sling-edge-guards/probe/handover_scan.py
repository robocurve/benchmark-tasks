"""Layout X: can the left hand pass the left fitting to the right hand (the hook's throat faces the right arm)?

Both hands hold the fitting's 64 mm base bar (along x) near its two ends (left at x = 0.25 + 0.026, right at
0.25 - 0.026), each with its approach in the y-z plane (any direction; fingers closing across the bar). For each
candidate base-bar position (y, z) in the sling's plane, the best left and right poses (joint margin) and the two
arms' hull clearance, and both hands' clearance to the hook and the bar.

python handover_scan.py --jobs 20 > out/ik_X_handover.json
"""

import argparse
import itertools
import json
import math
import sys

import numpy as np

import ik_scan as S


def job(args):
    y, z, arm, ang, sgn = args
    if S.LAYOUT is None:
        S._init("X")
    L = S.LAYOUT
    x = .25 + (.026 if arm == "left" else -.026)
    a = np.array((0., math.cos(math.radians(ang)), math.sin(math.radians(ang))))
    c = sgn * np.cross((1., 0., 0.), a)
    row, res = S.evaluate(L, arm, np.array((x, y, z)), S.frame(a, c), -.009, f"handover {arm}")
    row.update(y=y, z=z, ang=ang, sgn=sgn)
    return row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--jobs", type=int, default=20)
    args = ap.parse_args()
    from multiprocessing import Pool
    from scipy.spatial import cKDTree

    S._init("X")
    ys = (-.10, -.07, -.04, -.01, .02, .05, .08)
    zs = (.17, .20, .23, .26)
    angs = range(-180, 180, 20)        # approach direction in the y-z plane (0 = +y, -90 = straight down)
    jobs = [(y, z, arm, ang, sgn) for y, z in itertools.product(ys, zs) for arm in ("left", "right") for ang in angs for sgn in (1, -1)]
    with Pool(args.jobs, initializer=S._init, initargs=("X",)) as pool:
        rows = pool.map(job, jobs, chunksize=4)
    out = []
    for y, z in itertools.product(ys, zs):
        best = {}
        for arm in ("left", "right"):
            cand = [r for r in rows if r["y"] == y and r["z"] == z and r["arm"] == arm and r["ok"]
                    and r["hook_body_mm"] > 5 and r["bar_body_mm"] > 10]
            best[arm] = sorted(cand, key=lambda r: (min(r["hook_body_mm"], 20) + min(r["bar_body_mm"], 20), r["margin_deg"]), reverse=True)[:4]
        cell = {"y_mm": 1000 * y, "z_mm": 1000 * z, "n_left": len(best["left"]), "n_right": len(best["right"])}
        if best["left"] and best["right"]:
            pairs = []
            for lr in best["left"]:
                lp = np.concatenate(list(S.HULLS.points(np.radians(lr["q_deg"]), S.BASES["left"], -.009).values()))
                tree = cKDTree(lp)
                for rr in best["right"]:
                    rp = np.concatenate(list(S.HULLS.points(np.radians(rr["q_deg"]), S.BASES["right"], -.009).values()))
                    d = float(tree.query(rp)[0].min())
                    pairs.append((d, lr, rr))
            d, lr, rr = max(pairs, key=lambda p: (min(p[0], .02), min(p[1]["margin_deg"], p[2]["margin_deg"])))
            cell.update(arm_arm_mm=round(1000 * d, 1), left={k: lr[k] for k in ("ang", "margin_deg", "hook_body_mm", "bar_body_mm", "tcp_mm")},
                        right={k: rr[k] for k in ("ang", "margin_deg", "hook_body_mm", "bar_body_mm", "tcp_mm")})
        out.append(cell)
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
