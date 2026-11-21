"""Gate check for the left-fitting route "hook parked off-centre in the sling plane" (layout X, throat -y).

The hook hangs on a long pendulum (a chain hoist's load chain) whose top is at y = Y_PARK; at rest its bowl is over
(0.25, Y_PARK). The left hand hands the left fitting to the right hand near y = 0 (free of the parked hook), then
steadies the hook, pulling it toward itself to Y_STEADY; the right hand hangs both fittings at the throat (-y side).
For each (Y_PARK, Y_STEADY): guard poses' clearance to the parked hook, the hand-over, the steadying grip, the hang
path and the two arms together.   python plan_check.py --jobs 26 > out/plan_check.json
"""

import argparse
import itertools
import json
import math
import sys

import numpy as np

import ik_scan as S

import os
PARKS = tuple(float(v) for v in os.environ.get('PARKS', '-.05,-.065,-.08').split(','))
STEADY_SHIFT = tuple(float(v) for v in os.environ.get('SHIFTS', '0,.02,.035').split(','))
HANG_GRIPS = [((15, 15, 15, 15), g, 1) for g in (30, 45, 60)]


def layout(y):
    return S.Layout(f"P{y:+.3f}", axis="x", xs=.25, hook_z=.20, throat=(0., -1., 0.), hook_y=y)


def init():
    S.init()
    S.FIT_H, S.GRIP_X = .055, .026


def guard_job(args):
    y_park, cand = args
    if S.HULLS is None:
        init()
    ename, fam, arm, p, R, finger, extra = cand
    row, _ = S.evaluate(layout(y_park), arm, p, R, finger, f"{ename}/{fam}")
    row.update(edge=ename, family=fam, park=y_park, **extra)
    return row


def hang_job(args):
    y_hook, alphas, gamma, sgn = args
    if S.HULLS is None:
        init()
    S.LAYOUT = layout(y_hook)
    return {"y_hook": y_hook, **S._hang_job(("right", alphas, gamma, sgn))}


def steady_job(args):
    y_hook, cand = args
    if S.HULLS is None:
        init()
    fam, arm, p, R, extra = cand
    row, _ = S.evaluate(layout(y_hook), arm, p, R, -.010, f"steady/{fam}")
    row.update(family=fam, y_hook=y_hook, **extra)
    return row


def handover_job(args):
    y_park, y, z, arm, ang, sgn = args
    if S.HULLS is None:
        init()
    L = layout(y_park)
    x = .25 + (.026 if arm == "left" else -.026)
    a = np.array((0., math.cos(math.radians(ang)), math.sin(math.radians(ang))))
    c = sgn * np.cross((1., 0., 0.), a)
    row, _ = S.evaluate(L, arm, np.array((x, y, z)), S.frame(a, c), -.009, "handover")
    # the fitting itself (apex straight up, 55 mm) against the parked hook
    fit = np.array([(xx, y, z + t) for xx in np.linspace(.218, .282, 9) for t in np.linspace(0., .055, 12)])
    row.update(park=y_park, y=y, z=z, ang=ang, sgn=sgn, fitting_hook_mm=round(1000 * (L.hook_dist(fit) - .007), 1))
    return row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--jobs", type=int, default=26)
    args = ap.parse_args()
    from multiprocessing import Pool
    from scipy.spatial import cKDTree

    init()
    S.END_PINCH = True
    out = {"parks": PARKS, "steady_shift": STEADY_SHIFT}
    with Pool(args.jobs, initializer=init) as pool:
        # 1. guard poses against the parked hook (place by the ends, and the seating pushes)
        jobs = []
        for y_park in PARKS:
            for cand in S.guard_candidates(layout(y_park)):
                if cand[1] in ("E-endpinch", "T2-push", "B2-push"):
                    jobs.append((y_park, cand))
        rows = pool.map(guard_job, jobs, chunksize=4)
        g = {}
        for r in rows:
            k = f"{r['park']:+.3f}|{r['edge']}|{r['family']}|{r['arm']}"
            ok = r["ok"] and r.get("bar_body_mm", -1) >= 20 and r.get("hook_body_mm", -1) >= 20
            best = g.get(k)
            key = (ok, min(r.get("hook_body_mm", -99), r.get("bar_body_mm", -99)), r["margin_deg"])
            if best is None or key > best[0]:
                g[k] = (key, r)
        out["guards"] = {k: {"clear": v[0][0], "min_clear_mm": v[0][1], **{f: v[1].get(f) for f in (
            "margin_deg", "bar_body_mm", "hook_body_mm", "bar_fingers_mm", "tcp_mm", "approach")}} for k, v in sorted(g.items())}
        print("guards done", file=sys.stderr, flush=True)
        # 2. hand-over near y = 0 with the hook parked
        jobs = [] if os.environ.get('NO_HANDOVER') else [(p, y, z, arm, ang, sgn) for p in PARKS for y in (-.01, 0., .01) for z in (.19, .22, .25)
                for arm in ("left", "right") for ang in range(-180, 180, 20) for sgn in (1, -1)]
        rows = pool.map(handover_job, jobs, chunksize=4)
        ho = []
        for p, y, z in ([] if not jobs else itertools.product(PARKS, (-.01, 0., .01), (.19, .22, .25))):
            best = {}
            for arm in ("left", "right"):
                cand = [r for r in rows if r["park"] == p and r["y"] == y and r["z"] == z and r["arm"] == arm and r["ok"]
                        and r["hook_body_mm"] > 10 and r["bar_body_mm"] > 10]
                best[arm] = sorted(cand, key=lambda r: (min(r["hook_body_mm"], 25), r["margin_deg"]), reverse=True)[:3]
            cell = {"park": p, "y": y, "z": z, "n_left": len(best["left"]), "n_right": len(best["right"]),
                    "fitting_hook_mm": rows[[i for i, r in enumerate(rows) if r["park"] == p and r["y"] == y and r["z"] == z][0]]["fitting_hook_mm"]}
            if best["left"] and best["right"]:
                pairs = []
                for lr in best["left"]:
                    tree = cKDTree(np.concatenate(list(S.HULLS.points(np.radians(lr["q_deg"]), S.BASES["left"], -.009).values())))
                    for rr in best["right"]:
                        d = float(tree.query(np.concatenate(list(S.HULLS.points(np.radians(rr["q_deg"]), S.BASES["right"], -.009).values())))[0].min())
                        pairs.append((d, lr, rr))
                d, lr, rr = max(pairs, key=lambda q: (min(q[0], .025), min(q[1]["margin_deg"], q[2]["margin_deg"])))
                cell.update(arm_arm_mm=round(1000 * d, 1),
                            left={k: lr[k] for k in ("ang", "margin_deg", "hook_body_mm", "bar_body_mm", "tcp_mm", "q_deg")},
                            right={k: rr[k] for k in ("ang", "margin_deg", "hook_body_mm", "bar_body_mm", "tcp_mm", "q_deg")})
            ho.append(cell)
        out["handover"] = ho
        print("handover done", file=sys.stderr, flush=True)
        # 3. steadying (left) and hanging (right) with the hook pulled to y_park + shift
        hooks = sorted({round(p + s, 4) for p in PARKS for s in STEADY_SHIFT})
        S.STEADY_V = (.010, .030, .085)
        jobs = [(yh, c) for yh in hooks for c in S.steady_candidates(layout(yh)) if c[1] == "left"]
        steady = pool.map(steady_job, jobs, chunksize=4)
        jobs = [(yh, al, ga, sg) for yh in hooks for al, ga, sg in HANG_GRIPS]
        hang = pool.map(hang_job, jobs, chunksize=1)
    res = {}
    for yh in hooks:
        st = sorted([s for s in steady if s["y_hook"] == yh and s["ok"] and s["hook_body_mm"] > 5],
                    key=lambda s: (min(s["hook_body_mm"], 20), s["margin_deg"]), reverse=True)[:4]
        hg = [h for h in hang if h["y_hook"] == yh and h["ok_all"]]
        cell = {"steady_ok": len(st), "hang_ok": len(hg)}
        if st:
            cell["steady_best"] = {k: st[0][k] for k in ("family", "v", "az", "margin_deg", "hook_body_mm", "tcp_mm", "approach")}
        if hg:
            h = max(hg, key=lambda h: (min(min(w.get("hook_fingers_mm", -99) for w in h["waypoints"]), 10), h["min_margin_deg"]))
            cell["hang_best"] = {"gamma": h["gamma"], "min_margin_deg": h["min_margin_deg"], "min_hook_body_mm": h["min_hook_body_mm"],
                                 "fingers_mm": [w.get("hook_fingers_mm") for w in h["waypoints"]],
                                 "tcp_mm": [w["tcp_mm"] for w in h["waypoints"]]}
            if st:
                best = None
                for s in st:
                    tree = cKDTree(np.concatenate(list(S.HULLS.points(np.radians(s["q_deg"]), S.BASES["left"], -.010).values())))
                    d = min(float(tree.query(np.concatenate(list(S.HULLS.points(np.radians(w["q_deg"]), S.BASES["right"], -.009).values())))[0].min())
                            for w in h["waypoints"])
                    if best is None or d > best[0]:
                        best = (d, s)
                cell["arm_arm_mm"] = round(1000 * best[0], 1)
                cell["pair_steady"] = {k: best[1][k] for k in ("family", "v", "az", "margin_deg", "hook_body_mm")}
        res[f"{yh:+.3f}"] = cell
    out["steady_hang"] = res
    print(json.dumps(out, indent=1, default=float))


if __name__ == "__main__":
    main()
