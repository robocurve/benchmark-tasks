"""Gate check: the mirror case. Throat toward the LEFT arm (+y): the left hand hangs the left fitting (hang2 paths, held
near one end of the base bar, nearly flat), the right hand steadies the hook from its back (-y side). For hook
positions y = HOOKS. (A swivel hook turned to face the left arm, or a ramshorn hook's left horn.)
python mirror_check.py > out/mirror_check.json"""
import json, os, sys
import numpy as np
import ik_scan as S

HOOKS = tuple(float(v) for v in os.environ.get("HOOKS", "0,-.035,-.065,.035,.065").split(","))


def lay(y):
    return S.Layout(f"M{y:+.3f}", axis="x", xs=.25, hook_z=.20, throat=(0., 1., 0.), hook_y=y)


def init():
    S.init()
    S.FIT_H = .055


def steady_job(a):
    if S.HULLS is None:
        init()
    y, c = a
    fam, arm, p, R, extra = c
    row, _ = S.evaluate(lay(y), arm, p, R, -.010, "steady")
    row.update(family=fam, y_hook=y, **extra)
    return row


def hang_job(a):
    if S.HULLS is None:
        init()
    y, gx, g, s = a
    S.LAYOUT, S.GRIP_X = lay(y), gx
    return {"y_hook": y, "grip_x": gx, **S._hang_job(("left", (15, 15, 15, 15), g, s))}


def main():
    from multiprocessing import Pool
    from scipy.spatial import cKDTree
    init()
    S.STEADY_V = (.010, .050, .085)
    sj = [(y, c) for y in HOOKS for c in S.steady_candidates(lay(y)) if c[1] == "right"]
    hj = [(y, gx, g, s) for y in HOOKS for gx in (.026, -.026) for g in (30, 45, 60) for s in (1, -1)]
    with Pool(26, initializer=init) as pool:
        steady = pool.map(steady_job, sj, chunksize=4)
        hang = pool.map(hang_job, hj, chunksize=1)
    out = {}
    for y in HOOKS:
        st = sorted([s for s in steady if s["y_hook"] == y and s["ok"] and s["hook_body_mm"] > 5],
                    key=lambda s: (min(s["hook_body_mm"], 20), s["margin_deg"]), reverse=True)[:4]
        hg = [h for h in hang if h["y_hook"] == y and h["ok_all"]]
        cell = {"steady_ok": len(st), "hang_ok": len(hg)}
        if st:
            cell["steady_best"] = {k: st[0][k] for k in ("family", "v", "az", "margin_deg", "hook_body_mm", "tcp_mm", "approach")}
        if hg:
            h = max(hg, key=lambda h: (min(min(w.get("hook_fingers_mm", -99) for w in h["waypoints"]), 8), h["min_margin_deg"]))
            cell["hang_best"] = {"grip_x": h["grip_x"], "gamma": h["gamma"], "min_margin_deg": h["min_margin_deg"],
                                 "min_hook_body_mm": h["min_hook_body_mm"], "fingers_mm": [w.get("hook_fingers_mm") for w in h["waypoints"]],
                                 "tcp_mm": [w["tcp_mm"] for w in h["waypoints"]]}
            if st:
                d = None
                for s in st:
                    tree = cKDTree(np.concatenate(list(S.HULLS.points(np.radians(s["q_deg"]), S.BASES["right"], -.010).values())))
                    dd = min(float(tree.query(np.concatenate(list(S.HULLS.points(np.radians(w["q_deg"]), S.BASES["left"], -.009).values())))[0].min()) for w in h["waypoints"])
                    d = dd if d is None else max(d, dd)
                cell["arm_arm_mm"] = round(1000 * d, 1)
        out[f"{y:+.3f}"] = cell
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
