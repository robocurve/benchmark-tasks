"""Gate check: throat toward the robot (layout Xf). Each arm hangs its own fitting from the front (hang2 paths, the
fitting held near one end of its base bar, nearly flat); the other arm steadies the hook. Steadying grips at the
shank's top and the eye, both arms; then each hang path paired with the other arm's best steadying grip.
python xf_check.py > out/xf_check.json"""
import json, math, sys
import numpy as np
import ik_scan as S


def init():
    S.init()
    S.FIT_H = .055
    S.LAYOUT = S.LAYOUTS["Xf"]()


def steady_job(c):
    if S.HULLS is None:
        init()
    fam, arm, p, R, extra = c
    row, _ = S.evaluate(S.LAYOUT, arm, p, R, -.010, f"steady/{fam}")
    row.update(family=fam, **extra)
    return row


def hang_job(j):
    if S.HULLS is None:
        init()
    arm, gx, alphas, gamma, sgn = j
    S.GRIP_X = gx
    return {"grip_x": gx, **S._hang_job((arm, alphas, gamma, sgn))}


def main():
    from multiprocessing import Pool
    from scipy.spatial import cKDTree
    init()
    S.STEADY_V = (.050, .065, .075, .085)
    cands = S.steady_candidates(S.LAYOUT)
    hj = [(arm, gx, (15, 15, 15, 15), g, s) for arm in ("left", "right") for gx in (.026, -.026) for g in (15, 30, 45, 60) for s in (1, -1)]
    with Pool(26, initializer=init) as pool:
        steady = pool.map(steady_job, cands, chunksize=4)
        hang = pool.map(hang_job, hj, chunksize=1)
    out = {}
    for arm in ("left", "right"):
        ok = [s for s in steady if s["arm"] == arm and s["ok"] and s["hook_body_mm"] > 5]
        out[f"steady_{arm}"] = {"n_ok": len(ok), "best": sorted(ok, key=lambda s: (min(s["hook_body_mm"], 20), s["margin_deg"]), reverse=True)[:4]}
    for arm, other in (("left", "right"), ("right", "left")):
        hs = [h for h in hang if h["arm"] == arm and h["ok_all"]]
        st = out[f"steady_{other}"]["best"]
        res = []
        for h in hs:
            fing = min(w.get("hook_fingers_mm", -99) for w in h["waypoints"])
            best = None
            for s in st:
                tree = cKDTree(np.concatenate(list(S.HULLS.points(np.radians(s["q_deg"]), S.BASES[other], -.010).values())))
                d = min(float(tree.query(np.concatenate(list(S.HULLS.points(np.radians(w["q_deg"]), S.BASES[arm], -.009).values())))[0].min()) for w in h["waypoints"])
                if best is None or d > best[0]:
                    best = (d, s)
            res.append({"grip_x": h["grip_x"], "gamma": h["gamma"], "sgn": h["sgn"], "min_margin_deg": h["min_margin_deg"],
                        "min_hook_body_mm": h["min_hook_body_mm"], "min_fingers_mm": fing,
                        "arm_arm_mm": round(1000 * best[0], 1) if best else None,
                        "steady": {k: best[1][k] for k in ("family", "v", "az", "margin_deg", "hook_body_mm")} if best else None})
        out[f"hang_{arm}"] = sorted(res, key=lambda r: (min(r["min_fingers_mm"], 8), r["arm_arm_mm"] or -1, r["min_margin_deg"]), reverse=True)[:5]
    for k in ("steady_left", "steady_right"):
        out[k]["best"] = [{kk: s[kk] for kk in ("family", "v", "az", "margin_deg", "hook_body_mm", "tcp_mm", "approach")} for s in out[k]["best"]]
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
