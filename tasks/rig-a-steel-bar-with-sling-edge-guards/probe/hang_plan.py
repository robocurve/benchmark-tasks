"""Double hook (CondorLift) hang paths with the fitting physically threaded over its horn's tip.

Fitting state in the worked horn's plane: apex-bar centre A = (u, v) in mm (u outward along that horn's throat
direction, v up, from its bowl centre) and phi, the angle of A->B (apex bar to base bar) below the outward horizontal;
B = A + 55 (cos phi, -sin phi). The tip (u 38, v 17.7) must pass between the bars; the latch (pivot (-6, 32), free end
at the tip) is pushed down by the apex bar. The hand holds the base bar grip_x along it from its middle, approach turned
gamma from B->A about the bar (gamma 90: the face normal, i.e. the top-down pick of the fitting lying flat).

python hang_plan.py > out/hang_plan.json   (env HOOK_Z, XS, GAMMAS, GRIPS, PATHS)"""

import json
import math
import os

import numpy as np

import ik_scan as S
import rams_ik as RI
import rams_model as R

LATCH_P = np.array((-.006, .032))
LATCH_L = math.hypot(.044, .016)
LATCH_A0 = math.atan2(-.016, .044)

PATHS = {   # name: [(waypoint, (u, v) mm, phi deg)]
    "p35": [("W0-approach", (45, 72), 35), ("W1-prethread", (25, 47), 35), ("W2-on-latch", (24.7, 32), 35),
            ("W3-push", (16, 19.5), 30), ("W4-deep", (10, 5), 25), ("W5-seated", (0, -24), 40)],
    "p45": [("W0-approach", (45, 78), 46), ("W1-prethread", (24.7, 50), 46), ("W2-on-latch", (24.7, 32), 46),
            ("W3-push", (16, 19.5), 32), ("W4-deep", (10, 5), 25), ("W5-seated", (0, -24), 45)],
    "p25": [("W0-approach", (45, 70), 28), ("W1-prethread", (25, 45), 30), ("W2-on-latch", (24.7, 32), 30),
            ("W3-push", (16, 19.5), 22), ("W4-deep", (10, 5), 18), ("W5-seated", (0, -24), 30)],
    "p60": [("W0-approach", (45, 72), 35), ("W1-prethread", (25, 47), 35), ("W2-on-latch", (24.7, 32), 35),
            ("W3-push", (16, 19.5), 30), ("W4-deep", (10, 5), 35), ("W5-seated", (0, -24), 70)],
    # released inside the bowl past the latch (the fitting drops onto the bowl's floor by itself)
    "f15": [("W0-approach", (45, 66), 15), ("W1-prethread", (25, 42), 15), ("W2-on-latch", (24.7, 32), 15),
            ("W3-push", (16, 19.5), 12), ("W4-deep", (10, 5), 12)],
    "f25": [("W0-approach", (45, 68), 25), ("W1-prethread", (25, 44), 25), ("W2-on-latch", (24.7, 32), 25),
            ("W3-push", (16, 19.5), 20), ("W4-deep", (10, 5), 20)],
    "f35": [("W0-approach", (45, 72), 35), ("W1-prethread", (25, 47), 35), ("W2-on-latch", (24.7, 32), 35),
            ("W3-push", (16, 19.5), 30), ("W4-deep", (10, 5), 25)],
    "f35d": [("W0-approach", (45, 72), 35), ("W1-prethread", (25, 47), 35), ("W2-on-latch", (24.7, 32), 35),
             ("W3-push", (16, 19.5), 30), ("W4-deep", (10, 5), 25), ("W5-low", (6, -8), 30)],
    "p35s": [("W0-approach", (45, 72), 35), ("W1-prethread", (25, 47), 35), ("W2-on-latch", (24.7, 32), 35),
             ("W3-push", (16, 19.5), 30), ("W4-deep", (10, 5), 25), ("W5-seated", (0, -24), 60)],
}
GAMMAS = [int(g) for g in os.environ.get("GAMMAS", "60,75,90,105,120").split(",")]
GRIPS = [float(g) for g in os.environ.get("GRIPS", ".026,-.026,.030,-.030").split(",")]
SEL = os.environ.get("PATHS", ",".join(PATHS)).split(",")
SUB = 3                                         # interpolated states between waypoints
GRASP = os.environ.get("GRASP", "base")         # "base": the base bar at grip_x; "side": a side bar at UG from the base
UGS = [float(g) for g in os.environ.get("UGS", ".012,.018,.024").split(",")]
DX = float(os.environ.get("DX", "0"))           # the fitting shifted this far along x, away from the gripped side bar
GRIP_BACK = float(os.environ.get("GRIP_BACK", "0"))   # tool point this far back along the approach (ALIGN only)


def side_pose(lay, A, phi, gamma, sgn, ug, xs):
    """TCP on a side bar's centreline ``ug`` from the base bar (``xs`` +1 / -1: which side bar, along world x), pads
    closing along x; approach turned gamma from B->A toward the face normal on the hand's side (as hang_pose)."""
    apex = lay.hp(A[0], A[1])
    al = math.radians(phi)
    u = -lay.t * math.cos(al) + lay.v * math.sin(al)
    w = lay.n
    a = S.rot(w, math.radians(gamma)) @ u
    base = apex - R.FIT_H * u
    xoff = R.FIT_BASE_HALF - (R.FIT_BASE_HALF - R.FIT_APEX_HALF) * ug / R.FIT_H
    shift = -xs * DX * np.array((1., 0., 0.))
    p = base + ug * u + xs * xoff * np.array((1., 0., 0.)) + shift
    if os.environ.get("ALIGN") == "1":          # pads square to the slanted side bar (scripted.py's grip)
        x = np.array((1., 0., 0.))
        along = -xs * (R.FIT_BASE_HALF - R.FIT_APEX_HALF) * x + R.FIT_H * u
        along /= np.linalg.norm(along)
        across = np.cross(along, np.cross(x, u))
        a2 = a - (a @ across) * across
        a2 /= np.linalg.norm(a2)
        return p - GRIP_BACK * a2, S.frame(a2, sgn * across), apex + shift
    return p, S.frame(a, sgn * np.array((1., 0., 0.))), apex + shift


def states(path):
    out = []
    for k, (name, a, phi) in enumerate(path):
        if k:
            _, a0, p0 = path[k - 1]
            for j in range(1, SUB + 1):
                f = j / (SUB + 1)
                out.append((f"{path[k-1][0]}>{name}:{j}", tuple(np.add(np.multiply(a0, 1 - f), np.multiply(a, f))),
                            p0 + (phi - p0) * f))
        out.append((name, a, phi))
    return [(n, (a[0] / 1000., a[1] / 1000.), p) for n, a, p in out]


def latch_needed(A):
    """Smallest latch rotation (deg, down) that leaves the apex bar's centre A (u, v) 10 mm above the latch's capsule
    (99: not even at the 80 degree stop); None once A is under the closed latch (it has passed the latch's end)."""
    d0 = np.asarray(A) - LATCH_P
    e0 = np.array((math.cos(LATCH_A0), math.sin(LATCH_A0)))
    if e0[0] * d0[1] - e0[1] * d0[0] < -.0099 and 0. < d0 @ e0 < LATCH_L:
        return None
    for th in np.arange(0., 80.5, .5):
        a = LATCH_A0 - math.radians(th)
        e = np.array((math.cos(a), math.sin(a)))
        above = e[0] * d0[1] - e[1] * d0[0]
        t = d0 @ e
        if t > LATCH_L + .0099 or (above >= .0099) or (t > LATCH_L and np.linalg.norm(d0 - LATCH_L * e) >= .0099):
            return float(th)
    return 99.


def fitting_clear(lay, A, phi):
    """Clearance (mm) of the fitting's apex and base bars from the hook (the worked latch left out)."""
    Aw = lay.hp(A[0], A[1])
    ph = math.radians(phi)
    Bw = Aw + R.FIT_H * (math.cos(ph) * lay.t - math.sin(ph) * lay.v)
    ex = np.array((1., 0., 0.))
    apex = np.array([Aw + x * ex for x in np.linspace(-.020, .020, 9)])
    base = np.array([Bw + x * ex for x in np.linspace(-.032, .032, 13)])
    keep = [c for c in lay.caps if c[0] != f"latch{lay.side}"]
    da = min(S.capsule_dist(apex, a, b, r) for _, a, b, r in keep) - R.FIT_BAR_R
    db = min(S.capsule_dist(base, a, b, r) for _, a, b, r in keep) - R.FIT_BAR_R
    return round(1000 * da, 1), round(1000 * db, 1)


def job(j):
    if S.HULLS is None:
        RI.init()
    side, pname, gx, gamma, sgn = j
    lay = RI.Rams(side)
    arm = "right" if side == "R" else "left"
    S.GRIP_X = gx
    rows, prev, ok_all = [], None, True

    def pose(A, phi):
        if GRASP == "side":
            ug, xs = gx
            return side_pose(lay, A, phi, gamma, sgn, ug, xs)
        return S.hang_pose(lay, A, phi, gamma, sgn)

    for name, A, phi in states(PATHS[pname]):
        p, Rm, _ = pose(A, phi)
        skip = ("latch",) if name.startswith(("W3", "W4", "W5")) or ">W3" in name else ()
        hold = -(R.FIT_SIDE_R if GRASP == "side" else R.FIT_BAR_R) - .0005       # pads closed on the bar
        row, res = S.evaluate(lay, arm, p, Rm, hold, name, seeds=[prev] if prev is not None else None, hook_skip=skip)
        row["fit_apex_mm"], row["fit_base_mm"] = fitting_clear(lay, A, phi)
        row["latch_deg"] = latch_needed(np.array(A))
        if res is None:
            ok_all = False
            rows.append(row)
            break
        row["jump_deg"] = round(float(np.degrees(np.abs(res[0] - prev)).max()), 1) if prev is not None else 0.
        prev = res[0]
        rows.append(row)
    # release: open the fingers at the seat, then back the hand out along its approach
    rel = []
    if ok_all:
        _, (u, v), phi = PATHS[pname][-1]
        p, Rm, _ = pose((u / 1000., v / 1000.), phi)
        opened = -(R.FIT_SIDE_R if GRASP == "side" else R.FIT_BAR_R) - float(os.environ.get("RELEASE_OPEN", ".006"))
        for back, fing in ((0., opened), (.03, opened), (.06, opened)):
            row, res = S.evaluate(lay, arm, p - back * Rm[:, 2], Rm, fing, f"release-{int(back*1000)}", seeds=[prev],
                                  hook_skip=("latch",))
            if res is not None:
                row["jump_deg"] = round(float(np.degrees(np.abs(res[0] - prev)).max()), 1)
                prev = res[0]
            rel.append(row)
            if res is None:
                break
    allr = rows + rel
    okr = [r for r in allr if r["ok"]]
    summ = {"side": side, "path": pname, "grip_x": gx, "gamma": gamma, "sgn": sgn, "ok_all": ok_all and all(r["ok"] for r in rel),
            "n_ok": len(okr), "n": len(states(PATHS[pname])) + 3}
    if okr:
        summ.update(min_margin=min(r["margin_deg"] for r in okr), min_fingers=min(r["hook_fingers_mm"] for r in okr),
                    min_body=min(r["hook_body_mm"] for r in okr), min_bar=min(r["bar_body_mm"] for r in okr),
                    max_jump=max(r.get("jump_deg", 0) for r in okr), min_fit_apex=min(r["fit_apex_mm"] for r in rows),
                    min_fit_base=min(r["fit_base_mm"] for r in rows),
                    worst=[(r["tag"], r["margin_deg"], r["hook_fingers_mm"], r["worst_joint"]) for r in
                           sorted(okr, key=lambda r: min(r["margin_deg"] / 3, r["hook_fingers_mm"]))[:3]])
    summ["rows"] = allr
    return summ


def main():
    from multiprocessing import Pool

    RI.init()
    if GRASP == "side":
        grips = [(ug, xs) for ug in UGS for xs in (1., -1.)]
        jobs = [(side, pn, gx, g, sg) for side in ("R", "L") for pn in SEL for gx in grips for g in GAMMAS for sg in (1, -1)]
    else:
        jobs = [(side, pn, gx * (1 if side == "R" else -1), g, sg) for side in ("R", "L") for pn in SEL for gx in GRIPS
                for g in GAMMAS for sg in (1, -1)]
    with Pool(int(os.environ.get("NP", "24")), initializer=RI.init) as pool:
        res = pool.map(job, jobs, chunksize=1)
    out = {"fitting": {pn: [(n, [round(1000 * a[0], 1), round(1000 * a[1], 1)], round(p, 1), fitting_clear(RI.Rams("R"), a, p),
                             latch_needed(np.array(a))) for n, a, p in states(PATHS[pn])] for pn in SEL}}
    for side in ("R", "L"):
        rs = [r for r in res if r["side"] == side]
        good = sorted([r for r in rs if r["ok_all"]], key=lambda r: (min(r["min_fingers"], 10), min(r["min_margin"], 25)),
                      reverse=True)
        out[side] = {"n": len(rs), "ok_all": len(good),
                     "all": [{k: v for k, v in r.items() if k != "rows"} for r in rs],
                     "best": [{k: v for k, v in r.items() if k != "rows"} for r in good[:8]],
                     "best_rows": [r["rows"] for r in good[:2]],
                     "partial": sorted([{k: v for k, v in r.items() if k != "rows"} for r in rs if not r["ok_all"]],
                                       key=lambda r: r["n_ok"], reverse=True)[:4]}
    print(json.dumps(out, indent=1, default=float))


if __name__ == "__main__":
    main()
