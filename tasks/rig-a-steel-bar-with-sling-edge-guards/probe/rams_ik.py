"""Double hook (Carl Stahl CondorLift 1,000 kg) reach check: each arm hangs its own fitting into its own horn (hang2
paths: the fitting held near one end of its base bar, nearly flat), the other arm steadies the hook by its own horn's
outside or the eye's side; the two arms' hulls, and the steadying hand's clearance from the fitting's sweep.
python rams_ik.py > out/rams_ik.json"""

import json
import math
import sys

import numpy as np

import ik_scan as S
import rams_model as R

C = R.RAM_B / 2
import os  # noqa: E402
HZ = float(os.environ.get("HOOK_Z", ".20"))
XS = float(os.environ.get("XS", ".25"))
W1V = float(os.environ.get("W1V", ".066"))
W2V = float(os.environ.get("W2V", ".058"))
ALPHAS = [int(a) for a in os.environ.get("ALPHAS", "15").split(",")]
GAMMAS = [int(a) for a in os.environ.get("GAMMAS", "30,45,60").split(",")]


class Rams(S.Layout):
    """The bar and edges as layout X; the hook replaced by the double hook. ``side`` 'R': the right horn is worked
    (its throat faces -y), 'L': the left horn (+y). O is that horn's bowl centre."""

    def __init__(self, side):
        sgn = -1. if side == "R" else 1.
        super().__init__(f"rams{side}", axis="x", xs=XS, hook_z=HZ, throat=(0., sgn, 0.), hook_y=0.)
        self.side = side
        self.O = np.array((XS, sgn * C, HZ + R.BOWL_R_IN))
        mid = np.array((XS, 0., HZ + R.BOWL_R_IN))
        self.caps = [(nm, np.asarray(a) - R.HOOK_O + mid, np.asarray(b) - R.HOOK_O + mid, r) for nm, a, b, r in R.hook_capsules()]
        for k, (lp, le) in R.LATCHES.items():
            self.caps.append((f"latch{k}", lp - R.HOOK_O + mid, le - R.HOOK_O + mid, .003))

    def hook_dist(self, pts, skip=()):
        keep = [c for c in self.caps if not (("latch" in skip) and c[0] == f"latch{self.side}") and c[0] not in skip]
        return min(S.capsule_dist(pts, a, b, r) for _, a, b, r in keep)


def waypoints(layout):
    rc = R.RC
    return [("W1-over-tip", (rc + .014, W1V)), ("W2-over-throat", (.006, W2V)), ("W3-on-latch", (-.004, .014)),
            ("W4-seated", (0., -R.BOWL_R_IN + R.FIT_BAR_R))]


def init():
    S.init()
    S.FIT_H = .055
    S.hang_waypoints = waypoints


def hang_job(j):
    if S.HULLS is None:
        init()
    side, arm, gx, alphas, gamma, sgn = j
    S.LAYOUT, S.GRIP_X = Rams(side), gx
    return {"side": side, "grip_x": gx, **S._hang_job((arm, alphas, gamma, sgn))}


def steady_cands(side_hung):
    """The other arm on its own horn's outside wall or the eye's side, pads closing along x."""
    other = "left" if side_hung == "R" else "right"
    s = 1. if other == "left" else -1.
    mid = np.array((XS, 0., HZ + R.BOWL_R_IN))
    pts = []
    for th in (-40., -15., 10.):                       # the outside of the other horn
        a = math.radians(th)
        pts.append(("horn", mid + np.array((0., s * (C + R.RC * math.cos(a)), R.RC * math.sin(a)))))
    for y, z in ((.035, .030), (.035, .055), (.030, .075)):   # the eye's side wall on the other arm's side
        pts.append(("eye", mid + np.array((0., s * y, z))))
    out = []
    for name, p in pts:
        for az in range(0, 360, 20):
            a = S.rot((1., 0., 0.), math.radians(az)) @ np.array((0., 1., 0.))
            out.append((other, name, p, S.frame(a, (1., 0., 0.)), az))
    return out


def steady_job(j):
    if S.HULLS is None:
        init()
    side_hung, (arm, name, p, Rm, az) = j
    lay = Rams(side_hung)
    row, _ = S.evaluate(lay, arm, p, Rm, -.010, f"steady/{name}")
    row.update(where=name, az=az, side_hung=side_hung)
    return row


def main():
    from multiprocessing import Pool
    from scipy.spatial import cKDTree

    init()
    hj = [(side, "right" if side == "R" else "left", gx, (al, al, al, al), g, sg)
          for side in ("R", "L") for gx in (.026, -.026) for g in GAMMAS for sg in (1, -1) for al in ALPHAS]
    sj = [(side, c) for side in ("R", "L") for c in steady_cands(side)]
    with Pool(26, initializer=init) as pool:
        hang = pool.map(hang_job, hj, chunksize=1)
        steady = pool.map(steady_job, sj, chunksize=4)
    out = {}
    for side in ("R", "L"):
        lay = Rams(side)
        arm = "right" if side == "R" else "left"
        other = "left" if side == "R" else "right"
        hg = [h for h in hang if h["side"] == side and h["ok_all"]]
        st = sorted([s for s in steady if s["side_hung"] == side and s["ok"] and s["hook_body_mm"] > 5],
                    key=lambda s: (min(s["hook_body_mm"], 20), s["margin_deg"]), reverse=True)[:6]
        cell = {"hang_ok": len(hg), "steady_ok": len(st)}
        if st:
            cell["steady_best"] = [{k: s[k] for k in ("where", "az", "margin_deg", "hook_body_mm", "bar_body_mm", "tcp_mm", "approach", "q_deg")} for s in st[:3]]
        rows = []
        for h in hg:
            fing = min(w.get("hook_fingers_mm", -99) for w in h["waypoints"])
            best = None
            for s in st:
                tree = cKDTree(np.concatenate(list(S.HULLS.points(np.radians(s["q_deg"]), S.BASES[other], -.010).values())))
                d = min(float(tree.query(np.concatenate(list(S.HULLS.points(np.radians(w["q_deg"]), S.BASES[arm], -.009).values())))[0].min())
                        for w in h["waypoints"])
                # the fitting's sweep (base bar, sides, apex bar at each waypoint) against the steadying hand
                fit = []
                for (_, wp), al in zip(waypoints(lay), h["alpha"]):
                    S.GRIP_X = h["grip_x"]
                    tcp, Rr, ap = S.hang_pose(lay, wp, al, h["gamma"], h["sgn"])
                    base = tcp - h["grip_x"] * lay.n
                    for k in np.linspace(0, 1, 10):
                        fit.append(base + (2 * k - 1) * .032 * lay.n)
                        fit.append(ap + (2 * k - 1) * .020 * lay.n)
                        for sd in (-1, 1):
                            fit.append(base + sd * .032 * lay.n + k * (ap + sd * .020 * lay.n - base - sd * .032 * lay.n))
                fd = float(tree.query(np.array(fit))[0].min()) - R.FIT_BAR_R
                key = min(d, fd + .01)
                if best is None or key > best[0]:
                    best = (key, d, fd, s)
            rows.append({"grip_x": h["grip_x"], "gamma": h["gamma"], "sgn": h["sgn"], "min_margin_deg": h["min_margin_deg"],
                         "min_hook_body_mm": h["min_hook_body_mm"], "min_fingers_mm": fing,
                         "tcp_mm": [w["tcp_mm"] for w in h["waypoints"]], "approach": h["waypoints"][0]["approach"],
                         "arm_arm_mm": round(1000 * best[1], 1) if best else None,
                         "steady_to_fitting_mm": round(1000 * best[2], 1) if best else None,
                         "steady": {k: best[3][k] for k in ("where", "az", "margin_deg", "hook_body_mm")} if best else None})
        cell["hang_best"] = sorted(rows, key=lambda r: (min(r["min_fingers_mm"], 8), min(r["arm_arm_mm"] or -1, 25),
                                                        min(r["steady_to_fitting_mm"] or -1, 20), r["min_margin_deg"]), reverse=True)[:5]
        out[side] = cell
    print(json.dumps(out, indent=1, default=float))


if __name__ == "__main__":
    main()
