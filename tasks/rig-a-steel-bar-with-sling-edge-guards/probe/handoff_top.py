"""Gate check: the left hand lays the left fitting flat on the bar's top face under the hook (hook centred, throat -y),
base bar along x at y = Y, apex toward -y; the right hand then takes it by the base bar's near end. Hands hold the base
bar's ends (left x = 0.276, right x = 0.224), approach in the y-z plane.  python handoff_top.py > out/handoff_top.json"""
import itertools, json, math, sys
import numpy as np
import ik_scan as S
from plan_check import handover_job, init


def main():
    from multiprocessing import Pool
    init()
    ys, zs = (-.01, -.005, 0., .005, .01, .02), (.148, .156, .165)
    jobs = [(0., y, z, arm, ang, sgn) for y in ys for z in zs for arm in ("left", "right")
            for ang in (range(-180, -90, 10) if arm == "left" else range(-90, 1, 10)) for sgn in (1, -1)]
    with Pool(26, initializer=init) as pool:
        rows = pool.map(handover_job, jobs, chunksize=2)
    out = []
    for y, z in itertools.product(ys, zs):
        cell = {"y": y, "z": z}
        for arm in ("left", "right"):
            c = [r for r in rows if r["y"] == y and r["z"] == z and r["arm"] == arm and r["ok"]]
            c.sort(key=lambda r: (min(r["hook_body_mm"], 20) + min(r["bar_body_mm"], 15), r["margin_deg"]), reverse=True)
            cell[arm] = {k: c[0][k] for k in ("ang", "margin_deg", "hook_body_mm", "bar_body_mm", "bar_fingers_mm", "hook_fingers_mm", "tcp_mm", "q_deg")} if c else None
            cell["n_" + arm] = len(c)
        out.append(cell)
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
