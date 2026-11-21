"""ikcheck.py [seed=N | cli=N | corners]: per-waypoint static IK error of both arms' demo plans (each pose held, its IK
started from the pose before it) and each plan's interpolated worst (lancet_fingerstick.TiltPlan, the plan the patched
scripted demo plays), and joint6 against its +-2.0944 rad limit (the arms' clearance from each other is
tools/grippers.py's). seed=N is the run seed as the task sees it; cli=N the inspect-robots --seed N (its trial seed); corners the
eight corners of the layout box; stems the sim stems (sim/stems.py) on --seed 0. With no argument: the eight corners,
--seed 0, 2, 4 and the stems. No sim."""
import dataclasses
import math
import pathlib
import sys
import warnings
import zlib

warnings.simplefilter("ignore")
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import lancet_fingerstick as F  # noqa: E402
from yam_arena import station  # noqa: E402

J6 = 2.0944
STEMS = "ojxwfkcagy"               # the sim stems (sim/stems.py, sim/mk.sh)


def stem_plan(stem, seed):
    """A sim stem's (right, left) plan: sim/stems.py run in a copy of the task's namespace (as sim/mk.sh appends it)."""
    src = (pathlib.Path(__file__).resolve().parents[1] / "sim" / "stems.py").read_text()
    ns = dict(vars(F))
    ns["run_seed"] = lambda: seed
    exec(compile(src.replace('_STEM = "none"', f'_STEM = "{stem}"'), "stems.py", "exec"), ns)
    d = ns["demo"]()
    return d["right"], d["left"]


def check(seed, name, stem=None):
    """Print one layout's reach report (the demo's plan, or a sim stem's); return its count of bad waypoints."""
    print(f"== {name}: shift {tuple(round(1000 * v, 1) for v in F.shift_turn(seed)[:2])} mm, turn "
          f"{math.degrees(F.shift_turn(seed)[2]):.1f} deg" + (f", stem {stem}00" if stem else ""))
    plans = dict(zip(("right", "left"), F.plan(0.0, seed) if stem is None else stem_plan(stem, seed)))
    total_bad = 0
    for side, ws in plans.items():
        bad, q0 = [], None
        for w in ws:
            p = F.TiltPlan([dataclasses.replace(w, duration=1.0, label="a"), dataclasses.replace(w, duration=0.5,
                                                                                               label="b")],
                           station.BASES[side], **({} if q0 is None else {"q_start": q0}))
            t, j6 = 0.0, 0.0
            while t <= p.duration:
                q, _ = p.action7(t)
                j6 = max(j6, abs(float(q[5])))
                t += 0.1
            q0 = p.q.copy()
            if p.worst_ik_mm > 0.5 or p.worst_ik_deg > 0.5 or j6 > J6:
                bad.append(f"  {w.label:40s} {p.worst_ik_mm:6.2f} mm {p.worst_ik_deg:6.2f} deg j6 {j6:5.2f} pos "
                           f"{tuple(round(v, 4) for v in w.pos)} yaw {math.degrees(w.yaw):.0f}")
        total_bad += len(bad)
        print(side, len(ws), "waypoints;", len(bad), "bad")
        if bad:
            print("\n".join(bad))
        p = F.TiltPlan(ws, station.BASES[side])
        t, j6 = 0.0, 0.0
        while t <= p.duration:
            q, _ = p.action7(t)
            j6 = max(j6, abs(float(q[5])))
            t += 0.02
        print(f"  interpolated: dur {p.duration:.1f} s worst {p.worst_ik_mm:.2f} mm {p.worst_ik_deg:.2f} deg; max "
              f"|joint6| {j6:.2f} rad (limit {J6})")
        if p.worst_ik_mm > 0.5 or p.worst_ik_deg > 0.5 or j6 > J6:
            total_bad += 1
    return total_bad


def main():
    jobs = []
    for a in sys.argv[1:]:
        if a == "stems":   # the sim stems on --seed 0, as sim/run.sh runs them
            jobs += [(zlib.crc32(b"0:None:0"), f"--seed 0", k) for k in STEMS]
            continue
        if a == "corners":
            jobs += [(c, f"corner {c[0] * 1000:+.0f}, {c[1] * 1000:+.0f} mm, {math.degrees(c[2]):+.0f} deg", None)
                     for c in F.corners()]
        else:
            k, v = a.split("=")
            jobs.append((int(v), f"run seed {v}", None) if k == "seed" else (zlib.crc32(f"{v}:None:0".encode()),
                                                                              f"--seed {v}", None))
    if not jobs:
        jobs = [(c, f"corner {c[0] * 1000:+.0f}, {c[1] * 1000:+.0f} mm, {math.degrees(c[2]):+.0f} deg", None)
                for c in F.corners()] + [(zlib.crc32(f"{c}:None:0".encode()), f"--seed {c}", None) for c in (0, 2, 4)]
        jobs += [(zlib.crc32(b"0:None:0"), "--seed 0", k) for k in STEMS]
    total = sum(check(s, n, k) for s, n, k in jobs)
    print("TOTAL BAD", total)


if __name__ == "__main__":
    main()
