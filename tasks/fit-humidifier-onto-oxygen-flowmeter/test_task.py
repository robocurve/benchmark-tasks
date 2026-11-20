"""Offline checks of the scorer (every way to game it scores low), the thread's geometry and the demo plan.
No simulator: the bottle and its nut are posed directly; arm contact is posed with the arms' own collision hulls at
forward-kinematics poses. Run: python3 test_task.py [--slow] (--slow adds the hull-overlap thread checks)."""
import math
import os
import sys
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import task as M  # noqa: E402
import threadkit as tk  # noqa: E402
import yamkit as kit  # noqa: E402

FAILS = []


def check(ok, msg):
    print(("ok   " if ok else "FAIL ") + msg)
    if not ok:
        FAILS.append(msg)


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def st(pos, R):
    return SimpleNamespace(pos=tuple(float(v) for v in pos), quat_xyzw=tuple(float(v) for v in tk.quat(R)))


def on_outlet(rise, turn_deg=0.0, tilt_deg=0.0, off=0.0, q=0.0):
    """The objects with the nut `rise` m off its seat, turned turn_deg (outlet frame), its axis tilted tilt_deg about
    the outlet's x through the nut's mouth and moved `off` m along y; the bottle hangs under it, its swivel at q."""
    o, Ro = M.outlet_pose()
    Rn = Ro @ rz(math.radians(turn_deg)) @ rx(math.radians(tilt_deg))  # nut frame in the world
    pn = o + Ro @ np.array([0.0, off, tk.SEAT_Z + rise])
    Rl = Rn @ M.R_NUT.T  # the nut link's frame
    Rb = Rl @ rz(-q)  # the bottle's frame: the link is the bottle's turned q about its z
    pb = pn - Rb @ np.array([0.0, 0.0, M.NUT_MOUTH])
    return {"bottle": st(pb, Rb), "bottle/nut": st(pn, Rl)}


def standing(x=M.BOTTLE_START[0], y=M.BOTTLE_START[1], z=M.TOP, R=np.eye(3), q=M.NUT_START):
    pb = np.array([x, y, z])
    return {"bottle": st(pb, R), "bottle/nut": st(pb + R @ np.array([0.0, 0.0, M.NUT_MOUTH]), R @ rz(q))}


class Trial:
    """A scored trial fed poses step by step (each call is one 20 Hz control step)."""

    def __init__(self, gap=0.02):
        kit._EPISODE["count"] = kit._EPISODE.get("count", 0) + 1
        kit._EPISODE["last"] = None
        M._L = None
        self.gap = gap
        self.n = 0

    def step(self, objs, n=1, gap=None):
        g = self.gap if gap is None else gap
        real = M.arm_gap
        M.arm_gap = lambda objects, robots=M.ROBOTS: g
        try:
            s = 0.0
            for _ in range(n):
                self.n += 1
                kit._EPISODE["last"] = self.n
                s = M.score(objs)
        finally:
            M.arm_gap = real
        return s


def test_start_and_nothing():
    t = Trial()
    check(t.step(standing(), 60) == 0.0, "the start state (bottle standing on the table) and doing nothing score 0")
    f = M.nut_fit(standing())
    check(f["rise"] > 0.03, f"at the start the nut is {f['rise'] * 1000:.0f} mm below its seat")


def test_full():
    t = Trial()
    s = t.step(on_outlet(0.0), 25)
    check(s == 1.0, f"seated, hanging, hands clear, still 1 s: 1.0 (got {s})")
    t = Trial()
    check(t.step(on_outlet(0.0), 10) == 0.5, "seated but still for only half a second: 0.5")


def test_held():
    t = Trial(gap=0.0)
    check(t.step(on_outlet(0.0), 60) == 0.5, "seated but a hand still on the bottle or nut: 0.5")
    t = Trial(gap=0.004)
    check(t.step(on_outlet(0.0), 60) == 0.5, "a hand hovering 4 mm off (inside the 5 mm clearance): 0.5")


def test_short():
    for turns in (0.5, 1.0, 2.0):
        t = Trial()
        rise = tk.PITCH * turns
        s = t.step(on_outlet(rise, -360.0 * turns), 40)
        check(s == 0.25 if turns < 2.4 and rise <= M.ON_RISE else s == 0.0,
              f"{turns} turns short of the seat, square: 0.25 at most (got {s})")
    t = Trial()
    check(t.step(on_outlet(M.STOP0 - 0.0005, 0.0), 40) == 0.0, "only just caught (under a turn on): 0")
    t = Trial()
    s = t.step(on_outlet(tk.PITCH * 0.6, 180.0), 40)
    check(s == 0.25, f"pushed up into the thread's play half a turn short (wrong turn, rise 0.85 mm): not seated ({s})")
    t = Trial()
    s = t.step(on_outlet(0.0, 90.0), 40)
    check(s == 0.25, f"at the seat height but a quarter turn off the seated turn: not seated ({s})")


def test_crooked():
    for tilt in (3.5, 6.0):
        t = Trial()
        t.step(on_outlet(M.STOP0 - tk.PITCH / 2, 0.0, tilt), 5)  # turned half a turn onto the thread, crooked
        s = t.step(on_outlet(0.0), 40)
        check(s == 0.0, f"tilted {tilt} degrees over the thread, then seated: cross-threaded, 0 ({s})")
    t = Trial()
    t.step(on_outlet(M.CATCH_RISE + 0.0005, 0.0, 2.5), 5)
    s = t.step(on_outlet(0.0), 40)
    check(s == 1.0, f"2.5 degrees off plumb while lining up under the stub, then seated square: 1.0 ({s})")
    t = Trial()
    t.step(on_outlet(0.02, 0.0, 30.0, off=0.02), 5)
    s = t.step(on_outlet(0.0), 40)
    check(s == 1.0, f"swung 30 degrees while carried well clear of the outlet, then seated: 1.0 ({s})")


def test_dropped_tipped():
    t = Trial()
    t.step(standing(z=M.TOP + 0.05), 1)
    t.step(standing(z=M.TOP + 0.02), 1)  # 0.6 m/s down
    s = t.step(on_outlet(0.0), 40)
    check(s == 0.0, f"dropped (falling 0.6 m/s), then seated: 0 ({s})")
    t = Trial()
    t.step(standing(z=M.TOP + 0.05), 1)
    t.step(standing(z=M.TOP + 0.035), 1)  # 0.3 m/s: lowered, not dropped
    s = t.step(on_outlet(0.0), 40)
    check(s == 1.0, f"lowered at 0.3 m/s, then seated: 1.0 ({s})")
    t = Trial()
    t.step(standing(R=rx(math.radians(M.TIP_DEG + 2.0))), 1)
    s = t.step(on_outlet(0.0), 40)
    check(s == 0.0, f"tipped past {M.TIP_DEG:.0f} degrees (water at the gas outlet), then seated: 0 ({s})")
    t = Trial()
    t.step(standing(R=rx(math.radians(45.0))), 1)
    s = t.step(on_outlet(0.0), 40)
    check(s == 1.0, f"tipped 45 degrees (water below the gas outlet), then seated: 1.0 ({s})")


def test_swinging():
    t = Trial()
    s = 0.0
    for k in range(40):
        o = on_outlet(0.0)
        b = o["bottle"]
        o["bottle"] = SimpleNamespace(pos=(b.pos[0] + 0.002 * (k % 2), b.pos[1], b.pos[2]), quat_xyzw=b.quat_xyzw)
        s = t.step(o, 1)
    check(s == 0.5, f"seated but the bottle still swinging (2 mm a step): 0.5 ({s})")


def test_done_then_undone():
    t = Trial()
    t.step(on_outlet(0.0), 25)
    s = t.step(on_outlet(tk.PITCH * 0.6, -216.0), 40)
    check(s == 0.25, f"seated, then backed off 0.6 turn at the end: 0.25 ({s})")
    t = Trial()
    t.step(on_outlet(0.0), 25)
    s = t.step(on_outlet(tk.PITCH * 2, -720.0), 40)
    check(s == 0.0, f"seated, then backed off two turns (under a turn on the thread) at the end: 0 ({s})")
    t = Trial()
    t.step(on_outlet(0.0), 25)
    s = t.step(standing(), 40)
    check(s == 0.0, f"seated, then unscrewed and stood back on the table: 0 (0.25 is judged at the end) ({s})")


def test_tip_angle():
    d = M._tip_deg()
    check(abs(d - M.TIP_DEG) < 0.5, f"water reaches the gas outlet at {d:.0f} degrees (TIP_DEG {M.TIP_DEG})")
    check(not M.water_reaches_port(60.0), "at 60 degrees the water is still below the gas outlet")


def test_sizes():
    check(abs(tk.R_CREST * 2 - 0.5625 * 0.0254) < 1e-9 and abs(tk.PITCH - 0.0254 / 18) < 1e-12,
          "the outlet thread is 9/16-18 UNF (14.29 mm, 1.411 mm pitch)")
    check(abs(M.NUT_MOUTH - 7.0 * 0.0254) < 0.0005, f"the bottle stands {M.NUT_MOUTH * 1000:.1f} mm tall (7.00 in)")
    check(abs(2 * M.LID_R - 3.25 * 0.0254) < 0.0002, "the lid is 3.25 in across")
    vol = math.pi * (M.JAR_R - M.JAR_WALL) ** 2 * (M.JAR_H - M.JAR_WALL) * 1e6
    check(330 <= vol <= 380, f"the jar holds {vol:.0f} mL to its rim (350 cc bottle)")
    check(abs(M.BOTTLE_KG + M.NUT_KG - 0.380) < 1e-9, "the filled bottle weighs 380 g (90 g dry + 290 g water)")
    check(M.BOTTLE_COM[2] < M.GRIP_Z, "the right's pinch line is above the full bottle's center of mass (hangs plumb)")
    check(2 * M.LID_R < 2 * M.FINGER_HALF_OPEN - 0.01, "the lid fits the open gripper with 5 mm a side")
    n_hulls = len(tk.nut_point_sets())
    check(all(len(p) <= 64 for p in tk.nut_point_sets() + tk.stub_point_sets()), f"every hull has at most 64 points "
          f"({n_hulls} nut hulls)")


def test_demo_plan():
    import inspect

    d = M.demo()
    steps = M._t(d["left"]) / M.DT
    check(steps < M.RUN_LIMIT_STEPS - 100, f"the demo plan takes {steps:.0f} steps (limit {M.RUN_LIMIT_STEPS})")
    check(abs(M._t(d["left"]) - M._t(d["right"])) < 1e-6, "both arms' plans end together")
    src = inspect.getsource(M.demo) + inspect.getsource(M._rows)
    check(not any(w in src for w in ("bottle/nut", "quat_xyzw", "LINK_POSES", "objects", "nut_fit", "seated(")),
          "demo() and its plan read no object or simulator state (fixed waypoints from demoplan.json)")
    real = M.nut_fit
    M.nut_fit = lambda *a, **k: (_ for _ in ()).throw(RuntimeError("demo read the nut"))
    try:
        M.demo()
        ok = True
    except RuntimeError:
        ok = False
    finally:
        M.nut_fit = real
    check(ok, "the plan builds with every object accessor raising")
    right = d["right"]
    opens = [w for w in right if isinstance(w, M.Fixed) and "let go" in w.label and w.finger < -0.04]
    check(len(opens) >= 1, "the right lets go of the bottle (it then hangs from the thread)")
    heads = [M._heading(w.rot) for w in d["left"] if isinstance(w, M.Fixed) and "turn" in w.label]
    check(min(heads) >= -101 and max(heads) <= 11, f"the left's strokes point the hand between {min(heads):.0f} and "
          f"{max(heads):.0f} degrees (its reach: -100 to 10)")
    for v in ("crooked", "short", "held", "nothing"):
        os.environ["O2_VARIANT"] = v
        try:
            dv = M.demo()
        finally:
            del os.environ["O2_VARIANT"]
        if v == "nothing":
            check(all(isinstance(w, kit.Home) for w in dv["left"] + dv["right"]), "the do-nothing variant stays home")
        elif v == "crooked":
            a = [w for w in d["right"] if isinstance(w, M.Fixed) and w.label.startswith("look")][0]
            b = [w for w in dv["right"] if isinstance(w, M.Fixed) and w.label.startswith("look")][0]
            ang = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(np.asarray(a.rot).T @ b.rot) - 1) / 2))))
            check(abs(ang - 6.0) < 0.01, f"the crooked variant holds the bottle 6 degrees off ({ang:.2f})")
        elif v == "short":
            n0 = sum(1 for w in d["left"] if "turn" in w.label)
            n1 = sum(1 for w in dv["left"] if "turn" in w.label)
            check(n1 < n0, f"the short variant stops turning early ({n1} of {n0} turn waypoints)")
        elif v == "held":
            check(not any(isinstance(w, M.Fixed) and w.finger < -0.04 for w in dv["right"][-20:]),
                  "the held variant never opens the right hand at the end")


def test_thread_geometry():
    from scipy.optimize import linprog
    from scipy.spatial import ConvexHull

    stub = [np.asarray(p, float) for p in tk.stub_point_sets()]
    nut = [np.asarray(p, float) for p in tk.nut_point_sets()]
    SE = [ConvexHull(p).equations for p in stub]
    NE = [ConvexHull(p).equations for p in nut]

    def depth(phi, rise, tilt=0.0):
        R = rz(math.radians(phi)) @ rx(math.radians(tilt))
        t = np.array([0.0, 0.0, tk.SEAT_Z + rise])
        worst = -1.0
        for j, pc in enumerate(nut):
            pw = pc @ R.T + t
            n = NE[j][:, :3] @ R.T
            d = NE[j][:, 3] - n @ t
            lo, hi = pw.min(0), pw.max(0)
            for i, pn in enumerate(stub):
                if np.any(pn.min(0) > hi + 1e-4) or np.any(pn.max(0) < lo - 1e-4):
                    continue
                A = np.vstack([SE[i][:, :3], n])
                b = -np.concatenate([SE[i][:, 3], d])
                nn = np.linalg.norm(A, axis=1)
                res = linprog([0, 0, 0, -1], A_ub=np.hstack([A, nn[:, None]]), b_ub=b,
                              bounds=[(None, None)] * 3 + [(None, 1)], method="highs")
                if res.status == 0:
                    worst = max(worst, float(res.x[3]))
        return worst

    check(depth(0.0, 0.0) <= 1e-6, "seated: the nut touches the outlet only at its seat (no overlap)")
    deep = max(depth(ph, tk.PITCH * ph / 360.0) for ph in (60, 180, 300, 420, 540, 800))  # rise = P x turn / 360
    check(deep <= 1e-6, f"turned along the helix the ridges never cut each other ({deep * 1000:.3f} mm)")
    check(depth(0.0, 0.0005) > 1e-5 and depth(0.0, 0.001) > 1e-5, "pushed straight up without turning: blocked")
    check(depth(0.0, -0.0003) > 1e-5, "pushed past its seat: the stub's end blocks it")
    r = 0.006
    while r > 0 and depth(0.0, r) <= 1e-6:
        r -= 0.0001
    check(abs(r - M.STOP0) <= 0.00011, f"pushed straight up at its seated turn the nut stops {r * 1000:.2f} mm off its "
          f"seat (STOP0 {M.STOP0 * 1000:.2f})")
    for tilt in (3.0, 6.0):
        d = min(depth(ph, 0.0, tilt) for ph in (0.0, 90.0, 180.0, 270.0))
        check(d > 1e-5, f"a nut tilted {tilt:.0f} degrees cannot reach its seat ({d * 1000:.2f} mm)")


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_") and (name != "test_thread_geometry" or "--slow" in sys.argv):
            fn()
    print(f"{len(FAILS)} failed" if FAILS else "all passed")
    sys.exit(1 if FAILS else 0)
