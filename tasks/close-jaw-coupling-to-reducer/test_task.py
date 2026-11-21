"""Offline checks of the jaw-coupling task (no simulator): the scorer's tiers and cheat routes on hand-built poses with
real arm-link poses from forward kinematics, the spawn gaps, the meshing geometry and the demo's plan. Run:
    python test_task.py"""
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from yam_arena.task_file import ObjectState, load  # noqa: E402

M = load(os.path.join(HERE, "task.py")).module
kit = M.kit
FAILS = []
_CACHE = {}


def check(ok, what):
    print(("ok   " if ok else "FAIL ") + what)
    if not ok:
        FAILS.append(what)


def quat(R):
    R = np.asarray(R, float)
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return np.array([x, y, z, w])


def state(pos, R=np.eye(3)):
    z = np.zeros(3)
    return ObjectState(np.asarray(pos, float), quat(R), z, z, z, z)


# -- arm-link poses --------------------------------------------------------------------------------------------------------
def _fk(robot, q6, finger):
    import yam_arena.demo as demo
    from yam_arena.analysis import fk_link_poses_world

    base = demo.LEFT_BASE_POS if robot == "left_robot" else demo.RIGHT_BASE_POS
    q = {f"joint{i + 1}": float(v) for i, v in enumerate(q6)}
    q["joint7"] = q["joint8"] = float(finger)
    return {f"{robot}/{k}": (P[:3, 3].copy(), P[:3, :3].copy()) for k, P in fk_link_poses_world(q, base).items()
            if k in M.ARM_LINKS}


def _ik(robot, pos, rot):
    import yam_arena.demo as demo

    base = demo.LEFT_BASE_POS if robot == "left_robot" else demo.RIGHT_BASE_POS
    best = None
    for seed in (np.array(demo.READY, float), np.array([0.3, 1.6, 1.4, -1.0, 0.0, 0.5])):
        q, pe, _ = demo.ik.solve(np.asarray(pos, float), demo.ik.matrix_to_quat_xyzw(rot), seed, base_pos=base,
                                 iters=400)
        if best is None or pe < best[1]:
            best = (q, pe)
    check(best[1] < 0.002, f"IK reaches the {robot} pose ({best[1] * 1000:.1f} mm)")
    return best[0]


def ready_links():
    if "ready" not in _CACHE:
        import yam_arena.demo as demo

        out = {}
        for r in M.ROBOTS:
            out.update(_fk(r, demo.READY, M.FINGER_OPEN))
        _CACHE["ready"] = out
    return _CACHE["ready"]


def right_at(pos, tilt_deg, finger):
    """The right hand at tool point `pos`, fingers closing along x, tilted about y; the left at the ready pose."""
    import yam_arena.demo as demo

    key = (tuple(np.round(pos, 5)), tilt_deg, round(finger, 6))
    if key not in _CACHE:
        rot = kit._rot_xyz(0.0, math.radians(tilt_deg)) @ demo.grip_rotation(M.YAW_X)
        q = _ik("right_robot", pos, rot)
        out = _fk("right_robot", q, finger)
        out.update({k: v for k, v in ready_links().items() if k.startswith("left_robot")})
        _CACHE[key] = out
    return _CACHE[key]


def right_clear_of(part, xy, top_z, clearance, finger, trial):
    """The right hand straight down over `xy` with its lowest hull `clearance` m from `part` (found by moving the tool
    point along z), for the trial's part poses."""
    z = top_z + 0.013 + clearance
    for _ in range(4):
        links = right_at((xy[0], xy[1], round(z, 5)), 0.0, finger)
        M.LINK_POSES.clear()
        M.LINK_POSES.update(links)
        g = M.arm_gaps(trial.objects())[part]
        if abs(g - clearance) < 0.0003:
            break
        z += clearance - g
    return links


# -- a trial driven by hand ------------------------------------------------------------------------------------------------
class Trial:
    def __init__(self):
        kit._EPISODE["count"] = kit._EPISODE.get("count", 0) + 1
        kit._EPISODE["last"] = 0
        M._L = None
        self.gap = M.GAP_START
        self.turn = M.START_TURN_DEG
        self.spider_pos = np.asarray(M.SPIDER_START, float) + np.array([0.0, M.SPAWN_GAP, -0.0002])
        self.spider_R = np.eye(3)
        self.links = ready_links()

    def objects(self):
        R = M.ry(self.turn)
        return {"reducer": state(M.REDUCER.pos), "drive": state(M.DRIVE.pos),
                "drive/shaft": state(M.DRIVE.pos, R),
                "drive/hub": state((M.XC, M.YA - self.gap, M.AXIS_Z), R),
                "spider": state(self.spider_pos, self.spider_R)}

    def run(self, n=1):
        s = None
        for _ in range(n):
            kit._EPISODE["last"] += 1
            M.LINK_POSES.clear()
            M.LINK_POSES.update(self.links)
            o = self.objects()
            M.success(o)
            s = M.score(o)
        return s

    def closed(self):
        """The hub slid in to the stop in phase, the spider pushed home on the reducer hub's face."""
        self.gap, self.turn = M.SPIDER_L + 0.0001, 0.0
        self.spider_pos = np.array([M.XC, M.YA, M.AXIS_Z - 0.0002])


def t_start():
    t = Trial()
    check(all(t.run() == 0.0 for _ in range(40)), "do nothing: the start state scores 0 at every step")
    c = M._L.c
    check(c["seated"], "the spider starts seated on the reducer hub")
    check(c["depth"] < -0.008, f"the driven hub's jaws start {-c['depth'] * 1000:.1f} mm clear of the spider")
    check(abs(c["phase"]) > 25, f"the driven hub starts out of phase ({c['phase']:.0f} degrees)")


def t_full():
    t = Trial()
    t.run(5)
    t.closed()
    s19 = t.run(M.HOLD_STEPS - 1)
    s = t.run(2)
    check(s19 == 0.0 and s == 1.0, f"closed, in phase, spider seated, hands clear: 0 before the 1 s hold, then 1.0 "
                                   f"({s19}, {s})")
    for g in (M.LINE_HI - 0.0002, M.GAP):
        t2 = Trial()
        t2.gap, t2.turn = g, 0.0
        check(t2.run(M.HOLD_STEPS + 2) == 1.0, f"a face gap of {g * 1000:.2f} mm (inside G +- delta-G) scores 1.0")
    t3 = Trial()
    t3.gap, t3.turn = M.SPIDER_L + 0.0001, 120.0
    check(t3.run(M.HOLD_STEPS + 2) == 1.0, "the next phase (120 degrees on) meshes too: 1.0")
    t4 = Trial()
    t4.gap, t4.turn = M.SPIDER_L + 0.0001, 4.0
    check(t4.run(M.HOLD_STEPS + 2) == 1.0, "4 degrees off the pockets' middle (inside the play): 1.0")


def t_partial():
    for depth, want in ((0.009, 0.5), (0.003, 0.25), (0.0005, 0.0)):
        t = Trial()
        t.gap, t.turn = M.MESH_MIN - depth, 0.0
        s = t.run(M.HOLD_STEPS + 5)
        check(s == want, f"in phase, jaws {depth * 1000:.1f} mm into the pockets, let go: {want} ({s})")
    t = Trial()
    t.gap, t.turn = M.LINE_HI + 0.0005, 0.0
    check(t.run(M.HOLD_STEPS + 5) == 0.5, "meshed but 0.5 mm past the line's far side: 0.5")
    t = Trial()
    t.gap, t.turn = M.MESH_MIN - 0.009, 0.0
    s = t.run(3)
    check(s == 0.0, f"meshed 9 mm in but only for 3 steps (not held 1 s): 0 ({s})")
    out = []
    for k in range(M.HOLD_STEPS + 10):
        t.gap = M.MESH_MIN - 0.009 + (0.0007 if k % 2 else 0.0)
        out.append(t.run())
    check(max(out) == 0.0, "meshed 9 mm in but the hub still shuttling (a flick at the end): 0")


def t_noturn():
    t = Trial()
    t.gap = M.MESH_MIN + 0.0002
    check(t.run(M.HOLD_STEPS + 5) == 0.0, "pushed jaw-on-leg without turning (stopped on the legs): 0")
    t = Trial()
    t.gap, t.turn = M.SPIDER_L + 0.0001, 30.0
    check(t.run(M.HOLD_STEPS + 5) == 0.0, "at the line but out of phase (only by driving through the legs): 0")
    t = Trial()
    t.gap, t.turn = M.FORCED - 0.0005, 0.0
    check(t.run(M.HOLD_STEPS + 5) == 0.0, "face gap under the spider's length (driven into it): 0")


def t_hands():
    grip = M.hub_tcp(M.SPIDER_L + 0.0001)
    hold = M.finger_for(M.R_HUB - M.HUB_SQUEEZE)
    t = Trial()
    t.closed()
    t.links = right_at(grip, -30.0, hold)
    check(t.run(M.HOLD_STEPS + 5) == 0.0, "closed but the right still holding the hub: 0")
    check(M._L.gaps["hub"] <= M.TOUCH, f"the gripping pads touch the hub ({M._L.gaps['hub'] * 1000:.2f} mm)")
    for part, xy, top, c, want in (("hub", grip, M.R_HUB, 0.003, 0.0), ("hub", grip, M.R_HUB, 0.008, 1.0),
                                   ("spider", (M.XC, M.YA - 0.009), M.SPIDER_R, 0.0005, 0.0),
                                   ("shaft", (M.XC, -0.075), M.SHAFT_R, 0.0005, 0.0)):
        t = Trial()
        t.closed()
        t.links = right_clear_of(part, xy, M.AXIS_Z + top, c, M.FINGER_CLOSED, t)
        s = t.run(M.HOLD_STEPS + 5)
        check(s == want, f"closed, a closed hand {c * 1000:.1f} mm over the {part}: {want} ({s}, measured "
                         f"{M._L.gaps[part] * 1000:.1f} mm)")


def t_spider():
    t = Trial()
    t.closed()
    t.spider_pos = np.array([M.XC + 0.08, M.YA - 0.03, M.TOP + M.PLATE_T + 0.004])
    t.spider_R = np.array([[1.0, 0, 0], [0, 0, -1.0], [0, 1.0, 0]])
    check(t.run(M.HOLD_STEPS + 5) == 0.0, "closed without the spider (hub jaws among the reducer hub's): 0")
    check(M._L.knocked, "the spider lying on the base plate with no hand on it: knocked off (latched)")
    t.spider_pos = np.asarray(M.SPIDER_START, float)
    t.spider_R = np.eye(3)
    check(t.run(M.HOLD_STEPS + 5) == 0.0, "the spider back on afterwards: still 0 (the knock-off is latched)")
    t = Trial()
    t.closed()
    t.spider_pos = t.spider_pos - np.array([0.0, 0.003, 0.0])
    check(t.run(M.HOLD_STEPS + 5) == 0.0, "the spider pulled 3 mm off the reducer hub's face: not seated, 0")
    t = Trial()
    t.closed()
    t.spider_R = M.ry(30.0)
    check(t.run(M.HOLD_STEPS + 5) == 0.0, "the spider turned 30 degrees (legs on the reducer hub's jaws): 0")
    t = Trial()
    t.closed()
    a = math.radians(8.0)
    t.spider_R = np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])
    check(t.run(M.HOLD_STEPS + 5) == 0.0, "the spider cocked 8 degrees: 0")


def t_undone_and_moving():
    t = Trial()
    t.closed()
    check(t.run(M.HOLD_STEPS + 5) == 1.0, "closed and held 1 s: 1.0 ...")
    t.gap = M.MESH_MIN + 0.004
    check(t.run(M.HOLD_STEPS + 5) == 0.0, "... then pulled back out of the spider: 0 (the end state counts)")
    t = Trial()
    t.closed()
    out = []
    for k in range(M.HOLD_STEPS + 10):
        t.gap = M.SPIDER_L + 0.0001 + (0.0007 if k % 2 else 0.0)
        out.append(t.run())
    check(max(out) <= 0.5, "closed but the hub still shuttling (over 1 cm/s): no 1.0")
    t = Trial()
    t.closed()
    out = []
    for k in range(M.HOLD_STEPS + 10):
        t.turn = 1.0 if k % 2 else 0.0
        out.append(t.run())
    check(max(out) <= 0.5, "closed but the hub still rocking 1 degree a step: no 1.0")


# -- geometry ---------------------------------------------------------------------------------------------------------------
def _hulls(cols, pos, R=np.eye(3)):
    from scipy.spatial import ConvexHull
    from yam_arena import analysis as A

    out = []
    for c in cols:
        if c.hull is not None:
            p = np.asarray(c.center) + np.asarray(c.hull)
        elif c.size is not None:
            s = np.asarray(c.size) / 2
            p = np.asarray(c.center) + np.array([[a, b, d] for a in (-1, 1) for b in (-1, 1) for d in (-1, 1)]) * s
        else:
            a = np.linspace(0, 2 * np.pi, 32, endpoint=False)
            ax = "XYZ".index(c.axis.upper())
            pts = []
            for t in a:
                for z in (-c.height / 2, c.height / 2):
                    v = [c.radius * np.cos(t), c.radius * np.sin(t)]
                    v.insert(ax, z)
                    pts.append(v)
            p = np.asarray(c.center) + np.array(pts)
        p = p @ np.asarray(R).T + np.asarray(pos)
        h = ConvexHull(p)
        out.append(("c", A.Hull(vertices=p, planes=np.c_[h.equations[:, :3], -h.equations[:, 3]])))
    return out


def _dist(a, b):
    from yam_arena import analysis as A

    return A.hull_set_distance(a, b)[0]


def t_geometry():
    spider = _hulls(M.SPIDER.colliders, M.SPIDER_START)
    red = _hulls(M.REDUCER.colliders, M.REDUCER.pos)
    hub0 = _hulls(M.HUB.colliders, (M.XC, M.YB0, M.AXIS_Z), M.ry(M.START_TURN_DEG))
    blocks = _hulls(M.DRIVE.colliders, M.DRIVE.pos)
    shaft = _hulls(M.SHAFT.colliders, M.DRIVE.pos)
    d = _dist(spider, red)
    check(d > 0.0001, f"spawn: the spider clears the reducer side by {d * 1000:.2f} mm (no overlap at spawn)")
    d = _dist(spider, hub0)
    check(d > 0.005, f"spawn: the spider clears the driven hub by {d * 1000:.1f} mm")
    d = _dist(hub0, red)
    check(d > 0.008, f"spawn: the driven hub's jaws clear the reducer hub's jaws by {d * 1000:.1f} mm")
    d = _dist(hub0, blocks)
    check(d > 0.015, f"spawn: the driven hub clears the pillow blocks by {d * 1000:.1f} mm")
    d = _dist(shaft, red)
    check(d > 0.001, f"spawn: the driven shaft clears the reducer hub by {d * 1000:.1f} mm")
    d = _dist(shaft, spider)
    check(d > 0.001, f"spawn: the driven shaft's end clears the spider by {d * 1000:.1f} mm")
    # closed at the stop, in phase: the driven hub's jaws clear the spider's legs and the reducer hub's jaws
    spider = _hulls(M.SPIDER.colliders, (M.XC, M.YA, M.AXIS_Z))  # pushed home on the reducer hub's face
    hub_c = _hulls(M.HUB.colliders, (M.XC, M.YA - M.SPIDER_L - 0.0001, M.AXIS_Z), M.ry(0.0))
    d1, d2 = _dist(hub_c, spider), _dist(hub_c, red)
    check(d1 > 0.0 and d1 < 0.001 and d2 > 0.0001, f"closed in phase: the driven hub {d1 * 1000:.2f} mm from the "
                                                   f"spider (face on the legs' ends), {d2 * 1000:.2f} mm from the "
                                                   f"reducer side")
    # 10 mm in, in phase: the jaws sit in the pockets with play either side
    hub_in = _hulls(M.HUB.colliders, (M.XC, M.YA - M.MESH_MIN + 0.010, M.AXIS_Z), M.ry(0.0))
    d = _dist(hub_in, spider)
    check(0.0001 < d < 0.002, f"10 mm in, in phase, all centred: {d * 1000:.2f} mm clear between the jaws and the "
                              f"spider (its solid center 0.3 mm inside the jaws' inner radius)")
    for turn in (2.5, -2.5):
        h = _hulls(M.HUB.colliders, (M.XC, M.YA - M.MESH_MIN + 0.010, M.AXIS_Z), M.ry(turn))
        check(_dist(h, spider) > 0.0, f"10 mm in, {turn} degrees off: still clear (the play)")
    for turn in (M.START_TURN_DEG, 12.0, -12.0, 60.0):
        h = _hulls(M.HUB.colliders, (M.XC, M.YA - M.MESH_MIN + 0.002, M.AXIS_Z), M.ry(turn))
        check(_dist(h, spider + red) <= 1e-6, f"{turn:.0f} degrees off phase the jaws cannot enter 2 mm "
                                                   f"(they overlap a leg or a reducer jaw)")
    # the entry window with the spider settled under gravity (resting on the reducer hub's jaws) and turned anywhere in
    # +-2.5 degrees: turning from the start toward the demo's final turn, the driven jaws pass a phase where they enter
    # 3 mm with radial and angular clearance, at least 0.5 degrees before the turn ends
    final = M.START_TURN_DEG + M.TURN_DEG
    for rot in np.arange(-2.5, 2.51, 0.5):
        lo, hi = 0.0, 0.003
        for _ in range(16):
            m = (lo + hi) / 2
            s = _hulls(M.SPIDER.colliders, (M.XC, M.YA - M.SPAWN_GAP, M.AXIS_Z - m), M.ry(rot))
            lo, hi = (m, hi) if _dist(s, red) > 1e-7 else (lo, m)
        s = _hulls(M.SPIDER.colliders, (M.XC, M.YA - M.SPAWN_GAP, M.AXIS_Z - lo), M.ry(rot))
        ok = [t for t in np.arange(8.0, final - 1e-9, -0.25)
              if _dist(_hulls(M.HUB.colliders, (M.XC, M.YA - M.MESH_MIN + 0.003, M.AXIS_Z), M.ry(t)), s) > 0.00005]
        first = max(ok) if ok else None
        check(first is not None and first - final >= 0.5 and lo < 0.0005,
              f"spider settled {lo * 1000:.2f} mm, turned {rot:+.1f} deg: the jaws enter (0.05 mm clear) at "
              f"{first} deg, before the demo's final {final:.1f}")
    # the spider's legs sit either side of the reducer hub's jaws
    for turn in (2.5, -2.5):
        s = _hulls(M.SPIDER.colliders, M.SPIDER_START, M.ry(turn))
        check(_dist(s, red) > 0.0, f"the spider turned {turn} degrees still clears the reducer hub's jaws")
    # sizes
    check(abs(M.GAP - 0.01905) < 1e-5 and abs(M.GAP_TOL - 0.001016) < 1e-6, "G = 0.750 in, delta-G = 0.040 in")
    check(abs(2 * M.LTB + M.SPIDER_L - M.OAL) < 1e-9, "closed: two lengths through bore and the spider make the 3.480 "
                                                     "in overall length")
    check(M.LINE_LO <= M.SPIDER_L <= M.LINE_HI, f"the spider's stop ({M.SPIDER_L * 1000:.2f} mm) is inside G +- "
                                                f"delta-G")


def t_demo_and_text():
    import yam_arena.demo as demo

    for v in M.VARIANTS:
        os.environ["JAW_VARIANT"] = v
        plan = M.demo()
        steps = max(sum(w.duration for w in s) for s in plan.values()) / M.DT
        check(steps < M.RUN_LIMIT_STEPS, f"demo {v or 'full'}: {steps:.0f} steps, inside the run limit")
        for side, wps in plan.items():
            base = demo.LEFT_BASE_POS if side == "left" else demo.RIGHT_BASE_POS
            p = demo.ArmPlan(wps, base)
            worst = 0.0
            for k in range(int(p.duration / M.DT)):
                pos, _, _, _ = p.target(k * M.DT)
                a, _ = p.action7(k * M.DT)
                q, _ = demo.ik.tcp_pose(a[:6], base)
                worst = max(worst, float(np.linalg.norm(q - pos)))
            check(worst < 0.003, f"demo {v or 'full'} {side}: every waypoint reached by the IK ({worst * 1000:.1f} mm)")
    os.environ["JAW_VARIANT"] = ""
    src = open(os.path.join(HERE, "task.py")).read()
    check("objects[" not in src.split("def demo():")[1], "the demo never reads the sim's object poses (open loop)")
    check("pxr" not in src and "import isaac" not in src, "task.py imports no pxr/Isaac")
    tj = json.load(open(os.path.join(HERE, "task.json")))
    check(tj["instruction"] == M.INSTRUCTION, "task.json instruction == task.py INSTRUCTION")


if __name__ == "__main__":
    for f in (t_start, t_full, t_partial, t_noturn, t_hands, t_spider, t_undone_and_moving, t_geometry,
              t_demo_and_text):
        print("--", f.__name__)
        f()
    print(f"{len(FAILS)} failed")
    sys.exit(1 if FAILS else 0)
