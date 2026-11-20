"""Offline checks of the medicine-cap task, no simulator. Bottle and cap states are built by hand from the same geometry
the demo uses; the arms' links are posed by the robot's forward kinematics (at the ready pose, or at the demo's grip on
the bottle) and checked with their own collision hulls.

Covers: the thread (seated without overlap, free along its helix, the loose start resting on the neck's ridge, a straight
pull blocked once engaged, pushed past the seat blocked, crooked caps cannot seat); the relock reading (thresholds, a
turn short, pushed into the thread's play); doing nothing; the ladder (stored loose, latched; relocked on the counter;
relocked and stored; a hand still on it; not still); storing checks on both sides (in front of the wall, half in, under
the floor, hovering, on a shelf, outside the sides); spills, knock-overs and drops; a new trial; the demo plan's reach
and the hands' clearance from each other; the text's thresholds.

    python3 test_task.py
"""
import math
import re
import sys
from pathlib import Path

import numpy as np
from yam_arena.task_file import ObjectState, load

HERE = Path(__file__).parent
M = load(HERE / "task.py").module
import threadkit as ck  # noqa: E402
import yamkit as kit  # noqa: E402


def check(cond, what):
    print(("ok   " if cond else "FAIL ") + what)
    if not cond:
        raise AssertionError(what)


def st(pos, rot=np.eye(3)):
    p = np.asarray(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.asarray(ck.quat(np.asarray(rot)), float), lin_vel=np.zeros(3),
                       ang_vel=np.zeros(3), low=p - 0.01, high=p + 0.01)


def rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


_CACHE = {}


# -- the arms' link poses (forward kinematics), as the simulator reports them -------------------------------------------
def _fk_poses(robot, q6, finger):
    import yam_arena.demo as demo
    from yam_arena.analysis import fk_link_poses_world

    base = demo.LEFT_BASE_POS if robot == "left_robot" else demo.RIGHT_BASE_POS
    q = {f"joint{i + 1}": float(q6[i]) for i in range(6)}
    q["joint7"] = q["joint8"] = float(finger)
    poses = fk_link_poses_world(q, base)
    return {f"{robot}/{k}": (P[:3, 3].copy(), P[:3, :3].copy()) for k, P in poses.items() if k in M.ARM_LINKS}


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
            out.update(_fk_poses(r, demo.READY, M.FINGER_OPEN))
        _CACHE["ready"] = out
    return _CACHE["ready"]


R_FWD = np.column_stack([[0.0, 0.0, 1.0], [0.0, -1.0, 0.0], [1.0, 0.0, 0.0]])  # tool z forward; built below


def fwd_rot():
    """The left's rotation pointing forward, as kit.Tilted builds it (pitch -90 degrees from the downward grip)."""
    import yam_arena.demo as demo

    return kit._rot_xyz(0.0, M.PITCH_FWD) @ demo.grip_rotation(0.0)


def grip_links(bottle_pos, finger):
    """The left hand on the standing bottle as the demo grips it, at finger command `finger`; the right at the ready
    pose."""
    key = ("grip", tuple(np.round(bottle_pos, 5)), round(finger, 6))
    if key not in _CACHE:
        tcp = np.asarray(bottle_pos, float) + np.array([0.0, 0.0, M.GRIP_H])
        q = _ik("left_robot", tcp, fwd_rot())
        out = _fk_poses("left_robot", q, finger)
        out.update({k: v for k, v in ready_links().items() if k.startswith("right_robot")})
        _CACHE[key] = out
    return _CACHE[key]


# -- a trial driven by hand ------------------------------------------------------------------------------------------------
class Trial:
    def __init__(self):
        kit._EPISODE["count"] = kit._EPISODE.get("count", 0) + 1
        kit._EPISODE["last"] = 0
        M._L = None
        self.n = 0
        self.bottle_pos = np.asarray(M.BOTTLE_START, float) - np.array([0, 0, M.SPAWN_GAP])
        self.bottle_R = np.eye(3)
        self.cap_pos = np.asarray(M.CAP_START, float) - np.array([0, 0, M.SPAWN_GAP])  # top up on the counter
        self.cap_R = M._CAP_R
        self.links = ready_links()

    def cap(self, turn_deg, extra_rise=0.0, tilt_deg=0.0, off=None):
        o, R = M.seated_cap_in_bottle(turn_deg)
        o = o + np.array([0, 0, extra_rise])
        R = R @ rx(math.radians(tilt_deg))
        self.cap_pos = self.bottle_pos + self.bottle_R @ o
        self.cap_R = self.bottle_R @ R
        if off is not None:  # the cap lying apart from the bottle
            self.cap_pos, self.cap_R = np.asarray(off, float), np.eye(3)

    def move(self, pos, R=np.eye(3)):
        """Move the bottle, carrying the cap with it on the neck."""
        rel_p = self.bottle_R.T @ (self.cap_pos - self.bottle_pos)
        rel_R = self.bottle_R.T @ self.cap_R
        self.bottle_pos, self.bottle_R = np.asarray(pos, float), np.asarray(R, float)
        self.cap_pos, self.cap_R = self.bottle_pos + self.bottle_R @ rel_p, self.bottle_R @ rel_R

    def objects(self):
        return {"bottle": st(self.bottle_pos, self.bottle_R), "cap": st(self.cap_pos, self.cap_R),
                "cup": st((M.CUP_XY[0], M.CUP_XY[1], M.TOP)), "cabinet": st((M.X_WALL, M.CAB_Y, M.TOP))}

    def step(self, n=1, links=None):
        M.LINK_POSES.clear()
        M.LINK_POSES.update(self.links if links is None else links)
        for _ in range(n):
            self.n += 1
            kit._EPISODE["last"] = self.n
            M.success(self.objects())
        return M.score(self.objects())


STORE = np.array([M.STORE_XY[0], M.STORE_XY[1], M.TOP + M.FLOOR_Z])


def stored(t, turn=None):
    """Hold the bottle at the store spot (a hand on it), then let go and stand clear."""
    if turn is not None:
        t.cap(turn)
    t.move(STORE + np.array([0, 0, 0.0005]))
    grip = grip_links(t.bottle_pos, M.finger_for(M.BODY_R - M.BODY_SQUEEZE))
    t.step(3, links=grip)
    return t.step(30)


# -- the thread -----------------------------------------------------------------------------------------------------------
def _overlap_depth(phi_deg, rise, off=(0.0, 0.0), tilt_deg=0.0):
    """Deepest overlap (m; <= 0: none) between any cap hull and any neck hull, the cap turned phi and risen `rise` off
    its seat (neck frame), by linear programming on the hulls' face planes (the largest ball inside both)."""
    from scipy.optimize import linprog
    from scipy.spatial import ConvexHull

    if "hulls" not in _CACHE:
        neck = [np.asarray(p, float) for p in ck.neck_point_sets(M.BODY_R)]
        cap = [np.asarray(p, float) for p in ck.cap_point_sets()]
        _CACHE["hulls"] = (neck, [ConvexHull(p).equations for p in neck], cap, [ConvexHull(p).equations for p in cap])
    neck, NE, cap, CE = _CACHE["hulls"]
    R = rz(math.radians(phi_deg)) @ rx(math.radians(tilt_deg))
    t = np.array([off[0], off[1], ck.SEAT_Z + rise])
    worst = -1.0
    for j, pc in enumerate(cap):
        pw = pc @ R.T + t
        n = CE[j][:, :3] @ R.T
        d = CE[j][:, 3] - n @ t
        lo, hi = pw.min(0), pw.max(0)
        for i, pn in enumerate(neck):
            if np.any(pn.min(0) > hi + 1e-4) or np.any(pn.max(0) < lo - 1e-4):
                continue
            A = np.vstack([NE[i][:, :3], n])
            b = -np.concatenate([NE[i][:, 3], d])
            nn = np.linalg.norm(A, axis=1)
            res = linprog([0, 0, 0, -1], A_ub=np.hstack([A, nn[:, None]]), b_ub=b,
                          bounds=[(None, None)] * 3 + [(None, 1)], method="highs")
            if res.status == 0:
                worst = max(worst, float(res.x[3]))
    return worst


def test_thread():
    P = ck.PITCH
    check(_overlap_depth(0.0, 0.0) <= 1e-6, "seated: the cap touches the neck only at its seat (no overlap)")
    deep = max(_overlap_depth(ph, P * ph / 360.0) for ph in (30, 120, 240, 360, 420, 480))
    check(deep <= 1e-6, "turned along the helix, the cap's ridge never cuts the neck's")
    for rise in (0.0005, 0.001):
        check(_overlap_depth(0.0, rise) > 1e-5, f"pulled {rise * 1000:.1f} mm straight up off its seat without "
              "turning: the ridges block it")
    check(_overlap_depth(0.0, -0.0003) > 1e-5, "pushed past its seat: the neck's lip blocks it")
    lo = M.LOOSE_TURN_DEG
    check(_overlap_depth(lo, P * lo / 360.0 - 0.0005) > 1e-5,
          "the loose start rests on the neck's ridge: pushed straight down 0.5 mm it cuts the ridge (it cannot drop "
          "to its seat without turning)")
    free = max(_overlap_depth(lo, P * lo / 360.0 + dz) for dz in np.linspace(0.0003, 0.008, 12))
    check(free <= 1e-6, "the loose start lifts straight off (nothing above it holds it)")
    check(_overlap_depth(240.0, P * 240 / 360.0 + 0.001) > 1e-5,
          "once engaged (2/3 of a turn up), a straight pull is blocked")
    for tilt in (4.0, 8.0):
        deep = min(_overlap_depth(ph, 0.0, tilt_deg=tilt) for ph in (0.0, 90.0, 180.0, 270.0))
        check(deep > 1e-5, f"a cap tilted {tilt:.0f} degrees cannot reach its seat at any turn ({deep * 1000:.2f} mm)")
    play = [dz for dz in np.linspace(-0.0006, 0.0006, 25) if _overlap_depth(180.0, P * 0.5 + dz) <= 1e-6]
    check(max(play) - min(play) < 0.0006, f"axial play on the thread {1000 * (max(play) - min(play)):.2f} mm "
          f"(under the relock check's {1000 * M.SEAT_RISE:.2f} mm rise and its turn check)")


# -- the relock reading ------------------------------------------------------------------------------------------------------
def test_relock_reading():
    t = Trial()
    o = t.objects()
    check(not M.relocked(o), "the start: the loose cap is not relocked")
    for turn, want in ((0.0, True), (20.0, True), (-5.0, True), (45.0, False), (90.0, False), (360.0, False),
                       (M.LOOSE_TURN_DEG, False)):
        t.cap(turn)
        check(M.relocked(t.objects()) == want, f"cap {turn:.0f} degrees up the thread: relocked = {want}")
    t.cap(60.0, extra_rise=-0.0003)
    check(not M.relocked(t.objects()), "60 degrees short, pushed down into the play: not relocked")
    t.cap(0.0, tilt_deg=4.0)
    check(not M.relocked(t.objects()), "crooked 4 degrees: not relocked (a cap that crooked cannot sit on its seat)")


# -- the ladder ---------------------------------------------------------------------------------------------------------------
def test_nothing():
    t = Trial()
    check(t.step(40) == 0.0, "doing nothing: 0 (the cap starts loose, the bottle on the counter)")


def test_stored_loose():
    check(M.STORED_LOOSE == 0.0, "storing the bottle open or with its cap loose scores 0, the same as doing nothing")
    t = Trial()
    s = stored(t)
    check(s == 0.0, f"put away as it stands, cap loose: {s} (0)")
    t.cap(0.0)  # relocked afterwards, in the cabinet
    s = t.step(4 * M.HOLD_STEPS)
    check(s == 0.0 and not M.success(t.objects()), f"relocked only after it was stored, left still: still {s} (latched)")
    t2 = Trial()
    t2.cap(0.0, off=(0.30, -0.20, M.TOP + 0.001))
    s = stored(t2)
    check(s == 0.0, f"stored with the cap off, beside it on the counter: {s}")
    t4 = Trial()
    s = stored(t4, turn=M.LOOSE_TURN_DEG)
    check(s == 0.0, f"cap set on the neck (resting on the thread, not turned), then stored: {s}")
    t3 = Trial()
    s = stored(t3, turn=60.0)
    check(s == 0.0, f"stored with the cap 60 degrees short of its seat: {s}")


def test_relocked_counter():
    t = Trial()
    t.cap(0.0)
    s = t.step(40)
    check(s == M.RELOCKED, f"relocked, left on the counter, hands clear: {s} (0.5)")
    t.cap(M.LOOSE_TURN_DEG)
    s = t.step(5)
    check(s == 0.0, f"relocked, then loosened again on the counter: {s}")


def test_full():
    t = Trial()
    t.cap(0.0)
    s = stored(t)
    check(s == 1.0, f"relocked, stored upright on the cabinet's floor, let go, hands clear, still: {s}")
    t = Trial()
    t.cap(0.0)
    t.move(STORE + np.array([0, 0, 0.0005]))
    grip = grip_links(t.bottle_pos, M.finger_for(M.BODY_R - M.BODY_SQUEEZE))
    s = t.step(40, links=grip)
    check(s == M.RELOCKED, f"relocked and in the cabinet, but the hand still on it: {s} (0.5)")
    t = Trial()
    t.cap(0.0)
    t.move(STORE + np.array([0, 0, 0.0005]))
    for k in range(40):  # creeping 2 cm/s along the floor: never still
        t.move(STORE + np.array([0, 0.001 * (k % 2), 0.0005]))
        s = t.step(1)
    check(s == M.RELOCKED, f"relocked and in the cabinet but moving: {s}")
    t = Trial()
    t.cap(0.0)
    t.move(STORE + np.array([0, 0, 0.0005]))
    s = t.step(M.HOLD_STEPS - 2)
    check(s == M.RELOCKED, f"not yet held still for 1 s: {s}")


def test_store_bounds():
    cases = [
        ("on the counter in front of the wall", (M.X_WALL - 0.05, M.STORE_XY[1], M.TOP), False),
        ("half in the opening", (M.X_WALL + 0.010, M.STORE_XY[1], M.TOP + M.FLOOR_Z), False),
        ("on the floor, well in", (M.X_WALL + 0.046, M.STORE_XY[1], M.TOP + M.FLOOR_Z), True),
        ("hovering 1 cm over the floor", (M.X_WALL + 0.046, M.STORE_XY[1], M.TOP + M.FLOOR_Z + 0.010), False),
        ("sunk 5 mm under the floor's top", (M.X_WALL + 0.046, M.STORE_XY[1], M.TOP + M.FLOOR_Z - 0.005), False),
        ("on the lower glass shelf", (M.X_WALL + 0.046, M.STORE_XY[1], M.TOP + M.SHELF_Z[0] + M.GLASS), True),
        ("on the floor at the left side, base over the side wall", (M.X_WALL + 0.046, M.CAB_Y + M._IN_Y - 0.01,
                                                                    M.TOP + M.FLOOR_Z), False),
        ("past the back (in the wall)", (M.X_WALL + 0.080, M.STORE_XY[1], M.TOP + M.FLOOR_Z), False),
        ("beside the cabinet, on the wall's lower edge height", (M.X_WALL + 0.046, M.CAB_Y + 0.25,
                                                                 M.TOP + M.FLOOR_Z), False),
    ]
    for what, pos, want in cases:
        t = Trial()
        t.cap(0.0)
        t.move(pos)
        check(M.in_cabinet(t.objects()) == want, f"{what}: stored = {want}")
    t = Trial()
    t.cap(0.0)
    t.move((M.X_WALL + 0.046, M.STORE_XY[1], M.TOP + M.FLOOR_Z + 0.02), rx(math.radians(25)))
    check(not M.in_cabinet(t.objects()), "leaning 25 degrees: not stored upright")


def test_spill_drop():
    t = Trial()
    t.move(t.bottle_pos, rx(math.radians(60)))
    grip = grip_links(np.asarray(M.BOTTLE_START) - np.array([0, 0, M.SPAWN_GAP]),
                      M.finger_for(M.BODY_R - M.BODY_SQUEEZE))
    t.step(1, links=grip)
    t.move(np.asarray(M.BOTTLE_START) - np.array([0, 0, M.SPAWN_GAP]))
    t.cap(0.0)
    s = stored(t)
    check(s == 0.0, f"tipped 60 degrees with the cap loose (spilled), then relocked and stored: {s}")
    t = Trial()
    t.cap(0.0)
    t.move(t.bottle_pos + np.array([0, 0, 0.0]), rx(math.radians(90)))
    s = t.step(5)
    check(s == 0.0, f"relocked but knocked over on the counter (no hand on it): {s}")
    t = Trial()
    t.cap(0.0)
    grip0 = grip_links(np.asarray(M.BOTTLE_START) - np.array([0, 0, M.SPAWN_GAP]),
                       M.finger_for(M.BODY_R - M.BODY_SQUEEZE))
    t.step(2, links=grip0)  # held at the start spot
    t.move(t.bottle_pos + np.array([0, 0, 0.03]))
    s = t.step(2)  # let go 3 cm up
    t.move(STORE + np.array([0, 0, 0.0005]))
    s = t.step(40)
    check(s == 0.0, f"relocked, let go 3 cm over the counter (dropped), then found in the cabinet: {s}")
    t = Trial()
    t.cap(0.0)
    t.step(2, links=grip0)
    t.move(t.bottle_pos + np.array([0.02, 0, 0.0]))  # let go while sliding 0.4 m/s
    s = t.step(2)
    check(s == 0.0, f"relocked, flung along the counter on release (0.4 m/s): {s}")


def test_hover_not_touch():
    """A hand 2 mm off the bottle is not holding it: a loose bottle left in the cabinet with a hand hovering is stored
    loose."""
    t = Trial()
    t.move(STORE + np.array([0, 0, 0.0005]))
    grip = grip_links(t.bottle_pos, M.finger_for(M.BODY_R + 0.002))
    t.step(3, links=grip)
    gap = M.arm_gap(t.objects(), "bottle")
    check(gap > M.TOUCH, f"open hand 2 mm off each side: gap {gap * 1000:.1f} mm, not touching")
    check(M._L.stored_loose, "the loose bottle in the cabinet with a hand hovering 2 mm off is stored loose")


def test_cap_apart():
    """The cap held or lying apart from the bottle counts neither as holding the bottle nor as part of its outline."""
    t = Trial()
    check(not M.cap_on_neck(t.objects()), "the start: the cap lies on the counter, not on the neck")
    t.move(t.bottle_pos + np.array([0, 0, 0.010]))
    check(not M.on_counter(t.objects()), "bottle 1 cm up, its cap on the counter: the bottle is not resting there")
    t = Trial()
    real = M.arm_gap
    try:
        M.arm_gap = lambda objects, part, robots=M.ROBOTS: 0.0 if part == "cap" else 0.02  # a hand on the cap only
        t.move(STORE + np.array([0, 0, 0.0005]))
        s = t.step(3)
    finally:
        M.arm_gap = real
    check(M._L.stored_loose and s == 0.0, f"bottle stored open while a hand holds its cap apart: {s}")
    t = Trial()
    t.cap(0.0)
    try:
        M.arm_gap = lambda objects, part, robots=M.ROBOTS: 0.0 if part == "cap" else 0.02  # holding it by its cap
        t.step(2)
        t.move(STORE + np.array([0, 0, 0.0005]))
        t.step(2)
    finally:
        M.arm_gap = real
    s = t.step(40)
    check(s == 1.0, f"relocked bottle carried by its cap, set down in the cabinet, let go: {s}")


def test_new_trial():
    t = Trial()
    t.cap(0.0)
    stored(t)
    t2 = Trial()
    check(t2.step(3) == 0.0, "a new trial starts over (no carried latch or hold)")


def test_demo_plan():
    import yam_arena.demo as demo

    plans = M.demo()
    for side, base in (("left", demo.LEFT_BASE_POS), ("right", demo.RIGHT_BASE_POS)):
        plan = demo.ArmPlan(plans[side], base)
        worst, prev, jump = 0.0, None, 0.0
        for k in range(int(plan.duration / 0.05)):
            a, _ = plan.action7(k * 0.05)
            pos = plan.target(k * 0.05)[0]
            p, _ = demo.ik.tcp_pose(a[:6], base)
            worst = max(worst, float(np.linalg.norm(p - pos)))
            if prev is not None:
                jump = max(jump, float(np.degrees(np.abs(a[:6] - prev).max())))
            prev = a[:6].copy()
        check(worst < 0.003, f"{side} arm: every demo target within 3 mm of IK ({worst * 1000:.1f} mm)")
        check(jump < 6.0, f"{side} arm: no joint jumps more than 6 degrees a step ({jump:.1f})")
    lt, rt = sum(w.duration for w in plans["left"]), sum(w.duration for w in plans["right"])
    check(abs(lt - rt) < 1e-6 and lt / M.DT < M.RUN_LIMIT_STEPS, f"demo {lt:.1f} s = {lt / M.DT:.0f} steps")


def test_text():
    readme = (HERE / "README.md")
    if not readme.exists():
        print("skip README checks (no README yet)")
        return
    txt = readme.read_text()
    for frag in ("30 degrees", "45 degrees", "5 mm", "1 s", "0.5", "3.5"):
        check(frag in txt, f"README states {frag!r}")
    check(not re.search(r"\$\d", txt), "README names no prices")


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    only = sys.argv[1:]
    for f in tests:
        if only and f.__name__ not in only:
            continue
        print(f"-- {f.__name__}")
        f()
    print("all passed")
