"""Offline checks of the masher-jammed drawer task, no simulator. Drawer and masher states are built by hand from the
task's own geometry; the arms' links are posed by the robot's forward kinematics and checked with their collision
hulls.

Covers: the geometry (the head standing on end is taller than the rail's clearance, lying on its fins it clears; the
masher starts inside the drawer, on its floor, clear of its walls; the snag comes before OPEN_PAST); the readings
(flat, pitch, in the drawer); every cheat mode (doing nothing, pulled to the snag, a yank that levers the masher up, a
head forced under the rail, lifted out and put back, opened then shut again, still holding the pull, not still) and
the full ladder; a new trial; the demo plans'
reach for every variant; per-hull clearance of both arms from the cabinet, the drawer and the masher, and from each
other, over the whole demo; the README's numbers.

    /data/IsaacLab-Arena/.venv/bin/python tests/test_task.py
"""
import math
import os
import re
import sys
from pathlib import Path

import numpy as np
from yam_arena.task_file import ObjectState, load

HERE = Path(__file__).resolve().parent.parent
M = load(HERE / "task.py").module
import yamgeom as yg  # noqa: E402
import yamkit as kit  # noqa: E402


def check(cond, what):
    print(("ok   " if cond else "FAIL ") + what)
    if not cond:
        raise AssertionError(what)


def st(pos, rot=np.eye(3)):
    p = np.asarray(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.asarray(M.quat(np.asarray(rot)), float), lin_vel=np.zeros(3),
                       ang_vel=np.zeros(3), low=p - 0.01, high=p + 0.01)


CAB = np.array([M.XF, M.CAB_Y, M.TOP])


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])
_CACHE = {}


# -- masher poses (cabinet frame, drawer shut; the drawer's opening is added when posed) -------------------------------
def standing():
    return M.MASHER_START_C.copy(), M.MASHER_R0.copy()


def _rest(R, dy=0.0):
    """The masher turned to R, its head against the back wall (1 mm) and its lowest point on the floor."""
    c = M.MASHER_START_C.copy()
    c[1] += dy
    P = c + M.OUTLINE @ R.T
    return c + np.array([M.IN_X1 - 0.001 - P[:, 0].max(), 0.0, M.FLOOR + 0.0005 - P[:, 2].min()]), R


def flat_pose(dy=0.0):
    """Rolled 90 degrees toward the drawer's middle: lying on its fins (the handle level)."""
    return _rest(_rx(math.pi / 2) @ M._R_LIE, dy - M.HEAD_S / 2 - M.HEAD_L / 2)


def tilted(deg):
    """Rolled to `deg` from lying on its fins (the head's long side `deg` from level)."""
    return _rest(_rx(math.pi / 2 - math.radians(deg)) @ M._R_LIE, -M.HEAD_L / 2)


def levered(deg):
    """The masher levered up about the head's bottom against the back wall (the handle raised `deg` from level)."""
    return _rest(M._ry(math.radians(deg)) @ M._R_LIE)


def grip_up(deg):
    """The start stance with the handle turned about the head's middle to `deg` above level (the grip lifted by a
    hand), its lowest point on the floor."""
    R = M._ry(math.radians(deg)) @ M._R_LIE
    c = M.MASHER_START_C.copy()
    c[2] = M.FLOOR - (M.OUTLINE @ R.T)[:, 2].min()
    return c, R


def flat_grip_up(deg):
    """Lying on its fins with the grip's end raised `deg` (about the head, against the back wall, on the floor)."""
    return _rest(M._ry(math.radians(deg)) @ _rx(math.pi / 2) @ M._R_LIE, -M.HEAD_S / 2 - M.HEAD_L / 2)


def snag_d(pose=None):
    """How far the drawer opens before the masher's outline (standing, or `pose`) meets the rail's back face."""
    c, R = pose if pose is not None else standing()
    P = c + M.OUTLINE @ R.T
    hi = P[P[:, 2] > M.RAIL_Z0]
    return float(hi[:, 0].min() - M.FF_T)


# -- the arms' link poses (forward kinematics) ---------------------------------------------------------------------------
def _fk_poses(robot, q6, grip):
    import yam_arena.demo as demo
    from yam_arena.analysis import fk_link_poses_world

    base = demo.LEFT_BASE_POS if robot == "left_robot" else demo.RIGHT_BASE_POS
    q = {f"joint{i + 1}": float(q6[i]) for i in range(6)}
    q["joint7"] = q["joint8"] = float(grip) * M.FINGER_OPEN  # ArmPlan's a[6] is the opening fraction
    poses = fk_link_poses_world(q, base)
    return {f"{robot}/{k}": (P[:3, 3].copy(), P[:3, :3].copy()) for k, P in poses.items() if k in M.ARM_LINKS}


def ready_links():
    if "ready" not in _CACHE:
        import yam_arena.demo as demo

        out = {}
        for r in M.ROBOTS:
            out.update(_fk_poses(r, demo.READY, 1.0))
        _CACHE["ready"] = out
    return _CACHE["ready"]


def plan_links(side, t, variant=""):
    """Both arms posed as the demo has them at time t (the other arm at the ready pose)."""
    import yam_arena.demo as demo

    key = ("plan", side, round(t, 3), variant)
    if key not in _CACHE:
        os.environ["MASHER_VARIANT"] = variant
        plans = M.demo()
        os.environ["MASHER_VARIANT"] = ""
        base = demo.LEFT_BASE_POS if side == "left" else demo.RIGHT_BASE_POS
        a, _ = demo.ArmPlan(plans[side], base).action7(t)
        out = dict(ready_links())
        out.update(_fk_poses(f"{side}_robot", a[:6], a[6]))
        _CACHE[key] = out
    return _CACHE[key]


def _plan_time(side, label_prefix, frac=0.99):
    t = 0.0
    for w in M.demo()[side]:
        if w.label.startswith(label_prefix):
            return t + frac * w.duration
        t += w.duration
    raise KeyError(label_prefix)


# -- a trial driven by hand ------------------------------------------------------------------------------------------------
class Trial:
    def __init__(self):
        kit._EPISODE["count"] = kit._EPISODE.get("count", 0) + 1
        kit._EPISODE["last"] = 0
        M._L = None
        self.n = 0
        self.d = 0.0
        self.c, self.R = standing()
        self.links = ready_links()

    def set(self, pose=None, d=None):
        if pose is not None:
            self.c, self.R = pose
        if d is not None:
            self.d = d

    def objects(self):
        kit._JOINTS["cabinet/drawer"] = kit.JointState(pos=-self.d, vel=0.0, latched=False)
        p = CAB + self.c - np.array([self.d, 0.0, 0.0])
        return {"cabinet": st(CAB), "cabinet/drawer": st(CAB - np.array([self.d, 0, 0])), "masher": st(p, self.R)}

    def step(self, n=1, links=None):
        M.LINK_POSES.clear()
        M.LINK_POSES.update(self.links if links is None else links)
        for _ in range(n):
            self.n += 1
            kit._EPISODE["last"] = self.n
            M.success(self.objects())
        return M.score(self.objects())


def solved(t, links=None):
    """Masher flat, the drawer drawn out to the demo's end, hands at the ready pose for 30 steps."""
    t.set(flat_pose(), M.FULL_PULL)
    return t.step(30, links=links)


# -- geometry -------------------------------------------------------------------------------------------------------------
def _corners_world(c, R):
    loc = np.array([(sx * M.HEAD_T / 2, sy * M.HEAD_L / 2, sz * M.HEAD_S / 2)
                    for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    return c + loc @ R.T


def test_geometry():
    c, R = standing()
    pc = _corners_world(c, R)
    P = c + M.OUTLINE @ R.T
    top = P[:, 2].max() - M.FLOOR
    check(top > M.RAIL_Z0 - M.FLOOR + 0.010, f"the head on end {top * 1000:.1f} mm tall > rail clearance "
                                             f"{(M.RAIL_Z0 - M.FLOOR) * 1000:.0f} mm by >10 mm")
    check(abs(pc[:, 2].min() - M.FLOOR) < 1e-4 and abs(P[:, 2].min() - M.FLOOR) < 1e-4,
          "the head's end rests on the floor, nothing lower")
    butt_low = (c + R @ np.array([M.GRIP_X1, 0, 0]))[2] - M.GRIP_R * math.cos(M.BETA)
    check(abs(butt_low - M.FLOOR) < 1e-4, "the handle's end rests on the floor")
    check(P[:, 0].max() <= M.IN_X1 - 0.0009, "the head stands in front of the back wall (1 mm)")
    end_x = (c + R @ np.array([M.GRIP_X1, 0, 0]))[0] - M.GRIP_R * math.sin(M.BETA)
    check(end_x >= M.IN_X0 + 0.002, f"the handle's end is behind the front wall ({(end_x - M.IN_X0) * 1000:.1f} mm)")
    check(np.all(np.abs(P[:, 1]) <= M.IN_HALF_W - 0.02), "clear of the sides")
    com = c + R @ np.array(M.MASHER_COM)  # it stands: its centre of mass over the triangle of the head's bottom edge
    foot = pc[np.argsort(pc[:, 2])[:2]]  # and the handle's end
    end = c + R @ np.array([M.GRIP_X1, 0, 0])
    f = (foot[:, 0].mean() - com[0]) / (foot[:, 0].mean() - end[0])
    half = (1 - f) * abs(foot[0, 1] - foot[1, 1]) / 2
    check(abs(com[1] - c[1]) < half - 0.01, f"standing on end is stable ({half * 1000:.0f} mm each side at its centre)")
    fc, fR = flat_pose()
    ftop = (fc + M.OUTLINE @ fR.T)[:, 2].max() - M.FLOOR
    check(ftop < M.RAIL_Z0 - M.FLOOR - 0.015, f"on its fins, it is {ftop * 1000:.1f} mm tall: clears the rail by 15 mm")
    check(M.SIDE_TOP < M.RAIL_Z0 - 0.010, "the drawer's sides pass under the rail")
    snag = snag_d()
    check(snag < M.OPEN_PAST < M.OPEN_FULL < M.TRAVEL,
          f"snag {snag:.3f} < past {M.OPEN_PAST} < full {M.OPEN_FULL} < stop {M.TRAVEL}")
    check(snag - M.EASE > 0.010 and M.SNAG_PULL > snag,
          f"the demo pulls into the snag, eases back to {M.EASE * 1000:.0f} mm ({(snag - M.EASE) * 1000:.0f} mm clear)")
    for deg in range(0, 95, 5):  # turned under the counter at EASE, nothing rises under or past the rail
        cc, RR = _rest(_rx(math.radians(deg)) @ M.MASHER_R0)
        Q = cc + M.OUTLINE @ RR.T - np.array([M.EASE, 0, 0])
        bad = (Q[:, 2] > M.RAIL_Z0) & (Q[:, 0] < M.FF_T)
        check(not bad.any(), f"rolled {deg} deg at {M.EASE * 1000:.0f} mm open: clear of the rail")
    tot = M.MASHER_KG * 1000
    check(abs(tot - 136.0) < 0.5, f"masher {tot:.0f} g (0.3 lb)")
    check(abs(M.GRIP_X1 + M.HEAD_T / 2 - 0.259) < 0.001, "10.2 in overall")
    check(abs(M.HEAD_L - 0.099) < 0.001, "3.9 in wide")
    check(M.XF + M.CAB_D <= 0.75, "the cabinet stands on the bench (its back inside the far edge)")


def test_readings():
    t = Trial()
    o = t.objects()
    level = math.degrees(math.asin(min(1.0, abs(float(M.masher_pose(o)[1][2, 1])))))
    check(81 < level < 82 and not M.flat(o), "standing: the head 81 degrees from level, not flat")
    check(abs(M.pitch_deg(o) - math.degrees(M.BETA)) < 0.1, "the handle starts 8.6 degrees from level")
    check(not M.over_rail(o), "the start is clear of the rail")
    check(M.in_drawer(o) and M.lowest_over_floor(o) < 1e-3, "the start is in the drawer, on its floor")
    t.set(flat_pose())
    o = t.objects()
    check(M.flat(o) and M.in_drawer(o), "rolled over: flat and in the drawer")
    for deg, want in ((5, True), (10, True), (13, False), (18, False)):
        t.set(tilted(deg))
        o = t.objects()
        check(M.flat(o) == want, f"tilted {deg} deg from its fins ({M.head_top(o) * 1000:.1f} mm tall): "
                                 f"{'flat' if want else 'not flat'}")
    t.set(flat_pose(dy=-0.05))
    check(not M.in_drawer(t.objects()), "over the side wall: not in the drawer")


# -- cheat modes and the ladder ---------------------------------------------------------------------------------------------
def test_nothing():
    t = Trial()
    check(t.step(40) == 0.0, "doing nothing scores 0 (the start satisfies no tier)")


def test_snag():
    t = Trial()
    t.set(d=snag_d() + 0.002)
    check(t.step(40) == 0.0 and not M._L.forced, "pulled 2 mm into the snag and left there: 0 (not latched forced)")


def test_yank():
    t = Trial()
    t.set(levered(35.0), snag_d() - 0.001)  # at the snag, levered up against the rail
    t.step(2)
    t.set(flat_pose(), M.FULL_PULL)  # however it then lands, flat and fully out
    check(t.step(40) == 0.0, "a yank that levers the masher up past 30 degrees scores 0 (latched)")
    t = Trial()
    t.set(levered(25.0), snag_d(levered(25.0)) - 0.001)
    t.step(2)
    t.set(flat_pose(), M.FULL_PULL)
    check(t.step(40) == 1.0, "a pull that lifts the handle 25 degrees, then the masher turned: still 1.0")
    t = Trial()
    t.set(d=M.OPEN_PAST + 0.002)  # a standing masher past the rail (only by passing through it)
    t.step(2)
    t.set(flat_pose(), M.FULL_PULL)
    check(t.step(40) == 0.0, "a standing masher dragged past the rail scores 0 (latched)")
    t = Trial()
    t.set(d=snag_d() + 0.010)  # 10 mm through the rail, short of OPEN_PAST
    t.step(2)
    t.set(flat_pose(), M.FULL_PULL)
    check(t.step(40) == 0.0, "the masher forced 10 mm under the rail, then turned flat and opened: 0 (latched)")
    t = Trial()
    t.set(tilted(18.0), 0.285)  # tilted 18 deg (91 mm tall), dragged under the rail
    t.step(2)
    t.set(flat_pose(), M.FULL_PULL)
    check(t.step(40) == 0.0, "a head tilted 18 deg dragged under the rail to 285 mm, then flat at the stop: 0")


def test_grip_lifted():
    """Lifting the grip in front of the cabinet while turning the masher is the skill, not a forced head."""
    for deg, d in ((9.4, M.EASE), (12.0, 0.170)):  # (at 185 mm the fork meets the rail's underside at about +10 deg)
        t = Trial()
        t.set(grip_up(deg), d)
        t.step(2)
        check(not M._L.forced, f"on end, the grip lifted to {deg} deg above level at {d * 1000:.0f} mm open: no latch")
        t.set(flat_pose(), M.FULL_PULL)
        check(t.step(40) == 1.0, "  then turned flat and drawn out: 1.0")
    for deg, d in ((15.0, M.EASE), (32.0, 0.150)):  # 32 deg: past the pitch limit, short of the snag
        t = Trial()
        t.set(flat_grip_up(deg), d)
        t.step(2)
        check(not M._L.forced, f"flat, the grip's end raised {deg:.0f} deg at {d * 1000:.0f} mm open: no latch")
        t.set(flat_pose(), M.FULL_PULL)
        check(t.step(40) == 1.0, "  then laid flat and drawn out: 1.0")


def test_lift_out():
    t = Trial()
    c, R = flat_pose()
    t.set((c + np.array([0, 0, M.LIFTED + 0.005]), R), M.EASE)
    t.step(2)
    t.set(flat_pose(), M.FULL_PULL)
    check(t.step(40) == 0.0, "lifted more than 40 mm off the floor and put back: 0 (latched)")
    t = Trial()
    t.set((np.array([-0.15, -0.30, 0.04]), _rx(math.pi / 2) @ M.MASHER_R0), M.FULL_PULL)
    check(t.step(40) == 0.0, "taken out and left on the bench, the drawer fully open: 0")
    t = Trial()
    c, R = flat_pose()
    t.set((c + np.array([0, 0, 0.02]), R), M.EASE)
    t.step(2)
    t.set(flat_pose(), M.FULL_PULL)
    check(t.step(40) == 1.0, "lifted 20 mm while being turned (under the 40 mm limit): still 1.0")


def test_ladder():
    t = Trial()
    t.set(flat_pose(), M.EASE)
    check(t.step(30) == 0.25, "turned flat, the drawer not past the snag: 0.25")
    t.set(d=M.OPEN_PAST + 0.005)
    check(t.step(30) == 0.5, "flat, the drawer past the snag but not fully out: 0.5")
    t.set(d=M.OPEN_FULL + 0.001)
    check(t.step(M.HOLD_STEPS + 2) == 1.0, "flat, fully out, hands clear, still 1 s: 1.0")
    t = Trial()
    t.set(flat_pose(), M.OPEN_FULL + 0.001)
    check(t.step(M.HOLD_STEPS - 2) == 0.5, "not yet still for 1 s: 0.5")


def test_close_again():
    t = Trial()
    check(solved(t) == 1.0, "solved")
    t.set(d=0.02)
    check(t.step(30) == 0.25, "opened, then shut again: 0.25 (end state)")


def test_not_still():
    t = Trial()
    t.set(flat_pose(), M.FULL_PULL)
    for k in range(40):
        t.set(d=M.FULL_PULL - 0.002 * (k % 2))  # 4 cm/s back and forth
        s = t.step(1)
    check(s == 0.5, "the drawer still moving: 0.5")


def test_holding():
    tt = _plan_time("left", "draw it out to its stop", 0.999)
    links = plan_links("left", tt)
    t = Trial()
    t.set(flat_pose(), M.FULL_PULL)
    s = t.step(40, links=links)
    gf = M.arm_gap(t.objects(), "front")
    check(gf <= M.CLEAR and s == 0.5, f"still holding the pull at the end: 0.5 (gap {gf * 1000:.1f} mm)")


def test_new_trial():
    t = Trial()
    solved(t)
    t2 = Trial()
    check(t2.step(3) == 0.0, "a new trial starts over (no carried latch or hold)")


# -- the demo plans -----------------------------------------------------------------------------------------------------
VARIANTS = ("", "yank", "lift", "close", "hold", "nothing")


def test_demo_plan():
    import yam_arena.demo as demo

    for v in VARIANTS:
        os.environ["MASHER_VARIANT"] = v
        plans = M.demo()
        os.environ["MASHER_VARIANT"] = ""
        for side, base in (("left", demo.LEFT_BASE_POS), ("right", demo.RIGHT_BASE_POS)):
            plan = demo.ArmPlan(plans[side], base)
            worst, prev, jump = 0.0, None, 0.0
            for k in range(0, int(plan.duration / 0.05), 2):
                a, _ = plan.action7(k * 0.05)
                pos = plan.target(k * 0.05)[0]
                p, _ = demo.ik.tcp_pose(a[:6], base)
                worst = max(worst, float(np.linalg.norm(p - pos)))
                if prev is not None:
                    jump = max(jump, float(np.degrees(np.abs(a[:6] - prev).max())))
                prev = a[:6].copy()
            check(worst < 0.003 and jump < 12.0,
                  f"[{v or 'demo'}] {side}: targets within 3 mm of IK ({worst * 1000:.1f}), joint steps {jump:.1f} deg/0.1 s")
        lt, rt = sum(w.duration for w in plans["left"]), sum(w.duration for w in plans["right"])
        check(abs(lt - rt) < 1e-6 and lt / M.DT < M.RUN_LIMIT_STEPS, f"[{v or 'demo'}] {lt:.1f} s = {lt / M.DT:.0f} steps")


def test_open_loop():
    src = (HERE / "task.py").read_text()
    body = src[src.index("def demo():"):]
    check("objects" not in body and "LINK_POSES" not in body and "kit.joint" not in body,
          "the demo reads no simulator state (open loop)")


# -- per-hull clearance over the whole demo -------------------------------------------------------------------------------
def _scene_boxes():
    """(name, lo, hi, moves_with_drawer) of the cabinet's and the drawer's box colliders, cabinet frame."""
    out = []
    for i, (_, c) in enumerate(M._CAB_PARTS):
        ce, s = np.asarray(c.center), np.asarray(c.size)
        out.append((f"cab{i}", ce - s / 2, ce + s / 2, False))
    for i, (_, c) in enumerate(M._DRAWER_PARTS):
        ce, s = np.asarray(c.center), np.asarray(c.size)
        out.append((f"drw{i}", ce - s / 2, ce + s / 2, True))
    return out


def _link_points(key, pose, every=40):  # ~3 mm spacing: a screen; gaps checked with 2 mm added
    link = key.split("/")[1]
    if ("pts", link) not in _CACHE:
        _CACHE[("pts", link)] = yg.points(link)[::every]
    o, R = pose
    return np.asarray(o) + _CACHE[("pts", link)] @ np.asarray(R).T


def _clearance_run(variant=""):
    import yam_arena.demo as demo
    from scipy.spatial import cKDTree

    os.environ["MASHER_VARIANT"] = variant
    plans = M.demo()
    os.environ["MASHER_VARIANT"] = ""
    lp = demo.ArmPlan(plans["left"], demo.LEFT_BASE_POS)
    rp = demo.ArmPlan(plans["right"], demo.RIGHT_BASE_POS)
    boxes = _scene_boxes()
    pull_shut = M.XF + M._PULL_X
    d, worst_scene, worst_arm, roll_started = 0.0, (1.0, ""), (1.0, ""), False
    snag = snag_d()
    T = min(lp.duration, rp.duration)
    for k in range(int(T / 0.4)):
        t = k * 0.4
        al, lab_l = lp.action7(t)
        ar, lab_r = rp.action7(t)
        if al[6] < 0.5 and "pull" not in lab_l.split("(")[0][:4]:  # the left holds the bar: the drawer follows it
            d = float(np.clip(pull_shut - lp.target(t)[0][0], 0.0, M.TRAVEL))
        d_eff = min(d, snag) if not roll_started else d  # the snag stops it
        if lab_r.startswith("roll"):
            roll_started = True
        links = {}
        links.update(_fk_poses("left_robot", al[:6], al[6]))
        links.update(_fk_poses("right_robot", ar[:6], ar[6]))
        pts = {key: _link_points(key, pose) for key, pose in links.items()}
        for key, P in pts.items():
            q = P - CAB
            for name, lo, hi, moves in boxes:
                sh = np.array([d_eff, 0, 0]) if moves else 0.0
                g = float(M._box_gap(q + sh, lo, hi).min())
                if g < worst_scene[0]:
                    worst_scene = (g, f"t={t:.1f} {key} vs {name} [{lab_l} | {lab_r}]")
            gt = float((P[:, 2] - M.TOP).min())  # the bench
            if gt < worst_scene[0]:
                worst_scene = (gt, f"t={t:.1f} {key} vs bench")
            if key.startswith("right") and not roll_started:  # the masher where it starts (before the roll)
                c, R = standing()
                qm = (q + np.array([d_eff, 0, 0]) - c) @ R
                allowed = lab_r.startswith(("grip", "settle", "roll")) and key.split("/")[1] in ("tip_left", "tip_right")
                g = float(M.masher_gap(qm).min())
                if not allowed and g < worst_scene[0]:
                    worst_scene = (g, f"t={t:.1f} {key} vs masher [{lab_r}]")
            if key.startswith("right") and "/tip" not in key:  # the right's arm vs the pull
                g = float(M.front_gap(q + np.array([d_eff, 0, 0])).min())
                if g < worst_scene[0]:
                    worst_scene = (g, f"t={t:.1f} {key} vs pull/front")
        L = np.vstack([P for key, P in pts.items() if key.startswith("left")])
        Rr = np.vstack([P for key, P in pts.items() if key.startswith("right")])
        dd, _ = cKDTree(L).query(Rr, k=1)
        if dd.min() < worst_arm[0]:
            worst_arm = (float(dd.min()), f"t={t:.1f} [{lab_l} | {lab_r}]")
    return worst_scene, worst_arm


def test_clearance():
    for v in ("", "yank"):
        scene, arm = _clearance_run(v)
        check(scene[0] >= 0.005, f"[{v or 'demo'}] arms >= 5 mm (3 mm + 2 mm sampling) from cabinet, drawer, bench, masher "
                                 f"(min {scene[0] * 1000:.1f} mm: {scene[1]})")
        check(arm[0] >= 0.010, f"[{v or 'demo'}] arms >= 10 mm apart (min {arm[0] * 1000:.1f} mm: {arm[1]})")


def test_text():
    readme = HERE / "README.md"
    if not readme.exists():
        print("skip README checks (no README yet)")
        return
    txt = readme.read_text()
    for frag in ("83 mm", "30 degrees", "40 mm", "5 mm", "1 s", "280 mm", "295 mm", "0.25", "0.5"):
        check(frag in txt, f"README states {frag!r}")
    check(not re.search(r"\$\d", txt), "README names no prices")


if __name__ == "__main__":
    tests = [v for k, v in globals().items() if k.startswith("test_")]
    only = sys.argv[1:]
    for f in tests:
        if only and f.__name__ not in only:
            continue
        print(f"-- {f.__name__}")
        f()
    print("all passed")
