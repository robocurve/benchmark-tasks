"""Offline checks of the toothpaste task, no simulator. Brush, tube and cap states are built by hand from the same
geometry the demo uses; the tube's wall pushes are set as its joint states; the arms' links are posed by the robot's
forward kinematics (at the ready pose, or at the demo's grip on the tube) and checked with their own collision hulls.

Covers: the thread (seated without overlap, free along its helix, a straight pull blocked until it has turned about
1.1 turns); the start state (capped, nozzle sealed, scores 0); doing nothing; never opening (squeezing through the
cap); a cap loosened but still on; the demo's pea dose with the cap off, then the ladder of cap states at the end
(screwed fully on, left off, set on loose, part screwed, crooked, a hand still on the tube); too much; too little;
paste off the bristles; the dose scored the same wherever the brush is; bristles tipped while squeezing; the brush
flipped; the tube dropped; the tube left lying capped; a new trial; the start-state cap seat after a shift; the
written USD (no joint or drive on the cap, its materials and offsets); the thresholds in the text; the demo plan.

    python3 test_task.py
"""
import json
import math
import re
import sys
from pathlib import Path

import numpy as np
from yam_arena.task_file import ObjectState, load

HERE = Path(__file__).parent
M = load(HERE / "task.py").module
import capkit as ck  # noqa: E402
import yamkit as kit  # noqa: E402

GRIP_DOWN = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
R_DOSE = np.array([[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]])  # the tube's nozzle (-x) to world -z
R_STAND = np.array([[0.0, 0.0, 1.0], [0.0, -1.0, 0.0], [1.0, 0.0, 0.0]])  # standing on its cap


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


def ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


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


_CACHE = {}


def ready_links():
    if "ready" not in _CACHE:
        import yam_arena.demo as demo

        out = {}
        for r in M.ROBOTS:
            out.update(_fk_poses(r, demo.READY, M.FINGER_OPEN))
        _CACHE["ready"] = out
    return _CACHE["ready"]


def gripping_links(tube_pos, tube_R, finger):
    """The left hand on the lying tube as the demo grips it (from above, tool point on its axis GRIP_X from the body's
    middle, fingers closing across y), at finger command `finger`; the right at the ready pose."""
    key = ("grip", tuple(np.round(tube_pos, 5)), tuple(np.round(tube_R, 5).ravel()), round(finger, 6))
    if key not in _CACHE:
        import yam_arena.demo as demo

        tcp = np.asarray(tube_pos) + tube_R @ np.array([M.GRIP_X, 0.0, 0.0])
        q = _ik("left_robot", tcp, demo.grip_rotation(0.0))
        out = _fk_poses("left_robot", q, finger)
        out.update(_fk_poses("right_robot", demo.READY, M.FINGER_OPEN))
        _CACHE[key] = out
    return _CACHE[key]


def cap_links(cap_pos, cap_R, finger):
    """The right hand on the cap as the demo grips it (pointing forward along the cap's axis, tool point CAP_GRIP_Z in
    from its mouth), the left at the ready pose."""
    key = ("cap", tuple(np.round(cap_pos, 5)), tuple(np.round(cap_R, 5).ravel()), round(finger, 6))
    if key not in _CACHE:
        import yam_arena.demo as demo

        tcp = np.asarray(cap_pos) + cap_R @ np.array([0.0, 0.0, M.CAP_GRIP_Z])
        q = _ik("right_robot", tcp, ry(M.PITCH_FWD) @ demo.grip_rotation(0.0))
        out = _fk_poses("right_robot", q, finger)
        out.update(_fk_poses("left_robot", demo.READY, M.FINGER_OPEN))
        _CACHE[key] = out
    return _CACHE[key]


def capped_pose(tube_pos, tube_R, turned_deg=0.0, extra_rise=0.0, tilt_deg=0.0, off=0.0):
    """The cap on the tube's neck, unscrewed `turned_deg` along the thread (plus `extra_rise` m straight out, a tilt
    and a sideways offset): (pos, R) in the world."""
    o_n = np.asarray(tube_pos) + tube_R @ M.NECK_ORIGIN
    R_n = tube_R @ M.M_NECK
    R_rel = rz(math.radians(turned_deg)) @ rx(math.radians(tilt_deg))
    p_rel = np.array([off, 0.0, ck.SEAT_Z + ck.PITCH * turned_deg / 360.0 + extra_rise])
    return o_n + R_n @ p_rel, R_n @ R_rel


def fresh():
    kit._EPISODE["count"] += 1
    kit._EPISODE["last"] = None
    M._L = None


class Trial:
    """A hand-built trial: the brush, tube and cap where we put them, the tube's walls pushed in `push` m a side."""

    def __init__(self):
        fresh()
        self.brush = (np.array(M.BRUSH_START, float), np.eye(3))
        self.tube = (np.array(M.TUBE_START, float), M.R_LIE.copy())
        self.cap = (np.array(M.CAP_START, float), M.R_LIE @ M._CAP_R)
        self.links = ready_links()
        self.s = 0.0

    def cap_on(self, turned_deg=0.0, **kw):
        self.cap = capped_pose(*self.tube, turned_deg, **kw)

    def cap_off(self):
        """The cap off the tube, standing on its top on the counter to the side (mouth up)."""
        self.cap = (np.array([0.20, 0.12, M.TOP + ck.H_CAP]), rx(math.pi))

    def nozzle_over(self, dx=0.0, dy=0.0, gap=M.NOZZLE_GAP, rot=None):
        """The tube nozzle-down, its mouth `gap` over the bristle top's middle (offset dx, dy along the brush's x, y)."""
        rot = R_DOSE if rot is None else rot
        bp, br = self.brush
        mouth = bp + br @ np.array([dx, dy, 0.0]) + np.array([0.0, 0.0, gap])
        self.tube = (mouth - rot @ np.array(M.TIP), rot)

    def tube_home(self):
        self.tube = (np.array(M.TUBE_START, float), M.R_LIE.copy())

    def step(self, push=0.0, n=1):
        for _ in range(n):
            for link, sign in M.PANEL_SIGN.items():
                d = push if link in ("panel_t", "panel_b") else 0.0
                kit._JOINTS[f"tube/{link}"] = kit.JointState(sign * d, 0.0, False)
            M.LINK_POSES.clear()
            M.LINK_POSES.update(self.links)
            o = {"brush": st(*self.brush), "tube": st(*self.tube), "cap": st(*self.cap),
                 "left_gripper": st((0.15, 0.30, M.TOP + 0.30), GRIP_DOWN),
                 "right_gripper": st((0.15, -0.30, M.TOP + 0.30), GRIP_DOWN)}
            M.success(o)
            self.s = M.score(o)
        return self.s

    def squeeze(self, push, hold_s, tail_s=M.TAIL_S):
        n_ramp = 6
        for k in range(1, n_ramp + 1):
            self.step(push * k / n_ramp)
        self.step(push, round(hold_s / M.DT))
        self.step(M.HOLD_PUSH, round(tail_s / M.DT))

    def recap(self, turned_deg=0.0, still_s=1.5, **kw):
        """The tube back on its cap on the counter, the cap at `turned_deg` up the thread, hands at the ready pose."""
        self.tube_home()
        self.cap_on(turned_deg, **kw)
        self.links = ready_links()
        return self.step(0.0, round(still_s / M.DT))

    @property
    def on(self):
        return M._L.paste.on_g()

    @property
    def off(self):
        return M._L.paste.off_g()


def dosed():
    """The cap off on the counter, the demo's squeeze over the brush lying at its start."""
    t = Trial()
    t.cap_off()
    t.nozzle_over()
    t.squeeze(M.DOSE_PUSH, M.DOSE_S)
    return t


# -- the thread ----------------------------------------------------------------------------------------------------------
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
    R = rz(math.radians(phi_deg)) @ rx(math.radians(tilt_deg))  # tilted about its mouth's middle
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
    check(_overlap_depth(0.0, 0.0) <= 1e-6, "seated: the cap touches the neck only at its seat (no overlap)")
    deep = max(_overlap_depth(ph, ck.PITCH * ph / 360.0) for ph in (60, 180, 300, 420, 540))
    check(deep <= 1e-6, "turned along the helix, the cap's ridge never cuts the neck's")
    for rise in (0.0005, 0.001):
        check(_overlap_depth(0.0, rise) > 1e-5, f"pulled {rise * 1000:.1f} mm straight off its seat without turning: "
              "the ridges block it")
    check(_overlap_depth(0.0, -0.0003) > 1e-5, "pushed past its seat: the neck's tip blocks it")

    def blocked(ph):
        pr = ck.PITCH * ph / 360.0
        return max(_overlap_depth(ph, pr + dz) for dz in np.linspace(0.0005, 0.005, 10)) > 1e-5

    check(blocked(360.0), "after one full turn a straight pull is still blocked")
    for tilt in (3.0, 6.0):  # started crooked (cross-threaded): it cannot sit down on its seat
        deep = min(_overlap_depth(ph, 0.0, tilt_deg=tilt) for ph in (0.0, 90.0, 180.0, 270.0))
        check(deep > 1e-5, f"a cap tilted {tilt:.0f} degrees cannot reach its seat at any turn: its ridges and bore hit "
              f"the neck ({deep * 1000:.2f} mm)")
    free = next(ph for ph in range(360, 540, 15) if not blocked(ph))
    check(360 < free <= M.UNSCREW_DEG - 30, f"the cap pulls free after {free} degrees of turn (the demo turns "
          f"{M.UNSCREW_DEG:.0f})")


def test_start_state():
    t = Trial()
    o = {"tube": st(*t.tube), "cap": st(*t.cap)}
    f = M.cap_fit(o)
    check(abs(f["rise"] + M.SPAWN_GAP) < 1e-6 and f["off"] < 1e-9 and f["tilt"] < 1e-6,
          f"the cap starts seated on the neck (rise {f['rise'] * 1000:.2f} mm)")
    check(M.capped_fully(o) and M.nozzle_covered(o), "start: capped fully, the nozzle inside the cap")
    low = M.lowest_z(o) - M.TOP
    check(0 < low < 0.0005, f"the capped tube lies on the counter (lowest point {low * 1000:.2f} mm up)")
    check(t.step(0.0, 60) == 0.0, "the start state (capped, no paste) scores 0")


def test_nothing():
    t = Trial()
    check(t.step(0.0, 60) == 0.0, "doing nothing scores 0")


def test_never_opened():
    """Squeezing through the closed cap: nothing comes out, however hard."""
    t = Trial()
    t.nozzle_over()
    t.cap_on(0.0)
    t.squeeze(0.006, 4.0)
    check(t.on + t.off == 0.0, "a hard squeeze with the cap screwed on dispenses nothing")
    check(t.recap(0.0) == 0.0, "never opened: 0")


def test_loosened_not_off():
    """A cap loosened (part unscrewed) but still on the neck: the nozzle is still inside it and nothing reaches the
    brush; once the cap is off, the pressure built up comes out."""
    t = Trial()
    t.nozzle_over()
    t.cap_on(200.0)
    t.squeeze(M.DOSE_PUSH, M.DOSE_S)
    check(t.on + t.off == 0.0, "cap loosened 200 degrees but still on: nothing dispensed")
    t.cap_on(400.0)
    t.squeeze(M.DOSE_PUSH, M.DOSE_S)
    check(t.on + t.off == 0.0, "cap loosened 400 degrees, still over the nozzle: nothing dispensed")
    t2 = Trial()
    t2.nozzle_over()
    t2.cap_on(0.0)
    t2.step(M.DOSE_PUSH, 40)
    t2.cap_off()
    t2.step(M.DOSE_PUSH, 1)
    t2.step(M.HOLD_PUSH, 40)
    check(t2.on + t2.off > 0.0, f"squeezed while capped, then uncapped: the pressure comes out ({t2.on + t2.off:.3f} g)")


def test_pea_and_cap_ladder():
    t = dosed()
    check(M.PEA_G[0] <= t.on <= M.PEA_G[1], f"the demo's squeeze with the cap off leaves a pea dose ({t.on:.3f} g)")
    check(t.off <= M.OFF_G, f"nothing off the bristles ({t.off:.3f} g)")
    check(t.step(0.0, 5) == M.OPEN_CAP, "dosed, the cap still off: 0.5")
    dose = t.on
    cases = (("screwed fully on, hands away, all still 1 s", dict(turned_deg=0.0), 1.0),
             ("screwed on to within 40 degrees of its seat", dict(turned_deg=40.0), 1.0),
             ("screwed on to 90 degrees short of its seat", dict(turned_deg=90.0), M.OPEN_CAP),
             ("set on loose without turning (on the thread's crests)", dict(turned_deg=405.0, extra_rise=0.0002),
              M.OPEN_CAP),
             ("on its first turn of thread only", dict(turned_deg=300.0), M.OPEN_CAP),
             ("seated but tilted 3 degrees (crooked)", dict(turned_deg=0.0, tilt_deg=3.0), M.OPEN_CAP),
             ("seated but 1 mm off the neck's axis", dict(turned_deg=0.0, off=0.001), M.OPEN_CAP),
             ("90 degrees short, pushed down into the thread's play", dict(turned_deg=90.0, extra_rise=-0.00025),
              M.OPEN_CAP),
             ("a whole turn short", dict(turned_deg=360.0), M.OPEN_CAP))
    for label, kw, want in cases:
        t = dosed()
        got = t.recap(**kw)
        check(got == want and t.on == dose, f"{label}: {want} (got {got})")
    t = dosed()
    t.tube_home()
    t.cap_off()
    t.links = ready_links()
    check(t.step(0.0, 40) == M.OPEN_CAP, "dosed, tube laid back on the counter but the cap left off: 0.5")
    t = dosed()
    check(t.recap(0.0, still_s=0.5) == 0.75, "capped, but still for only 0.5 s: 0.75")


def test_hand_on_tube():
    """A hand still on the tube (holding it on its cap, or pressing it onto a loose cap) is not finished."""
    t = dosed()
    t.tube_home()
    t.cap_on(0.0)
    hold = M.tube_grip(M.HOLD_PUSH)
    t.links = gripping_links(t.tube[0], t.tube[1], hold)
    o = {"tube": st(*t.tube), "cap": st(*t.cap)}
    M.LINK_POSES.clear()
    M.LINK_POSES.update(t.links)
    g = M.arm_gap(o, "tube")
    check(g <= M.TOUCH, f"the demo's grip on the tube counts as touching it (gap {g * 1000:.2f} mm)")
    check(t.step(0.0, 40) == 0.75, "capped fully but the hand still on the tube: 0.75")
    t.links = gripping_links(t.tube[0], t.tube[1], M.finger_for(M.FINGER_HALF_OPEN))
    M.LINK_POSES.clear()
    M.LINK_POSES.update(t.links)
    g = M.arm_gap(o, "tube")
    check(g > M.CLEAR, f"fingers opened wide around the body: every link more than 5 mm off ({g * 1000:.1f} mm)")
    check(t.step(0.0, 40) == 1.0, "the hand opened wide around the capped tube, touching nothing: 1.0")
    t = dosed()
    t.tube = M.work_tube()  # the cap held by the right at the work height
    t.cap_on(0.0)
    t.links = cap_links(t.cap[0], t.cap[1], M.finger_for(ck.R_CAP - M.CAP_SQUEEZE))
    M.LINK_POSES.clear()
    M.LINK_POSES.update(t.links)
    g = M.arm_gap({"tube": st(*t.tube), "cap": st(*t.cap)}, "cap")
    check(g <= M.TOUCH, f"the demo's grip on the cap counts as touching it (gap {g * 1000:.2f} mm)")
    check(t.step(0.0, 40) == 0.75 and not M._L.clear, "capped fully but the right hand still on the cap: 0.75")
    t = dosed()
    t.tube_home()
    t.cap_on(405.0, extra_rise=0.0002)
    t.links = gripping_links(t.tube[0], t.tube[1], hold)
    check(t.step(0.0, 40) == M.OPEN_CAP, "a hand pressing the tube onto a loose cap: 0.5")
    t.links = ready_links()
    check(t.step(0.0, 40) == M.OPEN_CAP, "and once the hand lets go, the loose cap is still loose: 0.5")


def test_too_much_and_too_little():
    t = Trial()
    t.cap_off()
    t.nozzle_over()
    t.squeeze(*M.VARIANTS["much"])
    check(M.PEA_G[1] < t.on <= M.OVER_G, f"a firmer squeeze: too much ({t.on:.3f} g)")
    check(t.recap() == 0.25, "too much (0.35-0.70 g), capped: 0.25")
    t = Trial()
    t.cap_off()
    t.nozzle_over()
    t.squeeze(0.0060, 4.0)
    check(t.recap() == 0.0 and (t.on > M.OVER_G or t.off > M.OFF_G),
          f"a full load scores 0 ({t.on:.3f} g on, {t.off:.3f} g off)")
    t = Trial()
    t.cap_off()
    t.nozzle_over()
    t.squeeze(*M.VARIANTS["smear"])
    check(M.SMEAR_G <= t.on < M.PEA_G[0], f"a short squeeze: a smear ({t.on:.3f} g)")
    check(t.recap() == 0.25, "a smear, capped: 0.25")
    t = Trial()
    t.cap_off()
    t.nozzle_over()
    t.squeeze(0.0009, 0.3)
    check(t.on < M.SMEAR_G and t.recap() == 0.0, f"a trace ({t.on:.3f} g) scores 0")


def test_off_bristles():
    t = Trial()
    t.cap_off()
    t.nozzle_over(dy=0.030)  # beside the head: onto the counter
    t.squeeze(M.DOSE_PUSH, M.DOSE_S)
    check(t.on == 0.0 and t.off > M.OFF_G, f"squeezed beside the brush: all off ({t.off:.3f} g)")
    check(t.recap() == 0.0, "on the counter scores 0")
    t = Trial()
    t.cap_off()
    t.nozzle_over(dx=0.0085)
    t.squeeze(M.DOSE_PUSH, M.DOSE_S)
    check(t.off > M.OFF_G, f"dosed on the field's edge: some runs off ({t.on:.3f} on, {t.off:.3f} off)")
    check(t.recap() in (0.25, 0.5), f"a dab half off the bristles scores below 0.75 ({t.s})")


def test_placement_sweep():
    """The dose is measured in the brush's frame: the same squeeze over the bristles' middle gives the same grams
    whether the brush lies on the counter or is held anywhere (any heading, tipped up to 40 degrees), and moving it
    afterwards changes nothing."""
    ref, n = None, 0
    for pos in ((0.30, -0.04, M.TOP + 0.014), (0.20, -0.10, M.TOP + 0.08), (0.30, 0.00, M.TOP + 0.15)):
        for k in range(4):
            for tilt in ((0.0, 0.0), (40.0, 0.0), (0.0, 40.0)):
                rot = rz(k * math.pi / 2) @ rx(math.radians(tilt[0])) @ ry(math.radians(tilt[1]))
                t = Trial()
                t.cap_off()
                t.brush = (np.asarray(pos, float), rot)
                t.nozzle_over()
                t.squeeze(M.DOSE_PUSH, M.DOSE_S)
                ref = t.on if ref is None else ref
                t.brush = (np.array(M.BRUSH_START, float), rz(0.3))
                s = t.recap()
                if abs(t.on - ref) > 0.01 or t.off > M.OFF_G or s != 1.0:
                    check(False, f"brush at {pos} heading {k} tilt {tilt}: {t.on:.3f} g on, {t.off:.3f} off, {s}")
                n += 1
    check(n == 36, f"same dose ({ref:.3f} g) and 1.0 at all {n} brush poses")


def test_tipped_and_flipped():
    t = Trial()
    t.cap_off()
    t.brush = (np.array([0.30, 0.02, M.TOP + 0.10]), rx(math.radians(70)))
    t.nozzle_over()
    t.squeeze(M.DOSE_PUSH, M.DOSE_S)
    check(t.on == 0.0, "bristles 70 degrees from up while squeezing: nothing lands on them")
    t = dosed()
    t.brush = (t.brush[0], rx(math.radians(120)))
    t.step(0.0, 3)
    t.brush = (np.array(M.BRUSH_START, float), np.eye(3))
    check(t.recap() == 0.25, "brush turned bristles-down with the dab on it: capped at 0.25")
    t = dosed()
    t.brush = (t.brush[0], rx(math.radians(60)))
    check(t.recap() == 0.75, "brush left with the bristles 60 degrees from up: 0.75")


def test_tube_dropped_or_left_lying():
    """Let go 6 cm up, the capped tube falls onto the counter: not put down (0.75); the same tube set down capped and
    let go: 1.0. A capped tube left lying on its side on the counter is put down too."""
    hold = M.tube_grip(M.HOLD_PUSH)
    for label, h, want in (("dropped from 6 cm", 0.06, 0.75), ("set down", 0.0, 1.0)):
        t = dosed()
        rest = np.array(M.TUBE_START, float)
        t.tube = (rest + np.array([0.0, 0.0, h]), M.R_LIE)
        t.cap_on(0.0)
        t.links = gripping_links(t.tube[0], t.tube[1], hold)
        t.step(0.0, 3)
        t.links = ready_links()
        for k in range(1, 5):
            t.tube = (rest + np.array([0.0, 0.0, h * (1 - k / 4)]), M.R_LIE)
            t.cap_on(0.0)
            t.step(0.0)
        check(t.step(0.0, 40) == want, f"capped tube {label}: {want}")
    t = dosed()
    o = np.array([0.30, -0.20, 0.0])
    t.tube = (o, R_STAND)
    t.cap_on(0.0)
    low = M.lowest_z({"tube": st(*t.tube), "cap": st(*t.cap)})
    t.tube = (o + np.array([0.0, 0.0, M.TOP - low]), R_STAND)
    t.cap_on(0.0)
    t.links = ready_links()
    check(t.step(0.0, 40) == 1.0, "capped fully and standing on its cap on the counter, hands away: 1.0")


def test_new_trial():
    t = dosed()
    t.recap()
    t = Trial()
    check(t.step(0.0, 30) == 0.0 and t.on == 0.0, "a new trial starts with no paste and scores 0")


def test_cap_reseat_math():
    """The reset hook puts the cap screwed on under the tube wherever the tube was shifted: the pose it writes is the
    seated pose."""
    for dx, dy, yaw in ((0.0, 0.0, 0.0), (0.03, -0.02, 0.0), (-0.025, 0.03, 0.4)):
        Rt = rz(yaw) @ M.R_LIE
        tp = np.asarray(M.TUBE_START) + np.array([dx, dy, 0.0])
        o, R = M.seated_cap_in_tube(M.SPAWN_GAP)
        objs = {"tube": st(tp, Rt), "cap": st(tp + Rt @ o, Rt @ R)}
        f = M.cap_fit(objs)
        check(abs(f["rise"] + M.SPAWN_GAP) < 1e-9 and f["off"] < 1e-9 and M.capped_fully(objs),
              f"tube shifted ({dx}, {dy}, yaw {yaw}): cap re-seated")


def test_usd():
    cap = kit._usda(M.CAP)
    check("Joint" not in cap and "Drive" not in cap, "the cap's USD has no joint or drive: a free body")
    check(cap.count("convexHull") == len(M.CAP.colliders), f"every cap collider is a convex hull "
          f"({len(M.CAP.colliders)})")
    check("physxCollision:contactOffset = 0.001" in cap and "staticFriction = 0.2" in cap and "diagonalInertia" in cap,
          "the cap carries its contact offset, plastic friction and its own inertia")
    tube = kit._mech_usda(M.TUBE)
    names = re.findall(r'def Physics\w*Joint "(\w+)"', tube)
    check(sorted(names) == sorted(f"{p}_joint" for p in M.PANEL_SIGN), f"the tube's only joints are its four wall "
          f"panels ({names})")
    src = (HERE / "task.py").read_text() + (HERE / "capkit.py").read_text()
    for word in ("ZONE", "PULL", "Latch(", "snap_"):
        check(word not in src, f"no '{word}' in task.py or capkit.py")


def test_text_numbers():
    js = json.loads((HERE / "task.json").read_text())
    readme = (HERE / "README.md").read_text()
    text = js["scoring"] + " " + js["setup"] + " " + js["fidelity"]
    for v in ("0.15", "0.35", "0.05 g", "0.70", "45 degrees", "60 degrees", "1.30 g/mL", "0.01 g", "5 mm"):
        check(v in text and v in readme, f"'{v}' in task.json and the README")
    check(js["instruction"] == M.INSTRUCTION, "instruction matches task.py")
    check(re.search(r"2,400 steps", js["run_limit"]) is not None and M.RUN_LIMIT_STEPS == 2400, "run limit 2,400")
    check(js["category"] == "Societal relatability", "category")
    limits = {"title": 120, "instruction": 1000, "context": 1000, "setup": 4000, "scoring": 4000, "fidelity": 500,
              "scores": 300, "reset_notes": 1500}
    for k, n in limits.items():
        check(len(js[k]) <= n, f"task.json {k} within {n} characters ({len(js[k])})")


def test_demo_plan():
    plan = M.demo()
    total = {a: sum(w.duration for w in p) for a, p in plan.items()}
    check(max(total.values()) / M.DT + 2 * M.HOLD_STEPS <= M.RUN_LIMIT_STEPS,
          f"demo fits the run limit with room for the hold ({max(total.values()):.1f} s)")
    tw = [w for w in plan["right"] if isinstance(w, kit.Tilted)]
    check(min(w.yaw for w in tw) >= M.YAW_LO - 1e-9 and max(w.yaw for w in tw) <= M.YAW_HI + 1e-9,
          "the wrist stays within its stroke")
    check(all(abs(b.yaw - a.yaw) <= math.pi / 2 + 1e-9 for a, b in zip(tw, tw[1:])),
          "no waypoint turns the wrist more than a quarter turn (each turn goes the way it is meant to)")
    rl = [w.label for w in plan["right"]]
    ll = [w.label for w in plan["left"]]

    def at(seq, lab):
        return sum(w.duration for w in seq[:[w.label for w in seq].index(lab) + 1])

    check(at(plan["right"], "draw the cap off") <= at(plan["left"], "squeeze")
          <= at(plan["right"], "onto the neck, pressed lightly"),
          "unscrew, dose, then screw back on")
    check("grip the cap" in rl and "grip the body" in ll, "the left holds the tube and the right turns the cap")
    _, _, turned = M.strokes(M.UNSCREW_DEG, +1.0, M.YAW_LO, 0.0, 0.0, "x")
    check(abs(turned - M.UNSCREW_DEG) < 1e-6 and M.SCREW_DEG > M.UNSCREW_DEG, "screws back on past where it came off")
    check(all(w.finger == M.finger_for(ck.R_CAP + M.CAP_OPEN) for w in plan["right"] if "turn back" in w.label),
          "every turn-back is made with the fingers open clear of the cap")
    # screwing back on, the cap goes onto the neck and every stroke is aimed PUSH_ON past where the thread puts it (the
    # arms' give presses it on); the hand opens and turns back where the thread puts the cap (open fingers not pushed in)
    i0, i1 = rl.index("onto the neck, pressed lightly"), rl.index("let go of the cap")
    axis = np.array(M.cap_tcp(0.0, back=0.01)) - np.array(M.cap_tcp(0.0))
    axis /= np.linalg.norm(axis)  # out of the neck, along the cap's axis

    def on_line(w, back):  # the waypoint lies on cap_tcp(t, back) for some turn t
        return min(abs(float(np.dot(np.array(w.pos) - np.array(M.cap_tcp(t, back=back)), axis)))
                   for t in np.arange(0.0, 481.0, 0.5)) < 1e-6

    seg = [w for w in plan["right"][i0:i1] if isinstance(w, kit.Tilted)]
    check(np.allclose(seg[0].pos, M.cap_tcp(M.UNSCREW_DEG, back=-M.PUSH_ON), atol=1e-9) and M.PUSH_ON >= 0.002,
          "the cap goes onto the neck pressed lightly (aimed PUSH_ON past the thread)")
    st_ = [w for w in seg if w.label.startswith("screw on: stroke") or w.label.endswith("press it on")]
    check(len(st_) > 4 and all(on_line(w, -M.PUSH_ON) for w in st_), "every screw-on stroke presses the cap on")
    op_ = [w for w in seg if w.finger == M.finger_for(ck.R_CAP + M.CAP_OPEN)]
    check(len(op_) > 0 and all(on_line(w, 0.0) for w in op_),
          "the hand opens and turns back where the thread puts the cap (not pressed in)")
    _, _, t_unscr = M.strokes(M.UNSCREW_DEG, +1.0, M.YAW_LO, 0.0, 0.0, "x")
    w_un, _, _ = M.strokes(M.UNSCREW_DEG, +1.0, M.YAW_LO, 0.0, 0.0, "x")
    check(not any("press" in w.label for w in w_un), "unscrewing is unchanged (no press)")
    closes = [w for w in plan["right"] + plan["left"] if w.label.endswith("grip again") or w.label.startswith("grip the")]
    check(all(w.duration >= 1.2 for w in closes), "every close takes at least 1.2 s")


def test_brush_must_be_let_go():
    """The brush has to end lying on the counter, out of the hands: held up at the end it is not finished."""
    t = dosed()
    t.brush = (np.array(M.BRUSH_START, float) + np.array([0.0, 0.0, 0.05]), np.eye(3))
    check(t.recap() == 0.75, "dosed and capped, the brush hovering 5 cm up at the end: 0.75")


def test_nozzle_direction_and_height():
    """Paste reaches the bristles only from a nozzle pointing down and close to them."""
    t = Trial()
    t.cap_off()
    up = rx(math.pi) @ R_DOSE  # nozzle up
    bp = np.array(M.BRUSH_START, float)
    t.tube = (bp + np.array([0.0, 0.0, 0.10]), up)
    t.squeeze(M.DOSE_PUSH, M.DOSE_S)
    check(t.on == 0.0, f"squeezed with the nozzle pointing up over the brush: nothing on the bristles ({t.on:.3f} g)")
    t = Trial()
    t.cap_off()
    t.nozzle_over(gap=0.30)
    t.squeeze(M.DOSE_PUSH, M.DOSE_S)
    check(t.on == 0.0 and t.off > 0.0, "squeezed from 30 cm up: lands off the bristles")
    t = Trial()
    t.cap_off()
    t.nozzle_over(gap=0.02)
    t.squeeze(M.DOSE_PUSH, M.DOSE_S)
    check(M.PEA_G[0] <= t.on <= M.PEA_G[1], f"from 2 cm up it lands ({t.on:.3f} g)")


def test_hull_sizes():
    """Every convex hull collider has at most 64 points (PhysX cooks larger ones down, changing their shape)."""
    for name, cols in (("tube", M.TUBE.colliders), ("cap", M.CAP.colliders)):
        n = max(len(c.hull) for c in cols if c.hull is not None)
        check(n <= 64, f"the {name}'s largest hull has {n} points")


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for f in tests:
        f()
    print("all pass")
    sys.exit(0)
