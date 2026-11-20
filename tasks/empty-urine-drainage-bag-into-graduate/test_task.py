"""Offline checks of the drainage-bag scorer, the drain model and the demo geometry, no simulator. Link states come from
the task's own forward kinematics (fk) for chosen joint angles; joint angles are set as joint states, as the simulator
does each control step. Covers: doing nothing; the start layout (the spout up in its sleeve, no flow from it even with
the valve open, the graduate clear of the drain spot); a full drain (0.75, then 1.0 once the valve is closed, the spout
is back up in its sleeve and both hands are clear for 1 s); the drain model against Torricelli; placement robustness;
a missed stream; every touch case and cheat (rim, inner wall, floor, pour-spout lip, knee, swipe, tilt, gutter, a hand
carrying lipstick); the sleeve held by friction only, its readout and its cheats; no joint law pulling the tube or the
valve; the valve left open; the spout left out; a dropped tube; a tipped or shaken graduate; the stream into a hand;
the pan; the arms' clearances on their collision hulls; the reset order; the thresholds in the text.

    python3 test_task.py
"""
import json
import math
from pathlib import Path

import numpy as np
from yam_arena.demo import grip_rotation
from yam_arena.task_file import ObjectState, load

HERE = Path(__file__).parent
M = load(HERE / "task.py").module
import liquidkit as lk  # noqa: E402
import yamkit as kit  # noqa: E402

AWAY = {"left_gripper": (0.15, 0.30, M.TOP + 0.30), "right_gripper": (0.15, -0.30, M.TOP + 0.30)}
DRAIN = (M.DRAIN_A, M.DRAIN_PHI - M.DRAIN_A)
STOW = (M.STOW_A, M.STOW_B)


def check(cond, what):
    print(("ok   " if cond else "FAIL ") + what)
    if not cond:
        raise AssertionError(what)


def st(pos, rot=np.eye(3)):
    p = np.asarray(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.asarray(lk.quat(np.asarray(rot, float)), float), lin_vel=np.zeros(3),
                       ang_vel=np.zeros(3), low=p - 0.05, high=p + 0.05)


def rz(a):
    return M.rot_axis("Z", a)


def fresh():
    kit._EPISODE["count"] += 1
    kit._EPISODE["last"] = None


class Trial:
    """A hand-built trial: the tube at joint angles (stub, tube), the valve at `valve` rad, the graduate where we put it,
    the left hand on the collar or away."""

    def __init__(self):
        fresh()
        self.a, self.b = STOW
        self.v = 0.0
        self.gpos = np.array([*M.GR_START, M.FLOOR])
        self.grot = np.eye(3)
        self.hand_on_collar = False
        self.s = 0.0
        self.n = 0

    def at(self, ab, valve=None, gpos=None, grot=None):
        self.a, self.b = ab
        if valve is not None:
            self.v = valve
        if gpos is not None:
            self.gpos = np.asarray(gpos, float)
        if grot is not None:
            self.grot = np.asarray(grot, float)
        return self

    def objects(self):
        f = M.fk(a=self.a, b=self.b, v=self.v)
        o = {"drainbag": st(M.STAND_POS), "graduate": st(self.gpos, self.grot)}
        for k in ("bag", "tube_x", "tube_a", "tube_b", "valve"):
            o[f"drainbag/{k}"] = st(*f[k])
        opening = {}
        for g, p in AWAY.items():
            o[g] = st(p, grip_rotation(0.0))
            opening[g] = 1.0
        if self.hand_on_collar:
            o["left_gripper"] = st(f["collar"], M.collar_turn(self.a + self.b, 0.0) @ grip_rotation(0.0))
            opening["left_gripper"] = (M.COLLAR_R - 0.002) / M.FINGER_HALF_OPEN
        kit._OPENING.update(opening)
        return o

    def step(self, n=1):
        for _ in range(n):
            for key, q in (("tube_a", self.a), ("tube_b", self.b), ("valve", self.v)):
                kit._JOINTS[f"drainbag/{key}"] = kit.JointState(q, 0.0, False)
            kit._EPISODE["last"] = self.n
            self.n += 1
            o = self.objects()
            M.success(o)
            self.s = M.score(o)
        return self.s


PLACE = np.array([*M.graduate_place(), M.FLOOR])
HOLD = M.HOLD_STEPS + 2


def drain_all(t, gpos=PLACE, grot=np.eye(3), seconds=None):
    t.at(DRAIN, valve=M.VALVE_OPEN_Q, gpos=gpos, grot=grot)
    return t.step(round((seconds or (M.drain_seconds() + 3.0)) / M.DT))


def test_nothing_done():
    t = Trial()
    check(t.step(30) == 0.0, "nothing done scores 0")
    check(M._L.graduate_ml() == 0.0 and M._L.spilled_ml() == 0.0, "no flow with the valve closed")
    check(abs(M._L.bag / M.ML - M.START_ML) < 1e-9, "the bag starts with 600 mL")


def test_start_layout():
    t = Trial()
    t.at(STOW, valve=M.VALVE_OPEN_Q)
    t.step(40)
    check(M._L.bag / M.ML == M.START_ML and M._L.spilled_ml() == 0.0,
          "the stowed spout points up above the bag's level: even opened it does not flow")
    f = M.fk()
    level = f["tube_a"][0][2] + M.bag_depth(M.START_ML * M.ML)
    check(f["tip"][2] - level > 0.005, f"the stowed spout is {1000 * (f['tip'][2] - level):.0f} mm above the full level")
    t2 = Trial()
    check(M.in_holder(t2.objects()), "the spout starts all the way up in its sleeve")
    t2.at((M.PULL_A, M.sleeve_phi(M.PULL_A) - M.PULL_A))
    check(not M.in_holder(t2.objects()), "pulled down 20 degrees, it is out")
    check(np.linalg.norm(np.array(M.GR_START) - PLACE[:2]) > 0.15, "the graduate starts away from the drain spot")
    check(not M.touches(t.objects()), "no touch at the start")


def test_full_drain():
    t = Trial()
    t.hand_on_collar = True
    drain_all(t)
    g, s = M._L.graduate_ml(), M._L.spilled_ml()
    check(g >= M.FULL_ML and s <= M.CLEAN_ML, f"the demo's aim drains the bag into the graduate ({g:.1f} mL, {s:.2f} "
                                             f"spilled)")
    check(abs(g + s + M._L.bag / M.ML - M.START_ML) < 1e-6, "volume is conserved")
    check(t.s == 0.5, "drained with the valve still open scores 0.5")
    t.at(DRAIN, valve=0.0)
    check(t.step(5) == 0.75, "valve closed, tube still out: 0.75")
    t.at(STOW)
    check(t.step(HOLD) == 0.75, "spout back in its sleeve but the hand still on the collar: 0.75")
    t.hand_on_collar = False
    check(t.step(2) == 0.75, "released: not yet held 1 s")
    check(t.step(HOLD) == 1.0, "released, sleeved, closed and still for 1 s: 1.0")


def test_drain_rate():
    f = M.fk(a=DRAIN[0], b=DRAIN[1])
    head = f["tube_a"][0][2] + M.bag_depth(M.START_ML * M.ML) - f["tip"][2]
    q = M.CD * M.OUTLET_A * math.sqrt(2 * 9.81 * head) / M.ML
    check(20.0 < q < 40.0, f"the full bag starts draining at {q:.1f} mL/s (Torricelli, Cd 0.61, 6 mm bore)")
    check(15.0 < M.drain_seconds() < 60.0, f"it empties in {M.drain_seconds():.0f} s")
    t = Trial()
    t.at(DRAIN, valve=math.copysign((M.VALVE_OPEN + math.radians(M.VALVE_CLOSED_DEG)) / 2, M.VALVE_OPEN_Q), gpos=PLACE)
    t.step(20)
    half = M._L.graduate_ml()
    t2 = Trial()
    t2.at(DRAIN, valve=M.VALVE_OPEN_Q, gpos=PLACE)
    t2.step(20)
    check(abs(half / M._L.graduate_ml() - 0.5) < 0.05, "a half-open valve passes half the flow")
    check(M.valve_open_fraction(-math.radians(M.VALVE_CLOSED_DEG) + 0.01) == 0.0, "within 10 degrees: closed")


def test_placement_robust():
    results = []
    for yaw in (0.0, math.pi / 2, math.pi, -2.0):
        for dx, dy in ((0.0, 0.0), (0.01, 0.0), (-0.01, 0.0), (0.0, 0.015), (0.0, -0.015)):
            t = Trial()
            drain_all(t, gpos=PLACE + [dx, dy, 0.0], grot=rz(yaw))
            results.append((M._L.graduate_ml(), M._L.spilled_ml(), t.s))
    gs = [r[0] for r in results]
    check(max(gs) - min(gs) < 1e-6 and all(r[1] == 0.0 for r in results),
          "the graduate turned any way or moved 1-1.5 cm: the same volume caught, nothing spilled")


def test_missed_stream():
    t = Trial()
    drain_all(t, gpos=PLACE + [0.12, 0.0, 0.0])
    check(M._L.spilled_ml() > M.SPILL_MAX_ML and t.s == 0.0, "the graduate out of the stream's way: spilled, 0")


DOWN = (math.pi / 2, -math.pi / 2)  # the stub out toward the robot, the tube hanging straight down from it


def gap_at(t, gpos, grot=None):
    """The tube's gap to the graduate's whole collider set (coated or not)."""
    t.gpos = np.asarray(gpos, float)
    if grot is not None:
        t.grot = np.asarray(grot, float)
    return float(M.graduate_distance(M.tube_local(t.objects())).min())


def place_at_gap(t, start, direction, target, grot=None, span=0.2):
    """Move the graduate from `start` along `direction` (toward the spout) until the spout's gap is `target`."""
    start, direction = np.asarray(start, float), np.asarray(direction, float)
    lo, hi = 0.0, span
    for _ in range(60):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if gap_at(t, start + mid * direction, grot) > target else (lo, mid)
    g = gap_at(t, start + lo * direction, grot)
    check(abs(g - target) < 2e-5, f"placed at a {1000 * target:.2f} mm gap ({1000 * g:.3f})")
    return start + lo * direction


def touch_case(ab, start, direction, what, grot=None, span=0.2, coated=True):
    """On the coated surface (rim top, inside wall, inside floor): at a 0.3 mm gap the tube touches; at 0.7 mm (just
    outside the 0.5 mm contact band) it does not. On the bare outside wall or handle no gap is a touch (the real trial
    leaves them uncoated: the hands carry the graduate there)."""
    for gap, want in ((0.0003, True), (0.0007, False), (0.0, True), (-0.0005, True)):
        want = want and coated
        t = Trial()
        t.at(ab, valve=0.0)
        pos = place_at_gap(t, start, direction, gap, grot, span)
        t.at(ab, gpos=pos)
        t.step(1)
        check(M._L.touched == want, f"{what}, {1000 * gap:.1f} mm gap: {'a touch' if want else 'no touch'}")


def tip_of(ab):
    return M.fk(a=ab[0], b=ab[1])["tip"]


def test_touch_rim():
    # the draining pose; the graduate rises straight up under the spout's collar, its rim first
    f = M.fk(a=DRAIN[0], b=DRAIN[1])
    rim_near = f["tip"] - np.array([-(M.GR_T + M.GR_WALL / 2), 0.0, 0.0])
    start = np.array([rim_near[0], rim_near[1], M.FLOOR - 0.15])
    touch_case(DRAIN, start, (0.0, 0.0, 1.0), "the rim rising to the collar")


def test_touch_inner_wall_and_floor():
    tip = tip_of(DOWN)
    # the tube hanging straight down inside the graduate, 6 cm above the floor: the graduate slides sideways
    start = np.array([tip[0], tip[1], tip[2] - M.GR_FLOOR - 0.06])
    touch_case(DOWN, start, (0.0, -1.0, 0.0), "the collar's side against the inner wall (low)", span=0.06)
    start = np.array([tip[0], tip[1], tip[2] - M.GR_FLOOR - 0.11])
    touch_case(DOWN, start, (1.0, 0.0, 0.0), "the collar's side against the inner wall (near the rim)", span=0.065)
    # the spout's mouth coming down onto the inside floor
    start = np.array([tip[0], tip[1], tip[2] - M.GR_FLOOR - 0.03])
    touch_case(DOWN, start, (0.0, 0.0, 1.0), "the spout's mouth onto the inside floor", span=0.033)


def test_bare_outer_wall_and_handle():
    # the tube held out level toward the robot, the graduate coming at its mouth from the robot's side so the spout's
    # end meets the outside wall 7 cm below the rim (the flared rim passes above the tube): the bare outside wall
    level = (math.pi / 2, 0.0)
    tip = tip_of(level)
    start = np.array([tip[0] - 0.14, tip[1], tip[2] - (M.GR_RIM_Z - 0.07)])
    touch_case(level, start, (1.0, 0.0, 0.0), "the spout against the bare outside wall", span=0.09, coated=False)
    # the handle (toward -y in the graduate's frame) swung under the hanging spout's mouth
    tip = tip_of(DOWN)
    hx, hy, hz = M.HANDLE_BAR_C
    start = np.array([tip[0] - hx, tip[1] - hy, tip[2] - (hz + M.HANDLE_BAR[2] / 2) - 0.05])
    touch_case(DOWN, start, (0.0, 0.0, 1.0), "the spout on the bare handle", span=0.051, coated=False)


def test_touch_graduate_tilted_to_spout():
    # the graduate picked up, tipped 20 degrees and raised to meet the stream: its rim comes up to the collar
    for deg in (20.0, -20.0):
        t = Trial()
        t.at(DRAIN, valve=M.VALVE_OPEN_Q)
        grot = M.rot_axis("Y", math.radians(deg))
        f = M.fk(a=DRAIN[0], b=DRAIN[1])
        rim = grot @ np.array([-(M.GR_T + M.GR_WALL / 2), 0.0, M.GR_RIM_Z])  # its rim's top, right under the spout
        start = f["tip"] - rim - np.array([0.0, 0.0, 0.15])
        pos = place_at_gap(t, start, (0.0, 0.0, 1.0), 0.0003, grot)
        t.at(DRAIN, gpos=pos, grot=grot)
        t.step(1)
        check(M._L.touched, f"the graduate tipped {deg:.0f} degrees and raised against the spout is a touch")
        t.at(DRAIN, gpos=PLACE, grot=np.eye(3))
        t.step(round((M.drain_seconds() + 3) / M.DT))
        t.at(STOW, valve=0.0)
        check(t.step(HOLD) <= M.TOUCH_CAP, "set back upright and drained in full: still capped at 0.25")


def test_touch_whole_trial():
    t = Trial()
    f = M.fk(a=DRAIN[0], b=DRAIN[1])
    rim_near = f["tip"] - np.array([-(M.GR_T + M.GR_WALL / 2), 0.0, 0.0])
    t.at(DRAIN, valve=0.0)
    pos = place_at_gap(t, (rim_near[0], rim_near[1], M.FLOOR - 0.15), (0.0, 0.0, 1.0), 0.0)
    t.at(DRAIN, gpos=pos)
    t.step(1)
    check(M._L.touched, "one step resting on the rim is a touch")
    t.at(DRAIN, valve=M.VALVE_OPEN_Q, gpos=PLACE)
    t.step(round((M.drain_seconds() + 3) / M.DT))
    t.at(STOW, valve=0.0)
    check(t.step(HOLD) == M.TOUCH_CAP, "a touch early in the trial caps a full, clean drain at 0.25")
    t2 = Trial()
    drain_all(t2, gpos=PLACE)
    t2.at(DRAIN, valve=0.0)
    t2.step(5)
    low = None
    for da in np.radians(np.arange(0.0, 60.0, 0.1)):  # the stub lowered: the spout dips into the mouth
        ab = (DRAIN[0] - da, DRAIN[1])
        if M.tube_gap(t2.at(ab).objects()) <= 0.0003:
            low = ab
            break
    check(low is not None, "lowering the tube brings the spout onto the graduate")
    t2.at(low)
    t2.step(1)
    t2.at(STOW)
    check(t2.step(HOLD) == M.TOUCH_CAP, "a touch after the bag is empty (putting the tube away) caps it too")
    t3 = Trial()
    t3.at(DRAIN, valve=0.0, gpos=PLACE)
    t3.step(5)
    check(not M._L.touched and M.tube_gap(t3.objects()) > 0.008, "the demo's draining pose: no touch, 8 mm or more clear")


def test_touch_swept_between_steps():
    # one control step the spout is 3 mm outside the wall, the next 3 mm inside it: it passed through the rim
    tip = tip_of(DOWN)
    t = Trial()
    t.at(DOWN, valve=0.0)
    rho_rim = M.GR_T + M.GR_WALL + M.COLLAR_R
    z = tip[2] - M.GR_RIM_Z + 0.004  # the spout's mouth 4 mm below the rim
    out_pos = np.array([tip[0] - (rho_rim + 0.003), tip[1], z])
    in_pos = np.array([tip[0] - (M.GR_T - M.COLLAR_R - 0.004), tip[1], z])
    check(gap_at(t, out_pos) > 0.002 and gap_at(t, in_pos) > 0.002, "both ends of the swipe are clear of the wall")
    t.at(DOWN, gpos=out_pos)
    t.step(1)
    t.at(DOWN, gpos=in_pos)
    t.step(1)
    check(M._L.touched, "a swipe over the rim within one control step is a touch")
    a = M.tube_local(t.at(DOWN, gpos=out_pos).objects())
    b = M.tube_local(t.at(DOWN, gpos=in_pos).objects())
    check(M.touched_between(a, b), "the sweep finds the crossing")
    c = M.tube_local(t.at(DOWN, gpos=in_pos + [0.0, 0.0, -0.01]).objects())
    check(not M.touched_between(b, c), "a move that stays clear is not a touch")


def test_tube_over_the_rim():
    # the tube laid over the rim like a gutter with the spout dipped inside the mouth: the tube touches the graduate
    f = M.fk(a=DRAIN[0], b=DRAIN[1])
    mid = (f["tube_b"][0] + f["collar"]) / 2 + np.array([0.0, 0.0, -M.TUBE_R])  # under the tube, halfway along it
    t = Trial()
    t.at(DRAIN, valve=M.VALVE_OPEN_Q)
    rim = np.array([M.GR_T + M.GR_WALL / 2, 0.0, M.GR_RIM_Z])  # the graduate's rim on its +x side under that point
    start = mid - rim - np.array([0.0, 0.0, 0.05])
    pos = place_at_gap(t, start, (0.0, 0.0, 1.0), 0.0, span=0.08)
    t.at(DRAIN, gpos=pos)
    t.step(1)
    check(M._L.touched, "the tube resting on the rim is a touch (any part of the tubing)")
    t.step(round((M.drain_seconds() + 3) / M.DT))
    t.at(STOW, valve=0.0)
    check(t.step(HOLD) <= M.TOUCH_CAP, "draining through the gutter of the tube on the rim: capped at 0.25")


def test_jet_up_out_of_the_mouth():
    # the stub hanging down and the tube folded back up: the spout points up, below the bag's level, so it flows; held
    # with its mouth inside the graduate's mouth below the rim, its jet rises out and comes down outside (leaning 34
    # degrees: at 23 degrees the 15 mm spout's slower jet falls partly back into the mouth, which is caught, as it would be)
    ab = (0.0, math.pi - 0.6)
    tip = M.fk(a=ab[0], b=ab[1])["tip"]
    t = Trial()
    t.at(ab, valve=M.VALVE_OPEN_Q, gpos=np.array([tip[0], tip[1], tip[2] - M.GR_RIM_Z + 0.01]))
    t.step(10)
    check(M._L.bag < M.START_ML * M.ML and M._L.graduate_ml() == 0.0 and M._L.spilled_ml() > 0.0,
          "its jet, thrown up out of the mouth, is not caught")


def test_stream_on_a_hand_over_1s():
    # a hand in the stream briefly at low flow: under 1 s is spill only; over 1 s in all scores 0 even if little spills
    f = M.fk(a=DRAIN[0], b=DRAIN[1])
    head = f["tube_a"][0][2] + M.bag_depth(M.START_ML * M.ML) - f["tip"][2]
    d = (f["tip"] - f["collar"]) / np.linalg.norm(f["tip"] - f["collar"])
    mid = M.arc_points(f["tip"], math.sqrt(2 * M.G * head) * d, M.FLOOR + M.GR_RIM_Z, n=20)[10]
    half = math.copysign(math.radians(M.VALVE_CLOSED_DEG) + 0.05, M.VALVE_OPEN_Q)  # barely open: a thin stream
    try:
        t = Trial()
        t.at(DRAIN, valve=half, gpos=PLACE)
        M._HULLS["now"] = [("left_robot/tip_left", _Hull(mid - 0.015, mid + 0.015))]
        t.step(round(1.5 / M.DT))
        M._HULLS["now"] = None
        check(M._L.hand_steps * M.DT > M.HAND_HIT_S and M._L.spilled_ml() <= M.SPILL_MAX_ML and t.step(1) == 0.0,
              f"a thin stream on a hand for 1.5 s ({M._L.spilled_ml():.1f} mL spilled): 0")
        t2 = Trial()
        t2.at(DRAIN, valve=half, gpos=PLACE)
        M._HULLS["now"] = [("left_robot/tip_left", _Hull(mid - 0.015, mid + 0.015))]
        t2.step(round(0.5 / M.DT))
        M._HULLS["now"] = None
        check(M._L.hand_steps * M.DT <= M.HAND_HIT_S, "half a second on a hand: under the 1 s rule")
    finally:
        M._HULLS["now"] = None


def test_fingers_on_the_collar_do_not_block():
    # a hull wrapped round the collar beside its mouth (a pinch) does not stop the water leaving the bore
    f = M.fk(a=DRAIN[0], b=DRAIN[1])
    c = f["collar"]
    side = np.cross(f["tip"] - f["collar"], [0.0, 1.0, 0.0])
    side /= np.linalg.norm(side)
    pad = c + side * (M.COLLAR_R + 0.004)
    try:
        M._HULLS["now"] = [("right_robot/tip_left", _Hull(pad - 0.004, pad + 0.004))]
        t = Trial()
        drain_all(t)
        check(M._L.hand_ml == 0.0 and M._L.spilled_ml() <= M.CLEAN_ML, "a pad beside the collar: the stream is not blocked")
    finally:
        M._HULLS["now"] = None


def test_demo_aim_follows():
    worst = 0.0
    center = np.array(M.graduate_place())
    for frac in (0.4, 0.5, 0.75, 1.0):
        for (tk, a, phi) in M.aim_schedule():
            v = M.predicted_volume(min(tk, M.T_CLOSING - 2.0) - M.T_FLOW, frac)
            if v < 0.03 * M.START_ML * M.ML:
                continue
            worst = max(worst, float(np.linalg.norm(M.landing(a, phi, v) - center)))
    check(worst <= M.GR_T - 0.015, f"the following stream lands within {1000 * worst:.1f} mm of the mouth's center, 15 mm "
                                   f"or more inside its {1000 * M.GR_T:.1f} mm radius (40-100 % open)")


_HULL_CACHE = {}


def _arm_hulls_over_demo(t0=0.0, t1=math.inf, step=0.25):
    """(time, side, link, hull) of both arms' collision hulls at the planned joint angles, every `step` s, on the
    station's arm bases (computed once per step size)."""
    if step not in _HULL_CACHE:
        _HULL_CACHE[step] = list(_arm_hulls_over_demo_raw(step))
    return [x for x in _HULL_CACHE[step] if t0 <= x[0] <= t1]


def _near_box(h, lo, hi):
    v = h.vertices
    return not (np.any(v.max(0) < lo) or np.any(v.min(0) > hi))


def _arm_hulls_over_demo_raw(step):
    import yam_arena.demo as demo
    from yam_arena import analysis as an
    from yam_arena import station
    d = M.demo()
    for side in ("left", "right"):
        base = station.BASES[side]
        plan = demo.ArmPlan(d[side], base)
        k = 0
        while k * 0.05 <= plan.duration:
            t = k * 0.05
            act, _ = plan.action7(t)
            if abs(t / step - round(t / step)) < 1e-6:
                q = {f"joint{i + 1}": float(act[i]) for i in range(6)}
                q["joint7"] = q["joint8"] = float(act[6]) * demo.FINGER_OPEN
                for n, h in an.world_hulls(an.fk_link_poses_world(q, base)):
                    yield t, side, n, h
            k += 1


def test_arms_clear_the_pan():
    # both arms' collision hulls (bases included) over the whole scripted plan stay clear of the pan's walls and lip
    walls = [b for b in M.PAN if b.name != "pan_floor"]
    worst, where = math.inf, None
    for t, side, n, h in _arm_hulls_over_demo():
        v = h.vertices
        for w in walls:
            c, hs = np.asarray(w.pos), np.asarray(w.size) / 2
            gap = float(np.linalg.norm(np.maximum(np.abs(v - c) - hs, 0.0), axis=1).min())
            if gap < worst:
                worst, where = gap, (side, n, w.name, t)
    check(worst >= 0.02, f"the arms (bases included) stay {1000 * worst:.0f} mm or more from the pan's walls ({where})")


def _graduate_surface(step=0.003):
    """Points on the graduate's outside: its wall, rim and handle (prop frame)."""
    out = []
    for z in np.arange(0.0, M.GR_RIM_Z + 1e-9, step):
        r = M.GR_B + M.GR_WALL + (M.GR_T - M.GR_B) * max(z - M.GR_FLOOR, 0.0) / M.GR_H
        n = int(2 * math.pi * r / step)
        a = np.linspace(0, 2 * math.pi, n, endpoint=False)
        out.append(np.stack([r * np.cos(a), r * np.sin(a), np.full(n, z)], 1))
    for c, sz in ((M.HANDLE_ARM_C, M.HANDLE_ARM), (M.HANDLE_BAR_C, M.HANDLE_BAR)):
        g = [np.arange(-s / 2, s / 2 + 1e-9, step) for s in sz]
        pts = np.array([(x, y, z) for x in g[0] for y in g[1] for z in g[2]])
        out.append(pts + np.asarray(c))
    return np.concatenate(out)


def test_arms_clear_the_graduate_once_down():
    # from the moment the graduate is down and the carrying hand has let go, no arm comes within 8 mm of it (a lower
    # bound on the gap: each hull's farthest face plane)
    pts = _graduate_surface() + np.array([*PLACE[:2], M.FLOOR])
    worst, where = math.inf, None
    lo, hi = pts.min(0) - 0.02, pts.max(0) + 0.02
    for t, side, n, h in _arm_hulls_over_demo(t0=M.GRADUATE_DONE_S):
        if not _near_box(h, lo, hi):
            continue
        v = h.vertices
        near = np.all((pts >= v.min(0) - 0.02) & (pts <= v.max(0) + 0.02), axis=1)
        if not near.any():
            continue
        lb = (pts[near] @ h.planes[:, :3].T - h.planes[:, 3]).max(1).min()
        if lb < worst:
            worst, where = float(lb), (side, n, t)
    check(worst >= 0.008, f"after it is set down the arms stay {1000 * min(worst, 0.02):.0f} mm or more from the graduate ({where}; "
                          "20 mm means none came within 20 mm)")


def test_rim_in_the_knee():
    # the graduate's rim crest at the tube's knee (where the run folds up), up to 4 mm below its middle, at bends from
    # 0 to 120 degrees: inside the space a continuous tube fills, so a touch
    for a_deg in (0.0, 30.0, 60.0, 90.0, 120.0):
        for b_deg in (30.0, 60.0, 90.0, 120.0):
            ab = (math.radians(a_deg), math.radians(b_deg))
            knee = M.fk(a=ab[0], b=ab[1])["tube_b"][0]
            for dz in (0.0, -0.002, -0.004):
                t = Trial()
                t.at(ab, valve=0.0, gpos=np.array([knee[0] - (M.GR_T + M.GR_WALL / 2), knee[1],
                                                    knee[2] - M.GR_RIM_Z + dz]))
                t.step(1)
                check(M._L.touched, f"rim crest {1000 * -dz:.0f} mm under the knee (stub {a_deg:.0f}, tube "
                                    f"{b_deg:.0f} deg): a touch")


def test_hand_check_off_in_sim():
    # in the sim, any step without the arms' hulls (the stream and lipstick-on-a-hand checks could not run) bars 1.0
    try:
        M._HULLS["sim"] = True
        t = Trial()
        drain_all(t)
        t.at(STOW, valve=0.0)
        check(t.step(HOLD) == 0.75 and M._L.hand_check_off, "no arm hulls in the sim: at most 0.75")
    finally:
        M._HULLS["sim"] = False
    t = Trial()
    drain_all(t)
    t.at(STOW, valve=0.0)
    check(t.step(HOLD) == 1.0 and not M._L.hand_check_off, "offline (no sim): the full drain still scores 1.0")


def test_coated_distance_matches_the_coat():
    # the scorer's exact distance to the coated surface agrees with a dense sampling of that surface (0.5 mm apart)
    from scipy.spatial import cKDTree
    rng = np.random.default_rng(3)
    P = rng.uniform([-0.07, -0.07, 0.0], [0.07, 0.07, M.GR_RIM_Z + 0.015], (6000, 3))
    d = M.coated_distance(P)
    ref = cKDTree(M._coated_samples(0.0005)).query(P)[0]
    check(np.all(d <= ref + 1e-6) and np.all(ref - d <= 0.0006),
          f"exact coated distance within {1000 * float(np.max(ref - d)):.2f} mm of a 0.5 mm sampling, never above it")


class _Art:
    """Just enough of an Isaac Lab articulation to record what the reset hook writes."""

    def __init__(self, names):
        self.joint_names, self.calls = list(names), []

    def __getattr__(self, name):
        def rec(**kw):
            self.calls.append((name, {k: (v.tolist() if hasattr(v, "tolist") else v) for k, v in kw.items()}))
        return rec


def test_reset_springs_the_pad():
    # the reset hook puts the sleeve's pad spring on from the first physics step (the settle runs before any control
    # step, and yamkit leaves law-held joints limp until then); no other joint is driven
    import torch

    class _Data:
        joint_pos = torch.zeros((1, 7))

    art = _Art(["bag_joint"] + [f"{lk.name}_joint" for lk in M.BAG.links[1:]])
    art.data = _Data()
    M.spring_laws(art)
    j = art.joint_names.index("pad_joint")
    stiff = {c[1]["joint_ids"][0]: c[1]["stiffness"][0][0] for c in art.calls if c[0] == "write_joint_stiffness_to_sim_index"}
    target = {c[1]["joint_ids"][0]: c[1]["target"][0][0] for c in art.calls if c[0] == "set_joint_position_target_index"}
    check(set(stiff) == {j} and abs(stiff[j] - M.PAD_K) < 1e-6 and abs(target[j] + M.PAD_PRELOAD) < 1e-9,
          "only the pad is sprung at reset, at its spring and preload")


def test_no_joint_pulls_the_tube_or_valve():
    # nothing pulls the tube, the collar or the bag anywhere: their joints carry damping only (no law, spring, latch,
    # stay, drag or follow); the only law is the sleeve pad's plain spring, which presses only on what is in the sleeve
    for lk in M.BAG.links:
        quiet = lk.spring == 0.0 and lk.latch is None and lk.stay == 0.0 and lk.drag == 0.0 and lk.follow is None
        if lk.name == "pad":
            check(quiet and isinstance(lk.law, M.Spring), "the pad: a plain spring, nothing else")
        else:
            check(quiet and lk.law is None, f"{lk.name}: no law, spring, latch or hold (damping {lk.damping})")
    check(M.BAG.self_collide and M.BAG.no_collide == (("bag", "tube_a"),),
          "the bag's parts collide with each other (only the run in its outlet is excused)")


def test_valve_stays_where_it_is_left():
    # from a sim run of task.py 31b81c30832f, damping only (0.01 N*m*s/rad): the closed collar turned itself fully open while
    # the tube was pulled out and swung up (30.8 mL spilled before the spout was over the mouth), then, let go open, crept
    # shut in ~28 s (189 mL left in the bag). rec-152711's detent (0.02 N*m/rad, 0.15 rad band, up to 3 mN*m) kept it
    # put through the same moves. The seal's joint friction must hold at least that, yet stay well under what the valve
    # hand's pinch can turn.
    lk_ = next(lk for lk in M.BAG.links if lk.name == "valve")
    check(lk_.friction == M.VALVE_FRICTION > 0.0 and lk_.law is None, "the collar has joint friction and no law")
    creep = M.VALVE_DAMP * math.radians(90.0) / 28.0  # N*m that drove the creep
    hold = M.VALVE_FRICTION  # N*m: joint friction is a static friction effort in this Isaac Lab version
    check(hold > 5.0 * creep, f"hold {hold * 1e3:.1f} mN*m > 5 x the creep {creep * 1e3:.2f} mN*m")
    check(hold >= 0.02 * 0.15, f"hold {hold * 1e3:.1f} mN*m: at least the old detent's that kept the valve put")
    half = M.COLLAR_R - (M.GRASP_FINGER - M.finger_for(0.0)) / (M.finger_for(1.0) - M.finger_for(0.0))
    pinch = 5000.0 * half  # N: each finger aimed `half` inside the collar's side on the 5000 N/m gripper drive
    turn = 2 * 1.0 * pinch * M.COLLAR_R  # N*m the two pads can turn, dynamic grip friction 1.0
    check(hold < 0.25 * turn, f"twisting: {hold * 1e3:.0f} mN*m of friction < a quarter of the pinch's {turn * 1e3:.0f}")


def _collar_twists():
    rec = []
    real = M.collar_wp

    def spy(duration, a, phi, twist, finger, label, back=0.0):
        rec.append((label, twist, finger))
        return real(duration, a, phi, twist, finger, label, back=back)
    M.collar_wp = spy
    try:
        M.right_plan()
    finally:
        M.collar_wp = real
    return rec


def test_closing_never_turns_the_collar_back_open():
    # from the same sim run: after twisting the valve closed, the hand turned back 25 deg open-handed to grip again;
    # its open fingers dragged the free collar back to 42 deg open and the second press left it at 27 deg. From the
    # closing twist on, the hand's commanded turn only moves toward closed, and it never turns with the fingers open.
    rec = _collar_twists()
    k = next(i for i, r in enumerate(rec) if r[0] == "twist the valve closed")
    tw = [r[1] for r in rec[k:]]
    check(all(b >= a - 1e-9 for a, b in zip(tw, tw[1:])), f"closing turns only toward closed: {[round(math.degrees(t)) for t in tw]}")
    check(rec[k + 1][0] == "let go" and len({r[1] for r in rec[k:]}) == 1,
          "after the twist: let go and back away at the same turn")
    for (l0, t0, f0), (l1, t1, f1) in zip(rec, rec[1:]):
        if abs(t1 - t0) > 1e-9:
            check(f0 == f1 == M.GRASP_FINGER, f"'{l1}': turns the collar only while gripping it")
    check(not any("again" in r[0] or "turn the open hand" in r[0] for r in rec), "one press, no turn back")


def test_push_goes_deep_then_eases_off_before_letting_go():
    # from the same sim run: aiming 2 mm past the sleeve's top, the arm stopped 11 mm short (the collar on the pad's
    # chamfer, pad 0.6 mm back) and the let-go dropped the tube; with the 30 mm sleeve (task.py 42bca433c847), aiming
    # 15 mm past, it stopped 9 mm short. The push aims deeper than that shortfall and keeps on,
    # then eases off to a light push (never a pull down) before the fingers open.
    poses = M.left_tube_poses()
    k = next(i for i, p in enumerate(poses) if p[4] == "let go")
    pushes = [p for p in poses if p[4].startswith("push the spout") or p[4].startswith("keep pushing")]
    deepest = max(p[1] for p in pushes)
    check((deepest - M.STOW_A) * M.LA > 0.015 + 0.009, f"aims {1e3 * (deepest - M.STOW_A) * M.LA:.0f} mm past the top")
    check(poses[k - 2][4].startswith("keep pushing") and poses[k - 2][0] >= 1.0 and poses[k - 2][3] == M.TUBE_FINGER,
          "keeps pushing at least 1 s, gripping")
    ease = poses[k - 1]
    check(ease[4] == "ease off" and ease[3] == M.TUBE_FINGER and 0.0 < (ease[1] - M.STOW_A) * M.LA <= 0.003,
          "eases off to aim at most 3 mm past the top, still gripping")
    check(poses[k][1] == ease[1] and poses[k][3] == M.TUBE_WIDE, "lets go where it eased off")


def test_sleeve_holds_by_friction():
    # statics: the pad's push and the far wall's reaction give the collar more grip than the tube's weight asks, over
    # half to twice the pad's spring and friction 0.25 to 0.5; with a fifth of the spring it would slide out
    g = 9.81
    moment = g * (M.TUBE_KG[1] * M.LA / 2 + (M.TUBE_KG[2] + M.TUBE_KG[3]) * M.LA)  # about the outlet, stub level
    load = moment / M.LA  # what the sleeve must carry at the spout to keep the stub up
    defl = M.PAD_PRELOAD + (M.COLLAR_R - (M.PAD_FACE + (M.SLEEVE_R - M.COLLAR_R)))
    for k_scale in (0.5, 1.0, 2.0):
        for mu in (0.25, M.PLASTIC_MU[0], 0.5):
            grip = mu * 2 * M.PAD_K * k_scale * defl
            check(grip > 1.3 * load, f"spring x{k_scale}, friction {mu}: grip {grip:.2f} N against {load:.2f} N")
    check(M.PLASTIC_MU[0] * 2 * M.PAD_K * 0.2 * defl < load, "a fifth of the pad's spring would not hold it")


def test_mechanism_usd():
    # the generated articulation: every sleeve panel a real convex hull (no zero-size shape), the sleeve's top and pad
    # carry the plastic, every other collider the environment's grip material, and nothing binds over them at spawn
    import re
    txt = kit._mech_usda(M.BAG)
    check(not re.search(r"double radius = 0\.0\n|double height = 0\.0\n", txt), "no zero-size collider")
    n_hull = txt.count('physics:approximation = "convexHull"')
    check(n_hull == len(M._sleeve_colliders()) - 1 + 1, f"{n_hull} convex hulls: the sleeve's panels and the pad")
    check("enabledSelfCollisions = 1" in txt and txt.count("physics:filteredPairs") == 2, "self-collision, one pair off")
    check(M.BAG.material is not None, "the mechanism's own material is set, so the spawner binds none over it")
    # the pad's flat ends lie behind the bore's wall: a collar end only ever meets its chamfers
    check(M.PAD_FACE + M.PAD_LEAD >= M.SLEEVE_R - 1e-9, "the pad's chamfers reach the bore's wall")


def test_holder_readout_and_cheats():
    # "in its holder" is where the collar is: all the way up the bore. Near but under it, beside it, draped over the
    # sleeve, half in, or held there by a hand do not score the holder
    t = Trial()
    check(M.in_holder(t.objects()), "stowed, all the way up: in")
    base = t.objects()
    bp, _ = M._state_pose(base, "drainbag/valve")

    def moved(dx, dz, tilt=0.0):
        o = t.objects()
        p, R = M._state_pose(o, "drainbag/valve")
        o["drainbag/valve"] = st(p + np.array([dx, 0.0, dz]), M.rot_axis("Y", tilt) @ R)
        return M.in_holder(o)

    check(not moved(0.0, -0.010), "10 mm under the sleeve's top (half in): not in")
    check(not moved(0.0, -M.IN_SLEEVE_TOP - 0.001), "1 mm short of the home threshold: not in")
    check(moved(0.0, -M.IN_SLEEVE_TOP + 0.0005), "within the home threshold: in")
    check(not moved(0.0, -M.SPOUT_LEN), "only the spout in (the collar at the mouth): not in")
    check(not moved(0.0, -M.SLEEVE_DEPTH + 0.001), "the spout's tip resting just inside the mouth: not in")
    check(not moved(0.0, -0.030), "under the sleeve's mouth: not in")
    check(not moved(-0.016, -0.005), "beside the sleeve (draped against it): not in")
    check(not moved(0.0, 0.008), "above the sleeve (through its top): not in")
    check(moved(-(M.SLEEVE_R - M.COLLAR_R), -0.002), "against the far wall, 2 mm short of the top: in")
    check(not moved(-0.004, 0.0), "at the top's height but 4 mm off the sleeve's axis (through its wall): not in")
    check(not moved(0.0, 0.0, tilt=0.4), "cocked 23 degrees across the mouth: not in")
    # a hand holding the tube there: the trial cannot finish (both hands must be clear)
    t2 = Trial()
    t2.hand_on_collar = True
    drain_all(t2)
    t2.at(STOW, valve=0.0)
    check(t2.step(HOLD) == 0.75, "spout in its sleeve but a hand still on it: 0.75")


def test_sleeve_spec_is_consistent_and_home_is_reachable():
    # the drain's specification is one consistent set of sizes, and the full-score state is a real pose: the spout's
    # tip at the sleeve's closed top with 15 mm of the 22 mm collar inside and 7 mm showing (15 + 15 = 30)
    check(abs(M.SLEEVE_DEPTH - 0.030) < 1e-12 and abs(M.LV - 0.022) < 1e-12 and abs(M.SPOUT_LEN - 0.015) < 1e-12,
          "sleeve 30 mm deep, collar 22 mm, spout 15 mm")
    check(abs(M.COLLAR_HOME_IN + M.SPOUT_LEN - M.SLEEVE_DEPTH) < 1e-12 and abs(M.COLLAR_HOME_IN - 0.015) < 1e-12
          and abs(M.COLLAR_SHOWING - 0.007) < 1e-12, "fully home: 15 mm of collar inside, 7 mm showing")
    check(0.0 < M.COLLAR_MIN_IN < M.COLLAR_HOME_IN < M.LV, "the home threshold asks less collar than fully home has")
    check(M.SPOUT_R < M.PAD_FACE < M.COLLAR_R < M.SLEEVE_R < M.SLEEVE_R + M.SLEEVE_FLARE[1],
          "the spout passes the pad; the collar pushes it back; the bore and the mouth are wider than the collar")
    lo, hi = M.PAD_Z - M.PAD_SIZE[2] / 2, M.PAD_Z + M.PAD_SIZE[2] / 2
    check(M.SLEEVE_MOUTH + M.SLEEVE_FLARE[0] <= lo and hi <= M.SLEEVE_MOUTH + M.COLLAR_HOME_IN,
          "the pad lies on the collar's part in the straight bore when fully home")
    check(lo + M.PAD_RAMP < M.SLEEVE_MOUTH + M.COLLAR_MIN_IN and 2 * M.PAD_RAMP < M.PAD_SIZE[2],
          "at the home threshold the collar still reaches past the pad's lower chamfer onto its face")
    zs = [c.center[2] + p[2] for c in M._sleeve_colliders() if c.hull for p in c.hull]
    check(abs(min(zs) - M.SLEEVE_MOUTH) < 1e-9 and abs(max(zs) - M.SLEEVE_TOP) < 1e-9,
          "the sleeve's colliders run from its mouth to its closed top")
    t = Trial()
    below, inside, radial, up = M.holder_readout(t.objects())
    check(abs(below) < 1e-9 and abs(inside - M.COLLAR_HOME_IN) < 1e-9 and radial < 1e-9 and up,
          f"the stowed (start) pose: tip at the top, {1000 * inside:.1f} mm of collar inside, coaxial")
    check(M.in_holder(t.objects()), "the start pose is in its sleeve")
    # as the sim leaves it: the pad pushes the collar onto the far wall
    o = t.objects()
    p, R = M._state_pose(o, "drainbag/valve")
    o["drainbag/valve"] = st(p + np.array([-(M.SLEEVE_R - M.COLLAR_R), 0.0, 0.0]), R)
    check(M.in_holder(o), "fully home against the far wall (where the pad holds it): in")
    # pulled out, the spout's tip is below the mouth before the tube leans out
    a, phi = M.sleeve_path(M.STOW_A, M.PULL_A, 3)[-1]
    f = M.fk(a=a, b=phi - a)
    tip_b = f["bag"][1].T @ (f["tip"] - f["bag"][0])
    check(float(tip_b[2]) <= M.SLEEVE_MOUTH - 0.005, f"pulled out: the tip {1000 * (M.SLEEVE_MOUTH - tip_b[2]):.1f} mm "
          "below the mouth")


def test_sleeve_holds_the_hanging_tube_with_the_arms_clear():
    # fully home and let go, the pad presses the collar (it stands 1 mm back from rest) and the friction of the pad and
    # the far wall carries the tube; with the arms clear the trial scores 1.0, with no other support
    near_side = 2 * M.COLLAR_R - M.SLEEVE_R  # the collar's side toward the pad, from the axis, against the far wall
    check(near_side > M.PAD_FACE, f"the collar pushes the pad back {1000 * (near_side - M.PAD_FACE):.1f} mm")
    force = M.PAD_K * (M.PAD_PRELOAD + near_side - M.PAD_FACE)
    g = 9.81
    load = g * (M.TUBE_KG[1] * M.LA / 2 + (M.TUBE_KG[2] + M.TUBE_KG[3]) * M.LA) / M.LA
    check(M.PLASTIC_MU[0] * 2 * force > 1.3 * load, f"grip {M.PLASTIC_MU[0] * 2 * force:.2f} N > the tube's {load:.2f} N")
    t = Trial()
    drain_all(t)
    t.at(STOW, valve=0.0)
    check(t.step(HOLD) == 1.0, "fully home, arms clear: 1.0")


def test_released_needs_the_arms_off_the_tube():
    # in the sim, a palm or wrist propping the tube (its tool points well away) is not "released": no 1.0
    f = M.fk()
    knee = f["tube_b"][0]
    try:
        M._HULLS["sim"] = True
        M._HULLS["now"] = [("left_robot/link5", _Hull(knee - [0.02, 0.02, 0.02], knee + [0.02, 0.02, -M.TUBE_R + 0.0001])),
                           ("right_robot/link5", _Hull(np.array([5.0, 5.0, 5.0]), np.array([5.1, 5.1, 5.1])))]
        t = Trial()
        drain_all(t)
        t.at(STOW, valve=0.0)
        check(t.step(HOLD) == 0.75, "a hand propping the knee from below: not released, 0.75")
        M._HULLS["now"] = [("left_robot/link5", _Hull(np.array([5.0, 5.0, 5.0]), np.array([5.1, 5.1, 5.1]))),
                           ("right_robot/link5", _Hull(np.array([6.0, 6.0, 6.0]), np.array([6.1, 6.1, 6.1])))]
        t = Trial()
        drain_all(t)
        t.at(STOW, valve=0.0)
        check(t.step(HOLD) == 1.0, "both arms away: released, 1.0")
    finally:
        M._HULLS["sim"], M._HULLS["now"] = False, None


def test_arms_clear_the_bag_and_sleeve():
    # the arms (on the robot's collision hulls at the planned joint angles) stay clear of the bag and its sleeve, except
    # the left hand's fingers on the tube while it pushes the spout in (they hold the tube 8 cm under the sleeve)
    hook = np.asarray(M.STAND_POS) + np.asarray(M.HOOK)
    pts = []
    for z in np.arange(M.SLEEVE_MOUTH, M.SLEEVE_TOP + M.SLEEVE_WALL, 0.002):
        r = M.SLEEVE_R + M.SLEEVE_WALL + M.SLEEVE_FLARE[1]
        a = np.linspace(0, 2 * math.pi, 40, endpoint=False)
        pts.append(np.stack([M.SLEEVE_X + r * np.cos(a), r * np.sin(a), np.full(40, z)], 1))
    sleeve = np.concatenate(pts) + hook
    c, h = np.asarray(M._BAG_C) + hook, np.array([M.BAG_T, M.BAG_W, M.BAG_H]) / 2
    g = np.meshgrid(*[np.linspace(-1, 1, 9)] * 3)
    box = np.stack([x.ravel() for x in g], 1)
    box = box[np.abs(box).max(1) == 1] * h + c  # the bag's surface
    worst = {"sleeve": (math.inf, None), "bag": (math.inf, None)}
    for t, side, n, hull in _arm_hulls_over_demo():
        for key, P in (("sleeve", sleeve), ("bag", box)):
            if not _near_box(hull, P.min(0) - 0.02, P.max(0) + 0.02):
                continue
            lb = float((P @ hull.planes[:, :3].T - hull.planes[:, 3]).max(1).min())
            if lb < worst[key][0]:
                worst[key] = (lb, (side, n, t))
    for key, (w, where) in worst.items():
        check(w >= 0.002, f"the arms stay {1000 * min(w, 0.02):.1f} mm or more from the {key} ({where})")


def test_springs_go_on_after_yamkits_reset():
    # yamkit's reset hook (installed at the first control step) leaves law-held joints limp; the task's wrapper runs
    # after it, so the springs are what the settle sees on every later reset
    order = []

    class _Base:
        _yamkit_reset = True
        scene = {M.BAG.name: "art"}

        def reset(self, *a, **k):
            order.append("yamkit limp")

    base = _Base()
    real = M.spring_laws
    try:
        M.spring_laws = lambda art: order.append("sprung")
        M._spring_after_resets(base)
        M._spring_after_resets(base)  # wrapped once only
        base.reset()
    finally:
        M.spring_laws = real
    check(order == ["yamkit limp", "sprung"], f"reset order {order}")


def test_reset_order_over_three_trials_with_jitter():
    # the embodiment's reset: env.reset() (through every wrapper on base.reset), then its start jitter (root poses
    # only), then the settle steps. The task's make_env wrapper is innermost, yamkit's watcher wraps it at the first
    # control step, and the task re-wraps once more there: on every trial the last write before the settle is the springs
    log = []

    class _Base:
        scene = {M.BAG.name: "art"}

        def reset(self, *a, **k):
            log.append("env reset")

    base = _Base()
    real = M.spring_laws
    try:
        M.spring_laws = lambda art: log.append("sprung")
        inner = base.reset

        def prime_reset(*a, **k):  # what _prime installs at make_env
            out = inner(*a, **k)
            M.spring_laws(base.scene[M.BAG.name])
            return out

        base.reset = prime_reset

        def embodiment_reset():
            base.reset()
            log.append("jitter (root poses)")
            log.append("settle")

        def first_control_step():  # yamkit's watcher, then the task's own hook (both idempotent)
            if not getattr(base, "_yamkit_reset", False):
                base._yamkit_reset = True
                y_inner = base.reset

                def yamkit_reset(*a, **k):
                    out = y_inner(*a, **k)
                    log.append("yamkit limp")
                    return out

                base.reset = yamkit_reset
            M._spring_after_resets(base)

        for trial in range(3):
            log.clear()
            embodiment_reset()
            before_settle = [e for e in log[:log.index("settle")] if e in ("sprung", "yamkit limp")]
            check(before_settle and before_settle[-1] == "sprung",
                  f"trial {trial + 1}: the springs are the last joint write before the settle ({log})")
            first_control_step()
    finally:
        M.spring_laws = real


def test_spout_lip_is_coated():
    # the pour spout's lip (opposite the handle) is coated like the rest of the rim: the tube there is a touch, and
    # the tube resting in the spout's notch with the spout inside the mouth (a gutter) is capped at 0.25
    f = M.fk(a=DOWN[0], b=DOWN[1])
    tip = f["tip"]
    lip = math.atan2(-M.GR_SIDE, 0.0)  # opposite the handle
    for along in (0.0, 0.012):
        a = lip + along / M.GR_T
        rim = np.array([(M.GR_T + M.GR_WALL / 2) * math.cos(a), (M.GR_T + M.GR_WALL / 2) * math.sin(a), M.GR_RIM_Z])
        t = Trial()
        t.at(DOWN, valve=0.0)
        pos = place_at_gap(t, tip - rim - np.array([0.0, 0.0, 0.03]), (0.0, 0.0, 1.0), 0.0003, span=0.05)
        t.at(DOWN, gpos=pos)
        t.step(1)
        check(M._L.touched, f"the spout's mouth on the pour spout's lip ({1000 * along:.0f} mm along it): a touch")
    # the gutter at the lip: the graduate turned so its spout faces the tube, the tube lying on the lip, draining
    fd = M.fk(a=DRAIN[0], b=DRAIN[1])
    mid = (fd["tube_b"][0] + fd["collar"]) / 2 + np.array([0.0, 0.0, -M.TUBE_R])
    yaw = M.rot_axis("Z", -lip)  # the lip turned toward +x, under the tube
    t = Trial()
    t.at(DRAIN, valve=M.VALVE_OPEN_Q)
    rim = yaw @ np.array([(M.GR_T + M.GR_WALL / 2) * math.cos(lip), (M.GR_T + M.GR_WALL / 2) * math.sin(lip),
                          M.GR_RIM_Z])
    pos = place_at_gap(t, mid - rim - np.array([0.0, 0.0, 0.05]), (0.0, 0.0, 1.0), 0.0, grot=yaw, span=0.08)
    t.at(DRAIN, gpos=pos, grot=yaw)
    t.step(1)
    check(M._L.touched, "the tube lying in the pour spout's lip: a touch")
    t.step(round((M.drain_seconds() + 3) / M.DT))
    t.at(STOW, valve=0.0)
    check(t.step(HOLD) <= M.TOUCH_CAP, "draining along the lip's gutter: capped at 0.25")


def test_graduate_on_the_barrier():
    # drained cleanly into the graduate standing off the underpad (held or set beside it): 0.75, not 1.0
    off = np.array([PLACE[0], M.UNDERPAD.pos[1] - M.UNDERPAD.size[1] / 2 - 0.01, M.FLOOR])
    t = Trial()
    f = M.fk(a=DRAIN[0], b=DRAIN[1])
    tip_xy = f["tip"][:2]
    gpos = np.array([*(tip_xy + (PLACE[:2] - tip_xy)), M.FLOOR])  # the same spot under the spout ...
    check(M.on_underpad({"graduate": st(gpos)}) and not M.on_underpad({"graduate": st(off)}),
          "the drain spot is on the underpad; just past its edge is off it")
    t.at(DRAIN, valve=M.VALVE_OPEN_Q, gpos=gpos)
    t.step(round((M.drain_seconds() + 3) / M.DT))
    t.at(STOW, valve=0.0)
    n = int(np.ceil(np.linalg.norm(off - gpos) / 0.0005))  # slid off gently, 1 cm/s
    for k in range(1, n + 1):
        t.gpos = gpos + (off - gpos) * k / n
        t.step(1)
    check(M._L.spilled_ml() <= M.CLEAN_ML and t.step(HOLD) == 0.75,
          "the full graduate slid off the underpad at the end: 0.75")


def test_tilted_graduate_catch():
    # a graduate tipped 25 degrees: the stream counts only where it crosses the tipped mouth's own plane inside it
    f = M.fk(a=DRAIN[0], b=DRAIN[1])
    tip, d, head = M.tip_head(M.DRAIN_A, M.DRAIN_PHI, M.START_ML * M.ML)
    v = math.sqrt(2 * M.G * head) * d
    R = M.rot_axis("Y", math.radians(25.0))
    hits = 0
    for dx in np.arange(-0.08, 0.081, 0.004):
        gpos = PLACE + [dx, 0.0, 0.0]
        o = gpos + R @ np.array([0.0, 0.0, M.GR_FLOOR])
        mouth = o + R @ np.array([0.0, 0.0, M.GR_H])
        t = Trial()
        t.at(DRAIN, valve=M.VALVE_OPEN_Q, gpos=gpos, grot=R)
        t.step(1)
        hit = lk.ballistic(tip, v, mouth, R[:, 2])
        inside = hit is not None and float(np.hypot(*(R.T @ (hit[1] - o))[:2])) <= M.GR_T - M.MOUTH_MARGIN
        caught = M._L.graduate_ml() > 0.0
        if float((tip - mouth) @ R[:, 2]) > 0.0:
            check(caught == inside, f"tipped 25 deg, moved {1000 * dx:.0f} mm: caught {caught} = crosses the tipped "
                                    f"mouth inside its rim {inside}")
            hits += caught
    check(hits > 0, "some placements of the tipped graduate catch the stream")


def test_lipstick_carried_by_a_hand():
    # a fingertip that touches the coated rim and then the tube carries the red to it: a touch; one that touched only
    # the bare outside wall carries nothing
    f = M.fk(a=DRAIN[0], b=DRAIN[1])
    o = PLACE + np.array([0.0, 0.0, 0.0])
    rim_pt = o + np.array([M.GR_T + M.GR_WALL / 2, 0.0, M.GR_RIM_Z])
    out_pt = o + np.array([M.GR_T + M.GR_WALL + 0.0003, 0.0, M.GR_RIM_Z * 0.5])
    grip = f["tube_b"][0] + (f["collar"] - f["tube_b"][0]) * 0.3
    for name, first, want in (("the coated rim", rim_pt + [0.0, 0.0, 0.0003 + 0.004], True),
                              ("the bare outside wall", out_pt + [0.004, 0.0, 0.0], False)):
        try:
            t = Trial()
            t.at(DRAIN, valve=0.0, gpos=PLACE)
            M._HULLS["now"] = [("right_robot/tip_left", _Hull(first - 0.004, first + 0.004))]
            t.step(1)
            red = set(M._L.red)
            M._HULLS["now"] = [("right_robot/tip_left", _Hull(grip - 0.004 - M.TUBE_R, grip + 0.004 - M.TUBE_R))]
            t.step(1)
            check(M._L.touched == want and M._L.carried == want and bool(red) == want,
                  f"a fingertip on {name}, then on the tube: {'a touch' if want else 'no touch'} (red links {red})")
        finally:
            M._HULLS["now"] = None


def test_demo_hands_stay_clean():
    # at every checked pose of the scripted plan (every 0.25 s; while the graduate is carried it moves with the hand
    # that holds its handle, so the grip pose stands for the carry) no robot link comes within 5 mm of the coated
    # surface: the demo's hands never pick up lipstick (the right hand carries the graduate by its bare handle)
    if M.COATED_SAMPLES is None:
        M.COATED_SAMPLES = M._coated_samples()
    worst, where = math.inf, None
    for t, side, n, h in _arm_hulls_over_demo(step=0.25):
        moves = M.graduate_moves()
        ends = np.cumsum([w.duration for w in moves])
        labels = [w.label for w in moves]
        lift, down = ends[labels.index("grip")], ends[labels.index("set it on the underpad")]
        if lift < t < down:
            continue  # carried: the graduate moves with the hand that holds its handle; checked at both ends
        place = np.array([*M.GR_START, M.FLOOR]) if t <= lift else np.array([*PLACE[:2], M.FLOOR])
        pts = M.COATED_SAMPLES + place
        if not _near_box(h, place + [-0.08, -0.08, -0.01], place + [0.08, 0.08, M.GR_RIM_Z + 0.01]):
            continue
        v = h.vertices
        near = np.all((pts >= v.min(0) - 0.01) & (pts <= v.max(0) + 0.01), axis=1)
        if not near.any():
            continue
        lb = float((pts[near] @ h.planes[:, :3].T - h.planes[:, 3]).max(1).min())
        if lb < worst:
            worst, where = lb, (side, n, t)
    check(worst >= 0.005, f"the demo's arms stay {1000 * min(worst, 0.01):.1f} mm or more from the coated surface ({where}; 10 mm "
                          "means none came within 10 mm)")


def test_hover_inside_is_not_a_touch():
    # the spout held inside the mouth, below the rim, clear of everything: the stream goes straight in, no touch
    t = Trial()
    tip = tip_of(DOWN)
    gpos = np.array([tip[0], tip[1], tip[2] - M.GR_FLOOR - 0.10])
    t.at(DOWN, valve=M.VALVE_OPEN_Q, gpos=gpos)
    check(M.tube_gap(t.objects()) > 0.03, "the hovering spout is clear of the graduate")
    t.step(40)
    check(not M._L.touched and M._L.spilled_ml() == 0.0 and M._L.graduate_ml() > 0.0,
          "inside the mouth and not touching: caught, no touch")


def test_colliders_are_the_checked_shape():
    # every wall panel corner lies on the checked section to within the chord's 0.14 mm
    for c in M.GRADUATE.colliders:
        if c.hull is None:
            continue
        pts = np.asarray(c.hull) + np.asarray(c.center)
        d = M.graduate_distance(pts)
        check(np.all(np.abs(d) <= 0.00014), "panel corners within 0.14 mm of the checked wall")
        break
    n = sum(1 for c in M.GRADUATE.colliders if c.hull is not None)
    check(n == M.GR_PANELS, f"{n} wall panels round the axis")
    # chord middles of the inside face lie on the inside line: the collider's inside face is the checked one
    worst = 0.0
    for k in range(M.GR_PANELS):
        a = 2 * math.pi * (k + 0.5) / M.GR_PANELS
        for h in (0.001, M.GR_H / 2, M.GR_H - 0.001):
            r = M.GR_B + (M.GR_T - M.GR_B) * h / M.GR_H
            p = np.array([[r * math.cos(a), r * math.sin(a), M.GR_FLOOR + h]])
            worst = max(worst, abs(float(M.graduate_distance(p)[0])))
    check(worst < 1e-9, "every panel's inside face meets the checked inside line at its middle")
    # the drain tube's samples cover its whole surface finely: every point of it is within 0.4 mm of a sample
    from scipy.spatial import cKDTree
    rng = np.random.default_rng(1)
    worst = 0.0
    for key, r, z0, z1 in (("drainbag/tube_a", M.TUBE_R, 0.0, -M.LA), ("drainbag/tube_b", M.TUBE_R, 0.0, -M.LB),
                           ("drainbag/valve", M.COLLAR_R, 0.0, -M.LV), ("drainbag/valve", M.SPOUT_R, -M.LV, -M.TIP_L)):
        s = M.TUBE_SAMPLES[key]
        a = rng.uniform(0.0, 2 * math.pi, 9000)
        side = np.stack([r * np.cos(a[:5000]), r * np.sin(a[:5000]), rng.uniform(z1, z0, 5000)], 1)
        rr = r * np.sqrt(rng.uniform(0.0, 1.0, 4000))
        ends = np.stack([rr * np.cos(a[5000:]), rr * np.sin(a[5000:]), np.where(rng.random(4000) < 0.5, z0, z1)], 1)
        pts = np.concatenate([side, ends])
        if key == "drainbag/tube_b":  # and the knee's sphere
            u = rng.normal(size=(4000, 3))
            pts = np.concatenate([pts, M.TUBE_R * u / np.linalg.norm(u, axis=1, keepdims=True)])
        worst = max(worst, float(cKDTree(s).query(pts)[0].max()))
        z_lo = -M.TIP_L if key == "drainbag/valve" else z1
        check(abs(float(s[:, 2].max()) - max(0.0, M.TUBE_R if key == "drainbag/tube_b" else 0.0)) < 1e-4
              and abs(float(s[:, 2].min()) - z_lo) < 1e-9, f"{key}'s samples span its colliders")
    check(any(c.sphere for c in M.TUBE_B.colliders) and all(abs(c.height - L) < 1e-12 for c, L in
          ((M.TUBE_A.colliders[0], M.LA), (M.TUBE_B.colliders[0], M.LB), (M.VALVE_LINK.colliders[0], M.LV),
           (M.VALVE_LINK.colliders[1], M.SPOUT_LEN))),
          "the tube's colliders run end to end, with a sphere at the knee")
    check(worst <= 0.0004, f"every point of the drain tube's surface is within {1000 * worst:.2f} mm of a sample")


class _Hull:
    def __init__(self, lo, hi):
        lo, hi = np.asarray(lo, float), np.asarray(hi, float)
        self.vertices = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
        self.planes = np.array([[1, 0, 0, hi[0]], [-1, 0, 0, -lo[0]], [0, 1, 0, hi[1]], [0, -1, 0, -lo[1]],
                                [0, 0, 1, hi[2]], [0, 0, -1, -lo[2]]], float)


def test_stream_into_a_hand():
    # a hand (a 3 cm block) held in the stream's path: the water runs off it onto the floor
    f = M.fk(a=DRAIN[0], b=DRAIN[1])
    head = f["tube_a"][0][2] + M.bag_depth(M.START_ML * M.ML) - f["tip"][2]
    d = (f["tip"] - f["collar"]) / np.linalg.norm(f["tip"] - f["collar"])
    arc = M.arc_points(f["tip"], math.sqrt(2 * M.G * head) * d, M.FLOOR + M.GR_RIM_Z, n=20)
    mid = arc[10]
    try:
        M._HULLS["now"] = [("right_robot/tip_left", _Hull(mid - 0.015, mid + 0.015))]
        t = Trial()
        drain_all(t)
        check(M._L.hand_ml > M.SPILL_MAX_ML and M._L.spilled_ml() > M.SPILL_MAX_ML and t.s == 0.0,
              "a hand in the stream: the water splashes off it, spilled, 0")
        M._HULLS["now"] = [("right_robot/tip_left", _Hull(mid + [0.0, 0.05, -0.015], mid + [0.03, 0.08, 0.015]))]
        t = Trial()
        drain_all(t)
        check(M._L.hand_ml == 0.0 and M._L.spilled_ml() <= M.CLEAN_ML, "a hand 3.5 cm beside the stream: no effect")
    finally:
        M._HULLS["now"] = None


def test_jolted_graduate_spills():
    # the full graduate shaken 1 cm back and forth at 2.5 Hz (near its sloshing frequency) for 4 s slops over its rim
    t = Trial()
    drain_all(t)
    s0 = M._L.spilled_ml()
    t.at(DRAIN, valve=0.0)
    for k in range(80):
        t.gpos = PLACE + [0.01 * math.sin(2 * math.pi * 2.5 * k * M.DT), 0.0, 0.0]
        t.step(1)
    check(M._L.spilled_ml() - s0 > M.SPILL_MAX_ML and t.s == 0.0,
          f"shaken: {M._L.spilled_ml() - s0:.0f} mL slopped out, 0")
    t2 = Trial()
    drain_all(t2)
    s0 = M._L.spilled_ml()
    t2.at(DRAIN, valve=0.0)
    for k in range(60):  # slid 1 cm over 3 s, as a careful hand would
        t2.gpos = PLACE + [0.01 * k / 60, 0.0, 0.0]
        t2.step(1)
    check(M._L.spilled_ml() - s0 < 1e-6, "slid gently: nothing slops out")


def contained(tip, v):
    """A stream from tip with velocity v ends in the pan: it lands on the inside floor, or meets a wall's inside face
    below the wall's top (and runs down into the pan)."""
    x0, y0, x1, y1 = M.pan_inside()
    for pt in M.arc_points(tip, v, M.FLOOR, n=400):
        if not (x0 <= pt[0] <= x1 and y0 <= pt[1] <= y1):
            return pt[2] <= M.TOP + M.PAN_H
    return True


def test_pan_contains_the_scene():
    x0, y0, x1, y1 = M.pan_inside()
    check(abs((x1 - x0) - 0.7112) < 1e-9 and abs((y1 - y0) - 0.762) < 1e-9, "the pan's inside floor is 28 x 30 in")
    inside = lambda x, y, m=0.0: x0 + m <= x <= x1 - m and y0 + m <= y <= y1 - m  # noqa: E731
    rim = M.GR_T + M.GR_WALL
    for xy in (M.GR_START, PLACE[:2]):
        handle_end = xy[1] + M.GR_SIDE * (rim + M.HANDLE_ARM[1])
        check(inside(*xy, rim + 0.02) and inside(xy[0], handle_end, 0.02),
              f"the graduate at ({100 * xy[0]:.1f}, {100 * xy[1]:.1f}) cm, its handle included, is 2 cm or more inside "
              "the pan")
    u = M.UNDERPAD
    check(inside(u.pos[0] - u.size[0] / 2, u.pos[1] - u.size[1] / 2) and inside(u.pos[0] + u.size[0] / 2,
                                                                                   u.pos[1] + u.size[1] / 2),
          "the underpad lies inside the pan")
    check(inside(M.STAND_X + 0.105, M.BAG_Y + 0.11) and inside(M.STAND_X - 0.105, M.BAG_Y - 0.11),
          "the holder's base stands inside the pan")
    f = M.fk(a=0.0, b=0.0)  # a dropped tube hangs straight down: it drains onto the pan's floor
    check(inside(*f["tip"][:2], 0.05), "a dropped tube drains into the pan")
    t = Trial()
    t.at((0.0, 0.0), valve=M.VALVE_OPEN_Q, gpos=PLACE)
    t.step(5)
    land = M._L.stream[-1]
    check(inside(*land[:2], 0.05) and land[2] <= M.FLOOR + 1e-6, "its stream runs onto the pan's floor")
    for vol in (600.0, 450.0, 300.0, 150.0, 50.0):  # the stream at the draining pose with no graduate under it
        tip, d, head = M.tip_head(M.DRAIN_A, M.DRAIN_PHI, vol * M.ML)
        check(contained(tip, math.sqrt(2 * M.G * head) * d),
              f"a stream that misses the graduate ({vol:.0f} mL in the bag) ends in the pan")
    # the pan stands clear of the station's fixed parts: the arm bases and the top camera's stand foot
    from yam_arena import station
    for variant in station.TOP_CAMS:
        foot = station.camera_stand(station.top_camera(variant))["stand_foot"]
        fx, hx = foot[2][0], foot[1][0] / 2
        check(M.PAN_X0 - (fx + hx) >= 0.005, f"the pan clears the top camera stand's foot ({variant}) by "
                                             f"{1000 * (M.PAN_X0 - (fx + hx)):.0f} mm")


def test_left_open_and_out():
    t = Trial()
    drain_all(t)
    t.at(STOW)
    check(t.step(HOLD) == 0.5, "spout in its sleeve but the valve left open: 0.5")
    t2 = Trial()
    drain_all(t2)
    t2.at(DRAIN, valve=0.0)
    check(t2.step(HOLD) == 0.75, "valve closed, spout left out of its sleeve: 0.75")


def test_dropped_tube():
    t = Trial()
    t.at((0.0, 0.0), valve=M.VALVE_OPEN_Q, gpos=PLACE)
    t.step(round(10 / M.DT))
    check(M._L.spilled_ml() > M.SPILL_MAX_ML and t.s == 0.0, "a dropped tube hangs down and drains onto the floor: 0")


def test_tipped_graduate():
    t = Trial()
    drain_all(t)
    t.at(DRAIN, valve=0.0, grot=M.rot_axis("X", math.radians(80)))
    t.step(40)
    check(M._L.spilled_ml() > M.SPILL_MAX_ML and t.s == 0.0, "the full graduate knocked over: spilled, 0")


def test_new_trial():
    t = Trial()
    drain_all(t)
    Trial().step(2)
    check(M._L.graduate_ml() == 0.0 and abs(M._L.bag / M.ML - M.START_ML) < 1e-9 and not M._L.touched,
          "a new trial starts with a full bag and an empty graduate")


def test_demo_geometry():
    f = M.fk(a=DRAIN[0], b=DRAIN[1])
    gap = f["tip"][2] - (M.FLOOR + M.GR_RIM_Z)
    check(0.01 < gap < 0.06, f"the draining spout is {1000 * gap:.0f} mm above the rim")
    c = np.array(M.graduate_place())
    check(np.linalg.norm(f["tip"][:2] - c) < M.GR_T - 0.02, "the spout is over the graduate's mouth")
    t = Trial()
    t.at(DRAIN, valve=0.0, gpos=PLACE)
    o, _ = M.graduate_frame(t.objects())
    check(M.tube_gap(t.objects()) > 0.008, "the spout's surface stays 8 mm or more from the graduate")
    check(PLACE[0] + M.GR_T + M.GR_WALL < M.STAND_X + M.HOOK[0] - M.BAG_T / 2 - 0.05,
          "the graduate stands 5 cm or more in front of the bag")
    d = M.demo()
    dur = max(sum(w.duration for w in d[s]) for s in d)
    check(dur * 20 + 100 < M.RUN_LIMIT_STEPS, f"the demo ({dur:.0f} s) fits the run limit")
    labels = [w.label for w in d["left"]]
    t_swing = sum(w.duration for w in d["left"][:labels.index("raise the stub 1")])
    check(t_swing >= M.LEFT_DONE_S - 1e-9, "the tube swings down only after the graduate is down")
    rl = [w.label for w in d["right"]]
    t_open = sum(w.duration for w in d["right"][:rl.index("twist the valve open")])
    t_aim = sum(w.duration for w in d["left"][:labels.index("hold the aim while the valve is opened")])
    check(t_open >= t_aim - 1e-9, "the valve is opened only once the spout is aimed")
    t_close = sum(w.duration for w in d["right"][:rl.index("twist the valve closed")])
    t_up = sum(w.duration for w in d["left"][:labels.index("swing the tube back up 1")])
    check(t_up >= t_close + M.TWIST_S - 1e-9, "the tube swings back up only after the valve is closed")
    check(M.T_CLOSING - M.T_FLOW >= M.drain_seconds(upto=0.97, frac=M.HOLD_OPEN_FRAC),
          "the valve stays open until 97 % of the bag has passed even at 60 % open")
    turn = M.collar_turn(M.DRAIN_PHI, -M.VALVE_OPEN) @ M.collar_turn(M.DRAIN_PHI, 0.0).T
    ang = math.degrees(math.acos((np.trace(turn) - 1) / 2))
    check(abs(ang - 90.0) < 1e-6, "the twist turns the hand 90 degrees about the collar")
    m0 = M.collar_turn(M.DRAIN_PHI, 0.0)
    d_axis = np.array([-math.sin(M.DRAIN_PHI), 0.0, -math.cos(M.DRAIN_PHI)])
    check(abs(float((m0 @ np.array([0.0, 1.0, 0.0])) @ d_axis)) < 1e-9 and abs(float((m0 @ np.array([0, 0, -1.0]))
                                                                                     @ d_axis)) < 1e-9,
          "the fingers close and point square to the collar's axis")


def test_text_thresholds():
    j = json.loads((HERE / "task.json").read_text())
    readme = (HERE / "README.md").read_text()
    for s in ("570 mL", "600 mL", "5 mL", "20 mL", "300 mL", "100 mL", "10 degrees", "1 s", "0.25", "0.5 mm"):
        check(s in j["scoring"], f"task.json scoring states '{s}'")
        check(s in readme, f"README states '{s}'")
    # the sleeve's thresholds and specification, from task.py's own numbers
    mm = lambda v: f"{round(v * 1000):d} mm"  # noqa: E731
    for v in (M.IN_SLEEVE_TOP, M.COLLAR_MIN_IN, M.LV):
        check(mm(v) in j["scoring"] and mm(v) in readme, f"task.json scoring and README state '{mm(v)}'")
    for v in (M.SLEEVE_DEPTH, M.COLLAR_HOME_IN, M.COLLAR_SHOWING, M.SPOUT_LEN):
        check(mm(v) in j["setup"] and mm(v) in readme, f"task.json setup and README state '{mm(v)}'")
    for bad in ("both ends", "16 mm deep", "14 mm bore", "estimate", "measured"):
        check(bad not in j["setup"] + j["scoring"] + j["reset_notes"], f"task.json does not say '{bad}'")
    check("both ends" not in readme and "16 mm deep" not in readme, "README: no 'both ends' / '16 mm deep'")
    check(j["instruction"] == M.INSTRUCTION, "instruction matches")
    check("2,400 steps" in j["run_limit"] and M.RUN_LIMIT_STEPS == 2400, "run limit 2,400")
    check(len(j.get("context", "")) <= 1000, f"context at most 1,000 characters ({len(j.get('context', ''))})")
    check("Real-world evaluation (reproducible protocol)" in readme, "README has the reproducible real-world protocol")


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
    print("all pass")
