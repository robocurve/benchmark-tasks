"""A bar during service. Two 750 ml liquor bottles stand side by side on a rubber bar mat: on the robot's left the
well bottle that has just run dry, with a stainless speed pourer still in its neck; on the right its replacement,
full, cap already off. The pourer is a tapered steel spout on a finned stopper that is a friction fit in the neck.

Bar practice: when a well bottle runs out, the bartender takes its pourer and fits it to the new bottle. The stopper
grips the neck harder than the empty bottle weighs, so one hand has to hold the bottle down while the other draws
the pourer out; pulled one-handed, the bottle comes up off the bar with it. The pourer is then pushed into the new
neck until its collar sits on the lip; a 30 cm bottle goes over with a sideways push of about 1.5 N at the lip, so
the demo steadies the new bottle as well. A pourer left resting in the mouth falls out, or lets go in the middle
of a pour.

Scored from the poses: the pourer out of the empty bottle with that bottle still standing; its stopper started in
the new neck; pushed home; let go, both bottles standing, and a 3.5 N tug on the pourer not moving it. A bottle knocked
over at any moment ends in 0.

Mechanism parts come from mechkit.py (in this folder): the stopper's fit in each neck is a prismatic joint along the
bore with Coulomb friction, built with the scene; the one in the new neck closes when the stopper's lead enters it.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/speed-pourer-bottle-change/speed_pourer_bottle_change.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (multi-box parts, the stopper's friction fit and per-episode hooks, in memory)

from yam_arena import ik, station  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, READY  # noqa: E402
from yam_arena.constants import SHOULDER as ARM_J123, WRIST as ARM_J456  # noqa: E402  (the joint drives' gains)
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]  # the bar top

# Frame: x forward from the arm bases, y left, z up, metres. Arm bases at (0, +-0.305, 0.75).
MAT = (0.270, 0.000)  # centre of the bar mat
MAT_SIZE = (0.160, 0.300, 0.004)
BASE_Z = TOP + MAT_SIZE[2]  # the bottles stand on the mat
OLD = (0.270, 0.070)  # the empty bottle's axis
NEW = (0.270, -0.070)  # the new bottle's axis

# A 750 ml round liquor bottle. Part frame: origin at the centre of the base, +z up the axis. Round sections are
# regular octagons across the flats, two flats facing +-y.
BOTTLE_H = 0.295  # base to lip
BODY_D, BODY_H = 0.076, 0.190
SHOULDER = ((0.070, 0.005), (0.062, 0.005), (0.053, 0.005), (0.044, 0.005), (0.036, 0.005))  # (across, height) steps
NECK_Z0 = BODY_H + sum(h for _, h in SHOULDER)  # 0.215
NECK_OD, BORE = 0.029, 0.019
BEAD_OD, BEAD_Z0, BEAD_H = 0.034, 0.266, 0.008
EMPTY_KG = 0.420  # the glass
FULL_KG = 1.130  # glass plus 750 ml of 40 % spirit
SIDE_DEG = 67.5  # the empty bottle is turned so a pair of flats is square to a hand coming in 67.5 deg from +x
GLASS = (0.72, 0.84, 0.80)
AMBER = (0.62, 0.36, 0.12)
T8 = math.tan(math.pi / 8)


def _oct(d, z0, h, color=None):
    """A solid regular octagonal prism, ``d`` across the flats, from z0 up h: four boxes turned 45 deg apart."""
    return [mk.box((d, d * T8 + 0.0002, h), at=(0.0, 0.0, z0 + h / 2), yaw=k * math.pi / 4, color=color)
            for k in range(4)]


def _oct_tube(od, bore, z0, h, color=None):
    """An octagonal tube: eight wall boxes, ``od`` across the outer flats, ``bore`` across the inner flats."""
    t = (od - bore) / 2
    r = od / 2 - t / 2
    return [mk.box((t, od * T8 + 0.0002, h), at=(r * math.cos(k * math.pi / 4), r * math.sin(k * math.pi / 4),
                                                  z0 + h / 2), yaw=k * math.pi / 4, color=color) for k in range(8)]


def _bottle():
    out = _oct(BODY_D, 0.0, BODY_H)
    z = BODY_H
    for d, h in SHOULDER:
        out += _oct(d, z, h)
        z += h
    out += _oct_tube(NECK_OD, BORE, NECK_Z0, BOTTLE_H - NECK_Z0)
    out += _oct_tube(BEAD_OD, NECK_OD - 0.001, BEAD_Z0, BEAD_H)
    return out


# The speed pourer. Part frame: origin at the centre of the collar's underside (on the lip when seated), +z up
# the spout. Below the collar the finned stopper (rigid here, 1.5 mm clear of the bore all round) and its lead;
# above it the spout's base, gripped by the fingers, and the tapered spout, bent toward +x at the tip.
STOPPER_D, STOPPER_L = 0.016, 0.026
LEAD_D, LEAD_L = 0.011, 0.006
FIT_DEPTH = STOPPER_L + LEAD_L  # 32 mm of stopper below the collar
COLLAR_D, COLLAR_H = 0.026, 0.003
GRIP_D, GRIP_H = 0.013, 0.034  # the spout's base, the thick end of its taper
SPOUT = 0.008
SPOUT_L, BEND_L, BEND_DEG = 0.030, 0.022, 35.0
POURER_KG = 0.035
STEEL = (0.78, 0.79, 0.80)
CORK = (0.10, 0.10, 0.11)


def _pourer():
    grip_top = COLLAR_H + GRIP_H
    bend = math.radians(BEND_DEG)
    tip_at = (math.sin(bend) * BEND_L / 2, 0.0, grip_top + SPOUT_L + math.cos(bend) * BEND_L / 2 - 0.001)
    return [
        *_oct(STOPPER_D, -STOPPER_L, STOPPER_L, CORK),
        *_oct(LEAD_D, -FIT_DEPTH, LEAD_L, CORK),
        *_oct(COLLAR_D, 0.0, COLLAR_H),
        *_oct(GRIP_D, COLLAR_H, GRIP_H),
        mk.box((SPOUT, SPOUT, SPOUT_L), at=(0.0, 0.0, grip_top + SPOUT_L / 2)),
        mk.box((SPOUT, SPOUT, BEND_L), at=tip_at, quat=tuple(mk.mat_to_quat(mk.axis_angle((0, 1, 0), bend)))),
    ]


# The stopper's fit in a neck: friction along the bore while any of the stopper is inside it.
FIT_N = 6.0  # N to slide the stopper in the neck, in or out (more than the empty bottle's 4.1 N weight)
FIT_TRAVEL = 0.045  # the joint's outer stop, beyond the lip
ENTER_M = 0.005  # the fit takes hold once the stopper's lead is this far inside the bore
LEAVE_M = 0.002  # and lets go when less than this is left inside
ENTER_OFF, ENTER_TILT = 0.0035, 8.0  # the lead has to be this close to the bore's axis (m) and in line (deg)

kit = mk.Kit()
kit.part("empty_bottle", (*OLD, BASE_Z), _bottle(), mass=EMPTY_KG, color=GLASS, yaw=math.radians(90.0 - SIDE_DEG))
kit.part("new_bottle", (*NEW, BASE_Z), _bottle(), mass=FULL_KG, color=AMBER)
kit.part("pourer", (*OLD, BASE_Z + BOTTLE_H), _pourer(), mass=POURER_KG, color=STEEL)


def _seat(xy):
    t = np.eye(4)
    t[:3, 3] = (*xy, BASE_Z + BOTTLE_H)
    return t


kit.fit("fit_old", "empty_bottle", "pourer", _seat(OLD), (0, 0, 1), FIT_TRAVEL, FIT_N, engaged=True, stick_mm=0.5,
        damping=40.0)
kit.fit("fit_new", "new_bottle", "pourer", _seat(NEW), (0, 0, 1), FIT_TRAVEL, FIT_N, engaged=False, stick_mm=0.5,
        damping=40.0)

RUBBER = (0.07, 0.07, 0.08)
FIXTURES = [Box("bar_mat", pos=(*MAT, TOP + MAT_SIZE[2] / 2), size=MAT_SIZE, color=RUBBER)]
OBJECTS = [*kit.objects(), *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- scoring ---------------------------------------------------------------------------------------------------

STAND_DEG = 8.0  # a standing bottle: within 8 deg of upright
STAND_GAP = 0.004  # and its base within 4 mm of the mat
DOWN_DEG = 45.0  # knocked over
HOME_M = 0.002  # pushed home: the collar within 2 mm of the lip
HOME_OFF, HOME_TILT = 0.002, 5.0
TUG_N = 3.5  # N, the retention tug, straight up on the spout's base
TUG_STEPS = 10  # 0.5 s
TUG_MOVE = 0.001  # m the pourer may rise in the neck under the tug
STILL_M = 0.0005
_TEST_TCPS = None  # self-test stand-ins for the grippers' TCPs and the arms' collision boxes
_TEST_ROBOT_BOXES = None


def _finite_state(st) -> bool:
    """True when the object's pose, velocity and world bounds are finite and well-formed."""
    try:
        pos = np.asarray(st.pos, dtype=float)
        quat = np.asarray(st.quat_xyzw, dtype=float)
        lv = np.asarray(st.lin_vel, dtype=float)
        av = np.asarray(st.ang_vel, dtype=float)
        lo = np.asarray(st.low, dtype=float)
        hi = np.asarray(st.high, dtype=float)
    except Exception:
        return False
    return bool(
        pos.shape == (3,) and quat.shape == (4,) and lv.shape == (3,) and av.shape == (3,)
        and lo.shape == (3,) and hi.shape == (3,)
        and np.isfinite([pos, lv, av, lo, hi]).all() and np.isfinite(quat).all()
        and float(np.linalg.norm(quat)) > 1e-6 and np.all(lo <= hi)
    )


def _tilt(st) -> float:
    """Angle of the part's +z from world up (deg); 180 on an invalid quaternion so checks fail closed."""
    try:
        q = np.asarray(st.quat_xyzw, dtype=float)
        n = float(np.linalg.norm(q))
        if q.shape != (4,) or not np.isfinite(q).all() or n <= 1e-6:
            return 180.0
        x, y, _, _ = q / n
        return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * float(x * x + y * y)))))
    except Exception:
        return 180.0


def standing(objects, name) -> bool:
    """The bottle upright on the mat: base centre over the mat and within 4 mm of its top, within 8 deg of vertical."""
    if not _finite_state(objects[name]):
        return False
    base, _ = kit.pose(objects, name)
    if not np.isfinite(base).all():
        return False
    on_mat = (abs(float(base[0]) - MAT[0]) <= MAT_SIZE[0] / 2
              and abs(float(base[1]) - MAT[1]) <= MAT_SIZE[1] / 2)
    return on_mat and _tilt(objects[name]) <= STAND_DEG and abs(float(base[2]) - BASE_Z) <= STAND_GAP


def in_neck(objects, fit) -> bool:
    """Some of the stopper is inside this bottle's bore: its lower end below the lip and within the bore."""
    out, off, _ = kit.fit_pose(objects, fit)
    return math.isfinite(out) and math.isfinite(off) and out < FIT_DEPTH and off <= BORE / 2


def pourer_out(objects) -> bool:
    """The pourer is out of the empty bottle, and that bottle is still standing on the mat."""
    return (_finite_state(objects["pourer"]) and not in_neck(objects, "fit_old")
            and standing(objects, "empty_bottle"))


def started(objects) -> bool:
    """The stopper's lead is 5 mm or more inside the new bottle's bore, on its axis within 3.5 mm and 8 deg."""
    if not (_finite_state(objects["pourer"]) and _finite_state(objects["new_bottle"])):
        return False
    out, off, tilt = kit.fit_pose(objects, "fit_new")
    return -0.002 <= out <= FIT_DEPTH - ENTER_M and off <= ENTER_OFF and tilt <= ENTER_TILT


def home(objects) -> bool:
    """Pushed home in the new bottle: collar within 2 mm of the lip, on the axis within 2 mm and 5 deg."""
    if not (_finite_state(objects["pourer"]) and _finite_state(objects["new_bottle"])):
        return False
    out, off, tilt = kit.fit_pose(objects, "fit_new")
    return -0.002 <= out <= HOME_M and off <= HOME_OFF and tilt <= HOME_TILT


def _clear_of(objects, name, margin=0.020, gap=0.002) -> bool:
    """Nothing of either arm on the part: both TCPs 2 cm clear of its bounds, every collision piece of both arms
    (links, palm, finger pads, as posed) 2 mm clear."""
    st = objects[name]
    # Missing or invalid measurements cannot certify that both arms are clear.
    # Offline self-tests may explicitly supply an empty collision fixture; the
    # live readout must contain the robot collision geometry.
    try:
        lo, hi = np.asarray(st.low, dtype=float), np.asarray(st.high, dtype=float)
        tcps = np.asarray(mk.tcps() if _TEST_TCPS is None else _TEST_TCPS, dtype=float)
        boxes = list(mk.robot_collision_boxes() if _TEST_ROBOT_BOXES is None else _TEST_ROBOT_BOXES)
        if (lo.shape != (3,) or hi.shape != (3,) or not np.isfinite([lo, hi]).all()
                or np.any(lo > hi) or tcps.shape != (2, 3) or not np.isfinite(tcps).all()
                or (not boxes and _TEST_ROBOT_BOXES is None)):
            return False
        for blo, bhi in boxes:
            blo, bhi = np.asarray(blo, dtype=float), np.asarray(bhi, dtype=float)
            if (blo.shape != (3,) or bhi.shape != (3,) or not np.isfinite([blo, bhi]).all()
                    or np.any(blo > bhi)):
                return False
            d = np.maximum(np.maximum(lo - bhi, blo - hi), 0.0)
            if float(np.linalg.norm(d)) < gap:
                return False
        return not any(bool(np.all(t >= lo - margin) and np.all(t <= hi + margin)) for t in tcps)
    except Exception:
        # A failed observation is unknown clearance, including during a tug.
        return False


def let_go(objects) -> bool:
    return all(_clear_of(objects, n) for n in ("pourer", "empty_bottle", "new_bottle"))


def _fit_rule(objects) -> None:
    """The stopper's fins take hold of a neck once the lead is 5 mm inside it and in line, and let go when the
    stopper is drawn out again. Also latches a bottle knocked over."""
    for name in ("empty_bottle", "new_bottle"):
        if _tilt(objects[name]) > DOWN_DEG:
            mk.MEMORY["knocked_over"] = True
    if not all(_finite_state(objects[n]) for n in ("pourer", "empty_bottle", "new_bottle")):
        return
    for fit in ("fit_old", "fit_new"):
        out, off, tilt = kit.fit_pose(objects, fit)
        if kit.fit_engaged(fit):
            if out >= FIT_DEPTH - LEAVE_M:
                kit.clear_fit(fit)
        elif -0.002 <= out <= FIT_DEPTH - ENTER_M and off <= ENTER_OFF and tilt <= ENTER_TILT:
            kit.engage_fit(fit, objects)


def _tug_rule(objects) -> None:
    """The retention tug. Once the pourer is home in the new bottle with nothing of the arms on it and it has not
    moved for 0.5 s, a 3.5 N pull straight up on the spout's base is held for 0.5 s. It passes if the pourer rises 1 mm
    or less. Drawing the pourer out again cancels a pass."""
    m = mk.MEMORY
    if not (home(objects) and standing(objects, "empty_bottle") and standing(objects, "new_bottle")
            and kit.fit_engaged("fit_new")):
        if m.get("tug_n"):
            kit.pull("pourer")
        m.update(retained=False, tug_n=0, still_n=0)
        return
    if m.get("retained"):
        return
    out = kit.fit_pose(objects, "fit_new")[0]
    free = _clear_of(objects, "pourer")
    n = m.get("tug_n", 0)
    if n:
        m["tug_peak"] = max(m.get("tug_peak", 0.0), out - m["tug_from"])
        if n < TUG_STEPS and free:
            m["tug_n"] = n + 1
            return
        kit.pull("pourer")
        m.update(tug_n=0, still_n=0, tugs=m.get("tugs", 0) + 1)
        m["retained"] = bool(free and n >= TUG_STEPS and m["tug_peak"] <= TUG_MOVE)
        return
    still = m.get("still_n", 0)
    if not free or abs(out - m.get("still_from", out)) > STILL_M:
        still = 0
    if not still:
        m["still_from"] = out
    m["still_n"] = still = still + 1 if free else 0
    if still >= 11:  # eleven samples span 0.5 s at 20 Hz
        m.update(tug_from=out, tug_peak=0.0)
        if kit.pull("pourer", (0.0, 0.0, TUG_N), kit.point(objects, "pourer", (0, 0, COLLAR_H + GRIP_H / 2))):
            m["tug_n"] = 1


kit.on_step(_fit_rule)
kit.on_step(_tug_rule)


def score(objects) -> float:
    """0 if a bottle was knocked over (more than 45 deg off upright) at any moment. Otherwise by stage:
    0.3 the pourer out of the empty bottle, that bottle standing on the mat; 0.5 also the stopper started in the new
    bottle's neck, that bottle standing; 0.7 also pushed home (collar within 2 mm of the lip); 1.0 also let go, all
    three at rest, and the 3.5 N tug passed."""
    if mk.MEMORY.get("knocked_over"):
        return 0.0
    if not all(_finite_state(objects[n]) for n in ("pourer", "empty_bottle", "new_bottle")):
        return 0.0
    if not pourer_out(objects):
        return 0.0
    if not (started(objects) and standing(objects, "new_bottle")):
        return 0.3
    if not home(objects):
        return 0.5
    still = all(objects[n].resting() for n in ("pourer", "empty_bottle", "new_bottle"))
    return 1.0 if mk.MEMORY.get("retained") and still and let_go(objects) else 0.7


def success(objects) -> bool:
    """The pourer home in the new bottle and through the tug, both bottles standing, everything let go; 1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Metres from the pourer's collar to its seat on the new bottle's lip."""
    out, off, _ = kit.fit_pose(objects, "fit_new")
    return float(math.hypot(out, off))


# -- the scripted demo ------------------------------------------------------------------------------------------
# Both grippers work level, fingers closing across a pair of the octagons' flats. Left: comes in from the left
# (67.5 deg round from +x toward -y), takes the empty bottle by its neck under the bead and bears down on it; when
# the pourer is out it lets go, backs off, turns to come in at 45 deg (toward +x, -y) and takes the new bottle's
# neck. Right: comes in at 45 deg (toward +x, +y), closes on the spout's base, takes up the pull and draws the
# pourer up out of the neck, carries it across to above the new bottle's lip, waits for that bottle to be held,
# lowers the stopper's lead into the neck, bears down and pushes it home, eases off, lets go and backs off.
#
# The arms are position-controlled and soft: about 0.1 N per mm at the fingers, 6 mm above the neck. To push with a
# force F without the hand sagging or tipping, a waypoint is commanded ahead of where the hand is to be, at the
# pose the joints reach when each is led by its share of the load over its stiffness (J^T F / K, the joint gains
# being the arm's own). Under the load the hand then sits where it was meant to, square to what it holds.

_LEVEL = mk.axis_angle((0, 1, 0), -math.pi / 2) @ grip_rotation(0.0)  # approach +x, fingers closing along y
JOINT_K = np.array([ARM_J123["stiffness"]] * 3 + [ARM_J456["stiffness"]] * 3)  # N*m/rad, joints 1-6


def _level(deg):
    """(waypoint aim, approach direction) of a level grip coming in at ``deg`` from +x toward +y."""
    a = math.radians(deg)
    return mk.Aim(mk.axis_angle((0, 0, 1), a) @ _LEVEL), np.array([math.cos(a), math.sin(a), 0.0])


AIM_SIDE, DIR_SIDE = _level(-SIDE_DEG)  # the left hand on the empty bottle
AIM_L, DIR_L = _level(-45.0)  # the left hand on the new bottle
AIM_R, DIR_R = _level(45.0)  # the right hand, throughout
HOLD_Z = BASE_Z + 0.240  # the holding hand, on the neck: pads from 15 mm above the shoulder to 16 mm under the bead
GRIP_Z = BASE_Z + BOTTLE_H + COLLAR_H + GRIP_H - 0.014  # the pulling hand, on the spout's base: 23 mm above the lip
BACK = 0.110  # the hands come in from, and back off to, this far out along their approach
LIFT = 0.047  # the pourer is carried this much higher: the stopper's lead 15 mm above the lip
START = FIT_DEPTH - ENTER_M - 0.003  # the lead is lowered 8 mm into the new neck before the push
PRESS = 0.002  # commanded past the seat, to put the collar on the lip
PULL_N = 7.8  # N the right hand pulls up with to draw the stopper (the fit's 6 N and the pourer's weight, and a margin)
PUSH_N = 7.2  # N it bears down with to push the stopper home
BEAR_N = 4.5  # N the left hand bears down with on the empty bottle, so the pull does not lift it
HALF = FINGER_OPEN * 0.5
NECK_GRIP = -(NECK_OD / 2 - 0.006)  # finger target on the 29 mm neck: 6 mm past contact, about 30 N a finger
PARK_L = (0.180, 0.180, HOLD_Z)
PARK_R = (0.180, -0.180, GRIP_Z)
UP, DOWN = np.array([0.0, 0.0, 1.0]), np.array([0.0, 0.0, -1.0])


def _at(xy, z, direction, back=0.0):
    return np.array([xy[0] - back * direction[0], xy[1] - back * direction[1], z])


class _Hand:
    """Builds one arm's waypoints. It follows the arm's joints along the plan, so that a waypoint with a ``push``
    (the force, N, the hand applies to what it holds) can be commanded at the led pose described above."""

    def __init__(self, arm):
        self.base = np.asarray(station.BASES[arm], dtype=float)
        self.q = np.array(READY, dtype=float)
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.plan = []

    def _follow(self, pos, rot, steps=12):
        for k in range(1, steps + 1):
            s = k / steps
            p = self.pos + s * (pos - self.pos)
            r = ik.slerp_matrix(self.rot, rot, s)
            self.q = ik.solve(p, ik.matrix_to_quat_xyzw(r), self.q, base_pos=self.base, iters=60)[0]
        self.pos, self.rot = pos, rot

    def _led(self, push):
        here = ik.tcp_pose(self.q, self.base)[0]
        jac = np.zeros((3, 6))
        for i in range(6):
            dq = self.q.copy()
            dq[i] += 1e-6
            jac[:, i] = (ik.tcp_pose(dq, self.base)[0] - here) / 1e-6
        return ik.tcp_pose(self.q + jac.T @ np.asarray(push, dtype=float) / JOINT_K, self.base)

    def to(self, duration, pos, finger, label, aim, push=None):
        pos = np.asarray(pos, dtype=float)
        self._follow(pos, aim.rot)
        if push is not None:
            pos, rot = self._led(push)
            aim = mk.Aim(rot)
        self.plan.append(Waypoint(duration, tuple(float(v) for v in pos), finger, label, aim))


def _left(hold_old=True):
    o, g = HALF, NECK_GRIP
    bear = BEAR_N * DOWN
    s, ds, a, da = AIM_SIDE, DIR_SIDE, AIM_L, DIR_L
    h = _Hand("left")
    h.to(3.0, _at(OLD, HOLD_Z, ds, BACK), o, "level, to the left of the empty bottle's neck", s)
    if hold_old:
        h.to(2.0, _at(OLD, HOLD_Z, ds), o, "fingers either side of the neck", s)
        h.to(1.0, _at(OLD, HOLD_Z, ds), g, "hold the empty bottle", s)
        h.to(1.5, _at(OLD, HOLD_Z, ds), g, "bear down on it", s, bear)
        h.to(12.5, _at(OLD, HOLD_Z, ds), g, "hold it down while the pourer is drawn", s, bear)
        h.to(1.0, _at(OLD, HOLD_Z, ds), g, "ease off", s)
        h.to(1.0, _at(OLD, HOLD_Z, ds), o, "let go", s)
        h.to(1.5, _at(OLD, HOLD_Z, ds, BACK), o, "back off", s)
    else:
        h.to(20.5, _at(OLD, HOLD_Z, ds, BACK), o, "stay back: the empty bottle is not held", s)
    h.to(2.5, _at(NEW, HOLD_Z, da, BACK), o, "turn and cross, behind the new bottle's neck", a)
    h.to(2.0, _at(NEW, HOLD_Z, da), o, "fingers either side of the neck", a)
    h.to(1.0, _at(NEW, HOLD_Z, da), g, "hold the new bottle", a)
    h.to(13.5, _at(NEW, HOLD_Z, da), g, "steady it while the pourer is pushed home", a)
    h.to(1.0, _at(NEW, HOLD_Z, da), o, "let go", a)
    h.to(1.5, _at(NEW, HOLD_Z, da, BACK), o, "back off", a)
    h.to(2.0, PARK_L, o, "clear", a)
    return h.plan


def _right(push=True):
    c = FINGER_CLOSED
    up = GRIP_Z + LIFT
    a, d = AIM_R, DIR_R
    pull = PULL_N * UP
    h = _Hand("right")
    h.to(3.0, _at(OLD, up, d, BACK), HALF, "level, behind and above the pourer", a)
    h.to(3.5, _at(OLD, GRIP_Z, d, BACK), HALF, "wait for the bottle to be held", a)
    h.to(2.0, _at(OLD, GRIP_Z, d), HALF, "fingers either side of the spout's base", a)
    h.to(1.0, _at(OLD, GRIP_Z, d), c, "grip it", a)
    h.to(1.0, _at(OLD, GRIP_Z, d), c, "take up half the pull", a, pull / 2)
    h.to(1.0, _at(OLD, GRIP_Z, d), c, "take up the pull", a, pull)
    h.to(4.0, _at(OLD, GRIP_Z + FIT_DEPTH + 0.008, d), c, "draw the pourer up out of the neck", a, pull)
    h.to(2.5, _at(OLD, GRIP_Z + FIT_DEPTH + 0.008, d), c, "the stopper is out: hold", a, pull)
    h.to(2.0, _at(OLD, up, d), c, "ease off, the pourer clear above the neck", a)
    h.to(3.0, _at(NEW, up, d), c, "across, over the new bottle's neck", a)
    h.to(6.0, _at(NEW, up, d), c, "wait for the new bottle to be held", a)
    h.to(3.0, _at(NEW, GRIP_Z + START, d), c, "lower the stopper's lead into the neck", a)
    if push:
        bear = PUSH_N * DOWN
        h.to(1.0, _at(NEW, GRIP_Z + START, d), c, "bear down, half", a, bear / 2)
        h.to(1.0, _at(NEW, GRIP_Z + START, d), c, "bear down", a, bear)
        h.to(3.0, _at(NEW, GRIP_Z - PRESS, d), c, "push the stopper home", a, bear)
        h.to(2.5, _at(NEW, GRIP_Z - PRESS, d), c, "hold the collar on the lip", a, bear)
        h.to(1.5, _at(NEW, GRIP_Z, d), c, "ease off", a)
        seat = GRIP_Z
    else:
        h.to(1.0, _at(NEW, GRIP_Z + START, d), c, "leave it resting in the mouth", a)
        seat = GRIP_Z + START
    h.to(1.0, _at(NEW, seat, d), HALF, "let go", a)
    h.to(1.5, _at(NEW, seat, d, BACK), HALF, "back off", a)
    h.to(2.0, PARK_R, HALF, "clear", a)
    return h.plan


def demo():
    """Left holds each bottle in turn; right draws the pourer, carries it across and pushes it home. About 46 s,
    then the scorer's tug. DEMO_VARIANT picks a wrong way of doing it: ``one_handed`` (the empty bottle is not held
    while the pourer is pulled), ``rest_only`` (the pourer is set in the new neck and let go, not pushed home),
    ``none`` (neither arm moves)."""
    variant = os.environ.get("DEMO_VARIANT", "")
    if variant == "one_handed":
        return {"left": _left(hold_old=False), "right": _right()}
    if variant == "rest_only":
        return {"left": _left(), "right": _right(push=False)}
    if variant == "none":
        return {}
    if variant:
        raise ValueError(f"Unknown DEMO_VARIANT: {variant}")
    return {"left": _left(), "right": _right()}


# -- offline self-test ------------------------------------------------------------------------------------------


def _states(pourer=None, old=None, new=None):
    """Hand-built states. ``pourer``: ("old" | "new", travel out of the seat, sideways offset, tilt deg) or an
    absolute (pos, quat); ``old`` / ``new``: (xy, lift, tilt deg) of a bottle."""
    moved = {}
    seats = {"old": np.array([*OLD, BASE_Z + BOTTLE_H]), "new": np.array([*NEW, BASE_Z + BOTTLE_H])}
    for name, spec, xy0 in (("empty_bottle", old, OLD), ("new_bottle", new, NEW)):
        if spec is not None:
            xy, lift, tilt = spec
            xy = xy0 if xy is None else xy
            moved[name] = (np.array([*xy, BASE_Z + lift]), mk.mat_to_quat(mk.axis_angle((1, 0, 0), math.radians(tilt))))
            key = "old" if name == "empty_bottle" else "new"
            r = mk.axis_angle((1, 0, 0), math.radians(tilt))
            seats[key] = np.array([*xy, BASE_Z + lift]) + r @ np.array([0, 0, BOTTLE_H])
    if pourer is not None:
        if isinstance(pourer[0], str):
            where, out, off, tilt = pourer
            moved["pourer"] = (seats[where] + np.array([off, 0.0, out]),
                               mk.mat_to_quat(mk.axis_angle((0, 1, 0), math.radians(tilt))))
        else:
            moved["pourer"] = pourer
    return kit.fake_states(moved=moved)


def self_test():
    """The scoring rules on hand-built poses, the fit's take-hold and let-go rule, and the geometry."""
    global _TEST_TCPS, _TEST_ROBOT_BOXES
    far = [(0.15, 0.23, 0.95), (0.15, -0.23, 0.95)]
    on_bar = (np.array([0.20, -0.20, BASE_Z + 0.012]), mk.mat_to_quat(mk.axis_angle((0, 1, 0), math.pi / 2)))
    R = {"retained": True}
    cases = {  # name: (states, TCPs, memory, score)
        "start: nothing done": (_states(), far, {}, 0.0),
        "pourer half drawn": (_states(("old", 0.015, 0, 0)), far, {}, 0.0),
        "pourer out, held above the empty bottle": (_states(("old", 0.050, 0, 0)), far, {}, 0.3),
        "pourer out, laid on the bar": (_states(on_bar), far, {}, 0.3),
        "one-handed: bottle up off the bar on the pourer": (_states(("old", 0.0, 0, 0), old=(None, 0.03, 0)), far, {},
                                                            0.0),
        "pourer out, empty bottle leaning 20 deg": (_states(("new", 0.0, 0, 0), old=(None, 0.0, 20)), far, R, 0.0),
        "lead resting in the mouth, 3 mm in": (_states(("new", FIT_DEPTH - 0.003, 0, 0)), far, {}, 0.3),
        "over the new bottle, 4 mm off axis": (_states(("new", 0.020, 0.004, 0)), far, {}, 0.3),
        "started 6 mm in": (_states(("new", FIT_DEPTH - 0.006, 0, 0)), far, {}, 0.5),
        "half way in": (_states(("new", 0.015, 0, 0)), far, {}, 0.5),
        "3 mm short of the lip": (_states(("new", 0.003, 0, 0)), far, R, 0.5),
        "home, still gripped": (_states(("new", 0.0005, 0, 0)), [far[0], (*NEW, GRIP_Z)], R, 0.7),
        "home, let go, no tug yet": (_states(("new", 0.0005, 0, 0)), far, {}, 0.7),
        "home, let go, tug failed": (_states(("new", 0.0005, 0, 0)), far, {"retained": False, "tugs": 1}, 0.7),
        "home, let go, tug passed": (_states(("new", 0.0005, 0, 0)), far, R, 1.0),
        "home and tugged, new bottle still held": (_states(("new", 0.0005, 0, 0)), [(*NEW, HOLD_Z), far[1]], R, 0.7),
        "home and tugged, new bottle slid 3 cm": (_states(("new", 0.0005, 0, 0), new=((0.30, -0.07), 0, 0)), far, R,
                                                  1.0),
        "home and tugged, a bottle was knocked over": (_states(("new", 0.0005, 0, 0)), far,
                                                       {"retained": True, "knocked_over": True}, 0.0),
        "started, new bottle leaning 12 deg": (_states(("new", 0.010, 0, 0), new=(None, 0.0, 12)), far, {}, 0.3),
    }
    try:
        for name, (st, tcps, mem, want) in cases.items():
            mk.new_episode()
            _TEST_TCPS, _TEST_ROBOT_BOXES = tcps, []
            mk.MEMORY.update(mem)
            got = score(st)
            assert abs(got - want) < 1e-9 and success(st) == (want >= 1.0), (name, got, want)
        # a finger pad 1 mm from the pourer while the TCPs are clear: not let go
        mk.new_episode()
        mk.MEMORY.update(R)
        _TEST_TCPS = far
        st = _states(("new", 0.0005, 0, 0))
        hi = np.asarray(st["pourer"].high)
        _TEST_ROBOT_BOXES = [(hi + np.array([0.001, -0.02, -0.03]), hi + np.array([0.02, 0.0, 0.0]))]
        assert score(st) == 0.7, "a finger pad on the pourer"
        # missing or non-finite clearance readouts, off-mat bottle, or invalid pose fail closed
        _TEST_TCPS = [np.full(3, np.nan), np.full(3, np.nan)]
        _TEST_ROBOT_BOXES = [(np.full(3, np.nan), np.full(3, np.nan))]
        assert not let_go(st) and score(st) == 0.7, "non-finite clearance fails closed"
        _TEST_TCPS, _TEST_ROBOT_BOXES = far, None
        saved_boxes = mk.robot_collision_boxes
        try:
            mk.robot_collision_boxes = lambda: []
            assert not let_go(st) and score(st) == 0.7, "empty live collision boxes fail closed"
        finally:
            mk.robot_collision_boxes = saved_boxes
        _TEST_TCPS, _TEST_ROBOT_BOXES = far, []
        assert score(_states(("new", 0.0005, 0, 0), new=((0.40, -0.07), 0, 0))) == 0.3, "bottle off mat is not standing"
        # the fit rule: holds at the start, lets go when drawn, takes hold in the new neck, latches a fall
        mk.new_episode()
        _TEST_TCPS, _TEST_ROBOT_BOXES = far, []
        assert kit.fit_engaged("fit_old") and not kit.fit_engaged("fit_new")
        _fit_rule(_states(("old", 0.020, 0, 0)))
        assert kit.fit_engaged("fit_old")
        _fit_rule(_states(("old", FIT_DEPTH - 0.001, 0, 0)))
        assert not kit.fit_engaged("fit_old")
        _fit_rule(_states(("new", FIT_DEPTH - 0.003, 0, 0)))
        assert not kit.fit_engaged("fit_new"), "3 mm in: not yet"
        _fit_rule(_states(("new", FIT_DEPTH - 0.006, 0.005, 0)))
        assert not kit.fit_engaged("fit_new"), "5 mm off the axis: the stopper is on the rim"
        _fit_rule(_states(("new", FIT_DEPTH - 0.006, 0.001, 2)))
        assert kit.fit_engaged("fit_new")
        _fit_rule(_states(("new", 0.0, 0, 0)))
        assert kit.fit_engaged("fit_new") and not mk.MEMORY.get("knocked_over")
        _fit_rule(_states(("new", 0.0, 0, 0), old=(None, 0.0, 60)))
        assert mk.MEMORY.get("knocked_over")
        # the tug rule offline: no simulator, so no pull starts and nothing is credited
        mk.new_episode()
        mk.MEMORY[("fit", kit.id, "fit_new")] = True
        for _ in range(40):
            _tug_rule(_states(("new", 0.0005, 0, 0)))
        assert not mk.MEMORY.get("retained") and not mk.MEMORY.get("tug_n")
    finally:
        mk.new_episode()
        _TEST_TCPS = None
        _TEST_ROBOT_BOXES = None
    # geometry and forces
    assert STOPPER_D < BORE < COLLAR_D < NECK_OD and LEAD_D < STOPPER_D
    assert FULL_KG * 9.81 > FIT_N > 1.4 * EMPTY_KG * 9.81, "drawn one-handed, the empty bottle comes up with the pourer"
    assert TUG_N < FIT_N / 1.5 and TUG_N > 5 * POURER_KG * 9.81
    assert PULL_N > 1.2 * (FIT_N + POURER_KG * 9.81) and PUSH_N > 1.2 * (FIT_N - POURER_KG * 9.81)
    assert EMPTY_KG * 9.81 + BEAR_N > 1.1 * PULL_N, "held down harder than the pull"
    assert FIT_TRAVEL > FIT_DEPTH + 0.005 and ENTER_M > LEAVE_M
    assert -NECK_GRIP * 2 < NECK_OD < -2 * HALF - 0.010, "the neck fits the half-open fingers"
    assert NECK_Z0 + 0.010 < HOLD_Z - BASE_Z - 0.010 and HOLD_Z - BASE_Z + 0.010 < BEAD_Z0 - 0.010
    assert GRIP_Z - 0.010 > BASE_Z + BOTTLE_H + COLLAR_H + 0.008, "the pads close on the base, clear of the collar"
    assert GRIP_Z + 0.010 < BASE_Z + BOTTLE_H + COLLAR_H + GRIP_H
    assert abs(NEW[1] - OLD[1]) > BODY_D + 0.050
    return len(cases) + 12


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules on hand-built states, then every demo plan replayed through the
    # kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    print(f"scoring: {self_test()} cases pass")
    task = load(__file__)
    hz = 20.0
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
