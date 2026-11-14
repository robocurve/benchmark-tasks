"""On the station table: a manual cup-sealing machine (a bubble-tea shop's lever sealer) stands in front of the arms,
its sealing head raised. Its base has a cup holder, a square recess 74 mm across and 20 mm deep, under the head. A
lever on a pivot at the back of the machine carries the sealing head; its handle, a bar across the machine's width,
sticks out at the front. A spring holds the lever up. Pulling the handle down swings the head down onto the rim of a
cup standing in the holder. Beside the machine, on the table in front of the left arm, stands an open 16 oz drink cup
(95 mm rim), filled with milk tea.

Goal: the cup sealed. It must stand in the holder, the head must be pressed fully down on its rim and held there for
at least 3 s (the lever sealer's manual: pull the handle down for 3-4 seconds), then the lever let back up, with the cup still upright
in the holder. A gripper inside the cup-holder zone while the head is coming down, or a spilled drink, scores 0.

Run it:  inspect-robots run --embodiment yam_arena -E task=cup_sealer_seal.py -E object_jitter=0 \\
           --instruction "Seal the drink." --scorer task_score ...
Offline check (no GPU):  python cup_sealer_seal.py
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import hingekit as hk  # noqa: E402
import yam_arena.demo as _demo  # noqa: E402
from yam_arena import ik  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Seal the drink."

# -- dimensions (m) -----------------------------------------------------------------------------------------------
# The machine: a 32 x 23 cm footprint (front to back, side to side), like a common 90-95 mm manual sealer.
X0, X1, HALF_W = 0.24, 0.56, 0.115  # front and back edges (forward from the arm bases), half width
BASE_H = 0.050  # base top above the table
POCKET_C = np.array([0.320, 0.0])  # the cup holder's centre
POCKET_W, POCKET_D = 0.074, 0.020  # the holder recess: 74 mm square, 20 mm deep (5 mm clearance round the 64 mm base)
FLOOR = TOP + BASE_H - POCKET_D  # the holder's floor
COLUMN = (0.48, X1)  # the back column (front, back), up to the pivot block
PIVOT = np.array([0.520, 0.0, TOP + 0.240])  # the lever's pivot, axis along +y (side to side)
# The cup: a 16 oz PP cup, 95 mm rim, 64 mm base, 140 mm body plus a 4 mm rolled rim; filled to 2 cm below the rim.
CUP_R = (0.032, 0.036, 0.040, 0.044)  # four 35 mm bands, bottom to top (the taper, in steps)
CUP_BAND = 0.035
RIM_R, RIM_H = 0.0475, 0.004
CUP_H = 4 * CUP_BAND + RIM_H  # 144 mm
CUP_MASS = 0.35  # cup and about 330 ml of milk tea
CUP_START = np.array([0.280, 0.200])  # on the table in front of the left arm, left of the machine
# The lever, drawn at angle 0 (head down on a seated cup's rim): an arm from the pivot to the handle.
HEAD_R, HEAD_H = 0.055, 0.025  # the heated sealing plate
PLATE_FRICTION = 0.1  # PTFE on PP: about 0.1
HEAD_BOTTOM_0 = FLOOR + CUP_H  # the plate's underside at angle 0 sits on a seated cup's rim
HANDLE_DX = -0.310  # the handle bar's centre, from the pivot, at angle 0 (forward is -x from the pivot)
UP_DEG = 35.0  # the lever's top stop (the spring holds it there)
LOWER_DEG = -3.0  # the bottom stop, below the rim: with no cup the head would go lower

STEEL = (0.75, 0.76, 0.78)
DARK = (0.22, 0.23, 0.25)
RED = (0.80, 0.16, 0.12)
TEA = (0.80, 0.66, 0.50)
WHITE = (0.95, 0.95, 0.95)

kit = hk.Kit()

# The machine's body: static boxes (a base with the holder recess, the back column and its pivot block).
_px, _py = POCKET_C
_pw = POCKET_W / 2
_body = [
    hk.box((X1 - X0, 2 * HALF_W, BASE_H - POCKET_D), at=((X0 + X1) / 2, 0, TOP + (BASE_H - POCKET_D) / 2)),
    # the base's top layer, with a 74 mm square opening at the holder
    hk.box((_px - _pw - X0, 2 * HALF_W, POCKET_D), at=((X0 + _px - _pw) / 2, 0, FLOOR + POCKET_D / 2)),
    hk.box((X1 - (_px + _pw), 2 * HALF_W, POCKET_D), at=((_px + _pw + X1) / 2, 0, FLOOR + POCKET_D / 2)),
    hk.box((POCKET_W, HALF_W - _pw, POCKET_D), at=(_px, (HALF_W + _pw) / 2, FLOOR + POCKET_D / 2)),
    hk.box((POCKET_W, HALF_W - _pw, POCKET_D), at=(_px, -(HALF_W + _pw) / 2, FLOOR + POCKET_D / 2)),
    hk.box((0.010, POCKET_W + 0.02, 0.003), at=(_px - _pw - 0.006, 0, TOP + BASE_H + 0.0015), color=RED),  # mark
    hk.box((COLUMN[1] - COLUMN[0], 2 * HALF_W, PIVOT[2] - 0.02 - TOP - BASE_H),
           at=(sum(COLUMN) / 2, 0, (TOP + BASE_H + PIVOT[2] - 0.02) / 2), color=DARK),
]
kit.part("sealer_body", origin=(0, 0, 0), shapes=[hk.box(b.size, b.at, b.color) for b in _body], color=STEEL)

# The lever: one rigid body pivoting about PIVOT (frame origin at the pivot), carrying the sealing head.
_head_dx = POCKET_C[0] - PIVOT[0]  # -0.20: the head hangs over the holder
_head_dz = HEAD_BOTTOM_0 + HEAD_H / 2 - PIVOT[2]
kit.part("lever", origin=PIVOT, mass=0.6, color=DARK, shapes=[
    hk.box((0.07, 0.10, 0.04), at=(0.0, 0.0, 0.0)),  # the hub on the pivot
    hk.box((abs(HANDLE_DX), 0.030, 0.020), at=(HANDLE_DX / 2, 0.0, 0.0)),  # the arm
    hk.cyl(0.013, 0.140, at=(HANDLE_DX, 0.0, 0.0), color=RED, axis="Y"),  # the round handle bar, across the machine
    hk.box((0.030, 0.030, -_head_dz - HEAD_H / 2), at=(_head_dx, 0.0, (_head_dz + HEAD_H / 2) / 2)),  # head stem
    hk.cyl(HEAD_R, HEAD_H, at=(_head_dx, 0.0, _head_dz), color=STEEL, friction=PLATE_FRICTION),  # the heated sealing
    # plate, PTFE-coated like most sealer plates (so the film doesn't stick): it slides on the rim, doesn't drag it
])
kit.hinge("lever_pivot", "lever", anchor=PIVOT, axis=(0, 1, 0), lower_deg=LOWER_DEG, upper_deg=UP_DEG,
          start_deg=UP_DEG, stiffness=4.0, rest_deg=60.0, damping=0.3)

# The cup: four stepped bands for its taper and a thin rolled rim (frame origin at its base centre).
kit.part("cup", origin=(CUP_START[0], CUP_START[1], TOP), mass=CUP_MASS, color=TEA, shapes=[
    *[hk.cyl(r, CUP_BAND, at=(0, 0, CUP_BAND * (i + 0.5))) for i, r in enumerate(CUP_R)],
    hk.cyl(RIM_R, RIM_H, at=(0, 0, 4 * CUP_BAND + RIM_H / 2), color=WHITE),
])

OBJECTS = kit.objects()

# -- scoring ------------------------------------------------------------------------------------------------------
SEAT_XY, SEAT_Z, SEAT_TILT = 0.0075, 0.003, 4.0  # m, m, degrees: seated in the holder (7.5 mm: the plate's overhang
# past the 95 mm rim, and more than the recess lets the base move, 7.1 mm into a corner)
SEAL_S = 3.0  # seconds of contact: a lever sealer's manual says to pull the handle down for 3-4 s
RIM_POINTS = 36  # the rim is checked at 36 points, every 10 degrees round it
CONTACT_GAP = 0.001  # a rim point is under the plate when it is within 1 mm of the plate's underside ...
FULL, PARTIAL = 36, 27  # ... full contact: all 36 points; partial: at least 27 (three quarters of the rim)
UP_AGAIN_DEG = 25.0  # the lever is let back up
SPILL_TILT = 23.0  # degrees: 2 cm of headspace under a 47.5 mm rim radius pours past atan(20 / 47.5) = 23 degrees
ZONE_R = 0.070  # the pinch zone: within 7 cm of the holder's axis ...
ZONE_DEG = 15.0  # ... from the floor up to the plate, once the lever is below 15 degrees (head within ~5 cm of the rim)
CONTROL_HZ = 20.0
DEBUG = bool(__import__('os').environ.get('SEALER_DEBUG'))  # SEALER_DEBUG=1: print the monitor's state each second

# The seal grades (each needs the cup seated and the contact held SEAL_S without a break):
#   "full"    all 36 rim points within 1 mm of the plate -> film fused all round the rim
#   "partial" 27-35 points -> part of the rim unfused: the cup would leak
#   "none"    not held that long, or under 27 points
GRADE_SCORE = {"full": 0.7, "partial": 0.55, "pressed": 0.45, "none": 0.3}

MEM: dict = {}


def _reset() -> None:
    MEM.clear()
    MEM.update(t=0.0, full_s=0.0, partial_s=0.0, grade="none", pressed=False, spilled=False, pinch=False,
               best_points=0, reported=False)


_reset()
kit.on_reset(_reset)

_RIM_LOCAL = np.array([[RIM_R * math.cos(2 * math.pi * k / RIM_POINTS), RIM_R * math.sin(2 * math.pi * k / RIM_POINTS),
                        CUP_H] for k in range(RIM_POINTS)])
_PLATE_LOCAL = np.array([_head_dx, 0.0, _head_dz - HEAD_H / 2])  # the plate's underside centre, in the lever frame


def _cup_frame(objects):
    return kit.frame(objects, "cup")


def tilt_deg(objects) -> float:
    z = _cup_frame(objects)[:3, 2]
    return math.degrees(math.acos(max(-1.0, min(1.0, float(z[2])))))


def seated(objects) -> bool:
    """The cup stands in the holder: base centre within 7.5 mm of the holder's centre (anywhere the recess lets it stand), on its floor (3 mm), upright."""
    base = _cup_frame(objects)[:3, 3]
    return (float(np.linalg.norm(base[:2] - POCKET_C)) <= SEAT_XY and abs(float(base[2]) - FLOOR) <= SEAT_Z
            and tilt_deg(objects) <= SEAT_TILT)


def lever_deg(objects) -> float:
    return math.degrees(kit.angle(objects, "lever_pivot"))


def rim_contact(objects):
    """(points under the plate, worst gap in m): each of the 36 rim points counts when it lies within 1 mm of the
    plate's underside (measured along the plate's normal) and inside the plate's 55 mm radius."""
    c = _cup_frame(objects)
    rim = _RIM_LOCAL @ c[:3, :3].T + c[:3, 3]
    lv = kit.frame(objects, "lever")
    centre = lv[:3, :3] @ _PLATE_LOCAL + lv[:3, 3]
    n = lv[:3, 2]  # the plate's upward normal
    d = rim - centre
    gap = -(d @ n)  # how far below the underside each rim point is (negative: pressed into it)
    radial = np.linalg.norm(d - np.outer(d @ n, n), axis=1)
    under = (np.abs(gap) <= CONTACT_GAP) & (radial <= HEAD_R)
    return int(under.sum()), float(np.max(np.abs(gap)))


def _in_zone(p, lever_angle_deg: float) -> bool:
    """A gripper point inside the pinch zone: within ZONE_R of the holder's axis, between the floor and the plate."""
    head_bottom = HEAD_BOTTOM_0 + 0.20 * math.sin(math.radians(lever_angle_deg))  # about: the plate's height
    return float(np.linalg.norm(np.asarray(p[:2]) - POCKET_C)) < ZONE_R and FLOOR - 0.01 < p[2] < head_bottom


def monitor(objects, grippers=None) -> None:
    """Every control step (sim): latch spills and pinch hazards, time the rim contact and grade the seal."""
    dt = 1.0 / CONTROL_HZ
    MEM["t"] += dt
    a = lever_deg(objects)
    if MEM["grade"] != "full" and tilt_deg(objects) > SPILL_TILT:
        MEM["spilled"] = True
    tips = hk.tcps() if grippers is None else grippers
    if a < ZONE_DEG and any(_in_zone(p, a) for p in tips):
        MEM["pinch"] = True
    pts, _ = rim_contact(objects) if seated(objects) else (0, 0.0)
    MEM["best_points"] = max(MEM["best_points"], pts)
    if pts >= PARTIAL:
        MEM["pressed"] = True
    MEM["full_s"] = MEM["full_s"] + dt if pts >= FULL else 0.0
    MEM["partial_s"] = MEM["partial_s"] + dt if pts >= PARTIAL else 0.0
    if MEM["full_s"] >= SEAL_S - 1e-9:
        MEM["grade"] = "full"
    elif MEM["partial_s"] >= SEAL_S - 1e-9 and MEM["grade"] == "none":
        MEM["grade"] = "partial"
    if DEBUG and int(round(MEM["t"] * CONTROL_HZ)) % int(CONTROL_HZ) == 0:  # once a second
        base = _cup_frame(objects)[:3, 3]
        print(f"[cup_sealer dbg] t={MEM['t']:.1f} lever={a:.1f} cup=({base[0]:.3f},{base[1]:.3f},{base[2]:.3f}) "
              f"tilt={tilt_deg(objects):.1f} seated={seated(objects)} rim={pts} grade={MEM['grade']} "
              f"spill={MEM['spilled']} pinch={MEM['pinch']} tips={[tuple(round(float(v), 3) for v in p) for p in tips]}",
              flush=True)
    if MEM["grade"] != "none" and not MEM["reported"]:
        MEM["reported"] = True
        print(f"[cup_sealer] seal grade: {MEM['grade']} ({pts}/{RIM_POINTS} rim points within 1 mm of the plate "
              f"for {SEAL_S:g} s, at t={MEM['t']:.1f} s)", flush=True)


kit.on_step(monitor)


def seal_grade() -> str:
    """'full', 'partial', 'pressed' (contact, not held 3 s) or 'none'."""
    if MEM["grade"] != "none":
        return MEM["grade"]
    return "pressed" if MEM["pressed"] else "none"


def score(objects) -> float:
    """0 after a spill or a gripper in the pinch zone while the head came down. Otherwise by the seal grade: 0.3 for
    the cup seated in the holder (no seal); 0.45 pressed (three quarters of the rim under the plate) but not held 3 s;
    0.55 a partial seal (27-35 of 36 rim points held 3 s); 0.7 a full seal (all 36 held 3 s); 1.0 a full seal and then
    the lever let back up (25 degrees or more) with the cup still seated."""
    if MEM["spilled"] or MEM["pinch"]:
        return 0.0
    g = seal_grade()
    if g == "none" and not seated(objects):
        return 0.0
    s = GRADE_SCORE[g]
    if g == "full" and seated(objects) and lever_deg(objects) >= UP_AGAIN_DEG:
        s = 1.0
    return round(s, 3)


def success(objects) -> bool:
    return score(objects) >= 0.999 and objects["cup"].resting()


def distance(objects) -> float:
    base = _cup_frame(objects)[:3, 3]
    return float(np.linalg.norm(base - np.array([*POCKET_C, FLOOR])) + math.radians(max(0.0, lever_deg(objects))) * 0.1)


# -- scripted demo ------------------------------------------------------------------------------------------------
# yam_arena's Waypoint grips straight down; this demo also needs horizontal grips, so a waypoint may carry ``rot``,
# a full TCP rotation matrix. ArmPlan.target is extended to honour it (stock waypoints are unchanged).

def _target(self, t):
    prev_pos, prev_rot, prev_finger = self.start_pos, self.start_rot, self.finger_start
    for w in self.waypoints:
        goal = np.asarray(w.pos, dtype=float)
        rot = getattr(w, "rot", None)
        rot = grip_rotation(w.yaw) if rot is None else np.asarray(rot, float)
        if t < w.duration:
            s = ik.smoothstep(t / w.duration)
            return prev_pos + s * (goal - prev_pos), ik.slerp_matrix(prev_rot, rot, s), \
                prev_finger + s * (w.finger - prev_finger), w.label
        t -= w.duration
        prev_pos, prev_rot, prev_finger = goal, rot, w.finger
    return prev_pos, prev_rot, prev_finger, "done"


_demo.ArmPlan.target = _target


class RotWaypoint(Waypoint):
    """A Waypoint with a full TCP rotation."""

    def __init__(self, duration, pos, finger, label="", rot=None):
        super().__init__(duration, tuple(float(v) for v in pos), finger, label, 0.0)
        object.__setattr__(self, "rot", None if rot is None else np.asarray(rot, float))


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


# Horizontal grip: the TCP's approach axis (local z) along world +x, the jaws closing along world y.
H_FWD = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


# The same, turned 90 degrees about the approach axis: the jaws close up and down (on a horizontal bar).
H_FWD_VERT = H_FWD @ _rz(math.pi / 2)


def handle_point(angle_deg: float, dy: float = 0.0) -> np.ndarray:
    """The handle bar's centre (offset dy along it) at a lever angle."""
    r = _ry(math.radians(angle_deg))  # the lever turns +angle about +y; that raises its front (-x) end
    p = r @ np.array([HANDLE_DX, dy, 0.0])
    return PIVOT + p


CUP_GRIP_Z = 0.120  # grip the cup 120 mm above its base, across its top band (88 mm wide) just under the rim: 4 cm
# above the drink's centre of mass (about 82 mm up), so the held cup hangs upright from the fingers like a pendulum
OPEN_CUP = -0.046
CLOSED_CUP = 0.0
SETTLE = 0.8


PRESS_DEG = -16.0  # the handle target while pressing (the plate stops on the rim at 0 degrees)
CUP_PITCH_DEG = 50.0  # the cup grip points 50 degrees down. Steep, so each pad meets the cup along a long line and
# holds it from turning about the jaws' axis (a level grip touches it along a short one, and the cup swings)


def _left_plan():
    c0 = np.array([*CUP_START, TOP + CUP_GRIP_Z])
    lift = TOP + BASE_H + 0.015 + CUP_GRIP_Z  # the cup's base 15 mm above the base top
    over = np.array([*POCKET_C, lift])
    set_down = np.array([*POCKET_C, FLOOR + 0.002 + CUP_GRIP_Z])
    R = _ry(math.radians(CUP_PITCH_DEG)) @ H_FWD
    a = R[:, 2]  # the approach direction
    back = set_down + np.array([-0.08, 0.0, 0.0])  # straight back first: the 95 mm rim is wider than the open jaws
    park = np.array([0.18, 0.26, TOP + 0.15])
    return [
        RotWaypoint(2.0, c0 - 0.12 * a, OPEN_CUP, "behind the cup, grip pointing forward and down", R),
        RotWaypoint(1.5, c0, OPEN_CUP, "around the cup", R),
        RotWaypoint(0.8, c0, CLOSED_CUP, "grip", R),
        RotWaypoint(1.2, c0 + [0, 0, lift - c0[2]], CLOSED_CUP, "lift", R),
        RotWaypoint(2.5, over, CLOSED_CUP, "over the holder, under the raised head", R),
        RotWaypoint(SETTLE, over, CLOSED_CUP, "settle", R),
        RotWaypoint(1.2, set_down, CLOSED_CUP, "lower into the holder", R),
        RotWaypoint(SETTLE, set_down, CLOSED_CUP, "settle", R),
        RotWaypoint(0.8, set_down, OPEN_CUP, "let go", R),
        RotWaypoint(1.5, back, OPEN_CUP, "pull straight back, under the rim, out of the machine", R),
        RotWaypoint(2.0, park, FINGER_OPEN, "park clear", R),
    ]


def _bar_grip(point, prev, base):
    """The bar grip the arm reaches best at ``point``: jaws closing across the round bar (their closing direction
    within 20 degrees of perpendicular to it), approach turned by (yaw, pitch); joint-limit margins under 15 degrees
    cost extra."""
    from yam_arena.constants import READY
    best = None
    for yaw in range(-60, 61, 10):
        for pitch in range(-30, 61, 10):
            rot = _rz(math.radians(yaw)) @ _ry(math.radians(pitch)) @ H_FWD_VERT
            jaw = rot[:, 0]  # the jaws close along the TCP's local x
            if abs(jaw[1]) > math.sin(math.radians(20)):
                continue
            q, pe, re = ik.solve(np.asarray(point, float), ik.matrix_to_quat_xyzw(rot), np.array(READY, float),
                                 base_pos=base, iters=120)
            margin = math.degrees(float(np.min(np.minimum(q - ik.LOWER, ik.UPPER - q))))
            cost = (pe * 1000 + math.degrees(re) + 2.0 * max(0.0, 15.0 - margin)
                    + 0.02 * (abs(yaw - prev[0]) + abs(pitch - prev[1])))
            if best is None or cost < best[0]:
                best = (cost, (yaw, pitch), rot)
    return best[1], best[2]


BACK_OFF = 0.05
APPROACH = 0.05
MID = np.array([0.20, -0.15, TOP + 0.30])
PARK_R = np.array([0.22, -0.22, TOP + 0.15])
_GRIPS: dict = {}
SEARCH_GRIPS = False  # True: search each bar grip (slow; how BAR_GRIP was chosen)
BAR_GRIP = (10, 0)  # (yaw, pitch) degrees of the bar grip: 10 degrees toward the machine, level


def _bar_rot(yaw, pitch):
    return _rz(math.radians(yaw)) @ _ry(math.radians(pitch)) @ H_FWD_VERT


def _right_plan(wait_s: float):
    from yam_arena import station
    base = station.BASES["right"]
    park = PARK_R
    dy = -0.035  # grip the bar right of centre
    grip = (0, 0)
    cache = {}

    def at(angle, back=0.0):
        nonlocal grip
        p0 = handle_point(angle, dy)
        if angle not in _GRIPS:
            _GRIPS[angle] = _bar_grip(p0, grip, base) if SEARCH_GRIPS else (BAR_GRIP, _bar_rot(*BAR_GRIP))
        grip, rot = _GRIPS[angle]
        return p0 - back * rot[:, 2], rot

    p_up, r_up = at(UP_DEG)
    p_app, _ = at(UP_DEG, APPROACH)
    down = grip_rotation(0.0)
    way = [RotWaypoint(wait_s, park, FINGER_OPEN, "wait clear", down),
           RotWaypoint(2.0, MID, -0.03, "turn the grip on the way", r_up),
           RotWaypoint(1.5, p_app, -0.03, "in front of the handle", r_up),
           RotWaypoint(1.5, p_up, -0.03, "onto the handle", r_up),
           RotWaypoint(0.8, p_up, 0.0, "grip the handle", r_up)]
    # The arm's joints are position servos that give under load: against the spring (about 10 N at the handle) the
    # handle lags its target by some 8 degrees, so the targets run that far below the arc, down to PRESS_DEG.
    for a in (25.0, 15.0, 5.0, PRESS_DEG / 2):  # down the arc, slower near the rim
        p, r = at(a)
        way.append(RotWaypoint(0.9 if a > 5 else 0.8, p, 0.0, f"pull down to {a:g} deg", r))
    p, r = at(PRESS_DEG)
    way.append(RotWaypoint(4.0, p, 0.0, "press on the rim and hold 4 s (seal)", r))
    for a in (10.0, 25.0, UP_DEG - 1.0):  # let it back up, guiding it
        p, r = at(a)
        way.append(RotWaypoint(0.9, p, 0.0, f"up to {a:g} deg", r))
    p, r = at(UP_DEG - 1.0)
    way.append(RotWaypoint(0.6, p, -0.03, "let go", r))
    p2 = p + np.array([-BACK_OFF, 0.0, 0.0])  # straight back toward the robot, off the bar
    way.append(RotWaypoint(1.2, p2, -0.03, "back off", r))
    way.append(RotWaypoint(2.0, park, FINGER_OPEN, "park", down))
    return way


def demo():
    """The left arm takes the cup with a horizontal grip, carries it under the raised head, sets it into the holder,
    lets go and pulls straight back out of the machine. Then the right arm grips the handle bar and pulls the lever
    down along its arc, holds the head on the rim for 4 s, guides the lever back up and lets go."""
    left = _left_plan()
    t_left = sum(w.duration for w in left)
    return {"left": left, "right": _right_plan(t_left)}


# -- offline self-test --------------------------------------------------------------------------------------------

def _states(cup_pose=None, angle_deg=UP_DEG):
    out = {"sealer_body": kit.state("sealer_body"), "lever": kit.state("lever", {"lever_pivot": math.radians(angle_deg)})}
    out["cup"] = kit.state("cup", pose=cup_pose) if cup_pose is not None else kit.state("cup")
    return out


def _cup_at(x, y, z, tilt_deg_=0.0):
    t = hk.T(_rx(math.radians(tilt_deg_)), (x, y, z))
    return t


def _touch_angle(cup_pose):
    """The lever angle where the plate first touches a (tilted) cup's highest rim point (offline tests)."""
    lo, hi = -3.0, UP_DEG
    for _ in range(40):
        mid = (lo + hi) / 2
        st = {"cup": kit.state("cup", pose=cup_pose), "lever": kit.state("lever", {"lever_pivot": math.radians(mid)})}
        c = _cup_frame(st)
        rim = _RIM_LOCAL @ c[:3, :3].T + c[:3, 3]
        lv = kit.frame(st, "lever")
        gap = -((rim - (lv[:3, :3] @ _PLATE_LOCAL + lv[:3, 3])) @ lv[:3, 2])
        lo, hi = (mid, hi) if gap.min() < 0 else (lo, mid)  # pressed into the rim: raise the lever
    return (lo + hi) / 2


def _selftest() -> None:
    seated_pose = _cup_at(*POCKET_C, FLOOR)
    cases = []

    def run(name, frames, want):
        _reset()
        s = None
        for st, grips in frames:
            monitor(st, grips)
            s = score(st)
        ok = abs(s - want) < 1e-9
        cases.append(ok)
        print(f"{'ok ' if ok else 'BAD'} {name}: {s} (want {want})")

    far = [np.array([0.15, 0.25, TOP + 0.15]), np.array([0.15, -0.25, TOP + 0.15])]
    up = _states(seated_pose, UP_DEG)
    down = _states(seated_pose, 0.0)
    run("cup on the table", [(_states(None, UP_DEG), far)], 0.0)
    run("cup seated, lever up", [(up, far)], 0.3)
    run("seated, pressed 2 s (too short)", [(up, far)] + [(down, far)] * 40, 0.45)
    _reset(); print("   seated cup, lever at 0: rim points", rim_contact(down))
    run("seated, pressed 3 s, still down", [(up, far)] + [(down, far)] * 61, 0.7)
    run("sealed and lever back up", [(up, far)] + [(down, far)] * 61 + [(up, far)], 1.0)
    in_zone = [np.array([*POCKET_C, FLOOR + 0.08]), far[1]]
    run("hand left in the holder while head comes down", [(up, far), (_states(seated_pose, 10.0), in_zone)]
        + [(down, far)] * 61 + [(up, far)], 0.0)
    run("hand in the holder while lever is up (allowed)", [(up, in_zone)] + [(down, far)] * 61 + [(up, far)], 1.0)
    high = _cup_at(*POCKET_C, FLOOR + 0.020)  # standing on the holder's edge, not in it
    run("cup not in the holder, lever pushed down", [(_states(high, 0.0), far)] * 25, 0.0)
    off = _cup_at(POCKET_C[0] + 0.009, POCKET_C[1], FLOOR)
    run("cup 9 mm off centre", [(_states(off, UP_DEG), far)], 0.0)
    for deg, want in ((0.6, 0.55), (3.5, 0.3)):  # seated (under 4 degrees) but tilted: the plate meets part of the rim
        tilted = _cup_at(*POCKET_C, FLOOR, deg)
        touch = _states(tilted, _touch_angle(tilted))
        print(f"   cup tilted {deg} deg, plate resting on it: rim points {rim_contact(touch)}")
        run(f"cup seated but tilted {deg} deg, plate on its high side", [(_states(tilted, UP_DEG), far)]
            + [(touch, far)] * 61 + [(_states(tilted, UP_DEG), far)], want)
    run("cup tipped over", [(_states(_cup_at(0.25, 0.20, TOP + 0.04, 80.0), UP_DEG), far)], 0.0)
    run("spill then sealed", [(_states(_cup_at(0.25, 0.20, TOP, 30.0), UP_DEG), far)] + [(down, far)] * 61
        + [(up, far)], 0.0)
    assert all(cases), "scoring self-test failed"
    print(f"scoring: {len(cases)} cases pass")

    from yam_arena import station
    for side, way in demo().items():
        plan = _demo.ArmPlan(way, station.BASES[side])
        t = 0.0
        while t < plan.duration:
            plan.action7(t)
            t += 1 / CONTROL_HZ
        print(f"{side}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.2f} mm / {plan.worst_ik_deg:.2f} deg")


if __name__ == "__main__":
    _selftest()
