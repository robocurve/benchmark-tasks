"""On the station table, in front of the two arms: a long parcel lies across the arms' view (its length side to side)
between two other parcels, one on each side, with only about 2 cm to spare at each end. Ahead of it, two yellow
bollards mark the mouth of a loading bay taped out in red on the table, which ends at a fixed stop bar. The long parcel
has to end up lying lengthwise in the bay, pushed up to the stop bar, without bumping either neighbour. The bollards
are closer together (37 cm) than the parcel is long (42 cm), so it cannot go forward lying across: it has to be turned
a quarter turn first, right where it lies. Pushed at one end only, a parcel swings about its other end and slides
sideways into a neighbour; turned by both ends at once in opposite directions (a pure turn), its corners sweep a
circle that just clears the neighbours and the bollards. Once turned, it slides between the bollards into the bay.

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475.
- long parcel: brown corrugated cardboard box, 14 cm front to back x 42 cm side to side x 10 cm tall, 800 g, centred
  at (0.30, 0), its long sides parallel to the y axis. Turning about its centre, its corners sweep a circle 22.1 cm
  in radius.
- neighbours: two white cardboard boxes, 14 x 20 x 10 cm, 1.2 kg each, centred at (0.30, 0.333) and (0.30, -0.333):
  their inner faces are 2.3 cm beyond the long parcel's ends and 1.2 cm outside the circle its corners sweep.
- bollards: two fixed yellow posts 3 x 3 cm, 8 cm tall, centred at (0.48, 0.20) and (0.48, -0.20): 37 cm apart
  inside, 2.7 cm outside the circle the parcel's corners sweep.
- bay: red tape (1.5 cm wide, 0.5 mm thick) marking a rectangle 20 cm wide (y -0.10 to 0.10, inside the tape) from
  x 0.20 to x 0.66 on the table; it runs under the parcel's start position.
- stop bar: fixed dark grey bar, 2 cm front to back x 22 cm wide x 4 cm tall, its back face at x 0.66 (the bay's far
  end).

Full score: the long parcel lies flat in the bay (all four bottom corners inside the red rectangle), its long sides
within 5 degrees of front to back, still for 2 s; and at no time did its centre move more than 4 cm to either side of
the centre line, or either neighbour move more than 1 cm.
"""

import os

import numpy as np

from yam_arena.constants import FINGER_CLOSED
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 2.0

KRAFT = (0.62, 0.47, 0.30)
WHITE_BOX = (0.90, 0.89, 0.86)
YELLOW = (0.95, 0.78, 0.10)
TAPE_RED = (0.82, 0.10, 0.10)
DARK = (0.20, 0.21, 0.23)

PARCEL = (0.14, 0.42, 0.10)  # x, y, z at the start
PARCEL_KG = 0.8
CENTRE = np.array([0.30, 0.0])
CX = 0.30  # plain float for object positions (the simulator rejects NumPy scalars)
NEIGHBOUR = (0.14, 0.20, 0.10)
NEIGHBOUR_KG = 1.2
SWEEP_R = float(np.hypot(PARCEL[0], PARCEL[1]) / 2)  # 0.221
NEIGHBOUR_Y = SWEEP_R + 0.012 + NEIGHBOUR[1] / 2  # 0.333
BOLLARD, BOLLARD_XY = (0.03, 0.03, 0.08), (0.48, 0.20)
BAY_X, BAY_HALF_Y = (0.20, 0.66), 0.10  # inside of the taped rectangle
TAPE_W, TAPE_T = 0.015, 0.0005
STOP = (0.02, 0.22, 0.04)


def _tape():
    x0, x1 = BAY_X
    z = TOP + TAPE_T / 2
    side_len = x1 - x0
    return [
        Box("tape_back", pos=(x0 - TAPE_W / 2, 0.0, z), size=(TAPE_W, 2 * (BAY_HALF_Y + TAPE_W), TAPE_T),
            color=TAPE_RED),
        Box("tape_left", pos=((x0 + x1) / 2, BAY_HALF_Y + TAPE_W / 2, z), size=(side_len, TAPE_W, TAPE_T),
            color=TAPE_RED),
        Box("tape_right", pos=((x0 + x1) / 2, -(BAY_HALF_Y + TAPE_W / 2), z), size=(side_len, TAPE_W, TAPE_T),
            color=TAPE_RED),
    ]


OBJECTS = _tape() + [
    Box("stop_bar", pos=(BAY_X[1] + STOP[0] / 2, 0.0, TOP + STOP[2] / 2), size=STOP, color=DARK),
    Box("bollard_left", pos=(BOLLARD_XY[0], BOLLARD_XY[1], TOP + BOLLARD[2] / 2), size=BOLLARD, color=YELLOW),
    Box("bollard_right", pos=(BOLLARD_XY[0], -BOLLARD_XY[1], TOP + BOLLARD[2] / 2), size=BOLLARD, color=YELLOW),
    Box("parcel", pos=(CX, 0.0, TOP + 0.001 + PARCEL[2] / 2), size=PARCEL, mass=PARCEL_KG, color=KRAFT),
    Box("neighbour_left", pos=(CX, NEIGHBOUR_Y, TOP + 0.001 + NEIGHBOUR[2] / 2), size=NEIGHBOUR,
        mass=NEIGHBOUR_KG, color=WHITE_BOX),
    Box("neighbour_right", pos=(CX, -NEIGHBOUR_Y, TOP + 0.001 + NEIGHBOUR[2] / 2), size=NEIGHBOUR,
        mass=NEIGHBOUR_KG, color=WHITE_BOX),
]
NEIGHBOUR_START = {"neighbour_left": np.array([CENTRE[0], NEIGHBOUR_Y]),
                   "neighbour_right": np.array([CENTRE[0], -NEIGHBOUR_Y])}

# Thresholds, as a reviewer would check them with a tape measure and a protractor (and the video for "at no time").
YAW_TOL, YAW_TOL_HALF = np.radians(5.0), np.radians(10.0)
IN_BAY_SLACK = 0.005  # a corner counts as inside the tape when within 5 mm of its inner edge
SIDE_MAX = 0.04  # "at any time": the parcel's centre more than 4 cm to either side of the centre line scores 0
BUMP_MAX = 0.01  # "at any time": a neighbour moved more than 1 cm scores 0
FLAT_MAX = np.radians(5)

_DEBUG = os.environ.get("TP_DEBUG") == "1"
_state = {"side": False, "bump": False, "max_side": 0.0, "max_bump": 0.0, "step": 0}


def _yaw_tilt(q):
    x, y, z, w = q
    return (float(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))),
            float(np.arccos(np.clip(1.0 - 2.0 * (x * x + y * y), -1.0, 1.0))))


def _turn_error(yaw: float) -> float:
    """Angle between the parcel's long sides and the x axis: 0 when turned 90 degrees either way (a box looks the
    same turned 180 degrees)."""
    d = (yaw - np.pi / 2) % np.pi
    return float(min(d, np.pi - d))


def _latch(objects) -> None:
    side = abs(float(objects["parcel"].pos[1]))
    _state["max_side"] = max(_state["max_side"], side)
    if side > SIDE_MAX:
        _state["side"] = True
    for name, start in NEIGHBOUR_START.items():
        moved = float(np.linalg.norm(np.asarray(objects[name].pos[:2]) - start))
        _state["max_bump"] = max(_state["max_bump"], moved)
        if moved > BUMP_MAX:
            _state["bump"] = True


def _corners(p):
    yaw, _ = _yaw_tilt(p.quat_xyzw)
    c, s = np.cos(yaw), np.sin(yaw)
    hx, hy = PARCEL[0] / 2, PARCEL[1] / 2
    local = np.array([[hx, hy], [hx, -hy], [-hx, hy], [-hx, -hy]])
    return np.asarray(p.pos[:2]) + local @ np.array([[c, s], [-s, c]])


def _in_bay(p) -> bool:
    k = _corners(p)
    return bool(np.all(k[:, 0] >= BAY_X[0] - IN_BAY_SLACK) and np.all(k[:, 0] <= BAY_X[1] + IN_BAY_SLACK)
                and np.all(np.abs(k[:, 1]) <= BAY_HALF_Y + IN_BAY_SLACK))


def _pose(objects):
    p = objects["parcel"]
    yaw, tilt = _yaw_tilt(p.quat_xyzw)
    return _turn_error(yaw), _in_bay(p), tilt


def success(objects):
    _latch(objects)
    err, in_bay, tilt = _pose(objects)
    if _DEBUG:
        _state["step"] += 1
        if _state["step"] % 10 == 1:
            p = objects["parcel"]
            yaw = np.degrees(_yaw_tilt(p.quat_xyzw)[0])
            print(f"DBG step={_state['step']} pos=({p.pos[0]:.3f},{p.pos[1]:+.3f}) yaw={yaw:.1f} "
                  f"err={np.degrees(err):.1f} in_bay={in_bay} max_side={_state['max_side'] * 100:.1f}cm "
                  f"max_bump={_state['max_bump'] * 1000:.0f}mm tilt={np.degrees(tilt):.1f}", flush=True)
    still = all(objects[n].resting() for n in ("parcel", "neighbour_left", "neighbour_right"))
    ok = err <= YAW_TOL and in_bay and tilt < FLAT_MAX and still
    return bool(ok and not _state["side"] and not _state["bump"])


def distance(objects):
    p = objects["parcel"]
    err, _, _ = _pose(objects)
    target = np.array([BAY_X[1] - PARCEL[1] / 2, 0.0])  # turned and pushed up to the stop bar
    return float(np.linalg.norm(np.asarray(p.pos[:2]) - target)) + 0.2 * err


def score(objects):
    """Partial credit (success scores 1.0): 0.5 when the parcel lies flat, turned to within 10 degrees of front to
    back, but not in the bay; 0 if at any time its centre moved more than 4 cm to either side or a neighbour moved
    more than 1 cm, or otherwise."""
    _latch(objects)
    if _state["side"] or _state["bump"]:
        return 0.0
    err, _, tilt = _pose(objects)
    return 0.5 if err <= YAW_TOL_HALF and tilt < FLAT_MAX else 0.0


# -- scripted demo -------------------------------------------------------------------------------------------------
# The turn: both grippers, closed and turned with the parcel, push it near its two ends in opposite directions: the
# left one pushes the back face forward near the left end, the right one pushes the front face back near the right
# end. Each contact point is moved along its circle about the parcel's centre, in small steps, so the parcel turns
# clockwise (seen from above) about its centre. The right wrist cannot turn a full quarter turn in one go, so halfway
# the left gripper eases off to a light touch while the right one backs off, turns its fingers half a turn (the closed
# fingers look the same) and comes back; then both finish the turn together, 90 degrees plus a small overshoot.
# The push: the left gripper moves out of the way; the right one comes round behind the parcel's back end and pushes
# it straight forward between the bollards until it meets the stop bar (the commanded point runs ahead of the parcel
# because the arm gives a little under the load), holds it there a moment and backs off.
FINGER_REACH = 0.034  # closed fingers reach this far ahead of the grip point along the push (turned square to it)
TIP = 0.013
PUSH_AT = float(os.environ.get("TP_PUSH_AT", "0.15"))  # contact this far from the centre along the long side
PUSH_IN = float(os.environ.get("TP_PUSH_IN", "0.008"))  # contacts commanded this far into the faces
STEPS = int(os.environ.get("TP_STEPS", "5"))  # steps per half turn
STEP_S = float(os.environ.get("TP_STEP_S", "1.0"))
OVERSHOOT = np.radians(float(os.environ.get("TP_OVERSHOOT", "1.5")))
FWD_LEAD = float(os.environ.get("TP_FWD_LEAD", "0.10"))  # forward push: commanded point this far past the stop
FWD_S = float(os.environ.get("TP_FWD_S", "5.0"))
PUSH_Z = TOP + 0.001 + 0.03 + TIP  # pad tips 3 cm above the parcel's underside
HOVER = 0.14
LOW_HOVER = 0.10


def _rot(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s], [s, c]])


def demo():
    hx = PARCEL[0] / 2
    contacts = {"left": (np.array([-hx, PUSH_AT]), np.array([1.0, 0.0])),  # back face near the left end, push +x
                "right": (np.array([hx, -PUSH_AT]), np.array([-1.0, 0.0]))}  # front face near the right end, push -x

    def tcp(arm, yaw, depth):
        local, normal = contacts[arm]
        c = CENTRE + _rot(yaw) @ local
        n = _rot(yaw) @ normal
        return c - n * (FINGER_REACH - depth)

    mid_yaw = -np.pi / 4
    end_yaw = -np.pi / 2 - OVERSHOOT
    half1 = [mid_yaw * k / STEPS for k in range(1, STEPS + 1)]
    half2 = [mid_yaw + (end_yaw - mid_yaw) * k / STEPS for k in range(1, STEPS + 1)]
    regrasp = [0.8, 0.8, 1.2, 0.8, 0.8]  # right arm: back off, up, turn the fingers over, down, touch

    left = [
        Waypoint(2.0, (*tcp("left", 0.0, -0.02), TOP + HOVER), FINGER_CLOSED, "above the back face", 0.0),
        Waypoint(1.5, (*tcp("left", 0.0, -0.02), PUSH_Z), FINGER_CLOSED, "down behind the face", 0.0),
        Waypoint(1.0, (*tcp("left", 0.0, PUSH_IN), PUSH_Z), FINGER_CLOSED, "touch", 0.0),
    ]
    left += [Waypoint(STEP_S, (*tcp("left", y, PUSH_IN), PUSH_Z), FINGER_CLOSED, f"turn {k + 1}", float(y))
             for k, y in enumerate(half1)]
    left += [
        Waypoint(0.8, (*tcp("left", mid_yaw, 0.0), PUSH_Z), FINGER_CLOSED, "ease off to a light touch", float(mid_yaw)),
        Waypoint(sum(regrasp) - 0.8 - 0.8, (*tcp("left", mid_yaw, 0.0), PUSH_Z), FINGER_CLOSED, "wait", float(mid_yaw)),
        Waypoint(0.8, (*tcp("left", mid_yaw, PUSH_IN), PUSH_Z), FINGER_CLOSED, "touch again", float(mid_yaw)),
    ]
    left += [Waypoint(STEP_S, (*tcp("left", y, PUSH_IN), PUSH_Z), FINGER_CLOSED, f"turn {STEPS + k + 1}", float(y))
             for k, y in enumerate(half2)]
    back_l = tcp("left", end_yaw, -0.03) - np.array([0.03, 0.0])
    left += [
        Waypoint(1.0, (*back_l, PUSH_Z), FINGER_CLOSED, "back off", float(end_yaw)),
        Waypoint(1.5, (*back_l, TOP + LOW_HOVER), FINGER_CLOSED, "rise", float(end_yaw)),
        Waypoint(2.0, (0.20, 0.30, TOP + HOVER), FINGER_CLOSED, "out of the way", float(end_yaw)),
    ]

    flip = np.pi  # the right gripper's fingers turned half a turn for the second half
    right = [
        Waypoint(2.0, (*tcp("right", 0.0, -0.02), TOP + HOVER), FINGER_CLOSED, "above the front face", 0.0),
        Waypoint(1.5, (*tcp("right", 0.0, -0.02), PUSH_Z), FINGER_CLOSED, "down in front of the face", 0.0),
        Waypoint(1.0, (*tcp("right", 0.0, PUSH_IN), PUSH_Z), FINGER_CLOSED, "touch", 0.0),
    ]
    right += [Waypoint(STEP_S, (*tcp("right", y, PUSH_IN), PUSH_Z), FINGER_CLOSED, f"turn {k + 1}", float(y))
              for k, y in enumerate(half1)]
    off = tcp("right", mid_yaw, -0.03)
    right += [
        Waypoint(regrasp[0], (*off, PUSH_Z), FINGER_CLOSED, "back off", float(mid_yaw)),
        Waypoint(regrasp[1], (*off, TOP + LOW_HOVER), FINGER_CLOSED, "up", float(mid_yaw)),
        Waypoint(regrasp[2], (*off, TOP + LOW_HOVER), FINGER_CLOSED, "turn the fingers over", float(mid_yaw + flip)),
        Waypoint(regrasp[3], (*off, PUSH_Z), FINGER_CLOSED, "down", float(mid_yaw + flip)),
        Waypoint(regrasp[4], (*tcp("right", mid_yaw, PUSH_IN), PUSH_Z), FINGER_CLOSED, "touch again", float(mid_yaw + flip)),
    ]
    right += [Waypoint(STEP_S, (*tcp("right", y, PUSH_IN), PUSH_Z), FINGER_CLOSED, f"turn {STEPS + k + 1}",
                       float(y + flip)) for k, y in enumerate(half2)]
    back_r = tcp("right", end_yaw, -0.03)
    right += [
        Waypoint(1.0, (*back_r, PUSH_Z), FINGER_CLOSED, "back off", float(end_yaw + flip)),
        Waypoint(1.5, (*back_r, TOP + LOW_HOVER), FINGER_CLOSED, "rise", float(end_yaw + flip)),
    ]
    # forward push, by the right gripper alone, fingers square to the parcel's back end (closed fingers 9.6 cm wide)
    back_face = CX - PARCEL[1] / 2  # the turned parcel's back end, x
    final_face = BAY_X[1] - PARCEL[1]  # where that end is once the parcel meets the stop bar
    behind = back_face - FINGER_REACH - 0.015
    right += [
        Waypoint(0.8, (*back_r, TOP + HOVER), FINGER_CLOSED, "up, clear of the parcel", float(end_yaw + flip)),
        Waypoint(2.0, (behind, 0.0, TOP + HOVER), FINGER_CLOSED, "round behind the parcel", 0.0),
        Waypoint(1.5, (behind, 0.0, PUSH_Z), FINGER_CLOSED, "down", 0.0),
        Waypoint(0.8, (back_face - FINGER_REACH + PUSH_IN, 0.0, PUSH_Z), FINGER_CLOSED, "touch the back end", 0.0),
        Waypoint(FWD_S, (final_face - FINGER_REACH + FWD_LEAD, 0.0, PUSH_Z), FINGER_CLOSED,
                 "push it forward to the stop bar", 0.0),
        Waypoint(1.0, (final_face - FINGER_REACH + FWD_LEAD, 0.0, PUSH_Z), FINGER_CLOSED, "hold", 0.0),
        Waypoint(1.0, (final_face - FINGER_REACH - 0.03, 0.0, PUSH_Z), FINGER_CLOSED, "back off", 0.0),
        Waypoint(1.2, (final_face - FINGER_REACH - 0.03, 0.0, TOP + HOVER), FINGER_CLOSED, "rise", 0.0),
    ]
    end = max(sum(w.duration for w in right), sum(w.duration for w in left))
    for plan in (left, right):
        if sum(w.duration for w in plan) < end:
            last = plan[-1]
            plan.append(Waypoint(end - sum(w.duration for w in plan), last.pos, last.finger, "wait", last.yaw))
    return {"left": left, "right": right}
