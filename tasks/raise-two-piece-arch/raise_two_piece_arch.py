"""On the station table: the two halves of a small wooden arch. Each half is an L of pine: an upright post 15 cm tall
with an arm 11 cm long along its top that overhangs one side, so the half is top-heavy towards its arm and falls over
on its own. They stand at the front, one on each side with its arm towards the middle, each propped up by a dark
block under the end of its arm. Two blue tape marks further back, 25 cm apart, show where the posts go: there the
ends of the two arms meet in the middle, the halves lean on each other and stand as an arch. As neither half stands
alone, both have to be set down together, one in each gripper, and let go of at the same time.

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475.
- halves: free, 180 g each: a 3 x 5 x 15 cm post with an 11 x 5 x 3 cm arm on its top, overhanging one side. The
  left half's post stands at (0.17, 0.255), its arm towards the middle (its end at y 0.13); the right half mirrors it.
- props: fixed dark grey blocks, 5 x 2 x 12 cm, under the ends of the arms at (0.17, +-0.145).
- marks: fixed blue tape squares, 6 x 5 cm and 1 mm thick, centred at (0.31, +-0.125): one post on each.

Full score: one post on each mark (within 1.5 cm), both halves upright (within 5 degrees) and standing on their own,
the ends of their arms touching (within 2 cm), both grippers clear of them (TCP 3 cm or more away), still for 2 s.
"""

import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from compound_body import TCP_KEYS, Part, compound, quat_matrix  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 2.0
SCORE_OVER = "best"  # partial credit: the best reached during the run

PINE = (0.80, 0.62, 0.40)
PROP_GREY = (0.28, 0.29, 0.31)
TAPE_BLUE = (0.16, 0.36, 0.78)

# One half, in its own frame: origin at the centre of its bounding box, the arm towards -y (the left half as it
# stands; the right half is the same body turned 180 degrees).
DEPTH, POST_W, HEIGHT = 0.05, 0.03, 0.15
ARM_L, ARM_T = 0.11, 0.03
HALF_KG = 0.18
HALF_W = ARM_L + POST_W  # 0.14
POST = Part((0.0, HALF_W / 2 - POST_W / 2, 0.0), (DEPTH, POST_W, HEIGHT))
ARM = Part((0.0, -HALF_W / 2 + ARM_L / 2, HEIGHT / 2 - ARM_T / 2), (DEPTH, ARM_L, ARM_T))
ARM_END = np.array([0.0, -HALF_W / 2, HEIGHT / 2 - ARM_T / 2])  # the middle of the arm's end face
POST_FOOT = np.array([0.0, HALF_W / 2 - POST_W / 2, -HEIGHT / 2])  # the middle of the post's foot

START_X, START_Y = 0.17, 0.20  # the left half's origin at the start: post at y 0.255, arm's end at y 0.13
MARK_X, MARK_Y = 0.31, 0.07  # the left half's origin on its mark: post at y 0.125, arm's end at y 0
TAPE = (0.06, 0.05, 0.001)
PROP = (DEPTH, 0.02, HEIGHT - ARM_T)  # 12 cm tall: the arm's underside rests on it
PROP_Y = START_Y - HALF_W / 2 + 0.015  # 0.145, 5 mm in from the arm's end
FOOT_Y = MARK_Y + POST.center[1]  # 0.125
MARKS = np.array([[MARK_X, FOOT_Y], [MARK_X, -FOOT_Y]])
HALVES = ("half_left", "half_right")


def _half(name: str, side: int):
    return compound(name, (START_X, side * START_Y, TOP + HEIGHT / 2 + 0.0005), (POST, ARM), mass=HALF_KG,
                    color=PINE, yaw=0.0 if side > 0 else np.pi)


SPECS = {n: _half(n, s) for n, s in zip(HALVES, (1, -1))}

OBJECTS = [
    Box("mark_left", pos=(MARK_X, FOOT_Y, TOP + TAPE[2] / 2), size=TAPE, color=TAPE_BLUE),
    Box("mark_right", pos=(MARK_X, -FOOT_Y, TOP + TAPE[2] / 2), size=TAPE, color=TAPE_BLUE),
    Box("prop_left", pos=(START_X, PROP_Y, TOP + PROP[2] / 2), size=PROP, color=PROP_GREY),
    Box("prop_right", pos=(START_X, -PROP_Y, TOP + PROP[2] / 2), size=PROP, color=PROP_GREY),
    *SPECS.values(),
]

# Thresholds, as a reviewer would check them from the video and with a ruler and a protractor.
UPRIGHT = np.radians(5.0)
ON_MARK = 0.015
ENDS_MEET = 0.02
CLEAR = 0.03  # a gripper's TCP at least this far from both halves: it has let go
FOOT_ON_TABLE = 0.006  # the post's foot at most this high above the table (or a mark)
IN_AIR = 0.01  # a half's lowest point this high above the table: it is held up

_DEBUG = os.environ.get("ARCH_DEBUG") == "1"
_state = {"step": 0}


def _world(h, p) -> np.ndarray:
    return h.pos + quat_matrix(h.quat_xyzw) @ p


def _tilt(q) -> float:
    x, y, _, _ = q
    return float(np.arccos(np.clip(1.0 - 2.0 * (x * x + y * y), -1.0, 1.0)))


def _standing(h) -> bool:
    """Upright, the post's foot down on the table (or a mark)."""
    return _tilt(h.quat_xyzw) <= UPRIGHT and _world(h, POST_FOOT)[2] <= TOP + TAPE[2] + FOOT_ON_TABLE


def _ends_gap(objects) -> float:
    a, b = (objects[n] for n in HALVES)
    return float(np.linalg.norm(_world(a, ARM_END) - _world(b, ARM_END)))


def _arch(objects) -> bool:
    """Both halves stand upright, still, the ends of their arms touching: the two lean on each other."""
    a, b = (objects[n] for n in HALVES)
    return _standing(a) and _standing(b) and _ends_gap(objects) <= ENDS_MEET and a.resting() and b.resting()


def _on_marks(objects) -> bool:
    feet = [_world(objects[n], POST_FOOT)[:2] for n in HALVES]

    def near(f, m):
        return np.hypot(*(f - m)) <= ON_MARK

    return bool((near(feet[0], MARKS[0]) and near(feet[1], MARKS[1]))
                or (near(feet[0], MARKS[1]) and near(feet[1], MARKS[0])))


def _tcps(objects) -> list:
    return [objects[k].pos for k in TCP_KEYS.values() if k in objects]


def _clear(objects) -> bool:
    """Neither gripper holds a half: both TCPs at least CLEAR from both halves."""
    return all(SPECS[n].distance(objects[n], p) >= CLEAR for n in HALVES for p in _tcps(objects))


def _both_in_air(objects) -> bool:
    return all(objects[n].low[2] >= TOP + IN_AIR for n in HALVES)


def _debug(objects) -> None:
    _state["step"] += 1
    if _state["step"] % (2 if 280 <= _state["step"] <= 360 else 10) != 1:
        return
    parts = []
    for n in HALVES:
        h = objects[n]
        foot = _world(h, POST_FOOT)
        x, y, z, w = h.quat_xyzw
        rpy = np.degrees([np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
                          np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)),
                          np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))])
        parts.append(f"{n[5:]}: origin=({h.pos[0]:.3f},{h.pos[1]:.3f},{h.pos[2] - TOP:.3f}) "
                     f"foot=({foot[0]:.3f},{foot[1]:.3f},{foot[2] - TOP:.3f}) tilt={np.degrees(_tilt(h.quat_xyzw)):.1f} "
                     f"rpy=({rpy[0]:.1f},{rpy[1]:.1f},{rpy[2]:.1f}) low={h.low[2] - TOP:.3f} rest={h.resting()}")
    tcps = " ".join(f"{k}=({objects[k].pos[0]:.3f},{objects[k].pos[1]:.3f},{objects[k].pos[2] - TOP:.3f})"
                    for k in TCP_KEYS.values() if k in objects)
    print(f"DBG step={_state['step']} {' | '.join(parts)} | gap={_ends_gap(objects):.3f} arch={_arch(objects)} "
          f"marks={_on_marks(objects)} clear={_clear(objects)} air={_both_in_air(objects)} {tcps}", flush=True)


def success(objects):
    if _DEBUG:
        _debug(objects)
    return _arch(objects) and _on_marks(objects) and _clear(objects)


def distance(objects):
    feet = [_world(objects[n], POST_FOOT)[:2] for n in HALVES]
    straight = max(np.hypot(*(feet[0] - MARKS[0])), np.hypot(*(feet[1] - MARKS[1])))
    swapped = max(np.hypot(*(feet[0] - MARKS[1])), np.hypot(*(feet[1] - MARKS[0])))
    return float(min(straight, swapped))


def score(objects):
    """Partial credit (success scores 1.0; the best over the run counts): 0.5 when the two halves stand on their
    own leaning on each other, but not on the marks; 0.25 when both halves are off the table at the same time;
    0 otherwise."""
    if _arch(objects) and _clear(objects):
        return 0.5
    if _both_in_air(objects):
        return 0.25
    return 0.0


# -- scripted demo -------------------------------------------------------------------------------------------------
# Each gripper takes one half by the top of its corner (turned 90 degrees, the fingers closing front to back), lifts
# it 2 cm off the table and its prop, carries it back to the marks' line and in towards the middle until the ends of
# the two arms meet, sets it down on its mark and holds it for a second; both let go at the same moment, rise clear
# and back away. Both arms' waypoints have the same durations, so they move, set down and let go together.
TIP = 0.013  # pad tips below the TCP
OPEN = FINGER_OPEN
PADS_DOWN = float(os.environ.get("ARCH_PADS_DOWN", "0.035"))  # the pads hold the top 3.5 cm of the post
LIFT = 0.02
GRIP_IN = float(os.environ.get("ARCH_GRIP_IN", "0.055"))  # grip on the post: from the half's origin towards it (the
# centre of mass is 2.5 cm that way)
# Close the fingers only to 4.2 cm apart round the 5 cm deep post: each finger presses ~20 N, below the drive's 50 N
# limit, so the two stay balanced and the post stays centred between them (fully closed, both saturate and the half
# drifts ~2.5 cm off centre and pitches).
GRIP = float(os.environ.get("ARCH_GRIP", "-0.021"))
YAW = {"left": -np.pi / 2, "right": np.pi / 2}
# Where a held half sits off the TCP along the closing direction, if anywhere: aim each TCP that much the other way
# once it holds its half.
DX = {"left": float(os.environ.get("ARCH_DX_LEFT", "0.0")), "right": float(os.environ.get("ARCH_DX_RIGHT", "0.0"))}
RELEASE = float(os.environ.get("ARCH_RELEASE", "-0.030"))  # let go by opening the fingers ~1 cm, gently
PARK = (0.20, 0.25)


def demo():
    tcp_grip = TOP + HEIGHT + 0.0005 + TIP - PADS_DOWN
    tcp_lift = tcp_grip + LIFT
    tcp_set = tcp_grip + TAPE[2] - 0.0005 - 0.002  # the post's foot on the tape, pressed 2 mm
    tcp_high = tcp_grip + 0.05
    plan = {}
    for side, s in (("left", 1), ("right", -1)):
        yaw, dx = YAW[side], DX[side]
        y0, y1 = s * (START_Y + GRIP_IN), s * (MARK_Y + GRIP_IN)
        x0, x1 = START_X, MARK_X + dx
        plan[side] = [
            Waypoint(2.0, (x0, y0, tcp_grip + 0.05), OPEN, "above the half's corner", yaw),
            Waypoint(1.2, (x0, y0, tcp_grip), OPEN, "down round its top", yaw),
            Waypoint(1.0, (x0, y0, tcp_grip), GRIP, "grip", yaw),
            Waypoint(0.4, (x0, y0, tcp_grip), GRIP, "steady", yaw),
            Waypoint(1.0, (x0, y0, tcp_lift), GRIP, "lift it off the table and its prop", yaw),
            Waypoint(2.5, (x1, y0, tcp_lift), GRIP, "carry it back to the marks' line", yaw),
            Waypoint(2.5, (x1, y1 + s * 0.01, tcp_lift), GRIP, "in towards the middle", yaw),
            Waypoint(1.5, (x1, y1, tcp_lift), GRIP, "the ends of the arms meet", yaw),
            Waypoint(0.8, (x1, y1, tcp_lift), GRIP, "steady", yaw),
            Waypoint(1.5, (x1, y1, tcp_set), GRIP, "set both down on their marks", yaw),
            Waypoint(1.0, (x1, y1, tcp_set), GRIP, "hold", yaw),
            Waypoint(0.8, (x1, y1, tcp_set), RELEASE, "let go together", yaw),
            Waypoint(0.6, (x1, y1, tcp_set), RELEASE, "wait", yaw),
            Waypoint(1.0, (x1, y1, tcp_set + 0.035), RELEASE, "rise straight up", yaw),
            Waypoint(1.0, (x1 - 0.04, y1, tcp_high), RELEASE, "rise clear", yaw),
            Waypoint(1.5, (PARK[0], s * PARK[1], tcp_high), OPEN, "back away", yaw),
        ]
    return plan
