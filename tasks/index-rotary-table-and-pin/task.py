"""On the station table: a small rotary table on a base tilted 7 degrees down to the left. A square plate turns on a
central post, carrying a heavy off-centre fixture that swings it back to its home position unless it is held, and a peg to
turn it by. The base has two index holes, marked at 45 and at 90 degrees clockwise from home, and the plate has one hole.
A steel plunger pin stands in a socket on the left. The goal is to turn the table 90 degrees clockwise and lock it there
by dropping the pin through the plate's hole into the base's 90-degree hole.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py \\
           --instruction "Turn the rotary table 90 degrees clockwise by its peg and lock it there: drop the pin through the plate's hole into the base hole at the white mark." \\
           --scorer task_score ...
"""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from rigidparts import axis_world, quat_matrix, rigid_part  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

STEEL, DARK, BLUE, RED, WHITE, YELLOW, PIN_STEEL = ((0.45, 0.47, 0.50), (0.28, 0.30, 0.33), (0.20, 0.35, 0.65), (0.80, 0.15, 0.12),
                                                   (0.93, 0.93, 0.93), (0.92, 0.75, 0.15), (0.70, 0.72, 0.75))

TILT = math.radians(7)  # the base falls toward +y (left)
CX, CY, CZ = 0.22, 0.0, TOP + 0.045  # the base's top surface at the post
BASE_HALF, BASE_T = 0.125, 0.012
PLATE_HALF, PLATE_T = 0.10, 0.01
R = 0.08  # the index holes' radius from the post
PAD_R, PAD_UP = 0.006, 0.004  # the plate rests on three small balls, this far proud of the base
HOLE, PLATE_HOLE_W, PIVOT_HOLE, POST_R = 0.016, 0.02, 0.018, 0.008  # the plate's hole is wider, as a lead-in for the pin
PIN_R, PIN_H, HEAD_R, HEAD_H = 0.005, 0.06, 0.011, 0.006
PIN_START = (0.17, 0.24)
STATIONS = {90: (0.0, R), 45: (-R * math.sin(math.radians(45)), R * math.cos(math.radians(45)))}  # base holes, base frame
PLATE_HOLE = (-R, 0.0)  # plate frame: over the 90-degree hole when turned 90 degrees clockwise


def base_point(x, y, up=0.0):
    """A point of the base's frame in the world; ``up`` is from the base's top surface."""
    return (CX + x, CY + y * math.cos(TILT) + up * math.sin(TILT), CZ - y * math.sin(TILT) + up * math.cos(TILT))


def holed_slab(half, z, thick, holes, color):
    """A square slab of boxes, centred at height ``z``, leaving a square hole at each (x, y, width) in ``holes``."""
    xs = sorted({-half, half, *[h[0] + s * h[2] / 2 for h in holes for s in (-1, 1)]})
    ys = sorted({-half, half, *[h[1] + s * h[2] / 2 for h in holes for s in (-1, 1)]})
    out = []
    for x0, x1 in zip(xs, xs[1:]):
        for y0, y1 in zip(ys, ys[1:]):
            cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
            if not any(abs(cx - h[0]) < h[2] / 2 and abs(cy - h[1]) < h[2] / 2 for h in holes):
                out.append((((cx, cy, z)), (x1 - x0, y1 - y0, thick), color))
    return out


base_holes = [(*xy, HOLE) for xy in STATIONS.values()]
BASE = rigid_part(
    "base",
    pos=base_point(0.0, 0.0),
    boxes=holed_slab(BASE_HALF, -BASE_T / 2, BASE_T, base_holes, STEEL),
    cylinders=[((0.0, 0.0, 0.012), POST_R, 0.024, DARK)],
    spheres=[((0.05 * math.cos(a), 0.05 * math.sin(a), PAD_UP - PAD_R), PAD_R, DARK) for a in (math.radians(d) for d in (-20, 100, 220))],
    friction=0.01,
    slick=True,
    tilt=-TILT,
)
FIXTURE = ((0.0, 0.07, PLATE_T + 0.035), (0.10, 0.06, 0.07), RED)
PLATE = rigid_part(
    "plate",
    pos=base_point(0.0, 0.0, PAD_UP + 0.0005),
    boxes=[*holed_slab(PLATE_HALF, PLATE_T / 2, PLATE_T, [(0.0, 0.0, PIVOT_HOLE), (*PLATE_HOLE, PLATE_HOLE_W)], BLUE),
           FIXTURE,
           ((R, 0.0, PLATE_T + 0.02), (0.02, 0.02, 0.04), YELLOW)],
    mass=0.8,
    tilt=-TILT,
)
PIN = rigid_part(
    "pin",
    pos=(*PIN_START, TOP),
    cylinders=[((0.0, 0.0, PIN_R + (PIN_H - PIN_R) / 2), PIN_R, PIN_H - PIN_R, PIN_STEEL),
               ((0.0, 0.0, PIN_H + HEAD_H / 2), HEAD_R, HEAD_H, PIN_STEEL)],
    spheres=[((0.0, 0.0, PIN_R), PIN_R, PIN_STEEL)],
    mass=0.03,
)
SOCKET_HOLE, SOCKET_DEPTH, SOCKET_WALL = 0.011, 0.03, 0.01
_s, _l = SOCKET_HOLE / 2 + SOCKET_WALL / 2, SOCKET_HOLE + 2 * SOCKET_WALL
SOCKET = [Box(f"socket_{i}", pos=(PIN_START[0] + dx, PIN_START[1] + dy, TOP + SOCKET_DEPTH / 2), size=(sx, sy, SOCKET_DEPTH), color=STEEL)
          for i, (dx, dy, sx, sy) in enumerate([(_s, 0, SOCKET_WALL, _l), (-_s, 0, SOCKET_WALL, _l), (0, _s, SOCKET_HOLE, SOCKET_WALL), (0, -_s, SOCKET_HOLE, SOCKET_WALL)])]
_edge = base_point(0.0, BASE_HALF)
MARKS = [Box("mark_90", pos=(_edge[0], _edge[1] + 0.012, TOP + 0.0005), size=(0.012, 0.012, 0.001), color=WHITE),
         Box("mark_45", pos=(CX - 0.105, _edge[1] + 0.012, TOP + 0.0005), size=(0.012, 0.012, 0.001), color=YELLOW)]
LEGS = [Box(f"leg_{i}", pos=(CX + sx * 0.09, base_point(0, sy * 0.09)[1], (TOP + base_point(0, sy * 0.09)[2] - BASE_T - 0.004) / 2),
            size=(0.03, 0.03, base_point(0, sy * 0.09)[2] - BASE_T - 0.004 - TOP), color=DARK)
        for i, (sx, sy) in enumerate(((-1, -1), (1, -1), (-1, 1), (1, 1)))]
OBJECTS = [BASE, PLATE, PIN, *SOCKET, *MARKS, *LEGS]

MAX_ANGLE = 8.0  # degrees the plate may sit off its station: a pinned plate leans on the pin's play
PLAY = 0.005  # m from a base hole's centre: the hole's 3 mm of play, plus 2 mm for a pin leaning in it


def turned(plate) -> float:
    """How far the plate has turned clockwise from home about the post, in degrees, in the base's frame."""
    base = quat_matrix((-math.sin(TILT / 2), 0.0, 0.0, math.cos(TILT / 2)))
    x = base.T @ quat_matrix(plate.quat_xyzw)[:, 0]
    return float(-math.degrees(math.atan2(x[1], x[0])))


def pinned_at(objects, station) -> bool:
    """Locked at this station: the pin within 12 degrees of upright, its axis within 5 mm of the base hole's centre at the
    base's top, its foot below the base's top, and the plate within 8 degrees of the station."""
    pin = objects["pin"]
    axis = axis_world(pin)
    if axis[2] < math.cos(math.radians(12)):
        return False
    top = np.array(base_point(*STATIONS[station]))
    at = np.asarray(pin.pos) + axis * (top[2] - pin.pos[2]) / axis[2]
    return bool(np.hypot(*(at - top)[:2]) < PLAY and pin.pos[2] < top[2] and abs(turned(objects["plate"]) - station) < MAX_ANGLE)


def success(objects):
    """Locked at the 90-degree station, the plate still and the pin not moving (a pin spinning in its hole doesn't matter);
    must hold for 1 s."""
    return bool(pinned_at(objects, 90) and objects["plate"].resting() and np.linalg.norm(objects["pin"].lin_vel) < 0.01)


def score(objects):
    """Partial credit (success scores 1.0): 0.5 if the plate is within 8 degrees of 90 but not pinned; 0.25 if it is pinned
    at the 45-degree station."""
    if pinned_at(objects, 45):
        return 0.25
    return 0.5 if abs(turned(objects["plate"]) - 90) < MAX_ANGLE else 0.0


# Measured in traced runs, the same way every run with -E object_jitter=0.
PUSH_LAG = 6  # degrees the plate stops short of where the hand goes
PIN_GRIP_SHIFT = (-0.015, 0.0132)  # the pin's foot minus the hand, while held (x, y; m)


def demo():
    """Right arm: put its closed fingers behind the peg and push it round 90 degrees clockwise, then hold it there against
    the fixture's weight. Left arm: take the pin and drop it through the plate into the 90-degree hole. Then the right arm
    backs away."""
    hover = TOP + 0.16
    yaw_l, yaw_r = -np.pi / 2, np.pi / 2  # the hands turned this way reach further, and hold the pin upright
    finger_up = PAD_UP + PLATE_T + 0.03  # the TCP above the base: the fingertips clear the plate, level with the peg
    d = 0.024  # the hand's centre behind the peg's centre, along the direction it pushes

    def behind(phi):
        a = math.radians(phi)
        px, py = R * math.cos(a), -R * math.sin(a)  # the peg, turned phi clockwise
        tx, ty = -math.sin(a), -math.cos(a)  # the way it moves
        return base_point(px - d * tx, py - d * ty, finger_up)

    start = behind(-12)
    right = [
        Waypoint(1.5, (start[0], start[1], start[2] + 0.03), FINGER_CLOSED, "above, behind the peg", yaw_r),
        Waypoint(1.2, start, FINGER_CLOSED, "down behind it", yaw_r),
    ]
    right += [Waypoint(0.7, behind(p), FINGER_CLOSED, f"push it round to {p} degrees", yaw_r) for p in (0, 15, 30, 45, 60, 75, 90 + PUSH_LAG)]
    hold = behind(90 + PUSH_LAG)
    grasp = PIN_H - 0.012
    target = base_point(-PIN_GRIP_SHIFT[0], R - PIN_GRIP_SHIFT[1], PAD_UP + PLATE_T)
    above = target[2] + 0.008 + grasp
    left = [
        Waypoint(1.5, (*PIN_START, hover), FINGER_OPEN, "above the pin", yaw_l),
        Waypoint(1.0, (*PIN_START, TOP + grasp), FINGER_OPEN, "down to it", yaw_l),
        Waypoint(0.6, (*PIN_START, TOP + grasp), FINGER_CLOSED, "grip", yaw_l),
        Waypoint(1.0, (*PIN_START, above + 0.02), FINGER_CLOSED, "lift it out", yaw_l),
        # Wait to one side: the fixture swings through the space over the hole while the table turns.
        Waypoint(3.6, (*PIN_START, above + 0.02), FINGER_CLOSED, "wait for the table", yaw_l),
        Waypoint(1.5, (target[0], target[1], above + 0.02), FINGER_CLOSED, "over the 90-degree hole", yaw_l),
        Waypoint(0.8, (target[0], target[1], above + 0.02), FINGER_CLOSED, "let the pin settle", yaw_l),
        Waypoint(0.8, (target[0], target[1], above), FINGER_CLOSED, "down to just above it", yaw_l),
        Waypoint(0.5, (target[0], target[1], above), FINGER_OPEN, "drop it", yaw_l),
        Waypoint(0.8, (target[0], target[1], hover), FINGER_OPEN, "back up", yaw_l),
    ]
    settle = sum(w.duration for w in left) - sum(w.duration for w in right)
    right += [Waypoint(max(settle, 0.5), hold, FINGER_CLOSED, "hold it while the pin goes in", yaw_r),
              Waypoint(1.0, (hold[0], hold[1], hover), FINGER_CLOSED, "back away", yaw_r)]
    return {"left": left, "right": right}
