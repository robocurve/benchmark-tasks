"""On the station table: a pump on a blue baseplate stands on four ball feet, its feet on the thin ends of two opposing yellow
wedges that lie tips inward on a low-friction slide plate. The pump's outlet flange sits 16 mm below a fixed pipe's flange.
Each wedge has a red peg at its thick end and a flat seat behind its slope. The slope is steeper than friction can hold, so
the baseplate's weight squeezes a wedge back out unless a hand holds it, until the feet reach the seats. The goal is to push both
wedges in together and raise the baseplate until the flanges line up, without tilting it. Two guide posts in notches, standing
in for anchor studs, keep the baseplate from sliding sideways.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py \\
           --instruction "Line the pump's outlet flange up with the fixed pipe flange: push the two yellow wedges in together by their red pegs until the blue baseplate's feet sit on the wedges' flat seats, 15 mm higher, never letting the baseplate tilt more than 2 degrees." \\
           --scorer task_score ...
"""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from rigidparts import axis_world, rigid_part  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

STEEL, DARK, BLUE, RED, YELLOW = ((0.45, 0.47, 0.50), (0.28, 0.30, 0.33), (0.20, 0.35, 0.65), (0.80, 0.15, 0.12), (0.92, 0.75, 0.15))

CX = 0.20  # everything is centred this far forward, on the midline
PLATE_T = 0.004  # the slide plate
LANE_TOP = TOP + PLATE_T
LANE_HALF_Y, LANE_HALF_X = 0.265, 0.09
SLOPE = 0.2  # a wedge rises 1 in 5
TIP, SLOPE_LEN, WEDGE_LEN, WEDGE_HALF_X = 0.006, 0.09, 0.15, 0.08
SEAT = TIP + SLOPE * SLOPE_LEN  # the seat's height above the slide plate
PEG_AT, PEG_W, PEG_H = 0.135, 0.02, 0.05  # the peg's centre from the tip
FOOT_Y, FOOT_X, FOOT_R = 0.105, 0.05, 0.006  # the feet's centres from the midlines
FOOT_START = 0.01  # how far back from the tip the feet start
TIP_START = FOOT_Y - FOOT_START  # a wedge's tip from the midline, at the start
FOOT_DROP = 0.012  # the feet's lowest point below the baseplate's underside
BASE_HALF_X, BASE_HALF_Y, BASE_T = 0.10, 0.12, 0.015
PUMP_R, PUMP_LEN = 0.04, 0.12  # the pump's body, a cylinder lying left to right on the baseplate
OUTLET_Z, OUTLET_R, FLANGE_R, FLANGE_T = BASE_T + PUMP_R, 0.012, 0.018, 0.008
FLANGE_X = 0.112  # the pump's flange from the baseplate's centre, forward
FLANGE_GAP, ALIGN_RISE = 0.006, 0.016  # the gap between the flanges; the pipe's centre above the pump's at the start
NOTCH_W, NOTCH_DEEP = 0.014, 0.012  # a notch in the middle of each of the baseplate's front and back edges
POST_Y, POST_X, POST_H = 0.01, 0.008, 0.057  # a guide post in each notch, 2 mm clear on each side
BASE_MASS, WEDGE_MASS = 1.5, 0.35
LOW_FRICTION, FOOT_FRICTION = 0.02, 0.05


def wedge(name, side):
    """A wedge with its tip inward; ``side`` is +1 on the left, -1 on the right. Its frame is at its tip, on the plate."""
    profile = [(0.0, 0.0), (0.0, TIP), (SLOPE_LEN, SEAT), (WEDGE_LEN, SEAT), (WEDGE_LEN, 0.0)]
    return rigid_part(
        name,
        pos=(CX, side * TIP_START, LANE_TOP + 0.0003),
        prisms=[([(side * y, z) for y, z in profile], WEDGE_HALF_X, YELLOW)],
        boxes=[((0.0, side * PEG_AT, SEAT + PEG_H / 2), (PEG_W, PEG_W, PEG_H), RED)],
        mass=WEDGE_MASS,
    )


WEDGES = {"wedge_left": 1, "wedge_right": -1}
BASE_START_Z = LANE_TOP + TIP + SLOPE * FOOT_START + FOOT_R * math.hypot(1, SLOPE) - FOOT_R + FOOT_DROP
BASE_START = (CX, 0.0, BASE_START_Z)
BASE = rigid_part(
    "baseplate",
    pos=(CX, 0.0, BASE_START_Z + 0.0005),
    boxes=[((0.0, 0.0, BASE_T / 2), (2 * (BASE_HALF_X - NOTCH_DEEP), 2 * BASE_HALF_Y, BASE_T), BLUE),
           *[((sx * (BASE_HALF_X - NOTCH_DEEP / 2), sy * (BASE_HALF_Y + NOTCH_W / 2) / 2, BASE_T / 2),
              (NOTCH_DEEP, BASE_HALF_Y - NOTCH_W / 2, BASE_T), BLUE) for sx in (-1, 1) for sy in (-1, 1)],
           ],
    cylinders=[((0.0, 0.0, OUTLET_Z), PUMP_R, PUMP_LEN, STEEL, "Y"),
               (((PUMP_R + FLANGE_X) / 2 - 0.005, 0.0, OUTLET_Z), OUTLET_R, FLANGE_X - PUMP_R + 0.01, STEEL, "X"),
               ((FLANGE_X, 0.0, OUTLET_Z), FLANGE_R, FLANGE_T, DARK, "X")],
    spheres=[((sx * FOOT_X, sy * FOOT_Y, FOOT_R - FOOT_DROP), FOOT_R, DARK) for sx in (-1, 1) for sy in (-1, 1)],
    mass=BASE_MASS,
    friction=FOOT_FRICTION,
    slick=True,
)
LANE = rigid_part("slide_plate", pos=(CX, 0.0, TOP + PLATE_T / 2),
                  boxes=[((0.0, 0.0, 0.0), (2 * LANE_HALF_X, 2 * LANE_HALF_Y, PLATE_T), DARK)], friction=LOW_FRICTION, slick=True)
RAIL_H = 0.01
RAILS = [Box(f"rail_{i}", pos=(CX + s * (WEDGE_HALF_X + 0.006), 0.0, LANE_TOP + RAIL_H / 2), size=(0.01, 2 * LANE_HALF_Y, RAIL_H), color=STEEL)
         for i, s in enumerate((-1, 1))]
STOP_AT = TIP_START + WEDGE_LEN + 0.001  # the outer stops' inner faces, from the midline
STOPS = [Box(f"stop_{i}", pos=(CX, s * (STOP_AT + 0.005), LANE_TOP + 0.01), size=(2 * WEDGE_HALF_X, 0.01, 0.02), color=STEEL)
         for i, s in enumerate((-1, 1))]
POSTS = rigid_part("guide_posts", pos=(CX, 0.0, TOP),
                   boxes=[((s * (BASE_HALF_X - NOTCH_DEEP + POST_X / 2 + 0.002), 0.0, POST_H / 2), (POST_X, POST_Y, POST_H), STEEL) for s in (-1, 1)],
                   friction=LOW_FRICTION, slick=True)
PIPE_X = CX + FLANGE_X + FLANGE_T + FLANGE_GAP
PIPE_Z = BASE_START_Z + OUTLET_Z + ALIGN_RISE
PIPE = rigid_part("pipe", pos=(PIPE_X, 0.0, PIPE_Z),
                  cylinders=[((0.0, 0.0, 0.0), FLANGE_R, FLANGE_T, DARK, "X"), ((0.05, 0.0, 0.0), OUTLET_R, 0.10, STEEL, "X")],
                  boxes=[((0.06, 0.0, (TOP - PIPE_Z) / 2 - OUTLET_R / 2), (0.03, 0.03, PIPE_Z - TOP - OUTLET_R), STEEL)])
OBJECTS = [LANE, BASE, POSTS, PIPE, *[wedge(n, s) for n, s in WEDGES.items()], *RAILS, *STOPS]

MAX_TILT = 2.0  # degrees the baseplate may tilt from level
MAX_RISE = 0.03  # m: past this the baseplate has been lifted off its wedges
FULL_RISE, ON_SEAT = 0.015, 0.08  # m: the baseplate's rise for full score, and how far in a wedge has its seat under the feet
PARTIAL = ((0.010, 0.04, 0.5), (0.005, 0.02, 0.25))  # (rise, how far in both wedges must be, score)


def rise(objects) -> float:
    return float(objects["baseplate"].pos[2]) - BASE_START_Z


def shift(objects) -> float:
    return float(np.hypot(*(np.asarray(objects["baseplate"].pos)[:2] - np.array(BASE_START[:2]))))


def tilt(objects) -> float:
    return float(math.degrees(math.acos(min(1.0, abs(float(axis_world(objects["baseplate"])[2]))))))


def pushed_in(objects, name) -> float:
    """How far a wedge has moved inward from its start."""
    return TIP_START - WEDGES[name] * float(objects[name].pos[1])


def wedges_flat(objects) -> bool:
    """Both wedges lying on the slide plate: their frames within 4 mm of its surface."""
    return all(abs(float(objects[n].pos[2]) - LANE_TOP) < 0.004 for n in WEDGES)


class Watch:
    """Whether the baseplate has ever tilted too far or been lifted off. A new trial restores everything in one control step, far faster
    than anything moves on its own, and only that clears the record."""

    def __init__(self):
        self.last, self.broken = None, False

    def update(self, objects):
        now = (*[pushed_in(objects, n) for n in WEDGES], shift(objects), rise(objects))
        home = all(abs(v) < 0.002 for v in now)
        if self.last is not None and home and max(abs(a - b) for a, b in zip(now, self.last)) > 0.005:
            self.broken = False
        self.last = now
        self.broken = self.broken or tilt(objects) > MAX_TILT or rise(objects) > MAX_RISE


WATCH = Watch()


def success(objects):
    """The baseplate 15 mm up, both wedges in far enough for the feet to be on the seats, neither limit ever passed, all still;
    must hold for 1 s. Runs first in every control step, so it also keeps the record."""
    WATCH.update(objects)
    return bool(rise(objects) >= FULL_RISE and all(pushed_in(objects, n) >= ON_SEAT for n in WEDGES) and wedges_flat(objects) and not WATCH.broken
                and all(objects[n].resting() for n in ("baseplate", *WEDGES)))


def score(objects):
    """Partial credit (success scores 1.0), only if neither limit was ever passed: 0.5 with the baseplate up at least 10 mm and
    both wedges in at least 4 cm; 0.25 with it up at least 5 mm and both wedges in at least 2 cm; both wedges lying flat."""
    if WATCH.broken or not wedges_flat(objects):
        return 0.0
    least = min(pushed_in(objects, n) for n in WEDGES)
    return next((s for up, inward, s in PARTIAL if rise(objects) >= up and least >= inward), 0.0)


PUSH = 0.104  # how far each hand moves: the arms give about 2 cm under load, and the tips meet at 9.5 cm


def demo():
    """Both arms grip their wedge's peg, push the two wedges in together at the same pace until the feet are on the seats,
    then let go and back up."""
    grip_z = LANE_TOP + SEAT + PEG_H
    hover = grip_z + 0.08
    y0 = TIP_START + PEG_AT

    def arm(side, yaw):
        y, y1 = side * y0, side * (y0 - PUSH)
        return [
            Waypoint(2.0, (CX, y, hover), FINGER_OPEN, "above the peg", yaw),
            Waypoint(1.2, (CX, y, grip_z), FINGER_OPEN, "down to it", yaw),
            Waypoint(0.7, (CX, y, grip_z), FINGER_CLOSED, "grip", yaw),
            Waypoint(8.0, (CX, y1, grip_z), FINGER_CLOSED, "push the wedge in", yaw),
            Waypoint(1.5, (CX, y1, grip_z), FINGER_CLOSED, "settle", yaw),
            Waypoint(0.6, (CX, y1, grip_z), FINGER_OPEN, "let go", yaw),
            Waypoint(1.0, (CX, y1, hover), FINGER_OPEN, "back up", yaw),
        ]

    return {"left": arm(1, 0.0), "right": arm(-1, 0.0)}
