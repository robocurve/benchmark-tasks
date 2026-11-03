"""On the station table: the first course of a short brick wall is laid, two red bricks (80 x 40 x 25 mm) end to end
along a line pointing away from the arms, on the midline between them, with a 10 mm head joint between them. Three
loose pieces lie on the table: one full brick and two half bricks (35 mm long), the full brick and one half in front
of the left arm, the other half in front of the right arm. The next course goes on top in half running bond: the full
brick centred over the joint below, so half of it overlaps each brick under it, and a half brick at each end so the
wall's ends stay flush. Head joints are 10 mm, like the joint below.
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

BRICK = (0.080, 0.040, 0.025)  # length, width, height
JOINT = 0.010  # head joint, as below
HALF = ((BRICK[0] - JOINT) / 2, BRICK[1], BRICK[2])  # two halves and a joint make a brick
RED = (0.70, 0.22, 0.15)
DARK_RED = (0.52, 0.16, 0.12)  # WHY: the laid course a shade darker, so the new course stands out on the top camera

WALL_X = 0.25  # the head joint below, on the wall's line (y = 0)
COURSE_1 = [Box(f"laid_{i}", pos=(WALL_X + s * (JOINT + BRICK[0]) / 2, 0.0, TOP + BRICK[2] / 2), size=BRICK,
                color=DARK_RED) for i, s in ((1, -1), (2, 1))]
WALL_NEAR = WALL_X - JOINT / 2 - BRICK[0]  # the wall's end nearer the arms
WALL_FAR = WALL_X + JOINT / 2 + BRICK[0]
COURSE_1_TOP = TOP + BRICK[2]

STARTS = {  # piece name: its centre on the table (x, y), its length along x
    "brick": (0.20, 0.25),
    "half_1": (0.33, 0.25),
    "half_2": (0.30, -0.25),
}
PIECES = [Box(name, pos=(x, y, TOP + BRICK[2] / 2 + 0.001), size=BRICK if name == "brick" else HALF, mass=m, color=RED)
          for (name, (x, y)), m in zip(STARTS.items(), (0.150, 0.066, 0.066))]  # 1.9 g/cm3, as clay brick
OBJECTS = [*COURSE_1, *PIECES]

IN_LINE_MM = 3.0  # a piece's centre at most this far off the wall's line, sideways
SQUARE_DEG = 3.0  # its length within this of the wall's line
FLAT_DEG = 3.0
ON_COURSE_MM = 1.0  # its bottom within this of the laid course's top: on the wall, not on another piece or held above
FLUSH_MM = 3.0  # a half brick's outer end at most this far from the wall's end, either way
OVER_JOINT_MM = 5.0  # the full brick's centre at most this far from the joint below
JOINT_MM = (7.0, 13.0)  # each head joint of the new course


def _rotation(quat_xyzw) -> np.ndarray:
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def _tilt_deg(piece) -> float:
    """How far the piece's 25 mm axis is from vertical (deg), either way up."""
    return math.degrees(math.acos(min(1.0, abs(_rotation(piece.quat_xyzw)[2, 2]))))


def _off_line_deg(piece) -> float:
    """How far the piece's length is from the wall's line, in the table plane (deg), either way round."""
    r = _rotation(piece.quat_xyzw)
    d = math.degrees(math.atan2(r[1, 0], r[0, 0])) % 180.0
    return min(d, 180.0 - d)


def _laid(piece) -> bool:
    """Flat on the laid course, along and on the wall's line."""
    return (
        abs(float(piece.low[2]) - COURSE_1_TOP) <= ON_COURSE_MM / 1000  # low is exact for a box
        and _tilt_deg(piece) <= FLAT_DEG
        and _off_line_deg(piece) <= SQUARE_DEG
        and abs(float(piece.pos[1])) <= IN_LINE_MM / 1000
    )


def _ends(piece, length) -> tuple[float, float]:
    """The piece's two ends along the wall (x, m), from its centre and ``length``."""
    # WHY: not low/high: a piece turned the allowed 3 degrees widens its bounding box about 1 mm at each end, which
    # would read each joint about 2 mm narrower than a ruler along the wall does.
    return float(piece.pos[0]) - length / 2, float(piece.pos[0]) + length / 2


def _in_place(objects) -> dict[str, bool]:
    """Which places of the new course hold a laid piece of the right kind: "near" and "far" a half brick flush with
    that end of the wall, "middle" the full brick over the joint below."""
    halves = [objects[n] for n in ("half_1", "half_2") if _laid(objects[n])]
    brick = objects["brick"]
    return {
        "near": any(abs(_ends(h, HALF[0])[0] - WALL_NEAR) <= FLUSH_MM / 1000 for h in halves),
        "far": any(abs(_ends(h, HALF[0])[1] - WALL_FAR) <= FLUSH_MM / 1000 for h in halves),
        "middle": _laid(brick) and abs(float(brick.pos[0]) - WALL_X) <= OVER_JOINT_MM / 1000,
    }


def _joints_mm(objects) -> tuple[float, float]:
    """The new course's two head joints, near then far (mm), along the wall."""
    near, far = sorted((objects[n] for n in ("half_1", "half_2")), key=lambda h: float(h.pos[0]))
    lo, hi = _ends(objects["brick"], BRICK[0])
    return 1000.0 * (lo - _ends(near, HALF[0])[1]), 1000.0 * (_ends(far, HALF[0])[0] - hi)


def success(objects):
    """All three pieces in place, both head joints 7 to 13 mm, every piece still; holds 1 s."""
    return (
        all(_in_place(objects).values())
        and all(JOINT_MM[0] <= j <= JOINT_MM[1] for j in _joints_mm(objects))
        and all(objects[p.name].resting() for p in PIECES)
    )


def distance(objects):
    """Mean over the three places of how far the nearest right piece's centre is from it (m)."""
    places = {"near": WALL_NEAR + HALF[0] / 2, "middle": WALL_X, "far": WALL_FAR - HALF[0] / 2}
    target_z = COURSE_1_TOP + BRICK[2] / 2

    def gap(piece, x):
        return float(np.linalg.norm(np.asarray(piece.pos, dtype=float) - (x, 0.0, target_z)))

    halves = [objects["half_1"], objects["half_2"]]
    return float(np.mean([
        min(gap(h, places["near"]) for h in halves),
        gap(objects["brick"], places["middle"]),
        min(gap(h, places["far"]) for h in halves),
    ]))


def score(objects):
    """Partial credit (success scores 1.0): 0.25 for each place of the new course holding the right piece."""
    return 0.25 * sum(_in_place(objects).values())


TRAVEL_Z = TOP + 0.12
GRIP_ABOVE = 0.005  # grip 5 mm above a piece's middle, so the pad tips stay above the course below
PLACE_CLEAR = 0.002  # let go with the piece this far above the laid course
# WHY: the first traced demo (2026-11-03) laid the full brick 2.0 mm toward the arms and 1.9 mm to the right of its
# aim, leaving the far joint at 12.2 mm of a 13 mm limit; the halves landed within 1.4 mm. This offset only makes the
# scripted demo land in the sim; the scorer doesn't use it.
AIM = {"half_2": (0.0, 0.0), "half_1": (0.0, 0.0), "brick": (0.002, 0.002)}
HOMES = {"half_1": WALL_NEAR + HALF[0] / 2, "brick": WALL_X, "half_2": WALL_FAR - HALF[0] / 2}


def _move(piece):
    x, y = STARTS[piece]
    ax, ay = AIM[piece]
    pick = (x, y, TOP + 0.001 + BRICK[2] / 2 + GRIP_ABOVE)
    place = (HOMES[piece] + ax, ay, COURSE_1_TOP + PLACE_CLEAR + BRICK[2] / 2 + GRIP_ABOVE)
    return pick_place_waypoints(pick, place, travel_z=TRAVEL_Z)


def demo():
    """The right arm lays a half brick at the wall's far end and moves back over its side. The left arm then lays the
    other half at the near end and the full brick over the joint. The fingers close across each piece's 40 mm width;
    no piece is turned."""
    right = _move("half_2") + [Waypoint(1.5, (0.25, -0.25, TRAVEL_Z), FINGER_OPEN, "back over its side")]
    wait_s = sum(w.duration for w in right)
    left = [Waypoint(wait_s, (*STARTS["half_1"], TRAVEL_Z), FINGER_OPEN, "wait for the right arm")]
    left += _move("half_1") + _move("brick")
    return {"left": left, "right": right}
