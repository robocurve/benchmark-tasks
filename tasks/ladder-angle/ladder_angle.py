"""On the station table, in front of the right arm: a grey wall 12 cm tall, fixed, its face toward the arms. A yellow
ladder (a 160 x 40 x 15 mm bar) stands straight up on the table against the wall's face, its 40 mm side along the
wall, its top about 4 cm above the wall.
"""

import math

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

LENGTH, WIDTH, THICK = 0.160, 0.040, 0.015
WALL_X, WALL_Y = 0.30, -0.20  # the wall's face (x) and the ladder's line (y)
# WHY: lower than the ladder, so the gripper can reach the top end of the ladder standing against it (a taller wall
# stops the gripper on its top), and the leaning ladder rests on the wall's top edge, as on a roof edge.
WALL_SIZE = (0.03, 0.16, 0.12)  # deep, wide, tall
WALL_TOP = TOP + WALL_SIZE[2]
WALL = Box("wall", pos=(WALL_X + WALL_SIZE[0] / 2, WALL_Y, TOP + WALL_SIZE[2] / 2), size=WALL_SIZE,
           color=(0.55, 0.55, 0.55))
# The ladder's 160 mm along z, its 40 mm across y, 15 mm deep, 1 mm off the wall's face.
LADDER = Box("ladder", pos=(WALL_X - THICK / 2 - 0.001, WALL_Y, TOP + LENGTH / 2 + 0.001),
             size=(THICK, WIDTH, LENGTH), mass=0.06, color=(0.95, 0.75, 0.10))
OBJECTS = [WALL, LADDER]

# OSHA 29 CFR 1926.1053(b)(5)(i): the foot about a quarter of the working length (foot to top support, along the
# ladder) out from the top support. That is cos(angle) = 1/4, 75.5 degrees from the table, whatever the wall's height.
# WHY: the rule says "approximately"; the bands test knowing the angle, not placing the foot to the millimetre.
SAFE = (0.20, 0.30)  # cos of the ladder's angle: 72.5 to 78.5 degrees
NEAR = (0.15, 0.35)  # half credit: 69.5 to 81.4 degrees
ON_TABLE_MM = 2.0  # the foot's lowest corner within this of the table
ON_EDGE_MM = 3.0  # the wall's top edge within this of the ladder's wall-side face
SQUARE_DEG = 10.0  # the ladder faces the wall: its 40 mm side level and along the wall, each within this


def _rotation(quat_xyzw) -> np.ndarray:
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def foot_share(ladder) -> float:
    """cos of the ladder's angle from the table: how far the foot stands out from the top support, per unit of length
    between them."""
    along = _rotation(ladder.quat_xyzw)[:, 2]  # the ladder's 160 mm axis
    return float(math.hypot(along[0], along[1]) / np.linalg.norm(along))


def _leaning(ladder) -> bool:
    """Foot on the table, its face toward the wall resting on the wall's top edge, facing the wall, over the wall."""
    r = _rotation(ladder.quat_xyzw)
    across = r[:, 1]
    toward = r[:, 0] * math.copysign(1.0, r[0, 0])  # the normal of the ladder's face toward the wall
    centre = np.asarray(ladder.pos, dtype=float)
    edge = np.array([WALL_X, float(centre[1]), WALL_TOP]) - centre  # the wall's top edge, beside the ladder's centre
    # WHY: a ladder turned about the vertical touches the edge at one corner of its face, not at its centre.
    gap = min(float(edge @ toward) + s * WIDTH / 2 * float(toward[1]) for s in (-1, 1)) - THICK / 2
    sin_sq = math.sin(math.radians(SQUARE_DEG))
    return (
        abs(float(ladder.low[2]) - TOP) <= ON_TABLE_MM / 1000
        and abs(gap) <= ON_EDGE_MM / 1000
        and abs(float(across[2])) <= sin_sq
        and abs(float(across[0])) <= sin_sq
        and abs(float(centre[1]) - WALL_Y) <= WALL_SIZE[1] / 2
    )


def success(objects):
    """Leaning on the wall at a safe angle and still; holds 1 s."""
    ladder = objects["ladder"]
    return _leaning(ladder) and SAFE[0] <= foot_share(ladder) <= SAFE[1] and ladder.resting()


def distance(objects):
    """How far cos of the ladder's angle is from a quarter."""
    return abs(foot_share(objects["ladder"]) - 0.25)


def score(objects):
    """Partial credit (success scores 1.0): 1.0 leaning at a safe angle, 0.5 leaning a little outside it, else 0."""
    ladder = objects["ladder"]
    if not _leaning(ladder):
        return 0.0
    share = foot_share(ladder)
    return 1.0 if SAFE[0] <= share <= SAFE[1] else 0.5 if NEAR[0] <= share <= NEAR[1] else 0.0


GRIP_BELOW_TOP = 0.012  # the pads close this far below the ladder's top end
LIFT = 0.004  # carry the ladder with its foot this far above the table
# WHY: tipped over its foot's wall-side edge, a standing ladder's wall-side face meets the wall's top edge once
# tan(tilt from vertical) = gap at the foot / wall height. A quarter cosine is a tilt of 14.5 degrees, so the gap is
# 0.258 x the wall's height, 31 mm. GAP_AIM is what the demo needs in the sim to land there.
GAP_AIM = WALL_SIZE[2] * 0.25 / math.sqrt(1 - 0.25**2)
PUSH_BELOW_TOP = 0.015  # push the standing ladder this far below its top


def demo():
    """The right arm grips the ladder's top end, lifts it, carries it out from the wall until the gap at its foot is
    GAP_AIM, stands it there and lets go, then pushes its top toward the wall until it tips over and leans on it."""
    x0 = WALL_X - THICK / 2 - 0.001
    x1 = WALL_X - GAP_AIM - THICK / 2  # the ladder's centre once its wall-side face stands GAP_AIM out
    top = TOP + 0.001 + LENGTH
    grip = top - GRIP_BELOW_TOP
    above = top + 0.03  # WHY: the arm reaches about 0.20 m above the table with a straight-down grip
    y = WALL_Y
    push_z = top - PUSH_BELOW_TOP
    behind = x1 - THICK / 2 - 0.012  # the closed fingers' wall-side face just short of the ladder's front face
    right = [
        Waypoint(2.0, (x0, y, above), FINGER_OPEN, "above the top end"),
        Waypoint(1.5, (x0, y, grip), FINGER_OPEN, "down around it"),
        Waypoint(0.8, (x0, y, grip), FINGER_CLOSED, "grip"),
        Waypoint(1.0, (x0, y, grip + LIFT), FINGER_CLOSED, "lift its foot off the table"),
        Waypoint(2.0, (x1, y, grip + LIFT), FINGER_CLOSED, "carry it out from the wall"),
        Waypoint(1.0, (x1, y, grip), FINGER_CLOSED, "stand it down"),
        Waypoint(1.0, (x1, y, grip), FINGER_CLOSED, "settle"),
        Waypoint(0.6, (x1, y, grip), FINGER_OPEN, "let go"),
        Waypoint(1.0, (x1, y, above), FINGER_OPEN, "up"),
        Waypoint(1.5, (behind - 0.02, y, above), FINGER_CLOSED, "over to its front"),
        Waypoint(1.5, (behind - 0.02, y, push_z), FINGER_CLOSED, "down beside the top"),
        Waypoint(2.5, (behind + 0.03, y, push_z), FINGER_CLOSED, "push the top toward the wall"),
        Waypoint(1.0, (behind - 0.03, y, push_z), FINGER_CLOSED, "back off"),
        Waypoint(1.0, (behind - 0.03, y, above), FINGER_CLOSED, "up"),
    ]
    return {"right": right}
