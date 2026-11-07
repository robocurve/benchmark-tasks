"""On the station table, at 1:10 scale: a trench (30 mm wide, dark, with orange edge lines) runs side to side across the
table 270 mm in front of the arms. A red fence runs side to side 160 mm beyond it, closing off the far side of the work
area. Two brown spoil piles (40 x 40 x 30 mm blocks) have been dumped on the near side, only 30 mm (0.3 m at scale)
from the trench edge, where workers need to get in and out.

Goal: both spoil piles on the far side of the trench, at least 60 mm (0.6 m, the OSHA minimum) from its edge and wholly
inside the fence. That leaves a 66 mm deep band for a 40 mm pile.

Run it:  inspect-robots run --embodiment yam_arena -E task=trench_spoil_setback.py -E object_jitter=0 \\
           --instruction "The spoil piles are too close to the trench. Move both of them to the far side of the trench, \\
           at least 0.6 m (6 cm in this 1:10 model) from its edge and wholly inside the red fence. Leave the near side \\
           clear for the workers." --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- trench and fence (fixed) ----------------------------------------------------------------------------------------
TRENCH_X0, TRENCH_W, TRENCH_LEN = 0.270, 0.030, 0.300  # near edge, width, length side to side
TRENCH_X1 = TRENCH_X0 + TRENCH_W  # far edge
LINE_W, PAINT_T = 0.003, 0.001
SETBACK = 0.060  # 0.6 m at 1:10
FENCE_X, FENCE_SIZE = 0.430, (0.008, 0.300, 0.025)
FENCE_FACE = FENCE_X - FENCE_SIZE[0] / 2
OBJECTS = [
    Box("trench", pos=(TRENCH_X0 + TRENCH_W / 2, 0.0, TOP + PAINT_T / 2), size=(TRENCH_W - 2 * LINE_W, TRENCH_LEN, PAINT_T),
        color=(0.12, 0.10, 0.09)),
    *[Box(f"edge_{s}", pos=(x, 0.0, TOP + PAINT_T / 2), size=(LINE_W, TRENCH_LEN, PAINT_T), color=(0.95, 0.45, 0.08))
      for s, x in (("near", TRENCH_X0 + LINE_W / 2), ("far", TRENCH_X1 - LINE_W / 2))],
    Box("fence", pos=(FENCE_X, 0.0, TOP + FENCE_SIZE[2] / 2), size=FENCE_SIZE, color=(0.80, 0.12, 0.10)),
]

# -- spoil piles -----------------------------------------------------------------------------------------------------
PILE = (0.040, 0.040, 0.030)
PILE_MASS = 0.06
PILES = {"spoil_left": (0.22, 0.10), "spoil_right": (0.22, -0.10)}
OBJECTS += [Box(n, pos=(x, y, TOP + PILE[2] / 2), size=PILE, mass=PILE_MASS, color=(0.45, 0.32, 0.20))
            for n, (x, y) in PILES.items()]

# -- scoring ---------------------------------------------------------------------------------------------------------
FLAT_DZ, FLAT_TILT_DEG = 0.004, 5.0


def _rot(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _footprint(st):
    c = np.array([[a, b, d] for a in (-1, 1) for b in (-1, 1) for d in (-1, 1)]) * (np.array(PILE) / 2) @ _rot(st.quat_xyzw).T
    xy = st.pos[:2] + c[:, :2]
    return xy[:, 0].min(), xy[:, 0].max(), xy[:, 1].min(), xy[:, 1].max()


def pile_state(st):
    """'legal': upright on the table on the far side, its nearest point >= 60 mm from the trench's far edge and its
    farthest point short of the fence, within the fence's length; 'too_close': upright on the far side but inside the
    60 mm setback; otherwise None."""
    R = _rot(st.quat_xyzw)
    tilt = math.degrees(math.acos(min(1.0, abs(R[2, 2]))))
    if abs(st.pos[2] - (TOP + PILE[2] / 2)) >= FLAT_DZ or tilt >= FLAT_TILT_DEG:
        return None
    x0, x1, y0, y1 = _footprint(st)
    if x0 < TRENCH_X1 or x1 > FENCE_FACE or y0 < -TRENCH_LEN / 2 or y1 > TRENCH_LEN / 2:
        return None
    return "legal" if x0 - TRENCH_X1 >= SETBACK else "too_close"


def score(objects):
    """Per pile: 0.5 if legal, 0.15 if on the far side but inside the setback, else 0."""
    pts = {"legal": 0.5, "too_close": 0.15, None: 0.0}
    return round(sum(pts[pile_state(objects[n])] for n in PILES), 3)


def success(objects):
    return score(objects) >= 0.999 and all(objects[n].resting() for n in PILES)


TARGET_X = TRENCH_X1 + SETBACK + PILE[0] / 2 + 0.013  # 13 mm beyond the setback, 13 mm short of the fence
TARGET = {"spoil_left": (TARGET_X, 0.07), "spoil_right": (TARGET_X, -0.07)}


def distance(objects):
    return float(sum(math.hypot(objects[n].pos[0] - x, objects[n].pos[1] - y) for n, (x, y) in TARGET.items()))


# -- scripted demo ---------------------------------------------------------------------------------------------------
TRAVEL_Z = TOP + 0.09
YAW = {"left": -math.pi / 2, "right": math.pi / 2}
GRIP_ABOVE_BOTTOM = PAD_TIP_PAST_TCP + 0.008
RELEASE_GAP = 0.003
SETTLE_S = 0.8
OPEN_W = -(PILE[0] / 2 + 0.012)


def _move(name, yaw):
    sx, sy = PILES[name]
    gx, gy = TARGET[name]
    pick = TOP + GRIP_ABOVE_BOTTOM
    drop = TOP + RELEASE_GAP + GRIP_ABOVE_BOTTOM
    return [
        Waypoint(1.5, (sx, sy, TRAVEL_Z), OPEN_W, f"to {name}", yaw),
        Waypoint(1.2, (sx, sy, pick), OPEN_W, "descend", yaw),
        Waypoint(0.8, (sx, sy, pick), 0.0, "grip", yaw),
        Waypoint(1.0, (sx, sy, TRAVEL_Z), 0.0, "lift", yaw),
        Waypoint(1.8, (gx, gy, TRAVEL_Z), 0.0, "over the trench", yaw),
        Waypoint(1.0, (gx, gy, drop + 0.012), 0.0, "lower", yaw),
        Waypoint(SETTLE_S, (gx, gy, drop + 0.012), 0.0, "settle", yaw),
        Waypoint(0.8, (gx, gy, drop), 0.0, "down", yaw),
        Waypoint(SETTLE_S, (gx, gy, drop), 0.0, "settle", yaw),
        Waypoint(0.6, (gx, gy, drop), OPEN_W, "let go", yaw),
        Waypoint(1.0, (gx, gy, TRAVEL_Z), OPEN_W, "up", yaw),
    ]


def demo():
    """Left arm carries the left pile over the trench into the band between the setback and the fence and moves clear;
    then the right arm does the same with the right pile."""
    left = _move("spoil_left", YAW["left"]) + [Waypoint(1.5, (0.20, 0.25, TRAVEL_Z), FINGER_OPEN, "clear", YAW["left"])]
    wait = sum(w.duration for w in left)
    right = [Waypoint(wait, (0.20, -0.25, TRAVEL_Z), FINGER_OPEN, "wait for the left arm")] + _move(
        "spoil_right", YAW["right"]) + [Waypoint(1.5, (0.20, -0.25, TRAVEL_Z), FINGER_OPEN, "clear", YAW["right"])]
    return {"left": left, "right": right}
