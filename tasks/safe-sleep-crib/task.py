"""A crib in front of the arms: four fixed white walls 8 cm tall around a 30 x 26 cm inside, with a fixed
mint-green mattress (1 cm thick) as its floor. A baby doll (a skin-coloured block, 12 x 6 x 5 cm, 400 g) lies on
its back in the middle along the crib's length. Around it on the mattress: a white pillow (5 x 7 x 3 cm) and a
brown stuffed bear (5 x 5 x 5 cm) on the robot's left of the baby, and a folded pink blanket (6 x 7 x 2 cm) on
the robot's right. The table around the crib is clear.
"""

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Get the crib ready, the baby is going down for a nap."

T = 0.01
X0, X1, HY = 0.16, 0.46, 0.13  # crib inside: x from X0 to X1, y within +-HY
WALL_H = 0.08
MAT = TOP + 0.01  # mattress top
WHITE = (0.95, 0.95, 0.95)
CX = (X0 + X1) / 2

CRIB = [
    Box("crib_front", pos=(X0 - T / 2, 0.0, TOP + WALL_H / 2), size=(T, 2 * HY + 2 * T, WALL_H), color=WHITE),
    Box("crib_back", pos=(X1 + T / 2, 0.0, TOP + WALL_H / 2), size=(T, 2 * HY + 2 * T, WALL_H), color=WHITE),
    Box("crib_left", pos=(CX, HY + T / 2, TOP + WALL_H / 2), size=(X1 - X0, T, WALL_H), color=WHITE),
    Box("crib_right", pos=(CX, -HY - T / 2, TOP + WALL_H / 2), size=(X1 - X0, T, WALL_H), color=WHITE),
    Box("mattress", pos=(CX, 0.0, TOP + 0.005), size=(X1 - X0, 2 * HY, 0.01), color=(0.62, 0.80, 0.68)),
]
BABY = Box("baby", pos=(0.31, 0.0, MAT + 0.025), size=(0.12, 0.06, 0.05), mass=0.4, color=(0.93, 0.76, 0.62))
PILLOW = Box("pillow", pos=(0.23, 0.075, MAT + 0.015), size=(0.05, 0.07, 0.03), mass=0.03, color=(1.0, 1.0, 1.0))
BEAR = Box("bear", pos=(0.39, 0.075, MAT + 0.025), size=(0.05, 0.05, 0.05), mass=0.05, color=(0.50, 0.33, 0.18))
BLANKET = Box("blanket", pos=(0.31, -0.075, MAT + 0.01), size=(0.06, 0.07, 0.02), mass=0.05,
              color=(0.97, 0.70, 0.80))
OBJECTS = [*CRIB, BABY, PILLOW, BEAR, BLANKET]
SOFT = ("pillow", "bear", "blanket")


def _flat(s, max_tilt_deg=20.0):
    qx, qy = s.quat_xyzw[0], s.quat_xyzw[1]
    return 1 - 2 * (qx * qx + qy * qy) > np.cos(np.radians(max_tilt_deg))


def _out_of_crib(s):
    """Still, its centre outside the crib's outer walls, resting on the table."""
    inside = X0 - T < s.pos[0] < X1 + T and abs(s.pos[1]) < HY + T
    return bool(s.resting() and not inside and s.low[2] < TOP + 0.02)


def _baby_ok(objects):
    """The baby still lies flat on the mattress inside the crib, with nothing resting on it."""
    b = objects["baby"]
    if not (X0 < b.pos[0] < X1 and abs(b.pos[1]) < HY and b.low[2] < MAT + 0.01 and _flat(b)):
        return False
    for n in SOFT:
        s = objects[n]
        over = s.low[0] < b.high[0] and s.high[0] > b.low[0] and s.low[1] < b.high[1] and s.high[1] > b.low[1]
        if over and s.low[2] > b.high[2] - 0.015:
            return False
    return True


def success(objects):
    """1.0: the pillow, the bear and the blanket all out of the crib, and the baby still lying on the
    mattress."""
    return _baby_ok(objects) and all(_out_of_crib(objects[n]) for n in SOFT)


def score(objects):
    """0 if the baby is no longer lying flat on the mattress inside the crib, or anything rests on it.
    Otherwise one third per soft item (pillow, bear, blanket) resting on the table outside the crib; an item
    still in the crib, on a wall, or held earns nothing."""
    if not _baby_ok(objects):
        return 0.0
    return round(sum(_out_of_crib(objects[n]) for n in SOFT) / 3, 2)


def demo():
    """Left arm: the pillow, then the bear, out over the crib's left wall onto the table. Right arm, after it:
    the blanket out over the right wall. Each is gripped across the crib's length (fingers closing along x),
    clear of the baby."""
    travel = TOP + 0.17
    yaw = np.pi / 2
    def grip(o, below=0.015):
        return (o.pos[0], o.pos[1], o.pos[2] + o.size[2] / 2 - below + 0.013)
    def drop(o, x, y, below=0.015):
        return (x, y, TOP + 0.01 + o.size[2] - below + 0.013)
    left = pick_place_waypoints(grip(PILLOW), drop(PILLOW, 0.22, 0.22), travel_z=travel, yaw=yaw)
    left += pick_place_waypoints(grip(BEAR, 0.02), drop(BEAR, 0.34, 0.22, 0.02), travel_z=travel, yaw=yaw)
    left.append(Waypoint(1.5, (0.15, 0.25, TOP + 0.25), FINGER_OPEN, "park"))
    wait = sum(w.duration for w in left)
    right = [Waypoint(wait, (0.15, -0.25, TOP + 0.25), FINGER_OPEN, "wait")]
    right += pick_place_waypoints(grip(BLANKET, 0.01), drop(BLANKET, 0.30, -0.22, 0.01), travel_z=travel, yaw=yaw)
    right.append(Waypoint(1.5, (0.15, -0.25, TOP + 0.25), FINGER_OPEN, "park"))
    return {"left": left, "right": right}
