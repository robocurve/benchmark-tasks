"""On the table: a rectangle outlined with green tape (15 x 22 cm inside, 2.5 cm wide tape), four fixed grey posts
(3 cm across, 8 cm tall), one inside the rectangle and three outside it, and a yogurt cup (6.8 cm across, 5.4 cm tall)
standing upright outside the rectangle. The cup goes inside the rectangle, at the spot whose distance to the nearest
post is as large as possible.
"""

import math

from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Asset, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
GREEN = (0.15, 0.62, 0.25)
GREY = (0.55, 0.55, 0.57)

# Tape rectangle: inner edges (m). Strips 2.5 cm wide, 2 mm thick, laid just outside the inner edges.
X0, X1, Y0, Y1 = 0.275, 0.425, -0.24, -0.02
TAPE_W, TAPE_T = 0.025, 0.002
MARGIN = 0.035  # the cup's centre must be this far inside the inner edges (the whole cup inside the rectangle)

POST_R, POST_H = 0.015, 0.08
POSTS = {
    "post_a": (0.285, 0.025),
    "post_b": (0.415, 0.025),
    "post_c": (0.35, -0.195),
    "post_d": (0.36, 0.20),
}

CUP_XY = (0.30, -0.32)
D_FULL, D_HALF = 0.10, 0.07  # nearest-post distance for 1.0 and for 0.5
CUP_EXTENT = 0.06796  # the library model's own width: default size, scale 1


def _nearest_post(x, y):
    return min(math.hypot(x - px, y - py) for px, py in POSTS.values())


def _grid():
    nx = int(round((X1 - X0 - 2 * MARGIN) / 0.01))
    ny = int(round((Y1 - Y0 - 2 * MARGIN) / 0.01))
    return [(round(X0 + MARGIN + 0.01 * i, 3), round(Y0 + MARGIN + 0.01 * j, 3))
            for i in range(nx + 1) for j in range(ny + 1)]


def _best_spot():
    ranked = sorted(((_nearest_post(x, y), (x, y)) for x, y in _grid()), reverse=True)
    (d1, p1), (d2, _) = ranked[0], ranked[1]
    assert d1 - d2 >= 0.005, "best spot is not unique"
    return p1, d1, d2


BEST_XY, BEST_D, NEXT_D = _best_spot()
CORNERS = [(x, y) for x in (X0 + MARGIN, X1 - MARGIN) for y in (Y0 + MARGIN, Y1 - MARGIN)]
assert BEST_D >= D_FULL
assert _nearest_post((X0 + X1) / 2, (Y0 + Y1) / 2) < D_HALF  # the rectangle's centre scores 0
assert all(_nearest_post(x, y) < D_FULL for x, y in CORNERS)  # no corner of the allowed area scores 1.0


def _continuous_optimum():
    """The best spot on a 1 mm grid: the point equally far from posts a, b and c."""
    ax0, ay0 = X0 + MARGIN, Y0 + MARGIN
    nx, ny = int(round((X1 - X0 - 2 * MARGIN) / 0.001)), int(round((Y1 - Y0 - 2 * MARGIN) / 0.001))
    return max((_nearest_post(ax0 + 0.001 * i, ay0 + 0.001 * j), (round(ax0 + 0.001 * i, 3), round(ay0 + 0.001 * j, 3)))
               for i in range(nx + 1) for j in range(ny + 1))

OBJECTS = [
    Box("tape_front", pos=(X0 - TAPE_W / 2, (Y0 + Y1) / 2, TOP + TAPE_T / 2),
        size=(TAPE_W, Y1 - Y0 + 2 * TAPE_W, TAPE_T), color=GREEN),
    Box("tape_back", pos=(X1 + TAPE_W / 2, (Y0 + Y1) / 2, TOP + TAPE_T / 2),
        size=(TAPE_W, Y1 - Y0 + 2 * TAPE_W, TAPE_T), color=GREEN),
    Box("tape_right", pos=((X0 + X1) / 2, Y0 - TAPE_W / 2, TOP + TAPE_T / 2),
        size=(X1 - X0, TAPE_W, TAPE_T), color=GREEN),
    Box("tape_left", pos=((X0 + X1) / 2, Y1 + TAPE_W / 2, TOP + TAPE_T / 2),
        size=(X1 - X0, TAPE_W, TAPE_T), color=GREEN),
    *[Cylinder(n, pos=(x, y, TOP + POST_H / 2), radius=POST_R, height=POST_H, color=GREY)
      for n, (x, y) in POSTS.items()],
    Asset("cup", "yogurt_cup_hope_robolab", xy=CUP_XY, extent=CUP_EXTENT, mass=0.1),
]


def _upright(o):
    qx, qy = float(o.quat_xyzw[0]), float(o.quat_xyzw[1])
    return bool(1 - 2 * (qx * qx + qy * qy) > 0.99)


def score(objects):
    """Scored at the end, from the cup's centre (the centre of its round footprint). The cup counts only when it is
    upright and its centre is inside the rectangle, at least 3.5 cm from every inner tape edge (the whole cup inside).
    Then, with d the distance from the cup's centre to the nearest post's centre: 1.0 if d >= 10 cm, 0.5 if
    7 cm <= d < 10 cm, otherwise 0. The best possible d is 11.96 cm."""
    o = objects["cup"]
    x, y = float(o.pos[0]), float(o.pos[1])
    inside = bool(X0 + MARGIN <= x <= X1 - MARGIN and Y0 + MARGIN <= y <= Y1 - MARGIN)
    d = float(_nearest_post(x, y))
    print("DBG cup", [round(float(v), 3) for v in o.pos], "w", [round(float(h - l), 3) for h, l in zip(o.high, o.low)],
          "up", _upright(o), "in", inside, "d", round(d, 3), "best", BEST_XY, round(BEST_D, 3))
    if not (inside and _upright(o)):
        return 0.0
    return 1.0 if d >= D_FULL else 0.5 if d >= D_HALF else 0.0


TRAVEL_Z = TOP + 0.14  # 0.16 is out of reach above the cup's start; the cup still clears the 8 cm posts
GRIP_TIP = 0.02  # pad tips this far above the cup's base


def demo():
    """The right arm carries the cup from its start to the best spot, then moves clear of the top camera's view;
    the left arm stays parked."""
    z = TOP + PAD_TIP_PAST_TCP + GRIP_TIP
    # the best spot lies on the 3.5 cm line, so aim 5 mm further inside it
    bx = min(max(BEST_XY[0], X0 + MARGIN + 0.005), X1 - MARGIN - 0.005)
    by = min(max(BEST_XY[1], Y0 + MARGIN + 0.005), Y1 - MARGIN - 0.005)
    right = pick_place_waypoints((*CUP_XY, z), (bx, by, z + 0.004), travel_z=TRAVEL_Z)
    right.insert(2, Waypoint(0.6, (*CUP_XY, z), FINGER_OPEN, "settle"))
    right.append(Waypoint(2.0, (0.12, -0.40, TOP + 0.20), FINGER_OPEN, "move clear"))
    return {"right": right}


if __name__ == "__main__":
    print("best spot", BEST_XY, "nearest post", round(BEST_D, 3), "next best", round(NEXT_D, 3),
          "rectangle centre", round(_nearest_post((X0 + X1) / 2, (Y0 + Y1) / 2), 3),
          "corners", [round(_nearest_post(x, y), 3) for x, y in CORNERS], "continuous optimum", _continuous_optimum())
