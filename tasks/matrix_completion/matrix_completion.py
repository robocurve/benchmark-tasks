"""On the station table: a 3 x 3 matrix of grey plates on the left half of the table, seven of them holding a fixed
piece and the two in the middle row on the right empty, and four loose answer pieces in front of the right arm. The
robot completes the pattern by putting the right loose piece on each empty plate.

Pieces are 5 cm tall painted wood in three shapes (cube 5 x 5 cm, cylinder 5 cm across, bar 7 cm along x by 3 cm
along y) and three colours (red, blue, yellow). Coordinates in cm, x forward, y left.
Matrix plates (fixed, grey, 8 x 8 x 0.5 cm): rows at x 38, 29, 20 (far to near), columns at y 32, 23, 14 (left to
right). Fixed pieces, by row: red cube, yellow cylinder, blue bar / blue cylinder, empty, empty / yellow bar,
blue cube, red cylinder.
Loose pieces (60 g): red cube (17, -21), red bar (17, -30), yellow cube (26, -21), yellow bar (26, -30). Neither arm
can reach in front of the other, so pieces change hands.

Run the scripted demo:  inspect-robots run --embodiment yam_arena --policy yam_scripted \\
    -E task=tasks/matrix_completion/matrix_completion.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 \\
    -E cam_height=176 --instruction "Complete the pattern: put the right pieces on the two empty squares." \\
    --scorer task_score --max-steps 2400 --store-frames --log-dir runs/matrix-completion-demo
"""

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

H = 0.05  # every piece's height
MASS = 0.06
COLORS = {
    "red": (0.85, 0.12, 0.12),
    "blue": (0.15, 0.30, 0.85),
    "yellow": (0.95, 0.80, 0.10),
}
PLATE_SIZE, PLATE_T, PLATE_COLOR = 0.08, 0.005, (0.5, 0.5, 0.5)
PLATE_TOP = TOP + PLATE_T
ROWS_X = (0.38, 0.29, 0.20)  # far to near
COLS_Y = (0.32, 0.23, 0.14)  # left to right, as the arms see it

# Shape (r + c) % 3 and colour (r + 2c) % 3 are two orthogonal Latin squares: each row and each column holds every
# shape and every colour once, and every shape/colour pair appears once.
SHAPES = ("cube", "cylinder", "bar")
HUES = ("red", "blue", "yellow")
EMPTY = {(1, 1): "red_bar", (1, 2): "yellow_cube"}  # cell -> the loose piece that belongs there


def cell_xy(r, c):
    return ROWS_X[r], COLS_Y[c]


def _piece(name, color, shape, xy, floor, mass=None):
    pos = (*xy, floor + H / 2)
    if shape == "cube":
        return Box(name, pos=pos, size=(0.05, 0.05, H), color=COLORS[color], mass=mass)
    if shape == "bar":
        return Box(name, pos=pos, size=(0.07, 0.03, H), color=COLORS[color], mass=mass)
    return Cylinder(name, pos=pos, radius=0.025, height=H, color=COLORS[color], mass=mass)


PLATES, TILES = [], []
for r in range(3):
    for c in range(3):
        xy = cell_xy(r, c)
        PLATES.append(Box(f"plate_{r}{c}", pos=(*xy, TOP + PLATE_T / 2), size=(PLATE_SIZE, PLATE_SIZE, PLATE_T),
                          color=PLATE_COLOR))
        if (r, c) not in EMPTY:
            shape, color = SHAPES[(r + c) % 3], HUES[(r + 2 * c) % 3]
            TILES.append(_piece(f"tile_{r}{c}_{color}_{shape}", color, shape, xy, PLATE_TOP))

START = {
    "red_cube": (0.17, -0.21),
    "red_bar": (0.17, -0.30),
    "yellow_cube": (0.26, -0.21),
    "yellow_bar": (0.26, -0.30),
}
OPTIONS = [_piece(n, *n.split("_"), xy, TOP, mass=MASS) for n, xy in START.items()]
OBJECTS = [*PLATES, *TILES, *OPTIONS]

# -- scoring: 0.5 per empty cell holding its answer piece, upright on the plate and still, with no other loose piece
# in that cell. Read at the end of the episode; there is no success(), so no early end reveals a right answer. -------
Z_TOL = 0.01


def in_cell(s, cell) -> bool:
    """The piece's centre is over the cell's plate (at any height)."""
    return bool(np.all(np.abs(s.pos[:2] - np.array(cell_xy(*cell))) < PLATE_SIZE / 2))


def cell_correct(objects, cell) -> bool:
    answer = objects[EMPTY[cell]]
    standing = abs(answer.low[2] - PLATE_TOP) < Z_TOL and abs(answer.high[2] - answer.low[2] - H) < Z_TOL
    alone = not any(in_cell(objects[n], cell) for n in START if n != EMPTY[cell])
    return bool(in_cell(answer, cell) and standing and answer.resting() and alone)


def score(objects):
    """0.5 per correctly filled empty cell."""
    return 0.5 * sum(cell_correct(objects, cell) for cell in EMPTY)


def distance(objects):
    """Summed distance (m) of each answer piece from where it stands on its cell."""
    return float(
        sum(
            np.linalg.norm(objects[name].pos - np.array([*cell_xy(*cell), PLATE_TOP + H / 2]))
            for cell, name in EMPTY.items()
        )
    )


# -- demo: the right arm sets the two answer pieces down near the centre line and moves aside; then the left arm
# puts each on its cell. Fingers close along y (yaw 0), across the bar's 3 cm. --------------------------------------
TRAVEL_Z = TOP + 0.14
RELEASE_GAP = 0.005
HANDOVER = {"red_bar": (0.20, 0.03), "yellow_cube": (0.29, 0.03)}


def _rest(arm, seconds):
    return [Waypoint(seconds, (0.20, 0.30 if arm == "left" else -0.30, TRAVEL_Z), FINGER_OPEN, "rest")]


def demo():
    right = []
    for name, xy in HANDOVER.items():
        right += pick_place_waypoints(
            (*START[name], TOP + H / 2), (*xy, TOP + H / 2 + RELEASE_GAP), travel_z=TRAVEL_Z
        )
    right += _rest("right", 2.0)
    left = _rest("left", sum(w.duration for w in right))  # wait until the right arm is out of the way
    for cell, name in EMPTY.items():
        left += pick_place_waypoints(
            (*HANDOVER[name], TOP + H / 2),
            (*cell_xy(*cell), PLATE_TOP + H / 2 + RELEASE_GAP),
            travel_z=TRAVEL_Z,
        )
    left += _rest("left", 2.0)
    return {"left": left, "right": right}
