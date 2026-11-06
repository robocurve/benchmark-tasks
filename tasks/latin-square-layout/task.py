"""On the station table: a slate-grey board, 26 x 50 cm, lies flat in the middle of the table, with white tape
lines dividing it into a 3 x 3 grid of squares. Each square is 8 cm front to back and 16 cm left to right. Rows run
left to right (near, middle, far from the arms); columns run front to back (left, middle, right).

Nine painted wooden blocks, all 2.5 x 2.5 cm in section, come in three colours (red, yellow, blue) and three lengths
(short 4 cm, medium 8 cm, long 12 cm): one block of every colour and length. Every block lies flat on a long face.

Three blocks are fixed to the board, centred in their squares with their long side running left to right, and
cannot move:
- red short in the near-left square
- blue long in the near-right square
- blue short in the centre square

The other six blocks are loose on the bare table, turned with their long side running front to back, in one column
on each side of the board: on the left (toward the left arm) blue medium (nearest the arms), yellow long, yellow
medium (farthest); on the right (toward the right arm) yellow short (nearest), red medium, red long (farthest).

The goal: one block in every square, with every row and every column holding one block of each colour and one
of each length. "Left" and "right" are as the arms face the table.
"""

import itertools
import math

import numpy as np

from yam_arena.constants import FINGER_CLOSED
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

# -- the board: 5 mm slate-grey hardboard with 6 mm white tape lines (1 mm thick) -------------------------------------
# x forward from the arm bases, y to the left; rows are indexed by x (0 near), columns by y (0 left, +y).

ROW_X = (0.20, 0.28, 0.36)  # cell centres front to back
COL_Y = (0.16, 0.0, -0.16)  # cell centres, left (+y) to right
PITCH_X, PITCH_Y = 0.08, 0.16
GRID_X = (ROW_X[0] - PITCH_X / 2, ROW_X[-1] + PITCH_X / 2)  # 0.16 .. 0.40: outer tape lines
GRID_Y = (COL_Y[-1] - PITCH_Y / 2, COL_Y[0] + PITCH_Y / 2)  # -0.24 .. 0.24
BOARD_MARGIN = 0.010  # board edge beyond the outer tape lines
BOARD_T = 0.005
BOARD_TOP = TOP + BOARD_T
TAPE_W, TAPE_T = 0.006, 0.001
BOARD_COLOR = (0.30, 0.32, 0.35)
TAPE_COLOR = (0.95, 0.95, 0.93)


def _box(name, x0, x1, y0, y1, z0, z1, color):
    return Box(name, pos=((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), size=(x1 - x0, y1 - y0, z1 - z0), color=color)


BOARD = [
    _box("board", GRID_X[0] - BOARD_MARGIN, GRID_X[1] + BOARD_MARGIN, GRID_Y[0] - BOARD_MARGIN,
         GRID_Y[1] + BOARD_MARGIN, TOP, BOARD_TOP, BOARD_COLOR),
]
for _k in range(4):  # lines across the table (constant x) and along it (constant y)
    _x = GRID_X[0] + PITCH_X * _k
    _y = GRID_Y[0] + PITCH_Y * _k
    BOARD.append(_box(f"tape_x{_k}", _x - TAPE_W / 2, _x + TAPE_W / 2, GRID_Y[0] - TAPE_W / 2, GRID_Y[1] + TAPE_W / 2,
                      BOARD_TOP, BOARD_TOP + TAPE_T, TAPE_COLOR))
    BOARD.append(_box(f"tape_y{_k}", GRID_X[0] - TAPE_W / 2, GRID_X[1] + TAPE_W / 2, _y - TAPE_W / 2, _y + TAPE_W / 2,
                      BOARD_TOP, BOARD_TOP + TAPE_T, TAPE_COLOR))

# -- the blocks: painted pine, 25 x 25 mm section, lying along y ---------------------------------------------------------

COLOURS = {"red": (0.80, 0.12, 0.12), "yellow": (0.95, 0.78, 0.10), "blue": (0.15, 0.30, 0.85)}
LENGTHS = {"short": 0.040, "medium": 0.080, "long": 0.120}  # 4 cm steps: visible in the 320 x 176 top camera
SECTION = 0.025
DENSITY = 500.0  # kg/m^3, pine
SPAWN_GAP = 0.001  # loose blocks spawn this far above the table and settle onto it


def block_size(name: str) -> tuple[float, float, float]:
    return (SECTION, LENGTHS[name.split("_")[1]], SECTION)


def attributes(name: str) -> tuple[str, str]:
    colour, length = name.split("_")
    return colour, length


# Fixed givens: (row, column) -> block. With these three, exactly one layout meets every rule (test_task.py checks).
GIVENS = {(0, 0): "red_short", (0, 2): "blue_long", (1, 1): "blue_short"}
# The one complete layout (rows near to far, columns left to right).
SOLUTION = {
    (0, 0): "red_short", (0, 1): "yellow_medium", (0, 2): "blue_long",
    (1, 0): "yellow_long", (1, 1): "blue_short", (1, 2): "red_medium",
    (2, 0): "blue_medium", (2, 1): "red_long", (2, 2): "yellow_short",
}  # fmt: skip
CELLS = tuple(itertools.product(range(3), range(3)))

# Loose blocks start on the bare table beside the board, each turned a quarter turn (long side front to back), in one
# column per side, away from where the arms' ready pose hides the table from the top camera.
STAGE_Y = {"left": 0.32, "right": -0.32}
STAGE_YAW = math.pi / 2
STAGING = {  # loose block -> (x, y) of its centre at the start
    "blue_medium": (0.17, STAGE_Y["left"]),
    "yellow_long": (0.30, STAGE_Y["left"]),
    "yellow_medium": (0.43, STAGE_Y["left"]),
    "yellow_short": (0.15, STAGE_Y["right"]),
    "red_medium": (0.24, STAGE_Y["right"]),
    "red_long": (0.38, STAGE_Y["right"]),
}


def cell_xy(cell) -> tuple[float, float]:
    return ROW_X[cell[0]], COL_Y[cell[1]]


def _block(name, xy, z_bottom, loose, yaw=0.0):
    size = block_size(name)
    mass = DENSITY * size[0] * size[1] * size[2] if loose else None
    colour = COLOURS[name.split("_")[0]]
    return Box(name, pos=(xy[0], xy[1], z_bottom + size[2] / 2), size=size, mass=mass, color=colour, yaw=yaw)


FIXED_BLOCKS = [_block(n, cell_xy(c), BOARD_TOP, loose=False) for c, n in GIVENS.items()]
LOOSE_BLOCKS = [_block(n, xy, TOP + SPAWN_GAP, loose=True, yaw=STAGE_YAW) for n, xy in STAGING.items()]
BLOCKS = {b.name: b for b in (*FIXED_BLOCKS, *LOOSE_BLOCKS)}
LOOSE = tuple(STAGING)
OBJECTS = [*BOARD, *FIXED_BLOCKS, *LOOSE_BLOCKS]
GOAL_CELL = {n: c for c, n in SOLUTION.items()}

# -- scoring --------------------------------------------------------------------------------------------------------

ON_BOARD = 0.006  # a block's lowest point within 6 mm of the board top (lying on it or on the 1 mm tape)
BELOW_BOARD = 0.004  # and not more than 4 mm below it (it fell off)
LINES = tuple(
    [*([(r, c) for c in range(3)] for r in range(3)), *([(r, c) for r in range(3)] for c in range(3))]
)  # three rows, then three columns


def cell_of(state):
    """The (row, column) whose tape-bounded square holds the block's centre (seen from above), or None."""
    x, y = float(state.pos[0]), float(state.pos[1])
    if not (GRID_X[0] <= x <= GRID_X[1] and GRID_Y[0] <= y <= GRID_Y[1]):
        return None
    row = min(int((x - GRID_X[0]) // PITCH_X), 2)
    col = min(int((GRID_Y[1] - y) // PITCH_Y), 2)
    return row, col


def seated(state) -> bool:
    """Resting directly on the board: bottom within 6 mm above (and at most 4 mm below) the board top, and still.
    A block on top of another block, held in the air, or fallen to the table or floor is not seated."""
    low = float(state.low[2])
    return bool(BOARD_TOP - BELOW_BOARD <= low <= BOARD_TOP + ON_BOARD and state.resting())


def layout(objects) -> dict:
    """(row, column) -> the one block seated in that square, or None if it holds none or more than one."""
    found = {c: [] for c in CELLS}
    for name in BLOCKS:
        s = objects[name]
        cell = cell_of(s)
        if cell is not None and seated(s):
            found[cell].append(name)
    return {c: (names[0] if len(names) == 1 else None) for c, names in found.items()}


def line_ok(grid, line) -> bool:
    """Three squares each with exactly one block, three different colours and three different lengths."""
    names = [grid[c] for c in line]
    if any(n is None for n in names):
        return False
    colours = {attributes(n)[0] for n in names}
    lengths = {attributes(n)[1] for n in names}
    return len(colours) == 3 and len(lengths) == 3


def lines_satisfied(objects) -> int:
    grid = layout(objects)
    return sum(line_ok(grid, line) for line in LINES)


def score(objects):
    """1/6 for every row and every column (six lines) whose three squares each hold exactly one seated block of
    three different colours and three different lengths; 0..1."""
    return lines_satisfied(objects) / 6.0


def success(objects):
    """All six lines satisfied and every loose block still; must hold for 1 s."""
    return bool(lines_satisfied(objects) == 6 and all(objects[n].resting() for n in LOOSE))


def distance(objects):
    """Metres: summed over the six loose blocks, the horizontal distance from the block's centre to the centre of
    its square in the one complete layout, plus 0.05 m for each of those blocks not seated on the board."""
    total = 0.0
    for n in LOOSE:
        s = objects[n]
        total += float(np.linalg.norm(np.asarray(s.pos[:2], float) - np.asarray(cell_xy(GOAL_CELL[n]))))
        total += 0.05 * (not seated(s))
    return total


# -- demo ------------------------------------------------------------------------------------------------------------
# Each arm moves three blocks, one at a time. It grips each staged block across its 25 mm width (fingers closing
# along y, yaw 0), lifts it, turns it a quarter turn on the way so its long side runs left to right, and sets it in
# its square with the fingers closing along x; any off-centre grip then lands along x, where a square has 24 mm of
# slack each side. The left arm fills the left column's two empty squares and the near middle square; the right arm
# the right column's two and the far middle square. The right arm starts half a move (5.2 s) later, so whenever one
# arm is setting a block down on the board the other is at its own staging column: two wrists side by side over
# neighbouring squares came within 1 cm of each other in the offline sweep.

DEMO = {
    "left": ("yellow_medium", "yellow_long", "blue_medium"),
    "right": ("red_medium", "yellow_short", "red_long"),
}
PICK_YAW = {"left": 0.0, "right": 0.0}
PLACE_YAW = {"left": -math.pi / 2, "right": math.pi / 2}
GRIP_ABOVE_BOTTOM = 0.019  # TCP this far above the block's bottom: pad tips 6 mm above its support
LIFT_CLEAR = 0.045  # carried block's bottom this far above the board: 20 mm over standing blocks
RELEASE_GAP = 0.003
CLEAR_RISE = 0.030  # after letting go, rise this far before any sideways move
HOVER_Z = TOP + 0.100
CARRY_Z = BOARD_TOP + LIFT_CLEAR + GRIP_ABOVE_BOTTOM
APPROACH_CLEARANCE = 0.010  # each pad this far off the block on the way down
LOOSEN = 0.006  # each pad backs off this far to let go
PARK = {"left": (0.15, 0.40, HOVER_Z), "right": (0.15, -0.40, HOVER_Z)}  # beside the board, out of its top view
PARK_YAW = 0.0  # the wrist turns back to yaw 0 while parking: -90 deg out there is past joint6's limit
FINAL_HOLD_S = 2.0
STAGGER_S = 5.2  # the right arm's head start delay, and the left arm's extra hold at the end


def _finger(clearance: float) -> float:
    """Finger joint target leaving ``clearance`` between each pad and the 25 mm block."""
    return -(SECTION / 2 + clearance)


def move_waypoints(name: str, side: str) -> list[Waypoint]:
    """Grip a staged block, lift it to carry height, carry it over its square while turning it a quarter turn, set
    it down from 3 mm, let go, rise 3 cm, then go up to the hover height."""
    px, py = STAGING[name]
    qx, qy = cell_xy(GOAL_CELL[name])
    yaw, place_yaw = PICK_YAW[side], PLACE_YAW[side]
    approach, loose = _finger(APPROACH_CLEARANCE), _finger(LOOSEN)
    grip_z = TOP + GRIP_ABOVE_BOTTOM
    release_z = BOARD_TOP + GRIP_ABOVE_BOTTOM + RELEASE_GAP
    return [
        Waypoint(1.5, (px, py, HOVER_Z), approach, "to the block", yaw),
        Waypoint(1.5, (px, py, grip_z), approach, "descend", yaw),
        Waypoint(0.8, (px, py, grip_z), FINGER_CLOSED, "close", yaw),
        Waypoint(1.0, (px, py, CARRY_Z), FINGER_CLOSED, "lift", yaw),
        Waypoint(2.0, (qx, qy, CARRY_Z), FINGER_CLOSED, "carry", place_yaw),
        Waypoint(1.0, (qx, qy, release_z), FINGER_CLOSED, "lower", place_yaw),
        Waypoint(0.6, (qx, qy, release_z), loose, "release", place_yaw),
        Waypoint(1.0, (qx, qy, release_z + CLEAR_RISE), loose, "rise clear", place_yaw),
        Waypoint(1.0, (qx, qy, HOVER_Z), loose, "up", place_yaw),
    ]


def demo():
    """39.9 s per arm: three blocks each, the right arm starting 5.2 s after the left, then both park beside the board
    and hold still (the right arm 2 s, the left arm 7.2 s)."""
    plans = {}
    for side, names in DEMO.items():
        plan = [w for n in names for w in move_waypoints(n, side)]
        loose = _finger(LOOSEN)
        hold = FINAL_HOLD_S + (STAGGER_S if side == "left" else 0.0)
        plan += [
            Waypoint(1.5, PARK[side], loose, "park", PARK_YAW),
            Waypoint(hold, PARK[side], loose, "hold still", PARK_YAW),
        ]
        if side == "right":  # wait above the first block while the left arm makes its first move
            first = plan[0]
            plan.insert(0, Waypoint(STAGGER_S, first.pos, first.finger, "wait", first.yaw))
        plans[side] = plan
    return plans
