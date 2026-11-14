"""On the station table: a 6 x 6 x 6 inch corrugated shipping carton holding four full 330 ml cans standing two by
two. Its top flaps are folded out and down against the walls; its four bottom flaps are folded in but not taped, so
nothing but what is under the carton keeps them shut. To the right of the carton stands a low white melamine shelf
whose top board is 12 cm above the table. The robot must put the carton, with all four cans in it, on the shelf.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from hingekit import Hinge, finger_points, on_reset, on_step, part  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Put the carton of cans on the shelf, with all the cans still in it."

KRAFT = (0.72, 0.55, 0.36)
KRAFT_DARK = (0.60, 0.44, 0.27)
WHITE = (0.92, 0.92, 0.90)
CAN_RED = (0.78, 0.08, 0.10)
ALU = (0.80, 0.81, 0.83)

# -- the carton, in its own frame: origin at the centre of its bottom face, x forward, y left (metres) --------------
SIZE = 0.152          # outside, a 6 x 6 x 6 inch regular slotted carton
WALL = 0.004          # single-wall corrugated board
HALF = SIZE / 2
IN = HALF - WALL      # inside half-width
FLAP = 0.004          # flap thickness; the outer (side) flaps lie under the inner (end) flaps
FLOOR = 2 * FLAP      # top of the folded bottom
OPEN_MAX = math.radians(120)

WALL_BOXES = [
    ((HALF - WALL / 2, 0.0, SIZE / 2), (WALL, SIZE, SIZE), KRAFT),       # back wall
    ((-HALF + WALL / 2, 0.0, SIZE / 2), (WALL, SIZE, SIZE), KRAFT),      # front wall
    ((0.0, HALF - WALL / 2, SIZE / 2), (SIZE - 2 * WALL, WALL, SIZE), KRAFT),   # left wall
    ((0.0, -HALF + WALL / 2, SIZE / 2), (SIZE - 2 * WALL, WALL, SIZE), KRAFT),  # right wall
]
# the four top flaps, folded out and down flat against the outside of the walls (each half the carton's width long);
# the rim is therefore two layers of board, 8 mm thick, and its top edge is the fold
TOP_FLAP = SIZE / 2
WALL_BOXES += [
    ((s * (HALF + WALL / 2), 0.0, SIZE - TOP_FLAP / 2), (WALL, SIZE - 2 * WALL, TOP_FLAP), KRAFT_DARK) for s in (1, -1)
] + [
    ((0.0, s * (HALF + WALL / 2), SIZE - TOP_FLAP / 2), (SIZE - 2 * WALL, WALL, TOP_FLAP), KRAFT_DARK) for s in (1, -1)
]
RIM_X = -HALF         # centre of the doubled front rim (wall plus folded flap), carton frame
GAP = 0.001  # the two flaps of a pair meet at the centre line with 1 mm between them
SLOT = 0.004  # each flap's side edges stop 4 mm short of the corners: the slots cut between a carton's flaps
FLAPS = (
    # outer flaps, hinged on the bottom edges of the left and right walls (axis x); a flap opens by swinging down
    Hinge("flap_left", anchor=(0.0, IN, FLAP / 2), axis="X", lower=0.0, upper=OPEN_MAX, damping=0.002, mass=0.010,
          boxes=(((0.0, (IN + GAP) / 2, FLAP / 2), (2 * (IN - SLOT), IN - GAP, FLAP), KRAFT_DARK),)),
    Hinge("flap_right", anchor=(0.0, -IN, FLAP / 2), axis="X", lower=-OPEN_MAX, upper=0.0, damping=0.002, mass=0.010,
          boxes=(((0.0, -(IN + GAP) / 2, FLAP / 2), (2 * (IN - SLOT), IN - GAP, FLAP), KRAFT_DARK),)),
    # inner flaps, hinged on the bottom edges of the front and back walls (axis y), lying on the outer ones
    Hinge("flap_front", anchor=(-IN, 0.0, 1.5 * FLAP), axis="Y", lower=0.0, upper=OPEN_MAX, damping=0.002,
          mass=0.010, boxes=(((-(IN + GAP) / 2, 0.0, 1.5 * FLAP), (IN - GAP, 2 * (IN - SLOT), FLAP), KRAFT),)),
    Hinge("flap_back", anchor=(IN, 0.0, 1.5 * FLAP), axis="Y", lower=-OPEN_MAX, upper=0.0, damping=0.002,
          mass=0.010, boxes=(((+(IN + GAP) / 2, 0.0, 1.5 * FLAP), (IN - GAP, 2 * (IN - SLOT), FLAP), KRAFT),)),
)
CARTON_MASS = 0.090   # walls and top flaps; with the four bottom flaps the empty carton weighs 130 g

CAN_R, CAN_H, CAN_MASS = 0.033, 0.115, 0.350   # 330 ml standard can, full
CAN_SPOTS = [(sx * 0.0345, sy * 0.0345) for sx in (1, -1) for sy in (1, -1)]

# -- fixed layout (world frame: x forward from the arms, y left, metres) ----------------------------------------------
CARTON_XY = (0.35, 0.05)
SHELF_XY = (0.46, -0.19)              # centre of the shelf board
SHELF_TOP = 0.120                      # above the table
SHELF_BOARD = (0.25, 0.30, 0.018)      # depth (x), width (y), thickness
SHELF_END = 0.018                      # end panels

CARTON = part("carton", pos=(*CARTON_XY, TOP + 0.0005), mass=CARTON_MASS, boxes=WALL_BOXES, hinges=FLAPS,
              color=KRAFT, friction=0.7, self_collide=True)
CANS = [part(f"can_{i}", pos=(CARTON_XY[0] + dx, CARTON_XY[1] + dy, TOP + FLOOR + CAN_H / 2 + 0.001), mass=CAN_MASS,
             cylinders=[((0.0, 0.0, 0.0), CAN_R, CAN_H, CAN_RED)], color=CAN_RED, friction=0.4)
        for i, (dx, dy) in enumerate(CAN_SPOTS)]
SHELF = [
    Box("shelf_board", pos=(*SHELF_XY, TOP + SHELF_TOP - SHELF_BOARD[2] / 2), size=SHELF_BOARD, color=WHITE),
    *[Box(f"shelf_end_{i}", pos=(SHELF_XY[0], SHELF_XY[1] + s * (SHELF_BOARD[1] / 2 - SHELF_END / 2),
                                  TOP + (SHELF_TOP - SHELF_BOARD[2]) / 2),
          size=(SHELF_BOARD[0], SHELF_END, SHELF_TOP - SHELF_BOARD[2]), color=WHITE) for i, s in enumerate((1, -1))],
]
OBJECTS = [*SHELF, CARTON, *CANS]
NAMES = ["carton", *[f"can_{i}" for i in range(4)]]

# -- scoring -----------------------------------------------------------------------------------------------------------
LIFT = 0.08            # carton clear of the table by more than a bottom flap's length (7.2 cm)
OVERHANG = 0.02        # at most 2 cm of the carton beyond the shelf board's edges
TILT_OK = math.radians(10)
RELEASE_GAP = 0.03     # every finger pad at least 3 cm from the carton
_STATE: dict = {}


def _reset_state(environment=None) -> None:
    _STATE.update(spilled=False, lifted=False, recent=[], released=False, finger_check=None, step=0, cache=None)


_reset_state()


def _can_inside(carton, can) -> bool:
    """A can is in the carton while its centre is inside the carton's walls, above the bottom."""
    c = carton.to_local(can.pos)
    return bool(abs(c[0]) < IN and abs(c[1]) < IN and -0.01 < c[2] < SIZE + 0.02)


@on_reset
def _on_reset(environment) -> None:
    _reset_state()


@on_step
def _fingers(environment) -> None:
    """Released: every finger pad of both arms at least 3 cm from the carton (score() cannot see the robot)."""
    from yam_arena.scene import object_states

    _STATE["step"] += 1
    try:
        carton = object_states(environment, ["carton"])["carton"]
        lo, hi = np.array([-HALF, -HALF, -0.01]), np.array([HALF, HALF, SIZE])
        gaps = [np.linalg.norm(np.maximum(np.maximum(lo - c, c - hi), 0.0))
                for c in (carton.to_local(p) for p in finger_points(environment))]
        _STATE["released"] = bool(min(gaps) >= RELEASE_GAP)
        _STATE["finger_check"] = True
    except Exception as exc:  # noqa: BLE001 - report once; an unknown grip never counts as released
        if _STATE["finger_check"] is None:
            print(f"open-carton: finger check unavailable: {exc!r}", flush=True)
        _STATE["finger_check"], _STATE["released"] = False, False


def _on_shelf(carton) -> tuple[bool, bool]:
    """(resting on the board, and within the overhang limit). Corners of the carton's bottom, in the world."""
    corners = np.array([carton.to_world((sx * HALF, sy * HALF, 0.0)) for sx in (1, -1) for sy in (1, -1)])
    bottom = TOP + SHELF_TOP
    on = bool(np.all(np.abs(corners[:, 2] - bottom) < 0.012) and
              abs(carton.pos[0] - SHELF_XY[0]) < SHELF_BOARD[0] / 2 and abs(carton.pos[1] - SHELF_XY[1]) < SHELF_BOARD[1] / 2)
    within = bool(np.all(np.abs(corners[:, 0] - SHELF_XY[0]) <= SHELF_BOARD[0] / 2 + OVERHANG) and
                  np.all(np.abs(corners[:, 1] - SHELF_XY[1]) <= SHELF_BOARD[1] / 2 + OVERHANG))
    return on, within


def _update(objects) -> tuple[float, bool]:
    # success() and score() are both called after each control step: evaluate once per step
    if _STATE["cache"] is not None and _STATE["cache"][0] == _STATE["step"]:
        return _STATE["cache"][1]
    result = _evaluate(objects)
    _STATE["cache"] = (_STATE["step"], result)
    return result


def _evaluate(objects) -> tuple[float, bool]:
    carton = objects["carton"]
    cans = [objects[f"can_{i}"] for i in range(4)]
    if not all(_can_inside(carton, c) for c in cans):
        _STATE["spilled"] = True           # a can that ever left the carton fails the trial, even if put back
    up = carton.rotation()[2, 2]
    lowest = min(carton.to_world((sx * HALF, sy * HALF, 0.0))[2] for sx in (1, -1) for sy in (1, -1))
    _STATE["lifted"] |= lowest > TOP + LIFT
    on, within = _on_shelf(carton)
    recent = _STATE["recent"]
    recent.append((carton.pos.copy(), _STATE["released"]))
    del recent[:-20]
    still = len(recent) == 20 and all(np.linalg.norm(p - carton.pos) < 0.002 and free for p, free in recent)
    if _STATE["spilled"]:
        return 0.0, False
    if on and within and up > math.cos(TILT_OK) and still:
        return 1.0, True
    if on and within:
        return 0.5, False
    return (0.25 if _STATE["lifted"] else 0.0), False


def success(objects) -> bool:
    """All four cans in the carton the whole time; the carton resting on the shelf board, upright within 10 degrees,
    no more than 2 cm beyond the board's edges, let go (every finger pad 3 cm away) and still over the last second."""
    return _update(objects)[1]


def score(objects) -> float:
    """0 if a can ever left the carton. Otherwise 1.0 for success; 0.5 carton resting on the shelf board within the
    overhang limit (still held, tilted or moving); 0.25 carton lifted with all its bottom corners 8 cm above the table at
    some point; else 0."""
    return _update(objects)[0]
