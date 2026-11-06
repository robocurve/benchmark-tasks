"""On the station table: four white sample plates lying in a row on a dark mat, a low dark deck behind them with a row of
four shallow pale-rimmed nests, and a peg board behind the deck with one hole lined up with each nest. A red peg stands
at each end of the board. A green mat lies at the right end of the deck.

One nest is faulty from the start: an orange block is glued to its floor, so no plate can sit flat there. The fault is
in the scene, not the robot's doing. What counts is what the robot does about it: a plate that won't sit flat comes out
of its nest and goes on the green mat, and a red peg goes in the board hole behind that nest.

Each plate has a yellow band round its foot. Seen level from the side, a seated plate's band is behind the rim and a
proud plate's stands above it. At Dream-1's 320 x 176 the band is under a pixel: the camera's cue is the block itself.

Run it:  inspect-robots run --embodiment yam_arena -E task=seat_four_plates_and_peg_any_that_wont_sit.py \
         --scorer task_score ...
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from benchkit import tool  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint, pick_place_waypoints  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = (
    "Set the four plates in the nests on the deck. If a plate won't sit flat, take it out, put it on the green mat and "
    "stand a red peg in the board hole behind its nest."
)

# -- the plates: long side along x, so the fingers close across their 6.2 cm width -------------------------------------
PLATE_L = 0.100
PLATE_W = 0.062
PLATE_H = 0.030
PLATE_MASS = 0.12
BAND_H = 0.008  # the yellow foot, as tall as the nest rim
ROW_Y = (0.165, 0.055, -0.055, -0.165)  # plates, nests and board holes share these, from the robot's left
BLOCKED = 2  # the third nest from the robot's left
GOOD = tuple(i for i in range(len(ROW_Y)) if i != BLOCKED)

# -- where things are ---------------------------------------------------------------------------------------------------
MAT_H = 0.001
SUPPLY_X = 0.15
SUPPLY_SIZE = (0.135, 0.470)
NEST_X = 0.285
NEST_L = 0.118  # 9 mm of play at each end of the plate, 10 mm at each side
NEST_W = 0.082
RIM_W = 0.005
RIM_H = BAND_H
DECK_SIZE = (0.130, 0.450, 0.010)
DECK_TOP = TOP + DECK_SIZE[2]
STOP_SIZE = (0.024, 0.050, 0.016)  # on the near half of the blocked nest's floor, so a plate's raised end faces the top camera
STOP_X = NEST_X - 0.016 - STOP_SIZE[0] / 2
HOLE_X = 0.375
HOLE_INNER = 0.040
HOLE_WALL = 0.003
HOLE_H = 0.025
PEG_R = 0.012
PEG_H = 0.060
PEG_MASS = 0.02
PEG_Y = 0.25
HOLD_XY = (0.285, -0.285)
HOLD_SIZE = (0.150, 0.100)

DARK = (0.22, 0.24, 0.27)
SLATE = (0.33, 0.37, 0.42)
PALE = (0.70, 0.71, 0.72)
WHITE = (0.93, 0.93, 0.91)
YELLOW = (0.95, 0.80, 0.10)
ORANGE = (0.92, 0.45, 0.08)
RED = (0.80, 0.08, 0.07)
GREEN = (0.16, 0.60, 0.34)

PLATES = tuple(f"plate{i}" for i in range(len(ROW_Y)))
PEGS = ("peg_left", "peg_right")


def _plate(i: int):
    """White top on a yellow foot, with the body origin at the plate's centre."""
    z = MAT_H + PLATE_H / 2
    return tool(
        PLATES[i],
        pos=(SUPPLY_X, ROW_Y[i], TOP + z),
        boxes=[((0.0, 0.0, (BAND_H - PLATE_H) / 2), (PLATE_L, PLATE_W, BAND_H), YELLOW),
               ((0.0, 0.0, BAND_H / 2), (PLATE_L, PLATE_W, PLATE_H - BAND_H), WHITE)],
        mass=PLATE_MASS,
        color=WHITE,
    )


def _row(name: str, x: float, base: float, inner_x: float, inner_y: float, wall: float, height: float, color) -> list[Box]:
    """Openings along y, one per ``ROW_Y``: two long rails and a pair of short walls at each opening."""
    span = 2 * ROW_Y[0] + inner_y + 2 * wall
    z = base + height / 2
    off_x, off_y = inner_x / 2 + wall / 2, inner_y / 2 + wall / 2
    rails = [Box(f"{name}_rail{i}", pos=(x + s * off_x, 0.0, z), size=(wall, span, height), color=color)
             for i, s in enumerate((-1, 1))]
    walls = [Box(f"{name}_wall{i}{j}", pos=(x, y + s * off_y, z), size=(inner_x, wall, height), color=color)
             for i, y in enumerate(ROW_Y) for j, s in enumerate((-1, 1))]
    return rails + walls


OBJECTS = [
    Box("supply_mat", pos=(SUPPLY_X, 0.0, TOP + MAT_H / 2), size=(*SUPPLY_SIZE, MAT_H), color=SLATE),
    Box("hold_mat", pos=(*HOLD_XY, TOP + MAT_H / 2), size=(*HOLD_SIZE, MAT_H), color=GREEN),
    Box("deck", pos=(NEST_X, 0.0, TOP + DECK_SIZE[2] / 2), size=DECK_SIZE, color=DARK),
    *_row("nest", NEST_X, DECK_TOP, NEST_L, NEST_W, RIM_W, RIM_H, PALE),
    Box("stop", pos=(STOP_X, ROW_Y[BLOCKED], DECK_TOP + STOP_SIZE[2] / 2), size=STOP_SIZE, color=ORANGE),
    *_row("board", HOLE_X, TOP, HOLE_INNER, HOLE_INNER, HOLE_WALL, HOLE_H, PALE),
    *(Box(f"board_fill{i}", pos=(HOLE_X, (ROW_Y[i] + ROW_Y[i + 1]) / 2, TOP + HOLE_H / 2),  # solid between the holes
          size=(HOLE_INNER, ROW_Y[i] - ROW_Y[i + 1] - HOLE_INNER - 2 * HOLE_WALL, HOLE_H), color=PALE)
      for i in range(len(ROW_Y) - 1)),
    *(Cylinder(name, pos=(HOLE_X, s * PEG_Y, TOP + PEG_H / 2), radius=PEG_R, height=PEG_H, mass=PEG_MASS, color=RED)
      for name, s in zip(PEGS, (1, -1))),
    *(_plate(i) for i in range(len(PLATES))),
]

# -- scoring -------------------------------------------------------------------------------------------------------
PER_PLATE = 0.15
ON_THE_MAT = 0.15
PEGGED = 0.40
PER_PLATE_LEFT_PROUD = 0.10  # peg or no peg, a plate left standing on the deck is the collision risk
SEATED_TOP = 0.034  # the highest corner of a seated plate, over the deck. Flat on the floor it is 3 cm
FLAT_DEG = 5.0
UPRIGHT_DEG = 35.0
BOTTOM_UP = 0.015  # the most a peg's foot can stand above the table and still be in a hole
LANDED = 0.060  # a plate's lowest corner above the table: higher than this and it's still in the gripper


def _tilt_deg(state) -> float:
    x, y, z, w = state.quat_xyzw
    return float(np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1.0, 1.0))))


def seated(state, nest: int) -> bool:
    """Down flat on a good nest's floor, centre inside the rim. The blocked nest can't seat a plate."""
    p = state.pos
    return bool(nest != BLOCKED and abs(p[0] - NEST_X) <= NEST_L / 2 and abs(p[1] - ROW_Y[nest]) <= NEST_W / 2
                and state.high[2] <= DECK_TOP + SEATED_TOP and _tilt_deg(state) <= FLAT_DEG)


def _over(p, xy, size) -> bool:
    return bool(abs(p[0] - xy[0]) <= size[0] / 2 and abs(p[1] - xy[1]) <= size[1] / 2)


def place_of(state) -> str:
    """Where a plate that isn't seated ended up: "air", "deck", "mat", "supply" or "lost"."""
    if any(seated(state, i) for i in GOOD):
        return "seated"
    if state.low[2] > TOP + LANDED:
        return "air"
    if _over(state.pos, (NEST_X, 0.0), DECK_SIZE):
        return "deck"
    if _over(state.pos, HOLD_XY, HOLD_SIZE):
        return "mat"
    if _over(state.pos, (SUPPLY_X, 0.0), SUPPLY_SIZE):
        return "supply"
    return "lost"


def in_hole(state, i: int) -> bool:
    """Standing in board hole ``i``: centre inside the walls, foot down, not lying across it."""
    p = state.pos
    return bool(abs(p[0] - HOLE_X) <= HOLE_INNER / 2 and abs(p[1] - ROW_Y[i]) <= HOLE_INNER / 2
                and state.low[2] < TOP + BOTTOM_UP and _tilt_deg(state) <= UPRIGHT_DEG)


def grade(objects) -> float:
    """The score as the outcome table states it."""
    pegs = [objects[name] for name in PEGS]
    if any(in_hole(peg, i) for peg in pegs for i in GOOD):
        return 0.0  # a false peg
    places = [place_of(objects[name]) for name in PLATES]
    if "lost" in places:
        return 0.0  # dropped, knocked or thrown out of the way
    n = places.count("seated")
    pegged = any(in_hole(peg, BLOCKED) for peg in pegs)
    if "deck" in places:  # standing on the deck, not seated
        return round(PER_PLATE_LEFT_PROUD * n, 6) if pegged else 0.0  # no peg claims all four
    total = PER_PLATE * n
    if n == len(GOOD):  # credit for the fourth plate is earned once the other three are done
        spare = next(p for p in places if p != "seated")
        total += ON_THE_MAT if spare == "mat" else 0.0
        total += PEGGED if pegged and spare != "air" else 0.0
    return round(total, 6)


def success(objects) -> bool:
    return grade(objects) >= 1.0 and all(objects[n].resting(max_speed=0.02, max_spin=1.0) for n in (*PLATES, *PEGS))


def score(objects) -> float:
    return grade(objects)


def distance(objects) -> float:
    """Metres from the nearest peg to the hole behind the blocked nest."""
    hole = np.array((HOLE_X, ROW_Y[BLOCKED]))
    return float(min(np.linalg.norm(objects[n].pos[:2] - hole) for n in PEGS))


# -- the scripted demo: each arm works its own half, and the two never reach the middle nests together ----------------------
GRIP_UP = 0.024  # the tool point above a plate's underside: the pad tips stay 11 mm up it, clear of the rim
DROP = 0.004  # plates and pegs are let go this far above where they land
TRAVEL_Z = TOP + 0.130
PICK_Z = TOP + MAT_H + GRIP_UP
SEAT_Z = DECK_TOP + DROP + GRIP_UP
TRY_Z = DECK_TOP + STOP_SIZE[2] + DROP + GRIP_UP  # the plate stops just over the block, still in the fingers
HOLD_PLACE_Z = TOP + MAT_H + DROP + GRIP_UP
PEG_GRIP_Z = TOP + 0.042
PEG_PLACE_Z = TOP + DROP + 0.042
PARK = {"left": (0.20, 0.27), "right": (0.20, -0.27)}
# Measured in the simulator. Plate 3 slides 17 mm across the right gripper as it lifts, and hangs 19 mm off the fingers'
# centre by the time it is lowered, so the fingers go that much to the left for it to come down inside the rim.
TRIM = {"pick": (0.0, 0.0), "nest": {0: (0.0, 0.0), 1: (0.0, 0.0), 3: (0.0, 0.019)}, "try": (0.0, 0.0),
        "hold": (0.0, 0.0), "peg": (0.0, 0.0), "hole": (0.0, 0.0)}


def _shift(xy, key, sub=None) -> tuple[float, float]:
    dx, dy = TRIM[key] if sub is None else TRIM[key][sub]
    return xy[0] + dx, xy[1] + dy


def _move(pick, place) -> list[Waypoint]:
    """Pick and place, with a pause over the target so the arm stops swinging before it goes down."""
    plan = pick_place_waypoints(pick, place, travel_z=TRAVEL_Z, speed=0.7)
    at = next(i for i, w in enumerate(plan) if w.label == "lower")
    plan.insert(at, Waypoint(0.8, plan[at - 1].pos, FINGER_CLOSED, "settle"))
    return plan


def _seat(i: int) -> list[Waypoint]:
    return _move((*_shift((SUPPLY_X, ROW_Y[i]), "pick"), PICK_Z), (*_shift((NEST_X, ROW_Y[i]), "nest", i), SEAT_Z))


def _try_the_blocked_nest() -> list[Waypoint]:
    """Lower the plate onto the block, stop there, lift it out without letting go and put it on the green mat. Letting go
    would leave it tilted, and an open-loop grip on a tilted plate is a gamble."""
    i = BLOCKED
    x, y = _shift((NEST_X, ROW_Y[i]), "try")
    mx, my = _shift(HOLD_XY, "hold")
    plan = _move((*_shift((SUPPLY_X, ROW_Y[i]), "pick"), PICK_Z), (x, y, TRY_Z))[:-2]  # stops at the end of "lower"
    return plan + [
        Waypoint(1.5, (x, y, TRY_Z), FINGER_CLOSED, "it won't go down"),
        Waypoint(2.0, (x, y, TRAVEL_Z), FINGER_CLOSED, "lift it out"),
        Waypoint(2.5, (mx, my, TRAVEL_Z), FINGER_CLOSED, "over the mat"),
        Waypoint(0.8, (mx, my, TRAVEL_Z), FINGER_CLOSED, "settle"),
        Waypoint(1.7, (mx, my, HOLD_PLACE_Z), FINGER_CLOSED, "lower"),
        Waypoint(0.9, (mx, my, HOLD_PLACE_Z), FINGER_OPEN, "release"),
        Waypoint(1.4, (mx, my, TRAVEL_Z), FINGER_OPEN, "up"),
    ]


def _peg_it() -> list[Waypoint]:
    return _move((*_shift((HOLE_X, -PEG_Y), "peg"), PEG_GRIP_Z), (*_shift((HOLE_X, ROW_Y[BLOCKED]), "hole"), PEG_PLACE_Z))


def _park(arm: str) -> Waypoint:
    return Waypoint(2.0, (*PARK[arm], TRAVEL_Z), FINGER_OPEN, "clear of the deck")


def demo():
    """Left arm: its two plates into their nests. Right arm: one plate into the outer nest, the next lowered into the
    blocked nest and carried out to the green mat, then the peg. The inner nests are worked in turn, so the arms stay
    apart."""
    return {
        "left": [*_seat(1), *_seat(0), _park("left")],
        "right": [*_seat(3), *_try_the_blocked_nest(), *_peg_it(), _park("right")],
    }
