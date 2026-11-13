"""On the station table: four pocket books lying flat in a pile, left of centre, and a short empty shelf to its right.

Each book is 80 mm deep, 120 mm tall and 26 mm thick, weighs 160 g, and carries a four-letter code in block
letters down its spine. The pile is staggered, each book 14 mm nearer the arms than the one beneath it, with the
spines facing the arms, and the pile's order has nothing to do with the codes. The books have to end up standing
upright on the shelf, spines facing out, in alphabetical order of their codes from left to right.

A book lying flat cannot be stood up by a hand that comes straight down on it. It can be pinched by the spine edge
that sticks out past the book beneath, lifted out, and turned upright by a quarter turn of the wrist.

Run it:    inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
Demo:      python demo.py task.py out_dir
Second layout: set the environment variable BOOKS_LOT=B before the command (default A).
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bookkit import finger_points, on_reset, on_step, part  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = ("Stand the four books upright on the shelf with their spines facing out, in alphabetical order of the "
               "four-letter codes on their spines, from left to right.")

WOOD = (0.55, 0.38, 0.22)
STAND = (0.30, 0.31, 0.33)
LABEL = (0.96, 0.95, 0.90)
INK = (0.06, 0.06, 0.07)
COVERS = ((0.62, 0.14, 0.12), (0.12, 0.36, 0.20), (0.14, 0.24, 0.52), (0.72, 0.52, 0.12))   # by place in the pile

# -- book: a pocket-format paperback. Its own frame: x from the spine to the fore-edge, y from tail to head, z through
#    its thickness, origin at its centre. ----------------------------------------------------------------------------
BOOK_D, BOOK_H, BOOK_T = 0.080, 0.120, 0.026
BOOK_MASS = 0.16
BOOK_FRICTION = dict(static_friction=0.5, dynamic_friction=0.4, combine="average")
N = 4

# Block letters, five cells wide and seven tall, printed down the spine as on a real book: read from head to tail,
# upright when the book lies face up with its spine toward the reader.
CELL = 0.0030
LETTER_GAP = 0.0040
FONT = {
    "A": (".###.", "#...#", "#...#", "#####", "#...#", "#...#", "#...#"),
    "B": ("####.", "#...#", "#...#", "####.", "#...#", "#...#", "####."),
    "C": (".###.", "#...#", "#....", "#....", "#....", "#...#", ".###."),
    "D": ("####.", "#...#", "#...#", "#...#", "#...#", "#...#", "####."),
    "E": ("#####", "#....", "#....", "####.", "#....", "#....", "#####"),
    "F": ("#####", "#....", "#....", "####.", "#....", "#....", "#...."),
    "G": (".###.", "#...#", "#....", "#.###", "#...#", "#...#", ".###."),
    "H": ("#...#", "#...#", "#...#", "#####", "#...#", "#...#", "#...#"),
    "I": (".###.", "..#..", "..#..", "..#..", "..#..", "..#..", ".###."),
    "J": ("..###", "...#.", "...#.", "...#.", "...#.", "#..#.", ".##.."),
    "K": ("#...#", "#..#.", "#.#..", "##...", "#.#..", "#..#.", "#...#"),
    "L": ("#....", "#....", "#....", "#....", "#....", "#....", "#####"),
    "M": ("#...#", "##.##", "#.#.#", "#.#.#", "#...#", "#...#", "#...#"),
    "N": ("#...#", "##..#", "#.#.#", "#..##", "#...#", "#...#", "#...#"),
    "O": (".###.", "#...#", "#...#", "#...#", "#...#", "#...#", ".###."),
    "P": ("####.", "#...#", "#...#", "####.", "#....", "#....", "#...."),
    "Q": (".###.", "#...#", "#...#", "#...#", "#.#.#", "#..#.", ".##.#"),
    "R": ("####.", "#...#", "#...#", "####.", "#.#..", "#..#.", "#...#"),
    "S": (".####", "#....", "#....", ".###.", "....#", "....#", "####."),
    "T": ("#####", "..#..", "..#..", "..#..", "..#..", "..#..", "..#.."),
    "U": ("#...#", "#...#", "#...#", "#...#", "#...#", "#...#", ".###."),
    "V": ("#...#", "#...#", "#...#", "#...#", "#...#", ".#.#.", "..#.."),
    "W": ("#...#", "#...#", "#...#", "#.#.#", "#.#.#", "##.##", "#...#"),
    "X": ("#...#", "#...#", ".#.#.", "..#..", ".#.#.", "#...#", "#...#"),
    "Y": ("#...#", "#...#", ".#.#.", "..#..", "..#..", "..#..", "..#.."),
    "Z": ("#####", "....#", "...#.", "..#..", ".#...", "#....", "#####"),
}

# -- pile: flat on a low stand, spines toward the arms, each book STAGGER nearer the arms than the one beneath --------
STAND_H = 0.045                                # high enough for a level hand to get a finger under the bottom book
STAGGER = 0.014

# -- shelf: a board with a back and a left end, open on the right; the books stand on it side by side along y -------
SHELF_DEPTH = 0.095
SHELF_T = 0.020                                # the board's top is this far above the table
SHELF_WALL = 0.012
SHELF_WALL_H = 0.070                           # the back and the end rise this far above the board
PLACE_GAP = 0.009                              # the demo leaves this much room either side of each book
SHELF_LEN = N * BOOK_T + (N + 1) * PLACE_GAP   # length of the board from the left end's inside face: 0.149

# -- layout ----------------------------------------------------------------------------------------------------------
LOT = os.environ.get("BOOKS_LOT", "A")
LOTS = {  # the pile from top to bottom; where the top book's spine is; where the shelf's front left inside corner is
    "A": dict(pile=("MOTH", "FERN", "MINT", "BRIM"), pile_at=(0.390, 0.214), shelf_at=(0.420, 0.136)),
    "B": dict(pile=("DUSK", "WREN", "DUNE", "LARK"), pile_at=(0.384, 0.216), shelf_at=(0.414, 0.138)),
}
if LOT not in LOTS:
    raise ValueError(f"BOOKS_LOT must be one of {sorted(LOTS)}, got {LOT!r}")
PILE = LOTS[LOT]["pile"]                        # codes, top book first
PILE_X, PILE_Y = LOTS[LOT]["pile_at"]          # x of the top book's spine; y of the pile's middle
SHELF_X, SHELF_Y = LOTS[LOT]["shelf_at"]       # x of the board's front edge; y of the left end's inside face
BOOKS = tuple(f"book_{i + 1}" for i in range(N))           # book_1 is the top of the pile
CODE = dict(zip(BOOKS, PILE))
WANTED = tuple(sorted(BOOKS, key=CODE.get))                # left to right on the shelf
SPAWN_GAP = 0.0004

# -- scoring constants ---------------------------------------------------------------------------------------------
UPRIGHT_DEG = 10.0                             # head up, within this of vertical
FACING_DEG = 20.0                              # spine facing the front, within this
ON_BOARD = 0.004                               # the book's bottom is within this of the board's top
REST_SPEED = 0.05                              # m/s
HELD_GAP = 0.020                               # a book with a gripper's grasp point this close to it is held


def spine_marks(code: str) -> list:
    """The label and the letters of ``code`` on a book's spine, as boxes that are drawn but do not collide."""
    span = len(code) * 5 * CELL + (len(code) - 1) * LETTER_GAP
    x = -BOOK_D / 2
    marks = [((x - 0.0001, 0.0, 0.0), (0.0002, span + 0.016, 7 * CELL + 0.004), LABEL)]
    for k, letter in enumerate(code):
        left = span / 2 - k * (5 * CELL + LETTER_GAP)          # reading runs from head (+y) to tail (-y)
        for r, row in enumerate(FONT[letter]):
            c = 0
            while c < 5:
                if row[c] != "#":
                    c += 1
                    continue
                c1 = c
                while c1 + 1 < 5 and row[c1 + 1] == "#":
                    c1 += 1
                marks.append(((x - 0.0004, left - (c + c1 + 1) / 2 * CELL, (3 - r) * CELL),
                              (0.0004, (c1 - c + 1) * CELL, CELL), INK))
                c = c1 + 1
    return marks


def pile_spine_x(i: int) -> float:
    """World x of the spine of the pile's i-th book from the top, at the start."""
    return PILE_X + i * STAGGER


def book_start(i: int) -> tuple:
    """World position of the centre of the pile's i-th book from the top, at the start."""
    layer = N - 1 - i
    return (pile_spine_x(i) + BOOK_D / 2, PILE_Y, TOP + STAND_H + (layer + 0.5) * BOOK_T + (layer + 1) * SPAWN_GAP)


def _book(i: int):
    return part(BOOKS[i], pos=book_start(i), mass=BOOK_MASS, boxes=[((0.0, 0.0, 0.0), (BOOK_D, BOOK_H, BOOK_T), COVERS[i])],
                marks=spine_marks(PILE[i]), color=COVERS[i], contact_offset=0.002, **BOOK_FRICTION)


def _stand():
    """A low stand under the pile; the bottom book's spine edge sticks out past it like the others'."""
    x0, x1 = pile_spine_x(N - 1) + STAGGER, pile_spine_x(N - 1) + BOOK_D
    return part("stand", pos=((x0 + x1) / 2, PILE_Y, TOP), boxes=[((0.0, 0.0, STAND_H / 2), (x1 - x0, BOOK_H - 0.02, STAND_H), STAND)],
                color=STAND, **BOOK_FRICTION)


def _shelf():
    y_left, y_right = SHELF_Y, SHELF_Y - SHELF_LEN
    mid_y, top = (y_left + y_right) / 2, SHELF_T + SHELF_WALL_H

    def span(x0, x1, y0, y1, z0, z1):
        return (((x0 + x1) / 2 - SHELF_X, (y0 + y1) / 2 - mid_y, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0), WOOD)

    back = SHELF_X + SHELF_DEPTH
    boxes = [span(SHELF_X, back + SHELF_WALL, y_right, y_left + SHELF_WALL, 0.0, SHELF_T),      # board
             span(back, back + SHELF_WALL, y_right, y_left + SHELF_WALL, SHELF_T, top),         # back
             span(SHELF_X, back, y_left, y_left + SHELF_WALL, SHELF_T, top)]                    # left end
    return part("shelf", pos=(SHELF_X, mid_y, TOP), boxes=boxes, color=WOOD, **BOOK_FRICTION)


def slot_y(rank: int) -> float:
    """World y of the middle of the rank-th place on the shelf, counted from the left end."""
    return SHELF_Y - PLACE_GAP - BOOK_T / 2 - rank * (BOOK_T + PLACE_GAP)


OBJECTS = [_stand(), _shelf(), *(_book(i) for i in range(N))]
NAMES = list(BOOKS)                            # the parts that move


# -- what happened during the trial --------------------------------------------------------------------------------
_STATE: dict = {}


def _clear() -> None:
    _STATE.update(held=set())


_clear()


@on_reset
def _reset(environment) -> None:
    _clear()


def _rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def gap_to_book(state, point) -> float:
    """Distance from a point to a book's surface (0 inside it)."""
    p = _rot(state.quat_xyzw).T @ (np.asarray(point, float) - np.asarray(state.pos, float))
    out = np.maximum(np.abs(p) - np.array([BOOK_D, BOOK_H, BOOK_T]) / 2, 0.0)
    return float(np.linalg.norm(out))


@on_step
def _watch(environment) -> None:
    """Runs after every control step: notes which books a gripper is holding."""
    from yam_arena.scene import object_states   # importable only once the simulator has started

    states = object_states(environment, NAMES)
    hands = finger_points(environment)
    _STATE["held"] = {n for n in BOOKS if any(gap_to_book(states[n], h) < HELD_GAP for h in hands)}


# -- scoring -------------------------------------------------------------------------------------------------------
def lean_deg(state) -> float:
    """Angle between a book's tail-to-head direction and straight up (0 standing head up, 90 lying, 180 on its head)."""
    return math.degrees(math.acos(max(-1.0, min(1.0, float(_rot(state.quat_xyzw)[2, 1])))))


def facing_deg(state) -> float:
    """Angle between the way a book's spine faces and the shelf's front (toward the arms), seen from above."""
    r = _rot(state.quat_xyzw)
    out = -r[:2, 0]                               # the spine faces along the book's -x
    n = float(np.linalg.norm(out))
    return 180.0 if n < 1e-6 else math.degrees(math.acos(max(-1.0, min(1.0, float(-out[0] / n)))))


def shelved(objects, name: str) -> bool:
    """A book is shelved when it stands on the shelf's board head up, spine facing out, still and let go."""
    s = objects[name]
    x, y, z = (float(v) for v in s.pos)
    on_board = (SHELF_X <= x <= SHELF_X + SHELF_DEPTH and SHELF_Y - SHELF_LEN <= y <= SHELF_Y
                and abs(z - BOOK_H / 2 - (TOP + SHELF_T)) <= ON_BOARD)
    return (on_board and lean_deg(s) <= UPRIGHT_DEG and facing_deg(s) <= FACING_DEG
            and name not in _STATE["held"] and float(np.linalg.norm(s.lin_vel)) < REST_SPEED)


def row(objects) -> list:
    """The shelved books from left to right."""
    return sorted((n for n in BOOKS if shelved(objects, n)), key=lambda n: -float(objects[n].pos[1]))


def in_place(objects, name: str) -> bool:
    """A shelved book is in place when every shelved book to its left has an earlier code and every one to its right
    a later code."""
    r = row(objects)
    if name not in r:
        return False
    k = r.index(name)
    return all(CODE[m] < CODE[name] for m in r[:k]) and all(CODE[m] > CODE[name] for m in r[k + 1:])


def score(objects) -> float:
    return sum(in_place(objects, n) for n in BOOKS) / N


def success(objects) -> bool:
    return row(objects) == list(WANTED)


def report(objects) -> dict:
    """Where everything is, for result files."""
    books = {n: {"code": CODE[n], "shelved": shelved(objects, n), "in_place": in_place(objects, n),
                 "held": n in _STATE["held"], "lean_deg": round(lean_deg(objects[n]), 1),
                 "facing_deg": round(facing_deg(objects[n]), 1),
                 "head": [round(float(v), 3) for v in _rot(objects[n].quat_xyzw)[:, 1]],
                 "pos_cm": [round(100 * float(v), 2) for v in objects[n].pos]} for n in BOOKS}
    return {"books": books, "row": [CODE[n] for n in row(objects)], "wanted": [CODE[n] for n in WANTED]}


def demo() -> dict:
    """yam_scripted plays straight-down pick-and-place only; the demo is demo.py. Returning no waypoints holds both
    arms still, which is a check of the scene by itself."""
    return {}
