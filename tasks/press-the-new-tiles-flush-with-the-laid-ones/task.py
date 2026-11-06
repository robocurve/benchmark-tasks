"""On the station table, between the arms and a little further out: a strip of floor being tiled, running left to
right. A grey concrete slab carries a row of five cream tiles, each 60 x 60 x 10 mm, with 3 mm joints. The middle
tile and the two end tiles are laid (darker cream). The second and fourth tiles are new (lighter cream): each sits
on a dark bed of fresh mortar and stands proud of the laid tiles around it. Fresh mortar gives way under a firm press and stays where it is pressed.
"""

import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import parts  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# World frame: x away from the arm bases, y to the robot's left, z up (m).
SLAB_GREY = (0.58, 0.58, 0.56)
CREAM = (0.88, 0.82, 0.70)  # laid tiles
NEW_CREAM = (0.97, 0.95, 0.90)  # new tiles, a lighter shade
MORTAR = (0.30, 0.30, 0.32)
ROW_X = 0.320
PITCH = 0.063  # 60 mm tile + 3 mm joint
TILE, TILE_T = 0.060, 0.010
SLAB_T = 0.030
SLAB_TOP = TOP + SLAB_T
FLUSH = SLAB_TOP + TILE_T  # the laid tiles' tops, 40 mm above the table
SLAB_X = 0.100
LAID_Y = (-2 * PITCH, 0.0, 2 * PITCH)
NEW = {"new_tile_left": (PITCH, 0.006), "new_tile_right": (-PITCH, 0.009)}  # y, starting height above flush

SLABS = [Box(f"slab_{i}", pos=(ROW_X, y, TOP + SLAB_T / 2), size=(SLAB_X, PITCH, SLAB_T), color=SLAB_GREY)
         for i, y in enumerate(LAID_Y)]
LAID = [Box(f"laid_tile_{i}", pos=(ROW_X, y, SLAB_TOP + TILE_T / 2), size=(TILE, TILE, TILE_T), color=CREAM)
        for i, y in enumerate(LAID_Y)]

# Fresh mortar: each new tile rides a slider (straight down, 15 mm travel) on a stiff spring whose rest point the
# ratchet below moves down as the tile is pressed. The slab under a new tile is the part's own body, which its
# moving tile and mortar pass into without colliding.
K = 1000.0  # N/m
YIELD = 0.0010  # the spring gives way 1.0 mm past the set depth: 1.0 N, of which the tile's weight supplies 0.49 N,
# so a press of about 0.51 N sinks the tile further; on release it springs back 0.51 mm to its resting depth
LINK_MASS = 0.05
SAG = LINK_MASS * 9.81 / K  # the tile and mortar's own 50 g presses them 0.49 mm into the spring
# Let go: resting on the mortar under its own weight, not pressed. A tile pressed with more than about 0.26 N sits
# more than 0.75 mm past its set depth.
RELEASED = (SAG + YIELD) / 2


def _new_tile(name, y, proud):
    return parts.Part(
        name, pos=(ROW_X, y, TOP),
        shapes=[parts.box((SLAB_X, PITCH, SLAB_T), at=(0.0, 0.0, SLAB_T / 2), color=SLAB_GREY)],
        joints=[parts.Slider(
            "bed", axis=(0.0, 0.0, -1.0), travel=0.015, mass=LINK_MASS, spring=K, damping=12.0, max_effort=200.0,
            shapes=[parts.box((TILE, TILE, proud), at=(0.0, 0.0, SLAB_T + proud / 2), color=MORTAR),
                    parts.box((TILE, TILE, TILE_T), at=(0.0, 0.0, SLAB_T + proud + TILE_T / 2), color=NEW_CREAM)])],
    )


NEW_TILES = [_new_tile(n, y, p) for n, (y, p) in NEW.items()]
OBJECTS = [*SLABS, *LAID, *NEW_TILES]

SUCCESS_HOLD_S = 3.0  # success must hold 3 s, so the trial (and its video) runs on while the arms move away
WINDOW = 0.0010  # flush: within 1.0 mm above or below the laid tiles' tops
RESET_JUMP = 0.002
EPS = 1e-6

# The mortar's set depth per tile (m pressed below the start) and whether a tile has ever been sunk more than 1 mm
# below flush. Both are cleared by an episode reset (the kit's reset callback, or, seen from the scorer alone, a
# tile jumping back up more than 2 mm to within 0.5 mm of its start in one step, which pressing cannot do).
_SET = {n: 0.0 for n in NEW}
_H = {"last_q": None, "sunk": False}


def _clear():
    for n in _SET:
        _SET[n] = 0.0
    _H.update(last_q=None, sunk=False)


def _ratchet(states) -> None:
    for n in NEW:
        q = float(states[n].joints["bed"])
        if q > _SET[n] + YIELD + EPS:
            _SET[n] = q - YIELD


@parts.on_step
def _hold_mortar(sim):
    states = sim.states()
    _ratchet(states)
    for n in NEW:
        sim.set_target(n, "bed", _SET[n])


@parts.on_reset
def _new_episode(sim):
    _clear()


def lippage(name) -> float:
    """Height of the tile's top above the laid tiles' tops (m; negative below), where it rests once let go."""
    return NEW[name][1] - (_SET[name] + SAG)


def _released(objects, name) -> bool:
    return float(objects[name].joints["bed"]) - _SET[name] <= RELEASED + EPS


def _update(objects) -> None:
    qs = {n: float(objects[n].joints["bed"]) for n in NEW}
    last = _H["last_q"]
    if last is not None and all(last[n] - qs[n] > RESET_JUMP and qs[n] < 0.0005 for n in NEW):
        _clear()
    _ratchet(objects)
    if any(lippage(n) < -WINDOW - EPS for n in NEW):
        _H["sunk"] = True
    _H["last_q"] = qs


def score(objects):
    """1.0: both new tiles let go and within 1.0 mm of flush with the laid tiles. 0.5: one let go within 1.0 mm,
    the other still proud. 0 otherwise, and 0 whenever either tile has been pressed more than 1.0 mm below flush at any point."""
    _update(objects)
    if _H["sunk"]:
        return 0.0
    flush = [abs(lippage(n)) <= WINDOW + EPS and _released(objects, n) for n in NEW]
    if all(flush):
        return 1.0
    if any(flush):
        return 0.5
    return 0.0


def success(objects):
    return score(objects) >= 1.0 - EPS


def distance(objects):
    return float(sum(abs(NEW[n][1] - float(objects[n].joints["bed"])) for n in NEW))


# Demo: one tile at a time, so the two wrists never meet (pressing both tiles at once puts the arms' wrists 22 mm
# into each other). The left arm presses the left tile at its centre with closed fingers, straight down to a fixed
# fingertip height below the laid tiles' tops, holds, lifts and parks clear while the right arm waits parked; then
# the right arm does the same with the right tile. The fingertip height allows for the arm giving way as it pushes
# and for the 0.51 mm spring-back.
TIP_PAST_TCP = 0.013
# Commanded fingertip depth below flush, per tile, set from the traces (see README.md).
PRESS_BELOW = {"new_tile_left": 0.0028, "new_tile_right": 0.0027}
CLEAR_X, CLEAR_Z = 0.200, FLUSH + 0.100  # parked 120 mm in front of the row and 100 mm above it
CLEAR_Y = {"left": 0.200, "right": -0.200}
TURN = 8.5  # s: the left arm's turn, then the right arm's


def _press(arm, name):
    y, proud = NEW[name]
    above = FLUSH + proud + 0.030 + TIP_PAST_TCP
    touch = FLUSH + proud + 0.002 + TIP_PAST_TCP
    press = FLUSH - PRESS_BELOW[name] + TIP_PAST_TCP
    park = (CLEAR_X, CLEAR_Y[arm], CLEAR_Z)
    return [
        Waypoint(1.5, (ROW_X, y, above), FINGER_CLOSED, "over the tile"),
        Waypoint(1.0, (ROW_X, y, touch), FINGER_CLOSED, "down to it"),
        Waypoint(3.0, (ROW_X, y, press), FINGER_CLOSED, "press"),
        Waypoint(0.5, (ROW_X, y, press), FINGER_CLOSED, "hold"),
        Waypoint(1.0, (ROW_X, y, above), FINGER_CLOSED, "lift away"),
        Waypoint(1.5, park, FINGER_CLOSED, "park clear of the floor"),
    ]


def demo():
    park_r = (CLEAR_X, CLEAR_Y["right"], CLEAR_Z)
    park_l = (CLEAR_X, CLEAR_Y["left"], CLEAR_Z)
    left = _press("left", "new_tile_left") + [Waypoint(TURN + 2.0, park_l, FINGER_CLOSED, "wait")]
    right = [Waypoint(1.5, park_r, FINGER_CLOSED, "park"), Waypoint(TURN - 1.5, park_r, FINGER_CLOSED, "wait")]
    right += _press("right", "new_tile_right") + [Waypoint(2.0, park_r, FINGER_CLOSED, "wait")]
    return {"left": left, "right": right}
