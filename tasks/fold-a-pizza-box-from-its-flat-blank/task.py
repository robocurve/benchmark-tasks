"""On the station table: a 10 in corrugated pizza box, still a flat die-cut blank, lying inside-up on a length of 2x8
board with its front toward the robot. The board is narrower and shorter than the blank, so every wall and flap hangs
over an edge, as a blank does on the edge of a prep counter.

The job is the whole fold, from blank to closed box:

1. the two side walls up, and the tab at the front end of each turned in along the front;
2. the front wall up outside the tabs, then rolled over them: its narrow cap across the top, its inner leaf down behind
   the tabs, the leaf's two feet into the base's two slots. That is the lock. It holds the three walls up by itself;
3. the lid over, with its two side flaps inside the side walls and its front flap inside the front wall.

The scores (the pressed fold lines) are stiff. A wall folded just to upright and let go springs most of the way back
down. Folding a side wall on past upright, in over the base, breaks its score in: the wall then comes back only to about
upright and stays there. A box folder does this to every wall before locking the front, and here it has to be done: with
two grippers nobody is left to hold the side walls up while the front wall is rolled over their tabs.

The blank's place changes with ``BOX_LOT`` (A or B).

Run it:    inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
Demo:      python demo.py task.py out_dir
"""

from __future__ import annotations

import ast
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import boxkit as K  # noqa: E402
from boxkit import Blank, on_reset, on_step  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = ("Fold this pizza box from its flat blank and close it: lock the front wall over the side walls' tabs, "
               "then shut the lid with all three of its flaps inside. Leave the closed box on the board and let go.")

# -- layout (m; x away from the robot, y left, heights above the table) ---------------------------------------------------
LOT = os.environ.get("BOX_LOT", "A")
CENTRE = np.array({"A": (0.360, 0.000), "B": (0.360, -0.006)}[LOT])     # the base's centre
if os.environ.get("BOX_CENTRE"):                                         # probes only: try another place
    CENTRE = np.array([float(v) for v in os.environ["BOX_CENTRE"].split(",")])
BOARD = (0.610, 0.184, 0.038)             # the board the blank lies on: 24 in of 2x8 (38 x 184 mm), length along x
BOARD_BACK = 0.015                        # the board's front end is this far behind the box's front crease
START = dict(ast.literal_eval(os.environ.get("BOX_START") or "{}"))      # starting folds, degrees (probes and tests only)

BLANK, SHELF = "blank", "board"

# -- scoring -------------------------------------------------------------------------------------------------------------
UP = 30.0                 # degrees: a wall is "up" within this of upright
PLUMB = 8.0               # degrees: a wall "stands" within this of upright (its top edge within 7 mm of plumb)
BACK_PLUMB = 10.0
POCKET = 0.012            # m: a tab is "under the roll-over" within this of the front wall and of the inner leaf, between them
FOOT_IN = K.T / 2 - 0.001  # m: a foot is in its slot when its end is at least 1 mm below the base's top face
LID_DOWN = 0.020          # m: the lid is "down" when its front edge is within this of the height of the walls' tops
LID_SHUT = 0.006          # ... and "shut" within this, lying level within LID_LEVEL degrees
LID_LEVEL = 5.0
FLAP_DEEP = 0.010         # m: a flap is "inside" when its free edge is this far below the walls' tops, inside the wall
CLEAR = 0.030             # m: a gripper's grasp point this far from the board counts as let go
STILL = 20.0              # degrees a second: creases at rest
WALLS = ("wall_l", "wall_r", "front")
LOCK_PARTS = ("wall_l", "wall_r", "front", "cap", "leaf", "tab_l", "tab_r")


def blank_pos() -> np.ndarray:
    return np.array([CENTRE[0], CENTRE[1], TOP + BOARD[2] + K.T / 2])


def board_pos() -> np.ndarray:
    return np.array([CENTRE[0] + K.XF + BOARD_BACK + BOARD[0] / 2, CENTRE[1], TOP + BOARD[2] / 2])


OBJECTS = [Box(SHELF, pos=tuple(float(v) for v in board_pos()), size=BOARD, color=(0.80, 0.68, 0.48)),
           Blank(BLANK, pos=tuple(float(v) for v in blank_pos()), size=(0.72, 0.36, K.T), mass=float(sum(K.masses().values())),
                 color=(0.93, 0.92, 0.88), start=tuple(START.items()))]


# -- what the box looks like --------------------------------------------------------------------------------------------
def in_base(panels: dict, name: str, local) -> np.ndarray:
    """A point of panel ``name`` (given in the blank's flat frame) in the base's frame."""
    Rb, pb = panels["base"]
    w = K.panel_points(name, panels[name], local)[0]
    return np.asarray(Rb, float).T @ (w - np.asarray(pb, float))


def side_of(panels: dict, name: str, world) -> float:
    """How far a world point is in front of panel ``name``'s inside face (m, along the face's normal)."""
    R, p = panels[name]
    return float((np.asarray(R, float).T @ (np.asarray(world, float) - np.asarray(p, float)))[2])


def measure(folds: dict, panels: dict, hands=()) -> dict:
    """Everything the score is built from, from the creases' fold angles (degrees) and the panels' poses."""
    m = {}
    m["sides_up"] = all(abs(folds[w] - 90.0) <= UP for w in ("wall_l", "wall_r"))
    x_leaf = K.XF - K.H - K.CAP - K.LEAF - K.FOOT
    feet = [in_base(panels, "leaf", (x_leaf, s * 0.5 * (K.FOOT_Y[0] + K.FOOT_Y[1]), 0.0)) for s in (1.0, -1.0)]
    m["feet_in"] = [bool(K.SLOT_X[0] - 0.001 <= f[0] - K.XF <= K.SLOT_X[1] + 0.001 and K.SLOT_Y[0] <= abs(f[1]) <= K.SLOT_Y[1]
                         and f[2] <= FOOT_IN) for f in feet]
    m["feet_mm"] = [[round(1000 * float(f[0] - K.XF), 1), round(1000 * float(f[2]), 1)] for f in feet]
    tabs = []
    rolled = abs(folds["front"] - 90.0) <= UP and folds["front"] + folds["cap"] + folds["leaf"] >= 225.0   # leaf turned down inside
    for s, n in ((1.0, "tab_l"), (-1.0, "tab_r")):
        tip = K.panel_points(n, panels[n], (K.XF + K.TAB_IN - K.TAB_L + 0.008, s * (K.YS + 0.5 * (K.TAB_Z[0] + K.TAB_Z[1])), 0.0))[0]
        a, b = side_of(panels, "front", tip), side_of(panels, "leaf", tip)
        tabs.append(bool(rolled and 0.0 < a <= POCKET and 0.0 < b <= POCKET))
    m["tabs_under"] = tabs
    m["plumb"] = {w: round(float(folds[w] - 90.0), 1) for w in WALLS + ("back",)}
    m["standing"] = all(abs(folds[w] - 90.0) <= PLUMB for w in WALLS)
    m["locked"] = bool(all(m["feet_in"]) and all(tabs) and m["standing"])
    front_edge = in_base(panels, "lid", (K.XB + K.H + K.LID_L, 0.0, 0.0))
    Rb, Rl = np.asarray(panels["base"][0], float), np.asarray(panels["lid"][0], float)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(-(Rb[:, 2] @ Rl[:, 2]))))))     # 0: lying level, inside face down
    m["lid_gap_mm"] = round(1000 * float(front_edge[2] - K.H), 1)
    m["lid_tilt"] = round(tilt, 1)
    m["lid_down"] = bool(front_edge[2] - K.H <= LID_DOWN and tilt <= 30.0)
    m["lid_shut"] = bool(front_edge[2] - K.H <= LID_SHUT and tilt <= LID_LEVEL and abs(folds["back"] - 90.0) <= BACK_PLUMB)
    inside = []
    for s, n in ((1.0, "flap_l"), (-1.0, "flap_r")):
        e = in_base(panels, n, (K.XB + K.H + K.LID_L / 2, s * (K.LID_W + K.SIDE_FLAP[2]), 0.0))
        inside.append(bool(abs(e[1]) < K.YS - K.T / 2 and e[2] <= K.H - FLAP_DEEP))
    e = in_base(panels, "flap_f", (K.XB + K.H + K.LID_L + K.FRONT_FLAP[1], 0.0, 0.0))
    inside.append(bool(e[0] - K.XF >= 0.008 and e[2] <= K.H - FLAP_DEEP))
    m["flaps_in"] = inside
    hands = [np.asarray(h, float) for h in hands]
    gap = lambda names: min([K.box_distance(h, n, panels[n]) for h in hands for n in names], default=9.0)   # noqa: E731
    m["lock_gap_mm"] = round(1000 * gap(LOCK_PARTS), 1)
    m["gap_mm"] = round(1000 * gap(K.ORDER), 1)
    m["walls_free"] = bool(gap(LOCK_PARTS) >= CLEAR)
    m["released"] = bool(gap(K.ORDER) >= CLEAR)
    return m


def score_from(m: dict, still: bool = True, on_board: bool = True) -> float:
    """Pure scoring rule, from :func:`measure`'s answers."""
    if not on_board:
        return 0.0
    locked = m["locked"] and m["walls_free"]
    if locked and m["lid_shut"] and all(m["flaps_in"]):
        return 1.0 if (m["released"] and still) else 0.75
    if locked and m["lid_down"]:
        return 0.75
    if locked:
        return 0.5
    if m["sides_up"] and any(m["tabs_under"]):
        return 0.25
    return 0.0


_STATE: dict = {}


def _clear() -> None:
    _STATE.clear()
    _STATE.update(steps=0, m=measure(K_FLAT, K.fk(), ()), still=True, on_board=True, folds=dict(K_FLAT))


K_FLAT = {n: 0.0 for n in K.ORDER[1:]}
_clear()


@on_reset
def _reset(environment) -> None:
    _clear()


@on_step
def _watch(environment) -> None:
    from yam_arena.scene import object_states   # importable only once the simulator has started

    st = _STATE
    st["steps"] += 1
    b = object_states(environment, [BLANK])[BLANK]
    st["folds"] = {k: round(float(v), 1) for k, v in b.folds.items()}
    st["m"] = measure(b.folds, b.panels, K.finger_points(environment))
    st["still"] = bool(max(b.speeds.values()) < STILL and float(np.linalg.norm(b.lin_vel)) < 0.02)
    st["on_board"] = bool(float(b.pos[2]) > TOP + BOARD[2] - 0.010)
    st["base_mm"] = [round(1000 * float(v), 1) for v in (b.pos - blank_pos())]


# -- scoring -------------------------------------------------------------------------------------------------------------
def score(objects) -> float:
    """Scored at the end of the trial, from how the box stands then. 0.25: both side walls are up (within 30 degrees of
    upright) and the front wall's inner leaf is rolled over at least one tab (the tab's free end lies between the front
    wall and the leaf). 0.5: the front wall is locked and holds by itself: both tabs under the roll-over, both feet in
    their slots (each foot's end at least 1 mm below the base's top face), the two side walls and the front wall
    each within 8 degrees of upright, and no gripper within 3 cm of them. 0.75: locked, and the lid is down (its front
    edge within 2 cm of the height of the walls' tops) but not shut, or a flap is outside, or a gripper is within 3 cm
    of the box, or something still moves. 1.0: locked; the lid shut (front edge within 6 mm of the walls' tops, level
    within 5 degrees, back wall within 10 degrees of upright); all three flaps inside; both grippers at least 3 cm from
    the box; at rest. 0 if the box ends off the board. Sliding, turning or tipping the box on the board costs
    nothing."""
    st = _STATE
    return score_from(st["m"], st["still"], st["on_board"])


def success(objects) -> bool:
    return score(objects) >= 1.0


def report(objects) -> dict:
    st = _STATE
    m = st["m"]
    return {"score": score(objects), "lot": LOT, "folds": st["folds"], "sides_up": m["sides_up"],
            "tabs_under": m["tabs_under"], "feet_in": m["feet_in"], "feet_mm": m["feet_mm"], "plumb": m["plumb"],
            "locked": m["locked"], "walls_free": m["walls_free"], "lid_gap_mm": m["lid_gap_mm"], "lid_tilt": m["lid_tilt"],
            "lid_down": m["lid_down"], "lid_shut": m["lid_shut"], "flaps_in": m["flaps_in"],
            "released": m["released"], "gap_mm": m["gap_mm"], "still": st["still"], "on_board": st["on_board"],
            "base_mm": st.get("base_mm")}


def demo(objects=None, seed=None):
    return {}
