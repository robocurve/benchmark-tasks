"""On the station table: a photo-display wire stretched taut between two posts, running out from the arms, with a
photo print hanging from it by one soft-grip wooden clothes peg at the print's far top corner, and more pegs clipped
on the rim of a peg basket to the right.

The photo has to be pegged at its other top corner too. A clothes peg is two wooden legs on a pivot with a torsion
spring: squeezing its handles together opens its jaws, and letting go closes them on whatever is between them. So a
peg has to be squeezed to come off the basket's rim, kept squeezed while it is carried, brought down so that its open
jaws pass either side of the wire and the photo's edge together (the wire 1.2 mm, the photo 0.3 mm, behind it), and
let go there.

Run it:    inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
Demo:      python demo.py task.py out_dir
Second layout: set the environment variable PEG_LOT=B before the command (default A).
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import pegkit as K  # noqa: E402
from pegkit import Fixed, Peg, Photo, finger_points, on_reset, on_step  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "The photo hangs from the wire by one peg. Peg its other top corner to the wire too."

# -- layout ---------------------------------------------------------------------------------------------------------
LOT = os.environ.get("PEG_LOT", "A")
LOTS = {"A": dict(photo_x=0.300, basket_y=-0.150, pegs_x=(0.360, 0.400)),
        "B": dict(photo_x=0.310, basket_y=-0.165, pegs_x=(0.355, 0.395))}
if LOT not in LOTS:
    raise ValueError(f"PEG_LOT must be one of {sorted(LOTS)}, got {LOT!r}")
PHOTO_X = LOTS[LOT]["photo_x"]          # where along the wire the photo hangs (its middle)
BASKET_Y = LOTS[LOT]["basket_y"]
PEGS_X = LOTS[LOT]["pegs_x"]

# -- the wire: a 1.2 mm steel photo-display wire, stretched taut between two posts, running out from the arms (x) ----
LINE_Y, LINE_Z, LINE_R = 0.000, 0.085, 0.0006
LINE_X = (0.170, 0.490)
POST = 0.020

# -- the photo: a 5 x 3.5 in (127 x 89 mm) print on 250 gsm photo paper, hanging beside the wire ------------------------
PHOTO = (0.127, 0.0003, 0.089)          # wide, thick, tall
PHOTO_MASS = 0.0028
PHOTO_QUAT = (0.0, 0.0, 0.0, 1.0)       # upright along the wire, the picture facing the right arm (-y)
ABOVE = 0.012                           # its top edge this far above the wire's axis
PEG_IN = 0.005                          # a corner's peg sits this far in from the photo's end

# -- the pegs and their basket -------------------------------------------------------------------------------------------
PEG_MASS = 0.006
PEG_QUAT = (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5))   # the legs part across the wire (y)
RIM_Z = 0.090                           # the basket's near wall's top, where the spare pegs are clipped
RIM_T = 0.003
CLIP_DEPTH = 0.005                      # the wire (or the rim) sits this far up from a clipped peg's jaw tips
OPEN_ON_WIRE = math.radians(-1.6)       # a peg clipped on the wire and the photo: its spring joint's angle

# -- scoring -----------------------------------------------------------------------------------------------------------
HANG_STEPS = 40                         # held this many control steps (2 s) for full marks
STILL = 0.005                           # m/s
CLEAR = 0.040                           # both grippers' tool points this far from a peg: let go
TAKEN = 0.020                           # a spare peg moved this far from its place on the basket has been taken
CORNER = 0.020                          # a peg within this of the photo's near end (along the wire) holds that corner
FALLEN = 0.010                          # the photo's top edge this far below where it hung: knocked off
SPARES = tuple(f"peg_{i + 1}" for i in range(len(PEGS_X)))
FIRST = "peg_far"


def beside() -> float:
    """The photo's middle plane, just beside the wire (toward +y)."""
    return LINE_Y + LINE_R + PHOTO[1] / 2


def photo_start() -> np.ndarray:
    return np.array([PHOTO_X, beside(), TOP + LINE_Z + ABOVE - PHOTO[2] / 2])


def corner_pivot(end: int) -> np.ndarray:
    """Where a peg clipped on the wire and the photo at its far (+1) or near (-1) top corner has its pivot's middle:
    leg a's jaw against the wire's near side, leg b's against the photo's back."""
    return np.array([PHOTO_X + end * (PHOTO[0] / 2 - PEG_IN), LINE_Y - LINE_R + K.GAP / 2,
                     TOP + LINE_Z - CLIP_DEPTH + K.PIVOT_Z])


def spare_start(i: int) -> np.ndarray:
    """A spare peg clipped on the basket's rim: its pivot's middle."""
    return np.array([PEGS_X[i], BASKET_Y, TOP + RIM_Z - CLIP_DEPTH + K.PIVOT_Z])


def _line() -> list:
    wire = (0.80, 0.81, 0.84)
    out = [Fixed("line", pos=(sum(LINE_X) / 2, LINE_Y, TOP + LINE_Z), size=(LINE_X[1] - LINE_X[0], 2 * LINE_R, 2 * LINE_R),
                 color=wire, roughness=0.3, metallic=0.8, static_friction=0.5, dynamic_friction=0.4)]
    for k, x in enumerate(LINE_X):
        side = -1 if k == 0 else 1
        out.append(Fixed(f"post_{k + 1}", pos=(x + side * POST / 2, LINE_Y, TOP + (LINE_Z + 0.012) / 2),
                         size=(POST, POST, LINE_Z + 0.012), color=(0.35, 0.25, 0.18), roughness=0.8))
    return out


def _basket() -> list:
    blue = (0.20, 0.45, 0.75)
    y1 = BASKET_Y + RIM_T / 2
    x0, x1 = min(PEGS_X) - 0.040, max(PEGS_X) + 0.040
    d = 0.12
    parts = [("basket_near", (x0, x1), (y1 - RIM_T, y1), RIM_Z), ("basket_far", (x0, x1), (y1 - d, y1 - d + RIM_T), RIM_Z),
             ("basket_left", (x1 - RIM_T, x1), (y1 - d, y1), RIM_Z), ("basket_right", (x0, x0 + RIM_T), (y1 - d, y1), RIM_Z),
             ("basket_floor", (x0, x1), (y1 - d, y1), 0.004)]
    return [Fixed(n, pos=((xs[0] + xs[1]) / 2, (ys[0] + ys[1]) / 2, TOP + h / 2), size=(xs[1] - xs[0], ys[1] - ys[0], h),
                  color=blue, roughness=0.6) for n, xs, ys, h in parts]


OBJECTS = [
    *_line(), *_basket(),
    Photo("photo", pos=tuple(float(v) for v in photo_start()), quat=PHOTO_QUAT, size=PHOTO, mass=PHOTO_MASS),
    Peg(FIRST, pos=tuple(float(v) for v in corner_pivot(+1)), quat=PEG_QUAT, size=(0.024, K.PEG_W, K.PEG_LEN),
        mass=PEG_MASS, open0=OPEN_ON_WIRE),
    *(Peg(SPARES[i], pos=tuple(float(v) for v in spare_start(i)), quat=PEG_QUAT, size=(0.024, K.PEG_W, K.PEG_LEN),
          mass=PEG_MASS, open0=math.radians(-3.0)) for i in range(len(SPARES))),
]


# -- what happened during the trial ---------------------------------------------------------------------------------
_STATE: dict = {}


def _clear() -> None:
    _STATE.update(taken=False, on_line=False, held_steps=0, held=False, fallen=False, settled_top=None, steps=0)


_clear()


def peg_on_line(pos, quat) -> bool:
    """Whether a peg's jaws are closed on the wire: the wire passes between the legs, low in the jaws."""
    rot = K._rotation(quat)
    x_line = float(np.clip(pos[0], *LINE_X))
    local = rot.T @ (np.array([x_line, LINE_Y, TOP + LINE_Z]) - pos)
    return (abs(local[0]) < 0.004 and abs(local[1]) < K.PEG_W / 2 + 0.003
            and -K.PIVOT_Z - 0.002 < local[2] < -K.PIVOT_Z + 0.016)


def photo_top_edge(photo_pos, photo_quat) -> np.ndarray:
    """The middle of the photo's top edge."""
    return np.asarray(photo_pos) + K._rotation(photo_quat)[:, 2] * PHOTO[2] / 2


def holds_near_corner(pos, quat, photo_pos, photo_quat) -> bool:
    """Whether a peg on the wire also holds the photo's near top corner: it sits within 2 cm of the photo's near end,
    and the photo's top edge passes between its legs."""
    top = photo_top_edge(photo_pos, photo_quat)
    near_end = top[0] - PHOTO[0] / 2
    return (peg_on_line(pos, quat) and near_end - 0.002 < pos[0] < near_end + CORNER
            and abs(top[1] - pos[1]) < 0.004)


@on_reset
def _reset(environment) -> None:
    _clear()


@on_step
def _watch(environment) -> None:
    from yam_arena.scene import object_states   # importable only once the simulator has started

    s = _STATE
    s["steps"] += 1
    states = object_states(environment, ["photo", FIRST, *SPARES])
    hands = finger_points(environment)
    ph = states["photo"]
    top = float(photo_top_edge(ph.pos, ph.quat_xyzw)[2])
    if s["steps"] == 10:
        s["settled_top"] = top                                          # where the photo hangs once settled
    if s["settled_top"] is not None and top < s["settled_top"] - FALLEN:
        s["fallen"] = True
    on_line, corner = False, False
    for i, p in enumerate(SPARES):
        st = states[p]
        if float(np.linalg.norm(np.asarray(st.pos) - spare_start(i))) > TAKEN:
            s["taken"] = True
        free = all(float(np.linalg.norm(np.asarray(st.pos) - h)) > CLEAR for h in hands)
        still = float(np.linalg.norm(st.lin_vel)) < STILL
        if free and still and peg_on_line(np.asarray(st.pos), st.quat_xyzw):
            on_line = True
            corner = corner or holds_near_corner(np.asarray(st.pos), st.quat_xyzw, np.asarray(ph.pos), ph.quat_xyzw)
    s["on_line"] = on_line
    good = corner and not s["fallen"] and float(np.linalg.norm(ph.lin_vel)) < STILL
    s["held_steps"] = s["held_steps"] + 1 if good else 0
    s["held"] = s["held"] or s["held_steps"] >= HANG_STEPS


# -- scoring --------------------------------------------------------------------------------------------------------
def score(objects) -> float:
    """0 if the photo was ever knocked off the wire (its top edge 1 cm below where it hung). Otherwise 1.0 once a
    second peg holds the photo's near top corner on the wire, let go, with the photo still, for 2 s; 0.5 if a peg is
    closed on the wire, let go; 0.25 if a peg has been taken off the basket; 0 otherwise."""
    s = _STATE
    if s["fallen"]:
        return 0.0
    if s["held"]:
        return 1.0
    if s["on_line"]:
        return 0.5
    return 0.25 if s["taken"] else 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0


def report(objects) -> dict:
    out = {k: _STATE[k] for k in ("taken", "on_line", "held_steps", "held", "fallen")}
    out["score"] = score(objects)
    for n in ("photo", FIRST, *SPARES):
        st = objects[n]
        out[n] = [round(100 * float(v), 2) for v in st.pos]
        if n != "photo":
            out[n + "_open_deg"] = round(math.degrees(st.joints.get("spring", 0.0)), 1)
    return out


def demo(objects, seed):
    return {}
