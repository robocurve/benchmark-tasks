"""On the station table: a wooden ornament stand (a thick post with four dowel branches) and, either side of it, an
ornament box holding a glass bauble cap up.

The baubles have to be hung on the stand's branches. Each bauble hangs by a wire hook: a straight shank up from its
cap that curls over into a crook. To hang one, the crook has to be brought down over a branch (5 mm thick) so that
the shank and the crook's free end pass either side of it, and the bauble let go to hang from it. The glass breaks if
it is dropped or knocked hard.

Run it:    inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
Demo:      python demo.py task.py out_dir
Second layout: set the environment variable ORN_LOT=B before the command (default A).
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import ornkit as K  # noqa: E402
from ornkit import Fixed, Ornament, Rod, finger_points, on_reset, on_step  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Hang both glass baubles on the ornament stand's branches."

# -- layout ---------------------------------------------------------------------------------------------------------
LOT = os.environ.get("ORN_LOT", "A")
LOTS = {"A": dict(stand=(0.330, 0.000), boxes=((0.250, 0.200), (0.250, -0.200))),
        "B": dict(stand=(0.340, 0.010), boxes=((0.240, 0.215), (0.260, -0.190)))}
if LOT not in LOTS:
    raise ValueError(f"ORN_LOT must be one of {sorted(LOTS)}, got {LOT!r}")
STAND_XY = np.array(LOTS[LOT]["stand"])
BOX_XY = tuple(np.array(b) for b in LOTS[LOT]["boxes"])        # left box, right box

# -- the stand: a 25 mm post on a base, with four 5 mm dowel branches, each with a knob at its tip ------------------------
POST_D, POST_H = 0.025, 0.200
BASE = (0.140, 0.140, 0.012)
BRANCH_D, BRANCH_L = 0.005, 0.100
KNOB = 0.009
# each branch: (direction it points, height above the table)
BRANCHES = {"left": ((0.0, 1.0), 0.130), "right": ((0.0, -1.0), 0.130),
            "back": ((1.0, 0.0), 0.165), "front": ((-1.0, 0.0), 0.165)}
WOOD = (0.55, 0.38, 0.22)

# -- the baubles and their boxes --------------------------------------------------------------------------------------------
BAUBLE_MASS = 0.025                      # a 60 mm glass bauble with its cap and hook
COLOURS = ((0.80, 0.10, 0.12), (0.10, 0.35, 0.75))
NAMES = ("bauble_left", "bauble_right")
CELL, CELL_H, CELL_FLOOR = 0.062, 0.025, 0.004

# -- scoring -----------------------------------------------------------------------------------------------------------
HANG_STEPS = 40                          # every bauble hangs, let go and still, this many control steps (2 s)
STILL = 0.010                            # m/s
CLEAR = 0.040                            # both grippers' tool points this far from a bauble's cap: let go
BROKEN_SPEED = 1.0                       # m/s: a bauble moving this fast has been flung or dropped hard
FLOOR_GAP = 0.003                        # a bauble's lowest point this close to the table, outside its box: dropped


def branch_axis(name: str):
    """A branch's root (at the post's surface), tip, and direction."""
    (dx, dy), z = BRANCHES[name]
    d = np.array([dx, dy, 0.0])
    root = np.array([STAND_XY[0], STAND_XY[1], TOP + z]) + d * POST_D / 2
    return root, root + d * BRANCH_L, d


def hang_point(name: str, from_tip: float = 0.025) -> np.ndarray:
    """Where a bauble's centre is when it hangs from branch ``name``, ``from_tip`` in from its tip (its crook's
    inside resting on the branch's top). Valid for the side branches, whose baubles hang unturned."""
    _, tip, d = branch_axis(name)
    on = tip - d * from_tip
    rest = K.SHANK_Z[1] + K.CROOK_R - K.WIRE_R                         # the crook's inside top, above the ball's centre
    return np.array([on[0] - K.CROOK_R, on[1], on[2] + BRANCH_D / 2 - rest])


def bauble_start(i: int) -> np.ndarray:
    return np.array([BOX_XY[i][0], BOX_XY[i][1], TOP + CELL_FLOOR + K.BALL_R])


def _stand() -> list:
    x, y = STAND_XY
    out = [Fixed("stand_base", pos=(float(x), float(y), TOP + BASE[2] / 2), size=BASE, color=WOOD),
           Rod("stand_post", pos=(float(x), float(y), TOP + POST_H / 2), size=(POST_D, POST_H, 3), color=WOOD)]
    for name in BRANCHES:
        root, tip, d = branch_axis(name)
        axis = 2 if d[1] else 1
        mid = (root + tip) / 2
        out.append(Rod(f"branch_{name}", pos=tuple(float(v) for v in mid), size=(BRANCH_D, BRANCH_L, axis), color=WOOD))
        out.append(Rod(f"knob_{name}", pos=(float(tip[0]), float(tip[1]), float(tip[2]) + KNOB / 2 - BRANCH_D / 2),
                       size=(KNOB, KNOB, 3), color=(0.90, 0.75, 0.30)))
    return out


SEAT_W = 0.040                           # each box's card insert: a square opening the ball nests in
SEAT_TOP = CELL_FLOOR + K.BALL_R - math.sqrt(K.BALL_R ** 2 - (SEAT_W / 2) ** 2)   # resting on its four edges


def _boxes() -> list:
    out = []
    card = (0.85, 0.80, 0.70)
    for i, (x, y) in enumerate(BOX_XY):
        h = CELL / 2
        out.append(Fixed(f"box_{i}_floor", pos=(float(x), float(y), TOP + 0.001),
                         size=(CELL + 0.006, CELL + 0.006, 0.002), color=card))
        for k, (dx, dy, sx, sy) in enumerate(((h + 0.0015, 0, 0.003, CELL + 0.006), (-h - 0.0015, 0, 0.003, CELL + 0.006),
                                               (0, h + 0.0015, CELL, 0.003), (0, -h - 0.0015, CELL, 0.003))):
            out.append(Fixed(f"box_{i}_wall_{k}", pos=(float(x + dx), float(y + dy), TOP + CELL_H / 2),
                             size=(sx, sy, CELL_H), color=card))
        bar = (CELL - SEAT_W) / 2                                         # the insert, round the opening
        for k, (dx, dy, sx, sy) in enumerate(((SEAT_W / 2 + bar / 2, 0, bar, CELL), (-SEAT_W / 2 - bar / 2, 0, bar, CELL),
                                               (0, SEAT_W / 2 + bar / 2, SEAT_W, bar), (0, -SEAT_W / 2 - bar / 2, SEAT_W, bar))):
            out.append(Fixed(f"box_{i}_insert_{k}", pos=(float(x + dx), float(y + dy), TOP + 0.002 + (SEAT_TOP - 0.002) / 2),
                             size=(sx, sy, SEAT_TOP - 0.002), color=(0.75, 0.70, 0.62)))
    return out


OBJECTS = [
    *_stand(), *_boxes(),
    *(Ornament(NAMES[i], pos=tuple(float(v) for v in bauble_start(i)), size=(0.07, 0.06, 0.11), mass=BAUBLE_MASS,
               glass=COLOURS[i]) for i in range(len(NAMES))),
]


# -- what happened during the trial ---------------------------------------------------------------------------------
_STATE: dict = {}


def _clear() -> None:
    _STATE.update({n: dict(hung=False) for n in NAMES})
    _STATE.update(broken=False, all_steps=0, done=False, steps=0)


_clear()


def on_branch(pos, quat) -> str | None:
    """The branch a bauble hangs from by its crook, if any: the branch's axis passes through the crook."""
    rot = K._rotation(quat)
    for name in BRANCHES:
        root, tip, d = branch_axis(name)
        crook = pos + rot @ K.crook_centre()
        t = float(np.clip((crook - root) @ d, 0.0, BRANCH_L))
        local = rot.T @ (root + d * t - pos)                           # the nearest point of the branch, bauble frame
        if (abs(local[0] - K.CROOK_R) < 0.0045 and abs(local[1]) < 0.006
                and K.SHANK_Z[1] - 0.004 < local[2] < K.SHANK_Z[1] + K.CROOK_R + 0.002):
            return name
    return None


def in_box(pos) -> bool:
    return any(abs(pos[0] - b[0]) < CELL / 2 + 0.01 and abs(pos[1] - b[1]) < CELL / 2 + 0.01 for b in BOX_XY)


@on_reset
def _reset(environment) -> None:
    _clear()


@on_step
def _watch(environment) -> None:
    from yam_arena.scene import object_states   # importable only once the simulator has started

    s = _STATE
    s["steps"] += 1
    states = object_states(environment, list(NAMES))
    hands = finger_points(environment)
    every = True
    for n in NAMES:
        st = states[n]
        pos = np.asarray(st.pos)
        speed = float(np.linalg.norm(st.lin_vel))
        if s["steps"] > 5 and (speed > BROKEN_SPEED or (pos[2] - K.BALL_R < TOP + FLOOR_GAP and not in_box(pos))):
            s["broken"] = True
        cap = pos + K._rotation(st.quat_xyzw) @ np.array([0.0, 0.0, K.CAP_Z[1]])
        free = all(float(np.linalg.norm(cap - h)) > CLEAR for h in hands)
        s[n]["hung"] = bool(on_branch(pos, st.quat_xyzw) and free and speed < STILL)
        every = every and s[n]["hung"]
    s["all_steps"] = s["all_steps"] + 1 if every else 0
    s["done"] = s["done"] or s["all_steps"] >= HANG_STEPS


# -- scoring --------------------------------------------------------------------------------------------------------
def score(objects) -> float:
    """0 if a bauble was ever dropped or flung (broken glass). Otherwise 1.0 once both hang from branches by their
    hooks, let go and still, for 2 s; else 0.5 if at least one bauble hangs (let go and still) at the end; else 0."""
    s = _STATE
    if s["broken"]:
        return 0.0
    if s["done"]:
        return 1.0
    return 0.5 if any(s[n]["hung"] for n in NAMES) else 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0


def report(objects) -> dict:
    out = {"broken": _STATE["broken"], "all_steps": _STATE["all_steps"], "score": score(objects)}
    for n in NAMES:
        st = objects[n]
        rot = K._rotation(st.quat_xyzw)
        yaw = math.degrees(math.atan2(float(rot[1, 0]), float(rot[0, 0])))
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(rot[2, 2])))))
        out[n] = [round(100 * float(v), 2) for v in st.pos] + [int(_STATE[n]["hung"]), round(yaw, 1), round(tilt, 1)]
    return out


def demo(objects, seed):
    return {}
