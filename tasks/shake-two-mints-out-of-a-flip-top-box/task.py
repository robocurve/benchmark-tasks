"""On the station table: a flip-top mint box lies on its side with its lid shut, part full of loose mints, and beside
it stands a small square dish.

The job is the one a person does with such a box many times a week: flick the lid open, hold the box over the dish,
and shake it so that mints come out of the small opening; stop at two; snap the lid shut and put the box down. The
opening passes one mint at a time and the mints jam behind it: a box tipped over and held still lets out one mint or
none, and a box shaken on lets out too many. Counting to two is the easy part; letting out two and no more is the
task.

Where the box and the dish stand, and how many mints the box holds, change with ``MINT_LOT`` (A or B).

Run it:    inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
Demo:      python demo.py task.py out_dir
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import mintkit as K  # noqa: E402
from mintkit import Flap, Mint, MintBox, on_reset, on_step  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Shake two mints out of the mint box into the dish, then close the box and put it back on the table."

# -- layout (m; x away from the robot, y left, heights above the table) ---------------------------------------------------
LOT = os.environ.get("MINT_LOT", "A")
# box: the centre of the box, which lies on a big face with its lid end toward the robot, turned yaw degrees about
# the vertical. dish: the centre of the dish. mints: how many the box holds.
LAYOUT = {
    "A": {"box": (0.300, 0.030), "yaw": 0.0, "dish": (0.200, 0.190), "mints": 30},
    "B": {"box": (0.315, 0.000), "yaw": 10.0, "dish": (0.215, 0.165), "mints": 24},
}[LOT]
BOX_XY, BOX_YAW, DISH_XY, COUNT = LAYOUT["box"], math.radians(LAYOUT["yaw"]), LAYOUT["dish"], LAYOUT["mints"]
GAP = 0.0004                       # loose parts start this far above what they rest on

BOX = "mint_box"
LID = "lid"
MINTS = tuple(f"mint_{k:02d}" for k in range(COUNT))
DISH = tuple(f"dish_{tag}" for tag, _, _ in K.dish_boards(DISH_XY))

# -- scoring -----------------------------------------------------------------------------------------------------------
WANT = 2                 # mints asked for
CLEAR = 0.030            # both grippers this far from the box for it to count as put down
ON_TABLE = 0.003         # ... and its lowest point within this of the table top
STILL, STILL_TURN = 0.02, 0.5    # m/s and rad/s
HOLD = 20                # control steps (1 s) the full-score state has to last before the trial ends


def box_start() -> tuple:
    """The box's position (its centre) and quaternion at the start."""
    return np.array([BOX_XY[0], BOX_XY[1], TOP + K.T / 2 + GAP]), K._quat(K.lying_rot(BOX_YAW))


def mint_start(k: int) -> tuple:
    pos, quat = box_start()
    local, rot = K.mint_grid(COUNT)[k]
    return K.place(pos, quat, local), K._quat(K._rotation(quat) @ rot)


def _objects() -> list:
    tup = lambda v: tuple(float(x) for x in v)   # noqa: E731
    objs = [Box(name, pos=tup(centre + [0.0, 0.0, TOP]), size=tup(size), color=K.DISH_BLUE)
            for name, (_, centre, size) in zip(DISH, K.dish_boards(DISH_XY))]
    pos, quat = box_start()
    objs.append(MintBox(BOX, pos=tup(pos), quat=quat, size=(K.W, K.T, K.H), mass=K.BOX_MASS))
    fp, fq = K.flap_pose(pos, quat, 0.0)
    objs.append(Flap(LID, pos=tup(fp), quat=fq, size=(K.FLAP_L + K.LIP, K.FLAP_W, K.FLAP_T), mass=K.FLAP_MASS, box=BOX))
    for k, name in enumerate(MINTS):
        mp, mq = mint_start(k)
        objs.append(Mint(name, pos=tup(mp), quat=mq, size=(K.MINT_L, K.MINT_D, K.MINT_D), mass=K.MINT_MASS))
    return objs


OBJECTS = _objects()


# -- what the trial looks like now ------------------------------------------------------------------------------------
_STATE: dict = {}


def _clear() -> None:
    _STATE.update(steps=0, in_dish=0, in_box=COUNT, spilled=0, lid=0.0, shut=True, down=True, near=0.0, low=0.0,
                  held=0, left=[], out=[], score=0.0)


_clear()


def tally(mints, box_pos, box_quat) -> tuple:
    """Count the mints by where their centres are: (in the dish, in the box, anywhere else)."""
    in_dish = in_box = 0
    for p in mints:
        if K.in_box(p, box_pos, box_quat):
            in_box += 1
        elif K.in_dish(p, DISH_XY, TOP):
            in_dish += 1
    return in_dish, in_box, len(mints) - in_dish - in_box


def box_down(low: float, speed: float, spin: float, near: float, on_dish: bool) -> bool:
    """The box has been put down: its lowest point within 3 mm of the table top, no part of it over the dish, at rest
    (under 2 cm/s and 0.5 rad/s), and both grippers at least 3 cm from it."""
    return bool(low <= ON_TABLE and not on_dish and speed < STILL and spin < STILL_TURN and near >= CLEAR)


def score_from(in_dish: int, spilled: int, shut: bool, down: bool) -> float:
    """Pure scoring rule. ``in_dish``: mints in the dish. ``spilled``: mints that are neither in the dish nor in the
    box. ``shut``: the lid is within 3 degrees of closed. ``down``: the box has been put down (see ``box_down``).
    With a mint spilled: 0.25 if exactly two are in the dish, otherwise 0. With none spilled: 1.0 for exactly two in
    the dish, 0.5 for one or three, 0 for none or for four or more; halved unless the lid is shut and the box is down."""
    if spilled:
        return 0.25 if in_dish == WANT else 0.0
    base = {WANT: 1.0, WANT - 1: 0.5, WANT + 1: 0.5}.get(in_dish, 0.0)
    return base if (shut and down) else base / 2


@on_reset
def _reset(environment) -> None:
    _clear()


@on_step
def _watch(environment) -> None:
    from yam_arena.scene import object_states   # importable only once the simulator has started

    s = _STATE
    s["steps"] += 1
    st = object_states(environment, [BOX, LID, *MINTS])
    box = st[BOX]
    in_dish, in_box, spilled = tally([st[m].pos for m in MINTS], box.pos, box.quat_xyzw)
    if in_box < s["in_box"]:
        s["left"] += [s["steps"]] * (s["in_box"] - in_box)      # when each mint left the box
    hands = np.array([p for side in K.gripper_points(environment) for p in side])
    near = K.box_distance(hands, box.pos, box.quat_xyzw)
    on_dish = bool(box.high[0] > DISH_XY[0] - K.DISH_IN / 2 - K.DISH_WALL and box.low[0] < DISH_XY[0] + K.DISH_IN / 2 + K.DISH_WALL
                   and box.high[1] > DISH_XY[1] - K.DISH_IN / 2 - K.DISH_WALL and box.low[1] < DISH_XY[1] + K.DISH_IN / 2 + K.DISH_WALL)
    low = float(box.low[2] - TOP)
    lid = K.lid_angle(box.quat_xyzw, st[LID].quat_xyzw)
    down = box_down(low, float(np.linalg.norm(box.lin_vel)), float(np.linalg.norm(box.ang_vel)), near, on_dish)
    s["out"] = [[round(1000 * float(v)) for v in st[m].pos - [0.0, 0.0, TOP]] for m in MINTS if not K.in_box(st[m].pos, box.pos, box.quat_xyzw)][:8]
    s.update(in_dish=in_dish, in_box=in_box, spilled=spilled, lid=lid, shut=bool(abs(lid) <= K.SHUT_AT), down=down,
             near=near, low=low)
    s["score"] = score_from(in_dish, spilled, s["shut"], down)
    s["held"] = s["held"] + 1 if s["score"] >= 1.0 else 0


# -- scoring --------------------------------------------------------------------------------------------------------
def score(objects) -> float:
    """Scored at the end of the trial, from where things are then. A mint is in the box if its centre is inside the
    box's outline, in the dish if its centre is over the dish's floor and below the top of its rim, and spilled
    otherwise. The lid is shut within 3 degrees of closed. The box is down when its lowest point is within 3 mm of
    the table, no part of it is over the dish, it is at rest (under 2 cm/s and 0.5 rad/s) and both grippers are at
    least 3 cm from it. With any mint spilled: 0.25 if exactly two are in the dish, otherwise 0. With none spilled:
    1.0 for exactly two in the dish, 0.5 for one or three, 0 for none or four or more; halved unless the lid is shut
    and the box is down."""
    return float(_STATE["score"])


def success(objects) -> bool:
    return _STATE["held"] >= HOLD


def report(objects) -> dict:
    s = _STATE
    return {"score": score(objects), "lot": LOT, "in_dish": s["in_dish"], "in_box": s["in_box"], "spilled": s["spilled"],
            "lid_deg": round(float(s["lid"]), 1), "shut": s["shut"], "down": s["down"],
            "near_mm": round(1000 * float(s["near"]), 1), "low_mm": round(1000 * float(s["low"]), 1), "left_at": list(s["left"]),
            "out_mm": list(s.get("out", []))}


def demo(objects=None, seed=None):
    return {}
