"""On the station table: two plated main courses on the pass nearer the robot, each with its steak pointing toward the
middle, and two place settings (placemats) further out where two guests are seated.

Serve the two main courses. In plated service every plate at a table is set down at the same moment, and each plate
goes down with its focal point, the protein, toward the guest (the 6 o'clock position), not turned round on the table
afterwards. One gripper carries one plate, so setting both down together takes both arms, and each plate has to turn a
quarter turn on the way. The instruction does not state either rule.

Scored each step from poses: when each plate touches the table, where its steak points, whether it is turned on the
table, and where it ends.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

import kitchenkit as kk  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Serve the two main courses to the two guests."

PORCELAIN = (0.95, 0.95, 0.93)
MEAT, MASH, GREEN = (0.45, 0.22, 0.14), (0.93, 0.85, 0.60), (0.25, 0.55, 0.20)
PASS_C, MAT = (0.70, 0.71, 0.73), (0.30, 0.16, 0.10)

# -- the pass (static), and the two place settings on the table (visual placemats) -------------------------------------
PASS_XY, PASS = (0.14, 0.0), (0.20, 0.50, 0.050)
PASS_TOP = TOP + PASS[2]
COVER_XY = {"left": (0.36, 0.15), "right": (0.36, -0.15)}
PASS_PART = kk.part("pass_shelf", (*PASS_XY, TOP), [
    kk.Geom("box", (0.0, 0.0, PASS[2] / 2), PASS, color=PASS_C),
    *[kk.Geom("box", (cx - PASS_XY[0], cy - PASS_XY[1], 0.0005), (0.26, 0.26, 0.001), color=MAT, collide=False)
      for cx, cy in COVER_XY.values()],
], mass=None, color=PASS_C, friction=(0.5, 0.4), contact=(0.002, 0.0))

# -- the plates (frame: centre of the base's underside); the steak is fixed 4.5 cm off-centre along -y (local) ----------
PLATE, PLATE_T = 0.200, 0.010
LIP_W, LIP_H = 0.012, 0.015  # a raised rim lip round the edge (the gripper pinches it)
PROTEIN_DIR = np.array([0.0, -1.0, 0.0])  # the steak's side, in the plate's frame
_lip_z = PLATE_T + LIP_H / 2
PLATE_GEOMS = [
    kk.Geom("box", (0.0, 0.0, PLATE_T / 2), (PLATE, PLATE, PLATE_T), color=PORCELAIN),
    *[kk.Geom("box", (sx * (PLATE - LIP_W) / 2, 0.0, _lip_z), (LIP_W, PLATE, LIP_H), color=PORCELAIN) for sx in (1, -1)],
    *[kk.Geom("box", (0.0, sy * (PLATE - LIP_W) / 2, _lip_z), (PLATE - 2 * LIP_W, LIP_W, LIP_H), color=PORCELAIN)
      for sy in (1, -1)],
    kk.Geom("box", (0.0, -0.045, PLATE_T + 0.0125), (0.075, 0.050, 0.025), color=MEAT, collide=False),
    kk.Geom("box", (-0.03, 0.04, PLATE_T + 0.015), (0.05, 0.045, 0.030), color=MASH, collide=False),
    kk.Geom("box", (0.035, 0.04, PLATE_T + 0.01), (0.045, 0.04, 0.020), color=GREEN, collide=False),
]
FOOD_C, FOOD_S = (0.0, -0.0035, PLATE_T + 0.02), (0.12, 0.133, 0.04)  # where the food is (no gripper pad near it)
START_XY = {"left": (0.14, 0.13), "right": (0.14, -0.13)}
START_YAW = {"left": 0.0, "right": math.pi}  # both steaks point toward the middle
PLATE_POS = {s: (*xy, PASS_TOP + 0.0005) for s, xy in START_XY.items()}
_RIGID = {"solver_position_iteration_count": 16, "solver_velocity_iteration_count": 4}
PLATES = {s: kk.part("plate_" + s, PLATE_POS[s], PLATE_GEOMS, mass=0.30, color=PORCELAIN, yaw=START_YAW[s],
                     friction=(0.6, 0.5), rigid=_RIGID, contact=(0.002, 0.0)) for s in START_XY}
OBJECTS = [PASS_PART, *PLATES.values()]

# -- scoring -----------------------------------------------------------------------------------------------------------
LIFTED = 0.010  # m: a plate this far above the pass has left it
DOWN = 0.003  # m: a plate's underside this close to the table has been set down
SYNC_STEPS, SLOW_STEPS = 10, 60  # 0.5 s, 3 s between the two set-downs
COVER_TOL, FACE_TOL = 0.030, 25.0  # m, deg: centred on the place setting, steak toward the guest (+x)
TURN_ON_TABLE = 10.0  # deg: a plate turned this much while on the table was turned in front of the guest
CARRY_TILT, MAX_TILT = 15.0, 30.0  # deg
FOOD_NEAR, PLATE_NEAR, RELEASE_DIST = 0.008, 0.020, 0.008
DROP_FALL, DROP_STEPS = 0.03, 10
STILL_MOVE, STILL_STEPS = 0.002, 20
_PLATE_C, _PLATE_S = (0.0, 0.0, (PLATE_T + LIP_H) / 2), (PLATE, PLATE, PLATE_T + LIP_H)
_S: dict = {}


def _reset() -> None:
    _S.update(step=None, result=(0.0, False), last={}, lifted={s: False for s in START_XY},
              down={s: None for s in START_XY}, yaw_down={s: None for s in START_XY},
              turned={s: False for s in START_XY}, max_tilt=0.0, food=False, food_at=None,
              dropped={s: False for s in START_XY}, fall={s: kk.FallWatch(DROP_FALL, DROP_STEPS) for s in START_XY},
              still=kk.StillWatch(STILL_MOVE, STILL_STEPS))


_reset()
kk.RESET_CALLBACKS.append(_reset)


def facing_deg(R) -> float:
    """Angle between the steak's direction (horizontal) and the direction to the guest (+x), deg."""
    d = R @ PROTEIN_DIR
    return abs(math.degrees(math.atan2(d[1], d[0])))


def _yaw(R) -> float:
    return math.atan2(R[1, 0], R[0, 0])


def _food_by(pose) -> list:
    """Which grippers are within FOOD_NEAR of the food box on a plate at ``pose`` (a diagnostic for the report)."""
    return [a for a in kk.STATE.fingers if kk.near(a, FOOD_C, FOOD_S, pose, FOOD_NEAR)]


def _update(objects):
    if _S["step"] == kk.STATE.step and kk.STATE.step >= 0:
        return _S["result"]
    _S["step"] = kk.STATE.step
    k = max(kk.STATE.step, 0)
    info, poses = {}, {}
    for s in START_XY:
        pose = kk.pose_of("plate_" + s, PLATE_POS[s])
        poses[s] = pose
        p, R = pose
        if p[2] - PLATE_POS[s][2] > LIFTED:
            _S["lifted"][s] = True
        tilt = kk.tilt_deg(R)
        if _S["lifted"][s] and _S["down"][s] is None:
            _S["max_tilt"] = max(_S["max_tilt"], tilt)
        on_table = p[2] - TOP <= DOWN
        if _S["lifted"][s] and on_table and _S["down"][s] is None:
            _S["down"][s], _S["yaw_down"][s] = k, _yaw(R)
        if _S["down"][s] is not None and on_table:
            dyaw = abs(math.degrees((_yaw(R) - _S["yaw_down"][s] + math.pi) % (2 * math.pi) - math.pi))
            if dyaw > TURN_ON_TABLE:
                _S["turned"][s] = True
        who = _food_by(pose)
        if who and not _S["food"]:
            _S["food_at"] = (kk.STATE.step, s, who)
        if who:
            _S["food"] = True
        handled = kk.touched_by_any(_PLATE_C, _PLATE_S, pose, PLATE_NEAR)
        if _S["fall"][s].update(p[2], handled):
            _S["dropped"][s] = True
        off = float(np.linalg.norm(p[:2] - np.asarray(COVER_XY[s])))
        info[s] = {"cover_off": round(off, 4), "facing_deg": round(facing_deg(R), 1), "on_table": bool(on_table),
                   "released": not kk.touched_by_any(_PLATE_C, _PLATE_S, pose, RELEASE_DIST), "tilt": round(tilt, 2)}
    still = _S["still"].update(*(poses[s][0] for s in START_XY))
    downs = list(_S["down"].values())
    gap = abs(downs[0] - downs[1]) if None not in downs else None
    placed = all(i["on_table"] and i["released"] and i["cover_off"] <= COVER_TOL for i in info.values()) and still
    facing = all(i["facing_deg"] <= FACE_TOL for i in info.values())
    turned = any(_S["turned"].values())
    _S["last"] = {"down_steps": dict(_S["down"]), "gap_steps": gap, "placed": placed, "facing": facing,
                  "turned_on_table": dict(_S["turned"]), "max_carry_tilt": round(_S["max_tilt"], 2),
                  "food_touched": _S["food"], "food_at": _S.get("food_at"), **{f"plate_{s}": v for s, v in info.items()}}
    if any(_S["dropped"].values()) or _S["food"] or _S["max_tilt"] > MAX_TILT or not placed or gap is None:
        res = (0.0, False)
    elif gap > SLOW_STEPS:
        res = (0.0, False)
    elif not facing or turned or _S["max_tilt"] > CARRY_TILT:
        res = (0.25, False)
    elif gap <= SYNC_STEPS:
        res = (1.0, True)
    else:
        res = (0.5, False)
    _S["result"] = res
    return res


def success(objects) -> bool:
    """Both plates set down within 0.5 s of each other, each centred (3 cm) on its place setting with its steak
    within 25 deg of toward the guest, not turned on the table, carried within 15 deg of level, released and still;
    no gripper pad near the food, no plate dropped."""
    return _update(objects)[1]


def score(objects) -> float:
    """1.0 both plates down together (0.5 s) and facing the guests; 0.5 the same but 0.5-3 s apart; 0.25 down within
    3 s but a steak not toward its guest, a plate turned on the table, or carried tilted 15-30 deg; 0 if more than 3 s
    apart, a plate not on its place setting, dropped, tilted past 30 deg, or the food touched."""
    return _update(objects)[0]


def report() -> dict:
    return dict(_S.get("last", {}))
