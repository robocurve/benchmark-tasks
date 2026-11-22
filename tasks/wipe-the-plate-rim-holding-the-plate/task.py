"""On the station table: a plated main course on the wet stainless-steel pass, with three sauce smudges on its rim (near
side, right side and far-right), and a folded towel pad on a small stand to the right.

Clean up the plate's rim before it goes out. Plating standard: wipe the rim clean before a plate leaves the kitchen, and
do not touch the food. Wet porcelain on a wet steel pass slides when a towel is dragged across it, so the plate has to
be held still by its rim with one arm while the other wipes. The instruction does not state how.

Scored each step from poses and the towel-plate contact force: which smudges the towel has been rubbed across, how far
the plate moved, and whether anything came near the food.
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

import kitchenkit as kk  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Clean up the plate's rim with the towel pad before the plate goes out to the guest."

PORCELAIN = (0.95, 0.95, 0.93)
SAUCE = (0.45, 0.12, 0.06)
MEAT, MASH, GREEN = (0.45, 0.22, 0.14), (0.93, 0.85, 0.60), (0.25, 0.55, 0.20)
STEEL = (0.72, 0.73, 0.76)
TOWEL_C = (0.85, 0.85, 0.92)

# -- the wet steel pass (static, slippery) ------------------------------------------------------------------------------
PASS_XY, PASS = (0.24, 0.0), (0.30, 0.42, 0.040)
PASS_TOP = TOP + PASS[2]
PASS_PART = kk.part("pass_shelf", (*PASS_XY, TOP), [kk.Geom("box", (0.0, 0.0, PASS[2] / 2), PASS, color=STEEL)],
                    mass=None, color=STEEL, friction=(0.02, 0.02), contact=(0.002, 0.0))

# -- the plate (frame: centre of its underside): base, a flat rim band, a raised lip at the edge ------------------------
PLATE, PLATE_T = 0.236, 0.010
LIP_W, LIP_H = 0.016, 0.012
BAND = 0.037  # the flat rim band inside the lip, where the smudges are (the towel pad fits inside it: a wider pad
# landed on the lip, and the lip took the press and twisted the plate in the hold)
SMUDGE_R = PLATE / 2 - LIP_W - BAND / 2  # smudge centres from the plate's centre
SMUDGES = {"far": (SMUDGE_R * 0.7071, -SMUDGE_R * 0.7071), "right": (0.0, -SMUDGE_R), "near": (-SMUDGE_R, 0.0)}  # plate
# frame (x, y): all on the half away from the holding hand (a smudge on the far side brought the wiping arm into it)
_lz = PLATE_T + LIP_H / 2
PLATE_GEOMS = [
    kk.Geom("box", (0.0, 0.0, PLATE_T / 2), (PLATE, PLATE, PLATE_T), color=PORCELAIN),
    *[kk.Geom("box", (sx * (PLATE - LIP_W) / 2, 0.0, _lz), (LIP_W, PLATE, LIP_H), color=PORCELAIN, friction=(0.9, 0.8))
      for sx in (1, -1)],
    *[kk.Geom("box", (0.0, sy * (PLATE - LIP_W) / 2, _lz), (PLATE - 2 * LIP_W, LIP_W, LIP_H), color=PORCELAIN,
              friction=(0.9, 0.8)) for sy in (1, -1)],
    *[kk.Geom("box", (x, y, PLATE_T + 0.0005), (0.030, 0.030, 0.001), kk.quat_z(0.5 * i), color=SAUCE, collide=False)
      for i, (x, y) in enumerate(SMUDGES.values())],
    kk.Geom("box", (0.015, -0.015, PLATE_T + 0.0125), (0.06, 0.045, 0.025), color=MEAT, collide=False),
    kk.Geom("box", (-0.025, 0.025, PLATE_T + 0.015), (0.045, 0.045, 0.030), color=MASH, collide=False),
    kk.Geom("box", (0.025, 0.03, PLATE_T + 0.01), (0.04, 0.035, 0.020), color=GREEN, collide=False),
]
FOOD_C, FOOD_S = (0.0, 0.0, PLATE_T + 0.02), (0.07, 0.07, 0.04)  # the food in the middle, clear of the band
PLATE_XY = (0.24, 0.0)
PLATE_POS = (*PLATE_XY, PASS_TOP + 0.0005)
_RIGID = {"solver_position_iteration_count": 16, "solver_velocity_iteration_count": 4}
PLATE_PART = kk.part("plate", PLATE_POS, PLATE_GEOMS, mass=0.45, color=PORCELAIN, friction=(0.10, 0.10), rigid=_RIGID,
                     contact=(0.002, 0.0))

# -- the towel pad on its stand (frame: centre of the pad's underside) --------------------------------------------------
TOWEL = (0.030, 0.030, 0.025)
STAND_XY, STAND_H = (0.15, -0.24), 0.030
TOWEL_POS = (*STAND_XY, TOP + STAND_H + 0.0005)
TOWEL_PART = kk.part("towel", TOWEL_POS, [kk.Geom("box", (0.0, 0.0, TOWEL[2] / 2), TOWEL, color=TOWEL_C)],
                     mass=0.03, color=TOWEL_C, friction=(0.8, 0.7), rigid=_RIGID, contact=(0.002, 0.0))  # a damp towel grips
# porcelain (at 0.3 / 0.25 it glided over the rim and an unheld plate crept only 5 mm)
STAND_PART = kk.part("stand", (*STAND_XY, TOP), [kk.Geom("box", (0.0, 0.0, STAND_H / 2), (0.05, 0.05, STAND_H),
                                                         color=STEEL)],
                     mass=None, color=STEEL, friction=(0.6, 0.5), contact=(0.002, 0.0))
OBJECTS = [PASS_PART, PLATE_PART, TOWEL_PART, STAND_PART]
kk.watch_contact("towel", "plate")  # the scorer reads the towel-on-plate contact force each step

# -- scoring -----------------------------------------------------------------------------------------------------------
PRESS = 0.8  # N: a wiping contact
OVER = 0.020  # m: the towel's centre this close (horizontally) to a smudge is over it
RUB_SPEED = 0.010  # m/s: the towel moving at least this fast relative to the plate is rubbing
RUB_STEPS = 10  # steps of rubbing over a smudge to clean it
STILL_MOVE, MAX_MOVE, MAX_YAW = 0.010, 0.030, 5.0  # m, m, deg: held still while wiped (from the first towel touch) / pushed off (from the start)
LIFT_MAX = 0.020
FOOD_NEAR = 0.010
DT = 0.05
_S: dict = {}


def _reset() -> None:
    _S.update(step=None, result=(0.0, False), last={}, rub={k: 0 for k in SMUDGES}, prev=None, max_move=0.0,
              max_yaw=0.0, food=False, food_at=None, lifted=False, wipe0=None, max_slide=0.0)


_reset()
kk.RESET_CALLBACKS.append(_reset)


def _food_by(pose) -> list:
    """Which grippers are within FOOD_NEAR of the food box on a plate at ``pose`` (a diagnostic for the report)."""
    return [a for a in kk.STATE.fingers if kk.near(a, FOOD_C, FOOD_S, pose, FOOD_NEAR)]


def _update(objects):
    if _S["step"] == kk.STATE.step and kk.STATE.step >= 0:
        return _S["result"]
    _S["step"] = kk.STATE.step
    plate = kk.pose_of("plate", PLATE_POS)
    towel = kk.pose_of("towel", TOWEL_POS)
    force = float(kk.STATE.contacts.get(("towel", "plate"), 0.0))
    rel = (towel[0] - plate[0]) @ plate[1]  # the towel's frame in the plate's frame
    speed = 0.0 if _S["prev"] is None else float(np.linalg.norm((rel - _S["prev"])[:2])) / DT
    _S["prev"] = rel
    over = None
    for k, (sx, sy) in SMUDGES.items():
        if float(np.hypot(rel[0] - sx, rel[1] - sy)) <= OVER:
            over = k
    if over is not None and force >= PRESS and speed >= RUB_SPEED:
        _S["rub"][over] += 1
    move = float(np.linalg.norm(plate[0][:2] - np.asarray(PLATE_XY)))
    yaw = float(np.degrees(np.arctan2(plate[1][1, 0], plate[1][0, 0])))
    if _S["wipe0"] is None and force > 0.0:  # the towel's first touch: from here on the plate must not slide
        _S["wipe0"] = (plate[0][:2].copy(), yaw)
    if _S["wipe0"] is not None:
        _S["max_slide"] = max(_S["max_slide"], float(np.linalg.norm(plate[0][:2] - _S["wipe0"][0])))
        _S["max_yaw"] = max(_S["max_yaw"], abs(yaw - _S["wipe0"][1]))
    _S["max_move"] = max(_S["max_move"], move)
    if plate[0][2] - PLATE_POS[2] > LIFT_MAX:
        _S["lifted"] = True
    who = _food_by(plate) + (["towel"] if float(kk.box_dist(towel[0][None] + towel[1] @ np.array([0.0, 0.0, TOWEL[2] / 2]),
                                                            FOOD_C, np.asarray(FOOD_S) + np.asarray(TOWEL), plate)[0]) <= 0.0 else [])
    if who and not _S["food"]:
        _S["food_at"] = (kk.STATE.step, who)
    if who:
        _S["food"] = True
    cleaned = [k for k, n in _S["rub"].items() if n >= RUB_STEPS]
    _S["last"] = {"force": round(force, 3), "over": over, "speed": round(speed, 4), "rub_steps": dict(_S["rub"]),
                  "cleaned": cleaned, "plate_moved": round(_S["max_move"], 4), "plate_slid": round(_S["max_slide"], 4),
                  "plate_yaw": round(_S["max_yaw"], 2),
                  "food_touched": _S["food"], "food_at": _S.get("food_at"), "lifted": _S["lifted"]}
    if _S["food"] or _S["lifted"] or _S["max_move"] > MAX_MOVE:
        res = (0.0, False)
    else:
        frac = round(len(cleaned) / len(SMUDGES), 2)
        steady = _S["max_slide"] <= STILL_MOVE and _S["max_yaw"] <= MAX_YAW
        res = (frac, frac == 1.0) if steady else (min(frac, 0.5), False)
    _S["result"] = res
    return res


def success(objects) -> bool:
    """All three smudges rubbed with the towel (contact >= 0.8 N, moving >= 1 cm/s over the smudge for 0.5 s); the
    plate kept within 1 cm and 5 deg of where it was when the towel first touched it (gripping the lip may nudge it);
    nothing near the food; the plate not lifted."""
    return _update(objects)[1]


def score(objects) -> float:
    """One third per cleaned smudge, capped at 0.5 if the plate slid more than 1 cm or turned more than 5 deg once the
    towel touched it; 0 if it ended up more than 3 cm from its start, was lifted, or the food was touched (a gripper pad within 1 cm of it, or the towel on it)."""
    return _update(objects)[0]


def report() -> dict:
    return dict(_S.get("last", {}))
