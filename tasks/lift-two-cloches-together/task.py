"""On the station table: two plated main courses under silver cloches, one in front of each arm, and an empty service
tray on the counter nearer the robot.

Unveil both plates and put the cloches on the tray. In plated service each guest's cloche is lifted at the same moment
(a synchronised reveal: one server per guest, a single smooth lift) and straight up, so the rim does not scrape across
the plate. One gripper can hold one cloche by its knob, so lifting both together takes both arms. The instruction does
not state the timing or the straight-up lift.

Scored each step from poses: when each cloche lifts off, how high it is before it moves sideways, where it ends.
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

INSTRUCTION = "Unveil the two guests' plates and put the cloches on the service tray."

SILVER = (0.78, 0.79, 0.82)
PORCELAIN = (0.95, 0.95, 0.93)
MAT = (0.35, 0.18, 0.12)
TRAY_C = (0.55, 0.56, 0.58)
MEAT, GREEN, MASH = (0.45, 0.22, 0.14), (0.25, 0.55, 0.20), (0.93, 0.85, 0.60)

# -- the two plates (static, glued to the table) with food, under the cloches ------------------------------------------
PLATE, PLATE_H = 0.200, 0.012
PLATE_XY = {"left": (0.34, 0.16), "right": (0.34, -0.16)}
CL_TOP_Z = TOP + PLATE_H  # the plate's top: where a cloche's rim stands


def _plate(name, xy):
    return kk.part(name, (*xy, TOP), [
        kk.Geom("box", (0.0, 0.0, -0.001), (0.26, 0.26, 0.002), color=MAT, collide=False),  # placemat
        kk.Geom("box", (0.0, 0.0, PLATE_H / 2), (PLATE, PLATE, PLATE_H), color=PORCELAIN),
        kk.Geom("box", (0.03, 0.0, PLATE_H + 0.0125), (0.06, 0.08, 0.025), color=MEAT, collide=False),
        kk.Geom("box", (-0.035, 0.03, PLATE_H + 0.015), (0.05, 0.05, 0.030), color=MASH, collide=False),
        kk.Geom("box", (-0.035, -0.035, PLATE_H + 0.01), (0.05, 0.04, 0.020), color=GREEN, collide=False),
    ], mass=None, color=PORCELAIN, friction=(0.5, 0.4), contact=(0.002, 0.0))


PLATES = [_plate("plate_" + s, xy) for s, xy in PLATE_XY.items()]

# -- the cloches (frame: centre of the rim's underside) -----------------------------------------------------------------
CL, CL_H, CL_WALL = 0.180, 0.075, 0.004  # outer footprint, height, wall and top thickness
KNOB, KNOB_H = 0.028, 0.025
CL_GEOMS = [
    kk.Geom("box", (0.0, 0.0, CL_H - CL_WALL / 2), (CL, CL, CL_WALL), color=SILVER),
    *[kk.Geom("box", (sx * (CL - CL_WALL) / 2, 0.0, (CL_H - CL_WALL) / 2), (CL_WALL, CL, CL_H - CL_WALL), color=SILVER)
      for sx in (1, -1)],
    *[kk.Geom("box", (0.0, sy * (CL - CL_WALL) / 2, (CL_H - CL_WALL) / 2), (CL - 2 * CL_WALL, CL_WALL, CL_H - CL_WALL),
              color=SILVER) for sy in (1, -1)],
    kk.Geom("box", (0.0, 0.0, CL_H + KNOB_H / 2), (KNOB, KNOB, KNOB_H), color=SILVER),
]
CL_POS = {s: (*xy, CL_TOP_Z + 0.0005) for s, xy in PLATE_XY.items()}
CLOCHES = {s: kk.part("cloche_" + s, CL_POS[s], CL_GEOMS, mass=0.25, color=SILVER, friction=(0.5, 0.4),
                      rigid={"solver_position_iteration_count": 16, "solver_velocity_iteration_count": 4},
                      contact=(0.002, 0.0)) for s in PLATE_XY}
KNOB_Z = CL_H + KNOB_H / 2  # the knob's centre above the cloche's frame

# -- the service tray (static) ------------------------------------------------------------------------------------------
TRAY_XY, TRAY, TRAY_H = (0.12, 0.0), (0.22, 0.42), 0.012
TRAY_SPOTS = {"left": (0.12, 0.10), "right": (0.12, -0.10)}  # where the demo sets the cloches down
TRAY_PART = kk.part("tray", (*TRAY_XY, TOP), [kk.Geom("box", (0.0, 0.0, TRAY_H / 2), (*TRAY, TRAY_H), color=TRAY_C)],
                    mass=None, color=TRAY_C, friction=(0.5, 0.4), contact=(0.002, 0.0))
OBJECTS = [*PLATES, *CLOCHES.values(), TRAY_PART]

# -- scoring -----------------------------------------------------------------------------------------------------------
LIFT_OFF = 0.010  # m: a cloche whose rim has risen this far has been lifted off
SYNC_STEPS, LATE_STEPS, SLOW_STEPS = 5, 20, 60  # 20 Hz steps between the two lift-offs: 0.25 s, 1 s, 3 s
SIDE_MOVE, STRAIGHT_UP = 0.030, 0.050  # m: before a cloche's knob moves 3 cm sideways it must have risen 5 cm
TRAY_TOL, TRAY_OVER, TRAY_TILT = 0.006, 0.010, 10.0  # rim on the tray, at most 1 cm overhang, nearly upright
NEAR, RELEASE_DIST = 0.020, 0.008
DROP_FALL, DROP_STEPS = 0.05, 10
STILL_MOVE, STILL_STEPS = 0.002, 20
_C = (0.0, 0.0, (CL_H + KNOB_H) / 2)
_S_BOX = (CL, CL, CL_H + KNOB_H)
_S: dict = {}


def _reset() -> None:
    _S.update(step=None, result=(0.0, False), last={}, lift={s: None for s in PLATE_XY},
              dragged={s: False for s in PLATE_XY}, judged={s: False for s in PLATE_XY},
              dropped={s: False for s in PLATE_XY}, fall={s: kk.FallWatch(DROP_FALL, DROP_STEPS) for s in PLATE_XY},
              still=kk.StillWatch(STILL_MOVE, STILL_STEPS))


_reset()
kk.RESET_CALLBACKS.append(_reset)


def on_tray(pose) -> bool:
    p, R = pose
    if abs(p[2] - (TOP + TRAY_H)) > TRAY_TOL or kk.tilt_deg(R) > TRAY_TILT:
        return False
    corners = np.array([[sx * CL / 2, sy * CL / 2, 0.0] for sx in (1, -1) for sy in (1, -1)]) @ R.T + p
    loc = corners[:, :2] - np.asarray(TRAY_XY)
    return bool(np.all(np.abs(loc[:, 0]) <= TRAY[0] / 2 + TRAY_OVER) and np.all(np.abs(loc[:, 1]) <= TRAY[1] / 2 + TRAY_OVER))


def _update(objects):
    if _S["step"] == kk.STATE.step and kk.STATE.step >= 0:
        return _S["result"]
    _S["step"] = kk.STATE.step
    k = max(kk.STATE.step, 0)
    poses, info = {}, {}
    for s in PLATE_XY:
        pose = kk.pose_of("cloche_" + s, CL_POS[s])
        poses[s] = pose
        knob = pose[0] + pose[1] @ np.array([0.0, 0.0, KNOB_Z])  # the knob is what the gripper moves
        rise = float(knob[2] - (CL_POS[s][2] + KNOB_Z))
        side = float(np.linalg.norm(knob[:2] - np.asarray(PLATE_XY[s])))
        if _S["lift"][s] is None and rise > LIFT_OFF:
            _S["lift"][s] = k
        if not _S["judged"][s] and side > SIDE_MOVE:  # the first time it has moved 3 cm sideways: high enough?
            _S["judged"][s] = True
            _S["dragged"][s] = rise < STRAIGHT_UP
        handled = kk.touched_by_any(_C, _S_BOX, pose, NEAR)
        if _S["fall"][s].update(pose[0][2], handled):
            _S["dropped"][s] = True
        info[s] = {"rise": round(rise, 4), "side": round(side, 4), "on_tray": on_tray(pose),
                   "released": not kk.touched_by_any(_C, _S_BOX, pose, RELEASE_DIST)}
    still = _S["still"].update(*(poses[s][0] for s in PLATE_XY))
    lifts = list(_S["lift"].values())
    gap = abs(lifts[0] - lifts[1]) if None not in lifts else None
    placed = all(info[s]["on_tray"] and info[s]["released"] for s in PLATE_XY) and still
    _S["last"] = {"lift_steps": dict(_S["lift"]), "lift_gap_steps": gap, "dragged": dict(_S["dragged"]),
                  "dropped": dict(_S["dropped"]), "both_on_tray": placed, **{f"cloche_{s}": v for s, v in info.items()}}
    if any(_S["dropped"].values()) or not placed or gap is None:
        res = (0.0, False)
    elif any(_S["dragged"].values()):
        res = (0.25 if gap <= SLOW_STEPS else 0.0, False)
    elif gap <= SYNC_STEPS:
        res = (1.0, True)
    elif gap <= LATE_STEPS:
        res = (0.5, False)
    elif gap <= SLOW_STEPS:
        res = (0.25, False)
    else:
        res = (0.0, False)
    _S["result"] = res
    return res


def success(objects) -> bool:
    """Both cloches lifted off within 0.25 s of each other, each straight up (knob 5 cm up before it moves 3 cm sideways),
    both resting on the tray, released and still, neither dropped."""
    return _update(objects)[1]


def score(objects) -> float:
    """With both cloches ending on the tray, released: 1.0 lift-offs within 0.25 s, 0.5 within 1 s, 0.25 within 3 s
    or a cloche's knob moved 3 cm sideways before it had risen 5 cm; 0 if more than 3 s apart, a cloche is dropped or not on the
    tray at the end."""
    return _update(objects)[0]


def report() -> dict:
    return dict(_S.get("last", {}))
