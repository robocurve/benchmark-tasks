"""On the station table: a teapot of tea on a trivet, its loose lid resting on top, and an empty cup beside it.

Pour a cup of tea. Tea-service practice: hold the handle with one hand and keep the lid on with the other while
pouring, or it slides off as the pot tilts and falls into the cup or onto the table. This lid has no locking tab; it slides off once the pot
tilts past about 25 deg, and pouring needs about 45 deg, so one arm tilts the pot by its handle while the other keeps
the lid on. The pot is a side-handle pot (like a Japanese kyusu): a straight handle at a right angle to the spout. The
instruction does not state the rule.

Liquid is not simulated: the pour is scored from the pose of the spout over the cup.

Scored each step from poses: how far the pot is tilted and where its spout is, whether the lid stays seated on the pot
while it is tilted (and at the end), and where the pot ends.
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

INSTRUCTION = "Pour a cup of tea from the teapot into the cup, then put the teapot back on its trivet."

POT_C = (0.90, 0.90, 0.86)
BLUE = (0.20, 0.35, 0.65)
TEA = (0.55, 0.30, 0.10)
CUP_C = (0.95, 0.95, 0.95)
TRIVET_C = (0.45, 0.30, 0.18)

# -- the teapot (frame: centre of its underside): an open box body, a stepped spout toward -y, a side handle at -x -----
POT, POT_H, POT_T = 0.100, 0.080, 0.006
_pin = POT - 2 * POT_T
SPOUT_TIP = np.array([0.0, -0.0995, 0.075])  # in the pot's frame
HANDLE_XYZ, HANDLE = (-0.080, 0.0, 0.060), (0.060, 0.020, 0.016)  # a straight side handle sticking out 6 cm toward the
# robot, at a right angle to the spout: pinched top and bottom along its length, the pads hold the pot's hanging torque,
# and pouring turns the pot about the handle's own axis (a handle opposite the spout, pinched across, let the pot turn
# in the pinch or put the hand in an awkward pose)
POT_GEOMS = [
    kk.Geom("box", (0.0, 0.0, POT_T / 2), (POT, POT, POT_T), color=POT_C),
    *[kk.Geom("box", (sx * (POT - POT_T) / 2, 0.0, POT_H / 2), (POT_T, POT, POT_H), color=POT_C) for sx in (1, -1)],
    *[kk.Geom("box", (0.0, sy * (POT - POT_T) / 2, POT_H / 2), (_pin, POT_T, POT_H), color=POT_C) for sy in (1, -1)],
    *[kk.Geom("box", (0.0, y, z), (0.018, 0.024, 0.020), color=POT_C)
      for y, z in ((-0.058, 0.035), (-0.073, 0.050), (-0.088, 0.065))],  # spout, stepped up and out
    kk.Geom("box", HANDLE_XYZ, HANDLE, color=BLUE),
    kk.Geom("box", (0.0, 0.0, POT_T + 0.025), (_pin - 0.002, _pin - 0.002, 0.050), color=TEA, collide=False),
]

# -- its loose lid (frame: centre of the cap's underside): a flat cap and a knob ----------------------------------------
CAP, CAP_T, KNOB, KNOB_H = 0.098, 0.006, 0.022, 0.018
KNOB_Z = CAP_T + KNOB_H / 2  # the knob's centre above the lid's frame
LID_GEOMS = [kk.Geom("box", (0.0, 0.0, CAP_T / 2), (CAP, CAP, CAP_T), color=POT_C),
             kk.Geom("box", (0.0, 0.0, KNOB_Z), (KNOB, KNOB, KNOB_H), color=BLUE)]

TRIVET_XY, TRIVET, TRIVET_H = (0.30, 0.07), 0.140, 0.010
POT_POS = (*TRIVET_XY, TOP + TRIVET_H + 0.0005)
LID_POS = (*TRIVET_XY, POT_POS[2] + POT_H + 0.0005)
_RIGID = {"solver_position_iteration_count": 16, "solver_velocity_iteration_count": 4}
TEAPOT = kk.part("teapot", POT_POS, POT_GEOMS, mass=0.30, color=POT_C, friction=(0.40, 0.35), rigid=_RIGID,
                 contact=(0.002, 0.0))
LID = kk.part("lid", LID_POS, LID_GEOMS, mass=0.05, color=POT_C, friction=(0.40, 0.35), rigid=_RIGID,
              contact=(0.002, 0.0))
TRIVET_PART = kk.part("trivet", (*TRIVET_XY, TOP), [kk.Geom("box", (0.0, 0.0, TRIVET_H / 2), (TRIVET, TRIVET, TRIVET_H),
                                                            color=TRIVET_C)],
                      mass=None, color=TRIVET_C, friction=(0.6, 0.5), contact=(0.002, 0.0))

# -- the cup (static) ---------------------------------------------------------------------------------------------------
CUP_XY, CUP, CUP_H, CUP_T = (0.30, -0.058), 0.070, 0.045, 0.004
_cin = CUP - 2 * CUP_T
CUP_PART = kk.part("cup", (*CUP_XY, TOP), [
    kk.Geom("box", (0.0, 0.0, CUP_T / 2), (CUP, CUP, CUP_T), color=CUP_C),
    *[kk.Geom("box", (sx * (CUP - CUP_T) / 2, 0.0, CUP_H / 2), (CUP_T, CUP, CUP_H), color=CUP_C) for sx in (1, -1)],
    *[kk.Geom("box", (0.0, sy * (CUP - CUP_T) / 2, CUP_H / 2), (_cin, CUP_T, CUP_H), color=CUP_C) for sy in (1, -1)],
], mass=None, color=CUP_C, friction=(0.5, 0.4), contact=(0.002, 0.0))
OBJECTS = [TRIVET_PART, TEAPOT, LID, CUP_PART]

# -- scoring -----------------------------------------------------------------------------------------------------------
POUR_TILT = 45.0  # deg: the pot tilted at least this far is pouring
OVER_CUP, ABOVE_MIN, ABOVE_MAX = 0.025, 0.010, 0.080  # spout within 2.5 cm of the cup's axis, 1-8 cm above its rim
SPILL_STEPS = 10  # pouring with the spout not over the cup this long is pouring onto the table
FULL_STEPS, MIN_STEPS = 60, 20  # 3 s, 1 s of pouring
LID_SHIFT, LID_LIFT, LID_TILT = 0.008, 0.005, 8.0  # the lid still seated, relative to the pot
LID_CHECK_TILT = 10.0  # deg: the lid must stay seated whenever the pot is tilted this far (a hand closing on the knob
# of the upright pot nudged it 8 mm for a moment, and it settled back); it must also be seated at the end
HOME_TOL, HOME_OFF, HOME_TILT = 0.006, 0.030, 10.0
POT_NEAR, RELEASE_DIST = 0.020, 0.008
DROP_FALL, DROP_STEPS = 0.05, 10
STILL_MOVE, STILL_STEPS = 0.002, 20
_POT_C, _POT_S = (0.0, 0.0, POT_H / 2), (POT, 0.23, POT_H)
_HANDLE_C, _HANDLE_S = HANDLE_XYZ, HANDLE  # the side handle (left out of the body box, a hand on it read as let go)
_SEAT = np.array([0.0, 0.0, POT_H])
_S: dict = {}


def _reset() -> None:
    _S.update(step=None, result=(0.0, False), last={}, pour=0, astray=0, spilled=False, lid_lost=False,
              dropped=False, fall=kk.FallWatch(DROP_FALL, DROP_STEPS), still=kk.StillWatch(STILL_MOVE, STILL_STEPS))


_reset()
kk.RESET_CALLBACKS.append(_reset)


def lid_state(pot, lid) -> dict:
    rel = (lid[0] - pot[0]) @ pot[1] - _SEAT  # the lid's frame in the pot's frame, from its seat
    rel_tilt = kk.tilt_deg(pot[1].T @ lid[1])
    ok = float(np.linalg.norm(rel[:2])) <= LID_SHIFT and abs(rel[2]) <= LID_LIFT and rel_tilt <= LID_TILT
    return {"lid_shift": round(float(np.linalg.norm(rel[:2])), 4), "lid_lift": round(float(rel[2]), 4),
            "lid_tilt": round(rel_tilt, 2), "lid_ok": bool(ok)}


def _update(objects):
    if _S["step"] == kk.STATE.step and kk.STATE.step >= 0:
        return _S["result"]
    _S["step"] = kk.STATE.step
    pot = kk.pose_of("teapot", POT_POS)
    lid = kk.pose_of("lid", LID_POS)
    ls = lid_state(pot, lid)
    tilt = kk.tilt_deg(pot[1])
    if not ls["lid_ok"] and tilt >= LID_CHECK_TILT:
        _S["lid_lost"] = True
    tip = pot[0] + pot[1] @ SPOUT_TIP
    off = float(np.linalg.norm(tip[:2] - np.asarray(CUP_XY)))
    above = float(tip[2] - (TOP + CUP_H))
    over = off <= OVER_CUP and ABOVE_MIN <= above <= ABOVE_MAX
    if tilt >= POUR_TILT:
        if over:
            _S["pour"] += 1
            _S["astray"] = 0
        else:
            _S["astray"] += 1
            if _S["astray"] >= SPILL_STEPS:
                _S["spilled"] = True
    else:
        _S["astray"] = 0
    handled = kk.touched_by_any(_POT_C, _POT_S, pot, POT_NEAR) or kk.touched_by_any(_HANDLE_C, _HANDLE_S, pot, POT_NEAR)
    if _S["fall"].update(pot[0][2], handled):
        _S["dropped"] = True
    released = not (kk.touched_by_any(_POT_C, _POT_S, pot, RELEASE_DIST)
                    or kk.touched_by_any(_HANDLE_C, _HANDLE_S, pot, RELEASE_DIST))
    still = _S["still"].update(pot[0])
    home = (abs(pot[0][2] - (TOP + TRIVET_H)) <= HOME_TOL and tilt <= HOME_TILT
            and float(np.linalg.norm(pot[0][:2] - np.asarray(TRIVET_XY))) <= HOME_OFF)
    _S["last"] = {**ls, "tilt": round(tilt, 2), "spout_off": round(off, 4), "spout_above": round(above, 4),
                  "over_cup": over, "pour_steps": _S["pour"], "spilled": _S["spilled"], "home": bool(home),
                  "released": released, "still": still}
    if _S["lid_lost"] or _S["spilled"] or _S["dropped"]:
        res = (0.0, False)
    elif _S["pour"] >= FULL_STEPS:
        res = (1.0, True) if home and released and still and ls["lid_ok"] else (0.5, False)
    elif _S["pour"] >= MIN_STEPS:
        res = (0.25, False)
    else:
        res = (0.0, False)
    _S["result"] = res
    return res


def success(objects) -> bool:
    """At least 3 s pouring (pot tilted >= 45 deg, spout within 2.5 cm of the cup's axis and 1-8 cm above its rim),
    the lid seated on the pot whenever it was tilted 10 deg or more and at the end, the pot back upright on its trivet,
    released (no finger pad within 8 mm of its body, spout or handle) and still."""
    return _update(objects)[1]


def score(objects) -> float:
    """1.0 a 3 s pour with the lid kept on and the pot put back on the trivet and let go; 0.5 the same pour, but the pot
    not back, not let go or not still (or the lid not seated at the end); 0.25 a 1-3 s pour with the lid kept on; 0 if
    the lid came off its seat while the pot was tilted 10 deg or more, the pot was tilted to pour with the spout away
    from the cup for 0.5 s, the pot was dropped, or there was no pour."""
    return _update(objects)[0]


def report() -> dict:
    return {**_S.get("last", {}), "lid_lost": _S.get("lid_lost"), "drop_flag": _S.get("dropped")}
