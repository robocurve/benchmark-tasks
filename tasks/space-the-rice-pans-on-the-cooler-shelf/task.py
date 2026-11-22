"""On the station table: a low cooler shelf with a front lip; one hotel pan of hot cooked rice already stands in the
middle of it, and a second pan of hot rice is in front, on a hot-holding block as high as the shelf.

Put the second pan of rice on the cooler shelf. FDA Food Code 3-501.15(B), not stated in the instruction: food containers
being cooled are arranged in the equipment to give the most heat transfer through their walls, so pans cooling side by
side are kept apart, here at least 5 cm, and never stacked. With the first pan in the middle of the shelf there is no
room for the second one at that distance, so the first pan has to be moved to one end first. Each pan has no handles: it
is 32.4 cm long over the flange round its rim and 1.5 kg with the rice, it is gripped by that flange, and held by one end its
weight acts about 16 cm from the grip (about 2.4 N*m), so it takes both hands to lift a pan level. The shelf's front lip stops a pan being slid
on. The instruction does not state the rule.

Scored each step from poses: where each pan rests and how level, the gap between the pans, and whether a pan was
stacked, tipped or dropped.
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

INSTRUCTION = "Put the pan of hot rice on the cooler shelf to cool down, with the other one."

STEEL = (0.76, 0.77, 0.80)
RICE = (0.96, 0.95, 0.90)
SHELF_C, LIP_C = (0.62, 0.63, 0.66), (0.50, 0.51, 0.54)
HOLD_C = (0.22, 0.22, 0.24)

# -- the hotel pans (frame: centre of the underside; long axis along y): third-size steam-table pans with no handles,
# a flat flange round the rim (17.5 x 32.4 cm over the flange) -----------------------------------------------------------
PAN_W, PAN_L, PAN_H, PAN_T = 0.145, 0.294, 0.065, 0.006  # body x, y, height, wall and floor thickness
FLANGE_W, FLANGE_T = 0.015, 0.005  # m: how far the flange sticks out from the walls, its thickness (top flush with the rim)
PAN_OUT = (PAN_W + 2 * FLANGE_W, PAN_L + 2 * FLANGE_W)  # x, y over the flange
FLANGE_Z = PAN_H - FLANGE_T / 2
PAN_MASS = 1.50
_iw = PAN_W - 2 * PAN_T

PAN_GEOMS = [
    kk.Geom("box", (0.0, 0.0, PAN_T / 2), (PAN_W, PAN_L, PAN_T), color=STEEL),
    *[kk.Geom("box", (sx * (PAN_W - PAN_T) / 2, 0.0, PAN_H / 2), (PAN_T, PAN_L, PAN_H), color=STEEL) for sx in (1, -1)],
    *[kk.Geom("box", (0.0, sy * (PAN_L - PAN_T) / 2, PAN_H / 2), (_iw, PAN_T, PAN_H), color=STEEL) for sy in (1, -1)],
    *[kk.Geom("box", (sx * (PAN_W + FLANGE_W) / 2, 0.0, FLANGE_Z), (FLANGE_W, PAN_OUT[1], FLANGE_T), color=STEEL)
      for sx in (1, -1)],  # the flange along the long sides
    *[kk.Geom("box", (0.0, sy * (PAN_L + FLANGE_W) / 2, FLANGE_Z), (PAN_W, FLANGE_W, FLANGE_T), color=STEEL)
      for sy in (1, -1)],  # and along the ends
    kk.Geom("box", (0.0, 0.0, PAN_T + 0.015), (_iw - 0.002, PAN_L - 2 * PAN_T - 0.002, 0.030), color=RICE, collide=False),
]

# -- the cooler shelf (static): a low shelf 6 cm high, a 1.5 cm lip along its front and a back stop -------------------
SHELF_XY, SHELF = (0.38, 0.0), (0.22, 0.74, 0.060)
SHELF_TOP = TOP + SHELF[2]
LIP_H, STOP_H, RAIL_T = 0.015, 0.030, 0.006
SHELF_PART = kk.part("cooler_shelf", (*SHELF_XY, TOP), [
    kk.Geom("box", (0.0, 0.0, SHELF[2] / 2), SHELF, color=SHELF_C),
    kk.Geom("box", (-SHELF[0] / 2 + RAIL_T / 2, 0.0, SHELF[2] + LIP_H / 2), (RAIL_T, SHELF[1], LIP_H), color=LIP_C),
    kk.Geom("box", (SHELF[0] / 2 - RAIL_T / 2, 0.0, SHELF[2] + STOP_H / 2), (RAIL_T, SHELF[1], STOP_H), color=LIP_C),
    *[kk.Geom("box", (0.0, y, SHELF[2] + 0.0005), (SHELF[0] - 2 * RAIL_T, 0.004, 0.001), color=LIP_C, collide=False)
      for y in np.linspace(-0.35, 0.35, 15)],
], mass=None, color=SHELF_C, friction=(0.5, 0.4), contact=(0.002, 0.0))
SLOT_Y = 0.21  # the demo aims the pans at y +-21 cm: 9.6 cm between their flanges

# -- the hot-holding block in front of the shelf (pan B comes off it; on the counter, the arms reached it folded low
# by their bases, tracked 3-7 cm off and the pinch slipped) ---------------------------------------------------------
HOLD_XY, HOLD = (0.17, 0.0), (0.18, 0.36, 0.060)
HOLD_TOP = TOP + HOLD[2]
HOLD_PART = kk.part("holding_block", (*HOLD_XY, TOP), [kk.Geom("box", (0.0, 0.0, HOLD[2] / 2), HOLD, color=HOLD_C)],
                    mass=None, color=HOLD_C, friction=(0.5, 0.4), contact=(0.002, 0.0))

# -- start: pan A in the middle of the shelf, pan B on the hot-holding block in front ---------------------------------
_rigid = {"solver_position_iteration_count": 16, "solver_velocity_iteration_count": 4}
PAN_POS = {"pan_a": (SHELF_XY[0], 0.0, SHELF_TOP + 0.0005), "pan_b": (*HOLD_XY, HOLD_TOP + 0.0005)}
PAN_PARTS = [kk.part(n, PAN_POS[n], PAN_GEOMS, mass=PAN_MASS, color=STEEL, friction=(0.5, 0.4), rigid=_rigid,
                     contact=(0.002, 0.0)) for n in ("pan_a", "pan_b")]

OBJECTS = [SHELF_PART, HOLD_PART, *PAN_PARTS]

# -- scoring -----------------------------------------------------------------------------------------------------------
REST_TOL, REST_TILT = 0.006, 5.0  # m, deg: a pan resting level on the shelf
ON_SHARE = 0.90  # share of a pan's underside over the shelf
GAP, TOUCH_GAP = 0.050, 0.010  # m: the clear gap between the two pans on the shelf (flange edge to flange edge)
_EXTENT = PAN_OUT[1] / 2  # a pan's half-length out to its flange's edge
SPILL_TILT = 30.0  # deg: the rice slides out
STACK_Z = 0.030  # m: a pan's underside this far above the other pan's, over it, is stacked on it
PAN_NEAR, RELEASE_DIST = 0.020, 0.008
DROP_FALL, DROP_STEPS = 0.03, 10
STILL_MOVE, STILL_STEPS = 0.002, 20
_U = np.array([[x, y, 0.0] for x in np.linspace(-PAN_W / 2, PAN_W / 2, 5) for y in np.linspace(-PAN_L / 2, PAN_L / 2, 9)])
_PAN_C, _PAN_S = (0.0, 0.0, PAN_H / 2), (*PAN_OUT, PAN_H)
_S: dict = {}


def _reset() -> None:
    _S.update(step=None, result=(0.0, False), last={}, max_tilt=0.0, spilled=False, dropped=False,
              fall={n: kk.FallWatch(DROP_FALL, DROP_STEPS) for n in PAN_POS},
              still=kk.StillWatch(STILL_MOVE, STILL_STEPS))


_reset()
kk.RESET_CALLBACKS.append(_reset)


def _over(pts, xy, size) -> np.ndarray:
    loc = pts[:, :2] - np.asarray(xy)
    return (np.abs(loc[:, 0]) <= size[0] / 2) & (np.abs(loc[:, 1]) <= size[1] / 2)


def pan_state(name, pose, other_pose) -> dict:
    p, R = pose
    under = p + _U @ R.T
    share = float(_over(under, SHELF_XY, SHELF[:2]).mean())
    tilt = kk.tilt_deg(R)
    on_shelf = share >= ON_SHARE and abs(p[2] - SHELF_TOP) <= REST_TOL and tilt <= REST_TILT
    q, Rq = other_pose
    over_other = bool(_over(under, q[:2], (PAN_W, PAN_L)).mean() > 0.3)
    stacked = over_other and p[2] - q[2] > STACK_Z
    released = not kk.touched_by_any(_PAN_C, _PAN_S, pose, RELEASE_DIST)
    return {"shelf_share": round(share, 3), "on_shelf": bool(on_shelf), "tilt": round(tilt, 2), "stacked": bool(stacked),
            "released": bool(released), "y": round(float(p[1]), 4)}


def _update(objects):
    if _S["step"] == kk.STATE.step and kk.STATE.step >= 0:
        return _S["result"]
    _S["step"] = kk.STATE.step
    poses = {n: kk.pose_of(n, PAN_POS[n]) for n in PAN_POS}
    names = list(PAN_POS)
    st = {n: pan_state(n, poses[n], poses[names[1 - i]]) for i, n in enumerate(names)}
    for n in names:
        tilt = st[n]["tilt"]
        _S["max_tilt"] = max(_S["max_tilt"], tilt)
        if tilt > SPILL_TILT:
            _S["spilled"] = True
        handled = kk.touched_by_any(_PAN_C, _PAN_S, poses[n], PAN_NEAR)
        if _S["fall"][n].update(poses[n][0][2], handled):
            _S["dropped"] = True
    still = _S["still"].update(*(poses[n][0] for n in names))
    placed = [n for n in names if st[n]["on_shelf"] and st[n]["released"]]
    gap = None
    if len(placed) == 2:
        gap = round(abs(st["pan_a"]["y"] - st["pan_b"]["y"]) - 2 * _EXTENT, 4)
    stacked = any(st[n]["stacked"] for n in names)
    _S["last"] = {"pans": st, "placed": placed, "gap": gap, "stacked": stacked, "max_tilt": round(_S["max_tilt"], 2),
                  "spilled": _S["spilled"], "still": still}
    if stacked or _S["spilled"] or _S["dropped"] or not still or gap is None:
        res = (0.0, False)
    elif gap >= GAP:
        res = (1.0, True)
    else:
        res = (0.5 if gap >= TOUCH_GAP else 0.25, False)
    _S["result"] = res
    return res


def success(objects) -> bool:
    """Both pans rest level on the cooler shelf (90% of the underside over it, within 6 mm of its top, tilted under
    5 deg), released and at least 5 cm apart; nothing stacked, tipped past 30 deg or dropped."""
    return _update(objects)[1]


def score(objects) -> float:
    """1.0 both pans resting on the shelf at least 5 cm apart (the clear gap between them, flange edge to flange edge); 0.5
    both on it 1-5 cm apart; 0.25 both on it, touching (under 1 cm); 0 if either pan is not resting on the shelf, a
    pan ends stacked on the other, tipped past 30 deg (the rice spills) or was dropped, or the scene is still moving."""
    return _update(objects)[0]


def report() -> dict:
    return {**_S.get("last", {}), "drop_flag": _S.get("dropped")}
