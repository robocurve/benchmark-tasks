"""On the station table: a serving-line block with two wells, the left one a steam well (hot holding) and the right one
an ice well (cold holding), and a hotel pan of hot chili on the counter in front of the block, between the two wells.

Set up the chili on the serving line. FDA Food Code 3-501.16, not stated in the instruction: hot food is held hot (57 C
/ 135 F or above), cold food cold (5 C / 41 F or below). So the chili goes into the steam well, not the ice well. The pan
has no handles: 32.4 cm long and 1.5 kg with the chili, it is gripped by the flange round its rim; held by one end its
weight acts about 16 cm from the grip (about 2.4 N*m), so it takes both hands to lift it level over the block and lower it
into a well. Heat,
steam and ice are not simulated: the wells are told apart by their colour (red rim, steam; blue rim, ice) and the scorer
checks which well the pan ends in.

Scored each step from poses: where the pan rests, how level, and whether it was tipped or dropped.
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

INSTRUCTION = "Set up the chili on the serving line."

STEEL = (0.76, 0.77, 0.80)
CHILI, SALAD = (0.55, 0.16, 0.08), (0.93, 0.88, 0.62)
BLOCK_C, HOT_C, COLD_C = (0.55, 0.56, 0.60), (0.80, 0.15, 0.10), (0.15, 0.35, 0.80)
STEAM, ICE = (0.92, 0.92, 0.95), (0.85, 0.93, 0.98)

# -- the hotel pan (frame: centre of the underside; long axis along y): a third-size steam-table pan with no handles,
# a flat flange round its rim (17.5 x 32.4 cm over the flange) -----------------------------------------------------------
PAN_W, PAN_L, PAN_H, PAN_T = 0.145, 0.294, 0.100, 0.006  # body x, y, height (a 4 in pan), wall and floor thickness
FLANGE_W, FLANGE_T = 0.015, 0.005  # m: how far the flange sticks out from the walls, its thickness (top flush with the rim)
PAN_OUT = (PAN_W + 2 * FLANGE_W, PAN_L + 2 * FLANGE_W)  # x, y over the flange
FLANGE_Z = PAN_H - FLANGE_T / 2
PAN_MASS = 1.50
_iw = PAN_W - 2 * PAN_T


def _pan_geoms(food):
    return [
        kk.Geom("box", (0.0, 0.0, PAN_T / 2), (PAN_W, PAN_L, PAN_T), color=STEEL),
        *[kk.Geom("box", (sx * (PAN_W - PAN_T) / 2, 0.0, PAN_H / 2), (PAN_T, PAN_L, PAN_H), color=STEEL) for sx in (1, -1)],
        *[kk.Geom("box", (0.0, sy * (PAN_L - PAN_T) / 2, PAN_H / 2), (_iw, PAN_T, PAN_H), color=STEEL) for sy in (1, -1)],
        *[kk.Geom("box", (sx * (PAN_W + FLANGE_W) / 2, 0.0, FLANGE_Z), (FLANGE_W, PAN_OUT[1], FLANGE_T), color=STEEL)
          for sx in (1, -1)],  # the flange along the long sides
        *[kk.Geom("box", (0.0, sy * (PAN_L + FLANGE_W) / 2, FLANGE_Z), (PAN_W, FLANGE_W, FLANGE_T), color=STEEL)
          for sy in (1, -1)],  # and along the ends
        kk.Geom("box", (0.0, 0.0, PAN_T + 0.022), (_iw - 0.002, PAN_L - 2 * PAN_T - 0.002, 0.044), color=food,
                collide=False),
    ]


# -- the serving line (static): a 6 cm block with two wells 5 cm deep, 1.25-3.1 cm wider than the pan on every side ---
BLOCK_XY, BLOCK = (0.38, 0.0), (0.24, 0.84, 0.060)
BLOCK_TOP = TOP + BLOCK[2]
WELL_Y = {"hot": 0.21, "cold": -0.21}
WELL = (0.200, 0.386, 0.050)  # opening x, y, depth: 1.25 cm (x) and 3.1 cm (y) wider than the pan on each side
WELL_FLOOR = BLOCK_TOP - WELL[2]
_bx0, _bx1 = BLOCK_XY[0] - BLOCK[0] / 2, BLOCK_XY[0] + BLOCK[0] / 2
_wx0, _wx1 = BLOCK_XY[0] - WELL[0] / 2, BLOCK_XY[0] + WELL[0] / 2
_ys = sorted([-BLOCK[1] / 2, BLOCK[1] / 2] + [y + s * WELL[1] / 2 for y in WELL_Y.values() for s in (-1, 1)])
_upper = WELL[2]


def _block_geoms():
    g = [kk.Geom("box", (0.0, 0.0, (BLOCK[2] - _upper) / 2), (BLOCK[0], BLOCK[1], BLOCK[2] - _upper), color=BLOCK_C)]
    zc = BLOCK[2] - _upper / 2
    g.append(kk.Geom("box", ((_bx0 + _wx0) / 2 - BLOCK_XY[0], 0.0, zc), (_wx0 - _bx0, BLOCK[1], _upper), color=BLOCK_C))
    g.append(kk.Geom("box", ((_wx1 + _bx1) / 2 - BLOCK_XY[0], 0.0, zc), (_bx1 - _wx1, BLOCK[1], _upper), color=BLOCK_C))
    for a, b in zip(_ys[0::2], _ys[1::2]):  # the strips beside and between the wells
        g.append(kk.Geom("box", (0.0, (a + b) / 2, zc), (WELL[0], b - a, _upper), color=BLOCK_C))
    for name, y in WELL_Y.items():  # rim colour bands and what is in the well (visual)
        c = HOT_C if name == "hot" else COLD_C
        for sy in (1, -1):
            g.append(kk.Geom("box", (0.0, y + sy * (WELL[1] / 2 + 0.006), BLOCK[2] + 0.0005), (WELL[0] + 0.024, 0.012, 0.001),
                             color=c, collide=False))
        for sx in (1, -1):
            g.append(kk.Geom("box", (sx * (WELL[0] / 2 + 0.006), y, BLOCK[2] + 0.0005), (0.012, WELL[1] + 0.024, 0.001),
                             color=c, collide=False))
        g.append(kk.Geom("box", (0.0, y, BLOCK[2] - _upper + 0.002), (WELL[0] - 0.004, WELL[1] - 0.004, 0.003),
                         color=STEAM if name == "hot" else ICE, collide=False))
    return g


BLOCK_PART = kk.part("serving_line", (*BLOCK_XY, TOP), _block_geoms(), mass=None, color=BLOCK_C, friction=(0.5, 0.4),
                     contact=(0.002, 0.0))

# -- the chili pan on the counter in front of the block, between the wells -----------------------------------------------
PAN_XY = {"chili": (0.165, 0.0)}
PAN_POS = {n: (*xy, TOP + 0.0005) for n, xy in PAN_XY.items()}
GOAL = {"chili": "hot"}
_rigid = {"solver_position_iteration_count": 16, "solver_velocity_iteration_count": 4}
PAN_PARTS = [kk.part(n, PAN_POS[n], _pan_geoms(CHILI if n == "chili" else SALAD), mass=PAN_MASS, color=STEEL,
                     friction=(0.5, 0.4), rigid=_rigid, contact=(0.002, 0.0)) for n in PAN_POS]

OBJECTS = [BLOCK_PART, *PAN_PARTS]

# -- scoring -----------------------------------------------------------------------------------------------------------
SEAT_OFF, SEAT_TOL, SEAT_TILT = 0.025, 0.006, 5.0  # m, m, deg: a pan seated in a well
SPILL_TILT = 30.0  # deg: the food slides out
PAN_NEAR, RELEASE_DIST = 0.020, 0.008
DROP_FALL, DROP_STEPS = 0.03, 10
STILL_MOVE, STILL_STEPS = 0.002, 20
_PAN_C, _PAN_S = (0.0, 0.0, PAN_H / 2), (*PAN_OUT, PAN_H)
_S: dict = {}


def _reset() -> None:
    _S.update(step=None, result=(0.0, False), last={}, max_tilt=0.0, spilled=False, dropped=False,
              fall={n: kk.FallWatch(DROP_FALL, DROP_STEPS) for n in PAN_POS},
              still=kk.StillWatch(STILL_MOVE, STILL_STEPS))


_reset()
kk.RESET_CALLBACKS.append(_reset)


def well_of(pose):
    """The well a pan is seated in (centred within 2.5 cm, on the well floor, level), or None."""
    p, R = pose
    for w, y in WELL_Y.items():
        if (float(np.hypot(p[0] - BLOCK_XY[0], p[1] - y)) <= SEAT_OFF and abs(p[2] - WELL_FLOOR) <= SEAT_TOL
                and kk.tilt_deg(R) <= SEAT_TILT):
            return w
    return None


def opening_of(pose):
    """The well whose opening the pan's frame is down inside (seated or not), or None."""
    p = pose[0]
    for w, y in WELL_Y.items():
        if abs(p[0] - BLOCK_XY[0]) <= WELL[0] / 2 and abs(p[1] - y) <= WELL[1] / 2 and p[2] < BLOCK_TOP:
            return w
    return None


def _update(objects):
    if _S["step"] == kk.STATE.step and kk.STATE.step >= 0:
        return _S["result"]
    _S["step"] = kk.STATE.step
    poses = {n: kk.pose_of(n, PAN_POS[n]) for n in PAN_POS}
    st = {}
    for n, pose in poses.items():
        tilt = kk.tilt_deg(pose[1])
        _S["max_tilt"] = max(_S["max_tilt"], tilt)
        if tilt > SPILL_TILT:
            _S["spilled"] = True
        handled = kk.touched_by_any(_PAN_C, _PAN_S, pose, PAN_NEAR)
        if _S["fall"][n].update(pose[0][2], handled):
            _S["dropped"] = True
        released = not kk.touched_by_any(_PAN_C, _PAN_S, pose, RELEASE_DIST)
        st[n] = {"well": well_of(pose), "in_opening": opening_of(pose), "tilt": round(tilt, 2), "released": released,
                 "x": round(float(pose[0][0]), 4), "y": round(float(pose[0][1]), 4), "z": round(float(pose[0][2] - TOP), 4)}
    still = _S["still"].update(*(p[0] for p in poses.values()))
    right = [n for n in st if st[n]["well"] == GOAL[n] and st[n]["released"]]
    wrong = [n for n in st if st[n]["in_opening"] not in (None, GOAL[n])]  # down in the other well
    _S["last"] = {"pans": st, "correct": right, "wrong_well": wrong, "max_tilt": round(_S["max_tilt"], 2),
                  "spilled": _S["spilled"], "still": still}
    if wrong or _S["spilled"] or _S["dropped"] or not still:
        res = (0.0, False)
    elif right:
        res = (1.0, True)
    else:
        res = (0.0, False)
    _S["result"] = res
    return res


def success(objects) -> bool:
    """The chili seated in the steam (hot) well: centred within 2.5 cm, on the well floor, tilted under 5 deg and
    released; never tipped past 30 deg or dropped."""
    return _update(objects)[1]


def score(objects) -> float:
    """1.0 the chili seated in the steam well; 0 if it is in the ice well (hot food in cold holding), tipped past 30 deg
    (the chili spills), dropped, or in no well."""
    return _update(objects)[0]


def report() -> dict:
    return {**_S.get("last", {}), "drop_flag": _S.get("dropped")}
