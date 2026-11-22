"""On the station table: a black glass cooktop with two burners, a large one (ring 21 cm across, right) and a small one
(14 cm, left). The large pot sits on the small burner; the small pot waits on the counter to the right of the cooktop.

Put each pot on the burner that matches its size, lifting the pots: cookware makers say to lift pans on and off a
glass-topped cooktop, never slide them (sliding scratches it), and to use a burner similar in size to the pan's base
(a mismatched burner wastes heat). The large pot is heavy and has two handles, so it is lifted with both hands; it has
to move first, since the small burner is under it.

Scored each step from poses: which burner each pot is centred on, whether a pot was ever slid on the glass, tipped or
dropped, and whether both are released and still at the end.
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

INSTRUCTION = ("Someone has put the big pot on the small burner. Put each pot on the burner that matches its size. "
               "Lift the pots: never slide a pot across the glass cooktop.")

GLASS_BLACK = (0.05, 0.05, 0.06)
STEEL = (0.62, 0.63, 0.66)
COPPER = (0.72, 0.45, 0.30)
RING = (0.55, 0.10, 0.06)

# -- the cooktop: a glass slab with two burner rings (visual only) --------------------------------------------------
COOK_XY, COOK_SIZE = (0.30, 0.0), (0.28, 0.48, 0.010)
CT = TOP + COOK_SIZE[2]  # the cooktop's surface
LARGE_XY, SMALL_XY = (0.30, -0.110), (0.30, 0.125)
LARGE_RING_R, SMALL_RING_R = 0.100, 0.064  # inner radius of each ring mark (6 mm wide): 21.2 cm and 14.0 cm across outside


def _ring_marks(xy, r_in):
    return [kk.Geom(g.kind, (g.center[0] + xy[0] - COOK_XY[0], g.center[1] + xy[1] - COOK_XY[1], g.center[2]),
                    g.size, g.quat_xyzw, color=RING, collide=False)
            for g in kk.ring(r_in, 0.006, 0.0006, COOK_SIZE[2], n=32)]


COOKTOP = kk.part("cooktop", (*COOK_XY, TOP), [
    kk.Geom("box", (0.0, 0.0, COOK_SIZE[2] / 2), COOK_SIZE, color=GLASS_BLACK),
    *_ring_marks(LARGE_XY, LARGE_RING_R), *_ring_marks(SMALL_XY, SMALL_RING_R),
], mass=None, color=GLASS_BLACK, friction=(0.35, 0.30), contact=(0.002, 0.0))

# -- the two pots (frame: centre of the floor's underside) ----------------------------------------------------------
# outer radius = ring radius + ~5 mm, so each pot visibly matches one ring
LARGE_R, SMALL_R, WALL = 0.105, 0.066, 0.004  # inner radii
LARGE_H, SMALL_H, FLOOR = 0.090, 0.075, 0.006
LARGE_OUT, SMALL_OUT = LARGE_R + WALL, SMALL_R + WALL
LARGE_POS = (*SMALL_XY, CT + 0.0005)  # the large pot starts on the small burner
SMALL_POS = (0.28, -0.335, TOP + 0.0005)  # the small pot starts on the counter, right of the cooktop
_RIGID = {"solver_position_iteration_count": 16, "solver_velocity_iteration_count": 4}
EAR = (0.020, 0.030, 0.012)  # the large pot's two side handles: x, y (sticking out), z
EAR_Y = LARGE_OUT + EAR[1] / 2 - 0.004  # handle centre from the pot's axis (+y and -y)
EAR_Z = LARGE_H - 0.015
LARGE_POT = kk.part("large_pot", LARGE_POS, [
    *kk.vessel(LARGE_R, WALL, LARGE_H, FLOOR, n=32, color=STEEL),
    *[kk.Geom("box", (0.0, sy * EAR_Y, EAR_Z), EAR, color=(0.12, 0.12, 0.13)) for sy in (1, -1)],
], mass=1.50, color=STEEL, rigid=_RIGID, contact=(0.002, 0.0))
SMALL_POT = kk.part("small_pot", SMALL_POS, kk.vessel(SMALL_R, WALL, SMALL_H, FLOOR, n=24, color=COPPER),
                    mass=0.35, color=COPPER, rigid=_RIGID, contact=(0.002, 0.0))
OBJECTS = [COOKTOP, LARGE_POT, SMALL_POT]

POTS = {  # name: (start pose position, matching burner, outer radius, height)
    "large_pot": (LARGE_POS, LARGE_XY, LARGE_OUT, LARGE_H),
    "small_pot": (SMALL_POS, SMALL_XY, SMALL_OUT, SMALL_H),
}
REACH_Y = {"large_pot": EAR_Y + EAR[1] / 2, "small_pot": SMALL_OUT}  # half-width along y incl. handles

# -- scoring -----------------------------------------------------------------------------------------------------------
ON_OFF, ON_TILT, ON_HEIGHT = 0.015, 10.0, 0.005  # m, deg, m: centred on a burner, upright, resting on the glass
DRAG_DOWN = 0.003  # m: a pot's base this close to the glass (and over it) counts as "on the glass"
DRAG_MAX = 0.030  # m: moved this far across the glass within one stretch of touching it = slid -> 0
TIP_MAX = 30.0  # deg: a pot tipped this far at any time -> 0
POT_NEAR = 0.020  # m: for the drop rule, a finger-pad point this close to a pot still has it
DROP_FALL, DROP_STEPS = 0.05, 10
RELEASE_DIST = 0.008
STILL_MOVE, STILL_STEPS = 0.002, 20
_S: dict = {}


def _reset() -> None:
    _S.update(step=None, result=(0.0, False), last={},
              drag={n: 0.0 for n in POTS}, anchor={n: None for n in POTS}, tipped=set(), dropped=set(),
              fall={n: kk.FallWatch(DROP_FALL, DROP_STEPS) for n in POTS}, still=kk.StillWatch(STILL_MOVE, STILL_STEPS))


_reset()
kk.RESET_CALLBACKS.append(_reset)


def _over_glass(xy) -> bool:
    return bool(np.all(np.abs(np.asarray(xy) - np.asarray(COOK_XY)) <= np.asarray(COOK_SIZE[:2]) / 2))


def _update(objects):
    if _S["step"] == kk.STATE.step and kk.STATE.step >= 0:
        return _S["result"]
    _S["step"] = kk.STATE.step
    info, poses, matched, released = {}, {}, 0, True
    for name, (pos0, burner, r_out, h) in POTS.items():
        pose = kk.pose_of(name, pos0)
        poses[name] = pose
        xy, z, tilt = pose[0][:2], float(pose[0][2]), kk.tilt_deg(pose[1])
        box_c, box_s = (0.0, 0.0, h / 2), (2 * r_out, 2 * REACH_Y[name], h)
        low = z - r_out * np.sin(np.radians(tilt))  # the lowest point of the base's rim (a tipped pot drags on it)
        if _over_glass(xy) and low - CT <= DRAG_DOWN:  # touching the glass: net travel since it first touched
            if _S["anchor"][name] is None:
                _S["anchor"][name] = xy.copy()
            _S["drag"][name] = max(_S["drag"][name], float(np.linalg.norm(xy - _S["anchor"][name])))
        else:
            _S["anchor"][name] = None
        if tilt > TIP_MAX:
            _S["tipped"].add(name)
        if _S["fall"][name].update(z, kk.touched_by_any(box_c, box_s, pose, POT_NEAR)):
            _S["dropped"].add(name)
        if kk.touched_by_any(box_c, box_s, pose, RELEASE_DIST):
            released = False
        offs = {b: float(np.linalg.norm(xy - np.asarray(bxy))) for b, bxy in (("large", LARGE_XY), ("small", SMALL_XY))}
        on = {b: bool(o <= ON_OFF and tilt <= ON_TILT and abs(z - CT) <= ON_HEIGHT) for b, o in offs.items()}
        right = on["large" if name == "large_pot" else "small"]
        matched += int(right)
        info[name] = {"to_large_burner": round(offs["large"], 4), "to_small_burner": round(offs["small"], 4),
                      "tilt": round(tilt, 2), "base_above_glass": round(z - CT, 4),
                      "on_matching_burner": right, "on_wrong_burner": on["large" if name == "small_pot" else "small"],
                      "slid_on_glass": round(_S["drag"][name], 4)}
    still = _S["still"].update(*(p[0] for p in poses.values()))
    slid = {n for n, d in _S["drag"].items() if d > DRAG_MAX}
    _S["last"] = {**info, "matched": matched, "released": released, "still": still}
    if slid or _S["tipped"] or _S["dropped"]:
        res = (0.0, False)
    elif released and still:
        res = (0.5 * matched, matched == 2)
    else:
        res = (0.0, False)
    _S["result"] = res
    return res


def success(objects) -> bool:
    """Both pots are centred on their matching burners, released and still for 1 s, and neither was ever slid on the
    glass, tipped over or dropped."""
    return _update(objects)[1]


def score(objects) -> float:
    """0.5 for each pot centred on the burner that matches its size (both released and still); 0 if a pot was slid
    across the glass (> 3 cm), tipped past 30 degrees or dropped."""
    return _update(objects)[0]


def report() -> dict:
    return {**_S.get("last", {}), "slid": sorted(n for n, d in _S["drag"].items() if d > DRAG_MAX),
            "tipped": sorted(_S["tipped"]), "dropped": sorted(_S["dropped"])}
