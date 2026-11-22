"""On the station table: a guest's place setting further out with the finished starter plate on it, the main course
on the pass nearer the robot, and a bus tray to the left. The guest sits facing the robot, so the guest's right is the
robot's left (+y) and the guest's left is the robot's right (-y).

Clear the starter and serve the main course. The instruction states the house standard (clear from the guest's right,
serve food from the guest's left; houses differ on the serving side, clearing from the right is near-universal); the
robot must map the guest's sides onto its own frame, and never set the new plate down before the used one is gone
(unstated). With an arm on each side, the natural way is for the left arm to clear toward the guest's right while the
right arm brings the main in from the guest's left (one arm could also do both in turn).

Scored each step from poses: which side each plate leaves or arrives from, the order, and where the plates end.
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

INSTRUCTION = ("Clear the guest's starter plate onto the bus tray and serve the main course. House standard: clear "
               "from the guest's right, serve food from the guest's left.")

PORCELAIN = (0.95, 0.95, 0.93)
MEAT, MASH, GREEN, CRUMB = (0.45, 0.22, 0.14), (0.93, 0.85, 0.60), (0.25, 0.55, 0.20), (0.80, 0.70, 0.50)
PASS_C, MAT, TUB_C, GUEST_C = (0.70, 0.71, 0.73), (0.30, 0.16, 0.10), (0.25, 0.28, 0.32), (0.35, 0.40, 0.55)

PLATE, PLATE_T, LIP_W, LIP_H = 0.200, 0.010, 0.012, 0.015
_lz = PLATE_T + LIP_H / 2
_BASE = [
    kk.Geom("box", (0.0, 0.0, PLATE_T / 2), (PLATE, PLATE, PLATE_T), color=PORCELAIN),
    *[kk.Geom("box", (sx * (PLATE - LIP_W) / 2, 0.0, _lz), (LIP_W, PLATE, LIP_H), color=PORCELAIN) for sx in (1, -1)],
    *[kk.Geom("box", (0.0, sy * (PLATE - LIP_W) / 2, _lz), (PLATE - 2 * LIP_W, LIP_W, LIP_H), color=PORCELAIN)
      for sy in (1, -1)],
]
MAIN_GEOMS = _BASE + [
    kk.Geom("box", (0.045, 0.0, PLATE_T + 0.0125), (0.050, 0.075, 0.025), color=MEAT, collide=False),
    kk.Geom("box", (-0.04, 0.03, PLATE_T + 0.015), (0.045, 0.05, 0.030), color=MASH, collide=False),
    kk.Geom("box", (-0.04, -0.035, PLATE_T + 0.01), (0.04, 0.045, 0.020), color=GREEN, collide=False),
]
STARTER_GEOMS = _BASE + [kk.Geom("box", (0.02, -0.02, PLATE_T + 0.003), (0.05, 0.04, 0.006), color=CRUMB, collide=False)]
PROTEIN_DIR = np.array([1.0, 0.0, 0.0])  # the steak's side on the main plate, in its frame
FOOD_C, FOOD_S = (0.0, 0.0, PLATE_T + 0.02), (0.13, 0.13, 0.04)

# -- the place setting (cover) with the starter, the guest (scenery), the pass with the main, the bus tub ---------------
COVER_XY = (0.36, 0.0)
PASS_XY, PASS = (0.14, 0.0), (0.22, 0.24, 0.040)
PASS_TOP = TOP + PASS[2]
TUB_XY, TUB = (0.20, 0.30), (0.24, 0.24, 0.015)  # a low-rimmed bus tray
SCENERY = kk.part("pass_shelf", (*PASS_XY, TOP), [
    kk.Geom("box", (0.0, 0.0, PASS[2] / 2), PASS, color=PASS_C),
    kk.Geom("box", (COVER_XY[0] - PASS_XY[0], COVER_XY[1] - PASS_XY[1], 0.0005), (0.26, 0.26, 0.001), color=MAT,
            collide=False),
    kk.Geom("box", (0.62 - PASS_XY[0], 0.0, 0.20), (0.18, 0.34, 0.40), color=GUEST_C, collide=False),  # the guest
], mass=None, color=PASS_C, friction=(0.5, 0.4), contact=(0.002, 0.0))
TUB_PART = kk.part("bus_tub", (*TUB_XY, TOP), [
    kk.Geom("box", (0.0, 0.0, 0.003), (TUB[0], TUB[1], 0.006), color=TUB_C),
    *[kk.Geom("box", (s * (TUB[0] - 0.006) / 2, 0.0, TUB[2] / 2), (0.006, TUB[1], TUB[2]), color=TUB_C) for s in (1, -1)],
    *[kk.Geom("box", (0.0, s * (TUB[1] - 0.006) / 2, TUB[2] / 2), (TUB[0] - 0.012, 0.006, TUB[2]), color=TUB_C)
      for s in (1, -1)],
], mass=None, color=TUB_C, friction=(0.5, 0.4), contact=(0.002, 0.0))
_RIGID = {"solver_position_iteration_count": 16, "solver_velocity_iteration_count": 4}
STARTER_POS = (*COVER_XY, TOP + 0.0005)
MAIN_POS = (*PASS_XY, PASS_TOP + 0.0005)
STARTER = kk.part("starter_plate", STARTER_POS, STARTER_GEOMS, mass=0.25, color=PORCELAIN, friction=(0.6, 0.5),
                  rigid=_RIGID, contact=(0.002, 0.0))
MAIN = kk.part("main_plate", MAIN_POS, MAIN_GEOMS, mass=0.30, color=PORCELAIN, friction=(0.6, 0.5), rigid=_RIGID,
               contact=(0.002, 0.0))
OBJECTS = [SCENERY, TUB_PART, STARTER, MAIN]

# -- scoring -----------------------------------------------------------------------------------------------------------
LIFTED, CROSS_R = 0.010, 0.10  # m: lift-off; where a plate's path crosses the circle 10 cm round the cover decides the side
SIDE = 0.050  # m: the crossing at least this far to that side of the cover's centre (within 60 deg of the side)
COVER_TOL, FACE_TOL = 0.030, 25.0
LAP_X = 0.50  # m: no plate may pass over the guest's lap
OFF_COVER = 0.20  # m: the starter's centre this far from the cover's centre has left it
FOOD_NEAR, NEAR, RELEASE_DIST = 0.008, 0.020, 0.008
DROP_FALL, DROP_STEPS = 0.03, 10
STILL_MOVE, STILL_STEPS = 0.002, 20
_PC, _PS = (0.0, 0.0, (PLATE_T + LIP_H) / 2), (PLATE, PLATE, PLATE_T + LIP_H)
_S: dict = {}


def _reset() -> None:
    _S.update(step=None, result=(0.0, False), last={}, s_path=[], m_path=[], s_lift=False, m_lift=False,
              starter_gone=False, early=False, lap=False, food=False, food_at=None, dropped=False,
              fall={n: kk.FallWatch(DROP_FALL, DROP_STEPS) for n in ("starter", "main")},
              still=kk.StillWatch(STILL_MOVE, STILL_STEPS))


_reset()
kk.RESET_CALLBACKS.append(_reset)


def _side(path, first: bool) -> float | None:
    """The y offset from the cover's centre where a plate's horizontal path crosses the circle CROSS_R round the cover:
    the first time it goes out of it (first=True: the starter leaving) or the last time it comes into it (first=False:
    the main arriving); None if it never crosses. (A mean over the path's last stretch was diluted by the plate's
    millimetre settling jitter once it rested on the cover.)"""
    if len(path) < 2:
        return None
    c = np.asarray(COVER_XY, float)
    pts = np.asarray(path, float)
    r = np.linalg.norm(pts - c, axis=1)
    inside = r <= CROSS_R
    order = range(1, len(pts)) if first else range(len(pts) - 1, 0, -1)
    for i in order:
        if (first and inside[i - 1] and not inside[i]) or (not first and not inside[i - 1] and inside[i]):
            f = (CROSS_R - r[i - 1]) / (r[i] - r[i - 1]) if r[i] != r[i - 1] else 0.0
            return float(pts[i - 1][1] + f * (pts[i][1] - pts[i - 1][1]) - c[1])
    return None


def _food_by(pose) -> list:
    """Which grippers are within FOOD_NEAR of the food box on a plate at ``pose`` (a diagnostic for the report)."""
    return [a for a in kk.STATE.fingers if kk.near(a, FOOD_C, FOOD_S, pose, FOOD_NEAR)]


def _update(objects):
    if _S["step"] == kk.STATE.step and kk.STATE.step >= 0:
        return _S["result"]
    _S["step"] = kk.STATE.step
    st = kk.pose_of("starter_plate", STARTER_POS)
    mn = kk.pose_of("main_plate", MAIN_POS)
    if st[0][2] - STARTER_POS[2] > LIFTED:
        _S["s_lift"] = True
    if mn[0][2] - MAIN_POS[2] > LIFTED:
        _S["m_lift"] = True
    if _S["s_lift"]:
        _S["s_path"].append(st[0][:2].copy())
    if _S["m_lift"]:
        _S["m_path"].append(mn[0][:2].copy())
    if float(np.linalg.norm(st[0][:2] - np.asarray(COVER_XY))) >= OFF_COVER:
        _S["starter_gone"] = True
    m_off = float(np.linalg.norm(mn[0][:2] - np.asarray(COVER_XY)))
    m_down = mn[0][2] - TOP <= 0.004 and m_off <= 0.08
    # or left resting on the used plate (within 8 cm of it, at most 2.5 cm above its base, no finger near it)
    stacked = (float(np.linalg.norm(mn[0][:2] - st[0][:2])) <= 0.08 and 0.0 < mn[0][2] - st[0][2] <= LIP_H + 0.010
               and not kk.touched_by_any(_PC, _PS, mn, NEAR))
    if (m_down and not _S["starter_gone"]) or stacked:
        _S["early"] = True
    for pose, name in ((st, "starter"), (mn, "main")):
        if pose[0][0] > LAP_X:
            _S["lap"] = True
        if _S["fall"][name].update(pose[0][2], kk.touched_by_any(_PC, _PS, pose, NEAR)):
            _S["dropped"] = True
    who = _food_by(mn)
    if who and not _S["food"]:
        _S["food_at"] = (kk.STATE.step, who)
    if who:
        _S["food"] = True
    d = mn[1] @ PROTEIN_DIR
    facing = abs(math.degrees(math.atan2(d[1], d[0])))
    in_tub = (abs(st[0][0] - TUB_XY[0]) <= TUB[0] / 2 - 0.05 and abs(st[0][1] - TUB_XY[1]) <= TUB[1] / 2 - 0.05
              and st[0][2] - TOP <= 0.012)
    served = m_down and m_off <= COVER_TOL
    released = not (kk.touched_by_any(_PC, _PS, st, RELEASE_DIST) or kk.touched_by_any(_PC, _PS, mn, RELEASE_DIST))
    still = _S["still"].update(st[0], mn[0])
    exit_y, entry_y = _side(_S["s_path"], True), _side(_S["m_path"], False)
    clear_ok = exit_y is not None and exit_y >= SIDE
    serve_ok = entry_y is not None and entry_y <= -SIDE
    _S["last"] = {"exit_side_y": None if exit_y is None else round(exit_y, 4),
                  "entry_side_y": None if entry_y is None else round(entry_y, 4), "clear_ok": clear_ok,
                  "serve_ok": serve_ok, "in_tub": bool(in_tub), "served": bool(served), "facing_deg": round(facing, 1),
                  "set_before_cleared": _S["early"], "over_lap": _S["lap"], "food_touched": _S["food"], "food_at": _S.get("food_at"),
                  "released": released, "still": still}
    if _S["early"] or _S["dropped"] or _S["food"] or _S["lap"] or not (in_tub and served and released and still):
        res = (0.0, False)
    else:
        base = 1.0 if clear_ok and serve_ok else (0.5 if clear_ok or serve_ok else 0.25)
        if facing > FACE_TOL:
            base = max(0.0, base - 0.25)
        res = (base, base == 1.0)
    _S["result"] = res
    return res


def success(objects) -> bool:
    """The starter left the cover toward the guest's right (robot's left) and ended on the bus tray; after it was gone,
    the main arrived from the guest's left (robot's right) and was set down centred on the cover with the steak toward
    the guest; nothing passed over the guest's lap, the food was not touched, nothing dropped."""
    return _update(objects)[1]


def score(objects) -> float:
    """1.0 both sides right; 0.5 one side wrong; 0.25 both wrong; minus 0.25 if the steak is not toward the guest; 0 if
    the main went down before the starter left the cover, a plate passed over the guest's lap, the food was touched, a
    plate was dropped, or the starter is not on the tray / the main not on the cover at the end."""
    return _update(objects)[0]


def report() -> dict:
    return dict(_S.get("last", {}))
