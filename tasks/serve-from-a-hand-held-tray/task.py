"""On the station table: a service station on the far right with a serving tray on it (two end handles) carrying two
clean tumblers, and on the far left a guest's table (a cloth with two coasters) with the guest seated beyond it.

Serve the two drinks onto the coasters. Service rules, not stated in the instruction: the tray is never set down on the
guest's table (hotel food and beverage service practice: hold the tray in one hand and serve with the other), and clean
glasses are handled so that their lip-contact surface is not touched (FDA Food Code 4-904.11: tableware such as
tumblers is handled so that contamination of lip-contact surfaces is prevented); the tray goes back to the station
afterwards. Holding the tray up beside the table with one arm while the other serves is the standard way to do it and
what the scripted demo does; both the station and the coasters are within reach of either arm, though
(the demo's reach check), so one arm could also serve the drinks straight from the tray left on the station: the
task is bimanual by service practice, not by physics.

Scored each step from poses: where each glass is and whether it is upright and let go, whether any gripper pad came near
a glass's rim zone, whether the tray ever touched the guest's table, and where the tray ends up.
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

INSTRUCTION = "Serve the two drinks from the tray onto the guest's coasters."

STEEL = (0.72, 0.73, 0.76)
TRAY_C = (0.20, 0.20, 0.22)
GLASS_C, RIM_C = (0.78, 0.86, 0.92), (0.90, 0.95, 0.98)
CLOTH_C, COASTER_C = (0.93, 0.92, 0.88), (0.45, 0.30, 0.20)
GUEST_C = (0.30, 0.35, 0.55)

# -- the service station (static) ----------------------------------------------------------------------------------------
STATION_XY, STATION = (0.36, -0.32), (0.34, 0.22, 0.040)
STATION_TOP = TOP + STATION[2]
STATION_PART = kk.part("station", (*STATION_XY, TOP), [kk.Geom("box", (0.0, 0.0, STATION[2] / 2), STATION)],
                       mass=None, color=STEEL, contact=(0.002, 0.0))

# -- the guest's table: a cloth with two coaster marks, and the guest (static) --------------------------------------------
CLOTH_XY, CLOTH = (0.40, 0.25), (0.20, 0.28, 0.002)
CLOTH_TOP = TOP + CLOTH[2]
COASTERS = {"near": (0.36, 0.20), "far": (0.40, 0.29)}
COASTER = 0.080
CLOTH_PART = kk.part("cloth", (*CLOTH_XY, TOP), [
    kk.Geom("box", (0.0, 0.0, CLOTH[2] / 2), CLOTH, color=CLOTH_C),
    *[kk.Geom("box", (x - CLOTH_XY[0], y - CLOTH_XY[1], CLOTH[2] + 0.0005), (COASTER, COASTER, 0.001), color=COASTER_C,
              collide=False) for x, y in COASTERS.values()],
], mass=None, color=CLOTH_C, contact=(0.002, 0.0))
GUEST_PART = kk.part("guest", (0.64, 0.25, TOP), [
    kk.Geom("box", (0.0, 0.0, 0.20), (0.20, 0.30, 0.40), color=GUEST_C),
    kk.Geom("box", (0.0, 0.0, 0.47), (0.14, 0.14, 0.14), color=(0.85, 0.70, 0.58)),
], mass=None, color=GUEST_C, contact=(0.002, 0.0))

# -- the tray (frame: centre of its underside): a slab with a low rim along its long sides and a handle bar standing
# across each end (+-x) ------------------------------------------------------------------------------------------------
TRAY, TRAY_T, RIM_W, RIM_H = (0.30, 0.16), 0.008, 0.008, 0.006  # 30 cm long: the hand on a handle stays 7 cm clear of
# the 12 cm tall glasses (on a 22 cm tray the gripper came down on the near glass and never got hold of the handle)
HANDLE = (0.020, 0.080, 0.030)  # x, y, z: a bar across each end (2 x 3 cm, so one pinch holds the loaded tray level;
# a 1 x 1.8 cm bar let it turn 9 deg in the pinch)
HANDLE_X = TRAY[0] / 2 - HANDLE[0] / 2
TRAY_GEOMS = [
    kk.Geom("box", (0.0, 0.0, TRAY_T / 2), (*TRAY, TRAY_T)),
    *[kk.Geom("box", (0.0, sy * (TRAY[1] - RIM_W) / 2, TRAY_T + RIM_H / 2), (TRAY[0], RIM_W, RIM_H)) for sy in (1, -1)],
    *[kk.Geom("box", (sx * HANDLE_X, 0.0, TRAY_T + HANDLE[2] / 2), HANDLE, contact=(0.005, 0.0)) for sx in (1, -1)],
]
TRAY_XY = (0.36, -0.30)
TRAY_POS = (*TRAY_XY, STATION_TOP + 0.0005)
_RIGID = {"solver_position_iteration_count": 32, "solver_velocity_iteration_count": 4}
TRAY_PART = kk.part("tray", TRAY_POS, TRAY_GEOMS, mass=0.15, color=TRAY_C, rigid=_RIGID, friction=(0.8, 0.7),
                    contact=(0.002, 0.0))
TRAY_HANDLE_C = {"near": (-HANDLE_X, 0.0, TRAY_T + HANDLE[2] / 2), "far": (HANDLE_X, 0.0, TRAY_T + HANDLE[2] / 2)}

# -- the tumblers (frame: centre of the base) ------------------------------------------------------------------------------
GLASS, GLASS_H, RIM_ZONE = 0.060, 0.120, 0.025  # width, height (a 12 cm highball), the top band that touches the
# guest's lips
GLASS_ON_TRAY = {"glass_a": (-0.042, 0.0), "glass_b": (0.042, 0.0)}  # tray frame (x, y): either side of the centre
GLASS_PARTS = []
GLASS_POS = {}
for _n, (_x, _y) in GLASS_ON_TRAY.items():
    GLASS_POS[_n] = (TRAY_XY[0] + _x, TRAY_XY[1] + _y, TRAY_POS[2] + TRAY_T + 0.0005)
    GLASS_PARTS.append(kk.part(_n, GLASS_POS[_n], [
        kk.Geom("box", (0.0, 0.0, (GLASS_H - RIM_ZONE) / 2), (GLASS, GLASS, GLASS_H - RIM_ZONE), color=GLASS_C),
        kk.Geom("box", (0.0, 0.0, GLASS_H - RIM_ZONE / 2), (GLASS, GLASS, RIM_ZONE), color=RIM_C),
    ], mass=0.10, color=GLASS_C, rigid=_RIGID, friction=(0.9, 0.8), contact=(0.002, 0.0)))

OBJECTS = [STATION_PART, CLOTH_PART, GUEST_PART, TRAY_PART, *GLASS_PARTS]

# -- scoring -----------------------------------------------------------------------------------------------------------
COASTER_TOL = 0.020  # m: a served glass's centre this close to a coaster's centre
UPRIGHT = 10.0  # deg
REST = 0.006  # m: a glass's base this close to the cloth is standing on it
RELEASE_DIST = 0.010  # m: no gripper pad this close to a served glass
RIM_NEAR = 0.004  # m: a gripper pad this close to a glass's rim zone touches it
TRAY_TOUCH = 0.005  # m: the tray's underside this close to the cloth, over it, touches the guest's table
TRAY_HOME_TILT = 5.0  # deg
FALL = (0.05, 10)  # m, steps: a glass dropping this far within this many steps with no gripper near it
TIP = 45.0  # deg: a glass knocked over
_RIM_C, _RIM_S = (0.0, 0.0, GLASS_H - RIM_ZONE / 2), (GLASS, GLASS, RIM_ZONE)
_GLASS_C, _GLASS_S = (0.0, 0.0, GLASS_H / 2), (GLASS, GLASS, GLASS_H)
_S: dict = {}
_FALLS = {n: kk.FallWatch(*FALL) for n in GLASS_ON_TRAY}


def _reset() -> None:
    _S.update(step=None, result=(0.0, False), last={}, rim=False, tray_on_table=False, fell=False)
    for w in _FALLS.values():
        w.reset()


_reset()
kk.RESET_CALLBACKS.append(_reset)


def _served(glass) -> str | None:
    p, R = glass
    if kk.tilt_deg(R) > UPRIGHT or p[2] - CLOTH_TOP > REST:
        return None
    if any(kk.near(a, _GLASS_C, _GLASS_S, glass, RELEASE_DIST) for a in kk.STATE.fingers):
        return None
    for k, (x, y) in COASTERS.items():
        if float(np.hypot(p[0] - x, p[1] - y)) <= COASTER_TOL:
            return k
    return None


def _tray_over_cloth(tray) -> bool:
    p, R = tray
    corners = np.array([[sx * TRAY[0] / 2, sy * TRAY[1] / 2, 0.0] for sx in (1, -1) for sy in (1, -1)]) @ R.T + p
    lo = float(corners[:, 2].min())
    x0, x1 = CLOTH_XY[0] - CLOTH[0] / 2, CLOTH_XY[0] + CLOTH[0] / 2
    y0, y1 = CLOTH_XY[1] - CLOTH[1] / 2, CLOTH_XY[1] + CLOTH[1] / 2
    over = (corners[:, 0].max() > x0 and corners[:, 0].min() < x1 and corners[:, 1].max() > y0
            and corners[:, 1].min() < y1)
    return over and lo - CLOTH_TOP <= TRAY_TOUCH


def _tray_home(tray) -> bool:
    p, R = tray
    on = (abs(p[0] - STATION_XY[0]) <= (STATION[0] - TRAY[0]) / 2 + 0.01
          and abs(p[1] - STATION_XY[1]) <= (STATION[1] - TRAY[1]) / 2 + 0.01 and abs(p[2] - STATION_TOP) <= 0.006)
    held = any(kk.near(a, c, HANDLE, tray, RELEASE_DIST) for a in kk.STATE.fingers for c in TRAY_HANDLE_C.values())
    return on and kk.tilt_deg(R) <= TRAY_HOME_TILT and not held


def _update(objects):
    if _S["step"] == kk.STATE.step and kk.STATE.step >= 0:
        return _S["result"]
    _S["step"] = kk.STATE.step
    tray = kk.pose_of("tray", TRAY_POS)
    served, glasses = {}, {}
    for n in GLASS_ON_TRAY:
        g = kk.pose_of(n, GLASS_POS[n])
        if any(kk.near(a, _RIM_C, _RIM_S, g, RIM_NEAR) for a in kk.STATE.fingers):
            _S["rim"] = True
        handled = any(kk.near(a, _GLASS_C, _GLASS_S, g, 0.02) for a in kk.STATE.fingers)
        if _FALLS[n].update(g[0][2], handled) or kk.tilt_deg(g[1]) > TIP:
            _S["fell"] = True
        served[n] = _served(g)
        glasses[n] = {"x": round(float(g[0][0]), 3), "y": round(float(g[0][1]), 3), "z": round(float(g[0][2] - TOP), 4),
                      "tilt": round(kk.tilt_deg(g[1]), 1), "coaster": served[n]}
    if _tray_over_cloth(tray):
        _S["tray_on_table"] = True
    home = _tray_home(tray)
    coasters = {c for c in served.values() if c}
    n_served = len(coasters)  # two glasses on one coaster count once
    _S["last"] = {"glasses": glasses, "served": n_served, "rim_touched": _S["rim"],
                  "tray_on_guest_table": _S["tray_on_table"], "glass_fell": _S["fell"], "tray_back": home,
                  "tray_tilt": round(kk.tilt_deg(tray[1]), 1), "tray_z": round(float(tray[0][2] - TOP), 4)}
    if _S["rim"] or _S["tray_on_table"] or _S["fell"]:
        res = (0.0, False)
    else:
        sc = 0.5 * n_served
        res = (sc if home else min(sc, 0.75), n_served == 2 and home)
    _S["result"] = res
    return res


def success(objects) -> bool:
    """Both glasses upright on the two coasters (centres within 2 cm) and let go, the tray back on the station and let
    go; the tray never touched the guest's table, no gripper pad came within 4 mm of a glass's top 2.5 cm, no glass
    fell."""
    return _update(objects)[1]


def score(objects) -> float:
    """0.5 per served coaster, capped at 0.75 while the tray is not back on the station; 0 if the tray touched the guest's
    table, a glass's rim zone was touched, or a glass fell or was knocked over."""
    return _update(objects)[0]


def report() -> dict:
    return dict(_S.get("last", {}))
