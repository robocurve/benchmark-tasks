"""On the station table: a grey steel baseplate with a motor base standing on it. The motor base is a dark steel
frame 20 x 14 x 5 cm on four square foot pads, with a blue motor lying on top, its silver shaft pointing away from
the robot. Thin dark scribe lines on the baseplate mark where the base belongs. A steel shim, a thin plate with an
orange tab at one end to hold it by, lies flat on the baseplate between the robot and the base.
"""

import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import parts  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
EPS = 1e-6

# Baseplate (fixed). World frame: x away from the arm bases, y to the robot's left, z up (m).
PLATE = (0.38, 0.24, 0.015)
PLATE_XY = (0.26, 0.0)  # x from 0.07 to 0.45
PT = TOP + PLATE[2]  # baseplate top

# Motor base (one loose rigid part). Origin at the centre of the frame box.
BASE_XY = (0.30, 0.0)
FRAME = (0.20, 0.14, 0.05)
FOOT = 0.03  # square foot pad side
FOOT_H = 0.010  # three feet
SHORT_H = 0.005  # the near-left foot: 5 mm short
FOOT_XY = {"near_left": (-0.085, 0.055), "near_right": (-0.085, -0.055),
           "far_left": (0.085, 0.055), "far_right": (0.085, -0.055)}
FOOT_HEIGHT = {k: (SHORT_H if k == "near_left" else FOOT_H) for k in FOOT_XY}
BASE_Z = PT + FOOT_H + FRAME[2] / 2
MOTOR_R, MOTOR_L = 0.035, 0.10
MOTOR_AT = (0.05, -0.035, FRAME[2] / 2 + MOTOR_R)
BASE_MASS = 1.5

DARK, BLUE, SILVER, GREY, ORANGE = (0.18, 0.19, 0.21), (0.12, 0.30, 0.80), (0.78, 0.79, 0.80), (0.55, 0.56, 0.58), \
    (0.95, 0.50, 0.10)

MOTOR_BASE = parts.Part(
    "motor_base", pos=(*BASE_XY, BASE_Z), mass=BASE_MASS,
    shapes=[parts.box(FRAME, at=(0.0, 0.0, 0.0), color=DARK),
            *[parts.box((FOOT, FOOT, FOOT_HEIGHT[k]), at=(x, y, -FRAME[2] / 2 - FOOT_HEIGHT[k] / 2), color=DARK)
              for k, (x, y) in FOOT_XY.items()],
            parts.cylinder(MOTOR_R, MOTOR_L, at=MOTOR_AT, color=BLUE, axis="x"),
            parts.cylinder(0.008, 0.03, at=(MOTOR_AT[0] + MOTOR_L / 2 + 0.015, MOTOR_AT[1], MOTOR_AT[2]),
                           color=SILVER, axis="x")],
)

# Shim (one loose rigid part): a 4.2 mm steel plate with a tab block at its near end. Origin at the plate's centre.
SHIM_PLATE = (0.10, 0.05, 0.0042)
TAB = (0.015, 0.05, 0.03)
SHIM_START = (0.135, 0.0)
SHIM_Z = PT + SHIM_PLATE[2] / 2
SHIM = parts.Part(
    "shim", pos=(*SHIM_START, SHIM_Z), mass=0.10,
    shapes=[parts.box(SHIM_PLATE, at=(0.0, 0.0, 0.0), color=SILVER),
            parts.box(TAB, at=(-SHIM_PLATE[0] / 2 + TAB[0] / 2, 0.0, -SHIM_PLATE[2] / 2 + TAB[2] / 2), color=ORANGE)],
)

BASEPLATE = Box("baseplate", pos=(*PLATE_XY, TOP + PLATE[2] / 2), size=PLATE, color=GREY)
# Scribe lines (0.5 mm proud, 2 mm wide) 4 mm outside the frame: along the middle of each long side and across the
# far end. They mark the base's place; none is on a foot's path or the shim's.
_LINE_Z = PT + 0.00025
SCRIBES = [
    Box("scribe_left", pos=(BASE_XY[0], FRAME[1] / 2 + 0.005, _LINE_Z), size=(0.10, 0.002, 0.0005), color=DARK),
    Box("scribe_right", pos=(BASE_XY[0], -FRAME[1] / 2 - 0.005, _LINE_Z), size=(0.10, 0.002, 0.0005), color=DARK),
    Box("scribe_far", pos=(BASE_XY[0] + FRAME[0] / 2 + 0.005, 0.0, _LINE_Z), size=(0.002, 0.08, 0.0005), color=DARK),
]
OBJECTS = [BASEPLATE, *SCRIBES, MOTOR_BASE, SHIM]

GAP_MAX = 0.0010  # every foot's underside within 1.0 mm of what it stands on
LEVEL_MAX = np.radians(0.5)
PLACE_MAX, YAW_MAX = 0.003, np.radians(2.0)
COVER_MIN = 0.75  # a shim counts as under a foot when it covers at least 75 % of the pad
COVER_SOME = 0.02  # less than this is "not under it"
SHIM_FLAT = np.radians(2.0)


def _rot(q):
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _tilt(state) -> float:
    return float(np.arccos(np.clip(_rot(state.quat_xyzw)[2, 2], -1.0, 1.0)))


def _yaw(state) -> float:
    r = _rot(state.quat_xyzw)
    return float(np.arctan2(r[1, 0], r[0, 0]))


def _foot_corners(base, k) -> np.ndarray:
    """World corners of the underside of foot k (4 x 3, in order round the pad)."""
    x, y = FOOT_XY[k]
    z = -FRAME[2] / 2 - FOOT_HEIGHT[k]
    h = FOOT / 2
    local = np.array([[x - h, y - h, z], [x + h, y - h, z], [x + h, y + h, z], [x - h, y + h, z]])
    return np.asarray(base.pos) + local @ _rot(base.quat_xyzw).T


def _area(poly) -> float:
    if len(poly) < 3:
        return 0.0
    x, y = np.asarray(poly)[:, 0], np.asarray(poly)[:, 1]
    return float(abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))) / 2)


def _cover(shim, corners) -> float:
    """Fraction of the foot pad's area that lies over the shim's plate, seen from above (exact polygon clip)."""
    r = _rot(shim.quat_xyzw)
    poly = [tuple(v) for v in ((corners - np.asarray(shim.pos)) @ r)[:, :2]]  # pad in the shim's frame
    hx, hy = SHIM_PLATE[0] / 2, SHIM_PLATE[1] / 2
    for axis, sign, lim in ((0, 1, hx), (0, -1, hx), (1, 1, hy), (1, -1, hy)):
        out = []
        for i, p in enumerate(poly):
            q = poly[i - 1]
            pin, qin = sign * p[axis] <= lim, sign * q[axis] <= lim
            if pin != qin:
                t = (lim - sign * q[axis]) / (sign * (p[axis] - q[axis]))
                out.append((q[0] + t * (p[0] - q[0]), q[1] + t * (p[1] - q[1])))
            if pin:
                out.append(p)
        poly = out
        if not poly:
            return 0.0
    return _area(poly) / FOOT ** 2


def _shim_flat(shim) -> bool:
    """Lying flat on the baseplate: tilted at most 2 degrees, its underside within 1 mm of the plate top."""
    bottom = float(shim.pos[2]) - SHIM_PLATE[2] / 2
    return _tilt(shim) <= SHIM_FLAT + EPS and abs(bottom - PT) <= 0.001 + EPS


def gaps(objects) -> dict:
    """Each foot's gap (m): its underside's centre height above what is under it, the shim's top if the shim covers
    at least 75 % of the pad, else the baseplate. None if the shim is part-way under that foot."""
    base, shim = objects["motor_base"], objects["shim"]
    shim_top = float(shim.pos[2]) + SHIM_PLATE[2] / 2
    out = {}
    for k in FOOT_XY:
        pts = _foot_corners(base, k)
        c = _cover(shim, pts)
        if COVER_SOME <= c < COVER_MIN - EPS:
            out[k] = None
            continue
        under = shim_top if c >= COVER_MIN - EPS else PT
        out[k] = float(pts[:, 2].mean()) - under
    return out


def _in_place(base) -> bool:
    d = np.hypot(base.pos[0] - BASE_XY[0], base.pos[1] - BASE_XY[1])
    return d <= PLACE_MAX + EPS and abs(_yaw(base)) <= YAW_MAX + EPS


def success(objects):
    """Solid on four feet: shim flat on the baseplate and fully under a foot (at least 75 % of it) or clear of every
    foot, every foot within 1.0 mm of what it stands on, the base level within 0.5 degrees and on its marks, and
    base and shim still."""
    base, shim = objects["motor_base"], objects["shim"]
    g = gaps(objects)
    return bool(
        all(v is not None and -0.0005 - EPS <= v <= GAP_MAX + EPS for v in g.values())
        and _tilt(base) <= LEVEL_MAX + EPS
        and _in_place(base)
        and _shim_flat(shim)
        and base.resting() and shim.resting()
    )


def distance(objects):
    s = objects["shim"].pos
    target = np.array(BASE_XY) + np.array(FOOT_XY["near_left"]) - np.array([0.015, 0.0])
    return float(np.hypot(s[0] - target[0], s[1] - target[1]))


def score(objects):
    """Pass/fail: 1.0 when success() holds, else 0."""
    return 1.0 if success(objects) else 0.0


# ---- demo: fixed waypoints only --------------------------------------------------------------------------------
# The left arm grips the shim's tab, slides it along the baseplate to in front of the near-left foot, then straight
# in under it, lets go and leaves.
TIP = 0.013
GRIP_Z = PT + 0.014 + TIP  # pad tips 14 mm above the baseplate, on the tab's sides
PRESS = 0.001  # command 1 mm low while sliding so the plate stays on the baseplate
LIFT = 0.012  # carried 12 mm up when moving sideways
OVER_X = 0.004  # the slide is commanded 4 mm further than wanted: the soft arm lags against friction
ABOVE_Z = PT + 0.09
TAB_DX = -SHIM_PLATE[0] / 2 + TAB[0] / 2  # tab centre from the plate centre
SAG_Y = -0.010  # measured here: the shim ends about 13 mm to the robot's right of the commanded line (finger drag and release); aim 10 mm left


def _slide(stop_x, foot):
    fx, fy = np.array(BASE_XY) + np.array(FOOT_XY[foot])
    sx, sy = SHIM_START
    t0 = (sx + TAB_DX, sy)
    ty = fy - SAG_Y
    w = [Waypoint(1.5, (*t0, ABOVE_Z), FINGER_OPEN, "over the tab"),
         Waypoint(1.5, (*t0, GRIP_Z), FINGER_OPEN, "down"),
         Waypoint(0.8, (*t0, GRIP_Z), FINGER_CLOSED, "grip"),
         Waypoint(1.0, (*t0, GRIP_Z + LIFT), FINGER_CLOSED, "lift a little"),
         Waypoint(2.0, (t0[0], ty, GRIP_Z + LIFT), FINGER_CLOSED, "carry sideways"),
         Waypoint(1.0, (t0[0], ty, GRIP_Z - PRESS), FINGER_CLOSED, "set down"),
         Waypoint(3.0, (stop_x + TAB_DX + OVER_X, ty, GRIP_Z - PRESS), FINGER_CLOSED, "slide in"),
         Waypoint(0.8, (stop_x + TAB_DX + OVER_X, ty, GRIP_Z), FINGER_CLOSED, "ease"),
         Waypoint(0.8, (stop_x + TAB_DX + OVER_X, ty, GRIP_Z), FINGER_OPEN, "release"),
         Waypoint(1.0, (stop_x + TAB_DX + OVER_X - 0.02, ty, ABOVE_Z), FINGER_OPEN, "up"),
         Waypoint(1.5, (0.15, 0.20, ABOVE_Z), FINGER_OPEN, "away"),
         Waypoint(2.5, (0.15, 0.20, ABOVE_Z), FINGER_OPEN, "wait")]
    return w


SHIM_HOME_X = BASE_XY[0] + FOOT_XY["near_left"][0] + FOOT / 2 + 0.006 - SHIM_PLATE[0] / 2  # plate end 6 mm past the pad


def _press(foot, depth=0.025):
    """Closed fingers press down on the frame's top above a foot, hold, and lift off."""
    x, y = np.array(BASE_XY) + np.array(FOOT_XY[foot]) * np.array([1.0, 0.9])
    top = BASE_Z + FRAME[2] / 2
    arm = "left" if y > 0 else "right"
    w = [Waypoint(2.0, (0.15, 0.20 if y > 0 else -0.20, ABOVE_Z), FINGER_CLOSED, "wait, base untouched"),
         Waypoint(1.5, (x, y, top + TIP + 0.03), FINGER_CLOSED, "over the corner"),
         Waypoint(1.5, (x, y, top + TIP - depth), FINGER_CLOSED, "press down"),
         Waypoint(2.0, (x, y, top + TIP - depth), FINGER_CLOSED, "hold"),
         Waypoint(1.0, (x, y, top + TIP + 0.03), FINGER_CLOSED, "let go"),
         Waypoint(3.0, (x, y, top + TIP + 0.03), FINGER_CLOSED, "wait")]
    return arm, w


def demo():
    """The shim slid under the near-left foot. To replay the failure cases: SHIM_DEMO=press presses the near-left
    corner down and lets go; SHIM_DEMO=press_fr presses the far-right corner; SHIM_DEMO=wrong slides the shim under
    the near-right foot; SHIM_DEMO=part stops the shim with it only part-way under the near-left foot."""
    mode = os.environ.get("SHIM_DEMO", "")
    if mode == "press":
        arm, w = _press("near_left")
        return {arm: w}
    if mode == "press_fr":
        arm, w = _press("far_right")
        return {arm: w}
    if mode == "wrong":
        return {"left": _slide(SHIM_HOME_X, "near_right")}
    if mode == "shift":  # the right arm first shoves the base to the left at mid-height, then the shim as in the demo
        y0, z = -FRAME[1] / 2 - 0.012, BASE_Z + TIP
        push = [Waypoint(1.5, (BASE_XY[0], y0 - 0.03, z + 0.05), FINGER_CLOSED, "beside the base"),
                Waypoint(1.0, (BASE_XY[0], y0 - 0.03, z), FINGER_CLOSED, "down"),
                Waypoint(3.0, (BASE_XY[0], y0 + 0.10, z), FINGER_CLOSED, "shove 10 cm past the face"),
                Waypoint(1.0, (BASE_XY[0], y0 - 0.03, z), FINGER_CLOSED, "back"),
                Waypoint(2.0, (0.15, -0.20, ABOVE_Z), FINGER_CLOSED, "away")]
        wait = [Waypoint(9.0, (0.15, 0.20, ABOVE_Z), FINGER_OPEN, "wait for the shove")]
        return {"left": wait + _slide(SHIM_HOME_X, "near_left"), "right": push}
    if mode == "part":
        return {"left": _slide(SHIM_HOME_X - 0.020, "near_left")}
    return {"left": _slide(SHIM_HOME_X, "near_left")}
