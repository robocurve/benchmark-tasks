"""On the station table, between the arms: a stack of three cardboard boxes, largest at the bottom. The bottom
box is 20 cm wide (left to right), 12 cm deep and 5 cm tall, the middle box 12 cm square and 3.5 cm tall, the top
box 8 cm square and 3 cm tall; each sits centred on the one below. Directly beyond the stack, 3 cm from it, stands
a fixed grey shelf block 6 cm tall with a flat top 16 cm deep and 24 cm wide. The boxes are loose; nothing else is
on the table.
"""

import numpy as np

from yam_arena.demo import FINGER_CLOSED, Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# World frame: x away from the arm bases, y to the robot's left, z up (m).
STACK_XY = (0.20, 0.0)
SIZES = {  # x, y, z edge lengths
    "bottom_box": (0.120, 0.200, 0.050),
    "middle_box": (0.120, 0.120, 0.035),
    "top_box": (0.080, 0.080, 0.030),
}
MASSES = {"bottom_box": 0.12, "middle_box": 0.06, "top_box": 0.04}
COLORS = {"bottom_box": (0.62, 0.45, 0.28), "middle_box": (0.74, 0.56, 0.36), "top_box": (0.84, 0.68, 0.46)}
ORDER = ("bottom_box", "middle_box", "top_box")

SHELF_X0, SHELF_X1 = 0.290, 0.450  # shelf top: 160 mm deep, 30 mm beyond the bottom box's far face
SHELF_W = 0.240
SHELF_H = 0.060
SHELF_TOP = TOP + SHELF_H
SHELF_XY = ((SHELF_X0 + SHELF_X1) / 2, 0.0)


def _start_z():
    z, out = TOP, {}
    for n in ORDER:
        out[n] = z + SIZES[n][2] / 2
        z += SIZES[n][2]
    return out


START_Z = _start_z()
BOXES = [Box(n, pos=(*STACK_XY, START_Z[n]), size=SIZES[n], mass=MASSES[n], color=COLORS[n]) for n in ORDER]
SHELF = Box("shelf", pos=(*SHELF_XY, TOP + SHELF_H / 2), size=(SHELF_X1 - SHELF_X0, SHELF_W, SHELF_H),
            color=(0.55, 0.56, 0.58))
OBJECTS = [*BOXES, SHELF]

ON_TOL = 0.001  # resting on a surface: lowest corner within 1 mm of it
FLAT_DEG = 5.0  # a box lying flat: its top face within 5 degrees of level (upside down counts as flat)
NEAT = 0.010  # neatly stacked: each upper box's centre within 10 mm (horizontally) of the box below's centre
# The stack is broken, for the rest of the trial, once at any moment an upper box is more than 5 mm above the box
# below it, its centre is more than 25 mm (horizontally) from the box below's centre, or any box tilts past 15 deg.
GAP_BREAK = 0.005
OFFSET_BREAK = 0.025
TILT_BREAK_DEG = 15.0
LIFT = 0.020  # lifted as one: the bottom box's lowest corner at least 20 mm above the table, stack unbroken
MOVED = 0.030  # set down elsewhere: the bottom box's centre at least 30 mm from its start
EPS = 1e-6  # every stated limit is inclusive: a value exactly at a limit passes


def _rot(st) -> np.ndarray:
    x, y, z, w = np.asarray(st.quat_xyzw, float) / np.linalg.norm(st.quat_xyzw)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _tilt_deg(st) -> float:
    """Angle between the box's own vertical axis and the world vertical, either way up: a box lying on a side face
    counts as tilted 90 degrees, but one turned upside down counts as flat (its top face is level)."""
    return float(np.degrees(np.arccos(np.clip(abs(_rot(st)[2, 2]), -1.0, 1.0))))


def _corners(st, name) -> np.ndarray:
    """The box's eight corners, from its true size and pose (works either way up)."""
    hx, hy, hz = np.asarray(SIZES[name]) / 2
    local = np.array([[sx * hx, sy * hy, sz * hz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    return np.asarray(st.pos) + local @ _rot(st).T


def _lowest(objects, name) -> float:
    return float(_corners(objects[name], name)[:, 2].min())


def _gap(objects, upper, lower) -> float:
    """Height of the upper box's lowest corner above the lower box's top (its highest corner)."""
    return _lowest(objects, upper) - float(_corners(objects[lower], lower)[:, 2].max())


def _offset(objects, upper, lower) -> float:
    return float(np.linalg.norm(np.asarray(objects[upper].pos[:2]) - np.asarray(objects[lower].pos[:2])))


def _pairs():
    return ((ORDER[1], ORDER[0]), (ORDER[2], ORDER[1]))


def _broken_now(objects) -> bool:
    if any(_tilt_deg(objects[n]) > TILT_BREAK_DEG + EPS for n in ORDER):
        return True
    return any(_gap(objects, u, l) > GAP_BREAK + EPS or _offset(objects, u, l) > OFFSET_BREAK + EPS
               for u, l in _pairs())


# Trial history. The scorer sees one step at a time, so it keeps here whether the stack has been broken and whether
# it has been lifted as one since the trial started. Both clear only on a scene reset: every box within 0.5 mm and
# 1 degree (full rotation) of its starting pose and at rest, reached by a jump of more than 20 mm in one control step
# for some box (the simulator's reset). Rebuilding the start by hand does not clear it.
RESET_TOL, RESET_TURN_DEG, RESET_JUMP = 0.0005, 1.0, 0.020
_HISTORY = {"broken": False, "lifted": False, "prev": None}


def _turn_deg(st) -> float:
    """Full rotation from the starting orientation (every box starts unrotated)."""
    w = abs(float(np.asarray(st.quat_xyzw, float)[3] / np.linalg.norm(st.quat_xyzw)))
    return float(np.degrees(2.0 * np.arccos(min(1.0, w))))


def _at_start(objects) -> bool:
    for b in BOXES:
        st = objects[b.name]
        if (np.linalg.norm(np.asarray(st.pos) - np.asarray(b.pos)) > RESET_TOL + EPS
                or _turn_deg(st) > RESET_TURN_DEG + EPS or not st.resting()):
            return False
    return True


def _update(objects) -> None:
    prev = _HISTORY["prev"]
    now = {n: np.asarray(objects[n].pos, float).copy() for n in ORDER}
    _HISTORY["prev"] = now
    jumped = prev is None or any(np.linalg.norm(now[n] - prev[n]) > RESET_JUMP for n in ORDER)
    if jumped and _at_start(objects):
        _HISTORY["broken"] = False
        _HISTORY["lifted"] = False
    if _broken_now(objects):
        _HISTORY["broken"] = True
    elif _lowest(objects, "bottom_box") >= TOP + LIFT - EPS:
        _HISTORY["lifted"] = True


def _stacked_at_rest(objects, max_offset) -> bool:
    """Every box flat and still, each upper box resting on the box below (lowest corner within 1 mm of its top)
    with its centre within max_offset of the lower box's centre."""
    if not all(objects[n].resting() and _tilt_deg(objects[n]) <= FLAT_DEG + EPS for n in ORDER):
        return False
    return all(abs(_gap(objects, u, l)) <= ON_TOL + EPS and _offset(objects, u, l) <= max_offset + EPS
               for u, l in _pairs())


def _bottom_on(objects, surface_z) -> bool:
    return bool(abs(_lowest(objects, "bottom_box") - surface_z) <= ON_TOL + EPS)


def _wholly_on_shelf(objects) -> bool:
    c = _corners(objects["bottom_box"], "bottom_box")  # its outline seen from above
    return bool(np.all((c[:, 0] >= SHELF_X0 - EPS) & (c[:, 0] <= SHELF_X1 + EPS)
                       & (np.abs(c[:, 1]) <= SHELF_W / 2 + EPS)))


def _centre_on_shelf(objects) -> bool:
    x, y = objects["bottom_box"].pos[:2]
    return bool(SHELF_X0 - EPS <= x <= SHELF_X1 + EPS and abs(y) <= SHELF_W / 2 + EPS)


def _tier(objects) -> float:
    _update(objects)
    if _HISTORY["broken"]:
        return 0.0
    if _bottom_on(objects, SHELF_TOP):
        if _wholly_on_shelf(objects) and _stacked_at_rest(objects, NEAT):
            return 1.0
        if _centre_on_shelf(objects) and _stacked_at_rest(objects, OFFSET_BREAK):
            return 0.5
        return 0.0
    moved = np.linalg.norm(np.asarray(objects["bottom_box"].pos[:2]) - np.asarray(STACK_XY))
    if (_HISTORY["lifted"] and _bottom_on(objects, TOP) and moved >= MOVED - EPS
            and _stacked_at_rest(objects, OFFSET_BREAK)):
        return 0.25
    return 0.0


def success(objects):
    """The bottom box flat on the shelf with its whole base over the shelf top, the middle box on it and the top box
    on the middle box, each within 10 mm of centred, all still, and the stack never broken during the trial."""
    return _tier(objects) == 1.0


def distance(objects):
    return float(np.linalg.norm(np.asarray(objects["bottom_box"].pos[:2]) - np.asarray(SHELF_XY)))


def score(objects):
    """1.0 as success. 0.5: the intact stack on the shelf, its bottom box's centre over the shelf, but overhanging
    the shelf's edge or with an upper box 10 to 25 mm off centre. 0.25: the stack was lifted as one and set down,
    still stacked, on the table at least 30 mm from where it started. 0 otherwise, and 0 for the whole trial once
    the stack has been broken."""
    return _tier(objects)


# ---- scripted demo ------------------------------------------------------------------------------------------
# Both arms squeeze the bottom box between their closed grippers, one on each side face (the faces toward the
# arms), lift the whole stack about 90 mm, carry it 170 mm out over the shelf, lower it to 7 mm above the shelf top and
# pull the grippers sideways off it so it drops onto the shelf. The closed fingers widen upward; pressed against
# the bottom box they touch its upper edge, and they stay clear of the narrower boxes above it. Fingers close
# along x (yaw -pi/2 on the left arm, +pi/2 on the right), which keeps the gripper body off the upper boxes.
TIP = 0.013  # finger tips below the TCP
TIP_Z = 0.010  # finger tips 10 mm above the bottom box's base
TOUCH_Y = 0.123  # TCP distance from the stack's centre line at which the closed fingers touch the bottom box
SQUEEZE = 0.030  # commanded this far past touching, on each side
CLEAR_Y = TOUCH_Y + 0.030
LIFT_Z = 0.100  # commanded: bottom box 100 mm up (about 88 mm with the sag), clear of the shelf top
DROP = 0.007  # released with the bottom box 7 mm above the shelf
# Measured on the L40S with the trace wrapper: holding the stack, both TCPs settle about 12 mm below their targets,
# so the demo aims that much higher when it lowers the stack over the shelf.
SAG_Z = 0.012
TRAVEL_Z = TOP + 0.20  # above the start only
UP_Z = SHELF_TOP + 0.08  # after releasing over the shelf
PLACE_X = SHELF_XY[0]
YAW = {"left": -np.pi / 2, "right": np.pi / 2}
SIDE = {"left": 1.0, "right": -1.0}


def _arm(arm):
    s, yaw = SIDE[arm], YAW[arm]
    x0, y0 = STACK_XY
    z0 = TOP + TIP_Z + TIP  # TCP height with the bottom box on the table
    z_lift = z0 + LIFT_Z
    z_place = SHELF_TOP + DROP + TIP_Z + TIP + SAG_Z
    yc, ys = y0 + s * CLEAR_Y, y0 + s * (TOUCH_Y - SQUEEZE)

    def wp(t, x, y, z, label):
        return Waypoint(t, (float(x), float(y), float(z)), FINGER_CLOSED, label, yaw)

    return [
        wp(2.0, x0, yc, TRAVEL_Z, "beside the stack, fingers closed"),
        wp(1.5, x0, yc, z0, "down beside the bottom box"),
        wp(1.5, x0, ys, z0, "squeeze the bottom box"),
        wp(0.5, x0, ys, z0, "hold the squeeze"),
        wp(2.0, x0, ys, z_lift, "lift the stack"),
        wp(5.0, PLACE_X, ys, z_lift, "carry over the shelf"),
        wp(1.5, PLACE_X, ys, z_place, "lower to just above the shelf"),
        wp(0.5, PLACE_X, ys, z_place, "settle"),
        wp(1.0, PLACE_X, yc, z_place, "pull away: the stack drops onto the shelf"),
        wp(1.0, PLACE_X, yc, UP_Z, "up"),
    ]


def demo():
    return {"left": _arm("left"), "right": _arm("right")}
