"""A cafe counter is being set for a camera test: two mugs on blue taped squares, one in front of each arm, and a
red pin in the middle of the table. Every mug has to be turned in place, upright, so its handle points at the pin,
the way a tray is set with all handles facing the same spot. Each mug can only be reached by the arm beside it.
"""

import math

from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Asset, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
RED, BLUE, GREEN = (0.85, 0.1, 0.1), (0.1, 0.25, 0.85), (0.3, 0.55, 0.35)
YELLOW = (0.95, 0.8, 0.1)

PIN = (0.42, 0.0)
PIN_R, PIN_H = 0.02, 0.08
MUG_START = {"mug_left": (0.30, 0.28), "mug_right": (0.30, -0.28)}  # body centre = square centre
HANDLE_START = {"mug_left": math.radians(40.0), "mug_right": math.radians(-160.0)}  # handle bearing, world x = 0
MUG_EXTENT, MUG_H, MUG_MASS = 0.09, 0.063, 0.2  # widest extent incl. handle; the body is 7.2 cm across
HANDLE_OFFSET = 0.0093  # bounding-box centre sits this far from the body axis, towards the handle
TOL_DEG, TOL_POS = 5.0, 0.02
SQUARE, TAPE_W, TAPE_T = 0.12, 0.012, 0.002  # inner side of each taped square


def _frame(prefix, cx, cy):
    s, h = SQUARE + TAPE_W, SQUARE / 2 + TAPE_W / 2
    z = TOP + TAPE_T / 2
    return [Box(f"{prefix}_x{i}", pos=(cx + d, cy, z), size=(TAPE_W, s, TAPE_T), color=BLUE) for i, d in enumerate((-h, h))] + [
        Box(f"{prefix}_y{i}", pos=(cx, cy + d, z), size=(s, TAPE_W, TAPE_T), color=BLUE) for i, d in enumerate((-h, h))]


OBJECTS = [
    Cylinder("pin", pos=(PIN[0], PIN[1], TOP + PIN_H / 2), radius=PIN_R, height=PIN_H, color=RED),
    *_frame("sq_l", *MUG_START["mug_left"]),
    *_frame("sq_r", *MUG_START["mug_right"]),
    # open tray, decoration only, beyond the arms' reach
    Box("tray_base", pos=(0.66, 0.0, TOP + 0.002), size=(0.14, 0.30, 0.004), color=GREEN),
    Box("tray_back", pos=(0.725, 0.0, TOP + 0.02), size=(0.01, 0.30, 0.04), color=GREEN),
    Box("tray_l", pos=(0.66, 0.15, TOP + 0.02), size=(0.14, 0.01, 0.04), color=GREEN),
    Box("tray_r", pos=(0.66, -0.15, TOP + 0.02), size=(0.14, 0.01, 0.04), color=GREEN),
    *[Asset(n, "mug_ycb_robolab", xy=(float(x + HANDLE_OFFSET * math.cos(HANDLE_START[n])),
                                       float(y + HANDLE_OFFSET * math.sin(HANDLE_START[n]))),
            extent=MUG_EXTENT, mass=MUG_MASS, color=YELLOW, yaw=float(HANDLE_START[n]))
      for n, (x, y) in MUG_START.items()],
]


def _target(name):
    x, y = MUG_START[name]
    return math.atan2(PIN[1] - y, PIN[0] - x)  # bearing of the pin from the mug, world x = 0


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _mug(o):
    """(body centre x, y, handle bearing, upright and on the table). The handle points along the mug's yaw."""
    yaw = 2 * math.atan2(float(o.quat_xyzw[2]), float(o.quat_xyzw[3]))
    x = float(o.pos[0]) - HANDLE_OFFSET * math.cos(yaw)
    y = float(o.pos[1]) - HANDLE_OFFSET * math.sin(yaw)
    qx, qy = float(o.quat_xyzw[0]), float(o.quat_xyzw[1])
    ok = 1 - 2 * (qx * qx + qy * qy) > 0.99 and float(o.low[2]) < TOP + 0.01
    return x, y, yaw, bool(ok)


def score(objects):
    """Per mug: upright on the table, body centre within 2 cm of its square centre, handle bearing within 5 degrees
    of the bearing from the mug centre to the pin centre. 1.0 if both mugs pass, 0.5 if one, otherwise 0."""
    n = 0
    for name, (sx, sy) in MUG_START.items():
        x, y, yaw, ok = _mug(objects[name])
        err = abs(math.degrees(_wrap(yaw - _target(name))))
        n += int(ok and math.hypot(x - sx, y - sy) <= TOL_POS and err <= TOL_DEG)
    return n / 2


TRAVEL_Z = TOP + 0.16
PARK = {"left": (0.16, 0.32, TOP + 0.20), "right": (0.12, -0.38, TOP + 0.20)}
GRIP = {"mug_left": math.radians(40.0), "mug_right": math.radians(20.0)}  # grip yaw: fingers square to the handle
TURN = {"mug_left": _target("mug_left") - HANDLE_START["mug_left"],
        "mug_right": _wrap(_target("mug_right") - HANDLE_START["mug_right"])}


def _turn(name):
    x, y = MUG_START[name]
    z = TOP + PAD_TIP_PAST_TCP + min(0.4 * MUG_H, 0.035)
    g = float(GRIP[name])
    return pick_place_waypoints((float(x), float(y), z), (float(x), float(y), z + 0.003), travel_z=TRAVEL_Z,
                                yaw=g, place_yaw=g + float(TURN[name]))


def demo():
    """Each arm lifts its own mug, turns its wrist so the handle points at the pin, sets it back and moves clear."""
    left, right = _turn("mug_left"), _turn("mug_right")
    left.append(Waypoint(2.0, PARK["left"], FINGER_OPEN, "move clear"))
    right.append(Waypoint(2.0, PARK["right"], FINGER_OPEN, "move clear"))
    return {"left": left, "right": right}
