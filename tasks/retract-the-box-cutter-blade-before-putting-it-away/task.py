"""On the station table: a box cutter lies flat in front of the right arm, its yellow handle running away from the
robot. Its grey blade slides along the handle and is moved by the dark thumb knob on the handle's top; the blade
stays where the knob leaves it. Between the arms, a little further out, is an open grey tray with low walls.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402

import parts  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

YELLOW, STEEL, DARK, GREY = (0.95, 0.78, 0.10), (0.70, 0.72, 0.75), (0.15, 0.15, 0.17), (0.55, 0.56, 0.58)

# Cutter, in its own frame: origin at the handle's bottom centre, x along the handle toward the blade end.
HANDLE = (0.110, 0.025, 0.022)
FRONT = HANDLE[0] / 2  # the handle's front face
BLADE = (0.045, 0.004, 0.012)  # length, thickness, height
BLADE_OUT = 0.030  # blade showing past the front at the start
KNOB = (0.014, 0.014, 0.010)
KNOB_X = 0.010  # knob centre at the start; it moves back with the blade
CUTTER_XY = (0.200, -0.200)

BLADE_SLIDE = parts.Slider(
    "blade",
    shapes=[
        parts.box(BLADE, at=(FRONT + BLADE_OUT - BLADE[0] / 2, 0.0, 0.011), color=STEEL),
        parts.box(KNOB, at=(KNOB_X, 0.0, HANDLE[2] + KNOB[2] / 2), color=DARK),
    ],
    axis=(-1.0, 0.0, 0.0),  # positive: the blade slides back into the handle
    travel=BLADE_OUT,
    mass=0.015,
    spring=15.0,  # N/m toward the current notch (see _detent): at most 0.23 N to push past the middle
    damping=2.0,
)
CUTTER = parts.Part(
    "box_cutter",
    pos=(*CUTTER_XY, TOP),
    shapes=[parts.box(HANDLE, at=(0.0, 0.0, HANDLE[2] / 2), color=YELLOW)],
    joints=[BLADE_SLIDE],
    mass=0.050,
)

# Tray: fixed, open top. Inner 160 x 80 mm, floor 5 mm, walls 10 mm thick and 15 mm above the floor.
TRAY_XY = (0.280, -0.030)
TRAY_IN, TRAY_FLOOR, TRAY_WALL, TRAY_H = (0.160, 0.080), 0.005, 0.010, 0.015
_ox, _oy = TRAY_IN[0] + 2 * TRAY_WALL, TRAY_IN[1] + 2 * TRAY_WALL
_wz = TRAY_FLOOR + TRAY_H / 2
TRAY = parts.Part(
    "tray",
    pos=(*TRAY_XY, TOP),
    shapes=[
        parts.box((_ox, _oy, TRAY_FLOOR), at=(0.0, 0.0, TRAY_FLOOR / 2), color=GREY),
        parts.box((_ox, TRAY_WALL, TRAY_H), at=(0.0, (TRAY_IN[1] + TRAY_WALL) / 2, _wz), color=GREY),
        parts.box((_ox, TRAY_WALL, TRAY_H), at=(0.0, -(TRAY_IN[1] + TRAY_WALL) / 2, _wz), color=GREY),
        parts.box((TRAY_WALL, TRAY_IN[1], TRAY_H), at=((TRAY_IN[0] + TRAY_WALL) / 2, 0.0, _wz), color=GREY),
        parts.box((TRAY_WALL, TRAY_IN[1], TRAY_H), at=(-(TRAY_IN[0] + TRAY_WALL) / 2, 0.0, _wz), color=GREY),
    ],
)

OBJECTS = [CUTTER, TRAY]

# Two-notch slide lock, as on a utility knife: the spring pulls the blade to the nearer notch, fully out (0) or fully
# in (30 mm). Past half-way the rest point jumps to the other notch, so the blade snaps into it. Holding force at a
# notch: 15 N/m, so the blade's own weight (0.15 N) moves it at most 10 mm even with the cutter vertical, and less than
# 2 mm at 10 deg of tilt; it never crosses half-way on its own.
NOTCHES = (0.0, BLADE_OUT)
_notch = {"target": 0.0}


@parts.on_reset
def _detent_reset(sim):
    _notch["target"] = 0.0
    sim.set_target("box_cutter", "blade", 0.0)


@parts.on_step
def _detent(sim):
    q = sim.states()["box_cutter"].joints["blade"]
    target = NOTCHES[1] if q >= sum(NOTCHES) / 2 else NOTCHES[0]
    if target != _notch["target"]:
        _notch["target"] = target
        sim.set_target("box_cutter", "blade", target)

EPS = 1e-6
FLOOR_Z = TOP + TRAY_FLOOR
SAFE_SHOWING = 0.002  # at most 2 mm of blade past the handle's front
FLAT_DEG = 10.0


def showing(cutter) -> float:
    """Blade length past the handle's front face (m)."""
    return BLADE_OUT - cutter.joints["blade"]


def _corners(cutter) -> np.ndarray:
    """Every shape corner of the cutter in the world, from its true pose (not its widened bounding box)."""
    poses = {"base": (cutter.pos, parts._quat_to_matrix(cutter.quat_xyzw))}
    for name, (p, q) in cutter.links.items():
        poses[name] = (p, parts._quat_to_matrix(q))
    signs = np.array([[a, b, c] for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)], float)
    return np.concatenate([c + (signs * (s / 2)) @ r.T for _, c, r, s in CUTTER.world_boxes(poses)])


def in_tray(objects) -> bool:
    """Flat (handle top up, within 10 deg), every part of it inside the tray's inner walls seen from above, its
    lowest point within 1 mm of the tray floor, and still."""
    c = objects["box_cutter"]
    r = parts._quat_to_matrix(c.quat_xyzw)
    if math.degrees(math.acos(float(np.clip(r[2, 2], -1.0, 1.0)))) > FLAT_DEG + EPS:
        return False
    pts = _corners(c)
    lo, hi = np.array(TRAY_XY) - np.array(TRAY_IN) / 2, np.array(TRAY_XY) + np.array(TRAY_IN) / 2
    if np.any(pts[:, :2] < lo - EPS) or np.any(pts[:, :2] > hi + EPS):
        return False
    low = float(pts[:, 2].min())
    return bool(FLOOR_Z - 0.003 <= low <= FLOOR_Z + 0.001 + EPS and c.resting())


def success(objects):
    """In the tray with at most 2 mm of blade showing."""
    return bool(in_tray(objects) and showing(objects["box_cutter"]) <= SAFE_SHOWING + EPS)


def distance(objects):
    return float(np.linalg.norm(objects["box_cutter"].pos[:2] - np.array(TRAY_XY)))


def score(objects):
    """1.0 in the tray with the blade retracted (at most 2 mm showing); 0 otherwise (pass/fail)."""
    return 1.0 if success(objects) else 0.0


# -- demo --------------------------------------------------------------------------------------------------------

TIP = 0.013
TRAVEL_Z = TOP + 0.12
FINGER_REACH = 0.0094  # TCP to the closed fingers' face in the push direction (offline FK on the finger hulls)
PUSH_Z = TOP + HANDLE[2] + 0.005 + TIP  # finger tips 5 mm above the handle top, at the knob
GRIP_DX = 0.025  # grip the handle this far in front of its centre, clear of the retracted knob
GRIP_Z = TOP + 0.003 + TIP  # finger tips 3 mm above the table
HALF_OPEN = FINGER_OPEN * 0.45  # opens to about 42 mm inside the tray, clear of its walls
DROP = 0.008  # the handle's underside is released this far above the tray floor


def demo():
    """Right arm: closed fingers come down in front of the thumb knob and push it back until the blade is in; then
    the arm grips the handle across its width, carries the cutter to the tray and lets go just above the floor."""
    cx, cy = CUTTER_XY
    face = cx + KNOB_X + KNOB[0] / 2
    touch = face + FINGER_REACH
    tx, ty = TRAY_XY
    place_z = GRIP_Z + TRAY_FLOOR + DROP
    right = [
        Waypoint(1.5, (touch + 0.02, cy, TRAVEL_Z), FINGER_CLOSED, "over the knob"),
        Waypoint(1.5, (touch + 0.02, cy, PUSH_Z), FINGER_CLOSED, "down in front of it"),
        Waypoint(2.0, (touch - BLADE_OUT - 0.004, cy, PUSH_Z), FINGER_CLOSED, "push the blade in"),
        Waypoint(0.8, (touch - BLADE_OUT + 0.015, cy, PUSH_Z), FINGER_CLOSED, "back off"),
        Waypoint(1.0, (cx + GRIP_DX, cy, TRAVEL_Z), FINGER_OPEN, "over the handle"),
        Waypoint(1.5, (cx + GRIP_DX, cy, GRIP_Z), FINGER_OPEN, "descend"),
        Waypoint(0.8, (cx + GRIP_DX, cy, GRIP_Z), FINGER_CLOSED, "grip"),
        Waypoint(1.2, (cx + GRIP_DX, cy, TRAVEL_Z), FINGER_CLOSED, "lift"),
        Waypoint(2.0, (tx + GRIP_DX, ty, TRAVEL_Z), FINGER_CLOSED, "carry"),
        Waypoint(1.5, (tx + GRIP_DX, ty, place_z), FINGER_CLOSED, "lower into the tray"),
        Waypoint(0.8, (tx + GRIP_DX, ty, place_z), HALF_OPEN, "release"),
        Waypoint(1.2, (tx + GRIP_DX, ty, TRAVEL_Z), HALF_OPEN, "up"),
    ]
    return {"right": right}
