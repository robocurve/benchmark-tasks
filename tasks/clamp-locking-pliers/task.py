"""On the station table: a vertical 20 mm aluminium tube fixed upright in front of the right arm, with an orange
band 9 to 11 cm above the table; and a pair of locking pliers lying open on a two-block holder to its right. The
robot must pick up the pliers by their handles, put the jaws around the tube at the orange band, squeeze the
handles shut until the catch clicks, and let go: the pliers must stay clamped on the tube by themselves.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from partkit import Hinge, PartState, Slider, on_reset, on_step, part  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Clamp the locking pliers onto the metal tube at the orange band, level, so that they stay there on their own."

STEEL = (0.55, 0.57, 0.60)
ALU = (0.78, 0.79, 0.80)
RED = (0.75, 0.10, 0.08)
DARK = (0.22, 0.22, 0.24)
ORANGE = (1.00, 0.50, 0.00)
PAD = (0.35, 0.35, 0.37)


def _mm(x0, x1, y0, y1, z0, z1, color, rot=None):
    spec = (((x0 + x1) / 2000, (y0 + y1) / 2000, (z0 + z1) / 2000), ((x1 - x0) / 1000, (y1 - y0) / 1000,
                                                                     (z1 - z0) / 1000), color)
    return spec if rot is None else (*spec, rot)


# -- the pliers, in their own frame: origin on the pivot, x toward the jaws, y left, z up, mm ---------------------
# Shut: jaws 16 mm apart at the pad, handles 18 mm apart, 36 mm across their outer faces.
# Fixed half: upper jaw (with its sprung pad), pivot boss, lower handle, and at the handle's end a bracket carrying
# the orange spring latch, under the moving handle's end.
FIXED_BOXES = [
    _mm(8, 52, 14, 26, -9, 9, STEEL),            # fixed jaw
    _mm(-128, -4, -18, -9, -7, 7, RED),          # uniform straight fixed handle; no enlarged grip
    _mm(-142, -108, -18, 28, -9, -5.5, STEEL),   # bracket under the moving handle's end and the latch
]
FIXED_CYLS = [((0.0, 0.0, 0.0), 0.013, 0.012, STEEL)]  # pivot boss
OPEN = -0.34                 # rad: the moving half's open stop, from shut (jaws about 26 mm apart at the tube)


def _turned(spec, a):
    """A box spec turned by ``a`` about the pivot (z axis)."""
    (cx, cy, cz), size, color = spec[:3]
    c, s_ = math.cos(a), math.sin(a)
    return ((cx * c - cy * s_, cx * s_ + cy * c, cz), size, color, (math.cos(a / 2), 0.0, 0.0, math.sin(a / 2)))


# The moving half is authored open: its hinge reads 0 open and -OPEN (0.40 rad) shut.
MOVING_BOXES = tuple(_turned(b, OPEN) for b in (
    _mm(8, 52, -24, -8, -9, 9, STEEL),           # moving jaw
    _mm(-124, -4, 9, 18, -7, 7, RED),            # moving handle
))
_PIN = (-0.128, 0.0135)      # m: the round end of the moving handle (a 8 mm pin) that the latch closes over
# The handle has a uniform 9 x 14 mm rectangular cross-section along its full length.
# There is no extra grasp collar, cylinder, visual bulb or hidden enlarged collider.
_GA = OPEN
MOVING_CYLS = (((0.0, 0.0, 0.0055), 0.011, 0.002, DARK),   # rivet head
               ((_PIN[0] * math.cos(_GA) - _PIN[1] * math.sin(_GA), _PIN[0] * math.sin(_GA) + _PIN[1] * math.cos(_GA),
                 0.0), 0.004, 0.009, STEEL))
JAW_X = 32.0                 # mm: where the tube sits in the jaws
# The opening spring: 0.5 N m per rad, preloaded 1.1 rad past the open stop, so it holds the pliers open with 0.55 N m
# and takes 0.72 N m (10 N at the grips, 7 cm behind the pivot) to hold them shut.
MOVING = Hinge("moving_half", pivot=(0.0, 0.0, 0.0), boxes=MOVING_BOXES, cylinders=MOVING_CYLS, lower=0.0,
               upper=-OPEN + 0.08, start=0.0, spring_zero=-1.1, rate=0.5, damping=0.02, mass=0.06)
# Sprung jaw pad (the jaw's give): 8 N per mm; on the 20 mm tube it is pressed 4 mm, a 32 N clamp (15 N more at the
# grips).
JAW_PAD = Slider("jaw_pad", rest=(0.033, 0.011, 0.0), boxes=(_mm(16, 50, 8, 14, -8, 8, PAD),), axis="Y",
                 pressed=1.0, travel=0.008, rate=8000.0, damping=20.0, mass=0.008, friction=1.2)
# Spring latch at the end of the fixed handle: a block that slides along the handles, pushed toward the pivot by a
# spring (0.3 N per mm, 7 mm travel). Its outer corner is a 45 degree chamfer: the moving handle's round end pushes it
# back as the handles close, and once that end is past, the latch springs over it and its square face holds the
# handles shut (0.5 mm short of fully shut). Pressing the latch back releases them.
def _chamfer(p_from, p_to, thick, z0, z1, color):
    """A box whose face runs from ``p_from`` to ``p_to`` (x, y in mm); open on the left of that direction, solid on the right."""
    a, b = np.array(p_from, float), np.array(p_to, float)
    d = b - a
    alpha = math.atan2(d[1], d[0])
    n = np.array([-math.sin(alpha), math.cos(alpha)])        # left of the direction: the open side
    c = (a + b) / 2 - n * thick / 2
    return ((c[0] / 1000, c[1] / 1000, (z0 + z1) / 2000), (float(np.linalg.norm(d)) / 1000, thick / 1000,
                                                           (z1 - z0) / 1000), color,
            (math.cos(alpha / 2), 0.0, 0.0, math.sin(alpha / 2)))


CATCH = Slider("catch", rest=(-0.134, 0.022, 0.0), boxes=(
    _mm(-142, -127, 18, 21, -4.5, 4.5, ORANGE),                      # tooth: its face at y 18 holds the pin
    _chamfer((-133, 27), (-127, 21), 3.0, -4.5, 4.5, ORANGE),        # 45 degree lead-in
    _mm(-142, -133, 21, 27, -4.5, 4.5, ORANGE)),                     # body behind it
    axis="X", pressed=-1.0, travel=0.007, rate=300.0, damping=1.0, mass=0.004, friction=0.3)

PLIERS_Z = 0.040             # pliers' mid-plane above the table on the holder
PLIERS_START = (0.25, -0.24, TOP + PLIERS_Z)
PLIERS = part("pliers", pos=PLIERS_START, yaw=0.0, mass=0.09, boxes=FIXED_BOXES, cylinders=FIXED_CYLS,
              sliders=(JAW_PAD, CATCH), hinges=(MOVING,), color=STEEL, friction=1.0,
              patch_radius=0.008)

# Holder: two dark blocks, under the jaws and under the handle ends, leaving the grip clear.
HOLDER = [
    Box("holder_front", pos=(PLIERS_START[0] + 0.026, PLIERS_START[1] - 0.008, TOP + (PLIERS_Z - 0.0095) / 2),
        size=(0.05, 0.09, PLIERS_Z - 0.0095), color=DARK),
    Box("holder_back", pos=(PLIERS_START[0] - 0.118, PLIERS_START[1] + 0.005, TOP + (PLIERS_Z - 0.0095) / 2),
        size=(0.024, 0.07, PLIERS_Z - 0.0095), color=DARK),
]

# The tube: 20 mm aluminium, 16 cm tall, fixed upright; an orange band of tape 9 to 11 cm above the table.
TUBE_XY = (0.36, -0.12)
TUBE_R = 0.010
BAND = (0.09, 0.11)
TUBE = [
    Cylinder("tube_low", pos=(*TUBE_XY, TOP + BAND[0] / 2), radius=TUBE_R, height=BAND[0], color=ALU),
    Cylinder("tube_band", pos=(*TUBE_XY, TOP + sum(BAND) / 2), radius=TUBE_R, height=BAND[1] - BAND[0],
             color=ORANGE),
    Cylinder("tube_high", pos=(*TUBE_XY, TOP + (BAND[1] + 0.16) / 2), radius=TUBE_R, height=0.16 - BAND[1],
             color=ALU),
]
OBJECTS = [*TUBE, *HOLDER, PLIERS]

# -- scoring -----------------------------------------------------------------------------------------------------------
SHUT = -0.015                # rad: handle ends within 2 mm of shut (the catch holds them at 0)
_STATE: dict = {}


@on_reset
def _reset_state(environment=None) -> None:
    _STATE.update(step=0, scored=None, last=(0.0, False), recent=[], tips=[])


def _fingertips(environment):
    """World positions (relative to the env origin) of both grippers' tool centre points, between the finger pads."""
    from yam_arena.constants import TCP_OFFSET

    base = environment.unwrapped
    origin = base.scene.env_origins[0]
    origin = getattr(origin, "torch", origin).detach().cpu().numpy()
    out = []
    for key in ("left_robot", "right_robot"):
        robot = base.scene[key]
        names = list(robot.body_names)
        pose = robot.data.body_link_pose_w
        pose = getattr(pose, "torch", pose).detach().cpu().numpy()[0][names.index("gripper")]
        x, y, z, w = pose[3:7]
        R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                      [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                      [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
        out.append(pose[:3] + R @ np.array(TCP_OFFSET) - origin)
    return out


@on_step
def _count_step(environment=None) -> None:
    _STATE["step"] += 1
    if environment is not None:
        _STATE["tips"] = _fingertips(environment)


_reset_state()


def measure(pliers: PartState) -> dict:
    """What a ruler and a photo show: where the tube passes the pliers' jaws, how far shut the handles are, height of
    the jaws, tilt, lowest point, and whether a finger is on the pliers."""
    R = pliers.rotation()
    # The tube's axis where it crosses the pliers' mid-plane, in the pliers frame (mm).
    p0 = R.T @ (np.array([*TUBE_XY, TOP]) - pliers.pos)
    d = R.T @ np.array([0.0, 0.0, 1.0])
    t = -p0[2] / d[2] if abs(d[2]) > 1e-6 else 0.0
    axis = (p0 + t * d) * 1000
    between = bool(16.0 <= axis[0] <= 50.0 and -8.0 <= axis[1] <= 12.0)
    angle = pliers.hinges.get("moving_half", 0.0) + OPEN   # 0 shut, OPEN at the open stop
    jaw = pliers.to_world((JAW_X / 1000, 0.002, 0.0))
    # Lowest point of the pliers: the corners of its bounding box (both halves, open or shut, fit in it).
    lo = np.array([-0.142, -0.06, -0.009])
    hi = np.array([0.052, 0.07, 0.009])
    corners = [pliers.to_world([a, b, c]) for a in (lo[0], hi[0]) for b in (lo[1], hi[1]) for c in (lo[2], hi[2])]
    held = any(np.all(np.abs(pliers.to_local(tip) - (lo + hi) / 2) <= (hi - lo) / 2 + 0.015) for tip in _STATE["tips"])
    return dict(between=between, shut=bool(angle >= SHUT), angle=angle, jaw_z=float(jaw[2] - TOP),
                tilt=math.degrees(math.acos(min(1.0, abs(R[2, 2])))), low=float(min(c[2] for c in corners) - TOP),
                held=bool(held), axis_mm=axis)


def _update(objects) -> tuple[float, bool]:
    if _STATE["scored"] == _STATE["step"]:
        return _STATE["last"]
    _STATE["scored"] = _STATE["step"]
    pliers = objects["pliers"]
    m = measure(pliers)
    off_table = m["low"] > 0.02
    clamped = m["between"] and m["shut"] and off_table and not m["held"]
    in_band = BAND[0] <= m["jaw_z"] <= BAND[1]
    level = m["tilt"] <= 10.0
    recent = _STATE["recent"]
    recent.append(pliers.pos.copy())
    del recent[:-20]
    still = len(recent) == 20 and all(np.linalg.norm(p - pliers.pos) < 0.001 for p in recent)
    if clamped and in_band and level:
        result = (1.0, bool(still))
    elif clamped:
        result = (0.5, False)
    elif m["between"] and off_table:
        result = (0.25, False)
    else:
        result = (0.0, False)
    _STATE["last"] = result
    return result


def success(objects) -> bool:
    """Clamped on the tube by themselves (tube between the jaws, handles locked shut, off the table, no finger on
    them), jaws on the orange band, level within 10 degrees, still for the last second; held 1 s."""
    return _update(objects)[1]


def score(objects) -> float:
    """1.0 clamped by themselves on the band and level; 0.5 clamped by themselves elsewhere or tilted; 0.25 the tube
    between the jaws with the pliers off the table (still held); else 0."""
    return _update(objects)[0]


