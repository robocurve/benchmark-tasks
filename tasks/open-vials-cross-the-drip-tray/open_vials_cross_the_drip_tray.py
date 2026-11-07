"""On the station table: a grey drip tray (14 x 40 cm, with a low dark lip) runs left to right between the arms. Two
amber open vials stand at its left end; two white pads mark its right end. The vials are loose, 4 cm across and 7 cm
tall. The left arm cannot reach the right end and the right arm cannot reach the left end, so each vial has to be
passed from one arm to the other near the middle of the tray.

Requires -E object_jitter=0: the scorer finds the start of a trial from the vials' start spots.

Instruction: Move both open vials from the left end of the grey drip tray onto the white pads at its right end.

Run it:  inspect-robots run --embodiment yam_arena -E task=open_vials_cross_the_drip_tray.py -E object_jitter=0 \\
           --instruction "$INSTR" --scorer task_score ...
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
from belimits import Rect, ResetOnJumpTracker, bottom, inside, is_airborne, is_flat_on  # noqa: E402

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from closedloop import placeholder_demo  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
SUCCESS_HOLD_S = 6.0  # long enough that the demo opens the gripper and clears the arm before the run stops

GREY, DARK, WHITE, AMBER = (0.55, 0.57, 0.60), (0.30, 0.32, 0.36), (0.95, 0.95, 0.93), (0.75, 0.45, 0.15)

# The drip tray: a fixed 4 mm plate, footprint x 0.17..0.31, y -0.20..+0.20, with a 6 mm thick, 12 mm tall lip of four
# fixed boxes just outside the footprint (a polypropylene lab tray in the real world).
TRAY = Rect((0.17, 0.31), (-0.20, 0.20))
TRAY_H, LIP_T, LIP_H = 0.004, 0.006, 0.012
_CX, _CY = 0.24, 0.0
PLATE = Box("tray_plate", pos=(_CX, _CY, TOP + TRAY_H / 2), size=(0.14, 0.40, TRAY_H), color=GREY)
LIP = [
    Box("lip_front", pos=(TRAY.x0 - LIP_T / 2, _CY, TOP + LIP_H / 2), size=(LIP_T, 0.40 + 2 * LIP_T, LIP_H), color=DARK),
    Box("lip_back", pos=(TRAY.x1 + LIP_T / 2, _CY, TOP + LIP_H / 2), size=(LIP_T, 0.40 + 2 * LIP_T, LIP_H), color=DARK),
    Box("lip_left", pos=(_CX, TRAY.y1 + LIP_T / 2, TOP + LIP_H / 2), size=(0.14, LIP_T, LIP_H), color=DARK),
    Box("lip_right", pos=(_CX, TRAY.y0 - LIP_T / 2, TOP + LIP_H / 2), size=(0.14, LIP_T, LIP_H), color=DARK),
]

# The pads: white, r 3 cm, 2 mm thick, on the tray at its right end. Top TOP + 0.006.
PAD_R, PAD_H = 0.03, 0.002
PAD_TOP = TOP + TRAY_H + PAD_H
PAD_XY = ((0.21, -0.16), (0.27, -0.16))
PADS = [
    Cylinder(f"pad_{i}", pos=(*xy, TOP + TRAY_H + PAD_H / 2), radius=PAD_R, height=PAD_H, color=WHITE)
    for i, xy in enumerate(PAD_XY)
]

# The vials: amber, r 2 cm, 7 cm tall, 50 g, standing on the tray at its left end, 2 cm inside the edges.
VIAL_R, VIAL_H = 0.02, 0.07
VIAL_Z = TOP + TRAY_H + VIAL_H / 2
VIALS = [
    Cylinder("vial_a", pos=(0.21, 0.16, VIAL_Z), radius=VIAL_R, height=VIAL_H, mass=0.05, color=AMBER),
    Cylinder("vial_b", pos=(0.27, 0.16, VIAL_Z), radius=VIAL_R, height=VIAL_H, mass=0.05, color=AMBER),
]

OBJECTS = [PLATE, *LIP, *PADS, *VIALS]

EDGE_TOL = 0.002  # m: a vial's outline may stand this far past the tray's edge
OFF_TABLE = TOP - 0.05  # a vial's underside below this has fallen off
ON_PAD_R = 0.02  # vial centre within this of a pad centre
SUPPORT_TOL = 0.01  # underside within this of the pad top (more than this above it is held or stacked)
LIFT = TOP + TRAY_H + 0.03  # a vial counts as carried only after its underside has been this high on each side
SIDE = 0.06  # |y| beyond which the lift is on the left (y >= SIDE) or right (y <= -SIDE) half

# Latches over the whole run, kept across control steps and cleared when a real trial reset is seen.
_track = ResetOnJumpTracker({v.name: v.pos for v in VIALS})


def _update(objects) -> dict:
    values = {"breach": 0.0}
    for v in VIALS:
        s = objects[v.name]
        if not inside(s, v, TRAY.grown(EDGE_TOL)) or bottom(s, v) < OFF_TABLE:
            values["breach"] = 1.0
        up = bottom(s, v) >= LIFT
        values[v.name + "_left"] = float(up and s.pos[1] >= SIDE)
        values[v.name + "_right"] = float(up and s.pos[1] <= -SIDE)
    return _track.update(objects, values)


def _on_pad(s, spec) -> bool:
    near = any(np.hypot(s.pos[0] - x, s.pos[1] - y) <= ON_PAD_R for x, y in PAD_XY)
    return bool(near and not is_airborne(s, spec, (PAD_TOP,), SUPPORT_TOL) and is_flat_on(s, spec, PAD_TOP, SUPPORT_TOL, 15.0) and s.resting())


def score(objects):
    """0 if either vial's outline was ever outside the tray (at any height) or a vial fell off. Otherwise 0.5 per vial
    resting upright on a pad that has been lifted clear of the tray on both halves. A held vial is airborne, so it never
    counts as placed."""
    peaks = _update(objects)
    if peaks.get("breach", 0.0) > 0:
        return 0.0
    total = 0.0
    for v in VIALS:
        carried = peaks.get(v.name + "_left", 0.0) > 0 and peaks.get(v.name + "_right", 0.0) > 0
        if carried and _on_pad(objects[v.name], v):
            total += 0.5
    return total


def success(objects):
    return score(objects) == 1.0


# The robot keeps clear too (checked offline on the demo with the URDF; the scorer only sees objects): no part of either arm
# goes low in the strips of table in front of and behind the tray, where a gripped vial would be outside its edges.
ARM_KEEP_OUT = [
    {"name": "table in front of the tray", "x": (0.09, 0.15), "y": (-0.22, 0.22), "z_top": TOP + 0.25},
    {"name": "table behind the tray", "x": (0.33, 0.45), "y": (-0.22, 0.22), "z_top": TOP + 0.25},
]

# -- the demo ------------------------------------------------------------------------------------------------------
# Closed loop (closedloop.py): the right arm grasps each vial where it really stands in the hand-off strip and sets it
# down on the pad allowing for where it sits in the fingers. In the offline dry run the vials read as at their start, so
# a reading far from the strip is ignored and the nominal spot is used.
STAGE_Z, CARRY_Z, GRASP_Z, HAND_Z, PAD_Z = TOP + 0.19, TOP + 0.17, TOP + 0.039, TOP + 0.042, TOP + 0.044
HAND = (0.24, 0.0)
STAGE_L, STAGE_R = (0.24, 0.24, STAGE_Z), (0.24, -0.24, STAGE_Z)
PARK_L = (0.27, 0.16, CARRY_Z)  # over the second vial's start spot, 16 cm from the strip
SPEED = 0.2  # m/s, peak commanded TCP speed (the smooth profile peaks at 1.5 times a segment's average)
demo = placeholder_demo


def _go(arm, xyz, finger):
    d = float(np.linalg.norm(np.array(xyz, float) - arm.pos))
    yield from arm.move(xyz, max(0.8, 1.5 * d / SPEED), finger)


def _near(ctx, name, nominal, tol=0.03):
    """The vial's live (x, y) if it stands within ``tol`` of ``nominal``, else the nominal spot."""
    p = ctx.pos(name)[:2]
    return (float(p[0]), float(p[1])) if np.hypot(*(p - np.array(nominal))) <= tol else tuple(nominal)


def _pick(ctx, arm, name, nominal):
    yield from _go(arm, (*nominal, CARRY_Z), FINGER_OPEN)
    xy = _near(ctx, name, nominal)
    yield from _go(arm, (*xy, CARRY_Z), FINGER_OPEN)
    yield from _go(arm, (*xy, GRASP_Z), FINGER_OPEN)
    yield from arm.grip(FINGER_CLOSED, 0.8)
    yield from _go(arm, (*xy, CARRY_Z), FINGER_CLOSED)


def _set_down(arm, xy, z):
    yield from _go(arm, (*xy, z), FINGER_CLOSED)
    yield from arm.grip(FINGER_OPEN, 0.8)
    yield from _go(arm, (*xy, CARRY_Z), FINGER_OPEN)


def _left_hands_over(ctx, name, nominal):
    L = ctx.left
    yield from _pick(ctx, L, name, nominal)
    yield from _go(L, (*HAND, CARRY_Z), FINGER_CLOSED)
    yield from _set_down(L, HAND, HAND_Z)
    yield from _go(L, PARK_L, FINGER_OPEN)


def _right_takes(ctx, name, pad_xy):
    """Over the strip, grasp the vial where it stands, carry it over the pad (allowing for its offset in the fingers)."""
    R = ctx.right
    yield from _go(R, (*HAND, CARRY_Z), FINGER_OPEN)
    xy = _near(ctx, name, HAND)
    yield from _go(R, (*xy, CARRY_Z), FINGER_OPEN)
    yield from _go(R, (*xy, GRASP_Z), FINGER_OPEN)
    yield from R.grip(FINGER_CLOSED, 0.8)
    yield from _go(R, (*xy, CARRY_Z), FINGER_CLOSED)
    held = ctx.pos(name)[:2] - np.asarray(R.measured_pos)[:2]
    off = held if np.hypot(*held) < 0.03 else np.zeros(2)
    return (pad_xy[0] - off[0], pad_xy[1] - off[1])


def demo_script(ctx):
    """Left arm: vial a up and over to the hand-off spot (0.24, 0), set down; vial b fetched and held over its own spot
    until the right arm has taken vial a away, then handed over the same way. Right arm: takes each vial from the spot to
    a pad. Only one arm is ever low in the shared strip, the other waits 16 cm or more away, and both vials stay over the
    tray the whole time."""
    L, R = ctx.left, ctx.right
    a, b = VIALS[0], VIALS[1]
    # A: left hands vial a over; right waits at the tray's right end.
    yield from ctx.both(_go_stage(L, STAGE_L, a, ctx), _go(R, STAGE_R, FINGER_OPEN))
    # B: right takes vial a to the first pad; left fetches vial b and holds it over its spot.
    drop = {}

    def right_b():
        drop["a"] = yield from _right_takes(ctx, a.name, PAD_XY[0])

    def left_b():
        yield from _pick(ctx, L, b.name, b.pos[:2])

    yield from ctx.both(left_b(), right_b())
    # C: right sets vial a on the pad while left hands vial b over.
    def right_c():
        yield from _set_down(R, drop["a"], PAD_Z)

    def left_c():
        yield from _go(L, (*HAND, CARRY_Z), FINGER_CLOSED)
        yield from _set_down(L, HAND, HAND_Z)
        yield from _go(L, PARK_L, FINGER_OPEN)

    yield from ctx.both(left_c(), right_c())
    # D: right takes vial b to the second pad and clears; left parks over its end.
    def right_d():
        xy = yield from _right_takes(ctx, b.name, PAD_XY[1])
        yield from _set_down(R, xy, PAD_Z)
        yield from _go(R, STAGE_R, FINGER_OPEN)

    yield from ctx.both(_go(L, STAGE_L, FINGER_OPEN), right_d())
    yield from ctx.wait(1.5)


def _go_stage(L, stage, vial, ctx):
    yield from _go(L, stage, FINGER_OPEN)
    yield from _left_hands_over(ctx, vial.name, vial.pos[:2])
