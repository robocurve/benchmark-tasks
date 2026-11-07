"""Every dimension of the keyed-plug task in one place. Pure Python, no simulator needed.

World frame (the station's): x forward from the arm bases, y left, z up, metres. The left arm base is at
(0, +0.305), the right at (0, -0.305). "Block frame" and "plug frame" are the parts' own frames; their origins
are given below. A box is ``(centre, size, colour)`` in its part's frame (x, y, z edge lengths, sRGB colour).
"""

from __future__ import annotations

import math

TOP = 0.7475  # the station table top (yam_arena.task_file.TABLE_TOP["station"])

GREY = (0.62, 0.63, 0.65)
DARK = (0.12, 0.12, 0.13)
STEEL = (0.78, 0.79, 0.80)
ALUMINIUM = (0.74, 0.76, 0.78)
RED = (0.85, 0.15, 0.12)

CLEAR = 0.001  # radial play between the plug's nose and the socket cavity, per side (m); 0.5 mm was tighter than an
#                open-loop demo can hold (the block shifts about 1 mm in the grip)

# -- the loose block: origin at its centre; the socket sits on its -y face and opens along -y ------------------------
BLOCK_SIZE = (0.070, 0.110, 0.060)  # 70 mm across: the open gripper (about 94 mm) clears it by 12 mm a side
BLOCK_MASS = 0.30
BLOCK_FRICTION = 0.25  # on a smooth sheet: a 0.30 kg block then needs under 1 N to start sliding
BLOCK_XY = (0.28, 0.08)
# The whole block is turned -90 degrees about the vertical, so its socket (which opens along the block's own -y) faces
# the arms: the plug is pushed straight forward (+x) and the two hands end up apart, which the arms need (a layout
# search with the simulator's collision hulls found this one keeps the arms clear).
BLOCK_YAW = -math.pi / 2
BLOCK_POS = (*BLOCK_XY, TOP + BLOCK_SIZE[2] / 2)  # block bottom on the table
FACE_Y = -BLOCK_SIZE[1] / 2

FLANGE_W, FLANGE_H, FLANGE_T = 0.056, 0.042, 0.006
SHELL_HX, SHELL_HZ, SHELL_L = 0.019, 0.015, 0.024
FLOOR_Y = FACE_Y - FLANGE_T  # cavity floor (the flange's outer face)
MOUTH_Y = FLOOR_Y - SHELL_L  # socket mouth, block frame

# -- the plug: origin at the centre of the plane where the nose meets the housing; the nose points along +y ----------
NOSE_W, NOSE_H, NOSE_L = 0.028, 0.016, 0.018
HOUSING = (0.045, 0.018, 0.028)  # x, y (length), z
KEY_W, KEY_H, KEY_X = 0.006, 0.003, 0.001
PLUG_MASS = 0.049
PLUG_FRICTION = 0.8
PLUG_XY0 = (0.18, -0.12)
PLUG_YAW0 = BLOCK_YAW - math.pi / 4  # the nose starts turned 45 degrees away from the socket axis
PLUG_Z0 = TOP + HOUSING[2] / 2  # lying on its housing, key up
SEAT_Z = TOP + BLOCK_SIZE[2] / 2  # height of the socket axis, and so of the seated plug's origin

# -- socket cavity (block frame) -----------------------------------------------------------------------------------
CAV_HX = NOSE_W / 2 + CLEAR
CAV_HZ = NOSE_H / 2 + CLEAR
SLOT_X0 = KEY_X - KEY_W / 2 - CLEAR
SLOT_X1 = KEY_X + KEY_W / 2 + CLEAR
SLOT_Z1 = NOSE_H / 2 + KEY_H + CLEAR

# -- spring plunger (the retention detent): a round tip on the cavity's -x wall, 1 mm proud of it ------------------
PLUNGER_S = 0.010  # distance from the mouth along the cavity
PLUNGER_R, PLUNGER_H = 0.002, 0.006
PLUNGER_PROUD = CLEAR + 0.0005  # always presses the nose back 0.5 mm, so the latch clicks whatever the play is
PLUNGER_TRAVEL = 0.0015
PLUNGER_RATE = 12000.0  # N/m: 6 N to press it back 0.5 mm, a push firm enough to slide an unbraced block
PLUNGER_DAMPING = 40.0
PLUNGER_MASS = 0.02  # kg; 2 g on a 12 N/mm spring would ring above the 240 Hz physics rate, and sit 150:1 to the block
GROOVE = (0.0075, 0.0125)  # along the nose from its root (m); the plunger rests in it when the plug is seated
GROOVE_DEPTH = 0.0015


def _box(x0, x1, y0, y1, z0, z1, colour):
    return (((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0), colour)


def block_boxes():
    """The block, its flange and the socket's walls (block frame)."""
    h = BLOCK_SIZE[1] / 2
    shell_y0, shell_y1 = MOUTH_Y, FLOOR_Y
    return [
        _box(-BLOCK_SIZE[0] / 2, BLOCK_SIZE[0] / 2, -h, h, -BLOCK_SIZE[2] / 2, BLOCK_SIZE[2] / 2, GREY),
        _box(-FLANGE_W / 2, FLANGE_W / 2, FACE_Y - FLANGE_T, FACE_Y, -FLANGE_H / 2, FLANGE_H / 2, ALUMINIUM),
        _box(-SHELL_HX, -CAV_HX, shell_y0, shell_y1, -SHELL_HZ, SHELL_HZ, DARK),  # left wall
        _box(CAV_HX, SHELL_HX, shell_y0, shell_y1, -SHELL_HZ, SHELL_HZ, DARK),  # right wall
        _box(-CAV_HX, CAV_HX, shell_y0, shell_y1, -SHELL_HZ, -CAV_HZ, DARK),  # bottom wall
        _box(-CAV_HX, SLOT_X0, shell_y0, shell_y1, CAV_HZ, SHELL_HZ, DARK),  # top wall, left of the key slot
        _box(SLOT_X1, CAV_HX, shell_y0, shell_y1, CAV_HZ, SHELL_HZ, DARK),  # top wall, right of the key slot
        _box(SLOT_X0, SLOT_X1, shell_y0, shell_y1, SLOT_Z1, SHELL_HZ, DARK),  # cap over the key slot
    ]


def plunger_slider_spec():
    """The plunger as ``(rest, cylinders)``: a round tip, axis vertical, pressed back along -x (block frame)."""
    cx = -CAV_HX - PLUNGER_R + PLUNGER_PROUD  # tip's inner edge sits PLUNGER_PROUD inside the cavity wall
    centre = (cx, MOUTH_Y + PLUNGER_S, 0.0)
    return centre, ((centre, PLUNGER_R, PLUNGER_H, ALUMINIUM, "Z"),)


def plug_boxes():
    """The plug: housing, a nose in three pieces (the middle one has the groove), and the key rib (plug frame)."""
    g0, g1 = GROOVE
    hx = NOSE_W / 2
    return [
        _box(-HOUSING[0] / 2, HOUSING[0] / 2, -HOUSING[1], 0.0, -HOUSING[2] / 2, HOUSING[2] / 2, DARK),
        _box(-hx, hx, 0.0, g0, -NOSE_H / 2, NOSE_H / 2, STEEL),  # root
        _box(-hx + GROOVE_DEPTH, hx, g0, g1, -NOSE_H / 2, NOSE_H / 2, STEEL),  # groove, on the plunger's side (-x)
        _box(-hx, hx, g1, NOSE_L, -NOSE_H / 2, NOSE_H / 2, STEEL),  # tip
        _box(KEY_X - KEY_W / 2, KEY_X + KEY_W / 2, 0.0, NOSE_L, NOSE_H / 2, NOSE_H / 2 + KEY_H, RED),  # key rib
    ]


def seated_plug_origin_in_block_frame():
    """Where the plug's origin sits, in the block frame, when fully seated (housing face on the socket mouth)."""
    return (0.0, MOUTH_Y, 0.0)


def block_to_world_xy(local_xy, block_xy=None, yaw=None):
    """A point given in the block frame (x, y; metres) as a world x, y."""
    bx, by = BLOCK_XY if block_xy is None else block_xy
    dx, dy = rotate_xy(local_xy, BLOCK_YAW if yaw is None else yaw)
    return (bx + dx, by + dy)


def yaw_matrix(yaw: float):
    c, s = math.cos(yaw), math.sin(yaw)
    return ((c, -s), (s, c))


def rotate_xy(vec, yaw: float):
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * vec[0] - s * vec[1], s * vec[0] + c * vec[1])
