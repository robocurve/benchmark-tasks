"""Fixed layout of the conveyor photo-build task. Pure Python and NumPy: no Isaac Sim.

World frame of the station: x forward from the line through the arm bases, y to the left, z up, metres.
The arm bases stand at (0, +-0.305, 0.75); the table top is z = 0.7475.
"""
from __future__ import annotations

import numpy as np

TOP = 0.7475  # station table top
FLOOR_Z = -0.0125  # top of the station's floor
TABLE_EDGE_Y = 0.65  # the 1.30 m wide table spans y = -0.65 .. +0.65
SETTLE_S = 0.5  # yam_arena settles the scene this long after a reset; the belt is already running

# -- conveyor ---------------------------------------------------------------------------------------------
BELT_X = 0.35  # belt centre line, along y
BELT_W = 0.10
BELT_T = 0.025  # the belt slab lies on the table, so its top is 25 mm up
BELT_TOP = TOP + BELT_T
BELT_UP = 1.85  # upstream end: the belt comes in over the left edge of the table
BELT_END = -0.72  # downstream end, 70 mm past the right edge of the table
SPEED = 0.04  # m/s, the belt surface runs toward -y (left to right as the robot sees it)
BELT_LEN = BELT_UP - BELT_END
BELT_CY = (BELT_UP + BELT_END) / 2

# -- parts on the belt, in the order they arrive (nearest the downstream end first) ---------------------------
PITCH = 0.20  # centre to centre along the belt: one part every 5 s
FIRST_Y = 0.35  # the first part's centre at the start of the trial
YELLOW = (0.95, 0.78, 0.10)
ORANGE = (0.95, 0.42, 0.08)
RED = (0.80, 0.12, 0.10)
BLUE = (0.15, 0.35, 0.80)
GREEN = (0.18, 0.60, 0.25)

# name: (size as it rides on the belt (m): x, y, z (a cylinder's x and y are its diameter), mass (kg), colour),
# in belt order. The model in the photo is the red cube on the yellow beam on the two tall green cylinders; each
# other part differs from one of those in a single feature: shape, colour or height.
PARTS = {
    'cap': ((0.040, 0.040, 0.040), 0.040, RED),  # red cube: the model's top
    'square_post': ((0.040, 0.040, 0.080), 0.077, GREEN),  # extra: square, not round
    'beam': ((0.040, 0.180, 0.030), 0.130, YELLOW),  # yellow beam: across the posts
    'orange_beam': ((0.040, 0.180, 0.030), 0.130, ORANGE),  # extra: orange, not yellow
    'blue_cube': ((0.040, 0.040, 0.040), 0.040, BLUE),  # extra: blue, not red
    'post_a': ((0.040, 0.040, 0.080), 0.060, GREEN),  # tall green cylinder: a post
    'short_post': ((0.040, 0.040, 0.050), 0.038, GREEN),  # extra: a green cylinder 50 mm tall, not 80
    'post_b': ((0.040, 0.040, 0.080), 0.060, GREEN),  # tall green cylinder: the other post
}
CYLINDERS = frozenset({'post_a', 'post_b', 'short_post'})
ORDER = tuple(PARTS)
NEEDED = ('post_a', 'post_b', 'beam', 'cap')
EXTRA = ('square_post', 'orange_beam', 'blue_cube', 'short_post')


def start_y(name: str) -> float:
    """A part's centre along the belt at the start of the trial (after the reset's settle)."""
    return FIRST_Y + PITCH * ORDER.index(name)


def spawn_pos(name: str) -> tuple[float, float, float]:
    """Where a part is spawned: upstream of its trial start by what the belt carries it during the settle."""
    size = PARTS[name][0]
    return (BELT_X, start_y(name) + SPEED * SETTLE_S, BELT_TOP + size[2] / 2)


def leaves_belt_s(name: str) -> float:
    """When an untouched part's centre reaches the downstream end of the belt (s after the trial starts)."""
    return (start_y(name) - BELT_END) / SPEED


# -- the gate ---------------------------------------------------------------------------------------------
GATE_X = 0.20  # the gate's centre line, parallel to the belt
GATE_Y = 0.05
MARK_GAP = 0.12  # the two post marks, centre to centre, along y
MARK = 0.044  # white square marks, 44 x 44 mm
MARK_T = 0.0005
MARKS = ((GATE_X, GATE_Y + MARK_GAP / 2), (GATE_X, GATE_Y - MARK_GAP / 2))
MARK_TOP = TOP + MARK_T
POST_H = PARTS['post_a'][0][2]
BEAM_H = PARTS['beam'][0][2]
POST_TOP = MARK_TOP + POST_H
BEAM_TOP = POST_TOP + BEAM_H

# -- staging mat: free space to park parts -------------------------------------------------------------------
STAGE = (0.20, 0.32)  # centre
STAGE_SIZE = (0.16, 0.22)
STAGE_T = 0.001
STAGE_TOP = TOP + STAGE_T

# -- downstream tote: what rides off the belt falls into it, beside the table --------------------------------
TOTE = (BELT_X, -0.78)  # centre: under the belt's end, just past the table's edge
TOTE_IN = (0.24, 0.20)  # inside, x by y
TOTE_FLOOR = 0.40  # top of its floor, 0.3475 m below the table top
TOTE_WALL_H = 0.16
TOTE_T = 0.01
LEG_W = 0.06

# -- the photo card: a picture of the finished model, leaning back 45 degrees beyond the belt --------------------
CARD_SIZE = (0.004, 0.24, 0.18)  # thickness, width (along y), height (along its slope)
CARD_TILT = np.radians(45.0)  # leaning back from vertical, its picture facing the arms and the overhead camera
CARD_CENTER = (0.534, 0.05, TOP + 0.066)  # its lower edge rests on the table about 0.47 m forward
PHOTO_FILE = 'photo.png'  # 4:3, mapped onto the card's front face

GREY = (0.55, 0.55, 0.57)
DARK = (0.12, 0.12, 0.13)
STEEL = (0.45, 0.47, 0.50)
WHITE = (0.93, 0.93, 0.92)
TOTE_BLUE = (0.20, 0.30, 0.45)


def rotation(quat_xyzw) -> np.ndarray:
    x, y, z, w = (float(v) for v in quat_xyzw)
    n = x * x + y * y + z * z + w * w
    if not np.isfinite(n) or n < 1e-12:
        raise ValueError('bad quaternion')
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
    ])


def corners(name: str, pos, quat_xyzw) -> np.ndarray:
    """The 8 corners of a box part, or of a cylinder's bounding box (world frame), from centre and orientation."""
    half = np.asarray(PARTS[name][0]) / 2
    signs = np.array([(a, b, c) for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)], dtype=float)
    return np.asarray(pos, dtype=float) + (signs * half) @ rotation(quat_xyzw).T


def z_extent(name: str, pos, quat_xyzw) -> tuple[float, float]:
    """A part's lowest and highest points (world z): exact for boxes and for cylinders at any tilt."""
    if name not in CYLINDERS:
        c = corners(name, pos, quat_xyzw)[:, 2]
        return float(c.min()), float(c.max())
    radius, height = PARTS[name][0][0] / 2, PARTS[name][0][2]
    az = abs(float(rotation(quat_xyzw)[2, 2]))  # the cylinder's axis, vertical component
    reach = height / 2 * az + radius * np.sqrt(max(0.0, 1.0 - az * az))
    return float(pos[2]) - reach, float(pos[2]) + reach


def card_quat_xyzw() -> tuple[float, float, float, float]:
    """The card's orientation: turned CARD_TILT about +y, so its top leans away from the arms."""
    return (0.0, float(np.sin(CARD_TILT / 2)), 0.0, float(np.cos(CARD_TILT / 2)))


def studio_pos(name: str) -> tuple[float, float, float]:
    """Where a part stands in the finished model (for the photo), or in the tote for an extra part."""
    gap = 0.001  # spawned this far apart; they settle together
    if name in ('post_a', 'post_b'):
        x, y = MARKS[0] if name == 'post_a' else MARKS[1]
        return (x, y, MARK_TOP + POST_H / 2 + gap)
    if name == 'beam':
        return (GATE_X, GATE_Y, POST_TOP + BEAM_H / 2 + 2 * gap)
    if name == 'cap':
        return (GATE_X, GATE_Y, BEAM_TOP + PARTS['cap'][0][2] / 2 + 3 * gap)
    i = EXTRA.index(name)
    return (TOTE[0] - 0.06 + 0.04 * i, TOTE[1], TOTE_FLOOR + PARTS[name][0][2] / 2 + gap)
