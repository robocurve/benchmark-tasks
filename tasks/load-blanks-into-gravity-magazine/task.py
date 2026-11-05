"""On the station table, in front of the left arm: a fixed steel gravity magazine, a square vertical channel 46 x 46
mm inside and 90 mm tall, open at the top and standing on the table, with a 12 mm sight slot down the middle of
its near wall. Nearer the robot's left, four loose aluminium blanks, 41 x 41 x 15 mm, lie flat in a row on a thin
blue tray pad, each turned a different amount about the vertical.
"""

import math

import numpy as np

from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Magazine. World frame: x away from the arm bases, y to the robot's left, z up (m).
MAG_XY = (0.30, 0.12)
INNER = 0.046  # inside width, both ways
WALL = 0.004
MAG_H = 0.090
RIM = TOP + MAG_H
SLOT = 0.012  # sight slot in the near (-x) wall: narrower than a blank is thick, so none passes through it
STEEL = (0.55, 0.57, 0.60)

_mx, _my = MAG_XY
_outer = INNER + 2 * WALL
_strip = (_outer - SLOT) / 2  # each side of the slot
MAGAZINE = [
    Box("mag_far_wall", pos=(_mx + INNER / 2 + WALL / 2, _my, TOP + MAG_H / 2), size=(WALL, _outer, MAG_H), color=STEEL),
    Box("mag_left_wall", pos=(_mx, _my + INNER / 2 + WALL / 2, TOP + MAG_H / 2), size=(INNER, WALL, MAG_H), color=STEEL),
    Box("mag_right_wall", pos=(_mx, _my - INNER / 2 - WALL / 2, TOP + MAG_H / 2), size=(INNER, WALL, MAG_H), color=STEEL),
    # The near wall is two strips with the sight slot between them.
    Box("mag_near_left", pos=(_mx - INNER / 2 - WALL / 2, _my + _outer / 2 - _strip / 2, TOP + MAG_H / 2),
        size=(WALL, _strip, MAG_H), color=STEEL),
    Box("mag_near_right", pos=(_mx - INNER / 2 - WALL / 2, _my - _outer / 2 + _strip / 2, TOP + MAG_H / 2),
        size=(WALL, _strip, MAG_H), color=STEEL),
]

# Tray pad and blanks.
PAD_H = 0.003
PAD_TOP = TOP + PAD_H
BLANK = (0.041, 0.041, 0.015)  # 15 mm thick: its 43.7 mm diagonal cannot wedge across the 46 mm channel
BLANK_MASS = 0.068  # aluminium, 41 x 41 x 15 mm
ROW_Y = 0.25
BLANK_X = (0.13, 0.20, 0.27, 0.34)
BLANK_YAW_DEG = (20.0, -15.0, 30.0, -25.0)
PAD = Box("tray_pad", pos=(0.235, ROW_Y, TOP + PAD_H / 2), size=(0.30, 0.09, PAD_H), color=(0.20, 0.30, 0.70))
ALU = (0.80, 0.81, 0.83)
BLANKS = [
    Box(f"blank_{i + 1}", pos=(x, ROW_Y, PAD_TOP + BLANK[2] / 2), size=BLANK, mass=BLANK_MASS, color=ALU,
        yaw=math.radians(yaw))
    for i, (x, yaw) in enumerate(zip(BLANK_X, BLANK_YAW_DEG))
]
OBJECTS = [*MAGAZINE, PAD, *BLANKS]
NAMES = [b.name for b in BLANKS]

TILT_MAX_DEG = 5.0
CENTRE_TOL = (INNER - BLANK[0]) / 2 + 0.0005  # 3.0 mm: the centre over the channel's inside
RIM_TOL = 0.001  # wholly inside: its top no higher than 1 mm above the rim
SUPPORT_TOL = 0.002  # its bottom within 2 mm of the table or of a loaded blank's top


def _tilt_deg(st) -> float:
    """Angle of the blank's 15 mm side from vertical, either face up."""
    x, y, _, _ = (float(v) for v in st.quat_xyzw)
    return float(np.degrees(np.arccos(np.clip(abs(1 - 2 * (x * x + y * y)), 0.0, 1.0))))


def _inside_flat(st) -> bool:
    """In the channel (centre within 3 mm of its centre line both ways, top no more than 1 mm above the rim), lying
    flat (15 mm side within 5 deg of vertical, either face up) and at rest."""
    return bool(
        abs(st.pos[0] - _mx) <= CENTRE_TOL
        and abs(st.pos[1] - _my) <= CENTRE_TOL
        and st.high[2] <= RIM + RIM_TOL
        and _tilt_deg(st) <= TILT_MAX_DEG
        and st.resting()
    )


def loaded_names(objects) -> list[str]:
    """Blanks that are inside and flat, and supported: the lowest point of each within 2 mm of the table top or of
    the top of a blank below it that is itself loaded. Built from the bottom of the stack up."""
    cands = sorted((n for n in NAMES if _inside_flat(objects[n])), key=lambda n: float(objects[n].low[2]))
    tops, out = [TOP], []
    for n in cands:
        low = float(objects[n].low[2])
        if any(abs(low - t) <= SUPPORT_TOL for t in tops):
            out.append(n)
            tops.append(float(objects[n].high[2]))
    return out


def loaded(st) -> bool:
    """A single blank on its own: inside, flat, at rest and on the magazine floor (used for checks)."""
    return _inside_flat(st) and abs(float(st.low[2]) - TOP) <= SUPPORT_TOL


def count(objects) -> int:
    return len(loaded_names(objects))


def success(objects):
    """All four blanks inside the magazine, flat and at rest."""
    return count(objects) == len(NAMES)


def distance(objects):
    done = set(loaded_names(objects))
    return float(sum(np.linalg.norm(objects[n].pos[:2] - np.asarray(MAG_XY)) for n in NAMES if n not in done))


def score(objects):
    """n / 4, where n counts the blanks inside the magazine, flat, supported and at rest (1.0 when all four are)."""
    return count(objects) / len(NAMES)


# ---- scripted demo (left arm) -------------------------------------------------------------------------------
# Grip each blank across two faces with the TCP 15 mm above its bottom (finger tips 2 mm above it), turning the
# grip to the blank's own yaw; carry it square (yaw 0) and let go with its bottom 10 mm above the rim.
OPEN_PART = -0.030  # pads 60 mm apart around the 41 mm blank
# Each finger joint is position-controlled with its force capped at 50 N. Commanded fully closed on a 41 mm blank,
# both fingers sit at the cap and the clamped blank can slide sideways in the grip while it is carried (traces
# showed 15 mm). Commanded 2 mm into each face (pads 37 mm apart) each finger pushes about 10 N, below the cap, so
# the grip stays centred on the TCP.
GRIP = -(BLANK[0] / 2 - 0.002)
GRIP_ABOVE_BOTTOM = 0.015
TRAVEL_Z = TOP + 0.135
RELEASE_Z = RIM + 0.010 + GRIP_ABOVE_BOTTOM
# Aim offset (TCP minus wanted blank centre) to cancel the left arm's tracking error while carrying; from traces.
AIM = (0.0, 0.0)


def _load(b: Box) -> list[Waypoint]:
    px, py, _ = b.pos
    qx, qy = _mx + AIM[0], _my + AIM[1]
    pick_z = PAD_TOP + GRIP_ABOVE_BOTTOM
    return [
        Waypoint(1.5, (px, py, TRAVEL_Z), OPEN_PART, f"over {b.name}", b.yaw),
        Waypoint(1.2, (px, py, pick_z), OPEN_PART, "descend", b.yaw),
        Waypoint(0.8, (px, py, pick_z), GRIP, "grip", b.yaw),
        Waypoint(1.0, (px, py, TRAVEL_Z), GRIP, "lift", b.yaw),
        Waypoint(2.0, (qx, qy, TRAVEL_Z), GRIP, "carry", 0.0),
        Waypoint(1.0, (qx, qy, RELEASE_Z), GRIP, "lower over the channel", 0.0),
        Waypoint(0.8, (qx, qy, RELEASE_Z), GRIP, "settle", 0.0),
        Waypoint(0.5, (qx, qy, RELEASE_Z), OPEN_PART, "release", 0.0),
        Waypoint(0.8, (qx, qy, TRAVEL_Z), OPEN_PART, "up", 0.0),
    ]


def demo():
    """Left arm: each blank in turn, nearest the magazine first."""
    return {"left": [w for b in sorted(BLANKS, key=lambda b: abs(b.pos[0] - _mx)) for w in _load(b)]}
