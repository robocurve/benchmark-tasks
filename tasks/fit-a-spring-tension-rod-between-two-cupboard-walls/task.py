"""On the station table, centred between the arms: two white cupboard walls standing parallel, front to back, with
a clear gap between their inner faces. A short yellow band is marked on the front edge of each wall. In front of
the walls, a spring tension rod lies across two small grey risers, pointing across the gap: a blue square tube
with a black rubber end cap, and a grey inner bar with a second black cap that slides into the tube against a
spring. The rod is longer than the gap; squeezed, it gets shorter.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402

import parts  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# World frame: x away from the arm bases, y to the robot's left, z up (m). Heights below are above the table top.
WALL_X0, WALL_X1 = 0.250, 0.420  # front and back edges of both walls
WALL_CX = (WALL_X0 + WALL_X1) / 2
GAP = 0.300  # between the inner faces
WALL_T, WALL_H = 0.020, 0.200
INNER = GAP / 2  # inner faces at y = +-0.150
BAND_Z0, BAND_Z1 = 0.120, 0.150  # the yellow band on each wall's front edge

WHITE, YELLOW, BLUE, GREY, BLACK = (0.94, 0.94, 0.92), (0.95, 0.80, 0.10), (0.15, 0.30, 0.80), (0.6, 0.6, 0.62), (
    0.08, 0.08, 0.08)

CUPBOARD = parts.Part(
    "cupboard",
    pos=(WALL_CX, 0.0, TOP),
    shapes=[
        *(parts.box((WALL_X1 - WALL_X0, WALL_T, WALL_H), at=(0.0, s * (INNER + WALL_T / 2), WALL_H / 2), color=WHITE)
          for s in (1, -1)),
        # The band: a 2 mm proud yellow strip on each wall's front edge (the face toward the robot).
        *(parts.box((0.002, WALL_T, BAND_Z1 - BAND_Z0),
                    at=(-(WALL_X1 - WALL_X0) / 2 - 0.001, s * (INNER + WALL_T / 2), (BAND_Z0 + BAND_Z1) / 2),
                    color=YELLOW)
          for s in (1, -1)),
    ],
    friction=(0.6, 0.5),  # painted board
)

# The rod, written at its free length (joint 0) in its own frame: origin at its middle, axis along y.
TUBE, BAR, CAP = (0.025, 0.180, 0.025), (0.018, 0.170, 0.018), (0.030, 0.010, 0.030)
FREE = 0.320  # cap face to cap face
TRAVEL = 0.050  # squeezed fully: 270 mm
SPRING = 70.0  # N/m
TUBE_Y0 = FREE / 2 - CAP[1] - TUBE[1]  # tube from -0.030 to +0.150, its cap beyond (+y)
BAR_Y1 = -FREE / 2 + CAP[1] + BAR[1]  # bar from -0.150 to +0.020, its cap beyond (-y)
ROD_START = (0.170, 0.0, TOP + 0.030 + TUBE[2] / 2)  # tube resting on its riser, 30 mm up
SLIDE = parts.Slider(
    "slide",
    shapes=[
        parts.box(BAR, at=(0.0, (BAR_Y1 + -FREE / 2 + CAP[1]) / 2, 0.0), color=GREY),
        parts.box(CAP, at=(0.0, -FREE / 2 + CAP[1] / 2, 0.0), color=BLACK),
    ],
    axis=(0.0, 1.0, 0.0),  # squeezing moves the bar and its cap into the tube (+y)
    travel=TRAVEL,
    mass=0.04,
    spring=SPRING,
    damping=1.0,
)
ROD = parts.Part(
    "rod",
    pos=ROD_START,
    shapes=[
        parts.box(TUBE, at=(0.0, TUBE_Y0 + TUBE[1] / 2, 0.0), color=BLUE),
        parts.box(CAP, at=(0.0, FREE / 2 - CAP[1] / 2, 0.0), color=BLACK),
    ],
    joints=[SLIDE],
    mass=0.06,  # tube and its cap; with the bar and cap, 100 g
    friction=(1.0, 0.8),  # rubber caps
)
RISER_XY = ((0.170, 0.030), (0.170, -0.060))  # under the tube, and under the bar clear of the tube
RISERS = [
    Box("riser_tube", pos=(*RISER_XY[0], TOP + 0.015), size=(0.040, 0.020, 0.030), color=GREY),
    Box("riser_bar", pos=(*RISER_XY[1], TOP + (0.030 + (TUBE[2] - BAR[2]) / 2) / 2),
        size=(0.040, 0.020, 0.030 + (TUBE[2] - BAR[2]) / 2), color=GREY),
]
OBJECTS = [CUPBOARD, ROD, *RISERS]

SUCCESS_HOLD_S = 4.0
TOUCH = 0.0003  # m: a cap face within 0.3 mm of a wall's inner face touches it (spring-loaded contact, not a near gap)
LEVEL = 0.010  # m: cap centres within 10 mm of the same height for full credit
SQUARE = 0.010  # m: cap centres within 10 mm of each other front to back for full credit


def _caps(rod) -> tuple[np.ndarray, np.ndarray]:
    """World centres of the tube's cap (+y end) and the bar's cap (-y end)."""
    poses = {"base": (rod.pos, parts._quat_to_matrix(rod.quat_xyzw))}
    if "slide" in rod.links:
        lp, lq = rod.links["slide"]
        poses["slide"] = (lp, parts._quat_to_matrix(lq))
    boxes = ROD.world_boxes(poses)
    caps = [c for _, c, _, s in boxes if np.allclose(s, CAP)]
    return caps[0], caps[1]


def _held_across(rod) -> bool:
    """Both caps against the walls' inner faces and fully on them: the rod spans the gap and nothing else holds it
    there (there is nothing between the walls but the table)."""
    a, b = _caps(rod)
    half = CAP[0] / 2
    return bool(
        INNER - TOUCH - 1e-6 <= rod.high[1] <= INNER + TOUCH + 1e-6
        and -INNER - TOUCH - 1e-6 <= rod.low[1] <= -INNER + TOUCH + 1e-6
        and all(WALL_X0 + half - 1e-6 <= c[0] <= WALL_X1 - half + 1e-6 for c in (a, b))
        and all(c[2] + CAP[2] / 2 <= TOP + WALL_H + 1e-6 for c in (a, b))
    )


def _heights(rod):
    a, b = _caps(rod)
    return a[2] - TOP, b[2] - TOP


def success(objects):
    rod = objects["rod"]
    if not (_held_across(rod) and rod.resting()):
        return False
    za, zb = _heights(rod)
    a, b = _caps(rod)
    return bool(
        all(BAND_Z0 - 1e-6 <= z <= BAND_Z1 + 1e-6 for z in (za, zb))
        and abs(za - zb) <= LEVEL + 1e-6
        and abs(a[0] - b[0]) <= SQUARE + 1e-6
    )


def score(objects):
    """1.0 fitted level and square in the band; else 0 (pass/fail)."""
    return 1.0 if success(objects) else 0.0


# -- demo: both arms grip the rod near its ends (fingers closing across it), squeeze it shorter, lift it to the band,
# carry it straight back between the walls and let go together.
TIP = 0.013
YAW = math.pi / 2  # fingers close along x, across the rod (turned -90 deg on the left arm, +90 on the right)
GRIP_Y = 0.130  # left arm on the tube at +y, right arm on the bar at -y
SQUEEZE = 0.030  # each arm moves this far inward; the arms give, so the rod shortens by less than 60 mm
SAG_Z = 0.004
FIT_Z = (BAND_Z0 + BAND_Z1) / 2  # rod axis height when fitted
TCP_ABOVE_AXIS = TIP - (TUBE[2] / 2 - 0.003)  # pad tips 3 mm above the tube's underside


def demo():
    x0, z0 = ROD_START[0], ROD_START[2]
    grip_z = z0 + TCP_ABOVE_AXIS
    fit_z = TOP + FIT_Z + TCP_ABOVE_AXIS + SAG_Z
    up_z = TOP + 0.12
    pre_x = WALL_X0 - CAP[0] / 2 - 0.025  # caps 25 mm in front of the walls

    def arm(s):
        yaw = -s * YAW
        y0, y1 = s * GRIP_Y, s * (GRIP_Y - SQUEEZE)
        return [
            Waypoint(1.5, (x0 - 0.06, y0, up_z), FINGER_OPEN, "in front of the rod", yaw),
            Waypoint(1.0, (x0, y0, up_z), FINGER_OPEN, "over the rod", yaw),
            Waypoint(1.5, (x0, y0, grip_z), FINGER_OPEN, "descend", yaw),
            Waypoint(0.8, (x0, y0, grip_z), FINGER_CLOSED, "grip", yaw),
            Waypoint(1.5, (x0, y0, fit_z), FINGER_CLOSED, "lift to the band", yaw),
            Waypoint(1.0, (pre_x, y0, fit_z), FINGER_CLOSED, "up to the walls", yaw),
            Waypoint(2.0, (pre_x, y1, fit_z), FINGER_CLOSED, "squeeze", yaw),
            Waypoint(2.0, (WALL_CX, y1, fit_z), FINGER_CLOSED, "carry between the walls", yaw),
            Waypoint(0.8, (WALL_CX, y1, fit_z), FINGER_CLOSED, "settle", yaw),
            Waypoint(0.8, (WALL_CX, y1, fit_z), FINGER_OPEN, "let go", yaw),
            Waypoint(0.7, (WALL_CX, y1, fit_z + 0.025), FINGER_OPEN, "up", yaw),
            Waypoint(1.0, (0.17, y1, fit_z + 0.025), FINGER_OPEN, "back out of the gap", yaw),
            Waypoint(0.8, (0.17, s * 0.20, fit_z + 0.025), FINGER_OPEN, "aside, clear of the walls", yaw),
            Waypoint(4.0, (0.17, s * 0.20, fit_z + 0.025), FINGER_OPEN, "wait while the rod holds", yaw),
        ]

    return {"left": arm(1), "right": arm(-1)}
