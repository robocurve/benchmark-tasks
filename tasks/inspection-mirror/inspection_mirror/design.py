"""Geometry of the hidden-leak inspection. NumPy only (no Isaac).

Units: metres, kilograms, radians; quaternions xyzw. Station frame: x forward from the line between the arm bases,
y left, z up; TOP is the table top. Heights are above the table top.

A steel machine housing stands 55 mm from a wall, both running straight away from the robot. The gap between them is
open at its near end (facing the robot) and closed at its far end, at the bottom, and on top by a steel cover strip.
A coolant leak has left a bright green trace on the housing's side face, inside the gap, at one of five places along
it (drawn per trial from the episode seed). Looking into the open end, that face is seen edge-on, so nothing shows the
trace. An Ullman HTC-2 inspection mirror (57 mm round glass head, set at 45 degrees on its swivel) slid into the gap
shows the face to a camera behind the handle. The robot then marks the leak's place on the cover strip, directly
above the trace, with an oil-based paint marker, so the repair goes to the right spot.
"""

from __future__ import annotations

import math

import numpy as np

TOP = 0.7475  # station table top

# -- housing, wall, gap (the gap runs along x) ----------------------------------------------------------
GAP_X = (0.225, 0.405)  # open end at x 0.225 (facing the robot), closed end at 0.405
GAP = 0.055  # housing side face to wall: the 57 mm head at 45 degrees is 43 mm across, so 6 mm each side
FACE_Y = -0.0275  # the hidden face: the housing's side face, facing +y; the wall's inner face is at FACE_Y + GAP
PANEL_T = 0.005
HEIGHT = 0.130
CAP_T = 0.005
CAP_TOP = HEIGHT + CAP_T  # the cover strip's top face (where the mark goes)
CAP_Y = (FACE_Y - PANEL_T, FACE_Y + GAP + PANEL_T)  # the strip spans the housing side, the gap and the wall
BASE_T = 0.005

# -- the leak trace (hidden state: one of five places, drawn from the episode seed) ----------------------
LEAK_X = (0.300, 0.320, 0.340, 0.360, 0.380)  # 20 mm apart, 7.5 to 15.5 cm in from the open end
LEAK_Z = 0.075  # centre height of the trace on the face
LEAK_SIZE = (0.022, 0.050)  # width along x, height: a run-down trace
LEAK_PARK = np.array([0.0, -0.60, -0.30])  # where the unused trace body is never; the live one is placed at reset
GREEN = (0.10, 0.85, 0.20)  # coolant dye


def leak_index(seed: int | None) -> int:
    """The trial's leak place from the episode seed (no seed: the middle place)."""
    if seed is None:
        return len(LEAK_X) // 2
    return int(np.random.default_rng(int(seed)).integers(len(LEAK_X)))


# -- inspection mirror (Ullman HTC-2, free tool): the handle along the tool's +x, the head at the +x end ---------
# 2-1/4 in (57 mm) round glass mirror; 6-1/2 in (165 mm) collapsed, 36-3/8 in extended (Ullman listings).
GRIP = (0.100, 0.0127)  # vinyl grip: length, diameter (diameter [estimate])
SHAFT = (0.120, 0.0050)  # the telescoped steel shaft past the grip: length set for this job, diameter [estimate]
HEAD_D = 0.057
HEAD_T = 0.004  # glass and its steel back
HEAD_YAW = math.radians(45.0)  # set on the swivel: a ray travelling +x is turned toward -y (onto the hidden face)
# The swivel sits on the head's rim at its top, so the head hangs below the shaft's axis:
HEAD_DROP = HEAD_D / 2 + SHAFT[1] / 2  # shaft axis to head centre
HEAD_REACH = GRIP[0] / 2 + SHAFT[0]  # tool origin (grip centre) to head centre, along the tool's +x
TOOL_MASS = 0.090  # [estimate]
LED_INTENSITY = 1.5e5  # the HTC-2LT's LED (render units)
TOOL_REST = np.array([0.165, -0.130, 0.100])  # grip centre at rest, the free end of the grip toward the robot
SUPPORT = (0.012, 0.030, 0.094)  # each support block (x, y, z) under the grip, clear of the hand at its free end
SUPPORT_DX = (-0.040, 0.060)  # under the grip and under the shaft's root; the hand takes the grip between them

# -- paint marker (Sharpie oil-based paint marker, medium point: about 142 x 16 mm) ---------------------------
MARKER = (0.142, 0.016)  # length, barrel diameter
MARKER_TIP = (0.008, 0.004)  # tip length, diameter
MARKER_MASS = 0.022  # [estimate]
HOLDER_XY = np.array([0.190, 0.090])  # a steel pen cup: the marker stands in it tip down
HOLDER = (0.034, 0.034, 0.060)  # outer size; 18 mm square bore, 8 mm floor
HOLDER_BORE = 0.018
HOLDER_FLOOR = 0.008
HOLDER_STAND = 0.050  # the cup sits on a 50 mm steel stand (keeps the side grasp within the arm's reach)
MARK_DOTS = 4  # paint dots the scene can show (a fifth dab still counts in the score; it is just not drawn)
DAB_Z = 0.0015  # the tip counts as touching the strip within 1.5 mm of its top face (the tip is soft felt)
DOT_D = 0.008

# -- scoring ------------------------------------------------------------------------------------------
FULL_MM = 6.0  # a mark within 6 mm (along the gap) of the trace's centre earns 1.0
ZERO_MM = 14.0  # 0 at 14 mm or more (less than the 20 mm between places: a neighbouring place earns nothing)
REST_SPEED = 0.01
TOUCH_MM = 0.5  # the head counts as touching the face when its nearest point is within 0.5 mm of it


def _axes(q):
    x, y, z, w = (float(v) for v in q)
    ax = np.array([1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)])
    ay = np.array([2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x)])
    az = np.array([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])
    return ax, ay, az


def head_frame(tool_pose):
    """World centre, normal and in-plane axes of the mirror head for a tool pose."""
    ax, ay, az = _axes(tool_pose[3:])
    c = np.asarray(tool_pose[:3], float) + ax * HEAD_REACH - az * HEAD_DROP
    cy, sy = math.cos(HEAD_YAW), math.sin(HEAD_YAW)
    n = cy * ax + sy * ay  # the plate's thin axis (body x turned by the yaw)
    u = -sy * ax + cy * ay  # in-plane, horizontal
    return c, n, u, az


def head_center(tool_pose) -> np.ndarray:
    return head_frame(tool_pose)[0]


def head_face_clearance(tool_pose) -> float:
    """Distance (m) from the hidden face to the head's nearest point (negative: through it)."""
    c, n, u, v = head_frame(tool_pose)
    reach_y = HEAD_D / 2 * math.hypot(u[1], v[1]) + HEAD_T / 2 * abs(n[1])
    return float(c[1] - reach_y - FACE_Y)


def marker_tip(marker_pose) -> np.ndarray:
    """World point of the marker's tip: the marker's body -z end (origin at the barrel's centre)."""
    _, _, az = _axes(marker_pose[3:])
    return np.asarray(marker_pose[:3], float) - az * (MARKER[0] / 2 + MARKER_TIP[0])


def on_strip(p) -> bool:
    """Is a point on the cover strip's top face (within DAB_Z above it)?"""
    return (GAP_X[0] <= p[0] <= GAP_X[1] + PANEL_T and CAP_Y[0] <= p[1] <= CAP_Y[1]
            and -0.001 <= p[2] - (TOP + CAP_TOP) <= DAB_Z)


def start_poses() -> dict[str, np.ndarray]:
    ident = [0.0, 0.0, 0.0, 1.0]
    out = {"housing": np.array([0.0, 0.0, TOP, *ident]),
           "leak": np.array([*LEAK_PARK, *ident]),
           "mirror": np.array([TOOL_REST[0], TOOL_REST[1], TOP + TOOL_REST[2], *ident]),
           "marker": np.array([HOLDER_XY[0], HOLDER_XY[1],  # tip down on the cup's floor
                               TOP + HOLDER_STAND + HOLDER_FLOOR + MARKER_TIP[0] + MARKER[0] / 2, *ident])}
    for i in range(MARK_DOTS):
        out[f"dot_{i}"] = np.array([0.0, -0.60 - 0.02 * i, -0.30, *ident])
    return out


def leak_pose(index: int) -> np.ndarray:
    """The trace body's pose: its origin on the hidden face at the place's centre."""
    return np.array([LEAK_X[index], FACE_Y, TOP + LEAK_Z, 0.0, 0.0, 0.0, 1.0])


def head_in_gap(tool_pose) -> bool:
    """Is the mirror head inside the gap (only there can it touch the hidden face)?"""
    c = head_center(tool_pose)
    return GAP_X[0] < c[0] < GAP_X[1] and c[2] - TOP < HEIGHT and FACE_Y < c[1] < FACE_Y + GAP
