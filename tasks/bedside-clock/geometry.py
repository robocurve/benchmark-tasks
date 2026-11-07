"""Geometry of the bedside-clock scene on the yam_arena station table, shared by build_assets.py, task.py and demo.py.

World frame (yam_arena's): x forward from the arms, y to the robot's left, z up, metres; heights are above the table
top (TOP) unless they say "world".

A wedge-shaped bedside clock stands in the middle of the table. Its display is three number drums (hours 1-12,
tens of minutes 0-5, minutes 0-9) whose fronts show through a window in the 45-degree front slope, as on a flip or
tumbler clock; its HOUR and MIN buttons stand on the flat top behind it. Each press of HOUR adds an hour (12 wraps to
1) and each press of MIN adds a minute (59 wraps to 00 without changing the hour). Behind it, an analog clock on a stand
shows the time to set. Pure NumPy, so the task file can import it before Isaac Sim starts.
"""

from __future__ import annotations

import math

import numpy as np

TOP = 0.7475                            # yam_arena.task_file.TABLE_TOP["station"]

# -- bedside clock: a pentagonal prism (profile in x-z, extruded along y) -----------------------------------------------
CLOCK_Y = 0.0
FRONT_X, LOW_Z = 0.20, 0.03             # the bottom of the front slope
SLOPE_X, TOP_Z = 0.27, 0.10             # the top of the front slope (45 degrees)
BACK_X = 0.35
CLOCK_W = 0.22                          # along y
PROFILE = ((FRONT_X, 0.0), (BACK_X, 0.0), (BACK_X, TOP_Z), (SLOPE_X, TOP_Z), (FRONT_X, LOW_Z))

SLOPE = math.atan2(SLOPE_X - FRONT_X, TOP_Z - LOW_Z)      # from vertical, toward +x
N = np.array([-math.cos(SLOPE), 0.0, math.sin(SLOPE)])     # display normal, out of the slope
U = np.array([0.0, -1.0, 0.0])                             # display right, as the robot sees it
V = np.array([math.sin(SLOPE), 0.0, math.cos(SLOPE)])      # display up, along the slope
DISP_C = np.array([(FRONT_X + SLOPE_X) / 2, CLOCK_Y, (LOW_Z + TOP_Z) / 2])

# -- number drums, turning about axes along y, their fronts 4 mm proud of the slope --------------------------------------
PITCH = 0.036                           # arc length per number on a drum's surface
PROUD = 0.004
DRUMS = {                               # name: (numbers, centre along U, length along y)
    "hours": (tuple(str(h) for h in range(1, 13)), -0.046, 0.060),
    "tens": (tuple(str(t) for t in range(6)), 0.024, 0.032),
    "ones": (tuple(str(o) for o in range(10)), 0.060, 0.032),
}
WINDOW_H = 0.032                        # the bezel's opening along V
BEZEL = (0.17, 0.020, 0.008)            # each bar above and below the opening: along U, along V, out of the slope
COLON_U = -0.004
INK, PAPER = (0.95, 0.95, 0.92), (0.07, 0.07, 0.08)        # white numbers on black drums


def drum_radius(name: str) -> float:
    return len(DRUMS[name][0]) * PITCH / (2 * math.pi)


def drum_centre(name: str) -> np.ndarray:
    return DISP_C + U * DRUMS[name][1] - N * (drum_radius(name) - PROUD)


def drum_step(name: str) -> float:
    """The drum's turn (rad) from one number to the next."""
    return 2 * math.pi / len(DRUMS[name][0])


def drum_start(name: str) -> int:
    """The number each drum shows at the start (its position at joint angle 0)."""
    h0, m0 = START
    return {"hours": h0 - 1, "tens": m0 // 10, "ones": m0 % 10}[name]


def drum_counts(hour_presses: int, min_presses: int) -> dict:
    """Cumulative number positions of the three drums after these presses from START (unwrapped, so they only ever
    turn forward)."""
    h0, m0 = START
    ones = m0 % 10 + min_presses
    return {"hours": (h0 - 1) + hour_presses, "tens": m0 // 10 + (ones // 10), "ones": ones}


# -- buttons on the flat top ----------------------------------------------------------------------------------------------
BUTTON_X = 0.305
BUTTON_Y = {"hour": 0.07, "min": -0.07}     # HOUR on the robot's left, MIN on its right; 14 cm apart, so both arms fit
CAP, CAP_H, TRAVEL = 0.024, 0.008, 0.005
CAP_TOP = TOP_Z + CAP_H                 # a cap's top at rest
PRESSED, RELEASED = 0.7 * TRAVEL, 0.3 * TRAVEL   # a press counts past 70 % of the travel; re-arms under 30 %

# -- times -------------------------------------------------------------------------------------------------------------------
START = (1, 25)                         # the bedside clock after the power cut
TARGET = (4, 40)                        # the analog clock's hands

# -- the analog clock, on an easel stand behind the bedside clock, leaning back toward the camera ----------------------
ANALOG_C = np.array([0.58, 0.18, 0.17])
ANALOG_R = 0.11
ANALOG_LEAN = math.radians(25.0)

