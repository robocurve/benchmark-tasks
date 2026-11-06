"""Geometry of the metronome scene on the yam_arena station table, shared by build_assets.py, task.py and demo.py.

World frame (yam_arena's): x forward from the arms, y to the robot's left, z up, metres; heights are above the table
top (TOP) unless they say "world".

A mechanical metronome stands free on the table, its scale facing the arms. Its pendulum hangs on a pivot in front
of the case: a fixed bob below the pivot (inside the case) and a sliding weight on the rod above it. The higher the
weight, the slower the swing. The tempo scale behind the rod is read at the weight's top edge. Pure Python, so the
task file can import it before Isaac Sim starts.
"""

from __future__ import annotations

import math

TOP = 0.7475                            # yam_arena.task_file.TABLE_TOP["station"]
G = 9.81

# -- the case (its origin is the centre of its base; the scale faces -x, toward the arms) -------------------------------
MET_X, MET_Y = 0.34, 0.0
CASE_D, CASE_W, CASE_H = 0.11, 0.11, 0.22       # base depth (x) and width (y), height
TOP_D, TOP_W = 0.045, 0.045                      # the top's depth and width; the front face is vertical
FRONT_X = -CASE_D / 2                            # the front face, in the case frame
CASE_MASS = 0.5

# -- the pendulum ---------------------------------------------------------------------------------------------------------
PIVOT_X, PIVOT_Z = FRONT_X - 0.007, 0.085        # the rod swings 7 mm in front of the scale
ROD_W, ROD_T = 0.004, 0.002                      # across (y) and front to back (x)
ROD_LOW, ROD_TOP = -0.005, 0.130                 # the rod's ends, from the pivot
ROD_MASS = 0.008
BOB_MASS, BOB_L, BOB_R = 0.0725, 0.050, 0.010    # the fixed bob, hidden in the case below the pivot
SWING_LIMIT = math.radians(45)                   # the rod's stops on the case

# -- the sliding weight (its frame is at its top edge, which reads the scale) ---------------------------------------------
WEIGHT_W, WEIGHT_T, WEIGHT_H = 0.020, 0.010, 0.013    # its back 1.6 mm off the scale: little parallax
WEIGHT_MASS = 0.025
X_MIN, X_MAX = 0.015, 0.123                      # how far it slides: its top edge's height above the pivot
DETENT_K, DETENT_D = 400.0, 3.0                  # N/m, N s/m: the spring clip holding it on a notch
SAG = WEIGHT_MASS * G / DETENT_K                 # the clip's target sits this far above the notch, against gravity
CLICK = 0.0003                                   # the clip moves to the next notch once the weight is this much nearer it

# -- the tempo scale --------------------------------------------------------------------------------------------------------
MARKS = (40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 63, 66, 69, 72, 76, 80, 84, 88, 92, 96, 100, 104, 108, 112, 116,
         120, 126, 132, 138, 144, 152, 160, 168, 176, 184, 192, 200, 208)         # Maelzel's standard marks
START, TARGET = 60, 120
SCALE_Z = (0.012, 0.128)                         # the scale plate, from the pivot
SCALE_HALF_W = 0.025

# -- the escapement (task code) ---------------------------------------------------------------------------------------------
SWING = math.radians(25)                         # it keeps the swing at this amplitude
START_SWING = math.radians(10)                   # swinging less than this at the centre line, it doesn't start
DAMP_RUN = 1e-6                                  # N m s/rad: the pivot while the escapement runs
DAMP_STOP = 4e-4                                 # N m s/rad: a stopped escapement brakes the swing
KICK_MAX = math.radians(5)                       # the most one beat changes the swing by
BEAT_LATE = 1.3                                  # a beat this late (times the time between beats): it has stopped
SWING_TOL = math.radians(3)                      # the swing off SWING by more than this: something touched it
ON_ITS_OWN = 5.0                                 # s running with nothing touching it: it is ticking on its own
LIFTED = 0.005                                   # the case's base this far off the table: it is being held up
UPRIGHT = math.radians(10)                       # tilted past this, it is not standing on its base
TIPPED = math.radians(45)                        # the case tilted past this, at any time, is knocked over


# -- the pendulum's physics -------------------------------------------------------------------------------------------------
def _rod_inertia() -> tuple[float, float]:
    length = ROD_TOP - ROD_LOW
    c = (ROD_TOP + ROD_LOW) / 2
    return c, ROD_MASS * (length ** 2 / 12 + c * c)


def restoring(x: float) -> float:
    """g times the pendulum's mass moment below the pivot (N m per unit sin of the swing), weight top at ``x``."""
    c, _ = _rod_inertia()
    return G * (BOB_MASS * BOB_L - ROD_MASS * c - WEIGHT_MASS * (x - WEIGHT_H / 2))


def inertia(x: float) -> float:
    """The pendulum's moment of inertia about the pivot (kg m^2), weight top at ``x``."""
    _, i_rod = _rod_inertia()
    i_w = WEIGHT_MASS * (WEIGHT_W ** 2 + WEIGHT_H ** 2) / 12
    return (BOB_MASS * BOB_L ** 2 + 0.4 * BOB_MASS * BOB_R ** 2 + i_rod + i_w
            + WEIGHT_MASS * (x - WEIGHT_H / 2) ** 2)


def amplitude_factor(a: float) -> float:
    """A pendulum's period at amplitude ``a`` over its small-swing period."""
    return 1 + a * a / 16 + 11 * a ** 4 / 3072


def mark_height(bpm: float) -> float:
    """Where the weight's top edge goes for ``bpm`` beats a minute (one beat per swing, at the escapement's amplitude)."""
    period = 120.0 / bpm / amplitude_factor(SWING)
    c = (period / (2 * math.pi)) ** 2 * G           # I = c (k / g): solve the quadratic in the weight's centre
    _, i_rod = _rod_inertia()
    rod_c, _ = _rod_inertia()
    b0 = BOB_MASS * BOB_L - ROD_MASS * rod_c
    i0 = BOB_MASS * BOB_L ** 2 + 0.4 * BOB_MASS * BOB_R ** 2 + i_rod + WEIGHT_MASS * (WEIGHT_W ** 2 + WEIGHT_H ** 2) / 12
    m = WEIGHT_MASS
    u = (-c * m + math.sqrt(c * c * m * m - 4 * m * (i0 - c * b0))) / (2 * m)
    return u + WEIGHT_H / 2


MARK_X = tuple(mark_height(b) for b in MARKS)    # each mark's height above the pivot (descending)


def notch(x: float) -> int:
    """The index of the mark nearest the weight's top edge at ``x``."""
    return min(range(len(MARKS)), key=lambda i: abs(MARK_X[i] - x))
