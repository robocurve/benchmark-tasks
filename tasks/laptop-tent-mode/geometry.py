"""Shared laptop geometry. All lengths are metres and all angles are degrees."""

import math

import numpy as np

from yam_arena.constants import TABLE_HEIGHT


WIDTH = 0.304
FRONT_X = -0.214
REAR_X = -0.006
CHASSIS_Z = (-0.01120, 0.0)
DISPLAY_Z = (0.00055, 0.00625)
FEET = (0.015, 0.018, 0.0014)
FOOT_X = (-0.200, -0.020)
FOOT_Y = (-0.122, 0.122)
MASS = (0.94, 0.43)
COM = ((-0.110, 0.0, -0.0056), (-0.110, 0.0, 0.0034))
HINGE_FRICTION = (0.65, 0.52, 0.04)  # N m, N m, N m s / rad
HINGE_LIMIT = (0.5, 359.5)
RESET_ANGLE = 112.0
RESET_HINGE = (0.360, 0.0, TABLE_HEIGHT - CHASSIS_Z[0] + FEET[2])
ARM_BASES = ((0., 0.52, TABLE_HEIGHT), (0., -0.42, TABLE_HEIGHT))
DESK_SIZE = (0.8, 0.8, 0.04)
DESK_CENTER = (0.35, -0.267, TABLE_HEIGHT - DESK_SIZE[2] / 2)
CONTROL_HZ = 20
PHYSICS_HZ = 120
ACTION_LIMIT = 800


def pitch(degrees):
    angle = math.radians(degrees)
    c, s = math.cos(angle), math.sin(angle)
    return np.array(((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c)))


def box_corners(bounds):
    """The corners of a shell, without cosmetic details and rubber feet."""
    return np.array([(x, y, z) for x in (FRONT_X, REAR_X) for y in (-WIDTH / 2, WIDTH / 2) for z in bounds])


def contact_height(chassis_pitch, display_pitch):
    """Hinge elevation for the first shell edge to meet the table."""
    low = min((box_corners(z) @ pitch(a).T)[:, 2].min() for z, a in (
        (CHASSIS_Z, chassis_pitch), (DISPLAY_Z, display_pitch)
    ))
    return TABLE_HEIGHT - low
