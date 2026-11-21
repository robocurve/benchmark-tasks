"""SI geometry of a kitchen book rest, its leaf-spring clips and one cookbook."""

import numpy as np
from yam_arena import station
from yam_arena.constants import READY
from yam_arena.urdf import axis_angle_matrix

KEY = "cookbook_stand"
INSTRUCTION = "Secure the cookbook open to the Lemon Potatoes spread using both page holders, and leave it ready to read."
DESCRIPTION = "A sprung cookbook rests open on a sloping bamboo stand with two swiveling page clips."
MAX_STEPS = 450
PHYSICS_HZ = 240
CHECK_DT = .05
BASES = station.BASES
STOWED = dict.fromkeys(BASES, READY)
TABLE = .7475
PIVOT_X = -.044
PIVOT_Y = .110
PIVOT_Z = -.009
CLIP_START = 5.
CLIP_END = 103.
CLIP_LENGTH = .072
BOOK_HEIGHT = .207
PAGE_WIDTH = .143
PAGE_CENTER_Y = .074
PRINT_BOTTOM = .002
PRINT_TOP = BOOK_HEIGHT - .002
PRINT_FRONT = -.01386
PIVOT_RADIUS = .006
PIVOT_DEPTH = .007
STRIP_SIZE = (.0045, .054, .013)
BEND_SIZE = (.010, .014, .012)
RETURN_SIZE = (.012, .008, .010)
RUBBER_SIZE = (.007, .013, .012)
RETURN_CENTER = (.002, 0., 0.)
RUBBER_CENTER = (.0065, 0., 0.)
BOOK_START = 16.
STAND_ORIGIN = np.array([.407, 0., TABLE + .028])
FRAME = axis_angle_matrix(np.array([0., 1., 0.]), np.radians(30.))
BOOK_ORIGIN = np.array([-.004, 0., .004])


def world(point):
    return STAND_ORIGIN + FRAME @ np.asarray(point)


def sign(side):
    return 1 if side == "left" else -1


def pivot(side):
    return np.array([PIVOT_X, sign(side) * PIVOT_Y, PIVOT_Z])


def strip_center(side):
    return (-.016, -sign(side) * .027, 0.)


def bend_center(side):
    return (-.010, -sign(side) * .054, 0.)
