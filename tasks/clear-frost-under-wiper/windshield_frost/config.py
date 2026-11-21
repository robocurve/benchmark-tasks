"""Original full-size model dimensions, expressed in metres."""

import numpy as np
from yam_arena import station
from yam_arena.constants import READY
from yam_arena.urdf import axis_angle_matrix

KEY = "windshield_frost"
INSTRUCTION = (
    "Clear the frost from the lower windshield with the ice scraper, including "
    "the band beneath the wiper blade. Set the scraper down and leave the wiper "
    "resting against the glass."
)
DESCRIPTION = "Full-size unpowered lower windshield with a spring wiper and loose plastic scraper."
MAX_STEPS = 720
PHYSICS_HZ = 240
BASES = station.BASES
STOWED = {name: READY for name in BASES}
TABLE = .7475
GLASS_ORIGIN = np.array([.24, 0., .812])
GLASS_ANGLE = 20.
GLASS_ROT = axis_angle_matrix(np.array([0., 1., 0.]), np.deg2rad(-GLASS_ANGLE))


def wind(x, y, h=0.):
    """Map coordinates tangent to the glass into the bench frame."""
    return GLASS_ORIGIN + GLASS_ROT @ np.array([x, y, h])


HINGE = wind(-.024, .196, .031)
ARM_GRIP = np.array([.126, 0., 0.])
BLADE_X = .250
BLADE_Y = -.140
BLADE_LENGTH = .440
LIFT_DEG = 22.0
SCRAPER_START = np.array([.274, -.494, TABLE + .002])
SCRAPER_GRASP = np.array([-.034, 0., .015])
EDGE_X = .091
EDGE_WIDTH = .140
FLEX_K = 6200.
FORCE_MIN = 1.2
FORCE_MAX = 45.
BAND_X0 = .180
BAND_X1 = .276
BAND_Y0 = -.145
BAND_Y1 = -.025
CELLS_X = 16
CELLS_Y = 12
