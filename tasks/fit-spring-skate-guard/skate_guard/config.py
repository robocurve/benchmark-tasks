"""Standard bench geometry and dry-skate setup."""

import numpy as np
from yam_arena import station
from yam_arena.constants import READY

KEY = "skate_guard"
INSTRUCTION = "Put the skate in the gear bag."
DESCRIPTION = "One dry skate, an open gear bag, and loose two-piece spring runner protector are on the bench."
MAX_STEPS = 540
PHYSICS_HZ = 240
BASES = station.BASES
STOWED = {name: READY for name in BASES}
TABLE = 0.7475
INVERTED = np.diag([-1., 1., -1.])
BOOT_START = np.array([0.344, 0.019, TABLE + .176])
GUARD_START = np.array([0.216, 0.019, TABLE + .068])
BAG = np.array([0.517, 0.019, TABLE])
BAG_SIZE = (0.235, 0.387, 0.187)
BAG_FLOOR = 0.006
RUNNER_TOE = 0.124
RUNNER_HEEL = -0.119
GUARD_TOE_INNER = 0.127
HEEL_AT_REST = -0.100
GUARD_SEAT_Z = 0.012
HEEL_BODY_Y = -0.068
GUARD_TRAVEL = 0.054
BOOT_TOE_GRIP = np.array([0., .126, .050])
BOOT_HEEL_GRIP = np.array([0., -.091, .067])
GUARD_TOE_GRIP = np.array([0., .082, .012])
GUARD_HEEL_GRIP = np.array([0., -.071, .012])
SPRING_RATE = 105.0
SPRING_DAMPING = 1.8
SPRING_PRELOAD_EXTENSION = 0.012

# Profile stations for the solid boot, from heel to toe.
BOOT_SECTIONS = (
    (-.113, .063, .037, .101),
    (-.076, .066, .065, .157),
    (-.014, .077, .075, .105),
    (.049, .088, .074, .081),
    (.101, .075, .055, .066),
    (.131, .057, .018, .057),
)
HOLDER_STATIONS = (-.091, .092)


def corners(position, size):
    """Corners of one component's conservative local bounds."""
    return np.asarray(position) + np.asarray(size) / 2 * np.array([
        [x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)
    ])


RUNNER_CORNERS = corners((0, (RUNNER_TOE + RUNNER_HEEL) / 2, .0075),
                         (.0048, RUNNER_TOE - RUNNER_HEEL, .015))
BOOT_CORNERS = np.concatenate([
    RUNNER_CORNERS,
    *(corners((0, y, (.0295 + top) / 2), (width + .004, length, top - .0295))
      for y, width, length, top in BOOT_SECTIONS),
    *(corners((0, y, .0215), (.014, .041, .026)) for y in HOLDER_STATIONS),
])
TOE_CORNERS = corners((0, .058, .016), (.038, .152, .032))
HEEL_CORNERS = corners((0, -.050 - HEEL_BODY_Y, .016), (.038, .112, .032))
