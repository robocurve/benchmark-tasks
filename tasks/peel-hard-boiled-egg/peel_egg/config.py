"""Physical kitchen station and large, cooled chicken egg dimensions in metres."""

import numpy as np
from yam_arena import station
from yam_arena.constants import READY

KEY = "peel_egg"
INSTRUCTION = "Gently crack the hard-boiled egg, peel away its entire shell, and leave the intact egg on the blue plate."
DESCRIPTION = "A whole hard-boiled egg, a wood board and a clean ceramic plate."
MAX_STEPS = 820
PHYSICS_HZ = 240
BASES = station.BASES
STOWED = {name: READY for name in BASES}
TABLE = .7475
BOARD = np.array([.30, -.05, TABLE + .012])
START = np.array([.282, -.048, BOARD[2] + .0218])
PLATE = np.array([.32, .148, TABLE])
HEIGHT = .935
WORK = np.array([.282, -.048, HEIGHT])
SHELL_BOUNDS = {"blunt": (-1., -.55), "band": (-.55, .45), "point": (.45, 1.)}
