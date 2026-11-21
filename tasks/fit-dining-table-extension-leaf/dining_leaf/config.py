"""Full-size four-seat dining table with the extension rails drawn out."""
import numpy as np
from yam_arena import station
from yam_arena.constants import READY

KEY = "dining_leaf"
INSTRUCTION = "Seat and lock the extension leaf so the dining table is ready for four people."
DESCRIPTION = "A rectangular dining table with a loose extension leaf resting on its open support rails."
BASES = station.BASES
STOWED = dict.fromkeys(BASES, READY)
PHYSICS_HZ = 240
MAX_STEPS = 370
CHECK_DT = .05

LEAF_DEPTH = .400
LEAF_WIDTH = .700
THICKNESS = .022
SURFACE_Z = .750
SEAM_X = .710
LEAF_MASS = 3.5
CLOSED_X = SEAM_X - LEAF_DEPTH / 2
START_X = CLOSED_X - .066
START_YAW = .021
LATCH_PIVOT = np.array([.309, 0., .679])
LATCH_ANGLE = 83.0
CATCH_END = np.array([-.052, 0., .006])
CATCH_SIZE = np.array([.013, .027, .011])
RECEIVER = np.array([-.181, 0., -.021])
RECEIVER_SIZE = np.array([.022, .030, .026])
