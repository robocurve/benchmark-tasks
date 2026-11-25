"""Station layout and dimensions in SI units."""

import numpy as np

KEY = "strip_uncapping"
DESCRIPTION = "Remove the linked caps from a filled eight-tube strip using the pronged removal tool."
INSTRUCTION = (
    "Use the removal tool to uncap the eight-tube strip. "
    "Keep the tubes seated upright and put the connected cap strip in the empty dish."
)
PHYSICS_HZ = 960
MAX_STEPS = 480
TABLE = 0.7475

RACK = np.array([0.371, 0.060, TABLE + 0.0006])
RACK_SIZE = (0.076, 0.117, 0.033)
RACK_MASS = 0.148
RACK_COM = np.array([0.0, 0.0, 0.013])
WELL_Y = (np.arange(8) - 3.5) * 0.009
RIM_Z = 0.0335
WELL_RADIUS = 0.002695
WELL_BOTTOM = 0.024
WATER_UL = 100.0
CAP_Z = RIM_Z + 0.003
CAP_COM = np.array([0.0, 0.0, -0.0005])
CAP_MASS = 0.0018
FIT_DEPTH = 0.0027
RELEASE_TRAVEL = 0.0022

TOOL = np.array([0.245, -0.183, TABLE + 0.0066])
TOOL_MASS = 0.038
TIP_FROM_TOOL = np.array([0.046, 0.0, -0.005])
TOOTH_T = 0.0008
TOOTH_POSITIONS = (-0.018, 0.018)

DISH = np.array([0.248, -0.306, TABLE])
DISH_SIZE = (0.114, 0.105, 0.014)
