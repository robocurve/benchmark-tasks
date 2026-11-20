"""Bench geometry in SI units."""

import numpy as np

KEY = "tactile_calibration"
INSTRUCTION = "Stabilize the loose optical fingertip and use the steel ball probe to capture a clean impression in each of the three marked calibration regions."
DESCRIPTION = "Optical fingertip sphere-impression calibration for robot tactile sensing."
MAX_STEPS = 620
PHYSICS_HZ = 240
TABLE = 0.7475
SENSOR_START = np.array([0.347, 0.120, TABLE + 0.100])
SENSOR_WORK = np.array([0.359, 0.025, TABLE + 0.119])
TOOL_START = np.array([0.397, -0.139, TABLE + 0.008])
PAD_Z = 0.042
BALL_Z = 0.007
BALL_R = 0.004
TOOL_GRIP = 0.084
SENSOR_GRIP = 0.019
PAD_HALF = (0.013, 0.015)
REGIONS = np.array([[-0.006, -0.005], [0.006, -0.005], [0.0, 0.006]])
SPRING = 1250
DAMPING = 10
PRESS_MIN = 0.00075
PRESS_MAX = 0.00165
HOLD_FRAMES = 8
