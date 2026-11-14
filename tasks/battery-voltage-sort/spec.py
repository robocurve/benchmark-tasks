"""Reproducible apparatus dimensions and editable electrical parameters (SI units)."""

from decimal import Decimal
import json
from pathlib import Path

import numpy as np

DIRECTORY = Path(__file__).resolve().parent
SETTINGS = json.loads((DIRECTORY / "settings.json").read_text())


def centivolts(value):
    number = Decimal(str(value)) * 100
    if number != number.to_integral_value() or not 0 <= number <= 200:
        raise ValueError("Voltages must be 0.00–2.00 V, in 0.01 V increments")
    return int(number)


VOLTAGES_CV = tuple(centivolts(v) for v in SETTINGS["battery_voltages_v"])
if len(VOLTAGES_CV) not in (4, 5):
    raise ValueError("Use four or five batteries")
THRESHOLD_CV = centivolts(SETTINGS["threshold_v"])
CONTACT_HOLD = float(SETTINGS["contact_hold_s"])
SUCCESS_HOLD = float(SETTINGS["success_hold_s"])
HZ = int(SETTINGS["control_hz"])
MAX_STEPS = int(SETTINGS["max_steps"])
if HZ != 20 or CONTACT_HOLD <= 0 or SUCCESS_HOLD <= 0 or MAX_STEPS <= 0:
    raise ValueError("Use 20 Hz and positive hold durations and action limit")

TOP = 0.7475
CELL_RADIUS = 0.00725
CELL_HALF = 0.02525
POSITIVE_RADIUS = 0.00275
NEGATIVE_RADIUS = 0.0055
TIP_RADIUS = 0.0015
TIP_LOCAL = np.array([0.0, 0.1155, 0.0])
CONTACT_TOLERANCE = 0.0006
MIN_CONTACT_FORCE = 0.005
CELL_NAMES = tuple(f"battery_{i + 1}" for i in range(len(VOLTAGES_CV)))
CELL_START = {name: np.array([0.16 + 0.05 * i, 0.0, TOP + 0.054]) for i, name in enumerate(CELL_NAMES)}
PROBE_NAMES = ("red_probe", "black_probe")
PROBE_RESTS = {"red_probe": np.array([0.11, 0.30, TOP + 0.0603]),
               "black_probe": np.array([0.11, -0.30, TOP + 0.0603])}
BIN_CENTERS = {"alive": np.array([0.26, 0.30, TOP]), "dead": np.array([0.26, -0.30, TOP])}
BIN_INNER_HALF = np.array([0.105, 0.085])
BIN_FLOOR = TOP + 0.006
BIN_RIM = TOP + 0.070
BIN_TOLERANCE = 0.001
STILL_SPEED, STILL_SPIN = 0.025, 0.8
ROBOT_CLEARANCE = 0.015
TABLE_LOW = np.array([-0.19, -0.65, TOP - 0.015])
TABLE_HIGH = np.array([0.91, 0.65, 1.80])
METER_POS = (0.525, 0.0, TOP)
INSTRUCTION = (
    f"Use the two loose voltmeter probes to test all {len(CELL_NAMES)} AA batteries. "
    f"Keep the probe tips touching opposite terminals of the same battery for at least {CONTACT_HOLD:.2f} seconds. "
    f"Use the magnitude of the displayed voltage: put batteries reading {THRESHOLD_CV / 100:.2f} V or more "
    "in the green ALIVE bin and lower readings in the red DEAD bin. Leave every battery released inside its bin."
)


def rotation(quaternion):
    x, y, z, w = np.asarray(quaternion, dtype=float)
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def cell_bounds(position, quaternion):
    axis = rotation(quaternion)[:, 1]
    radial = np.sqrt(np.maximum(0, 1-axis*axis))
    bounds = []
    for center_y, half_length, radius in ((-0.0004, 0.02465, CELL_RADIUS),
                                        (-0.02515, 0.0001, NEGATIVE_RADIUS),
                                        (0.02475, 0.0005, POSITIVE_RADIUS)):
        center = np.asarray(position) + axis*center_y
        half = np.abs(axis)*half_length + radial*radius
        bounds.append((center-half, center+half))
    return np.min([b[0] for b in bounds], axis=0), np.max([b[1] for b in bounds], axis=0)
