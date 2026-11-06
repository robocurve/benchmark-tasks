"""One fixed scene, physical dimensions and public grading thresholds (SI units)."""

import numpy as np

INSTRUCTION = (
    "Remove the camera and its attached plate from the bench mount and set the assembly upright on the green pad. "
    "Keep the camera attached to the plate, do not drop it, and leave the assembly released."
)
TABLE_TOP = 0.7475
MOUNT_POSITION = np.array([0.28, 0.0, 0.7675])
PAYLOAD_POSITION = np.array([0.28, 0.0, 0.7775])
PAD_POSITION = np.array([0.20, 0.14, 0.7495])
PAD_TOP = float(PAD_POSITION[2] + 0.002)
PAD_HALF_SIZE = np.array([0.065, 0.055])
CAM_PIVOT = np.array([0.025, -0.030, 0.017])
CONTROL_HZ = 20
PHYSICS_HZ = 240
MAX_STEPS = 2400
RELEASE_ANGLE = 0.8
MAX_TILT_RAD = np.deg2rad(10)
MIN_ROBOT_CLEARANCE = 0.030
MAX_UNSUPPORTED_DESCENT = 0.010
CONTACT_THRESHOLD_N = 0.02

# Every collision box, in its body's frame: centre and half extents.
PAYLOAD_BOXES = (
    (np.array([0, 0, 0.0025]), np.array([0.025, 0.019, 0.0025])),
    (np.array([0, 0, 0.033]), np.array([0.0115, 0.021, 0.021])),
)
STAND_BOXES = (
    ((0, 0, -0.016), (0.055, 0.055, 0.004)),
    ((0, 0, -0.008), (0.013, 0.013, 0.004)),
    ((0, 0, 0.0045), (0.035, 0.026, 0.0045)),
    ((0, 0.023, 0.012), (0.027, 0.003, 0.003)),
    ((0, 0.021, 0.0165), (0.027, 0.005, 0.0015)),
    ((-0.031, 0, 0.011), (0.002, 0.019, 0.002)),
    ((0.031, 0, 0.011), (0.002, 0.019, 0.002)),
)
CAM_BOXES = (
    ((-0.020, -0.018, 0), (0.012, 0.007, 0.0035)),
    ((-0.009, -0.0081, 0), (0.011, 0.004, 0.0025)),
    ((-0.018, 0.010, 0), (0.014, 0.004, 0.002)),
    ((0, 0, 0), (0.004, 0.004, 0.004)),  # conservative pivot-cylinder envelope
)
