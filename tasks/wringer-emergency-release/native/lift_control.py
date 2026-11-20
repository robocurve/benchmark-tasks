"""Load compensation for the scripted gripper holding the release tab."""
import numpy as np
from yam_arena import ik
from yam_arena.constants import SHOULDER, WRIST

# Extra Cartesian load demand, expressed through the normal six joint-position targets.
# The spring and yoke weight at 12 mm require about 28 N in total.
SET_GAP_MM = 12.
LIMIT_N = 32.
RATE_N_S = 18.
GAINS = np.array([SHOULDER["stiffness"]] * 3 + [WRIST["stiffness"]] * 3)
EFFORT = np.array([SHOULDER["effort"]] * 3 + [WRIST["effort"]] * 3)

def next_load(previous, gap_mm, dt, seated):
    if not seated:
        return 0.
    rate = np.clip(3.0 * (SET_GAP_MM - gap_mm), -RATE_N_S, RATE_N_S)
    return float(np.clip(previous + dt * rate, 0., LIMIT_N))

def compensated_target(planned, measured, load):
    """Map upward TCP demand through J_z at the observed pose and the YAM drive gains."""
    if not load:
        return planned.copy()
    eps = 1e-5
    z = ik.tcp_pose(measured)[0][2]
    jz = np.array([(ik.tcp_pose(measured + step)[0][2] - z) / eps
                   for step in np.eye(6) * eps])
    delta = jz * load / GAINS
    # Only use remaining proportional-effort headroom. Leave ordinary motion to the stock drive.
    nominal = planned - measured
    margin = 0.85 * EFFORT / GAINS
    delta = np.clip(delta, np.minimum(0., -margin - nominal), np.maximum(0., margin - nominal))
    return np.clip(planned + delta, ik.LOWER, ik.UPPER)
