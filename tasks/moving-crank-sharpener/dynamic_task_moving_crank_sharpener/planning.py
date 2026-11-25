"""Joint-limited IK with a second seed when a trajectory reaches a local minimum."""
import numpy as np
from yam_arena import ik
from yam_arena.constants import READY


def solve_target(position, rotation, previous_q, base_pos):
    quaternion = ik.matrix_to_quat_xyzw(rotation)
    result = ik.solve(position, quaternion, previous_q, base_pos=base_pos, iters=60)
    if result[1] > 0.005 or result[2] > 0.05:
        alternate = ik.solve(position, quaternion, np.array(READY), base_pos=base_pos, iters=100)
        if alternate[1]+0.1*alternate[2] < result[1]+0.1*result[2]:
            result = alternate
    return result
