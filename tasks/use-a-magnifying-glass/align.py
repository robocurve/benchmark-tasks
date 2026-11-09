"""Small measured-tool corrections through ordinary right-arm joint commands.

Privileged scripted-demo feedback only. This does not move the tool directly,
impose a grasp, change contacts, or expose tool state to a model policy.
"""

from __future__ import annotations

import math

import numpy as np


GOAL_TOOL_POSITION = np.array([.35, -.02, .83])
GOAL_TOOL_ROTATION = np.array([[0., 1., 0.], [-1., 0., 0.], [0., 0., 1.]])
LENS_OFFSET_LOCAL = np.array([-.070, 0., 0.])
GOAL_LENS_POSITION = GOAL_TOOL_POSITION + GOAL_TOOL_ROTATION @ LENS_OFFSET_LOCAL


def action14(current_command, measured_tool_pose_xyzw, *, gain=.25,
             max_translation_m=.001, max_rotation_deg=.5):
    """Return a bounded correction using a measured XYZ+XYZW tool-root pose.

    Translation is limited at the lens centre. The TCP receives the same rigid
    transform, including the lever-arm displacement from rotating about that
    centre. Thus its total displacement can exceed the centre's 1 mm limit by
    the small rotation contribution. Fingers and left-arm commands are retained.

    A runner should bound this phase to 60 control steps (3 seconds at 20 Hz),
    then hold for 3 seconds and independently inspect measured optical benefit.
    Correction success or retention is not inferred from the commanded pose.
    """
    from yam_arena import ik, station
    from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix

    command = np.asarray(current_command, dtype=float)
    pose = np.asarray(measured_tool_pose_xyzw, dtype=float)
    if command.shape != (14,) or not np.isfinite(command).all():
        raise ValueError("current_command must contain 14 finite values")
    if pose.shape != (7,) or not np.isfinite(pose).all() or np.linalg.norm(pose[3:]) < 1e-12:
        raise ValueError("measured tool pose must be finite XYZ plus nonzero XYZW quaternion")
    if not (0 < gain <= 1 and 0 < max_translation_m <= .001 and 0 < max_rotation_deg <= .5):
        raise ValueError("Corrections require gain in (0,1], translation <=1mm and rotation <=0.5deg")
    if not (0 <= command[6] <= 1 and 0 <= command[13] <= 1):
        raise ValueError("Finger targets must be normalized in [0,1]")

    measured_rotation = quat_xyzw_to_matrix(pose[3:])
    lens_center = pose[:3] + measured_rotation @ LENS_OFFSET_LOCAL
    displacement = gain * (GOAL_LENS_POSITION - lens_center)
    distance = float(np.linalg.norm(displacement))
    if distance > max_translation_m:
        displacement *= max_translation_m / distance

    relative_quaternion = ik.matrix_to_quat_xyzw(GOAL_TOOL_ROTATION @ measured_rotation.T)
    if relative_quaternion[3] < 0:
        relative_quaternion = -relative_quaternion
    vector_norm = float(np.linalg.norm(relative_quaternion[:3]))
    if vector_norm > 1e-12:
        angle = 2 * math.atan2(vector_norm, float(relative_quaternion[3]))
        step_angle = min(gain * angle, math.radians(max_rotation_deg))
        correction_rotation = axis_angle_matrix(relative_quaternion[:3] / vector_norm, step_angle)
    else:
        correction_rotation = np.eye(3)

    tcp, tcp_rotation = ik.tcp_pose(command[7:13], station.BASES["right"])
    target_tcp = lens_center + displacement + correction_rotation @ (tcp - lens_center)
    target_rotation = correction_rotation @ tcp_rotation
    joints, position_error, rotation_error = ik.solve(
        target_tcp, ik.matrix_to_quat_xyzw(target_rotation), command[7:13],
        station.BASES["right"], iters=100, tol=1e-6)
    if position_error > .0003 or rotation_error > math.radians(.1):
        raise RuntimeError(f"Alignment IK did not reach its bounded target: "
                           f"{position_error * 1000:.3f} mm / {math.degrees(rotation_error):.4f} deg")
    if np.any(joints < ik.LOWER) or np.any(joints > ik.UPPER):
        raise RuntimeError("Alignment IK exceeded right-arm joint limits")
    result = command.copy()
    result[7:13] = joints
    return result.tolist()
