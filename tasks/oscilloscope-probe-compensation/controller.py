"""Scripted demonstration using observed poses and ordinary YAM joint commands."""

import math

import numpy as np

from .geometry import TABLE_Z, TOOL_GRIP, TOOL_REST, TOOL_TIP, TRIMMER_CENTER


def scripted_actions(task, base, policy, max_steps):
    """Yield robot joint targets, a stage name and measurements at 20 Hz."""
    from yam_arena import ik, station
    from yam_arena.runtime import arm_q
    from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix

    if policy not in {"hold", "pickup", "engage", "full"}:
        raise ValueError(f"Unsupported oscilloscope policy: {policy}")

    left = base.scene["left_robot"]
    left_base = station.BASES["left"]
    action = np.r_[arm_q(left), 1.0, arm_q(base.scene["right_robot"]), 1.0]
    steps = 0
    up = np.array([0.0, 0.0, 1.0])
    grip_rotation = np.diag([1.0, -1.0, -1.0])
    closed = 0.055

    def output(phase, **metrics):
        nonlocal steps
        if steps >= max_steps:
            raise RuntimeError(f"Action limit reached during {phase}")
        steps += 1
        return action.copy(), phase, metrics

    def tool():
        position, quaternion, _ = task.pose_state(base, "tool")
        position = position.cpu().numpy().astype(float)
        quaternion = quaternion.cpu().numpy().astype(float)
        if (not np.isfinite(position).all() or not np.isfinite(quaternion).all()
                or abs(np.linalg.norm(quaternion) - 1.0) > 0.001):
            raise RuntimeError("Invalid native adjustment-tool pose")
        return position, quat_xyzw_to_matrix(quaternion)

    def hand():
        return ik.tcp_pose(arm_q(left), left_base)

    def command(position, rotation, phase, **metrics):
        previous = action[:6].copy()
        q, position_error, rotation_error = ik.solve(
            position, ik.matrix_to_quat_xyzw(rotation), previous, left_base,
            iters=120, damping=0.027, tol=2e-6,
        )
        if (position_error > 0.0006 or rotation_error > 0.009
                or np.max(np.abs(q - previous)) > 0.18):
            raise RuntimeError(
                f"{phase} IK: {position_error:.6f} m, {rotation_error:.4f} rad, "
                f"joint step {np.max(np.abs(q - previous)):.3f} rad, TCP {position.tolist()}"
            )
        action[:6] = q
        return output(phase, position_m=float(position_error), rotation_rad=float(rotation_error), **metrics)

    def move(position, rotation, count, phase):
        start, start_rotation = hand()
        for step in range(count):
            fraction = ik.smoothstep((step + 1) / count)
            yield command(start + fraction * (position - start),
                          ik.slerp_matrix(start_rotation, rotation, fraction), phase)

    def gripper(value, count, phase):
        start = action[6]
        for step in range(count):
            action[6] = start + (value - start) * ik.smoothstep((step + 1) / count)
            yield output(phase)

    if policy == "hold":
        for _ in range(min(max_steps, 40)):
            yield output("settle")
        return

    position, rest_rotation = tool()
    # The pad tips extend 13 mm below the TCP. Support the actual handle
    # between their bottom edges, leaving room above the table.
    grip_point = position + rest_rotation @ TOOL_GRIP + 0.009 * up
    yield from move(grip_point + 0.060 * up, grip_rotation, 52, "approach stock tool")
    yield from gripper(0.29, 12, "preshape around handle")
    yield from move(grip_point, grip_rotation, 48, "lower parallel pads")
    yield from gripper(closed, 30, "grasp stock handle")
    yield from move(grip_point + 0.092 * up, grip_rotation, 58, "lift stock tool")
    position, rotation = tool()
    if position[2] < TABLE_Z + 0.055:
        raise RuntimeError(f"The free tool did not lift with the pads: {position.tolist()}")
    if policy == "pickup":
        for _ in range(20):
            yield output("hold lifted tool")
        return

    hand_position, hand_rotation = hand()
    grasp_offset = rotation.T @ (hand_position - position)
    grasp_rotation = rotation.T @ hand_rotation
    tip_height = -0.00114
    translation_bias = np.zeros(3)
    angular_bias = np.zeros(3)

    def command_tip(tip, desired_rotation, phase, *, follow=False, turning=False, angle=None):
        current_position, current_rotation = tool()
        actual_tip = current_position + current_rotation @ TOOL_TIP
        tip_error = tip - actual_tip
        rotation_error = ik._rot_error(desired_rotation, current_rotation)
        if follow:
            # Only compensate ordinary grip and actuator deflection. Corrections
            # remain bounded so a missed slot cannot call for arbitrary force.
            if (np.linalg.norm(tip_error) > 0.005
                    or np.linalg.norm(rotation_error[:2]) > 0.12
                    or abs(rotation_error[2]) > (0.45 if turning else 0.20)):
                raise RuntimeError(f"Tool left the slot approach in {phase}: "
                                   f"tip error {tip_error.tolist()}, "
                                   f"angular error {rotation_error.tolist()}")
            translation_bias[:] = np.clip(translation_bias + 0.20 * tip_error,
                                          [-0.003, -0.003, -0.002],
                                          [0.003, 0.003, 0.001])
            angular_bias[:] = np.clip(angular_bias + 0.16 * rotation_error, -0.045, 0.045)
            if turning:
                # The slot flank can oppose twist, so do not integrate that error.
                angular_bias[2] = 0.0
        magnitude = np.linalg.norm(angular_bias)
        correction = (axis_angle_matrix(angular_bias / magnitude, magnitude)
                      if magnitude > 1e-10 else np.eye(3))
        target_rotation = correction @ desired_rotation
        target_hand = tip + translation_bias + target_rotation @ (grasp_offset - TOOL_TIP)
        return command(target_hand, target_rotation @ grasp_rotation, phase,
                       tip_m=actual_tip.tolist(), tip_error_m=tip_error.tolist(),
                       blade_yaw_rad=float(math.atan2(current_rotation[1, 0], current_rotation[0, 0])),
                       angular_error_rad=rotation_error.tolist(),
                       rotor_angle_rad=None if angle is None else float(angle))

    rotor_start = task.trimmer_angle(base)
    blade_rotation = axis_angle_matrix(up, rotor_start)
    above_slot = TRIMMER_CENTER + 0.025 * up
    start_tip = position + rotation @ TOOL_TIP
    # Lift is complete before swinging over the scope, so the tool does not skim the panel.
    for step in range(70):
        fraction = ik.smoothstep((step + 1) / 70)
        tip = start_tip + fraction * (above_slot - start_tip)
        yield command_tip(tip, ik.slerp_matrix(rotation, blade_rotation, fraction),
                          "align blade above slot", angle=rotor_start)

    seated = TRIMMER_CENTER + tip_height * up
    for step in range(100):
        fraction = ik.smoothstep(min((step + 1) / 70, 1.0))
        tip = above_slot + fraction * (seated - above_slot)
        yield command_tip(tip, blade_rotation, "seat blade through recessed opening",
                          follow=True, angle=task.trimmer_angle(base))

    if policy == "engage":
        return

    # The 0.70 mm slot has clearance around the 0.50 mm blade. Advance past
    # that clearance so the real blade meets a slot flank, then use rotor
    # readback to limit the advance through the final few degrees.
    direction = -1.0 if rotor_start > 0 else 1.0
    angle = rotor_start
    for _ in range(300):
        rotor = task.trimmer_angle(base)
        if abs(rotor) <= 0.032:
            break
        increment = 0.003 if abs(rotor) < 0.12 else 0.006
        ahead = min(direction * (angle - rotor) + increment, 0.40)
        angle = rotor + direction * ahead
        yield command_tip(seated, axis_angle_matrix(up, angle), "turn trimmer using blade contact",
                          follow=True, turning=True, angle=rotor)
    else:
        raise RuntimeError(f"Blade did not turn the rotor to compensation: {task.trimmer_angle(base):.4f} rad")

    # Withdraw along the tool axis to avoid reopening the slot contact.
    position, rotation = tool()
    start_tip = position + rotation @ TOOL_TIP
    translation_bias.fill(0)
    angular_bias.fill(0)
    for step in range(48):
        fraction = ik.smoothstep((step + 1) / 48)
        yield command_tip(start_tip + 0.062 * fraction * up, rotation, "withdraw blade above instrument",
                          angle=task.trimmer_angle(base))

    rest_tip = TOOL_REST + rest_rotation @ TOOL_TIP
    position, rotation = tool()
    start_tip = position + rotation @ TOOL_TIP
    raised_rest = rest_tip + 0.085 * up
    for step in range(70):
        fraction = ik.smoothstep((step + 1) / 70)
        tip = start_tip + fraction * (raised_rest - start_tip)
        yield command_tip(tip, ik.slerp_matrix(rotation, rest_rotation, fraction), "return tool over bench")
    for step in range(50):
        fraction = ik.smoothstep((step + 1) / 50)
        tip = raised_rest + fraction * (rest_tip + 0.0005 * up - raised_rest)
        yield command_tip(tip, rest_rotation, "put tool down on bench")
    yield from gripper(1.0, 20, "release stock tool")
    hand_position, hand_rotation = hand()
    yield from move(hand_position + 0.140 * up, hand_rotation, 54, "clear tool and scope")
    # Clear the front display so the released result remains visible on the stock camera.
    start = action[:6].copy()
    ready = task.initial_action()[:6]
    for step in range(75):
        action[:6] = start + ik.smoothstep((step + 1) / 75) * (ready - start)
        yield output("return arm to ready")
    for _ in range(40):
        yield output("hold displayed waveform with hands clear",
                     rotor_angle_rad=float(task.trimmer_angle(base)))
