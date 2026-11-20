"""Reference manipulation through ordinary YAM arm and gripper targets."""

import numpy as np

from .geometry import (
    BUN_FINAL, BUN_FINAL_ROTATION_XYZW, BUN_LENGTH, CAVITY_TRAVEL,
    SAUSAGE_LENGTH, SAUSAGE_RADIUS, SPIKE_TIP, TABLE_Z,
)


UP = np.array([0., 0., 1.])
BUN_GRIP_LOCAL = np.array([0., 0., .044])
SAUSAGE_GRIP_LOCAL = np.array([0., 0., .055])
BUN_GRIP_PITCH = .35
BUN_GRIP_FRAME = np.array([[0., -np.sin(BUN_GRIP_PITCH), np.cos(BUN_GRIP_PITCH)],
                           [-1., 0., 0.],
                           [0., -np.cos(BUN_GRIP_PITCH), -np.sin(BUN_GRIP_PITCH)]])
SAUSAGE_GRIP_FRAME = np.diag([1., -1., -1.])
# Rolling the sausage about its own axis gives the wrist room to turn.
SAUSAGE_WRIST_ROLL = np.array([[0., 1., 0.], [-1., 0., 0.], [0., 0., 1.]])


def scripted_actions(task, base, policy, max_steps):
    """Yield actions at 20 Hz. Native poses guide the two physical grasps."""
    from yam_arena import ik, station
    from yam_arena.runtime import arm_q
    from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix

    if policy not in {"hold", "pierce", "full"}:
        raise ValueError(f"Unsupported hot-dog policy: {policy}")

    action = np.r_[arm_q(base.scene['left_robot']), 1., arm_q(base.scene['right_robot']), 1.]
    ready = action.copy()
    steps = 0

    def output(phase, **metrics):
        nonlocal steps
        if steps >= max_steps:
            raise RuntimeError(f"Action limit reached during {phase}")
        steps += 1
        return action.copy(), phase, metrics

    def tcp(side):
        offset = 0 if side == 'left' else 7
        return ik.tcp_pose(action[offset:offset + 6], station.BASES[side])

    def command(side, point, rotation, phase):
        offset = 0 if side == 'left' else 7
        previous = action[offset:offset + 6]
        q, pe, re = ik.solve(point, ik.matrix_to_quat_xyzw(rotation), previous,
                             station.BASES[side], iters=100, tol=2e-5)
        shift = float(np.max(np.abs(q - previous)))
        if pe > .0012 or re > .017 or shift > .20:
            raise RuntimeError(f"{phase}: {side} IK {pe:.5f} m, {re:.4f} rad, joint step {shift:.3f} rad at {point.tolist()}")
        action[offset:offset + 6] = q
        return output(phase, position_m=float(pe), rotation_rad=float(re))

    def move(side, end, rotation, count, phase):
        start, orientation = tcp(side)
        for i in range(count):
            f = ik.smoothstep((i + 1) / count)
            yield command(side, start + f * (end - start),
                          ik.slerp_matrix(orientation, rotation, f), phase)

    def grip(side, value, count, phase):
        offset = 6 if side == 'left' else 13
        start = action[offset]
        for i in range(count):
            action[offset] = start + ik.smoothstep((i + 1) / count) * (value - start)
            yield output(phase)

    def retreat_ready(side, count, phase):
        offset = 0 if side == 'left' else 7
        start = action[offset:offset + 6].copy()
        end = ready[offset:offset + 6]
        for i in range(count):
            action[offset:offset + 6] = start + ik.smoothstep((i + 1) / count) * (end - start)
            yield output(phase)

    def hold(count, phase):
        for _ in range(count):
            yield output(phase)

    def held_transform(side, name):
        p, r, _ = task.pose(base, name)
        hand_p, hand_r = tcp(side)
        return r.T @ (hand_p - p), r.T @ hand_r

    def carry(side, point, rotation, held, count, phase):
        offset, wrist = held
        yield from move(side, point + rotation @ offset, rotation @ wrist, count, phase)

    if policy == 'hold':
        yield from hold(min(60, max_steps), 'settle')
        return

    bun_p, bun_r, _ = task.pose(base, 'bun')
    if bun_r[2, 2] < .985:
        raise RuntimeError('Roll did not reset upright')
    pickup = bun_p + bun_r @ BUN_GRIP_LOCAL
    yield from move('left', pickup - .085 * BUN_GRIP_FRAME[:, 2], BUN_GRIP_FRAME, 70, 'approach the whole roll')
    yield from grip('left', .67, 14, 'size the jaws for the roll')
    yield from move('left', pickup, BUN_GRIP_FRAME, 45, 'surround the roll below its end')
    yield from grip('left', .482, 24, 'grasp the roll through pad contact')
    yield from move('left', pickup + .054 * UP, BUN_GRIP_FRAME, 55, 'lift the intact roll')
    lifted, lifted_r, _ = task.pose(base, 'bun')
    if lifted[2] < bun_p[2] + .034 or lifted_r[2, 2] < .98:
        raise RuntimeError(f'Left gripper did not lift the roll upright: {lifted.tolist()}')
    bun_hold = held_transform('left', 'bun')

    tip = np.asarray(SPIKE_TIP)
    mouth_to_center = BUN_LENGTH / 2
    prepress = tip - (mouth_to_center + .010) * UP
    beside = prepress + np.array([-.057, .041, 0.])
    upright = np.eye(3)
    yield from carry('left', beside, upright, bun_hold, 48, 'take the roll below the mandrel tip')
    yield from carry('left', prepress, upright, bun_hold, 48, 'align the roll end under the fixed tip')

    # Axial load deflects the wrist. Keep the center of the engaged shaft on
    # the roll axis and correct this deflection while the jaws keep their grip.
    xy_bias = np.zeros(2)
    tilt = np.zeros(3)
    offset, wrist = bun_hold

    def on_spike(advance, phase, insertion):
        position, rotation, _ = task.pose(base, 'bun')
        axis = rotation[:, 2]
        shaft_center = position + axis * (mouth_to_center - .5 * max(0., insertion - .0205))
        xy_bias[:] = np.clip(xy_bias + .16 * (tip[:2] - shaft_center[:2]), -.009, .009)
        tilt[:] = np.clip(tilt + np.clip(.16 * np.cross(axis, UP), -.009, .009), -.18, .18)
        angle = np.linalg.norm(tilt)
        aim = axis_angle_matrix(tilt / angle, angle) if angle > 1e-8 else upright
        center = prepress + advance * UP + np.r_[xy_bias, 0.]
        return command('left', center + aim @ offset, aim @ wrist, phase)

    # PhysX position error provides the force. Limit each advance by the measured
    # depth and the observed arm deflection so a bind does not command full travel.
    depth_goal = CAVITY_TRAVEL - .002
    press_advance = 0.
    for _ in range(190):
        depth = float(task.read_depth(base))
        if depth >= depth_goal:
            break
        command_z = tcp('left')[0][2]
        measured_z = ik.tcp_pose(arm_q(base.scene['left_robot']), station.BASES['left'])[0][2]
        deflection = float(np.clip(command_z - measured_z, 0., .030))
        lead = .010 + max(0., depth) + deflection + .005
        press_advance = min(press_advance + .00120, lead)
        yield on_spike(press_advance, 'press the roll onto the mandrel', depth)
    else:
        raise RuntimeError(f'Mandrel did not open the cavity: {task.read_depth(base):.4f} m')
    for i in range(78):
        advance = press_advance * (1. - ik.smoothstep((i + 1) / 78))
        position, rotation, _ = task.pose(base, 'bun')
        inside = mouth_to_center - float((tip - position) @ rotation[:, 2])
        yield on_spike(advance, 'withdraw the roll straight from the mandrel', inside)
    if policy == 'pierce':
        yield from hold(12, 'show the opened roll below the mandrel')
        return
    bun_hold = held_transform('left', 'bun')

    final = np.asarray(BUN_FINAL)
    final_rotation = quat_xyzw_to_matrix(BUN_FINAL_ROTATION_XYZW)
    axis = final_rotation[:, 2]
    # Move laterally clear of the mandrel before tilting the roll.
    yield from carry('left', prepress + [0., -.091, 0.], upright, bun_hold, 55, 'carry the opened roll clear of the tool')
    above_stand = final + .045 * UP + .012 * axis
    yield from carry('left', above_stand, final_rotation, bun_hold, 68, 'tilt the roll to match the serving holder')
    yield from carry('left', final + .003 * UP, final_rotation, bun_hold, 63, 'seat the roll against the holder stop')
    yield from grip('left', .72, 18, 'release the supported roll')
    left_p, left_r = tcp('left')
    yield from move('left', left_p - .086 * left_r[:, 2] + .030 * UP, left_r, 54, 'clear the roll opening')
    yield from retreat_ready('left', 50, 'return left arm to ready')

    bun_p, bun_r, _ = task.pose(base, 'bun')
    if np.linalg.norm(bun_p - final) > .019 or bun_r[:, 2] @ axis < .970:
        raise RuntimeError(f'Roll did not settle in the serving holder: {bun_p.tolist()}')
    meat_p, meat_r, _ = task.pose(base, 'sausage')
    pickup = meat_p + meat_r @ SAUSAGE_GRIP_LOCAL + .006 * UP
    yield from move('right', pickup + .098 * UP, SAUSAGE_GRIP_FRAME, 64, 'approach the trailing end of the sausage')
    yield from grip('right', .34, 14, 'size the jaws for the sausage')
    yield from move('right', pickup, SAUSAGE_GRIP_FRAME, 46, 'place fingertips around the sausage')
    yield from grip('right', .105, 28, 'grasp the sausage through pad contact')
    yield from move('right', pickup + .168 * UP, SAUSAGE_GRIP_FRAME, 78, 'lift the whole sausage')
    yield from hold(18, 'settle the lifted sausage')
    lifted, rotation, _ = task.pose(base, 'sausage')
    lowest = lifted[2] - SAUSAGE_RADIUS - abs(rotation[2, 2]) * (SAUSAGE_LENGTH / 2 - SAUSAGE_RADIUS)
    if lowest < TABLE_Z + .020:
        raise RuntimeError(f'Right gripper did not lift the whole sausage clear: lowest point {lowest:.4f} m')
    sausage_hold = held_transform('right', 'sausage')
    bun_p, bun_r, _ = task.pose(base, 'bun')
    axis = bun_r[:, 2]
    mouth = bun_p + mouth_to_center * axis
    spacing = SAUSAGE_LENGTH / 2
    sausage_rotation = bun_r @ SAUSAGE_WRIST_ROLL
    front = mouth + (spacing + .029) * axis
    yield from carry('right', front, sausage_rotation, sausage_hold, 88, 'align the leading sausage tip with the cavity')

    correction = np.zeros(3)
    tilt = np.zeros(3)
    offset, wrist = sausage_hold

    def insert(start, end, count, phase):
        for i in range(count):
            f = ik.smoothstep((i + 1) / count)
            position, rotation, _ = task.pose(base, 'sausage')
            bun_position, bun_rotation, _ = task.pose(base, 'bun')
            opening_axis = bun_rotation[:, 2]
            target_tip = bun_position + (mouth_to_center + start + f * (end - start)) * opening_axis
            actual_tip = position - spacing * rotation[:, 2]
            error = target_tip - actual_tip
            correction[:] = np.clip(correction + np.clip(.13 * error, -.0013, .0013), -.008, .008)
            tilt[:] = np.clip(tilt + np.clip(.17 * np.cross(rotation[:, 2], opening_axis), -.008, .008), -.13, .13)
            angle = np.linalg.norm(tilt)
            aim = axis_angle_matrix(tilt / angle, angle) if angle > 1e-8 else upright
            rotation = aim @ bun_rotation @ SAUSAGE_WRIST_ROLL
            yield command('right', target_tip + correction + rotation @ (offset + spacing * UP),
                          rotation @ wrist, phase)

    yield from insert(.029, .004, 43, 'meet the opening along its axis')
    yield from insert(.004, -.101, 100, 'insert the sausage into the supported roll')
    yield from grip('right', .49, 18, 'release the inserted sausage')
    right_p, right_r = tcp('right')
    yield from move('right', right_p + .069 * axis - .044 * right_r[:, 2], right_r, 54, 'clear the finished hot dog')
    yield from retreat_ready('right', 65, 'return right arm to ready')
    yield from hold(min(40, max_steps - steps), 'show the released hot dog in its holder')
