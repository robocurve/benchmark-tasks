"""Orient the stock YAM jaws for the tool, cap, and native latch pad."""

import math

import numpy as np
from yam_arena import demo
from yam_arena.urdf import axis_angle_matrix


class TiltedYaw(float):
    def __new__(cls, yaw, tilt, roll=0., heading=0.):
        value = super().__new__(cls, yaw)
        value.tilt = tilt
        value.roll = roll
        value.heading = heading
        return value


_vertical_rotation = demo.grip_rotation


def grip_rotation(yaw):
    rotation = _vertical_rotation(yaw)
    if isinstance(yaw, TiltedYaw):
        tilt = axis_angle_matrix(np.array([1., 0., 0.]), yaw.tilt)
        roll = axis_angle_matrix(np.array([0., 1., 0.]), yaw.roll)
        heading = axis_angle_matrix(np.array([0., 0., 1.]), yaw.heading)
        return heading @ tilt @ roll @ rotation
    return rotation


demo.grip_rotation = grip_rotation
WITHDRAW_YAW = TiltedYaw(-math.pi / 2, math.radians(-80))
ALIGNED_DRAW_YAW = TiltedYaw(-math.pi / 2, math.radians(-78.4))
SETDOWN_YAW = TiltedYaw(-math.pi / 2, math.radians(-37))
PRESS_HEADING = math.radians(-60)
PRESS_ROLL = math.radians(70)
PRESS_YAW = TiltedYaw(0., 0., PRESS_ROLL, PRESS_HEADING)

PAD_HOLD = "hold the latch against the rear pad"
PAD_DRAW = "hold the rear pad while withdrawing the plug"
PAD_LIFT = "lift the pick once the teeth have crossed"
PAD_CONTROL_LIFT = "lift the pick straight away before releasing the blocked plug"


def track_pad(read_position, pad_y, loaded_z):
    """Keep the expert tool over the moving pad while the plug clears the shoulders."""
    stock = demo.ArmPlan.target
    if getattr(stock, "_tracks_connector", False):
        return

    def target(plan, t):
        pos, rotation, fingers, label = stock(plan, t)
        if plan.base_pos[1] >= 0 or label not in (PAD_HOLD, PAD_DRAW, PAD_LIFT, PAD_CONTROL_LIFT):
            return pos, rotation, fingers, label
        position = read_position()
        if position is None:
            return pos, rotation, fingers, label
        if label == PAD_HOLD:
            plan._pad_hold_position = position.copy()
        elif hasattr(plan, "_pad_hold_position"):
            delta = np.clip(position - plan._pad_hold_position, [-.002, -.001, -.002], [.002, .024, .002])
            pos = pos.copy()
            if label == PAD_LIFT and delta[1] < .0065:
                pos[2] = loaded_z
            pos[[0, 2]] += delta[[0, 2]]
            pos[1] = pad_y + delta[1]
        return pos, rotation, fingers, label

    target._tracks_connector = True
    demo.ArmPlan.target = target
