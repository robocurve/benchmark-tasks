"""Score contact poses of a pointed tool fixed to a gripper."""

import math

import numpy as np

REFERENCE = np.array([0.27, 0.22, 0.7985])
TIP_LOCAL = np.array([0., 0., 0.088])
CONTACT_RADIUS_M = 0.001
CONTACT_Z_M = 0.00025
MOUNT_DRIFT_M, MOUNT_DRIFT_DEG = 0.0005, 0.5
TIP_DRIFT_M, AXIS_DRIFT_DEG = 0.0005, 2.0
MIN_TILT_DEG, MAX_TILT_DEG, MIN_SPREAD_DEG = 10., 35., 15.
MIN_FORCE_N = 0.02


def angle_deg(a, b):
    return math.degrees(math.acos(float(np.clip(np.dot(a, b), -1., 1.))))


class CalibrationScore:
    def __init__(self, control_hz=20):
        self.hold_steps = round(float(control_hz))
        self.reset()

    def reset(self):
        self.success = False
        self.axes = []
        self.tips = []
        self.anchor = None
        self.held = 0

    def update(self, tool_pos, tool_rot, gripper_pos, gripper_rot, contact_force_n):
        tool_pos, gripper_pos = np.asarray(tool_pos), np.asarray(gripper_pos)
        tool_rot, gripper_rot = np.asarray(tool_rot), np.asarray(gripper_rot)
        tip = tool_pos + tool_rot @ TIP_LOCAL
        axis = -tool_rot[:, 2]
        mount_rotation = math.degrees(math.acos(float(np.clip(
            (np.trace(gripper_rot.T @ tool_rot)-1.)/2., -1., 1.))))
        tilt = angle_deg(axis, np.array([0., 0., 1.]))
        valid = (np.linalg.norm(tool_pos-gripper_pos) <= MOUNT_DRIFT_M
                 and mount_rotation <= MOUNT_DRIFT_DEG
                 and contact_force_n >= MIN_FORCE_N
                 and np.linalg.norm(tip[:2]-REFERENCE[:2]) <= CONTACT_RADIUS_M
                 and abs(tip[2]-REFERENCE[2]) <= CONTACT_Z_M
                 and MIN_TILT_DEG <= tilt <= MAX_TILT_DEG)
        if not valid:
            self.anchor, self.held = None, 0
            return len(self.axes)/4
        if (self.anchor is None or np.linalg.norm(tip-self.anchor[0]) > TIP_DRIFT_M
                or angle_deg(axis, self.anchor[1]) > AXIS_DRIFT_DEG):
            self.anchor, self.held = (tip.copy(), axis.copy()), 1
        else:
            self.held += 1
        if (self.held >= self.hold_steps and len(self.axes) < 4
                and all(angle_deg(self.anchor[1], old) >= MIN_SPREAD_DEG for old in self.axes)):
            self.axes.append(self.anchor[1].copy())
            self.tips.append(self.anchor[0].copy())
        self.success = len(self.axes) == 4
        return len(self.axes)/4
