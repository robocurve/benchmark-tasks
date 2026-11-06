"""Scoring from measured states. History is reset per episode; violations are permanent.

Physics samples are supplied by a native post-physics callback at 240 Hz.
Control/end-state samples arrive at 20 Hz. This module does not simulate physics.
"""

from collections import deque
from dataclasses import dataclass

import numpy as np

from .config import (
    MAX_TILT_RAD, MAX_UNSUPPORTED_DESCENT, MIN_ROBOT_CLEARANCE, MOUNT_POSITION,
    PAD_HALF_SIZE, PAD_POSITION, PAD_TOP, RELEASE_ANGLE, TABLE_TOP,
)
from .geometry import angle_between, payload_corners, payload_mount_gap, rotation_xyzw


@dataclass
class Sample:
    time: float
    position: np.ndarray
    quaternion: np.ndarray
    velocity: np.ndarray
    cam_angle: float
    stand_position: np.ndarray
    stand_quaternion: np.ndarray
    cam_anchor_error: float
    support_contact: bool
    robot_clearance: float


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.violations = set()
        self.unsupported_peak = None
        self.max_unsupported_descent = 0.0
        self.physics_samples = 0
        self.history = deque()

    def observe_physics(self, position, quaternion, support_contact):
        self.physics_samples += 1
        try:
            pts = payload_corners(position, quaternion)
            finite = np.isfinite(position).all()
        except (ValueError, TypeError):
            self.violations.add("invalid_state")
            return
        if not finite:
            self.violations.add("invalid_state")
            return
        lo, hi = pts.min(axis=0), pts.max(axis=0)
        if (lo[0] < -0.19 or hi[0] > 0.91 or lo[1] < -0.65 or hi[1] > 0.65
                or lo[2] < TABLE_TOP-0.005 or hi[2] > TABLE_TOP+0.5):
            self.violations.add("workspace_escape")
        z = float(position[2])
        if support_contact:
            self.unsupported_peak = None
        else:
            self.unsupported_peak = z if self.unsupported_peak is None else max(self.unsupported_peak, z)
            descent = self.unsupported_peak-z
            self.max_unsupported_descent = max(self.max_unsupported_descent, descent)
            if descent > MAX_UNSUPPORTED_DESCENT:
                self.violations.add("drop")

    def evaluate(self, state: Sample):
        try:
            rotation = rotation_xyzw(state.quaternion)
            rotation_xyzw(state.stand_quaternion)
            values = [state.time, *state.position, *state.velocity, state.cam_angle,
                      *state.stand_position, state.cam_anchor_error, state.robot_clearance]
            if not np.isfinite(values).all():
                raise ValueError("Non-finite task measurement")
        except (ValueError, TypeError):
            self.violations.add("invalid_state")
            return self._result(0.0, False, {})
        fixture_shift = float(np.linalg.norm(state.stand_position-MOUNT_POSITION))
        fixture_angle = angle_between(state.stand_quaternion, np.array([0, 0, 0, 1]))
        if fixture_shift > 0.005 or fixture_angle > np.deg2rad(5):
            self.violations.add("fixture_moved")
        if state.cam_anchor_error > 0.002 or not -0.01 <= state.cam_angle <= 0.96:
            self.violations.add("apparatus_detached")
        pts = payload_corners(state.position, state.quaternion)
        lo, hi = pts.min(axis=0), pts.max(axis=0)
        horizontal = float(np.linalg.norm(state.position[:2]-PAD_POSITION[:2]))
        height_error = abs(float(lo[2])-PAD_TOP)
        tilt = float(np.arccos(np.clip(rotation[2, 2], -1, 1)))
        on_pad = bool(
            horizontal <= 0.020 and height_error <= 0.005 and tilt <= MAX_TILT_RAD
            and np.all(lo[:2] >= PAD_POSITION[:2]-PAD_HALF_SIZE)
            and np.all(hi[:2] <= PAD_POSITION[:2]+PAD_HALF_SIZE)
            and state.support_contact
        )
        gap = payload_mount_gap(state.position, state.quaternion, state.cam_angle)
        clear = state.robot_clearance >= MIN_ROBOT_CLEARANCE
        eligible = on_pad and clear and not self.violations
        if eligible:
            self.history.append((state.time, state.position.copy(), state.quaternion.copy()))
            while len(self.history) > 1 and state.time-self.history[1][0] >= 1.0-1e-6:
                self.history.popleft()
        else:
            self.history.clear()
        stable = False
        if self.history and state.time-self.history[0][0] >= 1.0-1e-6:
            p0, q0 = self.history[0][1:]
            stable = all(np.linalg.norm(p-p0) <= 0.002 and angle_between(q, q0) <= np.deg2rad(2)
                         for _, p, q in self.history)
        success = bool(eligible and stable)
        score = 1.0 if success else 0.75 if on_pad else 0.5 if gap >= 0.010 and state.support_contact else (
            0.25 if state.cam_angle >= RELEASE_ANGLE and gap < 0.010 and state.support_contact else 0.0
        )
        if self.violations:
            score, success = 0.0, False
        return self._result(score, success, {
            "cam_angle_rad": state.cam_angle, "payload_position_m": state.position.tolist(),
            "payload_quaternion_xyzw": state.quaternion.tolist(), "pad_error_m": horizontal,
            "pad_height_error_m": height_error, "tilt_rad": tilt,
            "fixture_shift_m": fixture_shift, "fixture_angle_rad": fixture_angle,
            "cam_anchor_error_m": state.cam_anchor_error, "mount_gap_lower_bound_m": gap,
            "robot_clearance_lower_bound_m": state.robot_clearance,
            "support_contact": state.support_contact, "on_pad": on_pad,
            "released": clear, "stable": stable,
        })

    def _result(self, score, success, metrics):
        return dict(score=float(score), success=bool(success), violations=sorted(self.violations),
                    physics_samples=self.physics_samples,
                    max_unsupported_descent_m=self.max_unsupported_descent, **metrics)
