"""Scorer for the servo bus commissioning task. Pure Python + numpy.

Call ``Scorer.update`` once per control step (20 Hz, dt = 0.05 s).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Mapping

import numpy as np

import geometry as g
from logic import CommissioningLogic, LogicState, SERVOS

# Thresholds
TOL = 1e-9  # Slack for floating point comparisons

SEATED_DEPTH = 0.016        # m, nose past socket mouth (>=); GPU: seated servos rest at 17.03 mm
SEATED_LATERAL = 0.0015     # m, lateral offset from socket axis (<=)
SEATED_ANG_DEG = 10.0       # deg, rotation from seated orientation (<=); GPU: servos rest at 3.9 deg at the start and 7.1 deg after a release (1 mm play)

FIXTURE_MOVE_LIMIT = 0.005  # m, fixture movement beyond 5 mm causes failure (>)
FIXTURE_TILT_LIMIT = 5.0    # deg, fixture tilt beyond 5 deg causes failure (>)
SERVO_FALL_LIMIT = 0.20     # m, below table causes failure (>)

STILL_LIN = 0.005           # m/s, linear speed for "still" (<=)
STILL_ANG = 0.2             # rad/s, angular speed for "still" (<=)

SCORE_NONE = 0.0
SCORE_TOUCHED = 0.25
SCORE_RED_COMMISSIONED = 0.5
SCORE_TWO_COMMISSIONED = 0.75
SCORE_SUCCESS = 1.0


def _vec(x, n: int, name: str) -> np.ndarray:
    a = np.array(x, dtype=float)
    if a.shape != (n,):
        raise ValueError(f"{name} must have shape ({n},), got {a.shape}")
    if not np.all(np.isfinite(a)):
        raise ValueError(f"{name} contains non-finite values: {a}")
    return a


def quat_to_matrix(q_xyzw) -> np.ndarray:
    q = _vec(q_xyzw, 4, "quaternion")
    n = float(np.linalg.norm(q))
    if n < 1e-12:
        raise ValueError("zero-length quaternion")
    x, y, z, w = q / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


R_SEATED = np.array([
    [0.0, 1.0, 0.0],
    [-1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0],
])  # yaw = -pi/2


def rotation_angle_deg(R1, R2) -> float:
    rel = R1.T @ R2
    c = (float(np.trace(rel)) - 1.0) / 2.0
    return math.degrees(math.acos(min(1.0, max(-1.0, c))))


def is_servo_seated_in_socket(pos, quat_xyzw, socket_y: float) -> bool:
    """Check if a servo is seated in the socket at socket_y."""
    # Nose depth past mouth along +x
    # When seated, origin is at MOUTH_X, nose tip is at MOUTH_X + NOSE_L
    depth = (pos[0] + g.NOSE_L) - g.MOUTH_X
    if depth < SEATED_DEPTH - TOL:
        return False

    # Lateral offsets
    dy = abs(pos[1] - socket_y)
    dz = abs(pos[2] - g.SOCKET_Z)
    if dy > SEATED_LATERAL + TOL or dz > SEATED_LATERAL + TOL:
        return False

    # Orientation
    R = quat_to_matrix(quat_xyzw)
    ang = rotation_angle_deg(R, R_SEATED)
    if ang > SEATED_ANG_DEG + TOL:
        return False

    return True


def is_servo_seated(pos, quat_xyzw) -> bool:
    """True if the servo is seated in any of the three sockets."""
    return any(is_servo_seated_in_socket(pos, quat_xyzw, sy) for sy in g.SOCKET_Y)


@dataclass
class StepResult:
    score: float
    success: bool
    details: dict = field(default_factory=dict)


class Scorer:
    def __init__(self) -> None:
        self.logic = CommissioningLogic()
        self.reset()

    def reset(self) -> None:
        self.logic.reset()
        self.fixture_start_pos: np.ndarray | None = None
        self.failed: bool = False
        self.touched_or_unplugged: bool = False
        self.highest_tier: float = 0.0

    def update(
        self,
        fixture_pos,
        fixture_quat_or_sliders,
        fixture_sliders_or_servos,
        servos_data_or_gripper=None,
        gripper_open_both: bool = True,
    ) -> StepResult:
        if isinstance(fixture_quat_or_sliders, (list, tuple, np.ndarray)) and len(fixture_quat_or_sliders) == 4:
            fixture_quat = fixture_quat_or_sliders
            fixture_sliders = fixture_sliders_or_servos
            servos_data = servos_data_or_gripper
            g_open = gripper_open_both
        else:
            fixture_quat = None
            fixture_sliders = fixture_quat_or_sliders
            servos_data = fixture_sliders_or_servos
            g_open = servos_data_or_gripper if servos_data_or_gripper is not None else True

        # 1. Capture fixture start position on first step
        fpos = np.asarray(fixture_pos, dtype=float)
        if self.fixture_start_pos is None:
            self.fixture_start_pos = fpos.copy()

        # 2. Check failure conditions (zero-gates)
        fixture_shift = float(np.linalg.norm(fpos - self.fixture_start_pos))
        if fixture_shift > FIXTURE_MOVE_LIMIT + TOL:
            self.failed = True

        if fixture_quat is not None:
            try:
                R_fix = quat_to_matrix(fixture_quat)
                tilt = rotation_angle_deg(R_fix, np.eye(3))
                if tilt > FIXTURE_TILT_LIMIT + TOL:
                    self.failed = True
            except Exception:
                pass

        for name in SERVOS:
            spos = np.asarray(servos_data[name][0], dtype=float)
            if spos[2] < g.TOP - SERVO_FALL_LIMIT - TOL:
                self.failed = True

        if self.failed:
            return StepResult(score=SCORE_NONE, success=False, details={"failed": True, "fixture_shift": fixture_shift})

        # 3. Check seated status for each servo
        seated = {}
        for name in SERVOS:
            pos, quat, _, _ = servos_data[name]
            seated[name] = is_servo_seated(pos, quat)

        all_seated = all(seated.values())

        # 4. Check button presses
        assign_q = float(fixture_sliders.get("assign_btn", 0.0))
        reset_q = float(fixture_sliders.get("reset_btn", 0.0))
        button_pressed = (assign_q <= -0.002) or (reset_q <= -0.002)

        if (not all_seated) or button_pressed:
            self.touched_or_unplugged = True

        # 5. Update logic state
        logic_state = self.logic.update(seated, assign_q, reset_q)

        # 6. Check velocities
        all_still = True
        for name in SERVOS:
            _, _, lin_vel, ang_vel = servos_data[name]
            l_spd = float(np.linalg.norm(lin_vel))
            a_spd = float(np.linalg.norm(ang_vel))
            if l_spd > STILL_LIN + TOL or a_spd > STILL_ANG + TOL:
                all_still = False
                break

        # 7. Evaluate score tiers
        tier = SCORE_NONE
        if self.touched_or_unplugged:
            tier = max(tier, SCORE_TOUCHED)

        if logic_state.ids.get("red") == 1:
            tier = max(tier, SCORE_RED_COMMISSIONED)

        if logic_state.ids.get("red") == 1 and logic_state.ids.get("green") == 2:
            tier = max(tier, SCORE_TWO_COMMISSIONED)

        # Full score condition:
        # red=1, green=2, blue=3, all three seated, lamp green, grippers open, all still
        commissioned_all = (
            logic_state.ids.get("red") == 1
            and logic_state.ids.get("green") == 2
            and logic_state.ids.get("blue") == 3
        )

        success = (
            commissioned_all
            and all_seated
            and logic_state.bus_lamp == "green"
            and g_open
            and all_still
        )

        if success:
            tier = SCORE_SUCCESS

        self.highest_tier = max(self.highest_tier, tier)

        details = {
            "ids": dict(logic_state.ids),
            "bus_lamp": logic_state.bus_lamp,
            "all_seated": all_seated,
            "all_still": all_still,
            "gripper_open_both": g_open,
            "fixture_shift": fixture_shift,
            "highest_tier": self.highest_tier,
        }

        # Highest tier reached counts, except 1.0 requires its simultaneous hold
        current_score = SCORE_SUCCESS if success else min(self.highest_tier, SCORE_TWO_COMMISSIONED)
        if self.touched_or_unplugged and current_score < SCORE_TOUCHED:
            current_score = SCORE_TOUCHED

        return StepResult(score=current_score, success=success, details=details)
