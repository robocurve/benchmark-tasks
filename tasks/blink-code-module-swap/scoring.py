"""Scorer for the blink-code module swap task. Pure Python + numpy.

Call ``Scorer.update`` once per control step (20 Hz, dt = 0.05 s).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Mapping

import numpy as np

import geometry as g
from logic import (
    ALL_MODULES,
    FAILED_MODULE,
    FAILED_SOCKET,
    HEALTHY_MODULES,
    SPARE_MODULE,
    BlinkCodeLogic,
    LogicState,
)

TOL = 1e-9

SEATED_DEPTH = 0.016        # m, nose past socket mouth (>=)
SEATED_LATERAL = 0.0015     # m, lateral offset from socket axis (<=)
SEATED_ANG_DEG = 10.0       # deg, rotation from seated orientation (<=)

FIXTURE_MOVE_LIMIT = 0.005  # m, fixture displacement limit (>)
FIXTURE_TILT_LIMIT = 5.0    # deg, fixture tilt limit (>)
MODULE_FALL_LIMIT = 0.20    # m, below table causes failure (>)

STILL_LIN = 0.005           # m/s, linear speed for still (<=)
STILL_ANG = 0.2             # rad/s, angular speed for still (<=)

SCORE_NONE = 0.0
SCORE_TOUCHED = 0.25
SCORE_PULLED = 0.50
SCORE_SWAPPED = 0.75
SCORE_SUCCESS = 1.0

SUCCESS_HOLD_STEPS = 20     # 1.0 s at 20 Hz
HEALTHY_UNSEATED_TOL_STEPS = 20  # 1.0 s tolerance before removal is flagged


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


def is_module_seated_in_socket(pos, quat_xyzw, socket_idx: int) -> bool:
    """Check if a module is seated in socket socket_idx."""
    socket_y = g.SOCKET_Y[socket_idx]
    # Nose depth past mouth along +x
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


def get_seated_socket(pos, quat_xyzw) -> int | None:
    """Return the socket index (0..3) if seated in any socket, else None."""
    for idx in range(len(g.SOCKET_Y)):
        if is_module_seated_in_socket(pos, quat_xyzw, idx):
            return idx
    return None


@dataclass
class StepResult:
    score: float
    success: bool
    details: dict = field(default_factory=dict)


class Scorer:
    def __init__(self) -> None:
        self.logic = BlinkCodeLogic()
        self.reset()

    def reset(self) -> None:
        self.logic.reset()
        self.fixture_start_pos: np.ndarray | None = None
        self.failed: bool = False
        self.touched: bool = False
        self.highest_tier: float = SCORE_NONE
        self.success_hold_count: int = 0
        self.healthy_unseated_duration: dict[str, int] = {m: 0 for m in HEALTHY_MODULES}

    def update(
        self,
        fixture_pos,
        fixture_quat_or_sliders,
        fixture_sliders_or_modules,
        modules_data_or_gripper=None,
        gripper_open_both: bool = True,
    ) -> StepResult:
        if isinstance(fixture_quat_or_sliders, (list, tuple, np.ndarray)) and len(fixture_quat_or_sliders) == 4:
            fixture_quat = fixture_quat_or_sliders
            fixture_sliders = fixture_sliders_or_modules
            modules_data = modules_data_or_gripper
            g_open = gripper_open_both
        else:
            fixture_quat = None
            fixture_sliders = fixture_quat_or_sliders
            modules_data = fixture_sliders_or_modules
            g_open = modules_data_or_gripper if modules_data_or_gripper is not None else True

        # 1. Capture fixture start position
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

        for name in ALL_MODULES:
            if name in modules_data:
                mpos = np.asarray(modules_data[name][0], dtype=float)
                if mpos[2] < g.TOP - MODULE_FALL_LIMIT - TOL:
                    self.failed = True

        # 3. Check seated status for each module
        seated_in_socket: dict[str, int | None] = {}
        for name in ALL_MODULES:
            if name in modules_data:
                pos, quat, _, _ = modules_data[name]
                seated_in_socket[name] = get_seated_socket(pos, quat)
            else:
                seated_in_socket[name] = None

        # Track healthy module removal
        healthy_target = {"module_1": 0, "module_2": 1, "module_4": 3}
        for m, target_sock in healthy_target.items():
            if seated_in_socket.get(m) != target_sock:
                self.healthy_unseated_duration[m] += 1
                # If removed for > 1 s and currently unseated: failure zero-gate
                if self.healthy_unseated_duration[m] > HEALTHY_UNSEATED_TOL_STEPS:
                    self.failed = True
            else:
                self.healthy_unseated_duration[m] = 0

        if self.failed:
            return StepResult(score=SCORE_NONE, success=False, details={"failed": True, "fixture_shift": fixture_shift})

        # 4. Check button slider and update logic
        diag_q = float(fixture_sliders.get("diag_btn", 0.0))
        if diag_q <= -0.002:
            self.touched = True

        # Check if any module moved significantly from starting pose
        for name, start_p in g.START_POS.items():
            if name in modules_data:
                p = np.asarray(modules_data[name][0], dtype=float)
                if float(np.linalg.norm(p - np.array(start_p))) > 0.005:
                    self.touched = True

        logic_state: LogicState = self.logic.update(seated_in_socket, diag_q)
        if logic_state.diag_held_ge_3s:
            self.touched = True

        # 5. Check velocities
        all_still = True
        for name in ALL_MODULES:
            if name in modules_data:
                _, _, lin_vel, ang_vel = modules_data[name]
                if float(np.linalg.norm(lin_vel)) > STILL_LIN + TOL or float(np.linalg.norm(ang_vel)) > STILL_ANG + TOL:
                    all_still = False
                    break

        # 6. Evaluate score tiers
        tier = SCORE_NONE
        if self.touched:
            tier = max(tier, SCORE_TOUCHED)

        # 0.50: failed module out of socket 3 with healthy modules seated
        healthy_seated = (
            seated_in_socket.get("module_1") == 0
            and seated_in_socket.get("module_2") == 1
            and seated_in_socket.get("module_4") == 3
        )
        failed_out = seated_in_socket.get(FAILED_MODULE) != FAILED_SOCKET
        if healthy_seated and failed_out:
            tier = max(tier, SCORE_PULLED)

        # 0.75: spare seated in socket 3, failed module not in any socket, healthy modules seated
        spare_in_failed_socket = seated_in_socket.get(SPARE_MODULE) == FAILED_SOCKET
        failed_not_in_any = seated_in_socket.get(FAILED_MODULE) is None
        if healthy_seated and spare_in_failed_socket and failed_not_in_any:
            tier = max(tier, SCORE_SWAPPED)

        # 1.00: verified with green lamp held >= 3s, all 4 sockets seated, grippers open, still, held 1.0s
        all_four_seated = healthy_seated and spare_in_failed_socket and failed_not_in_any
        instant_success = (
            logic_state.green_verified
            and all_four_seated
            and g_open
            and all_still
        )

        if instant_success:
            self.success_hold_count += 1
        else:
            self.success_hold_count = 0

        success = self.success_hold_count >= SUCCESS_HOLD_STEPS
        if success:
            tier = SCORE_SUCCESS

        self.highest_tier = max(self.highest_tier, min(tier, SCORE_SWAPPED))

        details = {
            "lamp_state": logic_state.lamp_state,
            "swap_completed": logic_state.swap_completed,
            "healthy_seated": healthy_seated,
            "green_verified": logic_state.green_verified,
            "all_four_seated": all_four_seated,
            "all_still": all_still,
            "gripper_open_both": g_open,
            "fixture_shift": fixture_shift,
            "highest_tier": self.highest_tier,
            "success_hold_count": self.success_hold_count,
        }

        current_score = SCORE_SUCCESS if success else self.highest_tier
        return StepResult(score=current_score, success=success, details=details)
