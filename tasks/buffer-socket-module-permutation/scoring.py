"""Scorer for the buffer socket module permutation task. Pure Python + NumPy.

Call ``Scorer.update`` once per control step (20 Hz, dt = 0.05 s).

Scoring ladder:
- 0.00: Initial resting state or failure.
- 0.25: Any module pulled out (insertion depth < 16 mm from starting socket).
- 0.50: One mis-racked module placed in its matching socket.
- 0.75: Both mis-racked modules placed in their matching sockets (two matching).
- 1.00: All three matching, COMMIT pressed with STATUS lamp green, grippers open,
        and all bodies still for 1.0 s (20 control steps).

Zero-gates:
- Fixture displaced > 5 mm from starting position.
- Fixture tilted > 5 degrees.
- Any module dropped > 20 cm below the table top.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Mapping

import numpy as np

import geometry as g
from logic import (
    MODULE_NAMES,
    TARGET_SOCKETS,
    LogicState,
    PermutationLogic,
)

TOL = 1e-9

SEATED_DEPTH = 0.016        # m, nose past socket mouth (>= 16 mm of 18 mm)
SEATED_LATERAL = 0.0015     # m, lateral offset from socket axis (<= 1.5 mm)
SEATED_ANG_DEG = 10.0       # deg, rotation from seated orientation (<= 10 deg)

FIXTURE_MOVE_LIMIT = 0.005  # m, fixture displacement limit (> 5 mm)
FIXTURE_TILT_LIMIT = 5.0    # deg, fixture tilt limit (> 5 deg)
MODULE_FALL_LIMIT = 0.20    # m, below table top causes failure (> 20 cm)

STILL_LIN = 0.005           # m/s, linear speed for still (<=)
STILL_ANG = 0.2             # rad/s, angular speed for still (<=)

SCORE_NONE = 0.0
SCORE_PULLED = 0.25
SCORE_ONE_MATCHING = 0.50
SCORE_TWO_MATCHING = 0.75
SCORE_SUCCESS = 1.0

SUCCESS_HOLD_STEPS = 20     # 1.0 s at 20 Hz


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
    if quat_xyzw is not None:
        R = quat_to_matrix(quat_xyzw)
        ang = rotation_angle_deg(R, R_SEATED)
        if ang > SEATED_ANG_DEG + TOL:
            return False

    return True


def find_seated_socket(pos, quat_xyzw) -> int | None:
    """Return the socket index (0..3) if the module is seated in one, else None."""
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
        self.logic = PermutationLogic()
        self.reset()

    def reset(self) -> None:
        self.logic.reset()
        self.fixture_start_pos: np.ndarray | None = None
        self.fixture_start_quat: np.ndarray | None = None
        self.failed: bool = False
        self.fail_reason: str = ""
        self.has_pulled_module: bool = False
        self.highest_tier: float = 0.0
        self.hold_steps: int = 0

    def update(
        self,
        fixture_pos,
        fixture_quat_or_sliders,
        fixture_sliders_or_modules,
        modules_data_or_gripper=None,
        gripper_open_both: bool = True,
    ) -> StepResult:
        """Evaluate one control step.

        Accepts either 4 arguments (without quat) or 5 arguments (with quat).
        """
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

        # 1. Capture fixture start pose
        fpos = np.asarray(fixture_pos, dtype=float)
        if self.fixture_start_pos is None:
            self.fixture_start_pos = fpos.copy()
        if fixture_quat is not None and self.fixture_start_quat is None:
            self.fixture_start_quat = np.asarray(fixture_quat, dtype=float).copy()

        # 2. Check zero-gates
        fixture_shift = float(np.linalg.norm(fpos - self.fixture_start_pos))
        if fixture_shift > FIXTURE_MOVE_LIMIT + TOL:
            self.failed = True
            self.fail_reason = f"fixture_moved_{fixture_shift*1000:.1f}mm"

        if fixture_quat is not None and self.fixture_start_quat is not None:
            try:
                R_fix = quat_to_matrix(fixture_quat)
                R_start = quat_to_matrix(self.fixture_start_quat)
                tilt = rotation_angle_deg(R_fix, R_start)
                if tilt > FIXTURE_TILT_LIMIT + TOL:
                    self.failed = True
                    self.fail_reason = f"fixture_tilted_{tilt:.1f}deg"
            except Exception:
                pass

        for name in MODULE_NAMES:
            if name in modules_data:
                mpos = np.asarray(modules_data[name][0], dtype=float)
                if mpos[2] < g.TOP - MODULE_FALL_LIMIT - TOL:
                    self.failed = True
                    self.fail_reason = f"{name}_fallen"

        if self.failed:
            return StepResult(
                score=SCORE_NONE,
                success=False,
                details={
                    "failed": True,
                    "fail_reason": self.fail_reason,
                    "fixture_shift": fixture_shift,
                },
            )

        # 3. Detect seated sockets and pulled status
        seated_in_socket: dict[str, int | None] = {}
        any_displaced = False

        for name in MODULE_NAMES:
            pos, quat, lin_vel, ang_vel = modules_data[name]
            socket_idx = find_seated_socket(pos, quat)
            seated_in_socket[name] = socket_idx

            # Check if pulled from starting socket
            start_sock = g.START_SOCKET[name]
            # Nose depth in start socket:
            start_depth = (pos[0] + g.NOSE_L) - g.MOUTH_X
            start_y = g.SOCKET_Y[start_sock]
            if (
                start_depth < SEATED_DEPTH - TOL
                or abs(pos[1] - start_y) > SEATED_LATERAL + TOL
                or abs(pos[2] - g.SOCKET_Z) > SEATED_LATERAL + TOL
            ):
                self.has_pulled_module = True

            # If not seated in any socket, or speed is high, it is displaced/in-hand
            if socket_idx is None:
                any_displaced = True
            else:
                l_spd = float(np.linalg.norm(lin_vel))
                a_spd = float(np.linalg.norm(ang_vel))
                if l_spd > STILL_LIN + TOL or a_spd > STILL_ANG + TOL:
                    any_displaced = True

        # 4. Check COMMIT slider and update logic
        commit_q = float(fixture_sliders.get("commit_btn", 0.0))
        logic_state: LogicState = self.logic.update(
            seated_in_socket,
            commit_q,
            any_displaced=any_displaced,
        )

        # 5. Check still velocities
        all_still = not any_displaced
        if all_still:
            for name in MODULE_NAMES:
                _, _, lin_vel, ang_vel = modules_data[name]
                if float(np.linalg.norm(lin_vel)) > STILL_LIN + TOL or float(np.linalg.norm(ang_vel)) > STILL_ANG + TOL:
                    all_still = False
                    break

        # 6. Evaluate partial credit ladder
        tier = SCORE_NONE
        if self.has_pulled_module:
            tier = max(tier, SCORE_PULLED)
            if logic_state.matching_swapped_count >= 1:
                tier = max(tier, SCORE_ONE_MATCHING)
            if logic_state.matching_swapped_count >= 2:
                tier = max(tier, SCORE_TWO_MATCHING)

        self.highest_tier = max(self.highest_tier, tier)

        # 7. Evaluate 1.00 success condition:
        # All three matching, COMMIT was pressed validly while STATUS was green,
        # both grippers open, and all still for 20 steps (1.0 s).
        ready_for_success = (
            logic_state.all_three_matching
            and logic_state.commit_pressed_valid
            and g_open
            and all_still
        )

        if ready_for_success:
            self.hold_steps += 1
        else:
            self.hold_steps = 0

        is_success = self.hold_steps >= SUCCESS_HOLD_STEPS
        current_score = SCORE_SUCCESS if is_success else self.highest_tier

        return StepResult(
            score=current_score,
            success=is_success,
            details={
                "failed": False,
                "highest_tier": self.highest_tier,
                "has_pulled": self.has_pulled_module,
                "hold_steps": self.hold_steps,
                "seated": seated_in_socket,
                "lamp_state": logic_state.lamp_state,
                "commit_held": logic_state.commit_held,
                "commit_valid": logic_state.commit_pressed_valid,
                "all_three_matching": logic_state.all_three_matching,
                "matching_swapped": logic_state.matching_swapped_count,
                "matching_total": logic_state.matching_total_count,
                "grippers_open": g_open,
                "all_still": all_still,
            },
        )
