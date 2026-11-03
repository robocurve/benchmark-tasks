"""Scorer for the keyed-plug / loose-socket task. Pure Python + numpy, no simulator needed.

All inputs are in the WORLD frame (x forward, y left, z up, metres, quaternions as x, y, z, w) and refer to the
body origins. The plug's pose is judged in the block's own frame (``p_local = R_block^T (plug_pos - block_pos)``,
``rel = R_block^T R_plug``), so the score does not depend on where or how the loose block happens to lie.

Call ``Scorer.update`` once per control step (20 Hz, dt = 0.05 s). ``reset()`` clears every latch; the first
``update`` after a reset captures the starting poses (plug height, block position) the later rules refer to.

Every number below is a named constant. Comparisons are made with a 1e-9 slack (``TOL``, one nanometre or
nano-degree) so a value that sits exactly on a boundary is classified by the rule as written despite float
round-off: ``<=`` / ``>=`` thresholds include their boundary, ``<`` / ``>`` thresholds exclude it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

try:  # imported as part of a package
    from .geometry import MOUTH_Y, NOSE_L
except ImportError:  # imported with the task folder on sys.path
    from geometry import MOUTH_Y, NOSE_L

# -- thresholds ----------------------------------------------------------------------------------------------------
TOL = 1e-9  # slack for float round-off at a boundary (m, degrees, m/s, rad/s)

SEATED_DEPTH = 0.017  # m, nose tip past the mouth, "seated"
SEATED_ANG_DEG = 3.0  # degrees from the socket axes, "seated"
SEATED_LATERAL = 0.0015  # m, |x| and |z| of the plug origin in the block frame, "seated"
DETENT_MIN = -0.0003  # m, "seated" needs detent > this (plunger out in its groove, not pressed back)

ENGAGED_DEPTH = 0.006  # m, "engaged" tier
ENGAGED_ANG_DEG = 5.0  # degrees, "engaged" tier
ENGAGED_LATERAL = 0.003  # m, "engaged" tier

BLOCK_MOVED_OK = 0.005  # m, block xy displacement allowed for the 1.0 / 0.75 tiers (<=)
BLOCK_MOVED_FAIL = 0.03  # m, block xy displacement beyond this fails the episode (>)
BLOCK_TILT_FAIL_DEG = 20.0  # degrees, block z axis from world z beyond this fails the episode (>)
PLUG_FALL = 0.02  # m, plug origin further below its starting height than this fails the episode (>)

LIFT_HEIGHT = 0.03  # m, plug origin this far above its starting height counts as "lifted" (>=, latched)
NEAR_DIST = 0.10  # m, plug origin to the socket mouth for the "near" condition (<=)

STILL_LIN = 0.005  # m/s, plug and block linear speed must be below this to be "still"
STILL_ANG = 0.2  # rad/s, plug and block angular speed must be below this to be "still"

SCORE_SUCCESS = 1.0
SCORE_SEATED_HELD = 0.75
SCORE_ENGAGED = 0.5
SCORE_LIFTED_NEAR = 0.25
SCORE_NONE = 0.0

TIER_FAILED = "failed"
TIER_SUCCESS = "success"
TIER_SEATED_HELD = "seated_held_or_moving"
TIER_SEATED_BLOCK_MOVED = "seated_block_moved"
TIER_ENGAGED = "engaged"
TIER_LIFTED_NEAR = "lifted_near"
TIER_NONE = "none"


# -- pure maths ----------------------------------------------------------------------------------------------------
def _vec(x, n: int, name: str) -> np.ndarray:
    """A float copy of ``x`` with shape ``(n,)``; raises ``ValueError`` on a wrong shape or non-finite values."""
    a = np.array(x, dtype=float)
    if a.shape != (n,):
        raise ValueError(f"{name} must have shape ({n},), got {a.shape}")
    if not np.all(np.isfinite(a)):
        raise ValueError(f"{name} contains non-finite values: {a}")
    return a


def quat_to_matrix(q_xyzw) -> np.ndarray:
    """Rotation matrix of a quaternion given as (x, y, z, w). The input is normalised first, so any non-zero
    length is accepted; ``q`` and ``-q`` give the same matrix. Raises ``ValueError`` for a zero or non-finite one."""
    q = _vec(q_xyzw, 4, "quaternion")
    n = float(np.linalg.norm(q))
    if n < 1e-12:
        raise ValueError("zero-length quaternion")
    x, y, z, w = q / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def plug_pose_in_block(plug_pos, R_plug, block_pos, R_block):
    """The plug's pose in the block frame: ``(p_local, rel)`` with ``p_local = R_block^T (plug_pos - block_pos)``
    and ``rel = R_block^T R_plug``."""
    Rb = np.asarray(R_block, dtype=float)
    p_local = Rb.T @ (np.asarray(plug_pos, dtype=float) - np.asarray(block_pos, dtype=float))
    rel = Rb.T @ np.asarray(R_plug, dtype=float)
    return p_local, rel


def insertion_depth(p_local) -> float:
    """How far the nose tip is past the socket mouth (m): ``clip((y + NOSE_L) - MOUTH_Y, 0, NOSE_L)``."""
    return float(np.clip((float(p_local[1]) + NOSE_L) - MOUTH_Y, 0.0, NOSE_L))


def lateral_offset(p_local) -> float:
    """Sideways offset of the plug origin from the socket axis (m): ``max(|x|, |z|)`` in the block frame."""
    return float(max(abs(float(p_local[0])), abs(float(p_local[2]))))


def rotation_angle_deg(R) -> float:
    """Angle of a rotation matrix from identity, in degrees: ``acos(clip((trace - 1) / 2, -1, 1))``."""
    c = (float(np.trace(np.asarray(R, dtype=float))) - 1.0) / 2.0
    return math.degrees(math.acos(min(1.0, max(-1.0, c))))


def block_tilt_deg(R_block) -> float:
    """Angle between the block's z axis and world z, in degrees (yaw alone gives 0)."""
    c = float(np.asarray(R_block, dtype=float)[2, 2])
    return math.degrees(math.acos(min(1.0, max(-1.0, c))))


def xy_displacement(pos, start_pos) -> float:
    """Norm of the xy displacement of ``pos`` from ``start_pos`` (m); z is ignored."""
    d = np.asarray(pos, dtype=float)[:2] - np.asarray(start_pos, dtype=float)[:2]
    return float(np.linalg.norm(d))


def mouth_world(block_pos, R_block) -> np.ndarray:
    """The socket mouth centre (block frame ``(0, MOUTH_Y, 0)``) in the world frame."""
    return np.asarray(block_pos, dtype=float) + np.asarray(R_block, dtype=float) @ np.array([0.0, MOUTH_Y, 0.0])


# float-tolerant comparisons: the boundary value itself is classified as the rule's inequality says
def _le(a: float, b: float) -> bool:
    return a <= b + TOL


def _ge(a: float, b: float) -> bool:
    return a >= b - TOL


def _gt(a: float, b: float) -> bool:
    return a > b + TOL


def _lt(a: float, b: float) -> bool:
    return a < b - TOL


# -- the scorer ----------------------------------------------------------------------------------------------------
@dataclass
class StepResult:
    score: float
    success: bool
    details: dict = field(default_factory=dict)


class Scorer:
    """Stateful per-episode scorer; see the module docstring."""

    def __init__(self):
        self.reset()

    def reset(self):
        """Forget everything: the next ``update`` is the first of a new episode and captures the starting poses."""
        self._start_plug_z = None
        self._start_block_xy = None
        self._lifted_ever = False
        self._fail_reason = None

    @property
    def failed(self) -> bool:
        return self._fail_reason is not None

    def update(
        self,
        plug_pos,
        plug_quat_xyzw,
        plug_lin_vel,
        plug_ang_vel,
        block_pos,
        block_quat_xyzw,
        block_lin_vel,
        block_ang_vel,
        detent: float,
        grippers_open: bool,
    ) -> StepResult:
        plug_pos = _vec(plug_pos, 3, "plug_pos")
        block_pos = _vec(block_pos, 3, "block_pos")
        plug_lin = float(np.linalg.norm(_vec(plug_lin_vel, 3, "plug_lin_vel")))
        plug_ang = float(np.linalg.norm(_vec(plug_ang_vel, 3, "plug_ang_vel")))
        block_lin = float(np.linalg.norm(_vec(block_lin_vel, 3, "block_lin_vel")))
        block_ang = float(np.linalg.norm(_vec(block_ang_vel, 3, "block_ang_vel")))
        detent = float(detent)
        if not math.isfinite(detent):
            raise ValueError(f"detent is not finite: {detent}")
        grippers_open = bool(grippers_open)
        R_plug = quat_to_matrix(plug_quat_xyzw)
        R_block = quat_to_matrix(block_quat_xyzw)

        if self._start_plug_z is None:  # first update since reset: capture the starting poses
            self._start_plug_z = float(plug_pos[2])
            self._start_block_xy = block_pos[:2].copy()

        # quantities
        p_local, rel = plug_pose_in_block(plug_pos, R_plug, block_pos, R_block)
        depth = insertion_depth(p_local)
        lateral = lateral_offset(p_local)
        ang = rotation_angle_deg(rel)
        block_moved = xy_displacement(block_pos, self._start_block_xy)
        block_tilt = block_tilt_deg(R_block)
        height_gain = float(plug_pos[2]) - self._start_plug_z
        near_dist = float(np.linalg.norm(plug_pos - mouth_world(block_pos, R_block)))
        near = _le(near_dist, NEAR_DIST)
        still = (
            _lt(plug_lin, STILL_LIN)
            and _lt(plug_ang, STILL_ANG)
            and _lt(block_lin, STILL_LIN)
            and _lt(block_ang, STILL_ANG)
        )

        # latches
        if _ge(height_gain, LIFT_HEIGHT):
            self._lifted_ever = True
        if self._fail_reason is None:
            if _gt(block_moved, BLOCK_MOVED_FAIL):
                self._fail_reason = "block_moved"
            elif _gt(block_tilt, BLOCK_TILT_FAIL_DEG):
                self._fail_reason = "block_tilted"
            elif _gt(-height_gain, PLUG_FALL):
                self._fail_reason = "plug_fell"

        # tiers
        seated = (
            _ge(depth, SEATED_DEPTH)
            and _le(ang, SEATED_ANG_DEG)
            and _le(lateral, SEATED_LATERAL)
            and _gt(detent, DETENT_MIN)
        )
        engaged = _ge(depth, ENGAGED_DEPTH) and _le(ang, ENGAGED_ANG_DEG) and _le(lateral, ENGAGED_LATERAL)
        block_settled = _le(block_moved, BLOCK_MOVED_OK)

        if self._fail_reason is not None:
            score, tier = SCORE_NONE, TIER_FAILED
        elif seated and block_settled and grippers_open and still:
            score, tier = SCORE_SUCCESS, TIER_SUCCESS
        elif seated and block_settled:
            score, tier = SCORE_SEATED_HELD, TIER_SEATED_HELD
        elif seated:
            score, tier = SCORE_ENGAGED, TIER_SEATED_BLOCK_MOVED
        elif engaged:
            score, tier = SCORE_ENGAGED, TIER_ENGAGED
        elif self._lifted_ever and near:
            score, tier = SCORE_LIFTED_NEAR, TIER_LIFTED_NEAR
        else:
            score, tier = SCORE_NONE, TIER_NONE

        details = {
            "depth": depth,
            "ang": ang,
            "lateral": lateral,
            "p_local": [float(v) for v in p_local],
            "detent": detent,
            "block_moved": block_moved,
            "block_tilt": block_tilt,
            "plug_height_gain": height_gain,
            "plug_lin_speed": plug_lin,
            "plug_ang_speed": plug_ang,
            "block_lin_speed": block_lin,
            "block_ang_speed": block_ang,
            "grippers_open": grippers_open,
            "still": bool(still),
            "lifted_ever": bool(self._lifted_ever),
            "near": bool(near),
            "near_dist": near_dist,
            "seated": bool(seated),
            "engaged": bool(engaged),
            "failed": self._fail_reason is not None,
            "fail_reason": self._fail_reason,
            "tier": tier,
        }
        return StepResult(score=score, success=(tier == TIER_SUCCESS), details=details)
