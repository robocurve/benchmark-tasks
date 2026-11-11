"""Evaluation logic and zero-gate tracking for clear-lane-then-push-block-to-dock."""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np

import geometry as g


@dataclass
class EvalResult:
    score: float
    success: bool
    details: dict = field(default_factory=dict)


def _tilt_deg(quat_xyzw: np.ndarray) -> float:
    if quat_xyzw is None:
        return 0.0
    qx, qy, qz, qw = quat_xyzw
    r22 = 1.0 - 2.0 * (qx * qx + qy * qy)
    r22 = max(-1.0, min(1.0, r22))
    return float(np.degrees(np.arccos(r22)))


def _yaw_deg(quat_xyzw: np.ndarray) -> float:
    if quat_xyzw is None:
        return 0.0
    qx, qy, qz, qw = quat_xyzw
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return float(np.degrees(abs(np.arctan2(siny_cosp, cosy_cosp))))


def _xy_dist(p1, p2) -> float:
    return float(np.linalg.norm(np.asarray(p1)[:2] - np.asarray(p2)[:2]))


class TaskLogic:
    def __init__(self, hold_s: float = g.HOLD_TIME_S, hz: float = 20.0):
        self.hold_steps_req = int(round(hold_s * hz))
        self.reset()

    def reset(self) -> None:
        self.ever_post_hit = False
        self.hold_counter = 0

    def update(
        self,
        cargo_pos: np.ndarray,
        cargo_quat: np.ndarray,
        blue_pos: np.ndarray,
        yellow_pos: np.ndarray,
        post_pos: np.ndarray,
        post_quat: np.ndarray,
        arm_tcps: dict | None = None,
    ) -> EvalResult:
        # Check red post zero-gate: tilt > 10 deg or moved > 10 mm
        post_tilt = _tilt_deg(post_quat)
        post_shift = _xy_dist(post_pos, g.POST_START_POS)
        if post_tilt > g.POST_MAX_TILT_DEG or post_shift > g.POST_MAX_DIST:
            self.ever_post_hit = True

        # Loose blocks clearance
        blue_cleared = abs(float(blue_pos[1])) > g.LANE_CLEAR_Y
        yellow_cleared = abs(float(yellow_pos[1])) > g.LANE_CLEAR_Y
        both_cleared = blue_cleared and yellow_cleared

        # Loose block inside lane bounds
        def in_lane(p):
            return abs(float(p[1])) <= g.LANE_CLEAR_Y and (g.LANE_X_MIN - 0.02 <= float(p[0]) <= g.LANE_X_MAX + 0.02)

        lane_blocked = in_lane(blue_pos) or in_lane(yellow_pos)

        # Cargo metrics
        cargo_x = float(cargo_pos[0])
        cargo_y = float(cargo_pos[1])
        cargo_dx_mark = abs(cargo_x - g.MARK_POS[0])
        cargo_dy_mark = abs(cargo_y - g.MARK_POS[1])
        cargo_yaw = _yaw_deg(cargo_quat)

        # Raw score computation
        raw_score = 0.0

        if both_cleared:
            raw_score = 0.25

        if both_cleared and (cargo_x >= g.CARGO_STAGE2_X):
            raw_score = 0.50

        if both_cleared and (cargo_dx_mark <= g.DOCK_STAGE3_TOL):
            raw_score = 0.75

        # Check arms clear
        arms_clear = True
        if arm_tcps is not None:
            for tcp in arm_tcps.values():
                if tcp is not None:
                    d_cargo = float(np.linalg.norm(np.asarray(tcp)[:2] - np.asarray(cargo_pos)[:2]))
                    if d_cargo < 0.080:
                        arms_clear = False

        perfect_pose = (
            both_cleared
            and (cargo_dx_mark <= g.DOCK_STAGE4_XY_TOL)
            and (cargo_dy_mark <= g.DOCK_STAGE4_XY_TOL)
            and (cargo_yaw <= g.DOCK_STAGE4_YAW_DEG)
            and not self.ever_post_hit
            and arms_clear
        )

        if perfect_pose:
            self.hold_counter += 1
        else:
            self.hold_counter = 0

        if raw_score >= 0.75 and self.hold_counter >= self.hold_steps_req:
            raw_score = 1.00

        # Zero-gate cap: post hit caps score at 0.25
        final_score = raw_score
        if self.ever_post_hit:
            final_score = min(final_score, 0.25)

        success = (final_score >= 1.00)

        details = {
            "post_hit": self.ever_post_hit,
            "post_tilt_deg": post_tilt,
            "post_shift_m": post_shift,
            "blue_cleared": blue_cleared,
            "yellow_cleared": yellow_cleared,
            "both_cleared": both_cleared,
            "lane_blocked": lane_blocked,
            "cargo_x": cargo_x,
            "cargo_dx_mark": cargo_dx_mark,
            "cargo_dy_mark": cargo_dy_mark,
            "cargo_yaw_deg": cargo_yaw,
            "arms_clear": arms_clear,
            "hold_counter": self.hold_counter,
            "raw_score": raw_score,
            "final_score": final_score,
        }

        return EvalResult(score=final_score, success=success, details=details)

