"""Evaluation logic and zero-gate tracking for squeeze-in-the-last-slab."""

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


def _xy_dist(p1, p2) -> float:
    return float(np.linalg.norm(np.asarray(p1)[:2] - np.asarray(p2)[:2]))


class TaskLogic:
    def __init__(self, hold_s: float = g.HOLD_TIME_S, hz: float = 20.0):
        self.hold_steps_req = int(round(hold_s * hz))
        self.reset()

    def reset(self) -> None:
        self.ever_knocked = False
        self.hold_counter = 0

    def update(
        self,
        s1_pos: np.ndarray, s1_quat: np.ndarray,
        s2_pos: np.ndarray, s2_quat: np.ndarray,
        s3_pos: np.ndarray, s3_quat: np.ndarray,
        s4_pos: np.ndarray, s4_quat: np.ndarray,
        s5_pos: np.ndarray, s5_quat: np.ndarray,
        arm_tcps: dict | None = None,
    ) -> EvalResult:
        neighbours = [
            (s1_pos, s1_quat, g.SLAB_1_POS),
            (s2_pos, s2_quat, g.SLAB_2_POS),
            (s3_pos, s3_quat, g.SLAB_3_POS),
            (s4_pos, s4_quat, g.SLAB_4_POS),
        ]

        # Check neighbour knocked zero-gate (cap 0.50)
        for pos, quat, start in neighbours:
            tilt = _tilt_deg(quat)
            d_xy = _xy_dist(pos, start)
            if tilt > g.KNOCK_TILT_DEG or d_xy > g.KNOCK_DIST_XY:
                self.ever_knocked = True

        max_n_dist = max(_xy_dist(p, s) for p, _, s in neighbours)
        max_n_tilt = max(_tilt_deg(q) for _, q, _ in neighbours)

        # Check fifth slab (s5)
        s5_tilt = _tilt_deg(s5_quat)
        s5_z_ok = abs(s5_pos[2] - g.SLAB_STANDING_Z) <= 0.010
        on_strip = (abs(s5_pos[0] - g.STRIP_POS[0]) <= g.STRIP_SIZE[0] / 2.0 and
                    abs(s5_pos[1] - g.STRIP_POS[1]) <= g.STRIP_SIZE[1] / 2.0 and
                    s5_z_ok and s5_tilt <= 15.0)

        in_gap = (abs(s5_pos[0] - g.SLOT_TARGET_POS[0]) <= g.GOAL_XY_TOL and
                  abs(s5_pos[1] - g.SLOT_TARGET_POS[1]) <= 0.015 and
                  s5_z_ok)

        # Base score ladder
        if in_gap and s5_tilt < 6.0:
            base_score = 0.75
        elif in_gap and s5_tilt <= 15.0:
            base_score = 0.50
        elif on_strip:
            base_score = 0.25
        else:
            base_score = 0.0

        # Apply knocked zero-gate cap (max 0.50)
        if self.ever_knocked:
            base_score = min(base_score, 0.50)

        # Arm clearance
        arms_clear = True
        if arm_tcps:
            for tcp in arm_tcps.values():
                if tcp[2] < 0.84 and abs(tcp[1]) < 0.15 and 0.18 <= tcp[0] <= 0.42:
                    arms_clear = False
                    break

        all_ready_for_full = (
            in_gap and
            s5_tilt <= g.GOAL_TILT_DEG and
            max_n_dist <= g.NEIGHBOUR_STABLE_DIST and
            max_n_tilt <= g.NEIGHBOUR_STABLE_TILT and
            not self.ever_knocked and
            arms_clear
        )

        if all_ready_for_full:
            self.hold_counter += 1
            if self.hold_counter >= self.hold_steps_req:
                score = 1.00
                success = True
            else:
                score = base_score
                success = False
        else:
            self.hold_counter = 0
            score = base_score
            success = False

        details = {
            "s5_tilt": s5_tilt,
            "in_gap": in_gap,
            "max_n_dist": max_n_dist,
            "max_n_tilt": max_n_tilt,
            "knocked_cap": self.ever_knocked,
            "arms_clear": arms_clear,
            "hold_progress": self.hold_counter / float(self.hold_steps_req),
        }

        return EvalResult(score=score, success=success, details=details)

