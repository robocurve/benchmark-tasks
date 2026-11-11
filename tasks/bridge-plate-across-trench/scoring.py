"""Scoring ladder and verification for bridge-plate-across-trench. Pure Python + NumPy.

Ladder:
  0.00: Initial / invalid state.
  0.25: Bridge plate seated across trench (both y-edges overlap platform lips by >= 20 mm, tilt < 3 deg, resting).
  0.50: Block on the bridge (centre within bridge footprint, crossing zone).
  0.75: Block on the far platform past the trench lip (y <= -0.045).
  1.00: Block centred on target mark within 15 mm and still, bridge still seated,
        arms clear (> 60 mm), held stable for 2.0 s (40 steps at 20 Hz).

Zero-gates:
  - Plate or block fell in trench (z < TOP + 0.030 - 0.020 m) latches fell = score 0.0 permanently.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping
import numpy as np

import geometry as g
import logic


@dataclass(frozen=True)
class ScoreResult:
    score: float
    success: bool
    details: dict


class Scorer:
    """Stateful scorer evaluating bridge-plate-across-trench ladder and zero-gates."""

    def __init__(self, hold_s: float = g.HOLD_TIME_S, hz: float = 20.0) -> None:
        self.dt = 1.0 / hz
        self.logic = logic.BridgeLogic(hold_s=hold_s)
        self.reset()

    def reset(self) -> None:
        self.logic.reset()
        self.best_score = 0.0

    def update(
        self,
        bridge_pos: tuple[float, float, float] | np.ndarray,
        bridge_quat: tuple[float, float, float, float] | np.ndarray = (0.0, 0.0, 0.0, 1.0),
        bridge_vel: tuple[float, float, float] | np.ndarray = (0.0, 0.0, 0.0),
        block_pos: tuple[float, float, float] | np.ndarray = (0.0, 0.0, 0.0),
        block_quat: tuple[float, float, float, float] | np.ndarray = (0.0, 0.0, 0.0, 1.0),
        block_vel: tuple[float, float, float] | np.ndarray = (0.0, 0.0, 0.0),
        arm_tcps: Mapping[str, tuple[float, float, float] | np.ndarray] | None = None,
    ) -> ScoreResult:
        st = self.logic.step(
            dt=self.dt,
            bridge_pos=bridge_pos,
            bridge_quat=bridge_quat,
            bridge_vel=bridge_vel,
            block_pos=block_pos,
            block_quat=block_quat,
            block_vel=block_vel,
            arm_tcps=arm_tcps,
        )

        dbg_details = {
            "dbg_fell": st.fell,
            "dbg_bridge_seated": st.bridge_seated,
            "dbg_block_on_bridge": st.block_on_bridge,
            "dbg_block_on_far": st.block_on_far,
            "dbg_block_centered": st.block_centered,
            "dbg_bridge_pos": [round(float(x), 4) for x in bridge_pos],
            "dbg_block_pos": [round(float(x), 4) for x in block_pos],
            "dbg_near_overlap_mm": round(float(st.near_overlap) * 1000, 1),
            "dbg_far_overlap_mm": round(float(st.far_overlap) * 1000, 1),
            "dbg_bridge_tilt_deg": round(float(st.bridge_tilt), 2),
            "dbg_target_dist_mm": round(float(st.target_dist) * 1000, 1),
            "dbg_hold_time": round(float(st.hold_time), 2),
            "dbg_ever_25": st.ever_25,
            "dbg_ever_50": st.ever_50,
            "dbg_ever_75": st.ever_75,
            "dbg_ever_100": st.ever_100,
        }

        if arm_tcps is not None:
            if "left" in arm_tcps:
                dbg_details["dbg_left_tcp"] = [round(float(x), 4) for x in arm_tcps["left"]]
            if "right" in arm_tcps:
                dbg_details["dbg_right_tcp"] = [round(float(x), 4) for x in arm_tcps["right"]]

        if st.fell:
            return ScoreResult(score=0.0, success=False, details=dbg_details)

        score = 0.0
        if st.ever_25:
            score = max(score, 0.25)
        if st.ever_50:
            score = max(score, 0.50)
        if st.ever_75:
            score = max(score, 0.75)
        if st.ever_100:
            score = 1.00

        self.best_score = max(self.best_score, score)
        success = st.solved and (self.best_score >= 1.00)

        dbg_details["best_score"] = self.best_score
        return ScoreResult(
            score=self.best_score,
            success=success,
            details=dbg_details,
        )
