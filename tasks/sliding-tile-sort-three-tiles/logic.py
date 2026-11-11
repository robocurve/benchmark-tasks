"""Logic and evaluation tracking for sliding-tile-sort-three-tiles."""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np

import geometry as g


@dataclass
class EvalResult:
    score: float
    success: bool
    details: dict = field(default_factory=dict)


def _cell_index(pos: np.ndarray) -> int | None:
    for idx, c in enumerate(g.CELLS):
        if abs(pos[0] - c[0]) <= g.CELL_MATCH_TOL_XY and abs(pos[1] - c[1]) <= g.CELL_MATCH_TOL_XY:
            return idx
    return None


def _tilt_deg(quat_xyzw: np.ndarray) -> float:
    if quat_xyzw is None:
        return 0.0
    qx, qy, qz, qw = quat_xyzw
    r22 = 1.0 - 2.0 * (qx * qx + qy * qy)
    r22 = max(-1.0, min(1.0, r22))
    return float(np.degrees(np.arccos(r22)))


class TaskLogic:
    def __init__(self, hold_s: float = g.HOLD_TIME_S, hz: float = 20.0):
        self.hold_steps_req = int(round(hold_s * hz))
        self.reset()

    def reset(self) -> None:
        self.ever_escaped = False
        self.escape_info = None
        self.hold_counter = 0

    def update(
        self,
        blue_pos: np.ndarray,
        blue_quat: np.ndarray,
        blue_vel: np.ndarray,
        orange_pos: np.ndarray,
        orange_quat: np.ndarray,
        orange_vel: np.ndarray,
        yellow_pos: np.ndarray,
        yellow_quat: np.ndarray,
        yellow_vel: np.ndarray,
        arm_tcps: dict | None = None,
    ) -> EvalResult:
        # Check escape / lift zero-gate
        for label, p in (("blue", blue_pos), ("orange", orange_pos), ("yellow", yellow_pos)):
            why = None
            if p[2] > g.TILE_MAX_Z:
                why = "z"
            if abs(p[0] - g.FRAME_POS[0]) > g.FRAME_MAX_XY[2] or abs(p[1] - g.FRAME_POS[1]) > g.FRAME_MAX_XY[2]:
                why = "xy"
            if why:
                if not self.ever_escaped:
                    self.escape_info = {"tile": label, "why": why, "pos": [float(p[0]), float(p[1]), float(p[2])]}
                self.ever_escaped = True

        cell_b = _cell_index(blue_pos)
        cell_o = _cell_index(orange_pos)
        cell_y = _cell_index(yellow_pos)

        b_in_goal = (cell_b == g.GOAL_CELL_BLUE)
        o_in_goal = (cell_o == g.GOAL_CELL_ORANGE)
        y_in_goal = (cell_y == g.GOAL_CELL_YELLOW)

        num_in_goal = int(b_in_goal) + int(o_in_goal) + int(y_in_goal)

        if num_in_goal == 1:
            base_score = 0.25
        elif num_in_goal == 2:
            base_score = 0.50
        elif num_in_goal == 3:
            base_score = 0.75
        else:
            base_score = 0.0

        if self.ever_escaped:
            base_score = 0.0

        # Tilts and velocities
        tilts_ok = (
            _tilt_deg(blue_quat) < 5.0 and
            _tilt_deg(orange_quat) < 5.0 and
            _tilt_deg(yellow_quat) < 5.0
        )
        vels = [
            float(np.linalg.norm(v)) if v is not None else 0.0
            for v in (blue_vel, orange_vel, yellow_vel)
        ]
        resting = max(vels) < 0.05

        # Arm clearance
        arms_clear = True
        if arm_tcps:
            for tcp in arm_tcps.values():
                if tcp[2] < 0.82 and abs(tcp[0] - g.FRAME_POS[0]) < 0.12 and abs(tcp[1] - g.FRAME_POS[1]) < 0.12:
                    arms_clear = False
                    break

        all_ready_for_full = (
            num_in_goal == 3 and
            not self.ever_escaped and
            tilts_ok and
            resting and
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
            "cell_b": cell_b,
            "cell_o": cell_o,
            "cell_y": cell_y,
            "num_in_goal": num_in_goal,
            "tilts_ok": tilts_ok,
            "resting": resting,
            "arms_clear": arms_clear,
            "escaped": self.ever_escaped,
            "dbg_escape": self.escape_info,
            "dbg_blue": [float(x) for x in blue_pos],
            "dbg_orange": [float(x) for x in orange_pos],
            "dbg_yellow": [float(x) for x in yellow_pos],
            "hold_progress": self.hold_counter / float(self.hold_steps_req),
        }

        return EvalResult(score=score, success=success, details=details)

