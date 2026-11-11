"""Task logic and state machine for hook-block-from-guarded-zone. Pure Python.

Tracks:
1. Boundary safety fault: if either arm TCP (or gripper body) x > 0.40 + 0.01 m, permanently latch FAULT (score 0.0).
2. Rod held: hooked rod lifted or grasped by arm -> 0.25.
3. Block moved >= 60 mm (x <= 0.52) -> 0.50.
4. Block outside guarded zone (x < 0.38 resting) -> 0.75.
5. Block in tray, rod set down outside zone, arms clear, held 2 s, no fault -> 1.00.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Mapping, Sequence

import geometry as g


@dataclass
class HookBlockState:
    fault: bool = False
    fault_reason: str = ""
    rod_held: bool = False
    block_moved_60mm: bool = False
    block_outside_zone: bool = False
    block_in_tray: bool = False
    block_resting_in_tray: bool = False
    rod_outside_zone: bool = False
    arms_clear: bool = False
    block_outside_rest_time: float = 0.0
    tray_hold_time: float = 0.0
    solved: bool = False
    score: float = 0.0

    def as_dict(self) -> dict[str, Any]:
        return {
            "fault": self.fault,
            "fault_reason": self.fault_reason,
            "rod_held": self.rod_held,
            "block_moved_60mm": self.block_moved_60mm,
            "block_outside_zone": self.block_outside_zone,
            "block_in_tray": self.block_in_tray,
            "rod_outside_zone": self.rod_outside_zone,
            "arms_clear": self.arms_clear,
            "tray_hold_time": round(self.tray_hold_time, 3),
            "solved": self.solved,
            "score": round(self.score, 4),
        }


class HookBlockLogic:
    def __init__(self, hold_s: float = g.HOLD_TIME_S, rest_s: float = g.REST_TIME_S) -> None:
        self.hold_s = hold_s
        self.rest_s = rest_s
        self.state = HookBlockState()

    def reset(self) -> None:
        self.state = HookBlockState()

    def step(
        self,
        dt: float,
        block_pos: Sequence[float],
        rod_pos: Sequence[float],
        block_vel: Sequence[float] | None = None,
        rod_vel: Sequence[float] | None = None,
        arm_tcps: Mapping[str, Sequence[float]] | None = None,
        gripper_poses: Mapping[str, Any] | None = None,
    ) -> HookBlockState:
        st = self.state

        # 1. Boundary check: if either arm TCP (or gripper body) x > 0.40 + 0.01 m, latch FAULT (score 0.0)
        if not st.fault:
            if arm_tcps is not None:
                for arm_name, tcp in arm_tcps.items():
                    if g.is_boundary_violation(tcp):
                        st.fault = True
                        st.fault_reason = f"tcp_boundary_violation_{arm_name}"
                        break

            if not st.fault and gripper_poses is not None:
                for arm_name, pose in gripper_poses.items():
                    if hasattr(pose, "shape") and pose.shape == (4, 4):
                        px = float(pose[0, 3])
                    elif isinstance(pose, (tuple, list)) and len(pose) >= 1:
                        px = float(pose[0])
                    else:
                        continue
                    if px > g.FAULT_BOUNDARY_X:
                        st.fault = True
                        st.fault_reason = f"gripper_boundary_violation_{arm_name}"
                        break

        # If fault latched, score is permanently 0.0
        if st.fault:
            st.score = 0.00
            st.solved = False
            return st

        b_pos = (float(block_pos[0]), float(block_pos[1]), float(block_pos[2]))
        r_pos = (float(rod_pos[0]), float(rod_pos[1]), float(rod_pos[2]))

        v_b = math.sqrt(sum(float(v) ** 2 for v in block_vel[:3])) if block_vel is not None else 0.0
        v_r = math.sqrt(sum(float(v) ** 2 for v in rod_vel[:3])) if rod_vel is not None else 0.0

        # 2. Rod held check: rod lifted off table or moved from initial pos while TCP near
        if not st.rod_held:
            rod_lifted = (r_pos[2] > g.TOP + g.ROD_THICKNESS / 2 + 0.010)
            rod_moved = (math.hypot(r_pos[0] - g.ROD_INIT_POS[0], r_pos[1] - g.ROD_INIT_POS[1]) > 0.020)
            tcp_near = True
            if arm_tcps is not None:
                tcp_near = any(
                    math.hypot(tcp[0] - r_pos[0], tcp[1] - r_pos[1]) <= 0.220
                    for tcp in arm_tcps.values()
                )
            if (rod_lifted or rod_moved) and tcp_near:
                st.rod_held = True

        # 3. Block moved >= 60 mm check (x <= 0.520)
        if not st.block_moved_60mm:
            if b_pos[0] <= g.BLOCK_MOVED_X + 1e-4:
                st.block_moved_60mm = True

        # 4. Block outside zone check (x < 0.380 resting)
        if b_pos[0] < g.BLOCK_OUTSIDE_X:
            if v_b <= g.STILL_LIN_VEL:
                st.block_outside_rest_time += dt
                if st.block_outside_rest_time >= self.rest_s - 1e-4:
                    st.block_outside_zone = True
            else:
                st.block_outside_rest_time = 0.0
        else:
            st.block_outside_rest_time = 0.0

        # 5. Block in tray check
        st.block_in_tray = g.in_tray(b_pos)
        st.block_resting_in_tray = st.block_in_tray and (v_b <= g.STILL_LIN_VEL)

        # 6. Rod set down outside zone check
        rod_on_table = (r_pos[2] <= g.TOP + g.ROD_THICKNESS + 0.010)
        rod_outside = g.is_rod_outside(r_pos)
        rod_still = (v_r <= g.STILL_LIN_VEL * 2)

        # Ensure rod is not being held
        rod_not_grasped = True
        if arm_tcps is not None:
            for tcp in arm_tcps.values():
                d_tcp_rod = math.hypot(tcp[0] - r_pos[0], tcp[1] - r_pos[1])
                if d_tcp_rod < 0.060 and tcp[2] < g.TOP + 0.060:
                    rod_not_grasped = False
                    break

        st.rod_outside_zone = rod_on_table and rod_outside and rod_still and rod_not_grasped

        # 7. Arms clear check
        arms_clear = True
        if arm_tcps is not None:
            for tcp in arm_tcps.values():
                d_tray = math.hypot(tcp[0] - g.TRAY_POS[0], tcp[1] - g.TRAY_POS[1])
                is_clear = (d_tray >= g.ARMS_CLEAR_DIST) or (tcp[2] >= g.HOVER_Z - 0.020)
                if not is_clear:
                    arms_clear = False
                    break
        st.arms_clear = arms_clear

        # 8. Hold stable for 2.0 s (40 steps at 20 Hz) for 1.00
        if (
            st.block_resting_in_tray
            and st.rod_outside_zone
            and st.arms_clear
            and not st.fault
        ):
            st.tray_hold_time += dt
            if st.tray_hold_time >= self.hold_s - 1e-4:
                st.solved = True
        else:
            st.tray_hold_time = 0.0

        # 9. Score calculation
        if st.fault:
            st.score = 0.00
        elif st.solved:
            st.score = 1.00
        elif st.block_outside_zone:
            st.score = 0.75
        elif st.block_moved_60mm:
            st.score = 0.50
        elif st.rod_held:
            st.score = 0.25
        else:
            st.score = 0.00

        return st
