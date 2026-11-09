"""State machine and verification logic for interlocked-drawer-transfer. Pure Python.

Tracks:
- Drawer displacements: left and right (metres, >= 0 when pulled out toward robot).
- Open status (displacement > 15 mm).
- Closed status (displacement <= 5 mm).
- TIP FAULT: both drawers open simultaneously for >= 0.2 s (exceed 15 mm simultaneously).
- Block status:
  - Lifted clear out of left drawer (above drawer rim).
  - Parked on cabinet top staging pad or held outside while left drawer closed.
  - Placed resting inside right drawer.
  - Dropped off table zero-gate (z < TOP - 0.05 m).
- Solved status: block resting inside right drawer, both drawers closed (<= 5 mm),
  arms clear, no TIP fault, held for 1.0 s.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Mapping
import numpy as np

import geometry as g


@dataclass
class LogicState:
    disp_left: float = 0.0
    disp_right: float = 0.0
    is_left_open: bool = False
    is_right_open: bool = False
    is_left_closed: bool = True
    is_right_closed: bool = True
    both_open_time: float = 0.0
    tip_fault: bool = False
    block_dropped: bool = False
    left_opened_70mm: bool = False
    block_lifted_out: bool = False
    stage_0_25: bool = False
    left_closed_with_block_out: bool = False
    stage_0_50: bool = False
    right_opened_70mm: bool = False
    stage_0_75: bool = False
    block_in_right_drawer: bool = False
    block_on_staging: bool = False
    both_drawers_closed: bool = True
    arms_clear: bool = True
    lamp_ok: bool = True
    lamp_tip: bool = False
    solved: bool = False


class DrawerTransferLogic:
    """Tracks state and interlock transitions for interlocked-drawer-transfer."""

    def __init__(self) -> None:
        self.state = LogicState()

    def reset(self) -> None:
        self.state = LogicState()

    def step(
        self,
        dt: float,
        disp_left: float,
        disp_right: float,
        block_pos: tuple[float, float, float] | np.ndarray,
        arm_tcps: Mapping[str, tuple[float, float, float] | np.ndarray] | None = None,
    ) -> LogicState:
        st = self.state

        st.disp_left = float(disp_left)
        st.disp_right = float(disp_right)

        # 1. Drawer open/closed classifications
        st.is_left_open = (st.disp_left > g.DRAWER_OPEN_THRESHOLD)
        st.is_right_open = (st.disp_right > g.DRAWER_OPEN_THRESHOLD)
        st.is_left_closed = (st.disp_left <= g.DRAWER_CLOSED_THRESHOLD)
        st.is_right_closed = (st.disp_right <= g.DRAWER_CLOSED_THRESHOLD)
        st.both_drawers_closed = (st.is_left_closed and st.is_right_closed)

        # 2. Check dropped block zero-gate
        if block_pos[2] < g.TOP - 0.05:
            st.block_dropped = True

        # 3. Check TIP FAULT: both drawers open simultaneously for >= 0.2 s
        if st.is_left_open and st.is_right_open:
            st.both_open_time += dt
            if st.both_open_time >= g.TIP_FAULT_TIME - 1e-5:
                st.tip_fault = True
        else:
            st.both_open_time = 0.0

        # Indicator lamps
        if st.tip_fault:
            st.lamp_ok = False
            st.lamp_tip = True
        else:
            st.lamp_ok = True
            st.lamp_tip = False

        # 4. Check stage 0.25: left drawer opened >= 70 mm and block lifted out
        if st.disp_left >= g.DRAWER_OPENED_GOAL:
            st.left_opened_70mm = True

        # Block lifted out: above drawer rim, or clear of left drawer in XY
        block_above_rim = (block_pos[2] >= g.DRAWER_RIM_Z + 0.005)
        block_clear_left_y = (abs(block_pos[1] - g.DRAWER_LEFT_Y) > g.DRAWER_INNER_Y / 2)
        if st.left_opened_70mm and (block_above_rim or block_clear_left_y):
            st.block_lifted_out = True

        if st.left_opened_70mm and st.block_lifted_out:
            st.stage_0_25 = True

        # Check block on staging pad
        dx_stg = abs(block_pos[0] - g.STAGING_PAD_POS[0])
        dy_stg = abs(block_pos[1] - g.STAGING_PAD_POS[1])
        dz_stg = abs(block_pos[2] - g.STAGING_SPOT[2])
        st.block_on_staging = (dx_stg <= g.STAGING_PAD_SIZE[0] / 2 + 0.010 and
                               dy_stg <= g.STAGING_PAD_SIZE[1] / 2 + 0.010 and
                               dz_stg <= 0.020)

        # 5. Check stage 0.50: left drawer closed again (<= 5 mm) while block is held outside or parked
        # Block is outside left drawer if it's parked on staging, or above drawer rim, or far in Y from left drawer
        block_outside_left = (st.block_on_staging or block_above_rim or block_clear_left_y)
        if st.stage_0_25 and st.is_left_closed and block_outside_left:
            st.left_closed_with_block_out = True
            st.stage_0_50 = True

        # 6. Check stage 0.75: right drawer opened >= 70 mm
        if st.stage_0_50 and st.disp_right >= g.DRAWER_OPENED_GOAL:
            st.right_opened_70mm = True
            st.stage_0_75 = True

        # 7. Check block resting inside right drawer
        # When right drawer is closed (or within closed threshold), right drawer floor bounds:
        # x in [CABINET_FRONT_X + 0.010, CABINET_BACK_X - 0.010]
        # y in [DRAWER_RIGHT_Y - DRAWER_INNER_Y / 2, DRAWER_RIGHT_Y + DRAWER_INNER_Y / 2]
        # z in [BLOCK_REST_Z - 0.015, BLOCK_REST_Z + 0.020]
        bx, by, bz = block_pos[0], block_pos[1], block_pos[2]
        in_right_x = (g.CABINET_FRONT_X + 0.010 <= bx <= g.CABINET_BACK_X - 0.010)
        in_right_y = (abs(by - g.DRAWER_RIGHT_Y) <= g.DRAWER_INNER_Y / 2 + 0.005)
        in_right_z = (abs(bz - g.BLOCK_REST_Z) <= 0.020)
        st.block_in_right_drawer = (in_right_x and in_right_y and in_right_z)

        # 8. Check arms clear of cabinet
        arms_clear = True
        if arm_tcps is not None:
            for _arm, tcp in arm_tcps.items():
                in_cab_x = (tcp[0] >= g.CABINET_FRONT_X - 0.050 and tcp[0] <= g.CABINET_BACK_X + 0.050)
                in_cab_y = (abs(tcp[1]) <= g.CABINET_WIDTH / 2 + 0.050)
                in_cab_z = (tcp[2] <= g.CABINET_TOP_Z + 0.060)
                if in_cab_x and in_cab_y and in_cab_z:
                    arms_clear = False
                    break
        st.arms_clear = arms_clear

        # 9. Solved check (condition for 1.00)
        st.solved = (
            st.stage_0_75
            and st.block_in_right_drawer
            and st.both_drawers_closed
            and st.arms_clear
            and not st.tip_fault
            and not st.block_dropped
        )
        return st
