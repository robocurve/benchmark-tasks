"""State machine and verification logic for spring-drawer-hold-and-retrieve. Pure Python.

Tracks:
- Drawer displacement d (0-140 mm along -X).
- Open status (d >= 90 mm) held for >= 1.0 s.
- Block elevation (lifted >= 30 mm clear of drawer floor).
- Block resting on green shelf.
- Drawer closed (< 5 mm) at end.
- Pinching zero-gate: if drawer is closing (< 20 mm out) while block is within 20 mm of drawer front lip (X).
- Dropped block zero-gate.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
import numpy as np

import geometry as g


@dataclass
class DrawerState:
    drawer_opened_1s: bool = False
    open_time: float = 0.0
    block_lifted_30mm: bool = False
    block_on_shelf: bool = False
    drawer_closed: bool = False
    cyan_lamp: bool = False
    pinch: bool = False
    block_dropped: bool = False
    solved: bool = False


class DrawerLogic:
    def __init__(self) -> None:
        self.state = DrawerState()

    def step(
        self,
        dt: float,
        drawer_disp: float,  # displacement in metres (>= 0 when pulled out)
        block_pos: tuple[float, float, float] | np.ndarray,
        arm_tcps: dict[str, tuple[float, float, float] | np.ndarray] | None = None,
    ) -> DrawerState:
        st = self.state

        if block_pos[2] < g.TOP - 0.05:
            st.block_dropped = True

        # Check cyan lamp and open duration
        if drawer_disp >= g.DRAWER_OPEN_THRESHOLD:
            st.cyan_lamp = True
            st.open_time += dt
            if st.open_time >= 1.0:
                st.drawer_opened_1s = True
        else:
            st.cyan_lamp = False

        # Check block lifted >= 30 mm clear of drawer
        # Drawer floor is at TOP + 0.010 = 0.7575 m. Normal block rest z is 0.7775 m.
        # Lifted >= 30 mm above rest z means z >= 0.8075 m.
        if block_pos[2] >= g.BLOCK_REST_Z + 0.030:
            st.block_lifted_30mm = True

        # Check pinch condition:
        # If drawer is closing (< 20 mm out) while block is within 20 mm of drawer lip (x) inside drawer channel
        # (the block is still in the cabinet mouth zone, i.e. within 60 mm behind the cabinet front face, when the drawer
        # front wall comes within 30 mm of closed: the block is caught between the closing drawer and the cabinet).
        if drawer_disp < g.PINCH_TRAVEL_THRESHOLD:
            if (block_pos[0] <= g.CABINET_FRONT_X + g.PINCH_ZONE_DEPTH and abs(block_pos[1]) <= g.DRAWER_SIZE[1] / 2
                    and block_pos[2] <= g.CABINET_TOP_Z):
                st.pinch = True

        # Check block on green shelf
        dx_sh = abs(block_pos[0] - g.SHELF_POS[0])
        dy_sh = abs(block_pos[1] - g.SHELF_POS[1])
        dz_sh = abs(block_pos[2] - g.SHELF_SPOT[2])
        if dx_sh <= g.SHELF_SIZE[0] / 2 and dy_sh <= g.SHELF_SIZE[1] / 2 and dz_sh <= 0.020:
            st.block_on_shelf = True

        # Check drawer closed (< 5 mm)
        st.drawer_closed = (drawer_disp <= g.DRAWER_CLOSED_THRESHOLD)

        # Check arms clear of cabinet
        arms_clear = True
        if arm_tcps is not None:
            for arm, tcp in arm_tcps.items():
                if tcp[0] >= g.CABINET_FRONT_X - 0.060 and abs(tcp[1]) <= g.CABINET_SIZE[1] / 2 + 0.050:
                    arms_clear = False
                    break

        st.solved = (
            st.drawer_opened_1s
            and st.block_lifted_30mm
            and st.block_on_shelf
            and st.drawer_closed
            and arms_clear
            and not st.pinch
            and not st.block_dropped
        )
        return st
