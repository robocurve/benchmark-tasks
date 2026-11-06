"""State machine and weighing logic for weigh-and-reject-heavy-block. Pure Python.

Tracks:
- Platform deflection and steady measurement (>= 1.0 s steady with no arm contact).
- Block weighed status.
- Final placements: heavy block in reject tray, light blocks on green shelf.
- Fault conditions: wrong rejection (light block in reject tray), dropped block.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
import numpy as np

import geometry as g


@dataclass
class WeighState:
    block_weighed: dict[str, bool] = field(default_factory=lambda: {b: False for b in g.BLOCK_NAMES})
    steady_weigh_time: dict[str, float] = field(default_factory=lambda: {b: 0.0 for b in g.BLOCK_NAMES})
    current_block_on_scale: str | None = None
    all_three_weighed: bool = False
    heavy_in_tray: bool = False
    both_lights_on_shelf: bool = False
    scale_empty: bool = True
    wrong_rejection: bool = False
    block_dropped: bool = False
    solved: bool = False


class WeighLogic:
    def __init__(self) -> None:
        self.state = WeighState()

    def step(
        self,
        dt: float,
        scale_disp: float,  # platform displacement (m, negative when depressed)
        block_positions: dict[str, tuple[float, float, float] | np.ndarray],
        arm_tcps: dict[str, tuple[float, float, float] | np.ndarray] | None = None,
    ) -> WeighState:
        st = self.state

        # Check dropped blocks
        for b, pos in block_positions.items():
            if pos[2] < g.TOP - 0.05:
                st.block_dropped = True

        # Check arm contact near scale
        arms_clear_of_scale = True
        if arm_tcps is not None:
            for arm, tcp in arm_tcps.items():
                d_scale = math.hypot(tcp[0] - g.SCALE_CENTER_XY[0], tcp[1] - g.SCALE_CENTER_XY[1])
                if d_scale < 0.070 and tcp[2] < g.SCALE_PLATFORM_TOP_REST_Z + 0.050:
                    arms_clear_of_scale = False
                    break

        # Check which block is on scale
        block_on_scale = None
        for b in g.BLOCK_NAMES:
            pos = block_positions.get(b)
            if pos is None:
                continue
            dx = abs(pos[0] - g.SCALE_CENTER_XY[0])
            dy = abs(pos[1] - g.SCALE_CENTER_XY[1])
            dz = abs(pos[2] - (g.SCALE_PLATFORM_TOP_REST_Z + scale_disp + g.BLOCK_SIZE[2] / 2))
            if dx <= 0.045 and dy <= 0.045 and dz <= 0.020:
                block_on_scale = b
                break

        st.current_block_on_scale = block_on_scale
        st.scale_empty = (block_on_scale is None)

        # Weighing physics check
        if block_on_scale is not None and arms_clear_of_scale:
            # Check expected sag
            is_heavy = (block_on_scale == "block_1")
            expected_sag = g.SAG_HEAVY_BLOCK if is_heavy else g.SAG_LIGHT_BLOCK
            measured_sag = abs(scale_disp)
            # Allow tolerance on platform friction / settling
            if abs(measured_sag - expected_sag) <= 0.003:  # within 3 mm of expected sag
                st.steady_weigh_time[block_on_scale] += dt
                if st.steady_weigh_time[block_on_scale] >= 1.0:
                    st.block_weighed[block_on_scale] = True

        st.all_three_weighed = all(st.block_weighed[b] for b in g.BLOCK_NAMES)

        # Check placements
        # 1. Heavy block in reject tray
        p_heavy = block_positions.get("block_1")
        if p_heavy is not None:
            tx = abs(p_heavy[0] - g.TRAY_POS[0])
            ty = abs(p_heavy[1] - g.TRAY_POS[1])
            st.heavy_in_tray = (tx <= g.TRAY_SIZE[0] / 2 and ty <= g.TRAY_SIZE[1] / 2 and p_heavy[2] <= g.TRAY_TOP_Z + 0.030)

        # 2. Light blocks in reject tray -> WRONG REJECTION (zero-gate / penalty)
        for b in ("block_0", "block_2"):
            p_light = block_positions.get(b)
            if p_light is not None:
                tx = abs(p_light[0] - g.TRAY_POS[0])
                ty = abs(p_light[1] - g.TRAY_POS[1])
                if tx <= g.TRAY_SIZE[0] / 2 and ty <= g.TRAY_SIZE[1] / 2 and p_light[2] <= g.TRAY_TOP_Z + 0.030:
                    if st.block_weighed[b]:
                        st.wrong_rejection = True

        # 3. Light blocks on green shelf
        lights_ok = True
        for b in ("block_0", "block_2"):
            p_light = block_positions.get(b)
            if p_light is None:
                lights_ok = False
                continue
            sx = abs(p_light[0] - g.SHELF_POS[0])
            sy = abs(p_light[1] - g.SHELF_POS[1])
            on_shelf = (sx <= g.SHELF_SIZE[0] / 2 and sy <= g.SHELF_SIZE[1] / 2 and abs(p_light[2] - (g.SHELF_TOP_Z + g.BLOCK_SIZE[2] / 2)) <= 0.020)
            if not on_shelf:
                lights_ok = False
        st.both_lights_on_shelf = lights_ok

        st.solved = (
            st.all_three_weighed
            and st.heavy_in_tray
            and st.block_weighed["block_1"]
            and st.both_lights_on_shelf
            and st.scale_empty
            and not st.wrong_rejection
            and not st.block_dropped
        )

        return st

