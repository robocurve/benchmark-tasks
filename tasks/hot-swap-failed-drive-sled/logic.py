"""State machine and verification logic for hot-swap-failed-drive-sled. Pure Python.

Tracks:
- Sled displacements from seated positions (X axis).
- Latch button compression.
- Detent zone (0-25 mm): forced pull fault if pulled > 3 mm while latch not pressed (disp > -6.4 mm).
- Healthy bay sleds disturbance (> 5 mm movement = fault).
- Discard pad status (old sled resting on the pad, current state, centre within 40 mm in x and y, z within 10 mm).
- Spare insertion status in failed bay (>= 95% = >= 142 mm of 150 mm).
- Keying check: blue spare turns lamp green; orange spare stays red (wrong keying).
- FAULT latch (zero-gate).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
import numpy as np

import geometry as g


@dataclass
class HotSwapState:
    pulled_60mm: bool = False
    old_sled_on_discard: bool = False
    spare_inserted_95pct: bool = False
    blue_spare_used: bool = False
    orange_spare_used: bool = False
    lamp_green: bool = False
    fault: bool = False
    fault_reason: str = ""
    solved: bool = False


class HotSwapLogic:
    def __init__(self) -> None:
        self.state = HotSwapState()
        self.failed_sled_max_pull = 0.0

    def step(
        self,
        dt: float,
        latch_disps: dict[str, float],   # negative when pressed, e.g. "bay_1": -0.007
        sled_positions: dict[str, tuple[float, float, float] | np.ndarray],
        spare_positions: dict[str, tuple[float, float, float] | np.ndarray],
        arm_tcps: dict[str, tuple[float, float, float] | np.ndarray] | None = None,
    ) -> HotSwapState:
        st = self.state
        if st.fault:
            return st

        # End-state scoring: these flags describe the current step, so undoing a goal lowers the score again.
        # Only faults (violations) and the "pulled out 60 mm with the latch held" event stay latched.
        st.old_sled_on_discard = False
        st.spare_inserted_95pct = False
        st.blue_spare_used = False
        st.orange_spare_used = False

        # 1. Check healthy sleds disturbance (> 5 mm movement from seated = FAULT)
        for bay in ("bay_0", "bay_2"):
            p = sled_positions.get(bay)
            if p is not None:
                dx = abs(p[0] - g.SLED_SEATED_X)
                dy = abs(p[1] - g.BAY_Y[bay])
                dz = abs(p[2] - g.SLED_REST_Z)
                if dx > 0.005 or dy > 0.005 or dz > 0.005:
                    st.fault = True
                    st.fault_reason = f"healthy_{bay}_moved"
                    return st

        # 2. Check failed sled pull and latch detent rule
        p_failed = sled_positions.get(g.FAILED_BAY)
        latch_failed_disp = latch_disps.get(g.FAILED_BAY, 0.0)
        latch_pressed = (latch_failed_disp <= g.LATCH_THRESHOLD)

        if p_failed is not None:
            # Sled pull distance (moving forward -X, so seated_x - p[0])
            pull_dist = g.SLED_SEATED_X - p_failed[0]
            self.failed_sled_max_pull = max(self.failed_sled_max_pull, pull_dist)

            # Detent zone check: between 3 mm and 25 mm, latch MUST be pressed
            if 0.003 < pull_dist <= 0.025:
                if not latch_pressed:
                    st.fault = True
                    st.fault_reason = "forced_pull_no_latch"
                    return st

            if self.failed_sled_max_pull >= 0.060:
                st.pulled_60mm = True

            # Check if old sled is completely resting on discard pad
            dx_disc = abs(p_failed[0] - g.DISCARD_SPOT[0])
            dy_disc = abs(p_failed[1] - g.DISCARD_SPOT[1])
            dz_disc = abs(p_failed[2] - g.DISCARD_SPOT[2])
            if dx_disc <= g.DISCARD_XY_TOL and dy_disc <= g.DISCARD_XY_TOL and dz_disc <= g.DISCARD_Z_TOL:
                st.old_sled_on_discard = True

        # 3. Check spare insertion in failed bay (Bay 1 at y = 0.000)
        # Sled body is 150 mm long. 95% seated means inserted >= 142 mm, so center is within 8 mm of SLED_SEATED_X.
        for color, p_spare in spare_positions.items():
            if p_spare is None:
                continue
            dy_bay = abs(p_spare[1] - g.BAY_Y[g.FAILED_BAY])
            dz_bay = abs(p_spare[2] - g.SLED_REST_Z)
            # Must be aligned in Y and Z with the bay channel
            if dy_bay <= 0.015 and dz_bay <= 0.015:
                # Sled center distance to seated position
                dist_to_seated = abs(p_spare[0] - g.SLED_SEATED_X)
                if dist_to_seated <= 0.008:  # >= 95% seated
                    st.spare_inserted_95pct = True
                    if color == "blue":
                        st.blue_spare_used = True
                    elif color == "orange":
                        st.orange_spare_used = True

        # 4. Status lamp logic
        # Turns green if blue spare seated >= 95% AND latch released
        if st.spare_inserted_95pct and st.blue_spare_used and not latch_pressed:
            st.lamp_green = True
        else:
            st.lamp_green = False

        # Check arms clear of rack area
        arms_clear = True
        if arm_tcps is not None:
            for arm, tcp in arm_tcps.items():
                if tcp[0] >= g.RACK_FRONT_X - 0.050 and abs(tcp[1]) <= 0.250:
                    arms_clear = False
                    break

        st.solved = (
            st.lamp_green
            and st.old_sled_on_discard
            and arms_clear
            and not st.fault
        )
        return st

