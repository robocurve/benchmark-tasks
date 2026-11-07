"""State machine and stack verification logic for stack-crates-largest-first. Pure Python.

Tracks:
- Pallet-centered positions of all three crates.
- Stacking order rule: Large on Pallet, Medium on Large, Small on Medium.
- A crate cannot be supported by a smaller crate (invalid order).
- Tilt tolerance (< 5 degrees) via quaternion rotation matrix.
- Stillness (< 5 mm/s) and arm clearance (> 60 mm from stack).
- Zero-gate: any crate falling off the table (z < TOP - 0.05 m) latches floor fault (score 0.0).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping
import numpy as np

import geometry as g

COS_5_DEG = math.cos(math.radians(g.MAX_TILT_DEG))  # ~0.99619


def _quat_to_rot_z(quat_xyzw) -> float:
    """Return R[2, 2] from quaternion [x, y, z, w]."""
    x, y, z, w = quat_xyzw
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm > 0:
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return 1.0 - 2.0 * (x * x + y * y)


def _tilt_deg(quat_xyzw) -> float:
    r22 = max(-1.0, min(1.0, _quat_to_rot_z(quat_xyzw)))
    return math.degrees(math.acos(r22))


@dataclass
class StackState:
    large_on_pallet: bool = False
    medium_on_large: bool = False
    small_on_medium: bool = False
    all_offsets_valid: bool = False
    all_tilts_valid: bool = False
    all_crates_still: bool = False
    arms_clear: bool = False
    floor_fault: bool = False
    solved: bool = False
    details: dict = None

    def __post_init__(self):
        if self.details is None:
            self.details = {}


class StackLogic:
    def __init__(self) -> None:
        self.state = StackState()

    def step(
        self,
        dt: float,
        large_pos: tuple[float, float, float] | np.ndarray,
        large_quat: tuple[float, float, float, float] | np.ndarray,
        large_vel: tuple[float, float, float] | np.ndarray,
        medium_pos: tuple[float, float, float] | np.ndarray,
        medium_quat: tuple[float, float, float, float] | np.ndarray,
        medium_vel: tuple[float, float, float] | np.ndarray,
        small_pos: tuple[float, float, float] | np.ndarray,
        small_quat: tuple[float, float, float, float] | np.ndarray,
        small_vel: tuple[float, float, float] | np.ndarray,
        arm_tcps: Mapping[str, tuple[float, float, float] | np.ndarray] | None = None,
    ) -> StackState:
        st = self.state

        # 1. Floor fault zero-gate: crate fell off table
        if (
            large_pos[2] < g.TOP - 0.050
            or medium_pos[2] < g.TOP - 0.050
            or small_pos[2] < g.TOP - 0.050
        ):
            st.floor_fault = True
            st.solved = False
            return st

        # 2. Check tilt of each crate (< 5 deg)
        tilt_large = _tilt_deg(large_quat)
        tilt_medium = _tilt_deg(medium_quat)
        tilt_small = _tilt_deg(small_quat)
        st.all_tilts_valid = (
            tilt_large < g.MAX_TILT_DEG
            and tilt_medium < g.MAX_TILT_DEG
            and tilt_small < g.MAX_TILT_DEG
        )

        # 3. Check velocity of each crate (< 5 mm/s)
        spd_large = float(np.linalg.norm(large_vel[:3]))
        spd_medium = float(np.linalg.norm(medium_vel[:3]))
        spd_small = float(np.linalg.norm(small_vel[:3]))
        st.all_crates_still = (
            spd_large < g.STILL_LIN_VEL
            and spd_medium < g.STILL_LIN_VEL
            and spd_small < g.STILL_LIN_VEL
        )

        # 4. Offsets from target pallet centre (0.340, 0.000)
        d_large_pallet_xy = math.hypot(large_pos[0] - g.PALLET_POS[0], large_pos[1] - g.PALLET_POS[1])
        d_med_pallet_xy = math.hypot(medium_pos[0] - g.PALLET_POS[0], medium_pos[1] - g.PALLET_POS[1])
        d_small_pallet_xy = math.hypot(small_pos[0] - g.PALLET_POS[0], small_pos[1] - g.PALLET_POS[1])

        # Relative offsets
        d_med_large_xy = math.hypot(medium_pos[0] - large_pos[0], medium_pos[1] - large_pos[1])
        d_small_med_xy = math.hypot(small_pos[0] - medium_pos[0], small_pos[1] - medium_pos[1])

        # Rule check: A crate cannot be supported by a smaller crate
        # Large cannot be on top of medium or small
        large_on_smaller = (
            (large_pos[2] > medium_pos[2] + 0.020 and d_med_large_xy <= 0.050)
            or (large_pos[2] > small_pos[2] + 0.020 and math.hypot(large_pos[0] - small_pos[0], large_pos[1] - small_pos[1]) <= 0.040)
        )
        # Medium cannot be on top of small
        med_on_smaller = (
            medium_pos[2] > small_pos[2] + 0.020 and d_small_med_xy <= 0.040
        )

        # LEVEL 1: Large crate on pallet
        # Centre within 15 mm of pallet mark, resting near level 1 height
        if (
            d_large_pallet_xy <= g.LEVEL_1_MAX_OFFSET
            and abs(large_pos[2] - g.LEVEL_1_TARGET_Z) <= 0.012
            and not large_on_smaller
        ):
            st.large_on_pallet = True
        else:
            st.large_on_pallet = False

        # LEVEL 2: Medium crate on Large
        # Large must be on pallet; medium must sit above large by ~40 mm, offset <= 12 mm
        if (
            st.large_on_pallet
            and (g.MIN_SUPPORT_OVERLAP_DZ <= (medium_pos[2] - large_pos[2]) <= 0.055)
            and d_med_pallet_xy <= g.LEVEL_2_MAX_OFFSET
            and not med_on_smaller
        ):
            st.medium_on_large = True
        else:
            st.medium_on_large = False

        # LEVEL 3: Small crate on Medium
        # Medium must be on large; small must sit above medium by ~40 mm, offset <= 12 mm
        if (
            st.medium_on_large
            and (g.MIN_SUPPORT_OVERLAP_DZ <= (small_pos[2] - medium_pos[2]) <= 0.055)
            and d_small_pallet_xy <= g.LEVEL_3_MAX_OFFSET
        ):
            st.small_on_medium = True
        else:
            st.small_on_medium = False

        # Check arm clearances (TCP > 60 mm from stack centre)
        if arm_tcps:
            clear_l = math.hypot(arm_tcps["left"][0] - g.PALLET_POS[0], arm_tcps["left"][1] - g.PALLET_POS[1]) > g.ARMS_CLEAR_DIST
            clear_r = math.hypot(arm_tcps["right"][0] - g.PALLET_POS[0], arm_tcps["right"][1] - g.PALLET_POS[1]) > g.ARMS_CLEAR_DIST
            st.arms_clear = clear_l and clear_r
        else:
            st.arms_clear = True

        # For full 1.00 score: all three offsets <= 12 mm from pallet centre
        st.all_offsets_valid = (
            d_large_pallet_xy <= g.FINAL_ALL_MAX_OFFSET
            and d_med_pallet_xy <= g.FINAL_ALL_MAX_OFFSET
            and d_small_pallet_xy <= g.FINAL_ALL_MAX_OFFSET
        )

        st.solved = (
            st.large_on_pallet
            and st.medium_on_large
            and st.small_on_medium
            and st.all_offsets_valid
            and st.all_tilts_valid
            and st.all_crates_still
            and st.arms_clear
            and not st.floor_fault
        )

        st.details = {
            "large_on_pallet": st.large_on_pallet,
            "medium_on_large": st.medium_on_large,
            "small_on_medium": st.small_on_medium,
            "d_large_xy": d_large_pallet_xy,
            "d_med_xy": d_med_pallet_xy,
            "d_small_xy": d_small_pallet_xy,
            "tilt_large": tilt_large,
            "tilt_medium": tilt_medium,
            "tilt_small": tilt_small,
            "arms_clear": st.arms_clear,
            "floor_fault": st.floor_fault,
            "dbg_z": [round(float(large_pos[2]), 4), round(float(medium_pos[2]), 4), round(float(small_pos[2]), 4)],
            "dbg_speed": [round(spd_large, 4), round(spd_medium, 4), round(spd_small, 4)],
        }
        return st

