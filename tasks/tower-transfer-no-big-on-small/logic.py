"""Pure Python stateful logic for tower-transfer-no-big-on-small."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
import numpy as np

import geometry as g


def quat_tilt_deg(quat: tuple[float, float, float, float] | np.ndarray) -> float:
    """Compute tilt from vertical Z-axis in degrees from quaternion (x, y, z, w)."""
    qx, qy, qz, qw = float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])
    # z_axis_z = 1 - 2*(qx*qx + qy*qy)
    z_z = 1.0 - 2.0 * (qx * qx + qy * qy)
    z_z = max(-1.0, min(1.0, z_z))
    return float(np.degrees(math.acos(z_z)))


def mark_of_pos(pos: tuple[float, float, float] | np.ndarray, tol: float = g.MARK_XY_TOL) -> str | None:
    """Identify which mark (LEFT, MIDDLE, RIGHT) a block is over, if within tol."""
    x, y = float(pos[0]), float(pos[1])
    for name, mark_pos in (("LEFT", g.LEFT_MARK_POS), ("MIDDLE", g.MID_MARK_POS), ("RIGHT", g.RIGHT_MARK_POS)):
        if math.hypot(x - mark_pos[0], y - mark_pos[1]) <= tol:
            return name
    return None


@dataclass
class TowerState:
    rule_broken: bool = False
    floor_fault: bool = False
    ever_25: bool = False
    ever_50: bool = False
    ever_75: bool = False
    ever_100: bool = False
    hold_time: float = 0.0
    solved: bool = False

    large_mark: str | None = "LEFT"
    med_mark: str | None = "LEFT"
    small_mark: str | None = "LEFT"

    large_resting: bool = True
    med_resting: bool = True
    small_resting: bool = True

    large_rest_time: float = 0.0
    med_rest_time: float = 0.0
    small_rest_time: float = 0.0


class TowerLogic:
    def __init__(self, hold_s: float = g.HOLD_TIME_S, rest_s: float = g.REST_TIME_S) -> None:
        self.hold_s = hold_s
        self.rest_s = rest_s
        self.state = TowerState()

    def reset(self) -> None:
        self.state = TowerState()

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
        arm_tcps: dict[str, tuple[float, float, float]] | None = None,
    ) -> TowerState:
        st = self.state

        # Check floor faults (dropped off table)
        for p in (large_pos, medium_pos, small_pos):
            if p[2] < g.TOP - 0.05:
                st.floor_fault = True

        # Velocity and resting tracking
        v_l = math.hypot(large_vel[0], large_vel[1], large_vel[2])
        v_m = math.hypot(medium_vel[0], medium_vel[1], medium_vel[2])
        v_s = math.hypot(small_vel[0], small_vel[1], small_vel[2])

        if v_l < g.STILL_LIN_VEL:
            st.large_rest_time += dt
        else:
            st.large_rest_time = 0.0
        st.large_resting = st.large_rest_time >= self.rest_s

        if v_m < g.STILL_LIN_VEL:
            st.med_rest_time += dt
        else:
            st.med_rest_time = 0.0
        st.med_resting = st.med_rest_time >= self.rest_s

        if v_s < g.STILL_LIN_VEL:
            st.small_rest_time += dt
        else:
            st.small_rest_time = 0.0
        st.small_resting = st.small_rest_time >= self.rest_s

        # Identify marks
        st.large_mark = mark_of_pos(large_pos)
        st.med_mark = mark_of_pos(medium_pos)
        st.small_mark = mark_of_pos(small_pos)

        # Zero-gate check: bigger block resting on a smaller one
        # "centre within 40 mm in xy and higher by >= 20 mm ... rests < 5 mm/s for 0.3 s"
        def rests_on(upper_pos, lower_pos, upper_resting, lower_resting):
            if not (upper_resting and lower_resting):
                return False
            d_xy = math.hypot(upper_pos[0] - lower_pos[0], upper_pos[1] - lower_pos[1])
            dz = upper_pos[2] - lower_pos[2]
            return (d_xy <= 0.040) and (dz >= 0.020)

        # Large on Medium
        if rests_on(large_pos, medium_pos, st.large_resting, st.med_resting):
            st.rule_broken = True
        # Large on Small
        if rests_on(large_pos, small_pos, st.large_resting, st.small_resting):
            st.rule_broken = True
        # Medium on Small
        if rests_on(medium_pos, small_pos, st.med_resting, st.small_resting):
            st.rule_broken = True

        # Tilts
        tilt_l = quat_tilt_deg(large_quat)
        tilt_m = quat_tilt_deg(medium_quat)
        tilt_s = quat_tilt_deg(small_quat)
        all_upright = (tilt_l <= g.MAX_TILT_DEG) and (tilt_m <= g.MAX_TILT_DEG) and (tilt_s <= g.MAX_TILT_DEG)

        # Check milestones:
        # 0.25: Small block is on MIDDLE or RIGHT mark (first move done) and resting
        if st.small_resting and st.small_mark in ("MIDDLE", "RIGHT"):
            st.ever_25 = True

        # 0.50: Large block is off LEFT and resting on the table (Level 1) at MIDDLE or RIGHT
        l_on_table = abs(large_pos[2] - g.LEVEL_1_TARGET_Z) <= 0.012
        if st.large_resting and l_on_table and st.large_mark in ("MIDDLE", "RIGHT"):
            st.ever_50 = True

        # 0.75: Large block is on RIGHT (Level 1) with a smaller block on it OR both others on MIDDLE
        l_on_right_table = st.large_resting and l_on_table and (st.large_mark == "RIGHT")
        med_or_small_on_large_right = l_on_right_table and (
            (rests_on(medium_pos, large_pos, st.med_resting, st.large_resting)) or
            (rests_on(small_pos, large_pos, st.small_resting, st.large_resting))
        )
        both_on_mid = (st.med_mark == "MIDDLE") and (st.small_mark == "MIDDLE") and st.med_resting and st.small_resting
        if l_on_right_table and (med_or_small_on_large_right or both_on_mid):
            st.ever_75 = True

        # 1.00: All three on RIGHT in order, offsets <= 12 mm from mark centre, tilt < 5 deg, arms clear, held 2 s
        d_l_r = math.hypot(large_pos[0] - g.RIGHT_MARK_POS[0], large_pos[1] - g.RIGHT_MARK_POS[1])
        d_m_r = math.hypot(medium_pos[0] - g.RIGHT_MARK_POS[0], medium_pos[1] - g.RIGHT_MARK_POS[1])
        d_s_r = math.hypot(small_pos[0] - g.RIGHT_MARK_POS[0], small_pos[1] - g.RIGHT_MARK_POS[1])

        z_levels_ok = (
            abs(large_pos[2] - g.LEVEL_1_TARGET_Z) <= 0.010 and
            abs(medium_pos[2] - g.LEVEL_2_TARGET_Z) <= 0.012 and
            abs(small_pos[2] - g.LEVEL_3_TARGET_Z) <= 0.014
        )

        all_still = (v_l < g.STILL_LIN_VEL) and (v_m < g.STILL_LIN_VEL) and (v_s < g.STILL_LIN_VEL)
        offsets_ok = (d_l_r <= g.FINAL_XY_TOL) and (d_m_r <= g.FINAL_XY_TOL) and (d_s_r <= g.FINAL_XY_TOL)

        arms_clear = True
        if arm_tcps is not None:
            for tcp in arm_tcps.values():
                d_tcp = math.hypot(tcp[0] - g.RIGHT_MARK_POS[0], tcp[1] - g.RIGHT_MARK_POS[1])
                if d_tcp < g.ARMS_CLEAR_DIST:
                    arms_clear = False
                    break

        all_three_ready = offsets_ok and z_levels_ok and all_upright and all_still and arms_clear and (not st.rule_broken) and (not st.floor_fault)

        if all_three_ready:
            st.hold_time += dt
            if st.hold_time >= self.hold_s:
                st.ever_100 = True
                st.solved = True
        else:
            st.hold_time = 0.0

        return st

