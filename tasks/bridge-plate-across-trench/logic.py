"""Pure Python stateful logic for bridge-plate-across-trench."""

from __future__ import annotations

import math
from dataclasses import dataclass
import numpy as np

import geometry as g


def quat_tilt_deg(quat: tuple[float, float, float, float] | np.ndarray) -> float:
    """Compute tilt from vertical Z-axis in degrees from quaternion (x, y, z, w)."""
    qx, qy, qz, qw = float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])
    # z_axis_z = 1 - 2*(qx*qx + qy*qy)
    z_z = 1.0 - 2.0 * (qx * qx + qy * qy)
    z_z = max(-1.0, min(1.0, z_z))
    return float(np.degrees(math.acos(z_z)))


@dataclass
class BridgePlateState:
    fell: bool = False
    ever_25: bool = False
    ever_50: bool = False
    ever_75: bool = False
    ever_100: bool = False
    hold_time: float = 0.0
    solved: bool = False

    bridge_seated: bool = False
    block_on_bridge: bool = False
    block_on_far: bool = False
    block_centered: bool = False

    bridge_resting: bool = True
    block_resting: bool = True
    bridge_rest_time: float = 0.0
    block_rest_time: float = 0.0

    near_overlap: float = 0.0
    far_overlap: float = 0.0
    bridge_tilt: float = 0.0
    target_dist: float = 0.0


class BridgeLogic:
    def __init__(self, hold_s: float = g.HOLD_TIME_S, rest_s: float = g.REST_TIME_S) -> None:
        self.hold_s = hold_s
        self.rest_s = rest_s
        self.state = BridgePlateState()

    def reset(self) -> None:
        self.state = BridgePlateState()

    def step(
        self,
        dt: float,
        bridge_pos: tuple[float, float, float] | np.ndarray,
        bridge_quat: tuple[float, float, float, float] | np.ndarray,
        bridge_vel: tuple[float, float, float] | np.ndarray,
        block_pos: tuple[float, float, float] | np.ndarray,
        block_quat: tuple[float, float, float, float] | np.ndarray,
        block_vel: tuple[float, float, float] | np.ndarray,
        arm_tcps: dict[str, tuple[float, float, float]] | None = None,
    ) -> BridgePlateState:
        st = self.state

        # Zero-gate check: fell into trench (latches permanently)
        if bridge_pos[2] < g.FELL_Z_THRESHOLD or block_pos[2] < g.FELL_Z_THRESHOLD:
            st.fell = True

        # Resting tracking
        v_bridge = math.hypot(bridge_vel[0], bridge_vel[1], bridge_vel[2])
        v_block = math.hypot(block_vel[0], block_vel[1], block_vel[2])

        if v_bridge < g.STILL_LIN_VEL:
            st.bridge_rest_time += dt
        else:
            st.bridge_rest_time = 0.0
        st.bridge_resting = st.bridge_rest_time >= self.rest_s

        if v_block < g.STILL_LIN_VEL:
            st.block_rest_time += dt
        else:
            st.block_rest_time = 0.0
        st.block_resting = st.block_rest_time >= self.rest_s

        # Bridge geometry and overlap check
        # Bridge length is 130 mm (y in [pos[1]-0.065, pos[1]+0.065])
        h_y = g.BRIDGE_SIZE[1] / 2.0  # 0.075 m
        y_near_edge = bridge_pos[1] + h_y
        y_far_edge = bridge_pos[1] - h_y

        st.near_overlap = y_near_edge - g.NEAR_PLATFORM_LIP_Y
        st.far_overlap = g.FAR_PLATFORM_LIP_Y - y_far_edge
        st.bridge_tilt = quat_tilt_deg(bridge_quat)

        overlaps_ok = (st.near_overlap >= g.BRIDGE_MIN_OVERLAP - 1e-4) and (st.far_overlap >= g.BRIDGE_MIN_OVERLAP - 1e-4)
        tilt_ok = st.bridge_tilt < g.BRIDGE_MAX_TILT_DEG
        height_ok = abs(bridge_pos[2] - (g.PLATFORM_TOP_Z + g.BRIDGE_SIZE[2] / 2.0)) <= 0.012
        lane_ok = abs(bridge_pos[0] - g.BRIDGE_START_POS[0]) <= 0.040

        st.bridge_seated = overlaps_ok and tilt_ok and height_ok and lane_ok and st.bridge_resting and (not st.fell)

        if st.bridge_seated:
            st.ever_25 = True

        # Block on bridge check
        # Block center within bridge footprint in XY, resting at bridge level
        h_x = g.BRIDGE_SIZE[0] / 2.0  # 0.040 m
        in_bridge_x = abs(block_pos[0] - bridge_pos[0]) <= h_x + 0.010
        in_bridge_y = abs(block_pos[1] - bridge_pos[1]) <= h_y + 0.010
        # Block on bridge: in trench region or on bridge surface
        crossing_zone = (block_pos[1] < g.NEAR_PLATFORM_LIP_Y + 0.010) and (block_pos[1] > g.FAR_PLATFORM_LIP_Y - 0.010)
        on_bridge_z = abs(block_pos[2] - (g.PLATFORM_TOP_Z + g.BRIDGE_SIZE[2] + g.BLOCK_SIZE[2] / 2.0)) <= 0.015

        st.block_on_bridge = in_bridge_x and in_bridge_y and crossing_zone and on_bridge_z and (not st.fell)
        if st.ever_25 and st.block_on_bridge:
            st.ever_50 = True

        # Block on far platform check
        # Block past the trench lip onto far platform (y <= -0.045)
        on_far_y = block_pos[1] <= g.FAR_PLATFORM_LIP_Y + 1e-4
        on_far_x = abs(block_pos[0] - g.FAR_PLATFORM_POS[0]) <= (g.PLATFORM_X_SIZE / 2.0 + 0.020)
        on_far_z = abs(block_pos[2] - (g.PLATFORM_TOP_Z + g.BLOCK_SIZE[2] / 2.0)) <= 0.015

        st.block_on_far = on_far_y and on_far_x and on_far_z and (not st.fell)
        if st.ever_50 and st.block_on_far:
            st.ever_75 = True

        # Target centering & completion check (1.00)
        st.target_dist = math.hypot(block_pos[0] - g.TARGET_MARK_POS[0], block_pos[1] - g.TARGET_MARK_POS[1])
        st.block_centered = (st.target_dist <= g.TARGET_XY_TOL) and (v_block < g.STILL_LIN_VEL)

        # Arms clear check
        arms_clear = True
        if arm_tcps is not None:
            for tcp in arm_tcps.values():
                d_block = math.hypot(tcp[0] - block_pos[0], tcp[1] - block_pos[1])
                d_target = math.hypot(tcp[0] - g.TARGET_MARK_POS[0], tcp[1] - g.TARGET_MARK_POS[1])
                if min(d_block, d_target) < g.ARMS_CLEAR_DIST:
                    arms_clear = False
                    break

        all_complete = (
            st.block_centered and
            st.bridge_seated and
            arms_clear and
            (not st.fell)
        )

        if all_complete:
            st.hold_time += dt
            if st.hold_time >= self.hold_s:
                st.ever_100 = True
                st.solved = True
        else:
            st.hold_time = 0.0

        return st
