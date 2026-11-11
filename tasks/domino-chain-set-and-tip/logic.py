"""Pure Python stateful logic for domino-chain-set-and-tip."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
import numpy as np

import geometry as g


def quat_tilt_deg(quat: tuple[float, float, float, float] | np.ndarray) -> float:
    """Compute tilt from vertical Z-axis in degrees from quaternion (x, y, z, w)."""
    qx, qy, qz, qw = float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])
    # z_z = 1 - 2*(qx*qx + qy*qy)
    z_z = 1.0 - 2.0 * (qx * qx + qy * qy)
    z_z = max(-1.0, min(1.0, z_z))
    return float(np.degrees(math.acos(z_z)))


def is_fallen_plus_x(quat: tuple[float, float, float, float] | np.ndarray) -> bool:
    """Check if domino has fallen (tilt > 60 deg) toward world +x.

    Local Z in world frame: (rx, ry, rz).
    rx = 2 * (qx*qz + qy*qw)
    ry = 2 * (qy*qz - qx*qw)
    rz = 1 - 2 * (qx*qx + qy*qy)
    """
    qx, qy, qz, qw = float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])
    rx = 2.0 * (qx * qz + qy * qw)
    ry = 2.0 * (qy * qz - qx * qw)
    rz = 1.0 - 2.0 * (qx * qx + qy * qy)
    rz = max(-1.0, min(1.0, rz))
    tilt_deg = float(np.degrees(math.acos(rz)))

    # Fallen = tilt > 60 deg toward +x
    # rx must be positive (forward along +x) and dominate lateral tilt |ry|
    return tilt_deg > g.FALLEN_TILT_DEG and rx > 0.0 and rx > abs(ry)


def mark_index_of_pos(pos: tuple[float, float, float] | np.ndarray, tol: float = g.MARK_XY_TOL) -> int | None:
    """Identify which mark (0..3) a domino is over, if within tol in XY."""
    x, y = float(pos[0]), float(pos[1])
    for idx, mark_pos in enumerate(g.MARK_POSITIONS):
        if math.hypot(x - mark_pos[0], y - mark_pos[1]) <= tol:
            return idx
    return None


@dataclass
class DominoState:
    # Milestones & latches
    ever_standing_2: bool = False
    ever_standing_4: bool = False
    first_tipped: bool = False
    ever_fallen_4: bool = False
    premature: bool = False
    solved: bool = False

    # Current step tracking
    standing_count: int = 0
    marks_occupied: list[int] = field(default_factory=list)
    fallen_count: int = 0
    tilts: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    rest_times: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    arms_clear: bool = True
    tip_timer: float = 0.0
    floor_fault: bool = False


class DominoLogic:
    def __init__(
        self,
        rest_s: float = g.REST_TIME_S,
        max_tip_time_s: float = g.MAX_TIP_TO_FALL_TIME_S,
    ) -> None:
        self.rest_s = rest_s
        self.max_tip_time_s = max_tip_time_s
        self.state = DominoState()

    def reset(self) -> None:
        self.state = DominoState()

    def step(
        self,
        dt: float,
        domino_poses: list[tuple[float, float, float] | np.ndarray],
        domino_quats: list[tuple[float, float, float, float] | np.ndarray],
        domino_vels: list[tuple[float, float, float] | np.ndarray],
        arm_tcps: dict[str, tuple[float, float, float]] | None = None,
    ) -> DominoState:
        st = self.state

        # Floor fault check
        for p in domino_poses:
            if p[2] < g.TOP - 0.05:
                st.floor_fault = True

        # Track velocities and resting times
        st.tilts = [quat_tilt_deg(q) for q in domino_quats]
        for i, vel in enumerate(domino_vels):
            v_lin = math.hypot(vel[0], vel[1], vel[2])
            if v_lin < g.STILL_LIN_VEL:
                st.rest_times[i] += dt
            else:
                st.rest_times[i] = 0.0

        # Standing check per domino:
        # Standing = tilt < 6 deg, resting 0.3 s, on mark within 8 mm
        occupied_marks = {}
        for i in range(len(domino_poses)):
            tilt_ok = st.tilts[i] < g.MAX_STANDING_TILT_DEG
            rest_ok = st.rest_times[i] >= self.rest_s
            midx = mark_index_of_pos(domino_poses[i], tol=g.MARK_XY_TOL)
            if tilt_ok and rest_ok and midx is not None:
                occupied_marks[midx] = i

        st.marks_occupied = sorted(occupied_marks.keys())
        st.standing_count = len(st.marks_occupied)

        # Latch 2 and 4 standing milestones
        if st.standing_count >= 2:
            st.ever_standing_2 = True
        if st.standing_count == 4:
            st.ever_standing_4 = True

        # Zero-gate check: any domino falling (tilt > 45 deg) before all four are standing = premature
        # Exempt dominoes that are still in staging area (|y| > 0.15 and not on mark)
        if not st.ever_standing_4:
            for i in range(len(domino_poses)):
                # If a domino is on/near the mark strip (|y| <= 0.15) or has ever stood/moved, check tilt
                in_play = abs(domino_poses[i][1]) <= 0.15 or math.hypot(
                    domino_poses[i][0] - g.DOMINO_START_POSITIONS[i][0],
                    domino_poses[i][1] - g.DOMINO_START_POSITIONS[i][1],
                ) > 0.030
                if in_play and st.tilts[i] > g.PREMATURE_TILT_DEG:
                    st.premature = True

        # Check first domino tipped:
        # First domino is on Mark 0 (x=0.26, y=0.0). Tipped = tilt > 20 deg while others standing
        if st.ever_standing_4 and not st.first_tipped:
            # Check if first domino (near mark 0) has tilt > 20 deg
            first_idx = None
            for i, pos in enumerate(domino_poses):
                if math.hypot(pos[0] - g.MARK_0_POS[0], pos[1] - g.MARK_0_POS[1]) <= 0.030:
                    first_idx = i
                    break
            if first_idx is not None and st.tilts[first_idx] > g.TIPPED_TILT_DEG:
                # Check that others are standing or just starting to topple
                st.first_tipped = True
                st.tip_timer = 0.0

        if st.first_tipped:
            st.tip_timer += dt

        # Fallen count (all four fallen toward +x)
        fallen_flags = [is_fallen_plus_x(q) for q in domino_quats]
        st.fallen_count = sum(1 for f in fallen_flags if f)

        # Arms clear check: TCPs > 60 mm from all marks
        st.arms_clear = True
        if arm_tcps is not None:
            for tcp in arm_tcps.values():
                for m_pos in g.MARK_POSITIONS:
                    if math.hypot(tcp[0] - m_pos[0], tcp[1] - m_pos[1]) < g.ARMS_CLEAR_DIST:
                        st.arms_clear = False
                        break
                if not st.arms_clear:
                    break

        # 1.00 requirement: all four fallen toward +x within 5 s of tip, arms clear
        if (
            st.first_tipped
            and st.tip_timer <= self.max_tip_time_s
            and st.fallen_count == 4
            and st.arms_clear
            and not st.floor_fault
        ):
            st.ever_fallen_4 = True
            st.solved = True

        return st
