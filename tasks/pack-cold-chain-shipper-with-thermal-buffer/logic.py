"""State evaluation and transition logic for pack-cold-chain-shipper-with-thermal-buffer.

Tracks:
1. Gel brick placement flat in shipper base (Layer 1).
2. Thermal buffer barrier pad placement over gel brick (Layer 2).
3. Medication payload carton placement over buffer pad (Layer 3).
4. Direct contact freezing hazard detection (latched violation).
5. Shipper lid seated flat on the rim with both spring latches hanging over its end edges (the closing step).
6. Arm clearance and dwell hold verification for 1.00 full success.

The lid never adds credit by itself: it counts only when the three layers under it are placed in the right order and
no contact violation has latched. A lid on a stack with a layer missing scores what the layers present score.
"""


import math
from dataclasses import dataclass
import numpy as np

import geometry as g


def _tilt_deg(rot=None, quat=None) -> float:
    """Calculate tilt angle (degrees) from vertical Z axis."""
    if rot is not None:
        r22 = float(np.asarray(rot)[2, 2])
        return float(np.degrees(np.arccos(np.clip(r22, -1.0, 1.0))))
    if quat is not None:
        q = np.asarray(quat, dtype=float)
        # quat is [x, y, z, w]
        qx, qy, qz, qw = q
        r22 = 1.0 - 2.0 * (qx * qx + qy * qy)
        return float(np.degrees(np.arccos(np.clip(r22, -1.0, 1.0))))
    return 0.0


def _vel_norm(vel) -> float:
    if vel is None:
        return 0.0
    return float(np.linalg.norm(vel))


LATCH_NAMES = ("latch_pos_y", "latch_neg_y")   # the slider on the +Y end and the one on the -Y end of the shipper


def latch_overlaps(lid_y_rel: float, latch_press) -> tuple[float, float]:
    """How far each latch head hangs over the lid's end edge (m; positive = over the lid).

    ``lid_y_rel`` is the lid centre's Y offset from the shipper centre; ``latch_press`` maps each latch name to how far
    its slider is pushed outward (m, 0 at rest). A head at rest has its inner point LATCH_TIP_Y from the centreline;
    pushed out it sits LATCH_TIP_Y + press from it.
    """
    press = latch_press or {}
    tip_pos = g.LATCH_TIP_Y + abs(float(press.get(LATCH_NAMES[0], 0.0)))
    tip_neg = g.LATCH_TIP_Y + abs(float(press.get(LATCH_NAMES[1], 0.0)))
    over_pos = (lid_y_rel + g.LID_LENGTH_Y / 2.0) - tip_pos
    over_neg = (g.LID_LENGTH_Y / 2.0 - lid_y_rel) - tip_neg
    return float(over_pos), float(over_neg)


def evaluate_lid(lid_pos, lid_rot=None, lid_quat=None, lid_vel=None, latch_press=None, shipper_pos=None):
    """Lid seated flat on the rim and each latch engaged over its end edge. Returns (seated, (pos engaged, neg engaged))."""
    if lid_pos is None:
        return False, (False, False)
    p = np.asarray(lid_pos, dtype=float)
    off = np.zeros(2)
    if shipper_pos is not None:
        off = np.asarray(shipper_pos, dtype=float)[:2] - np.array([g.SHIPPER_X, g.SHIPPER_Y])
    dx, dy = float(p[0] - g.SHIPPER_X - off[0]), float(p[1] - g.SHIPPER_Y - off[1])
    z_ok = (g.LID_SEATED_Z - 0.001) <= float(p[2]) <= (g.LID_SEATED_Z + g.LID_SEAT_TOL_Z)
    seated = bool(
        abs(dx) <= g.LID_SEAT_TOL_XY
        and abs(dy) <= g.LID_SEAT_TOL_XY
        and z_ok
        and _tilt_deg(lid_rot, lid_quat) <= g.LID_TILT_TOL_DEG
        and _vel_norm(lid_vel) <= g.VEL_TOL
    )
    if latch_press is None:
        return seated, (False, False)
    over_pos, over_neg = latch_overlaps(dy, latch_press)
    return seated, (bool(seated and over_pos >= g.LATCH_MIN_OVERLAP), bool(seated and over_neg >= g.LATCH_MIN_OVERLAP))


@dataclass
class State:
    brick_placed: bool = False
    buffer_placed: bool = False
    carton_placed: bool = False
    safe_order: bool = False
    lid_seated: bool = False
    latch_pos_engaged: bool = False
    latch_neg_engaged: bool = False
    lid_closed: bool = False
    tilt_ok: bool = True
    arms_clear: bool = True
    direct_contact_violation: bool = False
    score: float = 0.0
    success: bool = False


def check_arms_clear(arm_tcps: dict[str, list[float]] | None) -> bool:
    """Verify both arms are withdrawn clear from the shipper work zone."""
    if arm_tcps is None:
        return True
    for arm, home in (("left", g.HOME_L), ("right", g.HOME_R)):
        tcp = arm_tcps.get(arm)
        if tcp is None:
            continue
        p = np.asarray(tcp[:3], dtype=float)
        # Clear if in home standby XY zone, or pulled well back, or parked high
        dist_home_xy = float(np.linalg.norm(p[:2] - np.array(home[:2])))
        pulled_back = p[0] < 0.200 and abs(p[1]) > 0.150
        parked_high = p[2] > (g.HOVER_Z + 0.02)
        if not (dist_home_xy <= 0.080 or pulled_back or parked_high):
            return False
    return True


def evaluate_step(
    brick_pos,
    brick_rot=None,
    brick_quat=None,
    brick_vel=None,
    buffer_pos=None,
    buffer_rot=None,
    buffer_quat=None,
    buffer_vel=None,
    carton_pos=None,
    carton_rot=None,
    carton_quat=None,
    carton_vel=None,
    arm_tcps=None,
    latched_violation: bool = False,
    lid_pos=None,
    lid_rot=None,
    lid_quat=None,
    lid_vel=None,
    latch_press=None,
    shipper_pos=None,
) -> tuple[State, bool]:
    """Evaluate current step geometry, detecting contacts and seated states."""
    st = State()
    p_brick = np.asarray(brick_pos, dtype=float)
    p_buffer = np.asarray(buffer_pos, dtype=float) if buffer_pos is not None else None
    p_carton = np.asarray(carton_pos, dtype=float) if carton_pos is not None else None

    tilt_brick = _tilt_deg(brick_rot, brick_quat)
    tilt_buffer = _tilt_deg(buffer_rot, buffer_quat) if p_buffer is not None else 0.0
    tilt_carton = _tilt_deg(carton_rot, carton_quat) if p_carton is not None else 0.0

    v_brick = _vel_norm(brick_vel)
    v_buffer = _vel_norm(buffer_vel)
    v_carton = _vel_norm(carton_vel)

    # 1. Gel brick seated flat in shipper base (Layer 1)
    brick_xy_ok = (
        abs(p_brick[0] - g.BRICK_TARGET_X) <= g.SEATED_TOL_XY
        and abs(p_brick[1] - g.BRICK_TARGET_Y) <= g.SEATED_TOL_XY
    )
    brick_z_ok = abs(p_brick[2] - g.BRICK_TARGET_Z) <= g.SEATED_TOL_Z
    brick_tilt_ok = tilt_brick <= g.TILT_TOL_DEG
    brick_vel_ok = v_brick <= g.VEL_TOL
    st.brick_placed = bool(brick_xy_ok and brick_z_ok and brick_tilt_ok and brick_vel_ok)

    # 2. Thermal buffer barrier pad seated over gel brick inside shipper (Layer 2)
    buffer_xy_ok = False
    buffer_z_ok = False
    buffer_tilt_ok = False
    buffer_vel_ok = False
    if p_buffer is not None:
        buffer_xy_ok = (
            abs(p_buffer[0] - g.BUFFER_TARGET_X) <= g.SEATED_TOL_XY
            and abs(p_buffer[1] - g.BUFFER_TARGET_Y) <= g.SEATED_TOL_XY
        )
        buffer_z_ok = (
            abs(p_buffer[2] - g.BUFFER_TARGET_Z) <= g.SEATED_TOL_Z
            and (p_buffer[2] > p_brick[2] + g.BRICK_HEIGHT_Z / 2.0 - 0.005)
        )
        buffer_tilt_ok = tilt_buffer <= g.TILT_TOL_DEG
        buffer_vel_ok = v_buffer <= g.VEL_TOL
    st.buffer_placed = bool(
        st.brick_placed and buffer_xy_ok and buffer_z_ok and buffer_tilt_ok and buffer_vel_ok
    )

    # 3. Medication carton seated over buffer pad inside shipper (Layer 3)
    carton_xy_ok = False
    carton_z_ok = False
    carton_tilt_ok = False
    carton_vel_ok = False
    if p_carton is not None:
        carton_xy_ok = (
            abs(p_carton[0] - g.CARTON_TARGET_X) <= g.SEATED_TOL_XY
            and abs(p_carton[1] - g.CARTON_TARGET_Y) <= g.SEATED_TOL_XY
        )
        carton_z_ok = (
            abs(p_carton[2] - g.CARTON_TARGET_Z) <= g.SEATED_TOL_Z
            and (p_buffer is not None and p_carton[2] > p_buffer[2] + g.BUFFER_HEIGHT_Z / 2.0 - 0.005)
        )
        carton_tilt_ok = tilt_carton <= g.TILT_TOL_DEG
        carton_vel_ok = v_carton <= g.VEL_TOL
    st.carton_placed = bool(
        st.buffer_placed and carton_xy_ok and carton_z_ok and carton_tilt_ok and carton_vel_ok
    )

    # 4. Check for freezing hazard: direct contact between medication carton and refrigerant brick
    contact_freeze_detected = False
    if p_carton is not None:
        carton_in_shipper_zone = (
            abs(p_carton[0] - g.SHIPPER_X) <= (g.CAVITY_WIDTH_X / 2.0 + 0.025)
            and abs(p_carton[1] - g.SHIPPER_Y) <= (g.CAVITY_LENGTH_Y / 2.0 + 0.025)
            and p_carton[2] <= (g.CAVITY_RIM_Z + 0.020)
        )
        brick_in_shipper_zone = (
            abs(p_brick[0] - g.SHIPPER_X) <= (g.CAVITY_WIDTH_X / 2.0 + 0.025)
            and abs(p_brick[1] - g.SHIPPER_Y) <= (g.CAVITY_LENGTH_Y / 2.0 + 0.025)
        )
        if carton_in_shipper_zone and brick_in_shipper_zone:
            sep_z = float(p_carton[2] - p_brick[2])
            # Without buffer pad separating them, vertical center separation drops < 30 mm
            if sep_z < g.CONTACT_FREEZE_DIST_Z:
                contact_freeze_detected = True
            elif p_buffer is None:
                contact_freeze_detected = True
            elif p_buffer[2] < p_brick[2] or p_buffer[2] > p_carton[2]:
                contact_freeze_detected = True
            else:
                buf_dist_xy = float(np.linalg.norm(p_buffer[:2] - np.array([g.SHIPPER_X, g.SHIPPER_Y])))
                if buf_dist_xy > 0.040:
                    contact_freeze_detected = True

    new_latched_violation = latched_violation or contact_freeze_detected
    st.direct_contact_violation = new_latched_violation

    # Safe order check
    if p_buffer is not None and p_carton is not None:
        st.safe_order = bool(
            p_brick[2] < p_buffer[2] < p_carton[2]
            and st.brick_placed
            and st.buffer_placed
            and st.carton_placed
        )
    else:
        st.safe_order = False

    st.tilt_ok = bool(
        brick_tilt_ok
        and (buffer_tilt_ok if p_buffer is not None else True)
        and (carton_tilt_ok if p_carton is not None else True)
    )
    st.arms_clear = check_arms_clear(arm_tcps)

    # 5. Lid seated flat on the rim with both latch heads over its end edges. It counts only on a correct stack:
    # all three layers placed in order and no contact violation. Otherwise the lid adds nothing.
    seated, (eng_pos, eng_neg) = evaluate_lid(lid_pos, lid_rot, lid_quat, lid_vel, latch_press, shipper_pos)
    st.lid_seated, st.latch_pos_engaged, st.latch_neg_engaged = seated, eng_pos, eng_neg
    st.lid_closed = bool(
        seated
        and eng_pos
        and eng_neg
        and st.brick_placed
        and st.buffer_placed
        and st.carton_placed
        and st.safe_order
        and not st.direct_contact_violation
    )

    # Base instantaneous score calculation
    if st.direct_contact_violation:
        st.score = 0.25 if st.brick_placed else 0.0
    elif st.carton_placed and st.buffer_placed and st.brick_placed:
        st.score = 0.75
    elif st.buffer_placed and st.brick_placed:
        st.score = 0.50
    elif st.brick_placed:
        st.score = 0.25
    else:
        st.score = 0.0

    return st, new_latched_violation
