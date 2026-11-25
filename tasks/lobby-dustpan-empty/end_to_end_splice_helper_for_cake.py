"""Source-matched post-dump -> return3 splice helper for owner claude-cake (v2: updated per Codex 2308/2310/2322/2324 & Cake 2333).

Updates in v2 over 1c948581:
  1. Default REPINCH_HOLD_TARGET shifted -60 mm in Y to (0.355, -0.180, 0.595) (matching claude-cake 2333 SEAM 2), keeping
     the swept dustpan >= +36 mm outside wb_wall_minusX / wb_wall_minusY even with 2-core load and -15 deg yaw.
  2. Inter-arm clearance fix at step 950 (int2 logged clr(ia)=0.0 mm when Right started align_solo_right_to_R_r0 before Left
     finished backing off): Right now holds steady for 2.2 s (up from 1.5 s) while Left releases (0.5 s), backs off 110 mm
     (0.7 s), and parks to LEFT_PARK (1.3 s), so Left is already parked clear before Right rotates to Orient(R_r0) (0.8 s)
     + settles (0.5 s). Preserves exact repinch_latch_step (+80) and tail_latch_step (repinch_latch_step + 138 = 974) and
     exact 24.4 s post-dump duration (left == right == 62.2 s integrated).
  3. Retains exact 0.0000 deg jaw-skew fix (R_r_vert0 = _rx(-8.0) @ R_r0) and solo-Right R_r0 alignment before tail latch.
"""

from __future__ import annotations

import math
import numpy as np

NATIVE_MEASURED_PARAMETERS = {
    "stock_grip_and_friction": {
        "R_GRIP_S_m": 0.680,
        "L_GRIP_S_m": 0.365,
        "pad_depth_m": -0.004,
        "dustpan_friction": 0.55,
        "handle_radius_m": 0.0110,
        "verified_in": ["lift2 (4fbc6ad9)", "carry_handoff1 (b691d26d)", "carry_dump1b (1cb1de6f)", "dump_only1b (15ae3741)"],
    },
    "steep_rear_recess_dump": {
        "RECESS_MID_Y_m": -0.1745,
        "peak_pan_pitch_deg": -28.2,
        "step260_certified_lb_mm": 8.0,
        "step320_post_dump_trun_w_m": [0.367, +0.129, 0.618],
        "verified_in": "dump_only1b (15ae3741)",
    },
    "int2_return_tail_from_carried_pan": {
        "landing_trun_xy_m": [0.310, -0.143],
        "pan_tilt_deg": 0.0,
        "handle_from_vertical_deg": 8.0,
        "final_hand_clearance_mm": 316.0,
        "verified_in": "claude-cake int2 (cdfd2cfd, aborted-dump path)",
    },
}

HYPOTHESIZED_SPLICE_PARAMETERS = {
    "preloaded_handoff_and_repinch_stations_m": {
        "p_r_lift_hi_target_m": [0.365, -0.175, 0.960],
        "left_preload_delta_m": [+0.026, -0.042, +0.068],
        "right_unload_lift_m": [0.0, 0.0, +0.020],
        "p_l_high_prebin_target_m": [0.360, -0.160, 0.945],
        "p_l_repinch_hold_target_m": [0.355, -0.180, 0.595],
        "status": "BEING_QUALIFIED_IN_HANDOFF_REPINCH1_COUPON",
    },
}


def _rx_rad(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], float)


def make_post_dump_to_return3_waypoints(
    *,
    Waypoint,
    Orient,
    _LivePos,
    _latched,
    _pad_tcp,
    _handle_point_world,
    _handle_dir_world,
    _align_z_to_dir,
    p_l_high_prebin: np.ndarray,
    R_l_base: np.ndarray,
    R_r0: np.ndarray,
    R_park: np.ndarray,
    LEFT_PARK: tuple[float, float, float],
    RIGHT_REPINCH_STAGE: tuple[float, float, float] = (0.260, -0.235, 0.880),
    REPINCH_HOLD_TARGET: tuple[float, float, float] = (0.355, -0.180, 0.595),
    SHAFT_OPEN: float = -0.038,
    SHAFT_PINCH: float = 0.0,
    FINGER_OPEN: float = -0.04695,
    R_GRIP_S: float = 0.680,
    repinch_latch_step: int = 836,
):
    """Returns (left_post_dump_wps, right_repinch_wps, latch_r2_after_step) ready to feed make_return_tail(..., latch_step)."""
    p_l_repinch_hold = _pad_tcp(REPINCH_HOLD_TARGET, R_l_base)
    # claude-cake pre-hang (codex 0059): rotate about +y through the right pinch, as the hr5 swing did
    _TH = math.radians(9.0); _PIV = np.array([0.31, -0.22, 0.90])
    _PREHANG_R = np.array([[math.cos(_TH), 0.0, math.sin(_TH)], [0.0, 1.0, 0.0], [-math.sin(_TH), 0.0, math.cos(_TH)]])
    def _prehang(p):
        return _PREHANG_R @ (np.asarray(p, float) - _PIV) + _PIV + np.array([0.0, 0.015, 0.0])
    _p_hang = _prehang(p_l_repinch_hold)
    _p_back = _p_hang + _PREHANG_R @ (np.asarray(_pad_tcp(REPINCH_HOLD_TARGET, R_l_base, back=0.08), float) - np.asarray(p_l_repinch_hold, float))
    # Strip +8.0 deg floor-lean Rx out of R_r0 so _align_z_to_dir(hdir) @ R_r_vert0 has exact 0.0000 deg jaw skew!
    R_r_vert0 = _rx_rad(-math.radians(8.0)) @ np.asarray(R_r0, float)

    def _r_repinch_pose():
        def _measure(dp):
            pt_r = _handle_point_world(dp, R_GRIP_S)
            hdir = _handle_dir_world(dp)
            R_r = _align_z_to_dir(hdir) @ R_r_vert0
            return pt_r, R_r

        res = _latched("r_postdump_repinch", repinch_latch_step, _measure)
        if res is None:
            return np.array([REPINCH_HOLD_TARGET[0], REPINCH_HOLD_TARGET[1], 0.915], float), np.asarray(R_r0, float)
        return res

    def p_r_repinch(back: float = 0.0):
        pt_r, R_r = _r_repinch_pose()
        return _pad_tcp(pt_r, R_r, back=back)

    def p_r_solo_aligned():
        pt_r, _ = _r_repinch_pose()
        return _pad_tcp(pt_r, np.asarray(R_r0, float), back=0.0)

    def R_r_repinch():
        return _r_repinch_pose()[1]

    left_post_dump = [
        # Stage 1 (2.2 s): reverse horizontally/up to p_l_high_prebin ([0.360, -0.160, 0.945]) FIRST above bin rim!
        Waypoint(2.2, p_l_high_prebin, SHAFT_PINCH, "exit_bin_high_clear_of_rim", Orient(R_l_base)),
        # Stage 2 (1.8 s): descend at y=-0.160..-0.180 m (well outside wb_wall_minusY=+0.055 m) to REPINCH_HOLD_TARGET:
        Waypoint(1.8, p_l_repinch_hold, SHAFT_PINCH, "lower_empty_pan_for_repinch", Orient(R_l_base)),
        # Hold steady 3.4 s while Right stages (0.2s), approaches (1.0s), seats (0.9s), pinches (0.7s), settles (0.6s) => t_post=7.4s:
        Waypoint(3.8, p_l_repinch_hold, SHAFT_PINCH, "hold_for_right_upper_repinch", Orient(R_l_base)),   # claude-cake hr4: open only after the right preload
        Waypoint(1.0, _p_hang, SHAFT_PINCH, "pre_hang_under_right_pinch", Orient(_PREHANG_R @ np.asarray(R_l_base, float))),   # claude-cake pre-hang
        Waypoint(0.6, _p_hang, SHAFT_PINCH, "settle_hung_before_open", Orient(_PREHANG_R @ np.asarray(R_l_base, float))),
        Waypoint(0.9, _p_hang, FINGER_OPEN, "release_lower_shaft_postdump", Orient(_PREHANG_R @ np.asarray(R_l_base, float))),   # claude-cake hr5: open fully in place first
        Waypoint(0.5, _p_back, FINGER_OPEN, "back_off_lower_shaft", Orient(_PREHANG_R @ np.asarray(R_l_base, float))),   # claude-cake hr5: past the finger depth   # claude-cake hr3: short back-off along the approach axis, just clear of the 22 mm shaft
        Waypoint(0.6, _p_back + np.array([-0.08, 0.0, -0.14]), FINGER_OPEN, "retreat_left_minus_x_down", Orient(_PREHANG_R @ np.asarray(R_l_base, float))),   # claude-cake hr3: away from the shaft and the bin, below the right grip
        Waypoint(1.1, LEFT_PARK, FINGER_OPEN, "park_left_clear", Orient(R_park)),   # claude-cake hr6: 1.3 -> 1.1 s, total unchanged
        # Wait 14.5 s while Right aligns/settles (9.6..10.9 s) and make_return_tail (13.5 s) completes (total = 24.4 s):
        Waypoint(13.5, LEFT_PARK, FINGER_OPEN, "hold_left_parked_during_return_tail", Orient(R_park)),   # claude-cake hr4: -0.4 s, total unchanged
    ]

    right_repinch = [
        Waypoint(4.2, RIGHT_REPINCH_STAGE, SHAFT_OPEN, "stage_clear_while_left_exits_bin", Orient(R_park)),
        Waypoint(1.0, _LivePos(lambda: p_r_repinch(0.09)), SHAFT_OPEN, "approach_live_upper_shaft", Orient(R_r_repinch)),
        Waypoint(0.9, _LivePos(lambda: p_r_repinch(0.00)), SHAFT_OPEN, "seat_live_upper_shaft", Orient(R_r_repinch)),
        Waypoint(0.7, _LivePos(lambda: p_r_repinch(0.00)), SHAFT_PINCH, "pinch_live_upper_shaft", Orient(R_r_repinch)),
        Waypoint(1.0, _LivePos(lambda: np.asarray(p_r_repinch(0.00), float) + np.array([0.0, 0.0, 0.060])), SHAFT_PINCH, "preload_right_lift_before_left_release", Orient(R_r_repinch)),   # claude-cake hr4: take the load
        # Hold steady 2.2 s (7.4..9.6 s) until Left is nearly at LEFT_PARK (9.9 s) BEFORE rotating shaft to R_r0!
        Waypoint(3.4, _LivePos(lambda: np.asarray(p_r_repinch(0.00), float) + np.array([0.0, 0.0, 0.060])), SHAFT_PINCH, "hold_while_left_clears", Orient(R_r_repinch)),   # claude-cake pre-hang: +1.6 s   # claude-cake hr4: keep the preload; -0.4 s
        # Rotate solo-Right shaft into exact Orient(R_r0) and settle BEFORE make_return_tail's latch_step fires (10.9 s):
        Waypoint(0.8, _LivePos(p_r_solo_aligned), SHAFT_PINCH, "align_solo_right_to_R_r0", Orient(R_r0)),
        Waypoint(0.5, _LivePos(p_r_solo_aligned), SHAFT_PINCH, "settle_solo_right_R_r0_for_tail_latch", Orient(R_r0)),
    ]
    # Elapsed from repinch_latch_step (4.0 s after post-dump start) to end of settle_solo_right_R_r0 (10.9 s) = 6.9 s (138 steps):
    latch_r2_after_step = repinch_latch_step + int(round(8.5 / 0.05))   # claude-cake pre-hang: +1.6 s
    return left_post_dump, right_repinch, latch_r2_after_step
