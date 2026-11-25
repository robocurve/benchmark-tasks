"""Return-to-mark tail as a drop-in interface for the integrated dustpan plan (owner claude-cake; codex 2146).
It follows antigravity-microscope's end_to_end_splice_helper_for_cake.py: called AFTER the splice's right upper-shaft
re-pinch, it returns the right-arm waypoints from the hanging carry to the final hold.
The geometry is NATIVE-validated in return2/return3 (rear-edge pivot; pan flat on the mark; hands 118-120 mm in the final hold)
with ONE change: the vertical lift_clear (native contact at step 360) is replaced by move_away_diagonal
(retreat 80 mm, then +(-0.08, -0.10, +0.08) m; CPU TCP clearance 38 -> 136 mm, monotonic; IK 0.06 mm).
Latches are keyed to `latch_step`: the integrated control step at which the right re-pinch has settled. Pass it from
the splice, never a nominal number. Before that step, the nominal fallbacks below apply and are DISCLOSED as nominal.
No dump pose is assumed here; the live trunnion and handle axis come from the latches."""
from __future__ import annotations

import math

import numpy as np

P0_DEG = 54.0                                   # measured hang pitch (return2/return3: 54.4 deg)
EDGE_TO_TRUN = (0.165, 0.105)                   # pan frame: rear bottom edge (0,-0.165,0) -> trunnion (0,0,0.105)
ANCHOR_SHIFT = (+0.025, +0.008)                 # rev3 tape rule: footprint-centred landing (demo3b landed pivot +19/-15 mm, yaw -7.8 deg, rear corner 13 mm outside the tape); was (+0.037, -0.029)
FAR_PARK = (0.18, -0.36, 0.85)                  # >= 149.6 mm TCP clearance (park_check.py), native 118-120 mm at the pads
AWAY = np.array([-0.08, -0.10, +0.08])          # diagonal move-away after the 80 mm retreat


def make_return_tail(*, Waypoint, Orient, _LivePos, _latched, _pad_tcp, _handle_dir_world, PARK_XY, FLOOR, TRUN_Z,
                     R_r0, R_park, SHAFT_PINCH, SHAFT_OPEN, FINGER_OPEN, latch_step: int, grip_s: float = 0.680):
    def _hdir():
        h = _latched("tail_hdir", latch_step, lambda dp: _handle_dir_world(dp))
        return np.array([0.0, 0.0, 1.0]) if h is None else np.asarray(h, float) / np.linalg.norm(h)

    def _trun_hang_z():
        t = _latched("tail_trun", latch_step, lambda dp: np.asarray(dp.to_world((0.0, 0.0, TRUN_Z)), float))
        return 0.30 if t is None else float(t[2])

    e2t = lambda pr: np.array([0.0, EDGE_TO_TRUN[0] * math.cos(pr) - EDGE_TO_TRUN[1] * math.sin(pr),
                               EDGE_TO_TRUN[0] * math.sin(pr) + EDGE_TO_TRUN[1] * math.cos(pr)])
    touch_y = PARK_XY[1] - 0.141
    edge = np.array([PARK_XY[0] + ANCHOR_SHIFT[0], touch_y - 0.012 + ANCHOR_SHIFT[1], FLOOR])
    tcp_for = lambda trun: _pad_tcp(np.asarray(trun, float) + _hdir() * (grip_s - TRUN_Z), R_r0)
    arc = lambda pdeg, above=0.0: tcp_for(edge + np.array([0.0, 0.0, above]) + e2t(math.radians(pdeg)))
    rest_hp = lambda: edge + e2t(0.0) + _hdir() * (grip_s - TRUN_Z)
    p_ret = lambda: _pad_tcp(rest_hp(), R_r0, back=0.08)
    return [
        Waypoint(1.4, _LivePos(lambda: tcp_for([PARK_XY[0], touch_y, _trun_hang_z()])), SHAFT_PINCH, "tail_level_to_touch_xy", Orient(R_r0)),
        Waypoint(2.0, _LivePos(lambda: arc(P0_DEG, above=0.010)), SHAFT_PINCH, "tail_lower_rear_edge_to_10mm", Orient(R_r0)),
        Waypoint(0.7, _LivePos(lambda: arc(P0_DEG)), SHAFT_PINCH, "tail_rear_edge_touch", Orient(R_r0)),
        Waypoint(0.7, _LivePos(lambda: arc(40.0)), SHAFT_PINCH, "tail_pivot_40", Orient(R_r0)),
        Waypoint(0.7, _LivePos(lambda: arc(25.0)), SHAFT_PINCH, "tail_pivot_25", Orient(R_r0)),
        Waypoint(0.7, _LivePos(lambda: arc(10.0)), SHAFT_PINCH, "tail_pivot_10", Orient(R_r0)),
        Waypoint(0.7, _LivePos(lambda: arc(0.0)), SHAFT_PINCH, "tail_pivot_flat", Orient(R_r0)),
        Waypoint(0.8, _LivePos(lambda: arc(0.0)), SHAFT_PINCH, "settle_pan_on_floor", Orient(R_r0)),
        Waypoint(0.7, _LivePos(lambda: arc(0.0)), SHAFT_OPEN, "release_parked_handle", Orient(R_r0)),
        Waypoint(0.9, _LivePos(p_ret), FINGER_OPEN, "retreat_from_handle", Orient(R_r0)),
        Waypoint(1.0, _LivePos(lambda: np.asarray(p_ret(), float) + AWAY), FINGER_OPEN, "move_away_diagonal", Orient(R_r0)),
        Waypoint(1.2, FAR_PARK, FINGER_OPEN, "park_right_far", Orient(R_park)),
        Waypoint(2.0, FAR_PARK, FINGER_OPEN, "final_stillness_hold", Orient(R_park)),
    ]


TAIL_SECONDS = 1.4 + 2.0 + 0.7 * 5 + 0.8 + 0.7 + 0.9 + 1.0 + 1.2 + 2.0      # 13.5 s = 270 steps
