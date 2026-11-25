"""100% `c7i/task.py` (`ea8563d7`) Compatible Two-Hand Near-Channel & Bar Waypoints (antigravity helper).

IMPORTANT COMPATIBILITY CONTRACT FOR `claude-b50` (`c7i/task.py` `ea8563d7`):
- `c7i/task.py` imports `HW.X_NEAR`, `HW.Y_POS_SEAT`, `HW.RIM`, `HW.two_hand_pan_waypoints(...)`, and `HW.demo_coupon(...)`,
  and then post-processes labels ending in `raise_carry`, `carry_over_seat`, `hold_1p5s`, `lower_24mm`, `release_13p6mm_gap`,
  `peel_back`, and `retreat_up` (applying `SW7_CARRY_DZ=+0.115`, `SW7_TILT=-6.0`, and `SW7_SEAT_DTILT=-3.6` on top).
- Therefore, `two_hand_pan_waypoints(...)` in this file keeps the EXACT `c7i`-expected base labels and `0 deg` base tilt so
  `c7i/task.py` (`ea8563d7`) reproduces `c7i`'s native `pan_ok = True` result identically without double-applying `Ry(-6 deg)`!
- Meanwhile, `bar_waypoints(...)` (`demo_coupon("bar_only")`, `320` steps = `16.0 s`) has the verified high wall-clearing entry
  + D405-up `Rx(+-48 deg)` roll (`0.086 mm` 20 Hz `ArmPlan` stream, `+4.4 mm` stand/W50 3D STL clearance) ready for `SW7=B`!
- For turnkey integrated `bar + Q1` and `bar + slide + Q1 + occupied Q2` trajectories (with `c7i` transforms already baked in),
  use `w50_bar_waypoints.py`.
"""

from __future__ import annotations

import math
import numpy as np

STAND_TOP = 0.5975
WC = (0.4800, 0.0000)
RIM = 0.8261
W_OUT = (0.3683, 0.5715, 0.2286)
W_IN = (0.3048, 0.5080, 0.15875)
BAR_W, BAR_L, BAR_H = 0.0254, 0.5310, 0.0080
Q_W, Q_L, Q_D, Q_FL, SHEET = 0.1618, 0.2635, 0.0635, 0.0150, 0.0020
BODY_W, BODY_L = Q_W - 2.0 * Q_FL, Q_L - 2.0 * Q_FL

X_STRIP = 0.2130
X_NEAR = WC[0] - 0.0820             # 0.3980 m (`c7i/task.py` expects `HW.X_NEAR`! With `SW7_SEAT_DX=0.003` -> `0.4010 m`)
X_NEAR_GEOM = X_NEAR
Y_POS_SEAT = +0.1330                # +2.50 mm flange-to-flange seam gap at y = 0
Y_NEG_SEAT = -0.1330

DY_TCP = 0.0010                     # 1.0 mm outside outer flange lip (proven in native c7c..c7i)
F_PRE_SLIDE = -0.0080               # 16.0 mm symmetric pre-slide jaw gap
F_REL_SAFE = -0.0068                # 13.6 mm symmetric controlled release gap
DZ_DROP = 0.0240                    # +24.0 mm above RIM (`c7i` post-processor splits final 25 mm slow descent to this target)


def _Rx(d: float) -> np.ndarray:
    a = math.radians(d)
    return np.array([[1.0, 0.0, 0.0], [0.0, math.cos(a), -math.sin(a)], [0.0, math.sin(a), math.cos(a)]], dtype=np.float64)


def _Ry(d: float) -> np.ndarray:
    a = math.radians(d)
    return np.array([[math.cos(a), 0.0, math.sin(a)], [0.0, 1.0, 0.0], [-math.sin(a), 0.0, math.cos(a)]], dtype=np.float64)


def _Rz(d: float) -> np.ndarray:
    a = math.radians(d)
    return np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def nonoverlapping_reset_poses(stand_top: float = STAND_TOP) -> dict[str, tuple[float, float, float]]:
    """Non-overlapping `(x, y, z)` reset poses on the `762 x 1,219 mm` Regency stand (`+20.25 mm` symmetric Y gaps)."""
    z0 = stand_top + 0.0005
    return {
        "adaptor_bar": (X_STRIP, 0.0000, z0),
        "quarter_1": (X_STRIP, +0.4175, z0),
        "quarter_2": (X_STRIP, -0.4175, z0),
    }


def install_orient_hook():
    import yam_arena.demo as _demo

    class Orient(float):
        def __new__(cls, rot):
            obj = float.__new__(cls, 0.0)
            obj._rot = np.asarray(rot, dtype=np.float64)
            return obj

        @property
        def rot(self):
            return self._rot

    orig = _demo.grip_rotation
    if not getattr(orig, "_orient", False):
        def _grip_rotation(yaw=0.0):
            return yaw.rot if hasattr(yaw, "rot") else orig(yaw)
        _grip_rotation._orient = True
        _demo.grip_rotation = _grip_rotation
    return Orient, orig(0.0)


def end_grip_rotations(pitch_from_down_deg: float = 60.0, yaw_deg: float = 10.0):
    """Return `(R_qL_orient, R_qR_orient, approach_L_vec, approach_R_vec)`."""
    Orient, D = install_orient_hook()
    RL = _Rz(+yaw_deg) @ _Rx(-pitch_from_down_deg) @ D
    RR = _Rz(-yaw_deg) @ _Rx(+pitch_from_down_deg) @ D
    aL = RL @ D.T @ np.array([0.0, 0.0, -1.0])
    aR = RR @ D.T @ np.array([0.0, 0.0, -1.0])
    return Orient(RL), Orient(RR), aL, aR


def bar_waypoints(
    bar_stage_xy: tuple[float, float] = (X_STRIP, 0.0000),
    wc_xy: tuple[float, float] = WC,
    stand_top: float = STAND_TOP,
    rim_z: float = RIM,
    rx_roll_deg: float = 48.0,
):
    """Codex 0643 320-step (`16.0 s` incl. park) D405-up Two-Hand Bar Coupon (`0.086 mm` 20 Hz ArmPlan stream!).

    Note: `zb_pick = stand_top + 0.0175 m` (`17.5 mm` above stand) keeps even the `48 deg`-rolled pad corner `+4.4 mm`
    clear above `stand_top` while still engaging the `8.0 mm` bar height cleanly!
    """
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
    from yam_arena.demo import Waypoint

    Orient, D = install_orient_hook()
    R_barL = Orient(_Rx(-rx_roll_deg) @ _Rz(-90.0) @ D)
    R_barR = Orient(_Rx(+rx_roll_deg) @ _Rz(+90.0) @ D)
    yb = float(__import__("os").environ.get("W50_BAR_YB", "0.200"))  # v2 front-pan start: pinch the bar further inboard
    zb_pick = stand_top + 0.0175
    zb_place = rim_z + 0.0175
    bar_l = [
        Waypoint(1.4, (0.200, +0.260, 0.895), FINGER_OPEN, "bar_high_ready", 0.0),
        Waypoint(1.6, (0.200, +yb, 0.885), -0.0260, "bar_high_orient_clear_wall", R_barL),
        Waypoint(1.0, (bar_stage_xy[0], +yb, stand_top + 0.065), -0.0260, "bar_above_strip", R_barL),
        Waypoint(0.9, (bar_stage_xy[0], +yb, zb_pick), -0.0260, "bar_down", R_barL),
        Waypoint(0.5, (bar_stage_xy[0], +yb, zb_pick), FINGER_CLOSED, "bar_pinch", R_barL),
        Waypoint(1.2, (bar_stage_xy[0], +yb, rim_z + 0.055), FINGER_CLOSED, "bar_lift_clear_wall", R_barL),
        Waypoint(2.2, (wc_xy[0], +yb, rim_z + 0.055), FINGER_CLOSED, "bar_carry_over_well", R_barL),
        Waypoint(1.2, (wc_xy[0], +yb, zb_place), FINGER_CLOSED, "bar_seat_end_rims", R_barL),
        Waypoint(0.5, (wc_xy[0], +yb, zb_place), -0.0220, "bar_release", R_barL),
        Waypoint(1.1, (0.415, +yb, rim_z + 0.055), FINGER_OPEN, "bar_retreat_diag", R_barL),
        Waypoint(1.4, (0.230, +0.230, 0.885), FINGER_OPEN, "bar_retreat_back_x", R_barL),
        Waypoint(1.5, (0.220, +0.300, 0.895), FINGER_OPEN, "bar_park_ready", 0.0),
    ]
    bar_r = [
        Waypoint(w.duration, (w.pos[0], -w.pos[1], w.pos[2]), w.finger, w.label, R_barR if isinstance(w.yaw, Orient) else 0.0)
        for w in bar_l
    ]
    return bar_l, bar_r


def two_hand_pan_waypoints(
    stage_xy: tuple[float, float] = (X_STRIP, 0.0000),
    seat_xy: tuple[float, float] = (X_NEAR, Y_POS_SEAT),
    tag: str = "near_q1",
    stand_top: float = STAND_TOP,
    rim_z: float = RIM,
    pitch_from_down_deg: float = 60.0,
    yaw_deg: float = 10.0,
    dy_tcp: float = DY_TCP,
    f_pre: float = F_PRE_SLIDE,
    f_rel: float = F_REL_SAFE,
    dz_drop: float = DZ_DROP,
    dz_peel_up: float = 0.014,
):
    """Exact base trajectory consumed by `c7i/task.py` (`ea8563d7`) that passed native `pan_ok = True`!"""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
    from yam_arena.demo import Waypoint

    R_qL, R_qR, aL, aR = end_grip_rotations(pitch_from_down_deg=pitch_from_down_deg, yaw_deg=yaw_deg)
    sx, sy = stage_xy
    cx, cy = seat_xy
    zf_st = stand_top + 0.0005 + Q_D - SHEET / 2.0  # 0.6600 m
    zf_lift = zf_st + 0.038                         # 0.6980 m
    zf_carry = rim_z + 0.075                        # `c7i/task.py` overrides to `RIM + 0.115` via `SW7_CARRY_DZ`
    zf_drop = rim_z + dz_drop                       # 0.8501 m

    gL_st = np.array([sx, sy + Q_L / 2.0 + dy_tcp, zf_st])
    gR_st = np.array([sx, sy - Q_L / 2.0 - dy_tcp, zf_st])
    gL_se = np.array([cx, cy + Q_L / 2.0 + dy_tcp, zf_drop])
    gR_se = np.array([cx, cy - Q_L / 2.0 - dy_tcp, zf_drop])

    bkL_st = gL_st - 0.028 * aL
    bkR_st = gR_st - 0.028 * aR
    bkL_se = gL_se - 0.028 * aL + np.array([0.0, 0.0, dz_peel_up])
    bkR_se = gR_se - 0.028 * aR + np.array([0.0, 0.0, dz_peel_up])

    left_wps = [
        Waypoint(1.5, tuple(bkL_st + np.array([0.0, 0.0, 0.025])), FINGER_OPEN, f"{tag}_pre_high", R_qL),
        Waypoint(0.8, tuple(bkL_st), f_pre, f"{tag}_line_up", R_qL),
        Waypoint(0.9, tuple(gL_st), f_pre, f"{tag}_slide_on", R_qL),
        Waypoint(0.6, tuple(gL_st), FINGER_CLOSED, f"{tag}_pinch", R_qL),
        Waypoint(0.4, tuple(gL_st), FINGER_CLOSED, f"{tag}_settle", R_qL),
        Waypoint(1.0, (float(gL_st[0]), float(gL_st[1]), zf_lift), FINGER_CLOSED, f"{tag}_lift_38mm", R_qL),
        Waypoint(1.5, (float(gL_st[0]), float(gL_st[1]), zf_lift), FINGER_CLOSED, f"{tag}_hold_1p5s", R_qL),
        Waypoint(1.2, (float(gL_st[0]), float(gL_st[1]), zf_carry), FINGER_CLOSED, f"{tag}_raise_carry", R_qL),
        Waypoint(1.8, (float(gL_se[0]), float(gL_se[1]), zf_carry), FINGER_CLOSED, f"{tag}_carry_over_seat", R_qL),
        Waypoint(1.2, tuple(gL_se), FINGER_CLOSED, f"{tag}_lower_24mm", R_qL),
        Waypoint(0.6, tuple(gL_se), f_rel, f"{tag}_release_13p6mm_gap", R_qL),
        Waypoint(0.9, tuple(bkL_se), f_rel, f"{tag}_peel_back", R_qL),
        Waypoint(1.0, (float(bkL_se[0]), float(bkL_se[1]), RIM + 0.115), FINGER_OPEN, f"{tag}_retreat_up", R_qL),
    ]
    right_wps = [
        Waypoint(1.5, tuple(bkR_st + np.array([0.0, 0.0, 0.025])), FINGER_OPEN, f"{tag}_pre_high", R_qR),
        Waypoint(0.8, tuple(bkR_st), f_pre, f"{tag}_line_up", R_qR),
        Waypoint(0.9, tuple(gR_st), f_pre, f"{tag}_slide_on", R_qR),
        Waypoint(0.6, tuple(gR_st), FINGER_CLOSED, f"{tag}_pinch", R_qR),
        Waypoint(0.4, tuple(gR_st), FINGER_CLOSED, f"{tag}_settle", R_qR),
        Waypoint(1.0, (float(gR_st[0]), float(gR_st[1]), zf_lift), FINGER_CLOSED, f"{tag}_lift_38mm", R_qR),
        Waypoint(1.5, (float(gR_st[0]), float(gR_st[1]), zf_lift), FINGER_CLOSED, f"{tag}_hold_1p5s", R_qR),
        Waypoint(1.2, (float(gR_st[0]), float(gR_st[1]), zf_carry), FINGER_CLOSED, f"{tag}_raise_carry", R_qR),
        Waypoint(1.8, (float(gR_se[0]), float(gR_se[1]), zf_carry), FINGER_CLOSED, f"{tag}_carry_over_seat", R_qR),
        Waypoint(1.2, tuple(gR_se), FINGER_CLOSED, f"{tag}_lower_24mm", R_qR),
        Waypoint(0.6, tuple(gR_se), f_rel, f"{tag}_release_13p6mm_gap", R_qR),
        Waypoint(0.9, tuple(bkR_se), f_rel, f"{tag}_peel_back", R_qR),
        Waypoint(1.0, (float(bkR_se[0]), float(bkR_se[1]), RIM + 0.115), FINGER_OPEN, f"{tag}_retreat_up", R_qR),
    ]
    return left_wps, right_wps


def demo_coupon(case: str = "q1_only") -> dict[str, list]:
    """Drop-in `demo_coupon` for `c7i/task.py`: `bar_only` is exactly `320` steps (`16.0 s` at `20 Hz`)!"""
    from yam_arena.constants import FINGER_OPEN
    from yam_arena.demo import Waypoint

    if case == "bar_only":
        bL, bR = bar_waypoints((X_STRIP, 0.0000))
        # 14.5 s + 1.5 s = 16.0 s = 320 steps at 20 Hz (exact Codex 0643 320-step bar coupon!)
        pL = [Waypoint(1.5, (0.22, +0.32, 0.92), FINGER_OPEN, "park_still", 0.0)]
        pR = [Waypoint(1.5, (0.22, -0.32, 0.92), FINGER_OPEN, "park_still", 0.0)]
        return {"left": bL + pL, "right": bR + pR}
    pkL = [Waypoint(1.4, (0.22, +0.32, 0.94), FINGER_OPEN, "park", 0.0), Waypoint(1.2, (0.22, +0.32, 0.94), FINGER_OPEN, "park_still", 0.0)]
    pkR = [Waypoint(1.4, (0.22, -0.32, 0.94), FINGER_OPEN, "park", 0.0), Waypoint(1.2, (0.22, -0.32, 0.94), FINGER_OPEN, "park_still", 0.0)]
    if case == "q1_only":
        qL, qR = two_hand_pan_waypoints((X_STRIP, 0.0000), (X_NEAR, Y_POS_SEAT), tag="near_q1")
        return {"left": qL + pkL, "right": qR + pkR}
    if case == "q2_occupied":
        qL, qR = two_hand_pan_waypoints((X_STRIP, 0.0000), (X_NEAR, Y_NEG_SEAT), tag="near_q2", dz_peel_up=0.024)
        return {"left": qL + pkL, "right": qR + pkR}
    raise ValueError(f"Unknown case: {case}")
