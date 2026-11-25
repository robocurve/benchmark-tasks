"""Seat the adaptor bar across the steam well with both hands, then hang one quarter pan in the far channel on the bar and the far rim.

On the station floor (z = -0.0125 m) sits a fixed stainless equipment stand (762.0 x 1,219.0 mm top at z = 0.5975 m,
front edge x = 0.1300 m) carrying a fixed, unplugged, cold, dry, empty 12 x 20 in stainless countertop food warmer
centred at (0.4800, 0.0000) m (368.3 x 571.5 x 228.6 mm outside; 304.8 x 508.0 x 158.75 mm inner well; 31.75 mm flat top
rim at z = 0.8261 m). A loose stainless adaptor bar (25.4 x 531.0 x 8.0 mm, 0.136 kg) lies flat on the stand top at
(0.2130, 0.0000) m along Y. One stainless 1/4-size hotel pan (161.8 x 263.5 mm top, 63.5 mm deep, 15 mm flanges, 0.453 kg)
sits upright on the stand top at (0.2130, +0.4675) m. The pan's 161.8 mm side cannot span the 304.8 mm opening: it hangs
only from the bar on one long flange and the far rim on the other.

Demo: bar two-hand transfer, then the pan is slid to the front strip, squared against a straightedge push, picked with
both hands and seated in the far channel, then both hands park. Closed-loop guards (pre-push bounds, engaged trim,
post-square-up Y trim, pick gate) act on the pan only; out-of-bounds readings stop and park both arms.
Control run: put an empty file named NO_BAR next to this file (or set SWBP_NO_BAR=1) to remove the bar from the scene.
"""
from __future__ import annotations

from collections import deque
import json
import math
import os
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

os.environ.setdefault("SW_WCX", "0.48")
import labkit  # noqa: E402
labkit.floor_bay()
from labkit import on_reset, on_step  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402

import w50_bar1pan_scene as S  # noqa: E402
import w50_bar_pan_scorer as SC  # noqa: E402

labkit._RESET.clear()
labkit._STEP.clear()

NO_BAR = os.environ.get("SWBP_NO_BAR") == "1" or (HERE / "NO_BAR").exists()

SCENE = S.SCENE
STAND, WARMER, BAR, Q1 = S.STAND, S.WARMER, S.BAR, S.Q1
OBJECTS = [STAND, WARMER, Q1] if NO_BAR else [STAND, WARMER, BAR, Q1]
WC, RIM, ST = S.WC, S.RIM, S.ST

# square-up (straightedge push) constants, as in the proven 4-pan runner
PITCH_WIDE_DEG = -60.0
YAW_ZERO_WIDE_DEG = -0.75
DX_TIP_WIDE = 0.01383
PD_PUSH_PRELOAD_X = 0.00380
Z_TCP_WIDE = ST + 0.0260
WALL_INNER_HALF_Y = (S.Q_L - 2.0 * S.Q_FL) / 2.0 - S.SHEET

FAR_CARRY_RY_DEG = -12.0
FAR_SEAT_RY_DEG = -10.5
FAR_CARRY_Z = RIM + 0.115
FAR_X_PRELOAD = 0.0060

PARK_TAIL_S = 1.50
# Seat the pan centred along the well (y = 0): its short-end flanges then stay 122 mm clear of the end rims, so only the
# bar and the far long rim can carry it. (At y = +0.133 m its end flange also reached the end rim and it hung level in
# the corner without the bar.)
SEAT_Y = float(os.environ.get("SWBP_SEAT_Y", "0.0"))

_PLAN: dict[str, list[Waypoint]] = {}
_ST: dict[str, object] = {}


# ----------------------------------------------------------------------------------------------- plan builders
def _square_up_and_post_trim(slide_side: str = "left") -> tuple[list[Waypoint], list[Waypoint]]:
    """6.15 s: 0.20 s pre-push guard slot, 4.15 s wide straightedge push by the other arm, 1.80 s post Y-trim slot."""
    import w50_two_hand_near_waypoints as HW
    import w50_squareup_and_crossing_witness as SQW

    Orient, D_DOWN = HW.install_orient_hook()
    R_sq0 = Orient(SQW._Ry_deg(PITCH_WIDE_DEG) @ SQW._Rz_deg(YAW_ZERO_WIDE_DEG) @ D_DOWN)
    sgn_slide = +1.0 if slide_side == "left" else -1.0
    sq_arm = "right" if slide_side == "left" else "left"
    park_sq_y = -0.300 if sq_arm == "right" else +0.300

    pre_s = [Waypoint(0.20, (0.215, sgn_slide * 0.290, 0.880), FINGER_OPEN, "yguard1_slot_0", 0.0)]
    pre_q = [Waypoint(0.20, (0.220, park_sq_y, 0.895), FINGER_OPEN, "wait_yguard1_slot", 0.0)]

    x_push_1 = (S.X_STRIP - S.BODY_HALF_X) - DX_TIP_WIDE + PD_PUSH_PRELOAD_X
    x_start, x_ret, z = 0.0800, 0.0880, Z_TCP_WIDE
    sq_q = [
        Waypoint(0.35, (0.1100, 0.0, 0.7800), FINGER_OPEN, "p1_sqA_above_pitched", R_sq0),
        Waypoint(0.35, (x_start, 0.0, 0.7200), FINGER_OPEN, "p1_sqA_pre_descent", R_sq0),
        Waypoint(0.40, (x_start, 0.0, z), FINGER_OPEN, "p1_sqA_down_clear", R_sq0),
        Waypoint(1.35, (x_push_1, 0.0, z), FINGER_OPEN, "p1_sqA_push_plus_x", R_sq0),
        Waypoint(0.50, (x_push_1, 0.0, z), FINGER_OPEN, "p1_sqA_hold_wall", R_sq0),
        Waypoint(0.20, (x_push_1, 0.0, z), FINGER_OPEN, "p1_sqA_hold_wall_settle", R_sq0),
        Waypoint(0.30, (x_ret, 0.0, z), FINGER_OPEN, "p1_sqA_retract_before_lift", R_sq0),
        Waypoint(0.35, (x_ret, 0.0, 0.7800), FINGER_OPEN, "p1_sqA_rise_clear", R_sq0),
        Waypoint(0.35, (0.2200, park_sq_y, 0.8950), FINGER_OPEN, "p1_sqA_return_ready", 0.0),
    ]
    sq_s = [Waypoint(4.15, (0.2200, sgn_slide * 0.300, 0.8950), FINGER_OPEN, "wait_p1_sqA", 0.0)]

    post_durs = (0.25, 0.25, 0.45, 0.25, 0.20, 0.20, 0.20)
    post_s = [Waypoint(d, (0.2200, sgn_slide * 0.300, 0.8950), FINGER_OPEN, f"yguard1_post_{i}", 0.0) for i, d in enumerate(post_durs)]
    post_q = [Waypoint(round(sum(post_durs), 4), (0.2200, park_sq_y, 0.8950), FINGER_OPEN, "wait_yguard1_post", 0.0)]

    seq_slide, seq_sq = pre_s + sq_s + post_s, pre_q + sq_q + post_q
    return (seq_slide, seq_sq) if slide_side == "left" else (seq_sq, seq_slide)


def _far_seat(seat_xy: tuple[float, float], tag: str = "far_q1", dy_unhook: float = 0.0250,
              f_rel: float = -0.0068) -> tuple[list[Waypoint], list[Waypoint]]:
    """13.70 s two-hand flange pinch at the front strip, arched carry over the bar, slow lower onto bar + far rim,
    controlled release, outward peel, retreat."""
    import w50_two_hand_near_waypoints as HW
    import w50_occupied_q2_and_far_isolated_audit as OQA

    Orient, D_DOWN = HW.install_orient_hook()
    yaw_a, pitch_a = float(os.environ.get("W50_YAW_A", "10.0")), float(os.environ.get("W50_PITCH_A", "60.0"))
    R_carry_tilt = OQA._Ry_deg(FAR_CARRY_RY_DEG)
    R_seat_tilt = OQA._Ry_deg(FAR_SEAT_RY_DEG)
    z_carry = FAR_CARRY_Z
    z_seat_tcp = RIM + 0.0300 + float(os.environ.get("W50_SEAT_DZ", "-0.061"))  # seat-only z correction for a changed hand orientation
    x_far_cmd = float(seat_xy[0]) + FAR_X_PRELOAD

    RL0 = OQA._Rz_deg(+yaw_a) @ OQA._Rx_deg(-pitch_a) @ D_DOWN
    RR0 = OQA._Rz_deg(-yaw_a) @ OQA._Rx_deg(+pitch_a) @ D_DOWN
    aL0 = RL0 @ D_DOWN.T @ np.array([0.0, 0.0, -1.0])
    aR0 = RR0 @ D_DOWN.T @ np.array([0.0, 0.0, -1.0])

    zf = ST + 0.0005 + S.Q_D - S.SHEET / 2.0
    bite = 0.0085
    # v2: grip line moved +GRIP_DX in x so it passes over the pan's centre of mass (v1 hung 13.7 deg about the grip line);
    # the seat target moves by the same amount so the final pan pose is unchanged
    gdx = float(os.environ.get("W50_GRIP_DX", "0.0"))
    gL = np.array([S.X_STRIP + gdx, +S.Q_L / 2.0 - bite, zf])
    gR = np.array([S.X_STRIP + gdx, -S.Q_L / 2.0 + bite, zf])
    sdx = float(os.environ.get("W50_SEAT_DX", "-0.005"))  # seat-only x correction for a changed hand orientation
    sL = np.array([x_far_cmd + gdx + sdx, float(seat_xy[1]) + S.Q_L / 2.0 - bite, z_seat_tcp])
    sR = np.array([x_far_cmd + gdx + sdx, float(seat_xy[1]) - S.Q_L / 2.0 + bite, z_seat_tcp])
    f_pre = -0.0080

    y_start = float(S.PAN_Y0) if S.PAN_FRONT else 0.0  # front start: pinch the pan where it stands, then shift it to the centre

    def _side(g, s, a0, R0, sgn_y, park_y):
        O0, Oc, Os = Orient(R0), Orient(R_carry_tilt @ R0), Orient(R_seat_tilt @ R0)
        y_peel = round(float(s[1]) + sgn_y * dy_unhook, 6)
        g0 = g + np.array([0.0, y_start, 0.0])
        shift = ([Waypoint(1.10, tuple(g + np.array([0.0, 0.0, 0.035])), FINGER_CLOSED, f"{tag}_shift_to_centre", O0)]
                 if y_start else [])
        return [
            Waypoint(0.65, tuple(g0 - 0.028 * a0 + np.array([0.0, 0.0, 0.115])), FINGER_OPEN, f"{tag}_pre_high", O0),
            Waypoint(0.55, tuple(g0 - 0.028 * a0), f_pre, f"{tag}_pre_line_up", O0),
            Waypoint(0.55, tuple(g0), f_pre, f"{tag}_slide_on", O0),
            Waypoint(0.40, tuple(g0), FINGER_CLOSED, f"{tag}_pinch", O0),
            Waypoint(0.65, tuple(g0 + np.array([0.0, 0.0, 0.035])), FINGER_CLOSED, f"{tag}_lift_35mm", O0),
            *shift,
            Waypoint(0.45, tuple(g + np.array([0.0, 0.0, 0.035])), FINGER_CLOSED, f"{tag}_hold_1p5s", Oc),
            Waypoint(1.10, (float(g[0]), float(g[1]), z_carry), FINGER_CLOSED, f"{tag}_raise_carry", Oc),
            Waypoint(1.30, (0.4850, float(s[1]), z_carry), FINGER_CLOSED, f"{tag}_cross_front_rim", Oc),
            Waypoint(1.60, (float(s[0]) - 0.0035, float(s[1]), z_carry), FINGER_CLOSED, f"{tag}_carry_over_seat", Oc),
            Waypoint(0.55, (float(s[0]) - 0.0035, float(s[1]), z_carry), FINGER_CLOSED, f"{tag}_pitch_to_seat", Os),
            Waypoint(1.65, (float(s[0]) + 0.0027, float(s[1]), RIM + 0.0650), FINGER_CLOSED, f"{tag}_lower_to_slow", Os),
            Waypoint(1.85, (float(s[0]), float(s[1]), float(s[2])), FINGER_CLOSED, f"{tag}_lower_slow_final", Os),
            Waypoint(0.40, (float(s[0]), float(s[1]), float(s[2])), float(f_rel), f"{tag}_release_controlled", Os),
            Waypoint(0.60, (float(s[0]), y_peel, float(s[2])), float(f_rel), f"{tag}_unhook_outward_peel", Os),
            *([Waypoint(0.35, (float(s[0]), y_peel, 0.9120), float(f_rel), f"{tag}_rise_straight_up", Os),
               Waypoint(0.30, (0.5000, y_peel, 0.9120), float(f_rel), f"{tag}_rise_above_neighbors", Oc)]
              if os.environ.get("W50_RISE_VERTICAL", "1") == "1" else
              [Waypoint(0.65, (0.5000, y_peel, 0.9120), float(f_rel), f"{tag}_rise_above_neighbors", Oc)]),
            Waypoint(0.75, (0.2200, park_y, 0.9300), FINGER_OPEN, f"{tag}_retreat_park", 0.0),
        ]

    return _side(gL, sL, aL0, RL0, +1.0, +0.320), _side(gR, sR, aR0, RR0, -1.0, -0.320)


def build_plan(y_stage_abs: float = S.Y_STAGE_ABS, y_trim_abs: float = 0.00435) -> dict[str, list[Waypoint]]:
    import w50_bar_waypoints as BW
    import w50_squareup_and_crossing_witness as SQW

    bL, bR = BW.bar_two_hand_d405_up()  # 16.00 s
    s1L_raw, _ = SQW.slide_int2_x0185_trimmed("left", y_stage_abs, -abs(y_trim_abs))
    s1L = [Waypoint(w.duration, w.pos, w.finger, f"p1_{w.label}", w.yaw) for w in s1L_raw]
    s1R = [Waypoint(sum(w.duration for w in s1L), (0.220, -0.300, 0.895), FINGER_OPEN, "wait_p1_slide", 0.0)]
    m1L, m1R = _square_up_and_post_trim("left")
    f1L, f1R = _far_seat((S.X_FAR_SEAT, SEAT_Y))
    kL = [Waypoint(PARK_TAIL_S, (0.220, +0.320, 0.920), FINGER_OPEN, "park_still", 0.0)]
    kR = [Waypoint(PARK_TAIL_S, (0.220, -0.320, 0.920), FINGER_OPEN, "park_still", 0.0)]
    if S.PAN_FRONT:  # no slide / square-up: the pan already stands on the front strip beside the bar
        return {"left": bL + f1L + kL, "right": bR + f1R + kR}
    return {"left": bL + s1L + m1L + f1L + kL, "right": bR + s1R + m1R + f1R + kR}


def planned_steps(plan: dict[str, list[Waypoint]]) -> int:
    return max(sum(max(1, int(round(w.duration * 20.0))) for w in plan[k]) for k in ("left", "right"))


# ----------------------------------------------------------------------------------------------- guards (pan 1 only)
def _safe_park(plan, prefix_L, prefix_R, tcp_L, f_L, tcp_R, f_R, stop_label) -> None:
    z_hi = max(0.9000, tcp_L[2], tcp_R[2])
    for arm, pfx, cur, fcur, park in (("left", prefix_L, tcp_L, f_L, (0.220, +0.320, 0.920)),
                                      ("right", prefix_R, tcp_R, f_R, (0.220, -0.320, 0.920))):
        x_safe = min(cur[0], 0.0880) if cur[2] < 0.760 else min(cur[0], 0.2200)
        seq = [((x_safe, cur[1], z_hi), fcur, "abort_lift_vertical"), (park, fcur, "abort_retreat_closed"),
               (park, FINGER_OPEN, "abort_open_at_park")]
        i0 = next((i for i, w in enumerate(plan[arm]) if w.label.startswith(pfx)), None)
        if i0 is None:
            continue
        for off, (pn, fn, ln) in enumerate(seq):
            if i0 + off < len(plan[arm]):
                w = plan[arm][i0 + off]
                w.pos, w.finger, w.yaw, w.label = pn, fn, 0.0, ln
        for j in range(i0 + len(seq), len(plan[arm])):
            w = plan[arm][j]
            w.pos, w.finger, w.yaw, w.label = park, FINGER_OPEN, 0.0, stop_label


def _yaw_tilt(Rm) -> tuple[float, float]:
    ax = Rm @ np.array([0.0, 1.0, 0.0])
    return (math.degrees(math.atan2(-float(ax[0]), float(ax[1]))),
            math.degrees(math.acos(float(np.clip(Rm[2, 2], -1.0, 1.0)))))


def guard_prepush(plan, p, Rm, tcps, fm) -> dict:
    yaw, tilt = _yaw_tilt(Rm)
    dy_mm = float(p[1] * 1e3)
    ok = (0.1800 <= float(p[0]) <= 0.2160) and abs(dy_mm) <= 25.0 and abs(yaw) <= 12.5 and tilt <= 3.0
    if not ok:
        _safe_park(plan, "yguard1_slot_", "wait_yguard1_slot", tcps["L"], fm["L"], tcps["R"], fm["R"], "stopped_by_prepush_guard")
    return {"ok": ok, "x": float(p[0]), "dy_mm": dy_mm, "yaw_deg": yaw, "tilt_deg": tilt}


def guard_engaged_trim(plan, p, Rm, d_face_mm, tcps, fm) -> dict:
    import w50_two_hand_near_waypoints as HW
    import w50_squareup_and_crossing_witness as SQW

    yaw, tilt = _yaw_tilt(Rm)
    if d_face_mm > 2.0 or tilt > 2.5:
        _safe_park(plan, "wait_p1_sqA", "p1_sqA_hold_wall", tcps["L"], fm["L"], tcps["R"], fm["R"], "stopped_by_blade_gate")
        return {"ok": False, "d_face_mm": d_face_mm, "yaw_deg": yaw, "tilt_deg": tilt}
    Orient, D_DOWN = HW.install_orient_hook()
    x_push_1 = (S.X_STRIP - S.BODY_HALF_X) - DX_TIP_WIDE + PD_PUSH_PRELOAD_X
    err_x = float(np.clip(S.X_STRIP - float(p[0]), -0.0040, +0.0080))
    err_yaw = float(np.clip(yaw, -6.0, +6.0))
    x_cmd = float(np.clip(x_push_1 + 1.05 * err_x, 0.1320, 0.1490))
    yaw_cmd = YAW_ZERO_WIDE_DEG - 1.15 * err_yaw
    R_cmd = Orient(SQW._Ry_deg(PITCH_WIDE_DEG) @ SQW._Rz_deg(yaw_cmd) @ D_DOWN)
    for w in plan["right"]:
        if w.label in ("p1_sqA_hold_wall", "p1_sqA_hold_wall_settle"):
            w.pos, w.finger, w.yaw = (x_cmd, 0.0, Z_TCP_WIDE), FINGER_OPEN, R_cmd
    return {"ok": True, "d_face_mm": d_face_mm, "x": float(p[0]), "yaw_deg": yaw, "x_cmd": x_cmd, "yaw_cmd_deg": yaw_cmd}


def guard_post_y_trim(plan, p, Rm) -> dict:
    yaw, _ = _yaw_tilt(Rm)
    dy = float(p[1])
    if abs(dy) <= 0.0015 or abs(dy) > 0.0250 or abs(yaw) > 4.0:
        return {"nudge": False, "dy_mm": dy * 1e3, "yaw_deg": yaw}
    sgn_push = -1.0 if dy > 0.0 else +1.0
    y_wall_now, y_wall_goal = dy + sgn_push * WALL_INNER_HALF_Y, sgn_push * WALL_INNER_HALF_Y
    fh, pre = 0.01005, 0.01585
    y_entry = y_wall_now - sgn_push * (fh + 0.0220)
    y_end = y_wall_goal - sgn_push * (fh - pre)
    y_unhook = y_wall_goal - sgn_push * (fh + 0.0200)
    z_hi, z_in = ST + S.Q_D + 0.085, ST + 0.028
    targets = [
        ((0.1850, y_entry, z_hi), FINGER_CLOSED, "yguard1_above_cavity"),
        ((0.1850, y_entry, z_in), FINGER_CLOSED, "yguard1_dip_cavity"),
        ((0.1850, y_end, z_in), FINGER_CLOSED, "yguard1_nudge_to_y0"),
        ((0.1850, y_end, z_in), FINGER_CLOSED, "yguard1_hold_y0"),
        ((0.1850, y_unhook, z_in), FINGER_CLOSED, "yguard1_unhook"),
        ((0.1850, y_unhook, z_hi), FINGER_CLOSED, "yguard1_lift_vertical_out"),
        ((0.2150, 0.2900, 0.8850), FINGER_OPEN, "yguard1_lift_clear"),
    ]
    slot = [w for w in plan["left"] if w.label.startswith("yguard1_post_")]
    for w, (pn, fn, ln) in zip(slot, targets):
        w.pos, w.finger, w.yaw, w.label = tuple(float(v) for v in pn), fn, 0.0, ln
    return {"nudge": True, "dy_mm": dy * 1e3, "yaw_deg": yaw}


def guard_pick_gate(plan, p, Rm, tcps, fm) -> dict:
    yaw, tilt = _yaw_tilt(Rm)
    dx_mm, dy_mm = float((p[0] - S.X_STRIP) * 1e3), float(p[1] * 1e3)
    ok = math.hypot(dx_mm, dy_mm) <= 2.0 and abs(yaw) <= 2.0 and tilt <= 2.0
    retarget = False
    if not ok and math.hypot(dx_mm, dy_mm) <= PICK_RETARGET_MM and abs(yaw) <= 3.0 and tilt <= 2.0:
        # v2 (food): the loaded pan stops a few mm off the strip centre; aim the pinch at the pan's MEASURED pose instead of
        # aborting (camera-visible). Only the pick-phase waypoints move; the carry keeps the nominal hand-to-pan offset.
        for side in ("left", "right"):
            for w in plan.get(side, []):
                if w.label in _PICK_LABELS:
                    w.pos = (float(w.pos[0]) + dx_mm / 1e3, float(w.pos[1]) + dy_mm / 1e3, float(w.pos[2]))
        ok, retarget = True, True
    if not ok:
        _safe_park(plan, "far_q1_", "far_q1_", tcps["L"], fm["L"], tcps["R"], fm["R"], "stopped_by_pick_gate")
    return {"ok": ok, "retargeted": retarget, "dx_mm": dx_mm, "dy_mm": dy_mm, "yaw_deg": yaw, "tilt_deg": tilt}


PICK_RETARGET_MM = float(os.environ.get("W50_PICK_RETARGET_MM", "25.0"))
_PICK_LABELS = {f"far_q1_{k}" for k in ("pre_high", "pre_line_up", "slide_on", "pinch", "lift_35mm", "hold_1p5s", "raise_carry")}


def _sq_arm_face_dist_mm(lposes_arm, p, Rm, clouds) -> float:
    import w50_multi_pan_isolated_runner as MPR

    best = 999.0
    for oc, oR, osz in MPR.pan_obbs_world(p, Rm).values():
        for dl in ("tip_left", "tip_right"):
            if dl not in lposes_arm:
                continue
            Td = lposes_arm[dl]
            for zone in ("pad", "blade"):
                zl = clouds.get(f"{dl}_{zone}")
                if zl is None or len(zl) == 0:
                    continue
                best = min(best, float(MPR.signed_obb_mm((Td[:3, :3] @ zl.T).T + Td[:3, 3], oc, oR, osz)))
    return best


def _step_before(wps, label, lead=2) -> int:
    acc = 0
    for w in wps:
        if w.label == label:
            return max(1, acc + 1 - lead)
        acc += max(1, int(round(float(w.duration) * 20.0)))
    return -1


def _label_at(wps, n) -> str:
    rem = int(n)
    for w in wps:
        t = int(round(float(w.duration) * 20.0))
        if rem < t:
            return w.label
        rem -= t
    return wps[-1].label if wps else "done"


# ----------------------------------------------------------------------------------------------- task hooks
def demo() -> dict[str, list[Waypoint]]:
    plan = build_plan()
    _PLAN.clear()
    _PLAN.update(plan)
    return plan


@on_reset
def _reset(env) -> None:
    import w50_multi_pan_isolated_runner as MPR

    tr = _ST.get("tracker")
    if tr is None:
        tr = SC.BarPanTracker(wc_xy=WC, rim_z=RIM, stand_top=ST)
    else:
        tr.reset()
    clouds = _ST.get("clouds")
    if clouds is None:
        try:
            clouds = MPR.load_urdf_link_clouds_portable()
        except Exception as exc:  # guard falls back to "no slip measured"
            print(f"[bar1pan] link clouds unavailable: {exc!r}", flush=True)
            clouds = {}
    L, R = _PLAN.get("left", []), _PLAN.get("right", [])
    _ST.clear()
    _ST.update(
        tracker=tr, clouds=clouds, n=0, latest_score=0.0, aborted=False,
        gate_prepush=_step_before(L, "yguard1_slot_0"),
        gate_trim=_step_before(R, "p1_sqA_hold_wall"),
        gate_posty=_step_before(L, "yguard1_post_0"),
        gate_pick=-1 if S.PAN_FRONT else _step_before(L, "far_q1_pre_high", 3),
        planned=planned_steps(_PLAN) if _PLAN else 0,
    )
    print("[bar1pan_reset] " + json.dumps({k: _ST[k] for k in ("gate_prepush", "gate_trim", "gate_posty", "gate_pick", "planned")}
                                          | {"no_bar": NO_BAR}), flush=True)


@on_step
def _step(env) -> None:
    from yam_arena import runtime as _rt
    from hand_cover_data import TIP_LEFT_COVER_NATIVE, TIP_RIGHT_COVER_NATIVE

    _ST["n"] = int(_ST.get("n", 0)) + 1
    n = int(_ST["n"])
    b = env.unwrapped
    origin = labkit._np(b.scene.env_origins)[0]

    lposes, tcps, fm, q78, pads = {}, {}, {}, {}, {}
    for k, rk in (("L", "left_robot"), ("R", "right_robot")):
        rob = b.scene[rk]
        env_lp = {}
        for lnk, Tw in _rt.link_poses(rob).items():
            Te = np.asarray(Tw, float).copy()
            Te[:3, 3] -= origin
            env_lp[lnk] = Te
        lposes[k] = env_lp
        Tg = env_lp["gripper"]
        tcps[k] = tuple(float(v) for v in (Tg[:3, :3] @ np.asarray(TCP_OFFSET, float) + Tg[:3, 3]))
        jids = rob.find_joints(["joint7", "joint8"], preserve_order=True)[0]
        jv = labkit._np(rob.data.joint_pos)[0, jids]
        fm[k] = float(np.mean(jv))
        q78[k] = [float(jv[0] * 1e3), float(jv[1] * 1e3)]
        pads[k] = np.vstack([(env_lp[l][:3, :3] @ np.asarray(c, float).T).T + env_lp[l][:3, 3]
                             for l, c in (("tip_left", TIP_LEFT_COVER_NATIVE), ("tip_right", TIP_RIGHT_COVER_NATIVE))])

    ps = labkit._part_state(env, Q1)
    pp, pR = np.asarray(ps.pos, float), np.asarray(ps.rotation(), float)
    bar_pose = None
    if not NO_BAR:
        bs = labkit._part_state(env, BAR)
        bar_pose = (np.asarray(bs.pos, float), np.asarray(bs.rotation(), float))
    fixtures = {}
    for fobj, fk in ((STAND, "stand"), (WARMER, "warmer")):
        try:
            fs = labkit._part_state(env, fobj)
            fixtures[fk] = (np.asarray(fs.pos, float), np.asarray(fs.rotation(), float))
        except Exception:
            pass

    if _PLAN and not _ST["aborted"]:
        if n == _ST["gate_prepush"]:
            d = guard_prepush(_PLAN, pp, pR, tcps, fm)
            print(f"[bar1pan_guard_prepush] step={n} {json.dumps(d)}", flush=True)
            _ST["aborted"] = not d["ok"]
        elif n == _ST["gate_trim"]:
            dface = _sq_arm_face_dist_mm(lposes["R"], pp, pR, _ST["clouds"]) if _ST["clouds"] else 0.0
            d = guard_engaged_trim(_PLAN, pp, pR, dface, tcps, fm)
            print(f"[bar1pan_guard_trim] step={n} {json.dumps(d)}", flush=True)
            _ST["aborted"] = not d["ok"]
        elif n == _ST["gate_posty"]:
            d = guard_post_y_trim(_PLAN, pp, pR)
            print(f"[bar1pan_guard_posty] step={n} {json.dumps(d)}", flush=True)
        elif n == _ST["gate_pick"]:
            d = guard_pick_gate(_PLAN, pp, pR, tcps, fm)
            print(f"[bar1pan_guard_pick] step={n} {json.dumps(d)}", flush=True)
            _ST["aborted"] = not d["ok"]

    rec = _ST["tracker"].record(bar_pose, (pp, pR), pads["L"], pads["R"], q78_mm=q78, fixtures=fixtures)
    _ST["latest_score"] = float(rec["score"])

    planned = int(_ST.get("planned", 0))
    if n == 1 or n % 5 == 0 or n >= planned - 25:
        row = {"step": n, "phase": [_label_at(_PLAN.get("left", []), n), _label_at(_PLAN.get("right", []), n)],
               "pan_pos": [round(float(v), 5) for v in pp],
               "bar_pos": None if bar_pose is None else [round(float(v), 5) for v in bar_pose[0]],
               "q78": {k: [round(v, 2) for v in q78[k]] for k in q78}}
        row.update(rec)
        print("[bar1pan_tick] " + json.dumps(row), flush=True)


def score(objects) -> float:
    return float(_ST.get("latest_score", 0.0))


def success(objects) -> bool:
    return float(_ST.get("latest_score", 0.0)) >= 1.0 - 1e-6
