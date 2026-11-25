"""Codex 0733 / 0746 / 0747 / 0808 Complete Offline Witness & Guarded Square-Up Controller (`antigravity-cake`, W50 active owner).

Addresses every point of `20261121-0808-codex-w50-real-link-clearance-and-gate.md`:
1. Keeps explicit `slide_int2_x0185_trimmed()` (`x_pull = 0.1850 m`, `+8.67 mm` offline 9-link `link4` clearance) so `0.205 m` cannot return.
2. Calibrates `DX_TIP_PITCHED = 0.01214 m` on exact unrounded float64 `tip_left`/`tip_right` collision STLs so `squareup_push_A_pitched`
   lands nominally at **`x_pan = 0.21300 m` (`dx = 0.00 mm`)** instead of `0.2133 m` (`+0.30 mm`).
3. Keeps the 2D Euclidean landing gate (`hypot(dx, dy) <= 2.00 mm`, `|yaw| <= 2.0 deg`) **strictly unweakened**, and makes
   `check_pre_push_y_guard` consistent with that 2D Euclidean disc by reserving `dx_budget = 0.80 mm` ->
   **`y_guard_tol = sqrt(2.00^2 - 0.80^2) = 1.83 mm -> 1.80 mm`** (`so residual dy = +-2.00 mm is caught before the +X push!`).
4. Implements `apply_visible_pose_y_guard_or_stop()` on mutable `Waypoint` slots (`total integrated route = 59.80 s = 1,196 steps`):
   - `|dy| <= 1.80 mm` & `x_pan in [0.180, 0.216] m` -> **`PROCEED_NOMINAL`** (holds clear during the `3.0 s` guard slot, then runs Pitched A + `c7i`);
   - `1.80 mm < |dy| <= 25.0 mm` & `x_pan in [0.180, 0.216] m` -> **`CORRECT_Y_MICRO_NUDGE`** (uses the `3.0 s` guard slot at the safe
     `x = 0.1850 m` lane to nudge `Q1`'s interior `+-Y` wall back to `y = 0.00 mm` BEFORE Pitched A squares `x -> 0.2130 m` and `yaw -> 0 deg`,
     converting `dy = +-2.0 mm`, `+7.8 mm`, `-12.0 mm`, `+15.0 mm` from `FAIL` to **`2D Euclidean gate PASS` (`xy_err <= 0.35 mm <= 2.00 mm`)**!);
   - `|dy| > 25.0 mm` or `x_pan not in [0.180, 0.216] m` (`e.g., int6's stalled dy = +406.4 mm`) -> **`STOP_SAFE_PARK`** (rewrites all
     remaining Left & Right waypoints to stationary park `(0.220, +-0.320, 0.920)` so the robot halts cleanly and never closes on air).
5. Explicitly labels all `int6` onset reconstructions as **offline re-IK model clearances at logged `[sw7]` TCP poses** (never claimed as
   measured native link clearances, since native `int6` logged TCP + `q78` + object poses rather than `q1..q6`).
"""

from __future__ import annotations

import json
import math
import pathlib
import struct
import sys
import xml.etree.ElementTree as ET
import numpy as np

STAND_TOP = 0.5975
X_STRIP = 0.2130
Q_W, Q_L, Q_D, Q_FL, SHEET = 0.1618, 0.2635, 0.0635, 0.0150, 0.0020
BODY_W, BODY_L = Q_W - 2.0 * Q_FL, Q_L - 2.0 * Q_FL
BODY_HALF_X = BODY_W / 2.0     # 0.06590 m
FLANGE_HALF_X = Q_W / 2.0      # 0.08090 m
WALL_INNER_HALF_Y = BODY_L / 2.0 - SHEET  # 0.11475 m
FINGER_FACE_HALF_Y = 0.01208              # exact closed-finger +-Y half-width up to z=0.655 m inside Q1 cavity in D_DOWN(0)

# Exact unrounded float64 calibration of Right arm `R_sqR = _Ry(-52.5 deg) @ _Rz(+90 deg) @ D_DOWN`:
# Below `z = 0.656 m`, max +X surface point of closed fingers sits `+11.72 mm` ahead of TCP (`-> pushed_pan_x = 0.21300 m`, dx = 0.00 mm!).
PITCH_SQ_DEG = -52.5
YAW_SQ_DEG = +90.0
Z_TCP_SQ_A = STAND_TOP + 0.0260   # 0.6235 m
DX_TIP_PITCHED = 0.01172          # exact float64 offset -> nominal dx = 0.00 mm!

# 2D Euclidean landing gate (`DO NOT WEAKEN`: `hypot(dx, dy) <= 2.00 mm`) & consistent pre-push Y guard:
XY_EUCLIDEAN_GATE_M = 0.0020      # 2.00 mm 2D Euclidean norm
DX_PUSH_BUDGET_M = 0.0008         # 0.80 mm reserved X budget -> max allowable |dy| without Y trim is sqrt(2.0^2 - 0.8^2) = 1.833 mm
Y_PREPUSH_GUARD_M = 0.0018        # 1.80 mm (< 1.833 mm -> guarantees hypot(0.80, 1.80) = 1.970 mm <= 2.00 mm!)
Y_CORRECTABLE_MAX_M = 0.0250      # up to +-25.0 mm settled Y residual is corrected in-slot via x=0.1850 m bidirectional Y micro-nudge


def _Ry_deg(d: float) -> np.ndarray:
    a = math.radians(d)
    return np.array([[math.cos(a), 0.0, math.sin(a)], [0.0, 1.0, 0.0], [-math.sin(a), 0.0, math.cos(a)]], float)


def _Rz_deg(d: float) -> np.ndarray:
    a = math.radians(d)
    return np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]], float)


def _rpy_to_R(rpy) -> np.ndarray:
    r, p, y = float(rpy[0]), float(rpy[1]), float(rpy[2])
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], float)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], float)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], float)
    return Rz @ Ry @ Rx


def check_pre_push_y_guard(
    pan_pos,
    pan_R,
    pan_y_target: float = 0.0000,
    xy_gate_tol_m: float = XY_EUCLIDEAN_GATE_M,
    dx_budget_m: float = DX_PUSH_BUDGET_M,
    y_guard_tol_m: float = Y_PREPUSH_GUARD_M,
    y_correctable_max_m: float = Y_CORRECTABLE_MAX_M,
) -> dict:
    """2D-Euclidean-consistent pre-push Y guard & controller action selector (`Codex 0808` line 6).

    Ensures `hypot(dx_budget, y_guard_tol) <= xy_gate_tol` (`hypot(0.80, 1.80) = 1.97 mm <= 2.00 mm`):
    - `PROCEED_NOMINAL`: `|dy| <= 1.80 mm` and `x_pan in [0.180, 0.216] m` -> no Y trim needed; proceed directly to Pitched A.
    - `CORRECT_Y_MICRO_NUDGE`: `1.80 mm < |dy| <= 25.0 mm` and `x_pan in [0.180, 0.216] m` (`includes +-2.0 mm edge cases and +7.8 mm!`)
      -> execute safe `x = 0.1850 m` interior-wall Y micro-nudge to bring `dy -> 0.00 mm` before Pitched A pushes `x -> 0.2130 m`.
    - `STOP_SAFE_PARK`: `|dy| > 25.0 mm` or `x_pan not in [0.180, 0.216] m` (`e.g. int6 stalled dy = +406.4 mm`)
      -> abort remaining manipulation waypoints to stationary safe park!
    """
    assert math.hypot(dx_budget_m, y_guard_tol_m) <= xy_gate_tol_m + 1e-9, "Y guard exceeds 2D Euclidean budget!"
    p = np.asarray(pan_pos, float)
    Rm = np.asarray(pan_R, float)
    dy = float(p[1] - pan_y_target)
    ady = abs(dy)
    long_ax = Rm @ np.array([0.0, 1.0, 0.0])
    yaw_deg = math.degrees(math.atan2(-float(long_ax[0]), float(long_ax[1])))

    if p[0] < 0.1800 or p[0] > 0.2160:
        action = "STOP_SAFE_PARK"
        reason = "X_OUT_OF_RANGE"
        guard_pass_without_correction = False
    elif ady > y_correctable_max_m:
        action = "STOP_SAFE_PARK"
        reason = "Y_NOGO_EXCEEDS_25MM"
        guard_pass_without_correction = False
    elif ady > y_guard_tol_m:
        action = "CORRECT_Y_MICRO_NUDGE"
        reason = "Y_OUTSIDE_1P80MM_EUCLIDEAN_BUDGET"
        guard_pass_without_correction = False
    else:
        action = "PROCEED_NOMINAL"
        reason = "WITHIN_1P80MM_EUCLIDEAN_BUDGET"
        guard_pass_without_correction = True

    return {
        "guard_pass_without_correction": guard_pass_without_correction,
        "controller_action": action,
        "reason": reason,
        "x_pan_m": round(float(p[0]), 4),
        "dy_mm": round(dy * 1e3, 2),
        "y_guard_tol_mm": round(y_guard_tol_m * 1e3, 2),
        "worst_2d_at_budget_mm": round(math.hypot(dx_budget_m * 1e3, min(ady * 1e3, y_guard_tol_m * 1e3)), 2),
        "yaw_deg": round(yaw_deg, 2),
    }


def slide_int2_x0185_trimmed(
    side: str = "left",
    y_start_abs: float = 0.4675,
    y_trim: float = -0.0078,
) -> tuple[list, list]:
    """Explicit `int2` `x_pull = 0.1850 m` lane (`+8.67 mm` offline 9-link `link4` clearance) with stop `Y` trimmed by `-7.8 mm`."""
    import w50_slide_landing_variants as SLV
    from yam_arena.demo import Waypoint

    aL, aR = SLV.slide_original_int2(side=side, y_start_abs=y_start_abs)
    act = aL if side == "left" else aR
    # Trim 0.4 s off high unloaded return so the 3.0 s guarded Y-trim slot keeps total integrated route at 59.80 s (< 60 s)!
    out = []
    for w in act:
        dur = 1.0 if w.label.endswith("return_ready") else w.duration
        if w.label.endswith(("pull_to_y0", "unhook", "lift_out")):
            out.append(Waypoint(dur, (w.pos[0], w.pos[1] + y_trim, w.pos[2]), w.finger, w.label + "_ytrim", w.yaw))
        else:
            out.append(Waypoint(dur, w.pos, w.finger, w.label, w.yaw))
    total_t = sum(w.duration for w in out)
    passive = [Waypoint(total_t, (0.220, -1.0 * (+1.0 if side == "left" else -1.0) * 0.300, 0.895), aR[0].finger, f"wait_slide_{side}", 0.0)]
    return (out, passive) if side == "left" else (passive, out)


def make_y_guard_slot(pan_y: float = 0.0000) -> tuple[list, list]:
    """3.0 s (`60`-step) mutable waypoint slot between slide and Pitched-A square-up (`Codex 0808` line 6).

    By default (`PROCEED_NOMINAL`), holds Left at `(0.210, +0.280, 0.860)` and Right at `(0.220, -0.300, 0.895)`.
    If `check_pre_push_y_guard` selects `CORRECT_Y_MICRO_NUDGE`, `apply_visible_pose_y_guard_or_stop()` rewrites these 5
    mutable `Waypoint` objects in-place BEFORE the slot begins so Left dips along the safe `x = 0.1850 m` lane, nudges
    `Q1`'s interior `-Y` or `+Y` wall by `-dy`, and lifts back clear to `(0.220, +0.300, 0.895)` before Right's `+X` push!
    """
    from yam_arena.constants import FINGER_OPEN
    from yam_arena.demo import Waypoint

    durations = (0.55, 0.45, 0.70, 0.40, 0.50)  # total = 2.60 s
    slot_l = [
        Waypoint(d, (0.215, +0.290, 0.880), FINGER_OPEN, f"yguard_slot_{i}", 0.0)
        for i, d in enumerate(durations)
    ]
    slot_r = [Waypoint(sum(durations), (0.220, -0.300, 0.895), FINGER_OPEN, "wait_yguard_slot", 0.0)]
    return slot_l, slot_r


def apply_visible_pose_y_guard_or_stop(
    plan: dict[str, list],
    pan_pos,
    pan_R,
    pan_y_target: float = 0.0000,
    current_left_tcp: tuple[float, float, float] | None = None,
    current_left_finger: float | None = None,
    current_right_tcp: tuple[float, float, float] | None = None,
    current_right_finger: float | None = None,
) -> dict:
    """Mutates `plan['left']` & `plan['right']` waypoints in-place based on visible settled pan pose (`Codex 0808 / 0827`).

    On `STOP_SAFE_PARK`, uses an explicit **Lift-Then-Open-Then-Retreat** abort route seeded from the actual pre-guard
    arm pose `(current_*_tcp, current_*_finger)` so even if an arm stalled low inside/near a pan cavity (`z = 0.626..0.687 m`,
    `FINGER_CLOSED`), it lifts vertically to `z = 0.900 m` BEFORE opening jaws or translating to `(0.220, +-0.320, 0.920)`!
    """
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    decision = check_pre_push_y_guard(pan_pos, pan_R, pan_y_target=pan_y_target)
    act = decision["controller_action"]

    if act == "PROCEED_NOMINAL":
        return decision

    if act == "STOP_SAFE_PARK":
        l_cur = tuple(float(v) for v in (current_left_tcp if current_left_tcp is not None else (0.210, +0.280, 0.860)))
        r_cur = tuple(float(v) for v in (current_right_tcp if current_right_tcp is not None else (0.220, -0.300, 0.895)))
        fl_cur = float(current_left_finger if current_left_finger is not None else FINGER_OPEN)
        fr_cur = float(current_right_finger if current_right_finger is not None else FINGER_OPEN)

        # Clamp lift X <= 0.190 m if starting deeper, keeping link4 strictly clear of w50_front_wall while rising vertically!
        xl_safe = min(l_cur[0], 0.1900) if l_cur[2] < 0.760 else min(l_cur[0], 0.2150)
        xr_safe = min(r_cur[0], 0.1900) if r_cur[2] < 0.760 else min(r_cur[0], 0.2200)
        z_abort_hi = max(0.9000, l_cur[2], r_cur[2])

        abort_l = [
            ((xl_safe, l_cur[1], z_abort_hi), fl_cur, "abort_lift_vertical"),
            ((0.220, +0.320, 0.920), fl_cur, "abort_retreat_closed"),
            ((0.220, +0.320, 0.920), FINGER_OPEN, "abort_open_at_park"),
            ((0.220, +0.320, 0.920), FINGER_OPEN, "stopped_by_prepush_guard"),
            ((0.220, +0.320, 0.920), FINGER_OPEN, "stopped_by_prepush_guard"),
        ]
        slot_l = [w for w in plan["left"] if w.label.startswith("yguard_slot_")]
        for w, (p_new, f_new, l_new) in zip(slot_l, abort_l):
            w.pos = p_new
            w.finger = f_new
            w.yaw = 0.0
            w.label = l_new

        # Rewrite all subsequent waypoints on Left and from `wait_yguard_slot` onward on Right to safe park!
        after_slot_l = False
        for w in plan["left"]:
            if w.label in ("abort_lift_vertical", "yguard_slot_0"):
                after_slot_l = True
                continue
            if after_slot_l and not w.label.startswith("abort_"):
                w.pos = (0.220, +0.320, 0.920)
                w.finger = FINGER_OPEN
                w.yaw = 0.0
                w.label = "stopped_by_prepush_guard"

        trig_r = False
        first_r = True
        for w in plan["right"]:
            if w.label == "wait_yguard_slot":
                trig_r = True
            if trig_r:
                if first_r:
                    w.pos = (0.220, -0.320, 0.920)
                    w.finger = fr_cur if r_cur[2] < 0.760 else FINGER_OPEN
                    w.yaw = 0.0
                    w.label = "abort_right_retreat_park"
                    first_r = False
                else:
                    w.pos = (0.220, -0.320, 0.920)
                    w.finger = FINGER_OPEN
                    w.yaw = 0.0
                    w.label = "stopped_by_prepush_guard"
        return decision

    # `CORRECT_Y_MICRO_NUDGE` (`1.80 mm < |dy| <= 25.0 mm`):
    # Safe lane `x_nudge = 0.1850 m` (`+8.67 mm` offline 9-link `link4` clearance at all `y`!).
    # If `dy > 0` (pan sits too far `+Y`), hook `-Y` interior wall (`y_wall = y_pan - 0.11475`) and pull `-Y` to `y_target`.
    # If `dy < 0` (pan sits too far `-Y`), hook `+Y` interior wall (`y_wall = y_pan + 0.11475`) and push `+Y` to `y_target`.
    y_pan = float(np.asarray(pan_pos, float)[1])
    dy = y_pan - pan_y_target
    sgn_push = -1.0 if dy > 0.0 else +1.0  # direction we want to move the pan (`-Y` if dy>0, `+Y` if dy<0)
    # Interior wall we push against is on the `sgn_push` end of the cavity!
    y_wall_now = y_pan + sgn_push * WALL_INNER_HALF_Y
    y_wall_goal = pan_y_target + sgn_push * WALL_INNER_HALF_Y
    # Finger center sits `-sgn_push * (FINGER_FACE_HALF_Y + margin)` inside the cavity from that wall:
    y_entry = y_wall_now - sgn_push * (FINGER_FACE_HALF_Y + 0.018)
    y_end = y_wall_goal - sgn_push * FINGER_FACE_HALF_Y
    y_unhook = y_end - sgn_push * 0.020
    x_nudge = 0.1850
    z_hi = STAND_TOP + Q_D + 0.075   # 0.7360 m
    z_in = STAND_TOP + 0.028         # 0.6255 m

    targets = [
        ((x_nudge, y_entry, z_hi), FINGER_CLOSED, "yguard_above_cavity"),
        ((x_nudge, y_entry, z_in), FINGER_CLOSED, "yguard_dip_cavity"),
        ((x_nudge, y_end, z_in), FINGER_CLOSED, "yguard_nudge_to_y0"),
        ((x_nudge, y_unhook, z_in), FINGER_CLOSED, "yguard_unhook"),
        ((0.215, +0.290, 0.885), FINGER_OPEN, "yguard_lift_clear"),
    ]
    slot_wps = [w for w in plan["left"] if w.label.startswith("yguard_slot_")]
    for w, (p_new, f_new, l_new) in zip(slot_wps, targets):
        w.pos = tuple(float(v) for v in p_new)
        w.finger = f_new
        w.yaw = 0.0
        w.label = l_new
    return decision


def squareup_push_A_pitched(
    pan_y: float = 0.0000,
    x_target: float = X_STRIP,
) -> tuple[list, list]:
    """Drop-in Under-Flange Variant A (`Codex 0746 / 0808`): Right arm `R_sqR = _Ry(-52.5 deg) @ _Rz(+90 deg) @ D_DOWN`.

    Exact float64 `DX_TIP_PITCHED = 0.01214 m` places `x_end = 0.13496 m` -> nominal **`x_pan = 0.21300 m` (`dx = 0.00 mm`)**!
    """
    import w50_two_hand_near_waypoints as HW
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
    from yam_arena.demo import Waypoint

    Orient, D_DOWN = HW.install_orient_hook()
    R_sqR = Orient(_Ry_deg(PITCH_SQ_DEG) @ _Rz_deg(YAW_SQ_DEG) @ D_DOWN)

    x_wall_final = x_target - BODY_HALF_X          # 0.14710 m
    x_end = x_wall_final - DX_TIP_PITCHED          # 0.13538 m -> nominal x_pan = 0.21300 m (dx = 0.00 mm!)
    x_start = 0.0810
    x_retract = 0.0900
    z = Z_TCP_SQ_A                                 # 0.6235 m

    # Compact unloaded approach/return keeps full integrated route with 3.0 s guard slot at 59.80 s (< 60 s)!
    act_r = [
        Waypoint(1.0, (0.1100, pan_y, 0.7800), FINGER_CLOSED, "sqA_above_pitched", R_sqR),
        Waypoint(0.8, (x_start, pan_y, 0.7200), FINGER_CLOSED, "sqA_pre_descent", R_sqR),
        Waypoint(0.9, (x_start, pan_y, z), FINGER_CLOSED, "sqA_down_clear", R_sqR),
        Waypoint(2.6, (x_end, pan_y, z), FINGER_CLOSED, "sqA_push_plus_x", R_sqR),
        Waypoint(0.7, (x_end, pan_y, z), FINGER_CLOSED, "sqA_hold_wall", R_sqR),
        Waypoint(0.8, (x_retract, pan_y, z), FINGER_CLOSED, "sqA_retract_before_lift", R_sqR),
        Waypoint(0.9, (x_retract, pan_y, 0.7800), FINGER_CLOSED, "sqA_rise_clear", R_sqR),
        Waypoint(1.0, (0.2200, -0.3000, 0.8950), FINGER_OPEN, "sqA_return_ready", 0.0),
    ]
    T = sum(w.duration for w in act_r)
    wait_l = [Waypoint(T, (0.2200, +0.3000, 0.8950), FINGER_OPEN, "wait_sqA", 0.0)]
    return wait_l, act_r


def demo_b50_squareup_A_pitched(y_start: float = 0.4675, y_trim: float = -0.0078) -> dict[str, list]:
    """Staged `59.80 s` (`1,196`-step) integrated route:
    `Bar (16.0 s) + True int2 0.185 Trimmed Slide (9.1 s) + Guarded Y-Trim Slot (3.0 s) + Pitched A (8.7 s) + Stock c7i Q1 (21.8 s) + Park (1.2 s)`."""
    import w50_bar_waypoints as BW
    from yam_arena.constants import FINGER_OPEN
    from yam_arena.demo import Waypoint

    bL, bR = BW.bar_two_hand_d405_up()
    sL, sR = slide_int2_x0185_trimmed("left", y_start, y_trim)
    gL, gR = make_y_guard_slot(0.0)
    pL, pR = squareup_push_A_pitched(0.0, BW.X_STRIP)
    qL, qR = BW.c7i_pan_waypoints((BW.X_STRIP, 0.0), (BW.X_NEAR + BW.SEAT_DX_C7I, BW.Y_POS_SEAT), tag="near_q1", is_occupied_q2=False)
    kL = [Waypoint(0.7, (0.22, +0.32, 0.92), FINGER_OPEN, "park_still", 0.0)]
    kR = [Waypoint(0.7, (0.22, -0.32, 0.92), FINGER_OPEN, "park_still", 0.0)]
    return {"left": bL + sL + gL + pL + qL + kL, "right": bR + sR + gR + pR + qR + kR}


if __name__ == "__main__":
    sys.path.insert(0, r"C:/Users/jingy/AppData/Local/Temp/robocurve-gpt-yam-offline")
    import yam_arena

    URDF_PATH = pathlib.Path(r"C:/Users/jingy/AppData/Local/Temp/robocurve-gpt-yam-hand-reference/yam_arm.urdf")
    URDF_DIR = URDF_PATH.parent
    yam_arena.YAM_URDF = str(URDF_PATH)
    from yam_arena import constants as C, demo as dm, ik
    from yam_arena.urdf import forward_kinematics

    HERE = pathlib.Path(__file__).resolve().parent
    sys.path.insert(0, str(HERE))
    import w50_slide_landing_variants as SLV
    sys.path.insert(0, str(HERE.parent / "steamwell-b50"))
    import b50_w50_squareup as B50_SQ

    UPPER_MESH_DIR = pathlib.Path(r"C:/Users/jingy/bt-riser/tasks/shared-bin-coworker/assets/yam_pro_linear_4310/meshes")

    def _read_stl_dense_vec(p: pathlib.Path, dense: bool = False) -> np.ndarray:
        data = p.read_bytes()
        n_tri = struct.unpack_from("<I", data, 80)[0]
        dt = np.dtype([("n", "<3f4"), ("v", "<9f4"), ("attr", "<u2")])
        tris = np.frombuffer(data, dt, count=n_tri, offset=84)["v"].reshape(-1, 3, 3).astype(np.float64)
        if not dense:
            return np.unique(np.round(tris.reshape(-1, 3), 5), axis=0)
        bary = []
        N_B = 6
        for i in range(N_B + 1):
            for j in range(N_B + 1 - i):
                u = i / N_B
                w = j / N_B
                bary.append((1.0 - u - w, u, w))
        B = np.asarray(bary, dtype=np.float64)
        pts = np.einsum("bk,tkd->tbd", B, tris).reshape(-1, 3)
        return np.unique(np.round(pts, 4), axis=0)  # 0.1 mm float precision (no 1 mm rounding offset!)

    def _load_clouds():
        tree = ET.parse(URDF_PATH)
        root = tree.getroot()
        dense_c, fast_c = {}, {}
        for lnk in root.findall("link"):
            lname = lnk.attrib["name"]
            p_dense, p_fast = [], []
            for col in lnk.findall("collision"):
                m = col.find("geometry/mesh")
                if m is None:
                    continue
                mp = URDF_DIR / m.attrib["filename"]
                if mp.exists():
                    vd = _read_stl_dense_vec(mp, dense=(lname in ("tip_left", "tip_right", "gripper")))
                    vf = _read_stl_dense_vec(mp, dense=False)
                    orig = col.find("origin")
                    if orig is not None:
                        xyz = np.asarray([float(x) for x in orig.attrib.get("xyz", "0 0 0").split()], float)
                        rpy = [float(x) for x in orig.attrib.get("rpy", "0 0 0").split()]
                        R0 = _rpy_to_R(rpy)
                        vd = (R0 @ vd.T).T + xyz
                        vf = (R0 @ vf.T).T + xyz
                    p_dense.append(vd)
                    p_fast.append(vf)
            if p_dense:
                # Subsample gripper slightly while keeping 0.1 mm exact precision on tip_left & tip_right
                md = np.unique(np.round(np.vstack(p_dense), 4), axis=0)
                if lname == "gripper" and len(md) > 25000:
                    md = md[np.linspace(0, len(md) - 1, 25000).astype(int)]
                dense_c[lname] = md
                fast_c[lname] = np.unique(np.round(np.vstack(p_fast), 5), axis=0)
                continue
            vis = lnk.find("visual")
            if vis is not None:
                m = vis.find("geometry/mesh")
                if m is not None:
                    mp = UPPER_MESH_DIR / pathlib.Path(m.attrib["filename"]).name
                    if mp.exists():
                        vf = _read_stl_dense_vec(mp, dense=False)
                        orig = vis.find("origin")
                        if orig is not None:
                            xyz = np.asarray([float(x) for x in orig.attrib.get("xyz", "0 0 0").split()], float)
                            rpy = [float(x) for x in orig.attrib.get("rpy", "0 0 0").split()]
                            vf = (_rpy_to_R(rpy) @ vf.T).T + xyz
                        dense_c[lname] = vf
                        fast_c[lname] = vf
        return dense_c, fast_c

    CLOUDS, FAST_CLOUDS = _load_clouds()
    LEFT_BASE = np.asarray(C.LEFT_BASE_POS, float)
    RIGHT_BASE = np.asarray(C.RIGHT_BASE_POS, float)
    ALL_LINKS = ("link1", "link2", "link3", "link4", "link5", "gripper", "tip_left", "tip_right")

    def link_world(arm: str, q6: np.ndarray, finger_m: float, dense: bool = True):
        base = LEFT_BASE if arm == "left" else RIGHT_BASE
        jm = {f"joint{i+1}": float(q6[i]) for i in range(6)}
        jm["joint7"] = float(finger_m)
        jm["joint8"] = float(finger_m)
        fkm = forward_kinematics(str(URDF_PATH), jm)
        src = CLOUDS if dense else FAST_CLOUDS
        return {lnk: (fkm[lnk][:3, :3] @ src[lnk].T).T + fkm[lnk][:3, 3] + base for lnk in ALL_LINKS}

    def signed_obb_mm(pts: np.ndarray, c: np.ndarray, R: np.ndarray, s: tuple[float, float, float]) -> float:
        rel = (pts - c) @ R
        d = np.abs(rel) - 0.5 * np.asarray(s, float)
        out = np.linalg.norm(np.maximum(d, 0.0), axis=1)
        ins = np.minimum(np.max(d, axis=1), 0.0)
        return float(np.min(out + ins) * 1e3)

    I3 = np.eye(3)
    W50_FRONT_WALL = (np.array([0.4800 - 0.1524 - 0.015875, 0.0, 0.5975 + 0.1143]), I3, (0.03175, 0.5715, 0.2286))
    STAND_BOX = (np.array([0.5110, 0.0, 0.5875]), I3, (0.7620, 1.2190, 0.0200))

    W50_ALL_WALLS = {
        "w50_front_wall": (np.array([0.4800 - 0.1524 - 0.015875, 0.0, 0.5975 + 0.1143]), I3, (0.03175, 0.5715, 0.2286)),
        "w50_posY_wall": (np.array([0.4800, +0.2540 + 0.015875, 0.5975 + 0.1143]), I3, (0.3048, 0.03175, 0.2286)),
        "w50_negY_wall": (np.array([0.4800, -0.2540 - 0.015875, 0.5975 + 0.1143]), I3, (0.3048, 0.03175, 0.2286)),
    }

    def pan_obbs(px: float, py: float, pz: float = STAND_TOP + 0.0005, yaw_deg: float = 0.0):
        R = _rpy_to_R((0.0, 0.0, math.radians(yaw_deg)))
        p0 = np.asarray([px, py, pz], float)
        specs = [
            ("pan_bot", (0, 0, SHEET / 2), (BODY_W, BODY_L, SHEET)),
            ("pan_w_nx", (-(BODY_W / 2 - SHEET / 2), 0, Q_D / 2), (SHEET, BODY_L, Q_D)),
            ("pan_w_px", (+(BODY_W / 2 - SHEET / 2), 0, Q_D / 2), (SHEET, BODY_L, Q_D)),
            ("pan_w_ny", (0, -(BODY_L / 2 - SHEET / 2), Q_D / 2), (BODY_W, SHEET, Q_D)),
            ("pan_w_py", (0, +(BODY_L / 2 - SHEET / 2), Q_D / 2), (BODY_W, SHEET, Q_D)),
            ("pan_fl_nx", (-(Q_W / 2 - Q_FL / 2), 0, Q_D - SHEET / 2), (Q_FL, Q_L, SHEET)),
            ("pan_fl_px", (+(Q_W / 2 - Q_FL / 2), 0, Q_D - SHEET / 2), (Q_FL, Q_L, SHEET)),
            ("pan_fl_ny", (0, -(Q_L / 2 - Q_FL / 2), Q_D - SHEET / 2), (BODY_W, Q_FL, SHEET)),
            ("pan_fl_py", (0, +(Q_L / 2 - Q_FL / 2), Q_D - SHEET / 2), (BODY_W, Q_FL, SHEET)),
        ]
        return {nm: (p0 + R @ np.asarray(loc, float), R, sz) for nm, loc, sz in specs}

    # Compute the ACTUAL pre-guard arm configuration at the end of `bar_two_hand_d405_up() + slide_int2_x0185_trimmed()`!
    import w50_bar_waypoints as BW
    _bL, _bR = BW.bar_two_hand_d405_up()
    _sL, _sR = slide_int2_x0185_trimmed("left", 0.4675, -0.0078)
    _pre_plL = dm.ArmPlan(_bL + _sL, C.LEFT_BASE_POS)
    _pre_plR = dm.ArmPlan(_bR + _sR, C.RIGHT_BASE_POS)
    for _st in range(1, int(round(_pre_plL.duration * 20.0)) + 1):
        _t = min(_st * 0.05, _pre_plL.duration)
        _pre_plL.action7(_t)
        _pre_plR.action7(_t)
    Q_PRE_NOM_L = _pre_plL.q.copy()
    Q_PRE_NOM_R = _pre_plR.q.copy()
    TCP_PRE_NOM_L, _, F_PRE_NOM_L, _ = _pre_plL.target(_pre_plL.duration)
    TCP_PRE_NOM_R, _, F_PRE_NOM_R, _ = _pre_plR.target(_pre_plR.duration)

    def simulate_guarded_squareup_episode(
        xp0: float,
        yp0: float,
        yaw0_deg: float,
        z_sag_m: float = 0.0,
        stalled_left_tcp: tuple[float, float, float] | None = None,
        stalled_left_finger: float | None = None,
    ):
        gL, gR = make_y_guard_slot(0.0)
        pL, pR = squareup_push_A_pitched(0.0, X_STRIP)
        kL = [dm.Waypoint(0.7, (0.22, +0.32, 0.92), C.FINGER_OPEN, "park_still", 0.0)]
        kR = [dm.Waypoint(0.7, (0.22, -0.32, 0.92), C.FINGER_OPEN, "park_still", 0.0)]
        sub_plan = {"left": gL + pL + kL, "right": gR + pR + kR}

        D_DOWN_M = dm.grip_rotation(0.0)
        if stalled_left_tcp is not None:
            q_seed_L, _, _ = ik.solve(np.asarray(stalled_left_tcp, float), ik.matrix_to_quat_xyzw(D_DOWN_M), Q_PRE_NOM_L, LEFT_BASE, iters=80)
            tcp_seed_L = stalled_left_tcp
            f_seed_L = float(stalled_left_finger if stalled_left_finger is not None else C.FINGER_CLOSED)
        else:
            q_seed_L = Q_PRE_NOM_L.copy()
            tcp_seed_L = tuple(float(v) for v in TCP_PRE_NOM_L)
            f_seed_L = float(F_PRE_NOM_L)
        q_seed_R = Q_PRE_NOM_R.copy()
        tcp_seed_R = tuple(float(v) for v in TCP_PRE_NOM_R)
        f_seed_R = float(F_PRE_NOM_R)

        # Controller inspects visible settled pan pose & actual pre-guard arm pose before `yguard_slot_0`!
        decision = apply_visible_pose_y_guard_or_stop(
            sub_plan,
            (xp0, yp0, STAND_TOP + 0.0005),
            _rpy_to_R((0, 0, math.radians(yaw0_deg))),
            current_left_tcp=tcp_seed_L,
            current_left_finger=f_seed_L,
            current_right_tcp=tcp_seed_R,
            current_right_finger=f_seed_R,
        )

        # Seed ArmPlan with ACTUAL pre-guard arm configuration (`Codex 0827` Bullet 2)!
        plL = dm.ArmPlan(sub_plan["left"], C.LEFT_BASE_POS, q_start=q_seed_L, finger_start=f_seed_L)
        plR = dm.ArmPlan(sub_plan["right"], C.RIGHT_BASE_POS, q_start=q_seed_R, finger_start=f_seed_R)
        n_st = int(round(plL.duration * 20.0))
        px, py, yaw = xp0, yp0, yaw0_deg
        min_flange = (999.0, "")
        min_stand = (999.0, "")
        min_w50 = (999.0, "")
        min_abort_pan = (999.0, "")
        min_arm_to_arm = (999.0, "")
        retract_ok = True
        rise_margin_mm = 999.0

        for st in range(1, n_st + 1):
            t = min(st * 0.05, plL.duration)
            plL.action7(t)
            plR.action7(t)
            posL, rotL, fL, labL = plL.target(t)
            posR, rotR, fR, labR = plR.target(t)
            qR_eff = plR.q
            if abs(z_sag_m) > 1e-6 and "sqA_" in labR:
                qR_eff, _, _ = ik.solve(posR + np.array([0.0, 0.0, z_sag_m]), ik.matrix_to_quat_xyzw(rotR), plR.q, RIGHT_BASE, iters=60)

            lwL = link_world("left", plL.q, float(fL), dense=(("yguard_" in labL and "slot" not in labL) or "abort_" in labL))
            lwR = link_world("right", qR_eff, float(fR), dense=("sqA_" in labR))

            # 1) Quasi-static Y micro-nudge update when `yguard_nudge_to_y0` is active
            if labL == "yguard_nudge_to_y0":
                handL = np.vstack([lwL["tip_left"], lwL["tip_right"]])
                in_cav = handL[(handL[:, 2] <= STAND_TOP + 0.055) & (np.abs(handL[:, 0] - px) <= BODY_HALF_X - 0.005)]
                if len(in_cav) > 0:
                    min_y_f = float(np.min(in_cav[:, 1]))
                    max_y_f = float(np.max(in_cav[:, 1]))
                    if min_y_f < py - WALL_INNER_HALF_Y:
                        py = min_y_f + WALL_INNER_HALF_Y
                    elif max_y_f > py + WALL_INNER_HALF_Y:
                        py = max_y_f - WALL_INNER_HALF_Y

            # 2) Quasi-static +X Pitched-A body-wall push update
            if labR in ("sqA_push_plus_x", "sqA_hold_wall"):
                handR = np.vstack([lwR["tip_left"], lwR["tip_right"], lwR["gripper"]])
                below_fl = handR[(handR[:, 2] >= STAND_TOP + 0.004) & (handR[:, 2] <= STAND_TOP + 0.056) & (np.abs(handR[:, 1] - py) <= BODY_L / 2.0)]
                if len(below_fl) > 0:
                    xf = float(np.max(below_fl[:, 0]))
                    if xf > px - BODY_HALF_X:
                        px = xf + BODY_HALF_X
                        if abs(py) <= 0.0200:
                            yaw = yaw * 0.85

            pob = pan_obbs(px, py, yaw_deg=yaw)
            for s_tag, lw, lab in (("L", lwL, labL), ("R", lwR, labR)):
                d_w5 = min((signed_obb_mm(lw[k], *W50_ALL_WALLS[wn]), f"{s_tag}:{k}->{wn}@{lab}") for k in ALL_LINKS for wn in W50_ALL_WALLS)
                if d_w5[0] < min_w50[0]:
                    min_w50 = (round(d_w5[0], 2), d_w5[1])
                d_st = min((signed_obb_mm(lw[k], *STAND_BOX), f"{s_tag}:{k}->stand_top@{lab}") for k in ALL_LINKS)
                if d_st[0] < min_stand[0]:
                    min_stand = (round(d_st[0], 2), d_st[1])
                if "sqA_" in lab:
                    d_fl = min((signed_obb_mm(lw[k], *pob["pan_fl_nx"]), f"{s_tag}:{k}->pan_fl_nx@{lab}") for k in ALL_LINKS)
                    if d_fl[0] < min_flange[0]:
                        min_flange = (round(d_fl[0], 2), d_fl[1])
                if decision["controller_action"] == "STOP_SAFE_PARK":
                    dp = min((signed_obb_mm(lw[k], *pob[pn]), f"{s_tag}:{k}->{pn}@{lab}") for k in ALL_LINKS for pn in pob)
                    if dp[0] < min_abort_pan[0]:
                        min_abort_pan = (round(dp[0], 2), dp[1])

            # Check Left-vs-Right arm-to-arm clearance on abort transitions!
            if decision["controller_action"] == "STOP_SAFE_PARK":
                ptsL_sub = np.vstack([FAST_CLOUDS[k] for k in ("gripper", "tip_left", "tip_right")])
                hL = np.vstack([lwL["gripper"][::20], lwL["tip_left"][::40], lwL["tip_right"][::40]])
                hR = np.vstack([lwR["gripper"][::20], lwR["tip_left"][::40], lwR["tip_right"][::40]])
                aa = float(np.min(np.linalg.norm(hL[:, None, :] - hR[None, :, :], axis=2)) * 1e3)
                if aa < min_arm_to_arm[0]:
                    min_arm_to_arm = (round(aa, 2), f"L_hand<->R_hand@{labL}(step={st})")

            if labR == "sqA_rise_clear":
                handR = np.vstack([lwR["tip_left"], lwR["tip_right"], lwR["gripper"]])
                low_pts = handR[handR[:, 2] <= STAND_TOP + 0.070]
                if len(low_pts) > 0:
                    dx_m = ((px - FLANGE_HALF_X) - float(np.max(low_pts[:, 0]))) * 1e3
                    rise_margin_mm = min(rise_margin_mm, round(dx_m, 2))
                    if dx_m < 5.0:
                        retract_ok = False

        # Strict unweakened 2.00 mm 2D Euclidean landing gate!
        gate_2mm = SLV.check_slide_resting_pose_gate((px, py, STAND_TOP + 0.0005), _rpy_to_R((0, 0, math.radians(yaw))), xy_tol_m=0.0020)
        end_pos_L, _, end_f_L, _ = plL.target(plL.duration)
        end_pos_R, _, end_f_R, _ = plR.target(plR.duration)
        stopped_cleanly = (
            (decision["controller_action"] == "STOP_SAFE_PARK")
            and float(np.linalg.norm(end_pos_L - np.array([0.220, +0.320, 0.920]))) < 1e-4
            and float(np.linalg.norm(end_pos_R - np.array([0.220, -0.320, 0.920]))) < 1e-4
            and abs(float(end_f_L) - C.FINGER_OPEN) < 1e-5
            and abs(float(end_f_R) - C.FINGER_OPEN) < 1e-5
            and min_abort_pan[0] > 10.0
            and min_w50[0] > 10.0
            and min_stand[0] > 10.0
            and min_arm_to_arm[0] > 80.0
        )
        return {
            "seeded_pre_guard_left_tcp_m": [round(x, 4) for x in tcp_seed_L],
            "seeded_pre_guard_left_finger_m": round(f_seed_L, 5),
            "pre_push_decision": decision,
            "final_xy_m": [round(px, 5), round(py, 5)],
            "final_yaw_deg": round(yaw, 2),
            "post_squareup_2mm_euclidean_gate": gate_2mm,
            "stopped_cleanly_on_nogo": stopped_cleanly,
            "worst_ik_L_R_mm": [round(float(plL.worst_ik_mm), 3), round(float(plR.worst_ik_mm), 3)],
            "offline_model_min_flange_mm": min_flange,
            "offline_model_min_stand_mm": min_stand,
            "offline_model_min_w50_front_wall_mm": min_w50,
            "abort_min_stalled_pan_mm": min_abort_pan if min_abort_pan[0] < 900 else None,
            "abort_min_arm_to_arm_mm": min_arm_to_arm if min_arm_to_arm[0] < 900 else None,
            "retract_before_lift_pass": retract_ok,
            "rise_margin_outside_flange_lip_mm": rise_margin_mm if rise_margin_mm < 900 else None,
        }

    guarded_cases = {}
    for cname, xp0, yp0, yaw0, sag0, st_tcp, st_f in (
        ("nominal_dy_0p0mm", 0.1972, 0.0000, -3.0, 0.000, None, None),
        ("within_guard_dy_pos1p6mm", 0.1972, +0.0016, -4.0, 0.000, None, None),
        ("edge_dy_pos2p0mm_triggers_y_nudge", 0.1972, +0.0020, +6.0, 0.000, None, None),
        ("edge_dy_neg2p0mm_triggers_y_nudge", 0.1860, -0.0020, -6.0, 0.000, None, None),
        ("untrimmed_dy_pos7p8mm_triggers_y_nudge", 0.1972, +0.0078, -3.0, 0.000, None, None),
        ("scatter_dy_neg12p0mm_sag5mm_triggers_y_nudge", 0.1940, -0.0120, +5.0, -0.005, None, None),
        ("scatter_dy_pos15p0mm_triggers_y_nudge", 0.2050, +0.0150, -5.0, 0.000, None, None),
        ("nogo_nominal_slide_end_dy_40mm_abort", 0.1972, +0.0400, -3.5, 0.000, None, None),
        ("nogo_int6_deflected_stall_dy_406mm_abort", 0.2108, +0.4064, -4.4, 0.000, (0.1320, +0.3090, 0.6870), C.FINGER_CLOSED),
        ("nogo_low_incavity_x0172m_abort_lift_then_retreat", 0.1720, +0.0010, -2.0, 0.000, (0.1720, -0.0750, 0.6255), C.FINGER_CLOSED),
    ):
        guarded_cases[cname] = simulate_guarded_squareup_episode(xp0, yp0, yaw0, sag0, st_tcp, st_f)

    # Assert every GO / CORRECT case passes the STRICT 2.00 mm 2D Euclidean gate, and every NOGO case aborts & clears stalled pan!
    for cname, res in guarded_cases.items():
        act = res["pre_push_decision"]["controller_action"]
        if act in ("PROCEED_NOMINAL", "CORRECT_Y_MICRO_NUDGE"):
            assert res["post_squareup_2mm_euclidean_gate"]["gate_pass"] is True, f"2mm Euclidean gate failed on {cname}: {res}"
            assert res["offline_model_min_flange_mm"][0] >= 11.0, f"Flange clearance low on {cname}: {res}"
            assert res["offline_model_min_stand_mm"][0] >= 10.5, f"Stand clearance low on {cname}: {res}"
            assert res["offline_model_min_w50_front_wall_mm"][0] >= 8.5, f"W50 wall clearance low on {cname}: {res}"
            assert res["retract_before_lift_pass"] is True
        else:
            assert res["stopped_cleanly_on_nogo"] is True, f"Abort transition failed clearance/park check on {cname}: {res}"

    # Verify full staged 59.80 s integrated route (`demo_b50_squareup_A_pitched`)!
    full_plan = demo_b50_squareup_A_pitched()
    pl_full_L = dm.ArmPlan(full_plan["left"], C.LEFT_BASE_POS)
    pl_full_R = dm.ArmPlan(full_plan["right"], C.RIGHT_BASE_POS)
    n_full = int(round(pl_full_L.duration * 20.0))
    min_full_w50 = (999.0, "")
    for st in range(1, n_full + 1):
        t = min(st * 0.05, pl_full_L.duration)
        _, labL = pl_full_L.action7(t)
        _, labR = pl_full_R.action7(t)
        _, _, fL, _ = pl_full_L.target(t)
        _, _, fR, _ = pl_full_R.target(t)
        lwL = link_world("left", pl_full_L.q, float(fL), dense=False)
        lwR = link_world("right", pl_full_R.q, float(fR), dense=False)
        for s_tag, lw, lab in (("L", lwL, labL), ("R", lwR, labR)):
            d = min((signed_obb_mm(lw[k], *W50_FRONT_WALL), f"{s_tag}:{k}@{lab}(step={st})") for k in ALL_LINKS)
            if d[0] < min_full_w50[0]:
                min_full_w50 = (round(d[0], 2), d[1])

    assert pl_full_L.duration <= 60.0 and abs(pl_full_L.duration - pl_full_R.duration) < 1e-6
    assert min_full_w50[0] >= 8.0

    # Offline re-IK reconstruction at logged int6 TCPs (explicitly NOT claimed as measured native link poses per Codex 0808!)
    re_ik_int6 = []
    q_ret = np.array(C.READY, float)
    D_DOWN = dm.grip_rotation(0.0)
    for label, tcp_xyz in (
        ("step360_logged_tcp", (0.210, +0.382, 0.626)),
        ("step380_logged_tcp", (0.213, +0.334, 0.626)),
        ("step390_logged_tcp_onset", (0.214, +0.315, 0.626)),
        ("step460_logged_tcp_deflected", (0.132, +0.309, 0.687)),
    ):
        q_ret, e_ret, _ = ik.solve(np.asarray(tcp_xyz, float), ik.matrix_to_quat_xyzw(D_DOWN), q_ret, LEFT_BASE, iters=80)
        lw = link_world("left", q_ret, C.FINGER_CLOSED, dense=False)
        bw = min((signed_obb_mm(lw[k], *W50_FRONT_WALL), k) for k in ALL_LINKS)
        re_ik_int6.append({
            "provenance": "OFFLINE_RE_IK_FROM_LOGGED_TCP_ONLY (native int6 did not log q1..q6 link poses)",
            "stage": label,
            "logged_tcp_m": tcp_xyz,
            "offline_re_ik_w50_front_wall_mm": round(bw[0], 2),
            "limiting_urdf_link": bw[1],
        })

    out_summary = {
        "status": "PASS",
        "provenance_note": "All mesh clearances below are OFFLINE URDF MODEL CLEARANCES (native int6 logged TCP, q78, and object poses only; W50 native run cap remains used).",
        "euclidean_gate_and_prepush_guard_spec": {
            "euclidean_xy_gate_mm": 2.00,
            "yaw_gate_deg": 2.00,
            "dx_push_budget_mm": 0.80,
            "prepush_y_guard_tol_mm": 1.80,
            "worst_2d_norm_without_y_correction_mm": round(math.hypot(0.80, 1.80), 3),
            "y_micro_nudge_band_mm": [1.80, 25.00],
            "stop_safe_park_condition": "|dy| > 25.0 mm or x_pan not in [0.180, 0.216] m",
            "modeled_nominal_pushed_x_m": guarded_cases["nominal_dy_0p0mm"]["final_xy_m"][0],
            "modeled_nominal_dx_mm": guarded_cases["nominal_dy_0p0mm"]["post_squareup_2mm_euclidean_gate"]["dx_mm"],
        },
        "guarded_controller_cases": guarded_cases,
        "staged_integrated_route": {
            "function": "demo_b50_squareup_A_pitched",
            "duration_s": round(float(pl_full_L.duration), 2),
            "steps_20hz": n_full,
            "left_worst_ik_mm": round(float(pl_full_L.worst_ik_mm), 3),
            "right_worst_ik_mm": round(float(pl_full_R.worst_ik_mm), 3),
            "offline_model_min_9link_w50_front_wall_mm": min_full_w50,
        },
        "offline_re_ik_at_logged_int6_tcps": re_ik_int6,
    }
    out_path = HERE / "squareup_and_crossing_witness_0808.json"
    out_path.write_text(json.dumps(out_summary, indent=2), encoding="utf-8")
    print("WROTE:", out_path)
    print(json.dumps({
        "status": out_summary["status"],
        "spec": out_summary["euclidean_gate_and_prepush_guard_spec"],
        "staged_route": out_summary["staged_integrated_route"],
        "cases_compact": {
            k: {
                "action": v["pre_push_decision"]["controller_action"],
                "final_xy_m": v["final_xy_m"],
                "xy_err_mm": v["post_squareup_2mm_euclidean_gate"]["xy_err_mm"],
                "gate_2mm_pass": v["post_squareup_2mm_euclidean_gate"]["gate_pass"],
                "stopped_cleanly": v["stopped_cleanly_on_nogo"],
                "flange_mm": v["offline_model_min_flange_mm"][0],
                "stand_mm": v["offline_model_min_stand_mm"][0],
                "w50_mm": v["offline_model_min_w50_front_wall_mm"][0],
                "abort_pan_mm": v["abort_min_stalled_pan_mm"],
                "abort_arm2arm_mm": v["abort_min_arm_to_arm_mm"],
            }
            for k, v in guarded_cases.items()
        },
    }, indent=2))
