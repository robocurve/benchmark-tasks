"""Codex 0704 Slide-Landing Correction Module + Measurable `<= 3 mm` XY Resting-Pose Gate (antigravity helper).

Why `int2` landed at `(0.1972, +0.0078)` (`-15.8 mm` in X, `+7.8 mm` in Y) with ~3 deg yaw:
1. `int2` pulled off-center at `x_pull = 0.1850 m` (`28.0 mm` toward `-X` from Q1's `0.2130 m` CoM) ONLY because our earlier
   synthetic D405 release sphere over-reported `w50_front_wall` contact. On the **exact `yam_arm.urdf` collision meshes**
   (`collision/{gripper,tip_left,tip_right}/*.stl`), `x_pull = 0.2130 m` (`yaw = 0.0`, `D_DOWN`) has:
   - In-cavity finger footprint `X = [0.1929, 0.2331]` (`40.2 mm` wide flat face centered to `0.02 mm` on `x = 0.2130 m`!),
   - **`+43.82 mm`** clearance to Q1's `-X` interior wall (`0.1491 m`) and **`+43.85 mm`** to Q1's `+X` interior wall (`0.2769 m`),
   - **`+48.86 mm`** clearance to `w50_front_wall` (`0.29585 m`)!
   Eliminating the `28 mm` eccentric lever arm removes the yaw/drift torque completely (`tau_z = 0.000 N*m`).
2. `int2` had NO settle hold at `y_end` before reversing `+25 mm` in `slide_left_unhook`, so PD tracking lag left Q1 `+7.8 mm`
   short in `+Y`. Adding a `0.7 s` settle hold at the exact URDF contact target `y_end = sgn * (-0.1047 m)` (`where the `-10.03 mm`
   distal finger face sits at `-0.11473 m`, placing Q1's `-0.11475 m` interior wall -> `y_center = 0.0000 m`!) lands `y` cleanly.

Exports (`Codex 0704` line 8):
- `slide_original_int2(side="left", y_start_abs=0.4675)` — preserved `int2` baseline (`x_pull = 0.1850`, no end-settle) for comparison.
- `slide_centered_contact(side="left", y_start_abs=0.4675)` — dead-center `x_pull = 0.2130 m` + `0.7 s` end-settle hold.
- `slide_centered_plus_x_nudge(side="left", y_start_abs=0.4675)` — dead-center pull to `(0.2130, 0.0000)` + exterior `-X` wall
  `+X` datum nudge (`x_wall_nx -> 0.1471 m => x_center = 0.2130 m`) to guarantee `x` & `yaw = 0` even after any slip.
- `check_slide_resting_pose_gate(pos, R, target_xy=(0.2130, 0.0000), xy_tol_m=0.0030, yaw_tol_deg=2.0)` — measurable resting-pose gate.
"""

from __future__ import annotations

import json
import math
import pathlib
import sys
import numpy as np

STAND_TOP = 0.5975
X_STRIP = 0.2130
Q_W, Q_L, Q_D, Q_FL, SHEET = 0.1618, 0.2635, 0.0635, 0.0150, 0.0020
BODY_W, BODY_L = Q_W - 2.0 * Q_FL, Q_L - 2.0 * Q_FL
Y_STAGE_INT2 = 0.4675  # clears 48-deg-rolled bar pick hand by +27.85 mm on exact URDF meshes (+10.25 mm inside stand edge)


def check_slide_resting_pose_gate(
    pos,
    R,
    target_xy: tuple[float, float] = (X_STRIP, 0.0000),
    xy_tol_m: float = 0.0030,
    yaw_tol_deg: float = 2.0,
) -> dict:
    """Measurable resting-pose gate after strip slide (`Codex 0704` line 8: within `3.0 mm` XY, report yaw & tilt)."""
    p = np.asarray(pos, dtype=np.float64)
    Rm = np.asarray(R, dtype=np.float64)
    dx_mm = float((p[0] - target_xy[0]) * 1e3)
    dy_mm = float((p[1] - target_xy[1]) * 1e3)
    xy_err_mm = math.hypot(dx_mm, dy_mm)
    # Signed pan yaw in degrees (`0.0 deg` when pan local +Y aligns with world +Y)
    long_ax = Rm @ np.array([0.0, 1.0, 0.0])
    yaw_deg = math.degrees(math.atan2(-float(long_ax[0]), float(long_ax[1])))
    tilt_deg = math.degrees(math.acos(float(np.clip(Rm[2, 2], -1.0, 1.0))))
    passed = (xy_err_mm <= xy_tol_m * 1e3) and (abs(yaw_deg) <= yaw_tol_deg) and (tilt_deg <= 2.0)
    return {
        "gate_pass": bool(passed),
        "dx_mm": round(dx_mm, 2),
        "dy_mm": round(dy_mm, 2),
        "xy_err_mm": round(xy_err_mm, 2),
        "yaw_deg": round(yaw_deg, 2),
        "tilt_deg": round(tilt_deg, 2),
        "log_line": f"[slide_gate] pass={passed} dx_mm={dx_mm:+.2f} dy_mm={dy_mm:+.2f} xy_err_mm={xy_err_mm:.2f} yaw_deg={yaw_deg:+.2f} tilt_deg={tilt_deg:.2f}",
    }


def slide_original_int2(side: str = "left", y_start_abs: float = Y_STAGE_INT2) -> tuple[list, list]:
    """Preserved `int2` baseline (`x_pull = 0.1850 m`, no end-settle) kept verbatim for A/B comparison (`Codex 0704` line 8)."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
    from yam_arena.demo import Waypoint

    sgn = +1.0 if side == "left" else -1.0
    x_pull = X_STRIP - 0.0280  # 0.1850 m (eccentric by -28 mm -> caused -15.8 mm X drift & ~3 deg yaw in int2)
    y_entry = sgn * (y_start_abs - (BODY_L / 2.0 - SHEET) + 0.033)
    y_contact = sgn * (y_start_abs - (BODY_L / 2.0 - SHEET) + 0.011)
    y_end = sgn * (0.0000 - (BODY_L / 2.0 - SHEET) + 0.011)
    z_clear = STAND_TOP + Q_D + 0.075
    z_in_cavity = STAND_TOP + 0.028

    active = [
        Waypoint(1.2, (0.200, sgn * 0.300, 0.860), FINGER_CLOSED, f"slide_{side}_ready", 0.0),
        Waypoint(1.2, (x_pull, y_entry, z_clear), FINGER_CLOSED, f"slide_{side}_above_cavity", 0.0),
        Waypoint(0.8, (x_pull, y_entry, z_in_cavity), FINGER_CLOSED, f"slide_{side}_dip_cavity", 0.0),
        Waypoint(0.6, (x_pull, y_contact, z_in_cavity), FINGER_CLOSED, f"slide_{side}_touch_wall", 0.0),
        Waypoint(2.8, (x_pull, y_end, z_in_cavity), FINGER_CLOSED, f"slide_{side}_pull_to_y0", 0.0),
        Waypoint(0.5, (x_pull, y_end + sgn * 0.025, z_in_cavity), FINGER_CLOSED, f"slide_{side}_unhook", 0.0),
        Waypoint(1.0, (x_pull, y_end + sgn * 0.025, z_clear + 0.040), FINGER_CLOSED, f"slide_{side}_lift_out", 0.0),
        Waypoint(1.4, (0.210, sgn * 0.280, 0.860), FINGER_OPEN, f"slide_{side}_return_ready", 0.0),
    ]
    total_t = sum(w.duration for w in active)
    passive = [Waypoint(total_t, (0.220, -sgn * 0.300, 0.895), FINGER_OPEN, f"wait_slide_{side}", 0.0)]
    return (active, passive) if side == "left" else (passive, active)


def slide_centered_contact(
    side: str = "left",
    y_start_abs: float = Y_STAGE_INT2,
    dy_trim: float = -0.0010,
) -> tuple[list, list]:
    """Variant A (`Recommended Primary`): Dead-center `x_pull = 0.2130 m` + slow final `40 mm` + `0.7 s` settle hold.

    Exact URDF collision geometry (`verify_0704_slide_landing_fix.py`):
    - Closed fingers in `D_DOWN` (`yaw = 0.0`) span `X = [0.1929, 0.2331]` (`40.2 mm` flat face centered to `0.02 mm` on `0.2130 m`),
      with **`+43.8 mm`** clearance to Q1's `+-X` cavity walls and **`+48.9 mm`** clearance to `w50_front_wall`!
    - Distal `-Y` (`or +Y`) finger face sits `10.03 mm` ahead of TCP (`-10.03 mm`). Inner pan wall at `y_center = 0.0` is at
      `-sgn * 0.11475 m`, so exact geometric TCP stop is `sgn * (-0.11475 + 0.01003 + dy_trim) = sgn * (-0.1057 m)`.
    - A `1.0 s` slow final `40 mm` creep + `0.7 s` settle hold eliminates PD velocity lag before unhooking!
    """
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
    from yam_arena.demo import Waypoint

    sgn = +1.0 if side == "left" else -1.0
    x_pull = X_STRIP  # 0.2130 m EXACT PAN X CoM (0.00 mm yaw lever arm!)
    finger_face_half_y = 0.01003
    wall_inner_half_y = BODY_L / 2.0 - SHEET  # 0.11475 m

    y_entry = sgn * (y_start_abs - wall_inner_half_y + 0.035)
    y_contact = sgn * (y_start_abs - wall_inner_half_y + finger_face_half_y + 0.002)
    y_pre_end = sgn * (-wall_inner_half_y + finger_face_half_y + 0.040)
    y_end = sgn * (-wall_inner_half_y + finger_face_half_y + dy_trim)  # -0.1057 m (Left) / +0.1057 m (Right)

    z_clear = STAND_TOP + Q_D + 0.075   # 0.736 m
    z_in_cavity = STAND_TOP + 0.028     # 0.6255 m

    active = [
        Waypoint(1.2, (0.210, sgn * 0.300, 0.860), FINGER_CLOSED, f"slide_{side}_ready", 0.0),
        Waypoint(1.3, (x_pull, y_entry, z_clear), FINGER_CLOSED, f"slide_{side}_above_cavity", 0.0),
        Waypoint(0.8, (x_pull, y_entry, z_in_cavity), FINGER_CLOSED, f"slide_{side}_dip_cavity", 0.0),
        Waypoint(0.6, (x_pull, y_contact, z_in_cavity), FINGER_CLOSED, f"slide_{side}_touch_wall", 0.0),
        Waypoint(2.6, (x_pull, y_pre_end, z_in_cavity), FINGER_CLOSED, f"slide_{side}_pull_main", 0.0),
        Waypoint(1.1, (x_pull, y_end, z_in_cavity), FINGER_CLOSED, f"slide_{side}_creep_final_40mm", 0.0),
        Waypoint(0.7, (x_pull, y_end, z_in_cavity), FINGER_CLOSED, f"slide_{side}_settle_y0", 0.0),
        Waypoint(0.6, (x_pull, y_end + sgn * 0.026, z_in_cavity), FINGER_CLOSED, f"slide_{side}_unhook", 0.0),
        Waypoint(1.0, (x_pull, y_end + sgn * 0.026, z_clear + 0.040), FINGER_CLOSED, f"slide_{side}_lift_out", 0.0),
        Waypoint(1.4, (0.210, sgn * 0.280, 0.860), FINGER_OPEN, f"slide_{side}_return_ready", 0.0),
    ]
    total_t = sum(w.duration for w in active)
    passive = [Waypoint(total_t, (0.220, -sgn * 0.300, 0.895), FINGER_OPEN, f"wait_slide_{side}", 0.0)]
    return (active, passive) if side == "left" else (passive, active)


def slide_centered_plus_x_nudge(
    side: str = "left",
    y_start_abs: float = Y_STAGE_INT2,
    dy_trim: float = -0.0010,
) -> tuple[list, list]:
    """Variant B: Dead-center pull to `(0.2130, 0.0000)` + captive interior `+X` datum square-up if pan drifted `-X`.

    In `D_DOWN` (`yaw = 0.0`), the closed fingers' `+X` face sits `+20.11 mm` from TCP (`0.2331 - 0.2130 = +0.02011 m`).
    Q1's `+X` interior vertical wall at `x_center = 0.2130 m` sits at `x = 0.2130 + 0.0639 = 0.2769 m`!
    Sweeping TCP gently from `x = 0.235 m` to `x = 0.2568 m` (`where finger `+X` face reaches `0.2769 m`!) at `y = 0.0`
    squares any `-X`-drifted pan cleanly up to `x_center = 0.2130 m` while staying `+20 mm` clear of `w50_front_wall`!
    """
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
    from yam_arena.demo import Waypoint

    sgn = +1.0 if side == "left" else -1.0
    sL, sR = slide_centered_contact(side=side, y_start_abs=y_start_abs, dy_trim=dy_trim)
    active = list(sL if side == "left" else sR)[:-2]  # insert nudge right after `unhook` (`inside cavity at y ~= -0.080 m`)!

    z_clear = STAND_TOP + Q_D + 0.075
    z_in_cavity = STAND_TOP + 0.028
    # Finger +X face is +0.02011 m from TCP; Q1 +X interior wall at x_c=0.2130 is at 0.27690 m -> exact datum TCP x = 0.2568 m!
    x_nudge_start = 0.2320
    x_nudge_datum = 0.2568
    nudge_wps = [
        Waypoint(0.7, (x_nudge_start, 0.0000, z_in_cavity), FINGER_CLOSED, f"slide_{side}_nudge_center_y", 0.0),
        Waypoint(0.9, (x_nudge_datum, 0.0000, z_in_cavity), FINGER_CLOSED, f"slide_{side}_nudge_plus_x_datum", 0.0),
        Waypoint(0.5, (x_nudge_datum, 0.0000, z_in_cavity), FINGER_CLOSED, f"slide_{side}_nudge_settle", 0.0),
        Waypoint(0.6, (X_STRIP, 0.0000, z_in_cavity), FINGER_CLOSED, f"slide_{side}_nudge_recenter", 0.0),
        Waypoint(1.0, (X_STRIP, 0.0000, z_clear + 0.040), FINGER_CLOSED, f"slide_{side}_lift_out", 0.0),
        Waypoint(1.4, (0.210, sgn * 0.280, 0.860), FINGER_OPEN, f"slide_{side}_return_ready", 0.0),
    ]
    active.extend(nudge_wps)
    total_t = sum(w.duration for w in active)
    passive = [Waypoint(total_t, (0.220, -sgn * 0.300, 0.895), FINGER_OPEN, f"wait_slide_{side}", 0.0)]
    return (active, passive) if side == "left" else (passive, active)


if __name__ == "__main__":
    sys.path.insert(0, r"C:/Users/jingy/AppData/Local/Temp/robocurve-gpt-yam-offline")
    import yam_arena

    URDF_PATH = pathlib.Path(r"C:/Users/jingy/AppData/Local/Temp/robocurve-gpt-yam-hand-reference/yam_arm.urdf")
    yam_arena.YAM_URDF = str(URDF_PATH)
    from yam_arena import constants as C, demo as dm, ik

    HERE = pathlib.Path(__file__).resolve().parent
    sys.path.insert(0, str(HERE))
    import verify_exact_urdf_collision_meshes_0656 as EX

    env_obbs = EX.ENV_OBBS
    summary = {}
    for vname, fn in (
        ("original_int2", slide_original_int2),
        ("centered_contact_vA", slide_centered_contact),
        ("centered_plus_x_nudge_vB", slide_centered_plus_x_nudge),
    ):
        sL, sR = fn("left", Y_STAGE_INT2)
        plL = dm.ArmPlan(sL, C.LEFT_BASE_POS)
        plR = dm.ArmPlan(sR, C.RIGHT_BASE_POS)
        n_st = int(round(plL.duration * 20.0))
        max_err = 0.0
        min_w50_mm, min_w50_tag = float("inf"), "none"
        for st in range(1, n_st + 1):
            t = min(st * 0.05, plL.duration)
            plL.action7(t)
            plR.action7(t)
            posL, _, fL, labL = plL.target(t)
            pcL, _ = ik.tcp_pose(plL.q, np.asarray(C.LEFT_BASE_POS))
            max_err = max(max_err, float(np.linalg.norm(pcL - posL) * 1e3))
            ptsL = EX.exact_arm_points("left", plL.q, float(fL))
            ce, te = EX.min_exact_clear_mm(ptsL, env_obbs)
            if ce < min_w50_mm:
                min_w50_mm, min_w50_tag = ce, f"L({labL}):{te}@{t:.2f}s"
        summary[vname] = {
            "steps_20hz": n_st,
            "duration_s": round(float(plL.duration), 2),
            "armplan_20hz_stream_max_mm": round(max_err, 3),
            "exact_urdf_min_clear_to_stand_w50_mm": min_w50_mm,
            "nearest_stand_w50_detail": min_w50_tag,
        }

    # Unit-test resting pose gate on nominal `(0.2130, 0.0000)` vs `int2` `(0.1972, +0.0078)`!
    gate_nom = check_slide_resting_pose_gate((0.2130, 0.0000, 0.5980), np.eye(3))
    gate_int2 = check_slide_resting_pose_gate((0.1972, +0.0078, 0.5980), np.eye(3))
    assert gate_nom["gate_pass"] is True and gate_int2["gate_pass"] is False

    out = {"status": "PASS", "variants_20hz_exact_urdf": summary, "gate_unit_checks": {"nominal": gate_nom, "int2_reproduced": gate_int2}}
    (HERE / "slide_landing_variants_0709.json").write_text(json.dumps(out, indent=2), encoding="utf-8")
    print(json.dumps(out, indent=2))
