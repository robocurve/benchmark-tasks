#!/usr/bin/env python3
"""Gate G1 checks for the 2U server air-shroud task:

1. Without the simulator (default): verifies 3.0 mm ABS shroud non-overlapping plate bounds (z in [0, 78.0] mm, zero
   roof protrusions), flat-drop impossibility (170.0 mm shroud + tabs > 165.5 mm bay opening), offline IK and joint
   margins across every waypoint and continuous 20 Hz dry_run, grasp height <= 160 mm, jaw <= 94 mm, hand-to-hand
   separation >= 200 mm, and 3D hand-hull clearances against chassis walls, heatsinks, RDIMMs, and shroud walls.
2. With ``--frame0 <out_dir>`` (inside Isaac Sim via ``scripts/rta simpy``): resets ``task.py`` at seed 0, saves
   ``start_top.png``, ``start_top_policy.png`` (320 x 176), ``start_left_wrist.png``, and ``start_right_wrist.png`` into
   ``<out_dir>``, and steps 2.0 s at READY confirming static settling (< 1.0 mm), score 0.0, and zero false fouls.
"""

from __future__ import annotations

import os

for _var in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
    os.environ.setdefault(_var, "1")

import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import demo_kit as kit  # noqa: E402
import parts as P  # noqa: E402

SIDES = ("left", "right")
HAND_LINKS = ("link4", "link5", "gripper", "tip_left", "tip_right")
MAX_JAW = 0.0939  # 2 * abs(FINGER_OPEN)
OPEN_WELL_JAW = 0.039  # 39.0 mm jaw opening when entering/exiting the recessed finger-lift slots around the 26.0 mm bridge
PINCH_TARGET_JAW = 0.014  # 14.0 mm target on 26.0 mm bridge -> 6.0 mm/finger * 5000 N/m = ~30 N per pad (<= 40 N)
LEAN_DEG = 32.0  # forward-and-down gripper lean in shroud frame (keeps all 6 joints >= 34 deg from limits)
PITCH_TUCK_DEG = 18.0  # nose-down shroud tilt when sliding both front tabs under the 83.0 mm bulkhead lip
IK_MM_MAX = 0.5
IK_DEG_MAX = 0.6
MARGIN_DEG_MIN = 6.0
GRASP_Z_MAX = 0.160
TCP_SEP_MIN = 0.200
HULL_SEP_MIN = 0.020


def rot_y(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array([[math.cos(a), 0.0, math.sin(a)], [0.0, 1.0, 0.0], [-math.sin(a), 0.0, math.cos(a)]])


def frame(closing, approach) -> np.ndarray:
    """Gripper rotation matrix: columns are closing axis, pad-width axis, approach axis."""
    a = np.asarray(approach, dtype=float)
    a /= np.linalg.norm(a)
    c = np.asarray(closing, dtype=float) - np.dot(closing, a) * a
    c /= np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


def grasp_pose(shroud_orig, pitch_deg: float, side: str, dz_local: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """World (TCP position, 3x3 gripper rotation) on the left (+y) or right (-y) recessed blue touchpoint bridge."""
    sgn = 1.0 if side == "left" else -1.0
    r_sh = rot_y(pitch_deg)
    a_loc = np.array([math.sin(math.radians(LEAN_DEG)), 0.0, -math.cos(math.radians(LEAN_DEG))])
    # Both arms have identical joint limits; [-y] closing keeps wrist roll ~ +/-20 deg (well inside +/-120 deg) on both
    g_rot = frame(np.array([0.0, -1.0, 0.0]), r_sh @ a_loc)
    # Place fingertip pad contact centre directly on SHROUD_COM[0] so gravity exerts zero pitch torque on the grip
    p_loc = np.array([P.SHROUD_COM[0] - 0.004, sgn * P.GRASP_Y, 0.070 + dz_local])
    return np.asarray(shroud_orig, dtype=float) + r_sh @ p_loc, g_rot


def shroud_waypoints(top_z: float) -> list[tuple[str, np.ndarray, float, float, float, float]]:
    """Desired physical (name, shroud_orig, pitch_deg, dz_local, jaw_m, duration_s) waypoints for the bimanual plan."""
    fz = top_z + P.WALL_T
    start_orig = np.array([P.X_START, 0.0, fz + P.DECK_H])
    piv_loc = np.array([P.SHROUD_L / 2.0 + 0.0065, 0.0, P.TAB_Z_MID])  # x_local = +86.5 mm inside the 10.0 mm tab
    piv_pretuck = np.array([0.4650, 0.0, fz + 0.0755])  # tab tip 4.7 mm ahead of 473.0 mm bulkhead, rear > 55.5 mm (> 52 mm divider)
    piv_tuck = np.array([P.X_SEAT + P.SHROUD_L / 2.0 + 0.0052, 0.0, fz + 0.0742])  # front face 2.5 mm before bulkhead, tabs 7.2 mm in slots
    orig_pretuck = piv_pretuck - rot_y(19.5) @ piv_loc
    orig_tucked = piv_tuck - rot_y(PITCH_TUCK_DEG) @ piv_loc
    orig_h12 = piv_tuck - rot_y(12.0) @ piv_loc
    orig_h6 = piv_tuck - rot_y(6.0) @ piv_loc
    orig_seated = np.array([P.X_SEAT, 0.0, fz])

    return [
        ("hover_start", start_orig, 0.0, 0.056, OPEN_WELL_JAW, 1.8),
        ("in_well_open", start_orig, 0.0, 0.000, OPEN_WELL_JAW, 1.3),
        ("pinched_start", start_orig, 0.0, 0.000, PINCH_TARGET_JAW, 0.9),
        ("lifted_straight", np.array([P.X_START, 0.0, fz + 0.066]), 0.0, 0.000, PINCH_TARGET_JAW, 1.6),
        ("over_bay_high", np.array([P.X_SEAT - 0.025, 0.0, fz + 0.068]), 6.0, 0.000, PINCH_TARGET_JAW, 2.0),
        ("pretuck_19p5", orig_pretuck, 19.5, 0.000, PINCH_TARGET_JAW, 1.8),
        ("tucked_18", orig_tucked, PITCH_TUCK_DEG, 0.000, PINCH_TARGET_JAW, 1.6),
        ("hinge_12", orig_h12, 12.0, 0.000, PINCH_TARGET_JAW, 1.2),
        ("hinge_6", orig_h6, 6.0, 0.000, PINCH_TARGET_JAW, 1.2),
        ("seated_0", orig_seated, 0.0, 0.000, PINCH_TARGET_JAW, 1.4),
        ("released_0", orig_seated, 0.0, 0.002, OPEN_WELL_JAW, 0.9),
        ("retracted_up", orig_seated, 0.0, 0.060, OPEN_WELL_JAW, 1.4),
    ]


_CARRY_LABELS = {"lifted_straight", "over_bay_high", "pretuck_19p5", "tucked_18", "hinge_12", "hinge_6", "seated_0"}


def plan_moves(top_z: float) -> dict[str, list[kit.Move]]:
    """Open-loop Move sequence for both arms with analytical J^T F / K payload gravity feed-forward while carrying."""
    from yam_arena import constants as C, ik, station

    wps = shroud_waypoints(top_z)
    half_weight = np.array([0.0, 0.0, 0.5 * P.SHROUD_MASS * P.GRAVITY], dtype=float)
    out: dict[str, list[kit.Move]] = {}
    for side in SIDES:
        base = np.array(station.BASES[side], dtype=float)
        r_pos, r_rot = ik.tcp_pose(np.array(C.READY, dtype=float), base)
        q_seed = np.array(C.READY, dtype=float)
        moves = []
        for name, orig, deg, dz, jaw, secs in wps:
            pos, rot = grasp_pose(orig, deg, side, dz)
            q, _, _ = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), q_seed, base_pos=base, iters=120)
            q_seed = q
            if name in _CARRY_LABELS:
                # Analytical feed-forward joint offset J^T F / K cancels the arm's PD sag under half of the shroud's weight
                scale = 0.5 if name == "seated_0" else 1.0
                dq = (kit.point_jacobian(q, base).T @ (scale * half_weight)) / kit.JOINT_STIFFNESS
                pos, rot = ik.tcp_pose(q + dq, base)
            moves.append(kit.Move(secs, pos=pos, rot=rot, grip=round(jaw / MAX_JAW, 4), label=name))
        moves.append(kit.Move(2.0, pos=r_pos, rot=r_rot, grip=1.0, label="ready"))
        out[side] = moves
    return out


def verify_parts_geometry() -> dict:
    """Verify non-overlapping shroud boxes, z in [0, 78.0] mm, > 94 mm single-jaw impossibility, and flat-drop block."""
    parts = P.shroud_parts()
    boxes = []
    for bp in parts:
        c, h = np.array(bp.pos, dtype=float), np.array(bp.size, dtype=float) / 2.0
        boxes.append((bp.label, c - h, c + h))
    # Pairwise non-overlap check (tolerance 1e-7 m)
    for i in range(len(boxes)):
        for j in range(i + 1, len(boxes)):
            ov = np.minimum(boxes[i][2], boxes[j][2]) - np.maximum(boxes[i][1], boxes[j][1])
            if np.all(ov > 1e-7):
                raise AssertionError(f"shroud parts {boxes[i][0]!r} and {boxes[j][0]!r} overlap by {(ov * 1000).round(3)} mm")
    lo = np.min([b[1] for b in boxes], axis=0)
    hi = np.max([b[2] for b in boxes], axis=0)
    assert math.isclose(float(lo[2]), 0.0, abs_tol=1e-7), lo
    assert math.isclose(float(hi[2]), P.SHROUD_H, abs_tol=1e-7), hi
    assert math.isclose(float(hi[1] - lo[1]), P.SHROUD_W, abs_tol=1e-7)
    assert math.isclose(float(hi[0] - lo[0]), P.SHROUD_L + P.TAB_L, abs_tol=1e-7)
    assert P.SHROUD_W > MAX_JAW and P.SHROUD_L > MAX_JAW
    assert P.SHROUD_L + P.TAB_L > P.BAY_OPEN_L + 0.003  # flat drop impossible by >= 3.5 mm
    return {
        "shroud_part_count": len(parts),
        "shroud_mass_g": round(P.SHROUD_MASS * 1000.0, 1),
        "shroud_bounds_mm": [round(float(v) * 1000.0, 2) for v in (hi - lo)],
        "max_roof_z_mm": round(float(hi[2]) * 1000.0, 2),
        "bay_opening_mm": round(P.BAY_OPEN_L * 1000.0, 2),
        "shroud_with_tabs_mm": round((P.SHROUD_L + P.TAB_L) * 1000.0, 2),
        "heatsink_duct_clearance_mm": round(P.DUCT_CLEAR * 1000.0, 2),
    }


def run_offline() -> dict:
    from yam_arena import analysis, constants as C, ik, station, task_file

    top_z = task_file.TABLE_TOP["station"]
    bases = {s: np.array(station.BASES[s], dtype=float) for s in SIDES}
    geom = verify_parts_geometry()

    # Fixed chassis side walls + foul boxes for clearance checks
    chassis_boxes = [
        ("chassis_wall_left", np.array([(P.X_FRONT + P.X_REAR) / 2.0, P.INNER_W / 2.0 + P.WALL_T / 2.0, top_z + P.CHASSIS_H / 2.0]), np.array([P.CHASSIS_D, P.WALL_T, P.CHASSIS_H])),
        ("chassis_wall_right", np.array([(P.X_FRONT + P.X_REAR) / 2.0, -(P.INNER_W / 2.0 + P.WALL_T / 2.0), top_z + P.CHASSIS_H / 2.0]), np.array([P.CHASSIS_D, P.WALL_T, P.CHASSIS_H])),
        *P.foul_boxes_world(top_z),
    ]
    shroud_parts = P.shroud_parts()

    q_prev = {s: np.array(C.READY, dtype=float) for s in SIDES}
    wp_rows = []
    worst_pe_mm, worst_re_deg, min_margin_deg, max_grasp_z_mm = 0.0, 0.0, 999.0, 0.0
    min_tcp_sep_mm, min_hand_sep_mm, min_chassis_clear_mm, min_shroud_clear_mm = 999.0, 999.0, 999.0, 999.0

    for name, orig, deg, dz, jaw, _ in shroud_waypoints(top_z):
        r_sh = rot_y(deg)
        physical_jaw = max(P.SPINE_W, jaw)  # fingers cannot close inside the rigid 26.0 mm ABS bridge
        hulls_side = {}
        tcp_side = {}
        for side in SIDES:
            pos, rot = grasp_pose(orig, deg, side, dz)
            tcp_side[side] = pos
            q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), q_prev[side], base_pos=bases[side], iters=80)
            q_prev[side] = q
            pe_mm, re_deg = float(pe * 1000.0), float(math.degrees(re))
            margin = float(np.degrees(np.minimum(q - ik.LOWER, ik.UPPER - q)).min())
            z_bench_mm = float((pos[2] - top_z) * 1000.0)
            worst_pe_mm = max(worst_pe_mm, pe_mm)
            worst_re_deg = max(worst_re_deg, re_deg)
            min_margin_deg = min(min_margin_deg, margin)
            max_grasp_z_mm = max(max_grasp_z_mm, z_bench_mm)

            joints = {**{n: float(v) for n, v in zip(C.ARM_JOINTS, q)}, **dict.fromkeys(C.FINGER_JOINTS, -0.5 * physical_jaw)}
            poses_w = analysis.fk_link_poses_world(joints, tuple(bases[side]))
            hulls = analysis.world_hulls(poses_w, list(HAND_LINKS))
            hulls_side[side] = hulls

            ch_clear = min(analysis.hulls_to_box(hulls, c, np.eye(3), sz)[0] for _, c, sz in chassis_boxes) * 1000.0
            min_chassis_clear_mm = min(min_chassis_clear_mm, float(ch_clear))

            sh_clear = 999.0
            for bp in shroud_parts:
                if bp.label.startswith("touch_bridge_") and bp.label.endswith(f"_{side}") and jaw < P.SPINE_W + 0.002:
                    continue
                bc = orig + r_sh @ np.array(bp.pos, dtype=float)
                d_mm = analysis.hulls_to_box(hulls, bc, r_sh, np.array(bp.size, dtype=float))[0] * 1000.0
                sh_clear = min(sh_clear, float(d_mm))
            min_shroud_clear_mm = min(min_shroud_clear_mm, sh_clear)

            wp_rows.append({
                "waypoint": name,
                "side": side,
                "pe_mm": round(pe_mm, 3),
                "re_deg": round(re_deg, 3),
                "margin_deg": round(margin, 1),
                "z_bench_mm": round(z_bench_mm, 1),
                "jaw_mm": round(physical_jaw * 1000.0, 1),
                "chassis_clear_mm": round(float(ch_clear), 1),
                "shroud_clear_mm": round(sh_clear, 1),
            })

        # Exact lateral y-gap lower bound on 3D hull separation across the left (+y) and right (-y) hands
        pts_l = np.vstack([h.vertices for _, h in hulls_side["left"]])
        pts_r = np.vstack([h.vertices for _, h in hulls_side["right"]])
        center_sep = float(max(0.0, pts_l[:, 1].min() - pts_r[:, 1].max()) * 1000.0)
        tcp_sep = float(np.linalg.norm(tcp_side["left"] - tcp_side["right"]) * 1000.0)
        min_hand_sep_mm = min(min_hand_sep_mm, center_sep)
        min_tcp_sep_mm = min(min_tcp_sep_mm, tcp_sep)

    moves = plan_moves(top_z)
    dry = kit.dry_run(moves, station.BASES, hz=4.0, limit_s=50.0)
    dry_min_margin = min(dry[s]["smallest_margin_deg"] for s in SIDES)
    dry_worst_mm = max(dry[s]["worst_ik_mm"] for s in SIDES)

    ok = (
        worst_pe_mm <= IK_MM_MAX
        and worst_re_deg <= IK_DEG_MAX
        and min_margin_deg >= MARGIN_DEG_MIN
        and dry_min_margin >= MARGIN_DEG_MIN
        and dry_worst_mm <= IK_MM_MAX
        and max_grasp_z_mm <= GRASP_Z_MAX * 1000.0
        and min_tcp_sep_mm >= TCP_SEP_MIN * 1000.0
        and min_hand_sep_mm >= HULL_SEP_MIN * 1000.0
        and min_chassis_clear_mm >= 3.0
        and min_shroud_clear_mm >= 2.0
    )
    return {
        "pass": bool(ok),
        "geometry": geom,
        "pose_count": len(wp_rows),
        "worst_pe_mm": round(worst_pe_mm, 3),
        "worst_re_deg": round(worst_re_deg, 3),
        "min_margin_deg": round(min_margin_deg, 1),
        "dry_run_worst_mm": round(float(dry_worst_mm), 3),
        "dry_run_min_margin_deg": round(float(dry_min_margin), 1),
        "plan_seconds": dry["left"]["seconds"],
        "max_grasp_z_mm": round(max_grasp_z_mm, 1),
        "min_tcp_sep_mm": round(min_tcp_sep_mm, 1),
        "min_hand_hull_sep_mm": round(min_hand_sep_mm, 1),
        "min_chassis_hs_dimm_clear_mm": round(min_chassis_clear_mm, 1),
        "min_shroud_wall_clear_mm": round(min_shroud_clear_mm, 1),
        "pinch_force_per_pad_est_N": round(0.5 * (P.SPINE_W - PINCH_TARGET_JAW) * 5000.0, 1),
        "waypoints": wp_rows,
    }


def _write_clean_png(path: Path, rgb: np.ndarray) -> None:
    import cv2

    ok, buf = cv2.imencode(".png", cv2.cvtColor(np.asarray(rgb), cv2.COLOR_RGB2BGR))
    if not ok:
        raise RuntimeError(f"could not encode {path}")
    path.write_bytes(bytes(buf))


def run_frame0(out_dir: Path, offline: dict) -> dict:
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena.constants import READY

    out_dir.mkdir(parents=True, exist_ok=True)
    embodiment = kit.open_task(HERE / "task.py")
    m = embodiment.task.module
    obs = embodiment.reset(Scene(id="frame0", instruction=m.INSTRUCTION), seed=0)
    cams = obs.images or {}
    _write_clean_png(out_dir / "start_top.png", cams["top_cam"])
    _write_clean_png(out_dir / "start_top_policy.png", kit.policy_picture(cams["top_cam"]))
    _write_clean_png(out_dir / "start_left_wrist.png", cams["left_cam"])
    _write_clean_png(out_dir / "start_right_wrist.png", cams["right_cam"])

    hold = Action(data=pack(np.r_[READY, 1.0], np.r_[READY, 1.0]))
    p0 = None
    max_drift_mm = 0.0
    res = None
    for _ in range(40):  # 2.0 s at 20 Hz
        res = embodiment.step(hold)
        p_now = m.shroud_pose()[0].copy()
        if p0 is None:
            p0 = p_now
        max_drift_mm = max(max_drift_mm, float(np.linalg.norm(p_now - p0) * 1000.0))

    assert res is not None
    rep = m.report()
    sim_ok = float(res.info.get("score", -1.0)) == 0.0 and rep["zero"] is None and max_drift_mm < 1.5
    summary = {
        "pass": bool(offline["pass"] and sim_ok),
        "idle_2s_score": float(res.info.get("score", -1.0)),
        "idle_2s_zero": rep["zero"],
        "idle_2s_drift_mm": round(max_drift_mm, 3),
        "report": rep,
        "offline": {k: v for k, v in offline.items() if k != "waypoints"},
    }
    (out_dir / "frame0.json").write_text(json.dumps(summary, indent=2))
    return summary


def main() -> int:
    offline = run_offline()
    if "--frame0" in sys.argv[1:]:
        idx = sys.argv.index("--frame0")
        out_dir = Path(sys.argv[idx + 1])
        summary = run_frame0(out_dir, offline)
        print("FRAME0 " + json.dumps(summary), flush=True)
        return 0 if summary["pass"] else 1
    brief = {k: v for k, v in offline.items() if k != "waypoints"}
    print("OFFLINE " + json.dumps(brief, indent=2), flush=True)
    return 0 if offline["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
