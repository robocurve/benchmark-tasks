#!/usr/bin/env python3
"""Gate G1 checks for the game-box lid task:
1. Without the simulator (default): checks every preset in ``parts.BOX_PRESETS`` for 5-wall collider bounds, single-hand
   impossibility (> 94 mm), continuous IK tracking and joint margins through ``demo_kit.dry_run``, hand-to-hand and
   hand-to-bench/skirt collision hull clearances, and top-camera visibility at 640x360 and 320x176.
2. With ``--render-start <out_dir>`` (inside the simulator container via ``scripts/rta simpy``): resets ``task.py`` at
   seed 0, writes ``start_top.png``, ``start_top_policy.png`` (320x176), and wrist-camera start views, and steps 3.0 s
   at READY to verify static settling and zero false faults.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import demo_kit as kit  # noqa: E402
import parts as P  # noqa: E402

SIDES = ("left", "right")
HAND_LINKS = ("link4", "link5", "gripper", "tip_left", "tip_right")
TIP_LINKS = ("tip_left", "tip_right")
MAX_JAW = 0.094


def frame(closing, approach) -> np.ndarray:
    """Gripper rotation matrix from closing axis and approach axis (columns: closing, pads' width, approach)."""
    a = np.asarray(approach, dtype=float)
    a /= np.linalg.norm(a)
    c = np.asarray(closing, dtype=float) - np.dot(closing, a) * a
    c /= np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


_LOCAL_HAND_CACHE: dict[float, tuple[np.ndarray, np.ndarray]] = {}


def local_hand_points(opening: float = P.PADDLE_OPENING) -> tuple[np.ndarray, np.ndarray]:
    """(all hand hull vertices, fingertip hull vertices) in the gripper frame relative to TCP, cached by opening."""
    key = round(float(opening), 6)
    if key not in _LOCAL_HAND_CACHE:
        from yam_arena import analysis, constants as C

        joints = {**{n: float(v) for n, v in zip(C.ARM_JOINTS, C.READY)}, **dict.fromkeys(C.FINGER_JOINTS, -0.5 * key)}
        poses = analysis.fk_link_poses_world(joints, (0.0, 0.0, 0.0))
        g = poses["gripper"]
        to_tcp = lambda links: (np.vstack([h.vertices for _, h in analysis.world_hulls(poses, list(links))]) - g[:3, 3]) @ g[:3, :3] - np.array(C.TCP_OFFSET)  # noqa: E731
        _LOCAL_HAND_CACHE[key] = (to_tcp(("gripper", "tip_left", "tip_right")), to_tcp(TIP_LINKS))
    return _LOCAL_HAND_CACHE[key]


def grasp_geometry(spec: P.BoxSpec, side: str, pitch_deg: float = P.APPROACH_PITCH_DEG, opening: float = P.PADDLE_OPENING) -> dict:
    """Gripper rotation, inward contact point offset from TCP (world), and local ``push_at`` (gripper frame) for ``side``."""
    sgn = 1.0 if side == "left" else -1.0
    half_y = 0.5 * spec.lid_out_l
    # Roll symmetry of the 20 mm closed paddle keeps joint6 well inside [-120, +120] deg across narrow and wide boxes
    cx = sgn if half_y > 0.115 else -sgn
    th = math.radians(pitch_deg)
    approach = np.array([0.0, -sgn * math.cos(th), -math.sin(th)])
    rot = frame([cx, 0.0, 0.0], approach)
    all_local, _ = local_hand_points(opening)
    world_off = all_local @ rot.T
    inward_pt = world_off[int(np.argmin(sgn * world_off[:, 1]))]
    return {
        "sgn": sgn,
        "rot": rot,
        "inward_world_from_tcp": inward_pt,
        "push_at_local": rot.T @ inward_pt,
        "grip": round(opening / MAX_JAW, 4),
    }


def bimanual_plan(
    spec: P.BoxSpec,
    top_z: float,
    *,
    lid_xy: tuple[float, float] | None = None,
    base_xy: tuple[float, float] | None = None,
    squeeze_n: float = P.SQUEEZE_FORCE_N,
    carry_z: float = P.CARRY_CLEAR_Z,
    roll_deg: float = 0.0,
) -> dict[str, list[kit.Move]]:
    """Scripted two-arm opposed outer-wall squeeze, lift, carry, lower, release, and return to READY."""
    from yam_arena import constants as C, ik, station

    lx, ly = spec.lid_at if lid_xy is None else lid_xy
    bx, by = spec.base_at if base_xy is None else base_xy
    half_y = 0.5 * spec.lid_out_l
    z_touch = min(0.028, spec.lid_out_h - 0.008)
    roll = math.radians(roll_deg)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, math.cos(roll), -math.sin(roll)], [0.0, math.sin(roll), math.cos(roll)]])

    moves: dict[str, list[kit.Move]] = {}
    for side in SIDES:
        g = grasp_geometry(spec, side)
        sgn, rot, ip, pat, grip = g["sgn"], g["rot"], g["inward_world_from_tcp"], g["push_at_local"], g["grip"]
        rot_tilted = rx @ rot
        ip_tilted = rx @ ip
        r_pos, r_rot = ik.tcp_pose(np.array(C.READY, dtype=float), station.BASES[side])

        def tcp_level(x: float, y: float, z: float, _ip=ip) -> np.ndarray:
            return np.array([x, y, z], dtype=float) - _ip

        def tcp_carry(x: float, dz_bottom: float, _sgn=sgn, _ip_t=ip_tilted) -> np.ndarray:
            # Contact point on the lid wall rotated by roll_deg about lid bottom center (x, by, top_z + dz_bottom)
            wall_local = np.array([0.0, _sgn * half_y, z_touch])
            wall_world = np.array([x, by, top_z + dz_bottom]) + rx @ wall_local
            return wall_world - _ip_t

        push_vec = rx @ np.array([0.0, -sgn * squeeze_n, 0.0])
        moves[side] = [
            kit.Move(2.0, tcp_level(lx, ly + sgn * (half_y + 0.038), top_z + z_touch + 0.055), rot, grip, "approach"),
            kit.Move(1.2, tcp_level(lx, ly + sgn * (half_y + 0.038), top_z + z_touch), rot, grip, "align"),
            kit.Move(1.0, tcp_level(lx, ly + sgn * half_y, top_z + z_touch), rot, grip, "touch"),
            kit.Move(0.8, None, None, grip, "squeeze", push=tuple(push_vec) if roll_deg == 0.0 else (0.0, -sgn * squeeze_n, 0.0), push_at=pat),
            kit.Move(2.0, tcp_carry(lx, carry_z), rot_tilted, grip, "lift", push=tuple(push_vec)),
            kit.Move(2.6, tcp_carry(bx, carry_z), rot_tilted, grip, "carry", push=tuple(push_vec)),
            kit.Move(3.2, tcp_carry(bx, 0.0005), rot_tilted, grip, "seat", push=tuple(push_vec)),
            kit.Move(0.8, tcp_level(bx, by + sgn * (half_y + 0.050), top_z + z_touch + 0.008), rot, grip, "release", push=(0.0, 0.0, 0.0)),
            kit.Move(1.1, tcp_level(bx, by + sgn * (half_y + 0.055), top_z + 0.110), rot, 1.0, "up"),
            kit.Move(1.8, r_pos, r_rot, 1.0, "ready"),
        ]
    return moves


def shape_bounds_check(spec: P.BoxSpec) -> dict:
    """Verify 5-wall hollow base and lid shapes stay in z >= 0 and match outer dimensions."""
    b_lows, b_highs = zip(*(s.bounds() for s in P.base_shapes(spec)))
    l_lows, l_highs = zip(*(s.bounds() for s in P.lid_shapes(spec) if s.solid))
    b_lo, b_hi = np.min(b_lows, axis=0), np.max(b_highs, axis=0)
    l_lo, l_hi = np.min(l_lows, axis=0), np.max(l_highs, axis=0)
    assert np.allclose(b_hi - b_lo, [spec.base_w, spec.base_l, spec.base_h], atol=1e-6), (b_lo, b_hi)
    assert np.allclose(l_hi - l_lo, [spec.lid_out_w, spec.lid_out_l, spec.lid_out_h], atol=1e-6), (l_lo, l_hi)
    assert b_lo[2] >= -1e-9 and l_lo[2] >= -1e-9
    assert spec.lid_out_w > MAX_JAW and spec.lid_out_l > MAX_JAW
    return {
        "base_size_mm": (1000 * (b_hi - b_lo)).round(2).tolist(),
        "lid_solid_size_mm": (1000 * (l_hi - l_lo)).round(2).tolist(),
        "min_z_mm": [round(1000 * float(b_lo[2]), 4), round(1000 * float(l_lo[2]), 4)],
        "exceeds_single_jaw_94mm": True,
    }


def _hands_apart_m(hulls_a: list, hulls_b: list) -> float:
    """Minimum distance (m) between two arms' convex hulls using their bounding-box lower bounds and exact hull planes."""
    from yam_arena import analysis

    best = math.inf
    for _, ha in hulls_a:
        lo_a, hi_a = ha.vertices.min(0), ha.vertices.max(0)
        for _, hb in hulls_b:
            lo_b, hi_b = hb.vertices.min(0), hb.vertices.max(0)
            aabb_gap = float(np.linalg.norm(np.maximum(np.maximum(lo_b - hi_a, lo_a - hi_b), 0.0)))
            if aabb_gap >= best:
                continue
            signed = max(float(analysis._signed(ha.vertices, hb).min()), float(analysis._signed(hb.vertices, ha).min()))
            best = min(best, max(aabb_gap, signed))
    return best


def hull_clearances_for_preset(spec: P.BoxSpec, hz: float = 5.0) -> dict:
    """Single-pass continuous IK + joint-margin + URDF collision-hull check across the full trajectory."""
    from yam_arena import analysis, constants as C, ik, station
    from yam_arena.task_file import TABLE_TOP

    top_z = TABLE_TOP["station"]
    bases = {s: np.array(station.BASES[s], dtype=float) for s in SIDES}
    moves = bimanual_plan(spec, top_z)
    arms = {s: kit.Arm(s, bases[s], moves[s]) for s in SIDES}
    tight = {s: (math.inf, "") for s in SIDES}

    # Exact stage times to sample hulls (aligned to 1/hz = 0.20 s steps):
    # touch=4.20 s, lift=7.00 s, carry=9.60 s, seat=12.80 s, release=13.60 s, ready=16.40 s
    sample_times = {"touch": 4.20, "lift": 7.00, "carry": 9.60, "seat": 12.80, "release": 13.60, "ready": 16.40}
    min_hand_sep_mm = math.inf
    min_tcp_sep_mm = math.inf
    min_bench_clear_mm = math.inf
    min_tip_above_skirt_mm = math.inf
    max_skirt_intrusion_mm = -math.inf
    stage_records = {}

    total_s = max(arm.seconds for arm in arms.values())
    n_steps = int(round(total_s * hz)) + 1
    for i in range(n_steps):
        t = round(i / hz, 6)
        acts = {}
        pose_qs = {}
        for s, arm in arms.items():
            action, label = arm.action(t)
            acts[s] = action
            pose_qs[s] = arm.q.copy()  # actual pose joints at the wall surface (action[:6] adds spring press_offsets)
            margin = float(np.degrees(np.minimum(action[:6] - ik.LOWER, ik.UPPER - action[:6])).min())
            if margin < tight[s][0]:
                tight[s] = (margin, f"{label} at {t:.2f} s, joints {np.degrees(action[:6]).round(0).astype(int).tolist()}")

        for name, ts in sample_times.items():
            if abs(t - ts) < 1e-6:
                hulls_hand = {}
                verts_all = {}
                verts_tip = {}
                tcps = {}
                for s in SIDES:
                    q = pose_qs[s]
                    opening = float(acts[s][6]) * MAX_JAW
                    joints = {**{n: float(v) for n, v in zip(C.ARM_JOINTS, q)}, **dict.fromkeys(C.FINGER_JOINTS, -0.5 * opening)}
                    poses = analysis.fk_link_poses_world(joints, tuple(bases[s]))
                    hulls_hand[s] = analysis.world_hulls(poses, list(HAND_LINKS))
                    verts_all[s] = np.vstack([h.vertices for _, h in hulls_hand[s]])
                    verts_tip[s] = np.vstack([h.vertices for _, h in analysis.world_hulls(poses, list(TIP_LINKS))])
                    tcps[s] = arms[s].state(t)[0]

                tcp_sep = float(np.linalg.norm(tcps["left"] - tcps["right"])) * 1000.0
                hand_sep = _hands_apart_m(hulls_hand["left"], hulls_hand["right"]) * 1000.0
                bench_clear = min(float(verts_all[s][:, 2].min() - top_z) for s in SIDES) * 1000.0
                lid_z0 = top_z + (P.CARRY_CLEAR_Z if name in ("lift", "carry") else 0.0)
                tip_above_bottom = min(float(verts_tip[s][:, 2].min() - lid_z0) for s in SIDES) * 1000.0
                if name in ("touch", "lift", "carry", "seat"):
                    min_hand_sep_mm = min(min_hand_sep_mm, hand_sep)
                    min_tcp_sep_mm = min(min_tcp_sep_mm, tcp_sep)
                    min_tip_above_skirt_mm = min(min_tip_above_skirt_mm, tip_above_bottom)
                    for s, sgn in (("left", 1.0), ("right", -1.0)):
                        near_lid_z = verts_all[s][verts_all[s][:, 2] <= lid_z0 + spec.lid_out_h + 0.010]
                        if len(near_lid_z):
                            intrusion = float((0.5 * spec.lid_out_l - sgn * near_lid_z[:, 1]).max()) * 1000.0
                            max_skirt_intrusion_mm = max(max_skirt_intrusion_mm, intrusion)
                min_bench_clear_mm = min(min_bench_clear_mm, bench_clear)
                stage_records[name] = {
                    "tcp_sep_mm": round(tcp_sep, 1),
                    "hand_hull_sep_mm": round(hand_sep, 1),
                    "bench_clear_mm": round(bench_clear, 1),
                    "tip_above_lid_bottom_mm": round(tip_above_bottom, 1),
                }

    dry = {
        s: {
            "seconds": round(arms[s].seconds, 2),
            "worst_ik_mm": round(arms[s].worst_mm, 2),
            "worst_ik_deg": round(arms[s].worst_deg, 2),
            "smallest_margin_deg": round(tight[s][0], 1),
            "there": tight[s][1],
            "push_effort": round(arms[s].worst_effort, 2),
        }
        for s in SIDES
    }

    for s in SIDES:
        assert dry[s]["worst_ik_mm"] <= 0.5, (spec.key, s, dry[s])
        assert dry[s]["smallest_margin_deg"] >= 5.0, (spec.key, s, dry[s])
        assert dry[s]["push_effort"] <= 1.0, (spec.key, s, dry[s])
    assert min_tcp_sep_mm >= 200.0, (spec.key, min_tcp_sep_mm)
    assert min_bench_clear_mm >= 10.0, (spec.key, min_bench_clear_mm)
    assert min_tip_above_skirt_mm >= 10.0, (spec.key, min_tip_above_skirt_mm)
    assert max_skirt_intrusion_mm <= 0.5, (spec.key, max_skirt_intrusion_mm)

    return {
        "dry_run": dry,
        "min_tcp_sep_mm": round(min_tcp_sep_mm, 1),
        "min_hand_hull_sep_mm": round(min_hand_sep_mm, 1),
        "min_bench_clear_mm": round(min_bench_clear_mm, 1),
        "min_tip_above_lid_bottom_mm": round(min_tip_above_skirt_mm, 1),
        "max_wall_intrusion_mm": round(max_skirt_intrusion_mm, 3),
        "stages": stage_records,
    }


def top_camera_visibility(spec: P.BoxSpec) -> dict:
    """Verify lid, base, contents, and corner marks lie inside top_cam (640x360 and 320x176) clear of READY arms."""
    from yam_arena import analysis, constants as C, station
    from yam_arena.task_file import TABLE_TOP
    from yam_arena.urdf import quat_xyzw_to_matrix

    top_z = TABLE_TOP["station"]
    cam = station.top_camera("calibrated")
    c_pos = np.array(cam.pos, dtype=float)
    c_rot = quat_xyzw_to_matrix(np.array(cam.rot_xyzw, dtype=float))
    lens = cam.lens

    def proj(pts: np.ndarray) -> np.ndarray:
        c = (pts - c_pos) @ c_rot
        u = lens.cx + lens.fx * c[:, 0] / -c[:, 2]
        v = lens.cy - lens.fy * c[:, 1] / -c[:, 2]
        return np.c_[u, v]

    def box_corners(cx: float, cy: float, w: float, length: float, h: float) -> np.ndarray:
        return np.array([[cx + sx * 0.5 * w, cy + sy * 0.5 * length, top_z + sz * h] for sx in (-1, 1) for sy in (-1, 1) for sz in (0, 1)])

    lid_uv = proj(box_corners(spec.lid_at[0], spec.lid_at[1], spec.lid_out_w, spec.lid_out_l, spec.lid_out_h))
    base_uv = proj(box_corners(spec.base_at[0], spec.base_at[1], spec.base_w, spec.base_l, spec.base_h))
    mark_w = spec.base_w + 2 * (P.MARK_CLEAR + P.MARK_W)
    mark_l = spec.base_l + 2 * (P.MARK_CLEAR + P.MARK_W)
    mark_uv = proj(box_corners(spec.base_at[0], spec.base_at[1], mark_w, mark_l, P.MARK_T))

    arm_uv = {}
    for s in SIDES:
        joints = {**{n: float(v) for n, v in zip(C.ARM_JOINTS, C.READY)}, **dict.fromkeys(C.FINGER_JOINTS, C.FINGER_OPEN)}
        poses = analysis.fk_link_poses_world(joints, station.BASES[s])
        verts = np.vstack([h.vertices for _, h in analysis.world_hulls(poses, list(C.LINKS))])
        arm_uv[s] = proj(verts)

    scene_u_min = float(min(lid_uv[:, 0].min(), base_uv[:, 0].min(), mark_uv[:, 0].min()))
    scene_u_max = float(max(lid_uv[:, 0].max(), base_uv[:, 0].max(), mark_uv[:, 0].max()))
    left_u_max = float(arm_uv["left"][:, 0].max())
    right_u_min = float(arm_uv["right"][:, 0].min())
    assert 0.0 <= scene_u_min and scene_u_max <= 640.0
    assert left_u_max < scene_u_min and scene_u_max < right_u_min
    return {
        "lid_bbox_640x360": [round(float(lid_uv[:, 0].min()), 1), round(float(lid_uv[:, 1].min()), 1), round(float(lid_uv[:, 0].max()), 1), round(float(lid_uv[:, 1].max()), 1)],
        "base_bbox_640x360": [round(float(base_uv[:, 0].min()), 1), round(float(base_uv[:, 1].min()), 1), round(float(base_uv[:, 0].max()), 1), round(float(base_uv[:, 1].max()), 1)],
        "lid_span_320x176_px": [round(0.5 * float(lid_uv[:, 0].max() - lid_uv[:, 0].min()), 1), round((176.0 / 360.0) * float(lid_uv[:, 1].max() - lid_uv[:, 1].min()), 1)],
        "base_span_320x176_px": [round(0.5 * float(base_uv[:, 0].max() - base_uv[:, 0].min()), 1), round((176.0 / 360.0) * float(base_uv[:, 1].max() - base_uv[:, 1].min()), 1)],
        "clear_of_left_arm_px": round(scene_u_min - left_u_max, 1),
        "clear_of_right_arm_px": round(right_u_min - scene_u_max, 1),
    }


def render_start(out_dir: Path) -> dict:
    """Inside the simulator container: save frame-0 camera pictures + 320x176 policy crops and verify 3 s static rest."""
    import cv2
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena.constants import READY

    out_dir.mkdir(parents=True, exist_ok=True)
    emb = kit.open_task(HERE / "task.py")
    m = emb.task.module
    obs = emb.reset(Scene(id="g1_start", instruction=m.INSTRUCTION), seed=0)
    names = {"top_cam": "start_top", "left_cam": "start_left_wrist", "right_cam": "start_right_wrist"}
    for cam, stem in names.items():
        rgb = obs.images[cam]
        cv2.imwrite(str(out_dir / f"{stem}.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        pol = kit.policy_picture(rgb)
        cv2.imwrite(str(out_dir / f"{stem}_policy.png"), cv2.cvtColor(pol, cv2.COLOR_RGB2BGR))

    ready_act = Action(data=pack(np.r_[READY, 1.0], np.r_[READY, 1.0]))
    first_poses = None
    res = None
    for i in range(60):
        res = emb.step(ready_act)
        if i == 0:
            first_poses = {b: m.mk.pose(b)[0].copy() for b in m.FREE_BODIES}
    last_poses = {b: m.mk.pose(b)[0].copy() for b in m.FREE_BODIES}
    drift_mm = {b: round(1000.0 * float(np.linalg.norm(last_poses[b] - first_poses[b])), 4) for b in m.FREE_BODIES}
    summary = {
        "score_after_3s_idle": float(res.info["score"]),
        "drift_3s_mm": drift_mm,
        "report": m.report(),
    }
    assert summary["score_after_3s_idle"] == 0.0
    assert m.STATE["fault"] is None
    assert max(drift_mm.values()) < 1.0, drift_mm
    (out_dir / "start_check.json").write_text(json.dumps(summary, indent=2))
    print("START_CHECK " + json.dumps(summary), flush=True)
    return summary


def run_offline() -> dict:
    out = {}
    for key, spec in P.BOX_PRESETS.items():
        out[key] = {
            "summary": P.preset_summary(spec),
            "shapes": shape_bounds_check(spec),
            "ik_and_hulls": hull_clearances_for_preset(spec),
            "top_cam": top_camera_visibility(spec),
        }
    return out


def main(argv: list[str] | None = None) -> int:
    args = sys.argv[1:] if argv is None else argv
    if len(args) >= 2 and args[0] == "--render-start":
        render_start(Path(args[1]))
        return 0
    result = run_offline()
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
