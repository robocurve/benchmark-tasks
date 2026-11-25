"""Isolated Multi-Pan (`2-Pan Near` & Intact `<= 2,400-Step 4-Pan`) Runner + Portable Native Body-Transform Named-Pair Logger.

Addresses `20261121-0907-codex-w50-audit-readback-integrated-short-case.md`:
1. `load_urdf_link_clouds_portable()`: loads exact `<collision>` / `<visual>` link STL point clouds directly from `yam_arena.YAM_URDF`
   on Linux 5090 (falling back cleanly to local Windows URDF/mesh caches for offline verification — zero hardcoded Windows paths!).
2. `RuntimeNamedPairContactLogger`: consumes **actual simulator body transforms** (`_rt.link_poses(rob)[lnk]` minus `env_origin`, zero IK!)
   and actual `PartState` transforms (`BAR, quarter_1, quarter_2, ...`), retaining exact `(arm:link -> obj:subpart@phase)` pair names
   for `housing_min_mm`, `unintended_distal_min_mm`, `distal_to_occ_min_mm`, `pan_to_occ_min_mm`, and `intentional_distal_min_mm`.
3. `demo_2pan_near_guarded()` (`1,984` steps = `99.2 s <= 2,400`): continuous undeformed sequence in ONE scene:
   `Bar (0.213, 0.0 -> W50 rims)` -> `Q1 (+0.4675 -> 0.185 slide -> Guard 1 -> Pitched-A -> unmodified stock c7i Q1 pickup & seat)`
   -> `Q2 (-0.4675 -> 0.185 slide -> Guard 2 -> Left Pitched-A -> Q2 pickup, occupied descent & straight-up release)` -> `park_still`.
4. `demo_4pan_2400step_bimanual()` (`2,336` steps = `116.8 s <= 2,400`): intact 4-pan source preserved separately.
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
RIM = 0.8261
X_STRIP = 0.2130
X_NEAR_2PAN = 0.4010      # exact stock c7i X_NEAR + 0.0030
X_NEAR_4PAN = 0.3990      # 4-pan shared-bar X target (+2.60 mm flange gap to Far_Q1/Q2)
X_FAR_SEAT = 0.5625
Y_POS_SEAT = +0.1270      # exact stock c7i WC[1] + OPEN_Y / 4 = +0.1270 m (settled +0.1325 m in c7i)
Y_NEG_SEAT = -0.1352      # +4.2 mm flange gap to c7i's settled Q1 (+0.1325 m) & +1.75 mm to -Y end wall

Q_W, Q_L, Q_D, Q_FL, SHEET = 0.1618, 0.2635, 0.0635, 0.0150, 0.0020
BODY_W, BODY_L = Q_W - 2.0 * Q_FL, Q_L - 2.0 * Q_FL
BODY_HALF_X = BODY_W / 2.0

HOUSING_LINKS = ("link1", "link2", "link3", "link4", "link5", "gripper")
DISTAL_LINKS = ("tip_left", "tip_right")
ALL_LINKS = HOUSING_LINKS + DISTAL_LINKS


def _rpy_to_R(rpy) -> np.ndarray:
    r, p, y = float(rpy[0]), float(rpy[1]), float(rpy[2])
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], float)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], float)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], float)
    return Rz @ Ry @ Rx


def _read_stl_pts(p: pathlib.Path, dense_tip: bool = False) -> np.ndarray:
    data = p.read_bytes()
    if data[:5] == b"solid" and b"facet" in data[:400]:
        vs = [
            [float(x) for x in ln.split()[1:4]]
            for ln in data.decode(errors="ignore").splitlines()
            if ln.strip().startswith("vertex")
        ]
        tris = np.asarray(vs, float).reshape(-1, 3, 3)
    else:
        n_tri = struct.unpack_from("<I", data, 80)[0]
        dt = np.dtype([("n", "<3f4"), ("v", "<9f4"), ("attr", "<u2")])
        tris = np.frombuffer(data, dt, count=n_tri, offset=84)["v"].reshape(-1, 3, 3).astype(np.float64)
    if not dense_tip:
        return np.unique(np.round(tris.reshape(-1, 3), 4), axis=0)
    bary = [(1.0 - i / 4 - j / 4, i / 4, j / 4) for i in range(5) for j in range(5 - i)]
    B = np.asarray(bary, float)
    pts = np.einsum("bk,tkd->tbd", B, tris).reshape(-1, 3)
    return np.unique(np.round(pts, 3), axis=0)


_CACHED_PORTABLE_CLOUDS: dict[str, np.ndarray] | None = None


def load_urdf_link_clouds_portable(max_pts_per_link: int = 900) -> dict[str, np.ndarray]:
    """Portable 8-link URDF collision/mesh loader for both Linux 5090 native runs (`yam_arena.YAM_URDF`) and offline CPU."""
    global _CACHED_PORTABLE_CLOUDS
    if _CACHED_PORTABLE_CLOUDS is not None:
        return _CACHED_PORTABLE_CLOUDS

    import yam_arena

    cand_urdfs = [
        pathlib.Path(yam_arena.YAM_URDF),
        pathlib.Path(r"C:/Users/jingy/AppData/Local/Temp/robocurve-gpt-yam-hand-reference/yam_arm.urdf"),
    ]
    urdf_path = next((u for u in cand_urdfs if u.is_file()), cand_urdfs[0])
    cand_mesh_roots = [
        urdf_path.parent,
        pathlib.Path(r"C:/Users/jingy/AppData/Local/Temp/robocurve-gpt-yam-hand-reference"),
        pathlib.Path(r"C:/Users/jingy/bt-riser/tasks/shared-bin-coworker/assets/yam_pro_linear_4310/meshes"),
    ]

    def _find_mesh(rel_str: str) -> pathlib.Path | None:
        for root in cand_mesh_roots:
            p1 = root / rel_str
            if p1.is_file():
                return p1
            p2 = root / pathlib.Path(rel_str).name
            if p2.is_file():
                return p2
        return None

    tree = ET.parse(urdf_path)
    clouds: dict[str, np.ndarray] = {}
    for lnk in tree.getroot().findall("link"):
        nm = lnk.attrib["name"]
        if nm not in ALL_LINKS:
            continue
        ps = []
        for col in lnk.findall("collision"):
            m = col.find("geometry/mesh")
            if m is None:
                continue
            mp = _find_mesh(m.attrib["filename"])
            if mp is not None:
                v = _read_stl_pts(mp, dense_tip=(nm in DISTAL_LINKS))
                orig = col.find("origin")
                if orig is not None:
                    xyz = np.asarray([float(x) for x in orig.attrib.get("xyz", "0 0 0").split()], float)
                    rpy = [float(x) for x in orig.attrib.get("rpy", "0 0 0").split()]
                    v = (_rpy_to_R(rpy) @ v.T).T + xyz
                ps.append(v)
        if not ps:
            vis = lnk.find("visual")
            if vis is not None:
                m = vis.find("geometry/mesh")
                if m is not None:
                    mp = _find_mesh(m.attrib["filename"])
                    if mp is not None:
                        v = _read_stl_pts(mp, dense_tip=False)
                        orig = vis.find("origin")
                        if orig is not None:
                            xyz = np.asarray([float(x) for x in orig.attrib.get("xyz", "0 0 0").split()], float)
                            rpy = [float(x) for x in orig.attrib.get("rpy", "0 0 0").split()]
                            v = (_rpy_to_R(rpy) @ v.T).T + xyz
                        ps.append(v)
        if ps:
            merged = np.unique(np.round(np.vstack(ps), 3), axis=0)
            if len(merged) > max_pts_per_link:
                merged = merged[np.linspace(0, len(merged) - 1, max_pts_per_link).astype(int)]
            clouds[nm] = merged

    # Partition `tip_left` and `tip_right` along gripper approach axis `z_gripper` (`TCP = 0.134 m`, pad tip = `0.147 m`):
    # - `_pad` (`z_gripper >= 0.120 m`): distal clamping pad (intended for flange/bar pinch, lift, carry, & release),
    # - `_blade` (`0.096 <= z_gripper < 0.120 m`): flat finger blade (intended ONLY for wall slide/push, NEVER flange pinch!),
    # - `_shank` (`z_gripper < 0.096 m`): widened finger knuckle/root (NEVER intended in any phase!).
    from yam_arena.urdf import forward_kinematics as _fk_init
    fkm0 = _fk_init(str(urdf_path), {f"joint{i}": 0.0 for i in range(1, 9)})
    Tg0_inv = np.linalg.inv(fkm0["gripper"])
    for dlnk in DISTAL_LINKS:
        if dlnk in clouds and dlnk in fkm0:
            Td0 = fkm0[dlnk]
            pts_d = clouds[dlnk]
            pts_w0 = (Td0[:3, :3] @ pts_d.T).T + Td0[:3, 3]
            pts_g0 = (Tg0_inv[:3, :3] @ (pts_w0 - fkm0["gripper"][:3, 3]).T).T
            clouds[f"{dlnk}_pad"] = pts_d[pts_g0[:, 2] >= 0.120]
            clouds[f"{dlnk}_blade"] = pts_d[(pts_g0[:, 2] >= 0.096) & (pts_g0[:, 2] < 0.120)]
            clouds[f"{dlnk}_shank"] = pts_d[pts_g0[:, 2] < 0.096]

    _CACHED_PORTABLE_CLOUDS = clouds
    return clouds


def pan_obbs_world(pos, R_mat: np.ndarray) -> dict[str, tuple[np.ndarray, np.ndarray, tuple[float, float, float]]]:
    p0 = np.asarray(pos, float)
    R = np.asarray(R_mat, float)
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


def obb_corners_and_mids(c: np.ndarray, R: np.ndarray, s: tuple[float, float, float]) -> np.ndarray:
    hx, hy, hz = 0.5 * s[0], 0.5 * s[1], 0.5 * s[2]
    loc = np.array([[sx * hx, sy * hy, sz * hz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], float)
    corn = (R @ loc.T).T + c
    mids = 0.5 * (corn[:, None, :] + corn[None, :, :]).reshape(-1, 3)
    return np.vstack([corn, mids])


def signed_obb_mm(pts: np.ndarray, c: np.ndarray, R: np.ndarray, s: tuple[float, float, float]) -> float:
    rel = (pts - c) @ R
    d = np.abs(rel) - 0.5 * np.asarray(s, float)
    out = np.linalg.norm(np.maximum(d, 0.0), axis=1)
    ins = np.minimum(np.max(d, axis=1), 0.0)
    return float(np.min(out + ins) * 1e3)


def obb_separation_margin_mm(
    center_a: np.ndarray,
    rotation_a: np.ndarray,
    size_a: tuple[float, float, float],
    center_b: np.ndarray,
    rotation_b: np.ndarray,
    size_b: tuple[float, float, float],
) -> float:
    """Largest signed SAT axis gap (`codex-w50-pan-pair-sat.patch`).
    Positive separates boxes (axis margin, not Euclidean distance); negative means overlap; 0 means exact touch.
    """
    axes_a, axes_b = rotation_a.T, rotation_b.T
    edge_axes = np.cross(axes_a[:, None, :], axes_b[None, :, :]).reshape(-1, 3)
    axes = np.concatenate((axes_a, axes_b, edge_axes))
    lengths = np.linalg.norm(axes, axis=1)
    valid = lengths > 1e-10
    axes = axes[valid] / lengths[valid, None]
    span_a = np.abs(axes @ rotation_a) @ (0.5 * np.asarray(size_a))
    span_b = np.abs(axes @ rotation_b) @ (0.5 * np.asarray(size_b))
    offset = np.abs(axes @ (center_b - center_a))
    return float(np.max(offset - span_a - span_b) * 1000.0)


class RuntimeNamedPairContactLogger:
    """Evaluates signed clearances using **measured link poses** (`link_poses_L`, `link_poses_R`, env-origin-relative, zero IK!),
    retaining exact `(arm:link -> obj:subpart@phase)` pair names and distinguishing intentional push/grasp surfaces (`Codex 0907`).
    """

    def __init__(self, local_link_clouds: dict[str, np.ndarray] | None = None):
        self.clouds = local_link_clouds if local_link_clouds is not None else load_urdf_link_clouds_portable()
        self.reset()

    def reset(self):
        self.worst_housing = (999.0, "none")
        self.worst_unintended_distal = (999.0, "none")
        self.worst_distal_to_occ = (999.0, "none")
        self.worst_pan_to_occ = (999.0, "none")
        self.worst_intentional_distal = (999.0, "none")
        self.latched_housing_violation = False
        self.latched_unintended_distal_violation = False
        self.latched_pan_to_occ_violation = False

    def evaluate_step(
        self,
        step: int,
        link_poses_L: dict[str, np.ndarray],
        link_poses_R: dict[str, np.ndarray],
        phase_L: str,
        phase_R: str,
        active_pan_name: str,
        pan_states: dict[str, tuple[np.ndarray, np.ndarray]],
        bar_state: tuple[np.ndarray, np.ndarray],
        occupied_pan_names: set[str],
    ) -> dict:
        import w50_occupied_q2_and_far_isolated_audit as OQA

        I3 = np.eye(3)
        scene_obbs = {
            ("env", "stand_top"): (np.array([0.5110, 0.0, 0.5875]), I3, (0.7620, 1.2190, 0.0200)),
            ("env", "w50_front_wall"): (np.array([0.4800 - 0.1524 - 0.015875, 0.0, 0.5975 + 0.1143]), I3, (0.03175, 0.5715, 0.2286)),
            ("env", "w50_back_wall"): (np.array([0.4800 + 0.1524 + 0.015875, 0.0, 0.5975 + 0.1143]), I3, (0.03175, 0.5715, 0.2286)),
            ("env", "w50_posY_wall"): (np.array([0.4800, +0.2540 + 0.015875, 0.5975 + 0.1143]), I3, (0.3048, 0.03175, 0.2286)),
            ("env", "w50_negY_wall"): (np.array([0.4800, -0.2540 - 0.015875, 0.5975 + 0.1143]), I3, (0.3048, 0.03175, 0.2286)),
            ("adaptor_bar", "bar_strip"): (
                np.asarray(bar_state[0], float) + np.asarray(bar_state[1], float) @ np.array([0.0, 0.0, 0.0040]),
                np.asarray(bar_state[1], float),
                (0.0254, 0.5310, 0.0080),
            ),
        }
        pan_obb_trees = {}
        for pnm, (ppos, pR) in pan_states.items():
            tree = pan_obbs_world(ppos, pR)
            pan_obb_trees[pnm] = tree
            for sub, obb in tree.items():
                scene_obbs[(pnm, sub)] = obb

        step_h = (999.0, "none")
        step_ud = (999.0, "none")
        step_occ = (999.0, "none")
        step_p2o = (999.0, "none")
        step_id = (999.0, "none")

        for arm_tag, lposes, lab in (("L", link_poses_L, phase_L), ("R", link_poses_R, phase_R)):
            world_pts = {}
            for lnk in ALL_LINKS:
                if lnk in lposes and lnk in self.clouds:
                    T = lposes[lnk]
                    world_pts[lnk] = (T[:3, :3] @ self.clouds[lnk].T).T + T[:3, 3]
            for (obj_nm, sub_nm), (oc, oR, osz) in scene_obbs.items():
                is_bar_grasp = ("bar_" in lab) and (obj_nm == "adaptor_bar")
                for hlnk in HOUSING_LINKS:
                    if hlnk not in world_pts:
                        continue
                    d = signed_obb_mm(world_pts[hlnk], oc, oR, osz)
                    ps = f"{arm_tag}:{hlnk}->{obj_nm}:{sub_nm}@{lab}(step={step})"
                    if d < step_h[0]:
                        step_h = (round(d, 2), ps)
                for dlnk in DISTAL_LINKS:
                    if dlnk not in lposes:
                        continue
                    Td = lposes[dlnk]
                    is_wall_push_phase = any(p in lab for p in ("side_pull_", "slide_", "yguard", "sqA"))
                    # 1) Widened finger knuckle/root (`z_gripper < 0.096 m`): NEVER excused as intentional!
                    for sub_zone, allow_intent in (
                        ("shank", False),
                        ("blade", is_wall_push_phase and OQA.is_intentional_distal_surface(arm_tag, lab, obj_nm, sub_nm, active_pan_name)),
                        ("pad", is_bar_grasp or OQA.is_intentional_distal_surface(arm_tag, lab, obj_nm, sub_nm, active_pan_name)),
                    ):
                        z_key = f"{dlnk}_{sub_zone}"
                        z_loc = self.clouds.get(z_key)
                        if z_loc is None or len(z_loc) == 0:
                            continue
                        z_pts = (Td[:3, :3] @ z_loc.T).T + Td[:3, 3]
                        dz = signed_obb_mm(z_pts, oc, oR, osz)
                        psz = f"{arm_tag}:{z_key}->{obj_nm}:{sub_nm}@{lab}(step={step})"
                        if allow_intent:
                            if dz < step_id[0]:
                                step_id = (round(dz, 2), psz)
                        else:
                            if dz < step_ud[0]:
                                step_ud = (round(dz, 2), psz)
                            if obj_nm in occupied_pan_names and dz < step_occ[0]:
                                step_occ = (round(dz, 2), psz)

        # Active and occupied pans use complete 15-axis SAT box-box overlap at each measured step (`Codex 0916`).
        if active_pan_name in pan_obb_trees and occupied_pan_names:
            for asub, (ac, aR, asz) in pan_obb_trees[active_pan_name].items():
                for onm in occupied_pan_names:
                    if onm == active_pan_name or onm not in pan_obb_trees:
                        continue
                    for osub, (oc, oR, osz) in pan_obb_trees[onm].items():
                        d = obb_separation_margin_mm(ac, aR, asz, oc, oR, osz)
                        if d < step_p2o[0]:
                            step_p2o = (round(d, 2), f"{active_pan_name}:{asub}->{onm}:{osub}@{phase_L}(step={step})")

        if step_h[0] < self.worst_housing[0]:
            self.worst_housing = step_h
        if step_ud[0] < self.worst_unintended_distal[0]:
            self.worst_unintended_distal = step_ud
        if step_occ[0] < self.worst_distal_to_occ[0]:
            self.worst_distal_to_occ = step_occ
        if step_p2o[0] < self.worst_pan_to_occ[0]:
            self.worst_pan_to_occ = step_p2o
        if step_id[0] < self.worst_intentional_distal[0]:
            self.worst_intentional_distal = step_id

        if step_h[0] < -0.5:
            self.latched_housing_violation = True
        if step_ud[0] < -1.5:
            self.latched_unintended_distal_violation = True
        if step_p2o[0] < -1.0:
            self.latched_pan_to_occ_violation = True

        return {
            "step": step,
            "active_pan": active_pan_name,
            "sampled_housing_min_mm": step_h,
            "sampled_unintended_distal_min_mm": step_ud,
            "sampled_distal_to_occ_min_mm": step_occ if step_occ[0] < 900 else None,
            "pan_to_occ_sat_axis_margin_mm": step_p2o if step_p2o[0] < 900 else None,
            "sampled_intentional_distal_min_mm": step_id if step_id[0] < 900 else None,
            "latched_violations": {
                "sampled_housing": self.latched_housing_violation,
                "sampled_unintended_distal": self.latched_unintended_distal_violation,
                "pan_to_occ_sat": self.latched_pan_to_occ_violation,
            },
        }

    def episode_summary(self) -> dict:
        return {
            "worst_sampled_housing_min_mm": self.worst_housing,
            "worst_sampled_unintended_distal_min_mm": self.worst_unintended_distal,
            "worst_sampled_distal_to_occ_min_mm": self.worst_distal_to_occ if self.worst_distal_to_occ[0] < 900 else None,
            "worst_pan_to_occ_sat_axis_margin_mm": self.worst_pan_to_occ if self.worst_pan_to_occ[0] < 900 else None,
            "worst_sampled_intentional_distal_min_mm": self.worst_intentional_distal if self.worst_intentional_distal[0] < 900 else None,
            "latched_housing_violation": self.latched_housing_violation,
            "latched_unintended_distal_violation": self.latched_unintended_distal_violation,
            "latched_pan_to_occ_violation": self.latched_pan_to_occ_violation,
        }


def apply_visible_pose_y_guard_q2(
    plan: dict[str, list],
    pan_pos,
    pan_R,
    pan_y_target: float = 0.0000,
    current_left_tcp: tuple[float, float, float] | None = None,
    current_left_finger: float | None = None,
    current_right_tcp: tuple[float, float, float] | None = None,
    current_right_finger: float | None = None,
) -> dict:
    """Mirrored pre-push Y guard for `Near_Q2` (`Right` arm slid `Q2` from `-0.4675`, `yguard2_slot_*` is on `Right` arm!)."""
    import w50_squareup_and_crossing_witness as SQW
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    decision = SQW.check_pre_push_y_guard(pan_pos, pan_R, pan_y_target=pan_y_target)
    act = decision["controller_action"]
    if act == "PROCEED_NOMINAL":
        return decision

    if act == "STOP_SAFE_PARK":
        l_cur = tuple(float(v) for v in (current_left_tcp if current_left_tcp is not None else (0.220, +0.300, 0.895)))
        r_cur = tuple(float(v) for v in (current_right_tcp if current_right_tcp is not None else (0.210, -0.280, 0.860)))
        fl_cur = float(current_left_finger if current_left_finger is not None else FINGER_OPEN)
        fr_cur = float(current_right_finger if current_right_finger is not None else FINGER_OPEN)
        xr_safe = min(r_cur[0], 0.1900) if r_cur[2] < 0.760 else min(r_cur[0], 0.2150)
        z_hi = max(0.9000, l_cur[2], r_cur[2])

        abort_r = [
            ((xr_safe, r_cur[1], z_hi), fr_cur, "abort2_lift_vertical"),
            ((0.220, -0.320, 0.920), fr_cur, "abort2_retreat_closed"),
            ((0.220, -0.320, 0.920), FINGER_OPEN, "abort2_open_at_park"),
            ((0.220, -0.320, 0.920), FINGER_OPEN, "stopped_by_q2_prepush_guard"),
            ((0.220, -0.320, 0.920), FINGER_OPEN, "stopped_by_q2_prepush_guard"),
        ]
        slot_r = [w for w in plan["right"] if w.label.startswith("yguard2_slot_")]
        for w, (pn, fn, ln) in zip(slot_r, abort_r):
            w.pos, w.finger, w.yaw, w.label = pn, fn, 0.0, ln

        after_r = False
        for w in plan["right"]:
            if w.label in ("abort2_lift_vertical", "yguard2_slot_0"):
                after_r = True
                continue
            if after_r and not w.label.startswith("abort2_"):
                w.pos, w.finger, w.yaw, w.label = (0.220, -0.320, 0.920), FINGER_OPEN, 0.0, "stopped_by_q2_prepush_guard"

        trig_l = False
        for w in plan["left"]:
            if w.label == "wait_yguard2_slot":
                trig_l = True
            if trig_l:
                w.pos, w.finger, w.yaw, w.label = (0.220, +0.320, 0.920), FINGER_OPEN, 0.0, "stopped_by_q2_prepush_guard"
        return decision

    # `CORRECT_Y_MICRO_NUDGE` on Right arm along safe `x = 0.1850 m` lane!
    y_pan = float(np.asarray(pan_pos, float)[1])
    dy = y_pan - pan_y_target
    sgn_push = -1.0 if dy > 0.0 else +1.0
    y_wall_now = y_pan + sgn_push * SQW.WALL_INNER_HALF_Y
    y_wall_goal = pan_y_target + sgn_push * SQW.WALL_INNER_HALF_Y
    y_entry = y_wall_now - sgn_push * (SQW.FINGER_FACE_HALF_Y + 0.018)
    y_end = y_wall_goal - sgn_push * SQW.FINGER_FACE_HALF_Y
    y_unhook = y_end - sgn_push * 0.020
    z_hi, z_in = STAND_TOP + Q_D + 0.075, STAND_TOP + 0.028

    targets = [
        ((0.1850, y_entry, z_hi), FINGER_CLOSED, "yguard2_above_cavity"),
        ((0.1850, y_entry, z_in), FINGER_CLOSED, "yguard2_dip_cavity"),
        ((0.1850, y_end, z_in), FINGER_CLOSED, "yguard2_nudge_to_y0"),
        ((0.1850, y_unhook, z_in), FINGER_CLOSED, "yguard2_unhook"),
        ((0.2150, -0.290, 0.885), FINGER_OPEN, "yguard2_lift_clear"),
    ]
    slot_r = [w for w in plan["right"] if w.label.startswith("yguard2_slot_")]
    for w, (pn, fn, ln) in zip(slot_r, targets):
        w.pos, w.finger, w.yaw, w.label = tuple(float(v) for v in pn), fn, 0.0, ln
    return decision


def demo_2pan_near_guarded(y_stage_abs: float = 0.4675, y_trim_abs: float = 0.0078) -> dict[str, list]:
    """Isolated 2-Pan Near-Channel (`Bar + Q1 pickup/seat + Q2 pickup/occupied straight-up release`) in ONE undeformed sequence!
    Total duration = `99.80 s` (`1,996` steps at 20 Hz `<= 2,400` steps).
    """
    import w50_bar_waypoints as BW
    import w50_squareup_and_crossing_witness as SQW
    import w50_isolated_q2_and_latched_interpenetration as IQ2
    import w50_occupied_q2_and_far_isolated_audit as OQA
    from yam_arena.constants import FINGER_OPEN
    from yam_arena.demo import Waypoint

    # 1. Bar transfer (`0.00..16.00 s`, steps `1..320`)
    bL, bR = BW.bar_two_hand_d405_up()

    # 2. Near_Q1 (`16.00..59.00 s`, steps `321..1180`: `9.1 s` slide -> Guard 1 at step `498` -> `2.6 s` slot -> `8.7 s` Pitched-A -> `22.6 s` stock c7i)
    s1L, s1R = SQW.slide_int2_x0185_trimmed("left", y_stage_abs, -y_trim_abs)
    g1L, g1R = SQW.make_y_guard_slot(0.0)
    p1L, p1R = SQW.squareup_push_A_pitched(0.0, X_STRIP)
    q1L, q1R = BW.c7i_pan_waypoints((X_STRIP, 0.0), (X_NEAR_2PAN, Y_POS_SEAT), tag="near_q1", is_occupied_q2=False)

    # 3. Occupied Near_Q2 (`59.00..99.80 s`, steps `1181..1996`: `9.1 s` slide -> Guard 2 at step `1358` -> `2.6 s` slot -> `8.7 s` Left Pitched-A -> `19.4 s` straight-up release + `1.0 s` park)
    s2L, s2R = SQW.slide_int2_x0185_trimmed("right", y_stage_abs, +y_trim_abs)
    g2_durs = (0.55, 0.45, 0.70, 0.40, 0.50)  # 2.60 s
    g2R = [Waypoint(d, (0.215, -0.290, 0.880), FINGER_OPEN, f"yguard2_slot_{i}", 0.0) for i, d in enumerate(g2_durs)]
    g2L = [Waypoint(sum(g2_durs), (0.220, +0.300, 0.895), FINGER_OPEN, "wait_yguard2_slot", 0.0)]
    p2L, p2R = IQ2.squareup_push_A_pitched_left_arm(0.0, X_STRIP)
    q2L, q2R = OQA.two_hand_seat_waypoints_isolated((X_STRIP, 0.0), (X_NEAR_2PAN, Y_NEG_SEAT), channel="near", is_occupied_q2=True, tag="near_q2")

    kL = [Waypoint(1.0, (0.22, +0.32, 0.92), FINGER_OPEN, "park_still", 0.0)]
    kR = [Waypoint(1.0, (0.22, -0.32, 0.92), FINGER_OPEN, "park_still", 0.0)]
    return {
        "left": bL + s1L + g1L + p1L + q1L + s2L + g2L + p2L + q2L + kL,
        "right": bR + s1R + g1R + p1R + q1R + s2R + g2R + p2R + q2R + kR,
    }


def _compact_slide_x0185(side: str, y_start_abs: float = 0.4675, y_trim: float = -0.0078) -> list:
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
    from yam_arena.demo import Waypoint

    sgn = +1.0 if side == "left" else -1.0
    xp = 0.1850
    ye = sgn * (y_start_abs - (BODY_L / 2.0 - SHEET) + 0.033)
    yc = sgn * (y_start_abs - (BODY_L / 2.0 - SHEET) + 0.011)
    y0 = sgn * (0.0000 - (BODY_L / 2.0 - SHEET) + 0.011) + y_trim
    zc, zi = STAND_TOP + Q_D + 0.075, STAND_TOP + 0.028
    return [
        Waypoint(0.55, (0.200, sgn * 0.300, 0.860), FINGER_CLOSED, f"slide_{side}_ready", 0.0),
        Waypoint(0.75, (xp, ye, zc), FINGER_CLOSED, f"slide_{side}_above_cavity", 0.0),
        Waypoint(0.50, (xp, ye, zi), FINGER_CLOSED, f"slide_{side}_dip_cavity", 0.0),
        Waypoint(0.35, (xp, yc, zi), FINGER_CLOSED, f"slide_{side}_touch_wall", 0.0),
        Waypoint(2.10, (xp, y0, zi), FINGER_CLOSED, f"slide_{side}_pull_to_y0_ytrim", 0.0),
        Waypoint(0.35, (xp, y0 + sgn * 0.025, zi), FINGER_CLOSED, f"slide_{side}_unhook_ytrim", 0.0),
        Waypoint(0.60, (xp, y0 + sgn * 0.025, zc + 0.040), FINGER_CLOSED, f"slide_{side}_lift_out_ytrim", 0.0),
        Waypoint(0.60, (0.210, sgn * 0.280, 0.860), FINGER_OPEN, f"slide_{side}_return_ready", 0.0),
    ]


def _compact_side_deck_pull(side: str, x_deck: float = 0.4500, y_abs: float = 0.4675, total_duration: float = 5.80) -> list:
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
    from yam_arena.demo import Waypoint

    sgn = +1.0 if side == "left" else -1.0
    wall_ix = BODY_HALF_X - SHEET
    fhx = 0.02004
    xe = x_deck - wall_ix + fhx + 0.020
    xc = x_deck - wall_ix + fhx + 0.002
    x0 = X_STRIP - wall_ix + fhx
    zh, zi = STAND_TOP + Q_D + 0.075, STAND_TOP + 0.028
    wps = [
        Waypoint(0.55, (0.240, sgn * 0.340, 0.860), FINGER_CLOSED, f"side_pull_{side}_ready", 0.0),
        Waypoint(0.75, (xe, sgn * y_abs, zh), FINGER_CLOSED, f"side_pull_{side}_above", 0.0),
        Waypoint(0.50, (xe, sgn * y_abs, zi), FINGER_CLOSED, f"side_pull_{side}_dip", 0.0),
        Waypoint(0.35, (xc, sgn * y_abs, zi), FINGER_CLOSED, f"side_pull_{side}_touch_nx", 0.0),
        Waypoint(1.90, (x0, sgn * y_abs, zi), FINGER_CLOSED, f"side_pull_{side}_pull_to_strip", 0.0),
        Waypoint(0.35, (x0 + 0.022, sgn * y_abs, zi), FINGER_CLOSED, f"side_pull_{side}_unhook", 0.0),
        Waypoint(0.65, (x0 + 0.022, sgn * y_abs, zh + 0.040), FINGER_CLOSED, f"side_pull_{side}_lift", 0.0),
    ]
    rem = max(0.45, round(total_duration - sum(w.duration for w in wps), 4))
    wps.append(Waypoint(rem, (0.220, sgn * 0.300, 0.895), FINGER_OPEN, f"side_pull_{side}_done", 0.0))
    return wps


def _compact_squareup_A(arm_side: str) -> tuple[list, list]:
    import w50_two_hand_near_waypoints as HW
    import w50_squareup_and_crossing_witness as SQW
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
    from yam_arena.demo import Waypoint

    Orient, D_DOWN = HW.install_orient_hook()
    yaw_s = +SQW.YAW_SQ_DEG if arm_side == "right" else -SQW.YAW_SQ_DEG
    R_sq = Orient(SQW._Ry_deg(SQW.PITCH_SQ_DEG) @ SQW._Rz_deg(yaw_s) @ D_DOWN)
    park_y = -0.300 if arm_side == "right" else +0.300
    wait_y = -park_y
    tag = "sqA" if arm_side == "right" else "sqA_L"

    x_end = (X_STRIP - BODY_HALF_X) - SQW.DX_TIP_PITCHED
    x_start, x_ret, z = 0.0810, 0.0900, SQW.Z_TCP_SQ_A
    act = [
        Waypoint(0.55, (0.1100, 0.0, 0.7800), FINGER_CLOSED, f"{tag}_above_pitched", R_sq),
        Waypoint(0.50, (x_start, 0.0, 0.7200), FINGER_CLOSED, f"{tag}_pre_descent", R_sq),
        Waypoint(0.60, (x_start, 0.0, z), FINGER_CLOSED, f"{tag}_down_clear", R_sq),
        Waypoint(1.80, (x_end, 0.0, z), FINGER_CLOSED, f"{tag}_push_plus_x", R_sq),
        Waypoint(0.45, (x_end, 0.0, z), FINGER_CLOSED, f"{tag}_hold_wall", R_sq),
        Waypoint(0.50, (x_ret, 0.0, z), FINGER_CLOSED, f"{tag}_retract_before_lift", R_sq),
        Waypoint(0.50, (x_ret, 0.0, 0.7800), FINGER_CLOSED, f"{tag}_rise_clear", R_sq),
        Waypoint(0.55, (0.2200, park_y, 0.8950), FINGER_OPEN, f"{tag}_return_ready", 0.0),
    ]
    T = sum(w.duration for w in act)
    wait = [Waypoint(T, (0.2200, wait_y, 0.8950), FINGER_OPEN, f"wait_{tag}", 0.0)]
    return (wait, act) if arm_side == "right" else (act, wait)


def _compact_two_hand_seat(seat_xy: tuple[float, float], channel: str, is_occupied_q2: bool, tag: str) -> tuple[list, list]:
    import w50_occupied_q2_and_far_isolated_audit as OQA
    qL, qR = OQA.two_hand_seat_waypoints_isolated((X_STRIP, 0.0), seat_xy, channel=channel, is_occupied_q2=is_occupied_q2, tag=tag)
    scale_map = {
        "pre_high": 0.65, "pre_line_up": 0.60, "slide_on": 0.60, "pinch": 0.40,
        "lift_35mm": 0.65, "hold_1p5s": 0.50, "raise_carry": 1.15,
        "cross_front_rim": 1.35, "carry_over_seat": 1.65 if channel == "far" else 2.90,
        "pitch_to_seat": 0.60, "lower_to_slow": 0.75, "lower_slow_final": 1.85,
        "release_controlled": 0.40, "unhook_straight_up": 0.60, "peel_back": 0.60,
        "rise_above_neighbors": 0.65, "retreat_up": 0.65, "retreat_park": 0.75,
    }
    for arm_wps in (qL, qR):
        for w in arm_wps:
            for k, d_new in scale_map.items():
                if w.label.endswith(k):
                    w.duration = d_new
                    break
    return qL, qR


def demo_4pan_2400step_bimanual() -> dict[str, list]:
    """Intact 4-Pan (`Far_Q1 -> Far_Q2 -> Near_Q1 -> Near_Q2`) source inside `<= 2,400` steps (`Codex 0907` line 7)."""
    import w50_bar_waypoints as BW
    from yam_arena.constants import FINGER_OPEN
    from yam_arena.demo import Waypoint

    bL, bR = BW.bar_two_hand_d405_up()  # 16.0 s
    s1L = _compact_slide_x0185("left", 0.4675, -0.0078)
    s1R = _compact_side_deck_pull("right", 0.4500, 0.4675, total_duration=sum(w.duration for w in s1L))
    sq1L, sq1R = _compact_squareup_A("right")
    f1L, f1R = _compact_two_hand_seat((X_FAR_SEAT, Y_POS_SEAT), channel="far", is_occupied_q2=False, tag="far_q1")

    s2R = _compact_slide_x0185("right", 0.4675, +0.0078)
    s2L = _compact_side_deck_pull("left", 0.4500, 0.4675, total_duration=sum(w.duration for w in s2R))
    sq2L, sq2R = _compact_squareup_A("left")
    f2L, f2R = _compact_two_hand_seat((X_FAR_SEAT, Y_NEG_SEAT), channel="far", is_occupied_q2=True, tag="far_q2")

    s3L = _compact_slide_x0185("left", 0.4675, -0.0078)
    s3R = _compact_side_deck_pull("right", 0.4500, 0.4675, total_duration=sum(w.duration for w in s3L))
    sq3L, sq3R = _compact_squareup_A("right")
    n1L, n1R = _compact_two_hand_seat((X_NEAR_4PAN, Y_POS_SEAT), channel="near", is_occupied_q2=False, tag="near_q1_4pan")

    s4R = _compact_slide_x0185("right", 0.4675, +0.0078)
    s4L = [Waypoint(sum(w.duration for w in s4R), (0.220, +0.300, 0.895), FINGER_OPEN, "wait_slide4", 0.0)]
    sq4L, sq4R = _compact_squareup_A("left")
    n2L, n2R = _compact_two_hand_seat((X_NEAR_4PAN, Y_NEG_SEAT), channel="near", is_occupied_q2=True, tag="near_q2_4pan")

    kL = [Waypoint(1.0, (0.22, +0.32, 0.92), FINGER_OPEN, "park_still", 0.0)]
    kR = [Waypoint(1.0, (0.22, -0.32, 0.92), FINGER_OPEN, "park_still", 0.0)]
    return {
        "left": bL + s1L + sq1L + f1L + s2L + sq2L + f2L + s3L + sq3L + n1L + s4L + sq4L + n2L + kL,
        "right": bR + s1R + sq1R + f1R + s2R + sq2R + f2R + s3R + sq3R + n1R + s4R + sq4R + n2R + kR,
    }
