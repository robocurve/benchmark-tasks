"""CPU geometry for the exit-guide pickup (not physics).

``plan``: solves exit_guide_pickup_probe's right-arm path with the same
warm-started IK, poses the PhysX convex pieces, and checks them against the
kiosk (including the exit-guide prongs), case, scale, table and camera mast
taken from task.py. It sweeps the jaw closure at the grasp (nominal and with
tracking offsets), and predicts pad coverage of panel09 when the sheet is
clamped flat on the jaw centre. Reset panels are the predicted rest pose
(flat on the channel floor and prongs); the GPU run records the actual one.

``trace RUN_DIR``: poses the pieces from recorded joints and counts panel07-09
mid-plane samples inside any piece, plus pad coverage of panel09 faces.
Face-plane samples (as in extract_penetration_cpu.py) count touching pads as
inside; mid-plane samples do not.
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import task  # noqa: E402
import yam_hulls  # noqa: E402
from bagtag_probe import Plan  # noqa: E402
from exit_guide_pickup_probe import (CLOSE_START_S, GRASP, HOLD_END_S, MOUTH_X,  # noqa: E402
                                     pickup_waypoints, panel_frame)
from yam_arena import ik, station  # noqa: E402
from yam_arena.constants import CAMERA_MAST_POS, CAMERA_MAST_SIZE, FINGER_OPEN, READY  # noqa: E402

L, WID, TH = task.SEGMENT_LENGTH, task.TAG_WIDTH, task.TAG_COLLISION_THICKNESS
REST_Z = task.TOP + .232 + TH / 2
STALL = -TH / 2
PRONGS = [b for b in task.kiosk.boxes if abs(b[0][0] + .214) < 1e-6]


def inside_any(points, hulls):
    """Convex half-space test with a bounding-box prefilter (hull pieces are convex)."""
    mask = np.zeros(len(points), bool)
    if not len(points) or not hulls:
        return mask
    lo = np.min([h.vertices.min(0) for h in hulls], 0) - 1e-4
    hi = np.max([h.vertices.max(0) for h in hulls], 0) + 1e-4
    near = np.flatnonzero(((points >= lo) & (points <= hi)).all(1))
    for h in hulls:
        # Only points in this convex piece's AABB can be inside it. Testing
        # the complete arm-wide cloud against every piece wasted most CPU.
        candidate = near[~mask[near]]
        bounds = ((points[candidate] >= h.vertices.min(0)-1e-4) &
                  (points[candidate] <= h.vertices.max(0)+1e-4)).all(1)
        candidate = candidate[bounds]
        if not len(candidate):
            continue
        n = h.face_normals
        d = (h.triangles[:, 0] * n).sum(1)
        sub = points[candidate]
        mask[candidate] |= ((sub @ n.T - d) <= 0).all(1)
    return mask


def world_boxes(prt, specs):
    return [(np.add(b[0], prt.pos), np.asarray(b[1], float)) for b in specs]


def obstacles():
    groups = {
        "kiosk": world_boxes(task.kiosk, [b for b in task.kiosk.boxes if b not in PRONGS]),
        "prongs": world_boxes(task.kiosk, PRONGS),
        "case": world_boxes(task.bag, task.bag.boxes) + world_boxes(task.bag, task.bag.sliders[0].boxes),
        "scale": world_boxes(task.scale, task.scale.boxes),
        "table": [(np.array([.45, 0, task.TOP - .005]), np.array([.9, 1.2, .01]))],
        "mast": [(np.asarray(CAMERA_MAST_POS), np.asarray(CAMERA_MAST_SIZE))],
    }
    pts = {}
    for k, boxes in groups.items():
        pts[k] = np.concatenate([
            yam_hulls.box_points(c, np.eye(3), s, tuple(max(2, int(math.ceil(v / .004)) + 1) for v in s))
            for c, s in boxes])
    return groups, pts


def box_clearance(verts, boxes):
    """Min distance (m) from posed hull vertices to axis-aligned boxes."""
    best = np.inf
    for c, s in boxes:
        d = np.maximum(np.abs(verts - c) - s / 2, 0.0)
        best = min(best, float(np.sqrt((d * d).sum(1)).min()))
    return best


def predicted_reset_panels():
    out = []
    for i in range(task.SEGMENTS):
        c = np.array([task.TAG_X0 - (i + .5) * L, task.TAG_Y, task.TAG_Z if i == 0 else REST_Z])
        out.append((c, np.eye(3)))
    return out


def sheet_points(c, R, n=(22, 25)):
    """Mid-plane samples of a panel (thickness centre only)."""
    return yam_hulls.box_points(c, R, (L, WID, 0.0), (*n, 1))


def face_cover(c, R, upper, lower):
    """Pad coverage (mm^2) of a panel's top/bottom faces, sampled 0.3 mm outside each face."""
    n = (44, 50)
    cell = (L / n[0]) * (WID / n[1]) * 1e6
    up_n = R[:, 2] if R[2, 2] >= 0 else -R[:, 2]
    top = yam_hulls.box_points(c + (TH / 2 + .0003) * up_n, R, (L, WID, 0.0), (*n, 1))
    bot = yam_hulls.box_points(c - (TH / 2 + .0003) * up_n, R, (L, WID, 0.0), (*n, 1))
    a = inside_any(top, upper)
    b = inside_any(bot, lower)

    def xs(m, p):
        return [round(float(p[m][:, 0].min()), 4), round(float(p[m][:, 0].max()), 4)] if m.any() else None

    return {"upper_pad_mm2": round(float(a.sum() * cell), 1), "lower_pad_mm2": round(float(b.sum() * cell), 1),
            "upper_x_range": xs(a, top), "lower_x_range": xs(b, bot)}


def tips_by_height(h, tcp):
    """(upper, lower) pad pieces: only pieces near the TCP; the finger carriages cross over when closed."""
    def pad(name):
        return [x for x in h[name] if np.linalg.norm(x.vertices.mean(0) - tcp) < .03]

    def z(pieces):
        return np.mean([x.vertices[:, 2].mean() for x in pieces])

    left, right = pad("tip_left"), pad("tip_right")
    return (right, left) if z(right) > z(left) else (left, right)


LEFT_READY = cKDTree(np.concatenate([v for vs in yam_hulls.posed(np.r_[READY, FINGER_OPEN, FINGER_OPEN],
                                                                 "left").values() for v in vs]))


def check_pose(q6, finger, groups, pts, panels=()):
    joints = np.r_[q6, finger, finger]
    h = yam_hulls.hulls(joints, "right")
    allh = [x for xs in h.values() for x in xs]
    verts = np.concatenate([x.vertices for x in allh])
    row = {k: int(inside_any(p, allh).sum()) for k, p in pts.items()}
    row["clear_mm"] = {k: round(box_clearance(verts, groups[k]) * 1000, 2) for k in ("kiosk", "prongs", "case", "table", "mast")}
    row["clear_mm"]["left_arm_ready"] = round(float(LEFT_READY.query(verts)[0].min()) * 1000, 1)
    for i, (c, R) in panels:
        row[f"panel{i:02d}_inside"] = int(inside_any(sheet_points(c, R), allh).sum())
    return row, h


def plan_check():
    groups, pts = obstacles()
    reset = predicted_reset_panels()
    plan = Plan(pickup_waypoints(), station.BASES["right"])
    base = station.BASES["right"]
    steps = round(plan.duration * 20)
    k_close, k_hold = round(CLOSE_START_S * 20), round(HOLD_END_S * 20)
    rows, q_grasp = [], None
    for k in range(1, steps + 1):
        action, label = plan.action7(k / 20)
        if k == k_close:
            q_grasp = action[:6].copy()
        if k % 4 and k not in (k_close, k_hold, steps):
            continue
        open_ = k <= k_close
        finger = FINGER_OPEN * float(action[6]) if open_ else STALL
        panels = [(i, reset[i]) for i in (6, 7, 8, 9)] if open_ else []
        row, _ = check_pose(action[:6], finger, groups, pts, panels)
        tcp, _ = ik.tcp_pose(action[:6], base)
        rows.append({"t_s": k / 20, "phase": label, "tcp": tcp.round(4).tolist(), "finger": round(finger, 4), **row})

    # Jaw closure at the grasp, nominal and with tracking offsets seen in end-pinch runs.
    sweeps = {}
    R = plan.waypoints[2].R
    for name, off in {"nominal": (0, 0, 0), "z-3mm": (0, 0, -.003), "z+3mm": (0, 0, .003),
                      "x+4mm": (.004, 0, 0), "x-4mm": (-.004, 0, 0), "y+2mm": (0, .002, 0),
                      "y-2mm": (0, -.002, 0)}.items():
        q, pe, re = ik.solve(np.add(GRASP, off), ik.matrix_to_quat_xyzw(R), q_grasp, base_pos=base, iters=200)
        worst = {"kiosk": 0, "prongs": 0, "case": 0, "table": 0}
        clear = np.inf
        for f in np.linspace(FINGER_OPEN, STALL, 24):
            row, _ = check_pose(q, f, groups, pts)
            for key in worst:
                worst[key] = max(worst[key], row[key])
            clear = min(clear, row["clear_mm"]["prongs"])
        sweeps[name] = {"ik_mm": round(pe * 1000, 3), "max_points_inside": worst, "min_prong_clear_mm": round(clear, 2)}

    # Predicted clamp: panel09 flat on the jaw centre, panel08 hinged from the mouth.
    q, _, _ = ik.solve(np.asarray(GRASP), ik.matrix_to_quat_xyzw(R), q_grasp, base_pos=base, iters=200)
    tcp, _ = ik.tcp_pose(q, base)
    c9 = np.array([task.TAG_X0 - 9.5 * L, task.TAG_Y, tcp[2]])
    row, h = check_pose(q, STALL, groups, pts, [(9, (c9, np.eye(3)))])
    upper, lower = tips_by_height(h, tcp)
    prong_end_x = groups["prongs"][0][0][0] - groups["prongs"][0][1][0] / 2
    clamp = {"tcp": tcp.round(4).tolist(), "panel09_centre": c9.round(4).tolist(), **row,
             "cover": face_cover(c9, np.eye(3), upper, lower),
             "prong_end_x": round(float(prong_end_x), 4),
             "panel09_com_over_prong_mm": round(float(c9[0] - prong_end_x) * 1000, 2)}

    final_tcp, final_R = ik.tcp_pose(plan.q, base)
    pad_front_grasp = GRASP[0] + .0128
    strip_beyond_pads = task.SEGMENTS * L - (pad_front_grasp - (task.TAG_X0 - task.SEGMENTS * L))
    front = final_tcp + .0128 * final_R[:, 2]
    mouth = np.array([MOUTH_X, task.TAG_Y, REST_Z])
    out = {
        "stage": "cpu_geometry_not_physics", "probe": "exit_guide_pickup_probe.py",
        "right_worst_ik_mm": round(plan.worst_ik_mm, 3), "right_worst_ik_deg": round(plan.worst_ik_deg, 3),
        "prongs_world": [{"centre": np.round(c, 4).tolist(), "size": s.tolist()} for c, s in groups["prongs"]],
        "predicted_reset_free_end_x": round(task.TAG_X0 - task.SEGMENTS * L, 4),
        "predicted_rest_z": round(REST_Z, 4),
        "closure_sweeps": sweeps, "predicted_clamp": clamp,
        "extraction": {"final_tcp": final_tcp.round(4).tolist(),
                       "pad_front_to_mouth_m": round(float(np.linalg.norm(front - mouth)), 4),
                       "strip_beyond_pads_m": round(float(strip_beyond_pads), 4)},
        "rows": rows,
    }
    (HERE / "runs/exit-guide-pickup-cpu.json").write_text(json.dumps(out, indent=1) + "\n")
    worst_rows = {k: max(r[k] for r in rows) for k in ("kiosk", "prongs", "case", "scale", "table", "mast")}
    min_clear = {k: min(r["clear_mm"][k] for r in rows)
                 for k in ("kiosk", "prongs", "case", "table", "mast", "left_arm_ready")}
    panel_hits = max(r.get(f"panel{i:02d}_inside", 0) for r in rows for i in (6, 7, 8, 9))
    print(json.dumps({"ik_mm": out["right_worst_ik_mm"], "ik_deg": out["right_worst_ik_deg"],
                      "path_points_inside": worst_rows, "path_min_clear_mm": min_clear,
                      "approach_panel_points_inside": panel_hits, "sweeps": sweeps,
                      "clamp": {k: clamp[k] for k in ("tcp", "panel09_inside", "cover", "prongs", "kiosk",
                                                      "panel09_com_over_prong_mm")},
                      "extraction": out["extraction"]}, indent=1))


def trace_check(run_dir: Path):
    data = json.loads((run_dir / "result.json").read_text())
    rows = []
    for s in data["trace"]:
        if s["step"] < round(CLOSE_START_S * 20) or s["step"] % 4:
            continue
        h = yam_hulls.hulls(np.asarray(s["robots"]["right"]["joints"]), "right",
                            links=("gripper", "tip_left", "tip_right"))
        row = {"step": s["step"], "phase": s["right_phase"],
               "fingers": np.round(s["robots"]["right"]["joints"][6:8], 5).tolist(), "inside": {}}
        for i in (7, 8, 9):
            c, R = panel_frame(s["panel_xyz"][i], s["panel_quat_xyzw"][i])
            pts = sheet_points(c, R)
            counts = {n: int(inside_any(pts, hs).sum()) for n, hs in h.items()}
            if any(counts.values()):
                row["inside"][f"panel{i:02d}"] = counts
        c, R = panel_frame(s["panel_xyz"][9], s["panel_quat_xyzw"][9])
        tcp, _ = ik.tcp_pose(np.asarray(s["robots"]["right"]["joints"][:6]), station.BASES["right"])
        upper, lower = tips_by_height(h, tcp)
        row["panel09_cover"] = face_cover(c, R, upper, lower)
        rows.append(row)
        print(json.dumps(row))
    out = {"stage": "cpu_penetration_check_of_recorded_contact_run", "run": run_dir.name, "rows": rows,
           "any_midplane_penetration": any(r["inside"] for r in rows)}
    (run_dir.parent / f"{run_dir.name}-midplane-cpu.json").write_text(json.dumps(out, indent=1) + "\n")


if __name__ == "__main__":
    if len(sys.argv) > 2 and sys.argv[1] == "trace":
        trace_check(Path(sys.argv[2]))
    else:
        plan_check()
