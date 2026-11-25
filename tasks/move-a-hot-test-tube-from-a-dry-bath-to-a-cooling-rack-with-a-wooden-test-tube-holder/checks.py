#!/usr/bin/env python3
"""The hands' reach, wrist clearances and start state of the hot test-tube transfer task.

1. Without the simulator: every waypoint of the reference plan (picking the wooden holder from the blue mat by its rear
   handle, squeezing the short lever at 120.0 mm spacing from the carrier hand, fitting the open V-jaws round the
   exposed glass neck above the 95.0 C block without crossing over the 96 x 76 mm heating block, lifting plumb out of
   the 45.0 mm bore, lowering plumb through both tiers of the stainless Z-rack, squeezing open to release, and laying
   the holder back on the blue mat), solved by yam_arena's inverse kinematics (<= 0.5 mm, <= 0.6 deg, every joint >= 6.0
   deg inside its limits); the two hands' collision hulls >= 25.0 mm apart at every two-hand step; neither hand ever
   touching or crossing over the 96 x 76 mm heating block.
2. ``--frame0 <dir>`` in the simulator: the first pictures of the three cameras (full 640 x 360 and policy 320 x 176)
   and 2.0 s at rest: all bodies still (< 1.5 mm from built), score 0.0, no zero rule triggered.

    scripts/rta simpy <id> mech/tasks/hot_tube_clamp/checks.py [--frame0 /workspace/robocurve/outputs/<id>]
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np
from yam_arena import analysis, constants as C, ik, station
from yam_arena.constants import READY
from yam_arena.task_file import TABLE_TOP

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import parts as P  # noqa: E402

TOP = TABLE_TOP["station"]
BASES = {side: np.array(station.BASES[side], dtype=float) for side in ("left", "right")}
HAND_LINKS = ("link3", "link4", "link5", "gripper", "tip_left", "tip_right")
MAX_JAW = 2.0 * abs(float(C.FINGER_OPEN))

IK_MM = 0.5
IK_DEG = 0.6
MARGIN_DEG = 6.0
TCP_SEPARATION = 0.115  # m: 120 mm nominal handle-to-tail grasp spacing along x
WRIST_HULL_MIN = 0.025  # m: >= 25 mm collision-hull clearance between both wrists/hands at all simultaneous poses


def frame(closing, approach) -> np.ndarray:
    """Gripper rotation: columns are closing axis, pads' width axis and approach axis."""
    a = np.asarray(approach, dtype=float)
    a /= np.linalg.norm(a)
    c = np.asarray(closing, dtype=float) - np.dot(closing, a) * a
    c /= np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


# Orthogonal / opposing wrist postures:
# - Right hand (carrier on rear handle, X_REAR_GRIP = -0.175 m): approaches from behind at 58 deg from vertical along +x
#   with purely horizontal closing = (0, -1, 0), so its 45 mm finger pads lie lengthwise along the wooden handle
#   (100 % vertical side faces, ~5x torsional anti-seesaw grip!).
# - Left hand (squeezer on short_lever tail, X_SQUEEZE = -0.055 m, 120 mm ahead): approaches from +y/above.
R_ROT = frame((0.0, -1.0, 0.0), (math.sin(math.radians(48.0)), 0.0, -math.cos(math.radians(48.0))))
L_ROT = frame((0.0, -1.0, 0.0), (0.22, -0.72, -0.66))

# Vertical relation between the clamp bar's centre z and the right hand's grasp TCP z:
CLAMP_START_Z = P.MAT_T + P.BAR_H / 2.0  # 0.010 m over bench on the 2 mm mat
R_TCP_ABOVE_BAR = 0.0080  # m: at 48 deg forward pitch, puts lower finger hull ~4.8 mm over bench, pads lengthwise along bar!
Z_CLAMP_AT_BATH = 0.0960  # m: clamp bottom at 88.0 mm; grips 55 mm up the 100 mm tube
Z_CLAMP_LIFT = 0.1510  # m: lifts tube 55.0 mm -> tube bottom at 96.0 mm (10 mm clear above 86 mm block); R_TCP = 159.0 mm <= 160 mm!
Z_CLAMP_AT_RACK = 0.0600  # m: lowers tube 86.5 mm -> tube bottom at 4.0 mm (2.0 mm above z = 2.0 mm rack floor); clamp bottom at 52.0 mm > 50 mm rack!

# Open angle when fitting jaws round / releasing the 16.0 mm tube (18.4 mm rim):
THETA_OPEN = 0.33  # rad (18.9 deg) -> jaw gap = 27.8 mm (9.4 mm clear of 18.4 mm rim); outer tail span = 30.2 mm
THETA_TUBE = P.theta_for_tube(P.TUBE_D)  # 0.1549 rad (8.88 deg)


def clamp_pin_for_tube_at(xy: tuple[float, float], z_bar: float) -> np.ndarray:
    """World position of the clamp's hinge pin when the 16.0 mm tube axis sits at ``xy`` in long_arm's V-pocket."""
    y_tube = P.tube_y_in_clamp(P.TUBE_R)  # +0.00424 m in clamp frame
    return np.array([xy[0] - P.X_JAW, xy[1] - y_tube, TOP + z_bar], dtype=float)


def right_grip_for_pin(pin_w: np.ndarray) -> np.ndarray:
    """Right-hand TCP in world coordinates when holding long_arm at X_REAR_GRIP."""
    return pin_w + np.array([P.X_REAR_GRIP, P.LONG_Y, R_TCP_ABOVE_BAR], dtype=float)


def left_squeeze_for_pin(pin_w: np.ndarray, theta: float, out_y: float = 0.0, up_z: float = 0.0) -> np.ndarray:
    """Left-hand TCP in world coordinates centred across (long_arm, short_lever) at X_SQUEEZE."""
    return pin_w + np.array([P.X_SQUEEZE, P.tail_mid_y(theta) + out_y, up_z], dtype=float)


# Staging station in front of the dry bath where the two hands open the jaws before advancing round the tube neck:
STAGE_BACK_X = 0.048  # m back along -x from WELL_XY so the closed/opening jaws clear the tube by > 30 mm


def poses() -> dict[str, tuple[str, np.ndarray, np.ndarray, float]]:
    """Every key pose of the plan: name -> (side, world TCP, rotation 3x3, jaw opening m)."""
    pin_mat = np.array([P.CLAMP_START_PIN[0], P.CLAMP_START_PIN[1], TOP + CLAMP_START_Z], dtype=float)
    pin_bath = clamp_pin_for_tube_at(P.WELL_XY, Z_CLAMP_AT_BATH)
    pin_stage = pin_bath - np.array([STAGE_BACK_X, 0.0, 0.0])
    pin_lift = clamp_pin_for_tube_at(P.WELL_XY, Z_CLAMP_LIFT)
    pin_rack_hi = clamp_pin_for_tube_at(P.TARGET_HOLE_XY, Z_CLAMP_LIFT)
    pin_rack_lo = clamp_pin_for_tube_at(P.TARGET_HOLE_XY, Z_CLAMP_AT_RACK)
    pin_rack_back = pin_rack_lo - np.array([0.050, 0.0, 0.0])

    span_0 = P.tail_span(0.0)
    span_open = P.tail_span(THETA_OPEN)
    span_tube = P.tail_span(THETA_TUBE)

    return {
        # 1. Right hand picks holder on blue mat:
        "r: over holder on mat": ("right", right_grip_for_pin(pin_mat) + np.array([0.0, 0.0, 0.055]), R_ROT, 0.050),
        "r: grasp holder on mat": ("right", right_grip_for_pin(pin_mat), R_ROT, P.BAR_W),
        "r: lift holder off mat": ("right", right_grip_for_pin(pin_mat) + np.array([0.0, 0.0, 0.090]), R_ROT, P.BAR_W),
        # 2. Stage in front of bath, left hand squeezes short_lever open, advance round tube, release to clamp:
        "r: stage before bath": ("right", right_grip_for_pin(pin_stage), R_ROT, P.BAR_W),
        "l: approach tail at stage": ("left", left_squeeze_for_pin(pin_stage, 0.0, out_y=0.045, up_z=0.020), L_ROT, MAX_JAW),
        "l: around tail at stage": ("left", left_squeeze_for_pin(pin_stage, 0.0), L_ROT, span_0 + 0.022),
        "l: squeezed open at stage": ("left", left_squeeze_for_pin(pin_stage, THETA_OPEN), L_ROT, span_open),
        "r: jaws round tube at bath": ("right", right_grip_for_pin(pin_bath), R_ROT, P.BAR_W),
        "l: squeezed open at bath": ("left", left_squeeze_for_pin(pin_bath, THETA_OPEN), L_ROT, span_open),
        "l: released onto tube": ("left", left_squeeze_for_pin(pin_bath, THETA_TUBE, out_y=0.030), L_ROT, span_tube + 0.030),
        "l: clear beside bath": ("left", left_squeeze_for_pin(pin_bath, THETA_TUBE, out_y=0.090, up_z=0.035), L_ROT, MAX_JAW),
        # 3. Right hand lifts plumb out of 45 mm bore, carries to rack target hole, lowers plumb through both tiers:
        "r: lifted plumb over bath": ("right", right_grip_for_pin(pin_lift), R_ROT, P.BAR_W),
        "r: plumb over rack hole": ("right", right_grip_for_pin(pin_rack_hi), R_ROT, P.BAR_W),
        "r: lowered into rack hole": ("right", right_grip_for_pin(pin_rack_lo), R_ROT, P.BAR_W),
        # 4. Left hand squeezes open at rack to release tube, right hand withdraws open jaws, lays holder back on mat:
        "l: approach tail at rack": ("left", left_squeeze_for_pin(pin_rack_lo, THETA_TUBE, out_y=0.055, up_z=0.025), L_ROT, MAX_JAW),
        "l: around tail at rack": ("left", left_squeeze_for_pin(pin_rack_lo, THETA_TUBE), L_ROT, span_tube + 0.022),
        "l: squeezed open at rack": ("left", left_squeeze_for_pin(pin_rack_lo, THETA_OPEN), L_ROT, span_open),
        "r: withdrawn from rack": ("right", right_grip_for_pin(pin_rack_back), R_ROT, P.BAR_W),
        "l: withdrawn with tail": ("left", left_squeeze_for_pin(pin_rack_back, THETA_OPEN), L_ROT, span_open),
        "l: let go after rack": ("left", left_squeeze_for_pin(pin_rack_back, 0.0, out_y=0.065, up_z=0.040), L_ROT, MAX_JAW),
    }


TOGETHER = (
    ("r: stage before bath", "l: approach tail at stage"),
    ("r: stage before bath", "l: around tail at stage"),
    ("r: stage before bath", "l: squeezed open at stage"),
    ("r: jaws round tube at bath", "l: squeezed open at bath"),
    ("r: jaws round tube at bath", "l: released onto tube"),
    ("r: jaws round tube at bath", "l: clear beside bath"),
    ("r: lifted plumb over bath", "l: clear beside bath"),
    ("r: lowered into rack hole", "l: approach tail at rack"),
    ("r: lowered into rack hole", "l: around tail at rack"),
    ("r: lowered into rack hole", "l: squeezed open at rack"),
    ("r: withdrawn from rack", "l: withdrawn with tail"),
    ("r: withdrawn from rack", "l: let go after rack"),
)


def solve(side: str, pos: np.ndarray, rot: np.ndarray, start=None):
    best = None
    seeds = ([np.asarray(start, dtype=float)] if start is not None else []) + [
        np.array(READY, dtype=float),
        np.array([0.4 if side == "left" else -0.4, 1.3, 1.3, -0.5, 0.0, 0.0], dtype=float),
    ]
    r = np.asarray(rot, dtype=float)
    quat = ik.matrix_to_quat_xyzw(r)
    for q0 in seeds:
        q, pe, re = ik.solve(np.asarray(pos, dtype=float), quat, q0, base_pos=BASES[side], iters=55)
        margin = float(np.degrees(min((q - ik.LOWER).min(), (ik.UPPER - q).min())))
        ok = pe < IK_MM / 1000.0 and math.degrees(re) < IK_DEG
        key = (ok, margin if ok else -(pe * 1000.0 + math.degrees(re)))
        if best is None or key > best[0]:
            best = (key, q, float(pe * 1000.0), float(math.degrees(re)), margin, False)
    assert best is not None
    return best[1], best[2], best[3], best[4], best[5]


def hand_hulls(side: str, q: np.ndarray, opening: float) -> list[tuple[str, analysis.Hull]]:
    half = -0.5 * max(0.0, min(MAX_JAW, opening))
    joints = {**{n: float(v) for n, v in zip(C.ARM_JOINTS, q)}, **dict.fromkeys(C.FINGER_JOINTS, half)}
    poses_w = analysis.fk_link_poses_world(joints, tuple(BASES[side]))
    out = []
    for link in HAND_LINKS:
        for h in analysis.link_hulls().get(link, []):
            r, p = poses_w[link][:3, :3], poses_w[link][:3, 3]
            n_w = h.planes[:, :3] @ r.T
            out.append((link, analysis.Hull(vertices=h.vertices @ r.T + p, planes=np.c_[n_w, h.planes[:, 3] + n_w @ p])))
    return out


def box_hull(centre, rotation, size) -> analysis.Hull:
    centre, rotation, half = np.asarray(centre, float), np.asarray(rotation, float), np.asarray(size, float) / 2.0
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * half
    vertices = corners @ rotation.T + centre
    planes = []
    for i in range(3):
        for s in (1.0, -1.0):
            n = s * rotation[:, i]
            planes.append([*n, float(n @ centre + half[i])])
    return analysis.Hull(vertices=vertices, planes=np.array(planes))


def gap(hulls_a: list[tuple[str, analysis.Hull]], hulls_b: list[tuple[str, analysis.Hull]]) -> float:
    """Exact separating-plane lower bound on distance (m) between two hull lists (vectorised)."""
    best = math.inf
    for _, a in hulls_a:
        va = a.vertices
        pa_n, pa_d = a.planes[:, :3], a.planes[:, 3]
        for _, b in hulls_b:
            d1 = float((va @ b.planes[:, :3].T - b.planes[:, 3]).max(axis=1).min())
            d2 = float((b.vertices @ pa_n.T - pa_d).max(axis=1).min())
            d = max(d1, d2) if (d1 > 0.0 or d2 > 0.0) else max(d1, d2)
            if d < best:
                best = d
    return best


def scene_obstacles() -> dict[str, list[tuple[str, analysis.Hull]]]:
    bath_box = box_hull((P.BATH_XY[0], P.BATH_XY[1], TOP + P.BATH_H / 2.0), np.eye(3), (P.BATH_D, P.BATH_W, P.BATH_H))
    rack_box = box_hull((P.RACK_XY[0], P.RACK_XY[1], TOP + P.RACK_H / 2.0), np.eye(3), (P.RACK_L, P.RACK_W, P.RACK_H))
    return {"bath": [("bath", bath_box)], "rack": [("rack", rack_box)]}


def run_offline() -> dict:
    obs = scene_obstacles()
    all_poses = poses()
    found, rows = {}, {}
    ok = True
    for name, (side, pos, rot, opening) in all_poses.items():
        q, mm, deg, margin, flip = solve(side, pos, rot)
        hulls = hand_hulls(side, q, opening)
        found[name] = (side, q, opening, hulls)
        verts = np.vstack([h.vertices for _, h in hulls])
        min_bench_mm = round(1000.0 * float(verts[:, 2].min() - TOP), 1)
        tcp_z_mm = round(1000.0 * float(pos[2] - TOP), 1)
        clear = {k: round(1000.0 * gap(hulls, v), 1) for k, v in obs.items()}
        # Mat-pickup pose stands over the 2 mm mat, every pose stays <= 160 mm TCP height and clear of bench/obstacles:
        floor_need = 3.0 if "mat" in name else 5.0
        good = (
            mm <= IK_MM
            and deg <= IK_DEG
            and margin >= MARGIN_DEG
            and tcp_z_mm <= 160.0
            and min_bench_mm >= floor_need
            and min(clear.values()) > 2.0
        )
        ok &= good
        rows[name] = {
            "side": side,
            "tcp (m)": [round(float(v), 4) for v in pos],
            "tcp z over bench (mm)": tcp_z_mm,
            "IK (mm, deg)": [round(mm, 3), round(deg, 3)],
            "least joint margin (deg)": round(margin, 1),
            "lowest hull over bench (mm)": min_bench_mm,
            "clear of (mm)": clear,
            "flipped": flip,
            "ok": good,
        }

    pairs = {}
    for a, b in TOGETHER:
        _, _, _, ha = found[a]
        _, _, _, hb = found[b]
        tcp_dist = float(np.linalg.norm(all_poses[a][1] - all_poses[b][1]))
        hull_dist = gap(ha, hb)
        good = tcp_dist >= TCP_SEPARATION and hull_dist >= WRIST_HULL_MIN
        ok &= good
        pairs[f"{a} + {b}"] = {
            "tcp apart (mm)": round(1000.0 * tcp_dist, 1),
            "wrist/hand hulls apart (mm)": round(1000.0 * hull_dist, 1),
            "ok": good,
        }

    return {
        "pass": bool(ok),
        "worst_ik_mm": max(r["IK (mm, deg)"][0] for r in rows.values()),
        "worst_ik_deg": max(r["IK (mm, deg)"][1] for r in rows.values()),
        "least_margin_deg": min(r["least joint margin (deg)"] for r in rows.values()),
        "max_tcp_z_mm": max(r["tcp z over bench (mm)"] for r in rows.values()),
        "least_inter_wrist_hull_mm": min(p["wrist/hand hulls apart (mm)"] for p in pairs.values()),
        "poses": rows,
        "two_hand_pairs": pairs,
    }


def run_frame0(out_dir: Path) -> dict:
    import cv2
    import demo_kit as kit
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack

    out_dir.mkdir(parents=True, exist_ok=True)
    emb = kit.open_task(HERE / "task.py")
    m = emb.task.module
    obs = emb.reset(Scene(id="g1", instruction=m.INSTRUCTION), seed=0)
    for cam, stem in (("top_cam", "start_top"), ("left_cam", "start_left_wrist"), ("right_cam", "start_right_wrist")):
        bgr = cv2.cvtColor(np.asarray(obs.images[cam]), cv2.COLOR_RGB2BGR)
        cv2.imwrite(str(out_dir / f"{stem}.png"), bgr)
        cv2.imwrite(str(out_dir / f"{stem}_policy.png"), cv2.resize(bgr, (320, 176), interpolation=cv2.INTER_AREA))
    ready = np.array([*READY, 1.0], dtype=float)
    action = Action(data=pack(ready, ready))
    first = None
    for _ in range(40):
        res = emb.step(action)
        if first is None:
            first = m.report()
    moved = {
        name: round(1000.0 * float(np.linalg.norm(m.mk.pose(name)[0] - np.asarray(m.mk.built_pose(name)[0]))), 2)
        for name in ("bath", "rack", "tube", "long_arm", "short_lever")
    }
    out = {
        "first step": first,
        "after 2 s": m.report(),
        "moved from built (mm)": moved,
        "score after 2 s": float(res.info.get("score", 0.0)),
    }
    (out_dir / "g1_frame0.json").write_text(json.dumps(out, indent=1, default=str))
    return out


def main() -> int:
    if "--parts" in sys.argv:
        print(json.dumps(P.summary(), indent=1))
        return 0
    offline = run_offline()
    summary = {k: v for k, v in offline.items() if k not in ("poses", "two_hand_pairs")}
    print("G1_OFFLINE_SUMMARY " + json.dumps(summary, indent=1), flush=True)
    if not offline["pass"] or "--verbose" in sys.argv:
        print("G1_OFFLINE_DETAIL " + json.dumps(offline, indent=1), flush=True)
    if "--frame0" in sys.argv:
        out = run_frame0(Path(sys.argv[sys.argv.index("--frame0") + 1]))
        print("G1_FRAME0 " + json.dumps(out, indent=1, default=str), flush=True)
    return 0 if offline["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
