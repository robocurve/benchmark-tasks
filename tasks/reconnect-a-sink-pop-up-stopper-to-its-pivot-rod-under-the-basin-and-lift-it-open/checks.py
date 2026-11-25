#!/usr/bin/env python3
"""G1 reach, hull clearances, grasp heights and the frame-0 idle test for sink_stopper.

1. Offline (no simulator): solves IK in sequence from READY for every left and right waypoint and checks
   - IK error <= 0.5 mm and <= 0.6 deg, every arm joint >= 5 deg inside its limits,
   - every grasp at most 160 mm over the bench,
   - forearm, wrist, gripper and finger hulls clear of the bench, the basin and wall, the P-trap and the drain,
   - left and right hand hulls >= 20 mm apart on the waypoints they reach together.
2. ``--frame0 <dir>`` in Isaac Sim: writes ``start_top.png``, ``start_left_wrist.png``, ``start_right_wrist.png`` (and
   ``*_policy.png`` at 320 x 176), holds READY for 2.0 s and checks that nothing moves more than 2.5 mm and the score
   is 0.0.
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
HAND_LINKS = ("link4", "link5", "gripper", "tip_left", "tip_right")
UPPER_LINKS = ("link2", "link3") + HAND_LINKS
MAX_JAW = 2.0 * abs(float(C.FINGER_OPEN))
IK_MM, IK_DEG, MARGIN_DEG, SEPARATION_HULL = 0.5, 0.6, 5.0, 0.020
GRASP_CEILING = 0.160  # m over the bench

NUT_C = 0.020  # m: the left grasp point 20 mm behind the nut's front face (its pads cover the nut's back part)
NUT_GRIP_OPEN = 0.042  # m
NUT_PINCH = 0.021  # m: finger target across the 31 mm nut (5 mm a finger, about 25 N a pad)
CLEVIS_GRIP_OPEN = 0.036  # m
CLEVIS_PINCH = 0.010  # m: across the 16 mm clevis block (3 mm a finger, about 15 N a pad)
LEFT_NUT = (17.0, 0.0)  # (yaw, tilt) deg of the left hand holding the nut: from the left, 17 deg toward the front
LEFT_STROKE = (8.0, 5.0)  # for the turning strokes: 8 deg toward the front, 5 deg down (pressing just over the nut's axis)
RIGHT_UP = 12.0  # deg the right hand points up at the clevis
PRESS_DEG = 20.0  # deg the rod's rear is pressed down (stopper lift about 9 mm; the rod settles 2-3 deg further)

TIP_REACH = 0.013  # m: the closed fingertips' end past the grasp point
TIP_EDGE = 0.0035  # m: the closed fingertips' upper edge, along the closing axis from the grasp point (the end is 7.6 mm)
STROKE_AT = 0.0065  # m behind the nut's front face: the fingertips' edge meets the nut's front band (36 facets), not its flats
STROKE_PHI = (-20.0, -55.0)  # deg: the fingertips' upper edge runs down the nut's left side, 20 to 55 deg under level,
# where the push along the approach (5 deg down) bears on the nut 25 to 60 deg off its radius: most of it turns the nut on


def frame(closing, approach) -> np.ndarray:
    """Gripper rotation: columns are the closing axis, the pads' width axis and the approach axis."""
    a = np.asarray(approach, dtype=float)
    a = a / np.linalg.norm(a)
    c = np.asarray(closing, dtype=float) - np.dot(closing, a) * a
    c = c / np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


def rot_y(deg: float) -> np.ndarray:
    t = math.radians(deg)
    return np.array([[math.cos(t), 0.0, math.sin(t)], [0.0, 1.0, 0.0], [-math.sin(t), 0.0, math.cos(t)]])


def left_rot(yaw_tilt=LEFT_NUT) -> np.ndarray:
    """Left hand pointing -y (turned ``yaw`` deg toward +x, pointing ``tilt`` deg down); fingers close across the nut."""
    y, t = (math.radians(v) for v in yaw_tilt)
    return frame((0.0, 0.0, 1.0), (math.sin(y) * math.cos(t), -math.cos(y) * math.cos(t), -math.sin(t)))


def right_rot(press_deg: float = 0.0, up_deg: float = RIGHT_UP) -> np.ndarray:
    """Right hand pointing +y, ``up_deg`` up, fingers closing on the clevis block's top and bottom, turned with the rod
    when its rear end is pressed ``press_deg`` down."""
    u = math.radians(up_deg)
    return rot_y(press_deg) @ frame((0.0, 0.0, 1.0), (0.0, math.cos(u), math.sin(u)))


def port(u: float, dz: float = 0.0) -> np.ndarray:
    """World point on the port's axis ``u`` behind the drain axis (``dz`` up)."""
    return np.array([P.X_DRAIN + u, P.Y_DRAIN, TOP + P.PORT_Z + dz])


CLEVIS_DY = -0.006  # m: the right grasp point sits 6 mm right of the rod, its fingertips 7 mm past the block's left face


def clevis_at(ball_u: float, press_deg: float = 0.0, dz: float = 0.0) -> np.ndarray:
    return port(ball_u, dz) + rot_y(press_deg) @ np.array([P.CLEVIS_AT_X, 0.0, 0.0]) + np.array([0.0, CLEVIS_DY, 0.0])


def nut_grasp(front_u: float, dz: float = 0.0) -> np.ndarray:
    return port(front_u + NUT_C, dz)


def edge_to_tcp(edge, rot) -> np.ndarray:
    """The grasp point that puts the closed fingertips' upper edge at ``edge``."""
    return np.asarray(edge, dtype=float) - TIP_REACH * rot[:, 2] - TIP_EDGE * rot[:, 0]


def stroke_tcp(front_u: float, phi_deg: float, lift: float = 0.0) -> np.ndarray:
    """Left grasp point with the closed fingertips' upper edge on the nut's left side at ``phi_deg`` (up from +y),
    ``lift`` out from the nut (negative: pressed in). The fingertips' end leans 5 deg, so under 5 deg only its upper
    edge touches the nut, and the push along the approach has a part along the nut's turning-on direction."""
    phi = math.radians(phi_deg)
    radial = np.array([0.0, math.cos(phi), math.sin(phi)])
    return edge_to_tcp(port(front_u + STROKE_AT) + (P.NUT_OD / 2.0 + lift) * radial, left_rot(LEFT_STROKE))


START_DZ = -P.ROD_START_DROP  # the rod's axis at the start


def nut_start_u() -> float:
    touch = math.sqrt((P.BALL_R + P.WASHER_BEAD_R) ** 2 - P.WASHER_RING_R**2)
    return P.U_BALL_START + touch + P.WASHER_BEAD_R - P.NUT_SKIRT_L + 0.0002


def poses() -> dict[str, tuple[str, np.ndarray, np.ndarray, float]]:
    """Named waypoints in the demo's order: (side, grasp point, rotation, finger opening as held)."""
    l0, l1, ls = left_rot(LEFT_NUT), left_rot(LEFT_NUT), left_rot(LEFT_STROKE)
    n0, r0 = nut_start_u(), right_rot()
    rp = right_rot(PRESS_DEG)
    return {
        "left: over nut": ("left", nut_grasp(n0, START_DZ) - 0.040 * l0[:, 2], l0, NUT_GRIP_OPEN),
        "left: round nut": ("left", nut_grasp(n0, START_DZ), l0, NUT_GRIP_OPEN),
        "left: nut held": ("left", nut_grasp(n0), l0, P.NUT_OD),
        "left: nut near start": ("left", nut_grasp(P.U_START + 0.0020), l1, P.NUT_OD),
        "left: nut on the land": ("left", nut_grasp(P.U_START), l1, P.NUT_OD),
        "left: let go of nut": ("left", nut_grasp(P.U_START) - 0.010 * l1[:, 2], l1, NUT_GRIP_OPEN),
        "left: stroke top": ("left", stroke_tcp(P.U_START, STROKE_PHI[0]), ls, 0.0),
        "left: stroke bottom": ("left", stroke_tcp(P.U_START, STROKE_PHI[1]), ls, 0.0),
        "left: stroke back": ("left", stroke_tcp(P.U_SNUG, STROKE_PHI[0], lift=0.008), ls, 0.0),
        "left: stroke bottom snug": ("left", stroke_tcp(P.U_SNUG, STROKE_PHI[1]), ls, 0.0),
        "left: clear": ("left", stroke_tcp(P.U_SNUG, 0.0, lift=0.070), ls, NUT_GRIP_OPEN),
        "right: over clevis": ("right", clevis_at(P.U_BALL_START, 0.0, START_DZ) - 0.032 * r0[:, 2], r0, CLEVIS_GRIP_OPEN),
        "right: round clevis": ("right", clevis_at(P.U_BALL_START, 0.0, START_DZ), r0, CLEVIS_GRIP_OPEN),
        "right: rod on axis": ("right", clevis_at(P.U_BALL_START, 0.0, -0.001), r0, P.CLEVIS_BLOCK[2]),
        "right: rod pushed in": ("right", clevis_at(P.SEAT_OFFSET_X), r0, P.CLEVIS_BLOCK[2]),
        "right: rod pressed down": ("right", clevis_at(P.SEAT_OFFSET_X, PRESS_DEG), rp, P.CLEVIS_BLOCK[2]),
        "right: let go of rod": ("right", clevis_at(P.SEAT_OFFSET_X, PRESS_DEG), rp, CLEVIS_GRIP_OPEN),
        "right: back off": ("right", clevis_at(P.SEAT_OFFSET_X, PRESS_DEG) - 0.030 * rp[:, 2] + np.array([0.0, 0.0, 0.012]), rp, CLEVIS_GRIP_OPEN),
    }


GRASPS = ("left: round nut", "left: nut held", "left: nut on the land", "left: stroke top", "left: stroke bottom",
          "right: round clevis", "right: rod on axis", "right: rod pushed in", "right: rod pressed down")
TOGETHER = (
    ("left: round nut", "right: round clevis"),
    ("left: nut held", "right: rod on axis"),
    ("left: nut on the land", "right: rod pushed in"),
    ("left: stroke top", "right: rod pushed in"),
    ("left: stroke bottom", "right: rod pushed in"),
    ("left: stroke bottom snug", "right: rod pushed in"),
    ("left: clear", "right: rod pressed down"),
)


def solve_chain(waypoints) -> dict:
    """IK in sequence per arm from READY, as demo_kit.Arm runs it."""
    q_prev = {"left": np.array(READY, dtype=float), "right": np.array(READY, dtype=float)}
    out = {}
    for name, side, pos, rot, opening in waypoints:
        q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), q_prev[side], base_pos=BASES[side], iters=200)
        q_prev[side] = q
        margin = float(np.degrees(min((q - ik.LOWER).min(), (ik.UPPER - q).min())))
        out[name] = (side, q, opening, float(pe * 1000.0), float(math.degrees(re)), margin)
    return out


def box_hull(centre, rotation, size) -> analysis.Hull:
    centre, rotation, half = np.asarray(centre, float), np.asarray(rotation, float), np.asarray(size, float) / 2.0
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], dtype=float) * half
    planes = []
    for i in range(3):
        for s in (1.0, -1.0):
            n = s * rotation[:, i]
            planes.append([*n, float(n @ centre + half[i])])
    return analysis.Hull(vertices=corners @ rotation.T + centre, planes=np.array(planes))


def hand_hulls(side: str, q: np.ndarray, opening: float, links=HAND_LINKS) -> list:
    half = -0.5 * max(0.0, min(MAX_JAW, opening))
    joints = {**{n: float(v) for n, v in zip(C.ARM_JOINTS, q)}, **dict.fromkeys(C.FINGER_JOINTS, half)}
    poses_w = analysis.fk_link_poses_world(joints, tuple(BASES[side]))
    out = []
    for link in links:
        for h in analysis.link_hulls().get(link, []):
            r, p = poses_w[link][:3, :3], poses_w[link][:3, 3]
            n_w = h.planes[:, :3] @ r.T
            out.append((link, analysis.Hull(vertices=h.vertices @ r.T + p, planes=np.c_[n_w, h.planes[:, 3] + n_w @ p])))
    if "gripper" in links:  # wrist-camera housing on the gripper
        rg, pg = poses_w["gripper"][:3, :3], poses_w["gripper"][:3, 3]
        out.append(("wrist_cam", box_hull(pg + rg @ np.array([0.0, -0.060, 0.005]), rg, (0.040, 0.050, 0.040))))
    return out


def gap(hulls_a, hulls_b) -> float:
    best = math.inf
    for _, a in hulls_a:
        for _, b in hulls_b:
            best = min(best, float(analysis._signed(a.vertices, b).min()), float(analysis._signed(b.vertices, a).min()))
    return best


def prism_hull(centre, rotation, radius: float, half_len: float, n: int = 16) -> analysis.Hull:
    """A prism whose n faces touch a cylinder of ``radius`` round its local z (``half_len`` each way)."""
    centre, rotation = np.asarray(centre, float), np.asarray(rotation, float)
    r_out = radius / math.cos(math.pi / n)
    verts, planes = [], []
    for k in range(n):
        a = 2.0 * math.pi * (k + 0.5) / n
        for z in (-half_len, half_len):
            verts.append(rotation @ np.array([r_out * math.cos(a), r_out * math.sin(a), z]) + centre)
        nl = rotation @ np.array([math.cos(2.0 * math.pi * k / n), math.sin(2.0 * math.pi * k / n), 0.0])
        planes.append([*nl, float(nl @ centre + radius)])
    for s in (1.0, -1.0):
        nl = s * rotation[:, 2]
        planes.append([*nl, float(nl @ centre + half_len)])
    return analysis.Hull(vertices=np.array(verts), planes=np.array(planes))


def body_hulls(name: str) -> list:
    """Hulls round every solid shape of a fixed or kinematic body, as built: boxes as they are, cylinders and capsules
    as 16-sided prisms (a capsule's prism runs over its end caps)."""
    import task as T

    spec = T.mk.MECH.bodies[name]
    bpos, brot = np.array(spec.pos, float), spec.rotation()
    out = []
    for k, sh in enumerate(spec.shapes):
        if not sh.solid:
            continue
        srot = brot @ T.mk.quat_to_matrix(sh.quat)
        centre = bpos + brot @ np.array(sh.pos, float)
        if sh.kind in ("cylinder", "capsule"):
            radius, length = sh.dims
            half = length / 2.0 + (radius if sh.kind == "capsule" else 0.0)
            out.append((f"{name}_{k}", prism_hull(centre, srot, radius, half)))
        else:
            out.append((f"{name}_{k}", box_hull(centre, srot, 2.0 * sh.half_extents())))
    return out


def obstacles() -> dict[str, list]:
    return {
        "bench": [("bench", box_hull((0.35, 0.0, TOP - 0.020), np.eye(3), (0.90, 1.00, 0.040)))],
        "basin": body_hulls("basin"),
        "trap": body_hulls("trap"),
        "drain": body_hulls("drain"),
    }


PATHS = (("left: nut held", "left: nut on the land"), ("left: round nut", "left: nut held"), ("left: let go of nut", "left: stroke top"),
         ("right: rod on axis", "right: rod pushed in"), ("right: rod pushed in", "right: rod pressed down"))


def path_clearance(a: str, b: str, steps: int = 6) -> float:
    """Least clearance (m) to the fixtures along a straight blend between two waypoints of one arm."""
    named = poses()
    side, pa, ra, oa = named[a]
    _, pb, rb, ob = named[b]
    obs = obstacles()
    q = solve_chain([(a, side, pa, ra, oa)])[a][1]
    least = math.inf
    for k in range(1, steps + 1):
        s_ = k / steps
        pos = pa + s_ * (pb - pa)
        rot = ik.slerp_matrix(ra, rb, s_)
        q, _pe, _re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), q, base_pos=BASES[side], iters=100)
        hulls = hand_hulls(side, q, max(oa, ob), UPPER_LINKS)
        least = min(least, min(gap(hulls, v) for v in obs.values()))
    return least


def run_offline() -> dict:
    named = poses()
    solved = solve_chain([(k, *v) for k, v in named.items()])
    obs = obstacles()
    rows, ok = {}, True
    for name, (side, pos, _rot, opening) in named.items():
        _, q, _, mm, deg, margin = solved[name]
        hulls = hand_hulls(side, q, opening, UPPER_LINKS)
        clear = {k: round(1000.0 * gap(hulls, v), 1) for k, v in obs.items()}
        over = float(pos[2] - TOP)
        good = mm <= IK_MM and deg <= IK_DEG and margin >= MARGIN_DEG and min(clear.values()) > 0.5
        if name in GRASPS:
            good &= over <= GRASP_CEILING
        ok &= good
        rows[name] = {"over_bench_mm": round(1000.0 * over, 1), "ik_mm_deg": [round(mm, 3), round(deg, 3)],
                      "margin_deg": round(margin, 1), "clear_mm": clear, "ok": bool(good)}
    pairs = {}
    for a, b in TOGETHER:
        (sa, qa, oa, *_), (sb, qb, ob, *_) = solved[a], solved[b]
        d = gap(hand_hulls(sa, qa, oa), hand_hulls(sb, qb, ob))
        ok &= d >= SEPARATION_HULL
        pairs[f"{a} + {b}"] = {"hulls_apart_mm": round(1000.0 * d, 1), "ok": bool(d >= SEPARATION_HULL)}
    paths = {}
    for a, b in PATHS:
        d = path_clearance(a, b)
        ok &= d > 0.0005
        paths[f"{a} -> {b}"] = round(1000.0 * d, 1)
    return {"pass": bool(ok), "parts": P.summary(), "poses": rows, "pairs": pairs, "paths_clear_mm": paths}


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
    res = None
    for _ in range(40):
        res = emb.step(action)
    assert res is not None
    moved = {n: round(1000.0 * float(np.linalg.norm(m.mk.pose(n)[0] - np.asarray(m.mk.built_pose(n)[0]))), 2)
             for n in ("stopper", "pivot_rod", "pivot_nut")}
    rep = m.report()
    good = float(res.info.get("score", 0.0)) == 0.0 and rep["zero"] is None and max(moved.values()) <= 2.5
    settled = {n: {"pos": [round(float(v), 6) for v in m.mk.pose(n)[0]], "quat": [round(float(v), 6) for v in m.mk.matrix_to_quat(m.mk.pose(n)[1])]}
               for n in ("pivot_rod", "pivot_nut", "stopper")}
    out = {"pass": good, "moved_from_built_mm": moved, "score_after_2s": float(res.info.get("score", 0.0)), "settled": settled, "report": rep}
    (out_dir / "g1_frame0.json").write_text(json.dumps(out, indent=1, default=str))
    return out


def main() -> int:
    offline = run_offline()
    print("G1_OFFLINE " + json.dumps(offline, indent=1), flush=True)
    ok = bool(offline["pass"])
    if "--frame0" in sys.argv:
        f0 = run_frame0(Path(sys.argv[sys.argv.index("--frame0") + 1]))
        print("G1_FRAME0 " + json.dumps(f0, indent=1, default=str), flush=True)
        ok &= bool(f0["pass"])
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
