#!/usr/bin/env python3
"""Reach check for the cake task, without the simulator: the arms' inverse kinematics on the URDF for the poses the
job needs, before any scene is built.

    scripts/rta simpy <id> mech/tasks/cake_smooth/reach.py /workspace/robocurve/outputs/<id>

Right hand: the smoother pinched across its 2 mm plate, the plate upright in a vertical plane that stands out from
the cake's right side, its straight edge upright against the icing with the lower corner on the cake board. Two
readings of which edge is straight: "long" (152 mm upright, the plate standing out 96.5 mm) and "short" (96.5 mm
upright, standing out 152 mm). The hand pinches near the plate's far end, coming in at ``approach`` degrees from
straight down within the plate's plane (0: down onto the plate's top edge; 90: level, along the plate towards
the cake). For each pose: the IK's error, the smallest joint margin, and how far the edge's middle gives under 1 N
from the cut, by the arm's joint springs alone (J K^-1 J^T; the pinch and the fingers add to it).
Left hand: closed fingertips pressing on the turntable's top 15 mm inside its rim, along an arc of the rim.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

STAND_TOP = 0.127  # m: the turntable's top over the bench (Ateco 612, 127 mm high)
RIM_R = 0.1525  # m: the top's radius (305 mm across)
BOARD_T, MAT_T = 0.002, 0.002  # m: cake board and non-slip mat (a)
CAKE_R, CAKE_H, ICING = 0.076, 0.102, 0.005  # m: dummy radius and height; icing on the side
PLATE = {"long": (0.152, 0.0965), "short": (0.0965, 0.152)}  # straight edge upright: (upright length, standing out)
PINCH_IN = 0.015  # m: the grasp point this far in from the plate's edges it is pinched near
TIP = 0.0129  # m: the closed fingertips' ends past the grasp point
PRESS = 0.002  # m: the fingertips this far into the turntable's top (a press)


def rotation(closing, approach) -> np.ndarray:
    """Columns: the closing axis, the pads' width axis, the approach axis (demo_kit's convention)."""
    c = np.asarray(closing, dtype=float) / np.linalg.norm(closing)
    a = np.asarray(approach, dtype=float) / np.linalg.norm(approach)
    return np.column_stack([c, np.cross(a, c), a])


def mk_axis(facing: float, lean: float) -> np.ndarray:
    """A plate's frame standing in a slot that faces ``facing`` (rad from +x) and leans ``lean`` about the slot."""
    c, s = math.cos(facing), math.sin(facing)
    yaw = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    cl, sl = math.cos(lean), math.sin(lean)
    tilt = np.array([[1.0, 0.0, 0.0], [0.0, cl, -sl], [0.0, sl, cl]])
    return yaw @ tilt


def solve(target, rot, base) -> dict:
    from yam_arena import ik

    best = None
    for start in ([0.0, 1.0, 1.0, -0.6, 0.0, 0.0], [0.0, 1.6, 1.4, -0.4, 0.0, 0.0], [0.0, 0.6, 0.6, -1.2, 0.0, 0.0], [0.0, 1.2, 1.8, 0.4, 0.0, 0.0]):
        q, err, rerr = ik.solve(target, ik.matrix_to_quat_xyzw(rot), np.array(start), base_pos=base, iters=200)
        margin = float(np.degrees(np.minimum(q - ik.LOWER, ik.UPPER - q)).min())
        found = {"q": q, "mm": 1000 * err, "deg": math.degrees(rerr), "margin": margin}
        if best is None or (found["mm"] + found["deg"], -margin) < (best["mm"] + best["deg"], -best["margin"]):
            best = found
    return best


def compliance(q, base, at, directions) -> dict:
    """mm the point ``at`` (from the grasp point, gripper frame) gives along each direction under 1 N along it."""
    import demo_kit as kit

    j = kit.point_jacobian(q, base, at)
    c = j @ np.diag(1.0 / kit.JOINT_STIFFNESS) @ j.T
    return {name: round(1000 * float(np.asarray(d) @ c @ np.asarray(d)), 2) for name, d in directions.items()}


def smoother_poses(centre, top_z, reading: str, swing_deg: float, approach_deg: float, base) -> dict:
    """The right hand holding the smoother on the cake's right side; the plate stands out at ``swing_deg`` from the
    radial (towards +x positive), the hand comes in at ``approach_deg`` from straight down."""
    upright, out = PLATE[reading]
    board_z = top_z + STAND_TOP + MAT_T + BOARD_T
    edge_xy = np.asarray(centre) + np.array([0.0, -(CAKE_R + ICING)])
    swing = math.radians(swing_deg)
    d = np.array([math.sin(swing), -math.cos(swing), 0.0])  # from the edge into the plate, level
    n = np.array([math.cos(swing), math.sin(swing), 0.0])  # the plate's normal
    a = math.radians(approach_deg)
    approach = math.cos(a) * np.array([0.0, 0.0, -1.0]) + math.sin(a) * (-d)  # within the plate's plane
    # grasp point: PINCH_IN in from the far end and from the top edge, moved along the approach so that the
    # fingertips' 13 mm stand on the plate
    far = np.array([*edge_xy, board_z]) + (out - PINCH_IN) * d
    top = board_z + upright
    if approach_deg <= 1e-6:
        grasp = np.array([far[0], far[1], top - PINCH_IN])
    elif approach_deg >= 90 - 1e-6:
        grasp = np.array([far[0], far[1], board_z + upright / 2])
    else:
        grasp = np.array([far[0], far[1], top - PINCH_IN])
    rot = rotation(n, approach)
    found = solve(grasp, rot, base)
    edge_mid = np.array([*edge_xy, board_z + min(upright, CAKE_H) / 2])
    at = rot.T @ (edge_mid - grasp)
    tangent = np.array([1.0, 0.0, 0.0])
    radial = np.array([0.0, -1.0, 0.0])
    found["compliance mm per N"] = compliance(found["q"], base, at, {"along the turning": tangent, "outward": radial, "up": np.array([0.0, 0.0, 1.0])})
    found["grasp over the bench (mm)"] = round(1000 * (grasp[2] - top_z), 1)
    found["edge from the grasp point (mm)"] = round(1000 * float(np.linalg.norm(edge_mid - grasp)), 1)
    return found


def rim_arc(centre, top_z, base) -> list:
    """Closed fingertips on the top 15 mm inside its rim, every 15 degrees round the rim; the hand pointing down,
    turned about the vertical as suits the arm best (a pressing fingertip needs no particular turn)."""
    from yam_arena.demo import grip_rotation

    out = []
    for deg in range(0, 360, 15):
        t = math.radians(deg)
        p = np.array([centre[0] + (RIM_R - 0.015) * math.cos(t), centre[1] + (RIM_R - 0.015) * math.sin(t), top_z + STAND_TOP + TIP - PRESS])
        best = None
        for yaw_deg in range(-90, 91, 30):
            found = solve(p, grip_rotation(math.radians(yaw_deg)), base)
            found["yaw"] = yaw_deg
            if best is None or (round(found["mm"] + found["deg"], 1), -found["margin"]) < (round(best["mm"] + best["deg"], 1), -best["margin"]):
                best = found
        out.append({"deg": deg, "mm": round(best["mm"], 1), "rot deg": round(best["deg"], 1), "margin": round(best["margin"], 1), "yaw": best["yaw"]})
    return out


def main() -> int:
    from yam_arena import station
    from yam_arena.task_file import TABLE_TOP

    out_dir = Path(sys.argv[1])
    out_dir.mkdir(parents=True, exist_ok=True)
    top_z = TABLE_TOP["station"]
    bases = station.BASES
    report: dict = {"bases": {k: list(v) for k, v in bases.items()}, "table top": top_z, "smoother": [], "rim": {}, "lever": []}
    only = sys.argv[2] if len(sys.argv) > 2 else "all"  # all | rim | lever | block | lean | plan:<run>
    if only == "tips":  # the closed fingertips' size about the grasp point, in the hand's frame (closing x, width y, pointing z)
        import demo_kit as kit
        from yam_arena.demo import grip_rotation

        kit.lowest_point(grip_rotation(0.0))
        corners = kit._HAND[0]
        lowest = corners[:, 2].max()  # the hand's z points along the approach: the fingertips' ends are the largest z
        for band in (0.005, 0.015, 0.030, 0.060):
            near = corners[corners[:, 2] > lowest - band]
            print(f"TIPS within {1000 * band:.0f} mm of the ends: x {1000 * near[:, 0].min():.1f} to {1000 * near[:, 0].max():.1f} mm, y {1000 * near[:, 1].min():.1f} to {1000 * near[:, 1].max():.1f} mm, z to {1000 * lowest:.1f} mm", flush=True)
        body = corners[corners[:, 2] < lowest - 0.060]
        print(f"TIPS the hand more than 60 mm up from the ends: x {1000 * body[:, 0].min():.1f} to {1000 * body[:, 0].max():.1f}, y {1000 * body[:, 1].min():.1f} to {1000 * body[:, 1].max():.1f} mm", flush=True)
        return 0
    if only == "stroke":  # the left hand pushing the rim: its collision hulls' clearance from the iced cake and the board
        from yam_arena import analysis, ik
        from yam_arena.constants import ARM_JOINTS, FINGER_CLOSED, FINGER_JOINTS
        from yam_arena.demo import grip_rotation

        centre = np.array([0.30, 0.0])
        board_top = top_z + STAND_TOP + MAT_T + BOARD_T
        outside = len(sys.argv) > 3 and sys.argv[3] == "outside"  # the fingertips' side against the rim's edge from outside
        for rim_in in ((-0.0072,) if outside else (0.004, 0.006, 0.010)):
            for yaw_deg in (-90, -60, -30, 0, 30, 60):
                worst, reached, where = 1e9, True, None
                for deg in range(70, 166, 5):
                    t = math.radians(deg)
                    p = np.array([centre[0] + (RIM_R - rim_in) * math.cos(t), centre[1] + (RIM_R - rim_in) * math.sin(t), top_z + (STAND_TOP - 0.003 - 0.004 + 0.0128 if outside else STAND_TOP + TIP - 0.001)])
                    for up in (0.0, 0.03):
                        f = solve(p + np.array([0.0, 0.0, up]), grip_rotation(math.radians(yaw_deg)), bases["left"])
                        if f["mm"] > 1.0 or f["deg"] > 1.0:
                            reached = False
                        joints = {**{n: float(v) for n, v in zip(ARM_JOINTS, f["q"])}, **dict.fromkeys(FINGER_JOINTS, FINGER_CLOSED)}
                        hulls = analysis.world_hulls(analysis.fk_link_poses_world(joints, bases["left"]), ["gripper", "tip_left", "tip_right", "link5", "link4"])
                        for link, h in hulls:
                            pts = h.vertices
                            r = np.hypot(pts[:, 0] - centre[0], pts[:, 1] - centre[1])
                            cake = np.where((pts[:, 2] > board_top) & (pts[:, 2] < board_top + CAKE_H + 0.002), r - (CAKE_R + 0.0115), np.inf)  # the icing at its thickest
                            board = np.where((pts[:, 2] > board_top - 0.006) & (pts[:, 2] < board_top + 0.001), r - 0.1015, np.inf)
                            for name, gap in (("cake", cake), ("board", board)):
                                i = int(np.argmin(gap))
                                if gap[i] < worst:
                                    worst, where = float(gap[i]), (link, name, round(1000 * float(pts[i, 2] - top_z), 1), round(1000 * float(r[i]), 1), deg, up)
                print(f"STROKE rim_in {1000 * rim_in:.0f} mm yaw {yaw_deg}: reached {reached}, least clearance from the iced cake or the board {1000 * worst:.1f} mm: (link, what, height over the bench mm, radius mm, rim deg, up) {where}", flush=True)
        return 0
    if only == "blocks":  # where the block can stand: the level pinch (+y), from behind, out of the slot, all reached
        rows = []
        for bx in (0.14, 0.17, 0.20, 0.23, 0.26):
            for by in (-0.16, -0.20, -0.24, -0.28, -0.32):
                for facing in (20.0, 35.0, 50.0, 65.0, 80.0, 95.0):
                    for lean in (30.0, -30.0):
                        r = mk_axis(math.radians(facing), math.radians(lean))
                        centre = np.array([bx, by, top_z + 0.0277]) + r @ np.array([0.0, 0.0, 0.076])
                        grasp = centre + (0.051 - 0.060) * r[:, 0]
                        rot = rotation(r[:, 1], r[:, 0])
                        poses = {"grasp": grasp, "behind": grasp - 0.060 * r[:, 0], "above behind": grasp - 0.060 * r[:, 0] + np.array([0.0, 0.0, 0.04]), "out": grasp + 0.033 * r[:, 2], "out and up": grasp + 0.033 * r[:, 2] + np.array([0.0, 0.0, 0.05])}
                        found = {k: solve(p, rot, bases["right"]) for k, p in poses.items()}
                        worst = max(f["mm"] for f in found.values())
                        least = min(f["margin"] for f in found.values())
                        rows.append((worst, -least, bx, by, facing, lean, {k: (round(f["mm"], 1), round(f["margin"], 1)) for k, f in found.items()}))
        rows.sort(key=lambda r: (r[0] > 1.0, -(-r[1]) if r[0] <= 1.0 else r[0]))
        for row in rows[:15]:
            print(f"BLOCKS worst {row[0]:.1f} mm, least margin {-row[1]:.1f}: block ({row[2]}, {row[3]}) facing {row[4]} lean {row[5]}: {row[6]}", flush=True)
        return 0
    if only == "lean":  # the leaning slot: which way it leans, which way it faces, which way the hand closes
        for bx, by in ((0.14, -0.25), (0.16, -0.22), (0.12, -0.28)):
            for facing in (45.0, 55.0, 65.0, 75.0, 85.0):
                for lean in (30.0, -30.0):
                    for sign in (1.0, -1.0):
                        f_ = math.radians(facing)
                        r = mk_axis(f_, math.radians(lean))
                        centre = np.array([bx, by, top_z + 0.0277]) + r @ np.array([0.0, 0.0, 0.076])
                        grasp = centre + (0.051 - 0.060) * r[:, 0]
                        rot = rotation(sign * r[:, 1], r[:, 0])
                        f = solve(grasp, rot, bases["right"])
                        over = solve(grasp + 0.10 * r[:, 2], rot, bases["right"])
                        print(f"LEAN block {bx},{by} facing {facing} lean {lean} sign {sign:+.0f}: grasp {f['mm']:.1f} mm {f['deg']:.1f} deg margin {f['margin']:.1f}; "
                              f"over {over['mm']:.1f} mm {over['deg']:.1f} deg margin {over['margin']:.1f}", flush=True)  # fmt: skip
        return 0
    if only == "take":  # the right hand along the take, its collision hulls against the plate's and the block's boxes
        import demo
        import demo_kit as kit
        from scipy.spatial import ConvexHull
        from yam_arena import analysis, task_file
        from yam_arena.constants import ARM_JOINTS, FINGER_CLOSED, FINGER_JOINTS, FINGER_OPEN

        m = task_file.load(str(HERE / "task.py")).module
        m.begin(0)
        plan = demo.Plan(m)
        settled = None
        if len(sys.argv) > 3:  # "lean,dx,dy,dz": the plate as it settled in a rest trial (its lean in degrees, its shift in mm)
            lean, *shift = (float(v) for v in sys.argv[3].split(","))
            p0, _ = m.mk.built_pose("smoother")
            settled = (p0 + np.array(shift) / 1000.0, m.mk.axis_angle((0.0, 0.0, 1.0), m.FACING) @ m.mk.axis_angle((1.0, 0.0, 0.0), -math.radians(lean)))
            built_pose = plan.pose
            plan.pose = lambda name: settled if name == "smoother" else built_pose(name)
        phase = next(plan.phases("take"))  # behind the plate, over it, close, out of the slot
        arm = kit.Arm("right", bases["right"], phase["right"])
        boxes = []  # (name, centre in the world, rotation, half extents, points on its faces)
        for name in ("smoother", "block"):
            p, r = settled if (name == "smoother" and settled is not None) else m.mk.built_pose(name)
            for i, s in enumerate(m.mk.MECH.bodies[name].shapes):
                if s.kind == "box" and s.solid:
                    half = s.half_extents()
                    g = [np.linspace(-h, h, max(3, int(2 * h / 0.002) + 1)) for h in half]
                    face = [np.array([a, b, c]) for a in g[0] for b in g[1] for c in g[2] if np.isclose(abs(a), half[0]) or np.isclose(abs(b), half[1]) or np.isclose(abs(c), half[2])]
                    rot = r @ m.mk.quat_to_matrix(s.quat)
                    centre = p + r @ np.array(s.pos)
                    boxes.append((f"{name} {i}", centre, rot, half, np.array(face) @ rot.T + centre))
        t, worst = 0.0, {}
        while t < arm.seconds:
            action, label = arm.action(t)
            joints = {**{n: float(v) for n, v in zip(ARM_JOINTS, action[:6])}, **dict.fromkeys(FINGER_JOINTS, FINGER_CLOSED + float(action[6]) * (FINGER_OPEN - FINGER_CLOSED))}
            for link, h in analysis.world_hulls(analysis.fk_link_poses_world(joints, bases["right"]), ["link4", "link5", "gripper", "tip_left", "tip_right"]):
                eq = ConvexHull(h.vertices).equations
                for name, centre, rot, half, face in boxes:
                    local = (h.vertices - centre) @ rot
                    d = np.abs(local) - half
                    sdf = np.linalg.norm(np.maximum(d, 0.0), axis=1) + np.minimum(d.max(axis=1), 0.0)
                    inside = (face @ eq[:, :3].T + eq[:, 3]).max(axis=1)  # < 0: the box's face point is inside the hull
                    depth = max(0.0, -float(sdf.min()), -float(inside.min()))
                    gap = float(sdf.min()) if depth == 0.0 else -depth
                    key = (label, link, name)
                    if key not in worst or gap < worst[key][0]:
                        worst[key] = (gap, round(t, 2))
            t += 0.05
        for (label, link, name), (gap, at) in sorted(worst.items(), key=lambda kv: kv[1][0])[:25]:
            print(f"TAKE {label}: {link} to {name}: {1000 * gap:.1f} mm at {at} s", flush=True)
        return 0
    if only.startswith("plan:"):  # the demo's plan move by move: the IK followed at 20 Hz, and each move's end solved afresh
        import demo
        import demo_kit as kit
        from yam_arena import task_file

        m = task_file.load(str(HERE / "task.py")).module
        m.begin(0)
        for arg in sys.argv[3:]:  # NAME=value: a constant of the plan set otherwise, for a what-if
            name, value = arg.split("=")
            setattr(demo, name, float(value))
        moves = demo.Plan(m).whole(only.split(":", 1)[1])
        for side in ("left", "right"):
            arm = kit.Arm(side, bases[side], moves[side])
            t, worst = 0.0, {}
            while t < arm.seconds:
                label = arm.state(t)[3]
                before = arm.q.copy()
                arm.worst_mm = arm.worst_deg = 0.0
                arm.action(t)
                from yam_arena import ik

                margins = np.degrees(np.minimum(arm.q - ik.LOWER, ik.UPPER - arm.q))
                margin = float(margins.min())
                w = worst.setdefault(label, {"followed mm": 0.0, "followed deg": 0.0, "least margin": 1e9, "at": None})
                if margin < w["least margin"]:
                    w["joint"] = int(np.argmin(margins)) + 1
                if arm.worst_mm > w["followed mm"]:
                    w["followed mm"], w["at"] = round(arm.worst_mm, 1), round(t, 2)
                w["followed deg"] = max(w["followed deg"], round(arm.worst_deg, 1))
                w["least margin"] = min(w["least margin"], round(margin, 1))
                del before
                t += 0.05
            elapsed = 0.0
            for mv in moves[side]:
                elapsed += mv.seconds
                if mv.pos is not None and mv.label in worst and "afresh" not in worst[mv.label]:
                    pos, rot = arm.state(elapsed - 1e-6)[:2]
                    f = solve(pos, rot, bases[side])
                    worst[mv.label]["afresh"] = {"mm": round(f["mm"], 1), "deg": round(f["deg"], 1), "margin": round(f["margin"], 1)}
            for label, w in worst.items():
                print(f"PLAN {side} {label}: {json.dumps(w)}", flush=True)
            if side == "right":  # the plate in the hand while it is turned over: its lowest corner, and how near the stand's axis
                t, lowest = 0.0, {}
                while t < arm.seconds:
                    pos, rot, _, label = arm.state(t)[:4]
                    if label.startswith("turns it"):
                        a, w_ = rot[:, 2], np.cross(rot[:, 2], rot[:, 0])
                        corners = [pos + u * a + v * w_ for u in (0.009 - 0.051, 0.009 + 0.051) for v in (-0.076, 0.076)]
                        low = min(corners, key=lambda c: c[2])
                        near = min(math.hypot(c[0] - m.CENTRE[0], c[1] - m.CENTRE[1]) for c in corners)
                        k = lowest.setdefault(label, {"lowest corner over the bench (mm)": 1e9, "nearest corner to the stand's axis (mm)": 1e9, "plate's edges to the block's box (mm)": 1e9})
                        k["lowest corner over the bench (mm)"] = min(k["lowest corner over the bench (mm)"], round(1000 * (low[2] - top_z), 1))
                        k["nearest corner to the stand's axis (mm)"] = min(k["nearest corner to the stand's axis (mm)"], round(1000 * near, 1))
                        block_p, block_r = m.mk.built_pose("block")
                        lo, hi = m.mk.MECH.bodies["block"].bounds()
                        samples = [pos + u * a + v * w_ for u in np.linspace(0.009 - 0.051, 0.009 + 0.051, 11) for v in np.linspace(-0.076, 0.076, 15)]  # the plate's face, sampled
                        gaps = [float(np.linalg.norm(np.maximum(np.maximum(lo - (block_r.T @ (p - block_p)), (block_r.T @ (p - block_p)) - hi), 0.0))) for p in samples]
                        k["plate's edges to the block's box (mm)"] = min(k["plate's edges to the block's box (mm)"], round(1000 * min(gaps), 1))
                    t += 0.05
                print(f"PLAN plate while turned: {json.dumps(lowest)}", flush=True)
        return 0
    if only == "block":  # the smoother standing in its block, straight edge upright: the hand takes it level at mid-height
        report["block"] = []
        for bx in (0.14, 0.18, 0.22):
            for by in (-0.20, -0.25, -0.30):
                for facing in (0.0, 30.0, 60.0, 90.0):  # degrees from +x of the way the straight edge faces (the hand points that way)
                    for z in (0.080, 0.100):
                        f_ = math.radians(facing)
                        u = np.array([math.cos(f_), math.sin(f_), 0.0])
                        normal = np.array([-math.sin(f_), math.cos(f_), 0.0])
                        grasp = np.array([bx, by, top_z + z]) + (0.051 - 0.060) * u
                        f = solve(grasp, rotation(normal, u), bases["right"])
                        row = {"block": [bx, by], "facing": facing, "grasp over bench mm": 1000 * z, "mm": round(f["mm"], 1), "rot deg": round(f["deg"], 1), "margin": round(f["margin"], 1)}
                        report["block"].append(row)
                        print("BLOCK " + json.dumps(row), flush=True)
        (out_dir / "reach.json").write_text(json.dumps(report, indent=1, default=str))
        return 0
    if only == "lever":  # the pinch nearer the edge: how the edge's give falls with the lever
        board_z = top_z + STAND_TOP + MAT_T + BOARD_T
        centre = (0.30, 0.0)
        edge_xy = np.asarray(centre) + np.array([0.0, -(CAKE_R + ICING)])
        edge_mid = np.array([*edge_xy, board_z + CAKE_H / 2])
        for approach in (45.0, 90.0):
            for swing in (-40.0, -20.0, 0.0):
                for lever in (0.040, 0.060, 0.090, 0.130):
                    sw = math.radians(swing)
                    d = np.array([math.sin(sw), -math.cos(sw), 0.0])
                    n = np.array([math.cos(sw), math.sin(sw), 0.0])
                    a = math.radians(approach)
                    appr = math.cos(a) * np.array([0.0, 0.0, -1.0]) + math.sin(a) * (-d)
                    for height in (0.060, 0.090, 0.120):
                        grasp = np.array([*(edge_xy + lever * d[:2]), board_z + height])
                        rot = rotation(n, appr)
                        f = solve(grasp, rot, bases["right"])
                        at = rot.T @ (edge_mid - grasp)
                        comp = compliance(f["q"], bases["right"], at, {"along": np.array([1.0, 0.0, 0.0]), "outward": np.array([0.0, -1.0, 0.0])})
                        row = {"approach": approach, "swing": swing, "lever mm": 1000 * lever, "grasp over board mm": 1000 * height, "mm": round(f["mm"], 1), "rot deg": round(f["deg"], 1), "margin": round(f["margin"], 1), "compliance mm/N": comp}
                        report["lever"].append(row)
                        print("LEVER " + json.dumps(row), flush=True)
        (out_dir / "reach.json").write_text(json.dumps(report, indent=1, default=str))
        return 0
    for centre in ((0.30, 0.0), (0.34, 0.0), (0.30, 0.04), (0.26, 0.0)):
        for reading in ("long", "short") if only == "all" else ():
            for swing in (-40.0, 0.0, 40.0):
                for approach in (0.0, 45.0, 90.0):
                    f = smoother_poses(centre, top_z, reading, swing, approach, bases["right"])
                    row = {"centre": centre, "straight edge": reading, "swing": swing, "approach": approach, "mm": round(f["mm"], 1), "rot deg": round(f["deg"], 1),
                           "margin": round(f["margin"], 1), "grasp over bench (mm)": f["grasp over the bench (mm)"], "edge from grasp (mm)": f["edge from the grasp point (mm)"],
                           "compliance mm/N": f["compliance mm per N"]}  # fmt: skip
                    report["smoother"].append(row)
                    print("SMOOTHER " + json.dumps(row), flush=True)
        arc = rim_arc(centre, top_z, bases["left"])
        report["rim"][str(centre)] = arc
        reached = [r["deg"] for r in arc if r["mm"] < 2.0 and r["rot deg"] < 2.0 and r["margin"] > 3.0]
        print(f"RIM {centre}: reached at {reached}", flush=True)
    (out_dir / "reach.json").write_text(json.dumps(report, indent=1, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
