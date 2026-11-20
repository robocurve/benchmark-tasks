#!/usr/bin/env python3
"""Check (c) of the cupcake task, without the simulator: the grasps the job needs against the hand, on the URDF's
collision hulls and the arms' inverse kinematics, before any scene is built.

    scripts/rta simpy <id> mech/tasks/cupcake_candles/hands.py /workspace/robocurve/outputs/<id>
    PYTHONPATH=<yam-isaac-lab-arena> python3 mech/tasks/cupcake_candles/hands.py <out dir> [cup|row|seeds]   # numpy only

Distances between convex hulls are exact: GJK on the hulls' corners, the closest point of each simplex found by
trying all its faces. An overlap is given as a negative number: the smallest push along a face normal of either
hull that parts the two (an upper bound of the true depth; the depth along an edge-edge axis can be less).

Layout (cm from the midpoint between the arm bases, x away from the arms, y left; heights over the bench): board
300 x 200 x 15 mm at x 26, y 0, its long side along y; three cupcakes on it 9 cm apart along y; egg cup at x 20,
y -20. The left hand holds the cupcake, the right hand brings the candle (the cup is on the right).
"""

from __future__ import annotations

import itertools
import json
import math
import sys
from pathlib import Path

import numpy as np

# -- the things (m) ---------------------------------------------------------------------------------------------
BOARD_C, BOARD_SIZE = (0.26, 0.0), (0.20, 0.30, 0.015)
CAKES_Y = (0.09, 0.0, -0.09)
CAKE_R_TOP, CAKE_R_BASE, CAKE_H = 0.030, 0.025, 0.035
CANDLE_R, CANDLE_L = 0.0025, 0.063
CUP_C = (0.20, -0.20)
CUP_IN_R, CUP_DEPTH = 0.0225, 0.035  # inside; the cup's own wall and foot are below
CUP_WALL, CUP_FOOT_H = 0.003, 0.025  # (a) porcelain wall; the foot lifts the cup's floor this much over the bench
DEPTH_FULL = 0.015  # the candle this deep at the end of the push (spec: its top 83 mm over the board)


def rotation(closing, approach) -> np.ndarray:
    """Columns: closing axis, the pads' width axis, approach axis (demo_kit's convention, the gripper link's axes)."""
    c = np.asarray(closing, dtype=float) / np.linalg.norm(closing)
    a = np.asarray(approach, dtype=float) / np.linalg.norm(approach)
    c = c - (c @ a) * a
    c /= np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


def side_grip(azimuth_deg: float, tilt_deg: float) -> np.ndarray:
    """The hand pointing towards ``azimuth`` (deg from +x, in the bench plane), ``tilt`` below level (90: straight
    down), its closing axis level and square to that azimuth."""
    p, t = math.radians(azimuth_deg), math.radians(tilt_deg)
    approach = (math.cos(p) * math.cos(t), math.sin(p) * math.cos(t), -math.sin(t))
    return rotation((-math.sin(p), math.cos(p), 0.0), approach)


# -- convex geometry --------------------------------------------------------------------------------------------
def _closest_on_simplex(w: np.ndarray) -> tuple[np.ndarray, tuple[int, ...]]:
    """The point of conv(w) nearest the origin and the corners of the face it lies on (all faces tried)."""
    best = None
    for r in range(1, len(w) + 1):
        for idx in itertools.combinations(range(len(w)), r):
            p = w[list(idx)]
            if r == 1:
                lam = np.ones(1)
            else:
                mu = np.linalg.lstsq((p[1:] - p[0]).T, -p[0], rcond=None)[0]
                lam = np.concatenate([[1.0 - mu.sum()], mu])
                if lam.min() < -1e-12:
                    continue
            x = lam @ p
            if best is None or x @ x < best[0]:
                best = (float(x @ x), x, idx)
    return best[1], best[2]


def gjk_distance(a: np.ndarray, b: np.ndarray, tol: float = 1e-7) -> float:
    """Euclidean distance between conv(a) and conv(b) (corners as rows); 0 when they touch or overlap."""
    v = a[0] - b[0]
    w_set = np.empty((0, 3))
    for _ in range(200):
        vv = float(v @ v)
        if vv < 1e-16:
            return 0.0
        w = a[np.argmax(a @ -v)] - b[np.argmax(b @ v)]
        if math.sqrt(vv) - float(v @ w) / math.sqrt(vv) < tol:
            return math.sqrt(vv)
        w_set = np.vstack([w_set, w])
        v, idx = _closest_on_simplex(w_set)
        w_set = w_set[list(idx)]
        if len(w_set) == 4 and float(v @ v) < 1e-16:
            return 0.0
    return math.sqrt(float(v @ v))


def overlap_depth(a, b) -> float:
    """Smallest push along a face normal of either hull that parts them (both hulls with planes)."""
    normals = np.vstack([a.planes[:, :3], b.planes[:, :3]])
    pa, pb = a.vertices @ normals.T, b.vertices @ normals.T
    ov = np.minimum(pa.max(0) - pb.min(0), pb.max(0) - pa.min(0))
    return float(ov.min())


class Solid:
    """A convex solid: corners and outward planes (n, d with n.x <= d inside), like yam_arena.analysis.Hull."""

    def __init__(self, vertices: np.ndarray, planes: np.ndarray) -> None:
        self.vertices, self.planes = vertices, planes


def box(center, size) -> Solid:
    c, h = np.asarray(center, dtype=float), np.asarray(size, dtype=float) / 2
    corners = np.array(list(itertools.product(*[(-1.0, 1.0)] * 3))) * h + c
    n = np.vstack([np.eye(3), -np.eye(3)])
    d = np.concatenate([c + h, -(c - h)])
    return Solid(corners, np.c_[n, d])


def frustum(center_xy, z0, z1, r0, r1, sides: int = 32) -> Solid:
    """A level frustum (radius r0 at height z0, r1 at z1), as a prism of ``sides`` faces circumscribing the circles."""
    k = 1.0 / math.cos(math.pi / sides)  # corners on the circumscribed polygon: never smaller than the round part
    ang = 2 * math.pi * (np.arange(sides) + 0.5) / sides
    ring = np.c_[np.cos(ang), np.sin(ang)]
    cx, cy = center_xy
    lo = np.c_[cx + k * r0 * ring[:, 0], cy + k * r0 * ring[:, 1], np.full(sides, z0)]
    hi = np.c_[cx + k * r1 * ring[:, 0], cy + k * r1 * ring[:, 1], np.full(sides, z1)]
    mid = 2 * math.pi * np.arange(sides) / sides
    slope = (r0 - r1) / (z1 - z0)  # the side's outward normal leans up by this
    n_side = np.c_[np.cos(mid), np.sin(mid), np.full(sides, slope)]
    n_side /= np.linalg.norm(n_side, axis=1)[:, None]
    d_side = n_side @ np.array([cx, cy, z0]) + r0 / np.linalg.norm([1.0, slope])
    n = np.vstack([n_side, [0, 0, 1], [0, 0, -1]])
    d = np.concatenate([d_side, [z1, -z0]])
    return Solid(np.vstack([lo, hi]), np.c_[n, d])


# -- the arms ---------------------------------------------------------------------------------------------------
def solve(target, rot, base) -> dict:
    """IK for the grasp point ``target`` with the hand turned ``rot`` or turned half round its approach axis (the
    hand is the same either way; a joint limit may allow only one). From the ready pose first (what a scripted run
    reaches), other starts only if it fails; of two good answers the one farther from the joint limits."""
    from yam_arena import ik

    good, best = [], None
    for r in (rot, rot @ np.diag([-1.0, -1.0, 1.0])):
        for start in ([0.0, 1.0, 1.0, -0.6, 0.0, 0.0], [0.0, 1.6, 1.4, -0.4, 0.0, 0.0], [0.0, 0.6, 0.6, -1.2, 0.0, 0.0],
                      [0.0, 1.2, 1.8, 0.4, 0.0, 0.0], [0.0, 1.4, 1.0, -1.2, 0.0, 1.57], [0.0, 1.4, 1.0, -1.2, 0.0, -1.57]):
            q, err, rerr = ik.solve(target, ik.matrix_to_quat_xyzw(r), np.array(start), base_pos=base, iters=300)
            margin = float(np.degrees(np.minimum(q - ik.LOWER, ik.UPPER - q)).min())
            found = {"q": q, "mm": 1000 * err, "deg": math.degrees(rerr), "margin": margin}
            if best is None or found["mm"] + found["deg"] < best["mm"] + best["deg"]:
                best = found
            if found["mm"] < 0.5 and found["deg"] < 0.5:
                good.append(found)
                break
    return max(good, key=lambda f: f["margin"]) if good else best


def arm_solids(q, base, pad_gap: float, links=None) -> list[tuple[str, Solid]]:
    """The arm's hulls in the world at joints ``q``, the pads ``pad_gap`` apart (m)."""
    from yam_arena import analysis
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS

    joints = {**{n: float(v) for n, v in zip(ARM_JOINTS, q)}, **dict.fromkeys(FINGER_JOINTS, -pad_gap / 2)}
    return [(n, Solid(h.vertices, h.planes)) for n, h in analysis.world_hulls(analysis.fk_link_poses_world(joints, base), links)]


def set_distance(sa, sb) -> tuple[float, str, str]:
    """(distance m, part of a, part of b); an overlap as minus its face-normal depth."""
    box_a = np.array([[s.vertices.min(0), s.vertices.max(0)] for _, s in sa])
    box_b = np.array([[s.vertices.min(0), s.vertices.max(0)] for _, s in sb])
    gap = np.maximum(box_b[None, :, 0] - box_a[:, None, 1], box_a[:, None, 0] - box_b[None, :, 1])
    gap = np.linalg.norm(np.maximum(gap, 0.0), axis=2)
    order = np.dstack(np.unravel_index(np.argsort(gap, axis=None), gap.shape))[0]
    best, pair = math.inf, ("", "")
    for i, j in order:
        if gap[i, j] > max(best, 0.0):
            break
        d = gjk_distance(sa[i][1].vertices, sb[j][1].vertices)
        if d <= 0.0:
            d = -overlap_depth(sa[i][1], sb[j][1])
        if d < best:
            best, pair = d, (sa[i][0], sb[j][0])
    return best, pair[0], pair[1]


def scene_solids(skip_cake: int | None = None) -> list[tuple[str, Solid]]:
    from yam_arena.station import TABLE_TOP

    z_board = TABLE_TOP + BOARD_SIZE[2]
    out = [("bench", box((0.35, 0.0, TABLE_TOP - 0.03), (1.1, 1.3, 0.06))),
           ("board", box((*BOARD_C, TABLE_TOP + BOARD_SIZE[2] / 2), BOARD_SIZE))]
    for k, y in enumerate(CAKES_Y):
        if k != skip_cake:
            out.append((f"cake{k}", frustum((BOARD_C[0], y), z_board, z_board + CAKE_H, CAKE_R_BASE, CAKE_R_TOP)))
    z_cup = TABLE_TOP + CUP_FOOT_H
    out.append(("cup", frustum(CUP_C, z_cup - 0.003, z_cup + CUP_DEPTH, CUP_IN_R + CUP_WALL, CUP_IN_R + CUP_WALL)))
    out.append(("cup_foot", frustum(CUP_C, TABLE_TOP, z_cup, 0.022, 0.012)))
    return out


HAND_LINKS = ["link4", "link5", "gripper", "tip_left", "tip_right"]  # checked against the things; the whole arms against each other


def report_pair(label, held, push, rows):
    d, la, lb = set_distance(held["solids"], push["solids"])
    row = {"case": label, "hands_mm": round(1000 * d, 1), "where": f"left {la} / right {lb}",
           "left_ik": [round(held["ik"]["mm"], 2), round(held["ik"]["deg"], 2), round(held["ik"]["margin"], 1)],
           "right_ik": [round(push["ik"]["mm"], 2), round(push["ik"]["deg"], 2), round(push["ik"]["margin"], 1)]}
    for side, hand in (("left", held), ("right", push)):
        d2, l2, part = set_distance([s for s in hand["solids"] if s[0] in HAND_LINKS], hand["scene"])
        row[f"{side}_scene_mm"] = round(1000 * d2, 1)
        row[f"{side}_scene_where"] = f"{l2} / {part}"
    rows.append(row)
    print(f"{label:58s} hands {row['hands_mm']:7.1f} mm ({row['where']}); scene L {row['left_scene_mm']:6.1f} "
          f"({row['left_scene_where']}) R {row['right_scene_mm']:6.1f} ({row['right_scene_where']}); "
          f"ik L {row['left_ik']} R {row['right_ik']}", flush=True)


def main(out_dir: str) -> None:
    from yam_arena.station import BASES, TABLE_TOP

    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    z_board = TABLE_TOP + BOARD_SIZE[2]
    rows: list[dict] = []
    cake_i = 1  # the middle cupcake, y 0
    cx, cy = BOARD_C[0], CAKES_Y[cake_i]
    cake = frustum((cx, cy), z_board, z_board + CAKE_H, CAKE_R_BASE, CAKE_R_TOP)
    scene_l = scene_solids(skip_cake=cake_i)  # the held cupcake is touched by design: checked on its own below
    scene_r = scene_solids()

    def right_hand(grip_below_top, depth, closing):
        top = z_board + CAKE_H - depth + CANDLE_L
        target = np.array([cx, cy, top - grip_below_top])
        rot = rotation(closing, (0.0, 0.0, -1.0))
        ik = solve(target, rot, BASES["right"])
        sol = arm_solids(ik["q"], BASES["right"], 2 * CANDLE_R)
        # the pushing hand may touch the held cupcake only through the candle: check it against the cupcake too
        return {"ik": ik, "solids": sol, "scene": scene_r}

    def left_hand(rot, height, pad_gap):
        target = np.array([cx, cy, z_board + height])
        ik = solve(target, rot, BASES["left"])
        sol = arm_solids(ik["q"], BASES["left"], pad_gap)
        return {"ik": ik, "solids": sol, "scene": scene_l}

    rim_gap = 2 * CAKE_R_TOP + 0.002  # pads 1 mm off the rim on each side

    print("== 1. both hands pointing down at one cupcake (the spec's way) ==", flush=True)
    for close_l, close_r, name in (((0, 1, 0), (1, 0, 0), "L closes along y, R along x"),
                                   ((1, 0, 0), (0, 1, 0), "L closes along x, R along y"),
                                   ((0, 1, 0), (0, 1, 0), "both close along y")):
        held = left_hand(rotation(close_l, (0, 0, -1)), CAKE_H / 2, rim_gap)
        for depth, tag in ((0.0, "push start"), (DEPTH_FULL, "push end")):
            push = right_hand(0.030, depth, close_r)
            report_pair(f"down/down {name}, {tag}", held, push, rows)

    print("== 2. the holding hand from the side, tilted; the right hand straight down ==", flush=True)
    for azim in (0.0, -45.0, -90.0, 45.0, 180.0):
        for tilt in (0.0, 15.0, 30.0, 45.0, 60.0, 75.0):
            rot = side_grip(azim, tilt)
            # lowest grasp height (over the board) at which the left hand clears the board and bench by 2 mm
            # lowest grasp height (over the board) at which the left hand clears the board and bench by 2 mm: from the
            # gripper's and fingers' corners first, then raised until the whole hand (with its wrist) clears
            from yam_arena import analysis
            from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, READY, TCP_OFFSET
            jo = {**{n: float(v) for n, v in zip(ARM_JOINTS, READY)}, **dict.fromkeys(FINGER_JOINTS, -rim_gap / 2)}
            poses = analysis.fk_link_poses_world(jo, (0.0, 0.0, 0.0))
            g = poses["gripper"]
            local = np.vstack([hh.vertices for _, hh in analysis.world_hulls(poses, ["gripper", "tip_left", "tip_right"])])
            local = (local - g[:3, 3]) @ g[:3, :3] - np.array(TCP_OFFSET)
            height = max(0.002 - float((local @ rot.T)[:, 2].min()), 0.004)
            for _ in range(12):
                held = left_hand(rot, height, rim_gap)
                d, _, _ = set_distance([x for x in held["solids"] if x[0] in HAND_LINKS], scene_l[:2])
                if d >= 0.0019:
                    break
                height += 0.0015
            if held["ik"]["mm"] > 1.0 or held["ik"]["deg"] > 1.0 or d < 0.0019 or height > CAKE_H - 0.002:
                print(f"azimuth {azim:6.1f} tilt {tilt:4.1f}: h {1000 * height:.1f} mm, board {1000 * d:.1f} mm, "
                      f"ik {held['ik']['mm']:.2f} mm {held['ik']['deg']:.2f} deg: not a grasp", flush=True)
                continue
            held = left_hand(rot, height, rim_gap)
            # where on the finger the rim is met (hand frame, from the grasp point): (along approach, along pad width)
            v = np.array([0.0, 0.0, CAKE_H - height])
            rim = (round(1000 * float(v @ rot[:, 2]), 1), round(1000 * float(v @ rot[:, 1]), 1))
            d_cake, part, _ = set_distance(held["solids"], [("held", cake)])
            for close_r in ((1, 0, 0), (0, 1, 0)):
                for depth, tag in ((0.0, "start"), (DEPTH_FULL, "end")):
                    push = right_hand(0.030, depth, close_r)
                    label = f"side az {azim:+.0f} tilt {tilt:.0f} h {1000 * height:.1f} R closes {'x' if close_r[0] else 'y'} {tag}"
                    report_pair(label, held, push, rows)
                    rows[-1].update({"azimuth": azim, "tilt": tilt, "grasp_height_mm": round(1000 * height, 1),
                                     "rim_on_finger_mm": rim, "left_to_held_cake_mm": round(1000 * d_cake, 1),
                                     "left_held_part": part})

    (out / "hands.json").write_text(json.dumps(rows, indent=1))
    print(f"wrote {out / 'hands.json'}")




def cup_check(out_dir: str) -> None:
    """The right hand pinching one candle among four standing upright in the egg cup: the hand's clearance to the
    other three candles and to the cup's rim and wall, for the candles' spacing (a square of side ``s`` round the
    cup's middle), the turn of the closing axis, the pads' opening and how far below the wick end it pinches."""
    from yam_arena.station import BASES, TABLE_TOP

    out = Path(out_dir)
    z_floor = TABLE_TOP + CUP_FOOT_H
    top = z_floor + CANDLE_L  # the candles stand on the cup's floor
    wall = []  # the cup's wall as 24 convex blocks round the inside
    for k in range(24):
        a = 2 * math.pi * k / 24
        r = CUP_IN_R + CUP_WALL / 2
        c = (CUP_C[0] + r * math.cos(a), CUP_C[1] + r * math.sin(a), z_floor + CUP_DEPTH / 2)
        blk = box((0.0, 0.0, 0.0), (CUP_WALL, 2 * math.pi * r / 24 + 0.001, CUP_DEPTH))
        rz = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])
        wall.append(("cup", Solid(blk.vertices @ rz.T + c, np.c_[blk.planes[:, :3] @ rz.T, blk.planes[:, 3] + blk.planes[:, :3] @ rz.T @ c])))
    rows = []
    print("== 3. one candle out of four standing in the egg cup (rim 28 mm under the wick ends) ==", flush=True)
    for s in (0.012, 0.015, 0.018, 0.021, 0.024):
        h = s / 2
        spots = [(CUP_C[0] + dx * h, CUP_C[1] + dy * h) for dx, dy in ((1, 1), (-1, 1), (-1, -1), (1, -1))]
        if math.hypot(h, h) + CANDLE_R > CUP_IN_R:
            continue
        target = spots[0]
        others = [(f"candle{k}", frustum(p, z_floor, top, CANDLE_R, CANDLE_R, 16)) for k, p in enumerate(spots) if k]
        for turn in (0.0, 45.0, 90.0, 135.0):
            closing = (math.cos(math.radians(turn)), math.sin(math.radians(turn)), 0.0)
            for below in (0.010, 0.015, 0.020, 0.030):
                for opening in (0.009, 0.011, 0.015):
                    ik = solve(np.array([*target, top - below]), rotation(closing, (0, 0, -1)), BASES["right"])
                    hand = [x for x in arm_solids(ik["q"], BASES["right"], opening) if x[0] in HAND_LINKS]
                    d_c, l_c, p_c = set_distance(hand, others)
                    d_w, l_w, _ = set_distance(hand, wall)
                    rows.append({"side_mm": 1000 * s, "turn": turn, "below_top_mm": 1000 * below, "opening_mm": 1000 * opening,
                                 "candles_mm": round(1000 * d_c, 1), "candle_part": l_c, "cup_mm": round(1000 * d_w, 1),
                                 "cup_part": l_w, "ik": [round(ik["mm"], 2), round(ik["deg"], 2), round(ik["margin"], 1)]})
                    r = rows[-1]
                    print(f"square {r['side_mm']:4.0f} mm turn {turn:5.1f} pinch {r['below_top_mm']:4.0f} mm under the top, pads "
                          f"{r['opening_mm']:4.1f} mm: other candles {r['candles_mm']:6.1f} ({l_c}), cup {r['cup_mm']:6.1f} ({l_w}); ik {r['ik']}", flush=True)
    (out / "cup.json").write_text(json.dumps(rows, indent=1))



def row_check(out_dir: str) -> None:
    """The lead's layout (2026-11-20): the three cupcakes in a row along y, ``gap`` from the board's near long edge,
    so that the holding hand's finger carriers and body hang over the bench. The left hand comes from the near
    side (pointing +x, closing along y), tilted ``tilt`` below level, at the height that puts the rim on its pads.
    Reported per cupcake: the hands' distance (right hand straight down, closing along x or y, candle pinched 20 or
    30 mm under its wick end, push start and end), the left hand's distance to board, bench and neighbours (pads at
    the hold, 62 mm, and at the approach opening, 72 mm), where the rim meets the finger, the pads' lowest edge over
    the board, and the closing past touch at which the pads' contact reaches that edge (1 mm of radius per 7 mm of
    height: 3.5 mm down the wall per mm of closing)."""
    from yam_arena.station import BASES, TABLE_TOP

    out = Path(out_dir)
    z_board = TABLE_TOP + BOARD_SIZE[2]
    cake_x = 0.26
    rows = []
    print("== 4. cupcakes near the board's near long edge; left hand from the near side ==", flush=True)
    bench = box((0.35, 0.0, TABLE_TOP - 0.03), (1.1, 1.3, 0.06))
    for spacing in (0.09,):
        cakes = [frustum((cake_x, y), z_board, z_board + CAKE_H, CAKE_R_BASE, CAKE_R_TOP) for y in (spacing, 0.0, -spacing)]
        for k, cy in enumerate((spacing, 0.0, -spacing)):
            neighbours = [(f"cake{j}", c) for j, c in enumerate(cakes) if j != k]
            right = {}  # the right hand straight down over this cupcake: (grip, closing, depth) -> hulls
            for grip in (0.020, 0.030):
                for close_r in ((1, 0, 0), (0, 1, 0)):
                    for depth in (0.0, DEPTH_FULL):
                        top = z_board + CAKE_H - depth + CANDLE_L
                        ikr = solve(np.array([cake_x, cy, top - grip]), rotation(close_r, (0, 0, -1)), BASES["right"])
                        right[grip, close_r, depth] = (arm_solids(ikr["q"], BASES["right"], 2 * CANDLE_R), round(ikr["mm"], 2))
            for tilt in (10.0, 15.0, 20.0):
                rot = side_grip(0.0, tilt)
                t = math.radians(tilt)
                h = CAKE_H - min(0.012 / math.cos(t), 0.0021 / max(math.sin(t), 1e-9))  # the rim 1 mm inside the pad's edges
                ik = solve(np.array([cake_x, cy, z_board + h]), rot, BASES["left"])
                if ik["mm"] > 0.5 or ik["deg"] > 0.5:
                    print(f"cake {k} tilt {tilt}: no IK ({ik['mm']:.2f} mm {ik['deg']:.2f} deg)", flush=True)
                    continue
                full = arm_solids(ik["q"], BASES["left"], 2 * CAKE_R_TOP)
                hold = [x for x in full if x[0] in HAND_LINKS]
                wide = [x for x in arm_solids(ik["q"], BASES["left"], 0.072) if x[0] in HAND_LINKS]
                d_bench, l_bench, _ = set_distance(hold, [("bench", bench)])
                board = {}
                for gap in (0.030, 0.035, 0.040):
                    plank = box((cake_x - gap + BOARD_SIZE[0] / 2, 0.0, TABLE_TOP + BOARD_SIZE[2] / 2), BOARD_SIZE)
                    d1, l1, _ = set_distance(hold, [("board", plank)])
                    d2, _, _ = set_distance(wide, [("board", plank)])
                    board[f"{1000 * gap:.0f}"] = (round(1000 * d1, 1), l1, round(1000 * d2, 1))
                d_nb, l_nb, p_nb = set_distance(hold, neighbours)
                d_nbw, l_nbw, p_nbw = set_distance(wide, neighbours)
                v = np.array([0.0, 0.0, CAKE_H - h])
                rim = (round(1000 * float(v @ rot[:, 2]), 1), round(1000 * float(v @ rot[:, 1]), 1))
                pad_low = h - (0.0128 * math.sin(t) + 0.0132 * math.cos(t))  # the pads' lower corner (z 12.8, y 13.2 mm)
                reach_close = 2 * (CAKE_H - max(pad_low, 0.0)) / 7.0  # closing past touch for contact down to it
                hands = {}
                for (grip, close_r, depth), (sol, ik_mm) in right.items():
                    d, la, lb = set_distance(full, sol)
                    hands[f"grip {1000 * grip:.0f} closes {'x' if close_r[0] else 'y'} {'end' if depth else 'start'}"] = (round(1000 * d, 1), la, lb, ik_mm)
                row = {"spacing_mm": 1000 * spacing, "cake": k, "y_mm": 1000 * cy, "tilt": tilt, "grasp_h_mm": round(1000 * h, 1),
                       "rim_on_finger_mm": rim, "pad_low_mm": round(1000 * pad_low, 1), "closing_to_reach_pad_low_mm": round(1000 * reach_close, 1),
                       "board_by_edge_gap_mm (hold, part, wide)": board, "bench_mm": round(1000 * d_bench, 1), "bench_part": l_bench,
                       "neighbour_hold_mm": round(1000 * d_nb, 1), "neighbour_hold_part": f"{l_nb}/{p_nb}",
                       "neighbour_wide_mm": round(1000 * d_nbw, 1), "neighbour_wide_part": f"{l_nbw}/{p_nbw}",
                       "hands_mm": hands, "left_ik_margin": round(ik["margin"], 1)}
                rows.append(row)
                worst = min(hands.items(), key=lambda kv: kv[1][0])
                print(f"cake y {row['y_mm']:+.0f} tilt {tilt:.0f}: grasp {row['grasp_h_mm']} mm, rim on finger {rim}, pad low {row['pad_low_mm']}, "
                      f"close to reach it {row['closing_to_reach_pad_low_mm']} | board by gap {board} | bench {row['bench_mm']} ({l_bench}) | "
                      f"neighbours hold {row['neighbour_hold_mm']} wide {row['neighbour_wide_mm']} | hands min {worst[1][0]} at {worst[0]} ({worst[1][1]}/{worst[1][2]}); "
                      f"all {dict((k2, v2[0]) for k2, v2 in hands.items())} | L margin {row['left_ik_margin']}", flush=True)
    (out / "row.json").write_text(json.dumps(rows, indent=1))


def seed_check(out_dir: str) -> None:
    """Every start the seeds can draw (the row up to 1 cm along the board's edge, each cupcake up to 7 mm more): the
    trial seed the harness gives for --seed 0; both arms' reach to every pose of the script's hold and push at each
    cupcake moved 17 mm along the row and 7 mm across it; the holding hand, centred on a cupcake whose neighbours are
    7.6 cm away on both sides (the nearest a seed can bring them), against them and its own cupcake."""
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import demo
    from yam_arena import task_file
    from yam_arena.station import BASES

    m = task_file.load(str(Path(__file__).resolve().parent / "task.py")).module
    out = {}
    rng = np.random.default_rng(1415158438)
    p0, p1 = m.placement(0), m.placement(1415158438)
    out["trial seed 1415158438 (row shift mm; each cupcake dx, dy, off mm)"] = (
        round(1000 * float(rng.uniform(-0.010, 0.010)), 1),
        {k: (round(1000 * (p1[k][0] - p0[k][0]), 1), round(1000 * (p1[k][1] - p0[k][1]), 1), round(1000 * math.dist(p1[k], p0[k]), 1)) for k in p0})
    a = demo.hold_rot()[:, 2]
    reach = {"left": [], "right": []}
    for k, (x0, y0) in p0.items():
        for dx, dy in itertools.product((-0.007, 0.0, 0.007), (-0.017, 0.0, 0.017)):
            grasp = np.array([x0 + dx, y0 + dy, m.BOARD_TOP + 0.0269])
            for label, target in (("back", grasp - 0.08 * a + np.array([0.0, 0.0, 0.04])), ("front", grasp - 0.03 * a), ("hold", grasp)):
                s = solve(target, demo.hold_rot(), BASES["left"])
                reach["left"].append((s["mm"], s["deg"], s["margin"], f"{k} {1000 * dx:+.0f} {1000 * dy:+.0f} {label}"))
            for label, above in (("over", 0.015), ("onto", 0.004), ("pushed 24 mm in", -0.0242)):
                tcp = np.array([x0 + dx, y0 + dy, m.BOARD_TOP + m.CAKE_H + above + m.cd.CANDLE_L - demo.PINCH_BELOW_TOP])
                s = solve(tcp, demo.down_rot(), BASES["right"])
                reach["right"].append((s["mm"], s["deg"], s["margin"], f"{k} {1000 * dx:+.0f} {1000 * dy:+.0f} {label}"))
    for side, rows in reach.items():
        out[f"{side}: worst IK mm, deg; least joint margin deg, where"] = (
            round(max(r[0] for r in rows), 2), round(max(r[1] for r in rows), 3), round(min(r[2] for r in rows), 1), min(rows, key=lambda r: r[2])[3])
    x, y, z = 0.26, -0.03, m.BOARD_TOP
    room = {}
    for gap_mm in (75.0, 68.0, 58.5):
        for label, back in (("in front", 0.03), ("at the rim", 0.0)):
            s = solve(np.array([x, y, z + 0.0269]) - back * a, demo.hold_rot(), BASES["left"])
            hand = [h for h in arm_solids(s["q"], BASES["left"], gap_mm / 1000) if h[0] in HAND_LINKS]
            nb = [(f"next {dy:+.3f}", frustum((x, y + dy), z, z + CAKE_H, CAKE_R_BASE, CAKE_R_TOP)) for dy in (0.076, -0.076)]
            d_nb, l_nb, _ = set_distance(hand, nb)
            d_own, l_own, _ = set_distance(hand, [("own", frustum((x, y), z, z + CAKE_H, CAKE_R_BASE, CAKE_R_TOP))])
            room[f"pads {gap_mm:.1f} mm {label}"] = {"next cupcake mm": round(1000 * d_nb, 1), "part": l_nb, "own cupcake mm": round(1000 * d_own, 1), "own part": l_own}
    out["holding hand centred, neighbours 7.6 cm away"] = room
    Path(out_dir).mkdir(parents=True, exist_ok=True)
    (Path(out_dir) / "seeds.json").write_text(json.dumps(out, indent=1))
    print(json.dumps(out, indent=1), flush=True)


if __name__ == "__main__":
    if len(sys.argv) > 2 and sys.argv[2] == "cup":
        cup_check(sys.argv[1])
    elif len(sys.argv) > 2 and sys.argv[2] == "row":
        row_check(sys.argv[1])
    elif len(sys.argv) > 2 and sys.argv[2] == "seeds":
        seed_check(sys.argv[1])
    else:
        main(sys.argv[1] if len(sys.argv) > 1 else "outputs/w7c-hands")
