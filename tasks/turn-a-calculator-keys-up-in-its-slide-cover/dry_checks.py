#!/usr/bin/env python3
"""First-hour dry checks for the calculator's slide cover (no simulator): the hand's figures against the scene's.

    PYTHONPATH=<yam-isaac-lab-arena> python3 mech/tasks/calc_cover/dry_checks.py

1. What the YAM hand needs: its collision hulls (yam_arena) in the gripper's frame, from the grasp point.
2. Pinch margins across the cover's and the body's width.
3. A level pinch across the thickness (closing vertical) near the bench: how high the grasp point must be.
4. The candidate plan's grasp points through the IK of the arm that makes them (error, joint margins), the push
   forces the arm's drives allow there, and how far the arm gives per newton at the grasp point.
All sizes of calculator and cover are (a): assumed until measured.
"""

from __future__ import annotations

import math

import numpy as np

TOP = 0.7475  # the station table (yam_arena.task_file.TABLE_TOP["station"])
NOTEBOOK_T = 0.015  # (a) an ordinary A5 hardback, about 15 mm (first run of these checks: 20 mm)
Z_NB = TOP + NOTEBOOK_T
BODY = (0.155, 0.081, 0.019)  # (a) length (y), width (x), thickness: retailers' published figures
COVER_W, COVER_TOP = 0.084, 0.022  # (a) the cover's width outside; the assembly's thickness with it
X0 = 0.24  # the calculator's middle line (spec: notebook middle at x 24 cm)
OPEN = 0.0939  # m between the pads, fully open (yam_arena: 93.9 mm)


def hand_figures():
    from yam_arena import analysis
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, FINGER_OPEN, READY, TCP_OFFSET

    joints = {**{n: float(v) for n, v in zip(ARM_JOINTS, READY)}, **dict.fromkeys(FINGER_JOINTS, FINGER_OPEN)}
    poses = analysis.fk_link_poses_world(joints, (0.0, 0.0, 0.0))
    g = poses["gripper"]
    out = {}
    for link in ("gripper", "tip_left", "tip_right"):
        c = np.vstack([(h.vertices - g[:3, 3]) @ g[:3, :3] - np.array(TCP_OFFSET) for n, h in analysis.world_hulls(poses, [link]) if n == link])
        out[link] = c
    return out


def main() -> None:
    from yam_arena import ik, station

    hulls = hand_figures()
    allc = np.vstack(list(hulls.values()))
    print("1. HAND (open, gripper frame from the grasp point; x closing, y pad width, z approach), mm:")
    for link, c in hulls.items():
        print(f"   {link:9}: closing {1000 * c[:, 0].min():6.1f}..{1000 * c[:, 0].max():6.1f}  width {1000 * c[:, 1].min():6.1f}..{1000 * c[:, 1].max():6.1f}  approach {1000 * c[:, 2].min():6.1f}..{1000 * c[:, 2].max():6.1f}")

    print("2. WIDTH PINCH MARGINS (fully open 93.9 mm):")
    for name, w in (("cover 84 mm (a)", COVER_W), ("body 81 mm (a)", BODY[1])):
        print(f"   {name}: {1000 * (OPEN - w) / 2:.2f} mm a side")

    print("3. LEVEL PINCH ACROSS THE THICKNESS (closing vertical, hand pitched nose-down by p):")
    for p_deg in (0, 5, 10, 15, 18, 20, 25):
        p = math.radians(p_deg)
        # gripper frame -> world: approach z_g level along +y, tilted down by p; closing x_g vertical
        a = np.array([0.0, math.cos(p), -math.sin(p)])
        c = np.array([0.0, math.sin(p), math.cos(p)])
        w = np.cross(a, c)
        rot = np.column_stack([c, w, a])
        low = float((allc @ rot.T)[:, 2].min())
        print(f"   p {p_deg:2d} deg: lowest hand point {1000 * low:6.1f} mm under the grasp point")
    mid = Z_NB + COVER_TOP / 2 - TOP
    print(f"   the cover-on calculator's middle on the {1000 * NOTEBOOK_T:.0f} mm notebook: {1000 * mid:.1f} mm over the bench")

    print("4. THE PLAN'S GRASP POINTS (IK of the arm that makes them):")

    def down(yaw):
        from yam_arena.demo import grip_rotation

        return grip_rotation(yaw)

    def level(approach, closing):
        a, c = np.asarray(approach, float), np.asarray(closing, float)
        return np.column_stack([c, np.cross(a, c), a])

    def roll(rot, angle):
        a = rot[:, 2]
        k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
        r = np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k
        return r @ rot

    from yam_arena.constants import READY

    stiff = np.array([80.0, 80.0, 80.0, 10.0, 10.0, 10.0])
    effort = np.array([27.0, 27.0, 27.0, 7.0, 7.0, 7.0])
    zb = Z_NB  # the notebook's top
    poses = [
        ("R", "1 holds the cover: from above, across its width, at the overhang", (X0, -0.105, zb + 0.015), down(math.pi / 2), (0, 8.0, 0)),
        ("L", "1 pushes the body's left end face (closed fingers, from above)", (X0, 0.0374, zb + 0.015), down(0.0), (0, -10.0, 0)),
        ("L", "1 ... after 50 mm", (X0, -0.0126, zb + 0.015), down(0.0), (0, -10.0, 0)),
        ("R", "1 takes the body where it sticks out, from above across its width", (X0, -0.150, zb + 0.015), down(math.pi / 2), (0, -8.0, 0)),
        ("L", "1 holds the cover from above across its width", (X0, 0.000, zb + 0.015), down(math.pi / 2), (0, 8.0, 0)),
        ("R", "1 ... has drawn the body out 107 mm", (X0, -0.257, zb + 0.015), down(math.pi / 2), (0, -8.0, 0)),
        ("R", "2 lays the body on the bench", (X0, -0.257, TOP + 0.015), down(math.pi / 2), None),
        ("L", "2 lifts the cover, from above across its width", (X0, -0.050, TOP + 0.100), down(math.pi / 2), None),
        ("R", "2 takes the cover's top at its end, level, closing vertical", (X0, -0.105, TOP + 0.098), level((0, 1, 0), (0, 0, 1)), None),
        ("R", "2 ... rolled 180 deg about its approach", (X0, -0.105, TOP + 0.098), roll(level((0, 1, 0), (0, 0, 1)), math.pi), None),
        ("L", "2 lays the cover down channel up", (X0, -0.050, zb + 0.015), down(math.pi / 2), None),
        ("R", "3 picks the body off the bench, from above across its width", (X0, -0.262, TOP + 0.015), down(math.pi / 2), None),
        ("R", "3 holds its end at the cover's mouth", (X0, -0.262, zb + 0.018), down(math.pi / 2), (0, 8.0, 0)),
        ("L", "3 holds the cover, from above across its width", (X0, 0.000, zb + 0.015), down(math.pi / 2), (0, -8.0, 0)),
        ("R", "3 ... 30 mm short of home", (X0, -0.135, zb + 0.018), down(math.pi / 2), (0, 8.0, 0)),
        ("R", "3 pushes the body's right end face home (closed fingers)", (X0, -0.1324, zb + 0.017), down(0.0), (0, 10.0, 0)),
    ]
    q_seed = {"left": np.array(READY, float), "right": np.array(READY, float)}
    for side, label, pos, rot, push in poses:
        arm = "left" if side == "L" else "right"
        base = station.BASES[arm]
        best = None
        for seed in (q_seed[arm], np.array(READY, float)):
            q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), seed, base_pos=base, iters=200)
            if best is None or pe + 0.05 * re < best[1] + 0.05 * best[2]:
                best = (q, pe, re)
        q, pe, re = best
        q_seed[arm] = q
        margin = float(np.degrees(np.minimum(q - ik.LOWER, ik.UPPER - q)).min())
        # the arm's give at the grasp point: C = J K^-1 J^T (position-driven joints as springs)
        eps, here = 1e-6, ik.tcp_pose(q, base)[0]
        jac = np.column_stack([(ik.tcp_pose(q + eps * np.eye(6)[i], base)[0] - here) / eps for i in range(6)])
        give = jac @ np.diag(1.0 / stiff) @ jac.T
        line = f"   {side} {label}: IK {1000 * pe:.2f} mm {math.degrees(re):.2f} deg, joint margin {margin:.0f} deg, j6 {math.degrees(q[5]):.0f} deg"
        line += f"; gives {1000 * give[0, 0]:.1f}/{1000 * give[1, 1]:.1f}/{1000 * give[2, 2]:.1f} mm per N in x/y/z"
        if push is not None:
            torque = jac.T @ np.asarray(push, float)
            line += f"; push {list(push)} N uses {float(np.abs(torque / effort).max()):.2f} of a drive"
        print(line)


if __name__ == "__main__":
    main()
