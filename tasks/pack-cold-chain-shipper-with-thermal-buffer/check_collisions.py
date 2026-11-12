"""Offline collision, finger-body, and fixture clearance audit for pack-cold-chain-shipper-with-thermal-buffer.

Every obstacle is built in WORLD coordinates (table top at z = 0.7475 m) from geometry.py:
- static: table, shipper floor and four walls, three staging pads;
- parts: gel brick, buffer pad, medication carton, each at its staging pad until the arm grasps it, carried
  (box under the TCP) until the release starts, then seated in the shipper.

Two audits run for every arm and every demo label:
1. Gripper links (URDF convex hulls) against static obstacles and against every part that is not being carried
   by that arm.
2. A finger-body model: two boxes of 12 mm x 24 mm cross-section reaching 40 mm up from the pad tips, centred at
   +/- (half the pad gap + 6 mm) from the TCP along the closing axis, against the shipper walls, pads and every
   part that is not being carried. While a part is held, or being released, the pad gap is at least the part
   width (the fingers cannot close through it).
Carried parts are also checked against the four shipper walls.

Minimum clearance for everything: 4 mm. Arm-to-arm link distance must exceed 50 mm.
"""

from __future__ import annotations

import argparse
import os
import sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
if "YAM_ARENA_SRC" in os.environ:
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA_SRC"]))
elif os.path.exists(os.path.expanduser("~/Projects/Robocurve/work/refs/yam-isaac-lab-arena")):
    sys.path.insert(0, os.path.expanduser("~/Projects/Robocurve/work/refs/yam-isaac-lab-arena"))

MIN_CLEAR_MM = 4.0          # every unintended scene clearance must be at least 4 mm
ARM_ARM_MIN_MM = 50.0       # bimanual arm-arm minimum separation (> 50 mm)
DT = 0.05
EYE = np.eye(3)
WALLS = ("shipper_front", "shipper_rear", "shipper_left", "shipper_right")


def part_sizes() -> dict[str, np.ndarray]:
    """Part box sizes (X, Y, Z) in metres."""
    import geometry as g

    return {
        "brick": np.array([g.BRICK_WIDTH_X, g.BRICK_LENGTH_Y, g.BRICK_HEIGHT_Z]),
        "buffer": np.array([g.BUFFER_WIDTH_X, g.BUFFER_LENGTH_Y, g.BUFFER_HEIGHT_Z]),
        "carton": np.array([g.CARTON_WIDTH_X, g.CARTON_LENGTH_Y, g.CARTON_HEIGHT_Z]),
    }


def static_obstacles():
    """Return dict of static boxes (centre, size) in world coordinates, built from geometry.py."""
    import geometry as g

    wall_z = g.CAVITY_FLOOR_Z + g.CAVITY_DEPTH_Z / 2.0
    boxes = {
        "table": (np.array([0.300, 0.000, g.TABLE_Z - 0.030]), np.array([1.100, 1.300, 0.060])),
        "shipper_floor": (
            np.array([g.SHIPPER_X, g.SHIPPER_Y, g.TABLE_Z + g.SHIPPER_FLOOR_THICKNESS / 2.0]),
            np.array([g.SHIPPER_EXT_X, g.SHIPPER_EXT_Y, g.SHIPPER_FLOOR_THICKNESS]),
        ),
        "shipper_front": (
            np.array([g.SHIPPER_X - g.CAVITY_WIDTH_X / 2.0 - g.SHIPPER_WALL_THICKNESS / 2.0, g.SHIPPER_Y, wall_z]),
            np.array([g.SHIPPER_WALL_THICKNESS, g.SHIPPER_EXT_Y, g.CAVITY_DEPTH_Z]),
        ),
        "shipper_rear": (
            np.array([g.SHIPPER_X + g.CAVITY_WIDTH_X / 2.0 + g.SHIPPER_WALL_THICKNESS / 2.0, g.SHIPPER_Y, wall_z]),
            np.array([g.SHIPPER_WALL_THICKNESS, g.SHIPPER_EXT_Y, g.CAVITY_DEPTH_Z]),
        ),
        "shipper_left": (
            np.array([g.SHIPPER_X, g.SHIPPER_Y + g.CAVITY_LENGTH_Y / 2.0 + g.SHIPPER_WALL_THICKNESS / 2.0, wall_z]),
            np.array([g.CAVITY_WIDTH_X, g.SHIPPER_WALL_THICKNESS, g.CAVITY_DEPTH_Z]),
        ),
        "shipper_right": (
            np.array([g.SHIPPER_X, g.SHIPPER_Y - g.CAVITY_LENGTH_Y / 2.0 - g.SHIPPER_WALL_THICKNESS / 2.0, wall_z]),
            np.array([g.CAVITY_WIDTH_X, g.SHIPPER_WALL_THICKNESS, g.CAVITY_DEPTH_Z]),
        ),
    }
    for name, (x, y, z) in (
        ("pad_left", (g.PAD_LEFT_X, g.PAD_LEFT_Y, g.PAD_LEFT_Z)),
        ("pad_right", (g.PAD_RIGHT_X, g.PAD_RIGHT_Y, g.PAD_RIGHT_Z)),
        ("pad_front", (g.PAD_FRONT_X, g.PAD_FRONT_Y, g.PAD_FRONT_Z)),
    ):
        boxes[name] = (np.array([x, y, z]), np.array([g.PAD_SIZE_X, g.PAD_SIZE_Y, g.PAD_THICKNESS]))
    cyls = {}
    return boxes, cyls


def box_dist(points, centre, size):
    from yam_arena import analysis

    return analysis.box_signed_distance(points, np.asarray(centre), EYE, np.asarray(size))


def timelines(plans):
    """{arm: [(label, t0, t1), ...]} from the waypoint durations."""
    out = {}
    for arm, wps in plans.items():
        t, rows = 0.0, []
        for w in wps:
            rows.append((w.label, t, t + w.duration))
            t += w.duration
        out[arm] = rows
    return out


def part_schedule(plans, variant: str):
    """Per part: carrying arm, start, grasp start, release start, release end, final seat bottom z (None = never moved)."""
    import demo_plan
    import geometry as g

    lay = demo_plan.layout(variant)
    tl = timelines(plans)
    sched = {}
    for name in demo_plan.PART_NAMES:
        arm = lay["arms"][name]
        row = {"arm": arm, "xy": lay["staging"][name], "t_grasp": None, "t_release": None, "t_release_end": None, "seat_bottom": None}
        for label, t0, t1 in tl[arm]:
            if label == f"grasp_{name}":
                row["t_grasp"] = t0
            if label == f"release_{name}":
                row["t_release"], row["t_release_end"] = t0, t1
        if row["t_release"] is not None:
            wp = next(w for w in plans[arm] if w.label == f"release_{name}")
            row["seat_bottom"] = wp.pos[2] - g.GRIP_DZ - g.PLACE_DROP
            row["seat_xy"] = (wp.pos[0], wp.pos[1])
        sched[name] = row
    return sched


def part_boxes(sched, t: float, tcps: dict):
    """{name: (centre, size, state)} at time ``t``; state is staged, carried or seated."""
    import geometry as g

    sizes = part_sizes()
    out = {}
    for name, row in sched.items():
        size = sizes[name]
        if row["t_grasp"] is None or t < row["t_grasp"]:
            x, y = row["xy"]
            bottom = g.PAD_SURFACE_Z + g.SPAWN_GAP
            state = "staged"
        elif t < row["t_release"]:
            tcp = tcps[row["arm"]]
            x, y, bottom = float(tcp[0]), float(tcp[1]), float(tcp[2]) - g.GRIP_DZ
            state = "carried"
        else:
            x, y = row["seat_xy"]
            bottom = row["seat_bottom"]  # the 2 mm drop at release is ignored: the part is treated as seated
            state = "seated"
        out[name] = (np.array([x, y, bottom + size[2] / 2.0]), size, state)
    return out


def finger_points(tcp, rot, gap_m):
    """Sample points of both finger bodies. The closing axis is the gripper X axis (horizontal part)."""
    import geometry as g

    c = np.asarray(rot)[:, 0].copy()
    c[2] = 0.0
    c /= np.linalg.norm(c)
    a = np.cross([0.0, 0.0, 1.0], c)
    u = np.linspace(-g.FINGER_THICK / 2.0, g.FINGER_THICK / 2.0, 5)
    v = np.linspace(-g.FINGER_WIDTH / 2.0, g.FINGER_WIDTH / 2.0, 9)
    w = np.linspace(0.0, g.FINGER_BODY_LEN, 15)
    uu, vv, ww = np.meshgrid(u, v, w, indexing="ij")
    local = np.stack([uu.ravel(), vv.ravel(), ww.ravel()], axis=1)
    tip_z = tcp[2] - g.PAD_TIP_PAST_TCP
    pts = []
    for sign in (-1.0, 1.0):
        centre = np.array([tcp[0], tcp[1], tip_z]) + sign * c * (gap_m / 2.0 + g.FINGER_THICK / 2.0)
        pts.append(centre + local[:, :1] * c + local[:, 1:2] * a + local[:, 2:3] * np.array([0.0, 0.0, 1.0]))
    return np.vstack(pts)


def trace_arm(arm: str, mode: str = "demo", variant: str = "A", plans=None):
    import demo_plan
    from yam_arena import analysis, ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS
    from yam_arena.demo import ArmPlan

    plans = plans or demo_plan.plans(mode, variant=variant)
    waypoints = plans[arm]
    base = station.BASES[arm]
    plan = ArmPlan(list(waypoints), base)
    out, t = [], 0.0
    while t <= plan.duration + 1e-9:
        pos, rot, finger, label = plan.target(t)
        plan.action7(t)
        p, r = ik.tcp_pose(plan.q, base)
        pe = float(np.linalg.norm(p - pos) * 1000)
        re = float(np.degrees(np.arccos(np.clip((np.trace(rot.T @ r) - 1) / 2, -1, 1))))
        margin = float(min((plan.q - ik.LOWER).min(), (ik.UPPER - plan.q).min()))
        q = {n: float(v) for n, v in zip(ARM_JOINTS, plan.q, strict=True)}
        q.update({n: float(finger) for n in FINGER_JOINTS})
        poses = analysis.fk_link_poses_world(q, base)
        hulls = analysis.world_hulls(poses)
        pts = np.vstack([h.vertices for _, h in hulls])
        who = np.concatenate([[name] * len(h.vertices) for name, h in hulls])
        out.append({
            "t": t,
            "label": label,
            "poses": poses,
            "pts": pts,
            "who": who,
            "pe": pe,
            "re": re,
            "margin": margin,
            "tcp": p.copy(),
            "rot": r.copy(),
            "finger": float(finger),
        })
        t += DT
    return out


def analyse(arm: str, samples, sched, other_tcps):
    """Per-label worst clearances for one arm; ``other_tcps`` gives the other arm's TCP at each sample index."""
    import geometry as g

    boxes, _cyls = static_obstacles()
    sizes = part_sizes()
    rep: dict[str, dict] = {}
    bad: list[str] = []

    for i, s in enumerate(samples):
        label, t = s["label"], s["t"]
        r = rep.setdefault(
            label,
            {"pe": 0.0, "re": 0.0, "margin": 9.0, "min": (np.inf, ""), "finger": (np.inf, ""), "carried": (np.inf, ""), "t": t},
        )
        r["pe"], r["re"], r["margin"] = max(r["pe"], s["pe"]), max(r["re"], s["re"]), min(r["margin"], s["margin"])

        tcps = {arm: s["tcp"], ("right" if arm == "left" else "left"): other_tcps[i]}
        parts = part_boxes(sched, t, tcps)

        # obstacles this arm may NOT touch: statics plus every part that this arm is not holding or releasing
        obstacles = dict(boxes)
        holding = None
        for name, (c, size, state) in parts.items():
            row = sched[name]
            in_hold_window = (
                row["arm"] == arm
                and row["t_grasp"] is not None
                and row["t_grasp"] <= t <= row["t_release_end"]
            )
            if in_hold_window:
                holding = name
                continue
            obstacles[f"part_{name}"] = (c, size)

        # (1) gripper link hulls
        pts, who = s["pts"], s["who"]
        for obj, (c, size) in obstacles.items():
            d_all = box_dist(pts, c, size)
            k = int(np.argmin(d_all))
            link = str(who[k])
            if link == "base" and obj == "table":
                continue
            d_mm = float(d_all[k] * 1000)
            if d_mm < r["min"][0]:
                r["min"] = (d_mm, f"{link} vs {obj}")

        # (2) finger bodies
        gap = -2.0 * s["finger"]
        if holding is not None:
            gap = max(gap, float(sizes[holding][0]))
        fpts = finger_points(s["tcp"], s["rot"], gap)
        for obj, (c, size) in obstacles.items():
            d_mm = float(box_dist(fpts, c, size).min() * 1000)
            if d_mm < r["finger"][0]:
                r["finger"] = (d_mm, f"finger vs {obj}")

        # (3) the part this arm is carrying against the shipper walls
        if holding is not None and parts[holding][2] == "carried":
            c, size, _ = parts[holding]
            corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * (size / 2.0) + c
            for wall in WALLS:
                wc, wsize = boxes[wall]
                d_mm = float(box_dist(corners, wc, wsize).min() * 1000)
                if d_mm < r["carried"][0]:
                    r["carried"] = (d_mm, f"carried {holding} vs {wall}")

        r["t"] = t

    for label, r in rep.items():
        for key, tag in (("min", "gripper link"), ("finger", "finger body"), ("carried", "carried part")):
            d, who_obj = r[key]
            if key != "carried" and not np.isfinite(d):
                bad.append(f"{arm} at '{label}': no {tag} clearance computed")
            elif d < MIN_CLEAR_MM:
                bad.append(f"{arm} at '{label}': {tag} clearance {d:.2f} mm < {MIN_CLEAR_MM} mm ({who_obj})")
    return rep, bad


def check_arm_arm(left_samples, right_samples):
    n = min(len(left_samples), len(right_samples))
    min_dist_mm = np.inf
    worst_t = 0.0
    worst_pair = ("", "")

    for i in range(n):
        ls = left_samples[i]
        rs = right_samples[i]
        t = ls["t"]
        poses_l = ls["poses"]
        poses_r = rs["poses"]

        step_min_d = np.inf
        step_pair = ("", "")
        for nl, pl in poses_l.items():
            if nl == "base":
                continue
            pos_link_l = pl[:3, 3]
            for nr, pr in poses_r.items():
                if nr == "base":
                    continue
                pos_link_r = pr[:3, 3]
                d = float(np.linalg.norm(pos_link_l - pos_link_r) * 1000.0)
                if d < step_min_d:
                    step_min_d = d
                    step_pair = (f"L:{nl}", f"R:{nr}")

        if step_min_d < min_dist_mm:
            min_dist_mm = step_min_d
            worst_t = t
            worst_pair = step_pair

    return min_dist_mm, worst_t, worst_pair


def _fmt(d: float) -> str:
    return f"{d:7.2f}" if np.isfinite(d) else "    n/a"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mode", default="demo", help="demo mode to check")
    ap.add_argument("--variant", choices=["A", "B"], default="A", help="task variant")
    ap.add_argument("--yam-arena", help="yam_arena path override")
    args = ap.parse_args()

    if args.yam_arena:
        sys.path.insert(0, os.path.expanduser(args.yam_arena))

    os.environ["TASK_VARIANT"] = args.variant
    import demo_plan

    plans = demo_plan.plans(args.mode, variant=args.variant)
    sched = part_schedule(plans, args.variant)

    print(f"Tracing arm trajectories for mode: {args.mode} (Variant {args.variant})...")
    l_samples = trace_arm("left", mode=args.mode, variant=args.variant, plans=plans)
    r_samples = trace_arm("right", mode=args.mode, variant=args.variant, plans=plans)

    print("Auditing left arm scene clearances (links, finger bodies, carried part)...")
    l_rep, l_bad = analyse("left", l_samples, sched, [s["tcp"] for s in r_samples])
    print("Auditing right arm scene clearances (links, finger bodies, carried part)...")
    r_rep, r_bad = analyse("right", r_samples, sched, [s["tcp"] for s in l_samples])

    print("Auditing bimanual arm-arm clearance...")
    arm_arm_min, arm_arm_t, arm_arm_pair = check_arm_arm(l_samples, r_samples)

    overall_min_scene = np.inf
    worst_scene_str = ""
    overall_min_finger = np.inf
    worst_finger_str = ""
    for arm, rep in (("left", l_rep), ("right", r_rep)):
        print(f"\n=== {arm.capitalize()} Arm Scene Clearance Summary (mm) ===")
        print(f"  {'label':<32} {'links':>7} {'fingers':>7} {'carried':>7}")
        for label, r in sorted(rep.items()):
            d_link, who_link = r["min"]
            d_fing, who_fing = r["finger"]
            d_carr, _ = r["carried"]
            print(f"  {label:<32} {_fmt(d_link)} {_fmt(d_fing)} {_fmt(d_carr)}   [{who_link}; {who_fing}]")
            for d, who in ((d_link, who_link), (d_fing, who_fing), (d_carr, r["carried"][1])):
                if d < overall_min_scene:
                    overall_min_scene = d
                    worst_scene_str = f"{arm} at {label} ({who})"
            if d_fing < overall_min_finger:
                overall_min_finger = d_fing
                worst_finger_str = f"{arm} at {label} ({who_fing})"

    print(f"\nBimanual arm-arm min clearance: {arm_arm_min:.2f} mm at t={arm_arm_t:.2f}s ({arm_arm_pair[0]} vs {arm_arm_pair[1]})")
    print(f"Minimum finger-body clearance: {overall_min_finger:.2f} mm [{worst_finger_str}]")
    print(f"Minimum scene clearance: {overall_min_scene:.2f} mm [{worst_scene_str}]")

    all_ok = True
    if not np.isfinite(overall_min_scene):
        print("\nFAIL: no obstacle clearance was computed (min scene clearance is not a real number).")
        all_ok = False
    for arm, bad in (("Left", l_bad), ("Right", r_bad)):
        if bad:
            print(f"\n{arm} arm violations:")
            for b in bad:
                print("  FAIL:", b)
            all_ok = False

    if arm_arm_min < ARM_ARM_MIN_MM:
        print(f"\nArm-arm clearance FAIL: {arm_arm_min:.2f} mm < {ARM_ARM_MIN_MM} mm")
        all_ok = False

    if all_ok:
        print("\nALL CLEARANCES SATISFIED. 0 collisions detected.")
        return 0
    print("\nCLEARANCE VIOLATIONS DETECTED.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
