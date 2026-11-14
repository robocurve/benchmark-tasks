"""Read-only collision-envelope measurements from authored native geometry."""

from itertools import product
import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix


def local_solids(records):
    groups = {}
    for item in records:
        if "solid" not in item or item["asset"] == "fixtures":
            continue
        key = item["asset"] + ":" + item["solid"].split("/")[-2]
        if item["kind"] == "box":
            p = (
                np.array(list(product((-1.0, 1.0), repeat=3)))
                * np.array(item["size_m"])
                / 2
            )
        elif item["kind"] == "cone":
            angles = np.linspace(0, 2 * np.pi, 64, endpoint=False)
            p = np.array(
                [
                    (
                        item["radius_m"] * np.cos(a),
                        item["radius_m"] * np.sin(a),
                        -item["height_m"] / 2,
                    )
                    for a in angles
                ]
                + [(0, 0, item["height_m"] / 2)]
            )
        else:
            angles = np.linspace(0, 2 * np.pi, 64, endpoint=False)
            p = np.array(
                [
                    (item["radius_m"] * np.cos(a), item["radius_m"] * np.sin(a), z)
                    for a in angles
                    for z in (-item["height_m"] / 2, item["height_m"] / 2)
                ]
            )
        a = item.get("yaw_rad", 0)
        c, s = np.cos(a), np.sin(a)
        r = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        groups.setdefault(key, []).append(p @ r.T + item["center_m"])
    return {k: np.concatenate(v) for k, v in groups.items()}


def world_points(local, poses):
    return {
        k: p @ quat_xyzw_to_matrix(np.array(poses[k][3:])).T + poses[k][:3]
        for k, p in local.items()
    }


def robot_clearance(points, hands):
    from yam_arena.analysis import link_hulls

    objects = [(p.min(0), p.max(0)) for p in points.values()]
    robot = []
    for hand in hands.values():
        for name, pose in zip(hand["names"], hand["poses"]):
            for hull in link_hulls().get(name, []):
                p = hull.vertices @ quat_xyzw_to_matrix(np.array(pose[3:])).T + pose[:3]
                robot.append((p.min(0), p.max(0)))
    if not robot:
        raise RuntimeError("Whole-robot collision geometry is required")
    return min(
        float(np.linalg.norm(np.maximum(np.maximum(a[0] - b[1], b[0] - a[1]), 0)))
        for a in objects
        for b in robot
    )
