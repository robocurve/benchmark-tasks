"""Both grippers as point clouds in the world, for the release check and the nail-region rule.

Each hand is the YAM's wrist (`link5`), `gripper`, `tip_left` and `tip_right` collision hulls. Every hull vertex is kept, and every hull
face is filled with points about 3 mm apart, so a broad flat face lying on a board is measured where it is, not only
at its corners."""

import math
from functools import lru_cache

import numpy as np

LINKS = ("link5", "gripper", "tip_left", "tip_right")
ARMS = ("left_robot", "right_robot")
SPACING = 0.003


def _face_points(vertices, planes):
    out = [vertices]
    for plane in np.unique(np.round(planes, 7), axis=0):
        face = vertices[np.abs(vertices @ plane[:3] - plane[3]) < 2e-6]
        if len(face) < 3:
            continue
        c = face.mean(axis=0)
        u = face[0] - c
        u /= np.linalg.norm(u)
        v = np.cross(plane[:3], u)
        face = face[np.argsort(np.arctan2((face - c) @ v, (face - c) @ u))]
        for a, b in zip(face, np.roll(face, -1, axis=0)):  # fan triangles (c, a, b), filled on a grid
            n = max(1, math.ceil(max(np.linalg.norm(a - c), np.linalg.norm(b - c), np.linalg.norm(a - b)) / SPACING))
            i, j = np.meshgrid(np.arange(n + 1), np.arange(n + 1), indexing="ij")
            keep = (i + j) <= n
            s, t = i[keep] / n, j[keep] / n
            out.append(c + np.outer(s, a - c) + np.outer(t, b - c))
    return np.concatenate(out)


@lru_cache(maxsize=1)
def hand_mesh():
    """{link: (N, 3) points in the link frame}."""
    from yam_arena import analysis

    hulls = analysis.link_hulls()
    out = {}
    for name in LINKS:
        if not hulls.get(name):
            raise ValueError(f"no collision hulls for {name}")
        pts = np.concatenate([_face_points(h.vertices, h.planes) for h in hulls[name]])
        _, keep = np.unique(np.floor(pts / 0.0015).astype(int), axis=0, return_index=True)
        out[name] = pts[keep]
    return out


def clouds(sim):
    """[(N, 3) world points of the left hand, of the right hand], in the env frame the object states use.
    Raises ValueError when any pose is missing or not finite (the scorer then gives no credit)."""
    import parts

    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    if origin.shape != (3,) or not np.isfinite(origin).all():
        raise ValueError("unknown env origin")
    mesh = hand_mesh()
    out = []
    for arm in ARMS:
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        if poses.ndim != 2 or poses.shape[1] != 7 or not np.isfinite(poses).all():
            raise ValueError(f"unknown {arm} link poses")
        pts = []
        for name in LINKS:
            pose = poses[asset.body_names.index(name)]
            if np.linalg.norm(pose[3:]) < 1e-6:
                raise ValueError(f"unknown {arm} {name} orientation")
            pts.append(mesh[name] @ parts._quat_to_matrix(pose[3:]).T + pose[:3] - origin)  # xyzw, as parts.py reads it
        cloud = np.concatenate(pts)
        if not np.isfinite(cloud).all():
            raise ValueError(f"unknown {arm} hand surface")
        out.append(cloud)
    return out
