"""fingers: where the grippers' fingers are, as points on their collision meshes, for a scorer that must know when a
finger comes near something.

The task file's scorer only sees the objects. :func:`finger_points` reads both arms' ``tip_left`` and ``tip_right``
bodies from the running simulation and returns points on the fingers' collision meshes in the task file's frame (the
environment's origin removed), so a scorer can ask how close any part of a finger came to a point. The meshes are the
ones yam_arena's URDF names for those links, thinned to one point per 2 mm cube.

Pure Python at import time: the meshes are read on first use, and the simulation only when :func:`finger_points` is
called (from an ``on_step`` hook).
"""

from __future__ import annotations

import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ARMS = ("left_robot", "right_robot")
TIPS = ("tip_left", "tip_right")
VOXEL = 0.002  # m: one point kept per cube of this size
_CLOUDS: dict[str, np.ndarray] = {}


def _read_stl(path: Path) -> np.ndarray:
    data = path.read_bytes()
    if data[:5] == b"solid" and b"facet" in data[:300]:
        rows = [line.split()[1:] for line in data.decode().splitlines() if line.strip().startswith("vertex")]
        return np.asarray(rows, float)
    n = struct.unpack("<I", data[80:84])[0]
    tri = np.frombuffer(data[84:84 + 50 * n], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return tri["v"].reshape(-1, 3).astype(float)


def _rpy(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def cloud(link: str, urdf: Path | None = None) -> np.ndarray:
    """Points on ``link``'s collision meshes, in the link's frame, one per 2 mm cube."""
    if link in _CLOUDS:
        return _CLOUDS[link]
    if urdf is None:
        from yam_arena import YAM_URDF as urdf
    urdf = Path(urdf)
    node = ET.parse(urdf).getroot().find(f"link[@name='{link}']")
    if node is None:
        raise KeyError(f"no link {link} in {urdf}")
    parts = []
    for col in node.findall("collision"):
        origin = col.find("origin")
        xyz = np.array([float(v) for v in origin.get("xyz", "0 0 0").split()]) if origin is not None else np.zeros(3)
        rpy = [float(v) for v in origin.get("rpy", "0 0 0").split()] if origin is not None else [0.0] * 3
        for mesh in col.iter("mesh"):
            scale = np.array([float(v) for v in mesh.get("scale", "1 1 1").split()])
            v = _read_stl(urdf.parent / mesh.get("filename")) * scale
            parts.append(v @ _rpy(*rpy).T + xyz)
    if not parts:
        raise ValueError(f"{link} has no collision meshes in {urdf}")
    v = np.vstack(parts)
    _, keep = np.unique(np.floor(v / VOXEL).astype(np.int64), axis=0, return_index=True)
    _CLOUDS[link] = v[np.sort(keep)]
    return _CLOUDS[link]


def _rotations(q_xyzw: np.ndarray) -> np.ndarray:
    """Rotation matrices of an (n, 4) array of x, y, z, w quaternions."""
    q = q_xyzw / np.linalg.norm(q_xyzw, axis=1, keepdims=True)
    x, y, z, w = q.T
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], -1),
        np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], -1),
        np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1),
    ], -2)


def _numpy(t) -> np.ndarray:
    t = getattr(t, "torch", t)
    return t.detach().cpu().numpy().astype(np.float64)


def tip_poses(env) -> dict[tuple[str, str], tuple[np.ndarray, np.ndarray]]:
    """Each finger's (position, x y z w quaternion) in the task file's frame, keyed (arm, link)."""
    base = env.unwrapped
    origin = _numpy(base.scene.env_origins[0])
    out = {}
    for arm in ARMS:
        robot = base.scene[arm]
        names = list(robot.body_names if hasattr(robot, "body_names") else robot.data.body_names)
        pos, quat = _numpy(robot.data.body_pos_w)[0], _numpy(robot.data.body_quat_w)[0]
        for tip in TIPS:
            i = names.index(tip)
            out[(arm, tip)] = (pos[i] - origin, quat[i])
    return out


def finger_points(env) -> np.ndarray:
    """Points on all four fingers, (n, 3), in the task file's frame."""
    clouds = []
    for (_, tip), (pos, quat) in tip_poses(env).items():
        rot = _rotations(np.asarray([quat]))[0]
        clouds.append(cloud(tip) @ rot.T + pos)
    return np.vstack(clouds)
