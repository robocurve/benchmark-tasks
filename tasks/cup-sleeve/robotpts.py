"""Surface sample points of each arm's wrist, gripper and fingers, from the robot's own meshes, for the scorer's
"every part of the robot at least 3 cm from the shoe" check. NumPy only, except :func:`robot_points`."""

from __future__ import annotations

import math
import re
from functools import lru_cache
from pathlib import Path

import numpy as np

LINKS = ("link4", "link5", "gripper", "tip_left", "tip_right")
VOXEL = 0.004


def _urdf() -> Path:
    import yam_arena
    return Path(yam_arena.__file__).resolve().parent / "assets" / "yam" / "yam.urdf"


def _rpy(r, p, y) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def _stl_vertices(path: Path) -> np.ndarray:
    data = path.read_bytes()
    if data[:5] == b"solid" and b"facet" in data[:400]:
        v = re.findall(rb"vertex\s+(\S+)\s+(\S+)\s+(\S+)", data)
        return np.array(v, dtype=float)
    n = int.from_bytes(data[80:84], "little")
    rec = np.frombuffer(data[84:84 + 50 * n], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    return rec["v"].reshape(-1, 3).astype(float)


@lru_cache(maxsize=1)
def local_points() -> dict[str, np.ndarray]:
    """Per link: mesh vertices in the link frame, thinned to one per 4 mm voxel."""
    urdf = _urdf()
    text = urdf.read_text()
    out = {}
    for name in LINKS:
        blk = re.search(r'<link name="%s".*?</link>' % name, text, re.S).group(0)
        vis = re.search(r"<visual>.*?</visual>", blk, re.S).group(0)
        xyz = [float(v) for v in re.search(r'xyz="([^"]*)"', vis).group(1).split()]
        rpy = [float(v) for v in re.search(r'rpy="([^"]*)"', vis).group(1).split()]
        fn = re.search(r'filename="([^"]*)"', vis).group(1)
        v = _stl_vertices(urdf.parent / fn)
        v = v @ _rpy(*rpy).T + np.array(xyz)
        keys = np.floor(v / VOXEL).astype(np.int64)
        _, first = np.unique(keys, axis=0, return_index=True)
        out[name] = v[np.sort(first)]
    return out


def quat_to_mat(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def robot_points(env) -> np.ndarray:
    """Both arms' sample points in the station frame (env 0)."""
    base = env.unwrapped if hasattr(env, "unwrapped") else env
    t = lambda x: getattr(x, "torch", x)  # noqa: E731
    origin = t(base.scene.env_origins)[0].detach().cpu().numpy()
    pts = []
    loc = local_points()
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        names = list(robot.body_names)
        pos = t(robot.data.body_link_pos_w)[0].detach().cpu().numpy() - origin
        quat = t(robot.data.body_link_quat_w)[0].detach().cpu().numpy()
        for link in LINKS:
            if link in names:
                i = names.index(link)
                pts.append(loc[link] @ quat_to_mat(quat[i]).T + pos[i])
    return np.vstack(pts) if pts else np.empty((0, 3))
