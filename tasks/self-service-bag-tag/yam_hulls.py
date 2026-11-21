"""YAM convex collision pieces (the decomposition PhysX uses) posed by FK. CPU only."""
from __future__ import annotations

import xml.etree.ElementTree as ET
from functools import lru_cache
from pathlib import Path

import numpy as np
import trimesh

from yam_arena import YAM_URDF, station
from yam_arena.constants import ARM_JOINTS
from yam_arena.urdf import forward_kinematics, rpy_matrix

ROOT = Path(YAM_URDF).parent
LINKS = ("link3", "link4", "link5", "gripper", "tip_left", "tip_right")


@lru_cache(None)
def pieces():
    """{link: [vertices (n,3) in the link frame], ...} from collision/<link>/*.stl."""
    origins = {}
    for link in ET.parse(YAM_URDF).getroot().findall("link"):
        o = link.find("collision/origin")
        if o is not None:
            origins[link.get("name")] = (np.fromstring(o.get("rpy", "0 0 0"), sep=" "),
                                         np.fromstring(o.get("xyz", "0 0 0"), sep=" "))
    out = {}
    for name in LINKS:
        rpy, xyz = origins.get(name, (np.zeros(3), np.zeros(3)))
        out[name] = [np.asarray(trimesh.load(p, force="mesh").vertices) @ rpy_matrix(rpy).T + xyz
                     for p in sorted((ROOT / "collision" / name).glob("*.stl"))]
    return out


def posed(joints, side):
    """World-frame hull vertex arrays per link for an 8-vector (6 arm + 2 finger) joint state."""
    vals = {n: float(v) for n, v in zip(ARM_JOINTS, joints[:6])}
    vals.update(joint7=float(joints[6]), joint8=float(joints[7]))
    poses = forward_kinematics(YAM_URDF, vals)
    base = np.asarray(station.BASES[side])
    return {n: [v @ poses[n][:3, :3].T + poses[n][:3, 3] + base for v in vs] for n, vs in pieces().items()}


def hulls(joints, side, links=LINKS):
    return {n: [trimesh.convex.convex_hull(v) for v in vs] for n, vs in posed(joints, side).items()
            if n in links}


def box_points(centre, R, size, n=(20, 20, 3)):
    """Grid of points filling a box (for overlap counts against hulls)."""
    axes = [np.linspace(-s / 2, s / 2, k) for s, k in zip(size, n)]
    g = np.stack(np.meshgrid(*axes, indexing="ij"), -1).reshape(-1, 3)
    return g @ np.asarray(R).T + np.asarray(centre)


def inside_any(points, hull_list):
    mask = np.zeros(len(points), bool)
    for h in hull_list:
        mask |= h.contains(points)
    return mask
