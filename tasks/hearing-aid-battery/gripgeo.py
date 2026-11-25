"""The YAM gripper's collision pieces as a few boxes per link, for the dressing model's contacts and pinches.

Each link's URDF collision meshes are grouped by height along the finger (bins of 15 mm) and each group is bounded by
one box in the link frame. The lowest box of each finger is its fingertip pad.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from functools import lru_cache
from pathlib import Path

import numpy as np

LINKS = ("gripper", "tip_left", "tip_right")
BIN = 0.015


@lru_cache(maxsize=1)
def link_boxes() -> dict:
    """{link: [(lo, hi), ...]} in each link's own frame, the fingertip pad first for the two fingers."""
    import trimesh
    from yam_arena import YAM_URDF
    from yam_arena.urdf import rpy_matrix

    urdf = Path(YAM_URDF)
    out = {}
    for link in ET.parse(urdf).getroot().findall("link"):
        name = link.get("name")
        if name not in LINKS:
            continue
        pieces = []
        for c in link.findall("collision"):
            o = c.find("origin")
            t = np.fromstring(o.get("xyz", "0 0 0"), sep=" ") if o is not None else np.zeros(3)
            r = rpy_matrix(np.fromstring(o.get("rpy", "0 0 0"), sep=" ")) if o is not None else np.eye(3)
            m = c.find("geometry").find("mesh")
            v = trimesh.load((urdf.parent / m.get("filename")).resolve(), force="mesh").vertices @ r.T + t
            pieces.append(v)
        zc = np.array([p[:, 2].mean() for p in pieces])
        order = np.argsort(zc)  # most distal (lowest z in the link frame) first
        groups, cur, z0 = [], [], None
        for k in order:
            if z0 is None or zc[k] - z0 > BIN:
                if cur:
                    groups.append(cur)
                cur, z0 = [], zc[k]
            cur.append(k)
        groups.append(cur)
        boxes = []
        for g in groups:
            v = np.concatenate([pieces[k] for k in g])
            boxes.append((v.min(axis=0), v.max(axis=0)))
        out[name] = boxes
    return out


if __name__ == "__main__":
    from yam_arena import YAM_URDF
    from yam_arena.constants import TCP_OFFSET
    from yam_arena.urdf import forward_kinematics

    lb = link_boxes()
    for q in (0.0, -0.01, -0.04695):
        fk = forward_kinematics(YAM_URDF, {"joint7": q, "joint8": q})
        g = fk["gripper"]
        print("finger q", q)
        for name, boxes in lb.items():
            t = np.linalg.inv(g) @ fk[name]
            for lo, hi in boxes[:3 if name != "gripper" else 6]:
                c = np.array([[lo[0], lo[1], lo[2]], [hi[0], hi[1], hi[2]]])
                w = c @ t[:3, :3].T + t[:3, 3] - np.array(TCP_OFFSET)
                print(f"  {name:9s} tcp-frame lo {np.round(w.min(0)*1000,1)} hi {np.round(w.max(0)*1000,1)}")


@lru_cache(maxsize=64)
def _tcp_boxes(q_mm: int):
    from yam_arena import YAM_URDF
    from yam_arena.constants import TCP_OFFSET
    from yam_arena.urdf import forward_kinematics

    q = -q_mm / 1000.0
    fk = forward_kinematics(YAM_URDF, {"joint7": q, "joint8": q})
    g = fk["gripper"]
    out = []
    for name, boxes in link_boxes().items():
        t = np.linalg.inv(g) @ fk[name]
        for k, (lo, hi) in enumerate(boxes):
            c = np.array([[a, b, cc] for a in (lo[0], hi[0]) for b in (lo[1], hi[1]) for cc in (lo[2], hi[2])])
            w = c @ t[:3, :3].T + t[:3, 3] - np.array(TCP_OFFSET)
            out.append(((name, k), w.min(0), w.max(0)))
    return tuple(out)


def tcp_boxes(aperture: float):
    """The gripper's boxes in the TCP frame (x: closing axis, z: along the fingers) for a jaw opening (m) between the
    fingertip pads. Each finger's pad box is the one tagged (tip_left|tip_right, 0)."""
    return _tcp_boxes(int(round(min(max(aperture, 0.0), 0.0939) / 2 * 1000)))
