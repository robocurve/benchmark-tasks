"""Conservative collision-mesh envelopes, in metres; no simulator imports.

Each URDF collision mesh gets its own enclosing box. Transform all eight corners
with the measured link pose; never substitute the TCP for a gripper surface.
"""
from functools import lru_cache
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from yam_arena import YAM_URDF
from yam_arena.analysis import read_stl
from yam_arena.urdf import rpy_matrix, quat_xyzw_to_matrix

GRIPPER_LINKS = {'gripper', 'tip_left', 'tip_right'}

@lru_cache(maxsize=1)
def collision_boxes():
    names, centres, halves = [], [], []
    for link in ET.parse(YAM_URDF).getroot().findall('link'):
        for collision in link.findall('collision'):
            mesh = collision.find('geometry/mesh')
            if mesh is None:
                raise ValueError('Expected mesh collider; update the envelope loader for this robot')
            origin = collision.find('origin')
            xyz = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ') if origin is not None else np.zeros(3)
            rpy = np.fromstring(origin.get('rpy', '0 0 0'), sep=' ') if origin is not None else np.zeros(3)
            scale = np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
            points = read_stl(Path(YAM_URDF).parent / mesh.get('filename')).reshape(-1, 3) * scale
            points = points @ rpy_matrix(rpy).T + xyz
            low, high = points.min(axis=0), points.max(axis=0)
            names.append(link.get('name'))
            centres.append((low + high) / 2)
            halves.append((high - low) / 2)
    return names, np.asarray(centres), np.asarray(halves)

def world_boxes(body_names, poses):
    names, centres, halves = collision_boxes()
    ids = np.asarray([body_names.index(n) for n in names])
    rotations = np.asarray([quat_xyzw_to_matrix(p[3:7]) for p in poses])
    r = rotations[ids]
    c = np.einsum('nij,nj->ni', r, centres) + poses[ids, :3]
    h = np.einsum('nij,nj->ni', np.abs(r), halves)
    return c - h, c + h, np.asarray([n in GRIPPER_LINKS for n in names])

def gap(a, b):
    """Minimum surface distance between two collections of axis-aligned boxes."""
    al, ah = a
    bl, bh = b
    if len(al) == 0 or len(bl) == 0:
        return float('inf')
    d = np.maximum(0, np.maximum(bl[None] - ah[:, None], al[:, None] - bh[None]))
    return float(np.sqrt(np.sum(d * d, axis=-1)).min())

def intersects(boxes, low, high):
    lo, hi = boxes
    return bool(np.any(np.all((hi >= low) & (lo <= high), axis=1)))
