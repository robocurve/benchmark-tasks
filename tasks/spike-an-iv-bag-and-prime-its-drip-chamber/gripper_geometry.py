"""Collision-mesh hull checks for the complete YAM palm and moving fingers.

Bounding spheres are only a broad-phase rejection; convex planes test contact.
Both are derived from the actual URDF assets, including their local transforms.
"""
from functools import lru_cache
from pathlib import Path
import struct
import xml.etree.ElementTree as ET
import numpy as np


@lru_cache(maxsize=1)
def collision_hulls():
    from yam_arena import YAM_URDF
    from yam_arena.urdf import axis_angle_matrix
    from scipy.spatial import ConvexHull
    path = Path(YAM_URDF)
    result = {}
    for link in ET.parse(path).getroot().findall('link'):
        name = link.attrib['name']
        if name not in ('gripper', 'tip_left', 'tip_right'):
            continue
        result[name] = []
        for collision in link.findall('collision'):
            mesh = collision.find('geometry/mesh')
            if mesh is None:
                raise ValueError(f'Unsupported collision primitive on {name}')
            data = (path.parent / mesh.attrib['filename']).read_bytes()
            n = struct.unpack_from('<I', data, 80)[0]
            if len(data) != 84 + 50*n:
                raise ValueError('Expected binary STL collision mesh')
            dtype = np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attribute','<u2')])
            vertices = np.frombuffer(data, dtype=dtype, count=n, offset=84)['vertices'].reshape(-1,3).astype(float)
            vertices *= np.fromstring(mesh.attrib.get('scale','1 1 1'),sep=' ')
            origin = collision.find('origin')
            attrs = {} if origin is None else origin.attrib
            xyz = np.fromstring(attrs.get('xyz','0 0 0'),sep=' ')
            roll,pitch,yaw = np.fromstring(attrs.get('rpy','0 0 0'),sep=' ')
            rot = axis_angle_matrix(np.array([0,0,1]),yaw) @ axis_angle_matrix(np.array([0,1,0]),pitch) @ axis_angle_matrix(np.array([1,0,0]),roll)
            center = (vertices.min(0)+vertices.max(0))/2
            radius = float(np.linalg.norm(vertices-center,axis=1).max())
            vertices = vertices @ rot.T + xyz
            planes = ConvexHull(vertices).equations
            result[name].append((rot@center+xyz,radius,planes))
    return result


def collision_spheres():
    return {name: [(c,r) for c,r,_ in hulls] for name,hulls in collision_hulls().items()}


def segment_hits_planes(a, b, radius, planes):
    """Clip a capsule axis against convex planes expanded by its radius.

    The expansion is conservative at edges/corners; all actual mesh contacts
    are included, without the large empty regions of bounding spheres.
    """
    start = planes[:,:3] @ a + planes[:,3] - radius
    slope = planes[:,:3] @ (b-a)
    parallel = np.abs(slope) < 1e-12
    if np.any(start[parallel] > 0):
        return False
    enter, leave = 0.0, 1.0
    falling, rising = slope < -1e-12, slope > 1e-12
    if falling.any(): enter = max(enter,float(np.max(-start[falling]/slope[falling])))
    if rising.any(): leave = min(leave,float(np.min(-start[rising]/slope[rising])))
    return enter <= leave


def link_contact(name, pose, a, b, radius):
    from yam_arena.urdf import quat_xyzw_to_matrix
    rot = quat_xyzw_to_matrix(pose[3:7])
    a,b = rot.T@(a-pose[:3]), rot.T@(b-pose[:3])
    ab=b-a
    length2=float(ab@ab)
    for center,bound,planes in collision_hulls()[name]:
        u=0.0 if length2 < 1e-12 else np.clip((center-a)@ab/length2,0.0,1.0)
        if np.linalg.norm(center-a-u*ab) > bound+radius: continue
        if segment_hits_planes(a,b,radius,planes): return True
    return False
