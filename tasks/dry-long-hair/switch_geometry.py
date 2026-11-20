"""Actual unmodified collision-mesh support under the physical slider.

Clip triangles, rather than selecting vertices: an edge/face can cross the small
slider footprint with no original vertex inside it. Coordinates are metres.
"""
import struct
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from yam_arena import YAM_URDF
from yam_arena.constants import TCP_OFFSET
from yam_arena.urdf import forward_kinematics, rpy_matrix

SLIDER_HALF = np.array((.003, .0065, .007))


def clipped_support(triangles, xy_min, xy_max):
    """Return lower/upper Z of triangle portions inside an XY rectangle."""
    lo, hi = np.asarray(xy_min), np.asarray(xy_max)
    triangles = np.asarray(triangles)
    candidates = triangles[np.all(triangles[:, :, :2].max(1) >= lo, axis=1)
                           & np.all(triangles[:, :, :2].min(1) <= hi, axis=1)]
    zs = []
    for triangle in candidates:
        polygon = list(triangle)
        for axis, bound, sign in ((0,lo[0],1),(0,hi[0],-1),(1,lo[1],1),(1,hi[1],-1)):
            clipped = []
            if not polygon:
                break
            previous = polygon[-1]
            for current in polygon:
                a, b = sign*(previous[axis]-bound), sign*(current[axis]-bound)
                if (a >= 0) != (b >= 0):
                    clipped.append(previous + (current-previous)*a/(a-b))
                if b >= 0:
                    clipped.append(current)
                previous = current
            polygon = clipped
        zs.extend(p[2] for p in polygon)
    return None if not zs else (float(min(zs)), float(max(zs)))


def separating_gap(first, second):
    """Conservative separating-plane gap for collision triangle sets.

    A positive result certifies disjoint geometry, including crossed-face
    cases that vertex inclusion misses. Zero means no separating plane found
    on face normals; it does not certify penetration of nonconvex geometry.
    """
    av,bv = first.reshape(-1,3),second.reshape(-1,3)
    gap = float(np.maximum(av.min(0)-bv.max(0),bv.min(0)-av.max(0)).max())
    if gap > .003:
        return gap
    triangles = np.concatenate((first,second))
    axes = np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])
    lengths = np.linalg.norm(axes,axis=1)
    axes = np.unique(np.round(axes[lengths>1e-12]/lengths[lengths>1e-12,None],8),axis=0)
    # Any of these planes is a sufficient absence certificate, without making
    # an assumption that the dense collision pieces are perfectly convex.
    for start in range(0,len(axes),128):
        a,b = av@axes[start:start+128].T,bv@axes[start:start+128].T
        gap = max(gap,float(np.maximum(a.min(0)-b.max(0),b.min(0)-a.max(0)).max()))
    return max(0.,gap)


def minimum_separation(first_pieces, second_pieces):
    """Smallest certified gap between all ordinary collision-mesh pieces."""
    first_min = np.array([p.min((0,1)) for p in first_pieces])
    first_max = np.array([p.max((0,1)) for p in first_pieces])
    second_min = np.array([p.min((0,1)) for p in second_pieces])
    second_max = np.array([p.max((0,1)) for p in second_pieces])
    gaps = np.maximum(first_min[:,None]-second_max[None,:],second_min[None,:]-first_max[:,None]).max(2)
    for i,j in np.argwhere(gaps <= .003):
        gaps[i,j] = separating_gap(first_pieces[i],second_pieces[j])
    return float(gaps.min())


class FingerGeometry:
    """STL collision surfaces of both fingers and gripper in the TCP frame."""
    def __init__(self, urdf=YAM_URDF):
        self.urdf = Path(urdf)
        self.meshes = []
        for link in ET.parse(urdf).getroot().findall('link'):
            if link.get('name') not in ('gripper','tip_left','tip_right'):
                continue
            for collision in link.findall('collision'):
                mesh = collision.find('geometry/mesh')
                if mesh is None:
                    raise ValueError('Unsupported non-mesh gripper collider')
                name = mesh.get('filename')
                filename = self.urdf.parent / name
                data = filename.read_bytes()
                count = struct.unpack('<I', data[80:84])[0]
                dtype = np.dtype([('normal','<f4',(3,)),('v','<f4',(3,3)),('attribute','<u2')])
                triangles = np.ndarray(count,dtype=dtype,buffer=data,offset=84)['v'].astype(float)
                origin = collision.find('origin')
                xyz = np.zeros(3) if origin is None else np.array([float(v) for v in origin.get('xyz','0 0 0').split()])
                rpy = np.zeros(3) if origin is None else np.array([float(v) for v in origin.get('rpy','0 0 0').split()])
                scale = np.array([float(v) for v in mesh.get('scale','1 1 1').split()])
                self.meshes.append((link.get('name'), (triangles*scale)@rpy_matrix(rpy).T+xyz))

    def pieces(self, fingers):
        fk = forward_kinematics(self.urdf, {'joint7':float(fingers[0]),'joint8':float(fingers[1])})
        inverse = np.linalg.inv(fk['gripper'])
        result = []
        for name, triangles in self.meshes:
            transform = inverse@fk[name]
            result.append((name,triangles@transform[:3,:3].T + transform[:3,3] - np.array(TCP_OFFSET)))
        return result

    def triangles(self, fingers):
        return np.concatenate([triangles for _,triangles in self.pieces(fingers)])

    def support(self, fingers, tcp_position, tcp_rotation, slider_center):
        triangles = self.triangles(fingers)@tcp_rotation.T+tcp_position
        return clipped_support(triangles, slider_center[:2]-SLIDER_HALF[:2], slider_center[:2]+SLIDER_HALF[:2])
