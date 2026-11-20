"""Measured unmodified gripper collision meshes and separating-plane certificates."""
import struct
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from yam_arena import YAM_URDF
from yam_arena.constants import TCP_OFFSET
from yam_arena.urdf import forward_kinematics, rpy_matrix


def _prepare_piece(triangles):
    vertices = np.unique(triangles.reshape(-1,3),axis=0)
    axes = np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])
    lengths = np.linalg.norm(axes,axis=1)
    axes = np.unique(np.round(axes[lengths>1e-12]/lengths[lengths>1e-12,None],8),axis=0)
    return vertices,axes


def _prepared_gap(first, second, gap):
    av,aa = first
    bv,ba = second
    axes = np.unique(np.concatenate((aa,ba)),axis=0)
    for start in range(0,len(axes),128):
        a,b = av@axes[start:start+128].T,bv@axes[start:start+128].T
        gap = max(gap,float(np.maximum(a.min(0)-b.max(0),b.min(0)-a.max(0)).max()))
    return max(0.,gap)


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
    # Any of these planes is a sufficient absence certificate, without making
    # an assumption that the dense collision pieces are perfectly convex.
    return _prepared_gap(_prepare_piece(first),_prepare_piece(second),gap)


def minimum_separation(first_pieces, second_pieces):
    """Smallest certified gap between all ordinary collision-mesh pieces."""
    first_min = np.array([p.min((0,1)) for p in first_pieces])
    first_max = np.array([p.max((0,1)) for p in first_pieces])
    second_min = np.array([p.min((0,1)) for p in second_pieces])
    second_max = np.array([p.max((0,1)) for p in second_pieces])
    gaps = np.maximum(first_min[:,None]-second_max[None,:],second_min[None,:]-first_max[:,None]).max(2)
    # Lifetime is this call only: measured meshes may change every frame.
    first_prepared,second_prepared = {},{}
    for i,j in np.argwhere(gaps <= .003):
        if i not in first_prepared:first_prepared[i] = _prepare_piece(first_pieces[i])
        if j not in second_prepared:second_prepared[j] = _prepare_piece(second_pieces[j])
        gaps[i,j] = _prepared_gap(first_prepared[i],second_prepared[j],gaps[i,j])
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
