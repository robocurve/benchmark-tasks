"""URDF geometric Jacobian for the right-arm pinch policy."""
from functools import lru_cache

import numpy as np
from yam_arena import YAM_URDF, ik
from yam_arena.constants import ARM_JOINTS, TCP_OFFSET
from yam_arena.urdf import parse_joints, rpy_matrix


@lru_cache(maxsize=1)
def chain():
    joints = parse_joints(YAM_URDF)
    result = []
    for name in ARM_JOINTS:
        j = joints[name]
        x,y,z = j.axis / np.linalg.norm(j.axis)
        skew = np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])
        result.append((j.xyz,rpy_matrix(j.rpy),j.axis,skew,skew@skew))
    return result


def geometry(q, base):
    position = np.asarray(base,dtype=float).copy()
    rotation = np.eye(3)
    centers, axes = [], []
    for angle,(offset,fixed,axis,skew,square) in zip(q,chain()):
        position = position + rotation@offset
        rotation = rotation@fixed
        centers.append(position)
        axes.append(rotation@axis)
        rotation = rotation@(np.eye(3)+np.sin(angle)*skew+(1-np.cos(angle))*square)
    tcp = position + rotation@TCP_OFFSET
    j = np.vstack((np.cross(axes,tcp-np.asarray(centers)).T,np.asarray(axes).T))
    return tcp,rotation,j


def solve(p, r, seed, base):
    q = seed.copy()
    for _ in range(30):
        tcp,orientation,j = geometry(q,base)
        error = np.r_[p-tcp,.3*ik._rot_error(r,orientation)]
        if np.linalg.norm(error)<2e-5:
            break
        j[3:]*=.3
        delta = j.T@np.linalg.solve(j@j.T+.0025*np.eye(6),error)
        q = np.clip(q+delta,ik.LOWER,ik.UPPER)
    return q
