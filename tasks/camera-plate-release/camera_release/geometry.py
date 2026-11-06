"""Geometry with explicit XYZW quaternions, independent of simulator state."""

from itertools import product

import numpy as np

from .config import CAM_BOXES, CAM_PIVOT, MOUNT_POSITION, PAYLOAD_BOXES, STAND_BOXES

SIGNS = np.array(list(product((-1.0, 1.0), repeat=3)))


def rotation_xyzw(quaternion):
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or abs(np.dot(q, q) - 1) > 0.001:
        raise ValueError("Expected a finite unit XYZW quaternion")
    x, y, z, w = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def angle_between(a, b):
    return float(2 * np.arccos(np.clip(abs(np.dot(a, b)), 0, 1)))


def corners(centre, half_size, position, rotation):
    return (np.asarray(centre) + SIGNS * np.asarray(half_size)) @ rotation.T + position


def payload_corners(position, quaternion):
    rot = rotation_xyzw(quaternion)
    return np.concatenate([corners(c, h, position, rot) for c, h in PAYLOAD_BOXES])


def bounds(points):
    return np.min(points, axis=0), np.max(points, axis=0)


def aabb_gap(a, b):
    """Conservative lower bound on surface distance between enclosing boxes."""
    alo, ahi = a
    blo, bhi = b
    delta = np.maximum(np.maximum(alo-bhi, blo-ahi), 0)
    return float(np.linalg.norm(delta))


def mount_bounds(cam_angle):
    identity = np.eye(3)
    result = [bounds(corners(c, h, MOUNT_POSITION, identity)) for c, h in STAND_BOXES]
    c, s = np.cos(cam_angle), np.sin(cam_angle)
    rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    result += [bounds(corners(c, h, MOUNT_POSITION+CAM_PIVOT, rotation)) for c, h in CAM_BOXES]
    return result


def payload_mount_gap(position, quaternion, cam_angle):
    rot = rotation_xyzw(quaternion)
    payload = [bounds(corners(c, h, position, rot)) for c, h in PAYLOAD_BOXES]
    return min(aabb_gap(a, b) for a in payload for b in mount_bounds(cam_angle))
