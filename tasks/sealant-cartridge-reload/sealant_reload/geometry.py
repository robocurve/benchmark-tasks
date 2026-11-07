"""Read-only geometry measurements from the same solids that author the USD."""
from itertools import product
import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

SIGNS = np.array(list(product((-1., 1.), repeat=3)))


def local_solids(inventory):
    groups = {}
    for item in inventory:
        if 'solid' not in item or item['asset'] == 'fixtures':
            continue
        key = item['asset']+':'+item['solid'].split('/')[-2]
        kind = item['kind']
        if kind == 'box':
            p = SIGNS*np.array(item['half_extents_m'])
            a = item['rotation_about_y_rad']
            c, s = np.cos(a), np.sin(a)
            r = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
            p = p@r.T + item['center_m']
        elif kind == 'cylinder':
            a = np.linspace(0, 2*np.pi, 64, endpoint=False)
            p = np.array([(item['radius_m']*np.cos(t), y, item['radius_m']*np.sin(t))
                          for t in a for y in (-item['length_m']/2, item['length_m']/2)])
            p += item['center_m']
        else:
            p = np.array(item['vertices_m'])
        groups.setdefault(key, []).append(p)
    return {k: np.concatenate(v) for k, v in groups.items()}


def world_points(local, poses):
    return {k: p@quat_xyzw_to_matrix(np.array(poses[k][3:])).T+poses[k][:3] for k, p in local.items()}


def bounds(points):
    return np.min(points, axis=0), np.max(points, axis=0)


def gap(a, b):
    return float(np.linalg.norm(np.maximum(np.maximum(a[0]-b[1], b[0]-a[1]), 0)))


def robot_clearance(points, hands):
    from yam_arena.analysis import link_hulls
    apparatus = [bounds(v) for v in points.values()]
    robot = []
    for hand in hands.values():
        for name, pose in zip(hand['names'], hand['poses']):
            r = quat_xyzw_to_matrix(np.array(pose[3:]))
            for h in link_hulls().get(name, []):
                robot.append(bounds(h.vertices@r.T+pose[:3]))
    if not robot:
        raise RuntimeError('Robot collision geometry is required for release measurement')
    return min(gap(a, b) for a in apparatus for b in robot)
