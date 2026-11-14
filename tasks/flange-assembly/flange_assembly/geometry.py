"""Geometry checks for horizontal flange assembly, in metres.

Object positions are bounding-box centres; axes derive from XYZW quaternions.
The side-lug offset corrects the loose flange's bounding-box centre.
"""
import numpy as np

HALF_DEPTH = .0255
HOLE_RADIUS = .05499
AXIS_COS = np.cos(np.deg2rad(7.25))


def rotation(q):
    x, y, z, w = np.asarray(q, dtype=float)
    norm = np.linalg.norm([x, y, z, w])
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError('invalid quaternion')
    x, y, z, w = np.array([x, y, z, w]) / norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def holes(obj, half_depth=HALF_DEPTH, local_y_offset=0.):
    r = rotation(obj.quat_xyzw)
    face = np.asarray(obj.pos) - half_depth*r[:, 2] - local_y_offset*r[:, 1]
    return [face + r @ np.array([x, y, 0.]) for x, y in
            ((HOLE_RADIUS, 0), (0, HOLE_RADIUS), (-HOLE_RADIUS, 0), (0, -HOLE_RADIUS))]


def score(fixed, loose, pins):
    """Final state only; .4 alignment plus .3 per distinct seated pin.

    Fixed local +z points along +x, loose local +z along -x, pin +z
    towards -x (head side). Holes are matched in three dimensions. Pin
    bbox-centre must be between 5 mm towards the head and 8 mm towards the tip from the mating plane.
    """
    f, l = rotation(fixed.quat_xyzw), rotation(loose.quat_xyzw)
    if f[0, 2] < AXIS_COS or -l[0, 2] < AXIS_COS:
        return 0.0
    fh, lh = holes(fixed), holes(loose, .065, .02)
    # Hole centres account for both axial mating and bolt-circle alignment.
    if any(min(np.linalg.norm(a-b) for b in lh) >= .003 for a in fh):
        return 0.0
    claimed = set()
    for pin in pins:
        axis = rotation(pin.quat_xyzw)[:, 2]
        if -axis[0] < AXIS_COS:
            continue
        delta = [np.asarray(pin.pos) - h for h in fh]
        j = int(np.argmin([np.linalg.norm(d[1:]) for d in delta]))
        d = delta[j]
        if j in (0, 2) and j not in claimed and np.linalg.norm(d[1:]) < .0045 and -.005 <= d[0] <= .008:
            claimed.add(j)
    return min(1.0, .4 + .3*len(claimed))
