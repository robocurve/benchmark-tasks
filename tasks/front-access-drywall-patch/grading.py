"""Geometric thread engagement and end-state fastener coverage, in metres."""
import numpy as np
from scipy.spatial.transform import Rotation


def bite(pose, point, axis, thickness=.016, length=.251, width=.025, board=.0127):
    """Either broad face of an unmarked rectangular strip can receive the screw."""
    rotation = Rotation.from_quat(pose[3:]).inv()
    origin = rotation.apply(point-pose[:3])
    direction = rotation.apply(axis)
    if abs(direction[0]) < .85:
        return False
    entry = (-np.copysign(thickness/2, direction[0])-origin[0])/direction[0]
    samples = origin + np.linspace(.015, .030, 10)[:, None]*direction
    inside = np.all(np.abs(samples) < [thickness/2, length/2-.003, width/2-.002], axis=1)
    return -.0015 <= entry-board <= .0025 and inside.sum() >= 6


def backing_rows(shots, mounts, middle):
    """Each row spans both jambs in one half. An intact strip counts only once."""
    rows = {}
    for name in ('strip', 'spare_strip'):
        for side in (-1, 1):
            sites = [s for s,m in zip(shots, mounts, strict=True) if s['wood']==name
                     and not m['broken'] and m['gap'] < .002 and side*(s['head'][2]-middle) >= .025]
            walls = {s['front'] for s in sites}
            if {'jamb_minus', 'jamb_plus'} <= walls:
                wings = {sign for sign in (-1,1) if any(s['front']=='patch'
                         and sign*s['head'][1] >= .025 for s in sites)}
                rows.setdefault(side, {})[name] = (any(s['front']=='patch' for s in sites), wings)
    candidates = [(0,0,False)]
    for side in (-1,1):
        for name, (panel, wings) in rows.get(side, {}).items():
            candidates.append((1, int(panel), False))
            for other, (other_panel, other_wings) in rows.get(-side, {}).items():
                if name != other:
                    candidates.append((2, int(panel)+int(other_panel),
                                       len(wings)==len(other_wings)==2))
    return max(candidates)
