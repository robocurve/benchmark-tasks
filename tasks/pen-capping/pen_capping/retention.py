"""Chosen liner dimensions and geometric eligibility; no elastomer force model.

All lengths are metres. A silicone sleeve is primer-bonded into the PP
shell's recessed seat. Only its stiff outer backing is a rigid collider. The
interfering inner zone is visual and represented by the retention joint.
"""
import math
import numpy as np

BARREL_RADIUS = .0097
BARREL_START = -.070
BARREL_END = .070
NIB_TIP = -.0924
SHELL_APOTHEM = .0125
LINER_OUTER_APOTHEM = .0129
LINER_START = .010
LINER_END = .016
MOUTH_X = .0175
LINER_INNER_RADIUS = .0095
LEAD_IN_RADIUS = .0105
BACKING_INNER_RADIUS = .0106
MIN_OVERLAP = .003
MAX_ECCENTRICITY = .0003
MAX_COMPRESSION = .00055
CLOSED_INNER_X = -.0145
SEATED_X = -.0821


def engagement(relative, cap_rotation=None):
    """Intersection of the barrel axis with the liner's actual axial interval.

    relative and cap_rotation describe the cap in pen coordinates. Report the
    conservative full-section overlap (trimmed for barrel tilt), maximum axis
    eccentricity through that overlap and largest radial interference. These
    are geometric quantities, not measured contact pressure or holding force.
    """
    relative = np.asarray(relative, dtype=float)
    rotation = np.eye(3) if cap_rotation is None else np.asarray(cap_rotation, dtype=float)
    if relative.shape != (3,) or rotation.shape != (3, 3) or not np.all(np.isfinite(relative)) or not np.all(np.isfinite(rotation)):
        return dict(eligible=False, overlap_m=0., eccentricity_m=None, compression_m=None)
    axis = rotation.T @ np.array([1., 0., 0.])
    origin = -rotation.T @ relative
    axis_angle=math.degrees(math.acos(float(np.clip(axis[0],-1.,1.))))
    if axis[0] <= 1e-8:
        return dict(eligible=False, overlap_m=0., eccentricity_m=None, compression_m=None,
                    axis_angle_deg=axis_angle, failed_conditions=['axis_direction'])
    rim = BARREL_RADIUS * math.sqrt(max(0., 1. - axis[0] ** 2))
    low = max(LINER_START, origin[0] + BARREL_START * axis[0] + rim)
    high = min(LINER_END, origin[0] + BARREL_END * axis[0] - rim)
    overlap = max(0., high - low)
    endpoints = [origin + (x - origin[0]) / axis[0] * axis for x in (low, high)]
    eccentricity = max(float(np.linalg.norm(p[1:])) for p in endpoints)
    compression = BARREL_RADIUS / axis[0] + eccentricity - LINER_INNER_RADIUS
    tip_clearance = origin[0] + NIB_TIP * axis[0] - .003 * math.sqrt(max(0., 1. - axis[0] ** 2)) - CLOSED_INNER_X
    checks = dict(axis_angle=axis[0] >= math.cos(math.radians(2)),
                  overlap=overlap >= MIN_OVERLAP, eccentricity=eccentricity <= MAX_ECCENTRICITY,
                  compression=compression <= MAX_COMPRESSION, tip_clearance=tip_clearance >= .0004)
    return dict(eligible=bool(all(checks.values())), overlap_m=overlap,
                eccentricity_m=eccentricity, compression_m=compression,
                tip_clearance_m=tip_clearance, axis_alignment=float(axis[0]),
                axis_angle_deg=axis_angle, overlap_interval_m=[low,high],
                failed_conditions=[key for key,passed in checks.items() if not passed])
