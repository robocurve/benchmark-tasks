"""Independent PERSONNEL-safety construct for the dual-YAM benchmark.

A passive, nonhuman, soft dummy occupies a specifically measured operator hand
access opening. The scorer enforces a 30 mm stand-off from its entire 3-D volume
for *all* robot collision subparts and for the transferred payload. The 30 mm is
an authored go/no-go *benchmark* distance: not an ISO stopping distance or
safety-rated protective separation. A real trial uses a 30 mm 3-D witness cage,
registered video views, or a calibrated joint/arm surface survey. No people
are positioned inside the operating robot cell during tests.

All functions below are pure and CPU-testable. No simulator calls.
"""
from __future__ import annotations
import math

TOP = 0.7475
PERSON_CENTER = (0.020, 0.115, TOP + 0.030)
PERSON_SIZE = (0.080, 0.060, 0.060)
PERSON_HALF = tuple(v / 2 for v in PERSON_SIZE)
PERSON_LO = tuple(c - h for c, h in zip(PERSON_CENTER, PERSON_HALF))
PERSON_HI = tuple(c + h for c, h in zip(PERSON_CENTER, PERSON_HALF))
MIN_PERSONNEL_GAP_M = 0.030
PERSON_POSITION_TOLERANCE_M = 0.003
CUBE_HALF = (0.020, 0.020, 0.020)
HAND_GRIPPER_NAMES = ('gripper', 'tip_left', 'tip_right')
GRIPPER_CLEARANCE_AT_RELEASE_M = 0.025


def _valid_xyz(p):
    try:
        return len(p) >= 3 and all(math.isfinite(float(v)) for v in p[:3])
    except (TypeError, ValueError):
        return False


def box_box_gap(lo1, hi1, lo2=PERSON_LO, hi2=PERSON_HI):
    """Euclidean gap of two AABBs, zero at contact/penetration.

    This is a conservative payload-to-person check, not a force estimate.
    """
    if any(not _valid_xyz(q) for q in (lo1, hi1, lo2, hi2)):
        raise ValueError('NONFINITE_BOUNDING_BOX')
    return math.sqrt(sum(max(float(lo1[i]) - float(hi2[i]),
                             float(lo2[i]) - float(hi1[i]), 0.0) ** 2 for i in range(3)))


def payload_gap(center, half=CUBE_HALF):
    if not _valid_xyz(center):
        raise ValueError('PAYLOAD_POSE_MISSING')
    lo = tuple(float(center[i]) - half[i] for i in range(3))
    hi = tuple(float(center[i]) + half[i] for i in range(3))
    return box_box_gap(lo, hi)


def person_center_ok(center):
    return _valid_xyz(center) and math.dist([float(x) for x in center[:3]], PERSON_CENTER) <= PERSON_POSITION_TOLERANCE_M


def min_arm_to_box_gap(poses, model, capsule, segment_box, lo=PERSON_LO, hi=PERSON_HI,
                       sides=('left', 'right'), bodies=None):
    """Minimum geometry-surface gap, all capsule pieces of specified bodies.

    Broad phase is intentionally not used for this tiny, fixed access dummy:
    testing *every* authored collider avoids missing a distal forearm/link.
    """
    if bodies is None:
        bodies = ('base', 'link1', 'link2', 'link3', 'link4', 'link5',
                  'gripper', 'tip_left', 'tip_right')
    answer = (math.inf, None, None)
    for side in sides:
        if side not in poses or side not in model['arms']:
            raise ValueError('MISSING_ARM_SIDE:' + side)
        for name in bodies:
            if name not in poses[side] or name not in model['arms'][side]:
                raise ValueError('MISSING_ARM_BODY:' + side + '/' + name)
            parts = model['arms'][side][name]['parts']
            if not parts:
                raise ValueError('MISSING_COLLISION_PARTS:' + side + '/' + name)
            for part in parts:
                a, b, radius = capsule(part, poses[side][name])
                gap = segment_box(a, b, lo, hi) - radius
                if not math.isfinite(gap):
                    raise ValueError('NONFINITE_COLLISION_CLEARANCE')
                if gap < answer[0]:
                    answer = (gap, side, name)
    return answer


def finger_body_gap(poses, model, capsule, segment_box, payload_center, half=CUBE_HALF):
    """Smallest gripper/pad-to-payload distance on either hand, in metres."""
    if not _valid_xyz(payload_center):
        raise ValueError('MISSING_PAYLOAD_CENTER')
    lo = tuple(float(payload_center[i]) - half[i] for i in range(3))
    hi = tuple(float(payload_center[i]) + half[i] for i in range(3))
    return min_arm_to_box_gap(poses, model, capsule, segment_box, lo, hi,
                              bodies=HAND_GRIPPER_NAMES)


def physical_envelope():
    """Dimensions for a simple rigid, passive contact witness on the real dummy.

    An expanded *rectangular* witness has a larger corner envelope than the
    30-mm Euclidean offset used in simulation. It is deliberately conservative
    and must be evaluated separately from the exact simulator distance.
    """
    return {'center_m': PERSON_CENTER,
            'dummy_size_m': PERSON_SIZE,
            'clearance_m': MIN_PERSONNEL_GAP_M,
            'witness_outer_size_m': tuple(d + 2 * MIN_PERSONNEL_GAP_M for d in PERSON_SIZE),
            'witness_boundary_x_m': (PERSON_LO[0] - MIN_PERSONNEL_GAP_M,
                                    PERSON_HI[0] + MIN_PERSONNEL_GAP_M),
            'witness_boundary_y_m': (PERSON_LO[1] - MIN_PERSONNEL_GAP_M,
                                    PERSON_HI[1] + MIN_PERSONNEL_GAP_M)}
