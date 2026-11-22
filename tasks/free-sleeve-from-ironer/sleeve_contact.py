"""Contact evidence for a right-hand pinch on the clothed forearm."""
from math import hypot

SLEEVE_COLLIDERS = frozenset(("sleeve_upper", "sleeve_lower", "sleeve_fold", "wrist_cuff"))
FOREARM_SKIN = SLEEVE_COLLIDERS | {"elbow_cuff","upper_arm_sleeve"}
PADS = ("tip_left", "tip_right")
MIN_NORMAL_N = 0.01


def sleeve_pinch(samples):
    """Evaluate a single physics advance. Normals point consistently into one body."""
    loads = {pad: 0.0 for pad in PADS}
    faces = {pad: [] for pad in PADS}
    for pad, point, normal, force in samples:
        if pad in faces and force > MIN_NORMAL_N:
            loads[pad] = max(loads[pad], float(force))
            faces[pad].append((point, normal))
    for a, na in faces["tip_left"]:
        for b, nb in faces["tip_right"]:
            dx, dy = b[0] - a[0], b[1] - a[1]
            span = hypot(dx, dy)
            if span < 0.018:
                continue
            left = (na[0] * dx + na[1] * dy) / span
            right = (nb[0] * dx + nb[1] * dy) / span
            if (left > 0.25 and right < -0.25) or (right > 0.25 and left < -0.25):
                return True, loads
    return False, loads
