"""Conservative world-Y separation of the two complete YAM gripper collision assemblies."""

import json
import numpy as np
from evd.layout import LINKS, ROOT

DATA = json.loads((ROOT / "assets/gripper_hulls.json").read_text())
HULLS = [(LINKS.index(name), np.asarray(points)) for name, points in DATA.items()]
MIN_REFERENCE_GAP_MM = 10.0


def lateral_gap_mm(positions, rotations):
    # Extrema of a linear projection occur at hull vertices. A positive gap is
    # a lower bound on 3-D clearance. A negative gap alone cannot establish contact.
    left, right = float("inf"), -float("inf")
    for i, vertices in HULLS:
        j = i + len(LINKS)
        left = min(left, positions[i, 1] + np.min(vertices @ rotations[i, 1]))
        right = max(right, positions[j, 1] + np.max(vertices @ rotations[j, 1]))
    return float((left - right) * 1000)
