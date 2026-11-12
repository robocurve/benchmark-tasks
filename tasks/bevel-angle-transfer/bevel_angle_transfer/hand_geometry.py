# SPDX-License-Identifier: MIT
"""Conservative YAM finger-surface distances for visible release/support checks.

Per-hull boxes are derived from the pinned robot's collision meshes, including
their URDF origins. Link origins alone are not finger surfaces. The separating
axis distance is a lower bound; zero means the enclosing boxes overlap.
"""
from functools import lru_cache
import json
from pathlib import Path

import numpy as np


@lru_cache(maxsize=1)
def finger_envelopes():
    data = json.loads(Path(__file__).with_name("finger_envelopes.json").read_text(encoding="utf-8"))
    return tuple(np.asarray(data["links"][name], dtype=float) for name in ("tip_left", "tip_right"))


def _axes(rotation_a, rotation_b):
    axes = [*rotation_a.T, *rotation_b.T]
    axes.extend(np.cross(a, b) for a in rotation_a.T for b in rotation_b.T)
    return np.asarray([axis / np.linalg.norm(axis) for axis in axes if np.linalg.norm(axis) > 1e-10])


def box_clearance(center_a, rotation_a, half_a, center_b, rotation_b, half_b):
    """Lower bound on OBB surface clearance using normalized separating axes."""
    axes = _axes(rotation_a, rotation_b)
    delta = center_b - center_a
    radii = np.abs(axes @ rotation_a) @ half_a + np.abs(axes @ rotation_b) @ half_b
    return max(0., float(np.max(np.abs(axes @ delta) - radii)))


def finger_clearances(finger_poses, frame, parts):
    """Return each jaw's minimum surface gap to object-local box parts, metres.

    ``parts`` contains (local centre, half extents), expressed in ``frame``.
    Input finger poses must be the measured tip_left/tip_right link frames.
    """
    from .contact import pose

    values = np.asarray(finger_poses, dtype=float)
    if values.shape != (2, 7) or not np.isfinite(values).all():
        raise ValueError("Expected two finite measured finger link poses")
    origin, rotation = frame
    world_parts = [(origin + rotation @ np.asarray(center), np.asarray(half)) for center, half in parts]
    if not world_parts:
        raise ValueError("At least one object part is required")
    gaps = []
    for value, boxes in zip(values, finger_envelopes()):
        finger_origin, finger_rotation = pose(value)
        axes = _axes(finger_rotation, rotation)
        centers = finger_origin + boxes[:, :3] @ finger_rotation.T
        finger_radii = boxes[:, 3:] @ np.abs(axes @ finger_rotation).T
        part_gaps = []
        for center, half in world_parts:
            radii = finger_radii + np.abs(axes @ rotation) @ half
            gaps_by_hull = np.maximum(0., np.max(np.abs((center - centers) @ axes.T) - radii, axis=1))
            part_gaps.append(float(np.min(gaps_by_hull)))
        gaps.append(min(part_gaps))
    return tuple(gaps)
