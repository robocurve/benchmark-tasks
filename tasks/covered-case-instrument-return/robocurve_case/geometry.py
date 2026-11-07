"""Pure geometry for private collision and safe-lid audits; no simulator imports.

Distances are metres and quaternion inputs use the scorer's wxyz convention.
These predicates evaluate sampled state, not continuous collision-free motion.
"""
from __future__ import annotations

from collections.abc import Sequence

import numpy as np

from .design import Mechanism, PARKING_REGION_SIZE
from .scoring import GripperState, ModuleState, Pose, rotation, world_corners


# A parked Covered lid rests directly on the existing tabletop inside a
# virtual set-down region. There is no separate parking-pad solid.
SUPPORT_TOLERANCE = 0.002
CASE_XY_TOLERANCE = 0.006
PARK_XY_TOLERANCE = 0.010


def _vector(value, length: int) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape != (length,) or not np.isfinite(result).all():
        raise ValueError(f"Expected {length} finite coordinates")
    return result


def obb_penetration(a: ModuleState, b: ModuleState) -> float:
    """Return SAT minimum separating translation depth; zero for contact/separation.

    Test both boxes' face normals and all nine edge cross products. The result
    is a penetration depth, not Euclidean separation distance when disjoint.
    """
    ra, rb = rotation(a.pose), rotation(b.pose)
    half_a, half_b = _vector(a.size, 3)/2, _vector(b.size, 3)/2
    if np.any(half_a <= 0) or np.any(half_b <= 0):
        raise ValueError("OBB dimensions must be positive")
    delta = _vector(b.pose.position, 3)-_vector(a.pose.position, 3)
    axes = [*ra.T, *rb.T]
    axes.extend(np.cross(x, y) for x in ra.T for y in rb.T)
    depth = np.inf
    for candidate in axes:
        norm = np.linalg.norm(candidate)
        if norm <= 1e-10:
            continue
        axis = candidate/norm
        radius_a = np.dot(np.abs(ra.T@axis), half_a)
        radius_b = np.dot(np.abs(rb.T@axis), half_b)
        overlap = radius_a+radius_b-abs(np.dot(delta, axis))
        if overlap <= 1e-12:
            return 0.0
        depth = min(depth, overlap)
    return float(depth)


def mechanism_parts(spec: Mechanism, body: ModuleState) -> list[tuple[str, ModuleState]]:
    """Place each compound box at the current body pose, retaining its real size.

    Linear velocity is evaluated at each part's origin, including the angular
    contribution from its offset to the body actor origin.
    """
    r = rotation(body.pose)
    origin = _vector(body.pose.position, 3)
    velocity = _vector(body.linear_velocity, 3)
    omega = _vector(body.angular_velocity, 3)
    result = []
    for part in spec.parts:
        offset = r@_vector(part.position, 3)
        result.append((part.name, ModuleState(
            Pose(tuple(origin+offset), body.pose.quaternion_wxyz), part.size,
            tuple(velocity+np.cross(omega, offset)), body.angular_velocity,
        )))
    return result


def lid_is_safe(spec: Mechanism, body: ModuleState,
                parking: Sequence[tuple[float, float, float]],
                grippers: Sequence[GripperState], table_top: float,
                *, require_case: bool = False) -> bool:
    """Check supported, released, slow lid placement at its case or the table.

    Gripper distances must conservatively cover the whole compound lid, not
    just its panel or body origin. The caller owns the continuous stable hold.
    The panel rests on the case walls. The four skirt edges must rest on the
    tabletop inside a designated region. ``require_case`` is used for
    occupied boxes in the bimanual task: an inspected occupied box must be
    reclosed before success, rather than left on the table.
    """
    if not np.isfinite(table_top):
        raise ValueError("table_top must be finite")
    panels = [part for part in spec.parts if part.name == "panel"]
    if len(panels) != 1:
        raise ValueError("A lid must contain exactly one panel part")
    skirts = [part for part in spec.parts if part.name.startswith("skirt_")]
    if len(skirts) != 4:
        raise ValueError("A removable lid must contain four support skirt edges")
    if len(grippers) != 2:
        raise ValueError("Release requires both robot grippers")
    for gripper in grippers:
        distance, opening = gripper.min_distance_to_intake, gripper.opening
        if not np.isfinite([distance, opening]).all() or opening < 0:
            raise ValueError("Invalid gripper geometry")
        if distance < 0.008 or (distance < 0.060 and opening < 0.045):
            return False
    if (np.linalg.norm(_vector(body.linear_velocity, 3)) > 0.015
            or np.linalg.norm(_vector(body.angular_velocity, 3)) > 0.10):
        return False
    angle = np.arccos(np.clip((np.trace(rotation(body.pose))-1)/2, -1, 1))
    if angle > np.deg2rad(10)+1e-12:
        return False
    panel = panels[0]
    part_states = dict(mechanism_parts(spec, body))
    panel_state = part_states["panel"]
    # world_corners orders the local bottom face at indices 0, 2, 4, 6.
    bottom = world_corners(panel_state)[::2, 2]
    skirt_bottoms = [world_corners(part_states[part.name])[::2] for part in skirts]
    skirt_bottoms = np.concatenate(skirt_bottoms)
    position = _vector(body.pose.position, 3)
    case_origin = _vector(spec.origin, 3)
    case_support = case_origin[2]+panel.position[2]-panel.size[2]/2
    if (np.linalg.norm(position[:2]-case_origin[:2]) <= CASE_XY_TOLERANCE+1e-12
            and np.all(np.abs(bottom-case_support) <= SUPPORT_TOLERANCE+1e-12)):
        return True
    if require_case:
        return False
    park_support = float(table_top)
    for park in parking:
        origin = _vector(park, 3)
        if np.linalg.norm(position[:2]-origin[:2]) > PARK_XY_TOLERANCE+1e-12:
            continue
        if not np.all(np.abs(skirt_bottoms[:, 2]-park_support) <= SUPPORT_TOLERANCE+1e-12):
            continue
        if np.all(np.abs(skirt_bottoms[:, :2]-origin[:2]) <= PARKING_REGION_SIZE/2+1e-12):
            return True
    return False
