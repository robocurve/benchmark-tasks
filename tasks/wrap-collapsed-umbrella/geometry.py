"""Dimensions and scoring measurements for the umbrella prototype (SI units)."""

from dataclasses import dataclass
import math

import numpy as np

TABLE_Z = 0.7475
AXIS = np.array([0.255, 0.0, TABLE_Z + 0.258])
COVER_LENGTH = 0.190
HANDLE_Y = 0.128
TIP_Y = -0.127
LOOSE_RADIUS = 0.048
PLEAT_HALF_WIDTH = 0.013
# Compare the wrapped envelope to the relaxed canopy, including its panel corners.
RELAXED_ENVELOPE_RADIUS = math.hypot(LOOSE_RADIUS, PLEAT_HALF_WIDTH)
CLOSED_RADIUS = 0.80 * RELAXED_ENVELOPE_RADIUS
STRAP_WIDTH = 0.024
PAD_LENGTH = 0.024
STRAP_THICK = 0.0022
STRAP_SEGMENT = 0.016
TAB_LENGTH = 0.031
TAB_PINCH = 0.020
STRAP_COUNT = 13
PAD_SEPARATION = 0.0013    # permitted free space between the two mating faces
PAD_OVERLAP = 0.008
FINGER_CLEARANCE = 0.020
SETTLE_STEPS = 20         # one second at the required 20 Hz
PARTIAL_WRAP = 4.7        # radians: the strap reaches three quarters of the way round
FULL_WRAP = math.radians(365)  # once round the canopy with visible overlap
FASTENED_AREA = 0.000048  # m^2 of hook face still engaged with the loop face
SEAM = AXIS + [0, 0, LOOSE_RADIUS + STRAP_THICK / 2]
PLEAT_ANGLES = np.deg2rad(np.arange(0, 360, 30))


def radial(angle):
    return np.array([math.sin(angle), 0.0, math.cos(angle)])


def tangent(angle):
    return np.array([math.cos(angle), 0.0, -math.sin(angle)])


def strip_frame(direction):
    lengthwise = np.asarray(direction, float)
    lengthwise /= np.linalg.norm(lengthwise)
    across = np.array([0.0, 1.0, 0.0])
    return np.column_stack((lengthwise, across, np.cross(lengthwise, across)))


def open_strap():
    """The long side drapes from its seam with its loose tip above the table."""
    angles = np.deg2rad([9, 27, 45, 63, 81, *([90] * (STRAP_COUNT - 5))])
    vertices = [SEAM]
    for angle in angles:
        vertices.append(vertices[-1] + STRAP_SEGMENT * tangent(angle))
    return np.asarray(vertices)


@dataclass(frozen=True)
class Pad:
    pos: np.ndarray
    rotation: np.ndarray


# A 12 x 12 grid measures the touching area of the sewn hook face.
FACE_GRID = 12
_u = (np.arange(FACE_GRID) + 0.5) / FACE_GRID - 0.5
_x, _y = np.meshgrid(_u * PAD_LENGTH, _u * STRAP_WIDTH)
HOOK_FACE = np.column_stack((_x.ravel(), _y.ravel(), np.full(_x.size, -STRAP_THICK / 2)))
CELL_AREA = PAD_LENGTH * STRAP_WIDTH / FACE_GRID**2


def mating_points(base, tip):
    a, b = base.rotation, tip.rotation
    if a[:, 2] @ b[:, 2] <= 0.90 or a[:, 0] @ b[:, 0] <= 0.86:
        return np.empty((0, 3))
    world = tip.pos + HOOK_FACE @ b.T
    local = (world - base.pos) @ a
    overlap = (abs(local[:,0]) <= PAD_LENGTH/2) & (abs(local[:,1]) <= STRAP_WIDTH/2)
    gap = local[:,2] - STRAP_THICK / 2
    if not overlap.any():
        return np.empty((0, 3))
    touching = overlap & (gap >= -0.0007) & (gap <= PAD_SEPARATION)
    return world[touching]


def pad_mates(base, tip):
    delta = base.rotation.T @ (tip.pos-base.pos)
    return (abs(delta[0]) <= PAD_LENGTH-PAD_OVERLAP and abs(delta[1]) <= STRAP_WIDTH*0.4
            and len(mating_points(base,tip))*CELL_AREA >= FASTENED_AREA)


def press_samples(previous, *, mated, held, touching, normal_load):
    """Count consecutive loaded contacts between overlapping hook-and-loop patches."""
    if mated and held and touching and math.isfinite(normal_load) and normal_load >= 0.15:
        return previous + 1
    return 0


def strap_winding(points, centre):
    """Net angle traced by the physical strap centreline around the shaft."""
    points = np.asarray(points) - centre
    angles = np.unwrap(np.arctan2(points[:, 0], points[:, 2]))
    return float(angles[-1] - angles[0])


@dataclass(frozen=True)
class Outcome:
    """End-state observations. How the strap was wound is not recorded: any route counts."""
    pleat_radius: float
    winding: float
    mated: bool
    latched: bool
    fingers_clear: bool
    still: bool
    handle_held: bool
    table_supported: bool = False
    both_clear: bool = False
    set_down: bool = False


def score_outcome(outcome, released_steps):
    """The highest tier whose observable conditions all hold now.

    0.25 gathered; 0.50 also wrapped three quarters round; 0.75 also wrapped once round with
    overlap and fastened; 1.00 also lowered onto the table, released and retained for one second.
    """
    if not all(map(math.isfinite, (outcome.pleat_radius, outcome.winding))):
        return 0.0
    if not (outcome.handle_held or outcome.table_supported):
        return 0.0
    if outcome.pleat_radius > CLOSED_RADIUS:
        return 0.0
    if outcome.winding < PARTIAL_WRAP:
        return 0.25
    if outcome.winding < FULL_WRAP or not (outcome.latched and outcome.mated):
        return 0.5
    if not (outcome.both_clear and outcome.fingers_clear and outcome.table_supported and outcome.set_down
            and outcome.still and released_steps >= SETTLE_STEPS):
        return 0.75
    return 1.0


def set_down(resting_now, rested_when_released, still_held):
    """Lowered onto the table, not dropped: it rests on the table now, no gripper holds it, and it was
    already resting on the table in the last control in which a gripper held it."""
    return bool(resting_now and rested_when_released and not still_held)


def engaged_cells(previous, base, tip, loaded_points):
    """Keep each hook cell until that pair of material points peels or slides."""
    world = tip.pos + HOOK_FACE @ tip.rotation.T
    local = (world-base.pos) @ base.rotation
    cells = dict(previous)
    for idx, anchor in previous.items():
        motion = local[idx]-anchor
        if motion[2] > .0017 or np.linalg.norm(motion[:2]) > .003:
            del cells[idx]
    if len(loaded_points):
        touching = (loaded_points-tip.pos) @ tip.rotation
        nearest = np.linalg.norm(touching[:,None,:]-HOOK_FACE[None,:,:],axis=2).argmin(axis=1)
        for idx in nearest:
            cells.setdefault(int(idx),local[idx].copy())
    return cells
