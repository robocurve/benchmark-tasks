# SPDX-License-Identifier: MIT
"""Physical contact and rigid-body geometry for the sliding bevel task."""
from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass, fields
from pathlib import Path

import numpy as np

from .hand_geometry import finger_clearances


@dataclass(frozen=True)
class ContactConfig:
    TABLE_TOP: float = .7475
    FIT_GAP: float = .003
    BOARD_CENTER: tuple = (.29, 0., .7555)
    BOARD_SIZE: tuple = (.14, .18, .016)
    BOARD_TOP: float = .7635
    TARGET_START: tuple = (.22, -.03)
    TICK_CENTER: tuple = (.222, -.03)
    MARKER_NIB_LOCAL: tuple = (0., 0., -.05)
    MARKER_NIB_RADIUS: float = .002
    STOCK_SIZE: tuple = (.024, .25, .036)
    BLADE_SIZE: tuple = (.17, .02, .006)
    TAB_CENTER: tuple = (.115, .015, .015)
    TAB_SIZE: tuple = (.018, .018, .024)
    BLADE_PIVOT_LOCAL: tuple = (0., -.1075, .001)
    BLADE_LOCAL_EDGE: tuple = ((0., -.01, -.003), (.17, -.01, -.003))
    REFERENCE_STOCK_SIZE: tuple = (.02, .05, .025)
    REFERENCE_STOCK_CENTER: tuple = (.23, .365, .7600)
    REFERENCE_BLADE_SIZE: tuple = (.14, .02, .025)
    REFERENCE_BLADE_CENTER: tuple = (.295128859211, .127004133252, .7600)
    REFERENCE_ANGLE_DEG: float = -20.
    RETENTION_ANGLE_DEG: float = 2.
    FINGER_CLEARANCE: float = .003
    INK_WIDTH: float = .002
    PHYSICS_DT: float = 1/240


DEFAULT_CONFIG = ContactConfig()


def config_dict(config=DEFAULT_CONFIG):
    return {field.name: getattr(config, field.name, getattr(DEFAULT_CONFIG, field.name))
            for field in fields(ContactConfig)}


def source_manifest(package_dir=None):
    """Hash executable task code and USD assets, excluding evolving reports."""
    root = Path(package_dir) if package_dir is not None else Path(__file__).parent
    paths = sorted([*root.glob("*.py"), *root.glob("assets/*.usda"),
                    *root.glob("finger_envelopes.json"), *root.glob("data/*.json")],
                   key=lambda path: path.relative_to(root).as_posix())
    files = {path.relative_to(root).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
             for path in paths}
    digest = hashlib.sha256()
    for name, checksum in files.items():
        digest.update(name.encode("utf-8")+b"\0"+checksum.encode("ascii")+b"\n")
    return dict(schema="bevel-angle-transfer-source-v1", revision=digest.hexdigest(), files=files)


def rotation_xyzw(quaternion):
    """Rotation matrix for Isaac Lab 3's xyzw root pose."""
    q = np.asarray(quaternion, dtype=float)
    norm = float(np.linalg.norm(q))
    if q.shape != (4,) or not np.isfinite(q).all() or not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("Invalid xyzw quaternion")
    x, y, z, w = q / norm
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def contact_gate(force, point, marker_pose, board_pose, *,
                 nib_local=(0.0, 0.0, -0.05), nib_radius=0.002,
                 board_size=(0.14, 0.18, 0.016)):
    """Fail closed unless a measured marker/board pair contacts at the nib.

    Return (accepted, diagnostics). Coordinates are rigid-body root poses,
    never bounding-box centres. The localized collision point establishes
    contact. Force is diagnostic.
    """
    force, point = np.asarray(force, float), np.asarray(point, float)
    marker_pose, board_pose = np.asarray(marker_pose, float), np.asarray(board_pose, float)
    arrays = (force, point, marker_pose, board_pose)
    if (force.shape != (3,) or point.shape != (3,)
            or marker_pose.shape != (7,) or board_pose.shape != (7,)
            or not all(np.isfinite(a).all() for a in arrays[1:])):
        return False, {"reason": "nonfinite_or_invalid_contact"}
    try:
        marker_r = rotation_xyzw(marker_pose[3:])
        board_r = rotation_xyzw(board_pose[3:])
    except ValueError:
        return False, {"reason": "invalid_pose"}
    nib_w = marker_pose[:3] + marker_r @ np.asarray(nib_local)
    point_b = board_r.T @ (point-board_pose[:3])
    nib_b = board_r.T @ (nib_w-board_pose[:3])
    size = np.asarray(board_size, float)
    normal = board_r[:, 2]
    tilt = math.degrees(math.acos(float(np.clip(marker_r[:, 2] @ normal, -1, 1))))
    force_n = float(abs(force @ normal)) if np.isfinite(force).all() else None
    tip_distance = float(np.linalg.norm(point-nib_w))
    surface_error = float(abs(point_b[2]-size[2]/2))
    nib_surface_error = float(abs(nib_b[2]-nib_radius-size[2]/2))
    inside = bool(np.all(np.abs(point_b[:2]) <= size[:2]/2))
    accepted = (tip_distance <= nib_radius+.001
                and surface_error <= .001 and nib_surface_error <= .002 and inside)
    return bool(accepted), dict(
        reason="nib_contact" if accepted else "contact_gate_rejected",
        force_n=force_n, tip_distance=tip_distance,
        surface_error=surface_error, nib_surface_error=nib_surface_error,
        marker_tilt_deg=tilt, point_board=point_b.tolist(),
        nib_world=nib_w.tolist(), inside_board=inside,
    )


def lift_gate(marker_pose, board_pose, quiet_s, *, nib_local=(0, 0, -.05),
              nib_radius=.002, board_size=(.14, .18, .016), clear_s=0.0):
    """Measure final nib clearance from the board."""
    marker, board = np.asarray(marker_pose, float), np.asarray(board_pose, float)
    if (marker.shape != (7,) or board.shape != (7,)
            or not np.isfinite(marker).all() or not np.isfinite(board).all()):
        return False, {"nib_clearance": None, "ink_quiet_s": None, "lift_clear_s": None}
    try:
        nib_w = marker[:3]+rotation_xyzw(marker[3:]) @ np.asarray(nib_local)
        nib_b = rotation_xyzw(board[3:]).T @ (nib_w-board[:3])
    except ValueError:
        return False, {"nib_clearance": None, "ink_quiet_s": float(quiet_s), "lift_clear_s": float(clear_s)}
    clearance = float(nib_b[2]-nib_radius-board_size[2]/2)
    return bool(clearance >= .005-1e-9), dict(
        nib_clearance=clearance, ink_quiet_s=float(quiet_s), lift_clear_s=float(clear_s))


def target_endpoints(config=DEFAULT_CONFIG):
    """Fixed board-local target constructed from the reference angle and board width."""
    start = np.asarray(config.TARGET_START) - np.asarray(config.BOARD_CENTER[:2])
    width = config.BOARD_SIZE[0]
    end = start + [width, width*math.tan(math.radians(config.REFERENCE_ANGLE_DEG))]
    return start.tolist(), end.tolist()


def pose(value):
    """Return (origin, rotation) for a finite world xyzw pose."""
    value = np.asarray(value, dtype=float)
    if value.shape != (7,) or not np.isfinite(value).all():
        raise ValueError("Invalid rigid-body pose")
    return value[:3], rotation_xyzw(value[3:])


def angle_deg(a, b):
    return math.degrees(math.acos(float(np.clip(a @ b, -1, 1))))


def angle_difference(a, b):
    return abs((a-b+180.) % 360.-180.)


def relative_angle(stock, blade, config=DEFAULT_CONFIG):
    """Measure the joint from the tool poses and reject a detached blade frame."""
    stock_origin, stock_r = stock
    blade_origin, blade_r = blade
    pivot = stock_origin+stock_r @ config.BLADE_PIVOT_LOCAL
    if np.linalg.norm(blade_origin-pivot) > .003:
        return None
    direction = stock_r.T @ blade_r[:, 0]
    return math.degrees(math.atan2(direction[1], direction[0]))


def _face(frame, size, axis, side, *, blade=False):
    origin, rotation = frame
    half = np.asarray(size) / 2
    center = origin + side * half[axis] * rotation[:, axis]
    if blade:
        center = center + half[0] * rotation[:, 0]
    return center, rotation[:, axis] * side, rotation[:, 1-axis], rotation[:, 2], half[1-axis], half[2]


def _face_gaps(tool, fixed, *, full_face, tolerance):
    """Measure endpoint gaps over a finite mating segment and shared face height."""
    center, normal, along, up, half_length, half_height = tool
    datum, fixed_normal, fixed_along, fixed_up, fixed_length, fixed_height = fixed
    if normal @ fixed_normal >= 0 or up @ fixed_up <= 0:
        return None
    projected_ends = sorted(float((center + sign * half_length * along - datum) @ fixed_along)
                            for sign in (-1, 1))
    if full_face:
        lo, hi = -fixed_length, fixed_length
    else:
        lo, hi = max(-fixed_length, projected_ends[0]), min(fixed_length, projected_ends[1])
        if hi <= lo:
            return None
    # A sloping face can be separated at one end even if it touches at the other.
    along_projection = float(along @ fixed_along)
    if abs(along_projection) < 1e-6:
        return None
    gaps = []
    for endpoint in (lo, hi):
        run = (endpoint - (center-datum) @ fixed_along) / along_projection
        edge = center + np.clip(run, -half_length, half_length) * along
        edge_z = (edge-datum) @ fixed_up
        vertical_half = half_height * (up @ fixed_up)
        bottom, top = max(-fixed_height, edge_z-vertical_half), min(fixed_height, edge_z+vertical_half)
        if top <= bottom:
            return None
        height = (bottom+top)/2
        fixed_point = datum + endpoint*fixed_along + height*fixed_up
        tool_point = edge + ((height-edge_z)/(up @ fixed_up))*up
        gaps.append(float(np.linalg.norm(fixed_point-tool_point)))
    return gaps if max(gaps) <= tolerance + 1e-9 else None


def capture_contacts(record, stock, blade, reference_stock, reference_blade, config=DEFAULT_CONFIG):
    """Check both endpoints of each of the fixture's mating faces."""
    return all(_face_gaps(tool, fixed, full_face=True, tolerance=config.FIT_GAP) is not None
               for tool, fixed in (
                   (_face(stock, config.STOCK_SIZE, 0, 1),
                    _face(reference_stock, config.REFERENCE_STOCK_SIZE, 0, -1)),
                   (_face(blade, config.BLADE_SIZE, 1, -1, blade=True),
                    _face(reference_blade, config.REFERENCE_BLADE_SIZE, 1, 1)),
               ))


def _measured_finger_gaps(record, prefix, frame, parts):
    half_parts = tuple((center, np.asarray(size)/2) for center, size in parts)
    try:
        return finger_clearances(record.get(f"{prefix}_finger_poses"), frame, half_parts)
    except (TypeError, ValueError):
        return None


def _blade_finger_gaps(record, blade, config):
    parts = (((config.BLADE_SIZE[0]/2, 0., 0.), config.BLADE_SIZE),
             (config.TAB_CENTER, config.TAB_SIZE))
    return _measured_finger_gaps(record, "right", blade, parts)


def blade_released(record, blade, config=DEFAULT_CONFIG):
    """Observe an open right gripper with both fingers withdrawn from blade and tab."""
    if record.get("right_gripper_open") is not True:
        return False
    gaps = _blade_finger_gaps(record, blade, config)
    return gaps is not None and min(gaps) >= config.FINGER_CLEARANCE


def blade_regrasped(record, blade, config=DEFAULT_CONFIG):
    """Detect closed fingers coming back to the blade or adjustment tab."""
    opening = record.get("right_gripper_open")
    if opening is True:
        return False
    gaps = _blade_finger_gaps(record, blade, config)
    return opening is not False or gaps is None or min(gaps) < config.FINGER_CLEARANCE


def blade_recontacted(record, blade, config=DEFAULT_CONFIG):
    """A withdrawn hand must not return to push the blade, even with open jaws."""
    gaps = _blade_finger_gaps(record, blade, config)
    return gaps is None or min(gaps) < config.FINGER_CLEARANCE


def stock_carried(record, stock, config=DEFAULT_CONFIG):
    """Check that both jaws remain closed on the stock during carry."""
    if record.get("left_gripper_open") is not False:
        return False
    gaps = _measured_finger_gaps(record, "left", stock, (((0., 0., 0.), config.STOCK_SIZE),))
    return gaps is not None and max(gaps) <= config.FINGER_CLEARANCE


def _part_ground_clearance(frame, center, size, config):
    origin, rotation = frame
    return float(origin[2] + rotation[2] @ center
                 - np.abs(rotation[2]) @ (np.asarray(size)/2) - config.TABLE_TOP)


def stock_ground_clearance(stock, config=DEFAULT_CONFIG):
    """Vertical distance of the lowest stock corner from the tabletop."""
    return _part_ground_clearance(stock, (0., 0., 0.), config.STOCK_SIZE, config)


def gauge_ground_clearance(stock, blade, config=DEFAULT_CONFIG):
    """Vertical distance of the lowest gauge surface from the tabletop."""
    return min(stock_ground_clearance(stock, config),
               _part_ground_clearance(blade, (config.BLADE_SIZE[0]/2, 0., 0.), config.BLADE_SIZE, config),
               _part_ground_clearance(blade, config.TAB_CENTER, config.TAB_SIZE, config))


def stock_at_fixture(stock, reference_stock, config=DEFAULT_CONFIG):
    """Check the stock face while the gauge still rests at the fixture."""
    return _face_gaps(_face(stock, config.STOCK_SIZE, 0, 1),
                      _face(reference_stock, config.REFERENCE_STOCK_SIZE, 0, -1),
                      full_face=True, tolerance=config.FIT_GAP) is not None


def stock_at_board(stock, board, config=DEFAULT_CONFIG):
    """Check finite near-face seating at the board."""
    return _face_gaps(_face(stock, config.STOCK_SIZE, 0, 1),
                      _face(board, config.BOARD_SIZE, 0, -1),
                      full_face=False, tolerance=config.FIT_GAP) is not None


def board_placement(stock, blade, board, config=DEFAULT_CONFIG):
    """Check finite near-face seating and blade height over the board."""
    edge = np.asarray(config.BLADE_LOCAL_EDGE) @ blade[1].T + blade[0]
    edge_b = (edge-board[0]) @ board[1]
    delta = edge_b[1]-edge_b[0]
    half = np.asarray(config.BOARD_SIZE)/2
    if not stock_at_board(stock, board, config) or delta[0] <= 1e-9:
        return False, edge_b
    # Check only the section above the board, so an overhanging blade is allowed.
    lo = max(0., (-half[0]-edge_b[0, 0])/delta[0])
    hi = min(1., (half[0]-edge_b[0, 0])/delta[0])
    if hi <= lo:
        return False, edge_b
    ends = edge_b[0]+np.array([lo, hi])[:, None]*delta
    inside_near_edge = abs(ends[0, 1]) <= half[1] and ends[0, 0] <= -half[0]+config.FIT_GAP
    flat = np.max(np.abs(ends[:, 2]-half[2])) <= config.FIT_GAP + 1e-9
    return bool(inside_near_edge and flat), edge_b


def guide_metrics(point, stock, blade, board, guide_force, stock_board_force, config=DEFAULT_CONFIG):
    """Locate the marking tip alongside the seated blade's working edge."""
    placement, edge_b = board_placement(stock, blade, board, config)
    point = np.asarray(point, dtype=float)
    if point.shape != (3,) or not np.isfinite(point).all():
        return dict(guide_placement=placement, guide_geometry=False)
    point_b = board[1].T @ (point-board[0])
    delta = edge_b[1, :2]-edge_b[0, :2]
    length = float(np.linalg.norm(delta))
    if length < 1e-9:
        return dict(guide_placement=placement, guide_geometry=False)
    direction = delta/length
    normal = np.array([-direction[1], direction[0]])
    offset = float((point_b[:2]-edge_b[0, :2]) @ normal)
    along = float((point_b[:2]-edge_b[0, :2]) @ direction)
    beside_edge = (-config.MARKER_NIB_RADIUS-.002 <= offset <= .001
                   and -.002 <= along <= length+.002)
    return dict(guide_placement=placement, guide_geometry=bool(placement and beside_edge),
                guide_nib_distance=abs(offset), guide_nib_offset=offset)

