# SPDX-License-Identifier: MIT
"""Physical contact and rigid-body geometry for the sliding bevel task."""
from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass, fields
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class ContactConfig:
    BOARD_CENTER: tuple = (.29, 0., .7555)
    BOARD_SIZE: tuple = (.14, .18, .016)
    BOARD_TOP: float = .7635
    TARGET_START: tuple = (.22, -.03)
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
    INITIAL_REFERENCE_ERROR_DEG: float = 15.
    REQUIRE_GUIDE_CONTACT: bool = False
    CAPTURE_ANGLE_DEG: float = 2.
    CAPTURE_HOLD_S: float = .3
    CAPTURE_ANGLE_SPAN_DEG: float = .25
    CAPTURE_STOCK_SPAN: float = .0005
    ADJUSTMENT_DEG: float = 10.
    RETENTION_ANGLE_DEG: float = 2.
    TRANSFER_LIFT: float = .02
    TRANSFER_MIN_AIRBORNE_HEIGHT: float = .01
    TRANSFER_TRANSLATION: float = .08
    FINGER_CLEARANCE: float = .018
    RELEASE_HOLD_S: float = .1
    PAIR_FORCE_MIN: float = .005
    INK_WIDTH: float = .002
    PHYSICS_DT: float = 1/240


DEFAULT_CONFIG = ContactConfig()


def config_dict(config=DEFAULT_CONFIG):
    return {field.name: getattr(config, field.name, getattr(DEFAULT_CONFIG, field.name))
            for field in fields(ContactConfig)}


def source_manifest(package_dir=None):
    """Hash executable task code and USD assets, excluding evolving reports."""
    root = Path(package_dir) if package_dir is not None else Path(__file__).parent
    paths = sorted([*root.glob("*.py"), *root.glob("assets/*.usda")],
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
    if q.shape != (4,) or not np.isfinite(q).all() or norm < 1e-12:
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
    never bounding-box centres. The force threshold is applied here because
    ContactSensorCfg.force_threshold only affects contact-duration tracking.
    """
    force, point = np.asarray(force, float), np.asarray(point, float)
    marker_pose, board_pose = np.asarray(marker_pose, float), np.asarray(board_pose, float)
    arrays = (force, point, marker_pose, board_pose)
    if (force.shape != (3,) or point.shape != (3,)
            or marker_pose.shape != (7,) or board_pose.shape != (7,)
            or not all(np.isfinite(a).all() for a in arrays)):
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
    force_n = float(abs(force @ normal))
    tip_distance = float(np.linalg.norm(point-nib_w))
    surface_error = float(abs(point_b[2]-size[2]/2))
    nib_surface_error = float(abs(nib_b[2]-nib_radius-size[2]/2))
    inside = bool(np.all(np.abs(point_b[:2]) <= size[:2]/2))
    accepted = (force_n > 0.005 and tip_distance <= nib_radius+0.002
                and surface_error <= 0.001 and nib_surface_error <= 0.002
                and inside and tilt <= 25.0)
    return bool(accepted), dict(
        reason="nib_contact" if accepted else "contact_gate_rejected",
        force_n=force_n, tip_distance=tip_distance,
        surface_error=surface_error, nib_surface_error=nib_surface_error,
        marker_tilt_deg=tilt, point_board=point_b.tolist(),
        nib_world=nib_w.tolist(), inside_board=inside,
    )


def lift_gate(marker_pose, board_pose, quiet_s, *, nib_local=(0, 0, -.05),
              nib_radius=.002, board_size=(.14, .18, .016), clear_s=0.0):
    """Require a physical nib lift held continuously for half a second."""
    marker, board = np.asarray(marker_pose, float), np.asarray(board_pose, float)
    if (marker.shape != (7,) or board.shape != (7,)
            or not np.isfinite(marker).all() or not np.isfinite(board).all()
            or not math.isfinite(quiet_s) or not math.isfinite(clear_s)):
        return False, {"nib_clearance": None, "ink_quiet_s": None, "lift_clear_s": None}
    try:
        nib_w = marker[:3]+rotation_xyzw(marker[3:]) @ np.asarray(nib_local)
        nib_b = rotation_xyzw(board[3:]).T @ (nib_w-board[:3])
    except ValueError:
        return False, {"nib_clearance": None, "ink_quiet_s": float(quiet_s), "lift_clear_s": float(clear_s)}
    clearance = float(nib_b[2]-nib_radius-board_size[2]/2)
    return bool(clearance >= .005-1e-9 and quiet_s >= .5-1e-9 and clear_s >= .5-1e-9), dict(
        nib_clearance=clearance, ink_quiet_s=float(quiet_s), lift_clear_s=float(clear_s))


def advance_lift_hold(held_ticks, accepted_ink, marker_pose, board_pose, config=DEFAULT_CONFIG):
    """Count consecutive observed physics samples with at least 5 mm clearance."""
    _, details = lift_gate(marker_pose, board_pose, 0.0,
                           nib_local=config.MARKER_NIB_LOCAL, nib_radius=config.MARKER_NIB_RADIUS,
                           board_size=config.BOARD_SIZE)
    clearance = details["nib_clearance"]
    return held_ticks+1 if not accepted_ink and clearance is not None and clearance >= .005-1e-9 else 0



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
    """Measure the joint from poses and reject detached/non-planar blade frames."""
    stock_origin, stock_r = stock
    blade_origin, blade_r = blade
    pivot = stock_origin+stock_r @ config.BLADE_PIVOT_LOCAL
    if np.linalg.norm(blade_origin-pivot) > .003 or angle_deg(stock_r[:, 2], blade_r[:, 2]) > 2:
        return None
    direction = stock_r.T @ blade_r[:, 0]
    return math.degrees(math.atan2(direction[1], direction[0]))


def _face_contact(point, force, frame, size, axis, side, threshold, *, blade=False):
    origin, rotation = frame
    point = np.asarray(point, dtype=float)
    force = np.asarray(force, dtype=float)
    if point.shape != (3,) or force.shape != (3,) or not np.isfinite([point, force]).all():
        return False
    local = rotation.T @ (point-origin)
    if blade:
        # The blade actor frame is at the pivot, outside its centre of mass.
        local[0] -= size[0]/2
    half = np.asarray(size)/2
    return bool(abs(force @ rotation[:, axis]) > threshold
                and abs(local[axis]-side*half[axis]) <= .003
                and np.all(np.abs(local) <= half+.003))


def capture_contacts(record, stock, blade, reference_stock, reference_blade, config=DEFAULT_CONFIG):
    """Check measured side contacts on both mating tool and reference faces."""
    stock_point, blade_point = record["reference_stock_point"], record["reference_blade_point"]
    stock_force, blade_force = record["reference_stock_force"], record["reference_blade_force"]
    threshold = config.PAIR_FORCE_MIN
    yaw = math.radians(config.REFERENCE_ANGLE_DEG)
    reference_direction = np.array([math.cos(yaw), math.sin(yaw), 0.])
    vertical = np.array([0., 0., 1.])
    return (
        np.linalg.norm(reference_stock[0]-config.REFERENCE_STOCK_CENTER) <= .003
        and np.linalg.norm(reference_blade[0]-config.REFERENCE_BLADE_CENTER) <= .003
        and angle_deg(reference_stock[1][:, 0], np.array([1., 0., 0.])) <= .5
        and angle_deg(reference_blade[1][:, 0], reference_direction) <= .5
        and angle_deg(reference_stock[1][:, 2], vertical) <= .5
        and angle_deg(reference_blade[1][:, 2], vertical) <= .5
        and angle_deg(stock[1][:, 0], reference_stock[1][:, 0]) <= 2.
        and angle_deg(blade[1][:, 0], reference_blade[1][:, 0]) <= 2.
        and angle_deg(stock[1][:, 2], reference_stock[1][:, 2]) <= 2.
        and angle_deg(blade[1][:, 2], reference_blade[1][:, 2]) <= 2.
        and _face_contact(stock_point, stock_force, stock, config.STOCK_SIZE, 0, 1, threshold)
        and _face_contact(stock_point, stock_force, reference_stock, config.REFERENCE_STOCK_SIZE, 0, -1, threshold)
        and _face_contact(blade_point, blade_force, blade, config.BLADE_SIZE, 1, -1, threshold, blade=True)
        and _face_contact(blade_point, blade_force, reference_blade, config.REFERENCE_BLADE_SIZE, 1, 1, threshold)
    )


def blade_released(record, blade, config=DEFAULT_CONFIG):
    """Check quiet blade-finger pairs and TCP clearance from blade and raised tab."""
    forces = np.asarray(record["right_finger_blade_forces"], dtype=float)
    tcp = np.asarray(record["right_tcp_position"], dtype=float)
    if forces.shape != (2, 3) or tcp.shape != (3,) or not np.isfinite(forces).all() or not np.isfinite(tcp).all():
        return False
    local = blade[1].T @ (tcp-blade[0])
    body_center = (config.BLADE_SIZE[0]/2, 0., 0.)
    clearance = min(
        np.linalg.norm(np.maximum(np.abs(local-center)-np.asarray(size)/2, 0.))
        for center, size in ((body_center, config.BLADE_SIZE), (config.TAB_CENTER, config.TAB_SIZE))
    )
    return bool(clearance >= config.FINGER_CLEARANCE
                and np.max(np.linalg.norm(forces, axis=1)) <= config.PAIR_FORCE_MIN)


def guide_metrics(point, stock, blade, board, guide_force, stock_board_force, config=DEFAULT_CONFIG):
    """Test whether the marker contact was beside a seated and aligned bevel."""
    board_origin, board_r = board
    stock_origin, stock_r = stock
    blade_origin, blade_r = blade
    edge = np.asarray(config.BLADE_LOCAL_EDGE) @ blade_r.T + blade_origin
    edge_b = (edge-board_origin) @ board_r
    delta = edge_b[1, :2]-edge_b[0, :2]
    length = float(np.linalg.norm(delta))
    guide_force = np.asarray(guide_force, dtype=float)
    if (length < 1e-9 or guide_force.shape != (3,) or not np.isfinite(guide_force).all()):
        return dict(guide_geometry=False)
    direction = delta/length
    normal = np.array([-direction[1], direction[0]])
    start, end = map(np.asarray, target_endpoints(config))
    target = end-start
    angle = angle_deg(direction, target/np.linalg.norm(target))
    point_b = board_r.T @ (np.asarray(point)-board_origin)
    offset = float(abs((point_b[:2]-edge_b[0, :2]) @ normal))
    along = float((point_b[:2]-edge_b[0, :2]) @ direction)
    tilt = angle_deg(blade_r[:, 2], board_r[:, 2])
    flat = float(np.max(np.abs(edge_b[:, 2]-config.BOARD_SIZE[2]/2)))
    target_offset = float((edge_b[0, :2]-start) @ normal)
    stock_face = stock_origin + stock_r[:, 0]*config.STOCK_SIZE[0]/2
    stock_face_b = board_r.T @ (stock_face-board_origin)
    # Verify seating near the working end, plus face alignment along the edge.
    pivot_face = stock_face_b+board_r.T @ stock_r[:, 1]*config.BLADE_PIVOT_LOCAL[1]
    seat = float(abs(pivot_face[0]+config.BOARD_SIZE[0]/2))
    stock_angle = angle_deg(stock_r[:, 0], board_r[:, 0])
    force = float(np.linalg.norm(guide_force))
    stock_board_force = np.asarray(stock_board_force, dtype=float)
    seat_force = (float(abs(stock_board_force @ board_r[:, 0]))
                  if stock_board_force.shape == (3,) and np.isfinite(stock_board_force).all() else 0.)
    placement = (angle <= 2. and tilt <= 2. and flat <= .002 and seat <= .003
                 and seat_force > config.PAIR_FORCE_MIN
                 and stock_angle <= 2. and abs(target_offset-.0025) <= .002)
    geometrical = (placement and offset <= config.MARKER_NIB_RADIUS+.0015
                   and -.002 <= along <= length+.002)
    return dict(guide_placement=bool(placement),
                guide_geometry=bool(geometrical and (not config.REQUIRE_GUIDE_CONTACT
                                                    or force > config.PAIR_FORCE_MIN)),
                guide_contact_force=force, guide_angle_deg=angle, guide_tilt_deg=tilt,
                guide_flat_error=flat, guide_nib_distance=offset,
                guide_target_offset=target_offset, guide_seat_error=seat, guide_seat_force=seat_force)







