# SPDX-License-Identifier: MIT
"""CPU-only physical contact and guide geometry shared by runtime and regrading."""
from __future__ import annotations

import math
import hashlib
from dataclasses import dataclass, fields
from pathlib import Path
import numpy as np


@dataclass(frozen=True)
class ContactConfig:
    BOARD_CENTER: tuple = (.29, 0.0, .7555)
    BOARD_SIZE: tuple = (.14, .28, .016)
    BOARD_TOP: float = .7635
    TARGET_START: tuple = (.22, -.03)
    TARGET_END: tuple = (.36, -.03)
    MARKER_NIB_LOCAL: tuple = (0.0, 0.0, -.05)
    MARKER_NIB_RADIUS: float = .002
    SQUARE_BLADE_LOCAL_EDGE: tuple = ((0.0, -.1175, -.002), (.17, -.1175, -.002))
    SQUARE_STOCK_INNER_X: float = .012
    REQUIRE_GUIDE_CONTACT: bool = False
    REQUIRE_GUIDE_USE: bool = True
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
    return dict(schema="square-cut-line-source-v1", revision=digest.hexdigest(), files=files)


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
                 board_size=(0.14, 0.28, 0.016)):
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
              nib_radius=.002, board_size=(.14, .28, .016), clear_s=0.0):
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


def guide_metrics(point, square_pose, board_pose, guide_force, config=DEFAULT_CONFIG):
    rotation = rotation_xyzw(square_pose[3:])
    local = getattr(config, "SQUARE_BLADE_LOCAL_EDGE", ((0, -.1175, -.002), (.17, -.1175, -.002)))
    edge = np.asarray(local) @ rotation.T + square_pose[:3]
    delta = edge[1]-edge[0]
    norm = float(np.linalg.norm(delta[:2]))
    if norm < 1e-9:
        return dict(guide_used=False, guide_contact_force=float(np.linalg.norm(guide_force)))
    target_delta = np.asarray(config.TARGET_END)-np.asarray(config.TARGET_START)
    target_direction = target_delta / np.linalg.norm(target_delta)
    angle = math.degrees(math.acos(float(np.clip(abs(delta[:2] @ target_direction/norm), 0, 1))))
    direction = delta[:2]/norm
    point_delta = np.asarray(point[:2])-edge[0, :2]
    offset = float(abs(direction[0]*point_delta[1]-direction[1]*point_delta[0]))
    along = float((np.asarray(point[:2])-edge[0, :2]) @ direction)
    board_r = rotation_xyzw(board_pose[3:])
    tilt = math.degrees(math.acos(float(np.clip(rotation[:, 2] @ board_r[:, 2], -1, 1))))
    edge_b = (edge-board_pose[:3]) @ board_r
    flat_error = float(np.max(np.abs(edge_b[:, 2]-config.BOARD_SIZE[2]/2)))
    target_delta_xy = np.asarray(config.TARGET_START)-edge[0, :2]
    target_offset = float(abs(direction[0]*target_delta_xy[1]-direction[1]*target_delta_xy[0]))
    stock_face = square_pose[:3] + rotation @ np.array([getattr(config, "SQUARE_STOCK_INNER_X", .012), 0, 0])
    # The stock's inner face seats against the board's near x edge.
    stock_face_b = board_r.T @ (stock_face-board_pose[:3])
    seat_error = float(abs(stock_face_b[0]+config.BOARD_SIZE[0]/2))
    force = float(np.linalg.norm(guide_force))
    geometrical = (angle <= 2 and tilt <= 2 and flat_error <= .002 and seat_error <= .003
                   and abs(target_offset-.0025) <= .002
                   and offset <= config.MARKER_NIB_RADIUS+.0015
                   and -.002 <= along <= norm+.002)
    used = geometrical and (force > .005 if getattr(config, "REQUIRE_GUIDE_CONTACT", False) else True)
    return dict(guide_used=bool(used), guide_contact_force=force,
                guide_angle_deg=angle, guide_tilt_deg=tilt, guide_flat_error=flat_error,
                guide_nib_distance=offset, guide_target_offset=target_offset,
                guide_seat_error=seat_error)
