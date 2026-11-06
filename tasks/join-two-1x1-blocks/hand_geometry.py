"""Conservative YAM hand collision envelopes from the installed robot assets.

Every collision STL in ``yam_arena.YAM_URDF`` for ``gripper`` (the palm/housing),
``tip_left`` and ``tip_right`` contributes to its link-frame bounding box. The
URDF mesh scale and collision origin are applied before taking bounds. These
three complete-link boxes can overestimate occupied space, but cannot omit a
finger carriage or palm surface as a TCP/pad-only approximation would.

At runtime, transform the boxes with *measured link-frame* body poses, not COM
poses or commanded joints. The pinned YAM adapter's ``runtime.link_poses`` uses
``robot.data.body_link_pose_w.torch`` in XYZ + XYZW order; this module follows
that same convention. No Isaac import is needed for offline use.

Provenance: kairosiann/yam-isaac-lab-arena, ``yam_arena/assets/yam/yam_arm.urdf``
and its ``collision/{gripper,tip_left,tip_right}/*.stl``. The inspected checkout
contains 44/45/47 collision meshes respectively; its asset SOURCE_COMMIT is
120c3c81400171174604e503943f8d1ebc891058. Dimensions are read, never hardcoded.
``geometry_provenance()`` records hashes/bounds for the actual installed files.

``robot_collision_boxes`` extends the same construction to every URDF link
with collision geometry, including the base, forearm and wrist. Use that for
support exclusion so another part of either arm cannot act as a shelf. Added
scene objects not present in the URDF need their own envelopes.
The envelopes bound collision geometry; they do not measure contact forces or
establish the presence of a grasp. A missing link or invalid pose raises rather
than returning partial clearance evidence.
"""

from __future__ import annotations

from functools import lru_cache
import hashlib
from pathlib import Path
import struct
from typing import Mapping
import xml.etree.ElementTree as ET

import numpy as np


HAND_LINKS = ("gripper", "tip_left", "tip_right")


def _vector(value, length: int) -> np.ndarray:
    array = np.asarray(value, dtype=float)
    if array.shape != (length,) or not np.isfinite(array).all():
        raise ValueError(f"expected {length} finite numbers")
    return array


def _rpy_matrix(rpy: np.ndarray) -> np.ndarray:
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def _xml_vector(text: str) -> np.ndarray:
    return _vector([float(item) for item in text.split()], 3)


def _stl_vertices(data: bytes) -> np.ndarray:
    # Binary headers may also start with "solid"; exact binary length wins.
    count = struct.unpack_from("<I", data, 80)[0] if len(data) >= 84 else 0
    if count > 0 and len(data) == 84 + count * 50:
        record = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attr", "<u2")])
        points = np.frombuffer(data, dtype=record, count=count, offset=84)["vertices"].reshape(-1, 3)
    else:
        points = np.array([list(map(float, line.split()[1:]))
                           for line in data.decode("ascii").splitlines()
                           if line.strip().startswith("vertex ")])
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 3 or not np.isfinite(points).all():
        raise ValueError("invalid or empty collision STL")
    return points


def _urdf_path(urdf_path: str | Path | None) -> Path:
    if urdf_path is None:
        from yam_arena import YAM_URDF  # This package initializer imports no simulator.
        urdf_path = YAM_URDF
    return Path(urdf_path).expanduser().resolve()


@lru_cache(maxsize=8)
def _geometry(path: Path, include_all_links: bool = False) -> tuple[dict, dict]:
    source = path.read_bytes()
    root = ET.fromstring(source)
    boxes = {}
    sources = hashlib.sha256()
    metadata = {}
    names = tuple(link.attrib["name"] for link in root.findall("link") if link.findall("collision")) if include_all_links else HAND_LINKS
    if not names or not set(HAND_LINKS).issubset(names):
        raise ValueError("robot collision geometry omits required hand links")
    for name in names:
        link = root.find(f"link[@name='{name}']")
        if link is None:
            raise ValueError(f"hand collision link missing from URDF: {name}")
        collisions = link.findall("collision")
        vertices = []
        for collision in collisions:
            geometry = collision.find("geometry")
            if geometry is None or len(geometry) != 1 or geometry[0].tag != "mesh":
                raise ValueError(f"unsupported collision geometry on {name}; refusing an incomplete envelope")
            mesh = geometry[0]
            filename = mesh.attrib["filename"]
            # The pinned assets use relative paths; URI meshes need an explicit resolver.
            if "://" in filename:
                raise ValueError(f"unresolved collision mesh URI on {name}")
            mesh_path = path.parent / filename
            data = mesh_path.read_bytes()
            sources.update(filename.encode("utf-8") + b"\0" + hashlib.sha256(data).digest())
            scale = _xml_vector(mesh.get("scale", "1 1 1"))
            if np.any(scale == 0):
                raise ValueError(f"zero collision mesh scale on {name}")
            points = _stl_vertices(data) * scale
            origin = collision.find("origin")
            if origin is not None:
                rotation = _rpy_matrix(_xml_vector(origin.get("rpy", "0 0 0")))
                points = points @ rotation.T + _xml_vector(origin.get("xyz", "0 0 0"))
            vertices.append(points)
        if not vertices:
            raise ValueError(f"hand link has no collision geometry: {name}")
        all_points = np.concatenate(vertices)
        low, high = all_points.min(axis=0), all_points.max(axis=0)
        center = (low + high) / 2
        # Outward floating-point rounding avoids shaving a source vertex off a bound.
        half_size = np.nextafter(np.maximum(high - center, center - low), np.inf)
        if np.any(half_size <= 0):
            raise ValueError(f"degenerate hand envelope: {name}")
        boxes[name] = (center, half_size)
        metadata[name] = {"collision_meshes": len(collisions), "center": center.tolist(),
                          "half_size": half_size.tolist()}
    source_commit = path.with_name("SOURCE_COMMIT")
    provenance = {"urdf": path.name, "urdf_sha256": hashlib.sha256(source).hexdigest(),
                  "collision_meshes_sha256": sources.hexdigest(),
                  "mesh_hash_format": "SHA256 of ordered filename+NUL+SHA256(mesh bytes), in selected-link/URDF-collision order",
                  "asset_source_commit": source_commit.read_text().strip() if source_commit.exists() else None,
                  "links": metadata, "scope": "robot" if include_all_links else "hand",
                  "envelope": "one link-frame AABB per complete selected collision link"}
    return boxes, provenance


def geometry_provenance(urdf_path: str | Path | None = None, *, include_all_links: bool = False) -> dict:
    """JSON-ready source hashes and measured dimensions, without local file paths."""
    import copy
    return copy.deepcopy(_geometry(_urdf_path(urdf_path), include_all_links)[1])


def _pose_matrix(value) -> np.ndarray:
    pose = np.asarray(value, dtype=float)
    if pose.shape == (7,):
        _vector(pose, 7)
        length = float(np.linalg.norm(pose[3:]))
        if length < 1e-12:
            raise ValueError("zero body quaternion")
        x, y, z, w = pose[3:] / length
        rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                             [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                             [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
        matrix = np.eye(4)
        matrix[:3, :3], matrix[:3, 3] = rotation, pose[:3]
        return matrix
    if pose.shape != (4, 4) or not np.isfinite(pose).all() or not np.allclose(pose[3], [0, 0, 0, 1]):
        raise ValueError("expected a finite link pose (XYZ+XYZW or 4x4)")
    rotation = pose[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6) or not np.isclose(np.linalg.det(rotation), 1., atol=1e-6):
        raise ValueError("link pose must have a proper rotation")
    return pose


def boxes_from_link_poses(link_poses: Mapping[str, object], *, urdf_path: str | Path | None = None,
                          origin=(0., 0., 0.), include_all_links: bool = False) -> list[dict]:
    """Return JSON-ready hand (or complete robot) OBBs using measured link poses.

    Each pose is a 4x4 transform or XYZ+XYZW seven-vector. ``origin`` is
    subtracted from world positions if brick telemetry uses environment-local
    coordinates. Rotations map box-local coordinates to world coordinates.
    ``include_all_links=True`` includes every URDF link with collision geometry.
    """
    local_boxes, _ = _geometry(_urdf_path(urdf_path), include_all_links)
    shift = _vector(origin, 3)
    boxes = []
    for link in local_boxes:
        if link not in link_poses:
            raise ValueError(f"measured hand link pose missing: {link}")
        pose = _pose_matrix(link_poses[link])
        center, half_size = local_boxes[link]
        rotation = pose[:3, :3]
        boxes.append({"link": link, "center": (rotation @ center + pose[:3, 3] - shift).tolist(),
                      "rotation": rotation.tolist(), "half_size": half_size.tolist()})
    return boxes


def _measured_link_poses(robot, env_index: int) -> dict:
    values = robot.data.body_link_pose_w
    values = values.torch if hasattr(values, "torch") else values
    poses = values[env_index]
    if hasattr(poses, "detach"):
        poses = poses.detach()
    if hasattr(poses, "cpu"):
        poses = poses.cpu()
    if hasattr(poses, "numpy"):
        poses = poses.numpy()
    names = list(robot.body_names)
    if len(names) != len(poses) or len(set(names)) != len(names):
        raise ValueError("body names do not uniquely match measured body poses")
    return dict(zip(names, poses))


def hand_collision_boxes(robot, env_index: int = 0, *, urdf_path: str | Path | None = None,
                         origin=(0., 0., 0.)) -> list[dict]:
    """Read actual articulation link poses and return all three hand envelopes.

    ``body_link_pose_w`` is required: falling back to center-of-mass poses would
    transform the hulls incorrectly. For full support exclusion prefer
    ``robot_collision_boxes``; this hand-only helper preserves the earlier API.
    """
    return boxes_from_link_poses(_measured_link_poses(robot, env_index), urdf_path=urdf_path, origin=origin)


def robot_collision_boxes(robot, env_index: int = 0, *, urdf_path: str | Path | None = None,
                          origin=(0., 0., 0.)) -> list[dict]:
    """Read every collision link's measured pose and return full-arm envelopes.

    Store this list under each hand's legacy ``finger_boxes`` field to make
    BrickScorer exclude support by fingers, palm, wrist, forearm and base.
    Missing measured links or unresolved collision geometry fail closed.
    """
    return boxes_from_link_poses(_measured_link_poses(robot, env_index), urdf_path=urdf_path,
                                origin=origin, include_all_links=True)
