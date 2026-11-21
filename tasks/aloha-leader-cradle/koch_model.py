"""The Koch v1.1 follower arm's geometry from its published URDF and meshes (jess-moss/koch-v1-1 via ManiSkill's assets, Apache-2.0), NumPy only.

Every link is expressed at joint zero in the base frame (frames aligned with the base), as mechkit's hinges are: each joint's origin, its axis (each is
one of the base's axes at joint zero, with a sign), the link's meshes, mass, centre of mass and principal inertia. The URDF's base_link and link_1 (the
pan servo, fixed to the base) form the fixed base. The moving jaw is merged into the wrist-roll body at jaw angle 0: the arm's own gripper is not
driven in this task. The meshes are the URDF's visual meshes, converted to STL and clustered to 1.5 mm by koch/convert.py.
"""

from __future__ import annotations

import math
import pathlib
import struct
import xml.etree.ElementTree as ET

import numpy as np

DIR = pathlib.Path(__file__).parent / "koch"
NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
CHILD = {"shoulder_pan": "link_2", "shoulder_lift": "link_3", "elbow_flex": "link_4", "wrist_flex": "link_5", "wrist_roll": "link_6"}  # LeRobot koch_follower names; URDF joint_1 to joint_5


def _triple(s):
    return np.array([float(v) for v in (s or "0 0 0").split()])


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr], [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]])


def _pose(el):
    T = np.eye(4)
    if el is not None:
        T[:3, :3] = _rpy(*_triple(el.get("rpy")))
        T[:3, 3] = _triple(el.get("xyz"))
    return T


def read_stl(path) -> np.ndarray:
    """A binary STL's triangles, shape (n, 3, 3), metres."""
    raw = pathlib.Path(path).read_bytes()
    n = struct.unpack("<I", raw[80:84])[0]
    return np.frombuffer(raw[84:84 + n * 50], np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))["v"].astype(float)


_root = ET.parse(DIR / "follower_arm_v1.1.urdf").getroot()
_J = {j.find("child").get("link"): j for j in _root.findall("joint")}
_L = {l.get("name"): l for l in _root.findall("link")}
T0 = {"base_link": np.eye(4)}  # each link's URDF frame at joint zero, base frame
for _c in ["link_1", "link_2", "link_3", "link_4", "link_5", "link_6", "gripper"]:
    _j = _J[_c]
    T0[_c] = T0[_j.find("parent").get("link")] @ _pose(_j.find("origin"))

ORIGIN, AXIS, SIGN, LIM = {}, {}, {}, {}
for _n in NAMES:
    _c = CHILD[_n]
    ORIGIN[_n] = T0[_c][:3, 3].copy()
    a = T0[_c][:3, :3] @ _triple(_J[_c].find("axis").get("xyz"))
    k = int(np.argmax(np.abs(a)))
    assert abs(a[k]) > 0.999, (_n, a)
    AXIS[_n], SIGN[_n] = k, (1.0 if a[k] > 0 else -1.0)
    lim = _J[_c].find("limit")
    LIM[_n] = (math.degrees(float(lim.get("lower"))), math.degrees(float(lim.get("upper"))))
assert [_J[CHILD[n]].get("name") for n in NAMES] == ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]


def link_tris(link: str):
    """(file, triangles at joint zero in the base frame) for each mesh of a URDF link."""
    out = []
    for v in _L[link].findall("visual"):
        P = T0[link] @ _pose(v.find("origin"))
        f = pathlib.Path(v.find("geometry/mesh").get("filename")).stem + ".stl"
        out.append((f, read_stl(DIR / f) @ P[:3, :3].T + P[:3, 3]))
    return out


def _inertial(link: str):
    i = _L[link].find("inertial")
    P = T0[link] @ _pose(i.find("origin"))
    att = i.find("inertia").attrib
    I = np.array([[float(att.get("i" + a + b, att.get("i" + b + a))) for b in "xyz"] for a in "xyz"])
    return float(i.find("mass").get("value")), P[:3, 3], P[:3, :3] @ I @ P[:3, :3].T


def _merge(parts):
    mass = sum(p[0] for p in parts)
    com = sum(p[0] * p[1] for p in parts) / mass
    I = np.zeros((3, 3))
    for m_, c, Ic in parts:
        d = c - com
        I += Ic + m_ * (d @ d * np.eye(3) - np.outer(d, d))
    return mass, com, I


BASE_TRIS = link_tris("base_link") + link_tris("link_1")
TRIS = {n: link_tris(CHILD[n]) for n in NAMES}
TRIS["wrist_roll"] += link_tris("gripper")
INERTIAL = {n: _inertial(CHILD[n]) for n in NAMES}
# link_6's URDF centre of mass lies 22 cm out along its y axis, outside the link (a URDF error): its mass is kept, its centre put at its meshes' centroid
_m6, _c6, _I6 = _inertial("link_6")
_v6 = np.concatenate([t for _, t in link_tris("link_6")]).reshape(-1, 3)
INERTIAL["wrist_roll"] = _merge([(_m6, _v6.mean(axis=0), _I6), _inertial("gripper")])
MASS_MOVING = sum(v[0] for v in INERTIAL.values())
MASS_BASE = _inertial("base_link")[0] + _inertial("link_1")[0]


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


ROT = [_rx, _ry, _rz]


def fk(q):
    """Each joint's frame (4x4, base frame; the link's frame aligned with the base's at joint zero) for URDF joint angles q."""
    T = np.eye(4)
    prev = np.zeros(3)
    out = {}
    for n, a in zip(NAMES, q):
        A = np.eye(4)
        A[:3, 3] = ORIGIN[n] - prev
        B = np.eye(4)
        B[:3, :3] = ROT[AXIS[n]](SIGN[n] * a)
        T = T @ A @ B
        out[n] = T.copy()
        prev = ORIGIN[n]
    return out


def samples(tri) -> np.ndarray:
    """Surface points of a triangle soup: its vertices, centroids and edge midpoints, deduplicated to 1 mm."""
    pts = [tri.reshape(-1, 3), tri.mean(axis=1), (tri[:, 0] + tri[:, 1]) / 2, (tri[:, 1] + tri[:, 2]) / 2, (tri[:, 2] + tri[:, 0]) / 2]
    return np.unique(np.round(np.concatenate(pts), 3), axis=0)


SURF = {n: samples(np.concatenate([t for _, t in TRIS[n]])) - ORIGIN[n] for n in NAMES}  # link frame (at the joint's origin)
BASE_SURF = samples(np.concatenate([t for _, t in BASE_TRIS]))
