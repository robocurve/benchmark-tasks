"""Fast YAM arm kinematics for planning the demo: the same URDF chain and TCP as ``yam_arena.ik``, with the chain's
fixed transforms cached and an analytic Jacobian, so a full bimanual plan solves in seconds. NumPy only.
``python fastik.py`` checks it against ``yam_arena.ik``."""

from __future__ import annotations

import math

import numpy as np
from yam_arena import YAM_URDF
from yam_arena.constants import ARM_JOINTS, TCP_OFFSET
from yam_arena.urdf import axis_angle_matrix, parse_joints, rpy_matrix

_J = parse_joints(YAM_URDF)
LOWER = np.array([_J[n].lower for n in ARM_JOINTS])
UPPER = np.array([_J[n].upper for n in ARM_JOINTS])


def _chain():
    by_child = {j.child: j for j in _J.values()}
    link, chain = "gripper", []
    while link in by_child:
        j = by_child[link]
        chain.append(j)
        link = j.parent
    chain.reverse()
    out = []
    for j in chain:
        t = np.eye(4)
        t[:3, :3] = rpy_matrix(j.rpy)
        t[:3, 3] = j.xyz
        idx = ARM_JOINTS.index(j.name) if j.name in ARM_JOINTS else None
        out.append((t, j.axis / np.linalg.norm(j.axis), idx))
    return out


_CHAIN = _chain()
_TCP = np.array(TCP_OFFSET)


def fk(q, base_pos=(0.0, 0.0, 0.0), jac: bool = False):
    """(TCP position, rotation) in the world, and with ``jac`` the 6x6 geometric Jacobian [v; w]."""
    t = np.eye(4)
    t[:3, 3] = base_pos
    axes, origins = [], []
    for origin, axis, idx in _CHAIN:
        t = t @ origin
        if idx is not None:
            axes.append(t[:3, :3] @ axis)
            origins.append(t[:3, 3].copy())
            r = np.eye(4)
            r[:3, :3] = axis_angle_matrix(axis, q[idx])
            t = t @ r
    p = t[:3, :3] @ _TCP + t[:3, 3]
    if not jac:
        return p, t[:3, :3]
    J = np.zeros((6, 6))
    for i, (a, o) in enumerate(zip(axes, origins)):
        J[:3, i] = np.cross(a, p - o)
        J[3:, i] = a
    return p, t[:3, :3], J


def _rotvec(r: np.ndarray) -> np.ndarray:
    c = (np.trace(r) - 1) / 2
    ang = math.acos(max(-1.0, min(1.0, c)))
    v = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    s = np.linalg.norm(v)
    if s < 1e-9:
        if ang < 1e-6:
            return np.zeros(3)
        w, vecs = np.linalg.eigh((r + r.T) / 2)  # 180 degrees
        return vecs[:, -1] * ang
    return v / s * ang


def solve(pos, rot, q0, base_pos, iters: int = 100, damping: float = 0.02):
    """Joints reaching TCP ``pos`` and rotation matrix ``rot``: (q, position error m, rotation error rad)."""
    q = np.array(q0, float)
    pos = np.asarray(pos, float)
    for _ in range(iters):
        p, r, J = fk(q, base_pos, jac=True)
        e = np.r_[pos - p, _rotvec(rot @ r.T)]
        if np.linalg.norm(e[:3]) < 1e-5 and np.linalg.norm(e[3:]) < 1e-4:
            break
        dq = J.T @ np.linalg.solve(J @ J.T + damping**2 * np.eye(6), e)
        n = np.abs(dq).max()
        if n > 0.3:
            dq *= 0.3 / n
        q = np.clip(q + dq, LOWER, UPPER)
    p, r = fk(q, base_pos)
    return q, float(np.linalg.norm(pos - p)), float(np.linalg.norm(_rotvec(rot @ r.T)))


if __name__ == "__main__":
    from yam_arena import ik

    rng = np.random.default_rng(0)
    worst = 0.0
    for _ in range(200):
        q = rng.uniform(LOWER, UPPER)
        b = rng.uniform(-0.3, 0.3, 3)
        p1, r1 = fk(q, b)
        p2, r2 = ik.tcp_pose(q, b)
        worst = max(worst, np.abs(p1 - p2).max(), np.abs(r1 - r2).max())
    print("fk vs yam_arena.ik worst difference:", worst)
    q = np.array([0.1, 1.2, 1.1, -0.5, 0.2, 0.3])
    p, r = fk(q)
    q2, pe, re = solve(p, r, [0.0, 1.0, 1.0, -0.6, 0.0, 0.0], (0, 0, 0))
    print("ik roundtrip", pe, re)
